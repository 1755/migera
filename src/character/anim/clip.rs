//! Animation clips: sparse keyframes of poses, sampled over time.
//!
//! # Why this exists at all, given springs
//!
//! Stages 1 and 2 cover a great deal — a pose plus damped springs plus
//! phase oscillators already produces continuous, non-repeating motion
//! from a single authored frame. That is the point of the rewrite, and it
//! is why this module arrives last rather than first.
//!
//! What it cannot produce is a *sequence*: a footfall pattern, a wind-up
//! and release, anything whose shape over time is the thing being
//! authored. Those want keyframes.
//!
//! # Sparse, and interpolated in rotation space
//!
//! A keyframe holds a whole [`LocalPose`], but poses are themselves sparse
//! — a bone nobody authored stays at rest. Sampling slerps between
//! neighbouring keyframes per bone, so a clip cannot stretch a bone any
//! more than a pose can.
//!
//! # Contact annotation
//!
//! Each keyframe carries which feet are planted. That is not decoration:
//! Stage 3's foot locking needs to know when a foot should stay put, and
//! deriving it from toe velocity at runtime is guesswork that
//! `tools/dump_animation_pose.py --velocities` can do far better offline,
//! against the real source clip.

use serde::{Deserialize, Serialize};

use super::math::quat_ext::neighborhood;
use super::rig::LocalPose;
use crate::character::skeleton::Bone;

/// Which feet are on the ground at a keyframe.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Contacts {
    /// The left foot is planted and should not slide.
    pub left: bool,
    /// The right foot is planted.
    pub right: bool,
}

impl Contacts {
    /// Both feet down — a standing frame.
    pub const BOTH: Self = Self { left: true, right: true };
    /// Neither — mid-air.
    pub const NONE: Self = Self { left: false, right: false };

    /// Whether `foot` is planted. Any bone in a leg chain answers for its
    /// own side, so a caller need not map bones to sides itself.
    pub fn is_planted(self, foot: Bone) -> bool {
        match foot.side() {
            crate::character::skeleton::Side::Left => self.left,
            crate::character::skeleton::Side::Right => self.right,
            crate::character::skeleton::Side::Center => self.left && self.right,
        }
    }
}

/// One authored moment in a clip.
#[derive(Debug, Clone)]
pub struct Keyframe {
    /// When it happens, in seconds from the clip's start.
    pub time: f32,
    /// The pose at that moment.
    pub pose: LocalPose,
    /// Which feet are planted.
    pub contacts: Contacts,
}

/// A sequence of keyframes.
///
/// Keyframes are kept sorted by time, enforced by [`Self::insert`] rather
/// than assumed — an out-of-order keyframe would make sampling silently
/// return the wrong pose, which is far harder to notice than a panic.
#[derive(Debug, Clone, Default)]
pub struct AnimClip {
    keyframes: Vec<Keyframe>,
    /// Whether sampling past the end wraps to the start.
    pub looping: bool,
}

impl AnimClip {
    /// An empty clip.
    pub fn new(looping: bool) -> Self {
        Self { keyframes: Vec::new(), looping }
    }

    /// The keyframes, in time order.
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// How many there are.
    pub fn len(&self) -> usize {
        self.keyframes.len()
    }

    /// Whether the clip has no keyframes.
    pub fn is_empty(&self) -> bool {
        self.keyframes.is_empty()
    }

    /// When the clip ends — the last keyframe's time, or zero.
    pub fn duration(&self) -> f32 {
        self.keyframes.last().map(|key| key.time).unwrap_or(0.0)
    }

    /// Adds a keyframe, keeping the clip sorted.
    ///
    /// A keyframe at an existing time REPLACES it rather than creating a
    /// duplicate: two keyframes at the same instant have no defined order
    /// between them, and "the later edit wins" is the only behaviour an
    /// editor can sensibly offer.
    pub fn insert(&mut self, keyframe: Keyframe) {
        let time = keyframe.time.max(0.0);
        let keyframe = Keyframe { time, ..keyframe };

        match self.index_at(time) {
            Some(index) => self.keyframes[index] = keyframe,
            None => {
                let position = self
                    .keyframes
                    .iter()
                    .position(|existing| existing.time > time)
                    .unwrap_or(self.keyframes.len());
                self.keyframes.insert(position, keyframe);
            }
        }
    }

    /// Removes the keyframe at `index`, if it exists.
    pub fn remove(&mut self, index: usize) -> Option<Keyframe> {
        (index < self.keyframes.len()).then(|| self.keyframes.remove(index))
    }

    /// Moves a keyframe to a new time, re-sorting.
    ///
    /// Returns the index it ended up at, so a UI can keep its selection
    /// pointed at the same keyframe after a drag reorders the list.
    pub fn move_keyframe(&mut self, index: usize, time: f32) -> Option<usize> {
        let keyframe = self.remove(index)?;
        let time = time.max(0.0);
        self.insert(Keyframe { time, ..keyframe });
        self.index_at(time)
    }

    /// The index of a keyframe at exactly `time`.
    fn index_at(&self, time: f32) -> Option<usize> {
        self.keyframes
            .iter()
            .position(|keyframe| (keyframe.time - time).abs() < 1.0e-6)
    }

    /// The pose at `time`.
    ///
    /// Before the first keyframe yields the first pose, after the last
    /// yields the last — or wraps, if [`Self::looping`]. Between them,
    /// each bone slerps, with neighbourhooding applied before the
    /// interpolation so a bone never takes the long way round between two
    /// rotations that are actually close.
    pub fn sample(&self, time: f32) -> Option<LocalPose> {
        let first = self.keyframes.first()?;
        let last = self.keyframes.last()?;

        if self.keyframes.len() == 1 {
            return Some(first.pose);
        }

        let time = self.wrap(time);

        if time <= first.time {
            return Some(first.pose);
        }
        if time >= last.time {
            return Some(last.pose);
        }

        let upper = self
            .keyframes
            .iter()
            .position(|keyframe| keyframe.time >= time)
            .unwrap_or(self.keyframes.len() - 1);
        let lower = upper.saturating_sub(1);

        let (before, after) = (&self.keyframes[lower], &self.keyframes[upper]);
        let span = after.time - before.time;

        // Two keyframes at the same instant cannot be interpolated between;
        // `insert` prevents it, but sampling must not divide by zero if a
        // clip arrives from elsewhere.
        if span <= 1.0e-6 {
            return Some(after.pose);
        }

        Some(blend(&before.pose, &after.pose, (time - before.time) / span))
    }

    /// The contacts at `time`.
    ///
    /// Stepped, not interpolated: a foot is planted or it is not, and a
    /// half-planted foot is not a thing Stage 3 can act on. The value
    /// holds from each keyframe until the next.
    pub fn contacts_at(&self, time: f32) -> Contacts {
        let time = self.wrap(time);

        self.keyframes
            .iter()
            .rev()
            .find(|keyframe| keyframe.time <= time)
            .or_else(|| self.keyframes.first())
            .map(|keyframe| keyframe.contacts)
            .unwrap_or_default()
    }

    /// Folds `time` into the clip's own span when looping.
    fn wrap(&self, time: f32) -> f32 {
        let duration = self.duration();
        if !self.looping || duration <= 0.0 {
            return time;
        }
        time.rem_euclid(duration)
    }
}

/// Interpolates two poses, per bone.
///
/// `t` outside `0..=1` is clamped rather than extrapolated: extrapolating
/// a rotation past a keyframe produces motion nobody authored, which is
/// exactly what a keyframe is supposed to prevent.
pub fn blend(from: &LocalPose, to: &LocalPose, t: f32) -> LocalPose {
    let t = t.clamp(0.0, 1.0);
    let mut blended = LocalPose::REST;

    for &bone in Bone::ALL.iter() {
        let start = from.rotation(bone);
        // Same hemisphere before interpolating, or a bone can swing the
        // long way round between two rotations that are nearly identical.
        let end = neighborhood(start, to.rotation(bone));
        blended.set_rotation(bone, start.slerp(end, t));
    }

    blended.root_translation = from.root_translation.lerp(to.root_translation, t);
    blended
}

/// Where a clip's playhead is.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClipPlayback {
    /// Seconds since the clip started.
    pub time: f32,
    /// Whether it is advancing.
    pub playing: bool,
    /// Playback rate; 1.0 is authored speed.
    pub speed: f32,
}

impl ClipPlayback {
    /// A playhead at the start, stopped.
    pub fn stopped() -> Self {
        Self { time: 0.0, playing: false, speed: 1.0 }
    }

    /// A playhead at the start, running.
    pub fn playing() -> Self {
        Self { playing: true, ..Self::stopped() }
    }

    /// Advances the playhead, holding at the end of a non-looping clip.
    ///
    /// Returns whether the clip finished on this step, so a caller can
    /// react to the end without polling the time against the duration.
    pub fn advance(&mut self, clip: &AnimClip, dt: f32) -> bool {
        if !self.playing || dt <= 0.0 {
            return false;
        }

        self.time += dt * self.speed;

        let duration = clip.duration();
        if duration <= 0.0 {
            return false;
        }

        if clip.looping {
            self.time = self.time.rem_euclid(duration);
            return false;
        }

        if self.time >= duration {
            self.time = duration;
            self.playing = false;
            return true;
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::poses;
    use crate::character::anim::rig::forward_kinematics;
    use bevy::math::Quat;

    fn key(time: f32, pose: LocalPose) -> Keyframe {
        Keyframe { time, pose, contacts: Contacts::BOTH }
    }

    fn two_frame_clip() -> AnimClip {
        let mut clip = AnimClip::new(false);
        clip.insert(key(0.0, poses::rest()));
        clip.insert(key(1.0, poses::wave()));
        clip
    }

    #[test]
    fn an_empty_clip_samples_to_nothing() {
        let clip = AnimClip::new(false);
        assert!(clip.sample(0.0).is_none());
        assert_eq!(clip.duration(), 0.0);
        assert!(clip.is_empty());
    }

    #[test]
    fn keyframes_stay_sorted_however_they_are_added() {
        // Sampling walks the list assuming time order. An out-of-order
        // keyframe would silently return the wrong pose, which is much
        // harder to notice than a panic.
        let mut clip = AnimClip::new(false);
        clip.insert(key(2.0, poses::rest()));
        clip.insert(key(0.5, poses::wave()));
        clip.insert(key(1.0, poses::relaxed_stand()));

        let times: Vec<f32> = clip.keyframes().iter().map(|k| k.time).collect();
        assert_eq!(times, vec![0.5, 1.0, 2.0]);
    }

    #[test]
    fn a_keyframe_at_an_existing_time_replaces_it() {
        // Two keyframes at the same instant have no defined order, so the
        // only sensible behaviour is that the later edit wins.
        let mut clip = AnimClip::new(false);
        clip.insert(key(1.0, poses::rest()));
        clip.insert(key(1.0, poses::wave()));

        assert_eq!(clip.len(), 1);
        assert!(
            clip.sample(1.0)
                .unwrap()
                .rotation(Bone::RightArm)
                .abs_diff_eq(poses::wave().rotation(Bone::RightArm), 1.0e-5),
            "the second insert should have replaced the first",
        );
    }

    #[test]
    fn sampling_before_the_start_or_after_the_end_holds() {
        let clip = two_frame_clip();

        for (time, expected) in [(-5.0, poses::rest()), (99.0, poses::wave())] {
            let sampled = clip.sample(time).unwrap();
            for &bone in Bone::ALL.iter() {
                assert!(
                    sampled.rotation(bone).abs_diff_eq(expected.rotation(bone), 1.0e-5),
                    "at t={time}, {} should hold the nearest keyframe",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn sampling_at_a_keyframe_returns_it_exactly() {
        // A keyframe is the thing being authored; sampling at its own time
        // must not return something merely close to it.
        let clip = two_frame_clip();

        for (time, expected) in [(0.0, poses::rest()), (1.0, poses::wave())] {
            let sampled = clip.sample(time).unwrap();
            for &bone in Bone::ALL.iter() {
                assert!(
                    sampled.rotation(bone).abs_diff_eq(expected.rotation(bone), 1.0e-5),
                    "at its own keyframe time {time}, {} should be exact",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn sampling_between_keyframes_interpolates() {
        let clip = two_frame_clip();
        let midpoint = clip.sample(0.5).unwrap();

        let start = poses::rest().rotation(Bone::RightArm);
        let end = poses::wave().rotation(Bone::RightArm);
        let sampled = midpoint.rotation(Bone::RightArm);

        assert!(
            !sampled.abs_diff_eq(start, 1.0e-3) && !sampled.abs_diff_eq(end, 1.0e-3),
            "a midpoint sample should be between the keyframes, not at either",
        );
        assert!(
            sampled.angle_between(start) > 0.0 && sampled.angle_between(end) > 0.0,
            "and should sit on the arc between them",
        );
    }

    #[test]
    fn a_looping_clip_wraps() {
        let mut clip = two_frame_clip();
        clip.looping = true;

        // One full duration past the start is the start again.
        let wrapped = clip.sample(1.0 + 0.25).unwrap();
        let direct = clip.sample(0.25).unwrap();

        for &bone in Bone::ALL.iter() {
            assert!(
                wrapped.rotation(bone).abs_diff_eq(direct.rotation(bone), 1.0e-5),
                "{} should sample the same at t=1.25 and t=0.25 in a 1 s loop",
                bone.name(),
            );
        }
    }

    #[test]
    fn interpolation_never_stretches_a_bone() {
        // The structural invariant, on the one path that could break it:
        // a blend produces rotations nobody authored, so it is exactly
        // where an incorrect interpolation would show up.
        let clip = two_frame_clip();

        for step in 0..=20 {
            let t = step as f32 / 20.0;
            let pose = clip.sample(t).unwrap();
            let positions = forward_kinematics(&pose);

            for &bone in Bone::ALL.iter() {
                let Some(parent) = bone.parent() else { continue };
                let rest = bone.t_pose_offset().length();
                let posed = (positions[bone] - positions[parent]).length();

                assert!(
                    (posed - rest).abs() < 1.0e-5,
                    "at t={t}, interpolation stretched {} to {posed} m against {rest} m",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn blending_takes_the_short_way_round() {
        // The neighbourhooding hazard, which is this project's single most
        // recurring quaternion bug: `q` and `-q` name the same rotation, so
        // slerping without putting them in the same hemisphere sends a bone
        // the long way round about half the time.
        let mut from = LocalPose::REST;
        let mut to = LocalPose::REST;

        let rotation = Quat::from_axis_angle(bevy::math::Vec3::Y, 0.2);
        from.set_rotation(Bone::LeftArm, rotation);
        // The same rotation, negated — identical as a rotation.
        to.set_rotation(Bone::LeftArm, -rotation);

        let midpoint = blend(&from, &to, 0.5);

        assert!(
            midpoint.rotation(Bone::LeftArm).abs_diff_eq(rotation, 1.0e-4)
                || midpoint.rotation(Bone::LeftArm).abs_diff_eq(-rotation, 1.0e-4),
            "blending a rotation with its own negation should not move the bone, got \
             {:?}",
            midpoint.rotation(Bone::LeftArm),
        );
    }

    #[test]
    fn contacts_step_rather_than_interpolate() {
        // A foot is planted or it is not. Stage 3 cannot act on a
        // half-planted foot, so the value holds from each keyframe until
        // the next rather than fading between them.
        let mut clip = AnimClip::new(false);
        clip.insert(Keyframe {
            time: 0.0,
            pose: poses::rest(),
            contacts: Contacts::BOTH,
        });
        clip.insert(Keyframe {
            time: 1.0,
            pose: poses::rest(),
            contacts: Contacts { left: true, right: false },
        });

        assert_eq!(clip.contacts_at(0.0), Contacts::BOTH);
        assert_eq!(clip.contacts_at(0.99), Contacts::BOTH, "holds until the next key");
        assert_eq!(clip.contacts_at(1.0), Contacts { left: true, right: false });
    }

    #[test]
    fn contacts_answer_for_a_bones_own_side() {
        let contacts = Contacts { left: true, right: false };

        assert!(contacts.is_planted(Bone::LeftFoot));
        assert!(contacts.is_planted(Bone::LeftToeBase));
        assert!(!contacts.is_planted(Bone::RightFoot));
    }

    #[test]
    fn a_playhead_advances_and_stops_at_the_end() {
        let clip = two_frame_clip();
        let mut playback = ClipPlayback::playing();

        assert!(!playback.advance(&clip, 0.5));
        assert!((playback.time - 0.5).abs() < 1.0e-5);

        let finished = playback.advance(&clip, 0.75);
        assert!(finished, "passing the end should report the clip finishing");
        assert!((playback.time - 1.0).abs() < 1.0e-5, "and should hold at the end");
        assert!(!playback.playing, "a non-looping clip stops when it finishes");
    }

    #[test]
    fn a_looping_playhead_never_finishes() {
        let mut clip = two_frame_clip();
        clip.looping = true;

        let mut playback = ClipPlayback::playing();
        for _ in 0..100 {
            assert!(!playback.advance(&clip, 0.1), "a loop does not finish");
            assert!(
                playback.time >= 0.0 && playback.time < clip.duration(),
                "and stays inside its own span, got {}",
                playback.time,
            );
        }
    }

    #[test]
    fn a_stopped_playhead_does_not_move() {
        let clip = two_frame_clip();
        let mut playback = ClipPlayback::stopped();

        playback.advance(&clip, 1.0);
        assert_eq!(playback.time, 0.0);
    }

    #[test]
    fn moving_a_keyframe_reports_where_it_landed() {
        // A UI keeps its selection by index, so a drag that reorders the
        // list has to say where the dragged keyframe went — otherwise the
        // selection silently jumps to a different keyframe.
        let mut clip = AnimClip::new(false);
        clip.insert(key(0.0, poses::rest()));
        clip.insert(key(1.0, poses::wave()));
        clip.insert(key(2.0, poses::relaxed_stand()));

        // Drag the first keyframe past the second.
        let landed = clip.move_keyframe(0, 1.5).expect("should still exist");

        assert_eq!(landed, 1, "it should now sit between the 1.0 and 2.0 keys");
        assert!(
            clip.keyframes()[landed]
                .pose
                .rotation(Bone::RightArm)
                .abs_diff_eq(poses::rest().rotation(Bone::RightArm), 1.0e-5),
            "and should still be the keyframe that was dragged",
        );
    }

    #[test]
    fn a_keyframe_cannot_be_dragged_before_zero() {
        let mut clip = two_frame_clip();
        clip.move_keyframe(1, -3.0);

        assert!(
            clip.keyframes().iter().all(|keyframe| keyframe.time >= 0.0),
            "a clip starts at zero; a negative keyframe time has no meaning",
        );
    }
}
