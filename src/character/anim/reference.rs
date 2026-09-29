//! A measured human walking stride, as smooth periodic curves.
//!
//! The data is Winter's Appendix A walking trial — D. A. Winter,
//! *Biomechanics and Motor Control of Human Movement*, 4th ed., Wiley 2009,
//! Tables A.2(a), A.3(d) and A.4 (printed pp. 301-345). One young adult,
//! 56.7 kg, filmed from the right side at 69.9 frames/s walking at about
//! 1.43 m/s: a 0.987 s stride, 1.414 m long, 61% of it in stance. The
//! knowledge-base note
//! `docs/knowledge/biomechanics-winter/appendices/a-walking-trial-kinematic-kinetic-energy-data.md`
//! records the trial, its events and every sign convention.
//!
//! `assets/anim/reference/winter_walking_stride.csv` is extracted from the
//! book by `tools/extract_winter_stride.py` (the PDF itself is gitignored),
//! and embedded here so the reference cannot drift from what the tests
//! check against.
//!
//! # Why harmonics
//!
//! Winter's own analysis of gait signals (section 2.2.4) finds the power of a
//! walking joint-angle curve in its first seven stride harmonics. Measured on
//! this stride, seven reproduce every sample within 0.23 degrees at the hip,
//! 0.41 at the knee and 0.67 at the ankle, and make the curve periodic: the
//! recorded stride's two heel contacts (frames 28 and 97) differ by up to
//! 1.4 degrees, a seam the fit closes smoothly instead of leaving a step once
//! per stride. The curves are C-infinity, so nothing downstream sees a kink.
//!
//! # Phase
//!
//! Every curve here takes the position in the stride as a fraction in
//! `[0, 1)`, **0 at heel contact** of the leg being described. Toe-off is at
//! [`STANCE_FRACTION`].

use std::sync::LazyLock;

/// The measured stride, one row per film frame.
const CSV: &str = include_str!("../../../assets/anim/reference/winter_walking_stride.csv");

/// The heel contacts that bound one stride: frames 28 and 97.
const HEEL_CONTACT: usize = 28;
const NEXT_HEEL_CONTACT: usize = 97;
/// Toe-off inside that stride.
const TOE_OFF: usize = 70;

/// Frames in one stride, and so samples in one period.
const STRIDE_FRAMES: usize = NEXT_HEEL_CONTACT - HEEL_CONTACT;

/// The fraction of the stride the foot is on the ground: 42 of 69 frames.
pub const STANCE_FRACTION: f32 = (TOE_OFF - HEEL_CONTACT) as f32 / STRIDE_FRAMES as f32;

/// The recorded stride's duration, seconds: 69 frames at 69.9 Hz.
pub const STRIDE_SECONDS: f32 = 0.987;

/// The recorded stride's length, metres: heel X at HCR 28 to HCR 97.
pub const STRIDE_METRES: f32 = 1.414;

/// The walking speed the stride was recorded at, m/s.
pub const SPEED: f32 = STRIDE_METRES / STRIDE_SECONDS;

/// The subject's thigh plus shank, metres — Figure A.1's hip-knee 31.4 cm
/// and knee-ankle 42.5 cm marker distances — the leg length for scaling
/// this stride onto a different body (Froude scaling: geometrically similar
/// walks share `speed² / (g · leg)`). Thigh plus shank to match
/// [`super::gait::leg_length_of`], which measures a rig the same way.
pub const LEG_LENGTH: f32 = 0.739;

/// Harmonics kept per curve. See the module note.
pub const HARMONICS: usize = 7;

/// One periodic curve as a truncated Fourier series in stride phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Periodic {
    mean: f32,
    cos: [f32; HARMONICS],
    sin: [f32; HARMONICS],
}

impl Stride {
    /// The pelvis's vertical bob about its mean, metres, on the recorded
    /// body: the mean of the two hip markers.
    ///
    /// The recording has one marker, on the right hip, and it rides the
    /// pelvis's side-to-side tilt as well as its rise and fall: 48 mm of
    /// travel, peaking once per stride on its own side. The left marker is
    /// the same curve half a stride later, so their mean is the pelvis's
    /// centre — 36 mm, lowest in each double support (4% and 52%) and
    /// highest over each foot (28% and 76%). Only the even harmonics
    /// survive, which is the "twice per stride" of a walking body.
    pub fn pelvis_bob(&self, phase: f32) -> f32 {
        0.5 * (self.pelvis_height.at(phase) + self.pelvis_height.at(phase + 0.5))
            - self.pelvis_height.mean()
    }
}

impl Periodic {
    /// Least-squares fit of `samples`, taken at equal spacing over one
    /// period starting at phase 0. With equal spacing this is exactly the
    /// discrete Fourier transform truncated to [`HARMONICS`].
    fn fit(samples: &[f32]) -> Self {
        let n = samples.len() as f32;
        let mean = samples.iter().sum::<f32>() / n;
        let mut cos = [0.0; HARMONICS];
        let mut sin = [0.0; HARMONICS];
        for h in 0..HARMONICS {
            let k = (h + 1) as f32;
            for (i, &y) in samples.iter().enumerate() {
                let angle = std::f32::consts::TAU * k * i as f32 / n;
                cos[h] += y * angle.cos();
                sin[h] += y * angle.sin();
            }
            cos[h] *= 2.0 / n;
            sin[h] *= 2.0 / n;
        }
        Self { mean, cos, sin }
    }

    /// The curve's value at `phase` (wrapped).
    pub fn at(&self, phase: f32) -> f32 {
        let theta = std::f32::consts::TAU * phase.rem_euclid(1.0);
        let mut value = self.mean;
        for h in 0..HARMONICS {
            let (s, c) = (theta * (h + 1) as f32).sin_cos();
            value += self.cos[h] * c + self.sin[h] * s;
        }
        value
    }

    /// The curve's mean over a period.
    pub fn mean(&self) -> f32 {
        self.mean
    }
}

/// The reference stride's curves, all indexed by stride phase from heel
/// contact.
#[derive(Debug, Clone, PartialEq)]
pub struct Stride {
    /// The thigh's ABSOLUTE angle from vertical, forward (+), radians —
    /// Table A.3(c)'s segment angle less 90 degrees.
    ///
    /// This, not [`Stride::hip`], is what places the leg. The book's hip
    /// angle is the thigh relative to its "1/2 HAT" segment, the line from
    /// the greater trochanter to the base of the rib cage, and that line
    /// pitches through 18 degrees over the stride (Table A.3(d): 78.9-96.8)
    /// — pelvic and lumbar motion plus marker movement, not a trunk that
    /// rocks that far. Replaying the relative angle on a rig whose trunk
    /// holds still transfers all of that pitch onto the leg: measured, it
    /// landed the heel 3 cm ahead of the hips instead of the recording's
    /// 21 cm and tipped the planted foot 15 degrees toe-down.
    pub thigh: Periodic,
    /// Hip flexion (+), thigh relative to the 1/2 HAT, radians. Kept for
    /// comparison with the book's own curves; see [`Stride::thigh`].
    pub hip: Periodic,
    /// Knee flexion (+), radians.
    pub knee: Periodic,
    /// Ankle dorsiflexion (+), plantarflexion (-), radians.
    pub ankle: Periodic,
    /// Greater-trochanter height, metres. Mean included.
    pub pelvis_height: Periodic,
    /// Trunk (half-HAT) centre-of-mass forward speed, m/s. Mean included.
    pub trunk_speed: Periodic,
    /// The foot segment's pitch from flat, heel-up (+) / toe-up (-),
    /// radians: Table A.3(a)'s metatarsal-to-ankle angle less its value at
    /// foot-flat. For checking a walk's foot against, not for driving it.
    pub foot_pitch: Periodic,
}

/// Table A.3(a)'s foot angle when the foot lies flat, degrees: the mean over
/// frames 40-50, mid-stance, where it stays within 144.7-149.6.
pub const FOOT_FLAT_DEGREES: f32 = 147.6;

/// Parsed rows: `(frame, [hip, knee, ankle deg, pelvis m, trunk m/s, thigh,
/// hat, foot deg])`.
fn rows() -> Vec<(usize, [f32; 8])> {
    CSV.lines()
        .skip(1)
        .map(|line| {
            let fields: Vec<&str> = line.split(',').collect();
            let number = |i: usize| -> f32 {
                fields[i].parse().unwrap_or_else(|_| panic!("bad field {i} in {line:?}"))
            };
            let frame = fields[0].parse().expect("frame");
            (
                frame,
                [
                    number(3),
                    number(4),
                    number(5),
                    number(6),
                    number(7),
                    number(8),
                    number(9),
                    number(10),
                ],
            )
        })
        .collect()
}

/// The reference stride, fitted once.
pub static WINTER: LazyLock<Stride> = LazyLock::new(|| {
    let rows = rows();
    let column = |c: usize, offset: f32, scale: f32| -> Vec<f32> {
        (HEEL_CONTACT..NEXT_HEEL_CONTACT)
            .map(|frame| {
                let (f, values) = rows[frame - 1];
                debug_assert_eq!(f, frame);
                (values[c] - offset) * scale
            })
            .collect()
    };
    let radians = std::f32::consts::PI / 180.0;
    Stride {
        thigh: Periodic::fit(&column(5, 90.0, radians)),
        hip: Periodic::fit(&column(0, 0.0, radians)),
        knee: Periodic::fit(&column(1, 0.0, radians)),
        ankle: Periodic::fit(&column(2, 0.0, radians)),
        pelvis_height: Periodic::fit(&column(3, 0.0, 1.0)),
        trunk_speed: Periodic::fit(&column(4, 0.0, 1.0)),
        // Minus: the segment runs toe to ankle, so its angle FALLS as the
        // heel rises.
        foot_pitch: Periodic::fit(&column(7, FOOT_FLAT_DEGREES, -radians)),
    }
});

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The raw samples of one column over the stride, for comparison.
    pub(crate) fn raw(column: usize) -> Vec<f32> {
        let rows = rows();
        (HEEL_CONTACT..NEXT_HEEL_CONTACT).map(|frame| rows[frame - 1].1[column]).collect()
    }

    #[test]
    fn the_embedded_data_is_winters_table() {
        // Rows checked against the rendered pages by the knowledge-base note
        // (Tables A.4, A.2(a)); independent of the extraction script, so a
        // botched re-extraction fails here rather than reshaping the walk.
        let rows = rows();
        assert_eq!(rows.len(), 106);
        let at = |frame: usize| rows[frame - 1].1;
        assert_eq!(at(1)[..3], [-2.4, 46.7, -15.2]); // TOR, p. 341
        assert_eq!(at(28)[..3], [12.8, -0.6, -0.4]); // HCR
        assert_eq!(at(70)[..3], [-2.5, 47.6, -20.1]); // TOR
        assert_eq!(at(77)[..3], [15.3, 66.6, -9.9]); // swing knee peak, p. 344
        assert_eq!(at(28)[3], 0.7959); // hip marker Y at HCR, p. 302
        // The book's hip angle is thigh minus 1/2 HAT, both from Table A.3.
        for frame in [28, 37, 61, 70, 77] {
            let [hip, _, _, _, _, thigh, hat, _] = at(frame);
            assert!((hip - (thigh - hat)).abs() <= 0.15, "frame {frame}");
        }
    }

    #[test]
    fn the_foot_strikes_toe_up_lies_flat_then_lifts_its_heel() {
        // Section 11.3.1: about 20 degrees of toe-up at heel strike, flat
        // through mid-stance, heel high by toe-off.
        let w = &*WINTER;
        let pitch = |p: f32| w.foot_pitch.at(p).to_degrees();
        assert!((-26.0..-18.0).contains(&pitch(0.0)), "heel strike {}", pitch(0.0));
        assert!(pitch(0.25).abs() < 3.0, "mid-stance {}", pitch(0.25));
        assert!(pitch(STANCE_FRACTION) > 45.0, "toe-off {}", pitch(STANCE_FRACTION));
    }

    #[test]
    fn seven_harmonics_reproduce_every_sample() {
        let stride = &*WINTER;
        let n = STRIDE_FRAMES as f32;
        for (column, curve, tolerance_degrees) in
            [(0, &stride.hip, 0.3), (1, &stride.knee, 0.5), (2, &stride.ankle, 0.8)]
        {
            for (i, &sample) in raw(column).iter().enumerate() {
                let fitted = curve.at(i as f32 / n).to_degrees();
                assert!(
                    (fitted - sample).abs() < tolerance_degrees,
                    "column {column} sample {i}: fitted {fitted:.2} against {sample}",
                );
            }
        }
    }

    #[test]
    fn the_stride_has_the_landmarks_of_a_walk() {
        // The features the knowledge-base note lists, read off the fitted
        // curves — the shape a procedural walk is measured against.
        let w = &*WINTER;
        let deg = |c: &Periodic, p: f32| c.at(p).to_degrees();
        let extreme = |c: &Periodic, from: f32, to: f32, max: bool| {
            (0..=200)
                .map(|i| from + (to - from) * i as f32 / 200.0)
                .map(|p| (p, deg(c, p)))
                .fold((0.0, if max { f32::MIN } else { f32::MAX }), |best, (p, v)| {
                    if (max && v > best.1) || (!max && v < best.1) { (p, v) } else { best }
                })
        };

        // Knee: straight at contact, a loading dip, a swing peak near 70%.
        assert!(deg(&w.knee, 0.0).abs() < 3.0);
        let (_, loading) = extreme(&w.knee, 0.05, 0.25, true);
        assert!((14.0..19.0).contains(&loading), "loading knee {loading}");
        let (at, peak) = extreme(&w.knee, 0.6, 0.85, true);
        assert!((64.0..68.0).contains(&peak) && (0.66..0.75).contains(&at));

        // Hip: extends to about -6 by 50%, flexes to about 22 late in swing.
        let (_, extension) = extreme(&w.hip, 0.3, 0.7, false);
        assert!((-7.5..-5.0).contains(&extension), "hip extension {extension}");
        let (_, flexion) = extreme(&w.hip, 0.7, 1.0, true);
        assert!((21.0..24.0).contains(&flexion), "hip flexion {flexion}");

        // Ankle: pushes off plantarflexed to about -20 just after toe-off.
        let (at, push_off) = extreme(&w.ankle, 0.5, 0.8, false);
        assert!((-22.0..-18.0).contains(&push_off) && at > STANCE_FRACTION - 0.02);

        // Pelvis: highest in mid-stance, lowest near contact and toe-off.
        let (high, _) = extreme(&w.pelvis_height, 0.0, 0.5, true);
        assert!((0.15..0.35).contains(&high), "pelvis peak at {high}");
    }

    #[test]
    fn the_trunk_is_fastest_in_double_support_and_slowest_in_mid_stance() {
        // Section 6.2.1 / 11.3.1: kinetic and potential energy exchange like
        // an inverted pendulum, so the trunk slows as it rises over the stance
        // foot. Mean-relative swing about +-17%.
        let w = &*WINTER;
        let mean = w.trunk_speed.mean();
        let mid_stance = w.trunk_speed.at(0.22) / mean;
        let double_support = w.trunk_speed.at(0.52) / mean;
        assert!(mid_stance < 0.98 && double_support > 1.1, "{mid_stance} {double_support}");
        assert!((1.35..1.5).contains(&SPEED));
    }
}
