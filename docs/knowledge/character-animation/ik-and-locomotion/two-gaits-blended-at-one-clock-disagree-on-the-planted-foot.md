---
title: Two gaits blended at one clock disagree on where the planted foot is
description: "At one clock phase the run is further into its stance than the walk, its planted foot ~160 mm further back under the hips; blended over the change, that foot moved ~10 mm a frame and braked the body to 0.8 m/s. Pose the gait coming in at the matching phase. Read before blending two cyclic gaits."
type: lesson
status: current
tags:
  - locomotion
  - correctness
  - verification
updated: 2026-10-05
verified: 2026-10-05
code:
  - src/character/anim/run.rs
  - src/character/anim/walker.rs
sources:
  - "test run::tests::a_change_between_walk_and_run_keeps_the_planted_foots_pace (1.16 of the body's pace at one clock)"
  - "test run::tests::a_walk_changes_its_speed_only_with_one_foot_down (18.3 mm a frame eased through double support)"
  - "live: character_gallery --anim-speed-schedule 1:3.0,4:1.0,7:0 and the two run schedules, every frame at 60 Hz over BRP"
aliases:
  - run::Matched
  - Matched::feet
  - Gaits::phases
  - walk-run change
  - gait change slides
  - run::paced both_down
  - walk speed change slides
---

# Two gaits blended at one clock disagree on where the planted foot is

A walk and a run on one clock are at different points of their stances at
the same phase: the run's stance is shorter, so it is further through it.
Blending them by a changing weight then moves the planted foot under the
hips by the weight's rate times the gap between them, a motion neither
gait makes. Pose the gait coming in at the phase where its planted foot is
where the outgoing gait has it, and hand the clock over at the end.

## What happened

- **The change step** (`run::Gaits`) blended walk and run poses over one
  step of single support, both at the shared clock.
- **At 1.88 m/s** the walk's planted right toe sat ~160 mm further ahead of
  the hips than the run's at every phase of that stretch (310 against
  153 mm at its start).
- **The weight moved over 16 frames,** so the blended toe moved ~10 mm a
  frame relative to the hips on top of its own motion.
- **Root motion read it:** the walk moves the body by the planted foot's
  rendered displacement, so changing from a run the body braked from 2.0 to
  0.8 m/s for half a step. The run's velocity root motion would instead
  have slid the foot. Live, the foot then jumped 37 mm in a frame.
- **And the speed snapped:** coming down, a walk was handed its asked speed
  at once (`run::paced`), and the walk's pose depends on its speed: the
  hips dropped 37 mm in the frame the change ended. Speeding up, a walk at
  1.2 m/s jumped to the changeover speed: 23 mm.

## How it is fixed

- **Matched phases** (`run::Matched::feet`): once a change is asked, the
  planted toe's height ahead of the hips is sampled over each gait's stance
  (13 points each). Each walk phase is paired with the run phase putting
  the toe as far ahead, linearly between samples.
  - A straight line through one match drifted 90 mm by the end of the
    walk's stance, the toe at 0.68 of the body's pace there.
- **Through the change** the outgoing gait keeps the clock, its stance
  share and its stride (the cadence), and the incoming one is posed at the
  matched phase (`Gaits::phases`). Root motion reads the planted foot
  throughout. At the end the clock moves onto the incoming gait's phase
  (by ~0.1-0.16 of a cycle), and the pose does not jump.
- **The window** keeps both gaits in single support on that foot.
  - From the run it opens no earlier than a third of the run's stance:
    started just after a landing, the sprung leg still landing behind its
    target braked the body to 0.3 m/s.
- **Speeds are eased** once walking, both ways, at the run's 2 and 3 m/s²;
  a start and a stop keep theirs.
- **And held with both feet down** (`run::paced`'s `both_down`, from the
  gait's clock). Eased through double support, a slowing walk's stance
  share grew under the trailing foot as it rolled onto its toe: the first
  walking toe-off after a run slid its toe tip 10-18 mm a frame, and a walk
  slowing from 1.2 to 0.6 m/s slid one 28 mm. Its ball came up to 35 mm
  nearer the tip's pin than the toe is long, so the rigid toe could reach
  the pin only along the floor.

## Why it matters

- **A blend is only foot-preserving if both inputs agree on the foot.**
  The single-support rule (see
  [fade a gait only through single support](./fade-a-gait-only-through-single-support.md))
  stops a blend scaling the distance between two planted feet. It does not
  stop it moving one planted foot that the two poses put in different
  places.
- **Any pose parameter that changes in a frame moves the body.** A gait
  whose pose depends on its speed needs its speed eased like any other
  input.
- **Eased is not enough with both feet down.** The single-support rule
  holds for every pose parameter, not only a blend weight: with two feet
  down, root motion keeps one and the other slides.

## How to apply

- **Check the foot first:** before blending two cyclic motions, compare
  the planted contact's position under the hips in each across the blend
  window. Any gap times the weight's rate is a slide or a speed change.
- **Match by the contact, not by the clock:** pose the incoming motion at
  the phase where its contact matches, and move the clock when the blend
  ends.
- **Test the pace:** assert the planted contact keeps the body's pace over
  the whole blend, not only frame by frame. The gaits' own steps vary
  0.7-1.35 of it per frame, which hid a 16 % error.

## Evidence

- Unit (`a_change_between_walk_and_run_keeps_the_planted_foots_pace`,
  `puppet_base`, 1.88 m/s): the planted toe keeps the body's pace within
  10 % over the change both ways (0.77-1.29 frame by frame). At one clock
  it kept 1.16.
- Live, run to walk: travel holds at 28-40 mm a frame through the change
  (it dropped to 13). The pelvis has no vertical step over the schedules,
  its worst acceleration 50-63 m/s² (137-145 before). The foot's 37 mm jump
  is gone.
- Speed held with both feet down: unit
  (`a_walk_changes_its_speed_only_with_one_foot_down`, 1.2 to 0.6 m/s from
  a footfall), the trailing foot moves 0.62 mm a frame on the floor, as in a
  steady walk; eased through double support it slid 1.2, 3.0, 7.9, then
  18.3 mm. Live, the worst toe-tip slide at a toe-off: run to a 1.0 m/s
  walk 2.4 mm (18.5); the mixed schedule's first walking toe-off back
  within the run's own 5 mm (10.5); a walk 1.2 → 0.6 → 1.6 m/s 2.3 mm (28).
  A/B on the same schedules with the hold switched off.

## Related

- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — prerequisite: why the change runs in single support; this is the second condition.
- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — context: the walk-run change this fixes.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — context: why a moving planted foot became a speed change.
