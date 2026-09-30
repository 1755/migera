---
title: IK and locomotion
description: "Leg/arm IK, foot grounding, stance and the measured walk: IK reach traps, the bind-pose singularity, foot-IK feedback loops, replaying Winter's stride, foot contact, root motion, and archived muscle-era bugs. Read before changing legik.rs, stance.rs, foot IK, gait.rs, walk.rs or locomotion.rs."
type: index
status: current
tags:
  - ik
  - locomotion
  - correctness
updated: 2026-09-29
---

# IK and locomotion

How migera's legs and arms reach their targets and stay on the ground. Most
notes are about reach budgets and reference points: which joint the solve
pivots at, how much slack the rig really has, and where the ground is
sampled from.

## Start here

Read [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md),
then [Rig authored at critical extension](./rig-authored-at-critical-extension.md),
before any leg IK work. They explain why the stance exists and why reach
margins are deadbands.

| Note | What it establishes | Read when |
|---|---|---|
| [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) | A T-pose bind is a straight-knee singularity on every rig; fix with an authored stance, not bone lengths | before foot IK or stance work, or if tempted to edit bone lengths |
| [Rig authored at critical extension](./rig-authored-at-critical-extension.md) | Permanent 0.0108 m flat-ground shortfall; `reach_margin` is a 0.02 deadband | before adding a "limb is straining" check or pelvis adaptation |
| [Foot IK on uneven ground has two feedback loops](./foot-ik-feedback-loops.md) | Sample ground from the animated pose, keep corrections out of spring state, solve on the real rig, keep the toe offset | before changing foot IK or grounding in `plugin.rs`/`legik.rs` |
| [Two-bone IK pivots at the upper joint, not the root](./two-bone-ik-pivots-at-upper-not-root.md) | Measuring reach from the root socket lands short by the socket offset (0.14 m) | before writing an IK solver or reach test, or when a solve lands short by a constant |
| [Replay a recorded gait by segment attitudes](./replay-a-recorded-gait-by-segment-attitudes.md) | Drive the thigh from vertical and the foot by its pitch, zeros geometric; the book's hip and ankle angles carry trunk-marker pitch and fibula-line offsets | before driving a rig from recorded joint angles, or changing `walk.rs`/`reference.rs` |
| [A walking foot touches the ground at its heel, ball and toe](./walking-foot-rocker-contact-model.md) | Three contact points; one set of support weights for pelvis height, root motion and drift correction | before changing foot contact, pelvis height or root motion |
| [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) | Velocity×dt slid feet 2 mm a frame and the target pose 39 mm a stance through springs; use the rendered contact's displacement | before changing how a gait moves its character |
| [A lagging pelvis rotation slides planted feet](./a-lagging-pelvis-rotation-slides-planted-feet.md) | Hips springs with the legs (0.015 s): a 4° roll at 0.16 s moved the rendered toe 4.2 cm, hidden by the foot lock until a first step | before tuning springs or posing the pelvis over planted feet |
| [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) | Locks work in the pose's frame; without `Turn::travel` a locked foot rode along with root motion (~14 cm first-step slide) and never locked mid-walk | before touching footlock.rs or anything that moves the character entity |
| [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) | A changing blend in double support slips a planted foot (12.5 mm/frame); root motion must include the hips' `root_translation`; a first swing must be lifted; a last swing is set down by the foot IK on the rendered foot | before changing transition.rs or blending any gait |
| [A gait-timed motion cannot ride a weighty spring](./a-gait-timed-motion-cannot-ride-a-weighty-spring.md) | A critical spring passes a stride-rate motion at λ²/(λ²+ω²), lagging 2·atan(ω/λ): 0.36 and 107° at the spine's 0.16 s. Arms, trunk counter-roll and chest twist each broke on it | before giving a gait-timed bone a slow spring, or judging a timed motion from its target |
| [The walking pelvis turns ±4° with the stepping leg, and the chest against it](./walking-pelvic-turn-and-chest-counter-twist.md) | Winter gives the timing, not the size (the hip angle includes femoral rotation); ±4° is Perry's. The chest twist is retimed to counter it at the heel contacts | before changing the walk's pelvic yaw or chest twist |
| [The walking pelvis's roll is integrated from Winter's hip abductor power](./walking-pelvic-obliquity-from-hip-abductor-power.md) | Frontal hip power over moment, integrated: swing side drops 3.9° at 17 %, lifted back; rolled about the stance hip, trunk upright. Replaced a mistimed authored roll that made the root weave 10 cm | before changing the walk's pelvis roll or the Spine spring |
| [The walk's step width and sideways sway come from the inverted pendulum](./walk-step-width-and-sideways-sway.md) | Feet were 22.9 cm apart with no sideways pelvis motion; now 13 cm (narrowest keeping the COM medial of the stance foot) and a ±1.5–2.3 cm pendulum sway, feet held exactly | before changing step width, walking pelvis sway or sway_over_feet |
| [The walk's pelvis rides one sinusoid per step](./walk-pelvis-rides-one-sinusoid-per-step.md) | The raw support-height path dropped the body onto each leg (44 m/s² live); one sinusoid per step fitted under it: 1.3 m/s², feet press ≤15 mm | before changing walk.rs's pelvis height or judging the bob |
| [Recorded pelvis path and recorded leg angles cannot both be kept](./recorded-pelvis-path-and-leg-angles-conflict.md) | Imposing Winter's pelvis bob cost 7-10° of leg angle at every size; the legs lead, a 14 mm bob | before imposing a pelvis or COM path on the walk |

## See also

- [Knee axis positive swings forward](../rig-and-retargeting/knee-axis-positive-swings-forward.md) — the knee-direction bug and how leg IK now picks its branch.
- [Synthetic rig's leg segments are shifted a joint](../rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md) — verify leg shape on the real rig only.
- [Unsigned measurements cannot see direction](../../engineering-practice/testing/unsigned-measurements-cannot-see-direction.md) — why leg tests must be signed.
- [Winter Appendix A — walking-trial data](../../biomechanics-winter/appendices/a-walking-trial-kinematic-kinetic-energy-data.md) — a measured reference stride (joint angles, GRF, powers) to validate the procedural walk against.
- [Winter Ch. 11 — Movement synergies](../../biomechanics-winter/ch11-biomechanical-movement-synergies/INDEX.md) — support moment, COM path over the stance foot, gait initiation and termination for start/stop transitions.

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [Walk-cycle IK and ground-lock bugs in the muscle solver](./walk-cycle-ik-and-ground-lock-bugs.md) | Four bugs in the deleted muscle walk (reach undercount, stride, knee sign, ground lock) caused 7.3x overstretch | for history, or when a leg overstretches and you want the prior failure modes |
