---
title: Replay a recorded gait by segment attitudes, not by the book's joint angles
description: "A recorded stride (Winter Appendix A) replays on a rig only with the thigh and foot driven by ABSOLUTE attitudes (thigh from vertical, foot pitch) and geometric zeros; the book's hip and ankle angles carry trunk-marker pitch and fibula-line offsets. Read before driving a rig from recorded joint angles."
type: lesson
status: current
tags:
  - locomotion
  - biomechanics
  - retargeting
  - correctness
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/walk.rs
  - src/character/anim/gait.rs
  - src/character/anim/reference.rs
  - assets/anim/reference/winter_walking_stride.csv
  - tools/extract_winter_stride.py
sources:
  - "Winter 2009, Appendix A, Tables A.3(a)-(d) and A.4 (printed pp. 321-345, PDF 334-358)"
  - "test locomotion::tests::the_walk_replays_winters_joint_curves"
aliases:
  - measured walk
  - Winter reference gait
  - LegCurves::Measured
---

# Replay a recorded gait by segment attitudes, not by the book's joint angles

A recorded stride's joint angles do not place a rig's legs where the
recording's were. Drive the **thigh by its angle from vertical** and the
**foot by its pitch on the ground** (Winter Tables A.3(c) and A.3(a)), take
the **knee** from Table A.4, and let the ankle be whatever joins the shank to
the foot. Measure thigh and knee from **geometric** zeros (vertical, a
straight hip-knee-ankle line), not from the rig's bind pose.

## What happened

`src/character/anim/walk.rs` drives `puppet_base` from Winter's Appendix A
walk. Four wrong mappings were each measured before the right one:

| Mapping | Symptom on `puppet_base` | Cause |
|---|---|---|
| Hip = Table A.4 hip angle (thigh relative to "1/2 HAT") | Heel landed 3 cm ahead of the hips (recording: 21 cm); planted foot tipped 15° toe-down; body nearly stalled at mid-stance | The HAT line (greater trochanter to rib cage) pitches through 18° over the stride (Table A.3(d): 78.9-96.8°). That is pelvic and lumbar motion plus marker movement, and a rig whose trunk holds still puts all of it on the leg. Hip = thigh − HAT exactly (12.8 = 109.6 − 96.8 at heel contact). |
| Ankle = Table A.4 ankle angle | Planted foot 5.6° onto its ball through mid-stance; heel strike 12° toe-up instead of 25°; pelvis had to drop 7 cm in late stance | The book's ankle is measured against the fibula-head line, not the knee-to-ankle line a rig bends about. |
| Thigh and knee zeroed on the bind pose | Late-stance leg short; pelvis dropped 6 cm, twice the recording's | `puppet_base`'s T-pose knee sits 6.4° off its hip-to-ankle line, so a recorded straight knee became a 6.4° bend. Winter's knee-vs-segment offset is noise, not bias: (thigh − leg segment) − A.4 knee has mean −0.7°, SD 2.8° over the stride. |
| Foot scaled by `sqrt(amplitude)` while the thigh scaled by `amplitude` | At 0.7 m/s the body's speed spiked to 1.8× its mean at each heel strike | The foot's roll carries the body; scaled more gently than the thigh, it dominates a short stride. |

With the attitude mapping, `the_walk_replays_winters_joint_curves` holds the
thigh within 2° of the recording at every sample (the residual is the
double-support correction, below), the knee within 3° RMS (the residual is
the 6.9° knee floor near contact), and the planted foot's attitude within 1°.

## Why it matters

Joint angles in a biomechanics table are defined by **marker lines**, not
joint centres, and relative to segments (the trunk, the fibula) that a game
rig does not animate the same way. Replaying them relative to the rig's
bones moves the offsets into the leg. Absolute segment attitudes are what
the ground sees: where the foot lands, and how it meets the floor.

## How to apply

- Drive the proximal segment by its absolute attitude (thigh from vertical)
  and the end effector by its absolute attitude (foot pitch). Take only the
  middle joint (knee) from the relative table.
- Zero geometric angles geometrically: verify the rig's bind, don't assume
  it is straight. `gait::sagittal_angles` documents which zeros are which.
- The rig's proportions still differ (Winter's marker thigh-to-shank ratio is
  0.74; `puppet_base`'s 0.93, near his own Figure 4.1's ~1.0), so two planted
  feet disagree slightly through double support. `walk::WalkCycle` corrects
  each thigh by its integrated drift while the foot carries the body, 1.4° at
  most; see [Recorded pelvis path and recorded leg angles cannot both be
  kept](./recorded-pelvis-path-and-leg-angles-conflict.md) for the larger
  conflict this leaves.

## Evidence

- Commit landing `reference.rs`, `walk.rs`, `foot.rs` (2026-09-29).
- Tests: `reference::tests::the_embedded_data_is_winters_table` (hip = thigh − HAT
  on five frames), `locomotion::tests::the_walk_replays_winters_joint_curves`.

## Related

- [Winter Appendix A](../../biomechanics-winter/appendices/a-walking-trial-kinematic-kinetic-energy-data.md) — prerequisite: the trial, its events and sign conventions.
- [Recorded pelvis path and recorded leg angles cannot both be kept](./recorded-pelvis-path-and-leg-angles-conflict.md) — deeper: the trade-off this mapping forces on a differently proportioned rig.
- [A walking foot touches the ground at its heel, ball and toe](./walking-foot-rocker-contact-model.md) — applies: the contact model the replayed foot attitude rolls over.
- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — applies: the same zeros, on Fukuchi's recorded runs.
- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — contrast: the bind is a straight-legged T-pose, which is why it is not the geometric zero here.
