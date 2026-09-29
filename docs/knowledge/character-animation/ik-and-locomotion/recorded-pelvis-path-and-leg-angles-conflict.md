---
title: Recorded pelvis path and recorded leg angles cannot both be kept
description: "On a rig proportioned unlike the recorded subject, imposing Winter's pelvis bob cost 7-10° of thigh and 9-13° of knee at every size tried; migera's walk keeps the leg angles and lets the pelvis ride the planted legs (14 mm bob, not 36). Read before trying to impose a recorded pelvis or COM path on the walk."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - balance
  - verification
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/walk.rs
  - src/character/anim/foot.rs
  - src/character/anim/reference.rs
sources:
  - "Winter 2009, Table A.2(a) greater-trochanter height (printed pp. 301-305)"
  - "test locomotion::tests::a_walking_body_is_highest_over_its_stance_foot"
aliases:
  - pelvis bob
  - vertical bob
  - pelvis leads
  - legs lead
---

# Recorded pelvis path and recorded leg angles cannot both be kept

On `puppet_base`, the recorded leg angles and the recorded pelvis path
disagree, and no compromise keeps both. migera's walk keeps the **leg
angles** and lets the pelvis ride wherever the planted legs hold it. The
bob that results is real but modest: 14 mm, lowest late in single support.
Winter's pelvis centre bobs 36 mm (43 mm scaled to this leg), lowest in
double support.

## Context

Winter's pelvis centre is the mean of his right hip marker and the same
curve half a stride later: `reference::Stride::pelvis_bob`. The single marker
travels 48 mm because it also rides pelvic tilt. The centre travels 36 mm,
highest over each foot (28% and 76% of the stride) and lowest in each double
support (4% and 52%).

Replayed on `puppet_base` with the attitude mapping (see [Replay a recorded
gait by segment attitudes](./replay-a-recorded-gait-by-segment-attitudes.md)),
the planted legs hold the pelvis 14 mm apart, lowest late in single
support. The difference is proportions: Winter's marker thigh is short for
his shank (0.74), and the rig's is not (0.93).

## Decision

**The legs lead.** The pelvis height is the soft maximum of what each
planted foot needs to touch the floor (`foot::support_height`), so no foot
passes through the ground. The foot that disagrees is lifted slightly, which
is what the trailing foot does in pre-swing anyway.

## Alternatives considered

**The pelvis leads.** Impose the recorded bob, then bend each planted leg's
knee and thigh to meet the floor (law-of-cosines leg solve, feet planted in
and out over 5% of stance). The mean level was chosen three ways, and the
size swept. Worst deviation from the recording, with feet planted, on
`puppet_base` (from `the_walk_replays_winters_joint_curves`'s per-sample
comparison):

| Bob size | Thigh | Stance knee | Also |
|---|---|---|---|
| least-squares fit to the legs' own path | 9.7° | 12.9° | the fit shrank the bob to 5 mm: the shapes disagree |
| 0.25× Winter | 9.9° | 12.7° | |
| 0.5× | 7.1° | 8.7° | |
| 0.75× | 7.2° | 9.8° | |
| 1.0×, mean from the cycle average | 6.7° | 11.1° | stance knee locked at the reach limit through 20-44% |
| 1.0×, mean anchored at the bob's peak | 8.9° | 21° | double-support knee 30° against 11 recorded; heel caught the floor in late swing |

Wherever the imposed bob asked for more leg than exists, the foot could not
reach: headless, planted feet floated up to 16 mm and slid 55 mm a stance
through the springs. It lost on every count the walk is judged by.

## Consequences

- Joint angles stay within 2° (thigh) and 3° RMS (knee) of the recording;
  the planted foot slides 5.9 mm a stance headless, all in the weight
  hand-overs, and the ball of the foot at most 1.2 mm per planted run live.
- The bob is smaller than a human's: 14 mm headless at the reference speed
  (15-38 mm live across 0.7-1.6 m/s, springs and foot IK included). The test
  pins 1-6 cm with the body higher at mid-stance than in double support.
- Which foot carries the body is ONE set of weights, shared by the pelvis
  height, root motion and the thigh correction (`foot::bearing`). With
  three different notions — stance timing, "is it touching", and the soft
  maximum — the planted foot slid 41 mm a stance.
- Holding both feet exactly still is possible (a fully converged thigh
  correction reached 0.03 mm a stance) but bent the thigh 4.4° off the
  recording and put the pelvis lowest at mid-stance, backwards for a walk.

## Revisit when

- The rig's thigh-to-shank ratio is closer to the recorded subject's (or a
  second reference stride with standard proportions is available): the two
  paths may then agree.
- An active-ragdoll or balance layer owns the pelvis; it could take the
  recorded bob as a target while physics resolves the legs.

## Related

- [Replay a recorded gait by segment attitudes](./replay-a-recorded-gait-by-segment-attitudes.md) — prerequisite: the angle mapping that exposes this conflict.
- [A walking foot touches the ground at its heel, ball and toe](./walking-foot-rocker-contact-model.md) — deeper: the support rule the pelvis rides on.
- [Winter 11.3.1 — inverted pendulum in steady walking](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.1-inverted-pendulum-in-steady-walking.md) — contrast: the recorded pelvis/COM behaviour this falls short of.
