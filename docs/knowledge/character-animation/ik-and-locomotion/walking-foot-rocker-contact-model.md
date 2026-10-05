---
title: A walking foot touches the ground at its heel, ball and toe
description: "migera's walk plants a foot at the lowest of three contact points (heel, ball, toe tip), not the ankle, and uses ONE set of support weights for pelvis height, root motion and drift; the ankle climbs 12 cm by toe-off. Read before changing foot contact, pelvis height or root motion."
type: concept
status: current
tags:
  - locomotion
  - biomechanics
  - ik
  - correctness
updated: 2026-10-05
verified: 2026-10-05
code:
  - src/character/anim/foot.rs
  - src/character/anim/walk.rs
  - src/character/anim/locomotion.rs
  - src/character/anim/gait.rs
sources:
  - "Winter 2009, Tables A.2(c)/(d) and A.3(a); section 11.3.1"
  - "tests foot::tests, locomotion::tests::the_cancellation_holds_across_a_whole_cycle_including_contact_switches"
aliases:
  - foot rocker
  - heel rocker
  - Sole
  - support_height
  - bearing
  - contact point
---

# A walking foot touches the ground at its heel, ball and toe

A walking foot is planted at its **loaded contact**, not its ankle. Winter's
foot strikes the ground 25° toe-up on its heel, lies flat, rises onto the
ball, and in pre-swing rolls onto the toes: his fifth-metatarsal marker
climbs from 3.4 to 9.6 cm while the toe marker stays down (Tables A.2(c)/(d),
frames 63-70). The ankle climbs 12 cm by toe-off. A gait that holds the
ankle still cannot roll, and a rolling foot then has to slide.

## The model

`foot::Sole` carries the heel and ball in the ankle bone's frame and the tip
in the toe bone's, so the tip goes with the toes as they bend (a run's at
push-off; see [running](./running-replays-measured-strides-at-their-froude-number.md));
a pose that leaves the toes alone reads exactly the contacts of a rigid foot.
All three are measured on the rig's bind pose standing on `y = 0`, or on the foot's lowest
joint if one dips below it (the synthetic rig's toe does). The runtime foot
IK plants the same sole (`plugin::toe_contact_offset`), so a standing foot
rests at the asset's own bind height:

| Point | Where | Source |
|---|---|---|
| heel | under the ankle, `HEEL_BEHIND_ANKLE = 0.61` of the ankle-to-ball distance behind it | Winter at foot-flat: heel 0.060 m behind the malleolus, fifth metatarsal 0.094-0.099 m ahead |
| ball | under the toe joint (`ToeBase`) | |
| tip | under the end of the toes (`RigGeometry::toe_end_offset`) | needed for pre-swing: without it the late-stance leg had nothing to stand on and the pelvis dropped 7 cm |

`foot::shares` splits a foot's load over the three by a soft minimum of
height (`HANDOVER_HEIGHT`, 4 mm), and `foot::contact_moved` takes the
share-weighted **displacement** of the points. It does not take the
displacement of a share-weighted point, which would slide along the sole as
the load hands over: a centre of pressure moves, the body does not.

## One set of support weights

Which foot carries the body is decided once, and every consumer uses it:

- **Pelvis height** is `foot::support_height`: a load-weighted soft
  **maximum** of what each planted foot needs to touch the floor
  (`SUPPORT_HANDOVER`, 1 cm). It is a maximum because the body cannot sit so
  low that a planted foot passes through the floor. An average pushed one
  foot under and lifted the other in every double support, and dropped the
  body 5 cm at each heel strike.
- **Root motion** weights each planted foot's contact displacement by its
  share of that soft maximum (`foot::bearing`).
- **The thigh drift correction** (`walk::WalkCycle`) counts a foot's slide
  by the same share.

Three different notions of "planted" were tried together: stance timing, a
"touching" height test, and the soft maximum. At every heel strike they
disagreed. The pelvis rode the trailing toe while root motion followed the
new heel, and the planted foot slid 41 mm a stance (3.9 mm in one step).
Unified, it slides 5.9 mm, all in the hand-overs. At 4 mm the support width
made the slow walk's root velocity step 0.15 m/s per heel strike, so it is
1 cm.

## Relevance to migera

- A foot's contact, not `Bone::LeftFoot`, is what root motion cancels and
  what a planted-foot test must measure. `locomotion::tests::contact_moved`
  is the helper.
- The ankle legitimately moves while the foot is planted. In live BRP
  checks, measure the ball (`ball_l`, `ball_r` on `puppet_base`): it slides
  at most 1.2 mm per planted run at 0.7-1.6 m/s.

## Related

- [Replay a recorded gait by segment attitudes](./replay-a-recorded-gait-by-segment-attitudes.md) — prerequisite: how the foot's recorded attitude is driven.
- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — applies: why the tip follows the toe bone (a run's toes bend at push-off).
- [A toe tip pivots on the floor and needs its own lock](./a-toe-tip-pivots-on-the-floor-and-needs-its-own-lock.md) — applies: the walk's rigid tip under the floor in pre-swing, and how the foot IK now holds it.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — applies: how the body is moved over these contacts.
- [Recorded pelvis path and recorded leg angles cannot both be kept](./recorded-pelvis-path-and-leg-angles-conflict.md) — deeper: why the pelvis rides the legs.
- [Foot IK on uneven ground has two feedback loops](./foot-ik-feedback-loops.md) — contrast: the runtime foot IK's own toe-joint contact offset.
- [The live rig geometry must match the rendered rig](../rig-and-retargeting/live-rig-geometry-must-match-the-rendered-rig.md) — applies: the IK's contact offset moved onto this sole, and why it had drifted 15 mm off it.
- [Winter 11.3.1 — inverted pendulum in steady walking](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.1-inverted-pendulum-in-steady-walking.md) — prerequisite: the heel-strike pitch and toe-supported trailing foot this models.
