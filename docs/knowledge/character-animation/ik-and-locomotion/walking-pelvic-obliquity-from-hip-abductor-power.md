---
title: The walking pelvis's roll is integrated from Winter's hip abductor power
description: "Winter gives no pelvic angles, but frontal hip power over moment is angular velocity: integrated, the swing side drops 3.9° at 17 % of stride, then lifts back. Rolled about the stance hip, trunk upright. Replaced a mistimed authored roll that weaved the root 10 cm. Read before changing the walk's pelvis roll."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - springs
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/phase.rs
  - src/character/anim/stance.rs
  - src/character/anim/dho.rs
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §7.4.5, Figs 7.4–7.5, pp. 196–197 (PDF pp. 209–210)"
  - "test phase::tests::the_pelvis_drops_on_the_swing_side_then_is_lifted_back"
  - "test locomotion::tests::a_walking_pelvis_drops_on_the_swing_side_with_the_trunk_upright"
  - "test locomotion::tests::the_rendered_trunk_stays_upright_while_the_pelvis_rolls"
aliases:
  - pelvic obliquity
  - pelvic drop
  - Trendelenburg
  - pelvic_obliquity_at
  - move_pelvis_over_feet
---

# The walking pelvis's roll is integrated from Winter's hip abductor power

Winter's 3D walking data (§7.4.5) gives frontal hip **moments and powers**,
not pelvic angles. Power over moment is the frontal hip's angular velocity,
and integrated through stance it gives the pelvis's roll over the stance
thigh. `phase::pelvic_obliquity_at` is that curve.

## Context

The locomotion layer used to roll the pelvis with an authored oscillator
(0.05 rad about the Hips' local Z). Measured on `puppet_base`, it:

- lowered the **stance** side through late single support, the reverse of
  Winter's pattern;
- swung each planted sole 45–48 mm in the target pose, because both legs
  hang from the rolled Hips. The foot IK absorbed it.
- made the body's path weave ~100 mm sideways live. Root motion follows the
  rendered contacts, and the roll moved them against the hips. Replacing it
  took the weave to 14 mm.

## Decision

**The curve.** Read off Figs 7.4–7.5 by eye (intersubject means, ~1 s
stride), taking ω = P/M wherever the abductor moment exceeds 0.25 N·m/kg:

| % of stride | Winter | Swing side of the pelvis |
|---|---|---|
| 0–17 | H1-F absorption, −1.0 W/kg at 11 % | drops, to 3.9° at 17 % |
| 17–30 | H2-F generation, +0.3 W/kg | lifted ~1.5° |
| 30–43 | ~0 | held |
| 43–57 | H3-F generation, +0.55 W/kg | lifted to 0.74° low at its heel contact |

The other leg's stance mirrors it half a stride later, so the curve has
only odd harmonics. Harmonics 1, 3 and 5 fit it within 0.37°. The
amplitude is ±20 % (read by eye); the timing is firmer. It matches the
gait-lab range of ±4–5° without being tuned to it.

**The pivot.** The pelvis rolls about the load-weighted hip socket
(`stance::move_pelvis_over_feet`), as a real one drops about the stance
hip. Rolled about its own centre, the stance socket would rise, and this rig
stands at critical extension, so the planted foot would float. The Spine
takes the counter-roll, so the trunk stays upright. Both legs are re-solved
once onto their old ankles (`keep_ankle`, told the turn the leg already
rode), together with the pendulum sway, so the feet stay exact.

**The spring.** The first spine bone (`Bone::Spine`) moved to the hips'
0.015 s spring. On the spine's 0.16 s it lagged the hips' roll, and the
rendered trunk rolled with the pelvis: 8.1° peak to peak live, 8.5°
headless. With it fast: 1.5° live. `Spine1` and `Spine2` keep 0.16 s, so
the trunk still has weight.

## Alternatives considered

- **Keep the oscillator and retime it.** It would still swing the planted
  feet and move the root; any roll of the Hips needs the legs re-solved.
- **Roll about the pelvis's centre.** It raises the stance socket, and this
  rig has no leg to spare for it.
- **Re-solve with `legik::solve_leg_on`.** It re-aims the foot, and heel
  and tip moved more than before it ran (see
  [the walk's step width and sideways sway](./walk-step-width-and-sideways-sway.md)).

## Consequences

- Headless, 1.2 m/s: swing side lowest 3.8° at 0.19 of the stride. Planted
  soles move ≤ 0.6 mm under the whole locomotion layer (was 45–48 mm).
  Target trunk < 1°.
- Live (BRP, A/B): roll in left single support +2.7° (left up; was −0.65°,
  wrong way). Root weave 101 → 14 mm. Trunk lean 5.9° → 1.5° peak to peak.
- Cost: sway, roll and exact feet together are +2.0 µs per character per
  frame in `anim_bench` (2.2 → 4.2 µs). `RigGeometry::forward` was
  accumulating the whole skeleton's bind rotations, 120 ns a call; it now
  walks one chain (30 ns).

## Revisit when

- Walking speeds far from ~1.2 m/s matter: the curve is Winter's natural
  cadence, not scaled with speed.
- The pelvic turn about the vertical is now composed into the same
  re-solve ([the walking pelvis's turn](./walking-pelvic-turn-and-chest-counter-twist.md)).
  Any further Hips motion belongs there too.

## Related

- [7.4.5 Sample moment and power curves](../../biomechanics-winter/ch07-three-dimensional-kinematics-and-kinetics/7.4-kinetic-analysis-reactions-and-moments/7.4.5-sample-moment-and-power-curves.md) — source: the H1-F/H2-F/H3-F bursts and abductor moment.
- [The walk's step width and sideways sway](./walk-step-width-and-sideways-sway.md) — context: the sway applied in the same pass.
- [A lagging pelvis rotation slides planted feet](./a-lagging-pelvis-rotation-slides-planted-feet.md) — same-trap: a slow spring breaking a counter-rotation the target gets right.
- [Rig authored at critical extension](./rig-authored-at-critical-extension.md) — why the roll pivots on the stance hip.
- [The walking pelvis's turn](./walking-pelvic-turn-and-chest-counter-twist.md) — contrast: the transverse motion, where Winter gives timing but not size.
- [A gait-timed motion cannot ride a weighty spring](./a-gait-timed-motion-cannot-ride-a-weighty-spring.md) — deeper: the general rule behind the Spine spring change.
