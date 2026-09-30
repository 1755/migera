---
title: A standing character recovers from a push as Winter's inverted pendulum
description: "balance::Balance steps Eq. 11.3 with COP = COM + s·x + b·ẋ clamped to the feet: a push sways the body over its ankles/hips and it returns, feet ≤ 1.3 mm live. A push must take ~0.1 s, and a push the feet can't absorb is flagged, not held at the edge. Read before changing balance.rs or adding pushes, hits or stepping."
type: decision
status: current
tags:
  - balance
  - biomechanics
  - locomotion
  - springs
updated: 2026-09-30
verified: 2026-09-30
code:
  - src/character/anim/balance.rs
  - examples/character_gallery.rs
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §11.2.1 (Eq. 11.3) and §5.2.9"
  - "test balance::tests (7), incl. the_com_moves_where_the_balance_puts_it, a_swaying_body_keeps_its_feet_planted"
  - "live BRP, character_gallery --push-schedule 3:0.25:0,6:0:0.2,9:-0.3:0,12:0.8:0, both rigs"
aliases:
  - push recovery
  - Balance
  - COP controller
  - ankle strategy
  - capture point
  - --push-schedule
---

# A standing character recovers from a push as Winter's inverted pendulum

`balance::Balance` is Winter's standing body (§11.2.1, Eq. 11.3): the
centre of pressure steers the centre of mass, `COM̈ = (COM − COP)/K` with
`K = d/g`. A push changes the COM's velocity; the body sways over planted
feet and returns.

## Decision

- **The COP law is Winter's stiffness plus damping:**
  `COP = COM + s·x + b·ẋ`. Winter et al. (1998) found quiet standing to be
  "stiffness, not reaction": the COP runs in phase with the COM, slightly
  wider. The damping term is the reactive response to an unexpected push,
  which the book says the sensors stand by for. The law gives
  `COM̈ = −(s·x + b·ẋ)/K`. The gains are this project's choice, not
  Winter's: critically damped at ω = 3.5 rad/s. A 0.2 m/s push peaks at
  2.1 cm (v/(ωe)) and is back within 2 mm and 2 mm/s at ~1.8 s.
- **The COP stays on the feet** (`Support`): a box over every `Sole`
  contact of both feet, 1 cm in. It is also bounded to Winter's 8° model
  validity in each plane, because `puppet_base`'s long toes would allow a
  10° lean.
- **Posed as the body does it** (`Balance::apply`), with the feet held
  exactly by `stance::move_pelvis_over_feet`:
  - front to back, the pelvis carries the trunk over the ankles;
  - side to side, the hips load one leg (the pelvis moves toward it and
    drops on the unloaded side), `shift_weight`'s roll scaled continuously.
  - The pelvis moves further than the COM is to move, by
    `COM_PER_PELVIS = (0.863, 0.810)`, measured with `centre_of_mass` on
    `puppet_base`. On `character.glb` it may be ±10 % off.

## Two things that failed first

- **A push delivered in one frame broke the feet.** A 0.8 m/s shove moved
  the unsprung pelvis 13 mm a frame while the legs' 0.015 s springs lagged
  it. The rendered toes moved fast enough for the foot locks to let go,
  and they re-planted 23 mm away. A push now takes `PUSH_SECONDS` (0.1 s),
  as a shove does. Feet then stayed ≤ 1.3 mm (balls) and ≤ 4.9 mm (ankles)
  through every push, on both rigs. During that 0.1 s an external force
  acts, so Eq. 11.3 does not hold; its test skips those frames.
- **Holding an unabsorbable push at the support's edge.** When the capture
  point `x + ẋ·√K` leaves the feet, only a step can catch the body. Held
  at the edge with its velocity zeroed, the body could never come back:
  the clamped COP sits exactly under the COM there, an equilibrium the law
  cannot leave. The clamp's jump also moved the planted feet 32 mm. Now the
  push is flagged (`needs_step`), and while it is, the COP is unclamped,
  standing in for the step that would put it there. The body comes back
  smoothly (0.8 m/s peaks at ~10 cm of COM). The capture point is a later
  formalisation of the same pendulum, not Winter's.

## Measured, live

| push | `puppet_base` peak pelvis | `character.glb` |
|---|---|---|
| 0.25 m/s forward | +22.9 mm | +28.4 mm |
| 0.2 m/s left | +21.8 mm | +26.7 mm |
| 0.3 m/s back | −31.8 mm | −75.0 mm (flagged: its heel reaches less far) |
| 0.8 m/s forward | +116.6 mm (flagged) | +146.1 mm (flagged) |

All return. Left view: the body leans from the ankles, trunk in line, feet
flat. Not in `anim_bench` (the gallery drives it); it runs only while the
body is unsettled.

## Revisit when

- Stepping exists: `needs_step` is its hook, and the unclamped COP should
  become the step.
- Pushes arrive while walking: only the standing side of the blend sways
  now.

## Related

- [11.2.1 Quiet standing](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.2-standing-balance-ml-and-ap/11.2.1-quiet-standing.md) — source: Eq. 11.3, stiffness control, load/unload.
- [The walk's step width and sideways sway](./walk-step-width-and-sideways-sway.md) — contrast: the same pendulum driven by the stride instead of a push.
- [A gait-timed motion cannot ride a weighty spring](./a-gait-timed-motion-cannot-ride-a-weighty-spring.md) — same-trap: a fast pelvis against lagging leg springs.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — context: the locks a one-frame push released.
