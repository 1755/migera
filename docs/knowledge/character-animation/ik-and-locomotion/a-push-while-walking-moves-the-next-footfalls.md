---
title: A push while walking moves the next footfalls; from behind, it speeds the walk
description: "walk_balance::WalkBalance models only a push's difference from the walk: the pendulum about the stance foot, each swinging foot aimed at the capture point (Hof), settled 300 ms before footfall; a forward push becomes a decaying speed surge. Catch limits depend on the phase. Read before changing walk_balance.rs or pushes/hits on a moving character."
type: decision
status: current
tags:
  - balance
  - biomechanics
  - locomotion
  - ik
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/walk_balance.rs
  - src/character/anim/plugin.rs
  - src/character/anim/balance.rs
  - examples/character_gallery.rs
sources:
  - "Hof, Vermerris & Gjaltema (2010), Balance responses to lateral perturbations in human treadmill walking, J Exp Biol 213:2655-2664, doi:10.1242/jeb.042572 — next foot placed a fixed distance outward of the extrapolated COM; lateral ankle strategy ≤ 2 cm COP shift, ~200 ms; stepping needs ≥ 300 ms before foot placement"
  - "Hof, Gazendam & Sinke (2005), The condition for dynamic stability, J Biomech 38:1-8 (extrapolated centre of mass)"
  - "tests walk_balance::tests (8); probe walk_balance::probe::probe_walking_catch_limits (ignored)"
  - "live BRP, character_gallery --anim-speed-schedule 0:1.2 --push-schedule …, puppet_base and character.glb"
aliases:
  - WalkBalance
  - walking push recovery
  - foot placement strategy
  - extrapolated centre of mass
  - XcoM
  - speed surge
  - AnimFootIk::displaced
---

# A push while walking moves the next footfalls; from behind, it speeds the walk

A walker pushed sideways or back recovers by where it puts the next foot,
not by holding still over the stance foot. `walk_balance::WalkBalance`
models only the push's difference from the walk. Each swinging foot lands
on the difference's capture point, and the walk goes on, displaced by the
push. A push from behind is a short surge in walking speed instead.

## Context

The standing balance ([push recovery](./push-recovery-is-winters-pendulum.md),
[stumble](./a-stumble-is-a-capture-point-step-then-a-join.md)) ran only on
the standing side of the walk blend. A push while walking moved nothing
visible, though it could still flag a fall.

## Decision

- **The difference is linear.** It is a COM offset from the walk's own path,
  its velocity, and each foot's offset from where the walk puts it. Through
  a stance the offset follows `COM̈ = (COM − COP)/K` about the stance foot's
  offset (the double support moves the COP from the trailing foot to the
  leading one). The ankle steers the COP up to `ANKLE_REACH` (3 cm forward,
  1.5 cm sideways) within the foot. Hof et al. (2010) measured ≤ 2 cm.
- **Foot placement (Hof).** Each swinging foot aims at where the capture
  point `offset + velocity·√K` will be at footfall. That is predicted by
  running the same pendulum and ankle law forward. Once landed, the offset
  is on the stable branch and comes to rest over the new foot. The aim is
  **settled `PLACEMENT_DELAY` (300 ms) before footfall**, Hof's measured
  minimum for the stepping strategy. A later push waits for the next step.
- **Limits:** a step is at most the leg's length foot to foot. It may cross
  in front of the stance foot by up to 15 cm (`MIN_STEP_WIDTH`). The
  crossover is this module's extrapolation, not Hof's finding, and moves
  inward only late in the swing, once past the stance foot. The body is
  lost past 0.45 m from its support (`LOST_OFFSET`, about a 25° lean). A
  3 s forecast decides `falls`, as the standing balance does.
- **From behind, a surge.** A walking step is already most of a leg long,
  so it can't lengthen enough to catch a forward push: 0.4 m/s from behind
  asked for a 1.16 m step, then longer ones, and fell. The forward part of
  a push becomes `surge` (m/s added to the asked speed), decaying with
  `SURGE_SECONDS` = 1 s. Past `MAX_SURGE` (1.5 m/s, the standing forward
  catch limit) it falls.
- **Applied:** the body moves by the offset as root motion
  (`GalleryStride::stepped`). Each foot goes to its offset relative to the
  moved body through `AnimFootIk::displaced`, which the IK adds to the
  animated toe before the lock and the ground see it. A displaced foot
  locks, grounds and releases where it really is. A hit lands on the
  standing `Balance`, and the gallery hands it over (`Balance::take_push`)
  while walking.

## Measured

Catch limits by the phase the push lands at, `puppet_base`-sized stride
(`probe_walking_catch_limits`, m/s):

| push lands | sideways | back |
|---|---|---|
| just after a footfall (cycle 0.0–0.15) | 0.25–0.50 | 0.65–1.20 |
| mid-swing, inside the placement delay (0.2–0.35) | 0.10–0.20 | 0.35–0.45 |
| late swing (0.4–0.45) | 0.20–0.35 | 0.55–0.70 |

A push inside the delay waits for the step after, most of a stride later;
the pendulum's 0.31 s time constant grows it ~12× by then. Live at
1.2 m/s, each push alone (0.12 sideways, 0.5 forward, 0.3 back) was caught
in 3 of 3 runs on both rigs. Four pushes 3 s apart felled `character.glb`
once. A 6 m/s hit to the chest was caught, the path displaced 0.70 m; at
14 m/s it fell. Planted feet slid a median 3.9 mm per stance, ≤ 4.8 mm
around pushes.

## Alternatives considered

- **Stop the walk and hand the push to the standing stumble.** Any push
  would end the walk, and the walk's own 1.2 m/s then has to be caught by
  stumble steps.
- **Hof's aim without the delay.** It caught more (0.25–1.15 m/s sideways)
  by re-aiming a foot about to land, which a walker can't.
- **Predicting without the ankle.** With the aim held through the delay,
  every foot landed past the body; a 0.5 m/s push back was still rocking
  between the feet 6 s later.

## Revisit when

- Step timing should change too (landing early or late): not modelled; it
  is the main thing missing at the weak phases.
- The trunk should lean with a push (hip strategy): nothing poses it.

## Related

- [Push recovery is Winter's pendulum](./push-recovery-is-winters-pendulum.md) — prerequisite: the standing model this extends to a moving support.
- [A stumble is a capture-point step, then a join](./a-stumble-is-a-capture-point-step-then-a-join.md) — contrast: the standing version, which steps from feet side by side.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — applies: why moving the body by the offset as travel keeps the stance foot planted.
- [A pinned root's velocity is not its pace](../ragdoll-and-physics/a-pinned-roots-velocity-is-not-its-pace.md) — applies: what a fall while walking launches with.
