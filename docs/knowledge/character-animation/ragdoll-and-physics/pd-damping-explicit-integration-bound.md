---
title: PD damping has an explicit-integration bound
description: "An explicitly integrated PD controller is stable only while kd·dt < 2 (and sqrt(kp)·dt < 2); past it more damping makes oscillation worse. At avian's 64 Hz, ζ=1 caps frequency near 10 Hz. Express 'corrects harder' via max_torque, never frequency. Read before tuning ragdoll or any PD gains."
type: lesson
status: current
tags:
  - ragdoll
  - physics
  - numerics
  - correctness
updated: 2026-09-25
code:
  - src/math/pd.rs
  - src/character/anim/ragdoll.rs
sources:
  - test ragdoll::tests::every_default_joint_is_well_conditioned_for_the_physics_timestep
aliases:
  - stable_damping
  - PD chatter
  - kd dt bound
---

# PD damping has an explicit-integration bound

A PD controller integrated explicitly is stable only while **`kd · dt < 2`**,
and `sqrt(kp) · dt < 2` for the stiffness term. Past that bound, one step's
correction overshoots the velocity it was meant to remove, and "damping"
starts *adding* energy on alternate steps.

## What happened

The diagnostic signature is that **more damping makes oscillation worse**.
Measured on a jointed two-body chain at avian's 64 Hz default, driving to a
target 15° away:

| ζ | `kd·dt` | chatter |
|---|---|---|
| 0.0 | 0.00 | 3.2 rad/s |
| 0.5 | 0.98 | 10.0 rad/s |
| 1.0 | 1.96 | 12.3 rad/s |
| 4.0 | 7.85 | 16.9 rad/s |

With `kd = 2ζω` and `ω = 2π·f`, at 64 Hz and ζ = 1.0 the frequency ceiling is
about **10 Hz**. migera's ragdoll hip and spine were authored at 9–10 Hz, at
98% of the bound. That hip was the worst-behaved joint in the rig.

## Why it matters

A free body never shows this, which is why it survived migera's Phase 6
single-body tests: with nothing to push against, an over-damped explicit step
just decays. Add a constraint that re-excites the error every step and the
loop closes.

## How to apply

- `PdParams::stable_damping(dt)` clamps the damping gain at runtime, at 0.8
  of the bound for headroom. The bound assumes an isolated body, and a joint
  solver adds its own stiffness.
- The **stiffness** half cannot be clamped after the fact. Author a
  too-high frequency lower instead.
- Express "this joint corrects harder" through `max_torque`, never through
  frequency. Frequency is how fast the correction is integrated, a property
  of the solver, not of the joint's role. (The shipped ceilings were later
  raised by `CEILING_SCALE = 12` for exactly this reason; see
  `CHARACTER_PROGRESS.md`.)
- Test controllers against a constraint, not only on a free body.

## Evidence

`ragdoll::tests::every_default_joint_is_well_conditioned_for_the_physics_timestep`
enforces both halves of the bound.

## Related

- [Use apply_angular_acceleration, not apply_torque](./avian-apply-angular-acceleration-not-torque.md) — prerequisite: gains must be acceleration-shaped first.
- [A pinned ragdoll tracks its targets' velocity, not rest](./a-pinned-ragdoll-tracks-its-targets-velocity.md) — applies: with the gains bounded, a moving target is followed by feeding its velocity forward instead.
- [The rational exp approximation in spring code diverges](../animation-core/spring-exp-approximation-diverges.md) — same-trap: another integration broken by a large rate × `dt`.
- [Anim studio is complete](../animation-core/anim-studio-is-complete.md) — applies: the corrected chatter measurements after `stable_damping`.
- [Restoring-force constraints need perturbed input](../../engineering-practice/testing/restoring-force-constraints-need-perturbed-input.md) — same-trap: a free, unperturbed test cannot exercise the failure.
