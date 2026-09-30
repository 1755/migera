---
title: Ragdoll and physics
description: "migera's active ragdoll on avian: PD controller units and stability bound, body and anchor frames, avian joint-limit semantics, and why full-strength read-back cannot verify physics. Read before changing ragdoll.rs, ragdoll_plugin.rs, math/pd.rs, or tuning any ragdoll gain or limit."
type: index
status: current
tags:
  - ragdoll
  - physics
  - correctness
  - numerics
updated: 2026-09-28
---

# Ragdoll and physics

migera's active ragdoll (`src/character/anim/ragdoll.rs`,
`ragdoll_plugin.rs`, `math/pd.rs`) drives avian rigid bodies joined by
`SphericalJoint`s with one quaternion PD controller per joint and a
continuous per-joint strength dial. Each note here is a trap that made the
ragdoll look right, or explode, for a reason other than the obvious one.

## Start here

Read [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md)
before claiming any ragdoll change works. Read the two avian notes before
touching controller or joint setup.

| Note | What it establishes | Read when |
|---|---|---|
| [Use avian's apply_angular_acceleration, not apply_torque, for PD control](./avian-apply-angular-acceleration-not-torque.md) | PD gains are acceleration-shaped; `apply_torque` double-scales by inertia; plus three avian setup gotchas | before wiring any controller or physics system into avian |
| [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) | Stable only while `kd·dt < 2`; more damping making things worse is the signature; tune `max_torque`, not frequency | before tuning PD gains or when damping increases oscillation |
| [Ragdoll body and anchor frames](./ragdoll-body-and-anchor-frames.md) | Body rotation = bone world rotation; anchors in the body's midpoint frame | before changing ragdoll spawning, body placement or anchors |
| [avian joint limits are not cone and twist](./avian-joint-limits-are-not-cone-and-twist.md) | With `twist_axis = +Y` limits are two bend stops; use `twist_axis = +X` | before configuring joint limits or upgrading avian |
| [An unpinned ragdoll needs soles and weight-bearing control](./an-unpinned-ragdoll-needs-soles-and-weight-bearing-control.md) | Unpinned it buckles at 0.5 s and capsule feet skate 0.8 m; sole blocks fix the feet (0.1° / 0.16 mm); the PD holds weight only by switching gravity off | before unpinning a ragdoll or making it balance |
| [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) | At strength 1 the screen is the animation; measure body-vs-target error and spin over BRP | before declaring any ragdoll change verified |

## See also

- [Lugaru's joint/muscle animation system](../lugaru-joint-muscle-system.md) — origin of the continuous strength dial.
- [Anim studio is complete](../animation-core/anim-studio-is-complete.md) — measured chatter at limits and mid-range at Phase 8.
- [Prefer BRP over prints for live ECS state](../../engineering-practice/debugging/prefer-brp-over-prints-for-live-ecs-state.md) — how every ragdoll bug here was localized.
- [Winter Ch. 4 — Anthropometry](../../biomechanics-winter/ch04-anthropometry/INDEX.md) — segment masses, COM fractions and radii of gyration (Table 4.1) for ragdoll bodies.
- [Winter Ch. 8 — Forward solutions](../../biomechanics-winter/ch08-synthesis-forward-solutions/INDEX.md) — why open-loop torque playback drifts, spring/damper joints, and the external-vs-internal torque distinction.
- [Winter Ch. 9 — Muscle mechanics](../../biomechanics-winter/ch09-muscle-mechanics/INDEX.md) — activation lag, force-velocity and passive elasticity as candidate actuator refinements.
