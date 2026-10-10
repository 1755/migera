---
title: Use avian's apply_angular_acceleration, not apply_torque, for PD control
description: "PD gains (kp=ω², kd=2ζω) are acceleration-shaped, so avian's apply_torque scales by inertia a second time (a capsule span to 1e7 rad/s); use apply_angular_acceleration. Also: register physics systems in Plugin::finish, Forces already borrows AngularVelocity. Read before wiring controllers into avian."
type: lesson
status: current
tags:
  - physics
  - ragdoll
  - correctness
  - numerics
  - bevy
updated: 2026-09-24
code:
  - src/character/anim/ragdoll_plugin.rs
  - src/math/pd.rs
aliases:
  - apply_torque
  - Forces
  - PhysicsSchedule
  - avian integration gotchas
---

# Use avian's apply_angular_acceleration, not apply_torque, for PD control

A quaternion PD controller's gains (`kp = ω²`, `kd = 2ζω`) are
**acceleration-shaped**: they describe a second-order system directly and
already account for inertia. Drive avian bodies with
`Forces::apply_angular_acceleration`, not `Forces::apply_torque`.

## What happened

avian's `Forces::apply_torque` multiplies by the body's inverse angular
inertia, applying that scaling a second time. On a capsule bone (inverse
angular inertia about 2.9e4 to 2.9e5), a reasonable 2232 became
6.4e8 rad/s², and the body span to 1e7 rad/s in one step. That looks exactly
like the "two rotational springs" instability, but it is a unit error.

## Why it matters

`apply_angular_acceleration` ignores inertia. That also makes a gain mean the
same thing on a heavy hip and a light wrist, so per-joint numbers express
intent rather than compensate for mass. For scale: correcting a 90° error in
about 0.15 s needs roughly 140 rad/s².

## How to apply

Three other avian integration gotchas found the same day:

- **Register physics-schedule systems in `Plugin::finish`, not `build`.**
  Reaching into `PhysicsSchedule` during `build` creates it before avian
  finishes setup. avian's diagnostics registration guards duplicates with
  `is_resource_added` (same tick only), so the result is a physics world
  missing resources its own systems require.
- **`Forces` already borrows `AngularVelocity` mutably.** Querying
  `(&Rotation, &AngularVelocity, Forces)` is a hard conflict. Read them with
  `forces.rotation()` and `forces.angular_velocity()`.
- **Headless avian tests need `AssetPlugin` + `MeshPlugin`** even with no
  renderer, because the collider cache reads `AssetEvent<Mesh>`.

## Related

- [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) — deeper: the other way a PD gain goes unstable in avian.
- [Ragdoll body and anchor frames](./ragdoll-body-and-anchor-frames.md) — applies: the ragdoll plugin these controllers drive.
- [avian joint limits are not cone and twist](./avian-joint-limits-are-not-cone-and-twist.md) — same-trap: another avian API whose meaning differs from its name.
