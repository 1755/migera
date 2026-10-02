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
updated: 2026-09-30
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
| [A pinned ragdoll tracks its targets' velocity, not rest](./a-pinned-ragdoll-tracks-its-targets-velocity.md) | Damped toward rest a body trails a moving target by `2ζ·ω_target/ω`; feeding each target's spin into the damping cut the walking error 11.5 → 9.5° median, p90 17 → 10-12°; the arms' ~5° left is shoulder coupling | before changing the ragdoll's PD control or chasing its walking error |
| [A fall test samples one chaotic landing](./a-fall-test-samples-one-chaotic-landing.md) | Rounding-level torques changed three tests' landings and they failed on the new ones; one landing pins one outcome | before writing, or "fixing", a fall or rise test that broke after an unrelated change |
| [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) | Stable only while `kd·dt < 2`; more damping making things worse is the signature; tune `max_torque`, not frequency | before tuning PD gains or when damping increases oscillation |
| [A pinned root's velocity is not its pace](./a-pinned-roots-velocity-is-not-its-pace.md) | A velocity-driven kinematic body reads 2× its pace, then 0, whenever a frame runs two physics steps; falls launched the root at 0 in 6/10 walking falls; use `KinematicRoot::velocity` | before reading any kinematic body's velocity, or changing the fall's launch |
| [Ragdoll body and anchor frames](./ragdoll-body-and-anchor-frames.md) | Body rotation = bone world rotation; anchors in the body's midpoint frame | before changing ragdoll spawning, body placement or anchors |
| [avian joint limits are not cone and twist](./avian-joint-limits-are-not-cone-and-twist.md) | With `twist_axis = +Y` limits are two bend stops; use `twist_axis = +X` | before configuring joint limits or upgrading avian |
| [An unpinned ragdoll needs soles and weight-bearing control](./an-unpinned-ragdoll-needs-soles-and-weight-bearing-control.md) | Unpinned it buckles at 0.5 s and capsule feet skate 0.8 m; sole blocks fix the feet (0.1° / 0.16 mm); the PD holds weight only by switching gravity off | before unpinning a ragdoll or making it balance |
| [A standing ragdoll carries its weight through joint torques](./a-standing-ragdoll-carries-its-weight-through-joint-torques.md) | `stand_on_own_feet`: paired joint torques solved implicitly every substep, the load and Winter's COP law fed forward as statics, planted feet dominant, ankles held to the sole; stands a minute, catches 0.4 m/s pushes, falls past its capture point | before changing `joint_drive.rs`, unpinning a ragdoll, or adding balance torques |
| [A step on its own feet aims its swing in the world](./a-step-on-its-own-feet-aims-its-swing-in-the-world.md) | Past its capture point it steps (on by default): swing thigh aimed in the world, tracked implicitly with the arc's rate and acceleration fed forward, landing re-aimed each frame, rest where caught, join once settled; ~0.2 m/s more each way; side steps end wide | before changing stepping, swing tracking or the join in `joint_drive.rs` |
| [A ragdoll's joint muscles have a strength, a twitch and a speed](./a-ragdolls-joint-muscles-have-a-strength-a-twitch-and-a-speed.md) | Joint drives capped by maximal voluntary torque per kg (Harbo 2012), Hill force-velocity, commands lagged by Winter's twitch; the lag cuts the push caught to ~0.4 m/s | before tuning joint budgets, lags or stun on a body standing on its own feet |
| [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) | At strength 1 the screen is the animation; measure body-vs-target error and spin over BRP | before declaring any ragdoll change verified |
| [A fall hands the body to physics](./a-fall-hands-the-body-to-physics.md) | A push or hit falls when the balance, forecast 3 s ahead at its first step, loses its lean (a hit with no balance: capture point outside the feet); the root is released with the push's velocity, the simulation shown with the skeleton on the hips body; tone is joint damping 3/s; a fallen body needs 12 substeps and a declared rest or it creeps | before changing falls, the fall trigger, or handing a body back (H3) |
| [A falling body has hinged knees and elbows and solid flesh](./a-falling-body-is-hinged-and-fleshed.md) | A fall swaps knees and elbows for hinges (AAOS ranges) and makes the body's parts collide; limbs are anthropometric capsules, the torso flat blocks; hip and shoulder cones lean to the middle of their range. Knees went from folding 163° backward to −5..99°, hip extension from 43° to 30°, overlaps from 180 mm to ≤ 18 | before changing ragdoll colliders, joint limits, joints during a fall, or rest detection |
| [Getting up goes through key poses chosen by how the body lies](./getting-up-is-a-timed-blend-then-a-re-pin.md) | Face up: sit → squat (VanSant); face down: hands and knees → half-kneel; on a side: side-sit first. Keys solved on the rig to meet the floor and chained so shared contacts hold, blended per bone in world space with dipping feet tucked, then the bodies are set back on their bones and the root pinned | before changing get-up, the key poses, or adding get-up motion |

## See also

- [Lugaru's joint/muscle animation system](../lugaru-joint-muscle-system.md) — origin of the continuous strength dial.
- [Anim studio is complete](../animation-core/anim-studio-is-complete.md) — measured chatter at limits and mid-range at Phase 8.
- [Prefer BRP over prints for live ECS state](../../engineering-practice/debugging/prefer-brp-over-prints-for-live-ecs-state.md) — how every ragdoll bug here was localized.
- [Winter Ch. 4 — Anthropometry](../../biomechanics-winter/ch04-anthropometry/INDEX.md) — segment masses, COM fractions and radii of gyration (Table 4.1) for ragdoll bodies.
- [Winter Ch. 8 — Forward solutions](../../biomechanics-winter/ch08-synthesis-forward-solutions/INDEX.md) — why open-loop torque playback drifts, spring/damper joints, and the external-vs-internal torque distinction.
- [Winter Ch. 9 — Muscle mechanics](../../biomechanics-winter/ch09-muscle-mechanics/INDEX.md) — activation lag, force-velocity and passive elasticity as candidate actuator refinements.
