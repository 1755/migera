---
title: A pinned ragdoll tracks its targets' velocity, not rest
description: "Damped toward rest, a body chasing a moving target trails it by 2ζ·ω_target/ω; the walking ragdoll's worst body trailed by a median 10-12°. JointTargetVelocity feeds each target's frame-to-frame spin into the damping: 9.5/8.1° median, p90 17 → 10-12°. Acceleration feedforward was noise. Read before changing the ragdoll's PD control."
type: decision
status: current
tags:
  - ragdoll
  - physics
  - springs
  - performance
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/math/pd.rs
  - src/character/anim/ragdoll.rs
  - src/character/anim/ragdoll_plugin.rs
sources:
  - "test math::pd::tests::a_tracked_target_is_followed_without_the_damping_lag (with its untracked control: 0.199 rad predicted and measured)"
  - "live BRP, character_gallery --anim-speed 1.2 --ragdoll on, JointTarget against the body's Rotation, both rigs, 2-3 runs each"
aliases:
  - JointTargetVelocity
  - pd_torque_tracking
  - velocity feedforward
  - walking ragdoll lag
---

# A pinned ragdoll tracks its targets' velocity, not rest

The ragdoll's PD drove each body toward its target orientation, with its
damping pulling the body's spin toward zero. Following a target that
keeps moving, a body like that trails it in steady state by
`2ζ·ω_target/ω_n`: at the shipped 8 Hz, critically damped, 0.04 s worth of
the target's motion. A limb swinging at 5 rad/s trails by 11.5°, the
walking error that had been measured all along.

## Decision

- `JointTargetVelocity` on each body: the target's own angular velocity,
  measured frame to frame as targets are published
  (`publish_joint_targets`). `pd_torque_tracking` damps the body's spin
  toward it, within the joint's ceiling.
- **Not while falling:** a fallen body's targets move only because the
  character follows and turns the body lying there.
- **Not for jumps or noise:** over 30 rad/s a target has jumped (a re-pin)
  and under 1e-3 rad/s it is rounding. Both are fed as zero.

## Measured

Live, walking at 1.2 m/s, the error of the worst body in each BRP sample:

| | puppet_base | character.glb |
|---|---|---|
| median, before → after | 11.5 → 9.5° | 10.4 → 8.1° |
| p90 | 16.8 → 12.4° | 16.8 → 10.3° |
| max | 21 → 15° | 25 → 13° |

The feet fell from 5.7° to about 2.5°. Standing still, the error is 0.0°
before and after.

What remains is the upper arms, about 5°. That is not lag: the arm's
error correlates with the target's acceleration (r ≈ 0.5) and not with
its velocity, has no constant part (0.4° mean), and doesn't change with
the torque ceilings doubled (it isn't saturation). The trunk turning under
the arm acts on it through the shoulder joint, which a per-body PD can't
see. Stiffer gains would cancel it, but the 64 Hz physics step bounds them
(`sqrt(kp)·dt`).

## Alternatives considered

- **Feeding the target's acceleration forward too.** The acceleration is
  taken frame to frame from targets published at a jittery frame rate. It
  bought at most 0.7° on the median and raised the worst sample from 20°
  to 28°. Dropped.
- **Higher ceilings.** No change (9.0° at twice the ceilings).

## Revisit when

- Physics runs faster than 64 Hz, so stiffer gains fit the step.
- Targets come from something smoother than frame differences (an
  analytic gait velocity), which could make acceleration feedforward
  worthwhile.

## Related

- [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) — prerequisite: why the gains can't simply go up.
- [A fall test samples one chaotic landing](./a-fall-test-samples-one-chaotic-landing.md) — same change: the 1e-3 deadband, and what it exposed in the tests.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — context: this lag shows only where the ragdoll shows (stun, partial strength).
