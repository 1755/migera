---
title: A standing ragdoll carries its weight through joint torques, fed forward, on planted feet
description: "Ragdoll::stand_on_own_feet: unpinned under full gravity, each joint applies +τ/−τ to its two bodies, solved implicitly every avian substep; each joint is fed the weight it carries; planted feet are Dominance 1. Stands 5-9 s, hips within 5 mm. Read before changing joint_drive.rs or unpinning a ragdoll."
type: decision
status: current
tags:
  - ragdoll
  - physics
  - balance
  - numerics
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/joint_drive.rs
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/ragdoll.rs
sources:
  - "tests ragdoll_plugin::tests::a_ragdoll_stands_on_its_own_feet_with_its_joints_carrying_it, a_ragdoll_on_its_own_feet_still_falls; joint_drive::tests"
  - "probe ragdoll_plugin::tests::probe_driven_ragdoll_stands (ignored)"
  - "live BRP, character_gallery --ragdoll on --stand-on-own-feet 180, puppet_base and character.glb"
  - "avian3d 0.7 source: dynamics/solver/schedule.rs (SubstepSchedule order), solver_body/mod.rs (SolverBody, SolverBodyInertia), rigid_body Dominance"
  - "Catto, Solver2D / Box2D v3 soft step (implicit soft constraints): https://box2d.org/posts/2024/02/solver2d/"
  - "Tan, Liu & Turk (2011), Stable Proportional-Derivative Controllers, IEEE CG&A 31(4)"
aliases:
  - JointDrive
  - stand_on_own_feet
  - carries_itself
  - carry_weight
  - joint drive
  - gravity compensation
  - feed-forward torque
  - planted foot dominance
---

# A standing ragdoll carries its weight through joint torques, fed forward, on planted feet

`Ragdoll::stand_on_own_feet` (plan step 4b) releases the pinned root and
puts gravity back in full. From then on each joint holds its pose with a
torque between its own two bodies, `+τ` on the child and `−τ` on the
parent, in N·m (`joint_drive::JointDrive`). Three parts are all needed.
Without any one of them, the body falls.

## Context

The pose controller (`math/pd.rs`) turns each body toward its WORLD
target with an angular acceleration on that body alone. Nothing pushes
back on the body it hangs from, and it holds the body up only by setting
`GravityScale = 1 − strength`. Unpinned under real gravity it buckled at
0.5 s (see
[an unpinned ragdoll needs soles](./an-unpinned-ragdoll-needs-soles-and-weight-bearing-control.md)).
Paired torques at a standing body's stiffness are unstable explicitly
integrated once per 64 Hz step against a light foot.

## Decision

1. **Each joint drive is an implicit soft constraint, solved every
   substep.** `apply_joint_drives` runs in avian's `SubstepSchedule`
   before velocity integration, on `SolverBody` (the live rotation is
   `delta_rotation × Rotation`; `Rotation` is the step's start). With `e`
   the child's rotation error relative to the parent's, `v` the relative
   spin and `K` the two bodies' world inverse inertias summed:
   `P = −(1 + cK)⁻¹(h·kp·e + c·v)`, `c = h·kd + h²·kp`. That is implicit
   Euler, stable at any gain and step (`joint_drive::tests`: a 1e6 N·m/rad
   drive on a 1e-4 kg·m² body at 64 Hz only settles). Gains:
   `DRIVE_STIFFNESS_PER_KG` 10 N·m/rad/kg of body mass for legs and
   spine, the head and arms less (`drive_share`), damping 0.1 s.
2. **Each joint is fed the weight it carries** (`carry_weight`, once per
   physics step). This is the gravity moment, about the joint point, of
   the side the ground does not hold:
   - the child's side, for an arm, the head, the trunk above a spine
     joint, or a leg in the air;
   - on a planted leg, that leg above the joint plus the leg's share of
     everything on no planted leg, by where the COM stands between the
     feet.

   Nothing is fed with no foot planted. Standing on `puppet_base`, each
   ankle carries 38 N·m (0.54 N·m/kg), each knee 34, each hip 27-29.
3. **A planted foot is held by the ground.** A foot touching a static
   body (`ContactGraph`) gets `Dominance(1)`, so the joint above moves the
   leg and never pushes the foot. Its ankle drive is sized on the shin
   alone and puts nothing on the foot (`child_grounded`): the ground
   carries the reaction.

The old per-body controller is off for a body that carries itself
(`apply_joint_torques` skips it), and a fall removes the drives and the
dominance (`manage_joint_drives`).

## Alternatives considered

Measured with `probe_driven_ragdoll_stands`, the drawn stance unpinned
after 1 s pinned:

| | result |
|---|---|
| drives only, kp 10-80 N·m/rad/kg | sagged 44 mm at once at every gain, fell over in 1.5-3 s |
| + ankle reaction on the planted foot, sized on the shin | spun the foot out of the floor: NaN in a frame |
| + ground carries the ankle's reaction, no dominance | ankles 20° off, fell sooner |
| + planted feet dominant | feet held (0.1 mm), still toppled; ankles 5° off at kp 10 and 40 alike |
| + weight fed forward | stands 10 s, hips within 5 mm, sway 2-6 cm |
| drives sized on each side as a rigid lump (composite inertia) | torques 1e8 N·m in a frame: the side flexes, so the drive overshoots it |

- **The 44 mm "sag" was the feet sinking**: both soles 52 mm into the
  floor and tilted 10° in 9 frames. avian solves joints after contacts in
  every substep, each correction split by inverse mass, so a 1 kg foot
  under the body takes nearly all of it. It is the cause of the toe dip in
  falls too (see
  [a falling body is hinged and fleshed](./a-falling-body-is-hinged-and-fleshed.md)),
  where dominance was rejected because a falling leg must drag its foot.
  A planted foot should not move.
- **Why stiffness alone cannot carry weight**: a drive's correction is
  sized on its two bodies, but the shin cannot turn without the body
  above it. Its effective stiffness is capped near the light bodies'
  scale, whatever `kp`: the ankle error was the same at 10 and 40.
  Feeding the weight forward leaves the drive only the small corrections.

## Consequences

- Headless (`a_ragdoll_stands_on_its_own_feet_with_its_joints_carrying_it`,
  5 s): hips drop ≤ 5 mm, sway ≤ 6 cm, feet ≤ 1 mm, knees 2-3° and ankles
  ~3.4° off their targets. Each part, removed, fails it: no feed-forward,
  the hips sank 607 mm; no dominance, 837.
- Live, both rigs, BRP (`--stand-on-own-feet 180`), ~9 s: hips height
  within 2 mm, sway ≤ 25 mm (`puppet_base`) and 41 (`character.glb`),
  feet 0.0 mm.
- **The whole body sways over its ankles, undamped**, ±2 cm with a ~2 s
  period. The weight fed forward cancels gravity at the present lean, so
  only the weak drive stiffness resists it. Steering the COM over the feet
  is the balance controller's job (step 4c).
- The screen still draws the animation while the body carries itself: the
  read-back hangs the skeleton on the bodies only while falling. Read the
  bodies over BRP.
- No way back to a pinned root yet (step 4.5). After a get-up the body
  stands on its own feet again.

## Revisit when

- A balance controller (4c) adds an ankle and hip torque: add it to the
  feed-forward, not as more stiffness.
- A planted foot must roll onto its toes or heel, or a hand bears weight:
  the planted test is "touches the ground", and dominance pins the whole
  foot.
- avian gains joint inverse-mass scaling (PhysX `setInvMassScale`): it
  could replace dominance.

## Related

- [An unpinned ragdoll needs soles and weight-bearing control](./an-unpinned-ragdoll-needs-soles-and-weight-bearing-control.md) — prerequisite: the feet and the failure this answers.
- [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) — contrast: the bound the implicit drive does not have.
- [Use avian's apply_angular_acceleration, not apply_torque](./avian-apply-angular-acceleration-not-torque.md) — contrast: the per-body controller's units, which these drives replace for a body standing on its own.
- [A falling body is hinged and fleshed](./a-falling-body-is-hinged-and-fleshed.md) — same-trap: the same mass ratio sinks feet in a fall.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — applies: why the check reads bodies, not the screen.
