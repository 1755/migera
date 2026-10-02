---
title: A standing ragdoll carries its weight through joint torques, fed forward, on planted feet
description: "Ragdoll::stand_on_own_feet: unpinned, full gravity, joint torques +τ/−τ solved implicitly every substep, each joint fed its load and Winter's COP law as statics, planted feet dominant, ankles held to the sole. Stands a minute; 0.4 m/s pushes caught. Read before changing joint_drive.rs or unpinning a ragdoll."
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
  - "tests ragdoll_plugin::tests::a_ragdoll_stands_on_its_own_feet_with_its_joints_carrying_it, a_ragdoll_on_its_own_feet_still_falls, a_ragdoll_on_its_own_feet_stands_a_minute_without_drifting, a_ragdoll_on_its_own_feet_recovers_a_push_within_its_ankles_budget, a_push_beyond_its_feet_makes_a_ragdoll_on_its_own_feet_fall; joint_drive::tests"
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed., §11.2.1 (ankle strategy, hip load/unload)"
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
  - within_sole
  - UNCATCHABLE
  - standing balance on the bodies
  - virtual model control
---

# A standing ragdoll carries its weight through joint torques, fed forward, on planted feet

Contents: [Context](#context) · [Decision](#decision) ·
[Alternatives](#alternatives-considered) · [Consequences](#consequences) ·
[Stepping](#stepping-on-its-own-feet-tried-experimental-off-by-default) ·
[Revisit when](#revisit-when)

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

4. **Balance (step 4c): Winter's law on the measured COM, as
   statics.** `carry_weight` puts the pressure at
   `COP = COM + (COM − rest)·k·ω² + v·2ζωk`, the kinematic balance's own
   law and gains (`RECOVERY_FREQUENCY` 3.5, `RECOVERY_DAMPING` 1). It is
   held inside the planted soles' hull. `rest` is the COM's place over the
   feet when both were first planted.
   - The ground then pushes on each planted foot at its share of the COP,
     with its load's weight and the horizontal force the pendulum needs
     (`g·(COM − COP)/height`).
   - Every stance-leg joint carries that push's moment about itself, less
     the segments below it. Every other joint carries its hanging part
     under the effective gravity `g − a` (d'Alembert). Static, this is
     item 2 exactly.
   - Each leg's share follows where the COP stands between the feet: the
     hips' load/unload.
   - A planted ankle's total torque is held to what keeps its pressure
     inside its sole (`within_sole`).
   - If the capture point (`COM + v·√k`) leaves the soles by more than
     `UNCATCHABLE` (2 cm), it falls: with no step to take on its own feet
     yet, nothing can catch it.

5. **Switching is blended (step 4.5).** On its own feet, the screen
   shows the bodies, the hips placed on the hips body.
   - `Ragdoll::stand_blend` eases between the animation and the bodies
     over `SWITCH_SECONDS` (0.3 s) each way.
   - `stop_standing_on_own_feet` pins the root where the body stands and
     eases the pin to the animation over the same time
     (`KinematicRoot::settle`).
   - Measured per frame on the drawn skeleton: on, 1.4 mm and 0.5° at
     worst; off, 2.0 mm and 0.6°. Switched in a frame instead: on, 3.7 mm
     and 1.05°; off, 20.4 mm and 2.2°
     (`switching_onto_and_off_its_own_feet_does_not_pop`).
   - The drives' strengths, twitch and stun are in
     [a ragdoll's joint muscles](./a-ragdolls-joint-muscles-have-a-strength-a-twitch-and-a-speed.md).
   - Headless, one ragdoll costs 0.64-0.88 ms a frame pinned and
     0.71-1.01 on its own feet (`probe_standing_cost`, three runs): no
     difference beyond the noise.

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
- **The balance as a COP on the ankles alone** (the extra torque on the
  shins only, knees and hips fed the static weight): the sway fell to
  ±5 mm but never settled, and the COM sat 2 cm from rest while the COP
  asked 2.7 cm the other way. The knees gave way instead of passing the
  push on. Carrying the ground reaction's moment up every stance joint
  settled it within 1.5 s.
- **Planted ankles without pose stiffness**, so the balance alone sets
  their torque: the shins tipped 60° on the feet while the rest of the
  body bent to keep the COM over them.
- **No limit on the ankle torque**: with the foot glued, the ankle
  pushed as if the pressure stood anywhere. 1.2 m/s each way was "caught",
  and a 0.8 m/s side push was held by the unloaded foot pulling on the
  floor, which a real foot would lift off.
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
- **With the balance (4c):** still within 1.5 s; over a further 55 s the
  COM stays within 5 mm (`a_ragdoll_on_its_own_feet_stands_a_minute_without_drifting`).
  It settles 16 mm from its recorded rest, where the pose targets and the
  law agree. Before the balance, the body swayed ±2 cm with a ~2 s
  period, undamped.
  - **Pushes on the body (feet excluded):**
    - 0.4 m/s each way: caught, the COM back within 1 cm, feet ≤ 3 mm,
      ankles ≤ 1.6 N·m/kg
      (`a_ragdoll_on_its_own_feet_recovers_a_push_within_its_ankles_budget`).
    - Before the muscles' limits: back 0.5 caught, 0.6 falls; forward
      0.4 caught, 0.6 falls; sideways 0.6 caught, 0.8 falls
      (`a_push_beyond_its_feet_makes_a_ragdoll_on_its_own_feet_fall`).
    - With them (strength per axis, force-velocity, the balance's twitch
      lag, 2026-10-02): forward 0.4 caught, 0.5 falls; back 0.5 caught,
      0.6 falls; sideways 0.5 caught, 0.6 falls. Why forward is the
      weakest is open: the sole reaches 0.22 m ahead of the ankle and
      0.08 behind it.
    - The drawn test character faces −Z. These limits were first
      recorded with forward and back swapped (a +Z push is backward).
  - **Live, both rigs:** the last 5 s within 5-6 mm (the idle animation's
    breathing moves the targets).
- The plan's co-contraction ratio is not logged: one feed-forward split
  by share and relative drives cannot co-contract by construction.
- After a get-up the body stands on its own feet again, the screen
  blending in afresh from the rise's end.

## Stepping on its own feet: tried, experimental, off by default

`Ragdoll::steps_on_own_feet` (2026-10-02) steps instead of falling when
the capture point leaves the soles. Each part was found by tracing a
backward 0.6 m/s push frame by frame (`probe_driven_ragdoll_stands`,
`PROBE_STEPS=1`):
- **Where:** the standing balance's own rule, the swinging sole landing
  where the capture point will be (`e^{T/√k}` from the stance foot's
  pressure point), plus `STEP_MARGIN` 5 cm. Landed exactly there, the
  body was only just caught and ran on.
- **The swing:** the thigh and shin are driven toward a two-bone IK
  solution (`leg_toward`, unit-tested) along a lifted arc. A tracking
  torque `I·(−ω²e − 2ω·ω_rel)` on the leg as a lump about each joint
  is applied every substep at 4 Hz. Sized on the thigh body alone, the
  foot covered 4 cm of a 20 cm step. Applied once per physics step, the
  tracking chattered and once flung the body away. Now the foot covers
  ~70 % of the step (`an_experimental_step_on_its_own_feet_swings_to_where_it_planned`).
- **The foot is carried flat** as it lifted: at the animation's angle to
  a shin swung forward, it landed toes down and behind, on a corner.
- **Support is the whole sole face**, whatever its tilt: taken as the
  corners on the ground, a backward step landing toes first stood on its
  toe edge and read as uncaught.
- **After a step:** the planted legs are solved to their feet from the
  hips held at the stance's height, against the animation's upright
  pelvis. Solved from where the hips were, they let the body sink 50 mm
  in 0.6 s; left on the animation's angles, the trunk tilted 7-23°. Then
  the other foot joins at the stance's own width. Pushing the COM onto
  the stepped foot first carried it past the foot's edge and into ever
  wider side steps.
- **Still open, why it stays off:** the swinging foot drifts 7-13 cm
  outward although its target and IK are right, at any tracking
  stiffness (4-10 Hz). The body then stands on a foot far to the side,
  falls toward the swing side, and runs away in side steps. Most pushes
  past the feet still fall.

## Revisit when

- The swinging foot's outward drift is understood (see the stepping
  section): then `steps_on_own_feet` can be on by default.
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
