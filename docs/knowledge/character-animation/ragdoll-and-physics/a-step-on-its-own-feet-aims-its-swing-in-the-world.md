---
title: A step on its own feet aims its swing in the world and lands where the capture point will be
description: "A ragdoll on its own feet steps past its capture point (on by default): swing thigh aimed in the world, tracked implicitly with the arc's rate and acceleration fed forward, landing re-aimed each frame. Catches ~0.2 m/s more each way. Read before changing stepping in joint_drive.rs."
type: decision
status: current
tags:
  - ragdoll
  - balance
  - locomotion
  - physics
  - debugging
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/joint_drive.rs
  - src/character/anim/ragdoll.rs
  - src/character/anim/ragdoll_plugin.rs
sources:
  - "tests ragdoll_plugin::tests::a_step_on_its_own_feet_lands_where_it_was_aimed, a_push_its_feet_cannot_catch_is_caught_by_a_step_on_its_own_feet"
  - "probes ragdoll_plugin::tests::probe_own_feet_push_matrix, probe_driven_ragdoll_stands (PUSH_V=x,z; ignored)"
  - "Yin, Loken & van de Panne (2007), SIMBICON: Simple Biped Locomotion Control, SIGGRAPH (swing hip in the world frame, stance hip takes the torso)"
  - "Hof, Gazendam & Sinke (2005), The condition for dynamic stability, J Biomech 38:1 (extrapolated COM, foot placement past it)"
  - "Raibert (1986), Legged Robots That Balance (foot placement re-aimed through the flight/swing)"
  - "live character_gallery --ragdoll on --stand-on-own-feet 60 --hit-bone Hips --hit-velocity 0,0,V, puppet_base and character.glb"
aliases:
  - steps_on_own_feet
  - OwnStep
  - own_rest
  - swing foot drift
  - recovery step on its own feet
  - plan_own_step
  - swing_point
---

# A step on its own feet aims its swing in the world and lands where the capture point will be

When its capture point leaves its soles, a body standing on its own feet
(`Ragdoll::steps_on_own_feet`, on by default since 2026-10-02) steps
rather than falls. It only acts where the body would otherwise fall, so it
can never lose a push the feet catch. Measured with
`probe_own_feet_push_matrix` on `puppet_base`, pushes on every body but
the feet:

| Direction (drawn rig faces −Z) | Feet only | Stepping |
|---|---|---|
| Forward | 0.4 m/s | 0.5-0.7 (two steps, joined) |
| Back | 0.5 | 0.6-0.8 (0.7+ ends in a wide stance) |
| Sideways, each way | 0.5 | 0.6-0.7 (one side step, wide stance) |

## Context

The first version swung the foot to a two-bone IK solution and landed it
where the capture point would be, but the swinging foot drifted 7-13 cm
outward and the body ran away in side steps. Each cause below was found by
tracing a push frame by frame (`probe_driven_ragdoll_stands`), not by
tuning. Each one looked like the drift's cause until the next was found.

## Decision

1. **The swing thigh aims in the world** (`JointDrive::override_world`),
   SIMBICON's swing hip. Its twist is the animation's under the
   animation's upright pelvis. Held relative to its pelvis, the thigh
   followed the pelvis's ~20° yaw in single support. The tracking error
   was only 3-4°, but at 0.9 m of leg that is 7-11 cm at the foot. This
   was the outward drift.
2. **The swing is tracked implicitly** with the joint drive
   (`drive_impulse_with`, per-axis gains). Its stiffness and damping are
   the leg beyond the joint as a lump at 4 Hz, critically damped, and the
   solve is sized on the bodies' own inertia.
   - Applied explicitly after the drive, the thigh body alone took the
     whole leg's damping and chattered ±100 N·m against its abductors'
     budget.
   - Sized on the lump instead, a ball-joint knee passes no twist on, and
     the shin spun ~50 rad/s about its own length.
3. **The arc's rate and acceleration are fed forward**
   (`override_rate`, `override_acceleration`): the damping acts on the
   spin less the arc's, and `lump × acceleration` is fed as torque. Without
   them a critically damped 4 Hz tracker trails an accelerating arc, and
   the foot landed at 0.18-0.21 of a 0.31 m step.
4. **Swing targets take their twist from the animation**, not from the
   bodies. Aimed from the thigh's own rotation, a target kept whatever
   twist the thigh had, its rate fed that twist back, and the hip spun up
   to 9 rad/s about the thigh.
5. **The landing is re-aimed each frame** at where the capture point will
   be when it lands (Hof; Raibert's hoppers do the same): the stance
   foot's pressure point plus `(capture − pressure)·e^{remaining/√k}`,
   plus `STEP_MARGIN` along the step. Sideways it is also at least
   `SIDE_MARGIN` 4 cm outside that point and at least half the stance
   width out, never across the stance foot. Planned once, a body standing
   on one foot beside its COM drifted sideways through the swing, ran on
   over the new foot, and fell as the other joined.
6. **A step ends when it has arrived** (within `ARRIVED` 4 cm, or 1.5× its
   time). Ended on touching alone, a dragging foot ended a side step 20 cm
   short. Side steps take `OWN_STEP_SECONDS` 0.3 s, not the kinematic
   balance's 0.2: at the swing's speed Hill's curve leaves the hip
   abductors ~50 of their 90 N·m, and the foot covered half the step.
7. **After the step the body rests where it was caught**
   (`Ragdoll::own_rest`, the capture point as it landed), not over the
   middle of the feet. Pulled to the middle, a forward step's staggered
   stance sent it back at 0.39 m/s, past the rear foot.
8. **The other foot joins only once the body has settled** (`JOIN_SPEED`
   0.05 m/s) and the standing foot can hold it alone (the capture point,
   not just the COM, over that sole). Before the join the weight shifts
   onto that foot at `SHIFT_SPEED` 0.2 m/s. Joined at 0.15 m/s, a body
   still drifting 0.1 m/s sideways could not be stopped by one ankle's
   evertors (0.42 N·m/kg), and it fell over the standing foot.

## Alternatives considered

Each of these was measured on the full push matrix and either lost or
changed nothing:
- **Capping the landing at the leg's reach from a hip at standing height.**
  Every step got shorter, and every direction fell more. This rig stands
  near full extension (see the rig note), so there is almost no reach at
  that height.
- **Carrying the foot 8 cm high through the swing** (up by a third, down
  in the last fifth) instead of a 5 cm `sin(πt)` arc. As many pushes were
  lost as were won.
- **A structural (hinge-strength) limit on the knee's axial rotation**,
  since a near-straight knee is locked by its ligaments. No gain on the
  matrix, so the muscle's 0.35 N·m/kg stays.
- **Choosing the foot to stand on by where the step caught the body**,
  rather than by the COM. Forward and back recoveries then failed.
- **Standing the COM at quiet stance's 5 cm ahead of the ankles.** It
  turned the feet-only limits round (see the standing note), but side
  steps failed at 0.6 m/s.

## Consequences

- Forward and back pushes to 0.6 m/s end standing as before: the feet
  joined, the hips within 15 mm of their height, the pelvis within 5° of
  how it stood (`a_push_its_feet_cannot_catch_is_caught_by_a_step_on_its_own_feet`).
  Each push in that test falls without the step.
- Steps land within 4 cm of their aim, measured ~1 cm on a 22 cm step
  (`a_step_on_its_own_feet_lands_where_it_was_aimed`). Re-aimed relative
  to the pelvis, the same test misses by 92 mm and the forward push falls.
- **Side steps end in a wide stance (feet ~0.6 m apart, hips 7-8 cm low)
  that never joins.** The weight cannot be shifted across it: the far
  ankle's evertors saturate (0.9-1.1 of budget) through its 9 cm height.
- **Single support is the weak phase.** The stance hip's abductors yield
  (1.2× budget, eccentric) and the pelvis rolls 10-15° toward the swing,
  which costs the swinging foot its clearance. This, not the step's aim,
  is what limits sideways recovery.
- **Live** (`character_gallery`, a blow to the hips alone, which also
  stuns them):
  - On `puppet_base`, a 3 m/s backward blow steps and stands again, feet
    together, in 8 of 9 runs; 4 and 5 m/s fall.
  - On `character.glb`, 2.5 m/s stands in 3 of 3 runs and 3 m/s falls in
    3 of 3; at 2 m/s the ankles alone hold it.
  - Front and Left gizmo views agree with the headless runs.
- The outcomes near each limit are chaotic: one cell of the matrix flips
  with small changes, as with fall landings.

## Revisit when

- A wide stance should close up: that needs a weight shift through the
  hips rather than the far ankle, or a step of the near foot instead.
- Lateral recovery should go further: a crossover step, or a stance hip
  that holds the pelvis level in single support.

## Related

- [A standing ragdoll carries its weight through joint torques](./a-standing-ragdoll-carries-its-weight-through-joint-torques.md) — prerequisite: the drives, the COP law and the capture-point test this extends.
- [A ragdoll's joint muscles have a strength, a twitch and a speed](./a-ragdolls-joint-muscles-have-a-strength-a-twitch-and-a-speed.md) — applies: the budgets and Hill's curve that limit the swing and the stance hip.
- [A stumble is a capture-point step, then a join](../ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md) — contrast: the kinematic balance's step, whose rule this one borrows.
- [A pinned ragdoll tracks its targets' velocity](./a-pinned-ragdoll-tracks-its-targets-velocity.md) — same-trap: damping toward rest trails a moving target.
- [A fall test samples one chaotic landing](./a-fall-test-samples-one-chaotic-landing.md) — same-trap: why one matrix cell is not a result.
- [Rig authored at critical extension](../ik-and-locomotion/rig-authored-at-critical-extension.md) — why the leg has no reach at standing height.
