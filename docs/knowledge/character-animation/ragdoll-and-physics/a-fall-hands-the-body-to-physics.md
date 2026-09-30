---
title: A fall hands the body to physics; tone is joint damping, and rest must be declared
description: "A push asking for a step over MAX_CATCH (0.8 m) falls: Ragdoll::fall releases the root with its velocity and shows the simulation, skeleton on the hips body, entity following. Tone is JointDamping 3/s, not a pose; a fallen body needs 12 substeps and a declared rest or it creeps. Read before changing falls or H3."
type: decision
status: current
tags:
  - balance
  - ragdoll
  - physics
  - correctness
updated: 2026-09-30
verified: 2026-09-30
code:
  - src/character/anim/balance.rs
  - src/character/anim/ragdoll.rs
  - src/character/anim/ragdoll_plugin.rs
  - examples/character_gallery.rs
sources:
  - "tests: balance::tests::a_push_past_a_catchable_step_falls; ragdoll_plugin::tests::a_released_ragdoll_falls_with_its_momentum_and_is_drawn_where_it_lies, a_fallen_body_that_never_sleeps_by_itself_is_put_to_rest"
  - "probes (ignored): ragdoll_plugin::tests::probe_fall_damping, probe_ragdoll_substep_cost"
  - "live BRP, character_gallery --ragdoll on --push-schedule 3:1.5:0 (and 3:0:1.2, 3:-1.5:0), puppet_base and character.glb"
aliases:
  - H2
  - fall
  - Ragdoll::fall
  - MAX_CATCH
  - FALL_DAMPING
  - FALLEN_SLEEP
  - REST_SPEED
  - at_rest
  - release the root
---

# A fall hands the body to physics; tone is joint damping, and rest must be declared

Standing and stumbling stay kinematic. When a push asks for a step no
foot can take, the character is handed to its active ragdoll:
`Ragdoll::fall` releases the pinned root, gravity acts in full, the
screen shows the simulation, and the character entity follows the body.
Three things were not obvious. The balance model could not fall at all.
Muscle tone as a pose controller keeps a fallen body pushing. And a body
lying on the floor does not stop by itself in avian.

## Context

H2 of the hybrid plan: physics only for falls, stumbles and hits. Before
this the ragdoll's root was always pinned (kinematic), the read-back blended
rotations only, and the gallery had no physics floor.

## Decision

- **Trigger: the planned step is longer than `MAX_CATCH` (0.8 m).**
  `Balance::falls` is set when a recovery step, planned from the whole
  push (the undelivered part too), wants more. This is capturability:
  no reachable step catches it. The 0.8 m comes from the model's own
  verdict, measured with its validity clamp lifted. Forward, 0.8 m/s
  (asks 0.73 m) is caught and 1.0 m/s (0.90 m) runs away. Backward,
  1.0 m/s (0.71 m) is caught and 1.2 m/s (0.87 m) runs away.
- **Release keeps the momentum.** The kinematic root already moves with
  the hips' velocity (`follow_kinematic_roots`). Turned `Dynamic`, it
  keeps that velocity: carried at 1 m/s, the falling hips go on 6+ cm in
  the next 0.1 s. The other bodies are dynamic throughout.
- **Display: all simulation, root included.** `Ragdoll::shown` is 1 while
  falling. `write_simulated_pose` puts the hips joint at the hips body
  minus its offset (`Fall::root_offset`, taken from `KinematicRoot` at
  release) and moves the character entity across the ground under it.
  Every bone is drawn within 0.1° of its body.
- **Tone is joint damping, not a pose.** `FALL_TONE` = 0 and
  `FALL_DAMPING` = 3/s (avian `JointDamping` on every joint). A pose
  controller at 0.15 kept pushing a lying forearm toward the standing
  pose (0.57 m/s after 3 s). The damping value came from a sweep at 12
  substeps over three fall directions: only 1–3/s rested every fall
  (asleep by 2.9–4.8 s); 0 whipped limbs at 7.4 m/s and two of three
  never slept; 10 turned a collapse into a slow ooze and one never slept.
- **Rest has to be declared.** Three layers:
  1. The gallery runs avian at **12 substeps**. At 6, a fallen
     `character.glb` jittered past the sleep bound and crept 7 mm/s
     forever.
  2. `FALLEN_SLEEP` (0.1 m/s, 0.3 rad/s) replaces avian's 0.15/0.15 on a
     fallen body. At rest, 6–7 bodies still spun 0.05–0.18 rad/s about
     changing axes.
  3. `rest_fallen_ragdolls`: every body under `REST_SPEED`/`REST_SPIN`
     (0.05 m/s, 0.5 rad/s) for 1 s is put to sleep (`SleepBody`) and
     marked `Fall::at_rest`, the settled signal H3 needs. Some landings
     never go under the bound by themselves: live, a hips-up body slid
     4 mm/s, and which landing happens varies run to run with the
     frame's physics-step count.

## Alternatives considered

- **Keep catching everything with the clamp.** The balance's 8° validity
  clamp held the COM at its bound *with velocity zeroed*, so a 2 m/s shove
  was "caught" by one 0.4 m step, with 1.98 m/s simply discarded. That was
  never a catch.
- **Fall when the clamp discards momentum.** This matches the model
  forward and back, but sideways it fires on every step. Stepping off the
  far foot starts the COM at the 8° bound, so even a longer side step
  is clamped mid-swing.
- **Body-level (air) damping.** This slows the fall itself. Joint
  damping resists only relative motion.
- **Sleep thresholds alone.** This isn't enough: some resting poses creep
  above any sane bound.

## Consequences

- Cost: 12 substeps take a `puppet_base` ragdoll frame from 0.66–0.77 ms
  to 0.94–0.96 ms p50, headless (`probe_ragdoll_substep_cost`). The
  kinematic stack is unaffected; `anim_bench` doesn't include physics.
- Live, both rigs: 1.5 m/s forward and back, 1.2 m/s sideways all fall
  and lie with the head on the floor (neck ~0.13 m). Four of five were at
  rest 2.5–3.5 s after the push. A sideways `puppet_base` fall rolled
  slowly for ~6 s first.
- **Sideways the balance cannot tell a catch from a fall.** Every
  sideways stumble is caught by the clamp, not by the step, so the
  "0.7 m/s left caught by a 0.4 m side step" of the stumble note was the
  clamp's doing. `MAX_CATCH` still makes 1.0 m/s sideways fall.
- `character.glb` falls at 0.8 m/s backward where `puppet_base` steps: its
  heel reaches less far, so the step asked for is longer.
- Found on the way: the read-back's quaternions drifted off unit length
  through their own feedback (see
  [normalize what you read back from your own output](../../engineering-practice/debugging/normalize-what-you-read-back-from-your-own-output.md)).

## Revisit when

- H3 hands a body back: `Fall::at_rest` is the trigger. The balance was
  reset at the fall, so the character stands anew from wherever the
  entity followed the body.
- Sideways stumbles should be real catches: the pendulum needs a
  crossover or loaded side step, or a validity bound measured with the
  landing foot.
- Hits: a `RagdollHit` strong enough to topple should call `fall` too;
  only pushes through the balance do now.

## Related

- [A stumble is a capture-point step, then a join](../ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md) — prerequisite: the step whose failure this is.
- [An unpinned ragdoll needs soles and weight-bearing control](./an-unpinned-ragdoll-needs-soles-and-weight-bearing-control.md) — context: why the ragdoll only falls, and doesn't balance.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — same-trap: verify bodies over BRP, not the picture.
- [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) — contrast: the PD's damping; joint damping is avian's implicit one.
