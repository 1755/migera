---
title: A fall hands the body to physics; tone is joint damping, and rest must be declared
description: "A push (or hit, as a push) falls when the balance, run 3 s ahead at its first step, loses its lean; a hit with no balance falls when its capture point leaves the feet. The root is released with the push's velocity; tone is JointDamping 3/s; rest must be declared. Read before changing falls or hits."
type: decision
status: current
tags:
  - balance
  - ragdoll
  - physics
  - correctness
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/balance.rs
  - src/character/anim/ragdoll.rs
  - src/character/anim/ragdoll_plugin.rs
  - examples/character_gallery.rs
sources:
  - "tests: balance::tests::a_push_past_a_catchable_step_falls; ragdoll_plugin::tests::a_released_ragdoll_falls_with_its_momentum_and_is_drawn_where_it_lies, a_fallen_body_that_never_sleeps_by_itself_is_put_to_rest, a_blow_topples_a_character_with_no_balance_when_its_feet_cannot_take_it"
  - "probes (ignored): ragdoll_plugin::tests::probe_fall_damping, probe_ragdoll_substep_cost; balance::tests::probe_forecast_cost, probe_catch_table"
  - "live BRP, character_gallery --ragdoll on --push-schedule 3:1.5:0 (and 3:0:1.2, 3:-1.5:0), puppet_base and character.glb"
aliases:
  - H2
  - fall
  - Ragdoll::fall
  - MAX_CATCH
  - fall forecast
  - catches_ahead
  - FORECAST_SECONDS
  - LOST_FALLS
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

- **Trigger: a forecast.** When the first recovery step is planned,
  `Balance::catches_ahead` runs a copy of the balance (it is `Copy`)
  `FORECAST_SECONDS` (3 s) ahead at 1/60 s, with every step it would take.
  If the copy loses momentum to the validity bound (`lost` >
  `LOST_FALLS`, 0.1 m/s), the push falls, known within 0.2 s of it. This
  is capturability judged by the model itself, over all its steps: runaways
  lost momentum 0.65–2.1 s in, and caught pushes settle well inside 3 s.
  It costs 42–55 µs, once per stumble (`probe_forecast_cost`, worst
  frame, three runs). The live balance ticks at the same 1/60 s at most, so it takes
  the steps the forecast did (see
  [the stumble note](../ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md)).
  `lost` stays a late safety net for anything the forecast missed.
- **A hit is a push.** `apply_ragdoll_hits` pushes the character's
  `Balance` by the struck body's share of the momentum, (m_body/m_total)·v,
  horizontal, in the rig's axes. The balance then absorbs, steps or falls
  as for any push. The thorax is 21.6% of the body, so a chest blow steps
  from about 3 m/s and falls from about 5.5. Before this, the pinned root
  held the character up through any blow.
- **A character with no `Balance` falls if its feet can't take the hit.**
  The push has nowhere to go, so the rule is capturability without a
  step: it falls when the capture point `Δv·√K` lies outside the support
  under both feet (`Support::of(pose, live rig)`). Before this, only a
  character with a balance controller could be toppled.
- **Release keeps the momentum, the push's too.** The kinematic root moves
  with the hips' velocity (`follow_kinematic_roots`) and keeps it when
  turned `Dynamic`: carried at 1 m/s, the falling hips go on 6+ cm in the
  next 0.1 s. But a fall is judged at the push's *start*, before the push
  has moved the body. So the caller passes the balance's velocity plus
  its undelivered push (`Ragdoll::fall_moving`), and every body gets it.
  Without it, every push, whatever its direction, dropped the body the
  same way onto its back (all eight test falls read face up and turned
  the same angle). With it, forward falls land face down and backward
  falls face up.
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
- **Fall only when the bound discards momentum** (`lost`). This is right,
  but it comes late: 0.65–2.1 s into a runaway of shuffling steps. The
  forecast asks the same question at once.
- **A length rule on the first step** (`MAX_CATCH`, 0.77 m; the trigger
  until 2026-09-30). Every push whose first step asked ≤ 0.73 m was
  caught and every one asking ≥ 0.81 m ran away, forward and back. But
  sideways it was too conservative: at 1.2 m/s the first step asks
  0.85 m, and four crossovers catch it anyway.
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
  slowly for ~6 s first. (Measured before the launch velocity.)
- With the push carried: forward falls land face down on both rigs,
  backward face up. Sideways, `puppet_base` rolled face down both ways
  and `character.glb` ended face up.
- With the forecast, `MAX_STEP` 0.7 m and the early join, `puppet_base`
  as drawn catches forward to 1.5 m/s, sideways to 1.4 and back to 1.4;
  forward 1.6, sideways 1.5 and back 1.5 fall. Live, both rigs catch
  1.2 m/s sideways each way. (Limits taken before 2026-10-01 stood the
  test character with its arms overhead and read inverted: see
  [the puppet_base fixture note](../rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md).)
- At the limit, frame timing decides some catches: 1 in 4 uneven frame
  patterns fell there. The forecast runs at 1/60 s, so the live balance
  can lose a catch the forecast called; `lost` then makes it a fall late.
- Found on the way: the read-back's quaternions drifted off unit length
  through their own feedback (see
  [normalize what you read back from your own output](../../engineering-practice/debugging/normalize-what-you-read-back-from-your-own-output.md)).

## Revisit when

- A character with no `Balance` that is walking: its rule assumes both
  feet down and a still body.
- Moving targets: the launch is the balance's COM velocity; a character
  hit while walking needs the walk's velocity added.

## Related

- [A stumble is a capture-point step, then a join](../ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md) — prerequisite: the step whose failure this is.
- [Getting up goes through key poses chosen by how the body lies](./getting-up-is-a-timed-blend-then-a-re-pin.md) — deeper: what `at_rest` hands on to.
- [A falling body has hinged knees and elbows and solid flesh](./a-falling-body-is-hinged-and-fleshed.md) — deeper: what the joints and colliders become during the fall, and why rest is judged by motion.
- [An unpinned ragdoll needs soles and weight-bearing control](./an-unpinned-ragdoll-needs-soles-and-weight-bearing-control.md) — context: why the ragdoll only falls, and doesn't balance.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — same-trap: verify bodies over BRP, not the picture.
- [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) — contrast: the PD's damping; joint damping is avian's implicit one.
- [A fall test samples one chaotic landing](./a-fall-test-samples-one-chaotic-landing.md) — same-trap: why a fall test can fail on a change that doesn't touch falls.
- [A pinned root's velocity is not its pace](./a-pinned-roots-velocity-is-not-its-pace.md) — deeper: the release launches the root at its target's pace, not its last physics step's velocity.
