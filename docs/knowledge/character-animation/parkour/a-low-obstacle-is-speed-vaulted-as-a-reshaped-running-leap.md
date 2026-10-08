---
title: A low obstacle is speed-vaulted as a reshaped running leap
description: "Step 7, second part: a speed vault is a running leap over the obstacle whose flight is reshaped (hips rolled, legs tucked out to the side, trunk leant onto a hand planted on the top), the body moved whole to keep the leap's COM; the walker adjusts its last steps to the take-off. Read before changing parkour/vault.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-08
verified: 2026-10-08
code:
  - src/character/anim/parkour/vault.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - src/character/anim/stance.rs
sources:
  - "tests parkour::vault::tests::{it_vaults_a_low_obstacle_from_a_run_and_runs_on, an_obstacle_out_of_a_vaults_reach_is_not_vaulted}"
  - "live: character_gallery --step-seconds 0.0333333 --block 0,-8,180,0.9,3.0,0.3 --vault-at 0.5 --anim-speed 3.5, Xvfb, gizmos on, mesh off"
  - "anim_bench --gait vault --characters 100"
  - "Mansour et al. 2024 (hurdle clearance); Slawinski et al. 2019 (steeplechase barrier); gymnastics vault table contact"
aliases:
  - speed vault
  - vault
  - Jump::vault
  - HangAsk::Vault
  - Vaulting
  - stride adjustment
---

# A low obstacle is speed-vaulted as a reshaped running leap

Step 7 of the [parkour design](./parkour-moves-implementation-design.md),
second part: running at a low obstacle, vault it with one hand and run on.

Contents: [Decision](#decision) · [Alternatives](#alternatives-considered) ·
[Traps](#traps) · [Consequences](#consequences) · [Revisit](#revisit-when)

## Decision

**A vault is a running leap whose flight is reshaped** (`Jump::vault`,
`parkour::vault::Vaulting`). The [leap from a
run](../ik-and-locomotion/a-jump-from-a-run-replays-the-runs-stance-on-a-planned-com.md)
already plans the take-off, a ballistic flight and a landing that runs on.
The vault plans one so that:
- the COM tops out 0.38 m over the top (hurdlers clear by 0.23-0.39 m);
- it rises at least 0.2 m (about a 0.4 s flight);
- it lands 0.9 m past the far face.

`Jump::pose_at` hands each pose to the vault (`Vaulting::reshape`), which:
1. **Rolls the hips** about the line of running, up to 1.15 rad, so the legs
   swing out to the lead (free) leg's side. It rolls in over 0.2 s (at most
   0.45 of the flight) from take-off, and back over 0.12 s before touchdown.
2. **Leans the trunk** onto the other hand, from 0.55 rad up to 1.45. It
   takes the least lean at which that hand reaches the top all through its
   contact; the rig's arms are short.
3. **Tucks each leg** in the rolled hips' frame: the ankle 0.3 m ahead of
   and 0.15 m under its socket, raised toward the socket's height as far as
   puts it 0.15 m over the top. The leg starts from where the rolled hips
   carry the leap's own leg, the knee pointing up and a little ahead, the
   foot level.
   - The **lead leg** tucks from 0.1 s before take-off, in 0.15 s, and is
     let down to land by touchdown.
   - The **take-off leg** rises first and then comes on (a quadratic path).
     It stays tucked until the leap's own swing has carried it 0.35 m past
     the far face, after touchdown.
4. **Moves the whole body** to keep the leap's COM, so the flight stays on
   the leap's parabola.
5. **Plants the hand** on the top, 3 cm over it (the palm under the wrist):
   - **where:** under its shoulder, in from the faces;
   - **when:** centred on the COM over the obstacle's middle, while the
     hips are fully rolled;
   - **how long:** while the shoulder sweeps 0.25 m either side, 0.12-0.17 s
     at 4-3 m/s (gymnasts touch the vault table 0.12-0.22 s);
   - **reaching and letting go:** reaching over 0.12 s, letting go over
     0.2 s;
   - **settling:** the arm and the COM's move are solved four times over.

**The plan checks itself.** Posed through its flight, nothing may go into
the obstacle. Failing, it springs up to 0.3 m higher before it gives up.
The best take-off has the COM topping out over the obstacle's middle, the
take-off toe at least 1.0 m from the near face (steeplechasers leave
1.34 m out). It plans from there to 0.4 m farther (0.8 m for the higher
obstacles); nearer, the lead toe reaches the face.

**The walker aims its last steps** (`HangAsk::Vault`, `Walker::ledge` the
obstacle), as a long jumper does on the approach:
- **at each foot's contact**, it finds the obstacle ahead along its way;
- **its pace**: it stretches or shortens its pace by up to 20 % (`vault_pace`)
  so a whole number of steps brings a foot 0.15 m past the best take-off;
- **when it takes off**: from the first foot whose vault plans, where the
  next foot would be past the best or no nearer the aim;
- **too late**: once past the best, the ask is dropped;
- **walls**: it does not go round walls while running to vault.

**Reach** (`LOWEST`..): obstacles 0.75 m up to 0.1 m over the hips (a 1.04 m
top on `puppet_base`), at most 0.7 m deep, met within 0.5 rad of square,
from a run of at least 2.5 m/s.

## Alternatives considered

- **A flight of its own**: the leap's flight already lands into the run at
  the right foot and speed. Reshaping it keeps that hand-back.
- **Keeping feet out of the obstacle by where they are** (lifting any foot
  near it over the top): measured by position, the lift switched on within
  a frame as a tucking foot moved 0.15 m a frame, at 21 m/s. Keeping the
  take-off leg tucked until it is past took its place.
- **Keeping the hand reach by a lower clearance**: 0.32 m put the take-off
  toe 3.8 cm into a 0.9 m wall. Leaning the trunk further took its place.
- **A fixed take-off spot**: the plan's window (0.4-0.8 m) is narrower than
  a running step (1.0-1.35 m). Without the pace, a live run at 3.5 m/s had
  no foot in it and never vaulted.

## Traps

- **The plan's COM is in the standing hips' frame.** Read as heights over
  the floor, the shoulder came out 1.73 m over a 0.6 m rail.
- **The hips' height is the forward kinematics', not the root
  translation's.** The root translation is not the hips joint, and every
  obstacle was refused as higher than the hips.
- **A reshaping switched on in a frame is a pop.** On a short flight the
  roll went full at take-off, a toe 1 m in a frame (61 m/s). Every part
  eases in and out over its own time.
- **The hand's target must not be lerped from the wrist as last solved**:
  each pass crept it on, and turning the clavicle from the last solve
  lifted it again each pass. Each pass starts from the reshaped body.
- **The legs must be carried with the rolled hips.** Tucked from where the
  unrolled leap had them, the take-off leg hung back under the rolled hips
  and its knee went 10 cm into a 0.9 m wall.
- **A knee left in its hinge swings across under the body** as its ankle
  goes over to the far side, at 12 m/s; it is pointed up and ahead.
- **The test reports only the deepest joint.** A lead toe 2.4 cm in hid
  behind a take-off leg 10 cm in until that was fixed.
- **A planted hand moves about the COM at the run's speed**, so letting go
  in 0.12 s from 4 m/s swung it at 11 m/s.

## Consequences

**Headless** (`puppet_base`, 60 fps; 0.75 m by 0.25 m, 0.9 by 0.3 and 1.0
by 0.5; 3 and 4 m/s; either foot; 0, 0.2 and 0.4 m past the best take-off;
36 vaults):
- nothing goes into the obstacle (a toe skims it by at most 0.32 mm);
- the hand is on its plant within 0.13 mm;
- the COM stays on the leap's parabola within 0.19 mm;
- no joint goes over 13.6 m/s about the COM (the leap's own: 7-9). The
  bound is 14 m/s, by eye: a sprinter's swing foot goes about 10 m/s about
  the COM; the pops were 21-61;
- it runs on at 2.98-3.0 m/s from 3, and 3.13-3.70 m/s from 4.

A 0.3 m kerb, a 1.2 m wall, a 1 m deep block and a 1.5 m/s run are refused.

**Live** (Xvfb, 1/30 s steps, gizmos on, mesh off; a 0.9 m block 0.3 m deep
at 3.5 m/s): the pace stretched to 1.2 over the last steps, it took off
1.64 m from the face, 0.15 m past its best of 1.49. From the front the body
crosses above the top's edge with a hand down to it, and from the top over
the block. The side views are hard to read: the block hides the crossing.

**Cost**: `anim_bench --gait vault --characters 100`, 152 µs a character
over its 0.9 s against a running leap's 71 µs. Each pose is reshaped and the
arm solved four times. Six times over it was 211 µs; nothing reshaped
outside the flight.

## Revisit when

- **The speed lost.** From 4 m/s it runs on at as little as 3.13 m/s (the
  leap's loss for its rise).
- **The legs' whip** (13.6 m/s about the COM, under a by-eye 14), if it
  shows. Skilled traceurs lose about 0.2 m/s over a monkey
  vault (Feletti et al. 2023).
- **The cost**, if crowds vault: the arm's four solves and the legs' IK
  each pose.
- **Data**: no speed-vault kinematics are published. The LAAS parkour
  motion database has safety and kong vaults to measure.
- **Lazy and kong vaults**: the lazy vault (from an angle, a leg and then
  the other) and the kong (both hands, legs between) are not built.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 7 of.
- [A jump from a run replays the run's stance on a planned COM](../ik-and-locomotion/a-jump-from-a-run-replays-the-runs-stance-on-a-planned-com.md) — prerequisite: the leap a vault reshapes, and its hand-back to the run.
- [A block is mantled as a climb up from the floor](./a-block-is-mantled-as-a-climb-up-from-the-floor.md) — contrast: the other half of step 7, above a vault's height.
- [Parkour movement data](./parkour-movement-data.md) — deeper: the hurdle, steeplechase and vault-table numbers the vault takes its clearance and contact from.
