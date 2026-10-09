---
title: A skid stop slides side-on and rises over stuck feet
description: "Step 11, second part: a fast run asked to stop skids. Its body turns side-on, the feet slide out ahead, braked at 0.5 g and leant back by atan(μ); slowed, the feet stick and the hips come over them into the stand. Asked to face back, the body turns round as it slides. Read before changing parkour/skid.rs."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/skid.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::skid::tests::a_run_skids_to_a_stop_or_round_to_face_back"
  - "live: character_gallery --anim-speed-schedule 2:5,8:0 --skid (stop) and --anim-speed-schedule 2:5 --steer-at 12,180 --skid (round), Xvfb, gizmos on/mesh off Left, mesh on; BRP pelvis and feet"
  - "anim_bench --gait skid / --gait plant-turn (real_rig) --characters 100"
aliases:
  - skid stop
  - plant-and-turn
  - 180 turn
  - Skid
  - Walker::skid
---

# A skid stop slides side-on and rises over stuck feet

Step 11 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md),
second part: reversing from a run. There is no skid data. The friction,
0.5, is a shoe sliding on a hard floor; the rest is by eye.

## Decision

**A skid** (`parkour::skid::Skid`) starts from a run of at least 3.2 m/s,
off the foot that comes down. It has four parts:
- **Turning side-on**: the body turns 1.2 rad away from the swinging leg
  over the braking.
- **The slide**: braked at `μg` (4.9 m/s²). The hips are 0.28 m low, the
  trunk leant 0.3 rad back against the way, and the arms out at 0.6 of
  the beam's. Both feet slide on the floor out ahead, as far as puts the
  centre of mass `atan(μ)` behind them.
- **The entry** (0.35 s): from the run's pose, the legs ease out over the
  whole of it and the hips lower from 0.05 s. A soft floor lift keeps the
  feet out of the floor while the shapes blend, faded out by the end of
  the entry.
- **The rise** (0.5 s): slowed to the speed that carries the hips over the
  feet as it stops, the feet stick where they are. The hips come over them
  into the stand, braked steadily, the legs re-placed onto the stuck feet.

**How far ahead the feet slide** depends on the facing: the centre of mass
sits off the hips differently as the body turns. It is solved at 17
facings through the turn (four corrections of what the shape gives each)
and interpolated.

**A plant-and-turn** is the same skid with the body turned half a turn while
it slides, so it stands facing back. It needs 0.45 s of braking (at least
3.8 m/s on `puppet_base`).

**The walker** (`Walker::skid`, opt-in) skids when running fully at
`SKID_FROM` or faster and either asked to stop or steered more than 2.4
rad off its facing. It stands after, at rest; asked to run, it starts
again from the stand.

## Alternatives considered

- **Feet straight ahead of an upright body**: the legs out ahead carry the
  centre of mass forward with them, so to lean by `atan(μ)` the feet went
  0.53-0.63 m out. From hips 0.12 m low that is 4-5 cm out of reach, and
  the stuck feet ended 5 cm off the stand. Lowering the hips 0.28 m and
  leaning the trunk back brought them in reach.
- **One distance ahead for every facing**: a plant-and-turn leant
  0.37-0.42 rad, not 0.46.
- **A staggered stance in the slide**: the stand it rises into has the feet
  side by side. A stance that does not match it would make the walker's
  stand move a foot.
- **Running back without stopping** (a turn handed back into the run):
  only the jump has a hand-back into the run's clock. Here the turn stands
  and the run starts again.

## Traps

- **Steered round before its foot came down**, the facing turned toward
  the way back at 2 rad/s, and the skid slid on along the turned facing,
  1.7 m aside of where it ran. The walker now holds its steer straight
  while a skid is due.
- **A 1 cm soft floor lift** came on sharply as a toe crossed it, changing
  a step by 3.9 cm in a frame. Softened over 2.5 cm and faded out by the
  end of the entry (the shape's own feet are on the floor), it is gone.
- **The legs thrown out over 0.18 s** whipped the swinging foot through
  12 m/s about the hips. Over the whole 0.35 s entry, 6.6 m/s.

## Consequences

**Headless** (`puppet_base`; runs at 3.5, 4.5 and 6 m/s, off either foot,
stop and round):
- braked at 4.905 m/s², exactly `0.5·g`;
- the centre of mass leant back 0.4632-0.4640 rad from the feet
  (`atan(0.5)` = 0.4636);
- once stuck, the feet held within 0.17 mm;
- toes no more than 1.1 cm into the floor;
- no joint's step changed over 3.1 cm in a frame (the swinging foot
  thrown out ahead; the slide under a slab's strike is 2.8 cm);
- no joint went over 7.6 m/s about the hips;
- it ended exactly standing, side-on or facing back.

A 2.5 m/s run does not skid.

**Live**:
- stopped from 5 m/s: the feet went out ahead on the floor, the whole body
  leant back side-on with the arms out, and it rose over its feet and
  stood;
- turned round from 5 m/s: slid 2.7 m on its line, the hips down to
  0.66 m, the feet held still, then ran back the other way.

**Cost**: `anim_bench --characters 100`: 27 µs a character at p50 for the
stop, 26 µs for the plant-and-turn (on the real rig: the synthetic rig's
legs cannot reach the turned shape).

## Revisit when

- **The run's clock can be picked up from a stand-in move** (as the jump's
  `Resume` does): a plant-and-turn could run on without standing.
- **Skid data**, or a measured shoe-floor friction for the surfaces in a
  level.
- **The knees look stiff** in the slide: the mesh reads as leaning back
  with nearly straight legs. A deeper crouch moves the feet nearer.

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 11's design.
- [A low slab is slid under on the seat from a run](./a-low-slab-is-slid-under-on-the-seat-from-a-run.md) — same-trap: the same run-to-slide entry, legs first, and a soft floor lift that must not come on sharply.
- [A run leans whole into a turn, and its trunk with its speed](./a-run-leans-whole-into-a-turn-and-its-trunk-with-its-speed.md) — context: the same `tan θ = a/g`, here at friction's braking.
