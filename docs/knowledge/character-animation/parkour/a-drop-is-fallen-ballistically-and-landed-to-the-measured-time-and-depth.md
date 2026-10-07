---
title: A drop is fallen ballistically and landed to the measured time and depth
description: "parkour::Falling: off an edge the hips fall under gravity; a squat landing to the measured time and knee depth (keyed by drop height); past 1.7 m a roll whose resting height is planned over time; past 4 m the ragdoll. Read before changing fall.rs, any landing, or the walker's ground snap."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-07
verified: 2026-10-07
code:
  - src/character/anim/parkour/fall.rs
  - src/character/anim/walker.rs
sources:
  - "tests parkour::fall::tests::* (drops of 0.9 and 1.8 m, still and at 1.4 m/s)"
  - "live: character_gallery --block 0,0.9,180,0.9,2.0,1.8 --start-height 0.9 --anim-speed-schedule 2:1.2, Xvfb, BRP"
  - "Dai et al. 2020; Puddle and Maulder 2013 (parkour-movement-data)"
aliases:
  - Falling
  - FallPhase
  - STEP_DOWN
  - landing from height
  - drop landing
  - walking off an edge
---

# A drop is fallen ballistically and landed to the measured time and depth

A walker whose ground drops more than a step (`STEP_DOWN`, 0.3 m) below it
from one frame to the next, walking off a top, falls (`parkour::Falling`)
and lands. It is the first part of step 4 of the
[parkour design](./parkour-moves-implementation-design.md); the roll and
the hand-off to the ragdoll come next.

## Decision

**The flight is ballistic**: the hips leave with the root's velocity and
fall under gravity until they are as high above the ground as touching down
puts them. The legs reach from the gait's shape to the landing's, relative
to the hips: knees 25° bent (20-29° measured at contact, every technique),
the feet planted where the hips will come to rest over them. The arms go out,
the trunk nearly upright.

**The landing takes the measured time to the measured depth** (squat
landings from 0.9, 1.8 and 2.7 m: 377, 335, 290 ms; knees at most 116°,
126°, 134°; [movement data](./parkour-movement-data.md)), keyed by the drop
from standing to standing. The depth is the legs' shortening from 25° to the
deepest flexion on this rig's own thigh and shin. The hips' velocity falls
as `v(1-s)^n (1+n·s)` over the time `T`, which goes `2vT/(n+2)` deep, so `n`
fits both. Its braking rises from touchdown to a peak at `T/n` (about 65 ms
from 0.9 m; 77-80 ms measured from 0.75 m) and fades into the bottom. The
forward speed is braked evenly over the same time. The trunk leans forward
1.1 rad a metre of depth, the arms reach forward, then it rises back to
standing at the foot IK's drop (the jump's recovery limit, at least 0.5 s).

**Past about standing height (`ROLL_DROP`, 1.7 m) it rolls** (the guidance
is to roll above standing height, not peer reviewed; a squat from 1.8 m
loads the hips with 6.5 body weights). The roll keeps a copy of the squat
landing it starts as:
1. the squat's own first 0.15 s, the feet planted, the knees giving;
2. 0.2 s tucking: the shape mixed from the squat into a tuck (trunk curled,
   knees to the chest, chin down, arms in), the turn rising to the roll's;
3. rolling once about the body's left tilted 0.45 rad about its way (over a
   shoulder), at the forward speed, or 0.45 of the touchdown speed turned
   forward, at least 2 m/s; the turn its speed over the tuck's radius;
4. 0.9 s coming up: the turn easing off to exactly once round, the shape
   mixed out to standing.

The body's centroid follows one path along the ground (from the squat's
centroid and velocity, at the roll's speed, braking to rest over standing
feet). Its height rests the body on the ground. The shape and turn do not
depend on it, so it is planned once: every 5 ms, the height that puts the
lowest joint as low as standing's, then the greatest over ±60 ms, then
averaged over it.

**Past `FATAL_DROP` (4 m) it does not land**: at touchdown the walker falls
(`Walker::fall_now`), the ragdoll taking the body with the velocity the
kinematic root had; without a ragdoll it rolls. No data; past the measured
2.7 m the loads climb steeply.

**The walker**: `ride_rendered_feet` does not snap the root down onto ground
more than a step below last frame's; it notes the ground (`fall_to`), and
`drive_walkers` starts the fall from the gait's pose next frame. While
falling it counts as `on_holds` (posed by a move off the floor: no snapping,
no foot locks, no pushes).

## Alternatives considered

- **A cubic from the touchdown speed to rest**: it cannot come to rest
  shallower than a third of `v·T`. From 0.9 m the knees went to 136°, not
  116°.
- **Braking hardest at touchdown** (`v(1-s)^n`): the depth fits, but the
  peak load is at the instant of contact, not 65-80 ms after it.
- **Keying the landing by touchdown speed**: see the trap.

## Traps

- **The study's touchdown speeds are not free fall's.** Dai et al. give 3.0,
  4.9 and 6.3 m/s from 0.9, 1.8 and 2.7 m. Free fall from 0.9 m touches down
  at 4.2 m/s, and the three do not scale as the square roots of the drops.
  Keyed by them, a 0.9 m drop was landed as a 1.4 m one. Key by the drop.
- **A gap measured a frame before touchdown** is that frame's fall:
  4.2 m/s × 1/240 s is 1.8 cm. Measure at the touchdown itself.
- **Feet planted ahead of the hips** (half the braking distance) lean the
  touching-down leg: taken upright, a planted ankle at 1.4 m/s was 3.5 mm
  short.
- **The walker's ground snap hid every drop**: walking off a top, the root
  went down to the floor in the frame it crossed the edge.
- **The roll's height, frame by frame.** Three versions failed:
  - a sphere's height for the tuck put the toes 29 cm under the ground as it
    came up;
  - held up by the ground only where a joint went under, the hips jerked at
    167 m/s²;
  - resting on the lowest joint each frame (hard or soft), they jerked at
    557 and 70 m/s² as the lowest joint changed.
  Joints sweep the bottom at about the roll's speed, so any per-frame
  minimum jerks. Planned over time and smoothed from above, the hips peak at
  36-46 m/s², swinging round the tucked centroid.
- **Tucking at touchdown** from the straight-legged contact shape: the body
  stalled on its feet (5.9 m/s down to 0.1 in a frame). The squat takes the
  impact first.
- **The centroid across touchdown** steps 2.4 m/s: the feet stop dead. Test
  the hips' continuity there.

## Consequences

**Headless** (`puppet_base`; 0.9 and 1.8 m; still and walking off at
1.4 m/s):
- touchdown at free fall's speed (4.2-6.0 m/s), the feet on the ground at
  the instant (0.04 mm), the hips' velocity continuous across it (within
  0.05 m/s);
- landing 0.377 and 0.335 s, knees at most 116° and 126° (122-131° moving
  on);
- 4.2-4.4 body weights at the hips from 0.9 m, 6.5-6.6 from 1.8 m (a parkour
  landing peaks at 3.2 from 0.75 m, a stiff one 5.2; rolling is the
  guidance above standing height);
- planted ankles within 0.4 mm, nothing below the ground, standing exactly
  at the end.

**Headless, rolling** (1.8 and 2.4 m; still and at 1.4 m/s): no joint below
the ground; the hips continuous across touchdown (within 0.05 m/s) and at
most 46 m/s² from the tuck on; the forward speed kept rolling; once round;
standing on its spot. From 4.5 m it does not land, from 3.5 m it does; from
1.2 m it squats, from 2.0 m it rolls.

**Live** (a 0.9 m block, walking off at 1.2 m/s, BRP): the pelvis from
1.84 m to 0.48 at the bottom, the feet planted on the floor, standing again
at 0.94, walking on.

**Cost** (`anim_bench --gait drop --characters 20`, each frame posed from
the start): 16 µs a character a frame (a walk 23).

## Revisit when

- **Rolling from speed**: a fast run off a low drop might roll too; it
  rolls by the drop's height only.
- **A hurting drop** between rolling and fatal (a stumble, a hand down) is
  not modelled.
- **A stride's own lift**: a walker steps off with its foot in the air; the
  fall starts from the root, between the feet.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is part of step 4 of.
- [Parkour movement data](./parkour-movement-data.md) — prerequisite: the landing times, knees and loads this is set from, and the speeds it does not use.
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — context: the jump's own landing (1 g, at most 0.42 m) this generalises to height.
