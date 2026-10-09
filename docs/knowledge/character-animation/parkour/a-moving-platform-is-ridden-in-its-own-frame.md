---
title: A moving platform is ridden in its own frame, its velocity handed on at every take-off and landing
description: "Step 18: a walker on a moving platform, or in a jump taken on it, is carried by its displacement each frame; a fall off it lands in its frame or the world with its velocity added; one onto it is planned in its frame. Traps: a fall's first frame. Read before changing parkour/platform.rs or the walker's carry."
type: decision
status: current
tags:
  - locomotion
  - correctness
  - verification
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/platform.rs
  - src/character/anim/parkour/fall.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::platform::tests::{a_platforms_top_is_stood_on_over_it, falls_carry_a_platforms_velocity_and_land_on_it_where_it_is}"
  - "live: character_gallery --step-seconds 0.0333333 --start-height 1.0 --platform 0,0,0,1.0,4,3,2,6 [--anim-speed-schedule 6:1.2 | --jump-at 3:0.35:0.8], Xvfb, BRP pelvis, feet and platform"
  - "live: character_gallery --step-seconds 0.0333333 --start-height 2.0 --block 0,-1,0,2.0,3,4 --platform 0,-2.5,90,1.0,3,2.5,2,6 --anim-speed-schedule 1:1.2, BRP; gizmos on/mesh off Left and Front, mesh Left"
aliases:
  - moving platform
  - Platform
  - Platforms
  - PlatformGround
  - frame_for_fall
  - Walker::platforms
  - riding
---

# A moving platform is ridden in its own frame, its velocity handed on at every take-off and landing

Step 18 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), last part: standing,
walking and jumping on a moving top, and jumping or walking off it with its
velocity carried. The hips' velocity in the world never jumps at a
hand-off.

## Decision

**The app owns the platforms** (`parkour::platform::Platform`). Each frame
it writes where each one's top is, its velocity, and how far it moved
since the frame before. It writes them through a shared handle
(`Platforms`) that two readers hold:
- the walker (`Walker::platforms`);
- the character's ground (`PlatformGround`, wrapping any other probe), so
  the foot IK stands the feet on the platform.

**Riding.** A walker is carried in a platform's frame
(`WalkerState::riding`) in three cases:
- standing or walking on it (its root over the top, within 0.1 m of it);
- in a jump taken on it;
- in a fall that will land on it.

Each frame its root moves by the platform's displacement. So does every
world point a fall keeps (`Falling::shift`). Its own motion goes on top.
The foot locks are not told: their anchors are in the body's frame, so
planted feet go with the platform. The vertical travel the locks see has
the platform's rise taken off, so a rising platform does not drag the
planted feet behind it.

**A fall begun in a frame** (`frame_for_fall`):
- **Off a platform.** If its flight comes down on the platform again (in
  the platform's frame), it stays in that frame. Otherwise it goes into
  the world with the platform's velocity added (`Falling::carry`), and its
  landing is found with the platforms left out (`without_platforms`). The
  ordinary landing then brakes that speed.
- **From the world.** If its flight, at its velocity relative to a
  platform, comes down on that platform where the platform is now, it is
  planned in that platform's frame. Otherwise it stays in the world.

Either way the hips' velocity in the world is unchanged at the hand-off.

**Walking off its edge, riding is kept** until the fall begins the next
frame, so the fall knows the frame it left.

The gallery: `--platform X,Z,HEADING,TOP,LENGTH,WIDTH,AMPLITUDE,PERIOD`, a
slab swinging along its length as a sine.

## Alternatives considered

- **Planning every fall in the world**, with the platform's velocity
  added. A fall back onto the platform it left, or onto one going by, is
  planned against where the platform is when it starts, not where it will
  be, and misses it. In the platform's frame the platform's own geometry
  is still: only the floor moves, and a fall that lands on the floor is in
  the world.
- **Writing the platforms into the ground and re-inserting it each
  frame.** The walker would also need their velocities, and two copies
  drift apart. Hence one shared handle.

## Traps

- **Riding let go the frame the root crossed the edge.** Riding was
  recomputed from the root before the drop was noticed, and the root was
  already off the top. The fall began in the world standing still: the
  platform's 1.7 m/s was lost. Riding is now updated after the drop check
  and kept while a fall is due.
- **The first frame of a walked-off fall**, three ways:
  - A fall begun by walking off starts where the root was before the
    frame's travel, and was not advanced that frame. On static ground the
    body stood still for a frame, a 5.7 cm change of step; the walker had
    always done this. It is now advanced that frame.
  - From a platform into the world, the frame's carry was applied and the
    fall's own velocity (with the platform's in it) moved it again:
    4.5 cm too far.
  - From the world into a platform's frame, neither moved it: 5.8 cm
    short.

  Each fall is now shifted by the new frame's movement less the old
  frame's carry.

## Consequences

**Headless** (`puppet_base`, 60 fps): walking off a platform's end or
side, dropping onto one going by at 1.5 m/s, and a hop back down onto
one.
- The hips' world velocity is the same across the hand-off (within
  1 mm/s).
- The feet land on the platform where it then is, or on the floor past
  it.
- No joint's change of step is larger than the same fall made plainly in
  its frame. That fall's own is 7-11 cm at touchdown when landing while
  moving 1.5-2 m/s across the ground: the fall's landing, unchanged here.

**Live** (Xvfb, BRP at 30 Hz; a platform swinging ±2 m every 6 s, up to
2.1 m/s):
- Standing, the pelvis kept its place on the platform within 7 mm over
  3.5 s, the feet fixed under it.
- Walking off its end at 1.2 m/s, the pelvis's step went 16, 8, 7, 3 mm a
  frame into the fall, with no step.
- A standing jump 0.8 m forward moved 0.76 m along the platform and
  landed on its top.
- Walking off a 2 m block onto it going by, it landed on the platform and
  rode it. The pelvis kept its place on it (−0.019, 0.427 m) through the
  squat and rise, then walked on along it.
- Gizmos from the left and front, and the mesh: a squat landing on the
  wood with both feet on its top, rising on it.

**Cost**: not on `anim_bench`, which times posing; riding changes no
pose. Its per-frame work is a copy of the platform list and one vector
added; a fall's start plans one extra landing per platform.

## Revisit when

- **A jump begun on static ground landing on a level platform** starts
  riding only once landed: the hips take the platform's velocity in a
  frame.
- **A platform that turns**: only displacement is carried, not rotation.
- **Platforms accelerating hard**: a fall in a platform's frame ignores
  the frame's acceleration.
- **Hanging from a moving ledge**, or climbing on one: not carried.

## Related

- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — prerequisite: the fall and landing a platform's velocity is handed to, and its 7-11 cm touchdown while moving.
- [Walking is kept out of walls](./walking-is-kept-out-of-walls.md) — context: the ground probe a platform's block also answers `blocks` for.
- [A springboard is a running leap whose foot rides the board down](./a-springboard-is-a-running-leap-whose-foot-rides-the-board-down.md) — contrast: a moving surface under one foot, posed in the leap rather than carried.
