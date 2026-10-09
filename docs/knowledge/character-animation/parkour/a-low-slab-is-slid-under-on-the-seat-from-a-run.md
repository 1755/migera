---
title: A low slab is slid under on the seat from a run
description: "Step 10, third part: running at a slab overhead 0.8-1.5 m up, the body drops into a seated slide braked at 0.3 g, legs first and hips after, slides past the slab and rises with its feet planted. It starts where the slide carries the hips 0.5 m beyond the slab. Read before changing parkour/underslide.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/underslide.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::underslide::tests::a_low_slab_is_slid_under_from_a_run"
  - "live: character_gallery --step-seconds 0.0333333 --slab 0,-12,180,0.9,0.8,4 --slide-under-at 0.5 --anim-speed 5.0, Xvfb, gizmos on/mesh off Left, mesh on; BRP capture of pelvis, feet and hands"
  - "anim_bench --features real_rig --gait slide-under --characters 100"
aliases:
  - slide under
  - UnderSlide
  - HangAsk::SlideUnder
---

# A low slab is slid under on the seat from a run

Step 10 of the [parkour design](./parkour-moves-implementation-design.md),
third part: running at an obstacle overhead too low to run under, slide
under it and rise past it.

## Decision

**The slab** is the walker's ledge with a wall below it as thick as the
slab, its underside 0.8-1.5 m up (`underslide::LOWEST`, `HIGHEST`).

**The slide** (`UnderSlide`), on its seat:
- **braking:** friction at 0.3 of gravity (cloth on a hard floor) down to
  1.2 m/s, then a stop over the 0.6 s rise;
- **start:** where that carries the hips 0.5 m past the slab's far side
  (`start_distance`);
- **the drop** (0.42 s, the hips' free fall over 0.7 m taking 0.38): the
  legs go to the slide's shape over 0.3 s, the hips start down 0.1 s in;
- **the shape:** the trunk leant back 0.9 rad, the hips 0.22 m up, the
  lead leg ahead (the foot 0.72 m before the hips), the other bent with its
  knee up, the trailing hand on the floor beside and behind the hips, the
  lead arm forward. Each ankle stays within 0.95 of its leg;
- **the rise:** the slide's shape blends to standing and the hips stand as
  high as its legs put them, the lower foot on the floor;
- **the floor:** between the shapes, a foot under the floor (an ankle under
  5 cm, a toe under 0) is lifted by a softplus of its lack, faded in over
  0.1 s;
- **validation:** the plan samples every frame and refuses if any joint or
  the head's top comes within 3 cm of the slab's underside over its
  footprint, or if it would rise before clearing it.

**The walker** (`HangAsk::SlideUnder`): running at it, the pace adjusted for
a foot to come down at the start. It waits while too slow yet (still
speeding up) and gives up only when 1.5 m short. The slide is posed led
ahead of its springs.

## Alternatives considered

- **Friction at 0.45 g**: a 4 m/s slide carried 1.65 m, so it began 0.65 m
  before a 0.5 m slab and the hands were under it before the body was down.
- **A run at 3.5 m/s**: carries 1.84 m at 0.3 g, too short past any slab
  with the drop's distance; slides start from 4.5 m/s.

## Traps

- **The lead foot's place out of reach early in the drop** (the hips still
  high): the leg straightened and its knee flipped 6.8 cm in a frame.
- **Legs and hips dropping together** from the run's pose put the feet
  34 cm under the floor; a hard lift at the floor made a 4.75 cm kink; a
  lift on from the first frame jumped the knees 7 cm. Legs first, a soft
  lift faded in fast, fixed it.
- **A rise on its own curve**: the legs stood up faster than the hips and
  the feet went 5 cm into the floor.
- **Posed unled live**: the sprung legs trailed the dropping hips and the
  feet went 6 cm into the floor; headless (no springs) it never showed.
- **The ask dropped at the first footfall**: a 5 m/s run was still at
  2.7 m/s, too slow to slide, and ran round the slab instead.
- **A test from a standing pose**: it missed the feet going through the
  floor; it plans and slides from the run's own poses, any phase and foot.

## Consequences

**Headless** (`puppet_base`; 4.5-6 m/s, slabs 0.85-1.1 m up and 0.5-1 m
deep; and from the run's poses at four phases off either foot):
- every slide plans, nothing touches the slab, and it stands 0.86 m past
  it;
- no joint goes over 6.5 m/s about the hips, and no joint's step changes
  over 2.6 cm a frame;
- the ankles stay 3 cm up and the toes no more than 2 cm into the floor.

A 0.6 m slab, and a 1.5 m deep one from 2 m/s, are refused.

**Live**: running at 5 m/s at a 0.9 m slab 0.8 m deep, it dropped 2.4 m
before it, slid under (neck 0.59 m, the trailing hand on the floor), rose
0.7 m past it and ran on. The lowest ankle was 5.8 cm.

**Cost**: `anim_bench --features real_rig --gait slide-under`, 30.5 µs a
character at p50.

## Revisit when

- **A slide that ends in a run** without standing first.
- **A knee slide or a feet-first slide into a gap**: one shape only.

## Related

- [A low obstacle is speed-vaulted as a reshaped running leap](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — contrast: over an obstacle, the same take-off pacing.
- [A near-straight leg is bent toward its kneecap](../ik-and-locomotion/a-near-straight-leg-bends-toward-its-kneecap.md) — same-trap: a leg past its reach straightens and its hinge flips.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — context: led poses against springs.
