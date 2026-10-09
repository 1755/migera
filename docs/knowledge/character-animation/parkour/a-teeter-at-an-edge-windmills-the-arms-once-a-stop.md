---
title: A teeter at an edge windmills the arms, once a stop
description: "Step 10, second part: brought to a stand with the ground 0.3 m ahead over 0.5 m lower, the walker teeters once a stop. For 1.6 s both arms circle twice up in front, and the trunk rocks. The layer is eased so the standing pose is untouched at both ends and the feet stay planted. Read before changing parkour/teeter.rs."
type: decision
status: current
tags:
  - locomotion
  - balance
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/teeter.rs
  - src/character/anim/walker.rs
sources:
  - "tests parkour::teeter::tests::{an_edge_is_a_drop_just_ahead, a_teeter_windmills_the_arms_and_settles}"
  - "live: character_gallery --step-seconds 0.0333333 --block 0,2.75,180,2.0,1.2,3.0 --start-height 2.0, Xvfb, gizmos on/mesh off Left and Front, mesh on; BRP capture of the hands"
aliases:
  - teeter
  - windmill
  - at_edge
---

# A teeter at an edge windmills the arms, once a stop

Step 10 of the [parkour design](./parkour-moves-implementation-design.md),
second part: brought to a stand at a drop, the body teeters and recovers.

## Decision

**An edge** (`teeter::at_edge`): the ground 0.3 m ahead of the root is more
than 0.5 m lower than under it (or there is none). A step down is not an
edge.

**A teeter** (`teeter::teeter`, a layer on the standing pose, 1.6 s):
- both arms circle twice, up in front and down behind, eased in and out so
  they hang at either end. They go round 0.35 rad out from the body at the
  middle of the teeter, so they clear the trunk;
- the trunk rocks back and forth 0.25 rad at 1.25 Hz under a `sin(πs)`
  envelope;
- the feet stay planted (the standing pose's).

**The walker** teeters once a stop: standing at rest, not crouched, sitting
or on holds, at an edge. Walking again resets it.

No data: the circles and the rock are by eye.

## Alternatives considered

- **Arms swung back and forth** instead of circling: reads as a shrug, not
  a recovery.
- **A teeter on every frame at an edge**: it would loop for as long as it
  stood there.

## Consequences

**Headless**: the pose is exactly standing at 0 and 1.6 s (within 1 µm).
The hands go round above the head (1.85 m against the head's 1.60). No
joint goes over 5.7 m/s about the hips, and no joint's step changes over
1.7 cm a frame.

**Live**: standing on a 2 m block, its edge 0.25 m ahead, it teetered at once
(the hands circling in front and behind, up to head height and above) and
settled, the feet planted 0.25 m from the edge.

**Cost**: a few quaternion turns on the arms and spine, within the walk's
cost.

## Revisit when

- **A walk carried toward an edge** (edge safety, the move controller's):
  only a stop already at an edge teeters.
- **A teeter that falls**: it always recovers.

## Related

- [A beam is walked with the feet near its line and the arms out](./a-beam-is-walked-with-the-feet-near-its-line-and-the-arms-out.md) — contrast: step 10's other balance layer.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — context: walking on over the edge falls instead.
