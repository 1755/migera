---
title: Feet keep clear of obstacles in the foot IK, as a capsule against a probe
description: "AnimObstacles (a FootObstacles probe, like AnimGround) moves each foot's toe target, and a planted foot's lock, so the heel-to-tip line keeps 8 cm from obstacles. Done in the IK stage, not on the walker's pose (12 cm there, for spring lag); as a line, not points. Read before changing obstacles.rs or foot avoidance."
type: decision
status: current
tags:
  - ik
  - locomotion
  - correctness
updated: 2026-10-03
verified: 2026-10-03
code:
  - src/character/anim/obstacles.rs
  - src/character/anim/plugin.rs
  - src/character/anim/footlock.rs
  - examples/character_gallery.rs
sources:
  - "tests obstacles::tests, plugin::tests::a_planted_foot_is_held_clear_of_an_obstacle_and_the_other_left_alone"
  - "live: character_gallery --sit chair:upright --chair X,Z,HEADING --step-seconds 0.0166667 on Xvfb, each foot (heel to tip, 4.5 cm half-wide) measured over BRP against the gallery chair's leg posts"
aliases:
  - AnimObstacles
  - FootObstacles
  - Footprints
  - Footprint
  - foot_line
  - FOOT_CLEARANCE
  - shift_anchor
  - foot avoidance
  - keep-out
---

# Feet keep clear of obstacles in the foot IK, as a capsule against a probe

A character with `AnimObstacles(Box<dyn FootObstacles>)` keeps each foot's
middle line, heel to tip, `FOOT_CLEARANCE` (8 cm) from every obstacle.
The foot IK moves the toe's target by the probe's answer, and moves a
planted foot's lock with it (`FootLock::shift_anchor`).

## Context

Turning to sit in front of a chair, every placement put a foot ~5 cm into
one of the chair's front legs, and a heel 14 cm under the seat (see
[walking to a chair](./walking-to-a-chair-turns-on-a-circle-and-paces-its-stop.md)).
A first fix in the walker knew only that chair, and needed a 12 cm margin.

## Decision

- **A probe, like the ground:** `FootObstacles::clear(heel, tip,
  clearance) -> Vec3` is a capsule query, the shape a physics world
  answers.
  - `Footprints` (boxes on the floor) is the plain implementation, for
    tests and simple scenes.
  - The gallery gives its character the chair's `Chair::footprint`.
- **In the foot IK** (`solve_foot_ik`), after the lock decides the toe's
  target and before the legs are solved, in the world (as the ground is
  sampled).
  - A swinging foot's target moves; a planted foot's lock moves with it,
    since a turn pivots a planted foot about the body.
- **The move:**
  - Clear of a box, away from it along their nearest points (round its
    corners radially, so it never jumps).
  - Crossing a box, the smallest move along the box's sides or square to
    the foot that separates them.
  - Several boxes' moves are combined each way along each axis, so two on
    either side of a foot cancel rather than one throwing it into the
    other.
- **`FOOT_CLEARANCE` 8 cm** is the foot's half width (4.5 cm), the ~2 cm
  the drawn foot falls short of the IK's toe target (its heel turns as the
  leg solve places the ankle), and a centimetre to spare.

## Alternatives considered

- **In the walker, on the pose it writes** (the first fix). The sprung leg
  trails that pose 4–7 cm swinging fast: at a 5 cm margin a foot still
  passed 4 cm into a leg; at 9 cm, 2 cm. It needed 12 cm, and it knew only
  the chair it was walking to.
- **Raycasts:** a ray samples one line, so a 3.5 cm leg slips between
  rays. A closest-point (capsule) query cannot miss it, and gives the way
  out.
- **Physical feet** (colliders, contacts pushing them out): the foot locks
  and the contact solver would fight, the ragdoll's old trouble. Contact
  response is for stumbles; avoidance stays kinematic.

## Traps it hit

- **Points instead of a line.** A box under the foot's arch had heel and
  toe pushed opposite ways, and the moves cancelled to nothing (the IK test
  caught it: the foot stayed 4 cm from the leg).
- **Out by the nearest side:** the move flipped 20 cm from front to side
  across a corner's diagonal; the foot jerked across and its lock broke.
- **Taking only the deepest point's move:** it flipped as the deepest went
  from heel to tip.
- **Swinging feet only:** the planted ones still swung 4 cm into a leg as
  the body turned about them.
- **5 cm was the foot alone:** the drawn foot came within 3.2 cm of the
  footprint, 1.9 cm into a leg.

## Consequences

- **Live,** four chair placements (one walked round from behind):
  - every foot 3.0–4.1 cm from every chair leg;
  - the drawn foot ≥ 6.2 cm from the footprint;
  - seated hips 0–1 mm from the seat's middle;
  - feet 0 mm of slide sitting, rising and standing after.
- **Tests:** the moves (out, round a corner for a point and for a foot, a
  post under the arch, between two posts), and the IK holding a planted
  foot off a post with the other foot unmoved.
- **The clearance includes the IK's own shortfall**, so it is tied to how
  the leg solve places the ankle, not to walking speed.

## Revisit when

- **A physics world should be the source:** a `FootObstacles` built on
  avian's spatial queries, filtered to foot height (above the floor, below
  a seat), would cover any collider.
- **The heel shortfall is fixed** (the leg solve reaching the ankle the
  pose wants): the clearance can drop toward the foot's half width.
- **Obstacles that move**, or a foot that must step over something: this
  only moves feet sideways.

## Related

- [Walking to a chair turns on a circle and paces its stop](./walking-to-a-chair-turns-on-a-circle-and-paces-its-stop.md) — context: the turn that put feet in the chair's legs.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — deeper: how a planted foot's lock is moved by the body, which this moves too.
- [Sample the ground in the world, not in the pose's frame](./sample-the-ground-in-the-world-not-the-pose.md) — same-trap: obstacles are asked in the world for the same reason.
