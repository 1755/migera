---
title: Foot locks need the body's travel
description: "A foot lock works in the pose's frame: pass it all the body's travel, rise included (a foot rode along 14 cm, rose 9 cm up a slope), but not the turn, which that frame already has (a foot flicked 0.55 m a frame turning far from the origin). Read before touching footlock.rs or anything that moves or turns the character."
type: lesson
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/footlock.rs
  - src/character/anim/plugin.rs
  - src/character/anim/walker.rs
sources:
  - "test footlock::tests::a_planted_foot_stays_put_in_the_world_while_the_body_travels_over_it"
aliases:
  - footlock travel
  - Turn::travel
  - locked foot dragged
---

# Foot locks need the body's travel

`FootLock` says it pins a toe to a *world* point, but the points it is fed
(`solve_foot_ik`'s `animated`) are in the pose's frame, which moves with the
character entity. That held while characters walked in place. Once root
motion started moving the entity, two things went wrong:

- **A locked foot rode along with the body.** A foot locked while
  standing stayed at its anchor in the pose's frame as the first step moved
  the entity, which carried it along. The foot then released and eased back
  to the animation, which read live as ~14 cm of slide in the first step.
- **A planted foot never locked mid-walk.** Under a moving body a
  world-still foot moves backward in the pose's frame at walking speed, far
  above `unlock_speed`, so the lock never engaged during a walk.

## The rule

Whatever moves the entity reports how far it moved: `Turn::travel`, in
world axes on `AnimFootIk::turn`. The IK stage rotates it into the pose's
axes (`root_rotation.inverse()`, as for arm targets). The lock subtracts it
from its anchor and adds it back into the speed estimate, so both are
judged in the world. `walker::ride_rendered_feet` and
`advance_turning_with` in `Authoritative` mode fill it in. A new mover that
forgets it gets the old behaviour back.

**All of the travel, rise and fall included.** Until 2026-10-01 the lock
dropped the vertical part and the gallery reported only the horizontal
one, which is harmless on flat ground, where the entity's height never
changes. Walking up a 0.2 grade, the entity rose about 9 cm under each
planted foot, and the foot rose with it while held horizontally. Within a
stance it changed height by 64 mm (median); now 13 mm, as on flat ground
(17 mm, the heel rising). `ride_rendered_feet` reports the rise the ground
gave the entity.

**But not the turn.** The pose's frame already turns with the body, so a
planted foot pivoting with the body (`a_planted_foot_pivots_with_the_body_instead_of_being_dragged`)
stays where it is there. Until 2026-10-02 the IK stage converted only the
travel and passed the turn on as it came, rotating the pose-frame anchor
by each frame's yaw about the character's WORLD position. The anchor
jumped yaw × the distance from the origin. Turning at 2.5 rad/s at a wall
7 m out, a planted foot flicked 0.55 m every frame and the hips 6 cm with
it. Near the origin, as in `character_gallery`'s circle walk, it was
centimetres. Now the IK stage hands the lock no turn: worst per-frame move
of a planted foot through a wall turn, 9.9 mm (the foot's own pivot with
the body), and of the hips, 2.9 mm. The rule is the same as for the
travel: convert every part of `Turn` into the frame the lock's points are
in.

## Measured

Live, start of a walk, worst slide of the planted ball: 17.6 mm → 2.7 mm.
Unit test: zero world slip over 40 frames of acceleration past the unlock
speed. It fails on the first frame (1.1 mm) with the travel removed from the
anchor. `a_planted_foot_stays_put_while_the_body_climbs_over_it` fails on
its first frame (4 mm) with the rise dropped.
`plugin::a_planted_foot_turning_far_from_the_origin_stays_with_the_body`
measured 340 mm in a frame before the turn was dropped, under 5 mm after.

## Related

- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — prerequisite: what moves the entity, and the travel reported here.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — context: the other first-step slide causes found at the same time.
