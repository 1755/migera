---
title: Foot locks need the body's travel
description: "A foot lock works in the pose's frame; with root motion moving the entity, a locked foot rode along (~14 cm first-step slide) and a planted one never locked mid-walk. Pass the body's travel (Turn::travel). Read before touching footlock.rs or anything that moves the character entity."
type: lesson
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/footlock.rs
  - src/character/anim/plugin.rs
  - examples/character_gallery.rs
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
judged in the world. `ride_rendered_feet` in the gallery and
`advance_turning_with` in `Authoritative` mode fill it in. A new mover that
forgets it gets the old behaviour back.

## Measured

Live, start of a walk, worst slide of the planted ball: 17.6 mm → 2.7 mm.
Unit test: zero world slip over 40 frames of acceleration past the unlock
speed. It fails on the first frame (1.1 mm) with the travel removed from the
anchor.

## Related

- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — prerequisite: what moves the entity, and the travel reported here.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — context: the other first-step slide causes found at the same time.
