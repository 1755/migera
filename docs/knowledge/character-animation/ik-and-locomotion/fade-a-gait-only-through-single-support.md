---
title: Fade a gait only through single support
description: "A blend weight changing in double support slips a foot (12.5 mm/frame). Fades run in single support; root motion includes the hips' root_translation; a first swing is lifted; a last swing is set down by the foot IK on the rendered foot (springs lag it ~10 cm). Read before changing transition.rs or blending a gait."
type: lesson
status: current
tags:
  - locomotion
  - correctness
updated: 2026-09-30
verified: 2026-09-29
code:
  - src/character/anim/transition.rs
  - src/character/anim/locomotion.rs
sources:
  - "test transition::tests::the_first_step_keeps_the_stance_foot_planted"
  - "test transition::tests::the_first_swing_lifts_before_it_travels"
aliases:
  - start fade
  - stop fade
  - double support blend
---

# Fade a gait only through single support

A start or stop blends the walking pose with a standing one by a weight
that changes over time. Three things decide whether the planted foot holds.

## 1. Only one foot may be down while the weight changes

With both feet planted, the walk holds the distance between them. A blend
at weight `w` scales that distance by roughly `w`, so while `w` changes, one
of the two planted feet has to slip. Measured headless: up to 12.5 mm a
frame at the end of a first step that faded into double support. Root motion
cannot help, because it can satisfy only one foot.

So `Transition` places both fades in single support (`TransitionConfig::fade`,
`0.5 − duty/2` of a stride):

- **First step:** from the swinging leg's mid-swing to its heel contact.
- **Last step:** after the footfall, wait out the double support, then fade
  from the other foot's toe-off to its mid-swing. The feet end side by side.

## 2. Root motion must see the hips move

Contacts are measured relative to the hips (`Sole::points`), so a pose that
moves its hips through `root_translation` moves every contact with them,
unseen. The walk only moves them vertically. The release before a first
step shifts them 4.5 cm sideways and 4 cm forward, and fading that out
walked the planted foot 47 mm headless (~13 cm live).
`root_displacement_between` adds the horizontal `root_translation` change.

## 3. A first swing has to be lifted

Mid-swing, where the first step joins, is where a walking foot is lowest
(~1.5 cm, Winter) and fastest. Blended with a standing foot it skimmed
94 mm along the floor before rising 2 mm. Reweighting joints does not help:
a leading knee pointed the toe 2 cm into the floor, and a leading whole leg
still skimmed 9 cm, because the walk itself is lowest there.
`Transition::blend` holds the swinging toe ≥ 5 cm above its standing spot
early in the fade and solves the leg to it.

## 4. A last swing has to be set down, on the RENDERED foot

The last step fades a whole swing into standing: the foot leaves the floor
~0.66 m behind its spot and arrives ~0.4 s later. Blended, it came down on
the way: 74 mm short at 5 mm into the floor. Live, the foot IK slid it
along the floor to its spot, the stop's "glide" (up to 27 mm within 5 mm of
the floor).

A lift on the target alone was not enough. The legs' 0.015 s springs lag a
foot moving at 2–3 m/s by over 100 mm. When the target reached its spot,
the rendered foot was still 38 mm behind at 6 mm up, and crept the rest in
along the floor.

So the landing is judged in the foot IK, which sees the sprung pose:

- `Transition::landing` publishes the swinging foot and its standing spot
  (`AnimFootIk::landing`), through the fade and for `LAND_HOLD` (0.25 s)
  after it.
- The IK holds that toe up by `landing_lift(distance)`, measured on the
  rendered foot: 3 cm, eased out over the last 12 cm.
- The lift is shaped `x(2 − x)`, not a smoothstep. A smoothstep is also
  flat at the floor, and the ball crept its last ~18 mm within 3 mm of it.

The target keeps the same lift too, so the target pose never goes into the
floor.

## Measured

Headless, stance slip over the first step is held to the steady walk's
over the same stretch of stride. Sabotaged: 73 mm with no hips term, 32 mm
fading into double support, against 16.9 mm steady. Live worst planted
slide: 1.5/2.2 mm at the start, 2.9/1.1 mm at the stop, where before it was
17.6 mm at the start and 73/77 mm at the stop.

Stop landing, live at 1.2 m/s (2026-09-30): the last swing's ball travels
2.9 mm (`puppet_base`) and 2.7 mm (`character.glb`) while within 2 mm of
the floor, where before it travelled 9.4 and 14.9 mm. It comes down onto
its spot (28.6 mm to go at 19 mm up, then 6.9 at 5.2) and settles within
0.2 mm, with no pop when the hold lets go. Tests:
`transition::tests::the_last_swing_is_set_down_onto_its_spot`,
`..::a_stop_publishes_its_landing_through_the_fade_and_a_hold_after`,
`plugin::tests::a_landing_foot_is_held_up_until_it_is_over_its_spot`.

## Related

- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — context: the IK-side cause of the same first-step slide.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — prerequisite: the displacement this adds the hips term to.
- [11.3.2 Gait initiation](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.2-gait-initiation.md) — source: the release and first step.
- [11.3.3 Gait termination](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.3-gait-termination.md) — source: the half-length last step.
