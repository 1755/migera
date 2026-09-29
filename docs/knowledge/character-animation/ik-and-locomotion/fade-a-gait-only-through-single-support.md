---
title: Fade a gait only through single support
description: "A blend weight changing while both feet are down slips one (12.5 mm/frame): the walk fixes their spacing, the blend scales it. Fades run in single support; root motion must include the hips' root_translation; a first swing must be lifted. Read before changing transition.rs or blending a gait."
type: lesson
status: current
tags:
  - locomotion
  - correctness
updated: 2026-09-29
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

## Measured

Headless, stance slip over the first step is held to the steady walk's
over the same stretch of stride. Sabotaged: 73 mm with no hips term, 32 mm
fading into double support, against 16.9 mm steady. Live worst planted
slide: 1.5/2.2 mm at the start, 2.9/1.1 mm at the stop, where before it was
17.6 mm at the start and 73/77 mm at the stop.

## Related

- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — context: the IK-side cause of the same first-step slide.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — prerequisite: the displacement this adds the hips term to.
- [11.3.2 Gait initiation](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.2-gait-initiation.md) — source: the release and first step.
- [11.3.3 Gait termination](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.3-gait-termination.md) — source: the half-length last step.
