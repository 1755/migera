---
title: A step aside is the balance's side step and close, asked for rather than pushed
description: "Walker::step_aside takes one deliberate side step and close from a stand: the leading foot's loaded side step (the stumble's), the body heading for its landing, then the other foot joins. Turning back waits for rest between the feet. Read before changing step_aside in balance.rs."
type: decision
status: current
tags:
  - locomotion
  - balance
  - ik
  - correctness
updated: 2026-10-04
verified: 2026-10-04
code:
  - src/character/anim/balance.rs
  - src/character/anim/walker.rs
  - src/character/anim/plugin.rs
  - examples/character_gallery.rs
sources:
  - "tests balance::tests::steps_aside_close_cleanly_and_end_standing_as_it_stood, stepping_aside_turns_back_cleanly; plugin::tests::a_placed_landing_carries_the_free_foot_onto_its_spot_and_locks_it_there"
  - "live: character_gallery --step-aside-at 2:0.25,5:-0.25 and (repeated, before the shuffle) --aside-schedule 1:0.2,7:-0.25,13:0, --step-seconds 0.0166667 on Xvfb, the pelvis and feet over BRP"
aliases:
  - step aside
  - step_aside
  - Walker::step_aside
  - side step
  - step and close
  - walk_aside
  - ASIDE_STEP_MAX
  - Landing::place
  - --step-aside-at
---

# A step aside is the balance's side step and close, asked for rather than pushed

`Walker::step_aside` (metres, positive to the character's left) takes one
side step from a stand: the leading foot steps out, then the other joins
it beside, at the width they stood. A person shifts over so, to stand
square to something or make room. The step is the standing balance's
(`balance::Balance::step_aside`), the same loaded side step a sideways push
gets (see [a stumble is a capture-point step](./a-stumble-is-a-capture-point-step-then-a-join.md)),
asked for instead of forced by a capture point. Walking sideways is not
this but the side shuffle, a gait cycle of its own (see
[walking sideways is a shuffle](./walking-sideways-is-a-shuffle-on-the-walks-clock.md)).

## Context

The balance already had everything one sideways step needs, tested and
live-verified: a foot swing to a planned spot, weight moving onto it
through a spring, the join, the pelvis carried over the feet, the travel
handed to root motion.

## Decision

- **The step**, feet together and no push to catch:
  1. The weight goes onto the trailing foot (`transfer`).
  2. Once that foot alone holds the body, the leading foot swings out the
     length asked (up to `ASIDE_STEP_MAX`, 0.3 m; `ASIDE_STEP_SECONDS`,
     0.3 s).
  3. On landing the weight moves onto it, and the trailing foot joins once
     it alone holds the body: the stumble's own landing and join.
  4. The feet together again, the travel goes to the caller.
- **Through the swing the body heads for the landing.** The COP law's
  rest is the leading foot's landing spot, the pressure pinned on the
  foot it stands on: it pushes off.
- **The other way after a step:** the body first comes to rest between the
  feet (within 2 cm and 2 cm/s).
- **In the walker:** from a stand, not shuffling; a walk asked for
  meanwhile waits until the feet have closed (`WalkerState::stepping_aside`).
- **A balance step's foot is placed on its spot** (`Landing::place`): the
  landing carries the foot across onto the planned spot while it swings,
  before its lock sees it. A stop's landing does not.

## Alternatives considered

- **Walking sideways as these steps repeated** (the first version): a step
  and close took ~1.0–1.25 s, ~0.24 m/s at most, a halting rhythm. The
  side shuffle replaced it for walking.
- **Waiting for the weight to settle over the trailing foot before
  lifting:** the step started with no sideways momentum; repeated, 0.07 m/s
  and a pelvis sunk 163 mm.

## Traps it hit

- **Held back over the stance foot through the swing,** as a recovery
  step is, the body went the wrong way and the pelvis sank 74 mm at
  touchdown, jolting 6.2 mm.
- **Turning back with the body still going** split the legs: 64 mm down.
  Stopped over the new trailing foot instead, the step fell the whole way
  onto the other foot and jolted 5.4 mm.
- **A wait meant for turning back, applied every step,** caught the body
  as the weight first moved onto the trailing foot and jolted 7.8 mm.
- **The sprung foot landed 1.9 cm wide of its spot,** the lock held it
  there, and the leg (a centimetre short of straight, see
  [the rig's critical extension](./rig-authored-at-critical-extension.md))
  left its ankle 18 mm up. Hence `Landing::place`.
- **The feet floated 18 mm after a few steps:** each step's travel reached
  the locks through the live hips' 4° roll. See
  [foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md).
- **The headless balance model passed throughout** both of the last two:
  it has no springs and no locks. They showed only live, in the BRP
  numbers.

## Consequences

- **Model** (`balance::tests`, the real rig, the gallery's loop headless),
  0.2 m steps asked as each closes: 0.200 and −0.187 m/s; planted tips
  fixed within 1 mm; never narrower than they stood; pelvis jolts under
  4.5 mm; sinking under 30 mm; at rest every sole point within 2 mm of
  where it stood under the body. Turning back at four moments: under
  35 mm down, no jolt.
- **Live**, a 0.25 m step left, then one back: 0.245 m out and back to
  within 7 mm, the feet ending where they stood, flat; planted feet within
  0.4 mm; the pelvis at most 35 mm down.

## Revisit when

- **A walk to a chair** could finish a near miss with a step aside instead
  of the seat making it up.

## Related

- [A stumble is a capture-point step, then a join](./a-stumble-is-a-capture-point-step-then-a-join.md) — prerequisite: the side step, landing and join this asks for.
- [A foot may lift only when the other holds the body](./a-foot-may-lift-only-when-the-other-holds-the-body.md) — prerequisite: why each lift waits for `holds`.
- [Walking sideways is a shuffle on the walk's clock](./walking-sideways-is-a-shuffle-on-the-walks-clock.md) — contrast: how the character walks sideways, rather than steps once.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — deeper: the floating feet, and the frame the travel goes through.
