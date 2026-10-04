---
title: Walking aside is the balance's side step and close, asked for rather than pushed
description: "Walker::aside walks a standing character sideways as step-and-close: the leading foot's loaded side step (the stumble's), the body heading for its landing, then the other foot joins. ~0.2 m/s, feet never crossing. Read before changing walk_aside in balance.rs or adding sideways or strafing motion."
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
  - "tests balance::tests::walking_aside_side_steps_and_closes_cleanly_and_stops_standing_as_it_stood, walking_aside_turns_back_cleanly; plugin::tests::a_placed_landing_carries_the_free_foot_onto_its_spot_and_locks_it_there"
  - "live: character_gallery --aside-schedule 1:0.2,7:-0.25,13:0 --step-seconds 0.0166667 on Xvfb, the pelvis and feet over BRP"
aliases:
  - walk aside
  - walk_aside
  - Walker::aside
  - side step
  - sidestep
  - strafe
  - step and close
  - aside_step
  - ASIDE_STEP_MAX
  - ASIDE_SECONDS
  - Landing::place
  - --aside-schedule
---

# Walking aside is the balance's side step and close, asked for rather than pushed

`Walker::aside` (m/s, positive to the character's left) walks a standing
character sideways. It takes a side step with the leading foot, then the
other foot joins it beside, at the width they stood. That is how a person
moves a short way along a table or between chairs. The steps are the
standing balance's (`balance::Balance::walk_aside`), the same loaded side
step a sideways push gets (see [a stumble is a capture-point step](./a-stumble-is-a-capture-point-step-then-a-join.md)),
asked for instead of forced by a capture point.

## Context

The walk is a recorded sagittal stride (Winter's): thigh, knee and ankle
angles about the side axis. It has no frontal-plane leg motion at all, and
there is no recorded sideways stride to replay. The balance already had
everything a sideways step needs, tested and live-verified: a foot swing
to a planned spot, weight moving onto it through a spring, the join, the
pelvis carried over the feet, the travel handed to root motion.

## Decision

- **The cycle**, feet together and no push to catch:
  1. The weight goes onto the trailing foot (`transfer`).
  2. Once that foot alone holds the body, the leading foot swings out
     `aside_step` (`ASIDE_STEP_SECONDS`, 0.3 s).
  3. On landing, the weight moves onto it, and the trailing foot joins
     once it alone holds the body: the stumble's own landing and join.
  4. The feet together again, the travel goes to the caller.
- **Through the swing the body heads for the landing.** The COP law's
  rest is the leading foot's landing spot, the pressure pinned on the
  foot it stands on: it pushes off.
- **Step length from speed:** `|speed| × ASIDE_SECONDS` (1.0 s), clamped
  0.1–`ASIDE_STEP_MAX` (0.3 m), well inside a recovery side step's 0.4 m.
- **Turning back:** the body first comes to rest between the feet
  (within 2 cm and 2 cm/s), then starts as from a stand.
- **Stopping:** the step under way finishes and closes.
- **In the walker:** only standing, at rest, asked neither to walk nor to
  sit. A walk asked for meanwhile waits until the feet have closed
  (`WalkerState::stepping_aside`).
- **A balance step's foot is placed on its spot** (`Landing::place`): the
  landing carries the foot across onto the planned spot by its strength,
  while it swings, before its lock sees it. A stop's landing does not.

## Alternatives considered

- **Sideways leg curves in the gait** (hip abduction through stance and
  swing): no recorded data to replay, and every rig-facing and IK trap the
  sagittal stride already went through would be met again.
- **The forward walk with the trunk turned 90°:** a crab walk, not a side
  step; and the pelvis cannot turn that far over the legs.
- **Waiting for the weight to settle over the trailing foot before the
  first lift:** 0.07 m/s instead of 0.2, and the pelvis sank 163 mm. The
  step started with no sideways momentum.

## Traps it hit

- **Held back over the stance foot through the swing,** as a recovery
  step is, the first step's body went the wrong way and the pelvis sank
  74 mm at touchdown, jolting 6.2 mm.
- **Turning back with the body still going** split the legs: 64 mm down.
  Stopped over the new trailing foot instead, the next step fell the whole
  way onto the other foot and jolted 5.4 mm.
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

- **Model** (`balance::tests`, the real rig, the gallery's loop headless):
  0.200 and −0.187 m/s asked 0.2; planted tips fixed within 1 mm; never
  narrower than they stood; pelvis jolts under 4.5 mm; sinking under
  30 mm; at rest every sole point within 2 mm of where it stood under the
  body. Turning back at four moments: under 35 mm down, no jolt.
- **Live** (gallery, 0.2 left, then 0.25 right, then stop):
  - 0.18 and 0.23 m/s;
  - the pelvis at most 38 mm down (104 before the turn-back fix);
  - planted feet within 5.3 mm (the idle alone shows 5.0 on this check);
  - the feet never narrower than they stood, ending side by side at their
    standing height.
- **Seen** mid-step, Front and Left, gizmos then the mesh: the leading
  leg out to the side, the stance leg upright, knees bent forward, feet
  apart and straight.
- **Slow:** a step and close takes ~1.0–1.25 s (the swing, the weight,
  the 0.45 s join), so ~0.24 m/s at the 0.3 m step.

## Revisit when

- **Faster sideways motion is wanted:** a quicker join, or longer steps
  with a deeper stance; or, beyond ~0.5 m/s, a gallop-like shuffle.
- **Walking and stepping aside at once** (strafing while walking): this
  only steps from a stand.
- **A walk to a chair** could finish a near miss with a side step instead
  of the seat making it up.
- **The standing idle's weight shift** moves the pelvis ~5 cm after ~14 s
  in the gallery; it does not know about a stance changed by side steps.

## Related

- [A stumble is a capture-point step, then a join](./a-stumble-is-a-capture-point-step-then-a-join.md) — prerequisite: the side step, landing and join this asks for.
- [A foot may lift only when the other holds the body](./a-foot-may-lift-only-when-the-other-holds-the-body.md) — prerequisite: why each lift waits for `holds`.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — deeper: the floating feet, and the frame the travel goes through.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — context: standing, only the balance's travel moves the character.
