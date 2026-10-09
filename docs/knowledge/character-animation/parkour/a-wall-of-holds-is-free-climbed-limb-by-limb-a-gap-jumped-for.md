---
title: A wall of holds is free climbed limb by limb, a gap jumped for
description: "Step 13: a wall of given holds climbed one limb at a time, three held, in a four-beat order for the way asked; the hips placed from the holds; a hold out of reach jumped for (a dyno); stepped off at the bottom, the lip taken into a hang at the top. Read before changing parkour/holds.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/character/anim/parkour/holds.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::holds::tests::{a_wall_of_holds_is_climbed_limb_by_limb_every_way, a_hold_out_of_reach_is_jumped_for, it_steps_off_at_the_bottom_and_tops_out_into_a_hang}"
  - "live: character_gallery --step-seconds 0.0333333 --holds-wall 0,-1.2,180,7,10,3.6 --free-climb 1,0,1, Xvfb, gizmos on/mesh off Left and Back, mesh on; BRP capture of pelvis, hands and feet"
  - "anim_bench --gait free-climb --characters 100"
aliases:
  - free climbing
  - FreeClimb
  - HoldWall
  - Hold
  - dyno
  - climb hop
  - Walker::free_climb
---

# A wall of holds is free climbed limb by limb, a gap jumped for

Step 13 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md):
the core of Assassin's Creed climbing, a facade of hand- and footholds.
The holds are given as values (`HoldWall`), as a `Ledge` is. Finding them
in level geometry is out of scope.

## Decision

**A hold** is its top's middle and a kind: an edge or a jug (hand or
foot), or a foothold only. **A wall** is a flat face, its outward normal,
its holds, and an optional top (a `Ledge`). When it has a top, holds are
added along the lip every 0.1 m.

**A move is one limb to a new hold**, the other three held:
- a hand takes 0.55 s and a foot 0.45 s; each comes 8 cm or 6 cm off the
  wall on its way (`sin²`);
- the order for the way asked is four-beat: up, a hand then the other
  side's foot; down, the feet first; aside, the leading side's hand then
  foot;
- the hold chosen progresses at least 0.1 m the way asked, nearest a
  step's progress (0.4 m a hand, 0.35 m a foot), little across the way,
  and on its own side of the body;
- every limb must still reach its hold from the hips the move leaves.

**The hips follow the holds** (`hips_for`):
- with footholds: 0.75 m over the feet, clamped to 0.65-0.95 m under the
  hands, 0.32 m out from the face;
- with the feet free: 0.9 m under the hands, 0.26 m out;
- the trunk leant 0.15 rad to the wall.

Each held limb is posed exactly:
- a foot stands on its hold, toes down 0.35 rad, its hold at least 0.5 m
  under the hips;
- a knee near the face is turned out, smoothly by its clearance;
- a hand hooks over its edge, the arm kept softly within 0.95-0.99 of its
  length;
- the elbow is held out and back from the wall, pulled down as its hand
  comes down to its shoulder (a lock-off).

**A dyno** (a climb hop) is used when no hand hold is in a move's reach
but one is up to 1.3 m over the higher hand and within 0.5 m aside. It
runs:
1. a 0.25 s sink, no lower than the hands still reach;
2. a 0.2 s drive;
3. a ballistic flight to the catch at its top, both hands matched 9 cm
   either side of the hold.

The feet leave over the first 0.7 of the flight and find holds again
after. A dyno is driven off the feet only: with them free there is none.

**The ends:**
- **Getting on** (1 s): from standing 0.4 m out, the hands swept up round
  the shoulders to the nearest holds over them, the feet lifted to holds.
- **Stepping off**: at the bottom, asked down, with the lower foot's hold
  0.45 m or less over the floor, it hands over to a fall that lands.
- **Topping out**: at the top, asked up, with both hands on lip holds, it
  becomes `Hanging::caught` on the lip, blended in over 0.3 s. The walker
  then asks the hang to climb up.

**The walker** (`Walker::holds`, `Walker::free_climb`): walks to the spot
in front of the wall facing it, gets on once stopped, and climbs the way
asked (zero holds still).

## Alternatives considered

- **The ladder's climber with rungs replaced by holds**: the ladder's
  limbs move on a fixed two-rung cycle up a fixed line. Holds are
  scattered, so each move chooses its limb's hold, and the hips are placed
  from the holds instead of rung by rung.
- **Hands at any height over the hips**: a hand held under 0.65 m over the
  hips passes its shoulder as the body rises, and the elbow flipped.
- **Dyno arms solved to hands flying on a path**: the arm passed through
  straight and an elbow flipped 50 cm in a frame. Instead, the arms' local
  rotations are slerped from the held pose at release to the held pose at
  the catch.

## Traps

- **Getting on searched from the root**, on the floor, found no holds.
  Search from the hips placed out from the face.
- **Hanging free 1.0 m under the hands** left the arms straight, and an
  elbow flipped 22 cm in a frame as it bent. Hang 0.9 m under them.
- **A high foothold folded the knee forward 10 cm into the wall.** The fix
  has two parts: footholds at least 0.5 m under the hips, and the knee
  turned out by its clearance. A knee that could not clear jumped to the
  full turn, 70 cm in a frame; over a 12 cm band, it swung 30 cm in a
  quarter second.
- **A foot took the other side's hold**: the left foot right of the hips,
  the right one 0.75 m out. The fix scores each foot by distance from
  under its own hip.
- **The dyno measured from the shoulders' reach** never jumped a 1.1 m
  gap. Measure it from the higher hand.
- **A sink of 12 cm hanging straight-armed** left a hand 7.4 cm off its
  hold. Sink only as far as the hands still reach.
- **A matched hand let go from its hold's middle** jumped 9 cm; let go
  from where it is.
- **The other hand of a matched pair** jumped 9 cm to the hold's middle as
  its partner left (found 2026-10-10 under an overhang). It slides there
  over its partner's move.
- **The rising body under the dyno's turning arms** carried a hand 5.6 cm
  into the wall. Each arm is turned out about `arm × out`, by
  0.12·sin(πs).
- **Turning an elbow's pole back from the wall** (nearly along an arm
  reaching up for the wall) flipped elbows.
- **Taking the lip at once** moved a joint 25 cm in a frame: the hang's
  arms and legs are its own. Blending over 0.3 s brings it to 3 cm.
- **A hand held at the shoulder with the elbow pole sideways** stuck the
  elbow straight out level with it (a chicken wing; seen live from Back,
  not by any test). Pulling the elbow down as the hand comes within
  0.2 m over the shoulder raised the fastest joint from 1.8 to 2.4 m/s.

## Consequences

**Headless** (`puppet_base`, 60 fps; a 6 m wall of edges 0.4 m across and
0.3 m up), got on from standing, then:
- climbed 1.40 m up in 9 s, 1.18 m and 1.12 m aside in 6 s, (0.63, 0.50) m
  diagonally in 5 s, and 1.33 m down in 9 s;
- held hands stayed within 1.2 µm of their holds and feet within 0.7 µm;
- three limbs were held while a hand moved;
- no joint went over 0.9 mm into the wall;
- no joint moved over 2.4 m/s about the hips;
- no joint's step changed over 1.3 cm in a frame.

**A 1.1 m gap** with footholds through it was jumped for. No hand was held
for 0.37 s; the hands stayed within 1 µm when held. The flight is the
fastest part, 7.3 m/s and a 6 cm kink, neither asserted.

**Climbed down**, it stepped off and landed. **Climbed up**, it took a lip
into a hang with no joint moving over 3 cm in the frame.

**A known limit**: with both hands matched on one hold, the hips are capped
0.65 m under it, so a hand can leave only from feet in a band about 0.18 m
high. On the gap test's wall, the rows above the gap are 0.2 m off the
footholds' 0.3 m spacing, so the feet never land in that band. It goes on
by dynos of one row (0.3 m) each, four in all.

**Live**: walked to a wall 1.2 m ahead, got on, climbed, jumped 0.55 m to
the lip, topped out into the hang and climbed up onto it. Seen Back and
Left mid-wall: hands on holds, feet on holds below the hips, knees turned
out clear of the face.

**Cost**: `anim_bench --gait free-climb --characters 100`, 68 µs a
character at p50. Most is `pose_led` posing a clone per spring lead, as for
the pole (39 µs), the bar swing (63) and the hang (58).

## Revisit when

- **Outside corners** (hands crossing the corner, the body turning about
  it) and **transitions** to a ladder, a pole or a beam end: not built.
- **The matched-hands band** stops a climb on a real wall: let the hips
  rise further over the feet for one move, or a high step (a rock-over).
- **Step 14 places holds itself**: it can avoid the band by construction.
- **Recreational climbing data** for the move times (by eye now; speed
  climbing's 2.5-2.8 hand moves a second is only an upper bound).

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 13's design and what comes after.
- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — contrast: the rung-by-rung climber whose hand grip and shoulder lift this reuses.
- [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — prerequisite: the hang a top-out becomes.
- [A pole is climbed as an inchworm, hands over a leg clamp](./a-pole-is-climbed-as-an-inchworm-hands-over-a-leg-clamp.md) — same-trap: a hand swept round the shoulder, and the cost of posing led clones.
- [A near-straight leg is bent toward its kneecap](../ik-and-locomotion/a-near-straight-leg-bends-toward-its-kneecap.md) — same-trap: a two-bone solve's hinge near straight, here the arms.
- [A hand grip is measured in the hand's own frame](../rig-and-retargeting/a-hand-grip-is-measured-in-the-hands-own-frame.md) — deeper (2026-10-09): the grips were taken unturned, and the hooked hand, kept fingers up and palm flat, bent the wrist 1.7 rad sideways and 1.05 back; it now follows its forearm (`hook_along`).
- [An overhang is climbed as the upright climb turned with its face](./an-overhang-is-climbed-as-the-upright-climb-turned-with-its-face.md) — deeper (2026-10-10): this climber on a leaning face (heights up the face, the body tilted with it, tighter elbow limits) and a dyno's catch under it swinging.
- [Any wall is climbed on holds grown from its roughness](./any-wall-is-climbed-on-holds-grown-from-its-roughness.md) — deeper (2026-10-09): irregular holds found three faults this climber's grid never reached: an elbow flipping where its arm points against its pole (moves are now kept clear of it, `ELBOW_CLEAR`), the fingers' sideways flip (`HOOK_SOFT`), and the get-on sweep into the wall.
