---
title: A sneak walks the walk's foot path moved by its crouch, the leg solved to it
description: "sneak::sneaking_on walks Winter's stride from a crouch, each ankle on the walk's path moved as the crouch moves it; SneakGait blends crouches on the move by their ankles; aside, the shuffle on the crouch. Traps: angle offsets bobbed the pelvis 72 mm; blended joints sank the foot 52 mm."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-06
verified: 2026-10-06
code:
  - src/character/anim/walk.rs
  - src/character/anim/sneak.rs
  - src/character/anim/gait.rs
  - src/character/anim/walker.rs
  - src/character/anim/shuffle.rs
sources:
  - "Winter 2009 Appendix A (the replayed stride); Steele et al. 2010 (crouch gait classed by the least stance knee flexion)"
  - "tests sneak::tests::a_sneak_walks_on_bent_knees_lower_than_a_walk, a_sneaks_feet_stay_on_the_floor_and_swing_clear_of_it"
  - "live: character_gallery --sneak-schedule 0.5:1 (and 0.5:0.6:toes) --anim-speed-schedule 0:0,3:0.8,10:0, against the plain walk at 0.8 m/s; --step-seconds 0.0166667, Xvfb, BRP every frame"
aliases:
  - sneaking_on
  - CrouchAngles
  - GaitParams::crouch
  - sneak walk
  - FASTEST
  - ARMS_HELD
  - SneakGait
  - crouch change while walking
  - walk to sneak
  - toe touchdown
  - crouched shuffle
  - sneaking aside
  - turning while sneaking
---

# A sneak walks the walk's foot path moved by its crouch, the leg solved to it

A walker asked to sneak (`Walker::sneak`) and to walk crouches first, then
walks from the crouch: `sneak::sneaking_on` gives a measured walk
(`walk::WalkCycle`, Winter's stride) a crouch (`GaitParams::crouch`).

## Decision

**The base pose is the crouch.** The trunk's lean and the arms carried
forward come with it (see
[the crouch](./a-crouch-is-the-jumps-countermovement-held-over-the-feet.md)).
The walk's arm swing composes on top, held back with depth (`ARMS_HELD`,
70 % at the deepest).

**Each leg is solved to the walk's foot path, moved by the crouch.**
`WalkCycle::pose_legs` works in each leg's sagittal plane:
1. It poses the walk's thigh and knee as recorded.
2. It finds where they put the ankle from the hip socket.
3. It moves the ankle by as much as the crouch moves it from standing.
4. It solves the thigh and knee to reach it, the knee forward (law of
   cosines).

The foot keeps the walk's attitude. So:
- At the walk's standing-like moments the leg is the crouch's.
- The hips ride the walk's own path lowered by the crouch: within 0.2 mm,
  flat.
- A swinging foot clears the floor exactly as the walk's does, and planted
  feet sit exactly as the walk's do: 13-15 mm pressed (the foot IK lifts
  them).

**The crouch's extra flexion** (`CrouchAngles`) is read off the crouch
against standing (`gait::sagittal_angles`), not authored. On the toes, the
foot is never flatter than the crouch's heel rise (`CrouchAngles::heel`,
eased in over 3°), so it lands on the forefoot and the heel stays up
through stance.

**On the toes, the whole stance counts toward the thigh correction**
(`WalkCycle::refine`). A walk counts each planted foot's drift by the share
of the body it bears, since a heel not yet loaded is still in the air. A
toe walker's tip is down from touchdown, before it bears the body.
Counted by load, it landed moving 8.9 mm a 60 Hz frame (the recorded
heel's roll carrying the rigid foot's ankle on). Counted whole, it moves at
most 0.11 mm a frame through 97 % of stance. Live at 0.8 m/s, half on the
toes, the tip on the floor moved up to 5.6 mm a frame before and 0.95 after
(the walk's tips: 1.07).

**Speed and stride.**
- At most `FASTEST` (1 m/s). A sneak never runs: asked to sneak while
  running, the run slows to a walk before the crouch begins.
- The walk's stride at the same speed (`GaitParams::walking_on`). Every
  crouch then walks its feet along the same path over the floor, and
  uncrouched the sneak is the walk.

**A crouch changing on the move** (`SneakGait`). The walk at the crouch it
set off from and the walk at the one it is going to share one clock, and
are blended by how far the crouch has gone (`Crouching::gone`). Each is a
cached cycle, so a change builds two at most; one cycle per crouch passed
through would be a build a frame.

The joints are blended, then each leg is put back where the two walks have
it:
- the ankle from the hips, lerped as the hips' height is, keeps the foot
  where both put it over the floor;
- the foot's world attitude is slerped.

Walk to sneak and back is the same thing, a crouch changing to or from
none.

**Turning** needs nothing of its own. Steered while sneaking, the walk's
heading turns as a walk's does, and the sneak's feet go as the walk's.
Standing, a crouch turns on its locked feet as a standing walker does.

**Going aside crouched** is the side shuffle (`shuffle::shuffle_pose`)
posed on the crouch it is in. The shuffle places each foot and solves the
pelvis to keep the loaded legs the base's length, so on a crouch:
- the legs stay as bent as the crouch's;
- the trunk's lean and the arms come with it;
- on the toes, the feet keep the heels up.

It is posed afresh every frame, not a cached cycle, so it takes the crouch
as the crouch changes.

**In the walker.**
- Asked to sneak, it crouches standing, walking or shuffling aside. Asked
  to stop sneaking while moving, it stands up on the move.
- From a stand, it sets off once its crouch is still.
- Asked to sit, jump or step aside, it stands up first.

## Alternatives considered

- **Add the crouch's extra thigh and knee flexion to the walk's angles.**
  This was the first try, and it is the trap below.
- **Give a swinging knee back part of the crouch's flexion** to keep the
  foot's clearance. Giving none, the foot rose 79-110 mm mid-swing; giving
  half, it dipped 7.7 mm into the floor; 0.3 cleared it. Made unneeded by
  solving the leg to the walk's path.

## Trap: a thigh swept far from vertical makes the hips bob

The crouch's thigh sits about 46° forward of vertical. There the hips'
height changes about four times as fast with the thigh's sweep as at a
walk's 10° (sin 46° against sin 10°).

So the walk's stance sweep, offset there, asked the pelvis for a 72 mm bob
a step, against the walk's 32 mm:
- the pelvis accelerated at 6.9 m/s²;
- the toe tips hung low into early swing and skimmed the floor at 26 mm a
  frame.

Solved to the walk's foot path instead, the bob is the walk's.

## Trap: blending two crouches' joints sinks the planted foot

Two walks at different crouches put a foot at the same place over the
floor, but their legs reach it with different joints: a knee at 15° in one,
75° in the other. Slerping the joints while the hips' height lerps does not
keep the ankle on a straight line between them. Half-way from standing to
the deepest, the planted foot was pressed 52 mm into the floor (the walk's
own: 15 mm).

Re-solving each leg to the lerped ankle brings it back to 15 mm, the
walk's, at every point of every change tried.

A test of this that measures the foot's slip cannot fail: root motion is
read off the same blended pose, so the planted contact holds still by
construction. The test measures its height.

## Consequences

**Measured** (`puppet_base`, 0.4-1 m/s, half and deepest, flat and on the
toes):

| | Least stance knee | Hips below the walk's |
|---|---|---|
| Half crouch | 47-52° | 92 mm, the crouch's own |
| Deepest | 65-72° | 181 mm |

On the toes, the hips ride 10-21 mm below the crouch's own height, more
the faster the walk: the walk they are measured against rides up over its
heel. The heel stays 105 mm over the tip through stance.

**Live at 0.8 m/s, deepest flat:**
- the pelvis spans 33 mm at 3.6 m/s² (the walk: 32 mm, 3.3 m/s²);
- planted balls slip at most 0.33 mm a frame and tips 1.07, as the walk's;
- its start and stop match the walk's frame for frame.

**Seen** (gizmos on, mesh off; then the bare mesh, Left):
- Left: the knees forward, the trunk leaning, the arms ahead;
- Front: the legs in their lanes;
- on the toes, both heels up and the forefoot landing.

**Live, changing on the move** (0.8 m/s: walk, then deepest at 5 s, half on
the toes at 9 s, standing at 12 s):
- planted balls slip at most 0.75 mm a frame and toe tips 1.84 mm;
- the pelvis accelerates at most 5.3 m/s², standing up from the toes on the
  move: the crouch's 2 m/s² ease on top of the walk's bob.

**Live, turning and going aside** (deepest, and half on the toes):
- turning on a 0.5 rad/s circle at 0.6 m/s, planted balls slip at most 1.44
  mm a frame and tips 1.74; the upright walk on the same circle, 1.39 and
  1.74;
- shuffling aside at 0.4 m/s each way, the crouch changing to half on the
  toes on the way, every figure is the upright shuffle's on the same
  schedule:
  - the pelvis peaks at 1.0-2.6 m/s² (8.6-10 before the shuffle's pelvis
    was smoothed);
  - tips dragged up to 11 mm a frame where it started and changed side, a
    faded swing's landing, until the shuffle faded over its whole swing
    (see the shuffle's note);
- the hips ride the crouch's own drop below the upright shuffle's.

**Cost** (`anim_bench --gait sneak`):
- 23-27 µs a character a frame for the walk (the walk's 23);
- 51 µs while a crouch changes (`--crouch-from`), two walks and the legs
  re-solved;
- plus up to three crouches posed a frame, about 20 µs each.

## Revisit when

- Many characters sneak at once: cache the crouches posed every frame.

## Related

- [A crouch is the jump's countermovement held over the feet](./a-crouch-is-the-jumps-countermovement-held-over-the-feet.md) — prerequisite: the crouch a sneak walks from.
- [Replay a recorded gait by segment attitudes](./replay-a-recorded-gait-by-segment-attitudes.md) — prerequisite: the walk replayed, thigh and foot by absolute attitudes.
- [Walk pelvis rides one sinusoid per step](./walk-pelvis-rides-one-sinusoid-per-step.md) — context: the pelvis path the sneak's lowered hips ride.
- [Walking sideways is a shuffle on the walk's clock](./walking-sideways-is-a-shuffle-on-the-walks-clock.md) — applies: the shuffle a sneak goes aside in, its pelvis and its open faded-swing landing.
