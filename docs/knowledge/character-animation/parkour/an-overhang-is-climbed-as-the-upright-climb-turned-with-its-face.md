---
title: An overhang is climbed as the upright climb turned with its face; caught under it, the body swings
description: "Step 15: a leaning face is climbed in its own frame (heights up the face, hips off it along the tilted body, palms on it), so on the lean the upright climb is turned whole; a dyno's catch under it cuts the feet loose into a damped rod pendulum. Read before changing HoldWall::leaning or the climber's swing."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
  - biomechanics
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/character/anim/parkour/holds.rs
  - src/character/anim/parkour/monkey.rs
  - examples/character_gallery.rs
  - examples/anim_bench.rs
sources:
  - "tests parkour::holds::tests::{an_overhang_is_climbed_with_the_feet_on_its_holds, under_an_overhang_the_feet_cut_loose_swing_and_come_back}, parkour::monkey::tests::a_line_of_jugs_under_a_roof_is_crossed_hand_over_hand"
  - "sabotage: the rod 0.4 m longer, the half periods read 1.20 s against 1.065 and the test failed; ELBOW_CLEAR_LEANING at 0.99, an elbow went 1.9 cm into the face and it failed"
  - "live: character_gallery --step-seconds 0.0333333 --overhang 0,-0.6,180,9,7,0.45,6,25.78 --free-climb 1,0,1, Xvfb, BRP; gizmos on/mesh off Left and Back, mesh"
  - "live: character_gallery --step-seconds 0.0333333 --roof-jugs 0,-0.6,0,2.4,0.4,6 --monkey-at 1, Xvfb, BRP; gizmos on/mesh off Left and Back, mesh"
  - "anim_bench --gait overhang / free-climb / monkey --characters 100"
aliases:
  - overhang
  - HoldWall::leaning
  - leaning face
  - cutting loose
  - feet cut loose
  - Swing
  - swing_through
  - roof
  - MonkeyBars::roof
  - JUG_DROP
  - hand over hand under a ceiling
---

# An overhang is climbed as the upright climb turned with its face; caught under it, the body swings

Step 15 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), second part: climbing
under an overhang, the feet cutting loose and coming back, and hand over
hand under a ceiling.

Contents: [Decision](#decision) · [Alternatives](#alternatives-considered) ·
[Traps](#traps) · [Consequences](#consequences) · [Revisit](#revisit-when)

## Decision

**An overhang** (`HoldWall::leaning(from, angle)`) is a wall upright to a
crease at `from` and leaning out `angle` above it. Its holds and top are
carried out with the face.
- A point's distance out is the less of its distances from the two planes
  (`place`). The space in front of the face is where both are in front.
- **Heights are measured up the face** (`rise`): the height itself where
  the face is upright, and along the lean above it.

**The climb is the upright one in the face's frame**, so wherever the
hands and feet are both on the lean, the upright climb is turned whole:
- **The body tilts** its top out by the face's lean between the feet's
  holds and the hands' (`tilt_for`, the chord). It tilts the whole lean
  with all four limbs on the lean, and not at all when hanging free. The
  tilt turns the hips bone, so everything solved after it is turned too.
- **Braced hips** are placed up the face from the feet as upright ones
  are, and off it along the tilted body.
- **The rules in heights** (a foot 0.5 m under the hips, the feet 0.5 m
  under the hands) are measured up the face or along the tilted body.
- **An arm's elbow pole and its lock-off** turn with the body.
- **A palm** lies on the face at its hold, the fingers in the face's
  plane (`frame_at`), eased over a 0.2 m band about the crease.
- **A foot** on a hold on the lean is turned with the face.
- **The elbow limits are tighter** on an overhang, where the face leans
  over the arm: a move is ruled out from cos 0.95 against the pole and
  chosen less from 0.9 (upright 0.99 and 0.95).

**The feet cut loose at a dyno's catch under the overhang.** The catch
hangs the hips plumb under the hands, well out from where they were
braced near the face, so the body swings on out:
- **A damped rod pendulum** about the hands' middle, the wall's along its
  axis (`Swing`): a rod as long as hands to feet hung from one end,
  ω = √(3g/2L), damped at 0.2 of critical.
- **It is set going by the flight's speed** through the catch, so the
  hips carry on without a jolt.
- **What it adds is its own.** It adds to the hips and the body's tilt
  (its top tips in as the hips swing out), and the climb goes on under it.
- **The hands hold still** for half a swing.
- **After a swing and a half** (by then down to 15 %) the free feet are
  brought back up onto the face's holds, the swing fading out over the
  foot's move.

**Hand over hand under a ceiling** is monkey bars on jugs
(`MonkeyBars::roof`):
- a line of jugs under a roof, crossed by the same `Crossing`;
- each held 8 cm under the roof (`JUG_DROP`), while the fingers round one
  reach 3.8 cm over where it is held.

## Alternatives considered

- **Only the tilt**, with the hooks, hips and height rules still against
  an upright face:
  - elbows flipped 70 cm in a frame;
  - a forearm went 3 cm into the lean;
  - the climb stuck on the lean.

  In the face's frame, a 26° overhang with holds all the way is climbed
  without a dyno.
- **Cutting loose whenever only a hanging hand move is left** (no braced
  move, but one with the feet off): it never fired on any wall tried
  (26°, 40°, 52°). Hanging puts the shoulders lower, so a hand reaches
  less, not more. It was removed: cutting loose is the catch's.
- **Swivelling an elbow out of the face** about its shoulder-to-wrist
  line. Every way flipped somewhere, as the [hairy-ball
  limit](./any-wall-is-climbed-on-holds-grown-from-its-roughness.md) says:
  - turning it a share of the way toward the face's normal flipped it
    50 cm where it pointed straight at the face;
  - toward its own side, an arm reaching aside drove it on in;
  - the least turn toward the normal still flipped it 13-30 cm where the
    arm itself pointed at the face.

  Moves are kept out of where the elbow is lost instead.

## Traps

- **Heights left in world height on the lean.** A foot 0.5 m under the
  hips, the feet 0.5 m under the hands: on a 26° face these refused every
  move, and the climb stuck.
- **A pole that stays upright** while the body tilts flipped elbows
  70 cm in a frame.
- **The other hand of a matched pair** jumped 9 cm to the hold's middle as
  its partner left it (a bug in step 13's climber that the overhang's
  dynos showed). It now slides there over its partner's move.
- **Pre-existing, not the overhang's**: got on from 0.6 m off a wall, the
  walk-up's swinging hands go 11 cm into it. An upright wall at the same
  distance does the same (see [walking is kept out of
  walls](./walking-is-kept-out-of-walls.md)).

## Consequences

**Headless** (`puppet_base` with its own fingers' grips, 60 fps). The wall
is upright to 2.3 m in 7 rows of edges 0.3 m apart, with 6 more rows above
leant 26° (0.45 rad), its top a lip.
- **Holds all the way** (`an_overhang_is_climbed_with_the_feet_on_its_holds`):
  - topped out with no dyno and no swing;
  - the body tilted 0.450 rad, the face's;
  - held hands within 1.2 µm, feet within 0.9 µm;
  - nothing into either face, no joint over 4.3 m/s;
  - the largest change of step 2.75 cm, the upright grid's own.
- **A gap of 0.45 m over the crease**
  (`under_an_overhang_the_feet_cut_loose_swing_and_come_back`):
  - **The jump and the catch**: it jumped the gap and was caught with the
    feet cut loose.
  - **The swing, held still**: the feet's line from the hands turned at
    0.47 s, 1.57 s and 2.65 s, half periods 1.10 and 1.08 s. A rod as long
    as the measured hands-to-toes, 1.624 m, gives 1.065 s.
  - **Its decay**: each turn was 0.520 of the last (0.527 at 0.2 of
    critical).
  - **Back on and up**: the feet came back onto holds and it topped out.
    Nothing went into the face.
  - **Its steps**: outside dynos, no step changed over 3.7 cm. A dyno's
    release changes an elbow's 12 cm here and 7 cm upright, its own.
- **A gap of 0.6 m**: jumped twice, it ends hanging with no foothold it
  can use. The lean's lowest are 0.9 m under the hands, too high, and the
  upright wall's are out of reach. Nothing goes into the face on the way.
- **Steeper**, out of what is tested:
  - at 40° a held foot strayed 15 cm;
  - at 52° the head went 3.7 cm into the lean at the crease, the feet on
    the upright part and the hands on the lean.
- **Under a roof**: six jugs 0.4 m apart under a roof 2.4 m up were
  crossed. The held wrists were at 2.237 m at most, a moving wrist no
  higher and every other joint lower, the fingers 4 cm clear of the
  roof. It landed under the last jug.

**Live** (Xvfb, BRP):
- **The overhang** (the gap of 0.45 m): it climbed, swung twice under the
  lean, got its feet back and topped out. The trunk leant 0.05 rad on the
  upright part and 0.35 braced on the lean.
  - From Left and Back, gizmos on with the mesh off: hanging from two
    leaning-face holds, the elbows out, the body plumb under the hands
    and clear of the upright wall below, the legs matched.
  - The mesh agreed.
- **The roof**: crossed to the sixth jug, the wrists at 2.237 m under it.
  Seen Left and Back, and with the mesh: the hands just under the slab.

**Cost** (`anim_bench --characters 100`, per character at p50):
- `--gait overhang`: 0.184 ms, from the catch, swinging and climbing on.
- `--gait free-climb`: 0.109 ms upright (0.108 before).
- Under a roof: the monkey bars' `Crossing`, 0.068 ms (`--gait monkey`).

## Revisit when

- **Overhangs steeper than about 30°**: the held feet and the head at the
  crease above.
- **Feet for a high step** after a dyno (the gap of 0.6 m): a rock-over,
  or the hips let rise further over a high foot.
- **A cut-loose without a dyno**, if a wall needs one: with the hips lower
  hanging, it needs a hold a hanging hand reaches that a braced one does
  not.
- **A roof of holds in two dimensions**, rather than a line of jugs.

## Related

- [A wall of holds is free climbed limb by limb, a gap jumped for](./a-wall-of-holds-is-free-climbed-limb-by-limb-a-gap-jumped-for.md) — prerequisite: the climber this turns with the face, and its dyno whose catch starts the swing.
- [Any wall is climbed on holds grown from its roughness](./any-wall-is-climbed-on-holds-grown-from-its-roughness.md) — same-trap: an elbow lost where its arm points against its pole, here closer to the face.
- [Monkey bars are crossed hand over hand](./monkey-bars-are-crossed-hand-over-hand-the-body-hung-from-the-hands-carrying-it.md) — applies: the crossing that goes hand over hand under a roof.
- [A bar is swung on, pumped, and let go of at a bar ahead](./a-bar-is-swung-on-pumped-and-let-go-of-at-a-bar-ahead.md) — contrast: a free hang's pendulum driven, not left to die away.
- [A window is climbed through crouched on its sill](./a-window-is-climbed-through-crouched-on-its-sill.md) — contrast: step 15's first part.
