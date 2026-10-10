---
title: An overhang is climbed as the upright climb turned with its face; caught under it, the body swings
description: "Step 15: a leaning face, 26° to a flat roof, is climbed in its own frame (heights up it from any point, hips off it along the tilted body, kept clear at the crease), the upright climb turned; a dyno's catch under it cuts the feet loose into a damped rod pendulum. Read before changing HoldWall::leaning/bent or the swing."
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
  - "tests parkour::holds::tests::{an_overhang_is_climbed_with_the_feet_on_its_holds, under_an_overhang_the_feet_cut_loose_swing_and_come_back, steep_overhangs_and_wide_gaps_are_climbed, steeper_overhangs_and_roofs_are_climbed}, parkour::monkey::tests::a_line_of_jugs_under_a_roof_is_crossed_hand_over_hand"
  - "sabotage: the rod 0.4 m longer, the half periods read 1.20 s against 1.065 and the test failed; no crease clearance, or the release always a third of the way, and the steep test failed; heights read as world height, the roof test failed (steps of 8-10 cm)"
  - "live: character_gallery --step-seconds 0.0333333 --overhang 0,-0.6,180,9,7,0.3,6,90,1 --free-climb 1,0,1, Xvfb, BRP; gizmos on/mesh off Left, Back and Right, mesh"
  - "live: character_gallery --overhang 0,-0.6,180,5,7,0.6,6,51.57 (and 9 columns) --free-climb 1,0,1, Xvfb, BRP; gizmos on/mesh off Left and Back, mesh"
  - "live: character_gallery --step-seconds 0.0333333 --overhang 0,-0.6,180,9,7,0.45,6,25.78 --free-climb 1,0,1, Xvfb, BRP; gizmos on/mesh off Left and Back, mesh"
  - "live: character_gallery --step-seconds 0.0333333 --roof-jugs 0,-0.6,0,2.4,0.4,6 --monkey-at 1, Xvfb, BRP; gizmos on/mesh off Left and Back, mesh"
  - "anim_bench --gait overhang / free-climb / monkey --characters 100"
aliases:
  - overhang
  - HoldWall::leaning
  - HoldWall::bent
  - HoldWall::rise
  - leaning face
  - roof climbing
  - limb_face
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

**An overhang** is a wall upright to a crease at `from` and leaning out
`angle` above it, up to a flat roof's quarter turn. `HoldWall::leaning`
carries its holds and top straight out with the face (under 90°, rows
spread up it as it steepens); `HoldWall::bent` folds them over, as far up
the face as they were (to a roof, its top's lip then a free hang).
- A point's distance out is the less of its distances from the two planes
  (`place`, `level·cos a − (y − from)·sin a` for the lean, finite at 90°).
  The space in front of the face is where both are in front.
- **Every measure is up the face** (`rise(point)`): a point's height where
  the face is upright, and its distance along the lean above. Off the
  face, it is the nearer face's nearest point's, blended over 0.1 m where
  the two are as near. Progress up, the feet under the hands, a dyno's
  reach, the top's lip and the body's tilt (the chord between the hands'
  and feet's holds, `atan2(Δout, Δy)`, a quarter turn under a roof) all
  read it. Heights in world height leave a roof's holds all as high.
- **Out of the face is the face's normal**: a move's arc off it, a
  dyno's hands kept off it, the crease clearance (between the two faces'
  normals, which takes the body off either alike).

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
  plane (`frame_at`), eased over a 0.2 m band about the crease. A forearm
  pointing nearly into the face has no way along it: under 0.7 of its
  length along the face, it is eased toward the face's way up
  (`HOOK_FLAT`). The fingers follow the forearm up to 2.2 rad off the
  face's way up (upright, 1.4).
- **A foot** on a hold on the lean is turned with the face, but only
  within 0.15 m of it and not at all past 0.45 m (a hanging foot is not).
- **A moving limb turns from its old hold's face to its new one's** over
  the move (`limb_face`), and a dyno's leaving feet from their holds' to
  the hanging feet's, not by where they are.
- **At the crease**, the face above leaning out over a body still
  half on the upright part, the braced hips are moved straight out until
  the head, chest and shoulders are as far off the face as on a flat face
  (`clear_of_the_face`). On one flat face, upright or leaning, that is no
  move at all.
- **The elbow limits are tighter** on an overhang, where the face leans
  over the arm: a move is ruled out from cos 0.975 against the pole and
  chosen less from 0.92 (upright 0.99 and 0.95). An arm already past the
  limit (both hands caught on one hold) may stay as far.
- **A hand's move whose arm passes near its pole** takes longer, up to
  twice as long at the limit, from 0.85, so the elbow sweeps round slowly.
- **A move that turns the body far** (the feet brought back onto a steep
  face) takes as long as turning it at 0.8 rad/s does.

**The feet cut loose at a dyno's catch under the overhang.** A climber on
a steep face catches with the body still in under the hold, and gravity
swings it out:
- **The catch** is on the hang's circle about the hold, at the angle the
  body comes up from, at most 0.5 rad in from plumb, tilted as its line
  from the hold leans. Its hold is the nearest out of reach (up and aside)
  from which the feet can get back onto a hold, hanging on it.
- **The flight** lasts at least 0.2 s and crosses at no more than
  2.5 m/s, falling onto a catch below the release if it must (along a
  roof).
- **The drive** keeps the body's tilt and lets go no farther than the
  held feet still reach. The tilt turns to the catch's in the flight.
- **A damped rod pendulum** then swings it about the hands' middle, the
  wall's along its axis (`Swing`): a rod as long as hands to feet hung from
  one end, ω = √(3g/2L), damped at 0.2 of critical. It is let go from the
  catch's angle at the flight's speed across it, and the body's own turn
  eases in from the catch's tilt over 0.3 s.
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
- **The elbow limit at 0.95** (set against a 1.9 cm elbow before the
  crease clearance existed) refused the feet's way back onto a 52° face:
  tilting the body 0.9 rad onto it takes a held arm to 0.956-0.964. At
  0.97, measured up the face, a 40° climb over a 0.6 m gap stuck on a
  foothold at 0.971; at 0.975 nothing goes into the face on any wall tried.
- **The crease clearance straight out**, by `short / cos(lean)`: off a
  roof that is no move at all.
- **Finer elbow checks (16 samples) and moves slowed up to 3×** near the
  pole: they halved the roof's elbow flip (14 to 6.8 cm), no more, and
  took a 78° climb's held hand 1.9 mm off its hold. Not kept.
- **Catching at the hang's rest**, plumb under the hold: under a 52° lean
  the hold is a metre out from the braced hips, and the flight left at
  4 m/s; a joint whipped round at 18 m/s and the feet changed step 8 cm at
  the catch.

## Traps

- **Heights left in world height on the lean.** A foot 0.5 m under the
  hips, the feet 0.5 m under the hands: on a 26° face these refused every
  move, and the climb stuck.
- **A pole that stays upright** while the body tilts flipped elbows
  70 cm in a frame.
- **The other hand of a matched pair** jumped 9 cm to the hold's middle as
  its partner left it (a bug in step 13's climber that the overhang's
  dynos showed). It now slides there over its partner's move.
- **The body placed off the upright face at the crease** put the head
  8 cm into a 52° lean above it.
- **A dyno's tilt let go from its start** untilted the body while the
  feet were still on, and a held foot was pulled 22 cm off its hold; so
  did a release a third of the way to the catch, the legs stretched past
  their length (0.917 m against 0.888).
- **The swing's turn put on at once** at the catch changed the feet's
  step 10 cm.
- **A forearm a fifth along the face** (pointing into a 40° face) swung
  its fingers from one side to the other in two frames, the wrist 5 cm.
- **Measuring a face as endless**: a wrist past the end of a narrow wall
  read 7.6 cm "into" it live.
- **A rise or normal taken from one face or the other** off the face: a
  hanging foot turned with whichever was nearer and changed its step
  17 cm; a hand turned out by the normal, 16 cm. Blend over a band.
- **A limb turned with the face by where it is**: a foot crossing a roof's
  crease turned a quarter turn over 20 cm (16 cm steps), and footholds
  just under a crease carried up through it in a dyno, 10 cm.
- **A dyno flown up to a catch below its release** (`rise.max(0.05)`):
  along a roof, a 0.1 s flight crossed a metre at 10 m/s and the swing
  carried the body up through the roof.
- **A dyno's target chosen by aside alone**, or the nearest: under a 52°
  lean it hung where the only foothold far enough under the hands (1.15 m
  up the face: 0.65 to the hips, 0.5 more to the feet) was across the gap.

## Consequences

**Headless** (`puppet_base` with its own fingers' grips, 60 fps). The wall
is upright to 2.3 m in 7 rows of edges 0.3 m apart, with 6 more rows above
leant 26° (0.45 rad), its top a lip.
- **Holds all the way** (`an_overhang_is_climbed_with_the_feet_on_its_holds`):
  - topped out with no dyno and no swing;
  - the body tilted 0.450 rad, the face's;
  - held hands within 1 µm, feet within 0.9 µm;
  - nothing into either face, no joint over 3.9 m/s;
  - the largest change of step 1.9 cm.
- **A gap of 0.45 m over the crease**
  (`under_an_overhang_the_feet_cut_loose_swing_and_come_back`):
  - **The jump and the catch**: it jumped the gap and was caught in from
    plumb, the feet cut loose.
  - **The swing, held still**: the feet's line from the hands turned at
    1.05 s, 2.13 s and 3.22 s, half periods 1.083 and 1.083 s. A rod as
    long as the measured hands-to-toes, 1.623 m, gives 1.065 s.
  - **Its decay**: each turn was 0.526 of the last (0.527 at 0.2 of
    critical); half periods 1.083 s against a 1.605 m rod's 1.059.
  - **Back on and up**: the feet came back onto holds and it topped out.
    Nothing went into the face.
  - **Its steps**: none changed over 2.4 cm, dynos included.
- **26°, 40° and 52° over gaps of 0.3 and 0.6 m**
  (`steep_overhangs_and_wide_gaps_are_climbed`): every one topped out, by
  dynos and swings (up to six at 52°).
  - held hands within 1.4 µm, feet within 0.9 µm; nothing into either face;
  - no step over 3.4 cm, dynos included (they had 10 cm);
  - a held wrist bent at most 0.41 rad sideways (0.70 before).
- **Bent over to 65°, 78° and a flat roof, a 0.3 m gap**
  (`steeper_overhangs_and_roofs_are_climbed`): each topped out into a hang
  on its lip. Under the roof the body lay flat (tilt 1.5708), the feet on
  its holds. Held hands within 0.9 mm, feet within 0.8 µm; nothing into
  either face; no step over 3.4 cm; a wrist bent at most 0.33 rad.
- **Sabotaged**: without the crease clearance, two walls stuck and 52°
  went 7.5 cm into the face; with the release a third of the way always,
  held feet came 9 cm off at 52°; with heights in world height, 78° and
  the roof stepped 8-10 cm. Each fails its test.
- **Under a roof**: six jugs 0.4 m apart under a roof 2.4 m up were
  crossed. The held wrists were at 2.237 m at most, a moving wrist no
  higher and every other joint lower, the fingers 4 cm clear of the
  roof. It landed under the last jug.

**Live** (Xvfb, BRP):
- **26° over a 0.45 m gap**: it swung twice under the lean, got its feet
  back and topped out. Left and Back, gizmos on with the mesh off, and the
  mesh: hanging from two leaning-face holds, the elbows out, the body
  plumb under the hands and clear of the upright wall below.
- **52° over a 0.6 m gap** (`--overhang ...,51.57`): up onto its top, the
  pelvis to 5.49 m, the trunk 0.77-0.80 rad braced and swung to −0.73, a
  wrist 1.6 cm off the face at nearest; from the left, and the mesh.
- **The jugs under a roof**: crossed to the sixth jug, the wrists at
  2.237 m, just under the slab (Left, Back, mesh).
- **A flat roof of holds** (`--overhang 0,-0.6,180,9,7,0.3,6,90,1`): it
  climbed out along the roof and topped out on its lip; until the hang
  nothing came within 1.7 cm of either face. From the right, gizmos on
  with the mesh off: the body flat under the roof, the arms up to it ahead,
  the legs folded up to its holds behind; the mesh agreed.

**Cost** (`anim_bench --characters 100`, per character at p50):
- `--gait overhang`: 0.187 ms over 10 s from the first catch (0.138 with
  two arm passes, from a fixed 12.6 s that no longer reached the catch).
- `--gait free-climb`: 0.136 ms upright (0.110 with two arm passes).
- Under a roof: the monkey bars' `Crossing`, 0.068 ms (`--gait monkey`).

## Revisit when

- **A roof over a 0.6 m gap**: it tops out, but near the lip a held elbow
  goes 1.7 cm up into the roof and a hand reaching 0.7 m aside under its
  shoulder flips its elbow (14 cm). The arm sweeps near its pole's
  reverse (estimated 0.91, under the 0.975 limit), the hairy-ball case.
- **A roof's lip turned onto a headwall above it**: one crease only.
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
