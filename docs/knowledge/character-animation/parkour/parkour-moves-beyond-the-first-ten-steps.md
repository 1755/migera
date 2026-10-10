---
title: Parkour moves beyond the first ten steps
description: "Design for steps 11-18 of the parkour set, the moves an Assassin's-Creed-style traversal game has past steps 1-10: running agility, more jumps, free climbing on holds, climbing any wall, overhangs and windows, perches, swinging fixtures, slides and long falls. Read before starting any of them."
type: design
status: current
tags:
  - character-animation
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-10
code:
  - src/character/anim/parkour
  - src/character/anim/ladder.rs
  - src/character/anim/run.rs
  - src/character/anim/walker.rs
aliases:
  - parkour steps 11-18
  - free climbing
  - climb anywhere
  - perch
  - corner swing
  - roof slide
  - leap of faith
  - sprint start
---

# Parkour moves beyond the first ten steps

[Steps 1-10](./parkour-moves-implementation-design.md) cover ledges,
landings, mantles, walls, bars and beams. Set against the traversal of the
Assassin's Creed games, Mirror's Edge, Prince of Persia, Dying Light,
Uncharted 4 and Tomb Raider (movement only: no crowds, fighting, horses or
boats), eight more steps are needed. They follow the same
[principles](./parkour-moves-implementation-design.md#principles): plan the
centre of mass and pose onto it, limbs move between holds, nothing through
the geometry, continuous hand-offs, measured then seen.

## Goal

The movement set of an Assassin's-Creed-style traversal game, as
procedural moves on the same `Walker` asks as steps 1-10.

**Out of scope here**, by decision (2026-10-08):
- swimming and anything in water;
- ropes, ziplines, rope launchers, grapples and counterweight lifts;
- trees (V-branches, trunk swings, branch runs);
- the move controller (input to move);
- finding holds in level geometry (holds are given as values, as
  `parkour::Ledge` is);
- edge safety (refusing to run off a fatal drop), which is the
  controller's.

## Design

### Step 11: running agility

- **Model.**
  - **Sprint start**: the first two or three steps from a stand leant
    forward by the acceleration, `tan θ = a/g`, the lean fading as the
    speed comes up.
  - **Leaning into a run's turn**: the whole body tilted toward the centre,
    `tan θ = v²/(r·g)`, the feet placed under the tilted COM.
  - **Reversing from a run**: a skid stop (both feet braking ahead of the
    COM, the trunk leant back), and a 180° plant-and-turn (a braking step,
    the turn over the planted foot, the run back).
  - **A hop past a small obstacle in stride**: the swing foot lifted over
    it, the step lengthened, no break in the run.
  - **A precision landing on a small target** (a post top, a beam end): the
    jump planned onto the point, landed on the balls of the feet, the COM
    held over a support near zero wide (step 10's beam balance).
  - **A hand on a nearby wall**: resting on it when standing beside one,
    brushing past it when walking close.
- **Data.** Turn lean and acceleration lean are mechanics. Obstacle
  crossing in gait has measured toe clearance and step changes (to find).
  The rest by eye.
- **Tests.** The lean matches the formula within 1°; feet planted, no
  slide, through the turn and the skid; the hopped foot clears the
  obstacle; on a post, the COM over the support.

### Step 12: more jumps

- **Model.**
  - Diagonal ejects from a hang: up and aside, back and aside (step 6's
    leap with a combined aim).
  - A jump from a hang or a standing top to a bar or a pole (step 9's
    catch).
  - A springboard off a sprung object (a flagpole's end, a plank): the
    contact adds an impulse.
  - A tuck over an obstacle in the air: the knees drawn up, the COM path
    unchanged.
  - A 180° turning jump from a stand.
  - **Leap of faith**: a swan dive from a height into a soft pile (hay),
    turned to land on the back and sink in.
- **Data.** Ballistic flight; the release window of step 6.
- **Tests.** The flight is ballistic; the catch or landing continuous; the
  tuck clears the obstacle; the dive's body ends inside the pile.

### Step 13: free climbing on holds

The core of Assassin's Creed climbing: a facade of hand- and footholds.

- **Model.** `parkour::Hold` (a point, the wall's normal, a kind: edge,
  jug, foothold only). The ladder's limb-by-limb machinery generalised from
  rungs to holds (`ladder::Climbing`): a move is one limb to a new hold, the
  hips where the feet reach, in four-beat order, up, down, sideways and
  diagonally. A **dyno** (climb hop) when the next hold is out of reach: a
  sink, a drive with the legs, a catch with both hands, step 6's launch on
  a wall. **Braced or free** by whether footholds are in reach. Round an
  **outside corner** of a building, the hands crossing it as step 3's
  shimmy does, the body turning about the corner. **Transitions** between
  holds, a ledge hang, a ladder, a pole and a beam end.
- **Data.** Climbing studies give hand move rates and contact times
  (speed climbing's 2.5-2.8 hand moves a second is an upper bound).
- **Tests.** Held limbs exactly on their holds every frame; at least three
  limbs held while one moves (two in a dyno's flight, none for under
  0.3 s); nothing through the wall; the COM's acceleration bounded across
  each move.

### Step 14: climbing any wall

- **Model.** A rough wall patch (`Wall` with a roughness) given in place of
  holds: each limb's next hold placed procedurally on the patch where the
  limb reaches, at a pace of the climber's own, the climb aimed by a
  direction (the controller's stick later). The hand reach aimed in that
  direction with whole-body IK (Uncharted 4's reach).
- **Tests.** As step 13, on holds the step chooses itself; the same patch
  and direction give the same holds (deterministic).

### Step 15: overhangs and windows

- **Model.** Climbing under an overhang (the feet cut loose and swinging,
  then brought back up onto it; hand over hand under a ceiling).
  Climbing in through a window (sill and lintel: the hands on the sill, a
  leg over, the body through, standing inside) and out of one into a hang.
- **Tests.** Nothing through the lintel or the walls of the opening; the
  free legs' swing a pendulum about the hands.

### Step 16: perches

- **Model.** A **perch**: crouched on a post or a narrow top, the feet
  together, the hands down beside them or on the knees. Onto a perch from
  a hang below (step 2's climb up, onto a top too small to stand on) and by
  a precision jump (step 11). From a perch: a leap (step 6's from a
  crouch), a drop to a hang (step 5). The **viewpoint pose**: perched at a
  high point, rising, looking round.
- **Tests.** The COM over the perch; the feet on it within 1 mm; nothing
  off its edge.

### Step 17: swinging on fixtures

- **Model.**
  - **Corner swing**: running at a building's corner, a hand catches a
    post or lantern on it and the body swings round it on the arm, let go
    on the far side running on.
  - **A flagpole** sticking out of a wall: caught, swung round once or
    twice, let go flung forward (step 9's bar swing, one end free).
  - **Monkey bars**: hand over hand along a line of bars, the body swinging
    (step 3's shimmy on bars, the body a pendulum).
  - **Hanging hooks or pots**: a one-handed catch, a swing, let go.
- **Data.** Step 9's bar swing (release window, flexion timing).
- **Tests.** The swing a pendulum about the grip; the release ballistic; the
  run's pace kept across a corner swing.

### Step 18: slides and long falls

- **Model.**
  - **A roof or slope slide**: down a slope too steep to walk, sliding on
    the feet and a hand, braking with friction; a jump out of it; a catch
    of the edge at its foot.
  - **A steep-surface slide** (a pyramid's face): down on the feet, the
    body leant back.
  - **A long fall's loop**: past the roll's height the arms windmill and
    the legs cycle; past the fatal height the ragdoll at touchdown, as step
    4.
  - **Moving platforms**: standing and walking on a moving top, jumping on
    and off it with its velocity carried.
- **Tests.** The slide's path follows the slope under gravity and friction;
  the feet on the slope; the platform's velocity continuous over the
  hand-off.

## Order and dependencies

| Step | Needs |
|---|---|
| 11 running agility | 4 (landings), 10 (beam balance) for the precision landing |
| 12 more jumps | 6, 9 |
| 13 free climbing on holds | 1, 3, 6 |
| 14 climbing any wall | 13 |
| 15 overhangs and windows | 2, 13 |
| 16 perches | 2, 5, 6, 11 |
| 17 swinging on fixtures | 9 |
| 18 slides and long falls | 4, 5 |

Suggested order after step 10: 13, then 11, 16, 12, 17, 18, 14, 15. Free
climbing is the heart of the style; perches and agility make it feel right.

## Open questions

- **Holds as values.** Steps 13-16 take holds and patches as given, the
  way `Ledge` is. A pass that finds them in level geometry is out of scope
  here, and its interface should match these values.
- **Climbing anywhere** may want the active ragdoll's arms for a reach
  that misses.

## Status

- **Step 13** built (2026-10-09):
  [free climbing on holds](./a-wall-of-holds-is-free-climbed-limb-by-limb-a-gap-jumped-for.md).
  Outside corners and the transitions to a ladder, pole or beam end are
  not built.
- **Step 11** built (2026-10-09):
  [leaning with acceleration](./a-run-leans-whole-into-a-turn-and-its-trunk-with-its-speed.md),
  [skid stops and plant-and-turns](./a-skid-stop-slides-side-on-and-rises-over-stuck-feet.md),
  [hops in stride](./a-small-obstacle-is-hopped-as-a-running-leap-its-feet-lifted.md),
  [precision jumps](./a-precision-jump-is-a-standing-jump-handed-to-a-fall-at-its-top.md)
  and [a hand on a wall](./a-hand-rests-on-a-wall-beside-the-body.md). Not
  built: the whole-body pitch of a sprint start (it trunk-pitches only), a
  plant-and-turn running on without standing, a landing on the balls of
  the feet.
- **Step 16** built (2026-10-09):
  [perches](./a-perch-is-the-get-ups-squat-feet-together-forearms-on-knees.md)
  and looking round. Not built: onto a perch from a hang below, a leap
  out of the crouch itself (it rises first), a drop from a perch to a
  hang.
- **Step 12** built (2026-10-09): diagonal ejects and jumps to a bar
  ([leaps from a hang](./a-hang-is-leapt-from-up-aside-or-back.md)), a
  jump to a pole ([the pole's
  catch](./a-pole-is-climbed-as-an-inchworm-hands-over-a-leg-clamp.md)),
  [a turning jump](./a-turning-jump-is-a-standing-jump-handed-to-a-spinning-fall.md),
  [the leap of faith](./a-leap-of-faith-is-a-ballistic-swan-dive-flipped-onto-the-back.md),
  [a tuck over an obstacle](./a-small-obstacle-is-hopped-as-a-running-leap-its-feet-lifted.md)
  and [a springboard](./a-springboard-is-a-running-leap-whose-foot-rides-the-board-down.md).
  Not built: a jump from a standing top to a bar (a leap from a hang
  only), a springboard's leap running on after it lands.
- **Step 17** built (2026-10-09):
  [monkey bars](./monkey-bars-are-crossed-hand-over-hand-the-body-hung-from-the-hands-carrying-it.md),
  [a flagpole and hooks](./a-flagpole-is-swung-round-as-a-driven-compound-pendulum.md),
  [a corner swing](./a-corner-post-is-swung-round-as-a-running-leap-bent-round-it.md).
  Not built: a corner swing past a quarter turn, a flagpole's release
  aimed at a ledge.
- **Step 18** built (2026-10-09):
  [roof and steep-face slides](./a-steep-slope-is-slid-down-on-the-feet-as-a-block-with-friction.md),
  [a long fall's loop](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md),
  [moving platforms](./a-moving-platform-is-ridden-in-its-own-frame.md).
  Not built: landing on a slope from a fall, steering on a slope, a
  turning platform.
- **Step 14** built (2026-10-09):
  [any wall climbed on holds grown from its roughness](./any-wall-is-climbed-on-holds-grown-from-its-roughness.md).
  Not built: the reach aimed with whole-body IK (Uncharted 4's); the
  step-13 climber's own choice of hold stands in for it.
- **Step 15** built (2026-10-10):
  [windows, in and out](./a-window-is-climbed-through-crouched-on-its-sill.md);
  [overhangs, the feet cut loose and swinging, and hand over hand under a
  roof](./an-overhang-is-climbed-as-the-upright-climb-turned-with-its-face.md).
  Overhangs are climbed from 26° to 52° over gaps up to 0.6 m, and bent
  over to 65°, 78° and a flat roof of holds over 0.3 m. Not built: a
  French window, a roof over a wider gap without an elbow flipping at its
  lip, a roof's lip turned onto a headwall, a cut-loose without a dyno.

All eight steps are built (2026-10-10). What each left unbuilt is listed
above and in its notes.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: steps 1-10 and the principles these steps follow.
- [Parkour movement data](./parkour-movement-data.md) — deeper: measured numbers each step takes its timings from, and the gaps.
- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — prerequisite: the holds model step 13 generalises from rungs to holds.
- [A wall of holds is free climbed limb by limb, a gap jumped for](./a-wall-of-holds-is-free-climbed-limb-by-limb-a-gap-jumped-for.md) — deeper: step 13 as built.
- [A springboard is a running leap whose take-off foot rides the board down](./a-springboard-is-a-running-leap-whose-foot-rides-the-board-down.md) — deeper: step 12's last part as built.
