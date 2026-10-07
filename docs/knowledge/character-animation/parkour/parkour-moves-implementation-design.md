---
title: Parkour moves, step by step
description: "Design for a Prince-of-Persia-style parkour set (ledges, mantles, landings, walls, bars, poles, ropes, beams, tight spaces) as procedural animation on character::anim: shared foundations, then ten steps, each with model, data, API, tests and done criteria. Read before starting or changing a parkour move."
type: design
status: draft
tags:
  - character-animation
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-07
code:
  - src/character/anim/ladder.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - src/character/anim/hand.rs
  - src/character/anim/armik.rs
aliases:
  - parkour
  - platformer moves
  - ledge hang
  - climb up
  - shimmy
  - wall run
  - vault
  - mantle
---

# Parkour moves, step by step

Contents: [Goal](#goal) · [Principles](#principles) · [Step 0](#step-0-foundations) ·
[Steps 1-10](#steps) · [Order and dependencies](#order-and-dependencies) ·
[Open questions](#open-questions) · [Status](#status)

## Goal

The animations a Prince-of-Persia-style platformer needs, without
fighting, as procedural moves in `character::anim`, posed on a measured
rig against the level's own geometry: ledges, mantles and vaults,
landings from height, walls, bars, poles and ropes, beams and tight
spaces.

**Animations only.** Choosing a move from player input and the geometry
ahead (the *move controller*) comes later. Until then each move is asked
for explicitly on the `Walker` (as `Walker::ladder` and `Walker::climb`
are), with the geometry to act on, and driven in `character_gallery` by a
schedule and panel buttons. A move refuses (returns `None`, keeps doing
what it does) when its geometry is out of reach; it never guesses.

## Principles

Taken from what worked for the jump and the ladder:

- **Plan the centre of mass, pose onto it.** A move is planned once as the
  COM's (or the hips') path under the forces a person can apply, with
  timings from measured movements; each frame poses the body's shape and
  solves the pelvis onto the path (`jump::Jump`). Airborne, the path is a
  parabola under `g` and nothing the limbs do changes it.
- **Limbs move between holds.** Hands and feet hold geometry (a ledge's
  edge, a bar, a wall, a beam, the floor); a move is a sequence of steps,
  each moving limbs between holds in windows, IK'd to them
  (`ladder::Climbing`). Held limbs are exactly on their holds every frame.
- **The body keeps clear of the geometry by construction**, and tests
  measure it: no limb segment through a wall, edge, bar or beam.
- **Every bone is led ahead of its spring** (`pose_led`), and the trunk is
  never moved fast while arms are solved for it (the ladder's fists went
  15-33 cm through the rungs when it did).
- **Hand-offs are continuous**: a move starts from the state the last one
  left (position *and* velocity), and returns the body standing, still, at
  the foot IK's pelvis drop.
- **Measured, then seen**: headless tests on `puppet_base` per move, then
  Left/Front/Back with `--gizmos on --show-real-mesh off`, then the bare
  mesh; cost from `anim_bench --gait <move>`.

## Step 0: foundations

Done once, before or alongside step 1, each piece only when its first
user needs it.

**0a. Geometry** (`parkour/geometry.rs`). Plain values in the world, like
`ladder::Ladder`:
- `Ledge`: the top edge as a polyline of corners (so shimmying can turn
  round them), the wall's outward normal on each segment, the top
  surface's depth, and how far the wall goes down below the edge
  (`wall_below`: none for an overhang or a beam end, so the hang is free).
- `Wall` (a vertical patch: base, normal, width, height), `Bar` (a
  horizontal segment and its radius), `Pole` (vertical), `Rope` (anchor,
  length; it swings), `Beam` (a narrow walkway), `Gap` (a low ceiling).
- Each gives the ground under it to the foot IK as `LadderGround` does
  (`ParkourGround`), so a platform reached is stood on.

**0b. Holds shared with the ladder.** The ladder's hand machinery moves
where the second user can call it, ladder tests unchanged:
`shoulder_lift` (to `armik`), the pole that turns from one hold's to the
next, the forearm-roll hand turn, the led pose. A **ledge grip** joins the
round bar's (`hand::gripped`): the palm flat on the top, the fingers hooked
over the lip.

**0c. The airborne body** (`parkour/air.rs`). A COM with a velocity under
`g`, the shape posed in flight (arms reaching for a hold, legs tucked or
reaching for a landing), handed to whatever catches it: a hang, a landing,
a wall. Steps 4-6 and 8-9 share it; step 1 uses the jump's own flight.

**0d. Walker integration.** One `WalkerState::parkour` slot, as
`climbing`: the root rides the hips, the facing square to the geometry,
`AnimFootIk::legs_free` and `off_floor` while off the floor, the look on
the neck and head only, the idle's arm/chest oscillators faded while the
hands hold, grips handed to `RelaxedHands`.

**0e. Verification.** A headless harness per move (as `ladder::tests`):
holds held, clearances to the geometry, COM and joint accelerations bounded
across hand-offs, led poses through nothing. A gallery scene builder per
geometry and an `anim_bench --gait` per move.

## Steps

Each step lists its model, the data to take its timings and shapes from,
its API, its tests, and when it is done.

### Step 1: grab a ledge and hang

- **Model.** From a standing jump (later from any airborne body, step 5):
  the jump is planned so the hands meet the edge on the way up, near the
  apex, the arms reaching for it through the flight. At contact the COM
  keeps its velocity and the arms take it: a spring-damper on the
  grip-to-hips distance (elbows and shoulders giving), and a pendulum about
  the grip for the swing. **Braced** (a wall below): the feet swing in and
  plant on the wall, knees bent, hips out. **Free** (no wall): the legs
  dangle and the body swings to rest. Then it holds: hands on the edge
  with the ledge grip, arms near straight, the shoulders lifted.
- **Data** ([movement data](./parkour-movement-data.md)). No direct data on
  the catch or a braced hang. The reach is ballistic (a rail grab: 185 ms,
  2.3 m/s at the wrist); the free hang is a compound pendulum of about
  2.4 s whose damping is the hanger's own. The catch's give is set from the
  rig's limits and checked by eye, the gap recorded.
- **API.** `Walker::ledge: Option<Ledge>` (the edge to act on);
  `Walker::hang: Option<HangAsk>` with `HangAsk::Grab` (step 1), later
  `ClimbUp`, `Shimmy(dir)`, `LetGo`, `Jump(dir)`. It walks to the spot
  under the edge first (the ladder's approach).
- **Tests.** Hands on the edge within 1 mm while hanging; no part of the
  body through the wall or the ledge; the catch's COM deceleration under
  the measured bound; the swing settles; braced feet planted within 1 mm.
- **Done when** a walker asked to grab a ledge in a standing jump's reach
  (1.9-2.35 m on `puppet_base`) walks under it, jumps, catches it braced or
  free, and hangs still, seen from Left and Back.
- **Built** 2026-10-07: [a ledge is caught near the top of a jump](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md).

### Step 2: climb up onto the ledge

- **Model.** From the hang: pull (elbows flex, COM rises toward the
  hands), transition (wrists rotate over the edge, hands from hook to
  press), press (arms straighten, COM over the edge), a knee or foot onto
  the top, stand up. Braced, the feet walk up the wall through the pull.
  The root's travel onto the top is root motion; the ground becomes the
  ledge's top.
- **Data.** No climb-up timings; the pull-up's elbow range (93-101°,
  Youdas et al. 2010) and a kip's added 49° hip and 57° knee flexion as
  proxies.
- **Tests.** Hands held through pull and press; the knee and foot clear
  the edge; standing still on the top at the end, at the pelvis drop.
- **Done when** it climbs from either hang onto the top and stands.
- **Built** 2026-10-07: [a hang is climbed up from by a pull, a press and a step on](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md).

### Step 3: shimmy and turn corners

- **Model.** Hand over hand along the edge (lead hand, then trail hand,
  never crossing), the hips following; braced feet stepping along the
  wall in turn, free legs swinging a little. At an outside corner the lead
  hand crosses onto the next segment and the body turns 90° about the
  corner; an inside corner the same, inward.
- **Data.** No hanging-traverse data; speed climbing's 2.5-2.8 hand moves
  a second is an upper bound only.
- **Tests.** Never fewer than one hand held, both while the body moves;
  the hips' acceleration bounded; turning a corner, nothing through the
  corner's walls.
- **Built** 2026-10-07: [a ledge is shimmied hand over hand, and round corners](./a-ledge-is-shimmied-hand-over-hand-and-round-corners.md).

### Step 4: landing from height, and falling off an edge

- **Model.** The airborne body (0c) meets the floor at any speed. Below a
  first height: the jump's landing, generalised to the touchdown speed
  (deep crouch, deceleration capped). Above it: a roll (forward, over a
  shoulder diagonally, the COM's horizontal speed carried through). Above
  a fatal or hurting height: hand the body to the ragdoll at touchdown
  (`AnimRagdoll`'s fall). Walking or running off an edge starts the
  airborne body from the gait's own COM velocity.
- **Data.** The firmest: from 0.9, 1.8 and 2.7 m the feet meet the floor at
  3.0, 4.9 and 6.3 m/s; a squat landing lasts 377-290 ms, the knees
  20-29° at contact and 116-134° at most; a roll 380-320 ms, keeping the
  forward speed (Dai et al. 2020). A parkour landing peaks at 2.9-3.2 body
  weights against 5.2 for a stiff one (Puddle and Maulder 2013). Roll above
  about standing height (guidance, not peer reviewed).
- **Tests.** Continuity of COM velocity at touchdown; peak deceleration
  under the landing's bound; the roll's body clear of the floor; a fatal
  drop ends in the ragdoll, not a landing.

### Step 5: drop to a hang, let go, and catch from a fall

- **Model.** Standing at an edge: turn the back to it, crouch, hands onto
  the edge, lower the body down the wall into the hang (step 2 backward,
  slower, eccentric). Let go: the hang hands its COM to the airborne body,
  which lands (step 4). Falling past an edge in reach (a drop, a wall run
  ending, a missed jump), the hands catch it (step 1's catch from the
  airborne body).
- **Tests.** As steps 1, 2 and 4, across the hand-offs.

### Step 6: jumps from a hang

- **Model.** Up to a higher ledge (a pull and release, the hands reaching
  up, caught as step 1); back off the wall (push off with the feet,
  turning 180° in the air, landing or catching); sideways to the next
  ledge along (a swing and lateral release).
- **Data.** A bar release works within a 73-157 ms window (Hiley and
  Yeadon 2003); the flight is ballistic.
- **Tests.** The flight is ballistic; the catch or landing continuous.

### Step 7: mantle and vault

- **Model.** Mantle onto a waist- to chest-high block from a stand or a
  walk: hands onto the top, a hop with the arms pressing, a knee or foot
  up, stand. Vault over a low obstacle while running: the speed vault
  (one hand, legs swung to the side) and the lazy vault; the run resumes on
  the far side (as the leap runs on).
- **Data.** Thin: a kong vault's take-off loads the feet 1.2 body weights,
  the hands 0.3 (unverified summary); no speed or lazy vault numbers.
- **Tests.** Nothing through the obstacle; the run's pace kept across.

### Step 8: walls

- **Model.** Wall run along a wall (two to four steps on it, the body
  leant off it, the COM's path a rising then falling arc); wall run up
  (two or three steps up, then a catch at the top, step 1, or a push
  away); wall jump (one foot's contact on a wall, a push off at an angle,
  to another wall or across a gap, chaining); wall slide down (hands and
  feet braking against the wall).
- **Data.** A wall climb comes in at 4.7 m/s, its last ground step 1.17 m
  out, its first wall step 1.0 m up (Croft et al. 2019); one foot on the
  wall for about 0.37 s adds a third to the jump's height (a thesis). No
  tic-tac data: use one rising contact of about 0.37 s.
- **Tests.** Feet planted on the wall during contact; the flight between
  ballistic; the body leant clear of the wall.

### Step 9: bars, poles and ropes

- **Model.** A horizontal bar: hang, swing (a driven pendulum, the legs
  and hips pumping), release into the airborne body at a chosen angle.
  A vertical pole: climb it hand over hand, legs gripping; spin round it.
  A rope: climb as the pole, and swing it (a pendulum carrying the body,
  pumped).
- **Data.** Hips and shoulders flex just after the bottom of a swing and
  extend just before the top (Yeadon and Hiley 2000); a rope climbed at
  about 3 s a metre (unverified summary); no pole data.
- **Tests.** Hands on the bar or rope throughout; energy only from the
  pump; the release ballistic.

### Step 10: balance and tight spaces

- **Model.** Beam walk (narrow steps on the line, arms out, the COM's
  sway corrected through the arms and trunk, the balance pendulum with a
  near-zero base of support); teeter at an edge (arms windmilling, hips
  back, the COM brought back over the feet); squeeze sideways along a wall
  (the side shuffle, back or chest to the wall); crawl on hands and knees
  under a low gap, and slide under one from a run.
- **Data.** On beams 10-6 cm wide, 0.82-0.69 m/s; with free arms the
  shoulders swing through 92-118° and the trunk bends sideways 61-100°
  (Lambrich et al. 2025). Crawling on hands and knees at 0.28-0.69 m/s,
  four-beat or diagonal pairs when slow (Ma et al. 2017). Shoulders turn
  into a gap narrower than 1.3 shoulder widths (Warren and Whang 1987).
- **Tests.** Feet on the beam; the COM over the support; nothing through
  the low ceiling.

## Order and dependencies

| Step | Needs |
|---|---|
| 1 grab and hang | 0a ledge, 0b grips, 0d |
| 2 climb up | 1 |
| 3 shimmy and corners | 1 |
| 4 landing from height | 0c |
| 5 drop to hang, let go, catch from a fall | 1, 2, 4 |
| 6 jumps from a hang | 1, 4, 5 |
| 7 mantle and vault | 2, 4 |
| 8 walls | 1, 4 |
| 9 bars, poles, ropes | 0c, 1, 4 |
| 10 balance and tight spaces | 4 |

## Open questions

- **The rig's arms are about 17% short for its legs** (0.50 m to the wrist
  against 0.89 m legs), which already shaped the ladder: hangs and pulls
  will reach less than a person's. Whether to lengthen the arms (a rig
  change) is open.
- **Physics.** Moves are kinematic, as the ladder; the ragdoll takes over
  only for falls. Whether the hang and swings should be driven through the
  active ragdoll instead is open.
- **The move controller's interface**: the `Walker` asks are a stand-in.

## Status

- **Step 0**, as far as step 1 needs it: `parkour::Ledge` (0a); the shoulder
  lift, hand turn and frame turn moved to `armik`, and the ledge grip
  (`hand::hooked`, `RelaxedHands::hook`) (0b); `WalkerState::on_holds` and
  the hang's slot (0d).
- **Step 1**, grab and hang: built 2026-10-07.
- **Step 2**, climb up: built 2026-10-07; the ledge's top is the walker's
  ground (`parkour::LedgeGround`, 0a).
- **Step 3**, shimmy and corners: built 2026-10-07. A ledge stays one
  straight segment; corners are other ledges meeting its end
  (`Ledge::joined`, `Ledge::block`, `Walker::ledges`), not a polyline.
- **Step 4**, landing from height: built 2026-10-07 ([the
  note](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md)):
  falling off an edge, the squat landing, the roll past 1.7 m, the ragdoll
  past 4 m. The airborne body (0c) is `parkour::Falling`'s flight.
- **Step 5**, drop to a hang, let go, catch: built 2026-10-08 ([the
  note](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md)):
  the climb-up run backward, slower; letting go into `Falling`; catching
  from any fall, swept over the frame. A missed `Jump` does not catch yet.
- Steps 6-10: not started.

## Related

- [Parkour movement data](./parkour-movement-data.md) — deeper: every step's measured numbers, their sources, and the gaps.
- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — prerequisite: the holds model, grips, shoulder lift, hips' bow and led poses steps 1-3 build on.
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — prerequisite: the COM-path planning and landing that steps 1, 4 and 6 extend.
- [A fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md) — applies: the ragdoll hand-off a fatal drop (step 4) uses.
- [An antipodal guard must still land on the target](../ik-and-locomotion/an-antipodal-guard-must-still-land-on-the-target.md) — same-trap: overhead arm aims in a hang sit near opposite to a hanging arm.
