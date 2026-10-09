---
title: A bar is swung on, pumped, and let go of at a bar ahead
description: "Step 9, first part: a bar is a ledge 4 cm deep with nothing below, so its hang is free. The swing is the free hang's compound pendulum with an energy pump to 1.2 rad (a resonant push starts it from still). The trunk turns whole about the hips, the hands roll round the bar, and the legs pike and arch. A lache waits in its window for the moment needing the least change to catch the bar ahead. Read before changing parkour/hang/swing.rs or a free hang's pose."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/hang/swing.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/parkour/geometry.rs
  - src/character/anim/jump.rs
  - src/character/anim/ground.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::hang::tests::a_bar_swing_is_pumped_up_and_let_go_of"
  - "test walker::tests::walking_goes_round_a_wall (the bar cases)"
  - "gap sim: a compound pendulum (hips 1.09 m from the grip, gyration 0.5 m) released anywhere in the window, the least velocity change that brings the shoulders to within the catch's reach of a bar ahead, per amplitude"
  - "live: character_gallery --step-seconds 0.0333333 --bar 0,-2,180,2.3,16 --bar 0,-4,180,2.3,16 --hang-at 1 --lache-at 6, Xvfb, gizmos on/mesh off Front and Left, mesh on; BRP capture of pelvis and hands"
  - "anim_bench --gait bar-swing / lache --characters 100"
  - "Yeadon and Hiley 2000, Hum Mov Sci 19:153; Hiley and Yeadon 2003, J Biomech 36:313"
aliases:
  - bar swing
  - lache
  - Ledge::bar
  - Hanging::pump
  - Hanging::swing_release
  - Hanging::lache
  - HangAsk::Swing
  - HangAsk::Lache
---

# A bar is swung on, pumped, and let go of at a bar ahead

Step 9 of the [parkour design](./parkour-moves-implementation-design.md),
first part: hang from a horizontal bar, pump a swing up, and let go
swinging forward to land or to catch another bar or ledge ahead (a lache).
The pole is the second part.

## Decision

**A bar is a ledge** (`Ledge::bar`): an edge `BAR_DEPTH` (0.04 m) deep with
no wall below. Everything a ledge has comes with it: grabbing it from a
jump, catching it from a fall, shimmying along it. Its hang is free.

**The swing is the free hang's compound pendulum** about the grip
(gyration 0.5 m, a period of about 2.4 s). Asked to pump
(`Hanging::pump`), the hanger's damping is replaced by a drive on the
pendulum's energy toward a swing `AMPLITUDE` (1.2 rad) out:
`PUMP_RATE · (1 - E/E_wanted) · dθ`. From still, that drive has nothing to
grow from, so a push at the swing's own rate (0.15 g) starts it, fading out
once the swing has a fifth of its energy. It reaches 1.2 rad within 8 s.

**The body swings as one line from the grip:**
- **The trunk turns whole about the hips** (`jump::upper_shared` with the
  pelvis taking all of the lean), leant along the line to the grip.
- **The hands roll round the bar** with the swing (`Hanging::bar_roll`),
  about the lip point the body swings about.
- **The legs pike ahead** through the bottom swinging forward (up to
  0.6 rad) and **arch back** swinging back (0.3 rad), scaled by the swing's
  rate (Yeadon and Hiley 2000: hips flex just after the bottom, extend just
  before the top). The shape eases in and out over 0.4 s.

**Letting go** (`Hanging::swing_release`) only happens swinging forward,
between 0.3 and 0.85 of the swing ahead of straight below. The hands let go
over 0.1 s (a bar release's 73-157 ms window, Hiley and Yeadon 2003),
through the leap's `Launch`, then fly as a `Falling`.

**At a bar or ledge ahead** (a lache): the flight aims the shoulders to
come down past its lip at one of five points well within the catch's reach
(0.9 of the arm). The velocity used is the one, over flights of 0.05-1.2 s
arriving falling, that differs least from the swing's own. It lets go only
when that change is 1.5 m/s or less, and only if no later moment in the
window needs 0.05 m/s less, so it waits through the swing for the best
moment. `Hanging::lache` picks the nearest bar or ledge ahead among the
hang's others (0.8-3 m ahead, no more than 0.5 m higher). With none ahead,
it lets go to land once the swing is pumped to 0.9 of its amplitude.

**The walker** asks `HangAsk::Swing` (pump while asked) and `HangAsk::Lache`
(pump and let go when the moment comes). A braced hang drops the lache. A
walker asked either before hanging grabs first, as for every hang ask.

**Bars are walked under.** The walker's wall probes asked the ground from
100 m up, where a bar's top reads as a 2.3 m wall. They now ask
`GroundProbe::blocks(point, low, high)`, whether anything is solid there
between a step up and the body's `HEADROOM` (2 m). `LedgeGround` answers it
from each block's own extent: top down to its wall, a bar its thickness.
The default answer is the old one.

## Alternatives considered

- **An amplitude of 0.8 rad**, a modest swing: in the gap sim no bar level
  with the one held was within a 1.5 m/s change from 1.2 m ahead on. At
  1.2 rad, bars level with it 1.2-2 m ahead are within 1 m/s, and bars
  0.5 m lower out to 2.4 m.
- **The leap's catch point** (the shoulders 8 cm under the lip at the top
  of the flight): from a swing that needed a 4 m/s change. A falling catch
  takes a bar anywhere within reach above the shoulders.
- **Letting go at the first moment in the window**: low and early, the
  flight rose at 0.5 m/s and missed.
- **A drive on the swing's angle (a forced pendulum)**: it fights the
  swing off resonance; driving the energy cannot.

## Traps

- **A lean shared between the pelvis and the spine bends the trunk.** It
  shortened 2.7 cm at a 0.8 rad lean. The shoulders lagged the line to the
  grip by 0.13 rad and the arms stretched 14 % past their length: a hand
  7.9 cm off the bar. Solving the lean so that the shoulders' direction
  matched the line only moved the shortening elsewhere. Turning the trunk
  whole kept the grip-to-clavicle distance at 0.547 m at every angle.
- **Hands hooked fixed on a bar as on a lip** stay behind it while the body
  swings round: 7 % more reach at 0.75 rad.
- **Rolling the hands about the bar's middle** while the body swings about
  the lip point 2 cm off it left 7 mm. About the lip point: 1.3 µm.
- **A release metric of a joint's move about the hips** read the swing's
  own 3 cm a frame as a jump. The step against the step before (the leap
  test's) is the measure.
- **A 2.4 m bar is out of a standing jump's reach** (2.35 m grabs): the
  walker dropped the ask silently. The gallery's bars are 2.3 m.

## Consequences

**Headless** (`puppet_base`, 60 fps; bars 2.3, 2.45 and 2.6 m up):
- pumped to 1.1997 rad in 8 s;
- the hands within 1.3 µm of their place on the bar through the swing;
- no joint over 3.3 m/s about the hips;
- letting go, no joint's step changes over 1.4 cm in the frame, and the
  flight is ballistic to 1 mm/s²;
- it lands with nothing ahead;
- it catches bars 1.5 and 2 m ahead level with it, and 2.4 m ahead 0.5 m
  lower, and holds them (a wrist under 1 µm off its hook after 3 s);
- a bar 3.5 m ahead it does not let go at.

The walker walks under a 2.3 m bar and round a 1.5 m one.

**Live**: grabbed the 2.3 m bar, pumped it up to a near-horizontal swing,
let go swinging forward, flew piked with the arms reaching, caught the bar
2 m ahead, and swung down to hang still under it. The hands are on a bar
throughout in the gizmo and mesh views, Front and Left.

**Cost**: `anim_bench --characters 100`, a character at p50:

| Move | Cost |
|---|---|
| a pumped swing (`--gait bar-swing`) | 63 µs |
| the lache from letting go to the catch (`--gait lache`) | 32 µs |
| a free hang, for comparison | 57 µs |

The swing figure includes the bench replaying the swing from its start
each frame.

## Revisit when

- **A swing on a ledge with no wall below**: it pumps too (any free hang),
  but its hands do not roll; untested live.
- **Giant swings or a kip up onto the bar**: the swing stops at its
  amplitude, and nothing climbs onto a bar.
- **A lache sideways or turning**: only straight ahead.

## Related

- [A hang is leapt from, up, aside or back](./a-hang-is-leapt-from-up-aside-or-back.md) — prerequisite: the launch and flight a release reuses.
- [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — deeper: the free hang's pendulum and the grips this swings.
- [Walking is kept out of walls](./walking-is-kept-out-of-walls.md) — context: the wall probes now ask what is solid up to the headroom.
- [Parkour movement data](./parkour-movement-data.md) — data: the high bar's release windows and pumping phases.
