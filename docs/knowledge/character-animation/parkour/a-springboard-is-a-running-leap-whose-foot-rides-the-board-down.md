---
title: A springboard is a running leap whose take-off foot rides the board down
description: "Step 12: a springboard is a running leap off one foot, planned as much higher as the board gives (0.4 m), the planted ankle sinking 4 cm with the board in step with the push, the COM's path the leap's own; handed to a fall at its top. Read before changing parkour/springboard.rs or Jump::from_board."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/springboard.rs
  - src/character/anim/jump/leap.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
  - examples/anim_bench.rs
sources:
  - "test parkour::springboard::tests::a_springboard_bends_under_the_foot_and_throws_the_leap_higher"
  - "live: character_gallery --step-seconds 0.0333333 --start-height 1.0 --block 0,-8,0,1.0,2.0,12 --springboard 0,-8.6,0,1.0,1.2 --block 0,-9.8,180,1.6,2.0,3 --anim-speed 4 --springboard-at 0.5, Xvfb, gizmos on/mesh off Left and Front, mesh on Left; BRP pelvis and feet"
  - "anim_bench --gait springboard --characters 100"
aliases:
  - springboard
  - sprung plank
  - Springboard
  - Jump::from_board
  - Board
  - board_sunk
---

# A springboard is a running leap whose take-off foot rides the board down

Step 12 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), last part: a sprung
plank (or a flagpole's end) run onto. The take-off foot comes down on its
end, the board bends under the push and springs back, and the leap goes
higher than a leap can.

## Decision

**A leap off a board is a running leap** (`Jump::from_board`, a
`jump::Board { give, dip }`): `Jump::from_run` with two changes.
- **Higher.** It may rise up to the board's give more than a leap may
  (`HIGHEST`, 0.6 m). Off a plank (`Springboard::plank`) it asks the
  leap's own 0.3 m and the board's 0.4 m (`LEAP`, `GIVE`).
- **The planted foot rides the board down.** Through the take-off stance
  the ankle sinks by the board's bend (`Stance::sunk`). The board bends as
  hard as the foot pushes, so the sink follows the floor's push
  (`push_share`, the bump `pushed_height` shapes the push with). It is
  squared, so the foot meets and leaves the board at the board's rest
  speed. It peaks at 0.04 m (`DIP`) where the push peaks, about 0.8 of the
  stance.

**The COM's path is the leap's own.** A board yielding under the same push
leaves the body's path as it is: only the leg reaches farther down after
the foot. What a real board adds is energy stored from a landing on it and
given back. A run's footfall brings little (0.3-0.5 m/s down). A game's
board adds rise by fiat: its give.

**The plank's 4 cm** is what the leg has spare. Where the push is hardest,
the plant leg has 7-10 cm of slack before straight.

**It is handed to a fall at its top** (`hands_over`, as a precision jump
is). The fall (`spring_fall`, `Falling::from_jump` then `land_on`) lands on
the first top along the flight, or on the ground below it. The walker
also holds it off walls (`against`) and lets it catch a ledge, as any fall.

**The walker** (`Walker::springboard`) paces the run's last steps as a
vault does (`vault_pace`), to bring a foot down with its ankle 0.3 m back
from the board's free end, within 0.15 m (`SPOT_BACK`, `ON_BOARD`). It
leaps off that foot. Come down past it, or off its line, the ask is
dropped. Until the hand-over:
- the walker's own over-an-edge fall is held off;
- the legs are left as the plan poses them (`legs_free`), not lifted clear
  of the floor (`off_floor`).

**The gallery** draws the plank bent about its fixed end by
`WalkerState::springboard_bent`. Its top is ground (a `LedgeGround`
ledge, not drawn).

## Alternatives considered

- **The COM sinking with the board** (a stiff leg riding a yielding
  board): the same force gives the same COM path whatever the floor does.
  A `sin²` sink added to the COM pulled at 11 g as the foot came down.
- **The board's bend in step with the push alone, not squared**: the foot
  left it at 2.2 m/s up of its flight's, a 3.7 cm change of step.
- **A two-footed board (a gymnast's hurdle onto it)**: a hurdle is the
  physical source of a board's energy, but a run in a game takes off from
  one foot. Not built.

## Traps

- **Handed to the fall while still rising** toward a top higher than it
  left, the fall's drop (standing height to standing height, from the
  hand-over) was negative. The landing had no depth to brake in and went
  NaN. The precision jump met this first. Planning every fall's landing
  from the top of its flight instead fixed the NaN, but moved a running
  jump's fall 1 cm off its path a frame on, and worsened the board's
  landings (25 cm changes of step). So the hand-over waits for the top.
- **The walker's own over-an-edge check** handed the leap to a fall the
  frame its root passed the plank's end, before the board's own hand-over,
  and went NaN the same way.
- **Aimed at the free end itself**, the foot came down 13 cm past it, over
  the drop. Its ground was the floor below, and it sank 25 cm.
- **The foot IK undid the bend twice.** Its locks held the planted toe at
  the ground plus the plan's height over standing, so the ankle rose as a
  plain leap's. Left free, the pose was lifted clear of the floor.
- **A test baseline from `Jump::from_run`** capped the rise at a leap's
  0.6 m. Against it, the foot seemed to sink 3.5 mm more than the board. The
  baseline is the same leap off a board that does not bend.

## Consequences

**Headless** (`puppet_base`, 60 fps; 4 and 5 m/s, either foot; onto the
floor, and onto tops 0.4 and 0.6 m higher):
- the board bent 0.039-0.040 m, from and to rest;
- the ankle sank with it within 0.1 mm (the plant leg reaching it);
- the COM rose 0.700 m, as asked;
- the fall's hips ballistic within 0.001 m/s²;
- landed where asked, every pose finite;
- across the hand-over, no step changed over 3.6 cm. The leap's own
  toe-off changes 8-11 cm.

**Live** (Xvfb, a fixed 1/30 s step), running at 4 m/s along a 1 m block
at a plank 0.6 m past its edge, a 1.6 m block 1.2 m beyond:
- the take-off foot came down on the plank 0.3 m from its end (BRP);
- the ankle sank 2.3 cm below where it came down where a plain leap's
  rises 4 cm;
- the pelvis rose to 2.66 m, and it landed on the 1.6 m top and stood.

From the left, the foot pushed off the plank's end, the plank sloping down
under it, and the feet passed well over the top. From the front, both legs
hung under the body in flight.

**Cost**: `anim_bench --gait springboard --characters 100`, 21 µs a
character at p50 (the clock mostly the fall's).

## Revisit when

- **A run on after landing**: the fall lands two-footed and stands. A
  board's leap landing running on would need the running leap's landing
  stance onto a top.
- **A two-footed hurdle onto the board**, if a game wants the gymnast's
  board.
- **A board under more than the take-off foot**: the plank's ground is
  static, and only the take-off foot rides its bend.

## Related

- [A precision jump is a standing jump handed to a fall at its top](./a-precision-jump-is-a-standing-jump-handed-to-a-fall-at-its-top.md) — same-trap: the hand-over that waits for the top, and why.
- [A jump from a run replays the run's stance on a planned COM](../ik-and-locomotion/a-jump-from-a-run-replays-the-runs-stance-on-a-planned-com.md) — prerequisite: the running leap and the push `pushed_height` shapes, which the board's bend follows.
- [A small obstacle is hopped as a running leap, its feet lifted](./a-small-obstacle-is-hopped-as-a-running-leap-its-feet-lifted.md) — contrast: another running leap reshaped, and the vault pacing a board's run reuses.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — deeper: the fall's landing depth, from the drop, that went NaN below nothing.
