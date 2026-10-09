---
title: A small obstacle is hopped as a running leap, its feet lifted
description: "Steps 11-12: a run hops an obstacle up to 0.45 m high in stride, a running leap lengthened and raised as little as clears it, each foot lifted over on a timed ease, the COM's path kept. A standing jump tucks its knees over one the same way. Read before changing VaultKind::Hop, HopLift or Jump::tuck_over."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/vault.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::vault::tests::{a_small_obstacle_is_hopped_in_stride, a_standing_jump_tucks_its_knees_over_an_obstacle}"
  - "live: character_gallery --block 0,-10,180,0.35,3.0,0.3 --vault-at 0.5,hop --anim-speed 4, Xvfb, gizmos on/mesh off Left; BRP pelvis and feet"
  - "live: character_gallery --step-seconds 0.0333333 --post 0,-0.8,0.55,0.2 --jump-at 3:0.35:1.4, Xvfb, gizmos on/mesh off Left and Front, mesh on"
  - "anim_bench --gait hop|tuck --characters 100"
aliases:
  - hop
  - hurdle step
  - VaultKind::Hop
  - HopLift
  - Jump::hop
  - tuck
  - tuck jump
  - Jump::tuck_over
  - lift_over
---

# A small obstacle is hopped as a running leap, its feet lifted

Step 11 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md),
third part: a hop past a small obstacle in a run's stride. Below a speed
vault's reach (0.75 m), a kerb, a low rail or a box is passed with one
lengthened step, the swinging foot lifted over it, the run unbroken.

## Decision

**A hop is a vault kind** (`VaultKind::Hop`, asked as `HangAsk::Vault`), so
it reuses the vault's walker plumbing. The last steps are paced to its best
take-off, it takes off from a footfall, and it runs on from the landing.

**The plan** (`Jump::hop`): for obstacles up to 0.45 m high and 0.6 m deep,
from a run of at least 2.5 m/s, met no more than 0.5 rad off square:
- a running leap (`Jump::from_run`), unreshaped;
- the take-off toe at least 1 m before the near face (`hop_takeoff`: the
  COM's top over the middle, or that), the landing toe 0.9 m past the far
  face;
- its COM rising 0.05 m, then 0.05 m more at a time (to 0.55 m), until
  nothing of the body goes into the obstacle.

**The feet lifted** (`HopLift`, laid over the leap's pose):
- for each leg, the times its ankle or toe is within 0.1 m of the obstacle
  along the way are found on the bare leap;
- the most it must rise there is also found: the ankle 0.14 m and the toe
  0.05 m over the top;
- the lift eases in over 0.12 s before and out over 0.12 s after, but
  only while the foot is off the floor (a foot is down when it is low and
  still);
- the ankle is put up by that much (`place_ankle`), so the knee comes up
  ahead, as a hurdler's.

**The centre of mass keeps the leap's path** (`lift_over`, added
2026-10-09). Legs drawn up carry the COM up with them, which the flight
cannot do: a body in the air moves its COM only by gravity. So the root
comes down by however much the lift raised the COM, and each foot is
lifted by `1/(1 − 0.16)` of its need (`LIFT_SINKS`: the legs are about a
third of the mass, and a leg's centre rises about half its foot's lift),
so it still clears by what it was asked to.

**A standing jump tucks the same way** (`Jump::tuck_over`, step 12 of the
steps beyond the first ten, the tuck in the air). A standing jump as asked,
with a `HopLift` over the obstacle in its way. It is `None` if a foot is
over the obstacle while on the floor, or if anything goes into it. The
walker uses it when a standing jump is asked with a ledge in its path (the
nearest the line of the jump meets, its far face short of the landing), and
otherwise jumps plainly.

## Alternatives considered

- **The leap alone, raised**: a running leap's own legs trailed into
  anything over a kerb. Over a 0.25-0.55 m rail, only 6 of 36 runs
  planned, even rising 0.55 m. The trailing leg swings through low after
  touchdown.
- **A foot lifted by how near it is to the obstacle** (a soft floor over
  its top, weighed in over 0.35 m or 0.5 m): the foot crosses that in a
  tenth of a second, and a knee rose 0.3 m in it. A step changed 6 cm in
  a frame, and 18-28 m/s about the COM.
- **Lifting only within the flight**: the take-off leg trails behind and
  crosses the obstacle after touchdown, while running on. Every hop failed
  to plan.
- **A speed vault's reshape** (hips rolled, a hand on the top): the hand
  cannot reach a top under 0.75 m from a running body.

## Traps

- **"Down" by height alone**: the leap's landing foot skims the floor
  before it lands. Taken as down, it cut a lift to a single frame (a 32 cm
  change of step). A foot is down when it is low and still (under
  0.3 m/s).
- **Landing 0.65 m past the far face** left the lead foot over the far
  edge until a few frames before touchdown, with no time to set its lift
  down. 0.9 m past gives it the ease.
- **Hopping over 0.55 m** made the COM rise 0.25 m and the run go on
  1.3 m/s slower; from 3 m/s at its best take-off, nothing cleared it. A
  hop stops at 0.45 m.
- **The walker's way round a wall ran during the jump.** The ask is dropped
  at take-off, so the walker turned off round the obstacle in the air and
  the hop curved 1.7 m aside. The detour now waits while a jump is under
  way.

## Consequences

**Headless** (`puppet_base`; rails 0.25, 0.35 and 0.45 m high, 0.2-0.4 m
deep, runs at 3 and 4.5 m/s, off either foot, from the best take-off to
0.4 m past it, and at 4 m/s up to 1.2 m past):
- nothing went into the obstacle;
- the COM rose 0.05-0.25 m;
- the run went on at its speed (3.00 and 4.50 m/s);
- no joint went over 11.1 m/s about the COM;
- the lift added no change of step to the leap's own;
- 0.6 m high or 1 m deep is not hopped.

**The leap's own step change is 11-20 cm in a frame**, as a planted toe
leaves or meets the floor. That is the running leap's, vault and all, and
unmeasured before this.

**Live**: running at 4 m/s at a 0.35 m block 10 m ahead, it paced to the
take-off, hopped with the pelvis up to 1.04 m and each foot over the block
by 0.17 m and more, and ran on along its line. From the left, the lead knee
came up and the foot passed over the top.

**Cost**: `anim_bench --gait hop --characters 100`, 76 µs a character at
p50 (the speed vault's 150).

**The tuck** (headless, `puppet_base`): standing jumps 0.35 m up and 1.4 m
on, and 0.45 m up and 1.8 m on, over posts 0.2-0.3 m deep in their middle,
the lowest each does not clear by itself (0.55 and 0.65 m) and 0.1 m
higher:
- untucked, a leg went 6.7-8.9 cm into each;
- tucked, nothing did;
- the COM kept the jump's path;
- the jump's own change of step (4.9 and 11.8 cm) and fastest joint (7.1
  and 7.8 m/s about the COM) were unchanged.

The shorter jump tucks over a 0.55 m post anywhere 0.35-0.9 m ahead. The
standing jump's own flight tuck already clears up to 0.45 m (0.35 m up) and
0.55 m (0.45 m up). Live, a 1.4 m jump over a 0.55 m post: from the left,
the knees came up and both feet passed just over its top; from the front,
both legs over it. `anim_bench --features real_rig --gait tuck`, 58 µs a
character at p50. The synthetic rig's stub shins tuck over nothing.

## Revisit when

- **The running leap's toe-off and touchdown** change of step (11-20 cm) is
  smoothed: it is in every running jump.
- **A hurdler's trail leg** (abducted, knee out, pulled through high) for
  higher obstacles; the hop now lifts the ankle straight up.
- **Obstacles found in the run's path** without an ask: hops are asked now.

## Related

- [A low obstacle is speed-vaulted as a reshaped running leap](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — prerequisite: the running leap, its take-off pacing and running on.
- [A jump from a run replays the run's stance on a planned COM](../ik-and-locomotion/a-jump-from-a-run-replays-the-runs-stance-on-a-planned-com.md) — deeper: the leap a hop lengthens.
- [A skid stop slides side-on and rises over stuck feet](./a-skid-stop-slides-side-on-and-rises-over-stuck-feet.md) — same-trap: a soft lift that must not come on in a few frames.
