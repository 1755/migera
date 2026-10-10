---
title: A jump forward leans out over its toes, and its travel is the root's
description: "A standing jump forward pushes the COM ahead from mid-countermovement, leaves leaning out over its toes at Wakai's 60° and brakes over the landed feet; its way forward moves the entity. Traps: feet out of reach in flight, plan passes that never settle, pins short by the travel. Read before changing its way forward."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-06
verified: 2026-10-06
code:
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - src/character/anim/plugin.rs
sources:
  - "Wakai & Linthorne 2005 (Hum Mov Sci 24:81), Table 2: standing long jump take-off 3.24-3.55 m/s at 31-39°, COM 0.57-0.72 m ahead of the toes at 0.92-1.04 m up, landing 0.16-0.21 m behind the heels"
  - "tests jump::tests::a_jump_forward_*, a_long_jump_leaves_leaning_out_over_its_toes, too_far_is_planned_as_far_as_the_fastest_take_off_reaches; plugin::tests::a_jumps_landing_pins_each_foot_where_it_lands_not_where_the_sprung_foot_is"
  - "live: character_gallery --jump-at 2:0.25:1.5,7:0.1:0.5 --step-seconds 0.0166667, Xvfb, BRP every frame"
aliases:
  - JumpAsk
  - Jump::com_ahead_at
  - Jump::travelled_at
  - standing long jump
  - broad jump
  - FORWARD_PUSH
  - LONG_JUMP_HEEL_RISE
---

# A jump forward leans out over its toes, and its travel is the root's

`JumpAsk { height, distance }` adds a way forward to the standing jump's
centre-of-mass (COM) plan. Two choices carry it:
- The COM is pushed forward from the second half of the countermovement, so
  it leaves leaning out over the toes as a long jumper does.
- Its way forward moves the character entity like root motion, the pose
  posed that far back so the COM stays over the root.

## Decision

**The way forward** (`Jump::com_ahead_at`):

| Stretch | COM along the rig's forward |
|---|---|
| The last `FORWARD_PUSH` (0.5 s) before take-off | from rest to the flight's speed: one constant acceleration, so with the rise the floor pushes along one line |
| Flight | constant speed |
| Touchdown on | constant deceleration to rest over the landed feet |

The feet land `distance` ahead, so:

`distance = speed · (FORWARD_PUSH/2 + flight + braking/2)`

The flight's time hangs on the take-off and touchdown shapes. Those hang
on the speed (lean, heel rise, arms), so the plan is worked out over six
passes.

**The travel is root motion.** `Jump::travelled_at(t)` is how far forward
the COM has gone:
- The walker adds each frame's share to `Stride::stepped`, which moves the
  entity.
- `pose_at` takes it back off the root, so the posed COM stays over the
  entity.
- The foot locks hold planted feet in the world through that travel, as
  they do walking.

**Shape with speed:**
- The trunk leans by `LEAN_PER_SPEED` per m/s.
- The heels rise further leaving, up to `LONG_JUMP_HEEL_RISE` (34°) at
  2.85 m/s.
- The arms swing as for a jump of the take-off's whole kinetic height
  (`h + v²/2g`).
- The countermovement is as deep as that height asks.
- Too far to reach leaving at `FASTEST` (3.43 m/s, the highest jump's
  speed), it is planned as far as that reaches.

## Alternatives considered

- **Hand the whole travel to the root at the end:** the camera and anything
  following the entity jump 1-2 m in a frame, and the foot locks' world
  anchors would need the same jump.
- **Push forward only from the bottom of the countermovement**, constant
  acceleration over the push: the COM left 0.3 m ahead of where it stood,
  77° up from the toes, the body near upright. Wakai's jumpers leave 60°
  up, 0.57-0.72 m ahead of the toes.
- **Iterate the plan until it settles:** see the traps.

## Traps found

- **Feet out of reach in flight.** Each foot's way across the floor is held
  in the world, at rest as it leaves and as it lands. The hips fly on at
  2.3 m/s, so just after take-off a foot trails out of the leg's reach, and
  just before touchdown it is out of reach ahead: the knees snapped
  straight. The foot is now raised until the leg reaches it, so it lifts
  behind and comes down onto its spot.
- **Plan passes that never settle.** The COM landing farther behind the
  feet touches down lower, which shortens the landing, which lands it less
  far behind: the passes swung by 7-36 mm. The landing's first frame then
  had the COM off the shape it was solved in, and the knee jumped from 15°
  to 49°. The passes now only estimate the shapes. The way forward is then
  made to meet the last pass's shapes exactly:
  - the flight speed is where they put the COM, over the flight's time;
  - the push is Hermite from rest to that speed;
  - the landing brakes in its own time (`braking`), not the vertical
    landing's.
  - The shapes are leant and swung for the estimate (`pace`), not the exact
    speed: a few millimetres apart, the legs, solved to just reach, came up
    short.
- **A landing too deep.** The landing's depth was limited below touchdown,
  but a forward touchdown is already low with its feet ahead: knees folded
  to 135°. It is now limited below standing (117°).
- **A crouch sized by height alone.** Leaning out over its toes, a long
  jump leaves low. Its push had 0.15 m to reach take-off speed: 0.14 s at
  2.6 body weights, the arms 26° behind. Its depth is now sized by the
  take-off's kinetic height (0.29 s).
- **Heels that rise like a vertical jump's.** At 20° the body reached 1.02
  m from toes to COM, against Wakai's 1.14 m. It left 0.2 m below standing,
  below its own touchdown less what it rose: the flight time was the root
  of a negative number, NaN.
- **Pins short by the frame's travel.** A lock takes the body's travel off
  its anchor. Pinned where the pose has the foot, a landing still
  travelling came down a frame's travel short (38 mm at 2.3 m/s). The pins
  now add it back.

## Consequences

**Measured against Wakai & Linthorne** (their jumper of our build, 3.4 m/s
at 33°):

| | Here | Wakai & Linthorne |
|---|---|---|
| Toe→COM line leaving | 59° up | 60° |
| COM ahead of the toes leaving | 0.54 m, 0.91 m up | 0.57-0.72 m, 0.92-1.04 m up |
| COM behind the heels landing | 0.11 m | 0.16-0.21 m |

**Live** (1.5 m and 0.5 m jumps):
- the feet land 1.507 and 0.503 m from where they left;
- nothing under the floor;
- the feet slow from 46 to 10 mm a frame along the way as they come down;
- the pelvis falls at −9.8 m/s² in flight. Its speed forward varies
  1.2-1.9 m/s while the COM's does not: the legs swing through under it.
- The toe-end joint moves ≤ 2.6 mm a frame on the floor as the heel comes
  down. That is the joint rolling about the sole's contact, which the
  tests hold within 1 mm.

**Through the springs** (led, headless), against 61 mm at worst unled:
- the COM is 15.5 mm off the plan the first frame in the air;
- 6-11 mm for 0.1 s after that;
- 1-3 mm through the rest of the flight;
- 8.6 mm the frame before touchdown, where the lead stops at the phase's
  end.

**Cost:** 57 µs a character a frame (`anim_bench --gait jump --speed 0.25
--distance 1.8`), against 53 straight up; the plan is made once a jump.
The flight's pelvis solve over-steps across (the feet stay behind in the
world) but not up (they ride with the hips): over-stepped both ways, it
rang, and a jump straight up cost 60 µs.

**Known gap:** a long jumper meets the floor in a deep pike, the COM 0.59 m
up. This forefoot touchdown is higher, so the flight is shorter. At their
3.4 m/s it jumps 1.97 m, not 2.33.

## Revisit when

- The landing gets a pike or a heel-first touchdown (the gap above).

## Related

- [A jump is planned as its centre of mass's path](./a-jump-is-planned-as-its-centre-of-mass-path.md) — prerequisite: the vertical plan, pelvis solve, spring lead and landing pins this extends.
- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — context: root motion read off the run's planted feet, the other way travel reaches the entity.
- [A jump from a run replays the run's stance under a planned COM](./a-jump-from-a-run-replays-the-runs-stance-on-a-planned-com.md) — extension: a stopping jump from a run lands with this landing.
- [A toe tip pivots on the floor and needs its own lock](./a-toe-tip-pivots-on-the-floor-and-needs-its-own-lock.md) — context: the tip lock the landing pins.
- [Third-person camera design](../../gameplay-camera/third-person-camera-design.md) — applies: the camera that follows this root motion, and its airborne deadband.
