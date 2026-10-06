---
title: A crouch is the jump's countermovement held over the feet, eased by its COM's whole rise
description: "sneak::Footing poses a crouch (depth 0-1, flat or on the toes) with the jump's foot-bound solver: trunk leant and arms forward by depth, the COM lowered over the feet, or over the balls with the heels up. Trap: an ease timed by depth alone ignored the toes' 7 cm rise: 3.3 m/s² where 2 was planned."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-06
verified: 2026-10-06
code:
  - src/character/anim/sneak.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
sources:
  - "Steele et al. 2010, J Biomech: crouch gait classed by least stance knee flexion (mild 20-35°, moderate 35-50°, severe past 50°)"
  - "tests sneak::tests::*"
  - "live: character_gallery --sneak-schedule 1:1,4:0.5:toes,7:0,9:1 --anim-speed-schedule 0:0,11:1.2, --step-seconds 0.0166667, Xvfb, BRP every frame"
aliases:
  - Sneak
  - Walker::sneak
  - Footing
  - Crouching
  - crouch
  - DEEPEST
  - TOES_HEEL
  - CROUCH_ACCELERATION
---

# A crouch is the jump's countermovement held over the feet, eased by its COM's whole rise

A sneak is asked as two dials (`Walker::sneak`, `sneak::Sneak`):
- `crouch`, 0 (standing) to 1, the deepest sneak;
- `on_toes`.

Standing still, the walker crouches into it. This note covers the
crouch from a stand. The sneak's gait builds on it.

## Decision

**A crouch is posed as the jump's countermovement is, held still.** The
jump's foot-bound solver (`jump::Feet::solved`, shared, the heel rise
passed in) does all of it:
- The trunk leans by the COM's drop (`jump::LEAN_PER_DEPTH`), the neck
  keeping the head up.
- The arms come forward by depth (`CROUCHED_ARMS`, authored).
- The pelvis is solved so the real COM (`anthropometry::centre_of_mass`)
  is where the crouch asks: lowered by its depth, over where it stood.
- The feet stay where they stood.

| | Flat | On the toes |
|---|---|---|
| Heels | on the floor | risen `TOES_HEEL` (20°) about the toe tips |
| COM over | where it stood | the balls of the feet (12 cm ahead on `puppet_base`) |
| COM height | standing less the drop | that, plus the ankles' rise (7 cm) |

Adding the ankles' rise keeps the legs as bent as they are flat. Without
it, standing on the toes asks the legs to fold to stay at standing height.

**Depth.** `DEEPEST` is 0.18 of leg length (0.16 m on `puppet_base`). It is
scaled to the rig like the gait's fractions. At the deepest:
- the knees fold 75°, past severe crouch gait's 50° (Steele et al. 2010);
- that is short of a jump's 90-110° countermovement;
- the shanks lean 29°, inside a loaded ankle's ~40°.

**Easing.** `Crouching` eases with a cubic that is at rest at its end:
- No faster than `CROUCH_ACCELERATION` (2 m/s², a fifth of g), a calm
  crouch, not a jump's unloading. Nor quicker than `QUICKEST` (0.4 s).
- Asked again on the way, it goes on from where it is at the rate it is
  going. The duration is lengthened until both ends' accelerations fit.

**In the walker.**
- The crouch replaces the standing pose at rest, and both feet are held
  planted.
- Asked to walk while sneaking, it walks from the crouch, and the crouch
  can change on the move
  ([the sneak's walk](./a-sneak-walks-the-walks-foot-path-moved-by-its-crouch.md)).
- Asked to sit, jump or step aside, it stands up first. The jump and the
  step stay asked until then.

## Trap: time the ease by the COM, not the depth

The first ease was timed on the change in depth alone. A deep flat crouch
changed to a half crouch on the toes moves the COM by both:
- 0.08 m less depth;
- plus the toes' 0.07 m rise.

It planned for 0.08 m and moved 0.15, and the pelvis accelerated at 3.26
m/s² live against the 2 planned.

`Crouching::ask` now takes the toes' rise (`Footing::rise`). It times the
COM's way, `drop − rise·toes`, and that way's rate. With the rise ignored,
the easing test fails at 3.69 m/s².

## Consequences

**Live** (`puppet_base`, 60 Hz, crouching deep, then half on the toes,
standing, deep again, then asked to walk):
- the pelvis goes from 0.943 m to 0.762 m deep and 0.925 m half on the
  toes;
- its sharpest acceleration is 2.09 m/s²;
- the balls of the feet move at most 0.24 mm a frame (99th percentile
  0.1 mm), the toe tips 0.3 mm;
- asked to walk while crouched, it stood within 0.5 s and walked.

**Seen** (Left and Front, gizmos on, mesh off; then the bare mesh):
- the knees fold forward, the trunk leans, the hands are ahead of the
  thighs, the feet are flat;
- on the toes, the heels are clear of the floor with the tips on it;
- from the Front, the legs are in their lanes and symmetric.

**Cost** (`anim_bench --gait crouch --speed 1`):
- 31 µs a character a frame crouching, against 4 µs idle.
- `Footing::of` is 7 µs of it. A footing cached across frames would save
  that; not done, as the standing pose it is built from can change.

## Revisit when

- Many characters crouch at once: cache the `Footing` per standing pose.

## Related

- [A jump is planned as its centre of mass's path](./a-jump-is-planned-as-its-centre-of-mass-path.md) — prerequisite: the foot-bound pelvis solve and lean this reuses.
- [A sneak walks the walk's foot path moved by its crouch](./a-sneak-walks-the-walks-foot-path-moved-by-its-crouch.md) — next: the walk from this crouch.
- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — context: the standing knee bend a crouch starts from.
