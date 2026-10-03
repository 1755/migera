---
title: Sitting down and standing up go through solved keys, chained on their contacts and refined against the floor
description: "A walker sits on a chair (4 poses) or the floor (5) through solved key poses: chair rise timed to Schenkman/Marsh (1.9 s, 28/18/54 %), feet fixed, floor routes through squat or half-kneel. Traps: leg IK flattens a turned knee, blends sweep feet through the floor, sprung toes lag. Read before changing sitting."
type: decision
status: current
tags:
  - locomotion
  - poses
  - biomechanics
  - correctness
updated: 2026-10-03
verified: 2026-10-03
code:
  - src/character/anim/sitting.rs
  - src/character/anim/walker.rs
  - src/character/anim/rig.rs
  - src/character/anim/plugin.rs
  - examples/character_gallery.rs
sources:
  - "Schenkman et al. (1990), Whole-body movements during rising to standing from sitting, Phys Ther 70(10):638 — four phases: flexion momentum, momentum transfer, extension, stabilization"
  - "Kinematic comparison of rising from two types of chairs (Wake Forest), http://users.wfu.edu/marshap/MattErin.htm — 1.9 s rise, phases 28/18/54 %, knee ~105° (interior) at seat-off, trunk 0.49 m forward"
  - "Stand-to-sit ~2.0 s, sit-to-stand ~2.2 s in healthy older adults, https://pmc.ncbi.nlm.nih.gov/articles/PMC8867046"
  - "tests sitting::tests (structural, both rigs; full sit-and-stand cycles at 60 Hz on the rig as rendered)"
  - "live: character_gallery --sit NAME --sit-at 3 --stand-at 9, BRP world joint heights and speeds"
aliases:
  - Sitting
  - ChairPose
  - FloorPose
  - Posture
  - Walker::sit
  - legs_free
  - legs_by_their_feet
  - refined
  - clear_floor
  - toes_on_floor
  - blend_in_world
  - sit to stand
  - stand to sit
---

# Sitting down and standing up go through solved keys, chained on their contacts and refined against the floor

A walker sits by `Walker::sit = Some(Sitting)` and stands again with `None`.

- **The poses:**
  - on a chair: upright, reclined, legs crossed, leaning forward;
  - on the floor: cross-legged, propped back, hugging the knees, side-sit, kneeling.
- **Solved on the rig**, as the get-up's keys are (`sitting.rs`): each
  pose's contacts meet the seat or the floor together.
- **Reached through key poses** that are chained so shared contacts stay
  put, and blended per bone in the world (`rig::blend_in_world`, shared
  with the get-up).

## Decision

- **Chair** (`CHAIR_HEIGHT` 0.45 m):
  - The feet stay where the character stood, since a person sits back
    onto a chair behind them. The chair goes under the seated hips
    (`seat_offset`); the gallery spawns it there.
  - The thighs are solved so the seat contact (hips joint −0.10 m) meets
    the chair with the feet flat.
  - **Down:** lowering (trunk 35° forward, knees ~60°), touching down
    still leaning, then the pose: about 2.1 s.
  - **Up:** Schenkman's phases timed to the measured 1.9 s rise. Lean
    forward 0.55 s, seat-off with the hips 3 cm up and the trunk 45°
    forward 0.35 s, extension to standing 1.0 s (28/18/54 %).
  - **Crossed legs and reclining** go through upright first.
- **Floor:**
  - **Down:** through the get-up's squat and propped sit (its face-up
    route reversed). **Up:** by that route.
  - **Kneeling** goes half-kneel → tall kneel → back onto the heels, with
    the toes tucked, and up the same way. The tucked toes are the
    half-kneel's back-foot angle.
  - **The side-sit** goes through the propped sit and a key with both
    knees tipped 80° to the side and the feet turned that way
    (`knees_aside`), 1.2 s each fold.
- **The walker's `Posture`**
  (Standing → Moving through keys → Seated, and back):
  - It sits only once stopped and asks no speed while not standing.
  - The idle sway is off while seated.
  - The root is carried so feet planted in both keys stay where the
    blend between the keys puts them.

## Traps it hit

- **Leg IK keeps every knee in its leg's plane.** A cross-legged pose's
  knees, turned 55° out, rendered pointing straight ahead.
  - `AnimFootIk::legs_free` leaves the legs as posed. It is set while
    seated on the floor, while kneeling, and in any move where a foot is
    not planted.
  - The IK stays on for chair moves and moves with both feet planted. Its
    locks hold the feet against the springs' lag: 21–24 mm without them.
- **A foot aimed backward by the shortest arc yaws round.** Its sole kept
  facing down, 119 mm into the floor. Kneeling feet are pitched with
  sagittal angles instead.
- **Two keys clear of the floor do not make a clear blend.** Blended bone
  by bone, a limb sweeps through:
  - a foot turning from flat to pointing back passed through pointing
    down, 253 mm under;
  - a folding leg swung its foot 258–348 mm under.

  The fixes:
  - **Keys laid out to avoid it:** tucked toes, feet turned aside before
    the side-sit.
  - **`refined`:** splits any move that would sweep more than 2 cm, at
    its middle, twice over. The new key is built with each leg carried by
    its foot (`legs_by_their_feet`): the ankle on the straight line
    between the keys, the knee bent the way the blend has it.
  - **A smooth lift** for the rest (`clear_floor`).
- **Floor corrections worked out per frame pop again** (see
  [a drawn floor correction is held between frames](../ragdoll-and-physics/a-drawn-floor-correction-is-held-between-frames.md)):
  - Turning a foot toes-up about its ankle lowers its heel, so a flat
    foot 1 mm under turned the full 90° (5.9 m/s).
  - Over a foot pointing straight down, the turn's axis flipped (a toe
    moved 35 cm in a frame).
  - Legs solved by their feet every frame whipped a knee 149–260 mm when
    it neared the hip-to-ankle line.

  Only the lift is per frame now. The leg solve builds static keys, and a
  plain blend between fixed keys is continuous.
- **The target clear is not the render clear.** The legs' 0.015 s springs
  trailed a foot turning fast near the floor by ~4 cm: a kneeling toe tip
  went 47 mm under while the target sat at −5 mm. With the legs free, the
  IK stage now lifts the sprung pose clear too.
- **Toes are not rigid.** A tucked toe bends ~90° at the ball. Kept
  straight, its tip went 71 mm under. Each floor key bends the toes up to
  the floor (`toes_on_floor`), and `lowest_point` measures the real toe
  tip, not the sole's rigid one.
- **Planted within 3 cm is not planted.** Holding a foot where the first
  key had it, the root jumped at the next key. The held foot moves along
  the blend between the keys' positions.
- **Test the cycle on the rig as rendered.** `puppet_base` faces away, so
  its standing pose holds the hands overhead (see
  [puppet_base faces away](../rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md)),
  and the arm swing up to them read as a 65 mm jump.

## Consequences

- **Tests:** every pose rests on its contacts on both rigs, nothing under
  the floor; the chair's seat is at 0.45 ± 0.01 m and its feet are where
  the character stood (< 2 cm) through every key; the upright knee bends
  70–110° forward.
- **The cycle test** sits and stands at 60 Hz for each way of sitting:
  nothing more than 6 mm under the floor, no joint over 60 mm a frame,
  and standing again at the end.
- **Live, every way, a full cycle:**

  | | Lowest point | Fastest joint | Seated pelvis |
  |---|---|---|---|
  | Chair | ≥ +0.015 m | 1.05–1.10 m/s | 0.55 m |
  | Floor | ≥ 0.000 m | 2.6–3.4 m/s (the arm swung forward to rise) | 0.10 m (kneeling 0.20) |

  Feet on a chair stayed within 0 mm of where the character stood.
- **Seen** from the side and the front, mesh and gizmos: each pose as
  described; standing up from the chair leans forward, leaves the seat,
  then extends.

## Revisit when

- Sitting on something other than a flat chair or the floor: the seat
  height is a parameter, the seat's shape is not.
- The side-sit mirrored (on the right hip): only the left is built.

## Related

- [Walking to a chair turns on a circle and paces its stop](./walking-to-a-chair-turns-on-a-circle-and-paces-its-stop.md) — extends: given `Walker::chair` it walks there and turns round first; `Seat::back` moves the seat as far as that walk stopped off its spot.
- [Getting up goes through key poses](../ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md) — prerequisite: the keys, contacts, chaining and world-space blend the floor routes reuse.
- [A drawn floor correction is held between frames](../ragdoll-and-physics/a-drawn-floor-correction-is-held-between-frames.md) — same-trap: stateless per-frame floor corrections that pop.
- [Foot IK on uneven ground has two feedback loops](./foot-ik-feedback-loops.md) — context: the leg IK that `legs_free` turns off.
