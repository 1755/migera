---
title: The walk's step width and sideways sway come from the inverted pendulum
description: "Walk feet were 22.9 cm apart, pelvis still sideways. Now 13 cm (STEP_WIDTH), the narrowest keeping the pendulum COM medial of the stance foot's inner border, and the pelvis sways ±1.5–2.3 cm by Eq. 11.3. Read before changing step width, walking sway or sway_over_feet."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - performance
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/stance.rs
  - src/character/anim/walk.rs
  - src/character/anim/phase.rs
sources:
  - "test phase::tests::the_walking_sway_is_the_pendulums_periodic_path"
  - "test locomotion::tests::a_walking_body_sways_over_its_stance_feet_but_never_past_them"
  - "test stance::tests::narrowed_feet_stand_the_step_width_apart_flat_and_mirrored"
  - "live BRP, character_gallery --anim-speed-schedule 0:0,4:1.2,10:0, A/B against the previous build"
aliases:
  - step width
  - STEP_WIDTH
  - walk sway
  - WalkSway
  - lateral pelvis
---

# The walk's step width and sideways sway come from the inverted pendulum

Winter's walking data (Appendix A) is side-view only. So the walk had no
frontal motion of its own: the feet tracked straight under the hip
sockets, 22.9 cm apart on `puppet_base` (about twice a person's step
width), and the pelvis stayed on the line of progression (±1 mm). The only
frontal motion was an authored hips roll.

## What Winter does give

- **A constraint, not a width.** The COM passes just medial of each stance
  foot's inside border, never over it (§11.3.1, Fig. 11.7). Fig. 11.7 has
  no scale bar and a stretched lateral axis, so no width can be read off it.
- **The dynamics.** Eq. 11.3, `COP − COM = −K·COM̈` with K ≈ d/g, and the
  pressure under each stance foot in turn. The periodic answer is closed
  form: the pressure path is a trapezoid wave (a square wave smoothed over
  double support), and the pendulum divides each harmonic k by
  `1 + K(2πk/T)²` (`phase::walk_sway_at`). The first harmonic is 98 % of it.

## Decisions

- **Step width 0.57 of the hip-socket spacing, 13 cm on `puppet_base`**
  (`stance::STEP_WIDTH`). This is the narrowest width that still keeps the
  pendulum's COM inside the stance foot's inner border at the slowest walk.
  The border was measured from the foot mesh: 11 cm wide, 3.8 cm inside the
  sole centreline. Margins:

  | width | 0.7 m/s | 1.2 m/s | 1.6 m/s |
  |---|---|---|---|
  | 10 cm | −5.4 mm (crosses) | −1.7 mm | +0.4 mm |
  | 12 cm | +1.1 mm | +5.6 mm | +8.1 mm |
  | **13 cm** | **+4.3 mm** | **+9.2 mm** | **+11.9 mm** |
  | 22.9 cm (old) | +36 mm | +45 mm | +50 mm |

  `narrow_feet` turns each leg whole about its hip, with the foot turned
  back so it keeps its attitude. `WalkCycle` composes Winter's stride onto
  those narrowed legs, and keeps the standing base's ground, so the pelvis
  comes down the 1.3 mm the turn lifts the foot.
- **The sway lives in the locomotion `PhaseLayer` (`WalkSway`), not
  `walk.rs`.** The pendulum needs the stride's seconds, which the phase
  layer's clock knows and a `WalkCycle` does not. It fades with the gait
  weight in `PhaseLayer::between`. It runs after root motion is derived
  from the bare walk and keeps the feet in place, so it cannot move the
  body's path.
- **K from a height ratio, not the COM.** The COM's height above the
  ankles is 1.19× the hips' (`COM_OVER_HIPS`, pinned by a test against
  `anthropometry::centre_of_mass`). Evaluating the COM every frame cost
  ~2 µs a character.

## Two lessons

- **One pelvis height cannot suit two whole-leg turns.** `sway_over_feet`
  averaged the height the two legs asked for. In a walk that moved a
  planted foot 1.2 mm in double support and put a swinging toe 0.6 mm into
  the floor after toe-off. Now the height is weighted by stance load
  (`sway_over_loaded_feet`), and each knee takes up its leg's residual
  (`keep_ankle`). The result: 0.02 mm in single support, ≤ 0.67 mm in
  double support (the trailing leg is near full extension), 0.01 mm on the
  swing foot.
- **The leg IK is the wrong tool for millimetre fixes.** `solve_leg_on`
  places the toe joint and re-aims the foot (`aim_foot`), so heel and tip
  moved *more* (1.4 mm single support, 4.2 mm swing) than before it ran.
  `keep_ankle` bends the knee, turns the leg and turns the foot back.

## Measured

- Headless, `puppet_base`: the pelvis sways 2.3 / 1.8 / 1.6 cm toward the
  stance foot at 0.7 / 1.2 / 1.6 m/s, peaking at ~0.31 of the stride.
  `walk_sway_at` is within 0.5 mm of a direct finite-difference solve; a
  sabotaged phase shift fails the test by 11.6 mm.
- Live, 1.2 m/s: step width 230 → 134 mm; pelvis sway against the root
  0 → 36.8 mm peak to peak. Start planted slide unchanged, 1.4 / 2.3 mm.
- An A/B on the same schedule found two older issues. The root weaved
  ~100 mm sideways over a steady walk; that was the authored pelvis roll,
  replaced by the
  [walking pelvic obliquity](./walking-pelvic-obliquity-from-hip-abductor-power.md)
  (weave now 14 mm). The last step still glides ~7–9 cm near the floor
  after the root stops.
- `anim_bench` (switched to the rig-aware `apply_on` on a stance base,
  same change in the baseline): 2.4 → 3.8 µs per character per frame for
  the sway; 4.2 µs with the roll as well, both in one re-solve.

## Revisit when

- A rig with different feet is used: the margin table depends on the foot's
  inner border (3.8 cm inside the sole centreline on `puppet_base`).
- Walks slower than 0.7 m/s matter: the sway grows as the stride slows,
  and 13 cm leaves only 4 mm at 0.7 m/s.
- A pelvic yaw is added (Winter §7.4.5). The roll is already in the
  medial-margin test, which runs the whole walking pelvis move.

## Related

- [11.3.1 The inverted pendulum in steady walking](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.1-inverted-pendulum-in-steady-walking.md) — source: the medial-of-the-foot constraint.
- [11.2.1 Quiet standing](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.2-standing-balance-ml-and-ap/11.2.1-quiet-standing.md) — source: Eq. 11.3 and K.
- [The walk's pelvis rides one sinusoid per step](./walk-pelvis-rides-one-sinusoid-per-step.md) — context: the vertical counterpart.
- [A lagging pelvis rotation slides planted feet](./a-lagging-pelvis-rotation-slides-planted-feet.md) — context: why pelvis moves over planted feet are checked on the sprung pose.
- [The walking pelvis's roll](./walking-pelvic-obliquity-from-hip-abductor-power.md) — applies: the roll applied in the same pass (`move_pelvis_over_feet`).
- [Rig authored at critical extension](./rig-authored-at-critical-extension.md) — why the trailing leg cannot absorb the last fraction of a millimetre.
