---
title: Winter — Biomechanics and Motor Control of Human Movement
description: "Chapter-by-chapter digest of Winter's 4th ed. (2009): signal processing, kinematics, anthropometry, inverse dynamics, energetics, 3D rotations, forward dynamics, muscle, EMG, balance synergies, and a full walking-trial data set, mapped to migera's anim stack. Read before gait, balance, ragdoll or muscle work."
type: index
status: current
tags:
  - biomechanics
  - character-animation
  - locomotion
  - balance
  - inverse-dynamics
  - muscle
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, D. A., Biomechanics and Motor Control of Human Movement, 4th ed., Wiley, 2009, ISBN 978-0-470-39818-0 (docs/books/bmcoh/)"
aliases:
  - Winter
  - bmcoh
  - Biomechanics and Motor Control of Human Movement
---

# Winter — Biomechanics and Motor Control of Human Movement

> **Source:** David A. Winter, *Biomechanics and Motor Control of Human
> Movement*, 4th ed., Wiley 2009 ·
> [open PDF (contents)](../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=3) ·
> Up: [Knowledge Index](../INDEX.md)

**Reading the PDF:** PDF page = printed page + 13 throughout the book (front
matter is 13 pages). Every note below cites the printed pages and links the
PDF page directly, so any summary can be expanded back to the full text.
Layout: this domain uses the *book digest* exception in
[AGENTS.md §8](../AGENTS.md#8-the-index-hierarchy): chapter folders → section
folders (their `INDEX.md` is the section summary) → one note per subsection.

## The book in five minutes

Winter treats the human body as a **link-segment model**: rigid segments of
known mass, centre of mass (COM) and moment of inertia, joined by frictionless
pin joints and driven by **net joint moments** that stand in for all muscles
crossing a joint. The book is the pipeline around that model.

1. **Measure and clean the motion** (Ch. 2–3). Marker positions carry ~1 mm
   noise; differentiating multiplies harmonic *n* by *n* (velocity) and *n²*
   (acceleration), so data is low-pass filtered first — a 2nd-order
   Butterworth run forward and backward (zero lag), cutoff ~6 Hz for walking
   chosen by residual analysis. Gait is periodic: ~7 harmonics hold 99.7 % of
   the power; strides are compared after normalizing time to 0–100 %.
2. **Give the segments mass** (Ch. 4). Dempster's Table 4.1 gives each
   segment's mass fraction, COM position and radii of gyration as fractions
   of body mass and segment length; the parallel-axis theorem moves inertias.
3. **Inverse dynamics** (Ch. 5, 7). With kinematics + ground reaction force,
   solve Newton–Euler per segment from the foot upward to get joint reaction
   forces and moments, in 2D and in 3D (Cardan angles, Euler's equations).
4. **Energetics** (Ch. 6). Joint power P = M·ω: positive = concentric
   generation, negative = eccentric absorption. In walking the ankle push-off
   is the dominant burst; energy also flows between segments without cost.
5. **Forward dynamics** (Ch. 8). Moments in, motion out, via Lagrange's
   equations. Open-loop torque playback drifts and collapses in ~0.5 s — only
   continuous feedback correction keeps a model upright.
6. **Muscles** (Ch. 9–10). A twitch is a critically damped second-order
   impulse response (T ≈ 40–110 ms); force depends on length (force-length),
   velocity (Hill hyperbola, eccentric plateau 1.1–1.8 × F₀) and activation;
   the EMG linear envelope is the same low-pass.
7. **Synergies and balance** (Ch. 11). The CNS controls task-level sums, not
   single joints: the support moment Ms = Mh + Mk + Ma stays consistent while
   hip and knee trade off. Balance is the COP steering the COM
   (COP − COM ∝ −COM acceleration), by the ankles front-to-back and the hips
   side-to-side; walking balance is foot placement, with scripted
   anticipatory COP shifts to start and stop.

Appendix A is a **complete measured walking stride** (raw markers through
joint powers) — the reference gait for validating migera's procedural walk.

## Key facts

- Dempster segment parameters (mass, COM, radius of gyration per segment), verified against the page — [4.1.3](./ch04-anthropometry/4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md).
- Reference stride: 0.987 s, 61 % stance, 1.414 m, ~1.43 m/s; knee peaks 66.6° in swing; hip marker bob 4.8 cm; toe clearance 1.52 cm — [Appendix A](./appendices/a-walking-trial-kinematic-kinetic-energy-data.md).
- Differentiation amplifies noise by n and n²; filter with a dual-pass Butterworth (C = 0.802) before differentiating — [3.4.3](./ch03-kinematics/3.4-processing-raw-kinematic-data/3.4.3-velocity-and-acceleration-problems.md), [3.4.4](./ch03-kinematics/3.4-processing-raw-kinematic-data/3.4.4-smoothing-and-curve-fitting.md).
- The book's Cardan x–y′–z″ matrix is the transpose of glam/scipy intrinsic `XYZ`; Euler-angle rates are not ω — [7.1.2](./ch07-three-dimensional-kinematics-and-kinetics/7.1-axes-systems/7.1.2-local-reference-systems-and-rotation.md), [7.3](./ch07-three-dimensional-kinematics-and-kinetics/7.3-segment-angular-velocity-and-acceleration.md).
- COP − COM = −(I/Wh)·COM̈: the COP must overshoot the COM to reverse it — [5.2.9](./ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.9-inverted-pendulum-model.md), [11.2.1](./ch11-biomechanical-movement-synergies/11.2-standing-balance-ml-and-ap/11.2.1-quiet-standing.md).
- Support moment is steady (CV 20 %) while hip and knee vary 60–68 %; Ch. 5 and Ch. 11 write it with different signs — [11.1](./ch11-biomechanical-movement-synergies/11.1-support-moment-synergy/INDEX.md).
- Open-loop forward simulations fall within ~500 ms; feedback is required — [8.1](./ch08-synthesis-forward-solutions/8.1-review-of-forward-solution-models.md).
- A twitch / EMG linear envelope is a critically damped low-pass, i.e. a migera critical spring with halflife T·ln 2 — [9.0.5](./ch09-muscle-mechanics/9.0-motor-units-and-twitches/9.0.5-muscle-twitch.md), [10.3.2](./ch10-kinesiological-electromyography/10.3-processing-the-emg/10.3.2-linear-envelope.md).
- Gait initiation: the COP first moves backward and toward the swing leg before the first step — [11.3.2](./ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.2-gait-initiation.md).
- Ankle push-off is the dominant positive power burst (+272 W just before toe-off) — [6.3.1](./ch06-work-energy-and-power/6.3-internal-and-external-work-calculation/6.3.1-internal-work-calculation.md).

## Chapters

| Chapter | What it establishes | Read when |
|---|---|---|
| [Ch. 1 — Biomechanics as an interdiscipline](./ch01-biomechanics-as-an-interdiscipline/INDEX.md) | Measure → describe → analyze → assess; inverse solution vs synthesis; the joint moment as the control signal; map of the book | first, to choose chapters; when deciding what counts as verification vs description |
| [Ch. 2 — Signal processing](./ch02-signal-processing/INDEX.md) | Correlation (lags), Fourier content of gait (~7 harmonics), filtering, ensemble averaging over % stride, waveform CV | comparing, testing or compactly representing periodic gait curves |
| [Ch. 3 — Kinematics](./ch03-kinematics/INDEX.md) | Signed angle conventions, noise amplification, zero-lag Butterworth, residual analysis, finite differences | smoothing/differentiating motion, sign-checking joint angles |
| [Ch. 4 — Anthropometry](./ch04-anthropometry/INDEX.md) | Segment lengths per height, full Table 4.1, whole-body COM, parallel axis, muscle PCSA/stress/moment arms | setting ragdoll masses, COMs, inertias or joint strengths; checking rig proportions |
| [Ch. 5 — Kinetics: forces and moments](./ch05-kinetics-forces-and-moments/INDEX.md) | Planar Newton–Euler inverse dynamics, force plates, COP, moment curves, COM vs COP, inverted pendulum | joint-torque budgets, balance, gait plausibility oracles |
| [Ch. 6 — Work, energy and power](./ch06-work-energy-and-power/INDEX.md) | Joint power, generation vs absorption, energy transfer, pendular exchange, causes of inefficiency | judging a gait by energy, locating push-off, auditing ragdoll energy |
| [Ch. 7 — 3D kinematics and kinetics](./ch07-three-dimensional-kinematics-and-kinetics/INDEX.md) | Frames, Cardan sequences, gimbal lock, angular velocity, 3D Newton–Euler with gyroscopic terms, 3D gait curves | decomposing bone rotations, reasoning about 3D ragdoll dynamics |
| [Ch. 8 — Synthesis: forward solutions](./ch08-synthesis-forward-solutions/INDEX.md) | Six requirements for forward models, why they drift, spring/damper joints and feet, Lagrangian recipe, 3-link example | designing or debugging the active ragdoll |
| [Ch. 9 — Muscle mechanics](./ch09-muscle-mechanics/INDEX.md) | Motor units, twitch, force-length, force-velocity (Hill), muscle–load equilibrium, EMG-driven model | giving actuators activation lag or angle/speed-dependent strength |
| [Ch. 10 — Kinesiological EMG](./ch10-kinesiological-electromyography/INDEX.md) | EMG physiology and recording (low relevance); linear envelope, EMG–force lag and nonlinearity, fatigue | activation dynamics, fatigue as a gameplay parameter |
| [Ch. 11 — Movement synergies](./ch11-biomechanical-movement-synergies/INDEX.md) | Support moment, standing balance A/P and M/L, walking balance, gait initiation and termination | balance, start/stop transitions, pelvis sway, ragdoll support |
| [Appendices](./appendices/INDEX.md) | A: complete walking-trial data set (Tables A.1–A.7) with sign conventions; B: SI units | need reference gait numbers, a known-answer fixture, or a unit |

## Applying the book to migera

The digest's "Relevance to migera" sections converge on these opportunities
for `src/character/anim`. They are researched ideas, not built features;
each links the note that holds the evidence.

| Area | What the book offers | Notes |
|---|---|---|
| Walk validation | **Built (2026-09-29):** the walk replays Appendix A as 7-harmonic curves and is scored against it on the real rig — see [replaying a recorded gait](../character-animation/ik-and-locomotion/replay-a-recorded-gait-by-segment-attitudes.md) and [the pelvis trade-off](../character-animation/ik-and-locomotion/recorded-pelvis-path-and-leg-angles-conflict.md) | [Appendix A](./appendices/a-walking-trial-kinematic-kinetic-energy-data.md), [2.3.2](./ch02-signal-processing/2.3-ensemble-averaging/2.3.2-time-base-normalization-to-100-percent.md), [2.3.3](./ch02-signal-processing/2.3-ensemble-averaging/2.3.3-variability-about-mean-waveform.md), [2.1.6](./ch02-signal-processing/2.1-auto-and-cross-correlation/2.1.6-digital-correlation-implementation.md) |
| Pelvis/COM motion | Upper-body height peaks at midstance, forward speed peaks in double support (antiphase, ~±12 % speed); COM passes medial of the stance foot, never over it | [6.2.1](./ch06-work-energy-and-power/6.2-forms-energy-storage/6.2.1-segment-energy-and-within-segment-exchange.md), [11.3.1](./ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.1-inverted-pendulum-in-steady-walking.md) |
| Start/stop transitions | Anticipatory COP shift before the first step; braking over two steps with a ~half-length final step | [11.3.2](./ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.2-gait-initiation.md), [11.3.3](./ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.3-gait-termination.md) |
| Ragdoll mass properties | Per-body COM at the Table 4.1 fraction and I = m(ρ₀L)² instead of a centred uniform capsule | [4.1.3](./ch04-anthropometry/4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md), [4.1.7](./ch04-anthropometry/4.1-density-mass-inertial-properties/4.1.7-anthropometric-tables-with-kinematic-data.md) |
| Ragdoll torque budgets | Measured joint-moment magnitudes (ankle push-off ≫ knee/hip; per-kg curves); strength asymmetry from PCSA | [5.2.5](./ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.5-combined-force-plate-and-kinematics.md), [7.4.5](./ch07-three-dimensional-kinematics-and-kinetics/7.4-kinetic-analysis-reactions-and-moments/7.4.5-sample-moment-and-power-curves.md), [4.3.1](./ch04-anthropometry/4.3-muscle-anthropometry/4.3.1-muscle-cross-sectional-area.md) |
| Muscle-like actuation | Activation lag via a critical spring on strength; Hill force-velocity as a velocity-dependent torque ceiling; parallel elasticity as soft joint limits | [9.0.5](./ch09-muscle-mechanics/9.0-motor-units-and-twitches/9.0.5-muscle-twitch.md), [9.2.1](./ch09-muscle-mechanics/9.2-force-velocity-characteristics/9.2.1-concentric-contractions.md), [9.2.2](./ch09-muscle-mechanics/9.2-force-velocity-characteristics/9.2.2-eccentric-contractions.md), [9.1.2](./ch09-muscle-mechanics/9.1-force-length-characteristics/9.1.2-parallel-connective-tissue.md) |
| Forward-dynamics caveats | Feedback is mandatory; per-body torques without parent reaction are external moments (add net angular momentum if the root is freed) | [8.0.1](./ch08-synthesis-forward-solutions/8.0-introduction/8.0.1-forward-model-assumptions-and-constraints.md), [8.4](./ch08-synthesis-forward-solutions/8.4-external-forces-torques.md), [6.4.1](./ch06-work-energy-and-power/6.4-power-balances/6.4.1-energy-transfer-via-muscles.md) |
| Balance controller | COM PD → desired COM acceleration → COP target, realized by ankle torque (A/P) and hip load/unload (M/L) | [11.2.1](./ch11-biomechanical-movement-synergies/11.2-standing-balance-ml-and-ap/11.2.1-quiet-standing.md), [11.2.2](./ch11-biomechanical-movement-synergies/11.2-standing-balance-ml-and-ap/11.2.2-ml-balance-in-workplace-tasks.md), [5.2.8](./ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.8-center-of-mass-vs-center-of-pressure.md) |
| Test oracles | Worked examples and problems with printed answers (planar and 3D inverse dynamics, anthropometry, 3-link Lagrangian) | [5.1](./ch05-kinetics-forces-and-moments/5.1-link-segment-equations-free-body-diagram.md), [7.4.3](./ch07-three-dimensional-kinematics-and-kinetics/7.4-kinetic-analysis-reactions-and-moments/7.4.3-kinetic-data-set-example.md), [8.6](./ch08-synthesis-forward-solutions/8.6-illustrative-example.md), [4.4](./ch04-anthropometry/4.4-problems-anthropometric-data.md) |
| Angle conventions | Every angle needs a zero, sign and viewing axis; Appendix A's ankle angle is dorsiflexion-positive, opposite to §3.5.2 | [3.1.1](./ch03-kinematics/3.1-kinematic-conventions/3.1.1-absolute-spatial-reference-system.md), [3.5.2](./ch03-kinematics/3.5-other-kinematic-variables/3.5.2-joint-angles.md) |

## Trust notes

- **The book has many misprints.** The notes record each one they found in a
  typo/errata table and give the corrected form, checked numerically where
  possible (numpy/scipy). The worst-affected are Ch. 2 (Eqs. 2.9, 2.16–2.17),
  Ch. 7 (worked example factors) and Ch. 8 (Eqs. 8.19–8.22 and the 3-link
  example). Where a note says "corrected", trust the note over the page.
- Values read off plots are marked approximate; content derived by the
  digest rather than printed in the book is labelled as such.
- Claims about migera code were checked against the source on 2026-09-28;
  re-verify before acting on them (`python3 tools/kb.py stale` does not
  track these notes because they carry no `code:` field).

## See also

- [Character animation](../character-animation/INDEX.md) — migera's animation stack that this book informs; its notes hold the implementation lessons.
- [Ragdoll and physics](../character-animation/ragdoll-and-physics/INDEX.md) — the PD/avian ragdoll that Ch. 4, 8 and 9 apply to.
- [IK and locomotion](../character-animation/ik-and-locomotion/INDEX.md) — gait, stance and foot IK that Ch. 5, 6 and 11 apply to.
