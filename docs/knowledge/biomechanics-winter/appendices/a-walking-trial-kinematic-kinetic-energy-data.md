---
title: Appendix A — Kinematic, kinetic, and energy data (walking trial)
description: Complete 2D sagittal data set for one right-side walking stride (56.7 kg, 69.9 Hz, 106 frames, HCR 28/97, TOR 1/70) — markers, segment and joint angles, reaction forces, moments, energies, powers. Read when you need a reference gait curve or a known-answer test fixture for a walk cycle.
type: reference
status: current
tags:
  - biomechanics
  - locomotion
  - inverse-dynamics
  - energetics
  - verification
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Appendix A, pp. 296–360 (PDF pp. 309–373)"
  - "Same book, Preface to the 4th ed., p. xiv (PDF p. 13): tables also at http://www.wiley.com/go/biomechanics"
aliases:
  - Winter gait data
  - Winter appendix A
  - reference walking trial
---

# Appendix A — Kinematic, kinetic, and energy data (walking trial)

Contents: [Trial](#the-trial) · [Conventions](#coordinate-and-sign-conventions) ·
[Tables](#table-map) · [Key ranges](#key-ranges-over-one-stride-hcr-28--hcr-97) ·
[Reference curve](#reference-curve-every-third-frame) · [Pages](#where-to-read-in-the-book) ·
[Relevance](#relevance-to-migera)

> **Source:** Winter (2009) Appendix A, pp. 296–360 ·
> [open PDF at p. 296](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=309) ·
> Up: [Appendices](./INDEX.md)

One subject walking at about 1.43 m/s, filmed from the right side and taken
all the way through a 2D inverse-dynamics pipeline. The appendix is
fourteen tables with no prose: raw markers, filtered markers with
velocities and accelerations, segment kinematics, joint angles, joint
reaction forces and moments, segment energies, and joint powers, for the
same 106 frames. Every chapter's problems use it. For migera it is a
self-consistent reference gait: a known input and a known answer at each
stage of the pipeline. The Preface (p. xiv, PDF 13) says the same tables
can be downloaded from `http://www.wiley.com/go/biomechanics`.

## The trial

From Figure A.1 (p. 296, PDF 309) and the tables:

| Item | Value |
|---|---|
| Body mass | **56.7 kg** |
| Frame rate | **69.9 frames/s** (Δt = 0.0143 s; the TIME column steps 0.014/0.015) |
| Frames | **1–106** (t = 0.000–1.501 s). A.7 covers only frames 2–70 |
| Events (right foot) | **TOR** (toe-off right) frames **1** and **70**; **HCR** (heel contact right) frames **28** and **97** |
| Stride | HCR 28 → HCR 97 = 69 frames = **0.987 s** (the book's problems also say "one stride period is 69 frames") |
| Stance / swing | stance 28–70: 42 frames, 0.601 s, **61 %**; swing 70–97: 27 frames, 0.386 s, 39 % |
| Stride length | heel X at HCR 28 = 1.228 m, at HCR 97 = 2.642 m → **1.414 m** (derived) |
| Speed | 1.414 m / 0.987 s ≈ **1.43 m/s**; mean rib-cage VX over the stride 1.41 m/s (derived) |
| Plane | 2D sagittal, right side only. X = progression, Y = vertical |

**Markers** (all on the right side), Fig. A.1: base of rib cage, greater
trochanter (hip), lateral epicondyle of femur (knee), head of fibula,
lateral malleolus (ankle), heel, fifth metatarsal, toe. Distances printed
on the figure: rib cage–hip 25 cm, hip–knee 31.4 cm, knee–ankle 42.5 cm,
fibula–ankle 33.6 cm, ankle–5th metatarsal 12.2 cm. The hip–knee value
checks against frame 1 of Table A.1 (31.4 cm).

**Segments** (Tables A.3, A.5, A.6). The segment angles in A.3 reproduce
from the filtered markers of A.2 to within 0.08° when each segment is taken
as the line from its distal to its proximal marker:

| Segment | Line (distal → proximal) | Mass (derived) |
|---|---|---|
| Foot | 5th metatarsal → lateral malleolus | 0.0145·M = 0.82 kg |
| Leg | lateral malleolus → femoral epicondyle | 0.0465·M = 2.64 kg |
| Thigh | femoral epicondyle → greater trochanter | 0.100·M = 5.67 kg |
| ½ HAT | greater trochanter → base of rib cage | 0.678·M/2 = 19.2 kg |

The masses are Winter's Table 4.1 fractions. They reproduce the potential
energies of Table A.6 (thigh frame 1: 5.67·9.81·0.652 = 36.3 J, as
tabulated; ½ HAT frame 50: 211.0 J, as tabulated; leg within about 1.5 %).
HAT (head, arms, trunk) is halved because only one side of the body is
analysed.

## Coordinate and sign conventions

- **Axes**: X horizontal in the direction of progression, Y vertical up,
  viewed from the subject's right side. The subject walks toward +X.
  (Book §3.1.1.)
- **Units**: Table A.1 is in **cm**. All later tables are SI: m, m/s,
  m/s², degrees for angles, rad/s and rad/s² for ω and α, N, N·m, J, W.
- **Segment angles (A.3)**: absolute, measured counterclockwise from +X.
  A vertical segment is 90°. ω and α are positive counterclockwise.
- **Joint angles (A.4)**, reconstructed from the tables (not printed in the
  appendix; checked to within 0.07° over all 106 frames):
  - hip = θ_thigh − θ_HAT. Positive = **flexion**.
  - knee = θ_thigh − θ(fibula head → malleolus). Positive = **flexion**.
    The knee uses the fibula line, not the A.3b leg line, so
    θ_thigh − θ_leg(A.3b) differs from A.4 by up to about 7°.
  - ankle = θ(heel → 5th metatarsal) − θ(fibula head → malleolus) + 90°.
    Positive = **dorsiflexion**, negative = plantarflexion. This is the
    opposite polarity to the formula printed in §3.5.2 (which calls positive
    plantarflexion). The data (−20° at toe-off) confirm the table's
    polarity.
- **Forces and moments (A.5)**: each is the force or moment *acting on the
  named segment*, counterclockwise positive. The two sides of a joint are
  equal and opposite (foot-segment ankle RX = −leg-segment ankle RX, and
  so on). In anatomical terms this means:
  - ankle moment on the **foot**: negative = plantarflexor;
  - knee moment on the **leg**: positive = extensor;
  - hip moment on the **thigh**: negative = extensor.

  Winter's support-moment plots (Ch. 11) use "extensor positive" at every
  joint, so flip signs to match them.
- **Ground reaction** (A.5a): RX and RY acting on the foot. RX < 0 is
  braking. CofP X is the absolute X of the centre of pressure, and 0.000
  whenever the foot is off the ground.
- **Powers (A.7)**: muscle power = M·ω_joint. Positive = generation
  (concentric), negative = absorption (eccentric).

## Table map

| Table | Printed pp. (PDF) | Content | Columns (after FRAME, TIME s) |
|---|---|---|---|
| Fig. A.1 | 296 (309) | Marker sketch, body mass, frame rate, event legend | — |
| A.1 | 297–300 (310–313) | Raw coordinate data, **cm** | X, Y for rib cage, hip, knee, fibula, ankle, heel, metatarsal, toe (16) |
| A.2(a) | 301–305 (314–318) | Filtered marker kinematics: rib cage, greater trochanter | per marker X m, VX m/s, AX m/s², Y, VY, AY (12) |
| A.2(b) | 306–310 (319–323) | Filtered: femoral lateral epicondyle (knee), head of fibula | same 12 |
| A.2(c) | 311–315 (324–328) | Filtered: lateral malleolus (ankle), heel | same 12 |
| A.2(d) | 316–320 (329–333) | Filtered: fifth metatarsal, toe | same 12 |
| A.3(a) | 321–325 (334–338) | Linear and angular kinematics, foot | THETA deg, OMEGA rad/s, ALPHA rad/s², CofM-X m, VEL-X, ACC-X, CofM-Y, VEL-Y, ACC-Y (9) |
| A.3(b) | 326–330 (339–343) | Same, leg | same 9 |
| A.3(c) | 331–335 (344–348) | Same, thigh | same 9 |
| A.3(d) | 336–340 (349–353) | Same, ½ HAT | same 9 |
| A.4 | 341–345 (354–358) | Relative joint angular kinematics | THETA deg, OMEGA, ALPHA for ankle, knee, hip (9) |
| A.5(a) | 346–349 (359–362) | Reaction forces and moments, ankle and knee | Foot: ground RX, RY N, ankle RX, RY N, ground CofP X m, ankle moment N·m. Leg: ankle RX, RY, knee RX, RY, ankle moment, knee moment (12) |
| A.5(b) | 350–352 (363–365) | Reaction forces and moments, hip | Thigh: knee RX, RY, hip RX, RY N, knee moment, hip moment N·m (6) |
| A.6 | 353–357 (366–370) | Segment energies | PE, TKE, RKE, TOTAL J for foot, leg, thigh, HAT (16) |
| A.7 | 358–360 (371–373) | Power generation/absorption and transfer, frames 2–70 only | Muscle power ankle, knee, hip W; rate of transfer Joint and Muscle W for leg→foot, thigh→leg, pelvis→thigh; segment ω foot, leg, thigh, HAT rad/s (13) |

Table layout: A.4 to A.7 and most A.2/A.3 pages are printed rotated 90°.
Event labels (TOR/HCR) sit in a left margin column. `pdftotext -layout`
extracts every row cleanly: all 106 frames of every table parse with a
constant column count, using U+2212 as the minus sign.

### Representative rows (checked against the rendered pages)

A.4 frame 1 (TOR, p. 341): ankle −15.2° / −2.29 rad/s / 94.89 rad/s²,
knee 46.7 / 6.74 / −21.91, hip −2.4 / 2.39 / 36.03.

A.4 frame 77 (p. 344): ankle −9.9°, knee **66.6°** (swing peak), hip 15.3°.

A.2(a) frame 28 (HCR, p. 302): rib cage X 0.9896 m, VX 1.61, AX 1.5,
Y 1.0367, VY −0.14, AY 5.0. Hip X 1.0183, VX 1.70, AX −0.4, Y 0.7959,
VY −0.03, AY 4.4.

A.5(a) frame 37 (p. 347): ground RX −98.1 N, RY **604.5 N**; foot ankle RX
97.1, RY −595.7; CofP 1.316 m; ankle moment (foot) 0.2. Leg: ankle RX −97.1,
RY 595.7; knee RX 80.9, RY −573.4; ankle moment −0.2, knee moment **37.8**.

A.5(b) frame 28 (HCR, p. 350): knee RX 64.3, RY 50.2; hip RX −66.1,
RY 29.7; knee moment 33.8, hip moment **−54.4** N·m.

A.7 frame 64 (p. 360): ankle **272.4 W**, knee −38.9, hip −2.6. Leg→foot
joint −387.8, muscle 225.0. Thigh→leg joint −24.0, muscle 0.0.
Pelvis→thigh joint 27.1, muscle 18.9. ω foot −7.25, leg −3.28,
thigh 0.56, HAT 0.64.

## Key ranges over one stride (HCR 28 → HCR 97)

Extracted from the parsed tables (frame in brackets; % = % of stride from HCR).

| Quantity | Value |
|---|---|
| Hip angle | 12.8° at HCR → max extension **−6.2°** (f64, 52 %) → max flexion **22.6°** (f85, 83 %); range 28.8° |
| Knee angle | ≈0° at HCR → loading-response peak **16.3°** (f38) → 5.2° mid-stance → 47.6° at TOR → swing peak **66.6°** (f77, 71 %) → −3.0° (f95) |
| Ankle angle | −0.4° at HCR → −7.8° plantarflexion (f33, foot flat) → max dorsiflexion **6.9°** (f53–55) → max plantarflexion **−20.5°** (f71, just after TOR) |
| Vertical GRF | 87 N at HCR → first peak **604.5 N** (f37, 1.09 BW) → trough 362.3 N (f46, 0.65 BW) → second peak **612.1 N** (f60, 1.10 BW). BW = 556 N |
| Horizontal GRF | braking min **−114.2 N** (f35) → propulsive max **+115.6 N** (f63) |
| COP X | 1.227 m at HCR → 1.486 m at f69: travels 0.26 m heel to toe |
| Ankle moment (on foot) | peak **−89.8 N·m** plantarflexor (f61) ≈ 1.58 N·m/kg |
| Knee moment (on leg) | −33.8 N·m flexor at HCR (swing braking) → **+37.8 N·m** extensor (f37) |
| Hip moment (on thigh) | **−54.4 N·m** extensor at HCR → +37.3 N·m flexor (f62) |
| Ankle power | absorption −32.6 W (f50) → push-off generation **272.4 W** (f64) ≈ 4.8 W/kg |
| Knee power | −55.8 W (f35, weight acceptance) → +21.0 W (f40) → −62.9 W (f67, push-off) |
| Hip power | +44 W at HCR, +51.4 W at f2 (early swing pull-off) |
| Pelvis (hip marker) height | 0.785 m (f69) → 0.834 m (f45): **4.8 cm** vertical excursion, peak at mid-stance |
| ½ HAT CofM height | 1.078 m (f68) → 1.128 m (f45): **5.0 cm** |
| ½ HAT CofM forward speed | 1.18 m/s (f77) → 1.66 m/s (f64) |
| Heel height in swing | max **27.4 cm** (f73, early swing) |
| Toe clearance | book answer (Problem 3.6-4): toe lowest 4.85 cm in swing (f13) vs 3.33 cm in late stance (f66) → **1.52 cm** clearance |
| ½ HAT total energy | 218.5 J (f72) – 232.2 J (f34) |

## Reference curve (every third frame)

Joint angles are in degrees (A.4). Moments are in N·m, acting on the named
segment (A.5); see the sign rules above.

| Frame | % stride | Ankle | Knee | Hip | GRF RX N | GRF RY N | M ankle (foot) | M knee (leg) | M hip (thigh) |
|---|---|---|---|---|---|---|---|---|---|
| 28 HCR | 0 | −0.4 | −0.6 | 12.8 | 37.3 | 87.1 | −1.7 | −33.8 | −54.4 |
| 31 | 4 | −6.5 | 5.1 | 11.4 | −74.0 | 404.2 | 2.8 | −4.5 | −20.8 |
| 34 | 9 | −7.2 | 12.1 | 11.5 | −110.5 | 552.9 | 5.0 | 24.8 | −0.4 |
| 37 | 13 | −3.1 | 16.2 | 10.8 | −98.1 | 604.5 | 0.2 | 37.8 | 9.9 |
| 40 | 17 | 0.4 | 15.2 | 7.2 | −48.7 | 516.7 | −6.6 | 24.8 | 4.7 |
| 43 | 22 | 2.8 | 12.3 | 3.5 | −27.0 | 400.3 | −14.1 | 15.8 | 12.5 |
| 46 | 26 | 4.2 | 10.1 | 1.5 | −16.6 | 362.3 | −25.3 | 6.6 | 11.2 |
| 49 | 30 | 5.3 | 8.2 | 0.4 | −14.5 | 386.1 | −33.5 | 5.9 | 17.7 |
| 52 | 35 | 6.7 | 6.4 | −1.2 | 2.2 | 441.0 | −45.4 | −0.1 | 14.6 |
| 55 | 39 | 6.9 | 5.2 | −3.1 | 26.8 | 523.9 | −64.8 | −7.9 | 16.6 |
| 58 | 43 | 6.8 | 6.2 | −4.6 | 68.1 | 595.8 | −82.7 | −12.4 | 18.3 |
| 61 | 48 | 4.8 | 10.9 | −5.6 | 101.5 | 602.3 | −89.8 | −2.2 | 34.4 |
| 64 | 52 | −1.4 | 19.6 | −6.2 | 114.5 | 463.0 | −68.6 | 10.1 | 33.6 |
| 67 | 57 | −11.8 | 32.4 | −5.7 | 65.7 | 190.1 | −24.2 | 12.5 | 15.2 |
| 70 TOR | 61 | −20.1 | 47.6 | −2.5 | 0.0 | 0.0 | 1.4 | 3.6 | 9.0 |
| 73 | 65 | −18.2 | 60.4 | 4.5 | 0 | 0 | 1.3 | 6.4 | 13.6 |
| 76 | 70 | −11.8 | 66.5 | 12.9 | 0 | 0 | 0.7 | 4.3 | 9.4 |
| 79 | 74 | −6.5 | 63.9 | 18.9 | 0 | 0 | 0.5 | 0.6 | 2.0 |
| 82 | 78 | −3.2 | 54.2 | 21.6 | 0 | 0 | 0.6 | −2.2 | −1.4 |
| 85 | 83 | −1.6 | 40.0 | 22.6 | 0 | 0 | 0.8 | −3.6 | −3.0 |
| 88 | 87 | −0.8 | 22.6 | 21.6 | 0 | 0 | 0.6 | −7.4 | −9.4 |
| 91 | 91 | 0.7 | 5.8 | 18.3 | 0 | 0 | −0.2 | −14.5 | −19.0 |
| 94 | 96 | 1.7 | −2.6 | 14.9 | 0 | 0 | −0.6 | −14.5 | −15.7 |
| 97 HCR | 100 | −1.8 | −0.8 | 12.3 | 0 | 0 | 0.0 | −4.9 | −2.9 |

Caveats: this is one young subject at one speed, in 2D, with no
medial/lateral data and no arms. The filtered tables are smoothed (the
method is in §3.4.4), but the appendix does not state the cutoff used.
Frames 1–27 and 98–106 are the neighbouring swing and stance, which is
useful for wrapping a periodic curve.

## Where to read in the book

- p. 296 (PDF 309): Fig. A.1, markers, marker distances, mass, frame rate, TOR/HCR legend.
- pp. 297–300 (PDF 310–313): Table A.1, raw cm coordinates. Compare it with A.2 to see what filtering removes.
- pp. 301–320 (PDF 314–333): Tables A.2(a)–(d), filtered marker X/Y with velocities and accelerations.
- pp. 321–340 (PDF 334–353): Tables A.3(a)–(d), segment angle/ω/α and CofM kinematics.
- pp. 341–345 (PDF 354–358): Table A.4, joint angles. **Start here for a walk-cycle oracle.**
- pp. 346–352 (PDF 359–365): Tables A.5(a)–(b), GRF, COP, joint reaction forces and moments.
- pp. 353–357 (PDF 366–370): Table A.6, segment PE/TKE/RKE/total.
- pp. 358–360 (PDF 371–373): Table A.7, joint powers and energy transfer, frames 2–70.

## Relevance to migera

- **Walk-cycle oracle for `src/character/anim/gait.rs`.** A.4 gives
  sagittal hip/knee/ankle angles against stride phase for a real stride. A
  test can resample the gait's per-phase joint rotations and assert the
  landmarks: knee near 0° at contact, a 10–20° loading-response flexion, a
  60–67° swing peak at about 70 % of the stride; hip flexion peak about 22°
  late in swing and extension about −6° at 50 %; ankle plantarflexed
  about −20° just after toe-off. Use each rig's own facing and
  signed angles (see [unsigned measurements](../../engineering-practice/testing/unsigned-measurements-cannot-see-direction.md)).
- **Timing.** Stance 61 %, swing 39 %, a 0.99 s stride at 1.43 m/s, and a
  1.41 m stride length. These are targets for phase oscillators and for
  when `footlock.rs` locks and releases.
- **Pelvis bob for `pelvis.rs`.** The hip marker rises and falls 4.8 cm
  and peaks at mid-stance (inverted-pendulum vault), twice per stride.
  Forward speed of the trunk swings between 1.18 and 1.66 m/s within a stride.
- **Foot contact.** COP rolls 0.26 m from heel to toe during stance. Toe
  clearance in swing is only about 1.5 cm, and the heel rises 27 cm in
  early swing. These are targets for foot IK and swing-foot trajectories.
- **Ragdoll PD budgets for `ragdoll.rs`.** Net joint moments at walking
  speed for a 56.7 kg body are about 90 N·m (ankle), 38–54 N·m (knee and
  hip). With the A.3 segment masses these give realistic torque ceilings
  for a strength dial. Scale by body mass.
- **Known-answer fixtures.** Chapters 3–6 run their problems on these
  tables, so any inverse-dynamics or energy code migera writes can be
  checked against printed answers (see the Problems notes below).

## Related

- [3.5.2 Joint angles](../ch03-kinematics/3.5-other-kinematic-variables/3.5.2-joint-angles.md) — prerequisite: the printed joint-angle formulas (note the ankle-polarity mismatch above).
- [3.1.1 Absolute spatial reference system](../ch03-kinematics/3.1-kinematic-conventions/3.1.1-absolute-spatial-reference-system.md) — prerequisite: the X/Y/CCW convention every table uses.
- [3.4.4 Smoothing and curve fitting](../ch03-kinematics/3.4-processing-raw-kinematic-data/3.4.4-smoothing-and-curve-fitting.md) — deeper: how A.1 raw becomes A.2 filtered.
- [4.1.3 Segment mass and center of mass](../ch04-anthropometry/4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md) — deeper: the Table 4.1 fractions behind the segment masses.
- [5.1 Link-segment equations — free-body diagram](../ch05-kinetics-forces-and-moments/5.1-link-segment-equations-free-body-diagram.md) — deeper: how A.5 was computed from A.2–A.3.
- [6.4 Power balances](../ch06-work-energy-and-power/6.4-power-balances/INDEX.md) — deeper: how A.7's transfer columns are defined (Example 6.5 uses frame 5).
- [11.1 Support moment synergy](../ch11-biomechanical-movement-synergies/11.1-support-moment-synergy/INDEX.md) — applies: sums the A.5 extensor moments into the support moment.
- [3.6 Problems based on kinematic data](../ch03-kinematics/3.6-problems-kinematic-data.md), [4.4](../ch04-anthropometry/4.4-problems-anthropometric-data.md), [5.4](../ch05-kinetics-forces-and-moments/5.4-problems-kinetic-kinematic-data.md), [6.5](../ch06-work-energy-and-power/6.5-problems-kinetic-kinematic-data.md) — example: ready-made test cases on this data set.
- [B SI units and definitions](./b-si-units-and-definitions.md) — prerequisite: unit definitions for every column.
- [IK and locomotion](../../character-animation/ik-and-locomotion/INDEX.md) — applies: migera's gait, foot-lock and pelvis lessons this data can validate.
- [Replay a recorded gait by segment attitudes](../../character-animation/ik-and-locomotion/replay-a-recorded-gait-by-segment-attitudes.md) — applies: migera's walk replays this stride (`src/character/anim/reference.rs`); read for which columns can drive a rig and which cannot.
