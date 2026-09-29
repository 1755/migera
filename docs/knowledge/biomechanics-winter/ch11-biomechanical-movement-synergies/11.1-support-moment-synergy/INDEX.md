---
title: 11.1 The support moment synergy
description: "Defines the support moment Ms = Mh + Mk + Ma (extensor +ve; = Mk − Ma − Mh in ch. 5 signs). Stance Ms is repeatable (CV 20%) while hip and knee trade off (CV 68%/60%, covariance 89%), and Ms tracks vertical GRF (r = 0.97). Read before judging leg torques, stance knee bend or ragdoll leg support."
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - locomotion
  - ragdoll
  - verification
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §11.1, pp. 282–286 (PDF pp. 295–299)"
aliases:
  - support moment
  - Ms
  - hip-knee trade-off
  - total limb extensor synergy
---

# 11.1 The support moment synergy

> **Source:** Winter (2009) §11.1, pp. 282–286 ·
> [open PDF at p. 282](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=295) ·
> Up: [Chapter 11 — Biomechanical movement synergies](../INDEX.md)

The **support moment** Ms is the sum of the hip, knee and ankle extensor
moments of the stance limb: how hard the whole leg is pushing away from the
ground. Across repeat walks, each joint's moment varies a lot, but the sum
is steady. The hip and knee trade off against each other, so the CNS appears
to control the total and not each joint. Ms also has the same double-hump
shape as the vertical ground reaction force (§11.1.1).

## The section's own content (pp. 282–284)

**Definition and sign convention (a trap).** The concept comes from Winter
(1980) and was introduced in §5.2.6. Two sign conventions appear in the book
for the *same* quantity:

| Where | Convention | Formula |
|---|---|---|
| §5.2.6 / Fig. 5.14 | the moment convention of ch. 5 | $M_s = M_k - M_a - M_h$ |
| §11.1 / Fig. 11.1 | **extensor moment +ve at every joint** (ankle "extensor" = plantarflexor) | $M_s = M_h + M_k + M_a$ |

Both mean "sum of the three extensor moments". Only the per-joint sign flips
differ. Before summing torques from any rig or simulation, convert each joint
to "extensor positive". The knee extends in the opposite rotational sense
from the hip and ankle in the sagittal plane.

**Repeat-trial data (Fig. 11.1).** One subject (WM22) walked at natural
cadence on 9 separate days. Profiles are in N·m/kg, toe-off is forced to 60%
of stride, and CVs are computed over stance.

- Kinematics were very consistent: rms s.d. over the stride was 1.5° at the
  ankle, 1.9° at the knee and 1.8° at the hip. Cadence stayed within 2%.
- Moments during stance: hip CV = **68%**, knee CV = **60%**, ankle CV = 18%.
  Hip+knee CV = **21%**, support Ms CV = **20%**. During swing, Mh and Mk
  barely vary.
- Shape: Ms is positive (extensor) through stance with two peaks, reaching
  about 1–1.5 N·m/kg. The ankle plantarflexor moment peaks near 1.6 N·m/kg
  late in stance (about 45–50% of stride).
- Interpretation: on a day when the hip is more extensor, the knee is more
  flexor, and the reverse on another day. Near-identical joint *angles* hide
  very different joint *moment* distributions.

**Covariance analysis (Eq. 11.1, 11.2, Fig. 11.2).** All quantities are
averaged over stance, in (N·m)².

$$\sigma^2_{hk} = \sigma^2_h + \sigma^2_k - \sigma^2_{h+k} \qquad (11.1)$$

$$\mathrm{COV} = \frac{\sigma^2_{hk}}{\sigma^2_h + \sigma^2_k} \times 100\% \qquad (11.2)$$

- $\sigma^2_h,\ \sigma^2_k$: mean hip and knee variance over stance.
  $\sigma^2_{h+k}$: variance of the summed profile. $\sigma^2_{hk}$: Winter's
  "covariance" term.
- COV = 100% means $\sigma^2_{h+k}=0$: the day-to-day changes cancel exactly.
- Note: by the usual identity
  $\mathrm{var}(h+k) = \sigma_h^2 + \sigma_k^2 + 2\,\mathrm{cov}(h,k)$,
  Winter's $\sigma^2_{hk}$ equals $-2\,\mathrm{cov}(h,k)$. It is a measure of
  **negative** (compensating) covariance, which is why it is positive here.

Fig. 11.2 values (N·m, shown as squares). These were re-checked by
arithmetic, and each COV reproduces:

| | $\sigma_h$ | $\sigma_k$ | $\sigma_a$ | $\sigma_{h+k}$ | $\sigma_{a+k}$ | $\sigma_{hk}$ (COV) | $\sigma_{ak}$ (COV) |
|---|---|---|---|---|---|---|---|
| Day-to-day (9 days) | 15.9 | 12.6 | 10.5 | 6.9 | 8.1 | 19.1 (**89%**) | 14.3 (76%) |
| Trial-to-trial (2nd subject, 10 trials minutes apart) | 5.5 | 5.4 | 5.9 | 4.1 | 5.7 | 6.5 (72%) | 5.6 (49%) |

The caption says 75% for knee–ankle. The figure and text say 76%.
Trial-to-trial covariance is lower mainly because the individual variances
are much smaller over minutes than over days.

## Key facts

- $M_s = M_h + M_k + M_a$ with extensor +ve. In ch. 5's signs this is $M_k - M_a - M_h$ (this INDEX, from p. 282).
- Hip and knee moments vary 60–68% trial to trial, their sum only 21%, Ms 20% (this INDEX, Fig. 11.1).
- The hip–knee compensation reaches 89% of its maximum possible value over days (this INDEX, Eq. 11.2).
- Ms has the double-hump shape of vertical GRF, r = 0.97 at natural cadence ([11.1.1](./11.1.1-support-moment-vs-vertical-grf.md)).
- The Ms–Fy correlation holds at fast (r = 0.95) and slow (r = 0.90) cadence and in knee-replacement patients (r = 0.92–0.96) ([11.1.1](./11.1.1-support-moment-vs-vertical-grf.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [11.1.1 Relationship between Ms and the vertical ground reaction force](./11.1.1-support-moment-vs-vertical-grf.md) | Ms and Fy share the double-hump profile (r = 0.90–0.97 across cadences and pathology), so Ms measures how hard the limb pushes down | when deriving a support-torque profile, vertical bob, or a GRF-shaped test oracle for a stance leg |

## Relevance to migera

- **Assert on the sum, not the joints.** For the active ragdoll (`ragdoll.rs`,
  `math/pd.rs`), a stance leg's hip and knee PD torques may legitimately
  redistribute between runs, speeds or strengths. A test or BRP check should
  require the *extensor-summed* leg torque to stay positive and double-humped
  through stance. It should not pin each joint's torque. Before summing,
  convert every joint to extensor-positive. Hardcoded signs have already bitten
  this project (see the knee-axis and unsigned-measurement lessons below).
- **Knee bend is a style knob, not a support knob.** The bent-knee stance
  (`stance.rs`) and the gait's stance-knee curve (`gait.rs`) can vary between
  characters. Real people carry the same load with different hip/knee splits,
  day to day, with nearly identical joint angles. A procedural personality
  variation can move flexion between hip and knee and keep pelvis height and
  support.
- **Honest limit:** the kinematic stack computes no torques, so Ms applies
  only to the physics side (ragdoll) or to deriving motion shapes
  (§11.1.1's vertical-force profile).

## Where to read in the book

- p. 282 (PDF 295): definition, the change of sign convention, repeat-trial
  kinematics and the CVs.
- p. 283 (PDF 296): **Fig. 11.1**, five stacked moment profiles (support,
  hip, hip+knee, knee, ankle), plus Eq. 11.1.
- p. 284 (PDF 297): **Fig. 11.2**, variance/covariance diagram and Eq. 11.2.
- pp. 285–286 (PDF 298–299): §11.1.1 and Fig. 11.3.

## Related

- [5.2.6 Interpreting moment-of-force curves](../../ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.6-interpreting-moment-of-force-curves.md) — prerequisite: the three stance moments and the ch. 5 sign convention ($M_s = M_k - M_a - M_h$).
- [Appendix A: walking trial data](../../appendices/a-walking-trial-kinematic-kinetic-energy-data.md) — applies: the joint moments needed to compute an Ms curve as a test fixture.
- [KNEE_AXIS positive swings forward](../../../character-animation/rig-and-retargeting/knee-axis-positive-swings-forward.md) — same-trap: joint sign conventions differ per joint and per rig facing; convert before summing.
- [Unsigned measurements cannot see direction](../../../engineering-practice/testing/unsigned-measurements-cannot-see-direction.md) — same-trap: a magnitude-only torque check cannot tell an extensor moment from a flexor one.
- [Ragdoll and physics](../../../character-animation/ragdoll-and-physics/INDEX.md) — applies: where a support-torque invariant would be checked.
