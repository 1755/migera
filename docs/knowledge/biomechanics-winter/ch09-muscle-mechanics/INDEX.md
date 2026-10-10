---
title: Chapter 9 — Muscle mechanics
description: "Winter ch. 9 — motor units and the twitch (critically damped impulse response), force-length (active, parallel, series), force-velocity (Hill, eccentric plateau), muscle–load equilibrium, an EMG-driven model. Read before giving the ragdoll muscle-like actuators (activation lag, angle- or speed-dependent strength)."
type: index
status: current
tags:
  - biomechanics
  - muscle
  - ragdoll
  - springs
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), ch. 9, pp. 224–249 (PDF pp. 237–262)"
aliases:
  - muscle mechanics
  - Hill-type muscle
---

# Chapter 9 — Muscle mechanics

> **Source:** Winter (2009) ch. 9, pp. 224–249 ·
> [open PDF at p. 224](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=237) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

## Chapter in one minute

Muscle force depends on four things, and this chapter covers each: **time**
(a command becomes force through the twitch, a critically damped
second-order impulse response $F_0 (t/T) e^{-t/T}$ with T ≈ 45–116 ms, so a
maximal effort takes ~200 ms to build and ~300 ms to release), **length**
(active force peaks at resting length and falls on both sides; passive
parallel tissue stiffens steeply when stretched; series tissue stretches under
load), **velocity** (Hill's hyperbola $(P + a)(V + b) = (P_0 + a)b$ when
shortening; up to 1.1–1.8 × Fmax when forcibly lengthened), and
**activation**, which scales only the active part. A muscle always sits at
the intersection of its characteristic and its load's. The chapter closes with
a working model: rectified EMG as impulses into a mass–damper–series-spring
system whose ratios all follow from the twitch time T.

**Start here:** for ragdoll work, read
[9.2.1](./9.2-force-velocity-characteristics/9.2.1-concentric-contractions.md)
(Hill), then [9.0.5](./9.0-motor-units-and-twitches/9.0.5-muscle-twitch.md)
(twitch) and [9.3.1](./9.3-muscle-modeling/9.3.1-emg-driven-model-example.md)
(activation filter), then
[9.1.2](./9.1-force-length-characteristics/9.1.2-parallel-connective-tissue.md)
(passive limits).

The chapter's short opening (before §9.0.1) calls the muscle the "living" part
of the system and sets the aim: describe motor units, connective tissue and
whole muscle, and show how unit-level properties shape whole-muscle
mechanics.

## Key facts

- Twitch $F(t) = F_0\,(t/T)\,e^{-t/T}$, peak $F_0/e$ at T; T ≈ 45–79 ms (submaximal) to 116 ms (soleus, supramaximal) ([9.0.5](./9.0-motor-units-and-twitches/9.0.5-muscle-twitch.md)).
- Slow-twitch units peak in 60–120 ms, fast-twitch in 10–50 ms; smallest units are recruited first ([9.0.4](./9.0-motor-units-and-twitches/9.0.4-fast-and-slow-twitch-motor-units.md), [9.0.3](./9.0-motor-units-and-twitches/9.0.3-size-principle.md)).
- Rapid maximal contraction: ~200 ms on, ~300 ms off; force lingers ~150 ms after EMG stops ([9.0.6](./9.0-motor-units-and-twitches/9.0.6-shape-graded-contractions.md)).
- Active force: peak at sarcomere 2.5 µm, zero at 4.0 µm, reduced but nonzero at 1.5 µm ([9.1.1](./9.1-force-length-characteristics/9.1.1-contractile-element-force-length-curve.md)).
- Passive parallel force is zero below rest length, stiffens nonlinearly beyond, and ignores activation ([9.1.2](./9.1-force-length-characteristics/9.1.2-parallel-connective-tissue.md)).
- Series elastic stretch is a few percent (≤ 7%) of rest length; its energy storage is too small to explain prestretch gains ([9.1.3](./9.1-force-length-characteristics/9.1.3-series-elastic-tissue.md)).
- Hill: $(P + a)(V + b) = (P_0 + a)b$, $b = aV_0/P_0$; Vmax ≈ 6 to > 10 l0/s ([9.2.1](./9.2-force-velocity-characteristics/9.2.1-concentric-contractions.md)).
- Eccentric force plateaus at 1.1–1.8 Fmax; level walking is half negative work ([9.2.2](./9.2-force-velocity-characteristics/9.2.2-eccentric-contractions.md)).
- Operating point = muscle–load intersection; ankle stance is negative, negative, then positive (push-off) work ([9.2.4](./9.2-force-velocity-characteristics/9.2.4-muscle-load-equilibrium.md)).
- Critically damped M–B–K muscle: B/M = 2/T, K/B = 1/2T; only an EMG gain is fitted ([9.3.1](./9.3-muscle-modeling/9.3.1-emg-driven-model-example.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [9.0 Introduction](./9.0-motor-units-and-twitches/INDEX.md) | Motor units, recruitment, size principle, twitch, graded-contraction timing | adding activation dynamics or timing strength changes |
| [9.1 Force-length characteristics of muscles](./9.1-force-length-characteristics/INDEX.md) | Active curve, parallel and series elastic tissue, in vivo curves | making strength angle-dependent or adding soft joint limits |
| [9.2 Force-velocity characteristics](./9.2-force-velocity-characteristics/INDEX.md) | Hill equation, eccentric plateau, force surface, muscle–load equilibrium | making torque ceilings velocity-dependent; checking gait work phases |
| [9.3 Muscle modeling](./9.3-muscle-modeling/INDEX.md) | Spring/damper element catalog, Fung equivalence, EMG-driven model | building a muscle-like actuator or activation filter |
| [9.4 References](./9.4-references.md) | Condensed bibliography: Hill, Gordon, Winters, Zahalak, Fuglevand | needing the primary source behind a number |

## Relevance to migera

migera's active ragdoll (`src/character/anim/ragdoll.rs`,
`src/math/pd.rs`) drives each of its joints with one quaternion
PD controller whose output is clamped to `max_torque` (an angular
acceleration), scaled by a continuous `RagdollStrength` dial borrowed from
Lugaru's per-muscle `strength`. Measured against this chapter, that actuator
is a **pure activation-scaled, constant-ceiling force generator**. What it
does **not** model, each with the note that supplies the missing piece:

| Muscle property | Ragdoll analogue | Status in migera |
|---|---|---|
| Twitch / activation dynamics (9.0.5, 9.0.6, 9.3.1) | low-pass the strength command with a critically damped filter, T ≈ 50–100 ms (= critical spring halflife T·ln 2); slower off than on | not modeled — strength changes take effect next step; stun recovery is linear over 1.2 s |
| Force-length (9.1.1, 9.1.4) | angle-dependent ceiling multiplier per joint | not modeled — ceiling constant over the range |
| Parallel elastic element (9.1.2) | passive, strength-independent torque stiffening near joint limits | not modeled — hard avian limits; a limp joint has zero passive stiffness |
| Series elastic element (9.1.3, 9.3.1) | compliance between PD output and body | not modeled; low priority |
| Force-velocity, concentric (9.2.1) | Hill falloff of the ceiling with joint speed along the torque — natural speed limit and "weight" | not modeled — clamp is velocity-independent |
| Force-velocity, eccentric (9.2.2) | ceiling rising to ~1.5× when the joint is forced backwards | not modeled — clamp is symmetric |

All of these change only the **ceiling** or the **strength signal**, not the PD
gains, so none interacts with the `kd·dt < 2` stability bound. Lugaru's
muscles, by contrast, are length constraints relaxed toward a blend of target
and rest length — no activation lag, no force-velocity. The one physiological
analogy migera already has: a critically damped PD at 8 Hz responds to an
impulse with a twitch-shaped curve peaking at 1/ω ≈ 20 ms, at the fast-twitch
end of human muscle.

## Where to read in the book

- pp. 224–231 (PDF 237–244): motor units and twitch — Eq. 9.1 and Fig. 9.5
  on p. 230 (PDF 243), Fig. 9.6 on p. 231 (PDF 244).
- pp. 231–236 (PDF 244–249): force-length — Figs. 9.8/9.9 on p. 233 (PDF 246).
- pp. 236–243 (PDF 249–256): force-velocity — Fig. 9.12 and Eq. 9.2 on p. 237
  (PDF 250), Hill Eq. 9.3 on p. 238 (PDF 251), Fig. 9.15 on p. 242 (PDF 255).
- pp. 243–247 (PDF 256–260): modeling — Eq. 9.4 on p. 243, Eq. 9.5 and Fig.
  9.19 on p. 246 (PDF 259).
- pp. 247–249 (PDF 260–262): references.

## See also

- [Chapter 10 — Kinesiological electromyography](../ch10-kinesiological-electromyography/INDEX.md) — the EMG signal that drives §9.3.1's model; read 10.3 before using EMG data.
- [4.3 Muscle anthropometry](../ch04-anthropometry/4.3-muscle-anthropometry/INDEX.md) — cross-section, stress and moment arms: the scale factors that turn this chapter's normalized curves into joint torques.
- [Ragdoll and physics](../../character-animation/ragdoll-and-physics/INDEX.md) — migera's ragdoll traps; read before changing any ceiling.
- [Lugaru's joint/muscle animation system](../../character-animation/lugaru-joint-muscle-system.md) — origin of the strength dial; its "muscles" are constraints, not Hill-type actuators.
- [PD damping has an explicit-integration bound](../../character-animation/ragdoll-and-physics/pd-damping-explicit-integration-bound.md) — why muscle-like behaviour should go into the ceiling, not the PD frequency.
