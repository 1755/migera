---
title: Chapter 8 — Synthesis of human movement, forward solutions
description: Forward dynamics of a link-segment body (moments in, motion out) - six modelling requirements, why open-loop simulations drift and collapse within ~500 ms, spring/damper joint and foot models that worked, and a Lagrangian recipe with a verified 3-link example. Read before designing or debugging the ragdoll.
type: index
status: current
tags:
  - biomechanics
  - physics
  - ragdoll
  - math
  - numerics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), ch. 8, pp. 200–223 (PDF pp. 213–236)"
aliases:
  - forward dynamics
  - forward simulation
  - movement synthesis
---

# Chapter 8 — Synthesis of human movement, forward solutions

> **Source:** Winter (2009) ch. 8, pp. 200–223 ·
> [open PDF at p. 200](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=213) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Inverse dynamics (chs. 5 and 7) goes from
measured motion to joint moments. A *forward solution* runs the other way:
joint moments and initial conditions go in, and the motion comes out by
integrating the equations of motion. This is exactly what an active ragdoll
does. Winter's verdict is sober. The whole body must be modelled, because
one joint's moment changes every segment's acceleration through the reaction
forces. Every mass, inertia and joint limit has to be nearly perfect. Even
then, feeding measured moments back into a good 3D 9-segment walking model
(Gilchrist and Winter 1997) drifts until it falls or collapses after about
**500 ms**, because the double integration turns small moment errors into
growing position errors. The only fix is continuous correction of the
input moments, which breaks the "pure simulation" rules. The rest of the
chapter (§8.2–8.6) is a list-based Lagrangian recipe for deriving the
equations of motion, ending with a 3-link standing-balance model whose
four coupled equations show why a single mass error spreads into every
coordinate.

**Start here:** read [8.0.1](./8.0-introduction/8.0.1-forward-model-assumptions-and-constraints.md)
and [8.1](./8.1-review-of-forward-solution-models.md) for the physics
lessons. Read §8.2–8.6 only if you need to derive equations of motion by
hand, for example to build a test oracle.

**Transcription warning.** The printed equations in §8.2–8.6 have many
typos: the Euler-angle matrices 8.19–8.22, the sliding-block example
8.12f–g, and the illustrative example's velocities, work and equations
(a)–(c). The notes here give corrected forms. Each correction was checked
numerically: the 3D matrices against `scipy.spatial.transform.Rotation`,
and the example equations by finite-differencing the Lagrangian.

## Key facts
- A forward model needs the whole body, all important DOF, passive joint-range forces, initial positions and velocities, and no kinematic constraints. See [8.0.1](./8.0-introduction/8.0.1-forward-model-assumptions-and-constraints.md).
- Internal validity comes first: inverse-dynamics moments fed forward must reproduce the measured motion. See [8.0.2](./8.0-introduction/8.0.2-forward-simulation-potential.md).
- An incomplete model walks only if its moment patterns are also wrong ("two wrongs can make a right"). See [8.1](./8.1-review-of-forward-solution-models.md).
- Open-loop playback of measured moments drifts to a fall within about 500 ms. Only continuous fine-tuning of the moments holds it. See [8.1](./8.1-review-of-forward-solution-models.md).
- A stiff foot spring gives huge heel-strike acceleration spikes. An array of parallel springs and dampers under a rigid foot fixed this. See [8.1](./8.1-review-of-forward-solution-models.md).
- Joint range limits were modelled as springs: nonlinear springs at the knee, ankle and MTP, linear springs at the hip, plus dampers at every joint. See [8.1](./8.1-review-of-forward-solution-models.md).
- The DOF count is $6S + 3P - C$. The count must be at least zero, and a system with DOF = 0 is solved kinematically. See [8.2.2](./8.2-mathematical-formulation/8.2.2-generalized-coordinates-and-dof.md).
- Viscous dampers enter the equations through Rayleigh's dissipation function, $Q_i = -\partial DE/\partial\dot q_i$. See [8.3.2](./8.3-system-energy/8.3.2-spring-potential-and-dissipative-energy.md).
- Joint reactions do no work. Either merge the joint points (fewer, longer equations) or split them and add constraint equations, whose multipliers are the reaction forces. See [8.5](./8.5-designation-joints.md).
- In the 3-link balance model, the HAT mass $m_3$ appears in all four equations of motion. See [8.6](./8.6-illustrative-example.md).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [8.0 Introduction](./8.0-introduction/INDEX.md) | Forward vs. inverse solutions: why the whole body must be modelled, the six requirements, the internal-validity test | before trusting any ragdoll or physics-driven character to "just work" from joint torques |
| [8.1 Review of forward solution models](./8.1-review-of-forward-solution-models.md) | Who modelled gait how (1968–1997), what failed and why; the viscoelastic foot, spring joint limits, the 500 ms drift | when choosing joint-limit, damping or foot-contact models for the ragdoll |
| [8.2 Mathematical formulation](./8.2-mathematical-formulation/INDEX.md) | The Lagrangian recipe: generalized coordinates, DOF, $L$, generalized forces, point/frame lists, 2D and 3D kinematics | when deriving equations of motion by hand or checking an Euler-angle formula |
| [8.3 System energy](./8.3-system-energy/INDEX.md) | Segment KE/PE with an off-COM origin, inertia tensor, linear and torsional springs, Rayleigh damping | when writing an energy-conservation test or a spring-damper joint |
| [8.4 External forces and torques](./8.4-external-forces-torques.md) | Force and torque lists; actuators as equal and opposite pairs; $Q_i = \partial W/\partial q_i$ | when mapping joint torques onto generalized coordinates |
| [8.5 Designation of joints](./8.5-designation-joints.md) | Shared point (no reactions) vs. split points + loop-closure constraint (reactions as multipliers) | when comparing reduced-coordinate vs. maximal-coordinate (avian-style) formulations |
| [8.6 Illustrative example](./8.6-illustrative-example.md) | 3-link standing-balance model (leg, thigh, HAT): full corrected equations of motion and the coupling lesson | when building an analytic oracle for a jointed chain, or wondering why one mass error ruins everything |
| [8.7 Conclusions](./8.7-conclusions.md) | Once the lists are right, deriving the equations is mechanical | rarely; a recap of the method's claim |
| [8.8 References](./8.8-references.md) | The cited simulation literature, with the handful worth reading for animation physics | when looking for original forward-dynamics gait or foot-contact models |

## Relevance to migera

migera's active ragdoll (`src/character/anim/ragdoll.rs`,
`ragdoll_plugin.rs`) is a forward solution: PD torques go in, avian
integrates the bodies. It already violates Winter's "pure simulation"
requirements in the ways Winter says any working model must. The root is a
pinned kinematic body, `GravityScale = 1 − strength` compensates gravity,
and the torques are a feedback controller rather than a replayed moment
history. Those torques are also per-body world-space accelerations with no
reaction on the parent: *external* torques in the book's terms, not
internal joint moments ([8.4](./8.4-external-forces-torques.md)). Chapter 8 explains *why* these violations are needed and not a
hack. Open-loop forward dynamics is unstable, because errors grow with the
double integral. A feedback term is the "continuous fine-tuning" that
Gilchrist and Winter had to add. So when the ragdoll drifts, judge it
against a tracking target, never against open-loop fidelity. The book's
joint models, with springs for range limits and dampers at every joint,
contrast with migera's hard avian joint limits
([avian joint limits are not cone and twist](../../character-animation/ragdoll-and-physics/avian-joint-limits-are-not-cone-and-twist.md)).
The book's dampers are continuous-time. migera's discrete controller has an
extra stability bound that the book does not discuss
([PD damping has an explicit-integration bound](../../character-animation/ragdoll-and-physics/pd-damping-explicit-integration-bound.md)).
The corrected 3-link equations of §8.6 can serve as an independent analytic
oracle for a pinned three-body avian chain.

## Where to read in the book
- pp. 200–202 (PDF 213–215): forward vs. inverse, requirements, internal validity.
- pp. 202–203 (PDF 215–216): the model review and the Gilchrist–Winter 500 ms result. This is the most useful page for physics animation.
- pp. 205–214 (PDF 218–227): Lagrange's method; Figs. 8.1–8.4, Eqs. 8.1–8.22.
- pp. 214–217 (PDF 227–230): energy, springs, dampers, forces, joints (Eqs. 8.23–8.34).
- pp. 217–221 (PDF 230–234): Fig. 8.5 and the 3-link example.

## See also
- [Ragdoll and physics](../../character-animation/ragdoll-and-physics/INDEX.md) — migera's forward-dynamics character; the practical side of this chapter.
- [Chapter 5 — Kinetics](../ch05-kinetics-forces-and-moments/INDEX.md) — the inverse solution that this chapter reverses; its link-segment assumptions are requirement 1 of §8.0.1.
- [Chapter 7 — 3D kinematics and kinetics](../ch07-three-dimensional-kinematics-and-kinetics/INDEX.md) — Newton–Euler 3D equations; the alternative to the Lagrangian route.
- [Chapter 9 — Muscle mechanics](../ch09-muscle-mechanics/INDEX.md) — muscle models, the next step past pure joint torques as model inputs.
