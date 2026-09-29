---
title: Chapter 7 — Three-dimensional kinematics and kinetics
description: "Winter's 3D chapter: lab vs segment frames, the x–y′–z″ Cardan matrix (transpose of glam EulerRot::XYZ), gimbal lock, sequence dependence, Euler rates vs ω, 3D Newton–Euler inverse dynamics with gyroscopic terms, joint power, gait curves. Read before decomposing bone rotations or reasoning about ragdoll dynamics."
type: index
status: current
tags:
  - biomechanics
  - math
  - inverse-dynamics
  - rig
  - ragdoll
  - retargeting
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), ch. 7, pp. 176–199 (PDF pp. 189–212)"
---

# Chapter 7 — Three-dimensional kinematics and kinetics

> **Source:** Winter (2009) ch. 7, pp. 176–199 ·
> [open PDF at p. 176](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=189) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Marker coordinates arrive in a lab frame (X
forward, Y up, Z lateral). Each segment gets an anatomical frame (origin at
COM, y along the long axis) built from markers with cross products; a
constant calibration matrix links marker and anatomical frames. The
orientation is expressed as three Cardan angles in the x–y′–z″ sequence
(frontal, axial, sagittal), from which body-frame angular velocity follows by
a sequence-specific matrix that is singular when the middle angle hits ±90°.
Kinetics is Chapter 5 in 3D: Newton for joint forces in the global frame,
Euler's principal-axis equations — with their (I_a − I_b)ω_aω_b coupling —
for joint moments in the segment frame, and power as moment · joint angular
velocity. Two fully worked numeric examples (a swing-phase leg frame, a
stance-phase knee) and averaged 3D gait curves round it off. I re-ran both
examples in scipy/numpy; they hold, apart from a handful of printed typos
noted in the notes.

**Start here:** [7.1.2](./7.1-axes-systems/7.1.2-local-reference-systems-and-rotation.md)
(the matrix and its glam/scipy/rig mapping) → [7.3](./7.3-segment-angular-velocity-and-acceleration.md)
(θ̇ is not ω) → [7.4.2](./7.4-kinetic-analysis-reactions-and-moments/7.4.2-euler-3d-equations-of-motion.md)
(Euler's equations). The worked examples are fixtures; open them when writing a test.

## Key facts

- Book [G to A] (Eq. 7.5) = transpose of scipy `from_euler('XYZ')` = transpose of glam `Quat::from_euler(EulerRot::XYZ, …)` (intrinsic) — [7.1.2](./7.1-axes-systems/7.1.2-local-reference-systems-and-rotation.md).
- Book lab frame → Bevy rig (+Z facing) is a 90° yaw; the sequence becomes Z–Y′–X″ with the flexion angle negated — [7.1.2](./7.1-axes-systems/7.1.2-local-reference-systems-and-rotation.md).
- Middle-angle singularity: a segment-vs-world decomposition breaks when the character turns 90°; decompose joint (relative) rotations instead — [7.1.2](./7.1-axes-systems/7.1.2-local-reference-systems-and-rotation.md).
- The same orientation reads as very different angles in different sequences; the same angles in two orders differ by ~26° at 30°/30°/30° — [7.1.3](./7.1-axes-systems/7.1.3-other-rotation-sequences.md).
- Frames from points: long axis from two joints, plane from a third landmark, two cross products, re-orthogonalise — [7.2.1](./7.2-marker-and-anatomical-axes/7.2.1-kinematic-data-set-example.md).
- ω_body = E(θ₂, θ₃)·θ̇, det E = cos θ₂ — never differentiate Euler angles as if they were ω — [7.3](./7.3-segment-angular-velocity-and-acceleration.md).
- Euler's equations: gyroscopic coupling ≈ 0.01 % of knee moment in walking; stance moments are quasi-static — [7.4.2](./7.4-kinetic-analysis-reactions-and-moments/7.4.2-euler-3d-equations-of-motion.md).
- Worked knee moments (42.35, 20.78, −26.11) N·m, reproducible from Tables 7.3–7.4 — [7.4.3](./7.4-kinetic-analysis-reactions-and-moments/7.4.3-kinetic-data-set-example.md).
- Ankle push-off (~50 % stride) dominates walking power; hip abductors carry the pelvis frontally — [7.4.5](./7.4-kinetic-analysis-reactions-and-moments/7.4.5-sample-moment-and-power-curves.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [7.0 Introduction](./7.0-introduction.md) | Chapter goal: marker coordinates → anatomical frames → 3D kinetics | Deciding whether Chapter 7 is needed |
| [7.1 Axes systems](./7.1-axes-systems/INDEX.md) | Lab and segment frames, Cardan matrix, 12 sequences, dot/cross products | Before decomposing/composing rotations as angles or converting book axes to the rig |
| [7.2 Marker and anatomical axes systems](./7.2-marker-and-anatomical-axes/INDEX.md) | Frames from markers, [G to M]·[M to A], worked leg example | When building a bone frame from joint positions or needing a frame fixture |
| [7.3 Determination of segment angular velocities and accelerations](./7.3-segment-angular-velocity-and-acceleration.md) | Eq. 7.7b, Euler rates vs body ω, singularity | Before computing ω/α from angles or feeding ω to a controller |
| [7.4 Kinetic analysis of reaction forces and moments](./7.4-kinetic-analysis-reactions-and-moments/INDEX.md) | 3D Newton–Euler inverse dynamics, worked knee, joint power, gait curves | Ragdoll torque/gyroscopic reasoning, gait timing, torque budgets |
| [7.5 Suggested further reading](./7.5-suggested-further-reading.md) | Greenwood, Zatsiorsky, D'Sousa & Garg | When single-segment treatment is not enough |
| [7.6 References](./7.6-references.md) | Davis 1991, Eng & Winter 1995, Õunpuu 1996 | When needing primary 3D gait kinetics data |

## Relevance to migera

- **Rotation representation.** migera composes one quaternion per bone on a
  Mixamo-style rig whose bones run along local +Y — the same "y = long axis"
  convention as Winter's anatomical frames, so the book's θ₂ corresponds to
  twist about the bone. Quaternions remove gimbal lock from *storage*; the
  chapter's pitfalls return whenever a rotation is decomposed for limits,
  sliders or comparisons with gait data.
- **Frames are the recurring bug.** The chapter's machinery is all "which
  frame is this in": passive vs active matrices (Eq. 7.5 is the transpose of
  an engine orientation), global vs moving axes (intrinsic sequence), body vs
  world ω. migera's rig lessons are the same bug class:
  [a pose delta names a world axis](../../character-animation/rig-and-retargeting/a-pose-delta-names-a-world-axis.md)
  (a `LocalPose` delta names a *world* T-pose axis, unlike Winter's moving
  axes) and
  [conjugate pose deltas by the bind rotation](../../character-animation/rig-and-retargeting/conjugate-pose-deltas-by-the-bind-rotation.md).
  Commuting (parallel-axis) test cases hide order errors in both.
- **Facing and sign.** Book X-forward/Z-right vs rig +Z-forward/−X-right:
  the sagittal angle's sign flips under the map
  ([KNEE_AXIS positive swings forward](../../character-animation/rig-and-retargeting/knee-axis-positive-swings-forward.md)).
- **Ragdoll dynamics.** Euler's equations explain gyroscopic precession of a
  spinning ragdoll limb and show it is negligible at walking speeds, which is
  why prescribing angular acceleration in the PD
  ([avian: apply_angular_acceleration](../../character-animation/ragdoll-and-physics/avian-apply-angular-acceleration-not-torque.md))
  is safe for walking. Gait moment/power curves give per-kg torque budgets.
- **Oracles.** Check any decomposition, angular-velocity or dynamics helper
  against `scipy.spatial.transform.Rotation` and the book's worked tables,
  never against a re-derivation in the same code.

## Where to read in the book

- pp. 176–180 (PDF 189–193): frames, Fig. 7.1, Eqs 7.1–7.6 — render pp. 178–179, text extraction loses primes and matrix layout.
- pp. 180–187 (PDF 193–200): marker/anatomical frames, Fig. 7.2, Tables 7.1–7.2, worked angles.
- pp. 187–188 (PDF 200–201): Eq. 7.7b.
- pp. 188–194 (PDF 201–207): Eqs 7.8–7.9, Fig. 7.3, Tables 7.3–7.4, worked knee.
- pp. 194–198 (PDF 207–211): Eq. 7.10, Figs 7.4–7.5 (printed sideways).

## See also

- [Chapter 3 — Kinematics](../ch03-kinematics/INDEX.md) — the planar angle, velocity and finite-difference definitions this chapter generalises.
- [Chapter 5 — Kinetics: forces and moments of force](../ch05-kinetics-forces-and-moments/INDEX.md) — the planar link-segment inverse dynamics this chapter lifts to 3D.
- [Chapter 8 — Synthesis of human movement: forward solutions](../ch08-synthesis-forward-solutions/INDEX.md) — the forward-dynamics counterpart, closer to what a ragdoll does.
