---
title: Character animation
description: "migera's character animation domain: the rotation-space procedural stack in src/character/anim (springs, phase, gait, IK, active ragdoll, studio), its rig/retargeting, IK and ragdoll lessons, and Lugaru/botica prior art. Read before any work in src/character or when a pose, leg or ragdoll looks wrong."
type: index
status: current
tags:
  - character-animation
  - rig
  - ik
  - ragdoll
  - locomotion
updated: 2026-09-28
---

# Character animation

How migera animates characters, and what it cost to learn. The domain holds
three kinds of note: lessons from building `src/character/anim` (grouped by
topic below), a decision record for the deleted `muscle` module, and prior
art from other engines (Lugaru, botica). The chronological build log is
`CHARACTER_PROGRESS.md`, not this tree.

## What migera built

`src/character/anim` is a complete rotation-space procedural animation
plugin and the only animation stack. The position-space `src/character/muscle`,
built from the Lugaru design, was **deleted** in 9981e16. Characters render
through Bevy's standard skinned PBR pipeline on a Mixamo-compatible rig; the
real test character is `assets/models/puppet_base.gltf`.

| Layer | Modules | What it does |
|---|---|---|
| Springs | `dho.rs`, `math/spring.rs`, `math/inertialize.rs` | Per-bone quaternion damped harmonic oscillators; velocity-continuous transitions via inertialization |
| Phase | `phase.rs` | Analytic breath/idle oscillators composed onto the target pose before the spring |
| Poses and clips | `poses.rs`, `asset.rs`, `clip.rs`, `convert.rs`, `stance.rs` | Hot-reloaded `assets/anim/*.pose.ron`, sparse-keyframe clips, a bent-knee stance applied per rig facing |
| Locomotion | `gait.rs`, `locomotion.rs`, `transition.rs`, `facing.rs`, `slide.rs` | Walk and run cycles, root motion, turning, starting/stopping, offline foot-sliding removal |
| IK and grounding | `math/ik.rs`, `legik.rs`, `armik.rs`, `lookat.rs`, `footlock.rs`, `ground.rs`, `pelvis.rs` | Two-bone leg/arm IK (Holden's recipe), foot locking, ground adaptation, spine-distributed look-at |
| Rig | `rig.rs`, `retarget.rs`, `gltf_rig.rs` | `[T; 22]` bone sets, `RigGeometry` measured from the real rig, retargeting onto glTF binds |
| Ragdoll | `ragdoll.rs`, `ragdoll_plugin.rs`, `math/pd.rs` | 14-body active ragdoll on avian: unmotorized `SphericalJoint`s, one quaternion PD per joint, continuous per-joint strength, hits (`RagdollHit`), a root that follows a walking character |
| Studio | `studio/` (`--features anim_studio`) | egui pose/spring/clip/phase/IK-effector authoring |

**The central inversion**, and why `muscle` was replaced rather than
extended: `muscle` simulated joint *positions* and derived each bone's
rotation from them. `anim` authors and springs each bone's local *rotation*,
and position falls out of transform propagation. Reconstructed roll, a bone's
inferred rotation really being its parent's, multi-child parents disagreeing,
and direction lagging a position blend stop being expressible, and bone
length becomes invariant by construction. The one Lugaru idea carried across
is the continuous strength dial, now scaling PD torque ceilings.

Current open work is tracked in `CHARACTER_PROGRESS.md`, not here.

## Key facts

- `anim` costs ~1.8 µs per character per frame, linear to 1000 characters; measure with `anim_bench`, never `character_gallery` frame time ([muscle deleted](./animation-core/muscle-deleted-anim-is-the-only-stack.md)).
- Tests on the synthetic T-pose rig cannot see retargeting bugs; verify on `puppet_base.gltf` or over BRP ([synthetic-rig tests are blind](./rig-and-retargeting/synthetic-rig-tests-are-blind-to-retargeting.md)).
- A pose delta names a world axis: conjugate by the accumulated bind, and use `world_correction_frame` for world corrections ([a pose delta names a world axis](./rig-and-retargeting/a-pose-delta-names-a-world-axis.md)).
- Never hardcode a facing: `puppet_base` faces `+Z`, the synthetic rig `-Z`; assert directions with signed measures such as `knee_fold_direction` ([knee axis](./rig-and-retargeting/knee-axis-positive-swings-forward.md)).
- The synthetic rig's `LeftLeg` is a 0.07 m stub, so leg shape must be judged on the real rig ([leg segments shifted](./rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md)).
- A bind pose has zero leg slack; the stance supplies knee bend, and reach margins are deadbands ([zero slack](./ik-and-locomotion/bind-pose-zero-leg-slack-is-normal.md), [critical extension](./ik-and-locomotion/rig-authored-at-critical-extension.md)).
- Foot IK samples ground from the animated pose, keeps its correction out of spring state, and solves on the real rig ([foot IK loops](./ik-and-locomotion/foot-ik-feedback-loops.md)).
- PD gains are accelerations (`apply_angular_acceleration`) and need `kd·dt < 2` ([torque vs acceleration](./ragdoll-and-physics/avian-apply-angular-acceleration-not-torque.md), [damping bound](./ragdoll-and-physics/pd-damping-explicit-integration-bound.md)).
- At ragdoll strength 1 the screen shows the animation; verify body-vs-target error and spin over BRP ([read-back](./ragdoll-and-physics/full-strength-readback-hides-the-physics.md)).
- Pose screenshots: `--gizmos on --show-real-mesh off`, Front and Left ([gizmos](./animation-core/gizmos-need-show-real-mesh-off.md)).

## Topics and notes

| Note | What it establishes | Read when |
|---|---|---|
| [Animation core](./animation-core/INDEX.md) | Stack status, studio, spring numerics, the verification view | before measuring cost, touching springs/studio, or taking a screenshot |
| [Rig and retargeting](./rig-and-retargeting/INDEX.md) | Bind-frame conjugation, world-axis deltas, synthetic-vs-real rig traps, facing | before touching `rig.rs`/`retarget.rs`/`stance.rs` or composing rotations |
| [IK and locomotion](./ik-and-locomotion/INDEX.md) | Reach budgets, IK pivots, foot-IK feedback loops, stance | before changing `legik`/`armik`/`pelvis`/foot IK/gait |
| [Ragdoll and physics](./ragdoll-and-physics/INDEX.md) | avian PD units and stability, body/anchor frames, joint limits, verifying physics | before changing ragdoll code or tuning gains/limits |
| [Lugaru's joint/muscle animation system](./lugaru-joint-muscle-system.md) | Source-verified Lugaru design (particles, muscles, strength dial) plus the history of migera's deleted port | before designing ragdoll blending or physics-driven animation |
| [botica's character animation system](./botica-character-animation-system.md) | botica's Overgrowth-style stack and the Mixamo skeleton standard migera follows | before changing bone names, axes or rest rotations, or looking for prior art |

## See also

- [migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — why characters use Bevy's pipeline and not the SDF renderer.
- [Engineering practice](../engineering-practice/INDEX.md) — the testing, debugging and measurement lessons most of these bugs taught.
- [Winter — Biomechanics and Motor Control of Human Movement](../biomechanics-winter/INDEX.md) — real-human reference: read its "Applying the book to migera" table before gait, balance, transition or ragdoll mass/torque/actuation work.
