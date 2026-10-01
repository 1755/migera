---
title: Lengthen a segment by moving its joint and scaling only its skinning
description: "To change a skinned character's segment length in Bevy, move the child joint and skin the segment through a helper joint scaled along the bone; moving the joint alone stretches the knee's triangles up to 2.7x, and a scaled parent shears its children. Read before building body proportions or rescaling any bone."
type: decision
status: current
tags:
  - rig
  - anthropometry
  - bevy
  - assets
updated: 2026-10-01
verified: 2026-10-01
code:
  - examples/character_gallery.rs
  - tools/skin_segment_stretch.py
  - src/character/anim/plugin.rs
sources:
  - "tools/skin_segment_stretch.py on assets/models/character.glb (thigh, shin, upper arm, factor 1.1)"
  - "character_gallery --proportion-spike move|proxy 1.1, character.glb, relaxed_stand and getup:half_kneel, Front and Left"
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed., §4.0.1 (segment lengths as fractions of height)"
aliases:
  - body proportions
  - segment scale compensate
  - proportion spike
  - --proportion-spike
  - skinning proxy joint
---

# Lengthen a segment by moving its joint and scaling only its skinning

A body-proportion change on a skinned rig needs two edits. Move the child
joint along the bone (the skeleton gets longer), and skin the segment
through a helper joint, parented to it, that scales along the bone's own
axis (the mesh gets longer). The hierarchy itself never carries a scale.
Measured on `character.glb`, this keeps every joint's blend region
unstretched.

## Context

Plan step 3b (`WINTER_MOTION_PLAN.md`) asked whether a skinned mesh
survives a segment-length change at all, before building Winter-style
height-fraction proportions (§4.0.1). The spike lengthened `character.glb`'s
left thigh by 10% and kept the right as the reference.

## Decision

- **Move the joint**: multiply the child's local translation by the factor.
  The anim stack needs nothing else. The foot IK re-reads every bone's live
  translation each frame (`plugin::live_rig_geometry`), so the lengthened
  leg stood with its foot planted and its knee bent a little more.
- **Scale only the skinning**: spawn a helper entity as a child of the
  segment's joint, `Transform::from_scale((1, f, 1))`, and replace that
  joint in every `SkinnedMesh::joints` list with the helper. The inverse
  bind matrices stay as they are. Both `character.glb` (Mixamo) and
  `puppet_base` bind each bone along its local +Y (the child translation is
  `(0, L, 0)`), so the scale is along the segment. For another rig, scale
  along the child translation's direction instead.
- `character_gallery --proportion-spike proxy 1.1` does both, on the left
  thigh. `--proportion-spike move 1.1` moves the joint only.

## Alternatives considered

| method | blend-region edges, straight | blend, bent 90° | worst edge |
|---|---|---|---|
| move the joint only | p50 1.131, max 1.44 | max 2.71 | 2.71 |
| move + skinning-only scale | p50 1.000, max 1.09 | max 1.16 | 1.40 (hip blend) |
| scale the bone in the hierarchy | not built: shears the children | | |

Ratios are skinned edge length over the unmodified rig's (thigh, factor
1.1, `tools/skin_segment_stretch.py`). Blend edges have an end weighted
over 0.1 on both the thigh and the shin. The shin and the upper arm agree:
the move-only method reaches 1.99 and 3.12; the skinning scale reaches 1.13
and 1.15.

- **Moving the joint alone** leaves the thigh's vertices where they were.
  The vertices weighted to both joints spread across the gap: the knee
  bandage visibly turned into a tall band, and at a 90° bend single knee
  triangles stretched 2.7×.
- **Scaling the bone in the hierarchy** (`Transform.scale` on the thigh,
  inverse scale on the shin) cannot work in Bevy. A child's world matrix
  is `parent · child`, so the parent's non-uniform scale applies after the
  child's rotation: a bent shin is sheared, and its rendered length
  changes with the knee angle. The inverse scale on the child only cancels
  it at the bind angle. Maya calls the missing feature segment scale
  compensate. Bevy has no equivalent.
- The proxy's worst edges (1.40) are 40 short edges in the upper thigh and
  hip blend. The thigh scales away from the hip joint, but the vertices
  partly weighted to the hips don't. That stretch is mild and gradual, and
  not visible on screen.

## Consequences

- Height-fraction proportions are feasible: per segment, scale the child
  translation and the segment's skinning by the same factor.
- Girth is unchanged, so a 10% longer thigh is also 10% more slender in
  proportion. Scaling the other two axes too would change it, but that
  isn't a proportion question.
- Anything that caches rig geometry at spawn (the ragdoll's capsules and
  joint anchors) has to be built after the change.

## Revisit when

- Proportions are built for real: scale every segment, not just one.
  Check the ragdoll's spawn ordering and the gallery's grounding then.
- A rig binds its bones along an axis other than +Y.

## Related

- [The live rig geometry must match the rendered rig](./live-rig-geometry-must-match-the-rendered-rig.md) — applies: why the moved joint needs nothing else; the IK reads live translations.
- [4.0.1 Segment dimensions](../../biomechanics-winter/ch04-anthropometry/4.0-scope-and-segment-dimensions/4.0.1-segment-dimensions.md) — prerequisite: the segment-length fractions a proportion system would scale to.
