---
title: Quality and Artifacts
description: Troubleshooting and prevention for 3DGS quality: floaters, popping, needles, SH colour outliers and zoom aliasing mapped to root causes (often capture/initialization, not training), Mip-Splatting's anti-aliasing fix, and capture practice. Read when a splat scene looks wrong or before a new capture.
type: index
status: current
tags:
  - 3dgs
  - troubleshooting
  - correctness
updated: 2026-09-28
---

# Quality and Artifacts

3DGS's characteristic failure modes each have specific, well-understood root causes.
Several of the most common trace back to capture and initialization rather than
training, so the highest-leverage fix is often prevention (better capture) rather than
post-hoc correction. This topic is both a troubleshooting reference and a prevention
guide.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Common quality artifacts and their causes](./common-artifacts.md) | Symptom → cause → mitigation for floaters, popping, needles and SH colour outliers | First stop for any visible splat quality problem |
| [Mip-Splatting: fixing aliasing artifacts](./mip-splatting-and-anti-aliasing.md) | Zoom aliasing comes from no minimum Gaussian size plus ad hoc 2D dilation; fixed by a 3D smoothing filter plus a 2D Mip filter | Splats shimmer, alias or over-sharpen when zooming |
| [Capture best practices](./capture-best-practices.md) | View coverage, overlap, SfM failure modes, lighting consistency, and verifying SfM output before training | Before shooting any new capture |
