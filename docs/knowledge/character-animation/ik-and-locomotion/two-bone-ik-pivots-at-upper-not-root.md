---
title: Two-bone IK pivots at the upper joint, not the root
description: "A two-bone IK chain root→upper→lower→tip pivots at upper; measuring the target distance from the fixed root socket lands short by the socket offset (0.14 m at the shoulder), and the test made the same mistake. Read before writing an IK solver or IK reach test, or when a solve lands short by a constant."
type: lesson
status: current
tags:
  - ik
  - correctness
  - testing
updated: 2026-09-25
code:
  - src/character/anim/studio/effector.rs
  - src/math/ik.rs
  - src/character/anim/armik.rs
  - src/character/anim/legik.rs
aliases:
  - IK reach shortfall
  - effector lands short
---

# Two-bone IK pivots at the upper joint, not the root

In `character::anim::studio::effector` a chain is `root → upper → lower →
tip`, for example `LeftShoulder → LeftArm → LeftForeArm → LeftHand`. The
`root` is a fixed socket the solve never rotates. The two bones are
`upper → lower` and `lower → tip`, so the law-of-cosines distance must be
measured from **`upper`**.

## What happened

Measuring from `root` adds the socket-to-limb offset to every target
distance, **0.14 m** at this rig's shoulder, and the solve landed exactly
that far short. It read as "the solver is imprecise", not as a wrong pivot.

It took long to find because *the test made the same mistake*. It computed
its target from `root` too, so it asked for points genuinely outside the
chain's reach. Two errors partly masked each other. Plausible wrong fixes
were tried first: an iterative refinement loop, and an explicit child lookup
for multi-child bones. Then measurement showed the achieved distance was
already exact to 4 decimals. Only the *reach* was wrong.

## Why it matters

Exact distance plus exact direction plus a wrong position means the
reference point is wrong, not the maths. And when a test computes the same
quantity the same way as the code, it cannot catch an error in that
computation.

## How to apply

- When an IK solve lands consistently short by a fixed amount, measure the
  achieved distance and direction separately before touching the algorithm.
- Compute a test's expected value independently of the code under test.

## Related

- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — deeper: why the test could not catch this.
- [Ragdoll body and anchor frames](../ragdoll-and-physics/ragdoll-body-and-anchor-frames.md) — same-trap: correct computation against the wrong reference point.
- [Rig authored at critical extension](./rig-authored-at-critical-extension.md) — same-trap: which offset belongs to the chain, from the leg side.
- [Anim studio is complete](../animation-core/anim-studio-is-complete.md) — applies: the effector tool where this was found.
- [Walk-cycle IK and ground-lock bugs](./walk-cycle-ik-and-ground-lock-bugs.md) — same-trap: an earlier IK reach computed over the wrong segments.
