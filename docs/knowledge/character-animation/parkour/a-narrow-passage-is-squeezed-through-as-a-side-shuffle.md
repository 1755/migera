---
title: A narrow passage is squeezed through as a side shuffle
description: "Step 10, fifth part: asked to squeeze along a passage, the walker turns square to it at its mouth, side-shuffles along it at 0.35 m/s, arms flat, and walks on once through. The side shuffle itself costs 190 µs a frame, eight times a walk. Read before changing parkour/squeeze.rs."
type: decision
status: current
tags:
  - locomotion
  - performance
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/squeeze.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::squeeze::tests::{a_passage_is_faced_across_and_shuffled_along, squeezed_the_arms_are_held_flat}"
  - "live: character_gallery --step-seconds 0.0333333 --squeeze 1,-2,4,-2,0.5, Xvfb, gizmos on/mesh off Top and Left, mesh on; BRP capture of the pelvis and hands"
  - "anim_bench --gait squeeze / shuffle / walk --characters 100"
  - "Warren and Whang 1987 (parkour-movement-data, Crawling and squeezing)"
aliases:
  - squeeze
  - Squeeze
  - Walker::squeeze
  - side shuffle cost
---

# A narrow passage is squeezed through as a side shuffle

Step 10 of the [parkour design](./parkour-moves-implementation-design.md),
fifth part: through a gap too narrow to walk, sideways.

## Decision

**A passage** (`parkour::squeeze::Squeeze`) is a line from its mouth `from`
to its far end `to`. A gap narrower than 1.3 shoulder widths is tight
(`Squeeze::is_tight`, Warren and Whang's critical ratio). Finding gaps is
out of scope, so the walker squeezes when asked.

**The walker** (`Walker::squeeze`):
- **approach:** it walks to the mouth, facing square across the passage.
  The side nearer its facing is chosen once, as asked, and kept;
- **in the passage:** it holds that facing and side-shuffles along it on
  the existing shuffle gait at 0.35 m/s, toward the far end, the head
  turned along its way. The walls keep it off themselves by the walker's
  own wall probe (0.2 m);
- **arms** (`squeeze::flatten`): held flat at its sides, 0.12 rad in and
  0.15 rad back, eased in and out over 0.4 s;
- **through:** within 8 cm of the far end, the ask is dropped and it walks
  on.

No data for a sideways shuffle while squeezing: the pace is a careful
side-step's.

## Alternatives considered

- **The facing re-chosen each frame**: walking toward the mouth along the
  passage, the nearer side could flip mid-approach.
- **A shuffle gait of its own**: the walker's side shuffle (`shuffle`)
  already moves sideways with its feet planted; only the arms change.

## Consequences

**Headless**:
- facing across and shuffling toward the far end, from facings on either
  side and askew;
- through within 8 cm;
- held flat, each hand in, behind where it hangs, and no farther out than
  its shoulder.

**Live**: walked to a 0.5 m passage's mouth, stopped square to it (the
pelvis 13 cm off the wall at its back), and shuffled along it at about
0.35 m/s, holding the line to within a centimetre. Seen from above, the
shoulders lie along the passage.

**Cost** (`anim_bench --characters 100`, a character at p50):

| Gait | Cost |
|---|---|
| squeezing | 191 µs |
| the side shuffle alone | 190 µs |
| a walk | 24 µs |

The side shuffle itself costs eight times a walk: it is posed afresh each
frame, its cycle not cached. That predates this step and is worth a look
of its own.

## Revisit when

- **The side shuffle's cost**: cache its cycle as the walk caches its own.
- **A passage that turns**: only straight.
- **Chest to the wall instead of back**: either side is faced as it
  comes.

## Related

- [Walking is kept out of walls](./walking-is-kept-out-of-walls.md) — context: the wall probe that keeps it off both walls.
- [A crawl is a four-beat gait on the get-up's hands and knees](./a-crawl-is-a-four-beat-gait-on-the-get-ups-hands-and-knees.md) — contrast: under a low gap rather than through a narrow one.
