---
title: A crawl is a four-beat gait on the get-up's hands and knees
description: "Step 10, fourth part: asked to crawl, the walker gets down through the face-down get-up's keys run backward and crawls four-beat at 0.4 m/s. Each limb is down 3/4 of the cycle and solved to its place on the floor, the body 5 cm lower than the key. It gets up through the same keys. Read before changing parkour/crawl.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/crawl.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::crawl::tests::it_gets_down_crawls_and_gets_up"
  - "live: character_gallery --step-seconds 0.0333333 --crawl-at 1,6, Xvfb, gizmos on/mesh off Left and Front, mesh on Left and Front"
  - "anim_bench --gait crawl --characters 100"
  - "Ma et al. 2017 (parkour-movement-data, Crawling and squeezing)"
aliases:
  - crawl
  - Crawling
  - Walker::crawl
  - hands and knees
---

# A crawl is a four-beat gait on the get-up's hands and knees

Step 10 of the [parkour design](./parkour-moves-implementation-design.md),
fourth part: crawling on hands and knees, as under a low gap.

## Decision

**Getting down and up** reuse the face-down get-up (`getup::keys`): down
into its half-kneel (1.3 s, the sitting floor route's) then onto its hands
and knees (0.9 s); up from hands and knees into the half-kneel (0.8 s) and
to standing (0.8 s). They are played as a posture plays keys (`walker`):
each blended in the world, the feet down in both keys held, cleared of the
floor.

**The crawl** (`Crawling`), on the get-up's hands-and-knees pose:
- **pace:** 0.4 m/s (0.28-0.69 measured), a 0.4 m stride;
- **sequence:** four-beat, right hand, left knee, left hand, right knee,
  each limb down for 3/4 of the cycle, so three are always down;
- **limbs down** go back at the body's pace and stay put on the floor;
  **limbs up** swing forward on a smoothstep, lifted 6 cm (hand) or 4 cm
  (knee);
- **solving:** the arms by IK to their hands (the shoulder lifted, the
  elbow back and a little out); the legs by their shins (the knee and ankle
  moved together, the ankle placed);
- **body height:** 5 cm under the key's, so the limbs stay bent through
  the stance;
- **ramps:** the pace and the stride ease in and out over 0.6 s, the solved
  limbs blended in with the stride, so the crawl starts and ends exactly
  on the key;
- **the root** goes along the facing; nothing else is asked of it while
  crawling.

**The walker** (`Walker::crawl`): got down once standing still, crawls while
asked, gets up once not.

## Alternatives considered

- **Diagonal pairs (a trot)**: two limbs down at times, unstable for the
  slow pace measured as four-beat or diagonal (Babič et al. 2001: below
  0.6 m/s no two-point support is needed).
- **A crawl key of its own**: the get-up's already reaches the floor with
  both hands and both knees, and its keys get there.

## Traps

- **The key's limbs straight down**: a hand or knee 19 cm ahead or behind
  was out of reach, the limb straightened, and the elbow's swivel jumped
  5.6 cm in a frame. Lowering the body 5 cm keeps them bent.
- **Solving the limbs the moment the crawl took over**: the elbows
  swivelled their own way and the forearms jumped 7 cm. Blended in with
  the stride, the crawl starts on the key.
- **The key as built** sat a little under the floor; the key player lifted
  it and the crawl did not, so the hips dropped 1.6 cm at the hand-off.
- **Elbows out to the side**: the arms spread as for a push-up.

## Consequences

**Headless** (`puppet_base`, two facings): got down, crawled 4 s (1.59 m,
ramps included) and got up, with:
- no joint over 2.4 m/s about the hips;
- no joint's step changing over 1.7 cm a frame;
- nothing under the floor by more than 1 cm;
- crawling, nothing higher than 0.65 m (a 0.9 m gap clears);
- a limb down sliding at most 4.9 mm;
- at least three limbs always down.

**Live**: down through the half-kneel, crawling with the trunk level, the
hands under the shoulders and the knees under the hips, and up again.

**Cost**: `anim_bench --gait crawl --characters 100`, 10.6 µs a character at
p50.

## Revisit when

- **Turning while crawling**, or crawling backward: only along the facing.
- **A low gap that asks for the crawl itself** (finding it is out of
  scope): the walker crawls only while asked.

## Related

- [A low slab is slid under on the seat from a run](./a-low-slab-is-slid-under-on-the-seat-from-a-run.md) — contrast: under a low gap at a run.
- [Parkour movement data](./parkour-movement-data.md) — data: crawling paces and footfall patterns.
