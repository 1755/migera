---
title: Monkey bars are crossed hand over hand, the body hung from the hands carrying it
description: "Step 17, first part: monkey bars are crossed by moving each hand in turn two bars on, the hips on a smooth path slow between the hands and fast under the holding one, hung from the hands weighted by how much each carries, twisted toward the leading hand. Read before changing parkour/monkey.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/character/anim/parkour/monkey.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
  - examples/anim_bench.rs
sources:
  - "tests parkour::monkey::tests::{monkey_bars_are_crossed_hand_over_hand, a_line_of_jugs_under_a_roof_is_crossed_hand_over_hand}"
  - "live: character_gallery --step-seconds 0.0333333 --monkey 0,-2,0,2.3,0.4,6 --monkey-at 0.5, Xvfb, gizmos on/mesh off Left and Back, mesh on Left and Back; BRP pelvis, hands and feet"
  - "anim_bench --gait monkey --characters 100"
aliases:
  - monkey bars
  - brachiation
  - MonkeyBars
  - Crossing
  - Walker::monkey_bars
---

# Monkey bars are crossed hand over hand, the body hung from the hands carrying it

Step 17 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), first part: a line of
bars overhead, crossed hand over hand. There is no monkey-bar data. The
pace is a hand move every 0.9 s, set by eye: a free hang's pendulum is
about 2.4 s, and a brachiating step about a third of it.

## Decision

**A line of bars** (`parkour::MonkeyBars`): the first bar's middle on its
axis, the way along the line (level), the spacing and the count. The
walker is asked `Walker::monkey_bars`. It walks to the spot under the
first bar, facing along the line, and gets on. It crosses, hangs still
under the last bar for 0.4 s, lets go into a fall and lands.

**Getting on** (0.8 s), as the pole's is:
- the hands are swept up round the shoulders through the front;
- the hips rise to the hang under the first bar over the same 0.6 s;
- the feet leave the floor no farther from the hips than hanging;
- the standing pose eases out over 0.2 s.

**The hand moves** (`Crossing::moves`): the right hand to bar 1, then
each hand in turn two bars on, to the bar past the other's. At the last
bar the other hand matches it. Each move is 0.9 s (`STEP`):
- for its first quarter both hands hold;
- then the moving hand goes, dipping 8 cm under the bars (`sin²`);
- its grip opens and closes over a fifth of its way each end.

**The hips** (`along_at`): a cubic Hermite through knots at each move's
end, where the hips are midway between the hands.
- At the knots they go half the mean speed (`SWAY`), so through the middle
  of a move, under the holding hand, a pendulum's faster.
- From rest under the first bar, to rest under the last.
- Their height: hung from the point the body hangs from, the hang's length
  below it. That length is the shoulders over the hips, the arm at 0.94 of
  its length, and the wrist under the bar.
- They rise as far as two hands apart along the line need, to reach.

**The point it hangs from** is the hands' bars weighted by how much each
carries (`carried_at`). A moving hand's share eases out to none half way
through its move, the hips then under the other hand's bar, and back by
its end. A quintic ease is used: no speed nor acceleration at either end.

**The body**:
- the trunk is turned whole toward that point;
- it is twisted 0.35 rad toward the leading hand as each hand catches,
  through none between (`twist_at`);
- the legs hang along the line from that point through the hips, piked
  0.15 rad ahead;
- each hand closes round its bar, the palm forward, the fingers on along
  its forearm: first from the shoulder as standing carries it, then from
  the elbow the arm solved to, solved again.

## Alternatives considered

- **The hips on pendulum arcs about each holding hand**: the arcs meet at
  a corner. The way through it turns 2α (0.36 rad at 0.4 m spacing),
  so the motion is not smooth there. A path through the hand-move ends
  is smooth by construction.
- **The point hung from weighted by each hand's grip**: see Traps.
- **Pendulum dynamics for the swing**: a brachiator does ride its
  pendulum, but here the pace is asked. The path keeps the pendulum's
  shape (fast under the holding hand) and its pace.

## Traps

- **A grip that let go at once** moved the point hung from in a frame.
  A toe's step changed 18 cm. Eased by the grip alone (0.14 s), the
  legs' line still swung 0.2 rad and a foot stepped 2.6 cm. The share a
  hand carries is eased over its whole move instead.
- **A cubic ease** has acceleration at its ends: a hand's hold finishing
  opening stepped a toe 2.3 cm. Hence the quintic.
- **The hands swept from hanging to overhead by the shortest arc**: near
  half a turn has no steady axis, and an elbow stepped 2.7 cm. They are
  swept through the front about the shoulder, then turned the small rest
  of the way onto the hold.
- **The clavicle lift's limit changes** from reaching forward to up. As the
  swept hands passed between them, a shoulder stepped 2.3 cm. The lift is
  eased in with the sweep.
- **Hips rising over the whole of getting on**: the hands met the bar with
  the hips 10 cm under the hang. The arms were clamped near straight and
  an elbow swung 2 cm. The hips now rise with the hands.
- **A hand turned by its shoulder as solved** (lifted by the clavicle) and
  checked by its shoulder before: 3.2 mm apart. The pose and the check now
  read the same posed result.
- **The rig's real finger grips** (in the hand bone's own frame) taken
  unturned bent each held hand 1.64 rad at the wrist in the gallery, while
  the test, on the estimated grip, passed. Turned toward the line from the
  shoulder only, 0.68 rad. See [a hand grip is measured in the hand's own
  frame](../rig-and-retargeting/a-hand-grip-is-measured-in-the-hands-own-frame.md).

## Consequences

**Headless** (`puppet_base` and its own fingers' grips, 60 fps; 5, 8 and 5
bars, 0.4, 0.35 and 0.45 m apart, 2.3 m up, along -Z and +X):
- held hands within 1 µm of their place on their bars, the wrist within
  0.07 rad of the forearm sideways and flexed 0-0.1 rad;
- nothing within 3 cm of a bar's surface but the hands and forearms;
- no joint over 3.8 m/s about the hips;
- no step changed over 1.4 cm in a frame (the hands' sweep getting on);
- across in the time its moves take (a move a bar and the match, then the
  rest and the hold);
- let go under the last bar, it landed under it.

**Live** (Xvfb, BRP): walked under the first of six bars, got on, crossed
2 m in hand moves to the last, hung, let go and stood under it. Each wrist
held at its bar, the pelvis level at 1.27 m. Left and Back with gizmos on
and the mesh off, and with the mesh: the hands on the bars, the body
hanging between them.

**Cost**: `anim_bench --gait monkey --characters 100`, 41 µs a character
at p50 when built (a pole climb's 39). Re-measured 2026-10-10: 69-71 µs at
commit fd84d28 (built in a worktree) and 68 µs with the roof added, so the
rise came from commits in between, not the roof.

**Under a roof** (step 15, hand over hand under a ceiling): a line of
jugs on a roof's underside is these bars (`MonkeyBars::roof`), each held
8 cm under it (`JUG_DROP`). Crossing six 0.4 m apart under a roof 2.4 m
up, nothing rose over a held wrist (2.237 m) but the fingers round the
jug, 4 cm clear of the roof
(`a_line_of_jugs_under_a_roof_is_crossed_hand_over_hand`; the
[overhang's note](./an-overhang-is-climbed-as-the-upright-climb-turned-with-its-face.md)
has the live check).

## Revisit when

- **The body swinging under the holding hand sideways**: the hips stay on
  the line's middle. A real brachiator shifts under the holding arm.
- **Getting on from a hang or a jump at speed**, getting off onto a top,
  skipping bars, crossing backward: not built.
- **Data**: no monkey-bar timings were found. The pace and the twist are
  set by eye.

## Related

- [A pole is climbed as an inchworm, hands over a leg clamp](./a-pole-is-climbed-as-an-inchworm-hands-over-a-leg-clamp.md) — contrast: the other self-contained mover on a fixture; its get-on sweep this reuses, and its traps.
- [A bar is swung on, pumped, and let go of at a bar ahead](./a-bar-is-swung-on-pumped-and-let-go-of-at-a-bar-ahead.md) — contrast: one bar, its pendulum driven; the hand's roll round a bar.
- [A ledge is shimmied hand over hand and round corners](./a-ledge-is-shimmied-hand-over-hand-and-round-corners.md) — contrast: hand over hand along one ledge, not across bars.
- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 17's design.
- [An overhang is climbed as the upright climb turned with its face](./an-overhang-is-climbed-as-the-upright-climb-turned-with-its-face.md) — applies (2026-10-10): these bars as jugs under a roof, crossed hand over hand.
- [A hand grip is measured in the hand's own frame](../rig-and-retargeting/a-hand-grip-is-measured-in-the-hands-own-frame.md) — deeper: why held hands bent the wrong way, and the wrist check.
