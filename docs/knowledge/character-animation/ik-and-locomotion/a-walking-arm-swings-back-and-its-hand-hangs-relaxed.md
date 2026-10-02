---
title: A walking arm swings back from the shoulder and forward from the elbow, and its hand hangs curled
description: "Murray 1967: the upper arm swings 8° forward and 24° back of vertical, the elbow folds 17-47°, so the hand leads by the elbow. The walk had it reversed (28°/10°, a 10° elbow), a march; fingers sat flat at the bind. Fingers now curl to Lee et al.'s relaxed angles. Read before changing arm swing or hands."
type: decision
status: current
tags:
  - locomotion
  - poses
  - biomechanics
  - correctness
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/gait.rs
  - src/character/anim/hand.rs
  - src/character/anim/humanoid.rs
sources:
  - "Murray, Sepic & Barnard (1967), Patterns of sagittal rotation of the upper limbs in walking, Phys Ther 47(4):272-284, https://academic.oup.com/ptj/article/47/4/272/4638307 — 30 men, free speed 154 cm/s"
  - "Lee, Mo, Hwang, Wang & Jung, Relaxed hand postures, J. Ergonomics 44 spl. 436, https://www.jstage.jst.go.jp/article/jergo/44spl/0/44spl_0_436/_pdf — 15 men, Vicon, Table 3"
  - "live BRP: character_gallery --anim-speed 1.3, left arm and fingers sampled over 3 s, before and after"
  - "tests gait::tests::a_walking_arm_swings_back_from_the_shoulder_and_forward_from_the_elbow, hand::tests"
aliases:
  - arm swing
  - ARM_SWING_CENTRE
  - ARM_FORWARD_BIAS
  - elbow_carry
  - relaxed hand
  - RelaxedHands
  - finger curl
  - relax_hands
  - curl_hands
---

# A walking arm swings back from the shoulder and forward from the elbow, and its hand hangs curled

In walking, the upper arm spends most of the stride behind the body, and
the hand comes forward because the elbow folds. Marching does the opposite:
it throws a straight arm forward. A relaxed hand is curled, not flat.

## Context

The walk's hands read as stiff, like a soldier's. Measured live on the
gallery's character at 1.3 m/s, against the published walk:

| | The walk | Murray 1967, free speed |
|---|---|---|
| Upper arm from vertical, forward / back | +28° / −10° | +8° / −24° (SD 10 / 6) |
| Elbow flexion | 20–30° | 17–47° (SD 8 / 11) |
| Fingers, knuckle to tip | 9°, flat (the bind) | curled (Lee et al., below) |
| Palm | faces the thigh, thumb forward | the same |

The code centred the swing forward (`ARM_FORWARD_BIAS = +0.3`), from a
belief that an arm swings about 2:1 forward. That ratio holds for the
**hand**, and the elbow carries the hand forward. The upper arm itself
swings mostly backward.

## Decision

- **Shoulder** (`gait.rs`): `ARM_SWING_CENTRE = −0.5` and `arm_swing` 0.21
  rad. At Murray's 1.54 m/s (`stride_scale_for` ×1.32) this gives 7°
  forward and 25° back. Timing is unchanged: peak flexion just after the
  opposite footfall, which matches Murray (50% of the cycle).
- **Elbow:** a new `GaitParams::elbow_carry`, the share of the fold kept at
  the back of the swing. The walk uses `elbow_bend` 0.73 and carry 0.29,
  which gives 17–46° on puppet_base (the stand is about 5°). The run keeps
  its 0.6, since a running arm stays folded.
- **Fingers** (`hand.rs`): the rig's finger joints are not `Bone`s, so
  nothing posed them. `relax_hands` bends each finger once, as the rig
  binds. The finger names follow the UE Mannequin or Mixamo convention. Lee
  et al.'s relaxed angles (neutral forearm, arm hanging), in degrees:

  | | MCP | PIP | DIP |
  |---|---|---|---|
  | Index | 28.4 | 25.5 | 13.1 |
  | Middle | 32.8 | 30.1 | 12.7 |
  | Ring | 24.7 | 34.5 | 11.7 |
  | Little | 16.6 | 32.1 | 15.6 |

  The thumb bends 46.1° at the MCP and 8.5° at the IP joint.
  - The palm's normal is read from the rig: across the knuckles crossed
    with the middle finger, in an order that depends on the side. A test
    pins it pointing down on the T-pose bind, independent of that formula.
  - The thumb bends across the palm, toward the little finger, not toward
    the palm. Its bind already points 0.45 out of the palm. Bent toward the
    palm like a finger, it stuck out sideways into the thigh.
- **A hand on the floor lies flat.** Curled, a fallen body's fingertips
  went 45 mm into the floor while lying. `curl_hands` straightens the
  fingers over 0.3 s while `Ragdoll::is_falling`, and curls them again once
  the character stands.

## Consequences

- Live, 1.3 m/s:
  - upper arm +9° / −18°;
  - elbow 18–46°;
  - middle finger 56° from the knuckle line to the tip chord;
  - palm still toward the thigh.

  Seen Front and Left, with gizmos and with the mesh: the forearm leads,
  the upper arm trails, and the thumb lies along the index finger.
- The hand still swings further forward than back, by the elbow
  (`a_walking_arm_swings_further_forward_than_back` passes unchanged).
- **Already there, not caused by this change:** during the get-up's
  push-up, the fingertips dip 84–90 mm under the floor for a moment, on the
  old build too (flat fingers). The get-up places a virtual fingertip
  (`HAND_PER_FOREARM`), not the rig's own finger joints.

## Revisit when

- A hand grips or carries something: the curl becomes a per-hand target,
  not a constant.
- The get-up should keep real fingertips out of the floor: it must place
  the rig's own finger joints.
- Arm swing should change with age or a load: Murray's SDs are 6–11°, and
  the swing grows at a fast pace (shoulder −31° back, elbow 15–55°).

## Related

- [A gait-timed motion cannot ride a weighty spring](./a-gait-timed-motion-cannot-ride-a-weighty-spring.md) — prerequisite: why the arms spring at 0.03 s, so the measured swing is the authored one.
- [Getting up goes through key poses](../ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md) — context: the flat palm the fingers straighten onto, and its virtual fingertip.
- [A measurement of a broken system](../../engineering-practice/measurement/a-measurement-of-a-broken-system.md) — same-trap: the forward bias was tuned by eye after the swing was fixed to move the hands, and was never checked against a recording.
