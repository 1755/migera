---
title: A ragdoll's joint muscles have a strength, a twitch and a speed
description: "Each joint drive of a body on its own feet is capped by maximal voluntary torque per kg (Harbo 2012, not Winter's walking peaks), scaled by Hill force-velocity, and its commanded torque lags by Winter's twitch time. The lag cuts the push a body catches without a step. Read before tuning joint_drive budgets or lags."
type: decision
status: current
tags:
  - ragdoll
  - biomechanics
  - physics
  - numerics
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/joint_drive.rs
sources:
  - "Harbo, Brincks & Andersen (2012), Maximal isokinetic and isometric muscle strength of major muscle groups related to age, body mass, height, and sex in 178 healthy subjects, Eur J Appl Physiol 112:267, https://link.springer.com/article/10.1007/s00421-011-1975-3"
  - "Vasavada, Li & Delp (2001), neck strength, https://pubmed.ncbi.nlm.nih.gov/11568704/"
  - "Thelen (2003), Adjustment of muscle mechanics model parameters to simulate dynamic contractions in older adults, J Biomech Eng 125:70, https://nmbl.stanford.edu/publications/pdf/Thelen2003.pdf"
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed., §7.4.5 (walking moments), §9.0.5 (twitch), §9.2.1-9.2.2 (force-velocity)"
  - "tests joint_drive::tests; ragdoll_plugin::tests::no_joint_of_a_ragdoll_on_its_own_feet_exceeds_its_budget, a_ragdoll_too_weak_for_its_weight_folds, a_struck_arm_goes_slack_and_comes_back_on_its_own_feet"
aliases:
  - joint_budget
  - force_velocity
  - twitch_seconds
  - activate
  - torque budget
  - activation lag
  - muscle strength cap
---

# A ragdoll's joint muscles have a strength, a twitch and a speed

A body standing on its own feet carries itself with joint torques
(see [a standing ragdoll carries its weight](./a-standing-ragdoll-carries-its-weight-through-joint-torques.md)).
Plan steps 4.2 and 4.4 give those torques three limits a real muscle
has: a maximum, a delay and a speed.

## Decision

- **A budget per joint** (`joint_budget`, `JointDrive::budget`): maximal
  voluntary isometric torque per kilogram of body mass, times the body's
  mass. It is one cap on the size of the joint's whole torque (drive and
  fed load together), in the direction that carries the body. Young men,
  from Harbo et al.'s regressions at 25 y, 1.80 m, 80 kg:

  | joint | N·m/kg | from |
  |---|---|---|
  | ankle | 1.8 | plantarflexion: isokinetic 1.6, isometric 10-30 % higher |
  | knee | 3.5 | extension |
  | hip | 2.5 | extension (flexion 2.2) |
  | trunk | 3.0 | lumbar extension, 241 N·m in men |
  | neck, head | 0.7 | extension, 52 N·m (Vasavada 2001) |
  | shoulder | 1.0 | adduction 1.07, abduction 0.84 |
  | elbow | 0.67 | flexion |
  | wrist | 0.33 | flexion |
- **Force-velocity** (`force_velocity`): the cap scales with the joint's
  speed in the direction it pulls, over a maximum speed (12 rad/s for the
  legs and trunk, 20 for the arms; estimates). Shortening follows Hill's
  hyperbola `(1 − v)/(1 + v/0.25)`, Winter's form with the literature's
  curvature. Lengthening rises to Thelen's young-adult 1.4, within
  Winter's 1.1-1.8, leaving rest with the same slope.
- **A twitch per joint** (`twitch_seconds`, `activate`): the commanded
  torque (the load and the balance) reaches the muscles through a
  critically damped lag, Winter's twitch `F0·(t/T)·e^{−t/T}`, solved
  exactly per step. The drive's own stiffness and damping act at once, as
  a muscle's short-range stiffness does.
  - Legs: 75 ms (soleus 74, gastrocnemius 79).
  - Arms: 50 ms (biceps 52, triceps 44.5).
  - Trunk and neck: 60 ms, a choice; Winter gives none.
- **A hit's stun weakens the drive** (`JointDrive::strength`): the
  joint's effective strength scales its stiffness, damping and command,
  so a struck forearm swings slack and its muscles take it back.

## Alternatives considered

- **Winter's walking peaks as budgets** (§7.4.5: ankle 1.6, hip 1.0,
  knee 0.5 N·m/kg). These are what a walk uses, not what a body can
  give. A standing knee already carries 0.49 N·m/kg, so its budget would
  have had nothing to spare.
- **Thelen's activation dynamics** (15 ms on, 50 ms off) instead of the
  twitch time. That models excitation to activation only; the twitch time
  is command to force, what the feed-forward stands for.
- **Lagging the drive's stiffness as well.** It would leave a joint with
  no immediate resistance to a blow, which real muscle has.

## Consequences

- **The lag costs pushes.** Caught without a step, measured on the same
  pushes:

  | | no lag | Winter's twitch | twice it |
  |---|---|---|---|
  | forward | 0.5 | 0.4 | under 0.4 |
  | back | 0.4 | 0.4 | under 0.4 |
  | sideways | 0.6 | under 0.6 | under 0.4 |

  A delayed controller absorbs less: 0.4-0.5 m/s before a step is needed
  is in the human range.
- **The budgets bind.** At a tenth of its strength the body folds, the
  hips sinking over 15 cm (`a_ragdoll_too_weak_for_its_weight_folds`;
  without the cap, 4 mm).
- **A 0.4 m/s push and a 4 m/s blow to a forearm stay within every
  joint's strength.** `no_joint_of_a_ragdoll_on_its_own_feet_exceeds_its_budget`
  passes without the cap too, so it records a result, not the cap.
- **A struck forearm** swings over 10° and is back within 5° in 3 s,
  standing.

## Revisit when

- Direction-specific strength matters (a dorsiflexing ankle has 0.57
  N·m/kg against 1.8 plantarflexing): split the cap per axis.
- A character is not a young man: the table is per kg, but strength per
  kg falls with age and differs by sex (Harbo: women ~10-30 % less).
- Strength should vary with joint angle (the force-length curve, Winter
  §9.1).

## Related

- [A standing ragdoll carries its weight through joint torques](./a-standing-ragdoll-carries-its-weight-through-joint-torques.md) — prerequisite: the drives these limits act on.
- [9.0.5 The muscle twitch](../../biomechanics-winter/ch09-muscle-mechanics/9.0-motor-units-and-twitches/9.0.5-muscle-twitch.md) — source: the twitch times and shape.
- [9.2.1 Concentric contractions](../../biomechanics-winter/ch09-muscle-mechanics/9.2-force-velocity-characteristics/9.2.1-concentric-contractions.md) — source: Hill's hyperbola.
- [7.4.5 Sample moment and power curves](../../biomechanics-winter/ch07-three-dimensional-kinematics-and-kinetics/7.4-kinetic-analysis-reactions-and-moments/7.4.5-sample-moment-and-power-curves.md) — contrast: walking peaks, not capacity.
