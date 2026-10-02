---
title: A ragdoll's joint muscles have a strength, a twitch and a speed
description: "Joint drives of a body on its own feet are capped per axis and way by maximal voluntary torque per kg (Harbo 2012), scaled by Hill force-velocity fit to Anderson 2007; hinges' sideways bend is structural; only the balance's correction lags by Winter's twitch. Read before tuning joint_drive strengths or lags."
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
  - "Anderson, Madigan & Nussbaum (2007), Maximum voluntary joint torque as a function of joint angle and angular velocity, J Biomech 40:3105, https://stacks.cdc.gov/view/cdc/188754/cdc_188754_DS1.pdf"
  - "Armour et al. (2004), knee internal/external rotation strength; pronation/supination https://pmc.ncbi.nlm.nih.gov/articles/PMC9515161/; wrist deviation https://pubmed.ncbi.nlm.nih.gov/8884484/; hip all directions https://pmc.ncbi.nlm.nih.gov/articles/PMC11329127/"
  - "Thelen (2003), Adjustment of muscle mechanics model parameters to simulate dynamic contractions in older adults, J Biomech Eng 125:70, https://nmbl.stanford.edu/publications/pdf/Thelen2003.pdf"
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed., §7.4.5 (walking moments), §9.0.5 (twitch), §9.2.1-9.2.2 (force-velocity)"
  - "tests joint_drive::tests; ragdoll_plugin::tests::no_joint_of_a_ragdoll_on_its_own_feet_exceeds_its_budget, a_ragdoll_too_weak_for_its_weight_folds, a_struck_arm_goes_slack_and_comes_back_on_its_own_feet"
aliases:
  - joint_strengths
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

- **A strength per joint, axis and way** (`joint_strengths`,
  `JointDrive::budgets`): maximal voluntary isometric torque per kilogram
  of body mass, times the body's mass, about the character's left
  (flexion-extension), forward (abduction-adduction) and up (axial
  rotation), each way. They are recorded in the parent body's frame as
  the character stands (`JointDrive::frame`), so they turn with it. The
  joint's whole torque (drive and fed load together) is resolved onto
  them and each part capped. Left-side table; the right is its mirror
  (forward and up swap ways):

  | joint | about left (+, −) | about forward (+, −) | about up (+, −) |
  |---|---|---|---|
  | ankle | plantar 1.8, dorsi 0.57 | eversion 0.42, inversion 0.46 | 0.3 (est.) |
  | knee | flexion 1.5, extension 3.5 | hinge, 5 | rotation 0.35 |
  | hip | extension 2.5, flexion 2.2 | abduction 1.29, adduction 1.08 | external 0.66, internal 0.72 |
  | trunk | flexion 2.2, extension 3.0 | lateral 1.75 | rotation 1.25 |
  | neck, head | flexion 0.40, extension 0.70 | lateral 0.48 | rotation 0.20 |
  | shoulder | extension 1.25, flexion 1.0 (est.) | abduction 0.84, adduction 1.07 | external 0.4, internal 0.6 (est.) |
  | elbow | extension 0.61, flexion 0.67 | hinge, 5 | supination 0.127, pronation 0.076 |
  | wrist | ulnar 0.13, radial 0.15 | extension 0.155, flexion 0.33 | 0.1 (est.) |

  Sources: Harbo et al. 2012 (young man, their regressions at 25 y,
  1.80 m, 80 kg) for the main flexions; Armour 2004 (knee rotation);
  Vasavada 2001 (neck); a 30-subject isometric hip study (hip frontal and
  rotation); others in `joint_strengths`' doc.
- **A hinge's sideways bend is structural, not muscular.** The knee and
  elbow are ball joints standing, so their drives also stand for the
  bones and ligaments that hold them in plane: 5 N·m/kg there. Capped at
  Winter's walking knee abductor moment (0.6) as if muscle, a side push
  that the body then caught kept it wandering, and −0.4 m/s sideways fell.
- **Force-velocity** (`force_velocity`): each part's cap scales with the
  joint's speed about that axis in the way it pulls, over a maximum
  speed. Shortening follows Hill's hyperbola `(1 − v)/(1 + v/0.25)`,
  Winter's form with the literature's curvature. Lengthening rises to
  Thelen's young-adult 1.4, within Winter's 1.1-1.8, leaving rest with
  the same slope. The maximum speeds are where that curve best fits
  Anderson, Madigan & Nussbaum's (2007) measured torque-velocity in young
  men: ankle 21 rad/s, knee 24, hip 19 (theirs is near-linear there).
  The elbow's 16.5 is its unloaded flexion speed; the trunk's, neck's,
  shoulder's and wrist's are estimates.
- **The balance's correction lags by a twitch** (`twitch_seconds`,
  `activate`): what each joint carries as the body stands now (the
  pressure straight under the COM) is the muscles' tone, applied at once.
  What the balance adds on top reaches them through a critically damped
  lag, Winter's twitch `F0·(t/T)·e^{−t/T}`, solved exactly per step. The
  drive's stiffness and damping also act at once, as a muscle's
  short-range stiffness does.
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
- **Lagging the whole command**, the weight carried as well as the
  balance. A leaning trunk's weight arrived after it had leant further:
  after a 0.3 m/s side push the body swayed ±10-18 mm and the trunk
  1-4.6°, without end, and 0.5 m/s backward fell
  (`a_side_push_settles_on_its_own_feet`: 28.9 mm of wander, now under
  3).
- **One cap on the torque's size**, whichever way it points (the first
  version). It let an ankle lift the toes with its plantarflexors'
  strength, three times its dorsiflexors'.

## Consequences

- **Pushes caught on the feet without a step**, with every limit on and
  the strength per axis (2026-10-02): forward (−Z on the drawn rig) 0.4
  m/s caught, 0.5 falls; back 0.5 caught, 0.6 falls; sideways 0.5
  caught, 0.6 falls. 0.4-0.5 m/s before a step is needed is in the
  human range. Why forward is the weakest is open; it is not the
  dorsiflexors (a forward push loads the plantarflexors). Earlier
  versions of this note had forward and back swapped: the drawn rig
  faces −Z, so a +Z push is backward. Lagging the whole command instead
  cut the limits by about 0.1 m/s, and at twice the twitch even 0.4 fell.
- **The axes are checked against the physics**
  (`a_standing_body_pulls_each_joint_the_anatomical_way`): pushed
  forward, the ankle pulls the toes down (positive about left) by over
  20 N·m; pushed back, it lifts them, never past the dorsiflexors'
  budget. Standing still, an ankle carries only ~2.5 N·m about left (the
  bodies' COM stands within millimetres of the ankles) and a knee 9.6,
  mostly frontal (the wide stance), as Winter's quiet stance has small
  knee moments.
- **The budgets bind.** At a tenth of its strength the body folds, the
  hips sinking over 15 cm (`a_ragdoll_too_weak_for_its_weight_folds`;
  without the cap, 4 mm).
- **A 0.4 m/s push and a 4 m/s blow to a forearm stay within every
  joint's strength.** `no_joint_of_a_ragdoll_on_its_own_feet_exceeds_its_budget`
  passes without the cap too, so it records a result, not the cap.
- **A struck forearm** swings over 10° and is back within 5° in 3 s,
  standing.

## Revisit when

- Sources turn up for the estimates (shoulder rotation sizes, the
  trunk's, neck's, shoulder's and wrist's maximum speeds).
- A character is not a young man: the table is per kg, but strength per
  kg falls with age and differs by sex (Harbo: women ~10-30 % less).
- Strength should vary with joint angle (the force-length curve, Winter
  §9.1).

## Related

- [A standing ragdoll carries its weight through joint torques](./a-standing-ragdoll-carries-its-weight-through-joint-torques.md) — prerequisite: the drives these limits act on.
- [9.0.5 The muscle twitch](../../biomechanics-winter/ch09-muscle-mechanics/9.0-motor-units-and-twitches/9.0.5-muscle-twitch.md) — source: the twitch times and shape.
- [9.2.1 Concentric contractions](../../biomechanics-winter/ch09-muscle-mechanics/9.2-force-velocity-characteristics/9.2.1-concentric-contractions.md) — source: Hill's hyperbola.
- [7.4.5 Sample moment and power curves](../../biomechanics-winter/ch07-three-dimensional-kinematics-and-kinetics/7.4-kinetic-analysis-reactions-and-moments/7.4.5-sample-moment-and-power-curves.md) — contrast: walking peaks, not capacity.
