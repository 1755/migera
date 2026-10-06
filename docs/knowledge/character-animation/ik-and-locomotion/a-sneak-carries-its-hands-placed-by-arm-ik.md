---
title: A sneak carries its hands, placed by arm IK, not its arms swung
description: "sneak::carry_arms places each hand ahead of its shoulder by arm IK (armik::solve_arm_toward, elbow toward a pole), lerped in by depth, the right hand leading, wrists hanging, hands swinging against the legs. Traps: one swing angle for both arms read as a puppet; a near-straight arm in IK softening jumped 12 mm."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-07
verified: 2026-10-07
code:
  - src/character/anim/sneak.rs
  - src/character/anim/armik.rs
  - src/character/anim/walker.rs
sources:
  - "tests sneak::tests::a_sneak_carries_its_hands_ahead_the_elbows_bent_under_and_out, carried_by_nothing_the_arms_are_left_as_they_are, a_sneaks_hands_swing_against_each_other"
  - "live: character_gallery --sneak-schedule 0.5:1 --anim-speed-schedule 0:0,3:0.8,10:0, hand_l/hand_r and pelvis over BRP every frame, Xvfb"
aliases:
  - carry_arms
  - solve_arm_toward
  - CARRIED_HAND
  - CARRIED_ELBOW
  - LEAD_HAND
  - WRIST_DROP
  - HAND_SWING
  - sneak arms
  - sneaking hands
---

# A sneak carries its hands, placed by arm IK, not its arms swung

A sneak carries its hands out in front of it, loose, and moves them a
little as it goes. `sneak::carry_arms` places each hand where it is carried
and lets arm IK find the shoulder and elbow, rather than turning joints by
fixed angles.

## Decision

**Each hand is placed relative to its shoulder** (`CARRIED_HAND`, fractions
of the arm's length): 0.5 ahead, 0.42 below, 0.04 in toward the middle. That
is in front of the belly, the elbow bent about 90°.

**The elbow bends toward a pole** (`armik::solve_arm_toward`): the elbow in
the plane of the shoulder-to-hand line and the pole, on the pole's side. The
pole points down under the shoulder, a little out and back
(`CARRIED_ELBOW`). The arm IK's own configured hinge is one world axis for
every target, which cannot say which way round the elbow goes for a hand
carried in front.

**What makes it read as a person, not a puppet:**
- The right hand is carried 0.06 of the arm further ahead and 0.04 higher
  than the left (`LEAD_HAND`): the two are never mirror images.
- Each hand hangs 20° from its wrist (`WRIST_DROP`), not straight out of
  the forearm. The fingers keep their relaxed curl (`hand::relax_hands`).
- Walking, each hand swings ahead and back against the opposite foot,
  0.05 m a m/s (`HAND_SWING`), rising as it comes forward (`HAND_RISE`).
  The hand moves, not the whole arm from the shoulder.

**Lerped in by depth.** Each hand goes from where the pose has it toward its
place by the crouch's depth, and its elbow pole from the side it bends to
now. At no depth the pose is untouched, so the arms come in as the crouch
does.

**Where it is applied:**
- The crouch (`Footing::pose`): on the upper body before the pelvis is
  solved, so the COM counts the arms.
- Walking crouched (`SneakGait::pose`): again over the walk, with the swing.
- A crouched shuffle (`walker`): over the shuffle, whose own carry (arms out
  from the body) is replaced.

## Traps

- **One swing angle for both arms is a puppet's.** The crouch first swung
  both upper arms forward and bent both elbows by one angle each about the
  rig's left, and walking added the walk's swing held back. The hands were
  side by side, fists straight out of the forearms, swinging fore and aft
  from the shoulder in one plane.
- **Elbows wide with the hands in makes the forearms converge.** Carried
  0.12 of the arm in with the elbow pole 0.7 out, the forearms ran 36° in
  and the hands met over the belly. Now 0.04 in and 0.4 out: 22°.
- **A near-straight arm inside the IK's softening jumps.** The standing arm
  is within 0.4 mm of straight, inside the arm IK's usual 5 mm of
  softening. Asked for its own wrist, its elbow jumped 12 mm out at a weight
  of 1e-4. `solve_arm_toward` uses 0.1 mm.
  - After that the motion is steep but continuous: bringing a near-straight
    wrist in a few millimetres swings the elbow out by more (0.3 mm at
    1e-4, 2.7 at 1e-3, 25 at 0.02). The crouch's ease starts slowly and the
    springs smooth it.
- **Every step's forward kinematics added 26 µs to a crouch.** The two arms
  do not move each other's joints, so one pass serves both, and the solver
  returns the joints it placed: 5 µs.

## Consequences

**Measured** (`puppet_base`, the deepest crouch):
- the hands are 0.22-0.25 m ahead of and 0.19-0.21 m below their shoulders;
- the elbows are 0.24 m below and 0.07 m out from their shoulders;
- the forearms run 22° in; the right hand leads by 3 cm.

**Live** (hands from the pelvis, every frame):
- crouching down, the hands travel 34-37 cm, at most 17 mm and 5 m/s² a
  frame;
- crouched, they hold within 0.5 mm;
- walking at 0.8 m/s, they swing 10.7 cm against each other, at most
  1.8 m/s².

**Seen** (Front and Left, gizmos on and the bare mesh): the hands ahead of
the belly and apart, the elbows under and a little out, the wrists hanging,
one hand ahead of the other walking. On the toes the hands are carried
higher, as the shoulders are. Crouched and shuffling, they are carried, not
held out wide.

**Cost:** a crouch posed 36 µs (31 without), a sneak's walk 43 µs a
character a frame (23 without: root motion poses it several times a frame).

## Revisit when

- A recording of a sneak's arms is found: the carry is authored.
- Crouched and still, the hands hold within 0.5 mm; a slow idle drift of the
  hands would make them less still.

## Related

- [A sneak walks the walk's foot path moved by its crouch](./a-sneak-walks-the-walks-foot-path-moved-by-its-crouch.md) — context: the walk the hands are carried over.
- [A crouch is the jump's countermovement held over the feet](./a-crouch-is-the-jumps-countermovement-held-over-the-feet.md) — context: the crouch the arms are carried in.
- [A walking arm swings back and its hand hangs relaxed](./a-walking-arm-swings-back-and-its-hand-hangs-relaxed.md) — contrast: the walk's arm swing, which a sneak replaces.
