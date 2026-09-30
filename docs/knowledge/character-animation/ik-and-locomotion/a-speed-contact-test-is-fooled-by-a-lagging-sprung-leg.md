---
title: A speed contact test is fooled by a lagging sprung leg
description: "The foot lock guesses contact from the sprung toe's speed; under a fast pelvis a planted foot's toe lags at 0.5-0.8 m/s, so in a stumble its lock let go 4 times (16-21 mm slide) while a headless replay held 1 mm. A controller that knows contact passes AnimFootIk::planted. Read before driving feet from a controller."
type: lesson
status: current
tags:
  - ik
  - springs
  - correctness
  - debugging
updated: 2026-09-30
verified: 2026-09-30
code:
  - src/character/anim/footlock.rs
  - src/character/anim/plugin.rs
  - examples/character_gallery.rs
sources:
  - "test footlock::tests::a_planted_foot_holds_its_lock_against_speed_but_not_a_drag"
  - "live character_gallery --push-schedule 5:0:0.7 with a temporary print on each lock release"
aliases:
  - planted
  - update_planted
  - foot lock releases a planted foot
  - contact heuristic
---

# A speed contact test is fooled by a lagging sprung leg

`FootLock` decides contact from the **animated** toe's speed (lock below
`lock_speed`, release above `unlock_speed`). The animated toe is the
sprung pose's, before IK. When the unsprung root moves fast, the leg
bones' springs lag it, so the toe of a foot that is really planted
moves. A lock that speed alone guards then releases a planted foot. When
some code already *knows* which feet are down, it must say so
(`AnimFootIk::planted`) instead of leaving the guess to the lock.

## What happened

A 0.7 m/s sideways stumble (`balance::Balance`, H1) moves the pelvis at up
to ~0.7 m/s. The balance places the planted foot exactly, and the headless
replay of the same loop, which has no springs and no foot IK, held it
within 1 mm a frame. Live, the planted right ball slid 16 mm on
`puppet_base` and 21 mm on `character.glb`.

A temporary print on each lock release showed four releases on the
planted foot during the lunge, all by speed and none by drag: speeds
0.49–0.78 m/s, gaps to the anchor 11–23 mm. Each release blended the foot
toward the lagging animated position, and the lock re-planted it
somewhere else.

After the fix, where the gallery sets `planted` from `Balance::planted()`
while the balance is active and no walk is blended in, every planted
ball stayed ≤ 1.0 mm on both rigs. That held through forward, sideways
and backward stumbles.

## Why it matters

- **The headless test could not see it.** It exercises the controller's
  target pose; the bug lives in springs plus locks downstream. A clean
  headless replay says the *target* is right, not the rendered foot.
- **Speed is the lock's evidence for contact because it has no other.**
  Foot locking on authored animation (Holden) has to infer contacts. A
  controller that plants feet itself has the ground truth, and
  re-inferring it through a lagging spring throws that away.
- The same root cause, a fast unsprung pelvis ahead of the leg springs,
  showed up before as a one-frame push re-planting feet 23 mm away (fixed
  by spreading pushes over 0.1 s). Slowing the body is not an option for a
  stumble, whose speed is the point.

## How to apply

- A controller that owns contact writes `AnimFootIk::planted` every frame
  and writes `[false; 2]` whenever it isn't in charge. A walk's feet stay
  the lock's call.
- `planted` only suppresses the **speed** release. Locking stays
  heuristic, so a landing foot locks only once the rendered foot has
  arrived. Forcing a lock at the swing's end would plant a foot still
  lagging ~10 cm. A drag past `break_distance` still releases it.
- When a headless replay is clean and live is not, suspect springs and
  locks first: add a print on the lock's state changes (`FootLock` is not
  reflected, so BRP can't show it).

## Evidence

- `footlock::tests::a_planted_foot_holds_its_lock_against_speed_but_not_a_drag`:
  the same 1 m/s motion releases an unplanted lock and not a planted one,
  and a 1 m drag releases both.
- Live `stuman.py` over `--push-schedule 3:0.6:0,8:0:0.7,13:-0.8:0`: planted
  slide 16.3/21.4 mm before, 0.1/0.4 mm after (sideways push, `puppet_base`
  and `character.glb`).

## Related

- [A stumble is a capture-point step, then a join](./a-stumble-is-a-capture-point-step-then-a-join.md) — applies: the controller that passes `planted`.
- [Push recovery is Winter's pendulum](./push-recovery-is-winters-pendulum.md) — same-trap: a one-frame push released the locks the same way.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — deeper: the other thing a lock cannot see and must be told.
- [A lagging pelvis rotation slides planted feet](./a-lagging-pelvis-rotation-slides-planted-feet.md) — same-trap: spring lag between pelvis and legs, seen through the feet.
