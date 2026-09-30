---
title: A stumble is a capture-point step, then a join the body's momentum carries
description: "balance::Balance steps to the predicted capture point, then the trailing foot joins once the capture point is inside the stepped foot, not once the COM is over it. The swinging leg holds the pelvis; a rear foot rolls onto its toes. Pelvis sinks 46/44/115 mm, feet ≤ 1 mm live. Read before changing stepping in balance.rs."
type: decision
status: current
tags:
  - balance
  - biomechanics
  - locomotion
  - ik
  - correctness
updated: 2026-09-30
verified: 2026-09-30
code:
  - src/character/anim/balance.rs
  - src/character/anim/stance.rs
  - examples/character_gallery.rs
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §11.2.1 (Eq. 11.3); Appendix A frames 63-70 (pre-swing roll onto the toes)"
  - "tests balance::tests::a_stumble_steps_cleanly_on_the_real_rig, a_stumble_plans_the_same_steps_at_uneven_frame_times"
  - "live BRP, character_gallery --push-schedule 3:0.6:0,8:0:0.7,13:-0.8:0, puppet_base and character.glb"
aliases:
  - stumble step
  - recovery step
  - join step
  - weight transfer
  - heel rise
  - MAX_HEEL_RISE
  - DROP_BEFORE_HEEL_RISE
  - move_pelvis_and_feet
---

# A stumble is a capture-point step, then a join the body's momentum carries

When a push puts the capture point `x + ẋ√K` outside the feet,
`balance::Balance` takes one recovery step to where the capture point will
be when the foot lands. The weight then moves onto that foot, and the
trailing foot joins it once the body's momentum will carry the COM onto
the stepped foot. The body does not wait until it is already over it. The
character then moves by the distance stepped (root motion), and the feet
stand side by side as before.

## Context

Pushes the feet can absorb were already handled by the pendulum sway (see
[push recovery](./push-recovery-is-winters-pendulum.md)); `needs_step` was
the hook left for this. H1 of the hybrid plan: a stumble stays kinematic,
and physics takes over only for a fall.

## Decision

- **Landing: the predicted capture point.** During the swing the COP
  stands on the other foot, and the capture point runs away from it as
  `e^{t/√K}`. So the foot goes to `p + (cp − p)·e^{T/√K}`, with
  `STEP_SECONDS` = 0.3. Travel is clamped to `MAX_STEP` = 0.4 m. The
  sideways component is kept only for a sideways push. The stepping leg
  is on the side the capture point left through; straight ahead or back,
  it is the unloaded leg.
- **The join starts when the capture point is inside the stepped foot**,
  not when the COM is over it. Both feet are down first (`transfer`), so
  the COM moves toward the stepped foot. Waiting until it was over that
  foot kept the rear leg planted while it reached a foot 0.4 m away. The
  pelvis sank 14 cm forward and 21 cm sideways (`COM_PER_PELVIS` puts the
  pelvis beyond the COM).
- **The swinging foot keeps holding the pelvis in reach.** Its leg gets
  load 0.1 in the drop, aimed at the foot's moving, lifted target. When
  it was let go, the pelvis sprang up 132 mm in the frame the trailing
  foot lifted.
- **A trailing foot rolls onto its toes instead of squatting the body.**
  `move_pelvis_and_feet(.., rise)` lets a foot whose tip trails its hip
  socket pitch up by as much as `MAX_HEEL_RISE` (0.6 rad). It does so only
  once, held flat, the foot would ask the pelvis to drop more than
  `DROP_BEFORE_HEEL_RISE` (4 cm). It rolls rigidly about the sole's
  **tip**, because `foot::Sole` is a rigid foot. About the toe joint, the
  rigid tip went into the floor. This is also Winter's pre-swing: the
  metatarsal marker climbs while the toe marker stays down. The walk's
  callers pass `rise = 0` and are unchanged.
- **The pelvis carries the sway, not the step.** Pelvis =
  `carried + (offset − carried)/COM_PER_PELVIS`, where `carried` is the
  feet's mean displacement. Scaling the whole offset put the pelvis 60 mm
  ahead of itself by the end of a step, and the hand-over popped it back.
- **One solve** (`stance::move_pelvis_and_feet`): the pelvis height accounts
  for where each loaded foot is going, and each ankle is placed once under
  the moved pelvis. Placing the stepped foot first, under a pelvis not yet
  over it, left that foot out of reach in the air.
- **The foot IK is told which feet are down** (`AnimFootIk::planted`); see
  [a speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md).
  The landing hint (`Balance::landing_spot`) eases in over the first
  quarter of the swing and fades out over `LAND_HOLD` after touchdown. At
  full strength from lift-off, the ball popped up 30 mm. Held at full
  strength until the hold ended, it dropped 20 mm in one frame.

## Alternatives considered

- **Carry the COM fully over the stepped foot, then join.** This lost for
  the reason above: the rear leg's reach sets the pelvis height (14–21 cm
  sinks).
- **Shorter steps, so the stance is never wide.** This would under-catch
  hard pushes. The capture-point prediction asks for about 0.4 m at
  0.6–0.8 m/s.
- **Leave the rear foot to the foot IK's toe lock**, which re-aims a foot
  it can't reach. This couldn't be tested headless, and the balance's own
  pose lifted the whole foot flat first.

## Consequences

Measured headless on `puppet_base` (`a_stumble_steps_cleanly_on_the_real_rig`):

| push | steps | pelvis sank | note |
|---|---|---|---|
| 0.6 m/s forward | 0.40 m, join | 46 mm | 69 mm without the heel rise; rear heel rises 9 mm |
| 0.7 m/s left | 0.40 m (0.39 sideways), join | 115 mm | feet 0.63 m apart; leg length alone asks ~10 cm |
| 0.8 m/s back | 0.40 m, join | 44 mm | 0.6 m/s back needs no step: the real soles reach 0.18 m behind the COM, 0.11 m ahead |

Live on both rigs, same pushes: every planted ball stayed within
≤ 1.0 mm (`character.glb` catches the 0.6 m/s forward push without a
step). No vertical pop outside the swing arc, and the pelvis sank at most
123 mm (`puppet_base`, sideways). The planned steps are identical under
frame times cycling 5–50 ms.

## Revisit when

- A falling push (H2): the capture point beyond `MAX_STEP`'s reach is where
  physics should take over, not a clamped step.
- Pushes while walking: the balance only runs on the standing side of the
  blend.
- A sideways lunge looks too deep: a crossover step, or the trailing foot
  rolling onto its inner edge, are the human alternatives. The leg-length
  floor of ~10 cm stays either way.

## Related

- [Push recovery is Winter's pendulum](./push-recovery-is-winters-pendulum.md) — prerequisite: the sway, COP law and support this steps from.
- [A speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md) — deeper: why the gallery passes `planted` to the foot IK.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — applies: the join's travelled distance becomes root motion the locks must be given.
- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — context: why a wide stance leaves the legs no reach to spare.
