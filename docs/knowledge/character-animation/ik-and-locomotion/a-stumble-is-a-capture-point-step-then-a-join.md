---
title: A stumble is a capture-point step, then a join the body's momentum carries
description: "balance::Balance steps to the predicted capture point (COP at the stance foot's nearest point, ≤ 0.6 m), never re-using the leg just stepped, crossing over sideways; the trailing foot joins once the capture point is inside the stepped foot; a rear foot rolls onto its toes. Read before changing stepping in balance.rs."
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
  - "live BRP, character_gallery --push-schedule 3:0.6:0,8:0:0.7,13:-0.8:0 and 3:0:0.8, puppet_base and character.glb"
  - "Maki & McIlroy (1997), The role of limb movements in maintaining upright stance: the 'change-in-support' strategy, Phys Ther 77(5)"
  - "Postural reactions to external mediolateral perturbations: a review, Applied Sciences 13(3):1696 (2023), https://www.mdpi.com/2076-3417/13/3/1696 — loaded side step, unloaded crossover, unloaded medial step; young adults favour the loaded side step"
  - "Untangling biomechanical differences in perturbation-induced stepping strategies for lateral balance stability in older individuals, https://pmc.ncbi.nlm.nih.gov/articles/PMC7778461/"
  - "probe balance::tests::probe_catch_table (ignored)"
aliases:
  - crossover step
  - CROSSOVER_AHEAD
  - MAX_STEP
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
  `STEP_SECONDS` = 0.3. `p` is the stance foot's point **nearest the
  capture point**, where `step`'s law pins the COP, not its middle; the
  middle asked for steps ~2× too long (0.48 m where 0.26 m caught a
  0.5 m/s push). Travel is clamped to `MAX_STEP` = 0.6 m, about 0.65 of
  `puppet_base`'s leg length. The sideways component is kept only for a
  sideways push.
- **Judged and planned on the whole push**, the part not yet delivered
  too. Judged on what had landed, a frame's delay (50 ms live) put a
  0.7 m/s side push's step 17% further out, past `MAX_STEP`.
- **Which leg.** Straight ahead or back, the unloaded one. **Never the
  foot that just stepped**: stepping it again lifts it before it takes
  the weight, and a sideways push walked the stepping foot out 0.4 m at a
  time with the COM running after it. **Sideways, whichever leg needs the
  shorter step**, and in this model that is usually the far leg
  **crossing over** in front. The near leg's side step stands on the far
  foot, whose pressure drives the body on. The crossover stands on the near
  foot, whose pressure brakes it. A 0.8 m/s side-step lunge sank the
  pelvis 266 mm; the crossover sinks it 91 mm. People use both: the
  literature names the loaded side step, the unloaded crossover and the
  unloaded medial step (Maki & McIlroy's change-in-support strategy).
  **Young adults mostly take the loaded side step**, which starts faster;
  older adults cross over more. So the choice here is the model's
  dynamics, not the typical young adult's. A crossover lands `CROSSOVER_AHEAD` (0.12 m)
  forward and bows out that far mid-swing, so the legs pass rather than
  through each other. The join uncrosses them, because the joining foot
  goes to the stood width beside the stepped one.
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

Every catch is the step's, not the validity bound's: nothing lost to the
bound (`Balance::lost`), settled within 6 s. Caught on `puppet_base`:
forward to 1.0 m/s, sideways to 1.0 (a crossover then a join), back to
1.2 (`a_push_past_a_catchable_step_falls`). Harder pushes fall; see
[a fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md).

Near-full-reach steps, headless (`a_stumble_steps_cleanly_on_the_real_rig`):

| push | pelvis sank | note |
|---|---|---|
| 1.0 m/s forward | 52 mm | rear heel rises 84 mm onto the toes |
| 0.8 m/s left | 91 mm | crossover; the side-step lunge was 266 mm |
| 1.2 m/s back | 79 mm | rear heel rises 48 mm |

Live, 0.8 m/s left on both rigs: the right foot crosses over (0.52 m),
the left joins (0.51 m), planted balls slide ≤ 1.7 mm. The planned steps
are identical under frame times cycling 5–50 ms.

**A hard landing remains.** At a long step's touchdown the pelvis height
follows the leg's reach through a V: its per-frame move changes by up to
12.8 mm (≈ 5 g at 60 Hz). The pops fixed earlier were 60–132 mm; this is a
landing, but a real one decelerates over 50–100 ms of knee flexion.

Until 2026-09-30 the sideways catch was the clamp's: the 8° bound, from
the stance foot alone, held the COM with its velocity zeroed, and every
side step was posed over a body the clamp had stopped.

## Revisit when

- Pushes while walking: the balance only runs on the standing side of the
  blend.
- The touchdown V: plan the pelvis height across the swing (smooth descent
  to the landing's need) instead of following the reach frame by frame.
- Young-adult sideways stepping: a loaded side step would need the
  side-step lunge made shallow (a narrower join, or two shorter steps).

## Related

- [Push recovery is Winter's pendulum](./push-recovery-is-winters-pendulum.md) — prerequisite: the sway, COP law and support this steps from.
- [A fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md) — deeper: what happens when no step catches the push.
- [A speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md) — deeper: why the gallery passes `planted` to the foot IK.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — applies: the join's travelled distance becomes root motion the locks must be given.
- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — context: why a wide stance leaves the legs no reach to spare.
