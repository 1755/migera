---
title: A segment angle measured from standing is not an angle from vertical
description: "Fukuchi's thighs and shanks were extracted as change from standing and replayed as angles from vertical; standing thighs lean back 5.6°, so the rig ran legs forward, hips 43-50 mm low at landing, blamed on soft tissue. Read before extracting or replaying recorded segment angles."
type: lesson
status: current
tags:
  - biomechanics
  - locomotion
  - correctness
  - verification
updated: 2026-10-05
verified: 2026-10-05
code:
  - tools/extract_running_strides.py
  - src/character/anim/run.rs
sources:
  - "Fukuchi, Fukuchi & Duarte 2017, PeerJ 5:e3298, data doi:10.6084/m9.figshare.4543435: static trials with ASIS/PSIS, epicondyle and malleolus markers"
  - "Harrington, Zavatsky, Lawson, Yuan & Theologis 2007, J Biomech 40:595-602 (hip joint centre regression)"
  - "test run::tests::a_running_body_flies_at_g_and_its_legs_give_little_for_it (fails on the old curves: a level take-off, 19.8° of knee)"
aliases:
  - standing zero
  - static trial zero
  - cluster zero
  - bone zero
  - standing_bones
  - hip_centre
---

# A segment angle measured from standing is not an angle from vertical

A segment angle zeroed on a subject's standing trial is a change from
standing. The replay reads it as an angle from vertical. The two differ by
how the bone leans when the subject stands, here 5.6° at the thigh. That
difference put the rig's hips 4-5 cm wrong at the two ends of a running
stance, which was blamed for a session on skin moving under the markers.

## What happened

- **The extraction:** `tools/extract_running_strides.py` took each thigh and
  shank attitude from a marker cluster, less the same cluster's attitude in
  the runner's static trial. That subtraction is needed, because a cluster
  strapped to the side of a thigh does not lie along the bone. But it
  zeroes the angle on the standing bone, not on vertical.
- **The replay:** `run::RunCycle` sets the rig's thigh to that number as an
  angle from vertical, as the walk does with Winter's absolute angles.
- **The standing bones lean:** measured along the bones in the 28 static
  trials (hip centre by Harrington's regression, knee centre between the
  epicondyles, ankle centre between the malleoli), the thigh leans back
  5.6° (SD 2.7) and the shank 3.1° (SD 2.8). The hip stands about 6 cm
  ahead of the ankle, as a standing body's line of gravity puts it.
- **The cost:** the rig ran with every thigh 5.6° and every shank 3.1° more
  forward than the runners'.
  - At landing that tilts the leg further, hips about 21 mm lower.
  - At toe-off it straightens a leg reaching back, hips about 29 mm higher.
  - Replayed, the hips came 43-50 mm lower at contact than at toe-off. The
    runners' joint centres say +3 mm (pelvis markers +12).
- **The wrong diagnosis:** imposing the recorded pelvis "needed 4 % more
  leg than the rig has". Soft tissue under the thigh clusters looked like
  the cause, and a flight plan was built to live with it: a level take-off,
  landings at 0.8-1.2 m/s, a bob 20 % short, up to 17° of extra knee.

## Why it matters

- **A change is not an attitude.** Subtracting a static trial is the
  standard way to cancel marker placement. It also cancels the posture
  itself, which a replay driven by angles from vertical needs.
- **Small angles, large heights.** 5° on a 0.43 m segment at 20-30° from
  vertical moves the hip 15-20 mm, opposite ways at the two ends of
  stance. A gait's heights are a sensitive check on its angles.
- **The angles still looked right.** Joint centres put the thigh and shank
  within 1-2° of the clusters' changes all stride. Only the constant
  offset was wrong, and a constant offset hides in every comparison of
  shapes.

## How to apply

- **Know what zero the replay assumes.** Drive a rig with angles from
  vertical only when the data is from vertical. Data zeroed on a static
  trial needs the static bone attitude added back, from bony landmarks.
- **Check heights at both ends of stance, from the joint centres.** Put
  hip and ankle centres from rigid marker sets (pelvis, shoe). Compare the
  hip's height at contact and toe-off with what the replayed angles give,
  before blaming skin.
- **The joint centres are not perfect either.** Their hip-ankle distance
  ran 1-1.5 % longer than the standing leg at both ends of stance, the same
  at each end. So the gap is no help for an end-to-end height difference.

## Evidence

- **Re-extracted** (`standing_bones`, `hip_centre`): the thigh curve moved
  5.7° back and the knee 2.5° straighter. Replayed on `puppet_base`, the
  hips at toe-off are 9 mm over contact, against the recording's 11 at
  3.5 m/s.
- **The flight plan then gave the recording's flight**
  (`run::FlightPlan`; see
  [running](./running-replays-measured-strides-at-their-froude-number.md)):
  - leaving rising at 0.22-0.42 m/s and landing at 0.42-0.75 m/s;
  - a bob 71-97 mm against the recorded 78-97;
  - knee off the recording by 7.5-10° up to 3.5 m/s, 16-17° above.

  On the old curves, the same plan takes off level at 2.2 m/s and needs
  19.8° of knee, and the flight test fails.

## Related

- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — applies: the run this re-extracted, and the flight plan on top.
- [Replay a recorded gait by segment attitudes](./replay-a-recorded-gait-by-segment-attitudes.md) — prerequisite: why the replay drives the thigh from vertical; Winter's walk angles already are.
- [The recorded pelvis path and leg angles conflict](./recorded-pelvis-path-and-leg-angles-conflict.md) — contrast: the walk's conflict is the rig's proportions, its angles being from vertical already.
