---
title: Fifty game camera mistakes (John Nesky, GDC 2014) — digest
description: "Digest of Journey camera designer John Nesky's 50-item list of third-person camera mistakes, grouped into architecture, collision, framing, auto-behaviour, input, comfort and process, with what each means for a rig. Read before designing camera DOFs, collision response, auto-recentre or input curves."
type: research
status: current
tags:
  - camera
  - prior-art
  - correctness
  - verification
updated: 2026-10-10
sources:
  - "GDC Vault, John Nesky, '50 Game Camera Mistakes' (2014): https://gdcvault.com/play/1020460/50-Camera"
  - "Video: https://www.youtube.com/watch?v=C7307qRmlMI"
  - "Full title list: https://shermanrose.uk/knowledge/programming/games/50-game-camera-mistakes/"
  - "Explanations of items 1-16: https://matellion.blogspot.com/2016/08/50-common-game-camera-mistakes-and-how.html"
  - "Items 17-50 explanations: AI summary of the video, https://videohighlight.com/v/C7307qRmlMI (verify against the video)"
aliases:
  - 50 camera mistakes
  - Journey camera
  - whisker raycasts
---

# Fifty game camera mistakes (John Nesky, GDC 2014) — digest

John Nesky built the camera for *Journey*. His list is the most complete
practitioner checklist for third-person cameras. Three rules matter most.
First, **store the camera as orbit degrees of freedom around the avatar**, not
as a world pose. Second, **player intent beats automation**. Third,
**derive distance and FOV from pitch "like gears"** instead of animating them
separately. The titles of all 50 items come from sources. The explanations of
items 17–50 come from a video summary, so verify them against the talk before
relying on fine detail.

## Architecture

1. Don't use a dynamic camera when a fixed or first-person camera would do.
2. Design levels and camera behaviour together.
3. Don't store camera state as world coordinates or quaternions. Store an
   orbit around the avatar with **7 DOF**: yaw, pitch, roll, distance, lateral
   offset, vertical offset, FOV.
4. Don't use a default distance that breaks line of sight in the level's
   passages. Shorten it to fit them.

## Collision and occlusion

5. Detect side occluders early with **whisker raycasts**. Reuse the previous
   frame's results and swing away before the main ray is blocked.
6. **"When avoidance and intent conflict, intent wins."**
7. Never let the player push the camera into geometry. The fix is to shorten
   the line of sight, not to block the input.
8. Don't let independent forces fight. Give each DOF its own prioritised
   forces.
9. Tag small occluders (columns, poles) that may break line of sight, and
   have the whiskers ignore them.
10. Use simple sphere collision, so thin columns don't snag the camera.
11. Check the hit normal so a hill isn't treated as a wall. The camera should
    **rise over hills**.
12. When the occluder is behind the camera, **pull forward** instead of
    swinging sideways.
13. Keep the near plane out of the avatar. Leave a gap between the model and
    the collision radius, or fade the avatar.

## Framing

14. Vary distance with pitch: closer at a worm's-eye angle, a little farther
    looking down.
15. Use a wider FOV at low angles so the sky shows.
16. **Derive distance and FOV instantly from pitch, so they shift together
    like gears.**
17. Cut when the avatar passes through opaque objects.
18. **A cut must not remap directional controls.**
19. Preserve the player's sense of direction, for example with landmarks.
20. The 180° rule matters less in games than in film.
21. Don't frame only the avatar; show where they are going.

## Auto-behaviour

22. Don't make the player steer the camera all the time.
23. **Auto-yaw toward the movement direction while running.**
24. Make distances easy to judge.
25. **Raycast down near cliffs and pitch down.**
26. Keep the camera level on slopes.
27. For rule-of-thirds framing, slide the camera sideways; don't rotate it.
28. Use separate ground and air logic. Don't track jumps vertically.
29. Mix procedural and authored behaviour.
30. Don't let players get themselves lost.
31. Don't over-rotate to look at nearby targets.
32. Don't translate the camera to look at distant targets.
33. Keep the avatar's body from occluding the target. Place hints slightly
    off-centre.
34. Don't give control and then take it away.
35. **Wait before an auto-behaviour resumes after the player stops turning.**
36. Let experts explore. Hints must be resistible.

## Input

37. Offer inverted axes.
38. Ignore accidental input with a deadzone.
39. Use an **S-curve** stick response, not a linear one.
40. Allow limited pivot drift, but keep the avatar on screen.

## Comfort

41. Avoid a too-narrow FOV.
42. Avoid rapid FOV changes.
43. Avoid heavy shake.
44. Don't bob the camera with the walk cycle.
45. Don't translate or rotate the camera on jumps, and offer toggles for
    such effects.
46. Avoid rapid transitions.
47. **Slow the pitch down as it approaches its limit.**

## Process

48. Don't treat VR as the main camera.
49. Playtest with a broad range of players, including children.
50. **Don't write a general camera "constraint solver".** Solve each DOF with
    simple, owned rules.

## Relevance to migera

The design adopts these directly:
- orbit scalars as state (3)
- pitch curves for distance, height and FOV (14–16)
- one owner per DOF (8)
- intent over avoidance, with recentring only after a delay (6, 35)
- hill normal handling and pull-forward on occlusion from behind (11, 12)
- avatar fade (13)
- a latched control yaw so cuts can't remap input (18)
- a vertical deadband for jumps (28, 45)
- an S-curve with a deadzone (38, 39)
- soft pitch limits (47)

Item 50 is why the design is a fixed pipeline of small owned stages rather
than a solver.

## Related

- [Third-person camera design](./third-person-camera-design.md) — applies: where each item above lands in the pipeline.
- [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md) — deeper: whiskers, the hill normal, avatar fade.
- [Gothic's ZenGin camera](./gothic-zengin-camera-modes-and-collision.md) — example: a shipped orbit-scalar rig that predates this list.
- [Shipped action-game camera behaviours](./shipped-action-game-camera-behaviours.md) — example: games that hit or missed these items.
