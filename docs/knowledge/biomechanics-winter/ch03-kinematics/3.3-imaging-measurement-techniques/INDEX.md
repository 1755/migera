---
title: 3.3 Imaging measurement techniques
description: "Camera-based motion capture: lens optics, f-stop, 16-mm film and manual digitizing (1–1.5 mm rms noise), video (60/50 Hz, blur, scan skew, marker centroids ~1 mm), optoelectric IREDs (0.03 mm), and trade-offs. Read for the origin of the noise the book filters; otherwise low relevance for migera."
type: index
status: current
tags:
  - biomechanics
  - signal-processing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §3.3, pp. 53–64 (PDF pp. 66–77)"
---

# 3.3 Imaging measurement techniques

> **Source:** Winter (2009) §3.3, pp. 53–64 ·
> [open PDF at p. 53](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=66) ·
> Up: [Chapter 3 — Kinematics](../INDEX.md)

Only an imaging system can capture a whole complex movement, and describing a
dynamic activity means sampling images at regular intervals over time. The
section's introduction limits itself to three kinds — movie camera,
television and optoelectric — all of which use a lens, so it opens with
optics (§3.3.1–3.3.2), then covers each system (§3.3.3–3.3.5) and closes with
trade-offs (§3.3.6–3.3.7). The practical output for the rest of the book is a
noise level: about 1–1.5 mm rms at 4 m for digitized film, ~1 mm for good
video, 0.015 mm for OPTOTRAK.

## Key facts
- For distant subjects the image lies at the focal length ($f \approx v$), so image size scales by triangulation ([3.3.1](./3.3.1-review-basic-lens-optics.md)).
- Each f-stop step halves or doubles the light; wide apertures narrow the in-focus range ([3.3.2](./3.3.2-f-stop-and-field-of-focus.md)).
- Frame rate must satisfy the sampling theorem; manual film digitizing noise is 1–1.5 mm rms at 4 m ([3.3.3](./3.3.3-cinematography.md)).
- Video scans top-to-bottom in ~15 ms, skewing marker times by ~10 ms unless strobed or shuttered ([3.3.4](./3.3.4-television.md)).
- Active IRED markers need no labelling and reach 0.03 mm precision at 4 m ([3.3.5](./3.3.5-optoelectric-techniques.md)).
- Imaging's key advantage is an absolute spatial reference frame ([3.3.6](./3.3.6-optical-systems-pros-and-cons.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [3.3.1 Review of basic lens optics](./3.3.1-review-basic-lens-optics.md) | Thin-lens law, f ≈ v for distant subjects, zoom rationale | only for camera-geometry background |
| [3.3.2 f-stop setting and field of focus](./3.3.2-f-stop-and-field-of-focus.md) | Aperture vs light vs depth of field | only for camera setup background |
| [3.3.3 Cinematography](./3.3.3-cinematography.md) | 16-mm film, shutter exposure, sampling-theorem frame rate, manual digitizing noise | when you need the book's raw-data noise figures |
| [3.3.4 Television](./3.3.4-television.md) | Field rates, blur/skew fixes, marker-centroid precision history | for mocap hardware history |
| [3.3.5 Optoelectric techniques](./3.3.5-optoelectric-techniques.md) | OPTOTRAK plane-intersection geometry, precision | for mocap hardware background |
| [3.3.6 Advantages and disadvantages of optical systems](./3.3.6-optical-systems-pros-and-cons.md) | Pros/cons across cine, TV, IRED | when comparing capture methods |
| [3.3.7 Summary of various kinematic systems](./3.3.7-kinematic-systems-summary.md) | Which lab picks which system | when comparing capture methods |

## Relevance to migera
Low. migera synthesizes motion rather than capturing it. What carries over:
the sampling-theorem constraint on any fixed-rate motion signal, and the
realistic noise magnitudes that make §3.4's differentiation analysis
quantitative.

## Where to read in the book
- pp. 53–55 (PDF 66–68): intro, optics, f-stop.
- pp. 55–58 (PDF 68–71): film and digitizing.
- pp. 58–61 (PDF 71–74): television.
- pp. 61–64 (PDF 74–77): optoelectric, pros/cons, summary.
