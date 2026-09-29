---
title: 4.0 Scope of anthropometry in movement biomechanics
description: Frames anthropometry for movement analysis — beyond lengths and volumes it needs masses, COM locations, moments of inertia, joint centers and muscle geometry — and holds the Fig. 4.1 segment-length-per-height note. Read when deciding which body-segment parameters a rig or ragdoll needs.
type: index
status: current
tags:
  - biomechanics
  - anthropometry
  - rig
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §4.0, pp. 82–83 (PDF pp. 95–96)"
---

# 4.0 Scope of anthropometry in movement biomechanics

> **Source:** Winter (2009) §4.0, pp. 82–83 ·
> [open PDF at p. 82](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=95) ·
> Up: [Chapter 4 — Anthropometry](../INDEX.md)

Anthropometry measures the human body. Classical work served evolutionary
studies and later equipment design (cockpits, workspaces, armour), which
needs only linear, area and volume measures. Movement analysis additionally
needs **kinetic** measures — segment masses, moments of inertia and where
they sit — plus joint centers of rotation, muscle origins/insertions, tendon
pull angles, and muscle lengths and cross-sections. This short opening
section sets that scope and gives the one purely geometric dataset, segment
lengths as fractions of height.

## Key facts
- Every joint height and segment length can be defaulted from stature H alone: thigh ≈ shank ≈ 0.245 H, upper arm 0.186 H, forearm 0.146 H, hip joint at 0.530 H ([4.0.1](./4.0.1-segment-dimensions.md)).
- Directly measured lengths always beat the table ([4.0.1](./4.0.1-segment-dimensions.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [4.0.1 Segment dimensions](./4.0.1-segment-dimensions.md) | Fig. 4.1 transcribed: landmark heights, segment lengths and widths as fractions of H | sizing or sanity-checking a rig, ragdoll capsule, or stride from height |

## Relevance to migera

A proportion oracle for `puppet_base.gltf` and any scaled/procedural rig;
the inertial data this scope promises lives in
[4.1](../4.1-density-mass-inertial-properties/INDEX.md), which is what the
ragdoll actually consumes.

## Where to read in the book
- pp. 82–83 (PDF 95–96): scope text; **Fig. 4.1** on p. 83 (render it — the numbers do not survive text extraction).
