---
title: Appendix B — Units and definitions related to biomechanical and EMG measurements
description: The SI base and supplementary units and the book's definitions of the derived quantities (force, moment, momentum, work, energy, power, strain, electrical units), plus prefix and notation rules. Read when unsure of a unit or a definition used anywhere in Winter.
type: reference
status: current
tags:
  - biomechanics
  - physics
  - math
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Appendix B, pp. 361–366 (PDF pp. 374–379)"
aliases:
  - SI units
---

# Appendix B — Units and definitions related to biomechanical and EMG measurements

> **Source:** Winter (2009) Appendix B, pp. 361–366 ·
> [open PDF at p. 361](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=374) ·
> Up: [Appendices](./INDEX.md)

The book uses only SI units. The appendix lists the base units (Table B.1),
defines about 30 derived quantities (Table B.2), and gives rules for
prefixes and compound-unit notation. Nothing here is new physics. Its value
is that each definition is stated in the planar form the book uses, for
example angular momentum as I·ω about the centroid.

## Key ideas

**Table B.1: base SI units.** There are seven base units plus two
supplementary ones. Every other unit is a product or quotient of these.

| Quantity | Unit |
|---|---|
| length *l* | metre, m |
| mass *m* | kilogram, kg |
| time *t* | second, s |
| electric current *I* | ampere, A |
| temperature *T* | kelvin, K |
| amount of substance *n* | mole, mol |
| luminous intensity | candela, cd |
| plane angle θ, φ (supplementary) | radian, rad |
| solid angle (supplementary) | steradian, sr |

**Table B.2: derived units and definitions.** Condensed below.

| Quantity | Symbol, unit | Definition as given |
|---|---|---|
| Velocity, acceleration | v m·s⁻¹, a m·s⁻² | Time rates of change of position and of velocity |
| Gravity | g m·s⁻² | Free-fall acceleration in vacuum; **g = 9.80665 m·s⁻²** at sea level |
| Angular velocity, acceleration, displacement | ω rad·s⁻¹, α rad·s⁻², θ rad | Rate of change of a line segment's orientation in a plane; rate of change of ω; plane angle between the initial and final orientations |
| Period, frequency | T s, f Hz | Duration of one cycle (or of any phase); repetitions per second, 1 Hz = 1 s⁻¹ |
| Density, specific gravity | ρ kg·m⁻³, d (none) | Mass per volume; density relative to water at 4 °C |
| Force | F, N | Effect of one body on another that accelerates it relative to an inertial frame; 1 N = 1 kg·m·s⁻² |
| Weight | G, N | G = m·g |
| Mass moment of inertia | I kg·m² | Σ (mass element × squared distance to the axis); resistance to angular acceleration |
| Linear momentum | p kg·m·s⁻¹ | m × velocity of the mass centre |
| Angular momentum | L kg·m²·s⁻¹ | Moment of linear momentum about a point; **planar: I about the centroid × ω** |
| Moment of force | M, N·m | Force × perpendicular distance from its line of action to the point |
| Pressure, normal and shear stress | p, Pa | Force per unit area; 1 Pa = 1 N·m⁻² |
| Linear and shear strain | ε, γ (none) | Fractional change in length; change in angle of an initially perpendicular line |
| Young's and shear modulus | E, G, Pa | Stress/strain over the initial linear part of the curve |
| Work | W, J | Energy change from a force acting through a displacement; 1 J = 1 N·m = 1 W·s; the time integral of power |
| Mechanical energy | E, J | Capacity to do work: potential + kinetic |
| Potential energy | V, J | Gravitational **mgh**; elastic spring **k·e²/2** |
| Kinetic energy | T, J | Translational **½mv²**; rotational **½Iω²** |
| Power | P, W | Rate of doing work: **P = F·V** for a force, **P = M·ω** for a moment |
| Coefficient of friction | μ (none) | Tangential / normal contact force |
| Coefficient of viscosity | η N·s·m⁻² | Shear stress / rate of deformation |
| Charge, voltage, resistance, capacitance | C, V, Ω, F | e = 1.602×10⁻¹⁹ C; 1 V = 1 J·C⁻¹; 1 Ω = 1 V·A⁻¹; 1 F = 1 C·V⁻¹ (used by the EMG chapter) |

**Notes to the tables.**
- Prefixes: mega 10⁶ (M), kilo 10³ (k), centi 10⁻² (c), milli 10⁻³ (m),
  micro 10⁻⁶ (μ).
- Write a product unit as `N·m` or `N m`, never `Nm`. Write a quotient as
  `kg/m³` or `kg·m⁻³`.
- Unit symbols are never pluralised: "kgs" could be read as kg·s.

## Where to read in the book

- p. 361 (PDF 374): Table B.1 and the start of B.2.
- pp. 362–365 (PDF 375–378): Table B.2, mechanical, then energy and power, then electrical quantities.
- p. 366 (PDF 379): notes on prefixes and notation.

## Relevance to migera

Low but handy. Bevy and avian are unit-agnostic, and migera treats them as
SI (metres, kilograms, seconds, radians). The definitions that matter most
are P = M·ω (joint power from a PD torque and joint angular velocity) and
the kinetic-energy terms ½mv² + ½Iω², which are what an energy-drift check
on the ragdoll would sum. Angles are in radians throughout. Appendix A
prints joint angles in degrees but ω and α in rad/s and rad/s².

## Related

- [Appendix A — walking trial data](./a-walking-trial-kinematic-kinetic-energy-data.md) — applies: every column there uses these units.
- [6.0.1 Mechanical energy and work](../ch06-work-energy-and-power/6.0-energy-and-work-of-muscles/6.0.1-mechanical-energy-work.md) — deeper: how the book uses the energy and work definitions.
- [avian: apply angular acceleration, not torque](../../character-animation/ragdoll-and-physics/avian-apply-angular-acceleration-not-torque.md) — contrast: where a torque (N·m) versus an angular acceleration (rad·s⁻²) unit mix-up caused a real bug.
