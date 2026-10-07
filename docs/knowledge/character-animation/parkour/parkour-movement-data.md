---
title: Parkour movement data
description: "Measured human data for parkour moves: ledge-catch proxies, hanging, pull-up ranges, drop landings (precision, squat, roll; 0.75-2.7 m), wall climbs and runs, bar swings, ropes, beams, crawling, squeezing; gaps and unverified numbers flagged. Read before timing or shaping a parkour move."
type: research
status: current
tags:
  - biomechanics
  - locomotion
  - character-animation
updated: 2026-10-07
sources:
  - "Gosine, Komisar and Novak 2024, Human Factors 66:56 (PMC10756024)"
  - "Hiley and Yeadon 2003, J Biomech 36:313 (PubMed 12594979)"
  - "Yeadon and Hiley 2000, Hum Mov Sci 19:153, doi:10.1016/S0167-9457(00)00008-7"
  - "Exel et al. 2026, Eur J Sport Sci (PMC13240036)"
  - "Walker et al. 2023, Int J Exerc Sci (PMC10824315)"
  - "Youdas et al. 2010, J Strength Cond Res 24:3404 (PubMed 21068680)"
  - "Dinunzio et al. 2019, Sports Biomech 18:622 (PubMed 29768093)"
  - "Askari Hosseini and Wolf 2023, Front Sports Act Living (PMC10766694)"
  - "Puddle and Maulder 2013, J Sports Sci Med 12:122"
  - "Dai et al. 2020, J Hum Kinet 72:15 (PMC7126243)"
  - "Standing and Maulder 2015, J Sports Sci Med 14:723 (PubMed 26664268)"
  - "Maldonado et al. 2020, Comput Methods Biomech Biomed Eng, doi:10.1080/10255842.2020.1714932"
  - "Croft, Schroeder and Bertram 2019, J Exp Biol 222:jeb190983"
  - "Lawson 2015, undergraduate honours thesis, Univ. Colorado (not peer reviewed)"
  - "Dhahbi et al. 2015, Int J Sports Physiol Perform 10:509 (PubMed 25392939)"
  - "da Silva Costa et al. 2022, Sci Rep 12:7179 (PMC9046185)"
  - "Lambrich et al. 2025, Front Sports Act Living (PMC12745434)"
  - "Ma et al. 2017, Sensors 17:692 (PMC5421652)"
  - "Babič et al. 2001, Gait Posture 14:56 (PubMed 11378425)"
  - "Warren and Whang 1987, J Exp Psychol Hum Percept Perform 13:371"
aliases:
  - parkour biomechanics
  - landing from height data
  - wall run data
  - ledge hang data
---

# Parkour movement data

What measured human movement says about each parkour move, for timing and
shaping them in `character::anim`. Gathered 2026-10-07 from full texts where
they could be read (Europe PMC, PubMed, PDFs). Two flags:
- **[summary]**: seen only in a search engine's summary, not checked against
  the paper. Check before using it as a constant.
- **[not peer reviewed]**: from a thesis, a course page or guidance.

Four moves have **no direct data**: the ledge catch, the braced hang, the
hanging traverse (shimmy), and the climb-up's phase timing. Proxies are
given for each.

## Catching a ledge or bar

- **No data** on how far the body drops after the hands contact, or the arm
  angle at the catch.
- **Proxy, a reach to grab a rail after losing balance** (young adults;
  Gosine et al. 2024):
  - the hand starts moving about 200 ms after the perturbation;
  - the reach takes 185 ms, contact at 385 ms;
  - peak wrist speed 2.29 m/s, 26 % of the reach after the peak (ballistic).
- **Proxy, the high bar**: a release-and-regrasp skill loads the shoulder
  at 4.5 ± 1.7 N·m/kg, the hip 2.3 [summary]. A release still works within
  a 73-157 ms window (Hiley and Yeadon 2003).

## Hanging

- **Dead-hang endurance**: 52 ± 16 s on a 20 mm edge, half-crimp, elbows
  straight (climbers; Exel et al. 2026). On a bar, men 64 ± 35 s, women
  48 ± 29 s [summary, study not identified].
- **Braced hang (feet on the wall): no data.**
- **Free-hang swing: no measurement.** As a rigid compound pendulum (grip to
  COM about 1.25 m, radius of gyration about 0.5 m, a 1.75 m person) the
  period is `2π√((d² + k²)/(g·d))` ≈ 2.4 s (derived). No damping data:
  hangers pump or damp actively at the hips and shoulders, as gymnasts are
  modelled (Yeadon and Hiley 2000).

## Climbing up from a hang

- **No phase timings** for a ledge climb-up or a muscle-up. The muscle-up's
  phases are only named: a pull (shoulder adduction and extension, elbow
  flexion), a transition, a press to straight elbows (Walker et al. 2023).
- **Proxy, the pull-up**: the elbow moves through 93 ± 15° (pull-up) and
  101 ± 15° (chin-up) (Youdas et al. 2010). A course page gives about
  135-140°, at its most bent 24° (included) at 36 % of the movement [not
  peer reviewed].
- **A kipping pull-up** adds 49° of hip and 57° of knee flexion to a strict
  one (Dinunzio et al. 2019).

## Shimmying along a ledge

- **No data** for a hanging traverse. Speed climbers move a hand 2.5-2.8
  times a second and a foot 2.5-2.9 (Askari Hosseini and Wolf 2023, a
  review): an upper bound, far faster than a shimmy.

## Landings from height

Drops from 0.75 m (Puddle and Maulder 2013, 10 males):

| Landing | Peak vertical force | Time to peak | Loading rate |
|---|---|---|---|
| Parkour precision | 3.2 ± 0.5 BW | 77 ms | 83 BW/s |
| Parkour roll | 2.9 ± 0.2 BW | 80 ms | 64 BW/s |
| Traditional | 5.2 ± 1.2 BW | 44 ms | 154 BW/s |

Drops from 0.9, 1.8 and 2.7 m (Dai et al. 2020, 20 practitioners):
- vertical speed at contact 3.0, 4.9 and 6.3 m/s [not free fall's: from
  0.9 m that is 4.2 m/s, and these do not scale as the drops' square roots;
  key a landing by the drop, not by them];
- landing duration, squat 377, 335, 290 ms; roll 380, 364, 320 ms; stiff
  169 ms (0.9 m) and 224 ms (1.8 m);
- the knees 20-29° flexed at contact, every technique;
- most knee flexion: squat 116°, 126°, 134°; forward 135-140°; roll
  115-121°; stiff 77-94°; the hips at most 132-152° in the squat;
- the roll keeps the forward speed and slows the body over longer.

Choosing: guidance is to roll above about standing height [not peer
reviewed]. Traceurs' habitual landings from 25 % and 50 % of body height
had 40-49 % lower peak force and 66-69 % longer time to it than others',
on the forefoot in 93 % of trials (Standing and Maulder 2015).

## Vaults and mantles

- **Kong vault from a stand** (11 traceurs; Maldonado et al. 2020): take-off,
  flight, landing; early in take-off the feet carry 1.2 BW, the hands 0.3
  [summary].
- **Speed and lazy vaults: no peer-reviewed numbers.**

## Walls

- **Wall climb onto a 3 m wall** (6 athletes; Croft, Schroeder and Bertram
  2019): approach 4.70 ± 0.40 m/s (the model's best 4.49); the last foot on
  the ground 1.17 m from the wall; the first foot on the wall at 1.01 m
  (best 0.88).
- **One foot's wall run up** (Lawson 2015, not peer reviewed): the COM leaves
  the ground at 0.96 m rising 2.93 m/s; meets the wall at 0.06 s (1.12 m,
  2.35 m/s); leaves it at 0.43 s (1.96 m, 1.55 m/s), about 0.37 s on the
  wall; tops at 2.18 m at 0.58 s. The wall adds 0.73 m (34 %), about 1.5×
  the height without it. A second contact (a hand or a foot) goes higher;
  the hand swipes from shoulder to hip height and pushes.
- **Wall jump (tic-tac): no data.** Use one wall contact of about 0.37 s
  while still rising.

## Bars, poles and ropes

- **High bar**: a giant circle about 2.2-2.3 s [summary]; a double-layout
  release window 88-157 ms scooped (95 % of 2000 Olympians), 73-84 ms
  traditional (Hiley and Yeadon 2003); hips and shoulders flex just after
  the bottom of the swing and extend just before the top (Yeadon and Hiley
  2000).
- **Rope**: a 5 m climb in 15.6 ± 3.5 s, about 3.1 s a metre (21 commandos)
  [summary]; the test is reliable to 0.51 s (Dhahbi et al. 2015).
- **Pole climbing: no data.**

## Beams

- **Older adults on beams 10, 8, 6 cm wide**: 0.82, 0.77, 0.69 m/s; arms
  crossed 0.74 m/s against 0.78 free (da Silva Costa et al. 2022).
- **Young adults backward on 4.5 and 3.0 cm beams** (Lambrich et al. 2025):
  shoulder abduction ranged over 92 ± 37° and 118 ± 61° with the arms free
  (16° and 21° held); elbows over about 60°; the trunk's sideways bend over
  61° then 100° on the narrower beam.

## Crawling and squeezing

- **Hands and knees** (30 adults; Ma et al. 2017): 0.28-0.69 m/s tried, only
  21 of 30 past 0.56 m/s; slow, trot-like (diagonal pairs) or four-beat;
  faster, trot-like or pace-like (same-side pairs). Below 0.6 m/s no
  unstable two-point support is needed (Babič et al. 2001).
- **Squeezing**: people turn their shoulders when a gap is narrower than
  about 1.3 shoulder widths (critical ratio 1.28-1.31, at any walking
  speed; Warren and Whang 1987).
- **A sideways shuffle while squeezing: no data.**

## Relevance to migera

- Landings (step 4 of the design) have the firmest data: contact speed,
  landing durations and joint ranges per technique and height.
- The ledge moves (steps 1-3) rest on proxies and the rig: the catch's
  absorption, the braced hang and the climb-up timing have to be set from
  the body's own limits and verified to look right, the gap recorded.
- The free hang's swing is a compound pendulum (≈ 2.4 s) whose damping is
  the hanger's own effort, so it can be chosen.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — applies: the design whose steps take their timings from here.
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — context: the jump's own landing data, which step 4 extends past standing heights.
