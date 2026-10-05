"""Extract mean running strides at three speeds into assets/anim/reference/.

Source: R. K. Fukuchi, C. A. Fukuchi, M. Duarte, "A public dataset of running
biomechanics and the effects of running speed on lower extremity kinematics
and kinetics", PeerJ 5:e3298 (2017), doi:10.7717/peerj.3298; data at
doi:10.6084/m9.figshare.4543435 (CC BY 4.0). Recreational and competitive
runners on an instrumented treadmill at 2.5, 3.5 and 4.5 m/s, 30 s each,
markers at 150 Hz and the treadmill's vertical force at 300 Hz.

The raw files are ~350 MB, so they are not kept in the repo: this script
downloads what it needs into a cache directory (default
`target/running-data`, gitignored) and writes the averaged CSV, which is
committed. Re-run with:

    python3 tools/extract_running_strides.py

# What is measured, and how

Everything is a SEGMENT ATTITUDE in the sagittal plane (the treadmill's X
forward, Y up) — the zeros the walk's replay uses too (thigh from vertical,
foot from flat; see the knowledge note
`replay-a-recorded-gait-by-segment-attitudes.md`). A joint angle relative
to the pelvis would carry the pelvis's own 2-9 degree tilt onto the leg.

Thigh and shank are FROM VERTICAL, along the bones: each cluster's change
from the subject's static standing trial (which takes out where the
cluster sits on the limb), added to the bone's own standing attitude there,
hip joint centre (Harrington et al. 2007's regression on the pelvis
markers) to knee centre (the epicondyles' midpoint) to ankle centre (the
malleoli's). Standing, the runners' thighs lean back 5.6 degrees and their
shanks 3.1 (SD 2.7, 2.8), the hip about 6 cm ahead of the ankle. Left at the
standing values, the curves put a runner's leg that far forward of vertical
throughout: replayed, the hips came 43-50 mm lower at contact than at
toe-off, where the runners' own are 3-12 mm lower.

- thigh_deg   thigh cluster (top -> bottom marker centroids), forward of
              vertical (+), less its standing value, plus the thigh bone's.
- knee_deg    thigh_deg - shank_deg (the shank cluster likewise): flexion (+).
- foot_deg    heel-bottom -> metatarsal (MT1, MT5 midpoint) line, toe up (+),
              less its standing value (foot flat).
- pelvis_tilt_deg  PSIS -> ASIS midpoints, anterior tilt (+), less standing.
- pelvis_bob_mm    mean height of the four pelvis markers about its stride
              mean.

Foot contacts come from the treadmill's vertical force (> 20 N, dropouts
under 10 samples closed, blips under 30 dropped), each assigned to the foot
whose lowest marker is lower. Each stride runs from one foot's contact to
its next, resampled to 100 points (phase 0 = contact, the last sample one
step short of the next contact, so the samples tile one period). Strides of
both legs are averaged per subject (strides more than 10 % off the
subject's median length dropped), then subjects are averaged.

The summary CSV gives per speed: subjects, strides, stride seconds (mean,
sd), the share of the stride the foot is down (mean, sd), and the subjects'
mean thigh-plus-shank length, estimated as 0.491 x height (Winter Table
4.1's 0.245 + 0.246), for Froude scaling onto a rig.
"""

import argparse
import json
import pathlib
import sys
import urllib.request

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "assets/anim/reference/fukuchi_running_strides.csv"
SUMMARY = ROOT / "assets/anim/reference/fukuchi_running_summary.csv"
ARTICLE = "https://api.figshare.com/v2/articles/4543435"
SPEEDS = ("25", "35", "45")
SAMPLES = 100
FORCE_HZ = 300.0


def fetch(cache, names):
    """Downloads `names` from the figshare article into `cache`, if missing."""
    cache.mkdir(parents=True, exist_ok=True)
    listing = cache / "article.json"
    if not listing.exists():
        listing.write_bytes(urllib.request.urlopen(ARTICLE).read())
    files = {f["name"]: f["download_url"] for f in json.loads(listing.read_text())["files"]}
    for name in names:
        path = cache / name
        if not path.exists() or path.stat().st_size == 0:
            print(f"downloading {name}", file=sys.stderr)
            path.write_bytes(urllib.request.urlopen(files[name]).read())


def table(path):
    """A tab-separated file as {column: array}; rows with missing fields skipped."""
    lines = path.read_text().strip().split("\n")
    header = lines[0].split("\t")
    rows = []
    for line in lines[1:]:
        fields = line.split("\t")
        if len(fields) == len(header):
            rows.append([float(x) if x not in ("", "NaN", "nan") else np.nan for x in fields])
    data = np.array(rows)
    return {h: data[:, i] for i, h in enumerate(header)}


def point(m, name):
    return np.stack([m[name + "X"], m[name + "Y"], m[name + "Z"]], 1)


def centroid(m, *names):
    return np.nanmean(np.stack([point(m, n) for n in names]), 0)


def forward_of_vertical(top, bottom):
    """A hanging segment's angle from vertical, degrees, its lower end ahead (+)."""
    v = bottom - top
    return np.degrees(np.arctan2(v[..., 0], -v[..., 1]))


def attitudes(m, side):
    """(thigh, shank, foot, pelvis) attitudes, degrees, for one side."""
    thigh = forward_of_vertical(
        centroid(m, f"{side}.Thigh.Top.Lateral", f"{side}.Thigh.Top.Medial"),
        centroid(m, f"{side}.Thigh.Bottom.Lateral", f"{side}.Thigh.Bottom.Medial"),
    )
    shank = forward_of_vertical(
        centroid(m, f"{side}.Shank.Top.Lateral", f"{side}.Shank.Top.Medial"),
        centroid(m, f"{side}.Shank.Bottom.Lateral", f"{side}.Shank.Bottom.Medial"),
    )
    along = centroid(m, f"{side}.MT1", f"{side}.MT5") - point(m, f"{side}.Heel.Bottom")
    foot = np.degrees(np.arctan2(along[..., 1], along[..., 0]))
    pelvis = centroid(m, "R.ASIS", "L.ASIS") - centroid(m, "R.PSIS", "L.PSIS")
    tilt = np.degrees(np.arctan2(-pelvis[..., 1], pelvis[..., 0]))
    return thigh, shank, foot, tilt


def hip_centre(m, side):
    """The hip joint centre, standing: Harrington et al. 2007's regression
    on the pelvis's width and depth (J Biomech 40:595-602), mm."""
    ra, la, rp, lp = (np.nanmean(point(m, n), 0) for n in ("R.ASIS", "L.ASIS", "R.PSIS", "L.PSIS"))
    origin = 0.5 * (ra + la)
    width = np.linalg.norm(ra - la)
    lateral = (ra - la if side == "R" else la - ra) / width
    forward = origin - 0.5 * (rp + lp)
    depth = np.linalg.norm(forward)
    forward = forward - lateral * (forward @ lateral)
    forward /= np.linalg.norm(forward)
    up = np.cross(lateral, forward) if side == "R" else np.cross(forward, lateral)
    up *= np.sign(up[1])
    return origin + forward * (-0.24 * depth - 9.9) + up * (-0.30 * width - 10.9) + lateral * (0.33 * width + 7.3)


def standing_bones(m, side):
    """(thigh, shank) attitudes standing, degrees from vertical, along the
    bones: hip centre to knee centre (epicondyles' midpoint) to ankle centre
    (malleoli's midpoint)."""
    middle = lambda a, b: 0.5 * (np.nanmean(point(m, a), 0) + np.nanmean(point(m, b), 0))
    hip = hip_centre(m, side)
    knee = middle(f"{side}.Knee", f"{side}.Knee.Medial")
    ankle = middle(f"{side}.Ankle", f"{side}.Ankle.Medial")
    return float(forward_of_vertical(hip, knee)), float(forward_of_vertical(knee, ankle))


def contacts(force_y):
    """(on, off) force-sample indices of every foot contact."""
    # The files zero the force in flight; 20 N is the usual contact threshold.
    on = force_y > 20.0

    def runs(x):
        edges = np.flatnonzero(np.diff(np.r_[0, x.astype(int), 0]))
        return list(zip(edges[::2], edges[1::2]))

    for start, end in runs(~on):
        if end - start < 10 and start > 0:
            on[start:end] = True
    for start, end in runs(on):
        if end - start < 30:
            on[start:end] = False
    return [(s, e) for s, e in runs(on) if s > 0 and e < len(on)]


def trial(cache, subject, speed):
    """Each leg's strides in one trial, resampled: a list of dicts."""
    markers = table(cache / f"RBDS{subject:03d}runT{speed}markers.txt")
    force = table(cache / f"RBDS{subject:03d}runT{speed}forces.txt")
    static = table(cache / f"RBDS{subject:03d}static.txt")
    time = markers["Time"]
    # The time column is rounded to 1 ms; the rate comes from its span.
    marker_hz = (len(time) - 1) / (time[-1] - time[0])
    at = lambda k: k / FORCE_HZ * marker_hz

    def lowest(side):
        heel = point(markers, f"{side}.Heel.Bottom")[:, 1]
        toe = centroid(markers, f"{side}.MT1", f"{side}.MT5")[:, 1]
        return np.fmin(heel, toe)

    low = {"R": lowest("R"), "L": lowest("L")}
    feet = []
    for on, off in contacts(force["Fy"]):
        k = int(round(at(on + 5)))
        if k >= len(time):
            break
        feet.append((on, off, "R" if low["R"][k] < low["L"][k] else "L"))

    pelvis_y = np.nanmean(
        np.stack([point(markers, n)[:, 1] for n in ("R.ASIS", "L.ASIS", "R.PSIS", "L.PSIS")]), 0
    )
    strides = []
    for side in "RL":
        thigh, shank, foot, tilt = attitudes(markers, side)
        standing = [np.nanmean(x) for x in attitudes(static, side)]
        # The clusters' zeros moved onto the bones' own standing attitudes,
        # so thigh and shank are from vertical (see the module docs).
        bones = standing_bones(static, side)
        standing[0] -= bones[0]
        standing[1] -= bones[1]
        own = [c for c in feet if c[2] == side]
        for (on, off, _), (next_on, _, _) in zip(own, own[1:]):
            seconds = (next_on - on) / FORCE_HZ
            if not 0.4 < seconds < 1.2:
                continue
            index = np.linspace(at(on), at(next_on), SAMPLES + 1)[:-1]
            if index[-1] >= len(time) - 1:
                continue
            sample = lambda x: np.interp(index, np.arange(len(x)), x)
            bob = sample(pelvis_y)
            stride = {
                "seconds": seconds,
                "duty": (off - on) / (next_on - on),
                "thigh": sample(thigh) - standing[0],
                "knee": (sample(thigh) - standing[0]) - (sample(shank) - standing[1]),
                "foot": sample(foot) - standing[2],
                "pelvis_tilt": sample(tilt) - standing[3],
                "pelvis_bob": bob - np.mean(bob),
            }
            if all(np.all(np.isfinite(v)) for v in stride.values()):
                strides.append(stride)
    return strides


def subject_heights(cache):
    rows = [line.split("\t") for line in (cache / "RBDSinfo.txt").read_text().strip().split("\n")]
    header = rows[0]
    s, h = header.index("Subject"), header.index("Height")
    return {int(r[s]): float(r[h]) / 100.0 for r in rows[1:] if r[s].strip()}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--cache", type=pathlib.Path, default=ROOT / "target/running-data")
    # The metadata, and the paper, cover 28 runners.
    parser.add_argument("--subjects", type=int, default=28, help="subjects 1..N")
    args = parser.parse_args()

    names = ["RBDSinfo.txt"]
    for subject in range(1, args.subjects + 1):
        names.append(f"RBDS{subject:03d}static.txt")
        for speed in SPEEDS:
            names += [f"RBDS{subject:03d}runT{speed}markers.txt", f"RBDS{subject:03d}runT{speed}forces.txt"]
    fetch(args.cache, names)
    heights = subject_heights(args.cache)

    curves = ("thigh", "knee", "foot", "pelvis_tilt", "pelvis_bob")
    rows, summary = [], []
    for speed in SPEEDS:
        means, seconds, duties, legs, count = [], [], [], [], 0
        for subject in range(1, args.subjects + 1):
            strides = trial(args.cache, subject, speed)
            if len(strides) < 10:
                print(f"subject {subject} at {speed}: {len(strides)} strides, skipped", file=sys.stderr)
                continue
            median = np.median([s["seconds"] for s in strides])
            strides = [s for s in strides if abs(s["seconds"] - median) < 0.1 * median]
            count += len(strides)
            means.append({c: np.mean([s[c] for s in strides], 0) for c in curves})
            seconds.append(np.mean([s["seconds"] for s in strides]))
            duties.append(np.mean([s["duty"] for s in strides]))
            legs.append(0.491 * heights[subject])
        mean = {c: np.mean([m[c] for m in means], 0) for c in curves}
        mps = int(speed) / 10.0
        for i in range(SAMPLES):
            rows.append(
                f"{mps:.1f},{i},{mean['thigh'][i]:.2f},{mean['knee'][i]:.2f},{mean['foot'][i]:.2f},"
                f"{mean['pelvis_tilt'][i]:.2f},{mean['pelvis_bob'][i]:.2f}"
            )
        summary.append(
            f"{mps:.1f},{len(means)},{count},{np.mean(seconds):.4f},{np.std(seconds):.4f},"
            f"{np.mean(duties):.4f},{np.std(duties):.4f},{np.mean(legs):.4f}"
        )
        print(f"{mps} m/s: {len(means)} subjects, {count} strides, stride {np.mean(seconds):.3f} s, "
              f"duty {np.mean(duties):.3f}", file=sys.stderr)

    OUT.write_text("speed_mps,sample,thigh_deg,knee_deg,foot_deg,pelvis_tilt_deg,pelvis_bob_mm\n" + "\n".join(rows) + "\n")
    SUMMARY.write_text(
        "speed_mps,subjects,strides,stride_s,stride_s_sd,duty,duty_sd,leg_m\n" + "\n".join(summary) + "\n"
    )


if __name__ == "__main__":
    main()
