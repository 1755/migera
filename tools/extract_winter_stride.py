"""Extract Winter's Appendix A walking stride into assets/anim/reference/.

Source: D. A. Winter, *Biomechanics and Motor Control of Human Movement*,
4th ed., Wiley 2009, Appendix A (printed pp. 296-360; PDF page = printed +
13). The book lives in docs/books/bmcoh/ and is gitignored, so the extracted
CSV is committed and this script is how it was made. Re-run it after
replacing the PDF:

    python3 tools/extract_winter_stride.py

Columns, one row per frame (1-106, 69.9 frames/s):

- event          TOR (toe-off right) at 1 and 70, HCR (heel contact right)
                 at 28 and 97; empty otherwise.
- hip_deg        Table A.4, hip flexion (+), thigh relative to 1/2 HAT.
- knee_deg       Table A.4, knee flexion (+).
- ankle_deg      Table A.4, ankle DORSIflexion (+). Opposite polarity to the
                 formula in section 3.5.2; the data (-20 deg at toe-off) fix it.
- hip_marker_y   Table A.2(a), greater trochanter height, m.
- hat_vx         Table A.3(d), half-HAT centre-of-mass forward speed, m/s.
- thigh_deg      Table A.3(c), thigh segment angle, ABSOLUTE: counterclockwise
                 from +X (the direction of travel), knee -> trochanter, so 90 is
                 vertical and above 90 the knee is ahead of the hip.
- hat_deg        Table A.3(d), 1/2 HAT segment angle, absolute, same convention.
- foot_deg       Table A.3(a), foot segment angle (5th metatarsal -> ankle),
                 absolute, same convention.

All sign conventions are recorded in
docs/knowledge/biomechanics-winter/appendices/a-walking-trial-kinematic-kinetic-energy-data.md.
"""

import csv
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
PDF = ROOT / "docs/books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf"
OUT = ROOT / "assets/anim/reference/winter_walking_stride.csv"

# `[event] frame time values...`, as pdftotext -layout prints every table row.
ROW = re.compile(r"^\s*(?:([A-Z]{3})\s+)?(\d{1,3})\s+(\d\.\d{3})\s+(.*)$")
FRAMES = range(1, 107)
EVENTS = {1: "TOR", 28: "HCR", 70: "TOR", 97: "HCR"}


def pdf_pages():
    text = subprocess.run(
        ["pdftotext", "-layout", str(PDF), "-"], check=True, capture_output=True, text=True
    ).stdout
    return text.split("\f")


def table(pages, first, last, columns):
    """Rows of a table spanning PDF pages first..last, keyed by frame."""
    rows = {}
    for page in pages[first - 1 : last]:
        for line in page.splitlines():
            match = ROW.match(line)
            if not match:
                continue
            fields = match.group(4).replace("−", "-").split()
            if len(fields) != columns:
                continue
            try:
                values = [float(v) for v in fields]
            except ValueError:
                continue
            rows[int(match.group(2))] = (float(match.group(3)), values)
    missing = [f for f in FRAMES if f not in rows]
    if missing:
        sys.exit(f"PDF pages {first}-{last}: frames {missing} did not parse")
    return rows


def main():
    pages = pdf_pages()
    joints = table(pages, 354, 358, 9)  # A.4: ankle, knee, hip (theta, omega, alpha each)
    markers = table(pages, 314, 318, 12)  # A.2(a): rib cage, greater trochanter
    hat = table(pages, 349, 353, 9)  # A.3(d): 1/2 HAT
    thigh = table(pages, 344, 348, 9)  # A.3(c): thigh
    foot = table(pages, 334, 338, 9)  # A.3(a): foot

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "w", newline="") as handle:
        out = csv.writer(handle)
        out.writerow(
            [
                "frame",
                "time_s",
                "event",
                "hip_deg",
                "knee_deg",
                "ankle_deg",
                "hip_marker_y",
                "hat_vx",
                "thigh_deg",
                "hat_deg",
                "foot_deg",
            ]
        )
        for frame in FRAMES:
            time, a4 = joints[frame]
            out.writerow(
                [
                    frame,
                    f"{time:.3f}",
                    EVENTS.get(frame, ""),
                    a4[6],
                    a4[3],
                    a4[0],
                    markers[frame][1][9],
                    hat[frame][1][4],
                    thigh[frame][1][0],
                    hat[frame][1][0],
                    foot[frame][1][0],
                ]
            )
    print(f"wrote {OUT.relative_to(ROOT)}: {len(FRAMES)} frames")


if __name__ == "__main__":
    main()
