#!/usr/bin/env python3
"""Reference comparator for the C filter-bank port.

Regenerates the exact synthetic profile from tests/test_filters_host.c,
runs the ORIGINAL benchmark reference filters (filter_benchmark/src/filters),
and compares them against the C runner's CSV output within float32
tolerance.

Usage:
  1. build+run the C runner (MSVC vcvars64):
       cl /TC /DVARIO_USE_DOUBLE tests/test_filters_host.c ^
          main/filters.c main/vario_core.c
       test_filters_host.exe > tests/out_c.csv
  2. python tests/test_filters_ref.py tests/out_c.csv
"""
import csv
import math
import sys
from pathlib import Path

import numpy as np

# benchmark reference package (workbook checkout)
WB = Path(r"C:\Users\zhang\OneDrive\Documents\Raghav\Engineeirng projects"
          r"\Vario USB\Kimi Workbook\Vario USB\filter_benchmark\src")
sys.path.insert(0, str(WB))
from filters import FILTERS  # noqa: E402
from filters.stacked import S4FastSlowGated, S5LightGateKF  # noqa: E402

FS_HZ = 40.0
N = int(20.0 * FS_HZ)
# C-runner column order
COLS = ["C03", "D01", "S4", "S5"]
REF = {"C03": "C03_CV_tracker", "D01": "D01_Kalman_CV",
       "S4": "S4_FastSlowGated", "S5": "S5_LightGateKF"}
TOL = 5e-3   # m/s — float32 port vs float64 reference


def profile_z(t: float) -> float:
    w = (0.05 * math.sin(2 * math.pi * 0.7 * t)
         + 0.02 * math.sin(2 * math.pi * 3.1 * t))
    if t < 5.0:
        return w
    if t < 10.0:
        return 1.2 * (t - 5.0) + w
    if t < 10.5:
        return 6.0 + w
    return 6.0 - 4.9 * (1.0 - math.exp(-(t - 10.5) / 0.15)) + w


def main(path: str) -> int:
    t = np.arange(N) / FS_HZ
    z = np.array([profile_z(float(x)) for x in t])

    refs = {}
    for short, name in REF.items():
        f = FILTERS[name]() if short in ("C03", "D01") else \
            (S4FastSlowGated() if short == "S4" else S5LightGateKF())
        refs[short] = np.array([f.update(float(zi), float(ti))
                                for zi, ti in zip(z, t)])

    rows = list(csv.DictReader(open(path, newline="")))
    assert len(rows) == N, f"row count {len(rows)} != {N}"

    worst = {c: 0.0 for c in COLS}
    n_bad = 0
    for i, row in enumerate(rows):
        for c in COLS:
            got = float(row[c])
            want = float(refs[c][i])
            err = abs(got - want)
            worst[c] = max(worst[c], err)
            if err > TOL:
                n_bad += 1
                if n_bad < 10:
                    print(f"MISMATCH i={i} t={t[i]:.3f} {c}: "
                          f"C={got:.6f} ref={want:.6f} err={err:.2e}")
    print("\nworst |err| per filter (m/s):")
    for c in COLS:
        print(f"  {c}: {worst[c]:.3e}  {'OK' if worst[c] <= TOL else 'FAIL'}")
    if n_bad:
        print(f"\nFAIL: {n_bad} samples beyond tolerance {TOL}")
        return 1
    print(f"\nPASS: all {N * len(COLS)} samples within {TOL} m/s of reference")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "tests/out_c.csv"))
