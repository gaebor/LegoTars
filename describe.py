"""Describe the columns in a tars_control capture.

Run with: uv run describe.py
Optionally pass a capture directory: uv run describe.py --capture results/tars_control/<run>
"""

import argparse
import json
from pathlib import Path

import numpy as np


def newest_capture(root: Path) -> Path:
    captures = [p for p in root.iterdir() if p.is_dir() and (p / "measurements.npy").is_file()]
    if not captures:
        raise FileNotFoundError(f"No measurements.npy captures found under {root}")
    return max(captures, key=lambda p: (p / "measurements.npy").stat().st_mtime_ns)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--capture",
        type=Path,
        help="capture folder; defaults to the newest capture under --measurement",
    )
    parser.add_argument("--measurement", type=Path, default=Path("results/tars_control"))
    args = parser.parse_args()

    capture = args.capture or newest_capture(args.measurement)
    data = np.load(capture / "measurements.npy")
    columns = json.loads((capture / "columns.json").read_text(encoding="utf-8"))

    time_values = data[:, columns.index("time")]

    print(f"Capture: {capture}")
    for index, name in enumerate(columns):
        values = data[:, index]
        finite = np.isfinite(values)
        measured = values[finite]
        if not measured.size:
            print(f"\n{name}: no finite samples")
            continue
        nonzero = finite & (values != 0)
        print(f"\n{name} ({measured.size} finite samples)")
        print(f"  mean: {np.mean(measured):g}")
        print(f"  std:  {np.std(measured):g}  (population)")
        print(f"  min:  {np.min(measured):g}")
        print(f"  max:  {np.max(measured):g}")
        print(f"  nonzero samples: {np.count_nonzero(nonzero)}")
        nonzero_indices = np.flatnonzero(nonzero)
        if nonzero_indices.size:
            first, last = nonzero_indices[0], nonzero_indices[-1]
            print(f"  first nonzero: {time_values[first]:.3f} s, {values[first]:g}")
            print(f"  last nonzero:  {time_values[last]:.3f} s, {values[last]:g}")
        else:
            print("  first/last nonzero: none")


if __name__ == "__main__":
    main()
