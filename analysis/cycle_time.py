#!/usr/bin/env python3
"""Cycle-time breakdown from a benchmark run.

Reads the JSON that ``analysis/grasp_benchmark.py`` writes and reports where
the time went::

    python analysis/cycle_time.py --input results/grasp_benchmark.json

Recomputed from the stored per-segment durations rather than measured again, so
the total here and the total the benchmark printed cannot disagree.

WHAT TO DO WITH THE SPLIT

A cycle time is only actionable once it is split by what produced it:

* **motion** responds to trajectory scaling, to shortcutting, and to moving the
  scan pose closer to the pick region. It is the large term here.
* **sensor** is the camera settle and transfer. It responds to nothing but a
  faster camera or overlapping the capture with the previous cycle's retreat.
* **gripper** is the jaw stroke. It responds to commanding a smaller opening,
  which trades against catching the part on the way down.
* **fieldbus** is the PLC handshake, two scans of it, and is noise at this
  scale. It is reported anyway because it is the term everyone assumes is
  large.

The per-segment table underneath says which individual move to attack. On this
cell the two free-space moves -- home to scan and place to home -- are together
about half the cycle, and neither of them does anything useful with the part.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List

import numpy as np


def summarise(records: List[Dict]) -> Dict:
    successes = [record for record in records if record["success"]]
    if not successes:
        return {"successful_cycles": 0}

    per_segment: Dict[str, List[float]] = defaultdict(list)
    per_kind: Dict[str, List[float]] = defaultdict(list)
    for record in successes:
        for segment in record["segments"]:
            per_segment[segment["name"]].append(segment["duration"])
        for kind, value in record["by_kind"].items():
            per_kind[kind].append(value)

    totals = np.array([record["cycle_time"] for record in successes])
    order = sorted(per_segment.items(), key=lambda item: -float(np.mean(item[1])))

    return {
        "successful_cycles": len(successes),
        "total_s": {
            "mean": round(float(np.mean(totals)), 3),
            "median": round(float(np.median(totals)), 3),
            "p05": round(float(np.percentile(totals, 5)), 3),
            "p95": round(float(np.percentile(totals, 95)), 3),
            "min": round(float(np.min(totals)), 3),
            "max": round(float(np.max(totals)), 3),
        },
        "by_kind_s": {
            kind: {
                "mean": round(float(np.mean(values)), 3),
                "share": round(float(np.mean(values) / np.mean(totals)), 4),
            }
            for kind, values in sorted(per_kind.items())
        },
        "by_segment_s": {
            name: {
                "mean": round(float(np.mean(values)), 3),
                "p95": round(float(np.percentile(values, 95)), 3),
                "share": round(float(np.mean(values) / np.mean(totals)), 4),
                "n": len(values),
            }
            for name, values in order
        },
    }


def report(summary: Dict) -> str:
    if not summary.get("successful_cycles"):
        return "no successful cycles in the input"

    total = summary["total_s"]
    lines = [
        f"Cycle time over {summary['successful_cycles']} successful cycles",
        f"  median {total['median']:.2f} s   mean {total['mean']:.2f} s   "
        f"p05 {total['p05']:.2f}   p95 {total['p95']:.2f}   "
        f"range {total['min']:.2f}-{total['max']:.2f}",
        "",
        "By kind:",
    ]
    for kind, entry in summary["by_kind_s"].items():
        lines.append(f"  {kind:<10} {entry['mean']:>6.2f} s  {entry['share'] * 100:>5.1f}%")
    lines.append("")
    lines.append("By segment, slowest first:")
    for name, entry in summary["by_segment_s"].items():
        lines.append(
            f"  {name:<24} {entry['mean']:>6.2f} s  p95 {entry['p95']:>5.2f}  {entry['share'] * 100:>5.1f}%"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", type=Path, default=Path("results/grasp_benchmark.json"))
    parser.add_argument("--condition", default="nominal")
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()

    if not arguments.input.exists():
        print(f"{arguments.input} does not exist. Run `make grasp` first.", file=sys.stderr)
        return 1

    payload = json.loads(arguments.input.read_text(encoding="utf-8"))
    # The benchmark writes either one condition's summary or a dict of them.
    if "records" in payload:
        records = payload["records"]
    elif arguments.condition in payload:
        records = payload[arguments.condition]["records"]
    else:
        print(f"no condition {arguments.condition!r} in {arguments.input}", file=sys.stderr)
        return 1

    summary = summarise(records)
    summary["condition"] = arguments.condition
    print(report(summary))

    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"\nwrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
