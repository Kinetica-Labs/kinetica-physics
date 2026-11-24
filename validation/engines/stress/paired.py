"""Сравнение времени Box3D и Jolt в отдельных 12 парах с протоколом confirm-order.csv."""
import csv
import math
import statistics
import sys
from pathlib import Path

from summarize import SCENES, frames, load


def wilson95(wins, untied):
    if not untied:
        return None, None
    n = untied
    z = statistics.NormalDist().inv_cdf(.975)
    rate = wins/n
    centre = (rate + z*z/(2*n))/(1+z*z/n)
    radius = z*math.sqrt(rate*(1-rate)/n + z*z/(4*n*n))/(1+z*z/n)
    return max(0.,centre-radius), min(1.,centre+radius)


def compare(folder, out):
    out.mkdir(exist_ok=True, parents=True)
    orders = load(folder / "confirm-order.csv")
    details, summary = [], []
    for scene in SCENES:
        paired = []
        for b in range(1, 13):
            jobs = [r for r in orders if int(r["block"]) == b and r["scene"] == scene]
            if [int(r["slot"]) for r in jobs] != [1, 2] or {r["engine"] for r in jobs} != {"box3d","jolt"}:
                raise ValueError(f"Incomplete or duplicated pair in {scene}, block {b}")
            ms = {}
            for r in jobs:
                engine = r["engine"]
                rows = frames(folder / f"{scene}-{engine}-time-{100+b}.csv", 1)
                reference = frames(folder / f"{scene}-{engine}-verify-0.csv", 0)[-1]["state_hash"]
                if {row["final_hash"] for row in rows} != {reference}:
                    raise ValueError(f"State mismatch in {scene}, block {b}, {engine}")
                vals = [float(row["step_us"])/1000 for row in rows]
                if any(not math.isfinite(x) or x <= 0 for x in vals):
                    raise ValueError("Invalid time observation")
                ms[engine] = statistics.mean(vals)
            paired.append(dict(scene=scene, block=b, first=jobs[0]["engine"], box3d_ms=ms["box3d"], jolt_ms=ms["jolt"],
                               saved_ms=ms["jolt"]-ms["box3d"], ratio=ms["jolt"]/ms["box3d"]))
        details.extend(paired)
        ratios = sorted(r["ratio"] for r in paired)
        w = sum(r["saved_ms"]>0 for r in paired)
        n = sum(r["saved_ms"]!=0 for r in paired)
        p = min(1., 2*sum(math.comb(n,k) for k in range(min(w,n-w)+1))/2**n)
        lo, hi = wilson95(w,n)
        summary.append(dict(scene=scene, pairs=len(paired), wins=w, losses=n-w, ties=len(paired)-n,
                            box3d_ms=statistics.median(r["box3d_ms"] for r in paired),
                            jolt_ms=statistics.median(r["jolt_ms"] for r in paired), median_ratio=statistics.median(ratios),
                            min_ratio=ratios[0], max_ratio=ratios[-1], exact_sign_p_two_sided=p,
                            wilson_95_low=lo, wilson_95_high=hi))
    for name, rows in [("pairs",details), ("statistics",summary)]:
        with (out / f"box3d-jolt-{name}.csv").open("w",newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0])
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    compare(Path(sys.argv[1]),Path(sys.argv[2]))
