"""Summarize 10 paired Windows trials after unpacking support-timing-windows.tar.gz."""
import csv
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paired import wilson95
from summarize import frames, load


def frame_times(folder, scene, block, mode, expected):
    path = folder / f"{scene}-block-{block}-mode-{mode}.csv"
    rows = frames(path, 1)
    if {r["final_hash"] for r in rows} != {expected}:
        raise ValueError(f"unexpected final state in {path}")
    ms = [float(r["step_us"]) / 1000 for r in rows]
    if any(not math.isfinite(t) or t <= 0 for t in ms):
        raise ValueError(f"invalid duration in {path}")
    return ms


def compare(folder, scene):
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8-sig"))
    order = [r for r in load(folder / "order.csv") if r["scene"] == scene]
    if len(order) != 20:
        raise ValueError(f"incomplete order for {scene}")
    pooled = {0: [], 3: []}
    pairs = []
    for block in range(1, 11):
        jobs = [r for r in order if int(r["block"]) == block]
        modes = [int(r["support"]) for r in jobs]
        if set(modes) != {0,3} or [r["slot"] for r in jobs] != ["1","2"]:
            raise ValueError(f"unpaired block {block} for {scene}")
        mean = {}
        for mode in modes:
            ms = frame_times(folder, scene, block, mode, manifest["expected_hashes"][f"{scene}-{mode}"])
            pooled[mode].extend(ms)
            mean[mode] = statistics.mean(ms)
        pairs.append(dict(scene=scene, block=block, first=modes[0], off_ms=mean[0], on_ms=mean[3],
                          delta_ms=mean[3]-mean[0], delta_pct=100*(mean[3]/mean[0]-1)))
    if sum(p["first"] == 0 for p in pairs) != 5:
        raise ValueError(f"unbalanced first position for {scene}")
    changes = [p["delta_pct"] for p in pairs]
    slower = sum(c > 0 for c in changes)
    n = sum(c != 0 for c in changes)
    lower, upper = wilson95(slower, n)
    p = min(1, 2*sum(math.comb(n, k) for k in range(min(slower, n-slower)+1))/2**n)
    def p99(ms):
        return sorted(ms)[math.ceil(.99*len(ms))-1]
    summary = dict(scene=scene, pairs=len(pairs), frames_per_mode=len(pooled[0]),
                   off_ms=statistics.median(r["off_ms"] for r in pairs),
                   on_ms=statistics.median(r["on_ms"] for r in pairs),
                   delta_ms=statistics.median(r["delta_ms"] for r in pairs),
                   delta_pct=statistics.median(changes), min_pct=min(changes), max_pct=max(changes),
                   off_p99_ms=p99(pooled[0]), on_p99_ms=p99(pooled[3]), slower=slower, ties=len(pairs)-n,
                   wilson_low=lower, wilson_high=upper, sign_p_two_sided=p)
    return pairs, summary


def write(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]),lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main(root, out):
    out.mkdir(parents=True,exist_ok=True)
    details, summary = [], []
    for scene in ("pyramid1000","rain1000","rest55"):
        folder = root / ("support-time-10-v1-rest" if scene=="rest55" else "support-time-10-v1")
        pairs, result = compare(folder, scene)
        details.extend(pairs)
        summary.append(result)
    write(out/"support-time-pairs.csv", details)
    write(out/"support-time-summary.csv", summary)


if __name__=="__main__":
    main(*(Path(p) for p in sys.argv[1:]))
