"""Analyse complete paired timing blocks and a fixed physical grid from the Windows stand."""
import csv
import json
from collections import Counter
from math import comb, isfinite, sqrt
from pathlib import Path
from statistics import NormalDist, mean, median

from figures import LAB, OUT, BLUE, TEAL, AMBER, GRAY, plt

DATA = LAB / "residual-budget/wilson"
NAMES = {"control": "Исходная Kinetica", "kinetica": "Опытная Kinetica", "rp3d": "ReactPhysics3D",
         "box3d": "Box3D", "bullet-f64": "Bullet double · 8 итераций"}
Z95 = NormalDist().inv_cdf(.975)


def wilson(wins, n):
    p = wins / n
    offset = Z95 * Z95 / n
    centre = (p + offset / 2) / (1 + offset)
    radius = Z95 * sqrt(p * (1 - p) / n + Z95 * Z95 / (4 * n * n)) / (1 + offset)
    return (0.0 if wins == 0 else centre - radius, 1.0 if wins == n else centre + radius)


def sign_p(positive, negative):
    n = positive + negative
    return min(1., 2 * sum(comb(n, i) for i in range(min(positive, negative) + 1)) / 2**n)


def median_interval(values):
    ordered = sorted(values)
    n = len(ordered)
    tail = 0
    for rank in range(1, n // 2 + 1):
        tail += comb(n, rank - 1)
        coverage = 1 - 2 * tail / 2**n
        if coverage < .95:
            break
        interval = ordered[rank - 1], ordered[-rank], coverage
    return interval


def read(name):
    with (DATA / name).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def save(name, rows):
    with (DATA / name).open("w", encoding="utf8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def load_experiment():
    manifest = json.loads((DATA / "inputs.json").read_text())
    assert manifest["blocks"] == 40 and manifest["seed"] == 2026112307 and manifest["independent_order_per_block"]
    assert manifest["frames"] == 360 and manifest["steps_per_frame"] == 8 and manifest["threads"] == 1 and not manifest["sleep"]
    assert set(manifest["sha256"]) == NAMES.keys()
    times = {name: {} for name in NAMES}
    slots = Counter()
    for row in read("timing.csv"):
        name, block, slot = row["variant"], int(row["block"]), int(row["slot"])
        value = float(row["us_per_frame"])
        assert block not in times[name] and isfinite(value) and value > 0 and int(row["frames"]) == 360
        times[name][block] = value
        slots[block, slot] += 1
    blocks = range(1, 41)
    assert all(set(times[name]) == set(blocks) for name in NAMES)
    assert slots == Counter((b, s) for b in blocks for s in range(1, len(NAMES) + 1))
    physics = {name: {} for name in NAMES}
    loads = {(.5,10,f,0) for f in (0,8,9.8,9.9,10,10.1,10.2,12,-12)} | {(0,10,10,0),(.5,0,10,0),(.5,10,10,1)}
    expected = {(hz,*case) for hz in (60,120,240) for case in loads}
    for row in read("accuracy.csv"):
        key = int(row["hz"]), float(row["mu"]), float(row["jaw_N"]), float(row["load_N"]), int(row["open"])
        grid = physics[row["variant"]]
        assert key not in grid and row["complete"] == "1" and row["pass"] in ("0","1")
        grid[key] = row
    assert all(set(grid) == expected for grid in physics.values())
    return times, physics, blocks


def main():
    times, physics, blocks = load_experiment()
    candidate = times["kinetica"]
    timing, pairs, accuracy = [], [], []
    for name, values in times.items():
        v = list(values.values())
        inside = sum(t <= 50 for t in v)
        low, high = wilson(inside, len(v))
        grip = physics[name][60,.5,10,10,0]
        timing.append(dict(variant=name, n=len(v), median_us=median(v), mean_us=mean(v), min_us=min(v),
                           max_us=max(v), within_50=inside, within_50_wilson_low=low, within_50_wilson_high=high,
                           drift_mm=grip["max_travel_mm"], timed_case_pass=int(grip["pass"]),
                           grid_pass=sum(r["pass"] == "1" for r in physics[name].values())))
        if name == "kinetica":
            continue
        ratio = [candidate[b] / values[b] for b in blocks]
        diff = [candidate[b] - values[b] for b in blocks]
        wins, losses = sum(x < 0 for x in diff), sum(x > 0 for x in diff)
        low, high = wilson(wins, len(diff))
        rl, rh, coverage = median_interval(ratio)
        dl, dh, _ = median_interval(diff)
        batches = [median(ratio[i:i + 5]) for i in range(0, len(ratio), 5)]
        batch_wins = sum(x < 1 for x in batches)
        bl, bh = wilson(batch_wins, len(batches))
        pairs.append(dict(peer=name, wins=wins, losses=losses, ties=len(diff)-wins-losses, n=len(diff),
                          win_wilson_low=low, win_wilson_high=high, ratio_median=median(ratio), ratio_low=rl, ratio_high=rh,
                          difference_median_us=median(diff), difference_low_us=dl, difference_high_us=dh,
                          interval_coverage=coverage, sign_p=sign_p(wins,losses),
                          batches_of_five=len(batches), batch_wins=batch_wins, batch_low=bl, batch_high=bh))
        counts = Counter((int(own["pass"]), int(physics[name][key]["pass"])) for key, own in physics["kinetica"].items())
        accuracy.append(dict(peer=name, both_pass=counts[1,1], only_kinetica=counts[1,0],
                             only_peer=counts[0,1], both_fail=counts[0,0]))
    highest = 0
    for rank, row in enumerate(sorted(pairs, key=lambda r: r["sign_p"])):
        highest = max(highest, min(1., (len(pairs)-rank) * row["sign_p"]))
        row["holm_p"] = highest
    save("timing-summary.csv", timing)
    save("paired-statistics.csv", pairs)
    save("grid-pairs.csv", accuracy)
    fig, (times_ax, win_ax) = plt.subplots(1, 2, figsize=(11.2,3.8), layout="constrained", width_ratios=[1.18,1])
    for i, row in enumerate(timing):
        name = row["variant"]
        color = TEAL if name=="kinetica" else AMBER if not row["timed_case_pass"] else GRAY
        values = list(times[name].values())
        times_ax.scatter(values, [i]*len(values), alpha=.22, color=color, s=14, edgecolor="none")
        times_ax.scatter(row["median_us"], i, marker="D", color=color, s=42)
        times_ax.annotate(f'{row["median_us"]:.2f}'.replace(".",","), (row["max_us"]+3,i), va="center", fontsize=8.5, color=color)
    times_ax.set_yticks(range(len(timing)),[NAMES[r["variant"]] for r in timing])
    times_ax.axvline(50, ls="--", lw=1, color=TEAL)
    times_ax.set_xlim(27,max(r["max_us"] for r in timing)*1.17)
    times_ax.set_title("Полный кадр захвата",fontsize=12)
    times_ax.set_xlabel("мкс · точки — 40 блоков, ромбы — медианы")
    for i, row in enumerate(pairs):
        lo, hi = row["win_wilson_low"]*100, row["win_wilson_high"]*100
        win_ax.plot([lo,hi],[i,i],color=BLUE,lw=3,solid_capstyle="round")
        win_ax.scatter(100*row["wins"]/row["n"],i,color=BLUE,s=38,zorder=3)
    win_ax.axvline(50,color=GRAY,lw=.9,ls="--")
    win_ax.set_yticks(range(len(pairs)), [NAMES[r["peer"]] for r in pairs])
    win_ax.set(xlim=(-4,104),xticks=[0,25,50,75,100],xlabel="доля блоков, где опытная Kinetica быстрее, %")
    win_ax.set_title("Победы и 95% интервал Уилсона",fontsize=12)
    for ax in (times_ax,win_ax):
        ax.invert_yaxis()
        ax.grid(axis="x", alpha=.2)
        ax.tick_params(axis="y",length=0)
    fig.savefig(OUT/"grip-wilson-2026-11-23.png", dpi=165, bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    main()
