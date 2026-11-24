"""Сводка сохранённых CSV стенда. python summarize.py <каталог results>."""
import csv
import math
import statistics
import sys
from pathlib import Path

ENGINES = ("kinetica", "box3d", "rp3d", "bullet", "jolt")
SCENES = ("pyramid1000", "rain1000")
DIRECTIONS = ("x_plus", "x_minus", "z_plus", "z_minus")


def load(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def frames(path, first):
    rows = load(path)
    if [int(row["frame"]) for row in rows] != list(range(first, 121)):
        raise ValueError(f"Incomplete frame sequence in {path}")
    return rows


def horizontal_spread(path):
    """Запас между исходным квадратом центров и каждой гранью конечной выпуклой оболочки."""
    poses = load(path)
    def centers(frame):
        rows = [r for r in poses if int(r["frame"]) == frame]
        if [int(r["id"]) for r in rows] != list(range(1, 1001)):
            raise ValueError(f"Incomplete body set in {path}, frame {frame}")
        return [(float(r["x_m"]), float(r["z_m"])) for r in rows]

    initial, final = centers(0), centers(120)
    rx, rz = max(x for x, _ in initial), max(z for _, z in initial)
    corners = {(x, z) for x in (-rx, rx) for z in (-rz, rz)}
    if not corners.issubset(initial) or any(abs(x)>rx or abs(z)>rz for x,z in initial):
        raise ValueError("Initial footprint is not the prescribed centered rectangle")
    points = sorted(set(final))
    def chain(seq):
        hull = []
        for x, z in seq:
            while len(hull) >= 2:
                (ax, az), (bx, bz) = hull[-2:]
                if (bx-ax)*(z-az) > (bz-az)*(x-ax):
                    break
                hull.pop()
            hull.append((x, z))
        return hull[:-1]
    hull = chain(points) + chain(reversed(points))
    clearance = math.inf
    for (ax, az), (bx, bz) in zip(hull, hull[1:]+hull[:1]):
        dx, dz = bx-ax, bz-az
        # Наружная нормаль обхода против часовой стрелки: (dz,-dx)/длина ребра.
        clearance = min(clearance, (dz*ax-dx*az-abs(dz)*rx-abs(dx)*rz)/math.hypot(dx,dz))
    sectors = [0]*8
    for x,z in final:
        if max(abs(x)-rx, abs(z)-rz) >= .125:
            k = math.floor((math.atan2(z,x)+math.pi/8)/(math.pi/4)) % 8
            sectors[k] += 1
    return clearance, min(sectors), sum(sectors)


def observations(folder, scene, engine):
    rows = frames(folder / f"{scene}-{engine}-verify-0.csv", 0)
    start, end = rows[0], rows[-1]
    def peak(key, seq=rows):
        return max(float(r[key]) for r in seq)

    floor, pair = peak("floor_mm"), peak("pair_mm")
    violations = [int(r["frame"]) for r in rows[1:] if float(r["floor_mm"]) > 2 or float(r["pair_mm"]) > 5]
    spread = min(float(end[k]) - float(start[k]) for k in DIRECTIONS)
    drop = float(end["apex_drop_m"])
    ballistic = peak("ballistic_mm", rows[1:13]) if scene == "rain1000" else None
    complete = peak("below") == 0 and peak("asleep") == 0
    scene_pass = ballistic <= 3 if ballistic is not None else drop >= .25 and spread >= .125
    perimeter = horizontal_spread(folder / f"{scene}-{engine}-verify-0-poses.csv") if scene == "pyramid1000" else (None,)*3
    return dict(floor_mm=floor, floor_frame=next(r["frame"] for r in rows if float(r["floor_mm"]) == floor),
                pair_mm=pair, pair_frame=next(r["frame"] for r in rows if float(r["pair_mm"]) == pair),
                violation_frames=len(violations), first_violation=violations[0] if violations else None,
                max_below=int(peak("below")), max_asleep=int(peak("asleep")), at_floor=int(end["at_floor"]),
                apex_drop_m=drop, com_drop_m=float(start["com_y_m"])-float(end["com_y_m"]),
                min_spread_m=spread, hull_margin_m=perimeter[0], min_sector=perimeter[1], outside_cubes=perimeter[2],
                ballistic_mm=ballistic, energy_J_0=float(start["energy_J"]),
                energy_J_120=float(end["energy_J"]), gain_percent=100*(peak("energy_J")/float(start["energy_J"])-1),
                scene_pass=scene_pass, passed=complete and scene_pass and floor <= 2 and pair <= 5,
                final_hash=end["state_hash"])


def timing(folder, scene, engine, final_hash):
    means, all_ms = [], []
    for block in range(1, 6):
        path = folder / f"{scene}-{engine}-time-{block}.csv"
        rows = frames(path, 1)
        if {r["final_hash"] for r in rows} != {final_hash}:
            raise ValueError(f"Timing and geometry ended in different states in {path}")
        samples = [float(r["step_us"])/1000 for r in rows]
        if any(not math.isfinite(x) or x <= 0 for x in samples):
            raise ValueError(f"Invalid timer in {path}")
        means.append(statistics.mean(samples))
        all_ms.extend(samples)
    return dict(blocks=len(means), frames=len(all_ms), median_block_ms=statistics.median(means),
                min_block_ms=min(means), max_block_ms=max(means), frame_median_ms=statistics.median(all_ms),
                p99_ms=sorted(all_ms)[math.ceil(.99*len(all_ms))-1], max_ms=max(all_ms),
                over_60hz=sum(x > 1000/60 for x in all_ms))


def main(folder):
    order = load(folder / "order.csv")
    for scene in SCENES:
        for engine in ENGINES:
            slots = sorted((int(r["block"]), int(r["slot"])) for r in order
                           if (r["scene"], r["engine"]) == (scene, engine))
            if [b for b, _ in slots] != list(range(1, 6)) or sorted(s for _, s in slots) != list(range(1, 6)):
                raise ValueError(f"Unbalanced order for {scene}, {engine}")
    out = None
    for scene in SCENES:
        for engine in ENGINES:
            obs = observations(folder, scene, engine)
            row = dict(scene=scene, engine=engine, **timing(folder, scene, engine, obs["final_hash"]), **obs)
            if out is None:
                out = csv.DictWriter(sys.stdout, fieldnames=row)
                out.writeheader()
            out.writerow(row)


if __name__ == "__main__":
    main(Path(sys.argv[1]))
