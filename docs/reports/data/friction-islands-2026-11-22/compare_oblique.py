"""Compare Windows oblique-grip CSVs. Usage: python compare_oblique.py candidate=a.csv baseline=b.csv"""
import argparse
import cmath
import csv
import itertools
import json
import math
from pathlib import Path


SPEED = 1e-6                 # m/s, also the resolution of a paired difference
NORMAL = 1e-7                # N s
NORMAL_IMPULSE = 10 / 480    # 10 N for one step of 1/480 s
INPUT_TOL = 1e-8             # m/s, 1% of the speed criterion, accommodates float input conversion
INITIAL = ((.004+.001j, -.003+.002j), (.08+.03j, -.006+.011j),
           (.05-.04j, -.08+.06j), (0j, .04+.03j))
GRID = set(itertools.product(range(4), (0, 37, 83), ((.5, .5), (.05, .6), (.6, .05), (0., 0.))))
LIMITS = {"pole_m_s": SPEED, "relative_m_s": SPEED, "jaw_m_s": SPEED, "normal_Ns": NORMAL}
SIDES = ("left", "right")


def number(row, field):
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"{field} is not finite")
    return value


def vector(row, field):
    return tuple(complex(number(row, f"{field}_{s}_y"), number(row, f"{field}_{s}_z")) for s in SIDES)


def distance(a, b):
    return max(abs(x-y) for x, y in zip(a, b))


def inspect(row, key, preload_steps):
    trial, angle, _ = key
    if int(row["preload_steps"]) != preload_steps or int(row["step_complete"]) != 1:
        raise ValueError(f"expected {preload_steps} preload steps and one completed measurement step")
    rotation = cmath.exp(1j * math.radians(angle))
    expected_input = [u * rotation for u in INITIAL[trial]]
    if distance(vector(row, "initial"), expected_input) > INPUT_TOL:
        raise ValueError("recorded initial velocities differ from the specified trial")
    oracle, pole, jaw = (vector(row, field) for field in ("oracle", "observed", "jaw"))
    relative = [p - j for p, j in zip(pole, jaw)]
    if distance(relative, vector(row, "relative")) > INPUT_TOL:
        raise ValueError("logged relative velocity disagrees with pole minus jaw")
    normal = max(abs(number(row, f"normal_{s}_Ns") - NORMAL_IMPULSE) for s in SIDES)
    metrics = dict(pole_m_s=distance(pole, oracle), relative_m_s=distance(relative, oracle),
                   jaw_m_s=max(map(abs, jaw)), normal_Ns=normal)
    if not all(map(math.isfinite, metrics.values())):
        raise ValueError("non-finite derived error")
    return metrics, oracle


def read_file(path, preload_steps=120):
    rows, oracles = {}, {}
    with Path(path).open(encoding="utf-8-sig", newline="") as source:
        for line, row in enumerate(csv.DictReader(source), start=2):
            try:
                key = int(row["trial"]), int(row["angle_deg"]), (number(row, "mu_left"), number(row, "mu_right"))
                if key not in GRID or key in rows:
                    raise ValueError(f"unexpected or duplicate trial {key}")
                rows[key], oracles[key] = inspect(row, key, preload_steps)
            except (TypeError, ValueError, KeyError, OverflowError) as error:
                raise ValueError(f"{path}:{line}: {error}") from error
    if rows.keys() != GRID:
        raise ValueError(f"{path}: missing trials {sorted(GRID - rows.keys())}")
    return rows, oracles


def fixture_ok(row):
    return row["jaw_m_s"] <= SPEED and row["normal_Ns"] <= NORMAL


def summary(rows):
    counts = {metric: sum(r[metric] <= limit for r in rows.values()) for metric, limit in LIMITS.items()}
    worst = {}
    for metric in LIMITS:
        key = max(sorted(rows), key=lambda k: rows[k][metric])
        worst[metric] = {"value": rows[key][metric], "case": key}
    exclusions = {"jaw_only": [], "normal_only": [], "jaw_and_normal": []}
    for key, row in sorted(rows.items()):
        jaw, normal = row["jaw_m_s"] > SPEED, row["normal_Ns"] > NORMAL
        if jaw or normal:
            reason = "jaw_and_normal" if jaw and normal else ("jaw_only" if jaw else "normal_only")
            exclusions[reason].append(key)
    return dict(total=len(rows), pass_by_metric=counts, worst=worst,
                pass_all=sum(all(r[m] <= v for m, v in LIMITS.items()) for r in rows.values()),
                fixture_valid=sum(map(fixture_ok, rows.values())), excluded_cases=exclusions)


def compare(left, right):
    counts = dict(left_wins=0, ties=0, left_losses=0, excluded_left_only=0, excluded_right_only=0, excluded_both=0)
    for key in GRID:
        a, b = left[key], right[key]
        av, bv = fixture_ok(a), fixture_ok(b)
        if av and bv:
            difference = b["relative_m_s"] - a["relative_m_s"]
            outcome = "left_wins" if difference > SPEED else ("left_losses" if difference < -SPEED else "ties")
        else:
            outcome = "excluded_both" if not av and not bv else ("excluded_right_only" if av else "excluded_left_only")
        counts[outcome] += 1
    return dict(total=len(GRID), **counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preload-steps", type=int, choices=(120, 1920), default=120,
                        help="required number of preload steps for every row (default: 120)")
    parser.add_argument("inputs", nargs="+", metavar="NAME=CSV")
    args = parser.parse_args()
    engines, shared_oracles = {}, {}
    try:
        for arg in args.inputs:
            name, path = arg.split("=", 1)
            if not name or name in engines:
                raise ValueError(f"invalid or duplicate engine name {name!r}")
            engines[name], oracles = read_file(path, args.preload_steps)
            if shared_oracles and any(distance(oracles[k], shared_oracles[k]) > 1e-12 for k in GRID):
                raise ValueError(f"{path}: discrete oracles differ across engines by more than 1e-12 m/s")
            if not shared_oracles:
                shared_oracles = oracles
        if len(engines) < 2:
            raise ValueError("at least two engines are required")
    except (ValueError, OSError) as error:
        parser.error(str(error))
    report = dict(preload_steps=args.preload_steps, limits=LIMITS, pair_tie_m_s=SPEED,
                  case_fields=["trial", "angle_deg", "[mu_left,mu_right]"],
                  engines={name: summary(rows) for name, rows in engines.items()},
                  pairs=[dict(left=a, right=b, **compare(engines[a], engines[b]))
                         for a, b in itertools.combinations(engines, 2)])
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
