"""Отберите сборки по траектории захвата, балансу импульса и переходу от скольжения к качению."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from grip_accuracy import admissible, read_cases


# Все сборки получают одни и те же шаги и силы. Порог регрессии дан в миллиметрах.
PATH_TIE_MM = 0.001
HOLD_MM = 0.5
BALANCE_NS = 1e-7
SPEED_M_S = 1e-6


def record(directory, program, args, output, manifest):
    exe = directory / (program + (".exe" if os.name == "nt" else ""))
    manifest[program] = hashlib.sha256(exe.read_bytes()).hexdigest()
    env = dict(os.environ, KP_THREADS="1", KP_GRIP_TIMING="0", KP_GRIP_PROFILE="0")
    with output.open("wb") as data:
        run = subprocess.run([str(exe.resolve()), *args], stdout=data, stderr=subprocess.PIPE,
                             env=env, timeout=180, check=False)
    if run.returncode not in (0, 1):
        raise ValueError(f"{program} exited {run.returncode}: {run.stderr.decode(errors='replace')[:300]}")
    return run.returncode


def indexed(path, fields, expected):
    """Сопоставить строки с физическими условиями. Потерянный или лишний кадр делает опыт неполным."""
    values = {}
    with path.open(newline="", encoding="utf-8-sig") as source:
        for row in csv.DictReader(source):
            key = tuple(row[field] for field in fields)
            if key in values:
                raise ValueError(f"{path.name}: duplicate {key}")
            numeric = (v for name, v in row.items() if name not in ("order", "scene"))
            if not all(math.isfinite(float(v)) for v in numeric):
                raise ValueError(f"{path.name}: nonfinite value at {key}")
            values[key] = row
    if values.keys() != expected:
        raise ValueError(f"{path.name}: case grid mismatch ({len(values)} rows)")
    return values


def measure_groove(directory, output, hashes, holdout):
    if holdout:
        angles, speeds = (20, 55), (-0.024, -0.002, 0.024, 0.065)
        coatings = ((0.35, 0.35), (0.12, 0.8), (0.8, 0.12), (0.18, 0.28))
    else:
        angles, speeds = (30, 45), (-0.01, 0, 0.01, 0.1)
        coatings = ((0.5, 0.5), (0.05, 0.6), (0.6, 0.05), (0, 0))
    grid = {(str(a), f"{s:.3g}", f"{l:.3g}", f"{r:.3g}") for a in angles for s in speeds for l, r in coatings}
    name = "groove-holdout" if holdout else "groove"
    path = output / (name + ".csv")
    status = record(directory, "study-groove-friction", ["--holdout"] if holdout else [], path, hashes)
    records = indexed(path, ("angle_deg", "initial_m_s", "mu_left", "mu_right"), grid)
    errors = [max(abs(float(row[k]) - float(row["oracle_" + k])) for k in ("left_m_s", "right_m_s"))
              for row in records.values()]
    passed = sum(err <= SPEED_M_S and float(row["rest_m_s"]) <= SPEED_M_S
                 for err, row in zip(errors, records.values()))
    if status != int(passed != len(records)):
        raise ValueError(f"{name}: inconsistent exit code")
    return {name + "_pass": passed, name + "_max_error_m_s": max(errors)}


def measure_islands(directory, output, hashes):
    scenes = ("alone", "separated", "touch-right", "touch-left")
    steps = range(1, 121)
    path = output / "islands.csv"
    status = record(directory, "study-grip-limit", ["islands"], path, hashes)
    grid = {(scene, str(step)) for scene in scenes for step in steps}
    readings = indexed(path, ("scene", "microstep"), grid)
    worst_path = worst_impulse = 0.0
    for (scene, step), row in readings.items():
        reference = readings["alone", step]
        path_difference = float(row["drop_mm"]) - float(reference["drop_mm"])
        expected_impulse = 10.0 / 480 if scene.startswith("touch-") else 0.0
        worst_path = max(worst_path, abs(path_difference))
        worst_impulse = max(worst_impulse, abs(float(row["remote_up_Ns"]) - expected_impulse))
    passed = worst_path <= PATH_TIE_MM and worst_impulse <= 1e-6
    if status != int(not passed):
        raise ValueError("independent islands: inconsistent exit code")
    return dict(islands_pass=passed, islands_max_difference_mm=worst_path,
                islands_max_reaction_error_Ns=worst_impulse)


def measure_oblique(directory, output, hashes):
    path = output / "oblique.csv"
    status = record(directory, "study-oblique-grip", [], path, hashes)
    coatings = ((0.5, 0.5), (0.05, 0.6), (0.6, 0.05), (0, 0))
    grid = {(str(i), str(a), f"{l:.3g}", f"{r:.3g}") for i in range(4)
            for a in (0, 37, 83) for l, r in coatings}
    rows = indexed(path, ("trial", "angle_deg", "mu_left", "mu_right"), grid)
    maxima = dict(pole_m_s=0., relative_m_s=0., jaw_m_s=0., normal_Ns=0.)
    original_passes = full_passes = 0
    for row in rows.values():
        def tangent(name, side):
            return complex(float(row[f"{name}_{side}_y"]), float(row[f"{name}_{side}_z"]))
        sides = ("left", "right")
        pole = max(abs(tangent("observed", s) - tangent("oracle", s)) for s in sides)
        jaw = max(abs(tangent("jaw", s)) for s in sides)
        relative = max(abs(tangent("observed", s) - tangent("jaw", s) - tangent("oracle", s)) for s in sides)
        normal = max(abs(float(row[f"normal_{s}_Ns"]) - 10 / 480) for s in sides)
        if row["preload_steps"] != "120" or row["step_complete"] != "1":
            raise ValueError("oblique grip: incomplete physical time")
        for key, value in zip(maxima, (pole, relative, jaw, normal)):
            maxima[key] = max(maxima[key], value)
        passed = pole <= SPEED_M_S and normal <= BALANCE_NS
        full_pass = passed and max(relative, jaw) <= SPEED_M_S
        if int(row["pass"]) != int(full_pass):
            raise ValueError("oblique grip: inconsistent relative slip, jaw speed or load")
        original_passes += passed
        full_passes += full_pass
    if status != int(full_passes != len(grid)):
        raise ValueError("oblique grip: inconsistent exit code")
    return dict(oblique_pass=full_passes, oblique_max=maxima, oblique_pole_pass=original_passes)


def measure(directory, output, hashes):
    grip_status = record(directory, "study-grip-limit", ["shared8"], output / "grip.csv", hashes)
    groups = read_cases(output / "grip.csv")
    suffix = "coupled" if os.environ.get("KP_COUPLED_FRICTION") == "1" else "sequential"
    engine = f"kinetica-shared8-{suffix}"
    if any(set(group) != {engine} for group in groups.values()):
        raise ValueError(f"expected a single friction mode: {engine}")
    cases = {key: group[engine] for key, group in groups.items()}
    if any(row["complete"] != "1" or row["pass"] not in ("0", "1") or
           not all(math.isfinite(float(v)) for k, v in row.items() if k != "engine") for row in cases.values()):
        raise ValueError("incomplete grip observation")
    grip_passes = sum(row["pass"] == "1" for row in cases.values())
    if grip_status != int(grip_passes != len(cases)):
        raise ValueError("inconsistent grip exit code")

    trace_status = record(directory, "study-grip-limit", ["trace-order"], output / "jaws.csv", hashes)
    trace_grid = {(order, mu, force, str(i)) for order in ("left-first", "right-first")
                  for mu, force in (("0.50", "10.0"), ("0.50", "-10.0"), ("0.00", "10.0"))
                  for i in range(1, 121)}
    trace = indexed(output / "jaws.csv", ("order", "mu", "load_N", "microstep"), trace_grid)
    balance = max(abs(float(r["balance_error_Ns"])) for r in trace.values())
    drift = max(abs(float(r["relative_drop_mm"])) for k, r in trace.items() if k[1] == "0.50")

    roll_status = record(directory, "study-slip-to-roll", [], output / "roll.csv", hashes)
    roll_grid = {(mu, speed) for mu in ("0", "0.5") for speed in ("-0.01", "0", "0.01", "0.1")}
    roll = indexed(output / "roll.csv", ("mu", "initial_m_s"), roll_grid)
    def speed_error(row):
        return max(abs(float(row[k]) - float(row["oracle_" + k])) for k in ("centre_m_s", "slip_m_s"))
    errors = [speed_error(row) for row in roll.values()]
    roll_passes = sum(error <= SPEED_M_S for error in errors)
    if roll_status != int(roll_passes != len(roll)) or trace_status != int(balance > BALANCE_NS):
        raise ValueError("inconsistent oracle exit code")
    metrics = dict(grip_pass=grip_passes, roll_pass=roll_passes, max_speed_error_m_s=max(errors),
                   max_balance_error_Ns=balance, max_hold_mm_both_orders=drift)
    return cases, metrics


def decide(parent, candidate, metrics):
    if parent.keys() != candidate.keys():
        raise ValueError("different grip conditions")
    losses, gains, lost_passes = 0, 0, 0
    for key, row in candidate.items():
        change = float(row["max_path_error_mm"]) - float(parent[key]["max_path_error_mm"])
        losses += change > PATH_TIE_MM
        gains += change < -PATH_TIE_MM
        lost_passes += parent[key]["pass"] == "1" and row["pass"] != "1"
    reasons = []
    if losses or lost_passes: reasons.append(f"grip regressions: {losses} paths, {lost_passes} previously passing")
    if not gains: reasons.append("no resolved trajectory improvement")
    if not all(map(admissible, candidate.values())): reasons.append("jaw reaction outside tolerance")
    if metrics["max_hold_mm_both_orders"] > HOLD_MM: reasons.append("hold drift above 0.5 mm")
    if metrics["max_balance_error_Ns"] > BALANCE_NS: reasons.append("impulse balance")
    if metrics["roll_pass"] != 8: reasons.append("slip-to-roll oracle")
    if metrics.get("islands_pass") is False: reasons.append("independent islands affect the grip")
    if metrics.get("oblique_pass", 48) != 48: reasons.append("oblique slip or jaw reaction outside tolerance")
    for name in ("groove", "groove-holdout"):
        if name + "_pass" in metrics and metrics[name + "_pass"] != 32:
            reasons.append(name + " contact-speed oracle")
    return reasons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="новый каталог для исходных строк и verdict.json")
    parser.add_argument("--groove", action="store_true", help="проверить также два набора по 32 опыта с V-образным жёлобом")
    parser.add_argument("--islands", action="store_true", help="проверить независимость захвата от удалённых тел")
    parser.add_argument("--oblique", action="store_true", help="проверить скорости обоих полюсов шара, губок и их реакции")
    parser.add_argument("builds", type=Path, nargs="+", help="каталоги двух приборов, первым идёт исходный")
    args = parser.parse_args()
    if len({p.resolve().name.casefold() for p in args.builds}) != len(args.builds):
        parser.error("use distinct build directory names")
    args.out.mkdir(parents=True, exist_ok=False)
    parent, accepted = None, None
    log = []
    incomplete = False
    for directory in args.builds:
        name = directory.resolve().name
        output = args.out / name
        output.mkdir()
        entry = dict(build=name, compared_to=accepted, sha256={})
        try:
            cases, metrics = measure(directory, output, entry["sha256"])
            if args.groove:
                for holdout in (False, True):
                    metrics.update(measure_groove(directory, output, entry["sha256"], holdout))
            if args.islands:
                metrics.update(measure_islands(directory, output, entry["sha256"]))
            if args.oblique:
                metrics.update(measure_oblique(directory, output, entry["sha256"]))
            entry.update(metrics)
            if parent is None and (metrics["roll_pass"] != 8 or metrics["max_balance_error_Ns"] > BALANCE_NS
                                   or metrics.get("islands_pass") is False):
                raise ValueError("baseline failed an independent control")
            reasons = decide(parent, cases, metrics) if parent is not None else []
            entry.update(decision="reject" if reasons else "keep" if parent is not None else "baseline", reasons=reasons)
            if not reasons:
                parent, accepted = cases, name
        except (OSError, ValueError, KeyError, TypeError, subprocess.TimeoutExpired) as error:
            entry.update(decision="incomplete", reasons=[str(error)])
            incomplete = True
        log.append(entry)
        result = dict(reference=accepted, runs=log)
        (args.out / "verdict.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        if parent is None:
            break
    print(json.dumps(result, indent=2))
    return int(incomplete)


if __name__ == "__main__":
    sys.exit(main())
