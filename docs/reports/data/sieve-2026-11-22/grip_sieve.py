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
            numeric = (v for name, v in row.items() if name != "order")
            if not all(math.isfinite(float(v)) for v in numeric):
                raise ValueError(f"{path.name}: nonfinite value at {key}")
            values[key] = row
    if values.keys() != expected:
        raise ValueError(f"{path.name}: case grid mismatch ({len(values)} rows)")
    return values


def measure(directory, output, hashes):
    grip_status = record(directory, "study-grip-limit", ["shared8"], output / "grip.csv", hashes)
    groups = read_cases(output / "grip.csv")
    cases = {key: group["kinetica-shared8"] for key, group in groups.items()}
    if any(set(group) != {"kinetica-shared8"} for group in groups.values()):
        raise ValueError("mixed solver modes")
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
    return reasons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="новый каталог для исходных строк и verdict.json")
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
            entry.update(metrics)
            if parent is None and (metrics["roll_pass"] != 8 or metrics["max_balance_error_Ns"] > BALANCE_NS):
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
