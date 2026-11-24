"""Windows: python kp-guide-verify.py ORIGINAL_BIN MODIFIED_BIN OUT. Inherit CPU affinity."""
import argparse
import csv
import hashlib
import io
import itertools
import math
import os
from pathlib import Path
import re
import subprocess

GRIP = "study-grip-limit"
CASES = (
    ("shared8", GRIP, ["shared8"], 36, 17),
    ("trace-order", GRIP, ["trace-order"], 720, 12),
    ("islands", GRIP, ["islands"], 480, 6),
    ("island-joints", GRIP, ["island-joints"], 360, 6),
    ("oblique", "study-oblique-grip", [], 48, 33),
    ("oblique-settled", "study-oblique-grip", ["--settled"], 48, 33),
    ("groove", "study-groove-friction", [], 32, 11),
    ("groove-holdout", "study-groove-friction", ["--holdout"], 32, 11),
    ("state", "study-state-hash", ["60", "realtime"], 27, 0),
)
STATE_KEYS = ("stack pyramid heavy arch slope dominoes wall bullet cradle chain buckets balls "
              "pile gripper bowl hills swing drift rail thrust platforms pendulum gyroscope cone crank chain100 beam").split()
ORDER = ("left-first", "right-first")
BASE_MU = ((.5, .5), (.05, .6), (.6, .05), (0, 0))


def check(condition, reason):
    if not condition:
        raise ValueError(reason)


def run(directory, stem, args, coupled, timed, raw, label):
    env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION=str(int(coupled)),
               KP_GRIP_TIMING=str(int(timed)), KP_GRIP_PROFILE="0")
    exe = directory / (stem + ".exe")
    process = subprocess.run([str(exe), *args], capture_output=True, env=env, timeout=900)
    (raw / (label + ".stdout")).write_bytes(process.stdout)
    if process.stderr:
        (raw / (label + ".stderr")).write_bytes(process.stderr)
    return process


def grid(name):
    if name == "shared8":
        loads = [(0.5, 10, d, 0) for d in (0, 8, 9.8, 9.9, 10, 10.1, 10.2, 12, -12)]
        loads += [(0, 10, 10, 0), (.5, 0, 10, 0), (.5, 10, 10, 1)]
        return ["hz", "mu", "jaw_N", "load_N", "open"], {(hz, *p) for hz in (60, 120, 240) for p in loads}
    if name == "trace-order":
        return ["order", "mu", "load_N", "microstep"], {
            (o, mu, force, step) for o in ORDER for mu, force in ((.5, 10), (.5, -10), (0, 10))
            for step in range(1, 121)}
    if name in ("islands", "island-joints"):
        scenes = (("alone", "separated", "touch-right", "touch-left") if name == "islands" else
                  ("alone", "static-static", "soft-shared-anchor"))
        return ["scene", "microstep"], set(itertools.product(scenes, range(1, 121)))
    if name.startswith("oblique"):
        return ["trial", "angle_deg", "mu_left", "mu_right"], {
            (i, a, *mu) for i in range(4) for a in (0, 37, 83) for mu in BASE_MU}
    holdout = name == "groove-holdout"
    angles = (20, 55) if holdout else (30, 45)
    speeds = (-.024, -.002, .024, .065) if holdout else (-.01, 0, .01, .1)
    mus = ((.35, .35), (.12, .8), (.8, .12), (.18, .28)) if holdout else BASE_MU
    return ["angle_deg", "initial_m_s", "mu_left", "mu_right"], {
        (a, s, *mu) for a in angles for s in speeds for mu in mus}


def records(data, rows_expected, width):
    reader = csv.DictReader(io.StringIO(data.decode("ascii")))
    header = reader.fieldnames or []
    check(len(set(header)) == len(header) == width, "incorrect header width")
    rows = list(reader)
    check(len(rows) == rows_expected, f"expected {rows_expected} rows, got {len(rows)}")
    for r in rows:
        check(None not in r and all(r.values()), "ragged or empty CSV field")
        for key, value in r.items():
            if key not in ("scene", "order", "engine"):
                check(math.isfinite(float(value)), "nonfinite " + key)
    return rows


def validate(name, proc, n, width, coupled):
    if name == "state":
        keys = []
        for line in proc.stdout.splitlines():
            match = re.fullmatch(rb"([a-z0-9]+)\s+\S+\s+([a-f0-9]{16})", line)
            check(match is not None, "malformed realtime state hash")
            keys.append(match[1].decode("ascii"))
        check(keys == STATE_KEYS and len(keys) == n and proc.returncode == 0, "incomplete state hash set")
        return
    check(proc.returncode in (0, 1), f"unexpected status {proc.returncode}")
    rs = records(proc.stdout, n, width)
    fields, expected = grid(name)
    seen = {tuple(r[k] if k in ("order", "scene") else float(r[k]) for k in fields) for r in rs}
    check(seen == expected, "incomplete case grid")
    if name == "shared8":
        engine = "kinetica-shared8-coupled" if coupled else "kinetica-shared8-sequential"
        check(all(r["complete"] == "1" and r["engine"] == engine for r in rs), "incomplete shared8 or wrong friction mode")
    if name.startswith("oblique"):
        preload = "1920" if name.endswith("settled") else "120"
        check(all(r["preload_steps"] == preload and r["step_complete"] == "1" for r in rs), "incomplete preload")
    if "pass" in rs[0]:
        check(all(r["pass"] in ("0", "1") for r in rs), "bad pass flag")
        check(proc.returncode == int(any(r["pass"] == "0" for r in rs)), "return code vs pass flags")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("original", type=Path)
    parser.add_argument("modified", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--parity-only", action="store_true")
    options = parser.parse_args()
    a, b, out = (p.resolve() for p in (options.original, options.modified, options.output))
    check(a != b, "identical binary directory")
    dirs = {"A": a, "B": b}
    out.mkdir(parents=True, exist_ok=True)
    raw = out / "raw"
    raw.mkdir(exist_ok=True)
    all_pass = True
    with (out / "parity.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["solver", "case", "expected_rows", "rc_A", "rc_B", "sha256_A", "sha256_B", "pass", "reason"])
        for name, stem, args, n, width in CASES:
            for coupled in ((False,) if name == "state" else (False, True)):
                solver = "coupled" if coupled else "stock"
                pair = [run(dirs[x], stem, args, coupled, False, raw, f"{solver}-{name}-{x}") for x in dirs]
                p, q = pair
                error = ""
                try:
                    check(p.returncode == q.returncode, "exit codes differ")
                    for proc in pair:
                        validate(name, proc, n, width, coupled)
                    check(p.stdout == q.stdout, "raw stdout differs")
                except (ValueError, KeyError) as err:
                    error = str(err)
                w.writerow([solver, name, n, p.returncode, q.returncode,
                            *(hashlib.sha256(proc.stdout).hexdigest() for proc in pair), int(not error), error])
                f.flush()
                all_pass &= not error
    if not all_pass:
        raise SystemExit("Parity gate failed. See parity.csv and raw/.")
    if options.parity_only:
        return
    with (out / "timing.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pair", "slot", "order", "variant", "solver", "frames", "us_per_frame", "end_drift_mm"])
        drift = None
        for block in range(20):
            order = "AB" if block % 2 == 0 else "BA"
            for slot, variant in enumerate(order, 1):
                proc = run(dirs[variant], GRIP, ["shared8"], True, True, raw, f"timing-{block+1:02}-{variant}")
                check(proc.returncode == 0, "timing program failed")
                r, = records(proc.stdout, 1, 4)
                check(r["engine"] == "kinetica-shared8-coupled" and r["frames"] == "360", "wrong timing experiment")
                check(float(r["us_per_frame"]) > 0 and float(r["end_drift_mm"]) >= 0, "invalid timing")
                if drift is None:
                    drift = r["end_drift_mm"]
                check(r["end_drift_mm"] == drift, "loaded trajectory changed during timing")
                w.writerow([block + 1, slot, order, variant, "coupled", r["frames"], r["us_per_frame"], drift])
                f.flush()


if __name__ == "__main__":
    main()
