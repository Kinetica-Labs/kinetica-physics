"""Compare already built Windows grip studies in balanced order using the common physical oracle."""
import argparse
import csv
import ctypes
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def run(file, args, env):
    proc = subprocess.run([file, *args], capture_output=True, env=env, timeout=180)
    if proc.returncode not in (0, 1):
        raise RuntimeError(f"{file.name} {args}: {proc.returncode}: {proc.stderr[-400:]!r}")
    return list(csv.DictReader(proc.stdout.decode().splitlines()))


def write(file, records):
    with file.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, records[0])
        writer.writeheader()
        writer.writerows(records)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--blocks", type=int, default=24)
    ap.add_argument("--shuffle-each", action="store_true", help="Draw a fresh permutation in every timing block")
    ap.add_argument("--seed", type=int, default=20261123)
    ap.add_argument("--extra-arg", nargs=2, action="append", default=[], metavar=("LABEL", "ARG"),
                    help="Append a study argument for one build, e.g. --extra-arg bullet-f64 8")
    ap.add_argument("program", nargs="+", help="label=full path to the native executable")
    args = ap.parse_args()
    programs = {key: Path(path) for key,path in (item.split("=", 1) for item in args.program)}
    arguments = {key: ["shared8"] for key in programs}
    for key, value in args.extra_arg:
        if key not in arguments:
            ap.error(f"Unknown build label: {key}")
        arguments[key].append(value)
    n = len(programs)
    if n < 2 or args.blocks <= 0 or (not args.shuffle_each and args.blocks % (2*n)):
        raise ValueError("Use an even number of complete rotations")
    if not ctypes.WinDLL("kernel32", use_last_error=True).SetProcessAffinityMask(ctypes.c_void_p(-1), ctypes.c_size_t(0x100)):
        raise ctypes.WinError(ctypes.get_last_error())
    env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION="1", KP_GRIP_PROFILE="0", KP_GRIP_TIMING="0")
    args.out.mkdir(parents=True, exist_ok=True)
    loads = {(0.5,10,f,0) for f in (0,8,9.8,9.9,10,10.1,10.2,12,-12)}
    loads.update({(0,10,10,0),(.5,0,10,0),(.5,10,10,1)})
    expected = {(h,*load) for h in (60,120,240) for load in loads}
    sweep = []
    for name,file in programs.items():
        rows = run(file, arguments[name], env)
        keys = {(int(r["hz"]),float(r["mu"]),float(r["jaw_N"]),float(r["load_N"]),int(r["open"])) for r in rows}
        if keys != expected or len(rows) != 36 or any(r["complete"] != "1" for r in rows):
            raise RuntimeError(f"{name}: incomplete run")
        sweep.extend(dict(variant=name, **r) for r in rows)
    write(args.out/"accuracy.csv", sweep)
    order = list(programs)
    rng = random.Random(args.seed)
    records = []
    for block in range(args.blocks):
        if args.shuffle_each or block % (2*n) == 0:
            rng.shuffle(order)
        if args.shuffle_each:
            sequence = order.copy()
        else:
            seq = order if block % (2*n) < n else order[::-1]
            sequence = [seq[(block+slot)%n] for slot in range(n)]
        for slot, name in enumerate(sequence):
            rows = run(programs[name], arguments[name], dict(env, KP_GRIP_TIMING="1"))
            if len(rows) != 1 or rows[0]["frames"] != "360" or float(rows[0]["us_per_frame"]) <= 0:
                raise ValueError(f"{name}: incomplete timing")
            records.append(dict(block=block+1, slot=slot+1, variant=name, **rows[0]))
        write(args.out/"timing.csv", records)
    (args.out/"inputs.json").write_text(json.dumps({
        "sha256": {k:digest(v) for k,v in programs.items()}, "arguments": arguments, "host": os.environ.get("COMPUTERNAME"),
        "threads": 1, "sleep": False, "coupled_friction": True, "steps_per_frame": 8,
        "frames": 360, "seed": args.seed, "independent_order_per_block": args.shuffle_each,
        "affinity": "0x100", "blocks": args.blocks}, indent=2))


if __name__ == "__main__":
    main()
