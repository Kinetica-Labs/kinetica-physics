"""Factorial Windows experiment: carry validated geometry and certify normal impulses independently."""
import argparse
import csv
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import random
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, rows[0])
        writer.writeheader()
        writer.writerows(rows)


def sample(exe, args, env, expected):
    run = subprocess.run([exe, *args], capture_output=True, env=env, timeout=240)
    if run.returncode not in expected or not run.stdout:
        raise RuntimeError(f"{exe} {args}: exit {run.returncode} {run.stderr[-500:]!r}")
    return run.stdout


def main():
    parser = argparse.ArgumentParser()
    for name in ("src", "build", "overlay", "out"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--balance", type=Path, help="Test a validated near-equilibrium trial and in-place manifold refresh")
    args = parser.parse_args()
    out, archive = args.out, args.build / "src/libkinetica-physics.a"
    out.mkdir(exist_ok=True, parents=True)
    env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION="1", KP_GRIP_PROFILE="0", KP_GRIP_TIMING="0")
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    if not api.SetProcessAffinityMask(ctypes.c_void_p(-1), ctypes.c_size_t(0x100)):
        raise ctypes.WinError(ctypes.get_last_error())
    flags = ["-std=c++17", "-O3", "-DNDEBUG", "-ffp-contract=off", "-g", "-gcodeview"]
    sources = {"carry": "src/collision/PairFinder.cpp", "normal": "src/dynamics/time_stepping/IslandNormals.cpp"}
    if args.balance:
        sources["manifold"] = "src/collision/ManifoldMemory.cpp"
    files = {key: args.overlay / path for key, path in sources.items()}
    if args.balance:
        base = (args.src / "src/dynamics/time_stepping/SoftStep.cpp").read_text(encoding="utf8")
        marker = "constexpr int kRechecks"
        states = "StepArray<SolverBody> states = solverBodiesOf(bodies);"
        if base.count(marker) != 1 or base.count(states) != 1:
            raise ValueError("Unexpected solver entry point")
        source = base.replace(marker, args.balance.read_text(encoding="utf8") + "\n" + marker)
        source = source.replace(states, states + "\n    if (tryBalancedStep(*this, bodies, pairs, joints, gravity, h, contactMemory, jointMemory, states)) return;")
        files["balance"] = out / "SoftStep-trial.cpp"
        files["balance"].write_text(source, encoding="utf8")
    objects = {}
    for name, path in files.items():
        obj = out / (name + ".obj")
        subprocess.run(["clang++", *flags, "-I" + str(args.overlay / "include"), "-I" + str(args.src / "include"),
                        "-c", path, "-o", obj], check=True)
        objects[name] = obj
    variants = {"stock": [], "carry": [objects["carry"]], "normal": [objects["normal"]], "both": list(objects.values())}
    if args.balance:
        direct = [objects["carry"], objects["manifold"]]
        variants = {"stock": [], "inplace": direct, "inplace-normal": direct + [objects["normal"]], "balanced": list(objects.values())}
    studies = {"grip": "study-grip-limit", "oblique": "study-oblique-grip", "roll": "study-slip-to-roll"}
    binaries = {}
    checks = {"grid": ("grip", ["shared8"], (0, 1)), "islands": ("grip", ["islands"], (0,)),
              "joints": ("grip", ["island-joints"], (0,)), "order": ("grip", ["trace-order"], (0,)),
              "oblique": ("oblique", [], (0,)), "roll": ("roll", [], (0,))}
    accuracy, raw, parity = [], {}, []
    for name, obj in variants.items():
        tools = {}
        for fixture, target in studies.items():
            study = args.build / "validation/studies"
            adapter = study / "CMakeFiles" / (target + ".dir") / "objects.a"
            exe = study / (target + ".exe") if name == "stock" else out / (name + "-" + fixture + ".exe")
            if obj:
                subprocess.run(["clang++", *flags, "-Wl,--whole-archive", adapter, "-Wl,--no-whole-archive",
                                *obj, archive, "-lpthread", "-o", exe], check=True)
            tools[fixture] = exe
        binaries[name] = tools["grip"]
        for key, (fixture, argv, expected) in checks.items():
            data = sample(tools[fixture], argv, env, expected)
            if name == "stock":
                raw[key] = data
            parity.append(dict(variant=name, scene=key, identical=int(raw[key] == data),
                               sha256=hashlib.sha256(data).hexdigest(), lines=len(data.splitlines())))
            (out / f"{name}-{key}.csv").write_bytes(data)
            if key == "grid":
                rows = list(csv.DictReader(data.decode().splitlines()))
                if len(rows) != 36 or len({(r["hz"],r["mu"],r["jaw_N"],r["load_N"],r["open"]) for r in rows}) != 36:
                    raise ValueError("incomplete grid")
                accuracy.extend(dict(variant=name, **r) for r in rows)
    save(out / "parity.csv", parity)
    save(out / "accuracy.csv", accuracy)
    orders = list(itertools.permutations(variants))
    random.Random(20261123).shuffle(orders)
    records = []
    for block, order in enumerate(orders, 1):
        for slot, name in enumerate(order, 1):
            data = sample(binaries[name], ["shared8"], dict(env, KP_GRIP_TIMING="1"), (0,))
            row, = csv.DictReader(data.decode().splitlines())
            if row["frames"] != "360":
                raise ValueError("unexpected frame count")
            records.append(dict(block=block, slot=slot, variant=name, **row))
        save(out / "timing.csv", records)
    (out / "inputs.json").write_text(json.dumps({
        "baseline_archive_sha256": sha(archive), "input_sha256": {key: sha(path) for key,path in files.items()},
        "baseline_source_sha256": {p: sha(args.src / p) for p in sources.values()},
        "binaries_sha256": {k: sha(v) for k,v in binaries.items()}, "env": {k:v for k,v in env.items() if k.startswith("KP_")},
        "compiler": subprocess.check_output(["clang++","--version"], text=True).splitlines()[0],
        "seed": 20261123, "flags": flags, "affinity": "0x100"}, indent=2))


if __name__ == "__main__":
    main()
