"""Replay the common grasp on Windows at fixed public steps and several Bullet iteration counts."""
import argparse
import csv
import ctypes
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(command, timing):
    env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION="1", KP_GRIP_PROFILE="0",
               KP_GRIP_TIMING="1" if timing else "0")
    run = subprocess.run(command, env=env, capture_output=True, timeout=90)
    values = list(csv.DictReader(run.stdout.decode().splitlines()))
    if run.returncode not in ((0,) if timing else (0, 1)) or len(values) != (1 if timing else 36):
        raise RuntimeError(f"{command} returned {run.returncode}: {run.stderr.decode(errors='replace')}")
    return values


def save(path, records):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, records[0])
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser()
    for name in ("source", "float-lib", "double-lib", "out", "kinetica", "rp3d", "box3d"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(exist_ok=True, parents=True)
    adapter = Path(__file__).resolve().parents[2] / "engines/grip_bullet.cpp"
    programs = {k: [str(getattr(args, k)), "shared8"] for k in ("kinetica", "rp3d", "box3d")}
    libs_used = {}
    for precision, folder in (("f32", args.float_lib), ("f64", args.double_lib)):
        libs = [folder / ("lib" + x + ".a") for x in ("BulletDynamics", "BulletCollision", "LinearMath")]
        libraries = [str(p) for p in libs]
        exe = args.out / ("bullet-" + precision + ".exe")
        flags = ["-DBT_USE_DOUBLE_PRECISION"] if precision == "f64" else []
        subprocess.run(["clang++", "-O3", "-DNDEBUG", "-ffp-contract=off", "-std=c++17", *flags,
                        "-I" + str(args.source / "src"), str(adapter), *libraries, "-o", str(exe)], check=True)
        libs_used[precision] = {p.name: sha(p) for p in libs}
        for it in (1, 2, 4, 8, 10):
            programs[f"bullet-{precision}-i{it}"] = [str(exe), "shared8", str(it)]
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    if not api.SetProcessAffinityMask(ctypes.c_void_p(-1), ctypes.c_size_t(0x100)):
        raise ctypes.WinError(ctypes.get_last_error())
    loads = {(0.5, 10, force, 0) for force in (0, 8, 9.8, 9.9, 10, 10.1, 10.2, 12, -12)}
    loads.update({(0, 10, 10, 0), (0.5, 0, 10, 0), (0.5, 10, 10, 1)})
    expected = {(hz, *load) for hz in (60, 120, 240) for load in loads}
    sweep = []
    for label, command in programs.items():
        current = rows(command, False)
        keys = {(int(r["hz"]), float(r["mu"]), float(r["jaw_N"]), float(r["load_N"]), int(r["open"])) for r in current}
        assert keys == expected and all(r["complete"] == "1" for r in current)
        sweep.extend(dict(variant=label, **r) for r in current)
    save(args.out / "accuracy.csv", sweep)
    names = list(programs)
    random.Random(20261123).shuffle(names)
    records = []
    for block in range(2 * len(names)):
        order = names if block < len(names) else list(reversed(names))
        for slot in range(len(order)):
            label = order[(block + slot) % len(order)]
            r, = rows(programs[label], True)
            assert r["frames"] == "360" and float(r["us_per_frame"]) > 0
            records.append(dict(block=block + 1, slot=slot + 1, variant=label, **r))
        save(args.out / "timing.csv", records)
    (args.out / "inputs.json").write_text(json.dumps({
        "program_sha256": {k: sha(Path(v[0])) for k, v in programs.items()},
        "library_sha256": libs_used, "adapter_sha256": sha(adapter),
        "oracle_sha256": sha(adapter.parent.parent / "studies/GripLimit.h"),
        "compiler": subprocess.check_output(["clang++", "--version"], text=True).splitlines()[0],
        "steps_per_frame": 8, "sleep": False, "affinity_mask": "0x100", "frames_per_timing": 360,
        "ordering": "every configuration in every slot twice, one cycle reversed",
        "bullet_iterations": [1,2,4,8,10], "seed": 20261123,
    }, indent=2))


if __name__ == "__main__":
    main()
