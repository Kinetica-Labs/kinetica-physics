"""Measure native Box3D SIMD selection on Windows. Run in an affinity-pinned parent process."""
import argparse
import csv
import ctypes
import hashlib
import io
import itertools
import json
import math
import os
from pathlib import Path
import re
import subprocess


VARIANTS = ("auto", "width4", "no-intrinsics")


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def run(args, env, valid=(0,), timeout=300):
    p = subprocess.run(args, env=env, capture_output=True, encoding="utf-8",
                       errors="replace", timeout=timeout)
    if p.returncode not in valid:
        raise RuntimeError(f"{args[0]} exited {p.returncode}\n{p.stdout[-2000:]}\n{p.stderr[-2000:]}")
    return p


def read_cmake(file):
    result = {}
    if file.is_file():
        for line in file.read_text(encoding="utf-8", errors="replace").splitlines():
            key, _, value = line.partition("=")
            if ":" in key and not line.startswith(("#", "//")):
                result[key.split(":")[0]] = value
    return result


def build_libraries(a, env):
    # Build both archives in the same session. The supplied archive separately anchors
    # the new auto configuration to the historically measured 36 trajectories.
    old = read_cmake(a.library.parent.parent / "CMakeCache.txt")
    options = ("CMAKE_C_FLAGS", "CMAKE_C_FLAGS_RELEASE", "CMAKE_CXX_FLAGS", "CMAKE_CXX_FLAGS_RELEASE",
               "BOX3D_VALIDATE", "BOX3D_DOUBLE_PRECISION")
    common = [f"-D{k}={old[k]}" for k in options if k in old]
    libs = {}
    for name, disable in (("auto", "OFF"), ("no-intrinsics", "ON")):
        directory = a.out / ("build-" + name)
        flags = ["cmake", "-S", a.source, "-B", directory, "-G", "MinGW Makefiles",
                 f"-DCMAKE_MAKE_PROGRAM={a.compiler_dir / 'mingw32-make.exe'}",
                 f"-DCMAKE_C_COMPILER={a.compiler_dir / 'clang.exe'}",
                 f"-DCMAKE_CXX_COMPILER={a.compiler_dir / 'clang++.exe'}",
                 "-DCMAKE_BUILD_TYPE=Release", "-DBOX3D_SAMPLES=OFF", "-DBOX3D_UNIT_TESTS=OFF",
                 "-DBOX3D_BENCHMARKS=OFF", "-DBOX3D_PROFILE=OFF", *common,
                 "-DBOX3D_DISABLE_SIMD=" + disable]
        run(flags, env)
        run(["cmake", "--build", directory, "--target", "box3d", "-j", str(a.jobs)], env, timeout=600)
        libs[name], = directory.rglob("libbox3d.a")
    return libs


def compile_adapter(a, env, stem, lib, name):
    exe = a.out / f"{stem}-{name}.exe"
    source = a.project / "validation/engines" / (stem + ".cpp")
    run([a.compiler_dir / "clang++.exe", "-O3", "-DNDEBUG", "-ffp-contract=off", "-std=gnu++17",
         "-I" + str(a.source / "include"), source, Path(__file__).with_suffix(".cpp"),
         lib, "-o", exe], env)
    return exe


def execute(exe, args, env, kind, allow_tolerance_fail=False):
    p = run([exe, *args], dict(env, KP_BOX_FORCE4="1" if kind == "width4" else "0"),
            valid=(0, 1) if allow_tolerance_fail else (0,))
    match = re.search(r"^BOX_WIDTH=(4|8)\s*$", p.stderr, re.MULTILINE)
    if not match or (kind in ("width4", "no-intrinsics") and match[1] != "4"):
        raise ValueError(f"Unexpected Box3D width for {kind}: {p.stderr}")
    return p, int(match[1])


def sweep(exe, env, variant, out):
    p, width = execute(exe, ["shared8"], env, variant, allow_tolerance_fail=True)
    (out / f"grip-{variant}.csv").write_text(p.stdout, encoding="utf-8")
    rows = list(csv.DictReader(io.StringIO(p.stdout)))
    cases = [(0.5, 10., x, 0) for x in (0, 8, 9.8, 9.9, 10, 10.1, 10.2, 12, -12)]
    cases.extend([(0., 10., 10., 0), (.5, 0., 10., 0), (.5, 10., 10., 1)])
    expected = {(hz, *case) for hz in (60, 120, 240) for case in cases}
    seen = set()
    for row in rows:
        key = (int(row["hz"]), float(row["mu"]), float(row["jaw_N"]), float(row["load_N"]), int(row["open"]))
        if row["complete"] != "1" or not all(math.isfinite(float(v)) for k, v in row.items() if k != "engine"):
            raise ValueError(f"Incomplete grip observation: {variant}, {key}")
        seen.add(key)
    if seen != expected or len(rows) != len(expected) or p.returncode != int(any(r["pass"] == "0" for r in rows)):
        raise ValueError(f"Invalid grip grid or exit status: {variant}")
    return p.stdout, width


def timings(a, env, executables, pyramid):
    timed = dict(env, KP_GRIP_TIMING="1")
    if pyramid:
        timed["BENCH"] = "7"
    filename = "pyramid-timing.csv" if pyramid else "grip-timing.csv"
    with (a.out / filename).open("w", newline="", encoding="utf-8") as dest:
        writer = csv.writer(dest)
        writer.writerow(["block", "slot", "variant", "width", "frames", "us_per_frame",
                         "max_frame_us" if pyramid else "end_drift_mm"])
        orders = list(itertools.permutations(VARIANTS))
        for block in range(12):
            for slot, name in enumerate(orders[block % len(orders)]):
                p, width = execute(executables[name], ["8"] if pyramid else ["shared8"], timed, name)
                if pyramid:
                    # The historical adapter prints one line, rounded to 0.01 ms, in Russian.
                    match = re.search(r"140:\s*([\d.]+)\s+.*?([\d.]+)\s*$", p.stdout.strip())
                    if not match:
                        raise ValueError(f"Incomplete pyramid output: {p.stdout}")
                    frames, us, metric = 60, float(match[1]) * 1000, float(match[2]) * 1000
                else:
                    row, = csv.DictReader(io.StringIO(p.stdout))
                    frames, us, metric = int(row["frames"]), float(row["us_per_frame"]), float(row["end_drift_mm"])
                    if frames != 360:
                        raise ValueError(f"Unexpected timed frame count: {frames}")
                if not math.isfinite(us) or us <= 0 or not math.isfinite(metric):
                    raise ValueError("Invalid timing observation")
                writer.writerow([block, slot, name, width, frames, us, metric])
                dest.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "library", "out", "compiler-dir", "project"):
        parser.add_argument("--" + name, type=lambda s: Path(s).resolve(), required=True)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--pyramid", action="store_true", help="also time 140 cubes, 7 levels, 8 internal substeps")
    a = parser.parse_args()
    if os.name != "nt":
        parser.error("This research runner requires the designated Windows PC")
    env = dict(os.environ, KP_GRIP_TIMING="0", KP_GRIP_PROFILE="0", KP_THREADS="1")
    env["PATH"] = str(a.compiler_dir) + os.pathsep + env["PATH"]
    env.pop("BENCH", None)
    proc, system = ctypes.c_size_t(), ctypes.c_size_t()
    if not ctypes.windll.kernel32.GetProcessAffinityMask(ctypes.c_void_p(-1), ctypes.byref(proc), ctypes.byref(system)):
        raise OSError("Cannot read CPU affinity")
    if not proc.value or proc.value & (proc.value - 1):
        parser.error("Pin the calling PowerShell process to one logical CPU before measuring")
    a.out.mkdir(parents=True, exist_ok=True)
    libs = build_libraries(a, env)
    summary = dict(cpu_affinity=hex(proc.value), comparison_preload_s=.25,
                   library_sha256={k: digest(p) for k, p in dict(libs, historical=a.library).items()},
                   compiler=run([a.compiler_dir / "clang.exe", "--version"], env).stdout.splitlines()[0],
                   width={}, same_rows_as_auto={}, adapter_sha256={})
    exe = {k: compile_adapter(a, env, "grip_box3d", lib, k) for k, lib in dict(libs, historical=a.library).items()}
    exe["width4"] = exe["auto"]
    raw = {}
    for v in ("auto", "historical", "width4", "no-intrinsics"):
        raw[v], summary["width"][v] = sweep(exe[v], env, v, a.out)
        summary["adapter_sha256"][v] = digest(exe[v])
        summary["same_rows_as_auto"][v] = raw[v] == raw["auto"]
    (a.out / "measurements.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if raw["historical"] != raw["auto"]:
        raise RuntimeError("Rebuilt auto archive differs from the supplied archive on the grip sweep; inspect both CSVs")
    timings(a, env, exe, False)
    if a.pyramid:
        pyramid = {k: compile_adapter(a, env, "box3d", lib, k) for k, lib in libs.items()}
        pyramid["width4"] = pyramid["auto"]
        timings(a, env, pyramid, True)


if __name__ == "__main__":
    main()
