"""Compare the native shipping binary, compiler AVX2, and four explicit response columns on Windows."""
import csv
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import random
import shutil
import subprocess

out = Path(__file__).resolve().parent
lab = out.parent.parent
src, build = lab / "active-rows-src", lab / "active-rows-build"
base_archive = build / "src/libkinetica-physics.a"
study = build / "validation/studies"
source = src / "src/dynamics/event_driven/ContactSystem.cpp"
oracle = out.parent / "memory-cpu/matrix-oracle.cpp"
env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION="1", KP_GRIP_PROFILE="0", KP_GRIP_TIMING="0")
flags = ["-O3", "-DNDEBUG", "-g", "-gcodeview", "-ffp-contract=off"]
inc = "-I" + str(src / "include")

def run(command, **kw):
    return subprocess.run(list(map(str, command)), check=True, **kw)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def build_variant(key):
    work = out / key
    work.mkdir(exist_ok=True)
    archive = base_archive
    exe = study / "study-grip-limit.exe"
    if key != "stock":
        code = source.read_text(encoding="utf8")
        if key == "packed-four":
            first = code.index("        for (size_t j : onBody[index]) {")
            last = code.index("\n    }\n    StepArray<Entry> merged;", first)
            code = "#include <immintrin.h>\n" + code[:first] + (out / "packed-response.inc").read_text(encoding="utf8") + code[last:]
        file = work / "ContactSystem.cpp"
        file.write_text(code, encoding="utf8")
        obj = work / "ContactSystem.cpp.obj"
        run(["clang++", *flags, inc, "-mavx2", "-std=c++17", "-c", file, "-o", obj])
        archive = work / "libkinetica-physics.a"
        shutil.copy2(base_archive, archive)
        assert run(["llvm-ar", "t", archive], capture_output=True).stdout.splitlines().count(obj.name.encode()) == 1
        run(["llvm-ar", "rcs", archive, obj])
        exe = work / "grip.exe"
        adapter = study / "CMakeFiles/study-grip-limit.dir/objects.a"
        run(["clang++", *flags, "-Wl,--whole-archive", adapter, "-Wl,--no-whole-archive", "-o", exe,
             "-Wl,--major-image-version,0,--minor-image-version,0", archive, "-lpthread"])
    matrix = work / "matrix.exe"
    run(["clang++", *flags, inc, "-std=c++17", oracle, archive, "-lpthread", "-o", matrix])
    measured = run([matrix], capture_output=True)
    (out / (key + "-matrix.csv")).write_bytes(measured.stdout)
    (out / (key + "-energy.csv")).write_bytes(measured.stderr)
    physical = subprocess.run([str(exe), "shared8"], env=env, capture_output=True)
    rows = list(csv.DictReader(physical.stdout.decode().splitlines()))
    assert len(rows) == 36 and all(r["complete"] == "1" for r in rows), physical.stderr
    (out / (key + "-grid.csv")).write_bytes(physical.stdout)
    return exe

def main():
    variants = {key: build_variant(key) for key in ("stock", "compiler-avx2", "packed-four")}
    parity = {}
    for key in variants:
        parity[key] = {s: (out / (key + s)).read_bytes() == (out / ("stock" + s)).read_bytes()
                       for s in ("-grid.csv", "-matrix.csv", "-energy.csv")}
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    if not api.SetProcessAffinityMask(ctypes.c_void_p(-1), ctypes.c_size_t(0x100)):
        raise ctypes.WinError(ctypes.get_last_error())
    orders = list(itertools.permutations(variants)) * 2
    random.Random(20261123).shuffle(orders)
    with (out / "simd-time.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["block", "slot", "variant", "frames", "us_frame", "drift_mm"])
        for block, order in enumerate(orders, 1):
            for slot, key in enumerate(order, 1):
                attempt = run([variants[key], "shared8"], env=dict(env, KP_GRIP_TIMING="1"), capture_output=True)
                row, = csv.DictReader(attempt.stdout.decode().splitlines())
                assert row["frames"] == "360"
                writer.writerow([block, slot, key, row["frames"], row["us_per_frame"], row["end_drift_mm"]])
            stream.flush()
    (out / "simd-inputs.json").write_text(json.dumps({
        "exe_sha256": {k: sha(v) for k, v in variants.items()}, "parity": parity,
        "source_sha256": sha(source), "parent_archive_sha256": sha(base_archive),
        "candidate_sha256": sha(out / "packed-response.inc"), "affinity": "0x100", "seed": 20261123,
        "flags": flags, "variant_flags": ["-mavx2"], "solver_threads": 1,
        "cpu_compiler": run(["clang++", "--version"], text=True, capture_output=True).stdout.splitlines()[0],
    }, indent=2))

if __name__ == "__main__":
    main()
