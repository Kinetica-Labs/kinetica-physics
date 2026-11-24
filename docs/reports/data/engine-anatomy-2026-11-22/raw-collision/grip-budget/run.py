"""Run on the Windows stand, beside budget.cpp in raw-collision/grip-budget."""
import csv
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess

out = Path(__file__).resolve().parent
lab = out.parent.parent
src = lab / "active-rows-src"
archive = lab / "active-rows-build/src/libkinetica-physics.a"
exe = out / "grip-budget.exe"
flags = ["-O3", "-DNDEBUG", "-g", "-gcodeview", "-ffp-contract=off", "-std=c++17"]
subprocess.run(["clang++", *flags, "-I" + str(src / "include"), "-I" + str(src / "validation/studies"),
                str(out / "budget.cpp"), str(archive), "-lpthread", "-o", str(exe)], check=True)
mask = 0x100
kernel = ctypes.WinDLL("kernel32", use_last_error=True)
kernel.GetCurrentProcess.restype = ctypes.c_void_p
kernel.SetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
if not kernel.SetProcessAffinityMask(kernel.GetCurrentProcess(), mask):
    raise ctypes.WinError(ctypes.get_last_error())
env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION="1")
rows = []
for block, order in enumerate(itertools.permutations(("awake", "sleep", "observed")), 1):
    for slot, variant in enumerate(order, 1):
        run = subprocess.run([str(exe), variant], env=env, capture_output=True, text=True, check=True)
        (row,) = csv.DictReader(run.stdout.splitlines())
        rows.append(dict(block=block, slot=slot, **row))
with (out / "results.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, rows[0])
    writer.writeheader()
    writer.writerows(rows)
inputs = [exe, archive, out / "budget.cpp", src / "validation/studies/GripLimit.cpp", src / "validation/studies/GripLimit.h"]
(out / "inputs.json").write_text(json.dumps({
    "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
    "compiler": subprocess.check_output(["clang++", "--version"], text=True).splitlines()[0],
    "flags": flags, "affinity_mask": hex(mask), "solver_threads": 1, "coupled_friction": True,
    "updates_per_frame": 8, "warm_trials": 3, "measured_trials": 24, "loaded_frames_per_trial": 15,
}, indent=2))
