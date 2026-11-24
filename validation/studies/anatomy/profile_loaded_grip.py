"""Collect VTune samples only during the loaded frames of the Windows grasp benchmark."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def main():
    p = argparse.ArgumentParser()
    for arg in ("source", "archive", "vtune", "out"):
        p.add_argument("--" + arg, required=True, type=Path)
    p.add_argument("--retired", action="store_true", help="Sample retired instructions in the loaded intervals")
    p.add_argument("--haswell-lto", action="store_true", help="Match the flags of the experimental Haswell archive")
    args = p.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    study = args.source / "validation/studies"
    for name in ("GripLimit.cpp", "GripSensitivity.h", "GripIslands.h", "StudyFriction.h"):
        shutil.copy2(study / name, out / name)
    header = (study / "GripLimit.h").read_text(encoding="utf8")
    for line in ("const auto start = Clock::now();", "const auto stop = Clock::now();"):
        if header.count(line) != 1:
            raise ValueError("The loaded timer boundary changed: " + line)
    header = header.replace("const auto start =", "if (run >= 0) __itt_resume();\n        const auto start =")
    header = header.replace("const auto stop = Clock::now();", "const auto stop = Clock::now();\n        if (run >= 0) __itt_pause();")
    (out / "GripLimit.h").write_text("#include <ittnotify.h>\n" + header, encoding="utf8")
    exe = out / "grip-loaded.exe"
    link = ["clang++", "-std=c++17", "-O3", "-DNDEBUG", "-ffp-contract=off", "-g", "-gcodeview",
            "-I" + str(args.source / "include"), "-I" + str(args.vtune / "include"),
            str(out / "GripLimit.cpp"), str(args.archive)]
    if args.haswell_lto:
        link[1:1] = ["-march=haswell", "-flto"]
    itt = args.vtune / "lib64/libittnotify.lib"
    if subprocess.run([*link, str(itt), "-lpthread", "-Wl,--pdb=" + str(out / "grip-loaded.pdb"), "-o", exe]).returncode:
        sdk = args.vtune / "sdk/src/ittnotify"
        obj = out / "itt.obj"
        subprocess.run(["clang", "-O2", "-I" + str(args.vtune / "include"), "-c",
                        sdk / "ittnotify_static.c", "-o", obj], check=True)
        subprocess.run([*link, str(obj), "-lpthread", "-Wl,--pdb=" + str(out / "grip-loaded.pdb"), "-o", exe], check=True)
    profiler = str(args.vtune / "bin64/vtune.exe")
    result = out / "collection"
    env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION="1", KP_GRIP_TIMING="1", KP_GRIP_PROFILE="1")
    collect = (["-collect-with", "runsa", "-knob", "event-config=INST_RETIRED.ANY:sa=100003"]
               if args.retired else ["-collect", "hotspots", "-knob", "sampling-mode=sw"])
    command = [profiler, *collect, "-start-paused", "-result-dir", str(result), "--", str(exe), "shared8"]
    with (out / "client.txt").open("wb") as client, (out / "collection.log").open("wb") as log:
        subprocess.run(command, env=env, stdout=client, stderr=log, check=True)
    with (out / "functions.csv").open("wb") as f, (out / "report.log").open("wb") as log:
        report, group = ("hw-events", "process") if args.retired else ("hotspots", "function")
        subprocess.run([profiler, "-report", report, "-r", result, "-format", "csv", "-csv-delimiter",
                        "comma", "-group-by", group], stdout=f, stderr=log, check=True)
    with (out / "summary.txt").open("wb") as f:
        subprocess.run([profiler, "-report", "summary", "-r", result], stdout=f, check=True)
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    (out / "inputs.json").write_text(json.dumps({"engine_archive_sha256": sha(args.archive),
        "benchmark_exe_sha256": sha(exe), "oracle_header_sha256": sha(study / "GripLimit.h"),
        "kernel": "ITT resume/pause, 1000 x 15 loaded frames", "profile":report, "group":group,
        "timer_performance": "use the corresponding uninstrumented balanced blocks",
        "solver_threads": 1, "sleep": False, "steps_per_frame": 8,
        "command": command}, indent=2))


if __name__ == "__main__":
    main()
