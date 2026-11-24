#!/usr/bin/env bash
# This file runs only inside WSL on the Windows measurement machine.
set -euo pipefail
lab=$1
out="$lab/engine-anatomy-nov22/raw-collision/memory-cpu"
src="$lab/engine-anatomy-nov22/sg-src"
peer=/mnt/c/Users/sshuser/CODE/kinetica-physics
tools=/tmp/kinetica-anatomy-20261122
if [[ ! -s "$tools/env" ]]; then
    bash "$lab/build/anatomy-thirdparty/wsl-tools.sh" > "$out/wsl-bootstrap.log" 2>&1
fi
source "$tools/env"
work=$(mktemp -d /tmp/kp-phase-profile.XXXXXX)
mkdir "$work/src"
cp -a "$src/." "$work/src/"
export LC_ALL=C
python3 - "$work" "$out" <<'PY'
import hashlib, json, sys
from pathlib import Path
w,out=map(Path,sys.argv[1:])
header=w/"src/validation/studies/GripLimit.h"
old=header.read_bytes()
s=old.decode("utf8")
changes={"#pragma once":"#pragma once\n#include <valgrind/callgrind.h>",
  "        const auto start = Clock::now();":"        if (run >= 0) { CALLGRIND_TOGGLE_COLLECT; }\n        const auto start = Clock::now();",
  "        const auto stop = Clock::now();":"        const auto stop = Clock::now();\n        if (run >= 0) { CALLGRIND_TOGGLE_COLLECT; }"}
for a,b in changes.items():
    assert s.count(a)==1,a
    s=s.replace(a,b)
header.write_bytes(s.encode("utf8"))
script=(w/"src/validation/studies/anatomy/callgrind.sh").read_text()
a='"$vg" --tool=callgrind --cache-sim=yes'
assert script.count(a)==1
(w/"run.sh").write_text(script.replace(a,'"$vg" --tool=callgrind --collect-atstart=no --cache-sim=yes'))
manifest={"base_header_sha256":hashlib.sha256(old).hexdigest(),"marked_header_sha256":hashlib.sha256(header.read_bytes()).hexdigest(),
  "workdir":str(w),"scope":"main solver thread, only the 360 loaded frames. Counters paused during initialization, preload, warmup and teardown. Memcheck covers the entire process."}
(out/"range.json").write_text(json.dumps(manifest,indent=2))
PY
export OUT="$work/counts" JOBS=6 CMAKE_POLICY_VERSION_MINIMUM=3.5
outcome=0
if ! bash "$work/run.sh" --sources "$work/src" "$peer/box3d" "$peer/rp3d" >"$out/callgrind-job.log" 2>&1; then outcome=1; fi
for name in events.csv status.txt version.txt config.txt; do test ! -f "$OUT/$name" || cp "$OUT/$name" "$out/$name"; done
if [[ -f "$OUT/build.log" ]]; then tail -35 "$OUT/build.log" >"$out/wsl-build-tail.txt"; fi
for v in kinetica-default kinetica-coupled box3d rp3d; do
    d="$OUT/$v"
    [[ -f "$d/callgrind.out" ]] || continue
    mkdir -p "$out/$v"
    cp "$d"/{valgrind.log,harness.csv,binary.sha256} "$out/$v/"
    gzip -c "$d/callgrind.out" > "$out/$v/callgrind.out.gz"
    callgrind_annotate --inclusive=no --auto=no --threshold=95 --show=Ir "$d/callgrind.out" >"$out/$v/self.txt"
done
if (( outcome )); then tail -15 "$out/callgrind-job.log"; test ! -f "$out/wsl-build-tail.txt" || tail -10 "$out/wsl-build-tail.txt"; exit 1; fi
cat "$out/status.txt"
export KP_THREADS=1 KP_GRIP_TIMING=1 KP_GRIP_PROFILE=0 KP_COUPLED_FRICTION=1
set +e
valgrind --tool=memcheck --track-origins=yes --leak-check=full --show-leak-kinds=all \
    --errors-for-leak-kinds=definite,indirect,possible --error-exitcode=97 --log-file="$out/memcheck.log" \
    "$OUT/bin/kinetica" shared8 >"$out/memcheck-trajectory.csv" 2>"$out/memcheck-stderr.txt"
rc=$?
set -e
echo "$rc" >"$out/memcheck-exit.txt"
tail -13 "$out/memcheck.log"
echo "Callgrind and Memcheck finished"
