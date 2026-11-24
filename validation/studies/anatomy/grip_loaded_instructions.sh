#!/usr/bin/env bash
# Count user-space instructions during the same 360 loaded frames as the native timer.
# Run in WSL on the Windows measurement host. Usage: bash grip_loaded_instructions.sh SOURCE OUTPUT
set -euo pipefail
source_tree=$(realpath "$1")
output=$(realpath -m "$2")
grep -qi microsoft /proc/sys/kernel/osrelease
mkdir -p "$output"
tool_env=${KP_TOOL_ENV:-/tmp/kinetica-anatomy-20261122/env}
if [[ ! -f "$tool_env" && -n ${3:-} ]] && ! command -v cmake >/dev/null; then
    bash "$3"
fi
if [[ -f "$tool_env" ]]; then source "$tool_env"; fi
export LC_ALL=C KP_THREADS=1 KP_COUPLED_FRICTION=1 KP_GRIP_TIMING=1 KP_GRIP_PROFILE=0
unset KP_STEP_MEMORY
work=$(mktemp -d /tmp/kp-grip-instructions.XXXXXX)
mkdir "$work/src"
tar -C "$source_tree" -cf - CMakeLists.txt src include tests examples validation | tar -C "$work/src" -xf -
python3 - "$work/src/validation/studies/GripLimit.h" "$output" <<'PY'
import hashlib
import json
import sys
from pathlib import Path
header, output = map(Path, sys.argv[1:])
old = header.read_bytes()
text = old.decode()
points = {
    "#pragma once": "#pragma once\n#include <valgrind/callgrind.h>",
    "const auto start = Clock::now();": "if (run >= 0) { CALLGRIND_TOGGLE_COLLECT; }\n        const auto start = Clock::now();",
    "const auto stop = Clock::now();": "const auto stop = Clock::now();\n        if (run >= 0) { CALLGRIND_TOGGLE_COLLECT; }",
}
for marker, code in points.items():
    if text.count(marker) != 1:
        sys.exit("Unexpected timing boundaries: " + marker)
    text = text.replace(marker, code)
header.write_text(text)
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
root = header.parents[2]
paths = ("src/dynamics/time_stepping/SoftStep.cpp", "src/collision/ContactPair.cpp", "src/world/World.cpp")
(output/"scope.json").write_text(json.dumps({
    "format":"Linux x86-64 ELF, WSL2 on the Windows benchmark host", "frames":360,
    "steps_per_frame":8, "threads":1, "sleep":False, "coupled_friction":True,
    "scope":"Only the 24 intervals of 15 loaded frames, including the two steady clock reads",
    "unmarked_header_sha256":hashlib.sha256(old).hexdigest(), "marked_header_sha256":sha(header),
    "source_sha256":{p:sha(root/p) for p in paths}}, indent=2))
PY
cm=${CMAKE:-cmake}
cxx=${CXX:-c++}
vg=${VALGRIND:-valgrind}
flags='-O3 -DNDEBUG -march=haswell -ffp-contract=off'
{
    "$cm" --version
    "$cxx" --version
    "$vg" --version
    uname -srmo
    echo "CMAKE_INTERPROCEDURAL_OPTIMIZATION=ON CMAKE_CXX_FLAGS_RELEASE=$flags"
} > "$output/toolchain.txt"
"$cm" -S "$work/src" -B "$work/build" -G 'Unix Makefiles' -DKP_VIEWER=OFF \
    "-DCMAKE_MAKE_PROGRAM=$(command -v make)" "-DCMAKE_CXX_COMPILER=$cxx" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON \
    "-DCMAKE_CXX_FLAGS_RELEASE=$flags" > "$output/configure.log" 2>&1
"$cm" --build "$work/build" --parallel 6 --target study-grip-limit > "$output/build.log" 2>&1
exe="$work/build/validation/studies/study-grip-limit"
sha256sum "$exe" > "$output/binary.sha256"
for repeat in 1 2; do
    "$vg" --tool=callgrind --collect-atstart=no --cache-sim=no --branch-sim=no \
        --callgrind-out-file="$work/counts" --log-file="$output/valgrind-$repeat.log" \
        "$exe" shared8 > "$output/harness-$repeat.csv"
    python3 - "$work/counts" "$output" "$repeat" <<'PY'
import csv
from pathlib import Path
import sys
raw, output = map(Path, sys.argv[1:3])
repeat = int(sys.argv[3])
rows = list(csv.DictReader((output/f"harness-{repeat}.csv").open()))
if len(rows) != 1 or rows[0]["frames"] != "360":
    sys.exit("Incomplete loaded-frame sample")
series = raw.read_text().splitlines()
events, = (line.split()[1:] for line in series if line.startswith("events:"))
totals, = (line.split()[1:] for line in series if line.startswith("totals:"))
count = int(totals[events.index("Ir")])
if count <= 0:
    sys.exit("No instruction count")
with (output/"events.csv").open("w" if repeat == 1 else "a", newline="") as f:
    writer = csv.writer(f)
    if repeat == 1: writer.writerow(("repeat","frames","loaded_instructions","instructions_per_frame"))
    writer.writerow((repeat,360,count,count/360))
PY
done
gzip -c "$work/counts" > "$output/callgrind.out.gz"
cat "$output/events.csv"
