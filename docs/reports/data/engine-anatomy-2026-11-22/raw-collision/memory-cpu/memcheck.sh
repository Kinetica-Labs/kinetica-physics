#!/usr/bin/env bash
set -euo pipefail
lab=/mnt/c/Users/PC/kp-2326667-mwrev8ox/sieve-20261122-100223
out="$lab/engine-anatomy-nov22/raw-collision/memory-cpu"
tools=/tmp/kinetica-anatomy-20261122
if [ ! -s "$tools/env" ]; then bash "$lab/build/anatomy-thirdparty/wsl-tools.sh" >"$out/wsl-tools-memcheck.log" 2>&1; fi
source "$tools/env"
d=$(mktemp -d /tmp/kp-memcheck.XXXXXX)
valgrind --tool=memcheck --log-file="$d/probe.log" /bin/true
cp -a "$lab/engine-anatomy-nov22/sg-src" "$d/src"
cmake -S "$d/src" -B "$d/build" -G 'Unix Makefiles' -DCMAKE_MAKE_PROGRAM="$(command -v make)" -DCMAKE_CXX_COMPILER="$CXX" -DCMAKE_BUILD_TYPE=Release -DKP_VIEWER=OFF '-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG -ffp-contract=off' >"$out/memcheck-build.log" 2>&1
cmake --build "$d/build" --target study-grip-limit -j6 >>"$out/memcheck-build.log" 2>&1
exe="$d/build/validation/studies/study-grip-limit"
if [ ! -f "$out/memcheck-startup.log" ]; then cp "$out/memcheck.log" "$out/memcheck-startup.log"; fi
sha256sum "$exe" >"$out/memcheck-sha.txt"
dpkg-query -W libc6 libc6-dbg >"$out/debug-versions.txt"
export KP_THREADS=1 KP_GRIP_TIMING=1 KP_GRIP_PROFILE=0 KP_COUPLED_FRICTION=1
set +e
valgrind --tool=memcheck --track-origins=yes --leak-check=full --show-leak-kinds=all --errors-for-leak-kinds=definite,indirect,possible --error-exitcode=97 --log-file="$out/memcheck.log" "$exe" shared8 >"$out/memcheck-trajectory.csv" 2>"$out/memcheck-stderr.txt"
rc=$?; echo "$rc" >"$out/memcheck-exit.txt"; tail -16 "$out/memcheck.log"; exit "$rc"
