#!/usr/bin/env bash
# Run under WSL on the designated Windows measurement host.
# --binaries KINETICA_ELF BOX3D_ELF RP3D_ELF
# --sources  KINETICA_SRC BOX3D_SRC RP3D_SRC
# Example: OUT=/tmp/grip-cg-01 JOBS=4 bash callgrind.sh --sources /tmp/k /tmp/b /tmp/r
set -Eeuo pipefail

if [[ $# != 4 || ( $1 != --sources && $1 != --binaries ) ]]; then
    echo "usage: bash callgrind.sh --binaries K B R | --sources K B R" >&2
    exit 2
fi
if ! grep -qi microsoft /proc/sys/kernel/osrelease; then
    echo "Run this profiler under WSL on the Windows test machine." >&2
    exit 2
fi
mode=$1
shift
if [[ -n ${OUT:-} ]]; then
    out=$(realpath -m -- "$OUT")
    [[ $out == /tmp/* ]] || { echo "OUT must be a new directory under /tmp" >&2; exit 2; }
    mkdir -- "$out"  # A retry preserves previous evidence.
else
    out=$(mktemp -d /tmp/kp-callgrind.XXXXXX)
fi
echo "$out"
trap 'rc=$?; printf "incomplete: line %s, exit %s\n" "$LINENO" "$rc" >> "$out/status.txt"; exit "$rc"' ERR
export LC_ALL=C KP_THREADS=1 KP_GRIP_PROFILE=0 KP_GRIP_TIMING=1
unset KP_STEP_MEMORY
vg=${VALGRIND:-valgrind}
"$vg" --version > "$out/version.txt"
{
    uname -srmo
    printf '%s\n' "mode=$mode" 'shared8, one engine thread, sleep off, force per public update' \
        'reported loaded frames=360; process-wide counters cover 810 frames (27 fresh worlds x 15 preload + 15 loaded each)' \
        'whole process, including initialization, warmups, teardown and the timing harness' \
        'cache and branch events are simulated; ignore instrumented us_per_frame'
} > "$out/config.txt"

if [[ $mode == --sources ]]; then
    ksrc=$(realpath -- "$1")
    bsrc=$(realpath -- "$2")
    rsrc=$(realpath -- "$3")
    cm=${CMAKE:-cmake}
    cc=${CC:-cc}
    cxx=${CXX:-c++}
    make=$(command -v "${MAKE:-make}")
    jobs=${JOBS:-4}
    precision=${RP_PRECISION:-double}
    case $precision in
        double) rp_double=ON; rp_flags=(-DIS_RP3D_DOUBLE_PRECISION_ENABLED) ;;
        float)  rp_double=OFF; rp_flags=() ;;
        *) echo "RP_PRECISION must be float or double" >&2; exit 2 ;;
    esac
    build=$(mktemp -d /tmp/kp-cg-build.XXXXXX)
    echo "scratch build directory: $build" >> "$out/config.txt"
    mkdir -- "$out/bin"
    kexe="$out/bin/kinetica"
    bexe="$out/bin/box3d"
    rexe="$out/bin/rp3d"
    # Engine libraries are linked statically. Executables retain function symbols.
    common=(-G 'Unix Makefiles' "-DCMAKE_MAKE_PROGRAM=$make" "-DCMAKE_CXX_COMPILER=$cxx"
            -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
            '-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG -ffp-contract=off')
    {
        "$cm" --version
        "$cc" --version
        "$cxx" --version
        printf 'rp_precision=%s\n' "$precision"
        for src in "$ksrc" "$bsrc" "$rsrc"; do
            printf '%s ' "$src"
            git -C "$src" rev-parse --verify HEAD 2>/dev/null || echo 'revision unavailable'
        done
    } >> "$out/config.txt"
    {
        "$cm" -S "$ksrc" -B "$build/k" "${common[@]}" -DKP_VIEWER=OFF
        "$cm" --build "$build/k" --target study-grip-limit -j "$jobs"
        cp -- "$build/k/validation/studies/study-grip-limit" "$kexe"
        "$cm" -S "$bsrc" -B "$build/b" "${common[@]}" "-DCMAKE_C_COMPILER=$cc" \
            '-DCMAKE_C_FLAGS_RELEASE=-O3 -DNDEBUG -ffp-contract=off' \
            -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_BENCHMARKS=OFF -DBOX3D_VALIDATE=OFF
        "$cm" --build "$build/b" --target box3d -j "$jobs"
        "$cxx" -std=gnu++17 -O3 -DNDEBUG -ffp-contract=off "-I$bsrc/include" \
            "$ksrc/validation/engines/grip_box3d.cpp" "$build/b/src/libbox3d.a" -pthread -lm -o "$bexe"
        "$cm" -S "$rsrc" -B "$build/r" "${common[@]}" "-DRP3D_DOUBLE_PRECISION_ENABLED=$rp_double"
        "$cm" --build "$build/r" --target reactphysics3d -j "$jobs"
        "$cxx" -std=c++17 -O3 -DNDEBUG -ffp-contract=off "${rp_flags[@]}" "-I$rsrc/include" \
            "$ksrc/validation/engines/grip_rp3d.cpp" "$build/r/libreactphysics3d.a" -pthread -o "$rexe"
    } > "$out/build.log" 2>&1
    # Remove only the unique, script-created scratch tree. Keep binaries, config and logs.
    rm -r -- "$build"
else
    kexe=$(realpath -- "$1")
    bexe=$(realpath -- "$2")
    rexe=$(realpath -- "$3")
fi

printf 'variant,event,count\n' > "$out/events.csv"
failed=0
for variant in kinetica-default kinetica-coupled box3d rp3d; do
    export KP_COUPLED_FRICTION=0
    case $variant in
        kinetica-default) exe=$kexe; expected='kinetica-shared8' ;;
        kinetica-coupled) exe=$kexe; expected='kinetica-shared8'; export KP_COUPLED_FRICTION=1 ;;
        box3d) exe=$bexe; expected='box3d-shared8' ;;
        rp3d) exe=$rexe; expected='rp3d-f(32|64)-shared8' ;;
    esac
    dir="$out/$variant"
    mkdir -- "$dir"
    if [[ ! -x $exe ]] || [[ $(od -An -tx1 -N4 -- "$exe" | tr -d ' \n') != 7f454c46 ]]; then
        echo "incomplete $variant: expected executable Linux ELF" >> "$out/status.txt"
        failed=1
        continue
    fi
    sha256sum -- "$exe" > "$dir/binary.sha256"
    # Raw events cover the complete process. The program's timed region is narrower.
    if ! "$vg" --tool=callgrind --cache-sim=yes --branch-sim=yes \
        --callgrind-out-file="$dir/callgrind.out" --log-file="$dir/valgrind.log" \
        "$exe" shared8 > "$dir/harness.csv" 2> "$dir/client.stderr"; then
        echo "incomplete $variant: see valgrind.log and client.stderr" >> "$out/status.txt"
        failed=1
        continue
    fi
    if ! awk -F, -v id="$expected" '
        NR == 1 { header = ($0 == "engine,frames,us_per_frame,end_drift_mm") }
        NR == 2 { ok = (NF == 4 && $1 ~ ("^" id "$") && $2 == "360") }
        END { exit !(NR == 2 && header && ok) }
    ' "$dir/harness.csv"; then
        echo "incomplete $variant: unexpected harness output" >> "$out/status.txt"
        failed=1
        continue
    fi
    if ! awk -v variant="$variant" '
        /^events:/ {
            n = NF - 1
            for (i=1; i<=n; ++i) { event[i] = $(i+1); found[$(i+1)] = 1 }
        }
        /^totals:/ {
            ++totals
            for (i=1; i<NF; ++i) count[i] = $(i+1)
            fields = NF - 1
        }
        END {
            if (totals != 1 || n == 0 || fields > n || event[1] != "Ir" || count[1] <= 0) exit 1
            if (!found["D1mr"] || !found["DLmr"] || !found["Bc"] || !found["Bcm"]) exit 1
            for (i=1; i<=fields; ++i) if (count[i] !~ /^[0-9]+$/) exit 1
            for (i=1; i<=n; ++i) print variant "," event[i] "," (i<=fields ? count[i] : "0")
        }
    ' "$dir/callgrind.out" > "$dir/totals.csv"; then
        echo "incomplete $variant: absent or invalid Callgrind totals" >> "$out/status.txt"
        failed=1
        continue
    fi
    cat "$dir/totals.csv" >> "$out/events.csv"
    echo "complete $variant" >> "$out/status.txt"
done
cat "$out/status.txt"
exit "$failed"
