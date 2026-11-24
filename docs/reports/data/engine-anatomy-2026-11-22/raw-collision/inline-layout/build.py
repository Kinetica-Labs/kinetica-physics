"""Build the three experimental variants from an immutable native Windows reference."""
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

OUT = Path(__file__).resolve().parent
LAB = Path(sys.argv[1]).resolve()
BASE = LAB / "combined-core-src"
BUILD = LAB / "combined-core-build"
VEC = Path("include/math/Vector3.h")
SOLVER = Path("include/dynamics/time_stepping/SolverBody.h")
TARGETS = ["study-grip-limit", "compare-ours", "study-state-hash", "study-oblique-grip", "study-groove-friction"]
REFERENCE_GRIP = "2213c6e2d92b9ce02c29a9ecb8bdaf333faa54b6f765e95d3aeec0aa4bc37b89"

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def cache(build):
    result = {}
    for line in (build / "CMakeCache.txt").read_text(encoding="utf8").splitlines():
        key, sep, value = line.partition("=")
        if sep and ":" in key and not line.startswith(("//", "#")):
            result[key.split(":")[0]] = value
    return result

def once(text, before, after):
    if text.count(before) != 1:
        raise ValueError("unexpected header at " + before)
    return text.replace(before, after, 1)

def experiment_headers(forced, aligned):
    vector = (BASE / VEC).read_bytes().decode()
    solver = (BASE / SOLVER).read_bytes().decode()
    if aligned:
        vector = once(vector, "struct Vector3 {", "struct alignas(16) Vector3 {")
    if forced:
        macro = """#if defined(_MSC_VER)
#define KP_FORCE_INLINE __forceinline
#elif defined(__clang__) || defined(__GNUC__)
#define KP_FORCE_INLINE inline __attribute__((always_inline))
#else
#define KP_FORCE_INLINE inline
#endif
"""
        vector = once(vector, "namespace kp {", macro + "\nnamespace kp {")
        for signature in ["Real dot(", "Vector3 cross(", "Real lengthSquare(", "Real length(", "Vector3 scaled("]:
            vector = once(vector, "    " + signature, "    KP_FORCE_INLINE " + signature)
        if vector.count("\ninline Vector3") != 6:
            raise ValueError("unexpected number of vector operators")
        vector = vector.replace("\ninline Vector3", "\nKP_FORCE_INLINE Vector3")
        for signature in ["Vector3 spinFrom(", "static void push(", "static void twist(", "static Real inverseMassAlong("]:
            expr = signature.replace("static ", "static KP_FORCE_INLINE ") if signature.startswith("static ") else "KP_FORCE_INLINE " + signature
            solver = once(solver, "    " + signature, "    " + expr)
    return {VEC: vector.encode(), SOLVER: solver.encode()}

def run(args, logfile, env):
    with (OUT / logfile).open("wb") as log:
        p = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT, env=env)
    if p.returncode:
        raise RuntimeError((logfile, (OUT / logfile).read_text(errors="replace")[-2800:]))

def main():
    reference = BUILD / "validation/studies/study-grip-limit.exe"
    if os.name != "nt" or sha(reference) != REFERENCE_GRIP:
        raise RuntimeError("wrong host or baseline executable")
    cfg = cache(BUILD)
    compiler = Path(cfg["CMAKE_CXX_COMPILER"])
    env = dict(os.environ, PATH=str(compiler.parent) + os.pathsep + os.environ["PATH"])
    keys = ["CMAKE_CXX_COMPILER", "CMAKE_MAKE_PROGRAM", "CMAKE_BUILD_TYPE", "CMAKE_CXX_FLAGS",
            "CMAKE_CXX_FLAGS_RELWITHDEBINFO", "CMAKE_EXE_LINKER_FLAGS", "CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO", "KP_VIEWER"]
    (OUT / "config.json").write_text(json.dumps({k: cfg[k] for k in keys}, indent=2))
    run([compiler, "--version"], "compiler.txt", env)
    source_hashes = {p.relative_to(BASE):sha(p) for p in BASE.rglob("*") if p.is_file()}
    with (OUT / "manifest.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["variant","forced","aligned16","path","sha256"])
        for variant,forced,aligned in [("stock",False,False),("force",True,False),("align",False,True),("both",True,True)]:
            src = BASE if variant == "stock" else LAB / ("il-" + variant + "-src")
            build = BUILD if variant == "stock" else LAB / ("il-" + variant + "-build")
            expected = experiment_headers(forced, aligned)
            if variant != "stock":
                if not src.exists():
                    shutil.copytree(BASE, src)
                    for path, contents in expected.items():
                        (src / path).write_bytes(contents)
                installed = {p.relative_to(src):sha(p) for p in src.rglob("*") if p.is_file()}
                desired = dict(source_hashes)
                desired.update({k:hashlib.sha256(v).hexdigest() for k,v in expected.items()})
                if installed != desired:
                    raise RuntimeError(("unexpected source delta", variant))
                run(["cmake","-S",src,"-B",build,"-G",cfg["CMAKE_GENERATOR"],
                     *["-D"+k+"="+cfg[k] for k in keys]], variant+"-configure.log",env)
                if any(cache(build)[k].replace("\\","/") != cfg[k].replace("\\","/") for k in keys):
                    raise RuntimeError(("flags mismatch", variant))
                run(["cmake","--build",build,"--target",*TARGETS,"--parallel","8"],variant+"-build.log",env)
            lib = build / "src/libkinetica-physics.a"
            probe = OUT / ("probe-"+variant+".exe")
            run([compiler,"-std=c++17","-O3","-ffp-contract=off","-I"+str(src/"include"),
                 OUT/"layout.cpp",lib,"-lpthread","-o",probe], variant+"-probe-build.log",env)
            run([probe], variant+"-layout.csv",env)
            for path in [src/VEC,src/SOLVER,lib,build/"validation/studies/study-grip-limit.exe",build/"validation/engines/compare-ours.exe",
                         build/"validation/studies/study-state-hash.exe",build/"validation/studies/study-groove-friction.exe",build/"validation/studies/study-oblique-grip.exe"]:
                w.writerow([variant, int(forced), int(aligned),path.name,sha(path)])
            f.flush()
            print("ready",variant,flush=True)
    if sha(reference) != REFERENCE_GRIP:
        raise RuntimeError("changed baseline")

if __name__ == "__main__":
    main()
