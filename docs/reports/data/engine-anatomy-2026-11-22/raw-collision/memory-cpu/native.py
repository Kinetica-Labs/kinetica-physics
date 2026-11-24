"""Run with the same llvm-mingw as the timed Kinetica build, on the Windows host."""
import bisect
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

out = Path(__file__).resolve().parent
lab = out.parent.parent
src, build = lab / "sg-src", lab / "sg-build"
study = src / "validation/studies"
mem = study / "memory"
archive = build / "src/libkinetica-physics.a"
release = build / "validation/studies/study-grip-limit.exe"
work = out / "native-work"
work.mkdir(exist_ok=True)
env = dict(os.environ, KP_THREADS="1", KP_COUPLED_FRICTION="1")
flags = ["-O3", "-DNDEBUG", "-g", "-gcodeview", "-ffp-contract=off"]
inc = [f"-I{src/'include'}", f"-I{mem}", f"-I{study}", f"-I{out}"]

def run(args, **kwargs):
    return subprocess.run(list(map(str, args)), check=True, env=env, **kwargs)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def build_probe(name, files):
    exe = work / (name + ".exe")
    run(["clang++", *flags, "-std=c++17", *inc, *files, archive, "-lpthread", "-lpsapi", "-o", exe])
    return exe

def output(exe, name, *args):
    with (out/(name+".csv")).open("wb") as stdout, (out/(name+".stderr")).open("wb") as stderr:
        run([exe,*args],stdout=stdout,stderr=stderr)

def symbols(exe):
    lines = subprocess.check_output(["llvm-nm","-n","-C",str(exe)],text=True,encoding="utf8").splitlines()
    return sorted((int(m[1],16),m[2]) for line in lines if (m:=re.fullmatch(r"([a-fA-F0-9]+) [TtWw] (.+)",line)))

def origins(exe):
    from collections import Counter
    stats=Counter()
    matches=symbols(exe)
    addrs=[p for p,_ in matches]
    with (out/"heap-origins.csv").open() as f, (out/"heap-resolved.csv").open("w",newline="") as dest:
        w=csv.writer(dest)
        w.writerow(["origin","calls","bytes","site","backtrace"])
        for row in csv.DictReader(f):
            trace=[]
            for rva in row["return_rvas"].split():
                pc=0x140000000+int(rva,16)-1
                ix=bisect.bisect_right(addrs,pc)-1
                if ix < 0 or pc-addrs[ix] > 65536: break
                trace.append(matches[ix][1])
            site=next((name for name in trace if name.startswith("kp::")), "unknown")
            stats[site]+=int(row["calls"])
            w.writerow([row["origin"],row["calls"],row["bytes"],site," <- ".join(trace)])
    assert "unknown" not in stats and stats
    return sum(stats.values())

def probe():
    original=(src/"src/math/StepMemory.cpp").read_text(encoding="utf8")
    marked=original
    for old,new in [
            ("#include <atomic>", '#include "MemoryCounters.h"\n#include <atomic>'),
            ("if (!blocks.back()) throw std::bad_alloc();", "if (!blocks.back()) throw std::bad_alloc();\n            memory_study::refill(size);"),
            ("memory[0] = stepped ?", "if (stepped) memory_study::issue(bytes, size, reserve.held(), reserve.block + 1);\n    memory[0] = stepped ?")]:
        assert marked.count(old)==1,old
        marked=marked.replace(old,new)
    markedfile=work/"StepMemory-counted.cpp"
    markedfile.write_text(marked,encoding="utf8")
    counters=mem/"MemoryCounters.cpp"
    counted=build_probe("world-lifetime",[out/"probe.cpp",markedfile,counters])
    output(counted,"world-lifetime")
    plain=build_probe("plain-global-new",[study/"anatomy/kinetica_memory.cpp",counters])
    output(plain,"plain-global-new","coupled")
    traced=counters.read_text(encoding="utf8").replace("namespace memory_study {",'#include "Trace.h"\nnamespace memory_study {')
    assert traced.count("++counts.heapCalls;")==1
    tracefile=work/"MemoryCounters-trace.cpp"
    tracefile.write_text(traced.replace("++counts.heapCalls;","traceAllocation(bytes); ++counts.heapCalls;"),encoding="utf8")
    addr=build_probe("heap-origins",[out/"origins.cpp",tracefile])
    output(addr,"heap-origins")
    total_origins=origins(addr)
    (out/"native-inputs.json").write_text(json.dumps({"release_sha256":sha(release),"archive_sha256":sha(archive),
        "StepMemory_sha256":sha(src/"src/math/StepMemory.cpp"),"heap_origin_calls":total_origins,
        "probe_sha256":sha(counted),"plain_sha256":sha(plain),"trace_sha256":sha(addr),
        "flags":flags,"worlds":128,"frames_per_world":30,"substeps_per_frame":8},indent=2))

if __name__=="__main__":
    probe()
