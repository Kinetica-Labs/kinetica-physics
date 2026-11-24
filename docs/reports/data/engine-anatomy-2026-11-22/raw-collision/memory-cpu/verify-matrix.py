"""Full acceptance check and fresh three-engine comparison of the reserved-row candidate."""
import csv
import importlib.util
import io
import itertools
import json
import os
import random
from pathlib import Path
import shutil
import subprocess
import sys
from native import build, env, lab, out, sha, src, work
from counterfactual import text_hash

target=out/"accepted-rows"
target.mkdir(exist_ok=True)
source=lab/"active-rows-src"
final=lab/"active-rows-build"
rel=Path("src/dynamics/time_stepping/IslandNormals.cpp")

def compare():
    import ctypes
    if not ctypes.WinDLL("kernel32").SetProcessAffinityMask(ctypes.c_void_p(-1),ctypes.c_size_t(0x100)):raise ctypes.WinError()
    peers={r["solver"]:Path(r["exe"]) for r in csv.DictReader((lab/"raw-collision/sparse-guides/competitors/executables.csv").open(encoding="utf-8-sig"))}
    peers["kinetica"]=final/"validation/studies/study-grip-limit.exe"
    meta={k:sha(exe) for k,exe in peers.items()}
    orders=list(itertools.permutations(peers))*2
    random.Random(231123).shuffle(orders)
    with (target/"competitors.csv").open("w",newline="") as dest:
        w=csv.writer(dest);w.writerow(["block","slot","variant","frames","mean_us","drift_mm"])
        for block,order in enumerate(orders,1):
            for slot,k in enumerate(order,1):
                p=subprocess.run([peers[k],"shared8"],env=dict(env,KP_GRIP_TIMING="1",KP_GRIP_PROFILE="0"),check=True,capture_output=True,timeout=90)
                row,=csv.DictReader(io.StringIO(p.stdout.decode()))
                assert int(row["frames"])==360
                w.writerow([block,slot,k,360,row["us_per_frame"],row["end_drift_mm"]])
            dest.flush()
    (target/"competitor-inputs.json").write_text(json.dumps({"binary_sha256":meta,"balanced_orders":12,"seed":231123,"single_thread_cpu_mask":256},indent=2))

def main():
    sys.argv.append(str(lab))
    spec=importlib.util.spec_from_file_location("check",lab/"raw-collision/sparse-guides/verify.py")
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    if not source.exists():shutil.copytree(src,source)
    (source/rel).write_bytes((work/"matrix/IslandNormals.cpp").read_text(encoding="utf8").encode("utf8"))
    def files(root):
        return {p.relative_to(root):sha(p) for p in root.rglob("*") if p.is_file()}
    a,b=files(src),files(source)
    assert a.keys()==b.keys() and [k for k in a if a[k]!=b[k]]==[rel]
    cfg=m.native.cache(build)
    m.OUT=m.native.OUT=target
    m.BUILD=m.native.BUILD=final
    m.BASE=build
    m.REF=src
    m.run(["cmake","-S",source,"-B",final,"-G",cfg["CMAKE_GENERATOR"],*["-D"+k+"="+cfg[k] for k in m.native.OPTIONS]],"configure.log")
    group=["study-grip-limit","study-oblique-grip","study-groove-friction","study-state-hash","compare-ours","unit-tests",
           "FloatingBox","GearTrain","study-slip-to-roll","study-rest-overlay","study-playback-clock","study-hull-events",
           "study-hull-motion","study-collision-contract","study-bucket-pairs"]
    m.run(["cmake","--build",final,"--target",*group,"--parallel","8"],"build.log")
    for exe in ("validation/studies/study-grip-limit.exe","validation/engines/compare-ours.exe"):
        assert text_hash(final/exe)==text_hash(work/"matrix/reserve"/exe),exe
    m.verify()
    assert "100% tests passed" in (target/"ctest.log").read_text()
    rows=list(csv.DictReader((target/"verification/parity.csv").open()))
    assert len(rows)==17 and all(r["pass"]=="1" and r["sha256_A"]==r["sha256_B"] for r in rows)
    (target/"input.json").write_text(json.dumps({"changed_file":rel.as_posix(),"before":a[rel],"after":b[rel],"only_source_difference":True,
        "timed_grip_and_pyramid_machine_code_reproduced":True},indent=2))
    compare()
if __name__=="__main__":main()
