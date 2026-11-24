"""Build two independent storage experiments on the Windows machine and time both scenes."""
import csv
import difflib
import importlib.util
import itertools
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
from counterfactual import libs, text_hash, variant
from native import archive, build, env, flags, lab, out, release, sha, src, work

suite=work/"matrix"
suite.mkdir(exist_ok=True)
def changed(name, original, replacements):
    source=(src/original).read_bytes().decode("utf8").replace("\r\n","\n")
    s=source
    for before,after in replacements:
        assert s.count(before)==1,before
        s=s.replace(before,after)
    file=suite/(name+".cpp")
    file.write_text(s,encoding="utf8")
    patch=difflib.unified_diff(source.splitlines(True),s.splitlines(True),fromfile="a/"+original,tofile="b/"+original)
    (out/("matrix-"+name+".patch")).write_text("".join(patch),encoding="utf8")
    return (name,file,[])
def sources():
    name="ContactSystem"
    fn="src/dynamics/event_driven/ContactSystem.cpp"
    orig=(src/fn).read_text(encoding="utf8")
    start=orig.index("    StepArray<size_t> count(bodies.size(), 0);")
    end=orig.index("\n    SparseMatrix matrix",start)
    flat=changed(name,fn,[(orig[start:end],"""    StepArray<size_t> offsets(bodies.size() + 1, 0);
    for (const Line& line : lines) ++offsets[size_t(line.a) + 1], ++offsets[size_t(line.b) + 1];
    for (size_t i = 0; i < bodies.size(); ++i) offsets[i + 1] += offsets[i];
    StepArray<size_t> cursor = offsets, rows(offsets.back());
    for (size_t i = 0; i < lines.size(); ++i) {
        rows[cursor[size_t(lines[i].a)]++] = i;
        rows[cursor[size_t(lines[i].b)]++] = i;
    }
"""),("0 : count[a]","0 : offsets[a + 1] - offsets[a]"),("0 : count[b]","0 : offsets[b + 1] - offsets[b]"),
          ("for (size_t j : onBody[index]) {","for (size_t sourceIndex = offsets[index]; sourceIndex < offsets[index + 1]; ++sourceIndex) {\n            const size_t j = rows[sourceIndex];"),
          ("for (size_t i : onBody[index]) {","for (size_t targetIndex = offsets[index]; targetIndex < offsets[index + 1]; ++targetIndex) {\n                const size_t i = rows[targetIndex];")])
    reserve=changed("IslandNormals","src/dynamics/time_stepping/IslandNormals.cpp",[
        ("    ContactSystem system;\n    StepArray<size_t> listed;", """    const size_t total = island.working.size() - size_t(std::count(island.working.begin(), island.working.end(), kOutside));
    if (total == 0) return 0;
    ContactSystem system;
    system.lines.reserve(total);
    system.rows.reserve(total);
    system.order.reserve(total);
    StepArray<size_t> listed;"""),
        ("    StepArray<char> pressing;\n    for (size_t r", """    StepArray<char> pressing;
    listed.reserve(total);
    impulse.reserve(total);
    free.reserve(total);
    pressing.reserve(total);
    for (size_t r"""),
        ("    const size_t total = listed.size();\n    if (total == 0) return 0;\n","")])
    return flat,reserve
def main():
    sys.argv.append(str(lab))
    spec=importlib.util.spec_from_file_location("timer",lab/"raw-collision/inline-layout/measure.py")
    timer=importlib.util.module_from_spec(spec);spec.loader.exec_module(timer)
    flat,reserve=sources()
    K=Path("validation/studies/study-grip-limit.exe")
    P=Path("validation/engines/compare-ours.exe")
    def pyramid(lib,exe):
        adapter=build/"validation/engines/CMakeFiles/compare-ours.dir/objects.a"
        subprocess.run(["clang++",*flags,"-Wl,--whole-archive",adapter,"-Wl,--no-whole-archive","-o",exe,
                        "-Wl,--major-image-version,0,--minor-image-version,0",lib,*libs],check=True)
    pyramid(archive,suite/"control.exe")
    assert text_hash(suite/"control.exe")==text_hash(build/P)
    control=variant("matrix-recompiled",[(x[0],src/y,[]) for x,y in [(flat,"src/dynamics/event_driven/ContactSystem.cpp"),(reserve,"src/dynamics/time_stepping/IslandNormals.cpp")]])
    assert text_hash(control)==text_hash(release)
    builds={"stock":build}
    meta={}
    for key,units in [("flat",[flat]),("reserve",[reserve]),("combined",[flat,reserve])]:
        exe=variant("matrix-"+key,units)
        loc=suite/key
        for rel in (K,P):(loc/rel).parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(exe,loc/K)
        pyramid(work/("matrix-"+key)/"libkinetica-physics.a",loc/P)
        builds[key]=loc
    for k,b in builds.items():
        p=subprocess.run([b/K,"shared8"],env=dict(env,KP_GRIP_TIMING="0"),capture_output=True)
        rows=list(csv.DictReader(p.stdout.decode().splitlines()))
        assert len(rows)==36 and all(r["complete"]=="1" for r in rows)
        (out/("matrix-"+k+"-grip.csv")).write_bytes(p.stdout)
        meta[k]={"grip":sha(b/K),"pyramid":sha(b/P),"oracle":sha(out/("matrix-"+k+"-grip.csv"))}
    mask=timer.pin()
    orders=list(itertools.permutations(builds));random.Random(23112026).shuffle(orders)
    with (out/"matrix-time.csv").open("w",newline="") as dest:
        w=csv.writer(dest);w.writerow(["scene","block","slot","variant","frames","mean_us","drift_mm"])
        for block,order in enumerate(orders,1):
            for scene in (("grip","pyramid") if block%2 else ("pyramid","grip")):
                for slot,key in enumerate(order,1):
                    us,frames,drift,_=timer.frame(scene,builds[key])
                    w.writerow([scene,block,slot,key,frames,us,drift])
            dest.flush()
    (out/"matrix-method.json").write_text(json.dumps({"affinity":mask,"design":"all 24 permutations, both scenes, original compiler and flags, one thread, strict contraction off",
        "unchanged_object_rebuild_reproduces_release_text":True,"inputs":meta},indent=2))
if __name__=="__main__":main()
