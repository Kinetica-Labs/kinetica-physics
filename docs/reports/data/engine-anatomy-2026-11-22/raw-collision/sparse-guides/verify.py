"""Native release build and paired timing on the dedicated Windows machine."""
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime,timezone

OUT=Path(__file__).resolve().parent
LAB=Path(sys.argv[1]).resolve()
BASE=LAB/"combined-core-build"
REF=LAB/"combined-core-src"
SRC=LAB/"sg-src"
BUILD=LAB/"sg-build"

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m

native=load("native",LAB/"raw-collision/split-force/native.py")
native.OUT=OUT
native.BUILD=BUILD
cfg=native.cfg
env=native.env
sha=native.sha
run=native.execute
def build():
    inputs=list(csv.DictReader((OUT/"inputs.csv").open()))
    fresh=not SRC.exists()
    if fresh: shutil.copytree(REF,SRC)
    before={p.relative_to(REF).as_posix():sha(p) for p in REF.rglob("*") if p.is_file()}
    expected=dict(before)
    for r in inputs:
        k=r["path"]
        assert before[k]==r["before"] and sha(OUT/"changed"/k)==r["after"]
        if fresh: shutil.copyfile(OUT/"changed"/k,SRC/k)
        expected[k]=r["after"]
    actual={p.relative_to(SRC).as_posix():sha(p) for p in SRC.rglob("*") if p.is_file()}
    assert actual==expected,"unexpected source change"
    run(["cmake","-S",SRC,"-B",BUILD,"-G",cfg["CMAKE_GENERATOR"],*["-D"+k+"="+cfg[k] for k in native.OPTIONS]],"configure.log")
    assert all(native.cache(BUILD)[k].replace("\\","/")==cfg[k].replace("\\","/") for k in native.OPTIONS)
    targets=["study-grip-limit","study-oblique-grip","study-groove-friction","study-state-hash","compare-ours","unit-tests","FloatingBox","GearTrain",
             "study-slip-to-roll","study-rest-overlay","study-playback-clock","study-hull-events","study-hull-motion","study-collision-contract","study-bucket-pairs"]
    run(["cmake","--build",BUILD,"--target",*targets,"--parallel","8"],"build.log")
    with (OUT/"executables.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["variant","file","sha256"])
        for v,b in (("stock",BASE),("sparse",BUILD)):
            for rel in ("src/libkinetica-physics.a","validation/studies/study-grip-limit.exe","validation/studies/study-state-hash.exe","validation/engines/compare-ours.exe"):
                w.writerow([v,rel,sha(b/rel)])
    print("build complete",flush=True)

def verify():
    gen=BUILD/"tests/CTestTestfile.cmake"
    content=gen.read_text(encoding="utf8")
    bash=Path("C:/Program Files/Git/bin/bash.exe")
    for key in ("no-libm","no-libm-scenes"):
        prefix=f'add_test({key} "'
        if bash.as_posix() not in content.split(prefix)[1].splitlines()[0]:
            content=content.replace(prefix,prefix+bash.as_posix()+'" "')
    gen.write_text(content,encoding="utf8")
    nm=BUILD/"ctest-nm";nm.mkdir(exist_ok=True)
    shutil.copy2(native.cc.parent/"llvm-nm.exe",nm/"nm.exe")
    rc=run(["ctest","--test-dir",BUILD,"--output-on-failure","-j2"],"ctest.log",(0,8),dict(env,PATH=str(nm)+os.pathsep+env["PATH"]))
    print("CTest",rc,flush=True)
    a,b=BASE/"validation/studies",BUILD/"validation/studies"
    script=REF/"validation/studies/anatomy/guide_verify.py"
    rc=run([sys.executable,script,a,b,OUT/"verification","--parity-only"],"parity.log",(0,1))
    print("parity",rc,flush=True)
    for v,folder in (("stock",a),("sparse",b)):
        run([folder/"study-state-hash.exe","2"],v+"-state2.txt",environ=dict(env,KP_STEP_MEMORY="check"))
        run([folder/"study-grip-limit.exe","island-joints"],v+"-island-wipe.csv",environ=dict(env,KP_STEP_MEMORY="check"))
    for tail in ("state2.txt","island-wipe.csv"):
        if sha(OUT/("stock-"+tail))!=sha(OUT/("sparse-"+tail)): raise RuntimeError("state mismatch: "+tail)

def timing():
    m=load("timer",LAB/"raw-collision/inline-layout/measure.py")
    mask=m.pin()
    begin=datetime.now(timezone.utc)
    with (OUT/"timings.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["scene","block","slot","order","variant","frames","mean_us","end_drift_mm","max_frame_us"])
        for n in range(1,25):
            for scene in (("grip","pyramid") if n%2 else ("pyramid","grip")):
                order="AB" if (n-1)%4 in (0,3) else "BA"
                for slot,v in enumerate(order,1):
                    us,frames,drift,worst=m.frame(scene,BASE if v=="A" else BUILD)
                    w.writerow([scene,n,slot,order,v,frames,us,drift,worst])
            f.flush()
            if n%8==0: print("timing",n,flush=True)
    (OUT/"method.json").write_text(json.dumps({"utc_start":begin.isoformat(),"utc_end":datetime.now(timezone.utc).isoformat(),
        "affinity_mask":mask,"worker_threads":1,"host":os.environ["COMPUTERNAME"],"builds":"A combined-core; B sparse guide response",
        "design":"24 blocks per scene; AB BA BA AB repeated; externally calibrated native release harness",
        "grip":"shared8, coupled, 10 N squeeze/jaw and load, mu .5, 360 timed frames, h=1/480 s",
        "pyramid":"BENCH=7 compare-ours realtime 8, 140 cubes, uncoupled, 60 frames per sample"},indent=2))

if __name__=="__main__":
    if os.name!="nt":raise SystemExit("native Windows only")
    mode=sys.argv[2]
    if mode=="build":build()
    elif mode=="verify":verify()
    elif mode=="time":timing()
    else:raise ValueError(mode)
