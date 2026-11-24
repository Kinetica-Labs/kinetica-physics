"""Native Windows study. Compile changed translation units with the release target flags."""
from pathlib import Path
import csv
import difflib
import hashlib
import importlib.util
import itertools
import json
import os
import random
import re
import shlex
import shutil
import struct
import subprocess
import sys
from datetime import datetime, timezone

OUT=Path(__file__).resolve().parent
LAB=Path(sys.argv[1]).resolve()
SRC=LAB/"combined-core-src"
BASE=LAB/"combined-core-build"
DI=Path("src/dynamics/time_stepping/DirectJoints.cpp")
JO=Path("src/joints/Joint.cpp")
FILES={DI:"100ffb2e5d9754322194901bfc4e298da6499cc23162b86c53ca78440c6746d5",
       JO:"9e456e3b14e413fec40c04ef7bb2d71e2411ca83775ace51e5e1b2590211fa21"}
TARGETS={"study-grip-limit":"studies","study-oblique-grip":"studies","study-groove-friction":"studies",
         "study-state-hash":"studies","compare-ours":"engines"}
VARIANTS=("stock","zero","reuse","both")
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def once(s,a,b):
    if s.count(a)!=1:
        raise ValueError(a)
    return s.replace(a,b,1)

def edits(v):
    src={k:(SRC/k).read_bytes().decode() for k in FILES}
    push="void DirectJoints::push(size_t row, StepArray<SolverBody>& states, Real impulse) const {\n"
    if v in ("zero","both"):
        src[DI]=once(src[DI],push,push+"    if (impulse == 0) return;\n")
    if v in ("reuse","both"):
        s=once(src[JO],"Vector3 freeSpinUp(const Body& body)", "Vector3 freeSpinUp(const Body& body, Vector3 angularVelocity)")
        s=once(s,"body.angularMomentum.cross(body.angularVelocity())","body.angularMomentum.cross(angularVelocity)")
        src[JO]=once(s,"m.wa = ma.angularVelocity(), m.wb = mb.angularVelocity(), m.spinUpA = freeSpinUp(ma), m.spinUpB = freeSpinUp(mb);",
                     "m.wa = ma.angularVelocity(), m.wb = mb.angularVelocity();\n    m.spinUpA = freeSpinUp(ma, m.wa), m.spinUpB = freeSpinUp(mb, m.wb);")
    if v=="instrument":
        for k in FILES:
            src[k]=once(src[k],"namespace kp {","#include <atomic>\n#include <cstdio>\nnamespace kp {")
        src[DI]=once(src[DI],push,"""namespace {
struct PushCounts {
    std::atomic<unsigned long long> calls{0}, zeros{0}, tiny{0};
    ~PushCounts() { std::fprintf(stderr,"push,%llu,%llu,%llu\\n",calls.load(),zeros.load(),tiny.load()); }
} counter;
}
"""+push+"""    counter.calls.fetch_add(1,std::memory_order_relaxed);
    if (impulse==0) counter.zeros.fetch_add(1,std::memory_order_relaxed);
    else if (abs(impulse)<Real(1e-12)) counter.tiny.fetch_add(1,std::memory_order_relaxed);
""")
        src[JO]=once(src[JO],"namespace {","""namespace {
struct BondCounts {
    std::atomic<unsigned long long> builds{0}, rows{0}, drift{0};
    ~BondCounts() { std::fprintf(stderr,"bonds,%llu,%llu,%llu\\n",builds.load(),rows.load(),drift.load()); }
} bondCounter;
""")
        src[JO]=once(src[JO],"    build(m, result);","""    build(m, result);
    bondCounter.builds.fetch_add(1,std::memory_order_relaxed);
    bondCounter.rows.fetch_add(result.size(),std::memory_order_relaxed);
    for (const auto& row: result) if (row.drift!=0) bondCounter.drift.fetch_add(1,std::memory_order_relaxed);""")
    return src

def labelbuild(v):
    return LAB/("gi-"+v+"-build")

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

cfg={}
for line in (BASE/"CMakeCache.txt").read_text(encoding="utf8").splitlines():
    if ":" in line and "=" in line and not line.startswith(("#","//")):
        k,_,v=line.partition("=")
        cfg[k.split(":")[0]]=v
cc=Path(cfg["CMAKE_CXX_COMPILER"])
env=dict(os.environ,PATH=str(cc.parent)+os.pathsep+os.environ["PATH"])

def run(args,log,cwd=None):
    with (OUT/log).open("wb") as f:
        r=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,cwd=cwd,env=env)
    if r.returncode:
        raise RuntimeError((log,(OUT/log).read_text(errors="replace")[-1600:]))

def text_sha(exe):
    b=exe.read_bytes()
    off,=struct.unpack_from("<I",b,0x3c)
    n,=struct.unpack_from("<H",b,off+6)
    opt,=struct.unpack_from("<H",b,off+20)
    for k in range(n):
        section=off+24+opt+40*k
        if b[section:section+8].rstrip(b"\0")==b".text":
            size,_,raw_size,raw=struct.unpack_from("<IIII",b,section+8)
            assert size<=raw_size
            return hashlib.sha256(b[raw:raw+size]).hexdigest()
    raise ValueError("missing .text")

def build():
    reference=BASE/"src/libkinetica-physics.a"
    for path,sha in FILES.items():
        assert digest(SRC/path)==sha,path
    assert digest(BASE/"validation/studies/study-grip-limit.exe")=="2213c6e2d92b9ce02c29a9ecb8bdaf333faa54b6f765e95d3aeec0aa4bc37b89"
    flags=(BASE/"src/CMakeFiles/kinetica-physics.dir/flags.make").read_text()
    opts={l.split(" = ")[0]:shlex.split(l.split(" = ")[1]) for l in flags.splitlines() if " = " in l}
    (OUT/"compile-flags.txt").write_text(flags,encoding="utf8")
    meta=[]
    for v in (*VARIANTS,"instrument"):
        base=labelbuild(v)
        base.mkdir(exist_ok=True)
        lib=base/"libkinetica-physics.a"
        shutil.copyfile(reference,lib)
        diff=[]
        new=[]
        for path,source in edits(v).items():
            original=(SRC/path).read_bytes().decode()
            if v=="stock" or original!=source:
                cpp=base/path.name
                cpp.write_bytes(source.encode())
                obj=base/(cpp.name+".obj")
                run([cc,*opts["CXX_DEFINES"],*opts["CXX_INCLUDES"],*opts["CXX_FLAGS"],"-c",cpp,"-o",obj],v+"-"+cpp.name+".log",BASE/"src")
                new.append(obj)
            diff.extend(difflib.unified_diff(original.splitlines(True),source.splitlines(True),"a/"+path.as_posix(),"b/"+path.as_posix()))
        (OUT/(v+".patch")).write_bytes("".join(diff).encode())
        run([cc.parent/"llvm-ar.exe","r",lib,*new],v+"-archive.log")
        for name,folder in TARGETS.items():
            if v=="instrument" and name!="study-grip-limit":
                continue
            target=base/"validation"/folder
            target.mkdir(parents=True,exist_ok=True)
            exe=target/(name+".exe")
            wd=BASE/"validation"/folder
            part=wd/"CMakeFiles"/(name+".dir")
            linkargs=shlex.split((part/"linkLibs.rsp").read_text())
            libs=[lib if x.endswith("/libkinetica-physics.a") else x for x in linkargs]
            if libs.count(lib)!=1: raise ValueError(linkargs)
            run([cc,*shlex.split(cfg["CMAKE_CXX_FLAGS_RELWITHDEBINFO"]),
                 "-Wl,--whole-archive",part/"objects.a","-Wl,--no-whole-archive","-o",exe,
                 "-Wl,--out-implib,"+str(target/(name+".dll.a")),"-Wl,--major-image-version,0,--minor-image-version,0",*libs],
                 v+"-"+name+".log",wd)
            old=wd/(name+".exe")
            same=text_sha(exe)==text_sha(old)
            if v=="stock" and not same: raise RuntimeError(("control .text differs",name,text_sha(exe),text_sha(old)))
            meta.append([v,name,digest(exe),text_sha(exe),digest(old),int(same),digest(lib)])
        run([cc,"-std=c++17","-O3","-ffp-contract=off","-I"+str(SRC/"include"),OUT/"impulse.cpp",lib,
             "-lpthread","-o",base/"witness.exe"],v+"-witness-link.log")
        run([base/"witness.exe"],v+"-witness.csv")
        print("built",v,flush=True)
    with (OUT/"manifest.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["variant","exe","sha256","text_sha256","reference_sha256","text_matches_reference","library_sha256"]);w.writerows(meta)

def timings():
    measure=load("prior_measure",LAB/"raw-collision/inline-layout/measure.py")
    measure.OUT=OUT
    measure.BUILDS={v:labelbuild(v) for v in VARIANTS}
    measure.pin()
    measure.physical()
    instrument=labelbuild("instrument")/measure.GRIP
    for mode in ("0","1"):
        e=dict(measure.ENV,KP_COUPLED_FRICTION=mode,KP_GRIP_TIMING="1")
        p=subprocess.run([instrument,"shared8"],capture_output=True,env=e,check=True)
        (OUT/("counts-"+mode+".csv")).write_bytes(p.stderr)
        (OUT/("counter-rig-"+mode+".csv")).write_bytes(p.stdout)
    orders=list(itertools.permutations(VARIANTS))
    random.Random(20261123).shuffle(orders)
    with (OUT/"timings.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["scene","block","slot","order","variant","frames","mean_us","end_drift_mm","max_frame_us"])
        for block,order in enumerate(orders,1):
            for scene in (("grip","pyramid") if block%2 else ("pyramid","grip")):
                for slot,v in enumerate(order,1):
                    us,n,drift,worst=measure.frame(scene,measure.BUILDS[v])
                    w.writerow([scene,block,slot,"/".join(order),v,n,us,drift,worst])
                f.flush()
            if block%6==0: print("timing",block,flush=True)
    for name in VARIANTS[1:]:
        b=labelbuild(name)/"validation/studies"
        cmd=[sys.executable,SRC/"validation/studies/anatomy/guide_verify.py",BASE/"validation/studies",b,OUT/("verify-"+name),"--parity-only"]
        run(cmd,name+"-verification.log")
        print("verified",name,flush=True)

if __name__=="__main__":
    if os.name!="nt": raise SystemExit("native Windows only")
    (build if sys.argv[2]=="build" else timings)()
