"""Compile and contrast continuous linear half-kicks with the original native step."""
import csv
import difflib
import hashlib
import importlib.util
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys

LAB=Path(sys.argv[1]).resolve()
OUT=Path(__file__).resolve().parent
BASE=LAB/"combined-core-build"
SOURCE=LAB/"combined-core-src"
SRC=LAB/"sforce-src"
BUILD=LAB/"sforce-build"
REL=Path("src/dynamics/time_stepping/SoftStep.cpp")
OPTIONS=["CMAKE_CXX_COMPILER","CMAKE_MAKE_PROGRAM","CMAKE_BUILD_TYPE","CMAKE_CXX_FLAGS","CMAKE_CXX_FLAGS_RELWITHDEBINFO",
         "CMAKE_EXE_LINKER_FLAGS","CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO","KP_VIEWER"]

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def cache(d):
    entries={}
    for line in (d/"CMakeCache.txt").read_text(encoding="utf8").splitlines():
        k,sep,v=line.partition("=")
        if sep and ":" in k and not k.startswith(("#","//")):
            entries[k.split(":")[0]]=v
    return entries

cfg=cache(BASE)
cc=Path(cfg["CMAKE_CXX_COMPILER"])
env={k:v for k,v in os.environ.items() if not k.startswith("KP_") and k not in ("BENCH","SCENE")}
env.update(PATH=str(cc.parent)+os.pathsep+env["PATH"],KP_THREADS="1",KP_COUPLED_FRICTION="1",KP_GRIP_TIMING="0",KP_GRIP_PROFILE="0")

def execute(cmd,log,accepted=(0,),environ=env):
    with (OUT/log).open("wb") as f:
        p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=environ,timeout=1200)
    if p.returncode not in accepted:
        raise RuntimeError((log,p.returncode,(OUT/log).read_text(errors="replace")[-1800:]))
    return p.returncode

def prepare():
    assert sha(SOURCE/REL)=="73e82c9839c1019ac5d04567c612c8c8dcaa25a5ad19add9945b4f3ab65a8998"
    original=(SOURCE/REL).read_bytes().decode()
    start=original.index("// Начало подшага: сопротивление")
    end=original.index("constexpr int kRechecks",start)
    replacement="""// Непрерывное поступательное ускорение, м/с². Сила в центре масс и поле тяжести
// остаются постоянными в течение одного подшага.
Vector3 linearAcceleration(const Body& body, Real inverseMass, Vector3 gravity) {
    return gravity + inverseMass * body.force;
}

void applyForces(StepArray<SolverBody>& states, const std::vector<Body>& bodies, Vector3 gravity, Real h) {
    for (size_t i = 0; i < states.size(); ++i) {
        SolverBody& state = states[i];
        const Body& body = bodies[i];
        if (state.inverseMass <= 0) continue;
        state.v = (1 / (1 + h * body.linearDamping)) * state.v + (h / 2) * linearAcceleration(body, state.inverseMass, gravity);
        state.w = (1 / (1 + h * body.angularDamping)) * state.w;
        if (isForced(body)) state.w += h * state.spinFrom(body.torque);
    }
}

// Вторая половина непрерывного толчка приходится на конец переноса. Затем решатель
// восстанавливает ограничения скорости в новом положении.
void finishLinearForces(StepArray<SolverBody>& states, const std::vector<Body>& bodies, Vector3 gravity, Real h) {
    for (size_t i = 0; i < states.size(); ++i) {
        SolverBody& state = states[i];
        if (state.inverseMass > 0) state.v += (h / 2) * linearAcceleration(bodies[i], state.inverseMass, gravity);
    }
}

"""
    changed=original[:start]+replacement+original[end:]
    call="        live.solveAndMove(moving, h, (substeps - substep) * h);\n"
    assert changed.count(call)==1
    changed=changed.replace(call,call+"        finishLinearForces(states, bodies, gravity, h);\n")
    if not SRC.exists():
        shutil.copytree(SOURCE,SRC)
        (SRC/REL).write_bytes(changed.encode())
    baseline={p.relative_to(SOURCE):sha(p) for p in SOURCE.rglob("*") if p.is_file()}
    installed={p.relative_to(SRC):sha(p) for p in SRC.rglob("*") if p.is_file()}
    expected=dict(baseline)
    expected[REL]=hashlib.sha256(changed.encode()).hexdigest()
    assert installed==expected
    diff=difflib.unified_diff(original.splitlines(True),changed.splitlines(True),"a/"+REL.as_posix(),"b/"+REL.as_posix())
    (OUT/"split.patch").write_bytes("".join(diff).encode())
    execute(["cmake","-S",SRC,"-B",BUILD,"-G",cfg["CMAKE_GENERATOR"],*["-D"+k+"="+cfg[k] for k in OPTIONS]],"configure.log")
    assert all(cache(BUILD)[k].replace("\\","/")==cfg[k].replace("\\","/") for k in OPTIONS)
    group=["study-grip-limit","study-oblique-grip","study-groove-friction","study-state-hash","compare-ours","unit-tests",
           "FloatingBox","GearTrain","study-slip-to-roll","study-rest-overlay","study-playback-clock","study-hull-events",
           "study-hull-motion","study-collision-contract","study-bucket-pairs"]
    execute(["cmake","--build",BUILD,"--target",*group,"--parallel","8"],"build.log")
    with (OUT/"manifest.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["variant","kind","name","sha256"])
        for v,s,b in (("stock",SOURCE,BASE),("split",SRC,BUILD)):
            for path in [s/REL,b/"src/libkinetica-physics.a",b/"validation/studies/study-grip-limit.exe",b/"validation/engines/compare-ours.exe"]:
                w.writerow([v,"compiled input or result",path.name,sha(path)])
    print("build complete",flush=True)

def probes():
    spec=importlib.util.spec_from_file_location("verifier",SOURCE/"validation/studies/anatomy/guide_verify.py")
    verify=importlib.util.module_from_spec(spec);spec.loader.exec_module(verify)
    with (OUT/"physical-summary.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["variant","coupled","case","rows","oracle_pass","exit","sha256"])
        for name,stem,args,n,width in verify.CASES:
            if name in ("state","trace-order","islands","island-joints"):
                continue
            for mode in (0,1):
                for v,b in (("stock",BASE),("split",BUILD)):
                    proc=subprocess.run([b/"validation/studies"/(stem+".exe"),*args],capture_output=True,env=dict(env,KP_COUPLED_FRICTION=str(mode)),timeout=400)
                    verify.validate(name,proc,n,width)
                    dest=OUT/f"{name}-{mode}-{v}.csv"
                    dest.write_bytes(proc.stdout)
                    records=list(csv.DictReader(io.StringIO(proc.stdout.decode("ascii"))))
                    w.writerow([v,mode,name,len(records),sum(r["pass"]=="1" for r in records),proc.returncode,sha(dest)])
                    f.flush()
    for v,s,b in (("stock",SOURCE,BASE),("split",SRC,BUILD)):
        for name,path in (("transport",OUT/"transport_oracles.cpp"),("free",OUT/"free.cpp")):
            exe=b/(name+"-sforce.exe")
            execute([cc,"-std=c++17","-O3","-ffp-contract=off","-I"+str(s/"include"),path,b/"src/libkinetica-physics.a","-lpthread","-o",exe],
                    v+"-"+name+"-link.log")
            execute([exe],v+"-"+name+".csv")
    # The unit executable has the same command line as in CTest.
    bash=Path("C:/Program Files/Git/bin/bash.exe")
    gen=BUILD/"tests/CTestTestfile.cmake"
    original=gen.read_text(encoding="utf8")
    for key in ("no-libm","no-libm-scenes"):
        before=f'add_test({key} "'
        if bash.as_posix() not in original.split(before)[1].splitlines()[0]:
            original=original.replace(before,before+bash.as_posix()+'" "')
    gen.write_text(original,encoding="utf8")
    nm=BUILD/"ctest-nm"
    nm.mkdir(exist_ok=True)
    shutil.copy2(cc.parent/"llvm-nm.exe",nm/"nm.exe")
    status=execute(["ctest","--test-dir",BUILD,"--output-on-failure","-j","2"],"ctest.log",(0,8),dict(env,PATH=str(nm)+os.pathsep+env["PATH"]))
    print("physical probes complete; CTest exit",status,flush=True)

if __name__=="__main__":
    if os.name!="nt": raise SystemExit("native Windows only")
    prepare()
    probes()
