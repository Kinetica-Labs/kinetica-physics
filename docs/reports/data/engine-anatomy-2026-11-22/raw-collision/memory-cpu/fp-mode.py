"""Test gradual underflow versus flush-to-zero in one fixed PE for each precision."""
import csv
import ctypes
import hashlib
import io
import itertools
import json
import os
import random
import subprocess
from native import build, env, flags, lab, out, src, work, sha

def main():
    paths={}
    info={}
    for kind,source,b in [("double",src,build),("float",lab/"precision-audit-src",lab/"precision-audit-build")]:
        exe=work/("fp-"+kind+".exe")
        obj=work/("fp-adapter-"+kind+".o")
        adapter=source/"validation/studies/GripLimit.cpp"
        subprocess.run(["clang++",*flags,"-Dmain=originalGripMain",f"-I{source/'include'}","-c",adapter,"-o",obj],check=True)
        subprocess.run(["clang++",*flags,out/"fp-mode.cpp",obj,b/"src/libkinetica-physics.a","-lpthread","-o",exe],check=True)
        paths[kind]=exe
        info[kind]={"exe":sha(exe),"archive":sha(b/"src/libkinetica-physics.a"),
                    "flags":(b/"src/CMakeFiles/kinetica-physics.dir/flags.make").read_text(),
                    "source":sha(source/"include/math/math_functions.h")}
    options=list(itertools.product(paths,("ieee","flush")))
    def go(kind,mode,timing):
        return subprocess.run([paths[kind],"shared8"],env=dict(env,KP_AUDIT_FLUSH=str(int(mode=="flush")),
                KP_GRIP_TIMING=str(int(timing)),KP_GRIP_PROFILE="0"),capture_output=True,timeout=90)
    for kind,mode in options:
        p=go(kind,mode,False)
        rr=list(csv.DictReader(io.StringIO(p.stdout.decode())))
        assert len(rr)==36 and all(r["complete"]=="1" for r in rr)
        filename=f"fp-{kind}-{mode}-oracle.csv"
        (out/filename).write_bytes(p.stdout)
        if mode=="ieee":
            assert p.stdout==(out/(kind+"-oracle.csv")).read_bytes()
        info[kind][mode]={"passed":sum(int(r["pass"]) for r in rr),
                          "oracle_sha":hashlib.sha256(p.stdout).hexdigest(),"csr":p.stderr.decode().strip()}
    api=ctypes.WinDLL("kernel32",use_last_error=True)
    if not api.SetProcessAffinityMask(ctypes.c_void_p(-1),ctypes.c_size_t(0x100)):raise ctypes.WinError()
    actual,system=ctypes.c_size_t(),ctypes.c_size_t()
    assert api.GetProcessAffinityMask(ctypes.c_void_p(-1),ctypes.byref(actual),ctypes.byref(system)) and actual.value==0x100
    orders=list(itertools.permutations(options));random.Random(113023).shuffle(orders)
    info["experiment"]={"affinity":actual.value,"orders":len(orders),"solver_threads":1,"seed":113023,
        "flush":"set both FTZ (bit 15) and DAZ (bit 6) in MXCSR on the main solver thread; IEEE clears both. Starts with cleared status. Only the environment variable differs for two launches of the same executable."}
    (out/"fp-method.json").write_text(json.dumps(info,indent=2))
    with (out/"fp-time.csv").open("w",newline="") as dest:
        w=csv.writer(dest);w.writerow(["block","slot","precision","mode","frames","mean_us","drift_mm","csr"])
        for block,order in enumerate(orders,1):
            for slot,(kind,mode) in enumerate(order,1):
                p=go(kind,mode,True);p.check_returncode()
                row,=csv.DictReader(io.StringIO(p.stdout.decode()))
                assert int(row["frames"])==360 and float(row["us_per_frame"])>0
                w.writerow([block,slot,kind,mode,row["frames"],row["us_per_frame"],row["end_drift_mm"],p.stderr.decode().strip()])
            dest.flush()
if __name__=="__main__":main()
