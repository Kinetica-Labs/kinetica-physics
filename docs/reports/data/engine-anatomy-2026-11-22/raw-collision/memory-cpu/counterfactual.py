"""Four native executables, balanced by all 24 launch orders on the same logical CPU."""
import csv
import ctypes
import hashlib
import io
import itertools
import json
import os
from pathlib import Path
import random
import shutil
import struct
import subprocess
from native import archive, build, env, flags, lab, out, release, sha, src, work

cwd=build/"validation/studies"
adapter=cwd/"CMakeFiles/study-grip-limit.dir/objects.a"
libs="-lpthread -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32".split()
sources=src/"src/dynamics/time_stepping"
def link(lib,exe):
    subprocess.run(["clang++",*flags,"-Wl,--whole-archive",adapter,"-Wl,--no-whole-archive","-o",exe,
                    "-Wl,--major-image-version,0,--minor-image-version,0",lib,*libs],check=True,cwd=cwd)
def text_hash(exe):
    data=exe.read_bytes()
    pe,=struct.unpack_from("<I",data,0x3c)
    assert data[pe:pe+4]==b"PE\0\0"
    sections,=struct.unpack_from("<H",data,pe+6)
    optional,=struct.unpack_from("<H",data,pe+20)
    for i in range(sections):
        header=pe+24+optional+i*40
        if data[header:header+8].rstrip(b"\0")==b".text":
            size,offset=struct.unpack_from("<II",data,header+16)
            return hashlib.sha256(data[offset:offset+size]).hexdigest()
    raise ValueError("missing .text")
def variant(key, files):
    folder=work/key
    folder.mkdir(exist_ok=True)
    obj=[]
    for name,source,additional in files:
        target=folder/(name+".cpp.obj")
        subprocess.run(["clang++",*flags,"-Wall","-Wextra","-Wpedantic",
                        "@CMakeFiles/kinetica-physics.dir/includes_CXX.rsp",*additional,
                        "-c",source,"-o",target],cwd=build/"src",check=True)
        obj.append(target)
    lib=folder/"libkinetica-physics.a"
    shutil.copy2(archive,lib)
    before=subprocess.check_output(["llvm-ar","t",lib]).decode().splitlines()
    assert all(before.count(o.name)==1 for o in obj)
    subprocess.run(["llvm-ar","r",lib,*obj],check=True)
    after=subprocess.check_output(["llvm-ar","t",lib]).decode().splitlines()
    assert before==after
    exe=work/(key+".exe")
    link(lib,exe)
    return exe
def bench():
    direct=sources/"DirectJoints.cpp"
    friction=sources/"CoupledFriction.cpp"
    old="bonds_.push_back(bond), impulses_.push_back(impulse);"
    string=direct.read_text(encoding="utf8")
    assert string.count(old)==1
    special=work/"reserve-ten.cpp"
    special.write_text(string.replace(old,"if (bonds_.capacity() == 0) bonds_.reserve(10); "+old),encoding="utf8")
    control=work/"relinked.exe"
    link(archive,control)
    th=text_hash(release)
    assert text_hash(control)==th,"control .text differs from the timed executable"
    baseline_obj=variant("recompiled",[("DirectJoints",direct,[]),("CoupledFriction",friction,[])])
    assert text_hash(baseline_obj)==th,"source recompilation differs in .text"
    off=["-fno-slp-vectorize","-fno-vectorize"]
    paths={"double":release,"float":lab/"precision-audit-build/validation/studies/study-grip-limit.exe",
           "reserve10":variant("reserve10",[("DirectJoints",special,[])]),
           "scalar-kernels":variant("scalar-kernels",[("DirectJoints",direct,off),("CoupledFriction",friction,off)])}
    inputs={k:{"binary_sha256":sha(v),"text_sha256":text_hash(v)} for k,v in paths.items()}
    for key,v in paths.items():
        phys=subprocess.run([v,"shared8"],env=dict(env,KP_GRIP_TIMING="0",KP_GRIP_PROFILE="0"),capture_output=True,timeout=90)
        rr=list(csv.DictReader(io.StringIO(phys.stdout.decode())))
        assert len(rr)==36 and all(r["complete"]=="1" for r in rr)
        (out/(key+"-oracle.csv")).write_bytes(phys.stdout)
        inputs[key]["passed"]=sum(int(r["pass"]) for r in rr)
        inputs[key]["oracle_sha256"]=hashlib.sha256(phys.stdout).hexdigest()
    api=ctypes.WinDLL("kernel32",use_last_error=True)
    if not api.SetProcessAffinityMask(ctypes.c_void_p(-1),ctypes.c_size_t(0x100)):
        raise ctypes.WinError(ctypes.get_last_error())
    actual,system=ctypes.c_size_t(),ctypes.c_size_t()
    assert api.GetProcessAffinityMask(ctypes.c_void_p(-1),ctypes.byref(actual),ctypes.byref(system)) and actual.value==0x100
    orders=list(itertools.permutations(paths))
    random.Random(20261123).shuffle(orders)
    (out/"counterfactual-method.json").write_text(json.dumps({"variants":inputs,"control_relinked_and_recompiled_text_sha256":th,
        "affinity":actual.value,"single_solver_thread":1,"seed":20261123,"design":"all 24 orders exactly once; eight public updates per frame; timing excludes all construction, preload and warmups",
        "reserve10":"Diagnostic only. Reserve the known ten hard scalar joint rows on first insertion in DirectJoints. No arbitrary reserve size in production.",
        "scalar-kernels":"Disable automatic loop/SLP vectorization ONLY in DirectJoints.cpp and CoupledFriction.cpp; compiler, ABI, libraries and other units stay fixed."},indent=2))
    with (out/"counterfactual-time.csv").open("w",newline="") as dest:
        w=csv.writer(dest);w.writerow(["block","slot","variant","frames","mean_us","drift_mm"])
        for block,order in enumerate(orders,1):
            for slot,key in enumerate(order,1):
                p=subprocess.run([paths[key],"shared8"],env=dict(env,KP_GRIP_TIMING="1",KP_GRIP_PROFILE="0"),check=True,capture_output=True,timeout=90)
                row,=csv.DictReader(io.StringIO(p.stdout.decode()))
                assert int(row["frames"])==360 and float(row["us_per_frame"])>0
                w.writerow([block,slot,key,row["frames"],row["us_per_frame"],row["end_drift_mm"]])
            dest.flush()
    # Count allocations with the same single source substitution, outside all timing runs.
    from native import inc, mem, study
    ex=work/"reserve10-count.exe"
    subprocess.run(["clang++",*flags,"-std=c++17",*inc,study/"anatomy/kinetica_memory.cpp",mem/"MemoryCounters.cpp",
                    work/"reserve10/libkinetica-physics.a","-lpthread","-o",ex],check=True)
    with (out/"reserve10-counts.csv").open("wb") as o,(out/"reserve10-counts.stderr").open("wb") as e:
        subprocess.run([ex,"coupled"],check=True,stdout=o,stderr=e,env=env)
if __name__=="__main__":bench()
