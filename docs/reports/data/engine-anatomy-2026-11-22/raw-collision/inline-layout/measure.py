"""Native Windows paired timing after all compiler processes have finished."""
import csv
import ctypes
import hashlib
import io
import itertools
import json
import math
import os
from pathlib import Path
import platform
import random
import re
import subprocess
import sys
from datetime import datetime, timezone

OUT = Path(__file__).resolve().parent
LAB = Path(sys.argv[1]).resolve()
BUILDS = {v: LAB / ("combined-core-build" if v == "stock" else "il-" + v + "-build")
          for v in ("stock", "force", "align", "both")}
SEED = 20261122
ENV = {k:v for k,v in os.environ.items() if not k.startswith("KP_") and k not in {"BENCH","SCENE"}}
ENV.update(KP_THREADS="1", KP_GRIP_PROFILE="0")
GRIP = Path("validation/studies/study-grip-limit.exe")
PYRAMID = Path("validation/engines/compare-ours.exe")

def utc():
    return datetime.now(timezone.utc).isoformat()

def pin():
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    handle = ctypes.c_void_p(-1)
    mask = ctypes.c_size_t(0x100)
    if not api.SetProcessAffinityMask(handle, mask):
        raise ctypes.WinError(ctypes.get_last_error())
    actual, system = ctypes.c_size_t(), ctypes.c_size_t()
    if not api.GetProcessAffinityMask(handle,ctypes.byref(actual),ctypes.byref(system)) or actual.value != mask.value:
        raise RuntimeError("affinity verification failed")
    return actual.value

def physical():
    sys.path.insert(0,str(LAB/"combined-core-src/validation/studies/anatomy"))
    import guide_verify
    rows = []
    reference = {}
    for mode in (0,1):
        for variant, build in BUILDS.items():
            p = subprocess.run([build/GRIP,"shared8"],env=dict(ENV,KP_GRIP_TIMING="0",KP_COUPLED_FRICTION=str(mode)),
                               capture_output=True,timeout=120)
            guide_verify.validate("shared8",p,36,17)
            digest = hashlib.sha256(p.stdout).hexdigest()
            reference.setdefault(mode,digest)
            data = list(csv.DictReader(io.StringIO(p.stdout.decode("ascii"))))
            rows.append([variant, mode, len(data), sum(r["pass"]=="1" for r in data),digest, int(digest==reference[mode])])
            (OUT/f"physical-{mode}-{variant}.csv").write_bytes(p.stdout)
    with (OUT/"physical.csv").open("w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["variant","coupled","cases","oracle_pass","sha256","same_stdout_as_stock"])
        w.writerows(rows)

def frame(scene,build):
    if scene == "grip":
        p = subprocess.run([build/GRIP,"shared8"],env=dict(ENV,KP_GRIP_TIMING="1",KP_COUPLED_FRICTION="1"),
                           stdout=subprocess.PIPE,check=True,timeout=90)
        row, = list(csv.DictReader(io.StringIO(p.stdout.decode("ascii"))))
        if row["engine"]!="kinetica-shared8" or row["frames"]!="360":
            raise RuntimeError("incorrect grip timer")
        return float(row["us_per_frame"]),360,row["end_drift_mm"],""
    p = subprocess.run([build/PYRAMID,"realtime","8"],env=dict(ENV,KP_COUPLED_FRICTION="0",BENCH="7"),
                       stdout=subprocess.PIPE,check=True,timeout=120)
    m = re.fullmatch(rb"[^\r\n]*\b140:\s*([\d.]+)\s+[^\r\n]*?\s([\d.]+)\s*",p.stdout)
    if not m:
        raise RuntimeError(("incorrect pyramid timer",p.stdout))
    return 1000*float(m[1]),60,"",1000*float(m[2])

def measure():
    mask = pin()
    blocks = list(itertools.permutations(BUILDS))
    random.Random(SEED).shuffle(blocks)
    metadata = dict(utc_start=utc(),host=platform.node(),os=platform.platform(),
                    cpu=os.environ.get("PROCESSOR_IDENTIFIER"),affinity=mask,threads=1,seed=SEED,
                    design="all 24 permutations once per scene; scenes alternate first position in successive blocks",
                    grip="shared8, 1/480 s x 8, mu=.5, squeeze=10 N/jaw, load=10 N; 15 preload, 15 loaded frames x 24 fixtures, 3 warmups",
                    pyramid="BENCH=7, 140 dynamic cubes; 60 frames at 60 Hz, realtime 8, sleep off, coupled off, time resolution 10 us/frame",
                    decision="per-scene median of paired time ratios <= .98 and at least 18/24 faster than stock on BOTH scenes, equal physical rows; a 4x explanation requires a ratio near .25",
                    state_note="full-precision state hashes are for 27 standard scenes at 60 frames; shared8 output uses the study's printed precision")
    (OUT/"method.json").write_text(json.dumps(metadata,indent=2))
    physical()
    with (OUT/"timings.csv").open("w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["scene","block","slot","order","variant","frames","mean_us","end_drift_mm","max_frame_us"])
        for i,order in enumerate(blocks,1):
            scenes = ("grip","pyramid") if i%2 else ("pyramid","grip")
            for scene in scenes:
                for slot,v in enumerate(order,1):
                    us,n,drift,worst = frame(scene,BUILDS[v])
                    if not math.isfinite(us) or us<=0:
                        raise ValueError("invalid frame timing")
                    w.writerow([scene,i,slot,"/".join(order),v,n,us,drift,worst])
                f.flush()
            if i%6==0:
                print("completed timing block",i,flush=True)
    metadata["utc_end"]=utc()
    (OUT/"method.json").write_text(json.dumps(metadata,indent=2))

def parity():
    script = LAB/"combined-core-src/validation/studies/anatomy/guide_verify.py"
    for name,build in BUILDS.items():
        if name=="stock":
            continue
        dest = OUT/("verify-"+name)
        proc = subprocess.run([sys.executable,script,BUILDS["stock"]/"validation/studies",build/"validation/studies",dest,
                               "--parity-only"],env=ENV,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=1200)
        (OUT/(name+"-verification.log")).write_bytes(proc.stdout)
        if not (dest/"parity.csv").is_file():
            raise RuntimeError((name,proc.stdout[-1500:]))
        print("verification",name,proc.returncode,flush=True)

if __name__=="__main__":
    if sys.argv[2]=="time":
        measure()
    elif sys.argv[2]=="verify":
        pin()
        parity()
    else:
        raise ValueError(sys.argv[2])
