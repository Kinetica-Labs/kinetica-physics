"""Count direct calls and tail jumps to the selected functions in the linked PE."""
import csv
import hashlib
import re
import struct
import subprocess
from pathlib import Path
import sys

OUT=Path(__file__).resolve().parent
LAB=Path(sys.argv[1]).resolve()
TOOLS=Path(sys.argv[2]).resolve()

def counters(exe):
    names=subprocess.check_output([TOOLS/"llvm-nm.exe","-n","-C",exe]).decode(errors="replace")
    targets={}
    for line in names.splitlines():
        m=re.match(r"^([0-9a-fA-F]+)\s+[tTwW]\s+(.+)$",line)
        if not m:
            continue
        name=m[2]
        if (re.match(r"^kp::(Vector3::(dot|cross|lengthSquare|length|scaled)|SolverBody::(spinFrom|push|twist|inverseMassAlong))\(",name)
                or re.match(r"^kp::operator(\+|-|\*|\+=|-=)\((kp::Vector3|double, kp::Vector3)",name)):
            targets[int(m[1],16)]=name
    calls=0
    records=[]
    caller=""
    p=subprocess.Popen([TOOLS/"llvm-objdump.exe","-d","-C","--no-show-raw-insn",exe],
                       stdout=subprocess.PIPE,text=True,encoding="utf8",errors="replace")
    for line in p.stdout:
        m=re.match(r"^\s*[0-9a-f]+ <(.+)>:$",line.rstrip())
        if m:
            caller=m[1]
            continue
        m=re.match(r"^\s*([0-9a-f]+):\s+(callq?|jmpq?)\s+0x([0-9a-f]+)\b",line)
        if not m:
            continue
        calls+=int(m[2].startswith("call"))
        target=targets.get(int(m[3],16))
        if target:
            records.append([m[1],caller,m[2],m[3],target])
    if p.wait() or calls<1000:
        raise RuntimeError(("disassembly extraction",exe,calls))
    return targets,records,calls

def code_section(exe):
    buf=exe.read_bytes()
    pe,=struct.unpack_from("<I",buf,0x3c)
    count,=struct.unpack_from("<H",buf,pe+6)
    opt,=struct.unpack_from("<H",buf,pe+20)
    for k in range(count):
        off=pe+24+opt+40*k
        if buf[off:off+8].rstrip(b"\0")==b".text":
            size,_,raw,pos=struct.unpack_from("<IIII",buf,off+8)
            if size>raw:
                raise ValueError("unexpected executable section")
            return size,hashlib.sha256(buf[pos:pos+size]).hexdigest()
    raise ValueError("missing PE .text")

with (OUT/"assembly.csv").open("w",newline="") as sf,(OUT/"sites.csv").open("w",newline="") as of:
    sums,sites=csv.writer(sf),csv.writer(of)
    sums.writerow(["variant","scene","text_bytes","text_sha256","all_direct_calls","selected_symbols","selected_calls","selected_tail_jumps"])
    sites.writerow(["variant","scene","site","caller","instruction","target_address","target"])
    for v in ("stock","force","align","both"):
        build=LAB/("combined-core-build" if v=="stock" else "il-"+v+"-build")
        for scene,path in [("grip","validation/studies/study-grip-limit.exe"),("pyramid","validation/engines/compare-ours.exe")]:
            exe=build/path
            targets,records,calls=counters(exe)
            size,digest=code_section(exe)
            sums.writerow([v,scene,size,digest,calls,len(targets),sum(r[2].startswith("call") for r in records),sum(r[2].startswith("jmp") for r in records)])
            sites.writerows([[v,scene,*r] for r in records])
