"""Inspect the timed Windows PE. Compiler spill comments come from matching source and flags."""
import csv
import gzip
import json
from pathlib import Path
import re
import shutil
import subprocess
from native import build, flags, out, release, sha, src, work

units = {
    "StepMemory::take": "math/StepMemory",
    "DirectJoints::assemble": "dynamics/time_stepping/DirectJoints",
    "DirectJoints::push": "dynamics/time_stepping/DirectJoints",
    "CoupledFriction::solve": "dynamics/time_stepping/CoupledFriction",
    "SoftStep::step": "dynamics/time_stepping/SoftStep",
    "(anonymous namespace)::JointRows::build": "dynamics/time_stepping/SoftStep",
}
def capture(args, **kwargs):
    return subprocess.check_output(list(map(str,args)),text=True,encoding="utf8",errors="replace",**kwargs)
def syms(demangle):
    cmd=["llvm-nm","-n"]
    if demangle: cmd+=["-C"]
    lines=capture([*cmd,release]).splitlines()
    return {int(m[1],16):m[2] for l in lines if (m:=re.fullmatch(r"([a-fA-F0-9]+) [TtWw] (.+)",l))}

def main():
    raw, demangled=syms(False),syms(True)
    starts=sorted(raw)
    gccflags=[*flags,"-Wall","-Wextra","-Wpedantic","@CMakeFiles/kinetica-physics.dir/includes_CXX.rsp"]
    all_asm={}
    for unit in set(units.values()):
        slug=unit.split("/")[-1]
        target=work/(slug+".s")
        subprocess.run(["clang++",*gccflags,"-S","-fverbose-asm",str(src/"src"/(unit+".cpp")),"-o",str(target)],
                       cwd=build/"src",check=True)
        all_asm[unit]=target.read_text(encoding="utf8",errors="replace")
    with gzip.open(out/"linked-kernels.asm.gz","wt",encoding="utf8") as archive, (out/"assembly.csv").open("w",newline="") as df, (out/"compiler-spills.csv").open("w",newline="") as sf:
        w,s=csv.writer(df),csv.writer(sf)
        w.writerow(["function","begin_va","instructions","direct_rsp_memory_operands","calls","scalar_fp_arithmetic","packed_fp_arithmetic","compiler_spill_sites","compiler_reload_sites"])
        s.writerow(["function","annotation","code"])
        for fn,unit in units.items():
            addr,= [a for a,n in demangled.items() if n.startswith("kp::"+fn+"(")]
            end=starts[starts.index(addr)+1]
            assembly=capture(["llvm-objdump","-d","--no-show-raw-insn",f"--start-address={addr}",f"--stop-address={end}",release])
            archive.write("\n"+fn+"\n"+assembly)
            ins=[m.groups() for l in assembly.splitlines() if (m:=re.match(r"\s*[0-9a-f]+:\s+(\w+)\s*(.*)",l))]
            ins=[(op,arg.split("#")[0]) for op,arg in ins if not op.startswith(("nop","int3"))]
            stack=sum("(%rsp" in arg and not op.startswith("lea") for op,arg in ins)
            ops=r"v?(?:add|sub|mul|div|sqrt|min|max|fmadd\d*)"
            scalar=sum(bool(re.fullmatch(ops+"s[sd]",op)) for op,_ in ins)
            packed=sum(bool(re.fullmatch(ops+"p[sd]",op)) for op,_ in ins)
            rawname=raw[addr]
            lines=all_asm[unit].splitlines()
            first=next(i for i,l in enumerate(lines) if l.split("#",1)[0].strip().rstrip(":").strip('"')==rawname)
            last=next(i for i in range(first+1,len(lines)) if lines[i].strip().startswith(".seh_endproc"))
            comments=[]
            for l in lines[first:last]:
                m=re.search(r"#\s*(\d+-byte (?:Folded )?(Spill|Reload))",l)
                if m:
                    comments.append(m[2])
                    s.writerow([fn,m[1],l.split("#",1)[0].strip()])
            w.writerow([fn,hex(addr),len(ins),stack,sum(op.startswith("call") for op,_ in ins),scalar,packed,comments.count("Spill"),comments.count("Reload")])
    (out/"assembly-inputs.json").write_text(json.dumps({"release_sha256":sha(release),
        "source_hashes":{u:sha(src/"src"/(u+".cpp")) for u in sorted(set(units.values()))},
        "compiler":capture(["clang++","--version"]).splitlines()[0],
        "direct_rsp_memory_operands":"Static instructions with an explicit effective address based on rsp, excluding LEA and padding. Implicit push/pop/call/ret and rbp are excluded.",
        "arithmetic":"SSE/AVX floating point add/sub/mul/div/sqrt/min/max/fmadd. Data moves and shuffles excluded. Counts are static, not event durations."},indent=2))
if __name__=="__main__":main()
