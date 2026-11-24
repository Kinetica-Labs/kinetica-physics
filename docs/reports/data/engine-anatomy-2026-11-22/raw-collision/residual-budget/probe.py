"""Record the first failed equilibrium condition per actual public step on the Windows stand."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    p = argparse.ArgumentParser()
    for key in ("src", "build", "trial", "out"):
        p.add_argument("--" + key, required=True, type=Path)
    args = p.parse_args()
    out, trial = args.out, args.trial
    out.mkdir(exist_ok=True, parents=True)
    archive = args.build / "src/libkinetica-physics.a"
    code = (trial / "SoftStep-trial.cpp").read_text(encoding="utf8")
    start, end = code.index("bool slowBodies("), code.index("constexpr int kRechecks")
    section = code[start:end]
    reasons = {}
    lines = section.splitlines()
    for i, line in enumerate(lines):
        if "return false;" in line:
            reasons[str(i+1)] = " ".join(lines[max(0,i-1):i+1]).strip()
            lines[i] = line.replace("return false;", f"return balanceProbe.reject({i+1});")
    section = "\n".join(lines)
    section = section.replace("const Real speed = s.v.length() + radius * s.w.length();",
        "const Real speed = s.v.length() + radius * s.w.length(); balanceProbe.recordSpeed(speed, s.w.length());")
    section = section.replace("const auto& point = m.points[size_t(k)];",
        "const auto& point = m.points[size_t(k)]; balanceProbe.gap = std::max(balanceProbe.gap, double(abs(point.separation)));")
    section = section.replace("if (!slowBodies(bodies, trial, h))", "balanceProbe.loaded = true; if (!slowBodies(bodies, trial, h))")
    section = section.replace("if (!(tangent.length() <= roundoff", "balanceProbe.tilt = std::max(balanceProbe.tilt, double(abs(tangent.dot(m.normal))));\n    if (!(tangent.length() <= roundoff")
    probe = """
struct BalanceProbe {
    int reason = 0;
    bool loaded = false;
    double speed0=0, speed1=0, spin0=0, spin1=0, gap=0, tilt=0;
    bool reject(int r) { if (!reason) reason=r; return false; }
    void recordSpeed(double v,double w) { auto &s=loaded?speed1:speed0, &o=loaded?spin1:spin0; s=std::max(s,v); o=std::max(o,w); }
};
BalanceProbe balanceProbe;
extern "C" void kpBalanceProbe(int* reason, double* v) {
    *reason=balanceProbe.reason;
    const double fields[]={balanceProbe.speed0,balanceProbe.speed1,balanceProbe.spin0,balanceProbe.spin1,balanceProbe.gap,balanceProbe.tilt};
    std::copy(fields,fields+6,v);
    balanceProbe={};
}
"""
    path = out / "probe.cpp"
    path.write_text(code[:start] + probe + section + "\n" + code[end:], encoding="utf8")
    (out / "reasons.json").write_text(json.dumps(reasons,indent=2))
    harness = out / "one.cpp"
    harness.write_text("""
#define main fixtureMain
#include "GripLimit.cpp"
#undef main
extern "C" void kpBalanceProbe(int*,double*);
int main() {
    kp::WorkerPool::shared().resize(1);
    std::puts("phase,step,reason,initial_speed,trial_speed,initial_spin,trial_spin,gap_m,tangent_dot_normal_N,cube_y_m");
    Rig rig(1, {0.5,10,10}, true);
    for(int i=0;i<240;++i) {
        if(!rig.advance(1./480,10,i<120?0:-10)) return 3;
        int reason; double v[6]; kpBalanceProbe(&reason,v);
        std::printf("%s,%d,%d",i<120?"preload":"load",i%120+1,reason);
        for (double value:v) std::printf(",%.15g",value);
        std::printf(",%.15g\\n",rig.read().relativeY);
    }
}
""",encoding="utf8")
    flags=["-O3","-std=c++17","-ffp-contract=off","-DNDEBUG","-I"+str(trial/"overlay/include"),"-I"+str(args.src/"include")]
    exe=out/"probe.exe"
    subprocess.run(["clang++",*flags,"-I"+str(args.src/"validation/studies"),harness,path,
                    trial/"carry.obj",trial/"manifold.obj",trial/"normal.obj",archive,"-lpthread","-o",exe],check=True)
    with (out/"steps.csv").open("wb") as f:
        subprocess.run([exe],env=dict(os.environ,KP_THREADS="1",KP_COUPLED_FRICTION="1"),stdout=f,check=True)


if __name__=="__main__":
    main()
