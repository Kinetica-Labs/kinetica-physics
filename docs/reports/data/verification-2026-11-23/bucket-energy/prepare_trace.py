"""Instrument a stand-only copy. Never read Body momenta between solver stages for kinetic energy."""
from pathlib import Path
import sys

repo, work = map(Path, sys.argv[1:])
work.mkdir(parents=True, exist_ok=True)
soft = (repo / "src/dynamics/time_stepping/SoftStep.cpp").read_text()
functions = """
extern int auditFrame, auditAngle;
namespace kp {
static void auditEnergy(const char* stage, const std::vector<Body>& bodies, const StepArray<SolverBody>& states) {
    if (auditFrame != 4860 && !((auditAngle == 16 && auditFrame == 33) ||
        (auditAngle == 0 && auditFrame == 28) || (auditAngle == -16 && auditFrame == 34))) return;
    double k = 0, u = 0;
    for (size_t i = 0; i < bodies.size(); ++i) {
        const auto& s = states[i];
        const auto& b = bodies[i];
        if (s.inverseMass == 0) continue;
        k += .5 * (s.v.dot(s.v) / s.inverseMass + s.w.dot(step_motion::momentumOf(b,s.w)));
        u += 9.81 * b.position.y / s.inverseMass;
    }
    std::fprintf(stderr,"%d,%d,%s,%.17g,%.17g\\n",auditAngle,auditFrame,stage,k,u);
}
}
"""
def replace_once(text, before, after):
    if text.count(before) != 1: raise ValueError(before)
    return text.replace(before, after)

soft = replace_once(soft, "namespace kp {\nnamespace {", "#include <cstdio>\n" + functions + "\nnamespace kp {\nnamespace {")
for needle,stage,scope in [
    ("StepArray<SolverBody> states = solverBodiesOf(bodies);","start","bodies, states"),
    ("step_motion::applyForces(states, bodies, gravity, h);","gravity","bodies, states"),
    ("notify(observer, StepStage::WarmStart);","warm-start","bodies, states"),
    ("impact(moving, h), notify(observer, StepStage::Normals);","biased-normals","moving.bodies, moving.states"),
    ("notify(observer, StepStage::Recheck);\n            missed.forget()", "recheck", "moving.bodies, moving.states"),
    ("notify(observer, StepStage::Motion);","motion","moving.bodies, moving.states"),
    ("moving.undo();", "rollback", "moving.bodies, moving.states"),
    ("notify(observer, StepStage::Normals);\n        if (coupledFriction)","unbiased-normals","bodies, states"),
    ("notify(observer, StepStage::Friction);","friction","bodies, states"),
]:
    call = f'auditEnergy("{stage}", {scope});'
    if "\n" in needle:
        left,right=needle.split("\n",1)
        soft=replace_once(soft,needle,left+" "+call+"\n"+right)
    else:
        soft=replace_once(soft,needle,needle+" "+call)
(work/"soft-trace.cpp").write_text(soft)
harness=(repo/"validation/studies/anatomy/bucket_energy.cpp").read_text()
harness=replace_once(harness,'#include "../../../examples/scenes/StandardScenes.h"','#include "StandardScenes.h"')
harness=replace_once(harness,"namespace {","int auditFrame = -1, auditAngle = 0;\nnamespace {")
harness=replace_once(harness,"if (i > 0) world.step", "auditFrame = i; auditAngle = int(degrees);\n        if (i > 0) world.step")
(work/"bucket-trace.cpp").write_text(harness)
# Контрфактический опыт. До интересующего кадра оба пути исполняют одинаковые операции.
needle="normals.solve(contacts, moving.bodies, moving.states, true, h, pushSpeed);"
skip="(auditAngle == 16 && auditFrame == 33) || (auditAngle == 0 && auditFrame == 28) || (auditAngle == -16 && auditFrame == 34)"
(work/"no-position-bias.cpp").write_text(replace_once(soft,needle,needle.replace("true", "!("+skip+")")))
(work/"early-frames.cpp").write_text(replace_once(harness,"i <= 86 * 60","i <= 34"))
