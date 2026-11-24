"""Install the projected-force prototype into the isolated LTO source tree on the Windows stand."""
import argparse
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("baseline", type=Path)
    p.add_argument("candidate", type=Path)
    p.add_argument("algorithm", type=Path)
    args = p.parse_args()
    header = Path("include/dynamics/ContactForces.h")
    src = (args.baseline/header).read_text(encoding="utf8")
    marker = "    const Entry* begin() const"
    assert src.count(marker) == 1
    method = """    void reviseFriction(const ContactId& key, Vector3 tangent, Real twist) {
        auto first = std::lower_bound(entries_.begin(), entries_.end(), key,
            [](const Entry& entry, const ContactId& id) { return entry.first.pairBefore(id); });
        for (auto it = first; it != entries_.end() && it->first.samePair(key); ++it) {
            it->second.tangent = tangent;
            it->second.twist = twist;
        }
    }
"""
    (args.candidate/header).write_text(src.replace(marker,method+marker),encoding="utf8")
    path = Path("src/dynamics/time_stepping/SoftStep.cpp")
    src = (args.baseline/path).read_text(encoding="utf8")
    marker = "constexpr int kRechecks"
    states = "StepArray<SolverBody> states = solverBodiesOf(bodies);"
    assert src.count(marker) == 1 and src.count(states) == 1
    algorithm = args.algorithm.read_text(encoding="utf8")
    src = src.replace(marker, algorithm+"\n"+marker)
    if "bool tryMomentumStep(" in algorithm:
        src = src.replace(states,"if (tryMomentumStep(*this, bodies, pairs, joints, gravity, h, contactMemory, jointMemory)) return;\n    "+states)
    else:
        src = src.replace(states, states+"\n    if (tryBalancedStep(*this, bodies, pairs, joints, gravity, h, contactMemory, jointMemory, states)) return;")
    (args.candidate/path).write_text(src,encoding="utf8")


if __name__ == "__main__":
    main()
