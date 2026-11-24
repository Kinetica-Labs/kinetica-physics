"""Replace pointwise normal pushes by their total impulse and moment about the patch centre."""
import argparse
from pathlib import Path


def edit(source, before, after):
    if source.count(before) != 1:
        raise ValueError("Unexpected projected solver at: " + before)
    return source.replace(before, after)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("source", type=Path)
    p.add_argument("out", type=Path)
    args = p.parse_args()
    code = args.source.read_text(encoding="utf8")
    lines = code.splitlines(keepends=True)
    pushes = [s for s in lines if "SolverBody::push(sa, sb, at[size_t(k)]" in s]
    if len(pushes) != 1:
        raise ValueError("Unexpected normal reaction")
    code = edit(code, pushes[0], "")
    before = "    for (int k = 0; k < m.count; ++k) leverLoad += (at[size_t(k)] - centre).length() * normal[size_t(k)];"
    code = edit(code, before, """    Vector3 weightedArm;
    for (int k = 0; k < m.count; ++k) {
        const Vector3 arm = at[size_t(k)] - centre;
        leverLoad += arm.length() * normal[size_t(k)];
        weightedArm += normal[size_t(k)] * arm;
    }""")
    begin = code.index("    SolverBody::push(sa, sb, centre")
    end = code.index("    return true;", begin)
    code = code[:begin] + """    // Apply the total impulse (N s) and its moment about the patch centre (N m s).
    const Vector3 impulse = h * (load * m.normal + tangent);
    const Vector3 angular = h * (weightedArm.cross(m.normal) + moment * m.normal);
    SolverBody::push(sa, sb, centre - a.position, centre - b.position, impulse);
    SolverBody::twist(sa, sb, angular);
""" + code[end:]
    args.out.write_text(code, encoding="utf8")


if __name__ == "__main__":
    main()
