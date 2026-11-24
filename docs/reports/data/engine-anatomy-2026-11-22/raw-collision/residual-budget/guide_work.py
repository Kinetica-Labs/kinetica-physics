"""Prepare one reversible change to the projected solver in the Windows experiment tree."""
import argparse
from pathlib import Path


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError(f"Unexpected source: {before}")
    return text.replace(before, after)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("source", type=Path)
    p.add_argument("destination", type=Path)
    p.add_argument("variant", choices=("reserve", "zero", "combined"))
    args = p.parse_args()
    code = args.source.read_text(encoding="utf8")
    if args.variant in ("reserve", "combined"):
        line = "        joints[j].writeBonds(bodies, bodies, rows);"
        code = replace_once(code, line, "        rows.reserve(memory[j].size());\n" + line)
    if args.variant in ("zero", "combined"):
        line = "            const Real impulse = memory[j][k];"
        code = replace_once(code, line, line + "\n            if (impulse == 0) continue;")
        line = "        SolverBody::push(sa, sb, at[size_t(k)] - a.position, at[size_t(k)] - b.position, (h * f.normal) * m.normal);"
        code = replace_once(code, line, line.replace("SolverBody::push", "if (f.normal != 0) SolverBody::push"))
        line = "    SolverBody::twist(sa, sb, (h * moment) * m.normal);"
        code = replace_once(code, line, line.replace("SolverBody::twist", "if (moment != 0) SolverBody::twist"))
    args.destination.write_text(code, encoding="utf8")


if __name__ == "__main__":
    main()
