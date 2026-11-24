"""Extract static instructions of sampled native functions from the matching Windows PE."""
import argparse
from collections import Counter
import csv
import hashlib
from pathlib import Path
import re
import subprocess


def output(tool, *args):
    return subprocess.check_output([tool, *args], encoding="utf-8", errors="replace")


def extract(exe, target):
    if hashlib.sha256(exe.read_bytes()).hexdigest() != (target / "hash.txt").read_text().strip().lower():
        raise ValueError(f"{exe}: profile and executable hashes differ")
    with (target / "resolved.csv").open(encoding="utf-8-sig", newline="") as f:
        hot = list(csv.DictReader(f))
    symbols = output("llvm-nm", "-n", "--defined-only", exe)
    starts = sorted({int(a, 16) for a in re.findall(r"^([0-9a-fA-F]+) [TtWw] ", symbols, re.M)})
    with (target / "instructions.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(("symbol", "cpu_s", "address", "end", "instructions", "xmm", "ymm", "zmm", "packed_arithmetic", "examples"))
        for row in hot:
            start = int(row["addr"], 16)
            if start not in starts:
                continue
            end = next((addr for addr in starts if addr > start), None)
            if not end:
                continue
            listing = output("llvm-objdump", "-d", "--no-show-raw-insn", f"--start-address={start}", f"--stop-address={end}", exe)
            ins = re.findall(r"^[ \t]*[0-9a-fA-F]+:[ \t]+(\w+)[ \t]*([^\r\n]*)\r?$", listing, re.M)
            lanes = [sum(bool(re.search(rf"\b{reg}\d+", ops)) for _, ops in ins) for reg in ("xmm", "ymm", "zmm")]
            packed = Counter(op for op, _ in ins if re.fullmatch(r"v?(add|sub|mul|div|min|max|sqrt|rsqrt|fmadd\d*|fmsub\d*)p[sd]", op))
            examples = " | ".join(op + " " + ops.strip() for op, ops in ins if op in packed and not ops.lstrip().startswith("#"))
            w.writerow((row["symbol"], row["cpu_s"], hex(start), hex(end), len(ins), *lanes, sum(packed.values()), examples[:220]))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("exe", type=Path)
    p.add_argument("profile_dir", type=Path)
    args = p.parse_args()
    extract(args.exe, args.profile_dir)
