"""Compare the worst trajectory error with the best reference on each identical grip case."""
import argparse
import csv
import math
import sys


REFERENCES = ("box3d", "rp3d-f32", "rp3d-f64")
KINETICA = ("kinetica-exact", "kinetica-realtime", "kinetica-realtime32")
CASE = ("hz", "mu", "jaw_N", "load_N", "open")


def admissible(row):
    errors = [float(row[field]) for field in
              ("max_path_error_mm", "max_left_force_error_N", "max_right_force_error_N")]
    return (row["complete"] == "1" and all(math.isfinite(e) and e >= 0 for e in errors)
            and max(errors[1:]) <= 0.2)


def read_cases(filename):
    cases = {}
    with open(filename, newline="", encoding="utf-8-sig") as source:
        for row in csv.DictReader(source):
            key = tuple(row[field] for field in CASE)
            group = cases.setdefault(key, {})
            if row["engine"] in group:
                raise ValueError(f"Duplicate observation for {key}, {row['engine']}")
            group[row["engine"]] = row
    if len(cases) != 36:
        raise ValueError(f"Expected 36 distinct cases, found {len(cases)}")
    return cases


def compare(ours, references, delta):
    if not all(row and admissible(row) for row in [ours, *references]):
        return ("", "", "", "", "unknown")
    best = min(references, key=lambda row: float(row["max_path_error_mm"]))
    ours_mm = float(ours["max_path_error_mm"])
    best_mm = float(best["max_path_error_mm"])
    diff = ours_mm - best_mm
    outcome = "win" if diff < -delta else "loss" if diff > delta else "tie"
    return (best["engine"], ours_mm, best_mm, diff, outcome)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--delta-mm", type=float, default=0.001, help="tie band in millimetres")
    parser.add_argument("--shared-clock", action="store_true", help="use eight public updates per frame")
    args = parser.parse_args()
    if not math.isfinite(args.delta_mm) or args.delta_mm < 0:
        parser.error("--delta-mm must be finite and nonnegative")
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow([*CASE, "engine", "best_reference", "ours_mm", "reference_mm", "difference_mm", "outcome"])
    peers = tuple(engine + "-shared8" for engine in REFERENCES) if args.shared_clock else REFERENCES
    candidates = ("kinetica-shared8",) if args.shared_clock else KINETICA
    for key, group in read_cases(args.csv).items():
        references = [group.get(engine) for engine in peers]
        for engine in candidates:
            writer.writerow([*key, engine, *compare(group.get(engine), references, args.delta_mm)])


if __name__ == "__main__":
    main()
