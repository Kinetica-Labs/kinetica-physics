"""Сводка опытов с поиском пар и порогом перепроверки по сохранённым кадрам Windows."""
import csv
import math
import statistics
import sys
from pathlib import Path

from summarize import DIRECTIONS, SCENES, frames, load


def depth_runs(folder):
    for label, mm in (("default", 1.0), ("500", 0.5), ("250", 0.25)):
        for scene in SCENES:
            stem = f"depth-{label}-{scene}"
            states = frames(folder / f"{stem}-verify.csv", 0)
            durations = frames(folder / f"{stem}-time.csv", 1)
            start, end = states[0], states[-1]
            if {r["final_hash"] for r in durations} != {end["state_hash"]}:
                raise ValueError(f"Different timed trajectory in {stem}")
            floor = max(states[1:], key=lambda r: float(r["floor_mm"]))
            pair = max(states[1:], key=lambda r: float(r["pair_mm"]))
            outside = [r["frame"] for r in states[1:] if float(r["floor_mm"]) > 2 or float(r["pair_mm"]) > 5]
            with (folder / f"{stem}-verify-violations.txt").open() as f:
                contacts = list(csv.reader(f))
            if [entry[0] for entry in contacts] != outside:
                raise ValueError(f"Contact identifiers do not cover all failing frames in {stem}")
            first = contacts[0] if contacts else [""] * 6
            yield dict(scene=scene, recheck_mm=mm, max_floor_mm=floor["floor_mm"], floor_frame=floor["frame"],
                       max_pair_mm=pair["pair_mm"], pair_frame=pair["frame"], failed_frames=len(outside),
                       first_frame=first[0], first_deepest_floor_body=first[1],
                       first_deepest_pair_a=first[3], first_deepest_pair_b=first[4],
                       apex_drop_m=end["apex_drop_m"], min_spread_m=min(float(end[k])-float(start[k]) for k in DIRECTIONS),
                       final_energy_J=end["energy_J"], single_run_mean_ms=statistics.mean(float(r["step_us"]) for r in durations)/1000,
                       final_hash=end["state_hash"])


def support_runs(folder):
    for variant in (0, 1, 2):
        for scene in SCENES:
            stem = f"support-{variant}-{scene}"
            positions = frames(folder / f"{stem}-verify.csv", 0)
            timings = frames(folder / f"{stem}-time.csv", 1)
            initial, final = positions[0], positions[-1]
            if {r["final_hash"] for r in timings} != {final["state_hash"]}:
                raise ValueError(f"Different geometry and timing trajectories in {stem}")
            bad = [r["frame"] for r in positions[1:] if float(r["floor_mm"]) > 2 or float(r["pair_mm"]) > 5]
            log = (folder / f"{stem}-verify.log").read_text().splitlines()
            if [line.split(",")[0] for line in (log[:-2] if variant else log)] != bad:
                raise ValueError(f"Wrong set of failing frames for {stem}")
            audit = next(csv.DictReader(log[-2:])) if variant else {}
            energy = [float(r["energy_J"]) for r in positions]
            yield dict(scene=scene, passes=variant, failed_frames=len(bad), first_bad=bad[0] if bad else "",
                       max_floor_mm=max(float(r["floor_mm"]) for r in positions),
                       max_pair_mm=max(float(r["pair_mm"]) for r in positions),
                       single_run_mean_ms=statistics.mean(float(r["step_us"]) for r in timings)/1000,
                       apex_drop_m=final["apex_drop_m"], min_spread_m=min(float(final[k])-float(initial[k]) for k in DIRECTIONS),
                       max_sleeping=max(int(r["asleep"]) for r in positions), max_centers_below=max(int(r["below"]) for r in positions),
                       max_ballistic_mm=max(float(r["ballistic_mm"]) for r in positions),
                       max_frame_energy_increase_J=max(0,max(b-a for a,b in zip(energy,energy[1:]))),
                       fixed_impulses=audit.get("external",""), internal_impulses=audit.get("internal",""),
                       max_deltaP_Ns=audit.get("max_deltaP_Ns",""), max_deltaL_Nms=audit.get("max_deltaL_Nms",""),
                       max_positive_deltaT_J=audit.get("max_positive_deltaT_J",""), final_hash=final["state_hash"])


def comparisons(folder, filename, variants, blocks):
    times = load(folder / filename)
    for scene in SCENES:
        selected = [r for r in times if r["scene"] == scene]
        grouped = {(int(r["block"]), r["variant"]): r for r in selected}
        expected = {(block, name) for block in range(1, blocks + 1) for name in variants}
        if set(grouped) != expected or len(selected) != len(expected):
            raise ValueError(f"Incomplete pairs in {filename}, {scene}")
        control = (folder / f"{scene}-kinetica-verify-0.csv").read_bytes()
        final = frames(folder / f"{scene}-kinetica-verify-0.csv", 0)[-1]["state_hash"]
        means = {}
        for name in variants:
            if (folder / f"opt-{name}-{scene}-verify.csv").read_bytes() != control:
                raise ValueError(f"Different recorded trajectory for {scene}, {name}")
            runs = []
            for block in range(1, blocks + 1):
                samples = frames(folder / f"opt-{name}-{scene}-time-{block}.csv", 1)
                run = statistics.mean(float(r["step_us"]) for r in samples) / 1000
                entry = grouped[block, name]
                if entry["final_hash"] != final or {r["final_hash"] for r in samples} != {final}:
                    raise ValueError(f"Unexpected final state in {scene}, {name}, {block}")
                if not math.isclose(run, float(entry["mean_ms"]), rel_tol=0, abs_tol=5.1e-7):
                    raise ValueError(f"Different recorded time in {scene}, {name}, {block}")
                runs.append(run)
            means[name] = runs
        reference = means[variants[0]]
        for name in variants[1:]:
            changes = [100*(base-trial)/base for base, trial in zip(reference, means[name])]
            yield dict(scene=scene, control=variants[0], variant=name, pairs=blocks, wins=sum(d>0 for d in changes),
                       control_median_ms=statistics.median(reference), variant_median_ms=statistics.median(means[name]),
                       median_reduction_pct=statistics.median(changes), min_reduction_pct=min(changes),
                       max_reduction_pct=max(changes))


if __name__ == "__main__":
    folder, experiment = Path(sys.argv[1]), sys.argv[2]
    if experiment == "depth":
        rows = depth_runs(folder)
    elif experiment == "support":
        rows = support_runs(folder)
    elif experiment == "sat":
        rows = comparisons(folder, "opt-pairs.csv", ("base", "signs", "axes"), 6)
    elif experiment == "pairs":
        rows = comparisons(folder, "opt-chunks-pairs.csv", ("unbatched", "chunks", "leaves", "stream"), 4)
    else:
        raise SystemExit("experiment: depth | support | sat | pairs")
    first = next(rows)
    writer = csv.DictWriter(sys.stdout, fieldnames=first, lineterminator="\n")
    writer.writeheader()
    writer.writerow(first)
    writer.writerows(rows)
