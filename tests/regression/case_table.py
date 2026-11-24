#!/usr/bin/env python3
"""Compare the complete contact grid with the release regression limits."""
import itertools
import json
import subprocess
import sys
import tempfile
from pathlib import Path

BASELINE = Path(__file__).resolve().parents[1] / 'data/case-table-counts.json'
TRIALS = 4
KEY = ("a_shape", "a_element", "b_shape", "b_element", "size", "regime")
FEATURES = {
    "шар": ("выпуклость",),
    "капсула": ("выпуклость", "бок"),
    "цилиндр": ("грань", "ребро", "бок"),
    "скругл. цилиндр": ("грань", "ребро", "бок"),
    "брусок": ("грань", "ребро", "вершина"),
    "оболочка": ("грань", "ребро", "вершина"),
}
SIZES = ("A впятеро меньше", "размеры равны", "A впятеро больше")
REGIMES = ("покой", "склон 35°", "волчок 20 рад/с", "удар 1 м/с", "удар 100 м/с",
           "удар 10 км/с", "удар 1 м/с с вращением", "удар 1 м/с, B свободно",
           "удар 100 м/с, B свободно", "удар 10 км/с, B свободно", "удар с вращением, B свободно")
# Мерка и её нарушение в строке прибора; пороги перечислены рядом с предикатами.
MEASURES = [
    # Если основной шаг не увидел удара, мелкий шаг различает пролёт (passed) и геометрический промах (missed).
    ("пролёт", lambda r: r["passed"] > 0),
    ("перекрытие > 0,02 мм", lambda r: r["overlap_m"] > 2e-5),
    ("энергия > 10⁻⁴", lambda r: r["energy_gain"] > 1e-4),
    ("импульс > 10⁻⁹", lambda r: r["momentum_loss"] > 1e-9),
    ("момент импульса > 10⁻⁹", lambda r: r["angular_loss"] > 1e-9),
    ("покой > 1 мм/с", lambda r: bool(r["rest_judged"]) and r["rest_speed"] > 1e-3),
    ("шаг без конца", lambda r: bool(r["endless"])),
    ("не число", lambda r: not r["finite"]),
]


def check_cells(cells, status):
    variants = [(shape, feature) for shape, features in FEATURES.items() for feature in features]
    expected = {(*a, *b, size, regime) for a, b, size, regime in itertools.product(variants, variants, SIZES, REGIMES)}
    seen = set()
    for cell in cells:
        key = tuple(cell[field] for field in KEY)
        if key not in expected:
            raise ValueError(f"непредусмотренная клетка: {key}")
        if key in seen:
            raise ValueError(f"повторная клетка: {key}")
        if cell["trials"] != TRIALS:
            raise ValueError(f"ожидалось {TRIALS} пробы, получено {cell['trials']}: {key}")
        seen.add(key)
    missing = expected - seen
    if missing:
        raise ValueError(f"пропущено клеток: {len(missing)}; первая: {min(missing)}")
    # bad считается в C++ по неокруглённым значениям, JSONL хранит округлённые.
    if status != int(any(cell["bad"] for cell in cells)):
        raise ValueError(f"код прибора {status} расходится с отметками bad")


def measure(study):
    """Счёт клеток с нарушением по каждой мерке на нынешней сборке."""
    with tempfile.TemporaryDirectory() as folder:
        rows = Path(folder) / "rows.jsonl"
        process = subprocess.run([str(study), str(TRIALS), str(rows)], stdout=subprocess.DEVNULL)
        if process.returncode not in (0, 1):
            raise RuntimeError(f"study-case-table завершился с кодом {process.returncode}")
        cells = [json.loads(line) for line in rows.read_text(encoding="utf-8").splitlines()]
    check_cells(cells, process.returncode)
    counts = {name: sum(1 for cell in cells if broken(cell)) for name, broken in MEASURES}
    counts["плохих всего"] = sum(1 for cell in cells if any(broken(cell) for _, broken in MEASURES))
    return len(cells), counts


def main():
    cells, counts = measure(sys.argv[1])
    baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
    print(f"клеток {cells}")
    worse = []
    for name, value in counts.items():
        limit = baseline[name]
        print(f"  {name:26s} {value:5d}   предел {limit}")
        if value > limit:
            worse.append(name)
    if worse:
        sys.exit(f"хуже эталона: {', '.join(worse)}")


if __name__ == "__main__":
    main()
