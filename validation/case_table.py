#!/usr/bin/env python3
"""Храповик полной таблицы случаев контакта: счёт клеток с нарушениями может только убывать.

  case_table.py            прогнать прибор study-case-table, напечатать счёт по меркам рядом с эталоном
  case_table.py --check    то же и ошибка, если хоть одна мерка стала хуже эталона
  case_table.py --update "что сделано"   записать нынешний счёт эталоном и строкой журнала; отказ, если
                           какая-то мерка хуже эталона (ухудшение записывается только с --worse)

Эталон — docs/reports/data/case-table-counts.json; журнал — docs/reports/case-table-history.jsonl
(только дописывается). Цель — нули во всех строках: 7 425 клеток — это все сочетания «чем касается A»,
«чего касается у B», отношения размеров и режима (validation/studies/CaseTable.cpp).
"""
import datetime
import itertools
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STUDY = ROOT / "build/validation/studies/study-case-table"
BASELINE = ROOT / "docs/reports/data/case-table-counts.json"
HISTORY = ROOT / "docs/reports/case-table-history.jsonl"
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
# Мерка и её нарушение в строке прибора; пороги — docs/reports/case-table.md, раздел 2.
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


def measure():
    """Счёт клеток с нарушением по каждой мерке на нынешней сборке."""
    with tempfile.TemporaryDirectory() as folder:
        rows = Path(folder) / "rows.jsonl"
        process = subprocess.run([str(STUDY), str(TRIALS), str(rows)], stdout=subprocess.DEVNULL)
        if process.returncode not in (0, 1):
            raise RuntimeError(f"study-case-table завершился с кодом {process.returncode}")
        cells = [json.loads(line) for line in rows.read_text(encoding="utf-8").splitlines()]
    check_cells(cells, process.returncode)
    counts = {name: sum(1 for cell in cells if broken(cell)) for name, broken in MEASURES}
    counts["плохих всего"] = sum(1 for cell in cells if any(broken(cell) for _, broken in MEASURES))
    return len(cells), counts


def main():
    cells, counts = measure()
    baseline = json.loads(BASELINE.read_text(encoding="utf-8")) if BASELINE.exists() else {}
    worse = [name for name, value in counts.items() if name != "плохих всего" and value > baseline.get(name, value)]
    print(f"клеток {cells}")
    for name, value in counts.items():
        was = baseline.get(name)
        mark = "" if was is None or value == was else "  ← хуже" if value > was else "  ← лучше"
        print(f"  {name:26s} {value:5d}   эталон {'—' if was is None else was}{mark}")
    if "--update" in sys.argv:
        label = sys.argv[sys.argv.index("--update") + 1]
        if worse and "--worse" not in sys.argv:
            sys.exit(f"не записано: хуже эталона — {', '.join(worse)}")
        BASELINE.write_text(json.dumps(counts, ensure_ascii=False) + "\n", encoding="utf-8")
        with HISTORY.open("a", encoding="utf-8") as history:
            history.write(json.dumps({"date": datetime.date.today().isoformat(), "label": label, "cells": cells, "counts": counts}, ensure_ascii=False) + "\n")
    if "--check" in sys.argv and worse:
        sys.exit(f"хуже эталона: {', '.join(worse)}")


if __name__ == "__main__":
    main()
