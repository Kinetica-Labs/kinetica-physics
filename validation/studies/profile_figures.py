#!/usr/bin/env python3
"""Рисунок отчёта docs/reports/profile-2026-11-21.md: куда уходит время кадра по профилю Intel VTune.

Сырьё — docs/reports/data/vtune-pyramid-functions.csv: отчёт `vtune -report hotspots -group-by function`
по прогону compare-ours realtime 8 (BENCH=14, один поток) на свободной машине. Функции сводятся в группы
правилами groupOf; запуск переписывает docs/reports/figures/profile-groups.svg, --png кладёт рядом png.
"""
import csv
import re
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from status_figures import BETTER, GRID, INK, MUTED, SURFACE, WORSE, frame, plt, ru, save

DATA = Path(__file__).resolve().parents[2] / "docs/reports/data/vtune-pyramid-functions.csv"
CONTACT = ("SoftContact", "CentreFriction", "RollingFriction", "SolverBody")
ARRAYS = re.compile(r"vector<.*>::(resize|assign|push_back|operator=|reserve|__append|clear|insert)|memcpy|memset")


def groupOf(name, module, source):
    if "prefetch" in name: return "ожидание памяти: подкачка контактов"
    if "solveDenseBlock" in name or "solveComplementarity" in name: return "задача площадки: разложение и перебор"
    if ARRAYS.search(name) or "ucrtbase" in module: return "служебные массивы: resize, assign, копии"
    if any(word in name for word in CONTACT) or source in ("SoftContact.cpp", "CentreFriction.cpp", "RollingFriction.cpp", "SolverBody.h"): return "код контакта: нормали, трение, пересчёт точек"
    if source in ("Vector3.h", "Quaternion.h", "Matrix3x3.h", "Matrix2x2.h", "math_functions.h") or name.startswith("kp::operator"): return "векторная арифметика внутри него"
    return "поиск пар, сдвиг тел и прочее"


def groups():
    spent = Counter()
    with DATA.open(encoding="utf-8", errors="replace") as file:
        for row in csv.reader(file, delimiter=";"):
            if len(row) > 7 and row[0] != "Function": spent[groupOf(row[0], row[5], row[7])] += float(row[1])
    return spent


def draw():
    spent = groups()
    total = sum(spent.values())
    rows = sorted(spent.items(), key=lambda item: item[1])
    fig, ax = plt.subplots(figsize=(8.4, 3.0))
    for y, (name, seconds) in enumerate(rows):
        idle = name.startswith(("ожидание", "служебные"))
        ax.barh(y, 100 * seconds / total, height=0.62, color=WORSE if idle else BETTER, edgecolor=SURFACE, linewidth=2)
        ax.annotate(f"{ru(100 * seconds / total)} %", (100 * seconds / total, y), xytext=(6, 0), textcoords="offset points", va="center", color=MUTED, fontsize=8)
    ax.set_yticks(range(len(rows))), ax.set_yticklabels([name for name, _ in rows])
    ax.set_xlim(0, 34), ax.set_xlabel("доля времени счёта, %")
    ax.plot([], [], "s", color=BETTER, label="счёт физики"), ax.plot([], [], "s", color=WORSE, label="не счёт: ожидание и служебная работа")
    ax.legend(frameon=False, loc="lower right", fontsize=8)
    ax.set_title("Пирамида из 1 015 кубов, один поток: куда уходит время (профиль VTune)")
    frame(ax), save(fig, "profile-groups")


if __name__ == "__main__":
    draw()
