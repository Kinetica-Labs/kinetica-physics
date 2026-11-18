#!/usr/bin/env python3
"""Рисунки отчёта о стандартном наборе сцен (docs/reports/standard-scenes.md).

    build/validation/studies/study-standard-scenes build/scenes.jsonl 100
    python3 validation/studies/standard_scenes_figures.py build/scenes.jsonl

Рисунки строятся только из строк прибора: поправить число на рисунке руками нельзя.
"""
import json
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parents[2] / "docs" / "reports" / "figures"
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "text.color": INK, "axes.labelcolor": MUTED, "axes.edgecolor": GRID,
                     "xtick.color": MUTED, "ytick.color": MUTED, "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlelocation": "left"})
REAL_TIME = 1000 / 60  # мс на кадр: медленнее — сцена считается дольше, чем идёт


def save(fig, name):
    fig.savefig(OUT / f"{name}.svg", bbox_inches="tight")
    if os.environ.get("CASE_TABLE_PNG"):
        fig.savefig(Path(os.environ["CASE_TABLE_PNG"]) / f"{name}.png", bbox_inches="tight", dpi=110)
    plt.close(fig)


def bars(ax, rows, key, title, unit, log=True, mark=None):
    """Горизонтальные столбики по сценам; цвет — итог сцены, число подписано у каждого столбика."""
    names = [r["title"] for r in rows]
    values = [max(r[key], 1e-3) for r in rows]
    ax.barh(range(len(rows)), values, color=[BLUE if r["ok"] else ORANGE for r in rows], height=0.62)
    if log:
        ax.set_xscale("log")
    ax.set_yticks(range(len(rows)), names)
    for i, (r, v) in enumerate(zip(rows, values)):
        shown = f"{r[key] / 1000:.3g} с" if unit == "мс" and r[key] >= 1000 else f"{r[key]:.3g} {unit}"
        ax.text(v * (1.12 if log else 1) + (0 if log else 0.02 * max(values)), i, shown.replace(".", ","), va="center", fontsize=8, color=INK)
    if mark:
        ax.axvline(mark[0], color=MUTED, linewidth=1, linestyle=(0, (4, 3)))
        ax.text(mark[0], -0.75, mark[1], ha="center", va="bottom", fontsize=8, color=MUTED)
    ax.set_title(title)
    ax.grid(axis="x", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.margins(x=0.25 if log else 0.18)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.tick_params(axis="y", length=0)


# С какой стороны показывать сцену: стена и куча понятнее с торца (вдоль оси x).
VIEW = {"wall": "end"}


def gallery(rows, shapes):
    """Очертания тел каждой сцены: слева — начало опыта, справа — момент проверки."""
    drawn = {}
    for line in shapes:
        drawn[(line["scene"], line["when"], line["view"])] = line["parts"]
    columns = 4
    lines = -(-len(rows) // 2)
    fig, axes = plt.subplots(lines, columns, figsize=(11, 2.3 * lines))
    for ax in axes.flat:
        ax.set_axis_off()
    for index, row in enumerate(rows):
        view = VIEW.get(row["scene"], "side")
        # Обе половины в одном масштабе — по движущимся телам начала и момента проверки.
        both = drawn.get((row["scene"], "start", view), []) + drawn.get((row["scene"], "check", view), [])
        moving = [p for part in both if not part["fixed"] for p in part["outline"]]
        for half, when in enumerate(("start", "check")):
            ax = axes[index // 2][2 * (index % 2) + half]
            parts = drawn.get((row["scene"], when, view))
            if parts is None:
                ax.text(0.5, 0.5, "не досчитана", ha="center", va="center", fontsize=8, color=MUTED, transform=ax.transAxes)
            for part in parts or []:
                xs, ys = zip(*part["outline"])
                ax.fill(xs, ys, facecolor="#d9d8d3" if part["fixed"] else (BLUE if row["ok"] else ORANGE), edgecolor=SURFACE, linewidth=0.4, alpha=0.9)
            if moving:
                xs, ys = zip(*moving)
                span = max(max(xs) - min(xs), max(ys) - min(ys), 0.3) * 0.62
                cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
                ax.set_xlim(cx - span, cx + span)
                ax.set_ylim(cy - span, cy + span)
            ax.set_aspect("equal")
            if half == 0:
                ax.set_title(row["title"], fontsize=8.5)
            ax.text(0.02, 0.02, "начало" if half == 0 else "в момент проверки", transform=ax.transAxes, fontsize=7, color=MUTED)
    fig.tight_layout()
    save(fig, "standard-scenes-gallery")


def main():
    rows = [json.loads(line) for line in open(sys.argv[1], encoding="utf-8")]
    OUT.mkdir(parents=True, exist_ok=True)
    shapes_path = Path(sys.argv[1] + ".shapes.jsonl")
    if shapes_path.exists():
        gallery(rows, [json.loads(line) for line in open(shapes_path, encoding="utf-8")])
    fig, axes = plt.subplots(1, 2, figsize=(11, 0.36 * len(rows) + 1.4), sharey=True)
    bars(axes[0], rows, "ms_per_frame", "Время счёта кадра 1/60 с", "мс", mark=(REAL_TIME, "в темпе сцены"))
    bars(axes[1], rows, "segments_per_frame", "На сколько отрезков делится шаг", "", log=True)
    axes[1].tick_params(axis="y", labelleft=False)
    axes[0].invert_yaxis()  # ось общая: первая сцена — сверху
    handles = [plt.Rectangle((0, 0), 1, 1, color=BLUE), plt.Rectangle((0, 0), 1, 1, color=ORANGE)]
    fig.legend(handles, ["сцена кончилась как должно", "не досчитана или кончилась не так"], loc="lower center", ncol=2, frameon=False, bbox_to_anchor=(0.5, -0.04))
    fig.tight_layout()
    save(fig, "standard-scenes-cost")
    done = sum(r["ok"] for r in rows)
    print(f"сцен {len(rows)}, как должно {done}")


if __name__ == "__main__":
    main()
