#!/usr/bin/env python3
"""Рисунки отчёта о полной таблице случаев (docs/reports/case-table.md).

    build/validation/studies/study-case-table 4 build/case-table.jsonl
    python3 validation/studies/case_table_figures.py build/case-table.jsonl ["что изменилось"]

Рисунки строятся только из строк прибора: поправить число на рисунке руками нельзя. Со вторым
аргументом итог прогона дописывается строкой в docs/reports/case-table-history.jsonl — из этого
журнала строится рисунок «было — стало»; строки журнала не правятся и не удаляются.
"""
import json
import os
import sys
from collections import Counter
from datetime import date
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

OUT = Path(__file__).resolve().parents[2] / "docs" / "reports" / "figures"
HISTORY = OUT.parent / "case-table-history.jsonl"
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
RAMP = LinearSegmentedColormap.from_list("blue", ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "text.color": INK, "axes.labelcolor": MUTED, "axes.edgecolor": GRID,
                     "xtick.color": MUTED, "ytick.color": MUTED, "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlelocation": "left"})

SHAPES = ["шар", "капсула", "цилиндр", "скругл. цилиндр", "брусок", "оболочка"]
ELEMENTS = ["грань", "ребро", "вершина", "бок", "выпуклость"]
SIZES = ["A впятеро меньше", "размеры равны", "A впятеро больше"]
# Пороги — те же, что в приборе (Result в CaseTable.cpp).
DEFECTS = [("пролёт", lambda r: r["passed"] > 0), ("перекрытие\n> 0,02 мм", lambda r: r["overlap_m"] > 2e-5), ("энергия\n> 10⁻⁴", lambda r: r["energy_gain"] > 1e-4),
           ("импульс\n> 10⁻⁹", lambda r: r["momentum_loss"] > 1e-9), ("момент\nимпульса\n> 10⁻⁹", lambda r: r["angular_loss"] > 1e-9),
           ("покой\n> 1 мм/с", lambda r: r["rest_judged"] and r["rest_speed"] > 1e-3),
           ("шаг\nбез конца", lambda r: r["endless"]), ("не число", lambda r: not r["finite"])]


def save(fig, name):
    """SVG в отчёт; с переменной CASE_TABLE_PNG — ещё и PNG в указанный каталог, для просмотра."""
    fig.savefig(OUT / f"{name}.svg", bbox_inches="tight")
    if os.environ.get("CASE_TABLE_PNG"):
        fig.savefig(Path(os.environ["CASE_TABLE_PNG"]) / f"{name}.png", bbox_inches="tight", dpi=110)
    plt.close(fig)


def heatmap(ax, values, rows, columns, title, total=None):
    """Тепловая карта счётчиков: ноль — пустая клетка, число подписано в каждой непустой."""
    values = np.asarray(values, dtype=float)
    top = max(1.0, values.max())
    ax.imshow(np.ma.masked_equal(values, 0), cmap=RAMP, vmin=0, vmax=top, aspect="auto")
    for (i, j), v in np.ndenumerate(values):
        if v:
            ax.text(j, i, f"{v:.0f}", ha="center", va="center", fontsize=8, color="white" if v > 0.45 * top else INK)
    ax.set_xticks(range(len(columns)), columns)
    ax.set_yticks(range(len(rows)), rows)
    ax.set_xticks(np.arange(-0.5, len(columns)), minor=True)
    ax.set_yticks(np.arange(-0.5, len(rows)), minor=True)
    ax.grid(which="minor", color=GRID, linewidth=0.8)
    ax.tick_params(which="both", length=0)
    ax.xaxis.tick_top()
    ax.set_title(title + (f"\nв каждой клетке рисунка — из {total}" if total else ""), pad=10)
    for spine in ax.spines.values():
        spine.set_visible(False)


def variant(shape, element):
    return f"{shape}: {element}"


def figure_variants(rows):
    names = [variant(s, e) for s in SHAPES for e in ELEMENTS]
    present = [n for n in names if any(variant(r["a_shape"], r["a_element"]) == n for r in rows)]
    index = {n: k for k, n in enumerate(present)}
    bad = np.zeros((len(present), len(present)))
    for r in rows:
        bad[index[variant(r["a_shape"], r["a_element"])], index[variant(r["b_shape"], r["b_element"])]] += r["bad"]
    per = len(rows) // len(present) ** 2
    fig, ax = plt.subplots(figsize=(9.2, 6.6))
    heatmap(ax, bad, present, present, "Плохие клетки: чем касается A (строки) × чего касается у B (столбцы)", per)
    plt.setp(ax.get_xticklabels(), rotation=55, ha="left", rotation_mode="anchor")
    # Границы между фигурами.
    edge = 0
    for shape in SHAPES[:-1]:
        edge += sum(n.startswith(shape + ":") for n in present)
        ax.axhline(edge - 0.5, color=MUTED, linewidth=1.2)
        ax.axvline(edge - 0.5, color=MUTED, linewidth=1.2)
    save(fig, "case-table-variants")


def figure_shapes(rows):
    share = np.zeros((6, 6))
    for i, a in enumerate(SHAPES):
        for j, b in enumerate(SHAPES):
            cells = [r["bad"] for r in rows if r["a_shape"] == a and r["b_shape"] == b]
            share[i, j] = round(100 * sum(cells) / len(cells))
    fig, ax = plt.subplots(figsize=(6.2, 3.6))
    heatmap(ax, share, SHAPES, SHAPES, "Плохие клетки, %: фигура A (строки) на фигуре B (столбцы)")
    plt.setp(ax.get_xticklabels(), rotation=30, ha="left", rotation_mode="anchor")
    save(fig, "case-table-shapes")


def figure_history():
    """Сколько клеток с каждым нарушением было после каждого прогона, записанного в журнал."""
    runs = [json.loads(line) for line in open(HISTORY, encoding="utf-8")]
    # Пороги нарушений со временем менялись, и вместе с ними — подписи в журнале; вид нарушения
    # узнаётся по началу подписи.
    kinds = ["плохих всего", "пролёт", "перекрытие", "энергия", "импульс", "момент импульса", "покой", "шаг без конца"]
    fig, axes = plt.subplots(2, 4, figsize=(11, 4.6), gridspec_kw={"wspace": 0.35, "hspace": 0.75})
    for ax, kind in zip(axes.flat, kinds):
        values = [next(v for k, v in run["counts"].items() if k.startswith(kind)) for run in runs]
        ax.bar(range(len(runs)), values, width=0.6, color=BLUE)
        for x, v in enumerate(values):
            ax.text(x, v, f"{v}", ha="center", va="bottom", fontsize=8)
        ax.set_xticks(range(len(runs)), [str(k) for k in range(len(runs))])
        ax.set_ylim(0, max(1, max(values)) * 1.2)
        ax.set_title(kind, fontsize=9)
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(axis="y", labelsize=7)
    for k, run in enumerate(runs):
        fig.text(0.125, -0.02 - 0.045 * k, f"{k} — {run['label']}", fontsize=8, color=MUTED)
    save(fig, "case-table-history")


def figure_regimes(rows):
    regimes = list(dict.fromkeys(r["regime"] for r in rows))
    by_defect = [[sum(test(r) for r in rows if r["regime"] == g) for _, test in DEFECTS] for g in regimes]
    by_size = [[sum(r["bad"] for r in rows if r["regime"] == g and r["size"] == s) for s in SIZES] for g in regimes]
    per = len(rows) // len(regimes)
    fig, (left, right) = plt.subplots(1, 2, figsize=(11.5, 4.8), gridspec_kw={"width_ratios": [8, 3], "wspace": 0.05})
    heatmap(left, by_defect, regimes, [name for name, _ in DEFECTS], "Режим × вид нарушения: число клеток", per)
    left.tick_params(axis="x", labelsize=7.5)
    heatmap(right, by_size, [""] * len(regimes), ["A впятеро\nменьше", "размеры\nравны", "A впятеро\nбольше"], "Режим × размер: плохих клеток", per // 3)
    save(fig, "case-table-regimes")


def figure_distributions(rows):
    """Доля клеток, у которых мерка превышает x: видно, сколько клеток отсекает любой порог."""
    panels = [("Перекрытие, мм", [1e3 * r["overlap_m"] for r in rows], 2e-2, "порог 0,02 мм", 1e-2, "допуск движка\n0,01 мм"),
              ("Прирост энергии, доля", [r["energy_gain"] for r in rows], 1e-4, "порог 10⁻⁴", None, None),
              ("Уход момента импульса, доля", [r["angular_loss"] for r in rows if "свободно" in r["regime"]], 1e-9, "порог 10⁻⁹", None, None)]
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.3), gridspec_kw={"wspace": 0.28})
    for ax, (label, values, threshold, note, second, second_note) in zip(axes, panels):
        values = np.sort(np.asarray(values))
        positive = values[values > 0]
        share = 1 - np.searchsorted(values, positive, side="left") / len(values)
        ax.step(positive, share, where="post", color=BLUE, linewidth=2)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_ylim(0.5 / len(values), 1.5)
        ax.set_xlim(left=max(positive.min(), threshold * 1e-8) / 3)  # левее — только округление
        for x, text, height in ((threshold, note, 1.25), (second, second_note, 0.02)):
            if x:
                ax.axvline(x, color=MUTED, linewidth=1, linestyle=(0, (4, 3)))
                ax.text(x, height, " " + text, fontsize=8, color=MUTED, va="top")
        above = (values > threshold).sum()
        ax.set_title(f"{label}\nвыше порога {above} из {len(values)} клеток")
        ax.set_xlabel(label)
        ax.grid(color=GRID, linewidth=0.6)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("доля клеток с мерой больше x")
    save(fig, "case-table-distributions")


def figure_cost(rows):
    regimes = list(dict.fromkeys(r["regime"] for r in rows))
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.8), sharey=True, gridspec_kw={"wspace": 0.06})
    for ax, key, factor, label in ((axes[0], "segments", 1, "отрезков в одном шаге"), (axes[1], "slowest_step_s", 1e3, "самый долгий шаг, мс")):
        for k, g in enumerate(regimes):
            values = np.asarray([factor * r[key] for r in rows if r["regime"] == g])
            marks = np.quantile(values, 0.5), np.quantile(values, 0.99), values.max()
            ax.plot([marks[0], marks[2]], [k, k], color=GRID, linewidth=2, zorder=1)
            for x, color, name in zip(marks, (BLUE, ORANGE, AQUA), ("половина клеток — не больше", "99 % клеток — не больше", "наибольшее")):
                ax.scatter(x, k, s=44, color=color, edgecolor=SURFACE, linewidth=1.5, zorder=2, label=name if k == 0 else None)
            cut = key == "segments" and marks[2] >= 10000
            text = "обрыв на 10 000" if cut else f"{marks[2]:,.0f}".replace(",", " ") if marks[2] >= 100 else f"{marks[2]:.3g}".replace(".", ",")
            ax.text(marks[2] * 1.25, k, text, va="center", fontsize=8, color=MUTED)
        ax.set_xscale("log")
        ax.set_xlim(right=ax.get_xlim()[1] * 12)
        ax.set_title(label.capitalize())
        ax.grid(axis="x", color=GRID, linewidth=0.6)
        ax.spines[["top", "right", "left"]].set_visible(False)
        ax.tick_params(axis="y", length=0)
    axes[0].set_yticks(range(len(regimes)), regimes)
    axes[0].invert_yaxis()
    axes[0].legend(loc="lower center", bbox_to_anchor=(1.0, -0.28), ncol=3, frameon=False, fontsize=8)
    save(fig, "case-table-cost")


def figure_scene():
    """Как поставлена одна проба: элемент A над элементом B."""
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.add_patch(plt.Polygon([(-2.2, -1.5), (2.2, -1.5), (2.2, 0), (-2.2, 0)], facecolor="#cde2fb", edgecolor=BLUE, linewidth=1.5))
    body = np.array([(-0.9, 0), (0.9, 0), (0.9, 1.1), (-0.9, 1.1)])
    turn = np.array([[np.cos(0.22), -np.sin(0.22)], [np.sin(0.22), np.cos(0.22)]])
    ax.add_patch(plt.Polygon(body @ turn.T + (0.45, 0.95), facecolor="#fbe0d5", edgecolor=ORANGE, linewidth=1.5))
    ax.text(-2.05, -1.3, "B: неподвижно или свободно", color=INK, fontsize=9)
    ax.text(0.95, 1.5, "A", color=INK, fontsize=11, weight="bold")
    ax.scatter([0, 0.45], [0, 0.95], s=30, color=INK, zorder=3)
    ax.annotate("точка элемента B", (0, 0), (-2.1, 0.45), fontsize=8, color=MUTED, arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.8})
    ax.annotate("точка элемента A", (0.45, 0.95), (-2.1, 1.25), fontsize=8, color=MUTED, arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.8})
    ax.annotate("", (0.45, 0.02), (0.45, 0.93), arrowprops={"arrowstyle": "<->", "color": MUTED, "lw": 1})
    ax.text(0.52, 0.42, "зазор c", fontsize=8, color=MUTED)
    ax.annotate("", (0.45, -0.25), (0, -0.25), arrowprops={"arrowstyle": "<->", "color": MUTED, "lw": 1})
    ax.text(-0.05, -0.55, "смещение δ", fontsize=8, color=MUTED)
    ax.annotate("", (1.9, 0.75), (1.9, 1.75), arrowprops={"arrowstyle": "-|>", "color": INK, "lw": 1.5})
    ax.text(2.0, 1.2, "скорость v\nили тяжесть g", fontsize=8, color=INK)
    ax.plot([0.45, 0.45], [0.95, 2.5], color=MUTED, linewidth=0.8, linestyle=(0, (4, 3)))
    ax.plot([0.45, 0.45 - 1.5 * np.sin(0.22)], [0.95, 0.95 + 1.5 * np.cos(0.22)], color=MUTED, linewidth=0.8)
    ax.text(0.05, 2.55, "наклон θ", fontsize=8, color=MUTED)
    ax.annotate("", (0.75, 2.75), (1.15, 2.6), arrowprops={"arrowstyle": "-|>", "color": MUTED, "lw": 1, "connectionstyle": "arc3,rad=0.6"})
    ax.text(1.25, 2.6, "поворот φ вокруг нормали", fontsize=8, color=MUTED)
    ax.set_xlim(-2.4, 3.6)
    ax.set_ylim(-1.6, 3.0)
    ax.set_aspect("equal")
    ax.axis("off")
    save(fig, "case-table-scene")


def summary(rows):
    """Числа для текста отчёта."""
    print(f"клеток {len(rows)}, плохих {sum(r['bad'] for r in rows)}")
    for name, test in DEFECTS:
        print(f"  {name.splitlines()[0]}: {sum(bool(test(r)) for r in rows)}")
    for title, key in (("по режиму", lambda r: r["regime"]), ("по размеру", lambda r: r["size"]), ("фигура A", lambda r: r["a_shape"]), ("фигура B", lambda r: r["b_shape"]),
                       ("элемент A", lambda r: r["a_element"]), ("элемент B", lambda r: r["b_element"])):
        total, bad = Counter(), Counter()
        for r in rows:
            total[key(r)] += 1
            bad[key(r)] += r["bad"]
        print(title + ": " + "; ".join(f"{k} {bad[k]}/{total[k]}" for k in total))


def main():
    rows = [json.loads(line) for line in open(sys.argv[1], encoding="utf-8")]
    OUT.mkdir(parents=True, exist_ok=True)
    if len(sys.argv) > 2:
        counts = {"плохих всего": sum(r["bad"] for r in rows)} | {name.replace("\n", " "): sum(bool(test(r)) for r in rows) for name, test in DEFECTS}
        with open(HISTORY, "a", encoding="utf-8") as history:
            history.write(json.dumps({"date": date.today().isoformat(), "label": sys.argv[2], "cells": len(rows), "counts": counts}, ensure_ascii=False) + "\n")
    figure_history()
    figure_shapes(rows)
    figure_scene()
    figure_variants(rows)
    figure_regimes(rows)
    figure_distributions(rows)
    figure_cost(rows)
    summary(rows)


if __name__ == "__main__":
    main()
