#!/usr/bin/env python3
"""Рисунки отчёта docs/reports/status-2026-11-21.md.

Числа — снимок замеров этого дня; откуда каждое, сказано в отчёте рядом с рисунком. Запуск без ключей
переписывает docs/reports/figures/status-*.svg; --png кладёт рядом png для проверки глазами.
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIGURES = Path(__file__).resolve().parents[2] / "docs/reports/figures"
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BETTER, WORSE, WAS = "#2a78d6", "#eb6834", "#9a9995"  # пара проверена validate_palette.js на светлой подложке
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "text.color": INK, "axes.labelcolor": MUTED, "axes.edgecolor": GRID,
                     "xtick.color": MUTED, "ytick.color": INK, "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
                     "savefig.facecolor": SURFACE, "axes.titleweight": "bold", "axes.titlelocation": "left", "svg.fonttype": "none"})

# Время кадра, мс: (сцена, Box3D, мы). Свободная машина, один поток на закреплённом ядре, 2026-11-21.
SPEED = [("стопка из 10 кубов", 0.034, 0.279), ("пирамида из 55 кубов", 0.288, 1.545), ("осыпь из пирамиды 36", 0.188, 1.891),
         ("тело в теле", 0.018, 0.094), ("два куба с зазором 1 мм", 0.019, 0.057), ("куб 100:1 на лёгком кубе", 0.017, 0.060),
         ("столбик из 100 кубов", 0.345, 2.819), ("маятник 120°", 0.007, 0.064), ("балка из 6 брусков", 0.024, 0.208),
         ("цепь из 10 шарниров, груз 100:1", 0.045, 0.582), ("пирамида из 1 015 кубов", 6.97, 68.0)]
# Ошибка: (сцена и величина, у нас, у Box3D). Табло validation/run.py, 8 подшагов, медиана 16 вариантов, 2026-11-18…20.
ACCURACY = [("пирамида 55: проседание, мм", 5.31, 8.95), ("маятник: ошибка угла, рад", 0.000748, 0.00296),
            ("маятник: разрыв шарнира, мм", 0.0478, 0.434), ("стопка 10: проседание, мм", 2.39, 27.9),
            ("куб 100:1: проседание, мм", 0.613, 35.4), ("осыпь: дрожь, мм/с", 4.59e-5, 0.00797),
            ("цепь 100:1: разрыв шарнира, мм", 0.155, 43.5), ("пирамида 55: дрожь, мм/с", 1.62e-6, 0.00677)]
# Было → стало за день, мс: (что, было, стало).
BEFORE_AFTER = [("рывок куба в пирамиде из 55, кадр", 13.9, 5.9), ("сцена rail, точный шаг, кадр", 4.5, 1.8),
                ("сцена rail, быстрый шаг, кадр", 0.45, 0.13), ("коробка на спирали из 385 точек, кадр", 2.688, 0.069),
                ("сборка связей пути, 385 точек", 0.225, 0.00355), ("сборка связей пути, 25 точек", 0.0164, 0.00159)]
# Новые проверки: (что, измерено, допуск, подпись измеренного).
TOLERANCE = [("энергия шара на параболе, точный шаг", 7.9e-4, 2e-3, "7,9e-4 от m g h"), ("упор на конце кривой, быстрый шаг", 8.6e-6, 1e-4, "8,6e-6 м"),
             ("смыкание нормали замкнутой кривой", 7.8e-11, 1e-9, "7,8e-11"), ("энергия шара на параболе, быстрый шаг", 2.7e-3, 5e-2, "2,7e-3 от m g h"),
             ("шар от кривой, быстрый шаг", 2.1e-5, 1e-3, "2,1e-5 м"), ("путь вагонетки за 2 с", 9.3e-5, 5e-3, "9,3e-5 м"),
             ("длина дуги против сумм хорд", 8.1e-9, 1e-6, "8,1e-9 м"), ("шар от кривой, точный шаг", 8.2e-12, 1e-6, "8,2e-12 м"),
             ("перпендикуляр в ближайшей точке", 7.8e-16, 1e-9, "7,8e-16 м")]
# Очередь возможностей (docs/ROADMAP.md, раздел 7): номера сделанных.
QUEUE = [("свойства тела и мира", "О", 12, set()), ("сочленения, формы, системы как в Jolt", "К", 11, {1})]


def ru(value, digits=1):
    text = f"{value:,.{digits}f}".replace(",", " ").replace(".", ",")
    return text[:-digits - 1] if digits and text.endswith("," + "0" * digits) else text


def save(fig, name):
    fig.savefig(FIGURES / f"{name}.svg", bbox_inches="tight")
    if "--png" in sys.argv:
        fig.savefig(FIGURES / f"{name}.png", bbox_inches="tight", dpi=110)
    plt.close(fig)


def frame(ax):
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.tick_params(axis="y", length=0)
    ax.grid(axis="x", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def times(value):
    """«в 4 раза», «в 58 раз», «в 1,7 раза»: число с нужной формой слова."""
    text = ru(value, 1 if value < 20 else 0)
    whole = "," not in text and not (int(round(value)) % 10 in (2, 3, 4) and int(round(value)) % 100 not in (12, 13, 14))
    return f"в {text} {'раз' if whole else 'раза'}"


def lollipops(ax, rows, word, color):
    for y, (name, gain) in enumerate(rows):
        ax.plot([1, gain], [y, y], color=color, linewidth=2, solid_capstyle="round")
        ax.plot(gain, y, "o", color=color, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
        ax.annotate(f"{times(max(gain, 1 / gain))} {word}", (max(gain, 1), y), xytext=(8, 0), textcoords="offset points", va="center", color=MUTED, fontsize=8)
    ax.axvline(1, color=INK, linewidth=1)
    ax.set_xscale("log"), ax.set_xlim(0.05, 60000)
    ax.set_yticks(range(len(rows))), ax.set_yticklabels([name for name, _ in rows])
    ax.set_ylim(-0.7, len(rows) - 0.3)
    frame(ax)


def versus():
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(8.4, 6.6), sharex=True, gridspec_kw={"height_ratios": [len(ACCURACY), len(SPEED)], "hspace": 0.22})
    lollipops(top, [(name, theirs / ours) for name, ours, theirs in ACCURACY], "точнее", BETTER)
    lollipops(bottom, [(name, box / ours) for name, box, ours in sorted(SPEED, key=lambda r: r[1] / r[2])], "медленнее", WORSE)
    top.set_title("Точность: во сколько раз наша ошибка меньше, чем у Box3D")
    bottom.set_title("Скорость: во сколько раз наш кадр дольше, чем у Box3D")
    bottom.set_xticks([0.1, 1, 10, 100, 1000, 10000])
    bottom.set_xticklabels(["в 10 раз хуже", "наравне", "в 10 раз лучше", "в 100", "в 1 000", "в 10 000"])
    save(fig, "status-versus-box3d")


def beforeAfter():
    fig, ax = plt.subplots(figsize=(8.4, 3.2))
    for y, (name, was, now) in enumerate(BEFORE_AFTER):
        ax.plot([now, was], [y, y], color=GRID, linewidth=3, solid_capstyle="round", zorder=1)
        ax.plot(was, y, "o", color=WAS, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
        ax.plot(now, y, "o", color=BETTER, markersize=8, markeredgecolor=SURFACE, markeredgewidth=1.5)
        ax.annotate(f"{times(was / now)} быстрее", (now, y), xytext=(-9, 0), textcoords="offset points", va="center", ha="right", color=MUTED, fontsize=8)
    ax.set_xscale("log"), ax.set_xlim(1e-4, 40), ax.set_xlabel("время, мс (шкала логарифмическая)")
    ax.set_yticks(range(len(BEFORE_AFTER))), ax.set_yticklabels([name for name, _, _ in BEFORE_AFTER])
    ax.plot([], [], "o", color=WAS, label="было"), ax.plot([], [], "o", color=BETTER, label="стало")
    ax.legend(frameon=False, loc="lower left", ncol=2)
    ax.set_title("Что ускорено за день, свободная машина, один поток")
    frame(ax), save(fig, "status-before-after")


def tolerance():
    fig, ax = plt.subplots(figsize=(8.4, 3.9))
    for y, (name, measured, allowed, shown) in enumerate(TOLERANCE):
        ax.plot([measured / allowed, 1], [y, y], color=GRID, linewidth=2, zorder=1)
        ax.plot(measured / allowed, y, "o", color=BETTER, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
        ax.annotate(f"{shown} при допуске {allowed:.0e}".replace("e-0", "e-"), (measured / allowed, y), xytext=(-9, 0), textcoords="offset points", va="center", ha="right", color=MUTED, fontsize=8)
    ax.axvline(1, color=INK, linewidth=1)
    ax.text(1, len(TOLERANCE) - 0.35, "допуск", ha="center", va="bottom", color=INK, fontsize=8)
    ax.set_xscale("log"), ax.set_xlim(1e-12, 3), ax.set_xlabel("измерено / допуск (шкала логарифмическая)")
    ax.set_yticks(range(len(TOLERANCE))), ax.set_yticklabels([name for name, *_ in TOLERANCE])
    ax.set_title("Новые проверки пути: какую долю допуска занимает ошибка", pad=14)
    frame(ax), save(fig, "status-tolerance")


def queue():
    fig, ax = plt.subplots(figsize=(8.4, 1.9))
    for row, (title, letter, count, done) in enumerate(QUEUE):
        y = 1 - row
        for k in range(1, count + 1):
            ax.add_patch(plt.Rectangle((k - 0.45, y - 0.3), 0.9, 0.6, facecolor=BETTER if k in done else SURFACE, edgecolor=BETTER if k in done else WAS, linewidth=1))
            ax.text(k, y, f"{letter}{k}", ha="center", va="center", fontsize=8, color=SURFACE if k in done else MUTED)
        ax.text(0.4, y, f"{title}: {len(done)} из {count}", ha="right", va="center")
    ax.add_patch(plt.Rectangle((8.0, -0.95), 0.5, 0.3, facecolor=BETTER, edgecolor=BETTER)), ax.text(8.65, -0.8, "сделано", va="center", fontsize=8, color=MUTED)
    ax.add_patch(plt.Rectangle((10.6, -0.95), 0.5, 0.3, facecolor=SURFACE, edgecolor=WAS)), ax.text(11.25, -0.8, "в очереди", va="center", fontsize=8, color=MUTED)
    ax.set_xlim(-7, 13.6), ax.set_ylim(-1.1, 1.6), ax.axis("off")
    ax.set_title("Очередь возможностей: сделана одна из 23")
    save(fig, "status-features")


if __name__ == "__main__":
    versus(), beforeAfter(), tolerance(), queue()
