#!/usr/bin/env python3
"""Рисунки отчёта docs/reports/day-2026-11-21.md: что сделано за день после утреннего отчёта о состоянии.

Числа — снимок замеров этого дня; откуда каждое, сказано в отчёте рядом с рисунком. Запуск без ключей
переписывает docs/reports/figures/day-*.svg; --png кладёт рядом png для проверки глазами. Оформление — то
же, что у status_figures.py.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from status_figures import BETTER, GRID, INK, MUTED, SURFACE, WAS, WORSE, frame, plt, ru, save

# Уход ящика на поворотном столе за оборот, мм: (случай, было, стало). Точный шаг, прибор build/probe/driven.cpp.
TURNTABLE = [("точный шаг, 1/120 с", 26.8, 0.09), ("точный шаг, 1/60 с", 54.8, 0.40)]
FAST_TURNTABLE = 3.3  # быстрый шаг, подшаг 1/480 с: не менялся
# Отрезков на кадр точного шага: (сцена, было, стало). study-standard-scenes, бюджет 15 с на сцену, тот же сеанс.
SEGMENTS = [("платформы", 2.0, 1.7), ("раскачанная цепь", 10.8, 5.6), ("домино", 15.0, 13.8), ("чаша", 22.2, 21.7),
            ("куча в ящике", 23.2, 22.0), ("холмы", 27.4, 20.2), ("стена под ударом", 45.4, 46.1)]
# Стадии кадра пирамиды из 1 015 кубов, мс: (стадия, было, с контактами по уровням). Свободная машина, один поток.
STAGES = [("запись сил", 0.87, 1.12), ("постройка контактов", 6.40, 7.99), ("тёплый старт", 6.83, 7.13), ("трение", 7.06, 7.34),
          ("поиск пар", 7.93, 7.83), ("сдвиг и пересчёт точек", 13.44, 13.91), ("нормали", 24.17, 24.86)]
# Новые проверки: (что, измерено, допуск, записанный до кода). Больше единицы — допуск не выполнен.
CHECKS = [("сила: импульс тела, точный шаг", 2.4e-15, 1e-12), ("движимая плита: уход от пути, точный", 1.9e-15, 1e-12),
          ("сила: путь тела, точный шаг", 1.7e-15, 1e-9), ("подъёмник: зазор, точный шаг", 7e-14, 1e-6),
          ("толкатель: скорость ящика, быстрый", 2e-12, 1e-3), ("сила: ящик с трением, быстрый", 2.7e-3, 1e-2),
          ("поворотный стол, быстрый шаг", 3.3e-3, 1e-2), ("рывок плиты, быстрый шаг", 1.0e-2, 3e-2),
          ("поворотный стол, точный, после правки", 4.0e-4, 1e-3), ("сила: путь тела, быстрый шаг", 2.08e-3, 5e-3),
          ("поворотный стол, точный, до правки", 5.48e-2, 1e-3), ("рывок плиты, точный шаг", 1.2e-3, 1e-5)]


def dumbbell(ax, rows, unit, digits):
    """Строка — «было» серой точкой и «стало» цветной: синей, если меньше, оранжевой, если больше."""
    for y, (name, was, now) in enumerate(rows):
        color = BETTER if now < was else WORSE
        ax.plot([was, now], [y, y], color=GRID, linewidth=3, solid_capstyle="round", zorder=1)
        ax.plot(was, y, "o", color=WAS, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
        ax.plot(now, y, "o", color=color, markersize=8, markeredgecolor=SURFACE, markeredgewidth=1.5)
        shown = lambda value: ru(value, digits if value < 10 else min(digits, 1))
        ax.annotate(f"{shown(was)} → {shown(now)} {unit}", (max(was, now), y), xytext=(10, 0), textcoords="offset points", va="center", color=MUTED, fontsize=8)
    ax.set_yticks(range(len(rows))), ax.set_yticklabels([name for name, _, _ in rows])
    ax.set_ylim(-0.7, len(rows) - 0.3)
    frame(ax)


def legend(ax, where, worse=True):
    ax.plot([], [], "o", color=WAS, label="было"), ax.plot([], [], "o", color=BETTER, label="стало меньше")
    if worse: ax.plot([], [], "o", color=WORSE, label="стало больше")
    ax.legend(frameon=False, loc=where, ncol=3, fontsize=8)


def turntable():
    fig, ax = plt.subplots(figsize=(8.4, 2.4))
    dumbbell(ax, TURNTABLE, "мм", 2)
    ax.axvline(FAST_TURNTABLE, color=MUTED, linewidth=1, linestyle=(0, (3, 3)))
    ax.text(FAST_TURNTABLE, len(TURNTABLE) - 0.35, "быстрый шаг: 3,3 мм", ha="center", va="bottom", color=MUTED, fontsize=8)
    ax.set_xscale("log"), ax.set_xlim(0.03, 2500), ax.set_xlabel("уход ящика за оборот стола, мм (шкала логарифмическая)")
    legend(ax, "lower right", worse=False)
    ax.set_title("Точный шаг на вращающейся опоре: 5,5 см → 0,4 мм за оборот", pad=14)
    save(fig, "day-turntable")


def segments():
    fig, ax = plt.subplots(figsize=(8.4, 3.4))
    dumbbell(ax, SEGMENTS, "", 1)
    ax.set_xlim(0, 58), ax.set_xlabel("отрезков на кадр точного шага (меньше — дешевле)")
    legend(ax, "lower right")
    ax.set_title("Та же правка: точный шаг дробит кадр реже — кроме сцены стены")
    save(fig, "day-segments")


def levelOrder():
    fig, ax = plt.subplots(figsize=(8.4, 3.4))
    dumbbell(ax, STAGES, "мс", 2)
    ax.set_xlim(0, 31), ax.set_xlabel("время стадии, мс: пирамида из 1 015 кубов, один поток")
    legend(ax, "lower right")
    ax.set_title("Контакты по уровням (шаг 5а): кадр 67,4 → 71,0 мс — шаг откатан")
    save(fig, "day-level-order")


def checks():
    fig, ax = plt.subplots(figsize=(8.4, 4.6))
    rows = sorted(CHECKS, key=lambda row: row[1] / row[2])
    for y, (name, measured, allowed) in enumerate(rows):
        share = max(measured / allowed, 3e-10)
        color = BETTER if measured <= allowed else WORSE
        ax.plot([share, 1], [y, y], color=GRID, linewidth=2, zorder=1)
        ax.plot(share, y, "o", color=color, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
        text, side = f"{measured:.1e} при допуске {allowed:.0e}".replace("e-0", "e-"), measured <= allowed
        ax.annotate(text, (share, y), xytext=(-9 if side else 9, 0), textcoords="offset points", va="center", ha="right" if side else "left", color=MUTED, fontsize=8)
    ax.axvline(1, color=INK, linewidth=1)
    ax.text(1, len(rows) - 0.35, "допуск", ha="center", va="bottom", color=INK, fontsize=8)
    ax.set_xscale("log"), ax.set_xlim(1e-17, 3e8), ax.set_xlabel("измерено / допуск, записанный до кода (шкала логарифмическая)")
    ax.set_yticks(range(len(rows))), ax.set_yticklabels([name for name, *_ in rows])
    ax.set_ylim(-0.7, len(rows) - 0.3)
    ax.plot([], [], "o", color=BETTER, label="в допуске"), ax.plot([], [], "o", color=WORSE, label="допуск не выполнен")
    ax.legend(frameon=False, loc="upper left", ncol=1, fontsize=8)
    ax.set_title("Новые проверки силы и движимого тела: какую долю допуска занимает ошибка", pad=14)
    frame(ax), save(fig, "day-checks")


if __name__ == "__main__":
    turntable(), segments(), levelOrder(), checks()
