#!/usr/bin/env python3
"""Рисунок и таблица отчёта docs/reports/ablation-2026-11-21.md по данным validation/ablation.py.

Сырьё — docs/reports/data/ablation.json. Запуск переписывает docs/reports/figures/ablation-frame.svg и
печатает таблицу Markdown; --png кладёт рядом png для проверки глазами.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from status_figures import BETTER, GRID, INK, MUTED, SURFACE, WAS, WORSE, frame, plt, ru, save

DATA = Path(__file__).resolve().parents[2] / "docs/reports/data/ablation.json"
NOISE = 1.0  # %: расхождение одинаковых сборок в этом замере (строка base)


def table():
    """Строки: (название, кадр одним потоком %, всеми потоками %, сцены %, прошло, перекрытие мм, сцен с другим ответом)."""
    results = json.loads(DATA.read_text(encoding="utf-8"))
    base = results.pop("base")
    scenes = lambda run: sum(row["ms_per_frame"] for row in run["rows"])
    rows = []
    for key, run in results.items():
        change = lambda on, off: 100 * (min(run[off]) / min(run[on]) - 1)
        rows.append({"title": run["title"], "one": change("A0", "A1"), "all": change("B0", "B1"), "scenes": 100 * (scenes(run) / scenes(base) - 1),
                     "passed": sum(row["ok"] for row in run["rows"]), "overlap": 1000 * max(row["overlap_m"] for row in run["rows"]),
                     "changed": sum(run["hash"].get(scene) != code for scene, code in base["hash"].items())})
    summary = {"passed": sum(row["ok"] for row in base["rows"]), "overlap": 1000 * max(row["overlap_m"] for row in base["rows"]), "scenes": scenes(base),
               "one": min(base["A0"]), "all": min(base["B0"]), "noise": 100 * abs(min(base["A1"]) / min(base["A0"]) - 1), "count": len(base["rows"]), "hashes": len(base["hash"])}
    return rows, summary


def draw(rows):
    rows = sorted(rows, key=lambda row: max((row["one"], row["all"], row["scenes"]), key=abs))
    fig, ax = plt.subplots(figsize=(8.4, 5.2))
    for y, row in enumerate(rows):
        for shift, key, color in ((0.24, "one", BETTER), (0.0, "all", WORSE), (-0.24, "scenes", WAS)):
            ax.barh(y + shift, row[key], height=0.22, color=color, edgecolor=SURFACE, linewidth=1)
        most, right = max((row["one"], row["all"], row["scenes"]), key=abs), max(row["one"], row["all"], row["scenes"], 0)
        ax.annotate(f"{'+' if most > 0 else '−'}{ru(abs(most), 0 if abs(most) >= 10 else 1)} %", (right, y), xytext=(6, 0), textcoords="offset points", va="center", color=MUTED, fontsize=8)
    ax.axvline(0, color=INK, linewidth=1)
    ax.set_yticks(range(len(rows))), ax.set_yticklabels([row["title"] for row in rows])
    ax.set_xlim(-40, 105), ax.set_xlabel("на сколько дольше считается кадр без приёма, % (правее — приём нужнее)")
    ax.plot([], [], "s", color=BETTER, label="пирамида из 1 015 кубов, один поток"), ax.plot([], [], "s", color=WORSE, label="она же, все потоки")
    ax.plot([], [], "s", color=WAS, label="стандартные сцены, сумма")
    ax.legend(frameon=False, loc="lower right", fontsize=8)
    ax.set_title("Абляция быстрого шага: что теряется, если выключить один приём")
    frame(ax), save(fig, "ablation-frame")


def markdown(rows, summary):
    sign = lambda value: f"{'+' if value > 0 else '−' if value < 0 else ''}{ru(abs(value), 0 if abs(value) >= 10 else 1)} %"
    lines = ["| Приём (выключен) | кадр, один поток | кадр, все потоки | стандартные сцены, сумма | сцен прошло | наибольшее перекрытие | ответ изменился |", "|---|---|---|---|---|---|---|"]
    for row in sorted(rows, key=lambda row: -max((row["one"], row["all"], row["scenes"]), key=abs)):
        lines.append(f"| {row['title']} | {sign(row['one'])} | {sign(row['all'])} | {sign(row['scenes'])} | {row['passed']} из {summary['count']} | {ru(row['overlap'], 2)} мм | "
                     f"{'нет' if row['changed'] == 0 else 'в ' + str(row['changed']) + ' из ' + str(summary['hashes'])} |")
    lines.append(f"| *всё включено* | {ru(summary['one'], 2)} мс | {ru(summary['all'], 2)} мс | {ru(summary['scenes'], 2)} мс | {summary['passed']} из {summary['count']} | {ru(summary['overlap'], 2)} мм | — |")
    return "\n".join(lines)


if __name__ == "__main__":
    found, total = table()
    draw(found)
    print(markdown(found, total))
    print(f"\nшум замера (одинаковые сборки): {ru(total['noise'], 2)} %")
