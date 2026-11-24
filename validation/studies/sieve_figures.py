"""Построить иллюстрации отчёта из сохранённых на Windows измерений."""
import csv
import json
from pathlib import Path
from statistics import median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from matplotlib.ticker import FuncFormatter
import numpy as np


REPORTS = Path(__file__).resolve().parents[2] / "docs/reports"
DATA = REPORTS / "data"
RUN = DATA / "sieve-2026-11-22"
PAIR = DATA / "contact-pair-2026-11-22"
MEMORY = DATA / "allocator-2026-11-22"
OUT = REPORTS / "figures"
INK, BLUE, TEAL, AMBER, RED = "#172f42", "#356886", "#08786c", "#ad660e", "#ad4743"
SOLVERS = (("before", "1,00", BLUE), ("sor125", "1,25", AMBER), ("sor15", "1,50", TEAL))
plt.rcParams.update({"font.size": 10.5, "text.color": INK, "axes.labelcolor": INK,
                     "axes.titlecolor": INK, "xtick.color": INK, "ytick.color": INK,
                     "axes.edgecolor": "#abbac1", "axes.spines.top": False,
                     "axes.spines.right": False, "axes.titleweight": "bold", "axes.titlepad": 12,
                     "svg.fonttype": "none", "svg.hashsalt": "kinetica-science-20261122"})


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as source:
        return list(csv.DictReader(source))


def number(x):
    return f"{x:g}".replace(".", ",")


def pair_plot(titles, size, widths=(1, 1)):
    fig, axes = plt.subplots(1, 2, figsize=size, layout="constrained",
                             gridspec_kw={"width_ratios": widths})
    for ax, title in zip(axes, titles):
        ax.set_title(title, loc="left")
        ax.set_axisbelow(True)
    return fig, axes


def save(fig, name):
    fig.savefig(OUT / f"sieve-{name}.svg", metadata={"Date": None}, bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)


def grip_diagram():
    fig, ax = plt.subplots(figsize=(9.7, 3.3), layout="constrained")
    for x, width, base, height, colour in ((-1.14, 0.34, -0.85, 1.85, "#c7dae2"),
                                            (0.8, 0.34, -0.85, 1.85, "#c7dae2"),
                                            (-0.8, 1.6, -0.53, 1.25, "#e2f1ea")):
        ax.add_patch(Rectangle((x, base), width, height, facecolor=colour, edgecolor=INK, lw=1.5))
    def arrow(start, end, colour):
        ax.annotate("", xy=end, xytext=start, arrowprops=dict(arrowstyle="-|>", lw=2.3, color=colour))
    for direction in (-1, 1):
        arrow((direction * 2.05, 0.06), (direction * 1.17, 0.06), BLUE)
        ax.text(direction * 1.74, 0.23, "N = 10 Н", ha="center", color=BLUE)
        arrow((direction * 0.78, 0.13), (direction * 0.78, 1.22), TEAL)
        ax.text(direction * 0.78, 1.33, "трение", ha="center", color=TEAL)
    ax.text(0, 0.32, "куб 1 кг", ha="center", va="center", fontweight="bold")
    arrow((0, 0.12), (0, -1.13), RED)
    ax.text(0.12, -0.95, "F вниз", va="center", color=RED)
    ax.text(2.38, 0.58, "Обе губки свободны\nвдоль горизонтали.", linespacing=1.4)
    ax.text(2.38, -0.22, "При μ = 0,5 каждая\nудерживает до 5 Н.", linespacing=1.4)
    ax.set(xlim=(-2.3, 4.83), ylim=(-1.26, 1.67), aspect="equal")
    ax.axis("off")
    save(fig, "rig")


def grip_comparison():
    fig, (a, b) = pair_plot(("Ошибка пути в 36 опытах", "Цена кадра при нагрузке 10 Н"), (11.4, 5.7), (1.05, 1.35))
    records = rows(DATA / "grip-shared8-pairs.csv")
    labels = ["0 Н", "8 Н", "9,8 Н", "9,9 Н", "10 Н", "10,1 Н", "10,2 Н", "12 Н", "−12 Н",
              "Без трения", "Без обжатия", "С зазором"]
    def condition(row):
        return tuple(row[k] for k in ("mu", "jaw_N", "load_N", "open"))
    conditions = [condition(r) for r in records if r["hz"] == "60"]
    outcomes = ["win", "tie", "loss", "unknown"]
    grid = np.empty((len(conditions), 3), dtype=int)
    for r in records:
        grid[conditions.index(condition(r)), (60, 120, 240).index(int(r["hz"]))] = outcomes.index(r["outcome"])
    a.imshow(grid, cmap=ListedColormap(["#c6e6dd", "#e6eef1", "#f2d0bc", "#e8e4ed"]), vmin=-0.5, vmax=3.5, aspect="auto")
    for (i, j), value in np.ndenumerate(grid):
        a.text(j, i, ["лучше", "ничья", "хуже", "н/с"][value], ha="center", va="center", fontsize=9)
    a.set_yticks(range(12), labels)
    a.set_xticks(range(3), ["60 Гц", "120 Гц", "240 Гц"])
    a.tick_params(length=0, pad=6)
    samples = rows(DATA / "grip-shared8-timing.csv")
    for y, (engine, colour) in enumerate((
            ("kinetica", TEAL), ("box3d", BLUE), ("rp3d-f32", AMBER), ("rp3d-f64", RED))):
        series = sorted(float(r["us_per_frame"]) for r in samples if r["engine"] == engine + "-shared8")
        b.scatter(series, y + np.linspace(-0.14, 0.14, len(series)), s=23, color=colour, alpha=0.48)
        b.plot(median(series), y, "D", ms=7, color=INK)
        b.text(235, y, number(round(median(series), 1)), va="center", ha="right", fontweight="bold")
    b.set_yticks(range(4), ["Kinetica · double", "Box3D · float", "RP3D · float", "RP3D · double"])
    b.set_ylim(3.6, -0.6)
    b.set_xlim(0, 245)
    b.set_xlabel("Время нагруженного кадра, мкс")
    b.grid(axis="x", color="#e1e8ea")
    b.text(0, -0.14, "Точка — замер по 360 кадрам. Ромб и число — медиана.", transform=b.transAxes, fontsize=9)
    save(fig, "comparison")


def friction():
    fig, (a, b) = pair_plot(("Деталь между губками", "Шар после одного подшага"), (10.7, 4.1))
    a.axhline(0.5, color=RED, ls="--", lw=1.2, label="Допуск 0,5 мм")
    for j, (name, factor, colour) in enumerate(SOLVERS):
        trace = [r for r in rows(RUN / name / "jaws.csv") if (r["order"], r["mu"], r["load_N"]) == ("left-first", "0.50", "10.0")]
        t = [0] + [float(r["time_s"]) * 1000 for r in trace]
        drop = [0] + [float(r["relative_drop_mm"]) for r in trace]
        a.plot(t, drop, color=colour, label="ω = " + factor, lw=2.3)
        sphere = {float(r["initial_m_s"]): float(r["slip_m_s"]) * 1000 for r in rows(RUN / name / "roll.csv") if float(r["mu"]) == 0.5}
        for k, speed in enumerate((-0.01, 0.01)):
            x, value = k + 0.22 * (j - 1), sphere[speed]
            b.vlines(x, 0, value, color=colour, lw=2)
            b.plot(x, value, "o", color=colour, ms=7)
            b.text(x, value + (0.38 if value > -0.01 else -0.38), number(round(value, 2)) if abs(value) > 0.001 else "≈0",
                   va="bottom" if value > -0.01 else "top", ha="center", fontsize=9, color=colour)
    a.set(xlabel="Время под нагрузкой, мс", ylabel="Смещение вниз, мм", xlim=(0, 250), ylim=(-0.06, 1.55))
    a.legend(frameon=False, fontsize=9, loc="upper left", ncol=2)
    b.axhline(0, color=INK, lw=1, zorder=0)
    b.set(xlim=(-0.5, 1.5), ylim=(-6.5, 6.5), ylabel="Скорость скольжения, мм/с", xlabel="Начальная скорость центра шара")
    b.set_xticks([0, 1], ["−10 мм/с", "+10 мм/с"])
    for ax in (a, b):
        ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: number(x)))
        ax.grid(axis="y", color="#e1e8ea")
    save(fig, "friction")


def solver_budget():
    fig, (error_ax, time_ax) = pair_plot(("Ошибка удержания", "Время нагруженного кадра"), (9.0, 2.9))
    sweep = rows(DATA / "grip-limit-windows.csv")
    timings = rows(DATA / "grip-timing-windows.csv")
    modes = ("kinetica-realtime", "kinetica-realtime32", "kinetica-exact")
    error_ax.axvline(0.5, color=RED, ls="--", lw=1.3, label="Допуск 0,5 мм")
    for y, mode in enumerate(modes):
        cell, = (r for r in sweep if r["engine"] == mode and
                 (r["hz"], r["mu"], r["jaw_N"], r["load_N"], r["open"]) == ("60", "0.50", "10.0", "10.00", "0"))
        error = float(cell["max_path_error_mm"])
        times = [float(r["us_per_frame"]) for r in timings if r["engine"] == mode]
        error_ax.plot(error, y, "D", color=BLUE, ms=7)
        time_ax.hlines(y, min(times), max(times), color=BLUE, lw=2)
        time_ax.plot(median(times), y, "D", color=BLUE, ms=7)
        time_ax.annotate(number(round(median(times))), (median(times), y), xytext=(0, 9),
                         textcoords="offset points", ha="center", color=INK, fontsize=9)
    for ax in (error_ax, time_ax):
        ax.set_ylim(2.65, -0.6)
        ax.grid(axis="x", color="#e1e8ea")
    error_ax.set_yticks(range(3), ["Быстрый · 8", "Быстрый · 32", "Событийный"])
    time_ax.set_yticks([])
    error_ax.set(xscale="log", xlim=(3e-9, 8), xlabel="Максимальная ошибка пути, мм")
    error_ax.set_xticks([1e-8, 1e-6, 1e-4, 1e-2, 1], ["10⁻⁸", "10⁻⁶", "10⁻⁴", "0,01", "1"])
    error_ax.legend(frameon=False, fontsize=9, loc="lower right")
    time_ax.set(xlim=(0, 550), xlabel="Медиана и размах 12 замеров, мкс")
    save(fig, "budget")


def jaw_impulses():
    fig, ax = plt.subplots(figsize=(8.3, 3.4), layout="constrained")
    groups = (("sequential", "left-first", "По одному · первой левая"),
              ("sequential", "right-first", "По одному · первой правая"),
              ("block", "left-first", "Два контакта вместе"),
              ("guides", "left-first", "Два контакта и направляющие"))
    for y, (build, order, label) in enumerate(groups):
        row, = (r for r in rows(PAIR / build / "jaws.csv") if r["order"] == order and
                (r["mu"], r["load_N"], r["microstep"]) == ("0.50", "10.0", "1"))
        left, right, required = (1000 * float(row[k]) for k in ("left_up_Ns", "right_up_Ns", "required_Ns"))
        for start, width, colour, title in ((0, left, BLUE, "Левая"), (left, right, TEAL, "Правая"),
                                           (left + right, max(0, required-left-right), "#ebd4c5", "Дефицит")):
            ax.barh(y, width, height=0.57, left=start, color=colour, label=title if y == 0 else None)
            if width > 0.1:
                ax.text(start + width / 2, y, number(round(width, 2)), ha="center", va="center",
                        color=INK if title == "Дефицит" else "white", fontsize=10, fontweight="bold")
    ax.axvline(required, color=RED, ls="--", lw=1.3, label="Требуется Fh")
    ax.set_yticks(range(len(groups)), [item[2] for item in groups])
    ax.set(xlim=(0, 21.6), ylim=(3.6, -0.6), xlabel="Импульс вверх за первый подшаг, мН·с")
    ax.legend(frameon=False, loc="lower left", bbox_to_anchor=(0, 1.01), ncol=4, fontsize=9)
    save(fig, "jaws")


def robotics_scenes():
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.3), sharey=True, layout="constrained")
    angles = json.loads((RUN / "angles.json").read_text(encoding="utf-8-sig"))
    engines = ("Kinetica", "Box3D", "RP3D")
    def scene(filename, metric):
        saved = json.loads((RUN / "scenes" / filename).read_text(encoding="utf-8-sig"))
        return saved["numbers"][metric]
    pendulum = [[v["rms"] for v in angles[k]] for k in ("kp", "bx", "rp")]
    chain = [scene(f"chain100-{name}.jsonl", "gap_mm") for name in ("kp", "box3d", "rp3d-eight")]
    pyramid = [scene(name + ".jsonl", "sink_mm") for name in ("pyramid-win-kp", "pyramid-win-box3d", "pyramid55-rp3d-eight")]
    panels = (("Маятник", "RMS угла, рад", pendulum), ("Цепь 100:1", "Зазор креплений, мм", chain),
              ("Пирамида", "Осадка, мм", pyramid))
    for ax, (title, unit, series) in zip(axes, panels):
        ax.set_title(title, loc="left")
        for y, (values, colour) in enumerate(zip(series, (TEAL, BLUE, AMBER))):
            ax.scatter(values, y + np.linspace(-0.11, 0.11, len(values)), s=15, color=colour, alpha=0.45)
            ax.plot(median(values), y, "D", ms=6, color=INK)
        ax.set_xlabel(unit)
        ax.set_axisbelow(True)
        ax.grid(axis="x", color="#e1e8ea")
    axes[0].set_yticks(range(3), engines)
    axes[0].set_ylim(2.5, -0.5)
    for ax, ticks, labels in ((axes[0], [0.01, 0.1, 1], ["0,01", "0,1", "1"]),
                              (axes[1], [0.2, 1, 5, 10], ["0,2", "1", "5", "10"])):
        ax.set_xscale("log")
        ax.set_xticks(ticks, labels)
    axes[2].set_xlim(0, 6.2)
    save(fig, "scenes")


def memory_cost():
    fig, axes = pair_plot(("Собрать пять строк ползуна", "Рассчитать один кадр захвата"), (10.5, 3.15))
    panels = ((rows(MEMORY / "vector.csv"), "storage", "ns_per_list", "нс на список",
               (("new-vector", "Новый vector"), ("reuse-vector", "Буфер vector"), ("fixed-array", "Массив из пяти"))),
              (rows(MEMORY / "timing.csv"), "variant", "us_per_frame", "мкс на нагруженный кадр",
               (("heap", "Через кучу"), ("stock", "Арена"), ("reuse", "Арена и буфер"))))
    for ax, (raw, key, metric, unit, variants) in zip(axes, panels):
        for y, ((value, label), colour) in enumerate(zip(variants, (AMBER, BLUE, TEAL))):
            sample = [float(r[metric]) for r in raw if r[key] == value]
            centre = median(sample)
            ax.scatter(sample, y + np.linspace(-0.13, 0.13, len(sample)), color=colour, s=15, alpha=0.5)
            ax.plot(centre, y, "D", ms=6, color=INK)
            ax.annotate(number(round(centre, 1)), (centre, y), xytext=(0, 11), textcoords="offset points",
                        ha="center", color=INK, fontsize=9)
        ax.set_yticks(range(len(variants)), [p[1] for p in variants])
        ax.set(xlim=(0, None), ylim=(2.48, -0.58), xlabel=unit)
        ax.grid(axis="x", color="#e1e8ea")
    save(fig, "memory")


def groove_errors():
    fig, axes = pair_plot(("Проверенные углы 30° и 45°", "Отложенные углы 20° и 55°"), (9.8, 3.4))
    names = (("sequential", "Исходный"), ("block", "Два контакта"), ("guides", "С направляющими"),
             ("active", "С пределом трения"), ("centred", "Две полусилы"))
    runs = {r["build"]: r for r in json.loads((PAIR / "verdict.json").read_text(encoding="utf-8-sig"))["runs"]}
    for ax, field in zip(axes, ("groove", "groove-holdout")):
        for y, (key, label) in enumerate(names):
            row = runs[key]
            error = row[field + "_max_error_m_s"]
            ax.plot(error, y, "o", color=TEAL if error <= 1e-6 else RED, ms=8)
            ax.annotate(f'{row[field + "_pass"]}/32', (error, y), xytext=(9, -3), textcoords="offset points", fontsize=9)
        ax.axvline(1e-6, color=INK, ls="--", lw=1.2)
        ax.set(xscale="log", xlim=(1e-10, 0.5), ylim=(4.5, -0.5), xlabel="Максимум ошибки скольжения, м/с")
        ax.set_yticks(range(len(names)), [p[1] for p in names])
        ax.grid(axis="x", color="#e1e8ea")
    save(fig, "groove")


def diagnosis():
    fig, (fall, islands) = pair_plot(("Свободное движение груза", "Куб на опоре в 50 м от захвата"), (10.0, 3.55))
    series = [r for r in rows(PAIR / "active" / "grip.csv") if
              (r["mu"], r["jaw_N"], r["load_N"], r["open"]) == ("0.00", "10.0", "10.00", "0")]
    hz = [int(r["hz"]) for r in series]
    frequency = np.linspace(min(hz), max(hz), 160)
    predicted = 1000 * 10 * 0.25 / (2 * 8 * frequency)
    fall.plot(frequency, predicted, color=INK, label=r"Расчёт $aTh/2$")
    fall.plot(hz, [float(r["max_path_error_mm"]) for r in series], "o", ms=8,
              mfc="white", mec=TEAL, mew=1.8, label="Измерено")
    fall.set_xticks(hz)
    fall.set(xlabel="Частота кадра, Гц", ylabel="Ошибка координаты за 0,25 с, мм", ylim=(0, 3))
    runs = (("sequential", "touch-right", "Штатный с соседом", INK, ":"),
            ("active", "alone", "Прототип один", TEAL, "-"),
            ("active", "touch-right", "Прототип с соседом", RED, "--"))
    for build, scene, label, colour, style in runs:
        sample = [r for r in rows(PAIR / "islands" / (build + ".csv")) if r["scene"] == scene]
        islands.plot([1000 * int(r["microstep"]) / 480 for r in sample], [float(r["drop_mm"]) for r in sample],
                     color=colour, lw=2.4 if build == "sequential" else 1.7, ls=style, label=label)
    islands.set(xlabel="Время после приложения нагрузки, мс", ylabel="Смещение куба вниз, мм")
    for ax in (fall, islands):
        ax.grid(color="#e1e8ea")
        ax.legend(frameon=False, fontsize=8.5)
    save(fig, "diagnosis")


def gears():
    fig, (a, b) = pair_plot(("Движение двух валов", "Ошибка угла относительно эталона"), (10.7, 4.0))
    data = rows(RUN / "gear-trajectory.csv")
    for mode in ("exact", "realtime"):
        points = [r for r in data if r["mode"] == mode]
        t = np.array([float(r["time_s"]) for r in points])
        oracle = np.degrees([float(r["oracle_a_rad"]) for r in points])
        for shaft, ratio, shade in (("a", 1, TEAL), ("b", -0.5, BLUE)):
            angle = np.degrees([float(r[shaft + "_rad"]) for r in points])
            if mode == "exact":
                a.plot(t, ratio * oracle, c=INK, lw=1, ls="--", label="Эталон" if shaft == "a" else None)
                a.plot(t[::8], angle[::8], "o", mec=shade, mfc="white", ms=4, zorder=3)
            else:
                a.plot(t, angle, color=shade, lw=1.8, label="Вход A" if shaft == "a" else "Выход B")
            b.plot(t, angle - ratio * oracle, color=shade if mode == "realtime" else INK,
                   ls="-" if mode == "realtime" else "--", lw=1.8,
                   label=("Быстрый · " + shaft.upper()) if mode == "realtime" else ("Событийный · A, B" if shaft == "a" else None))
    a.set_ylabel("Угол, °")
    b.set_ylabel("Угол модели минус эталон, °")
    for ax in (a, b):
        ax.set_xlabel("Время, с")
        ax.legend(frameon=False, fontsize=9)
        ax.grid(color="#e1e8ea")
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: number(x)))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: number(x)))
    save(fig, "gears")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    grip_diagram()
    grip_comparison()
    friction()
    gears()
    solver_budget()
    jaw_impulses()
    robotics_scenes()
    groove_errors()
    memory_cost()
    diagnosis()
