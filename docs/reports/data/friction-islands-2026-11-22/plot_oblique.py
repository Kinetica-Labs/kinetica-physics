"""Построить график точности и времени по двум отдельным измерениям на Windows."""
import csv
from pathlib import Path
from statistics import median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


DATA = Path(__file__).resolve().parent
OUT = DATA.parents[1] / "figures" / "sieve-oblique.svg"
INK, GRAY, TEAL, RUST = "#172f42", "#53718a", "#08786c", "#a85035"
VARIANTS = (
    ("baseline", "Исходный", GRAY), ("component", "Парный", RUST), ("disks", "Два диска", TEAL),
)
CLOCKS = (VARIANTS[0], VARIANTS[2], ("cache", "Диски с кэшем", "#459780"),
          ("box3d", "Box3D · float", "#526f91"), ("rp3d-float", "RP3D · float", "#866344"),
          ("rp3d-double", "RP3D · double", "#89636f"))


def read(path):
    with path.open(encoding="utf-8-sig", newline="") as source:
        return list(csv.DictReader(source))


def spread(ax, x, y, color):
    # Вертикальное смещение разносит совпадающие точки; физическая величина отложена по горизонтали.
    for rank, value in enumerate(sorted(x)):
        shift = .44 * (((rank * 7) % len(x)) / (len(x)-1) - .5)
        ax.plot(value, y + shift, "o", color=color, ms=3.6, alpha=.62)


def sphere(ax):
    floor, tol = 1e-13, 1e-6
    labels = []
    for y, (name, label, color) in enumerate(VARIANTS):
        rows = read(DATA / "gate02" / name / "oblique.csv")
        errors = [float(r["max_slip_error_m_s"]) for r in rows]
        if len(errors) != 48:
            raise ValueError(name)
        labels.append(f"{label}\n{sum(e <= tol for e in errors)} из 48")
        spread(ax, [max(floor, e) for e in errors], y, color)
        worst = max(errors)
        ax.plot(worst, y, "D", color=INK, ms=5)
        value = (f"{worst:.3g}" if worst > 1e-3 else f"{worst * 1e10:.2f}·10⁻¹⁰").replace(".", ",")
        ax.annotate(value, (worst, y), xytext=(-1, -25), textcoords="offset points",
                    ha="right", fontsize=9, color=INK)
    ax.axvline(tol, color="#aa6143", ls="--", lw=1.3)
    ax.text(tol, -.46, "допуск", ha="center", color="#944b36", fontsize=9)
    ax.set(xscale="log", xlim=(5e-14, .2), ylim=(2.52, -.67), xlabel="Ошибка скорости, м/с")
    ax.set_yticks(range(3), labels)
    ax.set_xticks([floor, 1e-10, 1e-6, 1e-3, 1e-1], ["≤10⁻¹³", "10⁻¹⁰", "10⁻⁶", "10⁻³", "10⁻¹"])
    ax.set_title("Шар · косое скольжение")


def cube(ax):
    rows = read(DATA / "timing.csv")
    for y, (name, _, color) in enumerate(CLOCKS):
        group = [r for r in rows if r["solver"] == name]
        if (len(group) != 12 or {int(r["block"]) for r in group} != set(range(12))
                or any(int(r["frames"]) != 360 for r in group)):
            raise ValueError(name)
        times = [float(r["us_per_frame"]) for r in group]
        spread(ax, times, y, color)
        centre = median(times)
        ax.plot(centre, y, "D", color=INK, ms=5)
        ax.text(267, y, f"{centre:.1f}".replace(".", ","), ha="right", va="center", fontweight="bold")
    ax.set(xlim=(0, 270), ylim=(5.55, -.6), xlabel="Время нагруженного кадра, мкс")
    ax.set_yticks(range(6), [label for _, label, _ in CLOCKS])
    ax.set_xticks([0, 50, 100, 150, 200])
    ax.set_title("Куб · удержание губками")


def main():
    plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "kinetica-oblique",
                         "font.size": 10, "text.color": INK, "axes.labelcolor": INK,
                         "axes.titlecolor": INK, "axes.edgecolor": "#acbbc3",
                         "xtick.color": INK, "ytick.color": INK, "axes.titlepad": 10,
                         "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(11.1, 4.85), layout="constrained")
    grid = fig.add_gridspec(2, 2, height_ratios=(1, .25))
    for i, draw in enumerate((sphere, cube)):
        ax = fig.add_subplot(grid[0, i])
        draw(ax)
        ax.tick_params(axis="y", length=0, pad=6)
        ax.spines[["top", "right", "left"]].set_visible(False)
        ax.grid(axis="x", color="#e0e6ea")
        ax.set_axisbelow(True)
    notes = ("Точка — опыт после 120 подшагов обжатия. Из двух полюсов\n"
             "взята бо́льшая ошибка. Ромб и число — максимум за 48 опытов.",
             "Точка — среднее за 360 кадров. Ромб и число — медиана\n"
             "12 замеров на одном ядре. Порядок сборок чередовался.")
    for i, text in enumerate(notes):
        note = fig.add_subplot(grid[1, i])
        note.axis("off")
        note.text(0, .7, text, fontsize=9, va="top", linespacing=1.45)
    fig.savefig(OUT, metadata={"Date": None}, bbox_inches="tight", pad_inches=.15)


if __name__ == "__main__":
    main()
