"""Срез Windows-прогонов захвата. Запуск из любого каталога: python3 plot_parameters.py."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.ticker import FuncFormatter


DATA = Path(__file__).resolve().parent
OUT = DATA.parents[1] / "figures" / "sieve-parameters.svg"
# Имя CSV, подпись, настройка для среза по трению, цвет.
ENGINES = (
    ("kinetica", "Kinetica · double", 60, "#08786c"),
    ("box3d", "Box3D · float", 30, "#356886"),
    ("rp3d-float", "RP3D · float", .03, "#a46213"),
    ("rp3d-double", "RP3D · double", .03, "#954b55"),
)
MU = (.49, .50, .51)
INK = "#172f42"


def read_samples():
    samples = {}
    for name, *_ in ENGINES:
        with (DATA / "parameters" / f"{name}.csv").open(encoding="utf-8-sig", newline="") as source:
            for row in csv.DictReader(source):
                if row["hz"] != "60" or float(row["load_N"]) != 10:
                    continue
                key = name, float(row["value"]), float(row["mu"])
                if row["complete"] != "1" or key in samples:
                    raise ValueError(f"Неполная или повторная строка: {key}")
                samples[key] = float(row["drop_mm"]) - float(row["expected_mm"])
    return samples


def number(value, precision):
    return f"{value:.{precision}f}".replace("-", "−").replace(".", ",")


def friction_panel(ax, data):
    errors = [[data[name, setting, mu] for mu in MU] for name, _, setting, _ in ENGINES]
    colors = LinearSegmentedColormap.from_list("signed", ["#8db6d2", "#f3f4ef", "#de9c71"])
    limit = max(abs(x) for row in errors for x in row)
    ax.imshow(errors, cmap=colors, vmin=-limit, vmax=limit, aspect="auto")
    for y, row in enumerate(errors):
        for x, value in enumerate(row):
            ax.text(x, y, ("+" if value > 0 else "") + number(value, 2), ha="center", va="center",
                    color=INK, fontsize=12, fontweight="bold")
    ax.set_xticks(range(3), [number(mu, 2) for mu in MU])
    ax.set_yticks(range(4), [engine[1] for engine in ENGINES])
    ax.set_xlabel("Коэффициент трения μ")
    ax.set_title("Ошибка конечного смещения, мм")
    ax.tick_params(length=0, pad=6)
    for spine in ax.spines.values():
        spine.set_visible(False)


def settings_panel(ax, data):
    y_values, labels = [], []
    for group, (name, label, _, color) in enumerate(ENGINES):
        settings = (.015, .03, .06) if name.startswith("rp3d") else (30, 60)
        step = .58 if len(settings) == 3 else .72
        for i, setting in enumerate(settings):
            y = 2.4 * group + i * step
            value = data[name, setting, .50]
            units = f"{int(setting * 1000)} мм" if name.startswith("rp3d") else f"{setting} Гц"
            ax.hlines(y, 0, value, color=color, linewidth=2)
            ax.plot(value, y, "o", color=color, markersize=5)
            ax.annotate(number(value, 3), (value, y), xytext=(6, 0),
                        textcoords="offset points", va="center", fontsize=9)
            y_values.append(y)
            labels.append(f"{label}   {units}" if i == 0 else units)
    ax.set_yticks(y_values, labels)
    ax.tick_params(axis="y", length=0, pad=5)
    ax.set(xlim=(0, 3.65), ylim=(y_values[-1] + .48, -.5), xlabel="Смещение вниз, мм")
    ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x:g}".replace(".", ",")))
    ax.set_title("На границе удержания, μ = 0,50")
    ax.set_axisbelow(True)
    ax.grid(axis="x", color="#dce4e7")
    ax.spines[["top", "right", "left"]].set_visible(False)


def main():
    plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "kinetica-parameters",
                         "font.size": 10, "text.color": INK, "axes.labelcolor": INK,
                         "axes.edgecolor": "#afbdc3", "xtick.color": INK, "ytick.color": INK,
                         "axes.titlecolor": INK, "axes.titleweight": "bold", "axes.titlepad": 12})
    data = read_samples()
    fig = plt.figure(figsize=(10.8, 5.7), layout="constrained")
    grid = fig.add_gridspec(3, 2, height_ratios=(.23, 1, .36), width_ratios=(1, 1.13))
    head = fig.add_subplot(grid[0, :])
    head.axis("off")
    head.text(0, .82, "Деталь 1 кг между двумя губками", fontsize=17, fontweight="bold")
    head.text(0, .23, "По 10 Н на губку · нагрузка вниз 10 Н · 60 кадров/с · 0,25 с под нагрузкой")
    friction_panel(fig.add_subplot(grid[1, 0]), data)
    settings_panel(fig.add_subplot(grid[1, 1]), data)
    notes = (
        "Плюс означает лишнее смещение вниз, минус — недобор.\n"
        "При μ = 0,49 оракул даёт 6,25 мм. При 0,50 и 0,51 — 0 мм.\n"
        "Слева Kinetica при 60 Гц, Box3D при 30 Гц, RP3D при 30 мм.",
        "В Kinetica и Box3D меняли частоту контакта.\n"
        "В RP3D меняли порог расстояния для сохранения контакта.\n"
        "Каждая точка соответствует отдельной настройке.",
    )
    for i, note in enumerate(notes):
        ax = fig.add_subplot(grid[2, i])
        ax.axis("off")
        ax.text(0, .79, note, fontsize=8.7, va="top", linespacing=1.5)
    fig.savefig(OUT, metadata={"Date": None}, bbox_inches="tight", pad_inches=.18)


if __name__ == "__main__":
    main()
