"""Plot the paired Windows recordings of the bucket resting on the rim."""
import csv
import gzip
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
traces = {}
for variant in ("before", "after"):
    with gzip.open(here / f"tilted-{variant}.csv.gz", "rt") as source:
        traces[variant] = list(csv.DictReader(source))
time = [int(row["frame"]) / 60 for row in traces["after"]]
fig, (contacts, travel) = plt.subplots(1, 2, figsize=(10, 3.5), layout="constrained")
for key, label, color in (("before", "До правки", "#aa5029"), ("after", "После правки", "#226785")):
    total = np.cumsum([int(row["shown_change"]) for row in traces[key]])
    contacts.plot(time, total, label=f"{label} · {total[-1]}", color=color, lw=2)
contacts.set_title("Видимые скачки нормалей")
contacts.set_ylabel("Число переходов с начала наблюдения")
contacts.set_ylim(-18, 565)
contacts.legend(frameon=False)
travel.plot(time, [float(row["travel_m"]) * 1e9 for row in traces["after"]], color="#226785", lw=2)
travel.set_title("Движение поверхности трёх вёдер")
travel.set_ylabel("Верхняя граница сдвига, нм")
travel.set_ylim(0, 8)
for ax in (contacts, travel):
    ax.set_xlim(time[0], time[-1])
    ax.set_xlabel("Время после отпускания, с")
    ax.grid(axis="y", alpha=.25)
    ax.spines[["top", "right"]].set_visible(False)
fig.savefig(here.parents[2] / "figures" / "tilted-bucket-windows.png", dpi=150)
