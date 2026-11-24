from pathlib import Path
import csv
import gzip

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
with gzip.open(here / "frames.csv.gz", "rt") as stream:
    baseline = list(csv.DictReader(stream))
with (here / "frames-without-bias.csv").open() as stream:
    counterfactual = list(csv.DictReader(stream))
with (here / "release.csv").open() as stream:
    hold = [r for r in csv.DictReader(stream) if r["mode"] == "hold"]

def changes(rows):
    rows = {int(r["frame"]):r for r in rows if int(r["initial_tilt_deg"]) == 16}
    dk, du = (1000 * (float(rows[33][key]) - float(rows[32][key])) for key in ("kinetic_J","potential_J"))
    return [dk,du,dk+du]

plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False})
fig, (ax,late) = plt.subplots(1,2,figsize=(10.8,4.3),layout="constrained")
x = np.arange(3)
for i,(rows,label,color) in enumerate([(baseline,"Штатный шаг","#266e98"),(counterfactual,"Без коррекции глубины","#b15b29")]):
    bars = ax.bar(x+(i-.5)*.37, changes(rows), .37, color=color, label=label)
    ax.bar_label(bars, labels=[(f"{v:+.4f}" if 0<abs(v)<0.001 else f"{v:+.3f}").replace(".",",") for v in changes(rows)], fontsize=8,padding=3)
ax.axhline(0,color="#333",lw=.8)
ax.set_xticks(x,["Кинетическая","Потенциальная","Сумма"])
ax.set_ylabel("Изменение за кадр, мДж")
ax.set_ylim(-.32,1.10)
ax.set_title("Падение с начальным наклоном 16°\nКадр 33, 0,533–0,550 с",fontsize=12)
ax.legend(frameon=False,loc="upper left",fontsize=9)
t = [int(r["frame"])/60 for r in hold]
e = np.array([float(r["potential_J"])+float(r["kinetic_J"]) for r in hold])
late.plot(t, (e-e[0])*1e9, color="#266e98", lw=1.5)
late.axhline(0,color="#333",lw=.7)
late.grid(axis="y",alpha=.2)
late.set_xlim(78,86)
late.set_xlabel("Время симуляции, с")
late.set_ylabel("Изменение K + U с 78-й секунды, нДж")
late.set_title("Отдельный опыт с отпусканием\nВерхнее ведро осталось под углом 27,03°",fontsize=12)
fig.savefig(here.parents[2]/"figures"/"bucket-energy-2026-11-23.png",dpi=170)
