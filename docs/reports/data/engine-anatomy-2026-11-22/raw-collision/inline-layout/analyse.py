"""Summarise the already measured native Windows runs and draw the paired changes."""
from pathlib import Path
import csv
import itertools
from statistics import median
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

data=Path(sys.argv[1])
figure=Path(sys.argv[2])
variants=("stock","force","align","both")
scenes=("grip","pyramid")
def rows(name):
    with (data/name).open(encoding="utf-8-sig",newline="") as f:
        return list(csv.DictReader(f))

timing=rows("timings.csv")
if len(timing)!=192:
    raise ValueError("expected two scenes, 24 blocks, four builds")
labels={"stock":"Обычный inline","force":"Force inline","align":"alignas(16)","both":"Оба изменения"}
names={"grip":"Захват · 360 кадров на замер","pyramid":"Пирамида · 140 кубов, 60 кадров на замер"}
colors={"force":"#245a87","align":"#aa5d20","both":"#427363"}
medians={}
summary=[]
paired=[]
with (data/"paired.csv").open("w",newline="") as f:
    writer=csv.writer(f)
    writer.writerow(["scene","block","variant","stock_us","variant_us","variant_over_stock"])
    for scene in scenes:
        obs=[r for r in timing if r["scene"]==scene]
        orders={r["order"] for r in obs}
        if orders!={"/".join(seq) for seq in itertools.permutations(variants)}:
            raise ValueError("incomplete or unbalanced design")
        if scene=="grip" and len({r["end_drift_mm"] for r in obs})!=1:
            raise ValueError("drift differs in timed runs")
        for v in variants:
            us=[]
            ratios=[]
            for block in range(1,25):
                group=[r for r in obs if r["block"]==str(block)]
                values={r["variant"]:float(r["mean_us"]) for r in group}
                if len(values)!=4 or len({r["order"] for r in group})!=1 or any(
                     r["variant"]!=r["order"].split("/")[int(r["slot"])-1] for r in group):
                    raise ValueError("invalid block")
                ratio=values[v]/values["stock"]
                us.append(values[v])
                ratios.append(ratio)
                writer.writerow([scene,block,v,values["stock"],values[v],ratio])
            medians[scene,v]=median(us)
            wins=sum(r<1 for r in ratios)
            summary.append([scene,v,median(us),median(ratios),wins,min(ratios),max(ratios),
                            int(median(ratios)<=.98 and wins>=18)])
            if v!="stock":
                paired.append((scene,v,[100*(r-1) for r in ratios]))

with (data/"summary.csv").open("w",newline="") as f:
    w=csv.writer(f)
    w.writerow(["scene","variant","median_frame_us","median_paired_ratio","wins_of_24","min_ratio","max_ratio","scene_gate"])
    w.writerows(summary)

plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.spines.top":False,"axes.spines.right":False})
fig,axes=plt.subplots(2,1,figsize=(10,5.8),sharex=True)
fig.subplots_adjust(left=.15,right=.97,top=.81,bottom=.14,hspace=.52)
for ax,scene in zip(axes,scenes):
    ax.axvline(0,color="#384550",linewidth=1)
    ax.axvline(-2,color="#718078",linewidth=1,linestyle="--")
    for s,v,deltas in paired:
        if s!=scene: continue
        j=("force","align","both").index(v)
        y=2-j
        ax.scatter(deltas,[y+(((i*7)%24)-11.5)*.017 for i in range(24)],
                   s=19,alpha=.53,color=colors[v],edgecolors="none")
        m=median(deltas)
        ax.scatter([m],[y],marker="D",s=55,color=colors[v],edgecolors="white",linewidth=1,zorder=4)
    ax.set_yticks([2,1,0],[labels[v] for v in ("force","align","both")])
    ax.set_ylim(-.5,2.55)
    ax.set_title(names[scene],loc="left",fontweight="bold",fontsize=11)
    ax.grid(axis="x",alpha=.16)
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y",length=0)
axes[-1].xaxis.set_major_formatter(FuncFormatter(lambda x,_: f"{x:g}%".replace("-", "−")))
axes[-1].set_xlabel("Изменение времени кадра относительно исходной сборки в том же блоке")
fig.suptitle("Сколько дают встраивание и выравнивание",x=.15,y=.973,ha="left",fontweight="bold",fontsize=16)
fig.text(.15,.893,"Точка — блок из четырёх сборок. Ромб — медиана 24 блоков. Пунктир — порог выигрыша 2%.",fontsize=10,color="#43505b")
fig.savefig(figure,dpi=170,facecolor="white")
