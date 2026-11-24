"""Render the report from saved native Windows CSV. No simulation is run here."""
import csv
from pathlib import Path
from statistics import median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle
from matplotlib.ticker import FuncFormatter
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LAB = ROOT/"data/engine-anatomy-2026-11-22/raw-collision"
OUT = ROOT/"figures"
BLUE, AMBER, TEAL, GRAY = "#234f7b", "#af601f", "#297467", "#52616c"
ENGINES = (("kinetica","Kinetica",BLUE), ("rp3d","RP3D",AMBER), ("box3d","Box3D",TEAL))
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.spines.right":False,
                     "axes.spines.top":False,"axes.titleweight":"bold","axes.titlelocation":"left",
                     "axes.labelcolor":"#334550","text.color":"#192c39","axes.axisbelow":True})

def read(path):
    with path.open(encoding="utf-8-sig",newline="") as f:
        return list(csv.DictReader(f))

def save(fig,key):
    fig.savefig(OUT/f"verification-2026-11-23-{key}.png",dpi=165,facecolor="white",bbox_inches="tight",pad_inches=.18)
    plt.close(fig)

def comparison():
    folder=LAB/"sparse-guides/competitors"
    if not (folder/"timing.csv").is_file(): folder=LAB/"combined-core/competitors"
    final=LAB/"memory-cpu/accepted-rows/competitors.csv"
    timings=[{"solver":r["variant"],"us_per_frame":r["mean_us"]} for r in read(final)] if final.is_file() else read(folder/"timing.csv")
    fig,axes=plt.subplots(1,2,figsize=(10.0,3.1),gridspec_kw={"width_ratios":[1.38,1]},layout="constrained")
    ti,acc=axes
    for i,(k,name,color) in enumerate(ENGINES):
        us=[float(r["us_per_frame"]) for r in timings if r["solver"]==k]
        assert len(us)==12
        y=2-i
        ti.plot([min(us),max(us)],[y,y],color=color,lw=3,alpha=.48)
        ti.scatter(us,[y+(((j*5)%12)-5.5)*.023 for j in range(12)],s=17,color=color,alpha=.5)
        ti.scatter([median(us)],[y],s=57,color=color,marker="D",edgecolor="white",lw=1,zorder=3)
        ti.text(max(us)+4,y,f"{median(us):.1f}".replace(".",","),va="center",weight="bold",color=color)
        phys=read(folder/f"{k}-grip.csv")
        good=sum(r["pass"]=="1" for r in phys)
        assert len(phys)==36
        acc.barh(y,36,color="#e9edf0",height=.45)
        acc.barh(y,good,color=color,height=.45)
        acc.text(good-1,y,f"{good}/36",ha="right",va="center",color="white",weight="bold")
    for ax in axes:
        ax.set_ylim(-.47,2.5)
        ax.set_yticks([2,1,0],[name for _,name,_ in ENGINES])
        ax.spines["left"].set_visible(False);ax.tick_params(axis="y",length=0)
        ax.grid(axis="x",alpha=.13)
    ti.set_xlim(left=0,right=max(float(r["us_per_frame"]) for r in timings)*1.18)
    ti.set_title("Время нагруженного кадра");ti.set_xlabel("мкс/кадр · ромб — медиана 12 блоков")
    acc.set_title("Случаи в физическом допуске");acc.set_xlim(0,37);acc.set_xticks([0,12,24,36]);acc.set_xlabel("одновременно верны путь и прижим")
    save(fig,"engines")

def fixture():
    fig,ax=plt.subplots(figsize=(9,3.4),layout="constrained")
    ax.set_aspect("equal");ax.set_xlim(-3.05,3.05);ax.set_ylim(-1.23,1.64);ax.axis("off")
    def arrow(start,end,color):
        ax.add_patch(FancyArrowPatch(start,end,arrowstyle="-|>",mutation_scale=16,color=color,lw=2.4,zorder=3))
    ax.plot([-2.70,-.57],[.30,.30],color="#aebbc3",lw=5)
    ax.plot([.57,2.70],[.30,.30],color="#aebbc3",lw=5)
    for x in (-1.04,.62):
        ax.add_patch(Rectangle((x,-.48),.42,1.52,facecolor="#d6e4eb",edgecolor=BLUE,lw=2))
    ax.add_patch(Rectangle((-.62,-.43),1.24,1.24,facecolor="#edf2f5",edgecolor="#384f5c",lw=2))
    ax.text(0,.39,"куб",ha="center",weight="bold",size=13);ax.text(0,.10,"1 кг",ha="center")
    arrow((-2.52,.72),(-1.06,.72),BLUE);arrow((2.52,.72),(1.06,.72),BLUE)
    ax.text(-1.76,1.01,"прижим $N$",ha="center",color=BLUE,size=12)
    ax.text(1.76,1.01,"прижим $N$",ha="center",color=BLUE,size=12)
    for x in (-.53,.53): arrow((x,-.20),(x,.64),TEAL)
    arrow((0,-.13),(0,-.98),AMBER);ax.text(.15,-.91,"нагрузка $F$",color=AMBER,va="center")
    ax.text(0,1.45,"Удержание при  |F| ≤ 2μN",ha="center",weight="bold",size=14)
    ax.text(-1.68,-.55,"ползун",ha="center",color=GRAY);ax.text(1.68,-.55,"ползун",ha="center",color=GRAY)
    save(fig,"rig")

def threshold():
    folder=LAB/"sparse-guides/competitors"
    if not (folder/"timing.csv").is_file():folder=LAB/"combined-core/competitors"
    f,axes=plt.subplots(1,2,figsize=(10.5,3.65),layout="constrained")
    for idx,ax in enumerate(axes):
        for k,name,color in ENGINES:
            rs=[r for r in read(folder/f"{k}-grip.csv") if int(r["hz"])==60 and float(r["mu"])==.5 and float(r["jaw_N"])==10 and r["open"]=="0" and 9.8<=float(r["load_N"])<=10.2]
            rs.sort(key=lambda r:float(r["load_N"]))
            x=[float(r["load_N"]) for r in rs]
            assert len(x)==5
            y=[float(r["drop_mm"] if idx==0 else r["max_path_error_mm"]) for r in rs]
            if idx==0 and k=="kinetica":
                ax.plot(x,[float(r["expected_drop_mm"]) for r in rs],color="#26333a",linestyle="--",lw=1.8,label="Аналитика",zorder=5)
            ax.plot(x,y,color=color,marker="o",markersize=4,lw=1.7,label=name,alpha=.93)
        ax.grid(alpha=.18);ax.set_xticks([9.8,9.9,10,10.1,10.2]);ax.set_xlabel("нагрузка, Н");ax.axvline(10,color="#777",lw=.7,alpha=.35)
        ax.set_title(("Куда сместился груз через 0,25 с","Наибольшая ошибка пути за 0,25 с")[idx],fontsize=11.5)
        ax.set_ylabel(("смещение вниз, мм","ошибка, мм")[idx])
    axes[0].legend(frameon=False,ncol=2,loc="upper left",fontsize=8.5)
    save(f,"threshold")

def forces():
    path=LAB/"split-force"
    fig,(free,held)=plt.subplots(1,2,figsize=(10.4,3.65),gridspec_kw={"width_ratios":[1,1.12]},layout="constrained")
    ff=[60,120,240]
    for v,label,c in (("stock","Исходный шаг",BLUE),("split","Два полуимпульса",AMBER)):
        data=read(path/f"{v}-free.csv")
        y=[1000*max(float(r["max_position_m"]) for r in data if int(r["hz"])==h) for h in ff]
        free.plot(ff,y,color=c,lw=2,marker="o",label=label)
    free.set_yscale("log");free.set_ylim(1e-12,10);free.set_yticks([1e-12,1e-8,1e-4,1])
    free.set_xticks(ff);free.set_xlabel("частота кадра, Гц · 8 подшагов");free.set_ylabel("максимальная ошибка положения, мм")
    free.set_title("Свободное движение",fontsize=12);free.grid(alpha=.18);free.legend(frameon=False,fontsize=9)
    labs=("Захват","Косой захват","V-опора")
    for i,name in enumerate(("shared8","oblique","groove")):
        for dy,v,c in ((.15,"stock",BLUE),(-.15,"split",AMBER)):
            rs=read(path/f"{name}-1-{v}.csv");count=sum(r["pass"]=="1" for r in rs)
            held.barh(2-i+dy,count,height=.24,color=c)
            held.text(count+.9,2-i+dy,f"{count}/{len(rs)}",va="center",fontsize=9,color=c)
    held.set_xlim(0,59);held.set_yticks([2,1,0],labs);held.set_title("Контакт с нагрузкой",fontsize=12)
    held.set_xlabel("число случаев в допуске");held.grid(axis="x",alpha=.17)
    save(fig,"forces")

def sparse_plot():
    f=LAB/"sparse-guides/timings.csv"
    if not f.is_file():return
    raw=read(f)
    fig,(ax,overall)=plt.subplots(1,2,figsize=(10.4,3.35),gridspec_kw={"width_ratios":[1.3,1]},layout="constrained")
    ax.axhline(0,color="#536578",lw=.9);ax.axhline(-2,ls="--",color="#8897a0",lw=.8)
    for s,label,c,y in (("grip","Захват",BLUE,1),("pyramid","Пирамида",TEAL,0)):
        ratios=[]
        for b in range(1,25):
            pair={r["variant"]:float(r["mean_us"]) for r in raw if r["scene"]==s and int(r["block"])==b}
            ratios.append(100*(pair["B"]/pair["A"]-1))
        ax.plot(range(1,25),ratios,color=c,marker="o",ms=3,lw=.7,alpha=.76,label=label)
        overall.plot([min(ratios),max(ratios)],[y,y],color=c,lw=2,alpha=.5)
        overall.scatter([median(ratios)],[y],marker="D",color=c,s=55,zorder=3)
        overall.annotate(f"{median(ratios):+.2f}%".replace(".",","),(median(ratios),y),xytext=(0,11),
                         textcoords="offset points",ha="center",color=c,weight="bold")
    ax.set_xlabel("номер парного блока");ax.set_ylabel("изменение времени кадра, %");ax.set_title("Каждая пара запусков",fontsize=12)
    ax.grid(alpha=.15);ax.legend(frameon=False,fontsize=8.5,ncol=2)
    overall.axvline(0,lw=.9,color="#536578");overall.set_yticks([1,0],["Захват","Пирамида"]);overall.set_ylim(-.55,1.65)
    overall.set_title("Медиана и полный разброс",fontsize=12);overall.set_xlabel("изменение времени, % · левее быстрее");overall.grid(axis="x",alpha=.17)
    save(fig,"sparse")

def cpu_work():
    path=LAB/"memory-cpu"
    events=read(path/"events.csv")
    tree=read(path/"phase-tree.csv")
    frames=360
    fig,(all_engines,solver)=plt.subplots(1,2,figsize=(10.5,3.6),layout="constrained")
    def bars(ax,labels,values,colors):
        y=np.arange(len(values))
        ax.barh(y,values,color=colors,height=.54)
        ax.set_yticks(y,labels,fontsize=9)
        ax.invert_yaxis()
        ax.set_xlim(0,max(values)*1.21)
        ax.spines["left"].set_visible(False)
        ax.tick_params(axis="y",length=0)
        ax.grid(axis="x",alpha=.15)
        for row,value in enumerate(values):
            ax.annotate(f"{value:.0f}",(value,row),xytext=(4,0),textcoords="offset points",va="center",fontsize=10,weight="bold")
        ax.set_xlabel("тыс. исполненных инструкций на кадр")
    engine_keys=["kinetica-coupled","rp3d","box3d"]
    work=[next(int(r["count"]) for r in events if r["variant"]==key and r["event"]=="Ir")/frames/1000 for key in engine_keys]
    bars(all_engines,["Kinetica","RP3D","Box3D"],work,[BLUE,AMBER,TEAL])
    all_engines.set_title("Полный нагруженный кадр",fontsize=12)
    stages=[
        ("Нормали до переноса","::LiveContacts::solveAndMove(","::IslandNormals::solve("),
        ("Совместное трение","::SoftStep::step(","::solveFrictionIslands("),
        ("Матрица направляющих","::SoftStep::step(","::DirectJoints::assemble("),
        ("Строки сочленений","::SoftStep::step(","::Joint::writeBonds("),
        ("Решение направляющих","::SoftStep::step(","::DirectJoints::solve(")]
    costs=[next(int(r["Ir"]) for r in tree if parent in r["parent"] and child in r["child"])/frames/1000
           for _,parent,child in stages]
    bars(solver,[s[0] for s in stages],costs,[BLUE,"#37668c","#49799b","#6593ab","#7b9fab"])
    solver.set_title("Пять участков Kinetica",fontsize=12)
    save(fig,"cpu-work")

def bullet_iterations():
    timing = read(LAB/"bullet-budget/timing.csv")
    accuracy = read(LAB/"bullet-budget/accuracy.csv")
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.3), layout="constrained")
    for precision, label, color in (("f32", "float", TEAL), ("f64", "double", BLUE)):
        prefix = f"bullet-{precision}-i"
        counts = sorted({int(r["variant"][len(prefix):]) for r in timing if r["variant"].startswith(prefix)})
        elapsed, drift = [], []
        for count in counts:
            variant = prefix + str(count)
            times = [float(r["us_per_frame"]) for r in timing if r["variant"] == variant]
            row, = [r for r in accuracy if r["variant"] == variant and int(r["hz"]) == 60
                    and float(r["mu"]) == 0.5 and float(r["jaw_N"]) == 10
                    and float(r["load_N"]) == 10 and r["open"] == "0"]
            elapsed.append(median(times))
            drift.append(float(row["max_travel_mm"]))
        for ax, values in zip(axes, (elapsed, drift)):
            ax.plot(counts, values, "o-" if precision == "f32" else "s--", color=color, label=label)
            ax.set_xticks(counts)
            ax.set_xlabel("Итерации на одно публичное обновление")
            ax.grid(axis="y", alpha=.22)
    axes[0].set_title("Время нагруженного кадра")
    axes[0].set_ylabel("Медиана, мкс")
    axes[0].axhline(50, color=AMBER, linestyle=":", label="Цель 50 мкс")
    axes[1].set_title("Удержание куба под силой 10 Н")
    axes[1].set_ylabel("Максимальный сдвиг, мм")
    axes[1].set_yscale("log")
    axes[1].set_yticks([.4,.5,.8,1.5,2.5], ["0,4","0,5","0,8","1,5","2,5"])
    axes[1].axhline(.5, color=AMBER, linestyle=":", label="Допуск 0,5 мм")
    for ax in axes:
        ax.legend(frameon=False, fontsize=8)
    save(fig, "bullet")


def full_frame_time():
    experiments = [
        ("projection","stock","Контроль"),
        ("projection","balanced","Прежние реакции"),
        ("projection","balanced-lto","Те же реакции + LTO"),
        ("projection","projected-lto","Проекция силы на площадку"),
        ("momentum","stock","Контроль, повтор"),
        ("momentum","projected","Проекция силы, повтор"),
        ("momentum","momentum","Суммирование импульсов"),
        ("bounds","projected","Проекция, третья серия"),
        ("bounds","bounds","Проекция и размеры оболочки"),
    ]
    fig, ax = plt.subplots(figsize=(9.2,5.05), layout="constrained")
    y = [9.8,8.8,7.8,6.8,5.4,4.4,3.4,2,1]
    ax.axvspan(20,50,color="#e4f2e9",zorder=0)
    ax.axvline(50,color="#227050",ls="--",lw=1.2)
    ax.text(50,10.55,"Цель 50 мкс",ha="center",color="#206444",fontsize=9,
            bbox=dict(facecolor="white",edgecolor="none",pad=1.8))
    for pos,(folder,variant,label) in zip(y,experiments):
        rows = read(LAB/"residual-budget"/folder/"timing.csv")
        values = [float(r["us_per_frame"]) for r in rows if r["variant"]==variant]
        color = TEAL if variant.startswith("projected") or variant=="bounds" else GRAY if variant=="stock" else BLUE
        ax.plot([min(values),max(values)],[pos,pos],color=color,lw=1.1)
        ax.scatter(values,[pos]*len(values),color=color,alpha=.23,s=17,edgecolor="none")
        ax.scatter([median(values)],[pos],s=60,color=color,edgecolor="white",linewidth=.7,zorder=4)
        ax.annotate(f"{median(values):.2f}".replace(".",","),(max(values)+2,pos),
                    va="center",fontsize=10,color=color,weight="bold")
    for divider in (6.1,2.7):
        ax.axhline(divider,color="#cdd3d6",lw=.8)
    ax.set(yticks=y,yticklabels=[label for _,_,label in experiments],xlim=(17,204),ylim=(.38,10.98),
           xlabel="Время полного нагруженного кадра, мкс")
    ax.set_title("На каждой строке 24 блока по 360 кадров",fontsize=13)
    ax.grid(axis="x",color="#e5e9eb",lw=.65)
    ax.tick_params(axis="y",length=0,pad=6)
    ax.spines["left"].set_visible(False)
    save(fig,"full-frame-time")


if __name__=="__main__":
    fixture();comparison();threshold();forces();sparse_plot();cpu_work();bullet_iterations();full_frame_time()
