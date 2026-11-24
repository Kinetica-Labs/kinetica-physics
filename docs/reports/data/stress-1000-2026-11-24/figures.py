from pathlib import Path
import csv
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import LinearSegmentedColormap

data, summary_file, output = map(Path, sys.argv[1:])
def read(path):
    with path.open(encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))

plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":11, "axes.spines.top":False,
                     "axes.spines.right":False, "axes.titlelocation":"left", "figure.facecolor":"white"})
engines = ["kinetica","box3d","rp3d","bullet","jolt"]
labels = ["Kinetica","Box3D","ReactPhysics3D","Bullet","Jolt"]
colors = ["#137870","#416588","#477ba2","#6190ad","#738fa5"]
stats={(r["scene"],r["engine"]):r for r in read(summary_file)}
scenes=["pyramid1000","rain1000"]
fig, axs=plt.subplots(1,2,figsize=(11,4.2),sharey=True,constrained_layout=True)
for ax, scene, title in zip(axs,scenes,["Обрушение 3D-пирамиды","Падение решётки кубов"]):
    for i,engine in enumerate(engines):
        row=stats[scene,engine]
        ms,p99=map(float,(row["median_block_ms"],row["p99_ms"]))
        ax.barh(i,ms,color=colors[i],height=.46,zorder=2)
        ax.plot([ms,p99],[i,i],color="#65717b",lw=1,zorder=1)
        ax.plot(p99,i,marker="|",color="#283846",ms=10)
        short = scene=="pyramid1000" and ms<18
        ax.text(p99+4 if short else ms*.5,i,f"{ms:.1f}".replace(".",","),ha="left" if short else "center",
                va="center",color="#253746" if short else "white",weight="bold",fontsize=10)
    ax.axvline(1000/60,color="#be6031",lw=1.2,ls="--",zorder=0)
    ax.set_title(title,weight="bold")
    ax.set_xlabel("мс на физический кадр")
    ax.set_yticks(range(5),labels)
    ax.grid(axis="x",alpha=.15,zorder=0)
axs[0].invert_yaxis()
fig.suptitle("1000 активных кубов · один логический процессор",weight="bold",fontsize=15)
fig.savefig(output/"stress-1000-time.png",dpi=155,bbox_inches="tight")
plt.close(fig)

records=read(data/"pyramid1000-kinetica-verify-0-poses.csv")
layers=np.repeat(np.arange(10),np.array([16,14,13,12,10,8,6,5,3,1])**2)
cmap=LinearSegmentedColormap.from_list("layers",["#33747f","#54a6a2","#dcc76c","#e78b45","#9b4431"])
corners=.125*np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
faces=[[4,5,7,6],[2,6,7,3],[1,3,7,5],[0,2,3,1],[0,1,5,4],[0,4,6,2]]
normals=np.array([[1,0,0],[0,1,0],[0,0,1],[-1,0,0],[0,-1,0],[0,0,-1]])
camera=np.array([.56,.54,.63]); camera/=np.linalg.norm(camera)
horizontal=np.array([camera[2],0,-camera[0]]); horizontal/=np.linalg.norm(horizontal)
vertical=np.cross(camera,horizontal)
def project(points):
    return np.stack((points@horizontal,points@vertical),axis=-1)

fig,axes=plt.subplots(1,3,figsize=(13.3,4.6),constrained_layout=True)
for ax,frame_index in zip(axes,[0,30,120]):
    rows=[r for r in records if int(r["frame"])==frame_index]
    assert [int(r["id"]) for r in rows]==list(range(1,1001))
    faces2d, fill, order = [],[],[]
    for r,layer in zip(rows,layers):
        at=np.array([float(r[k]) for k in ["x_m","y_m","z_m"]])
        w,x,y,z=[float(r[k]) for k in ["qw","qx","qy","qz"]]
        rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        pts=at+corners@rot.T
        for ids,n in zip(faces,normals@rot.T):
            if n@camera<=0: continue
            faces2d.append(project(pts[ids])); order.append(float(pts[ids].mean(0)@camera))
            shade=.68+.32*max(0,n@np.array([.25,.9,.28]))
            fill.append(np.array(cmap(layer/9)[:3])*shade)
    idx=np.argsort(order)
    for m in range(-4,5):
        for a,b in [([m,0,-4],[m,0,4]),([-4,0,m],[4,0,m])]:
            line=project(np.array([a,b])); ax.plot(*line.T,color="#d6dcde",lw=.65,zorder=0)
    ax.add_collection(PolyCollection(np.array(faces2d)[idx],facecolors=np.array(fill)[idx],
                                     edgecolors="#374953",linewidths=.16,zorder=2))
    ax.set(xlim=(-6.7,6.7),ylim=(-3.5,3.7),aspect="equal")
    ax.axis("off")
    ax.set_title(["Старт","Через 0,5 с","Через 2 с"][[0,30,120].index(frame_index)],weight="bold")
fig.suptitle("Как рассыпалась пирамида в Kinetica",weight="bold",fontsize=17)
fig.text(.5,.08,"Цвет показывает исходный слой, от бирюзового основания до оранжевой вершины. Шаг сетки пола 1 м.",ha="center")
fig.savefig(output/"stress-1000-collapse.png",dpi=170,bbox_inches="tight")
plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(11,4.2),constrained_layout=True)
palette=["#137870","#40658e","#db8b40","#8b6297","#ba5542"]
for ax,scene,title in zip(axes,scenes,["Обрушение пирамиды","Падение решётки"]):
    for key,label,col in zip(engines,labels,palette):
        rows=read(data/f"{scene}-{key}-verify-0.csv")
        yy=[float(r["pair_mm"]) for r in rows]
        ax.plot([int(r["frame"])/60 for r in rows],yy,color=col,label=label,lw=1.45,alpha=.93)
    ax.axhline(5,color="#343434",ls="--",lw=1)
    ax.set(yscale="symlog",ylim=(0,None),ylabel="перекрытие кубов, мм",xlabel="физическое время, с")
    ax.set_title(title,weight="bold")
    ax.grid(alpha=.2)
axes[1].legend(loc="best",fontsize=8)
fig.suptitle("Наибольшее перекрытие двух кубов в каждом кадре",weight="bold",fontsize=15)
fig.savefig(output/"stress-1000-depth.png",dpi=155,bbox_inches="tight")
plt.close(fig)

stage_path=summary_file.parent/"stage-cost.csv"
if stage_path.exists():
    values={r["stage"]:float(r["total_ms"])/120 for r in read(stage_path)}
    items=[("Поиск пар и точек",values["Pairs"]),("Повторные проверки",values["Recheck"]),
           ("Решение нормалей",values["Normals"]),("Подготовка контактов",values["Contacts"]),
           ("Перемещение тел",values["Motion"])]
    items.append(("Остальные стадии",sum(values.values())-sum(v for _,v in items)))
    fig,ax=plt.subplots(figsize=(8,3.8),constrained_layout=True)
    for i,(label,v) in enumerate(items):
        ax.barh(i,v,height=.55,color=("#137870" if i<2 else "#63849a"))
        ax.text(v+.8,i,f"{v:.2f}".replace(".",","),va="center")
    ax.axvline(10.76,color="#be6031",ls="--",lw=1.2,label="Весь кадр Box3D")
    ax.set_yticks(range(len(items)),[s for s,_ in items])
    ax.set(xlim=(0,72),xlabel="мс на физический кадр")
    ax.invert_yaxis()
    ax.legend(loc="lower right",fontsize=9)
    ax.set_title("Kinetica при обрушении · 120 кадров",weight="bold")
    fig.savefig(output/"stress-1000-stages.png",dpi=155,bbox_inches="tight")
    plt.close(fig)

accuracy = summary_file.parent / "accuracy-sweep.csv"
if accuracy.exists():
    pyramid = [r for r in read(accuracy) if r["scene"] == "pyramid1000"]
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.7), constrained_layout=True)
    for ax, key, limit, title in zip(axes, ("max_floor_mm", "max_pair_mm"), (2, 5),
                                    ("Куб и пол", "Два куба")):
        for r in pyramid:
            x, y = float(r["single_run_mean_ms"]), float(r[key])
            ax.scatter(x, y, s=65, color="#137870" if y <= limit else "#b25735", zorder=3)
            label = r["recheck_mm"].replace(".", ",") + " мм"
            ax.annotate(label, (x, y), xytext=(0, -17 if y <= limit else 9),
                        textcoords="offset points", ha="center")
        ax.axhline(limit, ls="--", color="#46535b", label=f"Допуск {limit} мм", lw=1.2)
        ax.set(xlim=(103, 204), ylim=(0, 4 if limit == 2 else 9), xlabel="мс за кадр · один прогон",
               ylabel="Максимум проникновения, мм")
        ax.set_title(title, weight="bold")
        ax.grid(axis="y", alpha=.2)
        ax.legend(loc="lower right", fontsize=9)
    fig.suptitle("Порог перепроверки контакта при обрушении 1000 кубов", weight="bold")
    fig.savefig(output / "stress-1000-threshold.png", dpi=155, bbox_inches="tight")
    plt.close(fig)
