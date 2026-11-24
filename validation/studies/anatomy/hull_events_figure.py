"""Orthographic views of the poses exported by hull_event_sweep; no physics calculations."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from matplotlib.patches import Polygon
import numpy as np


TYPES = ("vertex", "edge", "face")
NAMES = {"vertex": "Вершина", "edge": "Ребро", "face": "Грань"}
STYLE = {"A": ("#57adb2", "#115159"), "B": ("#e5a169", "#723d18")}
INK = "#142c39"


def records(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def unit(v):
    return v / np.linalg.norm(v)


def meshes(path):
    vertices = defaultdict(dict)
    faces = defaultdict(lambda: defaultdict(dict))
    for r in records(path):
        body = r["shape"]
        i, f, k = (int(r[c]) for c in ("vertex", "face", "loop"))
        p = [float(r[d + "_m"]) for d in "xyz"]
        if i in vertices[body] and vertices[body][i] != p:
            raise ValueError(f"Inconsistent vertex {body}/{i}")
        vertices[body][i] = p
        faces[body][f][k] = i
    result = {}
    for body, coords in vertices.items():
        pts = np.array([coords[i] for i in range(len(coords))])
        loops = [[loop[j] for j in range(len(loop))] for _, loop in sorted(faces[body].items())]
        edges = sorted({tuple(sorted((a, b))) for loop in loops for a,b in zip(loop, loop[1:] + loop[:1])})
        result[body] = pts, dict(vertex=[[i] for i in range(len(pts))], edge=edges, face=loops)
    return result


def posed_points(row, prefix, vertices):
    q = np.array([float(row[prefix + "q" + d]) for d in "wxyz"])
    if not np.isclose(q @ q, 1, atol=1e-12, rtol=0):
        raise ValueError(f"Invalid quaternion {prefix}")
    translation = np.array([float(row[prefix + d + "_m"]) for d in "xyz"])
    t = 2 * np.cross(q[1:], vertices)
    return vertices + q[0] * t + np.cross(q[1:], t) + translation


def project(row, models):
    witness = np.array([float(row["witness_" + d + "_m"]) for d in "xyz"])
    n = unit(np.array([float(row["n" + d]) for d in "xyz"]))
    tangent = unit(np.cross(n, [0, 0, 1]))
    view = unit(tangent + .21*n)  # Плоскости касания видны слегка наискосок.
    right = unit(n - (n @ view) * view)
    up = unit(np.cross(view, right))
    basis = np.column_stack([right, up, view])
    objects = {}
    for body, (vertices, features) in models.items():
        key = body.lower()
        pts = (posed_points(row, key, vertices) - witness) @ basis
        kind, i = row["type_"+key], int(row["id_"+key])
        selected = list(features[kind][i])
        if np.linalg.norm(pts[selected].mean(axis=0)) > 2e-9:
            raise ValueError(f"Exported witness is outside the selected feature centre on {body}")
        objects[body] = (pts, features["face"], kind, selected)
    return objects


def draw(ax, objects, marker=(0, 0)):
    faces = []
    for body, (pts, loops, _, _) in objects.items():
        for loop in loops:
            poly = pts[loop]
            normal = unit(np.cross(poly[1]-poly[0], poly[2]-poly[0]))
            if normal[2] < 0:
                continue
            light = .84 + .16 * normal @ unit(np.array([-.3,.7,1]))
            faces.append((poly[:,2].mean(), poly[:,:2], np.array(to_rgb(STYLE[body][0]))*light, STYLE[body][1]))
    for _, poly, color, outline in sorted(faces, key=lambda f:f[0]):
        ax.add_patch(Polygon(poly, closed=True, facecolor=color, edgecolor=outline, lw=.8, joinstyle="round"))
    # Поверх диагностической проекции выделяем сами выбранные рёбра и контуры граней.
    for body,(pts,_,kind,indices) in objects.items():
        feature = pts[indices,:2]
        if kind == "face":
            feature = np.vstack([feature, feature[0]])
        if len(feature) > 1:
            ax.plot(*feature.T, color=STYLE[body][1], lw=1.45,
                    ls="-" if body == "A" else (0,(2.5,1.5)), solid_capstyle="round")
    ax.plot(*marker, marker="o", ms=7.2, mfc="white", mec=INK, mew=.9)
    ax.plot(*marker, marker="o", ms=3.8, mfc="#ac1848", mec="none")
    ax.set_aspect("equal")
    ax.axis("off")


def ru_pairs(n):
    if 11 <= n % 100 <= 14:
        return "пар"
    return {1:"пара",2:"пары",3:"пары",4:"пары"}.get(n % 10,"пар")


def animated_data(rows, models):
    rows = sorted((r for r in rows if r["type_a"] == r["type_b"] == "edge"
                   and r["id_a"] == r["id_b"] == "0"), key=lambda r: int(r["frame"]))
    if len(rows) < 2 or [int(r["frame"]) for r in rows] != list(range(len(rows))):
        raise ValueError("Missing edge-edge animation frames")
    normal = unit(np.array([float(rows[0]["n" + d]) for d in "xyz"]))
    tangent = unit(np.cross(normal, [0, 0, 1]))
    side = np.cross(normal, tangent)
    view = unit(.40*tangent + .90*side + .18*normal)
    right = unit(normal - (normal @ view)*view)
    basis = np.column_stack([right, unit(np.cross(view, right)), view])
    frames = []
    times = np.array([float(row["t"]) for row in rows])
    if np.any(np.diff(times) <= 0):
        raise ValueError("Trajectory parameter must increase")
    gaps = []
    for row in rows:
        objects, centres = {}, {}
        for body, (vertices, features) in models.items():
            key = body.lower()
            pts = posed_points(row, key, vertices)
            indices = list(features["edge"][int(row["id_" + key])])
            centre = np.array([float(row["witness_" + key + d + "_m"]) for d in "xyz"])
            if np.linalg.norm(pts[indices].mean(axis=0) - centre) > 2e-9:
                raise ValueError(f"Exported witness differs from reconstructed edge centre on {body}")
            projections = pts @ normal
            support = projections.max() if body == "A" else projections.min()
            if abs(support - centre @ normal) > 2e-9:
                raise ValueError(f"Edge {body} lies outside its supporting plane")
            centres[body] = centre
            objects[body] = pts @ basis, features["face"], "edge", indices
        step_normal = np.array([float(row["n" + d]) for d in "xyz"])
        measured = (centres["B"] - centres["A"]) @ normal
        if np.linalg.norm(step_normal - normal) > 2e-12 or abs(measured - float(row["gap_m"])) > 2e-9:
            raise ValueError("Signed normal gap does not match the witness points")
        gaps.append(1000 * measured)
        middle = ((centres["A"] + centres["B"]) * .5) @ basis
        frames.append((objects, middle[:2]))
    return frames, times, np.array(gaps)


def draw_gap(ax, times, gaps, limits, detail=False):
    ax.axhline(0, color="#51636b", lw=1, ls=(0, (3, 2)))
    ax.plot(times, gaps, color="#285468", lw=2, marker=".", ms=3)
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["bottom", "left"]].set_color("#b1c0c5")
    ax.set_xlim(*(limits[0]))
    ax.set_ylim(*(limits[1]))
    ticks = np.array([-.1, -.05, 0]) if detail else np.array([0, 1, 2, 3])
    ax.set_yticks(ticks, [f"{v:g}".replace(".", ",") for v in ticks])
    ax.set_xticks([.25, .5, .75] if detail else [0, .25, .5, .75, 1],
                  ["0,25", "0,50", "0,75"] if detail else ["0", "0,25", "0,50", "0,75", "1"])
    ax.tick_params(colors="#52666e", labelsize=8, length=3)
    ax.grid(axis="y", color="#e0e6e8", lw=.55)
    ax.set_title("Вблизи касания, мм" if detail else "Зазор вдоль нормали, мм",
                 loc="left", fontsize=11, color=INK, weight="bold", pad=5)
    return ax.plot([], [], "o", ms=6, color="#ac1848", mec="white", mew=.7, clip_on=True)[0]


def animation(path, output, models):
    from PIL import Image

    frames, times, gaps = animated_data(records(path), models)
    cloud = np.vstack([obj[0] for objects, _ in frames for obj in objects.values()])
    xmin, ymin = cloud[:, :2].min(axis=0) - .035
    xmax, ymax = cloud[:, :2].max(axis=0) + .035
    fig = plt.figure(figsize=(9, 4.85), dpi=110, facecolor="white")
    fig.text(.04,.915,"Как сходятся и расходятся два ребра",fontsize=18,weight="bold",color=INK)
    fig.text(.04,.85,"Кинематическая траектория двух выпуклых тел",fontsize=11,color="#465d68")
    fig.text(.04,.748,"Тело A",fontsize=11,weight="bold",color=STYLE["A"][1])
    fig.text(.15,.748,"Тело B",fontsize=11,weight="bold",color=STYLE["B"][1])
    fig.text(.27,.748,"●",fontsize=11,color="#ac1848")
    fig.text(.29,.748,"середина между рёбрами¹",fontsize=9,color=INK)
    scene = fig.add_axes([.027,.24,.54,.46])
    full = fig.add_axes([.65,.54,.32,.19])
    detail = fig.add_axes([.65,.235,.32,.15])
    full_dot = draw_gap(full,times,gaps,((-.025,1.025),(-.25,3.18)))
    detail_dot = draw_gap(detail,times,gaps,((.215,.785),(-.113,.016)),detail=True)
    detail.set_xlabel("Параметр траектории",fontsize=8,color=INK,labelpad=3)
    status = fig.text(.04,.20,"",fontsize=13,weight="bold",color=INK)
    fig.text(.04,.093,"¹ Выделены рёбра № 0 каждого тела. Зазор вычислен по проекции разности их центров на нормаль.",fontsize=9,color="#465d68")
    fig.text(.04,.045,"Неподвижная камера. Отрицательные значения означают перекрытие вдоль общей нормали.",fontsize=9,color="#465d68")
    images = []
    for i, ((objects, marker), t, gap) in enumerate(zip(frames, times, gaps)):
        scene.clear()
        draw(scene,objects,marker)
        scene.set_xlim(xmin,xmax)
        scene.set_ylim(ymin,ymax)
        for dot in (full_dot,detail_dot):
            dot.set_data([t],[gap])
        if abs(gap) <= 2e-6:
            word, value = "Касание", 0.0
        elif gap > 0:
            word, value = ("Сближение" if i < len(frames)//2 else "Расхождение"), gap
        else:
            word, value = "Перекрытие", -gap
        status.set_text(f"{word} · {value:.3f} мм".replace(".",","))
        fig.canvas.draw()
        image = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy())
        images.append(image.convert("P",palette=Image.Palette.ADAPTIVE,colors=192,dither=Image.Dither.NONE))
    durations = [150] * len(images)
    for i in (0,int(np.argmin(gaps)),len(images)-1,*np.flatnonzero(np.abs(gaps) <= 2e-6)):
        durations[i] = 550
    images[0].save(output,save_all=True,append_images=images[1:],duration=durations,loop=0,optimize=False)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--animation", type=Path, help="Render the edge-edge trajectory from this exported CSV as GIF")
    args = parser.parse_args()
    models = meshes(args.input/"geometry.csv")
    if args.animation:
        animation(args.animation, args.output, models)
        return
    rows = records(args.input/"illustrations.csv")
    pictures = {(r["type_a"],r["type_b"]): project(r,models) for r in rows}
    if len(rows) != 9 or len(pictures) != 9:
        raise ValueError("Expected nine distinct poses")
    cloud = np.vstack([o[0] for scene in pictures.values() for o in scene.values()])
    xl, xr = cloud[:,0].min()-.06, cloud[:,0].max()+.06
    yb, yt = cloud[:,1].min()-.05, cloud[:,1].max()+.05
    fig = plt.figure(figsize=(10.6,8.4),facecolor="white")
    fig.text(.047,.950,"Как соприкасаются два выпуклых тела",fontsize=21,weight="bold",color=INK)
    count = np.prod([sum(len(f) for f in model[1].values()) for model in models.values()])
    fig.text(.048,.910,f"В каждой ячейке показана одна поза касания. Всего в опыте {count:,} индексных пар элементов.".replace(","," "),fontsize=11,color="#465d68")
    fig.text(.048,.868,"Тело A",fontsize=11,weight="bold",color=STYLE["A"][1])
    fig.text(.133,.868,"Тело B",fontsize=11,weight="bold",color=STYLE["B"][1])
    fig.text(.240,.868,"●",fontsize=12,color="#ac1848")
    fig.text(.263,.868,"общая точка выбранных элементов",fontsize=10,color=INK)
    left, right, top, bottom = .145,.975,.789,.135
    dx, dy = (right-left)/3,(top-bottom)/3
    for j, kind in enumerate(TYPES):
        fig.text(left+(j+.5)*dx,top+.022, NAMES[kind]+" B",ha="center",fontsize=12,weight="bold",color=STYLE["B"][1])
    for i, kind_a in enumerate(TYPES):
        fig.text(left-.018,top-(i+.46)*dy,NAMES[kind_a]+" A",ha="right",va="center",fontsize=11,weight="bold",color=STYLE["A"][1])
        for j, kind_b in enumerate(TYPES):
            x, y = left+j*dx,top-(i+1)*dy
            ax=fig.add_axes([x+.004,y+.032,dx-.008,dy-.035])
            draw(ax,pictures[kind_a,kind_b])
            ax.set_xlim(xl,xr)
            ax.set_ylim(yb,yt)
            n=len(models["A"][1][kind_a])*len(models["B"][1][kind_b])
            fig.text(x+dx*.5,y+.013,f"{n} {ru_pairs(n)}",ha="center",color=INK,fontsize=10)
    for j in range(1,3):
        fig.add_artist(plt.Line2D([left+j*dx]*2,[bottom,top],transform=fig.transFigure,color="#d7e1e5",lw=.8))
    for i in range(4):
        fig.add_artist(plt.Line2D([left,right],[top-i*dy]*2,transform=fig.transFigure,color="#d7e1e5",lw=.8))
    fig.text(.048,.080,"Число под рисунком — количество сочетаний конкретных элементов этого типа у A и B.",fontsize=10,color=INK)
    fig.text(.048,.048,"Маркер лежит в центре обоих выбранных элементов. У совпадающих граней он отмечает одну точку общей площадки.",fontsize=9,color="#465d68")
    fig.savefig(args.output,dpi=185,facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    main()
