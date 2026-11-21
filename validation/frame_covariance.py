#!/usr/bin/env python3
"""Разбор времени кадра через ковариации: отчёт docs/reports/frame-covariance.md и рисунки к нему.

  frame_covariance.py --remote    мерить на свободной машине (build/remote/sync.sh и rrun.sh, см. validation/CLAUDE.md)
  frame_covariance.py             мерить здесь — только для отладки: на рабочей машине время гуляет
  frame_covariance.py --cached    собрать отчёт заново из сырья прошлого замера (build/frame-covariance.json)

Сырьё — приборы study-frame-covariance (время стадий шага по кадрам и счётчики кадра) и study-window-pull
(рывок куба при разных настройках шага). Три разбора:
  стадии — кадр есть сумма стадий, T = Σ s_k, поэтому Var(T) = Σ Cov(s_k, T): доля стадии в разбросе
    времени кадра — Cov(s_k, T) / Var(T). Рядом доля в среднем и доля в самых долгих кадрах;
  цена единицы — наклон b = Cov(s_k, n) / Var(n) времени стадии по счётчику кадра n и r²;
  факторы — полный план 2⁴ по настройкам шага: эффект фактора на отклик y — разность средних на двух
    уровнях, 2 Cov(x, y) при x = ±1; рядом корреляция времени и перекрытия по тем же прогонам.
Отчёт пишется целиком из замера: число в нём руками не править.
"""
import datetime
import itertools
import json
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
REMOTE = ROOT / "build/remote"
REPORT = ROOT / "docs/reports/frame-covariance.md"
FIGURES = ROOT / "docs/reports/figures"
RAW = ROOT / "build/frame-covariance.json"
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
COLORS = ["#2a78d6", "#eb6834", "#3a9d5d", "#b04fc4", "#d6a72a", "#4fb7c4", "#c4504f", "#7a7a7a", "#1f4e8c", "#a8c42a", "#8c5a1f", "#c42a86", "#bdbcb8"]
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "text.color": INK, "axes.labelcolor": MUTED, "axes.edgecolor": GRID, "xtick.color": MUTED,
                     "ytick.color": MUTED, "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE, "axes.titlesize": 10,
                     "axes.titleweight": "bold", "axes.titlelocation": "left"})
NAMES = {"motion": "сдвиг тел", "pairs": "поиск пар", "contacts": "постройка контактов", "warm_start": "тёплый старт", "joints": "сочленения",
         "normals": "нормали", "recheck": "доиск контактов", "friction": "трение", "memory": "память сил", "other": "прочее",
         "separation": "разведение", "impact": "удары", "forces": "силы опоры"}
COUNTERS = {"awake": "тел не спит", "pairs": "пар в касании", "points": "точек контакта", "speed": "наибольшая скорость, м/с"}
# Опыты разбора стадий: подпись, аргументы прибора, окружение, закреплять ли ядро.
EXPERIMENTS = [
    ("рывок куба 1 пирамиды из 36", "pull realtime pyramid 1 2 1 0 0", {}, False),
    ("рывок куба 9 пирамиды из 36", "pull realtime pyramid 9 2 1 0 0", {}, False),
    ("рывок куба 12 пирамиды из 36", "pull realtime pyramid 12 2 1 0 0", {}, False),
    ("стена из 33 кирпичей под ударом шара", "run wall", {}, False),
    ("куча из тридцати тел в ящике", "run pile", {}, False),
    ("чаша с составными телами", "run bowl", {}, False),
    ("пирамида из 1 015 кубов, один поток", "bench 14", {"KP_THREADS": "1"}, True),
    ("пирамида из 1 015 кубов, все потоки", "bench 14", {}, False),
]
# Факторы плана 2⁴ (рывок куба 12): переменная окружения, подпись, нижний и верхний уровень.
FACTORS = [("KP_SUBSTEPS", "подшагов", 4, 8), ("KP_UPDATES", "обновлений контактов", 1, 4), ("KP_ITERATIONS", "шагов перебора на островок", 16, 64),
           ("KP_THREADS", "потоков", 1, None)]
RESPONSES = [("median_ms", "медиана кадра, мс"), ("p99_ms", "99 % кадров, мс"), ("worst_ms", "худший кадр, мс"), ("overlap_mm", "перекрытие, мм")]


def execute(study, arguments, env, remote, pinned=False):
    """Вывод прибора: здесь или на свободной машине (там исполняемые лежат в build\\, как здесь)."""
    env = dict(env, KP_REPEATS="5")
    if not remote:
        command = (["taskset", "-c", "3"] if pinned else []) + [str(ROOT / "build/validation/studies" / study)] + arguments.split()
        return subprocess.run(command, env=dict(__import__("os").environ, **env), capture_output=True, text=True).stdout
    settings = "".join(f'$env:{key}="{value}"; ' for key, value in env.items())
    program = f"build\\validation\\studies\\{study}.exe {arguments}"
    line = f'cmd /c "start /b /wait /affinity 0x100 {program}"' if pinned else f".\\{program}"
    return subprocess.run([str(REMOTE / "rrun.sh"), settings + line], capture_output=True, text=True).stdout


def stageTable(text):
    """Строки прибора → (шапка, времена стадий кадр × стадия, счётчики кадра)."""
    rows = [json.loads(line) for line in text.splitlines() if line.startswith("{")]
    head, frames = rows[0], rows[1:]
    times = np.array([f["ms"] for f in frames])
    used = times.mean(axis=0) > 1e-4 * times.sum(axis=1).mean()  # стадии другого шага и пустые не показываются
    head["stages"] = [name for name, keep in zip(head["stages"], used) if keep]
    return head, times[:, used], {key: np.array([f[key] for f in frames], dtype=float) for key in COUNTERS}


def covariance(a, b):
    return float(np.mean((a - a.mean()) * (b - b.mean())))


def shares(times):
    """По стадиям: среднее, доля среднего, доля дисперсии кадра, корреляция с кадром, доля в 5 % самых долгих кадров."""
    total = times.sum(axis=1)
    slow = total >= np.percentile(total, 95)
    out = []
    for k in range(times.shape[1]):
        stage = times[:, k]
        spread = np.sqrt(covariance(stage, stage) * covariance(total, total))
        out.append({"mean": stage.mean(), "of_mean": stage.mean() / total.mean(), "of_variance": covariance(stage, total) / covariance(total, total),
                    "correlation": covariance(stage, total) / spread if spread > 0 else 0.0, "of_tail": stage[slow].mean() / total[slow].mean()})
    return out


def unitCosts(times, counters, stages):
    """Для стадий заметнее 3 % кадра: счётчик с наибольшей корреляцией, наклон и r² (если r² не меньше 0,5)."""
    out = []
    for k, name in enumerate(stages):
        stage = times[:, k]
        if stage.mean() < 0.03 * times.sum(axis=1).mean() or covariance(stage, stage) == 0:
            continue
        fits = []
        for key, values in counters.items():
            if covariance(values, values) > 0:
                fits.append((covariance(stage, values) ** 2 / (covariance(stage, stage) * covariance(values, values)), key, covariance(stage, values) / covariance(values, values)))
        if fits and max(fits)[0] >= 0.5:  # прямая, объясняющая меньше половины разброса стадии, ценой единицы не называется
            r2, key, slope = max(fits)
            out.append({"stage": name, "counter": key, "slope_us": 1e3 * slope, "base_ms": stage.mean() - slope * counters[key].mean(), "r2": r2})
    return out


def factorial(remote):
    """Полный план 2⁴ на рывке куба 12: строки (уровни ±1, отклики)."""
    runs = []
    for levels in itertools.product((-1, 1), repeat=len(FACTORS)):
        env = {name: str(high if level > 0 else low) for (name, _, low, high), level in zip(FACTORS, levels) if (high if level > 0 else low) is not None}
        text = execute("study-window-pull", "realtime pyramid 12 2 1 0 0", env, remote)
        answer = json.loads(text.strip().splitlines()[-1])
        answer["overlap_mm"] = 1e3 * answer["overlap_m"]
        runs.append((levels, answer))
    return runs


def effects(runs):
    """Эффект фактора на отклик: разность средних на верхнем и нижнем уровне (2 Cov(x, y) при x = ±1) и её доля от среднего."""
    out = {}
    for key, _ in RESPONSES:
        y = np.array([answer[key] for _, answer in runs])
        out[key] = {"mean": y.mean(), "effects": [2 * covariance(np.array([levels[j] for levels, _ in runs], dtype=float), y) for j in range(len(FACTORS))]}
    return out


def drawShares(results):
    fig, axes = plt.subplots(len(results), 1, figsize=(8.6, 1.05 * len(results) + 0.9), sharex=True)
    seen = {}
    for axis, (label, head, table, _) in zip(axes, results):
        for row, key, title in ((1, "of_mean", "среднее"), (0, "of_variance", "разброс")):
            left = 0.0
            for stage, line in zip(head["stages"], table):
                width = 100 * max(line[key], 0)
                seen[stage] = axis.barh(row, width, left=left, color=COLORS[list(NAMES).index(stage) % len(COLORS)], height=0.72)
                left += width
            axis.text(-1, row, title, ha="right", va="center", color=MUTED)
        axis.set_title(label, pad=2)
        axis.set_yticks([]), axis.set_xlim(0, 100), axis.grid(axis="x", color=GRID, linewidth=0.6)
    axes[-1].set_xlabel("доля, %")
    fig.legend([seen[s] for s in seen], [NAMES[s] for s in seen], loc="lower center", ncol=5, frameon=False, bbox_to_anchor=(0.5, -0.02 - 0.05 / len(results)))
    fig.tight_layout(rect=(0.06, 0.05, 1, 1))
    save(fig, "frame-covariance-shares")


def drawTimeline(label, head, times):
    fig, axis = plt.subplots(figsize=(8.6, 3.2))
    order = np.argsort(-times.mean(axis=0))
    axis.stackplot(np.arange(times.shape[0]), *[times[:, k] for k in order], colors=[COLORS[list(NAMES).index(head["stages"][k]) % len(COLORS)] for k in order],
                   labels=[NAMES[head["stages"][k]] for k in order], linewidth=0)
    axis.axhline(1000 / 60, color=INK, linewidth=0.8, linestyle="--")
    axis.text(times.shape[0] - 1, 1000 / 60, "кадр экрана, 16,7 мс ", ha="right", va="top", color=MUTED)
    axis.set_title(f"Время кадра по стадиям: {label}"), axis.set_xlabel("кадр"), axis.set_ylabel("мс")
    axis.set_xlim(0, times.shape[0] - 1), axis.grid(axis="y", color=GRID, linewidth=0.6)
    axis.legend(frameon=False, ncol=5, loc="upper center", bbox_to_anchor=(0.5, -0.22))
    save(fig, "frame-covariance-timeline")


def drawEffects(found):
    fig, axes = plt.subplots(1, len(RESPONSES), figsize=(8.6, 2.5), sharey=True)
    labels = [f"{title}\n{low} → {high if high is not None else 'все'}" for _, title, low, high in FACTORS]
    for axis, (key, title) in zip(axes, RESPONSES):
        values = [100 * e / found[key]["mean"] for e in found[key]["effects"]]
        axis.barh(range(len(FACTORS)), values, color=[COLORS[1] if v > 0 else COLORS[0] for v in values], height=0.7)
        axis.axvline(0, color=INK, linewidth=0.8), axis.set_title(title), axis.grid(axis="x", color=GRID, linewidth=0.6)
        axis.set_yticks(range(len(FACTORS))), axis.set_yticklabels(labels), axis.set_xlabel("эффект, % от среднего")
    fig.tight_layout()
    save(fig, "frame-covariance-factors")


def save(fig, name):
    fig.savefig(FIGURES / f"{name}.svg", bbox_inches="tight")
    if "--png" in sys.argv:  # для проверки рисунка глазами; в отчёт идёт svg
        fig.savefig(ROOT / "build" / f"{name}.png", bbox_inches="tight", dpi=110)
    plt.close(fig)


def number(value, digits=2):
    return f"{value:.{digits}f}".replace(".", ",")


def stageSection(results):
    lines = []
    for label, head, table, costs in results:
        total = sum(line["mean"] for line in table)
        lines += [f"### {label}", "", f"Кадр в среднем {number(total)} мс; потоков {head['threads']}, подшагов {head['substeps']}, повторов {head['repeats']}.", "",
                  "| Стадия | Среднее, мс | Доля среднего | Доля разброса | Корреляция с кадром | Доля в 5 % долгих кадров |", "|---|---|---|---|---|---|"]
        for stage, line in sorted(zip(head["stages"], table), key=lambda pair: -pair[1]["of_variance"]):
            lines.append(f"| {NAMES[stage]} | {number(line['mean'], 3)} | {number(100 * line['of_mean'], 1)} % | {number(100 * line['of_variance'], 1)} % | "
                         f"{number(line['correlation'])} | {number(100 * line['of_tail'], 1)} % |")
        if costs:
            lines += ["", "| Стадия | Счётчик | Цена единицы, мкс | Постоянная часть, мс | r² |", "|---|---|---|---|---|"]
            lines += [f"| {NAMES[c['stage']]} | {COUNTERS[c['counter']]} | {number(c['slope_us'], 1)} | {number(c['base_ms'], 3)} | {number(c['r2'])} |" for c in costs]
        lines.append("")
    return lines


def headline(results):
    """Что видно сразу: кто делает разброс, кто рывки при малой средней доле, кто постоянная цена."""
    lines = []
    for label, head, table, _ in results:
        ranked = sorted(zip(head["stages"], table), key=lambda pair: -pair[1]["of_variance"])
        first, top = ranked[0]
        spiky = [NAMES[s] for s, t in ranked if t["of_variance"] >= 0.10 and t["of_variance"] >= 2 * t["of_mean"]]
        steady = [NAMES[s] for s, t in ranked if t["of_mean"] >= 0.10 and t["of_variance"] <= t["of_mean"] / 4]
        line = f"- **{label}:** разброс делает стадия «{NAMES[first]}» — {number(100 * top['of_variance'], 0)} % дисперсии при {number(100 * top['of_mean'], 0)} % среднего"
        line += f"; рывки при малой средней доле — {', '.join(spiky)}" if spiky else ""
        line += f"; постоянная цена — {', '.join(steady)}" if steady else ""
        lines.append(line + ".")
    return lines


def threadSection(results):
    """Ускорение стадий от потоков: опыты «один поток» и «все потоки» одной сцены."""
    single = next((r for r in results if r[0].endswith("один поток")), None)
    many = next((r for r in results if r[0].endswith("все потоки")), None)
    if not single or not many:
        return []
    one = dict(zip(single[1]["stages"], single[2]))
    lines = [f"## Потоки: что ускоряется", "", f"{single[0].rsplit(',', 1)[0]}: один поток против {many[1]['threads']}.", "",
             "| Стадия | Один поток, мс | Все потоки, мс | Ускорение |", "|---|---|---|---|"]
    for stage, line in sorted(zip(many[1]["stages"], many[2]), key=lambda pair: -pair[1]["mean"]):
        if stage in one and one[stage]["mean"] > 0.2:
            lines.append(f"| {NAMES[stage]} | {number(one[stage]['mean'])} | {number(line['mean'])} | в {number(one[stage]['mean'] / line['mean'])} раза |")
    whole = sum(t["mean"] for t in single[2]), sum(t["mean"] for t in many[2])
    lines += [f"| **кадр** | {number(whole[0])} | {number(whole[1])} | в {number(whole[0] / whole[1])} раза |", ""]
    return lines


def factorSection(runs, found):
    time = np.array([answer["p99_ms"] for _, answer in runs])
    overlap = np.array([answer["overlap_mm"] for _, answer in runs])
    r = covariance(time, overlap) / np.sqrt(covariance(time, time) * covariance(overlap, overlap))
    lines = ["| Фактор | Уровни | " + " | ".join(title for _, title in RESPONSES) + " |", "|---|---|" + "---|" * len(RESPONSES)]
    for j, (_, title, low, high) in enumerate(FACTORS):
        cells = [f"{'+' if found[key]['effects'][j] > 0 else '−'}{number(abs(100 * found[key]['effects'][j] / found[key]['mean']), 0)} %" for key, _ in RESPONSES]
        lines.append(f"| {title} | {low} → {high if high is not None else 'все'} | " + " | ".join(cells) + " |")
    lines.append("| среднее по плану | | " + " | ".join(number(found[key]["mean"], 3 if key == "overlap_mm" else 2) for key, _ in RESPONSES) + " |")
    lines += ["", f"Корреляция времени (99 % кадров) и перекрытия по шестнадцати прогонам плана: r = {number(r)}.", "", "Все прогоны плана:", "",
              "| " + " | ".join(title for _, title, _, _ in FACTORS) + " | " + " | ".join(title for _, title in RESPONSES) + " |", "|" + "---|" * (len(FACTORS) + len(RESPONSES))]
    for levels, answer in runs:
        setting = [str((high if level > 0 else low) if (high if level > 0 else low) is not None else "все") for (_, _, low, high), level in zip(FACTORS, levels)]
        lines.append("| " + " | ".join(setting) + " | " + " | ".join(number(answer[key], 3 if key == "overlap_mm" else 2) for key, _ in RESPONSES) + " |")
    return lines


def measure(remote):
    """Сырьё замера: вывод прибора стадий по опытам и прогоны плана факторов."""
    if remote and subprocess.run([str(REMOTE / "sync.sh")]).returncode != 0:
        sys.exit("сборка на свободной машине не прошла")
    stage = [execute("study-frame-covariance", arguments, env, remote, pinned) for _, arguments, env, pinned in EXPERIMENTS]
    return {"remote": remote, "date": f"{datetime.date.today():%d.%m.%Y}", "stage": stage, "runs": factorial(remote)}


def main():
    raw = json.loads(RAW.read_text(encoding="utf-8")) if "--cached" in sys.argv else measure("--remote" in sys.argv)
    RAW.write_text(json.dumps(raw, ensure_ascii=False), encoding="utf-8")
    remote, runs, results = raw["remote"], [(tuple(levels), answer) for levels, answer in raw["runs"]], []
    for (label, arguments, _, _), text in zip(EXPERIMENTS, raw["stage"]):
        head, times, counters = stageTable(text)
        results.append((label, head, shares(times), unitCosts(times, counters, head["stages"])))
        if arguments.startswith("pull realtime pyramid 12"):
            drawTimeline(label, head, times)
    found = effects(runs)
    drawShares(results), drawEffects(found)
    where = "свободная машина (Windows, clang, закреплённое ядро у опытов одного потока)" if remote else "рабочая машина — числа для отладки, не для выводов"
    text = ["# Время кадра через ковариации", "", f"Сгенерировано `validation/frame_covariance.py{' --remote' if remote else ''}` {raw['date']}; руками не править.",
            f"Где мерили: {where}. Время каждой стадии каждого кадра — наименьшее из пяти прогонов.", "", "## Главное", ""]
    text += headline(results)
    text += ["", "## Что считается", "", "Шаг сообщает наблюдателю о стадиях, и время кадра есть сумма времён стадий: $T = \\sum_k s_k$. Поэтому разброс времени кадра",
            "раскладывается по стадиям без остатка:", "", "$$ \\operatorname{Var}(T) = \\sum_k \\operatorname{Cov}(s_k, T), \\qquad d_k = \\frac{\\operatorname{Cov}(s_k, T)}{\\operatorname{Var}(T)}, \\qquad \\sum_k d_k = 1. $$", "",
            "Доля среднего $m_k = \\bar s_k / \\bar T$ говорит, куда уходит время; доля разброса $d_k$ — из-за чего кадры бывают долгими. Стадия с $d_k \\gg m_k$",
            "делает рывки; стадия с $d_k \\ll m_k$ — постоянная цена. Цена единицы работы — наклон прямой $s_k \\approx a + b\\,n$ по счётчику кадра $n$:",
            "$b = \\operatorname{Cov}(s_k, n) / \\operatorname{Var}(n)$; показана, только если прямая объясняет не меньше половины разброса стадии ($r^2 \\ge 0{,}5$).",
            "Эффект настройки — разность средних отклика на двух её уровнях в полном плане $2^4$: $2 \\operatorname{Cov}(x, y)$ при $x = \\pm 1$.", "",
            "![Доли стадий в среднем времени кадра и в его разбросе](figures/frame-covariance-shares.svg)", "",
            "![Время кадра по стадиям при рывке куба](figures/frame-covariance-timeline.svg)", "", "## Стадии по опытам", ""]
    text += stageSection(results) + threadSection(results)
    text += ["## Настройки шага: время и точность вместе", "", "Рывок куба 12 пирамиды из 36 (`study-window-pull`), полный план $2^4$. Эффект — на сколько процентов от среднего по плану",
             "меняется отклик при переходе фактора с нижнего уровня на верхний.", "", "![Эффекты настроек шага](figures/frame-covariance-factors.svg)", ""]
    text += factorSection(runs, found)
    REPORT.write_text("\n".join(text) + "\n", encoding="utf-8")
    print(f"отчёт: {REPORT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
