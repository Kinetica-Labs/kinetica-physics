#!/usr/bin/env python3
"""Табло: прогон сравнения движков, журнал results.jsonl и отчёт.

  run.py run      собрать compare-ours, прогнать быстрый шаг при 8 и 4 подшагах, дописать журнал
  run.py box3d    собрать Box3D из research/refs/box3d, прогнать при 4 и 8 подшагах со своими жёсткостями
                  и при 8 — с нашими, дописать журнал
  run.py report   пересобрать docs/reports/engine-comparison.md из журнала

Журнал только дописывается: в отчёт идёт последняя строка каждой пары (движок, сцена). Строки
Box3D остаются в журнале планкой, пока их не обновит следующий `run.py box3d`.

Сравнение — парами по номеру варианта сцены (validation/engines/common.h): в каждой паре
выигрыш, проигрыш или ничья (разница не больше 1 % и не больше 1e-6). Вердикт — критерий знаков
по парам без ничьих: «лучше» или «хуже», если вероятность такого счёта при равных движках
не больше 5 %, иначе «ничья».
"""
import datetime
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD, JOURNAL = ROOT / "build", ROOT / "validation/results.jsonl"
REPORT = ROOT / "docs/reports/engine-comparison.md"
BOX3D_SOURCE = ROOT / "research/refs/box3d"
OURS = [["realtime", "8"], ["realtime", "4"], ["realtime", "8", "sleep"]] # со сном — как в окне: пробуждения
# Наши жёсткости контакта и сочленений, Гц (SoftStep): при 4 подшагах предел 0,125/h и 0,25/h делает
# их равными умолчанию Box3D (30 и 60), при 8 — у нас вдвое жёстче.
BOX3D = [["4"], ["8"], ["8", "60", "120"]]
# Столбцы итога: заголовок, наш прогон, прогон Box3D.
PAIRS = [("4 подшага", "kinetica realtime 4", "box3d 4"), ("8 подшагов", "kinetica realtime 8", "box3d 8"),
         ("8 подшагов, у Box3D наши 60/120 Гц", "kinetica realtime 8", "box3d 8 60/120 Гц")]
# Что сравнивается в сцене: меньше — лучше. У энергии — модуль: маятник обязан её сохранять.
# Дрожь (common.h, Jitter): jitter_mm_s — медиана по кадрам после успокоения наибольшей скорости
# точки тела, reversals — развороты скорости тел; у осыпи сравнивается только она и покой в конце.
REST = ["drift_mm", "sink_mm", "speed_mm_s", "jitter_mm_s", "reversals"]
# Тело в теле: выталкивание без вылета — перелёт выше итогового положения и пик скорости.
# Высокий столбик: перекрытие после успокоения (overlap_mm, глубина по разделяющим осям) и вынос вбок.
JUDGED = {"stack10": REST, "pyramid55": REST, "rubble36": ["jitter_mm_s", "reversals", "speed_mm_s"],
          "overlap": ["overshoot_mm", "peak_mm_s", "speed_mm_s"], "column100": ["overlap_mm", "overlap_peak_mm"] + REST,
          "cube100": REST, "pendulum120": ["angle_err_rad", "energy_J", "gap_mm"],
          "beam6": ["sink_mm", "gap_mm"], "chain100": ["gap_mm"]}


def sh(*args, cwd=ROOT):
    return subprocess.run([str(a) for a in args], cwd=cwd, check=True, capture_output=True, text=True).stdout


def append(lines, source):
    meta = {"source": source, "date": datetime.datetime.now().isoformat(timespec="seconds")}
    with JOURNAL.open("a") as journal:
        for line in lines.splitlines():
            journal.write(json.dumps({**json.loads(line), **meta}, ensure_ascii=False) + "\n")


def runOurs():
    sh("cmake", "--build", BUILD, "--target", "compare-ours", "-j12")
    commit = sh("git", "rev-parse", "--short", "HEAD").strip()
    dirty = "+правки" if sh("git", "status", "--porcelain", "src", "include") else ""
    for args in OURS:
        append(sh(BUILD / "validation/engines/compare-ours", *args), f"kinetica@{commit}{dirty}")


def runBox3d():
    out = BUILD / "box3d"
    sh("cmake", "-S", BOX3D_SOURCE, "-B", out, "-DCMAKE_BUILD_TYPE=Release", "-DBOX3D_SAMPLES=OFF",
       "-DBOX3D_UNIT_TESTS=OFF", "-DBOX3D_BENCHMARKS=OFF")
    sh("cmake", "--build", out, "--target", "box3d", "-j12")
    sh("g++", "-O3", "-DNDEBUG", "-std=c++17", f"-I{BOX3D_SOURCE}/include", "validation/engines/box3d.cpp",
       out / "src/libbox3d.a", "-lpthread", "-o", out / "compare-box3d")
    commit = sh("git", "rev-parse", "--short", "HEAD", cwd=BOX3D_SOURCE).strip()
    for args in BOX3D:
        append(sh(out / "compare-box3d", *args), f"box3d@{commit}")


def latest():
    rows = {}
    for line in JOURNAL.read_text().splitlines():
        r = json.loads(line)
        rows[(r["engine"], r["scene"])] = r
    return rows


def signTest(ours, theirs, metric):
    """(побед, ничьих, поражений, вердикт) по парам вариантов."""
    size = abs if metric == "energy_J" else (lambda v: v)
    score = [0, 0, 0]
    for a, b in zip(map(size, ours), map(size, theirs)):
        score[0 if a < b and b - a > max(1e-6, 0.01 * b) else 2 if a > b and a - b > max(1e-6, 0.01 * a) else 1] += 1
    wins, ties, losses = score
    n = wins + losses
    tail = lambda k: sum(math.comb(n, i) for i in range(k, n + 1)) / 2 ** n
    verdict = "**лучше**" if n and tail(wins) <= 0.05 else "**хуже**" if n and tail(losses) <= 0.05 else "ничья"
    return wins, ties, losses, verdict


def number(v):
    return f"{v:.3g}" if abs(v) < 1000 else f"{v:.0f}"


def verdicts(rows, scenes):
    out = ["## Итог: наш быстрый шаг против Box3D при равном числе подшагов", "",
           "Одинаковы: проходов на подшаг (с поправкой и без), тёплый старт, затухание контакта 10, выталкивание "
           "не быстрее 3 м/с, трение 0,5, отскок 0, сон выключен. Различаются: точность чисел (у Box3D float), "
           "решение сочленений (у нас запертые свободы — одной системой) и при 8 подшагах жёсткости — "
           "поэтому третий столбец.", "",
           "Ячейка — медиана по вариантам у нас / у Box3D; счёт — побед : ничьих : поражений; вердикт — критерий знаков.", "",
           "| Сцена | Число | " + " | ".join(title for title, _, _ in PAIRS) + " |",
           "|---|---|" + "---|" * len(PAIRS)]
    for scene, title in scenes:
        for metric in JUDGED.get(scene, []): # сцена без пары с Box3D (gap1mm — только сон) в итог не идёт
            cells = []
            for _, mine, other in PAIRS:
                ours, theirs = rows.get((mine, scene)), rows.get((other, scene))
                if not ours or not theirs or metric not in ours["numbers"] or metric not in theirs["numbers"]:
                    cells.append("—") # журнал дописывается: в старых строках новых чисел нет
                    continue
                a, b = ours["numbers"][metric], theirs["numbers"][metric]
                w, t, l, v = signTest(a, b, metric)
                cells.append(f"{number(statistics.median(a))} / {number(statistics.median(b))} — {w}:{t}:{l} {v}")
            out.append(f"| {title} | `{metric}` | " + " | ".join(cells) + " |")
    return out


def details(rows, scenes):
    out = ["", "## Все числа", "", "Медиана и худшее значение по вариантам; `ms_frame` — время кадра, `done_s` — сколько секунд опыта посчитано."]
    for scene, title in scenes:
        engines = sorted(e for e, s in rows if s == scene)
        names = list(dict.fromkeys(n for e in engines for n in rows[(e, scene)]["numbers"]))
        out += ["", f"### {title}", "", "| Движок | " + " | ".join(f"`{n}`" for n in names) + " | хэш |", "|---|" + "---:|" * len(names) + "---|"]
        for e in engines:
            r = rows[(e, scene)]
            cell = lambda v: f"{number(statistics.median(v))} [{number(max(v))}]" if v else "—"
            out.append(f"| {e} | " + " | ".join(cell(r["numbers"].get(n)) for n in names)
                       + f" | `{r['hash'][:8]}` |")
    return out


def report():
    rows = latest()
    scenes = list(dict.fromkeys((s, r["title"]) for (e, s), r in rows.items()))
    sources = sorted({f"{r['engine']}: `{r['source']}`, {r['date']}" for r in rows.values()})
    out = ["# Сравнение с Box3D", "",
           "Сгенерировано `validation/run.py report` из `validation/results.jsonl` — руками не править. "
           "Сцены и варианты — `validation/engines/common.h`, правила сравнения — в `validation/run.py`.", "",
           f"Вариантов на сцену: {next(iter(rows.values()))['variants']}. Прогоны: " + "; ".join(sources) + ".", ""]
    out += verdicts(rows, scenes) + details(rows, scenes)
    REPORT.write_text("\n".join(out) + "\n")
    return REPORT


if __name__ == "__main__":
    command = sys.argv[1] if len(sys.argv) > 1 else ""
    if command == "run":
        runOurs()
    elif command == "box3d":
        runBox3d()
    elif command != "report":
        sys.exit(__doc__)
    print(report())
