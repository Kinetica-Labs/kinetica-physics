#!/usr/bin/env python3
"""Ворота правки: одна команда и один итог — ХУЖЕ, НЕ ХУЖЕ или ЛУЧШЕ.

  gate.py                 все проверки рядом с эталоном; код выхода 1, если хоть одна строка хуже
  gate.py --remote        скорость меряется на свободной машине (см. ниже), а не здесь
  gate.py --quick         без второго компилятора, таблицы случаев, разброса стены и сцен сравнения:
                          для промежуточных шагов правки, не для коммита
  gate.py --update        записать нынешние числа эталоном (те разделы, что измерены этим запуском);
                          отказ, если не прошли проверки «да/нет»

Эталон — docs/reports/data/gate-baseline.json; его изменение видно в diff коммита.

Проверки «да/нет»: сборка, ctest, хэш состояния против эталона (1, 4 и 12 потоков, затирание памяти
шага, clang++), стандарт кода, таблица случаев; с --remote — ещё сборка и хэш на свободной машине
(другая система, компилятор и процессор). Мерки с эталоном (порог записан у каждой):
  стандартные сцены быстрым шагом — итог сцены, перекрытие и прирост энергии: сцена, прошедшая в эталоне,
    обязана пройти; порог на перекрытие и энергию грубый, от порчи на порядок (сцены с ударами хаотичны:
    от последнего бита перекрытие гуляет в полтора раза, а прирост энергии вёдер — от 5e-5 до 7e-3);
  разброс перекрытия стены под ударом — медиана по 16 вариантам;
  рывок куба, как в окне (study-window-pull) — худший кадр, 99-й процентиль, скорость тел, перекрытие;
  скорость — пирамида из 1 015 кубов одним потоком и всеми, сцены сравнения одним потоком.

Где мерить скорость. На рабочей машине рядом идут другие программы: тот же код давал +6…9 % времени
кадра и ±4 % тактов процессора (2026-11-21), поэтому здесь скорость меряется тактами (perf) с порогом
6 % и годится только против грубой порчи. Точный замер — --remote: дерево передаётся на свободную
машину, собирается там, и приборы идут на ядре, закреплённом за ними; наименьшее из десяти прогонов
повторяется там до 0,1 %, порог 3 %. Связь с машиной — два сценария вне git (в них адрес и пароль):
build/remote/sync.sh — передать дерево и собрать, код выхода сборки; build/remote/rrun.sh "<команда>" —
выполнить команду PowerShell в каталоге проекта той машины (исполняемые — в build\\, как здесь).
Время и такты берутся наименьшими из повторов: помеха их только прибавляет; «лучше» объявляется только
за порогом в другую сторону.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"
STUDIES = BUILD / "validation/studies"
ENGINES = BUILD / "validation/engines"
BASELINE = ROOT / "docs/reports/data/gate-baseline.json"
HASH = ROOT / "docs/reports/data/state-hash.txt"
REMOTE = BUILD / "remote"
PULLS = [1, 9, 12]  # кубы пирамиды из 36: нижний ряд, середина, верх
SCENES = ["stack10", "pyramid55", "rubble36", "overlap", "gap1mm", "cube100", "column100", "pendulum120", "beam6", "chain100"]


def run(command, env=None, timeout=1800):
    full = dict(os.environ, **(env or {}))
    done = subprocess.run([str(c) for c in command], cwd=ROOT, env=full, capture_output=True, text=True, timeout=timeout)
    return done.returncode, done.stdout + done.stderr


def passFail(quick):
    """Проверки без чисел: имя → пройдена ли."""
    hashed = lambda env, study=STUDIES: run([study / "study-state-hash", 60], env)[1] == HASH.read_text(encoding="utf-8")
    checks = {
        "сборка": run(["cmake", "--build", BUILD, "-j"])[0] == 0,
        "ctest": run(["ctest", "--test-dir", BUILD])[0] == 0,
        "стандарт кода": run([sys.executable, ROOT / "validation/code_metrics.py", "--check"])[0] == 0,
    }
    for threads in (1, 4, 12):
        checks[f"хэш, потоков {threads}"] = hashed({"KP_THREADS": str(threads)})
    checks["хэш, затирание памяти шага"] = hashed({"KP_STEP_MEMORY": "check"})
    if not quick:
        clang = BUILD / "clang"
        built = clang.is_dir() and run(["cmake", "--build", clang, "-j"])[0] == 0
        checks["хэш, clang++ (build/clang)"] = built and hashed({}, clang / "validation/studies")
        checks["таблица случаев"] = run([sys.executable, ROOT / "validation/case_table.py", "--check"])[0] == 0
    return checks


def standardScenes():
    with tempfile.TemporaryDirectory() as folder:
        rows = Path(folder) / "rows.jsonl"
        run([STUDIES / "study-standard-scenes", rows], {"KP_REALTIME": "1"})
        found = [json.loads(line) for line in rows.read_text(encoding="utf-8").splitlines() if line.strip()]
    return {r["scene"]: {"ok": r["ok"], "overlap_m": r["overlap_m"], "energy_gain": r["energy_gain"]} for r in found}


class Here:
    """Замеры скорости на этой машине: такты пользователя на закреплённом ядре."""
    section, unit, margin = "local", "млрд тактов", 0.06

    def pull(self, body):
        return run([STUDIES / "study-window-pull", "realtime", "pyramid", body, 2, 1, 0, 0])[1]

    def single(self, env, repeats):
        best = float("inf")
        for _ in range(repeats):
            text = run(["taskset", "-c", "3", "perf", "stat", "-x,", "-e", "cycles:u", ENGINES / "compare-ours", "realtime", 8], dict(env, KP_THREADS="1"))[1]
            best = min(best, float(re.search(r"^(\d+),,cycles:u", text, re.M).group(1)) / 1e9)
        return best

    def allThreads(self):
        return run([ENGINES / "compare-ours", "realtime", 8], {"BENCH": "14"})[1]


class There:
    """Замеры скорости на свободной машине: время кадра на закреплённом ядре."""
    section, unit, margin = "remote", "мс", 0.03

    def command(self, env, line):
        settings = "".join(f'$env:{key}="{value}"; ' for key, value in env.items())
        return run([REMOTE / "rrun.sh", settings + line])[1]

    def pull(self, body):
        return self.command({"KP_REPEATS": "5"}, f".\\build\\validation\\studies\\study-window-pull.exe realtime pyramid {body} 2 1 0 0")

    def single(self, env, repeats):
        pinned = 'cmd /c "start /b /wait /affinity 0x100 build\\validation\\engines\\compare-ours.exe realtime 8"'
        text = self.command(dict(env, KP_THREADS="1"), f"1..{repeats} | ForEach-Object {{ {pinned} }}")
        frames = re.findall(r"([\d.]+) мс на кадр", text) or [m for line in text.splitlines() if line.startswith("{") for m in json.loads(line)["numbers"]["ms_frame"]]
        return min(float(f) for f in frames)

    def allThreads(self):
        return self.command({"BENCH": "14"}, "1..2 | ForEach-Object { .\\build\\validation\\engines\\compare-ours.exe realtime 8 }")


def speed(machine, quick):
    """Мерки скорости на машине machine."""
    numbers = {"pull": {f"pyramid {body}": json.loads(machine.pull(body).strip().splitlines()[-1]) for body in PULLS}}
    numbers["bench_single"] = machine.single({"BENCH": "14"}, 10)
    numbers["bench_all_ms"] = min(float(f) for _ in range(3) for f in re.findall(r"([\d.]+) мс на кадр", machine.allThreads()))
    if not quick:
        numbers["scene"] = {key: machine.single({"SCENE": key}, 5) for key in SCENES}
    return numbers


def confirm(fast, old, machine):
    """Мерку скорости, вышедшую за порог, перемерить: помеха время только прибавляет, поэтому второй замер
    либо подтверждает замедление, либо снимает его (на свободной машине разовый выброс — до 3,5 %)."""
    slow = lambda value, was: value > (1 + machine.margin) * was
    for _ in range(2):
        if slow(fast["bench_single"], old["bench_single"]):
            fast["bench_single"] = min(fast["bench_single"], machine.single({"BENCH": "14"}, 10))
        for key in fast.get("scene", {}):
            if slow(fast["scene"][key], old["scene"][key]):
                fast["scene"][key] = min(fast["scene"][key], machine.single({"SCENE": key}, 5))
        for key, pull in fast["pull"].items():
            if pull["worst_ms"] > 1.25 * old["pull"][key]["worst_ms"] or pull["p99_ms"] > 1.25 * old["pull"][key]["p99_ms"]:
                again = json.loads(machine.pull(int(key.split()[1])).strip().splitlines()[-1])
                pull["worst_ms"], pull["p99_ms"] = min(pull["worst_ms"], again["worst_ms"]), min(pull["p99_ms"], again["p99_ms"])


def measure(quick, machine):
    numbers = {"standard": standardScenes(), machine.section: speed(machine, quick)}
    if not quick:
        spread = run([STUDIES / "study-overlap-spread", "wall"])[1]
        numbers["wall_overlap_median_mm"] = float(re.search(r"медиана ([\d.]+) мм", spread).group(1))
    return numbers


def judge(now, was, worse, better=None):
    """Строка мерки: хуже — за порогом worse(was), лучше — за порогом better(was)."""
    if now > worse(was):
        return "ХУЖЕ"
    return "лучше" if better and now < better(was) else "ок"


def compare(now, base, machine):
    """Строки таблицы: (мерка, эталон, сейчас, порог, итог, единица)."""
    rows = []
    add = lambda name, was, value, worse, better=None, unit="": rows.append((name, was, value, worse(was), judge(value, was, worse, better), unit))
    for key, scene in now["standard"].items():
        was = base["standard"].get(key)
        if was is None:
            continue
        rows.append((f"сцена {key}: итог", "прошла" if was["ok"] else "нет", "прошла" if scene["ok"] else "нет", "", "ХУЖЕ" if was["ok"] and not scene["ok"] else "ок", ""))
        add(f"сцена {key}: перекрытие", 1e3 * was["overlap_m"], 1e3 * scene["overlap_m"], lambda w: 2 * w + 0.05, unit="мм")
        add(f"сцена {key}: энергия", was["energy_gain"], scene["energy_gain"], lambda w: 10 * w + 1e-2)
    if "wall_overlap_median_mm" in now:
        add("стена под ударом: перекрытие, медиана", base["wall_overlap_median_mm"], now["wall_overlap_median_mm"], lambda w: 1.5 * w, unit="мм")
    fast, old = now[machine.section], base[machine.section]
    slower, faster = (lambda w: (1 + machine.margin) * w), (lambda w: (1 - machine.margin) * w)
    for key, pull in fast["pull"].items():
        was = old["pull"][key]
        add(f"рывок {key}: худший кадр", was["worst_ms"], pull["worst_ms"], lambda w: 1.25 * w, lambda w: 0.8 * w, "мс")
        add(f"рывок {key}: 99 % кадров", was["p99_ms"], pull["p99_ms"], lambda w: 1.25 * w, lambda w: 0.8 * w, "мс")
        add(f"рывок {key}: скорость тел", was["peak_speed"], pull["peak_speed"], lambda w: 20.0, unit="м/с")
        add(f"рывок {key}: перекрытие", 1e3 * was["overlap_m"], 1e3 * pull["overlap_m"], lambda w: 2 * w + 0.05, unit="мм")
    add("пирамида 1 015, 1 поток", old["bench_single"], fast["bench_single"], slower, faster, machine.unit)
    add("пирамида 1 015, все потоки", old["bench_all_ms"], fast["bench_all_ms"], lambda w: 1.2 * w, lambda w: 0.8 * w, "мс")
    for key, value in fast.get("scene", {}).items():
        add(f"сцена сравнения {key}", old["scene"][key], value, slower, faster, machine.unit)
    return rows


def show(value):
    return value if isinstance(value, str) else f"{value:.4g}"


def main():
    quick, update, remote = "--quick" in sys.argv, "--update" in sys.argv, "--remote" in sys.argv
    machine = There() if remote else Here()
    checks = passFail(quick)
    if remote:
        checks["сборка на свободной машине"] = run([REMOTE / "sync.sh"])[0] == 0
        there = run([REMOTE / "rrun.sh", ".\\build\\validation\\studies\\study-state-hash.exe 60"])[1]
        checks["хэш на свободной машине"] = there.split() == HASH.read_text(encoding="utf-8").split()
    for name, ok in checks.items():
        print(f"{name:46} {'ок' if ok else 'ОТКАЗ'}")
    failed = [name for name, ok in checks.items() if not ok]
    if failed and (update or not checks["сборка"] or not checks.get("сборка на свободной машине", True)):
        sys.exit(f"ИТОГ: ХУЖЕ — не прошли: {', '.join(failed)}")
    now = measure(quick, machine)
    base = json.loads(BASELINE.read_text(encoding="utf-8")) if BASELINE.exists() else {}
    if update:
        BASELINE.write_text(json.dumps(dict(base, **now), ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        return print(f"эталон записан: {BASELINE.relative_to(ROOT)}")
    confirm(now[machine.section], base[machine.section], machine)
    rows = compare(now, base, machine)
    print(f"\n{'мерка':46} {'эталон':>10} {'сейчас':>10} {'порог':>10}  итог")
    for name, was, value, limit, verdict, unit in rows:
        if verdict != "ок" or not name.startswith("сцена ") or name.startswith("сцена сравнения"):
            print(f"{name:46} {show(was):>10} {show(value):>10} {show(limit):>10}  {verdict} {unit}")
    worse = failed + [r[0] for r in rows if r[4] == "ХУЖЕ"]
    better = [r[0] for r in rows if r[4] == "лучше"]
    print(f"стандартных сцен быстрым шагом прошло: {sum(s['ok'] for s in now['standard'].values())} из {len(now['standard'])}")
    if worse:
        sys.exit(f"ИТОГ: ХУЖЕ — {'; '.join(worse)}")
    print(f"ИТОГ: ЛУЧШЕ — {'; '.join(better)}" if better else "ИТОГ: НЕ ХУЖЕ")


if __name__ == "__main__":
    main()
