#!/usr/bin/env python3
"""Абляция быстрого шага: приёмы выключаются по одному, меряются кадр и качество.

  ablation.py prepare   копия дерева с переключателями → свободная машина, сборка build-ablation
  ablation.py run       замер → docs/reports/data/ablation.json (около получаса; машина должна быть свободна)
  ablation.py run ключ… перемерить только названные приёмы (ключи — в VARIANTS)

Переключатели — переменные окружения KP_OFF_* и KP_SET_*; они есть только в копии (build/ablation-src), в
библиотеку не попадают: вписываются заменами строк из SWITCHES. Замена, не нашедшая своей строки, — ошибка:
значит, код ушёл, и переключатель надо обновить. Без переключателей копия обязана давать эталонный хэш.

Что меряется у каждого приёма (он выключен, остальные включены):
  — пирамида из 1 015 кубов одним потоком на закреплённом ядре и всеми потоками: три пары прогонов
    «включён / выключен» попеременно, берётся наименьшее;
  — стандартные сцены быстрым шагом: сумма мс на кадр, сколько прошло, наибольшее перекрытие;
  — в скольких сценах изменился хэш состояния: меняет ли приём ответ.
"""
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REMOTE, COPY, DATA = ROOT / "build/remote", ROOT / "build/ablation-src", ROOT / "docs/reports/data/ablation.json"
OFF = '#include <cstdlib>\nnamespace { bool off(const char* name) { return std::getenv(name) != nullptr; } }\n'
WORLD = "src/world/World.cpp"
CONTACT = "src/dynamics/time_stepping/SoftContact.cpp"

# Приём: (ключ, переменные окружения, которые его выключают, название).
VARIANTS = [
    ("prefetch", {"KP_OFF_PREFETCH": "1"}, "упреждающая подкачка памяти контактов"),
    ("stack", {"KP_OFF_STACK": "1"}, "память задачи площадки на стеке (H116)"),
    ("candidates", {"KP_OFF_CANDIDATES": "1"}, "память кандидатов в пары (H102)"),
    ("manifolds", {"KP_OFF_MANIFOLDS": "1"}, "память площадок: площадка идёт за телами"),
    ("quiet", {"KP_OFF_QUIET": "1"}, "тихий кадр: один поиск контактов в покое (H69)"),
    ("restskip", {"KP_OFF_RESTSKIP": "1"}, "второй проход нормалей — только движущимся"),
    ("levels", {"KP_OFF_LEVELS": "1"}, "проходы решателя всеми потоками по уровням"),
    ("setfactor", {"KP_OFF_SETFACTOR": "1"}, "разложение островка идёт за набором (H88)"),
    ("fourpoints", {"KP_OFF_FOURPOINTS": "1"}, "четыре точки на пару вместо всех"),
    ("recheck", {"KP_OFF_RECHECK": "1"}, "доиск пропущенных контактов после сдвига"),
    ("warmstart", {"KP_OFF_WARMSTART": "1"}, "тёплый старт контактов"),
    ("direct", {"KP_SET_DIRECT_ROWS": "0"}, "жёсткая задача островка (иначе всё на пружинах)"),
    ("updates", {"KP_SET_UPDATES": "1"}, "четыре поиска контактов за кадр (иначе один)"),
    ("substeps", {"KP_SET_SUBSTEPS": "4"}, "восемь подшагов (иначе четыре)"),
    ("directjoints", {"KP_OFF_DIRECTJOINTS": "1"}, "сочленения одной системой"),
]

# Замены строк в копии: (файл, было, стало).
SWITCHES = [
    (WORLD, "#include <algorithm>\n", "#include <algorithm>\n" + OFF),
    (WORLD, "        SoftStep part = soft;\n", """        SoftStep part = soft;
        static const bool offQuiet = off("KP_OFF_QUIET"), offRestSkip = off("KP_OFF_RESTSKIP"), offManifolds = off("KP_OFF_MANIFOLDS"), offCandidates = off("KP_OFF_CANDIDATES");
        if (const char* v = std::getenv("KP_SET_UPDATES")) part.contactUpdates = std::atoi(v);
        if (const char* v = std::getenv("KP_SET_SUBSTEPS")) part.substeps = std::atoi(v);
        if (const char* v = std::getenv("KP_SET_DIRECT_ROWS")) part.directContactRows = std::atoi(v);
        if (off("KP_OFF_FOURPOINTS")) part.fourContactPoints = false;
        if (off("KP_OFF_RECHECK")) part.recheckDepth = Real(1e9);
        if (off("KP_OFF_DIRECTJOINTS")) part.directJoints = false;
"""),
    (WORLD, "        const int updates = quiet(dt) ? 1 : soft.contactUpdates;", "        const int updates = !offQuiet && quiet(dt) ? 1 : part.contactUpdates;"),
    (WORLD, "        part.restSpeed = soft.contactDrift / dt;", "        part.restSpeed = offRestSkip ? Real(0) : soft.contactDrift / dt;"),
    (WORLD, "        part.substeps = std::max(1, soft.substeps / updates);", "        part.substeps = std::max(1, part.substeps / updates);"),
    (WORLD, "finder.memory = &manifolds_, finder.remembered = &candidates_;", "finder.memory = offManifolds ? nullptr : &manifolds_, finder.remembered = offCandidates ? nullptr : &candidates_;"),
    (CONTACT, "#include <cmath>\n", "#include <cmath>\n" + OFF),
    (CONTACT, "    constexpr size_t kLine = 64;\n", '    static const bool offPrefetch = off("KP_OFF_PREFETCH");\n    if (offPrefetch) return;\n    constexpr size_t kLine = 64;\n'),
    (CONTACT, "    SmallBlock small;\n    if (count > kSmallBlock)", '    SmallBlock small;\n    static const bool offStack = off("KP_OFF_STACK");\n    if (count > kSmallBlock || offStack)'),
    (CONTACT, "    const BlockWork w = count > kSmallBlock ?", "    const BlockWork w = count > kSmallBlock || offStack ?"),
    (CONTACT, "    solveComplementarity(w.matrix, w.q, w.unscaled, count);",
     "    if (offStack) {\n        BlockMemory heap = heapBlock(count);\n        solveComplementarity(w.matrix, w.q, w.unscaled, count, heap);\n    } else solveComplementarity(w.matrix, w.q, w.unscaled, count);"),
    ("include/math/Complementarity.h", "inline void solveComplementarity(const std::vector<Real>& A, const std::vector<Real>& q, std::vector<Real>& lambda) {",
     """inline BlockMemory heapBlock(size_t n) { // память задачи из массивов потока — как было до H116
    thread_local std::vector<Real> L, rhs;
    thread_local std::vector<size_t> rows;
    thread_local std::vector<char> active;
    L.resize(n * n), rhs.resize(n), rows.resize(n), active.resize(n);
    return {L.data(), rhs.data(), rows.data(), active.data()};
}

inline void solveComplementarity(const std::vector<Real>& A, const std::vector<Real>& q, std::vector<Real>& lambda) {"""),
    ("src/dynamics/time_stepping/SoftStep.cpp", "#include <limits>\n", "#include <limits>\n" + OFF),
    ("src/dynamics/time_stepping/SoftStep.cpp",
     "            const Span<ContactForces::Entry> known = memory.of({pair.a, pair.b, 0, pair.partA, pair.partB}, next);\n            next = known.end();\n",
     '            static const bool offWarm = off("KP_OFF_WARMSTART");\n            Span<ContactForces::Entry> known = memory.of({pair.a, pair.b, 0, pair.partA, pair.partB}, next);\n            next = known.end();\n            if (offWarm) known = {};\n'),
    ("src/dynamics/time_stepping/ContactLevels.cpp", "#include <thread>\n", "#include <thread>\n" + OFF),
    ("src/dynamics/time_stepping/ContactLevels.cpp", "    if (WorkerPool::shared().threads() > 1 && count >= kParallelContacts) {",
     '    static const bool offLevels = off("KP_OFF_LEVELS");\n    if (!offLevels && WorkerPool::shared().threads() > 1 && count >= kParallelContacts) {'),
    ("src/dynamics/event_driven/SetFactor.cpp", "#include <algorithm>\n", "#include <algorithm>\n" + OFF),
    ("src/dynamics/event_driven/SetFactor.cpp", "    const bool rebuilt = factor_.stored() > 2 * fresh_;",
     '    static const bool offFollow = off("KP_OFF_SETFACTOR");\n    const bool rebuilt = offFollow || factor_.stored() > 2 * fresh_;'),
]


def remote(command, timeout=1800):
    return subprocess.run([str(REMOTE / "rrun.sh"), command], capture_output=True, text=True, timeout=timeout).stdout.replace("\r", "")


def prepare():
    COPY.mkdir(parents=True, exist_ok=True)
    listed = subprocess.run(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=ROOT, capture_output=True, check=True).stdout
    packed = subprocess.run(["tar", "--null", "-T", "-", "-cf", "-"], cwd=ROOT, input=listed, capture_output=True).stdout
    subprocess.run(["tar", "-xf", "-", "-C", str(COPY)], input=packed, check=True)
    for path, old, new in SWITCHES:
        text = (COPY / path).read_text(encoding="utf-8")
        if text.count(old) != 1: sys.exit(f"переключатель не встал: {path}: {old[:60]!r}")
        (COPY / path).write_text(text.replace(old, new), encoding="utf-8")
    archive = REMOTE / "tree-ablation.tgz"
    subprocess.run(["tar", "-czf", str(archive), "-C", str(COPY), "."], check=True)
    subprocess.run([str(REMOTE / "rcp.sh"), str(archive), "C:/Users/sshuser/CODE/kinetica-physics/tree-ablation.tgz"], check=True)
    print(remote('if (Test-Path src-ablation) { Remove-Item -Recurse -Force src-ablation }; New-Item -ItemType Directory src-ablation | Out-Null; tar -xzf tree-ablation.tgz -C src-ablation; Remove-Item tree-ablation.tgz; '
                 'if (-not (Test-Path build-ablation\\CMakeCache.txt)) { cmake -S src-ablation -B build-ablation -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=clang++ | Out-Null }; '
                 'cmake --build build-ablation -j 16 --target compare-ours study-standard-scenes study-state-hash 2>&1 | Select-String -Pattern "error" | Select-Object -Last 5 | ForEach-Object { $_.Line }; "BUILD_EXIT=$LASTEXITCODE"'))


def switch(env, on):
    return "; ".join(f'$env:{name}={chr(34) + value + chr(34) if on else "$null"}' for name, value in env.items())


def measure(env):
    """Один приём: пары прогонов большой пирамиды, стандартные сцены и хэш с выключенным приёмом."""
    bench = 'cmd /c "start /b /wait {pin}build-ablation\\validation\\engines\\compare-ours.exe realtime 8" | Select-Object -Last 1'
    pairs = lambda tag, pin, threads: (f'$env:KP_THREADS={threads}; foreach ($i in 1..3) {{ {switch(env, False)}; "{tag}0 " + ({bench.format(pin=pin)}); '
                                       f'{switch(env, True)}; "{tag}1 " + ({bench.format(pin=pin)}) }}; ')
    command = ('$env:BENCH="14"; ' + pairs("A", "/affinity 0x100 ", '"1"') + pairs("B", "", "$null") + '$env:BENCH=$null; $env:KP_THREADS=$null; ' + switch(env, True) +
               '; $env:KP_REALTIME="1"; build-ablation\\validation\\studies\\study-standard-scenes.exe rows-ab.jsonl 60 | Out-Null; "ROWS"; Get-Content rows-ab.jsonl; $env:KP_REALTIME=$null; '
               '"HASH"; build-ablation\\validation\\studies\\study-state-hash.exe 60')
    out, part = {"A0": [], "A1": [], "B0": [], "B1": [], "rows": [], "hash": {}}, None
    for line in remote(command).splitlines():
        found = re.match(r"([AB][01]) .*?([\d.]+) мс на кадр", line)
        if found: out[found.group(1)].append(float(found.group(2)))
        elif line in ("ROWS", "HASH"): part = line
        elif part == "ROWS" and line.startswith("{"): out["rows"].append(json.loads(line))
        elif part == "HASH" and "быстрый" in line: out["hash"][line.split()[0]] = line.split()[-1]
    return out


def run(only=()):
    """Все приёмы или только названные (перемер после шумного прогона): прежние числа остальных остаются."""
    results = json.loads(DATA.read_text(encoding="utf-8")) if only else {"base": measure({})}
    for key, env, title in VARIANTS:
        if only and key not in only: continue
        results[key] = measure(env)
        results[key]["title"] = title
        DATA.write_text(json.dumps(results, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        one, many = results[key]["A0"], results[key]["A1"]
        print(f"{title}: один поток {min(one):.2f} → {min(many):.2f} мс", flush=True)


if __name__ == "__main__":
    prepare() if sys.argv[1] == "prepare" else run(sys.argv[2:])
