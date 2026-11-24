#!/usr/bin/env python3
"""Метрики кода движка против стандарта docs/DEVELOPMENT.md, раздел 2.

  code_metrics.py          таблица: метрика, значение, порог; ниже — худшие нарушения
  code_metrics.py --all    все нарушения, а не пять худших
  code_metrics.py --json   одна строка JSON для журнала табло

Разбор C++ упрощённый: комментарии и строки вырезаются, функции и классы ищутся по скобкам.
Числа сравнимы между прогонами, но это не компилятор: макросы и шаблонные трюки он не видит.
"""
import json
import os
import posixpath
import re
import subprocess
import sys
import zlib
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parent.parent
FUNCTION_LINES, CLASS_LINES, PUBLIC_METHODS = 40, 300, 12
# Слои снизу вверх: файл может включать только заголовки своего слоя и ниже. Подкаталоги collision
# стоят раздельно: формы и площадка контакта ниже тела, поиск пар и луч — выше тел и сочленений.
LAYERS = ["math", "collision/broadphase", "collision/shapes", "collision/narrowphase", "body",
          "joints", "collision", "dynamics", "world", "debug"]


def sources():
    files = sorted(ROOT.glob("src/**/*.cpp")) + sorted(ROOT.glob("include/**/*.h"))
    return {f.relative_to(ROOT).as_posix(): f.read_text(encoding="utf-8") for f in files}


def strip(text):
    """Комментарии и строковые литералы — пробелами, с сохранением переводов строк."""
    blank = lambda m: re.sub(r"[^\n]", " ", m.group(0))
    return re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])\'', blank, text, flags=re.S)


def functions(text):
    """(первая строка, число строк, имя) каждого определения функции вне других функций."""
    found, depth, head, line, start = [], 0, "", 1, None
    for ch in strip(text):
        line += ch == "\n"
        if ch == "{":
            if start is None and ")" in head and not re.search(r"\b(namespace|class|struct|enum|union)\b|=\s*$", head.strip()):
                name = re.search(r"([~\w:]+|operator\S*)\s*$", head.split("(")[0])
                lead = head[:len(head) - len(head.lstrip())]  # пустые строки и комментарий над функцией
                start, opened, title = line - head.count("\n") + lead.count("\n"), depth, name.group(1) if name else "?"
            depth, head = depth + 1, ""
        elif ch == "}":
            depth, head = depth - 1, ""
            if start is not None and depth == opened:
                found.append((start, line - start + 1, title))
                start = None
        else:
            head = "" if ch == ";" else head + ch
    return found


def classes(text):
    """(имя, число строк определения, публичные методы) каждого класса и структуры заголовка."""
    text, found = strip(text), []
    for m in re.finditer(r"\b(class|struct)\s+(\w+)\s*(?:final\s*)?(?::[^{;()]*)?\{", text):
        depth, i = 1, m.end()
        while depth:
            depth += {"{": 1, "}": -1}.get(text[i], 0)
            i += 1
        body = text[m.end():i - 1]
        found.append((m.group(2), body.count("\n") + 1, publicMethods(body, m.group(1) == "struct")))
    return found


def publicMethods(body, isStruct):
    flat, depth = "", 0
    for ch in body:
        if ch == "{": depth += 1
        elif ch == "}":
            depth -= 1
            flat += ";" if depth == 0 else ""
        elif depth == 0: flat += ch
    flat = re.sub(r"<[^<>]*>", "", re.sub(r"<[^<>]*>", "", flat))
    parts = re.split(r"\b(public|private|protected)\s*:", flat)
    sections = [("public" if isStruct else "private", parts[0])] + list(zip(parts[1::2], parts[2::2]))
    statements = [s for access, text in sections if access == "public" for s in text.split(";")]
    isMethod = lambda s: re.search(r"(\w|operator\S+)\s*\(", s) and "=" not in s.split("(")[0].replace("operator=", "")
    return sum(1 for s in statements if isMethod(s) and not re.match(r"\s*(using|friend|typedef)\b", s))


def layerOf(path):
    parts = path.replace("include/", "").replace("src/", "").split("/")
    return "/".join(parts[:2]) if parts[0] == "collision" and len(parts) > 2 else parts[0]


def upwardIncludes(files):
    rank = {name: i for i, name in enumerate(LAYERS)}
    found = []
    for path, text in files.items():
        if path.count("/") < 3 and path.startswith("include"):
            continue  # общий заголовок physics.h включает всё
        for header in re.findall(r'#include\s*<((?:math|collision|body|joints|dynamics|world|debug)/[\w/]+\.h)', text):
            if rank[layerOf("include/" + header)] > rank[layerOf(path)]:
                found.append((path, header))
    return found


def brokenLinks():
    """Локальная ссылка должна вести к файлу или каталогу, доступному читателю репозитория."""
    listed = subprocess.check_output(["git", "ls-files", "-z", "--cached"], cwd=ROOT)
    if not listed:  # Измерители распаковывают снимок в новый git-каталог без индекса.
        listed = subprocess.check_output(["git", "ls-files", "-z", "--others", "--exclude-standard"], cwd=ROOT)
    files = {name for name in os.fsdecode(listed).split("\0") if name and (ROOT / name).is_file()}
    targets = files | {parent.as_posix() for name in files for parent in Path(name).parents}
    links = re.compile(r'\]\(\s*(?:<([^>\n]+)>|([^\s)]+))(?:\s+["\'][^\n]*?["\'])?\s*\)')
    found = []
    for doc in sorted(name for name in files if name.endswith(".md")):
        for match in links.finditer((ROOT / doc).read_text(encoding="utf-8")):
            target = match.group(1) or match.group(2)
            url = urlsplit(target)
            if url.scheme or url.netloc or not url.path:
                continue
            path = posixpath.normpath(posixpath.join(posixpath.dirname(doc), unquote(url.path)))
            if path not in targets:
                found.append((doc, target))
    return found


def measure():
    files = sources()
    lengths = {(p, start, name): n for p, t in files.items() for start, n, name in functions(t)}
    classLines = {}
    for p, t in files.items():
        for name, n, _ in classes(t) if p.endswith(".h") else []:
            classLines[name] = classLines.get(name, 0) + n
    for (p, start, name), n in lengths.items():
        owner = name.split("::")[-2] if "::" in name else None
        if owner in classLines and p.endswith(".cpp"):
            classLines[owner] += n
    wide = sorted(((m, c, p) for p, t in files.items() if p.endswith(".h") for c, _, m in classes(t) if m > PUBLIC_METHODS), reverse=True)
    text = "".join(files.values()).encode("utf-8")
    return {
        "lines": sum(t.count("\n") for t in files.values()),
        "khat": round(len(zlib.compress(text, 9)) / len(text), 3),
        "functions": len(lengths),
        "long_functions": sorted(((n, f"{p}:{s}", name) for (p, s, name), n in lengths.items() if n > FUNCTION_LINES), reverse=True),
        "long_classes": sorted(((n, c) for c, n in classLines.items() if n > CLASS_LINES), reverse=True),
        "wide_classes": wide,
        "upward_includes": sorted(upwardIncludes(files)),
        "broken_links": brokenLinks(),
    }


def report(m, everything):
    rows = [("строк в движке", m["lines"], "меньше — лучше"),
            ("K̂ = gzip / исходник", m["khat"], "больше — плотнее"),
            (f"функций длиннее {FUNCTION_LINES} строк", len(m["long_functions"]), f"0 из {m['functions']}"),
            (f"классов длиннее {CLASS_LINES} строк", len(m["long_classes"]), "0"),
            (f"классов с публичными методами > {PUBLIC_METHODS}", len(m["wide_classes"]), "0"),
            ("включений вверх по слоям", len(m["upward_includes"]), "0"),
            ("битых ссылок в *.md", len(m["broken_links"]), "0")]
    for title, value, goal in rows:
        print(f"{title:38s} {value:>8}   {goal}")
    limit = None if everything else 5
    for key in ("long_functions", "long_classes", "wide_classes", "upward_includes", "broken_links"):
        for item in m[key][:limit]:
            print(f"  {key:16s} " + "  ".join(str(x) for x in item))


if __name__ == "__main__":
    metrics = measure()
    if "--json" in sys.argv:
        print(json.dumps(metrics, ensure_ascii=False))
    else:
        report(metrics, "--all" in sys.argv)
    # Храповик: достигнутое нельзя потерять. С --check нарушение стандарта — ошибка сборки.
    broken = sum(len(metrics[key]) for key in ("long_functions", "long_classes", "wide_classes", "upward_includes", "broken_links"))
    if "--check" in sys.argv and broken:
        print(f"стандарт кода нарушен: {broken}")
        sys.exit(1)
