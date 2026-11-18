#!/usr/bin/env python3
"""Реестр исследования движков; пути ниже — относительно каталога research/.

  facts.py clone <ref> <url> [sparse-path ...]   клон --depth 1 в refs/<ref>, коммит -> data/sources.jsonl
  facts.py restore                               восстановить refs/ по закреплённым коммитам
  facts.py check [engine]                        проверить факты: схема, словарь, свидетельства
  facts.py sample <engine> [n] [seed]            n случайных фактов с процитированными строками
  facts.py tables                                tables/*.md из data/

Все файлы data/*.jsonl дописываются, не переписываются: действует последняя строка с тем же ключом.
"""
import datetime
import json
import random
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA, REFS, TABLES = ROOT / "data", ROOT / "refs", ROOT / "tables"
PRIOR = {"phys_realflow": Path.home() / "GIT/PHYSICS/phys_realflow"}
FILE_KINDS, URL_KINDS = {"src", "prior"}, {"doc", "paper", "talk", "secondary"}


def rows(path):
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []


def latest(rs, key):
    return {key(r): r for r in rs}


def sources():
    return latest(rows(DATA / "sources.jsonl"), lambda r: r["ref"])


def schema():
    return latest(rows(DATA / "schema.jsonl"), lambda r: (r["axis"], r["key"]))


def facts(engine=None):
    rs = [r for f in sorted((DATA / "facts").glob("*.jsonl")) for r in rows(f)]
    return latest([r for r in rs if engine in (None, r["engine"])], lambda r: (r["engine"], r["axis"], r["key"]))


def ref_root(ref):
    return PRIOR.get(ref, REFS / ref)


def git(*args, cwd=None):
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def squash(s):
    return re.sub(r"\s+", " ", s).strip()


def cited(ev):
    """Строки, на которые ссылается свидетельство, или None, если ссылка никуда не ведёт."""
    path = ref_root(ev["ref"]) / ev["path"]
    if not path.is_file():
        return None
    lines = path.read_text(errors="replace").splitlines()
    a, b = ev["lines"]
    return lines[a - 1:b] if 1 <= a <= b <= len(lines) else None


def evidence_errors(ev):
    kind = ev.get("kind")
    if kind in FILE_KINDS:
        if not all(k in ev for k in ("ref", "path", "lines", "quote")):
            return ["нужны ref, path, lines, quote"]
        if not ref_root(ev["ref"]).is_dir():
            return [f"нет каталога {ev['ref']}"]
        body = cited(ev)
        if body is None:
            return [f"{ev['ref']}:{ev['path']}:{ev['lines']} не существует"]
        if squash(ev["quote"]) not in squash("\n".join(body)):
            return [f"{ev['path']}:{ev['lines']} не содержит «{ev['quote']}»"]
        return []
    if kind == "absence":
        hits = subprocess.run(["rg", "-il", ev["pattern"], str(ref_root(ev["ref"]) / ev.get("path", ""))],
                              capture_output=True, text=True).stdout.split()
        return [f"«{ev['pattern']}» найдено в {len(hits)} файлах, напр. {hits[0]}"] if hits else []
    if kind in URL_KINDS:
        return [] if ev.get("url") else ["нужен url"]
    return [f"неизвестный kind {kind!r}"]


def cmd_clone(ref, url, *sparse):
    dst = REFS / ref
    REFS.mkdir(exist_ok=True)
    git("clone", "--depth", "1", *(["--filter=blob:none", "--sparse"] if sparse else []), url, str(dst))
    if sparse:
        git("sparse-checkout", "set", *sparse, cwd=dst)
    row = {"ref": ref, "url": url, "commit": git("rev-parse", "HEAD", cwd=dst),
           "committed": git("log", "-1", "--format=%cs", cwd=dst), "sparse": list(sparse),
           "date": datetime.date.today().isoformat()}
    with (DATA / "sources.jsonl").open("a") as f:
        f.write(json.dumps(row, ensure_ascii=False) + "\n")
    return row


def cmd_restore():
    done = []
    for ref, s in sources().items():
        dst = REFS / ref
        if dst.is_dir():
            continue
        dst.mkdir(parents=True)
        git("init", "-q", cwd=dst)
        git("remote", "add", "origin", s["url"], cwd=dst)
        if s["sparse"]:
            git("sparse-checkout", "set", *s["sparse"], cwd=dst)
        git("fetch", "--depth", "1", *(["--filter=blob:none"] if s["sparse"] else []), "origin", s["commit"], cwd=dst)
        git("checkout", "-q", "FETCH_HEAD", cwd=dst)
        done.append(ref)
    return done


def cmd_check(engine=None):
    sch, errors, vocab = schema(), [], []
    for ref, s in sources().items():
        if (REFS / ref).is_dir() and git("rev-parse", "HEAD", cwd=REFS / ref) != s["commit"]:
            errors.append(f"refs/{ref}: HEAD разошёлся с закреплённым {s['commit'][:9]}")
    fs = facts(engine)
    for (eng, axis, key), r in fs.items():
        tag = f"{eng}.{axis}.{key}"
        spec = sch.get((axis, key))
        if spec is None:
            errors.append(f"{tag}: ключа нет в schema.jsonl")
            continue
        values = r["value"] if isinstance(r["value"], list) else [r["value"]]
        if bool(spec.get("multi")) != isinstance(r["value"], list):
            errors.append(f"{tag}: value должен быть {'списком' if spec.get('multi') else 'скаляром'}")
        if "enum" in spec:
            vocab += [f"{tag}: «{v}» вне словаря" for v in values if v not in spec["enum"]]
        if not r.get("evidence"):
            errors.append(f"{tag}: нет свидетельства")
        errors += [f"{tag}: {e}" for ev in r.get("evidence", []) for e in evidence_errors(ev)]
    for title, items in (("ОШИБКИ", errors), ("ВНЕ СЛОВАРЯ", vocab)):
        if items:
            print(f"{title} ({len(items)}):", *items, sep="\n  ")
    print(f"фактов: {len(fs)}, ошибок: {len(errors)}, вне словаря: {len(vocab)}")
    return not errors


def cmd_sample(engine, n=10, seed=0):
    fs = [r for r in facts(engine).values() if any(e["kind"] in FILE_KINDS for e in r["evidence"])]
    for r in random.Random(int(seed)).sample(fs, min(int(n), len(fs))):
        print(f"\n### {r['engine']}.{r['axis']}.{r['key']} = {json.dumps(r['value'], ensure_ascii=False)}")
        print(f"    {r.get('note', '')}")
        for ev in r["evidence"]:
            if ev["kind"] in FILE_KINDS:
                print(f"--- {ev['ref']}/{ev['path']}:{ev['lines'][0]}-{ev['lines'][1]}")
                print("\n".join(cited(ev) or ["<нет строк>"]))


def cell(r):
    if r is None:
        return "·"
    value = ", ".join(map(str, r["value"])) if isinstance(r["value"], list) else str(r["value"])
    kinds = "/".join(sorted({e["kind"] for e in r["evidence"]}))
    return f"{value} <sub>{kinds}</sub>".replace("|", "\\|")


def cmd_tables():
    sch, fs = schema(), facts()
    engines = sorted({e for e, _, _ in fs})
    axes = list(dict.fromkeys(a for a, _ in sch))
    out = ["# Сводная матрица «движок × ось»", "",
           "Сгенерировано `research/facts.py tables` из `research/data/` — руками не править. "
           "Подстрочная метка — вид свидетельства; `·` — факт не установлен.", "",
           "## Покрытие (заполнено ключей / всего)", "",
           "| Ось | " + " | ".join(engines) + " |", "|---|" + "---|" * len(engines)]
    for axis in axes:
        keys = [k for a, k in sch if a == axis]
        out.append(f"| {axis} | " + " | ".join(
            f"{sum((e, axis, k) in fs for k in keys)}/{len(keys)}" for e in engines) + " |")
    for axis in axes:
        out += ["", f"## {axis}", "", "| Ключ | " + " | ".join(engines) + " |", "|---|" + "---|" * len(engines)]
        out += [f"| `{k}` | " + " | ".join(cell(fs.get((e, axis, k))) for e in engines) + " |"
                for a, k in sch if a == axis]
    TABLES.mkdir(exist_ok=True)
    (TABLES / "matrix.md").write_text("\n".join(out) + "\n")
    return TABLES / "matrix.md"


if __name__ == "__main__":
    commands = {n[4:]: f for n, f in globals().items() if n.startswith("cmd_")}
    if len(sys.argv) < 2 or sys.argv[1] not in commands:
        sys.exit(__doc__)
    result = commands[sys.argv[1]](*sys.argv[2:])
    if result not in (None, True, False):
        print(result)
    sys.exit(result is False)
