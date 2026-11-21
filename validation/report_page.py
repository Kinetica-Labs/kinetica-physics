#!/usr/bin/env python3
"""Отчёт Markdown — одной страницей для браузера: рисунки вписаны в неё, внешних файлов нет.

  report_page.py отчёт.md страница.html

Рисунки SVG вставляются как есть, остальные — строкой base64. Страницу можно открыть с диска или отдать
любым сервером: python3 -m http.server --directory <папка страницы>.
"""
import base64
import re
import sys
from pathlib import Path

import markdown

STYLE = """
body { font: 16px/1.5 -apple-system, "Segoe UI", Roboto, "DejaVu Sans", sans-serif; color: #0b0b0b; background: #fcfcfb; margin: 0; }
main { max-width: 900px; margin: 0 auto; padding: 16px 18px 48px; }
h1 { font-size: 1.5em; line-height: 1.25; } h2 { font-size: 1.2em; margin-top: 2em; border-top: 1px solid #e4e3df; padding-top: 0.8em; }
p, li { max-width: 72ch; } code { background: #f0efeb; padding: 0 3px; border-radius: 3px; font-size: 0.92em; }
.table { overflow-x: auto; } table { border-collapse: collapse; margin: 0.8em 0; font-size: 0.93em; }
th, td { border-bottom: 1px solid #e4e3df; padding: 5px 10px 5px 0; text-align: left; vertical-align: top; }
th { color: #52514e; font-weight: 600; } figure { margin: 1em 0; overflow-x: auto; } figure svg, figure img { max-width: 100%; min-width: 560px; height: auto; }
a { color: #2a78d6; }
"""


def picture(folder, source, title):
    path = (folder / source).resolve()
    if path.suffix == ".svg":
        drawing = re.sub(r"<\?xml.*?\?>|<!DOCTYPE.*?>", "", path.read_text(encoding="utf-8"), flags=re.S)
        return f'<figure role="img" aria-label="{title}">{drawing}</figure>'
    kind = {".png": "png", ".jpg": "jpeg", ".gif": "gif"}[path.suffix]
    return f'<figure><img alt="{title}" src="data:image/{kind};base64,{base64.b64encode(path.read_bytes()).decode()}"></figure>'


def page(report):
    text = report.read_text(encoding="utf-8")
    body = markdown.markdown(text, extensions=["tables", "fenced_code", "sane_lists"])
    body = re.sub(r'<p><img alt="([^"]*)" src="([^"]+)" /></p>', lambda found: picture(report.parent, found.group(2), found.group(1)), body)
    body = re.sub(r'href="([^":]+)\.md"', r'href="\1.html"', body)  # соседние отчёты — такими же страницами
    body = body.replace("<table>", '<div class="table"><table>').replace("</table>", "</table></div>")
    title = re.search(r"^# (.+)$", text, flags=re.M).group(1)
    return f'<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>{title}</title><style>{STYLE}</style></head><body><main>{body}</main></body></html>'


if __name__ == "__main__":
    Path(sys.argv[2]).parent.mkdir(parents=True, exist_ok=True)
    Path(sys.argv[2]).write_text(page(Path(sys.argv[1])), encoding="utf-8")
