"""Build the 2PyBot PDFs from the HTML sources with Microsoft Edge (headless).

Two passes: the first PDF is searched for each contents entry's heading to find
its page; the numbers are written into the contents list and the PDF is rebuilt.
Requires Edge and PyMuPDF (pip install pymupdf).
    python docs/manual/build_pdf.py
"""
import html
import re
import subprocess
import tempfile
from pathlib import Path

import pymupdf

from ring_svg import svg as ring_svg

HERE = Path(__file__).resolve().parent
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
DOCS = {"user_manual": ("2PyBot_User_Manual.pdf", "2PyBot  |  UM-2PB-001 Rev 4.7"),
        "technical_qa": ("2PyBot_Technical_QA.pdf", "2PyBot  |  AN-2PB-002 Rev 4.7")}


def expand(src):
    """{{ring:NAME}} -> LED ring diagram; {{photo:FILE|Title|how to shoot}} -> the
    photo if img/FILE.(jpg|png) exists, otherwise a labelled placeholder."""
    src = re.sub(r"\{\{ring:(\w+)\}\}", lambda m: ring_svg(m.group(1)), src)

    def photo(m):
        name, title, how = (m.group(1).split("|") + ["", ""])[:3]
        for ext in (".jpg", ".jpeg", ".png"):
            if (HERE / "img" / (name + ext)).exists():
                return f'<img class="photo" src="img/{name}{ext}" alt="{html.escape(title)}">'
        return (f'<div class="ph"><b>{html.escape(title)}</b><span>Photo needed: {html.escape(how)}</span>'
                f'<i>img/{name}.jpg</i></div>')
    return re.sub(r"\{\{photo:([^}]+)\}\}", photo, src)


def render(src_html: str, out: Path):
    with tempfile.NamedTemporaryFile("w", suffix=".html", dir=HERE, delete=False, encoding="utf-8") as tmp:
        tmp.write(src_html)
    try:
        subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                        "--virtual-time-budget=4000", f"--print-to-pdf={out}", Path(tmp.name).as_uri()],
                       check=True, capture_output=True, timeout=120)
    finally:
        Path(tmp.name).unlink()


def norm(s):
    s = re.sub(r"[^0-9a-z.& ]+", " ", html.unescape(s).lower())
    return re.sub(r"\s+", " ", s).strip()


def build(name, pdf_name, footer):
    src = expand((HERE / f"{name}.html").read_text(encoding="utf-8"))
    src = src.replace("</head>", f'<style>@page {{ @bottom-left {{ content: "{footer}"; font: 8pt "Segoe UI", sans-serif; color: #73695f; }} }}'
                                 f'@page cover {{ @bottom-left {{ content: none }} }}</style></head>')
    out = HERE / pdf_name
    render(src, out)
    entries = re.findall(r'<a(?: class="l2")? href="#([^"]+)">([^<]+)</a>', src)
    doc = pymupdf.open(out)
    # A heading is one rendered line equal to its contents label (number + title).
    pages = [{norm(line) for line in p.get_text().splitlines()} for p in doc]
    doc.close()
    found = {}
    for anchor, label in entries:
        want = norm(label)
        for i, lines in enumerate(pages[2:], start=3):     # skip cover and contents
            if want in lines:
                found[anchor] = i
                break
    for anchor, page in found.items():
        src = src.replace(f'href="#{anchor}">', f'href="#{anchor}" data-pg="{page}">', 1)
    src = re.sub(r'(href="#[^"]+" data-pg="(\d+)">)([^<]+)</a>', r'\1\3<span class="pg">\2</span></a>', src)
    render(src, out)
    missing = [a for a, _ in entries if a not in found]
    print(f"{pdf_name}: contents pages resolved {len(found)}/{len(entries)}" + (f", missing {missing}" if missing else ""))


if __name__ == "__main__":
    for name, (pdf, footer) in DOCS.items():
        build(name, pdf, footer)
