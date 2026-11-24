"""Build the 2PyBot PDFs (dark theme, A4) from the HTML sources with Microsoft Edge (headless).

    python docs/manual/build_pdf.py            # all documents
    python docs/manual/build_pdf.py hardware   # one document

Two passes: the first PDF is searched for each contents entry's heading to find
its page; the numbers are written into the contents list and the PDF is rebuilt.
Requires Edge, PyMuPDF and Pillow.

Tokens in the HTML:
  {{ring:NAME}}               LED ring diagram (ring_svg.py)
  {{photo:NAME|Title|how}}    img/cut/NAME.png (background removed) on a dark stage,
                              else img/NAME.jpg, else a labelled placeholder
  {{hero:NAME}}               cover photo (cutout preferred)
  {{logo}}                    the 2PyBot logo (hardware/cad/enclosure/rev5/logo_B.svg)
"""
import html
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import pymupdf
from PIL import Image, ImageOps

from ring_svg import svg as ring_svg

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
REV = "Rev 4.7"
DOCS = {
    "user_manual": ("2PyBot_User_Manual.pdf", "2PyBot  |  UM-2PB-001 " + REV),
    "technical_qa": ("2PyBot_Technical_QA.pdf", "2PyBot  |  AN-2PB-002 " + REV),
    "software_manual": ("2PyBot_Software_Manual.pdf", "2PyBot  |  SW-2PB-003 " + REV),
    "hardware_manual": ("2PyBot_Hardware_Manual.pdf", "2PyBot  |  HW-2PB-004 " + REV),
}


CACHE = Path(tempfile.gettempdir()) / "2pybot_manual_img"


def web(src):
    """Downscaled copy of a photo for print (keeps the PDFs small); returns a file URI."""
    CACHE.mkdir(exist_ok=True)
    dst = CACHE / (src.parent.name + "_" + src.name)
    if not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime:
        im = ImageOps.exif_transpose(Image.open(src))
        if src.suffix.lower() == ".png":
            im.thumbnail((1200, 1200), Image.LANCZOS)
            im.save(dst, optimize=True)
        else:
            im = im.convert("RGB")
            im.thumbnail((1500, 1500), Image.LANCZOS)
            im.save(dst, quality=85, optimize=True)
    return dst.as_uri()


def photo_src(name):
    cut = HERE / "img" / "cut" / (name + ".png")
    if cut.exists():
        return web(cut), True
    for ext in (".jpg", ".jpeg", ".png"):
        if (HERE / "img" / (name + ext)).exists():
            return web(HERE / "img" / (name + ext)), False
    return None, False


LOGO = REPO / "hardware" / "cad" / "enclosure" / "rev5" / "logo_B.svg"   # the 2PyBot logo printed on the face


def logo(color="#f08a4b"):
    """The original 2PyBot logo, recoloured, as inline SVG."""
    s = LOGO.read_text(encoding="utf-8")
    s = re.sub(r'\s(width|height)="[^"]*"', "", s, count=2)
    return s.replace('fill="#000"', 'fill="%s"' % color).replace("<svg ", '<svg class="logo" ', 1)


def expand(src):
    src = src.replace("{{logo}}", logo())
    src = re.sub(r"\{\{ring:(\w+)\}\}", lambda m: ring_svg(m.group(1)), src)

    def hero(m):
        path, _ = photo_src(m.group(1))
        return '<img src="%s" alt="">' % path if path else '<div class="ph"><b>Cover photo</b><i>img/%s.jpg</i></div>' % m.group(1)
    src = re.sub(r"\{\{hero:(\w+)\}\}", hero, src)

    def photo(m):
        name, title, how = (m.group(1).split("|") + ["", ""])[:3]
        path, cut = photo_src(name)
        if path and cut:
            return '<div class="stage"><img class="photo" src="%s" alt="%s"></div>' % (path, html.escape(title))
        if path:
            return '<img class="photo full" src="%s" alt="%s">' % (path, html.escape(title))
        return ('<div class="ph"><b>%s</b><span>Photo needed: %s</span><i>img/%s.jpg</i></div>'
                % (html.escape(title), html.escape(how), name))
    return re.sub(r"\{\{photo:([^}]+)\}\}", photo, src)


def finalize(out):
    """Paint the page margins dark (Edge leaves them white) and store large photos as JPEG.

    Edge writes every photo losslessly; the colour part of each large RGB image is
    re-encoded as JPEG in place (its alpha mask is kept), which leaves patterns intact.
    """
    doc = pymupdf.open(out)
    fill = (0x14 / 255, 0x12 / 255, 0x10 / 255)
    for page in doc:
        page.draw_rect(page.rect, color=None, fill=fill, overlay=False)
    done = set()
    for page in doc:
        for img in page.get_images(full=True):
            xref, w, h, bpc, cs, filt = img[0], img[2], img[3], img[4], img[5], img[8]
            if xref in done or filt != "FlateDecode" or cs != "DeviceRGB" or bpc != 8 or w * h < 120000:
                continue
            done.add(xref)
            pix = pymupdf.Pixmap(doc, xref)
            if pix.alpha:
                pix = pymupdf.Pixmap(pix, 0)
            doc.update_stream(xref, pix.tobytes("jpg", jpg_quality=84), compress=False)
            doc.xref_set_key(xref, "Filter", "/DCTDecode")
            doc.xref_set_key(xref, "DecodeParms", "null")
    tmp = out.with_suffix(".tmp.pdf")
    doc.save(tmp, garbage=3, deflate=True)
    doc.close()
    tmp.replace(out)


def render(src_html, out):
    with tempfile.NamedTemporaryFile("w", suffix=".html", dir=HERE, delete=False, encoding="utf-8") as tmp:
        tmp.write(src_html)
    try:
        subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", "--allow-file-access-from-files",
                        "--virtual-time-budget=8000", "--print-to-pdf=" + str(out), Path(tmp.name).as_uri()],
                       check=True, capture_output=True, timeout=240)
    finally:
        Path(tmp.name).unlink()


def norm(s):
    s = re.sub(r"[^0-9a-z.& ]+", " ", html.unescape(s).lower())
    return re.sub(r"\s+", " ", s).strip()


def build(name, pdf_name, footer):
    src = expand((HERE / (name + ".html")).read_text(encoding="utf-8"))
    src = src.replace("</head>", '<style>@page { @bottom-left { content: "%s"; font: 8pt "Segoe UI", sans-serif; color: #857a6f; } }'
                                 "@page cover { @bottom-left { content: none } }</style></head>" % footer)
    out = HERE / pdf_name
    render(src, out)
    entries = re.findall(r'<a(?: class="l2")? href="#([^"]+)">([^<]+)</a>', src)
    doc = pymupdf.open(out)
    pages = [{norm(line) for line in p.get_text().splitlines()} for p in doc]
    doc.close()
    found = {}
    for anchor, label in entries:
        want = norm(label)
        for i, lines in enumerate(pages[2:], start=3):
            if want in lines:
                found[anchor] = i
                break
    for anchor, page in found.items():
        src = src.replace('href="#%s">' % anchor, 'href="#%s" data-pg="%d">' % (anchor, page), 1)
    src = re.sub(r'(href="#[^"]+" data-pg="(\d+)">)([^<]+)</a>', r'\1\3<span class="pg">\2</span></a>', src)
    render(src, out)
    finalize(out)
    doc = pymupdf.open(out)
    n = len(doc)
    doc.close()
    missing = [a for a, _ in entries if a not in found]
    print("%s: %d pages, contents %d/%d%s" % (pdf_name, n, len(found), len(entries), (", missing %s" % missing) if missing else ""))


if __name__ == "__main__":
    pick = sys.argv[1:] or list(DOCS)
    for key in DOCS:
        if any(key.startswith(p) for p in pick):
            build(key, *DOCS[key])
