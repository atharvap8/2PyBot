"""Remove photo backgrounds for the manuals (rembg), crop and size them.

    <rembg venv python> docs/manual/tools/cutout.py

Writes docs/manual/img/cut/<name>.png (RGBA, longest side 1600 px). Originals stay.
"""
from pathlib import Path

from PIL import Image, ImageFilter, ImageOps
from rembg import new_session, remove

IMG = Path(__file__).resolve().parents[1] / "img"
OUT = IMG / "cut"
JOBS = {
    "photo_cover": "photo_cover.jpg", "photo_front": "photo_front.jpg", "photo_side": "photo_side.jpg",
    "photo_rear": "photo_rear.jpg", "photo_top": "photo_top.jpg", "photo_rest": "photo_rest.jpg",
    "photo_gamepad": "photo_gamepad.jpg", "photo_ring": "photo_ring.jpg",
    "photo_quarter_left": "IMG_20261124_151749.jpg", "photo_quarter_right": "IMG_20261124_152403.jpg",
}


def main():
    OUT.mkdir(exist_ok=True)
    session = new_session("isnet-general-use")
    for name, src in JOBS.items():
        im = ImageOps.exif_transpose(Image.open(IMG / src)).convert("RGB")
        im.thumbnail((2200, 2200), Image.LANCZOS)
        cut = remove(im, session=session, post_process_mask=True)
        a = cut.getchannel("A").filter(ImageFilter.GaussianBlur(0.6))
        cut.putalpha(a)
        box = a.point(lambda v: 255 if v > 24 else 0).getbbox()
        if box:
            pad = int(max(cut.size) * 0.025)
            box = (max(0, box[0] - pad), max(0, box[1] - pad), min(cut.width, box[2] + pad), min(cut.height, box[3] + pad))
            cut = cut.crop(box)
        cut.thumbnail((1600, 1600), Image.LANCZOS)
        cut.save(OUT / (name + ".png"), optimize=True)
        print(name, cut.size)


if __name__ == "__main__":
    main()
