"""Contact sheet of the manual photos (for review)."""
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageOps

src = Path(sys.argv[1])
out = Path(sys.argv[2])
files = sorted([p for p in src.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")])
T = 360
cols = 5
rows = (len(files) + cols - 1) // cols
sheet = Image.new("RGB", (cols * T, rows * (T + 24)), (30, 30, 30))
d = ImageDraw.Draw(sheet)
for i, f in enumerate(files):
    im = ImageOps.exif_transpose(Image.open(f)).convert("RGB")
    im.thumbnail((T, T))
    x, y = (i % cols) * T, (i // cols) * (T + 24)
    sheet.paste(im, (x + (T - im.width) // 2, y + (T - im.height) // 2))
    d.text((x + 4, y + T + 4), f.name, fill=(240, 138, 75))
sheet.save(out)
print(out, len(files))
