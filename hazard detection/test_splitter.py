import cv2
import numpy as np
import matplotlib.pyplot as plt

from src import preprocessing, structures
from pathlib import Path

# ---------------------------------------------------------------
# Load test image
# Image requires exactly 3840x2160, so resize if the test photo isn't 4K.
# ---------------------------------------------------------------
PATH = "testing/test_images/Image2.jpg"
img = structures.Image.from_file(PATH,1)

# ---------------------------------------------------------------
# Split
# ---------------------------------------------------------------
splitter = preprocessing.TileSplitter()
tiles = splitter.split(img)

# ---------------------------------------------------------------
# Tests
# ---------------------------------------------------------------

# 1. Count and Shape
assert len(tiles) == 9, f"expected 9 tiles, got {len(tiles)}"
for i, t in enumerate(tiles, start=1):
    print(f"Tile {i}: offset ({t.offset_x}, {t.offset_y}), shape {t.data.shape}")


# 2. Display in 3x3 grid
fig, axes = plt.subplots(3, 3, figsize=(15, 9))

for i, t in enumerate(tiles):
    row, col = divmod(i, 3)          # i = 0..8 -> (row, col)
    ax = axes[row, col]
    ax.imshow(t.data)                # Image stores RGB, which matplotlib expects
    ax.set_title(f"Tile {i + 1}  ({t.offset_x}, {t.offset_y})", fontsize=9)
    ax.axis("off")

fig.suptitle("3x3 tiles")
plt.tight_layout()
plt.show()

# ---------------------------------------------------------------
# 3. Save tiles
# ---------------------------------------------------------------
out_dir = Path("testing/tiles")
out_dir.mkdir(parents=True, exist_ok=True)       # create the folder if it doesn't exist

for i, t in enumerate(tiles):
    row, col = divmod(i, 3)
    name = f"img{t.img_id}({row},{col}).png"

    bgr = cv2.cvtColor(t.data, cv2.COLOR_RGB2BGR)   # cv2 saves BGR, Image stores RGB
    ok = cv2.imwrite(str(out_dir / name), bgr)
    if not ok:
        raise IOError(f"failed to write {out_dir / name}")

print(f"saved {len(tiles)} tiles to {out_dir}")