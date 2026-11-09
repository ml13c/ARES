import cv2
import numpy as np
from dataclasses import dataclass
from .structures import Tile, Image
from .structures import GrayImage, EdgeMap

class SobelOperator:
    def __init__(self):
        self.k_x = np.array([[-1, 0, 1],
                             [-2, 0, 2],
                             [-1, 0, 1]], dtype=np.float32)
        self.k_y = self.k_x.T # Transposed k_x

    def apply(self, img: GrayImage) -> EdgeMap:
        gx = cv2.filter2D(img.pix, cv2.CV_32F, self.k_x)
        gy = cv2.filter2D(img.pix, cv2.CV_32F, self.k_y)
        mag = np.sqrt(gx**2 + gy**2)
        mag = np.clip(mag, 0, 255).astype(np.uint8)
        return EdgeMap(mag, img.img_id)

# INFO: New idea -> split 4k image into 9 tiles w/ overlaps to feed into image seg and skip sobel
@dataclass (frozen=True) # tile config never changes
class TileConfig:
    tile_x: int = 1344 # /32 = 42 -> must be a multiple of 32
    tile_y: int = 768 # /32 = 24
    overlap_x: int = 96
    overlap_y: int = 72

class TileSplitter: # split image into 9 tiles, 3x3
    # collect where to start each split
    def __init__(self, cfg: TileConfig = TileConfig()):
        self.cfg = cfg
        # steps = how far to step for next tile
        step_x = cfg.tile_x - cfg.overlap_x
        step_y = cfg.tile_y - cfg.overlap_y

        # offsets = the top left corner for each tile (where to start from)
        self.offsets = []
        for row in range(3):
            for col in range(3):
                self.offsets.append((col*step_x,row*step_y))

    # create tiles starting from each offset
    def split(self, image: Image) -> list[Tile]:
        # w = tile_x, h = tile_y
        w,h = self.cfg.tile_x, self.cfg.tile_y

        tiles = []
        for (x,y) in self.offsets:
            # NumPy slices by (row,col) or (y,x)
            tile_pix = image.pix[y:y+h, x:x+w]

            tile = Tile(
                data=tile_pix,
                offset_x=x,
                offset_y=y,
                img_id=image.img_id
            )

            tiles.append(tile)

        return tiles #shape (9x[tile_x,tile_y,3])