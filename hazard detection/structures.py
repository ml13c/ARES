### structures.py ###
# TODO: write a better file descriptor

from dataclasses import dataclass
import numpy as np
import cv2
import time

# Constants
H_4K = 2160
W_4K = 3840

# Image
@dataclass
class Image:
    pix: np.ndarray # shape => (2160, 3840, 3), uint8
    img_id: int
    timestamp: float

    def __post_init__(self): # check for valid pix shape
        if not isinstance(self.pix, np.ndarray):
            raise TypeError(f"pix must be np.ndarray, got {type(self.pix).__name__}")
        if self.pix.dtype != np.uint8:
            raise TypeError(f"pix must be uint8, got {type(self.pix.dtype)}")
        if self.pix.shape != (H_4K, W_4K, 3):
            raise ValueError(f"expected shape {(H_4K, W_4K, 3)}, got {self.pix.shape}")

    @property
    def height(self):
        return self.pix.shape[0]
    
    @property
    def height(self):
        return self.pix.shape[1]
    
    @classmethod
    def from_file(cls, path: str, img_id: int) -> "Image":
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)

        if bgr is None:
            raise FileNotFoundError(f"could not read image: {path}")

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return cls(rgb, img_id, time.time())

    #TODO: Need to have a method that imports directly from frame.
        #i.e. CV2 capture is sent to this...

# GrayImage
@dataclass
class GrayImage:
    pix: np.ndarray # shape => (2160,3840), uint8
    img_id: int

    # from_file -> mostly useful for testing, otherwise GrayImage is created from GrayscaleConverter
    @classmethod
    def from_file(cls, path: str, img_id: int) -> "GrayImage":
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)

        if gray is None:
            raise FileNotFoundError(f"could not read image: {path}")

        return cls(gray, img_id)

# EdgeMap
@dataclass
class EdgeMap:
    magnitude: np.ndarray # shape => (2160, 3840), uint8
    img_id: int

# Tile
@dataclass
class Tile:
    data: np.ndarray # shape depends on TileSplitter
    offset_x: int
    offset_y: int

# BoundingBox
@dataclass
class BoundingBox:
    x1: float
    y1: float    
    x2: float
    y2: float
    score: float

    def to_global(offset_x: int, offset_y: int):
        # returns BoundingBox mapped to 4k image
        pass # implement later

    def width():
        # calculate BoundingBox width after mapping
        pass # implement later

    def height():
        # calculate BoundingBox height after mapping
        pass # implement later
    
# Crop
@dataclass
class Crop:
    img: np.ndarray # shape == shape of image input
    src_box: BoundingBox
    img_id: int

