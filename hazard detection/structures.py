### structures.py ###
# TODO: write a better file descriptor

from dataclasses import dataclass
import numpy as np


# Image
@dataclass
class Image:
    #TODO: write initializer to accept raw 4k image as input
    #TODO: Add enforcement for pix
    pix: np.ndarray # shape => (2160, 3840, 3), uint8
    img_id: int
    timestamp: float
    width: int = 3840
    hgt: int = 2160

# GrayImage
@dataclass
class GrayImage:
    pix: np.ndarray # shape => (2160,3840), uint8
    img_id: int

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

