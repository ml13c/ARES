import cv2
import numpy as np

from structures import GrayImage, EdgeMap

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

    