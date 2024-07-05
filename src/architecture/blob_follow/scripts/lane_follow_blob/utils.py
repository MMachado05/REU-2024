from numpy import ndarray
from .vec import Vec
import math
import cv2 as cv

def rows(mat: ndarray) -> int:
    """ Rows in an image """
    return mat.shape[0]

def cols(mat: ndarray) -> int:
    """ Cols in an image """
    return mat.shape[1]

def draw_point(image: ndarray, p: Vec, color=(255,255,255), r=5):
    """ Draw a point on an image """
    cv.circle(image, (int(p.x), int(p.y)), int(r), color, -1)

def deg2rad(deg):
    """ Convert radians to degrees """
    return deg * (math.pi / 180)