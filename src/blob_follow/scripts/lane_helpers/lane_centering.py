import math
from lane_follow_blob.vec import Vec
from numpy import ndarray
from .utils import rows, cols, draw_point, deg2rad
from typing import List
import numpy as np

def point_on_line(x0, y0, theta, r):
    """
    Use the parametric equation of a line to find a point
    that is r units away from (x0,y0) at theta degrees
    Convert to int t
    """
    x = x0 + r * math.cos(theta)
    y = y0 + r * math.sin(theta)
    return Vec(x, y)


def raytrace_find_nonzero(image: ndarray,
                         p0: Vec,
                         theta: float,
                         r_step=5,
                         iters=100,
                         debug_image=False) -> List[Vec]:
    """
    Cast a ray from p0 in the direction of theta
    Stop at the first non-zero pixel
    Assumes image is a 1-channel ndarray
    """
    # Don't start at zero if using draw_point in loop, see note below
    for r in range(3, iters+3):
        p = point_on_line(p0.x, p0.y, theta, -r * r_step)
        p = Vec(int(p.x), int(p.y)) # convert to int since we are indexing pixels

        # Edge of image
        if not (0 < p.x < cols(image) and 0 < p.y < rows(image)):
            # out of bounds, stop
            break

        # Found non-zero
        if image[int(p.y), int(p.x)] > 0:
            break

        if debug_image is not None:
            draw_point(debug_image, p, r=1)

    return p


def compute_spring_force(thetas: ndarray, spring_lengths: ndarray):
    k = 1 # spring constant

    # Use -k so springs pull
    force = -k * spring_lengths

    # Compute force vector along each angle
    force_vectors = [Vec(math.cos(theta), math.sin(theta)).smul(f) for theta, f in zip(thetas, force)]

    # Compute the mean force
    p_diff = Vec(0,0)
    for v in force_vectors:
        p_diff = p_diff.add(v)
    p_diff = p_diff.sdiv(len(force_vectors))

    return p_diff
    

def center_lane(image: ndarray, p0: Vec, debug_image=True, iters=100) -> Vec:
    if debug_image is not None:
        draw_point(debug_image, p0, color=(0,0,255), r=5)

    # The angles of our springs
    thetas = deg2rad(np.array(list(range(0, 180+1, 10))))

    # The locations where the springs intersect the lane lines
    points = [raytrace_find_nonzero(image, p0, theta, debug_image=debug_image, iters=iters) for theta in thetas]
    if debug_image is not None:
        for p in points:
            draw_point(debug_image, p, r=3, color=(255, 50, 50))

    # Compute the overall spring force on the point
    spring_lengths = np.array([Vec.dist(p0, p) for p in points])
    p_diff = compute_spring_force(thetas, spring_lengths)

    # Draw the force vector as a point relative to p0
    p_final = Vec(p0.x + p_diff.x, p0.y + p_diff.y)
    if debug_image is not None:
        draw_point(debug_image, Vec(p_final.x, p_final.y), color=(0,255,255), r=5)

    # Normalize to size of image
    p_diff = Vec(p_diff.x / cols(image), p_diff.y / rows(image))

    return p_diff
