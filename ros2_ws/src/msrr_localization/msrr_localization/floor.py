"""
Image pixels -> floor metres for the overhead camera.

The floor (more exactly: the plane of the tags on top of the modules) is flat, so
one 3x3 homography H maps an image point (u, v) to a world point (x, y):
    [x*w, y*w, w] = H @ [u, v, 1]
World frame: x, y on the floor, z up (right-handed), origin wherever the
calibration points put it. Fit H with tag_calibrate; FloorMap.from_scale is a
fallback that is only right if the camera looks straight down.
"""
import math
import os

import cv2
import numpy as np
import yaml


class FloorMap:

    def __init__(self, H, image_size=None):
        self.H = np.asarray(H, dtype=float).reshape(3, 3)
        self.image_size = tuple(image_size) if image_size is not None else None  # (w, h) px

    @classmethod
    def from_scale(cls, m_per_px, width, height):
        """Plain scale around the image centre: x = image right, y = image up."""
        cx, cy = width / 2.0, height / 2.0
        return cls([[m_per_px, 0.0, -m_per_px * cx],
                    [0.0, -m_per_px, m_per_px * cy],
                    [0.0, 0.0, 1.0]], (width, height))

    @classmethod
    def load(cls, path):
        with open(os.path.expanduser(path)) as f:
            data = yaml.safe_load(f)
        return cls(data['homography'], data.get('image_size'))

    def save(self, path, **extra):
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        data = {'homography': [float(v) for v in self.H.flatten()]}
        if self.image_size is not None:
            data['image_size'] = [int(v) for v in self.image_size]
        data.update(extra)
        with open(path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def point(self, u, v):
        x, y, w = self.H @ np.array([u, v, 1.0])
        return float(x / w), float(y / w)

    def pose(self, u, v, theta, length=20.0):
        """Image pose -> world (x, y, yaw).

        theta is the tracker's image heading: 0 = image right, CCW positive with
        image y flipped. The heading is mapped through H with a point `length` px ahead.
        """
        x, y = self.point(u, v)
        x2, y2 = self.point(u + length * math.cos(theta), v - length * math.sin(theta))
        return x, y, math.atan2(y2 - y, x2 - x)


def fit_homography(image_pts, world_pts, image_size=None):
    """Least-squares H from >= 4 point pairs (not all on one line)."""
    image_pts = np.asarray(image_pts, dtype=np.float32).reshape(-1, 2)
    world_pts = np.asarray(world_pts, dtype=np.float32).reshape(-1, 2)
    if len(image_pts) < 4:
        raise ValueError('need at least 4 points')
    H, _ = cv2.findHomography(image_pts, world_pts, 0)
    if H is None:
        raise ValueError('homography fit failed (points on one line?)')
    return FloorMap(H, image_size)
