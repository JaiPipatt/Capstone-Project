"""
AprilTag 36h11 detection, robust to glare and partly cut-off tags.

Each frame is searched twice, on the grayscale image and on a hard black/white
(adaptive threshold) version, and the results are merged:
  - binarized catches tags whose black cells are washed out by glare
  - grayscale catches tags at the frame edge that binarization merges into the background

Measured on the ESP32-CAM: XGA (1024x768) is the lowest resolution that keeps every
tag (rotated, glared, edge) at ~50 px per side. Detection costs ~13 ms per XGA frame.
"""
from dataclasses import dataclass

import cv2
import numpy as np


def wrap_angle(a):
    """Wrap an angle in radians to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class TagDetection:
    """One detected tag, in image pixels.

    x, y   : centre (px, y down)
    theta  : heading of the tag's FRONT (its printed top edge), rad,
             0 = image right, CCW positive (image y flipped to point up)
    side   : shortest side length (px), a size/quality hint
    corners: 4x2 array, TL, TR, BR, BL in the tag's own orientation
    source : 'gray' or 'bin', which image the tag was found in
    """

    tag_id: int
    x: float
    y: float
    theta: float
    side: float
    corners: np.ndarray
    source: str


def binarize(gray, offset=-20):
    """Hard black/white with a local threshold; block size follows image width (~tag size)."""
    block = max(11, (gray.shape[1] // 20) | 1)
    return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY, block, offset)


class TagDetector:
    """Detect AprilTag 36h11 markers. Works with the OpenCV 4.5 and 4.7+ aruco APIs."""

    def __init__(self, dictionary=cv2.aruco.DICT_APRILTAG_36h11, use_binarized=True):
        aruco = cv2.aruco
        self._dict = aruco.getPredefinedDictionary(dictionary)
        self.use_binarized = use_binarized
        if hasattr(aruco, 'ArucoDetector'):                       # OpenCV >= 4.7
            self._det = aruco.ArucoDetector(self._dict, aruco.DetectorParameters())
            self._detect = self._det.detectMarkers
        else:                                                     # OpenCV 4.5 (Humble)
            params = aruco.DetectorParameters_create()
            self._detect = lambda img: aruco.detectMarkers(img, self._dict, parameters=params)

    def detect(self, image, return_binarized=False):
        """Detect tags in a BGR or grayscale image.

        Returns a dict {tag_id: TagDetection}; with return_binarized=True returns
        (dict, binarized_image) so callers can display it.
        """
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        passes = [(gray, 'gray')]
        binimg = None
        if self.use_binarized:
            binimg = binarize(gray)
            passes.append((binimg, 'bin'))

        found = {}
        for img, source in passes:
            corners, ids, _ = self._detect(img)
            if ids is None:
                continue
            for tag_id, c in zip(ids.flatten(), corners):
                tag_id = int(tag_id)
                if tag_id not in found:                           # gray result wins
                    found[tag_id] = _to_detection(tag_id, c.reshape(-1, 2), source)
        return (found, binimg) if return_binarized else found


def _to_detection(tag_id, pts, source):
    centre = pts.mean(0)
    front = (pts[0] + pts[1]) / 2 - centre                        # centre -> middle of top edge
    theta = float(np.arctan2(-front[1], front[0]))
    side = float(min(np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)))
    return TagDetection(tag_id, float(centre[0]), float(centre[1]), theta, side, pts, source)
