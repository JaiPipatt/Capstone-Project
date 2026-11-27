import math

import numpy as np
import pytest

from msrr_localization.floor import fit_homography, FloorMap


def test_scale_maps_centre_to_origin_and_flips_y():
    floor = FloorMap.from_scale(0.001, 1000, 800)
    assert floor.point(500, 400) == pytest.approx((0.0, 0.0))
    assert floor.point(600, 300) == pytest.approx((0.1, 0.1))   # right and up


def test_scale_keeps_heading():
    floor = FloorMap.from_scale(0.001, 1000, 800)
    for theta in (0.0, 0.7, math.pi / 2, -2.5):
        assert floor.pose(500, 400, theta)[2] == pytest.approx(theta)


def test_fit_recovers_known_homography(tmp_path):
    truth = FloorMap([[0.0012, 0.0001, -0.6], [0.0002, -0.0011, 0.45], [1e-6, 2e-6, 1.0]])
    image = [(100, 100), (900, 120), (880, 700), (120, 680), (500, 400)]
    world = [truth.point(u, v) for u, v in image]
    floor = fit_homography(image, world, (1024, 768))
    for (u, v), w in zip(image, world):
        assert floor.point(u, v) == pytest.approx(w, abs=1e-6)

    path = tmp_path / 'h.yaml'
    floor.save(str(path))
    loaded = FloorMap.load(str(path))
    assert loaded.image_size == (1024, 768)
    assert np.allclose(loaded.H, floor.H)


def test_fit_needs_four_points():
    with pytest.raises(ValueError):
        fit_homography([(0, 0), (1, 0), (0, 1)], [(0, 0), (1, 0), (0, 1)])


def test_anchors_map_tags_to_their_world_points():
    a, b = (57.0, 84.0), (1210.0, 610.0)
    wa, wb = (0.0, 0.0), (1.635, -0.76)
    floor = FloorMap.from_anchors(a, b, wa, wb, (1280, 720))
    assert floor.point(*a) == pytest.approx(wa, abs=1e-9)
    assert floor.point(*b) == pytest.approx(wb, abs=1e-9)
    # image right -> world +x, image down -> world -y (no mirror)
    x0, y0 = floor.point(600, 300)
    x1, y1 = floor.point(700, 300)
    x2, y2 = floor.point(600, 400)
    assert x1 > x0 and y2 < y0
    # similarity: same scale in both directions, headings preserved up to rotation
    assert math.hypot(x1 - x0, y1 - y0) == pytest.approx(math.hypot(x2 - x0, y2 - y0))
    rot = floor.pose(600, 300, 0.0)[2]
    assert floor.pose(600, 300, 1.0)[2] == pytest.approx(rot + 1.0)
