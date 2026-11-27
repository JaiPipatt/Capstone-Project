import math

import numpy as np

from msrr_localization.fusion_filter import FusionFilter, wrap


def simulate(f, T=6.0, cam_delay=0.06, enc_scale=1.05, cam_noise=0.002, outlier_at=None,
             encoder=True, seed=0):
    """Module drives an arc (v=0.04 m/s, w=0.4 rad/s). Encoder 20 Hz with a 5 % scale
    error, camera 30 Hz arriving cam_delay late. Returns errors (fused, camera) in m."""
    rng = np.random.default_rng(seed)
    v, w = 0.04, 0.4

    def truth(t):
        return (v / w * math.sin(w * t), v / w * (1 - math.cos(w * t)), wrap(w * t))

    events = []
    if encoder:
        events += [(t, 'u', t) for t in np.arange(0.0, T, 1 / 20)]
    events += [(t + cam_delay, 'z', t) for t in np.arange(0.0, T, 1 / 30)]
    events.sort()
    err_f, err_c = [], []
    for t_arrive, kind, t in events:
        if kind == 'u':
            f.add_encoder(t, enc_scale * v + rng.normal(0, 0.001),
                          enc_scale * w + rng.normal(0, 0.01))
        else:
            x, y, th = truth(t)
            zx, zy = x + rng.normal(0, cam_noise), y + rng.normal(0, cam_noise)
            if outlier_at is not None and abs(t - outlier_at) < 1e-3:
                zx += 0.2
            f.add_camera(t, zx, zy, th + rng.normal(0, 0.005))
            if t_arrive > 1.0:   # after settling: compare at the arrival time ("now")
                tx, ty, _ = truth(t_arrive)
                fx, fy, _, _ = f.state_at(t_arrive)
                err_f.append(math.hypot(fx - tx, fy - ty))
                err_c.append(math.hypot(zx - tx, zy - ty))   # latest camera, uncompensated
    return np.mean(err_f), np.mean(err_c)


def test_fusion_beats_late_camera():
    fused, camera = simulate(FusionFilter())
    assert fused < 0.5 * camera, (fused, camera)
    assert fused < 0.002


def test_encoder_drift_is_corrected():
    # 5 % encoder error over 6 s would be ~1.2 cm by dead reckoning alone
    fused, _ = simulate(FusionFilter(), enc_scale=1.05)
    assert fused < 0.003


def test_outlier_rejected():
    f = FusionFilter()
    fused, _ = simulate(f, outlier_at=3.0)
    assert fused < 0.003


def test_camera_only_still_tracks():
    fused, camera = simulate(FusionFilter(), encoder=False)
    assert fused < 2 * camera + 0.003


def test_late_measurement_equals_in_order():
    a, b = FusionFilter(), FusionFilter()
    for f in (a, b):
        f.add_camera(0.0, 0.0, 0.0, 0.0)
    a.add_encoder(0.10, 0.05, 0.2)
    a.add_camera(0.15, 0.003, 0.0, 0.01)
    a.add_encoder(0.20, 0.05, 0.2)
    b.add_encoder(0.10, 0.05, 0.2)
    b.add_encoder(0.20, 0.05, 0.2)
    b.add_camera(0.15, 0.003, 0.0, 0.01)          # arrives after the newer encoder message
    assert np.allclose(a.state_at(0.3)[:3], b.state_at(0.3)[:3])
