"""
EKF that fuses wheel encoders with the overhead camera, for one module.

  f = FusionFilter()
  f.add_encoder(t, v, w)            # /module_i/esp32/vel_state, body frame m/s, rad/s
  f.add_camera(t, x, y, yaw)        # world frame, t = capture time of the frame
  x, y, yaw, P = f.state_at(now)

State [x, y, yaw] in the world frame (metres, rad).
  predict : unicycle model driven by the encoder v, w (fast, smooth, drifts with slip)
  update  : camera pose (absolute, no drift, but late and noisy)

The camera is late (exposure + USB + detection), so a measurement usually arrives
after encoder messages that are newer than it. Every event is kept for `horizon`
seconds with the state before it; a late measurement rewinds to its own time,
is applied there, and the newer encoder events are replayed on top.
"""
import bisect
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class FusionFilter:

    def __init__(self,
                 cam_pos_std=0.003,      # m, camera position noise (incl. calibration)
                 cam_yaw_std=0.01,       # rad
                 enc_v_std=0.002,        # m/s, encoder noise floor
                 enc_w_std=0.02,         # rad/s
                 enc_v_rel=0.10,         # fraction of |v|, wheel slip / radius error
                 enc_w_rel=0.10,         # fraction of |w|, track width error / slip
                 encoder_timeout=0.3,    # s without encoder -> assume still, large noise
                 stale_v_std=0.05,       # m/s, motion noise when the encoder is missing
                 stale_w_std=0.5,        # rad/s
                 gate_chi2=16.3,         # 3 dof, 99.9 %: reject camera outliers
                 reset_after=10,         # consecutive rejections -> jump to the camera
                 horizon=1.0):           # s of history for late measurements
        self.R = np.diag([cam_pos_std ** 2, cam_pos_std ** 2, cam_yaw_std ** 2])
        self.enc_std = (enc_v_std, enc_w_std, enc_v_rel, enc_w_rel)
        self.encoder_timeout = encoder_timeout
        self.stale_std = (stale_v_std, stale_w_std)
        self.gate_chi2 = gate_chi2
        self.reset_after = reset_after
        self.horizon = horizon

        self.x = None            # [x, y, yaw]; None until the first camera measurement
        self.P = None
        self.t = None            # time of the state
        self.u = (0.0, 0.0)      # latest encoder v, w
        self.t_u = -math.inf     # time of the latest encoder message
        self.t_cam = -math.inf   # time of the latest accepted camera measurement
        self.rejected = 0
        self.events = []         # [(t, kind, data, snapshot before it)], sorted by t
        self._times = []

    @property
    def initialized(self):
        return self.x is not None

    # ------------------------------------------------------------ inputs
    def add_encoder(self, t, v, w):
        self._add(t, 'u', (float(v), float(w)))

    def add_camera(self, t, x, y, yaw):
        self._add(t, 'z', np.array([x, y, wrap(yaw)], dtype=float))

    def _add(self, t, kind, data):
        if not self.initialized:
            if kind == 'u':
                self.u, self.t_u = data, t
                return
            self._reset(t, data)
            return
        if self.events and t < self.events[0][0]:
            return                                    # older than the history: drop
        k = bisect.bisect_right(self._times, t)
        if k == len(self.events):
            self._apply(t, kind, data, first=True)
            return
        self._restore(self.events[k][3])              # rewind to just before t
        later = self.events[k:]
        del self.events[k:], self._times[k:]
        self._apply(t, kind, data, first=True)
        for ev in later:
            self._apply(ev[0], ev[1], ev[2], first=False)

    # ------------------------------------------------------------ core
    def _snapshot(self):
        return (self.x.copy(), self.P.copy(), self.t, self.u, self.t_u, self.t_cam,
                self.rejected)

    def _restore(self, s):
        self.x, self.P, self.t, self.u, self.t_u, self.t_cam, self.rejected = \
            s[0].copy(), s[1].copy(), s[2], s[3], s[4], s[5], s[6]

    def _apply(self, t, kind, data, first):
        snap = self._snapshot()
        self._predict(t)
        if kind == 'u':
            self.u, self.t_u = data, t
        else:
            self._update(t, data, count=first)
        self.events.append((t, kind, data, snap))
        self._times.append(t)
        while self.events and self.events[0][0] < t - self.horizon:
            self.events.pop(0)
            self._times.pop(0)

    def _reset(self, t, z):
        self.x, self.P, self.t = z.copy(), self.R.copy(), t
        self.t_cam, self.rejected = t, 0
        self.events, self._times = [], []

    def _predict(self, t, x=None, P=None):
        """Propagate (x, P) from self.t to t. Updates the filter when x, P are None."""
        own = x is None
        x = self.x.copy() if own else x
        P = self.P.copy() if own else P
        dt = t - self.t
        if dt > 0:
            if t - self.t_u > self.encoder_timeout:
                v, w = 0.0, 0.0
                sv, sw = self.stale_std
            else:
                v, w = self.u
                v0, w0, vr, wr = self.enc_std
                sv, sw = v0 + vr * abs(v), w0 + wr * abs(w)
            th = x[2] + 0.5 * w * dt                  # midpoint heading
            c, s = math.cos(th), math.sin(th)
            x = x + np.array([v * c * dt, v * s * dt, w * dt])
            x[2] = wrap(x[2])
            F = np.array([[1.0, 0.0, -v * s * dt],
                          [0.0, 1.0, v * c * dt],
                          [0.0, 0.0, 1.0]])
            G = np.array([[c * dt, -0.5 * v * s * dt * dt],
                          [s * dt, 0.5 * v * c * dt * dt],
                          [0.0, dt]])
            P = F @ P @ F.T + G @ np.diag([sv * sv, sw * sw]) @ G.T
        if own:
            self.x, self.P, self.t = x, P, max(self.t, t)
        return x, P

    def _update(self, t, z, count):
        r = z - self.x
        r[2] = wrap(r[2])
        S = self.P + self.R
        d2 = float(r @ np.linalg.solve(S, r))
        if d2 > self.gate_chi2:
            if count:
                self.rejected += 1
            if self.rejected >= self.reset_after:     # picked up / moved by hand
                self.x, self.P, self.rejected, self.t_cam = z.copy(), self.R.copy(), 0, t
            return False
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ r
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(3) - K) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        self.rejected, self.t_cam = 0, t
        return True

    # ------------------------------------------------------------ output
    def state_at(self, t):
        """(x, y, yaw, P) extrapolated to t with the latest encoder input; filter unchanged."""
        x, P = self._predict(t, self.x.copy(), self.P.copy())
        return x[0], x[1], x[2], P

    def velocity(self, t):
        """Body-frame (v, w) used for prediction at time t."""
        return (0.0, 0.0) if t - self.t_u > self.encoder_timeout else self.u
