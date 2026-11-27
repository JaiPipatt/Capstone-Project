"""
Constant-velocity Kalman filter for one tag in the image plane.

State [x, y, theta, vx, vy, omega] in px, rad, px/s, rad/s (image frame, y down,
theta CCW positive). Handles:
  - variable dt (use the camera capture timestamp, not network arrival time)
  - prediction through missed detections
  - angle wrapping at +-180 deg
  - outlier gating (warped / rolling-shutter-skewed detections), with automatic
    re-initialisation if the filter itself is wrong (3 rejects in a row)
  - measurement noise that grows with speed (blur) and for small tags

Cost is ~20 us per predict+update.
"""
import numpy as np

from msrr_localization.tag_detection import wrap_angle


class TagKalmanFilter:
    H = np.hstack([np.eye(3), np.zeros((3, 3))])

    def __init__(self, x, y, theta, t,
                 accel_std=300.0,        # px/s^2, how hard modules accelerate / turn
                 alpha_std=6.0,          # rad/s^2, angular acceleration
                 gate_chi2=16.3,         # 3 dof, ~99.9 %
                 gate_after=8):          # updates before gating starts (velocity must settle)
        self.accel_std = accel_std
        self.alpha_std = alpha_std
        self.gate_chi2 = gate_chi2
        self.gate_after = gate_after
        self.reset(x, y, theta, t)

    def reset(self, x, y, theta, t):
        self.x = np.array([x, y, theta, 0.0, 0.0, 0.0])
        self.P = np.diag([4.0, 4.0, 0.01, 300.0**2, 300.0**2, 3.0**2])
        self.t = t               # time of the current state
        self.t_seen = t          # time of the last accepted detection
        self.hits = 1
        self.rejects = 0

    def predict(self, t):
        dt = t - self.t
        if dt <= 0:
            return
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        g = np.array([dt**2 / 2, dt])
        Q = np.zeros((6, 6))
        for i, std in ((0, self.accel_std), (1, self.accel_std), (2, self.alpha_std)):
            Q[np.ix_([i, i + 3], [i, i + 3])] = std**2 * np.outer(g, g)
        self.x = F @ self.x
        self.x[2] = wrap_angle(self.x[2])
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def update(self, x, y, theta, side_px=50.0):
        """Correct with a detection taken at self.t (call predict(t) first).

        Returns True if accepted, False if rejected as an outlier.
        """
        speed = np.hypot(self.x[3], self.x[4])
        s_xy = 1.0 + 0.02 * speed + 40.0 / max(side_px, 10.0)
        s_th = np.radians(1.5) + 0.002 * speed / max(side_px, 10.0)
        R = np.diag([s_xy**2, s_xy**2, s_th**2])

        y_res = np.array([x, y, theta]) - self.H @ self.x
        y_res[2] = wrap_angle(y_res[2])
        S = self.H @ self.P @ self.H.T + R
        d2 = float(y_res @ np.linalg.solve(S, y_res))       # squared Mahalanobis distance
        if self.hits > self.gate_after and d2 > self.gate_chi2:
            self.rejects += 1
            if self.rejects < 3:
                return False
            self.reset(x, y, theta, self.t)       # filter lost: trust the camera again
            return True

        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y_res
        self.x[2] = wrap_angle(self.x[2])
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.t_seen = self.t
        self.hits += 1
        self.rejects = 0
        return True

    def state_at(self, t):
        """State extrapolated to time t without changing the filter (latency compensation)."""
        dt = max(0.0, t - self.t)
        s = self.x.copy()
        s[0] += s[3] * dt
        s[1] += s[4] * dt
        s[2] = wrap_angle(s[2] + s[5] * dt)
        return s
