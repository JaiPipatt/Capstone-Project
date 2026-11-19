"""
Closed-loop execution of a wheel-dock plan, segment by segment (no ROS).

  pivot     wheel d held at zero rate, the other wheel turns the module to the
            segment's final heading; done when |theta error| < pivot_tol
  straight  drive along the planned wheel-d line with heading and cross-track
            correction; done when the docking wheel's ENCODER angle has turned
            by the planned dphi_d (not on time), so phi_d(T) = phi_f (mod pi/2)
            holds up to the encoder tolerance
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from msrr_control.wheel_dock import wheel_centre, WheelDockPlan, wrap_angle


@dataclass
class ExecConfig:
    pivot_tol: float = 0.005    # rad
    enc_tol: float = 0.002      # rad, docking-wheel encoder
    k_theta: float = 2.0        # 1/s, heading gain on straights
    k_lat: float = 60.0         # rad/(s m), cross-track gain on straights
    k_pivot: float = 1.5        # 1/s, pivot heading gain
    k_enc: float = 1.5          # 1/s, slow down over the last part of a straight
    min_rate: float = 0.03      # rad/s, smallest wheel rate commanded while moving


class WheelDockExecutor:

    def __init__(self, plan: WheelDockPlan, cfg: ExecConfig):
        self.plan = plan
        self.p = plan.params
        self.cfg = cfg
        self.idx = 0
        self.seg_phi_start: Optional[float] = None
        self.done = not plan.segments

    @property
    def segment(self):
        return self.plan.segments[self.idx] if not self.done else None

    def _limit(self, rates: np.ndarray) -> Tuple[float, float]:
        m = float(np.max(np.abs(rates)))
        if m > self.p.omega_max:
            rates = rates * (self.p.omega_max / m)
        return float(rates[0]), float(rates[1])

    def _next(self):
        self.idx += 1
        self.seg_phi_start = None
        self.done = self.idx >= len(self.plan.segments)

    def update(self, g: Sequence[float], phi: Sequence[float]) -> Tuple[float, float]:
        """
        Run one control tick.

        g: measured axle pose (x, y, theta), phi: wheel angles (phi_1, phi_2).
        Returns wheel rates (phi_dot_1, phi_dot_2); sets done when finished.
        """
        d, p, cfg = self.plan.d, self.p, self.cfg
        while not self.done:
            sg = self.segment
            if self.seg_phi_start is None:
                self.seg_phi_start = phi[d - 1]

            if sg.kind == 'pivot':
                err = wrap_angle(sg.end[2] - g[2])
                if abs(err) < cfg.pivot_tol:
                    self._next()
                    continue
                # theta_dot = -(r/W) phi_dot_other (eq. 2), wheel d held still
                th_rate = math.copysign(
                    max(min(cfg.k_pivot * abs(err), p.r * self.plan.omega_p / p.W),
                        p.r * cfg.min_rate / p.W), err)
                rates = np.zeros(2)
                rates[1 if d == 1 else 0] = -(p.W / p.r) * th_rate
                return self._limit(rates)

            # straight
            remaining = sg.dphi_d - (phi[d - 1] - self.seg_phi_start)
            if abs(remaining) < cfg.enc_tol or remaining * sg.dphi_d < 0:
                self._next()
                continue
            wheel_rate = max(min(self.plan.omega_s, cfg.k_enc * abs(remaining)), cfg.min_rate)
            v = sg.direction * p.r * wheel_rate
            e_th = wrap_angle(g[2] - sg.start[2])
            a = wheel_centre(sg.start, d, p)
            b = wheel_centre(sg.end, d, p)
            u = (b - a) / max(np.linalg.norm(b - a), 1e-9)    # travel direction
            n = np.array([-u[1], u[0]])                       # left of travel
            e_lat = float((wheel_centre(g, d, p) - a) @ n)
            # turning left (theta up) moves wheel d to the left of its travel direction
            th_dot = -cfg.k_theta * e_th - cfg.k_lat * e_lat * abs(v) / (p.r * self.plan.omega_s)
            # forward speed v = (r/2)(phi_dot_2 - phi_dot_1), theta_dot = -(r/W)(sum) (eq. 2)
            rates = np.array([-v / p.r, v / p.r]) - (p.W / (2.0 * p.r)) * th_dot
            return self._limit(rates)
        return (0.0, 0.0)
