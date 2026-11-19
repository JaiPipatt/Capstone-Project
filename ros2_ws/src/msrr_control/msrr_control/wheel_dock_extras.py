"""
OPTIONAL wheel-dock helpers, not used by algo_node (safe to ignore).

  optimize_slsqp        scipy SLSQP cross-check of wheel_dock.optimize
  max_rate              peak wheel rate of a plan (motor-cap check in tests)
  face_alignment_angle  phi_f from the parent face's roll (not wired in yet;
                        phi_f is the wheel_dock.phi_f param for now)
"""
from typing import Optional

import numpy as np

from msrr_control.wheel_dock import (
    basis, cost_J, phidot_nominal, pose_error, QUARTER, quadratic_cost, simulate,
    WheelDockPlan, wrap_mod)


def optimize_slsqp(plan: WheelDockPlan, N: int = 4, steps: int = 1000, maxiter: int = 200):
    """
    Cross-check with scipy SLSQP: min J(eps) s.t. g(T; eps) = g_f.

    phi_d(T) = phi_f (mod pi/2) holds for every eps (psi_j(T) = 0).
    """
    from scipy.optimize import minimize
    A, c = quadratic_cost(plan, N)
    J0 = cost_J(plan)
    gf = np.asarray(plan.gf)

    def fun(e):
        return J0 + c @ e + 0.5 * e @ A @ e

    def jac(e):
        return c + A @ e

    def cons(e):  # x, y only (theta(T) is fixed by the end wheel angles)
        return pose_error(simulate(plan, e, N=N, steps=steps), gf)[:2]

    res = minimize(fun, np.zeros(2 * N), jac=jac, method='SLSQP',
                   constraints=[{'type': 'eq', 'fun': cons}],
                   options={'maxiter': maxiter, 'ftol': 1e-10})
    return res.x, float(fun(res.x)), cons(res.x), res


def max_rate(plan: WheelDockPlan, eps: Optional[np.ndarray] = None, steps: int = 2000) -> float:
    t = np.linspace(0.0, plan.T, steps)
    rates = phidot_nominal(plan.segments, t)
    if eps is not None:
        N = len(eps) // 2
        _, dpsi = basis(t, plan.T, N)
        rates = rates + np.stack([dpsi @ eps[:N], dpsi @ eps[N:]], axis=1)
    return float(np.max(np.abs(rates)))


def face_alignment_angle(parent_face_orientation: float, zero_offset: float = 0.0) -> float:
    """
    Target docking-wheel angle phi_f (defined mod pi/2).

    parent_face_orientation: roll of the parent's face pattern about the
      parent face's OUTWARD normal [rad] (for a parent wheel face: that wheel's
      phi; for the pan (T) face: the pan joint angle; for the fixed B face: 0).
    zero_offset: our wheel angle at which the two patterns line up when the
      parent face orientation is 0 (calibrate on hardware; param override).

    Mating faces look at each other, so their outward normals are opposite and a
    positive roll of the parent's face appears as a negative roll about ours:
      phi_f = zero_offset - parent_face_orientation   (mod pi/2).
    """
    return wrap_mod(zero_offset - parent_face_orientation, QUARTER)
