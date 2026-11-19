"""
Helper-free LEFT/RIGHT (wheel-face) docking for a two-wheeled module.

Implements Wolfe, Kutzer, Armand, Chirikjian, "Trajectory Generation and
Steering Optimization for Self-Assembly of a Modular Robotic System",
ICRA 2010:
  Section II-C  nominal plan  pivot1 -> straight1 -> pivot2 -> straight2
  Section II-D  local optimisation by sinusoidal wheel-angle perturbations

Pure Python (numpy/scipy), no ROS.  Optional checks (SLSQP cross-check,
rate check, face-alignment helper) live in wheel_dock_extras.py.

Conventions (paper eq. 2-3)
---------------------------
g = (x, y, theta): (x, y) is the AXLE MIDPOINT, theta is the angle of the
wheel axle to world x, so the heading (driving direction) is
h(theta) = (-sin theta, cos theta).  With our yaw convention (base_link x
forward) theta = yaw - pi/2.  The axle unit vector a(theta) = (cos, sin)
points to the module's RIGHT side.

phi_i is wheel i's angle about its OUTWARD axis:
  wheel 1 = RIGHT wheel  (at  midpoint + W/2 a),  phi_1 = -q_right_wheel_joint
  wheel 2 = LEFT  wheel  (at  midpoint - W/2 a),  phi_2 = +q_left_wheel_joint
(both URDF wheel joints rotate about +y of base_link and a positive joint
velocity drives forward; the left wheel's outward axis is +y, the right
wheel's is -y.)  Driving forward is phi_dot = w(-1, +1); test_wheel_dock.py
checks this against the URDF and diff_drive_controller.

Docking constraint (our change vs the paper)
--------------------------------------------
The EP face has 4-fold symmetry, so the docking wheel's final angle only has
to satisfy phi_d(T) = phi_f (mod pi/2); the paper uses mod pi (eq. 4, 11, 15).
The `modulus` argument selects either.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

Pose = Tuple[float, float, float]
Vec2 = Tuple[float, float]

QUARTER = math.pi / 2  # 4-fold EP face symmetry


# ----------------------------------------------------------------------------
# Parameters and conventions
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class WheelParams:
    r: float = 0.05            # wheel radius [m]
    W: float = 0.09665         # wheel separation [m] (diff_drive_controller value)
    omega_max: float = 0.98    # max |phi_dot| of one wheel [rad/s] (0.049 m/s / r)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def wrap_mod(a: float, modulus: float = QUARTER) -> float:
    """Wrap a to (-modulus/2, modulus/2]: distance of a from the lattice k*modulus."""
    return a - modulus * round(a / modulus)


def heading(theta: float) -> np.ndarray:
    """Driving direction h(theta) = (-sin theta, cos theta)."""
    return np.array([-math.sin(theta), math.cos(theta)])


def axle(theta: float) -> np.ndarray:
    """Axle unit vector a(theta) = (cos theta, sin theta), points to wheel 1 (right)."""
    return np.array([math.cos(theta), math.sin(theta)])


def theta_from_yaw(yaw: float) -> float:
    return wrap_angle(yaw - math.pi / 2)


def yaw_from_theta(theta: float) -> float:
    return wrap_angle(theta + math.pi / 2)


def wheel_of_face(face: str) -> int:
    """Docking wheel index d for a moving face: 'R' -> 1 (right), 'L' -> 2 (left)."""
    return {'R': 1, 'L': 2}[face]


def sigma(d: int) -> int:
    """Sign of phi_d change when driving FORWARD (phi_dot = w(-1, 1))."""
    return -1 if d == 1 else 1


def phi_from_joints(q_left: float, q_right: float) -> Tuple[float, float]:
    """URDF joint positions -> paper wheel angles (phi_1, phi_2)."""
    return -q_right, q_left


def joints_from_phi(phi1: float, phi2: float) -> Tuple[float, float]:
    """Paper wheel angles/rates -> URDF (q_left, q_right)."""
    return phi2, -phi1


def twist_from_phidot(phidot1: float, phidot2: float, p: WheelParams) -> Tuple[float, float]:
    """
    (phi_dot_1, phi_dot_2) -> diff_drive_controller (v, omega_z).

    diff_drive_controller: v = r (qd_L + qd_R) / 2, omega = r (qd_R - qd_L) / W.
    """
    qd_l, qd_r = joints_from_phi(phidot1, phidot2)
    return p.r * (qd_l + qd_r) / 2.0, p.r * (qd_r - qd_l) / p.W


def phidot_from_twist(v: float, omega: float, p: WheelParams) -> Tuple[float, float]:
    """Inverse of twist_from_phidot."""
    qd_l = (v - omega * p.W / 2.0) / p.r
    qd_r = (v + omega * p.W / 2.0) / p.r
    return -qd_r, qd_l


def B(theta: float, p: WheelParams) -> np.ndarray:
    """Eq. 2 input matrix (3x2): g_dot = B phi_dot."""
    s, c = math.sin(theta), math.cos(theta)
    return (p.r / 2.0) * np.array([[s, -s],
                                   [-c, c],
                                   [-2.0 / p.W, -2.0 / p.W]])


def B_ext(theta: float, d: int, p: WheelParams) -> np.ndarray:
    """
    Eq. 3: extended state (x, y, theta, phi_d).

    The last row is just phi_dot_d, i.e. e_d without the r/2 factor (the
    printed paper has r/2 there, which would be a typo: phi_d is an angle).
    """
    last = np.array([[1.0, 0.0]]) if d == 1 else np.array([[0.0, 1.0]])
    return np.vstack([B(theta, p), last])


def wheel_centre(g: Sequence[float], d: int, p: WheelParams) -> np.ndarray:
    """Ground-contact centre of wheel d for axle-midpoint pose g."""
    sgn = 1.0 if d == 1 else -1.0
    return np.array(g[:2], dtype=float) + sgn * (p.W / 2.0) * axle(g[2])


def midpoint_from_wheel(pw: Sequence[float], theta: float, d: int, p: WheelParams) -> np.ndarray:
    sgn = 1.0 if d == 1 else -1.0
    return np.array(pw, dtype=float) - sgn * (p.W / 2.0) * axle(theta)


def axle_pose_from_base(pose: Pose, offset: Vec2) -> Pose:
    """
    base_link pose (x, y, yaw) -> paper axle-midpoint pose (x, y, theta).

    offset: axle midpoint in base_link coordinates.
    """
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * offset[0] - s * offset[1], y + s * offset[0] + c * offset[1],
            theta_from_yaw(yaw))


def base_pose_from_axle(g: Pose, offset: Vec2) -> Pose:
    yaw = yaw_from_theta(g[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return (g[0] - c * offset[0] + s * offset[1], g[1] - s * offset[0] - c * offset[1], yaw)


# ----------------------------------------------------------------------------
# Nominal plan (Section II-C)
# ----------------------------------------------------------------------------
@dataclass
class Segment:
    kind: str                   # 'pivot' or 'straight'
    t0: float                   # start time [s]
    duration: float             # [s]
    phidot: Tuple[float, float]  # constant wheel rates (phi_dot_1, phi_dot_2) [rad/s]
    dtheta: float = 0.0          # pivot: signed change of theta [rad]
    distance: float = 0.0        # straight: wheel-d travel [m]
    direction: int = 1           # straight: +1 forward, -1 backward
    dphi_d: float = 0.0          # change of the docking wheel angle [rad]
    start: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # axle pose at start
    end: Tuple[float, float, float] = (0.0, 0.0, 0.0)    # axle pose at end


@dataclass
class WheelDockPlan:
    d: int                      # docking wheel (1 right, 2 left)
    k: int                      # lattice index in eq. 15
    s: float                    # signed offset of p1 from pf along h_f (eq. 12)
    dir01: int                  # direction of straight 1
    dir1f: int                  # direction of straight 2 (= -sign(s))
    T: float                    # total time [s]
    omega_s: float              # straight wheel rate (eq. 21) [rad/s]
    omega_p: float              # pivot wheel rate = sqrt(2) omega_s
    g0: Pose                    # start axle pose
    gf: Pose                    # final axle pose
    phi0: Tuple[float, float]   # start wheel angles
    phi_f: float                # target docking-wheel angle (mod `modulus`)
    modulus: float
    p0: np.ndarray
    p1: np.ndarray
    pf: np.ndarray
    dtheta01: float
    dtheta1f: float
    length: float               # effective path length L (J = L^2 / (r^2 T))
    J: float                    # eq. 22 for the nominal inputs
    segments: List[Segment] = field(default_factory=list)
    params: WheelParams = field(default_factory=WheelParams)

    def phidot(self, t: np.ndarray) -> np.ndarray:
        """Nominal wheel rates at times t, shape (len(t), 2)."""
        return phidot_nominal(self.segments, np.asarray(t, dtype=float))


def _signed_angle(u: np.ndarray, v: np.ndarray) -> float:
    """Eq. 16-19: |angle| = arccos(u.v), sign = z of u x v."""
    dot = float(np.clip(u @ v, -1.0, 1.0))
    cross = float(u[0] * v[1] - u[1] * v[0])
    ang = math.acos(dot)
    return ang if cross >= 0.0 else -ang


def _build_segments(g0: Pose, d: int, p0, p1, pf, theta_f, dir01, dir1f,
                    dth01, dth1f, D01, D1f, omega_s, omega_p, p: WheelParams) -> List[Segment]:
    """
    Build the timed segments; pivots hold wheel d still and drive only the other wheel.

    NOTE (eq. 8): the paper prints the pivot input as (-delta_d1, delta_d2) * w,
    which would spin wheel d.  We implement the physical intent: wheel d has
    zero rate, the other wheel o turns by -(W/r) dtheta (from theta_dot =
    -(r/W)(phi_dot_1 + phi_dot_2), eq. 2).
    """
    segs: List[Segment] = []
    t = 0.0
    th = g0[2]
    other = 1 if d == 1 else 0  # index of the non-docking wheel in phidot

    def pivot(dth, pw):
        nonlocal t, th
        if abs(dth) < 1e-12:
            return
        rate = [0.0, 0.0]
        rate[other] = -math.copysign(omega_p, dth)
        dur = (p.W / p.r) * abs(dth) / omega_p
        start = tuple(midpoint_from_wheel(pw, th, d, p)) + (th,)
        th = wrap_angle(th + dth)
        end = tuple(midpoint_from_wheel(pw, th, d, p)) + (th,)
        segs.append(Segment('pivot', t, dur, (rate[0], rate[1]), dtheta=dth,
                            start=start, end=end))
        t += dur

    def straight(dist, direction, pa, pb):
        nonlocal t
        if dist < 1e-12:
            return
        rate = (-direction * omega_s, direction * omega_s)
        dur = dist / (p.r * omega_s)
        start = tuple(midpoint_from_wheel(pa, th, d, p)) + (th,)
        end = tuple(midpoint_from_wheel(pb, th, d, p)) + (th,)
        segs.append(Segment('straight', t, dur, rate, distance=dist, direction=direction,
                            dphi_d=sigma(d) * direction * dist / p.r, start=start, end=end))
        t += dur

    pivot(dth01, p0)
    straight(D01, dir01, p0, p1)
    pivot(dth1f, p1)
    straight(D1f, dir1f, p1, pf)
    return segs


def plan_candidates(g0: Pose, phi0: Tuple[float, float], d: int, pf: Sequence[float],
                    theta_f: float, phi_f: float, p: WheelParams, T: float,
                    modulus: float = QUARTER, min_final: Optional[float] = None,
                    s_max: float = 0.5) -> List[WheelDockPlan]:
    """
    Candidate set S over (s, dir01, k) (Section II-C).

    Path of wheel d's centre: p0 -> p1 -> pf with p1 = pf + s h_f (eq. 12),
    |s| >= min_final (default 2r, eq. 13).  Straight 2 is driven backward when
    s > 0 (p1 ahead of pf), so dir1f = -sign(s) and dir1f |s| = -s.

    Feasibility (eq. 15 with k*modulus; sign sigma_d made explicit because the
    sign of wheel d's rotation when driving forward depends on which wheel it is):
        phi_0d + sigma_d (dir01 |p1 - p0| - s) / r = phi_f + k * modulus
    Let q = pf - p0, b = q . h_f, c_k = sigma_d r (phi_f - phi_0d + k*modulus).
    Then dir01 sqrt(|q|^2 + 2 s b + s^2) = c_k + s, and squaring gives the closed
    form s = (c_k^2 - |q|^2) / (2 (b - c_k)); the root is kept if c_k + s has
    the sign of dir01 (rejects the spurious root of the squaring).
    """
    if min_final is None:
        min_final = 2.0 * p.r
    p0 = wheel_centre(g0, d, p)
    pf = np.asarray(pf, dtype=float)
    h0 = heading(g0[2])
    hf = heading(theta_f)
    q = pf - p0
    qq = float(q @ q)
    b = float(q @ hf)
    phi0d = phi0[d - 1]
    sd = sigma(d)

    c_max = math.sqrt(qq) + 2.0 * s_max
    base = phi_f - phi0d
    k_lo = math.floor((-c_max / p.r - base) / modulus) - 1
    k_hi = math.ceil((c_max / p.r - base) / modulus) + 1

    out: List[WheelDockPlan] = []
    for k in range(k_lo, k_hi + 1):
        c = sd * p.r * (base + k * modulus)
        if abs(b - c) < 1e-12:
            continue
        s = (c * c - qq) / (2.0 * (b - c))
        if abs(s) < min_final - 1e-12 or abs(s) > s_max:
            continue
        p1 = pf + s * hf
        D01 = float(np.linalg.norm(p1 - p0))
        lhs = c + s  # must equal dir01 * D01
        if abs(abs(lhs) - D01) > 1e-7:
            continue
        dirs = (1, -1) if D01 < 1e-9 else ((1,) if lhs > 0 else (-1,))
        for dir01 in dirs:
            dir1f = -1 if s > 0 else 1
            if D01 < 1e-9:
                u_travel = h0.copy()
                dth01 = 0.0
            else:
                u_travel = dir01 * (p1 - p0) / D01
                dth01 = _signed_angle(h0, u_travel)
            dth1f = _signed_angle(u_travel, hf)
            D1f = abs(s)
            # Timing (eq. 20-21): |phi_dot| norm is constant, omega_p = sqrt(2) omega_s.
            # T = (D01 + D1f)/(r w_s) + (W/r)(|dth01| + |dth1f|)/(sqrt2 w_s)
            length = D01 + D1f + p.W * (abs(dth01) + abs(dth1f)) / math.sqrt(2.0)
            T_eff = max(T, math.sqrt(2.0) * length / (p.r * p.omega_max))  # motor cap
            omega_s = length / (p.r * T_eff)
            omega_p = math.sqrt(2.0) * omega_s
            gf = tuple(midpoint_from_wheel(pf, theta_f, d, p)) + (wrap_angle(theta_f),)
            segs = _build_segments(g0, d, p0, p1, pf, theta_f, dir01, dir1f, dth01, dth1f,
                                   D01, D1f, omega_s, omega_p, p)
            J = omega_s ** 2 * T_eff  # 1/2 * int (2 w_s^2) dt
            out.append(WheelDockPlan(
                d=d, k=k, s=s, dir01=dir01, dir1f=dir1f, T=T_eff, omega_s=omega_s,
                omega_p=omega_p, g0=tuple(g0), gf=gf, phi0=tuple(phi0), phi_f=phi_f,
                modulus=modulus, p0=p0, p1=p1, pf=pf, dtheta01=dth01, dtheta1f=dth1f,
                length=length, J=J, segments=segs, params=p))
    return out


def plan_nominal(g0: Pose, phi0: Tuple[float, float], d: int, pf: Sequence[float],
                 theta_f: float, phi_f: float, p: WheelParams, T: float,
                 modulus: float = QUARTER, min_final: Optional[float] = None,
                 s_max: float = 0.5) -> WheelDockPlan:
    """Minimum-J candidate (Section II-C).  Raises ValueError if S is empty."""
    cands = plan_candidates(g0, phi0, d, pf, theta_f, phi_f, p, T, modulus, min_final, s_max)
    if not cands:
        raise ValueError('no feasible wheel-dock plan (increase s_max?)')
    return min(cands, key=lambda c: c.J)


# ----------------------------------------------------------------------------
# Inputs, simulation, cost
# ----------------------------------------------------------------------------
def phidot_nominal(segments: Sequence[Segment], t: np.ndarray) -> np.ndarray:
    out = np.zeros((len(t), 2))
    for sg in segments:
        m = (t >= sg.t0) & (t < sg.t0 + sg.duration)
        out[m] = sg.phidot
    return out


def phi_nominal(segments: Sequence[Segment], phi0: Sequence[float], t: np.ndarray) -> np.ndarray:
    out = np.tile(np.asarray(phi0, dtype=float), (len(t), 1))
    for sg in segments:
        tau = np.clip(t - sg.t0, 0.0, sg.duration)
        out += np.outer(tau, sg.phidot)
    return out


def basis(t: np.ndarray, T: float, N: int) -> Tuple[np.ndarray, np.ndarray]:
    """Eq. 23: psi_j(t) = sin(j pi t / T), j = 1..N, and its time derivative."""
    j = np.arange(1, N + 1)
    arg = np.outer(t, j) * math.pi / T
    return np.sin(arg), np.cos(arg) * (j * math.pi / T)


class _Grid:
    """
    Simpson nodes per segment, so no node interval straddles a rate jump.

    t: node times, w: Simpson weights, rates: nominal phi_dot at each node
    (boundary times appear twice, once with each neighbouring segment's rates).
    """

    def __init__(self, segments: Sequence[Segment], T: float, steps: int):
        ts, ws, rs = [], [], []
        for sg in segments:
            n = max(4, 2 * math.ceil(steps * sg.duration / T / 2))
            tt = np.linspace(sg.t0, sg.t0 + sg.duration, n + 1)
            w = np.ones(n + 1)
            w[1:-1:2] = 4.0
            w[2:-1:2] = 2.0
            ts.append(tt)
            ws.append(w * (sg.duration / n) / 3.0)
            rs.append(np.tile(sg.phidot, (n + 1, 1)))
        if not ts:
            ts, ws, rs = [np.array([0.0, T])], [np.zeros(2)], [np.zeros((2, 2))]
        self.t = np.concatenate(ts)
        self.w = np.concatenate(ws)
        self.rates = np.concatenate(rs)


def simulate(plan: WheelDockPlan, eps: Optional[np.ndarray] = None, N: int = 10,
             steps: int = 2000, trajectory: bool = False):
    """
    Integrate eq. 2-3 for Phi(t) = phi_nom(t) + Psi(t) eps (eq. 23).

    eps: (2N,) or a batch (B, 2N); layout [wheel-1 coefficients, wheel-2 coefficients].
    theta and phi_d are linear in the wheel angles, so they are evaluated exactly;
    x, y have right-hand sides independent of x, y, for which RK4 reduces to
    Simpson's rule -- that is what is used, segment by segment.

    Returns g(T) as (3,) / (B, 3); with trajectory=True returns
    (t, G, Phi) with G (len(t), 4) = (x, y, theta, phi_d) and Phi (len(t), 2)
    for a single eps (cumulative trapezoid for the in-between x, y).
    """
    p = plan.params
    T = plan.T
    grid = _Grid(plan.segments, T, steps)
    t = grid.t
    single = eps is None or np.ndim(eps) == 1
    E = np.zeros((1, 2 * N)) if eps is None else np.atleast_2d(np.asarray(eps, dtype=float))
    N = E.shape[1] // 2
    psi, dpsi = basis(t, T, N)
    phin = phi_nominal(plan.segments, plan.phi0, t)          # (P, 2)
    Phi1 = phin[:, 0] + E[:, :N] @ psi.T                     # (B, P)
    Phi2 = phin[:, 1] + E[:, N:] @ psi.T
    dPhi1 = grid.rates[:, 0] + E[:, :N] @ dpsi.T
    dPhi2 = grid.rates[:, 1] + E[:, N:] @ dpsi.T
    th = plan.g0[2] - (p.r / p.W) * (Phi1 + Phi2 - plan.phi0[0] - plan.phi0[1])
    fx = (p.r / 2.0) * np.sin(th) * (dPhi1 - dPhi2)
    fy = -(p.r / 2.0) * np.cos(th) * (dPhi1 - dPhi2)
    x = plan.g0[0] + fx @ grid.w
    y = plan.g0[1] + fy @ grid.w
    gT = np.stack([x, y, np.arctan2(np.sin(th[:, -1]), np.cos(th[:, -1]))], axis=1)
    if trajectory:
        from scipy.integrate import cumulative_trapezoid
        xs = plan.g0[0] + cumulative_trapezoid(fx[0], t, initial=0.0)
        ys = plan.g0[1] + cumulative_trapezoid(fy[0], t, initial=0.0)
        phid = (Phi1 if plan.d == 1 else Phi2)[0]
        G = np.stack([xs, ys, th[0], phid], axis=1)
        return t, G, np.stack([Phi1[0], Phi2[0]], axis=1)
    return gT[0] if single else gT


def cost_J(plan: WheelDockPlan, eps: Optional[np.ndarray] = None, N: int = 10) -> float:
    """
    Eq. 22: J = 1/2 int_0^T (phi_dot_1^2 + phi_dot_2^2) dt, exact for eps.

    J(eps) = J0 + c^T eps + 1/2 eps^T A eps with
      A = int Psi_dot^T Psi_dot dt = diag((j pi / T)^2 T / 2)   (per wheel)
      c_ij = int phi_dot_nom,i psi_dot_j dt = sum_seg rate_i [psi_j(t_end) - psi_j(t_start)]
    """
    J0 = 0.5 * sum((sg.phidot[0] ** 2 + sg.phidot[1] ** 2) * sg.duration for sg in plan.segments)
    if eps is None:
        return J0
    A, c = quadratic_cost(plan, len(eps) // 2)
    eps = np.asarray(eps, dtype=float)
    return float(J0 + c @ eps + 0.5 * eps @ A @ eps)


def quadratic_cost(plan: WheelDockPlan, N: int) -> Tuple[np.ndarray, np.ndarray]:
    T = plan.T
    j = np.arange(1, N + 1)
    a = (j * math.pi / T) ** 2 * T / 2.0
    A = np.diag(np.concatenate([a, a]))
    c = np.zeros(2 * N)
    for sg in plan.segments:
        dpsi = (np.sin(j * math.pi * (sg.t0 + sg.duration) / T)
                - np.sin(j * math.pi * sg.t0 / T))
        c[:N] += sg.phidot[0] * dpsi
        c[N:] += sg.phidot[1] * dpsi
    return A, c


def pose_error(g: Sequence[float], gf: Sequence[float]) -> np.ndarray:
    return np.array([g[0] - gf[0], g[1] - gf[1], wrap_angle(g[2] - gf[2])])


# ----------------------------------------------------------------------------
# Local optimisation (Section II-D)
# ----------------------------------------------------------------------------
@dataclass
class OptResult:
    eps: np.ndarray
    J: float
    J_nominal: float
    iterations: int
    pose_error: np.ndarray
    history: List[float]


def optimize(plan: WheelDockPlan, N: int = 10, max_iter: int = 30, step_norm: float = 2.0,
             tol: float = 1e-6, pos_tol: float = 1e-4, th_tol: float = 1e-3,
             fd_step: float = 1e-6, steps: int = 2000) -> OptResult:
    """
    Sinusoidal-perturbation SQP (eq. 22-30).

    Each iteration: M = dg(T)/d eps by batched forward differences; quadratic
    model of J (exact here, see cost_J); solve
        [[A, M^T], [M, 0]] [delta; lambda] = [-(A eps + c); -(g(T) - g_f)]
    (the paper's system with right-hand side [-c; 0] is the special case
    eps = 0 on the constraint; the residual term pulls the iterate back onto
    g(T) = g_f after the linearisation error of each step).  delta is scaled
    to ||delta|| <= step_norm.  Because psi_j(0) = psi_j(T) = 0 the wheel
    angles at both ends, hence phi_d(T) mod pi/2, are untouched.

    Returns the best iterate that satisfies the pose constraint; the nominal
    (eps = 0) is the fallback, so J never increases.
    """
    A, c = quadratic_cost(plan, N)
    J0 = cost_J(plan)
    gf = np.asarray(plan.gf)
    eps = np.zeros(2 * N)
    best = (J0, eps.copy())
    hist = [J0]
    eye = np.eye(2 * N)
    it = 0
    J_prev = J0
    mu = 1.0
    for it in range(1, max_iter + 1):
        G = simulate(plan, np.vstack([eps, eps + fd_step * eye]), N=N, steps=steps)
        res = pose_error(G[0], gf)[:2]
        # Only x(T), y(T) are constrained: theta = theta_0 - (r/W)(Phi_1 + Phi_2 - ...)
        # (eq. 2) depends on the end wheel angles alone, which psi_j(T) = 0 leaves
        # unchanged, so its row of M is identically zero and would make the KKT
        # matrix singular.
        M = (G[1:, :2] - G[0, :2]).T / fd_step               # 2 x 2N
        K = np.block([[A, M.T], [M, np.zeros((2, 2))]])
        rhs = np.concatenate([-(A @ eps + c), -res])
        try:
            sol = np.linalg.solve(K, rhs)
        except np.linalg.LinAlgError:
            break
        delta, lam = sol[:2 * N], sol[2 * N:]
        nrm = np.linalg.norm(delta)
        if nrm > step_norm:
            delta *= step_norm / nrm
        # Backtracking on the l1 merit J + mu |res|_1 (plain full steps oscillate
        # because the linearised constraint is only first-order accurate).
        mu = max(mu, 2.0 * float(np.max(np.abs(lam))) + 1.0)
        merit0 = cost_J(plan, eps, N) + mu * float(np.sum(np.abs(res)))
        alpha = 1.0
        while True:
            trial = eps + alpha * delta
            err = pose_error(simulate(plan, trial, N=N, steps=steps), gf)
            J = cost_J(plan, trial, N)
            if J + mu * float(np.sum(np.abs(err[:2]))) <= merit0 or alpha < 1e-4:
                break
            alpha *= 0.5
        eps = trial
        hist.append(J)
        feasible = math.hypot(err[0], err[1]) < pos_tol and abs(err[2]) < th_tol
        if feasible and J < best[0]:
            best = (J, eps.copy())
        if feasible and abs(J_prev - J) < tol:
            break
        J_prev = J
    eps_b = best[1]
    err = pose_error(simulate(plan, eps_b, N=N, steps=steps), gf)
    return OptResult(eps_b, best[0], J0, it, err, hist)


# ----------------------------------------------------------------------------
# Docking geometry
# ----------------------------------------------------------------------------
def face_normal(theta: float, d: int) -> np.ndarray:
    """Outward normal of wheel d's face (wheel 1 = right = +a, wheel 2 = left = -a)."""
    return axle(theta) if d == 1 else -axle(theta)


def dock_targets(goal_base: Pose, face: str, p: WheelParams, axle_offset: Vec2,
                 face_gap: float) -> Tuple[int, np.ndarray, float, np.ndarray]:
    """
    Compute the final wheel-d centre pf and axle angle theta_f of a wheel-face docking.

    goal_base: slot pose of the moving module (base_link, world frame, from unfold()).
    pf is the slot's wheel-d centre moved face_gap AWAY from the parent (against
    our face's outward normal), so the face slides in edge-on along l_f without
    scraping; magnetic capture closes the gap.
    Returns (d, pf, theta_f, pf_slot).
    """
    d = wheel_of_face(face)
    g = axle_pose_from_base(goal_base, axle_offset)
    pf_slot = wheel_centre(g, d, p)
    return d, pf_slot - face_gap * face_normal(g[2], d), g[2], pf_slot


def pre_dock_base_pose(pf: Sequence[float], theta_f: float, d: int, s: float,
                       p: WheelParams, axle_offset: Vec2) -> Pose:
    """base_link pose whose wheel-d centre sits at p1 = pf + s h_f (eq. 12), heading h_f."""
    p1 = np.asarray(pf, dtype=float) + s * heading(theta_f)
    g = tuple(midpoint_from_wheel(p1, theta_f, d, p)) + (theta_f,)
    return base_pose_from_axle(g, axle_offset)
