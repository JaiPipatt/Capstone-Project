import math
import os
import re

import numpy as np
import pytest

from msrr_control.wheel_dock import (
    axle_pose_from_base, B, B_ext, base_pose_from_axle, cost_J, heading, joints_from_phi,
    optimize, phi_from_joints, phidot_from_twist, plan_nominal, pose_error, QUARTER,
    simulate, twist_from_phidot, wheel_centre, WheelParams, wrap_mod)
from msrr_control.wheel_dock_extras import face_alignment_angle, max_rate, optimize_slsqp

P = WheelParams()
DESC = os.path.join(os.path.dirname(__file__), '..', '..', 'my_robot_description')


def random_cases(n, seed=0):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        g0 = (rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-math.pi, math.pi))
        yield dict(g0=g0, phi0=tuple(rng.uniform(-6, 6, 2)), d=int(rng.integers(1, 3)),
                   pf=tuple(rng.uniform(-0.1, 0.1, 2)), theta_f=rng.uniform(-math.pi, math.pi),
                   phi_f=rng.uniform(0, QUARTER))


# ------------------------------------------------------------------ conventions
def test_straight_moves_along_heading_without_rotation():
    for th in np.linspace(-3, 3, 7):
        gd = B(th, P) @ np.array([-1.0, 1.0])
        assert gd[2] == pytest.approx(0.0)
        assert gd[:2] == pytest.approx(P.r * heading(th))


def test_equal_wheel_rates_spin_in_place():
    gd = B(0.4, P) @ np.array([1.0, 1.0])
    assert gd[:2] == pytest.approx([0.0, 0.0])
    assert gd[2] == pytest.approx(-2 * P.r / P.W)


@pytest.mark.parametrize('d', [1, 2])
def test_pivot_holds_wheel_d_still(d):
    # wheel d stationary, other wheel driven: wheel d's centre has zero velocity
    th = 0.7
    rates = np.array([0.0, 1.0]) if d == 1 else np.array([1.0, 0.0])
    gd = B_ext(th, d, P) @ rates
    centre_vel = gd[:2] + (1 if d == 1 else -1) * (P.W / 2) * gd[2] * np.array(
        [-math.sin(th), math.cos(th)])
    assert centre_vel == pytest.approx([0.0, 0.0], abs=1e-12)
    assert gd[3] == 0.0
    assert gd[2] != 0.0


def test_diff_drive_conversion():
    v, w = twist_from_phidot(-1.0, 1.0, P)       # straight forward
    assert (v, w) == pytest.approx((P.r, 0.0))
    v, w = twist_from_phidot(1.0, 1.0, P)        # pure spin, clockwise (theta_dot < 0)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(B(0.0, P)[2] @ [1.0, 1.0])
    v, w = twist_from_phidot(0.0, 1.0, P)        # pivot about the right wheel
    assert w == pytest.approx(B(0.0, P)[2] @ [0.0, 1.0])
    assert phidot_from_twist(*twist_from_phidot(0.3, -0.7, P), P) == pytest.approx((0.3, -0.7))
    assert phi_from_joints(*joints_from_phi(0.2, 0.5)) == pytest.approx((0.2, 0.5))


def _xacro_prop(text, name):
    return float(re.search(rf'name="{name}" value="([-0-9.e]+)"', text).group(1))


@pytest.mark.skipif(not os.path.isdir(DESC), reason='my_robot_description not next to us')
def test_conventions_match_urdf_and_controller():
    core = open(os.path.join(DESC, 'urdf', 'my_robot_core.xacro')).read()
    yaml = open(os.path.join(DESC, 'config', 'controllers.yaml')).read()
    axis = re.search(r'<joint name="\$\{side\}_wheel_joint".*?<axis xyz="([^"]+)"', core, re.S)
    a = np.array([float(v) for v in axis.group(1).split()])
    y_left, y_right = _xacro_prop(core, 'left_wheel_y'), _xacro_prop(core, 'right_wheel_y')
    assert y_left > 0 > y_right
    # no-slip: hub velocity for joint rate qd about axis a is qd * r * (a x z)
    fwd = np.cross(a, [0, 0, 1])
    assert fwd == pytest.approx([1, 0, 0])  # positive joint rate drives forward
    # phi_i is about the OUTWARD axis: +y for the left wheel, -y for the right
    sign_left = float(a @ [0, 1, 0])
    sign_right = float(a @ [0, -1, 0])
    assert phi_from_joints(1.0, 1.0) == pytest.approx((sign_right, sign_left))
    r = float(re.search(r'wheel_radius:\s*([0-9.]+)', yaml).group(1))
    W = float(re.search(r'wheel_separation:\s*([0-9.]+)', yaml).group(1))
    assert (r, W) == pytest.approx((P.r, P.W))


def test_axle_pose_round_trip():
    pose = (0.1, -0.2, 0.9)
    g = axle_pose_from_base(pose, (-0.004905, 0.0))
    assert base_pose_from_axle(g, (-0.004905, 0.0)) == pytest.approx(pose)
    assert g[2] == pytest.approx(0.9 - math.pi / 2)


def test_face_alignment_angle():
    assert face_alignment_angle(0.0) == pytest.approx(0.0)
    assert wrap_mod(face_alignment_angle(0.3) + 0.3) == pytest.approx(0.0)
    assert wrap_mod(face_alignment_angle(0.3, 0.1) - (0.1 - 0.3)) == pytest.approx(0.0)


# ------------------------------------------------------------------ nominal plan
def check_plan(pl, d, pf, theta_f, phi_f, modulus=QUARTER):
    t, G, _ = simulate(pl, trajectory=True)
    gT = simulate(pl)
    assert np.linalg.norm(wheel_centre(gT, d, P) - np.array(pf)) < 1e-3
    assert abs(math.degrees(pose_error(gT, pl.gf)[2])) < 0.5
    assert abs(wrap_mod(G[-1, 3] - phi_f, modulus)) < 1e-3


def test_nominal_plan_reaches_goal_for_random_starts():
    n = 0
    for c in random_cases(60):
        pl = plan_nominal(c['g0'], c['phi0'], c['d'], c['pf'], c['theta_f'], c['phi_f'], P, T=30.0)
        check_plan(pl, c['d'], c['pf'], c['theta_f'], c['phi_f'])
        assert abs(pl.s) >= 2 * P.r - 1e-9            # final straight >= 2r (eq. 13)
        assert pl.segments[-1].kind == 'straight'
        assert pl.segments[-1].distance >= 2 * P.r - 1e-9
        assert max_rate(pl) <= P.omega_max + 1e-9      # motor cap
        assert cost_J(pl) == pytest.approx(pl.J)
        n += 1
    assert n >= 50


def test_pivot_segments_keep_docking_wheel_still():
    for c in random_cases(20, seed=3):
        pl = plan_nominal(c['g0'], c['phi0'], c['d'], c['pf'], c['theta_f'], c['phi_f'], P, T=30.0)
        for sg in pl.segments:
            if sg.kind == 'pivot':
                assert sg.phidot[c['d'] - 1] == 0.0
                assert np.linalg.norm(wheel_centre(sg.start, c['d'], P)
                                      - wheel_centre(sg.end, c['d'], P)) < 1e-9


def test_final_straight_is_along_final_heading():
    c = next(random_cases(1, seed=7))
    pl = plan_nominal(c['g0'], c['phi0'], c['d'], c['pf'], c['theta_f'], c['phi_f'], P, T=30.0)
    last = pl.segments[-1]
    assert last.start[2] == pytest.approx(pl.gf[2])
    assert last.end == pytest.approx(pl.gf)


# ------------------------------------------------------------------ optimisation
def test_optimization_never_increases_J_and_keeps_constraints():
    for c in random_cases(15, seed=11):
        pl = plan_nominal(c['g0'], c['phi0'], c['d'], c['pf'], c['theta_f'], c['phi_f'], P, T=30.0)
        res = optimize(pl, N=10)
        assert res.J <= res.J_nominal + 1e-12
        assert cost_J(pl, res.eps) == pytest.approx(res.J)
        gT = simulate(pl, res.eps)
        assert np.linalg.norm(wheel_centre(gT, c['d'], P) - np.array(c['pf'])) < 1e-3
        assert abs(pose_error(gT, pl.gf)[2]) < math.radians(0.5)
        _, G, _ = simulate(pl, res.eps, N=10, trajectory=True)
        assert abs(wrap_mod(G[-1, 3] - c['phi_f'])) < 1e-3


def test_sqp_matches_slsqp():
    c = next(random_cases(1, seed=1))
    pl = plan_nominal(c['g0'], c['phi0'], c['d'], c['pf'], c['theta_f'], c['phi_f'], P, T=30.0)
    res = optimize(pl, N=6, max_iter=60)
    _, J_slsqp, con, _ = optimize_slsqp(pl, N=6)
    assert np.abs(con).max() < 1e-6
    assert res.J == pytest.approx(J_slsqp, rel=1e-2)


# ------------------------------------------------------------------ mod pi/2 vs mod pi
def test_quarter_turn_symmetry_never_costs_more():
    ratios = []
    for c in random_cases(50, seed=5):
        args = (c['g0'], c['phi0'], c['d'], c['pf'], c['theta_f'], c['phi_f'], P)
        J_q = plan_nominal(*args, T=30.0, modulus=QUARTER).J
        J_h = plan_nominal(*args, T=30.0, modulus=math.pi).J
        assert J_q <= J_h + 1e-12
        ratios.append(1 - J_q / J_h)
    print(f'\nmod pi/2 vs mod pi: mean J reduction {100 * np.mean(ratios):.1f}% '
          f'(max {100 * np.max(ratios):.1f}%) over {len(ratios)} cases')


# ------------------------------------------------------------------ execution
def test_executor_closed_loop_with_slip():
    from msrr_control.wheel_dock_exec import ExecConfig, WheelDockExecutor
    rng = np.random.default_rng(2)
    for c in random_cases(5, seed=13):
        pl = plan_nominal(c['g0'], c['phi0'], c['d'], c['pf'], c['theta_f'], c['phi_f'], P,
                          T=30.0, min_final=0.16)
        ex = WheelDockExecutor(pl, ExecConfig())
        g, phi, rate = np.array(c['g0']), np.array(c['phi0']), np.zeros(2)
        for _ in range(4000):
            if ex.done:
                break
            cmd = np.array(ex.update(tuple(g), tuple(phi)))
            for _ in range(10):  # 10 Hz control, 100 Hz plant with lag and 5 % rate noise
                rate += (cmd * (1 + 0.05 * rng.standard_normal(2)) - rate) * 0.2
                g = g + B(g[2], P) @ rate * 0.01
                phi = phi + rate * 0.01
        assert ex.done
        assert np.linalg.norm(wheel_centre(g, c['d'], P) - np.array(c['pf'])) < 2e-3
        assert abs(wrap_mod(phi[c['d'] - 1] - c['phi_f'])) < 5e-3
