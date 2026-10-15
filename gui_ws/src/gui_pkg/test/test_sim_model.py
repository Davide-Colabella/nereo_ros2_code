"""Unit tests of the sim_dinamica model (gui_pkg.sim_model), no ROS."""
import math

import pytest

from gui_pkg.sim_model import Model, Parameters, quaternion_from_yaw

DT = 0.02


def run(model, seconds, cmd, armed=True):
    for _ in range(int(round(seconds / DT))):
        model.step(DT, cmd, armed)


def cmd(surge=0.0, sway=0.0, heave=0.0, yaw=0.0):
    return [surge, sway, heave, 0.0, 0.0, yaw]


def test_values_from_mission_env():
    p = Parameters()
    assert (p.bottom_m, p.v_vert_max_m_s, p.buoyancy_m_s) == (4.5, 0.3, 0.1)


def test_disarmed_rises_by_buoyancy_and_ignores_commands():
    m = Model()
    m.s.depth = 3.0
    run(m, 10.0, cmd(heave=-1.0, surge=1.0, yaw=1.0), armed=False)
    assert m.s.w == pytest.approx(-0.1, abs=1e-3)       # 0.1 m/s up, as mission_env
    assert m.s.depth == pytest.approx(3.0 - 0.1 * (10.0 - 1.0), abs=0.02)
    assert (m.s.x, m.s.yaw) == (0.0, 0.0)


def test_first_order_time_constant():
    m = Model()
    m.s.depth = 2.0
    run(m, 1.0, cmd(surge=1.0))                         # one tau
    assert m.s.u == pytest.approx(0.5 * (1 - math.exp(-1)), rel=1e-6)
    run(m, 9.0, cmd(surge=1.0))
    assert m.s.u == pytest.approx(0.5, rel=1e-3)


def test_vertical_steady_state_and_hover_command():
    m = Model()
    m.s.depth = 2.0
    run(m, 10.0, cmd(heave=-1.0))                      # full down: 0.3 - 0.1
    assert m.s.w == pytest.approx(0.2, abs=1e-3)
    m2 = Model()
    m2.s.depth = 2.0
    run(m2, 20.0, cmd(heave=-0.1 / 0.3))               # thrust cancels buoyancy
    assert m2.s.depth == pytest.approx(2.0, abs=1e-6)


def test_surface_and_seabed_limits():
    m = Model()
    m.s.depth = 0.2
    run(m, 5.0, cmd(heave=1.0))
    assert m.s.depth == 0.0 and m.s.w >= 0.0
    m.s.depth = 4.4
    run(m, 5.0, cmd(heave=-1.0))
    assert m.s.depth == 4.5 and m.altitude() == 0.0


def test_yaw_and_heading_of_the_motion():
    m = Model()
    m.s.depth = 2.0
    run(m, 20.0, cmd(yaw=1.0))
    assert m.s.r == pytest.approx(0.5, rel=1e-3)
    assert -math.pi < m.s.yaw <= math.pi               # wrapped
    m = Model()
    m.s.depth = 2.0
    m.s.yaw = math.pi / 2
    run(m, 10.0, cmd(surge=1.0))                       # heading +y
    assert abs(m.s.x) < 1e-9 and m.s.y > 3.0
    run(m, 10.0, cmd(sway=1.0))                        # body right = world -x
    assert m.s.x < -3.0


def test_non_finite_and_saturated_commands():
    m = Model()
    m.s.depth = 2.0
    run(m, 10.0, cmd(surge=float('nan'), sway=5.0))
    assert m.s.u == pytest.approx(0.0) and m.s.v == pytest.approx(0.5, rel=1e-3)


def test_quaternion_from_yaw():
    w, x, y, z = quaternion_from_yaw(math.pi / 2)
    assert (w, x, y, z) == pytest.approx((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
