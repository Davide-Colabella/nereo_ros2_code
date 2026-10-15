"""
First-order dynamics of the vehicle for sim_dinamica (no ROS, unit-tested).

Each axis is a velocity that follows its command with a first-order lag:

    dv/dt = (v_max * u + v_bias - v) / tau

with u the normalised command of /nereo_cmd_vel (CommandVelocity, [-1, 1]),
applied only while the firmware is armed. Positions integrate the velocities.

  depth  [m, positive down]   u = cmd_vel[2], heave > 0 ascends; positive
                              buoyancy rises at buoyancy_m_s with no thrust
                              (armed or not); 0 <= depth <= bottom
  yaw    [rad]                u = cmd_vel[5], positive turns to positive yaw
  surge, sway [m/s, body]     u = cmd_vel[0], cmd_vel[1]; x, y in the
                              world frame (yaw from the x axis)

Roll and pitch are not simulated (0).

Values from modello/mission_env.m (the closed-loop environment of the chart):
  bottom_m        4.5  BOTTOM, depth of the seabed
  v_vert_max_m_s  0.3  V_MAX, maximum vertical speed (also vAsc, ascent speed)
  buoyancy_m_s    0.1  rise when disarmed (v = -0.1)
Not in mission_env.m (it moves the vehicle kinematically, without thrusters,
heading or speed), chosen here:
  v_surge_max_m_s 0.5  as comandi.velocita_max_m_s of the bridge, so that the
                       bridge's speed -> command scaling is exact
  v_sway_max_m_s  0.5
  yaw_rate_max    0.5  rad/s
  tau_*           1.0 s (vertical, surge, sway), 0.5 s (yaw)
"""
import math
from dataclasses import dataclass, field


@dataclass
class Parameters:
    bottom_m: float = 4.5
    v_vert_max_m_s: float = 0.3
    buoyancy_m_s: float = 0.1
    v_surge_max_m_s: float = 0.5
    v_sway_max_m_s: float = 0.5
    yaw_rate_max_rad_s: float = 0.5
    tau_vert_s: float = 1.0
    tau_surge_s: float = 1.0
    tau_sway_s: float = 1.0
    tau_yaw_s: float = 0.5


@dataclass
class State:
    depth: float = 0.0     # m, positive down
    yaw: float = 0.0       # rad, in (-pi, pi]
    x: float = 0.0         # m, world
    y: float = 0.0
    w: float = 0.0         # m/s, vertical, positive down
    u: float = 0.0         # m/s, surge (body)
    v: float = 0.0         # m/s, sway (body)
    r: float = 0.0         # rad/s, yaw rate


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class Model:
    p: Parameters = field(default_factory=Parameters)
    s: State = field(default_factory=State)

    def step(self, dt, cmd_vel, armed):
        """Advance by dt seconds with command cmd_vel (6 values), armed or not."""
        p, s = self.p, self.s
        c = [clamp(float(c), -1.0, 1.0) if math.isfinite(c) else 0.0 for c in cmd_vel]
        if not armed:
            c = [0.0] * 6

        def lag(v, target, tau):
            # exact discretisation of the first-order lag
            return target + (v - target) * math.exp(-dt / tau)

        s.w = lag(s.w, -p.v_vert_max_m_s * c[2] - p.buoyancy_m_s, p.tau_vert_s)
        s.u = lag(s.u, p.v_surge_max_m_s * c[0], p.tau_surge_s)
        s.v = lag(s.v, p.v_sway_max_m_s * c[1], p.tau_sway_s)
        s.r = lag(s.r, p.yaw_rate_max_rad_s * c[5], p.tau_yaw_s)

        s.depth += s.w * dt
        if s.depth <= 0.0:             # at the surface: floats
            s.depth = 0.0
            s.w = max(s.w, 0.0)
        elif s.depth >= p.bottom_m:    # on the seabed
            s.depth = p.bottom_m
            s.w = min(s.w, 0.0)
        s.yaw = wrap(s.yaw + s.r * dt)
        s.x += (s.u * math.cos(s.yaw) - s.v * math.sin(s.yaw)) * dt
        s.y += (s.u * math.sin(s.yaw) + s.v * math.cos(s.yaw)) * dt

    def altitude(self):
        """Height over the seabed (m)."""
        return self.p.bottom_m - self.s.depth


def quaternion_from_yaw(yaw):
    """(w, x, y, z) of a pure yaw rotation, ZYX convention as tf2."""
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)
