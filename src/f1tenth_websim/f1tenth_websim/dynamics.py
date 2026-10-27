"""Vehicle dynamics: single-track model with the same parameters and low-level
controller as f1tenth_gym, so controllers tuned here behave the same there.

State vector x = [x, y, steer, v, yaw, yaw_rate, slip_angle]
"""
import math
from dataclasses import dataclass, field

G = 9.81


@dataclass
class VehicleParams:
    mu: float = 1.0489
    C_Sf: float = 4.718
    C_Sr: float = 5.4562
    lf: float = 0.15875
    lr: float = 0.17145
    h: float = 0.074
    m: float = 3.74
    I: float = 0.04712
    s_min: float = -0.4189
    s_max: float = 0.4189
    sv_min: float = -3.2
    sv_max: float = 3.2
    v_switch: float = 7.319
    a_max: float = 9.51
    v_min: float = -5.0
    v_max: float = 20.0
    width: float = 0.31
    length: float = 0.58


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    steer: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0
    slip: float = 0.0

    def as_list(self):
        return [self.x, self.y, self.steer, self.v, self.yaw, self.yaw_rate, self.slip]

    @staticmethod
    def from_list(s):
        return VehicleState(*s)


def _clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def steering_constraint(steer, sv, p: VehicleParams):
    if (steer <= p.s_min and sv <= 0) or (steer >= p.s_max and sv >= 0):
        return 0.0
    return _clip(sv, p.sv_min, p.sv_max)


def accl_constraint(vel, accl, p: VehicleParams):
    # Positive acceleration is limited above the switching velocity (motor power limit)
    if vel > p.v_switch:
        pos_limit = p.a_max * p.v_switch / vel
    else:
        pos_limit = p.a_max
    if (vel <= p.v_min and accl <= 0) or (vel >= p.v_max and accl >= 0):
        return 0.0
    return _clip(accl, -p.a_max, pos_limit)


def pid(speed, steer, cur_speed, cur_steer, p: VehicleParams):
    """Same low-level controller as f1tenth_gym: commanded (speed, steer) -> (accel, steer_vel)."""
    steer_diff = steer - cur_steer
    sv = math.copysign(p.sv_max, steer_diff) if abs(steer_diff) > 1e-4 else 0.0

    vel_diff = speed - cur_speed
    if cur_speed > 0.0:
        kp = 10.0 * p.a_max / p.v_max if vel_diff > 0 else 10.0 * p.a_max / (-p.v_min)
    elif cur_speed < 0.0:
        kp = 2.0 * p.a_max / p.v_max if vel_diff > 0 else 10.0 * p.a_max / (-p.v_min)
    else:
        kp = 10.0 * p.a_max / p.v_max
    return kp * vel_diff, sv


def derivatives(s: list, u_sv: float, u_acc: float, p: VehicleParams):
    x, y, steer, v, yaw, r, beta = s
    sv = steering_constraint(steer, u_sv, p)
    acc = accl_constraint(v, u_acc, p)
    lwb = p.lf + p.lr

    if abs(v) < 0.5:
        # Kinematic model at low speed (dynamic model is singular at v = 0)
        dx = v * math.cos(yaw)
        dy = v * math.sin(yaw)
        dyaw = v / lwb * math.tan(steer)
        dr = acc / lwb * math.tan(steer) + v / (lwb * math.cos(steer) ** 2) * sv
        return [dx, dy, sv, acc, dyaw, dr, 0.0]

    mu, m, I, lf, lr, h = p.mu, p.m, p.I, p.lf, p.lr, p.h
    Cf, Cr = p.C_Sf, p.C_Sr
    gf = G * lr - acc * h
    gr = G * lf + acc * h
    dr = (-mu * m / (v * I * (lr + lf)) * (lf ** 2 * Cf * gf + lr ** 2 * Cr * gr) * r
          + mu * m / (I * (lr + lf)) * (lr * Cr * gr - lf * Cf * gf) * beta
          + mu * m / (I * (lr + lf)) * lf * Cf * gf * steer)
    dbeta = ((mu / (v ** 2 * (lr + lf)) * (Cr * gr * lr - Cf * gf * lf) - 1) * r
             - mu / (v * (lr + lf)) * (Cr * gr + Cf * gf) * beta
             + mu / (v * (lr + lf)) * (Cf * gf) * steer)
    return [v * math.cos(yaw + beta), v * math.sin(yaw + beta), sv, acc, r, dr, dbeta]


def step(state: VehicleState, cmd_speed: float, cmd_steer: float, dt: float, p: VehicleParams):
    """Advance one explicit-Euler step (same integrator and dt as f1tenth_gym)."""
    s = state.as_list()
    acc, sv = pid(cmd_speed, cmd_steer, s[3], s[2], p)
    f = derivatives(s, sv, acc, p)
    s = [a + dt * b for a, b in zip(s, f)]
    # Keep steer within physical limits and yaw wrapped
    s[2] = _clip(s[2], p.s_min, p.s_max)
    s[4] = (s[4] + math.pi) % (2 * math.pi) - math.pi
    if abs(s[3]) < 0.5:
        # match gym: in kinematic regime slip and yaw-rate follow the kinematic model
        s[6] = 0.0
        s[5] = s[3] / (p.lf + p.lr) * math.tan(s[2])
    return VehicleState.from_list(s)
