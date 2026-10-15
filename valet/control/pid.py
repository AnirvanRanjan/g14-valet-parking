"""Discrete PID with derivative-on-measurement and anti-windup clamping."""


class PID:
    def __init__(self, kp, ki, kd, dt, out_min=-float("inf"), out_max=float("inf")):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.out_min, self.out_max = out_min, out_max
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_meas = None

    def __call__(self, setpoint, measurement):
        e = setpoint - measurement
        d_meas = 0.0 if self.prev_meas is None else (measurement - self.prev_meas) / self.dt
        self.prev_meas = measurement
        u_unsat = self.kp * e + self.ki * (self.integral + e * self.dt) - self.kd * d_meas
        u = min(max(u_unsat, self.out_min), self.out_max)
        # Conditional integration: only integrate when not pushing further into saturation.
        if u == u_unsat or (u_unsat > self.out_max and e < 0) or (u_unsat < self.out_min and e > 0):
            self.integral += e * self.dt
        return u
