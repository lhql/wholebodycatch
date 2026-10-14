"""Robot-independent metric tracking. Predictions are never observations."""
import numpy as np


class TargetTracker:
    def __init__(self, measurement_std=.004, acceleration_std=.08, gate=.06):
        self.noise = measurement_std**2
        self.acceleration = acceleration_std**2
        self.gate = gate
        self.x = self.P = self.stamp = None
        self.last_observed = None

    def predict(self, stamp):
        if self.x is None or stamp < self.stamp:
            return None
        dt = min(stamp-self.stamp, 2.)
        F = np.eye(6); F[:3, 3:] = np.eye(3)*dt
        G = np.vstack([np.eye(3)*dt*dt/2, np.eye(3)*dt])
        return F@self.x, F@self.P@F.T+self.acceleration*G@G.T

    def observe(self, stamp, position):
        z = np.asarray(position, float)
        if z.shape != (3,) or not np.isfinite(z).all() or not np.isfinite(stamp):
            return False
        if self.x is None:
            self.x = np.r_[z, np.zeros(3)]
            self.P = np.diag([self.noise]*3+[.03**2]*3)
        else:
            if stamp <= self.stamp:
                return False
            x, P = self.predict(stamp)
            innovation = z-x[:3]
            S = P[:3, :3]+self.noise*np.eye(3)
            if np.linalg.norm(innovation) > self.gate or innovation@np.linalg.solve(S, innovation) > 16.3:
                return False
            K = np.linalg.solve(S, P[:3, :]).T
            self.x = x+K@innovation
            H = np.zeros((3, 6)); H[:, :3] = np.eye(3)
            A = np.eye(6)-K@H
            self.P = A@P@A.T+self.noise*K@K.T
        self.stamp = self.last_observed = stamp
        return True
