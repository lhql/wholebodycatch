"""Precomputed straight approach with quintic time scaling."""
import numpy as np


class ApproachPlan:
    def __init__(self, start, end, maximum_speed):
        self.start = np.asarray(start, float).copy()
        self.end = np.asarray(end, float).copy()
        self.duration = max(.6, 1.875*np.linalg.norm(self.end-self.start)/maximum_speed)
        self.elapsed = 0.

    def sample(self, dt, advance=True):
        if advance:
            self.elapsed = min(self.duration, self.elapsed+max(0., dt))
        s = self.elapsed/self.duration
        blend = s**3*(10.+s*(-15.+6.*s))
        return self.start+blend*(self.end-self.start)

    @property
    def remaining_time(self):
        return self.duration-self.elapsed
