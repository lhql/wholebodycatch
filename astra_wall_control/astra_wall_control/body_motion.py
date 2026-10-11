"""Small bounded body motion while the tool remains fixed in the map frame."""
import numpy as np


class BoundedBodyMotion:
    def __init__(self, bounds=(.06, .06, .05, .20), amplitude=(.025, .035, .02, .08), period=32.):
        self.bounds = np.asarray(bounds, float); self.amplitude = np.asarray(amplitude, float)
        self.period = float(period); self.offset = np.zeros(4)
        self.start_offset = np.zeros(4)
        if (self.bounds.shape != (4,) or self.amplitude.shape != (4,) or
                not np.all(np.isfinite(self.bounds)) or not np.all(np.isfinite(self.amplitude)) or
                np.any(self.bounds <= 0.) or np.any(self.bounds > [.12, .12, .10, .35]) or
                np.any(self.amplitude < 0.) or np.any(self.amplitude > self.bounds) or
                not np.isfinite(period) or period < 8.):
            raise ValueError('Invalid bounded whole-body motion parameters')

    def set_offset(self, offset, initial_reference=None):
        offset = np.asarray(offset, float)
        if offset.shape != (4,) or not np.all(np.isfinite(offset)) or np.any(np.abs(offset) > self.bounds+1e-9):
            raise ValueError('Requested body reference is outside the whole-body motion box')
        self.offset = offset.copy()
        self.start_offset = np.zeros(4) if initial_reference is None else np.clip(initial_reference, -self.bounds, self.bounds)

    def sample(self, elapsed):
        t = max(0., float(elapsed)); ramp_time = 4.
        u = min(1., t/ramp_time)
        ramp = 10*u**3-15*u**4+6*u**5
        ramp_rate = (30*u**2-60*u**3+30*u**4)/ramp_time if u < 1. else 0.
        frequency = 2*np.pi/self.period*np.array([1., 1.2, .8, 1.])
        amplitude = np.minimum(self.amplitude, self.bounds-np.abs(self.offset))
        oscillation = amplitude*np.sin(frequency*t)
        position = (1-ramp)*self.start_offset+ramp*(self.offset+oscillation)
        velocity = ramp_rate*(self.offset+oscillation-self.start_offset)+ramp*amplitude*frequency*np.cos(frequency*t)
        return position, velocity
