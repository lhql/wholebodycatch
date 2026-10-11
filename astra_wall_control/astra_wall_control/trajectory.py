"""Corridor-constrained minimum-jerk quintic Bezier trajectories.

This implements the minimum-control-effort idea, not the external MINCO library.
All control points stay in their convex safe cell, guaranteeing continuous-time
position containment. Shared boundary derivatives give C2 continuity. A uniform
time stretch enforces exact polynomial velocity/acceleration extrema limits.
"""
import math
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize
from numpy.polynomial.polynomial import polyder, polyval, polyroots


class MinimumJerkTrajectory:
    def __init__(self, points, cells, speed=.12, acceleration=.18):
        points = np.asarray(points, float); cells = np.asarray(cells, float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or
                cells.shape != (len(points)-1, 2, 3) or
                not np.all(np.isfinite(points)) or not np.all(np.isfinite(cells)) or
                not np.isfinite(speed) or not np.isfinite(acceleration) or speed <= 0 or acceleration <= 0):
            raise ValueError('Invalid minimum-jerk trajectory input')
        self.n = len(points)-1
        self.durations = np.maximum(1., np.max(np.abs(np.diff(points, axis=0)), axis=1)/speed)
        for i, cell in enumerate(cells):
            if np.any(points[i:i+2] < cell[0]-1e-8) or np.any(points[i:i+2] > cell[1]+1e-8):
                raise ValueError('Trajectory endpoint outside its safe corridor')
        D = np.array([[-1., 3., -3., 1., 0., 0.], [0., -1., 3., -3., 1., 0.], [0., 0., -1., 3., -3., 1.]])
        gram = np.array([[math.comb(2, i)*math.comb(2, j)/(5.*math.comb(4, i+j)) for j in range(3)] for i in range(3)])
        H = np.zeros((6*self.n, 6*self.n))
        for i, T in enumerate(self.durations):
            H[6*i:6*i+6, 6*i:6*i+6] = 3600./T**5*(D.T@gram@D)
        H /= max(np.max(np.abs(H)), 1e-12)
        H += np.eye(len(H))*1e-10
        equations = []; values = []
        def equation(terms, value):
            row = np.zeros(6*self.n)
            for index, coefficient in terms: row[index] += coefficient
            equations.append(row); values.append(np.asarray(value)*np.ones(3))
        for i in range(self.n):
            equation([(6*i, 1.)], points[i]); equation([(6*i+5, 1.)], points[i+1])
        equation([(1, 1.), (0, -1.)], 0.)
        equation([(2, 1.), (1, -2.), (0, 1.)], 0.)
        end = 6*self.n-1
        equation([(end, 1.), (end-1, -1.)], 0.)
        equation([(end, 1.), (end-1, -2.), (end-2, 1.)], 0.)
        for i in range(self.n-1):
            a = 6*i+5; b = 6*(i+1); T = self.durations[i]; U = self.durations[i+1]
            equation([(a, 1/T), (a-1, -1/T), (b+1, -1/U), (b, 1/U)], 0.)
            equation([(a, 1/T**2), (a-1, -2/T**2), (a-2, 1/T**2),
                      (b+2, -1/U**2), (b+1, 2/U**2), (b, -1/U**2)], 0.)
        E, values = np.array(equations), np.array(values)
        initial = np.array([[points[i]]*3+[points[i+1]]*3 for i in range(self.n)])
        controls = np.empty_like(initial)
        for axis in range(3):
            low = np.repeat(cells[:, 0, axis], 6); high = np.repeat(cells[:, 1, axis], 6)
            result = minimize(lambda x: .5*x@H@x, initial[:, :, axis].ravel(), jac=lambda x: H@x,
                              bounds=Bounds(low, high), constraints=[LinearConstraint(E, values[:, axis], values[:, axis])],
                              method='SLSQP', options={'maxiter': 180, 'ftol': 1e-10})
            if not result.success or np.max(np.abs(E@result.x-values[:, axis])) > 1e-6:
                raise ValueError('Minimum-jerk corridor optimization failed: '+result.message)
            controls[:, :, axis] = result.x.reshape(self.n, 6)
        self.controls = controls
        self.coefficients = np.zeros_like(controls)
        for k in range(6):
            for i in range(k+1):
                self.coefficients[:, k] += math.comb(5, i)*math.comb(5-i, k-i)*(-1)**(k-i)*controls[:, i]
        vmax, amax = self.extrema()
        self.durations *= max(1., vmax/speed, np.sqrt(amax/acceleration))*1.00001
        self.ends = np.cumsum(self.durations)
        self.duration = float(self.ends[-1])

    def extrema(self):
        maxima = [0., 0.]
        for i, coefficients in enumerate(self.coefficients):
            for axis in range(3):
                for order in (1, 2):
                    derivative = polyder(coefficients[:, axis], order)
                    roots = polyroots(polyder(derivative))
                    candidates = [0., 1.]+[float(r.real) for r in roots if abs(r.imag) < 1e-8 and 0. < r.real < 1.]
                    peak = max(abs(polyval(t, derivative)) for t in candidates)/self.durations[i]**order
                    maxima[order-1] = max(maxima[order-1], peak)
        return maxima

    def sample(self, elapsed):
        elapsed = float(np.clip(elapsed, 0., self.duration))
        index = min(int(np.searchsorted(self.ends, elapsed, side='right')), self.n-1)
        start = 0. if index == 0 else self.ends[index-1]
        u = (elapsed-start)/self.durations[index]
        coeff = self.coefficients[index]
        return tuple(np.array([polyval(u, polyder(coeff[:, k], order)) for k in range(3)])/self.durations[index]**order
                     for order in range(3))+(index,)

    def path(self, count=180):
        return [self.sample(t)[0].tolist() for t in np.linspace(0., self.duration, count)]
