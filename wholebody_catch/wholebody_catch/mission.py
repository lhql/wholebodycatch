"""Deterministic grasp state machine, independent of ROS and Gazebo."""
from collections import deque
from dataclasses import dataclass

import numpy as np
from .approach import ApproachPlan


@dataclass
class Settings:
    pregrasp_distance: float = .12
    approach_speed: float = .025
    lift_height: float = .16
    lift_speed: float = .025
    vision_timeout: float = .8
    open_position: float = .018
    closed_position: float = -.018
    grasp_tip_offset: float = .024
    recovery_timeout: float = 10.
    recovery_speed: float = .02
    maximum_recoveries: float = 2.
    blind_distance: float = .045
    blind_timeout: float = 5.
    refine_approach: bool = False

    def __post_init__(self):
        if (not all(np.isfinite(v) for v in vars(self).values())
                or not .06 <= self.pregrasp_distance <= .2
                or not .005 <= self.approach_speed <= .04
                or not .1 <= self.lift_height <= .25
                or not .005 <= self.lift_speed <= .04
                or not .3 <= self.vision_timeout <= 2.
                or not .012 <= self.open_position <= .02
                or not -.019 <= self.closed_position <= -.01
                or not 0 <= self.grasp_tip_offset <= .035
                or not 3 <= self.recovery_timeout <= 20
                or not .005 <= self.recovery_speed <= .03
                or not 0 <= self.maximum_recoveries <= 5
                or not .01 <= self.blind_distance <= .06
                or not 1 <= self.blind_timeout <= 6):
            raise ValueError('Grasp settings outside the supported experiment range')


class CatchMission:
    def __init__(self, settings=None):
        self.cfg = settings or Settings()
        self.phase = 'WAIT_FLIGHT'
        self.reason = ''
        self.goal = None
        self.target = None
        self.gripper = self.cfg.open_position
        self.entered = 0.
        self.last_time = None
        self.since = None
        self.contact_lost = None
        self.observations = deque(maxlen=40)
        self.last_observation = None
        self.visible_path = deque(maxlen=400)
        self.path_stamp = -1.
        self.recoveries = 0
        self.recovery_point = None
        self.plan = None
        self.blind = False
        self.verification = ''

    def enter(self, phase, now):
        self.phase, self.entered, self.since = phase, now, None
        self.contact_lost = None

    def abort(self, reason):
        if self.phase != 'ABORT':
            self.phase, self.reason = 'ABORT', reason
        # Keep the existing finger command; never release a suspended payload.

    def stable(self, condition, now, duration):
        self.since = (now if self.since is None else self.since) if condition else None
        return self.since is not None and now-self.since >= duration

    def observe(self, stamp, point):
        p = np.asarray(point, float)
        if p.shape != (3,) or not np.isfinite(p).all() or not np.isfinite(stamp):
            return
        if self.last_observation is not None and stamp <= self.last_observation[0]:
            return
        self.last_observation = (stamp, p)
        self.observations.append((stamp, p))

    def update(self, now, flight_ready, tip, speed, tool_angle, fingers_open, contacts):
        if self.phase == 'ABORT':
            return
        dt = .02 if self.last_time is None else now-self.last_time
        self.last_time = now
        if dt < 0 or dt > 1.:
            self.abort('simulation clock reset or control gap')
            return
        if self.phase == 'WAIT_FLIGHT':
            if flight_ready:
                self.enter('SEARCH', now)
            return
        if not flight_ready:
            self.abort('flight controller not ready')
            return
        observation = self.last_observation
        fresh = observation is not None and 0 <= now-observation[0] <= self.cfg.vision_timeout
        tip = np.asarray(tip, float)
        planned_blind = (self.phase == 'INSERT' and self.plan is not None
                         and observation is not None and 0 <= now-observation[0] < self.cfg.blind_timeout
                         and np.linalg.norm(self.plan.end-tip) < self.cfg.blind_distance
                         and np.linalg.norm(tip-self.goal) < .02 and tool_angle < .10)
        self.blind = bool(not fresh and planned_blind)
        if self.phase in ('PREGRASP', 'INSERT') and not fresh and not planned_blind:
            if self.recoveries >= self.cfg.maximum_recoveries:
                self.abort('visual recovery budget exhausted')
                return
            self.recoveries += 1
            self.goal = tip.copy()  # Stop advancing immediately.
            self.recovery_point = tip.copy()
            for previous in reversed(self.visible_path):
                distance = np.linalg.norm(previous-tip)
                if .04 <= distance <= .10 and previous[0] < tip[0]:
                    self.recovery_point = previous.copy()
                    break
            self.enter('RECOVER', now)
            return
        if self.phase in ('PREGRASP', 'INSERT') and fresh and observation[0] > self.path_stamp:
            self.visible_path.append(tip.copy())
            self.path_stamp = observation[0]
        if self.phase in ('LIFT', 'VERIFY', 'SUCCESS'):
            self.contact_lost = None if contacts else (now if self.contact_lost is None else self.contact_lost)
            if self.contact_lost is not None and now-self.contact_lost > .35:
                self.abort('bilateral target contact lost')
                return
        limits = dict(SEARCH=35., PREGRASP=45., INSERT=25., CLOSE=9., LIFT=22., VERIFY=10.)
        if self.phase in limits and now-self.entered > limits[self.phase]:
            self.abort(self.phase+' timeout')
            return
        if self.phase == 'RECOVER':
            # Backtrack a short, already traversed segment; never search by
            # advancing into unseen space. Predictions cannot authorize close.
            if now-self.entered > self.cfg.recovery_timeout:
                self.abort('visual recovery timeout')
                return
            if now-self.entered > .4 and np.linalg.norm(tip-self.goal) < .025:
                delta = self.recovery_point-self.goal
                self.goal += delta*min(1., self.cfg.recovery_speed*dt/max(np.linalg.norm(delta), 1e-9))
            associated = fresh and np.linalg.norm(observation[1]-self.target) < .04
            if self.stable(associated, now, .6):
                self.target = observation[1].copy()
                self.goal = self.target-np.array([self.cfg.pregrasp_distance, 0., 0.])
                self.enter('PREGRASP', now)
            return
        if self.phase == 'SEARCH':
            samples = [(t, p) for t, p in self.observations if 0 <= now-t < 1.2]
            if fresh and fingers_open and speed < .025 and len(samples) >= 6 and samples[-1][0]-samples[0][0] >= .6:
                points = np.array([p for _, p in samples])
                center = np.median(points, axis=0)
                if center[2]+self.cfg.lift_height > 2.:
                    return  # The entire lift must remain in the WBC workspace.
                if np.max(np.linalg.norm(points-center, axis=1)) < .012:
                    self.target = center
                    self.goal = center-np.array([self.cfg.pregrasp_distance, 0., 0.])
                    self.enter('PREGRASP', now)
        elif self.phase in ('PREGRASP', 'INSERT'):
            if np.linalg.norm(observation[1]-self.target) > .045:
                self.abort('target moved or visual estimate disagrees with locked target')
                return
            if self.phase == 'PREGRASP':
                if self.stable(np.linalg.norm(tip-self.goal) < .02 and speed < .035
                               and tool_angle < .12 and fingers_open, now, 1.):
                    # Refresh the center from the closer, less noisy view.
                    self.target = observation[1].copy()
                    self.plan = ApproachPlan(self.goal, self.target+[self.cfg.grasp_tip_offset, 0., 0.], self.cfg.approach_speed)
                    self.enter('INSERT', now)
            else:
                # Optional correction when the complete object remains visible.
                # Default: keep the endpoint from the clear pregrasp view;
                # a partly visible surface must not drag the grasp upward.
                if fresh and self.cfg.refine_approach:
                    correction = observation[1]-self.target
                    self.target += correction*min(1., .01*dt/max(np.linalg.norm(correction), 1e-9))
                    self.plan.end = self.target+np.array([self.cfg.grasp_tip_offset, 0., 0.])
                # Pause reference motion when tracking falls behind.
                self.goal = self.plan.sample(dt, np.linalg.norm(tip-self.goal) < .025)
                grasp_tip = self.target+np.array([self.cfg.grasp_tip_offset, 0., 0.])
                if self.stable(self.plan.remaining_time < .05 and np.linalg.norm(tip-grasp_tip) < .006 and speed < .012
                               and tool_angle < .1, now, .6):
                    self.goal = grasp_tip.copy()
                    self.gripper = self.cfg.closed_position
                    self.enter('CLOSE', now)
        elif self.phase == 'CLOSE':
            if self.stable(contacts, now, .4):
                self.enter('LIFT', now)
        elif self.phase == 'LIFT':
            if np.linalg.norm(tip-self.goal) < .03:
                self.goal[2] = min(self.target[2]+self.cfg.lift_height,
                                   self.goal[2]+self.cfg.lift_speed*dt)
            if self.goal[2] >= self.target[2]+self.cfg.lift_height-1e-5 and np.linalg.norm(tip-self.goal) < .025:
                self.enter('VERIFY', now)
        elif self.phase == 'VERIFY':
            # A valid tactile grasp can remain occluded. Vision, when present,
            # must agree; it is not mandatory after the planned closure.
            visual_agrees = (not fresh or (observation[1][2]-self.target[2] > .75*self.cfg.lift_height
                              and np.linalg.norm(observation[1]-tip) < .05))
            carried = (contacts and visual_agrees and tip[2]-self.target[2] > .75*self.cfg.lift_height
                       and np.linalg.norm(tip-self.goal) < .025 and speed < .035)
            if self.stable(carried, now, 1.5):
                self.verification = 'vision_and_contact' if fresh else 'contact_and_tool_motion'
                self.enter('SUCCESS', now)
        # SUCCESS holds the payload; a contact loss still transitions to ABORT.


def touches_target(states, target_name):
    """Accept only collisions with this model, not pedestal/ground/self contact."""
    return any(target_name in collision.split('::')[:-1]
               for state in states
               for collision in (state.collision1_name, state.collision2_name))
