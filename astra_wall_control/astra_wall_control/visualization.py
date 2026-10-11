"""Layered RViz scene, measured robot, safety volumes and bounded trace history."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from .visual_geometry import VehicleGeometry, scene_boxes, box_edges


COLORS = dict(body=(.08, .65, 1., 1.), arm=(1., .62, .08, 1.),
              tip=(.2, 1., .35, 1.), goal=(1., .18, .22, 1.),
              route=(1., .85, .1, 1.), reference=(.9, .35, 1., .9))


def marker(ns, ident, kind, color, position=(0., 0., 0.), scale=(1., 1., 1.)):
    m = Marker(); m.header.frame_id = 'map'; m.ns = ns; m.id = ident; m.type = kind
    m.pose.orientation.w = 1.
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, position)
    m.scale.x, m.scale.y, m.scale.z = map(float, scale)
    m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
    return m


def line(ns, ident, points, color, width=.02, segments=False):
    m = marker(ns, ident, Marker.LINE_LIST if segments else Marker.LINE_STRIP, color,
               scale=(width, 0., 0.))
    for xyz in points:
        p = Point(); p.x, p.y, p.z = map(float, xyz); m.points.append(p)
    return m


class FlightVisualization:
    layers = ('vehicle', 'safety', 'corridors', 'free_space', 'scene', 'goals', 'trajectories', 'status')

    def __init__(self, share, world, history_points=1500):
        self.geometry = VehicleGeometry(share)
        self.preview_geometry = VehicleGeometry(share)
        self.preview_q = None; self.preview_body = None
        self.scene = scene_boxes(world)
        self.body_trace = deque(maxlen=history_points)
        self.tip_trace = deque(maxlen=history_points)
        self.reference_trace = deque(maxlen=history_points)
        self.last_time = None
        self.last_reference = None
        self.route_key = None; self.route_start = None
        self.command_status = 'No user command yet'

    def accept(self, row):
        t = row.get('t', 0.)
        if self.last_time is not None and t < self.last_time:
            self.body_trace.clear(); self.tip_trace.clear(); self.reference_trace.clear()
            self.route_key = None
        if self.last_time != t:
            self.body_trace.append(row['body']); self.tip_trace.append(row['tip'])
            self.reference_trace.append(row.get('body_reference', row['body']))
        self.last_time = t

    def build(self, row, stale=False, goal_kind='ee'):
        layers = {name: [] for name in self.layers}
        # DELETEALL prevents abandoned routes/labels lingering after goal changes.
        for items in layers.values():
            delete = Marker(); delete.header.frame_id = 'map'; delete.action = Marker.DELETEALL; items.append(delete)
        for i, (name, T, size) in enumerate(self.scene):
            obstacle = name == 'corridor_obstacle' or name in ('wall', 'pillar')
            color = (1., .43, .10, .65) if obstacle else (.50, .56, .66, .08)
            m = marker('world_geometry', i, Marker.CUBE, color, T[:3, 3], size)
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
            layers['scene'].append(m)
            edges = [(T[:3, :3]@p+T[:3, 3]) for p in box_edges(-size/2, size/2)]
            layers['scene'].append(line('world_edges', i, edges, color[:3]+(.45,), .012, True))
        if row is None:
            m = marker('status', 0, Marker.TEXT_VIEW_FACING, (1., .8, .2, 1.), (0., 0., 2.8), (.0, .0, .18))
            m.text = 'WAITING\nFlight-telemetry'
            layers['status'].append(m)
            return layers
        p = np.array(row['body']); tip = np.array(row['tip'])
        rotation = Rotation.from_quat(row.get('body_orientation', [0., 0., 0., 1.])).as_matrix()
        q = np.array(row.get('q', [0., -.05, .1, .05, 0., .01, .01]))
        for i, (name, shape, T, size) in enumerate(self.geometry.evaluate(p, rotation, q)):
            arm = name.startswith(('arm_', 'gripper'))
            color = COLORS['arm'] if arm else COLORS['body']
            if name == 'landing_gear': color = (.45, .55, .65, 1.)
            if name.startswith('rotor'): color = (.15, .85, 1., .6)
            if stale: color = (.5, .5, .5, .35)
            m = marker('measured_collision_model', i, Marker.CUBE if shape == 'box' else Marker.CYLINDER,
                       color, T[:3, 3], size)
            quat = Rotation.from_matrix(T[:3, :3]).as_quat()
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, quat)
            layers['vehicle'].append(m)
        tool_R = self.geometry.model.frames['arm_tool_link'][:3, :3]
        for base, origin, R, length in [('body_axes', p, rotation, .45), ('tool_axes', tip, tool_R, .15)]:
            for i, color in enumerate(((1., .15, .15, 1.), (.15, 1., .15, 1.), (.2, .4, 1., 1.))):
                m = line(base, i, [origin, origin+R[:, i]*length], color, .018)
                layers['vehicle'].append(m)
        radius = self.geometry.radius
        layers['safety'].append(marker('whole_body_envelope', 0, Marker.SPHERE,
                                        (.1, 1., .55, .07), p, [2*radius]*3))
        # Three great circles keep the envelope legible through transparent solids.
        for plane in range(3):
            points = []
            for angle in np.linspace(0., 2*np.pi, 81):
                v = np.zeros(3); v[plane] = radius*np.cos(angle); v[(plane+1)%3] = radius*np.sin(angle)
                points.append(p+v)
            layers['safety'].append(line('envelope_outline', plane, points, (.15, 1., .55, .6), .01))
        key = 'target' if goal_kind == 'ee' else 'body_target'
        target = np.array(row[key]); measured = tip if goal_kind == 'ee' else p
        for i, (point, color, size) in enumerate([(tip, COLORS['tip'], .065),
                                                 (target, COLORS['goal'], .10),
                                                 (row['body_target'], COLORS['body'], .06),
                                                 (row.get('body_reference', p), COLORS['reference'], .06)]):
            layers['goals'].append(marker('feedback_and_goals', i, Marker.SPHERE, color, point, [size]*3))
        layers['goals'].append(line('target_error', 0, [measured, target], COLORS['goal'], .012))
        layers['goals'].append(line('body_goal_error', 0, [p, row['body_target']], COLORS['body'], .008))
        if row.get('control_mode') == 'manual':
            preview_q = np.array(row.get('joint_goal', row.get('joint_target', q))) if self.preview_q is None else self.preview_q
            preview_p = np.array(row['body_target'])
            preview_R = Rotation.from_euler('z', row.get('body_target_yaw', 0.)).as_matrix()
            if self.preview_body is not None:
                preview_p = self.preview_body[:3]; preview_R = Rotation.from_quat(self.preview_body[3:]).as_matrix()
            pieces = self.preview_geometry.evaluate(preview_p, preview_R, preview_q)
            for i, (name, shape, T, size) in enumerate(pieces):
                ghost = marker('fk_goal_model', i, Marker.CUBE if shape == 'box' else Marker.CYLINDER,
                               (.8, .3, 1., .22), T[:3, 3], size)
                quat = Rotation.from_matrix(T[:3, :3]).as_quat()
                ghost.pose.orientation.x, ghost.pose.orientation.y, ghost.pose.orientation.z, ghost.pose.orientation.w = map(float, quat)
                layers['goals'].append(ghost)
            T = self.preview_geometry.model.frames['arm_tool_link']
            predicted_tip = T[:3, 3]+T[:3, :3]@np.array([.078, 0., 0.])
            layers['goals'].append(marker('fk_predicted_tip', 0, Marker.SPHERE, (.9, .35, 1., 1.), predicted_tip, [.075]*3))
            label = marker('fk_preview_label', 0, Marker.TEXT_VIEW_FACING, (.9, .6, 1., 1.),
                           predicted_tip+[0., 0., .27], (0., 0., .075)); label.text = 'FK'
            layers['goals'].append(label)
        wholebody = row.get('wholebody')
        if wholebody:
            center = np.asarray(wholebody['center']); bounds = np.asarray(wholebody['bounds'])[:3]
            layers['safety'].append(line('bounded_body_motion', 0, box_edges(center-bounds, center+bounds), (.9, .35, 1., 1.), .012, True))
            layers['goals'].append(marker('body_motion_center', 0, Marker.SPHERE, (.9, .35, 1., .8), center, [.035]*3))
        # At convergence the measured tool and goal coincide. Two labels at
        # the same height become unreadable, so label only the goal; green
        # tool geometry and the dashboard identify the measurement.
        m = marker('goal_labels', 0, Marker.TEXT_VIEW_FACING, (1., .65, .6, 1.),
                   np.asarray(target)+[0., 0., .28], (0., 0., .075))
        m.text = 'EE-goal' if goal_kind == 'ee' else 'Body-goal'
        layers['goals'].append(m)
        for ns, points, color in [('body_trace', self.body_trace, COLORS['body']),
                                   ('ee_trace', self.tip_trace, COLORS['tip']),
                                   ('setpoint_trace', self.reference_trace, COLORS['reference'])]:
            if len(points) > 1:
                layers['trajectories'].append(line(ns, 0, points, color, .018 if ns != 'setpoint_trace' else .008))
        corridor = row.get('corridor'); clearance = None
        if corridor:
            index = corridor['index']; route_cells = corridor.get('route_cells', list(range(len(corridor['cells']))))
            active = route_cells[min(index, len(route_cells)-1)]
            next_cell = route_cells[min(index+1, len(route_cells)-1)]
            for i, (low, high) in enumerate(corridor['cells']):
                color = (.2, 1., .4, .10) if i == active else ((1., .75, .1, .07) if i == next_cell else (.1, .7, 1., .04))
                layers['corridors'].append(marker('safe_body_centres', i, Marker.CUBE, color,
                                                  (np.array(low)+high)/2, np.array(high)-low))
                layers['corridors'].append(line('safe_corridor_edges', i, box_edges(low, high), color[:3]+(.85,), .016, True))
                m = marker('corridor_labels', i, Marker.TEXT_VIEW_FACING, color[:3]+(1.,),
                           [(low[0]+high[0])/2, (low[1]+high[1])/2, high[2]+.15], (0., 0., .13))
                m.text = 'C{}{}'.format(i, ':ACTIVE' if i == active else ':NEXT' if i == next_cell else '')
                layers['corridors'].append(m)
            for i, (low, high) in enumerate(corridor.get('raw_cells', [])):
                layers['free_space'].append(line('raw_free_space', i, box_edges(low, high), (.55, .65, .75, .2), .008, True))
            low, high = np.asarray(corridor['cells'][active])
            clearance = float(min(np.min(p-low), np.min(high-p)))
            # Negative clearance is shown red, even if a stale controller remains visible.
            if clearance < 0.:
                layers['safety'][1].color.r = 1.; layers['safety'][1].color.g = 0.; layers['safety'][1].color.a = .2
            route = corridor['route']; route_key = str(route)
            if route_key != self.route_key:
                self.route_start = corridor.get('start', row['body']); self.route_key = route_key
            layers['trajectories'].append(line('planned_corridor_route', 0, corridor.get('smooth_path', [self.route_start]+route), COLORS['route'], .035))
            layers['trajectories'].append(line('active_route_segment', 0, [p, route[index]], (1., 1., 1., 1.), .018))
            for i, point in enumerate(route):
                layers['corridors'].append(marker('route_waypoints', i, Marker.SPHERE, COLORS['route'], point, [.09]*3))
        mode = row.get('control_mode', 'wholebody')
        status = 'STALE' if stale else row['phase']
        text = '{}/{}\n'.format(mode.upper(), status)
        text += 'EE={:.3f}m(estimated)'.format(row['ee_error'])
        if corridor:
            text += '\nCorridor:{}/{}'.format(index+1, len(corridor['route']))
            if 'trajectory_time' in corridor:
                text += '/'+('PAUSED' if corridor['trajectory_paused'] else 'TRACKING')
        else:
            text += '\nEnvelope:display-only'
        if wholebody:
            text += '\nBody-motion:'+('PAUSED' if wholebody['paused'] else 'ACTIVE' if wholebody['started'] else 'WAITING')
        panel = marker('status_panel', 0, Marker.TEXT_VIEW_FACING,
                       (1., .3, .2, 1.) if stale else (.94, .97, 1., 1.),
                       (1., 0., 3.5) if corridor else p+[0., 0., .95], (0., 0., .085)); panel.text = text
        layers['status'].append(panel)
        return layers
