"""Geometry for RViz: measured collision solids, not decorative arm lines."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import transform
from .controller import ArmModel
from .corridor import whole_body_radius


def sdf_pose(element):
    values = np.fromstring(element.findtext('pose', '0 0 0 0 0 0'), sep=' ')
    return transform(values[:3], values[3:])


def sdf_shape(collision):
    box = collision.find('geometry/box')
    cylinder = collision.find('geometry/cylinder')
    if box is not None:
        return 'box', np.fromstring(box.findtext('size'), sep=' ')
    if cylinder is not None:
        return 'cylinder', np.array([2*float(cylinder.findtext('radius'))]*2+[float(cylinder.findtext('length'))])
    raise ValueError('Unsupported visual collision primitive')


class VehicleGeometry:
    def __init__(self, share):
        share = Path(share)
        urdf = share/'models/my/arm.urdf'
        self.model = ArmModel(urdf)
        self.radius = whole_body_radius(urdf)
        self.arm = []; self.fixed = []
        for link in ET.parse(urdf).getroot().findall('link'):
            for collision in link.findall('collision'):
                origin = collision.find('origin')
                T = transform(np.fromstring(origin.get('xyz', '0 0 0'), sep=' '),
                              np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
                size = np.fromstring(collision.find('geometry/box').get('size'), sep=' ')
                self.arm.append((link.get('name'), T, size))
        base = ET.parse(share/'models/iris_arm_base/iris_arm_base.sdf')
        for link in base.findall('.//model/link'):
            for collision in link.findall('collision'):
                shape, size = sdf_shape(collision)
                self.fixed.append((link.get('name'), shape, sdf_pose(link)@sdf_pose(collision), size))
        gear = ET.parse(share/'models/my/my.sdf').find(".//link[@name='landing_gear']")
        for collision in gear.findall('collision'):
            shape, size = sdf_shape(collision)
            self.fixed.append(('landing_gear', shape,
                               transform([0, 0, -.5])@sdf_pose(gear)@sdf_pose(collision), size))

    def evaluate(self, position, rotation, joints):
        self.model.evaluate(np.asarray(position), rotation, np.asarray(joints))
        B = self.model.frames['aircraft_base']
        pieces = [(name, shape, B@T, size) for name, shape, T, size in self.fixed]
        pieces += [(name, 'box', self.model.frames[name]@T, size) for name, T, size in self.arm]
        return pieces


def scene_boxes(path):
    boxes = []
    for model in ET.parse(path).findall('.//world/model'):
        if model.findtext('static') != 'true':
            continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                if collision.find('geometry/box') is not None:
                    _, size = sdf_shape(collision)
                    boxes.append((model.get('name'), sdf_pose(model)@sdf_pose(link)@sdf_pose(collision), size))
    return boxes


def box_edges(low, high):
    low, high = np.asarray(low), np.asarray(high)
    points = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        for a in (0, 1):
            for b in (0, 1):
                start = low.copy(); end = low.copy()
                start[others[0]] = end[others[0]] = (low, high)[a][others[0]]
                start[others[1]] = end[others[1]] = (low, high)[b][others[1]]
                end[axis] = high[axis]
                points.extend([start, end])
    return points
