"""Check the propulsion/allocator contract for the payload-rated local model."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class AirframeTest(unittest.TestCase):
    def test_payload_thrust_reserve_and_hover(self):
        model = ET.parse(ROOT/'models/iris_arm_base/iris_arm_base.sdf')
        config = json.loads((ROOT/'config/flight.json').read_text())
        paths = ['iris_arm_base/iris_arm_base.sdf', 'my/my.sdf', 'mid360/mid360.sdf',
                 'D435i/model.sdf', 'fpv_cam/fpv_cam.sdf']
        mass = sum(float(x.text) for p in paths for x in ET.parse(ROOT/'models'/p).findall('.//mass'))
        motors = [p for p in model.findall('.//plugin') if p.find('motorNumber') is not None]
        self.assertEqual(len(motors), 4)
        maximum = sum(float(p.findtext('motorConstant')) * float(p.findtext('maxRotVelocity'))**2 for p in motors)
        self.assertGreater(maximum/(mass*9.81), 1.8)
        channels = model.findall('.//control_channels/channel')[:4]
        hover = sum(float(p.findtext('motorConstant')) *
                    (config['MPC_THR_HOVER']*float(c.findtext('input_scaling'))+
                     float(c.findtext('zero_position_armed')))**2 for p,c in zip(motors,channels))
        self.assertLess(abs(hover/(mass*9.81)-1.), .1)

    def test_allocator_matches_rotor_geometry(self):
        model = ET.parse(ROOT/'models/iris_arm_base/iris_arm_base.sdf')
        config = json.loads((ROOT/'config/flight.json').read_text())
        for i in range(4):
            xyz = np.fromstring(model.findtext(".//link[@name='rotor_%d']/pose" % i), sep=' ')[:3]
            np.testing.assert_allclose([config['CA_ROTOR%d_PX' % i], config['CA_ROTOR%d_PY' % i]], xyz[:2]*[1,-1])
        parent = ET.parse(ROOT/'models/my/my.sdf')
        base = parent.find(".//include[uri='model://iris_arm_base']")
        self.assertEqual(base.findtext('name'), 'iris_without_GPS')
