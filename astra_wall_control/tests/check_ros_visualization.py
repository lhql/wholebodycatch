"""Check actual ROS marker messages, trajectory reset and stale feedback display."""
from pathlib import Path
import numpy as np
from astra_wall_control.visualization import FlightVisualization
from astra_wall_control.corridor import SafeCorridor

ROOT = Path(__file__).resolve().parents[1]


def main():
    v = FlightVisualization(ROOT, ROOT/'worlds/corridor.world', history_points=20)
    corridor = SafeCorridor(ROOT/'config/corridor.json', v.geometry.radius)
    corridor.plan(np.array([0., 0., 1.6]), np.array([2.7, 0., 1.6]))
    row = dict(t=0., body=[0., 0., 1.6], body_target=[2.7, 0., 1.6], tip=[.44, 0., 1.4],
               target=[3.14, 0., 1.4], phase='APPROACH', ee_error=2.7, body_error=2.7,
               control_mode='corridor', mode='OFFBOARD', q=[0., -.05, .1, .05, 0., .01, .01],
               body_orientation=[0., 0., .382683432, .923879533],
               corridor=dict(cells=corridor.cells.tolist(), raw_cells=corridor.raw.tolist(),
                             route=[p.tolist() for p in corridor.waypoints], start=corridor.start.tolist(),
                             route_cells=corridor.route_cells, index=0, radius=corridor.radius, margin=.1))
    for i in range(30):
        row['t'] = float(i); v.accept(row)
    assert len(v.body_trace) == 20
    layers = v.build(row, goal_kind='body')
    assert set(layers) == set(v.layers)
    assert len(layers['vehicle']) > 25
    assert any(m.ns == 'planned_corridor_route' and len(m.points) == 4 for m in layers['trajectories'])
    assert any('ACTIVE' in m.text for m in layers['corridors'])
    assert any(m.ns == 'raw_free_space' for m in layers['free_space'])
    for markers in layers.values():
        for m in markers:
            values = [m.pose.position.x, m.pose.position.y, m.pose.position.z,
                      m.scale.x, m.scale.y, m.scale.z, m.color.a]
            assert np.all(np.isfinite(values))
            assert m.header.frame_id == 'map'
    stale = v.build(row, stale=True, goal_kind='body')
    assert any('STALE' in m.text for m in stale['status'])
    robot = next(m for m in stale['vehicle'] if m.ns == 'measured_collision_model')
    assert robot.color.a < .5 and robot.color.r == robot.color.g
    row['t'] = -1.; v.accept(row); assert len(v.body_trace) == 1
    del row['corridor']
    cleared = v.build(row, goal_kind='ee')
    assert len(cleared['corridors']) == 1  # DELETEALL only, no stale corridor remains
    assert 'display-only' in cleared['status'][-1].text
    assert 'WAITING' in v.build(None)['status'][-1].text
    print('Layered geometry, safe corridors, bounded traces, stale display and clearing: PASS')


if __name__ == '__main__':
    main()
