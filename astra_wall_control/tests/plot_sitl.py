#!/usr/bin/python3
"""Export independent physical evidence from a sitl_modes result (no ROS)."""
import argparse
import json
import gzip
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(source, destination):
    source = Path(source)
    result = json.loads(gzip.decompress(source.read_bytes()) if source.suffix == '.gz' else source.read_text())
    mode = result['summary']['mode']
    rows = [r for r in result['samples'] if r['phase'] in ('APPROACH', 'HOLD')]
    target_key = 'target' if mode == 'wholebody' else 'body_target'
    rows = [r for r in rows if np.allclose(r[target_key], result['summary']['final'][target_key])]
    if mode == 'wholebody':
        rows = [r for r in rows if r.get('wholebody', {}).get('started')]
    if not rows: raise ValueError('No airborne task samples')
    t = np.array([r['t'] for r in rows]); t -= t[0]
    body = np.array([r['truth_body'] for r in rows]); tip = np.array([r['truth_tip'] for r in rows])
    estimate = np.array([r['body'] for r in rows]); q = np.array([r['truth_joints'] for r in rows])
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    ax = axes[0, 0]; ax.plot(body[:, 0], body[:, 1], label='Body (Gazebo)')
    ax.plot(tip[:, 0], tip[:, 1], label='Tool (Gazebo)')
    if mode == 'corridor':
        path = np.array(rows[-1]['corridor']['smooth_path']); ax.plot(path[:, 0], path[:, 1], '--', label='Minimum jerk plan')
        ax.add_patch(plt.Rectangle((1.2, -.7), .4, 1.4, color='orange', alpha=.6, label='Obstacle'))
    ax.set(xlabel='World X (m)', ylabel='World Y (m)', title='Measured trajectory'); ax.axis('equal'); ax.legend(fontsize=8)
    ax = axes[0, 1]
    ax.plot(t, [r['acceptance_error'] for r in rows], label='Independent truth error')
    ax.plot(t, [r['ee_error' if mode == 'wholebody' else 'body_error'] for r in rows], label='Controller estimate')
    ax.axhline(.035 if mode == 'wholebody' else .1, color='red', linestyle='--', label='Acceptance threshold')
    ax.set(xlabel='Task time (s)', ylabel='Position error (m)', title='Tool error' if mode == 'wholebody' else 'Body error'); ax.legend(fontsize=8)
    ax = axes[1, 0]
    if mode == 'wholebody':
        center = np.array(rows[-1]['wholebody']['center']); bounds = rows[-1]['wholebody']['bounds']
        for i, color in enumerate(('tab:red', 'tab:green', 'tab:blue')):
            ax.plot(t, body[:, i]-center[i], color=color, label='XYZ'[i])
            ax.axhline(bounds[i], color=color, linestyle=':', alpha=.5)
            ax.axhline(-bounds[i], color=color, linestyle=':', alpha=.5)
        ax.set(title='Actual body offset / configured limits', ylabel='Offset (m)')
    else:
        for i, label in enumerate('XYZ'): ax.plot(t, estimate[:, i]-body[:, i], label=label)
        ax.set(title='Localization error (estimate minus truth)', ylabel='Error (m)')
    ax.set_xlabel('Task time (s)'); ax.legend(fontsize=8)
    ax = axes[1, 1]
    if mode == 'corridor':
        ax.plot(t, [r.get('truth_obstacle_clearance', np.nan) for r in rows], label='Full-body sphere to obstacle')
        ax.axhline(0., color='red', linestyle='--'); ax.set(ylabel='Clearance (m)', title='Independent obstacle clearance')
    else:
        for i, label in [(0, 'Arm yaw'), (1, 'Shoulder'), (3, 'Wrist'), (4, 'Roll')]: ax.plot(t, q[:, i], label=label)
        ax.set(ylabel='Joint angle (rad)', title='Measured arm compensation' if mode == 'wholebody' else 'Measured arm command')
    ax.set_xlabel('Task time (s)'); ax.legend(fontsize=8)
    for ax in axes.flat: ax.grid(alpha=.25)
    fig.suptitle('Astra '+mode+' / independent Gazebo measurements / '+('PASS' if result['summary']['passed'] else 'NOT PASSED'))
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=150); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('input'); parser.add_argument('output')
    args = parser.parse_args(); plot(args.input, args.output)
