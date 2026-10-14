"""SITL fault injection. Usage: python3 check_occluded_sitl.py /path/launch.log.

Run in the same ROS_DOMAIN_ID as catch.launch.py, with evaluate:=true.
Pauses perception only; physical success is independently checked by evaluator.
"""
import json, os, pathlib, re, signal, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

log = pathlib.Path(sys.argv[1])
pid = None
paused = False
finished_at = None
started = time.monotonic()
rclpy.init(); node = Node('catch_occlusion_test')
previous = None
blind_seen = False
final_phase = None
def status(msg):
    global pid, paused, finished_at, previous, blind_seen, final_phase
    row = json.loads(msg.data)
    blind_seen |= bool(row.get('planned_blind'))
    final_phase = row['phase']
    key = (row['phase'], row.get('planned_blind'), row.get('verification'))
    if key != previous:
        print(json.dumps(row), flush=True); previous = key
    if (not paused and row['phase'] == 'INSERT'
            and row.get('approach_remaining_time', 100) < 2.
            and row.get('tracking_error', 100) < .012):
        match = re.search(r'\[object_vision.py-\d+\]: process started with pid \[(\d+)\]', log.read_text())
        if match:
            pid = int(match.group(1))
            assert b'object_vision.py' in pathlib.Path('/proc/{}/cmdline'.format(pid)).read_bytes()
            info = pathlib.Path('/proc/{}/status'.format(pid)).read_text()
            parent = int(re.search(r'^PPid:\s+(\d+)', info, re.M).group(1))
            command = pathlib.Path('/proc/{}/cmdline'.format(parent)).read_bytes()
            assert b'catch.launch.py' in command and b'wholebody_catch' in command, 'Only a catch SITL child may be paused'
            environment = pathlib.Path('/proc/{}/environ'.format(pid)).read_bytes().split(b'\0')
            own_domain = os.environ.get('ROS_DOMAIN_ID', '0')
            child_domain = next((v.split(b'=', 1)[1].decode() for v in environment if v.startswith(b'ROS_DOMAIN_ID=')), '0')
            assert own_domain == child_domain, 'Test and launch must use the same ROS domain'
            os.kill(pid, signal.SIGSTOP); paused = True
            print(json.dumps(dict(vision_paused_pid=pid, stamp=row['stamp'], remaining=row['approach_remaining_time'])), flush=True)
    if row['phase'] in ('SUCCESS', 'ABORT'):
        if finished_at is None: finished_at = time.monotonic()
node.create_subscription(String, '/wholebody_catch/status', status, 10)
try:
    while time.monotonic()-started < 240 and (finished_at is None or time.monotonic()-finished_at < 5.):
        rclpy.spin_once(node, timeout_sec=.2)
finally:
    if paused:
        try: os.kill(pid, signal.SIGCONT)
        except ProcessLookupError: pass
    node.destroy_node(); rclpy.shutdown()
assert paused and blind_seen and final_phase == 'SUCCESS', 'Occlusion mission did not complete; inspect independent evaluator report'
print('PASS: occluded mission completed; require evaluator passed=true for physical acceptance', flush=True)
