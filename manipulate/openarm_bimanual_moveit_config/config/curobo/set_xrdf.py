#!/usr/bin/env python3
"""Swap the cuMotion planner's URDF+XRDF at RUNTIME (no relaunch) via the
SetRobotDescription service. The planner reloads its kinematic model: cspace
(planning group) AND collision spheres update live.

Usage:
  set_xrdf.py [--live-idle] [--js-topic=/joint_states] <urdf_path> <xrdf_path> [service_name]
    service_name defaults to /cumotion/set_robot_description
    (confirm the real name with: ros2 service list | grep robot_description)

  --live-idle   Before sending, snapshot /joint_states and overwrite the IDLE
                (non-cspace / locked) joints in the xrdf's default_joint_positions
                with their CURRENT values. Without this the idle arm is locked at
                the xrdf's BAKED home pose, which is a phantom if the real arm
                moved. Active (cspace) joints are left untouched -- their start
                comes from the move_group request, not the xrdf.
  --js-topic=T  Topic to read the live state from (default /joint_states).

Example (reload shrunk left-arm spheres, or switch to both_arms):
  set_xrdf.py /workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf \
              /workspaces/isaac_ros-dev/openarm_cumotion/left_arm_shrunk.xrdf

Example (plan right arm, left arm pinned at its REAL current pose):
  set_xrdf.py --live-idle openarm_bimanual.urdf right_arm_shrunk.xrdf

Run it with the SAME RMW/domain as the planner (e.g. mock demo = FastDDS domain 0:
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp).
NOTE: if you change the GROUP (cspace), send move_group requests for the matching
group; reloading the same group with tweaked spheres is fully seamless.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from isaac_ros_cumotion_interfaces.srv import SetRobotDescription

# Split flags (--foo / --foo=bar) from positional args so existing positional
# usage keeps working.
flags = [a for a in sys.argv[1:] if a.startswith("-")]
pos = [a for a in sys.argv[1:] if not a.startswith("-")]
live_idle = any(f == "--live-idle" for f in flags)
js_topic = next((f.split("=", 1)[1] for f in flags if f.startswith("--js-topic=")),
                "/joint_states")

if len(pos) < 2:
    print(__doc__)
    sys.exit(2)
urdf_path, xrdf_path = pos[0], pos[1]
svc = pos[2] if len(pos) > 2 else "/cumotion/set_robot_description"

rclpy.init()
n = Node("set_xrdf")
cli = n.create_client(SetRobotDescription, svc)
if not cli.wait_for_service(timeout_sec=15.0):
    print(f"ERROR: service '{svc}' not available "
          f"(check: ros2 service list | grep robot_description)")
    sys.exit(1)

urdf_text = open(urdf_path).read()
xrdf_text = open(xrdf_path).read()

if live_idle:
    import yaml
    from sensor_msgs.msg import JointState

    latest = {}

    def _cb(msg):
        for name, p in zip(msg.name, msg.position):
            latest[name] = p

    n.create_subscription(JointState, js_topic, _cb, 10)
    print(f"--live-idle: waiting for {js_topic} ...")
    t0 = time.time()
    while rclpy.ok() and not latest and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not latest:
        print(f"ERROR: no message on {js_topic} within 5s; aborting (not sending).")
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    x = yaml.safe_load(xrdf_text)
    active = set(x.get("cspace", {}).get("joint_names", []))
    djp = x.setdefault("default_joint_positions", {})
    updated = []
    for j in list(djp.keys()):
        if j not in active and j in latest:   # idle/locked joint with a live value
            djp[j] = round(float(latest[j]), 6)
            updated.append(j)
    missing = [j for j in djp if j not in active and j not in latest]
    xrdf_text = yaml.safe_dump(x, sort_keys=False, default_flow_style=None, width=100)
    print(f"--live-idle: pinned {len(updated)} idle joints from {js_topic}: {updated}")
    if missing:
        print(f"--live-idle: WARN no live value for idle joints (kept baked): {missing}")

req = SetRobotDescription.Request()
req.urdf = urdf_text
req.xrdf = xrdf_text
print(f"sending urdf={urdf_path} ({len(req.urdf)} B), xrdf={xrdf_path} ({len(req.xrdf)} B) -> {svc}")
fut = cli.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=60.0)
r = fut.result()
print(f"success={getattr(r, 'success', None)}  message={getattr(r, 'message', '')}")
n.destroy_node()
rclpy.shutdown()
