#!/usr/bin/env python3
"""Send a cuMotion goal: joint-space OR a Cartesian hand pose. Plans via the
MoveGroup action (plan_only) so it animates in RViz (/display_planned_path).

Start state = the group's joints at the raised home (must match the planner
cspace -> launch the matching XRDF: CUMOTION_XRDF=<group>.xrdf).

Usage:
  python3 goto.py left_arm_lift --joints 0.7435 0.6 -1.0 0 1.2 0 0 0
  python3 goto.py left_arm_lift --pose 0.30 0.55 0.95           # xyz of the hand
  python3 goto.py left_arm_lift --pose 0.30 0.55 0.95 0 0 0 1   # xyz + quat
  python3 goto.py both_arms     --joints <14 values>
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint,
                             OrientationConstraint, RobotState, PlanningOptions,
                             BoundingVolume)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

L = [f"openarm_left_joint{i}" for i in range(1, 8)]
R = [f"openarm_right_joint{i}" for i in range(1, 8)]
GROUPS = {  # group -> (cspace joint names, tool frame)
    "left_arm":       (L, "openarm_left_hand"),
    "right_arm":      (R, "openarm_right_hand"),
    "both_arms":      (L + R, "openarm_left_hand"),
    "left_arm_lift":  (["lift_joint"] + L, "openarm_left_hand"),
    "right_arm_lift": (["lift_joint"] + R, "openarm_right_hand"),
    "both_arms_lift": (["lift_joint"] + L + R, "openarm_left_hand"),
}
RAISED = {**{f"openarm_left_joint{i}": v for i, v in enumerate([0,-1.4,0,0.6,0,0,0],1)},
          **{f"openarm_right_joint{i}": v for i, v in enumerate([0,1.4,0,0.6,0,0,0],1)},
          "lift_joint": 0.7435}
FRAME = "base_footprint"


class Goto(Node):
    def __init__(self, group):
        super().__init__("goto")
        self.group = group
        self.ac = ActionClient(self, MoveGroup, "/move_action")

    def joint_goal(self, vals):
        joints, _ = GROUPS[self.group]
        c = Constraints()
        for n, v in zip(joints, vals):
            jc = JointConstraint(); jc.joint_name = n; jc.position = float(v)
            jc.tolerance_above = 0.001; jc.tolerance_below = 0.001; jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def pose_goal(self, xyz, quat):
        _, tool = GROUPS[self.group]
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = FRAME; pc.link_name = tool; pc.weight = 1.0
        bv = BoundingVolume(); sp = SolidPrimitive(); sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]; bv.primitives.append(sp)
        bp = Pose(); bp.position.x, bp.position.y, bp.position.z = xyz; bp.orientation.w = 1.0
        bv.primitive_poses.append(bp); pc.constraint_region = bv
        oc = OrientationConstraint()
        oc.header.frame_id = FRAME; oc.link_name = tool; oc.weight = 1.0
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = quat
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.1
        c.position_constraints.append(pc); c.orientation_constraints.append(oc)
        return c

    def run(self, constraints):
        joints, _ = GROUPS[self.group]
        self.ac.wait_for_server(timeout_sec=20.0)
        g = MoveGroup.Goal(); m = g.request
        m.group_name = self.group; m.pipeline_id = "cumotion"
        m.num_planning_attempts = 10; m.allowed_planning_time = 15.0
        m.max_velocity_scaling_factor = 1.0; m.max_acceleration_scaling_factor = 1.0
        rs = RobotState(); js = JointState(); js.name = list(joints)
        js.position = [float(RAISED[n]) for n in joints]; rs.joint_state = js
        m.start_state = rs; m.goal_constraints.append(constraints)
        g.planning_options = PlanningOptions(); g.planning_options.plan_only = True
        for attempt in range(1, 9):
            sf = self.ac.send_goal_async(g); rclpy.spin_until_future_complete(self, sf, timeout_sec=30.0)
            gh = sf.result()
            if gh and gh.accepted:
                rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf, timeout_sec=40.0)
                res = rf.result().result
                if res.error_code.val == 1:
                    print(f"OK (try {attempt}): {len(res.planned_trajectory.joint_trajectory.points)} pts, "
                          f"{res.planning_time:.3f}s -> animating in RViz")
                    return
                code = res.error_code.val
            time.sleep(2)
        print(f"plan failed after retries (last error_code={code})")


def main():
    a = sys.argv
    group = a[1]
    n = Goto(rclpy.init() or group)
    if "--joints" in a:
        vals = [float(x) for x in a[a.index("--joints")+1:]]
        c = n.joint_goal(vals)
    elif "--pose" in a:
        p = [float(x) for x in a[a.index("--pose")+1:]]
        xyz = p[:3]; quat = p[3:7] if len(p) >= 7 else [0.0, 0.0, 0.0, 1.0]
        c = n.pose_goal(xyz, quat)
    else:
        print("specify --joints ... or --pose x y z [qx qy qz qw]"); return
    n.run(c); n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
