import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
ARM_L=[f"openarm_left_joint{i}" for i in range(1,8)]
ARM_R=[f"openarm_right_joint{i}" for i in range(1,8)]
OTHER=["lift_joint","bl_wheel_joint","br_wheel_joint","fr_wheel_joint","fl_wheel_joint",
       "head_servo_joint","openarm_left_finger_joint1","openarm_left_finger_joint2",
       "openarm_right_finger_joint1","openarm_right_finger_joint2"]
LV=[0.0,-1.4,0.0,0.6,0.0,0.0,0.0]; RV=[0.0,1.4,0.0,0.6,0.0,0.0,0.0]
# lift_joint fully raised (range [0,0.7435]); rest (wheels/head/fingers) at 0.
OTHER_POS=[0.7435]+[0.0]*(len(OTHER)-1)
class P(Node):
    def __init__(s):
        super().__init__("spread_js")
        s.pub=s.create_publisher(JointState,"/joint_states",10)
        s.t=s.create_timer(0.1,s.cb)
    def cb(s):
        m=JointState(); m.header.stamp=s.get_clock().now().to_msg()
        m.name=ARM_L+ARM_R+OTHER; m.position=LV+RV+OTHER_POS
        s.pub.publish(m)
rclpy.init(); rclpy.spin(P())
