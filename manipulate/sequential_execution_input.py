import rclpy
# import time  <--- Removed, we don't need time.sleep anymore
from rclpy.action import ActionClient
from rclpy.node import Node
import sys

# Import the action types
from my_robot_interfaces.action import SetJointPosition
from control_msgs.action import GripperCommand

class SequentialCommander(Node):

    def __init__(self):
        super().__init__('sequential_commander')
        
        # 1. Client for Arm Joints
        self.arm_client = ActionClient(
            self, 
            SetJointPosition, 
            '/set_joint_position'
        )
        
        # 2. Client for Gripper
        self.gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/left_hand_controller/gripper_cmd'
        )

    def send_arm_goal(self, group_name, positions):
        goal_msg = SetJointPosition.Goal()
        goal_msg.group_name = group_name
        goal_msg.joint_positions = positions

        self.get_logger().info(f'Sending Arm Goal: {positions}...')
        self.arm_client.wait_for_server()

        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Arm Result: {result.success}')
        return result.success

    def send_gripper_goal(self, position, max_effort=0.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f'Sending Gripper Goal: {position}...')
        self.gripper_client.wait_for_server()

        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info('Gripper Action Finished.')
        return True

def wait_for_step(step_name):
    """
    Pauses the script and waits for the user to press ENTER.
    """
    print("\n" + "="*40)
    print(f" NEXT STEP: {step_name}")
    print(" Press [ENTER] to execute...")
    print("="*40)
    input()  # Blocking call: waits for keyboard input

def main(args=None):
    rclpy.init(args=args)
    commander = SequentialCommander()

    try:
        # --- STEP 1: Initial Gripper Close ---
        wait_for_step("Close Gripper (0.0)")
        if not commander.send_gripper_goal(0.0):
            raise Exception("Step 1 (Gripper) Failed")

        # --- STEP 2: Move Arm to First Position ---
        wait_for_step("Move Arm to Home [0.663, ...]")
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, 0.0, 0.0]):
            raise Exception("Step 2 (Arm) Failed")

        # --- STEP 3: Move Arm to Second Position ---
        wait_for_step("Move Arm to Pose 2")
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, -1.57, 0.523]):
            raise Exception("Step 3 (Arm) Failed")

        # --- STEP 4: Move Arm to Third Position ---
        wait_for_step("Move Arm to Pose 3")
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, -1.54, 0.0, 0.0]):
            raise Exception("Step 4 (Arm) Failed")

        # --- STEP 5: Close Gripper (Repeated?) ---
        wait_for_step("Close Gripper (-12.5)")
        if not commander.send_gripper_goal(-12.5):
            raise Exception("Step 5 (Gripper) Failed")

        # --- STEP 6: Open Gripper ---
        wait_for_step("Open Gripper (0.2)")
        if not commander.send_gripper_goal(0.2):
            raise Exception("Step 6 (Gripper) Failed")

        # --- STEP 7: Move Arm Back ---
        wait_for_step("Move Arm Back")
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, -1.57, 0.0]):
            raise Exception("Step 7 (Arm) Failed")

        # --- STEP 8: Move Arm to Previous ---
        wait_for_step("Move Arm to Pose 3 Again")
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, -1.54, 0.0, 0.0]):
            raise Exception("Step 8 (Arm) Failed")

        # --- STEP 9: Close Gripper ---
        wait_for_step("Close Gripper (-12.5)")
        if not commander.send_gripper_goal(-12.5):
            raise Exception("Step 9 (Gripper) Failed")

        # --- STEP 10: Open Gripper ---
        wait_for_step("Open Gripper (0.2)")
        if not commander.send_gripper_goal(0.0):
            raise Exception("Step 10 (Gripper) Failed")

        # --- STEP 11: Final Move ---
        wait_for_step("Final Arm Move")
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, -1.57, 0.523]):
            raise Exception("Step 11 (Arm) Failed")

        commander.get_logger().info("All steps completed successfully!")

    except KeyboardInterrupt:
        # Handles Ctrl+C gracefully so you can exit while waiting for input
        print("\nUser Aborted (Ctrl+C).")
        
    except Exception as e:
        commander.get_logger().error(f"Sequence aborted: {e}")
    
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()