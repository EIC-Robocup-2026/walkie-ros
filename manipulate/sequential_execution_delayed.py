import rclpy
import time  # <--- NEW IMPORT
from rclpy.action import ActionClient
from rclpy.node import Node

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

def main(args=None):
    rclpy.init(args=args)
    commander = SequentialCommander()

    # CONFIG: Set delay duration (in seconds)
    DELAY_SECONDS = 2.0 

    try:
        # --- STEP 3: Close Gripper ---     
        if not commander.send_gripper_goal(0.0):
            raise Exception("Step 3 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY


        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, 0.0, 0.0]):
            raise Exception("Step 1 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY
        
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, -1.57, 0.523]):
            raise Exception("Step 1 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

        # --- STEP 2: Move Arm to Second Position ---
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, -1.57, 0.0, 0.0]):
            raise Exception("Step 1 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

        # --- STEP 3: Close Gripper ---     
        if not commander.send_gripper_goal(-12.5):
            raise Exception("Step 3 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

        # --- STEP 3: Close Gripper ---     
        if not commander.send_gripper_goal(0.2):
            raise Exception("Step 3 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, -1.57, 0.0]):
            raise Exception("Step 1 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

       # --- STEP 2: Move Arm to Second Position ---
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, -1.57, 0.0, 0.0]):
            raise Exception("Step 1 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

        # --- STEP 3: Close Gripper ---
        if not commander.send_gripper_goal(-12.5):
            raise Exception("Step 3 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

        # --- STEP 3: Close Gripper ---     
        if not commander.send_gripper_goal(0.0):
            raise Exception("Step 3 Failed")

        print(f"Waiting {DELAY_SECONDS}s...")
        time.sleep(DELAY_SECONDS) # <--- DELAY

        # --- STEP 1: Move Arm to First Position ---
        if not commander.send_arm_goal('left_arm', [0.663, 0.0, 0.0, -0.942, 0.0, -1.57, 0.523]):
            raise Exception("Step 1 Failed")
        commander.get_logger().info("All steps completed successfully!")

    except Exception as e:
        commander.get_logger().error(f"Sequence aborted: {e}")
    
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()