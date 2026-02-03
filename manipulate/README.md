# Manipulate

This folder contains packages related to the manipulation capabilities of the robot, specifically for the left arm.

## Packages

### [left_arm_commander_cpp](./left_arm_commander_cpp)
This package contains the commander node for controlling the left arm. It likely interfaces with MoveIt and other ROS 2 control components to send commands to the arm.

### [left_arm_moveit_config](./left_arm_moveit_config)
This package contains the MoveIt 2 configuration files for the left arm. This includes the SRDF, kinematics configuration, joint limits, and launch files for bringing up the MoveIt planning pipeline.

### [my_robot_interfaces](./my_robot_interfaces)
This package defines custom interfaces (messages, services, actions) used by the manipulation packages. It ensures consistent communication types between nodes.
    