# robot gazebo
This repository contains the necessary launch files to simulate our robot in the Gazebo environment. The main launch file will start the Gazebo world, spawn the robot model described in the URDF, and load all the required controllers and sensor interface nodes.

## Usage
To launch the simulation environment and run all the necessary nodes, execute the following command in your terminal:
```
ros2 launch robot_simulation gzsim_omnibot.launch.py
```
