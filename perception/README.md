# Perception Package

This package provides ROS 2 nodes for **3D Object Detection** using YOLOv8 and RGB-D cameras (specifically ZED). It contains two distinct approaches for locating objects:
1.  **`ob_detection`**: Runs YOLO internally and calculates 3D coordinates relative to the *camera*.
2.  **`ob_pose`**: Listens for external 2D detections and transforms them into absolute 3D coordinates on the *map*.

---

##  Dependencies

### 1. System Dependencies (ROS 2)
Ensure these packages are installed via `apt`:
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-vision-msgs \
                 ros-$ROS_DISTRO-tf2-ros \
                 ros-$ROS_DISTRO-tf2-geometry-msgs \
                 ros-$ROS_DISTRO-image-transport \
                 ros-$ROS_DISTRO-cv-bridge
```
### 2. Python Dependencies

The nodes rely on ultralytics for YOLO inference and numpy for matrix operations.
Bash

pip3 install ultralytics numpy opencv-python

### 3. Model Files

Ensure your YOLO model (e.g., yolov8n.onnx or yolov8n.pt) is placed inside the package: perception/models/yolov8n.onnx
# Nodes
### 1. ob_detection (Integrated YOLO)

Description: This node runs the YOLOv8 model internally. It synchronizes RGB and Depth images to detect objects and calculates their 3D position relative to the camera frame.

    Subscribes:

        /zed/zed_node/rgb/color/rect/image (RGB Image)

        /zed/zed_node/depth/depth_registered (Depth Map)

        /zed/zed_node/rgb/color/rect/camera_info (Intrinsics)

    Publishes:

        /ob_detection/poses (geometry_msgs/PoseArra): Absolute 3D coordinates in map frame.

        /ob_detection/markers (visualization_msgs/MarkerArray): Visualization spheres for RViz.

        /ob_detection/debug_image (sensor_msgs/Image): Live video with bounding boxes drawn.

    Service:

        /ob_detection/toggle (std_srvs/SetBool): Start/Stop processing.

### Run Command:
```Bash

ros2 run perception ob_detection
``` 
### 2. ob_pose (Absolute Map Positioning)

Description: This node is designed to work with an external 2D detector. It takes 2D bounding boxes, looks up the depth, and uses TF2 to transform the coordinate from the Camera Frame to the Map Frame.

    Subscribes:

        /yolo/detections_2d (vision_msgs/Detection2DArray): Input bounding boxes.

        /zed/zed_node/depth/depth_registered (Depth Map)

        /tf & /tf_static: To perform the camera_link -> map transform.

    Publishes:

        /ob_detection/poses (geometry_msgs/PoseArra): Absolute 3D coordinates in map frame.

        /ob_detection/markers (visualization_msgs/MarkerArray): Green spheres at the object's global map location.

    Service:

        /ob_detection/toggle (std_srvs/SetBool): Start/Stop processing.
```
Run Command:
``` bash
ros2 run perception ob_pose
```

Service Usage (Toggle)

Both nodes start in a START state (self.is_active = True). You must call the service to start detection:
Bash

# Enable Detection
``` bash
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: true}"
``` 
Disable
``` bash
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: false}"
``` 