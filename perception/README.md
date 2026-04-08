# Perception Package

A ROS 2 package for **3D Object Detection** using YOLOv8 and RGB-D cameras (ZED). It provides three complementary nodes for localizing objects in 3D space — one that runs inference internally, one that subscribes to external 2D detections and streams poses continuously, and a high-performance C++ service node for on-demand 3D projection.

---

## Architecture Overview

```
┌───────────────────────────────────────────────────────────────────────┐
│                          perception package                           │
│                                                                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────────────┐   │
│  │  ob_detection   │  │    ob_pose      │  │  ob_pose_service     │   │
│  │   (Python)      │  │   (Python)      │  │      (C++)           │   │
│  │                 │  │                 │  │                      │   │
│  │ Runs YOLO       │  │ Subscribes to   │  │ On-demand service    │   │
│  │ internally      │  │ external 2D     │  │ call: send 2D dets   │   │
│  │ RGB+Depth sync  │  │ detections      │  │ → get 3D poses back  │   │
│  │ → map frame     │  │ → map frame     │  │ No cv_bridge needed  │   │
│  └────────┬────────┘  └────────┬────────┘  └──────────┬───────────┘   │
│           │                    │                      │               │
│           └────────────────────┤                      │               │
│                                ▼                      ▼              │
│                  /ob_detection/poses            /get_3d_poses         │
│                     (map frame)                  (srv response)       │
└───────────────────────────────────────────────────────────────────────┘
```

---

## Dependencies

### System Dependencies (ROS 2)

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-vision-msgs \
                 ros-$ROS_DISTRO-tf2-ros \
                 ros-$ROS_DISTRO-tf2-geometry-msgs \
                 ros-$ROS_DISTRO-image-transport \
```

### Python Dependencies

```bash
pip3 install ultralytics numpy
```

### Model Files

Place your YOLO model inside the package at:

```
perception/models/yolov8n.onnx
```

Both `.onnx` and `.pt` formats are supported (e.g., `yolov8n.pt`).

---

## Nodes

### 1. `ob_detection` — Integrated YOLO Detection

Runs the YOLOv8 model internally. Synchronizes RGB and Depth image streams, performs object detection, and computes each object's 3D position relative to the camera frame before transforming it to the map frame.

**Subscriptions**

| Topic | Type | Description |
|---|---|---|
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | Rectified RGB stream |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | Registered depth map |
| `/zed/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |

**Publications**

| Topic | Type | Description |
|---|---|---|
| `/ob_detection/poses` | `geometry_msgs/PoseArray` | Absolute 3D coordinates in map frame |
| `/ob_detection/markers` | `visualization_msgs/MarkerArray` | Visualization spheres for RViz |
| `/ob_detection/debug_image` | `sensor_msgs/Image` | Live video with bounding boxes |

**Services**

| Service | Type | Description |
|---|---|---|
| `/ob_detection/toggle` | `std_srvs/SetBool` | Start / stop processing |

**Run**

```bash
ros2 run perception ob_detection
```

---

### 2. `ob_pose` — Absolute Map Positioning

Designed to work with an **external 2D detector**. Accepts incoming 2D bounding boxes, looks up their depth from the latest registered depth frame, then uses TF2 to project the point from the camera frame into the map frame.

Depth sampling is done using a robust **median filter** over a configurable pixel window, avoiding noisy single-pixel reads.

**Subscriptions**

| Topic | Type | Description |
|---|---|---|
| `/yolo/detections_2d` | `vision_msgs/Detection2DArray` | Input 2D bounding boxes |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | Registered depth map |
| `/tf`, `/tf_static` | — | `camera_link` → `map` transform tree |

**Publications**

| Topic | Type | Description |
|---|---|---|
| `/ob_detection/poses` | `geometry_msgs/PoseArray` | Absolute 3D coordinates in map frame |
| `/ob_detection/markers` | `visualization_msgs/MarkerArray` | Green spheres at object locations |

**Services**

| Service | Type | Description |
|---|---|---|
| `/ob_detection/toggle` | `std_srvs/SetBool` | Start / stop processing |

**Run**

```bash
ros2 run perception ob_pose
```

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `depth_topic` | `/zed/zed_node/depth/depth_registered` | Depth image topic |
| `info_topic` | `/zed/zed_node/depth/camera_info` | Camera info topic |
| `target_frame` | `map` | TF2 target frame for output poses |
| `search_radius` | `3` | Pixel radius for robust median depth sampling |

Override at launch:

```bash
ros2 run perception ob_pose --ros-args \
  -p target_frame:=odom \
  -p search_radius:=5
```

---

### 3. `ob_pose_service` — C++ On-Demand 3D Projection Service

A high-performance **C++ service node** that exposes the same depth-to-map projection logic as a ROS 2 service. Instead of streaming continuously, it waits for a service call, projects all provided 2D detections to 3D in one shot, and returns the result. Uses direct pointer casting on raw depth buffers — no `cv_bridge` dependency required.

**Subscriptions**

| Topic | Type | Description |
|---|---|---|
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | Registered depth map (cached) |
| `/zed/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics (latched once) |
| `/tf`, `/tf_static` | — | `camera_link` → `map` transform tree |

**Services**

| Service | Type | Description |
|---|---|---|
| `/get_3d_poses` | `perception/srv/GetObPose` | Project a batch of 2D detections → 3D map poses |

**Service Interface**

```
# Request
vision_msgs/Detection2DArray detections
---
# Response
geometry_msgs/PoseArray poses
bool success
```

**Call Example**

```bash
ros2 service call /get_3d_poses perception/srv/GetObPose \
  "{detections: {detections: [{bbox: {center: {position: {x: 640.0, y: 360.0}}}}]}}"
```

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `depth_topic` | `/zed/zed_node/depth/depth_registered` | Depth image topic |
| `info_topic` | `/zed/zed_node/depth/camera_info` | Camera info topic |
| `target_frame` | `map` | TF2 target frame for output poses |
| `search_radius` | `3` | Pixel radius for robust median depth sampling |

**Run**

```bash
ros2 run perception ob_pose_service
```

---

## Toggling Detection

`ob_detection` and `ob_pose` both start in an **active state**. Use their toggle service to pause and resume processing:

```bash
# Enable detection
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: true}"

# Disable detection
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: false}"
```

> `ob_pose_service` has no toggle — it is purely reactive and only processes data when its service is called.

---

## How 3D Projection Works

Both `ob_pose` and `ob_pose_service` share the same projection pipeline. For each 2D detection bounding box center `(u, v)`:

1. **Robust depth** — samples a `(2r+1) × (2r+1)` pixel window around `(u, v)`, discards non-finite and near-zero values, returns the **median** of the valid set.
2. **Perspective unprojection** — uses camera intrinsics `(fx, fy, cx, cy)` to compute the 3D point in camera frame:
   ```
   x_c = (u - cx) * z / fx
   y_c = (v - cy) * z / fy
   z_c = z
   ```
3. **TF2 transform** — looks up the `camera_link → map` transform (100 ms timeout) and transforms the point into the target frame.
4. **PoseArray output** — each successfully transformed point is appended to the response as a `geometry_msgs/Pose` with a neutral orientation (`w = 1.0`).

---

## Package Structure

```
perception/
├── models/
│   └── yolov8n.onnx              # YOLO model weights
├── src/
│   └── ob_pose_service.cpp       # C++ on-demand service node
│   ├── ob_detection.py           # Python integrated YOLO node
│   └── ob_pose.py                # Python streaming pose node
├── srv/
│   └── GetObPose.srv             # Service definition
├── CMakeLists.txt
└── package.xml
```

---

## Quick Start

```bash
# 1. Build the package
cd ~/ros2_ws
colcon build --packages-select perception
source install/setup.bash

# 2. Launch your ZED camera driver (ensure TF is being published)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

# 3a. Integrated detection — YOLO runs inside the node
ros2 run perception ob_detection

# 3b. Streaming pose node — feed it external 2D detections
ros2 run perception ob_pose

# 3c. On-demand C++ service node — call /get_3d_poses when needed
ros2 run perception ob_pose_service

# 4. Enable processing (ob_detection and ob_pose only)
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: true}"

# 5. Visualize in RViz (add PoseArray and MarkerArray displays)
rviz2
```
