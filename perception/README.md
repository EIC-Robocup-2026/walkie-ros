# Perception Package

ROS 2 package for 3D object localization using a ZED RGB-D camera. Three nodes cover different use cases — pick the one that fits.

| Node | Language | Use when… |
|---|---|---|
| `ob_detection` | Python | You want self-contained detection — no external detector needed |
| `ob_pose` | Python | You already have 2D detections and want continuous 3D poses |
| `ob_pose_service` | C++ | You need on-demand 3D projection via a service call |

---

## Setup

**Install dependencies**

```bash
sudo apt install ros-$ROS_DISTRO-vision-msgs \
                 ros-$ROS_DISTRO-tf2-ros \
                 ros-$ROS_DISTRO-tf2-geometry-msgs \
                 ros-$ROS_DISTRO-image-transport

pip3 install ultralytics numpy
```

**Download the YOLO model**

The model must live at `perception/models/`. Both `.pt` and `.onnx` are supported.

Option A — download `.pt` directly (simplest, works out of the box):

```bash
mkdir -p perception/models
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
mv ~/.config/Ultralytics/yolov8n.pt perception/models/yolov8n.pt
```

Option B — export to ONNX for faster CPU inference:

```bash
mkdir -p perception/models
python3 - <<'EOF'
from ultralytics import YOLO
model = YOLO("yolov8n.pt")          # downloads if not cached
model.export(format="onnx")         # writes yolov8n.onnx next to the script
EOF
mv yolov8n.onnx perception/models/yolov8n.onnx
```

> `ob_detection` loads whichever file is at the path in `get_package_share_directory('perception')/models/`. Update the filename in `ob_detection.py:37` if you use a different model.

**Build**

```bash
cd /home/ut/Walkie_Project/walkie_ros
colcon build --packages-select perception --symlink-install
source install/setup.bash
```

---

## Node 1 — `ob_detection`

Runs YOLOv8 internally. Synchronizes RGB and depth streams, detects objects (person, bottle, cup, bowl), and publishes their 3D positions in the map frame. No external detector required.

**Run**

```bash
ros2 run perception ob_detection
```

**Subscriptions**

| Topic | Type |
|---|---|
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` |
| `/zed/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/CameraInfo` |

**Publications**

| Topic | Type | Description |
|---|---|---|
| `/ob_detection/poses` | `geometry_msgs/PoseArray` | 3D object positions in map frame |
| `/ob_detection/markers` | `visualization_msgs/MarkerArray` | Green spheres for RViz |
| `/ob_detection/debug_image` | `sensor_msgs/Image` | Live feed with bounding boxes |

**Toggle on/off**

```bash
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: true}"   # start
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: false}"  # pause
```

Starts active by default.

---

## Node 2 — `ob_pose`

Receives 2D bounding boxes from an external detector, looks up depth for each box center, and streams 3D poses in the map frame. Pair this with any node that publishes `Detection2DArray`.

**Run**

```bash
ros2 run perception ob_pose
```

**Subscriptions**

| Topic | Type |
|---|---|
| `/yolo/detections_2d` | `vision_msgs/Detection2DArray` |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` |
| `/zed/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` |

**Publications**

| Topic | Type | Description |
|---|---|---|
| `/ob_detection/poses` | `geometry_msgs/PoseArray` | 3D object positions in map frame |
| `/ob_detection/markers` | `visualization_msgs/MarkerArray` | Green spheres for RViz |

**Toggle on/off**

```bash
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: true}"   # start
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool "{data: false}"  # pause
```

> Topics, output frame (`map`), and depth sampling radius (5 px) are hardcoded.

---

## Node 3 — `ob_pose_service`

C++ service node. Subscribes to depth in the background, then on each service call projects a batch of 2D detections to 3D and returns the result. No streaming — only runs when called. No `cv_bridge` dependency.

**Run**

```bash
ros2 run perception ob_pose_service
```

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `depth_topic` | `/zed/zed_node/depth/depth_registered` | Depth image topic |
| `info_topic` | `/zed/zed_node/depth/camera_info` | Camera info topic |
| `target_frame` | `map` | Output TF frame |
| `search_radius` | `3` | Pixel radius for median depth sampling |

```bash
ros2 run perception ob_pose_service --ros-args \
  -p target_frame:=odom \
  -p search_radius:=5
```

**Service: `/get_3d_poses`**

```
# Request
vision_msgs/Detection2DArray detections
---
# Response
geometry_msgs/PoseArray poses
bool success
```

**Call example**

```bash
ros2 service call /get_3d_poses perception/srv/GetObPose \
  "{detections: {detections: [{bbox: {center: {position: {x: 640.0, y: 360.0}}}}]}}"
```

---

## How 3D Projection Works

All three nodes use the same pipeline for each detection center `(u, v)`:

1. **Robust depth** — sample a `(2r+1) × (2r+1)` pixel window, discard non-finite and near-zero values, take the **median**.
2. **Pinhole unproject** — compute the camera-frame point:
   ```
   x_c = (u - cx) * z / fx
   y_c = (v - cy) * z / fy
   z_c = z
   ```
3. **TF2 transform** — look up `camera_link → map` (100 ms timeout) and transform the point.
4. **Output** — append as `geometry_msgs/Pose` with identity orientation (`w = 1.0`).

---

## Visualizing in RViz

Add these displays after running any node:

- `PoseArray` → `/ob_detection/poses`
- `MarkerArray` → `/ob_detection/markers`
- `Image` → `/ob_detection/debug_image` (`ob_detection` only)
