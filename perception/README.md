# walkie_perception

ROS 2 perception package for Walkie. Two pipelines share the same ZED RGB-D camera.

| Pipeline | When to use |
|---|---|
| **Classic** | You already have 2D detections and only need 3D positions |
| **Grasp** | Full pipeline: YOLO detection → 3D tracking → grasp pose generation |

---

## Prerequisites

- Ubuntu 24.04 + ROS 2 Jazzy
- CUDA-capable GPU (required for `yolo_node` and `grasp_from_mask`)
- `uv` package manager:
  ```bash
  pip install uv
  ```

---

## Setup

### Step 1 — Python environment

The grasp pipeline nodes need heavy ML libraries (PyTorch, Ultralytics, Open3D). These are installed into a dedicated venv at `~/perception-venv`, completely isolated from system Python and all other ROS packages. Classic pipeline nodes use only system packages — no venv needed.

```bash
cd <workspace>/src/walkie-ros/perception

./setup_env.sh            # base: yolo_node + obb3d (no grasp)
./setup_env.sh --grasp    # full: + grasp_from_mask + test_grasp_viz  (recommended)
```

The script creates `~/perception-venv` with system Python 3.12 + ROS library access, installs all deps from `uv.lock`, and (with `--grasp`) clones and installs `graspnetAPI`. Other nodes in the same launch file are unaffected — each node is a separate OS process and perception nodes select the venv via their shebang automatically.

### Step 2 — GraspNet CUDA extensions (grasp pipeline only)

One-time, device-specific step. Find your GPU's compute capability at [developer.nvidia.com/cuda-gpus](https://developer.nvidia.com/cuda-gpus) (e.g. `8.9` for RTX 40xx, `8.6` for RTX 30xx).

```bash
git clone https://github.com/graspnet/graspnet-baseline.git ~/graspnet-baseline

source ~/perception-venv/bin/activate
export TORCH_CUDA_ARCH_LIST="<your_sm>"   # e.g. "8.9"

cd ~/graspnet-baseline/pointnet2 && python setup.py install
cd ~/graspnet-baseline/knn       && python setup.py install
```

Place the checkpoint (requires free registration at [graspnet.net/datasets.html](https://graspnet.net/datasets.html)):
```
~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar
```

### Step 3 — Build

```bash
cd <workspace>
colcon build --packages-select walkie_perception --symlink-install
source install/setup.bash
```

---

## Node reference

### `ob_pose` — streaming 3D poses (Classic)

Receives 2D detections, looks up depth at each bounding-box centre, transforms to map frame, and streams 3D poses continuously.

```bash
ros2 run walkie_perception ob_pose
```

**Subscriptions (input)**

| Topic | Type | Description |
|---|---|---|
| `/yolo/detections_2d` | `Detection2DArray` | 2D bounding boxes from any detector |
| `/zed_head/zed_node/depth/depth_registered` | `Image` (32FC1) | ZED depth image |
| `/zed_head/zed_node/depth/camera_info` | `CameraInfo` | Camera intrinsics |

**Publications (output)**
**Publications (output)**

| Topic | Type | Description |
|---|---|---|
| `ob_detection/poses` | `PoseArray` | 3D object positions in `map` frame |
| `ob_detection/markers` | `MarkerArray` | Green sphere markers for RViz |

**Services**

| Service | Type | Description |
|---|---|---|
| `/ob_detection/toggle` | `SetBool` | `true` = start streaming, `false` = pause |

```bash
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool '{data: true}'
ros2 service call /ob_detection/toggle std_srvs/srv/SetBool '{data: false}'
```

---

### `ob_pose_service_cpp` — on-demand 3D projection (Classic)

C++ node. Subscribes to depth in the background, then on each service call projects a batch of 2D detections to 3D and returns the result immediately.

```bash
ros2 run walkie_perception ob_pose_service_cpp \
  --ros-args -p depth_topic:=/zed_head/zed_node/depth/depth_registered \
             -p info_topic:=/zed_head/zed_node/depth/camera_info \
             -p target_frame:=map
```

**Subscriptions (input)**

| Topic | Type | Description |
|---|---|---|
| `depth_topic` (param) | `Image` (32FC1) | ZED depth image |
| `info_topic` (param) | `CameraInfo` | Camera intrinsics |

**Services**

| Service | Type | Description |
|---|---|---|
| `/get_3d_poses` | `walkie_perception/srv/GetObPose` | Send detections, receive 3D poses |

Request:
```
vision_msgs/Detection2DArray detections   ← bounding boxes to project
```
Response:
```
geometry_msgs/PoseArray poses             ← 3D position per detection, in target_frame
bool success
```

```bash
ros2 service call /get_3d_poses walkie_perception/srv/GetObPose \
  "{detections: {detections: [{bbox: {center: {position: {x: 640.0, y: 360.0}}}}]}}"
```

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `depth_topic` | `/zed_head/zed_node/depth/depth_registered` | Depth image topic |
| `info_topic` | `/zed_head/zed_node/depth/camera_info` | CameraInfo topic |
| `target_frame` | `map` | Output TF frame |
| `search_radius` | `3` | Pixel radius for median depth sampling |

---

### `yolo_node` — detection + segmentation (Grasp)

Runs YOLOv8-seg with ByteTrack. Each detected object gets a persistent tracker ID that remains stable across frames.

```bash
ros2 run walkie_perception yolo_node \
  --ros-args -p model_path:=/path/to/yolo11l-seg.pt \
             -p confidence_threshold:=0.3 \
             -p device:=cuda
```

**Subscriptions (input)**

| Topic | Type | Description |
|---|---|---|
| `image_topic` (param) | `Image` / `CompressedImage` | RGB image from ZED |

**Publications (output)**

| Topic | Type | Description |
|---|---|---|
| `/yolo/tracked_detections_2d` | `Detection2DArray` | Bounding boxes with ByteTrack IDs |
| `/yolo/masks` | `Image` (16UC1) | Label image — pixel value = tracker ID of object |
| `/yolo/debug_image` | `Image` | Annotated RGB with boxes and IDs (optional) |

The `16UC1` mask is the key input to `grasp_from_mask`. Each pixel's value is the ByteTrack ID of the object covering it (0 = background).

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `model_path` | `models/yolo11l-seg.pt` | Path to `.pt` or `.onnx` model |
| `image_topic` | `/zed_head/zed_node/rgb/color/rect/image` | RGB input topic |
| `confidence_threshold` | `0.3` | YOLO confidence cutoff |
| `iou_threshold` | `0.45` | NMS IoU threshold |
| `device` | `cuda` | `cuda`, `cuda:0`, or `cpu` |
| `imgsz` | `640` | YOLO inference resolution |
| `publish_debug_image` | `true` | Publish annotated image |
| `class_filter` | `[]` | COCO class names to keep (empty = all) |

---

### `obb3d` — 3D object tracking (Grasp)

Consumes detections and ZED depth to maintain a 3D object database with oriented bounding boxes (OBB). Handles ID reuse when ByteTrack loses and reacquires objects.

```bash
ros2 run walkie_perception obb3d
```

**Subscriptions (input)**

| Topic | Type | Description |
|---|---|---|
| `/yolo/tracked_detections_2d` | `Detection2DArray` | Tracked 2D detections from `yolo_node` |
| `/zed_head/zed_node/depth/depth_registered` | `Image` (32FC1) | ZED depth |
| `/zed_head/zed_node/depth/camera_info` | `CameraInfo` | Camera intrinsics |
| `/zed_head/zed_node/confidence/confidence_map` | `Image` | ZED depth confidence (optional filter) |

**Publications (output)**

| Topic | Type | Description |
|---|---|---|
| `/obb3d/markers` | `MarkerArray` | 3D bounding box overlays for RViz |
| `/diagnostics` | `DiagnosticArray` | Pipeline health stats |

**Services**

| Service | Type | Description |
|---|---|---|
| `apex/get_object_cloud` | `walkie_perception/srv/GetObjectCloud` | Retrieve the point cloud for a tracked object by ID |

Request:
```
string object_id    ← tracker ID as string
```
Response:
```
sensor_msgs/PointCloud2 cloud
bool found
string message
```

---

### `grasp_from_mask` — grasp pose service (Grasp)

Loads GraspNet-1Billion into GPU on startup. On each service call: unprojects the masked depth region to a point cloud, runs GraspNet inference, applies NMS + score filtering, and returns ranked grasp poses in both camera frame and planning frame.

```bash
# With YOLO mask (default — better quality)
ros2 run walkie_perception grasp_from_mask \
  --ros-args \
  -p depth_topic:=/zed_head/zed_node/depth/depth_registered \
  -p info_topic:=/zed_head/zed_node/depth/camera_info

# Bounding box only (no mask required)
ros2 run walkie_perception grasp_from_mask \
  --ros-args \
  -p use_mask:=false \
  -p depth_topic:=/zed_head/zed_node/depth/depth_registered \
  -p info_topic:=/zed_head/zed_node/depth/camera_info
```

**Subscriptions (input)**

| Topic | Type | Description |
|---|---|---|
| `depth_topic` (param) | `Image` (32FC1 or 16UC1) | Depth frames cached continuously |
| `info_topic` (param) | `CameraInfo` | Camera intrinsics |

Both `32FC1` (ZED, metres) and `16UC1` (RealSense, millimetres) depth encodings are supported automatically.

**Publications (output)**

| Topic | Type | Description |
|---|---|---|
| `/grasp/debug/masked_cloud` | `PointCloud2` | Points fed to GraspNet (camera frame) |
| `/grasp/debug/grasp_markers` | `MarkerArray` | Top-10 gripper poses as LINE_LIST in RViz (permanent until next call) |

**Services**

| Service | Type | Description |
|---|---|---|
| `/grasp/from_mask` | `walkie_perception/srv/GraspFromMask` | Request grasp poses for an object |
| `/grasp/standby` | `SetBool` | `true` = load model into GPU, `false` = unload and free VRAM |
| `/grasp/status` | `Trigger` | Query current state and VRAM usage |

```bash
ros2 service call /grasp/status std_srvs/srv/Trigger
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: true}'
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: false}'
```

**`/grasp/from_mask` request**

| Field | Type | Description |
|---|---|---|
| `mask` | `Image` (16UC1) | Label image from `/yolo/masks` — ignored when `use_mask:=false` |
| `tracker_id` | `int32` | ByteTrack ID of the target object |
| `bbox` | `BoundingBox2D` | Bounding box ROI — used when `use_mask:=false` or mask has fewer than 10 pixels |
| `num_frames` | `int32` | Depth frames to merge (`0` = adaptive: tries 1 → 3 → 5) |
| `score_threshold` | `float32` | Minimum grasp score to return (`0` = no filter) |
| `max_grasps` | `int32` | Cap on returned poses (`0` = up to 20) |

**Calling with bbox only (no mask):**

```bash
# Load model first
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: true}'

# Call with bounding box — set centre and size to cover the target object
ros2 service call /grasp/from_mask walkie_perception/srv/GraspFromMask \
  "{tracker_id: 1, num_frames: 5, score_threshold: 0.5, max_grasps: 5,
    bbox: {center: {position: {x: 320.0, y: 240.0}}, size_x: 200.0, size_y: 200.0}}"
```

**`/grasp/from_mask` response**

| Field | Type | Description |
|---|---|---|
| `poses` | `PoseArray` | Grasp poses in **camera optical frame** |
| `poses_base` | `PoseArray` | Grasp poses in **planning frame** (`base_link` by default) — empty if TF unavailable |
| `planning_frame` | `string` | Frame used for `poses_base` |
| `scores` | `float32[]` | Quality scores 0–1, sorted highest first |
| `widths` | `float32[]` | Required gripper opening in metres per grasp |
| `success` | `bool` | `true` if at least one grasp was returned |
| `message` | `string` | Human-readable summary or error |
| `inference_ms` | `float32` | GraspNet GPU inference time only |
| `total_ms` | `float32` | Full service call time |
| `points_extracted` | `int32` | Depth pixels unprojected from the masked region |
| `points_fed` | `int32` | Points after voxel downsample + sample to `num_point` |
| `grasps_raw` | `int32` | GraspNet candidates before NMS (typically ~2000) |
| `grasps_returned` | `int32` | Final poses after NMS + score filter + cap |
| `frames_used` | `int32` | Depth frames merged |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `depth_topic` | `/zed_head/zed_node/depth/depth_registered` | Depth image topic |
| `info_topic` | `/zed_head/zed_node/depth/camera_info` | CameraInfo topic |
| `planning_frame` | `base_link` | Target TF frame for `poses_base` — changeable at runtime via `ros2 param set` |
| `ee_pitch_offset_deg` | `-90.0` | Body-frame pitch (about Y) applied to every grasp orientation so the gripper approach points along the EE's local Z — changeable at runtime via `ros2 param set` |
| `use_mask` | `true` | `true` = use YOLO mask, `false` = use bounding box only |
| `checkpoint_path` | `~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar` | GraspNet checkpoint |
| `num_point` | `10000` | Points sampled per inference call |
| `num_view` | `300` | GraspNet view anchors |
| `voxel_size_m` | `0.005` | Voxel downsample resolution (metres) |
| `min_points` | `300` | Minimum points required to attempt inference |
| `cache_size` | `10` | Max depth frames cached |
| `debug_cloud` | `true` | Publish `/grasp/debug/masked_cloud` |
| `outlier_removal` | `true` | Depth-weighted statistical outlier removal |
| `outlier_nb_neighbors` | `20` | Neighbours for outlier removal |
| `outlier_std_ratio` | `2.0` | Std ratio threshold for outlier removal |
| `cluster_filter` | `true` | DBSCAN dominant cluster filter (removes background noise) |
| `cluster_eps` | `0.02` | DBSCAN neighbourhood radius (metres) |
| `cluster_min_samples` | `10` | DBSCAN minimum points per cluster |

`planning_frame` is read on every service call so it can be changed without restarting the node:

# Change planning frame at runtime
ros2 param set /grasp_from_mask planning_frame map

# Check current value
ros2 param get /grasp_from_mask planning_frame

```bash
ros2 service call /grasp/status std_srvs/srv/Trigger
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: true}'
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: false}'
```

**`/grasp/from_mask` request**

| Field | Type | Description |
|---|---|---|
| `mask` | `Image` (16UC1) | Label image from `/yolo/masks` |
| `tracker_id` | `int32` | ByteTrack ID of the target object |
| `bbox` | `BoundingBox2D` | Fallback ROI used if mask has fewer than 10 pixels |
| `num_frames` | `int32` | Depth frames to merge (`0` = adaptive: tries 1 → 3 → 5) |
| `score_threshold` | `float32` | Minimum grasp score to return (`0` = no filter) |
| `max_grasps` | `int32` | Cap on returned poses (`0` = up to 20) |

**`/grasp/from_mask` response**

| Field | Type | Description |
|---|---|---|
| `poses` | `PoseArray` | Grasp poses in ZED camera optical frame — position (metres) + orientation (quaternion `{x,y,z,w}`) |
| `scores` | `float32[]` | Quality scores 0–1, sorted highest first |
| `widths` | `float32[]` | Required gripper opening in metres per grasp |
| `success` | `bool` | `true` if at least one grasp was returned |
| `message` | `string` | Human-readable summary or error |
| `inference_ms` | `float32` | GraspNet GPU inference time only |
| `total_ms` | `float32` | Full service call time |
| `points_extracted` | `int32` | Depth pixels unprojected from the masked region |
| `points_fed` | `int32` | Points after voxel downsample + sample to `num_point` |
| `grasps_raw` | `int32` | GraspNet candidates before NMS (typically ~2000) |
| `grasps_returned` | `int32` | Final poses after NMS + score filter + cap |
| `frames_used` | `int32` | Depth frames merged |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `depth_topic` | `/zed_head/zed_node/depth/depth_registered` | Depth image topic |
| `info_topic` | `/zed_head/zed_node/depth/camera_info` | CameraInfo topic |
| `checkpoint_path` | `~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar` | GraspNet checkpoint |
| `num_point` | `10000` | Points sampled per inference call |
| `num_view` | `300` | GraspNet view anchors |
| `voxel_size_m` | `0.005` | Voxel downsample resolution (metres) |
| `min_points` | `300` | Minimum points required to attempt inference |
| `cache_size` | `10` | Max depth frames cached |
| `debug_cloud` | `true` | Publish `/grasp/debug/masked_cloud` |

---

### `test_grasp_viz` — test and visualise (Grasp)

Waits for a live detection matching the target, calls `/grasp/from_mask`, unprojects the point cloud locally, and opens an Open3D window with the object cloud and gripper poses.

```bash
ros2 run walkie_perception test_grasp_viz bottle          # by COCO class name
ros2 run walkie_perception test_grasp_viz cell phone      # multi-word class
ros2 run walkie_perception test_grasp_viz --id 42         # by ByteTrack ID
ros2 run walkie_perception test_grasp_viz                 # first detected object

ros2 run walkie_perception test_grasp_viz bottle --score 0.3 --grasps 5

# Override ZED namespace if not zed_head
ros2 run walkie_perception test_grasp_viz bottle \
  --depth /zed/zed_node/depth/depth_registered \
  --info  /zed/zed_node/depth/camera_info
```

**Arguments**

| Argument | Description |
|---|---|
| `class_name` | COCO class name (positional, space-separated for multi-word) |
| `--id INT` | Target a specific ByteTrack tracker ID instead of a class name |
| `--score FLOAT` | Minimum grasp score (default `0.1`) |
| `--grasps INT` | Max grasps to request (default `10`) |
| `--depth TOPIC` | Depth image topic (default `/zed_head/zed_node/depth/depth_registered`) |
| `--info TOPIC` | CameraInfo topic (default `/zed_head/zed_node/depth/camera_info`) |

---

## Grasp output — coordinate frame and conventions

### Reference frame

All poses in the `/grasp/from_mask` response are in the **ZED left camera optical frame** (`zed_left_camera_frame_optical`). The node applies no TF transform.

```
       Z  (forward — depth direction)
      /
     /
    +———— X  (right)
    |
    Y  (down)
```

`position {x: 0.142, y: -0.031, z: 0.487}` → grasp centre is 48.7 cm in front of the camera, 14.2 cm to the right, 3.1 cm above the lens centre.

### Quaternion orientation

Orientation is a standard ROS 2 quaternion `{x, y, z, w}`, converted from the GraspNet rotation matrix by the node. It encodes the **gripper frame**:

| Axis | Meaning |
|---|---|
| Rotation matrix column 0 (approach) | Direction the finger tips point toward the object |
| Rotation matrix column 1 (closing) | Axis along which the fingers close |
| Rotation matrix column 2 (spread) | Left-to-right axis across the two fingers |

### Using the output in an arm controller

Before sending to MoveIt or the arm commander, transform the pose from the camera frame to `base_link` (or whichever planning frame your arm uses) via TF2.

The `widths[i]` value is the required gripper opening in metres — open the gripper to at least this width before approaching.

The grasp pose is the **contact point** at the finger tips. Approach from ~10 cm back along the approach vector (rotation column 0) and advance to the pose before closing.

Execution order:
1. Open gripper to `widths[0]`
2. Move arm to pre-grasp pose (10 cm back along approach direction)
3. Move arm to grasp pose
4. Close gripper
