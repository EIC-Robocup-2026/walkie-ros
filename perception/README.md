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

Loads GraspNet-1Billion into GPU on startup. On each service call it extracts the target object's point cloud, runs GraspNet inference, applies NMS + score filtering, and returns ranked grasp poses in both the camera optical frame and the planning frame.

```bash
ros2 run walkie_perception grasp_from_mask
```

The node caches frames continuously and answers requests on demand. **By default it reads the ZED organized point cloud** (`use_cloud:=true`) and needs no camera intrinsics; set `use_cloud:=false` to unproject the depth image instead.

**Point source (`use_cloud`)**

| Mode | Param | Notes |
|---|---|---|
| ZED point cloud (default) | `use_cloud:=true` | Uses ZED's registered organized cloud; converts body→optical internally |
| Depth unprojection | `use_cloud:=false` | Unprojects the depth image with `info_topic` intrinsics |

**Subscriptions (input)**

| Topic | Type | Description |
|---|---|---|
| `cloud_topic` (param) | `PointCloud2` | ZED organized cloud — used when `use_cloud:=true` |
| `depth_topic` (param) | `Image` (32FC1 or 16UC1) | Depth frames — used when `use_cloud:=false` |
| `info_topic` (param) | `CameraInfo` | Camera intrinsics (depth mode + bbox sizing) |

Both `32FC1` (ZED, metres) and `16UC1` (RealSense, millimetres) depth encodings are supported automatically.

**Publications (output)**

| Topic | Type | Description |
|---|---|---|
| `/grasp/debug/masked_cloud` | `PointCloud2` | Points fed to GraspNet (camera optical frame). Republished continuously so it stays visible in RViz between requests |
| `/grasp/debug/grasp_markers` | `MarkerArray` | Gripper poses as markers for RViz |

**Services**

| Service | Type | Description |
|---|---|---|
| `/grasp/from_mask` | `walkie_perception/srv/GraspFromMask` | Request grasp poses for an object |
| `/grasp/standby` | `SetBool` | `true` = load model into GPU, `false` = unload and free VRAM |
| `/grasp/status` | `Trigger` | Query current state and VRAM usage |

```bash
ros2 service call /grasp/status  std_srvs/srv/Trigger
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: true}'    # load
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: false}'   # unload
```

#### Calling `/grasp/from_mask`

**Request fields**

| Field | Type | Description |
|---|---|---|
| `mask` | `Image` (mono8 / 16UC1) | Mask image — by default **any non-zero pixel is the object**. Used in mask mode |
| `tracker_id` | `int32` | **Optional.** `> 0` selects that label in a multi-object mask; `0` (default) = use all non-zero pixels |
| `bbox` | `BoundingBox2D` | ROI used in bbox mode, or when the selected mask has fewer than 10 pixels |
| `num_frames` | `int32` | Frames to merge (`0` = adaptive: 1 → 3 → 5) |
| `score_threshold` | `float32` | Minimum grasp score (`0` = no filter) |
| `max_grasps` | `int32` | Cap on returned poses (`0` = up to 20) |

If the mask carries a single object you can leave `tracker_id` at `0` and the node uses every non-zero pixel. For a multi-object label image (e.g. raw `/yolo/masks`, where the pixel value is the tracker ID) set `tracker_id` to pick one object — no need to isolate it yourself.

**Region selection** — the node decides which pixels become the object cloud in this order:

1. `use_mask:=true` (default) **and** the selected mask has ≥ 10 pixels → use those mask pixels (`tracker_id > 0` → that label; else all non-zero pixels).
2. Otherwise → fall back to `bbox`.
3. `use_mask:=false` → always use `bbox` (the `mask` field is ignored).

Mask/bbox coordinates are in the full RGB image resolution; the node resizes the region down to the point-cloud / depth resolution automatically.

##### (A) With a segmentation mask

This is the normal, higher-quality path: the object's exact silhouette is used. The request needs a live mask, so it is called from code (or via `test_grasp_viz`), not by hand on the CLI.

Quickest way to test — `test_grasp_viz` grabs the live mask + detection and calls the service for you:

```bash
ros2 run walkie_perception test_grasp_viz --id 42      # by ByteTrack ID
ros2 run walkie_perception test_grasp_viz bottle       # by COCO class
```

To call it programmatically, copy the latest `/yolo/masks` message into the request and set `tracker_id` to pick one object out of the multi-object label image:

```python
req = GraspFromMask.Request()
req.mask            = self.mask          # latest /yolo/masks label image
req.tracker_id      = 42                 # select tracker 42's pixels
req.num_frames      = 0                  # adaptive
req.score_threshold = 0.3
req.max_grasps      = 5
future = self.cli.call_async(req)
```

If your mask already contains exactly one object, leave `tracker_id` at `0` (the default) and the node uses every non-zero pixel.

##### (B) Without a segmentation mask (bounding box)

Use a bounding box only — no `/yolo/masks` needed. Either start the node with `use_mask:=false`, or just leave the mask empty and supply a `bbox` (the node falls back to it). This is directly callable from the CLI:

```bash
# 1. (optional) start in bbox-only mode
ros2 run walkie_perception grasp_from_mask --ros-args -p use_mask:=false

# 2. load the model
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: true}'

# 3. call with a bounding box — centre + size cover the target object (RGB image pixels)
ros2 service call /grasp/from_mask walkie_perception/srv/GraspFromMask \
  "{num_frames: 5, score_threshold: 0.5, max_grasps: 5,
    bbox: {center: {position: {x: 960.0, y: 540.0}}, size_x: 200.0, size_y: 300.0}}"
```

**Response fields**

| Field | Type | Description |
|---|---|---|
| `poses` | `PoseArray` | Grasp poses in the **camera optical frame** (`zed_head_left_camera_frame_optical`) |
| `poses_base` | `PoseArray` | Grasp poses in the **planning frame** — empty if TF unavailable |
| `approach_poses_base` | `PoseArray` | Pre-grasp poses, backed out along the approach axis (`approach_dist_m`) |
| `planning_frame` | `string` | Frame used for `poses_base` / `approach_poses_base` |
| `scores` | `float32[]` | Quality scores 0–1, sorted highest first |
| `widths` | `float32[]` | Required gripper opening in metres per grasp |
| `object_size` | `Vector3` | Object AABB dimensions (x, y, z) in the planning frame — for a MoveIt CollisionObject |
| `object_bbox_pose` | `Pose` | Centre of the AABB in the planning frame (identity orientation) |
| `height_below_grasp` | `float32[]` | Metres from object bottom to each grasp point (planning-frame Z up) |
| `height_above_grasp` | `float32[]` | Metres from each grasp point to object top |
| `success` | `bool` | `true` if at least one grasp was returned |
| `message` | `string` | Human-readable summary or error |
| `inference_ms` | `float32` | GraspNet GPU inference time only |
| `total_ms` | `float32` | Full service call time |
| `points_extracted` | `int32` | Object points after filtering (before sampling) |
| `points_fed` | `int32` | Points sampled to `num_point` and fed to GraspNet |
| `grasps_raw` | `int32` | GraspNet candidates before NMS (typically ~2000) |
| `grasps_returned` | `int32` | Final poses after NMS + score filter + cap |
| `frames_used` | `int32` | Frames merged |

#### Python client example

A self-contained node that subscribes to `/yolo/masks`, calls `/grasp/from_mask`, and reads the result. It supports both modes — send a mask for mask mode, or set `USE_MASK = False` to call with a bounding box only.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from walkie_perception.srv import GraspFromMask

USE_MASK   = True       # True = segmentation mask, False = bounding box only
TRACKER_ID = 42         # multi-object mask: pick this label; 0 = all non-zero pixels


class GraspClient(Node):
    def __init__(self):
        super().__init__('grasp_client')
        self._mask = None

        # /yolo/masks is published BEST_EFFORT — match it or you get nothing.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, '/yolo/masks', self._mask_cb, qos)

        self._cli = self.create_client(GraspFromMask, '/grasp/from_mask')
        self.get_logger().info('waiting for /grasp/from_mask …')
        self._cli.wait_for_service()

    def _mask_cb(self, msg):
        self._mask = msg

    def request_grasp(self):
        req = GraspFromMask.Request()
        req.num_frames      = 0       # adaptive: 1 → 3 → 5
        req.score_threshold = 0.3
        req.max_grasps      = 5

        if USE_MASK:
            # Mask mode: wait for a live mask, then attach it. tracker_id picks
            # one object from a multi-object mask (0 = use all non-zero pixels).
            while self._mask is None and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)
            req.mask       = self._mask
            req.tracker_id = TRACKER_ID
        else:
            # Bounding-box mode: centre + size in full RGB image pixels.
            bbox = BoundingBox2D()
            bbox.center.position.x = 960.0
            bbox.center.position.y = 540.0
            bbox.size_x = 200.0
            bbox.size_y = 300.0
            req.bbox = bbox

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = GraspClient()
    resp = node.request_grasp()

    if not resp.success:
        node.get_logger().error(f'grasp failed: {resp.message}')
    else:
        node.get_logger().info(
            f'{resp.grasps_returned} grasps in {resp.planning_frame} '
            f'({resp.total_ms:.0f} ms)')
        for i, (pose, score, width) in enumerate(
                zip(resp.poses_base.poses, resp.scores, resp.widths)):
            p = pose.position
            node.get_logger().info(
                f'  [{i}] score={score:.2f} width={width*100:.1f}cm '
                f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        # Best grasp + its pre-grasp approach pose:
        # grasp    = resp.poses_base.poses[0]
        # pregrasp = resp.approach_poses_base.poses[0]

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Run it (the grasp node must already be up and loaded — `ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: true}'`):

```bash
python3 grasp_client.py        # or wrap as a console_script / ros2 run target
```

Notes:
- Use `poses_base` for arm planning (already in `planning_frame`); `poses` is the raw camera-frame output.
- Match the `/yolo/masks` QoS (`BEST_EFFORT`) or the subscription stays empty.
- In a node that also does other work, prefer `call_async` with a done-callback over `spin_until_future_complete` so you don't block your executor.

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `use_cloud` | `true` | `true` = ZED point-cloud source, `false` = depth unprojection |
| `cloud_topic` | `/zed_head/zed_node/point_cloud/cloud_registered` | Organized cloud topic (cloud mode) |
| `cloud_optical_frame` | `zed_head_left_camera_frame_optical` | Frame the cloud poses are emitted in |
| `depth_topic` | `/zed_head/zed_node/depth/depth_registered` | Depth image topic (depth mode) |
| `info_topic` | `/zed_head/zed_node/depth/camera_info` | CameraInfo topic |
| `use_mask` | `true` | `true` = use YOLO mask, `false` = bounding box only |
| `planning_frame` | `base_footprint` | Target TF frame for `poses_base` — changeable at runtime via `ros2 param set` |
| `approach_dist_m` | `0.10` | Pre-grasp back-off distance along the approach axis |
| `checkpoint_path` | `~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar` | GraspNet checkpoint |
| `num_point` | `10000` | Points sampled per inference call |
| `num_view` | `300` | GraspNet view anchors |
| `voxel_size_m` | `0.005` | Voxel downsample resolution (metres) |
| `min_points` | `300` | Minimum points required to attempt inference |
| `cache_size` | `10` | Max frames cached |
| `debug_cloud` | `true` | Publish `/grasp/debug/masked_cloud` |
| `debug_cloud_continuous` | `true` | Republish the last masked cloud continuously |
| `debug_cloud_rate_hz` | `5.0` | Republish rate |
| `outlier_removal` | `true` | Depth-weighted statistical outlier removal |
| `outlier_nb_neighbors` | `20` | Neighbours for outlier removal |
| `outlier_std_ratio` | `2.0` | Std ratio threshold for outlier removal |
| `cluster_filter` | `true` | DBSCAN dominant-cluster filter (removes background noise) |
| `cluster_eps` | `0.02` | DBSCAN neighbourhood radius (metres) |
| `cluster_min_samples` | `10` | DBSCAN minimum points per cluster |
| `gate_by_cloud` | `true` | **Depth path only** — drop depth pixels that have no point in the ZED cloud (removes exactly the low-confidence/edge points the cloud already rejected but the raw depth image keeps). Subscribes to the cloud even in depth mode |
| `depth_edge_filter` | `true` | **Depth path only** — reject residual sub-cloud-cell flying pixels at depth edges (neighbour-disagreement) |
| `depth_edge_tol_m` | `0.03` | Max depth difference (m) for a neighbour to count as agreeing |
| `depth_edge_radius` | `1` | Neighbourhood radius in pixels (1 = 3×3 window) |
| `depth_edge_min_ratio` | `0.6` | Min fraction of neighbours that must agree to keep a pixel |
| `mask_erosion_px` | `0` | Optional: shrink the region mask by N px before extraction (for loose masks). `0` = off |

`planning_frame` is read on every service call, so it can be changed without restarting the node:

```bash
ros2 param set /grasp_from_mask planning_frame map     # change at runtime
ros2 param get /grasp_from_mask planning_frame         # check current value
```

---

### `test_grasp_viz` — test and visualise (Grasp)

Waits for a live detection matching the target, calls `/grasp/from_mask`, builds the object point cloud locally, and opens an Open3D window with the cloud and gripper poses. It also prints the grasp poses in both the camera frame and the planning frame.

```bash
ros2 run walkie_perception test_grasp_viz bottle          # by COCO class name
ros2 run walkie_perception test_grasp_viz cell phone      # multi-word class
ros2 run walkie_perception test_grasp_viz --id 42         # by ByteTrack ID
ros2 run walkie_perception test_grasp_viz                 # first detected object

ros2 run walkie_perception test_grasp_viz bottle --score 0.3 --grasps 5

# Use the depth image instead of the ZED cloud for the local viewer cloud
ros2 run walkie_perception test_grasp_viz bottle --use-depth

# Override ZED namespace if not zed_head
ros2 run walkie_perception test_grasp_viz bottle \
  --cloud-topic /zed/zed_node/point_cloud/cloud_registered \
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
| `--cloud-topic TOPIC` | ZED organized cloud topic (default `/zed_head/zed_node/point_cloud/cloud_registered`) |
| `--use-depth` | Build the viewer cloud from depth unprojection instead of the ZED cloud |
| `--depth TOPIC` | Depth image topic (default `/zed_head/zed_node/depth/depth_registered`) |
| `--info TOPIC` | CameraInfo topic (default `/zed_head/zed_node/depth/camera_info`) |

---

## Grasp output — coordinate frames and conventions

### Reference frames

- **`poses`** — raw GraspNet output in the **ZED left camera optical frame** (`zed_head_left_camera_frame_optical`): X right, Y down, Z forward (depth).

  ```
         Z  (forward — depth direction)
        /
       /
      +———— X  (right)
      |
      Y  (down)
  ```

  `position {x: 0.142, y: -0.031, z: 0.487}` → grasp centre is 48.7 cm in front of the camera, 14.2 cm to the right, 3.1 cm above the lens centre.

- **`poses_base`** — the same grasps transformed into the **planning frame** (`base_footprint` by default) via TF, then aligned for the arm: a **−90° pitch about Y** is applied so the gripper approach points along the EE's local Z, plus a small **+0.025 m Z** calibration offset. Use these for arm planning.

### Quaternion orientation

Orientation is a standard ROS 2 quaternion `{x, y, z, w}`, converted from the GraspNet rotation matrix. It encodes the **gripper frame**:

| Axis | Meaning |
|---|---|
| Rotation matrix column 0 (approach) | Direction the finger tips point toward the object |
| Rotation matrix column 1 (closing) | Axis along which the fingers close |
| Rotation matrix column 2 (spread) | Left-to-right axis across the two fingers |

### Using the output in an arm controller

Use `poses_base` directly if your arm plans in `planning_frame`; otherwise transform `poses` via TF2 into your planning frame.

- `widths[i]` is the required gripper opening in metres — open the gripper to at least this width before approaching.
- The grasp pose is the **contact point** at the finger tips. Approach from the matching `approach_poses_base[i]` (backed out `approach_dist_m` along the approach vector) and advance to the grasp pose before closing.
- `object_size` + `object_bbox_pose` give the object AABB in the planning frame — feed them straight into a MoveIt `CollisionObject`.

Execution order:
1. Open gripper to `widths[0]`
2. Move arm to `approach_poses_base[0]` (pre-grasp)
3. Move arm to `poses_base[0]` (grasp)
4. Close gripper
