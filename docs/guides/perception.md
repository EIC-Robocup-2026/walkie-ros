# Perception

Turn the head ZED camera's RGB-D data into 3D object positions and grasp
poses with the `walkie_perception` package (`perception/`).

The package
[README](https://github.com/EIC-Robocup-2026/walkie-ros/blob/main/perception/README.md)
has message formats, client examples, and the conventions of the grasp
output, but parts of it are out of date. This page follows the code. See
[Where the README is out of date](#where-the-readme-is-out-of-date).

## Choose a pipeline

- **Classic**: you already have 2D detections and need their 3D positions in
  the `map` frame. Runs in the pixi environment, no GPU. Nodes:
  `ob_pose_service_cpp`, `ob_pose`, `cloud_to_map`.
- **Grasp**: you need detection, 3D object tracking, and grasp poses. Needs
  its own Python venv and a GPU for `grasp_server` (`yolo_node` uses CUDA by
  default). Nodes: `yolo_node`, `obb3d`, `grasp_server`, `test_grasp_viz`.

??? info "How the two pipelines work"
    The classic pipeline projects the center of each 2D bounding box into 3D
    using the depth image and returns positions in the `map` frame. The grasp
    pipeline segments objects with YOLO, keeps a 3D database of tracked
    objects, and runs GraspNet-1Billion to produce gripper poses.

    ```mermaid
    flowchart LR
        subgraph Classic
            det["Any 2D detector"] -->|"/yolo/detections_2d"| obpose["ob_pose"]
            obpose -->|"/ob_detection/poses (map)"| out1["Task code"]
            out1 -->|"/get_3d_poses"| obsrv["ob_pose_service_cpp"]
        end
        subgraph Grasp
            rgb["ZED RGB"] --> yolo["yolo_node"]
            yolo -->|"/yolo/tracked_detections_2d, /yolo/masks"| obb["obb3d"]
            obb -->|"/apex/get_object_cloud"| client["Task code"]
            client -->|"/grasp/from_mask, /grasp/from_cloud, /grasp/pos"| grasp["grasp_server"]
        end
    ```

## Camera topics

All nodes read the ZED stereo camera on the robot's head, under the namespace
`/zed_head/zed_node`.

- **Real robot**: the ZED ROS wrapper (`zed_wrapper`) publishes the topics
  below, except `.../cloud_registered/filtered`, which comes from the self
  filter in `real_omnibot.launch.py`. `real_omnibot.launch.py` starts the
  wrapper when `use_zed:=true` (the default). Running the camera on the robot
  requires ZED SDK 5.5 and CUDA 13.2, with the wrapper fetched via `pixi run setup-real`
  (see [Real Robot: Set up the ZED camera](real-robot.md#zed-setup)).
- **Gazebo**: `simulation/gazebo/config/gazebo/gz_bridge.yaml` bridges a
  simulated head camera to the same names, except where the table says no.

| Topic | Type | In Gazebo |
| --- | --- | --- |
| `/zed_head/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | yes |
| `/zed_head/zed_node/depth/depth_registered` | `sensor_msgs/Image` | yes |
| `/zed_head/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` | no, Gazebo publishes `/zed_head/zed_node/depth/depth_registered/camera_info` |
| `/zed_head/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | yes |
| `/zed_head/zed_node/point_cloud/cloud_registered/filtered` | `sensor_msgs/PointCloud2` | no |
| `/zed_head/zed_node/confidence/confidence_map` | `sensor_msgs/Image` | no |

To change the ZED settings (resolution, frame rate, depth mode, and so on),
edit `bringup/config/camera/zed2i.yaml` and restart the launch. No rebuild is
needed after a `--symlink-install` build.

??? info "How the ZED wrapper is supervised and configured"
    - `real_omnibot.launch.py` runs the wrapper through the `zed_watchdog`
      supervisor unless `use_zed_watchdog:=false`. See
      [Real Robot: ZED watchdog](real-robot.md#zed-watchdog).
    - `real_omnibot.launch.py`, `zed_watchdog`, and `test_zed.launch.py` pass
      `bringup/config/camera/zed2i.yaml` to the wrapper as
      `ros_params_override_path`, so its values replace the wrapper defaults.
    - After a `--symlink-install` build, the installed `zed2i.yaml` is a link
      to the source file. That is why edits apply on the next launch without
      a rebuild.

## Get 3D positions from 2D detections { #classic-pipeline }

`real_omnibot.launch.py` (with `use_zed`) and `gzsim_omnibot.launch.py`
already start `ob_pose_service_cpp` and `cloud_to_map`. Send your
`detections` (`Detection2DArray`) to `/get_3d_poses`
(`walkie_perception/srv/GetObPose`). You get back `poses` (`PoseArray`, `map`
frame) and `success`.

!!! warning
    In Gazebo, `/get_3d_poses` returns `success: false` until you fix
    `info_topic` in `gzsim_omnibot.launch.py`. See
    [Known issues](#known-issues).

To run the service node by hand, pass the camera topics. The code defaults
use the old `/zed` namespace.

```bash
pixi run ros2 run walkie_perception ob_pose_service_cpp --ros-args \
  -p depth_topic:=/zed_head/zed_node/depth/depth_registered \
  -p info_topic:=/zed_head/zed_node/depth/camera_info
```

For a continuous stream instead of a service, run `ob_pose`. It reads
`/yolo/detections_2d`, which no node in the repository publishes, so remap it
to the `yolo_node` output or publish your own detections there:

```bash
pixi run ros2 run walkie_perception ob_pose --ros-args \
  -r /yolo/detections_2d:=/yolo/tracked_detections_2d
```

`ob_pose` publishes `ob_detection/poses` (`PoseArray`, `map` frame) and
`ob_detection/markers` (`MarkerArray`). `/ob_detection/toggle`
(`std_srvs/SetBool`) pauses and resumes it.

`cloud_to_map` republishes the ZED point cloud in the `map` frame, by default
on `/zed_head/zed_node/point_cloud/cloud_registered/filtered_map`.

??? info "How the classic nodes work, and their parameters"
    - `ob_pose_service_cpp` keeps the latest depth image and camera
      intrinsics, and answers `/get_3d_poses` on demand. For each detection
      in the request, it samples the depth around the box center and
      transforms the point to `target_frame`.
    - `ob_pose` does the same projection continuously. Besides
      `/yolo/detections_2d`, it subscribes to the depth image and the camera
      info. It uses a normal `#!/usr/bin/env python3` line and runs with the
      pixi interpreter, not the perception venv.
    - `cloud_to_map` transforms a point cloud into the `map` frame.
      `gzsim_omnibot.launch.py` sets `source_topic` to the unfiltered cloud,
      because Gazebo has no self filter.

    `ob_pose_service_cpp` parameters:

    | Parameter | Code default | Value in the bringup launch files |
    | --- | --- | --- |
    | `depth_topic` | `/zed/zed_node/depth/depth_registered` | `/zed_head/zed_node/depth/depth_registered` |
    | `info_topic` | `/zed/zed_node/depth/camera_info` | `/zed_head/zed_node/depth/camera_info` |
    | `target_frame` | `map` | `map` |
    | `search_radius` | `3` (pixels) | not set |

    `cloud_to_map` parameters:

    | Parameter | Default |
    | --- | --- |
    | `source_topic` | `/zed_head/zed_node/point_cloud/cloud_registered/filtered` |
    | `target_topic` | `/zed_head/zed_node/point_cloud/cloud_registered/filtered_map` |
    | `target_frame` | `map` |

## Set up the grasp pipeline

The grasp nodes run in a separate Python venv at `~/perception-venv`, created
by `perception/setup_env.sh`. You need:

- `uv` on `PATH` and the system Python `/usr/bin/python3.12` (Ubuntu 24.04).
  Neither comes from pixi.
- An NVIDIA GPU with CUDA for `grasp_server`. Without one, it stops at
  startup with `CUDA not available — GPU required`. `yolo_node` runs without
  a GPU with `-p device:=cpu`. `obb3d` and the classic nodes need no GPU.

??? info "Why the grasp pipeline does not use the pixi environment"
    - The dependencies are pip wheels: PyTorch and torchvision from the
      PyTorch CUDA 12.8 index (`cu128`), Ultralytics, and Open3D. The grasp
      extra adds GraspNet helpers, and GraspNet's CUDA extensions are
      compiled against that exact PyTorch build.
    - `perception/pyproject.toml` pins `numpy>=1.26,<2.0`. The pixi
      environment ships numpy 2.5.3, so both cannot share one environment.
    - The CUDA wheels are large, and simulation, navigation, and manipulation
      work does not need them. Keeping them out of `pixi.toml` keeps the
      shared environment small.

### Create the venv { #what-a-pixi-user-must-do }

`setup_env.sh` stops unless the system Python can `import rclpy`, so run it
in a way that gives it a ROS 2 installation. Drop `--grasp` if you only need
`yolo_node` and `obb3d`.

=== "pixi only"

    ```bash
    pixi run ./perception/setup_env.sh --grasp
    ```

    Inside `pixi run`, `PYTHONPATH` points at the pixi packages, so the
    system Python can import pixi's `rclpy` and the check passes.

=== "System ROS 2 Jazzy"

    In a plain shell, not under pixi:

    ```bash
    source /opt/ros/jazzy/setup.bash
    ./perception/setup_env.sh --grasp
    ```

=== "Physical Docker image"

    Nothing to do. `docker/Dockerfile.physical` runs `setup_env.sh --grasp`
    with `/opt/ros/jazzy` sourced, and builds the GraspNet extensions with
    `TORCH_CUDA_ARCH_LIST="12.0"`. It does not download the GraspNet
    checkpoint.

!!! warning
    Under `pixi run`, the venv imports pixi's numpy 2.x and pixi's OpenCV 5.0.
    See [Known issues](#known-issues).

To use a venv at another path, set `PERCEPTION_VENV` to its Python
executable (`export PERCEPTION_VENV=/path/to/venv/bin/python3`), not to the
venv directory.

??? info "What setup_env.sh does, step by step"
    ```bash
    ./perception/setup_env.sh           # yolo_node and obb3d only
    ./perception/setup_env.sh --grasp   # also grasp_server and test_grasp_viz
    ```

    | Step | Action |
    | --- | --- |
    | 1 | Creates `~/perception-venv` from `/usr/bin/python3.12` with `--system-site-packages`, and adds `COLCON_IGNORE` to it. Recreates the venv if it exists |
    | 2 | Checks that `import rclpy` works inside the venv, and stops if it does not |
    | 3 | Installs the locked dependencies from `perception/uv.lock` with `uv sync` (`--extra grasp` with `--grasp`) |
    | 4 | With `--grasp`: clones `graspnetAPI` to `~/graspnetAPI` and installs it, registers `~/graspnet-baseline` in the venv (`graspnet_baseline.pth`) if it exists, and re-installs already-built CUDA extensions |

    The script does not build the package. Build it with
    `pixi run build-pkg walkie_perception`.

??? info "How the nodes find the venv"
    The Python nodes of the grasp pipeline start with these two lines instead
    of a Python shebang:

    ```sh
    #!/bin/sh
    "exec" "${PERCEPTION_VENV:-$HOME/perception-venv/bin/python3}" "$0" "$@"
    ```

    `/bin/sh` runs the second line, which re-runs the same file with the
    venv's interpreter. Python then reads that line as a plain string and
    ignores it. The effect is that `ros2 run` uses the venv for these nodes,
    while every other node keeps its normal interpreter.

### Build the GraspNet CUDA extensions { #graspnet-cuda-extensions }

GraspNet needs two CUDA extensions from `graspnet-baseline`, `pointnet2` and
`knn`. Compile them once per machine, after `setup_env.sh --grasp`. Set
`TORCH_CUDA_ARCH_LIST` to the GPU's compute capability (for example `8.9` for
an RTX 40 series card):

```bash
git clone https://github.com/graspnet/graspnet-baseline.git ~/graspnet-baseline
source ~/perception-venv/bin/activate
export TORCH_CUDA_ARCH_LIST="8.9"
cd ~/graspnet-baseline/pointnet2 && python setup.py install
cd ~/graspnet-baseline/knn && python setup.py install
```

- Keep the clone at `~/graspnet-baseline`. `grasp_server` adds that path to
  its import path itself.
- If you cloned it after your first `setup_env.sh --grasp` run, run
  `setup_env.sh --grasp` again. It registers the clone in the venv only when
  the clone exists. The rerun recreates the venv and re-installs the built
  extensions without compiling them again.

### Get the model weights

- **GraspNet checkpoint**: not in the repository. Download it from the
  GraspNet website (free registration) and place it at
  `~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar`, or point the
  `checkpoint_path` parameter of `grasp_server` at it.
- **YOLO model**: not in the repository (`perception/models/` is ignored by
  git). Pass its path to `yolo_node` with `model_path`, as in
  [Run the grasp pipeline](#run-the-grasp-pipeline).

### Build the package

```bash
pixi run build-pkg walkie_perception
```

!!! warning "Rebuild after every Python edit"
    The CMake build installs the perception Python scripts under new names,
    so they are copied, not symlinked, even with `--symlink-install`.

## Run the grasp pipeline

No launch file in the repository starts these nodes. Run each command in its
own terminal, with the camera topics available.

=== "Real robot"

    ```bash
    pixi run ros2 run walkie_perception yolo_node --ros-args -p model_path:=/path/to/yolo11l-seg.pt
    pixi run ros2 run walkie_perception obb3d
    pixi run ros2 run walkie_perception grasp_server
    pixi run ros2 run walkie_perception test_grasp_viz bottle
    ```

=== "Gazebo"

    The simulated camera publishes its intrinsics on
    `/zed_head/zed_node/depth/depth_registered/camera_info`. `obb3d` has the
    camera info topic fixed in the code, so remap it. `grasp_server` takes
    the topic as `info_topic`, and `test_grasp_viz` as `--info`.

    ```bash
    pixi run ros2 run walkie_perception yolo_node --ros-args -p model_path:=/path/to/yolo11l-seg.pt
    pixi run ros2 run walkie_perception obb3d --ros-args \
      -r /zed_head/zed_node/depth/camera_info:=/zed_head/zed_node/depth/depth_registered/camera_info
    pixi run ros2 run walkie_perception grasp_server --ros-args \
      -p info_topic:=/zed_head/zed_node/depth/depth_registered/camera_info
    pixi run ros2 run walkie_perception test_grasp_viz bottle \
      --info /zed_head/zed_node/depth/depth_registered/camera_info
    ```

`test_grasp_viz` is a test client. It picks an object by COCO class name
(here `bottle`) or tracker ID, calls `/grasp/from_mask`, and shows the cloud
and grippers in an Open3D window. To move an arm to a grasp pose, use the
commander described in [Manipulation](manipulation.md).

??? info "yolo_node: what it does, parameters, and outputs"
    `yolo_node` runs YOLO segmentation with ByteTrack tracking, so each object
    keeps the same ID across frames.

    | Parameter | Default |
    | --- | --- |
    | `model_path` | `<share of walkie_perception>/models/yolo11l-seg.pt` |
    | `image_topic` | `/zed_head/zed_node/rgb/color/rect/image` (a `/compressed` fallback is also subscribed) |
    | `confidence_threshold`, `iou_threshold` | `0.3`, `0.45` |
    | `tracker_config` | `bytetrack.yaml` |
    | `device` | `cuda` |
    | `imgsz` | `640` |
    | `class_filter` | `[]` (all classes) |
    | `publish_debug_image` | `true` |

    It publishes `/yolo/tracked_detections_2d`
    (`vision_msgs/Detection2DArray`), `/yolo/masks` (`Image`, `16UC1`, each
    pixel holds the tracker ID of the object, 0 is background), and
    `/yolo/debug_image`.

??? info "obb3d: what it does, inputs, and outputs"
    `obb3d` combines the tracked detections, the masks, and the depth image
    into a database of 3D objects with oriented bounding boxes. Its topic
    names are fixed in the code: it reads
    `/zed_head/zed_node/depth/depth_registered`,
    `/zed_head/zed_node/depth/camera_info`, `/yolo/masks`,
    `/yolo/tracked_detections_2d`, and the ZED confidence map
    (`confidence_topic` parameter). The output frame is `target_frame`
    (default `map`).

    | Output | Type |
    | --- | --- |
    | `apex/detections_3d` | `vision_msgs/Detection3DArray` |
    | `apex/markers` | `MarkerArray` |
    | `apex/debug/raw_cluster`, `apex/debug/accumulated_cloud` | `PointCloud2` |
    | `apex/debug/labels` | `MarkerArray` |
    | `/diagnostics` | `DiagnosticArray` |

## Request grasp poses

`grasp_server` loads GraspNet-1Billion onto the GPU once at startup, serves
three grasp services from that one model, and handles requests one at a time.

| Service | Input | Use it when |
| --- | --- | --- |
| `/grasp/from_mask` | A YOLO mask (or a bounding box) over the live camera view | You have a mask from `yolo_node` |
| `/grasp/from_cloud` | A segmented object cloud, for example from `obb3d` | You have the object's cloud in any TF frame |
| `/grasp/pos` | A segmented object cloud, combined with the live camera cloud | You want the best grasp |

- `/grasp/pos` runs GraspNet on the live crop, filters the grasps to the
  object box, and checks them against the object surface.
- To get an object's cloud, call `/apex/get_object_cloud` on `obb3d` with
  the tracker ID as a string.
- All three return `poses_base` and `approach_poses_base` (pre-grasp poses
  backed off along the approach axis), with scores, widths, and the object's
  bounding box. They are in the request's `planning_frame`, or in the
  server's `planning_frame` parameter (default `map`) when the request leaves
  it empty, as `test_grasp_viz` does.

To free VRAM, call `/grasp/standby` with `false` to unload the model, and
with `true` to load it again. The model is loaded at startup, so you only
need `true` after an unload. `/grasp/status` reports the state and VRAM use.

```bash
pixi run ros2 service call /grasp/status std_srvs/srv/Trigger
pixi run ros2 service call /grasp/standby std_srvs/srv/SetBool "{data: false}"
```

??? info "grasp_server parameters and debug topics"
    | Parameter | Default | Meaning |
    | --- | --- | --- |
    | `checkpoint_path` | `~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar` | GraspNet weights |
    | `planning_frame` | `map` | Output frame, when the request leaves it empty. Read on every request |
    | `approach_dist_m` | `0.15` | Back-off distance of the approach pose. Read on every request |
    | `use_cloud` | `false` | `/grasp/from_mask` source: `false` unprojects the depth image, `true` uses the ZED point cloud |
    | `use_mask` | `true` | Use the mask; `false` uses the bounding box only |
    | `min_points` | `200` | Minimum points needed to run inference |
    | `depth_topic`, `info_topic`, `cloud_topic` | the ZED topics in [Camera topics](#camera-topics) | Camera inputs |

    Debug output is published on `/grasp/debug/*` (point clouds and gripper
    markers for RViz).

??? info "All perception services, types, and fields"
    | Service | Type | Node |
    | --- | --- | --- |
    | `/get_3d_poses` | `walkie_perception/srv/GetObPose` | `ob_pose_service_cpp` |
    | `/ob_detection/toggle` | `std_srvs/srv/SetBool` | `ob_pose` |
    | `/apex/get_object_cloud` | `walkie_perception/srv/GetObjectCloud` | `obb3d` |
    | `/apex/dump_database` | `std_srvs/srv/Trigger` | `obb3d` |
    | `/grasp/from_mask` | `walkie_perception/srv/GraspFromMask` | `grasp_server` |
    | `/grasp/from_cloud` | `walkie_perception/srv/GraspFromCloud` | `grasp_server` |
    | `/grasp/pos` | `walkie_perception/srv/GraspPos` | `grasp_server` |
    | `/grasp/standby` | `std_srvs/srv/SetBool` | `grasp_server` |
    | `/grasp/status` | `std_srvs/srv/Trigger` | `grasp_server` |

    The service definitions are in `perception/srv/`:

    | Service type | Request | Response |
    | --- | --- | --- |
    | `GetObPose` | `detections` (`Detection2DArray`) | `poses` (`PoseArray`), `success` |
    | `GetObjectCloud` | `object_id` (the tracker ID as a string) | `cloud`, `found`, `message` |
    | `GraspFromMask` | `mask`, `tracker_id`, `bbox`, `num_frames`, `score_threshold`, `max_grasps`, `planning_frame` | grasp poses, approach poses, `scores`, `widths`, object box, heights above and below each grasp |
    | `GraspFromCloud` | `cloud`, thresholds, `planning_frame`, optional preference weights | Same as `GraspFromMask` |
    | `GraspPos` | `object_cloud`, `crop_margin_m`, thresholds, `planning_frame`, optional preference weights | Same, plus `antipodal_scores` |

??? info "Executables, source files, and what starts them"
    Source paths are relative to `perception/`.

    | Executable | Source | Runtime | Started by |
    | --- | --- | --- | --- |
    | `cloud_to_map` | `src/cloud_to_map_node.cpp` | C++ | `real_omnibot.launch.py` (with `use_zed`), `gzsim_omnibot.launch.py` |
    | `ob_pose_service_cpp` | `src/ob_pose_service.cpp` | C++ | `real_omnibot.launch.py` (with `use_zed`), `gzsim_omnibot.launch.py` |
    | `ob_pose` | `perception/ob_pose.py` | Python, pixi interpreter | `ros2 run` |
    | `yolo_node` | `perception/yolo_node.py` | Python, perception venv | `ros2 run` |
    | `obb3d` | `perception/obb_3d.py` | Python, perception venv | `ros2 run` |
    | `grasp_server` | `perception/grasp_node.py` | Python, perception venv | `ros2 run` |
    | `test_grasp_viz` | `perception/test_grasp_viz.py` | Python, perception venv | `ros2 run` |

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| `gzsim_omnibot.launch.py` passes `info_topic: /zed_head/zed_node/depth/camera_info` to `ob_pose_service_cpp`, but Gazebo publishes the intrinsics on `/zed_head/zed_node/depth/depth_registered/camera_info` | In Gazebo, `/get_3d_poses` returns `success: false` | Change `info_topic` in `simulation/gazebo/launch/gzsim_omnibot.launch.py` to the Gazebo topic |
| The `ob_pose_service_cpp` code defaults use the old `/zed` namespace | Started without parameters, the node waits for topics that nothing publishes | Pass `depth_topic` and `info_topic`, as in [Get 3D positions from 2D detections](#classic-pipeline) |
| No node in the repository publishes `/yolo/detections_2d`, the input of `ob_pose` | `ob_pose` never publishes poses | Remap its input to `/yolo/tracked_detections_2d`, or publish detections on `/yolo/detections_2d` |
| The default `model_path` of `yolo_node` points to `models/yolo11l-seg.pt` in the installed share directory, but the build installs no `models/` directory | The default model file is not there after a build | Pass `-p model_path:=/path/to/yolo11l-seg.pt` |
| Inside `pixi run`, `PYTHONPATH` lists pixi's `site-packages`, and `PYTHONPATH` entries come before the venv's own packages | The venv interpreter imports pixi's numpy 2.5.3 instead of the pinned numpy 1.x, and pixi's `cv2` (`py-opencv` 5.0.0) instead of the pinned `opencv-python` 4.11.0.86. See [OpenCV under pixi](#opencv-under-pixi). | If a node fails with a numpy or binary compatibility error, run the grasp pipeline in the physical Docker image |
| The final message of `setup_env.sh` tells you to build `grasp_interfaces` in `~/Walkie_Project` | No `grasp_interfaces` package exists; the service definitions are in `walkie_perception` | `pixi run build-pkg walkie_perception` |
| Parts of the perception README are out of date | The README and the code disagree | Follow the code, see the table below |

### Where the README is out of date

| README says | The code does |
| --- | --- |
| The grasp node is `grasp_from_mask`, node name `/grasp_from_mask` | The executable is `grasp_server`, node name `/grasp_server` |
| Only `/grasp/from_mask` exists | `/grasp/from_cloud` and `/grasp/pos` are served too |
| `use_cloud` defaults to `true` | `use_cloud` defaults to `false` (depth unprojection) |
| `planning_frame` defaults to `base_footprint` | `planning_frame` defaults to `map`. The comments in the `.srv` files have the same error |
| `approach_dist_m` 0.10, `min_points` 300 | 0.15 and 200 |
| The response has camera-frame `poses` and timing fields | Neither exists; timings are only logged |
| `obb3d` publishes `/obb3d/markers` | It publishes `apex/markers`, `apex/detections_3d`, and debug topics, and also serves `apex/dump_database` |
| `yolo_node` runs YOLOv8-seg | The default model is `yolo11l-seg.pt` |
| Install uv with `pip install uv`, build with `colcon build` | Use pixi for the build; `uv` is a separate tool on `PATH` |

### OpenCV under pixi

The perception code meets more than one OpenCV version. When you add OpenCV
code to the perception nodes, make sure it works on OpenCV 4 and OpenCV 5.

| Source | OpenCV |
| --- | --- |
| `perception/uv.lock`, for the venv | `opencv-python` and `opencv-python-headless` 4.11.0.86 |
| The pixi environment | `py-opencv` 5.0.0. The pixi `cv_bridge` is built against it. Under `pixi run`, `import cv2` in the venv loads this one, because `PYTHONPATH` comes first (see [Known issues](#known-issues)). |
| apt Jazzy and the [Docker images](docker.md) | Ubuntu's OpenCV 4.6, for their ROS packages such as `cv_bridge` |

??? info "OpenCV calls in the perception nodes, and the OpenCV 5 trial"
    Under pixi, OpenCV was 4.13.0 before
    [the move to RoboStack build set `_25`](../migrations/04-robostack-build-25.md).
    The nodes call only a few OpenCV functions:

    | File | Calls |
    | --- | --- |
    | `yolo_node.py` | `CvBridge.imgmsg_to_cv2(msg, 'bgr8')`, `cv2.imdecode`, `cv2.fillPoly` |
    | `grasp_node.py` | `cv2.resize` with `INTER_NEAREST`, `cv2.erode` |
    | `test_grasp_viz.py` | `cv2.resize` with `INTER_NEAREST` |

    In a trial on 2026-10-12, these calls gave bit-identical results on
    OpenCV 5.0.0 and 4.13.0, and a `CvBridge` round trip worked.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| A grasp node fails at once with `not found` for `python3` | `~/perception-venv` does not exist | Run `setup_env.sh`, or set `PERCEPTION_VENV` to the venv's Python executable |
| `setup_env.sh` stops with `rclpy not found` | The system Python cannot see a ROS 2 installation | Source `/opt/ros/jazzy/setup.bash`, or run the script with `pixi run` |
| `grasp_server` exits with `CUDA not available` | No usable NVIDIA GPU or driver | Run it on the GPU machine |
| `grasp_server` fails on `from models.graspnet import GraspNet` | `~/graspnet-baseline` is missing or the extensions are not built | Follow [Build the GraspNet CUDA extensions](#graspnet-cuda-extensions) |
| Grasp requests fail after a `/grasp/standby false` | The model is unloaded | Call `/grasp/standby` with `true` |
| Edits to a perception Python script have no effect | The scripts are copied on install | `pixi run build-pkg walkie_perception` |

More fixes are collected in [Troubleshooting](../troubleshooting.md).
