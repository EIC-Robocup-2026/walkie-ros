# walkie-ros

## Description
This repository contains the ROS 2 software stack for Walkie robot. It provides a comprehensive suite of packages to support both real-world robot operation and simulation.

The project includes:
- **Robot Description**: URDF and Xacro definitions for the robot's kinematics and visualization.
- **Hardware Interfaces**: Drivers for Cubemars actuators, Lidar sensors (Hokuyo, Lakibeam), and other peripherals.
- **Simulation**: Gazebo environments and configurations for testing the robot in a virtual world.
- **Navigation**: Integration with the Nav2 stack for autonomous navigation, localization, and SLAM.
- **Bringup**: Launch files to easily start the robot's hardware and software subsystems.

## Project Structure
```text
walkie-ros
├── bringup
├── description (sub repo)
├── hardware
│   ├── actuator
│   └── lidar
├── navigation (sub repo)
│   └── nav2
├── scripts
├── simulation
│   └── gazebo
├── tools
│   └── joy_interface
└── readme.md
```

- **bringup**: Launch files and config for launching necessary software stacks.
- **description**: Robot URDF and mesh files.
- **hardware**: Contains all hardware drivers and software/packages specific to hardware.
- **navigation**: Contains Nav2 config and launch files, and future custom navigation packages.
- **scripts**: Utility scripts for quick execution of common tasks (e.g., launching simulations).
- **simulation**: Gazebo launch, world, and model files.
- **tools**: Custom packages / tools for development/deployment.

# Getting Started

## 0. Prerequisites
Ensure you have the following software installed:
- **Ubuntu 24.04** (Native, VM, or WSL2)
- **ROS 2 Jazzy**
- **Git**
- **Git LFS**

## 1. Set up a ROS 2 Workspace
Create a workspace directory structure.
```text
ros2_ws
├── build/    # Build artifacts (generated)
├── install/  # Install artifacts (generated)
├── log/      # Build logs (generated)
└── src/      # Source code directory
    ├── walkie-ros  # This repository
    └── ...         # Other packages
```
Create the directory:
```bash
mkdir -p ~/ros2_ws/src
```

## 2. Clone the Repository
Clone this repository into your workspace `src` folder. Be sure to include submodules.
```bash
cd ~/ros2_ws/src/
git clone --recurse-submodules git@github.com:EIC-Robocup-2026/walkie-ros.git
```

## 3. Install Dependencies
Install the necessary ROS 2 dependencies using `rosdep`.
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

## 4. Build and Run
Build the packages and source the workspace.
```bash
colcon build --symlink-install
source install/setup.bash
```

### Run Navigation Simulation (Quick Start)
You can launch the full Gazebo simulation with the Nav2 stack using the provided script:
```bash
./src/walkie-ros/scripts/sim_nav_gazebo.sh
```

### Manual Launch

#### Run Navigation + Gazebo Simulation
```bash
ros2 launch robot_bringup nav_sim_omnibot.launch.py
```

#### Run Gazebo Simulation
```bash
ros2 launch robot_simulation gzsim_omnibot.launch.py
```

---

# Perception Setup

The perception stack (`walkie_perception`) uses a dedicated Python virtual environment so that heavy ML dependencies (PyTorch, Ultralytics, GraspNet) are isolated from the system Python used by ROS 2.

## 5. Create the Perception Virtual Environment

Install `uv` (fast pip/venv manager) and create the venv with access to system-site-packages (required for `cv_bridge`, `rclpy`, etc.):

```bash
pip install uv
uv venv ~/perception-venv --python /usr/bin/python3.12 --system-site-packages
source ~/perception-venv/bin/activate
```

> **Important**: always pass `/usr/bin/python3.12` (the system Python), not just `3.12`. If you pass a bare version, uv may download its own Python interpreter, which won't have ROS 2's C extensions (`rclpy._rclpy_pybind11` is compiled against the system Python 3.12 only).
>
> Verify before installing anything:
> ```bash
> ~/perception-venv/bin/python3 -c "import rclpy; print('rclpy ok')"
> ```

Install dependencies from the lock file (recommended — reproduces exact tested versions):

```bash
cd ~/Walkie_Project/walkie_ros/src/walkie-ros/perception
uv sync --active
```

Or install manually without the lock:

```bash
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install ultralytics opencv-python-headless open3d scipy
```

> The `pyproject.toml` and `uv.lock` live in `perception/`. Run `uv sync --active` from that directory to reproduce the exact environment used in development.

Add a `COLCON_IGNORE` so colcon never tries to build inside the venv:

```bash
touch ~/perception-venv/COLCON_IGNORE
```

## 6. Install GraspNet-1Billion

### Clone repositories

```bash
git clone https://github.com/graspnet/graspnet-baseline.git ~/graspnet-baseline
git clone https://github.com/graspnet/graspnetAPI.git ~/graspnetAPI
```

### Download checkpoint

Place the RealSense checkpoint at:
```
~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar
```

Available from the [GraspNet-1Billion dataset page](https://graspnet.net/datasets.html) (requires registration).

### Build CUDA extensions

Find your GPU's compute capability at [developer.nvidia.com/cuda-gpus](https://developer.nvidia.com/cuda-gpus), then set the architecture flag before building:

```bash
source ~/perception-venv/bin/activate
export TORCH_CUDA_ARCH_LIST="<your_sm>"   # e.g. "8.6" for sm_86, "8.9" for sm_89

cd ~/graspnet-baseline/pointnet2
python setup.py install

cd ~/graspnet-baseline/knn
python setup.py install
```

### Install graspnetAPI

```bash
cd ~/graspnetAPI
pip install -e .
uv pip install cvxopt trimesh autolab_core imageio scikit-learn
uv pip install "numpy>=1.26,<2.0"   # pin below numpy 2 for graspnetAPI compat
```

### Add graspnet to the venv's Python path

```bash
cat > ~/perception-venv/lib/python3.12/site-packages/graspnet_baseline.pth << 'EOF'
/home/ut/graspnet-baseline
/home/ut/graspnet-baseline/models
/home/ut/graspnet-baseline/pointnet2
/home/ut/graspnet-baseline/knn
EOF
```

## 7. Shell Aliases (add to `~/.zshrc` or `~/.bashrc`)

```bash
export PERCEPTION_VENV="$HOME/perception-venv/bin/python3"
alias perception='source $HOME/perception-venv/bin/activate && source /opt/ros/jazzy/setup.bash'
alias perception-build='cd ~/Walkie_Project/walkie_ros && colcon build --packages-select walkie_perception --symlink-install && source install/setup.zsh'
```

Reload: `source ~/.zshrc`

## 8. Build Perception Packages

```bash
cd ~/Walkie_Project/walkie_ros   # your workspace root
colcon build --packages-select walkie_perception --symlink-install
source install/setup.bash
```



---

# Running Perception Nodes

All Python nodes use a polyglot shebang that auto-selects the perception venv, so you do **not** need to activate the venv manually before `ros2 run`.

### YOLO + ByteTrack detection

```bash
ros2 run walkie_perception yolo_node
```

Publishes:
- `/yolo/tracked_detections_2d` — `Detection2DArray` with ByteTrack IDs
- `/yolo/masks` — `Image` (16UC1, tracker-ID label image)
- `/yolo/debug_image` — annotated image (if `publish_debug_image=True`)

Key parameters:
```bash
ros2 run walkie_perception yolo_node \
  --ros-args \
  -p model_path:=/path/to/yolo11l-seg.pt \
  -p confidence_threshold:=0.3 \
  -p device:=cuda \
  -p publish_debug_image:=true
```

### 3D OBB estimation

```bash
ros2 run walkie_perception obb3d
```

### Grasp pose service

```bash
ros2 run walkie_perception grasp_from_mask
```

On startup the node loads the GraspNet checkpoint into GPU (~1-2 s) then enters **standby** state.

---

# Grasp Service API

## `/grasp/status` — query node state

```bash
ros2 service call /grasp/status std_srvs/srv/Trigger
```

Returns the current state (`loading` / `standby` / `active` / `unloaded`), VRAM usage, and number of cached point-cloud frames.

## `/grasp/standby` — load / unload GPU model

Load model into GPU (required before calling `/grasp/from_mask`):
```bash
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: true}'
```

Unload model from GPU to free VRAM for other tasks:
```bash
ros2 service call /grasp/standby std_srvs/srv/SetBool '{data: false}'
```

## `/grasp/from_mask` — request grasp poses

The primary service. Requires the YOLO mask topic (`/yolo/masks`) to be live.

**Request fields:**

| Field | Type | Description |
|---|---|---|
| `mask` | `sensor_msgs/Image` | 16UC1 label image from `/yolo/masks` |
| `tracker_id` | `int32` | ByteTrack ID of the object to grasp |
| `bbox` | `vision_msgs/BoundingBox2D` | Fallback ROI if mask is sparse |
| `num_frames` | `int32` | Frames to accumulate (0 = adaptive 1→3→5) |
| `score_threshold` | `float32` | Minimum grasp score to return (0 = no filter) |
| `max_grasps` | `int32` | Cap on returned poses (0 = up to 20) |

**Response fields:**

| Field | Type | Description |
|---|---|---|
| `poses` | `geometry_msgs/PoseArray` | Grasp poses in ZED camera frame |
| `scores` | `float32[]` | GraspNet quality scores (higher = better) |
| `widths` | `float32[]` | Gripper widths in metres |
| `success` | `bool` | True if at least one grasp was found |
| `message` | `string` | Human-readable summary / error |
| `inference_ms` | `float32` | GraspNet inference time |
| `total_ms` | `float32` | End-to-end pipeline time |
| `points_extracted` | `int32` | Points from the masked region |
| `points_fed` | `int32` | Points after voxel downsample + sample |
| `grasps_raw` | `int32` | Candidates before NMS + score filter |
| `grasps_returned` | `int32` | Final poses returned |
| `frames_used` | `int32` | Point-cloud frames merged |

**Debug topics:**
- `/grasp/debug/masked_cloud` — `PointCloud2` of the points fed to GraspNet
- `/grasp/debug/grasp_markers` — `MarkerArray` of top-10 grasp arrows in RViz

### Minimal call example (CLI)

```bash
# Capture one mask frame and forward it
ros2 topic echo /yolo/masks --once > /tmp/mask.yaml

# Then call the service (typically done from a Python/C++ client, not CLI)
ros2 service call /grasp/from_mask walkie_perception/srv/GraspFromMask \
  '{tracker_id: 1, num_frames: 0, score_threshold: 0.2, max_grasps: 5}'
```

### Python client snippet

```python
from rclpy.node import Node
from walkie_perception.srv import GraspFromMask

class GraspClient(Node):
    def __init__(self):
        super().__init__('grasp_client')
        self.cli = self.create_client(GraspFromMask, '/grasp/from_mask')
        self.cli.wait_for_service()

    def call(self, mask_msg, tracker_id):
        req = GraspFromMask.Request()
        req.mask = mask_msg
        req.tracker_id = tracker_id
        req.num_frames = 0        # adaptive
        req.score_threshold = 0.2
        req.max_grasps = 5
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

---

# Full Perception Stack (tmux quick-start)

```bash
# Terminal 1 — YOLO tracker
ros2 run walkie_perception yolo_node

# Terminal 2 — 3D OBB estimation
ros2 run walkie_perception obb3d

# Terminal 3 — Grasp service
ros2 run walkie_perception grasp_from_mask

# Terminal 4 — check grasp node is ready
ros2 service call /grasp/status std_srvs/srv/Trigger
```

Or launch everything together:

```bash
ros2 launch walkie_perception perception.launch.py
```
