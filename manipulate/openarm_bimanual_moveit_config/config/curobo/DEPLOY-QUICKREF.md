# cuMotion deploy — quick reference

Condensed runbook for bringing the cuMotion stack up on a fresh PC. Full detail,
gotchas, and rationale live in [`DEPLOY.md`](DEPLOY.md); this is the checklist.

## 3 artifacts must travel (git is only 1)

| # | Artifact | How | In git? |
|---|----------|-----|---------|
| 1 | Image `isaac_ros_cumotion:walkie` (~15 GB) | `docker save`/`load` or rebuild | no |
| 2 | Repo, branch `feat/cumotion-integration` | `git clone` | yes |
| 3 | `~/workspaces/isaac_ros-dev/` (`install/` overlay + `openarm_cumotion/`) | `rsync` or rebuild | sources only |

## 0. Target prereqs
- NVIDIA GPU + driver + **`nvidia-container-toolkit`** (often missing; `nvidia-smi`
  on host ≠ Docker GPU). Test: `docker run --rm --gpus all isaac_ros_cumotion:walkie nvidia-smi -L`.
- Use `--gpus all` **alone** — never with `--runtime nvidia` (breaks every binary).
- Docker; if Docker Desktop was ever installed: `export DOCKER_HOST=unix:///var/run/docker.sock`.
- Host ROS 2 Jazzy + `rmw_zenoh_cpp` **only** for the real robot.

## 1. Image
```bash
# A) copy (fastest) — need ~30 GB free on target
docker save isaac_ros_cumotion:walkie | gzip \
  | ssh walkie@<TARGET_IP> 'DOCKER_HOST=unix:///var/run/docker.sock docker load'
# B) rebuild: NVIDIA isaac_ros_cumotion (Jazzy) + apply patches (step 3) + retag
```

## 2. Repo
```bash
git clone --recurse-submodules <repo-url> walkie-ros
cd walkie-ros && git checkout feat/cumotion-integration
git submodule update --init --recursive
```
⚠️ **Submodule pin:** branch pins `description` to a `joint1=0` sibling that LACKS
the robot's mesh/control tuning. Real robot → `cd description && git checkout main`
(`0a8625f`). Demo/sim box → keep the pin.

## 3. Host Isaac workspace
```bash
# A) copy
rsync -az --info=progress2 \
  ~/workspaces/isaac_ros-dev/install \
  ~/workspaces/isaac_ros-dev/openarm_cumotion \
  ~/workspaces/isaac_ros-dev/src/isaac_ros_cumotion \
  walkie@<TARGET_IP>:'~/workspaces/isaac_ros-dev/'
```
```bash
# B) rebuild from in-repo patch sources — 3 FILES, 2 PACKAGES
D=/workspaces/isaac_ros-dev/src/isaac_ros_cumotion
cp <repo>/.../cumotion_plugin_patch/cumotion_move_group_client.cpp "$D/isaac_ros_cumotion_moveit/src/"
cp <repo>/.../cumotion_plugin_patch/cumotion_interface.cpp        "$D/isaac_ros_cumotion_moveit/src/"
cp <repo>/.../cumotion_plugin_patch/robot_manager_impl.cpp        "$D/isaac_ros_cumotion/src/impl/"
cd /workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_cumotion_moveit isaac_ros_cumotion   # BOTH
```
Stage git-tracked scripts into `openarm_cumotion/`: `launch_cumotion_stack.sh`,
`goto.py`, `pub_js.py`, `set_xrdf.py`, `scale_spheres.py`, `shrink_*.py`,
`scale_body.py`. The flat `openarm_bimanual.urdf` + meshes are big → rsync only.

## 4. Container + walkie overlay
```bash
docker run -d --name isaac_ros_dev_container --gpus all --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v ~/walkie-ros:/walkie_ws \
  --entrypoint /bin/bash isaac_ros_cumotion:walkie -lc 'sleep infinity'

docker exec -it isaac_ros_dev_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /walkie_ws && \
   colcon build --packages-up-to openarm_bimanual_commander_cpp \
     --build-base /tmp/wb --install-base /tmp/wi --symlink-install'
```

## 5. Launch + acceptance tests
```bash
cd <repo>/manipulate/openarm_bimanual_moveit_config/config/curobo
./run_cumotion_demo.sh                       # expect: error_code = 1, < 1 s
```
- **Patched plugin loaded:** `grep -i cumotion_moveit /proc/$(pgrep -x move_group)/maps`
  → `/workspaces/isaac_ros-dev/install/...` (not `/opt/ros/jazzy`).
- **Planner patch built in:** `strings .../libcumotion_impl.so | grep re-latching`.
- **Tool-frame re-latch:** `set_xrdf.py … right_arm_shrunk.xrdf` → log `WARN … re-latching`
  (not `FATAL`); then `./run_cumotion_demo.sh plan left_arm` → `error_code = 1`.
- **Heap-race fix:** repeated successful plans don't abort move_group with
  `malloc_consolidate(): invalid chunk size`.

### Checklist
- [ ] `docker images | grep cumotion` on target
- [ ] `description` at intended commit (robot `0a8625f` / demo pinned)
- [ ] `find ~/workspaces/isaac_ros-dev -name libisaac_ros_cumotion_moveit.so` → `install/` path
- [ ] `libcumotion_impl.so` contains `re-latching`
- [ ] mock `both_arms` < 1 s; cross-arm swap WARN-not-FATAL
