# Deploying the cuMotion stack to another machine

This is the cross-machine runbook: how to bring the cuMotion (cuRobo) GPU planner
up on a **fresh machine** (e.g. the robot PC) starting from the dev laptop where
it was built. For *what it is* and *how to drive it once running*, see
[`README.md`](README.md) in this directory — this doc only covers getting it
onto a new box.

> **Audience note (Claude):** **THREE** things must travel to the new machine —
> the image alone is NOT enough:
> 1. the Docker **image** `isaac_ros_cumotion:walkie` (CUDA + cuRobo + the
>    **STOCK** apt cuMotion plugin at `/opt/ros/jazzy`);
> 2. this **git branch** `feat/cumotion-integration` (the walkie repo);
> 3. the host **Isaac workspace** `~/workspaces/isaac_ros-dev` — specifically
>    `install/` (the **PATCHED** plugin overlay) and `openarm_cumotion/` (staged
>    launch scripts + the flat URDF). **This is the easy thing to miss.**
>
> The patch is **NOT baked into the image.** It was built into a host directory
> (`~/workspaces/isaac_ros-dev/install`) that the container only sees via a
> **bind mount** (`-v …/isaac_ros-dev:/workspaces/isaac_ros-dev`). `docker save`
> serializes image layers only — it does **not** capture bind-mounted host dirs.
> So a machine that received only the image has the **stock** plugin, and every
> RViz/commander plan fails `cspace_position [24] must equal [14]`. The patch
> source is also captured in-repo at
> [`cumotion_plugin_patch/`](cumotion_plugin_patch/) so the overlay can be rebuilt
> instead of copied (§1-B).

---

## 0. Prerequisites on the target machine

- NVIDIA GPU + recent driver (tested RTX 3070/3080 Ti Laptop, driver 580–595,
  CUDA cap 13).
- **`nvidia-container-toolkit` — often NOT installed; install it explicitly.**
  `nvidia-smi` working on the host does **not** mean Docker can see the GPU. Check
  `docker info --format '{{json .Runtimes}}' | grep nvidia`; if absent:
  ```bash
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
  ```
  Smoke test: `docker run --rm --gpus all <image> nvidia-smi -L` lists the GPU.
- **⚠️ Use `--gpus all` ALONE — do NOT also pass `--runtime nvidia`.** Combining
  the two double-injects the driver and breaks the container so that *every*
  binary (even `bash`) fails with `cannot execute binary file`. Pick one:
  `--gpus all` (modern, used below) **or** `--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all`.
- Docker. **Gotcha:** if `docker` was ever installed via Docker Desktop, the CLI
  may point at a dead socket. Force the system daemon:
  `export DOCKER_HOST=unix:///var/run/docker.sock`.
- ROS 2 Jazzy + `rmw_zenoh_cpp` on the host **only if** you'll drive the real
  robot (host hardware runs under zenoh). Mock/RViz-only testing needs nothing on
  the host but Docker.

---

## 1. Get the Docker image onto the target

The image (`isaac_ros_cumotion:walkie`, ~15 GB loaded) carries CUDA + cuRobo +
the **STOCK** apt cuMotion plugin only. The **patched** overlay is transferred
separately in §1.5 (it lives on the host, not in the image — see the audience
note). Two options for the image itself:

### A. Transfer the built image (fastest, no rebuild)

```bash
# (dev machine) stream save -> ssh -> load. ~22 GB over the wire, ~15 GB loaded.
export DOCKER_HOST=unix:///var/run/docker.sock          # both ends, if Docker Desktop was ever installed
docker save isaac_ros_cumotion:walkie | gzip \
  | ssh walkie@<TARGET_IP> 'DOCKER_HOST=unix:///var/run/docker.sock docker load'

# (target) verify
DOCKER_HOST=unix:///var/run/docker.sock docker images | grep cumotion
```

Watch target disk first (`df -h`): the gz stream + unpacked layers need headroom
(~30 GB free recommended).

### B. Rebuild from the Isaac ROS base (if no image to copy)

Follow NVIDIA's `isaac_ros_cumotion` build for Jazzy, then apply the three patched
sources from [`cumotion_plugin_patch/`](cumotion_plugin_patch/) (see that
directory's README) and
`colcon build --packages-select isaac_ros_cumotion_moveit isaac_ros_cumotion`
inside the container so `/workspaces/isaac_ros-dev/install` holds the patched
plugin **and** planner. (Two packages: `cumotion_move_group_client.cpp` +
`cumotion_interface.cpp` are the move_group plugin in `isaac_ros_cumotion_moveit`;
`robot_manager_impl.cpp` is the planner node in `isaac_ros_cumotion`.) Tag the
result `isaac_ros_cumotion:walkie`.

---

## 1.5. Transfer the host Isaac workspace (the patched overlay) — REQUIRED

This is the step the image transfer does **not** cover. The patched plugin
overlay and the staged launch scripts live on the dev host and must be copied to
the **same host path** on the target (it gets bind-mounted into the container in
§3).

```bash
# (dev machine) the overlay is tiny (~1 MB); openarm_cumotion is ~225 MB (flat URDF + meshes).
#  --network host container expects the host dir at ~/workspaces/isaac_ros-dev
rsync -az --info=progress2 \
  ~/workspaces/isaac_ros-dev/install \
  ~/workspaces/isaac_ros-dev/openarm_cumotion \
  ~/workspaces/isaac_ros-dev/src/isaac_ros_cumotion \
  walkie@<TARGET_IP>:'~/workspaces/isaac_ros-dev/'
#  install/                -> the PATCHED isaac_ros_cumotion_moveit overlay (essential)
#  openarm_cumotion/       -> launch_cumotion_stack.sh, goto.py, flat openarm_bimanual.urdf
#  src/isaac_ros_cumotion/ -> source, so the overlay can be rebuilt on the target (optional)
```

Verify on the target the patched `.so` is present:
`find ~/workspaces/isaac_ros-dev -name libisaac_ros_cumotion_moveit.so`.
(If you skipped this and only loaded the image, this find returns nothing and
cuMotion will fail `cspace_position [N] must equal [M]` — that's the symptom.)

> Alternative to copying: on the target, `git clone` the cuMotion source into
> `~/workspaces/isaac_ros-dev/src`, apply the three files from
> [`cumotion_plugin_patch/`](cumotion_plugin_patch/), and rebuild the overlay
> inside the container (§3). Copying the prebuilt `install/` works because the
> target runs the **same** image (same `/opt/ros/jazzy`, same `libcurobo`).

---

## 2. Get the repo onto the target (branch + submodules)

```bash
git clone --recurse-submodules <repo-url> walkie-ros
cd walkie-ros
git checkout feat/cumotion-integration
git submodule update --init --recursive
```

### ⚠️ The description-submodule gotcha (read this)

The `feat/cumotion-integration` branch pins the `description` submodule to a
commit (`4548f93`, branch `feat/cumotion-home-joint1-zero`) that sets the arm
home **joint1 = 0**. That commit is a *sibling* of the robot's tuned description
`main` (`0a8625f`, "fix(urdf): update robot collision meshes and openarm config")
— it does **NOT** contain the robot's collision-mesh / openarm / control-gains /
`base.urdf.xacro` tuning. **On the real robot, keep the old description**, or you
lose that tuning and change the arm home.

To pin the submodule to the robot's tuned description after checkout:

```bash
cd description
git checkout main          # = 0a8625f, the robot's tuned description (+-0.2618 home)
cd ..
git status --short         # shows ' M description' — the gitlink now differs from the branch
# leave it as a working-tree change, OR commit it locally so it survives:
#   git add description && git commit -m "chore(robot): pin description to old main"
git -c protocol.file.allow=always submodule update --init  # do NOT run this — it would re-pull 4548f93
```

- Use the branch's `4548f93` (joint1=0) only where you actually want the
  cuMotion-tuned home (e.g. a clean mock/sim demo machine).
- On the robot, the joint1=0 home and the missing tuning matter — use `0a8625f`.
- The home pose is *also* set for mock stacks in
  [`../initial_positions.yaml`](../initial_positions.yaml); the real robot home
  comes from `description/urdf/walkie/ros2_control.xacro`.

---

## 3. Create the container + build the walkie overlay

The container mounts **both** workspaces. The patched cuMotion overlay comes from
the host `~/workspaces/isaac_ros-dev/install` you copied in §1.5 (the wrapper
sources it on top of `/opt/ros/jazzy`); the walkie packages must be built once
per fresh container. Adjust the two `-v` host paths to the target's layout.

```bash
# (target host) recreate the persistent container
export DOCKER_HOST=unix:///var/run/docker.sock
docker rm -f isaac_ros_dev_container 2>/dev/null
# NOTE: --gpus all ONLY, NOT --runtime nvidia (combining them breaks exec — see §0)
docker run -d --name isaac_ros_dev_container --gpus all \
  --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v <TARGET>/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v <TARGET>/walkie-ws:/walkie_ws \
  --entrypoint /bin/bash isaac_ros_cumotion:walkie -lc 'sleep infinity'

# (inside container) build walkie into SEPARATE bases so the host mount isn't clobbered
docker exec -it isaac_ros_dev_container bash
source /opt/ros/jazzy/setup.bash
cd /walkie_ws
colcon build --packages-up-to openarm_bimanual_commander_cpp \
  --build-base /tmp/wb --install-base /tmp/wi --symlink-install
```

If the image was rebuilt (option 1-B) rather than copied, also build the plugin +
planner overlay once:
`cd /workspaces/isaac_ros-dev && colcon build --packages-select isaac_ros_cumotion_moveit isaac_ros_cumotion`.

The staged container scripts live at `/workspaces/isaac_ros-dev/openarm_cumotion/`
— copy them from this repo's `config/curobo/` into that path on the target if the
Isaac workspace is fresh. These now include the runtime/sphere tooling tracked in
git: `launch_cumotion_stack.sh`, `goto.py`, `pub_js.py`, `set_xrdf.py` (runtime
URDF/XRDF swap, with `--live-idle`), and the sphere tools `scale_spheres.py`,
`shrink_arm_spheres.py`, `shrink_left_spheres.py`, `scale_body.py`. The flat
`openarm_bimanual.urdf` + meshes are large and stay outside git — `rsync` those
from §1.5.

---

## 4. Launch + verify

Everything from here is identical to a local run — see [`README.md`](README.md)
§3 (run the wrapper), §4 (drive it), §5 (stop/restart), §7 (troubleshooting).
Quick smoke test inside the container:

```bash
/workspaces/isaac_ros-dev/openarm_cumotion/launch_cumotion_stack.sh \
    moveit_demo_with_commander_cumotion.launch.py        # self-contained mock + RViz
# look for "-> PATCHED overlay (good)" in the output
```

Real-robot wiring (host hardware under zenoh + container move_group) is in
[`README.md`](README.md) §3 "Real robot". Keep exactly **one** move_group across
host + container.

### Deployment sanity checklist

- [ ] `docker images | grep cumotion` shows `isaac_ros_cumotion:walkie` on target.
- [ ] Repo on `feat/cumotion-integration`; `description` at the **intended**
      commit (robot → `0a8625f`; demo box → `4548f93`).
- [ ] Walkie overlay built at `/tmp/wi` (or wherever) inside the container.
- [ ] Wrapper prints `PATCHED overlay (good)` — the patched plugin `.so` is the
      one loaded (`README.md` §1), not `/opt/ros/jazzy/lib`.
- [ ] A mock `both_arms` plan returns a trajectory in RViz in well under a second.
- [ ] (Real robot) host `rmw_zenohd` up; container launched with `RMW=zenoh`;
      `ros2 node list` from the host sees the container's move_group.
