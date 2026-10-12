# Docker Images

**Goal:** understand the two Docker images, run them with the compose files,
and know how CI builds and publishes them.

The files live in `docker/`. The CI workflow is
`.github/workflows/build-docker.yml`.

!!! note "The images do not use pixi"
    Both images install ROS 2 Jazzy from apt, resolve dependencies with
    `rosdep`, and build with `colcon`. They do not read `pixi.toml` or
    `pixi.lock`. Package versions in the images can differ from the pixi
    environment. For daily development, use pixi (see
    [Working with Pixi](pixi.md)).

## Images

| Image | Dockerfile | Base image | Purpose |
| --- | --- | --- | --- |
| `ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation` | `docker/Dockerfile.simulation` | `osrf/ros:jazzy-desktop-full` | Gazebo simulation |
| `ghcr.io/eic-robocup-2026/walkie-ros/walkie-physical` | `docker/Dockerfile.physical` | `stereolabs/zed:5.2-gl-devel-cuda12.8-ubuntu24.04` | Physical robot: ZED SDK, CUDA, RealSense, GraspNet |

## Simulation image

`docker/Dockerfile.simulation` expects the repository root as the build
context. It does these steps:

1. Copy the whole repository to `/ros2_ws/src`.
2. Install `python3-pip`, `ros-dev-tools`, and `ros-jazzy-teleop-twist-keyboard`
   with apt.
3. Run `rosdep update`, then `rosdep install --from-paths src -y -i` to
   install the dependencies declared in the `package.xml` files.
4. Install `ros-jazzy-rmw-zenoh-cpp` with apt.
5. Build the workspace with `colcon build --symlink-install --parallel-workers 3`.
6. Remove `ros-dev-tools` and clean the apt cache to make the image smaller.

## Physical image

`docker/Dockerfile.physical` starts from the Stereolabs ZED image, which
already has CUDA and the ZED SDK. It does these steps:

1. Set the locale, add the ROS 2 apt repository, and install
   `ros-jazzy-desktop`, `ros-jazzy-ros-base`, colcon, rosdep, and vcstool.
2. Install point cloud transport, `rmw_zenoh_cpp`, `topic_tools`, and
   `foxglove_bridge` with apt.
3. Build librealsense from source.
4. Run `rosdep init` and `rosdep update`.
5. Clone and build `zed-ros2-wrapper` and `zed-ros2-examples` in a separate
   workspace, `/home/zed_deps_ws`.
6. Install uv to `/root/.local/bin`.
7. Copy `./src` from the build context to `/ros2_ws/src`. Install the
   RealSense ROS packages, PCL, and the `rosdep` dependencies of the source.
8. Set up GraspNet: run `perception/setup_env.sh --grasp`, which creates
   `~/perception-venv`, then build the GraspNet CUDA extensions with
   `TORCH_CUDA_ARCH_LIST="12.0"`.
9. Add `source` lines to `/root/.bashrc`.

The Dockerfile does not build the walkie workspace. `docker/entrypoint.sh`
does that when the container starts (see
[`compose.physical.yml`](#composephysicalyml)).

!!! warning "Build context"
    `COPY ./src /ros2_ws/src/` needs a build context that contains a `src/`
    folder with this repository inside it as `src/walkie-ros`. That is the
    layout of a classic colcon workspace: `<workspace>/src/walkie-ros`.
    `compose.physical.yml` uses this layout. The repository root has no
    `src/` folder, so builds with the repository root as context fail (see
    [Known issues](#known-issues)).

## Compose files

| File | Service | Container name | Image |
| --- | --- | --- | --- |
| `docker/compose.simulation.yml` | `robot_simulation` | `robot-simulation` | `ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation:feat-github_action_ci-cd` (pulled) |
| `docker/compose.physical.yml` | `zed_wrapper` | `robot_physical-zed_wrapper` | `zed-docker-zed_wrapper:latest` (built locally) |
| `docker/compose.remote.yml` | `docker-remote-desktop` | `docker-remote-desktop` | `ghcr.io/lanjelin/docker-remote-desktop:latest` (pulled) |

Relative paths in the compose files, such as `../`, are relative to the
`docker/` folder.

### `compose.simulation.yml`

- Pulls the simulation image with the tag `feat-github_action_ci-cd`. That tag
  comes from an old CI branch (see [Known issues](#known-issues)). CI on
  `main` publishes the tag `main`.
- Uses the host network and host IPC, and passes `/dev/dri` for graphics.
- Mounts the X11 socket (see [X11 access](#x11-access)).
- Mounts the repository at `/ros2_ws/src`. The build, install, and log
  folders are named volumes, so they survive container restarts.
- On start, runs `colcon build --symlink-install --parallel-workers 1` and
  then `ros2 launch robot_simulation gzsim_omnibot.launch.py`.
- Sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. See [Zenoh router](#zenoh-router).

### `compose.physical.yml`

- Builds the image locally from `Dockerfile.physical` with the context
  `../../..`. This is the workspace root when the repository sits at
  `<workspace>/src/walkie-ros`.
- Uses the NVIDIA runtime and reserves all GPUs.
- Runs privileged with host network, IPC, and PID, mounts `/dev`, and adds
  the `dialout` group. This gives the container the CAN interfaces, serial
  ports, and USB cameras.
- Sets `ROS_DOMAIN_ID=23` and `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- Mounts the X11 socket (see [X11 access](#x11-access)).
- Mounts `../../../src` at `/ros2_ws/src`. The build, install, and log
  folders are named volumes, as in the simulation file.
- Mounts `./models/checkpoint-rs.tar` as the GraspNet checkpoint. The
  repository does not contain this file (see [Known issues](#known-issues)).
- Keeps the ZED AI models and settings in the named volumes `zed_resources`
  and `zed_settings`, so the ZED SDK does not optimize the models again on
  every start.
- Runs `docker/entrypoint.sh`.

`docker/entrypoint.sh` does not launch the robot. It does these steps, then
waits with `sleep infinity`:

1. Source `/opt/ros/jazzy` and the ZED workspace.
2. Build `/ros2_ws` with `colcon build --symlink-install`.
3. Run `uv sync` in `src/walkie-ros/bringup`. This creates `bringup/.venv`
   for the lift node (see [Real Robot](real-robot.md)).

Open a shell in the container and start the robot from there. Source the
workspace yourself, because the image's `/root/.bashrc` uses a wrong path (see
[Known issues](#known-issues)):

```bash
source /ros2_ws/install/setup.bash
```

### `compose.remote.yml`

Runs a remote desktop container on ports 3000 (HTTP) and 3001 (HTTPS). The
volume `/path/to/config:/config` is a placeholder. Change it to a real folder
before you start the container.

## just recipes

`docker/justfile` holds shortcuts for the commands above. Install
[just](https://github.com/casey/just) and run the recipes from the `docker/`
folder. The recipes that run Docker use `sudo docker`.

| Recipe | What it runs |
| --- | --- |
| `just` | Lists the recipes |
| `just build-sim` | `sudo docker build -t eic-robocup2026-development:latest -f Dockerfile.simulation ..` |
| `just build-real` | `sudo docker build -t eic-robocup2026-development:latest -f Dockerfile.physical ..` |
| `just run-sim` | `xhost local:root`, then `sudo docker compose -f compose.simulation.yml up` |
| `just stop-sim` | `sudo docker compose -f compose.simulation.yml down` |
| `just run-real` | `xhost local:root`, then `sudo docker compose -f compose.physical.yml up` |
| `just stop-real` | `sudo docker compose -f compose.physical.yml down` |
| `just run-remote` | `sudo docker compose -f compose.remote.yml up` |
| `just shell <container>` | `sudo docker exec -it <container> /bin/bash` |
| `just run-topic-list` (alias `just list-topics`) | `ros2 topic list` inside `robot-simulation` |

`build-sim` and `build-real` have problems with their tag and context. Use
`run-sim` and `run-real` instead (see [Known issues](#known-issues)).

Example: open a shell in the physical container.

```bash
cd docker
just shell robot_physical-zed_wrapper
```

## X11 access

The containers run as root and open windows on your desktop (Gazebo, RViz).
`xhost local:root` allows local clients, including root in the container, to
connect to your X server. `run-sim` and `run-real` run it for you. The setting
stays active until the X server restarts. To remove it earlier:

```bash
xhost -local:root
```

Both `compose.simulation.yml` and `compose.physical.yml` require `DISPLAY`.
Without it, compose stops with an error. Run the compose commands from a
terminal in your desktop session, so `DISPLAY` is set.

## CI workflow

`.github/workflows/build-docker.yml` builds and pushes both images.

| Item | Value |
| --- | --- |
| Triggers | Push to `main`, push to `feat/github_action_CI/CD`, manual run (`workflow_dispatch`) |
| Jobs | `build-sim-image` and `build-physical-image`, in parallel on `ubuntu-latest` |
| Images | `ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation` and `ghcr.io/eic-robocup-2026/walkie-ros/walkie-physical` |
| Tags | From `docker/metadata-action`. A branch push is tagged with the branch name, for example `main`. |
| Build context | The repository root (`.`) |
| Cache | Registry cache at the `:buildcache` tag of each image, `mode=max` |

Each job has these steps:

1. Check out the repository with `submodules: recursive` and `lfs: true`.
2. Run `git submodule foreach --recursive git lfs pull`.
3. Set up Docker Buildx and log in to `ghcr.io` with `GITHUB_TOKEN`.
4. Compute the tags with `docker/metadata-action`.
5. Build and push with `docker/build-push-action`.

Why step 2 exists: `actions/checkout` fetches Git LFS files only for the
main repository, not for submodules. The `.dae` meshes in the `description/`
submodule are stored in Git LFS. Without this step, the image would contain
small LFS pointer files instead of the meshes.

`git submodule foreach --recursive` runs the command in every submodule,
including nested ones.

The images on GHCR are public. To pull the simulation image from `main`:

```bash
docker pull ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation:main
```

!!! warning
    The physical image does not build in CI, so `walkie-physical:main` is out of date (see [Known issues](#known-issues)).

## Zenoh router

Both compose files set `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Neither of them
starts a Zenoh router. Nodes in the container need a router to find each
other and to find nodes on the host. Start one on the host or in the
container, as described in [Middleware (Zenoh)](middleware.md).

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| The physical CI job builds with the repository root as context. `COPY ./src` in `Dockerfile.physical` needs the workspace root. | The job fails with `"/src": not found`. In the latest run on `main` (October 2026), the simulation job succeeded and the physical job failed. `walkie-physical:main` on GHCR dates from May 2026. | Build the physical image locally with `just run-real`. |
| `just build-real` also uses the repository root as context. | It fails at `COPY ./src` in the same way. | Use `just run-real`, which builds with the correct context. |
| `just build-sim` and `just build-real` both tag `eic-robocup2026-development:latest`, and no compose file uses that tag. | The second build replaces the first. `run-sim` still pulls from GHCR. | Use `run-sim` and `run-real`. |
| `compose.simulation.yml` uses the tag `feat-github_action_ci-cd`. | It pulls an image from April 2026 instead of the current `main` build. | Change the tag to `main` in a local copy of the file. |
| The repository has no `.dockerignore`. | A local build sends the whole repository folder to Docker, including `.pixi/` (several gigabytes), `build/`, and `install/`. `Dockerfile.simulation` copies all of it into the image. | Build from a clean clone. |
| `Dockerfile.physical` adds `source /home/ros2_ws/install/setup.bash` to `/root/.bashrc`, but the workspace is at `/ros2_ws`. | New shells in the container do not have the workspace packages. | Run `source /ros2_ws/install/setup.bash`. |
| `Dockerfile.physical` builds librealsense from source and later installs `ros-jazzy-librealsense2*` from apt. | Its own comment warns that the two copies can clash. | If they clash, remove one. The comment suggests dropping the source build. |
| `compose.physical.yml` mounts `./models/checkpoint-rs.tar`, which the repository does not contain. | The GraspNet checkpoint is missing in the container. | Put the checkpoint at `docker/models/checkpoint-rs.tar`, or change the left side of the mount to its path. |
| The source folder is mounted from the host, so the container's `uv sync` writes `bringup/.venv` in the host's repository. | The host and the container share one lift venv. | If the lift node fails on the host after you used the container, create the venv again (see [Real Robot](real-robot.md#python-environment)). |
| No compose file starts a Zenoh router. | Nodes in the container cannot find each other until a router runs. | See [Zenoh router](#zenoh-router). |
| [`docker/readme.md`](https://github.com/EIC-Robocup-2026/walkie-ros/blob/main/docker/readme.md) is out of date. It omits `build-real` and `run-remote`, and lists `run-topic-list` only by its alias `list-topics`. Its shell command sources `/ros_entrypoint.sh`, which `just shell` does not do. It says nothing about the image tag, the workspace layout, or pixi. | Its instructions do not match the current files. | Follow this page. |
