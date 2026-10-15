# Docker Images

Run the Walkie simulation or the physical robot stack in Docker with the
compose files in `docker/`, and pull the images that CI published.

!!! warning "CI no longer builds the images"
    `.github/workflows/build-docker.yml` was removed on 2026-10-15. The images
    on GHCR are the last ones it published and do not follow `main`. CI now
    builds the workspace with pixi instead (see [Tests: CI](testing.md#ci)).

!!! note "The images do not use pixi"
    The images install ROS 2 Jazzy from apt, resolve dependencies with
    `rosdep`, and build with `colcon`. They ignore `pixi.toml` and
    `pixi.lock`, so package versions can differ. For example, the images use
    Ubuntu's OpenCV 4.6, and the pixi environment has OpenCV 5.0. Code that
    uses OpenCV must work with both. For daily development, use
    [pixi](pixi.md).

## Before you start

- Install Docker and [just](https://github.com/casey/just). Run the recipes
  from the `docker/` folder. The recipes that run Docker use `sudo docker`.
- Work in a terminal of your desktop session ([X11 access](#x11-access)) and
  start a Zenoh router ([Zenoh router](#zenoh-router)).

## Images

| Image | Dockerfile | Base image | Purpose |
| --- | --- | --- | --- |
| `ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation` | `docker/Dockerfile.simulation` | `osrf/ros:jazzy-desktop-full` | Gazebo simulation |
| `ghcr.io/eic-robocup-2026/walkie-ros/walkie-physical` | `docker/Dockerfile.physical` | `stereolabs/zed:5.2-gl-devel-cuda12.8-ubuntu24.04` | Physical robot: ZED SDK, CUDA, RealSense, GraspNet |

??? info "Compose files, services, and container names"
    | File | Service | Container name | Image |
    | --- | --- | --- | --- |
    | `docker/compose.simulation.yml` | `robot_simulation` | `robot-simulation` | `ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation:feat-github_action_ci-cd` (pulled) |
    | `docker/compose.physical.yml` | `zed_wrapper` | `robot_physical-zed_wrapper` | `zed-docker-zed_wrapper:latest` (built locally) |
    | `docker/compose.remote.yml` | `docker-remote-desktop` | `docker-remote-desktop` | `ghcr.io/lanjelin/docker-remote-desktop:latest` (pulled) |

    Relative paths in the compose files, such as `../`, are relative to the
    `docker/` folder.

## Run the simulation container

```bash
cd docker
just run-sim
```

The container builds the workspace and launches `gzsim_omnibot.launch.py`
(Gazebo). Stop it with `just stop-sim`.

`compose.simulation.yml` pulls the old tag `feat-github_action_ci-cd`, an
April 2026 build. For a newer image, change the tag to `main` in a local copy
of the file ([Known issues](#known-issues)). That tag is the last build of
`main` before the Docker workflow was removed.

??? info "What `compose.simulation.yml` configures"
    - Pulls the simulation image with the tag `feat-github_action_ci-cd`. That
      tag comes from an old CI branch. The removed Docker workflow published
      the tag `main` from `main`.
    - Uses the host network and host IPC, and passes `/dev/dri` for graphics.
    - Mounts the X11 socket (see [X11 access](#x11-access)).
    - Mounts the repository at `/ros2_ws/src`. The build, install, and log
      folders are named volumes, so they survive container restarts.
    - On start, runs `colcon build --symlink-install --parallel-workers 1` and
      then `ros2 launch robot_simulation gzsim_omnibot.launch.py`.
    - Sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` (see
      [Zenoh router](#zenoh-router)).

## Run the physical robot container { #composephysicalyml }

Before the first run:

- Place the repository at `<workspace>/src/walkie-ros`. The image needs the
  workspace root as build context (see
  [How the images are built](#how-the-images-are-built)).
- Put the GraspNet checkpoint at `docker/models/checkpoint-rs.tar`, or change
  the left side of that mount in `compose.physical.yml`. The repository does
  not contain this file.

Steps:

1. Start the container. If the image does not exist yet, compose builds it
   first with the correct context.

    ```bash
    cd docker
    just run-real
    ```

    On every start, `docker/entrypoint.sh` builds `/ros2_ws` and runs
    `uv sync` in `bringup/`. Then it waits. It does not launch the robot.

2. In a second terminal, open a shell in the container:

    ```bash
    cd docker
    just shell robot_physical-zed_wrapper
    ```

3. In every new container shell, source the workspace. The image's
   `/root/.bashrc` uses a wrong path ([Known issues](#known-issues)).

    ```bash
    source /ros2_ws/install/setup.bash
    ```

4. Start the robot from that shell (see [Real Robot](real-robot.md)).
5. Stop the container with `just stop-real`.

The container sets `ROS_DOMAIN_ID=23` and `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
Nodes on the host need the same values to see the container's nodes.

??? info "What `compose.physical.yml` and `entrypoint.sh` configure"
    - Builds `zed-docker-zed_wrapper:latest` locally from `Dockerfile.physical`
      with the context `../../..`. This is the workspace root when the
      repository sits at `<workspace>/src/walkie-ros`.
    - Uses the NVIDIA runtime and reserves all GPUs.
    - Runs privileged with host network, IPC, and PID, mounts `/dev`, and adds
      the `dialout` group. This gives the container the CAN interfaces, serial
      ports, and USB cameras.
    - Sets `ROS_DOMAIN_ID=23` and `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
    - Mounts the X11 socket (see [X11 access](#x11-access)).
    - Mounts `../../../src` at `/ros2_ws/src`. The build, install, and log
      folders are named volumes, as in the simulation file.
    - Mounts `./models/checkpoint-rs.tar` as the GraspNet checkpoint.
    - Keeps the ZED AI models and settings in the named volumes
      `zed_resources` and `zed_settings`, so the ZED SDK does not optimize the
      models again on every start.
    - Runs `docker/entrypoint.sh`. It does these steps, then waits with
      `sleep infinity`:
        1. Source `/opt/ros/jazzy` and the ZED workspace.
        2. Build `/ros2_ws` with `colcon build --symlink-install`.
        3. Run `uv sync` in `src/walkie-ros/bringup`. This creates
           `bringup/.venv` for the lift node (see [Real Robot](real-robot.md)).

## Run the remote desktop container

```bash
cd docker
just run-remote
```

`compose.remote.yml` runs a remote desktop container on ports 3000 (HTTP) and
3001 (HTTPS). Its volume `/path/to/config:/config` is a placeholder. Change it
to a real folder before you start the container.

## X11 access

`compose.simulation.yml` and `compose.physical.yml` require `DISPLAY`.
Without it, compose stops with an error. Run them from a terminal in your
desktop session.

`just run-sim` and `just run-real` run `xhost local:root` for you. The setting
stays active until the X server restarts. To remove it earlier:

```bash
xhost -local:root
```

??? info "Why the containers need `xhost local:root`"
    The containers run as root and open windows on your desktop (Gazebo,
    RViz). `xhost local:root` allows local clients, including root in the
    container, to connect to your X server.

## Zenoh router

`compose.simulation.yml` and `compose.physical.yml` set
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`, but neither starts a Zenoh router. Nodes in the container need a router to find each other and to
find nodes on the host. Start one on the host or in the container, as
described in [Middleware (Zenoh)](middleware.md). On the host:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
```

## just recipes

`docker/justfile` holds shortcuts for the Docker commands on this page. Use
`run-sim` and `run-real` to build and start. Avoid `build-sim` and
`build-real`: they have problems with their tag and context (see
[Known issues](#known-issues)).

??? info "All recipes and the commands they run"
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

## Pull the published images

The images on GHCR are public. Until 2026-10-15,
`.github/workflows/build-docker.yml` built them on every push to `main` and
pushed the tag `main`:

```bash
docker pull ghcr.io/eic-robocup-2026/walkie-ros/walkie-simulation:main
```

!!! warning
    No workflow updates these tags anymore, so they fall behind `main`.
    `walkie-physical:main` is older still: the physical job always failed
    (see [Known issues](#known-issues)), and the image dates from May 2026.
    Build the physical image locally with `just run-real`.

## How the images are built

- `docker/Dockerfile.simulation` expects the repository root as build
  context and builds the workspace into the image.
- `docker/Dockerfile.physical` expects the workspace root as build context.
  It does not build the walkie workspace. `docker/entrypoint.sh` does that
  when the container starts.

!!! warning "Build context of the physical image"
    `COPY ./src /ros2_ws/src/` needs a context with this repository at
    `src/walkie-ros`, the layout of a classic colcon workspace.
    `compose.physical.yml` uses this layout. The repository root has no `src/`
    folder, so builds from it fail ([Known issues](#known-issues)).

??? info "`Dockerfile.simulation` build steps"
    1. Copy the whole repository to `/ros2_ws/src`.
    2. Install `python3-pip`, `ros-dev-tools`, and
       `ros-jazzy-teleop-twist-keyboard` with apt.
    3. Run `rosdep update`, then `rosdep install --from-paths src -y -i` to
       install the dependencies declared in the `package.xml` files.
    4. Install `ros-jazzy-rmw-zenoh-cpp` with apt.
    5. Build the workspace with
       `colcon build --symlink-install --parallel-workers 3`.
    6. Remove `ros-dev-tools` and clean the apt cache to make the image
       smaller.

??? info "`Dockerfile.physical` build steps"
    The image starts from the Stereolabs ZED image, which already has CUDA
    and the ZED SDK.

    1. Set the locale, add the ROS 2 apt repository, and install
       `ros-jazzy-desktop`, `ros-jazzy-ros-base`, colcon, rosdep, and
       vcstool.
    2. Install point cloud transport, `rmw_zenoh_cpp`, `topic_tools`, and
       `foxglove_bridge` with apt.
    3. Build librealsense from source.
    4. Run `rosdep init` and `rosdep update`.
    5. Clone and build `zed-ros2-wrapper` and `zed-ros2-examples` in a
       separate workspace, `/home/zed_deps_ws`.
    6. Install uv to `/root/.local/bin`.
    7. Copy `./src` from the build context to `/ros2_ws/src`. Install the
       RealSense ROS packages, PCL, and the `rosdep` dependencies of the
       source.
    8. Set up GraspNet: run `perception/setup_env.sh --grasp`, which creates
       `~/perception-venv`, then build the GraspNet CUDA extensions with
       `TORCH_CUDA_ARCH_LIST="12.0"`.
    9. Add `source` lines to `/root/.bashrc`.

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| The removed physical CI job used the repository root as context | The job failed, so `walkie-physical:main` on GHCR dates from May 2026 | Build locally with `just run-real` |
| `just build-real` uses the repository root as context | Fails at `COPY ./src` | Use `just run-real` |
| `build-sim` and `build-real` both tag `eic-robocup2026-development:latest`, which no compose file uses | The second build replaces the first. `run-sim` still pulls from GHCR. | Use `run-sim` and `run-real` |
| `compose.simulation.yml` uses the tag `feat-github_action_ci-cd` | Pulls an April 2026 image, not the current `main` build | Change the tag to `main` in a local copy |
| No `.dockerignore` | Local builds send `.pixi/` (several gigabytes), `build/`, and `install/` to Docker | Build from a clean clone |
| `Dockerfile.physical` adds `source /home/ros2_ws/install/setup.bash` to `/root/.bashrc`, but the workspace is at `/ros2_ws` | New shells lack the workspace packages | `source /ros2_ws/install/setup.bash` |
| `Dockerfile.physical` builds librealsense from source and also installs it from apt | The two copies can clash | If they clash, remove one. The Dockerfile comment suggests dropping the source build. |
| `compose.physical.yml` mounts `./models/checkpoint-rs.tar`, which the repository does not contain | No GraspNet checkpoint in the container | Put it at `docker/models/checkpoint-rs.tar`, or change the left side of the mount |
| The container's `uv sync` writes the host's `bringup/.venv` | Host and container share one lift venv | If the lift node then fails on the host, create the venv again ([Real Robot](real-robot.md#python-environment)) |
| No compose file starts a Zenoh router | Container nodes cannot find each other until a router runs | [Zenoh router](#zenoh-router) |
| [`docker/readme.md`](https://github.com/EIC-Robocup-2026/walkie-ros/blob/main/docker/readme.md) is out of date | Its instructions do not match the current files | Follow this page |

??? info "Known issue details"
    - **Physical CI job:** `COPY ./src` in `Dockerfile.physical` needs the
      workspace root as context, so the job failed with `"/src": not found`.
      In the last runs on `main` (October 2026), the simulation job
      succeeded and the physical job failed.
    - **No `.dockerignore`:** the local build sends the whole repository
      folder to Docker, and `Dockerfile.simulation` copies all of it into the
      image.
    - **`Dockerfile.physical` and librealsense:** it builds librealsense from
      source and later installs `ros-jazzy-librealsense2*` from apt. Its own
      comment warns that the two copies can clash.
    - **Shared lift venv:** the source folder is mounted from the host, so
      the venv that `uv sync` creates in the container is the host's
      `bringup/.venv`.
    - **`docker/readme.md`:** it omits `build-real` and `run-remote`, and lists
      `run-topic-list` only by its alias `list-topics`. Its shell command
      sources `/ros_entrypoint.sh`, which `just shell` does not do. It says
      nothing about the image tag, the workspace layout, or pixi.
