# 2.8 Head 3D Lidar

**Goal:** simulate the Unitree 4D L2 lidar at the top of Walkie in MuJoCo, publish its
cloud on the topic and frame of the real driver, and filter the robot out of it the same
way as on the robot.

This entry records how the lidar enters the model, what the simulated cloud looks like
next to the real one, and why it is built this way. For day-to-day use, read
[MuJoCo Simulation](../../guides/simulation-mujoco.md#topics).

## In short

- **Added:** the sensor `unitree_l2` of the MuJoCo extension `mujoco.plugin.lidar` in
  `walkie.xml`, written by `build_mjcf.py`; `ros-jazzy-mujoco-3d-lidar` in `pixi.toml`; a
  `Mujoco3dLidarPlugin` instance that publishes `/unilidar/cloud`; `robot_self_filter` in
  `mujoco_omnibot.launch.py`, publishing `/unilidar/cloud/filtered`; RViz displays; three
  tests in `test_mjcf.py`.
- **Key decisions:** the packaged extension instead of a fork or an own plugin; the sensor
  in the generated `walkie.xml`; 180 x 32 rays at 12 Hz over the hemisphere the L2 sees;
  synchronous ray casting; the 2D lidars stay on `RangefinderLidarPlugin`.
- **Found along the way:** `robot_self_filter` kept the NaN points of an organized cloud in
  its unorganized output and still marked that output dense. It now drops them, which also
  thins the real robot's filtered ZED cloud
  ([The self filter and NaN points](#the-self-filter-and-nan-points)).
- **Result:** both MuJoCo launch files publish `/unilidar/cloud` and
  `/unilidar/cloud/filtered` at about 11.7 clouds per simulated second, in the real
  driver's frame. The filtered cloud has no points on the robot and no NaN. A cloud costs
  about 53 ms and `ros2_control_node` needs about twice the CPU. `mujoco_omnibot.launch.py`
  still runs in real time; with Nav2 the development machine ran at 0.94 to 0.99 of real
  time ([Verification](#verification)).
- **Not done:** Nav2 does not use the cloud in simulation. The simulation parameter file has
  only the `scan` source ([Still open](#still-open)).

## Use it

```bash
pixi install                                            # adds ros-jazzy-mujoco-3d-lidar
pixi run build-pkg robot_bringup walkie_description robot_self_filter
pixi run ros2 launch robot_bringup mujoco_omnibot.launch.py
pixi run ros2 topic hz /unilidar/cloud/filtered          # another shell, same RMW_IMPLEMENTATION
```

RViz shows the filtered cloud as **Head 3D lidar**, colored by height. **Head 3D lidar
(raw)** shows the unfiltered cloud and is off by default.

!!! warning "Every load of `walkie.xml` needs the extension"
    `walkie.xml` declares the extension, so MuJoCo refuses to load it where the extension
    library is missing, with `plugin mujoco.plugin.lidar not found`. The pixi environment
    has it. A MuJoCo outside pixi, for example from `pip`, does not.

## What changed

| File | Change |
| --- | --- |
| `pixi.toml`, `pixi.lock` | `ros-jazzy-mujoco-3d-lidar = ">=0.1.2,<0.2"`. The lock gained two packages, `ros-jazzy-mujoco-3d-lidar` and `ros2-mujoco-3d-lidar` 0.1.2, and changed nothing else. |
| `description/scripts/build_mjcf.py` | Adds the extension instance and its sensor on the site `unitree4d_l2_imu_initial`, and documents the lidar in the `walkie.xml` header |
| `description/mujoco/walkie.xml` | Regenerated: an `<extension>` block and one `<plugin>` sensor |
| `description/test/test_mjcf.py` | `test_lidar_3d_layout`, `test_lidar_3d_sees_floor`, `test_lidar_3d_ranges_to_box` |
| `bringup/config/mujoco/mujoco_plugins.yaml` | The `mujoco_3d_lidar_plugin` instance: sensor `unitree_l2`, frame `unitree4d_l2_imu_initial`, topic `/unilidar/cloud` |
| `bringup/launch/mujoco_omnibot.launch.py` | Starts `robot_self_filter` with the real robot's configuration. `nav_mujoco_omnibot.launch.py` includes this file, so it runs there too. |
| `bringup/config/rviz/mujoco.rviz`, `mujoco_nav.rviz` | **Head 3D lidar** (filtered, on) and **Head 3D lidar (raw)** (off) |
| `bringup/package.xml` | `exec_depend` on `robot_self_filter`, which the real-robot launch already used |
| `hardware/lidar/robot_self_filter/include/robot_self_filter/self_see_filter.h` | Unorganized output drops non-finite points and is dense. Also applies to the real robot's ZED instance ([The self filter and NaN points](#the-self-filter-and-nan-points)). |

## The real lidar and the simulated one { #real-and-simulated }

The simulated cloud has the topic, frame, field of view and rate of the real one. It is
coarser, carries only x, y and z, and keeps one entry per ray.

| | Real L2 (`real_omnibot.launch.py`) | MuJoCo |
| --- | --- | --- |
| Topic | `/unilidar/cloud` | `/unilidar/cloud` |
| Frame | `unitree4d_l2_imu_initial` (`unitree_cloud_frame`) | `unitree4d_l2_imu_initial` |
| Field of view | The hemisphere z ≥ 0 of the frame, 360 x 90 degrees (standard FOV mode; `work_mode` 4) | The same hemisphere |
| Range | 0.05 m to 30 m (data sheet) | 0.05 m to 30 m |
| Rate | About 12 clouds per second: 216 scan lines per second, `cloud_scan_num` 18 per cloud | 12 Hz in simulated time |
| Points per cloud | Varies, unorganized | 5760 entries, organized 180 x 32; NaN where a ray hits nothing |
| Fields | `x`, `y`, `z`, `intensity`, `ring`, `time`; `point_step` 32 | `x`, `y`, `z` (float32); `point_step` 12 |
| Robot in the cloud | Yes, removed by `robot_self_filter` | Yes, removed by the same filter and configuration |

The real rate and point count are derived from the driver settings and the data sheet,
not measured on the robot.

??? info "Where the lidar sits and what it sees"
    `unitree4d_l2_imu_initial` is a child of `base_link` at
    xyz (-0.05196152, 0, 1.676) and rpy (3.36159, -0.445, 2.6607963). The L2 is mounted
    upside down and tilted: the axis of its dome, +z of the frame, points to the rear and
    down, (-0.473, 0.001, -0.881) in `base_link`, 61.7 degrees below the horizontal. The
    team confirmed that the real lidar tilts to the rear.

    The lidar therefore sees everything behind the robot and below it, and the horizon to
    the sides. Straight ahead it sees only what lies more than about 28 degrees below the
    horizontal, which is the floor closer than about 3.2 m. Nothing in front of the robot
    above that line is visible.

    In MuJoCo the frame is a site fixed to `base_link`, as in the URDF. The lift, the head
    and the arms move inside its field of view, so the cloud contains points on the
    robot.

??? info "The ray grid"
    The extension casts a fixed grid of rays: `resolution` "180 32" (azimuth, elevation),
    azimuth about the z axis of the site, elevation from its xy plane, both ranges
    inclusive. Each ray points along (cos az cos el, sin az cos el, sin el) in the site
    frame. The data is stored as 32 elevation rows of 180 azimuth columns.

    The generator puts the rays at the centers of an even grid of cells that covers the
    hemisphere: azimuth from -π + π/180 to π - π/180 in 2 degree steps, elevation from
    π/128 to π/2 - π/128 in 2.8 degree steps. Two simpler choices were rejected:

    - **-π to π:** the first and the last column point the same way.
    - **0 to π/2:** the last row points at the pole, so all 180 rays of that row coincide.

    The cell centers also keep every angle strictly inside [-π, π]. The extension stops
    the process with `mju_error` for a range outside that interval, and the 9 decimals in
    `walkie.xml` could round an angle of exactly π just past it.

## Key decisions

### The packaged extension { #packaged-extension }

`mujoco_3d_lidar` is a MuJoCo extension that casts the rays of one sensor with
`mj_multiRay` during `mj_step`. RoboStack ships it as `ros-jazzy-mujoco-3d-lidar` 0.1.2,
built in build set `_25` against libmujoco 3.15.0, the version the workspace pins.
`Mujoco3dLidarPlugin` of `mujoco_ros2_control_plugins` 0.1.2, already installed, turns its
readings into a `PointCloud2`.

| Option | Why it was or was not chosen |
| --- | --- |
| **Packaged extension (chosen)** | No new C++. `pixi add` only added two packages. The MJCF and the YAML keep working when a later version fixes the issues below. Costs: the ray count is limited by the real-time factor, the cloud has x, y and z only, and every MuJoCo run casts the rays. |
| Patched fork of the extension | A `geomgroup` or body-exclude key would skip the robot's own meshes, about 12 times cheaper per ray in a Python benchmark, and the async bug could be fixed. Costs: a C++ fork to maintain, and its install step writes into the pixi environment. Better as an upstream change. |
| Own `mujoco_ros2_control` plugin | Could cast on a worker thread and send the real driver's fields (`intensity`, `ring`, `time`) and scan pattern. The largest job, on a plugin API that is still 0.1.x, and no consumer needs those fields today. |

??? info "How the pieces find each other"
    - The package installs three copies of `libmujoco_3d_lidar.so`. Two matter:
      `$CONDA_PREFIX/bin/mujoco_plugin/`, which the `mujoco` Python package loads on
      import, and `$CONDA_PREFIX/lib/mujoco_3d_lidar/mujoco_plugin/`, the path registered
      in the ament index under `mujoco_plugins`, where `ros2_control_node` of
      `mujoco_ros2_control` finds it. The third sits in `$CONDA_PREFIX/lib/mujoco_3d_lidar/`.
      No environment variable is needed.
    - `Mujoco3dLidarPlugin` publishes every sensor of `mujoco.plugin.lidar` in the model.
      Its parameters are `mujoco_plugins.<instance>.<sensor>.frame_name` (default: the
      site name) and `.topic` (default: `/points`). Resolution, field of view, range and
      rate come from `walkie.xml`, not from the YAML file.
    - The plugin publishes from the hardware `write()`, which runs at the controller
      manager rate, 60 Hz. It publishes a cloud only when the extension has computed a new
      one, so the topic carries at most one cloud per computation, about 11.9 per simulated
      second. The stamp is the simulated time of the computation.
    - The publisher is reliable with depth 1. `robot_self_filter` and the RViz displays
      subscribe best effort, which works with a reliable publisher.

### The sensor lives in walkie.xml { #in-walkie-xml }

`build_mjcf.py` adds the sensor, so every world, the viewer and `test_mjcf.py` get it
without extra steps, and `test_generated_model_is_up_to_date` guards it like the rest of
the model. The alternative was a separate include file that only some worlds load. It
would keep `walkie.xml` free of the extension, but every custom world would need the
include by hand, and the sensor would sit outside the generator.

The cost: every MuJoCo run casts the rays, also a manipulation run that does not need the
cloud, and the extension is a hard dependency of `walkie.xml`.

The generator adds the sensor last, after the keyframe, so the forward passes of the
earlier steps do not cast its rays. Before it adds the sensor, it checks that the extension
is loaded and stops with `MuJoCo extension 'mujoco.plugin.lidar' is not available; install
ros-jazzy-mujoco-3d-lidar (pixi install)` otherwise.

### 180 x 32 rays at 12 Hz, synchronous { #resolution-and-rate }

The extension casts every ray against every geom of the model, including the visual
meshes of the robot. The rays cost simulation time, so the resolution is a trade between
density and the real-time factor.

180 x 32 rays at 12 Hz keep `mujoco_omnibot.launch.py` at a real-time factor of 1.0 on
the development machine, with little margin: a cloud costs about 53 ms of wall time, so
the simulation stalls for that long every 84 ms of simulated time and catches up in the
steps between. With Nav2 (`nav_mujoco_omnibot.launch.py`) it ran at 0.94 to 0.99, and a
slower machine falls further below real time. The cost is linear in the ray
count, about 9.3 µs per ray, because every ray is tested against the robot's visual
meshes too.

??? info "Measured cost (2026-10-16)"
    Measured on a 12-core laptop with 15 GB of RAM and other applications open. The
    Python rows step `scene.xml` from `home` without ROS; the ROS rows are headless
    launches with `rmw_zenoh_cpp`, partly repeated after the merge of `main` and
    `navigation2` `3286119`.

    | | Without the 3D lidar | 180 x 32 at 12 Hz |
    | --- | --- | --- |
    | Wall time of the step that computes a cloud (Python) | - | 53 ms (48 to 70 ms) |
    | Mean wall time per step (Python) | 0.10 ms | 1.35 ms |
    | Fastest possible real-time factor, physics and lidar only (Python) | about 20 | about 1.5 |
    | CPU of `ros2_control_node`, `mujoco_omnibot.launch.py` | 56 % | 106 to 113 % |
    | CPU of `ros2_control_node`, `nav_mujoco_omnibot.launch.py` | 66 % | 114 to 119 % |
    | Real-time factor, `mujoco_omnibot.launch.py` | 1.00 | 0.99 to 1.00 |
    | Real-time factor, `nav_mujoco_omnibot.launch.py` | 1.00 | 0.94 to 0.99 |
    | `/clock` messages per second | about 500 | about 340; about 270 with Nav2 |

    Other resolutions, same angle ranges (Python): 180 x 16 costs 26.9 ms per cloud
    (fastest real-time factor 2.7), 90 x 32 costs 29.0 ms (2.5). To lower the cost,
    change `LIDAR_3D_RESOLUTION` or `LIDAR_3D_UPDATE_RATE` in `build_mjcf.py`, run
    `pixi run mjcf`, then `test_mjcf.py`.

The rate matches the real lidar. The extension throttles on simulated time, so at a time
step of 0.002 s a cloud is computed every 42 steps (0.084 s), about 11.9 Hz.

The extension also has an asynchronous mode (`async`), which casts on a thread. It is off:
in 0.1.2 the thread writes the readings while the plugin reads them, so a published cloud
can mix two scans.

### The 2D lidars stay on RangefinderLidarPlugin

`RangefinderLidarPlugin` is deprecated upstream in favor of `Mujoco3dLidarPlugin`. The 2D
lidars still use it. Their rangefinder sensors skip `base_link`, the body they sit on. The
extension cannot skip any geom, and every 2D beam would end about 2 cm away, on the visual
mesh of `base_link`.

## The self filter and NaN points { #the-self-filter-and-nan-points }

The simulated cloud marks every ray without a hit with NaN coordinates. Before this change,
`robot_self_filter` passed these points through: a NaN point is outside every body, so the
filter keeps it. With `keep_organized: false`, as in `unitree_self_filter.yaml`, its output
is an unorganized list of points, but it still contained the NaN points and was marked
dense (`is_dense: true`). A consumer that trusts the flag, such as many PCL algorithms,
would read NaN coordinates.

The filter now drops non-finite points from unorganized output and sets `is_dense: true`
only there. Organized output (`keep_organized: true`) keeps one entry per input point and is
marked not dense.

!!! warning "The real robot's ZED filtered cloud changes too"
    The fix is in the shared filter code, so it applies to every `robot_self_filter`
    instance with `keep_organized: false`. On the real robot:

    - **L2** (`unitree_self_filter.yaml`): no change. The driver drops readings outside its
      range instead of sending NaN points.
    - **ZED head camera** (`zed2i_self_filter.yaml`): the input is organized, with NaN or
      inf for every pixel without valid depth. `/zed_head/zed_node/point_cloud/cloud_registered/filtered`
      no longer contains those points, so it has fewer points per cloud and is marked
      dense. Before, the MoveIt octomap (`sensors_3d.yaml`) skipped NaN points but not inf
      points, and the STVL voxel filter of the real and restaurant parameter files trusted
      the dense flag when it computed its bounds; both now get only finite points.
      `cloud_to_map_node` only transforms the cloud, so `filtered_map` changes the same way.
    - **D415** (`d415_bottom_self_filter.yaml`, also `keep_organized: false`): not started,
      because `real_omnibot.launch.py` comments out the camera and its filter. If they are
      enabled again, the same change applies whenever the input contains non-finite points.

    Not yet checked on the robot.

## Verification

Run on 2026-10-16 after `pixi run build`, on the tree with `main` and `navigation2`
`3286119` merged, headless, with `rmw_zenoh_cpp` on a private router. A probe node
subscribed best effort to both clouds and `/clock` for 30 s (40 s with Nav2) and
transformed the points into `base_footprint`. The box world ran before the merge, with
the same model and extension.

| Check | Result |
| --- | --- |
| `test_description.py` and `test_mjcf.py` | 101 passed, 1 strict xfail (a known controller pairing) |
| `colcon test` of the packages CI tests (`walkie_description`, `openarm_bimanual_moveit_config`, `topic_based_ros2_control`) | 106 tests, no failures, 1 skipped |
| `pixi run docs-build` | Passes |
| `/unilidar/cloud`, empty `scene.xml`, `mujoco_omnibot.launch.py` | Real-time factor 1.00. 353 clouds in 30 s, stamps 0.084 s apart, 11.8 clouds per simulated second. Frame `unitree4d_l2_imu_initial`, 180 x 32, `point_step` 12, fields `x`, `y`, `z`. 707 NaN entries: the rays above the horizon, which hit nothing in the empty world. 1191 points (21 %) inside the self-filter box. |
| `/unilidar/cloud/filtered`, same run | One cloud per input cloud. 3862 points, unorganized, `is_dense` true, no NaN, none inside the box: only floor points (z from -0.002 to 0.007 m in `base_footprint`), 1.86 m to 29.9 m away |
| World with a box beside the robot, its face at y = 1.4 m | 198 points above the floor beside the robot, the nearest at y = 1.400 m, in both clouds |
| `nav_mujoco_omnibot.launch.py`, empty world | Nav2 active. Real-time factor 0.94 and 0.99 in two runs (1.00 without the 3D lidar). 420 clouds in 40 s, 11.2 per simulated second. The same raw and filtered layout as without Nav2. |
| Launch logs | No errors before shutdown. `ros2_control_node` loads the extension from the `mujoco_3d_lidar` package and adds the sensor `unitree_l2` on `/unilidar/cloud`. |

The expected rate is 11.9 clouds per second. About 2 % of the clouds are not published:
the stamps then jump by 0.168 s. Both topics miss the same clouds, so they are lost
before the self filter, not by the probe. The cause was not investigated.

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| The rays hit the robot | About 21 % of the rays end on the robot in the `home` pose. The self filter removes them from `/unilidar/cloud/filtered`. | Use the filtered topic |
| No `intensity`, `ring` or `time` fields | A node that needs them gets no data | None. No node in the repository reads them. |
| Cost grows with the ray count | A cloud stalls the simulation for about 53 ms. The fastest possible real-time factor drops from about 20 to about 1.5. With Nav2 the development machine ran at 0.94 to 0.99, and a slower machine falls further behind. | Fewer rays, for example `LIDAR_3D_RESOLUTION = (180, 16)` in `build_mjcf.py` ([measured cost](#resolution-and-rate)) |
| About 2 % of the clouds are missing | The stamps jump by 0.168 s instead of 0.084 s now and then | None needed for a 12 Hz lidar |

## Still open

- **Nav2 with the cloud.** The real and restaurant parameter files read
  `/unilidar/cloud/filtered` in the costmaps and in `collision_monitor`. The simulation file
  is shared with Gazebo, where a source without data stops every Nav2 command after 2 s.
  Using the cloud in MuJoCo needs a MuJoCo-only parameter file.
- **An upstream exclude key.** A `geomgroup` or body-exclude key in `mujoco_3d_lidar` would
  skip the robot's own geoms and make each ray much cheaper.
