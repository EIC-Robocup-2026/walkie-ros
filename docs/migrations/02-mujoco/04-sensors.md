# 2.4 Sensors

The MuJoCo model carries the same sensors as the Gazebo model: two 2D lidars, one IMU
and two cameras. This entry records how each one is built and what is still missing
before they publish ROS topics.

**Goal:** give the MuJoCo model sensors that match the URDF frames exactly, so that
any ROS code that works on the real robot or in Gazebo sees the same frames in MuJoCo.

## In short

- The URDF stays the single source for sensor frames, ranges and resolutions. The
  generator copies its values instead of hard-coding them.
- Each lidar is a fan of 241 `rangefinder` sensors on `base_link`, because MuJoCo has
  no 2D lidar sensor.
- The IMU is a site on `top_imu_link`, wired through ros2_control. Each camera sits on
  its URDF optical frame.
- Result: all sensors exist in `walkie.xml` and `test_mjcf.py` checks them. Lidar and
  camera publishing works with a plugin parameter file, and the IMU topic needs a remap.
  Both were added to the repository with the launch files on 2026-10-13
  ([Still open](#still-open)).

!!! note "Update 2026-10-13: launch files added"
    With `bringup/launch/mujoco_omnibot.launch.py`, every sensor publishes on a ROS topic.
    The launch file loads the plugin parameter file
    `bringup/config/mujoco/mujoco_plugins.yaml`, merges the two lidars into `/scan` with
    `dual_laser_merger`, and remaps the IMU to `/imu`. The updates in the sections below
    give the details.
    [MuJoCo Simulation: Run with ROS 2](../../guides/simulation-mujoco.md#run-with-ros-2)
    has the commands, and [2.7 Launch Files and Nav2](07-launch.md) records the launch
    files.

## Sensors at a glance

`description/scripts/build_mjcf.py` reads the lidar and camera values from the Gazebo
`<sensor>` elements in the expanded URDF, and places the IMU on `top_imu_link` directly.
For the current numbers, read the header comment of `description/mujoco/walkie.xml`
first.

| Sensor | MJCF elements | ROS frame | Real device | Gazebo sensor used |
| --- | --- | --- | --- | --- |
| Front lidar | 241 `rangefinder` sensors `front_lidar-0` to `front_lidar-240` | `front_lidar_link` | Hokuyo URG (`urg_node2`) | `front_lidar` |
| Back lidar | 241 `rangefinder` sensors `back_lidar-0` to `back_lidar-240` | `back_lidar_link` | Lakibeam (`lakibeam1`) | `back_lidar` |
| IMU | site `top_imu`, sensors `top_imu_quat`, `top_imu_gyro`, `top_imu_accel` | `top_imu_link` | not covered here | none |
| Head camera | camera `head_camera` | `zed_head_left_camera_frame_optical` | ZED 2i (`zed_wrapper`) | `head_rgbd_sensor` |
| Bottom camera | camera `bottom_camera` | `bottom_color_optical_frame` | RealSense D415 (`realsense2_camera`) | `bottom_depth_sensor` |

## Lidars

### Build each lidar from rangefinders

Each lidar is a fan of rangefinders, with one site and one sensor per beam. MuJoCo has
no 2D lidar sensor. Its `rangefinder` is one ray along the +Z axis of a site. It reports
the distance to the first geom it hits, or -1 when it hits nothing.

### Beam layout

- 241 beams from -2.09333333 to 2.09333333 rad (about ±120 degrees), range 0.18 m to
  10 m, 40 Hz. Beam 0 points at `min_angle` and the last beam at `max_angle`, as a
  `sensor_msgs/LaserScan` consumer expects.
- The increment in the header is rounded **down** to 12 digits, so that a publisher
  recomputes 241 beams and not 240.
- No noise. Each beam is a separate sensor and ray cast, so the beam count is the main
  cost of the lidar emulation.

??? info "Full beam layout and where each value comes from"
    | Property | Value | Where it comes from |
    | --- | --- | --- |
    | Beams per lidar | 241 | The even spacing closest to `LIDAR_RESOLUTION_DEG = 1.0` |
    | Angular range | -2.09333333 to 2.09333333 rad (about ±120 degrees) | URDF `min_angle` and `max_angle` |
    | Angle increment | 0.017444444416 rad (about 0.9995 degrees) | Range divided by 240 intervals |
    | Range | 0.18 m to 10 m | URDF `<range>` `min` and `max` |
    | Update interval | 0.025 s (40 Hz) | URDF `update_rate` 40 |
    | Sensor names | `<lidar>-<index>`, for example `front_lidar-0` | Naming scheme of `RangefinderLidarPlugin` |
    | Site names | `<lidar>_ray_<index>`, for example `front_lidar_ray_000` | Generator |
    | Body | `base_link` | `LIDAR_RAY_BODY` |
    | Site group | 5 (hidden in the viewer by default) | `LIDAR_SITE_GROUP` |

    The first and last beams satisfy `min_angle + (count - 1) * increment == max_angle`.

    Each site is rotated by +90 degrees about Y, so the site +Z axis points along the
    beam direction (+X of the beam frame).

??? info "Why the increment is rounded down"
    The increment is rounded down to 12 digits (`LIDAR_INCREMENT_DIGITS`). A publisher
    that computes the beam count as `int((max_angle - min_angle) / increment) + 1` then
    gets 241 and not 240. Rounding to the nearest value could push the quotient below
    240 and lose the last beam.

??? info "Comparison with the Gazebo lidar"
    The Gazebo lidar in `description/urdf/walkie/lidar/lidar.gz.xacro` uses 360 samples
    over the same 240 degrees (about 0.67 degrees per sample). It adds Gaussian noise
    with a standard deviation of 0.01 m. The MuJoCo lidar uses 1 degree spacing and
    adds no noise.

### Put the beam sites on base_link

The beam sites sit on `base_link`, at the pose of `front_lidar_link` and
`back_lidar_link`, so the beams cannot hit the robot's own base. A MuJoCo rangefinder
ignores the geoms of the body that owns its site, and `base_link` owns the base shell
collision geometry around both scanners.

??? info "Site positions, and why not the lidar link bodies"
    If the sites sat on the lidar link bodies, every beam that passes the base shell
    would report a hit on the robot itself.

    | Lidar | Site position in `base_link` | Height above the floor at `home` |
    | --- | --- | --- |
    | `front_lidar` | (0, -0.235, 0.28232) | 0.307 m |
    | `back_lidar` | (0, 0.235, 0.28232) | 0.307 m |

    `base_link` is rotated by +90 degrees about Z relative to `base_footprint`. The
    front lidar is therefore 0.235 m ahead of the robot center and the back lidar
    0.235 m behind it.

    !!! note
        Since description `main` was merged (description `f6649ee`), `base_link` is
        aligned with `base_footprint`. The sites are now at (0.235, 0, 0.28232) and
        (-0.235, 0, 0.28232) in `base_link`. Their positions in the world are unchanged.

### Evaluate each beam once per scan

Every rangefinder has `interval="0.025 0"`, so MuJoCo casts each beam once per scan
period instead of every physics step. MuJoCo only honors `interval` for sensors with a
history buffer, which is why `nsample` is 1.

??? info "The rangefinder element and the saving"
    ```xml
    <rangefinder site="front_lidar_ray_000" name="front_lidar-0" nsample="1" interval="0.025 0"/>
    ```

    With `interval`, MuJoCo evaluates the sensor once per scan period (0.025 s) instead
    of every physics step (0.002 s). That is about 12.5 times fewer ray casts.

### Keep the beams out of camera images

MuJoCo draws every beam that hits something, in the viewer and in camera images. The
generator makes the beams fully transparent (`RANGEFINDER_RGBA`, alpha 0), so they stay
out of color images.

!!! warning
    Depth images still contain the beams unless the renderer disables the
    `mjVIS_RANGEFINDER` flag.

### Mapping to the real lidars

On the real robot, `bringup/launch/dual_lidar_hokuyo_lakibeam.launch.py` runs
`dual_laser_merger`. It merges `/front_lidar` and `/back_lidar` into one scan in the
frame `merged_lidar_link` and remaps the output to `/scan`. Nav2 reads `/scan`.

??? info "Real lidar settings next to the MuJoCo values"
    The real values are in `bringup/config/lidar/dual_lidar_config.yml`.

    | | Front | Back |
    | --- | --- | --- |
    | Real device | Hokuyo URG, driver `urg_node2` | Lakibeam, driver `lakibeam1` |
    | Real frame | `front_lidar_link` | `back_lidar_link` |
    | Real topic | `/front_lidar` | `/back_lidar` |
    | Real angular range | `angle_min` / `angle_max` ±2.0933 rad | `scan_range_start` 60 to `scan_range_stop` 300 (240 degrees) |
    | Real scan rate | not set in the config | `scanfreq` 30 |
    | MuJoCo angular range | ±2.09333333 rad | ±2.09333333 rad |
    | MuJoCo rate | 40 Hz | 40 Hz |

### What /scan will need

!!! note "Update 2026-10-13: /scan comes from dual_laser_merger"
    `mujoco_omnibot.launch.py` builds `/scan` from both parts listed below:

    - `RangefinderLidarPlugin`, configured in `bringup/config/mujoco/mujoco_plugins.yaml`,
      publishes `/front_lidar` (frame `front_lidar_link`) and `/back_lidar` (frame
      `back_lidar_link`).
    - `dual_laser_merger` runs with the same parameters as on the real robot. It remaps
      `/merged_lidar` to `/scan`, in the frame `merged_lidar_link`.
    - The merger publishes only when both lidars hit something. The default `scene.xml`
      has only a floor, so it has no `/scan`. Nav2 still runs there; its obstacle layers
      see nothing. In a walled test room, `/scan` published at about 27.8 Hz.
    - The launch files keep the deprecated `RangefinderLidarPlugin`. Its replacement,
      `Mujoco3dLidarPlugin`, needs the `mujoco_3d_lidar` MuJoCo extension, which is not
      in the pixi environment.

    [MuJoCo Simulation: The mujoco_plugins parameter file](../../guides/simulation-mujoco.md#the-mujoco_plugins-parameter-file)
    has the file, and [2.7 Launch Files and Nav2](07-launch.md) records the launch files.

!!! note "Update 2026-10-14: changed in entry 5"
    With `publish_on_empty_cloud`, a new merger parameter that `mujoco_omnibot.launch.py`
    sets to `true`, the merger publishes `/scan` in the empty `scene.xml`. A lidar with no
    return in range reads as free space. The real robot keeps the default, `false`. See
    [5. Repair Nav2 After the navigation2 Bump](../05-nav2-repair.md#scan-in-the-empty-mujoco-world).

When this entry was written, no launch file published the MuJoCo lidars on ROS topics.
Two parts were missing:

1. **A LaserScan publisher per lidar**, publishing `/front_lidar` and `/back_lidar`.
2. **The merger.** Run `dual_laser_merger` with the same parameters as on the real
   robot, so `/scan` is built the same way in both cases.

!!! warning "RangefinderLidarPlugin is deprecated upstream"
    The generator names the sensors for `RangefinderLidarPlugin` from
    `mujoco_ros2_control_plugins` 0.1.2. The alternative, `Mujoco3dLidarPlugin`, has
    not been tried with Walkie. Choosing between the two is an open decision.

    **Update 2026-10-13:** decided in [2.7 Launch Files and Nav2](07-launch.md). The launch
    files use `RangefinderLidarPlugin`.

MuJoCo does not clip readings to the range. A beam that hits nothing reads -1, and a
beam can read more than 10 m. The publisher must treat both as "no return".

??? info "Upstream deprecation notices and the alternative plugin"
    In `mujoco_ros2_control_plugins` 0.1.2 the plugin description reads
    "[Deprecated] Wraps MuJoCo rangefinder sensors as ROS 2 LaserScan publishers." The
    upstream demo config adds: "This is included for test purposes only. We do not
    recommend users use this plugin." The 0.1.2 core library also logs a deprecation
    message when rangefinder lidars come from the ros2_control xacro config.

    `Mujoco3dLidarPlugin` republishes the data of every MuJoCo simulation extension
    registered as `mujoco.plugin.lidar`. The `mujoco_3d_lidar` package is available on
    RoboStack build set `_25` but is not in `pixi.toml`.

??? info "RangefinderLidarPlugin parameters for Walkie, and open questions"
    The parameter names come from the upstream demo file
    `mujoco_ros2_control_demos/config/mujoco_ros2_control_plugins.yaml`. They sit under
    `mujoco_plugins.<instance name>` in a parameter file of the `ros2_control_node`.

    | Parameter | Upstream demo value | Walkie value (from the `walkie.xml` header) |
    | --- | --- | --- |
    | `type` | `mujoco_ros2_control_plugins/RangefinderLidarPlugin` | same |
    | `lidar_publish_rate` | 5.0 | 40 |
    | `lidar.frame_name` | `lidar_sensor_frame` | see the open questions below the table |
    | `lidar.angle_increment` | 0.025 | 0.017444444416 |
    | `lidar.min_angle` | -0.3 | -2.09333333 |
    | `lidar.max_angle` | 0.3 | 2.09333333 |
    | `lidar.range_min` | 0.05 | 0.18 |
    | `lidar.range_max` | 10.0 | 10 |
    | `lidar.laserscan_topic` | `/scan` | `/front_lidar` and `/back_lidar` |

    Open questions:

    - How `RangefinderLidarPlugin` finds the sensors of one lidar: whether it uses
      `frame_name` as the name prefix (`front_lidar`) or another parameter. The URDF
      frame is `front_lidar_link`, but the sensor prefix is `front_lidar`.
    - Whether two instances of the plugin (front and back) can run in one
      `ros2_control_node`.

!!! note "Update 2026-10-13: open questions answered"
    A manual bringup with `mujoco_ros2_control_plugins` 0.1.2, after the
    [move to RoboStack build set `_25`](../04-robostack-build-25.md), answered both open
    questions of the box above:

    - One plugin instance serves both lidars. Each lidar is a sub-key named after its
      sensor prefix (`front_lidar`, `back_lidar`). Its `frame_name` is the URDF frame,
      such as `front_lidar_link`, and the other per-lidar parameters sit next to it.
    - The instance must be named exactly `rangefinder_lidar_plugin`. The plugin reads its
      publish rate from the fixed key
      `mujoco_plugins.rangefinder_lidar_plugin.lidar_publish_rate`.

    With the Walkie values from the parameter table, `/front_lidar` and `/back_lidar` each
    published 241 ranges. Without a parameter file the lidars publish nothing. The manual
    bringup did not run `dual_laser_merger`.
    [MuJoCo Simulation](../../guides/simulation-mujoco.md#the-mujoco_plugins-parameter-file)
    has the working file.

??? info "Measured lidar rate"
    In the manual bringup, the lidars published at about 28 Hz instead of the configured
    40 Hz. The cause has not been investigated.

    **Update 2026-10-13:** runs of `mujoco_omnibot.launch.py` measured about 27.8 Hz. The
    likely cause, not verified in the plugin source: the plugin publishes from the
    controller manager loop, which runs at `update_rate` 60 (`mujoco_controllers.yaml`).
    A 40 Hz target is then met only every second cycle, about 30 Hz, and the loop runs at
    about 55 Hz in practice.

## IMU

The IMU is the MJCF site `top_imu` on the body `top_imu_link`, with three sensors.
ros2_control reads them through `<sensor name="top_imu">`, and `imu_sensor_broadcaster`
publishes them.

The accelerometer reports specific force, like a real IMU. At rest it reads about
+9.81 m/s² on Z, not zero. The MuJoCo IMU adds no noise.

!!! warning "The IMU topic differs from Gazebo"
    `imu_sensor_broadcaster` publishes on `~/imu`, which resolves to
    `/imu_sensor_broadcaster/imu`. Gazebo publishes on `/imu`. The launch file must
    remap the topic so that code written for Gazebo finds it.

!!! note "Update 2026-10-13: remapped in the launch file"
    `mujoco_omnibot.launch.py` remaps `/imu_sensor_broadcaster/imu` to `/imu` on the
    `ros2_control_node`. `/imu` carries the frame `top_imu_link`. See
    [2.7 Launch Files and Nav2](07-launch.md).

??? info "Full IMU wiring: mount, interfaces, broadcaster and Gazebo equivalent"
    | Item | Value |
    | --- | --- |
    | MJCF site | `top_imu`, on the body `top_imu_link` |
    | Mount | `top_imu_link` is fixed to `base_link` at (0, 0, 0.17); 0.195 m above the floor at `home` |
    | MJCF sensors | `top_imu_quat` (`framequat`), `top_imu_gyro` (`gyro`), `top_imu_accel` (`accelerometer`) |
    | ros2_control | `<sensor name="top_imu">` with `mujoco_type` = `imu` and `mujoco_sensor_name` = `top_imu` |
    | State interfaces | `orientation.x/y/z/w`, `angular_velocity.x/y/z`, `linear_acceleration.x/y/z` |
    | Broadcaster | `imu_sensor_broadcaster`, with `sensor_name: top_imu` and `frame_id: top_imu_link` |
    | Gazebo equivalent | `imu_sensor` (type `imu`) on `top_imu_link` at 50 Hz, bridged to the ROS topic `imu` |

    The `mujoco_sensor_name` value is a prefix for the three MJCF sensors
    `top_imu_quat`, `top_imu_gyro` and `top_imu_accel`. The test
    `test_ros2_control_contract` checks that all three exist.

## Cameras

`head_camera` (ZED 2i) renders 640 x 360 with a 110 degree horizontal field of view.
`bottom_camera` (RealSense D415) renders 1280 x 720 with a 69.4 degree horizontal field
of view.

!!! warning
    The bottom camera may point the wrong way. See [Known issues](#known-issues).

??? info "Full camera table: drivers, Gazebo sensors, fovy and pose at home"
    | | `head_camera` | `bottom_camera` |
    | --- | --- | --- |
    | Real camera | ZED 2i (`zed_wrapper`, `camera_model` `zed2i`) | RealSense D415 (`realsense2_camera`, `camera_name` `bottom`) |
    | MJCF body | `zed_head_left_camera_frame_optical` | `bottom_color_optical_frame` |
    | Gazebo sensor used | `head_rgbd_sensor` | `bottom_depth_sensor` |
    | Resolution | 640 x 360 | 1280 x 720 |
    | Horizontal field of view | 1.9199 rad (110 degrees) | 1.211258501 rad (69.4 degrees) |
    | MJCF `fovy` | 77.5544 degrees | 42.5612 degrees |
    | Position at `home` | 1.142 m above the floor | 0.212 m above the floor |
    | View direction at `home` | 9.0 degrees below horizontal | 50.0 degrees **above** horizontal |

### How the cameras are placed

- **On the optical frame, flipped.** Each camera sits on the body of its URDF optical
  frame, with `quat="0 1 0 0"` (180 degrees about X). A MuJoCo camera looks along -Z
  with +Y up, a ROS optical frame along +Z with +Y down. The rotation makes the camera
  image and the URDF frame agree.
- **Vertical field of view.** MuJoCo takes `fovy`, while Gazebo gives the horizontal
  one. The generator converts with `fovy = 2 * atan(tan(hfov / 2) * height / width)`.
- **Head camera follows the head.** Its frame hangs from `head_servo_joint` on the
  lift, so it moves with the lift and the servo.
- **Bottom camera uses the depth resolution.** It takes the resolution of the Gazebo
  depth sensor, not of the color sensor.

??? info "Head camera tilt and the bottom camera color resolution"
    The 9 degree downward tilt of the head camera at `home` comes from the 0.157 rad
    roll of the servo mount in `description/robots/gz_walkie.urdf.xacro`.

    The bottom camera uses the resolution of the Gazebo depth sensor (1280 x 720). The
    Gazebo color sensor of the same D415 is 1920 x 1080 with the same horizontal field
    of view.

### Settings that affect rendering

- The offscreen buffer in `walkie.xml` must be at least as large as the largest camera.
- The near clipping plane in `scene.xml` must stay above about 3 cm. Below that, the
  head camera renders the inside of the ZED housing it sits in.

??? info "Offscreen buffer and near clipping plane values"
    | Setting | Value | Why |
    | --- | --- | --- |
    | `<visual><global offwidth offheight>` in `walkie.xml` | 1280 x 720 | MuJoCo renders cameras into an offscreen buffer of this size. |
    | `<statistic extent>` and `<visual><map znear>` in `scene.xml` | 2.2 and 0.025 | The near clipping plane is `znear * extent` = 5.5 cm. |

### What camera topics will need

!!! note "Update 2026-10-13: head camera on the ZED topic names"
    `bringup/config/mujoco/mujoco_plugins.yaml` configures one `CameraPlugin` instance
    with `camera_publish_rate` 5.0, and `mujoco_omnibot.launch.py` loads it. The head
    camera publishes its images and camera info on the topic names of the real ZED, so
    nodes that read them need no remaps:

    | Camera | Color image | Depth image | Camera info | Frame |
    | --- | --- | --- | --- | --- |
    | `head_camera` | `/zed_head/zed_node/rgb/color/rect/image` | `/zed_head/zed_node/depth/depth_registered` | `/zed_head/zed_node/depth/camera_info` | `zed_head_left_camera_frame_optical` |
    | `bottom_camera` | `/bottom_camera/image_raw` | `/bottom_camera/depth/image_raw` | `/bottom_camera/camera_info` | `bottom_color_optical_frame` |

    One MuJoCo camera renders both images, so the depth image is registered to the color
    image. Before this update, the parameter file in the guide published the head camera
    on `/head_camera/...` topics. No ZED point cloud is published, so nodes that read
    `/zed_head/zed_node/point_cloud/cloud_registered` get no cloud.
    [MuJoCo Simulation: The mujoco_plugins parameter file](../../guides/simulation-mujoco.md#the-mujoco_plugins-parameter-file)
    has the current file.

When this entry was written, no launch file published the MuJoCo cameras, and the
`CameraPlugin` of `mujoco_ros2_control_plugins` 0.1.2 was not configured in the
repository.

??? info "CameraPlugin parameters in the upstream demo"
    The upstream demo config sets `camera_publish_rate` and, per camera, `policy`,
    `frame_name`, `info_topic`, `image_topic` and `depth_topic`.

!!! note "Update 2026-10-13: cameras published in a manual bringup"
    A manual bringup on build set `_25` ([the move to build set `_25`](../04-robostack-build-25.md))
    configured one `CameraPlugin` instance with `head_camera` and `bottom_camera`. Their
    `frame_name` values are the optical frames from the camera table. Both cameras
    published color images at their model resolution and 32-bit float depth images.
    Without a `CameraPlugin` entry, the cameras register themselves (deprecated) under
    `/mujoco_camera_plugin/...`, with frame names that are not in the TF tree.
    [MuJoCo Simulation](../../guides/simulation-mujoco.md#the-mujoco_plugins-parameter-file)
    has the working file.

## Sensor checks in test_mjcf.py

Six tests cover the sensors: `test_imu_at_rest`, `test_lidar_beam_layout`,
`test_lidar_sees_nothing_in_empty_scene`, `test_lidar_ranges_to_wall`,
`test_cameras_at_optical_frames` and the IMU part of `test_ros2_control_contract`.
[2.6 Tests](06-tests.md) describes the rest of the test suite.

??? info "Exact checks and tolerances of each sensor test"
    | Test | What it checks | Tolerance |
    | --- | --- | --- |
    | `test_imu_at_rest` | After 0.5 s at rest: the `top_imu` site is on `top_imu_link`; acceleration is (0, 0, 9.81); angular velocity is zero; `top_imu_quat` is a `framequat` sensor and matches the site orientation | 0.05 m/s², 0.01 rad/s, 1e-6 on the quaternion dot product |
    | `test_lidar_beam_layout[front_lidar]`, `[back_lidar]` | Beam count; first and last beam at `min_angle` and `max_angle`; spacing about 1 degree; the `walkie.xml` header gives back the same count through `int((max - min) / increment) + 1`; every beam starts at the `<lidar>_link` site and points in the right direction; `interval` is 1 / `update_rate`; history size is 1 | 1e-12 rad on the end angles, 1 % on the spacing, 1e-9 m on origins, 1e-5 on directions |
    | `test_lidar_sees_nothing_in_empty_scene[front_lidar]`, `[back_lidar]` | In the empty `scene.xml`, after 0.5 s, no beam reads between 0 and `range_max`: no hits on the robot and none on the floor | none (exact) |
    | `test_lidar_ranges_to_wall` | A FURN box wall (half size 0.05 x 3.0 x 0.5 m) whose face is 1.0 m in front of the robot. Each beam reads the analytic ray-box distance, or a miss. The front lidar has more than 100 hits; the back lidar has some hits with its outer beams, but fewer than the front | 0.02 m |
    | `test_cameras_at_optical_frames` | Each camera is on its optical frame body, at the same position, with -Z of the camera along +Z of the frame and +Y flipped; the horizontal field of view computed back from `fovy` equals the Gazebo value | 1e-9 on pose, 1e-4 rad on the field of view |
    | `test_ros2_control_contract` (IMU part) | For each ros2_control `<sensor>` with `mujoco_type` `imu`, the sensors `<prefix>_quat`, `<prefix>_gyro` and `<prefix>_accel` exist | none (exact) |

## Commits

| Repository | Commit | Change |
| --- | --- | --- |
| `description` submodule | `301c439` | MJCF generator, including lidars, cameras and the IMU |
| `description` submodule | `851618c` | ros2_control `mujoco` branch with the `top_imu` sensor, and `imu_sensor_broadcaster` in `mujoco_controllers.yaml` |
| `description` submodule | `2b15ba1` | `test_mjcf.py`, including the sensor tests |

## Still open

| Item | Status |
| --- | --- |
| Lidar publishing | Done 2026-10-13: `RangefinderLidarPlugin` for both lidars and `dual_laser_merger` to `/scan` in `mujoco_omnibot.launch.py`. The plugin stays deprecated upstream; `Mujoco3dLidarPlugin` needs the `mujoco_3d_lidar` MuJoCo extension, which is not in the pixi environment. Update 2026-10-14: since [entry 5](../05-nav2-repair.md#scan-in-the-empty-mujoco-world), `/scan` is also published in the empty `scene.xml` (`publish_on_empty_cloud`). |
| Lidar rate | About 27.8 Hz instead of the configured 40 Hz. The likely cause, not verified, is in "Measured lidar rate" under [Lidars](#lidars). |
| Camera publishing | Done 2026-10-13: `CameraPlugin` for `head_camera` and `bottom_camera` in `bringup/config/mujoco/mujoco_plugins.yaml` |
| IMU topic | Done 2026-10-13: remapped to `/imu` in `mujoco_omnibot.launch.py` |

!!! note "Update 2026-10-13"
    Lidar and camera publishing work with a plugin parameter file, see the updates above.
    The file, the choice of lidar plugin, `dual_laser_merger` and the IMU remap still have
    to go into a launch file.

!!! note "Update 2026-10-13: launch files added"
    Lidar and camera publishing first worked with a plugin parameter file in a manual
    bringup, see the updates above. The parameter file, the lidar plugin,
    `dual_laser_merger` and the IMU remap are now in `robot_bringup`, and the table above
    shows the result. See [2.7 Launch Files and Nav2](07-launch.md).

The requirements for the launch file are listed in
[2.5 ros2_control](05-ros2-control.md#the-launch-file-still-to-write), and the launch files
themselves are described in [2.7 Launch Files and Nav2](07-launch.md). The collision
classes that decide what the arms touch (but not what the lidars see) are in
[2.3 Collisions](03-collisions.md).

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| The D415 mount pitch tilts the bottom camera 50 degrees up | Every mode, including the real robot, has this pose in TF. The real mount has not been checked. | If the real camera looks down at the floor, flip the sign of the pitch in `description/robots/gz_walkie.urdf.xacro`, then run `pixi run mjcf`. |

??? info "Cause: the pitch of the D415 mount"
    The D415 mount in `description/robots/gz_walkie.urdf.xacro` has
    `rpy="0 -0.872664626  -${M_PI/2}"`. The pitch of -0.8727 rad (-50 degrees) tilts
    the camera up. At `home`, the bottom camera looks along (0.643, 0, 0.766) in the
    robot frame, 50 degrees above horizontal. The MJCF camera matches the URDF frame
    (`test_cameras_at_optical_frames`), so every mode, including the real robot, has
    this pose in TF.
