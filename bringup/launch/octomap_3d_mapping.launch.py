#!/usr/bin/env python3
"""
3D OctoMap mapping that overlays the slam_toolbox 2D map.

Run this ALONGSIDE slam_toolbox (it does NOT replace it and publishes no TF):

    ros2 launch robot_bringup slam_toolbox.launch.py        # or your usual SLAM bringup
    ros2 launch robot_bringup octomap_3d_mapping.launch.py

Why this overlays correctly
---------------------------
slam_toolbox already gives a globally-consistent `map` frame, and the full TF
chain map -> odom -> base_footprint -> sensor already exists. octomap_server is
told to build its octree in `frame_id:=map`, so it reuses slam_toolbox's
*corrected* pose for every cloud it integrates. The 3D map therefore lands in
the same frame as the 2D map and overlays it by construction -- including
slam_toolbox's online loop-closure corrections.

What this map is FOR (no-go zone editing)
-----------------------------------------
The 2D slam_toolbox grid is built from a horizontal lidar, so it only sees
obstacles at lidar height -- it misses tabletops, sofas, shelves and other
overhangs/low furniture. This octomap integrates the full 3D structure and
projects the occupied band back down to 2D (/projected_map). That projection
shows the *complete footprint* of the furniture, which is exactly what you want
to trace no-go rectangles around in the walkie map editor.

Save BOTH maps for the editor with the companion script (it resamples the
octomap projection onto the slam map's exact grid so the two line up pixel for
pixel):

    python3 scripts/save_octomap_for_editor.py --map-dir <map_output_dir>

"More detail"
-------------
Defaults here are tuned much finer than a plain Nav2 map: `resolution` is 0.02 m
(vs the 0.05 m slam grid) for a sharp, detailed 3D octree, speckles are
filtered, and the octree is losslessly compressed. `max_range` is held to 6 m so
the fine detail concentrates where the depth cameras are actually accurate
instead of being sprayed by noisy far returns. The 2D editor overlay is
reprojected back to the slam grid (0.05 m) by save_octomap_for_editor.py, so the
finer octree costs you nothing on alignment. Finer resolution is heavier on
CPU/RAM with three sensors fused -- raise resolution:=0.03 (or 0.05) if it can't
keep up.

Multi-sensor fusion (ZED + RealSense D415 + Unitree 4D L2)
----------------------------------------------------------
octomap_server subscribes to a single `cloud_in`. To fuse three sensors we run
one `topic_tools relay` per source, all republishing onto ONE merged topic
(/octomap/cloud_in). ROS allows multiple publishers on a topic; octomap looks up
TF for each incoming cloud's own header.frame_id, so the three clouds (each in a
different optical/lidar frame) are all integrated correctly without any manual
point transformation. Toggle each source with use_zed / use_realsense /
use_unitree.

We feed the SELF-FILTERED clouds (robot body already removed), so the robot
itself is never baked into the 3D map.

Dynamic objects (people)
------------------------
The self-filter removes the ROBOT, not people -- so anyone who walks into a
camera gets integrated as occupied voxels. octomap is probabilistic, so it CAN
clear them again, but only the way it clears anything: a later beam has to pass
THROUGH that space and mark it free. `sensor_model.max` is therefore lowered to
0.85 (from octomap's 0.97 default) so a transient blob decays back to free after
a handful of pass-through beams instead of being baked in permanently. We keep
it at 0.85 (not lower) and leave `sensor_model.miss` at its default so this does
NOT erode the rarely-observed thin structure (chair legs, table edges) the fine
0.02 m resolution is meant to capture.

Hard limit: tuning only clears a voxel the camera LATER sees through. It does
nothing if you save the map while a person is still standing there, or if
someone stood in a spot that never gets re-observed. So for a clean map:
  * map when the area is as clear of people as practical;
  * after someone walks through, keep scanning that area so octomap re-observes
    and clears it before you save;
  * any residual person-blob is easy to erase in the walkie map editor (this map
    feeds the editor anyway), which is usually enough for a one-off no-go map.
If people genuinely cannot be kept out, the proper fix is to mask person points
out of the cloud at the source (e.g. a node that drops the /yolo/masks person
pixels from the ZED registered cloud before it is relayed in here); that is not
wired up yet and would only cover the ZED, not the D415.

Outputs
-------
  /octomap_full                 full 3D octree (save with the script below)
  /octomap_binary               binary octree
  /octomap_point_cloud_centers  occupied voxel centers (drop into RViz/Foxglove
                                 on top of /map for the labelling overlay)
  /projected_map                2D down-projection of the occupancy band
                                 (occupancy_min_z..occupancy_max_z) -- the clean
                                 2D footprint that becomes the editor overlay

Save the finished maps for the editor (slam map + octree + aligned overlay):
    python3 scripts/save_octomap_for_editor.py --map-dir <map_output_dir>

Save just the raw 3D octree by hand (note: this jazzy octomap_saver_node takes
the path as the `octomap_path` PARAMETER, not as a positional argument):
    ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=my_area.bt   # binary
    ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=my_area.ot   # full
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")
    max_range = LaunchConfiguration("max_range")
    sensor_model_hit = LaunchConfiguration("sensor_model_hit")
    sensor_model_miss = LaunchConfiguration("sensor_model_miss")
    sensor_model_min = LaunchConfiguration("sensor_model_min")
    sensor_model_max = LaunchConfiguration("sensor_model_max")
    point_cloud_min_z = LaunchConfiguration("point_cloud_min_z")
    point_cloud_max_z = LaunchConfiguration("point_cloud_max_z")
    occupancy_min_z = LaunchConfiguration("occupancy_min_z")
    occupancy_max_z = LaunchConfiguration("occupancy_max_z")
    filter_ground = LaunchConfiguration("filter_ground")
    ground_filter_distance = LaunchConfiguration("ground_filter_distance")
    use_zed = LaunchConfiguration("use_zed")
    use_realsense = LaunchConfiguration("use_realsense")
    use_unitree = LaunchConfiguration("use_unitree")

    merged_cloud_topic = "/octomap/cloud_in"

    # Source point-cloud topics (the self-filtered clouds from real_omnibot).
    unitree_cloud_topic = "/unilidar/cloud/filtered"
    zed_cloud_topic = "/zed_head/zed_node/point_cloud/cloud_registered/filtered"
    realsense_cloud_topic = "/camera/camera/depth/color/points/filtered"

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock",
    )
    declare_resolution = DeclareLaunchArgument(
        "resolution", default_value="0.02",
        description="OctoMap voxel size [m]. Much finer than the 0.05 slam grid "
                    "for a detailed 3D octree; the 2D editor overlay is reprojected "
                    "back to 0.05 by save_octomap_for_editor.py so alignment is "
                    "unaffected. Raise to 0.03 (or 0.05) if CPU/RAM can't keep up.",
    )
    declare_max_range = DeclareLaunchArgument(
        "max_range", default_value="6.0",
        description="Max range [m] of points integrated. Held to 6 m so the fine "
                    "detail concentrates where the depth cameras are accurate "
                    "instead of being sprayed by noisy far returns. Caps CPU too. "
                    "-1 = unlimited.",
    )
    declare_sensor_model_hit = DeclareLaunchArgument(
        "sensor_model_hit", default_value="0.7",
        description="Probability a beam endpoint marks a voxel occupied. Higher = "
                    "obstacles build up faster / sharper but noise persists longer.",
    )
    declare_sensor_model_miss = DeclareLaunchArgument(
        "sensor_model_miss", default_value="0.4",
        description="Probability a beam passing through marks a voxel free. Lower = "
                    "clears space more aggressively.",
    )
    declare_sensor_model_min = DeclareLaunchArgument(
        "sensor_model_min", default_value="0.12",
        description="Lower clamp on voxel occupancy probability (octomap default).",
    )
    declare_sensor_model_max = DeclareLaunchArgument(
        "sensor_model_max", default_value="0.85",
        description="Upper clamp on voxel occupancy probability. Lowered from the "
                    "0.97 octomap default so a voxel never becomes 'permanently "
                    "confident': a transient obstacle (a person who walks through) "
                    "decays back to free after a handful of later pass-through "
                    "beams, instead of being baked in forever. Kept at 0.85 (not "
                    "lower) so genuine walls/furniture -- hit constantly -- stay "
                    "solid and rarely-seen thin structure isn't eroded. See the "
                    "'Dynamic objects (people)' note in the module docstring.",
    )
    declare_point_cloud_min_z = DeclareLaunchArgument(
        "point_cloud_min_z", default_value="0.15",
        description="Drop points below this height (in the map frame) before "
                    "integration -- the octomap floor cutoff. Clips the floor and "
                    "low clutter (cables, thresholds, baseboards).",
    )
    declare_point_cloud_max_z = DeclareLaunchArgument(
        "point_cloud_max_z", default_value="1.8",
        description="Drop points above this height (map frame) -- caps the max "
                    "height of the octomap. Raise it if your ceilings are taller "
                    "and you want them mapped.",
    )
    declare_occupancy_min_z = DeclareLaunchArgument(
        "occupancy_min_z", default_value="0.15",
        description="Lower bound of the voxel band collapsed into the 2D "
                    "/projected_map overlay (kept at/above point_cloud_min_z, the "
                    "octomap floor cutoff).",
    )
    declare_occupancy_max_z = DeclareLaunchArgument(
        "occupancy_max_z", default_value="1.8",
        description="Upper bound of the voxel band collapsed into /projected_map "
                    "(kept at/below point_cloud_max_z, the octomap height cap).",
    )
    declare_filter_ground = DeclareLaunchArgument(
        "filter_ground", default_value="false",
        description="Segment and drop the ground plane from each cloud before "
                    "integration (RANSAC). The occupancy_min_z band already "
                    "excludes the floor from the overlay; enable this only if "
                    "floor speckle is bleeding into the 3D map.",
    )
    declare_ground_filter_distance = DeclareLaunchArgument(
        "ground_filter_distance", default_value="0.04",
        description="Max distance [m] a point may sit from the fitted ground "
                    "plane to count as ground (only used when filter_ground:=true).",
    )
    declare_use_zed = DeclareLaunchArgument(
        "use_zed", default_value="true",
        description="Feed the ZED 2i registered cloud into the octomap",
    )
    declare_use_realsense = DeclareLaunchArgument(
        "use_realsense", default_value="true",
        description="Feed the RealSense D415 cloud into the octomap",
    )
    declare_use_unitree = DeclareLaunchArgument(
        "use_unitree", default_value="false",
        description="Feed the Unitree 4D L2 lidar cloud into the octomap. Off by "
                    "default -- the octomap is built from the ZED + RealSense "
                    "clouds only.",
    )

    octomap_server_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            # Build the octree in slam_toolbox's frame so it overlays the 2D map.
            "frame_id": "map",
            "base_frame_id": "base_footprint",
            "resolution": resolution,
            # Probabilistic sensor model (sharpness vs noise rejection).
            "sensor_model.max_range": max_range,
            "sensor_model.hit": sensor_model_hit,
            "sensor_model.miss": sensor_model_miss,
            "sensor_model.min": sensor_model_min,
            "sensor_model.max": sensor_model_max,
            # Height gates (map frame) for what gets integrated into the 3D map.
            "point_cloud_min_z": point_cloud_min_z,
            "point_cloud_max_z": point_cloud_max_z,
            # Band of the octree collapsed into the 2D /projected_map overlay.
            "occupancy_min_z": occupancy_min_z,
            "occupancy_max_z": occupancy_max_z,
            # Optional RANSAC ground-plane removal (off by default).
            "filter_ground_plane": filter_ground,
            "ground_filter.distance": ground_filter_distance,
            # Drop isolated single-voxel speckles for a cleaner map.
            "filter_speckles": True,
            # Losslessly prune the octree -> smaller files, same detail.
            "compress_map": True,
            # Latch so a late subscriber (the save script / RViz) immediately
            # gets the current map instead of waiting for the next cloud.
            "latch": True,
        }],
        remappings=[("cloud_in", merged_cloud_topic)],
    )

    # One relay per sensor, all republishing onto the single merged cloud topic.
    relay_unitree = Node(
        package="topic_tools",
        executable="relay",
        name="octomap_relay_unitree",
        arguments=[unitree_cloud_topic, merged_cloud_topic],
        output="screen",
        condition=IfCondition(use_unitree),
    )
    relay_zed = Node(
        package="topic_tools",
        executable="relay",
        name="octomap_relay_zed",
        arguments=[zed_cloud_topic, merged_cloud_topic],
        output="screen",
        condition=IfCondition(use_zed),
    )
    relay_realsense = Node(
        package="topic_tools",
        executable="relay",
        name="octomap_relay_realsense",
        arguments=[realsense_cloud_topic, merged_cloud_topic],
        output="screen",
        condition=IfCondition(use_realsense),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_resolution,
        declare_max_range,
        declare_sensor_model_hit,
        declare_sensor_model_miss,
        declare_sensor_model_min,
        declare_sensor_model_max,
        declare_point_cloud_min_z,
        declare_point_cloud_max_z,
        declare_occupancy_min_z,
        declare_occupancy_max_z,
        declare_filter_ground,
        declare_ground_filter_distance,
        declare_use_zed,
        declare_use_realsense,
        declare_use_unitree,
        octomap_server_node,
        relay_unitree,
        relay_zed,
        relay_realsense,
    ])
