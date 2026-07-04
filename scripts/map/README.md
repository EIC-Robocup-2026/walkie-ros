# Mapping rosbag — record during a match, rebuild the maps afterwards

Record a rosbag while driving during mapping, then after the match either **re-run**
slam_toolbox / octomap_server offline to rebuild the maps, or **pull the live map
exactly as it was at any moment** — all without re-driving the robot.

Files in this folder:

| File | Purpose |
|------|---------|
| `record_mapping_bag.sh` | Records the bag (run this while driving). |
| `odom_to_tf.py`         | Helper for the SLAM re-run (see [Playback for re-map](#2-playback-for-re-map)). |

The bag captures two layers:

- **Inputs** — `/scan`, wheel `/odom`, `/tf`, `/tf_static`, and the merged self-filtered
  cloud `/octomap/cloud_in` → for a full, re-tuneable rebuild of either map.
- **Live map outputs** — `/map`, `/map_metadata`, `/octomap_binary`, `/projected_map`
  → to pull the map as the robot had it at time `t`, with no reprocessing.

> **The `map→odom` gotcha.** slam_toolbox runs live during the match, so the recorded
> `/tf` already contains a `map→odom` transform. That is *ideal* for rebuilding the octomap
> (just replay and reuse the corrected poses), but if you re-run slam_toolbox it publishes
> its *own* `map→odom`, which fights the recorded one. That is the only reason the SLAM
> re-map recipe replays **without** `/tf` and regenerates `odom→base_footprint` with
> `odom_to_tf.py`.

---

## 1. Record during mapping

Start the robot and both mapping pipelines (these publish the topics we record):

```bash
ros2 launch robot_bringup real_omnibot.launch.py            # robot + sensors + zenoh router
ros2 launch robot_navigation mapping_Nav2_real.launch.py    # slam_toolbox (2D)
ros2 launch robot_bringup octomap_3d_mapping.launch.py      # octomap_server (3D)
```

Then record while you drive:

```bash
./scripts/map/record_mapping_bag.sh
# or choose the output dir:
./scripts/map/record_mapping_bag.sh ~/matches/round1
# also grab the heavy /octomap_full + voxel-centers cloud:
./scripts/map/record_mapping_bag.sh --heavy
```

Drive the arena, then press **Ctrl-C** to stop and close the bag. Default output is
`~/walkie_mapping_bags/mapping_<timestamp>` (MCAP + zstd).

The recorder runs on `rmw_zenoh_cpp` and `ROS_DOMAIN_ID=23` by default — **these must
match the running bringup** or it will see no topics (it warns you at startup if so).
Override with `RMW_IMPLEMENTATION=... ROS_DOMAIN_ID=... ./record_mapping_bag.sh`.

Check what you captured:

```bash
ros2 bag info ~/walkie_mapping_bags/mapping_<timestamp>
```

Every listed topic should have a non-zero message count. `/map` and `/octomap_binary`
only exist while the mapping launches are running — if they are empty, a launch wasn't up.

**Recorded topics**

| Topic | Why |
|-------|-----|
| `/tf`, `/tf_static` | transforms for both pipelines |
| `/scan` | merged lidar — the slam input |
| `/front_lidar`, `/back_lidar` | raw per-lidar scans (tiny; re-tune the merge later) |
| `/omni_wheel_drive_controller/odom` | wheel odom — enables the clean SLAM re-run |
| `/octomap/cloud_in` | merged self-filtered cloud — the octomap input (heavy) |
| `/map`, `/map_metadata` | live slam 2D map output |
| `/octomap_binary`, `/projected_map` | live 3D octree + its 2D footprint output |

---

## 2. Playback for re-map

Rebuild a map offline from the recorded **inputs** (re-tune params, recover loop closures,
etc.). Use `--clock` and `use_sim_time:=true` so the offline nodes run on bag time.

### Octomap 3D (faithful — reuses the live-corrected poses)

The recorded `/tf` already has the corrected `map→odom`, so just replay and let
octomap_server integrate. Disable the sensor relays — the bag already supplies the merged
`/octomap/cloud_in`:

```bash
ros2 bag play <bag> --clock

ros2 launch robot_bringup octomap_3d_mapping.launch.py \
     use_sim_time:=true use_zed:=false use_realsense:=false use_unitree:=false

# save the rebuilt maps for the editor (slam map + octree + aligned overlay):
python3 scripts/save_octomap_for_editor.py --map-dir <out_dir>
```

### SLAM 2D (re-run slam_toolbox, no `map→odom` conflict)

Replay **without** `/tf`, feed slam the odometry via `odom_to_tf.py`, and let the fresh
slam_toolbox own `map→odom`:

```bash
ros2 bag play <bag> --clock \
     --topics /scan /tf_static /omni_wheel_drive_controller/odom

python3 scripts/map/odom_to_tf.py --ros-args -p use_sim_time:=true

ros2 launch robot_navigation mapping_Nav2_real.launch.py use_sim_time:=true

# then save the 2D map:
ros2 run nav2_map_server map_saver_cli -t /map -f <out_dir>/map
```

If you see TF warnings about multiple `map→odom` publishers, you accidentally replayed
`/tf` — re-check the `--topics` filter above.

---

## 3. Get a map from playback (snapshot at time `t`)

No re-run needed: the live map outputs are in the bag, so fast-forward and save whatever
the robot had built by that moment.

```bash
# jump to t seconds into the run, then let it keep playing a few seconds
ros2 bag play <bag> --start-offset <t_seconds>

# 2D slam map at that time:
ros2 run nav2_map_server map_saver_cli -t /map -f snapshot_2d

# 2D octomap footprint (the no-go overlay) at that time:
ros2 run nav2_map_server map_saver_cli -t /projected_map -f snapshot_projected

# 3D octree at that time (octomap_saver grabs the latest /octomap_binary it hears):
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=snapshot.bt
```

`map_saver_cli` waits for the next message on the topic, so keep the bag playing until it
writes the file. The saved map is exactly what the robot had at time `t` — no reprocessing,
no `map→odom` concerns.
