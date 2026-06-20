# cuMotion (cuRobo) GPU planning for the OpenArm bimanual robot

cuMotion (`isaac_ros_cumotion` + `isaac_ros_cumotion_moveit`) wraps NVIDIA's
cuRobo CUDA trajectory optimizer as a **MoveIt 2 planning pipeline**. It is added
as a *second* pipeline alongside OMPL — OMPL stays the default, cuMotion is opt-in
per request. The commander selects it live with one parameter; nothing regresses
when it's off.

> **Where it runs:** cuMotion (CUDA + cuRobo + the cuMotion MoveIt plugin) only
> exists inside the **Isaac ROS Docker container** (FastDDS). The robot normally
> runs on the host under `rmw_zenoh`, which does **not** interoperate with the
> container's FastDDS. So the MoveIt stack + commander you want to drive with
> cuMotion must run **inside the container**. The plain (OMPL) launches stay on
> the host, unchanged.

> **Deploying to another machine** (image transfer, branch checkout, the
> description-submodule gotcha): see [`DEPLOY.md`](DEPLOY.md).

---

## 1. Architecture / key facts

- **Pipeline id is `cumotion`** (derived from `config/cumotion_planning.yaml`'s
  filename by the MoveItConfigsBuilder). Select it with the commander's
  `planning_pipeline` param or in RViz's Context tab. (The standalone validation
  launch uses the id `isaac_ros_cumotion` instead — don't confuse them.)
- **cuMotion plans the whole XRDF cspace.** One running planner = ONE planning
  group (whichever XRDF is loaded). To plan a different group, relaunch with a
  different `cumotion_xrdf:=…`.
- **Patched plugin.** Stock cuMotion 4.4 forwards the full robot state to cuRobo,
  so RViz / `MoveGroupInterface` (the commander) fail with
  `cspace_position [24] must equal [14]`. We patched
  `cumotion_move_group_client.cpp::updateGoal` to prune the start state to the
  group's joints; it's built as a container overlay at
  `/workspaces/isaac_ros-dev/install`. **This must be sourced** or RViz/commander
  planning fails.
- **Container:** `isaac_ros_dev_container`, image `isaac_ros_cumotion:walkie`.

---

## 2. One-time setup (per fresh container)

The persistent container must mount **both** the Isaac workspace (cuMotion overlay
+ staged files) and the walkie workspace (the robot packages):

```bash
# (host) recreate the container with both mounts + X11 for in-container RViz
docker rm -f isaac_ros_dev_container 2>/dev/null
docker run -d --name isaac_ros_dev_container --gpus all --runtime nvidia \
  --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ryu/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v /home/ryu/walkie_ws:/walkie_ws \
  --entrypoint /bin/bash isaac_ros_cumotion:walkie -lc 'sleep infinity'
```

Then **inside the container** build the two overlays (only if missing):

```bash
docker exec -it isaac_ros_dev_container bash      # use: sg docker -c "docker exec -it … bash" if no docker group
source /opt/ros/jazzy/setup.bash

# (a) patched cuMotion MoveIt plugin overlay
cd /workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_cumotion_moveit   # -> install/

# (b) walkie packages, into SEPARATE base dirs so the host's build/install
#     (the mount is the host's live workspace!) is NOT clobbered
cd /walkie_ws
colcon build --packages-up-to openarm_bimanual_commander_cpp \
  --build-base /tmp/wb --install-base /tmp/wi --symlink-install
```

> Rebuild walkie (`/tmp/wi`) only after a fresh container; the overlay at
> `/workspaces/isaac_ros-dev/install` persists in the mounted volume.

---

## 3. Run it — the wrapper (recommended)

`launch_cumotion_stack.sh` ALWAYS kills any existing stack first, sources the
three overlays in order, verifies the patched plugin, then launches one stack.
Run it **inside the container**:

```bash
/workspaces/isaac_ros-dev/openarm_cumotion/launch_cumotion_stack.sh                 # full demo (default)
/workspaces/isaac_ros-dev/openarm_cumotion/launch_cumotion_stack.sh moveit_only_cumotion.launch.py
/workspaces/isaac_ros-dev/openarm_cumotion/launch_cumotion_stack.sh \
    moveit_demo_with_commander_cumotion.launch.py cumotion_xrdf:=.../config/curobo/left_arm.xrdf
```

Look for `-> PATCHED overlay (good)` in the output. For in-container RViz, run
`xhost +local:` on the **host** once first.

### The three cuMotion launches (in `openarm_bimanual_moveit_config/launch/`)

| Launch | What it brings up | Use for |
|--------|-------------------|---------|
| `move_group_cumotion.launch.py` | move_group (ompl+cumotion) + GPU planner | wiring into an existing bringup |
| `moveit_only_cumotion.launch.py` | `mock_components` (default): RSP + mock control + controllers + move_group + RViz + GPU planner (self-contained, robot renders). `real_robot`/`gazebo`/`isaac`: overlay (move_group + RViz + planner only, assumes a bringup) | RViz interactive planning |
| `moveit_demo_with_commander_cumotion.launch.py` | mock `ros2_control` + controllers + move_group + GPU planner + **commander** + RViz | full self-contained test with mock execution |

Common args: `cumotion_xrdf:=<path>` (selects the group; default
`config/curobo/openarm_bimanual.xrdf` = both_arms), `cumotion_urdf:=<flat urdf>`,
`hardware_type:=mock_components|gazebo|isaac|real_robot`.

### Real robot (host hardware) — run under zenoh

The robot's hardware/controllers/commander run on the **host** under `rmw_zenoh`;
cuMotion runs in the container. They share one ROS graph if the container stack
runs under zenoh too (the container is `--network host`, so the host's
`rmw_zenohd` router on localhost is reachable — verified). No bridge needed.

```bash
# HOST: real bringup (ros2_control + controllers + commander + RSP + rmw_zenohd),
#       but DO NOT start the host move_group (the container provides the one move_group).

# CONTAINER: move_group (ompl+cumotion) + GPU planner, under zenoh, joined to the host router
RMW=zenoh /workspaces/isaac_ros-dev/openarm_cumotion/launch_cumotion_stack.sh \
    move_group_cumotion.launch.py \
    cumotion_urdf:=/workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf
```

Then on the host: `ros2 param set /bimanual_commander planning_pipeline cumotion`
(GPU plan in container → executes on the real controllers over zenoh); `""` =
OMPL fallback (same move_group). `RMW=zenoh` needs `rmw_zenoh` in the container
(already in the `isaac_ros_cumotion:walkie` image) and the host `rmw_zenohd` up.
Keep exactly ONE move_group across host+container.

---

## 4. Drive it

### Via the commander (action API — no code change needed)

```bash
# inside the container, sourced: source /opt/ros/jazzy/setup.bash; source /tmp/wi/setup.bash
ros2 param set /bimanual_commander planning_pipeline cumotion      # GPU; "" = OMPL fallback

# joint goal (both_arms = 14 values: left j1..7, right j1..7)
ros2 action send_goal /set_joint_position my_robot_interfaces/action/SetJointPosition \
  "{group_name: both_arms, joint_positions: [0,-1.4,0,0.6,0,0,0, 0,1.4,0,0.6,0,0,0]}"

# Cartesian pose goal (frame base_footprint)
ros2 action send_goal /go_to_pose my_robot_interfaces/action/GoToPose \
  "{group_name: left_arm, x: 0.42, y: 0.60, z: 1.05, roll: 0, pitch: 0, yaw: 0, cartesian_path: false, frame_id: base_footprint}"
```

On the mock stack the plan executes on the controllers (`Execute request success!`).

### Via RViz interactive markers

1. `xhost +local:` on host; launch a stack that includes RViz.
2. MotionPlanning → **Context** tab → **Planning Pipeline = `cumotion`**.
3. **Planning Group** = the loaded XRDF's group (default `both_arms`).
4. **Uncheck Allow Replanning + Allow Looking** (cuMotion's action server is
   single-goal; overlapping goals get "Planner is busy").
5. Drag the marker → **Plan** (trajectory animates) → **Plan & Execute**.

### Standalone scripts (no commander)

- `goto.py <group> --joints …` or `--pose x y z [qx qy qz qw]` — sends a
  plan-only MoveGroup goal that animates in RViz.

---

## 5. Stop / restart

```bash
# inside container: kill the stack (bracket trick avoids killing the kill shell)
for p in "[r]os2 launch" "[m]ove_group" "[c]umotion" "[r]os2_control_node" \
         "[b]imanual_commander" "[r]viz2" "[s]pawner" "[r]obot_state_publisher"; do
  pkill -9 -f "$p"; done

# host: full clean slate (clears zombies + stale DDS) — overlay/build survive
docker restart isaac_ros_dev_container
```

Verify exactly one live planner: `pgrep -x move_group` (one PID; zombies show as
`<defunct>` and are harmless — PID 1 `sleep infinity` doesn't reap them).
To relaunch, just run the wrapper (it kills first).

---

## 6. XRDF generation pipeline

The XRDF describes the robot for cuRobo (cspace, collision spheres, self-collision
ignore, tool frames). Pipeline, run from this directory with the sphere-fit venv:

```bash
# 0. flat URDF (head_servo velocity must be > 0); meshes file://. Staged in container at
#    /workspaces/isaac_ros-dev/openarm_cumotion/openarm_bimanual.urdf
xacro ../openarm_bimanual.urdf.xacro > /tmp/openarm_bimanual.urdf

# 1. skeleton: cspace + joint limits + self_collision.ignore from the SRDF
#    (drops DISTAL cross-arm pairs so the arms can't plan through each other)
python3 gen_xrdf_skeleton.py

# 2. arm collision spheres fit from the URDF meshes (surface k-means)
.spherefit_venv/bin/python gen_xrdf_spheres.py        # numpy<2 + trimesh + scipy + pyyaml venv

# 3. (optional) per-group XRDFs derived from the bimanual one
python3 make_single_arm_xrdf.py both_arms_lift        # right_arm | left_arm | *_lift | both_arms_lift
```

### XRDF files (one per planning group)

| File | Group | cspace |
|------|-------|--------|
| `openarm_bimanual.xrdf` | both_arms | 14 |
| `left_arm.xrdf` / `right_arm.xrdf` | single arm | 7 |
| `left_arm_lift.xrdf` / `right_arm_lift.xrdf` | arm + lift | 8 |
| `both_arms_lift.xrdf` | both arms + lift | 15 |

### Self-collision tooling

- `strip_cross_arm_ignore.py <xrdf…>` — removes DISTAL cross-arm pairs from the
  ignore map (keeps them collision-checked so arms can't cross). Proximal
  cross-arm pairs stay ignored (over-approximate shoulder spheres false-overlap
  at rest). Same rule now lives in `gen_xrdf_skeleton.py`.
- `check_self_collision.py --urdf … --xrdf …` — host FK + sphere overlap check at
  q=0; lists colliding link pairs.
- `curate_cross_arm_ignore.py` — FK-at-safe-poses cross-arm curation (alternative
  to the distal heuristic; under-detects vs cuMotion's sphere padding).
- `analyze_base_arm_collisions.py` — samples configs to measure base↔arm contact.
- `gen_body_spheres.py` — adds base_link/lift_link spheres (see Limitations).

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `cspace_position [24] must equal [14]` | A move_group without the patched plugin overlay. Ensure `/workspaces/isaac_ros-dev/install` is sourced; the wrapper prints `PATCHED overlay (good)`. Often caused by a **duplicate move_group** — keep exactly one (`pgrep -x move_group`). |
| No trajectory shown in RViz | cuMotion pipeline needs a response adapter. `cumotion_planning.yaml` includes `DisplayMotionPath` — confirm it's there. |
| "Planner is busy, rejecting goal" | cuMotion's single-goal server got overlapping requests. In RViz uncheck Allow Replanning/Looking; from CLI don't fire goals back-to-back. |
| `Velocity of last trajectory point … is not zero` (execution ABORTED) | cuMotion's min-jerk ends with tiny non-zero velocity. Fixed by `allow_nonzero_velocity_at_trajectory_end: true` in `config/ros2_controllers.yaml`. |
| RViz: `Could not load Qt platform plugin "xcb"` | Run `xhost +local:` on the host before launching. |
| `INVALID_INITIAL_CSPACE_POSITION` / "self collision detected" | The start (or goal) config is genuinely in self-collision (e.g. arms crossing, or arms folded into the base from `q=0`). Start from an arms-raised/clear pose. |
| Flaky planning / >1 `/joint_states` publisher | Stale processes. `docker restart isaac_ros_dev_container`, then relaunch via the wrapper. |

---

## 8. Base/torso collision — covered by mesh ValidateSolution (reject)

cuMotion has **no base/torso collision spheres** (adding them over-constrains —
see below), so it does not *route around* the body. Instead, arm-vs-base/torso is
**caught and rejected** by MoveIt's `ValidateSolution` response adapter, which
mesh-checks the whole planned trajectory against the scene + SRDF ACM
(base_link↔arm link0-3 disabled, link4-7 checked). This required patching the
cuMotion plugin's response (`cumotion_interface.cpp`) to seed `trajectory_start`
from the scene's **full current state** — otherwise non-group joints default to 0
(lift down / idle arm folded) and ValidateSolution false-rejects everything.

- **Behavior:** rejects unsafe plans (commander gets a failure → retry / OMPL
  fallback); does NOT plan around the body. Verified: normal goals pass,
  self-collision and arm-into-torso goals are rejected
  (`contact between 'base_link' and 'openarm_left_link5'`).
- Start/goal were already covered by `CheckStartStateCollision`; ValidateSolution
  adds the **mid-path** coverage.
- For true **avoidance** (route around), an **nvBlox ESDF world**
  (`read_esdf_world`) is the follow-up.

### Why not body spheres (`gen_body_spheres.py`)

Adding base_link/lift_link spheres over-constrains the planner: cuMotion pads the
over-approximate spheres, and the arms operate within that padded distance of the
lift/torso, so normal poses get falsely rejected (two attempts confirm this; the
activation distance is not a tunable param, and `buffer_distance` only widens the
margin). The tool is kept but not applied.
