# Whole-Body MoveIt on the Real Robot

Run whole-body MoveIt on the robot and drive it from RViz, with no agent,
perception, or Nav2. Go through three stages, and start the next one only when
the last one behaves:

1. [Rehearse in MuJoCo](#rehearse-in-mujoco) on the robot computer. Nothing on
   the robot moves.
2. [Real arms, base fixed](#real-arms-base-fixed). The arms and the lift move.
   MoveIt does not command the base, but the wheels stay powered.
3. [Real whole body](#real-whole-body). The base, the lift, and one arm move
   together. Hold this stage back for now: the lift does not follow the plan's
   timing yet.

!!! warning "Whole-body goals have not run on the real robot"
    Stage 3 is tested only with MuJoCo in place of the hardware (see
    [What has not been verified](#not-verified)). No node implements an
    emergency stop: keep a hand on the robot's power switch in stages 2 and 3.
    Read [Stop safely](#stop-safely) and
    [Do not run at the same time](#do-not-run-together) before stage 2.

## Prepare every terminal { #prepare-terminals }

The workspace is cloned and built on the robot computer in
`~/Project/robocup_experiments/walkie-ros-fullbody`, on the branch
`feat/moveit-full-body`, with the lift's Python environment. It is separate from
the team workspace `~/Project/robocup_ros_ws`.

Start a Zenoh router of your own in the first terminal:

```bash
cd ~/Project/robocup_experiments/walkie-ros-fullbody
export PATH=$HOME/.pixi/bin:$HOME/.local/bin:$PATH ROS_DOMAIN_ID=77
unset ZENOH_ROUTER_CONFIG_URI ZENOH_SESSION_CONFIG_URI
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/localhost:7499"];connect/endpoints=[]' pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
```

Run this first in every other terminal:

```bash
cd ~/Project/robocup_experiments/walkie-ros-fullbody
export PATH=$HOME/.pixi/bin:$HOME/.local/bin:$PATH ROS_DOMAIN_ID=77
unset ZENOH_ROUTER_CONFIG_URI ZENOH_SESSION_CONFIG_URI
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7499"]'
```

- The router listens on `localhost:7499` only and connects to no other router.
  With domain 77, this keeps the run's ROS traffic apart from the team router on
  port 7447, so team nodes such as the joystick, Nav2, or the agent cannot reach
  it. The `unset` drops a team Zenoh config that the shell may have exported,
  for example from `connect_walkie_zenoh_router.sh`.
- This does not cover the CAN buses or network bridges such as rosbridge, see
  [Do not run at the same time](#do-not-run-together).
- An ssh shell has neither `~/.pixi/bin` (pixi) nor `~/.local/bin` (uv, for the
  lift environment) on its `PATH`.

To update after new commits on the branch:

```bash
git pull --recurse-submodules
pixi run build-pkg walkie_description robot_bringup openarm_bimanual_moveit_config openarm_bimanual_commander_cpp
```

??? info "Build the workspace from scratch"
    The clone on the robot was made this way on 2026-11-19. The build of all 74
    packages, Nav2 included, took 16 min.

    ```bash
    cd ~/Project/robocup_experiments
    git clone --recurse-submodules -b feat/moveit-full-body git@github.com:EIC-Robocup-2026/walkie-ros.git walkie-ros-fullbody
    cd walkie-ros-fullbody
    git lfs pull
    git submodule foreach --recursive git lfs pull
    pixi install --locked
    MAKEFLAGS=-j6 pixi run build-setup 4
    cd bringup
    pixi run uv venv --python python3.12 --system-site-packages
    pixi run uv sync
    ```

    `build-setup 4` builds with four packages at a time and `MAKEFLAGS=-j6`
    limits each to six compile jobs, which keeps the build within the robot
    computer's memory.

## Rehearse in MuJoCo { #rehearse-in-mujoco }

```bash
pixi run ros2 launch robot_bringup whole_body_demo.launch.py start_router:=false
```

This starts a MuJoCo scene with a table, Nav2 (idle), MoveIt with
`base_mode:=whole_body`, the commander, the base follower, the hand target, the
MuJoCo viewer, and RViz. `headless:=true` leaves out the viewer. In RViz:

1. Send both arms to `rest_pose`: in **MotionPlanning**, choose **Planning
   Group** `both_arms_lift` and **Goal State** `rest_pose`, then **Plan** and
   **Execute**. The lift goes to 0.744 m.
2. Drag the orange box of **Whole-Body Hand Target**, right-click it, and pick
   **Reach with left arm**. The text above the box then reads
   `whole_body_left: done: ...` or `whole_body_left: failed: ...` with the reason.
   See
   [Drag a hand target in RViz](manipulation.md#drag-a-hand-target-in-rviz).
3. Pick **Cancel goal** while a goal runs.

A whole-body goal prefers base poses that leave the arm near `rest_pose`, see
[Tune whole-body goals](manipulation.md#tune-whole-body-goals). Before a goal near
the table, send the other arm to `standby`, see
[Clear the idle arm](manipulation.md#clear-the-idle-arm).

The scripted demo runs nine goals in about 2 min on the robot computer:

```bash
pixi run ros2 run openarm_bimanual_commander_cpp whole_body_demo
```

Stop the rehearsal with Ctrl-C before stage 2. It runs its own controllers in
domain 77.

## Real arms, base fixed { #real-arms-base-fixed }

### Before you power the arms { #before-power }

- The team stack is not running. This prints nothing:

    ```bash
    pgrep -af "ros2_control_node|lift_homing_node|move_group|openarm_bimanual_commander_cpp|whole_body"
    ```

    It also catches a bringup started from the team's walkie-commander web UI,
    whose saved commands launch `lift_homing.launch.py` and
    `real_omnibot.launch.py`. If it prints anything, agree with the team before
    you stop it. Never stop the web UI itself
    (`walkie-commander/.venv/bin/python main.py`): it runs the team router on
    port 7447.

- Each CAN name sits on the right adapter and is up:

    ```bash
    for n in can0 can1 can2; do echo "$n $(basename $(readlink -f /sys/class/net/$n/device/driver)) $(cat /sys/class/net/$n/dev_id) $(cat /sys/class/net/$n/operstate)"; done
    ```

    It must print exactly:

    ```text
    can0 peak_usb 0x0 up
    can1 peak_usb 0x1 up
    can2 gs_usb 0x0 up
    ```

    `can0` and `can1` are the two channels of the PCAN-USB Pro FD, for the right
    and the left arm (CAN FD). `can2` is the candleLight, for the wheels and the
    lift (1 Mbit/s). Do not start `lift_homing.launch.py` or
    `real_omnibot.launch.py` while the output differs. Both open their bus by
    name, so with swapped names the lift and the wheels talk on an arm bus, and
    an arm on the wheel bus.

    When the drivers and the `dev_id`s match and only the last column says
    `down`, bring that bus up by hand and run the check again. `can2` is down
    after every exit of the lift node, which takes it down on exit. With the
    motor power off, run only the line for each bus that shows `down`:

    ```bash
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on   # right arm, PCAN channel 0
    sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on   # left arm, PCAN channel 1
    sudo ip link set can2 up type can bitrate 1000000                          # wheels and lift, candleLight
    ```

    On a bus that is already up, the line fails with `Device or resource busy`.
    Do not run the device setup from
    [Real Robot: Set up the devices](real-robot.md#device-setup) for a bus that
    is only down: it rewrites the `.link` files and asks for replugs. Keep it for
    an interface that is missing.

- When a driver or a `dev_id` differs, fix the names at the robot, with nothing
  running on the buses. These steps have not been tried yet:

    1. Make sure the arms rest or hang where they cannot hit anything. Switch
       off the motor power for the arms, the wheels, and the lift, and keep the
       computer on.
    2. Unplug both USB-CAN adapters, the PCAN (arms) and the candleLight
       (wheels and lift).
    3. Plug the PCAN back into the same USB port, and wait until
       `ip -br link show type can` lists `can0` and `can1`.
    4. Plug in the candleLight.
    5. Run the check above again. `readlink -f /sys/class/net/can0/device` must
       end in `3-2.4:1.0`, the PCAN's port.
    6. Switch the motor power back on, with the rest of this list in place.

    Replugging only the PCAN does not help. The names follow the order in which
    the adapters appear, and the rule that names the PCAN `can0` cannot take a
    name that the candleLight already holds. On 2026-11-19 the candleLight held
    `can0` and the PCAN channels were `can1` and `can2`, from a boot where the
    candleLight came up first.
- The space around the arms, above the lift, and below it is clear. Homing first
  drives the lift down for 3 s, then up to its upper hard stop. When the arm
  hardware activates, every arm joint is commanded to 0 rad at once, then the
  arms go to `home` over 5 s.
- No hand is near the head or its ZED. `real_omnibot.launch.py` drives the head's
  Dynamixel (`/dev/ttyDynamixel`) and tilts the head to 0.25 rad shortly after
  it starts. Ctrl-C on it turns off the head's torque, so the head can drop.
- The grippers are empty, and no hand is near them or the arm joints. The
  grippers close when the arm hardware activates.
- No joystick or keyboard teleop runs in domain 77, in this stage or the next.
  MoveIt does not command the base here, but `twist_mux` and the omni controller
  run, so any velocity command still drives the wheels.
- Someone keeps a hand on the power switch.
- `right_joint2_fixed` has the same value for `real_omnibot.launch.py` and
  `moveit_only.launch.py`. Both default to `false`. If the right arm's joint 2 is
  locked, pass `right_joint2_fixed:=true` to both, and in stage 3 send
  whole-body goals to the left arm only: `commander.launch.py` has no such argument, and
  `whole_body_right` with a locked joint 2 is untested.

### Start { #start-fixed }

Each in its own terminal, in this order:

```bash
sudo -v && pixi run ros2 launch robot_bringup lift_homing.launch.py      # wait for "Homing Successful"
pixi run ros2 launch robot_bringup real_omnibot.launch.py use_zed:=false use_rviz:=false use_rosbridge:=false
pixi run ros2 launch openarm_bimanual_moveit_config moveit_only.launch.py hardware_type:=real_robot base_mode:=fixed
pixi run ros2 launch openarm_bimanual_commander_cpp commander.launch.py base_mode:=fixed
```

- `sudo -v`: the lift node runs `sudo ip link` itself to reset `can2`. Without a
  password cached in that terminal, it stops at a `[sudo] password` prompt
  buried in the launch output, and homing starts whenever the password goes in.
  `sudo -v` asks for it first.
- `use_zed:=false`: this workspace has no ZED wrapper, and nothing here needs
  the camera.
- `use_rosbridge:=false`: rosbridge listens on port 9090 on every network
  interface, and any client that connects there can publish into domain 77,
  velocity commands included.
- `use_rviz:=false`: the RViz with **MotionPlanning** comes from
  `moveit_only.launch.py`, see [Open RViz on the robot's desktop](#rviz-on-the-desktop).

Move the arms as in [Move the arms in RViz](manipulation.md#move-the-arms-in-rviz),
with the groups without `_lift`: `left_arm`, `right_arm`, and `both_arms`. Try
`rest_pose` on `left_arm` first, then on `both_arms`. Look at the planned path
before you click **Execute**. To change the height,
[move the lift alone](#move-the-lift-alone) while the arms hold still, then plan
the arms. The hand target does nothing in this stage: `base_mode:=fixed` refuses
whole-body goals.

!!! warning "The lift ignores the plan's timing and cannot be canceled"
    The lift node starts at once and drives to the plan's last height at its own
    speed, so in a group with `_lift` the arm and the lift do not move in step as
    planned. A plan that is free of collisions in MoveIt can then hit the base on
    the robot, for example `home` while the lift is still below 0.50 m. The lift
    node also rejects a cancel: a stop halts the arms and the base, and the lift
    finishes its move.

### Move the lift alone { #move-the-lift-alone }

```bash
pixi run ros2 topic pub --once /lift/cmd std_msgs/msg/Float64MultiArray "{data: [74.0]}"
```

- Send it only while nothing else moves: no goal running in **MotionPlanning**
  or on the hand target, and the arms and the base still. The lift node follows
  the newest command, from MoveIt or from this topic. A MoveIt goal that is still
  running then reports success after 60 s for a height the lift never reached.
- The value is the height in cm. `lift_homing.launch.py` limits it to 8 to 74.35,
  and the lift node rejects a value outside. 74.35 is where homing stopped
  against the upper hard stop. Use 74.0, 3.5 mm below it, to go near the top.
- The lift moves at 3 cm/s at most, so 30 cm takes about 12 s. The command does
  not wait for it.
- `--once` waits until it finds a subscriber on `/lift/cmd`. If the command
  hangs, it has not found the lift node through this router in domain 77. Press
  Ctrl-C before you look for the cause: a command left waiting publishes as soon
  as the lift node shows up.
- The arm controllers hold their joints, so the arms ride along in their current
  posture. Before you send it, look along the whole travel: nothing above or
  below the hands, and the arms clear the base and the head all the way.
- Watch the lift and wait until it stops. `lift/joint_states`, the **Joints**
  tab, and RViz show the height the lift node commands, not a measured one. The
  node also reports a MoveIt goal for the lift as done after 60 s, even when the
  lift has not arrived.

## Real whole body { #real-whole-body }

!!! danger "Do not run this stage on the robot yet"
    Every whole-body goal can move the lift. The lift node drops the plan's
    intermediate points and timing, and drives straight to the plan's last height
    at up to 3 cm/s from the start. A goal that lowers the lift therefore brings
    the arm down sooner than MoveIt checked, and keeping the hand target's moves
    small does not prevent that. This stage waits for a lift driver that follows
    the trajectory's timing, reports the measured height, and accepts a cancel.
    The steps below are kept for that run.

Keep `lift_homing.launch.py` and `real_omnibot.launch.py` running. Stop MoveIt and
the commander, then start these in order:

```bash
pixi run ros2 launch robot_bringup whole_body_real.launch.py
pixi run ros2 launch openarm_bimanual_moveit_config moveit_only.launch.py hardware_type:=real_robot base_mode:=whole_body
pixi run ros2 launch openarm_bimanual_commander_cpp commander.launch.py base_mode:=whole_body
```

`whole_body_real.launch.py` starts what the base needs: an identity `map` to
`odom` transform, the collision monitor with the real Nav2 parameters, the base
follower, and the hand target. `moveit_only.launch.py` waits for `map` to
`base_footprint`, so it comes second.

Before the first goal:

- `pixi run ros2 lifecycle get /collision_monitor` prints `active`.
- `pixi run ros2 topic hz /scan` and `pixi run ros2 topic hz /unilidar/cloud/filtered`
  show messages. Without either one, the collision monitor stops every base
  command.
- At least 1.5 m of free floor around the robot, and no table or shelf within
  reach. Without the ZED, MoveIt sees no obstacles. Only the collision monitor
  protects the base, from the two lidars.
- The lift is near the top: [move the lift alone](#move-the-lift-alone) to 74.0.
- The plans that move the base run slowly:

    ```bash
    pixi run ros2 param set /bimanual_commander whole_body_velocity_scaling 0.3
    pixi run ros2 param set /bimanual_commander whole_body_acceleration_scaling 0.3
    ```

Then, with the hand target (the RViz fixed frame is `map`):

1. Right-click the box, pick **Snap to left hand**, drag the box 5 cm, and pick
   **Reach with left arm**. The commander first tries the left arm and the lift
   alone. When that works, the status reads
   `whole_body_left: done: Success (base fixed)`: the base stayed, the lift may
   have moved. When it does not, the commander plans a base motion, and the
   status ends in `(base travel ... m)`. Be ready for the base to move from this
   first goal on.
2. Drag the box about 0.3 m further forward and pick **Reach with left arm**
   again. The base may drive: the status then ends in `(base travel ... m)`, and
   in `(base fixed)` when the arm and the lift reach it from where the robot
   stands.
3. Pick **Cancel goal** during a motion. The base, the lift, and the arm must
   stop.

??? info "What this stage relies on"
    - **Odometry:** the omni controller computes odometry from its commands
      (`open_loop: true` in `real_controllers.yaml`), and `map` to `odom` is a
      fixed identity. So `map` to `base_footprint` is where the base was sent,
      not where it is, and a slipping wheel goes unnoticed. With a map of the
      room, run `localization_Nav2_real.launch.py`, set the initial pose in RViz,
      and start `whole_body_real.launch.py static_map_odom:=false collision_monitor:=false`.
      Nav2 then runs its own collision monitor, which publishes no state, so the
      follower notices a stop only as a stall, after about 0.5 s.
    - **Velocity path:** the follower publishes on `/cmd_vel_smoothed`, the
      collision monitor passes it to `/cmd_vel_nav`, and `twist_mux` forwards it
      to the omni controller. The follower stops for the reasons in
      [Why the follower stops](manipulation.md#why-the-follower-stops).
    - **Collision monitor parameters:** `nav2_mppi_walkie_params.yaml` uses YAML
      anchors, which `--params-file` rejects. The launch file reads it through
      `RewrittenYaml`, as Nav2's launch files do, and sets `state_topic` to
      `/collision_monitor_state` so that the follower sees a stop at once. The
      monitor has its own lifecycle manager, `lifecycle_manager_whole_body`.

## Open RViz on the robot's desktop { #rviz-on-the-desktop }

The robot computer runs a KDE Plasma desktop on Wayland, and RViz opens on it
through XWayland. Start the commands from a terminal on that desktop.

Over ssh, someone has to be logged in on the desktop. Run
`echo $DISPLAY $XAUTHORITY` in a terminal on the desktop, then export the same
two values in the ssh terminal that starts RViz (`moveit_only.launch.py` or the
rehearsal), before the launch command. For example:

```bash
export DISPLAY=:0 XAUTHORITY=/run/user/1000/xauth_AbCdEf
```

The name of the `xauth_*` file changes at each login.

## Stop safely { #stop-safely }

In an emergency, cut the power. Ctrl-C on MoveIt or the commander does not stop
the lift, and Ctrl-C on `real_omnibot.launch.py` turns off the arm motors and
the head's torque, so the arms and the head drop.

Otherwise, in this order:

1. End the running goal: **Cancel goal** on the hand target, or **Stop** in
   **MotionPlanning**. Watch the lift until it stops.
2. Raise the lift to the top, with the arms held where they are, as in
   [Move the lift alone](#move-the-lift-alone):

    ```bash
    pixi run ros2 topic pub --once /lift/cmd std_msgs/msg/Float64MultiArray "{data: [74.0]}"
    ```

    Wait until you see the lift stop at the top.

3. Let both arms hang: **Planning Group** `both_arms` (without the lift),
   **Goal State** `home`, **Plan** and **Execute**. `home` needs the lift at
   0.50 m or higher, or the fingers hit the base. Wait until the arms are still.
4. Ctrl-C the commander, `moveit_only.launch.py`, and `whole_body_real.launch.py`.
5. Ctrl-C `real_omnibot.launch.py`. The arm motors and the head's torque turn
   off.
6. Ctrl-C `lift_homing.launch.py`. It takes `can2`, which the wheels share, down
   on exit, so it goes after the base. It runs `sudo` for that, so it may ask
   for the password again in its terminal.
7. Ctrl-C the router.
8. Close all of these terminals. They export `ZENOH_CONFIG_OVERRIDE`, which
   points ROS at the private router, and a later run started from one of them,
   such as the agent stack, would inherit it.

## Do not run at the same time { #do-not-run-together }

| Do not run | Why |
| --- | --- |
| The team bringup from `~/Project/robocup_ros_ws`, in any domain | It drives the same CAN buses: `can0` and `can1` for the arms, `can2` for the wheels and the lift. A ROS domain or a router does not separate CAN. |
| A second `lift_homing.launch.py` | It resets `can2` at start and takes it down at exit |
| A second MoveIt or commander in domain 77 | Both drive the same controllers. The hand target refuses to send goals while two commanders run. |
| `moveit_demo.launch.py hardware_type:=real_robot` | It spawns its own `lift_controller`, which clashes with the lift node's action server |
| The MuJoCo rehearsal during stages 2 and 3 | It runs its own controllers and base follower in domain 77 |
| Joystick or keyboard teleop in domain 77, in stages 2 and 3 | The wheels are live in both stages. `twist_mux` puts teleop above the follower, and teleop bypasses the collision monitor. |
| `real_omnibot.launch.py` without `use_rosbridge:=false` | rosbridge listens on port 9090 on every interface. A client that connects there publishes into domain 77, past the private router. |
| Nav2 next to `whole_body_real.launch.py` with its defaults | Both start a `collision_monitor`. Pass `static_map_odom:=false collision_monitor:=false`, see [Real whole body](#real-whole-body). |

## What has not been verified { #not-verified }

As of 2026-11-19:

- No whole-body goal and no `rest_pose` has run on the real robot.
- Stage 3 ran end to end only with MuJoCo in place of the hardware:
  `mujoco_omnibot.launch.py` with `whole_body_real.launch.py`, the real Nav2
  parameters, and the demo client. See
  [Migration 7: A first run on the real robot](../migrations/07-whole-body-moveit.md#a-first-run-on-the-real-robot).
- `real_omnibot.launch.py` has not been started from this workspace, so
  `use_rosbridge:=false` has not run on the robot either.
- The staged stop in [Stop safely](#stop-safely), the lift alone through
  `/lift/cmd` and then `both_arms` to `home`, has not run on the robot.
- The lift node does not follow a plan's timing and rejects a cancel. Stage 2
  avoids this with the groups without `_lift`. Stage 3 cannot, so it waits for a
  new lift driver.
- The replug order that fixes swapped CAN names, in
  [Before you power the arms](#before-power), has not been tried.
- The rehearsal ran on the robot computer without windows. RViz has not been
  opened on its desktop.
- How the arms, the head, and the lift behave when their motors turn off.
