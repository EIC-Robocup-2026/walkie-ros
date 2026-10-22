# 2.9 Named Worlds

**Goal:** run Walkie in MuJoCo worlds that are exported from Blender, selected by name,
with their own occupancy map, a start pose and AMCL, without editing the robot model or the
launch files for each new world.

This entry records how a world is described, where its files live, how the launch merges it
with the robot, and why it is built this way. For day-to-day use, read
[MuJoCo Simulation: Add a named world](../../guides/simulation-mujoco.md#add-a-named-world)
and [Chula Pat14 Room Simulation](../../guides/simulation-chula-pat14.md).

## In short

- **Added:** the package `walkie_mujoco_worlds` in `simulation/mujoco/`, with the first world,
  `chula_pat14_6`, in two variants (`lowpoly` and `baked`); the Python modules
  `robot_bringup.world_loader` and `robot_bringup.mujoco_launch`; the launch arguments
  `world`, `variant`, `spawn`, `spawn_pose`, `worlds_dir` and `world_output_dir` in both
  MuJoCo launch files; AMCL in `nav_mujoco_omnibot.launch.py` when a world is set; the pixi
  tasks `world-validate` and `world-overlay`; three pytest files in `robot_bringup`, which CI
  now runs.
- **Key decisions:** one `world.toml` manifest per world, next to its assets; a separate,
  data-only package instead of `robot_simulation` or a repository outside the workspace;
  plain Git instead of Git LFS; the robot owns the model-wide MuJoCo settings; the loader
  sets the collision class of every world geom.
- **Found along the way:** merging the world's `<compiler>` into the robot's changed
  Walkie's inertia. A fresh export from the add-on took the robot from 36.66 kg to 161 kg.
  The robot's settings now win
  ([The robot owns the model-wide settings](#robot-owns-model-wide-settings)).
- **Result:** `world:=chula_pat14_6` loads the room, places the robot at the `living-room`
  spawn and, with Nav2, starts AMCL at the same pose. Offline, both variants merge and
  compile, and the tests check that the merged robot keeps its physics. Without `world`,
  both launch files behave as before.
- **Live run (2026-10-22, headless):** Nav2 reaches goals in the room, from the spawn to the
  kitchen and back ([Verification](#verification)). AMCL stays 0.1 to 0.2 m off the true pose because the map has no
  furniture, and the collision monitor stops the robot at the room divider on the way back
  ([Known issues](#known-issues)). The run found three small faults, all fixed
  ([Found in the live run](#found-in-the-live-run)).

## Use it

```bash
pixi run build-pkg robot_bringup walkie_mujoco_worlds walkie_description robot_self_filter
pixi run ros2 launch robot_bringup nav_mujoco_omnibot.launch.py world:=chula_pat14_6
pixi run world-validate chula_pat14_6 --robot          # offline, needs no build
```

Start a Zenoh router first, as described in
[MuJoCo Simulation: Run with ROS 2](../../guides/simulation-mujoco.md#run-with-ros-2).

## What changed

| File | Change |
| --- | --- |
| `simulation/mujoco/CMakeLists.txt`, `package.xml`, `.gitignore` | New `ament_cmake` package `walkie_mujoco_worlds` without dependencies. It installs the runtime files of `worlds/` (`world.toml`, `*.xml`, `*.obj`, `*.stl`, `*.msh`, `*.png`, `*.yaml`, `*.pgm`) and leaves out `authoring/`. It installs them file by file with `install(FILES)`, because ament's `--symlink-install` copies an `install(DIRECTORY ... FILES_MATCHING)` instead of linking it. Git ignores `output/`, `*.blend1` and `export_report.json`. |
| `simulation/mujoco/worlds/chula_pat14_6/` | The Chula Pat14 room: `world.toml`, the two exported scenes, `scene_assets/`, the map, the two `.blend` projects, and `authoring/` with the map sources and the room checks |
| `simulation/mujoco/tools/` | `validate_world.py`, `make_map_overlay.py` and `world_tools.py`: offline checks driven by the manifest. World-specific checks are hooks in `authoring/checks.py`. |
| `bringup/robot_bringup/world_loader.py` | Reads and validates manifests, merges a scene with `walkie.xml`, places the spawn, and writes `world.xml` and `nav2.yaml` |
| `bringup/robot_bringup/mujoco_launch.py` | The world launch arguments and their checks, shared by both launch files, and the X11 override for the MuJoCo viewer |
| `bringup/launch/mujoco_omnibot.launch.py`, `nav_mujoco_omnibot.launch.py` | The world arguments. With `world`, the Nav2 launch starts AMCL on the world's map instead of the static `map` to `odom` transform. |
| `bringup/CMakeLists.txt`, `package.xml`, `robot_bringup/__init__.py` | Installs the Python package `robot_bringup`, registers the three tests, `exec_depend` on `walkie_mujoco_worlds` |
| `bringup/test/test_mujoco_world.py`, `test_mujoco_launch_worlds.py`, `test_mujoco_world_assets.py` | Loader tests on synthetic worlds, launch argument tests, and an integration test on every installed world |
| `.github/workflows/ci.yml` | `robot_bringup` joins `TEST_PACKAGES`, so its tests fail the job. Its lint tests still do not. The lint step now runs only the tests labelled `linter`, so the world tests do not run twice. |
| `pixi.toml` | The tasks `world-validate` and `world-overlay`. `pixi.lock` is unchanged. |
| `.gitattributes` | `*.blend` and `*.exr` are binary. The exported `scene.xml` files of the worlds are marked as generated, so GitHub collapses them, but `git diff` still shows their physics. The `*.obj` meshes are also hidden from diffs. |

## How a launch loads a world

1. The launch reads `<worlds_dir>/<world>/world.toml`. `worlds_dir` defaults to the
   `worlds/` directory of the installed `walkie_mujoco_worlds` share.
2. It selects the scene of `variant` and the pose of `spawn` or `spawn_pose`. Empty values
   select the manifest defaults.
3. `world_loader` merges the scene with the installed `walkie.xml`: the robot first, then
   the scene's bodies, the four wheel-floor pairs of `description/mujoco/scene.xml` and the
   collision classes. It writes the spawn into the free base and into every keyframe.
4. It writes `world.xml` into `<world_output_dir>/<world>/<variant>/`, by default under
   `$ROS_HOME/mujoco_worlds`. The Nav2 launch also writes `nav2.yaml` there: a copy of
   `params_file` with the AMCL initial pose at the spawn.
5. `mujoco_ros2_control` loads `world.xml`, `map_server` serves the world's map, and AMCL
   starts at the spawn.

The exported scene, `walkie.xml` and the source Nav2 parameter file are never changed.

## Key decisions

### A manifest per world { #manifest-per-world }

A world is one directory with a `world.toml`. The manifest names the variants, the map,
the spawns and the defaults, so the launch files have no code for a particular world, and a
new world needs no code change. The format is in
[MuJoCo Simulation: The manifest](../../guides/simulation-mujoco.md#world-manifest).

| Option | Why it was or was not chosen |
| --- | --- |
| **`world.toml` next to the assets (chosen)** | The world directory is self-contained: copy it, and its relative paths stay valid. `worlds_dir` can point at a copy outside the repository. The launch files, the tools and the tests read the same file with the same loader. |
| Manifests in `bringup/config/mujoco/worlds/` | The first version. The manifest and its assets lived in two packages, with paths from one into the other. |
| Launch arguments only (`mujoco_model`, `map` and a pose) | Already works for a custom world, but every user must pass the matching map and spawn, and keep the AMCL initial pose equal to the spawn by hand |

The loader rejects unknown and missing keys, so a typo stops the launch instead of
falling back to a default. `schema_version` leaves room for a later format change.

### A separate asset package { #asset-package }

The worlds live in their own package, `walkie_mujoco_worlds`, which holds only data and has
no dependencies. `robot_bringup` depends on it at run time.

| Option | Why it was or was not chosen |
| --- | --- |
| **`walkie_mujoco_worlds` in `simulation/mujoco/` (chosen)** | Installs only what a launch reads: 5.4 MB of the 16.3 MB in Git. The Blender projects and the authoring files stay in the source tree. Gazebo and MuJoCo keep separate packages. |
| `robot_simulation`, in `simulation/gazebo/worlds/` | The first version. It made the Gazebo package a dependency of every MuJoCo launch and mixed the worlds of two simulators. `robot_simulation` is Gazebo-only again. |
| A project outside the workspace | Tried and reverted. A launch then depends on a checkout outside the workspace, CI cannot test the worlds, and a world change is not reviewed together with the loader that reads it. |

### Plain Git, without LFS { #plain-git }

The world files are in plain Git. Git LFS in this repository is used only for the `*.dae`
meshes of the `description/` submodule, and `main` already keeps about 107 MB of Gazebo
models in plain Git. To keep the world small, duplicates were removed before it was added:

- The EXR lightmap was deleted. The same file is packed into `baked/baked.blend`, and
  MuJoCo reads the PNG lightmap `scene_assets/chula_baked_lightmap.png`.
- Preview images were deleted. The validator renders them on demand into the ignored
  `simulation/mujoco/output/`.
- Both variants share one `scene_assets/` directory.

The world adds about 16.3 MB: `baked.blend` 10.4 MB, the PNG lightmap 3.4 MB and the baked
OBJ 1.6 MB. If worlds grow much larger, LFS for `*.blend` and textures is the next step.
It costs every clone a Git LFS setup and a `git lfs pull` for the superproject as well.

### The map is a copy { #map-copy }

`map/map.yaml` and `map/map.pgm` are byte-for-byte copies of map collection
`30_16102026_ChulaPat14_6`, with their source revision and hashes in
`authoring/source/provenance.json`. The map collection submodule,
`navigation/walkie-navigation/map`, is pinned to an earlier revision without this map, and
moving that pin belongs to the `navigation` repository. The copy also keeps the world
directory self-contained. The cost: a new capture of the room must be copied in by hand.

### The robot owns the model-wide settings { #robot-owns-model-wide-settings }

MuJoCo applies `<compiler>`, `<option>`, `<size>`, `<visual>` and `<statistic>` to the
whole model, and the top-level `<default>` to every element in it. A merged world value
therefore changes the robot too. The first loader merged both sides, and a fresh export from
the add-on took Walkie from 36.66 kg to 161 kg through its `<compiler>` inertia settings.
The Chula export's `iterations="40"`, merged in, would lower the solver iterations that the
controllers were tuned with.

The loader now takes these sections from `walkie.xml`:

| World setting | What the loader does |
| --- | --- |
| `<compiler angle>` or `eulerseq` that differs from the robot's | Error, because the world's numbers would be read in another unit. MJCF defaults to `degree`, so a world must set `angle="radian"`. |
| Other `<compiler>` values that differ | Dropped and logged as a warning |
| `<option>`, `<size>`, `<visual>`, `<statistic>` | Dropped and logged as info, except the free-camera view, the headlight, the haze color and `<statistic center>`, which pass where the robot sets none |
| `<size memory>` | Kept only when larger than a value that the robot sets. `walkie.xml` sets none, so MuJoCo sizes the memory itself. |
| Settings in the top-level `<default>`, in either model | Error. Named default classes are merged, and their names must differ. |
| `<include>`, `<keyframe>`, unsupported top-level sections | Error |

Every merged model also gets `<statistic extent="2.2">` and `<map znear="0.025">`, the
values of `description/mujoco/scene.xml`, so the head camera's near plane stays at 5.5 cm.
All dropped values are also listed in a comment in the generated `world.xml`.

!!! warning "Named worlds render with the robot's visual settings"
    A named world now uses the visual settings of `walkie.xml`, not those of its export. For
    the Chula room, the free camera's far plane is 110 m instead of 17.6 m, with 4 instead
    of 2 multisamples, a shadow map of 4096 instead of 1024, and an offscreen height of 720
    instead of 960. The head camera renders with the same settings as in the default scene.

??? info "Why errors for some settings and warnings for others"
    Dropping a compiler value such as `inertiafromgeom` keeps the world loading and
    protects the robot. The world's own inertia can change, so it is a warning. Dropping
    `angle` or `eulerseq` would silently turn every rotation in the world into another
    one, so it is an error.

    Stripping a top-level `<default>` would silently restyle the world or the robot.
    Neither Chula variant nor a fresh export uses one, so it is an error whose message says
    to move the settings into a named class.

    For memory, the merged Chula model compiled with an automatic arena of 19.0 MiB. The
    export's `memory="8M"` would have shrunk it to 8 MiB. 500 steps used at most 116 KiB.

### The loader sets the collision classes { #loader-sets-collision-classes }

The loader overwrites `contype` and `conaffinity` of every world geom with the classes of
[2.3 Collisions](03-collisions.md): FLOOR for the plane named `floor`, OBJ for geoms in a
body with a joint or below one, FURN for the rest, and no collision for geoms that the
export marks with `contype="0" conaffinity="0"`. An export does not need to know Walkie's
bit masks, and an export mistake cannot make the base collide with furniture.
`test_mujoco_world_assets.py` checks the loader's masks against the class table in the
`walkie.xml` header, so a change of the classes in the generator fails the test.

### AMCL with a world, odometry without { #amcl-with-a-world }

A world has a captured map with walls, so `nav_mujoco_omnibot.launch.py` starts AMCL on it,
with the initial pose at the spawn. Without `world`, the launch keeps the static `map` to
`odom` transform and the blank map of [2.7](07-launch.md#odometry-only-localization). The
spawn, the keyframes and the AMCL initial pose come from one value, so they cannot
disagree.

### The X11 override is scoped

The viewer of `mujoco_ros2_control` has crashed in libdecor under Wayland. On Linux, when
both `WAYLAND_DISPLAY` and `DISPLAY` are set, the launch starts only the MuJoCo process
with `WAYLAND_DISPLAY` unset and `XDG_SESSION_TYPE=x11`, so its viewer runs on XWayland.
RViz and the other nodes keep the session's backend. Without `DISPLAY`, nothing changes.
This applies with and without `world`.

## Found in the live run

- **The ignored-settings lines printed their template.** `ros2 launch` formats log records
  with `{msg}`, which drops %-style logging arguments, so the world loader's info and
  warning lines showed `%s: ignored world settings ... %s`. The loader now formats the
  message itself, and the tests read the records the way the launch prints them.
- **RViz had no 2D Pose Estimate tool.** `mujoco_nav.rviz` offered goal tools only, so an
  AMCL pose that is off could not be corrected from RViz. The config now has the
  `SetInitialPose` tool on `/initialpose`, as `default.rviz` does, and a test checks it.
- **The room's cameras streamed.** `CameraPlugin` publishes every camera of the model, so
  `chula_top`, `chula_living` and `chula_kitchen` sent 1 × 1 color, depth and `camera_info`
  images at the ZED camera's rate, without frames in TF. They cost about 1.4 % of one core
  on the plugin's render thread and did not change the real-time factor (with them removed
  or disabled, it stayed within the run-to-run noise). `mujoco_plugins.yaml` now gives each
  of them `policy: disabled`; entries for cameras the model lacks are ignored. A test fails
  when an installed world has a camera without such an entry, and when a camera uses
  `policy: polled`: in 0.1.2 its trigger reaches the wrong camera once another one is
  disabled.

## What this changes in earlier entries

| Earlier decision | Now |
| --- | --- |
| [2.7: Odometry-only localization](07-launch.md#odometry-only-localization) | Still the default. With `world`, the Nav2 launch uses AMCL. |
| [2.1: Rules for a custom world](01-model-generation.md#rules-for-a-custom-world): stay in `description/mujoco/` and include `walkie.xml` | Still true for a custom world. A named world may live anywhere and must not include the robot; the loader adds it. |
| [2. Add MuJoCo Simulation: Next steps](index.md#next-steps): build custom worlds for the competition arenas | Named worlds are the way to add them |

## Verification

Run on 2026-10-22: the offline checks, then the live launches, headless, under
`rmw_zenoh_cpp` on a laptop running other work. The Nav2 runs used `navigation` at
`5df839a`, before navigation pull request #10 merged it. Its simulation parameters
(`PathAngleCritic` `offset_from_furthest: 20`, `max_angle_to_furthest: 0.1`) are the same
as in the pinned `359719f`.

| Check | Result |
| --- | --- |
| `pixi run test --packages-select robot_bringup --ctest-args " -LE" linter`, as in CI | Passes. `test_mujoco_world` 91, `test_mujoco_launch_worlds` 24 and `test_mujoco_world_assets` 22 tests, no failures |
| `test_description.py` and `test_mjcf.py` | 101 passed, 1 strict xfail (a known controller pairing) |
| `pixi run world-validate chula_pat14_6 --robot` | 39 of 39 scene checks and 30 of 30 robot checks pass |
| `pixi run world-overlay chula_pat14_6 --check-navigation` | 9 of 9 reachability checks pass |
| Both variants, merged from the installed share and compiled | Both compile (94 bodies; 283 and 256 geoms). The merged model keeps the robot mass of 36.664 kg, has 4 wheel-floor pairs and the robot's 100 solver iterations. |
| Sensitivity of the tests | With the world's `<option>` let through on purpose, three tests fail |
| `ros2 launch ... --show-args`, both launch files | Both parse and list the world arguments |
| `mujoco_omnibot.launch.py world:=chula_pat14_6`, live | All controllers active. Rates in simulated time: `/joint_states` and odometry 55 Hz, `/scan` 28 Hz, `/unilidar/cloud` and `/filtered` 12 Hz, cameras 4.7 Hz. Real-time factor 0.93 (`lowpoly`), 0.63 (`baked`); the MuJoCo process uses 255 MB and 303 MB. The ignored-settings line appears once. |
| `nav_mujoco_omnibot.launch.py world:=chula_pat14_6`, live | The 13 Nav2 nodes are active after about 10 s; `/map` is the room's 168 × 485 map. AMCL starts at the spawn. |
| `NavigateToPose` goals, live | Spawn to the kitchen: succeeded in 36 s (`lowpoly`) and 44 s (`baked`), AMCL 0.09 m off. Back to the spawn: succeeded in 156 s after 15 collision monitor stops, AMCL 0.17 m off. From `spawn:=origin`: not reached in 200 s. |
| `/cmd_vel_key` and `/goal_pose`, live | 0.9 m of manual driving, then a goal on `/goal_pose` back to the spawn: succeeded in 10 s |
| `pixi run docs-build` | Passes |

## Known issues

| Issue | Effect | Workaround |
| --- | --- | --- |
| AMCL is 0.1 to 0.2 m off in the room | The map has no furniture, so lidar hits on the sofa and the TV desk bias the estimate. After two goals, AMCL was 0.09 and 0.17 m from the ground truth; an initial pose at the true pose settled 0.1 m off as well. | Accept it for goals in open floor. A map with the furniture would remove it ([Still open](#still-open)). |
| The scan matches the map only in part at the `living-room` spawn | In an offline check, 59 % of the 2D lidar hits were within 0.05 m of an occupied map cell (91 % at `origin`). Most other hits were on the sofa and the open door. | Correct the pose with **2D Pose Estimate** |
| The collision monitor stops the robot at the room divider | On the way from the kitchen back to the spawn, the divider end lies in the `forward_right` stop band, which reaches 0.285 m behind the robot's center. Nav2 aborts and replans until it gets through: 156 s instead of about 40 s. | Wait, or nudge the robot on `/cmd_vel_key` and resend the goal |
| Nav2 does not get through the door from `spawn:=origin` | A goal from the hallway to the living-room spawn moved the robot 0.46 m in 200 s, with 32 `Failed to make progress` aborts and no collision monitor stop | Use the `living-room` spawn for navigation |
| The Chula exports still contain their own settings | One info line per launch and per validator run lists the ignored values | None needed. A re-export with the robot's settings would silence it. |
| Concurrent launches of the same world and variant | They overwrite each other's `world.xml` and `nav2.yaml` | Pass a different `world_output_dir` to each |
| Stale properties in the `.blend` files | `baked.blend` refers to a deleted `scene_spec.json`, and both files store absolute render output paths. Neither affects the export. | Fix them in Blender |

## Still open

- **A map that matches the world.** Adding the furniture to the AMCL map, or capturing the
  map with it, would remove the AMCL offset. The map is a byte-for-byte copy from the map
  collection, so this changes the room's provenance.
- **The `forward_right` stop band.** It covers the robot's right side behind its center,
  and stopped the robot at the room divider. Changing the band is a collision monitor
  decision in `navigation/` that applies beyond this room.
- **The map collection pin.** Once `navigation/walkie-navigation/map` includes map
  `30_16102026_ChulaPat14_6`, the world could read its map from there.
