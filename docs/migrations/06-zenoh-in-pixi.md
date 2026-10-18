# 6. Select Zenoh in Every Pixi Command

**Goal:** make every `pixi run` and `pixi shell` use the team middleware,
Zenoh (`rmw_zenoh_cpp`), without an `export` line in each terminal, and run
the CI tests on it.

This entry records what changed on 2026-10-19, why, and what you need to do
after pulling. For day-to-day use, read
[Middleware (Zenoh)](../guides/middleware.md).

## In short

- **Changed:** `pixi.toml` sets `RMW_IMPLEMENTATION = "rmw_zenoh_cpp"` in
  `[activation.env]`. CI starts a Zenoh router before the tests. The guides
  no longer tell you to export the variable.
- **Why:** pixi did not set the variable before. A terminal without
  `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` used the default DDS middleware
  (`rmw_fastrtps_cpp`), which cannot talk to Zenoh nodes. Under Fast DDS, the
  real-robot Nav2 launches also fail to configure `controller_server`
  ([Troubleshooting](../troubleshooting.md#nav2-voxel-grid-incompatible-type)).
- **After pulling:** start a Zenoh router before you run nodes or tests
  ([After pulling this change](#after-pulling-this-change)).
- **Watch out:** a value exported in your terminal no longer changes
  `pixi run`. To use another middleware for one command, set it inside pixi
  ([Another middleware for one command](#another-middleware-for-one-command)).
- **Result:** checked on 2026-10-19 in a shell without the variable
  ([Verification](#verification)). The new CI step has not run on GitHub yet
  ([Still open](#still-open)).

## After pulling this change

`pixi.lock` did not change, so `pixi install` is not needed.

1. Start one Zenoh router on your computer, in its own terminal, and leave it
   running:

    ```bash
    pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
    ```

2. Run ROS commands with `pixi run`, or in a `pixi shell`, as before.
   `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` in your shell profile has no
   effect on pixi commands any more. Keep it only if other ROS workspaces on
   the computer need it.
3. Start the router before `pixi run test` as well. A test that starts nodes
   in more than one process waits for the router until CTest stops it. See
   [Tests](../guides/testing.md).

## What changed

| File | Change |
| --- | --- |
| `pixi.toml` | `RMW_IMPLEMENTATION = "rmw_zenoh_cpp"` in `[activation.env]`, next to the CUDA variables, and in `[feature.docs.activation.env]` |
| `.github/workflows/ci.yml` | New step **Start the Zenoh router** before **Test**. It starts `rmw_zenohd` with the default config in the background and waits up to 30 s for port 7447. The **Test** step runs `topic_based_system_test` on Fast DDS in a second `colcon test` call. The `colcon-logs` artifact also holds `log/rmw_zenohd.log`. |
| `CLAUDE.md` and the guides | The `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` lines are gone. The pages say that pixi selects Zenoh and that a router is still needed. [Working with Pixi](../guides/pixi.md#activation-and-the-workspace-packages) lists the `[activation.env]` variables. |

Nodes keep the default session config of `rmw_zenoh_cpp`, which connects to
a router on `tcp/localhost:7447`. The workspace sets no `ZENOH_*` variable.

??? info "How pixi applies the value"
    Checked with pixi 0.80.0:

    1. Pixi sources the activation script, `install/setup.bash`. The script
       sees only the value from your terminal.
    2. Pixi sets the variables in `[activation.env]`. `RMW_IMPLEMENTATION`
       replaces the value from your terminal and from the script.
    3. A task with its own `env` table would replace it again. No task in
       `pixi.toml` sets `RMW_IMPLEMENTATION`.

    `[activation.env]` belongs to the default feature. The `docs` environment
    is declared with `no-default-feature = true`, so it does not get that
    table. `[feature.docs.activation.env]` sets the same value there, so every
    environment agrees, as in walkie-agent-ros and walkie-database. The `docs`
    environment has no ROS.

    A task's `env` table would reach only that task. Most ROS commands are
    not tasks, for example `pixi run ros2 launch ...`, so the variable is set
    for the whole environment instead.

## Another middleware for one command

Set the variable inside the pixi environment, after pixi has set its value:

```bash
pixi run env RMW_IMPLEMENTATION=rmw_fastrtps_cpp <command>
```

In a `pixi shell`, `export RMW_IMPLEMENTATION=...` also works for the rest of
that shell. All processes that must talk to each other need the same value.

## CI

The launch test of `topic_based_ros2_control` starts several nodes, each in
its own process. Without a router they cannot find each other, and CTest
stops the test after 120 s.
CI therefore starts one router, so the tests run on the middleware of the
robot. [Tests: CI](../guides/testing.md#ci) explains the steps.

One test runs on Fast DDS. The gtest `topic_based_system_test` passes, then
aborts at exit under Zenoh, because its `main()` never calls
`rclcpp::shutdown()`. The test is in the upstream submodule
`hardware/actuator/topic_based_ros2_control`. It runs in one process, so
Fast DDS changes nothing that it checks. See
[Tests: Known issues](../guides/testing.md#known-issues).

## Verification

Checked on 2026-10-19 with pixi 0.80.0 and `rmw_zenoh_cpp` 0.2.11, in a shell
without `RMW_IMPLEMENTATION` and with a test `ROS_DOMAIN_ID`:

| Check | Result |
| --- | --- |
| `pixi run printenv RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` |
| The same with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` exported | `rmw_zenoh_cpp` |
| `pixi run env RMW_IMPLEMENTATION=rmw_fastrtps_cpp printenv RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |
| `pixi run -e docs printenv RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` |
| `PATH` and `LD_LIBRARY_PATH` under `pixi run` | The CUDA folder first, then the entries from `install/setup.bash` and from the terminal |
| The commands of the CI **Test** step, with a router on port 7447 | 121 tests, 0 failures, 1 skipped |
| The same tests in one `colcon test` call, all on Zenoh | `topic_based_system_test` passes, then aborts at exit (return code -6). Run on its own, it aborted in 6 of 6 runs on Zenoh and passed in 3 of 3 runs on Fast DDS. |
| `topic_based_system_test` with `rclcpp::shutdown()` added, in a scratch copy of the package | 6 of 6 runs passed on Zenoh |
| Launch test of `topic_based_ros2_control` without a router | Every node logs `Unable to connect to a Zenoh router`. `ros2_control_node` waits for `robot_description` until CTest stops the test after 120 s. |
| `test_collision_checker` of `nav2_costmap_2d` | Passes in 7 s with a router. Without one, CTest stops it after 60 s. |
| `pixi run -e docs docs-build` | Builds without warnings |

For the runs without a router, the test processes connected to an unused
port, through a scratch copy of the default session config. The router of
the computer kept running.

## Still open

- The **Start the Zenoh router** step and the split **Test** step have not
  run on a GitHub runner yet. On the first run, check that **Test** passes
  and that the `colcon-logs` artifact contains `log/rmw_zenohd.log`.
- `topic_based_system_test` runs on Fast DDS until its `main()` calls
  `rclcpp::shutdown()`. That needs a change in the upstream
  `topic_based_ros2_control` repository and a submodule update.
