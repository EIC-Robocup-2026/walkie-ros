"""Launch support shared by the MuJoCo launch files.

``mujoco_omnibot.launch.py`` and ``nav_mujoco_omnibot.launch.py`` declare the
same world arguments and resolve them the same way: an empty ``world`` keeps
the launch file's own defaults, and ``world:=NAME`` prepares the exported world
through ``robot_bringup.world_loader`` before any node reads the model.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys

from launch.actions import (
    DeclareLaunchArgument, GroupAction, SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
import launch.logging
from launch.substitutions import LaunchConfiguration
from robot_bringup import world_loader


WORLD_ARGUMENTS = (
    "world", "variant", "worlds_dir", "world_output_dir", "spawn", "spawn_pose"
)


def default_world_output_dir() -> str:
    """Return the default cache for generated world files under ROS_HOME."""
    ros_home = os.environ.get("ROS_HOME") or str(Path.home() / ".ros")
    return str(Path(ros_home).expanduser() / "mujoco_worlds")


def declare_world_arguments(*, nav2: bool = False) -> list[DeclareLaunchArgument]:
    """Declare the launch arguments that select and place an exported world.

    ``nav2`` is True for a launch that also generates Nav2 parameters; it only
    changes the description of ``world_output_dir``.
    """
    output = "generated world MJCF and Nav2 parameters" if nav2 else "generated world MJCF"
    return [
        DeclareLaunchArgument(
            "world", default_value="",
            description="World directory name, e.g. chula_pat14_6; empty runs the "
                        "launch file's default scene",
        ),
        DeclareLaunchArgument(
            "variant", default_value="",
            description="Variant from the world's world.toml [variants]; empty uses "
                        "its default_variant",
        ),
        DeclareLaunchArgument(
            "worlds_dir", default_value="",
            description="Directory with one subdirectory per world; empty uses the "
                        "installed walkie_mujoco_worlds share",
        ),
        DeclareLaunchArgument(
            "world_output_dir", default_value=default_world_output_dir(),
            description=f"Writable cache for the {output}",
        ),
        DeclareLaunchArgument(
            "spawn", default_value="",
            description="Named spawn from the world's [spawns]; empty uses its default_spawn",
        ),
        DeclareLaunchArgument(
            "spawn_pose", default_value="",
            description="Custom 'X Y YAW_DEG' in the map frame; cannot be combined with spawn",
        ),
    ]


def prepare_world_from_context(context, *, description_dir, replaced_defaults=None,
                               nav2_params_source=None) -> dict | None:
    """Prepare the world selected by the launch arguments.

    Return None when ``world`` is empty, otherwise the dictionary returned by
    ``world_loader.prepare_world``. ``replaced_defaults`` maps the launch
    arguments that a world supplies (for example ``mujoco_model``) to their
    declared defaults; world mode rejects any other value. A value equal to the
    default cannot be told apart from an omitted argument and is accepted.

    Raise ValueError for spawn arguments without a world, for conflicting
    arguments, and for every error that the world loader reports in a world,
    its manifest or the robot model, such as malformed XML or a missing asset.
    An OSError can still come from the files themselves, for example an
    unreadable scene or a cache that cannot be written.
    """
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context).strip()

    world = value("world")
    spawn, spawn_pose = value("spawn"), value("spawn_pose")
    if not world:
        if spawn or spawn_pose:
            raise ValueError("spawn and spawn_pose require world:=NAME")
        return None
    conflicts = [
        name for name, default in (replaced_defaults or {}).items()
        if value(name) != str(default).strip()
    ]
    if conflicts:
        names = ", ".join(conflicts)
        raise ValueError(f"world:={world} supplies {names}; do not also set {names}")

    description = Path(description_dir) / "mujoco"
    # Route the loader's messages (ignored world settings) to the launch
    # screen and log file; a plain Python logger would drop its info messages.
    launch.logging.get_logger(world_loader.LOGGER.name)
    return world_loader.prepare_world(
        world=world,
        variant=value("variant") or None,
        worlds_dir=value("worlds_dir") or None,
        output_dir=Path(value("world_output_dir") or default_world_output_dir()).expanduser(),
        robot_path=description / "walkie.xml",
        wheel_scene_path=description / "scene.xml",
        nav2_params_source=nav2_params_source,
        spawn=spawn or None,
        spawn_pose=spawn_pose or None,
    )


def x11_override_needed(environ=None, platform=None) -> bool:
    """Return True when the MuJoCo viewer should use X11 instead of Wayland.

    The GLFW viewer of mujoco_ros2_control has crashed in libdecor under
    Wayland. On Linux, when both WAYLAND_DISPLAY and DISPLAY are set, XWayland
    offers an X11 display and the viewer is moved there. Without DISPLAY there
    is no X11 server to fall back to, so the environment is left unchanged.
    """
    environ = os.environ if environ is None else environ
    platform = sys.platform if platform is None else platform
    return (platform.startswith("linux")
            and bool(environ.get("WAYLAND_DISPLAY"))
            and bool(environ.get("DISPLAY")))


def with_viewer_backend(action, environ=None, platform=None):
    """Return ``action``, scoped to an X11 viewer backend when one is needed.

    The environment change applies only inside the returned GroupAction, so
    other nodes such as RViz keep the session's normal backend.
    """
    if not x11_override_needed(environ, platform):
        return action
    return GroupAction(actions=[
        UnsetEnvironmentVariable("WAYLAND_DISPLAY"),
        SetEnvironmentVariable("XDG_SESSION_TYPE", "x11"),
        action,
    ])
