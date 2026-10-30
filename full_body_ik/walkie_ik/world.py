"""
The main sim's named MuJoCo world (world:=NAME of nav_mujoco_omnibot.launch.py) for the preview.

The world is assembled by robot_bringup.world_loader, as the launch file does, into this tool's
own cache (WORLD_OUTPUT_DIR), so the files the running sim loaded are never rewritten. The
MuJoCo world frame is the map frame (world_loader places the spawn at its map coordinates), so
the preview robot is drawn at its map pose.
"""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from robot_bringup import world_loader

WORLD_OUTPUT_DIR = Path(os.environ.get("ROS_HOME") or Path.home() / ".ros").expanduser() / "mujoco_worlds_full_body_ik"


def prepare_world(world: str, variant: str | None = None, spawn: str | None = None,
                  worlds_dir: str | None = None) -> dict:
    """world_loader.prepare_world() with the walkie_description robot: {"model_path", "map_path", ...}."""
    description = Path(get_package_share_directory("walkie_description")) / "mujoco"
    return world_loader.prepare_world(
        world=world, variant=variant, worlds_dir=worlds_dir, output_dir=WORLD_OUTPUT_DIR,
        robot_path=description / "walkie.xml", wheel_scene_path=description / "scene.xml",
        spawn=spawn)


def map_xy_range(map_yaml: str, margin: float = 1.0):
    """((x min, x max), (y min, y max)) of a map_server map in the map frame, grown by `margin`."""
    with open(map_yaml) as f:
        info = yaml.safe_load(f)
    width, height = _pgm_size(Path(map_yaml).parent / info["image"])
    x0, y0 = info["origin"][:2]
    res = info["resolution"]
    return ((x0 - margin, x0 + width * res + margin), (y0 - margin, y0 + height * res + margin))


def _pgm_size(path: Path):
    """(width, height) in pixels from a PGM header (P2/P5, comments allowed)."""
    tokens = []
    with open(path, "rb") as f:
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: truncated PGM header")
            tokens += line.split(b"#")[0].split()
    if tokens[0] not in (b"P2", b"P5"):
        raise ValueError(f"{path}: not a PGM image")
    return int(tokens[1]), int(tokens[2])
