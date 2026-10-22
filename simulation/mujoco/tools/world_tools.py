"""Shared helpers for the MuJoCo world authoring tools in this directory.

The tools run from the source tree with the pixi Python and need no colcon
build: robot_bringup.world_loader is imported from bringup/ so that manifests
are validated exactly as the ROS launch files validate them. Every output goes
below simulation/mujoco/output/<world>/, which git ignores.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType

MUJOCO_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = MUJOCO_DIR.parents[1]
WORLDS_DIR = MUJOCO_DIR / "worlds"
OUTPUT_DIR = MUJOCO_DIR / "output"
ROBOT_MJCF = REPO_DIR / "description" / "mujoco" / "walkie.xml"
HOOKS_FILE = Path("authoring") / "checks.py"

sys.path.insert(0, str(REPO_DIR / "bringup"))
from robot_bringup import world_loader  # noqa: E402


def add_world_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the world selection arguments shared by all tools."""
    parser.add_argument("world", help="World directory name, e.g. chula_pat14_6")
    parser.add_argument("--worlds-dir", type=Path, default=WORLDS_DIR,
                        help="Directory with one subdirectory per world (default: %(default)s)")
    parser.add_argument("--output", type=Path,
                        help="Output directory (default: simulation/mujoco/output/<world>)")


def open_world(args: argparse.Namespace) -> tuple[Path, dict, Path]:
    """Return the world directory, its validated manifest and the output directory."""
    directory, manifest = world_loader.load_world(args.world, args.worlds_dir)
    output = (args.output or OUTPUT_DIR / args.world).resolve()
    output.mkdir(parents=True, exist_ok=True)
    return directory, manifest, output


def load_hooks(directory: Path) -> ModuleType | None:
    """Import the world's optional authoring/checks.py, or return None."""
    path = directory / HOOKS_FILE
    if not path.is_file():
        return None
    spec = importlib.util.spec_from_file_location(f"{directory.name}_authoring_checks", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    print(f"Wrote {path}")


def source_revision(model) -> str | None:
    """Return the exporter's source revision stored as custom text, if any."""
    import mujoco

    for index in range(model.ntext):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TEXT, index) or ""
        if name.endswith("source_revision"):
            start, size = model.text_adr[index], model.text_size[index]
            return model.text_data[start:start + size].rstrip(b"\0").decode()
    return None
