#!/usr/bin/env bash
# Pixi activation hook: sourced on `pixi shell` / `pixi run`.
# Overlays the colcon workspace (source-built packages) on top of the conda env.
if [ -f "$PROJECT_ROOT/install/local_setup.sh" ]; then
    source "$PROJECT_ROOT/install/local_setup.sh"
fi
