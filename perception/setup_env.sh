#!/usr/bin/env bash
# Sets up the perception virtual environment on a new machine.
#
# Usage:
#   ./setup_env.sh          # base only (yolo_node + obb3d)
#   ./setup_env.sh --grasp  # + GraspNet grasp pipeline
#
# The venv is built from the active environment's Python, the `python3` on PATH,
# because that is the Python its rclpy works with: pixi's under `pixi run`, the
# system one with /opt/ros/jazzy/setup.bash sourced. It gets access to that
# environment's ROS 2 packages. Nothing is installed into the environment itself.
#
# The venv goes to ~/perception-venv, or to $PERCEPTION_VENV_DIR if set. The nodes
# find it through PERCEPTION_VENV, which is a different variable: the path of the
# venv's Python executable, not of the venv directory.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# -s keeps symlinks in the path. A venv's bin/python3 is a symlink, and following it
# would turn a mistaken PERCEPTION_VENV_DIR into the path of the base interpreter.
DEFAULT_VENV="$(realpath -ms "$HOME/perception-venv")"
VENV="$(realpath -ms "${PERCEPTION_VENV_DIR:-$DEFAULT_VENV}")"
GRASP=false

for arg in "$@"; do
  [[ "$arg" == "--grasp" ]] && GRASP=true
done

SYNC_ARGS=(--no-build-package numpy)
$GRASP && SYNC_ARGS+=(--extra grasp)

# ── Preflight: an existing venv stays untouched until these pass ───────────────
# Step 1 deletes everything in an existing target, so the target must be a missing
# path, an empty directory, or a venv.
if [ -e "$VENV" ] || [ -L "$VENV" ]; then
  if [ ! -d "$VENV" ]; then
    {
      echo "ERROR: $VENV exists and is not a directory."
      echo "PERCEPTION_VENV_DIR is the directory of the venv. PERCEPTION_VENV is its Python executable."
    } >&2
    exit 1
  fi
  if [ -n "$(ls -A "$VENV")" ] && [ ! -f "$VENV/pyvenv.cfg" ]; then
    {
      echo "ERROR: $VENV is not empty and is not a venv (no pyvenv.cfg)."
      echo "Creating the venv there would delete its contents. Nothing was created or removed."
      echo "Set PERCEPTION_VENV_DIR to a new path, an empty directory, or an existing venv."
    } >&2
    exit 1
  fi
fi

if ! BASE_PYTHON="$(command -v python3)"; then
  echo "ERROR: no python3 on PATH. Run this under 'pixi run', or source /opt/ros/jazzy/setup.bash first." >&2
  exit 1
fi
PYTHON_VERSION="$("$BASE_PYTHON" -c 'import platform; print(platform.python_version())')"

# Verify ROS bindings are accessible
if ! IMPORT_ERROR="$("$BASE_PYTHON" -c "import rclpy" 2>&1)"; then
  {
    echo "ERROR: $BASE_PYTHON (Python $PYTHON_VERSION) cannot import rclpy:"
    echo "$IMPORT_ERROR" | tail -n 1
    echo "Run this under 'pixi run', or source /opt/ros/jazzy/setup.bash first."
  } >&2
  exit 1
fi

if ! command -v uv >/dev/null; then
  echo "ERROR: uv is not on PATH. It does not come from pixi: https://docs.astral.sh/uv/getting-started/installation/" >&2
  exit 1
fi

# Verify perception/uv.lock has installable packages for this Python. A dry run
# downloads no packages and creates nothing, so it fails before any large download.
# It targets a path that never exists, so uv does not judge a venv already at $VENV.
# numpy may not fall back to a source build: that takes long and needs a compiler.
cd "$SCRIPT_DIR"
if ! SYNC_ERROR="$(UV_PROJECT_ENVIRONMENT="$VENV.preflight" uv sync --locked --dry-run \
    --python "$BASE_PYTHON" "${SYNC_ARGS[@]}" 2>&1)"; then
  {
    echo "ERROR: 'uv sync --locked --dry-run' failed in perception/ on Python $PYTHON_VERSION ($BASE_PYTHON):"
    # A dry run first prints what it would do. Only the rest explains the failure.
    echo "$SYNC_ERROR" | grep -Ev '^(Using |Would |Resolved |Audited | *[-+~] )' || true
    echo "Nothing was created or removed."
    echo "If uv names a package without a wheel for this Python, use an environment with a supported"
    echo "Python, or update the pins in perception/pyproject.toml and run 'uv lock' in perception/."
    echo "If uv says the lockfile needs to be updated, run 'uv lock' in perception/."
  } >&2
  exit 1
fi

# ── 1. Create venv ─────────────────────────────────────────────────────────────
# From here on an existing venv is gone: a later failure, such as a lost network
# connection, leaves a partial venv. Run the script again to replace it.
echo "[1/4] Creating venv at $VENV from $BASE_PYTHON (Python $PYTHON_VERSION) …"
uv venv "$VENV" --python "$BASE_PYTHON" --system-site-packages --seed --clear
touch "$VENV/COLCON_IGNORE"

# ── 2. Install Python deps ─────────────────────────────────────────────────────
# --python names the venv's own interpreter. Otherwise a .python-version file makes
# uv replace the venv with another Python, without the system site packages.
echo "[2/4] Installing Python dependencies …"
VIRTUAL_ENV="$VENV" uv sync --active --python "$VENV/bin/python3" "${SYNC_ARGS[@]}"

# ── 3. GraspNet (only when --grasp) ───────────────────────────────────────────
if $GRASP; then
  echo "[3/4] Setting up GraspNet …"

  # graspnetAPI
  if [ ! -d "$HOME/graspnetAPI" ]; then
    git clone https://github.com/graspnet/graspnetAPI.git "$HOME/graspnetAPI"
  fi
  # Install graspnetAPI without its pinned deps (numpy==1.23.4, transforms3d==0.3.1
  # have no builds for current Python versions — our pyproject.toml already covers them)
  uv pip install --python "$VENV" -e "$HOME/graspnetAPI" --no-deps --quiet
  uv pip install --python "$VENV" transforms3d --quiet

  # graspnet-baseline path
  SITE_PACKAGES="$("$VENV/bin/python3" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  PTH="$SITE_PACKAGES/graspnet_baseline.pth"
  if [ -d "$HOME/graspnet-baseline" ]; then
    printf '%s\n' \
      "$HOME/graspnet-baseline" \
      "$HOME/graspnet-baseline/models" \
      "$HOME/graspnet-baseline/pointnet2" \
      "$HOME/graspnet-baseline/knn" > "$PTH"
    echo "  graspnet-baseline registered at $PTH"

    # Re-register already-compiled CUDA extensions into the new venv (no rebuild)
    for ext in pointnet2 knn; do
      BUILD_DIR="$HOME/graspnet-baseline/$ext/build"
      if [ -d "$BUILD_DIR" ]; then
        echo "  Installing $ext extension (skip-build) …"
        (cd "$HOME/graspnet-baseline/$ext" && \
          "$VENV/bin/python3" setup.py install --skip-build -q 2>&1 | tail -1)
      else
        echo "  WARNING: $ext not yet compiled — see README §2 to build CUDA extensions."
      fi
    done
  else
    echo "  WARNING: ~/graspnet-baseline not found."
    echo "  Clone it and build the CUDA extensions manually (see README §3)."
  fi
else
  echo "[3/4] Skipping GraspNet (run with --grasp to include)"
fi

# ── 4. Done ────────────────────────────────────────────────────────────────────
echo "[4/4] Done."
echo ""
echo "  Venv : $VENV"
if [ "$VENV" != "$DEFAULT_VENV" ]; then
  echo ""
  echo "  This is not the default location, so point the nodes at it:"
  echo "    export PERCEPTION_VENV=$VENV/bin/python3"
fi
if $GRASP && [ ! -d "$HOME/graspnet-baseline" ]; then
  echo ""
  echo "  Remaining manual step — build CUDA extensions:"
  echo "    git clone https://github.com/graspnet/graspnet-baseline.git ~/graspnet-baseline"
  echo "    source $VENV/bin/activate"
  echo "    export TORCH_CUDA_ARCH_LIST=\"<your_sm>\"  # e.g. 8.9 for RTX 40xx"
  echo "    cd ~/graspnet-baseline/pointnet2 && python setup.py install"
  echo "    cd ~/graspnet-baseline/knn       && python setup.py install"
  echo "    Place checkpoint at ~/graspnet-baseline/logs/log_rs/checkpoint-rs.tar"
fi
echo ""
echo "  Build the ROS package:"
echo "    cd ~/Walkie_Project/walkie_ros"
echo "    colcon build --packages-select grasp_interfaces walkie_perception --symlink-install"
