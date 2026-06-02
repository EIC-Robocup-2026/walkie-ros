#!/usr/bin/env bash
# Sets up the perception virtual environment on a new machine.
#
# Usage:
#   ./setup_env.sh          # base only (yolo_node + obb3d)
#   ./setup_env.sh --grasp  # + GraspNet grasp pipeline
#
# The venv is created at ~/perception-venv with access to ROS 2 system packages.
# Nothing is installed into system Python.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HOME/perception-venv"
GRASP=false

for arg in "$@"; do
  [[ "$arg" == "--grasp" ]] && GRASP=true
done

# ── 1. Create venv ─────────────────────────────────────────────────────────────
echo "[1/4] Creating venv at $VENV …"
uv venv "$VENV" --python /usr/bin/python3.12 --system-site-packages --seed --clear
touch "$VENV/COLCON_IGNORE"

# Verify ROS bindings are accessible
"$VENV/bin/python3" -c "import rclpy" 2>/dev/null \
  || { echo "ERROR: rclpy not found — is ROS 2 Jazzy installed?"; exit 1; }

# ── 2. Install Python deps ─────────────────────────────────────────────────────
echo "[2/4] Installing Python dependencies …"
cd "$SCRIPT_DIR"
if $GRASP; then
  VIRTUAL_ENV="$VENV" uv sync --active --extra grasp
else
  VIRTUAL_ENV="$VENV" uv sync --active
fi

# ── 3. GraspNet (only when --grasp) ───────────────────────────────────────────
if $GRASP; then
  echo "[3/4] Setting up GraspNet …"

  # graspnetAPI
  if [ ! -d "$HOME/graspnetAPI" ]; then
    git clone https://github.com/graspnet/graspnetAPI.git "$HOME/graspnetAPI"
  fi
  # Install graspnetAPI without its pinned deps (numpy==1.23.4, transforms3d==0.3.1
  # are incompatible with Python 3.12 — our pyproject.toml already covers them)
  uv pip install --python "$VENV" -e "$HOME/graspnetAPI" --no-deps --quiet
  uv pip install --python "$VENV" transforms3d --quiet

  # graspnet-baseline path
  PTH="$VENV/lib/python3.12/site-packages/graspnet_baseline.pth"
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
