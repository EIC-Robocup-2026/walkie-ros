# ZED 2i + ROS 2 Jazzy + Docker — Debugging Playbook

## The setup
- **Base image:** `stereolabs/zed:5.2-gl-devel-cuda12.8-ubuntu24.04`
  (note: 5.3 / cuda13.0 tags didn't exist yet — verify with `docker manifest inspect` before using)
- **Stack:** ZED SDK 5.2 + ROS 2 Jazzy + ZED ROS 2 wrapper, on an Acer Nitro AN515-45
  (hybrid Intel iGPU + NVIDIA dGPU laptop)
- **Goal:** run the ZED 2i wrapper in a container via docker compose

## The final root cause
The laptop was using its **Intel integrated GPU** instead of the NVIDIA discrete GPU.
CUDA had no NVIDIA device to find → `cudaErrorNoDevice` everywhere. Fixed with:

```bash
sudo prime-select nvidia
sudo reboot
```

## CUDA version facts (so I don't second-guess this again)
- Host driver "CUDA version" (from `nvidia-smi`) just means the **max** CUDA it supports.
  Container CUDA is the **actual toolkit** inside.
- **Rule: host driver ≥ container CUDA = fine.** Newer host running older container CUDA is correct and intended.
- **Never downgrade the host driver to "match"** a container — wrong fix, breaks other things.
- CUDA 12.8 is the stable pick for the ROS ecosystem; 13.0 is newer but third-party deps may lag.

## The bugs I hit, in order (each was a separate issue)

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | `Temporary failure resolving packages.ros.org` during build | Docker build had no DNS | `sudo systemctl restart docker`, or build with `--network=host`, or set DNS in `/etc/docker/daemon.json` |
| 2 | `Unable to locate package ros-jazzy-point-cloud-transport-plugins` | Side effect of #1 (apt index never downloaded) | Fix DNS first; package then resolves |
| 3 | `collect2: ld returned 1` on `zed_rgb_convert` + `zed_topic_benchmark` | Broken examples in `zed-ros2-examples` | `colcon build --packages-skip zed_debug zed_rgb_convert zed_topic_benchmark` — they aren't needed to run the camera |
| 4 | `no CUDA-capable device` but `deploy:` block present | `deploy.resources` is **Swarm-only**, ignored by plain `docker compose up` | Use `runtime: nvidia` + `gpus: all` + `NVIDIA_VISIBLE_DEVICES=all` |
| 5 | `nvidia-smi` works in container but CUDA fails | `nvidia_uvm` kernel module blocked by **Secure Boot** (unsigned module) | Disable Secure Boot in BIOS (F2 on Nitro), or enroll/sign a MOK |
| 6 | Still `cudaErrorNoDevice`; ZED Diagnostic says "OpenGL default GPU incorrect" | System using **Intel iGPU**, not NVIDIA | `sudo prime-select nvidia && reboot` ← **the real fix** |

## The diagnostic ladder (run top-down next time CUDA fails in a container)

```bash
# 1. Which GPU is active? (the #1 laptop gotcha)
sudo prime-select query                 # want: nvidia
glxinfo | grep "OpenGL vendor"          # want: NVIDIA Corporation

# 2. Secure Boot blocking modules?
mokutil --sb-state                      # if "enabled" → suspect unsigned nvidia_uvm
lsmod | grep nvidia_uvm                 # should be loaded
dmesg | grep -iE "PKCS#7|signature|lockdown"

# 3. Driver/kernel module match? (reboot fixes mismatch)
modinfo nvidia | grep ^version
nvidia-smi | grep "Driver Version"      # these two must match

# 4. Device nodes present?
ls /dev/nvidia*                         # want nvidia0, nvidiactl, nvidia-uvm, ...

# 5. Minimal CUDA test, bypassing ZED entirely:
docker run --rm --gpus all nvidia/cuda:12.8.0-devel-ubuntu24.04 bash -lc '
cat > /tmp/t.cu << "EOF"
#include <cuda_runtime.h>
#include <cstdio>
int main(){int n=0; cudaError_t e=cudaGetDeviceCount(&n);
printf("%s count=%d\n", cudaGetErrorString(e), n); return 0;}
EOF
nvcc /tmp/t.cu -o /tmp/t && /tmp/t'     # want: "no error count=1"
```

If step 5 passes but the ZED container fails → it's container config (#4).
If step 5 fails → it's host-level (#1, #5, or #6).

## Compose file gotchas to remember
- **`deploy.resources.reservations.devices` does nothing without Swarm.** Use `runtime: nvidia` + `gpus: all`.
- **Don't bind-mount `/dev:/dev`** — it can overwrite the NVIDIA device nodes the container toolkit injects.
  Use `privileged: true` (USB camera) and let the toolkit handle GPU nodes.
- **Persist `zed_resources` + `zed_settings` volumes** — otherwise the SDK re-optimizes AI models (minutes) on every run.

## Working compose snippet (GPU passthrough that actually works)

```yaml
services:
  zed_wrapper:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: zed_wrapper_container
    network_mode: host
    ipc: host
    pid: host
    privileged: true
    tty: true
    stdin_open: true
    group_add:
      - dialout
    runtime: nvidia                 # <-- needed
    gpus: all                       # <-- needed (Compose Spec v2.3+)
    environment:
      - USER
      - DISPLAY=${DISPLAY:?err}
      - XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}
      - NVIDIA_DRIVER_CAPABILITIES=all
      - NVIDIA_VISIBLE_DEVICES=all  # <-- needed
      - ROS_DOMAIN_ID=42
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ../:/home/ws/src
      - zed_resources:/usr/local/zed/resources
      - zed_settings:/usr/local/zed/settings
      # NOTE: do NOT mount /dev:/dev — it clobbers NVIDIA device nodes
    command: >
      bash -c "source /opt/ros/jazzy/setup.bash &&
               source /home/zed_deps_ws/install/setup.bash &&
               exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i"

volumes:
  zed_resources:
  zed_settings:
```

## Clean rebuild commands

```bash
# Targeted (keeps the ZED model cache):
docker compose down --rmi all --remove-orphans
docker compose build --no-cache
docker compose up

# Nuclear (⚠️ wipes ALL unused images/volumes machine-wide):
docker compose down --rmi all --volumes --remove-orphans
docker system prune -a --volumes
```

## Verify the camera is running

```bash
docker exec -it zed_wrapper_container bash
# inside:
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/rgb/image_rect_color   # ~15-30 Hz expected
```

## One-line takeaway
**On a hybrid-graphics laptop, before debugging CUDA-in-Docker for hours, check
`prime-select query` and `glxinfo | grep "OpenGL vendor"` first** — if it's on Intel,
nothing CUDA will ever work until you switch to NVIDIA.
