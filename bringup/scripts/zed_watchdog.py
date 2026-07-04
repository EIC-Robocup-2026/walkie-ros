#!/usr/bin/python3
# NOTE: absolute interpreter on purpose. This node is launched from
# real_omnibot.launch.py, which runs under the walkie-commander environment
# where `python3` on PATH is a 3.11 venv -- but ROS 2 Jazzy's rclpy is built for
# the system 3.12. `#!/usr/bin/env python3` would pick the venv and fail to load
# rclpy's C extension. The system interpreter inherits ROS's PYTHONPATH and works.
"""
ZED head-camera watchdog / supervisor.

Why this exists
---------------
The head ZED 2i has TWO observed long-run failure modes, and a plain process
restart only cures one of them:

  1. **Solid green.** A USB link glitch (`uvcvideo: Non-zero status (-71)` / a
     USB disconnect + re-enumeration) leaves the running zed_node holding a
     stale device handle. `grab()` keeps returning frames at ~9 Hz, but they are
     a zeroed YUV buffer that converts to a uniform BGRA (0,134,0,255) green,
     with all-NaN depth. The ZED SDK's own recovery never fires (from its point
     of view frames are still arriving). Detection must be **content-based** and
     recovery is a node restart -- the device re-enumerates cleanly, only the
     handle was dead.

  2. **Device fell off the bus.** The ZED 2i drops off USB entirely (power /
     cable / thermal). zed_node stops streaming and `sl::Camera::open()` returns
     `CAMERA STREAM FAILED TO START` on every relaunch until the device comes
     back. A node/launch restart CANNOT fix this -- there is no device to open.
     (This is what bit us on 2026-06-28: the camera dropped ~1 h in, five fast
     relaunches all failed to open, the old watchdog hit its restart-rate limit
     and *permanently gave up*, and the robot ran blind for ~55 min -- even
     though the device re-enumerated shortly after.)

What it does
------------
This node OWNS the ZED process. It launches the stock
`ros2 launch zed_wrapper zed_camera.launch.py ...` as a child process group,
subscribes to the colour stream, and restarts the ZED when:
  * frames are a uniform/near-constant colour for `fault_seconds` (green fault),
  * no frame arrives for `stall_seconds` (stream stall),
  * the ZED child process exits on its own (e.g. `Camera::open` gave up), or
  * no first frame arrives within `init_timeout_seconds` of a (re)launch.

Recovery policy (changed after the 2026-06-28 incident):
  * After a relaunch it stays muted for `cooldown_seconds` (NEURAL_PLUS reload
    is slow) and waits for the first real frame before re-arming the stall timer.
  * If `max_restarts` happen within `restart_window_seconds`, a fast relaunch is
    clearly not curing the fault, so it switches to a **persistent slow-retry**
    loop (`slow_retry_seconds`) instead of giving up -- a robot running blind is
    worse than a slow recovery loop -- and, if `usb_reset_enabled`, issues a USB
    port reset (vendor `usb_reset_vendor_id`) before each slow relaunch to bring
    a fallen-off device back. As soon as a healthy frame returns it drops back to
    fast response.
  * Every decision is mirrored to `decision_log_path` (default
    `~/.ros/zed_watchdog.log`) AND to the ROS logger, so "switched to slow retry"
    / "USB reset" / "recovered" are never lost to launch's pipe buffering.

Set `monitor_only:=true` to detect-and-log without owning/killing the process
(safe for testing against an already-running ZED).
"""

import datetime
import fcntl
import glob
import os
import shutil
import signal
import subprocess
import threading
import time
from collections import deque

import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

# Channels per encoding the ZED can emit on the colour topic.
_CHANNELS = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3, "mono8": 1}

# ioctl number for USBDEVFS_RESET == _IO('U', 20).
_USBDEVFS_RESET = (ord("U") << 8) | 20


def _default_zed_launch_arguments():
    """Default args for `ros2 launch zed_wrapper zed_camera.launch.py`.

    Mirrors the watchdog-owned ZED launch in real_omnibot.launch.py so running
    this node directly (`ros2 run robot_bringup zed_watchdog.py`) brings the head
    ZED up with the same camera name/frames AND the project's tuned zed2i.yaml
    depth params (NEURAL_PLUS + post-processing filters) loaded via
    `ros_params_override_path`. When launched from real_omnibot.launch.py these
    are overridden by the `zed_launch_arguments` parameter it passes.
    """
    args = [
        "camera_model:=zed2i",
        "camera_name:=zed_head",
        "base_frame:=zed_head_camera_link",
        "publish_urdf:=false",
        "publish_tf:=false",
    ]
    try:
        params = os.path.join(
            get_package_share_directory("robot_bringup"),
            "config", "camera", "zed2i.yaml",
        )
        args.append("ros_params_override_path:=" + params)
    except PackageNotFoundError:
        pass  # package not found (uninstalled): fall back to wrapper defaults
    return args


def classify_frame(data, height, width, encoding, sample_px=2000, std_eps=1.0):
    """Classify one raw Image payload as healthy or the green/uniform fault.

    Returns (is_bad, stats). A frame is "bad" when its COLOUR channels are
    (near-)constant across the sampled pixels -- the green fault has per-channel
    std == 0, a real scene has std of tens. We look at colour channels only: the
    alpha channel is a constant 255 even on healthy bgra8 frames, so including it
    would mask the signal. `modal_frac` (fraction of sampled pixels equal to the
    single most common pixel) is reported as a cross-check.

    Pure function (no ROS) so it can be unit-tested offline.
    """
    ch = _CHANNELS.get(encoding)
    if ch is None:
        # Unknown encoding -> don't claim a fault; let the stall timer handle it.
        return False, {"encoding": encoding, "reason": "unknown_encoding"}

    arr = np.frombuffer(data, dtype=np.uint8)
    if arr.size < height * width * ch:
        return False, {"encoding": encoding, "reason": "short_buffer"}

    flat = arr[: height * width * ch].reshape(-1, ch)

    # Deterministic subsample for speed (a 1080p frame is 2M pixels).
    step = max(1, flat.shape[0] // sample_px)
    samp = flat[::step]

    color = samp[:, :3] if ch >= 3 else samp
    per_ch_std = color.std(axis=0)
    max_color_std = float(per_ch_std.max())

    cols, cnts = np.unique(samp, axis=0, return_counts=True)
    modal_idx = int(cnts.argmax())
    modal_frac = float(cnts[modal_idx] / samp.shape[0])

    is_bad = max_color_std < std_eps
    stats = {
        "encoding": encoding,
        "max_color_std": round(max_color_std, 3),
        "modal": cols[modal_idx].tolist(),
        "modal_frac": round(modal_frac, 4),
    }
    return is_bad, stats


class ZedWatchdog(Node):
    def __init__(self):
        super().__init__("zed_watchdog")

        # --- Parameters ---
        p = self.declare_parameter
        self._image_topic = p("image_topic",
                              "/zed_head/zed_node/rgb/color/rect/image").value
        self._fault_seconds = float(p("fault_seconds", 4.0).value)
        self._stall_seconds = float(p("stall_seconds", 8.0).value)
        self._cooldown_seconds = float(p("cooldown_seconds", 30.0).value)
        # No first frame within this long after a (re)launch (process still alive
        # but never streams) -> treat as a fault. Generous: > cooldown and the
        # NEURAL_PLUS warm-up. A hard `Camera::open` failure makes the child
        # *exit*, which is caught immediately by the process-death check instead.
        self._init_timeout_seconds = float(p("init_timeout_seconds", 45.0).value)
        self._sample_px = int(p("sample_px", 2000).value)
        self._std_eps = float(p("std_eps", 1.0).value)
        self._monitor_only = bool(p("monitor_only", False).value)
        # Fast-restart budget: more than `max_restarts` within `restart_window`
        # means a plain relaunch is not curing the fault -> fall back to slow
        # retry (NOT give up).
        self._max_restarts = int(p("max_restarts", 5).value)
        self._restart_window = float(p("restart_window_seconds", 300.0).value)
        self._slow_retry_seconds = float(p("slow_retry_seconds", 60.0).value)
        # Optional USB port reset on the persistent-fault path (needs write
        # access to the device node: a udev rule or root). Off by default.
        self._usb_reset_enabled = bool(p("usb_reset_enabled", False).value)
        self._usb_vendor_id = str(p("usb_reset_vendor_id", "2b03").value)  # Stereolabs
        # Persistent decision log (independent of launch's pipe buffering).
        self._log_path = os.path.expanduser(
            str(p("decision_log_path", "~/.ros/zed_watchdog.log").value)
        )
        # ZED launch command pieces (so args stay configurable from the launch file).
        self._zed_pkg = p("zed_launch_package", "zed_wrapper").value
        self._zed_launch = p("zed_launch_file", "zed_camera.launch.py").value
        self._zed_args = list(
            p("zed_launch_arguments", _default_zed_launch_arguments()).value
        ) or []
        self._zed_args = [a for a in self._zed_args if a]

        # --- State ---
        self._lock = threading.Lock()
        self._last_frame_t = None         # monotonic time of last received frame
        self._await_first_frame_since = None  # monotonic time of last (re)launch, until frame 1
        self._bad_since = None            # monotonic time the current bad run began
        self._cooldown_until = 0.0        # monotonic time until which detection is muted
        self._restart_times = deque()     # monotonic times of recent restarts
        self._in_slow_retry = False       # tripped the fast-restart budget
        self._proc = None                 # Popen of the child ZED launch
        self._proc_dead_since = None      # monotonic time the child was found exited
        self._proc_dead_rc = None         # its exit code (for the restart reason)

        # Open the decision log (best effort; never fatal).
        self._log_fh = None
        if self._log_path:
            try:
                self._log_fh = open(self._log_path, "a", buffering=1)  # line-buffered
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"could not open decision log {self._log_path}: {e}")

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self._image_topic, self._on_image, qos)
        # Evaluate the state machine at a steady 2 Hz, independent of frame flow,
        # so the stall / process-death paths work even when zero frames arrive.
        self.create_timer(0.5, self._tick)

        mode = "MONITOR-ONLY" if self._monitor_only else "SUPERVISE"
        self._record(
            f"zed_watchdog [{mode}] topic={self._image_topic} "
            f"fault={self._fault_seconds}s stall={self._stall_seconds}s "
            f"cooldown={self._cooldown_seconds}s init_timeout={self._init_timeout_seconds}s "
            f"slow_retry={self._slow_retry_seconds}s usb_reset={self._usb_reset_enabled}",
            level="info")
        self._record(
            f"ZED launch: {self._zed_pkg}/{self._zed_launch} args={self._zed_args}",
            level="info")

        if not self._monitor_only:
            self._start_zed(initial=True)

    # ---- logging -----------------------------------------------------------
    def _record(self, msg, level="warn"):
        """Log to ROS AND to the decision file, so decisions survive pipe
        buffering / a lost terminal. `level` is one of info/warn/error.

        Each severity must call the logger from its OWN source line: rclpy caches
        severity per call site (file+function+line) and raises if one site logs
        different severities. Funnelling all levels through a single getattr call
        crashed the node on the first warn after the info banners."""
        logger = self.get_logger()
        if level == "error":
            logger.error(msg)
        elif level == "info":
            logger.info(msg)
        else:
            logger.warn(msg)
        if self._log_fh is not None:
            try:
                ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self._log_fh.write(f"{ts} [{level}] {msg}\n")
            except Exception:  # noqa: BLE001
                pass

    # ---- frame intake -------------------------------------------------------
    def _on_image(self, msg: Image):
        try:
            is_bad, stats = classify_frame(
                bytes(msg.data), msg.height, msg.width, msg.encoding,
                sample_px=self._sample_px, std_eps=self._std_eps)
        except Exception as e:  # never let a decode glitch kill the watchdog
            self.get_logger().warn(f"classify failed: {e}")
            return

        now = time.monotonic()
        with self._lock:
            first_frame = self._await_first_frame_since is not None
            self._last_frame_t = now
            self._await_first_frame_since = None
            if is_bad:
                if self._bad_since is None:
                    self._bad_since = now
                    self._record(f"green/uniform frame detected: {stats}", level="warn")
            else:
                if self._bad_since is not None:
                    self._record("colour stream recovered", level="info")
                self._bad_since = None
                if first_frame:
                    # A healthy frame after a (re)launch == recovery: clear the
                    # restart budget and leave slow-retry mode.
                    if self._in_slow_retry:
                        self._in_slow_retry = False
                        self._record("ZED recovered -- leaving slow-retry mode", level="info")
                    else:
                        self._record("ZED streaming healthy after (re)launch", level="info")
                    self._restart_times.clear()

    # ---- state machine ------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        with self._lock:
            # Reap a child that exited on its own (e.g. Camera::open gave up)
            # immediately, so it can never linger as a zombie -- but only REMEMBER
            # that it died; the cooldown below gates the actual restart, so a
            # fast-failing launch can't busy-loop restarting.
            if (not self._monitor_only and self._proc is not None
                    and self._proc.poll() is not None):
                if self._proc_dead_since is None:
                    self._proc_dead_since = now
                    self._proc_dead_rc = self._proc.returncode
                self._reap(self._proc)
                self._proc = None

            if now < self._cooldown_until:
                return

            reason = None
            if self._proc_dead_since is not None:
                reason = f"ZED process exited (code {self._proc_dead_rc})"
            elif self._bad_since is not None and (now - self._bad_since) >= self._fault_seconds:
                reason = f"uniform colour for {now - self._bad_since:.1f}s"
            elif self._last_frame_t is not None and (now - self._last_frame_t) >= self._stall_seconds:
                reason = f"no frames for {now - self._last_frame_t:.1f}s"
            elif (
                self._last_frame_t is None
                and self._await_first_frame_since is not None
                and (now - self._await_first_frame_since) >= self._init_timeout_seconds
            ):
                reason = (f"no first frame {now - self._await_first_frame_since:.0f}s "
                          f"after (re)launch")
            if reason is None:
                return
            self._trigger_restart(now, reason)

    def _trigger_restart(self, now, reason):
        """Called with self._lock held."""
        # Drop restarts that fell out of the rolling window.
        while self._restart_times and (now - self._restart_times[0]) > self._restart_window:
            self._restart_times.popleft()
        # Over the fast-restart budget? Then a plain relaunch is not fixing it.
        burst_tripped = len(self._restart_times) >= self._max_restarts
        self._restart_times.append(now)
        n = len(self._restart_times)

        if self._monitor_only:
            self._bad_since = None
            self._last_frame_t = now
            self._await_first_frame_since = None
            self._proc_dead_since = None
            self._cooldown_until = now + self._cooldown_seconds
            self._record(
                f"[monitor-only] would restart ZED ({reason}) "
                f"[{n} in {self._restart_window:.0f}s]", level="warn")
            return

        if burst_tripped:
            cooldown = self._slow_retry_seconds
            if not self._in_slow_retry:
                self._in_slow_retry = True
                self._record(
                    f"ZED fault persists ({reason}): {self._max_restarts} restarts in "
                    f"{self._restart_window:.0f}s did not recover it -- switching to slow "
                    f"retry every {self._slow_retry_seconds:.0f}s. Check the USB "
                    f"cable/power; the device may have fallen off the bus.", level="error")
            # On the persistent-fault path a node restart alone may be hopeless
            # (device off the bus); try to bring it back with a USB reset first.
            if self._usb_reset_enabled:
                self._usb_reset()
        else:
            cooldown = self._cooldown_seconds

        self._record(
            f"restarting ZED ({reason}) "
            f"[{n} in {self._restart_window:.0f}s, "
            f"{'slow' if burst_tripped else 'fast'}]", level="warn")
        self._stop_zed()
        self._start_zed()
        # Cooldown starts from when the fresh ZED began launching -- otherwise the
        # (blocking) stop time would be counted against the NEURAL_PLUS reinit and
        # a slow reinit could trip the budget. _start_zed already armed
        # _await_first_frame_since and cleared _last_frame_t.
        after = time.monotonic()
        self._bad_since = None
        self._proc_dead_since = None
        self._proc_dead_rc = None
        self._cooldown_until = after + cooldown

    # ---- USB recovery -------------------------------------------------------
    def _usb_reset(self):
        """Best-effort USB port reset of the ZED (vendor id self._usb_vendor_id).

        Issues USBDEVFS_RESET on the device node, which forces a re-enumeration
        and clears the 'device fell off the bus / Camera::open keeps failing'
        fault a process restart can't. Needs write access to
        /dev/bus/usb/<bus>/<dev> (a udev rule or root); logs and returns on
        failure so the relaunch still proceeds.
        """
        vid = self._usb_vendor_id.lower().removeprefix("0x").zfill(4)
        reset_any = False
        try:
            for dev_dir in glob.glob("/sys/bus/usb/devices/*"):
                vfile = os.path.join(dev_dir, "idVendor")
                try:
                    with open(vfile) as f:
                        if f.read().strip().lower() != vid:
                            continue
                    with open(os.path.join(dev_dir, "busnum")) as f:
                        busnum = int(f.read().strip())
                    with open(os.path.join(dev_dir, "devnum")) as f:
                        devnum = int(f.read().strip())
                except (OSError, ValueError):
                    continue  # not a device dir we can read; skip
                node = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
                try:
                    fd = os.open(node, os.O_WRONLY)
                    try:
                        fcntl.ioctl(fd, _USBDEVFS_RESET, 0)
                        reset_any = True
                        self._record(f"USB reset issued on {node} (vid {vid})", level="warn")
                    finally:
                        os.close(fd)
                except PermissionError:
                    self._record(
                        f"USB reset needs write access to {node} -- add a udev rule "
                        f"(SUBSYSTEM==\"usb\", ATTR{{idVendor}}==\"{vid}\", MODE=\"0666\") "
                        f"or run privileged. Skipping reset.", level="error")
                    return
                except OSError as e:
                    self._record(f"USB reset ioctl on {node} failed: {e}", level="warn")
            if not reset_any:
                self._record(
                    f"USB reset: no device with vendor id {vid} on the bus "
                    f"(already gone -- waiting for re-enumeration)", level="warn")
        except Exception as e:  # noqa: BLE001
            self._record(f"USB reset error: {e}", level="warn")

    # ---- child process management ------------------------------------------
    def _start_zed(self, initial=False):
        # Resolve `ros2` explicitly: PATH may not carry it in the launching
        # environment (the commander invokes ros2 by absolute path). Executing
        # the ros2 script directly uses its own (system-python) shebang.
        ros2_bin = shutil.which("ros2") or "/opt/ros/jazzy/bin/ros2"
        cmd = [ros2_bin, "launch", self._zed_pkg, self._zed_launch, *self._zed_args]
        self._record(("launching" if initial else "relaunching")
                     + " ZED: " + " ".join(cmd), level="info")
        # New session/process group so we can signal the whole launch tree.
        self._proc = subprocess.Popen(cmd, start_new_session=True)
        # Wait for the first real frame before re-arming the stall timer: a cold
        # relaunch (NEURAL_PLUS reload) can take tens of seconds to emit frame 1,
        # and we must not count that as a stall. _tick's init-timeout / process-
        # death paths cover a relaunch that never streams.
        self._last_frame_t = None
        self._await_first_frame_since = time.monotonic()

    @staticmethod
    def _reap(proc):
        """Reap an already-exited child so it doesn't linger as a zombie (the bug
        that left a 55-min defunct `ros2` on 2026-06-28)."""
        try:
            proc.wait(timeout=1.0)
        except Exception:  # noqa: BLE001
            pass

    def _stop_zed(self):
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        if proc.poll() is not None:
            self._reap(proc)
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)        # graceful (Ctrl-C equivalent)
            try:
                # Keep total stop time well under launch's SIGINT->SIGKILL
                # escalation (~10s): if launch kills US mid-reap at shutdown, the
                # child ZED would be orphaned and keep holding the camera.
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("ZED did not exit on SIGINT; SIGKILL")
                os.killpg(pgid, signal.SIGKILL)
                proc.wait(timeout=3.0)
        except ProcessLookupError:
            pass
        except Exception as e:
            self.get_logger().warn(f"error stopping ZED: {e}")


def main():
    rclpy.init()
    node = ZedWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Always reap the child ZED first -- it runs in its own session and does
        # NOT receive the bringup's Ctrl-C, so an orphan would keep the camera.
        node._stop_zed()
        if node._log_fh is not None:
            try:
                node._log_fh.close()
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()