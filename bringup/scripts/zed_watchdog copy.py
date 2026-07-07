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
The head ZED 2i occasionally goes **solid green** and stays green until the node
is restarted. Root cause (confirmed on-robot): the camera's USB link glitches
(`uvcvideo: Non-zero status (-71)` / a USB disconnect + re-enumeration). The
already-running zed_node keeps its now-stale device handle, so `grab()` keeps
returning frames at ~9 Hz -- but they are a zeroed YUV buffer that converts to a
uniform BGRA (0,134,0,255) green, and depth comes back ~entirely NaN.

The ZED SDK's own recovery (`camera_timeout_sec` / `camera_max_reconnect`) never
fires, because from its point of view frames are still arriving -- they're just
corrupt. So detection has to be **content-based**, and recovery is a full restart
of the ZED node (verified sufficient: the device re-enumerates cleanly, only the
handle was dead -- no USB replug needed).

What it does
------------
This node OWNS the ZED process. It launches the stock
`ros2 launch zed_wrapper zed_camera.launch.py ...` as a child process group,
subscribes to the colour stream, and:
  * if frames are a uniform/near-constant colour for `fault_seconds`, or
  * if no frame arrives for `stall_seconds`,
it kills the child group and relaunches it, then ignores detection for
`cooldown_seconds` while the ZED re-initialises (NEURAL_PLUS depth model reload
is slow). A restart-rate limiter stops it thrashing a genuinely dead camera.

Set `monitor_only:=true` to detect-and-log without owning/killing the process
(safe for testing against an already-running ZED).
"""

import os
import shutil
import signal
import subprocess
import threading
import time
from collections import deque

import numpy as np
import rclpy
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
        self._sample_px = int(p("sample_px", 2000).value)
        self._std_eps = float(p("std_eps", 1.0).value)
        self._monitor_only = bool(p("monitor_only", False).value)
        # Restart-rate limiter: stop auto-restarting if it happens too often.
        self._max_restarts = int(p("max_restarts", 5).value)
        self._restart_window = float(p("restart_window_seconds", 300.0).value)
        # ZED launch command pieces (so args stay configurable from the launch file).
        self._zed_pkg = p("zed_launch_package", "zed_wrapper").value
        self._zed_launch = p("zed_launch_file", "zed_camera.launch.py").value
        self._zed_args = list(p("zed_launch_arguments", [""]).value) or []
        self._zed_args = [a for a in self._zed_args if a]

        # --- State ---
        self._lock = threading.Lock()
        self._last_frame_t = None      # monotonic time of last received frame
        self._bad_since = None         # monotonic time the current bad run began
        self._cooldown_until = 0.0     # monotonic time until which detection is muted
        self._restart_times = deque()  # monotonic times of recent restarts
        self._giving_up = False        # tripped the rate limiter
        self._proc = None              # Popen of the child ZED launch

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self._image_topic, self._on_image, qos)
        # Evaluate the state machine at a steady 2 Hz, independent of frame flow,
        # so the stall path works even when zero frames arrive.
        self.create_timer(0.5, self._tick)

        mode = "MONITOR-ONLY" if self._monitor_only else "SUPERVISE"
        self.get_logger().info(
            f"zed_watchdog [{mode}] topic={self._image_topic} "
            f"fault={self._fault_seconds}s stall={self._stall_seconds}s "
            f"cooldown={self._cooldown_seconds}s")
        self.get_logger().info(
            f"ZED launch: {self._zed_pkg}/{self._zed_launch} args={self._zed_args}")

        if not self._monitor_only:
            self._start_zed(initial=True)

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
            self._last_frame_t = now
            if is_bad:
                if self._bad_since is None:
                    self._bad_since = now
                    self.get_logger().warn(f"green/uniform frame detected: {stats}")
            else:
                if self._bad_since is not None:
                    self.get_logger().info("colour stream recovered")
                self._bad_since = None

    # ---- state machine ------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        with self._lock:
            if self._giving_up or now < self._cooldown_until:
                return
            reason = None
            if self._bad_since is not None and (now - self._bad_since) >= self._fault_seconds:
                reason = f"uniform colour for {now - self._bad_since:.1f}s"
            elif self._last_frame_t is not None and (now - self._last_frame_t) >= self._stall_seconds:
                reason = f"no frames for {now - self._last_frame_t:.1f}s"
            if reason is None:
                return
            self._trigger_restart(now, reason)

    def _trigger_restart(self, now, reason):
        """Called with self._lock held."""
        # Rate limit.
        while self._restart_times and (now - self._restart_times[0]) > self._restart_window:
            self._restart_times.popleft()
        if len(self._restart_times) >= self._max_restarts:
            self._giving_up = True
            self.get_logger().error(
                f"ZED fault again ({reason}) but {len(self._restart_times)} restarts "
                f"in {self._restart_window:.0f}s -- giving up auto-restart. "
                f"Check the USB cable/power; manual bringup needed.")
            return
        self._restart_times.append(now)
        n = len(self._restart_times)
        if self._monitor_only:
            self._bad_since = None
            self._last_frame_t = now
            self._cooldown_until = now + self._cooldown_seconds
            self.get_logger().warn(
                f"[monitor-only] would restart ZED ({reason}) "
                f"[{n}/{self._max_restarts} in window]")
            return
        self.get_logger().warn(
            f"restarting ZED ({reason}) [{n}/{self._max_restarts} in window]")
        self._stop_zed()
        self._start_zed()
        # Start the cooldown from when the fresh ZED began launching -- otherwise
        # the (blocking) stop time would be counted against the NEURAL_PLUS depth
        # model reinit, and a slow reinit could trip the rate limiter.
        after = time.monotonic()
        self._bad_since = None
        self._last_frame_t = after
        self._cooldown_until = after + self._cooldown_seconds

    # ---- child process management ------------------------------------------
    def _start_zed(self, initial=False):
        # Resolve `ros2` explicitly: PATH may not carry it in the launching
        # environment (the commander invokes ros2 by absolute path). Executing
        # the ros2 script directly uses its own (system-python) shebang.
        ros2_bin = shutil.which("ros2") or "/opt/ros/jazzy/bin/ros2"
        cmd = [ros2_bin, "launch", self._zed_pkg, self._zed_launch, *self._zed_args]
        self.get_logger().info(("launching" if initial else "relaunching")
                               + " ZED: " + " ".join(cmd))
        # New session/process group so we can signal the whole launch tree.
        self._proc = subprocess.Popen(cmd, start_new_session=True)

    def _stop_zed(self):
        proc = self._proc
        self._proc = None
        if proc is None or proc.poll() is not None:
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
