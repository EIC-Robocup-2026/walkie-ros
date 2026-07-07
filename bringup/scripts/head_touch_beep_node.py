#!/usr/bin/python3
# NOTE: absolute interpreter on purpose (same reason as zed_watchdog.py):
# real_omnibot.launch.py runs under an environment where `python3` on PATH may
# be a venv without rclpy; the system 3.12 interpreter inherits ROS's
# PYTHONPATH and works.
"""
Head-touch R2D2 beeper.

Detects a person pressing/pushing the Dynamixel head and answers with a
randomly generated R2D2-style chirp phrase.

Detection
---------
The head servo (`head_servo_joint`) holds its position with a stiff position
loop, so a press shows up in `/joint_states` two ways:

  * **Position error** — present position deviates from the last commanded
    goal (subscribed from `/head_servo_controller/commands`).
  * **Load change** — `effort` (mapped from the Dynamixel `Present Load`
    register, raw counts of 0.1 % stall torque, roughly -1000..1000) jumps
    away from its quiescent baseline as the servo fights the push.

Either signal past its threshold, sustained for `trigger_duration` seconds,
counts as a touch. To avoid self-triggering on commanded motion, detection is
muted for `settle_time` after every goal change, and the load baseline is
re-zeroed when the mute expires (so a new pose's static load isn't mistaken
for a hand). After each beep a `cooldown` refractory period applies — holding
the head down keeps it chirping in protest every cooldown, which is the fun
part.

Each detected touch is also published on `~/touched` (std_msgs/Empty) so other
nodes can react.

Sound
-----
Chirps are synthesized on the fly with numpy (sine sweeps, warbles, trills —
3..6 random syllables between ~350 Hz and ~3.2 kHz) and piped as WAV to
`aplay` on the default PipeWire/ALSA sink (`audio_device` parameter). No sound
files, no extra dependencies.

Tuning
------
Run with `debug:=true` to get a `~/debug` topic and throttled log lines with
the live position-error and load-delta values, then set
`position_error_threshold` / `effort_delta_threshold` accordingly.
"""

import io
import math
import random
import shutil
import subprocess
import threading
import wave

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray


class HeadTouchBeepNode(Node):
    def __init__(self):
        super().__init__("head_touch_beep")

        self.declare_parameter("joint_name", "head_servo_joint")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("commands_topic", "/head_servo_controller/commands")

        # Detection thresholds
        self.declare_parameter("position_error_threshold", 0.06)  # rad (~3.4 deg)
        self.declare_parameter("effort_delta_threshold", 60.0)  # raw load counts (6 % stall)
        self.declare_parameter("trigger_duration", 0.08)  # s the signal must persist
        self.declare_parameter("settle_time", 1.5)  # s mute after a goal command
        self.declare_parameter("cooldown", 2.5)  # s refractory after a beep
        self.declare_parameter("debug", False)

        # Sound
        self.declare_parameter("audio_device", "default")
        self.declare_parameter("volume", 0.8)  # 0..1 output amplitude
        self.declare_parameter("sample_rate", 24000)
        self.declare_parameter("startup_beep", True)  # confirms the audio path

        self.joint_name = self.get_parameter("joint_name").value
        self.pos_err_thresh = self.get_parameter("position_error_threshold").value
        self.effort_thresh = self.get_parameter("effort_delta_threshold").value
        self.trigger_duration = self.get_parameter("trigger_duration").value
        self.settle_time = self.get_parameter("settle_time").value
        self.cooldown = self.get_parameter("cooldown").value
        self.debug = self.get_parameter("debug").value
        self.audio_device = self.get_parameter("audio_device").value
        self.volume = float(self.get_parameter("volume").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)

        # Detection state
        self.goal = None  # last commanded position (rad)
        self.mute_until = 0.0  # monotonic-ish node time, seconds
        self.rebaseline_on_unmute = False
        self.effort_baseline = None
        self.condition_since = None  # when the trigger condition became true
        self.cooldown_until = 0.0

        self._play_lock = threading.Lock()
        self._rng = random.Random()

        self.touched_pub = self.create_publisher(Empty, "~/touched", 10)
        self.debug_pub = (
            self.create_publisher(Float64MultiArray, "~/debug", 10) if self.debug else None
        )

        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.on_joint_states,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("commands_topic").value,
            self.on_command,
            10,
        )

        self.get_logger().info(
            f"watching '{self.joint_name}' "
            f"(pos_err > {self.pos_err_thresh:.3f} rad or "
            f"load delta > {self.effort_thresh:.0f} counts)"
        )

        if self.get_parameter("startup_beep").value:
            self.play_async(self.synth_phrase(n_syllables=2))

    # ------------------------------------------------------------------ time
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- detection
    def on_command(self, msg: Float64MultiArray):
        if not msg.data:
            return
        new_goal = float(msg.data[0])
        if self.goal is None or abs(new_goal - self.goal) > 1e-4:
            self.goal = new_goal
            self.mute_until = self.now() + self.settle_time
            self.rebaseline_on_unmute = True
            self.condition_since = None

    def on_joint_states(self, msg: JointState):
        try:
            i = msg.name.index(self.joint_name)
        except ValueError:
            return
        pos = msg.position[i] if i < len(msg.position) else None
        effort = msg.effort[i] if i < len(msg.effort) else None
        if pos is None:
            return

        t = self.now()

        # Before any command arrives, treat the current pose as the goal so a
        # stationary head can't false-trigger on startup.
        if self.goal is None:
            self.goal = pos

        muted = t < self.mute_until
        if muted:
            self.condition_since = None
            self.publish_debug(pos, effort, muted=True, triggered=False)
            return

        # Commanded motion just finished: adopt the new static load as baseline
        # so gravity/pose offsets don't read as a touch.
        if self.rebaseline_on_unmute:
            self.rebaseline_on_unmute = False
            self.effort_baseline = effort

        pos_err = abs(pos - self.goal)
        pressed = pos_err > self.pos_err_thresh

        effort_delta = 0.0
        if effort is not None:
            if self.effort_baseline is None:
                self.effort_baseline = effort
            effort_delta = abs(effort - self.effort_baseline)
            pressed = pressed or effort_delta > self.effort_thresh
            if not pressed:
                # Slow EMA tracks drift (temperature, friction) while quiet.
                self.effort_baseline += 0.02 * (effort - self.effort_baseline)

        triggered = False
        if pressed:
            if self.condition_since is None:
                self.condition_since = t
            if (
                t - self.condition_since >= self.trigger_duration
                and t >= self.cooldown_until
            ):
                triggered = True
                self.cooldown_until = t + self.cooldown
                self.condition_since = None
                self.get_logger().info(
                    f"head touch detected (pos_err={pos_err:.3f} rad, "
                    f"load_delta={effort_delta:.0f}) — beeping"
                )
                self.touched_pub.publish(Empty())
                self.play_async(self.synth_phrase())
        else:
            self.condition_since = None

        self.publish_debug(pos, effort, muted=False, triggered=triggered)

    def publish_debug(self, pos, effort, muted: bool, triggered: bool):
        if self.debug_pub is None:
            return
        pos_err = abs(pos - self.goal) if self.goal is not None else 0.0
        effort_delta = (
            abs(effort - self.effort_baseline)
            if effort is not None and self.effort_baseline is not None
            else 0.0
        )
        m = Float64MultiArray()
        m.data = [pos_err, effort_delta, float(muted), float(triggered)]
        self.debug_pub.publish(m)
        self.get_logger().info(
            f"pos_err={pos_err:.4f} load_delta={effort_delta:.1f} "
            f"muted={muted} triggered={triggered}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------- synthesis
    def _syllable(self, kind: str, rng: random.Random) -> np.ndarray:
        fs = self.sample_rate
        dur = rng.uniform(0.06, 0.22)
        n = max(1, int(dur * fs))
        t = np.arange(n) / fs
        tn = t / dur  # 0..1

        f0 = math.exp(rng.uniform(math.log(350.0), math.log(2200.0)))
        if kind == "chirp_up":
            f1 = min(f0 * rng.uniform(1.5, 3.0), 3200.0)
            f = f0 * (f1 / f0) ** tn
        elif kind == "chirp_down":
            f1 = max(f0 / rng.uniform(1.5, 3.0), 300.0)
            f = f0 * (f1 / f0) ** tn
        elif kind == "warble":
            rate = rng.uniform(15.0, 35.0)
            depth = rng.uniform(0.05, 0.25)
            f = f0 * (1.0 + depth * np.sin(2 * np.pi * rate * t))
        elif kind == "trill":
            f = f0 * (1.0 + 0.3 * tn)
        else:  # squeak: up-and-back-down bend
            fpeak = min(f0 * rng.uniform(1.8, 2.8), 3400.0)
            f = f0 + (fpeak - f0) * np.sin(np.pi * tn)

        phase = 2 * np.pi * np.cumsum(f) / fs
        x = np.sin(phase) + 0.35 * np.sin(2 * phase)  # classic synth timbre

        if kind == "trill":
            gate_rate = rng.uniform(20.0, 35.0)
            x = x * (np.sin(2 * np.pi * gate_rate * t) > 0)

        # Raised-cosine edges to avoid clicks
        edge = max(1, int(0.005 * fs))
        env = np.ones(n)
        ramp = 0.5 * (1 - np.cos(np.linspace(0, np.pi, min(edge, n))))
        env[: len(ramp)] *= ramp
        env[-len(ramp):] *= ramp[::-1]
        return (x * env * rng.uniform(0.7, 1.0)).astype(np.float32)

    def synth_phrase(self, n_syllables: int | None = None) -> bytes:
        """Generate a random R2D2-ish phrase and return it as WAV bytes."""
        rng = self._rng
        fs = self.sample_rate
        n = n_syllables if n_syllables is not None else rng.randint(3, 6)
        kinds = ["chirp_up", "chirp_down", "warble", "trill", "squeak"]
        parts = []
        for _ in range(n):
            parts.append(self._syllable(rng.choice(kinds), rng))
            parts.append(np.zeros(int(rng.uniform(0.01, 0.05) * fs), dtype=np.float32))
        audio = np.concatenate(parts)
        peak = np.max(np.abs(audio))
        if peak > 0:
            audio = audio / peak
        pcm = (audio * self.volume * 32767).astype(np.int16)

        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(fs)
            w.writeframes(pcm.tobytes())
        return buf.getvalue()

    # -------------------------------------------------------------- playback
    def play_async(self, wav_bytes: bytes):
        """Play WAV bytes in a background thread; skip if already playing."""
        if not self._play_lock.acquire(blocking=False):
            return
        threading.Thread(
            target=self._play, args=(wav_bytes,), daemon=True
        ).start()

    def _play(self, wav_bytes: bytes):
        try:
            if shutil.which("aplay"):
                cmd = ["aplay", "-q", "-D", self.audio_device]
            elif shutil.which("ffplay"):
                cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", "-"]
            else:
                self.get_logger().error(
                    "no audio player found (need aplay or ffplay)",
                    throttle_duration_sec=30.0,
                )
                return
            result = subprocess.run(
                cmd, input=wav_bytes, capture_output=True, timeout=15.0
            )
            if result.returncode != 0:
                self.get_logger().warning(
                    f"{cmd[0]} failed (rc={result.returncode}): "
                    f"{result.stderr.decode(errors='replace').strip()}",
                    throttle_duration_sec=10.0,
                )
        except Exception as e:  # noqa: BLE001 — never let audio kill detection
            self.get_logger().warning(f"beep playback failed: {e}")
        finally:
            self._play_lock.release()


def main():
    rclpy.init()
    node = HeadTouchBeepNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
