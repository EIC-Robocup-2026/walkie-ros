#!/usr/bin/env python3
"""Benchmark node for the bimanual commander — service throughput and action stress test."""

import argparse
import math
import statistics
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from my_robot_interfaces.action import SetJointPosition
from my_robot_interfaces.srv import GetJointStates


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def fmt_ms(seconds):
    return f"{seconds * 1000:.2f}ms"


def percentile(data, p):
    if not data:
        return 0.0
    s = sorted(data)
    idx = (len(s) - 1) * p / 100
    lo, hi = int(idx), min(int(idx) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (idx - lo)


def sine_positions(home, t, amplitude, freq):
    return [
        h + amplitude * math.sin(2 * math.pi * freq * t + i * math.pi / 4)
        for i, h in enumerate(home)
    ]


# ─────────────────────────────────────────────────────────────────────────────
# Benchmark node
# ─────────────────────────────────────────────────────────────────────────────

class BenchmarkNode(Node):
    def __init__(self, group: str):
        super().__init__("commander_benchmark")
        self.group = group
        self._jnt_cli  = self.create_client(GetJointStates, "get_joint_states")
        self._joint_ac = ActionClient(self, SetJointPosition, "set_joint_position")

    # ── Event-based service call (no spin_until_future_complete) ─────────────

    def _call_service(self, timeout=2.0):
        """Call get_joint_states using threading.Event — safe while background executor spins."""
        event = threading.Event()
        result_holder = [None]

        req = GetJointStates.Request()
        req.group_name = self.group
        future = self._jnt_cli.call_async(req)

        def cb(f):
            result_holder[0] = f.result()
            event.set()

        future.add_done_callback(cb)
        event.wait(timeout=timeout)
        r = result_holder[0]
        return r if (r is not None and r.success) else None

    # ── Event-based action send (no spin_until_future_complete) ─────────────

    def _send_joint_goal_sync(self, positions, exec_timeout=15.0):
        """Send a goal and block until complete. Returns (success, elapsed_s)."""
        goal = SetJointPosition.Goal()
        goal.group_name      = self.group
        goal.joint_positions = positions

        accepted_event = threading.Event()
        result_event   = threading.Event()
        gh_holder      = [None]
        result_holder  = [False]

        t0 = time.perf_counter()

        def on_goal(f):
            gh = f.result()
            gh_holder[0] = gh
            if gh and gh.accepted:
                rf = gh.get_result_async()
                rf.add_done_callback(on_result)
            accepted_event.set()

        def on_result(f):
            try:
                result_holder[0] = f.result().result.success
            except Exception:
                result_holder[0] = False
            result_event.set()

        f = self._joint_ac.send_goal_async(goal)
        f.add_done_callback(on_goal)

        accepted_event.wait(timeout=5.0)
        gh = gh_holder[0]
        if gh is None or not gh.accepted:
            return False, time.perf_counter() - t0

        result_event.wait(timeout=exec_timeout)
        return result_holder[0], time.perf_counter() - t0


# ─────────────────────────────────────────────────────────────────────────────
# Benchmark 1 — get_joint_states throughput
# ─────────────────────────────────────────────────────────────────────────────

def bench_get_joint_states(node: BenchmarkNode, n: int):
    print(f"\n{'='*60}")
    print(f"  Benchmark: get_joint_states")
    print(f"  Group: {node.group}  |  N={n} calls")
    print(f"{'='*60}")

    if not node._jnt_cli.wait_for_service(timeout_sec=3.0):
        print("  [ERROR] Service not available"); return

    latencies = []
    failed = 0
    for i in range(n):
        t0 = time.perf_counter()
        r = node._call_service()
        elapsed = time.perf_counter() - t0
        if r is None:
            failed += 1
        else:
            latencies.append(elapsed)

    if not latencies:
        print("  [ERROR] All calls failed"); return

    total = sum(latencies) + failed * 2.0   # failed calls counted at timeout
    rate  = len(latencies) / sum(latencies)

    print(f"  Successful : {len(latencies)} / {n}  (failed={failed})")
    print(f"  Total time : {sum(latencies):.3f} s  (successful calls only)")
    print(f"  Rate       : {rate:.1f} Hz")
    print(f"  Latency    : mean={fmt_ms(statistics.mean(latencies))}"
          f"  min={fmt_ms(min(latencies))}"
          f"  max={fmt_ms(max(latencies))}"
          f"  p95={fmt_ms(percentile(latencies, 95))}"
          f"  p99={fmt_ms(percentile(latencies, 99))}")


# ─────────────────────────────────────────────────────────────────────────────
# Benchmark 2 — set_joint_position sequential baseline
# ─────────────────────────────────────────────────────────────────────────────

def bench_sequential(node: BenchmarkNode, n: int, amplitude: float, freq: float):
    print(f"\n{'='*60}")
    print(f"  Benchmark: set_joint_position (sequential)")
    print(f"  Group: {node.group}  |  N={n}  amplitude={amplitude}rad  freq={freq}Hz")
    print(f"{'='*60}")

    if not node._joint_ac.wait_for_server(timeout_sec=5.0):
        print("  [ERROR] Action server not available"); return

    base = node._call_service()
    if base is None:
        print("  [ERROR] Could not read home joint positions"); return
    home = list(base.joint_positions)
    print(f"  Home ({len(home)} joints): {[f'{v:.3f}' for v in home]}")

    elapsed_times = []
    start = time.perf_counter()
    for i in range(n):
        t = time.perf_counter() - start
        positions = sine_positions(home, t, amplitude, freq)
        success, elapsed = node._send_joint_goal_sync(positions, exec_timeout=15.0)
        elapsed_times.append(elapsed)
        print(f"  [{i+1:02d}/{n}] {'OK' if success else 'FAIL'}  {fmt_ms(elapsed)}"
              f"  pos: {[f'{v:.3f}' for v in positions]}")

    if elapsed_times:
        mean_t = statistics.mean(elapsed_times)
        print(f"\n  Per-call : mean={fmt_ms(mean_t)}"
              f"  min={fmt_ms(min(elapsed_times))}"
              f"  max={fmt_ms(max(elapsed_times))}")
        print(f"  Rate     : ~{1.0/mean_t:.2f} Hz  (planning + execution bottleneck)")


# ─────────────────────────────────────────────────────────────────────────────
# Benchmark 3 — stress test (preemptive + fire-and-forget)
# ─────────────────────────────────────────────────────────────────────────────

class StressStats:
    def __init__(self):
        self.lock           = threading.Lock()
        self.sent           = 0
        self.accepted       = 0
        self.rejected       = 0       # goal explicitly rejected by server
        self.mw_dropped     = 0       # future never resolved (Zenoh queue overflow)
        self.completed      = 0
        self.cancelled      = 0
        self.cancel_times   = []


def _run_stress(node: BenchmarkNode, home, amplitude, freq,
                duration, target_hz, preemptive: bool) -> StressStats:
    stats    = StressStats()
    interval = 1.0 / target_hz
    ac       = node._joint_ac
    current_gh = [None]
    pending_futures = []   # track all unresolved goal futures
    start = time.perf_counter()

    def on_goal_response(future):
        try:
            gh = future.result()
        except Exception:
            with stats.lock:
                stats.mw_dropped += 1
            return

        if gh is None:
            with stats.lock:
                stats.mw_dropped += 1
            return

        with stats.lock:
            if gh.accepted:
                stats.accepted += 1
                current_gh[0] = gh
            else:
                stats.rejected += 1
                return

        def on_result(rf):
            try:
                code = rf.result().status
                with stats.lock:
                    if code == 4:   # SUCCEEDED
                        stats.completed += 1
            except Exception:
                pass

        gh.get_result_async().add_done_callback(on_result)

    next_tick = time.perf_counter()
    while time.perf_counter() - start < duration:
        t = time.perf_counter() - start
        positions = sine_positions(home, t, amplitude, freq)

        if preemptive:
            with stats.lock:
                gh = current_gh[0]
            if gh is not None:
                cancel_t0 = time.perf_counter()
                cf = gh.cancel_goal_async()

                def on_cancel(f, t0=cancel_t0):
                    with stats.lock:
                        stats.cancelled += 1
                        stats.cancel_times.append(time.perf_counter() - t0)

                cf.add_done_callback(on_cancel)
                with stats.lock:
                    current_gh[0] = None

        goal = SetJointPosition.Goal()
        goal.group_name      = node.group
        goal.joint_positions = positions

        try:
            f = ac.send_goal_async(goal)
            f.add_done_callback(lambda fut: on_goal_response(fut))
            pending_futures.append(f)
            with stats.lock:
                stats.sent += 1
        except Exception:
            with stats.lock:
                stats.mw_dropped += 1

        next_tick += interval
        sleep_t = next_tick - time.perf_counter()
        if sleep_t > 0:
            time.sleep(sleep_t)

    # wait a moment for in-flight callbacks to settle
    time.sleep(1.0)

    # count futures that never resolved as middleware drops
    unresolved = sum(1 for f in pending_futures if not f.done())
    with stats.lock:
        stats.mw_dropped += unresolved

    return stats


def bench_stress(node: BenchmarkNode, amplitude: float, freq: float,
                 duration: float, target_hz: float):
    print(f"\n{'='*60}")
    print(f"  Benchmark: set_joint_position stress test")
    print(f"  Group: {node.group}  |  {target_hz}Hz  |  {duration}s"
          f"  |  amplitude={amplitude}rad")
    print(f"{'='*60}")

    if not node._joint_ac.wait_for_server(timeout_sec=5.0):
        print("  [ERROR] Action server not available"); return

    base = node._call_service()
    if base is None:
        print("  [ERROR] Could not read home joint positions"); return
    home = list(base.joint_positions)

    for preemptive, label in [(True, "preemptive"), (False, "fire-and-forget")]:
        print(f"\n  --- Mode: {label} ---")
        s = _run_stress(node, home, amplitude, freq, duration, target_hz, preemptive)

        sent = s.sent
        def pct(v):
            return f"({100.0*v/sent:.1f}%)" if sent else ""

        print(f"  Goals sent           : {sent}")
        print(f"  Goals accepted       : {s.accepted}  {pct(s.accepted)}")
        print(f"  Goals rejected       : {s.rejected}  {pct(s.rejected)}")
        print(f"  Goals mw-dropped     : {s.mw_dropped}  {pct(s.mw_dropped)}  ← Zenoh queue overflow")
        if preemptive:
            print(f"  Goals cancelled      : {s.cancelled}")
            if s.cancel_times:
                print(f"  Cancel latency       : mean={fmt_ms(statistics.mean(s.cancel_times))}"
                      f"  max={fmt_ms(max(s.cancel_times))}")
        exec_rate = s.completed / duration
        print(f"  Goals completed      : {s.completed}  {pct(s.completed)}"
              f"  → effective rate ~{exec_rate:.2f} Hz")


# ─────────────────────────────────────────────────────────────────────────────
# Benchmark 4 — JointTrajectoryController topic streaming
# ─────────────────────────────────────────────────────────────────────────────

RIGHT_ARM_JOINTS = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]

JTC_TOPIC = "/right_joint_trajectory_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"


def bench_jtc_streaming(node: BenchmarkNode, amplitude: float, freq: float,
                         duration: float, target_hz: float):
    print(f"\n{'='*60}")
    print(f"  Benchmark: JTC topic streaming")
    print(f"  Topic: {JTC_TOPIC}")
    print(f"  Joints: {len(RIGHT_ARM_JOINTS)}  |  {target_hz}Hz  |  {duration}s"
          f"  |  amplitude={amplitude}rad  freq={freq}Hz")
    print(f"{'='*60}")

    pub = node.create_publisher(JointTrajectory, JTC_TOPIC, 10)

    # Track latest joint states from /joint_states
    latest_state: dict[str, float] = {}
    state_lock = threading.Lock()

    def on_joint_states(msg: JointState):
        with state_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in RIGHT_ARM_JOINTS:
                    latest_state[name] = pos

    sub = node.create_subscription(JointState, JOINT_STATES_TOPIC, on_joint_states, 10)

    # Get home positions via get_joint_states service
    base = node._call_service()
    if base is None:
        # Fall back to zero home if service unavailable
        home = [0.0] * len(RIGHT_ARM_JOINTS)
        print("  [warn] Could not read home positions, using zeros")
    else:
        # Map returned positions by name
        name_to_pos = dict(zip(base.joint_names, base.joint_positions))
        home = [name_to_pos.get(j, 0.0) for j in RIGHT_ARM_JOINTS]
    print(f"  Home: {[f'{v:.3f}' for v in home]}")

    interval = 1.0 / target_hz
    publish_times = []
    tracking_errors: list[list[float]] = []  # per publish: [abs_err_j1..j7]

    print(f"  Publishing for {duration}s ...")
    start = time.perf_counter()
    next_tick = start

    while time.perf_counter() - start < duration:
        t = time.perf_counter() - start
        cmd_positions = sine_positions(home, t, amplitude, freq)

        msg = JointTrajectory()
        msg.joint_names = RIGHT_ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = cmd_positions
        # Time from start: one control cycle ahead so controller doesn't drop it
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(interval * 2 * 1e9)
        msg.points = [pt]

        t_pub = time.perf_counter()
        pub.publish(msg)
        publish_times.append(t_pub)

        # Capture tracking error for this tick
        with state_lock:
            if len(latest_state) == len(RIGHT_ARM_JOINTS):
                errs = [abs(latest_state[j] - cmd_positions[i])
                        for i, j in enumerate(RIGHT_ARM_JOINTS)]
                tracking_errors.append(errs)

        next_tick += interval
        sleep_t = next_tick - time.perf_counter()
        if sleep_t > 0:
            time.sleep(sleep_t)

    # settle briefly so last state message arrives
    time.sleep(0.5)

    # ── stats ────────────────────────────────────────────────────────────────
    n_pub = len(publish_times)
    if n_pub < 2:
        print("  [ERROR] Not enough publishes"); return

    # Actual publish rate from inter-publish intervals
    intervals_s = [publish_times[i+1] - publish_times[i] for i in range(n_pub - 1)]
    actual_hz = 1.0 / statistics.mean(intervals_s)

    print(f"\n  Published        : {n_pub} messages over {duration:.1f}s")
    print(f"  Actual rate      : {actual_hz:.1f} Hz  (target={target_hz}Hz)")
    print(f"  Interval jitter  : mean={fmt_ms(statistics.mean(intervals_s))}"
          f"  max={fmt_ms(max(intervals_s))}"
          f"  p95={fmt_ms(percentile(intervals_s, 95))}")

    if tracking_errors:
        n_err = len(tracking_errors)
        per_joint_mean = [
            statistics.mean(tracking_errors[k][j] for k in range(n_err))
            for j in range(len(RIGHT_ARM_JOINTS))
        ]
        overall_mean = statistics.mean(e for row in tracking_errors for e in row)
        overall_max  = max(e for row in tracking_errors for e in row)
        print(f"\n  Tracking error   : {n_err} samples")
        print(f"  Overall          : mean={math.degrees(overall_mean):.3f}°"
              f"  max={math.degrees(overall_max):.3f}°")
        print("  Per-joint mean (°):")
        for j, (name, err) in enumerate(zip(RIGHT_ARM_JOINTS, per_joint_mean)):
            bar = "█" * int(math.degrees(err) * 10)
            print(f"    {name:<30} {math.degrees(err):6.3f}°  {bar}")
    else:
        print("\n  [warn] No joint state feedback received — is joint_state_broadcaster running?")

    node.destroy_subscription(sub)
    node.destroy_publisher(pub)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Bimanual commander benchmark")
    parser.add_argument("--group",     default="right_arm_lift")
    parser.add_argument("--benchmark", default="all",
                        choices=["all", "get_joint_states", "sequential", "stress", "jtc"])
    parser.add_argument("--n",         type=int,   default=1000,
                        help="N calls for get_joint_states benchmark")
    parser.add_argument("--n-seq",     type=int,   default=10,
                        help="N calls for sequential benchmark")
    parser.add_argument("--duration",  type=float, default=5.0)
    parser.add_argument("--hz",        type=float, default=100.0)
    parser.add_argument("--amplitude", type=float, default=0.05,
                        help="Sine amplitude in radians (0.05 ≈ 3°)")
    parser.add_argument("--freq",      type=float, default=0.5,
                        help="Sine trajectory frequency in Hz")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = BenchmarkNode(args.group)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"\n{'#'*60}")
    print(f"  Commander Benchmark")
    print(f"  Group: {args.group}  |  Mode: {args.benchmark}")
    print(f"{'#'*60}")

    bm = args.benchmark
    try:
        if bm in ("all", "get_joint_states"):
            bench_get_joint_states(node, args.n)

        if bm in ("all", "sequential"):
            bench_sequential(node, args.n_seq, args.amplitude, args.freq)

        if bm in ("all", "stress"):
            bench_stress(node, args.amplitude, args.freq, args.duration, args.hz)

        if bm in ("all", "jtc"):
            bench_jtc_streaming(node, args.amplitude, args.freq, args.duration, args.hz)
    except KeyboardInterrupt:
        print("\n  [interrupted]")

    print(f"\n{'#'*60}")
    print("  Done.")
    print(f"{'#'*60}\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
