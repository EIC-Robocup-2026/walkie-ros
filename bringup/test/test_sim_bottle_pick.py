"""Signal, goal and cleanup handling of sim_bottle_pick.py, with fake ROS clients.

The fakes stand in for rclpy's action client, its futures and goal handles, and the
commander's parameter services, so no ROS node runs. A signal is a direct call of the
script's SignalGuard, which is what Python does in the main thread when one arrives.
"""
import importlib.util
from pathlib import Path
import signal
from types import SimpleNamespace

import pytest
from robot_bringup.sim_pick import (
    COMMANDER_DEFAULTS, EXIT_INTERRUPTED, EXIT_PASSED, EXIT_UNSAFE, FAILED, INTERRUPTED,
    PASSED, STATUS_CANCELED,
)

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "sim_bottle_pick.py"


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("sim_bottle_pick", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeFuture:
    """rclpy.task.Future as the script uses it, completed by the test."""

    def __init__(self, before_callback=None):
        # Runs when add_done_callback is called, before the callback is stored.
        self.before_callback = before_callback
        self.callbacks = []
        self.finished = False
        self.value = None
        self.error = None

    def done(self):
        return self.finished

    def result(self):
        if self.error is not None:
            raise self.error
        return self.value

    def exception(self):
        return self.error

    def add_done_callback(self, callback):
        if self.before_callback is not None:
            self.before_callback()
        if self.finished:
            callback(self)
        else:
            self.callbacks.append(callback)

    def set_result(self, value):
        self.value = value
        self.finish()

    def set_exception(self, error):
        self.error = error
        self.finish()

    def finish(self):
        self.finished = True
        for callback in self.callbacks:
            callback(self)


class FakeHandle:
    """ClientGoalHandle of an accepted goal; a cancel ends the goal CANCELED."""

    def __init__(self):
        self.accepted = True
        self.result_future = FakeFuture()
        self.cancel_requests = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_requests += 1
        answer = FakeFuture()
        answer.set_result(SimpleNamespace(return_code=0, goals_canceling=[object()]))
        if not self.result_future.done():
            self.result_future.set_result(SimpleNamespace(status=STATUS_CANCELED, result=None))
        return answer


class FakeActionClient:
    """ActionClient whose send_goal_async can run a hook after the goal went out."""

    def __init__(self, response, during_send=None):
        self.response = response
        self.during_send = during_send
        self.sent = []

    def send_goal_async(self, goal):
        self.sent.append(goal)
        if self.during_send is not None:
            self.during_send()
        return self.response


class FakeNode:
    """PickClient as the cleanup uses it: the goal tracker and the commander parameters."""

    def __init__(self, script, guard):
        self.tracker = script.GoalTracker(guard, lambda: 0)
        self.changed_commander = False
        self.parameters = dict(COMMANDER_DEFAULTS)
        self.writes = []
        # Runs on every write, before it takes effect.
        self.on_write = None
        self.refuse = False
        self.destroyed = False

    def set_commander(self, **values):
        self.changed_commander = True
        self.writes.append(values)
        if self.on_write is not None:
            self.on_write()
        if self.refuse:
            return False, "refused"
        self.parameters.update(values)
        return True, None

    def get_parameters(self, node, names):
        return {name: self.parameters[name] for name in names}

    def destroy_node(self):
        self.destroyed = True


def make_run(script, guard, node, pick):
    """Return a PickRun whose pick() calls pick(node) on the fake node."""

    class Run(script.PickRun):
        def pick(self):
            self.node = node
            return pick(node)

    return Run(script.parse_arguments([]), guard)


def sigint(guard):
    return lambda: guard(signal.SIGINT, None)


def test_guard_raises_at_once_when_armed(script):
    guard = script.SignalGuard()
    guard.arm()
    with pytest.raises(script.Interrupted):
        guard(signal.SIGTERM, None)
    assert guard.received == "SIGTERM"


def test_guard_defers_a_signal_to_the_end_of_the_outer_section(script):
    guard = script.SignalGuard()
    guard.arm()
    steps = []
    with pytest.raises(script.Interrupted):
        with guard.deferred():
            guard(signal.SIGINT, None)
            steps.append("signal recorded")
            with guard.deferred():
                pass
            steps.append("inner section left")
    assert steps == ["signal recorded", "inner section left"]
    assert guard.received == "SIGINT"


def test_guard_records_the_first_signal_while_disarmed_and_raises_it_once(script):
    guard = script.SignalGuard()
    guard(signal.SIGINT, None)
    guard(signal.SIGTERM, None)
    assert guard.received == "SIGINT"
    assert guard.ignored == ["SIGTERM"]
    with pytest.raises(script.Interrupted):
        guard.arm()
    guard.disarm()
    guard.arm()
    with guard.deferred():
        pass


def test_a_pick_without_signals_passes(script):
    guard = script.SignalGuard()
    node = FakeNode(script, guard)

    def pick(node):
        node.set_commander(allow_gripper_vs_octomap=True)
        return PASSED

    run = make_run(script, guard, node, pick)
    assert run.execute() == EXIT_PASSED
    assert run.report["passed"] is True
    assert node.parameters == COMMANDER_DEFAULTS
    assert run.report["cleanup"]["verified"] is True
    assert node.destroyed


def test_signal_inside_send_goal_async_keeps_the_goal_and_cancels_it(script):
    guard = script.SignalGuard()
    handle = FakeHandle()
    response = FakeFuture()
    response.set_result(handle)
    # The goal has gone out when the signal arrives, before send_goal_async returns.
    client = FakeActionClient(response, during_send=sigint(guard))
    node = FakeNode(script, guard)
    reached = []

    def pick(node):
        node.set_commander(allow_gripper_vs_octomap=True)
        node.tracker.run_goal(client, "goal", "approach", 600.0)
        reached.append("after the goal")
        return PASSED

    run = make_run(script, guard, node, pick)
    code = run.execute()

    assert code == EXIT_INTERRUPTED
    assert reached == []
    assert len(client.sent) == 1
    [record] = node.tracker.goals
    assert record.send_future is response
    assert handle.cancel_requests == 1
    assert run.report["cleanup"]["goals"] == [
        {"goal": "approach", "cancel": "cancel accepted", "resolved": True, "end": "CANCELED"}]
    assert node.parameters == COMMANDER_DEFAULTS
    assert run.report["result"] == INTERRUPTED
    assert run.report["signal_during"] == "pick"


def test_signal_before_the_response_callback_still_cancels_a_late_acceptance(
        script, monkeypatch):
    monkeypatch.setattr(script, "CANCEL_RESPONSE_TIMEOUT", 0.05)
    guard = script.SignalGuard()
    # The signal arrives after the future is recorded, before its callback is stored.
    response = FakeFuture(before_callback=sigint(guard))
    client = FakeActionClient(response)
    node = FakeNode(script, guard)

    def pick(node):
        node.tracker.run_goal(client, "goal", "close", 120.0)
        return PASSED

    run = make_run(script, guard, node, pick)
    code = run.execute()

    # The server has not answered the goal request, so its end is unknown.
    assert code == EXIT_UNSAFE
    [record] = node.tracker.goals
    assert record.send_future is response
    assert len(response.callbacks) == 1
    assert run.report["cleanup"]["goals"] == [
        {"goal": "close", "resolved": False, "end": "unknown: no goal response"}]
    # The acceptance arrives after the cleanup gave up waiting; the callback cancels it.
    handle = FakeHandle()
    response.set_result(handle)
    assert handle.cancel_requests == 1


def test_a_failed_result_request_leaves_the_end_unknown_and_restores(script):
    guard = script.SignalGuard()
    handle = FakeHandle()
    handle.result_future.set_exception(RuntimeError("result request lost"))
    response = FakeFuture()
    response.set_result(handle)
    client = FakeActionClient(response)
    node = FakeNode(script, guard)

    def pick(node):
        node.set_commander(allow_gripper_vs_octomap=True)
        node.tracker.run_goal(client, "goal", "approach", 600.0)
        return PASSED

    run = make_run(script, guard, node, pick)
    code = run.execute()

    assert code == EXIT_UNSAFE
    assert run.report["result"] == FAILED
    assert run.report["error"] == ("approach: the result request failed "
                                   "(RuntimeError: result request lost)")
    [record] = node.tracker.goals
    assert not record.ended()
    assert handle.cancel_requests == 1
    [entry] = run.report["cleanup"]["goals"]
    assert entry["resolved"] is False
    assert entry["end"].startswith("unknown: the result request failed")
    # The defaults are written back also when a goal's end is unknown.
    assert node.parameters == COMMANDER_DEFAULTS
    assert run.report["cleanup"]["verified"] is True


@pytest.mark.parametrize("refuse, expected", [(False, EXIT_INTERRUPTED), (True, EXIT_UNSAFE)])
def test_signal_during_the_cleanup_of_a_pass_is_not_lost(script, refuse, expected):
    guard = script.SignalGuard()
    node = FakeNode(script, guard)

    def pick(node):
        node.set_commander(allow_gripper_vs_octomap=True)
        # The next write is the cleanup's restore, and a SIGINT arrives during it.
        node.on_write = sigint(guard)
        node.refuse = refuse
        return PASSED

    run = make_run(script, guard, node, pick)
    code = run.execute()

    assert code == expected
    assert node.writes[-1] == COMMANDER_DEFAULTS
    assert run.report["cleanup"]["restored"] is not refuse
    assert run.report["result"] == INTERRUPTED
    assert run.report["result_before_signal"] == PASSED
    assert run.report["signal"] == "SIGINT"
    assert run.report["signal_during"] == "cleanup"
    assert run.report["passed"] is False
    assert node.destroyed


def test_a_second_signal_during_the_cleanup_is_listed_and_ignored(script):
    guard = script.SignalGuard()
    node = FakeNode(script, guard)

    def pick(node):
        node.set_commander(allow_gripper_vs_octomap=True)
        node.on_write = lambda: guard(signal.SIGTERM, None)
        guard(signal.SIGINT, None)
        return PASSED

    run = make_run(script, guard, node, pick)
    assert run.execute() == EXIT_INTERRUPTED
    assert run.report["signal"] == "SIGINT"
    assert run.report["signals_ignored"] == ["SIGTERM"]
    assert "result_before_signal" not in run.report
    assert node.parameters == COMMANDER_DEFAULTS


def test_a_signal_before_the_run_stops_it_before_the_pick(script):
    guard = script.SignalGuard()
    node = FakeNode(script, guard)
    guard(signal.SIGTERM, None)

    def pick(node):
        raise AssertionError("the pick ran after a signal")

    run = make_run(script, guard, node, pick)
    assert run.execute() == EXIT_INTERRUPTED
    assert run.report["error"] == "interrupted by SIGTERM"
    assert run.report["signal_during"] == "pick"
    assert run.report["cleanup"]["restored"] is None
