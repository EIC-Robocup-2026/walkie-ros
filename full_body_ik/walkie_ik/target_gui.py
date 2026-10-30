"""
Target GUI, copied from walker_robot/sim/target_gui.py (Coordinated-Base-Arm-Motion-Control).
Changed: walkie slider ranges, the reset button syncs the preview to the robot, and a
"Robot" box executes the preview's solution (MoveIt + Nav2) and shows its status.
"""
import dataclasses
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np

# default x / y slider range in metres, for the targets and the manual base; with a world,
# run_full_body_ik.py passes the extent of its map instead (TargetGUI's xy_range)
DEFAULT_XY_RANGE = ((-15.0, 15.0), (-15.0, 15.0))
# (name, unit, slider min, slider max, slider resolution); in the selected target frame.
# x / y ranges are replaced by TargetGUI's xy_range.
FIELDS = [("x", "m", -15.0, 15.0, 0.005), ("y", "m", -15.0, 15.0, 0.005), ("z", "m", 0.0, 2.5, 0.005),
          ("roll", "deg", -180.0, 180.0, 1.0), ("pitch", "deg", -180.0, 180.0, 1.0),
          ("yaw", "deg", -180.0, 180.0, 1.0)]
SLIDER_LENGTH = 280     # pixels
# base group (as in WalkieIK.step) -> checkbox label
BASE_TOGGLES = [("xy", "Base movement (x, y)"), ("yaw", "Orientation (yaw)"), ("lift", "Lift")]
# manual base sliders: (joint, base group, unit, slider min, slider max, slider resolution), world frame.
# x, y: replaced by TargetGUI's xy_range (the base itself has no limits); lift: lift_joint's range in the MJCF.
BASE_FIELDS = [("x", "xy", "m", -15.0, 15.0, 0.005), ("y", "xy", "m", -15.0, 15.0, 0.005),
               ("yaw", "yaw", "deg", -180.0, 180.0, 1.0), ("lift", "lift", "m", 0.0, 0.7435, 0.005)]
# the IK may not move these base groups while the targets are relative to the base: a target that
# moves with the base cannot be reached by moving the base
RELATIVE_LOCKED_GROUPS = ("xy", "yaw")
# target frame (as in reference_frame.FRAMES) -> radio button label
FRAME_CHOICES = [("world", "World (absolute)"), ("base", "Robot base (relative, x/y/yaw)")]


class TargetGUI:
    """
    Pop-up window with sliders for the end-effector targets (xyz + roll/pitch/yaw, ZYX) for each arm.
    Runs in the main thread (run()); the control thread talks to it only through
    get_request(), set_status(), take_reset(), set_targets(), take_frame_request(),
    switch_frame(), set_base_values(), show_joint_frames(), drag_targets_enabled(),
    get_target_values(), take_execute(), take_cancel(),
    set_robot_status(), close() and is_closed(), which are thread-safe.

    Targets are given in the selected frame ("world" or "base"). The exact values live in
    self._values, not in the sliders (which clamp to their range), so a frame switch can
    re-express them without any rounding.
    """

    def __init__(self, initial_targets: dict, frame: str = "world", xy_range=DEFAULT_XY_RANGE):
        """
        initial_targets: {side: Target} in `frame`.
        xy_range: ((x min, x max), (y min, y max)) in metres of the x / y sliders, targets and manual base.
        """
        xy = dict(zip("xy", xy_range))
        self._fields = [(n, u, *xy[n], r) if n in xy else (n, u, lo, hi, r) for n, u, lo, hi, r in FIELDS]
        self._base_fields = [(j, g, u, *xy[j], r) if j in xy else (j, g, u, lo, hi, r)
                             for j, g, u, lo, hi, r in BASE_FIELDS]
        self._lock = threading.Lock()
        self._template = dict(initial_targets)        # used to build new Target objects
        # side -> [x, y, z, roll, pitch, yaw] (m, rad): what the IK tracks, in self._frame
        self._values = {side: np.concatenate([t.pos, t.rpy if t.rpy is not None else np.zeros(3)]).astype(float)
                        for side, t in initial_targets.items()}
        self._use_rot_value = {side: t.rpy is not None for side, t in initial_targets.items()}
        self._frame = frame
        self._frame_request = None
        self._status = {}                             # side -> (text, current pos, current rpy)
        self._closing = False
        self._reset_requested = False
        self._new_fields = None                       # side -> (pos, rpy) to show in the fields
        self._filling = False                         # GUI thread only: ignore slider writes we make
        # base joint -> value (m, rad): manual base command, or where the IK has the base if it moves it
        self._base_values = {joint: float(np.clip(0.0, lo, hi)) if unit == "m" else 0.0
                             for joint, _, unit, lo, hi, _ in self._base_fields}
        self._new_base_fields = None                  # base joint -> value to show in the sliders
        self._execute_requested = False
        self._cancel_requested = False
        self._robot_status = ""

        self.root = tk.Tk()
        self.root.title("Walkie full-body IK")
        self._vars, self._use_rot, self._status_labels = {}, {}, {}
        self._active, self._active_value, self._arm_widgets = {}, {}, {}

        for col, side in enumerate(initial_targets):
            box = ttk.LabelFrame(self.root, text=f"{side.capitalize()} arm", padding=8)
            box.grid(row=0, column=col, padx=8, pady=8, sticky="n")
            t = initial_targets[side]
            values = list(t.pos) + list(np.degrees(t.rpy if t.rpy is not None else np.zeros(3)))
            self._vars[side], self._arm_widgets[side] = [], []

            # arm on/off: an inactive arm gets no target, so the IK holds it still and ignores it
            self._active[side] = tk.BooleanVar(value=True)
            self._active_value[side] = True
            ttk.Checkbutton(box, text="Control this arm", variable=self._active[side],
                            command=lambda s=side: self._on_active(s)).grid(row=0, column=0, columnspan=2, sticky="w")

            for row, ((name, unit, lo, hi, res), value) in enumerate(zip(self._fields, values), start=1):
                ttk.Label(box, text=f"{name} [{unit}]").grid(row=row, column=0, sticky="sw", pady=(0, 4))
                var = tk.DoubleVar(value=value)
                slider = tk.Scale(box, variable=var, from_=lo, to=hi, resolution=res, orient="horizontal",
                                  length=SLIDER_LENGTH, digits=4 if unit == "m" else 0)
                slider.grid(row=row, column=1, padx=4)
                var.trace_add("write", lambda *_, s=side, i=row - 1: self._on_slider(s, i))
                self._vars[side].append(var)
                self._arm_widgets[side].append(slider)

            self._use_rot[side] = tk.BooleanVar(value=t.rpy is not None)
            rot_check = ttk.Checkbutton(box, text="Track orientation", variable=self._use_rot[side],
                                        command=lambda s=side: self._on_rot(s))
            rot_check.grid(row=7, column=0, columnspan=2, sticky="w")
            copy_button = ttk.Button(box, text="Set to current pose", command=lambda s=side: self._copy_current(s))
            copy_button.grid(row=8, column=0, columnspan=2, pady=4)
            self._arm_widgets[side] += [rot_check, copy_button]
            self._status_labels[side] = ttk.Label(box, text="", font=("TkFixedFont", 9), justify="left")
            self._status_labels[side].grid(row=9, column=0, columnspan=2, sticky="w")

        # base: each group of base joints is either moved by the IK (checked) or follows its
        # manual slider (unchecked; the slider then follows the robot while the IK moves it)
        base_box = ttk.LabelFrame(self.root, text="Base", padding=8)
        base_box.grid(row=1, column=0, columnspan=len(initial_targets), padx=8, pady=(0, 8), sticky="ew")
        ttk.Label(base_box, text="Moved by the IK (unchecked = manual sliders below):").grid(
            row=0, column=0, columnspan=len(BASE_TOGGLES), sticky="w")
        self._base_vars, self._base_checks = {}, {}
        self._base_groups_value = frozenset()
        for col, (group, label) in enumerate(BASE_TOGGLES):
            self._base_vars[group] = tk.BooleanVar(value=False)
            self._base_checks[group] = ttk.Checkbutton(base_box, text=label, variable=self._base_vars[group],
                                                       command=self._on_base)
            self._base_checks[group].grid(row=1, column=col, padx=6, sticky="w")

        manual = ttk.Frame(base_box)
        manual.grid(row=2, column=0, columnspan=len(BASE_TOGGLES), pady=(6, 0), sticky="w")
        self._base_slider_vars, self._base_sliders = {}, {}
        for i, (joint, _, unit, lo, hi, res) in enumerate(self._base_fields):
            ttk.Label(manual, text=f"{joint} [{unit}]").grid(row=i // 2, column=2 * (i % 2), sticky="sw",
                                                             padx=(6, 0), pady=(0, 4))
            value = self._base_values[joint]
            var = tk.DoubleVar(value=np.degrees(value) if unit == "deg" else value)
            slider = tk.Scale(manual, variable=var, from_=lo, to=hi, resolution=res, orient="horizontal",
                              length=SLIDER_LENGTH, digits=4 if unit == "m" else 0)
            slider.grid(row=i // 2, column=2 * (i % 2) + 1, padx=4)
            var.trace_add("write", lambda *_, j=joint: self._on_base_slider(j))
            self._base_slider_vars[joint], self._base_sliders[joint] = var, slider

        # target frame: switching re-expresses the targets in the new frame, so the robot does not move
        frame_box = ttk.LabelFrame(self.root, text="Target frame", padding=8)
        frame_box.grid(row=2, column=0, columnspan=len(initial_targets), padx=8, pady=(0, 8), sticky="ew")
        self._frame_var = tk.StringVar(value=frame)
        for col, (name, label) in enumerate(FRAME_CHOICES):
            ttk.Radiobutton(frame_box, text=label, value=name, variable=self._frame_var,
                            command=self._on_frame).grid(row=0, column=col, padx=6, sticky="w")

        # display: draw the frame of every joint of the controlled arms (and the base) in the viewer
        display_box = ttk.LabelFrame(self.root, text="Display", padding=8)
        display_box.grid(row=3, column=0, columnspan=len(initial_targets), padx=8, pady=(0, 8), sticky="ew")
        self._joint_frames = tk.BooleanVar(value=False)
        self._joint_frames_value = False
        ttk.Checkbutton(display_box, text="Show joint frames (controlled arms + base)",
                        variable=self._joint_frames, command=self._on_joint_frames).grid(row=0, column=0, sticky="w")
        # drag the targets in the MuJoCo viewer instead of pushing the robot (DragTargets)
        self._drag_targets = tk.BooleanVar(value=False)
        self._drag_targets_value = False
        ttk.Checkbutton(display_box, text="Drag targets in MuJoCo (double-click a hand; Ctrl + left drag = "
                                          "rotate, Ctrl + right drag = move)",
                        variable=self._drag_targets, command=self._on_drag_targets).grid(row=1, column=0, sticky="w")

        # robot: execute the preview's solution (lift + arms -> MoveIt, base -> Nav2, at once)
        robot_box = ttk.LabelFrame(self.root, text="Robot", padding=8)
        robot_box.grid(row=4, column=0, columnspan=len(initial_targets), padx=8, pady=(0, 8), sticky="ew")
        ttk.Button(robot_box, text="Execute (MoveIt + Nav2)", command=self._on_execute).grid(row=0, column=0, padx=4)
        ttk.Button(robot_box, text="Cancel", command=self._on_cancel).grid(row=0, column=1, padx=4)
        ttk.Button(robot_box, text="Sync preview to robot", command=self._on_reset).grid(row=0, column=2, padx=4)
        self._robot_status_label = ttk.Label(robot_box, text="", font=("TkFixedFont", 9), justify="left")
        self._robot_status_label.grid(row=1, column=0, columnspan=3, sticky="w", pady=(6, 0))

        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        self.root.after(100, self._poll)

    # ---- called from the control thread ----

    def get_request(self):
        """(targets {side: Target} of the active arms only, base groups the IK may move,
        frame the targets are in, manual base command {base joint: value})"""
        with self._lock:
            targets = {side: self._target(side) for side in self._values if self._active_value[side]}
            return targets, self._base_groups_value, self._frame, dict(self._base_values)

    def set_base_values(self, values: dict) -> None:
        """Move the manual base sliders to {base joint: value}, e.g. to where the IK has the base."""
        with self._lock:
            self._base_values.update({joint: float(v) for joint, v in values.items()})
            self._new_base_fields = {**(self._new_base_fields or {}), **{j: self._base_values[j] for j in values}}

    def set_status(self, side: str, text: str, pos, rpy) -> None:
        """pos, rpy: the arm's current pose in the frame get_request() returned."""
        with self._lock:
            self._status[side] = (text, np.array(pos), np.array(rpy))

    def take_reset(self) -> bool:
        """True once per press of the sync button."""
        with self._lock:
            requested, self._reset_requested = self._reset_requested, False
            return requested

    def set_targets(self, poses: dict) -> None:
        """Replace the targets with {side: (pos, rpy)} in the current frame, e.g. after a reset."""
        with self._lock:
            for side, (pos, rpy) in poses.items():
                self._values[side] = np.concatenate([pos, rpy]).astype(float)
            self._new_fields = dict(poses)

    def take_frame_request(self):
        """The frame the user switched to, once, or None if unchanged."""
        with self._lock:
            request, self._frame_request = self._frame_request, None
            return None if request == self._frame else request

    def switch_frame(self, frame: str, convert) -> None:
        """
        Switch the targets to `frame`. convert(pos, rpy, from_frame, to_frame) -> (pos, rpy)
        re-expresses every target (also untracked orientations and inactive arms), so the
        targets stay where they are in the world.
        """
        with self._lock:
            poses = {}
            for side, v in self._values.items():
                pos, rpy = convert(v[:3], v[3:], self._frame, frame)
                self._values[side] = np.concatenate([pos, rpy]).astype(float)
                poses[side] = (pos, rpy)
            self._frame = frame
            self._status.clear()                      # was in the old frame
            self._new_fields = poses

    def take_execute(self) -> bool:
        """True once per press of the execute button."""
        with self._lock:
            requested, self._execute_requested = self._execute_requested, False
            return requested

    def take_cancel(self) -> bool:
        """True once per press of the cancel button."""
        with self._lock:
            requested, self._cancel_requested = self._cancel_requested, False
            return requested

    def set_robot_status(self, text: str) -> None:
        with self._lock:
            self._robot_status = text

    def show_joint_frames(self) -> bool:
        with self._lock:
            return self._joint_frames_value

    def drag_targets_enabled(self) -> bool:
        with self._lock:
            return self._drag_targets_value

    def get_target_values(self) -> dict:
        """{side: (pos, rpy)} of every arm in the current frame, also inactive arms and
        untracked orientations (unlike get_request())."""
        with self._lock:
            return {side: (v[:3].copy(), v[3:].copy()) for side, v in self._values.items()}

    def close(self) -> None:
        with self._lock:
            self._closing = True

    def is_closed(self) -> bool:
        """True once the window is closed (or close() was called)."""
        with self._lock:
            return self._closing

    # ---- GUI thread ----

    def run(self) -> None:
        self.root.mainloop()
        with self._lock:
            self._closing = True                      # tells the control thread to stop
        self.root.destroy()

    def _target(self, side: str):
        """Target built from the exact values (call with the lock held)."""
        v = self._values[side]
        return dataclasses.replace(self._template[side], pos=v[:3].copy(),
                                   rpy=v[3:].copy() if self._use_rot_value[side] else None)

    def _on_slider(self, side: str, index: int) -> None:
        if self._filling:
            return
        value = self._vars[side][index].get()
        with self._lock:
            self._values[side][index] = value if index < 3 else np.radians(value)

    def _on_rot(self, side: str) -> None:
        use_rot = bool(self._use_rot[side].get())
        with self._lock:
            self._use_rot_value[side] = use_rot

    def _on_frame(self) -> None:
        relative = self._frame_var.get() != "world"
        for group in RELATIVE_LOCKED_GROUPS:
            if relative:
                self._base_vars[group].set(False)
            self._base_checks[group].state(["disabled"] if relative else ["!disabled"])
        self._on_base()
        with self._lock:
            self._frame_request = self._frame_var.get()

    def _on_base(self) -> None:
        groups = frozenset(g for g, var in self._base_vars.items() if var.get())
        for joint, group, *_ in self._base_fields:
            self._set_slider_state(self._base_sliders[joint], group not in groups)
        with self._lock:
            self._base_groups_value = groups

    def _set_slider_state(self, slider: tk.Scale, enabled: bool) -> None:
        """Reconfiguring a Scale writes its value rounded to the resolution back into its
        variable; ignore that write so the exact values are kept."""
        self._filling = True
        try:
            slider.config(state="normal" if enabled else "disabled")
        finally:
            self._filling = False

    def _on_base_slider(self, joint: str) -> None:
        if self._filling:
            return
        value = self._base_slider_vars[joint].get()
        with self._lock:
            self._base_values[joint] = np.radians(value) if joint == "yaw" else value

    def _on_active(self, side: str) -> None:
        active = bool(self._active[side].get())
        for widget in self._arm_widgets[side]:
            if isinstance(widget, tk.Scale):
                self._set_slider_state(widget, active)
            else:
                widget.state(["!disabled"] if active else ["disabled"])
        with self._lock:
            self._active_value[side] = active

    def _on_joint_frames(self) -> None:
        show = bool(self._joint_frames.get())
        with self._lock:
            self._joint_frames_value = show

    def _on_drag_targets(self) -> None:
        enabled = bool(self._drag_targets.get())
        with self._lock:
            self._drag_targets_value = enabled

    def _on_reset(self) -> None:
        with self._lock:
            self._reset_requested = True

    def _on_execute(self) -> None:
        with self._lock:
            self._execute_requested = True

    def _on_cancel(self) -> None:
        with self._lock:
            self._cancel_requested = True

    def _copy_current(self, side: str) -> None:
        with self._lock:
            status = self._status.get(side)
        if status is None:
            return
        _, pos, rpy = status
        with self._lock:
            self._values[side] = np.concatenate([pos, rpy]).astype(float)
        self._fill_fields(side, pos, rpy)

    def _fill_fields(self, side: str, pos, rpy) -> None:
        """Show a pose in the sliders without writing it back into self._values."""
        self._filling = True
        try:
            for var, value in zip(self._vars[side], list(pos) + list(np.degrees(rpy))):
                var.set(round(float(value), 3))
        finally:
            self._filling = False

    def _poll(self) -> None:
        with self._lock:
            closing = self._closing
            texts = {side: s[0] for side, s in self._status.items()}
            new_fields, self._new_fields = self._new_fields, None
            new_base, self._new_base_fields = self._new_base_fields, None
            robot_status = self._robot_status
        if closing:
            self.root.quit()
            return
        if new_base:
            self._filling = True
            try:
                for joint, value in new_base.items():
                    self._base_slider_vars[joint].set(round(float(np.degrees(value) if joint == "yaw" else value), 3))
            finally:
                self._filling = False
        for side, (pos, rpy) in (new_fields or {}).items():
            self._fill_fields(side, pos, rpy)
        for side, text in texts.items():
            self._status_labels[side].config(text=text)
        self._robot_status_label.config(text=robot_status)
        self.root.after(100, self._poll)
