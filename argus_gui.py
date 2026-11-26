#!/usr/bin/env python3
"""Handheld control for Argus modules, the OpenSim model, and the hotspot.

D-pad moves. South toggles or runs the highlighted control. East stops that
function. North switches the bag between pose data and raw data.
OpenSenseRT publishes /argusN/mahoney_odomimu from the IMU only. Pose and model
segments choose the OpenSim stream and model (models/<name>/).

    source source_env.sh
    ./argus_gui.py
"""

import os
import queue
import re
import select
import signal
import struct
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path

from skeleton_view import SkeletonView

ROOT = Path(__file__).resolve().parent
MODULES = ["all", "argus0", "argus1", "argus2", "argus3", "argus4"]
DEFAULT_MODEL = "rajagopal_upper"
MODEL_LABELS = {
    "rajagopal_upper": "Upper Limb",
    "rajagopal_lower": "Lower Limb",
    "bimanual_upper_arm": "Upper Limb (Holzbaur)",
    "gait2354": "Lower Limb (gait2354)",
}
MODEL_TITLES = {
    "rajagopal_upper": "Upper Limb",
    "rajagopal_lower": "Lower Limb",
    "bimanual_upper_arm": "Upper Limb",
    "gait2354": "Lower Limb",
}
GAMEPAD = "/dev/input/by-id/usb-ASUSTeK_Computer_Inc._N-KEY_Device-if05-event-joystick"
EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)
EV_KEY = 0x01
EV_ABS = 0x03
BTN_SOUTH = 304
BTN_EAST = 305
BTN_NORTH = 307
HAT_X = 16
HAT_Y = 17

BG = "#12141a"
PANEL = "#181b22"
BTN = "#2a2f38"
FOCUS = "#d4b45a"
OFF = "#2e333c"
ON = "#2f8f68"
WAIT = "#c4892a"
FG = "#e8e8e8"
MUTED = "#8b919a"
KNOB = "#f3f3f3"
TRACK_OFF = "#3a404a"
SEL = "#3a424e"
FONT = ("DejaVu Sans", 13)
FONT_SMALL = ("DejaVu Sans", 10)
FONT_LABEL = ("DejaVu Sans", 12)
FONT_MONO = ("DejaVu Sans Mono", 10)

STOP_SENSORS = (
    "pkill -INT -x bmi270_fast_nod; pkill -INT -x usb_cam_node; pkill -INT -x led_manager; "
    "pkill -INT -f '[r]os2 run argus_bmi270'; pkill -INT -f '[r]os2 run argus_usb_cam'; "
    "pkill -INT -f '[r]os2 launch argus_led_manager'"
)
STOP_VIO = (
    "pkill -INT -x run_subscribe_m; pkill -INT -f '[r]os2 run ov_msckf'; pkill -INT -x odom_throttle"
)
STOP_MAHONEY = (
    "pkill -INT -x mahoney_node; pkill -INT -f '[r]os2 run argus_mahoney'"
)
STOP_BAG = "pkill -INT -f '[b]in/ros2 bag record'"


def load_conf():
    values = {}
    for path in (ROOT / "argus.conf", ROOT / "argus.local.conf"):
        if not path.is_file():
            continue
        for line in path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def available_models():
    """Folders under models/ with an argus_model.json, the default first."""
    names = sorted(p.parent.name for p in (ROOT / "models").glob("*/argus_model.json"))
    return sorted(names, key=lambda name: name != DEFAULT_MODEL) or [DEFAULT_MODEL]


def step_focus(controls, current, direction):
    """Next control in a direction. Up and down stay near the same column."""
    row, col = current.row, current.col
    ranked = []
    for control in controls:
        if control is current:
            continue
        if direction == "up" and control.row < row:
            ranked.append((row - control.row, abs(control.col - col), control))
        elif direction == "down" and control.row > row:
            ranked.append((control.row - row, abs(control.col - col), control))
        elif direction == "left" and control.row == row and control.col < col:
            ranked.append((col - control.col, 0, control))
        elif direction == "right" and control.row == row and control.col > col:
            ranked.append((control.col - col, 0, control))
    if not ranked:
        return current
    ranked.sort(key=lambda item: (item[0], item[1]))
    return ranked[0][2]


class Control:
    def __init__(self, row, col, widget, command, module, kind):
        self.row = row
        self.col = col
        self.widget = widget
        self.command = command
        self.module = module
        self.kind = kind


class Switch(tk.Canvas):
    """Two-state toggle. The knob sits left (off) or right (on); no animation."""

    WIDTH = 54
    HEIGHT = 30

    def __init__(self, parent, command):
        super().__init__(
            parent, width=self.WIDTH, height=self.HEIGHT, bg=BG,
            highlightthickness=0, bd=0,
        )
        self._command = command
        self.on = False
        self.focused = False
        self.bind("<Button-1>", lambda _event: self._command())
        self.draw()

    def set(self, on):
        on = bool(on)
        if on != self.on:
            self.on = on
            self.draw()

    def set_focused(self, focused):
        focused = bool(focused)
        if focused != self.focused:
            self.focused = focused
            self.draw()

    def draw(self):
        self.delete("all")
        w, h = self.WIDTH, self.HEIGHT
        fill = ON if self.on else TRACK_OFF
        if self.focused:
            self._pill(0, 0, w - 1, h - 1, FOCUS)
            self._pill(2, 2, w - 3, h - 3, fill)
            pad = 6
        else:
            self._pill(1, 1, w - 2, h - 2, fill)
            pad = 4
        r = (h - 2 * pad) / 2
        cy = h / 2
        cx = (w - pad - r) if self.on else (pad + r)
        self.create_oval(cx - r, cy - r, cx + r, cy + r, fill=KNOB, outline="")

    def _pill(self, x1, y1, x2, y2, fill):
        r = (y2 - y1) / 2
        self.create_oval(x1, y1, x1 + 2 * r, y2, fill=fill, outline="")
        self.create_oval(x2 - 2 * r, y1, x2, y2, fill=fill, outline="")
        self.create_rectangle(x1 + r, y1, x2 - r, y2, fill=fill, outline="")


class Segment(tk.Frame):
    """Exclusive choice between two (or more) labeled states."""

    def __init__(self, parent, options, command):
        super().__init__(
            parent, bg=BG, highlightthickness=2,
            highlightbackground=BG, highlightcolor=FOCUS,
        )
        self.options = list(options)
        self._command = command
        self.value = self.options[0][0]
        self.parts = []
        for value, label in self.options:
            part = tk.Label(self, text=label, font=FONT_SMALL, padx=10, pady=5, cursor="hand2")
            part.pack(side="left")
            part.bind("<Button-1>", lambda _event, value=value: self._pick(value))
            self.parts.append((value, part))
        self._refresh()

    def _pick(self, value):
        if value != self.value:
            self._command(value)

    def set(self, value):
        if value != self.value:
            self.value = value
            self._refresh()

    def set_focused(self, focused):
        self.configure(highlightbackground=FOCUS if focused else BG)

    def _refresh(self):
        for value, part in self.parts:
            on = value == self.value
            part.configure(bg=SEL if on else OFF, fg=FG if on else MUTED)


class ArgusGui:
    def __init__(self):
        self.conf = load_conf()
        self.root = tk.Tk()
        self.root.title("Argus")
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        try:
            self.root.attributes("-zoomed", True)
        except tk.TclError:
            self.root.geometry("1280x800")

        self.jobs = {}
        self.modes = {name: "cam" for name in MODULES}
        self.pose_source = "openvins"
        self.models = available_models()
        self.model_name = self.models[0]
        self.drift_fix = False
        self.ips = {}
        self.status = {
            name: {"on": False, "cam": False, "imu": False, "vio": False, "mahoney": False, "rec": False, "rec_mode": ""}
            for name in MODULES
            if name != "all"
        }
        self.track_last = {name: 0.0 for name in self.status}
        self.hotspot_up = False
        self.model_up = False
        self.focused = None
        self.controls = []
        self.lamps = {}
        self.buttons = {}
        self.modal_ok = None
        self.modal_cancel = None
        self.repeat_id = None
        self.hat = {HAT_X: 0, HAT_Y: 0}
        self.alive = True
        self.polling = False
        self.poll_again = False
        self.poll_lock = threading.Lock()
        self.ips_lock = threading.Lock()
        self.status_queue = queue.Queue()
        self.status_epoch = 0.0
        self.last_discover = 0.0
        self.skeleton = None

        self._build()
        self._paint_focus(self.controls[0])
        self._bind_keys()
        threading.Thread(target=self._gamepad_loop, daemon=True).start()
        threading.Thread(target=self._track_loop, daemon=True).start()
        self.root.after(200, self._poll)
        self.root.after(500, self._refresh_lamps)

    def _build(self):
        grid = tk.Frame(self.root, bg=BG)
        grid.pack(anchor="w", padx=20, pady=(18, 0))
        pad = dict(padx=(0, 16), pady=5, sticky="w")
        headers = ["", "", "Sensors", "OpenVINS", "OpenSenseRT", "Bag Data", "Record Bag"]
        for col, text in enumerate(headers):
            tk.Label(grid, text=text, bg=BG, fg=MUTED, font=FONT_SMALL).grid(
                row=0, column=col, pady=(0, 4), padx=(0, 16), sticky="w"
            )

        for index, name in enumerate(MODULES):
            row = index + 1
            label = "All" if name == "all" else name
            tk.Label(grid, text=label, bg=BG, fg=FG, font=FONT_LABEL, width=8, anchor="w").grid(
                row=row, column=0, sticky="w", padx=(0, 8), pady=5
            )
            lamps = tk.Frame(grid, bg=BG)
            lamps.grid(row=row, column=1, padx=(0, 16), sticky="w")
            self.lamps[name] = {}
            for kind in ("ON", "CAM", "IMU", "TRK"):
                lamp = tk.Label(lamps, text=kind, width=5, bg=OFF, fg=MUTED, font=FONT_SMALL, padx=4, pady=2)
                lamp.pack(side="left", padx=2)
                self.lamps[name][kind] = lamp
            self.buttons[name] = {}
            specs = [
                (2, "sensors", lambda name=name: self.toggle_sensors(name),
                 Switch(grid, lambda name=name: self.toggle_sensors(name))),
                (3, "vio", lambda name=name: self.toggle_openvins(name),
                 Switch(grid, lambda name=name: self.toggle_openvins(name))),
                (4, "mahoney", lambda name=name: self.toggle_mahoney(name),
                 Switch(grid, lambda name=name: self.toggle_mahoney(name))),
                (5, "mode", lambda name=name: self.cycle_mode(name),
                 Segment(grid, (("cam", "Raw Data"), ("odom", "Pose Data")),
                         lambda value, name=name: self.set_mode(name, value))),
                (6, "rec", lambda name=name: self.toggle_record(name),
                 Switch(grid, lambda name=name: self.toggle_record(name))),
            ]
            for col, kind, command, widget in specs:
                widget.grid(row=row, column=col, **pad)
                self.controls.append(Control(row, col, widget, command, name, kind))
                self.buttons[name][kind] = widget

        osim_row, hot_row = 8, 9
        tk.Frame(grid, bg="#2a2f38", height=1).grid(
            row=7, column=0, columnspan=7, sticky="ew", pady=(10, 10)
        )
        tk.Label(grid, text="OpenSim", bg=BG, fg=FG, font=FONT_LABEL, width=8, anchor="w").grid(
            row=osim_row, column=0, sticky="w", padx=(0, 8)
        )
        self.model_switch = Switch(grid, self.toggle_model)
        self.model_switch.grid(row=osim_row, column=2, **pad)
        self.controls.append(Control(osim_row, 2, self.model_switch, self.toggle_model, None, "model"))

        osim_actions = tk.Frame(grid, bg=BG)
        osim_actions.grid(row=osim_row, column=3, columnspan=4, sticky="w")
        init = self._button(osim_actions, "Initialize", self.initialize_model)
        init.pack(side="left", padx=(0, 6))
        self.controls.append(Control(osim_row, 3, init, self.initialize_model, None, "model-init"))
        cal = self._button(osim_actions, "Calibrate", self.start_calibration)
        cal.pack(side="left", padx=(0, 14))
        self.controls.append(Control(osim_row, 4, cal, self.start_calibration, None, "model-cal"))
        tk.Label(osim_actions, text="Poses", bg=BG, fg=MUTED, font=FONT_SMALL).pack(side="left", padx=(0, 6))
        self.pose_button = Segment(
            osim_actions, (("openvins", "OpenVINS"), ("mahoney", "OpenSenseRT")), self.set_pose_source
        )
        self.pose_button.pack(side="left")
        self.controls.append(Control(osim_row, 5, self.pose_button, self.cycle_pose_source, None, "pose-source"))
        tk.Label(osim_actions, text="Model", bg=BG, fg=MUTED, font=FONT_SMALL).pack(side="left", padx=(14, 6))
        self.model_button = Segment(
            osim_actions, [(name, MODEL_LABELS.get(name, name)) for name in self.models], self.set_model
        )
        self.model_button.set(self.model_name)
        self.model_button.pack(side="left")
        self.controls.append(Control(osim_row, 6, self.model_button, self.cycle_model, None, "which-model"))
        tk.Label(osim_actions, text="Frame Correction", bg=BG, fg=MUTED, font=FONT_SMALL).pack(side="left", padx=(14, 6))
        self.drift_button = Switch(osim_actions, self.toggle_drift)
        self.drift_button.pack(side="left")
        self.controls.append(Control(osim_row, 7, self.drift_button, self.toggle_drift, None, "drift-fix"))

        tk.Label(grid, text="Hotspot", bg=BG, fg=FG, font=FONT_LABEL, width=8, anchor="w").grid(
            row=hot_row, column=0, sticky="w", padx=(0, 8), pady=(8, 0)
        )
        self.hotspot_switch = Switch(grid, self.toggle_hotspot)
        self.hotspot_switch.grid(row=hot_row, column=2, padx=(0, 16), pady=(8, 5), sticky="w")
        self.controls.append(Control(hot_row, 2, self.hotspot_switch, self.toggle_hotspot, None, "hot"))
        self.hotspot_label = tk.Label(grid, text="checking", bg=BG, fg=MUTED, font=FONT_LABEL, anchor="w")
        self.hotspot_label.grid(row=hot_row, column=3, columnspan=4, sticky="w", pady=(8, 0))

        bottom = tk.Frame(self.root, bg=BG)
        bottom.pack(fill="both", expand=True, padx=20, pady=(14, 18))
        log_wrap = tk.Frame(bottom, bg=PANEL)
        log_wrap.pack(side="left", fill="both", expand=True)
        tk.Label(log_wrap, text="Log", bg=PANEL, fg=MUTED, font=FONT_SMALL, anchor="w").pack(
            fill="x", padx=12, pady=(10, 0)
        )
        self.log_box = tk.Text(
            log_wrap, height=9, bg=PANEL, fg="#c8ccd2", font=FONT_MONO,
            state="disabled", takefocus=0, relief="flat", padx=12, pady=8,
            highlightthickness=0, bd=0,
        )
        self.log_box.pack(fill="both", expand=True, padx=4, pady=(0, 8))
        try:
            self.skeleton = SkeletonView(bottom)
            self.skeleton.pack(side="right", fill="y", padx=(12, 0))
        except Exception as exc:
            self.skeleton = None
            tk.Label(
                bottom, text=f"Skeleton view unavailable:\n{exc}",
                bg=PANEL, fg="#c8ccd2", font=FONT_SMALL, justify="left",
            ).pack(side="right", padx=(12, 0))

    def _button(self, parent, text, command):
        return tk.Button(
            parent, text=text, command=command, font=FONT_LABEL, bg=BTN, fg=FG,
            activebackground=SEL, activeforeground=FG,
            relief="flat", bd=0, highlightthickness=2,
            highlightbackground=BG, highlightcolor=FOCUS,
            padx=12, pady=6, takefocus=0, cursor="hand2",
        )

    def _bind_keys(self):
        for key, direction in (("<Up>", "up"), ("<Down>", "down"), ("<Left>", "left"), ("<Right>", "right")):
            self.root.bind_all(key, lambda event, direction=direction: self._on_arrow(direction))
        self.root.bind_all("<Return>", lambda _event: self._south())
        self.root.bind_all("<KP_Enter>", lambda _event: self._south())
        self.root.bind_all("<Escape>", lambda _event: self._east())
        self.root.bind_all("n", lambda _event: self._north_key())
        self.root.bind_all("N", lambda _event: self._north_key())

    def _on_arrow(self, direction):
        if isinstance(self.root.focus_get(), tk.Entry) or self.modal_ok:
            return
        self._move(direction)
        return "break"

    def _north_key(self):
        if isinstance(self.root.focus_get(), tk.Entry):
            return
        self._north()
        return "break"

    def _paint_focus(self, control):
        self.focused = control
        for item in self.controls:
            focused = item is control
            widget = item.widget
            if isinstance(widget, Switch):
                widget.set(self._switch_on(item))
                widget.set_focused(focused)
            elif isinstance(widget, Segment):
                widget.set_focused(focused)
            else:
                if focused:
                    bg, fg, ring = FOCUS, "#1b1e24", FOCUS
                else:
                    bg, fg, ring = BTN, FG, BG
                widget.configure(bg=bg, fg=fg, highlightbackground=ring)

    def _switch_on(self, item):
        if item.kind == "sensors":
            return self._sensors_up(item.module)
        if item.kind == "vio":
            return self._vio_up(item.module)
        if item.kind == "mahoney":
            return self._mahoney_up(item.module)
        if item.kind == "rec":
            return self._recording(item.module)
        if item.kind == "model":
            return self.model_up
        if item.kind == "hot":
            return self.hotspot_up
        if item.kind == "drift-fix":
            return self.drift_fix
        return False

    def _move(self, direction):
        if self.modal_ok or self.focused is None:
            return
        self._paint_focus(step_focus(self.controls, self.focused, direction))

    def _south(self):
        if self.modal_ok:
            self.modal_ok()
            return "break"
        if self.focused:
            self.focused.command()
        return "break"

    def _east(self):
        if self.modal_cancel:
            self.modal_cancel()
            return "break"
        control = self.focused
        if control is None:
            return "break"
        if control.module:
            if control.kind == "sensors":
                self.stop_sensors(control.module)
            elif control.kind == "vio":
                self.stop_openvins(control.module)
            elif control.kind == "mahoney":
                self.stop_mahoney(control.module)
            elif control.kind in ("rec", "mode"):
                self.stop_record(control.module)
        elif control.kind.startswith("model"):
            self.stop_model()
        elif control.kind == "hot":
            self.stop_hotspot()
        return "break"

    def _north(self):
        if self.modal_ok:
            return "break"
        module = self.focused.module if self.focused and self.focused.module else "all"
        self.cycle_mode(module)
        for control in self.controls:
            if control.module == module and control.kind == "mode":
                self._paint_focus(control)
                break
        return "break"

    def log(self, text):
        if re.search(r"password", text, re.I):
            return
        self.log_box.configure(state="normal")
        self.log_box.insert("end", text.rstrip() + "\n")
        lines = int(self.log_box.index("end-1c").split(".")[0])
        if lines > 400:
            self.log_box.delete("1.0", f"{lines - 400}.0")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _ros(self, command):
        return ["bash", "-c", f"source '{ROOT}/source_env.sh' && {command}"]

    def _start_job(self, key, args, env=None):
        proc = self.jobs.get(key)
        if proc is not None and proc.poll() is None:
            self.log(f"{self._job_name(key)} is already running")
            return
        merged = os.environ.copy()
        if env:
            merged.update(env)
        proc = subprocess.Popen(
            args, cwd=ROOT, env=merged, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True,
        )
        self.jobs[key] = proc
        threading.Thread(target=self._read_job, args=(key, proc), daemon=True).start()
        self.log(f"started {self._job_name(key)}")
        self._request_poll()

    def _job_name(self, key):
        if isinstance(key, tuple):
            return f"{key[0]} {key[1]}"
        return str(key)

    def _read_job(self, key, proc):
        name = self._job_name(key)
        for line in proc.stdout:
            text = line.rstrip()
            if text and not re.search(r"password", text, re.I):
                self.root.after(0, lambda text=text, name=name: self.log(f"{name}: {text}"))
        code = proc.wait()
        self.root.after(0, lambda code=code, name=name: self.log(f"{name} exited ({code})"))
        self._request_poll()

    def _signal_job(self, key):
        proc = self.jobs.get(key)
        if proc is None or proc.poll() is not None:
            return False
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return False
        self.log(f"stopping {self._job_name(key)}")
        return True

    def _ssh(self, name, remote):
        with self.ips_lock:
            ip = self.ips.get(name)
        if not ip:
            self.log(f"{name} has no address")
            return
        user = self.conf.get("ARGUS_SSH_USER", "argus")
        subprocess.Popen(
            ["ssh", "-o", "StrictHostKeyChecking=accept-new", "-o", "BatchMode=yes",
             "-o", "ConnectTimeout=5", f"{user}@{ip}", remote],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def _sensors_up(self, name):
        if name == "all":
            if self._job_alive(("all", "sensors")):
                return True
            return any(item.get("on") or item.get("cam") or item.get("imu") for item in self.status.values())
        if self._job_alive((name, "sensors")):
            return True
        item = self.status.get(name, {})
        return bool(item.get("on") or item.get("cam") or item.get("imu"))

    def _vio_up(self, name):
        if name == "all":
            if self._job_alive(("all", "openvins")):
                return True
            return any(item.get("vio") for item in self.status.values())
        if self._job_alive((name, "openvins")):
            return True
        return bool(self.status.get(name, {}).get("vio"))

    def toggle_sensors(self, name):
        if self._sensors_up(name):
            self.stop_sensors(name)
        else:
            self.start_sensors(name)

    def toggle_openvins(self, name):
        if self._vio_up(name):
            self.stop_openvins(name)
        else:
            self.start_openvins(name)

    def toggle_mahoney(self, name):
        if self._mahoney_up(name):
            self.stop_mahoney(name)
        else:
            self.start_mahoney(name)

    def toggle_record(self, name):
        if self._recording(name):
            self.stop_record(name)
        else:
            self.start_record(name)

    def toggle_model(self):
        if self.model_up or self._job_alive("model"):
            self.stop_model()
        else:
            self.start_model()

    def toggle_hotspot(self):
        if self.hotspot_up:
            self.stop_hotspot()
        else:
            self.start_hotspot()

    def start_sensors(self, name):
        if name == "all":
            self._start_job(("all", "sensors"), [str(ROOT / "scripts" / "all_active.sh"), "bringup"])
            return
        self._start_job((name, "sensors"), [str(ROOT / "scripts" / "module" / "bringup_sensors.sh"), name])

    def stop_sensors(self, name):
        if name == "all":
            self._signal_job(("all", "sensors"))
        names = [module for module in MODULES if module != "all"] if name == "all" else [name]
        for module in names:
            self._signal_job((module, "sensors"))
            self._ssh(module, STOP_SENSORS)
        self._drop_status(name, cam=False, imu=False)

    def start_mahoney(self, name):
        if name == "all":
            self._start_job(("all", "mahoney"), [str(ROOT / "scripts" / "all_active.sh"), "mahoney"])
            return
        self._start_job((name, "mahoney"), [str(ROOT / "scripts" / "module" / "bringup_mahoney.sh"), name])

    def stop_mahoney(self, name):
        if name == "all":
            self._signal_job(("all", "mahoney"))
        names = [module for module in MODULES if module != "all"] if name == "all" else [name]
        for module in names:
            self._signal_job((module, "mahoney"))
            self._ssh(module, STOP_MAHONEY)
        self._drop_status(name, mahoney=False)

    def _mahoney_up(self, name):
        if name == "all":
            if self._job_alive(("all", "mahoney")):
                return True
            return any(item.get("mahoney") for item in self.status.values())
        if self._job_alive((name, "mahoney")):
            return True
        return bool(self.status.get(name, {}).get("mahoney"))

    def _job_alive(self, key):
        proc = self.jobs.get(key)
        return proc is not None and proc.poll() is None

    def start_openvins(self, name):
        if name == "all":
            self._start_job(("all", "openvins"), [str(ROOT / "scripts" / "all_active.sh"), "openvins"])
            return
        self._start_job((name, "openvins"), [str(ROOT / "scripts" / "module" / "bringup_openvins.sh"), name])

    def stop_openvins(self, name):
        if name == "all":
            self._signal_job(("all", "openvins"))
        names = [module for module in MODULES if module != "all"] if name == "all" else [name]
        for module in names:
            self._signal_job((module, "openvins"))
            self._ssh(module, STOP_VIO)
        self._drop_status(name, vio=False)

    def cycle_mode(self, name):
        self.set_mode(name, "odom" if self.modes[name] == "cam" else "cam")

    def set_mode(self, name, mode):
        self.modes[name] = mode
        self.buttons[name]["mode"].set(mode)
        if self.focused is not None:
            self._paint_focus(self.focused)

    def start_record(self, name):
        mode = self.modes[name]
        bag = time.strftime("gui_%Y%m%d_%H%M%S")
        if name == "all":
            self._start_job(
                ("all", "record"),
                [str(ROOT / "scripts" / "all_active.sh"), "record", bag],
                env={"ARGUS_RECORD_MODE": mode},
            )
            return
        self._start_job((name, "record"), [str(ROOT / "scripts" / "bags" / "record_bag.sh"), name, bag, mode])

    def stop_record(self, name):
        if name == "all":
            self._signal_job(("all", "record"))
        names = [module for module in MODULES if module != "all"] if name == "all" else [name]
        for module in names:
            if not self._signal_job((module, "record")):
                self._ssh(module, STOP_BAG)
        self._drop_status(name, rec=False, rec_mode="")

    def cycle_pose_source(self):
        self.set_pose_source("mahoney" if self.pose_source == "openvins" else "openvins")

    def set_pose_source(self, source):
        self.pose_source = source
        self.pose_button.set(source)
        if self.focused is not None:
            self._paint_focus(self.focused)

    def cycle_model(self):
        nxt = self.models[(self.models.index(self.model_name) + 1) % len(self.models)]
        self.set_model(nxt)

    def set_model(self, name):
        if name == self.model_name:
            return
        if self.model_up or self._job_alive("model") or self._job_alive("calib"):
            self.log("stop the model and close calibration before switching models")
            self.model_button.set(self.model_name)
            return
        self.model_name = name
        self.model_button.set(name)
        self.log(f"model {self.model_name}")
        if self.skeleton is not None:
            try:
                self.skeleton.set_model(self.model_name, MODEL_TITLES.get(self.model_name, self.model_name))
            except Exception as exc:
                self.log(f"skeleton view: {exc}")
        if self.focused is not None:
            self._paint_focus(self.focused)

    def toggle_drift(self):
        self.drift_fix = not self.drift_fix
        if self.model_up or self._job_alive("model"):
            self.log("frame correction changes the next time the model is started")
        if self.focused is not None:
            self._paint_focus(self.focused)

    def start_model(self):
        drift = "on" if self.drift_fix else "off"
        pose = "OpenSenseRT" if self.pose_source == "mahoney" else "OpenVINS"
        self.log(f"model {self.model_name}, poses from {pose}, frame correction {drift}")
        self._start_job(
            "model",
            self._ros(
                "exec ros2 launch argus_server argus_server.launch.py "
                f"pose_source:={self.pose_source} model:={self.model_name} "
                f"drift_correction:={'true' if self.drift_fix else 'false'}"
            ),
        )

    def stop_model(self):
        if not self._signal_job("model"):
            for name in ("argus_server", "drift_node"):
                subprocess.Popen(
                    ["pkill", "-INT", "-x", name],
                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            self.log("stopping argus_server")
        self.model_up = False
        self.status_epoch = time.monotonic()
        self._paint_lamps()
        self._request_poll()

    def initialize_model(self):
        self.log("publishing /reset_tracking")
        self._start_job(
            "init",
            self._ros('exec ros2 topic pub --once /reset_tracking std_msgs/msg/Bool "{data: true}"'),
        )

    def start_calibration(self):
        proc = self.jobs.get("calib")
        if proc is not None and proc.poll() is None:
            self.log("calibration is already open")
            return
        self._start_job(
            "calib", self._ros(f"exec python3 '{ROOT / 'pose_capture.py'}' --model {self.model_name}")
        )

    def start_hotspot(self):
        self._sudo([str(ROOT / "scripts" / "network" / "start_hotspot.sh")])

    def stop_hotspot(self):
        self._sudo([str(ROOT / "scripts" / "network" / "start_hotspot.sh"), "stop"])

    def _sudo(self, args):
        threading.Thread(target=self._sudo_worker, args=(args, None), daemon=True).start()

    def _sudo_worker(self, args, password):
        command = ["sudo", "-n", *args] if password is None else ["sudo", "-S", "-p", "", *args]
        try:
            proc = subprocess.Popen(
                command, cwd=ROOT, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            output, _ = proc.communicate(None if password is None else password + "\n", timeout=150)
        except subprocess.TimeoutExpired:
            self.root.after(0, lambda: self.log("sudo command timed out"))
            return
        text = output or ""
        if proc.returncode != 0 and password is None and "password is required" in text.lower():
            self.root.after(0, lambda: self._ask_password(args))
            return
        if proc.returncode != 0 and "try again" in text.lower():
            self.root.after(0, lambda: self.log("sudo rejected the password"))
            return
        for line in text.splitlines():
            if line.strip() and not re.search(r"password", line, re.I):
                self.root.after(0, lambda line=line: self.log(line))

    def _ask_password(self, args):
        dialog = tk.Toplevel(self.root)
        dialog.title("sudo")
        dialog.configure(bg=BG)
        dialog.transient(self.root)
        tk.Label(dialog, text="sudo password", bg=BG, fg=FG, font=FONT).pack(padx=16, pady=(16, 8))
        entry = tk.Entry(dialog, show="*", font=FONT, width=24)
        entry.pack(padx=16, pady=8)
        entry.focus_set()

        def close():
            self.modal_ok = None
            self.modal_cancel = None
            dialog.destroy()

        def accept():
            password = entry.get()
            close()
            threading.Thread(target=self._sudo_worker, args=(args, password), daemon=True).start()

        self.modal_ok = accept
        self.modal_cancel = close
        dialog.protocol("WM_DELETE_WINDOW", close)
        tk.Button(dialog, text="OK  (South)", command=accept, font=FONT, bg=FOCUS, fg="#1b1e24").pack(
            side="left", padx=16, pady=16
        )
        tk.Button(dialog, text="Cancel  (East)", command=close, font=FONT, bg=BTN, fg=FG).pack(
            side="right", padx=16, pady=16
        )

    def _recording(self, module):
        if module == "all":
            if self._job_alive(("all", "record")):
                return True
            return any(item["rec"] for item in self.status.values())
        if self._job_alive((module, "record")):
            return True
        return self.status.get(module, {}).get("rec", False)

    def _refresh_lamps(self):
        self._drain_status()
        self._paint_lamps()
        if self.alive:
            self.root.after(500, self._refresh_lamps)

    def _paint_lamps(self):
        now = time.monotonic()
        fresh = {
            name: (now - stamp) < 2.0
            for name, stamp in self.track_last.items()
        }
        counts = {"ON": 0, "CAM": 0, "IMU": 0, "TRK": 0}
        waiting = 0
        for name, item in self.status.items():
            flags = {
                "ON": item["on"],
                "CAM": item["cam"],
                "IMU": item["imu"],
                "TRK": fresh[name],
            }
            for kind, value in flags.items():
                counts[kind] += int(value)
            if item["vio"] and not fresh[name]:
                waiting += 1
            self._set_lamps(name, flags, trk_wait=item["vio"] and not fresh[name])
        self._set_lamps(
            "all",
            {kind: counts[kind] > 0 for kind in counts},
            texts={kind: f"{kind} {counts[kind]}" for kind in counts},
            trk_wait=waiting > 0 and counts["TRK"] == 0,
        )
        ssid = self.conf.get("ARGUS_SSID", "argus")
        state = "up" if self.hotspot_up else "down"
        self.hotspot_label.configure(text=f"{ssid}  ·  {state}", fg=ON if self.hotspot_up else MUTED)
        self._paint_focus(self.focused)

    def _drop_status(self, name, **flags):
        """Clear lamps for a stop, and ignore any status check that started earlier."""
        names = list(self.status) if name == "all" else [name]
        for module in names:
            item = self.status.get(module)
            if item is None:
                continue
            item.update(flags)
            if flags.get("vio") is False:
                self.track_last[module] = 0.0
        self.status_epoch = time.monotonic()
        self._paint_lamps()
        self._request_poll()

    def _request_poll(self):
        start = False
        with self.poll_lock:
            if self.polling:
                self.poll_again = True
            else:
                self.polling = True
                start = True
        if start:
            threading.Thread(target=self._poll_worker, daemon=True).start()

    def _poll(self):
        self._request_poll()
        if self.alive:
            self.root.after(2000, self._poll)

    def _poll_worker(self):
        started = time.monotonic()
        try:
            with self.ips_lock:
                hosts = dict(self.ips)
            if hosts:
                self._enqueue_status(started, self._collect_status(hosts))
            if not hosts or time.monotonic() - self.last_discover >= 20:
                self.last_discover = time.monotonic()
                discovered = self._discover()
                if discovered and discovered != hosts:
                    self._enqueue_status(time.monotonic(), self._collect_status(discovered))
        except Exception as exc:
            self.status_queue.put(("err", started, exc))
        finally:
            again = False
            with self.poll_lock:
                self.polling = False
                again = self.poll_again
                self.poll_again = False
            if again and self.alive:
                self._request_poll()

    def _enqueue_status(self, started, result):
        found, hotspot, model, hosts = result
        self.status_queue.put(("ok", started, found, hotspot, model, hosts))

    def _collect_status(self, hosts):
        results = {}
        threads = []

        def run(name, ip):
            results[name] = self._probe(ip)

        for name in self.status:
            ip = hosts.get(name)
            if not ip:
                results[name] = {
                    "on": False, "cam": False, "imu": False, "vio": False,
                    "mahoney": False, "rec": False, "rec_mode": "",
                }
                continue
            thread = threading.Thread(target=run, args=(name, ip))
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join(timeout=6)
        found = dict(results)
        hotspot = self._hotspot_up()
        model = subprocess.run(
            ["pgrep", "-x", "argus_server"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        ).returncode == 0
        return found, hotspot, model, hosts

    def _drain_status(self):
        latest = None
        while True:
            try:
                item = self.status_queue.get_nowait()
            except queue.Empty:
                break
            started = item[1]
            if started < self.status_epoch:
                continue
            if item[0] == "err":
                self.log(f"status: {item[2]}")
                continue
            latest = item
        if latest is None:
            return
        _, _, found, hotspot, model, hosts = latest
        self._apply_status(found, hotspot, model, hosts)

    def _set_lamps(self, name, flags, texts=None, trk_wait=False):
        for kind, value in flags.items():
            lamp = self.lamps[name][kind]
            if kind == "TRK" and trk_wait and not value:
                color = WAIT
            else:
                color = ON if value else OFF
            text = texts[kind] if texts else kind
            lamp.configure(text=text, bg=color, fg=FG if value or color == WAIT else MUTED)

    def _discover(self):
        proc = subprocess.run(
            ["bash", "-c", "source ./scripts/argus_common.sh && argus_each_host"],
            cwd=ROOT, capture_output=True, text=True, timeout=45,
        )
        hosts = {}
        for line in proc.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 2:
                hosts[parts[0]] = parts[1]
        return hosts

    def _probe(self, ip):
        blank = {"on": False, "cam": False, "imu": False, "vio": False, "mahoney": False, "rec": False, "rec_mode": ""}
        ping = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        if ping.returncode != 0:
            return blank
        user = self.conf.get("ARGUS_SSH_USER", "argus")
        try:
            proc = subprocess.run(
                ["ssh", "-o", "StrictHostKeyChecking=accept-new", "-o", "BatchMode=yes",
                 "-o", "ConnectTimeout=3", f"{user}@{ip}", "ps -eo args="],
                capture_output=True, text=True, timeout=4,
            )
        except subprocess.TimeoutExpired:
            blank["on"] = True
            return blank
        text = proc.stdout or ""
        blank["on"] = True
        blank["cam"] = "usb_cam_node" in text
        blank["imu"] = "bmi270_fast_node" in text
        blank["vio"] = "run_subscribe_msckf" in text
        blank["mahoney"] = "mahoney_node" in text
        for line in text.splitlines():
            if "bag record" not in line:
                continue
            blank["rec"] = True
            blank["rec_mode"] = "odom" if "odomimu" in line and "image_raw" not in line else "cam"
        return blank

    def _hotspot_up(self):
        addr = self.conf.get("ARGUS_AP_ADDR", "10.42.0.1/24").split("/")[0]
        try:
            proc = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True, timeout=3)
        except subprocess.TimeoutExpired:
            return False
        return f"{addr}/" in (proc.stdout or "")

    def _apply_status(self, found, hotspot, model, hosts):
        with self.ips_lock:
            self.ips = dict(hosts)
        for name, item in found.items():
            if name in self.status:
                self.status[name] = item
        self.hotspot_up = hotspot
        self.model_up = model

    def _track_loop(self):
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.executors import ExternalShutdownException
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState
        except Exception as exc:
            self.root.after(0, lambda: self.log(f"tracking lamps need ROS ({exc})"))
            return
        rclpy.init()
        node = rclpy.create_node("argus_gui_track")
        for name in self.track_last:
            node.create_subscription(
                Odometry,
                f"/{name}/odomimu_throttled",
                lambda _msg, name=name: self._mark_track(name),
                qos_profile_sensor_data,
            )
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        try:
            while self.alive and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
        except ExternalShutdownException:
            return
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _mark_track(self, name):
        self.track_last[name] = time.monotonic()

    def _on_joints(self, msg):
        if self.skeleton is None:
            return
        self.skeleton.set_joints({name: float(pos) for name, pos in zip(msg.name, msg.position)})

    def _gamepad_loop(self):
        try:
            fd = os.open(GAMEPAD, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            self.root.after(0, lambda: self.log(f"gamepad not available ({exc})"))
            return
        self.root.after(0, lambda: self.log("gamepad ready"))
        pending = b""
        while self.alive:
            try:
                ready, _, _ = select.select([fd], [], [], 0.3)
            except Exception:
                break
            if not ready:
                continue
            try:
                pending += os.read(fd, EVENT_SIZE * 64)
            except BlockingIOError:
                continue
            count = len(pending) // EVENT_SIZE
            chunk, pending = pending[: count * EVENT_SIZE], pending[count * EVENT_SIZE :]
            for index in range(count):
                _sec, _usec, ev_type, code, value = struct.unpack_from(EVENT_FMT, chunk, index * EVENT_SIZE)
                self.root.after(0, lambda t=ev_type, c=code, v=value: self._pad(t, c, v))
        os.close(fd)

    def _pad(self, ev_type, code, value):
        if not self.alive:
            return
        if ev_type == EV_ABS and code in self.hat:
            self.hat[code] = value
            if value:
                self._nudge_hat()
                self._arm_repeat(320)
            elif self.hat[HAT_X] == 0 and self.hat[HAT_Y] == 0 and self.repeat_id is not None:
                self.root.after_cancel(self.repeat_id)
                self.repeat_id = None
        elif ev_type == EV_KEY and value == 1:
            if code == BTN_SOUTH:
                self._south()
            elif code == BTN_EAST:
                self._east()
            elif code == BTN_NORTH:
                self._north()

    def _nudge_hat(self):
        if self.modal_ok:
            return
        if self.hat[HAT_Y] < 0:
            self._move("up")
        elif self.hat[HAT_Y] > 0:
            self._move("down")
        elif self.hat[HAT_X] < 0:
            self._move("left")
        elif self.hat[HAT_X] > 0:
            self._move("right")

    def _arm_repeat(self, delay):
        if self.repeat_id is not None:
            self.root.after_cancel(self.repeat_id)
        self.repeat_id = self.root.after(delay, self._repeat_hat)

    def _repeat_hat(self):
        self.repeat_id = None
        if self.hat[HAT_X] == 0 and self.hat[HAT_Y] == 0:
            return
        self._nudge_hat()
        self._arm_repeat(130)

    def close(self):
        self.alive = False
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    if os.environ.get("ARGUS_GUI_READY") != "1":
        try:
            import rclpy  # noqa: F401
        except ImportError:
            script = str(Path(__file__).resolve())
            command = f"source '{ROOT}/source_env.sh' && export ARGUS_GUI_READY=1 && exec python3 '{script}'"
            os.execv("/bin/bash", ["bash", "-c", command])
    ArgusGui().run()


if __name__ == "__main__":
    main()
