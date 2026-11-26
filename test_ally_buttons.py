#!/usr/bin/env python3
"""Print ROG Ally X stick, D-pad, trigger, and button events.

The controller is the ASUS N-KEY device (0b05:1b4c). The kernel's hid-asus
driver already exposes it as a joystick; this script only reads that device.

    ./test_ally_buttons.py

Ctrl+C stops it. Stick lines are the position after it moves. Button and
D-pad lines are one event each.
"""

import glob
import os
import select
import struct
import sys

EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)
GAMEPAD = "/dev/input/by-id/usb-ASUSTeK_Computer_Inc._N-KEY_Device-if05-event-joystick"

EV_KEY = 0x01
EV_ABS = 0x03

# Codes the kernel assigns to this pad's 14 buttons, in report order.
BUTTONS = {
    304: "BTN_SOUTH",
    305: "BTN_EAST",
    306: "BTN_C",
    307: "BTN_NORTH",
    308: "BTN_WEST",
    309: "BTN_Z",
    310: "BTN_TL",
    311: "BTN_TR",
    312: "BTN_TL2",
    313: "BTN_TR2",
    314: "BTN_SELECT",
    315: "BTN_START",
    316: "BTN_MODE",
    317: "BTN_THUMBL",
}
AXES = {
    0: ("left x", 65535),
    1: ("left y", 65535),
    2: ("left trigger", 1023),
    3: ("right x", 65535),
    4: ("right y", 65535),
    5: ("right trigger", 1023),
    16: ("d-pad x", None),
    17: ("d-pad y", None),
}
STICKS = (0, 1, 3, 4)
TRIGGERS = (2, 5)
HAT = (16, 17)


def ioctl_read(fd, nr, size):
    import fcntl
    buf = bytearray(size)
    fcntl.ioctl(fd, (2 << 30) | (ord("E") << 8) | nr | (size << 16), buf)
    return buf


def device_name(fd):
    raw = ioctl_read(fd, 0x06, 256)
    return raw.split(b"\x00", 1)[0].decode(errors="replace")


def abs_state(fd, axis):
    raw = ioctl_read(fd, 0x40 + axis, 24)
    value, minimum, maximum, _fuzz, _flat, _res = struct.unpack("iiiiii", raw)
    return value, minimum, maximum


def open_gamepad():
    path = GAMEPAD if os.path.exists(GAMEPAD) else None
    if path is None:
        for candidate in sorted(glob.glob("/dev/input/event*")):
            try:
                fd = os.open(candidate, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                continue
            try:
                if "N-KEY" in device_name(fd) or device_name(fd) == "Asus Keyboard":
                    # The gamepad node is the one with stick axes. Confirm below.
                    _value, _lo, hi = abs_state(fd, 0)
                    if hi > 1000:
                        return fd, os.path.realpath(candidate)
            except OSError:
                pass
            os.close(fd)
        raise SystemExit("Ally gamepad not found. Expected " + GAMEPAD)
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    return fd, os.path.realpath(path)


def other_nkey_nodes(gamepad_path):
    found = []
    blocked = []
    for path in sorted(glob.glob("/dev/input/by-id/*N-KEY*")):
        real = os.path.realpath(path)
        if real == gamepad_path or not os.path.basename(real).startswith("event"):
            continue
        try:
            fd = os.open(real, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            blocked.append(path)
            continue
        found.append((fd, real))
    return found, blocked


def axis_ends(ranges, axis):
    _value, minimum, maximum = ranges[axis]
    return minimum, maximum


def norm_stick(value, minimum, maximum):
    center = (minimum + maximum) / 2
    half = (maximum - minimum) / 2 or 1
    return max(-1.0, min(1.0, (value - center) / half))


def norm_trigger(value, minimum, maximum):
    span = (maximum - minimum) or 1
    return max(0.0, min(1.0, (value - minimum) / span))


def hat_text(x, y):
    vertical = {-1: "up", 1: "down"}.get(y, "")
    horizontal = {-1: "left", 1: "right"}.get(x, "")
    text = " ".join(part for part in (vertical, horizontal) if part)
    return text or "center"


def main():
    sys.stdout.reconfigure(line_buffering=True)
    fd, path = open_gamepad()
    extras, blocked = other_nkey_nodes(path)
    ranges = {axis: abs_state(fd, axis) for axis in AXES}
    state = {axis: ranges[axis][0] for axis in AXES}
    held = set()
    last_stick = {axis: None for axis in STICKS}
    last_trigger = {axis: False for axis in TRIGGERS}

    print(f"ROG Ally X gamepad  {path}")
    print("Press sticks, the D-pad, triggers, and buttons. Ctrl+C quits.")
    if blocked:
        print("Some N-KEY interfaces are not readable from this shell, so a button")
        print("that the Ally sends as a keyboard key will not appear here:")
        for path_blocked in blocked:
            print(f"  {path_blocked}")
    print()

    fds = [fd] + [extra for extra, _path in extras]
    names = {fd: "gamepad"}
    pending = {open_fd: b"" for open_fd in fds}
    for extra, extra_path in extras:
        names[extra] = extra_path

    try:
        while True:
            readable, _, _ = select.select(fds, [], [], None)
            for ready in readable:
                try:
                    pending[ready] += os.read(ready, EVENT_SIZE * 64)
                except BlockingIOError:
                    continue
                data = pending[ready]
                count = len(data) // EVENT_SIZE
                pending[ready] = data[count * EVENT_SIZE:]
                for index in range(count):
                    _sec, _usec, ev_type, code, value = struct.unpack_from(
                        EVENT_FMT, data, index * EVENT_SIZE
                    )
                    if ready != fd:
                        if ev_type == EV_KEY:
                            edge = "pressed" if value == 1 else "released" if value == 0 else "repeat"
                            print(f"{edge:8}  key {code}  ({names[ready]})")
                        continue
                    if ev_type == EV_KEY and code in BUTTONS:
                        if value == 1:
                            held.add(code)
                            print(f"pressed   {BUTTONS[code]}")
                        elif value == 0 and code in held:
                            held.discard(code)
                            print(f"released  {BUTTONS[code]}")
                    elif ev_type == EV_ABS and code in AXES:
                        state[code] = value
                        if code in HAT:
                            print(f"d-pad     {hat_text(state[16], state[17])}")
                        elif code in STICKS:
                            position = norm_stick(value, *axis_ends(ranges, code))
                            previous = last_stick[code]
                            if previous is None or abs(position - previous) >= 0.08:
                                last_stick[code] = position
                                lx = norm_stick(state[0], *axis_ends(ranges, 0))
                                ly = norm_stick(state[1], *axis_ends(ranges, 1))
                                rx = norm_stick(state[3], *axis_ends(ranges, 3))
                                ry = norm_stick(state[4], *axis_ends(ranges, 4))
                                print(f"sticks    left {lx:+.2f} {ly:+.2f}    right {rx:+.2f} {ry:+.2f}")
                        elif code in TRIGGERS:
                            position = norm_trigger(value, *axis_ends(ranges, code))
                            down = position >= 0.05
                            if down != last_trigger[code]:
                                last_trigger[code] = down
                                edge = "pressed" if down else "released"
                                print(f"{edge:8}  {AXES[code][0]}  {position:.2f}")
    except KeyboardInterrupt:
        print()
    finally:
        for open_fd in fds:
            os.close(open_fd)


if __name__ == "__main__":
    sys.exit(main() or 0)
