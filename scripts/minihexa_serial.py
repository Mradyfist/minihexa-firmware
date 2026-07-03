#!/usr/bin/env python3
"""Send commands to a MiniHexa over USB serial (e.g. from a Raspberry Pi).

The robot must be running the Hiwonder *remote* firmware (UART server), such as
the sketch in ``app-remote-control/remote`` or
``serial-communication-tutorial/miniHexa从机端程序/remote``. MicroPython firmware
does not speak this protocol.

Wire-up
-------
Connect the Pi to the robot's USB port. On Linux this is usually ``/dev/ttyUSB0``
or ``/dev/ttyACM0``. The robot listens at **115200 8N1**.

Protocol
--------
Text frames use ``|`` as a field separator and ``&`` as the end marker, e.g.::

    C|0|30|0&          # strafe / move (vx, vy, rotate)
    K|1|5&             # run action group 5
    C|0|0|0&           # stop motion

The robot broadcasts battery voltage about once per second::

    A|12345&

Dependencies
------------
    pip install pyserial

Examples
--------
    python3 minihexa_serial.py --port /dev/ttyUSB0 stop
    python3 minihexa_serial.py move --vy 30
    python3 minihexa_serial.py action 5
    python3 minihexa_serial.py listen
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, Optional

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install with: pip install pyserial"
    ) from exc

BAUD_RATE = 115200
DEFAULT_PORT = "/dev/ttyUSB0"


def build_frame(*fields: str | int) -> str:
    """Build a protocol frame from fields (``&`` is appended automatically)."""
    return "|".join(str(f) for f in fields) + "&"


class MiniHexaSerial:
    """Minimal client for the MiniHexa UART control protocol."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = BAUD_RATE, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2.0)  # allow the ESP32 to finish booting after USB connect

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "MiniHexaSerial":
        self.open()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def send(self, *fields: str | int) -> None:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("serial port is not open")
        frame = build_frame(*fields)
        self._ser.write(frame.encode("ascii"))
        self._ser.flush()

    def read_line(self) -> Optional[str]:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("serial port is not open")
        raw = self._ser.read_until(b"&")
        if not raw:
            return None
        return raw.decode("ascii", errors="replace").strip()

    # --- high-level commands supported by uart_server.cpp ---

    def stop(self) -> None:
        """Stop translation and rotation."""
        self.send("C", 0, 0, 0)

    def move(self, vx: int = 0, vy: int = 0, rotate: int = 0) -> None:
        """Send a movement command.

        ``vx`` / ``vy`` are in roughly -50..50 (mapped to body speeds on the robot).
        ``rotate`` is 0 (none), 1 (counter-clockwise), or 2 (clockwise).
        """
        for name, value, lo, hi in (
            ("vx", vx, -50, 50),
            ("vy", vy, -50, 50),
            ("rotate", rotate, 0, 2),
        ):
            if not lo <= value <= hi:
                raise ValueError(f"{name} must be between {lo} and {hi}, got {value}")
        self.send("C", vx, vy, rotate)

    def crawl(self) -> None:
        """Enter crawl / gait state (command ``B``)."""
        self.send("B")

    def pose(
        self,
        yaw: int = 0,
        pitch: int = 0,
        roll: int = 0,
        x: int = 0,
        y: int = 0,
        z: int = 0,
    ) -> None:
        """Adjust body pose (command ``F``). All values are roughly -50..50 except ``z`` (0..30)."""
        self.send("F", yaw, pitch, roll, x, y, z)

    def run_action(self, action_id: int) -> None:
        """Run a stored action group (``K|1|<id>``). Built-ins: 3=wake, 4=wake run, 5=act cute."""
        self.send("K", 1, action_id)

    def stop_action(self) -> None:
        self.send("K", 2)

    def read_battery_mv(self) -> Optional[int]:
        """Wait for one ``A|<millivolts>&`` telemetry frame."""
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            line = self.read_line()
            if not line:
                continue
            parts = line.rstrip("&").split("|")
            if parts and parts[0] == "A" and len(parts) >= 2:
                try:
                    return int(parts[1])
                except ValueError:
                    pass
        return None


def listen(robot: MiniHexaSerial, duration: Optional[float]) -> None:
    """Print incoming frames until ``duration`` seconds elapse (or forever)."""
    end = None if duration is None else time.monotonic() + duration
    while end is None or time.monotonic() < end:
        line = robot.read_line()
        if line:
            print(line)


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Control a MiniHexa over USB serial")
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"serial device (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"baud rate (default: {BAUD_RATE})")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("stop", help="stop motion (C|0|0|0&)")

    move = sub.add_parser("move", help="send a movement command")
    move.add_argument("--vx", type=int, default=0, help="sideways speed, -50..50")
    move.add_argument("--vy", type=int, default=0, help="forward/back speed, -50..50")
    move.add_argument("--rotate", type=int, default=0, choices=(0, 1, 2), help="0=none, 1=CCW, 2=CW")

    sub.add_parser("crawl", help="enter crawl state (B&)")

    act = sub.add_parser("action", help="run an action group")
    act.add_argument("id", type=int, help="action group id")

    sub.add_parser("stop-action", help="stop the current action group")

    sub.add_parser("battery", help="read one battery voltage sample (mV)")

    mon = sub.add_parser("listen", help="print incoming serial frames")
    mon.add_argument("--duration", type=float, default=None, help="seconds to listen (default: until Ctrl+C)")

    send = sub.add_parser("raw", help="send a raw frame (fields before the trailing &)")
    send.add_argument("fields", nargs="+", help="frame fields, e.g. raw C 0 30 0")

    return parser.parse_args(list(argv) if argv is not None else None)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)
    with MiniHexaSerial(args.port, args.baud) as robot:
        if args.command == "stop":
            robot.stop()
            print("sent stop")
        elif args.command == "move":
            robot.move(args.vx, args.vy, args.rotate)
            print(f"sent move vx={args.vx} vy={args.vy} rotate={args.rotate}")
        elif args.command == "crawl":
            robot.crawl()
            print("sent crawl")
        elif args.command == "action":
            robot.run_action(args.id)
            print(f"sent action {args.id}")
        elif args.command == "stop-action":
            robot.stop_action()
            print("sent stop-action")
        elif args.command == "battery":
            mv = robot.read_battery_mv()
            if mv is None:
                print("no battery frame received", file=sys.stderr)
                return 1
            print(f"battery: {mv} mV ({mv / 1000:.2f} V)")
        elif args.command == "listen":
            try:
                listen(robot, args.duration)
            except KeyboardInterrupt:
                print()
        elif args.command == "raw":
            robot.send(*args.fields)
            print(f"sent {build_frame(*args.fields)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
