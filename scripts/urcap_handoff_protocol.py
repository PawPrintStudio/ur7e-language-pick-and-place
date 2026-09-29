"""Bounded pendant handshake; no ROS imports, robot scripts, or motor commands.

Integers use URScript's documented signed int32 network byte order. A fresh
nonce and explicit phase request prevent a delayed message advancing a run.
"""
from dataclasses import dataclass
import math
import secrets
import select
import struct
import time

MAGIC = 731901
VERSION = 1


@dataclass(frozen=True)
class Settings:
    tool: int = 2
    close_mm: float = 59.0
    open_mm: float = 70.0
    force_n: float = 5.0
    expect_object: bool = False

    def __post_init__(self):
        values = (self.close_mm, self.open_mm, self.force_n)
        if (not all(math.isfinite(x) for x in values)
                or not 0 <= self.close_mm < self.open_mm <= 110
                or self.open_mm - self.close_mm > 20
                or not 3 <= self.force_n <= 10 or self.tool not in (0, 1, 2)
                or any(abs(x * 10 - round(x * 10)) > 1e-6 for x in values)):
            raise ValueError("Require tool 0/1/2, widths 0..110 mm, <=20 mm span, force 3..10 N, tenths precision")

    @property
    def hello(self):
        return (MAGIC, VERSION, self.tool, round(self.close_mm * 10),
                round(self.open_mm * 10), round(self.force_n * 10))


class Wire:
    def __init__(self, connection, check, timeout=15.0):
        self.connection = connection
        self.check = check
        self.timeout = timeout

    def receive(self, count):
        deadline = time.monotonic() + self.timeout
        data = bytearray()
        while len(data) < 4 * count:
            self.check()
            if time.monotonic() >= deadline:
                raise TimeoutError("Pendant acknowledgement timed out")
            if not select.select([self.connection], [], [], 0.02)[0]:
                continue
            chunk = self.connection.recv(4 * count - len(data))
            if not chunk:
                raise ConnectionError("Pendant socket disconnected")
            data.extend(chunk)
        self.check()
        return struct.unpack("!" + "i" * count, data)

    def send(self, *values):
        self.check()
        self.connection.settimeout(1.0)
        self.connection.sendall(struct.pack("!" + "i" * len(values), *values))


def validate_result(values, nonce, phase, settings):
    token, returned_phase, width_tenths, busy, detected = values
    if (token, returned_phase) != (nonce, phase):
        raise RuntimeError("Stale or out-of-order pendant completion")
    width = width_tenths / 10
    if not 0 <= width <= 110 or busy != 0 or detected not in (0, 1):
        raise RuntimeError("Invalid or busy gripper feedback")
    if phase == 1 and settings.expect_object:
        if detected != 1 or not settings.close_mm <= width < settings.open_mm:
            raise RuntimeError("Expected object was not detected within the grip range")
    else:
        target = settings.close_mm if phase == 1 else settings.open_mm
        if detected or abs(width - target) > 1.0:
            raise RuntimeError("Empty/release width did not reach target or object still detected")
    return width


def run_session(connection, settings, hooks, timeout=15.0):
    """hooks owns ROS health, arm actions, and control transitions.

On any exception the caller closes the socket and requests program stop.
Never retries a handback or grants another grip after an uncertain outcome.
"""
    wire = Wire(connection, hooks.check, timeout)
    hello = wire.receive(7)
    if hello[:6] != settings.hello or abs(hello[6] / 10 - settings.open_mm) > 1:
        raise RuntimeError("Pendant settings/initial width do not match this run")
    nonce = secrets.randbelow(2**30 - 1) + 1
    wire.send(nonce)
    hooks.wait_ready()
    results = []
    for phase in (1, 2):
        hooks.move(phase)
        hooks.wait_ready()
        hooks.hand_back()
        hooks.wait_left()
        # The pendant blocks BEFORE its RG Grip node until this is granted.
        if wire.receive(2) != (nonce, -phase):
            raise RuntimeError("Wrong pendant phase request; grip not authorized")
        wire.send(nonce, phase)
        width = validate_result(wire.receive(5), nonce, phase, settings)
        hooks.check()
        wire.send(nonce, phase)
        # Only acknowledge success once External Control AND its arm
        # controller are active again, with settled fresh joint feedback.
        hooks.wait_ready()
        results.append({"phase": phase, "width_mm": width})
        hooks.report(results[-1])
    return results
