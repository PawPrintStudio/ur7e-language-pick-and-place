#!/usr/bin/env python3
"""Bounded RG2 bench check using the installed OnRobot 6.5.0 XML-RPC API.

Read-only by default. This is a lab diagnostic, not a ROS controller or a
safety-rated interlock. An operator must supervise every commanded movement.
The API and tool index were verified against a pendant-exported URScript.
"""
import argparse
import json
import math
import socket
import time
import xmlrpc.client


class MotionCanceled(Exception):
    """Operator/client requested cancellation; stop is confirmed before return."""


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.0
        return connection


def dashboard_gate(host):
    with socket.create_connection((host, 29999), timeout=2) as sock:
        stream = sock.makefile("rb")
        stream.readline()
        for command, expected in (
            ("running", "Program running: false"),
            ("safetystatus", "Safetystatus: NORMAL"),
            ("robotmode", "Robotmode: RUNNING"),
        ):
            sock.sendall((command + "\n").encode())
            actual = stream.readline().decode().strip()
            if actual != expected:
                raise RuntimeError(f"Dashboard gate: {actual!r}; expected {expected!r}")


def validate_state(state):
    width = state["width"]
    if not math.isfinite(width) or not 0 <= width <= 110:
        raise RuntimeError(f"Invalid RG2 width: {width}")
    # Native preamble ignores the two low busy/grip status bits (shift = 4).
    if state["status"] & ~3 or any(state[key] for key in (
        "safety_failed", "s1_pushed", "s1_triggered", "s2_pushed", "s2_triggered"
    )):
        raise RuntimeError(f"RG2 safety/error state: {state}")
    return state


def identify(rpc, tool, serial):
    devices = json.loads(rpc.get_discovery())["devices"]
    matches = [d for d in devices if d["deviceId"] == tool]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one device at index {tool}: {devices}")
    device = matches[0]
    if (device["deviceName"] != "RG2" or device["productCode"] != 32
            or str(device["serial"]) != serial
            or any(device[k] for k in ("status", "warning", "error", "update"))):
        raise RuntimeError(f"RG2 identity/health mismatch: {device}")
    return device


def find_by_serial(rpc, serial):
    """Return the healthy RG2 with this serial, at whatever index the URCap gave it.

    The URCap numbers its devices at discovery, and the number is not
    stable: the same gripper was index 2 on 2026-10-02 and index 1 on
    2026-10-06. The serial is the gripper's identity; the index is only
    where to address it today.
    """
    devices = json.loads(rpc.get_discovery())["devices"]
    matches = [d for d in devices if str(d["serial"]) == serial]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one device with serial {serial}: {devices}")
    return identify(rpc, matches[0]["deviceId"], serial)


def move(rpc, host, tool, target, force, feedback=None, canceled=None):
    dashboard_gate(host)
    before = validate_state(rpc.rg_get_all_variables(tool))
    if before["busy"]:
        raise RuntimeError("RG2 already busy")
    if (not math.isfinite(target) or not 0 <= target <= 110
            or abs(target - before["width"]) > 20 or not 0 < force <= 10):
        raise ValueError("Lab bounds: width 0..110 mm, step <=20 mm, force <=10 N")
    start = time.monotonic()
    observed_busy = False
    stable = 0
    try:
        if canceled and canceled():
            raise MotionCanceled()
        result = rpc.rg_grip(tool, float(target), float(force))
        if result != 0:
            raise RuntimeError(f"Native rg_grip rejected command: {result}")
        while time.monotonic() - start < 8:
            if canceled and canceled():
                raise MotionCanceled()
            dashboard_gate(host)
            state = validate_state(rpc.rg_get_all_variables(tool))
            if feedback:
                feedback(state)
            observed_busy |= state["busy"]
            stable = stable + 1 if not state["busy"] and abs(state["width"] - target) <= 1 else 0
            if stable >= 3:
                report = {"event": "move_complete", "target_mm": target,
                          "requested_force_n": force, "before_mm": before["width"],
                          "actual_mm": state["width"], "observed_busy": observed_busy,
                          "operation_counter": state["operation_counter"],
                          "elapsed_s": round(time.monotonic() - start, 3)}
                print(json.dumps(report), flush=True)
                return state
            time.sleep(0.1)
        raise TimeoutError("RG2 did not reach target within 8 seconds")
    except BaseException:
        try:
            result = rpc.rg_stop(tool)
            if result != 0:
                raise RuntimeError(f"rg_stop returned {result}")
            deadline = time.monotonic() + 2
            while rpc.rg_get_all_variables(tool)["busy"]:
                if time.monotonic() >= deadline:
                    raise TimeoutError("gripper remains busy after stop")
                time.sleep(0.1)
            print(json.dumps({"event": "stop_confirmed", "result": result}), flush=True)
        except Exception as stop_error:
            print(json.dumps({"event": "stop_unconfirmed", "error": str(stop_error)}), flush=True)
            raise RuntimeError("Gripper stop could not be confirmed") from stop_error
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.56.101")
    parser.add_argument("--tool-index", type=int, default=2)
    parser.add_argument("--serial", default="1000042561")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--empty-supervised", action="store_true")
    args = parser.parse_args()
    rpc = xmlrpc.client.ServerProxy(f"http://{args.host}:41414/", transport=TimeoutTransport())
    device = identify(rpc, args.tool_index, args.serial)
    state = validate_state(rpc.rg_get_all_variables(args.tool_index))
    print(json.dumps({"event": "read", "device": device, "state": state}), flush=True)
    if args.demo:
        if not args.empty_supervised:
            parser.error("--demo requires --empty-supervised with operator at pendant")
        if state["busy"] or abs(state["width"] - 59) > 1:
            raise RuntimeError("Expected idle gripper near the operator-confirmed 59 mm")
        initial = state["width"]
        move(rpc, args.host, args.tool_index, 70, 10)
        time.sleep(1)
        move(rpc, args.host, args.tool_index, initial, 10)


if __name__ == "__main__":
    main()
