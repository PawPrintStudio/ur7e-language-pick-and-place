#!/usr/bin/env python3
"""Ask the perception node where something is, from the command line.

    python3 scripts/lab_detect.py "blue helmet"

Prints one JSON line with the detection and the located pose (base_link,
metres), exactly what the orchestrator's DETECT and LOCATE stages receive.
"""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from ur7e_interfaces.srv import DetectObject, LocateObject


def call(node, kind, name, request, timeout=90.0):
    client = node.create_client(kind, name)
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit(f'{name} unavailable (is the perception node running?)')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise SystemExit(f'{name} timed out')
    return future.result()


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('query')
    args = cli.parse_args()
    rclpy.init()
    node = Node('lab_detect')
    try:
        detect = call(node, DetectObject, '/perception/detect_object',
                      DetectObject.Request(query=args.query))
        result = {'query': args.query, 'detected': detect.success,
                  'latency_ms': round(detect.latency_ms)}
        if not detect.success:
            result['reason'] = detect.reason
            print(json.dumps(result))
            return 1
        result.update(confidence=round(float(detect.confidence), 3),
                      capture_id=detect.capture_id,
                      bbox=[round(float(v)) for v in detect.bbox])
        locate = call(node, LocateObject, '/perception/locate_object',
                      LocateObject.Request(capture_id=detect.capture_id), timeout=30.0)
        result['located'] = locate.success
        if locate.success:
            p = locate.pose.pose.position
            result.update(xyz=[round(p.x, 4), round(p.y, 4), round(p.z, 4)],
                          axis_ratio=round(float(locate.axis_ratio), 2),
                          points=int(locate.valid_points))
        else:
            result['reason'] = locate.reason
        print(json.dumps(result))
        return 0 if locate.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
