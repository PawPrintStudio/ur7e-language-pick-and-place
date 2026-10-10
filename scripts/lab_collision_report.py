#!/usr/bin/env python3
"""Say exactly why MoveIt calls the arm's current pose a collision. Read-only.

Asks ``/check_state_validity`` about the live joint state, with the lab table
placed where ``lab_console --pick`` puts it (the calibrated board plane), and
prints every contact pair with its depth, plus where the RG2 TCP and the
table are. Nothing is sent to the arm.

  python3 scripts/lab_collision_report.py
"""
import json
import os
import sys

import rclpy
from moveit_msgs.srv import GetStateValidity

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import lab_pick  # noqa: E402
from ur7e_perception.monocular import TableCalibration  # noqa: E402


def main():
    rclpy.init()
    ex = lab_pick.PickExecutor(TableCalibration.load(os.path.join(_HERE, 'lab_table.json')),
                               plan_only=True, tool_link=lab_pick.PICK_LINK)
    try:
        ex.ensure_table()
        state = ex.fresh(settle_s=0.3)
        response = ex.call(GetStateValidity, '/check_state_validity',
                           GetStateValidity.Request(robot_state=state, group_name=ex.group))
        tcp = ex.fk(state).position
        pairs = {}
        for c in response.contacts:
            key = tuple(sorted((c.contact_body_1, c.contact_body_2)))
            pairs[key] = max(pairs.get(key, 0.0), c.depth)
        print(json.dumps({
            'valid': response.valid,
            'contacts': [{'between': list(k), 'depth_mm': round(d * 1000, 1)}
                         for k, d in sorted(pairs.items(), key=lambda kv: -kv[1])],
            'rg2_tcp_xyz_m': [round(tcp.x, 4), round(tcp.y, 4), round(tcp.z, 4)],
            'table_top_z_m_in_model': round(ex.table_z(tcp.x, tcp.y) - 0.005, 4),
            'joints_deg': {j.replace('_joint', ''): round(v * 57.29578, 1)
                           for j, v in ex.joints_of(state).items()},
        }, indent=2))
        return 0 if response.valid else 1
    finally:
        ex.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
