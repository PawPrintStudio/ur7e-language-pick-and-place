"""Render what a webcam would see of the table board and a few boxes.

Used by the monocular tests and by ``scripts/lab_pick_rehearsal.sh``: the
geometry is known exactly, so the same code that will run on the lab webcam
can be checked to the millimetre without a camera, a robot, or a printer.
"""
import argparse
import json
import math

import cv2
import numpy as np

from . import monocular as mono

SIZE = (1280, 720)
FOCAL = 950.0


def look_at(position, target):
    """Camera-to-base transform: optical z toward target, x right, y down."""
    forward = np.asarray(target, float) - np.asarray(position, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((right, down, forward))
    transform[:3, 3] = position
    return transform


def board_in_base(x=-0.45, y=-0.10, z=0.012, yaw=math.radians(30)):
    """Return a base_from_board transform for a board lying flat on the table."""
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(np.array([0, 0, float(yaw)]))[0]
    transform[:3, 3] = [x, y, z]
    return transform


def cuboid(center_xy, size, yaw, table_z):
    """Return the 8 vertices of a box standing on the table (base frame)."""
    sx, sy, sz = size
    corners = np.array([[dx * sx / 2, dy * sy / 2, dz * sz]
                        for dx in (-1, 1) for dy in (-1, 1) for dz in (0, 1)])
    rotation = cv2.Rodrigues(np.array([0, 0, float(yaw)]))[0]
    return corners @ rotation.T + [center_xy[0], center_xy[1], table_z]


def render(cam_to_base, base_from_board, boxes=(), focal=FOCAL, size=SIZE):
    """Return (rgb image, [mask per box]) as the camera would see the scene.

    ``boxes`` is a list of (vertices from :func:`cuboid`, (r, g, b)).
    """
    k = mono.intrinsics(focal, size)
    board = mono.table_board()
    drawn = board.draw((5 * 120, 7 * 120), marginSize=0, borderBits=1)
    pixels, xyz = mono.detect_board(cv2.cvtColor(drawn, cv2.COLOR_GRAY2RGB), board)
    to_drawn, _ = cv2.findHomography(xyz[:, :2], pixels)
    square = mono.touch_points()
    image_points = mono.project(square @ base_from_board[:3, :3].T + base_from_board[:3, 3],
                                k, cam_to_base)
    to_image = cv2.getPerspectiveTransform(square[:, :2].astype(np.float32),
                                           image_points.astype(np.float32))
    warp = to_image @ np.linalg.inv(to_drawn)
    image = np.full((size[1], size[0], 3), 170, np.uint8)
    paper = cv2.warpPerspective(cv2.cvtColor(drawn, cv2.COLOR_GRAY2RGB), warp, size)
    covered = cv2.warpPerspective(np.full(drawn.shape, 255, np.uint8), warp, size) > 0
    image[covered] = paper[covered]
    masks = []
    for vertices, color in boxes:
        hull = cv2.convexHull(mono.project(vertices, k, cam_to_base).astype(np.int32))
        mask = np.zeros((size[1], size[0]), np.uint8)
        cv2.fillConvexPoly(mask, hull, 1)
        image[mask.astype(bool)] = color
        masks.append(mask)
    return image, masks


def main():
    """Write a rehearsal scene: an image plus the matching (synthetic) calibration."""
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('image')
    cli.add_argument('calibration')
    cli.add_argument('--red', type=float, nargs=2, default=(-0.42, 0.22))
    cli.add_argument('--blue', type=float, nargs=2, default=(-0.50, 0.02))
    args = cli.parse_args()
    anchor = board_in_base(x=-0.62, y=-0.28, z=0.0)
    camera = look_at([-0.45, -0.95, 0.60], [-0.45, 0.0, 0.0])
    boxes = [(cuboid(args.red, (0.04, 0.04, 0.04), 0.0, 0.0), (200, 30, 30)),
             (cuboid(args.blue, (0.04, 0.04, 0.04), 0.0, 0.0), (30, 60, 200))]
    image, _ = render(camera, anchor, boxes)
    cv2.imwrite(args.image, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    touches = {str(n): {'tool_length': 0.20} for n in (1, 2, 3, 4)}
    mono.TableCalibration(anchor, focal_px=FOCAL, source='synthetic',
                          notes={'touches': touches}).save(args.calibration)
    print(json.dumps({'image': args.image, 'calibration': args.calibration,
                      'red_block_xy': list(args.red), 'blue_block_xy': list(args.blue)}))


if __name__ == '__main__':
    main()
