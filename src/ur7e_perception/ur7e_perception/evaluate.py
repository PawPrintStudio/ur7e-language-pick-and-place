"""Evaluate seeded geometry or labeled recorded RGB-D scenes; never mix scores."""

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from .backends import make_backend
from .core import localize, PerceptionError
from .synthetic import scene, load_frame, save_frame, CAMERA_TO_BASE, WORKSPACE


def evaluate(cases, backend, output, source):
    """Write per-case errors, actual mask IoU, and measured stage latencies."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, case in enumerate(cases):
        frame, truth, yaw, mask, query, transform, workspace = case
        start = time.perf_counter()
        row = dict(case=index, query=query, success=False)
        try:
            detection = backend.detect(frame, query)
            detected = time.perf_counter()
            grasp = localize(frame, detection, transform, workspace)
            row['detect_ms'] = (detected-start)*1000
            row['locate_ms'] = (time.perf_counter()-detected)*1000
            error = float(np.linalg.norm(grasp.position-truth))
            intersection = np.logical_and(mask, detection.mask).sum()
            union = np.logical_or(mask, detection.mask).sum()
            iou = float(intersection/max(union, 1))
            row.update(position_error_m=error, mask_iou=iou,
                       yaw_error_deg=math.degrees(abs((grasp.yaw-yaw+math.pi/2) % math.pi-math.pi/2)),
                       success=bool(error < .01 and iou >= .5),
                       position=grasp.position.tolist())
            overlay = frame.rgb.copy()
            overlay[detection.mask > 0] = (overlay[detection.mask > 0]*.5+[0, 127, 0]).astype('uint8')
            cv2.imwrite(str(output/f'case_{index:02d}.png'), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
        except PerceptionError as error:
            row['reason'] = str(error)
        rows.append(row)
    passed = sum(row['success'] for row in rows)
    report = dict(source=source, backend=type(backend).__name__, cases=rows,
                  passed=passed, total=len(rows), accepted=bool(rows) and passed/len(rows) >= .8,
                  evidence_scope='synthetic software regression' if source == 'synthetic' else 'recorded RGB-D evaluation')
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key: report[key] for key in ('source', 'backend', 'passed', 'total', 'accepted')}))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/ur7e-perception-eval')
    parser.add_argument('--manifest', help='Labeled recorded RGB-D manifest, relative paths resolved beside it')
    parser.add_argument('--backend', choices=['fixture', 'owlv2'], default='fixture')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--count', type=int, default=30)
    parser.add_argument('--export-frame', help='Export the first scene for replay')
    args = parser.parse_args()
    if args.count < 1:
        parser.error('--count must be positive')
    cases = []
    if args.manifest:
        path = Path(args.manifest)
        data = json.loads(path.read_text())
        if data.get('source') != 'recorded':
            parser.error('recorded manifest must declare source=recorded')
        for case in data['cases']:
            frame = load_frame(path.parent/case['frame'])
            mask = cv2.imread(str(path.parent/case['mask']), cv2.IMREAD_GRAYSCALE)
            if mask is None or mask.shape != frame.depth.shape:
                parser.error('ground-truth mask missing or wrong size')
            cases.append((frame, np.array(case['position_base_m']), case['yaw_rad'], mask > 0,
                          case['query'], np.array(data['camera_to_base']), data['workspace']))
    else:
        for i in range(args.count):
            color = ['red', 'green', 'blue'][i % 3]
            frame, point, yaw, mask = scene(i, color=color)
            cases.append((frame, point, yaw, mask, f'{color} block', CAMERA_TO_BASE, WORKSPACE))
    if not cases:
        parser.error('evaluation must contain cases')
    if args.export_frame:
        save_frame(args.export_frame, cases[0][0])
    kwargs = {'device': args.device} if args.backend == 'owlv2' else {}
    report = evaluate(cases, make_backend(args.backend, **kwargs), args.output,
                      'recorded' if args.manifest else 'synthetic')
    raise SystemExit(0 if report['accepted'] else 1)


if __name__ == '__main__':
    main()
