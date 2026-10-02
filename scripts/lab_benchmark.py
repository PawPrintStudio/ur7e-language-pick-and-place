#!/usr/bin/env python3
"""Measure the system instead of demonstrating it (tasks 1.7, 2.6, 4.4).

A demo that works once proves it *can* work. These two protocols say how
often, how fast, and where it fails -- every number comes from the
orchestrator's per-stage records, so a benchmark run and a demo run are the
same code path.

``pick``
    Run the same sentence N times. With ``--area X0 Y0 X1 Y1`` each run sets
    the object down at a new random point inside that rectangle (base_link
    metres), so the next run has to find it somewhere else: twenty runs then
    need no hands on the table. Without it the object goes back where it
    was. A safety abort ends the benchmark early -- a person has to look.

``perception``
    Detection only, no motion: put the objects on the table, name them, and
    the script asks the perception node for each one and saves the annotated
    capture. Whether the green mask is on the right object is a human
    judgement: answer y/n per object, or pass ``--judge later`` and fill in
    ``correct`` in the report after looking at the pictures.

Both write a JSON report (the evidence) and a Markdown summary (the thing a
person reads) into ``docs/evidence/``.
"""
import argparse
import json
import math
import os
import random
import statistics
import sys
import time

import rclpy

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import lab_pick  # noqa: E402
from lab_jog import JogError  # noqa: E402

from ur7e_interfaces.srv import DetectObject, LocateObject  # noqa: E402
from ur7e_orchestrator.adapters import StageError  # noqa: E402

EVIDENCE = os.path.join(_HERE, '..', 'docs', 'evidence')


def located_xy(record):
    """Return the (x, y) the LOCATE stage reported in a run record, or None."""
    for stage in record.get('stages', []):
        if stage['stage'] == 'locate' and stage['outcome'] == 'ok':
            xyz = (stage.get('detail') or {}).get('pick', {}).get('xyz')
            if xyz:
                return (float(xyz[0]), float(xyz[1]))
    return None


def stage_table(records):
    """Return {stage: {n, mean, min, max}} over successful stage records."""
    durations = {}
    for record in records:
        for stage in record['stages']:
            if stage['outcome'] == 'ok':
                durations.setdefault(stage['stage'], []).append(stage['duration_s'])
    return {name: {'n': len(values), 'mean_s': round(statistics.mean(values), 2),
                   'min_s': round(min(values), 2), 'max_s': round(max(values), 2)}
            for name, values in durations.items()}


def write_report(name, report, lines):
    """Write <name>.json and <name>.md under docs/evidence; return the paths."""
    os.makedirs(EVIDENCE, exist_ok=True)
    base = os.path.join(EVIDENCE, name)
    with open(base + '.json', 'w') as stream:
        json.dump(report, stream, indent=2, default=str)
        stream.write('\n')
    with open(base + '.md', 'w') as stream:
        stream.write('\n'.join(lines) + '\n')
    return os.path.normpath(base + '.json'), os.path.normpath(base + '.md')


def command_pick(args):
    executor, adapters, orchestrator = lab_pick.build(args)
    generator = random.Random(args.seed)
    records, stopped, previous_drop = [], '', None
    try:
        for index in range(1, args.runs + 1):
            if args.area:
                x0, y0, x1, y1 = args.area
                adapters.config['drop_xy'] = (generator.uniform(min(x0, x1), max(x0, x1)),
                                              generator.uniform(min(y0, y1), max(y0, y1)))
            lab_pick.say('BENCH', {'run': index, 'of': args.runs,
                                   'drop_xy': adapters.config['drop_xy']})
            record = orchestrator.run(args.say).to_dict()
            record['drop_xy'] = adapters.config['drop_xy']
            # The previous run set the object down at a known point; how far
            # from it did the camera find the object this time? (Includes
            # whatever the object did after release: a round object rolls.)
            located = located_xy(record)
            if located and previous_drop:
                record['locate_vs_previous_drop_mm'] = round(1000 * math.dist(
                    located, previous_drop), 1)
                lab_pick.say('CHECK', {'located': [round(v, 3) for v in located],
                                       'previous_drop': [round(v, 3) for v in previous_drop],
                                       'error_mm': record['locate_vs_previous_drop_mm']})
            if record['outcome'] == 'succeeded':
                previous_drop = tuple(adapters.config['drop_xy'] or located or ())
            records.append(record)
            lab_pick.say('RESULT', {'run': index, 'outcome': record['outcome'],
                                    'reason': record['reason'],
                                    'duration_s': round(record['duration_s'], 1)})
            if record['outcome'] == 'aborted':
                stopped = f'run {index} aborted ({record["reason"]}); a person must check'
                break
            if record['outcome'] != 'succeeded' and not args.keep_going:
                # After a failure the object may not be where the next run
                # expects; --keep-going is for when someone is resetting it.
                stopped = f'run {index} {record["outcome"]} ({record["reason"]})'
                break
    finally:
        executor.destroy_node()
        executor.motion.destroy_node()
    succeeded = sum(record['outcome'] == 'succeeded' for record in records)
    reasons = {}
    for record in records:
        if record['outcome'] != 'succeeded':
            reasons[record['reason']] = reasons.get(record['reason'], 0) + 1
    rate = succeeded / len(records) if records else 0.0
    stages = stage_table(records)
    report = {
        'protocol': 'pick', 'sentence': args.say, 'requested_runs': args.runs,
        'completed_runs': len(records), 'succeeded': succeeded,
        'success_rate': round(rate, 3), 'failure_reasons': reasons,
        'stopped_early': stopped, 'executed': bool(args.execute),
        'fake_gripper': bool(args.fake_gripper), 'backend': args.backend,
        'max_speed_percent': args.max_speed_percent, 'joint_rate': args.joint_rate,
        'area': args.area, 'seed': args.seed, 'stage_seconds': stages,
        'retries': {key: sum(record['retries'].get(key, 0) for record in records)
                    for key in ('reobserve_on_not_found', 'regrasp_on_miss')},
        'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'runs': records,
    }
    errors = [record['locate_vs_previous_drop_mm'] for record in records
              if 'locate_vs_previous_drop_mm' in record]
    report['locate_vs_previous_drop_mm'] = errors
    cycle = [record['duration_s'] for record in records if record['outcome'] == 'succeeded']
    lines = [f'# Pick benchmark: "{args.say}"', '',
             f'- {succeeded}/{len(records)} runs succeeded '
             f'({rate * 100:.0f} %); {args.runs} requested.',
             f'- Executed on hardware: {bool(args.execute) and not args.fake_gripper}; '
             f'pendant ceiling {args.max_speed_percent:g} %, nominal joint rate '
             f'{args.joint_rate:g} rad/s.',
             f'- Retries used: {report["retries"]}.']
    if cycle:
        lines.append(f'- Cycle time: mean {statistics.mean(cycle):.1f} s, '
                     f'min {min(cycle):.1f} s, max {max(cycle):.1f} s.')
    if errors:
        lines.append(f'- Camera vs where the robot last set the object down: '
                     f'median {statistics.median(errors):.0f} mm, max {max(errors):.0f} mm '
                     f'over {len(errors)} runs (includes any rolling after release).')
    if reasons:
        lines.append(f'- Failures by reason: {reasons}.')
    if stopped:
        lines.append(f'- Stopped early: {stopped}.')
    lines += ['', '| Stage | n | mean s | min s | max s |', '|---|---|---|---|---|']
    lines += [f'| {name} | {row["n"]} | {row["mean_s"]} | {row["min_s"]} | {row["max_s"]} |'
              for name, row in stages.items()]
    paths = write_report(args.name or time.strftime('pick-benchmark-%Y%m%d-%H%M%S'),
                         report, lines)
    lab_pick.say('REPORT', {'json': paths[0], 'markdown': paths[1], 'succeeded': succeeded,
                            'runs': len(records), 'success_rate': round(rate, 3)})
    return 0 if records and rate >= args.accept else 1


def command_perception(args):
    from lab_jog import JogExecutor
    node = JogExecutor(plan_only=True)
    rows = []
    try:
        for query in args.objects:
            row = {'query': query, 'found': False}
            detect = node.call(DetectObject, '/perception/detect_object',
                               DetectObject.Request(query=query), timeout=90.0)
            row['latency_ms'] = round(detect.latency_ms)
            if detect.success:
                locate = node.call(LocateObject, '/perception/locate_object',
                                   LocateObject.Request(capture_id=detect.capture_id))
                row['confidence'] = round(float(detect.confidence), 3)
                if locate.success:
                    p = locate.pose.pose.position
                    row.update(found=True, xyz=[round(p.x, 4), round(p.y, 4), round(p.z, 4)],
                               snapshot=f'{detect.capture_id}.jpg')
                else:
                    row['reason'] = locate.reason
            else:
                row['reason'] = detect.reason
            if row['found'] and args.judge == 'ask':
                answer = input(f'"{query}": is the mask on the right object? [y/n] ')
                row['correct'] = answer.strip().lower() in ('y', 'yes')
            else:
                row['correct'] = None if row['found'] else False
            lab_pick.say('OBJECT', row)
            rows.append(row)
    finally:
        node.destroy_node()
        node.motion.destroy_node()
    judged = [row for row in rows if row['correct'] is not None]
    correct = sum(bool(row['correct']) for row in judged)
    report = {'protocol': 'perception', 'objects': rows, 'found': sum(r['found'] for r in rows),
              'correct': correct, 'judged': len(judged), 'total': len(rows),
              'snapshot_dir': args.snapshot_dir, 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
              'note': 'correct = a person confirmed the mask is on the named object'}
    latencies = [row['latency_ms'] for row in rows]
    lines = ['# Perception validation (single webcam)', '',
             f'- {correct}/{len(rows)} objects detected and confirmed correct '
             f'({len(rows) - len(judged)} awaiting a human judgement).',
             f'- Detection latency: median {statistics.median(latencies):.0f} ms, '
             f'max {max(latencies):.0f} ms.', '',
             '| Object | found | correct | confidence | latency ms | note |',
             '|---|---|---|---|---|---|']
    lines += [f'| {row["query"]} | {row["found"]} | {row["correct"]} | '
              f'{row.get("confidence", "")} | {row["latency_ms"]} | {row.get("reason", "")} |'
              for row in rows]
    paths = write_report(args.name or time.strftime('perception-validation-%Y%m%d-%H%M%S'),
                         report, lines)
    lab_pick.say('REPORT', {'json': paths[0], 'markdown': paths[1], 'correct': correct,
                            'total': len(rows)})
    return 0


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = cli.add_subparsers(dest='command', required=True)
    pick = commands.add_parser('pick', help='N end-to-end runs of one sentence')
    pick.add_argument('--say', required=True, metavar='TEXT')
    pick.add_argument('--runs', type=int, default=20)
    pick.add_argument('--area', type=float, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'),
                      help='rectangle (base_link m) to scatter the object in between runs')
    pick.add_argument('--seed', type=int, default=0)
    pick.add_argument('--accept', type=float, default=0.8, help='success rate to exit 0')
    pick.add_argument('--keep-going', action='store_true',
                      help='continue after a failed run (someone resets the object)')
    pick.add_argument('--name', default='', help='report file name (no extension)')
    lab_pick.add_session_arguments(pick)
    pick.set_defaults(run=command_pick)
    perception = commands.add_parser('perception', help='detection-only validation')
    perception.add_argument('objects', nargs='+', help='noun phrases, one per object')
    perception.add_argument('--judge', choices=('ask', 'later'), default='ask')
    perception.add_argument('--snapshot-dir', default='log/perception')
    perception.add_argument('--name', default='')
    perception.set_defaults(run=command_perception)
    args = cli.parse_args()

    rclpy.init()
    try:
        return args.run(args)
    except (StageError, JogError, OSError, RuntimeError) as error:
        lab_pick.say('BLOCKED', str(error))
        return 2
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
