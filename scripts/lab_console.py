#!/usr/bin/env python3
"""Natural-language command console for the arm-only UR7e (task 3.2, issue #22).

Type a sentence; the arm moves -- or explains why it will not.

    could you go up a bit?        -> move the tool up by 2 cm
    go down 2                     -> move the tool down by 2 cm
    can you spin slowly?          -> rotate the wrist 30 degrees, speed +1
    spin at speed -1              -> ... clockwise, speed -1
    go home                       -> the pose the console started in
    go to the home pose but 3 cm up  -> deterministic pose + offset
    pick up the hammer            -> refused: no camera in this session
    go up one metre               -> refused: over the 20 cm bound
    let me drive it myself        -> keyboard teleop in this terminal, q returns

Three layers, each in its own file, each testable without the others:

1. ``arm_language`` parses the text into a *validated, bounded* command or a
   refusal (backend -> validator -> guardrails). The console never sees raw
   model output.
2. ``lab_jog.JogExecutor`` plans the command with MoveIt (collision-checked
   against the lab table) and executes it through the proven
   ``MotionClient`` path with its safety envelope.
3. This file: argument parsing, the confirmation prompt, and printing.

Rehearse first, move second::

    python3 scripts/lab_console.py                       # plan-only, offline parser
    python3 scripts/lab_console.py --execute             # real motion (needs Play)
    python3 scripts/lab_console.py --execute --backend claude   # the LLM front end

Prerequisites (two other terminals):
    ros2 launch ur7e_bringup ur7e_bringup.launch.py
    ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl   # KDL: see that file

Start posture matters. From the cold folded resting pose the wrist sits on the
shoulder singularity: up/down/left/forward plan, "right" cannot (physics, not
a bug). ``go to ready`` (a seed pose in ``lab_poses.json``, all six directions
reachable) needs ``--max-excursion 2.0`` because it is a large, deliberate
move; run it at a low pendant slider with eyes on the arm, then ``/teach home``.
"""
import argparse
import json
import os
import select
import subprocess
import sys

import rclpy

from arm_language import schema
from arm_language.backends import BackendError, available, create
from arm_language.guardrails import GuardrailPolicy
from arm_language.parser import IntentParser
from arm_language.result import Outcome

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lab_jog import JogError, JogExecutor  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
# Committed seed poses (e.g. `ready`), then the per-session taught poses.
SEED_POSES_FILE = os.path.join(_HERE, 'lab_poses.json')
DEFAULT_POSES_FILE = os.path.join(_HERE, '.console_poses.json')

HELP = """\
Say what you want in plain English ("go up a bit", "spin slowly", "go home",
"let me drive it myself" for keyboard teleop), or use a slash command:
  /where          tool position (m, base frame) and joint angles
  /teach NAME     remember the current pose as NAME ("go to NAME" later)
  /poses          list taught poses
  /help           this text
  /quit           leave (Ctrl-D works too)
"""


def say(tag, payload):
    """One line per stage, machine-readable, so a run log explains itself."""
    if isinstance(payload, str):
        print(f'{tag:8} {payload}', flush=True)
    else:
        print(f'{tag:8} {json.dumps(payload, default=str)}', flush=True)


def confirm(prompt, auto_yes):
    if auto_yes:
        say('CONFIRM', 'auto-yes')
        return True
    try:
        answer = input(f'{prompt} [y/N] ').strip().lower()
    except EOFError:
        return False
    return answer in ('y', 'yes')


TELEOP_SCRIPT = os.path.join(_HERE, 'motion', 'teleop_keyboard.py')
DEFAULT_VOICE_INBOX = os.path.join(_HERE, '.voice_inbox.txt')


class VoiceInbox:
    """Tail a file that scripts/voice_input.py appends transcripts to.

    Starts at the current end of the file, so sentences spoken before the
    console started are never replayed into a live robot.
    """

    def __init__(self, path):
        self.path = path
        open(path, 'a').close()
        self.offset = os.path.getsize(path)
        self.pending = []

    def poll(self):
        """Return the next complete new line, or None."""
        if self.pending:
            return self.pending.pop(0)
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return None
        if size < self.offset:
            self.offset = 0  # file was truncated/replaced
        if size == self.offset:
            return None
        with open(self.path) as handle:
            handle.seek(self.offset)
            chunk = handle.read()
        lines = chunk.split('\n')
        complete, tail = lines[:-1], lines[-1]
        self.offset = size - len(tail.encode())
        self.pending.extend(line.strip() for line in complete if line.strip())
        return self.pending.pop(0) if self.pending else None


def read_line(inbox):
    """Return (text, spoken): the next typed line, or a voice transcript.

    Waits on the terminal with ``select`` so a spoken sentence can arrive
    while the prompt is showing. Without an inbox this is plain ``input``.
    """
    if inbox is None:
        return input('arm> ').strip(), False
    sys.stdout.write('arm> ')
    sys.stdout.flush()
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.2)
        if ready:
            line = sys.stdin.readline()
            if not line:
                raise EOFError
            return line.strip(), False
        spoken = inbox.poll()
        if spoken is not None:
            sys.stdout.write('\n')
            return spoken, True


def hand_over(result, executor, args):
    """Run keyboard teleop in this terminal, then take the console back.

    Teleop is its own process on purpose: it owns the raw tty and its own
    ROS node, and it goes through the same ``MotionClient`` envelope as
    everything else. The console only checks the gates first, so the
    hand-over is refused for the same reasons a move would be, and reports
    where the arm ended up afterwards.
    """
    if executor.plan_only:
        say('PLAN', {'stage': 'teleop', 'executed': False,
                     'note': 'would hand over to keyboard teleop; plan-only run'})
        return True
    try:
        executor.gate()
        before, _ = executor.where()
    except JogError as error:
        say('BLOCKED', str(error))
        return False
    if not sys.stdin.isatty():
        say('BLOCKED', 'teleop needs an interactive terminal (a raw tty); '
                       'run the console without --say to use it')
        return False
    if result.outcome is Outcome.NEEDS_CONFIRMATION:
        if not confirm(result.message + ' Keys: 1-6 pick a joint, . and , jog it, '
                       '[ ] halve/double the step, h back to where teleop started, '
                       'q returns here.', args.yes):
            say('SKIPPED', 'not confirmed')
            return False
    say('TELEOP', 'yours. q to come back to the console.')
    completed = subprocess.run([sys.executable, TELEOP_SCRIPT])
    say('TELEOP', {'exit_code': completed.returncode})
    try:
        after, _ = executor.where()
        delta = [round((a - b) * 1000, 1) for a, b in zip(after, before)]
        say('MEASURE', {'tool_delta_mm_xyz': delta})
    except JogError as error:
        say('WAIT', str(error))
    return completed.returncode == 0


def run_pick(text, result, pick, args):
    """Hand a pick sentence to the orchestrator (camera sessions, --pick).

    The console has already parsed the sentence and shown the echo; the
    orchestrator runs the fixed stage sequence from there and prints each
    stage as it ends. The typed confirmation happens here, on the main
    thread, so the orchestrator's PARSE stage never waits on a keyboard.
    """
    adapters, orchestrator = pick
    if result.outcome is Outcome.NEEDS_CONFIRMATION:
        if not confirm(result.message, args.yes):
            say('SKIPPED', 'not confirmed')
            return False
    adapters.remember(text, result, confirmed=True)
    record = orchestrator.run(text)
    say('RESULT', {'run_id': record.run_id, 'outcome': record.outcome,
                   'reason': record.reason, 'message': record.message,
                   'duration_s': round(record.duration_s, 1), 'retries': record.retries})
    return record.outcome == 'succeeded'


def handle(text, parser, executor, args, pick=None):
    """Parse one utterance and, if allowed and confirmed, run it."""
    result = parser.parse(text)
    diag = {k: v for k, v in result.detail.items() if k != 'raw_response'}
    say('PARSE', {'outcome': result.outcome.value, 'reason': result.reason_code.value,
                  'backend': diag.get('backend'), 'latency_ms': diag.get('latency_ms'),
                  'confidence': diag.get('confidence')})
    if result.outcome is Outcome.REFUSED:
        say('REFUSED', result.message)
        return False

    command = result.command
    say('COMMAND', command.echo())

    if command.action == schema.ACTION_TELEOP:
        return hand_over(result, executor, args)
    if command.action in schema.MOTION_ACTIONS:
        return run_pick(text, result, pick, args)

    # Plan first, ask second: the person confirming sees what the plan
    # actually does (joint swing, duration), not only the parsed sentence.
    try:
        preview = executor.run_command(command, plan_only=True)
    except JogError as error:
        say('BLOCKED', str(error))
        return False
    for stage in preview['stages']:
        say('PLAN', stage)
    if executor.plan_only:
        return True

    if result.outcome is Outcome.NEEDS_CONFIRMATION:
        swing = max(stage['max_joint_excursion_rad'] for stage in preview['stages'])
        seconds = sum(stage['nominal_seconds'] for stage in preview['stages'])
        prompt = (f'{result.message} Plan: {swing:.2f} rad max joint swing, '
                  f'~{seconds:.0f} s nominal before the pendant slider.')
        if not confirm(prompt, args.yes):
            say('SKIPPED', 'not confirmed')
            return False

    try:
        report = executor.run_command(command)
    except JogError as error:
        say('BLOCKED', str(error))
        return False
    for stage in report['stages']:
        say('EXEC', stage)
    say('MEASURE', {'tool_delta_mm_xyz': report['measured_delta_mm']})
    return True


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--backend', default='keyword', choices=available(),
                     help='intent-parser backend (keyword = offline rules; claude = LLM)')
    cli.add_argument('--model', default='', help='backend model override')
    cli.add_argument('--execute', action='store_true',
                     help='really move the arm (default: plan only and report)')
    cli.add_argument('--max-speed-percent', type=float, default=50.0,
                     help='refuse to move if the pendant slider is above this')
    cli.add_argument('--joint-rate', type=float, default=0.05,
                     help='nominal rad/s for moves and go_to (before the pendant slider)')
    # 1.0 rad: a 5 cm jog near the folded pose's singularity already costs
    # 0.63 rad of shoulder_pan (measured 2026-09-30), and "go home" after it
    # must not be refused; the folded-to-ready swing (1.85 rad) still is.
    cli.add_argument('--max-excursion', type=float, default=1.0,
                     help='largest single-joint change a go_to may ask for, rad')
    cli.add_argument('--mirror-lr', action='store_true',
                     help="swap left/right so they match an audience facing the robot")
    cli.add_argument('--yes', action='store_true',
                     help='auto-confirm instead of prompting (scripted runs only)')
    cli.add_argument('--no-confirm', action='store_true',
                     help='do not ask before confident commands (default: always ask)')
    cli.add_argument('--poses-file', default=DEFAULT_POSES_FILE,
                     help='where taught poses are stored between runs')
    cli.add_argument('--forget-poses', action='store_true',
                     help='ignore poses stored from an earlier run')
    cli.add_argument('--say', action='append', default=[], metavar='TEXT',
                     help='run this sentence and exit (repeatable; no REPL)')
    cli.add_argument('--voice-inbox', metavar='PATH', nargs='?',
                     const=DEFAULT_VOICE_INBOX, default=None,
                     help='also take sentences from this file, one per line, as '
                          'scripts/voice_input.py appends them (default path if '
                          'given without a value)')
    cli.add_argument('--pick', action='store_true',
                     help='camera + gripper session: also accept "pick up the ..." and '
                          'run it through the orchestrator (needs the webcam perception '
                          'node and a table calibration; see scripts/lab_pick.py)')
    cli.add_argument('--calibration', default=os.path.join(_HERE, 'lab_table.json'),
                     help='table calibration from scripts/lab_table_calibration.py')
    cli.add_argument('--robot-ip', default='192.168.56.101')
    cli.add_argument('--drop-xy', type=float, nargs=2, metavar=('X', 'Y'),
                     help='where a plain "pick" sets the object down (default: in place)')
    cli.add_argument('--log-dir', default=os.path.join(_HERE, '..', 'log', 'runs'),
                     help='one JSONL file per pick run')
    cli.add_argument('--open-mm', type=float, default=80.0, help='jaw opening before a grasp')
    cli.add_argument('--force', type=float, default=20.0, help='grip force, N')
    cli.add_argument('--tip-clearance', type=float, default=0.008,
                     help='lowest fingertip height above the table when grasping, m')
    cli.add_argument('--hover', type=float, default=0.12,
                     help='fingertip height above the object while travelling, m')
    args = cli.parse_args()
    if args.pick and args.max_excursion < 2.5:
        # A pick run is a sequence of large, planned moves (observe -> hover ->
        # place); the jog cap is sized for single nudges.
        args.max_excursion = 2.5

    try:
        backend = create(args.backend, **({'model': args.model} if args.model else {}))
    except BackendError as error:
        cli.error(f'could not start backend "{args.backend}": {error}')

    # Camera-free session: only the jog actions are enabled. A pick request is
    # parsed fine and refused by *policy* -- which is the point worth showing.
    # With --pick (webcam + gripper session) the pick actions are added and
    # go to the orchestrator; the policy is the only thing that changed.
    allowed = schema.JOG_ACTIONS + (schema.MOTION_ACTIONS if args.pick else ())
    policy = GuardrailPolicy(allowed_actions=allowed,
                             require_confirmation=not args.no_confirm)
    parser = IntentParser(backend, policy)

    rclpy.init()
    pick = None
    if args.pick:
        import lab_pick
        from ur7e_orchestrator.workflow import Orchestrator
        from ur7e_perception.monocular import TableCalibration
        executor = lab_pick.PickExecutor(
            TableCalibration.load(args.calibration), hover=args.hover,
            max_speed_percent=args.max_speed_percent, joint_rate=args.joint_rate,
            max_excursion=args.max_excursion, mirror_lr=args.mirror_lr,
            plan_only=not args.execute, poses_file=args.poses_file)
    else:
        executor = JogExecutor(max_speed_percent=args.max_speed_percent,
                               joint_rate=args.joint_rate, max_excursion=args.max_excursion,
                               mirror_lr=args.mirror_lr, plan_only=not args.execute,
                               poses_file=args.poses_file)
    try:
        executor.load_poses(SEED_POSES_FILE)
        if not args.forget_poses:
            executor.load_poses()
        # "home" is wherever the arm is when the console starts: deterministic
        # for the session, and never a typed absolute target. The arm may
        # still be settling from a previous command, so be patient here.
        # A restarted console keeps the `home` it taught earlier (it is in
        # the session file), so "go home" still means the pose the session
        # started in, not wherever the arm happened to stop. --forget-poses
        # or `/teach home` re-teaches it here and now.
        for attempt in range(10):
            try:
                if 'home' in executor.poses:
                    home = executor.poses['home']
                    say('HOME', 'kept from the session file; /teach home to redefine')
                else:
                    home = executor.teach('home')
                position, _ = executor.where()
                break
            except JogError as error:
                say('WAIT', str(error))
        else:
            say('BLOCKED', 'could not read a stationary arm; is the driver up?')
            return 2
        if args.pick:
            config = dict(lab_pick.DEFAULT_CONFIG)
            if 'observe' not in executor.poses:
                config['observe_pose'] = config['home_pose'] = 'ready'
            if args.drop_xy:
                config['drop_xy'] = tuple(args.drop_xy)
            config.update(open_mm=args.open_mm, force_n=args.force,
                          min_tip_clearance=args.tip_clearance)
            gripper = lab_pick.Rg2(args.robot_ip) if args.execute else lab_pick.FakeRg2()
            adapters = lab_pick.LabAdapters(executor, gripper, parser, args.robot_ip,
                                            config, notify=say)
            pick = (adapters, Orchestrator(adapters, log_dir=args.log_dir,
                                           on_event=lab_pick.on_event))
        say('READY', {'mode': 'EXECUTE' if args.execute else 'PLAN-ONLY',
                      'pick': bool(args.pick),
                      'backend': args.backend, 'confirm': not args.no_confirm,
                      'max_speed_percent': args.max_speed_percent,
                      'tool_xyz_m': [round(v, 4) for v in position],
                      'home_joints': {k: round(v, 3) for k, v in home.items()},
                      'poses': sorted(executor.poses)})

        if args.say:
            failures = 0
            for text in args.say:
                say('SAY', text)
                if not handle(text, parser, executor, args, pick):
                    failures += 1
            return 1 if failures else 0

        print(HELP)
        inbox = VoiceInbox(args.voice_inbox) if args.voice_inbox else None
        if inbox:
            say('VOICE', f'listening on {args.voice_inbox} '
                         f'(run scripts/voice_input.py on the laptop)')
        while True:
            try:
                text, spoken = read_line(inbox)
            except (EOFError, KeyboardInterrupt):
                # Ctrl-D or Ctrl-C at the prompt: leave quietly. (rclpy's own
                # SIGINT handler has already shut the context down by now, so
                # the finally below must not shut it down a second time.)
                print()
                break
            if spoken:
                say('VOICE', text)
            if not text:
                continue
            if text in ('/quit', '/exit', 'quit', 'exit'):
                break
            if text == '/help':
                print(HELP)
            elif text == '/where':
                try:
                    position, joints = executor.where()
                    say('WHERE', {'tool_xyz_m': [round(v, 4) for v in position],
                                  'joints': {k: round(v, 3) for k, v in joints.items()}})
                except JogError as error:
                    say('BLOCKED', str(error))
            elif text.startswith('/teach'):
                parts = text.split(maxsplit=1)
                if len(parts) != 2 or not parts[1].replace(' ', '').isalnum():
                    say('USAGE', '/teach NAME')
                    continue
                try:
                    executor.teach(parts[1].lower())
                    say('TAUGHT', {'pose': parts[1].lower(), 'poses': sorted(executor.poses)})
                except JogError as error:
                    say('BLOCKED', str(error))
            elif text == '/poses':
                say('POSES', {name: {k: round(v, 3) for k, v in joints.items()}
                              for name, joints in executor.poses.items()})
            elif text.startswith('/'):
                say('USAGE', f'unknown command {text.split()[0]}; try /help')
            else:
                handle(text, parser, executor, args, pick)
        return 0
    finally:
        executor.destroy_node()
        executor.motion.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
