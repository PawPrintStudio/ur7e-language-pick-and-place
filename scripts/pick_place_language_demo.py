#!/usr/bin/env python3
"""Free text -> intent_parser -> Gazebo pick, task 3.1/3.3's integration driver.

Replaces `perception_pick_demo.py`'s throwaway `pick <noun phrase>` grammar
with the real `arm_language` pipeline (backend -> validator -> guardrails), so
a sentence like "could you grab the red block" reaches the same
observe/detect/locate/plan/grasp/lift sequence that script already proves.
`perception_pick_demo.py` is left untouched: its grammar is still useful as a
grammar-free smoke test, and the two scripts are meant to stay separate rather
than grow one script that does both jobs.

`pick_and_place` is understood by the parser but not by this demo -- there is
no place stage below `run_query`'s LIFT -- so the guardrail policy here
narrows `allowed_actions` to `('pick',)` only. That is the documented,
intended use of the narrowing knob (see `arm_language/guardrails.py`), not a
workaround.
"""
import argparse
import sys

import rclpy

from arm_language import schema
from arm_language.backends import BackendError, available, create
from arm_language.guardrails import GuardrailPolicy
from arm_language.parser import IntentParser
from arm_language.result import Outcome

from perception_pick_demo import PerceptionPick


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('text', help='Free-form request, e.g. "pick up the red block"')
    parser.add_argument('--execute-simulation', action='store_true', required=True)
    parser.add_argument('--backend', default='keyword', choices=available(),
                        help='Intent-parser backend (default: keyword -- offline, no API key)')
    parser.add_argument('--model', default='', help='Backend model override, if any')
    parser.add_argument('--yes', action='store_true',
                        help='Auto-confirm a needs_confirmation outcome instead of prompting')
    parser.add_argument('--require-confirmation', action='store_true',
                        help='Ask before every move, however confident (GuardrailPolicy'
                             '.require_confirmation) -- the recommended setting for a live demo')
    parser.add_argument('--stage-pause', type=float, default=0.0,
                        help='Narrated pause before each motion stage, in seconds (0-5)')
    args = parser.parse_args()
    if not 0 <= args.stage_pause <= 5:
        parser.error('--stage-pause must be between 0 and 5 seconds')

    try:
        backend = create(args.backend, **({'model': args.model} if args.model else {}))
    except BackendError as error:
        parser.error(f'could not start backend "{args.backend}": {error}')

    # `pick_and_place` is a real, validated outcome the parser can produce --
    # it is refused here, by policy, because this demo has nowhere to send a
    # place_target. See the module docstring.
    policy = GuardrailPolicy(allowed_actions=(schema.ACTION_PICK,),
                             require_confirmation=args.require_confirmation)
    result = IntentParser(backend, policy).parse(args.text)
    print(f'PARSE {result.outcome.value}/{result.reason_code.value}: {result.message}')

    if result.outcome is Outcome.REFUSED:
        return 1

    if result.outcome is Outcome.NEEDS_CONFIRMATION and not args.yes:
        reply = input('Proceed? [y/N]: ').strip().lower()
        if reply != 'y':
            print('PARSE not confirmed -- no motion')
            return 1

    query = result.command.target_query
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = PerceptionPick()
    try:
        node.run_query(query, args.stage_pause)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
