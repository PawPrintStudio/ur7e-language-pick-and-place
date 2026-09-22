"""
Language front-end: free-form text to a validated manipulation command.

Task 3.1 (issue #21) and the parser half of task 3.3 (issue #23).

Everything in this package except :mod:`arm_language.intent_parser_node` is
plain Python with no ROS imports. That is deliberate — it means the parser can
be developed, tested, and evaluated on a laptop with no ROS installation, which
is the same reason architecture D7 exists for the motion half.

Start with :class:`arm_language.parser.IntentParser`.
"""
from .guardrails import GuardrailPolicy
from .parser import IntentParser
from .result import Command, Outcome, ParseResult, ReasonCode

__all__ = [
    'Command',
    'GuardrailPolicy',
    'IntentParser',
    'Outcome',
    'ParseResult',
    'ReasonCode',
]
