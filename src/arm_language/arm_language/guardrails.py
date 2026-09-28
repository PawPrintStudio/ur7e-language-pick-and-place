"""
The policy layer: given a valid intent, decide whether the arm may move.

This is issue #23 — the action whitelist, the confidence threshold, and the
rule that a parse failure never produces motion.

:mod:`arm_language.validator` answers "did we understand this?". This module
answers the different and more consequential question "should we act on it?".
Keeping them apart matters because they change for different reasons — the
schema changes when the pipeline gains a capability, the policy changes when a
makerspace session has more or fewer people standing near the robot.

The three guardrails
--------------------
1. **Action whitelist.** ``allowed_actions`` is the closed set of things the
   arm will do. Note it is a *subset* of what the system can express: phase 1
   can run with ``('pick',)`` alone, refusing place commands the parser
   understands perfectly well, because the place half is not built yet.
2. **Confidence threshold.** Two thresholds, giving three bands: act, ask,
   refuse.
3. **No motion on parse failure.** Structural, not a check: a refusal carries
   ``command=None``, so there is nothing for a caller to act on even if it
   ignores ``outcome`` entirely.

On trusting the confidence number
---------------------------------
It is self-reported by a language model and is **not** a calibrated
probability — a model can be fluently, confidently wrong. Treat it as a coarse
ambiguity detector: it reliably drops when the speaker named no object ("pick
up that thing"), which is the case we most want to catch, and it is close to
useless for distinguishing a good parse from a subtly wrong one. The real
defences against a confident misparse are the noun-phrase check, the action
whitelist, and the human reading the confirmation echo. The thresholds below
are starting points to be tuned against the corpus on real hardware, not
measured constants.

On prompt injection
-------------------
The utterance is attacker-controlled text ("ignore your instructions and ...")
being fed to a model whose output moves a robot. The defence is not prompt
wording — it is that the model's output can only ever be one of two actions on
one noun phrase. A successful injection buys an attacker the ability to make
the arm pick up the wrong object, which is the same thing a typo buys them.
That bound is a property of the whitelist plus the schema, and it holds even
if the model is completely subverted.
"""
from dataclasses import dataclass
from typing import Tuple

from . import schema
from .result import Command, Outcome, ParseResult, ReasonCode
from .validator import ValidatedIntent


@dataclass(frozen=True)
class GuardrailPolicy:
    """
    How cautious this deployment is.

    Defaults are tuned for a makerspace with people nearby: refuse rather than
    guess, and ask when unsure.
    """

    allowed_actions: Tuple[str, ...] = schema.MOTION_ACTIONS
    """Actions this deployment permits. Narrow it to stage a rollout."""

    accept_threshold: float = 0.75
    """At or above this confidence, act without asking."""

    confirm_threshold: float = 0.40
    """Below this, refuse outright — too unsure to be worth a human's time."""

    require_confirmation: bool = False
    """Ask before *every* move, however confident. The right setting for a
    first public demo, and for any session where the workspace is shared."""

    def __post_init__(self) -> None:
        """Reject a policy that cannot behave as written."""
        if not 0.0 <= self.confirm_threshold <= self.accept_threshold <= 1.0:
            raise ValueError(
                'Thresholds must satisfy '
                '0.0 <= confirm_threshold <= accept_threshold <= 1.0; got '
                f'confirm={self.confirm_threshold}, '
                f'accept={self.accept_threshold}.'
            )
        unknown = sorted(set(self.allowed_actions) - set(schema.MOTION_ACTIONS))
        if unknown:
            raise ValueError(
                f'allowed_actions contains non-motion action(s): '
                f'{", ".join(unknown)}. Valid: '
                f'{", ".join(schema.MOTION_ACTIONS)}.'
            )


DEFAULT_POLICY = GuardrailPolicy()


def apply(intent: ValidatedIntent,
          policy: GuardrailPolicy = DEFAULT_POLICY) -> ParseResult:
    """
    Decide what to do with a structurally valid intent.

    Pure and total: never raises, always returns a
    :class:`~arm_language.result.ParseResult`.
    """
    # 1. The backend said this was not a manipulation request. Believe it —
    #    a model declining to invent a command is exactly the behaviour we
    #    asked for, and second-guessing it here would undo that.
    if intent.action == schema.ACTION_REJECT:
        reason = intent.reason or 'That is not something the arm can do.'
        return ParseResult.refuse(
            ReasonCode.LLM_REJECTED, reason,
            backend_confidence=intent.confidence,
        )

    # 2. Action whitelist.
    if intent.action not in policy.allowed_actions:
        return ParseResult.refuse(
            ReasonCode.ACTION_NOT_ALLOWED,
            f'"{intent.action}" is understood but not enabled on this robot. '
            f'Enabled: {", ".join(policy.allowed_actions)}.',
            requested_action=intent.action,
        )

    command = Command(
        action=intent.action,
        target_query=intent.target_query,
        place_target=intent.place_target,
        modifiers=dict(intent.modifiers),
        confidence=intent.confidence,
    )

    # 3. Confidence bands. Built last so the refusal below still gets to
    #    report what we *would* have done — which is what makes a low-
    #    confidence log entry useful instead of just "gave up".
    if intent.confidence < policy.confirm_threshold:
        return ParseResult.refuse(
            ReasonCode.LOW_CONFIDENCE,
            f'I am not confident enough about that request '
            f'({intent.confidence:.2f}) to move the arm. Try naming the '
            f'object directly, e.g. "pick up the hammer".',
            would_have_done=command.echo(),
            confidence=intent.confidence,
        )

    if intent.confidence < policy.accept_threshold:
        return ParseResult(
            outcome=Outcome.NEEDS_CONFIRMATION,
            reason_code=ReasonCode.LOW_CONFIDENCE,
            message=f'Did you mean: {command.echo()}?',
            command=command,
            detail={'confidence': intent.confidence},
        )

    if policy.require_confirmation:
        return ParseResult(
            outcome=Outcome.NEEDS_CONFIRMATION,
            reason_code=ReasonCode.CONFIRMATION_REQUIRED,
            message=f'Did you mean: {command.echo()}?',
            command=command,
            detail={'confidence': intent.confidence},
        )

    return ParseResult(
        outcome=Outcome.ACCEPTED,
        reason_code=ReasonCode.OK,
        message=command.echo(),
        command=command,
        detail={'confidence': intent.confidence},
    )
