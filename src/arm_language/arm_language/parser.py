"""
The parse pipeline, assembled.

``utterance -> backend -> validator -> guardrails -> ParseResult``

Each arrow is a narrowing. The backend produces arbitrary text; the validator
reduces that to a known shape or refuses; the guardrails reduce that to a
command we are willing to execute or refuse. Nothing widens. That one-way
property is what lets us reason about the node without reasoning about the
model inside it.

:meth:`IntentParser.parse` is **total**: it returns a
:class:`~arm_language.result.ParseResult` for every input, and raises nothing.
A caller that forgets to handle an error path cannot exist, because there are
no error paths — only outcomes.
"""
import logging
import time
from typing import Optional

from . import guardrails, schema, validator
from .backends.base import Backend, BackendError
from .guardrails import GuardrailPolicy
from .result import ParseResult, ReasonCode

LOGGER = logging.getLogger(__name__)


class IntentParser:
    """Free-form text in, validated command or refusal out."""

    def __init__(self, backend: Backend,
                 policy: Optional[GuardrailPolicy] = None) -> None:
        """Bind a backend and a guardrail policy."""
        self._backend = backend
        self._policy = policy or guardrails.DEFAULT_POLICY

    @property
    def backend_name(self) -> str:
        """Return the bound backend's name, for logs and reports."""
        return getattr(self._backend, 'name', type(self._backend).__name__)

    @property
    def policy(self) -> GuardrailPolicy:
        """Return the active guardrail policy."""
        return self._policy

    def parse(self, utterance: str) -> ParseResult:
        """Parse one utterance. Never raises."""
        started = time.monotonic()
        result = self._parse_inner(utterance)

        # Diagnostics are attached here, in one place, so every exit path gets
        # them — including the early refusals that never reached a backend.
        # A run log missing latency on exactly the failures you are chasing is
        # a log that has failed at its only job.
        enriched = ParseResult(
            outcome=result.outcome,
            reason_code=result.reason_code,
            message=result.message,
            command=result.command,
            detail=dict(
                result.detail,
                backend=self.backend_name,
                latency_ms=round((time.monotonic() - started) * 1000, 1),
                utterance=utterance,
            ),
        )
        LOGGER.debug('parse(%r) -> %s/%s in %sms', utterance,
                     enriched.outcome.value, enriched.reason_code.value,
                     enriched.detail['latency_ms'])
        return enriched

    def _parse_inner(self, utterance: str) -> ParseResult:
        """Run the pipeline, converting every failure into a refusal."""
        text = (utterance or '').strip()

        # Cheap input checks first: no reason to spend a network round trip
        # discovering that the operator hit Enter on an empty prompt.
        if not text:
            return ParseResult.refuse(
                ReasonCode.EMPTY_UTTERANCE,
                'Say what you would like the arm to pick up.',
            )

        if len(text) > schema.MAX_UTTERANCE_CHARS:
            return ParseResult.refuse(
                ReasonCode.UTTERANCE_TOO_LONG,
                f'That request is {len(text)} characters; the limit is '
                f'{schema.MAX_UTTERANCE_CHARS}. Name the object in a short '
                f'phrase, e.g. "pick up the hammer".',
                length=len(text),
            )

        try:
            raw = self._backend.complete(text)
        except BackendError as exc:
            return ParseResult.refuse(
                ReasonCode.BACKEND_ERROR,
                f'The language backend could not answer: {exc}',
                error=str(exc),
            )
        except Exception as exc:  # noqa: BLE001 - see comment below
            # A backend is third-party code (an SDK, a socket, a subprocess).
            # If it raises something undeclared, that must still not take the
            # node down mid-run: a crashed parser is a stuck orchestrator,
            # whereas a refusal is a state the workflow already knows how to
            # handle. The exception type is recorded so it can be fixed.
            LOGGER.exception('Backend %s raised an unexpected error',
                             self.backend_name)
            return ParseResult.refuse(
                ReasonCode.BACKEND_ERROR,
                f'The language backend failed unexpectedly '
                f'({type(exc).__name__}: {exc}).',
                error=str(exc),
                error_type=type(exc).__name__,
            )

        try:
            intent = validator.validate(raw)
        except validator.ValidationError as exc:
            # The raw text is kept because this is the failure you cannot
            # diagnose without it — "schema_violation" alone never tells you
            # which field the model got wrong.
            return ParseResult.refuse(
                exc.reason_code,
                f'I could not turn that into a command I trust. {exc.message}',
                raw_response=raw[:500],
            )

        return guardrails.apply(intent, self._policy)
