"""
Replay backend — plays back canned responses, for testing the layers above.

The validator and the guardrails have to behave correctly when a backend
returns something *wrong*: truncated JSON, a hallucinated field, a whole
sentence in ``target_query``, a confidence of ``1.5``, an action nobody
defined. Those cases are the entire reason those layers exist, and none of
them can be provoked by asking a well-behaved model nicely.

So the adversarial cases are written down as fixtures and replayed. This is the
only honest way to test a trust boundary: you cannot verify that a guard works
by feeding it inputs the guard is not for.

Read this carefully before using it for anything else: replaying a *correct*
response and asserting we accept it proves nothing about the parser's real
accuracy — the fixture and the assertion are both things we wrote. Fixtures
test **our** code. The 20-utterance corpus run against a real backend tests the
**parser**. Do not let the first stand in for the second.
"""
import json
from typing import Dict, Mapping, Optional

from .base import BackendError


class ReplayBackend:
    """Return a recorded response for each utterance."""

    name = 'replay'

    def __init__(self, responses: Mapping[str, str],
                 default: Optional[str] = None) -> None:
        """
        Store the utterance-to-raw-response map.

        ``default`` is returned for unknown utterances; without one, an unknown
        utterance raises :class:`~arm_language.backends.base.BackendError` so a
        stale fixture file fails loudly instead of silently skipping cases.
        """
        self._responses: Dict[str, str] = dict(responses)
        self._default = default

    @classmethod
    def from_file(cls, path: str, default: Optional[str] = None) -> 'ReplayBackend':
        """
        Load a ``{utterance: raw_response}`` JSON file.

        Values may be given as objects for readability; they are re-serialised
        to the text a real backend would return. Strings pass through
        untouched, which is how deliberately malformed fixtures survive.
        """
        with open(path, encoding='utf-8') as handle:
            data = json.load(handle)
        responses = {
            key: value if isinstance(value, str) else json.dumps(value)
            for key, value in data.items()
        }
        return cls(responses, default=default)

    def complete(self, utterance: str) -> str:
        """Return the recorded response for ``utterance``."""
        key = utterance.strip()
        if key in self._responses:
            return self._responses[key]
        if self._default is not None:
            return self._default
        raise BackendError(f'No recorded response for {utterance!r}.')

    def describe(self) -> str:
        """Return a one-line description for startup logs."""
        return f'{self.name} ({len(self._responses)} recorded responses)'
