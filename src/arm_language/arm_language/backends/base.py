"""
The backend contract: text in, raw JSON text out.

Architecture D5 requires the language backend to be swappable — cloud Claude
now, a local Llama on the Jetson later, without touching anything downstream.
This module is what makes that swap a one-line change.

The contract is deliberately tiny: ``complete(utterance) -> str``. A backend
returns *unvalidated text* and is trusted with nothing else. It does not build
a :class:`~arm_language.result.Command`, does not decide whether to move, and
does not get to declare its own output valid. Everything a backend produces
goes through :mod:`arm_language.validator` before anyone looks at it.

That narrowness is the safety property. If a backend could return a Command
directly, then adding a backend would mean auditing a new path to the arm.
Returning a string means a new backend cannot widen what the robot will do —
the worst a broken or malicious backend achieves is a refusal.
"""
from typing import Protocol, runtime_checkable


class BackendError(RuntimeError):
    """
    A backend could not produce a response at all.

    Distinct from "produced something invalid": this is no credentials, no
    network, a timeout, a 5xx. The distinction matters operationally — an
    invalid response is a prompt or model problem to investigate, a
    :class:`BackendError` is usually infrastructure and often transient.
    """


@runtime_checkable
class Backend(Protocol):
    """Anything that can turn an utterance into candidate JSON text."""

    name: str
    """Short identifier recorded in parse diagnostics and eval reports, so a
    result can always be traced back to what produced it."""

    def complete(self, utterance: str) -> str:
        """
        Return candidate JSON for ``utterance``.

        Implementations should raise :class:`BackendError` when they cannot
        respond. They must **not** raise for an unparseable or nonsensical
        response — returning it is correct, and the validator will reject it
        with a reason code that says so.
        """
        ...
