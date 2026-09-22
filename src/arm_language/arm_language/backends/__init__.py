"""
Backend registry.

Backends are constructed by name so the choice can be a launch parameter
(``backend:=claude``) rather than an import someone has to edit — that is what
"swappable behind the interface" means in practice for architecture D5.

Construction is lazy for a reason: importing :mod:`arm_language.backends.claude`
pulls in the ``anthropic`` SDK, which is not installed in CI. A registry of
factory callables keeps that import off the path of anyone who did not ask for
that backend.
"""
from typing import Callable, Dict

from .base import Backend, BackendError

__all__ = ['Backend', 'BackendError', 'available', 'create']


def _make_claude(**kwargs) -> Backend:
    """Construct the cloud Claude backend."""
    from .claude import ClaudeBackend
    return ClaudeBackend(**kwargs)


def _make_keyword(**kwargs) -> Backend:
    """Construct the offline keyword backend."""
    from .keyword import KeywordBackend
    return KeywordBackend(**kwargs)


_REGISTRY: Dict[str, Callable[..., Backend]] = {
    'claude': _make_claude,
    'keyword': _make_keyword,
}


def available() -> tuple:
    """
    Return the names that :func:`create` accepts.

    ``replay`` is intentionally absent: it is a test double, and a typo in a
    launch file should never quietly start the robot on canned answers.
    """
    return tuple(sorted(_REGISTRY))


def create(name: str, **kwargs) -> Backend:
    """
    Build the named backend.

    Raises :class:`~arm_language.backends.base.BackendError` for an unknown
    name — a failure at startup, where an operator will see it.
    """
    try:
        factory = _REGISTRY[name]
    except KeyError:
        raise BackendError(
            f'Unknown backend "{name}". Available: {", ".join(available())}.'
        ) from None
    return factory(**kwargs)
