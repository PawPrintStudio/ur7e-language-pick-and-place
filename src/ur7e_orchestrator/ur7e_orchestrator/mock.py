"""
Mock adapters: a pretend robot that succeeds at everything unless told not to.

This is what lets the whole workflow run in CI with no arm, camera, or
language model. Tests then *script* a failure for one call and check that
the state machine reacts the way the retry policy says it should.
"""
import time

from ur7e_orchestrator.adapters import (
    Adapters,
    Detection,
    GraspResult,
    Intent,
    Refused,
    Target,
)

_METHODS = (
    'parse', 'observe', 'detect', 'locate', 'plan', 'approach', 'grasp',
    'lift', 'retreat', 'home', 'release', 'safety_ok', 'abort',
)


class MockAdapters(Adapters):
    """
    Adapters that record every call and can be scripted to misbehave.

    ``calls`` is the ordered list of method names called; ``call_log`` is
    the same list with arguments, as ``(name, args)`` tuples.

    ``script`` maps a method name to a list with one entry per call of
    that method, used up in order; calls beyond the list behave normally.
    An entry may be:

    - ``None``: behave normally;
    - an exception instance: raise it;
    - ``('sleep', seconds)``: sleep, then behave normally (for timeouts);
    - a callable: call it with the method's arguments; its return value
      is the result, or ``None`` to then behave normally;
    - anything else: return it as the method's result, for example
      ``GraspResult(False, 0.0)`` or ``(False, 'protective_stop')``.

    So ``{'detect': [NotFound('not_found', 'nothing there'), None]}``
    makes the first ``detect()`` fail and the second succeed.
    """

    def __init__(self, script=None):
        self.calls = []
        self.call_log = []
        self._script = {name: list(entries) for name, entries in (script or {}).items()}
        unknown = sorted(set(self._script) - set(_METHODS))
        if unknown:
            raise ValueError(f'script names unknown adapter methods: {unknown}')

    def _respond(self, name, args, normally):
        """Record the call, then follow the script or behave ``normally``."""
        self.calls.append(name)
        self.call_log.append((name, args))
        entries = self._script.get(name)
        entry = entries.pop(0) if entries else None
        if isinstance(entry, BaseException):
            raise entry
        if isinstance(entry, tuple) and len(entry) == 2 and entry[0] == 'sleep':
            time.sleep(entry[1])
            entry = None
        elif callable(entry):
            entry = entry(*args)
        return normally() if entry is None else entry

    def parse(self, utterance):
        """Understand 'pick up X' and 'pick up X and put it on Y' only."""
        return self._respond('parse', (utterance,), lambda: _parse_pick(utterance))

    def observe(self):
        """Pretend to move the arm out of the camera view."""
        return self._respond('observe', (), lambda: None)

    def detect(self, query):
        """Pretend to find ``query`` with high confidence."""
        capture = f'mock-{self.calls.count("detect") + 1:04d}'
        return self._respond(
            'detect', (query,), lambda: Detection(capture_id=capture, confidence=0.9))

    def locate(self, detection):
        """Return a made-up pose; each call is 10 cm further along y."""
        y = round(0.1 * self.calls.count('locate'), 3)
        return self._respond('locate', (detection,), lambda: Target(
            xyz=(0.4, y, 0.02), yaw=0.0, detail={'capture_id': detection.capture_id}))

    def plan(self, intent, target, place):
        """Return a plan summary that echoes what it was asked to plan."""
        return self._respond('plan', (intent, target, place), lambda: {
            'action': intent.action,
            'pick_xyz': list(target.xyz),
            'place_xyz': None if place is None else list(place.xyz),
        })

    def approach(self):
        """Pretend to open the jaws, hover, and descend."""
        return self._respond('approach', (), lambda: None)

    def grasp(self):
        """Pretend to close the jaws on a 25 mm object."""
        return self._respond(
            'grasp', (), lambda: GraspResult(holding=True, width_mm=25.0))

    def lift(self):
        """Pretend to lift."""
        return self._respond('lift', (), lambda: None)

    def retreat(self):
        """Pretend to carry the object away and release it."""
        return self._respond('retreat', (), lambda: None)

    def home(self):
        """Pretend to go home."""
        return self._respond('home', (), lambda: None)

    def release(self):
        """Pretend to open the jaws."""
        return self._respond('release', (), lambda: None)

    def safety_ok(self):
        """Report that it is safe to move."""
        return self._respond('safety_ok', (), lambda: (True, ''))

    def abort(self):
        """Pretend to stop motion."""
        return self._respond('abort', (), lambda: None)


def _parse_pick(utterance):
    """Split 'pick up X [and put it on Y]' into an Intent, or refuse."""
    text = utterance.strip().rstrip('.!').lower()
    if not text.startswith('pick up '):
        raise Refused('not_a_pick', f'The mock parser only knows "pick up ...": {utterance!r}')
    target, _, place = text[len('pick up '):].partition(' and put it on ')
    target, place = _strip_article(target), _strip_article(place)
    if not target:
        raise Refused('not_a_pick', f'No object named in {utterance!r}')
    if place:
        return Intent(
            action='pick_and_place', target_query=target, place_query=place,
            echo=f'pick up the {target} and put it on the {place}')
    return Intent(action='pick', target_query=target, echo=f'pick up the {target}')


def _strip_article(phrase):
    """Return ``phrase`` without a leading 'the', 'a', or 'an'."""
    words = phrase.split()
    if words and words[0] in ('the', 'a', 'an'):
        words = words[1:]
    return ' '.join(words)
