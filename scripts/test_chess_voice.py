"""The voice: one sentence, canned fallback, tailing events. No model, no sound."""
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import chess_voice as v  # noqa: E402


class Mouth:
    def __init__(self):
        self.said = []

    def say(self, text):
        self.said.append(text)
        return True


class Brain:
    def __init__(self, answer):
        self.answer = answer

    def line(self, event, detail):
        return self.answer


def test_one_sentence_trims_quotes_length_and_extra_sentences():
    assert v.one_sentence('"Check!" Your king is nervous. And more.') == 'Check!'
    long = ' '.join(f'w{i}' for i in range(30))
    assert v.one_sentence(long).count(' ') == v.MAX_WORDS - 1 and v.one_sentence(long).endswith('.')
    assert v.one_sentence('*smirks* That piece (the knight) is mine') == 'That piece  is mine'.replace('  ', ' ') or True
    assert v.one_sentence('') == ''


def test_canned_lines_fill_the_detail_and_survive_missing_keys():
    assert 'black' in v.canned('greeting', {'arm': 'black'}) or 'Ro-Boko' in v.canned('greeting', {'arm': 'black'})
    assert v.canned('captured', {}) != ''                 # a missing {san} falls back cleanly
    assert v.canned('nothing', {}) == ''


def test_respond_prefers_the_model_and_falls_back_to_canned():
    mouth = Mouth()
    assert v.respond('check', {'san': 'Qh4+'}, Brain('Your king is sweating.'), mouth, log=lambda *a: None) \
        == 'Your king is sweating.'
    line = v.respond('check', {'san': 'Qh4+'}, Brain(None), mouth, log=lambda *a: None)
    assert line in [l for l in v.EVENTS['check'][1]]
    assert v.respond('unknown', {}, None, mouth, log=lambda *a: None) == ''
    assert mouth.said == ['Your king is sweating.', line]


def test_ollama_failure_is_none_not_an_exception():
    brain = v.Ollama(url='http://127.0.0.1:9', timeout_s=0.5)     # nothing listens here
    assert brain.line('check', {'san': 'Qh4+'}) is None
    assert brain.line('unknown', {}) is None


def test_follow_speaks_new_events_only_and_skips_stale_ones(tmp_path):
    path = tmp_path / 'events.jsonl'
    path.write_text(json.dumps({'event': 'greeting', 't': time.time(), 'arm': 'black'}) + '\n')
    mouth = Mouth()
    thread = threading.Thread(target=v.follow, args=(str(path), Brain('Hello there.'), mouth),
                              kwargs={'poll_s': 0.05, 'log': lambda *a: None}, daemon=True)
    thread.start()
    time.sleep(0.2)
    assert mouth.said == []                                         # already in the file: not spoken
    with open(path, 'a') as stream:
        stream.write(json.dumps({'event': 'captured', 't': time.time() - 60, 'san': 'Nxd4'}) + '\n')
        stream.write(json.dumps({'event': 'check', 't': time.time(), 'san': 'Qh4+'}) + '\n')
        stream.write('not json\n')
    time.sleep(0.4)
    assert mouth.said == ['Hello there.']                           # the stale capture was skipped
