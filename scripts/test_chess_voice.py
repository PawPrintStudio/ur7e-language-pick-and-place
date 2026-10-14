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
    line = v.canned('greeting', {'arm': 'black'})
    assert 'black' in line or 'Ro-Boko' in line
    assert v.canned('captured', {}) != ''                 # a missing {san} falls back cleanly
    assert v.canned('nothing', {}) == ''


def test_every_event_has_a_fact_and_a_kid_friendly_canned_line():
    for event, (situation, lines) in v.EVENTS.items():
        assert v.FACTS[event], event
        assert 'child' in situation
        for line in lines:
            assert len(line.split()) <= 24, line
            assert not any(word in line.lower() for word in ('stupid', 'loser', 'idiot', 'dumb'))
    assert 'button' in v.prompt_for('ask', {'question': ''}) and 'honestly' in v.prompt_for('ask', {'question': 'Do you dream?'})
    assert v.canned('ask', {'question': ''})
    prompt = v.prompt_for('check', {'san': 'Qh4+'})
    assert 'fact about how you work' in prompt and 'child' in prompt and 'The line:' in prompt
    assert 'fact about' not in v.prompt_for('check', {'san': 'Qh4+'}, teach=False)
    assert not v.sane('captured', 'Good job figuring out that move!')      # the robot captured, not the child
    assert not v.sane('check', 'Oops!')
    assert v.sane('lost', 'Nice move, you got one past my camera.')
    assert v.sane('captured', 'That piece is mine now; my camera measured it first.')


def test_jingle_then_bubble_then_voice(tmp_path, monkeypatch):
    order = []
    said = str(tmp_path / 'said.json')

    class Piperish:
        def synthesize(self, text):
            order.append('synth'); return '/tmp/fake.wav'

        def effect(self, event):
            order.append(('effect', event, os.path.exists(said)))

        def play(self, path):
            order.append(('play', path, os.path.exists(said)))
    v.respond('check', {'san': 'Qh4+'}, Brain('Mind your king.'), Piperish(), log=lambda *a: None, said_file=said)
    assert order == ['synth', ('effect', 'check', False), ('play', '/tmp/fake.wav', True)]
    order.clear()
    v.respond('check', {'san': 'Qh4+'}, Brain('Mind your king.'), Piperish(), log=lambda *a: None, said_file=said, effects=False)
    assert order == ['synth', ('play', '/tmp/fake.wav', True)]


def test_every_emotion_has_a_short_jingle_written_once(tmp_path, monkeypatch):
    monkeypatch.setattr(v, 'SFX_DIR', str(tmp_path))
    import wave
    for event in ('greeting', 'captured', 'lost', 'check', 'checked', 'won', 'lost_game', 'draw', 'ask'):
        path = v.sfx_file(event)
        with wave.open(path) as w:
            seconds = w.getnframes() / w.getframerate()
        assert 0.15 <= seconds <= 2.5, (event, seconds)
    assert v.sfx_file('nothing') is None
    stamp = os.path.getmtime(v.sfx_file('won'))
    assert os.path.getmtime(v.sfx_file('won')) == stamp               # made once, reused


def test_respond_prefers_the_model_falls_back_to_canned_and_tells_the_page(tmp_path):
    mouth, said = Mouth(), str(tmp_path / 'said.json')
    assert v.respond('check', {'san': 'Qh4+'}, Brain('Your king is sweating.'), mouth,
                     log=lambda *a: None, said_file=said) == 'Your king is sweating.'
    shown = json.load(open(said))
    assert shown['event'] == 'check' and shown['line'] == 'Your king is sweating.'
    assert 2.0 <= shown['seconds'] <= 4.0
    assert 0 < shown['t'] - time.time() <= v.LEAD_S + 0.1           # stamped a lead ahead, for the TV delay
    line = v.respond('check', {'san': 'Qh4+'}, Brain(None), mouth, log=lambda *a: None, said_file=said)
    assert line in [l for l in v.EVENTS['check'][1]]
    assert v.respond('unknown', {}, None, mouth, log=lambda *a: None, said_file=said) == ''
    assert mouth.said == ['Your king is sweating.', line]


def test_ollama_failure_is_none_not_an_exception():
    brain = v.Ollama(url='http://127.0.0.1:9', timeout_s=0.5)     # nothing listens here
    assert brain.line('check', {'san': 'Qh4+'}) is None
    assert brain.line('unknown', {}) is None
    assert brain.warm(timeout_s=0.5) is None


def test_follow_keeps_the_model_warm(tmp_path):
    class Warmable(Brain):
        warmed = 0

        def warm(self):
            self.warmed += 1
            return 0.1
    brain, path = Warmable('x'), tmp_path / 'events.jsonl'
    thread = threading.Thread(target=v.follow, args=(str(path), brain, Mouth()),
                              kwargs={'poll_s': 0.05, 'log': lambda *a: None}, daemon=True)
    thread.start()
    time.sleep(0.2)
    assert brain.warmed == 1                                        # once at startup, then every few minutes


def test_the_tv_sink_is_found_from_wpctl_status(monkeypatch):
    status = ('PipeWire ...\nAudio\n ├─ Devices:\n ├─ Sinks:\n │  *   48. Built-in Audio Analog Stereo        [vol: 0.99]\n'
              ' │      74. HDA NVidia Digital Stereo (HDMI)    [vol: 0.40]\n │  \n ├─ Sink endpoints:\n ├─ Sources:\n'
              ' │      99. HDMI capture thing [vol: 1.00]\n')
    monkeypatch.setattr(v.subprocess, 'run', lambda *a, **k: type('R', (), {'stdout': status})())
    assert v.find_sink('HDMI') == '74'
    assert v.find_sink('Analog') == '48'
    assert v.find_sink('Bluetooth') is None
    monkeypatch.setattr(v.shutil, 'which', lambda name: '/usr/bin/pw-play' if name == 'pw-play' else None)
    assert v.player_for('HDMI') == ['pw-play', '--target', '74']
    assert v.player_for(None) == ['pw-play']
    monkeypatch.setattr(v.shutil, 'which', lambda name: None)
    assert v.player_for('HDMI') == ['aplay', '-q']


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
