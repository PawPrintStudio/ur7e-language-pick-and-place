#!/usr/bin/env python3
"""The robot's voice, on the laptop: one spoken line per game event, fully local.

Reads ``log/chess/events.jsonl`` (written by ``lab_chess.py play`` in the
container), asks a local model through Ollama for one short sentence in the
robot's voice, speaks it with Piper, and never makes the game wait: the game
loop only appends a line; this process does the rest, late or not at all.

Why it lives on the laptop and not in ``play``: the container has no sound
device and no GPU; Ollama and Piper are host installs (``.venv-voice``).
Why a file and not a socket: same reason as the operator console, and the
file is also the record of what happened in a game.

Usage (host)::

    .venv-voice/bin/python scripts/chess_voice.py                 # tail and speak
    .venv-voice/bin/python scripts/chess_voice.py --say captured  # try one event
    .venv-voice/bin/python scripts/chess_voice.py --no-llm        # canned lines only

Hard limits: one sentence, 20 words, a 3 s model timeout after which a canned
line is spoken instead. ``--replay FILE`` answers from a JSON map for tests.
"""
import argparse
import json
import os
import random
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

_HERE = os.path.dirname(os.path.abspath(__file__))
EVENTS_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'events.jsonl')
SAID_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'said.json')   # what is being said, for the page
DEFAULT_MODEL = 'qwen2.5:3b'          # answers in about a second on a 6 GB GPU
DEFAULT_VOICE = 'en_US-lessac-medium'
OLLAMA = 'http://127.0.0.1:11434'
MAX_WORDS = 22

PERSONA = ('You are Ro-Boko, a friendly robot arm playing chess with children and '
           'families visiting a university makerspace. Speak in the first person as '
           'the robot, to the child in front of you. You are warm, playful, a little '
           'dramatic and always encouraging; gentle teasing is fine, never mean, never '
           'scary, nothing a parent would frown at. Never praise the child for a '
           'capture YOU made. When you are given a FACT about how you work, weave it '
           'in simply, the way you would tell a curious eight-year-old, without '
           'technical words. Answer with ONE sentence of at most 22 words, no quotes, '
           'no emoji, no stage directions. Do not invent moves, squares or pieces '
           'beyond the one you are told about. Examples of your style: '
           '"That piece is mine now; my camera spotted it from above." '
           '"Check! I looked at thousands of moves in one second to find that one." '
           '"Nice move; you got one past my eyes."')

# One thing about how the robot works, per event, told the simple way. Each
# line is offered to the model as the FACT to weave in, and used as a canned
# line on its own when the model is slow.
FACTS = {
    'greeting': ['A camera above the board shows me every piece in 3D, so I know where to reach.',
                 'I think with a chess program that tries thousands of moves before picking one.'],
    'captured': ['My camera measures how tall each piece is so my fingers close at the right height.',
                 'I grab pieces from the corner, where the neighbours give my fingers the most room.'],
    'lost': ['I only see the board between moves, so you surprised me there.',
             'My chess program already knew that could happen; it chose the move anyway.'],
    'check': ['I looked at thousands of moves in a second and picked the one that bothers your king.',
              'Before I move, I plan the whole path so my arm never bumps a piece.'],
    'won': ['A computer can look at more moves than any person, but you made me work for it.'],
    'lost_game': ['Even a robot that checks thousands of moves can be beaten by a good plan.'],
    'draw': ['Neither of us could win: my program and your brain agreed on that.'],
}

# What each event means, for the prompt and as the fallback line.
EVENTS = {
    'greeting': ('A new child sits down to play against you; you, the robot, play {arm}.',
                 ['Hello, I am Ro-Boko! My camera watches the board from above; make your move when you are ready.',
                  'Welcome, challenger! I play {arm}; my chess brain tries thousands of moves before I pick one.']),
    'captured': ('You, the robot, just took one of the child\'s pieces with the move {san}.',
                 ['That piece is mine now; my camera measured exactly how tall it was before I grabbed it.',
                  'Thank you for the {san}; I plan my whole arm path so I never bump the other pieces.',
                  'One for my box! I pick pieces up from the corner, where my fingers have the most room.']),
    'lost': ('The child just took one of YOUR pieces with the move {san}; you are the one who lost material.',
             ['Oh! You got one past my camera; I only look at the board between moves.',
              'Nice {san}; my chess program saw that coming and chose the move anyway.',
              'A fair trade, I hope; let us see what my program says next.']),
    'check': ('You, the robot, just put the child\'s king in check with {san}.',
              ['Check! I looked at thousands of moves in one second to find that one.',
               'Check; mind your king, I planned that move before my arm even started moving.']),
    'won': ('You, the robot, just WON the game against the child ({result}) after {moves} moves.',
            ['Good game! A computer can check more moves than anyone, but you made me work for it.',
             'I won this one; play again and see if you can trick my camera?']),
    'lost_game': ('The child just beat you: you, the robot, LOST the game ({result}) after {moves} moves.',
                  ['You beat a robot today! Even one that checks thousands of moves can lose to a good plan.',
                   'Well played; I bow my gripper to you. Rematch?']),
    'draw': ('Your game with the child ended in a draw ({result}).',
             ['A draw! Neither of us could win: my program and your brain agreed on that.']),
}


def one_sentence(text, max_words=MAX_WORDS):
    """The first sentence, trimmed to ``max_words``, without quotes or emoji."""
    text = re.sub(r'[\"“”*_]|\(.*?\)|\[.*?\]', '', text or '').strip()
    text = re.split(r'(?<=[.!?])\s', text, maxsplit=1)[0] if text else ''
    words = text.split()
    if len(words) > max_words:
        text = ' '.join(words[:max_words]).rstrip(',;:') + '.'
    return text.strip()


def canned(event, detail):
    """A fallback line for ``event`` with the detail filled in."""
    lines = EVENTS.get(event, (None, []))[1]
    if not lines:
        return ''
    try:
        return random.choice(lines).format(**detail)
    except (KeyError, IndexError):
        return lines[0].split('{')[0].strip()


def prompt_for(event, detail, teach=True):
    """The model rewrites a canned line rather than inventing one.

    2026-10-14: asked to react freely, the 3B model praised the child for the
    robot's own captures and drifted to "Good job!". Paraphrasing a line that
    already carries the meaning and the fact keeps it on track.
    """
    situation = EVENTS[event][0].format(**{k: detail.get(k, '?') for k in ('arm', 'san', 'result', 'moves')})
    line = canned(event, detail)
    fact = random.choice(FACTS.get(event, [''])) if teach else ''
    return (f'{situation} Say this to the child in your own words, keeping its meaning'
            + (' and this fact about how you work: ' + fact if fact else '')
            + f'. The line: "{line}"')


PRAISE = ('good job', 'nice move', 'well done', 'great move', 'your move was', 'good thinking',
          'good eye', 'you got', 'you did')


def sane(event, line):
    """Reject a model line that gets the perspective wrong or says nothing."""
    low = (line or '').lower()
    if len(low.split()) < 4:
        return False
    if event in ('captured', 'check', 'won', 'greeting') and any(p in low for p in PRAISE):
        return False
    return True


class Ollama:
    """One short completion from a local model; None on any failure or timeout.

    Measured 2026-10-14 on the RTX 3050: 0.3 s warm, 47 s for the first load
    from disk. So the model is loaded once at startup (``warm``), asked to
    stay loaded (``keep_alive``), and nudged every few minutes by ``follow``;
    Ollama's default is to drop it after five idle minutes, longer than a
    student takes to think.
    """

    def __init__(self, model=DEFAULT_MODEL, url=OLLAMA, timeout_s=3.0, keep_alive='2h'):
        self.model, self.url, self.timeout_s, self.keep_alive = model, url, timeout_s, keep_alive

    def _generate(self, prompt, system, num_predict, timeout_s):
        body = json.dumps({'model': self.model, 'system': system, 'prompt': prompt, 'stream': False,
                           'keep_alive': self.keep_alive,
                           'options': {'num_predict': num_predict, 'temperature': 0.7}}).encode()
        request = urllib.request.Request(f'{self.url}/api/generate', data=body,
                                         headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.load(response)

    def warm(self, timeout_s=180.0):
        """Load the model now; returns the seconds it took, or None when it cannot."""
        started = time.time()
        try:
            self._generate('Say: ready.', 'Answer with one word.', 2, timeout_s)
        except (urllib.error.URLError, OSError, ValueError, TimeoutError):
            return None
        return round(time.time() - started, 1)

    def line(self, event, detail):
        if event not in EVENTS:
            return None
        try:
            answer = self._generate(prompt_for(event, detail), PERSONA, 48, self.timeout_s)
            line = one_sentence(answer.get('response', ''))
            return line if sane(event, line) else None
        except (urllib.error.URLError, OSError, ValueError, TimeoutError):
            return None


class Replay:
    """Scripted answers (event -> line) for tests and rehearsals."""

    def __init__(self, path):
        with open(path) as stream:
            self.lines = json.load(stream)

    def line(self, event, detail):
        text = self.lines.get(event)
        return one_sentence(text.format(**detail)) if text else None


def find_sink(match='HDMI'):
    """PipeWire sink id whose name contains ``match`` (wpctl status), or None."""
    try:
        status = subprocess.run(['wpctl', 'status'], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    sinks = False
    for line in status.splitlines():
        if 'Sinks:' in line:
            sinks = True
            continue
        if sinks and ('Sources:' in line or 'Sink endpoints:' in line):
            break
        found = re.search(r'(\d+)\.\s+(.*?)\s+\[vol', line)
        if sinks and found and match.lower() in found.group(2).lower():
            return found.group(1)
    return None


def player_for(sink=None):
    """The command that plays a wav: pw-play on the chosen sink (the TV over HDMI), else aplay."""
    if shutil.which('pw-play'):
        target = find_sink(sink) if sink else None
        if sink and target is None:
            print(f'NOTE: no audio sink matching "{sink}" (wpctl status); using the default output', flush=True)
        return ['pw-play'] + (['--target', target] if target else [])
    return ['aplay', '-q']


class Piper:
    """Speak with Piper TTS through the laptop's audio; falls back to printing."""

    def __init__(self, voice=DEFAULT_VOICE, data_dir=None, player=None, sink=None):
        self.voice = voice
        self.player = list(player) if player else player_for(sink)
        self.data_dir = data_dir or os.path.join(_HERE, '..', 'log', 'chess', 'voices')
        self.ready = os.path.exists(os.path.join(self.data_dir, f'{voice}.onnx'))

    def say(self, text):
        if not text:
            return False
        print(f'ROBOT    {text}', flush=True)
        if not self.ready:
            return False
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as wav:
            path = wav.name
        try:
            subprocess.run([sys.executable, '-m', 'piper', '--model', os.path.join(self.data_dir, f'{self.voice}.onnx'),
                            '--output_file', path], input=text.encode(), check=True, timeout=20,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(self.player + [path], check=False, timeout=30,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except (subprocess.SubprocessError, OSError) as error:
            print(f'NOTE: could not speak ({error})', flush=True)
            return False
        finally:
            try:
                os.remove(path)
            except OSError:
                pass


def announce(event, line, path=SAID_FILE, seconds=None):
    """Tell the dashboard what is being said: watch serves it, the page shows the face and bubble."""
    seconds = seconds if seconds is not None else round(1.0 + 0.45 * len(line.split()), 1)
    row = {'event': event, 'line': line, 't': time.time(), 'seconds': seconds}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + '.tmp', 'w') as stream:
        json.dump(row, stream)
    os.replace(path + '.tmp', path)
    return row


def respond(event, detail, brain, mouth, log=print, said_file=SAID_FILE):
    """Choose the line (model, else canned), show it, speak it. Returns the line."""
    line = brain.line(event, detail) if brain is not None else None
    source = 'model'
    if not line:
        line, source = canned(event, detail), 'canned'
    if line:
        log(f'LINE     {json.dumps({"event": event, "source": source})}')
        announce(event, line, said_file)
        mouth.say(line)
    return line


def follow(path, brain, mouth, poll_s=0.3, from_start=False, log=print):
    """Tail the events file forever, speaking each new line's event."""
    position = 0 if from_start else (os.path.getsize(path) if os.path.exists(path) else 0)
    log(f'VOICE    {json.dumps({"events": os.path.relpath(path), "model": getattr(brain, "model", None), "stop": "Ctrl+C"})}')
    warm_every_s, warmed = 180.0, 0.0
    while True:
        if hasattr(brain, 'warm') and time.time() - warmed > warm_every_s:
            took = brain.warm()
            log(f'MODEL    {json.dumps({"loaded": took is not None, "seconds": took})}')
            warmed = time.time()
        if not os.path.exists(path):
            time.sleep(poll_s)
            continue
        with open(path) as stream:
            stream.seek(position)
            lines = stream.readlines()
            position = stream.tell()
        for line in lines:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            event = row.pop('event', None)
            if time.time() - float(row.get('t', time.time())) > 30:
                continue                  # an old event is not worth a late remark
            respond(event, row, brain, mouth, log)
        time.sleep(poll_s)


def main():
    cli = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--events', default=EVENTS_FILE)
    cli.add_argument('--model', default=DEFAULT_MODEL, help='Ollama model name')
    cli.add_argument('--voice', default=DEFAULT_VOICE, help='Piper voice (log/chess/voices/<voice>.onnx)')
    cli.add_argument('--no-llm', action='store_true', help='canned lines only')
    cli.add_argument('--replay', help='JSON file of event -> line, instead of the model')
    cli.add_argument('--say', metavar='EVENT', help='speak one event and exit (a test of the pipeline)')
    cli.add_argument('--from-start', action='store_true', help='speak the events already in the file')
    cli.add_argument('--sink', default='HDMI',
                     help='audio output: a substring of the PipeWire sink name (default HDMI: the TV); '
                          '"default" for the laptop speakers')
    args = cli.parse_args()
    if os.path.exists('/.dockerenv') or not os.path.exists('/dev/snd'):
        print('BLOCKED  this is the container (no sound card): run chess_voice.py in a HOST terminal, '
              'from the repo folder on the laptop', flush=True)
        return 2
    brain = None if args.no_llm else Replay(args.replay) if args.replay else Ollama(args.model)
    mouth = Piper(args.voice, sink=None if args.sink == 'default' else args.sink)
    print(f'AUDIO    {json.dumps({"player": " ".join(mouth.player), "voice": args.voice})}', flush=True)
    if not mouth.ready:
        print(f'NOTE: no Piper voice at log/chess/voices/{args.voice}.onnx: lines are printed, not spoken '
              f'(python -m piper.download_voices {args.voice} --data-dir log/chess/voices)', flush=True)
    if args.say:
        if hasattr(brain, 'warm'):
            print(f'MODEL    loaded in {brain.warm()} s', flush=True)
        started = time.time()
        respond(args.say, {'arm': 'black', 'san': 'Nxd4', 'result': '0-1', 'moves': 31}, brain, mouth)
        print(f'TIMING   {time.time() - started:.1f} s from event to the end of speech', flush=True)
        return 0
    try:
        follow(args.events, brain, mouth, from_start=args.from_start)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
