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
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

_HERE = os.path.dirname(os.path.abspath(__file__))
EVENTS_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'events.jsonl')
DEFAULT_MODEL = 'qwen2.5:3b'          # answers in about a second on a 6 GB GPU
DEFAULT_VOICE = 'en_US-lessac-medium'
OLLAMA = 'http://127.0.0.1:11434'
MAX_WORDS = 20

PERSONA = ('You are Ro-Boko, a robot arm playing chess against a student at a '
           'university makerspace. You are confident, a little dramatic and kind; '
           'a touch of trash talk is fine, never mean. Answer with ONE sentence of '
           'at most 20 words, no quotes, no emoji, no stage directions.')

# What each event means, for the prompt and as the fallback line.
EVENTS = {
    'greeting': ('A new student sits down to play you; you are {arm}.',
                 ['Welcome. I am Ro-Boko. Make your move when you are ready.',
                  'A new challenger. I play {arm}; show me what you have got.']),
    'captured': ('You just captured a piece with {san}.',
                 ['That piece is mine now.', 'Thank you for the {san}. I will keep it.',
                  'One fewer piece for you to worry about.']),
    'lost': ('The student just captured one of your pieces with {san}.',
             ['I meant to give that one away. Mostly.', 'Noted. I will remember {san}.',
              'A fair trade, I am sure. Let us see.']),
    'check': ('You just gave check with {san}.', ['Check. Mind your king.',
                                                   'Your king looks nervous.']),
    'won': ('You just won the game ({result}) after {moves} moves.',
            ['Good game. The arm wins this one.', 'Checkmate, as calculated. Play again?']),
    'lost_game': ('You just lost the game ({result}) after {moves} moves.',
                  ['Well played. You have beaten a robot today.',
                   'I resign with dignity. Rematch?']),
    'draw': ('The game ended in a draw ({result}).', ['A draw. Honour on both sides.']),
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


def prompt_for(event, detail):
    situation = EVENTS[event][0].format(**{k: detail.get(k, '?') for k in ('arm', 'san', 'result', 'moves')})
    fen = detail.get('fen')
    return (f'{situation}' + (f' Position (FEN): {fen}.' if fen else '')
            + ' Say one line to the student.')


class Ollama:
    """One short completion from a local model; None on any failure or timeout."""

    def __init__(self, model=DEFAULT_MODEL, url=OLLAMA, timeout_s=3.0):
        self.model, self.url, self.timeout_s = model, url, timeout_s

    def line(self, event, detail):
        if event not in EVENTS:
            return None
        body = json.dumps({'model': self.model, 'system': PERSONA, 'prompt': prompt_for(event, detail),
                           'stream': False, 'options': {'num_predict': 40, 'temperature': 0.9}}).encode()
        request = urllib.request.Request(f'{self.url}/api/generate', data=body,
                                         headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return one_sentence(json.load(response).get('response', '')) or None
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


class Piper:
    """Speak with Piper TTS through aplay; falls back to printing."""

    def __init__(self, voice=DEFAULT_VOICE, data_dir=None, player=('aplay', '-q')):
        self.voice, self.player = voice, list(player)
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


def respond(event, detail, brain, mouth, log=print):
    """Choose the line (model, else canned) and speak it. Returns the line."""
    line = brain.line(event, detail) if brain is not None else None
    source = 'model'
    if not line:
        line, source = canned(event, detail), 'canned'
    if line:
        log(f'LINE     {json.dumps({"event": event, "source": source})}')
        mouth.say(line)
    return line


def follow(path, brain, mouth, poll_s=0.3, from_start=False, log=print):
    """Tail the events file forever, speaking each new line's event."""
    position = 0 if from_start else (os.path.getsize(path) if os.path.exists(path) else 0)
    log(f'VOICE    {json.dumps({"events": os.path.relpath(path), "model": getattr(brain, "model", None), "stop": "Ctrl+C"})}')
    while True:
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
    args = cli.parse_args()
    brain = None if args.no_llm else Replay(args.replay) if args.replay else Ollama(args.model)
    mouth = Piper(args.voice)
    if not mouth.ready:
        print(f'NOTE: no Piper voice at log/chess/voices/{args.voice}.onnx: lines are printed, not spoken '
              f'(python -m piper.download_voices {args.voice} --data-dir log/chess/voices)', flush=True)
    if args.say:
        respond(args.say, {'arm': 'black', 'san': 'Nxd4', 'result': '0-1', 'moves': 31}, brain, mouth)
        return 0
    try:
        follow(args.events, brain, mouth, from_start=args.from_start)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
