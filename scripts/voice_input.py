#!/usr/bin/env python3
"""Push-to-talk voice input for the arm console (the "voice" stretch item).

Speak a sentence; it is transcribed locally and dropped into an inbox file
that ``lab_console.py --voice-inbox`` watches. The console then treats the
transcript exactly like a typed line: parse, plan, ask, move. Nothing in the
language pipeline knows or cares that the words were spoken.

Why it is a separate process on the host, not part of the console:

* The console runs inside the ROS container, which has no sound device
  (``/dev/snd`` is not passed through). The microphone is on the laptop.
* Speech-to-text is a heavy, swappable dependency (faster-whisper on a
  laptop CPU today; whisper.cpp with CUDA on the Jetson later, per
  docs/IMPLEMENTATION_PLAN.md). Keeping it out of the console keeps the
  console's dependencies at "ROS + arm_language".
* A file is the simplest bridge that crosses a bind mount with no blocking
  semantics to get wrong: the client appends one line per utterance, the
  console tails the file. Both sides can be restarted independently.

Usage (on the laptop, venv with faster-whisper — see the RUNBOOK)::

    .venv-voice/bin/python scripts/voice_input.py            # push-to-talk loop
    .venv-voice/bin/python scripts/voice_input.py --file x.wav   # transcribe a file

Push-to-talk: press Enter, speak, press Enter again. The transcript is
printed and appended to the inbox. Ctrl-C or ``q`` + Enter quits.

Recording uses ``arecord`` (ALSA, present on stock Ubuntu) at 16 kHz mono,
which is what Whisper models expect, so no resampling dependency is needed.
"""
import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INBOX = os.path.join(_HERE, '.voice_inbox.txt')

#: Small English model: ~1 s per short utterance on a laptop CPU, good enough
#: for a bounded command vocabulary. ``base.en`` is faster and worse;
#: ``medium.en`` is better and several times slower.
DEFAULT_MODEL = 'small.en'

#: Words the vocabulary never contains that Whisper likes to hallucinate on
#: silence. A transcript that is only these is dropped.
_SILENCE_JUNK = {'', 'you', 'thank you', 'thanks', 'bye', '.', 'uh', 'um'}


class Recorder:
    """Record from ALSA with arecord until told to stop."""

    def __init__(self, device='default'):
        self.device = device

    def record_until_enter(self, path):
        cmd = ['arecord', '-q', '-D', self.device, '-f', 'S16_LE', '-r', '16000',
               '-c', '1', '-t', 'wav', path]
        proc = subprocess.Popen(cmd)
        started = time.monotonic()
        try:
            input()
        finally:
            # SIGINT lets arecord finish the WAV header properly.
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
        return time.monotonic() - started


class Transcriber:
    """faster-whisper wrapper. The only place the model dependency lives."""

    def __init__(self, model=DEFAULT_MODEL, language='en'):
        try:
            from faster_whisper import WhisperModel
        except ImportError as exc:
            raise SystemExit(
                'faster-whisper is not installed. Create the host venv:\n'
                '  python3 -m venv .venv-voice && '
                '.venv-voice/bin/pip install faster-whisper\n'
                'and run this script with .venv-voice/bin/python.'
            ) from exc
        self.language = language
        print(f'loading whisper model "{model}" (first run downloads it)...',
              flush=True)
        self._model = WhisperModel(model, device='cpu', compute_type='int8')

    def transcribe(self, path):
        segments, info = self._model.transcribe(
            path, language=self.language, beam_size=5, vad_filter=True,
            # A robot command is one short sentence; bias decoding towards
            # our vocabulary so "go up a bit" is not heard as "go up a bid".
            initial_prompt='Robot arm commands: go up a bit, go down 2 cm, go '
                           'left, go right, spin slowly, rotate clockwise, go '
                           'home, go to the ready pose, let me drive it, '
                           'pick up the hammer.')
        text = ' '.join(s.text.strip() for s in segments).strip()
        return text, info


def clean(text):
    """Normalise a transcript into console text; '' if it is just noise."""
    text = ' '.join(text.split())
    if text.lower().strip(' .!?') in _SILENCE_JUNK:
        return ''
    return text


def deliver(inbox, text):
    with open(inbox, 'a') as handle:
        handle.write(text + '\n')


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--model', default=DEFAULT_MODEL)
    cli.add_argument('--language', default='en')
    cli.add_argument('--device', default='default', help='ALSA capture device')
    cli.add_argument('--inbox', default=DEFAULT_INBOX,
                     help='file the console watches (lab_console.py --voice-inbox)')
    cli.add_argument('--no-deliver', action='store_true',
                     help='only print transcripts, do not write the inbox')
    cli.add_argument('--file', help='transcribe this WAV instead of the microphone')
    args = cli.parse_args()

    transcriber = Transcriber(args.model, args.language)

    if args.file:
        started = time.monotonic()
        text, info = transcriber.transcribe(args.file)
        print(f'[{time.monotonic() - started:.1f}s, p={info.language_probability:.2f}] '
              f'{clean(text)!r}')
        if not args.no_deliver and clean(text):
            deliver(args.inbox, clean(text))
        return 0

    recorder = Recorder(args.device)
    print('push-to-talk: Enter to start, speak, Enter to stop; q + Enter quits.')
    while True:
        try:
            key = input('voice> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if key in ('q', 'quit', 'exit'):
            return 0
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            path = tmp.name
        try:
            print('recording... Enter to stop', flush=True)
            seconds = recorder.record_until_enter(path)
            if seconds < 0.3:
                print('too short, ignored')
                continue
            started = time.monotonic()
            text, _ = transcriber.transcribe(path)
            text = clean(text)
            elapsed = time.monotonic() - started
            if not text:
                print(f'[{seconds:.1f}s audio, {elapsed:.1f}s] heard nothing')
                continue
            print(f'[{seconds:.1f}s audio, {elapsed:.1f}s] {text}')
            if not args.no_deliver:
                deliver(args.inbox, text)
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass


if __name__ == '__main__':
    sys.exit(main())
