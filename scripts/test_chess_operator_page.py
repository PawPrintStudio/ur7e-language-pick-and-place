"""The operator page in a real browser against the real dashboard server.

The node checks drive the page's script with stand-in elements, which hides
DOM mistakes (2026-10-14: a span inside the prompt text was wiped by the
first paint and every click then died on it). This one needs Playwright
with Chromium (``pip install playwright && python -m playwright install
chromium``) and is skipped without it. No robot, no camera: the server is
started here with the state files in a temporary folder.
"""
import json
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import lab_chess
    from playwright.sync_api import sync_playwright
except ImportError:
    lab_chess = sync_playwright = None

pytestmark = pytest.mark.skipif(sync_playwright is None or lab_chess is None,
                                reason='playwright (with chromium) or python-chess is not installed')


@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setattr(lab_chess, 'STATE_FILE', str(tmp_path / 'state.json'))
    monkeypatch.setattr(lab_chess, 'ANSWER_FILE', str(tmp_path / 'answer.json'))
    monkeypatch.setattr(lab_chess, 'EVENTS_FILE', str(tmp_path / 'events.jsonl'))
    monkeypatch.setattr(lab_chess, 'SAID_FILE', str(tmp_path / 'said.json'))
    monkeypatch.setattr(lab_chess, 'TARGET_FILE', str(tmp_path / 'target.json'))
    http = lab_chess.serve_dashboard(0, {'live': {}})
    try:
        yield f'http://127.0.0.1:{http.server_address[1]}', tmp_path
    finally:
        http.shutdown()


def prompt(kind='student_move', text='Student (White): type the move', options=None, **extra):
    options = options if options is not None else [lab_chess.option('new', 'New game'), lab_chess.option('quit', 'Quit')]
    return dict(id=f'p-{time.time():.3f}', kind=kind, text=text, options=options, allow_text=True, **extra)


def test_the_page_answers_prompts_queues_new_game_and_asks(server):
    url, folder = server
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page()
        page.on('console', lambda m: errors.append(m.text) if m.type == 'error' else None)
        page.on('pageerror', lambda e: errors.append(str(e)))
        # A prompt is open and the game is alive (a fresh heartbeat).
        lab_chess.publish_state(fen=lab_chess.chess.Board().fen(), moves=[], arm='black',
                                prompt=prompt(), beat=time.time())
        page.goto(url + '/#operator', wait_until='domcontentloaded')   # the camera stream never "loads"
        page.wait_for_function("document.getElementById('op-input') && !document.getElementById('op-input').disabled")
        page.fill('#op-input', 'e4')
        page.click('#op-form button[type=submit]')
        page.wait_for_function("document.getElementById('op-note').textContent.startsWith('Sent')")
        answer = json.load(open(folder / 'answer.json'))
        assert answer['answer'] == 'e4' and answer['id'] == lab_chess.read_state()['prompt']['id']
        # The game took it: a new prompt with a countdown. The page shows the countdown and the buttons.
        lab_chess.publish_state(prompt=prompt('confirm_move', 'I saw e4. Right?',
                                              [lab_chess.option('y', 'Yes, e4', 'go'), lab_chess.option('n', 'No')],
                                              timeout_s=8, default='y'), beat=time.time())
        page.wait_for_function("document.getElementById('op-count').textContent.includes('Yes, e4 in')")
        page.click('#op-buttons button[data-key="y"]')
        page.wait_for_function("document.getElementById('op-note').textContent.startsWith('Sent')")
        assert json.load(open(folder / 'answer.json'))['answer'] == 'y'
        # Between prompts (the arm moving) New game is queued with id "*".
        lab_chess.publish_state(prompt=None, beat=time.time())
        page.wait_for_function("document.getElementById('op-standing-note').textContent.includes('after this step')")
        page.click('#op-new')
        page.wait_for_function("document.getElementById('op-note').textContent.startsWith('Sent')")
        assert json.load(open(folder / 'answer.json')) | {} == json.load(open(folder / 'answer.json'))
        assert json.load(open(folder / 'answer.json'))['id'] == '*'
        # Ask reaches the events file.
        page.fill('#op-question', 'Do you dream?')
        page.click('#op-ask-button')
        page.wait_for_function("document.getElementById('op-ask-note').textContent.startsWith('Sent')")
        events = [json.loads(l) for l in open(folder / 'events.jsonl')]
        assert events[-1]['event'] == 'ask' and events[-1]['question'] == 'Do you dream?'
        browser.close()
    assert errors == []
