"""Offline checks for the operator console: prompts, answers, new game, the shift fallback.

No robot, no ROS graph, no browser. Run with::

    python3 -m pytest -q scripts/test_chess_operator.py
"""
import json
import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import chess
    import lab_chess
except ImportError:       # python-chess missing: skip, do not break collection
    chess = lab_chess = None

pytestmark = pytest.mark.skipif(lab_chess is None, reason='python-chess is not installed')


@pytest.fixture(autouse=True)
def isolate(tmp_path, monkeypatch):
    """Never touch the live game's files; never read the real terminal."""
    monkeypatch.setattr(lab_chess, 'TARGET_FILE', str(tmp_path / 'target.json'))
    monkeypatch.setattr(lab_chess, 'STATE_FILE', str(tmp_path / 'state.json'))
    monkeypatch.setattr(lab_chess, 'ANSWER_FILE', str(tmp_path / 'answer.json'))
    monkeypatch.setattr(lab_chess, '_line_from_terminal', lambda timeout_s: time.sleep(timeout_s))


def answer_when_asked(value, prompt_id=None, delay_s=0.1):
    """A browser click, shortly after the prompt is published."""
    def click():
        deadline = time.time() + 3
        while time.time() < deadline:
            prompt = lab_chess.read_state().get('prompt')
            if prompt:
                time.sleep(delay_s)
                with open(lab_chess.ANSWER_FILE, 'w') as stream:
                    json.dump({'id': prompt_id or prompt['id'], 'answer': value}, stream)
                return
            time.sleep(0.02)
    thread = threading.Thread(target=click, daemon=True)
    thread.start()
    return thread


def test_the_dashboard_answer_is_taken_and_the_prompt_is_published_then_cleared():
    answer_when_asked('y')
    got = lab_chess.ask_operator('step', 'b7 -> b5: go?',
                                 [lab_chess.option(lab_chess.GO, 'Go'), lab_chess.option(lab_chess.STOP, 'Stop')])
    assert got == lab_chess.GO
    state = lab_chess.read_state()
    assert state['prompt'] is None and state['beat'] > 0
    assert not os.path.exists(lab_chess.ANSWER_FILE)


def test_a_stale_answer_never_answers_a_later_prompt():
    with open(lab_chess.ANSWER_FILE, 'w') as stream:
        json.dump({'id': 'old-prompt', 'answer': 'y'}, stream)
    answer_when_asked('n', delay_s=0.3)
    got = lab_chess.ask_operator('step', 'go?', [lab_chess.option('y', 'Go'), lab_chess.option('n', 'Stop')])
    assert got == 'n'


def test_typed_text_is_returned_only_where_allowed(capsys):
    answer_when_asked('e4')
    assert lab_chess.ask_operator('student_move', 'move?', [lab_chess.option(lab_chess.QUIT, 'Quit')],
                                  allow_text=True) == 'e4'
    answer_when_asked('bogus')
    answer_when_asked('y', delay_s=0.6)
    assert lab_chess.ask_operator('step', 'go?', [lab_chess.option('y', 'Go')]) == 'y'
    assert 'not one of' in capsys.readouterr().out


def test_new_game_and_quit_raise_only_when_offered():
    answer_when_asked('new')
    with pytest.raises(lab_chess.NewGame):
        lab_chess.ask_operator('student_move', 'move?', lab_chess.GAME_OPTIONS, allow_text=True)
    answer_when_asked('quit')
    with pytest.raises(lab_chess.QuitGame):
        lab_chess.ask_operator('game_over', 'over', lab_chess.GAME_OPTIONS)
    # On a step prompt "quit" is not an option: it is just a wrong answer, then Stop.
    answer_when_asked('quit')
    answer_when_asked('n', delay_s=0.6)
    assert lab_chess.ask_operator('step', 'go?', [lab_chess.option('y', 'Go'), lab_chess.option('n', 'Stop')]) == 'n'


def test_auto_yes_skips_the_prompt():
    assert lab_chess.ask_operator('step', 'go?', [lab_chess.option('y', 'Go')], auto='y') == 'y'
    assert lab_chess.read_state() == {}


def scripted(answers):
    """ask_operator stand-in: hands out the scripted answers, raising like the real one."""
    queue = list(answers)

    def fake(kind, text, options, allow_text=False, auto=None):
        if auto is not None:
            return auto
        value = queue.pop(0)
        keys = {o['key'] for o in options}
        if value == 'new' and lab_chess.NEW_GAME in keys:
            raise lab_chess.NewGame
        if value == 'quit' and lab_chess.QUIT in keys:
            raise lab_chess.QuitGame
        return value
    return fake, queue


def play(monkeypatch, answers, **flags):
    fake, queue = scripted(answers)
    monkeypatch.setattr(lab_chess, 'ask_operator', fake)
    argv = ['lab_chess.py', 'play', '--steps-only', '--engine', 'none', '--seed', '1']
    for flag in flags:
        argv.append('--' + flag.replace('_', '-'))
    monkeypatch.setattr(sys, 'argv', argv)
    return lab_chess.main(), queue


def test_a_new_game_from_the_page_restarts_the_record_without_leaving_play(monkeypatch):
    code, left = play(monkeypatch, ['e4', 'new', 'd4', 'quit'])
    assert code == 0 and left == []
    state = lab_chess.read_state()
    assert state['moves'][0] == 'd4' and len(state['moves']) == 2     # d4, the arm's reply
    assert state['fen'].split()[1] == 'w' and state['arm'] == 'black'
    assert state['prompt'] is None


def test_resume_carries_on_from_the_recorded_game(monkeypatch):
    board = chess.Board()
    board.push_san('e4')
    board.push_san('e5')
    lab_chess.new_game_state(board, arm_white=False)
    code, left = play(monkeypatch, ['Nf3', 'quit'], resume=True)
    assert code == 0 and left == []
    state = lab_chess.read_state()
    assert state['moves'][:3] == ['e4', 'e5', 'Nf3'] and len(state['moves']) == 4


def test_resume_without_a_record_is_refused(monkeypatch):
    code, _ = play(monkeypatch, [], resume=True)
    assert code == 2


def test_an_illegal_typed_move_is_refused_and_asked_again(monkeypatch, capsys):
    code, left = play(monkeypatch, ['e5', 'e4', 'quit'])
    assert code == 0 and left == []
    assert 'REFUSED' in capsys.readouterr().out
    assert lab_chess.read_state()['moves'][0] == 'e4'


def test_shift_slides_every_square_and_the_box_and_refuses_a_big_one(tmp_path, monkeypatch, capsys):
    path = tmp_path / 'geometry.json'
    geometry = lab_chess.BoardGeometry([[0, 0, 0], [0.4, 0, 0], [0.4, 0.4, 0], [0, 0.4, 0]],
                                       box=[0.5, -0.1, 0.05], notes={'stamp': 'test'})
    geometry.save(str(path))
    with open(path) as stream:
        data = json.load(stream)
    data['camera'] = {'base_from_camera': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]], 'mode': 'FHD'}
    path.write_text(json.dumps(data))
    monkeypatch.setattr(sys, 'argv', ['lab_chess.py', '--geometry', str(path), 'shift',
                                      '--dx', '-6', '--dy', '3', '--why', 'goto e4 --touch read 6 mm right'])
    assert lab_chess.main() == 0
    moved = lab_chess.BoardGeometry.load(str(path))
    x, y = moved.square_xy('a1')
    assert x == pytest.approx(0.025 - 0.006) and y == pytest.approx(0.025 + 0.003)
    assert moved.box[0] == pytest.approx(0.494) and moved.square_m == pytest.approx(0.05)
    with open(path) as stream:
        saved = json.load(stream)
    assert saved['camera']['mode'] == 'FHD'                      # the camera pose is kept
    assert any(k.startswith('shift_') for k in saved['notes'])
    monkeypatch.setattr(sys, 'argv', ['lab_chess.py', '--geometry', str(path), 'shift', '--dx', '40'])
    assert lab_chess.main() == 1
    assert 're-touch' in capsys.readouterr().out
