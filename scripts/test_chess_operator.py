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
    monkeypatch.setattr(lab_chess, 'EVENTS_FILE', str(tmp_path / 'events.jsonl'))
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


def test_new_game_and_quit_are_taken_on_every_prompt():
    answer_when_asked('new')
    with pytest.raises(lab_chess.NewGame):
        lab_chess.ask_operator('student_move', 'move?', lab_chess.GAME_OPTIONS, allow_text=True)
    answer_when_asked('quit')
    with pytest.raises(lab_chess.QuitGame):
        lab_chess.ask_operator('game_over', 'over', lab_chess.GAME_OPTIONS)
    # The page's standing buttons: a step prompt takes them too (the step is not started).
    answer_when_asked('quit')
    with pytest.raises(lab_chess.QuitGame):
        lab_chess.ask_operator('step', 'go?', [lab_chess.option('y', 'Go'), lab_chess.option('n', 'Stop')])


def test_the_whole_move_is_approved_once_unless_careful(monkeypatch):
    asked = []
    monkeypatch.setattr(lab_chess, 'ask_operator',
                        lambda kind, text, options, allow_text=False, auto=None: asked.append((kind, auto)) or (auto or 'y'))

    class Orchestrator:
        def run(self, text):
            return type('R', (), {'run_id': 'r', 'outcome': 'succeeded', 'reason': 'ok', 'message': '',
                                  'duration_s': 0.0, 'retries': 0})()
    lab = RecordingLab()
    adapters = lab_chess.ChessAdapters(lab, square_board(), lab_chess.load_pieces())
    session = (None, lab, adapters, Orchestrator())
    assert lab_chess.run_steps(session, [('d4', 'box'), ('c6', 'd4')], False, confirm_steps=False) == 2
    assert asked == [('step', 'y'), ('step', 'y')]           # auto-approved: no prompt shown
    asked.clear()
    assert lab_chess.run_steps(session, [('d4', 'box')], False, confirm_steps=True) == 1
    assert asked == [('step', None)]                          # --careful: the prompt is shown


def test_the_dashboard_server_answers_and_resets(monkeypatch):
    import urllib.request
    server = lab_chess.serve_dashboard(0, {'live': {}})
    port = server.server_address[1]

    def post(path, body=None):
        req = urllib.request.Request(f'http://127.0.0.1:{port}{path}', method='POST',
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                return r.status
        except urllib.error.HTTPError as error:
            return error.code
    try:
        lab_chess.publish_state(fen='8/8/8/8/8/8/8/8 w - - 0 1', moves=['e4'], arm='white',
                                prompt={'id': 'p1', 'kind': 'step', 'text': 'go?', 'options': []},
                                beat=time.time())
        assert post('/answer', {'id': 'p1', 'answer': 'y'}) == 200
        with open(lab_chess.ANSWER_FILE) as stream:
            assert json.load(stream)['answer'] == 'y'
        assert post('/answer', {'nope': 1}) == 400
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/state', timeout=3) as r:
            state = json.load(r)
        assert state['prompt']['id'] == 'p1' and 0 <= state['prompt_age_s'] < 5
        # Reset: refused while a chess command owns the record, else a fresh record keeping the arm's side.
        with lab_chess.game_state_lock():
            assert post('/reset') == 409
        assert post('/reset') == 200
        state = lab_chess.read_state()
        assert state['fen'] == chess.STARTING_FEN and state['moves'] == [] and state['arm'] == 'white'
        assert 'prompt' not in state
    finally:
        server.shutdown()


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


class RecordingLab:
    """What ChessAdapters touches of lab_pick.LabAdapters, recording the motions."""

    def __init__(self):
        self.calls = []
        self.executor = type('Executor', (), {'plan_only': False, 'table_z': staticmethod(lambda x, y: 0.0)})()
        self.config = {'finger_reach': 0.028, 'open_mm': 80.0}

    def observe(self):
        self.calls.append('observe')

    def retreat(self):
        self.calls.append('retreat')

    def home(self):
        self.calls.append('home')


def square_board():
    return lab_chess.BoardGeometry([[0, 0, 0], [0.4, 0, 0], [0.4, 0.4, 0], [0, 0.4, 0]])


def test_a_capture_parks_only_after_its_last_step_and_a_failure_always_parks():
    lab = RecordingLab()
    adapters = lab_chess.ChessAdapters(lab, square_board(), lab_chess.load_pieces())
    # Step 1 of a capture: the arm is not at home (fake executor: unknown), so
    # OBSERVE moves; after a good RETREAT the HOME is skipped.
    adapters.parse('d4 box')
    adapters.park_after = False
    adapters.observe(); adapters.retreat(); adapters.home()
    assert lab.calls == ['observe', 'retreat'] and adapters.parked_off_home
    # Step 2 starts where step 1 ended: no OBSERVE move; it is the last step: HOME runs.
    adapters.parse('c6 d4')
    adapters.park_after = True
    adapters.observe(); adapters.retreat(); adapters.home()
    assert lab.calls == ['observe', 'retreat', 'retreat', 'home'] and not adapters.parked_off_home
    # A step that failed before RETREAT goes home even between a capture's steps.
    adapters.parse('e7 e5')
    adapters.park_after = False
    adapters.home()
    assert lab.calls[-1] == 'home'


def test_park_arm_sends_a_parked_off_home_arm_home_once():
    lab = RecordingLab()
    adapters = lab_chess.ChessAdapters(lab, square_board(), lab_chess.load_pieces())
    session = (None, lab, adapters, None)
    lab_chess.park_arm(session)
    assert lab.calls == []                                 # nothing to do
    adapters.parse('d4 box'); adapters.park_after = False
    adapters.retreat(); adapters.home()
    lab_chess.park_arm(session)
    lab_chess.park_arm(session)
    assert lab.calls == ['retreat', 'home'] and not adapters.parked_off_home


def events():
    try:
        with open(lab_chess.EVENTS_FILE) as stream:
            return [json.loads(line) for line in stream if line.strip()]
    except FileNotFoundError:
        return []


def test_a_game_logs_its_events_and_the_result_is_from_the_arms_side(monkeypatch):
    # The random mover is not reproducible; drive the engine instead: after
    # e4 e5 Nf3 Nc6 Bc4 Bc5 Nc3 it takes on f2 with check, and the king takes back.
    replies = iter(['e5', 'Nc6', 'Bc5', 'Bxf2+', 'Qh4+'])
    monkeypatch.setattr(lab_chess.Engine, 'choose', lambda self, board: board.parse_san(next(replies)))
    code, left = play(monkeypatch, ['e4', 'Nf3', 'Bc4', 'Nc3', 'Kxf2', 'quit'])
    assert code == 0 and left == []
    kinds = [e['event'] for e in events()]
    # Bxf2+ is a capture and a check: the capture wins (celebrate over nod);
    # Kxf2 takes the bishop back (lost); Qh4+ is a plain check.
    assert kinds == ['greeting', 'captured', 'lost', 'check']
    assert all(e.get('fen') and e.get('san') for e in events()[1:])


def test_react_gestures_only_with_a_session_and_unless_switched_off(monkeypatch):
    performed = []
    monkeypatch.setattr(lab_chess.chess_gestures, 'perform',
                        lambda event, *a, **k: performed.append(event))
    args = type('Args', (), {'no_gestures': False})()
    lab_chess.react(None, args, 'captured', san='Nxd4')
    assert [e['event'] for e in events()] == ['captured'] and performed == []
    executor = type('Ex', (), {'joint_rate': 0.4})()
    lab = RecordingLab()
    adapters = lab_chess.ChessAdapters(lab, square_board(), lab_chess.load_pieces())
    lab_chess.react((executor, lab, adapters, None), args, 'won', result='0-1')
    assert performed == ['won']
    args.no_gestures = True
    lab_chess.react((executor, lab, adapters, None), args, 'check')
    assert performed == ['won'] and len(events()) == 3


class FakeVision:
    """The camera sees one pawn-sized piece, 3 mm off its square's centre."""

    def __init__(self, square, top_m=0.040):
        self.cv = type('CV', (), {'GRIP_FRACTION': 0.75})()
        self.square, self.top_m = square, top_m

    def look(self, tips):
        geometry = square_board()
        x, y = geometry.square_xy(self.square)
        return ({self.square: {'occupied': True, 'centre': [x + 0.003, y, 0.0], 'top_m': self.top_m}},
                None, None)


def test_a_measured_piece_grips_by_type_and_checks_its_width(monkeypatch):
    monkeypatch.setattr(lab_chess, 'executor_tips', lambda executor: None)
    pieces = lab_chess.load_pieces()
    pieces['pieces']['p'] = {'height_m': 0.040, 'grip_m': 0.030, 'width_mm': 21, 'base_mm': 18}
    pieces['pieces']['n'] = {'height_m': 0.054, 'width_mm': 26}            # not measured
    board = chess.Board()
    adapters = lab_chess.ChessAdapters(RecordingLab(), square_board(), pieces, board,
                                       vision=FakeVision('e2'))
    target = adapters.seen_target('e2', board.piece_at(chess.E2), lambda x, y: 0.0, 0.0)
    assert target.detail['tip_above_table_m'] == pytest.approx(0.030 - lab_chess.PAD_BELOW_GRIP_M)
    assert adapters.expected_width_mm == 21
    assert target.xyz[0] == pytest.approx(square_board().square_xy('e2')[0] + 0.003)  # camera centre
    adapters.vision = FakeVision('b1', top_m=0.054)
    target = adapters.seen_target('b1', board.piece_at(chess.B1), lambda x, y: 0.0, 0.0)
    assert target.detail['tip_above_table_m'] == pytest.approx(0.75 * 0.054 - lab_chess.PAD_BELOW_GRIP_M)
    assert adapters.expected_width_mm is None


def test_a_wrong_piece_height_is_warned_about(monkeypatch, capsys):
    monkeypatch.setattr(lab_chess, 'executor_tips', lambda executor: None)
    pieces = lab_chess.load_pieces()
    pieces['pieces']['p'] = {'height_m': 0.040, 'grip_m': 0.030, 'width_mm': 21}
    board = chess.Board()
    adapters = lab_chess.ChessAdapters(RecordingLab(), square_board(), pieces, board,
                                       vision=FakeVision('e2', top_m=0.080))
    adapters.seen_target('e2', board.piece_at(chess.E2), lambda x, y: 0.0, 0.0)
    assert 'right piece' in capsys.readouterr().out


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
