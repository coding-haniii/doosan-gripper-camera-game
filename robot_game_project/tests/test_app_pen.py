"""app.py 의 펜 집기·반납 연결 테스트. 로봇·카메라 없이 순서와 종료코드 처리만 확인한다."""
import sys
from types import SimpleNamespace

import pytest

from robot_games import app


def _cfg(on=True):
    return {'robot': {'pen_from_holder': on}, 'vision': {}, 'camera': {}}


def _run(rc, calls):
    def run(cmd, cwd=None):
        calls.append(cmd[3:])
        return SimpleNamespace(returncode=rc)
    return run


def test_pen_step_runs_pen_holder_in_its_own_process():
    calls = []
    assert app.pen_step('pick', SimpleNamespace(mode='real'), _cfg(), run=_run(0, calls))
    assert calls == [['pick', '--execute-motion', '--yes']]


def test_pen_step_already_holding_continues_but_other_failures_stop():
    assert app.pen_step('pick', SimpleNamespace(mode='real'), _cfg(), run=_run(3, []))
    assert not app.pen_step('pick', SimpleNamespace(mode='real'), _cfg(), run=_run(2, []))
    assert not app.pen_step('return', SimpleNamespace(mode='real'), _cfg(), run=_run(3, []))
    assert app.pen_step('return', SimpleNamespace(mode='real'), _cfg(), run=_run(4, []))   # 빈 손 → 반납할 것 없음


def test_pen_step_off_or_mock_never_spawns():
    calls = []
    assert app.pen_step('pick', SimpleNamespace(mode='real'), _cfg(False), run=_run(0, calls))
    assert app.pen_step('pick', SimpleNamespace(mode='mock'), _cfg(), run=_run(0, calls))
    assert calls == []


def _main(monkeypatch, argv, steps, pick_ok=True):
    monkeypatch.setattr(sys, 'argv', ['app', *argv])
    monkeypatch.setattr(app, 'cfg_load', lambda p: _cfg())
    def step(action, args, cfg):
        steps.append(action)
        return pick_ok if action == 'pick' else True
    monkeypatch.setattr(app, 'pen_step', step)


def test_maze_failure_still_returns_the_pen(monkeypatch):
    steps = []
    _main(monkeypatch, ['--game', 'maze', '--paper-maze'], steps)
    import robot_games.maze_game as mg
    monkeypatch.setattr(mg, 'run', lambda argv: (_ for _ in ()).throw(RuntimeError('boom')))
    with pytest.raises(RuntimeError):
        app.main()
    assert steps == ['pick', 'return']


def test_maze_nonzero_exit_returns_pen_and_keeps_code(monkeypatch):
    steps = []
    _main(monkeypatch, ['--game', 'maze', '--paper-maze'], steps)
    import robot_games.maze_game as mg
    monkeypatch.setattr(mg, 'run', lambda argv: 2)
    with pytest.raises(SystemExit) as e:
        app.main()
    assert e.value.code == 2 and steps == ['pick', 'return']


def test_tictactoe_returns_pen_after_robot_closed(monkeypatch):
    steps = []
    _main(monkeypatch, ['--game', 'tictactoe'], steps)
    order = []
    class Robot:
        def close(self): order.append('close')
    monkeypatch.setattr(app, 'MockRobot', Robot)
    monkeypatch.setattr(app, 'tictactoe', lambda a, r, c: order.append('game'))
    real_step = app.pen_step
    monkeypatch.setattr(app, 'pen_step', lambda action, *a: (order.append(action), real_step(action, *a))[1])
    app.main()
    assert order == ['pick', 'game', 'close', 'return']


def test_exit_card_and_failed_pick_skip_the_game(monkeypatch):
    steps = []
    _main(monkeypatch, ['--game', 'exit'], steps)
    monkeypatch.setattr(app, 'MockRobot', lambda: SimpleNamespace(home=lambda: None))
    app.main()
    assert steps == ['return']                           # 종료 카드: 쥐고 있으면 반납(빈 손이면 pen_holder 가 코드 4)
    steps = []
    _main(monkeypatch, ['--game', 'tictactoe'], steps, pick_ok=False)
    monkeypatch.setattr(app, 'tictactoe', lambda *a: pytest.fail('게임이 실행되면 안 됨'))
    with pytest.raises(SystemExit):
        app.main()
    assert steps == ['pick']                             # 못 집었으면 반납도 없음
