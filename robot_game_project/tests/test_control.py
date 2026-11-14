"""컨트롤 센터 worker 요청 처리 (가짜 로봇, 하드웨어 없음)."""
from robot_games.control import Worker


class FakeRobot:
    def __init__(self, z=100.0, stopped=False):
        self.p = [500.0, 0.0, z, 4.0, -180.0, 4.0]
        self.q = [0.0] * 6
        self.cfg = {'jog_speed_mm_s': 100, 'jog_accel_mm_s2': 300}
        self._stopped = stopped
        self.homed = 0

    def current_posx(self, ref=None):
        return list(self.p)

    def current_posj(self):
        return list(self.q)

    def move_line(self, pose, vel=None, acc=None):
        self.p = list(pose)

    def move_joint(self, q):
        self.q = list(q)

    def home(self):
        self.homed += 1

    def state(self):
        return 3 if self._stopped else 1

    def stopped(self):
        return self._stopped


class FakeGripper:
    def __init__(self):
        self.calls = []

    def open(self):
        self.calls.append('open')
        return 120

    def close(self, check_held=True):
        self.calls.append(('close', check_held))
        return 620

    def position(self):
        return 300


CFG = {'robot': {'min_pen_z_base_mm': 30.0}, 'gripper': {'readback_host': '1.2.3.4'}}


def worker(**kw):
    return Worker(FakeRobot(**kw), CFG, gripper_factory=FakeGripper)


def test_jog_moves_one_step():
    w = worker()
    r = w.handle({'cmd': 'jog', 'axis': 0, 'sign': 1, 'step': 10.0})
    assert r['ok'] and r['posx'][0] == 510.0


def test_jog_refuses_below_floor():
    w = worker(z=30.5)
    r = w.handle({'cmd': 'jog', 'axis': 2, 'sign': -1, 'step': 1.0})
    assert r['ok'] and r['posx'][2] == 30.5        # 하한 30 아래로 가지 않음 (JogPanel 이 거부)


def test_jog_near_floor_limits_step():
    w = worker(z=40.0)
    r = w.handle({'cmd': 'jog', 'axis': 2, 'sign': -1, 'step': 10.0})
    assert r['posx'][2] == 39.0                    # 바닥 15mm 안에서는 1mm 까지만


def test_jog_rejects_unknown_step():
    assert not worker().handle({'cmd': 'jog', 'axis': 0, 'sign': 1, 'step': 7.0})['ok']


def test_joint_limit():
    w = worker()
    assert not w.handle({'cmd': 'joint', 'joint': 1, 'delta': 45})['ok']
    r = w.handle({'cmd': 'joint', 'joint': 2, 'delta': -5})
    assert r['ok'] and r['posj'][1] == -5.0


def test_motion_refused_when_stopped():
    w = worker(stopped=True)
    for req in ({'cmd': 'jog', 'axis': 0, 'sign': 1, 'step': 1.0}, {'cmd': 'home'}, {'cmd': 'grip', 'action': 'open'}):
        assert not w.handle(req)['ok']
    assert w.robot.homed == 0
    assert w.handle({'cmd': 'status'})['ok']


def test_gripper_close_does_not_require_pen():
    w = worker()
    r = w.handle({'cmd': 'grip', 'action': 'close'})
    assert r['ok'] and r['gripper'] == 620 and w.gripper().calls == [('close', False)]


def test_choose_camera_never_falls_back_when_realsense_present(tmp_path, monkeypatch):
    import pytest
    from robot_games import camera_select as cs
    monkeypatch.setattr(cs, 'realsense_candidates', lambda devices=None: [6, 4])
    with pytest.raises(RuntimeError, match='RealSense'):
        cs.choose_camera(0, reader=lambda i, need_color=True: i == 0)     # 내장 카메라(0)만 읽힘 → 넘어가지 않음
    monkeypatch.setattr(cs, 'realsense_candidates', lambda devices=None: [])
    assert cs.choose_camera(0, reader=lambda i, need_color=True: i == 0) == 0   # RealSense 가 없는 PC 는 기존대로


def test_ocr_noise_two_letter_alias_not_used_inside_sentence():
    from robot_games.ocr_rules import load_rules
    r = load_rules()
    noisy = 'ad :" MILO ere, 7 eo . an rm It a ara ro! | Be tes. teenie'   # 2026-11-14 미로 카드가 종료로 읽힌 OCR
    assert r.match(noisy)[0] != 'exit'
    assert r.match('ab STOP cd')[0] == 'exit'        # 정식 키워드는 문장 안에서도
