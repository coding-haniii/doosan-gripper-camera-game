import pytest

from robot_games.calib.jog import JogPanel


class FakeDr:
    DR_BASE, DR_WORLD = 0, 2

    def __init__(self):
        self.x = [380.0, 0.0, 60.0, 0.0, 180.0, 0.0]
        self.j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

    def get_current_posx(self, ref=None):
        return (list(self.x), 0)

    def get_current_posj(self):
        return list(self.j)


class FakeRobot:
    def __init__(self):
        self.dr = FakeDr()
        self.lines = []

    def move_line(self, pose, label='', jog=False):
        self.lines.append(list(pose))
        self.dr.x = list(pose)

    def move_joint(self, q, label=''):
        self.dr.j = list(q)


def panel(keys, min_z=40.0, answers=()):
    robot, ks, ans = FakeRobot(), iter(keys), iter(answers)
    return robot, JogPanel(robot, min_z=min_z, key_fn=lambda: next(ks), input_fn=lambda _='': next(ans))


def test_steps_keep_orientation_and_confirm():
    robot, p = panel(['w', '2', 'a', 'r', '\n'])
    p.position('test')
    assert robot.dr.x == [390.0, 1.0, 61.0, 0.0, 180.0, 0.0]


def test_floor_refused_and_slow_near_floor():
    robot, p = panel(['3', 'f', 'f'] + ['f'] * 12 + ['\n'])
    p.position('test')
    zs = [l[2] for l in robot.lines]
    assert zs[0] == 50.0                      # 60 → 50 (10mm 스텝)
    assert zs[1] == 49.0                      # 바닥 15mm 안쪽: 스텝이 1mm 로 줄어듦
    assert min(zs) == 40.0                    # 하한 아래로는 가지 않음


def test_q_aborts_and_joint_step_capped():
    robot, p = panel(['j', 'j', 'q'], answers=['1 15', '2 90'])
    with pytest.raises(KeyboardInterrupt):
        p.position('test')
    assert robot.dr.j[:2] == [15.0, 0.0]


def test_rejected_move_is_reported_not_crashing():
    from robot_games.calib.motion import MotionFailed
    robot, p = panel(['w', '\n'])

    def reject(pose, label='', jog=False):
        raise MotionFailed('NOT REACHABLE')
    robot.move_line = reject
    p.position('test')                         # 예외로 죽지 않고 계속 받음
    assert robot.dr.x[0] == 380.0
