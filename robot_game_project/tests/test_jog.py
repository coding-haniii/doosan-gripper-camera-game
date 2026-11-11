"""키보드 스텝 조그(robot_games.jog) 테스트. GuardedDoosan 과 같은 인터페이스의 가짜 로봇을 쓴다."""
import pytest

from robot_games.jog import JogPanel


class FakeRobot:
    DR_BASE, DR_WORLD = 0, 2

    def __init__(self):
        self.cfg = {'jog_speed_mm_s': 100, 'jog_accel_mm_s2': 300}
        self.x = [380.0, 0.0, 60.0, 0.0, 180.0, 0.0]
        self.j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.lines, self.is_stopped = [], False

    def current_posx(self, ref=None):
        return list(self.x)

    def current_posj(self):
        return list(self.j)

    def move_line(self, pose, vel=None, acc=None):
        self.lines.append(list(pose))
        self.x = list(pose)

    def move_joint(self, q):
        self.j = list(q)

    def stopped(self):
        return self.is_stopped


def panel(keys, min_z=40.0, answers=()):
    robot, ks, ans = FakeRobot(), iter(keys), iter(answers)
    return robot, JogPanel(robot, min_z=min_z, key_fn=lambda: next(ks), input_fn=lambda _='': next(ans),
                           flush_fn=lambda: None)


def test_steps_keep_orientation_and_confirm():
    robot, p = panel(['w', '2', 'a', 'r', '\n'])
    p.position('test')
    assert robot.x == [390.0, 1.0, 61.0, 0.0, 180.0, 0.0]


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
    assert robot.j[:2] == [15.0, 0.0]


def test_rejected_move_reported_but_stop_state_aborts():
    robot, p = panel(['w', '\n'])
    robot.move_line = lambda pose, vel=None, acc=None: (_ for _ in ()).throw(RuntimeError('NOT REACHABLE'))
    p.position('test')                         # 도달 불가는 알리고 계속
    assert robot.x[0] == 380.0
    robot, p = panel(['w', '\n'])
    robot.move_line = lambda pose, vel=None, acc=None: (_ for _ in ()).throw(RuntimeError('stopped'))
    robot.is_stopped = True
    with pytest.raises(KeyboardInterrupt):     # 보호정지·비상정지면 조그 종료
        p.position('test')
