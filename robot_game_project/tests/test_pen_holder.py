"""펜 거치대 집기·반납(robot_games.pen_holder) 테스트. 로봇·카메라 없이 계산과 순서만 확인한다."""
import json
import math

import pytest

from robot_games import pen_holder as ph
from robot_games.gripper import PenNotHeld

H_FLIP = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]        # 카메라 (u, v) → base (v, u): 실기처럼 축이 바뀐 호모그래피


def test_marker_pose_center_and_yaw_through_homography():
    # 카메라에서 TL→TR 이 +u 방향 → base 에서는 +y 방향 → yaw 90°
    corners = [(100, 200), (140, 200), (140, 240), (100, 240)]
    x, y, yaw = ph.marker_pose(corners, H_FLIP)
    assert (x, y) == pytest.approx((220, 120))
    assert yaw == pytest.approx(90)


def test_marker_frame_round_trip_and_rotation():
    marker = (385.0, -3.6, 177.5)
    pose = [330.0, 2.0, 120.0, 13.5, -180.0, 13.5]
    rel = ph.to_marker_frame(pose, marker)
    assert ph.from_marker_frame(rel, marker) == pytest.approx(pose)
    # 거치대를 (+50, +20) 옮기고 10° 돌리면, 마커에서 본 상대 위치와 공구 회전이 그대로 따라온다
    moved = (435.0, 16.4, 187.5)
    p2 = ph.from_marker_frame(rel, moved)
    assert math.hypot(p2[0] - moved[0], p2[1] - moved[1]) == pytest.approx(math.hypot(pose[0] - marker[0], pose[1] - marker[1]))
    assert p2[3] == pytest.approx(23.5) and p2[2] == pose[2]


def _holder(marker=(385.0, -3.6, 177.5)):
    a = [330.0, 2.0, 140.0, 13.5, -180.0, 13.5]
    g = [330.0, 2.0, 70.0, 13.5, -180.0, 13.5]
    return {'marker_at_teach': list(marker), 'approach': ph.to_marker_frame(a, marker),
            'grasp': ph.to_marker_frame(g, marker), 'verified': True}


def test_holder_poses_lift_and_limits():
    h = _holder()
    poses = ph.holder_poses(h, (385.0, -3.6, 177.5), lift=50)
    assert poses['grasp'][2] == pytest.approx(120) and poses['approach'][2] == pytest.approx(190)
    with pytest.raises(ph.HolderError, match='돌아감'):
        ph.holder_poses(h, (385.0, -3.6, 177.5 + 30))
    with pytest.raises(ph.HolderError, match='옮겨짐'):
        ph.holder_poses(h, (700.0, 200.0, 177.5))
    assert ph.holder_poses(h, (385.0, -3.6, -179.0))      # 177.5° → -179° 는 3.5° 차이(각도 한 바퀴 넘김)


def test_load_holder_requires_teach_and_verified(tmp_path):
    p = tmp_path / 'robot_poses.json'
    p.write_text(json.dumps({'home': [0] * 6}))
    with pytest.raises(ph.HolderError, match='교시'):
        ph.load_holder(p)
    p.write_text(json.dumps({'pen_holder': {**_holder(), 'verified': False}}))
    with pytest.raises(ph.HolderError, match='verified'):
        ph.load_holder(p)
    assert ph.load_holder(p, rehearsal=True)


class FakeRobot:
    def __init__(self, log): self.log = log
    def move_line(self, pose, vel=None, acc=None): self.log.append(('move', round(pose[2])))


class FakeGripper:
    cfg = {}                                               # readback 없음 → '이미 쥐고 있나' 확인 생략
    def __init__(self, log, held=True): self.log, self.held = log, held
    def open(self): self.log.append(('open',))
    def close(self):
        self.log.append(('close',))
        if not self.held:
            raise PenNotHeld('빈 손')
        return 560


def _poses():
    return ph.holder_poses(_holder(), (385.0, -3.6, 177.5))


def test_pick_order_and_failed_grasp_backs_off():
    log = []
    assert ph.pick(FakeRobot(log), FakeGripper(log), _poses(), log=lambda *_: None) == 560
    assert log == [('open',), ('move', 140), ('move', 70), ('close',), ('move', 140)]
    log = []
    with pytest.raises(ph.BackedOff) as e:
        ph.pick(FakeRobot(log), FakeGripper(log, held=False), _poses(), log=lambda *_: None)
    assert isinstance(e.value.__cause__, PenNotHeld)
    assert log[-2:] == [('open',), ('move', 140)]          # 못 잡으면 열고 위로 빠진다


class BrokenOpen(FakeGripper):
    def open(self):
        self.log.append(('open',))
        raise TimeoutError('readback socket timed out')     # OSError 계열도 빠져 나와야 한다


def test_give_back_error_at_holder_lifts_straight_up_without_touching_gripper():
    log = []
    with pytest.raises(ph.BackedOff):
        ph.give_back(FakeRobot(log), BrokenOpen(log), _poses(), log=lambda *_: None)
    assert log == [('move', 140), ('move', 70), ('open',), ('move', 140)]


def test_pick_refuses_when_already_holding():
    log = []
    g = FakeGripper(log)
    g.cfg = {'readback_host': 'x', 'holding_min': 450, 'holding_max': 605}
    g.position = lambda: 560
    with pytest.raises(ph.HolderError, match='이미'):
        ph.pick(FakeRobot(log), g, _poses(), log=lambda *_: None)
    assert log == []                                         # 열지도, 움직이지도 않음


def test_approach_is_straight_above_grasp_and_rehearsal_lift():
    h = _holder()
    h['approach'][0] += 9.0                                   # 교시 때 접근 XY 를 조금 어긋나게 맞춰도
    poses = ph.holder_poses(h, (385.0, -3.6, 177.5))
    assert poses['approach'][:2] == pytest.approx(poses['grasp'][:2]) and poses['approach'][3:] == poses['grasp'][3:]
    assert ph.rehearsal_lift(h) == pytest.approx(70)         # 접근-잡기 70mm > 50mm


def test_marker_frame_exact_values():
    rel = ph.to_marker_frame([100.0, 0.0, 50.0, 0.0, -180.0, 0.0], (0.0, 0.0, 0.0))
    assert ph.from_marker_frame(rel, (0.0, 0.0, 90.0))[:2] == pytest.approx([0.0, 100.0])
    assert ph.from_marker_frame(rel, (10.0, 0.0, 90.0))[:2] == pytest.approx([10.0, 100.0])


def test_rehearsal_never_closes_and_return_opens_at_grasp():
    log = []
    ph.pick(FakeRobot(log), FakeGripper(log), _poses(), rehearsal=True, log=lambda *_: None)
    assert ('close',) not in log
    log = []
    ph.give_back(FakeRobot(log), FakeGripper(log), _poses(), log=lambda *_: None)
    assert log == [('move', 140), ('move', 70), ('open',), ('move', 140)]


def test_return_measures_again_from_above_the_holder(tmp_path):
    st = tmp_path / 's.json'
    ph.save_pick_marker((388.0, 0.3, 179.1), st)
    def hidden():
        raise ph.HolderError('안 보임')
    wrong = lambda: (386.0, 20.0, 160.0)                                  # home 에서 쥔 펜이 가려 틀어진 값
    assert ph.rough_marker_for_return(wrong, st, log=lambda *_: None) == pytest.approx((388.0, 0.3, 179.1))
    with pytest.raises(ph.HolderError):
        ph.rough_marker_for_return(hidden, tmp_path / 'none.json', log=lambda *_: None)
    # 거치대 위에서 다시 잰 값(실제로 20mm 밀림)으로 꽂는다 — 저장값을 믿지 않음
    h = _holder((388.0, 0.3, 179.1))
    log = []
    marker, poses = ph.view_and_refine(FakeRobot(log), h, (388.0, 0.3, 179.1), lambda: (386.3, 22.1, -169.4), log=lambda *_: None)
    assert marker == (386.3, 22.1, -169.4)
    assert poses == ph.holder_poses(h, marker)
    assert log == [('move', 140)]                                       # 먼저 높은 곳(옆으로 비킨 보기 자세)으로만 이동
    with pytest.raises(ph.HolderError):
        ph.view_and_refine(FakeRobot([]), h, (388.0, 0.3, 179.1), hidden, log=lambda *_: None)
