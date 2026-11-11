"""펜 경로 모서리 다듬기(motion.smooth_pen_path) 테스트. 로봇 없이 좌표만 확인한다."""
import numpy as np

from robot_games.motion import MIN_SPLINE_STEP_MM, smooth_pen_path


def staircase_corner():
    """+Y 로 60mm 간 뒤 직각으로 꺾어 +X 로 60mm. BFS 처럼 0.4mm 계단과 1px 점으로 촘촘하게."""
    pts = [[500.0, -60.0 + 0.4 * i, 45.0 + 0.004 * i, 4.4, -180.0, 4.4] for i in range(151)]
    pts += [[500.0 + 0.4 * i, 0.0, 45.6 + 0.004 * i, 4.4, -180.0, 4.4] for i in range(1, 151)]
    return pts


def test_corner_is_rounded_and_spaced():
    seg = staircase_corner()
    out = smooth_pen_path(seg)
    xy = np.array([p[:2] for p in out])
    steps = np.hypot(*np.diff(xy, axis=0).T)
    assert steps.min() >= MIN_SPLINE_STEP_MM - 1e-6          # 1mm 미만 구간 없음 → 일정 속도 유지
    heading = np.degrees(np.arctan2(*np.diff(xy, axis=0).T[::-1]))
    assert np.abs(((np.diff(heading) + 180) % 360) - 180).max() < 45.1   # 한 번에 90° 꺾지 않음
    corner_gap = min(np.hypot(x - 500.0, y - 0.0) for x, y in xy)
    assert 1.5 < corner_gap < 3.0                              # 반지름 5mm 원호(2mm 간격 점): 꼭짓점에서 2~3mm 안쪽
    assert np.allclose(out[0][:3], seg[0][:3]) and np.allclose(out[-1][:3], seg[-1][:3])
    assert all(p[3:] == [4.4, -180.0, 4.4] for p in out)        # 자세는 그대로
    assert len(out) < 100                                      # 스플라인 한 번(100점 이내)


def test_straight_line_kept_straight():
    seg = [[500.0, -60.0 + 0.4 * i, 45.0, 0.0, -180.0, 0.0] for i in range(301)]
    out = smooth_pen_path(seg)
    assert all(abs(p[0] - 500.0) < 1e-9 for p in out)
    assert np.allclose(out[-1][:2], [500.0, 60.0])
