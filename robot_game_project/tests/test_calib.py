"""재보정 도구(robot_games.calib) 테스트 — 예전 robot_games jmc 의 tests/test_pen_plan.py 에서 옮김."""
import json

import cv2
import numpy as np
import pytest

from robot_games.calib import pen_plan


def marker_frame(centers, size=60, shape=(720, 1280)):
    img = np.full(shape, 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(pen_plan.ARUCO_DICT)
    for mid, (cx, cy) in centers.items():
        m = cv2.aruco.generateImageMarker(d, mid, size)
        x0, y0 = int(cx - size / 2), int(cy - size / 2)
        img[y0:y0 + size, x0:x0 + size] = m
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


# 카메라 픽셀 → base: 1px = 0.5mm, 원점 (200, -150)
H = np.array([[0.5, 0, 200], [0, 0.5, -150], [0, 0, 1]], float)
PIXELS = {0: (200.0, 150.0), 1: (1000.0, 150.0), 2: (1000.0, 600.0), 3: (200.0, 600.0)}


def make_cal(tmp_path, verified=True):
    markers = [{'id': i, 'pixel': list(p), 'base_xy': list(cv2.perspectiveTransform(
        np.float64([[p]]), H)[0, 0])} for i, p in PIXELS.items()]
    cal = {'measured_and_verified': verified, 'H_camera_to_base': H.tolist(), 'markers': markers,
           'pen_contact_z_base_mm': 100.0, 'pen_up_z_base_mm': 130.0, 'tool_orientation_deg': [0, 180, 0]}
    path = tmp_path / 'cal.json'
    path.write_text(json.dumps(cal))
    return path


def test_board_to_base_composes_homographies():
    to_cam = pen_plan.translation(300, 200)               # 보드 (0,0) = 카메라 (300,200)
    xy = pen_plan.BoardToBase(to_cam, H)([(0, 0), (100, 0)])
    assert np.allclose(xy, [[350, -50], [400, -50]])


def test_calibration_requires_verified(tmp_path):
    with pytest.raises(pen_plan.PlanRefused):
        pen_plan.load_camera_calibration(make_cal(tmp_path, verified=False))


def test_camera_fixed_and_moved(tmp_path):
    cal = pen_plan.load_camera_calibration(make_cal(tmp_path))
    assert pen_plan.check_camera_fixed(marker_frame(PIXELS), cal) < 1.0
    moved = {k: (x + 8, y) for k, (x, y) in PIXELS.items()}
    with pytest.raises(pen_plan.PlanRefused, match='움직'):
        pen_plan.check_camera_fixed(marker_frame(moved), cal)


def test_workspace_rejects_outside(tmp_path):
    cal = pen_plan.load_camera_calibration(make_cal(tmp_path))
    seq = pen_plan.strokes_to_sequence([[(0, 0), (40, 0)]])
    inside = pen_plan.rows_from_camera_calibration(seq, pen_plan.translation(400, 300), cal)
    pen_plan.validate_rows(inside, workspace=pen_plan.workspace_polygon(cal))
    outside = pen_plan.rows_from_camera_calibration(seq, pen_plan.translation(1100, 300), cal)
    with pytest.raises(pen_plan.PlanRefused, match='작업 영역'):
        pen_plan.validate_rows(outside, workspace=pen_plan.workspace_polygon(cal))


def test_validate_rows_rejects_long_pen_down_jump():
    seq = pen_plan.strokes_to_sequence([[(0, 0), (200, 0)]])
    rows = pen_plan.sequence_to_rows(seq, lambda pts: np.asarray(pts, float) + [300, 0], 130, 100, [0, 180, 0])
    with pytest.raises(pen_plan.PlanRefused, match='점프'):
        pen_plan.validate_rows(rows)




def test_calibrate_homography_error():
    from robot_games.calib.calibrate import solve_homography
    px = [(200, 150), (1000, 150), (1000, 600), (200, 600), (600, 375)]
    base = cv2.perspectiveTransform(np.float64(px).reshape(-1, 1, 2), H).reshape(-1, 2)
    Hs, err = solve_homography(px, base)
    assert err.max() < 1e-6 and np.allclose(Hs / Hs[2, 2], H, atol=1e-6)
    base[4] += [6, 0]                                     # 가운데 마커 터치가 6mm 어긋남
    _, err = solve_homography(px, base)
    assert err.max() > 2.0


def test_a4_marker_sheet_detectable():
    from robot_games.calib.calibrate import a4_marker_sheet
    page = a4_marker_sheet(count=5, size_mm=50)
    assert page.shape == (3508, 2480)
    small = cv2.resize(page, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)  # 카메라처럼 축소
    assert sorted(pen_plan.detect_markers(small)) == [0, 1, 2, 3, 4]


def test_view_reports_missing_markers():
    from robot_games.calib.calibrate import annotate_markers
    _, ids = annotate_markers(marker_frame(PIXELS), expected=5)
    assert ids == [0, 1, 2, 3]
    view, ids = annotate_markers(marker_frame({**PIXELS, 4: (1100.0, 375.0)}), expected=5)
    assert ids == [0, 1, 2, 3, 4] and view.shape == (720, 1280, 3)


def test_predict_marker_hints_then_affine():
    from robot_games.calib.calibrate import predict_marker_xy
    pix = {i: p for i, p in PIXELS.items()}
    # 카메라 화면이 base 와 좌우 반전인 경우(실기): u 를 뒤집은 매핑
    base = {i: [p[1] * 0.9, -p[0] * 0.9 + 500] for i, p in pix.items()}
    hints = {0: [1.0, 2.0]}
    assert predict_marker_xy(0, pix, {}, hints) == [1.0, 2.0]                     # 힌트 우선
    assert predict_marker_xy(2, pix, {0: base[0], 1: base[1]}, {}) is None         # 2점으로는 예측하지 않음
    guess = predict_marker_xy(2, pix, {0: base[0], 1: base[1], 3: base[3]}, {})    # 3점 아핀은 반전도 맞춤
    assert np.allclose(guess, base[2], atol=0.5)


def test_contact_plane_follows_tilted_table(tmp_path):
    path = make_cal(tmp_path)
    cal = json.loads(path.read_text())
    for m in cal['markers']:                              # 먼 쪽(큰 X)일수록 2mm/100mm 높음
        m['base_z'] = 100.0 + 0.02 * (m['base_xy'][0] - 300)
    path.write_text(json.dumps(cal))
    cal = pen_plan.load_camera_calibration(path)
    zf = pen_plan.contact_z_fn(cal, press_mm=0.5)
    assert abs(zf(300, 0) - 99.5) < 0.01 and abs(zf(500, 0) - 103.5) < 0.01
    seq = pen_plan.strokes_to_sequence([[(0, 0), (40, 0)]])
    rows = pen_plan.rows_from_camera_calibration(seq, pen_plan.translation(400, 300), cal)
    pen_plan.validate_rows(rows, workspace=pen_plan.workspace_polygon(cal))
