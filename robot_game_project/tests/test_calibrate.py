"""보정 도구(robot_games.calibrate)와 마커 검출(paper_maze.detect_markers) 테스트. 카메라·로봇 없이 돈다."""
import cv2
import numpy as np

from robot_games import calibrate, paper_maze

# 카메라 픽셀 → base: 1px = 0.5mm, 원점 (200, -150)
H = np.array([[0.5, 0, 200], [0, 0.5, -150], [0, 0, 1]], float)
PIXELS = {0: (200.0, 150.0), 1: (1000.0, 150.0), 2: (1000.0, 600.0), 3: (200.0, 600.0)}


def marker_frame(centers, size=60, shape=(720, 1280)):
    img = np.full(shape, 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(calibrate.ARUCO_DICT)
    for mid, (cx, cy) in centers.items():
        x0, y0 = int(cx - size / 2), int(cy - size / 2)
        img[y0:y0 + size, x0:x0 + size] = cv2.aruco.generateImageMarker(d, mid, size)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_detect_markers_and_board_corners():
    frame = marker_frame(PIXELS)
    seen = paper_maze.detect_markers(frame)
    assert sorted(seen) == [0, 1, 2, 3]
    assert all(np.hypot(seen[i][0] - x, seen[i][1] - y) < 1.0 for i, (x, y) in PIXELS.items())
    corners = paper_maze.marker_board_corners(frame, ids=[0, 1, 2, 3])
    assert np.allclose(corners, [PIXELS[i] for i in (0, 1, 2, 3)], atol=1.0)
    assert paper_maze.marker_board_corners(frame, ids=[0, 1, 2, 4]) is None      # 4장이 다 보여야 함


def test_homography_error():
    px = [(200, 150), (1000, 150), (1000, 600), (200, 600), (600, 375)]
    base = cv2.perspectiveTransform(np.float64(px).reshape(-1, 1, 2), H).reshape(-1, 2)
    Hs, err = calibrate.solve_homography(px, base)
    assert err.max() < 1e-6 and np.allclose(Hs / Hs[2, 2], H, atol=1e-6)
    base[4] += [6, 0]                                     # 가운데 마커 터치가 6mm 어긋남
    _, err = calibrate.solve_homography(px, base)
    assert err.max() > 2.0


def test_fit_plane_tilted_table():
    pts = [(x, y, 36.0 + 0.02 * x) for x, y in ((450, -150), (450, 150), (660, 150), (660, -150), (555, -150))]
    a, b, c = calibrate.fit_plane(pts)
    assert abs(a - 0.02) < 1e-9 and abs(b) < 1e-9 and abs(c - 36.0) < 1e-6


def test_a4_marker_sheet_detectable():
    page = calibrate.a4_marker_sheet(count=5, size_mm=50)
    assert page.shape == (3508, 2480)
    small = cv2.resize(page, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)  # 카메라처럼 축소
    assert sorted(paper_maze.detect_markers(small)) == [0, 1, 2, 3, 4]


def test_view_reports_missing_markers():
    _, ids = calibrate.annotate_markers(marker_frame(PIXELS), expected=5)
    assert ids == [0, 1, 2, 3]
    view, ids = calibrate.annotate_markers(marker_frame({**PIXELS, 4: (1100.0, 375.0)}), expected=5)
    assert ids == [0, 1, 2, 3, 4] and view.shape == (720, 1280, 3)


def test_predict_marker_hints_then_affine():
    pix = dict(PIXELS)
    base = {i: [p[1] * 0.9, -p[0] * 0.9 + 500] for i, p in pix.items()}   # 카메라가 base 와 좌우 반전(실기)
    hints = {0: [1.0, 2.0]}
    assert calibrate.predict_marker_xy(0, pix, {}, hints) == [1.0, 2.0]                    # 힌트 우선
    assert calibrate.predict_marker_xy(2, pix, {0: base[0], 1: base[1]}, {}) is None        # 2점으로는 예측 안 함
    guess = calibrate.predict_marker_xy(2, pix, {0: base[0], 1: base[1], 3: base[3]}, {})   # 3점 아핀은 반전도 맞춤
    assert np.allclose(guess, base[2], atol=0.5)
