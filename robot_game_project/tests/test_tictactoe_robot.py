"""틱택토 로봇 X 그리기(tictactoe/tictactoe.py 의 RobotX) 테스트. 카메라·로봇 없이 합성 이미지로 확인한다."""
import json
from pathlib import Path
from runpy import run_path

import cv2
import numpy as np
import pytest

GAME = Path(__file__).resolve().parents[1] / 'robot_games' / 'tictactoe' / 'tictactoe.py'
H = np.array([[0.5, 0, 200], [0, 0.5, -150], [0, 0, 1]], float)            # 카메라 px → base mm
PIXELS = {0: (200.0, 150.0), 1: (1000.0, 150.0), 2: (1000.0, 600.0), 3: (200.0, 600.0)}


def marker_frame(centers, size=60, shape=(720, 1280)):
    img = np.full(shape, 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for mid, (cx, cy) in centers.items():
        x0, y0 = int(cx - size / 2), int(cy - size / 2)
        img[y0:y0 + size, x0:x0 + size] = cv2.aruco.generateImageMarker(d, mid, size)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


@pytest.fixture
def game(tmp_path):
    g = run_path(str(GAME), run_name='robot_games._tictactoe_game')
    field = {'markers': [{'id': i, 'base_xy': list(cv2.perspectiveTransform(np.float64([[p]]), H)[0, 0])}
                         for i, p in PIXELS.items()]}
    (tmp_path / 'field.json').write_text(json.dumps(field))
    cal = {'measured_and_verified': True, 'pen_contact_plane': [0.02, 0.0, 36.0], 'pen_press_mm': 1.0,
           'pen_up_z_base_mm': 80.0, 'tool_orientation_deg': [4.4, -180.0, 4.4]}
    (tmp_path / 'cal.json').write_text(json.dumps(cal))
    g['RobotX'].__init__.__globals__['FIELD_CALIB'] = tmp_path / 'field.json'
    return g, {'robot': {'drawing_calibration': str(tmp_path / 'cal.json')}}


def board_matrix(x0, y0, side):
    """카메라 px 정사각형 (x0,y0)~(x0+side,y0+side) 를 600x600 보드로 펴는 행렬 (grid_corners → perspective_board 와 같음)."""
    src = np.float32([(x0, y0), (x0 + side, y0), (x0 + side, y0 + side), (x0, y0 + side)])
    dst = np.float32([(0, 0), (599, 0), (599, 599), (0, 599)])
    return cv2.getPerspectiveTransform(src, dst)


def test_center_x_maps_to_cell_center_on_paper(game):
    g, cfg = game
    drawer = g['RobotX'](robot=None, config=cfg, frame=marker_frame(PIXELS), matrix=board_matrix(400, 200, 300))
    rows = drawer.rows(1, 1)
    assert [r['pen'] for r in rows] == ['up', 'down', 'down', 'up'] * 2          # 두 획
    xs = [r['x'] for r in rows]; ys = [r['y'] for r in rows]
    assert abs(np.mean(xs) - 475.0) < 1.0 and abs(np.mean(ys) - 25.0) < 1.0       # 가운데 칸 = 카메라 (550,350)
    down = [r for r in rows if r['pen'] == 'down']
    assert all(abs(r['z'] - (0.02 * r['x'] + 36.0 - 1.0)) < 1e-6 for r in down)  # 접촉면 - 누름
    assert all(r['z'] == 80.0 for r in rows if r['pen'] == 'up')


def test_refuses_outside_markers_and_unverified(game, tmp_path):
    g, cfg = game
    with pytest.raises(ValueError, match='작업 영역'):                             # 보드가 마커 사각형을 벗어남
        g['RobotX'](None, cfg, marker_frame(PIXELS), board_matrix(900, 450, 300)).rows(2, 2)
    cal = json.loads(Path(cfg['robot']['drawing_calibration']).read_text())
    cal['measured_and_verified'] = False
    Path(cfg['robot']['drawing_calibration']).write_text(json.dumps(cal))
    with pytest.raises(PermissionError):
        g['RobotX'](None, cfg, marker_frame(PIXELS), board_matrix(400, 200, 300))


def test_needs_four_markers(game):
    g, cfg = game
    with pytest.raises(RuntimeError, match='마커'):
        g['RobotX'](None, cfg, marker_frame({0: PIXELS[0], 1: PIXELS[1], 2: PIXELS[2]}), board_matrix(400, 200, 300))
