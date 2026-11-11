"""Camera tic-tac-toe preview for the existing action-card dispatcher."""

import json
from pathlib import Path
import random
import time

import cv2
import numpy as np

from .camera_select import choose_camera
from .motion import MockRobot
from .paper_maze import detect_markers, perspective_board
from .paper_tictactoe import cell_ink, detect_human_o, x_strokes
from .tictactoe import best_move, outcome, render_board
from .vision import tic_board_from_image


WINDOW = 'Tic-tac-toe | Camera preview'
RESULT_WINDOW = 'Tic-tac-toe | Result'
OUTPUT = Path(__file__).resolve().parents[2] / 'out/tictactoe_result.png'
CELL = 200
STABLE_SECONDS = .7
PREVIEW_SECONDS = .9
# [실기] 로봇이 X 를 종이에 그리는 경로 (mock 이면 화면에만)
ROOT = Path(__file__).resolve().parents[2]
FIELD_CALIB = ROOT / 'configs/field/camera_robot_calibration.json'   # 마커별 실측 로봇 좌표 (펜 끝 터치)
DRAW_CALIB = ROOT / 'configs/my_verified_calibration_real.json'       # 펜 높이(접촉면)·자세 — 미로와 같은 파일
MAX_REACH_XY_MM = 850.0
ROBOT_LABEL = 'Robot X: screen only'


def grid_corners(frame):
    """Find a closed quadrilateral containing two grid lines in each direction."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ink = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv2.THRESH_BINARY_INV, 31, 10)
    closed = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # The overhead camera sees the whole workbench, with the grid often under 2.5%.
    # Keep the four inner-line checks below to reject ordinary rectangles.
    minimum_area = max(900, .01 * gray.size)
    candidates = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(contour)
        if area < minimum_area:
            break
        if area > .95 * gray.size:
            continue
        quad = cv2.approxPolyDP(contour, .025 * cv2.arcLength(contour, True), True)
        if len(quad) != 4 or not cv2.isContourConvex(quad):
            continue
        points = quad.reshape(4, 2).astype(np.float32)
        sums = points.sum(axis=1)
        differences = points[:, 1] - points[:, 0]
        order = [sums.argmin(), differences.argmin(), sums.argmax(), differences.argmax()]
        if len(set(order)) != 4:
            continue
        points = points[order]
        grid, _ = perspective_board(ink, points, 300, 300)
        # Allow for slightly uneven spacing or slanted hand-drawn grid lines.
        bands = [grid[10:-10, p-18:p+19] for p in (100, 200)]
        bands += [grid[p-18:p+19, 10:-10].T for p in (100, 200)]
        support = [float(np.any(band > 128, axis=1).mean()) for band in bands]
        if min(support) >= .72:
            candidates.append((np.mean(support), area, points))
    return max(candidates, key=lambda candidate: candidate[:2])[2] if candidates else None


class StableObservation:
    def __init__(self):
        self.value = None
        self.since = 0.0
        self.frames = 0

    def update(self, value, same):
        now = time.monotonic()
        if value is None or self.value is None or not same(self.value, value):
            self.value, self.since, self.frames = value, now, 1
            return False
        self.frames += 1
        return self.frames >= 3 and now - self.since >= STABLE_SECONDS


def close_enough(first, second):
    return float(np.linalg.norm(first - second, axis=1).max()) <= 4


def read_frame(camera):
    ok, frame = camera.read()
    if not ok or frame is None:
        raise RuntimeError('Camera frame failed during tic-tac-toe')
    return frame


def show(image, message, board=None, planned=None):
    image = image.copy()
    if board is not None:
        for row, cells in enumerate(board):
            for col, mark in enumerate(cells):
                if mark == 'X':
                    for start, end in x_strokes(row, col):
                        cv2.line(image, start, end, (255, 110, 0), 5, cv2.LINE_AA)
    if planned is not None:
        row, col = planned
        cv2.rectangle(image, (col*CELL+12, row*CELL+12),
                      ((col+1)*CELL-12, (row+1)*CELL-12), (0, 190, 255), 4)
        for start, end in x_strokes(row, col):
            cv2.line(image, start, end, (0, 190, 255), 5, cv2.LINE_AA)
    view = cv2.copyMakeBorder(image, 64, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
    cv2.putText(view, message, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .52, (25, 25, 25), 1, cv2.LINE_AA)
    cv2.putText(view, f'{ROBOT_LABEL}  |  Human O: paper  |  Q: quit',
                (12, 50), cv2.FONT_HERSHEY_SIMPLEX, .44, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.imshow(WINDOW, view)
    check_quit(WINDOW)


def check_quit(window):
    key = cv2.waitKey(25) & 0xff
    if key in (ord('q'), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
        raise KeyboardInterrupt


def wait_for_board(camera):
    stable = StableObservation()
    while True:
        frame = read_frame(camera)
        corners = grid_corners(frame)
        status = 'Show a blank 3x3 grid with its outer border'
        if corners is not None:
            rectified, matrix = perspective_board(frame, corners)
            empty = all(
                cv2.countNonZero(ink) / ink.size < .035
                for row in range(3) for col in range(3)
                for ink in (cell_ink(rectified, row, col),)
            )
            ready = stable.update(corners if empty else None, close_enough)
            cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 210, 0), 2)
            status = 'Hold the blank board still' if empty else 'Clear all marks from the grid'
            show(frame, status)
            if ready:
                return corners, matrix, rectified
        else:
            stable.update(None, close_enough)
            show(frame, status)


def new_human_move(before, after, board):
    for row, cells in enumerate(board):
        for col, mark in enumerate(cells):
            if mark == ' ':
                continue
            old = cell_ink(before, row, col)
            new = cell_ink(after, row, col)
            added = cv2.bitwise_and(new, cv2.bitwise_not(old))
            added = cv2.morphologyEx(added, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
            if cv2.countNonZero(added) > 160:
                raise ValueError('Ink in a used cell')
    return detect_human_o(before, after, board)


def wait_for_o(camera, corners, matrix, before, board):
    stable = StableObservation()
    while True:
        frame = read_frame(camera)
        current = grid_corners(frame)
        if current is None or not close_enough(corners, current):
            stable.update(None, close_enough)
            show(frame, 'Restore the board and clear the camera view')
            continue

        image = cv2.warpPerspective(frame, matrix, before.shape[1::-1])
        move = None
        message = 'Your turn: draw one O in a free cell, then remove your hand'
        try:
            move = new_human_move(before, image, board)
        except ValueError as error:
            if str(error) == 'Ink in a used cell':
                message = 'That cell is used. Remove the new mark to continue'
        ready = stable.update(move, lambda previous, current: previous == current)
        if move is not None:
            message = f'Checking O: row {move[0]+1}, col {move[1]+1}'
        show(image, message, board)
        if ready:
            return move, image


class RobotX:
    """[실기] 로봇 X 차례에 종이에 X 를 그린다.
    보드 px(600x600) → inv(matrix) → 카메라 px → 마커 호모그래피 → 로봇 XY. 펜 Z 는 미로와 같은 접촉면 보정."""

    def __init__(self, robot, config, frame, matrix):
        rc = config['robot']
        path = Path(rc.get('drawing_calibration') or DRAW_CALIB)
        cal = json.loads((path if path.is_absolute() else ROOT / path).read_text(encoding='utf-8'))
        if cal.get('measured_and_verified') is not True:
            raise PermissionError(f'{path.name}: measured_and_verified 가 true 가 아님')
        missing = [k for k in ('pen_contact_plane', 'pen_up_z_base_mm', 'tool_orientation_deg') if not cal.get(k)]
        if missing:
            raise ValueError(f'{path.name}: {missing} 없음')
        base = {m['id']: m['base_xy'] for m in json.loads(FIELD_CALIB.read_text(encoding='utf-8'))['markers']}
        seen = detect_markers(frame)
        ids = [i for i in sorted(seen) if i in base]
        if len(ids) < 4:
            raise RuntimeError(f'보정 마커가 4개 이상 보여야 합니다 (보이는 마커 {sorted(seen)})')
        self.H, _ = cv2.findHomography(np.float64([seen[i] for i in ids]), np.float64([base[i] for i in ids]), 0)
        self.hull = cv2.convexHull(np.float32(list(base.values())))      # 펜이 내려가도 되는 영역
        self.robot, self.cal, self.to_camera = robot, cal, np.linalg.inv(matrix)

    def rows(self, row, col):
        """X 두 획을 maze_csv 와 같은 형식의 행(dict)으로: 위 이동 → 펜 내림 → 끝점까지 → 펜 올림."""
        a, b, c = self.cal['pen_contact_plane']
        press, up = float(self.cal.get('pen_press_mm', 0.5)), float(self.cal['pen_up_z_base_mm'])
        rx, ry, rz = (float(v) for v in self.cal['tool_orientation_deg'])
        out = []
        for stroke in x_strokes(row, col):
            cam = cv2.perspectiveTransform(np.float64(stroke).reshape(-1, 1, 2), self.to_camera)
            xy = cv2.perspectiveTransform(cam, self.H).reshape(-1, 2)
            for x, y in xy:
                if cv2.pointPolygonTest(self.hull, (float(x), float(y)), True) < -1.0:
                    raise ValueError(f'X 획이 마커 작업 영역 밖 ({x:.0f}, {y:.0f})')
                if np.hypot(x, y) > MAX_REACH_XY_MM:
                    raise ValueError(f'X 획이 도달 범위 밖 ({x:.0f}, {y:.0f})')
            (x0, y0), (x1, y1) = xy
            pt = lambda x, y, z, pen, act: {'action': act, 'x': float(x), 'y': float(y), 'z': float(z), 'pen': pen,
                                            'rx': rx, 'ry': ry, 'rz': rz}
            out += [pt(x0, y0, up, 'up', 'move_above_start'),
                    pt(x0, y0, a * x0 + b * y0 + c - press, 'down', 'lower_pen'),
                    pt(x1, y1, a * x1 + b * y1 + c - press, 'down', 'trace'),
                    pt(x1, y1, up, 'up', 'raise_pen')]
        return out

    def draw(self, row, col):
        self.robot.run_rows(self.rows(row, col))
        self.robot.home()                                     # 카메라 시야에서 비킴


def refresh_board(camera, corners, matrix, before, move, timeout=10.0):
    """[실기] 로봇이 X 를 그리고 비킨 뒤 보드 이미지를 다시 찍는다. 새 X 잉크가 들어간 이미지를 기준으로 삼아야
    new_human_move 가 'Ink in a used cell' 로 사람 O 를 막지 않는다."""
    for _ in range(10):                                       # 로봇이 움직이는 동안 쌓인 이전 프레임 버림
        camera.read()
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        frame = read_frame(camera)
        current = grid_corners(frame)
        if current is not None and close_enough(corners, current):
            image = cv2.warpPerspective(frame, matrix, before.shape[1::-1])
            row, col = move
            added = cv2.bitwise_and(cell_ink(image, row, col), cv2.bitwise_not(cell_ink(before, row, col)))
            if cv2.countNonZero(added) < 160:
                print('[TIC] 경고: 로봇이 그린 X 가 카메라에 잘 안 보입니다 (펜 높이·잉크 확인)', flush=True)
            return image
        show(frame, 'Robot moved away - looking for the board again')
    raise RuntimeError('로봇이 그린 뒤 격자를 다시 찾지 못함 (팔이 가리거나 종이가 움직임)')


def pause_preview(image, message, board, planned=None, seconds=PREVIEW_SECONDS):
    until = time.monotonic() + seconds
    while True:
        show(image, message, board, planned)
        if time.monotonic() >= until:
            return


def save_result(board):
    OUTPUT.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(OUTPUT), render_board(board))


def show_result(board, result):
    messages = {'X': 'ROBOT WINS', 'O': 'YOU WIN', 'DRAW': 'DRAW'}
    message = messages[result]
    print(f'[TIC] {message}', flush=True)
    image = cv2.copyMakeBorder(render_board(board), 75, 0, 0, 0, cv2.BORDER_CONSTANT,
                               value=(245, 245, 245))
    cv2.putText(image, f'{message}  |  Q: close', (16, 47), cv2.FONT_HERSHEY_SIMPLEX,
                .85, (35, 35, 35), 2, cv2.LINE_AA)
    cv2.namedWindow(RESULT_WINDOW, cv2.WINDOW_NORMAL)
    cv2.imshow(RESULT_WINDOW, image)
    while True:
        check_quit(RESULT_WINDOW)


def play(camera, first=None, make_drawer=None):
    corners, matrix, image = wait_for_board(camera)
    drawer = make_drawer(read_frame(camera), matrix) if make_drawer else None   # [실기] 로봇 X 그리기
    board = [[' '] * 3 for _ in range(3)]
    turn = first or random.choice(('X', 'O'))
    message = 'Robot X starts' if turn == 'X' else 'Human O starts'
    print(f'[TIC] {message}', flush=True)
    pause_preview(image, message, board)
    while outcome(board) is None:
        if turn == 'X':
            row, col = best_move(board, robot='X', human='O')
            message = f'Robot plans X: row {row+1}, col {col+1}'
            print(f'[TIC] {message}', flush=True)
            pause_preview(image, message, board, (row, col))
            if drawer is not None:
                drawer.draw(row, col)
                image = refresh_board(camera, corners, matrix, image, (row, col))
        else:
            (row, col), image = wait_for_o(camera, corners, matrix, image, board)
            print(f'[TIC] Accepted O: row {row+1}, col {col+1}', flush=True)
        board[row][col] = turn
        turn = 'O' if turn == 'X' else 'X'
    return board, outcome(board)


def run(args, robot, config):
    global ROBOT_LABEL
    real = not isinstance(robot, MockRobot)        # [실기] 실기 로봇이면 X 를 종이에 그린다
    if real and args.image:
        raise ValueError('실기 틱택토는 카메라로만 진행합니다 (--image 불가)')
    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            raise ValueError(f'Could not load {args.image}')
        board = tic_board_from_image(image)
        move = best_move(board)
        if move is not None:
            row, col = move
            board[row][col] = 'O'
        save_result(board)
        print('Winner:', outcome(board), 'Move:', move)
        return

    settings = config['camera']
    index = choose_camera(int(settings.get('index', 0)))   # [실기] RealSense 컬러를 자동으로 (설정 index 는 fallback)
    camera = cv2.VideoCapture(index)
    try:
        if not camera.isOpened():
            raise RuntimeError(f"Cannot reopen camera /dev/video{index}")
        for key, prop in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT)):
            if settings.get(key):
                camera.set(prop, settings[key])
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 720, 750)
        ROBOT_LABEL = 'Robot X: drawn by robot' if real else 'Robot X: screen only'
        make_drawer = (lambda frame, matrix: RobotX(robot, config, frame, matrix)) if real else None
        board, result = play(camera, make_drawer=make_drawer)
        save_result(board)
        show_result(board, result)
    except KeyboardInterrupt:
        print('[TIC] Preview closed', flush=True)
    finally:
        camera.release()
        cv2.destroyAllWindows()
