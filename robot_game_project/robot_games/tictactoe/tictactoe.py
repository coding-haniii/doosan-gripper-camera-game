"""Camera tic-tac-toe preview for the existing action-card dispatcher."""

from pathlib import Path
import random
import time

import cv2
import numpy as np

from .motion import MockRobot
from .paper_maze import perspective_board
from .paper_tictactoe import cell_ink, detect_human_o, x_strokes
from .tictactoe import best_move, outcome, render_board
from .vision import tic_board_from_image


WINDOW = 'Tic-tac-toe | Camera preview'
RESULT_WINDOW = 'Tic-tac-toe | Result'
OUTPUT = Path(__file__).resolve().parents[2] / 'out/tictactoe_result.png'
CELL = 200
STABLE_SECONDS = .7
PREVIEW_SECONDS = .9


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
    cv2.putText(view, 'Robot X: screen only  |  Human O: paper  |  Q: quit',
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


def play(camera, first=None):
    corners, matrix, image = wait_for_board(camera)
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
        else:
            (row, col), image = wait_for_o(camera, corners, matrix, image, board)
            print(f'[TIC] Accepted O: row {row+1}, col {col+1}', flush=True)
        board[row][col] = turn
        turn = 'O' if turn == 'X' else 'X'
    return board, outcome(board)


def run(args, robot, config):
    if not isinstance(robot, MockRobot):
        raise PermissionError('Tic-tac-toe is currently a camera preview. Use mock mode.')
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
    camera = cv2.VideoCapture(settings['index'])
    try:
        if not camera.isOpened():
            raise RuntimeError(f"Cannot reopen camera /dev/video{settings['index']}")
        for key, prop in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT)):
            if settings.get(key):
                camera.set(prop, settings[key])
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 720, 750)
        board, result = play(camera)
        save_result(board)
        show_result(board, result)
    except KeyboardInterrupt:
        print('[TIC] Preview closed', flush=True)
    finally:
        camera.release()
        cv2.destroyAllWindows()
