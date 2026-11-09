"""Camera-to-paper maze prototype. NEVER issues commands to physical actuators."""
import argparse
import json
from collections import deque
from pathlib import Path

import cv2
import numpy as np


def perspective_board(frame, corners, width=600, height=600):
    if len(corners) != 4:
        raise ValueError('Four ordered corners are required (TL TR BR BL).')
    src = np.asarray(corners, dtype=np.float32)
    dst = np.asarray([(0, 0), (width - 1, 0), (width - 1, height - 1), (0, height - 1)], dtype=np.float32)
    if abs(cv2.contourArea(src)) < 100:
        raise ValueError('Selected board corners are nearly collinear or too small.')
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, matrix, (width, height)), matrix


def candidate_walkable(board, invert=False, threshold=0, clearance=2):
    gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY) if board.ndim == 3 else board
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    if threshold > 0:
        _, binary = cv2.threshold(gray, threshold, 1, cv2.THRESH_BINARY)
    else:
        _, binary = cv2.threshold(gray, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    binary = binary.astype(np.uint8)
    if invert:
        binary = 1 - binary
    if clearance:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * clearance + 1, 2 * clearance + 1))
        binary = cv2.erode(binary, kernel, iterations=1)
    return binary


def crop_to_maze(board, invert=False, threshold=0, pad=3):
    wall = 1 - candidate_walkable(board, invert=invert, threshold=threshold, clearance=0)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(wall.astype(np.uint8), connectivity=8)
    if n <= 1:
        return board
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h = stats[big, :4]
    H, W = board.shape[:2]
    if w * h < 0.2 * W * H:
        return board
    return board[max(0, y - pad):min(H, y + h + pad), max(0, x - pad):min(W, x + w + pad)].copy()


def pick_endpoints(walk):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(walk.astype(np.uint8), connectivity=4)
    if n <= 1:
        raise ValueError('No walkable pixels. Try --invert or lower --clearance.')
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(labels == big)
    h, w = walk.shape
    i = int(np.argmin(xs + ys))
    j = int(np.argmin((w - 1 - xs) + (h - 1 - ys)))
    return (int(xs[i]), int(ys[i])), (int(xs[j]), int(ys[j]))


def snap_point(walkable, pt, max_radius=50):
    h, w = walkable.shape
    x, y = [int(round(v)) for v in pt]
    if not (0 <= x < w and 0 <= y < h):
        raise ValueError('Start/goal outside cropped maze.')
    if walkable[y, x]:
        return (x, y)
    for radius in range(1, max_radius + 1):
        for dy in range(-radius, radius + 1):
            for dx in (-radius, radius):
                xx, yy = x + dx, y + dy
                if 0 <= xx < w and 0 <= yy < h and walkable[yy, xx]:
                    return (xx, yy)
        for dx in range(-radius + 1, radius):
            for dy in (-radius, radius):
                xx, yy = x + dx, y + dy
                if 0 <= xx < w and 0 <= yy < h and walkable[yy, xx]:
                    return (xx, yy)
    raise ValueError('Start or finish not near a white corridor. Check maze contrast or clearance.')


def solve(board, start, goal, *, invert=False, threshold=0, clearance=2):
    walk = candidate_walkable(board, invert=invert, threshold=threshold, clearance=clearance)
    s, g = snap_point(walk, start), snap_point(walk, goal)
    h, w = walk.shape
    parent = np.full((h, w), -1, dtype=np.int32)
    queue = deque([s])
    parent[s[1], s[0]] = s[1] * w + s[0]
    while queue:
        x, y = queue.popleft()
        if (x, y) == g:
            break
        for nx, ny in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            if 0 <= nx < w and 0 <= ny < h and walk[ny, nx] and parent[ny, nx] < 0:
                parent[ny, nx] = y * w + x
                queue.append((nx, ny))
    if parent[g[1], g[0]] < 0:
        raise ValueError('No traversable route. Check paper_maze_walkable_pre.png; '
                         'adjust --clearance / --invert / --threshold or pass --start/--goal.')
    dense = []
    current = g[1] * w + g[0]
    start_idx = s[1] * w + s[0]
    while True:
        yy, xx = divmod(int(current), w)
        dense.append((xx, yy))
        if current == start_idx:
            break
        current = int(parent[yy, xx])
    dense.reverse()
    waypoints = [dense[0]]
    for i in range(1, len(dense) - 1):
        ax, ay = dense[i - 1]
        bx, by = dense[i]
        cx, cy = dense[i + 1]
        if (bx - ax, by - ay) != (cx - bx, cy - by) or i % 10 == 0:
            waypoints.append(dense[i])
    if dense[-1] != waypoints[-1]:
        waypoints.append(dense[-1])
    return waypoints, walk


def render(board, path):
    out = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR) if board.ndim == 2 else board.copy()
    for a, b in zip(path, path[1:]):
        cv2.line(out, tuple(a), tuple(b), (0, 0, 255), 2)
    cv2.circle(out, tuple(path[0]), 7, (0, 200, 0), -1)
    cv2.circle(out, tuple(path[-1]), 7, (255, 0, 0), -1)
    return out


def trajectory(path, image_shape):
    h, w = image_shape[:2]
    coords = [[round(x / max(1, w - 1), 6), round(y / max(1, h - 1), 6)] for x, y in path]
    return {
        'reference_frame': 'rectified_maze_image',
        'unit': 'fraction_of_image_width_height',
        'robot_coordinates_calibrated': False,
        'ready_for_execution': False,
        'pen_sequence': [
            {'action': 'move_above_start', 'xy': coords[0], 'pen': 'up'},
            {'action': 'lower_pen_at_start', 'xy': coords[0], 'pen': 'down'},
            *[{'action': 'trace', 'xy': p, 'pen': 'down'} for p in coords[1:]],
            {'action': 'raise_pen_at_finish', 'xy': coords[-1], 'pen': 'up'},
        ],
    }


def make_sample(seed=7, cells=7, scale=24):
    rng = np.random.default_rng(seed)
    walls = np.ones((2 * cells + 1, 2 * cells + 1), np.uint8)
    stack = [(0, 0)]
    visited = {(0, 0)}
    walls[1, 1] = 0
    while stack:
        r, c = stack[-1]
        neighbors = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                     if 0 <= r + dr < cells and 0 <= c + dc < cells and (r + dr, c + dc) not in visited]
        if not neighbors:
            stack.pop()
            continue
        rr, cc = neighbors[int(rng.integers(len(neighbors)))]
        walls[r + rr + 1, c + cc + 1] = 0
        walls[2 * rr + 1, 2 * cc + 1] = 0
        visited.add((rr, cc))
        stack.append((rr, cc))
    img = np.where(walls == 0, 255, 0).astype(np.uint8)
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return img, (int(1.5 * scale), int(1.5 * scale)), (int((2 * cells - .5) * scale), int((2 * cells - .5) * scale))


def auto_detect_board_corners(frame):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    screen = None
    for c in sorted(contours, key=cv2.contourArea, reverse=True):
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4 and cv2.contourArea(approx) > (w * h * 0.1):
            screen = approx
            break
    if screen is not None:
        pts = screen.reshape(4, 2)
        rect = np.zeros((4, 2), dtype='float32')
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect.tolist()
    mw, mh = int(w * 0.15), int(h * 0.15)
    return [[mw, mh], [w - mw, mh], [w - mw, h - mh], [mw, h - mh]]


def capture(camera_index):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open webcam {camera_index}')
    try:
        cv2.namedWindow('Maze camera', cv2.WINDOW_NORMAL)
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Webcam frame unavailable')
            preview = frame.copy()
            cv2.putText(preview, 'SPACE: auto-capture & solve | Q: quit', (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
            cv2.imshow('Maze camera', preview)
            key = cv2.waitKey(30) & 0xff
            if key == 32:
                return frame.copy()
            if key in (ord('q'), 27):
                raise KeyboardInterrupt('Capture canceled')
    finally:
        cap.release()
        cv2.destroyWindow('Maze camera')


def main(argv=None):
    p = argparse.ArgumentParser(description='Webcam paper maze -> path and pen trajectory preview; MOCK ONLY')
    p.add_argument('--source', choices=['camera', 'image', 'demo'], default='camera')
    p.add_argument('--camera-index', type=int, default=0)
    p.add_argument('--image')
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--start', type=int, nargs=2)
    p.add_argument('--goal', type=int, nargs=2)
    p.add_argument('--invert', action='store_true', help='Use dark corridors instead of white')
    p.add_argument('--threshold', type=int, default=0, help='0 = Otsu auto, >0 = fixed threshold')
    p.add_argument('--clearance', type=int, default=1)
    p.add_argument('--no-crop', action='store_true', help='Disable auto-crop to maze walls')
    p.add_argument('--output-dir', default='out')
    p.add_argument('--robot-mode', choices=['mock', 'real'], default='mock')
    p.add_argument('--approve', action='store_true',
                   help='Solved-path preview requires SPACE/ENTER to approve, Q/ESC to reject')
    args = p.parse_args(argv)
    if args.robot_mode != 'mock':
        raise PermissionError('Physical motion disabled: camera-to-robot calibration and safety are NOT commissioned')

    if args.source == 'demo':
        board, start, goal = make_sample(seed=args.seed)
    else:
        if args.source == 'image':
            if not args.image:
                p.error('--image is required with --source image')
            board = cv2.imread(args.image)
            if board is None:
                raise ValueError(f'Cannot open image: {args.image}')
        else:
            frame = capture(args.camera_index)
            board, _ = perspective_board(frame, auto_detect_board_corners(frame), width=600, height=600)
        if not args.no_crop:
            board = crop_to_maze(board, invert=args.invert, threshold=args.threshold)
        walk_temp = candidate_walkable(board, invert=args.invert, threshold=args.threshold, clearance=args.clearance)
        auto_s, auto_g = pick_endpoints(walk_temp)
        start = tuple(args.start) if args.start else auto_s
        goal = tuple(args.goal) if args.goal else auto_g

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / 'paper_maze_rectified.png'), board)
    cv2.imwrite(str(out / 'paper_maze_walkable_pre.png'),
                candidate_walkable(board, invert=args.invert, threshold=args.threshold,
                                   clearance=args.clearance) * 255)
    print(f'[MOCK] start={start} goal={goal} board={board.shape[1]}x{board.shape[0]}')

    path, walk = solve(board, start, goal, invert=args.invert, threshold=args.threshold, clearance=args.clearance)
    cv2.imwrite(str(out / 'paper_maze_walkable.png'), walk * 255)
    cv2.imwrite(str(out / 'paper_maze_solved.png'), render(board, path))
    plan = trajectory(path, board.shape)
    with open(out / 'paper_maze_pen_plan.json', 'w', encoding='utf-8') as fp:
        json.dump(plan, fp, indent=2)
    print(f'[MOCK] Camera/image maze solved: {len(path)} waypoints, start={path[0]}, finish={path[-1]}')
    print('[MOCK] Pen up -> approach -> pen down -> trace -> pen up -> home (simulation only)')
    print('Saved:', out / 'paper_maze_solved.png', out / 'paper_maze_pen_plan.json')
    print('REAL ARM NOT MOVED. No gripper movement performed.')

    #if args.source != 'demo':
    #    cv2.imshow('Solved maze / RED trajectory', render(board, path))
    #    if args.approve:
    #        print('Check RED path by eye.  SPACE/ENTER = approve and continue,  Q/ESC = reject.')
    #        key = cv2.waitKey(0) & 0xff
    #        cv2.destroyAllWindows()
    #        if key in (27, ord('q')):
    #            raise KeyboardInterrupt('Solved path rejected by operator')
    #    else:
    #        print('Press any key in preview window to close.')
    #        cv2.waitKey(0)
    #        cv2.destroyAllWindows()
    #return out / 'paper_maze_pen_plan.json'
    
    if args.source != 'demo':
        cv2.imshow('Solved maze / RED trajectory', render(board, path))
        cv2.waitKey(5000)  # 1초(1000ms) 동안만 표시하고 자동 진행
        cv2.destroyAllWindows()
    return out / 'paper_maze_pen_plan.json'


if __name__ == '__main__':
    main()
