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


def crop_to_maze(board, invert=False, threshold=0, pad=3, return_offset=False):
    # [실기] return_offset=True 면 (잘린 이미지, (x0, y0)) — 펜 좌표를 보드 전체 기준으로 되돌릴 때 필요
    wall = 1 - candidate_walkable(board, invert=invert, threshold=threshold, clearance=0)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(wall.astype(np.uint8), connectivity=8)
    H, W = board.shape[:2]
    out, off = board, (0, 0)
    # [실기] 입구·출구가 바깥 벽을 여러 조각으로 나누므로 가장 큰 덩어리 하나가 아니라 큰 벽 덩어리들을 모두 포함한다.
    # 보드 모서리 마커 조각처럼 이미지 가장자리에 닿은 덩어리는 제외한다.
    keep = [k for k in range(1, n) if stats[k, 0] > 0 and stats[k, 1] > 0
            and stats[k, 0] + stats[k, 2] < W and stats[k, 1] + stats[k, 3] < H]
    if keep:
        biggest = max(stats[k, cv2.CC_STAT_AREA] for k in keep)
        keep = [k for k in keep if stats[k, cv2.CC_STAT_AREA] >= 0.1 * biggest]
    if keep:
        x = min(stats[k, 0] for k in keep); y = min(stats[k, 1] for k in keep)
        w = max(stats[k, 0] + stats[k, 2] for k in keep) - x; h = max(stats[k, 1] + stats[k, 3] for k in keep) - y
        if w * h >= 0.2 * W * H:
            x0, y0 = max(0, x - pad), max(0, y - pad)
            out, off = board[y0:min(H, y + h + pad), x0:min(W, x + w + pad)].copy(), (int(x0), int(y0))
    return (out, off) if return_offset else out


def find_openings(board, invert=False, threshold=0, min_gap=10):
    """[실기] 미로 바깥 테두리 벽이 끊긴 곳(입구·출구)을 찾는다.
    pick_endpoints 는 이미지 모서리에 가까운 흰 점을 골라서, 크롭 여백(벽 바깥)을 시작·도착점으로 잡고
    미로 바깥 테두리를 도는 경로를 냈다(2026-11-11 실측). 그래서 네 변마다 바깥 벽의 중심선을 찾고, 그 선 위에서
    벽이 min_gap 픽셀 이상 끊긴 구간을 입구·출구로 본다.
    반환: (start, goal, inside, start_ext, goal_ext)
      start/goal  : 구멍에서 벽 안쪽으로 들어간 점 (BFS 시작·끝)
      inside      : (x0, y0, x1, y1) 바깥 벽 중심선 상자 (이 밖은 막는다)
      start_ext   : [구멍 바깥 점, 구멍 가운데] — 경로 앞에 붙인다 (입구 밖에서 시작)
      goal_ext    : [구멍 가운데, 구멍 바깥 점] — 경로 뒤에 붙인다 (출구 밖에서 끝)
    바깥 점은 크롭 이미지 밖(음수 좌표)일 수 있다. trajectory 가 크롭 오프셋을 더해 보드 전체 좌표로 바꾼다.
    두 곳을 못 찾으면 None."""
    wall = 1 - candidate_walkable(board, invert=invert, threshold=threshold, clearance=0)
    h, w = wall.shape
    depth = max(4, min(h, w) // 8)

    def best_line(profile_fn, n):
        fr = [profile_fn(k).mean() for k in range(n)]
        k = int(np.argmax(fr))
        return k, fr[k]
    top, ft = best_line(lambda k: wall[k, :], depth)
    bot, fb = best_line(lambda k: wall[h - 1 - k, :], depth)
    left, fl = best_line(lambda k: wall[:, k], depth)
    right, fr_ = best_line(lambda k: wall[:, w - 1 - k], depth)
    if min(ft, fb, fl, fr_) < 0.5:                  # 바깥 벽이 뚜렷하지 않음
        return None
    y0, y1, x0, x1 = top, h - 1 - bot, left, w - 1 - right
    inward = 3 * max(top, bot, left, right, 3)      # 벽 두께보다 충분히 안쪽
    outward = inward                                # 구멍 바깥으로 나가는 거리 (벽 중심선 기준)

    gaps = []
    def scan(line, fixed, axis, normal, lo, hi):
        run = None
        for i in range(lo, hi + 1):
            free = line[i] == 0
            if free and run is None:
                run = i
            if (not free or i == hi) and run is not None:
                end = i if free else i - 1
                if end - run + 1 >= min_gap:
                    mid = (run + end) // 2
                    p = (mid, fixed) if axis == 'x' else (fixed, mid)
                    q = (p[0] + normal[0] * inward, p[1] + normal[1] * inward)
                    o = (p[0] - normal[0] * outward, p[1] - normal[1] * outward)
                    gaps.append((end - run + 1, (int(np.clip(q[0], 0, w - 1)), int(np.clip(q[1], 0, h - 1))),
                                 (int(p[0]), int(p[1])), (int(o[0]), int(o[1]))))
                run = None
    scan(wall[y0, :], y0, 'x', (0, 1), x0, x1)
    scan(wall[y1, :], y1, 'x', (0, -1), x0, x1)
    scan(wall[:, x0], x0, 'y', (1, 0), y0, y1)
    scan(wall[:, x1], x1, 'y', (-1, 0), y0, y1)
    if len(gaps) < 2:
        return None
    two = sorted(gaps, key=lambda g: g[0], reverse=True)[:2]
    two.sort(key=lambda g: g[1][0] + g[1][1])       # 왼쪽 위에 가까운 쪽이 시작
    (_, s_in, s_gap, s_out), (_, g_in, g_gap, g_out) = two
    return s_in, g_in, (x0, y0, x1, y1), [s_out, s_gap], [g_gap, g_out]


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


def solve(board, start, goal, *, invert=False, threshold=0, clearance=2, inside=None):
    walk = candidate_walkable(board, invert=invert, threshold=threshold, clearance=clearance)
    if inside is not None:                          # [실기] 바깥 벽 밖 여백을 막아 경로가 미로 바깥을 돌지 못하게
        x0, y0, x1, y1 = inside
        mask = np.zeros_like(walk)
        mask[y0:y1 + 1, x0:x1 + 1] = 1
        walk = walk * mask
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


def solve_centered(board, start, goal, *, invert=False, threshold=0, inside=None, min_clearance=1, max_clearance=40):
    """[실기] 벽에서 최대한 떨어진 경로. clearance=1 이면 최단 경로가 모서리마다 벽에 붙어(약 0.4mm) 펜이 벽 선 위를
    겹쳐 그렸다(2026-11-11). 시작·도착이 이어지는 가장 큰 clearance 를 이분 탐색으로 찾아 그 값으로 푼다.
    반환: (path, walk, clearance)"""
    def attempt(c):
        try:
            return solve(board, start, goal, invert=invert, threshold=threshold, clearance=c, inside=inside)
        except ValueError:
            return None
    best = attempt(min_clearance)
    if best is None:
        raise ValueError('No traversable route even with minimum clearance.')
    lo, hi, best_c = min_clearance, max_clearance, min_clearance
    while lo < hi:
        mid = (lo + hi + 1) // 2
        r = attempt(mid)
        if r is not None:
            lo, best, best_c = mid, r, mid
        else:
            hi = mid - 1
    return best[0], best[1], best_c


def render(board, path):
    out = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR) if board.ndim == 2 else board.copy()
    for a, b in zip(path, path[1:]):
        cv2.line(out, tuple(a), tuple(b), (0, 0, 255), 2)
    cv2.circle(out, tuple(path[0]), 7, (0, 200, 0), -1)
    cv2.circle(out, tuple(path[-1]), 7, (255, 0, 0), -1)
    return out


def trajectory(path, image_shape, offset=(0, 0)):
    # [실기] image_shape 은 보드 전체(원근 보정한 600x600) 크기, offset 은 크롭 시작점.
    # 보정(my_verified_calibration_real.json)이 보드 모서리 기준이므로 크롭 이미지가 아니라 보드 전체 기준으로 정규화한다.
    h, w = image_shape[:2]
    ox, oy = offset
    coords = [[round((x + ox) / max(1, w - 1), 6), round((y + oy) / max(1, h - 1), 6)] for x, y in path]
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


def marker_board_corners(frame, ids=None):
    """[실기] 테이블에 고정한 ArUco 마커 4장의 중심을 보드 모서리(TL, TR, BR, BL)로 쓴다.
    흰 종이가 흰 테이블 위에 작게(화면의 ~4%) 놓이면 auto_detect_board_corners 가 종이를 못 찾고
    고정 사각형을 돌려준다(2026-11-11 실측). 마커는 항상 보이고 로봇 좌표를 펜으로 직접 재 두었으므로
    보정(my_verified_calibration_real.json)도 이 네 점 기준이다. 4장이 다 안 보이면 None."""
    import yaml
    if ids is None:
        cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / 'configs' / 'default.yaml').read_text(encoding='utf-8'))
        ids = (cfg.get('vision') or {}).get('board_marker_ids')
    if not ids:
        return None
    centers = detect_markers(frame)
    if not all(int(i) in centers for i in ids):
        return None
    return [list(centers[int(i)]) for i in ids]


def detect_marker_corners(frame):
    """[실기] ArUco(DICT_4X4_50) 마커 모서리 {id: 4x2 배열} 카메라 픽셀 (마커 기준 TL, TR, BR, BL 순)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
    corners, found, _ = det.detectMarkers(gray)
    if found is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(float) for i, c in zip(found.ravel(), corners)}


def detect_markers(frame):
    """[실기] ArUco(DICT_4X4_50) 마커 중심 {id: (x, y)} 카메라 픽셀. 보드 모서리와 보정 도구(calibrate.py)가 같이 쓴다."""
    return {i: tuple(float(v) for v in c.mean(axis=0)) for i, c in detect_marker_corners(frame).items()}


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


def capture(camera_index=None):
    if camera_index is None:                      # [실기] 번호를 안 주면 실행할 때마다 RealSense 컬러를 찾는다
        from .camera_select import choose_camera
        camera_index = choose_camera()
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open webcam {camera_index}')
    # [실기] 카드·틱택토와 같이 default.yaml 의 camera.width/height 를 쓴다 (1280x720: 1px ≈ 1mm, 보정과 같은 해상도)
    import yaml
    cam = yaml.safe_load((Path(__file__).resolve().parents[1] / 'configs' / 'default.yaml').read_text(encoding='utf-8')).get('camera') or {}
    for key, prop in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT)):
        if cam.get(key):
            cap.set(prop, cam[key])
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
    p.add_argument('--camera-index', type=int, default=None, help='생략하면 RealSense 컬러 카메라를 자동으로 찾음')
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

    crop_off, inside, start_ext, goal_ext = (0, 0), None, [], []
    if args.source == 'demo':
        board, start, goal = make_sample(seed=args.seed)
        full_shape = board.shape
    else:
        if args.source == 'image':
            if not args.image:
                p.error('--image is required with --source image')
            board = cv2.imread(args.image)
            if board is None:
                raise ValueError(f'Cannot open image: {args.image}')
        else:
            frame = capture(args.camera_index)
            corners = marker_board_corners(frame)           # [실기] 마커 4장이 보이면 그것을 보드 모서리로
            if corners is None:
                print('[BOARD] 보드 마커 4장이 다 보이지 않음 → 종이 자동 검출 사용')
                corners = auto_detect_board_corners(frame)
            board, _ = perspective_board(frame, corners, width=600, height=600)
        full_shape = board.shape
        if not args.no_crop:
            board, crop_off = crop_to_maze(board, invert=args.invert, threshold=args.threshold, return_offset=True)
        walk_temp = candidate_walkable(board, invert=args.invert, threshold=args.threshold, clearance=args.clearance)
        found = find_openings(board, invert=args.invert, threshold=args.threshold)
        if found is not None:
            auto_s, auto_g, inside, start_ext, goal_ext = found
            print(f'[MAZE] 입구 {auto_s} → 출구 {auto_g}')
        else:
            print('[MAZE] 입구·출구를 찾지 못함 → 이미지 모서리 기준 자동 선택')
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

    if args.source != 'demo' and args.clearance <= 1:   # [실기] 기본값이면 통로 가운데로 가도록 clearance 자동
        path, walk, used = solve_centered(board, start, goal, invert=args.invert, threshold=args.threshold,
                                          inside=None if (args.start or args.goal) else inside)
        print(f'[MAZE] 벽에서 떨어진 거리 {used}px 로 풀이')
        if not (args.start or args.goal):          # [실기] 입구 구멍 밖에서 시작해 출구 구멍 밖에서 끝나게
            path = [tuple(p) for p in start_ext] + list(path) + [tuple(p) for p in goal_ext]
    else:
        path, walk = solve(board, start, goal, invert=args.invert, threshold=args.threshold, clearance=args.clearance,
                           inside=None if (args.start or args.goal) else inside)
    cv2.imwrite(str(out / 'paper_maze_walkable.png'), walk * 255)
    cv2.imwrite(str(out / 'paper_maze_solved.png'), render(board, path))
    plan = trajectory(path, full_shape, crop_off)
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
