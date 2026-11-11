"""보정·교시 도구.  python -m robot_games.calibrate <명령>   (결과는 configs/field/)

  markers       ArUco 마커 생성 (out/markers/). markers_A4.pdf(.png) 를 100% 로 인쇄해 잘라서
                종이 보드 네 모서리 바깥(0~3)과 한 변 가운데 바깥(4)에 붙인다. 마커는 실행 중에도
                모두 보여야 하므로 종이 밑에 두면 안 된다. --size-mm 로 한 변 크기(기본 50mm).
  view          웹캠 화면에 인식된 마커를 표시 (로봇 연결 없음). 마커를 붙이거나 카메라 위치를 잡을 때 쓴다.
  camera-robot  펜 끝을 각 마커 중심에 대고 Enter → 로봇 좌표와 카메라 픽셀로 H_camera_to_base 계산
                → configs/field/camera_robot_calibration.json  (미로 보정은 scripts/make_maze_calibration.py 로 다시 만든다)
  poses         home / celebration / camera_clear(관절) 자세 교시 → configs/field/robot_poses.json
  gripper       플랜지 그리퍼 시험: --goal N / --torque on|off / --read  (펜 넣고 닫기: --goal 0 → --goal 620)

로봇을 옮기는 방법
  --jog  : 키보드 스텝 조그(jog.py). 이 로봇(E0509, 펜던트·콕핏 없음)은 직접 교시가 안 되므로 이 방식을 쓴다.
           --execute-motion 과 default.yaml 의 robot.armed=true 가 필요하다.
로봇 연결은 팀원 motion.GuardedDoosan(for_calibration=True)을 쓴다 (실제 컨트롤러만, armed 필요).
  (없음) : 위치를 읽기만 한다. 운영자가 다른 수단(DART 조그 등)으로 옮긴 뒤 Enter.
결과 파일은 항상 measured_and_verified=false 로 저장된다. 확인 후 사람이 직접 true 로 바꾼다.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from .camera_select import choose_camera
from .paper_maze import detect_markers

ROOT = Path(__file__).resolve().parents[1]
FIELD = ROOT / 'configs' / 'field'
CAMERA_CALIB_PATH = FIELD / 'camera_robot_calibration.json'
POSES_PATH = FIELD / 'robot_poses.json'
JOINT_KEYS = ('home', 'celebration', 'camera_clear')
ARUCO_DICT = cv2.aruco.DICT_4X4_50
PEN_UP_CLEARANCE_MM = 30.0
WARN_ERR_MM = 2.0
MAX_ERR_MM = 5.0
WARN_Z_SPREAD_MM = 2.0


def _cfg(path=None):
    return yaml.safe_load(Path(path or ROOT / 'configs' / 'default.yaml').read_text(encoding='utf-8'))


def _robot(cfg):
    """팀원 GuardedDoosan 으로 실제 컨트롤러에 연결 (보정 전이라 calibration_confirmed·home 은 요구하지 않음)."""
    from .motion import GuardedDoosan
    return GuardedDoosan(cfg['robot'], need_celebration=False, for_calibration=True)


def _open_camera(cfg, index=None):
    cam = cfg.get('camera') or {}
    cap = cv2.VideoCapture(choose_camera(int(cam.get('index', 0))) if index is None else index)
    for key, prop in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT)):
        if cam.get(key):
            cap.set(prop, cam[key])
    if not cap.isOpened():
        raise RuntimeError('카메라를 열 수 없음')
    for _ in range(10):
        cap.read()
    return cap


def load_poses(path=None) -> dict:
    p = Path(path or POSES_PATH)
    if not p.exists():
        return {}
    data = json.loads(p.read_text(encoding='utf-8'))
    for k in JOINT_KEYS:
        if data.get(k) is not None and len(data[k]) != 6:
            raise ValueError(f'{p.name}: {k} 는 값 6개여야 합니다 ({data[k]!r})')
    return data


def fit_plane(points):
    """[(x, y, z), ...] → z = a·x + b·y + c 의 (a, b, c). 점이 3개 미만이면 평균 높이의 수평면."""
    pts = np.asarray(points, float)
    if len(pts) < 3:
        return (0.0, 0.0, float(pts[:, 2].mean()))
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    a, b, c = np.linalg.lstsq(A, pts[:, 2], rcond=None)[0]
    return (float(a), float(b), float(c))


def _jog_panel(a, cfg, robot, input_fn):
    """--jog 면 키보드 스텝 조그 패널, 아니면 None (위치만 읽는 모드)."""
    if not getattr(a, 'jog', False):
        return None
    if not a.execute_motion:
        raise PermissionError('조그는 --execute-motion 이 필요합니다 (default.yaml 의 robot.armed=true 도 필요)')
    from .jog import JogPanel
    panel = JogPanel(robot, min_z=cfg['robot'].get('min_pen_z_base_mm'), input_fn=input_fn)
    panel.check_frames()
    return panel


def _positioner(a, cfg, robot, input_fn):
    """wait(prompt): 운영자가 로봇을 원하는 위치에 둘 때까지 기다린다. --jog 면 키보드 스텝 조그."""
    panel = _jog_panel(a, cfg, robot, input_fn)
    if panel is None:
        return lambda prompt: input_fn(prompt + ' 후 Enter > ')
    return panel.position


# ------------------------------------------------------------------ 마커 위 자동 접근 (camera-robot --jog)
HINTS_PATH = ROOT / 'configs' / 'field' / 'marker_hints.json'
HOVER_MM = 12.0          # 자동 접근 때 펜 끝을 바닥 하한(min_pen_z) 위 이만큼에 둔다
CLEAR_MM = 50.0          # 마커 사이를 옮길 때 높이 (hover 위로). 높으면 먼 마커에 팔이 닿지 않는다


def load_hints(path=None) -> dict:
    """마커를 붙일 때 로봇이 짚었던 대략 위치 {id: [x, y]} (base, mm). 없으면 {}."""
    p = Path(path or HINTS_PATH)
    if not p.exists():
        return {}
    data = json.loads(p.read_text(encoding='utf-8'))
    return {int(k): [float(v[0]), float(v[1])] for k, v in data.get('markers', {}).items()}


def predict_marker_xy(mid, pixel_of, touched, hints):
    """다음 마커의 대략 base XY. hints(붙일 때 로봇이 짚은 위치)가 있으면 그것을, 없으면 이미 찍은 마커가
    3개 이상일 때 카메라 픽셀 → base 아핀 변환으로 예측한다. 둘 다 안 되면 None(직접 조그).
    닮음변환(2점)은 쓰지 않는다: 카메라 화면이 base 좌표와 좌우 반전이라 반대쪽으로 예측된다(2026-11-10 실기)."""
    if mid in hints:
        return hints[mid]
    if len(touched) >= 3:
        src = np.float32([pixel_of[m] for m in touched])
        dst = np.float32([touched[m] for m in touched])
        M, _ = cv2.estimateAffine2D(src, dst)
        if M is not None:
            u, v = pixel_of[mid]
            return [float(M[0, 0] * u + M[0, 1] * v + M[0, 2]), float(M[1, 0] * u + M[1, 1] * v + M[1, 2])]
    return None


def _approach(robot, xy, rc):
    """현재 위치에서 이동 높이(clear)로 수직 이동 → 마커 위로 수평 이동 → hover 높이까지 천천히 하강.
    높은 곳(예: camera_clear 의 20cm)에서는 팔이 먼 마커까지 닿지 않으므로 먼저 clear 높이로 내려온다."""
    from .jog import jog_speed
    p = robot.current_posx()
    floor = float(rc.get('min_pen_z_base_mm') or 40.0)
    hover, clear = floor + HOVER_MM, floor + HOVER_MM + CLEAR_MM
    fast, acc = jog_speed(rc)
    slow = min(100.0, fast)                          # 테이블 쪽으로 오르내릴 때는 천천히
    if abs(p[2] - clear) > 1.0:
        robot.move_line([p[0], p[1], clear, *p[3:6]], slow, acc)
    robot.move_line([xy[0], xy[1], clear, *p[3:6]], fast, acc)
    robot.move_line([xy[0], xy[1], hover, *p[3:6]], slow, acc)


def _to_camera_clear(robot, rc):
    """camera_clear 의 직교 위치까지 직선으로 올라가며 이동한 뒤 관절 자세를 맞춘다.
    (낮은 곳에서 바로 관절 이동을 하면 도중에 펜 끝이 테이블 쪽으로 내려갈 수 있음)"""
    from .jog import jog_speed
    q = load_poses()['camera_clear']
    robot.move_line(robot.fkin(q), *jog_speed(rc))
    robot.move_joint(q)


def _lift(robot, rc):
    from .jog import jog_speed
    p = robot.current_posx()
    floor = float(rc.get('min_pen_z_base_mm') or 40.0)
    clear = floor + HOVER_MM + CLEAR_MM
    if p[2] < clear:
        fast, acc = jog_speed(rc)
        robot.move_line([p[0], p[1], clear, *p[3:6]], min(100.0, fast), acc)


def _posx(robot):
    return [round(v, 3) for v in robot.current_posx()]


def _posj(robot):
    return [round(v, 3) for v in robot.current_posj()]


# ------------------------------------------------------------------ markers
A4_MM = (210.0, 297.0)
PRINT_DPI = 300


def _mm(v, dpi=PRINT_DPI):
    return int(round(v / 25.4 * dpi))


def a4_marker_sheet(count=5, size_mm=50.0, dpi=PRINT_DPI) -> np.ndarray:
    """A4(세로) 한 장에 마커 count 개(2열)와 100mm 눈금자. 실제 크기(100%)로 인쇄하면 마커 한 변 = size_mm."""
    W, H = _mm(A4_MM[0], dpi), _mm(A4_MM[1], dpi)
    page = np.full((H, W), 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    side = _mm(size_mm, dpi)
    quiet = _mm(max(8.0, size_mm / 5), dpi)          # 마커 주위 흰 여백 (자를 때 남길 부분)
    box = side + 2 * quiet
    rows = (count + 1) // 2
    top, bottom = _mm(12, dpi), _mm(35, dpi)            # 아래는 눈금자 자리
    pitch_y = (H - top - bottom) / max(rows, 1)
    if box + _mm(10, dpi) > pitch_y or 2 * box > W:
        raise ValueError(f'마커 {size_mm}mm × {count}개는 A4 한 장에 안 들어갑니다')
    font, scale, th = cv2.FONT_HERSHEY_SIMPLEX, dpi / 150, max(2, dpi // 100)
    for i in range(count):
        cx = int(W * (0.25 if i % 2 == 0 else 0.75))
        cy = int(top + pitch_y * (i // 2 + 0.5))
        x0, y0 = cx - side // 2, cy - side // 2
        page[y0:y0 + side, x0:x0 + side] = cv2.aruco.generateImageMarker(d, i, side)
        # 자르는 선 (옅은 회색)
        cv2.rectangle(page, (cx - box // 2, cy - box // 2), (cx + box // 2, cy + box // 2), 190, 1)
        label = f'ID {i}'
        (tw, tht), _ = cv2.getTextSize(label, font, scale, th)
        cv2.putText(page, label, (cx - tw // 2, cy + box // 2 + tht + _mm(3, dpi)), font, scale, 0, th)
    # 100mm 눈금자: 인쇄 후 자로 재서 배율 확인
    ry = H - _mm(22, dpi)
    rx0, rx1 = _mm(20, dpi), _mm(120, dpi)
    cv2.line(page, (rx0, ry), (rx1, ry), 0, th)
    for k in range(11):
        x = _mm(20 + 10 * k, dpi)
        tick = _mm(5 if k % 5 == 0 else 3, dpi)
        cv2.line(page, (x, ry), (x, ry - tick), 0, th)
    small = scale * 0.6
    cv2.putText(page, '100 mm', (rx1 + _mm(4, dpi), ry), font, small, 0, max(1, th - 1))
    note = f'DICT_4X4_50, marker side {size_mm:g} mm. Print at 100% (Actual size, no fit-to-page).'
    cv2.putText(page, note, (rx0, ry + _mm(10, dpi)), font, small, 0, max(1, th - 1))
    return page


def write_a4_pdf(gray: np.ndarray, path) -> Path:
    """회색조 이미지를 A4 한 페이지 PDF 로 (페이지 전체에 정확히 맞춤). 외부 의존성 없음."""
    import zlib
    h, w = gray.shape
    pw, ph = (round(v / 25.4 * 72, 2) for v in A4_MM)
    data = zlib.compress(np.ascontiguousarray(gray, np.uint8).tobytes())
    content = f'q {pw} 0 0 {ph} 0 0 cm /Im0 Do Q'.encode()
    objs = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {pw} {ph}] '
        f'/Resources << /XObject << /Im0 4 0 R >> >> /Contents 5 0 R >>'.encode(),
        f'<< /Type /XObject /Subtype /Image /Width {w} /Height {h} /ColorSpace /DeviceGray '
        f'/BitsPerComponent 8 /Filter /FlateDecode /Length {len(data)} >>\nstream\n'.encode() + data + b'\nendstream',
        f'<< /Length {len(content)} >>\nstream\n'.encode() + content + b'\nendstream',
    ]
    out, offsets = bytearray(b'%PDF-1.4\n'), []
    for i, o in enumerate(objs, 1):
        offsets.append(len(out))
        out += f'{i} 0 obj\n'.encode() + o + b'\nendobj\n'
    xref = len(out)
    out += f'xref\n0 {len(objs) + 1}\n0000000000 65535 f \n'.encode()
    out += b''.join(f'{off:010d} 00000 n \n'.encode() for off in offsets)
    out += f'trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n'.encode()
    Path(path).write_bytes(bytes(out))
    return Path(path)


def cmd_markers(a):
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    for i in range(a.count):
        m = cv2.aruco.generateImageMarker(d, i, 400)
        sheet = np.full((560, 480), 255, np.uint8)
        sheet[40:440, 40:440] = m
        cv2.putText(sheet, f'ID {i}  (DICT_4X4_50)', (40, 500), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
        cv2.imwrite(str(out / f'marker_{i}.png'), sheet)
    page = a4_marker_sheet(a.count, a.size_mm)
    png = out / 'markers_A4.png'
    cv2.imwrite(str(png), page)
    pdf = write_a4_pdf(page, out / 'markers_A4.pdf')
    print(f'마커 {a.count}개 → {pdf}  (한 변 {a.size_mm:g}mm, 미리보기 {png.name})')
    print('  PDF 를 실제 크기(100%)로 인쇄 → 아래 100mm 눈금을 자로 확인 → 잘라서 평평하게 고정.')
    print('  0~3: 종이 네 모서리 바깥 대각선 방향, 4: 한 변 가운데 바깥. 종이에 가려지면 안 됨')
    return 0


# ------------------------------------------------------------------ view
def draw_markers(view, expected=5):
    """화면(view)에 인식된 마커 중심·번호와 작업 영역 선을 겹쳐 그린다(제자리 수정). 반환: 인식된 id 목록.
    초록 = expected 개가 모두 보임, 빨강 = 빠진 마커 있음. 선 안쪽이 그릴 수 있는 영역."""
    seen = detect_markers(view)
    ok = len(seen) >= expected
    color = (0, 200, 0) if ok else (0, 0, 255)
    if len(seen) >= 3:
        hull = cv2.convexHull(np.float32(list(seen.values()))).astype(np.int32)
        cv2.polylines(view, [hull], True, color, 1)
    for mid, (x, y) in seen.items():
        cv2.circle(view, (int(x), int(y)), 6, color, -1)
        cv2.putText(view, str(mid), (int(x) + 10, int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, .8, color, 2)
    missing = [i for i in range(expected) if i not in seen]
    text = f'markers {len(seen)}/{expected}' + ('' if ok else f' missing {missing}')
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .7, 2)
    cv2.putText(view, text, (view.shape[1] - tw - 12, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)
    return sorted(seen)


def annotate_markers(frame, expected=5):
    """마커 표시를 그린 화면과 인식된 id 목록."""
    view = frame.copy()
    ids = draw_markers(view, expected)
    cv2.putText(view, 'Q: quit  S: save snapshot', (12, view.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 200, 0), 2)
    return view, ids


def cmd_view(a, cfg):
    cap = _open_camera(cfg, a.camera_index)
    print('초록 = 마커가 모두 보임, 빨강 = 빠진 마커 있음. 선 안쪽이 그릴 수 있는 영역입니다. Q 종료, S 저장')
    title = 'Markers (Q quit / S save)'
    last = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('카메라 프레임 실패')
            view, ids = annotate_markers(frame, a.count)
            if ids != last:
                print(f'보이는 마커: {ids}')
                last = ids
            cv2.imshow(title, view)
            key = cv2.waitKey(30) & 0xff
            if key in (ord('q'), 27):
                return 0
            if key == ord('s'):
                out = ROOT / 'out' / 'markers_view.png'
                out.parent.mkdir(exist_ok=True)
                cv2.imwrite(str(out), view)
                print(f'저장: {out}')
    finally:
        cap.release()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ camera-robot
def _average_markers(cap, frames=8):
    acc = {}
    for _ in range(frames):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError('카메라 프레임 실패')
        for k, v in detect_markers(frame).items():
            acc.setdefault(k, []).append(v)
    return {k: tuple(np.mean(v, axis=0)) for k, v in acc.items() if len(v) >= frames // 2}, frame


def solve_homography(pixels, base_xy):
    """마커 픽셀 → base XY 호모그래피와 점별 오차(mm). 4점이면 오차는 항상 0(검증 불가)."""
    src = np.asarray(pixels, np.float64)
    dst = np.asarray(base_xy, np.float64)
    H, _ = cv2.findHomography(src, dst, 0)
    if H is None:
        raise ValueError('호모그래피 계산 실패 (마커가 일직선?)')
    proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    return H, np.linalg.norm(proj - dst, axis=1)


def cmd_camera_robot(a, cfg, input_fn=input):
    cap = _open_camera(cfg, a.camera_index)
    seen, frame = _average_markers(cap)
    ids = sorted(seen)
    print(f'보이는 마커: {ids}')
    if len(ids) < 4:
        raise RuntimeError('마커가 4개 이상 보여야 합니다 (robot-calibrate markers 로 인쇄)')
    if len(ids) == 4:
        print('[주의] 마커 4개로는 보정 오차를 검증할 수 없습니다. 한 변 가운데 바깥에 5번째 마커를 권장합니다.')
    robot = _robot(cfg)                  # 펜 끝 TCP(pen_tip)는 DART 에서 활성화되어 있어야 한다
    try:
        panel = _jog_panel(a, cfg, robot, input_fn)
        hints = load_hints() if panel else {}
        if panel and hints:
            print(f'마커 위치 힌트 {sorted(hints)} → 각 마커 위 {HOVER_MM:g}mm 까지 자동으로 갑니다. 거기서 1mm 단위로 맞추세요.')
        pts, touched = [], {}
        for mid in ids:
            prompt = f'펜 끝을 마커 {mid} 중심에 살짝 닿게 (누르지 말고 닿기만)'
            if panel is None:
                input_fn(prompt + ' 후 Enter > ')
            else:
                guess = predict_marker_xy(mid, seen, touched, hints)
                if guess is not None:
                    try:
                        _approach(robot, guess, cfg['robot'])
                    except (RuntimeError, ValueError) as e:
                        if robot.stopped():
                            raise
                        print(f'  [자동 접근 실패] {e} → 직접 조그로 옮기세요')
                    panel.step = 1.0
                panel.position(prompt)
            pts.append(_posx(robot))
            touched[mid] = pts[-1][:2]
            print(f'  마커 {mid}: 픽셀 {tuple(round(v, 1) for v in seen[mid])}  로봇 {pts[-1][:3]}')
            if panel is not None:
                _lift(robot, cfg['robot'])
        if panel is not None and load_poses().get('camera_clear'):
            print('카메라 비킴 자세(camera_clear)로 이동 → 마커 위치 재확인')
            _to_camera_clear(robot, cfg['robot'])
        else:
            input_fn('로봇을 카메라 시야 밖으로 옮긴 뒤 Enter (마커 위치 재확인용; 먼저 위로 들기) > ')
        seen2, frame = _average_markers(cap)
    finally:
        cap.release()
        robot.close()
    missing = [m for m in ids if m not in seen2]
    if missing:
        raise RuntimeError(f'재확인 때 마커 {missing} 가 안 보임')
    pixels = [seen2[m] for m in ids]
    H, err = solve_homography(pixels, [p[:2] for p in pts])
    zs = np.array([p[2] for p in pts])
    print(f'점별 오차(mm): {np.round(err, 2).tolist()}  최대 {err.max():.2f}')
    print(f'접촉 Z: 평균 {zs.mean():.2f}  편차 {zs.max() - zs.min():.2f} mm')
    if zs.max() - zs.min() > WARN_Z_SPREAD_MM:
        print(f'[경고] 접촉 Z 편차가 {WARN_Z_SPREAD_MM}mm 를 넘습니다. 테이블 수평/펜 TCP 를 확인하세요.')
    if err.max() > MAX_ERR_MM:
        print(f'[거부] 최대 오차 {err.max():.2f}mm > {MAX_ERR_MM}mm. 저장하지 않습니다.', file=sys.stderr)
        return 2
    if err.max() > WARN_ERR_MM:
        print(f'[경고] 최대 오차가 {WARN_ERR_MM}mm 를 넘습니다.')
    cal = {
        'measured_and_verified': False,
        'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'H_camera_to_base': H.tolist(),
        'markers': [{'id': m, 'pixel': [round(v, 2) for v in px], 'base_xy': p[:2], 'base_z': p[2]}
                    for m, px, p in zip(ids, pixels, pts)],
        'pen_contact_z_base_mm': round(float(zs.mean()), 3),
        # 접촉 높이 면 z = a·x + b·y + c (테이블 기울기·팔 처짐). 그릴 때 위치마다 이 높이를 쓴다
        'pen_contact_plane': [round(v, 6) for v in fit_plane([p[:3] for p in pts])],
        'pen_up_z_base_mm': round(float(zs.max()) + PEN_UP_CLEARANCE_MM, 3),
        'tool_orientation_deg': pts[0][3:6],
        'max_error_mm': round(float(err.max()), 3),
    }
    out = Path(a.out or CAMERA_CALIB_PATH)
    out.write_text(json.dumps(cal, indent=2, ensure_ascii=False), encoding='utf-8')
    cv2.imwrite(str(out.with_suffix('.png')), frame)
    print(f'저장: {out}  (확인 후 measured_and_verified 를 true 로 바꾸세요)')
    return 0


# ------------------------------------------------------------------ poses
POSE_HELP = {
    'home': '홈(대기) 자세 — default.yaml 의 robot.home_joints_deg 에도 넣어야 게임이 쓴다',
    'celebration': '이겼을 때 세리머니 자세',
    'camera_clear': '카메라 시야를 가리지 않는 대기 자세',
}


def cmd_poses(a, cfg, input_fn=input):
    names = a.only or list(JOINT_KEYS)
    data = load_poses()
    robot = _robot(cfg)
    try:
        wait = _positioner(a, cfg, robot, input_fn)
        for n in names:
            wait(f'[{n}] {POSE_HELP[n]} 으로 이동')
            data[n] = _posj(robot)
            print(f'  {n} = {data[n]}')
    finally:
        robot.close()
    data['measured_and_verified'] = False
    data['updated_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
    POSES_PATH.parent.mkdir(parents=True, exist_ok=True)
    POSES_PATH.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')
    print(f'저장: {POSES_PATH}')
    return 0


# ------------------------------------------------------------------ gripper
def cmd_gripper(a, cfg):
    from .gripper import FlangeGripper
    gcfg = dict(cfg['gripper'])
    if gcfg.get('backend', 'flange') != 'flange':
        raise ValueError('이 명령은 플랜지 그리퍼(gripper.backend: flange)용입니다. USB 어댑터는 gripper.RobotisGripper 를 쓰세요')
    # 시험용: 열림/닫힘 값이 아직 없으면 0 으로 두고, --goal 로 임의 위치를 보낸다
    for k in ('open_position', 'closed_position'):
        if gcfg.get(k) is None:
            gcfg[k] = 0
    robot = _robot(cfg)
    try:
        grip = FlangeGripper(gcfg)
        if a.torque:
            grip._run([grip.proto.torque(a.torque == 'on')])
            print(f'torque {a.torque}')
        if a.goal is not None:
            gcfg['open_position'] = a.goal
            grip.cfg = gcfg
            grip.open()
            print(f'goal {a.goal} 전송')
        if a.read:
            print(f'현재 위치: {grip.position()}')
    finally:
        robot.close()
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description='보정·교시 도구 (--jog 를 줄 때만 로봇을 움직임)')
    p.add_argument('--config')
    sub = p.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('markers')
    m.add_argument('--count', type=int, default=5)
    m.add_argument('--size-mm', type=float, default=50.0, help='A4 시트의 마커 한 변 (mm)')
    m.add_argument('--out', default=str(ROOT / 'out' / 'markers'))
    v = sub.add_parser('view')
    v.add_argument('--camera-index', type=int)
    v.add_argument('--count', type=int, default=5, help='보여야 하는 마커 수')
    for name in ('camera-robot', 'poses', 'gripper'):
        sp = sub.add_parser(name)
        if name in ('camera-robot', 'poses'):
            sp.add_argument('--jog', action='store_true', help='키보드 스텝 조그로 로봇을 옮김 (직접 교시 대신)')
            sp.add_argument('--execute-motion', action='store_true', help='실기 조그 게이트')
        if name == 'camera-robot':
            sp.add_argument('--camera-index', type=int)
            sp.add_argument('--out')
        if name == 'poses':
            sp.add_argument('--only', nargs='+', choices=list(JOINT_KEYS))
        if name == 'gripper':
            sp.add_argument('--read', action='store_true')
            sp.add_argument('--goal', type=int)
            sp.add_argument('--torque', choices=['on', 'off'])
    a = p.parse_args(argv)
    if a.cmd == 'markers':
        try:
            return cmd_markers(a)
        except ValueError as e:
            print(f'[ERROR] {e}', file=sys.stderr)
            return 2
    cfg = _cfg(a.config)
    try:
        return {'view': cmd_view, 'camera-robot': cmd_camera_robot, 'poses': cmd_poses,
                'gripper': cmd_gripper}[a.cmd](a, cfg)
    except (RuntimeError, ValueError, PermissionError) as e:
        print(f'[ERROR] {e}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('\n중단')
        return 130


if __name__ == '__main__':
    sys.exit(main())
