"""펜 경로 → 로봇 행(rows) 변환과 실행 전 검증. 미로와 틱택토가 함께 쓴다.

좌표 흐름 (실기/카메라)
  보정된 보드 이미지 픽셀 ──image_to_camera(3×3)──▶ 카메라 픽셀 ──H_cam→base(3×3)──▶ 로봇 base XY(mm)
  Z 는 펜 상태(up/down)에 따라 pen_up_z / pen_contact_z, 자세는 tool_orientation.
image_to_camera 는 원근 보정 행렬 M(카메라→보드)의 역행렬(크롭이 있으면 평행이동 포함).
H_cam→base 는 `robot-calibrate camera-robot` 이 ArUco 마커 + 펜 끝 터치로 만든다(configs/camera_robot_calibration.json).

rows 형식은 maze_csv CSV 와 같다: step, action, x, y, z, pen, rx, ry, rz
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAMERA_CALIB_PATH = ROOT / 'configs' / 'field' / 'camera_robot_calibration.json'
ARUCO_DICT = cv2.aruco.DICT_4X4_50

MAX_STEP_DOWN_MM = 60.0     # 펜 내린 상태 인접 점 최대 간격
MAX_CONTACT_SPREAD_MM = 10.0  # 한 그림 안에서 펜 내림 Z 가 달라도 되는 최대 폭 (테이블 기울기 보정분)
MIN_PEN_LIFT_MM = 5.0         # 펜 올림 Z 는 펜 내림 Z 들보다 최소 이만큼 위
PEN_PRESS_MM = 0.5            # 닿는 높이보다 이만큼 눌러 그린다 (확실히 잉크가 묻도록)
MAX_REACH_XY_MM = 850.0     # E0509 도달 900mm 보다 여유 (베이스 기준 수평거리)
CAMERA_MOVED_PX = 3.0       # 보정 당시 대비 마커 위치 허용 오차


class PlanRefused(RuntimeError):
    pass


# ---------------------------------------------------------------- 보정 파일
def load_camera_calibration(path=None, require_verified=True) -> dict:
    p = Path(path or CAMERA_CALIB_PATH)
    if not p.exists():
        raise PlanRefused(f'{p.name} 없음: robot-calibrate camera-robot 으로 먼저 보정하세요')
    c = json.loads(p.read_text(encoding='utf-8'))
    H = np.asarray(c.get('H_camera_to_base') or [], float)
    if H.shape != (3, 3):
        raise PlanRefused(f'{p.name}: H_camera_to_base 가 3×3 이 아님')
    for k in ('pen_contact_z_base_mm', 'pen_up_z_base_mm', 'tool_orientation_deg', 'markers'):
        if c.get(k) is None:
            raise PlanRefused(f'{p.name}: {k} 누락')
    if float(c['pen_up_z_base_mm']) <= float(c['pen_contact_z_base_mm']):
        raise PlanRefused('pen_up_z 는 pen_contact_z 보다 커야 함')
    if len(c['tool_orientation_deg']) != 3:
        raise PlanRefused('tool_orientation_deg 는 [rx,ry,rz] 3개')
    if require_verified and c.get('measured_and_verified') is not True:
        raise PlanRefused(f'{p.name}: measured_and_verified 가 true 가 아님 (현장 확인 후 직접 true 로)')
    c['_H'] = H
    c['_path'] = str(p)
    c['_plane'] = contact_plane(c)
    return c


def fit_plane(points):
    """[(x, y, z), ...] → z = a·x + b·y + c 의 (a, b, c). 점이 3개 미만이면 평균 높이의 수평면."""
    pts = np.asarray(points, float)
    if len(pts) < 3:
        return (0.0, 0.0, float(pts[:, 2].mean()))
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    a, b, c = np.linalg.lstsq(A, pts[:, 2], rcond=None)[0]
    return (float(a), float(b), float(c))


def contact_plane(cal):
    """펜이 종이에 닿는 높이의 면. 보정 때 각 마커에서 잰 접촉 Z 로 기울어진 테이블/팔 처짐을 반영한다
    (2026-11-10 실측: 로봇 앞 45cm 44.8mm, 66cm 48.8mm). 마커별 Z 가 없으면 평균 높이의 수평면."""
    if cal.get('pen_contact_plane'):
        return tuple(float(v) for v in cal['pen_contact_plane'])
    pts = [(*m['base_xy'], m['base_z']) for m in cal.get('markers', []) if m.get('base_z') is not None]
    if len(pts) >= 3:
        return fit_plane(pts)
    return (0.0, 0.0, float(cal['pen_contact_z_base_mm']))


def plane_z(plane, x, y):
    a, b, c = plane
    return a * x + b * y + c


def workspace_polygon(cal) -> np.ndarray:
    """보정 마커들의 base XY 볼록 껍질 = 펜이 내려가도 되는 영역."""
    pts = np.asarray([m['base_xy'] for m in cal['markers']], np.float32)
    return cv2.convexHull(pts).reshape(-1, 2)


# ---------------------------------------------------------------- 마커
def detect_markers(frame) -> dict:
    """{id: (cx, cy)} 카메라 픽셀."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return {}
    return {int(i): tuple(map(float, c.reshape(4, 2).mean(axis=0))) for i, c in zip(ids.ravel(), corners)}


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


def check_camera_fixed(frame, cal, tol_px=CAMERA_MOVED_PX):
    """보정 당시 마커 픽셀과 지금 프레임의 마커 픽셀을 비교. 카메라가 움직였으면 PlanRefused."""
    now = detect_markers(frame)
    worst = 0.0
    for m in cal['markers']:
        if m['id'] not in now:
            raise PlanRefused(f'마커 {m["id"]} 가 보이지 않음 (가려짐/카메라 이동?)')
        worst = max(worst, math.dist(now[m['id']], m['pixel']))
    if worst > tol_px:
        raise PlanRefused(f'카메라가 움직였습니다: 마커 위치 차이 {worst:.1f}px > {tol_px}px → 재보정 필요')
    return worst


# ---------------------------------------------------------------- 변환
def translation(dx, dy):
    return np.array([[1, 0, dx], [0, 1, dy], [0, 0, 1]], float)


def _apply(Hm, pts):
    pts = np.asarray(pts, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, np.asarray(Hm, np.float64)).reshape(-1, 2)


class BoardToBase:
    """보드 이미지 픽셀 → base XY. image_to_camera 와 H_camera_to_base 를 합성."""

    def __init__(self, image_to_camera, H_camera_to_base):
        self.M = np.asarray(H_camera_to_base, float) @ np.asarray(image_to_camera, float)

    def __call__(self, pts):
        return _apply(self.M, pts)


class LinearBoard:
    """(시뮬레이션용) 정규화 0~1 좌표를 origin + u·x_span + v·y_span 으로. 기존 my_verified_calibration.json 형식."""

    def __init__(self, cal, image_size):
        self.o = np.asarray(cal['origin_base_mm'], float)
        self.x = np.asarray(cal['x_span_base_mm'], float)
        self.y = np.asarray(cal['y_span_base_mm'], float)
        self.w, self.h = image_size

    def __call__(self, pts):
        out = []
        for px, py in pts:
            u, v = px / max(1, self.w - 1), py / max(1, self.h - 1)
            out.append((self.o + u * self.x + v * self.y)[:2])
        return np.asarray(out)


def strokes_to_sequence(strokes) -> list:
    """획 목록(각 획은 이미지 픽셀 점 목록) → 펜 시퀀스. 획마다 up → down → trace → up."""
    seq = []
    for k, stroke in enumerate(strokes):
        stroke = [tuple(map(float, p)) for p in stroke]
        if len(stroke) < 2:
            continue
        seq.append({'action': f'move_above_stroke{k}', 'px': stroke[0], 'pen': 'up'})
        seq.append({'action': f'lower_pen_stroke{k}', 'px': stroke[0], 'pen': 'down'})
        seq += [{'action': 'trace', 'px': p, 'pen': 'down'} for p in stroke[1:]]
        seq.append({'action': f'raise_pen_stroke{k}', 'px': stroke[-1], 'pen': 'up'})
    return seq


def sequence_to_rows(seq, to_base, z_up, z_down, rpy) -> list:
    """z_down: 숫자(일정 높이) 또는 (x, y) → z 함수(기울어진 면)."""
    xy = to_base([s['px'] for s in seq])
    rows = []
    for i, (s, (x, y)) in enumerate(zip(seq, xy)):
        if s['pen'] == 'up':
            z = float(z_up)
        else:
            z = float(z_down(float(x), float(y))) if callable(z_down) else float(z_down)
        rx, ry, rz = (float(v) for v in rpy)
        rows.append({'step': i, 'action': s['action'], 'x': round(float(x), 3), 'y': round(float(y), 3),
                     'z': round(z, 3), 'pen': s['pen'], 'rx': rx, 'ry': ry, 'rz': rz})
    return rows


def contact_z_fn(cal, press_mm=PEN_PRESS_MM):
    """그릴 때 펜 내림 높이 함수: 접촉면 - 누름 깊이."""
    plane = cal.get('_plane') or contact_plane(cal)
    return lambda x, y: plane_z(plane, x, y) - press_mm


def rows_from_camera_calibration(seq, image_to_camera, cal) -> list:
    return sequence_to_rows(seq, BoardToBase(image_to_camera, cal['_H']),
                            cal['pen_up_z_base_mm'], contact_z_fn(cal), cal['tool_orientation_deg'])


# ---------------------------------------------------------------- 검증
def validate_rows(rows, max_points=1500, workspace=None):
    """실기/에뮬레이터 실행 전 기하 검증. 하나라도 실패하면 PlanRefused."""
    if not rows:
        raise PlanRefused('실행할 행이 없음')
    if len(rows) > max_points + 4:
        raise PlanRefused(f'점 개수 {len(rows)} > 한계 {max_points}')
    if any(r[k] is None for r in rows for k in ('x', 'y', 'z', 'rx', 'ry', 'rz')):
        raise PlanRefused('x,y,z,rx,ry,rz 중 빈 값 있음 (tool_orientation 누락?)')
    downs = [r['z'] for r in rows if r['pen'] == 'down']
    ups = {r['z'] for r in rows if r['pen'] == 'up'}
    if len(ups) != 1:
        raise PlanRefused('펜 올림 Z 가 하나의 값이 아님')
    if downs and max(downs) - min(downs) > MAX_CONTACT_SPREAD_MM:
        raise PlanRefused(f'펜 내림 Z 차이가 {max(downs) - min(downs):.1f}mm > {MAX_CONTACT_SPREAD_MM:g}mm (테이블 기울기 보정이 이상함)')
    if downs and min(ups) < max(downs) + MIN_PEN_LIFT_MM:
        raise PlanRefused('펜 올림 Z 가 내림 Z 보다 충분히 높지 않음')
    for r in rows:
        if math.hypot(r['x'], r['y']) > MAX_REACH_XY_MM:
            raise PlanRefused(f'step {r["step"]}: 도달 범위 밖 (r={math.hypot(r["x"], r["y"]):.0f}mm)')
    for a, b in zip(rows, rows[1:]):
        if a['pen'] == b['pen'] == 'down' and math.dist((a['x'], a['y']), (b['x'], b['y'])) > MAX_STEP_DOWN_MM:
            raise PlanRefused(f'step {a["step"]}→{b["step"]}: 펜을 내린 채 {MAX_STEP_DOWN_MM:.0f}mm 이상 점프')
    if rows[0]['pen'] != 'up' or rows[-1]['pen'] != 'up':
        raise PlanRefused('펜 시퀀스가 up 으로 시작/끝나지 않음')
    if workspace is not None:
        poly = np.asarray(workspace, np.float32).reshape(-1, 1, 2)
        for r in rows:
            if cv2.pointPolygonTest(poly, (float(r['x']), float(r['y'])), True) < -1.0:
                raise PlanRefused(f'step {r["step"]}: 마커 작업 영역 밖 ({r["x"]:.0f},{r["y"]:.0f})')
