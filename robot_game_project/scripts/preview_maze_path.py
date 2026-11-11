"""로봇을 움직이지 않고 미로 경로를 확인한다: 카메라 한 장 → paper_maze 풀이 → maze_csv(real) 행 →
로봇 경로를 카메라 사진 위에 다시 그린다(out/maze_path_preview.png). 실기 그리기 전에 경로가 미로 통로를 지나는지 본다.
  python scripts/preview_maze_path.py
"""
import json, sys, tempfile
from pathlib import Path
import cv2, numpy as np, yaml
_root = Path(__file__).resolve().parents[1]
_args = sys.argv[1:] + [None] * 3
proj = Path(_args[0] or _root)
calib = Path(_args[1] or proj / 'configs/my_verified_calibration_real.json')
out_img = Path(_args[2] or proj / 'out/maze_path_preview.png')
sys.path.insert(0, str(proj))
from robot_games import paper_maze as pm, maze_csv
from robot_games.camera_select import choose_camera
cfg = yaml.safe_load((proj / 'configs/default.yaml').read_text())
cap = cv2.VideoCapture(choose_camera()); cap.set(3, cfg['camera']['width']); cap.set(4, cfg['camera']['height'])
for _ in range(15): ok, frame = cap.read()
cap.release()
corners = pm.marker_board_corners(frame)
board, M = pm.perspective_board(frame, corners, width=600, height=600)
full = board.shape
crop, off = pm.crop_to_maze(board, return_offset=True)
found = pm.find_openings(crop); assert found, "openings not found"
s, g, inside, s_ext, g_ext = found
path, _, used = pm.solve_centered(crop, s, g, inside=inside); print("clearance", used); path = [tuple(p) for p in s_ext] + list(path) + [tuple(p) for p in g_ext]
plan = pm.trajectory(path, full, off)
print(f'maze solved: {len(path)} waypoints, start {s} goal {g} (crop px), crop offset {off}')
c = json.loads(calib.read_text()); c['measured_and_verified'] = True   # 미리보기 변환용 임시 사본 (실행 안 함)
tmp = Path(tempfile.mkdtemp())
(tmp / 'cal.json').write_text(json.dumps(c)); (tmp / 'plan.json').write_text(json.dumps(plan))
meta = maze_csv.convert(tmp / 'plan.json', 'real', tmp / 'cal.json', tmp / 'robot.csv')
import csv
rows = list(csv.DictReader(open(meta['csv'])))
xs = [float(r['x']) for r in rows]; ys = [float(r['y']) for r in rows]
zd = [float(r['z']) for r in rows if r['pen'] == 'down']; zu = {float(r['z']) for r in rows if r['pen'] == 'up'}
print(f"rows {len(rows)}  X {min(xs):.1f}~{max(xs):.1f}  Y {min(ys):.1f}~{max(ys):.1f}  pen-down Z {min(zd):.2f}~{max(zd):.2f}  pen-up Z {sorted(zu)}")
# 로봇 XY → 보드 (u,v) → 보드 px → 카메라 px (검산: 팀원 선형 모델의 역변환)
o = np.array(c['origin_base_mm'][:2]); X = np.array(c['x_span_base_mm'][:2]); Y = np.array(c['y_span_base_mm'][:2])
A = np.c_[X, Y]
uv = np.linalg.solve(A, (np.c_[xs, ys] - o).T).T
bpx = uv * [full[1] - 1, full[0] - 1]
cam = cv2.perspectiveTransform(bpx.reshape(-1, 1, 2).astype(np.float64), np.linalg.inv(M)).reshape(-1, 2)
v = frame.copy()
cv2.polylines(v, [np.int32(corners)], True, (0, 0, 255), 1)
cv2.polylines(v, [np.int32(cam)], False, (0, 0, 255), 2)
cv2.circle(v, tuple(np.int32(cam[0])), 6, (0, 200, 0), -1); cv2.circle(v, tuple(np.int32(cam[-1])), 6, (255, 0, 0), -1)
cv2.imwrite(str(out_img), v)
x0, y0, x1, y1 = cv2.boundingRect(np.int32(cam)); print('path in camera px bbox', (x0, y0, x0 + x1, y0 + y1))
