"""보드 마커 4장(TL,TR,BR,BL)의 실측 로봇 좌표로 미로 실측 보정(configs/my_verified_calibration_real.json)을 만든다.

마커를 옮겼거나 robot_games.calibrate camera-robot 으로 다시 보정했을 때 실행한다. 로봇은 움직이지 않는다.
결과는 measured_and_verified=false 로 저장되므로, 경로 미리보기로 확인한 뒤 직접 true 로 바꾼다.
  python scripts/make_maze_calibration.py   (기본 경로 사용)
"""
import json, sys
from pathlib import Path
import cv2, numpy as np, yaml
_root = Path(__file__).resolve().parents[1]
_args = sys.argv[1:] + [None] * 4
proj = Path(_args[0] or _root)
field = Path(_args[1] or proj / 'configs/field/camera_robot_calibration.json')
out_json = Path(_args[2] or proj / 'configs/my_verified_calibration_real.json')
out_img = Path(_args[3] or proj / 'out/maze_board_markers.png')
out_img.parent.mkdir(exist_ok=True)
sys.path.insert(0, str(proj))
from robot_games.calibrate import fit_plane
from robot_games.camera_select import choose_camera
from robot_games.paper_maze import marker_board_corners, perspective_board, crop_to_maze
cfg = yaml.safe_load((proj / 'configs/default.yaml').read_text())
ids = cfg['vision']['board_marker_ids']
cal = json.loads(field.read_text())
m = {x['id']: x for x in cal['markers']}
TL, TR, BR, BL = (np.array(m[i]['base_xy'], float) for i in ids)
zs = [x['base_z'] for x in cal['markers']]
a, b, c = fit_plane([(*x['base_xy'], x['base_z']) for x in cal['markers']])
center = (TL + TR + BR + BL) / 4
mismatch = np.linalg.norm((TR - TL) - (BR - BL))
print('board markers TL,TR,BR,BL', ids, [np.round(v, 1).tolist() for v in (TL, TR, BR, BL)])
print(f'board {np.linalg.norm(TR-TL):.1f} x {np.linalg.norm(BL-TL):.1f} mm, parallelogram mismatch {mismatch:.2f} mm (linear model error)')
res = {
    "_note": "robot_game_project 실측 보정 (2026-11-11). 보드 = ArUco 마커 1,3,2,0 중심(TL,TR,BR,BL), 로봇 좌표는 2026-11-10 펜 끝 터치 실측. 마커를 움직이면 다시 재야 함",
    "measured_and_verified": False,
    "origin_base_mm": [round(float(TL[0]), 3), round(float(TL[1]), 3), round(float(a*TL[0]+b*TL[1]+c), 3)],
    "x_span_base_mm": [round(float(TR[0]-TL[0]), 3), round(float(TR[1]-TL[1]), 3), 0.0],
    "y_span_base_mm": [round(float(BL[0]-TL[0]), 3), round(float(BL[1]-TL[1]), 3), 0.0],
    "pen_contact_z_base_mm": round(float(a*center[0]+b*center[1]+c), 3),
    "pen_up_z_base_mm": round(float(max(zs)) + 30.0, 3),
    "tool_orientation_deg": cal['tool_orientation_deg'],
    "pen_contact_plane": [round(float(a), 6), round(float(b), 6), round(float(c), 4)],
    "pen_press_mm": 1.0,
    "board_marker_ids": ids,
}
out_json.write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: v for k, v in res.items() if k != '_note'}, ensure_ascii=False))
cap = cv2.VideoCapture(choose_camera()); cap.set(3, cfg['camera']['width']); cap.set(4, cfg['camera']['height'])
for _ in range(15): ok, frame = cap.read()
cap.release()
corners = marker_board_corners(frame)
print('marker corners px', None if corners is None else np.round(corners, 1).tolist())
v = frame.copy()
cv2.polylines(v, [np.int32(corners)], True, (0, 0, 255), 2)
board, _ = perspective_board(frame, corners, width=600, height=600)
crop, off = crop_to_maze(board, return_offset=True)
print('rectified board 600x600, maze crop offset', off, 'crop size', crop.shape[1], 'x', crop.shape[0])
cv2.imwrite(str(out_img), v); cv2.imwrite(str(out_img.with_name('board_rectified.png')), board)
