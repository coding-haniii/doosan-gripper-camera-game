"""펜 거치대(컵)에서 펜 부품을 집고 반납한다. 거치대 위치는 옆에 붙인 ArUco 마커로 카메라가 찾는다.

  교시(한 번):  python -m robot_games.calibrate holder --jog --execute-motion
               → 마커 위치·각도를 찍고, 조그로 '접근'·'잡기' 자세를 맞추면 마커 기준 상대값으로
                 configs/field/robot_poses.json 의 pen_holder 에 저장(verified: false). 확인 후 사람이 true 로 바꾼다.
  실행:        python -m robot_games.pen_holder pick|return --execute-motion [--rehearsal]
               --rehearsal: 모든 자세를 50mm 위로 올리고 그리퍼는 열기만 한다(닫지 않음).

좌표: TCP 는 바꾸지 않는다(pen_tip). '잡기' 자세는 펜 부품이 거치대에 꽂혀 있을 때 펜 끝이 있는 곳이다.
자세는 [x, y, z, A, B, C](BASE, ZYZ). 거치대가 돌아가 있으면 XY 와 A(베이스 Z축 회전)를 같이 돌린다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
FIELD = ROOT / 'configs' / 'field'
CAMERA_CALIB_PATH = FIELD / 'camera_robot_calibration.json'
POSES_PATH = FIELD / 'robot_poses.json'
POSE_NAMES = ('approach', 'grasp')
MAX_YAW_CHANGE_DEG = 20.0      # 교시 때보다 거치대가 이만큼 넘게 돌아가 있으면 거부
MAX_SHIFT_MM = 200.0           # 교시 때보다 이만큼 넘게 옮겨져 있으면 거부(오검출 방지)
SLOW_MM_S = 30.0               # 거치대 쪽으로 오르내릴 때 속도
REHEARSAL_LIFT_MM = 50.0
STATE_PATH = ROOT / 'out' / 'pen_holder_state.json'   # 집을 때 잰 거치대 위치 (반납 때 씀)
VIEW_OFFSET_Y_MM = 150.0       # 반납 전 마커를 다시 잴 때 거치대 위에서 옆(+Y)으로 비키는 거리
MIN_APPROACH_DZ_MM = 20.0
MIN_REHEARSAL_LIFT_MM = 10.0   # --lift 하한: 손가락 끝이 부품 윗면보다 위에 남도록


class HolderError(RuntimeError):
    pass


# ---------------------------------------------------------------- 마커 → 거치대 위치
def _to_base(H, px):
    q = np.asarray(H, float) @ [float(px[0]), float(px[1]), 1.0]
    return q[:2] / q[2]


def marker_pose(corners_px, H):
    """마커 모서리(카메라 px, TL TR BR BL) → BASE (x, y, yaw°). yaw 는 마커의 TL→TR 방향(테이블 평면 호모그래피).
    변 하나만 쓰면 모서리 1px 오차로 몇 도가 흔들리므로, 가로 두 변(TL→TR, BL→BR)의 방향 벡터를 합쳐 쓴다.
    (세로 변을 90° 돌려 더하면 카메라-로봇 좌우 반전에 따라 회전 방향이 바뀌어 틀린다)"""
    tl, tr, br, bl = [_to_base(H, p) for p in corners_px]
    cx, cy = np.mean([tl, tr, br, bl], axis=0)
    v = (tr - tl) + (br - bl)
    return float(cx), float(cy), math.degrees(math.atan2(v[1], v[0]))


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def to_marker_frame(pose, marker):
    """BASE 자세 → 마커 기준 상대값 [dx, dy, z, dA, B, C] (dx, dy 는 마커 축 기준, dA = A - yaw)."""
    mx, my, yaw = marker
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    ex, ey = pose[0] - mx, pose[1] - my
    return [c * ex + s * ey, -s * ex + c * ey, pose[2], _wrap(pose[3] - yaw), pose[4], pose[5]]


def from_marker_frame(rel, marker):
    """마커 기준 상대값 → BASE 자세 (to_marker_frame 의 역)."""
    mx, my, yaw = marker
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return [mx + c * rel[0] - s * rel[1], my + s * rel[0] + c * rel[1], rel[2], _wrap(rel[3] + yaw), rel[4], rel[5]]


def find_holder(frame, marker_id, H):
    from .paper_maze import detect_marker_corners
    corners = detect_marker_corners(frame).get(int(marker_id))
    if corners is None:
        raise HolderError(f'거치대 마커 {marker_id} 가 카메라에 보이지 않음 (가림·반사·화면 밖 확인)')
    return marker_pose(corners, H)


def capture_holder(cfg, H=None):
    """카메라 한 장으로 거치대 마커 위치 (x, y, yaw). 로봇이 움직이기 전에 부르고 카메라는 바로 닫는다."""
    import cv2
    from .camera_select import choose_camera
    cam = cfg.get('camera') or {}
    cap = cv2.VideoCapture(choose_camera(int(cam.get('index', 0))), cv2.CAP_V4L2)
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam.get('width', 1280)))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam.get('height', 720)))
        frames = []
        for i in range(20):
            ok, f = cap.read()
            if ok and i >= 8:                       # 앞쪽 몇 장은 노출이 덜 맞음
                frames.append(f)
    finally:
        cap.release()
    if not frames:
        raise HolderError('카메라 영상을 읽지 못함')
    if H is None:
        H = json.loads(CAMERA_CALIB_PATH.read_text(encoding='utf-8'))['H_camera_to_base']
    poses = []
    for f in frames:
        try:
            poses.append(find_holder(f, holder_marker_id(cfg), H))
        except HolderError:
            pass
    if len(poses) < len(frames) // 2:
        raise HolderError(f'거치대 마커 {holder_marker_id(cfg)} 가 카메라에 잘 보이지 않음 ({len(poses)}/{len(frames)}장, 가림·반사·화면 밖 확인)')
    return combine_poses(poses)


def combine_poses(poses):
    """여러 장의 (x, y, yaw) → 위치는 중앙값, 각도는 원형 평균."""
    xs, ys, yaws = zip(*poses)
    r = np.radians(yaws)
    return float(np.median(xs)), float(np.median(ys)), math.degrees(math.atan2(np.sin(r).sum(), np.cos(r).sum()))


def holder_marker_id(cfg):
    mid = (cfg.get('vision') or {}).get('holder_marker_id')
    if mid is None:
        raise HolderError('configs/default.yaml 의 vision.holder_marker_id 가 없음')
    return int(mid)


# ---------------------------------------------------------------- 교시값
def load_holder(path=POSES_PATH, rehearsal=False):
    data = json.loads(Path(path).read_text(encoding='utf-8')) if Path(path).exists() else {}
    h = data.get('pen_holder')
    if not h or any(n not in h for n in POSE_NAMES):
        raise HolderError('거치대 자세가 교시되지 않음 (python -m robot_games.calibrate holder --jog --execute-motion)')
    if not (h.get('verified') or rehearsal):
        raise HolderError('pen_holder.verified 가 false. 리허설(--rehearsal)로 확인한 뒤 robot_poses.json 에서 true 로 바꾸세요')
    return h


def holder_poses(h, marker, lift=0.0):
    """교시값 h 와 지금 마커 위치로 BASE 의 접근·잡기 자세. 교시 때보다 많이 옮겨지거나 돌아가 있으면 거부."""
    t = h['marker_at_teach']
    shift = math.hypot(marker[0] - t[0], marker[1] - t[1])
    turn = abs(_wrap(marker[2] - t[2]))
    if shift > MAX_SHIFT_MM:
        raise HolderError(f'거치대가 교시 때보다 {shift:.0f}mm 옮겨짐 (한도 {MAX_SHIFT_MM:g}) → 오검출이 아닌지 확인')
    if turn > MAX_YAW_CHANGE_DEG:
        raise HolderError(f'거치대가 교시 때보다 {turn:.0f}° 돌아감 (한도 {MAX_YAW_CHANGE_DEG:g}°) → 교시 때 방향에 가깝게 놓기')
    grasp = from_marker_frame(h['grasp'], marker)
    # approach_dz_mm: 접근 높이(잡기 위 mm). 펜이 컵에서 다 빠져나와야 하므로 교시한 높이보다 우선한다
    dz = float(h.get('approach_dz_mm') or (float(h['approach'][2]) - grasp[2]))
    if dz < MIN_APPROACH_DZ_MM:
        raise HolderError(f'접근 높이가 잡기보다 {dz:.0f}mm 위 (최소 {MIN_APPROACH_DZ_MM:g}mm) — 다시 교시')
    # 접근 자세는 잡기 자세 바로 위(같은 XY·방향). 오르내림이 수직이어야 부품이 컵 벽에 끌리지 않는다
    approach = list(grasp)
    approach[2] = grasp[2] + dz
    grasp[2] += lift
    approach[2] += lift
    return {'approach': approach, 'grasp': grasp}


def rehearsal_lift(h):
    """리허설에서 올릴 높이: 50mm 이상, 그리고 펜이 컵 밖에 있도록 적어도 접근-잡기 높이 차만큼."""
    return max(REHEARSAL_LIFT_MM, float(h.get('approach_dz_mm') or (float(h['approach'][2]) - float(h['grasp'][2]))))


# ---------------------------------------------------------------- 집기·반납
class BackedOff(RuntimeError):
    """거치대에서 오류가 나서 접근 높이로 수직으로 빠져 나온 상태. 원래 오류는 __cause__."""


def _back_off(robot, poses, err, log):
    """거치대 안에서 오류 → 그리퍼는 건드리지 않고 접근 높이로 수직 상승. 실패하면 그대로 두고 사람을 부른다."""
    try:
        robot.move_line(poses['approach'], SLOW_MM_S)
    except BaseException as e2:
        raise RuntimeError(f'{err} → 위로 빠지기도 실패({e2}). 로봇이 거치대에 있음: 수동으로 확인하세요') from err
    log(f'[HOLDER] 오류로 접근 높이까지 빠짐: {err}')
    raise BackedOff(str(err)) from err


def _holding_now(gripper):
    """그리퍼가 이미 펜 부품을 쥐고 있으면 True (위치를 못 읽는 설정이면 None)."""
    lo, hi = gripper.cfg.get('holding_min'), gripper.cfg.get('holding_max')
    if not gripper.cfg.get('readback_host') or lo is None or hi is None:
        return None
    return lo <= gripper.position() <= hi


def pick(robot, gripper, poses, rehearsal=False, log=print):
    """(이미 쥐고 있으면 거부) 열기(확인) → 접근 → 하강 → 닫기(파지 확인) → 상승.
    거치대에 내려간 뒤 오류가 나면: 파지 실패면 열고, 그 밖에는 그리퍼를 그대로 둔 채 수직으로 빠진 뒤 BackedOff."""
    from .gripper import PenNotHeld
    if _holding_now(gripper):
        raise HolderError('그리퍼가 이미 펜 부품을 쥐고 있음 — 열면 떨어지므로 집기를 하지 않음 (반납 먼저)')
    gripper.open()
    robot.move_line(poses['approach'])
    robot.move_line(poses['grasp'], SLOW_MM_S)
    try:
        if rehearsal:
            log('[HOLDER] 리허설: 잡기 자세 도착 — 닫지 않음')
            pos = None
        else:
            try:
                pos = gripper.close()
            except PenNotHeld:
                gripper.open()
                raise
    except BaseException as e:
        _back_off(robot, poses, e, log)
    robot.move_line(poses['approach'], SLOW_MM_S)
    if pos is not None:
        log(f'[HOLDER] 펜 부품 잡음 (그리퍼 위치 {pos})')
    return pos


def give_back(robot, gripper, poses, rehearsal=False, log=print):
    """접근 → 하강(꽂기) → 열기(확인) → 상승. 열기에서 오류가 나면 쥔 채로 수직으로 빠진 뒤 BackedOff."""
    robot.move_line(poses['approach'])
    robot.move_line(poses['grasp'], SLOW_MM_S)
    try:
        if rehearsal:
            log('[HOLDER] 리허설: 반납 자세 도착 — 열지 않음')
        else:
            gripper.open()
    except BaseException as e:
        _back_off(robot, poses, e, log)
    robot.move_line(poses['approach'], SLOW_MM_S)
    log('[HOLDER] 반납 완료' if not rehearsal else '[HOLDER] 리허설 반납 경로 완료')


def _near_holder(robot, poses, margin_mm=15.0):
    """현재 펜 끝이 거치대 기둥(잡기 XY 반경 60mm) 안에서 접근 높이보다 낮으면 True. 위치를 못 읽으면 True(안전 쪽)."""
    try:
        p = robot.current_posx()
    except Exception:
        return True
    g, ap = poses['grasp'], poses['approach']
    return math.hypot(p[0] - g[0], p[1] - g[1]) < 60.0 and p[2] < ap[2] - margin_mm


def save_pick_marker(marker, path=None):
    path = Path(path or STATE_PATH)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'marker': list(marker)}), encoding='utf-8')


def rough_marker_for_return(measure, path=None, log=print):
    """반납 첫 이동(거치대 위 높은 접근 자세)에 쓸 대략 위치. 펜을 쥐고 home 에 있으면 늘어진 펜이 마커를 가려
    잘못 잴 수 있으므로(2026-11-11 20° 오검출), 집을 때 저장한 위치를 먼저 쓰고 접근 자세에서 다시 잰다(view_and_refine)."""
    path = Path(path or STATE_PATH)
    if path.exists():                               # 집을 때 잰 위치가 있으면 그걸로 (home 에서 재면 쥔 펜이 가림)
        return tuple(json.loads(path.read_text(encoding='utf-8'))['marker'])
    log('[HOLDER] 집을 때 잰 위치가 없음 → 지금 잰 값으로 거치대 위까지 이동 (가려질 수 있음)')
    return measure()


def view_and_refine(robot, h, rough, measure, lift=0.0, log=print):
    """대략 위치의 접근 높이에서 옆으로 VIEW_OFFSET_Y_MM 비킨 '보기 자세'로 가서 멈춘 뒤 마커를 다시 잰다.
    2026-11-11: 집을 때 저장한 위치를 믿고 꽂았다가 그 사이 거치대가 20mm 밀려 있어 펜이 걸려 넘어짐.
    home(쥔 펜)과 거치대 바로 위(팔)에서는 마커가 가려 20° 넘게 틀리게 재짐 → 옆에서 잰다."""
    view = list(holder_poses(h, rough, lift)['approach'])
    view[1] += VIEW_OFFSET_Y_MM
    robot.move_line(view)
    marker = measure()                              # 안 보이면 HolderError → 꽂지 않음
    log(f'[HOLDER] 거치대 위에서 다시 잰 마커 ({marker[0]:.1f}, {marker[1]:.1f}) {marker[2]:.1f}°'
        f' (대략값과 {math.hypot(marker[0] - rough[0], marker[1] - rough[1]):.1f}mm 차이)')
    return marker, holder_poses(h, marker, lift)


# ---------------------------------------------------------------- 실행
def main(argv=None):
    p = argparse.ArgumentParser(description='펜 거치대에서 펜 부품 집기·반납 (실기)')
    p.add_argument('action', choices=['pick', 'return'])
    p.add_argument('--execute-motion', action='store_true')
    p.add_argument('--rehearsal', action='store_true', help='50mm 위에서 경로만, 그리퍼 닫기·열기 없음(집기 전 열기만)')
    p.add_argument('--lift', type=float, help='리허설 높이(mm, 기본=max(50, 접근-잡기)). 집기 리허설에서 부품 바로 위까지 보려면 10~20')
    p.add_argument('--config')
    a = p.parse_args(argv)
    if a.lift is not None and not a.rehearsal:
        print('[REFUSED] --lift 는 --rehearsal 과 같이 씁니다', file=sys.stderr)
        return 2
    if a.lift is not None and (a.lift < MIN_REHEARSAL_LIFT_MM or a.action == 'return'):
        print(f'[REFUSED] --lift 는 pick 리허설에서 {MIN_REHEARSAL_LIFT_MM:g}mm 이상만 (return 은 펜이 컵에 걸리지 않게 기본 높이)', file=sys.stderr)
        return 2
    if not a.execute_motion:
        print('[REFUSED] 실기 동작은 --execute-motion 이 필요합니다', file=sys.stderr)
        return 2
    cfg = yaml.safe_load(open(a.config or ROOT / 'configs' / 'default.yaml', encoding='utf8'))
    try:
        h = load_holder(rehearsal=a.rehearsal)
        measure = lambda: capture_holder(cfg)
        marker = measure() if a.action == 'pick' else rough_marker_for_return(measure)
        lift = (a.lift if a.lift is not None else rehearsal_lift(h)) if a.rehearsal else 0.0
        poses = holder_poses(h, marker, lift)
    except (HolderError, OSError, ValueError) as e:
        print(f'[REFUSED] {e}', file=sys.stderr)
        return 2
    print(f'[HOLDER] 마커 ({marker[0]:.1f}, {marker[1]:.1f}) {marker[2]:.1f}°  접근 {[round(v, 1) for v in poses["approach"][:3]]}'
          f'  잡기 {[round(v, 1) for v in poses["grasp"][:3]]}')
    if input('로봇이 움직입니다. 주변이 비었으면 RUN 입력: ').strip() != 'RUN':
        print('취소')
        return 1
    from .motion import GuardedDoosan
    from .gripper import FlangeGripper
    robot = GuardedDoosan(cfg['robot'], need_celebration=False)
    try:
        gripper = FlangeGripper(cfg['gripper'])
        robot.home()
        if a.action == 'pick':
            pick(robot, gripper, poses, rehearsal=a.rehearsal)
            if not a.rehearsal:
                save_pick_marker(marker)
        else:
            marker, poses = view_and_refine(robot, h, marker, measure, lift)
            give_back(robot, gripper, poses, rehearsal=a.rehearsal)
        robot.home()
        return 0
    except BaseException as e:                       # 소켓 시간 초과(OSError), Ctrl+C 등 모두
        print(f'[STOP] {type(e).__name__}: {e}', file=sys.stderr)
        # 거치대에서 수직으로 빠져 나온 경우(BackedOff)나 거치대에 내려가기 전 오류만 home 으로 간다.
        # 그 밖(빠지기 실패 등)에는 팔이 거치대 안에 있을 수 있으므로 관절 이동을 하지 않는다.
        safe = isinstance(e, (BackedOff, HolderError)) or not _near_holder(robot, poses)
        if safe and not robot.stopped():
            try:
                robot.home()
            except Exception as e2:
                print(f'home 실패: {e2}', file=sys.stderr)
        else:
            print('[STOP] 팔이 거치대 근처에 있어 자동으로 home 가지 않음. 상태를 확인하고 조그로 빼세요', file=sys.stderr)
        return 1
    finally:
        robot.close()


if __name__ == '__main__':
    sys.exit(main())
