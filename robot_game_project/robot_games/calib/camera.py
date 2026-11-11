"""카메라 열기. 카드·미로·틱택토·보정이 모두 이 함수로 같은 카메라를 같은 설정으로 연다.

configs/default.yaml 의 camera:
  name   : 장치 이름에 들어 있는 글자(예: RealSense). 주면 /dev/video 번호를 자동으로 찾는다
           (번호는 꽂을 때마다 바뀔 수 있음). 같은 이름의 노드가 여러 개면 영상 노드 중 USB 인터페이스 번호가
           가장 큰 것 = RealSense 의 컬러 스트림.
  index  : name 이 없거나 못 찾을 때 쓰는 번호 (0 = 노트북 내장 웹캠)
  width / height / fps / fourcc : RealSense 는 이 값을 지정해야 프레임이 나온다 (그냥 열면 read 가 실패).
명령줄의 --camera-index 를 주면 설정보다 우선한다.
"""
from __future__ import annotations

import re
from pathlib import Path

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[2]
SYSFS = Path('/sys/class/video4linux')


def camera_config(path=None) -> dict:
    cfg = yaml.safe_load(Path(path or ROOT / 'configs' / 'default.yaml').read_text(encoding='utf-8'))
    return cfg.get('camera') or {}


def find_device(name: str, sysfs: Path = SYSFS):
    """이름에 name 이 들어 있는 영상 노드(/dev/videoN) 경로. 없으면 None."""
    best = None
    for d in sorted(sysfs.glob('video*')):
        try:
            if name.lower() not in (d / 'name').read_text().lower():
                continue
            if (d / 'index').read_text().strip() != '0':      # index 1 은 메타데이터 노드
                continue
            m = re.search(r':\d+\.(\d+)$', str((d / 'device').resolve()))
            iface = int(m.group(1)) if m else 0
        except OSError:
            continue
        if best is None or iface > best[0]:
            best = (iface, f'/dev/{d.name}')
    return best[1] if best else None


def resolve_device(cam: dict, index=None):
    if index is not None:
        return int(index)
    if cam.get('name'):
        dev = find_device(cam['name'])
        if dev:
            return dev
        print(f"[CAMERA] '{cam['name']}' 카메라를 찾지 못해 index {cam.get('index', 0)} 을 씁니다")
        return int(cam.get('index', 0))
    # 팀원 default.yaml 에는 camera.name 이 없으므로, 미로와 같은 자동 선택(RealSense 컬러 우선)을 쓴다
    from ..camera_select import choose_camera
    return choose_camera(int(cam.get('index', 0)))


def open_camera(cam: dict | None = None, index=None, warmup=15):
    """카메라를 열고 첫 프레임이 나오는 것까지 확인한다. 실패하면 RuntimeError."""
    cam = camera_config() if cam is None else cam
    dev = resolve_device(cam, index)
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'카메라를 열 수 없음: {dev}')
    if index is None:                                   # 형식 설정은 설정 파일의 카메라에만 적용
        if cam.get('fourcc'):
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cam['fourcc'])))
        for key, prop in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT),
                          ('fps', cv2.CAP_PROP_FPS)):
            if cam.get(key):
                cap.set(prop, cam[key])
    ok, fails = False, 0
    for _ in range(max(1, warmup)):                     # 자동노출 안정화 겸 프레임 확인
        ok, _frame = cap.read()
        fails = 0 if ok else fails + 1
        if fails >= 2:                                  # 실패한 read 는 하나에 10초씩 걸리므로 오래 끌지 않는다
            break
    if not ok:
        cap.release()
        raise RuntimeError(f'카메라 {dev} 에서 프레임이 나오지 않음 (camera.width/height/fps/fourcc 설정 확인)')
    return cap
