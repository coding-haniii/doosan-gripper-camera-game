"""실행할 때마다 쓸 수 있는 카메라 번호(/dev/videoN)를 찾는다.

RealSense 는 영상 노드가 여러 개(깊이·적외선·컬러)이고, 번호와 이름이 PC·연결마다 다르다.
(이 프로젝트의 tictactoe/run_camera.py 는 이름에 'rgb' 가 있는 노드만 찾는데, 어떤 PC 에서는 모든 노드 이름이
'Depth Camera 455f' 로만 나와 못 찾는다.) 그래서 후보를 고른 뒤 실제로 컬러 영상이 나오는지 읽어서 확인한다.

순서: 이름에 rgb/color 가 있는 RealSense 노드 → USB 인터페이스 번호가 큰 RealSense 노드(D455f 는 컬러가 마지막)
      → 설정의 기본 번호(fallback). 깊이·적외선 노드는 세 채널 값이 같은 흑백 영상이라 걸러진다.
"""
import re
from pathlib import Path

import cv2
import numpy as np

VIDEO_DEVICES = Path('/sys/class/video4linux')


def _read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return ''


def realsense_candidates(devices=VIDEO_DEVICES):
    """RealSense 영상 캡처 노드 번호를 컬러일 가능성이 높은 순서로."""
    found = []
    for node in devices.glob('video[0-9]*'):
        name = _read(node / 'name').casefold()
        iface = _read(node / 'device' / 'interface').casefold()
        if 'realsense' not in name + iface or _read(node / 'index') not in ('', '0'):
            continue                                    # index 1 이상은 메타데이터 노드
        m = re.search(r':\d+\.(\d+)$', str((node / 'device').resolve()))
        usb_iface = int(m.group(1)) if m else 0
        named_color = 'rgb' in iface or 'color' in iface
        found.append((named_color, usb_iface, int(node.name.removeprefix('video'))))
    return [index for _, _, index in sorted(found, reverse=True)]


def is_color(frame):
    """세 채널이 거의 같으면 흑백(깊이·적외선을 BGR 로 바꾼 것)으로 본다."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return False
    f = frame.astype(np.int16)
    return float(np.abs(f[..., 0] - f[..., 1]).mean() + np.abs(f[..., 1] - f[..., 2]).mean()) > 2.0


def can_read(index, need_color=True, tries=3):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            return False
        for _ in range(tries):
            ok, frame = cap.read()
            if ok and frame is not None:
                return is_color(frame) if need_color else frame.ndim == 3
        return False
    finally:
        cap.release()


def choose_camera(fallback=0, devices=VIDEO_DEVICES, reader=can_read):
    """쓸 카메라 번호. RealSense 컬러를 우선, 없으면 fallback. 둘 다 안 되면 RuntimeError."""
    for index in realsense_candidates(devices):
        if reader(index):
            print(f'[CAMERA] RealSense 컬러: /dev/video{index}', flush=True)
            return index
    if reader(fallback, need_color=False):
        print(f'[CAMERA] RealSense 컬러를 찾지 못함 → 기본 카메라 /dev/video{fallback}', flush=True)
        return fallback
    raise RuntimeError(f'쓸 수 있는 카메라가 없음 (RealSense 컬러, /dev/video{fallback} 모두 실패)')
