"""교시한 로봇 자세 (configs/robot_poses.json). `robot-calibrate poses` 가 만든다.

키
  home, celebration, camera_clear : 관절값 [j1..j6] deg (posj)
  pen_holder_approach, pen_holder_grasp : 직교 좌표 [x,y,z,rx,ry,rz] mm/deg (posx, base 기준, 펜 TCP 아님 주의 → 아래 참고)
펜 거치대 자세는 '펜을 잡기 전' 상태에서 교시하므로 플랜지/그리퍼 TCP 기준이다. pen_tcp_name 으로 바꾸기 전에 사용한다.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
POSES_PATH = ROOT / 'configs' / 'field' / 'robot_poses.json'
JOINT_KEYS = ('home', 'celebration', 'camera_clear')
CART_KEYS = ('pen_holder_approach', 'pen_holder_grasp')


def load_poses(path=None) -> dict:
    p = Path(path or POSES_PATH)
    if not p.exists():
        return {}
    data = json.loads(p.read_text(encoding='utf-8'))
    for k in JOINT_KEYS + CART_KEYS:
        v = data.get(k)
        if v is not None and len(v) != 6:
            raise ValueError(f'{p.name}: {k} 는 값 6개여야 합니다 ({v!r})')
    return data


def save_poses(data: dict, path=None):
    p = Path(path or POSES_PATH)
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')
    return p
