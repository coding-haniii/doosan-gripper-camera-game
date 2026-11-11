"""키보드 스텝 조그. 직접 교시(핸드가이딩)가 안 되는 로봇에서 보정·교시 위치를 맞출 때 쓴다.

키를 한 번 누르면 정해진 거리만큼만 움직이고 멈춘다(누르고 있는 동안 계속 가는 방식이 아님).
움직이는 동안 눌린 키는 버린다(누르고 있어도 한 칸씩만 감).
이동은 robot.move_line / move_joint 를 거치므로 속도 상한과 SafetyMonitor(보호정지·비상정지 감시)가 그대로 적용된다.
자세(rx,ry,rz)는 바꾸지 않는다 → 펜이 수직이면 계속 수직.

  w/s : X+ / X-      a/d : Y+ / Y-      r/f : Z+ (위) / Z- (아래)
  1 2 3 4 : 스텝 0.2 / 1 / 10 / 50 mm
  j : 관절 하나를 상대 각도로 (예: "1 15" = J1 +15°, 한 번에 최대 ±30°)
  p : 현재 좌표 출력      Enter : 이 위치로 확정      q : 중단
"""
from __future__ import annotations

import sys

from .motion import MotionFailed

STEPS_MM = {'1': 0.2, '2': 1.0, '3': 10.0, '4': 50.0}
MOVES = {'w': (0, +1), 's': (0, -1), 'a': (1, +1), 'd': (1, -1), 'r': (2, +1), 'f': (2, -1)}
NEAR_FLOOR_MM = 15.0        # 바닥 근처에서는 내려가는 스텝을 1mm 이하로 제한
MAX_JOINT_STEP_DEG = 30.0
HELP = 'w/s X±  a/d Y±  r/f Z±  1-4 스텝(0.2/1/10/50mm)  j 관절  p 좌표  Enter 확정  q 중단'


def read_key():
    """키 하나. 터미널이 아니면(파이프·테스트) 한 줄을 읽어 첫 글자를 쓴다. 빈 줄 = Enter."""
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        if line == '':
            return 'q'              # 입력 끝
        return line.strip()[:1] or '\n'
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return '\n' if ch in ('\r', '\n') else ch


def flush_keys():
    """이동하는 동안 쌓인 키 입력을 버린다. 키를 누르고 있으면 자동 반복으로 입력이 줄을 서서,
    손을 뗀 뒤에도 로봇이 계속 움직이기 때문이다."""
    if sys.stdin.isatty():
        import termios
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)


def _posx(dr, ref=None):
    p = dr.get_current_posx() if ref is None else dr.get_current_posx(ref)
    p = p[0] if isinstance(p, tuple) else p
    return [float(v) for v in p]


class JogPanel:
    def __init__(self, robot, min_z=None, key_fn=read_key, input_fn=input, flush_fn=flush_keys):
        self.robot, self.dr = robot, robot.dr
        self.min_z = min_z
        self.key_fn, self.input_fn, self.flush_fn = key_fn, input_fn, flush_fn
        self.step = 10.0

    def check_frames(self):
        """ROS 가 읽는 BASE 좌표와 DART 가 보여 주는 WORLD 좌표가 같은지 확인(다르면 바닥 높이 기준이 달라짐)."""
        base, world = _posx(self.dr, self.dr.DR_BASE), _posx(self.dr, self.dr.DR_WORLD)
        diff = max(abs(a - b) for a, b in zip(base[:3], world[:3]))
        print(f'현재 TCP  BASE {[round(v, 2) for v in base[:3]]}  WORLD {[round(v, 2) for v in world[:3]]}')
        if diff > 0.5:
            print(f'[경고] BASE 와 WORLD 가 {diff:.1f}mm 다릅니다. min_pen_z_base_mm 는 BASE 기준으로 다시 정하세요.')
        return diff

    def _show(self):
        p = _posx(self.dr)
        print(f'  X {p[0]:8.2f}  Y {p[1]:8.2f}  Z {p[2]:8.2f}   (스텝 {self.step:g}mm)')
        return p

    def _move(self, axis, sign):
        cur = _posx(self.dr)
        step = self.step
        if axis == 2 and sign < 0 and self.min_z is not None:
            if cur[2] - self.min_z < NEAR_FLOOR_MM:
                step = min(step, 1.0)
            if cur[2] - step < self.min_z:
                print(f'  [거부] Z {cur[2] - step:.1f} < 하한 {self.min_z:g}mm (robot.min_pen_z_base_mm)')
                return
        target = list(cur)
        target[axis] += sign * step
        try:
            self.robot.move_line(target, 'jog', jog=True)
        except MotionFailed as e:
            print(f'  [거부] {e} → 더 작은 스텝(1~3 키)이나 다른 방향으로')
        self._show()

    def _joint(self):
        raw = self.input_fn('  관절번호 각도변화(도), 예 "1 15" > ').split()
        try:
            j, delta = int(raw[0]), float(raw[1])
        except (IndexError, ValueError):
            print('  형식: <1~6> <각도>')
            return
        if not 1 <= j <= 6 or abs(delta) > MAX_JOINT_STEP_DEG:
            print(f'  관절 1~6, 한 번에 ±{MAX_JOINT_STEP_DEG:g}° 이하')
            return
        q = [float(v) for v in self.dr.get_current_posj()]
        q[j - 1] += delta
        try:
            self.robot.move_joint(q, 'jog joint')
        except MotionFailed as e:
            print(f'  [거부] {e}')
        self._show()

    def position(self, prompt):
        """운영자가 조그로 위치를 맞추고 Enter 를 누를 때까지. q 면 KeyboardInterrupt."""
        print(f'\n{prompt}\n  {HELP}')
        self._show()
        while True:
            k = self.key_fn()
            if k == '\n':
                return
            if k == 'q':
                raise KeyboardInterrupt('조그 중단')
            if k in STEPS_MM:
                self.step = STEPS_MM[k]
                print(f'  스텝 {self.step:g}mm')
            elif k in MOVES:
                self._move(*MOVES[k])
                self.flush_fn()
            elif k == 'j':
                self._joint()
                self.flush_fn()
            elif k == 'p':
                self._show()
