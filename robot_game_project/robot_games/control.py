"""컨트롤 센터: 조그·home·그리퍼·펜 거치대·카메라 미리보기·게임 실행을 한 창에서.

  실행 (ROS source + bringup 필요):  python -m robot_games.control

구조
  - 창(Tkinter)은 로봇에 직접 붙지 않는다. DSR_ROBOT2 는 프로세스당 연결 하나라서, 로봇 연결은
    `python -m robot_games.control --worker` 프로세스가 맡고 창과는 표준입출력 JSON 한 줄씩 주고받는다.
  - 펜 집기·반납(robot_games.pen_holder)과 게임(robot_games.app)은 따로 연결하므로, 그동안 창은
    worker 와 카메라를 놓았다가 끝나면 다시 잡는다. 게임은 RUN 입력 등이 있어 새 터미널 창에서 실행한다.
  - 조그 이동은 jog.JogPanel 을 그대로 써서 펜 끝 하한·바닥 근처 1mm 제한·도착 확인이 같다.
  - 소프트웨어 정지 버튼은 없다. 위험하면 티치 펜던트의 비상정지를 누른다.
"""
from __future__ import annotations

import argparse
import base64
import json
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

from .jog import JogPanel, STEPS_MM, MAX_JOINT_STEP_DEG

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'configs' / 'default.yaml'
TAG = '@@'                       # worker → 창 응답 줄 표시 (라이브러리 출력과 구분)
ROS_SETUP = 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash'
GAMES = {                         # 버튼 이름 → robot_games.app 인자 (펜 집기·반납은 app 이 한다)
    '틱택토': ['--game', '1'],
    '미로': ['--game', '2', '--paper-maze'],
    '카드로 고르기': ['--game', 'camera'],
}


# ------------------------------------------------------------------ worker (로봇 연결 프로세스)
class Worker:
    """창의 요청 하나를 처리해 dict 로 답한다. robot 은 GuardedDoosan(또는 같은 인터페이스)."""

    def __init__(self, robot, cfg, gripper_factory=None):
        self.robot, self.cfg = robot, cfg
        self.panel = JogPanel(robot, min_z=cfg['robot'].get('min_pen_z_base_mm'),
                              key_fn=None, input_fn=input, flush_fn=lambda: None)
        self._gripper_factory = gripper_factory
        self._gripper = None

    def gripper(self):
        if self._gripper is None:
            if self._gripper_factory is None:
                from .gripper import FlangeGripper
                self._gripper_factory = lambda: FlangeGripper(self.cfg['gripper'])
            self._gripper = self._gripper_factory()
        return self._gripper

    def status(self):
        p, q = self.robot.current_posx(), self.robot.current_posj()
        return {'ok': True, 'posx': [round(v, 2) for v in p[:6]], 'posj': [round(v, 2) for v in q[:6]],
                'state': self.robot.state(), 'stopped': self.robot.stopped()}

    def handle(self, req):
        cmd = req.get('cmd')
        try:
            if cmd == 'status':
                return self.status()
            if cmd in ('jog', 'joint', 'home', 'grip') and self.robot.stopped():
                return {'ok': False, 'error': '로봇이 정지 상태(보호정지·비상정지·서보 꺼짐) — 펜던트에서 해제하세요'}
            if cmd == 'jog':
                step, axis, sign = float(req['step']), int(req['axis']), int(req['sign'])
                if step not in STEPS_MM.values() or axis not in (0, 1, 2) or sign not in (-1, 1):
                    return {'ok': False, 'error': f'잘못된 조그 요청 {req}'}
                self.panel.step = step
                self.panel._move(axis, sign)           # 하한·거부는 JogPanel 이 처리하고 출력으로 알림
                return self.status()
            if cmd == 'joint':
                j, delta = int(req['joint']), float(req['delta'])
                if not 1 <= j <= 6 or abs(delta) > MAX_JOINT_STEP_DEG:
                    return {'ok': False, 'error': f'관절 1~6, 한 번에 ±{MAX_JOINT_STEP_DEG:g}° 이하'}
                self.panel.input_fn = lambda _prompt: f'{j} {delta}'
                self.panel._joint()
                return self.status()
            if cmd == 'home':
                self.robot.home()
                return self.status()
            if cmd == 'grip':
                g, action = self.gripper(), req.get('action')
                if action == 'open':
                    pos = g.open()
                elif action == 'close':
                    pos = g.close(check_held=False)
                elif action == 'read':
                    pos = g.position()
                else:
                    return {'ok': False, 'error': f'잘못된 그리퍼 요청 {action}'}
                if pos is None and self.cfg['gripper'].get('readback_host'):
                    pos = g.position()
                return {**self.status(), 'gripper': pos}
            return {'ok': False, 'error': f'모르는 요청 {cmd}'}
        except KeyboardInterrupt as e:                 # JogPanel: 로봇 정지 상태
            return {'ok': False, 'error': str(e)}
        except Exception as e:
            return {'ok': False, 'error': f'{type(e).__name__}: {e}'}


def worker_main(config_path):
    proto = sys.stdout
    sys.stdout = sys.stderr                            # 라이브러리 print 는 창의 로그로
    send = lambda d: (proto.write(TAG + json.dumps(d, ensure_ascii=False) + '\n'), proto.flush())
    cfg = yaml.safe_load(open(config_path, encoding='utf8'))
    try:
        from .motion import GuardedDoosan
        robot = GuardedDoosan(cfg['robot'], need_celebration=False, for_calibration=True)
    except BaseException as e:
        send({'ready': False, 'error': f'{type(e).__name__}: {e}'})
        return 1
    worker = Worker(robot, cfg)
    send({'ready': True})
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except ValueError:
                send({'ok': False, 'error': f'JSON 아님: {line[:80]}'})
                continue
            send(worker.handle(req))
    finally:
        robot.close()
    return 0


class WorkerClient:
    """창 쪽: worker 프로세스를 띄우고 요청을 하나씩 보낸다(동시에 하나)."""

    def __init__(self, config_path, log):
        self.config_path, self.log = config_path, log
        self.proc = None
        self.lock = threading.Lock()

    def start(self, timeout_s=40.0):
        self.proc = subprocess.Popen([sys.executable, '-m', 'robot_games.control', '--worker', '--config', str(self.config_path)],
                                     cwd=str(ROOT), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, bufsize=1)
        threading.Thread(target=self._pump_stderr, daemon=True).start()
        reply = self._read(timeout_s)
        if not reply or not reply.get('ready'):
            self.stop()
            raise RuntimeError((reply or {}).get('error', '로봇 연결 프로세스가 응답하지 않음 (bringup 확인)'))

    def _pump_stderr(self):
        for line in self.proc.stderr:
            line = line.rstrip()
            if line and not line.startswith(('_robot_', '_srv_', '_topic_')):
                self.log(line)

    def _read(self, timeout_s=None):
        while True:
            line = self.proc.stdout.readline()
            if not line:
                return None
            if line.startswith(TAG):
                return json.loads(line[len(TAG):])

    def request(self, **req):
        with self.lock:
            if not self.alive():
                return {'ok': False, 'error': '로봇 연결 안 됨'}
            self.proc.stdin.write(json.dumps(req, ensure_ascii=False) + '\n')
            self.proc.stdin.flush()
            return self._read() or {'ok': False, 'error': '로봇 연결 프로세스가 끝남'}

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.proc is None:
            return
        with self.lock:
            try:
                self.proc.stdin.close()
                self.proc.wait(timeout=10)
            except Exception:
                self.proc.kill()
            self.proc = None


# ------------------------------------------------------------------ 카메라 미리보기
class Preview:
    """별도 스레드에서 카메라를 읽고 마커·격자를 그려 PNG(base64)로 둔다. 창이 주기적으로 가져간다."""

    def __init__(self, cfg, width=640):
        self.cfg, self.width = cfg, width
        self.latest, self.info = None, ''
        self._stop = threading.Event()
        self.thread = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self._stop.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def stop(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=5)
        self.thread, self.latest = None, None

    def _run(self):
        import cv2
        import numpy as np
        from runpy import run_path
        from .camera_select import choose_camera
        from .paper_maze import detect_marker_corners
        grid_corners = run_path(str(ROOT / 'robot_games/tictactoe/tictactoe.py'), run_name='robot_games._preview')['grid_corners']
        cam = self.cfg.get('camera') or {}

        def open_camera():
            self.info = '카메라 여는 중...'
            try:
                index = choose_camera(int(cam.get('index', 0)))     # RealSense 가 있으면 RealSense 만 (사용 중이면 예외)
            except RuntimeError as e:
                self.info = f'{e} (다시 여는 중)'
                return cv2.VideoCapture()                          # 열리지 않은 상태 → 아래에서 2초 뒤 다시
            c = cv2.VideoCapture(index, cv2.CAP_V4L2)
            c.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam.get('width', 1280)))
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam.get('height', 720)))
            return c

        cap, fails, dark_since = open_camera(), 0, None
        try:
            while not self._stop.is_set():
                ok, f = cap.read() if cap.isOpened() else (False, None)
                if not ok:
                    # 다른 프로그램(게임·펜 거치대·보정 도구)이 카메라를 쓰고 있으면 열려도 프레임이 안 온다 → 닫고 다시 연다
                    fails += 1
                    self.latest = None
                    if cap.isOpened():
                        self.info = f'카메라 영상 없음 — 다른 프로그램이 카메라를 쓰는지 확인 (다시 여는 중 {fails})'
                    cap.release()
                    self._stop.wait(2.0)
                    if not self._stop.is_set():
                        cap = open_camera()
                    continue
                fails = 0
                if f.mean() < 8:                       # 켜진 직후(노출 맞추는 중)나 렌즈가 가려졌을 때
                    dark_since = dark_since or time.time()
                    if time.time() - dark_since > 3:
                        self.info = '영상이 검게만 나옴 — 렌즈 가림·조명·다른 프로그램의 카메라 사용 확인'
                else:
                    dark_since = None
                marks = detect_marker_corners(f)
                grid = grid_corners(f)
                for mid, c in marks.items():
                    cv2.polylines(f, [np.int32(c)], True, (0, 200, 255), 2)
                    cv2.putText(f, str(mid), tuple(np.int32(c[0]) - [0, 6]), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 200, 255), 2)
                if grid is not None:
                    cv2.polylines(f, [np.int32(grid)], True, (0, 210, 0), 2)
                if dark_since is None:
                    self.info = f'마커 {sorted(marks)}  격자 {"OK" if grid is not None else "없음"}'
                h = int(f.shape[0] * self.width / f.shape[1])
                ok, png = cv2.imencode('.png', cv2.resize(f, (self.width, h)))
                if ok:
                    self.latest = base64.b64encode(png.tobytes()).decode('ascii')
        finally:
            cap.release()


# ------------------------------------------------------------------ 창
def run_ui(config_path):
    import tkinter as tk
    from tkinter import messagebox, scrolledtext

    cfg = yaml.safe_load(open(config_path, encoding='utf8'))
    root = tk.Tk()
    root.title('로봇 컨트롤 센터')
    logq = queue.Queue()
    log_file = ROOT / 'out' / 'control_center.log'
    log_file.parent.mkdir(exist_ok=True)

    def log(msg):
        line = f'[{time.strftime("%H:%M:%S")}] {msg}'
        logq.put(line)
        print(line, file=sys.stderr, flush=True)        # 창이 닫혀도 터미널·로그 파일에 남도록
        try:
            with open(log_file, 'a', encoding='utf-8') as fh:   # 원인 확인용(게임·테스트 출력 포함)
                fh.write(line + '\n')
        except OSError:
            pass
    client = WorkerClient(config_path, log)
    preview = Preview(cfg)
    state = {'busy': False, 'external': False, 'step': tk.DoubleVar(value=10.0), 'cam': tk.BooleanVar(value=True),
             'jdelta': tk.DoubleVar(value=5.0)}

    # --- 왼쪽: 카메라 / 로그
    left = tk.Frame(root)
    left.grid(row=0, column=0, sticky='nsew', padx=6, pady=6)
    TEXT_SIZE = {'width': 80, 'height': 20}               # 이미지가 없을 때(글자 단위) ≈ 640x360 픽셀
    cam_label = tk.Label(left, text='카메라 여는 중...', bg='#222', fg='#ddd', wraplength=600, **TEXT_SIZE)
    cam_label.image = None
    cam_label.pack()
    cam_info = tk.Label(left, text='', anchor='w')
    cam_info.pack(fill='x')
    logbox = scrolledtext.ScrolledText(left, height=12, width=90, state='disabled')
    logbox.pack(fill='both', expand=True)

    # --- 오른쪽: 상태 / 조그 / 그리퍼 / 펜 / 게임
    right = tk.Frame(root)
    right.grid(row=0, column=1, sticky='n', padx=6, pady=6)
    conn_var, pos_var, joint_var, grip_var = tk.StringVar(value='로봇 연결 안 됨'), tk.StringVar(), tk.StringVar(), tk.StringVar(value='그리퍼 위치 -')
    tk.Label(right, textvariable=conn_var, font=('', 11, 'bold')).pack(anchor='w')
    tk.Label(right, textvariable=pos_var, font=('Monospace', 10), justify='left').pack(anchor='w')
    tk.Label(right, textvariable=joint_var, font=('Monospace', 9), justify='left', fg='#555').pack(anchor='w')
    tk.Label(right, textvariable=grip_var).pack(anchor='w')
    motion_buttons = []

    def button(parent, text, cmd, motion=True, **kw):
        b = tk.Button(parent, text=text, command=cmd, **kw)
        if motion:
            motion_buttons.append(b)
        return b

    conn_row = tk.Frame(right)
    conn_row.pack(fill='x', pady=4)
    tk.Button(conn_row, text='로봇 연결', command=lambda: connect()).pack(side='left')
    tk.Button(conn_row, text='연결 끊기', command=lambda: disconnect()).pack(side='left')
    tk.Checkbutton(conn_row, text='카메라', variable=state['cam'], command=lambda: camera_toggle()).pack(side='left')

    jog = tk.LabelFrame(right, text='조그 (키보드: w/s X±  a/d Y±  r/f Z±  1-4 스텝)')
    jog.pack(fill='x', pady=4)
    steps = tk.Frame(jog)
    steps.grid(row=0, column=0, columnspan=4, sticky='w')
    for v in sorted(STEPS_MM.values()):
        tk.Radiobutton(steps, text=f'{v:g}mm', value=v, variable=state['step']).pack(side='left')
    for r, (label, axis) in enumerate([('X', 0), ('Y', 1), ('Z', 2)], start=1):
        tk.Label(jog, text=label, width=3).grid(row=r, column=0)
        button(jog, f'{label} −', lambda a=axis: jog_move(a, -1), width=6).grid(row=r, column=1)
        button(jog, f'{label} +', lambda a=axis: jog_move(a, +1), width=6).grid(row=r, column=2)
    tk.Label(jog, text='Z− 는 펜 끝 하한(robot.min_pen_z_base_mm) 아래로 가지 않음', fg='#777').grid(row=4, column=0, columnspan=4, sticky='w')
    joints = tk.LabelFrame(right, text='관절 (상대 각도)')
    joints.pack(fill='x', pady=4)
    tk.Label(joints, text='각도°').grid(row=0, column=0)
    tk.Spinbox(joints, from_=1, to=MAX_JOINT_STEP_DEG, increment=1, width=5, textvariable=state['jdelta']).grid(row=0, column=1, columnspan=2, sticky='w')
    for j in range(1, 7):
        r, c = 1 + (j - 1) // 3, ((j - 1) % 3) * 2
        button(joints, f'J{j}−', lambda j=j: joint_move(j, -1), width=4).grid(row=r, column=c)
        button(joints, f'J{j}+', lambda j=j: joint_move(j, +1), width=4).grid(row=r, column=c + 1)
    button(right, 'home 으로', lambda: run_cmd('home', cmd='home'), bg='#dde').pack(fill='x', pady=4)

    grip = tk.LabelFrame(right, text='그리퍼')
    grip.pack(fill='x', pady=4)
    button(grip, '열기', lambda: gripper('open')).pack(side='left', expand=True, fill='x')
    button(grip, '닫기', lambda: gripper('close')).pack(side='left', expand=True, fill='x')
    button(grip, '위치 읽기', lambda: gripper('read')).pack(side='left', expand=True, fill='x')

    pen = tk.LabelFrame(right, text='펜 거치대')
    pen.pack(fill='x', pady=4)
    button(pen, '펜 집기', lambda: pen_holder('pick')).pack(side='left', expand=True, fill='x')
    button(pen, '펜 반납', lambda: pen_holder('return')).pack(side='left', expand=True, fill='x')

    tests = tk.LabelFrame(right, text='테스트 (종이에 닿지 않음)')
    tests.pack(fill='x', pady=4)
    button(tests, '펜 집기 리허설', lambda: pen_rehearsal()).pack(side='left', expand=True, fill='x')
    button(tests, '틱택토 허공 테스트', lambda: tic_test()).pack(side='left', expand=True, fill='x')
    button(tests, '미로 허공 테스트', lambda: maze_test()).pack(side='left', expand=True, fill='x')

    games = tk.LabelFrame(right, text='게임 (새 터미널 창에서 실행, 펜 집기·반납 포함)')
    games.pack(fill='x', pady=4)
    for name in GAMES:
        button(games, name, lambda n=name: game(n)).pack(side='left', expand=True, fill='x')
    tk.Label(right, text='위험하면 티치 펜던트 비상정지', fg='#a00').pack(anchor='w', pady=(8, 0))

    # --- 동작
    def set_busy(b):
        state['busy'] = b
        for w in motion_buttons:
            w.configure(state='disabled' if b else 'normal')

    def show_status(r):
        if not r.get('ok'):
            log(f'[거부] {r.get("error")}')
            return
        p, q = r['posx'], r['posj']
        pos_var.set(f'X {p[0]:8.2f}  Y {p[1]:8.2f}  Z {p[2]:8.2f}\nA {p[3]:8.2f}  B {p[4]:8.2f}  C {p[5]:8.2f}')
        joint_var.set('J ' + ' '.join(f'{v:7.2f}' for v in q))
        conn_var.set(f'로봇 연결됨 · 상태 {r["state"]}' + (' · 정지 상태!' if r['stopped'] else ''))
        if r.get('gripper') is not None:
            grip_var.set(f'그리퍼 위치 {r["gripper"]}')

    def background(fn, done=None):
        def run():
            try:
                res = fn()
            except Exception as e:
                res = e
            root.after(0, lambda: done(res) if done else None)
        threading.Thread(target=run, daemon=True).start()

    def run_cmd(label, **req):
        if state['busy'] or state['external']:
            return
        if not client.alive():
            log('로봇이 연결되어 있지 않음 → [로봇 연결]')
            return
        set_busy(True)

        def done(r):
            set_busy(False)
            if isinstance(r, Exception):
                log(f'{label} 실패: {r}')
            else:
                show_status(r)
        background(lambda: client.request(**req), done)

    def jog_move(axis, sign):
        run_cmd('조그', cmd='jog', axis=axis, sign=sign, step=state['step'].get())

    def joint_move(j, sign):
        try:
            d = float(state['jdelta'].get())
        except (tk.TclError, ValueError):
            log('관절 각도는 숫자')
            return
        run_cmd('관절', cmd='joint', joint=j, delta=sign * d)

    def gripper(action):
        if action == 'open' and not messagebox.askyesno('그리퍼 열기', '펜을 쥐고 있으면 떨어집니다. 열까요?'):
            return
        log(f'그리퍼 {action} ...')
        run_cmd('그리퍼', cmd='grip', action=action)

    def connect():
        if client.alive() or state['external']:
            return
        conn_var.set('연결 중...')
        log('로봇 연결 중 (bringup 이 떠 있어야 함)')

        def done(r):
            if isinstance(r, Exception):
                conn_var.set('로봇 연결 안 됨')
                log(f'연결 실패: {r}')
            else:
                log('로봇 연결됨')
                run_cmd('상태', cmd='status')
        background(client.start, done)

    def disconnect():
        if state['busy']:
            return
        client.stop()
        conn_var.set('로봇 연결 안 됨')
        log('로봇 연결 끊음')

    def camera_toggle():
        if state['cam'].get() and not state['external']:
            preview.start()
        else:
            preview.stop()
            cam_label.configure(image='', text='카메라 꺼짐', **TEXT_SIZE)
            cam_label.image = None

    def external(title, argv, done_msg=None, terminal=False):
        """로봇 연결·카메라를 놓고 다른 프로그램을 실행, 끝나면 다시 잡는다."""
        if state['busy'] or state['external']:
            return
        state['external'] = True
        set_busy(True)
        was_connected, was_cam = client.alive(), state['cam'].get()
        client.stop()
        preview.stop()
        cam_label.configure(image='', text=f'{title} 실행 중 (카메라 사용)', **TEXT_SIZE)
        conn_var.set(f'{title} 실행 중 — 로봇 연결을 잠시 놓음')
        log(f'=== {title} 시작 ===')

        def run():
            if terminal:
                cmd = f'cd {ROOT} && {ROS_SETUP} && {" ".join(argv)}; echo; read -p "끝났습니다. Enter 로 창 닫기"'
                rc = subprocess.call(['gnome-terminal', '--wait', '--title', title, '--', 'bash', '-c', cmd])
            else:
                p = subprocess.Popen(argv, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
                for line in p.stdout:
                    line = line.rstrip()
                    if line and not line.startswith(('_robot_', '_srv_', '_topic_')):
                        log(line)
                rc = p.wait()
            return rc

        def done(rc):
            log(f'=== {title} 끝 (종료코드 {rc}) ===' + (f' {done_msg(rc)}' if done_msg and not isinstance(rc, Exception) else ''))
            state['external'] = False
            set_busy(False)
            if was_cam:
                camera_toggle()
            if was_connected:
                connect()
        background(run, done)

    def pen_holder(action):
        name = '펜 집기' if action == 'pick' else '펜 반납'
        msgs = {0: '완료', 3: '이미 쥐고 있음(움직이지 않음)', 4: '그리퍼가 비어 있음(움직이지 않음)'}
        external(name, [sys.executable, '-m', 'robot_games.pen_holder', action, '--execute-motion', '--yes'],
                 lambda rc: msgs.get(rc, '실패 — 로그 확인'))

    def pen_rehearsal():
        msgs = {0: '완료 — 손가락이 부품 양옆 가운데에 왔는지 확인', 3: '이미 펜을 쥐고 있음 → 먼저 펜 반납'}
        external('펜 집기 리허설', [sys.executable, '-m', 'robot_games.pen_holder', 'pick', '--execute-motion', '--yes',
                                 '--rehearsal', '--lift', '15'], lambda rc: msgs.get(rc, '실패 — 로그 확인'))

    def tic_test():
        if not messagebox.askyesno('틱택토 허공 테스트', '펜 집기 → 가운데·네 모서리 칸에 종이 5mm 위로 X → 펜 반납.\n'
                                   '빈 격자가 마커 사이에 있고 주변이 비었나요?'):
            return
        external('틱택토 허공 테스트', [sys.executable, '-m', 'robot_games.control', '--tic-air-test', '--config', str(config_path)],
                 lambda rc: {0: '완료', 2: '격자 안 보임(움직이지 않음)'}.get(rc, '실패 — 로그 확인'))

    def maze_test():
        if not messagebox.askyesno('미로 허공 테스트', '펜 집기 → \'Maze camera\' 창에서 SPACE 로 촬영(Q 는 취소) → 풀이 결과 5초 표시 → '
                                   '종이 5mm 위로 미로 경로 → 펜 반납.\n'
                                   '미로 종이가 마커 사이에 있고 주변이 비었나요?'):
            return
        external('미로 허공 테스트', [sys.executable, '-m', 'robot_games.control', '--maze-air-test', '--config', str(config_path)],
                 lambda rc: {0: '완료', 1: '취소·중단 — 로그 확인', 2: '보정 확인 필요'}.get(rc, '실패 — 로그 확인'))

    def game(name):
        if not messagebox.askyesno(name, f'{name} 을(를) 실기로 실행합니다(펜 집기 → 게임 → 반납).\n주변이 비었나요?'):
            return
        external(name, [sys.executable, '-m', 'robot_games.app', *GAMES[name], '--mode', 'real', '--execute-motion'], terminal=True)

    # --- 키보드 조그 (움직이는 동안 들어온 키는 버림: busy 면 무시)
    keymap = {'w': (0, 1), 's': (0, -1), 'a': (1, 1), 'd': (1, -1), 'r': (2, 1), 'f': (2, -1)}
    steps_by_key = {k: v for k, v in STEPS_MM.items()}

    def on_key(e):
        if isinstance(root.focus_get(), (tk.Entry, tk.Spinbox)):
            return
        k = e.char.lower()
        if k in keymap:
            jog_move(*keymap[k])
        elif k in steps_by_key:
            state['step'].set(steps_by_key[k])
    root.bind('<Key>', on_key)

    # --- 주기 작업: 로그, 카메라, 상태
    def tick():
        while not logq.empty():
            logbox.configure(state='normal')
            logbox.insert('end', logq.get() + '\n')
            logbox.see('end')
            logbox.configure(state='disabled')
        frame = preview.latest
        if frame is not None and frame is not state.get('shown'):
            img = tk.PhotoImage(data=frame, format='png')
            cam_label.configure(image=img, text='', width=img.width(), height=img.height())
            cam_label.image, state['shown'] = img, frame
        elif frame is None and preview.running():
            # 영상이 끊기면 멈춘 화면 대신 안내 문구. 이미지가 없을 때 width/height 는 글자 단위라 다시 지정해야
            # 한다(픽셀 640 이 640 글자로 바뀌어 창이 화면보다 커지고 검게 덮였음)
            cam_label.configure(image='', text=preview.info, **TEXT_SIZE)
            cam_label.image, state['shown'] = None, None
        if preview.running():
            cam_info.configure(text=preview.info)
        root.after(100, tick)

    def poll():
        if client.alive() and not state['busy'] and not state['external']:
            background(lambda: client.request(cmd='status'), lambda r: None if isinstance(r, Exception) else show_status(r) if r.get('ok') else None)
        root.after(1500, poll)

    def on_close():
        # [실기] 게임 창을 닫다가 관제센터까지 닫히는 일이 반복돼(2026-11-14) 항상 한 번 더 묻는다
        warn = '\n동작 중입니다(게임·펜 거치대).' if state['busy'] or state['external'] else ''
        if not messagebox.askyesno('관제센터 닫기', f'관제센터를 닫을까요?{warn}'):
            print(f'[{time.strftime("%H:%M:%S")}] 창 닫기 취소', file=sys.stderr, flush=True)
            return
        print(f'[{time.strftime("%H:%M:%S")}] 창 닫기 (사용자 확인)', file=sys.stderr, flush=True)
        preview.stop()
        client.stop()
        root.destroy()
    root.protocol('WM_DELETE_WINDOW', on_close)
    root.bind('<Destroy>', lambda e: print(f'[{time.strftime("%H:%M:%S")}] 창 파괴됨', file=sys.stderr, flush=True)
              if e.widget is root else None)

    camera_toggle()
    connect()
    tick()
    poll()
    root.mainloop()


TEST_CELLS = [(1, 1), (0, 0), (0, 2), (2, 2), (2, 0)]   # 가운데 → 네 모서리 칸
TEST_LIFT_MM = 5.0                                       # 허공 테스트: 실제 펜 내림 높이보다 이만큼 위


def tic_air_test(config_path, cells=TEST_CELLS, lift=TEST_LIFT_MM, return_pen=True):
    """펜 집기(쥐고 있으면 건너뜀) → 틱택토와 같은 경로(격자 검출 → RobotX → 보정)로 칸마다 X 를
    종이 lift mm 위 허공에 그림 → home → 펜 반납. 종이에 닿지 않는다. 0=성공."""
    import cv2
    import numpy as np
    from runpy import run_path
    from .camera_select import ReconnectingCamera
    from .paper_maze import perspective_board
    cfg = yaml.safe_load(open(config_path, encoding='utf8'))
    pen = lambda action: subprocess.run([sys.executable, '-m', 'robot_games.pen_holder', action, '--execute-motion', '--yes',
                                         '--config', str(config_path)], cwd=str(ROOT)).returncode
    rc = pen('pick')
    if rc not in (0, 3):                               # 3: 이미 쥐고 있음
        print(f'[TEST] 펜 집기 실패 (종료코드 {rc}) → 테스트 중단', flush=True)
        return rc
    tic = run_path(str(ROOT / 'robot_games/tictactoe/tictactoe.py'), run_name='robot_games._tic_test')
    cam = cfg.get('camera') or {}
    cap = ReconnectingCamera(int(cam.get('index', 0)))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam.get('width', 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam.get('height', 720)))
    found = []
    try:
        for i in range(40):
            ok, f = cap.read()
            if ok and i >= 8:
                c = tic['grid_corners'](f)
                if c is not None:
                    found.append((c, f))
    finally:
        cap.release()
    print(f'[TEST] 격자 검출 {len(found)}/32장', flush=True)
    if len(found) < 10:
        print('[TEST] 격자가 잘 안 보여 로봇을 움직이지 않음 (빈 3x3 격자를 마커 사이에)', flush=True)
        if return_pen:
            pen('return')
        return 2
    corners = np.median(np.array([c for c, _ in found]), 0).astype(np.float32)
    _, matrix = perspective_board(found[-1][1], corners)
    from .motion import GuardedDoosan
    robot = GuardedDoosan(cfg['robot'], need_celebration=False)
    result = 0
    try:
        drawer = tic['RobotX'](robot, cfg, found[-1][1], matrix)
        rows = []
        for row, col in cells:
            r = drawer.rows(row, col)
            for p in r:
                if p['pen'] == 'down':
                    p['z'] += lift
            down = [p for p in r if p['pen'] == 'down']
            cx, cy = np.mean([p['x'] for p in down]), np.mean([p['y'] for p in down])
            print(f'[TEST] 칸 ({row + 1},{col + 1}) X 중심 ({cx:.1f}, {cy:.1f})  펜 높이 {down[0]["z"]:.1f} (실제보다 {lift:g}mm 위)', flush=True)
            rows += r
        robot.home()
        robot.run_rows(rows)
        robot.home()
        print('[TEST] 허공 X 완료 → 펜 끝이 각 칸 대각선을 지나갔는지 확인하세요', flush=True)
    except BaseException as e:
        print(f'[TEST] 중단: {type(e).__name__}: {e}', flush=True)
        result = 1
        if not robot.stopped():
            try:
                p = robot.current_posx()
                up = float(drawer.cal['pen_up_z_base_mm']) if 'drawer' in locals() else p[2]
                if p[2] < up:
                    robot.move_line([p[0], p[1], up, *p[3:6]], 40)
                robot.home()
            except Exception as e2:
                print(f'[TEST] home 실패: {e2}', flush=True)
    finally:
        robot.close()
    if return_pen and result == 0:
        pen('return')
    return result


def maze_air_test(config_path, lift=TEST_LIFT_MM, return_pen=True):
    """펜 집기 → 미로 촬영·풀이(경로 미리보기 승인) → 펜 높이만 lift mm 올린 보정 사본으로 CSV → 실기 실행 → 펜 반납.
    미로 파이프라인(paper_maze → maze_csv → maze_executor)과 검증을 그대로 쓰고, 종이에는 닿지 않는다. 0=성공."""
    from . import paper_maze
    from .maze_csv import CALIB, OUT, CalibrationError, convert
    from .maze_executor import ExecutionRefused, execute
    pen = lambda action: subprocess.run([sys.executable, '-m', 'robot_games.pen_holder', action, '--execute-motion', '--yes',
                                         '--config', str(config_path)], cwd=str(ROOT)).returncode
    cal = json.loads(Path(CALIB['real']).read_text(encoding='utf-8'))
    if cal.get('measured_and_verified') is not True:
        print('[TEST] 그리기 보정이 measured_and_verified=true 가 아님 → 중단', flush=True)
        return 2
    cal['pen_contact_plane'][2] += lift                # 펜 높이만 올린다(XY 는 그대로)
    for k in ('pen_contact_z_base_mm', 'pen_up_z_base_mm'):
        cal[k] += lift
    cal['origin_base_mm'][2] += lift
    cal['_note'] = f'미로 허공 테스트용 사본: {CALIB["real"].name} 에서 펜 높이 +{lift:g}mm. 실제 그리기에 쓰지 말 것'
    test_cal = OUT / 'maze_air_test_calibration.json'
    OUT.mkdir(exist_ok=True)
    test_cal.write_text(json.dumps(cal, indent=2, ensure_ascii=False), encoding='utf-8')
    rc = pen('pick')
    if rc not in (0, 3):
        print(f'[TEST] 펜 집기 실패 (종료코드 {rc}) → 테스트 중단', flush=True)
        return rc
    result = 0
    try:
        print("=== 미로 촬영: 'Maze camera' 창에서 SPACE 로 촬영(Q/ESC 는 취소). 풀이 결과는 5초 표시 후 자동 진행 ===", flush=True)
        plan = paper_maze.main(['--source', 'camera', '--robot-mode', 'mock', '--approve'])
        meta = convert(plan, 'real', calib_path=test_cal, out_csv=OUT / 'maze_air_test.csv')
        print(f'[TEST] CSV {meta["rows"]}행 (펜 높이 +{lift:g}mm) → 실행', flush=True)
        n = execute(meta['csv'], 'real', True, True, str(config_path))
        print(f'[TEST] 미로 허공 테스트 완료 ({n}행) → 펜 끝이 미로 통로 가운데를 지나갔는지 확인하세요', flush=True)
    except (SystemExit, KeyboardInterrupt) as e:
        print(f'[TEST] 취소됨 ({type(e).__name__})', flush=True)
        result = 1
    except (CalibrationError, ExecutionRefused, PermissionError, ValueError, RuntimeError) as e:
        print(f'[TEST] 중단: {type(e).__name__}: {e}', flush=True)
        result = 1
    if return_pen:
        pen('return')
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description='로봇 컨트롤 센터 (조그·그리퍼·펜 거치대·카메라·게임)')
    p.add_argument('--config', default=str(DEFAULT_CONFIG))
    p.add_argument('--worker', action='store_true', help='(내부용) 로봇 연결 프로세스')
    p.add_argument('--tic-air-test', action='store_true', help='펜 집기 → 틱택토 칸 5곳에 허공 X → 펜 반납 (종이에 안 닿음)')
    p.add_argument('--maze-air-test', action='store_true', help='펜 집기 → 미로 촬영·풀이 → 종이 5mm 위로 미로 경로 → 펜 반납')
    p.add_argument('--keep-pen', action='store_true', help='테스트 뒤 펜을 반납하지 않음')
    a = p.parse_args(argv)
    if a.worker:
        return worker_main(a.config)
    if a.tic_air_test:
        return tic_air_test(a.config, return_pen=not a.keep_pen)
    if a.maze_air_test:
        return maze_air_test(a.config, return_pen=not a.keep_pen)
    run_ui(a.config)
    return 0


if __name__ == '__main__':
    sys.exit(main())
