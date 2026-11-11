"""로봇 계층: 구동 없는 MockRobot, 두산 연결(_DsrSession), 실기용 GuardedDoosan.

모든 로봇 객체가 같은 인터페이스를 가진다:
  home() / celebrate() / move_joint(q) / move_line(pose) / run_rows(rows) / draw(path) /
  set_tcp(name) / state() / stop() / close()
이동은 SafetyMonitor 를 거치므로 보호정지·비상정지 상태가 되면 RobotStopped 가 올라온다.
에뮬레이터용 EmulatorDoosan 은 emulator.py 에 있다.
"""
import time

import numpy as np

from .safety import RobotStopped, SafetyMonitor, STATE_STANDBY

ROBOT_SYSTEM_REAL = 0
ROBOT_SYSTEM_VIRTUAL = 1
MAX_LINEAR_VEL = 20.0      # mm/s, 커미셔닝 기간 상한
MAX_LINEAR_ACC = 50.0      # mm/s^2
MAX_JOG_VEL = 300.0        # mm/s, 보정용 조그·공중 이동 상한 (사용자 요청으로 상향, 2026-11-10)
MAX_JOG_ACC = 1000.0       # mm/s^2
MAX_JOINT_VEL = 30.0       # deg/s
MAX_JOINT_ACC = 60.0       # deg/s^2


class MotionFailed(RuntimeError):
    """컨트롤러가 이동을 거부했거나(예: NOT REACHABLE) 목표에 도착하지 못함. 로봇은 정상 상태일 수 있다."""


POS_TOL_MM = 3.0
JOINT_TOL_DEG = 1.0


def _linear_limits(cfg):
    vel = float(cfg['speed_mm_s'])
    acc = float(cfg['accel_mm_s2'])
    if not (0 < vel <= MAX_LINEAR_VEL and 0 < acc <= MAX_LINEAR_ACC):
        raise ValueError('Restrict speed/acceleration during commissioning')
    return vel, acc


def _jog_limits(cfg):
    vel = float(cfg.get('jog_speed_mm_s', 100))
    acc = float(cfg.get('jog_accel_mm_s2', 300))
    if not (0 < vel <= MAX_JOG_VEL and 0 < acc <= MAX_JOG_ACC):
        raise ValueError(f'jog 속도 상한 {MAX_JOG_VEL}mm/s, 가속 {MAX_JOG_ACC}mm/s^2')
    return vel, acc


def _joint_limits(cfg):
    vel = float(cfg.get('joint_speed_deg_s', 10))
    acc = float(cfg.get('joint_accel_deg_s2', 20))
    if not (0 < vel <= MAX_JOINT_VEL and 0 < acc <= MAX_JOINT_ACC):
        raise ValueError('Restrict joint speed/acceleration during commissioning')
    return vel, acc


def _row_pose(r, z_offset=0.0):
    return [r['x'], r['y'], r['z'] + z_offset, r['rx'], r['ry'], r['rz']]


class MockRobot:
    """구동 없음. 이벤트를 기록하고 출력만 한다. robot_state 를 바꿔 정지 상황을 흉내 낼 수 있다(테스트용)."""

    def __init__(self, poses=None, z_offset_mm=0.0):
        self.events = []
        self.robot_state = STATE_STANDBY
        self.poses = poses or {}
        self.z_offset_mm = z_offset_mm
        self.safety = SafetyMonitor(lambda: self.robot_state, self._stop)

    def _stop(self):
        self.events.append(('stop',))
        print('[MOCK] move_stop')

    def state(self):
        return self.robot_state

    def stop(self):
        self.safety.stop()

    def move_joint(self, q, label='joint'):
        self.safety.check()
        self.events.append(('move_joint', label))

    def move_line(self, pose, label='line', jog=False):
        self.safety.check()
        self.events.append(('move_line', label))

    def goto(self, name):
        """robot_poses 에 교시된 이름 자세(예: camera_clear)로 이동. 없으면 건너뜀."""
        self.safety.check()
        self.events.append(('goto', name))
        print(f'[MOCK] goto {name}')

    def draw(self, path, label='path'):
        self.safety.check()
        self.events.append(('draw', label, len(path)))
        print(f'[MOCK] drawing {label}: {len(path)} points')

    def run_rows(self, rows):
        """CSV 행(dict) 실행 흉내: 실제 구동 없음."""
        self.safety.check()
        down = sum(1 for r in rows if r['pen'] == 'down')
        self.events.append(('run_rows', len(rows), down))
        print(f'[MOCK] CSV {len(rows)}행 실행 (pen down {down}행). 실제 로봇은 움직이지 않음')

    def set_tcp(self, name):
        self.events.append(('set_tcp', name))

    def home(self):
        self.safety.check()
        self.events.append(('home',))
        print('[MOCK] home')

    def celebrate(self):
        self.safety.check()
        self.events.append(('celebrate',))
        print('[MOCK] straight-arm celebration (symbolic)')

    def close(self):
        pass


def image_path_to_base(path, shape, config):
    """(구버전) 보드 원점/축 벡터로 픽셀 경로를 base 좌표로. 새 코드는 pen_plan 을 쓴다."""
    required = ['board_origin_base_mm', 'board_x_axis_base_mm', 'board_y_axis_base_mm',
                'drawing_z_base_mm', 'safe_z_base_mm', 'tool_rpy_deg']
    if any(config.get(k) is None for k in required):
        raise ValueError('Robot board/TCP calibration incomplete')
    h, w = shape[:2]
    origin = np.asarray(config['board_origin_base_mm'], float)
    xaxis = np.asarray(config['board_x_axis_base_mm'], float)
    yaxis = np.asarray(config['board_y_axis_base_mm'], float)
    if origin.size != 3 or xaxis.size != 3 or yaxis.size != 3:
        raise ValueError('Axes/origin must be 3D')
    if np.linalg.norm(xaxis) > config.get('max_board_side_mm', 300) or np.linalg.norm(yaxis) > config.get('max_board_side_mm', 300):
        raise ValueError('Board too large for configured limit')
    if len(path) > config.get('max_path_points', 1500):
        raise ValueError('Path point count exceeds limit')
    z = float(config['drawing_z_base_mm'])
    safe = float(config['safe_z_base_mm'])
    if safe <= z:
        raise ValueError('Safe Z must be greater than drawing Z for horizontal top-down setup')
    orientation = list(map(float, config['tool_rpy_deg']))
    output = []
    for px, py in path:
        p = origin + (float(px) / max(1, w - 1)) * xaxis + (float(py) / max(1, h - 1)) * yaxis
        output.append([float(p[0]), float(p[1]), z, *orientation])
    return output


def _patch_dsr_msgs():
    """doosan-robot2 의 DSR_ROBOT2.py 는 `SetSingularityHandlingForce` 를 찾지만 dsr_msgs2 의 실제 서비스 이름은
    `SetSingularHandlingForce` 라 import 단계에서 NameError 가 난다(드라이버 쪽 버그). ROS 워크스페이스를
    고치지 않고 별칭만 걸어 둔다. 드라이버가 고쳐지면 아무 일도 하지 않는다."""
    import dsr_msgs2.srv as srv
    aliases = {'SetSingularityHandlingForce': 'SetSingularHandlingForce'}
    for want, have in aliases.items():
        if not hasattr(srv, want) and hasattr(srv, have):
            setattr(srv, want, getattr(srv, have))
            if hasattr(srv, '__all__') and want not in srv.__all__:
                srv.__all__.append(want)


class _DsrSession:
    """ROS 2 노드 + DSR_ROBOT2 연결. 프로세스당 하나만 만든다(DSR_ROBOT2 가 import 시점의 노드에 묶임).

    expect_system: ROBOT_SYSTEM_REAL 또는 ROBOT_SYSTEM_VIRTUAL. 컨트롤러가 다른 종류면 PermissionError.
    """

    def __init__(self, cfg, expect_system, timeout_s=5.0):
        import rclpy
        import DR_init
        from ..dsr_preflight import preflight
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        ns = cfg.get('namespace', 'dsr01')
        # 클래스 안에서 `DR_init.__dsr__id = ...` 로 쓰면 이름 맹글링되어 DSR_ROBOT2 가 못 읽음 → setattr
        setattr(DR_init, '__dsr__id', ns)
        setattr(DR_init, '__dsr__model', cfg.get('model', 'e0509'))
        self.node = rclpy.create_node('games_operator', namespace=ns)
        setattr(DR_init, '__dsr__node', self.node)
        try:
            system = preflight(self.node, ns, timeout_s=timeout_s)
            self.system = system
            if system != expect_system:
                kind = {ROBOT_SYSTEM_REAL: 'REAL', ROBOT_SYSTEM_VIRTUAL: 'VIRTUAL'}
                raise PermissionError(f'컨트롤러가 {kind.get(system, system)} 입니다. '
                                      f'이 모드는 {kind[expect_system]} 컨트롤러에만 연결합니다.')
            _patch_dsr_msgs()
            import DSR_ROBOT2 as dr
            from dsr_msgs2.srv import MoveStop
            self.dr = dr
            self._move_stop = self.node.create_client(MoveStop, 'motion/move_stop')
            self._MoveStop = MoveStop
            self._set_operation_speed(cfg.get('operation_speed_pct', 100))
        except BaseException:
            self.close()
            raise

    def _set_operation_speed(self, pct):
        """컨트롤러 전체 속도 배율(%). 명령한 속도에 이 배율이 곱해진다. DART 에서 10% 로 둔 값이 남아 있으면
        모든 동작이 10배 느려진다(2026-11-10 실측). 실제 속도 상한은 이 코드의 _linear/_jog/_joint_limits 가 정한다."""
        if pct is None:
            return
        pct = int(pct)
        if not 1 <= pct <= 100:
            raise ValueError('robot.operation_speed_pct 는 1~100')
        if self.dr.change_operation_speed(pct) != 0:
            print(f'[ROBOT] 속도 배율 {pct}% 설정 실패 (컨트롤러 값이 그대로 적용됨)')

    def robot_state(self):
        try:
            s = self.dr.get_robot_state()
        except Exception:
            return -1
        return s if isinstance(s, int) else -1

    def move_stop(self):
        """Quick stop (DR_QSTOP, Stop Category 2). 실패해도 예외를 내지 않는다(이미 정지 중일 수 있음)."""
        try:
            req = self._MoveStop.Request()
            req.stop_mode = int(getattr(self.dr, 'DR_QSTOP', 1))
            fut = self._move_stop.call_async(req)
            self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        except Exception as e:
            print(f'[STOP] move_stop 호출 실패: {e}')

    def close(self):
        try:
            self.node.destroy_node()
        finally:
            if self.rclpy.ok():
                self.rclpy.shutdown()


class _DoosanRobot:
    """GuardedDoosan / EmulatorDoosan 공통. 비동기 이동을 보내고 끝날 때까지 상태를 감시한다."""
    POLL_S = 0.05
    START_GRACE_S = 0.3

    def __init__(self, session, cfg, poses=None, z_offset_mm=0.0):
        self.s = session
        self.dr = session.dr
        self.cfg = cfg
        self.poses = poses or {}
        self.z_offset_mm = float(z_offset_mm)
        self.safety = SafetyMonitor(session.robot_state, session.move_stop)
        # 펜을 미리 물려 둔 구성: 어느 진입점이든 펜 끝 TCP 로 움직이게 한다 (에뮬레이터에는 등록된 TCP 가 없음)
        if session.system == ROBOT_SYSTEM_REAL and cfg.get('pen_preloaded') and cfg.get('pen_tcp_name'):
            self.set_tcp(cfg['pen_tcp_name'])

    # ---- 저수준 ----
    def _wait_motion(self):
        t0 = time.time()
        try:
            while True:
                self.safety.check()
                status = self.dr.check_motion()
                if status == self.dr.DR_STATE_IDLE and time.time() - t0 > self.START_GRACE_S:
                    return
                time.sleep(self.POLL_S)
        except (RobotStopped, KeyboardInterrupt):
            self.safety.stop()
            raise

    def state(self):
        return self.s.robot_state()

    def stop(self):
        self.safety.stop()

    def move_joint(self, q, label='joint'):
        if q is None or len(q) != 6:
            raise ValueError(f'{label}: 관절값 6개가 필요합니다 (교시되지 않음?)')
        vel, acc = _joint_limits(self.cfg)
        self.safety.check()
        target = [float(v) for v in q]
        if self.dr.amovej(self.dr.posj(*target), vel=vel, acc=acc) not in (0, None):
            raise MotionFailed(f'{label}: 컨트롤러가 관절 이동을 거부함')
        self._wait_motion()
        now = [float(v) for v in self.dr.get_current_posj()]
        if max(abs(a - b) for a, b in zip(now, target)) > JOINT_TOL_DEG:
            raise MotionFailed(f'{label}: 목표 관절 자세에 도착하지 못함 (도달 불가/관절 한계?)')

    def move_line(self, pose, label='line', jog=False):
        """jog=True 는 보정용 키보드 조그: 그리기보다 빠른 jog_speed_mm_s(상한 60) 를 쓴다."""
        vel, acc = _jog_limits(self.cfg) if jog else _linear_limits(self.cfg)
        floor = self.cfg.get('min_pen_z_base_mm')
        if floor is not None and float(pose[2]) < float(floor):
            raise ValueError(f'{label}: Z {float(pose[2]):.1f}mm 가 하한 {float(floor):g}mm(robot.min_pen_z_base_mm)보다 낮음')
        self.safety.check()
        target = [float(v) for v in pose]
        if self.dr.amovel(self.dr.posx(*target), vel=vel, acc=acc) not in (0, None):
            raise MotionFailed(f'{label}: 컨트롤러가 이동을 거부함')
        self._wait_motion()
        # 컨트롤러가 NOT REACHABLE 로 거부해도 서비스는 성공으로 끝날 수 있다 → 실제 위치로 확인
        p = self.dr.get_current_posx()
        now = [float(v) for v in (p[0] if isinstance(p, tuple) else p)]
        miss = sum((a - b) ** 2 for a, b in zip(now[:3], target[:3])) ** .5
        if miss > POS_TOL_MM:
            raise MotionFailed(f'{label}: 목표 ({target[0]:.0f},{target[1]:.0f},{target[2]:.0f}) 에 '
                               f'도착하지 못함 ({miss:.0f}mm 차이, 도달 불가?)')

    # ---- 고수준 ----
    def _pose(self, name):
        if name in self.poses and self.poses[name] is not None:
            return self.poses[name]
        return self.cfg.get(f'{name}_joints_deg')

    def home(self):
        self.move_joint(self._pose('home'), 'home')

    def celebrate(self):
        q = self._pose('celebration')
        if q is None:
            print('[ROBOT] celebration 자세가 교시되지 않아 생략')
            return
        self.move_joint(q, 'celebration')

    def goto(self, name):
        q = self._pose(name)
        if q is None:
            print(f'[ROBOT] {name} 자세가 교시되지 않아 생략')
            return
        self.move_joint(q, name)

    def set_tcp(self, name):
        if name:
            self.dr.set_tcp(name)

    def draw(self, path, label='path'):
        """(구버전) base 좌표 [x,y,z,rx,ry,rz] 목록을 safe Z 로 들어갔다 나오며 따라감."""
        if not path:
            return
        safe = float(self.cfg['safe_z_base_mm'])
        first = list(path[0])
        first[2] = safe
        last = list(path[-1])
        last[2] = safe
        for pose in [first, *path, last]:
            pose = list(pose)
            pose[2] += self.z_offset_mm
            self.move_line(pose, label)

    def run_rows(self, rows):
        """pen_plan/maze_csv 가 만든 행을 순서대로 movel. 펜 Z 는 행에 이미 반영되어 있다.
        z_offset_mm(>0)은 리허설용으로 모든 Z 를 올린다."""
        _linear_limits(self.cfg)
        for r in rows:
            self.move_line(_row_pose(r, self.z_offset_mm), r.get('action', 'row'))

    def close(self):
        self.s.close()


class GuardedDoosan(_DoosanRobot):
    """실기. armed + calibration_confirmed + home 교시 + REAL 컨트롤러일 때만 생성된다."""

    def __init__(self, config, need_celebration=True, poses=None, z_offset_mm=0.0):
        if not (config.get('armed') and config.get('calibration_confirmed')):
            raise PermissionError('Robot not armed and calibrated')
        poses = poses or {}
        home = poses.get('home') or config.get('home_joints_deg')
        celebration = poses.get('celebration') or config.get('celebration_joints_deg')
        if home is None or (need_celebration and celebration is None):
            raise ValueError('Home/celebration must be taught and validated on real arm')
        session = _DsrSession(config, ROBOT_SYSTEM_REAL)
        super().__init__(session, config, poses, z_offset_mm)
