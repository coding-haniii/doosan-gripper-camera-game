"""No-actuation mock, image -> calibrated robot-plane plan, and guarded ROS driver."""
import numpy as np

class MockRobot:
    def __init__(self): self.events=[]
    def draw(self, path, label='path'): self.events.append(('draw',label,len(path))); print(f'[MOCK] drawing {label}: {len(path)} points')
    def run_rows(self, rows):
        """CSV 행(dict) 실행 흉내: 실제 구동 없음."""
        down=sum(1 for r in rows if r['pen']=='down')
        self.events.append(('run_rows',len(rows),down))
        print(f'[MOCK] CSV {len(rows)}행 실행 (pen down {down}행). 실제 로봇은 움직이지 않음')
    def home(self): self.events.append(('home',)); print('[MOCK] home')
    def celebrate(self): self.events.append(('celebrate',)); print('[MOCK] straight-arm celebration (symbolic)')

def image_path_to_base(path, shape, config):
    """Calibrated XY plane: origin at image top-left, axes span image width and height.
    Requires onsite measured orthogonal/nonorthogonal vectors and absolute Z.
    """
    required=['board_origin_base_mm','board_x_axis_base_mm','board_y_axis_base_mm',
              'drawing_z_base_mm','safe_z_base_mm','tool_rpy_deg']
    if any(config.get(k) is None for k in required): raise ValueError('Robot board/TCP calibration incomplete')
    h,w=shape[:2]
    origin=np.asarray(config['board_origin_base_mm'],float)
    xaxis=np.asarray(config['board_x_axis_base_mm'],float)
    yaxis=np.asarray(config['board_y_axis_base_mm'],float)
    if origin.size!=3 or xaxis.size!=3 or yaxis.size!=3: raise ValueError('Axes/origin must be 3D')
    if np.linalg.norm(xaxis)>config.get('max_board_side_mm',300) or np.linalg.norm(yaxis)>config.get('max_board_side_mm',300):
        raise ValueError('Board too large for configured limit')
    if len(path)>config.get('max_path_points',1500): raise ValueError('Path point count exceeds limit')
    z=float(config['drawing_z_base_mm']); safe=float(config['safe_z_base_mm'])
    if safe <= z: raise ValueError('Safe Z must be greater than drawing Z for horizontal top-down setup')
    orientation=list(map(float,config['tool_rpy_deg']))
    output=[]
    for px,py in path:
        p=origin+(float(px)/max(1,w-1))*xaxis+(float(py)/max(1,h-1))*yaxis
        output.append([float(p[0]),float(p[1]),z,*orientation])
    return output

def _patch_dsr_msgs():
    """[실기] 로컬 doosan-robot2 의 DSR_ROBOT2.py 는 dsr_msgs2 에 없는 `SetSingularityHandlingForce` 를 참조해
    import 가 실패한다(실제 이름은 SetSingularHandlingForce). 드라이버를 고치지 않고 별칭만 건다."""
    import dsr_msgs2.srv as srv
    if not hasattr(srv,'SetSingularityHandlingForce') and hasattr(srv,'SetSingularHandlingForce'):
        srv.SetSingularityHandlingForce=srv.SetSingularHandlingForce
        if hasattr(srv,'__all__') and 'SetSingularityHandlingForce' not in srv.__all__:
            srv.__all__.append('SetSingularityHandlingForce')

POS_TOL_MM=3.0; JOINT_TOL_DEG=1.0
MAX_DRAW_VEL=60.0; MAX_DRAW_ACC=300.0   # [실기] 그리기 속도 상한 (2026-11-11 사용자 요청으로 20→60mm/s)
SPLINE_MAX_POINTS=100                  # movesx 한 번에 넣는 최대 점 수 (dsr_msgs2 pos_cnt 는 int8)
MIN_SPLINE_STEP_MM=2.0                 # 이보다 가까운 연속 점은 버린다. 1mm 미만 구간이 있으면 컨트롤러가 일정 속도를
                                       # 포기하고(알람 3213 CONSTANT SPEED IS NOT AVAILABLE) 크게 느려진다(2026-11-11 실측 44초 → 약 9초 기대)
STOP_STATES={3,5,6,9,10}   # SAFE_OFF, SAFE_STOP, EMERGENCY_STOP, SAFE_STOP2, SAFE_OFF2 (dsr_msgs2 GetRobotState)

class GuardedDoosan:
    """Only runs motion when armed AND calibration confirmed; requires sourced Jazzy DSR_ROBOT2."""
    def __init__(self, config, need_celebration=True, for_calibration=False):
        # [실기] for_calibration=True: 보정·교시 도구(calibrate.py)용. 보정 전이라 calibration_confirmed 와 home 교시는
        # 요구하지 않는다(armed 는 항상 필요).
        if not config.get('armed') or not (for_calibration or config.get('calibration_confirmed')):
            raise PermissionError('Robot not armed and calibrated')
        if not for_calibration and (config.get('home_joints_deg') is None or (need_celebration and config.get('celebration_joints_deg') is None)):
            raise ValueError('Home/celebration must be taught and validated on real arm')
        self.cfg=config
        import rclpy
        import DR_init
        self.rclpy=rclpy
        rclpy.init()
        # 클래스 안의 `DR_init.__dsr__id` 는 이름 맹글링(_GuardedDoosan__dsr__id)되어 DSR_ROBOT2 가 못 읽음 → setattr 사용
        ns=config.get('namespace','dsr01')
        setattr(DR_init,'__dsr__id',ns)
        setattr(DR_init,'__dsr__model',config.get('model','e0509'))
        self.node=rclpy.create_node('games_operator',namespace=ns)
        setattr(DR_init,'__dsr__node',self.node)
        from .dsr_preflight import preflight
        try:
            system=preflight(self.node,ns)  # 컨트롤러가 안 보이면 멈추지 않고 원인을 안내
            if system!=0:  # [실기] 0=REAL, 1=VIRTUAL(에뮬레이터): 실기 클래스는 실제 컨트롤러에만 연결
                raise PermissionError(f'Controller is not REAL (robot_system={system})')
            _patch_dsr_msgs()
            from DSR_ROBOT2 import movej,movel,movesx,posj,posx,get_current_posx,get_current_posj,get_robot_state,change_operation_speed,DR_MVS_VEL_CONST,fkin,DR_BASE,DR_WORLD
            # [실기] 컨트롤러 전체 속도 배율. DART 에서 10% 로 남아 있으면 모든 동작이 10배 느려진다(2026-11-10 실측)
            change_operation_speed(int(config.get('operation_speed_pct',100)))
        except BaseException:
            self.node.destroy_node(); rclpy.shutdown(); raise
        self.movej,self.movel,self.posj,self.posx=movej,movel,posj,posx
        self.movesx,self.vel_const=movesx,DR_MVS_VEL_CONST
        self._posx_now,self._posj_now,self._state=get_current_posx,get_current_posj,get_robot_state
        self._fkin,self.DR_BASE,self.DR_WORLD=fkin,DR_BASE,DR_WORLD
    # [실기] 보정·교시 도구(calibrate.py, jog.py)가 쓰는 공개 메서드
    def current_posx(self, ref=None):
        p=self._posx_now() if ref is None else self._posx_now(ref)
        return [float(v) for v in (p[0] if isinstance(p,tuple) else p)]
    def current_posj(self): return [float(v) for v in self._posj_now()]
    def fkin(self, q):
        f=self._fkin(self.posj(*map(float,q))); return [float(v) for v in (f[0] if isinstance(f,tuple) else f)]
    def move_line(self, pose, vel=None, acc=None):
        self._line(list(map(float,pose)),float(vel or self.cfg['speed_mm_s']),float(acc or self.cfg['accel_mm_s2']))
    def move_joint(self, q): self._joint(list(map(float,q)))
    def state(self):
        try: return self._state()
        except Exception: return -1
    def stopped(self):
        """보호정지·비상정지·서보 꺼짐·상태 읽기 실패면 True (이때는 home 등 자동 이동을 하지 않는다)."""
        s=self.state(); return not isinstance(s,int) or s<0 or s in STOP_STATES
    def _line(self, pose, vel, acc):
        """[실기] movel + 바닥 하한 + 도착 확인. NOT REACHABLE 로 거부돼도 서비스는 성공으로 끝날 수 있어 실제 위치로 확인한다."""
        floor=self.cfg.get('min_pen_z_base_mm')
        if floor is not None and float(pose[2])<float(floor):
            raise ValueError(f'Z {float(pose[2]):.1f}mm below min_pen_z_base_mm {floor}')
        self.movel(self.posx(*pose),vel=vel,acc=acc)
        p=self._posx_now(); now=[float(v) for v in (p[0] if isinstance(p,tuple) else p)]
        miss=sum((a-float(b))**2 for a,b in zip(now[:3],pose[:3]))**.5
        if miss>POS_TOL_MM: raise RuntimeError(f'Did not reach ({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f}): {miss:.0f}mm off (NOT REACHABLE?)')
    def _joint(self, vals):
        if len(vals)!=6: raise ValueError('Six joint angles required')
        self.movej(self.posj(*vals),vel=self.cfg['joint_speed_deg_s'],acc=self.cfg['joint_accel_deg_s2'])
        now=[float(v) for v in self._posj_now()]
        if max(abs(a-float(b)) for a,b in zip(now,vals))>JOINT_TOL_DEG: raise RuntimeError('Did not reach joint target')
    def home(self): self._joint(self.cfg['home_joints_deg'])
    def celebrate(self): self._joint(self.cfg['celebration_joints_deg'])
    def draw(self, path, label='path'):
        """Only path positions in BASE frame with verified TCP/tool settings; low speed.
        No ROS startup or emergency stop logic is bypassed. Execute only with operator approval.
        """
        if not path: return
        safe=float(self.cfg['safe_z_base_mm']); vel=float(self.cfg['speed_mm_s']); acc=float(self.cfg['accel_mm_s2'])
        if not (0<vel<=MAX_DRAW_VEL and 0<acc<=MAX_DRAW_ACC): raise ValueError('Restrict speed/acceleration during commissioning')
        first=path[0].copy(); first[2]=safe
        self._line(first,vel,acc)
        for point in path:
            self._line(point,vel,acc)
        last=path[-1].copy(); last[2]=safe
        self._line(last,vel,acc)
    def run_rows(self, rows):
        """maze_csv 가 만든 '실측 기준' CSV 행을 순서대로 movel. 행마다 x,y,z,rx,ry,rz 를 사용(펜 올림/내림 Z 는 CSV 에 이미 반영).
        저속 제한은 draw() 와 동일. 오류가 나면 즉시 중단(예외) — 호출자가 home/정지 처리."""
        vel=float(self.cfg['speed_mm_s']); acc=float(self.cfg['accel_mm_s2'])
        if not (0<vel<=MAX_DRAW_VEL and 0<acc<=MAX_DRAW_ACC): raise ValueError('Restrict speed/acceleration during commissioning')
        # [실기] 점마다 movel 로 멈췄다 가면 236점 미로에 2분 반이 걸렸다(2026-11-11). 펜을 내린 연속 구간은
        # movesx(스플라인, 일정 속도)로 한 번에 지나가고, 펜 올림/내림 이동만 movel 로 한다.
        pose=lambda r: [float(r['x']),float(r['y']),float(r['z']),float(r['rx']),float(r['ry']),float(r['rz'])]
        i=0
        while i<len(rows):
            if rows[i]['pen']!='down':
                self._line(pose(rows[i]),vel,acc); i+=1; continue
            j=i
            while j<len(rows) and rows[j]['pen']=='down': j+=1
            seg=[pose(r) for r in rows[i:j]]
            self._line(seg[0],vel,acc)                       # 펜 내리기 (시작점)
            pts=[seg[0]]
            for q in seg[1:]:
                if sum((a-b)**2 for a,b in zip(q[:3],pts[-1][:3]))**.5>=MIN_SPLINE_STEP_MM: pts.append(q)
            for k in range(1,len(pts),SPLINE_MAX_POINTS-1):  # 앞 묶음의 끝점에서 이어서 시작
                self._spline(pts[k:k+SPLINE_MAX_POINTS-1],vel,acc)
            i=j
    def _spline(self, chunk, vel, acc):
        """[실기] movesx + 바닥 하한 + 도착 확인 (_line 과 같은 검사)."""
        if not chunk: return
        floor=self.cfg.get('min_pen_z_base_mm')
        if floor is not None and min(float(p[2]) for p in chunk)<float(floor):
            raise ValueError(f'spline Z below min_pen_z_base_mm {floor}')
        if len(chunk)==1:
            self._line(chunk[0],vel,acc); return
        self.movesx([self.posx(*p) for p in chunk],vel=vel,acc=acc,vel_opt=self.vel_const)
        p=self._posx_now(); now=[float(v) for v in (p[0] if isinstance(p,tuple) else p)]
        end=chunk[-1]; miss=sum((a-float(b))**2 for a,b in zip(now[:3],end[:3]))**.5
        if miss>POS_TOL_MM: raise RuntimeError(f'Spline did not reach ({end[0]:.0f},{end[1]:.0f},{end[2]:.0f}): {miss:.0f}mm off')
    def close(self): self.node.destroy_node(); self.rclpy.shutdown()
