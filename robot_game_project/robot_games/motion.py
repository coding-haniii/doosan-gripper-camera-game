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

class GuardedDoosan:
    """Only runs motion when armed AND calibration confirmed; requires sourced Jazzy DSR_ROBOT2."""
    def __init__(self, config, need_celebration=True):
        if not (config.get('armed') and config.get('calibration_confirmed')):
            raise PermissionError('Robot not armed and calibrated')
        if config.get('home_joints_deg') is None or (need_celebration and config.get('celebration_joints_deg') is None):
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
            preflight(self.node,ns)  # 컨트롤러가 안 보이면 멈추지 않고 원인을 안내
        except BaseException:
            self.node.destroy_node(); rclpy.shutdown(); raise
        from DSR_ROBOT2 import movej,movel,posj,posx
        self.movej,self.movel,self.posj,self.posx=movej,movel,posj,posx
    def _joint(self, vals):
        if len(vals)!=6: raise ValueError('Six joint angles required')
        self.movej(self.posj(*vals),vel=self.cfg['joint_speed_deg_s'],acc=self.cfg['joint_accel_deg_s2'])
    def home(self): self._joint(self.cfg['home_joints_deg'])
    def celebrate(self): self._joint(self.cfg['celebration_joints_deg'])
    def draw(self, path, label='path'):
        """Only path positions in BASE frame with verified TCP/tool settings; low speed.
        No ROS startup or emergency stop logic is bypassed. Execute only with operator approval.
        """
        if not path: return
        safe=float(self.cfg['safe_z_base_mm']); vel=float(self.cfg['speed_mm_s']); acc=float(self.cfg['accel_mm_s2'])
        if not (0<vel<=20 and 0<acc<=50): raise ValueError('Restrict speed/acceleration during commissioning')
        first=path[0].copy(); first[2]=safe
        self.movel(self.posx(*first),vel=vel,acc=acc)
        for point in path:
            self.movel(self.posx(*point),vel=vel,acc=acc)
        last=path[-1].copy(); last[2]=safe
        self.movel(self.posx(*last),vel=vel,acc=acc)
    def run_rows(self, rows):
        """maze_csv 가 만든 '실측 기준' CSV 행을 순서대로 movel. 행마다 x,y,z,rx,ry,rz 를 사용(펜 올림/내림 Z 는 CSV 에 이미 반영).
        저속 제한은 draw() 와 동일. 오류가 나면 즉시 중단(예외) — 호출자가 home/정지 처리."""
        vel=float(self.cfg['speed_mm_s']); acc=float(self.cfg['accel_mm_s2'])
        if not (0<vel<=20 and 0<acc<=50): raise ValueError('Restrict speed/acceleration during commissioning')
        for r in rows:
            pose=[r['x'],r['y'],r['z'],r['rx'],r['ry'],r['rz']]
            self.movel(self.posx(*pose),vel=vel,acc=acc)
    def close(self): self.node.destroy_node(); self.rclpy.shutdown()
