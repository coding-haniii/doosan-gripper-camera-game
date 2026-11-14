"""Optional separate RS-485 USB adapter control; no motion by default."""
class RobotisGripper:
    MODEL=35074
    def __init__(self, config):
        if not config.get('enabled'): raise PermissionError('Gripper disabled')
        if config.get('open_position') is None or config.get('closed_position') is None:
            raise ValueError('Teach open/closed positions on real gripper')
        from dynamixel_sdk import PortHandler,PacketHandler,COMM_SUCCESS
        self.COMM_SUCCESS=COMM_SUCCESS; self.cfg=config
        self.port=PortHandler(config['port']); self.packet=PacketHandler(2.0)
        if not self.port.openPort() or not self.port.setBaudRate(config['baud']): raise ConnectionError('RS-485 adapter connection failed')
        model,comm,err=self.packet.read2ByteTxRx(self.port,config['id'],0)
        if comm!=COMM_SUCCESS or err or model!=self.MODEL:
            self.port.closePort();raise RuntimeError(f'Unexpected gripper ID/model/read: {model,comm,err}')
    def _write(self,addr,val,width):
        func=self.packet.write1ByteTxRx if width==1 else self.packet.write4ByteTxRx
        comm,err=func(self.port,self.cfg['id'],addr,val)
        if comm!=self.COMM_SUCCESS or err: raise RuntimeError(f'Gripper write failed: {comm}/{err}')
    def torque(self,on): self._write(512,int(on),1)
    def move(self,open_=True):
        self._write(564,int(self.cfg['open_position'] if open_ else self.cfg['closed_position']),4)
    def close(self):
        self.torque(False);self.port.closePort()

# ==================================================================================================
# [실기] RH-P12-RN(A) 두산 플랜지(툴) 시리얼 제어 — 2026-11-10 실기에서 Modbus RTU 로 닫기(620) 동작 확인.
# 위 RobotisGripper 는 PC USB RS-485 어댑터용이고, 이 셀의 그리퍼는 로봇 손목 플랜지에 연결되어 있어
# drl_script_run 으로 DRL 의 flange_serial_* 를 실행한다. 설정은 default.yaml 의 gripper: (protocol, registers 등).
# 사용: robot_games.calibrate gripper --goal 0|620 / --torque on|off / --read
# ==================================================================================================
import socket
import struct
import time

# ---------------------------------------------------------------- Dynamixel Protocol 2.0
_DXL_HEADER = bytes([0xFF, 0xFF, 0xFD, 0x00])
DXL_READ = 0x02
DXL_WRITE = 0x03
DXL_STATUS = 0x55


def _crc_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return table


_DXL_TABLE = _crc_table()


def crc16_dxl(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _DXL_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def _stuff(body: bytes) -> bytes:
    """FF FF FD 뒤에 FD 를 하나 더 넣는 바이트 스터핑(Instruction 이후 영역)."""
    out = bytearray()
    for b in body:
        out.append(b)
        if len(out) >= 3 and out[-3:] == b'\xFF\xFF\xFD':
            out.append(0xFD)
    return bytes(out)


def dxl_packet(dxl_id: int, inst: int, params: bytes = b'') -> bytes:
    body = _stuff(bytes([inst]) + params)
    pkt = _DXL_HEADER + bytes([dxl_id]) + struct.pack('<H', len(body) + 2) + body
    return pkt + struct.pack('<H', crc16_dxl(pkt))


def dxl_write(dxl_id: int, addr: int, value: int, size: int) -> bytes:
    return dxl_packet(dxl_id, DXL_WRITE, struct.pack('<H', addr) + int(value).to_bytes(size, 'little', signed=value < 0))


def dxl_read(dxl_id: int, addr: int, size: int) -> bytes:
    return dxl_packet(dxl_id, DXL_READ, struct.pack('<HH', addr, size))


def parse_dxl_status(buf: bytes, dxl_id: int) -> bytes:
    """상태 패킷에서 파라미터를 꺼낸다. CRC/ID/에러 검사 실패 시 ValueError."""
    i = buf.find(_DXL_HEADER)
    if i < 0 or len(buf) < i + 11:
        raise ValueError(f'Dynamixel 응답 없음/짧음: {buf.hex(" ")}')
    pkt = buf[i:]
    length = struct.unpack('<H', pkt[5:7])[0]
    pkt = pkt[:7 + length]
    if len(pkt) < 7 + length or crc16_dxl(pkt[:-2]) != struct.unpack('<H', pkt[-2:])[0]:
        raise ValueError('Dynamixel 응답 CRC 불일치')
    if pkt[4] != dxl_id or pkt[7] != DXL_STATUS:
        raise ValueError(f'예상치 못한 응답 (id={pkt[4]}, inst={pkt[7]:#x})')
    if pkt[8] & 0x7F:
        raise ValueError(f'그리퍼 에러 코드 {pkt[8]:#x}')
    return pkt[9:-2].replace(b'\xFF\xFF\xFD\xFD', b'\xFF\xFF\xFD')


# ---------------------------------------------------------------- Modbus RTU
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _mb(frame: bytes) -> bytes:
    return frame + struct.pack('<H', crc16_modbus(frame))


def modbus_write_single(slave: int, reg: int, value: int) -> bytes:
    return _mb(struct.pack('>BBHH', slave, 0x06, reg, value & 0xFFFF))


def modbus_write_multiple(slave: int, reg: int, values) -> bytes:
    values = list(values)
    body = b''.join(struct.pack('>H', v & 0xFFFF) for v in values)
    return _mb(struct.pack('>BBHHB', slave, 0x10, reg, len(values), len(body)) + body)


def modbus_read_holding(slave: int, reg: int, count: int) -> bytes:
    return _mb(struct.pack('>BBHH', slave, 0x03, reg, count))


def parse_modbus_read(buf: bytes, slave: int) -> list:
    """버퍼 안에서 CRC 가 맞는 읽기(0x03) 응답을 찾는다. 앞선 쓰기 명령의 응답이 같이 읽혀 들어와도
    (DART 로그의 "CRC Check Fail" 원인) 건너뛴다."""
    for i in range(max(0, len(buf) - 4)):
        if buf[i] != slave:
            continue
        if buf[i + 1] == 0x83 and crc16_modbus(buf[i:i + 3]) == struct.unpack('<H', buf[i + 3:i + 5])[0]:
            raise ValueError(f'Modbus 예외 코드 {buf[i + 2]}')
        if buf[i + 1] != 0x03:
            continue
        n = buf[i + 2]
        frame = buf[i:i + 3 + n + 2]
        if n and n % 2 == 0 and len(frame) == 3 + n + 2 \
                and crc16_modbus(frame[:-2]) == struct.unpack('<H', frame[-2:])[0]:
            return [struct.unpack('>H', frame[3 + 2 * k:5 + 2 * k])[0] for k in range(n // 2)]
    raise ValueError(f'Modbus 읽기 응답을 찾지 못함: {buf.hex(" ")}')


# ---------------------------------------------------------------- 프로토콜 → 바이트
class _Protocol:
    def __init__(self, cfg):
        self.cfg = cfg
        self.kind = cfg.get('protocol', 'dynamixel2')
        self.id = int(cfg.get('id', 1))
        reg = dict(DEFAULT_REGISTERS[self.kind])
        reg.update(cfg.get('registers') or {})
        missing = [k for k in ('torque_enable', 'goal_position', 'present_position') if reg.get(k) is None]
        if missing:
            raise ValueError(f'gripper.registers 에 {missing} 주소를 매뉴얼에서 확인해 넣으세요 (protocol={self.kind})')
        self.reg = reg

    def torque(self, on: bool) -> bytes:
        if self.kind == 'dynamixel2':
            return dxl_write(self.id, self.reg['torque_enable'], int(on), 1)
        return modbus_write_single(self.id, self.reg['torque_enable'], int(on))

    def goal_current(self):
        """Goal Current(파지 힘) 패킷. 설정(gripper.goal_current)이나 주소가 없으면 None."""
        value, addr = self.cfg.get('goal_current'), self.reg.get('goal_current')
        if value is None or addr is None:
            return None
        if self.kind == 'dynamixel2':
            return dxl_write(self.id, addr, int(value), 2)
        return modbus_write_single(self.id, addr, int(value))

    def profile(self):
        """Profile Acceleration / Velocity 패킷 목록(설정·주소가 있는 것만). 값이 작을수록 천천히 움직인다."""
        out = []
        for key in ('profile_acceleration', 'profile_velocity'):
            value, addr = self.cfg.get(key), self.reg.get(key)
            if value is None or addr is None:
                continue
            v = int(value)
            out.append(dxl_write(self.id, addr, v, 4) if self.kind == 'dynamixel2'
                       else modbus_write_multiple(self.id, addr, [v & 0xFFFF, (v >> 16) & 0xFFFF]))
        return out

    def goal(self, value: int) -> bytes:
        if self.kind == 'dynamixel2':
            return dxl_write(self.id, self.reg['goal_position'], value, 4)
        return modbus_write_multiple(self.id, self.reg['goal_position'], [value & 0xFFFF, (value >> 16) & 0xFFFF])

    def read_position(self) -> bytes:
        if self.kind == 'dynamixel2':
            return dxl_read(self.id, self.reg['present_position'], 4)
        return modbus_read_holding(self.id, self.reg['present_position'], 2)

    def parse_position(self, buf: bytes) -> int:
        if self.kind == 'dynamixel2':
            return int.from_bytes(parse_dxl_status(buf, self.id)[:4], 'little', signed=True)
        lo, hi = parse_modbus_read(buf, self.id)[:2]
        return lo | (hi << 16)


DEFAULT_REGISTERS = {
    # RH-P12-RN(A) 컨트롤 테이블(Dynamixel 2.0) — 기존 RobotisGripper 와 같은 주소
    'dynamixel2': {'torque_enable': 512, 'goal_current': 550, 'goal_position': 564, 'present_position': 580,
                   'profile_acceleration': 556, 'profile_velocity': 560},
    # Modbus 펌웨어는 주소가 다르다(0부터 세는 레지스터 번호 = 4xxxx - 40001). default.yaml 에 입력
    'modbus': {'torque_enable': None, 'goal_current': None, 'goal_position': None, 'present_position': None},
}


class PenNotHeld(RuntimeError):
    """닫았는데 그리퍼 위치가 펜 부품을 잡은 범위(holding_min~max) 밖."""


def _require_positions(cfg):
    if cfg.get('open_position') is None or cfg.get('closed_position') is None:
        raise ValueError('gripper.open_position / closed_position 을 실물에서 교시하세요 (robot-calibrate gripper)')



class FlangeGripper:
    """두산 플랜지 시리얼. DRL 스크립트를 실행하고 끝날 때까지 기다린다."""
    RETRIES = 3

    def __init__(self, cfg, robot_system=0):
        """GuardedDoosan 으로 컨트롤러에 연결한 뒤에 만든다(DSR_ROBOT2 가 그 노드를 쓴다). robot_system 0=REAL."""
        _require_positions(cfg)
        import DSR_ROBOT2 as dr
        self.dr = dr
        self.system = robot_system
        self.cfg = cfg
        self.proto = _Protocol(cfg)
        self.baud = int(cfg.get('baud', 57600))
        self.timeout_s = float(cfg.get('drl_timeout_s', 10))
        init = [self.proto.torque(True)]
        if self.proto.goal_current() is not None:
            init.append(self.proto.goal_current())
        init += self.proto.profile()                 # [실기] 켜질 때 기본값은 최고 속도라 잡고 놓을 때 진동 → 늦춤
        self._run(init)

    # DRL 코드 생성 --------------------------------------------------------
    def _drl(self, packets, read_back=False, host=None, port=None):
        lines = [f'flange_serial_open(baudrate={self.baud}, bytesize=DR_EIGHTBITS, '
                 f'parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)',
                 # [실기] 열자마자 쓰면 첫 패킷이 버려진다(2026-11-11 실측: 응답 없음 → 0.1초 뒤 쓰면 매번 응답)
                 'res0, rx0 = flange_serial_read(0.1)']
        # [실기] 그래도 가끔 패킷 하나가 응답 없이 사라진다(2026-11-11 열기 명령 유실).
        # 응답이 올 때까지 같은 패킷을 최대 RETRIES 번 보낸다. 토크·전류·목표 위치 쓰기와 읽기는 여러 번 보내도 결과가 같다.
        for k, p in enumerate(packets):
            last_read = read_back and k == len(packets) - 1
            var, wait = ('res', 1.0) if last_read else ('res0', 0.3)
            lines += [f'for _try in range({self.RETRIES}):',
                      f'    flange_serial_write({p!r})',
                      f'    {var}, rx{"" if last_read else "0"} = flange_serial_read({wait})',   # 응답을 읽어 비운다
                      f'    if {var} > 0:',
                      '        break']
        if read_back:
            lines += [
                f'sock = client_socket_open("{host}", {int(port)})',
                'if res > 0:',
                '    client_socket_write(sock, rx)',
                'else:',
                '    client_socket_write(sock, b"NODATA")',
                'client_socket_close(sock)',
            ]
        lines.append('flange_serial_close()')
        return '\n'.join(lines)

    def _ensure_autonomous(self):
        """[실기] DRL 은 자동 모드에서만 돈다. 게임을 강제 종료(터미널 닫기)한 뒤 그리퍼 DRL 이 시간 초과로 멈추고
        로봇이 수동 모드로 남아 펜 반납이 계속 'drl_script_run 실패' 였다(2026-11-14). 수동이면 자동으로 바꾼다(동작 없음)."""
        dr = self.dr
        get_mode, set_mode = getattr(dr, 'get_robot_mode', None), getattr(dr, 'set_robot_mode', None)
        auto = getattr(dr, 'ROBOT_MODE_AUTONOMOUS', 1)
        if get_mode is None or set_mode is None:
            return
        try:
            mode = get_mode()
        except Exception:
            return
        if mode != auto:
            print(f'[GRIPPER] 로봇이 수동 모드({mode}) → 자동 모드로 바꿈 (DRL 실행에 필요)', flush=True)
            if set_mode(auto) != 0:
                raise RuntimeError('로봇을 자동 모드로 바꾸지 못함 — 티치 펜던트에서 자동 모드로 바꾸세요')

    def _start_drl(self, code, timeout_s=5.0, tries=3):
        """[실기] DSR_ROBOT2.drl_script_run 은 응답을 무한정 기다린다. 연결 직후 첫 요청이 응답 없이 멈춘 일이
        2026-11-11 세 번 있었다(컨트롤러 쪽에서는 실행 안 됨) → 시간 제한을 두고 다시 요청한다."""
        import rclpy
        dr = self.dr
        self._ensure_autonomous()
        client, node = getattr(dr, '_ros2_drl_start', None), getattr(dr, 'g_node', None)
        if client is None or node is None:                    # 테스트·다른 버전: 원래 함수
            return dr.drl_script_run(self.system, code) == 0
        for k in range(tries):
            req = dr.DrlStart.Request()
            req.robot_system, req.code = self.system, code
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
            if fut.done():
                res = fut.result()
                return bool(res is not None and res.success)
            fut.cancel()
            print(f'[GRIPPER] DRL 시작 응답 없음 ({timeout_s:g}s) → 다시 요청 {k + 2}/{tries}', flush=True)
            if dr.get_drl_state() == dr.DRL_PROGRAM_STATE_PLAY:  # 응답만 늦고 실행은 된 경우
                return True
        return False

    def _run(self, packets, read_back=False, host=None, port=None):
        code = self._drl(packets, read_back, host, port)
        if not self._start_drl(code):
            raise RuntimeError('drl_script_run 실패 (로봇이 자동 모드인지, DRL 실행 중인 프로그램이 없는지 확인)')
        t0 = time.time()
        time.sleep(0.2)
        while self.dr.get_drl_state() == self.dr.DRL_PROGRAM_STATE_PLAY:
            if time.time() - t0 > self.timeout_s:
                self.dr.drl_script_stop(self.dr.DR_QSTOP)
                raise TimeoutError('그리퍼 DRL 스크립트 시간 초과 → 중지')
            time.sleep(0.05)

    # 인터페이스 -----------------------------------------------------------
    def open(self):
        self._run([self.proto.goal(int(self.cfg['open_position']))])
        time.sleep(float(self.cfg.get('settle_s', 0.8)))
        # [실기] 명령이 유실돼도 모르고 지나가지 않도록, 위치를 읽을 수 있으면 실제로 열렸는지 확인
        limit = int(self.cfg['open_position']) + int(self.cfg.get('open_tolerance', 60))
        pos = self._wait_position(lambda p: p <= limit)
        if pos is not None and pos > limit:
            raise RuntimeError(f'그리퍼가 열리지 않음 (위치 {pos}, 기대 {limit} 이하)')
        return pos

    def close(self, check_held=True):
        """닫고, 위치를 읽을 수 있으면 반환. holding_min~max 가 있으면 그 안인지 확인(밖이면 PenNotHeld)."""
        self._run([self.proto.goal(int(self.cfg['closed_position']))])
        time.sleep(float(self.cfg.get('settle_s', 0.8)))
        lo, hi = self.cfg.get('holding_min'), self.cfg.get('holding_max')
        pos = self._wait_settled()                    # 천천히 닫히는 중에는 빈 손이어도 450~605 를 지나가므로 멈춘 뒤 판정
        if not (check_held and lo is not None and hi is not None):
            return pos
        if pos is not None and not (lo <= pos <= hi):
            raise PenNotHeld(f'펜 부품을 잡지 못함: 그리퍼 위치 {pos} (허용 {lo}~{hi}, 끝까지 닫히면 빈 손)')
        return pos

    def _wait_settled(self, tol=3, interval_s=0.3):
        """위치가 연속 두 번 tol 이내로 같아질 때(멈춤)까지 읽는다. readback 없으면 None."""
        prev = None
        def ok(p):
            nonlocal prev
            done = prev is not None and abs(p - prev) <= tol
            prev = p
            return done
        return self._wait_position(ok, interval_s)

    def _wait_position(self, ok, interval_s=0.3):
        """readback_host 가 없으면 None(확인 생략). 있으면 ok(위치)가 되거나 motion_timeout_s 가 지날 때까지 읽고
        마지막 위치를 반환. 천천히 움직이게 설정하면 도착까지 몇 초 걸린다."""
        if not self.cfg.get('readback_host'):
            return None
        deadline = time.time() + float(self.cfg.get('motion_timeout_s', 6.0))
        while True:
            pos = self.position()
            if ok(pos) or time.time() >= deadline:
                return pos
            time.sleep(interval_s)

    def position(self):
        host = self.cfg.get('readback_host')
        port = int(self.cfg.get('readback_port', 20002))
        if not host:
            raise ValueError('gripper.readback_host(컨트롤러에서 보이는 PC IP)가 필요합니다')
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', port))
        srv.listen(1)
        srv.settimeout(self.timeout_s)
        try:
            self._run([self.proto.read_position()], read_back=True, host=host, port=port)
            conn, _ = srv.accept()
            with conn:
                conn.settimeout(2.0)
                data = b''
                try:
                    while True:
                        chunk = conn.recv(256)
                        if not chunk:
                            break
                        data += chunk
                except socket.timeout:
                    pass
        finally:
            srv.close()
        if data == b'NODATA':
            raise ValueError('그리퍼 응답 없음 (ID/baud/protocol 확인)')
        return self.proto.parse_position(data)

    def shutdown(self):
        try:
            self._run([self.proto.torque(False)])
        except Exception as e:
            print(f'[GRIPPER] torque off 실패: {e}')
