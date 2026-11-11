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
    'dynamixel2': {'torque_enable': 512, 'goal_current': 550, 'goal_position': 564, 'present_position': 580},
    # Modbus 펌웨어는 주소가 다르다(0부터 세는 레지스터 번호 = 4xxxx - 40001). default.yaml 에 입력
    'modbus': {'torque_enable': None, 'goal_current': None, 'goal_position': None, 'present_position': None},
}


def _require_positions(cfg):
    if cfg.get('open_position') is None or cfg.get('closed_position') is None:
        raise ValueError('gripper.open_position / closed_position 을 실물에서 교시하세요 (robot-calibrate gripper)')



class FlangeGripper:
    """두산 플랜지 시리얼. DRL 스크립트를 실행하고 끝날 때까지 기다린다."""

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
        self._run(init)

    # DRL 코드 생성 --------------------------------------------------------
    def _drl(self, packets, read_back=False, host=None, port=None):
        lines = [f'flange_serial_open(baudrate={self.baud}, bytesize=DR_EIGHTBITS, '
                 f'parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)',
                 # [실기] 열자마자 쓰면 첫 패킷이 버려진다(2026-11-11 실측: 응답 없음 → 0.1초 뒤 쓰면 매번 응답)
                 'res0, rx0 = flange_serial_read(0.1)']
        for k, p in enumerate(packets):
            lines.append(f'flange_serial_write({p!r})')
            if not (read_back and k == len(packets) - 1):
                # 쓰기 명령의 응답을 읽어서 버린다. 남겨 두면 다음 읽기 응답 앞에 섞여 들어온다
                lines.append('res0, rx0 = flange_serial_read(0.2)')
        if read_back:
            lines += [
                'res, rx = flange_serial_read(1.0)',
                f'sock = client_socket_open("{host}", {int(port)})',
                'if res > 0:',
                '    client_socket_write(sock, rx)',
                'else:',
                '    client_socket_write(sock, b"NODATA")',
                'client_socket_close(sock)',
            ]
        lines.append('flange_serial_close()')
        return '\n'.join(lines)

    def _run(self, packets, read_back=False, host=None, port=None):
        code = self._drl(packets, read_back, host, port)
        if self.dr.drl_script_run(self.system, code) != 0:
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

    def close(self):
        self._run([self.proto.goal(int(self.cfg['closed_position']))])
        time.sleep(float(self.cfg.get('settle_s', 0.8)))

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
