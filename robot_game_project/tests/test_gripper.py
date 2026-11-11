"""플랜지 그리퍼 패킷(robot_games.gripper) 테스트. 로봇 없이 바이트만 확인한다."""
import pytest

from robot_games import gripper as g


def test_dxl_crc_manual_examples():
    # ROBOTIS e-Manual Protocol 2.0 예제 패킷
    assert g.dxl_packet(1, 0x01) == bytes.fromhex('FF FF FD 00 01 03 00 01 19 4E')
    assert g.dxl_read(1, 132, 4) == bytes.fromhex('FF FF FD 00 01 07 00 02 84 00 04 00 1D 15')
    assert g.dxl_write(1, 116, 512, 4) == bytes.fromhex('FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89')


def test_modbus_crc_example_and_registers_required():
    assert g.modbus_read_holding(1, 0, 10) == bytes.fromhex('01 03 00 00 00 0A C5 CD')
    with pytest.raises(ValueError):
        g._Protocol({'protocol': 'modbus', 'id': 1})


REG = {'torque_enable': 256, 'goal_current': 275, 'goal_position': 282, 'present_position': 290}


def test_modbus_goal_and_current_packets():
    proto = g._Protocol({'protocol': 'modbus', 'id': 1, 'registers': REG, 'goal_current': 400})
    assert proto.goal_current()[:6] == bytes([1, 0x06, 0x01, 0x13, 0x01, 0x90])        # 40276 = 400
    assert proto.goal(620)[:7] == bytes([1, 0x10, 0x01, 0x1A, 0x00, 0x02, 0x04])        # 40283 FC16 2레지스터


def test_modbus_read_skips_stale_write_echo():
    echo = g.modbus_write_single(1, 256, 1)
    body = bytes([1, 0x03, 4]) + (620).to_bytes(2, 'big') + (0).to_bytes(2, 'big')
    reply = body + g.crc16_modbus(body).to_bytes(2, 'little')
    proto = g._Protocol({'protocol': 'modbus', 'id': 1, 'registers': REG})
    assert proto.parse_position(echo + reply) == 620


# ---------------------------------------------------------------- FlangeGripper (컨트롤러 없이)
CFG = {'protocol': 'modbus', 'id': 1, 'registers': REG, 'open_position': 0, 'closed_position': 620,
       'holding_min': 450, 'holding_max': 605, 'settle_s': 0, 'readback_host': '127.0.0.1'}


def _fake(positions, cfg=CFG):
    """DRL 을 실행하지 않고 position() 이 positions 를 차례로 돌려주는 FlangeGripper."""
    grip = g.FlangeGripper.__new__(g.FlangeGripper)
    grip.cfg, grip.baud, grip.proto = dict(cfg), 57600, g._Protocol(cfg)
    grip.sent = []
    grip._run = lambda packets, *a, **k: grip.sent.append(packets)
    seq = iter(positions)
    grip.position = lambda: next(seq)
    return grip


def test_drl_waits_after_open_and_retries_until_reply():
    code = _fake([])._drl([g.modbus_write_single(1, 256, 1)])
    lines = code.splitlines()
    assert lines[1] == 'res0, rx0 = flange_serial_read(0.1)'          # 포트 연 직후 바로 쓰지 않음
    assert lines[2] == f'for _try in range({g.FlangeGripper.RETRIES}):'
    assert '    if res0 > 0:' in lines and '        break' in lines


def test_open_checks_position_and_raises_when_not_moved():
    assert _fake([300, 10]).open() == 10                                  # 늦게 열려도 다시 읽어 확인
    with pytest.raises(RuntimeError, match='열리지 않음'):
        _fake([580] * 3, {**CFG, 'motion_timeout_s': 0.5}).open()


def test_close_detects_empty_hand_and_skips_check_without_readback():
    assert _fake([560, 561]).close() == 561
    with pytest.raises(g.PenNotHeld):
        _fake([350, 520, 610, 619, 619]).close()                          # 지나가는 중의 520 은 잡음이 아님 → 멈춘 619 = 빈 손
    assert _fake([618, 618]).close(check_held=False) == 618
    assert _fake([], {**CFG, 'readback_host': None}).close() is None       # 위치를 못 읽는 설정이면 확인 생략


def test_profile_packets_slow_the_gripper():
    proto = g._Protocol({**CFG, 'registers': {**REG, 'profile_acceleration': 278, 'profile_velocity': 280},
                         'profile_acceleration': 600, 'profile_velocity': 600})
    pk = proto.profile()
    assert [p[:7] for p in pk] == [bytes([1, 0x10, 0x01, 0x16, 0, 2, 4]), bytes([1, 0x10, 0x01, 0x18, 0, 2, 4])]
    assert pk[0][7:9] == (600).to_bytes(2, 'big')
    assert g._Protocol(CFG).profile() == []                               # 설정이 없으면 보내지 않음
