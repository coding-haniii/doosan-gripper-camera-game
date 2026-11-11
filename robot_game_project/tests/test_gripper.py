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
