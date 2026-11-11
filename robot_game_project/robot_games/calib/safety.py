"""로봇 상태 감시. ROS 에 의존하지 않아 mock/테스트에서도 같은 코드를 쓴다.

하드웨어 비상정지 버튼과 보호정지 자체는 두산 컨트롤러가 처리한다.
여기서는 그 상태를 감지해 시나리오를 멈추게 한다. 정지 후에는 home 등 어떤 자동 이동도 하지 않는다.
"""

# dsr_msgs2/srv/GetRobotState.srv
STATE_INITIALIZING = 0
STATE_STANDBY = 1
STATE_MOVING = 2
STATE_SAFE_OFF = 3
STATE_TEACHING = 4
STATE_SAFE_STOP = 5
STATE_EMERGENCY_STOP = 6
STATE_HOMMING = 7
STATE_RECOVERY = 8
STATE_SAFE_STOP2 = 9
STATE_SAFE_OFF2 = 10
STATE_NOT_READY = 15

STOP_STATES = {
    STATE_SAFE_OFF: 'SAFE_OFF(서보 꺼짐)',
    STATE_SAFE_STOP: 'SAFE_STOP(보호정지)',
    STATE_EMERGENCY_STOP: 'EMERGENCY_STOP(비상정지)',
    STATE_SAFE_STOP2: 'SAFE_STOP2(보호정지)',
    STATE_SAFE_OFF2: 'SAFE_OFF2(서보 꺼짐)',
    STATE_RECOVERY: 'RECOVERY(복구 모드)',
    STATE_NOT_READY: 'NOT_READY',
}


class RobotStopped(RuntimeError):
    """로봇이 정지 상태이거나 상태를 읽을 수 없음. 시나리오를 끝내야 한다(자동 재개·자동 이동 금지)."""


class SafetyMonitor:
    def __init__(self, get_state, stop_fn):
        self._get_state = get_state
        self._stop_fn = stop_fn
        self.tripped = None

    def check(self):
        if self.tripped:
            raise RobotStopped(self.tripped)
        s = self._get_state()
        if s is None or s < 0:
            self.tripped = '로봇 상태를 읽을 수 없음(통신 끊김?)'
        elif s in STOP_STATES:
            self.tripped = f'로봇 정지 상태: {STOP_STATES[s]}'
        if self.tripped:
            raise RobotStopped(self.tripped)
        return s

    def stop(self):
        self._stop_fn()
