"""두산 컨트롤러(dsr_controller2) 연결 사전 점검. DSR_ROBOT2 를 import 하기 전에 부른다.

DSR_ROBOT2 는 서비스가 없으면 무한정 기다리거나 import 단계에서 멈추기 때문에,
여기서 제한 시간 안에 응답을 확인하고 안 되면 원인을 안내한다.
"""

GUIDE = """두산 컨트롤러가 응답하지 않습니다 (namespace={ns}).
  - 에뮬레이터: ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual model:=e0509 name:={ns}
  - 실기:       ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<로봇IP> model:=e0509 name:={ns}
  - 이 셸에서 source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash 를 했는지 확인
  - ros2 service list | grep {ns}/system 으로 서비스가 보이는지 확인"""


def preflight(node, ns=None, timeout_s=5.0):
    """system/get_robot_system 을 호출해 컨트롤러 종류를 반환한다 (0=REAL, 1=VIRTUAL).
    응답이 없으면 RuntimeError."""
    import rclpy
    from dsr_msgs2.srv import GetRobotSystem
    ns = ns or node.get_namespace().strip('/') or 'dsr01'
    cli = node.create_client(GetRobotSystem, 'system/get_robot_system')
    try:
        if not cli.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(GUIDE.format(ns=ns))
        fut = cli.call_async(GetRobotSystem.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
        res = fut.result() if fut.done() else None
        if res is None or not getattr(res, 'success', True):
            raise RuntimeError('get_robot_system 응답 실패\n' + GUIDE.format(ns=ns))
        return int(res.robot_system)
    finally:
        node.destroy_client(cli)
