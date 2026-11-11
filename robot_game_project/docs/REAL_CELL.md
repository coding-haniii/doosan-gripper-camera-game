# 실기 셀 정보와 절차 (미로 그리기)

2026-11-10~11 현장에서 확인하고 실제로 미로를 그린 내용입니다. 설비를 바꾸면 이 값들을 다시 확인하세요.

## 설비 (확인됨)

| 항목 | 값 |
|---|---|
| 로봇 | 두산 E0509, 컨트롤러 V2.10.3 (GV02100300), IP **110.120.1.52** |
| 티치펜던트·콕핏 | **없음 → 직접 교시(손으로 끌기) 불가.** 설정은 윈도우 DART-Platform으로 합니다([WINDOWS_DART_SETUP.md](WINDOWS_DART_SETUP.md)). 위치 맞추기는 키보드 조그(`robot_games.calib`)로 합니다 |
| TCP | `pen_tip`(플랜지→펜 끝 0,0,180mm), `gripper_tip`(0,0,85). DART에서 등록·활성화됨. 코드는 TCP를 바꾸지 않습니다 |
| 그리퍼 | RH-P12-RN(A), 두산 플랜지 시리얼, Modbus RTU ID 1. 열림 0, 닫힘 620. 펜은 **미리 물려 둡니다**(게임 중에는 그리퍼를 건드리지 않음) |
| 카메라 | Intel RealSense D455f 컬러, 테이블 위 고정. **USB 3 포트**에 꽂아야 안정적입니다. 노드 번호는 PC마다 달라서 `camera_select`가 실행할 때마다 찾습니다. 1280×720 사용(테이블 위 1px ≈ 1mm) |
| 테이블 높이 | 펜 끝 Z 약 44.8mm(로봇 앞 45cm) ~ 48.8mm(66cm). 멀수록 10cm당 1.9mm 높아집니다(기울기 또는 팔 처짐) → `pen_contact_plane` |
| 도달 범위 | 펜 수직, 테이블 높이에서 로봇 중심 앞 약 22~70cm, 좌우 ±20cm. 높이가 높을수록 짧아집니다(Z 206mm에서 64.5cm). DART 안전 설정에서 J2는 ±95° |
| 컨트롤러 속도 배율 | DART에서 **10%**로 남아 있어 모든 동작이 10배 느렸습니다. 연결할 때 `operation_speed_pct: 100`으로 맞춥니다 |

## 마커와 보드

- ArUco `DICT_4X4_50` 50mm 다섯 장을 테이블에 테이프로 고정합니다. 붙인 뒤에는 움직이지 마세요.
- **보드 = 마커 1, 3, 2, 0 중심**(카메라 화면 기준 TL, TR, BR, BL), 300 × 210mm. 4번은 왼쪽 가운데에 둡니다(보정 오차 검증용).
- 마커의 로봇 좌표(펜 끝 터치 실측, BASE mm): 1 (449.9, −150.0), 3 (449.9, 150.0), 2 (659.9, 150.0), 0 (660.0, −150.0), 4 (554.9, −150.0) → [configs/field/camera_robot_calibration.json](../configs/field/camera_robot_calibration.json)
- 미로 종이는 마커 사각형 안 **아무 데나** 놓으면 됩니다. 흰 종이가 흰 테이블 위에 작게 있으면 종이 자동 검출이 실패하므로 마커로 보드를 잡습니다.

## 미로 그리기

```bash
# 셸 1: 로봇 드라이버 (DART에서 실행 중인 프로그램이 없어야 함)
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.52 model:=e0509 name:=dsr01 gui:=false

# 셸 2
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
cd robot_game_project
.venv/bin/python scripts/preview_maze_path.py         # (선택) 로봇 없이 경로를 사진 위에 그려 확인 → out/maze_path_preview.png
.venv/bin/python -m robot_games.maze_game --source camera --mode real --execute-motion
```

카메라 창에서 SPACE로 찍으면 풀이 화면이 잠시 뜨고, 터미널에서 `RUN`을 입력하면 그립니다. 경로는 입구 구멍 바깥에서 시작해 통로 가운데를 지나 출구 구멍 바깥에서 끝납니다.

## 다시 보정해야 할 때 (마커나 카메라를 옮긴 경우)

```bash
.venv/bin/python -m robot_games.calib.calibrate view                                  # 마커 5/5 보이는지
.venv/bin/python -m robot_games.calib.calibrate camera-robot --jog --execute-motion   # 펜 끝을 마커 중심에 대기
.venv/bin/python scripts/make_maze_calibration.py                                     # 미로 보정 다시 만들기
.venv/bin/python scripts/preview_maze_path.py                                         # 경로 확인
```

- `camera-robot --jog`은 붙일 때 짚은 위치(`configs/field/marker_hints.json`)로 마커 위 12mm까지 자동으로 갑니다. 거기서 1mm 단위로 맞추고 살짝 닿게 한 뒤 Enter를 누릅니다. 조그 키: `w/s` X±, `a/d` Y±, `r/f` 위/아래, `1`~`4` 스텝 0.2/1/10/50mm, `q` 중단.
- 결과는 `measured_and_verified: false`로 저장됩니다. 미리보기로 확인한 뒤 직접 `true`로 바꿉니다.
- 자세 교시: `calibrate poses --jog --execute-motion`(home, celebration, camera_clear). home 관절값은 `configs/default.yaml`의 `robot.home_joints_deg`에도 넣어야 `GuardedDoosan`이 씁니다.

## 실기에서 확인한 함정

- `DSR_ROBOT2.py`가 dsr_msgs2에 없는 `SetSingularityHandlingForce`를 참조해 import가 실패합니다 → `motion._patch_dsr_msgs()`가 별칭을 겁니다.
- 컨트롤러가 NOT REACHABLE로 이동을 거부해도 서비스는 성공으로 끝나서 조용히 건너뛰어집니다 → 이동 후 실제 위치로 도착을 확인합니다(3mm).
- 경로가 벽에 붙으면 펜이 벽 선 위에 겹칩니다 → `solve_centered`가 통로 가운데로 풉니다.
- 점마다 `movel`을 쓰면 매번 멈춰서 느립니다(236점에 2분 34초) → 펜 내린 구간은 `movesx`로 이어서 그립니다(50mm/s).
