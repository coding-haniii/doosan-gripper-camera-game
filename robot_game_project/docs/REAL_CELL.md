# 실기 셀 정보와 절차 (미로 그리기)

2026-11-10~11 현장에서 확인하고 실제로 미로를 그린 내용입니다. 설비를 바꾸면 이 값들을 다시 확인하세요.

## 설비 (확인됨)

| 항목 | 값 |
|---|---|
| 로봇 | 두산 E0509, 컨트롤러 V2.10.3 (GV02100300), IP **110.120.1.52** |
| 티치펜던트·콕핏 | **없음 → 직접 교시(손으로 끌기) 불가.** 설정은 윈도우 DART-Platform으로 합니다([WINDOWS_DART_SETUP.md](WINDOWS_DART_SETUP.md)). 위치 맞추기는 키보드 조그(`robot_games.calibrate --jog`)로 합니다 |
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

## 틱택토

```bash
.venv/bin/python -m robot_games.app --game 1 --mode real --execute-motion
```

- **빈 3×3 격자**(바깥 테두리 포함)를 마커 사각형 안에 둡니다. 표시가 있으면 "Clear all marks from the grid"를 띄우고 시작하지 않습니다.
- 선공은 무작위입니다. 사람은 O를 빈 칸 가운데에 진하고 동그랗게 그리고 손을 화면 밖으로 빼면 자동으로 넘어갑니다. 로봇은 X를 그린 뒤 home으로 비킵니다.
- 처음이거나 위치를 바꿨으면 `robot.drawing_calibration`을 +50mm 사본으로 바꾼 설정(`--config`)으로 공중 리허설부터 합니다.
- 알려진 점: 로봇 X를 그린 뒤 "X가 카메라에 잘 안 보입니다" 경고가 떠도, 실제로는 그려져 있는 경우가 있습니다(가는 펜 선이 판정 기준 아래). 게임 진행에는 영향 없습니다.
- 게임 중에 다른 프로그램으로 카메라를 열면 게임 쪽 영상이 끊깁니다.

## 카드로 게임 고르기 (앱)

```bash
.venv/bin/python -m robot_games.app --game camera --mode real --execute-motion
```

- 액션카드: TIC → 틱택토, 미로 카드 → 미로, STOP → 안전 종료(home). 2026-11-11 mock으로 세 장 모두 맞게 인식했습니다.
- OCR에는 `sudo apt install tesseract-ocr tesseract-ocr-kor`와 venv의 `pytesseract`가 필요합니다.
- **카드는 손에 들고 카메라에 가까이 대서** 초록 상자를 거의 채우게 찍습니다. 테이블 위에 두면 카드보다 테이블 판이 더 큰 밝은 사각형이라 그걸 카드로 잘라 인식이 실패합니다.
- 카드 하나로 게임 하나가 끝나면 앱도 끝납니다(카드 루프는 보류). 앱을 `--execute-motion`으로 띄우면 미로도 `RUN` 입력 없이 바로 그립니다(펜 집기·반납과 같음). 시작 전에 종이 주변과 비상정지를 확인합니다.
- OCR이 짧은 영문 조각(`IT` 등)을 카드 별칭으로 잘못 읽어 미로 카드가 STOP으로 잡힌 일이 있었습니다 → 여러 토큰 중 3글자 미만 영문 조각은 무시합니다.

## 관제실 (컨트롤 센터)

```bash
.venv/bin/python -m robot_games.control                     # 조그·home·그리퍼·펜 집기/반납·카메라·게임 실행
.venv/bin/python -m robot_games.control --tic-air-test      # 펜 집기 → 틱택토 5칸에 X를 종이 5mm 위 허공에 → 펜 반납
.venv/bin/python -m robot_games.control --maze-air-test     # 펜 집기 → 미로 촬영(SPACE)·풀이 → 경로를 5mm 위 허공에 → 펜 반납
```

- 창은 로봇에 직접 붙지 않고 `--worker` 하위 프로세스가 연결합니다. 펜 거치대·게임을 띄우는 동안 worker와 카메라를 놓았다가 끝나면 다시 잡습니다. 게임은 새 터미널 창에서 실행됩니다.
- 버튼이 회색(비활성)이면 로봇 연결이 안 된 것입니다. bringup이 "RT data is stale"이면 다시 켭니다.
- 로그는 `out/control_center.log`에도 남습니다.
- 관제실을 두 개 띄우면 로봇 연결과 카메라가 충돌합니다. 새로 열기 전에 이전 창을 닫습니다.

## 펜 거치대에서 집기·반납 (비전 Pick)

펜 부품(회색 3D 프린트 + 고무줄 펜)을 눕힌 컵 거치대에 꽂아 두고 로봇이 집고 반납합니다. 거치대 옆 판에 붙인 ArUco **25번**(`vision.holder_marker_id`)으로 카메라가 위치와 방향을 찾습니다.

```bash
.venv/bin/python -m robot_games.calibrate holder --jog --execute-motion        # 교시(거치대를 새로 만들었을 때만)
.venv/bin/python -m robot_games.pen_holder pick --execute-motion --rehearsal --lift 15   # 부품 바로 위까지 공중 확인
.venv/bin/python -m robot_games.pen_holder pick --execute-motion                  # 집기 → home
.venv/bin/python -m robot_games.pen_holder return --execute-motion                # 반납 → home
```

- 교시: approach에서는 부품 위 6~8cm에서 멈추고(내려가지 않음), grasp에서 1mm 스텝으로 손가락이 부품 양옆을 감싸는 높이까지 내린 뒤 `y`로 잡아 보기. 잡은 위치가 520~590 근처면 정상(컵을 잡으면 낮게 나옴). 저장은 `verified: false` → 리허설 확인 후 `configs/field/robot_poses.json`에서 `true`.
- 접근 높이는 `pen_holder.approach_dz_mm`(140mm). 80mm로는 펜이 컵에서 다 빠지지 않았습니다.
- **거치대(판과 컵)를 테이프로 단단히 고정합니다.** 고정하지 않았을 때는 펜을 뽑을 때 컵이 끌려가 20mm 넘게 밀렸고, 반납 때 펜이 걸려 넘어졌습니다.
- 반납은 home에서 마커를 재지 않습니다. 쥔 펜(home)이나 팔(거치대 바로 위)이 마커를 가려 20° 넘게 틀리게 재지기 때문입니다. 접근 높이에서 옆으로 15cm 비킨 자세에서 다시 재고 그 값으로 꽂습니다.
- 거치대가 교시 때보다 20° 넘게 돌아가 있거나 20cm 넘게 옮겨져 있으면 거부합니다. 거치대 안에서 오류가 나면 그리퍼를 그대로 둔 채 수직으로 빠진 뒤에만 home으로 갑니다.
- **앱 연결**(`robot.pen_from_holder: true`): 카드로 미로·틱택토가 정해지면 펜 집기 → 게임 → 펜 반납(게임이 오류로 끝나도). 종료 카드는 쥐고 있으면 반납(빈 손이면 그냥 home). 집기·반납은 별도 프로세스(`pen_holder --yes`)라 게임의 로봇 연결과 겹치지 않습니다. 종료코드 3=이미 쥐고 있음(그대로 진행), 4=빈 손(반납할 것 없음).
- 그리퍼는 `profile_velocity/acceleration: 600`으로 천천히 잡고 놓습니다(켜질 때는 최대 속도라 진동). 닫은 뒤 위치가 멈출 때까지 기다렸다가 잡음을 판정합니다(빈 손이면 620까지 닫힘).

## 다시 보정해야 할 때 (마커나 카메라를 옮긴 경우)

- **카메라만 움직였을 때**(2026-11-14): 미로·틱택토는 매번 마커 픽셀과 저장된 마커 로봇 좌표로 계산해서 영향이 없습니다. 펜 거치대는 `camera_robot_calibration.json`의 H(테이블 평면 호모그래피)를 그대로 쓰므로 틀어집니다(반납 때 86mm 어긋남). 보드 마커 0~4의 지금 픽셀과 원래 로봇 좌표로 H를 다시 계산했습니다. 반납 때 다시 잰 마커가 대략값과 30mm 넘게 다르면 펜을 쥔 채 거부합니다(`MAX_REFINE_SHIFT_MM`).

```bash
.venv/bin/python -m robot_games.calibrate view                                  # 마커 5/5 보이는지
.venv/bin/python -m robot_games.calibrate camera-robot --jog --execute-motion   # 펜 끝을 마커 중심에 대기
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
- 스플라인에 1mm보다 짧은 구간이 있으면 컨트롤러가 일정 속도를 포기합니다(알람 3213) → 점 간격 2mm 이상. 직각 모서리 직전에는 크게 감속해 멈칫합니다 → 반지름 5mm 원호로 다듬습니다(`smooth_pen_path`).
- `maze_game`이 `--clearance`를 강제로 넘기면 통로 가운데 경로가 꺼졌습니다 → 값을 줄 때만 넘깁니다.
- RealSense가 USB 통신 오류(-71)로 끊겼다 다시 붙는 일이 있었습니다(2026-11-11 여러 번). 케이블이 당겨지지 않게 고정하고, 노트북 USB 3 포트에 직접 꽂습니다. 끊기면 다시 꽂고 `calibrate view`로 영상이 나오는지 확인합니다. 다시 붙은 직후에는 열리기만 하고 영상이 안 나오는 일이 있어 잠깐 기다렸다 다시 실행합니다.
- bringup을 오래 켜 두면 로그에 "RT data is stale"이 계속 나오고 로봇 상태가 3(SAFE_OFF)으로 바뀌어 있을 수 있습니다. bringup을 끄고 다시 켜면 STANDBY(1)로 돌아옵니다.
- 그리퍼 케이블(플랜지 커넥터) 접촉이 나쁘면 건드릴 때마다 그리퍼가 재부팅됩니다(초록불이 잠깐 켜짐. 평소에는 꺼져 있음). 재부팅되면 토크가 꺼지고 전류 등 설정이 기본값으로 돌아가 위치 명령을 무시합니다. 회색 부품은 마찰로만 물려 있게 됩니다. 커넥터를 끝까지 꽂고 잠금 링을 조이며, 케이블이 팔 동작에 당겨지지 않게 정리합니다. 상태 확인: 레지스터 40257(토크)이 1이어야 합니다.
- 플랜지 시리얼은 포트를 연 직후에 쓰면 첫 패킷이 버려집니다(응답 없음). `FlangeGripper`가 열고 0.1초 기다린 뒤에 쓰고, 응답이 없으면 최대 3번 다시 보냅니다. 열기·닫기 후에는 위치를 읽어 실제로 움직였는지 확인합니다(`readback_host`).
- 연결 직후 첫 DRL 시작 요청(`drl_script_run`)이 응답 없이 멈추는 일이 있었습니다(하루 세 번). `FlangeGripper`는 5초 안에 응답이 없으면 다시 요청합니다.
- 그리퍼 DRL은 로봇 모드가 AUTONOMOUS여야 돕니다. 수동(MANUAL)이면 멈춥니다 → `FlangeGripper`가 시작 전에 자동 모드로 바꿉니다.
- 틱택토 격자 초록 테두리가 깜빡여 진행이 안 됐습니다(테이프 꼬리, 격자가 마커 4에 닿음) → 윤곽을 열림 연산으로 다듬고, 못 찾으면 마커를 흰색으로 지우고 다시 찾고, 안정 판정 허용을 8px로 넓혔습니다. 찌그러진 사각형(ValueError)은 건너뜁니다.
- RealSense가 끊겼다 붙으면 노드 번호가 바뀝니다(video6↔7). RealSense가 있는데 못 읽으면 노트북 웹캠으로 넘어가지 않고 오류를 냅니다. 게임·테스트 중에 끊기면 `ReconnectingCamera`가 최대 60초 동안 다시 찾아 열고 이어서 진행합니다.
- 입구·출구가 모서리에 있고 바깥 벽 한쪽이 짧은 미로에서 안쪽 벽을 바깥 벽으로 골라 시작점이 종이 밖으로 나가 CSV 변환이 거부됐습니다(2026-11-14) → 가장 바깥쪽 진한 선을 바깥 벽으로 고르고, 한 모서리의 두 구멍은 하나로 합치고, 구멍 바깥 점은 보드 안으로 제한합니다.
- 펜 재파지 시험(2026-11-11): 그리퍼를 열어 회색 부품을 손으로 뺐다 다시 끼우고 같은 점에 `+`를 그렸더니, 카메라로 구분되지 않을 만큼(약 1~2mm 안) 모였습니다. 거치대에서 펜 부품을 집는 방식이 가능해 보입니다.
