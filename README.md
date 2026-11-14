# doosan-gripper-camera-game

두산 E0509 로봇팔이 테이블 위 카메라로 종이를 보고 펜으로 **미로**를 풀어 그리거나 사람과 **틱택토**를 둡니다. 게임은 액션카드를 카메라에 보여 고르고, 펜은 거치대에서 집어 쓰고 게임이 끝나면 돌려놓습니다.

- Doosan Robot E0509
- Robotis Gripper (RH-P12-RN(A), 플랜지 시리얼)
- Camera (Intel RealSense, 컬러 영상만 사용)

코드는 모두 [`robot_game_project/`](robot_game_project/) 안에 있습니다. 설비 값, 보정 절차, 실기에서 겪은 문제는 [`docs/REAL_CELL.md`](robot_game_project/docs/REAL_CELL.md)에 정리했습니다.

## 흐름

1. **카드 인식**: 카드를 카메라에 가까이 대면 OCR로 TIC(틱택토)·미로·STOP(종료)을 고릅니다.
2. **펜 집기**: ArUco 25번 마커로 거치대 위치를 재고 펜을 집습니다.
3. **게임**
   - **미로**: 보드 마커(0~3) 사이의 미로를 찍고, 입구·출구를 찾아 통로 가운데로 풀고, 로봇 좌표로 바꿔 그립니다.
   - **틱택토**: 로봇이 X를 그리고 사람이 O를 그립니다. 판을 다시 찍어 사람 수를 알아냅니다.
4. **펜 반납**: 게임이 오류로 끝나도 펜을 거치대에 돌려놓고 home으로 갑니다.

## 준비

Ubuntu 24.04, ROS 2 Jazzy, 두산 ROS 2 드라이버(`~/ros2_ws`)가 필요합니다.

```bash
cd robot_game_project
python3 -m venv .venv && .venv/bin/pip install numpy "opencv-python>=4.8,<5" PyYAML "pytest>=8,<10"
sudo apt install tesseract-ocr tesseract-ocr-kor      # 카드 OCR
```

`requirements.txt`는 ROS 환경을 그대로 freeze한 파일이라 쓰지 않습니다.

## 실행

로봇을 움직이는 명령은 bringup을 먼저 띄우고, 같은 ROS 환경을 불러온 셸에서 실행합니다.

```bash
# 셸 1: 로봇 드라이버
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.52 model:=e0509 name:=dsr01 gui:=false

# 셸 2: robot_game_project 에서
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

| 하는 일 | 명령 |
|---|---|
| **관제실**(조그·home·그리퍼·펜 집기/반납·카메라·게임·테스트) | `.venv/bin/python -m robot_games.control` |
| 카드로 게임 고르기 | `.venv/bin/python -m robot_games.app --game camera --mode real --execute-motion` |
| 틱택토 바로 시작 | `.venv/bin/python -m robot_games.app --game 1 --mode real --execute-motion` |
| 미로 바로 시작 | `.venv/bin/python -m robot_games.maze_game --source camera --mode real --execute-motion` |
| 허공 테스트(종이 5mm 위) | `.venv/bin/python -m robot_games.control --tic-air-test` 또는 `--maze-air-test` |
| 펜 거치대 단독 | `.venv/bin/python -m robot_games.pen_holder pick\|return --execute-motion [--rehearsal]` |
| 보정·교시 도구 | `.venv/bin/python -m robot_games.calibrate view\|markers\|camera-robot\|poses\|holder\|gripper` |

로봇·카메라 없이 확인할 때는 다음 명령을 씁니다.

```bash
.venv/bin/python -m robot_games.maze_game --source demo          # 미로 mock
.venv/bin/python scripts/preview_maze_path.py                    # 실기 경로를 사진 위에 그려 확인
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -p no:cacheprovider tests/
```

## 구조

| 모듈 | 하는 일 |
|---|---|
| `app.py` | 카드 또는 `--game`으로 게임 선택, 앞뒤로 펜 집기·반납 |
| `maze_game.py` → `paper_maze.py` → `maze_csv.py` → `maze_executor.py` | 미로: 촬영·풀이(0~1 좌표 JSON) → 로봇 좌표 CSV → 검증 후 실행 |
| `tictactoe/tictactoe.py` | 틱택토: 격자 인식, O 인식, 로봇 X 그리기(`RobotX`) |
| `pen_holder.py` | 마커 25번으로 거치대를 찾아 펜 집기·반납 |
| `control.py` | 관제실 창. 로봇 연결은 `--worker` 하위 프로세스가 맡음 |
| `motion.py` (`GuardedDoosan`) | 두산 연결과 안전 검사(속도, 도착 확인, 펜 높이 하한) |
| `gripper.py` | 플랜지 시리얼로 그리퍼 열기·닫기 |
| `camera_select.py` | RealSense 컬러 노드 찾기, 끊기면 다시 연결 |
| `calibrate.py`, `jog.py` | 보정·교시 도구 |

보정 결과는 `configs/field/`에 저장되고, 미로 펜 높이 보정은 `configs/my_verified_calibration_real.json`, 설비 설정은 `configs/default.yaml`에 있습니다.

## 안전

- 실기로 움직이려면 `--execute-motion`, `default.yaml`의 `robot.armed`·`calibration_confirmed`, home 관절값, REAL 컨트롤러 확인이 모두 맞아야 합니다.
- 보정 도구는 결과를 `measured_and_verified: false`로 저장합니다. 사람이 리허설로 확인한 뒤 `true`로 바꿔야 실기 변환에 쓰입니다.
- 미로 실행 전에 도달 범위, 펜 높이 차이, 펜 내린 상태 점프를 검사합니다. 보호정지·비상정지 상태에서는 home으로 자동 이동하지 않습니다.
- 새 위치나 설정으로 처음 움직일 때는 펜을 띄운 리허설(관제실 허공 테스트, `pen_holder --rehearsal`)부터 돌립니다.
- 직접 교시가 안 되는 로봇이라 위치는 조그로 맞춥니다. 실행 중에는 비상정지에 손이 닿게 둡니다.
