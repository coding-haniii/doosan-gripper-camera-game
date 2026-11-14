# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 개요

팀 프로젝트(`coding-haniii/doosan-gripper-camera-game`)입니다. 두산 E0509 로봇팔이 테이블 위 RealSense 카메라로 종이를 보고, 그리퍼에 미리 물린 펜으로 미로·틱택토를 그립니다. 코드는 모두 `robot_game_project/` 안에 있고, 사용자 문서와 메시지는 한국어입니다.

- **팀원 코드가 기준입니다.** 지금 단계에서는 없는 기능을 새로 만들지 않습니다. 실기에서 꼭 필요하다고 확인된 수정만 팀원 파일 구조 그대로 몇 줄 단위로 넣습니다. 범위를 넓히기 전에 사용자에게 묻습니다.
- 작업 브랜치는 `jmc`(팀 `dev` 기준)입니다. 예전 개인 저장소 작업(감독 루프, 그리퍼 펜 거치대, 예전 구조 테스트 등)은 `legacy-robot_games-jmc` 브랜치에 보관되어 있습니다. 필요할 때 거기서 가져옵니다.
- 실기 확인(2026-11-11): **미로**(`maze_game`)와 **틱택토**(`app --game 1`, 로봇이 X를 종이에 그림) 각각 처음부터 끝까지 동작. 카드 인식(`app --game camera`)은 mock으로 TIC·미로·STOP 세 장 모두 확인했고(카드는 카메라에 가까이 대서 찍음), 카드로 묶은 실기 실행이 남았습니다. 카드 루프는 아직 보류입니다. 펜 거치대 비전 Pick(`robot_games.pen_holder`, 마커 25번으로 거치대를 찾아 펜 부품 집기·반납)은 단독으로 실기 확인했고, `app.py`에서 게임 앞뒤(집기·반납)와 종료 카드(쥐고 있으면 반납)에 연결했습니다(mock 확인, 앱 전체 실기 실행은 남음).
- 설비 값, 보정, 실기 절차와 함정은 [robot_game_project/docs/REAL_CELL.md](robot_game_project/docs/REAL_CELL.md)에 있습니다.

## 명령어 (`robot_game_project/`에서)

```bash
python3 -m venv .venv && .venv/bin/pip install numpy "opencv-python>=4.8,<5" PyYAML "pytest>=8,<10"   # requirements.txt 는 ROS 환경 freeze 라 쓰지 않음
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -p no:cacheprovider tests/   # 보정·조그·그리퍼·펜 거치대·app 펜 단계·틱택토 X·movesx 평활화 (로봇·카메라 없이)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -p no:cacheprovider tests/test_pen_holder.py -k <이름>   # 테스트 하나만
.venv/bin/python -m robot_games.maze_game --source demo                                  # 미로 mock (카메라·로봇 없음)
.venv/bin/python -m robot_games.maze_game --mode emulator --use-real-calibration --execute-motion   # 실측 보정 CSV를 에뮬레이터로 리허설
.venv/bin/python scripts/preview_maze_path.py                                            # 로봇 없이 실기 경로를 사진 위에 그려 확인
.venv/bin/python -m robot_games.maze_game --source camera --mode real --execute-motion   # 미로 실기 (bringup 필요, REAL_CELL.md)
.venv/bin/python -m robot_games.app --game 1 --mode real --execute-motion                 # 틱택토 실기 (로봇 X, 사람 O)
.venv/bin/python -m robot_games.app --game camera --mode real --execute-motion           # 카드(OCR)로 게임 선택 후 실기 (tesseract-ocr·kor 필요)
.venv/bin/python -m robot_games.control                                                  # 컨트롤 센터 창: 조그·home·그리퍼·펜 집기/반납·카메라·게임 실행 (bringup 필요)
.venv/bin/python -m robot_games.control --tic-air-test | --maze-air-test                 # 펜 집기 → 틱택토 5칸 / 미로 경로를 종이 5mm 위 허공에 → 펜 반납 (관제센터 '테스트' 버튼과 같음)
.venv/bin/python -m robot_games.pen_holder pick|return --execute-motion [--rehearsal]      # 펜 거치대 단독 집기·반납 (--rehearsal: 50mm 위에서 경로만)
.venv/bin/python -m robot_games.calibrate view|markers|camera-robot|poses|holder|gripper  # 재보정·교시(holder=거치대 잡기 자세)·그리퍼 시험 도구
```

실기·에뮬레이터 명령은 `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash` 한 셸에서 실행합니다. 이 PC에는 두산 에뮬레이터(`doosanrobot/dsr_emulator` Docker)도 있습니다.

보정 파일은 모드에 따라 정해집니다. `mock`은 `configs/my_verified_calibration.json`을 써서 `out/maze_coordinates_SIMULATION_ONLY.csv`를 만들고(로봇에 보내면 안 됨), `real`과 `emulator --use-real-calibration`은 `configs/my_verified_calibration_real.json`을 써서 `out/maze_coordinates_ROBOT.csv`를 만듭니다. 설비 설정(`robot.armed` 등)은 `configs/default.yaml`에 있습니다.

## 구조

- **미로 파이프라인**(팀원): `maze_game.run` → `paper_maze`(촬영 → 보드 원근 보정 → 미로 크롭 → 풀이 → 펜 계획 JSON, 0~1 정규화) → `maze_csv`(보드 선형 보정 `origin + u·x_span + v·y_span` → 로봇 좌표 CSV + `.meta.json` 해시) → `maze_executor`(검증 후 `GuardedDoosan.run_rows`).
- **실기용으로 넣은 수정**(`[실기]` 주석으로 표시)
  - `paper_maze`: 보드 = ArUco 마커 1,3,2,0 중심(`vision.board_marker_ids`)이고, 마커가 다 안 보이면 종이 자동 검출을 씁니다. 크롭은 벽 덩어리를 모두 포함하고, 정규화에 크롭 오프셋을 반영합니다. `find_openings`로 입구·출구 바깥에서 시작·끝을 잡고, `solve_centered`로 통로 가운데를 지납니다.
  - `maze_csv`: `pen_contact_plane`(테이블 기울기)과 `pen_press_mm`으로 행마다 펜 Z를 계산합니다.
  - `motion.GuardedDoosan`: `DSR_ROBOT2` 별칭, REAL 컨트롤러 확인, 속도 배율 100%, 도착 확인, `min_pen_z_base_mm` 하한. 펜 내린 구간은 `movesx`로 그립니다(50mm/s, 상한 60).
  - `dsr_preflight.py`: 팀원 커밋에서 빠져 있던 파일을 추가했습니다. `camera_select.py`는 실행할 때마다 RealSense 컬러 노드를 찾습니다.
- **틱택토 실기**(`tictactoe/tictactoe.py`, 팀원 미리보기에 연결 지점만 추가): `RobotX`가 X 획(보드 px)을 `inv(matrix)` → 카메라 px → 마커 호모그래피(`configs/field/` 실측 좌표) → 로봇 XY로 바꿔 `run_rows`로 그리고 home으로 비킵니다. 펜 높이는 미로와 같은 `my_verified_calibration_real.json`(설정 `robot.drawing_calibration`으로 리허설용 사본 지정 가능). 그린 뒤 `refresh_board`로 보드를 다시 찍어야 `new_human_move`가 다음 O를 막지 않습니다.
- **앱 흐름**(`app.py`): 카드/`--game`으로 게임을 고르고, `robot.pen_from_holder`가 true면 미로·틱택토 앞뒤로 `pen_step`이 `robot_games.pen_holder`를 **별도 프로세스**로 띄워 펜을 집고(오류여도) 반납합니다. `DSR_ROBOT2` 연결이 프로세스당 하나라서, 반납 전에 게임 쪽 연결을 `close()`합니다. 종료 코드 3(이미 쥐고 있음)·4(빈 손)는 성공으로 봅니다. 종료 카드는 혹시 쥐고 있으면 반납만 합니다.
- **펜 거치대**(`pen_holder.py`): 카메라로 마커 25번(`vision.holder_marker_id`)의 위치·각도를 재고, `configs/field/robot_poses.json`에 교시한 잡기 자세(교시 때 마커 기준 상대값)를 지금 마커 위치로 옮깁니다. 반납할 때는 집을 때 저장한 마커 값(`out/pen_holder_state.json`)으로 거치대 위까지 가서 다시 재고 꽂습니다.
- **컨트롤 센터**(`control.py`): Tk 창은 로봇에 직접 붙지 않고, `--worker` 하위 프로세스가 `GuardedDoosan`을 잡고 JSON 한 줄씩 주고받습니다. 조그는 `jog.JogPanel._move`를 그대로 써서 하한 검사가 같습니다. 펜 거치대·게임을 띄울 때는 worker와 카메라를 놓았다가 다시 잡습니다(게임은 RUN 입력 때문에 gnome-terminal에서 실행).
- **보정·교시 도구** `robot_games/calibrate.py` + `jog.py`(예전 저장소에서 옮김): 로봇 연결은 팀원 `motion.GuardedDoosan(for_calibration=True)`, 마커 검출은 `paper_maze.detect_markers`, 카메라는 `camera_select`, 그리퍼 시험은 `gripper.FlangeGripper`(팀원 `RobotisGripper` 옆에 추가)를 씁니다. 결과는 `configs/field/`에 저장되고, `scripts/make_maze_calibration.py`가 마커 실측값으로 `configs/my_verified_calibration_real.json`을 다시 만듭니다.
- 같은 일을 하는 코드를 두 벌 두지 않습니다. 필요한 기능은 팀원 파일에 메서드나 함수로 추가합니다.

## 주의할 점

- 팀원이 `out/`, `robot_games/__pycache__/`, `build/`, `install/`, `log/`를 커밋해 두었습니다. 실행하면 바뀌므로 **커밋 전에 `git restore robot_game_project/out robot_game_project/robot_games/__pycache__`를 하고, 고친 파일만 경로를 지정해 `git add`** 하세요. 실행할 때는 `PYTHONDONTWRITEBYTECODE=1`을 권합니다. `.venv`는 `.git/info/exclude`로 제외되어 있습니다.
- 커밋 작성자는 `syaseuta <syaseuta@gmail.com>`입니다.
- **하드웨어 절차를 만들기 전에 매뉴얼과 실제 설비를 확인하세요.** 이 로봇은 직접 교시가 안 됩니다.
- `DSR_ROBOT2`는 import할 때 노드에 묶이므로 프로세스당 연결 하나만 만드세요.

## 안전 규칙 (사용자가 명시적으로 요청할 때만 변경)

- real 연결 조건: `--execute-motion` + `robot.armed`·`calibration_confirmed` + home 관절값 + REAL 컨트롤러.
- 실측 변환에는 `measured_and_verified: true`인 보정 파일이 필요합니다. 보정 도구와 스크립트는 항상 `false`로 저장하고, 사람이 확인한 뒤 바꿉니다.
- 로봇이 보호정지·비상정지 상태면 `maze_executor`는 home으로 자동 이동하지 않습니다.
- `maze_executor` 검증 항목: 도달 범위 850mm, 펜 내림 상태 60mm 점프 금지, 펜 내림 Z 차이 10mm 이내, 올림은 내림보다 5mm 이상 위, 실기 첫 실행 전 `RUN` 확인(`--yes`로 생략).
- 새 위치나 새 설정으로 처음 움직일 때는 펜 Z를 +50mm 올린 리허설 보정으로 먼저 돌립니다(`maze_csv --calibration`).
