# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 개요

팀 프로젝트(`coding-haniii/doosan-gripper-camera-game`)입니다. 두산 E0509 로봇팔이 테이블 위 RealSense 카메라로 종이를 보고, 그리퍼에 미리 물린 펜으로 미로·틱택토를 그립니다. 코드는 모두 `robot_game_project/` 안에 있고, 사용자 문서와 메시지는 한국어입니다.

- **팀원 코드가 기준입니다.** 지금 단계에서는 없는 기능을 새로 만들지 않습니다. 실기에서 꼭 필요하다고 확인된 수정만 팀원 파일 구조 그대로 몇 줄 단위로 넣습니다. 범위를 넓히기 전에 사용자에게 묻습니다.
- 작업 브랜치는 `jmc`(팀 `dev` 기준)입니다. 예전 개인 저장소 작업(감독 루프, 그리퍼 펜 거치대, 예전 구조 테스트 등)은 `legacy-robot_games-jmc` 브랜치에 보관되어 있습니다. 필요할 때 거기서 가져옵니다.
- 현재 실기로 확인된 것은 **미로 그리기**(2026-11-11)입니다. 틱택토는 팀원 카메라 미리보기(`robot_games/tictactoe/`, mock 전용)까지만 있습니다.
- 설비 값, 보정, 실기 절차와 함정은 [robot_game_project/docs/REAL_CELL.md](robot_game_project/docs/REAL_CELL.md)에 있습니다.

## 명령어 (`robot_game_project/`에서)

```bash
python3 -m venv .venv && .venv/bin/pip install numpy "opencv-python>=4.8,<5" PyYAML "pytest>=8,<10"   # requirements.txt 는 ROS 환경 freeze 라 쓰지 않음
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -p no:cacheprovider tests/   # 보정 도구 테스트
.venv/bin/python -m robot_games.maze_game --source demo                                  # 미로 mock (카메라·로봇 없음)
.venv/bin/python scripts/preview_maze_path.py                                            # 로봇 없이 실기 경로를 사진 위에 그려 확인
.venv/bin/python -m robot_games.maze_game --source camera --mode real --execute-motion   # 실기 (bringup 필요, REAL_CELL.md)
.venv/bin/python -m robot_games.calib.calibrate view|markers|camera-robot|poses|gripper  # 재보정 도구
```

실기·에뮬레이터 명령은 `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash` 한 셸에서 실행합니다. 이 PC에는 두산 에뮬레이터(`doosanrobot/dsr_emulator` Docker)도 있습니다.

## 구조

- **미로 파이프라인**(팀원): `maze_game.run` → `paper_maze`(촬영 → 보드 원근 보정 → 미로 크롭 → 풀이 → 펜 계획 JSON, 0~1 정규화) → `maze_csv`(보드 선형 보정 `origin + u·x_span + v·y_span` → 로봇 좌표 CSV + `.meta.json` 해시) → `maze_executor`(검증 후 `GuardedDoosan.run_rows`).
- **실기용으로 넣은 수정**(`[실기]` 주석으로 표시)
  - `paper_maze`: 보드 = ArUco 마커 1,3,2,0 중심(`vision.board_marker_ids`)이고, 마커가 다 안 보이면 종이 자동 검출을 씁니다. 크롭은 벽 덩어리를 모두 포함하고, 정규화에 크롭 오프셋을 반영합니다. `find_openings`로 입구·출구 바깥에서 시작·끝을 잡고, `solve_centered`로 통로 가운데를 지납니다.
  - `maze_csv`: `pen_contact_plane`(테이블 기울기)과 `pen_press_mm`으로 행마다 펜 Z를 계산합니다.
  - `motion.GuardedDoosan`: `DSR_ROBOT2` 별칭, REAL 컨트롤러 확인, 속도 배율 100%, 도착 확인, `min_pen_z_base_mm` 하한. 펜 내린 구간은 `movesx`로 그립니다(50mm/s, 상한 60).
  - `dsr_preflight.py`: 팀원 커밋에서 빠져 있던 파일을 추가했습니다. `camera_select.py`는 실행할 때마다 RealSense 컬러 노드를 찾습니다.
- **재보정 도구 `robot_games/calib/`**(예전 저장소에서 옮김): 자체 `motion.py`(세션·조그 이동), `pen_plan.py`, `jog.py`, `gripper.py`(플랜지 Modbus)를 씁니다. 팀원 `robot_games/motion.py`와는 별개입니다. 결과는 `configs/field/`에 저장됩니다. `scripts/make_maze_calibration.py`가 마커 실측값으로 `configs/my_verified_calibration_real.json`을 다시 만듭니다.

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
