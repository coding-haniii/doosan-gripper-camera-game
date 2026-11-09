# 카메라 종이 미로찾기 패치 (PC MOCK)

기존 `robot_game_project` 폴더에 덮어씁니다. 기존 `maze.py`, `app.py`는 변경하지 않습니다.

```bash
cd /home/khe/ros2_ws/src/robot_game_project
unzip -o ~/Downloads/maze_camera_patch.zip
chmod +x scripts/run_paper_maze.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_paper_maze.py
```

## 1. 웹캠 종이 미로

```bash
bash scripts/run_paper_maze.sh --source camera --camera-index 0
```

1. 카메라 고정, 굵은 검정 벽과 흰색 통로가 있는 미로 종이를 올립니다. **시작·끝은 사람이 지정**합니다. 햇빛 반사, 손과 그림자가 없게 하세요.
2. 웹캠 창에서 **Space**를 눌러 촬영. **Q**는 취소.
3. 캡처된 화면에서 종이의 **미로판 경계** 네 모서리를 **왼쪽 위 → 오른쪽 위 → 오른쪽 아래 → 왼쪽 아래** 순으로 클릭, **Space** 확인. **R** 초기화.
4. 보정된 정사각형 화면에서 **시작점 → 도착점**을 클릭하고 **Space** 확인.
5. `out/paper_maze_rectified.png`, `out/paper_maze_walkable.png`, `out/paper_maze_solved.png` 와 `out/paper_maze_pen_plan.json` 확인.

```bash
xdg-open out/paper_maze_solved.png
```

**중요:** 검정벽/흰통로 미로를 대상으로 합니다. 일반 인쇄 미로에서 벽 밖 여백이 흰 통로와 이어져 있다면 테두리나 오인식으로 인한 오경로가 생길 수 있습니다. 시작/끝 클릭을 지원하지만 **자동 인식은 미구현**입니다. 해결 경로는 반드시 눈으로 검사해야 합니다. `--threshold 160`, `--clearance 2` 값을 바꿔보세요. 검정 통로/흰 벽은 `--invert`.

## 2. PC만으로 반복 테스트

```bash
bash scripts/run_paper_maze.sh --source demo --seed 11
bash scripts/run_paper_maze.sh --source demo --seed 25
```

## 3. 외부 이미지

```bash
bash scripts/run_paper_maze.sh --source image --image /path/to/maze.png
```

이미지 창에서 시작과 끝을 클릭. 명시 좌표가 있으면 `--start 20 20 --goal 250 250` 사용.

## 4. 실제 로봇 및 그리퍼

현재 버전은 **실제 두산 E0509나 RH-P12-RN(A)를 움직이지 않습니다**. `paper_maze_pen_plan.json` 파일은 이미지 내부 0~1 좌표로 표현한 **펜 올림/접근/펜 내림/따라 그리기/펜 올림** 순서로, **로봇 관절·TCP 좌표가 아니며 실행 불가 표시**를 포함합니다. 좌표만 넣어 실물에서 실행하지 마세요.

실제 종이 위 선을 그리려면 카메라-로봇 좌표 보정(ArUco/체커보드 및 로봇 base 기준), 고정 종이/테이블 평면 확인, RH-P12-RN(A) 그리퍼 통신과 펜 파지, 장착된 펜 TCP/접촉력, 수직 펜 올림 높이, 충돌 및 특이점 검사, 속도 및 사람 접근 안전설비, 보호정지/비상정지 검증을 거친 뒤, 별도로 검증한 ROS 2 motion executor를 추가해야 합니다.

액션카드 2번을 이 카메라 미로 모드에 자동 연결하는 디스패처는 **아직 연동되지 않았으며**, 위 스크립트로 직접 실행합니다.
