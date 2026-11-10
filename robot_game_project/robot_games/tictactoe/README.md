# 틱택토 카메라 미리보기

터미널에서 프로젝트 폴더로 이동한 뒤 실행합니다.

```bash
cd /home/code/Documents/workspace/doosan-gripper-camera-game/robot_game_project
python3 robot_games/tictactoe/run_camera.py
```

실행기는 RealSense의 RGB 영상을 먼저 확인합니다. 읽을 수 없으면 설정된 기본 카메라로 전환합니다.

1. 카메라 창에서 `TIC` 카드를 초록색 가이드 중앙에 놓고 **Space**를 누릅니다.
2. 카드를 치우고 바깥 테두리가 있는 빈 3×3 종이 판 전체를 비춥니다. 판을 잠시 고정하면 자동으로 인식하고 선공을 정합니다.
3. 로봇 차례에는 노란색으로 예정 칸을 표시한 뒤 파란색 X를 화면에 그립니다. 사람 차례에는 종이의 빈칸에 O 하나를 그리고 손을 치웁니다. O가 인식되면 자동으로 다음 차례로 넘어갑니다.
4. 게임이 끝나면 승자 또는 무승부 창이 뜹니다. **Q**로 닫습니다.

결과 이미지는 프로젝트의 `out/tictactoe_result.png`에 저장됩니다. 이 실행 명령은 실제 로봇 대신 화면으로 게임을 확인하는 미리보기 모드입니다.
