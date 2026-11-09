#!/usr/bin/env bash
set -euo pipefail

# 기존 run_paper_maze.sh 내부의 미로 생성, 비전 처리, 로봇 경로 연동 로직...
echo "Running paper maze pipeline..."

cd "$(dirname "$0")/.."
exec .venv/bin/python -m robot_games.paper_maze "$@"


# 파이프라인 완료 후 run_maze_csv.py 자동 실행 추가
echo "Paper maze finished. Launching run_maze_csv.py..."
python3 -m robot_games.run_maze_csv "$@"
