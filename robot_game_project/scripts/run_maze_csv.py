"""기존 이름 호환용. test_maze_coordinates.py 와 동일 동작."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_games.maze_csv import main

if __name__ == "__main__":
    sys.exit(main())
