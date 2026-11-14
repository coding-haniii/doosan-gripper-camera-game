from pathlib import Path

import cv2

from robot_games.paper_maze import find_openings

IMG = Path(__file__).with_name('data_maze_corner_openings.png')


def test_corner_openings_are_found_once_each():
    # 2026-11-14 실기 미로: 입구는 왼쪽 위 모서리, 출구는 오른쪽 아래 모서리, 오른쪽 바깥 벽이 짧음.
    # 예전에는 안쪽 벽을 바깥 벽으로 골라 시작점이 150px 밖으로 나가 CSV 변환이 거부됐다.
    board = cv2.imread(str(IMG))
    s, g, (x0, y0, x1, y1), s_ext, g_ext = find_openings(board)
    h, w = board.shape[:2]
    assert s[0] < w * 0.2 and s[1] < h * 0.2          # 왼쪽 위
    assert g[0] > w * 0.8 and g[1] > h * 0.8          # 오른쪽 아래
    assert x1 > w * 0.95                              # 바깥 벽 = 맨 오른쪽 선
    for p in (s_ext[0], g_ext[1]):                    # 바깥 점이 보드에서 크게 벗어나지 않음
        assert -60 < p[0] < w + 60 and -60 < p[1] < h + 60
