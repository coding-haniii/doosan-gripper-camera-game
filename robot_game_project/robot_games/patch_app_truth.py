#!/usr/bin/env python3
"""robot_games/app.py 에 '--truth' 옵션(학습용 정답 지정)만 안전하게 끼워 넣는다.

app.py 전체를 덮어쓰지 않으므로 기존에 수정해 둔 내용(paper_tictactoe, maze_game_123 연동 등)은 그대로 유지된다.
여러 번 실행해도 중복 적용되지 않는다. 적용 전 robot_games/app.py.bak 으로 백업한다.

사용:  (프로젝트 루트에서)  python3 scripts/patch_app_truth.py
"""
import re
import shutil
import sys
from pathlib import Path

APP = Path(__file__).resolve().parents[1] / 'robot_games' / 'app.py'
IMPORT = 'from . import vision as vision_mod'
ARG = ("    parser.add_argument('--truth',choices=['tictactoe','maze','exit'],"
       "help='지금 보여주는 카드의 정답(학습용). out/ocr_log.jsonl 에 함께 기록됨')")
APPLY = '    if args.truth: vision_mod.TRUTH=args.truth'


def main():
    if not APP.exists():
        sys.exit(f'app.py 를 찾을 수 없습니다: {APP}')
    text = APP.read_text(encoding='utf8')
    lines = text.split('\n')
    done, todo = [], []

    # 1) import
    if 'vision_mod' not in text:
        for i, ln in enumerate(lines):
            if re.match(r'\s*from \.vision import', ln):
                lines.insert(i, IMPORT)
                done.append('import 추가')
                break
        else:
            todo.append(f"app.py 상단 import 부분에 `{IMPORT}` 를 직접 추가")
    # 2) 인자
    if "'--truth'" not in text:
        for i, ln in enumerate(lines):
            if "'--card-image'" in ln and 'add_argument' in ln:
                lines.insert(i + 1, ARG)
                done.append('--truth 인자 추가')
                break
        else:
            todo.append("argparse 인자 목록에 `--truth` 한 줄을 직접 추가 (ARG 변수 내용 참고)")
    # 3) 적용
    if 'vision_mod.TRUTH' not in text:
        for i, ln in enumerate(lines):
            if 'parser.parse_args()' in ln:
                lines.insert(i + 1, APPLY)
                done.append('TRUTH 적용 추가')
                break
        else:
            todo.append(f"`args=parser.parse_args()` 바로 다음 줄에 `{APPLY.strip()}` 를 직접 추가")

    if done:
        shutil.copy(APP, APP.with_suffix('.py.bak'))
        APP.write_text('\n'.join(lines), encoding='utf8')
    if done:
        print('적용:', ', '.join(done))
    elif not todo:
        print('이미 모두 적용되어 있음')
    for t in todo:
        print('수동 작업 필요:', t)
    return 1 if todo else 0


if __name__ == '__main__':
    sys.exit(main())
