"""미로 게임 연속 실행:  (1) 카메라 → 미로 풀이 JSON  →  (2) 로봇 좌표 CSV  →  (3) 로봇이 CSV 를 읽어 그림 → 홈.

  bash scripts/run_maze_game.sh --source camera                         # PC mock (구동 없음, 전체 흐름 점검)
  bash scripts/run_maze_game.sh --source camera --mode real --execute-motion   # 실기 (실측 보정 + 게이트 필요)
  bash scripts/run_maze_game.sh --source camera --mode real --execute-motion --handoff
        → (1)(2)까지만 하고 CSV 를 남김. 다른 터미널에서 `run_maze_robot.sh --watch` 가 CSV 를 자동으로 집어 실행.
"""
from __future__ import annotations
import argparse
import sys
import time

from . import paper_maze
from .maze_csv import PLAN_DEFAULT, CSV_NAME, OUT, CalibrationError, convert
from .maze_executor import ExecutionRefused, execute


def run(argv=None) -> int:
    p = argparse.ArgumentParser(description="미로: 비전→JSON→CSV→로봇 연속 실행")
    p.add_argument("--mode", choices=["mock", "emulator", "real"], default="mock",
                   help="mock=구동없음 / emulator=두산 가상 컨트롤러(RViz·Gazebo) / real=실물")
    p.add_argument("--use-real-calibration", action="store_true",
                   help="emulator 에서 실측 보정으로 만든 CSV 를 리허설 (실기 전 최종 점검)")
    p.add_argument("--execute-motion", action="store_true")
    p.add_argument("--yes", action="store_true", help="실기에서 RUN 입력 생략")
    p.add_argument("--handoff", action="store_true", help="CSV 만 만들고 종료(로봇 감시 프로세스가 자동 실행)")
    p.add_argument("--config")
    p.add_argument("--source", choices=["camera", "image", "demo"], default="camera")
    p.add_argument("--camera-index", type=int, default=None, help="생략하면 RealSense 컬러 카메라를 자동으로 찾음")
    p.add_argument("--image")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--start", type=int, nargs=2)
    p.add_argument("--goal", type=int, nargs=2)
    p.add_argument("--invert", action="store_true")
    p.add_argument("--threshold", type=int, default=160)
    p.add_argument("--clearance", type=int, default=2)
    a = p.parse_args(argv)

    if a.mode == "real" and not a.execute_motion and not a.handoff:
        print("[REFUSED] 실기 모드는 --execute-motion 이 필요합니다", file=sys.stderr)
        return 2

    # 실기라면 비전을 시작하기 전에 보정 파일부터 확인 (사진 다 찍고 나서 실패하는 일 방지)
    csv_mode = "real" if (a.mode == "real" or (a.mode == "emulator" and a.use_real_calibration)) else "mock"
    if csv_mode == "real":
        from robot_games.maze_csv import CALIB, load_calibration
        try:
            load_calibration(CALIB["real"], "real")
        except CalibrationError as e:
            print(f"[REFUSED] 실측 보정이 준비되지 않음: {e}", file=sys.stderr)
            return 2

    # ---------- (1) 비전: 미로 풀이 + JSON ----------
    print("=== (1/3) 비전: 미로 인식·풀이 ===")
    t0 = time.time() - 1.0
    argv1 = ["--source", a.source, "--seed", str(a.seed),
             "--threshold", str(a.threshold), "--clearance", str(a.clearance), "--robot-mode", "mock"]
    if a.camera_index is not None: argv1 += ["--camera-index", str(a.camera_index)]
    if a.image: argv1 += ["--image", a.image]
    if a.start: argv1 += ["--start", *map(str, a.start)]
    if a.goal: argv1 += ["--goal", *map(str, a.goal)]
    if a.invert: argv1.append("--invert")
    if a.source != "demo": argv1.append("--approve")        # 풀이 경로를 눈으로 확인 후 승인
    try:
        plan_path = paper_maze.main(argv1)
    except KeyboardInterrupt as e:
        print(f"[ABORT] {e}")
        return 130
    plan_path = plan_path or PLAN_DEFAULT
    if not plan_path.exists() or plan_path.stat().st_mtime < t0:
        print("[ERROR] 이번 실행에서 새 JSON 이 만들어지지 않음 (이전 파일 사용 금지)", file=sys.stderr)
        return 1

    # ---------- (2) 변환: JSON → CSV ----------
    print("=== (2/3) 변환: 비전 좌표 → 로봇 좌표 CSV ===")
    try:
        meta = convert(plan_path, csv_mode)
    except (CalibrationError, ValueError) as e:
        print(f"[ERROR] 변환 실패: {e}", file=sys.stderr)
        return 2
    print(f"CSV {meta['rows']}행 → {meta['csv']}")
    if a.handoff:
        print("[HANDOFF] CSV 준비 완료. 로봇 감시 프로세스가 자동으로 실행합니다.")
        return 0

    # ---------- (3) 로봇: CSV 자동 읽기 → 실행 → 홈 ----------
    print("=== (3/3) 로봇: CSV 읽기 → 그리기 → 홈 ===")
    try:
        n = execute(meta["csv"], a.mode, a.execute_motion, a.yes, a.config)
    except (ExecutionRefused, PermissionError, ValueError) as e:
        print(f"[REFUSED] {e}", file=sys.stderr)
        return 2
    print(f"[DONE] {n}행 실행 완료. 홈 복귀.")
    return 0


if __name__ == "__main__":
    sys.exit(run())
