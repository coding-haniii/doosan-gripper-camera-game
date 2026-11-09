"""(3단계) 로봇이 CSV 를 '자동으로' 읽어 실행.

  단발 실행 :  python -m robot_games.maze_executor --csv out/maze_coordinates_ROBOT.csv --mode real --execute-motion
  상주(자동) :  python -m robot_games.maze_executor --watch --mode real --execute-motion --yes
               → out/ 에 새 실측 CSV(+메타)가 생기면 감지해서 한 번씩 실행. (비전 프로세스는 CSV 만 만들면 끝)

안전(기존 규칙 유지 + 추가):
  * mock  : 어떤 CSV 든 구동 없이 점검만. SIMULATION_ONLY CSV 는 mock 에서만 사용 가능.
  * real  : --execute-motion  +  default.yaml robot.armed/calibration_confirmed=true  +  메타가 '실측·verified'  +
            CSV 해시 일치  +  Z/단차/범위 검증  +  (--yes 가 없으면) 사람이 RUN 입력.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path

import yaml

from .maze_csv import OUT, ROOT, CSV_NAME, HEADER, sha256, meta_path
from .motion import MockRobot, GuardedDoosan

MAX_STEP_DOWN_MM = 60.0     # 펜 내린 상태 인접 점 최대 간격
MAX_REACH_XY_MM = 850.0     # E0509 도달 900mm 보다 여유 (베이스 기준 수평거리)


class ExecutionRefused(RuntimeError):
    pass


def load_rows(csv_path):
    with open(csv_path, newline="") as f:
        rd = csv.DictReader(f)
        if rd.fieldnames != HEADER:
            raise ExecutionRefused(f"CSV 헤더 불일치: {rd.fieldnames}")
        rows = []
        for r in rd:
            row = {"step": int(r["step"]), "action": r["action"], "pen": r["pen"]}
            for k in ("x", "y", "z", "rx", "ry", "rz"):
                row[k] = float(r[k]) if r[k] != "" else None
            rows.append(row)
    if not rows:
        raise ExecutionRefused("CSV 가 비어 있음")
    return rows


def validate_real(rows, meta, csv_path, max_points):
    """실기 실행 전 검증. 하나라도 실패하면 ExecutionRefused."""
    if meta.get("mode") != "real" or meta.get("simulation_only") or "SIMULATION" in Path(csv_path).name.upper():
        raise ExecutionRefused("시뮬레이션용 CSV 입니다. 실측 보정(real) CSV 만 로봇에 보낼 수 있음")
    if not meta.get("calibration_verified"):
        raise ExecutionRefused("보정 파일이 measured_and_verified=true 가 아님")
    if meta.get("csv_sha256") != sha256(csv_path):
        raise ExecutionRefused("CSV 가 메타 생성 이후 변경됨(해시 불일치)")
    if len(rows) > max_points + 4:
        raise ExecutionRefused(f"점 개수 {len(rows)} > 한계 {max_points}")
    if any(r[k] is None for r in rows for k in ("x", "y", "z", "rx", "ry", "rz")):
        raise ExecutionRefused("x,y,z,rx,ry,rz 중 빈 값 있음 (tool_orientation 누락?)")
    downs = {r["z"] for r in rows if r["pen"] == "down"}
    ups = {r["z"] for r in rows if r["pen"] == "up"}
    if len(downs) != 1 or len(ups) != 1:
        raise ExecutionRefused("펜 내림/올림 Z 가 각각 하나의 값이 아님")
    if min(ups) <= max(downs):
        raise ExecutionRefused("펜 올림 Z 가 내림 Z 보다 높지 않음")
    for r in rows:
        if math.hypot(r["x"], r["y"]) > MAX_REACH_XY_MM:
            raise ExecutionRefused(f"step {r['step']}: 도달 범위 밖 (r={math.hypot(r['x'], r['y']):.0f}mm)")
    for a, b in zip(rows, rows[1:]):
        if a["pen"] == b["pen"] == "down" and math.dist((a["x"], a["y"]), (b["x"], b["y"])) > MAX_STEP_DOWN_MM:
            raise ExecutionRefused(f"step {a['step']}→{b['step']}: 펜을 내린 채 {MAX_STEP_DOWN_MM:.0f}mm 이상 점프")
    seq = [r["pen"] for r in rows]
    if seq[0] != "up" or seq[-1] != "up":
        raise ExecutionRefused("펜 시퀀스가 up 으로 시작/끝나지 않음")


def load_cfg(path=None):
    with open(path or ROOT / "configs" / "default.yaml", encoding="utf8") as f:
        return yaml.safe_load(f)


def execute(csv_path, mode="mock", execute_motion=False, yes=False, cfg_path=None, robot=None, input_fn=input):
    """CSV 1개 실행. 반환: 실행한 행 수. (robot 인자는 테스트용 주입)"""
    csv_path = Path(csv_path)
    mp = meta_path(csv_path)
    if not mp.exists():
        raise ExecutionRefused(f"메타 파일 없음: {mp.name} (maze_csv 로 만든 CSV 만 허용)")
    meta = json.loads(mp.read_text(encoding="utf-8"))
    rows = load_rows(csv_path)
    cfg = load_cfg(cfg_path)
    rc = cfg["robot"]

    if mode == "mock":
        robot = robot or MockRobot()
        print(f"[MOCK] {csv_path.name}: {len(rows)}행 ({'SIMULATION ONLY' if meta.get('simulation_only') else '실측 기준'})")
        robot.run_rows(rows)
        robot.home()
        return len(rows)

    # ---------------- emulator (두산 가상 컨트롤러 → RViz/Gazebo) ----------------
    if mode == "emulator":
        for r in rows:
            if math.hypot(r["x"], r["y"]) > MAX_REACH_XY_MM:
                raise ExecutionRefused(f"step {r['step']}: 도달 범위 밖 (r={math.hypot(r['x'], r['y']):.0f}mm)")
        print(f"[EMU] {csv_path.name}: {len(rows)}행 → 두산 에뮬레이터 (RViz/Gazebo 에서 확인)")
        own = robot is None
        if own:
            from .emulator import EmulatorDoosan
            robot = EmulatorDoosan(rc)           # 가상 컨트롤러가 아니면 여기서 PermissionError
        try:
            robot.run_rows(rows)
        finally:
            try:
                robot.home()
            finally:
                if own and hasattr(robot, "close"):
                    robot.close()
        return len(rows)

    # ---------------- real ----------------
    if not execute_motion:
        raise PermissionError("실기 모드는 --execute-motion 이 필요합니다")
    validate_real(rows, meta, csv_path, int(rc.get("max_path_points", 1500)))
    bb = meta["xy_bbox_mm"]
    print(f"[REAL] {csv_path.name}: {len(rows)}행  XY 범위 x {bb[0]:.0f}~{bb[2]:.0f}, y {bb[1]:.0f}~{bb[3]:.0f} mm")
    print(f"       Z up={max(r['z'] for r in rows):.1f}  down={min(r['z'] for r in rows):.1f}  속도 {rc['speed_mm_s']}mm/s")
    print(f"       시작 {rows[0]['x']:.0f},{rows[0]['y']:.0f}  끝 {rows[-1]['x']:.0f},{rows[-1]['y']:.0f}")
    if not yes:
        print("       펜이 파지되어 있고, 종이 위·주변이 비어 있으며, 비상정지에 손이 닿는지 확인하세요.")
        if input_fn("       실행하려면 RUN 입력 > ").strip() != "RUN":
            raise ExecutionRefused("사용자가 실행을 승인하지 않음")
    own = robot is None
    robot = robot or GuardedDoosan(rc, need_celebration=False)   # armed/calibration_confirmed/홈 자세 게이트는 여기서 검사
    try:
        robot.run_rows(rows)
    except BaseException:
        print("[REAL] 오류/중단 → 홈 복귀 시도", file=sys.stderr)
        raise
    finally:
        try:
            robot.home()
        finally:
            if own and hasattr(robot, "close"):
                robot.close()
    return len(rows)


def watch(out_dir=OUT, mode="real", execute_motion=False, yes=False, cfg_path=None, poll_s=0.5, once=False, max_idle_s=None, csv_name=None):
    """out/ 의 실측 CSV 를 감시. 메타(.meta.json)가 마지막에 원자적으로 생기므로 '쓰다 만 파일'은 읽지 않는다.
    실행 완료 해시는 <csv>.done 에 기록해 같은 파일을 두 번 실행하지 않는다."""
    out_dir = Path(out_dir)
    csv_path = out_dir / (csv_name or CSV_NAME["real" if mode == "real" else "mock"])
    print(f"[WATCH] {csv_path} 감시 시작 (mode={mode}) — Ctrl+C 로 종료")
    t0 = time.time()
    ran = 0
    while True:
        mp = meta_path(csv_path)
        if csv_path.exists() and mp.exists():
            try:
                meta = json.loads(mp.read_text(encoding="utf-8"))
                h = meta.get("csv_sha256")
                done = Path(str(csv_path) + ".done")
                failed = Path(str(csv_path) + ".failed")
                seen = [p.read_text().strip() for p in (done, failed) if p.exists()]
                if h and h not in seen and h == sha256(csv_path):
                    print(f"[WATCH] 새 CSV 감지 ({h[:8]}) → 실행")
                    try:
                        execute(csv_path, mode, execute_motion, yes, cfg_path)
                        done.write_text(h)
                        ran += 1
                    except Exception as e:                      # 실패한 해시는 재시도하지 않음(사람이 확인)
                        failed.write_text(h)
                        print(f"[WATCH] 실행 거부/실패: {e}", file=sys.stderr)
                    if once:
                        return ran
            except (json.JSONDecodeError, FileNotFoundError):
                pass                                            # 쓰는 중 → 다음 주기에 재시도
        if max_idle_s is not None and time.time() - t0 > max_idle_s:
            return ran
        time.sleep(poll_s)


def main(argv=None):
    p = argparse.ArgumentParser(description="로봇이 maze CSV 를 읽어 실행 (mock 기본)")
    p.add_argument("--mode", choices=["mock", "emulator", "real"], default="mock",
                   help="mock=구동없음 / emulator=두산 가상 컨트롤러(RViz·Gazebo) / real=실물")
    p.add_argument("--real-csv", action="store_true", help="emulator 에서 실측 CSV(maze_coordinates_ROBOT.csv)를 리허설")
    p.add_argument("--csv")
    p.add_argument("--watch", action="store_true", help="out/ 에 새 CSV 가 생기면 자동 실행")
    p.add_argument("--execute-motion", action="store_true", help="실기 구동 추가 게이트")
    p.add_argument("--yes", action="store_true", help="RUN 입력 생략 (default.yaml 의 armed/confirmed 가 이미 사람의 승인)")
    p.add_argument("--config")
    a = p.parse_args(argv)
    try:
        if a.watch:
            watch(OUT, a.mode, a.execute_motion, a.yes, a.config,
                  csv_name=CSV_NAME["real" if (a.mode == "real" or a.real_csv) else "mock"])
        else:
            csv_path = a.csv or OUT / CSV_NAME["real" if (a.mode == "real" or a.real_csv) else "mock"]
            execute(csv_path, a.mode, a.execute_motion, a.yes, a.config)
    except (ExecutionRefused, PermissionError, ValueError) as e:
        print(f"[REFUSED] {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n중단")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
