"""(2단계) 비전 기준 펜 계획(JSON, 0~1 정규화) → 로봇 베이스 좌표(mm) CSV 변환.

scripts/run_maze_csv.py, scripts/test_maze_coordinates.py, maze_game(연속 실행)이 모두 이 모듈을 쓴다.

  mock : configs/my_verified_calibration.json       → out/maze_coordinates_SIMULATION_ONLY.csv  (로봇 전송 금지)
  real : configs/my_verified_calibration_real.json  → out/maze_coordinates_ROBOT.csv            (실측 + verified 필수)

CSV 옆에 <csv>.meta.json 을 같이 쓴다(모드/보정파일/해시). 로봇 실행기는 이 메타로 '실측 기준 CSV'만 받아들인다.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out"
PLAN_DEFAULT = OUT / "paper_maze_pen_plan.json"
CALIB = {"mock": ROOT / "configs" / "my_verified_calibration.json",
         "real": ROOT / "configs" / "my_verified_calibration_real.json"}
CSV_NAME = {"mock": "maze_coordinates_SIMULATION_ONLY.csv", "real": "maze_coordinates_ROBOT.csv"}
HEADER = ["step", "action", "x", "y", "z", "pen", "rx", "ry", "rz"]
MAX_BOARD_SIDE_MM = 300.0


class CalibrationError(ValueError):
    pass


def sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def meta_path(csv_path) -> Path:
    return Path(str(csv_path) + ".meta.json")


def load_calibration(path, mode: str) -> dict:
    c = json.loads(Path(path).read_text(encoding="utf-8"))
    for k in ("origin_base_mm", "x_span_base_mm", "y_span_base_mm"):
        v = c.get(k)
        if not (isinstance(v, list) and len(v) == 3):
            raise CalibrationError(f"{Path(path).name}: '{k}' 가 3차원 [x,y,z] mm 값이 아님 ({v!r})")
    if mode == "real":
        missing = [k for k in ("pen_contact_z_base_mm", "pen_up_z_base_mm", "tool_orientation_deg") if c.get(k) is None]
        if missing:
            raise CalibrationError(f"{Path(path).name}: 실측값 누락 {missing}")
        if c.get("measured_and_verified") is not True:
            raise CalibrationError(f"{Path(path).name}: 'measured_and_verified' 가 true 가 아님 "
                                   "(현장에서 실측·검증 후 직접 true 로 바꾸세요)")
        if len(c["tool_orientation_deg"]) != 3:
            raise CalibrationError("tool_orientation_deg 는 [rx,ry,rz] 3개여야 함")
        if float(c["pen_up_z_base_mm"]) <= float(c["pen_contact_z_base_mm"]):
            raise CalibrationError("pen_up_z 는 pen_contact_z 보다 커야 함(위쪽이 +Z 인 수평 테이블 가정)")
        for k in ("x_span_base_mm", "y_span_base_mm"):
            n = sum(float(x) ** 2 for x in c[k]) ** .5
            if n > MAX_BOARD_SIDE_MM:
                raise CalibrationError(f"{k} 길이 {n:.0f}mm > 한계 {MAX_BOARD_SIDE_MM:.0f}mm")
    return c


def convert(plan_path=PLAN_DEFAULT, mode="mock", calib_path=None, out_csv=None) -> dict:
    """JSON → CSV(+meta) 변환. 반환: meta dict (meta['csv'] = CSV 경로)"""
    if mode not in CALIB:
        raise ValueError("mode 는 mock|real")
    plan_path = Path(plan_path)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan.get("reference_frame") != "rectified_maze_image":
        raise ValueError(f"예상치 못한 plan 좌표계: {plan.get('reference_frame')}")
    calib_path = Path(calib_path or CALIB[mode])
    cal = load_calibration(calib_path, mode)
    origin, xs, ys = cal["origin_base_mm"], cal["x_span_base_mm"], cal["y_span_base_mm"]
    z_up, z_dn = cal.get("pen_up_z_base_mm"), cal.get("pen_contact_z_base_mm")
    rpy = cal.get("tool_orientation_deg") or [None, None, None]

    rows = []
    for step, item in enumerate(plan["pen_sequence"]):
        u, v = item["xy"]
        if not (-1e-6 <= u <= 1 + 1e-6 and -1e-6 <= v <= 1 + 1e-6):
            raise ValueError(f"step {step}: 정규화 좌표가 0~1 밖 ({u},{v})")
        x, y, z = (origin[i] + u * xs[i] + v * ys[i] for i in range(3))
        if z_up is not None and z_dn is not None:          # 실측 Z 가 있으면 펜 상태별 고정 높이
            z = float(z_up) if item["pen"] == "up" else float(z_dn)
        rows.append([step, item["action"], round(x, 3), round(y, 3), round(z, 3), item["pen"],
                     *(("" if r is None else r) for r in rpy)])

    out_csv = Path(out_csv or OUT / CSV_NAME[mode])
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_csv.with_suffix(".csv.tmp")                   # 원자적 쓰기: 로봇이 쓰다 만 파일을 읽지 않도록
    with tmp.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        w.writerows(rows)
    os.replace(tmp, out_csv)

    meta = {
        "mode": mode,
        "simulation_only": mode == "mock",
        "calibration_file": str(calib_path.name),
        "calibration_verified": bool(cal.get("measured_and_verified")),
        "plan_file": str(plan_path.name), "plan_sha256": sha256(plan_path),
        "csv": str(out_csv), "csv_sha256": sha256(out_csv), "rows": len(rows),
        "xy_bbox_mm": [min(r[2] for r in rows), min(r[3] for r in rows), max(r[2] for r in rows), max(r[3] for r in rows)],
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    tmpm = meta_path(out_csv).with_suffix(".tmp")
    tmpm.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    os.replace(tmpm, meta_path(out_csv))                    # 메타가 마지막 = '준비 완료' 신호
    return meta


def main(argv=None, default_mode="mock"):
    p = argparse.ArgumentParser(description="미로 펜 계획 JSON → 로봇 좌표 CSV")
    p.add_argument("--mode", choices=["mock", "real"], default=default_mode)
    p.add_argument("--plan", default=str(PLAN_DEFAULT))
    p.add_argument("--calibration")
    p.add_argument("--out")
    a = p.parse_args(argv)
    try:
        meta = convert(a.plan, a.mode, a.calibration, a.out)
    except (CalibrationError, ValueError, FileNotFoundError) as e:
        print(f"[ERROR] 변환 실패: {e}", file=sys.stderr)
        return 2
    rows = list(csv.reader(open(meta["csv"])))[1:]
    tag = "[SIMULATION ONLY]" if meta["simulation_only"] else "[ROBOT-CALIBRATED]"
    print(f"{tag} {meta['rows']} coordinates generated")
    print(f"First: {rows[0]}\nLast:  {rows[-1]}\nSaved: {meta['csv']}")
    print("[SAFETY] 시뮬레이션 좌표는 로봇에 보내지 마세요" if meta["simulation_only"]
          else "[SAFETY] 실측 보정 좌표. 실행 전 사람이 경로·주변을 확인하세요")
    return 0


if __name__ == "__main__":
    sys.exit(main())
