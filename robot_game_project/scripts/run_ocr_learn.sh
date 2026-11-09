#!/usr/bin/env bash
# OCR 로그 학습 도구 실행 래퍼 (어느 폴더에서 불러도 프로젝트 루트에서 실행)
#   scripts/run_ocr_learn.sh labels            # 정답 확인용 CSV 생성/갱신 (configs/ocr_labels.csv)
#   scripts/run_ocr_learn.sh learn             # configs/ocr_rules.json 생성
#   scripts/run_ocr_learn.sh replay --cv 5     # 개선 효과 점검
# 로그를 안 주면 ocr_logs/*.txt 와 out/ocr_log.jsonl 을 자동 사용한다.
set -euo pipefail
cd "$(dirname "$0")/.."
exec python3 -m robot_games.ocr_learn "$@"
