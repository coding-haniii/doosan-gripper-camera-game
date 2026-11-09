# PATCH: 액션카드 OCR 로그 학습

카드 사진을 대량으로 모아 재학습하는 대신, **실행 로그에서 키워드별 오인식 패턴을 뽑아 규칙 파일로 만들고 vision이 그 규칙을 쓰는** 방식입니다.

## 1. 파일을 놓을 위치 (이 폴더 구조 그대로 프로젝트 루트에 덮어 넣기)

```
robot_game_project/
├── ocr_logs/                          ← (신규) 터미널 로그 보관. 비전_텍스트_인식_오류_로그.txt 를 여기로 이동
├── configs/
│   ├── ocr_seed_rules.json            ← (신규) 사용자 정리 오류 목록(종료/송료/승도/송도…). 직접 편집
│   ├── ocr_labels.csv                 ← (신규) 실행별 정답. truth 열을 사람이 확인
│   └── ocr_rules.json                 ← (신규, 자동 생성) 학습 결과. vision 이 읽음
├── robot_games/
│   ├── vision.py                      ← (교체) 이미지 단계만. 텍스트 로직은 ocr_rules 로 분리
│   ├── ocr_rules.py                   ← (신규) 텍스트 매칭 + 학습 규칙 적용 + 투표(CardVoter)
│   ├── ocr_learn.py                   ← (신규) 로그 파싱 / 라벨 CSV / 학습 / replay·교차검증
│   ├── cards.py                       ← (교체, 선택) 한글·사진형 테스트 카드 생성
│   └── app.py                         ← 덮어쓰지 말고 scripts/patch_app_truth.py 로 3줄만 추가
├── scripts/
│   ├── patch_app_truth.py             ← (신규) app.py 에 --truth 옵션만 안전하게 추가
│   └── run_ocr_learn.sh               ← (신규) 학습 도구 실행 래퍼
├── tests/test_ocr_rules.py            ← (신규) 단위 테스트 12개 (카메라/tesseract 불필요)
└── out/ocr_learn_report.md            ← (참고) 현재 로그 기준 학습 리포트 (이후 learn 할 때마다 갱신)
```

적용 순서
```bash
cd ~/ros2_ws/src/robot_game_project
mkdir -p ocr_logs && mv 비전_텍스트_인식_오류_로그.txt ocr_logs/      # 로그 이동 (이름은 바꾸지 말 것)
# 위 폴더 구조대로 파일 복사 후
chmod +x scripts/run_ocr_learn.sh scripts/patch_app_truth.py
python3 scripts/patch_app_truth.py          # app.py 백업(app.py.bak) 후 --truth 만 추가. 여러 번 실행해도 안전
python3 -m unittest tests.test_ocr_rules -v # 12개 OK 확인
```
의존성 추가 없음(표준 라이브러리만 사용). tesseract 한글 데이터는 이미 필요: `sudo apt install tesseract-ocr tesseract-ocr-kor`

## 2. 서로 호출되는 관계

```
app.py ──► vision.py ──► ocr_rules.py ◄── ocr_learn.py
              │              │                │
              │              └ 읽음: configs/ocr_rules.json
              └ 씀: out/debug/*.png, out/ocr_log.jsonl      ocr_learn 이 읽음: ocr_logs/*.txt, out/ocr_log.jsonl,
                                                             configs/ocr_labels.csv, configs/ocr_seed_rules.json
                                                             ocr_learn 이 씀:   configs/ocr_rules.json, out/ocr_learn_report.md
```
- 모든 경로는 `robot_games/` 의 상위 폴더(프로젝트 루트) 기준 절대 경로라서 **어느 폴더에서 실행해도** 같은 `out/`, `configs/` 를 씁니다.
- `vision.py` 는 예전 이름(`CARD_MAP`, `card_from_text`, `_normalize`, `_lookup`, `_prepare_card_images`, `recognize_card(_result)`, `tic_board_from_image`)을 그대로 제공해서 기존 코드/테스트와 호환됩니다.
- `configs/ocr_rules.json` 이 없거나 읽을 수 없으면 기본 규칙으로 동작합니다(오류 없이).

## 3. 사용법

```bash
# (1) 정답을 알고 보여줄 때 --truth 를 붙이면 정답이 로그에 같이 기록됨 (데모 때는 생략)
python3 -m robot_games.app --game camera --truth exit

# (2) 정답 확인용 CSV 생성/갱신 → configs/ocr_labels.csv 의 truth 열 확인 (skip = 제외)
scripts/run_ocr_learn.sh labels

# (3) 학습 → configs/ocr_rules.json, out/ocr_learn_report.md
scripts/run_ocr_learn.sh learn

# (4) 효과 점검: 학습에 안 쓴 run 으로 정답/오인식/실패 측정
scripts/run_ocr_learn.sh replay --cv 5
```
새 오인식을 발견하면 `configs/ocr_seed_rules.json` 에 한 줄 추가 → (3) 다시 실행.

## 4. 현재 로그 기준 결과와 주의

| 구분 | 정답 | 오인식 | 실패 |
|---|---|---|---|
| 기본 규칙 | 11/18 (61%) | 0 | 7 |
| 로그만으로 학습(seed 없음, 교차검증) | 13/18 (72%) | 0 | 5 |
| 로그 + seed(사용자 메모), 교차검증 | 15/18 (83%) | 0 | 3 |

- 평가 시도가 18개뿐이라 방향만 보여주는 수치입니다. exit 정답이 3개뿐이라 `--truth exit` 로 10회 이상 모으는 것을 권장합니다.
- `configs/ocr_labels.csv`: 제가 글자 근거로 truth 를 채운 4개 run(note 에 `assist:` 표시)을 확인하세요. `...:2468`(8번 연속 실패)은 어떤 카드였는지 알 수 없어 비워 뒀습니다.
- 글자 형태가 완전히 깨진 경우(`Ui`, `pew` 등)는 규칙이 아니라 촬영(초점·조명·각도) 문제입니다.
- `.gitignore` 권장: `out/ocr_log.jsonl`, `out/debug/`, `configs/*.bak`, `robot_games/app.py.bak`
