"""Action-card OCR and simple printed-board vision helpers.

v3: 이미지 단계(카드 검출 -> 영역별 OCR)만 이 파일에 둔다.
    텍스트 매칭/학습 규칙/투표는 ocr_rules.py 로 분리 (로그만으로 재현·학습 가능: ocr_learn.py).
디버그: CARD_DEBUG=1(기본) 이면 OCR 결과 출력 + out/debug/ 이미지 + out/ocr_log.jsonl 저장. CARD_DEBUG=0 이면 끔.
라벨 수집: CARD_TRUTH=maze (또는 app.py --truth maze) 로 정답을 지정하면 jsonl 에 정답이 함께 기록된다.
기타: TESSERACT_CMD 로 tesseract 경로, OCR_RULES 로 규칙 파일 경로, OCR_JSONL 로 jsonl 경로(0 이면 끔).
"""
import os
import re
import shutil
import time
import json
from pathlib import Path

import cv2
import numpy as np

# 텍스트 단계는 ocr_rules.py 에 있다. 기존 이름(CARD_MAP, card_from_text 등)은 그대로 쓸 수 있게 다시 내보낸다.
from .ocr_rules import (CARD_MAP, ROOT, CardVoter, base_lookup, card_from_text, load_rules,  # noqa: F401
                        normalize as _normalize, text_kind)

DEBUG = os.environ.get('CARD_DEBUG', '1') != '0'
# 프로젝트 루트(robot_games 의 상위 폴더) 기준 절대 경로: 어느 폴더에서 실행해도 같은 out/ 에 저장된다.
DEBUG_DIR = ROOT / 'out' / 'debug'
JSONL_PATH = os.environ.get('OCR_JSONL') or str(ROOT / 'out' / 'ocr_log.jsonl')
TRUTH = os.environ.get('CARD_TRUTH') or None     # 정답(학습용 라벨). app.py --truth 로도 지정
_SESSION = time.strftime('%Y%m%d_%H%M%S')
_ATTEMPT = [0]

# ---- 잘라내기(crop) 설정: 중요한 영역이 잘리면 여기 값만 조절하세요 -------------------------
# 박스는 (y0, y1, x0, x1) 비율(0.0~1.0). 카드 테두리 제거 후의 안쪽 영역 기준.
CARD_MARGIN = 0.02                    # 카드 사방 테두리 제거 비율 (0이면 제거 안 함)
DIGIT_BOX = (0.00, 0.75, 0.10, 0.90)  # 큰 숫자(1/2/3)가 있는 영역
LABEL_BOX = (0.45, 1.00, 0.00, 1.00)  # TICTACTOE / MAZE / EXIT / 한글 라벨 영역
TIGHT_PAD = 0.30                      # 글자 바운딩박스 주변 여백 비율 (_tight)
USE_CARD_DETECT = os.environ.get('CARD_DETECT', '1') != '0'   # 0이면 카드 사각형 검출 생략


def _lookup(token):
    """(하위 호환) 예전 vision.py 의 _lookup: 토큰 -> 게임 또는 None"""
    return base_lookup(token)[0]


def _prepare_card_images(image):
    """(하위 호환) 예전 vision.py 의 전처리 함수. 새 인식 경로(_variants)에서는 쓰지 않는다."""
    if image is None or image.size == 0:
        return []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image.copy()
    h, w = gray.shape[:2]
    scale = max(1.0, min(4.0, 1200.0 / max(h, w)))
    gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    adaptive = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 11)
    return [gray, binary, adaptive, cv2.bitwise_not(binary)]


def _jsonl(row):
    """학습용 구조화 로그. 실패해도 인식 동작에는 영향이 없다."""
    if not DEBUG or JSONL_PATH == '0':
        return
    try:
        Path(JSONL_PATH).parent.mkdir(parents=True, exist_ok=True)
        with open(JSONL_PATH, 'a', encoding='utf8') as f:
            f.write(json.dumps(row, ensure_ascii=False) + '\n')
    except OSError:
        pass


# --------------------------------------------------------------------------- tesseract 준비
def _tesseract():
    try:
        import pytesseract
    except ImportError:
        print('[card] pytesseract 미설치 -> pip install pytesseract')
        return None
    cmd = os.environ.get('TESSERACT_CMD') or shutil.which('tesseract')
    if not cmd and os.name == 'nt':
        for p in (r'C:\Program Files\Tesseract-OCR\tesseract.exe',
                  r'C:\Program Files (x86)\Tesseract-OCR\tesseract.exe'):
            if os.path.exists(p):
                cmd = p
                break
    if cmd:
        pytesseract.pytesseract.tesseract_cmd = cmd
    try:
        pytesseract.get_tesseract_version()
    except Exception:
        print('[card] tesseract 실행파일을 찾을 수 없음 -> Ubuntu: sudo apt install tesseract-ocr tesseract-ocr-kor '
              '/ Windows: 설치 후 TESSERACT_CMD 환경변수 지정')
        return None
    return pytesseract


def _languages(pt):
    try:
        have = set(pt.get_languages(config=''))
    except Exception:
        have = {'eng'}
    if 'kor' not in have:
        print('[card] kor 언어데이터 없음 -> 한글 인식 불가 (sudo apt install tesseract-ocr-kor)')
    return have


# --------------------------------------------------------------------------- 카드 검출
def _order(pts):
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    return np.array([pts[s.argmin()], pts[d.argmin()], pts[s.argmax()], pts[d.argmax()]], np.float32)


def _warp(image, quad):
    tl, tr, br, bl = _order(quad)
    w = max(np.linalg.norm(tr - tl), np.linalg.norm(br - bl))
    h = max(np.linalg.norm(bl - tl), np.linalg.norm(br - tr))
    if w < 100 or h < 60:
        return None
    portrait = h > w
    W, H = (400, 700) if portrait else (700, 400)
    dst = np.array([[0, 0], [W - 1, 0], [W - 1, H - 1], [0, H - 1]], np.float32)
    m = cv2.getPerspectiveTransform(np.array([tl, tr, br, bl], np.float32), dst)
    card = cv2.warpPerspective(image, m, (W, H))
    return cv2.rotate(card, cv2.ROTATE_90_CLOCKWISE) if portrait else card


def _find_card(image):
    """가장 큰 밝은 사각형(종이/스마트폰 화면)을 찾아 700x400 으로 펴준다. 실패하면 None."""
    h, w = image.shape[:2]
    s = min(1.0, 800.0 / max(h, w))
    small = cv2.resize(image, None, fx=s, fy=s) if s < 1 else image
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_min = 0.08 * small.shape[0] * small.shape[1]
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:5]:
        if cv2.contourArea(c) < area_min:
            break
        approx = cv2.approxPolyDP(c, 0.03 * cv2.arcLength(c, True), True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            return _warp(image, approx.reshape(4, 2).astype(np.float32) / s)
    return None


def _card_candidates(image):
    card = _find_card(image) if USE_CARD_DETECT else None
    if card is not None:
        yield 'card', card
        yield 'card180', cv2.rotate(card, cv2.ROTATE_180)
    yield 'full', image                     # 검출 실패/오검출 대비: 항상 원본 전체도 시도


# --------------------------------------------------------------------------- 전처리
def _crop(img, box):
    h, w = img.shape[:2]
    y0, y1, x0, x1 = box
    return img[int(h * y0):int(h * y1), int(w * x0):int(w * x1)]


def _regions(card):
    """테두리를 제거하고 숫자 영역/라벨 영역/전체로 분할. 비율은 파일 상단 상수로 조절."""
    h, w = card.shape[:2]
    my, mx = int(h * CARD_MARGIN), int(w * CARD_MARGIN)
    inner = card[my:h - my, mx:w - mx] if (my or mx) else card
    return {
        'digit': _crop(inner, DIGIT_BOX),
        'label': _crop(inner, LABEL_BOX),
        'whole': inner,
    }


def _annotate(card):
    """디버그용: 카드 위에 digit(빨강)/label(파랑) 잘라내는 박스를 그려 저장."""
    vis = card.copy() if card.ndim == 3 else cv2.cvtColor(card, cv2.COLOR_GRAY2BGR)
    h, w = vis.shape[:2]
    my, mx = int(h * CARD_MARGIN), int(w * CARD_MARGIN)
    ih, iw = h - 2 * my, w - 2 * mx
    cv2.rectangle(vis, (mx, my), (w - mx, h - my), (0, 160, 0), 2)
    for box, color in ((DIGIT_BOX, (0, 0, 255)), (LABEL_BOX, (255, 0, 0))):
        y0, y1, x0, x1 = box
        cv2.rectangle(vis, (mx + int(iw * x0), my + int(ih * y0)),
                      (mx + int(iw * x1) - 1, my + int(ih * y1) - 1), color, 2)
    return vis


def _tight(bw):
    """잉크(검정) 바운딩박스로 잘라 여백 제거 (단일 숫자/단어 인식률 향상)."""
    pts = cv2.findNonZero(255 - bw)
    if pts is None:
        return bw
    x, y, w, h = cv2.boundingRect(pts)
    pad = int(TIGHT_PAD * max(w, h)) + 4
    y0, y1 = max(0, y - pad), min(bw.shape[0], y + h + pad)
    x0, x1 = max(0, x - pad), min(bw.shape[1], x + w + pad)
    return bw[y0:y1, x0:x1]


def _frame_mask(bw):
    """카드 테두리선·가는 선·점 노이즈에 해당하는 잉크 성분을 찾아 마스크로 반환.
    OCR이 '[종료 |' 처럼 테두리를 글자 '[', '|' 로 읽는 문제를 막는다."""
    ink = (255 - bw)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(ink, connectivity=8)
    H, W = bw.shape[:2]
    mask = np.zeros(bw.shape[:2], bool)
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        touches = x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1
        full_frame = w >= 0.85 * W and h >= 0.85 * H
        thin_line = touches and max(w, h) / max(1, min(w, h)) > 6
        speck = area < max(12, 0.0002 * H * W)
        if full_frame or thin_line or speck:
            mask |= (labels == i)
    return mask


def _variants(region, tight):
    gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY) if region.ndim == 3 else region.copy()
    if gray.size == 0:
        return []
    s = float(np.clip(1000.0 / max(gray.shape[:2]), 0.5, 4.0))
    gray = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    if np.count_nonzero(otsu) < otsu.size / 2:      # 글자가 더 밝은(어두운 배경) 경우 -> 반전해서 '흰 배경 검정 글씨'로 통일
        gray = cv2.bitwise_not(gray)
        otsu = cv2.bitwise_not(otsu)
    adap = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 15)
    mask = _frame_mask(otsu)
    otsu[mask] = 255
    adap[mask] = 255
    out = []
    for img in (otsu, adap):
        if tight:
            img = _tight(img)
        out.append(cv2.copyMakeBorder(img, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255))
    return out


def _passes(name, langs):
    """(lang, tesseract config, 최소 신뢰도)"""
    if name == 'digit':   # 단일 숫자: psm 10/8 + 화이트리스트(1,2,3)
        return [('eng', '--oem 3 --psm 10 -c tessedit_char_whitelist=123', .5),
                ('eng', '--oem 3 --psm 8 -c tessedit_char_whitelist=123', .5)]
    p = [('eng', '--oem 3 --psm 7', .3), ('eng', '--oem 3 --psm 6', .3)]
    if 'kor' in langs:
        p += [('kor', '--oem 3 --psm 7', .3), ('kor', '--oem 3 --psm 6', .3)]
    return p


def _ocr(pt, img, lang, cfg):
    try:
        d = pt.image_to_data(img, lang=lang, config=cfg, output_type=pt.Output.DICT)
    except pt.TesseractNotFoundError:
        raise
    except (pt.TesseractError, RuntimeError):
        return '', 0.0
    words, confs = [], []
    for w, c in zip(d.get('text', []), d.get('conf', [])):
        w = str(w).strip()
        try:
            c = float(c)
        except (TypeError, ValueError):
            continue
        if w and c >= 0:
            words.append(w)
            confs.append(c / 100.0)
    return ' '.join(words), (sum(confs) / len(confs) if confs else 0.0)


# --------------------------------------------------------------------------- 공개 API
def recognize_card_result(image, *, fallback=None, languages=None):
    """Return (game, confidence, raw_text). 여러 영역/전처리/언어의 OCR 결과를 투표(CardVoter)로 결정."""
    def _fb():
        return (card_from_text(fallback), 0.0, '') if fallback else (None, 0.0, '')

    if image is None or image.size == 0:
        return _fb()
    pt = _tesseract()
    if pt is None:
        return _fb()
    langs = _languages(pt)
    if languages:
        langs &= set(languages.split('+')) | {'eng'}

    h, w = image.shape[:2]
    if max(h, w) > 1600:                                  # 초고해상도 사진은 속도를 위해 축소
        image = cv2.resize(image, None, fx=1600.0 / max(h, w), fy=1600.0 / max(h, w), interpolation=cv2.INTER_AREA)

    voter = CardVoter()
    _ATTEMPT[0] += 1
    attempt = _ATTEMPT[0]
    if DEBUG:
        DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    def _finish(game):
        conf = voter.confidence(game)
        _jsonl({'event': 'outcome', 'session': _SESSION, 'attempt': attempt, 'truth': TRUTH, 'game': game,
                'conf': round(conf, 3)})
        return game, conf, ' | '.join(voter.raws)

    try:
        for cname, card in _card_candidates(image):
            if DEBUG:
                cv2.imwrite(str(DEBUG_DIR / f'{cname}.png'), card)
                cv2.imwrite(str(DEBUG_DIR / f'{cname}_boxes.png'), _annotate(card))   # 잘라내는 영역 확인용
            for rname, region in _regions(card).items():
                for vi, img in enumerate(_variants(region, tight=True)):
                    if DEBUG:
                        cv2.imwrite(str(DEBUG_DIR / f'{cname}_{rname}_{vi}.png'), img)
                    for lang, cfg, _ in _passes(rname, langs):
                        text, conf = _ocr(pt, img, lang, cfg)
                        ev = voter.add(cname, rname, vi, lang, text, conf)
                        if DEBUG and text:
                            # 1)신뢰도 2)예측 텍스트 3)텍스트 구분(eng/kor/digit/...)  -> 매칭된 게임(방식)
                            extra = f" -> {ev['game']}({ev['kind']}{'' if ev['counted'] else ',미반영'})" if ev['game'] else ''
                            print(f"[card] {cname}/{rname}/v{vi}/{lang} '{text}' conf={conf:.2f} type={ev['type']}{extra}")
                        if text:
                            psm = re.search(r'--psm (\d+)', cfg)
                            _jsonl({'event': 'read', 'session': _SESSION, 'attempt': attempt, 'truth': TRUTH,
                                    'cand': cname, 'region': rname, 'var': vi, 'lang': lang,
                                    'psm': int(psm.group(1)) if psm else None, 'text': text,
                                    'conf': round(conf, 3), 'type': ev['type'], 'game': ev['game'],
                                    'kind': ev['kind'], 'counted': ev['counted']})
                        decided = voter.early()               # 독립된 두 번의 판독이 일치하면 즉시 확정
                        if decided:
                            return _finish(decided)
            decided = voter.decision()
            if decided:
                return _finish(decided)
    except pt.TesseractNotFoundError:
        print('[card] tesseract 실행 실패(경로 확인)')
        return _fb()

    decided = voter.decision()
    if decided:
        return _finish(decided)
    _jsonl({'event': 'outcome', 'session': _SESSION, 'attempt': attempt, 'truth': TRUTH, 'game': None, 'conf': 0.0})
    return _fb()


def recognize_card(image, *, fallback=None):
    """Backward-compatible game-only API."""
    game, _, _ = recognize_card_result(image, fallback=fallback)
    return game


def tic_board_from_image(image):
    """Simple clean top-down O/X sheet recognition (not a production detector)."""
    g = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    g = cv2.resize(g, (600, 600))
    out = []
    for r in range(3):
        row = []
        for c in range(3):
            cell = g[r * 200 + 32:(r + 1) * 200 - 32, c * 200 + 32:(c + 1) * 200 - 32]
            ink = cv2.threshold(cell, 130, 255, cv2.THRESH_BINARY_INV)[1]
            ratio = cv2.countNonZero(ink) / ink.size
            if ratio < .015:
                row.append(' ')
                continue
            cy, cx = np.array(ink.shape) // 2
            center = ink[cy-9:cy+9, cx-9:cx+9]
            center_ratio = cv2.countNonZero(center) / center.size
            row.append('X' if center_ratio > .16 else 'O')
        out.append(row)
    return out
