"""액션카드/테스트 이미지 생성기.

- card_{1,2,3}.png          : 숫자 + 영어 라벨 (기존과 동일 구성, 중앙 정렬)
- card_{1,2,3}_ko.png       : 숫자 + 한글 라벨 (cv2.putText는 한글이 '???'로 나오므로 PIL 사용)
- card_text_{...}.png       : 숫자 없이 글자만 (영어/한글) -> 텍스트만 있는 카드 인식 테스트
- photo_card_{1,2,3}.png    : 어두운 배경 위에 기울어진 카드 -> 카드 검출/원근보정 테스트
- tic_board.png, maze_*.png : 기존 샘플

한글 폰트는 시스템에서 자동 탐색합니다. 못 찾으면 환경변수 KOREAN_FONT 로 .ttf/.ttc 경로를 지정하세요.
"""
import os
from pathlib import Path

import cv2
import numpy as np

from .maze import generate_maze
from .tictactoe import render_board

CARDS = [('1', 'TICTACTOE', '틱택토'), ('2', 'MAZE', '미로'), ('3', 'EXIT', '종료')]
CARD_W, CARD_H = 700, 400

FONT_CANDIDATES = [
    r'C:\Windows\Fonts\malgun.ttf',
    r'C:\Windows\Fonts\malgunbd.ttf',
    '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/noto/NotoSansCJK-Bold.ttc',
    '/System/Library/Fonts/AppleSDGothicNeo.ttc',
    '/Library/Fonts/AppleGothic.ttf',
]


def find_korean_font():
    env = os.environ.get('KOREAN_FONT')
    for p in ([env] if env else []) + FONT_CANDIDATES:
        if p and os.path.exists(p):
            return p
    return None


def _center_cv_text(img, text, cy, scale, thick):
    """cv2.putText 영어/숫자 텍스트를 가로 중앙 정렬. cy는 글자 세로 중심."""
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x = (img.shape[1] - tw) // 2
    y = cy + th // 2
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick, cv2.LINE_AA)


def _center_pil_text(img, text, cy, size, font_path):
    """PIL로 한글 텍스트를 가로 중앙 정렬해서 그린다 (BGR ndarray 반환)."""
    from PIL import Image, ImageDraw, ImageFont
    font = ImageFont.truetype(font_path, size)
    pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    l, t, r, b = draw.textbbox((0, 0), text, font=font)
    x = (img.shape[1] - (r - l)) // 2 - l
    y = cy - (b - t) // 2 - t
    draw.text((x, y), text, font=font, fill=(0, 0, 0))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def _blank_card():
    img = np.full((CARD_H, CARD_W, 3), 255, np.uint8)
    cv2.rectangle(img, (10, 10), (CARD_W - 10, CARD_H - 10), (0, 0, 0), 4)
    return img


def card_en(number, label):
    img = _blank_card()
    _center_cv_text(img, number, 140, 5, 12)
    _center_cv_text(img, label, 295, 1.6, 4)
    return img


def card_ko(number, label_ko, font_path):
    img = _blank_card()
    _center_cv_text(img, number, 140, 5, 12)
    return _center_pil_text(img, label_ko, 295, 80, font_path)


def card_text_only_en(label):
    img = _blank_card()
    _center_cv_text(img, label, CARD_H // 2, 3.0, 8)
    return img


def card_text_only_ko(label_ko, font_path):
    return _center_pil_text(_blank_card(), label_ko, CARD_H // 2, 160, font_path)


def photo_like(card, angle=8, bg=(70, 70, 70)):
    """카드를 어두운 배경 위에 기울여 올려둔 '사진' 느낌의 테스트 이미지."""
    H, W = 720, 1000
    canvas = np.full((H, W, 3), bg, np.uint8)
    h, w = card.shape[:2]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst = np.float32([[190, 150], [820, 110], [850, 560], [160, 600]])    # 약한 원근 + 기울기
    m = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(card, m, (W, H), borderMode=cv2.BORDER_TRANSPARENT, dst=canvas)
    rot = cv2.getRotationMatrix2D((W / 2, H / 2), angle, 1.0)
    return cv2.warpAffine(warped, rot, (W, H), borderValue=bg)


def make(output):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    font = find_korean_font()
    if font is None:
        print('[cards] 한글 폰트를 찾지 못해 한글 카드는 건너뜁니다. KOREAN_FONT=/path/to/font.ttf 로 지정하세요.')

    for number, label, label_ko in CARDS:
        en = card_en(number, label)
        cv2.imwrite(str(out / f'card_{number}.png'), en)
        cv2.imwrite(str(out / f'photo_card_{number}.png'), photo_like(en))
        cv2.imwrite(str(out / f'card_text_{label.lower()}.png'), card_text_only_en(label))
        if font:
            cv2.imwrite(str(out / f'card_{number}_ko.png'), card_ko(number, label_ko, font))
            cv2.imwrite(str(out / f'card_text_{label.lower()}_ko.png'), card_text_only_ko(label_ko, font))

    cv2.imwrite(str(out / 'tic_board.png'), render_board([[' ', ' ', ' '], [' ', 'X', ' '], [' ', ' ', ' ']]))
    for seed in (7, 11, 25):
        img, _, _ = generate_maze(seed)
        cv2.imwrite(str(out / f'maze_{seed}.png'), img)
    print('Generated samples:', out)


if __name__ == '__main__':
    make(Path(__file__).resolve().parents[1] / 'assets')
