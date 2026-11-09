"""텍스트 단계 로직 (이미지/OCR 엔진과 무관해서 로그만으로 재현·학습·평가 가능).

구성
  - CARD_MAP / normalize / text_kind : 키워드 표, 정규화, 텍스트 구분(eng/kor/digit/mixed/symbol/jamo)
  - Rules      : configs/ocr_rules.json (ocr_learn.py 가 로그에서 학습해 생성) 을 읽어 매칭에 반영
                 · aliases      : 자주 나오는 오인식 문자열 -> 게임 (예: '송료' -> exit)
                 · edge_noise   : 테두리가 글자로 읽혀 앞/뒤에 붙는 문자 (예: 'LOX' -> 'OX')
                 · confusions   : 글자 단위 혼동 가중치 (예: 읽힘 F -> 실제 T)  -> 학습형 퍼지 매칭
                 · source_weights : (언어/영역)별 신뢰도 가중치  예) 'kor/label': 0.6
                 · digit_posterior: digit 영역에서 읽힌 숫자가 실제 어느 카드였는지의 확률
  - CardVoter  : 여러 번의 OCR 판독을 투표로 합쳐 결정 (vision.py 실시간 인식과 ocr_learn.py replay 가 같은 코드를 사용)

규칙 파일이 없으면 이전 버전과 똑같이 동작한다 (기본 규칙만 사용).
"""
import difflib
import json
import os
import re
from collections import defaultdict
from pathlib import Path

GAMES = ('tictactoe', 'maze', 'exit')

CARD_MAP = {
    '1': 'tictactoe', 'OX': 'tictactoe', '틱택토': 'tictactoe', 'TIC': 'tictactoe', 'TICTACTOE': 'tictactoe',
    '2': 'maze', 'MAZE': 'maze', '미로': 'maze', '미로찾기': 'maze',
    '3': 'exit', 'EXIT': 'exit', 'STOP': 'exit', '종료': 'exit',
}

ROOT = Path(__file__).resolve().parents[1]
RULES_PATH = Path(os.environ.get('OCR_RULES') or ROOT / 'configs' / 'ocr_rules.json')

# ---- 투표/확정 기준 ---------------------------------------------------------------------
WORD_SINGLE_CONF = 0.35     # 글자(영어/한글 단어)가 1번만 읽혔을 때 확정 최소 신뢰도
DIGIT_OUTSIDE_CONF = 0.70   # 숫자만 읽힌 결과를 digit 영역 밖에서 인정할 최소 신뢰도
MIN_SINGLE_CONF = 0.5       # 판독 1번일 때 평균 신뢰도 기준
MIN_CONF_DIGIT = 0.5        # digit 영역 판독 최소 신뢰도
MIN_CONF_TEXT = 0.3         # 그 외 영역 판독 최소 신뢰도

_KOR = re.compile('[가-힣]')
_FUZZY_KEYS = [k for k in CARD_MAP if len(k) >= 4 or (len(k) >= 3 and _KOR.search(k))]


# --------------------------------------------------------------------------- 기본 유틸
def normalize(text):
    return re.sub(r'[^A-Z0-9가-힣]', '', str(text).upper())


def text_kind(text):
    """OCR 결과 텍스트 구분: digit / eng / kor / mixed / jamo / symbol"""
    s = re.sub(r'\s', '', str(text))
    n = normalize(s)
    if not n:
        return 'jamo' if re.search('[ㄱ-ㅣ]', s) else 'symbol'
    has_k = bool(_KOR.search(n))
    has_l = bool(re.search('[A-Z]', n))
    has_d = bool(re.search('[0-9]', n))
    if has_k and (has_l or has_d):
        return 'mixed'
    if has_k:
        return 'kor'
    if has_l and has_d:
        return 'mixed'
    return 'digit' if has_d else 'eng'


def base_lookup(token):
    """기본 매칭(학습 규칙 없음): 정확 일치 -> 철자 유사(difflib). 반환 (game, kind)"""
    if token in CARD_MAP:
        return CARD_MAP[token], 'exact'
    if len(token) >= 3:
        m = difflib.get_close_matches(token, _FUZZY_KEYS, n=1, cutoff=0.75)
        if m:
            return CARD_MAP[m[0]], 'fuzzy'
    return None, None


# --------------------------------------------------------------------------- 학습된 규칙
class Rules:
    def __init__(self, data=None):
        d = data or {}
        self.meta = d.get('meta', {})
        self.edge_noise = set(d.get('edge_noise', ''))
        self.source_weights = d.get('source_weights', {})
        self.digit_posterior = d.get('digit_posterior', {})
        self.confusions = d.get('confusions', {})     # {읽힌글자: {실제글자: 가중치(0~1)}}
        self.aliases = {}
        seen = defaultdict(dict)
        for game, items in d.get('aliases', {}).items():
            for k, info in items.items():
                w = float(info.get('weight', 1.0)) if isinstance(info, dict) else float(info)
                nk = normalize(k)
                if nk:
                    seen[nk][game] = max(w, seen[nk].get(game, 0.0))
        for nk, g in seen.items():            # 같은 문자열이 서로 다른 게임에 걸리면 모호하므로 버린다
            if len(g) == 1:
                game, w = next(iter(g.items()))
                self.aliases[nk] = (game, w)

    @classmethod
    def load(cls, path=None):
        p = Path(path or RULES_PATH)
        if p.exists():
            try:
                return cls(json.loads(p.read_text(encoding='utf8')))
            except (OSError, ValueError) as e:
                print(f'[card] 규칙 파일을 읽지 못해 기본 규칙 사용: {p} ({e})')
        return cls()

    # ---- 가중치
    def source_weight(self, lang, region):
        return float(self.source_weights.get(f'{lang}/{region}', 1.0))

    # ---- 글자 단위 가중 편집거리 (읽힌 글자 -> 실제 글자 치환 비용 = 1 - 학습 가중치)
    def _sub_cost(self, a, b):
        if a == b:
            return 0.0
        return 1.0 - float(self.confusions.get(a, {}).get(b, 0.0))

    def _wdist(self, s, key):
        prev = [float(j) for j in range(len(key) + 1)]
        for i, a in enumerate(s, 1):
            cur = [float(i)]
            for j, b in enumerate(key, 1):
                cur.append(min(prev[j] + 1.0, cur[j - 1] + 1.0, prev[j - 1] + self._sub_cost(a, b)))
            prev = cur
        return prev[-1]

    def _lfuzzy(self, n):
        """학습된 혼동(예: F->T)을 반영한 퍼지 매칭. 키워드 길이에 따라 허용 거리를 제한한다."""
        if not self.confusions or len(n) < 2:
            return None
        best = {}
        for key, game in CARD_MAP.items():
            if key.isdigit() or abs(len(key) - len(n)) > 1:
                continue
            allow = 0.6 if len(key) <= 3 else 0.25 * len(key)
            d = self._wdist(n, key)
            if d <= allow and d < 1.0 * len(key) - 0.5:
                best[game] = min(best.get(game, 9.0), d)
        if len(best) != 1:                   # 서로 다른 게임에 동시에 가까우면 모호 -> 거부
            return None
        return next(iter(best))

    # ---- 토큰 하나 매칭
    def _match_token(self, n):
        if n in self.aliases:
            return self.aliases[n][0], 'alias'
        g, k = base_lookup(n)
        if g:
            return g, k
        if self.edge_noise:                  # 테두리가 읽혀 앞/뒤에 붙은 문자를 벗겨서 재시도
            left, right = n, n
            while len(left) > 2 and left[0] in self.edge_noise:
                left = left[1:]
            while len(right) > 2 and right[-1] in self.edge_noise:
                right = right[:-1]
            both = left
            while len(both) > 2 and both[-1] in self.edge_noise:
                both = both[:-1]
            for v in (left, right, both):
                if v != n and len(v) >= 2:
                    if v in self.aliases:
                        return self.aliases[v][0], 'edge'
                    g, k = base_lookup(v)
                    if g:
                        return g, 'edge'
        g = self._lfuzzy(n)
        if g:
            return g, 'lfuzzy'
        return None, None

    def alias_weight(self, text):
        a = self.aliases.get(normalize(text))
        return a[1] if a else 1.0

    def match(self, text):
        """(game, kind) 반환. kind: exact | fuzzy | alias | edge | lfuzzy | None"""
        n = normalize(text)
        if not n:
            return None, None
        g, k = self._match_token(n)
        if g:
            return g, k
        # 숫자 토큰은 제외: '= H3/3' 같은 노이즈 속 숫자가 카드로 오인되는 것을 막음
        hits = {}
        for t in re.findall(r'[A-Z]+|[가-힣]+', str(text).upper()):
            g, k = self._match_token(t)
            if g:
                hits[g] = k
        if len(hits) == 1:
            return next(iter(hits.items()))
        return None, None


_RULES_CACHE = {}


def load_rules(path=None, refresh=False):
    key = str(path or RULES_PATH)
    if refresh or key not in _RULES_CACHE:
        _RULES_CACHE[key] = Rules.load(path)
    return _RULES_CACHE[key]


def card_from_text(text, rules=None):
    """Map recognized text to a game; None if empty/ambiguous. (규칙 파일이 있으면 학습 규칙 적용)"""
    return (rules or load_rules()).match(text)[0]


# --------------------------------------------------------------------------- 투표기
def _decide(votes):
    if not votes:
        return None
    ranked = sorted(votes.items(), key=lambda kv: kv[1], reverse=True)
    if len(ranked) > 1 and ranked[1][1] > 0.5 * ranked[0][1]:   # 서로 다른 게임이 비슷하게 득표 -> 모호
        return None
    return ranked[0][0]


class CardVoter:
    """OCR 판독 하나하나를 add() 로 넣으면 투표가 쌓이고, early()/decision() 으로 결정한다."""

    def __init__(self, rules=None):
        self.rules = rules or load_rules()
        self.votes = defaultdict(float)      # 가중 득표 (소스 신뢰도/별칭 가중치 반영) -> 게임 간 순위·모호성 판단용
        self.rawsum = defaultdict(float)     # 원래 신뢰도 합 -> 확정 기준/보고용 (가중치로 문턱이 올라가지 않게)
        self.nreads = defaultdict(int)
        self.seen = defaultdict(set)         # 게임별 '서로 다른 이미지 변형'에서 나온 판독 (독립 표)
        self.best_word = defaultdict(float)  # 게임별 '글자 단어'로 읽힌 최고 신뢰도
        self.raws = []

    def add(self, cand, region, var, lang, text, conf):
        """판독 1개 반영. 반환: dict(game, kind, counted, ...) (로그/디버그용)"""
        ev = {'game': None, 'kind': None, 'counted': False, 'type': text_kind(text)}
        if not str(text).strip():
            return ev
        n = normalize(text)
        game, kind = self.rules.match(text)
        weight = 1.0
        digit_only = bool(re.fullmatch(r'[0-9]+', n))
        min_conf = MIN_CONF_DIGIT if region == 'digit' else MIN_CONF_TEXT

        post = self.rules.digit_posterior.get(n) if (region == 'digit' and len(n) == 1) else None
        if post:                              # 학습된 '읽힌 숫자 -> 실제 카드' 확률로 대체
            g2 = max(post, key=post.get)
            game, kind, weight = (g2, 'digit_post', float(post[g2])) if post[g2] >= 0.4 else (None, None, 1.0)
        elif kind == 'alias':
            weight = self.rules.alias_weight(text)
            digit_only = False                # 학습된 별칭은 '글자 증거'로 취급 ('11' -> tictactoe 등)
        elif game and digit_only and region != 'digit':
            min_conf = max(min_conf, DIGIT_OUTSIDE_CONF)   # 라벨/전체 영역의 단독 숫자는 노이즈 가능성 높음

        ev.update(game=game, kind=kind)
        if game and conf >= min_conf:         # 문턱은 원래 신뢰도로 판단 (소스 가중치가 문턱을 올리면 숫자 카드가 통째로 탈락)
            if not digit_only and kind not in ('lfuzzy', 'digit_post'):
                self.best_word[game] = max(self.best_word[game], conf)
            self.votes[game] += conf * self.rules.source_weight(lang, region) * weight
            self.rawsum[game] += conf
            self.nreads[game] += 1
            self.seen[game].add((cand, region, var))
            self.raws.append(text)
            ev['counted'] = True
        return ev

    def counts(self, game):
        return len(self.seen[game])

    def confident(self, game):
        return (self.counts(game) >= 2 or self.best_word.get(game, 0.0) >= WORD_SINGLE_CONF
                or self.rawsum[game] / max(1, self.counts(game)) >= MIN_SINGLE_CONF)

    def early(self):
        """독립된 두 번 이상의 판독이 일치하면 즉시 확정"""
        d = _decide(self.votes)
        return d if d and self.counts(d) >= 2 else None

    def decision(self):
        d = _decide(self.votes)
        return d if d and self.confident(d) else None

    def confidence(self, game):
        return min(1.0, self.rawsum[game] / max(1, self.nreads[game]))
