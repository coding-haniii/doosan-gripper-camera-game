"""OCR 로그 기반 오류 패턴 학습 도구.

대량의 카드 사진을 모아 모델을 다시 학습시키는 대신,
  '실행 로그(터미널 출력 / out/ocr_log.jsonl)' -> 키워드(카드)별 자주 나오는 오인식 규칙 -> configs/ocr_rules.json
으로 정리해서 vision.py(ocr_rules.py)가 다음 실행부터 반영하게 한다.

사용 순서
  1) python -m robot_games.ocr_learn labels  로그.txt [로그2.txt ...]      # 정답 확인용 CSV 생성 (사람이 truth 열 확인/수정)
  2) python -m robot_games.ocr_learn learn   로그.txt [...] --labels configs/ocr_labels.csv
  3) python -m robot_games.ocr_learn replay  로그.txt [...] --labels configs/ocr_labels.csv --cv 5   # 개선 효과 점검

로그 형식: 새 형식 "[card] cand/region/vN/lang '텍스트' conf=0.xx", 예전 형식 "DEBUG OCR [psm N]: '텍스트'",
           그리고 vision.py 가 남기는 JSONL(.jsonl, 정답 truth 포함 가능) 모두 읽는다.
"""
import argparse
import csv
import difflib
import json
import re
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

from .ocr_rules import CARD_MAP, GAMES, ROOT, CardVoter, Rules, normalize

R_NEW = re.compile(r"^\[card\] (\w+)/(\w+)/v(\d+)/(\w+) '(.*)' conf=([\d.]+)")
R_OLD = re.compile(r"^DEBUG OCR \[psm (\d+)\]: '(.*)'")
R_DONE = re.compile(r'^(Detected|Card detected): (\w+)')
START_MARKS = ('python3 -m robot_games.app', 'python -m robot_games.app', 'Show action card')

DEFAULT_LABELS = ROOT / 'configs' / 'ocr_labels.csv'
DEFAULT_SEED = ROOT / 'configs' / 'ocr_seed_rules.json'
DEFAULT_RULES = ROOT / 'configs' / 'ocr_rules.json'
DEFAULT_REPORT = ROOT / 'out' / 'ocr_learn_report.md'
LOG_DIR = ROOT / 'ocr_logs'                       # 터미널 로그(.txt)를 모아두는 폴더
JSONL_LOG = ROOT / 'out' / 'ocr_log.jsonl'        # vision.py 가 자동으로 남기는 로그


# =========================================================================== 로그 파싱
def parse_terminal(path):
    """터미널 로그 -> runs. run = 프로그램 1회 실행, attempt = SPACE 1회(성공/실패)"""
    path = Path(path)
    lines = path.read_text(encoding='utf8', errors='replace').splitlines()
    runs, cur, att = [], None, []

    def flush(outcome):
        nonlocal att
        if cur is not None and (att or outcome):
            cur['attempts'].append({'reads': att, 'outcome': outcome})
        att = []

    for no, ln in enumerate(lines, 1):
        if any(m in ln for m in START_MARKS):
            if cur is None or cur['attempts'] or att:
                if cur is not None:
                    flush(None)
                    if cur['attempts']:
                        runs.append(cur)
                cur = {'id': f'{path.name}:{no}', 'attempts': [], 'final': None, 'truth': None}
            continue
        if cur is None:
            continue
        m = R_NEW.match(ln)
        if m:
            c, r, v, lang, t, cf = m.groups()
            att.append({'fmt': 'new', 'cand': c, 'region': r, 'var': int(v), 'lang': lang, 'text': t, 'conf': float(cf)})
            continue
        m = R_OLD.match(ln)
        if m:
            att.append({'fmt': 'old', 'cand': 'old', 'region': 'unknown', 'var': int(m.group(1)), 'lang': 'unknown',
                        'text': m.group(2), 'conf': None})
            continue
        m = R_DONE.match(ln)
        if m:
            flush(m.group(2))
            cur['final'] = m.group(2)
            continue
        if 'Card not recognized' in ln:
            flush('FAIL')
    if cur is not None:
        flush(None)
        if cur['attempts']:
            runs.append(cur)
    return runs


def parse_jsonl(path):
    path = Path(path)
    sessions = defaultdict(lambda: defaultdict(lambda: {'reads': [], 'outcome': None}))
    truth = {}
    for ln in path.read_text(encoding='utf8', errors='replace').splitlines():
        try:
            row = json.loads(ln)
        except ValueError:
            continue
        s, a = row.get('session'), row.get('attempt')
        if row.get('truth'):
            truth[s] = row['truth']
            sessions[s][a]['truth'] = row['truth']          # 정답은 시도(attempt) 단위로 보관: 카드를 바꿔가며 찍어도 안 섞인다
        if row.get('event') == 'read':
            sessions[s][a]['reads'].append({'fmt': 'new', 'cand': row['cand'], 'region': row['region'],
                                            'var': row['var'], 'lang': row['lang'], 'text': row['text'],
                                            'conf': row['conf']})
        elif row.get('event') == 'outcome':
            sessions[s][a]['outcome'] = row.get('game') or 'FAIL'
    runs = []
    for s, atts in sessions.items():
        attempts = [atts[a] for a in sorted(atts)]
        finals = [x['outcome'] for x in attempts if x['outcome'] not in (None, 'FAIL')]
        ts = {x.get('truth') for x in attempts if x.get('truth')}
        runs.append({'id': f'{path.name}:{s}', 'attempts': attempts, 'final': finals[-1] if finals else None,
                     'truth': ts.pop() if len(ts) == 1 else None})
    return runs


def load_runs(paths):
    runs = []
    for p in paths:
        runs += parse_jsonl(p) if str(p).endswith('.jsonl') else parse_terminal(p)
    return runs


# =========================================================================== 라벨링 (사람 확인)
def load_seed(path=None):
    p = Path(path or DEFAULT_SEED)
    if not p.exists():
        print(f'[경고] 사용자 오류 목록(seed) 파일이 없습니다: {p}  -> 로그에서 학습한 규칙만 사용합니다.')
        return {}
    return json.loads(p.read_text(encoding='utf8'))


def seed_rules(seed):
    data = {'aliases': {g: {k: 0.9 for k in ks if normalize(k) not in CARD_MAP} for g, ks in seed.get('aliases', {}).items()}}
    return Rules(data)


def run_evidence(run, label, rules):
    """이 run 의 읽힌 글자들이 label 을 얼마나 뒷받침하는지."""
    lw = ow = ld = 0
    for a in run['attempts']:
        for r in a['reads']:
            g = rules.match(r['text'])[0]
            if g is None:
                continue
            digit_only = bool(re.fullmatch(r'[0-9]+', normalize(r['text'])))
            if digit_only:
                ld += g == label
            elif g == label:
                lw += 1
            else:
                ow += 1
    if lw and not ow:
        return 'word'
    if lw:
        return 'word+conflict'
    return 'digit' if ld else 'none'


def suggest(run, rules):
    c = Counter()
    for a in run['attempts']:
        for r in a['reads']:
            g = rules.match(r['text'])[0]
            if g and not re.fullmatch(r'[0-9]+', normalize(r['text'])):
                c[g] += 1
    return c.most_common(1)[0][0] if c else ''


def cmd_labels(args):
    runs = load_runs(args.logs)
    seeds = seed_rules(load_seed(args.seed))
    base = Rules()
    out = Path(args.out)
    old = {}
    if out.exists():                                     # 사람이 이미 입력한 truth 는 보존
        with open(out, encoding='utf8', newline='') as f:
            old = {r['run_id']: r for r in csv.DictReader(f)}
    rows = []
    for r in runs:
        auto = r['truth'] or r['final'] or ''
        ev = run_evidence(r, auto, base) if auto else ''
        sug = suggest(r, seeds) if not auto else ''
        prev = old.get(r['id'], {})
        rows.append({'run_id': r['id'], 'attempts': len(r['attempts']),
                     'outcomes': '/'.join(str(a['outcome'] or '?') for a in r['attempts']),
                     'auto_label': auto, 'evidence': ev, 'suggest': sug,
                     'truth': prev.get('truth', ''), 'note': prev.get('note', '')})
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, 'w', encoding='utf8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['run_id', 'attempts', 'outcomes', 'auto_label', 'evidence', 'suggest', 'truth', 'note'])
        w.writeheader()
        w.writerows(rows)
    ev = Counter(r['evidence'] or ('(라벨없음)' if not r['auto_label'] else '') for r in rows)
    print(f'runs {len(rows)}개 -> {out}')
    print('auto_label 근거(evidence):', dict(ev))
    print("truth 열: tictactoe/maze/exit 중 하나를 적으면 그 값이 정답. 'skip' 이면 학습에서 제외. 비워두면 auto_label 이 "
          "글자 근거(word)가 있을 때만 사용.")


def resolve_labels(runs, labels_csv, trust_auto=False):
    """-> (labeled attempts, 제외 사유 카운트)"""
    truth = {}
    if labels_csv and Path(labels_csv).exists():
        with open(labels_csv, encoding='utf8', newline='') as f:
            truth = {r['run_id']: (r.get('truth') or '').strip() for r in csv.DictReader(f)}
    base = Rules()
    out, why = [], Counter()
    for r in runs:
        t = truth.get(r['id'], '')
        auto = r['truth'] or r['final'] or ''
        label = src = None
        if t in GAMES:
            label, src = t, 'human'
        elif t.lower() in ('skip', 'x', '-'):
            why['사람이 skip'] += 1
            continue
        elif auto and (r['truth'] or run_evidence(r, auto, base).startswith('word') or trust_auto):
            label, src = auto, 'auto'
        used = False
        for a in r['attempts']:
            lab = a.get('truth') if a.get('truth') in GAMES else label        # jsonl 의 시도별 정답이 있으면 그것을 우선
            if lab:
                out.append({'run': r['id'], 'label': lab, 'src': src or 'jsonl', 'reads': a['reads'], 'outcome': a['outcome']})
                used = True
        if not used:
            why['정답 없음/근거 부족' if not auto else 'auto 라벨인데 글자 근거 없음(노이즈 판정 의심)'] += 1
    return out, why


# =========================================================================== 학습 (규칙 추출)
def _toks(text):
    n = normalize(text)
    t = {n} if n else set()
    t |= {x for x in re.findall(r'[A-Z]+|[가-힣]+', str(text).upper()) if len(x) >= 2}
    return t


def _keys(label):
    return [k for k, g in CARD_MAP.items() if g == label]


def plausible(tok, label):
    """학습 후보가 '우연'이 아니라 그 카드의 키워드가 깨진 형태인지 대충 거르는 필터."""
    for key in _keys(label):
        if difflib.SequenceMatcher(None, tok, key).ratio() >= 0.5:
            return True
        if len(tok) == len(key) >= 3 and any(a == b for a, b in zip(tok, key)):
            return True
    return False


def best_key(tok, label):
    ks = [k for k in _keys(label) if not k.isdigit() and len(k) >= 2]
    return max(ks, key=lambda k: difflib.SequenceMatcher(None, tok, k).ratio()) if ks else None


def _digit_ok(tok, label):
    """다른 카드의 숫자(예: tictactoe 별칭에 '2')가 섞인 문자열은 별칭으로 부적합."""
    own = [k for k in _keys(label) if k.isdigit()]
    others = {d for d in '123'} - set(own)
    return not (set(tok) & others)


def mine(attempts, seed, min_support=2, include_old=False):
    """라벨된 시도(attempts) -> (rules dict, report dict)
    기본은 새 형식 로그([card] 영역/언어/신뢰도 포함)의 판독만 사용한다. 예전 형식(DEBUG OCR)은 출처 정보가 없고
    다른 전처리로 읽은 값이라 우연한 노이즈 문자열이 별칭으로 올라오기 쉬워서 include_old=True 일 때만 사용."""
    base = Rules()
    if not include_old:
        attempts = [dict(a, reads=[r for r in a['reads'] if r['fmt'] == 'new']) for a in attempts]
        attempts = [a for a in attempts if a['reads']]
    nA = Counter(a['label'] for a in attempts)
    df = defaultdict(Counter)                                   # token -> label -> 서로 다른 시도 수
    srcs = defaultdict(lambda: defaultdict(Counter))            # token -> label -> Counter((lang,region))
    for a in attempts:
        seen = set()
        for r in a['reads']:
            for t in _toks(r['text']):
                srcs[t][a['label']][(r['lang'], r['region'])] += 1
                if t not in seen:
                    seen.add(t)
                    df[t][a['label']] += 1

    # ---- 1) 별칭(alias): 키워드별로 자주 나오는 오인식 문자열
    aliases = {g: {} for g in GAMES}
    rejected, base_wrong, hits = [], [], defaultdict(list)
    for t, cnt in df.items():
        if len(t) < 2:
            continue
        lik = {L: cnt[L] / nA[L] for L in GAMES if nA[L]}
        s = sum(lik.values())
        if not s:
            continue
        L = max(lik, key=lik.get)
        post, sup = lik[L] / s, cnt[L]
        g0 = base.match(t)[0]
        if g0 == L:
            hits[L].append((t, sup))
            continue
        if g0 and g0 != L and sup >= min_support:
            base_wrong.append((t, g0, L, sup))                  # 기본 규칙이 엉뚱한 게임으로 매칭하는 패턴
        reason = None
        if sup < min_support:
            continue
        if post < 0.8:
            reason = f'여러 카드에 공통 등장(확률 {post:.0%})'
        elif not _digit_ok(t, L):
            reason = '다른 카드의 숫자가 섞여 있음'
        elif not plausible(t, L):
            reason = '키워드와 모양이 달라 우연일 가능성'
        if reason:
            rejected.append((t, L, sup, reason))
        else:
            aliases[L][t] = {'weight': round(min(1.0, post) * min(1.0, 0.5 + sup / 6), 2), 'support': sup, 'src': 'log'}
    for g, ks in seed.get('aliases', {}).items():                # 사용자가 정리한 오류 목록(문제풀이 메모)
        for k in ks:
            if normalize(k) not in CARD_MAP and normalize(k) not in aliases[g]:
                aliases[g][normalize(k)] = {'weight': 0.9, 'support': 0, 'src': 'seed'}

    # ---- 2) 테두리 문자 (앞/뒤에 붙어서 읽히는 글자) : 벗기면 인식이 늘고 오인식은 없는 문자만 채택
    texts = {(a['label'], r['text']) for a in attempts for r in a['reads']}
    edge_cand = Counter()
    for _, tx in texts:
        n = normalize(tx)
        if len(n) >= 3:
            edge_cand[n[0]] += 1
            edge_cand[n[-1]] += 1
    edge = ''
    for ch, c in edge_cand.items():
        if c < 3 or ch.isdigit():
            continue
        r_ch = Rules({'edge_noise': ch})
        gain, loss = set(), 0
        for i, a in enumerate(attempts):
            for r in a['reads']:
                if base.match(r['text'])[0] is None:
                    g = r_ch.match(r['text'])[0]
                    if g == a['label']:
                        gain.add(i)
                    elif g:
                        loss += 1
        if len(gain) >= min_support and loss == 0:
            edge += ch

    # ---- 3) digit 영역: 읽힌 숫자 -> 실제 카드 확률 (클래스 균형 + 키워드 기본값 사전확률)
    dfd, nd = defaultdict(Counter), Counter()
    for a in attempts:
        ds = {normalize(r['text']) for r in a['reads'] if r['region'] == 'digit' and len(normalize(r['text'])) == 1}
        if any(r['region'] == 'digit' for r in a['reads']):
            nd[a['label']] += 1
        for d in ds & set('123'):
            dfd[a['label']][d] += 1
    digit_post = {}
    if sum(nd.values()):
        for d in '123':
            lik = {L: (dfd[L][d] + 3 * (CARD_MAP[d] == L)) / (nd[L] + 3) for L in GAMES}
            s = sum(lik.values())
            digit_post[d] = {L: round(lik[L] / s, 3) for L in GAMES}

    # ---- 4) 글자 단위 혼동(읽힌 글자 -> 실제 글자): 별칭과 키워드를 정렬해서 집계
    conf_cnt = defaultdict(Counter)
    for g, items in aliases.items():
        for t, info in items.items():
            if t.isdigit():
                continue
            key = best_key(t, g)
            if not key or len(key) != len(t) or difflib.SequenceMatcher(None, t, key).ratio() < 0.3:
                continue
            for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, t, key).get_opcodes():
                if tag == 'replace' and i2 - i1 == j2 - j1:
                    for x, y in zip(t[i1:i2], key[j1:j2]):
                        conf_cnt[x][y] += max(1, info.get('support', 0))   # seed 별칭도 최소 1회로 반영
    for x, y in seed.get('confusions', []):
        x, y = normalize(x), normalize(y)
        if x and y and x != y:
            conf_cnt[x][y] += 2
    confusions = {x: {y: round(min(0.8, c / (c + 2)), 2) for y, c in d.items()} for x, d in conf_cnt.items()}

    # ---- 5) 소스(언어/영역)별 신뢰도: 기본 규칙으로 키워드가 잡힌 판독 중 정답과 일치한 비율
    tot, ok = Counter(), Counter()
    for a in attempts:
        for r in a['reads']:
            if r['fmt'] != 'new':
                continue
            g = base.match(r['text'])[0]
            if g:
                k = f"{r['lang']}/{r['region']}"
                tot[k] += 1
                ok[k] += g == a['label']
    src_w = {k: round(max(0.3, min(1.0, (ok[k] + 2) / (tot[k] + 2))), 2) for k in tot if tot[k] >= 8}

    rules = {
        'meta': {'generated': time.strftime('%Y-%m-%d %H:%M:%S'), 'attempts': dict(nA), 'min_support': min_support},
        'edge_noise': edge, 'aliases': aliases, 'confusions': confusions,
        'digit_posterior': digit_post, 'source_weights': src_w,
    }
    report = {'nA': nA, 'df': df, 'srcs': srcs, 'hits': hits, 'rejected': rejected, 'base_wrong': base_wrong,
              'digit_post': digit_post, 'tot': tot, 'ok': ok, 'edge': edge}
    return rules, report


def write_report(path, rules, rep, why, n_runs):
    nA, df = rep['nA'], rep['df']
    L = ['# OCR 학습 리포트', '', f"- 사용한 run: {n_runs}개 / 시도(attempt): {dict(nA)}",
         f"- 제외된 run: {dict(why) if why else '없음'}", '',
         '정답 라벨은 사람이 확인한 truth 가 우선이고, 없으면 "그 실행의 최종 Detected + 글자 근거 있음"일 때만 사용했습니다.', '']
    for g in GAMES:
        L += [f'## {g}   (키워드: {", ".join(_keys(g))})', f'시도 {nA[g]}회 기준, 읽힌 텍스트(정규화) 상위 변형', '',
              '| 읽힌 텍스트 | 등장 시도수 | 주 출처 | 학습 결과 |', '|---|---:|---|---|']
        top = sorted(((t, c[g]) for t, c in df.items() if c[g] and len(t) >= 2), key=lambda x: -x[1])[:14]
        for t, c in top:
            s = rep['srcs'][t][g].most_common(2)
            src = ', '.join(f'{a}/{b}' for (a, b), _ in s)
            if t in rules['aliases'][g]:
                st = f"별칭 추가 (가중치 {rules['aliases'][g][t]['weight']})"
            elif (t, c) in rep['hits'][g]:
                st = '기본 규칙으로 이미 인식'
            else:
                rj = [x for x in rep['rejected'] if x[0] == t and x[1] == g]
                st = f'제외: {rj[0][3]}' if rj else ('지지도 부족' if c < rules['meta']['min_support'] else '-')
            L.append(f'| `{t}` | {c} | {src} | {st} |')
        seeds = [t for t, i in rules['aliases'][g].items() if i['src'] == 'seed']
        if seeds:
            L += ['', f'사용자 메모(seed)에서 등록된 별칭: {", ".join(seeds)}']
        L.append('')
    L += ['## 기본 규칙이 엉뚱한 게임으로 매칭한 패턴 (오인식 위험)', '', '| 읽힌 텍스트 | 기본 규칙 결과 | 실제 | 시도수 |', '|---|---|---|---:|']
    L += [f'| `{t}` | {g0} | {real} | {n} |' for t, g0, real, n in sorted(rep['base_wrong'], key=lambda x: -x[3])[:15]] or ['| (없음) | | | |']
    L += ['', '## digit 영역: 읽힌 숫자 -> 실제 카드 확률', '', '| 읽힌 숫자 | ' + ' | '.join(GAMES) + ' |', '|---|' + '---:|' * 3]
    for d, p in rep['digit_post'].items():
        L.append(f'| {d} | ' + ' | '.join(f'{p[g]:.0%}' for g in GAMES) + ' |')
    if not rep['digit_post']:
        L.append('| (digit 영역 판독 없음) | | | |')
    L += ['', '## 소스(언어/영역)별 신뢰도 (키워드가 잡힌 판독 중 정답과 일치 비율)', '', '| 소스 | 판독수 | 정답 일치 | 적용 가중치 |', '|---|---:|---:|---:|']
    for k in sorted(rep['tot']):
        L.append(f"| {k} | {rep['tot'][k]} | {rep['ok'][k]} | {rules['source_weights'].get(k, '1.0 (표본 부족)')} |")
    L += ['', f"## 테두리 문자(앞/뒤에 붙는 글자): `{rules['edge_noise'] or '없음'}`", '',
          '## 글자 혼동(읽힌 글자 -> 실제 글자)', '']
    L += [f'- `{x}` -> ' + ', '.join(f'`{y}`({w})' for y, w in d.items()) for x, d in sorted(rules['confusions'].items())] or ['- 없음']
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(L) + '\n', encoding='utf8')


def cmd_learn(args):
    runs = load_runs(args.logs)
    attempts, why = resolve_labels(runs, args.labels, args.trust_auto)
    if not attempts:
        sys.exit('학습에 쓸 라벨된 시도가 없습니다. 먼저 labels 명령으로 CSV 를 만들고 truth 를 채우세요.')
    rules, rep = mine(attempts, load_seed(args.seed), args.min_support, args.include_old)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rules, ensure_ascii=False, indent=1), encoding='utf8')
    write_report(args.report, rules, rep, why, len({a['run'] for a in attempts}))
    n_alias = {g: len(v) for g, v in rules['aliases'].items()}
    print(f"규칙 저장: {out}\n리포트: {args.report}")
    print(f"사용 시도 {dict(rep['nA'])} / 별칭 {n_alias} / 테두리문자 '{rules['edge_noise']}' / 혼동쌍 {sum(len(v) for v in rules['confusions'].values())}개")
    if why:
        print('제외된 run:', dict(why))


# =========================================================================== 재생(replay)·교차검증
def classify(reads, rules):
    """로그에 남은 판독들을 vision.py 와 같은 CardVoter 로 다시 투표시켜 결과를 낸다."""
    voter, prev = CardVoter(rules), None
    for r in reads:
        if prev is not None and r['cand'] != prev and voter.decision():
            return voter.decision()
        prev = r['cand']
        voter.add(r['cand'], r['region'], r['var'], r['lang'], r['text'], r['conf'])
        d = voter.early()
        if d:
            return d
    return voter.decision()


def evaluate(attempts, rules):
    res = defaultdict(Counter)
    for a in attempts:
        reads = [r for r in a['reads'] if r['fmt'] == 'new']
        if not reads:
            continue
        g = classify(reads, rules)
        res[a['label']]['ok' if g == a['label'] else ('fail' if g is None else 'wrong')] += 1
    return res


def fmt(res):
    t = Counter()
    for c in res.values():
        t.update(c)
    n = sum(t.values())
    return f"정답 {t['ok']}/{n} ({t['ok'] / max(1, n):.0%}) · 오인식 {t['wrong']} · 인식실패 {t['fail']}"


def cmd_replay(args):
    runs = load_runs(args.logs)
    rid = sorted({r['id'] for r in runs})
    attempts, why = resolve_labels(runs, args.labels, args.trust_auto)
    attempts = [a for a in attempts if any(r['fmt'] == 'new' for r in a['reads'])]
    if not attempts:
        sys.exit('새 형식([card] ...) 로그가 있는 라벨된 시도가 없습니다.')
    seed = load_seed(args.seed)
    base = evaluate(attempts, Rules())
    print(f"평가 대상 시도 {len(attempts)}개: {dict(Counter(a['label'] for a in attempts))}")
    print('기본 규칙(학습 없음)      :', fmt(base))
    if args.rules and Path(args.rules).exists():
        print(f'규칙 파일 적용({args.rules}) :', fmt(evaluate(attempts, Rules.load(args.rules))), ' ※학습에 쓴 로그라 낙관적')
    k = args.cv
    if k and k > 1:
        fold = {r: i % k for i, r in enumerate(rid)}
        cv = defaultdict(Counter)
        for f in range(k):
            train = [a for a in attempts if fold[a['run']] != f]
            test = [a for a in attempts if fold[a['run']] == f]
            if not train or not test:
                continue
            rules, _ = mine(train, seed, args.min_support, args.include_old)
            for lab, c in evaluate(test, Rules(rules)).items():
                cv[lab].update(c)
        print(f'{k}-fold 교차검증(run 단위)  :', fmt(cv), ' ← 학습에 안 쓴 run 으로 측정한 값')
    print('\n라벨별(기본 -> 교차검증):')
    for g in GAMES:
        b = base.get(g, Counter())
        n = sum(b.values())
        if n:
            line = f"  {g:10s} n={n:3d}  기본 정답 {b['ok']:3d} 오인식 {b['wrong']:2d} 실패 {b['fail']:3d}"
            if k and k > 1 and cv.get(g):
                c = cv[g]
                line += f"  ->  CV 정답 {c['ok']:3d} 오인식 {c['wrong']:2d} 실패 {c['fail']:3d}"
            print(line)


def find_logs(paths):
    """로그 경로를 안 주면 ocr_logs/*.txt 와 out/ocr_log.jsonl 을 자동으로 사용한다."""
    if paths:
        return [str(p) for p in paths]
    found = sorted(str(p) for p in LOG_DIR.glob('*.txt')) if LOG_DIR.exists() else []
    if JSONL_LOG.exists():
        found.append(str(JSONL_LOG))
    if not found:
        sys.exit(f'로그가 없습니다. 터미널 로그를 {LOG_DIR}/ 에 넣거나 경로를 직접 지정하세요.')
    print('사용할 로그:', *found, sep='\n  ')
    return found


def main(argv=None):
    p = argparse.ArgumentParser(description='OCR 로그 기반 오류 패턴 학습')
    sub = p.add_subparsers(dest='cmd', required=True)
    for name in ('labels', 'learn', 'replay'):
        sp = sub.add_parser(name)
        sp.add_argument('logs', nargs='*', help='터미널 로그(.txt) 또는 .jsonl. 생략하면 ocr_logs/*.txt + out/ocr_log.jsonl')
        sp.add_argument('--seed', default=str(DEFAULT_SEED), help='사용자 정리 오류 목록(JSON)')
        if name == 'labels':
            sp.add_argument('-o', '--out', default=str(DEFAULT_LABELS))
        else:
            sp.add_argument('--labels', default=str(DEFAULT_LABELS))
            sp.add_argument('--min-support', type=int, default=2, help='별칭으로 인정할 최소 시도 수')
            sp.add_argument('--trust-auto', action='store_true', help='글자 근거 없는 Detected 라벨도 정답으로 사용(비권장)')
            sp.add_argument('--include-old', action='store_true', help='예전 형식(DEBUG OCR) 판독도 별칭 학습에 사용(비권장)')
        if name == 'learn':
            sp.add_argument('--out', default=str(DEFAULT_RULES))
            sp.add_argument('--report', default=str(DEFAULT_REPORT))
        if name == 'replay':
            sp.add_argument('--rules', default=str(DEFAULT_RULES))
            sp.add_argument('--cv', type=int, default=5, help='교차검증 fold 수 (0 이면 생략)')
    a = p.parse_args(argv)
    a.logs = find_logs(a.logs)
    {'labels': cmd_labels, 'learn': cmd_learn, 'replay': cmd_replay}[a.cmd](a)


if __name__ == '__main__':
    main()
