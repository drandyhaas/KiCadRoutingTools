#!/usr/bin/env python3
"""#1303 phase (a): a diff pair is graded on the copper it SHIPS.

route_diff called imx6ulz DQS0 'coupled' because both members had a routing
result, while the copper it wrote put P on In2.Cu and N on B.Cu: 0 mm coupled,
4.5 mm of P/N skew. Its intra-pair matcher measured its own result dict and
meandered the leg that was already longer, and nothing graded a pair at all.
Now:

  measure    net_queries.diff_pair_coupling / diff_pair_skew -- coupled and
             uncoupled length and P/N skew as KiCad's DRC computes them
             (KiCad 10.0.0 drc_test_provider_diff_pair_coupling.cpp, the
             integer SEG geometry ported in kicad_seg).
  check_drc  grades a board's .kicad_dru `skew` (with `within_diff_pairs`)
             and `diff_pair_uncoupled` rules through #1313's length channel.
  route_diff a pair whose shipped copper is not coupled is 'uncoupled': out of
             routed_diff_pairs and of the #521 protection unless its members
             still run alongside (#1361, test_1361_loose_pair_protection);
             the intra-pair status is re-read off the copper.
  board_score a `diff_pairs` component: every pair measured, graded against
             what the board's dru declares.

Fixture: a 2-layer board this test writes, one pair per case (straight,
diagonal, offset, split layers, wide gap, a track or a via between P and N at
the end KiCad checks, a track between them mid-run, N in two pieces, a skewed
pair). KICAD_* below are kicad-cli 10.0.0's own `actual` values for it
(`pcb drc --format json`, 2026-10-21); with kicad-cli on PATH or installed
the test re-runs KiCad and compares live as well.

    python3 tests/test_1303_pair_copper_grade.py
"""
import contextlib
import io
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import uuid
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, TESTS_DIR)

from check_drc import run_drc                                   # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from net_queries import (diff_pair_copper, CopperIndex,         # noqa: E402
                         diff_pair_coupling)
from run_utils import check as check_run, evidence              # noqa: E402

# kicad-cli 10.0.0 on the fixture with DRU below: diff_pair_uncoupled_length_too_long
# "actual" per pair (a pair KiCad did not flag has uncoupled <= 0 under max 0),
# and skew_out_of_range |actual| (within_diff_pairs).
KICAD_UNCOUPLED = {'S': 0.0, 'D': 0.0, 'O': 5.0, 'L': 10.0, 'W': 10.0,
                   'B': 10.0, 'M': 0.0, 'V': 10.0, 'T': 0.0, 'K': 2.0}
KICAD_SKEW = {'O': 2.0, 'K': 2.0}
DRU = """(version 1)
(rule "pairs"
  (condition "A.inDiffPair('*')")
  (constraint diff_pair_uncoupled (max 0mm)))
(rule "skew"
  (condition "A.inDiffPair('*')")
  (constraint skew (max 0mm) (within_diff_pairs)))
(rule "wgap"
  (condition "A.inDiffPair('W')")
  (constraint diff_pair_gap (min 0.05mm) (max 0.15mm)))
"""
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        return fn(*a, **k), out.getvalue()


# --------------------------------------------------------------------------
# the fixture board
# --------------------------------------------------------------------------
_HEAD = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
         '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n'
         '\t\t(legacy_teardrops no)\n\t)\n\t(paper "A4")\n\t(layers\n'
         '\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n\t\t(1 "F.Mask" user)\n'
         '\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
         '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n')


def fixture_text():
    n = [0]
    body = []

    def u():
        n[0] += 1
        return str(uuid.UUID(int=n[0]))

    def seg(x1, y1, x2, y2, net, layer='F.Cu', w=0.1):
        body.append(f'\t(segment\n\t\t(start {x1:.6f} {y1:.6f})\n\t\t(end {x2:.6f} {y2:.6f})\n'
                    f'\t\t(width {w})\n\t\t(layer "{layer}")\n\t\t(net "{net}")\n'
                    f'\t\t(uuid "{u()}")\n\t)\n')

    refs = [0]

    def pad(x, y, net, layer='F.Cu'):
        refs[0] += 1
        body.append(f'\t(footprint "TP"\n\t\t(layer "{layer}")\n\t\t(uuid "{u()}")\n'
                    f'\t\t(at {x:.6f} {y:.6f})\n\t\t(property "Reference" "TP{refs[0]}"\n'
                    f'\t\t\t(at 0 -1 0)\n\t\t\t(layer "F.Fab")\n\t\t\t(uuid "{u()}")\n'
                    f'\t\t\t(effects (font (size 0.5 0.5) (thickness 0.1)))\n\t\t)\n'
                    f'\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.1 0.1)\n'
                    f'\t\t\t(layers "{layer}" "{layer[0]}.Mask")\n\t\t\t(net "{net}")\n'
                    f'\t\t\t(uuid "{u()}")\n\t\t)\n\t)\n')

    def pair(name, p, n_, lp='F.Cu', ln='F.Cu', pads=True):
        seg(*p, f'{name}_P', lp)
        seg(*n_, f'{name}_N', ln)
        if pads:
            for (x1, y1, x2, y2), net, layer in ((p, f'{name}_P', lp), (n_, f'{name}_N', ln)):
                pad(x1, y1, net, layer)
                pad(x2, y2, net, layer)

    pair('S', (10, 10, 20, 10), (10, 10.2, 20, 10.2))
    a = math.radians(30)
    ux, uy = math.cos(a), math.sin(a)
    px, py = -uy * 0.2, ux * 0.2
    pair('D', (30, 10, 30 + 8 * ux, 10 + 8 * uy),
         (30 + px, 10 + py, 30 + px + 8 * ux, 10 + py + 8 * uy))
    pair('O', (10, 20, 20, 20), (13, 20.2, 25, 20.2))
    pair('L', (10, 25, 20, 25), (10, 25.2, 20, 25.2), 'F.Cu', 'B.Cu', pads=False)
    pair('W', (10, 30, 20, 30), (10, 30.5, 20, 30.5))
    pair('B', (10, 35, 20, 35), (10, 36, 20, 36))
    seg(10.03, 35.2, 10.03, 35.8, 'GNDX')
    pair('M', (30, 35, 40, 35), (30, 36, 40, 36))
    seg(35, 35.2, 35, 35.8, 'GNDX')
    pair('V', (10, 40, 20, 40), (10, 41, 20, 41))
    body.append(f'\t(via\n\t\t(at 10.000000 40.500000)\n\t\t(size 0.4)\n\t\t(drill 0.2)\n'
                f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "GNDX")\n\t\t(uuid "{u()}")\n\t)\n')
    seg(10, 45, 20, 45, 'T_P')
    seg(10, 45.2, 14, 45.2, 'T_N')
    seg(14, 45.2, 20, 45.2, 'T_N')
    for x, y, net in ((10, 45, 'T_P'), (20, 45, 'T_P'), (10, 45.2, 'T_N'), (20, 45.2, 'T_N')):
        pad(x, y, net)
    pair('K', (30, 45, 40, 45), (30, 45.2, 42, 45.2))
    for (x1, y1, x2, y2) in ((5, 5, 50, 5), (50, 5, 50, 50), (50, 50, 5, 50), (5, 50, 5, 5)):
        body.append(f'\t(gr_line\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
                    f'\t\t(stroke (width 0.05) (type default))\n\t\t(layer "Edge.Cuts")\n'
                    f'\t\t(uuid "{u()}")\n\t)\n')
    return _HEAD + ''.join(body) + ')\n'


def ids(pcb):
    return {n.name: i for i, n in pcb.nets.items()}


# --------------------------------------------------------------------------
# units
# --------------------------------------------------------------------------
def kicad_seg_unit():
    from kicad_seg import intersects, rescale, iu
    check('rescale rounds half away from zero, as KiCad __int128 does',
          rescale(3, 1, 2) == 2 and rescale(-3, 1, 2) == -2 and rescale(5, 2, 4) == 3)
    check('iu converts mm to nm as the board parser does', iu(1.0000005) == 1000001)
    a, b = (0, 0), (10_000_000, 0)
    check('two parallel distinct lines do not intersect',
          not intersects(a, b, (0, 200_000), (10_000_000, 200_000), lines=True))
    check('collinear lines intersect (not a coupled candidate)',
          intersects(a, b, (20_000_000, 0), (30_000_000, 0), lines=True))
    check('1 nm off parallel over 10 mm: the lines meet beyond int32, so KiCad '
          'reads them as parallel', not intersects(a, b, (0, 200_000), (10_000_000, 200_001),
                                                   lines=True))
    check('0.1 mm off parallel: the lines meet on the board, not parallel',
          intersects(a, b, (0, 200_000), (10_000_000, 300_000), lines=True))


def dru_channel_unit():
    from kicad_dru import parse_dru_length_rules, in_diff_pair
    rules, _ = parse_dru_length_rules(DRU, ('skew', 'diff_pair_uncoupled'))
    kinds = {(r.name, r.kind, r.within_diff_pairs, r.max_mm) for r in rules}
    check('the channel reads skew (with within_diff_pairs) and diff_pair_uncoupled',
          kinds == {('pairs', 'diff_pair_uncoupled', False, 0.0),
                    ('skew', 'skew', True, 0.0)}, str(kinds))
    check('by default it still reads length windows only (#1313 unchanged)',
          parse_dru_length_rules(DRU)[0] == [])
    timed, _ = parse_dru_length_rules(
        '(version 1)(rule t (constraint skew (max 10ps)))', ('skew',))
    check('a time-domain bound is kept, as not modelled',
          timed and timed[0].terms is None and 'time-domain' in timed[0].unsupported)
    _, notes = parse_dru_length_rules(
        '(version 1)(rule z (severity ignore) (constraint diff_pair_uncoupled (max 1mm)))',
        ('diff_pair_uncoupled',))
    check('a severity-ignore pair rule is skipped with a note',
          any("rule 'z'" in n and 'ignore' in n for n in notes), str(notes))
    names = {'/DRAM/DQS0_P', '/DRAM/DQS0_N', 'X_P'}
    check("inDiffPair matches the base name and the base without its '_'",
          in_diff_pair('/DRAM/DQS0_P', '/DRAM/DQS0_', names)
          and in_diff_pair('/DRAM/DQS0_P', '/DRAM/DQS0', names)
          and in_diff_pair('/DRAM/DQS0_N', '/DRAM/*', names))
    check('inDiffPair is case-sensitive (wxString::Matches)',
          not in_diff_pair('/DRAM/DQS0_P', '/dram/*', names))
    check('inDiffPair needs the complement net on the board',
          not in_diff_pair('X_P', '*', names))


def regrade_unit():
    from length_matching import regrade_intra_pair_reports
    pair = SimpleNamespace(p_net_id=1, n_net_id=2)
    recs = [
        {'pair': 'A', 'status': 'matched', 'reason': None, 'delta_mm': 0.0, 'delta_before_mm': 1.1},
        {'pair': 'B', 'status': 'improved', 'reason': 'meanders-ran-out', 'delta_mm': 0.39,
         'delta_before_mm': 1.08},
        {'pair': 'C', 'status': 'within-tolerance', 'reason': None, 'delta_mm': 0.02,
         'delta_before_mm': 0.02},
        {'pair': 'D', 'status': 'matched', 'reason': None, 'delta_mm': 0.0, 'delta_before_mm': 2.6},
    ]
    copper = {'A': {'skew_mm': 1.97}, 'B': {'skew_mm': 5.2}, 'C': {'skew_mm': 0.05},
              'D': {'skew_mm': None}}
    regrade_intra_pair_reports(recs, copper, None, [(r['pair'], pair) for r in recs], 0.1)
    st = {r['pair']: (r['status'], r['reason'], r['delta_mm'], r['matcher_status'])
          for r in recs}
    check("a matcher 'matched' over tolerance on the copper is unmatched, with why",
          st['A'] == ('unmatched', 'copper-skew-over-tolerance', 1.97, 'matched'), str(st['A']))
    check("an 'improved' pair keeps the matcher's reason and takes the copper skew",
          st['B'] == ('unmatched', 'meanders-ran-out', 5.2, 'improved'), str(st['B']))
    check('inside tolerance on the copper stays within-tolerance',
          st['C'][0] == 'within-tolerance', str(st['C']))
    check('an unmeasurable pair is not left claiming a match',
          st['D'][:2] == ('unmatched', 'not-measured'), str(st['D']))


def stress_stats_unit():
    sys.path.insert(0, os.path.join(TESTS_DIR, 'stress'))
    from ab_replay_grade import _diff_pair_stats
    log = 'JSON_SUMMARY: ' + json.dumps({
        'routed_diff_pairs': ['A'], 'uncoupled_diff_pairs': ['B'],
        'failed_diff_pairs': [], 'single_ended_diff_pairs': [], 'partial_diff_pairs': []})
    s = _diff_pair_stats(log)
    check('ab_replay_grade counts an uncoupled pair, not as coupled',
          s['diff_pairs_total'] == 2 and s['diff_pairs_coupled'] == 1
          and s['diff_pairs_uncoupled'] == 1, str(s))


# --------------------------------------------------------------------------
# the measure and its consumers on the fixture
# --------------------------------------------------------------------------
def measure_vs_kicad(pcb, live=None):
    idx = CopperIndex(pcb)
    byname = ids(pcb)
    bad = {}
    for name, want in KICAD_UNCOUPLED.items():
        win = (0.05, 0.15) if name == 'W' else (None, None)
        r = diff_pair_copper(pcb, byname[f'{name}_P'], byname[f'{name}_N'], *win, index=idx)
        got = max(r['uncoupled_mm'], 0.0)
        if abs(got - want) > 1e-4:
            bad[name] = (got, want)
        sk = r['skew_mm']
        if sk is not None and abs(sk - KICAD_SKEW.get(name, 0.0)) > 1e-4:
            bad[name + ' skew'] = (sk, KICAD_SKEW.get(name, 0.0))
    check("the coupled/uncoupled measure is KiCad's on every case (recorded actuals)",
          not bad, str(bad))
    m = diff_pair_coupling(pcb, byname['M_P'], byname['M_N'])
    b = diff_pair_coupling(pcb, byname['B_P'], byname['B_N'])
    check('KiCad checks ONE line between P and N: a track there blocks, '
          'one mid-run does not', b['coupled_mm'] == 0 and m['coupled_mm'] == 10.0,
          f"B {b['coupled_mm']} M {m['coupled_mm']}")
    s = diff_pair_coupling(pcb, byname['S_P'], byname['S_N'], None, 0.15)
    w = diff_pair_coupling(pcb, byname['W_P'], byname['W_N'], None, 0.15)
    check('a gap window keeps the 0.1 mm pair and drops the 0.4 mm one',
          s['coupled_mm'] == 10.0 and w['coupled_mm'] == 0.0)
    if live:
        diff = {k: (live.get(k), v) for k, v in KICAD_UNCOUPLED.items()
                if abs((live.get(k) or 0.0) - v) > 1e-4}
        check('kicad-cli, run now, still reports the recorded actuals', not diff, str(diff))


def live_kicad(board):
    """kicad-cli's uncoupled actuals for the fixture, or None without KiCad."""
    try:
        from kicad_oracle import find_kicad_cli
        cli = find_kicad_cli(warn=False)
    except Exception:                                            # noqa: BLE001
        cli = None
    if not cli:
        print('  (kicad-cli not found: comparing against the recorded actuals only)')
        return None
    out = board[:-len('.kicad_pcb')] + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--units', 'mm', '-o', out, board],
                   capture_output=True, text=True, timeout=600)
    d = json.load(open(evidence(out, 'kicad-cli drc json')))
    got = {k: 0.0 for k in KICAD_UNCOUPLED}
    for v in d.get('violations', []):
        if v['type'] != 'diff_pair_uncoupled_length_too_long':
            continue
        m = re.search(r'actual ([0-9.]+) mm', v['description'])
        net = re.search(r'\[(\w)_[PN]\]', ' '.join(i.get('description', '')
                                                    for i in v.get('items', [])))
        if m and net:
            got[net.group(1)] = float(m.group(1))
    return got


def drc_pass(board, pcb):
    rows, log = quiet(run_drc, board, clearance=0.05, max_print=0, check_sizes=False)
    unc = {v['net1'][0]: v['uncoupled_mm'] for v in rows
           if v.get('type') == 'diff-pair-uncoupled'}
    skew = {v['net1']: v['skew_mm'] for v in rows if v.get('type') == 'skew-out-of-range'}
    want = {k: v for k, v in KICAD_UNCOUPLED.items() if v > 0 and k != 'L'}
    check('check_drc flags exactly the pairs KiCad flags (L has no pads: not routed)',
          unc == want, f'{unc} vs {want}')
    check('check_drc flags the shorter member of each skewed pair, at its skew',
          skew == {'O_P': 2.0, 'K_P': 2.0}, str(skew))
    check("the console names KiCad's checks",
          'diff_pair_uncoupled_length_too_long' in log and 'skew_out_of_range' in log)
    rows, _ = quiet(run_drc, board, clearance=0.05, max_print=0, check_sizes=False,
                    net_patterns=['K_*'])
    scoped = sorted({v['net1'] for v in rows
                     if v.get('type') in ('diff-pair-uncoupled', 'skew-out-of-range')})
    check('--nets scopes the pair grade', scoped == ['K_P'], str(scoped))


def custody_and_route_diff(pcb):
    from diff_pair_custody import measure_pair_copper, build_pair_reports
    from route_diff import coupled_protection_pairs
    byname = ids(pcb)
    pairs = [(nm, SimpleNamespace(p_net_id=byname[f'{nm}_P'], n_net_id=byname[f'{nm}_N'],
                                  p_net_name=f'{nm}_P', n_net_name=f'{nm}_N'))
             for nm in ('S', 'D', 'L', 'W', 'B')]
    results = {}
    for _nm, pr in pairs:
        res = {'is_diff_pair': True, 'new_segments': [object()]}
        results[pr.p_net_id] = results[pr.n_net_id] = res
    copper, _ = quiet(measure_pair_copper, pcb, pairs, results, lambda _p: (0.1, 0.1))
    coupled = {k: v['coupled'] for k, v in copper.items()}
    check('the routed gap decides coupling: S and D couple; split layers, a 0.4 mm '
          'gap and a 0.9 mm gap do not', coupled == {'S': True, 'D': True, 'L': False,
                                                     'W': False, 'B': False}, str(coupled))
    state = SimpleNamespace(routed_results=results, diff_pair_single_ended_nets={},
                            pair_diagnostics={})
    reps, _ = quiet(build_pair_reports, state, pairs, {}, pair_copper=copper)
    out = {r['pair']: r['outcome'] for r in reps}
    check("a pair whose copper is not coupled is 'uncoupled', never 'coupled'",
          out == {'S': 'coupled', 'D': 'coupled', 'L': 'uncoupled', 'W': 'uncoupled',
                  'B': 'uncoupled'}, str(out))
    check('every claimed pair carries its copper record',
          all('copper' in r and 'skew_mm' in r['copper'] for r in reps))
    state.diff_pair_single_ended_nets = {pairs[2][1].p_net_id: 'L_P'}
    reps, _ = quiet(build_pair_reports, state, pairs, {}, pair_copper=copper)
    lrep = next(r for r in reps if r['pair'] == 'L')
    check("a 'partial' pair with no coupled copper has no coupled trunk to keep credit with",
          lrep['outcome'] == 'partial' and lrep['no_coupled_trunk']
          and lrep['member_audit_mismatch'], str(lrep))
    kept = [n for n, _p in coupled_protection_pairs(pairs, copper)]
    check('#521 protection skips the pairs whose copper is not coupled (no member '
          'audit given: none is rescued as alongside, #1361)',
          kept == ['S', 'D'], str(kept))


def board_score_component(tmp, board):
    sj = os.path.join(tmp, 's.json')
    check_run([sys.executable, '-X', 'utf8', 'py_tools/board_score.py', board, '--json', sj],
              code=4)
    d = json.load(open(evidence(sj, 'score json')))
    dp = d['components']['diff_pairs']
    check('board_score grades the pairs against the dru: one row per KiCad violation',
          dp.get('ran') and dp.get('count') == 7
          and dp.get('by_type') == {'diff-pair-uncoupled': 5, 'skew-out-of-range': 2},
          str({k: dp.get(k) for k in ('ran', 'count', 'by_type')}))
    check('...and the rows leave the drc component (counted once)',
          not set(d['components']['drc'].get('by_type') or {})
          & {'diff-pair-uncoupled', 'skew-out-of-range'})
    check('each measured pair is keyed by its P net and names its N net',
          dp['pairs'].get('K_P', {}).get('n_net') == 'K_N'
          and abs(dp['pairs']['K_P']['skew_mm'] - 2.0) < 1e-4)
    check('the net-name audit accepts every name the component publishes',
          not (d.get('net_name_audit') or {}).get('unknown_count'))
    os.remove(board[:-len('.kicad_pcb')] + '.kicad_dru')
    check_run([sys.executable, '-X', 'utf8', 'py_tools/board_score.py', board, '--json', sj],
              code=4)
    d = json.load(open(evidence(sj, 'score json')))
    dp = d['components']['diff_pairs']
    check('without a declared rule the pairs are measured, not graded (UNGRADED)',
          dp.get('ran') is False and dp.get('count') is None and dp.get('pairs')
          and 'diff_pairs' in d['ungraded'], str({k: dp.get(k) for k in ('ran', 'count')}))


def main():
    kicad_seg_unit()
    dru_channel_unit()
    regrade_unit()
    stress_stats_unit()
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'pairs.kicad_pcb')
        with open(board, 'w') as fh:
            fh.write(fixture_text())
        with open(os.path.join(tmp, 'pairs.kicad_dru'), 'w') as fh:
            fh.write(DRU)
        pcb, _ = quiet(parse_kicad_pcb, evidence(board, 'fixture board'))
        measure_vs_kicad(pcb, live_kicad(board))
        drc_pass(board, pcb)
        custody_and_route_diff(pcb)
        board_score_component(tmp, board)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
