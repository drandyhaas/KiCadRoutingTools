#!/usr/bin/env python3
"""#1706: a .kicad_dru rule check_drc's channels do not model decides library pad gaps too.

check_drc publishes one footprint's pad pairs closer than their clearance
(#1628) as certain KiCad `clearance` errors, priced at the class and the
rules its channels model. A rule they do not model -- one scoped by
`insideCourtyard()` -- was never asked, so a pair it clears was still
published, and board_score / check_complete said KiCad would report it. Now
every clearance rule is walked as KiCad walks them (the last matching one
wins), a courtyard term read for a pad as its footprint: a pair the binding
rule clears, or one a `(severity ignore)` rule binds, is not published; a
pair a rule decided by an unreadable term may clear is published TAGGED, and
the count is "up to N". kicad_drc_compare names a published pair KiCad does
not report.

The board: two identical parts, each with two pad pairs 0.150 / 0.145 mm
apart under a 0.16 mm Default class. Arms (KiCad's counts measured on
kicad-cli 10.0.6, and re-measured here when it is installed):
  none    no rule                                          4
  court   insideCourtyard rule clears the first part, 0.10    2
  later   that rule, then a later pad-pad rule at 0.16     4
  ignore  a (severity ignore) insideCourtyard rule         2
  front   insideFrontCourtyard (a term not read here)      2, published as up to 4

    python3 tests/test_1706_library_gap_rule.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []

OWN = ("A.Type == 'Pad' && B.Type == 'Pad' && A.insideCourtyard('U1') "
       "&& B.insideCourtyard('U1')")
ARMS = {
    'none': None,
    'court': f'(rule "U1 pads" (condition "{OWN}") (constraint clearance (min 0.10mm)))',
    'later': (f'(rule "U1 pads" (condition "{OWN}") (constraint clearance (min 0.10mm)))\n'
              "(rule \"all pads\" (condition \"A.Type == 'Pad' && B.Type == 'Pad'\") "
              "(constraint clearance (min 0.16mm)))"),
    'ignore': (f'(rule "U1 quiet" (severity ignore) (condition "{OWN}") '
               '(constraint clearance (min 0.2mm)))'),
    'front': ("(rule \"U1 front\" (condition \"A.insideFrontCourtyard('U1') && "
              "B.insideFrontCourtyard('U1')\") (constraint clearance (min 0.10mm)))"),
}
#: arm -> (pairs published, of them tagged, the refs published, KiCad's count)
EXPECT = {'none': (4, 0, {'U1', 'U2'}, 4), 'court': (2, 0, {'U2'}, 2),
          'later': (4, 0, {'U1', 'U2'}, 4), 'ignore': (2, 0, {'U2'}, 2),
          'front': (4, 4, {'U1', 'U2'}, 2)}


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def build(d, rule):
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    nets = ''.join(f' (net {i} "/S{i}")\n' for i in range(1, 9))
    parts = ''
    for ref, x0, n0 in (('U1', 110.0, 1), ('U2', 120.0, 5)):
        parts += (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x0} 110)\n'
                  f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                  f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                  f'    (attr smd)\n'
                  f'    (fp_rect (start -0.5 -0.7) (end 2.95 0.7) (stroke (width 0.05) '
                  f'(type solid)) (fill no) (layer "F.CrtYd"))\n')
        for k, (px, net) in enumerate(((0.0, n0), (0.45, n0 + 1), (2.0, n0 + 2),
                                       (2.445, n0 + 3))):
            parts += (f'    (pad "{k + 1}" smd rect (at {px} 0) (size 0.3 0.8) '
                      f'(layers "F.Cu") (net {net} "/S{net}"))\n')
        parts += '  )\n'
    edge = ''
    for x1, y1, x2, y2 in ((100, 100, 135, 100), (135, 100, 135, 120),
                           (135, 120, 100, 120), (100, 120, 100, 100)):
        edge += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
                 f'(stroke (width 0.05) (type solid)))\n')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
                ' (general (thickness 1.6))\n (paper "A4")\n'
                ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
                ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n'
                + nets + parts + edge + ')\n')
    cl = dict(name='Default', clearance=0.16, track_width=0.15, via_diameter=0.6,
              via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
              priority=2147483647)
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {
                       'rules': {'min_clearance': 0.1, 'min_track_width': 0.1},
                       'rule_severities': {}}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cl], 'meta': {'version': 4}}},
                  f, indent=1)
    if rule:
        with open(pcb[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write('(version 1)\n' + rule + '\n')
    return pcb


def check_drc(pcb):
    js = pcb + '.drc.json'
    r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), pcb,
                              '--json', js], accept=True)
    with open(run_utils.evidence(js, 'check_drc --json'), encoding='utf-8') as f:
        return json.load(f), r.stdout


def board_score(pcb):
    r = subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), pcb],
                       capture_output=True, text=True, encoding='utf-8', errors='replace',
                       cwd=REPO, timeout=600)
    acc = run_utils._accident((r.stdout or '') + (r.stderr or ''))
    if acc:
        raise AssertionError(f'BROKEN TEST: board_score died from {acc!r}\n'
                             + (r.stdout or '')[-800:])
    return r.stdout


def kicad_clearance_items(cli, pcb, d):
    out = os.path.join(d, 'kicad.json')
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', out, pcb],
                   capture_output=True, timeout=600)
    with open(run_utils.evidence(out, 'kicad-cli drc json'), encoding='utf-8') as f:
        return [v for v in json.load(f)['violations'] if v['type'] == 'clearance']


def main():
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    tmp = tempfile.mkdtemp(prefix='t1706_')
    try:
        for arm, rule in ARMS.items():
            print(f'== {arm}')
            n, tagged, refs, kicad_n = EXPECT[arm]
            pcb = build(os.path.join(tmp, arm), rule)
            doc, out = check_drc(pcb)
            rows = [i for i in doc['items'] if i.get('accepted') == 'own-footprint-gap']
            ok(f'{arm}: {n} pair(s) published', doc.get('own_footprint_gaps') == n,
               (doc.get('own_footprint_gaps'), [(i['pad_ref'], i['pad_ref2']) for i in rows]))
            ok(f'{arm}: the published pairs are the parts KiCad reports, or may',
               {i['pad_ref'].split('.')[0] for i in rows} == refs,
               sorted(i['pad_ref'] for i in rows))
            ok(f'{arm}: {tagged} tagged with the rule that may clear them',
               doc.get('own_footprint_gaps_unverified', 0) == tagged
               and sum(1 for i in rows if i.get('unverified_rule')) == tagged,
               (doc.get('own_footprint_gaps_unverified'),
                [i.get('unverified_rule') for i in rows]))
            ok(f'{arm}: still not graded', doc['violations'] == 0, doc['violations'])
            s = board_score(pcb)
            said = ('KiCad will report up to %d pair(s) (%d may be cleared' % (n, tagged)
                    if tagged else 'KiCad will report %d pair(s)' % n)
            ok(f'{arm}: board_score says "{said}"', said in s,
               [l for l in s.splitlines() if 'LIBRARY PAD GAPS' in l])
            if arm in ('court', 'ignore'):
                rname = 'U1 pads' if arm == 'court' else 'U1 quiet'
                ok(f'{arm}: check_drc names the rule and the pairs it clears',
                   f"2 of those pair(s) are cleared by a .kicad_dru rule KiCad applies "
                   f"to them ('{rname}' 2" in out, out[-1500:])
            if arm == 'front':
                ok('front: WARNINGS say "up to"',
                   "(up to 4: 4 may be cleared by a .kicad_dru rule" in out
                   and "(may be cleared by rule 'U1 front')" in out, out[-1500:])
            if cli:
                kd = os.path.join(tmp, arm + '_kicad')
                os.makedirs(kd, exist_ok=True)
                subprocess.run(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                                     pcb, os.path.join(kd, 'b.kicad_pcb')],
                               capture_output=True, timeout=120, cwd=REPO)
                items = kicad_clearance_items(cli, os.path.join(kd, 'b.kicad_pcb'), kd)
                ok(f'{arm}: KiCad reports {kicad_n}, as measured', len(items) == kicad_n,
                   [i['description'] for v in items for i in v['items']])
                ok(f'{arm}: KiCad reports no more than check_drc publishes, and as many '
                   f'as it publishes untagged', n - tagged <= len(items) <= n, len(items))

        # the comparator names a published pair KiCad does not report
        sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))
        import kicad_drc_compare as kdc
        nets = frozenset({'/S1', '/S2'})
        row = {'nets': nets, 'pos': (110.0, 110.0), 'pos2': (110.45, 110.0),
               'pad_pair': frozenset({'U1.1', 'U1.2'})}
        kitem = {'type': 'clearance', 'nets': nets, 'pos': (110.2, 110.0),
                 'pads': ['U1.1', 'U1.2']}
        ok('comparator: a row KiCad reports is not named',
           kdc._library_gaps_unreported([kitem], [row]) == ([], []))
        ok('comparator: a row KiCad does not report is named',
           kdc._library_gaps_unreported([], [row]) == ([row], []))
        tagged_row = dict(row, unverified_rule='U1 front')
        ok('comparator: a tagged row KiCad does not report is counted apart',
           kdc._library_gaps_unreported([], [tagged_row]) == ([], [tagged_row]))
        if not cli:
            print('  (kicad-cli not found: KiCad\'s counts are not re-measured)')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
