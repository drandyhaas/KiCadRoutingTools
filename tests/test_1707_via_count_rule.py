#!/usr/bin/env python3
"""#1707: a .kicad_dru via_count rule is named NOT honoured and graded.

`(constraint via_count (min N) (max M))` sat in design_rules' CONSUMED_KINDS,
so route.py listed nothing (`design_rules.unsupported_rules` was []), and
check_drc and board_score graded it nowhere, while KiCad failed the board
with `too_many_vias`. Now no routing step claims it -- a net is routed with
the vias it needs, completion first -- so every step that reads the rules
names it NOT honoured, and check_drc grades it per net as KiCad does
(`via-count-out-of-range`), which board_score counts in `drc`.

The board has no routing: parts whose pads carry four nets, vias placed by
hand. Rules (KiCad's verdict measured on kicad-cli 10.0.6, and re-measured
here when it is installed):
  a_max  /A at most 2: 3 vias, one of them stray       too many, actual 3
  b_min  /B at least 1: two pads, no via               too few, actual 0
  c_min  /C at least 1: one pad and nothing else       too few, actual 0
  d_via  A.Type == 'Via' && /D at most 0: 1 via        too many (the vias bind
         it, the pads nothing)
  e_max  /E at most 1: 1 via                           within
  f_crt  insideCourtyard('J9'), at least 1, FIRST in the file, so it is asked
         only of items no later rule binds -- the no-net pad and /D's pad
         (d_via binds only vias). KiCad: the part
         draws no courtyard, so it binds nothing. The resolver cannot read the
         term, so check_drc names it NOT GRADED (via_count) and board_score
         lists it ungraded

    python3 tests/test_1707_via_count_rule.py
"""
import json
import os
import re
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
NETS = ['/A', '/B', '/C', '/D', '/E']
#: (pad net, x) per one-pad part; '' is a pad with no net
PADS = [('/A', 110), ('/A', 114), ('/B', 118), ('/B', 122), ('/C', 126), ('/D', 130),
        ('/E', 134), ('/E', 138), ('', 142)]
VIAS = [('/A', 111, 108), ('/A', 113, 108), ('/A', 140, 116),    # the last one stray
        ('/D', 131, 108), ('/E', 135, 108)]
DRU = """(version 1)
(rule "f_crt" (condition "A.insideCourtyard('J9')") (constraint via_count (min 1)))
(rule "a_max" (condition "A.NetName == '/A'") (constraint via_count (max 2)))
(rule "b_min" (condition "A.NetName == '/B'") (constraint via_count (min 1)))
(rule "c_min" (condition "A.NetName == '/C'") (constraint via_count (min 1)))
(rule "d_via" (condition "A.Type == 'Via' && A.NetName == '/D'") (constraint via_count (max 0)))
(rule "e_max" (condition "A.NetName == '/E'") (constraint via_count (max 1)))
"""
#: net -> (rule, bound, limit, vias), as KiCad reports them
GRADED = {'/A': ('a_max', 'max', 2, 3), '/B': ('b_min', 'min', 1, 0),
          '/C': ('c_min', 'min', 1, 0), '/D': ('d_via', 'max', 0, 1)}


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def build(d):
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    num = {n: i + 1 for i, n in enumerate(NETS)}
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n'
         + ''.join(f' (net {num[n]} "{n}")\n' for n in NETS))
    for i, (net, x) in enumerate(PADS):
        tag = f' (net {num[net]} "{net}")' if net else ''
        b += (f'  (footprint "T:J{i + 1}" (layer "F.Cu") (at {x} 110)\n'
              f'    (property "Reference" "J{i + 1}" (at 0 -2 0) (layer "F.SilkS") (hide yes) '
              f'(effects (font (size 1 1) (thickness 0.15))))\n    (attr smd)\n'
              f'    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu"){tag})\n  )\n')
    for k, (net, x, y) in enumerate(VIAS):
        b += (f'  (via (at {x} {y}) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
              f'(net {num[net]}) (uuid "00000000-0000-0000-0000-00000000000{k}"))\n')
    for x1, y1, x2, y2 in ((100, 100, 150, 100), (150, 100, 150, 120),
                           (150, 120, 100, 120), (100, 120, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')
    cl = dict(name='Default', clearance=0.2, track_width=0.2, via_diameter=0.6,
              via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
              priority=2147483647)
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {
                       'rules': {'min_clearance': 0.1, 'min_track_width': 0.1},
                       'rule_severities': {}}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cl], 'meta': {'version': 4}}},
                  f, indent=1)
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as f:
        f.write(DRU)
    return pcb


def main():
    tmp = tempfile.mkdtemp(prefix='t1707_')
    try:
        pcb = build(os.path.join(tmp, 'b'))

        # the router's half: every rule-reading step names the rule
        from kicad_parser import parse_kicad_pcb
        from design_rules import DesignRules
        dr = DesignRules.from_project(parse_kicad_pcb(pcb), pcb)
        un = dict(dr.unsupported())
        ok('design_rules: every evaluable via_count rule is NOT honoured',
           {'a_max', 'b_min', 'c_min', 'e_max'} <= set(un)
           and all('via_count' in un[n] for n in ('a_max', 'b_min', 'c_min', 'e_max')),
           un)
        lines = dr.report_lines()
        ok("design_rules: the console says NOT honoured and that KiCad enforces it",
           any(l.startswith(".kicad_dru rule 'a_max': NOT honoured (via_count:")
               and l.endswith('KiCad will still enforce it') for l in lines), lines)

        # the grader's half
        js = pcb + '.drc.json'
        r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), pcb,
                                  '--json', js], code=1)
        with open(run_utils.evidence(js, 'check_drc --json'), encoding='utf-8') as f:
            doc = json.load(f)
        rows = {v['net1']: (v['rule'], v['bound'], v['limit'], v['vias'])
                for v in doc['items'] if v.get('type') == 'via-count-out-of-range'}
        ok('check_drc: each net out of its rule is a row, as KiCad counts', rows == GRADED,
           rows)
        ok('check_drc: the rows are violations', doc['violations'] == len(GRADED),
           doc['violations'])
        ok('check_drc: a term the resolver cannot read is NOT GRADED by name',
           "NOT GRADED (via_count): rule 'f_crt'" in r.stdout
           and '2 net(s) it may bind' in r.stdout, r.stdout[-1500:])

        sj = pcb + '.score.json'
        s_run = subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), pcb,
                                     '--json', sj], capture_output=True, text=True,
                               encoding='utf-8', errors='replace', cwd=REPO, timeout=600)
        acc = run_utils._accident((s_run.stdout or '') + (s_run.stderr or ''))
        if acc:
            raise AssertionError(f'BROKEN TEST: board_score died from {acc!r}\n'
                                 + (s_run.stdout or '')[-800:])
        with open(run_utils.evidence(sj, 'board_score --json'), encoding='utf-8') as f:
            s = json.load(f)
        drc = s['components']['drc']
        ok('board_score: drc counts the rows (blocking)',
           drc['by_type'].get('via-count-out-of-range') == len(GRADED)
           and s['blocking_by']['drc'] >= len(GRADED), (drc.get('by_type'), s['blocking_by']))
        ok('board_score: the rule it cannot read is listed ungraded',
           'f_crt' in (drc.get('ungraded_rules') or ()), drc.get('ungraded_rules'))

        from kicad_oracle import find_kicad_cli
        cli = find_kicad_cli(warn=False)
        if cli:
            kd = os.path.join(tmp, 'k')
            os.makedirs(kd, exist_ok=True)
            subprocess.run(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), pcb,
                                 os.path.join(kd, 'b.kicad_pcb')],
                           capture_output=True, timeout=120, cwd=REPO)
            out = os.path.join(kd, 'k.json')
            subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o',
                            out, os.path.join(kd, 'b.kicad_pcb')], capture_output=True,
                           timeout=600)
            with open(run_utils.evidence(out, 'kicad-cli drc json'), encoding='utf-8') as f:
                kv = [v for v in json.load(f)['violations'] if v['type'] == 'too_many_vias']
            krows = {}
            for v in kv:
                m = re.search(r"\(rule '(.*)' (min|max) count (\d+); actual (\d+)\)",
                              v['description'])
                net = re.search(r'\[(.*?)\]', v['items'][0]['description']).group(1)
                krows[net] = (m.group(1), m.group(2), int(m.group(3)), int(m.group(4)))
            ok('KiCad reports what was measured, and check_drc\'s rows are KiCad\'s',
               krows == GRADED == rows, (krows, rows))
        else:
            print('  (kicad-cli not found: KiCad\'s rows are not re-measured)')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
