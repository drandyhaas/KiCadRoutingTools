#!/usr/bin/env python3
"""A net class's diff_pair_gap below its clearance sets P<->N (#1542).

KiCad's DRC engine adds an implicit rule per net class: when
`0 < diff_pair_gap < clearance`, the P and N of a pair in the class (the two
names MatchDpSuffix couples) are graded against each other at the class gap,
floored at the board minimum clearance, on every item kind. Its report names
it "netclass 'HS' (diff pair)". KRT graded P<->N at the class clearance, so
check_drc / board_score / check_complete flagged copper KiCad passes, and
route_diff raised the coupling gap to the clearance ("#530 ... KiCad grades
P<->N as clearance", untrue for such a class).

The rule is modelled once, in design_rules (step 2b, after the class
clearance and before every custom rule), and read by check_drc's coupled-pair
channel and by route_diff's per-pair floor (coupled_pair_floors), the same
channel a #1293 coupled-pair rule takes.

The fixture is cap_chain (pairs DPA_P/N, DPB_P/N): Default at 0.15, class HS
(DP*) at clearance 0.25 with diff_pair_gap GAP, board minimum 0, no rules
file. Measured on kicad-cli 10.0.6 with the pairs built at 0.15: GAP 0.15
grades 0 P<->N clearance errors, 0.20 grades 28 "netclass 'HS' (diff pair)
clearance 0.2000 mm", 0.25 grades 30 at 0.25.

Checks:
  A. the resolver: P<->N at the class gap, named, a coupled rule; P against
     the other pair at the class clearance; no rule when the gap is not
     below the clearance; a class that sets no gap takes Default's; the
     board minimum floors it; a later .kicad_dru rule replaces it
  B. check_drc grades copper built at 0.15 at the class gap: clean at GAP
     0.15 (it flagged 28), the P<->N runs flagged at 0.20 and 0.25 (not
     blind)
  C. route_diff builds both pairs at the class gap 0.15 (it raised them to
     0.25), with the class rule named, neither member necked, check_drc clean
  D. KiCad grades that routed board clean (skipped, and said, without
     kicad-cli)
  E. --clearance-ceiling clamping the class to its gap or below makes no
     rule in the output project: the gap asked for stands, KiCad clean

    python3 tests/test_1542_class_pair_gap.py
"""
import contextlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, tool                  # noqa: E402

FAILS = []
PAIRS = ['DPA_P', 'DPA_N', 'DPB_P', 'DPB_N']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


def _cls(name, clr, gap, prio):
    d = {'name': name, 'clearance': clr, 'track_width': 0.2, 'via_diameter': 0.6,
         'via_drill': 0.3, 'diff_pair_width': 0.2, 'diff_pair_via_gap': 0.25,
         'microvia_diameter': 0.3, 'microvia_drill': 0.1, 'priority': prio,
         'bus_width': 12, 'wire_width': 6, 'line_style': 0,
         'pcb_color': 'rgba(0, 0, 0, 0.000)', 'schematic_color': 'rgba(0, 0, 0, 0.000)'}
    if gap is not None:
        d['diff_pair_gap'] = gap
    return d


def write_project(pcb, gap, min_clearance=0.0, default_gap=0.15):
    """Default 0.15 (pair gap ``default_gap``), HS (DP*) at clearance 0.25
    with diff_pair_gap ``gap`` (None: HS sets none of its own)."""
    stem = os.path.splitext(pcb)[0]
    pro = {'board': {'design_settings': {'rules': {'min_clearance': min_clearance,
                                                   'min_track_width': 0.0}}},
           'net_settings': {'meta': {'version': 4},
                            'classes': [_cls('Default', 0.15, default_gap, 2147483647),
                                        _cls('HS', 0.25, gap, 0)],
                            'netclass_patterns': [{'netclass': 'HS', 'pattern': 'DP*'}]},
           'meta': {'filename': os.path.basename(stem) + '.kicad_pro', 'version': 3}}
    with open(stem + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(pro, fh, indent=2)


def stage(d, stem, gap, dru=None, **kw):
    pcb = os.path.join(d, stem + '.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), pcb)
    write_project(pcb, gap, **kw)
    if dru is not None:
        with open(os.path.join(d, stem + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return evidence(pcb, 'staged board')


def coupled_gaps(path):
    """Edge-to-edge distance of the parallel P/N runs of each pair, and the
    width of every P/N segment."""
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, path)
    nm = {n.net_id: n.name for n in pcb.nets.values()}
    out, widths = {}, set()
    for base in ('DPA', 'DPB'):
        ps = [s for s in pcb.segments if nm.get(s.net_id) == base + '_P']
        ns = [s for s in pcb.segments if nm.get(s.net_id) == base + '_N']
        widths |= {round(s.width, 4) for s in ps + ns}
        gaps = set()
        for p in ps:
            vp = (p.end_x - p.start_x, p.end_y - p.start_y)
            lp = math.hypot(*vp)
            for n in ns:
                vn = (n.end_x - n.start_x, n.end_y - n.start_y)
                ln = math.hypot(*vn)
                if (p.layer != n.layer or lp < 0.5 or ln < 0.5
                        or abs(vp[0] * vn[1] - vp[1] * vn[0]) > 1e-6 * lp * ln):
                    continue
                mx, my = (n.start_x + n.end_x) / 2, (n.start_y + n.end_y) / 2
                dd = abs((mx - p.start_x) * vp[1] - (my - p.start_y) * vp[0]) / lp
                gaps.add(round(dd - (p.width + n.width) / 2, 3))
        out[base] = sorted(gaps)
    return out, widths


def pn_violations(path):
    """The check_drc CLI's violations between the P and the N of one pair
    (it reads the board's classes and rules itself, as board_score and
    check_complete run it)."""
    rep = path + '.krt.json'
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('check_drc.py'), path,
                        '--json', rep], cwd=ROOT, capture_output=True, text=True,
                       timeout=600)
    if r.returncode not in (0, 1):
        raise AssertionError(f'check_drc failed: {r.stderr[-400:]}')
    with open(evidence(rep, 'check_drc report'), encoding='utf-8') as fh:
        items = json.load(fh)['items']
    return [x for x in items
            if {x.get('net1'), x.get('net2')} in ({'DPA_P', 'DPA_N'}, {'DPB_P', 'DPB_N'})]


# ------------------------------------------------------------------ A
def t_resolver():
    print('A. the resolver (design_rules)')
    from design_rules import DesignRules
    from kicad_parser import parse_kicad_pcb

    def rules(gap, **kw):
        d = tempfile.mkdtemp(prefix='t1542_')
        pcb_path = stage(d, 'r', gap, **kw)
        pcb = quiet(parse_kicad_pcb, pcb_path)
        ids = {n.name: i for i, n in pcb.nets.items()}
        dr = DesignRules.from_project(pcb, pcb_path,
                                      copper_layers=pcb.board_info.copper_layers)
        shutil.rmtree(d, ignore_errors=True)
        return dr, ids

    def pn(dr, ids, a='DPA_P', b='DPA_N', types=('track', 'track')):
        return dr.resolve('clearance', dr.item_for_net(ids[a], types[0], 'F.Cu'),
                          dr.item_for_net(ids[b], types[1], 'F.Cu'), 'F.Cu')

    dr, ids = rules(0.15)
    c = pn(dr, ids)
    check("P<->N at the class gap 0.15, named \"netclass 'HS' (diff pair)\"",
          c.min == 0.15 and c.min_rule == "netclass 'HS' (diff pair)"
          and c.min_rule_coupled, f'{c}')
    check('...on every item kind (via against pad)',
          pn(dr, ids, types=('via', 'pad')).min == 0.15)
    check('...and in either order', pn(dr, ids, 'DPA_N', 'DPA_P').min == 0.15)
    check('P against the other pair: the class clearance 0.25',
          pn(dr, ids, 'DPA_P', 'DPB_N').min == 0.25)
    check('coupled_pair_clearance answers it with no .kicad_dru',
          dr.coupled_pair_clearance(ids['DPA_P'], ids['DPA_N'], 'B.Cu') is not None)
    dr, ids = rules(0.25)
    check('a gap equal to the clearance makes no rule: 0.25',
          pn(dr, ids).min == 0.25 and not pn(dr, ids).min_rule_coupled)
    dr, ids = rules(None)
    check("a class that sets no gap takes Default's 0.15 (KiCad 10.0.6 grades it so)",
          pn(dr, ids).min == 0.15)
    dr, ids = rules(0.15, min_clearance=0.21)
    c = pn(dr, ids)
    check('the board minimum 0.21 floors it, and it still governs the pair',
          c.min == 0.21 and c.min_rule_coupled, f'{c}')
    dr, ids = rules(0.15, dru='(version 1)\n(rule "F wide" (layer "F.Cu") '
                              '(constraint clearance (min 0.3mm)))\n')
    c = pn(dr, ids)
    check('a .kicad_dru rule replaces it (custom rules come after it)',
          c.min == 0.3 and not c.min_rule_coupled, f'{c}')


# ------------------------------------------------------------------ B-D
def _route(d, stem, gap, dru=None):
    import route_diff
    src = stage(d, stem, gap, dru=dru)
    out = os.path.join(d, stem + '_o.kicad_pcb')
    if dru is not None:
        shutil.copy(os.path.join(d, stem + '.kicad_dru'),
                    os.path.join(d, stem + '_o.kicad_dru'))
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        ok, bad, _t = route_diff.batch_route_diff_pairs(
            src, out, PAIRS, enable_layer_switch=True, diff_pair_gap=0.15,
            diff_pair_gap_from_class=True)
    return log.getvalue(), (ok, bad), evidence(out, 'routed board')


def t_grade():
    """The grader on its own: copper built at 0.15 by a #1293 coupled-pair
    rule (which route_diff honours with or without this change), graded with
    that rule removed -- the class alone, at three class gaps."""
    print('B. check_drc on 0.15 copper, the class alone')
    with tempfile.TemporaryDirectory(prefix='t1542_') as d:
        _log, _res, out = _route(
            d, 'rule', 0.15, dru='(version 1)\n(rule "HS P-N" (constraint clearance '
                                 "(min 0.15mm))\n  (condition \"AB.isCoupledDiffPair() "
                                 "&& A.hasNetclass('HS')\"))\n")
        gaps, _w = coupled_gaps(out)
        check('fixture: both pairs coupled at 0.15', all(
            v and all(abs(x - 0.15) < 0.002 for x in v) for v in gaps.values()), f'{gaps}')
        os.remove(os.path.join(d, 'rule_o.kicad_dru'))
        for gap, want in ((0.15, None), (0.20, 0.2), (0.25, 0.25)):
            write_project(out, gap)
            v = pn_violations(out)
            req = sorted({round(x.get('required_mm', 0.25), 4) for x in v})
            if want is None:
                check(f'class gap {gap:g}: no P<->N violation (KiCad: 0)', not v,
                      f'{len(v)} at {req}')
            else:
                check(f'class gap {gap:g}: the P<->N runs flagged at {want:g} (not blind)',
                      bool(v) and req == [want], f'{len(v)} at {req}')


def t_route_and_kicad():
    print('C. route_diff under the class alone, D. KiCad')
    from routing_defaults import TRACK_WIDTH
    with tempfile.TemporaryDirectory(prefix='t1542_') as d:
        log, (ok, bad), out = _route(d, 'cls', 0.15)
        check('fixture: both pairs route', (ok, bad) == (2, 0), f'{(ok, bad)}')
        named = [ln.strip() for ln in log.splitlines()
                 if "#1542" in ln and "netclass 'HS' (diff pair)" in ln]
        check('the class rule is named for both pairs', len(named) >= 2, f'{named}')
        check('no #530 / #441 raise to the clearance',
              not re.search(r'#530: DP|#441: DP', log))
        gaps, widths = coupled_gaps(out)
        check('both pairs couple at the class gap 0.15 (was 0.25)', all(
            v and all(abs(x - 0.15) < 0.002 for x in v) for v in gaps.values()), f'{gaps}')
        check('neither member is necked: one width for all four',
              widths == {round(TRACK_WIDTH, 4)}, f'{sorted(widths)}')
        v = pn_violations(out)
        check('check_drc grades it clean', not v, f'{len(v)}')
        bad = kicad_bad(out)
        if bad is not None:
            check('KiCad: no clearance or pair-gap violation', not bad, f'{bad[:3]}')


def kicad_bad(path):
    """KiCad's clearance / pair-gap violations on ``path``; None (said) with
    no kicad-cli."""
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        print("  SKIP: no kicad-cli found; KiCad's own grade is unchecked here")
        return None
    rep = path + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                    '--all-track-errors', '-o', rep, path],
                   capture_output=True, text=True, timeout=600)
    with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
        k = json.load(fh)
    return [x['description'] for x in k['violations']
            if x['type'] in ('clearance', 'diff_pair_gap_out_of_range')]


def t_ceiling():
    """--clearance-ceiling clamps the class clearance in the output project.
    Clamped to 0.15, under its 0.2 gap, the class makes no "(diff pair)" rule
    there, so KiCad grades P<->N at 0.15 and the gap asked for (0.15) stands
    rather than being raised to the class gap."""
    print('E. the CLI under --clearance-ceiling 0.15, class gap 0.20')
    with tempfile.TemporaryDirectory(prefix='t1542_') as d:
        src = stage(d, 'ceil', 0.20)
        out = os.path.join(d, 'ceil_o.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool('route_diff.py'), src, out,
                            '--nets', 'DP*', '--diff-pair-gap', '0.15',
                            '--clearance-ceiling', '0.15'],
                           cwd=ROOT, capture_output=True, text=True, timeout=900)
        check('route_diff.py exits 0', r.returncode == 0, r.stderr[-400:])
        if r.returncode != 0:
            return
        gaps, _w = coupled_gaps(evidence(out, 'routed board'))
        check('both pairs at the 0.15 asked for, no #1542 floor',
              '#1542' not in r.stdout and all(
                  v and all(abs(x - 0.15) < 0.002 for x in v) for v in gaps.values()),
              f'{gaps}')
        bad = kicad_bad(out)
        if bad is not None:
            check('KiCad: no clearance or pair-gap violation', not bad, f'{bad[:3]}')


if __name__ == '__main__':
    t_resolver()
    t_grade()
    t_route_and_kicad()
    t_ceiling()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')
