#!/usr/bin/env python3
"""A coupled-pair rule sets a diff pair's P<->N clearance (#1293).

KiCad lets a pair couple tighter than the clearance everything else keeps with
a custom rule scoped to the pair:

    (rule "USB P-N"
      (condition "AB.isCoupledDiffPair() && A.hasNetclass('USB')")
      (constraint clearance (min 0.1mm)))

design_rules listed isCoupledDiffPair as unmodelled, so the rule was never
read, and route_diff raised the gap to the run clearance (#441), the class
(#530) or another rule (#1145) -- only ever up. Now the predicate is KiCad's
(isCoupledDiffPairFunc over a port of DRC_ENGINE::MatchDpSuffix, conditions
commutative as in DRC_RULE_CONDITION::EvaluateFor), accepted as a positive
term of an AND only, and where such a rule is what KiCad grades P against N
at, that value is the pair's floor instead of those three. check_drc grades
P<->N at it per item type, config.pair_clearance prices it for the partner
neck and via spacing, and the writeback caps Board Setup min_clearance at it
-- KiCad grades the pair gap at max(that minimum, the class gap), so leaving
the run's 0.25 there failed every pair built at 0.15 (measured: 6
diff_pair_gap_out_of_range). The inverse "keep pairs apart" form
(`!AB.isCoupledDiffPair()`) and inDiffPair() are #1295's
(test_1295_diff_pair_rule_forms.py); neither is a coupled-pair rule here.

The fixture is cap_chain (pairs DPA_P/N, DPB_P/N) with a Default class at
clearance 0.25, an HS class (pattern DP*) with diff_pair_gap 0.15, and a rule
`AB.isCoupledDiffPair() && A.hasNetclass('HS')` at 0.15.

Checks:
  A. match_dp_suffix reproduces KiCad's MatchDpSuffix on its own examples
  B. the positive form is a coupled-pair rule; negated and inDiffPair are
     supported but are not (#1295); OR'ed is unsupported
  C. the resolver: P<->N at the rule on every layer; P against the other
     pair at the class; a later layer rule wins its layer; a two-item
     condition matches in either order; the diff-pair-context shortcut
  D. min_rule_clearance and config.pair_clearance carry the rule
  E. route_diff: with the rule both pairs couple at 0.15 and check_drc grades
     them clean at a 0.25 base (and flags the same copper with the rule
     removed); without it, or negated, the gap is raised to 0.25 as before
  F. the CLI over a project whose board minimum is 0.2: the engine gets the
     gap asked for, the writeback records min_clearance 0.15, and KiCad's own
     DRC grades the output clean (skipped, and said, without kicad-cli)

    python3 tests/test_1293_coupled_pair_clearance.py
"""
import contextlib
import copy
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, tool                   # noqa: E402

FAILS = []
RULE = "AB.isCoupledDiffPair() && A.hasNetclass('HS')"
NEGATED = "A.hasNetclass('HS') && !AB.isCoupledDiffPair()"
PAIRS = ['DPA_P', 'DPA_N', 'DPB_P', 'DPB_N']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


def project(min_clearance=0.0):
    """A KiCad 10 project KiCad itself reads (net_settings v4, from a tracked
    board): Default and HS at clearance 0.25, pair gap 0.15, DP* -> HS."""
    with open(os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pro'),
              encoding='utf-8') as fh:
        d = json.load(fh)
    base = copy.deepcopy(next(c for c in d['net_settings']['classes']
                              if c['name'] == 'Default'))
    base.update(clearance=0.25, track_width=0.2, diff_pair_gap=0.15,
                diff_pair_width=0.2, via_diameter=0.6, via_drill=0.3)
    d['net_settings']['classes'] = [base, dict(base, name='HS')]
    d['net_settings']['netclass_patterns'] = [{'netclass': 'HS', 'pattern': 'DP*'}]
    d['net_settings'].pop('netclass_assignments', None)
    d['board']['design_settings']['rules']['min_clearance'] = min_clearance
    return d


def dru(*rules):
    return '(version 1)\n' + ''.join(
        f'(rule "{name}"\n' + (f'  (layer "{layer}")\n' if layer else '')
        + (f'  (condition "{cond}")\n' if cond else '')
        + f'  (constraint {kind} (min {mm}mm)))\n'
        for name, cond, mm, layer, kind in rules)


def stage(d, stem, rules, min_clearance=0.0):
    pcb = os.path.join(d, stem + '.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), pcb)
    with open(os.path.join(d, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(project(min_clearance), fh, indent=2)
    if rules is not None:
        with open(os.path.join(d, stem + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(rules)
    return evidence(pcb, 'staged board')


def coupled_gaps(path):
    """Edge-to-edge distance of the parallel P/N runs of each pair."""
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, path)
    nm = {n.net_id: n.name for n in pcb.nets.values()}
    out = {}
    for base in ('DPA', 'DPB'):
        ps = [s for s in pcb.segments if nm.get(s.net_id) == base + '_P']
        ns = [s for s in pcb.segments if nm.get(s.net_id) == base + '_N']
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
                d = abs((mx - p.start_x) * vp[1] - (my - p.start_y) * vp[0]) / lp
                gaps.add(round(d - (p.width + n.width) / 2, 3))
        out[base] = sorted(gaps)
    return out


# ------------------------------------------------------------------ A-D
def t_match_dp_suffix():
    print('A. match_dp_suffix (KiCad DRC_ENGINE::MatchDpSuffix)')
    from design_rules import match_dp_suffix as m
    cases = {'DATA+1': (1, 'DATA-1', 'DATA'), 'DATA_P_1': (1, 'DATA_N_1', 'DATA_'),
             'D+1_2': (1, 'D-1_2', 'D'), 'A+B+': (1, 'A+B-', 'A+B'), '+': (1, '-', ''),
             'SPIN': (-1, 'SPIP', 'SPI'), 'DDP': (1, 'DDN', 'DD'),
             'USB_N': (-1, 'USB_P', 'USB_'), 'usb_p': (0, '', ''), 'CLK': (0, '', ''),
             '123': (0, '', '')}
    bad = {n: (m(n), want) for n, want in cases.items() if m(n) != want}
    check('every case matches KiCad', not bad, f'{bad}')


def t_support():
    print('B. which forms are supported')
    from design_rules import parse_dru
    text = dru(('pos', RULE, 0.15, None, 'clearance'),
               ('neg', NEGATED, 0.3, None, 'clearance'),
               ('or', "AB.isCoupledDiffPair() || A.NetClass == 'HS'", 0.15, None, 'clearance'),
               ('indp', "A.inDiffPair('*')", 0.15, None, 'clearance'))
    rules = {r.name: r for r in parse_dru(text)[0]}
    check('AB.isCoupledDiffPair() ANDed in: supported, a coupled rule',
          rules['pos'].unsupported is None and rules['pos'].coupled)
    for n in ('neg', 'indp'):
        check(f'{n}: supported (#1295), and not a coupled-pair rule',
              rules[n].unsupported is None and not rules[n].coupled,
              rules[n].unsupported or '')
    check('or: still unsupported, with the reason said',
          bool(rules['or'].unsupported), rules['or'].unsupported or '')


def _rules(d, text):
    from design_rules import DesignRules
    from kicad_parser import parse_kicad_pcb
    pcb_path = stage(d, 'r', text)
    pcb = quiet(parse_kicad_pcb, pcb_path)
    ids = {n.name: i for i, n in pcb.nets.items()}
    return DesignRules.from_project(pcb, pcb_path,
                                    copper_layers=pcb.board_info.copper_layers), ids


def t_resolver():
    print('C. the resolver')
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        dr, ids = _rules(d, dru(('HS P-N', RULE, 0.15, None, 'clearance')))
        p, n, other = ids['DPA_P'], ids['DPA_N'], ids['DPB_N']
        got = {l: dr.coupled_pair_clearance(p, n, l) for l in ('F.Cu', 'B.Cu')}
        check('P<->N at the rule on every layer',
              all(c is not None and c.min == 0.15 and c.min_rule == 'HS P-N'
                  for c in got.values()), f'{got}')
        c = dr.resolve('clearance', dr.item_for_net(p, 'track', 'F.Cu'),
                       dr.item_for_net(other, 'track', 'F.Cu'), 'F.Cu')
        check('P against the other pair: the class 0.25, not the rule',
              c.min == 0.25 and not c.min_rule_coupled, f'{c.min} {c.source}')
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        dr, ids = _rules(d, dru(('HS P-N', RULE, 0.15, None, 'clearance'),
                                ('F wide', None, 0.3, 'F.Cu', 'clearance')))
        p, n = ids['DPA_P'], ids['DPA_N']
        check('a later layer rule wins its layer (KiCad: last match per field)',
              dr.coupled_pair_clearance(p, n, 'F.Cu') is None
              and dr.coupled_pair_clearance(p, n, 'B.Cu') is not None)
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        dr, ids = _rules(d, dru(('via side', "AB.isCoupledDiffPair() && B.Type == 'via'",
                                 0.15, None, 'clearance')))
        p, n = ids['DPA_P'], ids['DPA_N']
        check('a two-item condition matches as (A, B)',
              dr.coupled_pair_clearance(p, n, 'F.Cu', ('track', 'via')) is not None)
        check('...and in the swapped order, as KiCad evaluates it',
              dr.coupled_pair_clearance(p, n, 'F.Cu', ('via', 'track')) is not None)
        check('...and not where neither order holds',
              dr.coupled_pair_clearance(p, n, 'F.Cu', ('track', 'track')) is None)
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        dr, ids = _rules(d, dru(('gap', RULE, 0.12, None, 'diff_pair_gap')))
        c = dr.resolve('diff_pair_gap', dr.item_for_net(ids['DPA_P'], 'track'), None)
        check('diff_pair_gap needs no B when the complement net exists (KiCad shortcut)',
              c.min == 0.12, f'{c.min} {c.source}')


def t_writeback_and_pricing():
    print('D. the writeback floor and pair_clearance')
    from kicad_dru import min_rule_clearance
    from routing_config import GridRouteConfig
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        pcb = stage(d, 'w', dru(('HS P-N', RULE, 0.15, None, 'clearance'),
                                ('B', None, 0.2, 'B.Cu', 'clearance')))
        check('min_rule_clearance counts the coupled-pair rule',
              min_rule_clearance(pcb) == 0.15, f'{min_rule_clearance(pcb)}')
        pcb = stage(d, 'v', dru(('neg', NEGATED, 0.12, None, 'clearance')))
        check('...and not a keep-apart one', min_rule_clearance(pcb) is None)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.25)
    cfg.coupled_pair_clearances = {frozenset((1, 3)): {
        'track': {'F.Cu': 0.15}, 'layer': {'F.Cu': 0.16}, 'stack': 0.17}}
    check('pair_clearance: track / layer / stack read the pair map',
          (cfg.pair_clearance(1, 3, 'F.Cu', kind='track'),
           cfg.pair_clearance(3, 1, 'F.Cu'), cfg.pair_clearance(1, 3, kind='stack'))
          == (0.15, 0.16, 0.17))
    check('...a layer the rule does not govern keeps the run value',
          cfg.pair_clearance(1, 3, 'B.Cu') == 0.25)
    check('...and any other pair of nets too', cfg.pair_clearance(1, 2, 'F.Cu') == 0.25)
    check('max_pair_clearance and pair_clearance_inert see the map',
          not cfg.pair_clearance_inert() and cfg.max_pair_clearance() >= 0.25)


# ------------------------------------------------------------------ E-F
def _route(d, stem, rules):
    import route_diff
    src = stage(d, stem, rules)
    out = os.path.join(d, stem + '_o.kicad_pcb')
    with open(os.path.join(d, stem + '_o.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(project(), fh)
    if rules is not None:
        shutil.copy(os.path.join(d, stem + '.kicad_dru'),
                    os.path.join(d, stem + '_o.kicad_dru'))
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        route_diff.batch_route_diff_pairs(src, out, PAIRS, enable_layer_switch=True,
                                          diff_pair_gap=0.15, diff_pair_gap_from_class=True)
    return log.getvalue(), evidence(out, 'routed board')


def _drc(path):
    """check_drc at margin 0, past the writer's 1 nm coordinate rounding
    (kicad_writer.EDGE_ROUNDING_MM): a diagonal coupled exactly at the rule
    reads up to sqrt(2) nm inside it in the file."""
    from check_drc import run_drc
    from kicad_writer import EDGE_ROUNDING_MM
    return [v for v in quiet(run_drc, path, clearance=0.25, quiet=True,
                             print_summary=False, check_sizes=False,
                             clearance_margin=0.0)
            if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM]


def t_route_diff():
    print('E. route_diff on cap_chain')
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        log, out = _route(d, 'rule', dru(('HS P-N', RULE, 0.15, None, 'clearance')))
        g = coupled_gaps(out)
        check('the rule is read for both pairs (the path under test)',
              log.count('graded at 0.15mm by the coupled-pair rule') == 2)
        check('both pairs couple at the rule, 0.15 mm', all(
            v and all(abs(x - 0.15) < 0.002 for x in v) for v in g.values()), f'{g}')
        check('check_drc grades them clean at a 0.25 base', not _drc(out))
        os.remove(os.path.join(d, 'rule_o.kicad_dru'))
        v = _drc(out)
        check('...and flags the same copper once the rule is gone (not blind)',
              any({x.get('net1'), x.get('net2')} in ({'DPA_P', 'DPA_N'}, {'DPB_P', 'DPB_N'})
                  for x in v), f'{len(v)} violations')
    for label, rules in (('no rule', None),
                         ('the negated form', dru(('neg', NEGATED, 0.3, None, 'clearance')))):
        with tempfile.TemporaryDirectory(prefix='t1293_') as d:
            log, out = _route(d, 'x', rules)
            g = coupled_gaps(out)
            check(f'{label}: the gap is raised to the clearance as before',
                  '#1293' not in log and all(
                      v and all(abs(x - 0.25) < 0.002 for x in v) for v in g.values()),
                  f'{g}')


def t_cli_and_kicad():
    print('F. the CLI, the writeback and KiCad')
    from kicad_oracle import find_kicad_cli
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        src = stage(d, 'c', dru(('HS P-N', RULE, 0.15, None, 'clearance')), 0.2)
        out = os.path.join(d, 'c_o.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool('route_diff.py'), src,
                            '--output', out, '--nets', 'DP*'],
                           cwd=ROOT, capture_output=True, text=True, timeout=900)
        check('route_diff.py exits 0', r.returncode == 0, r.stderr[-400:])
        if r.returncode != 0:
            return
        g = coupled_gaps(evidence(out, 'routed board'))
        check('the engine got the gap asked for: both pairs at 0.15', all(
            v and all(abs(x - 0.15) < 0.002 for x in v) for v in g.values()), f'{g}')
        with open(os.path.join(d, 'c_o.kicad_pro'), encoding='utf-8') as fh:
            mc = json.load(fh)['board']['design_settings']['rules'].get('min_clearance')
        check('the writeback records min_clearance 0.15, not the run 0.25 or the 0.2 '
              'it read', mc == 0.15, f'{mc}')
        cli = find_kicad_cli(warn=False)
        if not cli:
            print('  SKIP: no kicad-cli found; KiCad\'s own grade is unchecked here')
            return
        rep = os.path.join(d, 'drc.json')
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '-o', rep, out], capture_output=True, text=True, timeout=600)
        with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
            k = json.load(fh)
        bad = [v['type'] for v in k['violations']
               if v['type'] in ('clearance', 'diff_pair_gap_out_of_range')]
        check('KiCad: no clearance or pair-gap violation', not bad, f'{bad[:6]}')
        check('KiCad: nothing unconnected', not k.get('unconnected_items'))
    with tempfile.TemporaryDirectory(prefix='t1293_') as d:
        # An explicit gap is the call's own, so it reaches the pairs only if
        # main() hands the engine the gap asked for (the GUI tab always has).
        src = stage(d, 'e', dru(('HS P-N', RULE, 0.15, None, 'clearance')), 0.2)
        out = os.path.join(d, 'e_o.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool('route_diff.py'), src,
                            '--output', out, '--nets', 'DP*', '--diff-pair-gap', '0.15'],
                           cwd=ROOT, capture_output=True, text=True, timeout=900)
        g = coupled_gaps(evidence(out, 'routed board')) if r.returncode == 0 else {}
        check('--diff-pair-gap 0.15 given: both pairs at 0.15', bool(g) and all(
            v and all(abs(x - 0.15) < 0.002 for x in v) for v in g.values()),
            f'{g} rc={r.returncode}')


if __name__ == '__main__':
    t_match_dp_suffix()
    t_support()
    t_resolver()
    t_writeback_and_pricing()
    t_route_diff()
    t_cli_and_kicad()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')
