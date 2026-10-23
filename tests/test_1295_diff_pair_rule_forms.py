#!/usr/bin/env python3
"""KiCad's other two diff-pair rule forms are honoured (#1295).

#1293 read `AB.isCoupledDiffPair()` as a positive AND term. Two more uses of
KiCad's diff-pair predicates were reported unsupported and ignored by the
router and check_drc alike, while KiCad enforced them:

1. "Keep pairs apart" (three corpus boards, e.g. bmc-reference-carrier-board):

       (rule "100Diff_clearance"
         (condition "A.NetClass == '100Ohm-diff' && !AB.isCoupledDiffPair()
                     && A.Type == 'Track' && B.Type == 'Track'")
         (constraint clearance (min 0.2mm)))

   A class member's track keeps 0.2 from every other track EXCEPT its own
   partner. It is the #735 track channel's rule plus a partner exemption
   (`TrackRule.except_coupled`), and the partner is decided on the NAMES by
   KiCad's MatchDpSuffix (`kicad_dru.nets_coupled`). The router's per-net map
   over-approximates (the partner's stamp is raised too), so the P<->N sites
   read `config.track_partner_clearances` instead, #1145 exempts the pair,
   and check_drc and placement grade pair-exact. A (layer ...) clause is read
   too (pair-exact; the router prices it on every layer).
2. `A.inDiffPair('pattern')` (Obsidian, cm4-baseboard, bmc-reference-osm), the
   condition of layer-scoped track_width / diff_pair_gap rules, is KiCad's
   inDiffPairFunc: a pair suffix, the complement net on the board, the base
   name wildcard-matched (or the base before a trailing '_').

Checks:
  A. which forms parse: both, in the resolver and the track channel; an OR'ed
     coupled term, AB.inDiffPair() and an empty pattern stay unsupported
  B. semantics: inDiffPair on KiCad's rules; nets_coupled on names (DP/DM is
     no pair to KiCad); track_pair_clearance's exemption and layer clause
  C. install_track_clearances: the partner map, and pair_clearance reads it
  D. route_diff on cap_chain under a 0.3 keep-apart rule: both pairs couple at
     0.15 (no #1145 raise), check_drc grades them clean and flags the same
     copper under the rule without the term; KiCad agrees both ways
  E. route.py on splitflap_driver: the keep-apart rule moves copper (foreign
     tracks keep 0.45 from /LED_*; the rule-less control comes closer and
     check_drc flags it once the rule is added); KiCad agrees
  F. inDiffPair rules: route_diff's layer widths take an inDiffPair('*') rule,
     not an inDiffPair('XYZ*') one, and an inDiffPair('DPA') rule only for
     DPA; check_drc grades an inDiffPair('DPA') minimum width on DPA alone,
     and KiCad agrees

    python3 tests/test_1295_diff_pair_rule_forms.py
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
APART = ("A.NetClass == 'HS' && !AB.isCoupledDiffPair() "
         "&& A.Type == 'Track' && B.Type == 'Track'")
PLAIN = "A.NetClass == 'HS' && A.Type == 'Track' && B.Type == 'Track'"
PAIRS = ['DPA_P', 'DPA_N', 'DPB_P', 'DPB_N']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


def project(clearance=0.15, gap=0.15, width=0.2, cls='HS', pattern='DP*'):
    """A KiCad 10 project KiCad itself reads (net_settings v4, from a tracked
    board): Default and ``cls`` (nets matching ``pattern``) alike."""
    with open(os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pro'),
              encoding='utf-8') as fh:
        d = json.load(fh)
    base = copy.deepcopy(next(c for c in d['net_settings']['classes']
                              if c['name'] == 'Default'))
    base.update(clearance=clearance, track_width=width, diff_pair_gap=gap,
                diff_pair_width=width, via_diameter=0.6, via_drill=0.3)
    d['net_settings']['classes'] = [base, dict(base, name=cls)]
    d['net_settings']['netclass_patterns'] = [{'netclass': cls, 'pattern': pattern}]
    d['net_settings'].pop('netclass_assignments', None)
    # Board minimums below every value under test, so none of them clamps.
    d['board']['design_settings']['rules'].update(min_clearance=0.0, min_track_width=0.1)
    return d


def dru(*rules):
    """(name, condition, kind, (min|opt), mm, layer) -> .kicad_dru text."""
    return '(version 1)\n' + ''.join(
        f'(rule "{name}"\n' + (f'  (layer "{layer}")\n' if layer else '')
        + (f'  (condition "{cond}")\n' if cond else '')
        + f'  (constraint {kind} ({field} {mm}mm)))\n'
        for name, cond, kind, field, mm, layer in rules)


def stage(d, stem, rules, pro=None):
    pcb = os.path.join(d, stem + '.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), pcb)
    with open(os.path.join(d, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(pro or project(), fh, indent=2)
    if rules is not None:
        with open(os.path.join(d, stem + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(rules)
    return evidence(pcb, 'staged board')


def put_rules(board, rules):
    """Replace (or remove, rules=None) a board's sibling .kicad_dru."""
    path = os.path.splitext(board)[0] + '.kicad_dru'
    if os.path.isfile(path):
        os.remove(path)
    if rules is not None:
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(rules)


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


def drc(path, clearance=0.15, sizes=False):
    """check_drc at margin 0, past the writer's 1 nm coordinate rounding
    (kicad_writer.EDGE_ROUNDING_MM): a diagonal coupled exactly at the rule
    reads up to sqrt(2) nm inside it in the file."""
    from check_drc import run_drc
    from kicad_writer import EDGE_ROUNDING_MM
    return [v for v in quiet(run_drc, path, clearance=clearance, quiet=True,
                             print_summary=False, check_sizes=sizes,
                             clearance_margin=0.0)
            if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM]


def kicad_drc(path):
    """kicad-cli's report for ``path``, or None without kicad-cli."""
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        return None
    rep = path + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                    '-o', rep, path], capture_output=True, text=True, timeout=600)
    with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
        return json.load(fh)


def kicad_nets(v):
    """The net names a kicad-cli violation's items describe."""
    out = set()
    for it in v.get('items', []):
        desc = it.get('description', '')
        if '[' in desc and ']' in desc:
            out.add(desc[desc.rfind('[') + 1:desc.rfind(']')])
    return out


# ------------------------------------------------------------------ A-C
def t_support():
    print('A. which forms parse')
    from design_rules import parse_dru
    from kicad_dru import parse_dru_track_clearances
    text = dru(('apart', APART, 'clearance', 'min', 0.3, None),
               ('apart in', APART, 'clearance', 'min', 0.3, 'inner'),
               ('indp', "A.inDiffPair('*')", 'track_width', 'opt', 0.15, 'F.Cu'),
               ('indp cls', "A.NetClass == 'HS' && A.inDiffPair('DP*')",
                'diff_pair_gap', 'opt', 0.12, None),
               ('or', "!AB.isCoupledDiffPair() || A.NetClass == 'HS'",
                'clearance', 'min', 0.3, None),
               ('ab', "AB.inDiffPair('*')", 'track_width', 'opt', 0.15, None),
               ('empty', "A.inDiffPair('')", 'track_width', 'opt', 0.15, None))
    rules = {r.name: r for r in parse_dru(text)[0]}
    check('the keep-apart rule: supported, keep_apart, not a coupled-pair rule',
          rules['apart'].unsupported is None and rules['apart'].keep_apart
          and not rules['apart'].coupled)
    check('inDiffPair(): supported, alone and with a class term',
          rules['indp'].unsupported is None and rules['indp cls'].unsupported is None)
    for n in ('or', 'ab', 'empty'):
        check(f'{n}: still unsupported, with the reason said',
              bool(rules[n].unsupported), rules[n].unsupported or '')
    tr = {r.name: r for r in parse_dru_track_clearances(text)[0]}
    check('the track channel reads the keep-apart rule, partner exempt',
          'apart' in tr and tr['apart'].except_coupled and tr['apart'].cls == 'HS'
          and not tr['apart'].other_only and tr['apart'].layer_clause is None,
          f'{tr.get("apart")}')
    check('...and its (layer inner) clause',
          'apart in' in tr and tr['apart in'].layer_clause == 'inner', f'{tr.get("apart in")}')
    check('...and nothing else in the file', set(tr) == {'apart', 'apart in'}, f'{sorted(tr)}')


def t_semantics():
    print('B. semantics')
    from design_rules import DesignRules, Rule, parse_condition
    from kicad_dru import (TrackRule, nets_coupled, track_pair_clearance,
                           pair_gap_rule_floor)
    names = {1: 'DATA_P_1', 2: 'DATA_N_1', 3: 'USB_DP', 4: 'USB_DM', 5: 'CLK_P',
             6: 'D+', 7: 'D-', 8: 'Lone_P', 9: 'SPI', 10: 'SPIN', 11: 'SPIP'}
    dr = DesignRules(net_names=names)

    def indp(pattern, nid):
        dr.rules = [Rule(name='r', order=0, constraints={'track_width': {'opt': 0.1}},
                         condition=parse_condition(f"A.inDiffPair('{pattern}')"))]
        dr._cache.clear()
        return dr.resolve('track_width', dr.item_for_net(nid), None).opt == 0.1
    cases = [('*', 1, True), ('DATA', 1, True), ('DATA_', 1, True), ('DAT?', 1, True),
             ('data', 1, False), ('X*', 1, False), ('*', 8, False), ('*', 5, False),
             ('D', 6, True), ('SPI', 10, True), ('*', 9, False), ('[D]*', 1, False)]
    bad = [(p, names[n], want) for p, n, want in cases if indp(p, n) != want]
    check("inDiffPair() is KiCad's inDiffPairFunc (complement must exist, wx wildcards, "
          "the base before a trailing '_')", not bad, f'{bad}')
    check('nets_coupled: KiCad pairs, either order',
          nets_coupled('DATA_P_1', 'DATA_N_1') and nets_coupled('D-', 'D+')
          and nets_coupled('SPIN', 'SPIP'))
    check('...and not DP/DM, nor a net with itself or none',
          not nets_coupled('USB_DP', 'USB_DM') and not nets_coupled('CLK_P', 'CLK_P')
          and not nets_coupled('', 'CLK_N'))
    r = TrackRule('apart', 'HS', False, 0.3, True, None)
    hs, no = frozenset({'HS'}), frozenset()
    check('track_pair_clearance: a member against anything else binds',
          track_pair_clearance([r], hs, no, 0.15)[0] == 0.3
          and track_pair_clearance([r], no, hs, 0.15)[0] == 0.3
          and track_pair_clearance([r], hs, hs, 0.15)[0] == 0.3)
    check('...its own coupled partner is exempt',
          track_pair_clearance([r], hs, hs, 0.15, coupled=True) == (0.15, None))
    plain = TrackRule('plain', 'HS', False, 0.3)
    check('...and a rule without the term still binds the partner (#735)',
          track_pair_clearance([plain], hs, hs, 0.15, coupled=True)[0] == 0.3)
    inner = TrackRule('apart in', 'HS', False, 0.3, True, 'inner')
    check('a layer clause binds on its layers only; an unnamed layer binds',
          track_pair_clearance([inner], hs, no, 0.15, layer='In2.Cu')[0] == 0.3
          and track_pair_clearance([inner], hs, no, 0.15, layer='F.Cu')[0] == 0.15
          and track_pair_clearance([inner], hs, no, 0.15)[0] == 0.3)
    check('#1145: the keep-apart rule never raises a coupled pair\'s gap',
          pair_gap_rule_floor({}, ['F.Cu'], [r], hs, hs, coupled=True) == (0.0, None)
          and pair_gap_rule_floor({}, ['F.Cu'], [r], hs, hs)[0] == 0.3)


def t_install():
    print('C. install_track_clearances and pair_clearance')
    from kicad_parser import parse_kicad_pcb
    from kicad_dru import install_track_clearances
    from routing_config import GridRouteConfig
    with tempfile.TemporaryDirectory(prefix='t1295_') as d:
        pcb_path = stage(d, 'i', dru(('apart', APART, 'clearance', 'min', 0.3, None)))
        pcb = quiet(parse_kicad_pcb, pcb_path)
        ids = {n.name: i for i, n in pcb.nets.items()}
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.15)
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            install_track_clearances(cfg, None, pcb_path, pcb)
        p, n, other = ids['DPA_P'], ids['DPA_N'], ids['DPB_N']
        check('the rule is announced, partner exempt',
              '(its own coupled partner exempt)' in log.getvalue(), log.getvalue()[-300:])
        check('the per-net map prices every member (the router over-approximates)',
              all(cfg.track_clearances.get(i) == 0.3 for i in ids.values()),
              f'{cfg.track_clearances}')
        check('the partner map holds both pairs, at no track rule',
              cfg.track_partner_clearances == {frozenset((p, n)): 0.0,
                                               frozenset((ids['DPB_P'], other)): 0.0},
              f'{cfg.track_partner_clearances}')
        check('pair_clearance: P<->N stays at the base',
              cfg.pair_clearance(p, n, 'F.Cu', kind='track') == 0.15
              and cfg.pair_clearance(n, p, 'F.Cu', kind='track') == 0.15)
        check('...P against the other pair takes the rule',
              cfg.pair_clearance(p, other, 'F.Cu', kind='track') == 0.3)
        install_track_clearances(cfg, {p: 0.3}, pcb_path, pcb)
        check('an explicit map (a nested sub-run) carries no partner map',
              cfg.track_partner_clearances == {})
        sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
        from placement.legality import PadClearanceModel
        m = quiet(PadClearanceModel.for_board, pcb, 0.15, pcb_path)
        check('placement (the fanout connector gate): P<->N exempt, the other pair not',
              m.track_pair(p, n, 0.15) == (0.15, None)
              and m.track_pair(p, other, 0.15)[0] == 0.3,
              f'{m.track_pair(p, n, 0.15)} {m.track_pair(p, other, 0.15)}')


# ------------------------------------------------------------------ D-F
def _route_diff(d, stem, rules, pro=None, **kw):
    import route_diff
    src = stage(d, stem, rules, pro)
    out = os.path.join(d, stem + '_o.kicad_pcb')
    with open(os.path.join(d, stem + '_o.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(pro or project(), fh)
    if rules is not None:
        shutil.copy(os.path.join(d, stem + '.kicad_dru'),
                    os.path.join(d, stem + '_o.kicad_dru'))
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        route_diff.batch_route_diff_pairs(src, out, PAIRS, enable_layer_switch=True, **kw)
    return log.getvalue(), evidence(out, 'routed board')


def t_keep_apart_route_diff():
    print('D. route_diff on cap_chain under a 0.3 keep-apart rule')
    from check_connected import run_connectivity_check
    with tempfile.TemporaryDirectory(prefix='t1295_') as d:
        log, out = _route_diff(d, 'ka', dru(('apart', APART, 'clearance', 'min', 0.3, None)),
                               clearance=0.15, diff_pair_gap=0.15,
                               diff_pair_gap_from_class=True)
        check('the rule is read (the path under test)',
              '(its own coupled partner exempt)' in log)
        check('no #1145 raise for a partner-exempt rule', '#1145' not in log
              and 'raising gap' not in log,
              next((ln.strip() for ln in log.splitlines()
                    if '#1145' in ln or 'raising gap' in ln), ''))
        g = coupled_gaps(out)
        check('both pairs couple at 0.15, not the rule\'s 0.3', all(
            v and all(abs(x - 0.15) < 0.002 for x in v) for v in g.values()), f'{g}')
        conn = quiet(run_connectivity_check, out, net_patterns=['DP*'], quiet=True)
        check('every pair net connected', not list(conn), f'{list(conn)[:3]}')
        v = drc(out)
        check('check_drc grades it clean', not v, f'{[x.get("type") for x in v][:6]}')
        k = kicad_drc(out)
        if k is None:
            print('  SKIP: no kicad-cli found; KiCad\'s own grade is unchecked here')
        else:
            bad = [x['type'] for x in k['violations'] if x['type'] == 'clearance']
            check('KiCad: no clearance violation (it exempts the partner too)', not bad,
                  f'{bad[:6]}')
        put_rules(out, dru(('plain', PLAIN, 'clearance', 'min', 0.3, None)))
        v = drc(out)
        check('check_drc flags the same copper under the rule without the term',
              any({x.get('net1'), x.get('net2')} in ({'DPA_P', 'DPA_N'}, {'DPB_P', 'DPB_N'})
                  for x in v), f'{len(v)} violations')
        k = kicad_drc(out) if k is not None else None
        if k is not None:
            pn = [x for x in k['violations'] if x['type'] == 'clearance'
                  and kicad_nets(x) in ({'DPA_P', 'DPA_N'}, {'DPB_P', 'DPB_N'})]
            check('...and so does KiCad', bool(pn),
                  f'{len(k["violations"])} violations')


SPLITFLAP = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
LED_RULE = 0.45
LED_APART = ("A.NetClass=='CRIT' && !AB.isCoupledDiffPair() && "
             "A.Type=='track' && B.Type=='track'")
LED_PRO = project(clearance=0.2, gap=0.2, width=0.2, cls='CRIT', pattern='/LED_*')
# test_735_dru_track_clearance_e2e's nets: foreign pads 1.3-1.7 mm from the
# LED pads, so a plain 0.2 route comes under 0.45.
LED_NETS = ["/LED_*", "/LOOPBACK_*", "/MOTOR_C_*", "Net-(D?-Pad2)"]


def _route_led(d, stem, rules):
    from copy_board import copy_board
    src = os.path.join(d, stem + '.kicad_pcb')
    quiet(copy_board, SPLITFLAP, src)
    with open(os.path.splitext(src)[0] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(LED_PRO, fh)
    put_rules(src, rules)
    out = os.path.join(d, stem + '_o.kicad_pcb')
    js = out + '.json'
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('route.py'), src, out,
                        '--nets', *LED_NETS, '--layers', 'F.Cu', 'B.Cu',
                        '--clearance', '0.2', '--track-width', '0.2', '--json-out', js],
                       cwd=ROOT, capture_output=True, text=True, timeout=1800)
    summary = json.load(open(js)) if os.path.isfile(js) else {}
    return out, r, summary


def _on_led(v):
    """A check_drc finding on /LED_* copper (the board ships unrelated ones)."""
    return any(str(v.get(k, '')).startswith('/LED_') for k in ('net1', 'net2'))


def min_led_gap(board):
    """Min edge-to-edge gap from a /LED_* segment to a foreign one, same layer."""
    from kicad_parser import parse_kicad_pcb
    from check_drc import _seg_seg_dist_coords
    pcb = quiet(parse_kicad_pcb, board)
    crit = {i for i, n in pcb.nets.items() if n.name and n.name.startswith('/LED_')}
    best = float('inf')
    for a in pcb.segments:
        if a.net_id not in crit:
            continue
        for b in pcb.segments:
            if b.net_id in crit or b.net_id == 0 or b.layer != a.layer:
                continue
            gap = _seg_seg_dist_coords(a.start_x, a.start_y, a.end_x, a.end_y,
                                       b.start_x, b.start_y, b.end_x, b.end_y) \
                - a.width / 2 - b.width / 2
            best = min(best, gap)
    return best


def t_keep_apart_route():
    print('E. route.py on splitflap_driver under a 0.45 keep-apart rule on /LED_*')
    rule = dru(('crit apart', LED_APART, 'clearance', 'min', LED_RULE, None))
    with tempfile.TemporaryDirectory(prefix='t1295_') as d:
        out, r, s = _route_led(d, 'ka', rule)
        check('route.py exits 0', r.returncode == 0, (r.stdout + r.stderr)[-600:])
        if r.returncode != 0:
            return
        check('the engine announces the rule, partner exempt',
              "class 'CRIT'" in r.stdout and '(its own coupled partner exempt)' in r.stdout)
        check('every net still routes', s.get('failed') == 0, f"failed={s.get('failed')}")
        g = min_led_gap(evidence(out, 'routed board'))
        check(f'/LED_* copper keeps >= {LED_RULE} from foreign tracks',
              g >= LED_RULE - 0.01, f'min gap {g:.3f}')
        v = [x for x in drc(out, clearance=0.2) if _on_led(x)]
        check('check_drc grades /LED_* copper clean', not v,
              f'{[x.get("type") for x in v][:6]}')
        k = kicad_drc(out)
        if k is None:
            print('  SKIP: no kicad-cli found; KiCad\'s own grade is unchecked here')
        else:
            bad = [x for x in k['violations'] if x['type'] == 'clearance'
                   and any(nm.startswith('/LED_') for nm in kicad_nets(x))]
            check('KiCad: no clearance violation on /LED_*', not bad, f'{len(bad)}')
        out0, r0, _s0 = _route_led(d, 'plain', None)
        check('the control route.py exits 0', r0.returncode == 0)
        g0 = min_led_gap(evidence(out0, 'control board'))
        check(f'the control comes closer than {LED_RULE} (the rule moved copper)',
              g0 < LED_RULE - 0.01, f'min gap {g0:.3f}')
        v0 = [x for x in drc(out0, clearance=0.2) if _on_led(x)]
        check('the control grades clean without the rule', not v0, f'{len(v0)}')
        put_rules(out0, rule)
        v0 = [x for x in drc(out0, clearance=0.2) if _on_led(x)]
        check('check_drc flags the control once the rule is added (not blind)',
              bool(v0), f'{len(v0)} violations')
        if k is not None:
            k0 = kicad_drc(out0)
            hit = [x for x in k0['violations'] if x['type'] == 'clearance'
                   and any(nm.startswith('/LED_') for nm in kicad_nets(x))]
            check('...and so does KiCad', bool(hit), f'{len(k0["violations"])} violations')


WPRO = project(clearance=0.25, gap=0.25, width=0.25)


def t_in_diff_pair():
    print('F. inDiffPair() rules')
    with tempfile.TemporaryDirectory(prefix='t1295_') as d:
        log, _out = _route_diff(
            d, 'w', dru(('w', "A.inDiffPair('*')", 'track_width', 'opt', 0.15, 'F.Cu')),
            pro=WPRO, diff_pair_width_from_class=True, layers=['F.Cu', 'B.Cu'])
        check("an inDiffPair('*') layer rule sets the pair's F.Cu width",
              'Design-rule layer widths: F.Cu=0.15mm, B.Cu=0.25mm' in log,
              next((ln.strip() for ln in log.splitlines() if 'layer widths' in ln), ''))
        log, _out = _route_diff(
            d, 'x', dru(('x', "A.inDiffPair('XYZ*')", 'track_width', 'opt', 0.15, 'F.Cu')),
            pro=WPRO, diff_pair_width_from_class=True, layers=['F.Cu', 'B.Cu'])
        check("an inDiffPair('XYZ*') rule does not", 'Design-rule layer widths' not in log
              and 'NOT applied' not in log)
        log, out = _route_diff(
            d, 'a', dru(('a', "A.inDiffPair('DPA')", 'track_width', 'opt', 0.15, 'F.Cu')),
            pro=WPRO, diff_pair_width_from_class=True, layers=['F.Cu', 'B.Cu'])
        check("an inDiffPair('DPA') rule takes DPA_P/DPA_N alone (the base before "
              "the trailing '_'), so the pairs disagree on F.Cu",
              'NOT applied' in log and 'F.Cu (0.15/0.25mm)' in log,
              next((ln.strip() for ln in log.splitlines() if 'NOT applied' in ln), log[-300:]))
        put_rules(out, dru(('m', "A.inDiffPair('DPA')", 'track_width', 'min', 0.3, None)))
        v = [x for x in drc(out, clearance=0.25, sizes=True) if x.get('type') == 'track-width']
        nets = {x.get('net1') for x in v}
        check("check_drc grades an inDiffPair('DPA') minimum on DPA alone",
              nets == {'DPA_P', 'DPA_N'}, f'{sorted(nets)}')
        k = kicad_drc(out)
        if k is None:
            print('  SKIP: no kicad-cli found; KiCad\'s own grade is unchecked here')
        else:
            kn = set()
            for x in k['violations']:
                if x['type'] == 'track_width':
                    kn |= kicad_nets(x)
            check('...and so does KiCad', kn == {'DPA_P', 'DPA_N'}, f'{sorted(kn)}')


if __name__ == '__main__':
    t_support()
    t_semantics()
    t_install()
    t_keep_apart_route_diff()
    t_keep_apart_route()
    t_in_diff_pair()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')
