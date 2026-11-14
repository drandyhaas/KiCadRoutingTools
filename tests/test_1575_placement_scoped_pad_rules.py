#!/usr/bin/env python3
"""Placement prices a pad pair at the rule check_drc grades it at (#1575).

`PadClearanceModel` (placement/legality.py) priced two pads at their net-class
clearance and never read a `.kicad_dru` clearance rule scoped by net class or
item type (the #1491 scoped channel), nor the net class's implicit "(diff
pair)" rule that grades a pair's P against its N at the class diff_pair_gap
(#1542). check_drc's `_pad_pair_cl` grades the same pair with both. So
place_pose refused a 0.38 mm gap between two 0402s of one pair (class 0.5)
that check_drc and KiCad accept, and check_assembly printed "C1<->C2 requires
0.5mm (netclass)" beside a note that the rule was "handled by the scoped
channel".

The model now builds check_drc's own `PairRuleChannels` and prices a pair
through its `pair`, the resolution `_pad_pair_cl` grades with, so the two
cannot drift.

The fixture is cap_chain: C1 (DPA_P, DPB_P) and C2 (DPA_N, DPB_N), two 0402s
of two pairs. Default 0.15; class HS (DP*) at clearance 0.5 and diff_pair_gap
GAP. Arms:

  none    no rule, GAP 0.5 (no implicit rule)         -- the control
  gap     A.NetClass == 'HS' && B.NetClass == 'HS' at 0.127 mm
  pad     A.NetClass == 'HS' && B.Type == 'Pad'     at 0.2 mm
  dpgap   no rule, GAP 0.15: KiCad's "netclass 'HS' (diff pair)" rule
  coupled A.NetClass == 'HS' && AB.isCoupledDiffPair() at 0.12 mm (#1293)

C2 set at (18, 14) puts C1's and C2's pads 0.38 mm apart (pitch 1.0).

Checks:
  A. the resolver: each arm's C1.1/C2.1 value and the source that set it; a
     P against the OTHER pair's N stays at the class; inert on the stock board
     and on the control
  B. every consumer gives the one answer at C2 (18, 14): grade_pad_legality
     (place_pose's verdict, check_assembly's echo), the seat gate's
     LegalityContext.pair_shortfall (quench, place_simple), the fanout pass's
     pad / via / track resolvers -- and check_drc agrees, both ways
  C. place_pose: the control still refuses, for its stated reason; every rule
     arm accepts
  D. check_assembly: no "requires 0.5mm (netclass)" beside the rule's note
  E. a pair still short of the rule is charged AT the rule and the source
     names it: pad arm at pitch 0.75 (0.13 mm gap) requires 0.2 "rule
     'pad_escape'", and check_drc flags it there too
  F. KiCad grades the 1.0-pitch boards as the model does (skipped, and said,
     without kicad-cli)

    python3 tests/test_1575_placement_scoped_pad_rules.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence, tool    # noqa: E402

FAILS = []
BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
GAP_RULE = ("(rule hs_gap (condition \"A.NetClass == 'HS' && B.NetClass == 'HS'\") "
            "(constraint clearance (min 0.127mm)))")
PAD_RULE = ("(rule pad_escape (condition \"A.NetClass == 'HS' && B.Type == 'Pad'\") "
            "(constraint clearance (min 0.2mm)))")
PN_RULE = ("(rule pn (condition \"A.NetClass == 'HS' && AB.isCoupledDiffPair()\") "
           "(constraint clearance (min 0.12mm)))")
ARMS = {'none': (0.5, None), 'gap': (0.5, GAP_RULE), 'pad': (0.5, PAD_RULE),
        'dpgap': (0.15, None), 'coupled': (0.5, PN_RULE)}
# C1.1 (DPA_P) against C2.1 (DPA_N): (value, source) at base 0.15 and at 0.1
EXPECT = {
    'none': ((0.5, 'netclass'), (0.5, 'netclass')),
    'gap': ((0.127, ''), (0.127, "rule 'hs_gap'")),
    'pad': ((0.2, "rule 'pad_escape'"), (0.2, "rule 'pad_escape'")),
    'dpgap': ((0.15, ''), (0.15, "netclass 'HS' (diff pair)")),
    'coupled': ((0.12, ''), (0.12, "rule 'pn'")),
}


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


def _cls(n, c, g, prio):
    return {'name': n, 'clearance': c, 'track_width': 0.15, 'via_diameter': 0.6,
            'via_drill': 0.3, 'diff_pair_gap': g, 'diff_pair_width': 0.15,
            'diff_pair_via_gap': 0.25, 'microvia_diameter': 0.3, 'microvia_drill': 0.1,
            'priority': prio, 'bus_width': 12, 'wire_width': 6, 'line_style': 0,
            'pcb_color': 'rgba(0, 0, 0, 0.000)', 'schematic_color': 'rgba(0, 0, 0, 0.000)'}


def stage(d, arm, c2_y=None):
    """cap_chain beside its project and rules, C2 moved to (18, c2_y)."""
    from copy_board import copy_board
    gap, rule = ARMS[arm]
    sub = os.path.join(d, arm if c2_y is None else f'{arm}_{c2_y:g}')
    os.makedirs(sub, exist_ok=True)
    dst = os.path.join(sub, 'b.kicad_pcb')
    quiet(copy_board, BOARD, dst)
    stem = os.path.splitext(dst)[0]
    with open(stem + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.0,
                                                           'min_track_width': 0.0}}},
                   'net_settings': {'meta': {'version': 4},
                                    'classes': [_cls('Default', 0.15, 0.15, 2147483647),
                                                _cls('HS', 0.5, gap, 0)],
                                    'netclass_patterns': [{'netclass': 'HS', 'pattern': 'DP*'}]},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3}}, fh, indent=2)
    with open(stem + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + (rule + '\n' if rule else ''))
    if c2_y is not None:
        with open(dst, encoding='utf-8', newline='') as fh:
            text = fh.read()
        # C2's own anchor; the stock board has it exactly once
        assert text.count('(at 18 17)') == 1, 'cap_chain changed: C2 is not at (18, 17)'
        with open(dst, 'w', encoding='utf-8', newline='') as fh:
            fh.write(text.replace('(at 18 17)', f'(at 18 {c2_y:g})'))
    return dst


def pads(pcb, ref):
    return {p.pad_number: p for p in pcb.footprints[ref].pads}


def t_resolver(d):
    print('A. the resolver')
    from kicad_parser import parse_kicad_pcb
    from placement.legality import PadClearanceModel
    stock = PadClearanceModel.for_board(parse_kicad_pcb(BOARD), 0.15, BOARD)
    # MUTATION: `or self.channels is not None` -> `or True` in __init__'s active
    check('inert on the stock board (no project, no rules)',
          not stock.active and getattr(stock, 'channels', None) is None)
    for arm, (at15, at10) in EXPECT.items():
        path = stage(d, arm)
        pcb = parse_kicad_pcb(path)
        c1, c2 = pads(pcb, 'C1'), pads(pcb, 'C2')
        for base, want in ((0.15, at15), (0.1, at10)):
            m = PadClearanceModel.for_board(pcb, base, path)
            got = m.pair_with_source(m.pad_floor(c1['1']), m.pad_floor(c2['1']))
            # MUTATION: drop the `ra is not None and rb is not None` branch in
            # pair_with_source -> every rule arm reads (0.5, 'netclass')
            check(f'{arm} @{base:g}: C1.1/C2.1 = {want}',
                  abs(got[0] - want[0]) < 1e-9 and got[1] == want[1], f'{got}')
        if arm == 'none':
            # MUTATION: PairRuleChannels.live -> True
            check('none: the control is no channel at all',
                  getattr(m, 'channels', None) is None)
        # a P against the OTHER pair's N is no P/N pair: the class (or the rule)
        m = PadClearanceModel.for_board(pcb, 0.15, path)
        other = m.pair(m.pad_floor(c1['1']), m.pad_floor(c2['2']))
        want_other = {'gap': 0.127, 'pad': 0.2}.get(arm, 0.5)
        # MUTATION: coupled_rule's `net_a not in self.coupled_nets` test dropped
        # AND `_is_coupled` read as True -> dpgap prices DPA_P/DPB_N at 0.15
        check(f'{arm}: C1.1/C2.2 (DPA_P vs DPB_N) = {want_other:g}',
              abs(other - want_other) < 1e-9, f'{other}')
    # the scoped rule's note is "handled" only because this model prices it
    m = PadClearanceModel.for_board(parse_kicad_pcb(stage(d, 'gap')), 0.15,
                                    stage(d, 'gap'))
    check('gap: the rule is named handled, and the model holds it',
          any("'hs_gap'" in n and 'handled by the scoped channel' in n for n in m.notes)
          and getattr(m, 'channels', None) is not None
          and 'hs_gap' in m.channels.scoped.names, f'{m.notes}')
    m = PadClearanceModel.for_board(parse_kicad_pcb(stage(d, 'coupled')), 0.15,
                                    stage(d, 'coupled'))
    # MUTATION: kicad_dru._parse_dru_full's coupled branch dropped -> "skipped,
    # KiCad will still enforce it" for a rule the model prices (#1293: the
    # parser names the channel, and placement passes the note through)
    check('coupled: the P/N rule is named handled, not skipped',
          any("'pn'" in n and 'coupled-pair channel' in n for n in m.notes)
          and not any("'pn'" in n and 'skipped' in n for n in m.notes), f'{m.notes}')


def t_consumers(d):
    print('B. every consumer, one answer, at pitch 1.0 (0.38 mm pad gap)')
    from kicad_parser import parse_kicad_pcb
    import placement.legality as L
    from placement.fanout_clearance import _Repair
    for arm in ('none', 'gap', 'pad', 'dpgap'):
        path = stage(d, arm, 14)
        pcb = parse_kicad_pcb(path)
        clean = arm != 'none'
        g = quiet(L.grade_pad_legality, pcb, 0.15, pcb_file=path)
        check(f'{arm}: grade_pad_legality pad_conflicts {0 if clean else 1}',
              g['pad_conflicts'] == (0 if clean else 1), f"{g['pad_conflicts']} {g['required']}")
        # the seat gate exactly as quench builds it (QuenchState: model, parts,
        # LegalityContext) -- place_simple's candidate_valid reads this
        m = L.PadClearanceModel.for_board(pcb, 0.15, path)
        model = m if m.active else None
        pp = quiet(L.build_part_pads, pcb.footprints, 0.15, model)
        poses = {r: (fp.x, fp.y, fp.rotation or 0.0) for r, fp in pcb.footprints.items()}
        ctx = L.LegalityContext(pp, None, 0.15, pose_of=poses.get, seed_of=poses.get,
                                model=model)
        sf = ctx.pair_shortfall('C1', 'C2')
        check(f'{arm}: the seat gate charges {"nothing" if clean else "the pair"}',
              (sf.pad <= 1e-9) if clean else (sf.pad > 0.2), f'{sf.pad:.4f}')
        st = quiet(_Repair, pcb, path, 0.15, 0.1, 0.55, 1.0, 2.0, 0.3, 'C', set())
        c1, c2 = pads(pcb, 'C1'), pads(pcb, 'C2')
        fa = st.pad_floor(c1['1'])
        want = {'none': 0.5, 'gap': 0.127, 'pad': 0.2, 'dpgap': 0.15}[arm]
        got = (st.required(fa, st.pad_floor(c2['1'])),
               st.via_required(fa, c2['1'].net_id),
               st.required(fa, st.seg_floor(c2['1'].net_id, 'F.Cu')))
        # MUTATION: _via_floor_for / _seg_floor_for back to a bare
        # PadFloor(ncl, 0.0, layers) -> the via and track read 0.5
        check(f'{arm}: the fanout pass prices pad / via / track of DPA_N at {want:g}',
              all(abs(v - want) < 1e-9 for v in got), f'{got}')
        drc = [sys.executable, '-X', 'utf8', tool('check_drc.py'), path]
        if clean:
            run_check(drc, accept=True)
            check(f'{arm}: check_drc grades it clean too', True)
        else:
            run_check(drc, refuse='Required clearance: 0.5000mm', code=1)
            check(f'{arm}: check_drc flags it at 0.5 too', True)


def t_place_pose(d):
    print('C. place_pose set C2 18 14')
    for arm in ('none', 'gap', 'pad', 'dpgap'):
        src = stage(d, arm)
        out = os.path.join(os.path.dirname(src), 'o.kicad_pcb')
        argv = [sys.executable, '-X', 'utf8', tool('place_pose.py'), src, out,
                'set', 'C2', '18', '14']
        if arm == 'none':
            # the control: refused for its reason, not for any failure
            run_check(argv, refuse='pad_conflicts 0 -> 1', code=4)
            check('none: refused, pad_conflicts 0 -> 1', True)
        else:
            run_check(argv, accept=True)
            evidence(out, f'{arm}: place_pose output')
            check(f'{arm}: accepted and written', True)


def t_check_assembly(d):
    print('D. check_assembly at pitch 1.0')
    for arm in ('none', 'gap', 'pad'):
        path = stage(d, arm, 14)
        r = run_check([sys.executable, '-X', 'utf8', tool('check_assembly.py'), path],
                      accept=True)
        says = 'requires 0.5mm (netclass)' in r.stdout
        if arm == 'none':
            check('none: C1<->C2 requires 0.5mm (netclass)', says)
        else:
            check(f'{arm}: no "requires 0.5mm (netclass)" beside the rule note',
                  not says and 'handled by the scoped channel' in r.stdout,
                  [l for l in r.stdout.splitlines() if 'pad clearance' in l or 'requires' in l])


def t_named(d):
    print('E. pad arm at pitch 0.75 (0.13 mm gap): charged AT the rule, named')
    from kicad_parser import parse_kicad_pcb
    import placement.legality as L
    path = stage(d, 'pad', 13.75)
    g = quiet(L.grade_pad_legality, parse_kicad_pcb(path), 0.15, pcb_file=path)
    # MUTATION: `_channel_pair` returns (eff, src) of the class step -> 0.5 'netclass'
    check("required: C1<->C2 at 0.2, rule 'pad_escape'",
          g['required'] == [['C1', 'C2', 0.2, "rule 'pad_escape'"]], f"{g['required']}")
    check('the clause names the rule',
          "rule 'pad_escape'" in L.format_required_clause(g), L.format_required_clause(g))
    check('two pad pairs 0.07 short each',
          abs(g['pad_shortfall'] - 0.14) < 2e-3, f"{g['pad_shortfall']}")
    run_check([sys.executable, '-X', 'utf8', tool('check_drc.py'), path],
              refuse='Required clearance: 0.2000mm', code=1)
    check('check_drc flags the same pair at the rule', True)


def t_kicad(d):
    print('F. KiCad grades the pitch-1.0 boards')
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        print("  SKIP: no kicad-cli found; KiCad's own grade is unchecked here")
        return
    for arm in ('none', 'gap', 'pad', 'dpgap'):
        path = stage(d, arm, 14)
        rep = path + '.drc.json'
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '-o', rep, path], capture_output=True, text=True, timeout=600)
        with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
            v = [x for x in json.load(fh)['violations'] if x['type'] == 'clearance']
        n = sum('0.3800' in x['description'] for x in v)
        check(f'{arm}: KiCad {"flags" if arm == "none" else "accepts"} the 0.38 mm gap',
              (n > 0) if arm == 'none' else (n == 0), f'{n} error(s)')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='t1575_') as d:
        t_resolver(d)
        t_consumers(d)
        t_place_pose(d)
        t_check_assembly(d)
        t_named(d)
        t_kicad(d)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')
