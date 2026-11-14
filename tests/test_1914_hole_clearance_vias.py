#!/usr/bin/env python3
"""#1914: downsized vias break the 0.25 mm copper-to-hole clearance, two halves.

CHECKER (was: vias never entered the `holes` list, so --hole-clearance graded
only NPTH holes, uncovered plated holes and flush pegs): a plated via's drill
is now graded against other-net tracks, vias and pads, at
max(clearance, hole_clearance) -- with the pair-clearance suppression KiCad
itself applies (a hole row does not stand beside a failing copper-clearance
row, except pad-vs-via-hole, where KiCad reports both).

Synthetic (always runs), all at --clearance 0.1 --hole-clearance 0.25:
  A. a 0.45/0.2 via with an other-net pad 0.225 off its hole -> PAD-HOLE,
     overlap 0.025, required 0.25 'declared hole clearance' (the issue's pair);
  B. control: a 0.6/0.3 via with the pad at exactly 0.25 off its hole -> clean;
  C. a 0.45/0.2 via and a 0.6/0.3 via 0.63 apart (copper gap 0.105, hole
     gap 0.23) -> VIA-HOLE (the pre-fix board's third row, UART_CTS/VREF);
  D. a track at copper clearance off a 0.45/0.2 via, 0.225 off its hole ->
     TRACK-HOLE, overlap 0.025;
  E. the same track 0.05 off the via's copper -> the clearance row carries
     it, TRACK-HOLE is 0 (suppression);
  F. two vias 0.45 apart (copper overlapping) -> the clearance row carries
     it, VIA-HOLE is 0 (suppression);
  G. a pad 0.05 off a 0.45/0.2 via's copper and 0.175 off its hole ->
     PAD-HOLE still fires (KiCad reports both for pad-vs-via-hole).

ROUTER (was: the descent ladder shipped a 0.45/0.2 via whose 0.125 ring left
other-net copper 0.225 off its hole, and said nothing): the ladder PREFERS the
ring -- escalation_rungs puts every rung raised to drill + 2*(hole_clearance -
clearance) first and keeps the tier's own rungs as the last resort -- and what
ships below the ring is named in the design_rules ledger. Unit checks:
  * a fresh import with nothing installed walks the ladder (the pair's global
    is defined at module level; it once raised NameError);
  * no pair installed -> the tier's ladder, unchanged and untagged;
  * pair (0.25, 0.1) -> standard is 0.5/0.2 first, then 0.45/0.2;
  * tier auto -> the ring-holding rungs first, then the tier's rungs, each
    tagged with the tier rung it came from, so a nominal via is never read
    as a fab-tier escalation (rung_tier, ladder_escalates);
  * declared hole clearance 0 -> ring 0.0, the tier's ladder (KiCad "no check");
  * policy off -> no rungs at all, even with the pair installed;
  * a new run (set_escalation_policy) clears the pair; the GUI's config key
    installs it, and a config without it installs none;
  * set_policy_from_args and the GUI's PCBData resolve the same value
    (board_hole_clearance): the rules value, the fab_floor_origin when larger,
    KiCad's factory 0.25 when nothing is declared, 0 when 0 is declared;
  * the swap shrink loop ships (0.5, 0.2) with the pair installed where that
    fits, (0.5, 0.3) without it, and falls back to (0.45, 0.3) where no
    ring-holding via fits (completion first);
  * the census rows: only a via under its request AND under the ring is
    named, once per site, and the end-of-run line says so.

    python3 tests/test_1914_hole_clearance_vias.py
"""
import json
import os
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check, evidence  # noqa: E402

CHECK_DRC = os.path.join(ROOT, 'py_router', 'check_drc.py')

fails = []


def ok(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def _edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'(layer "Edge.Cuts") (width 0.1))' for i in range(4))


def _board(vias=(), pads=(), segs=()):
    """vias: (x, y, size, drill, net); pads: (ref, num, dx, dy, sx, sy, net);
    segs: (x1, y1, x2, y2, width, net). Footprints sit at (20, 20)."""
    via_txt = "\n".join(
        f' (via (at {x} {y}) (size {s}) (drill {d}) (layers "F.Cu" "B.Cu") '
        f'(net {n}) (tstamp 00000000-0000-0000-0000-000000000001))'
        for x, y, s, d, n in vias)
    pad_txt = ""
    for ref, num, dx, dy, sx, sy, n in pads:
        name = {1: 'A', 2: 'B'}.get(n, f'N{n}')
        pad_txt += (
            f'\n (footprint "test:pad" (layer "F.Cu") (at 20 20)\n'
            f'  (property "Reference" "{ref}" (at 0 -2) (layer "F.SilkS"))\n'
            f'  (pad "{num}" smd rect (at {dx} {dy}) (size {sx} {sy}) '
            f'(layers "F.Cu" "F.Paste" "F.Mask") (net {n} "{name}"))\n'
            f' )')
    seg_txt = "\n".join(
        f' (segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) '
        f'(layer "F.Cu") (net {n}) (uuid "s{n}"))'
        for x1, y1, x2, y2, w, n in segs)
    return f'''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "A")
 (net 2 "B")
{_edge(0, 0, 40, 40)}
{via_txt}{pad_txt}{seg_txt}
)'''


def _stage(tmp, name, text):
    d = os.path.join(tmp, name)
    os.makedirs(d)
    pcb = os.path.join(d, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(text)
    return pcb


def _grade(pcb, tmp, tag, violating, refuse=None):
    out = os.path.join(tmp, f'{tag}.json')
    argv = [sys.executable, '-X', 'utf8', CHECK_DRC, pcb, '--clearance', '0.1',
            '--hole-clearance', '0.25', '--json', out]
    if violating:
        r = check(argv, refuse=refuse, code=1)
    else:
        r = check(argv, accept=True)
    evidence(out, f'{tag} check_drc --json')
    with open(out, encoding='utf-8') as f:
        return json.load(f), r


def checker(tmp):
    print('1. pad-hole: the issue\'s pair, a 0.45/0.2 via vs an other-net pad')
    # Via at (20,20), 0.45/0.2. Pad 1.0 x 0.5 centred at (20, 20.575): its
    # edge sits 0.325 off the via centre, 0.225 off the 0.2 drill.
    pcb = _stage(tmp, 'pad_hole', _board(
        vias=[(20, 20, 0.45, 0.2, 1)],
        pads=[('R1', '1', 0, 0.575, 1.0, 0.5, 2)]))
    doc, _r = _grade(pcb, tmp, 'pad_hole', True, refuse='PAD-HOLE')
    ok('pad-hole fires once, 0.025 overlap at the declared 0.25',
       doc.get('by_type', {}).get('pad-hole', 0) == 1
       and doc.get('violations') == 1, doc.get('by_type'))
    items = [it for it in doc.get('items', []) if it.get('type') == 'pad-hole']
    ok('the pad-hole row names the declared hole clearance',
       items and abs(items[0].get('required_mm', 0) - 0.25) < 1e-9
       and items[0].get('required_source') == 'declared hole clearance'
       and abs(items[0].get('overlap_mm', 0) - 0.025) < 1e-9, items)
    ok('graded_at.hole_clearance is the 0.25 the flag asked for',
       abs(doc.get('graded_at', {}).get('hole_clearance', 0) - 0.25) < 1e-9,
       doc.get('graded_at'))

    print('2. control: a full-size via with the pad at exactly 0.25 is clean')
    # Via 0.6/0.3; pad edge at 0.40 off centre: 0.40 - 0.15 = 0.25 exactly,
    # and copper gap 0.40 - 0.30 = 0.10 = the clearance.
    pcb = _stage(tmp, 'control', _board(
        vias=[(20, 20, 0.6, 0.3, 1)],
        pads=[('R1', '1', 0, 0.65, 1.0, 0.5, 2)]))
    doc, _r = _grade(pcb, tmp, 'control', False)
    ok('control board grades clean', doc.get('violations') == 0, doc.get('by_type'))

    print('3. via-hole: copper passes, the hole rule fails')
    # A 0.45/0.2 at (20,20), B 0.6/0.3 at (20.63, 20): copper gap 0.105,
    # B copper to A hole 0.63 - 0.30 - 0.10 = 0.23, overlap 0.020.
    pcb = _stage(tmp, 'via_hole', _board(
        vias=[(20, 20, 0.45, 0.2, 1), (20.63, 20, 0.6, 0.3, 2)]))
    doc, _r = _grade(pcb, tmp, 'via_hole', True, refuse='VIA-HOLE')
    ok('via-hole fires once',
       doc.get('by_type', {}).get('via-hole', 0) == 1
       and doc.get('violations') == 1, doc.get('by_type'))
    items = [it for it in doc.get('items', []) if it.get('type') == 'via-hole']
    ok('the via-hole overlap is 0.020',
       items and abs(items[0].get('overlap_mm', 0) - 0.020) < 1e-9, items)

    print('4. track-hole: a track at clearance, under the hole rule')
    # Track width 0.2 centred at y = 20.425: edge 0.325 off the via centre,
    # 0.10 off its copper (clearance, passes), 0.225 off its hole.
    pcb = _stage(tmp, 'track_hole', _board(
        vias=[(20, 20, 0.45, 0.2, 1)],
        segs=[(15, 20.425, 25, 20.425, 0.2, 2)]))
    doc, _r = _grade(pcb, tmp, 'track_hole', True, refuse='TRACK-HOLE')
    ok('track-hole fires once, 0.025 overlap',
       doc.get('by_type', {}).get('track-hole', 0) == 1
       and doc.get('violations') == 1, doc.get('by_type'))

    print('5. suppression: the clearance row carries a grazing track')
    # Track centred at y = 20.375: edge 0.275 off centre, 0.05 off the copper.
    pcb = _stage(tmp, 'track_suppress', _board(
        vias=[(20, 20, 0.45, 0.2, 1)],
        segs=[(15, 20.375, 25, 20.375, 0.2, 2)]))
    doc, _r = _grade(pcb, tmp, 'track_suppress', True, refuse=None)
    ok('no track-hole beside a failing clearance row',
       doc.get('by_type', {}).get('track-hole', 0) == 0
       and doc.get('violations', 0) >= 1, doc.get('by_type'))

    print('6. suppression: the clearance row carries overlapping vias')
    pcb = _stage(tmp, 'via_suppress', _board(
        vias=[(20, 20, 0.45, 0.2, 1), (20.45, 20, 0.6, 0.3, 2)]))
    doc, _r = _grade(pcb, tmp, 'via_suppress', True, refuse=None)
    ok('no via-hole beside a failing clearance row',
       doc.get('by_type', {}).get('via-hole', 0) == 0
       and doc.get('violations', 0) >= 1, doc.get('by_type'))

    print('7. pad-vs-via-hole reports both rows, like KiCad')
    # Pad edge 0.275 off centre: 0.05 off the via copper (clearance fails)
    # and 0.175 off the hole (overlap 0.075).
    pcb = _stage(tmp, 'pad_both', _board(
        vias=[(20, 20, 0.45, 0.2, 1)],
        pads=[('R1', '1', 0, 0.525, 1.0, 0.5, 2)]))
    doc, _r = _grade(pcb, tmp, 'pad_both', True, refuse='PAD-HOLE')
    ok('pad-hole fires even when the copper clearance also fails',
       doc.get('by_type', {}).get('pad-hole', 0) == 1, doc.get('by_type'))


def router_units(tmp):
    print('8. the descent ladder prefers the copper-to-hole ring')
    # A fresh process: the pair's global must exist before anything installs it.
    check([sys.executable, '-c',
           'import sys; sys.path.insert(0, "py_router"); import fab_tiers; '
           'assert fab_tiers.escalation_rungs(4); '
           'assert fab_tiers.get_via_hole_clearance() is None; print("fresh ok")'],
          accept=True)
    ok('a fresh import walks the ladder with nothing installed', True)
    import fab_tiers
    saved_tier = fab_tiers._DEFAULT_TIER
    saved_esc = fab_tiers._ESCALATION

    def _pairs(rungs):
        return [(r['via_diameter'], r['via_drill']) for r in rungs]
    try:
        fab_tiers.set_default_fab_tier('standard')
        fab_tiers.set_escalation_policy('fab', {})
        raw = fab_tiers.escalation_rungs(4)
        ok('no pair installed: the standard ladder is 0.45/0.2, untagged',
           _pairs(raw) == [(0.45, 0.2)]
           and fab_tiers.TIER_RUNG_KEY not in raw[0], raw)
        fab_tiers.set_via_hole_clearance(0.25, 0.1)
        ok('pair (0.25, 0.1) installs a 0.15 ring',
           abs(fab_tiers.via_hole_ring() - 0.15) < 1e-9, fab_tiers.via_hole_ring())
        std = fab_tiers.escalation_rungs(4)
        ok('pair installed: standard tries 0.5/0.2 first, 0.45/0.2 last',
           _pairs(std) == [(0.5, 0.2), (0.45, 0.2)]
           and [fab_tiers.rung_tier(std, i) for i in range(2)] == [0, 0], std)
        ok('neither standard via is a fab-tier escalation',
           not fab_tiers.ladder_escalates(std, 0.5, 0.2)
           and not fab_tiers.ladder_escalates(std, 0.45, 0.2))
        fab_tiers.set_default_fab_tier('auto')
        auto = fab_tiers.escalation_rungs(4)
        held = [(r['via_diameter'] - r['via_drill']) / 2.0 >= 0.15 - 1e-9 for r in auto]
        first_short = held.index(False) if False in held else len(held)
        ok('tier auto: every ring-holding rung comes before every short one',
           first_short > 0 and not any(held[first_short:]), _pairs(auto))
        ok('tier auto: the tier\'s own rungs are all still there, last',
           _pairs(auto)[first_short:] == [(0.45, 0.2), (0.3, 0.15), (0.25, 0.15)],
           _pairs(auto))
        ok('tier auto: the nominal\'s vias are not escalations, the rest are',
           [fab_tiers.ladder_escalates(auto, d, h) for d, h in
            [(0.5, 0.2), (0.45, 0.2), (0.45, 0.15), (0.3, 0.15)]]
           == [False, False, True, True], _pairs(auto))
        fab_tiers.set_default_fab_tier('standard')
        fab_tiers.set_via_hole_clearance(0.0, 0.1)
        ok('declared hole clearance 0 installs a 0.0 ring: the tier\'s ladder',
           fab_tiers.via_hole_ring() == 0.0
           and _pairs(fab_tiers.escalation_rungs(4)) == [(0.45, 0.2)])
        fab_tiers.set_escalation_policy('off', {})
        fab_tiers.set_via_hole_clearance(0.25, 0.1)
        ok('policy off: no rungs at all, even with the pair installed',
           fab_tiers.escalation_rungs(4) == [])
        fab_tiers.set_escalation_policy('fab', {})
        ok('a new run clears the last run\'s pair',
           fab_tiers.get_via_hole_clearance() is None)
        fab_tiers.set_fab_tier_from_config({'fab_tier': 'standard', 'clearance': 0.1,
                                            'hole_clearance': 0.25})
        ok('the GUI config key installs the pair',
           fab_tiers.get_via_hole_clearance() == (0.25, 0.1))
        fab_tiers.set_fab_tier_from_config({'fab_tier': 'standard', 'clearance': 0.1})
        ok('a GUI config without the key installs none (no leak between runs)',
           fab_tiers.get_via_hole_clearance() is None)
    finally:
        fab_tiers.set_via_hole_clearance(None, None)
        fab_tiers.set_escalation_policy(saved_esc, {})
        fab_tiers.set_default_fab_tier(saved_tier)

    print('9. both fronts resolve the board\'s hole clearance the same way')

    def _pro(tmp, name, rules, origin=None):
        d = os.path.join(tmp, name)
        os.makedirs(d)
        pcb = os.path.join(d, 'b.kicad_pcb')
        with open(pcb, 'w', encoding='utf-8') as f:
            f.write(_board())
        proj = {'board': {'design_settings': {'rules': rules}}}
        if origin is not None:
            proj['kicad_routing_tools'] = {
                'fab_floor_origin': {'min_hole_clearance': origin}}
        with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump(proj, f)
        return pcb

    def _args():
        return SimpleNamespace(clearance=0.1, escalation=None,
                               no_fix_drc_settings=False)
    from kicad_parser import parse_kicad_pcb
    try:
        for name, rules, origin, want in (
                ('policy_declared', {'min_hole_clearance': 0.25}, None, 0.25),
                ('policy_none', {}, None, 0.25),
                ('policy_tight', {'min_hole_clearance': 0.1}, None, 0.1),
                ('policy_zero', {'min_hole_clearance': 0}, None, 0.0),
                ('policy_origin', {'min_hole_clearance': 0.1}, 0.25, 0.25)):
            pcb = _pro(tmp, name, rules, origin)
            fab_tiers.set_policy_from_args(_args(), pcb)
            cli = fab_tiers.get_via_hole_clearance()
            gui = fab_tiers.board_hole_clearance(parse_kicad_pcb(pcb))
            ok(f'{name}: the CLI installs ({want:g}, 0.1) and the PCBData '
               f'resolver reads {want:g}',
               cli is not None and abs(cli[0] - want) < 1e-9
               and abs(cli[1] - 0.1) < 1e-9 and abs(gui - want) < 1e-9,
               (cli, gui))
        # A live board: a declared rule from the provider wins over the file.
        data = parse_kicad_pcb(_pro(tmp, 'policy_live', {}))
        data.live_rules_provider = lambda: {'min_hole_clearance': 0.3}
        ok('a live board\'s declared rule is read first',
           abs(fab_tiers.board_hole_clearance(data) - 0.3) < 1e-9)
    finally:
        fab_tiers.set_via_hole_clearance(None, None)
        fab_tiers.set_escalation_policy(saved_esc, {})

    print('10. the layer-swap shrink loop prefers the ring, then falls back')
    from layer_swap_optimization import _swap_vias_fit_or_shrink

    def _fake_config():
        return SimpleNamespace(
            clearance=0.1, hole_to_hole_clearance=0.2,
            layers=['F.Cu', 'B.Cu'], via_size=0.6, via_drill=0.3,
            max_pair_clearance=lambda: 0.1,
            pair_clearance_inert=lambda: True,
            pair_clearance=lambda a, b, kind=None: 0.1,
            pad_pair_clearance_before_override=lambda pad, net: 0.1,
            rule_floors=lambda net_id: {})

    # A foreign 0.6/0.3 via at (20,20). A new via 0.66 from it: full-size
    # needs 0.70 (fails), (0.55, 0.25) needs 0.675 (fails), (0.5, 0.2)
    # needs 0.65 (fits). Without the ring the drill stays 0.3.
    pcb = _stage(tmp, 'swap', _board(vias=[(20, 20, 0.6, 0.3, 2)]))
    data = parse_kicad_pcb(pcb)
    cfg = _fake_config()
    try:
        fab_tiers.set_default_fab_tier('standard')
        fab_tiers.set_escalation_policy('fab', {})
        fab_tiers.set_via_hole_clearance(0.25, 0.1)
        v = SimpleNamespace(x=20.66, y=20.0, size=0.6, drill=0.3, net_id=1)
        fitted = _swap_vias_fit_or_shrink(data, [v], cfg)
        ok('pair installed: the swap via shrinks to (0.5, 0.2), the ring held',
           fitted and (v.size, v.drill) == (0.5, 0.2), (fitted, v.size, v.drill))
        # 0.63 off: no ring-holding via fits (0.5/0.2 needs 0.65); the
        # last resort 0.45/0.3 needs 0.625 copper and 0.5 hole-to-hole.
        v = SimpleNamespace(x=20.63, y=20.0, size=0.6, drill=0.3, net_id=1)
        fitted = _swap_vias_fit_or_shrink(data, [v], cfg)
        ok('pair installed, no ring-holding via fits: the last resort ships',
           fitted and (v.size, v.drill) == (0.45, 0.3), (fitted, v.size, v.drill))
        fab_tiers.set_via_hole_clearance(None, None)
        v = SimpleNamespace(x=20.66, y=20.0, size=0.6, drill=0.3, net_id=1)
        fitted = _swap_vias_fit_or_shrink(data, [v], cfg)
        ok('no pair: the same swap via shrinks to (0.5, 0.3), drill uncapped',
           fitted and (v.size, v.drill) == (0.5, 0.3), (fitted, v.size, v.drill))
    finally:
        fab_tiers.set_via_hole_clearance(None, None)
        fab_tiers.set_escalation_policy(saved_esc, {})
        fab_tiers.set_default_fab_tier(saved_tier)

    print('11. what ships below the ring is named in the design_rules ledger')
    import io
    import contextlib
    try:
        fab_tiers.set_default_fab_tier('standard')
        fab_tiers.set_escalation_policy('fab', {})
        fab_tiers.set_via_hole_clearance(0.25, 0.1)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rows = fab_tiers.replace_hole_ring_rows(
                [(1, 'A', {(0.45, 0.2): 3, (0.5, 0.2): 2}),
                 (2, 'B', {(0.55, 0.25): 1})], 'census')
            fab_tiers.replace_hole_ring_rows(
                [(1, 'A', {(0.45, 0.2): 3})], 'census')
        ok('only the short pair is a row: 0.45/0.2, 3 vias, at 0.225',
           len(rows) == 1 and rows[0]['count'] == 3
           and rows[0]['via'] == [0.45, 0.2]
           and abs(rows[0]['delivered'] - 0.225) < 1e-9
           and abs(rows[0]['requested'] - 0.25) < 1e-9, rows)
        led = [r for r in fab_tiers.escalation_summary()['narrowed']
               if r['kind'] == fab_tiers.HOLE_RING_KIND]
        ok('a census run twice counts once', len(led) == 1, led)
        ok('the step says so on the console', 'below the 0.15 mm ring' in buf.getvalue(),
           buf.getvalue())
        line = fab_tiers.escalation_report_line()
        ok('the end-of-run Design rules line names it',
           '3 via(s) below the copper-to-hole ring' in line and '0.225' in line, line)
        with contextlib.redirect_stdout(io.StringIO()):
            got = fab_tiers.disclose_hole_ring_vias(
                SimpleNamespace(nets={1: SimpleNamespace(name='A')}),
                [{'size': 0.45, 'drill': 0.2, 'net_id': 1},
                 {'size': 0.45, 'drill': 0.2, 'net_id': 1},
                 {'size': 0.6, 'drill': 0.3, 'net_id': 1}], 0.45, 'fan')
        ok('a via drawn at its request is not named (a choice, not a descent)',
           got == [], got)
        fab_tiers.set_via_hole_clearance(0.0, 0.1)
        ok('no ring, no rows',
           fab_tiers.replace_hole_ring_rows([(1, 'A', {(0.45, 0.2): 1})], 'x') == [])
    finally:
        fab_tiers.set_via_hole_clearance(None, None)
        fab_tiers.set_escalation_policy(saved_esc, {})
        fab_tiers.set_default_fab_tier(saved_tier)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        checker(tmp)
        router_units(tmp)

    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
