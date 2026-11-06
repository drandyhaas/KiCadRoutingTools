#!/usr/bin/env python3
"""#1679: a pair member the fallback necks below its layer width is disclosed.

A diff pair with no coupled result is routed member by member in the
single-ended engine. On a run with a layer wider than `--track-width` (an
impedance width) a member that does not fit at that width necks down, and the
copper ships narrower than the member's own layer width. The neck-down ladder
records nothing, so no row reached JSON_SUMMARY.design_rules, the
`Design rules [...]` line or --strict-sizes. route_diff now measures each
fallback member's new copper on the board it ships and replaces the member's
track_width rows with one `pair member fallback copper shipped under width`
row, as route.py does for a net with its own width.

Board: one pair on F.Cu (routed on F.Cu only) and a foreign wall across the
board with a pinch for each member, the pinches too far apart for the pair to
pass coupled. At a 0.5 mm pinch a 0.15 mm track threads it at 0.15 clearance
and the impedance width does not.

  * 0.5 mm pinches: exit 3 under --strict-sizes, one row per member, named,
    and the only track_width rows of the two members (FAILS on 99436a21:
    exit 0, the necked copper in no row at all);
  * control, pinches wide enough for the layer width: no row for either
    member and no track_width row of theirs left in the ledger;
  * the rule on its own: copper the input already had, a member that ships
    at full width after a narrower attempt, and another pair's
    `pair partner neck` row record or lose nothing.

    python3 tests/test_1679_member_fallback_width_disclosed.py
"""
import json
import os
import shutil
import sys
import tempfile
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
SITE = 'pair member fallback copper shipped under width'
MEMBERS = ('DA_P', 'DA_N')
PITCH = 2.4                  # the two members' pads, and the two pinches


def _pad(ref, x, y, net):
    return f'''\t(footprint "T:Pad"
\t\t(layer "F.Cu")
\t\t(uuid "{uuid.uuid4()}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS") (uuid "{uuid.uuid4()}")
\t\t\t(effects (font (size 1 1) (thickness 0.15))))
\t\t(attr smd)
\t\t(pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask")
\t\t\t(net "{net}") (uuid "{uuid.uuid4()}"))
\t)'''


def _seg(x1, y1, x2, y2, w, net):
    return (f'\t(segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) '
            f'(layer "F.Cu") (net "{net}") (uuid "{uuid.uuid4()}"))')


def _board(path, pinch):
    """The wall is 0.2 mm wide at x = 10, open at y = +-PITCH/2 by `pinch`."""
    yp, yn = -PITCH / 2, PITCH / 2
    h = pinch / 2 + 0.1                       # wall end centre off the pinch
    parts = ['''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6) (legacy_teardrops no))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(7 "B.SilkS" user "B.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t\t(35 "F.Fab" user)
\t\t(33 "B.Fab" user)
\t)
\t(setup
\t\t(stackup
\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 1" (type "core") (thickness 0.15) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))
\t\t)
\t\t(pad_to_mask_clearance 0)
\t)''',
             _pad('U1', 3, yp, 'DA_P'), _pad('U2', 3, yn, 'DA_N'),
             _pad('J1', 17, yp, 'DA_P'), _pad('J2', 17, yn, 'DA_N'),
             _pad('U3', 10, -9, 'WALL'),
             _seg(10, -10, 10, yp - h, 0.2, 'WALL'),
             _seg(10, yp + h, 10, yn - h, 0.2, 'WALL'),
             _seg(10, yn + h, 10, 10, 0.2, 'WALL')]
    for x1, y1, x2, y2 in [(0, -10, 20, -10), (20, -10, 20, 10),
                           (20, 10, 0, 10), (0, 10, 0, -10)]:
        parts.append(f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) '
                     f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts") '
                     f'(uuid "{uuid.uuid4()}"))')
    parts.append(')')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(parts) + '\n')
    return evidence(path, 'the synthetic board')


def _route(td, name, pinch, **kw):
    b = _board(os.path.join(td, name + '.kicad_pcb'), pinch)
    return check(PY + [tool('route_diff.py'), b, os.path.join(td, name + '_out.kicad_pcb'),
                       '--nets', 'DA_*', '--layers', 'F.Cu', '--impedance', '90',
                       '--track-width', '0.15', '--clearance', '0.15',
                       '--diff-pair-gap', '0.15', '--no-gnd-vias', '--strict-sizes'],
                 **kw)


def _summary(r):
    js = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    assert js, r.stdout[-1500:]
    return json.loads(js[-1].split(': ', 1)[1])


def _member_rows(s):
    rows = (s.get('design_rules') or {}).get('narrowed') or []
    ids = {r['net'] for r in rows if r.get('net_name') in MEMBERS}
    return [r for r in rows if r['kind'] == 'track_width'
            and (r.get('net_name') in MEMBERS or r['net'] in ids)]


def t_pinch_necks_and_is_disclosed(td):
    r = _route(td, 'pinch', 0.5, code=3, refuse=SITE)
    s = _summary(r)
    fb = s['single_ended_fallback']
    assert sorted(fb['routed']) == sorted(MEMBERS), fb
    assert 'neck-down' in r.stdout, 'no member necked: the case does not hold'
    rows = _member_rows(s)
    assert sorted(x['net_name'] for x in rows) == sorted(MEMBERS), rows
    assert all(x['site'] == SITE and x['delivered'] < x['requested'] - 1e-6
               and x['length_mm'] > 0 for x in rows), rows
    print("  ok  0.5 mm pinches: exit 3, one row per member: "
          + "; ".join(f"{x['net_name']} {x['length_mm']} mm at "
                      f"{x['delivered']:g} of {x['requested']:g}" for x in rows))


def t_control_full_width(td):
    r = _route(td, 'open', 0.7, accept=True)
    s = _summary(r)
    fb = s['single_ended_fallback']
    assert sorted(fb['routed']) == sorted(MEMBERS), fb
    rows = _member_rows(s)
    assert not rows, rows
    assert SITE not in r.stdout
    print("  ok  control (pinches the layer width fits): no member row")


def t_rule(td):
    import fab_tiers
    from diff_pair_custody import disclose_member_fallback_widths
    from routing_common import segment_identity
    from routing_config import GridRouteConfig
    from synth import make_net, make_pcb, make_seg
    A, B, C, D = 1, 2, 3, 4
    names = {A: 'A_P', B: 'B_P', C: 'C_P', D: 'D_P'}
    old = make_seg(0, 3, 4, 3, net_id=C, width=0.1)
    segs = [make_seg(0, 0, 4, 0, net_id=A, width=0.1),         # necked
            make_seg(4, 0, 6, 0, net_id=A, width=0.3),
            make_seg(0, 1, 6, 1, net_id=B, width=0.3),         # full width
            old,                                               # input copper
            make_seg(0, 4, 6, 4, net_id=D, width=0.2)]         # coupled pair
    pcb = make_pcb(nets={n: make_net(n, nm) for n, nm in names.items()},
                   segments=segs)
    cfg = GridRouteConfig(layers=['F.Cu'], track_width=0.1,
                          layer_widths={'F.Cu': 0.3})
    prev = fab_tiers.get_escalation_policy()
    try:
        fab_tiers.set_escalation_policy('fab')
        fab_tiers.reset_ledger()
        fab_tiers.note_narrowing(B, 'track_width', 0.3, 0.1, 'terminal neck')
        fab_tiers.note_narrowing(D, 'track_width', 0.3, 0.2, 'pair partner neck')
        disclose_member_fallback_widths(
            pcb, cfg, ['A_P', 'B_P', 'C_P'], {segment_identity(old, 'C_P')})
        rows = [(x['net'], x['site'], x.get('length_mm'))
                for x in fab_tiers.escalation_summary()['narrowed']]
        assert rows == [(D, 'pair partner neck', None), (A, SITE, 4.0)], rows
        fab_tiers.reset_ledger()
        disclose_member_fallback_widths(
            pcb, GridRouteConfig(layers=['F.Cu'], track_width=0.1), ['A_P'], set())
        assert fab_tiers.escalation_summary()['narrowed'] == []
    finally:
        fab_tiers.reset_ledger()
        fab_tiers.set_escalation_policy(*prev)
    print("  ok  input copper, a member shipped at full width after a narrower "
          "attempt and another pair's partner neck: unchanged; no wide layer: "
          "no row")


CASES = [t_rule, t_pinch_necks_and_is_disclosed, t_control_full_width]


def main():
    want = set(sys.argv[1:])
    for fn in CASES:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='t1679w_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
