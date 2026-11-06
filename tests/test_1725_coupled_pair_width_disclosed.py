#!/usr/bin/env python3
"""#1725: a coupled diff pair's narrowed copper is disclosed per net as shipped.

The pair router records its narrowings per routing ATTEMPT: a `terminal neck`
row each time an attempt necks a connector or terminal leg, and a `pair
partner neck` row for each candidate that passes the intra-pair gate, which a
later gate, a rip-up or a reroute can still discard. So the design_rules rows
counted attempts, copper that never shipped among them, and --strict-sizes
could exit 3 on copper that was never written. route_diff now grades every
member of a pair the member fallback did not route on the copper it ships,
against the width the pair router asks for there (the layer's impedance
width), and replaces the member's track_width rows with one `coupled pair
copper shipped under width` row -- #1679's rule for the fallback members, on
the same helper. The connectors the router draws at the pair's track width to
land on the pads are delivered as asked and are not a row.

  * the rule on its own: attempt rows of both kinds are replaced; a member
    that ships at full width (connectors at the track width, input copper
    narrower) records nothing; the narrowed member gets one row with length
    and delivered width; a pair's class track width is its drawn width;
    fallback members, nets outside the pairs and via rows are untouched;
  * routed: an in-repo board with a 2-layer stackup, --impedance 80 over a
    0.25 --track-width and a wall of another net's pads ending just below the
    pair's path, so one member necks where it passes the end pad: exit 3
    under --strict-sizes, one row for that member whose length and width
    match the WRITTEN board, and none for the partner, which ships at the
    layer width plus its connectors (FAILS on 95e6dc1d: the member carries
    `terminal neck` rows with no length, and the new site is absent).

    python3 tests/test_1725_coupled_pair_width_disclosed.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import uuid
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
SITE = 'coupled pair copper shipped under width'
ATTEMPT_SITES = ('terminal neck', 'pair partner neck')

STACKUP = ('\t\t(stackup\n'
           '\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t\t(layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4") '
           '(epsilon_r 4.5) (loss_tangent 0.02))\n'
           '\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t)\n')


def _wall():
    """Another net's 0.5 mm pads on both layers in a column at x = 14, from
    y = 17 up to the board's top half: the pair passes the end pad."""
    pads, y, k = [], 17.0, 0
    while y > 5.5:
        k += 1
        for sfx, lay in (('', 'F.Cu'), ('b', 'B.Cu')):
            pads.append(f'\t\t(pad "{k}{sfx}" smd rect (at 0 {y - 10:.3f}) (size 0.5 0.5) '
                        f'(layers "{lay}") (net "DPB_P") (uuid "{uuid.uuid4()}"))')
        y -= 0.55
    return ('\t(footprint "T:Wall"\n\t\t(layer "F.Cu")\n'
            f'\t\t(uuid "{uuid.uuid4()}")\n\t\t(at 14 10)\n'
            f'\t\t(property "Reference" "W1" (at 0 -1.5 0) (layer "F.SilkS") '
            f'(uuid "{uuid.uuid4()}")\n\t\t\t(effects (font (size 1 1) (thickness 0.15))))\n'
            '\t\t(attr smd)\n' + '\n'.join(pads) + '\n\t)\n')


PRO = {"net_settings": {"classes": [{
    "name": "Default", "clearance": 0.25, "track_width": 0.25, "diff_pair_gap": 0.25,
    "diff_pair_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3}]}}
TW = 0.25


def t_rule(td):
    import fab_tiers
    from diff_pair_custody import disclose_coupled_pair_widths
    from routing_common import segment_identity
    from routing_config import GridRouteConfig
    from synth import make_net, make_pcb, make_seg
    names = {1: 'A_P', 2: 'A_N', 3: 'B_P', 4: 'B_N', 5: 'X', 6: 'C_P', 7: 'C_N'}
    old = make_seg(0, 9, 3, 9, net_id=2, width=0.2)            # input copper
    segs = [make_seg(0, 0, 1, 0, net_id=1, width=0.1),          # connector
            make_seg(1, 0, 5, 0, net_id=1, width=0.3),
            make_seg(5, 0, 7, 0, net_id=1, width=0.2),          # necked
            make_seg(0, 1, 1, 1, net_id=2, width=0.1),          # connector
            make_seg(1, 1, 7, 1, net_id=2, width=0.3),          # full width
            old,
            make_seg(0, 2, 6, 2, net_id=3, width=0.1),          # fallback member
            make_seg(0, 3, 6, 3, net_id=5, width=0.1),          # not a pair
            make_seg(0, 4, 1, 4, net_id=6, width=0.15),         # class connector
            make_seg(1, 4, 6, 4, net_id=6, width=0.3),
            make_seg(0, 5, 1.5, 5, net_id=7, width=0.1),        # under its 0.15
            make_seg(1.5, 5, 6, 5, net_id=7, width=0.3)]
    pcb = make_pcb(nets={n: make_net(n, nm) for n, nm in names.items()},
                   segments=segs)

    def pair(p, n):
        return (names[p][:1], SimpleNamespace(p_net_id=p, p_net_name=names[p],
                                              n_net_id=n, n_net_name=names[n]))
    pairs = [pair(1, 2), pair(3, 4), pair(6, 7)]
    cfg = GridRouteConfig(layers=['F.Cu'], track_width=0.1,
                          layer_widths={'F.Cu': 0.3})
    prev = fab_tiers.get_escalation_policy()
    try:
        fab_tiers.set_escalation_policy('fab')
        fab_tiers.reset_ledger()
        fab_tiers.note_narrowing(1, 'track_width', 0.3, 0.2, 'pair partner neck')
        fab_tiers.note_narrowing(1, 'track_width', 0.3, 0.0889, 'terminal neck')
        fab_tiers.note_narrowing(2, 'track_width', 0.3, 0.0889, 'terminal neck')
        fab_tiers.note_narrowing(1, 'via_diameter', 0.6, 0.4, 'via')
        fab_tiers.note_narrowing(3, 'track_width', 0.3, 0.1, 'terminal neck')
        fab_tiers.note_narrowing(5, 'track_width', 0.3, 0.1, 'terminal neck')
        disclose_coupled_pair_widths(
            pcb, cfg, pairs, ['B_P', 'B_N'], {segment_identity(old, 'A_N')},
            pair_geom={6: (0.15, 0.1)})
        rows = sorted((x['net'], x['kind'], x['site'], x['delivered'],
                       x.get('length_mm'))
                      for x in fab_tiers.escalation_summary()['narrowed'])
        assert rows == [(1, 'track_width', SITE, 0.2, 2.0),
                        (1, 'via_diameter', 'via', 0.4, None),
                        (3, 'track_width', 'terminal neck', 0.1, None),
                        (5, 'track_width', 'terminal neck', 0.1, None),
                        (7, 'track_width', SITE, 0.1, 1.5)], rows
    finally:
        fab_tiers.reset_ledger()
        fab_tiers.set_escalation_policy(*prev)
    print("  ok  attempt rows replaced; full width with connectors, input copper: "
          "no row; one row per narrowed member; class drawn width; fallback, "
          "other nets and via rows untouched")


def _board(td):
    src = open(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'),
               encoding='utf-8').read()
    i = src.index('\n', src.index('(setup')) + 1
    txt = src[:i] + STACKUP + src[i:]
    k = txt.rstrip().rfind(')')
    txt = txt[:k] + _wall() + txt[k:]
    dst = os.path.join(td, 'cc.kicad_pcb')
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(txt)
    with open(os.path.join(td, 'cc.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    return evidence(dst, 'the staged board')


def t_routed(td):
    from kicad_parser import parse_kicad_pcb
    from routing_common import segment_identity
    src = _board(td)
    out = os.path.join(td, 'o.kicad_pcb')
    r = check(PY + [tool('route_diff.py'), src, out, '--nets', 'DPA_*',
                    '--impedance', '80', '--track-width', str(TW),
                    '--diff-pair-gap', '0.25', '--clearance', '0.25',
                    '--via-size', '0.6', '--via-drill', '0.3',
                    '--layers', 'F.Cu', 'B.Cu', '--strict-sizes'],
              code=3, refuse=SITE)
    js = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    assert js, r.stdout[-1500:]
    s = json.loads(js[-1].split(': ', 1)[1])
    assert s['routed_diff_pairs'] == ['DPA'], s['routed_diff_pairs']
    rows = [x for x in s['design_rules']['narrowed']
            if x['kind'] == 'track_width' and x['net_name'] in ('DPA_P', 'DPA_N')
            or x['site'] in ATTEMPT_SITES]
    assert len(rows) == 1 and rows[0]['site'] == SITE, rows
    row = rows[0]
    # the written board, measured independently
    inp = parse_kicad_pcb(evidence(src, 'the staged board'))
    keys = {segment_identity(g, inp.nets[g.net_id].name)
            for g in inp.segments if g.net_id in inp.nets}
    b = parse_kicad_pcb(evidence(out, 'the routed board'))
    nm = {n: v.name for n, v in b.nets.items()}
    meas = {}
    for g in b.segments:
        n = nm.get(g.net_id)
        if n not in ('DPA_P', 'DPA_N') or segment_identity(g, n) in keys:
            continue
        m = meas.setdefault(n, {'under': 0.0, 'min': None, 'drawn': 0.0, 'full': 0.0})
        L = math.hypot(g.end_x - g.start_x, g.end_y - g.start_y)
        if abs(g.width - TW) <= 1e-4:
            m['drawn'] += L
        elif g.width < row['requested'] - 1e-3:
            m['under'] += L
            m['min'] = g.width if m['min'] is None else min(m['min'], g.width)
        else:
            m['full'] += L
    nar = row['net_name']
    other = 'DPA_N' if nar == 'DPA_P' else 'DPA_P'
    assert abs(row['length_mm'] - meas[nar]['under']) <= 0.02, (row, meas)
    assert abs(row['delivered'] - round(meas[nar]['min'], 4)) <= 1e-4, (row, meas)
    assert row['delivered'] < row['requested'] and row['requested'] > TW, row
    # the partner: connectors at the track width, the rest at the layer width
    assert meas[other]['under'] == 0 and meas[other]['drawn'] > 0 \
        and meas[other]['full'] > 0, meas
    print(f"  ok  routed: exit 3, one row for {nar}: {row['length_mm']} mm at "
          f"{row['delivered']:g} of {row['requested']:g} (the written board: "
          f"{meas[nar]['under']:.2f} mm); {other} at full width with "
          f"{meas[other]['drawn']:.2f} mm of connectors at {TW}: no row")


CASES = [t_rule, t_routed]


def main():
    want = set(sys.argv[1:])
    for fn in CASES:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='t1725_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
