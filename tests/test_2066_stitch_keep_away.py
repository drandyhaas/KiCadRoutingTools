#!/usr/bin/env python3
"""#2066: route_planes.py's stitch vias keep out of the design brief's
keep-away bands.

route.py and route_diff.py read the board's design brief `keep_away` rules
engine-side (#1253); the planes step did not, so a /VCC stitch via landed
0.360 mm (edge to edge) from a /CLK- track inside the declared 0.5 mm band,
4.10 mm from the nearest /VCC pad (outside the rule's 1.5 mm free radius).
`create_plane` now resolves the same rules and the stitch pass gives up a
candidate site inside a band, nudging to the next one as it does for an
obstacle.

Chain (the issue's): route_diff the LVDS pairs, route.py the rest, then
route_planes --stitch-vias pours /VCC on F.Cu and B.Cu and stitches it.

1. With the brief: the planes log says the rule was read from the brief and
   the stitch summary counts the band; no new /VCC via is within 0.5 mm (edge
   to edge) of a /CLK+/- or /DATA+/- track. (Fails before the fix: the via
   at (99.50, 61.50) is 0.360 mm from /CLK-.)
2. Control, the same board with the brief removed: the log names no
   keep-away, and the stitch pass places the via at (99.50, 61.50) inside the
   band -- so check 1 is testing the rule, not a board where no site came
   near the pairs.
3. Every site the control has and the brief run does not was within GAP of
   the other side's copper (tracks, vias or pads, measured here
   independently), so the rule dropped only sites inside a band.
4. A malformed brief rule is refused (exit 2) before the board is parsed.

    python3 tests/test_2066_stitch_keep_away.py [--workdir DIR]
"""
import argparse
import json
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from run_utils import check, evidence, tool                    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')
AGG = '/VCC'
VICTIMS = ('/CLK+', '/CLK-', '/DATA+', '/DATA-')
GAP = 0.5
RULE = f"{AGG}:{','.join(VICTIMS)}:{GAP:g}"
#: The site the stitch pass took inside the band before the fix.
BAND_SITE = (99.5, 61.5)
TOL = 1e-6


def _seg_dist(px, py, s):
    ax, ay = s.start_x, s.start_y
    dx, dy = s.end_x - ax, s.end_y - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy or 1)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _pad_dist(px, py, p):
    """Point to a pad's rectangle (corners square: never farther than the
    true rounded pad, so a spacing under GAP here is under GAP there too)."""
    a = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
    dx, dy = px - p.global_x, py - p.global_y
    lx = abs(dx * math.cos(a) + dy * math.sin(a)) - p.size_x / 2
    ly = abs(-dx * math.sin(a) + dy * math.cos(a)) - p.size_y / 2
    return math.hypot(max(lx, 0.0), max(ly, 0.0)) + min(max(lx, ly), 0.0)


def _new_vias(before, after):
    """(pcb, new /VCC vias of `after`, victim net ids)."""
    from kicad_parser import parse_kicad_pcb
    b = parse_kicad_pcb(evidence(before))
    a = parse_kicad_pcb(evidence(after))
    nid = {n.name: i for i, n in a.nets.items()}
    old = {(round(v.x, 3), round(v.y, 3)) for v in b.vias}
    new = [v for v in a.vias if v.net_id == nid[AGG]
           and (round(v.x, 3), round(v.y, 3)) not in old]
    return a, new, {nid[n] for n in VICTIMS}


def _track_spacing(pcb, vic, v):
    """(edge-to-edge spacing, net name) from via `v` to the nearest victim track."""
    return min(((_seg_dist(v.x, v.y, s) - v.size / 2 - s.width / 2,
                 pcb.nets[s.net_id].name)
                for s in pcb.segments if s.net_id in vic), default=(math.inf, ''))


def _copper_spacing(pcb, vic, v):
    """Edge-to-edge spacing from via `v` to any victim copper on its layers."""
    best = _track_spacing(pcb, vic, v)[0]
    for o in pcb.vias:
        if o.net_id in vic:
            best = min(best, math.hypot(v.x - o.x, v.y - o.y) - v.size / 2 - o.size / 2)
    for n in vic:
        for p in pcb.pads_by_net.get(n, []):
            if getattr(p, 'pad_type', '') != 'np_thru_hole':
                best = min(best, _pad_dist(v.x, v.y, p) - v.size / 2)
    return best


def _planes(py, src, dst):
    r = check(py + [tool('route_planes.py'), src, dst, '--nets', AGG, AGG,
                    '--plane-layers', 'F.Cu', 'B.Cu', '--stitch-vias',
                    '--stitch-pitch', '1.0'], accept=True)
    evidence(dst, 'planes output')
    return (r.stdout or '') + (r.stderr or '')


def _check_pour_site_gate(d):
    """5. The site gate sees the other side's POUR too (#2070 made a pour
    part of the band): a via inside the pour or within GAP of its outline
    is in band, one past GAP is not."""
    import contextlib
    import io
    import uuid
    import keep_away
    from kicad_parser import parse_kicad_pcb
    src = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
    text = open(evidence(src, 'pour fixture'), encoding='utf-8').read()
    end = text.rindex('\t(embedded_fonts no)')
    ring = [(12.0, 14.0), (16.0, 14.0), (16.0, 15.0), (12.0, 15.0)]
    pts = ' '.join(f'(xy {x} {y})' for x, y in ring)
    zone = (f'\t(zone (net "DPB_N") (layer "F.Cu") (uuid "{uuid.uuid4()}") '
            f'(hatch edge 0.5)\n\t\t(connect_pads yes (clearance 0.2)) '
            f'(min_thickness 0.1)\n\t\t(fill yes (thermal_gap 0.5) '
            f'(thermal_bridge_width 0.5))\n\t\t(polygon (pts {pts})))\n')
    board = os.path.join(d, 'pour_gate.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as f:
        f.write(text[:end] + zone + text[end:])
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(board, 'pour board'))
        gate = keep_away.ViaBandGate(pcb, ['DPB_N:DPA_P:1.0'],
                                     pcb.board_info.copper_layers, 0.0)
    ids = {n.name: i for i, n in pcb.nets.items()}
    in_band = gate.for_net(ids['DPA_P'])
    assert in_band is not None, "no gate for the rule's victim net"
    rows = [((14.0, 14.5), True, 'inside the pour'),
            ((14.0, 15.8), True, '0.8 mm past its edge (0.5 mm via edge gap)'),
            ((14.0, 16.4), False, '1.4 mm past its edge (1.1 mm via edge gap)')]
    for (x, y), want, what in rows:
        got = in_band(x, y, 0.3)
        assert got is want, (f"a DPA_P via {what} reads in_band={got}, "
                             f"want {want}: the site gate misreads the DPB_N "
                             f"pour's 1.0 mm band")
    print("PASS 5: the site gate keeps a via out of the other side's pour band")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--workdir', help='scratch directory (default: a new temp dir)')
    args = ap.parse_args()
    d = args.workdir or tempfile.mkdtemp(prefix='t2066_')
    os.makedirs(d, exist_ok=True)
    py = [sys.executable, '-X', 'utf8', '-B']

    src = os.path.join(d, 'in.kicad_pcb')
    check(py + [tool('copy_board.py'), BOARD, src], accept=True)
    with open(os.path.join(d, 'in.design-brief.json'), 'w', encoding='utf-8') as f:
        json.dump({'keep_away': [{'rule': RULE,
                                  'why': 'supply copper off the LVDS pairs'}]}, f)
    d1, r1, p1 = (os.path.join(d, f'{s}.kicad_pcb') for s in ('d1', 'r1', 'p1'))
    check(py + [tool('route_diff.py'), src, d1, '--nets', *VICTIMS], accept=True)
    check(py + [tool('route.py'), evidence(d1), r1], accept=True)
    evidence(r1, 'routed board')
    evidence(os.path.join(d, 'r1.design-brief.json'), 'brief carried to r1')

    # 1. With the brief.
    log = _planes(py, r1, p1)
    pcb, new, vic = _new_vias(r1, p1)
    assert new, "the planes step stitched nothing: the check below tests nothing"
    hits = [(round(v.x, 2), round(v.y, 2), round(sp, 3), net)
            for v in new for sp, net in [_track_spacing(pcb, vic, v)]
            if sp < GAP - TOL]
    assert not hits, (f"{len(hits)} of {len(new)} new {AGG} stitch vias are "
                      f"inside the {GAP} mm band of a pair track: {hits}")
    want = "Keep-away: 1 rule(s) read from the design brief r1.design-brief.json"
    assert want in log, f"the planes step did not read the brief:\n{log[-2000:]}"
    assert 'in a keep-away band:' in log, \
        f"the stitch summary does not count the band:\n{log[-2000:]}"
    print(f"PASS 1: {len(new)} {AGG} stitch vias, none within {GAP} mm of a pair "
          f"track; the planes log reads the brief and counts the band")

    # 2. Control: the brief removed.
    nb = os.path.join(d, 'nobrief')
    os.makedirs(nb, exist_ok=True)
    nb_r1, nb_p1 = os.path.join(nb, 'r1.kicad_pcb'), os.path.join(nb, 'p1.kicad_pcb')
    check(py + [tool('copy_board.py'), r1, nb_r1], accept=True)
    os.remove(evidence(os.path.join(nb, 'r1.design-brief.json'), 'copied brief'))
    nb_log = _planes(py, nb_r1, nb_p1)
    assert 'keep-away' not in nb_log.lower(), \
        f"a board with no brief mentions a keep-away:\n{nb_log[-2000:]}"
    nb_pcb, nb_new, nb_vic = _new_vias(nb_r1, nb_p1)
    at = [v for v in nb_new if math.hypot(v.x - BAND_SITE[0], v.y - BAND_SITE[1]) < 1e-3]
    assert at, (f"the control placed no via at {BAND_SITE}: the board no longer "
                f"reaches the band, so check 1 proves nothing")
    sp, net = _track_spacing(nb_pcb, nb_vic, at[0])
    assert sp < GAP - TOL, (f"the control's via at {BAND_SITE} is {sp:.3f} mm "
                            f"from {net}, outside the band")
    print(f"PASS 2: without the brief, {len(nb_new)} stitch vias, the one at "
          f"{BAND_SITE} {sp:.3f} mm from {net}; no keep-away in the log")

    # 3. What the rule took away was inside a band.
    kept = {(round(v.x, 3), round(v.y, 3)) for v in new}
    dropped = [v for v in nb_new if (round(v.x, 3), round(v.y, 3)) not in kept]
    assert dropped, "the brief changed no site"
    outside = [(round(v.x, 2), round(v.y, 2), round(s, 3)) for v in dropped
               for s in [_copper_spacing(nb_pcb, nb_vic, v)] if s >= GAP - TOL]
    assert not outside, (f"the brief run dropped control sites that were "
                         f"outside every band: {outside}")
    print(f"PASS 3: the {len(dropped)} control site(s) the brief run does not "
          f"have were all inside a band")

    # 4. A malformed rule in the brief is a usage error, as route.py makes it.
    bad = os.path.join(d, 'bad')
    os.makedirs(bad, exist_ok=True)
    bad_in = os.path.join(bad, 'r1.kicad_pcb')
    check(py + [tool('copy_board.py'), r1, bad_in], accept=True)
    with open(evidence(os.path.join(bad, 'r1.design-brief.json'), 'copied brief'),
              'w', encoding='utf-8') as f:
        json.dump({'keep_away': [f"{AGG}:{VICTIMS[0]}"]}, f)
    check(py + [tool('route_planes.py'), bad_in, os.path.join(bad, 'p1.kicad_pcb'),
                '--nets', AGG, AGG, '--plane-layers', 'F.Cu', 'B.Cu',
                '--stitch-vias'], refuse='expected AGGRESSOR:VICTIM:GAP', code=2)
    print("PASS 4: a malformed brief rule is refused before the stitch pass")

    _check_pour_site_gate(d)

    if not args.workdir:
        shutil.rmtree(d, ignore_errors=True)
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
