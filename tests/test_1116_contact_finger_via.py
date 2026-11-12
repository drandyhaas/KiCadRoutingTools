#!/usr/bin/env python3
"""No via is placed inside an edge connector's contact finger (#1116).

A drilled hole in a contact finger damages the plating the socket wipes
against, yet nothing kept a via out of one: check_drc is clean on a same-net
via in a same-net pad, and the router dropped its layer change wherever it
was cheapest -- the finger itself. `net_keepouts.derive_contact_fingers` now
turns every copper pad of a mating-contact footprint (a PCB-edge plug,
`floorplan.is_board_copper_plug`, or card-edge fingers,
`part_class.is_card_fingers`) into a via-only net keep-out for every net,
installed by `design_rules.install_design_rules`, which both front-ends pass
through.

The board is written here from s-expressions (the suite image has no KiCad):
a 2-layer 30 x 20 mm board, a four-finger plug J1 (`connect` pads, no model,
`exclude_from_pos_files board_only`) whose pads stand at 90 degrees, and an
SMD pad on B.Cu under finger 1, on the same net, so the cheapest route is a
via in the finger.

  1. derive: one via-only region per finger, binding every net, each
     covering its pad; tracks allowed;
  2. control: the same footprint WITH a 3D model and SMD pads (an assembled
     part) derives nothing, and nor does the plug moved inside the board
     (fingers away from the outline: a pogo or test-pad target);
  3. stamped while a net routes, a region reaches the #568 small via rung
     only on a map that populates it: a rescue clone carries no small rung,
     and one holding the region ALONE would hide every other via block from
     a rung-1 search (#1070), which then laid a via on a foreign track;
  4. route.py --nets SIG: the net routes, check_connected passes, and no via
     touches a finger's copper;
  5. check_drc grades a via in a finger as a counted `keepout` row
     (`contact_finger`), accepted `inherited-keepout` only when the
     `--baseline` board had it, and finds none on the routed board.

    python3 tests/test_1116_contact_finger_via.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402

failures = []
# The fingers (6 mm, centred on the plug) end 0.1 mm inside the bottom edge
# (y = 20): a mating edge stands at the outline (`floorplan.fingers_at_outline`).
PLUG_AT = (10.0, 16.9)
FINGER_X = (-3.75, -1.25, 1.25, 3.75)
FINGER_NETS = ('SIG', 'N2', 'N3', 'N4')

HEAD = '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(13 "F.Paste" user)
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t\t(35 "F.Fab" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t)
'''


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def _ref(ref):
    return ('\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n' % ref)


def _plug(assembled, at=PLUG_AT):
    """J1: four 1.5 x 6 mm fingers at 90 degrees. `assembled` makes it an
    ordinary SMD part with a 3D model (the control)."""
    kind = 'smd' if assembled else 'connect'
    out = ['\t(footprint "Test:Plug"\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n' % at,
           _ref('J1')]
    if not assembled:
        out.append('\t\t(attr exclude_from_pos_files board_only)\n')
    for i, (x, net) in enumerate(zip(FINGER_X, FINGER_NETS), 1):
        out.append('\t\t(pad "%d" %s roundrect\n\t\t\t(at %g 0 90)\n\t\t\t(size 6 1.5)\n'
                   '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(roundrect_rratio 0.067)\n'
                   '\t\t\t(net "%s")\n\t\t)\n' % (i, kind, x, net))
    if assembled:
        out.append('\t\t(model "${KICAD10_3DMODEL_DIR}/Test.3dshapes/Plug.step")\n')
    out.append('\t)\n')
    return ''.join(out)


def _bottom_pad(ref, x, y, net):
    return ('\t(footprint ""\n\t\t(layer "B.Cu")\n\t\t(at %g %g)\n' % (x, y) + _ref(ref) +
            '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n'
            '\t\t\t(layers "B.Cu" "B.Mask")\n\t\t\t(net "%s")\n'
            '\t\t)\n\t)\n' % net)


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def write_board(path, assembled=False, plug_at=PLUG_AT):
    parts = [HEAD, _plug(assembled, plug_at),
             _bottom_pad('U1', PLUG_AT[0] + FINGER_X[0], PLUG_AT[1] + 1.0, 'SIG')]
    corners = [(0, 0), (30, 0), (30, 20), (0, 20)]
    for i in range(4):
        parts.append(_edge(*corners[i], *corners[(i + 1) % 4]))
    parts.append(')\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts))
    return run_utils.evidence(path, 'contact-finger board')


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def touches(via, pad):
    """Does the via's copper disc overlap the pad's (axis-aligned) rectangle?"""
    dx = max(abs(via.x - pad.global_x) - pad.size_x / 2, 0.0)
    dy = max(abs(via.y - pad.global_y) - pad.size_y / 2, 0.0)
    return (dx * dx + dy * dy) ** 0.5 < via.size / 2 - 1e-6


def derive(tmp):
    print('1. derive on the plug')
    try:
        from net_keepouts import derive_contact_fingers
    except ImportError as e:
        ok('net_keepouts.derive_contact_fingers exists', False, str(e))
        return
    from obstacle_map import point_in_polygon
    pcb = parse(write_board(os.path.join(tmp, 'derive.kicad_pcb')))
    fingers = pcb.footprints['J1'].pads
    regs = derive_contact_fingers(pcb)
    ok('one region per finger', len(regs) == 4, repr([r['name'] for r in regs]))
    ok('vias forbidden, tracks allowed',
       all(not r['vias_allowed'] and r['tracks_allowed'] for r in regs))
    names = {n.name for nid, n in pcb.nets.items() if nid}
    ok('every net bound', all(set(r['nets']) == names for r in regs),
       repr(sorted(names)))
    ok('fingers stand at 90 degrees (board-space 1.5 x 6)',
       all(abs(p.size_x - 1.5) < 1e-6 and abs(p.size_y - 6) < 1e-6 for p in fingers))
    ok('each region covers its pad and stops at its edge',
       all(point_in_polygon(p.global_x, p.global_y, r['polygon'])
           and point_in_polygon(p.global_x, p.global_y + 2.9, r['polygon'])
           and not point_in_polygon(p.global_x + 0.8, p.global_y, r['polygon'])
           for p, r in zip(fingers, regs)))
    print('2. control: an assembled part')
    pcb = parse(write_board(os.path.join(tmp, 'control.kicad_pcb'), assembled=True))
    ok('derives nothing', derive_contact_fingers(pcb) == [])
    print('2b. control: the same plug inside the board (a pogo or test-pad target)')
    pcb = parse(write_board(os.path.join(tmp, 'inside.kicad_pcb'), plug_at=(15.0, 9.0)))
    ok('derives nothing', derive_contact_fingers(pcb) == [])


def rungs(tmp):
    print('3. the small via rung')
    import numpy as np
    import net_keepouts as nk
    from obstacle_map import GridObstacleMap
    from obstacle_cache import rung_small_armed
    from routing_config import GridRouteConfig
    if not rung_small_armed():
        ok('the small rung is armed (KICAD_VIA_RUNG=2, the default)', False)
        return
    pcb = parse(write_board(os.path.join(tmp, 'rungs.kicad_pcb')))
    try:
        nk.attach(pcb, nk.derive_contact_fingers(pcb))
    except AttributeError as e:
        ok('net_keepouts.derive_contact_fingers exists', False, str(e))
        return
    sig = next(nid for nid, n in pcb.nets.items() if n.name == 'SIG')
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.2, clearance=0.2,
                          grid_step=0.1, via_size=0.6, via_drill=0.3)
    bare = GridObstacleMap(2)
    with nk.stamp(bare, pcb, (sig,), cfg):
        ok('a map with no small rung: the region is on rung 0',
           bare.rung_len(0) > 0, f'{bare.rung_len(0)} cell(s)')
        ok('... and the small rung stays unpopulated', bare.rung_len(1) == 0,
           f'{bare.rung_len(1)} cell(s)')
    held = GridObstacleMap(2)
    held.add_blocked_vias_small_batch(np.array([[0, 0]], dtype=np.int32))
    with nk.stamp(held, pcb, (sig,), cfg):
        ok('a populated small rung gets the region beside its copper',
           held.rung_len(1) > 1, f'{held.rung_len(1)} cell(s)')
    ok('both maps are as they were after the stamp',
       bare.rung_len(0) == bare.rung_len(1) == 0 and held.rung_len(1) == 1
       and held.rung_len(0) == 0)


def route(tmp):
    print('4. route.py --nets SIG')
    src = write_board(os.path.join(tmp, 'in.kicad_pcb'))
    out = os.path.join(tmp, 'out.kicad_pcb')
    run_utils.check([sys.executable, '-B', '-X', 'utf8',
                     os.path.join(ROOT, 'py_router', 'route.py'), src, out,
                     '--nets', 'SIG', '--track-width', '0.2', '--clearance', '0.2',
                     '--via-size', '0.6', '--via-drill', '0.3'], accept=True)
    pcb = parse(run_utils.evidence(out, 'routed board'))
    sig = {nid for nid, n in pcb.nets.items() if n.name == 'SIG'}
    ok('the net changes layer (a via is laid)',
       any(v.net_id in sig for v in pcb.vias), f'{len(pcb.vias)} via(s)')
    hits = [(round(v.x, 2), round(v.y, 2), f'J1.{p.pad_number}')
            for v in pcb.vias for p in pcb.footprints['J1'].pads if touches(v, p)]
    ok('no via touches a contact finger', not hits, repr(hits))
    run_utils.check([sys.executable, '-B', '-X', 'utf8',
                     os.path.join(ROOT, 'py_router', 'check_connected.py'), out,
                     '--nets', 'SIG'], accept=True)
    ok('SIG connected (check_connected)', True)


    return out


def _finger_rows(board, baseline=None):
    from check_drc import run_drc
    with contextlib.redirect_stdout(io.StringIO()):
        rows = run_drc(board, clearance=0.2, quiet=True, max_print=0,
                       check_sizes=False, baseline=baseline)
    return [r for r in rows if r.get('contact_finger')]


def grade(tmp, routed):
    print('5. check_drc grades a via in a finger')
    src = write_board(os.path.join(tmp, 'grade_in.kicad_pcb'))
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    x, y = PLUG_AT[0] + FINGER_X[1], PLUG_AT[1]
    via = ('\t(via\n\t\t(at %g %g)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
           '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "N2")\n\t)\n' % (x, y))
    bad = os.path.join(tmp, 'grade_via.kicad_pcb')
    with open(bad, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text[:text.rstrip().rfind(')')] + via + ')\n')
    run_utils.evidence(bad, 'board with a via in finger 2')
    rows = _finger_rows(bad)
    ok('one contact-finger row, counted', len(rows) == 1 and not rows[0].get('accepted')
       and rows[0]['type'] == 'keepout' and rows[0]['area'] == 'contact finger J1.2',
       repr(rows))
    rows = _finger_rows(bad, baseline=bad)
    ok('--baseline that had it: accepted inherited-keepout',
       len(rows) == 1 and rows[0].get('accepted') == 'inherited-keepout', repr(rows))
    rows = _finger_rows(bad, baseline=src)
    ok('--baseline without it: still counted',
       len(rows) == 1 and not rows[0].get('accepted'), repr(rows))
    if routed:
        ok('the routed board grades no finger via', _finger_rows(routed) == [],
           repr(_finger_rows(routed)))


def main():
    tmp = tempfile.mkdtemp(prefix='t1116_')
    try:
        derive(tmp)
        rungs(tmp)
        grade(tmp, route(tmp))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
