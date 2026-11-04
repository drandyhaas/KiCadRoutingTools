#!/usr/bin/env python3
"""#1442: a mounting hole's screw-head ring is checked, and can be declared.

KiCad checks a mounting hole against the ordinary hole clearance only, so
route.py laid +5V and signal tracks 0.10-0.25 mm from holes whose M3 head
covers 1.3 mm past the edge, and no checker saw it.

  1. mounting_holes: a footprint that is ONE round hole (9 pads or fewer, all
     round drills of 2.0 mm or wider at one position, none on a signal net)
     is a hole; a via-sized drill, a signal pad, a ground-tied hole, a milled
     slot and a part of several holes (a stabilizer) are told apart.
  2. check_mounting_holes: exit 1, naming the hole and the nets, when another
     net's track or via is inside the ring; exit 0 with the track outside it
     or on the hole's own net.
  3. add_rule_area --mounting-holes writes one area per net-less hole (the
     ground-tied hole is left open, said so), re-running replaces it, and the
     parser reads it back with tracks and vias not allowed.
  4. route.py: a net routed past a hole comes within the ring without the
     areas (non-vacuity), and keeps out of it with them.
  5. KiCad's stock variants are holes too: ``_Pad_Via`` (the hole plus small
     plated stitching vias in its pad; the in-repo glasgow_revC MK1-MK4) and
     ``_Pad_TopBottom`` (drill-less F/B pads over the drilled one), centred
     on the screw hole; a stitching via outside the pad, a via-sized drill
     too large to be one, and drill-less copper off the hole are refused. A
     hole on an ``unconnected-*`` pin net gets its keep-out; a GND one is
     still left open.

    python3 tests/test_1442_mounting_hole_keepout.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from synth import RoutingBoard                                 # noqa: E402
from run_utils import check as run_check, evidence, tool       # noqa: E402

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def hole(rb, ref, x, y, drill=3.2, net=''):
    """A one-pad mounting-hole footprint: NPTH, or plated on `net`."""
    if net:
        n = rb.net(net)
        pad = ('(pad "1" thru_hole circle (at 0 0) (size %s %s) (drill %s) '
               '(layers "*.Cu" "*.Mask") (net %d "%s") (uuid "%s"))'
               % (drill + 1, drill + 1, drill, n, net, rb._uid()))
    else:
        pad = ('(pad "" np_thru_hole circle (at 0 0) (size %s %s) (drill %s) '
               '(layers "*.Cu" "*.Mask") (uuid "%s"))'
               % (drill, drill, drill, rb._uid()))
    rb.items.append(
        '  (footprint "synth:MH" (layer "F.Cu") (uuid "%s") (at %s %s)\n'
        '    (property "Reference" "%s" (at 0 -4 0) (layer "F.SilkS") '
        '(uuid "%s")\n      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        '    %s\n  )' % (rb._uid(), x, y, ref, rb._uid(), pad))


def multi_hole(rb, ref, x, y, holes):
    """A footprint of several NPTH holes: [(dx, dy, drill)] -- a keyboard
    stabilizer's shape, which takes no screw."""
    pads = ' '.join('(pad "" np_thru_hole circle (at %s %s) (size %s %s) '
                    '(drill %s) (layers "*.Cu" "*.Mask") (uuid "%s"))'
                    % (dx, dy, d, d, d, rb._uid()) for dx, dy, d in holes)
    rb.items.append(
        '  (footprint "synth:STAB" (layer "F.Cu") (uuid "%s") (at %s %s)\n'
        '    (property "Reference" "%s" (at 0 -4 0) (layer "F.SilkS") '
        '(uuid "%s")\n      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        '    %s\n  )' % (rb._uid(), x, y, ref, rb._uid(), pads))


def slot(rb, ref, x, y, w, h):
    """A one-pad footprint whose drill is a milled slot (drill oval w h)."""
    rb.items.append(
        '  (footprint "synth:SLOT" (layer "F.Cu") (uuid "%s") (at %s %s)\n'
        '    (property "Reference" "%s" (at 0 -4 0) (layer "F.SilkS") '
        '(uuid "%s")\n      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        '    (pad "" np_thru_hole oval (at 0 0) (size %s %s) (drill oval %s %s) '
        '(layers "*.Cu" "*.Mask") (uuid "%s"))\n  )'
        % (rb._uid(), x, y, ref, rb._uid(), w, h, w, h, rb._uid()))


def part(rb, ref, x, y, pads):
    """A footprint of the given pad s-expressions (uuids filled in)."""
    body = ' '.join(p % rb._uid() for p in pads)
    rb.items.append(
        '  (footprint "synth:MHX" (layer "F.Cu") (uuid "%s") (at %s %s)\n'
        '    (property "Reference" "%s" (at 0 -5 0) (layer "F.SilkS") '
        '(uuid "%s")\n      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        '    %s\n  )' % (rb._uid(), x, y, ref, rb._uid(), body))


def thru(dx, dy, size, drill, rb, net):
    n = rb.net(net) if net else 0
    netx = ' (net %d "%s")' % (n, net) if net else ''
    return ('(pad "1" thru_hole circle (at %s %s) (size %s %s) (drill %s) '
            '(layers "*.Cu" "*.Mask")%s (uuid "%%s"))'
            % (dx, dy, size, size, drill, netx))


def flat(dx, dy, size, layer, rb, net, kind='connect'):
    n = rb.net(net) if net else 0
    netx = ' (net %d "%s")' % (n, net) if net else ''
    return ('(pad "1" %s circle (at %s %s) (size %s %s) (layers "%s" "%s")%s '
            '(uuid "%%s"))' % (kind, dx, dy, size, size, layer,
                               layer.replace('Cu', 'Mask'), netx))


def pad_via(rb, ref, x, y, net, drill=3.5, size=7.0, via_r=2.625,
            via_drill=0.5):
    """KiCad's MountingHole_*_Pad_Via: the hole plus 8 stitching via-pads."""
    import math
    pads = [thru(0, 0, size, drill, rb, net)]
    for k in range(8):
        a = 2 * math.pi * k / 8
        pads.append(thru(round(via_r * math.cos(a), 6), round(via_r * math.sin(a), 6),
                         0.8, via_drill, rb, net))
    part(rb, ref, x, y, pads)


def write(rb, path):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(rb.text())
    return evidence(path, 'the synthetic board')


def py(script, *args, **kw):
    return run_check([sys.executable, '-X', 'utf8', script, *args], **kw)


def parse(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def main():
    from mounting_holes import mounting_holes, hole_copper
    checker = tool('check_mounting_holes.py')
    adder = tool('add_rule_area.py')
    with tempfile.TemporaryDirectory() as tmp:
        print('1: which footprints are mounting holes')
        rb = RoutingBoard(w=60, h=40)
        hole(rb, 'H1', 10, 10)                       # an M3 NPTH
        hole(rb, 'H2', 30, 10, drill=2.2)            # an M2 NPTH
        hole(rb, 'H3', 50, 10, net='GND')            # plated, ground-tied
        hole(rb, 'H4', 10, 30, drill=0.9)            # a via-sized drill
        hole(rb, 'H5', 30, 30, net='+5V')            # a signal pad
        multi_hole(rb, 'S1', 45, 30, [(-6, 0, 3.05), (6, 0, 3.05),
                                      (-6, 7, 3.99), (6, 7, 3.99)])  # a stabilizer
        slot(rb, 'S2', 52, 20, 6.0, 2.4)             # a milled slot
        p = write(rb, os.path.join(tmp, 'holes.kicad_pcb'))
        found = {h.ref: h for h in mounting_holes(parse(p))}
        ok('M3, M2 and the ground-tied hole are mounting holes; the via-sized '
           'drill, the signal pad, the stabilizer and the slot are not',
           sorted(found) == ['H1', 'H2', 'H3'], str(sorted(found)))
        ok('the ground-tied hole carries its net', bool(found['H3'].nets))
        ok('the ring is 0.4 x the drill',
           abs(found['H1'].ring() - 1.28) < 1e-9
           and abs(found['H2'].ring() - 0.88) < 1e-9)

        print('2: check_mounting_holes')
        rb = RoutingBoard(w=60, h=40)
        hole(rb, 'H1', 20, 20)
        rb.pad('R1', 5, 20, '/SIG')
        rb.pad('R2', 35, 20, '/SIG')
        rb.seg(5, 20, 35, 20, '/SIG', w=0.3)         # through the hole's ring
        p = write(rb, os.path.join(tmp, 'in_ring.kicad_pcb'))
        r = run_check([sys.executable, '-X', 'utf8', checker, p],
                      refuse='MOUNTING HOLE COPPER', code=1)
        ok('exit 1 names the hole and the net', 'H1' in r.stdout and '/SIG' in r.stdout,
           r.stdout[-300:])
        rb = RoutingBoard(w=60, h=40)
        hole(rb, 'H1', 20, 20)
        rb.pad('R1', 5, 20, '/SIG')
        rb.pad('R2', 35, 20, '/SIG')
        rb.seg(5, 20, 5, 30, '/SIG', w=0.3)
        rb.seg(5, 30, 35, 30, '/SIG', w=0.3)         # 10 mm away
        rb.seg(35, 30, 35, 20, '/SIG', w=0.3)
        p = write(rb, os.path.join(tmp, 'clear.kicad_pcb'))
        r = py(checker, p, accept=True)
        ok('a track outside the ring passes', 'ring(s) clear' in r.stdout, r.stdout[-200:])
        rb = RoutingBoard(w=60, h=40)
        hole(rb, 'H1', 20, 20, net='GND')
        rb.pad('R1', 5, 20, 'GND')
        rb.pad('R2', 35, 20, 'GND')
        rb.seg(5, 20, 35, 20, 'GND', w=0.3)          # the hole's own net
        p = write(rb, os.path.join(tmp, 'own_net.kicad_pcb'))
        py(checker, p, accept=True)
        ok("copper of the hole's own net passes", True)
        rb = RoutingBoard(w=60, h=40)
        hole(rb, 'H1', 20, 20)
        rb.via(21.5, 20, '/SIG')                     # inside the ring, a via
        p = write(rb, os.path.join(tmp, 'via.kicad_pcb'))
        run_check([sys.executable, '-X', 'utf8', checker, p],
                  refuse='under the screw head', code=1)
        ok('a via inside the ring fails', True)

        print('3: add_rule_area --mounting-holes')
        src = os.path.join(tmp, 'holes.kicad_pcb')
        out = os.path.join(tmp, 'with_areas.kicad_pcb')
        r = py(adder, src, out, '--name', 'MOUNT', '--mounting-holes', accept=True)
        ok('the ground-tied hole is left open, and it says so',
           'H3' in r.stdout and 'left open' in r.stdout, r.stdout[-300:])
        kos = parse(out).board_info.keepouts
        ok('one area per net-less hole, tracks and vias not allowed, pour allowed',
           len(kos) == 2 and all(k.get('tracks_allowed') is False
                                 and k.get('vias_allowed') is False
                                 and k.get('copper_pour_allowed') is not False
                                 for k in kos), str(kos)[:300])
        cx = sorted(sum(x for x, _ in k['polygon']) / len(k['polygon']) for k in kos)
        ok('centred on the holes (10 and 30 mm)',
           abs(cx[0] - 10) < 1e-6 and abs(cx[1] - 30) < 1e-6, str(cx))
        r = py(adder, out, out, '--name', 'MOUNT', '--mounting-holes', accept=True)
        ok('a re-run replaces them', len(parse(out).board_info.keepouts) == 2
           and 'replaced 1 earlier area' in r.stdout, r.stdout[-200:])
        # #1455: every hole carries a net -> exit 0 still writes the output.
        src = os.path.join(tmp, 'own_net.kicad_pcb')
        out = os.path.join(tmp, 'all_netted.kicad_pcb')
        r = py(adder, src, out, '--name', 'MOUNT', '--mounting-holes', accept=True)
        with open(src, 'rb') as a:
            with open(evidence(out, 'the all-netted output'), 'rb') as b:
                same = a.read() == b.read()
        ok('with no hole to ring, the output is the input, unchanged',
           same and 'no mounting hole' in r.stdout, r.stdout[-300:])
        py(adder, src, os.path.join(tmp, 'x.kicad_pcb'), '--name', 'M',
           '--mounting-holes', '--ref', 'H1', refuse='finds the holes itself', code=2,
           allow=('usage:',))

        print('4: route.py keeps out of the ring')
        def board(path):
            rb = RoutingBoard(w=60, h=40)
            hole(rb, 'H1', 30, 20)
            rb.pad('R1', 10, 20, '/SIG')
            rb.pad('R2', 50, 20, '/SIG')
            return write(rb, path)
        route = tool('route.py')
        free = board(os.path.join(tmp, 'free.kicad_pcb'))
        rout = os.path.join(tmp, 'free_routed.kicad_pcb')
        py(route, free, rout, '--nets', '/SIG', accept=True, timeout=600)
        near = [h for _h, hs in hole_copper(parse(rout)) for h in hs]
        ok('without the area the route comes inside the ring (non-vacuity)',
           bool(near), str(near[:2]))
        areas = os.path.join(tmp, 'areas.kicad_pcb')
        py(adder, free, areas, '--name', 'MOUNT', '--mounting-holes', accept=True)
        rout2 = os.path.join(tmp, 'areas_routed.kicad_pcb')
        py(route, areas, rout2, '--nets', '/SIG', accept=True, timeout=600)
        segs = [s for s in parse(rout2).segments if s.net_id]
        near2 = [h for _h, hs in hole_copper(parse(rout2)) for h in hs]
        ok('with the area the route keeps out, and is still routed',
           not near2 and len(segs) >= 1, f'{len(segs)} segments, {near2[:2]}')
        py(checker, rout2, accept=True)

        print('5: stock variants, and a hole on an unconnected-* pin net')
        rb = RoutingBoard(w=100, h=40)
        pad_via(rb, 'H1', 10, 10, 'GND')                         # _Pad_Via
        part(rb, 'H2', 30, 10, [thru(0, 0, 6.0, 3.2, rb, 'GND'),  # _Pad_TopBottom
                                flat(0, 0, 6.0, 'F.Cu', rb, 'GND'),
                                flat(0, 0, 6.0, 'B.Cu', rb, 'GND')])
        part(rb, 'H3', 50, 10, [flat(0, 0, 6.0, 'F.Cu', rb, ''),  # NPTH + F/B
                                '(pad "" np_thru_hole circle (at 0 0) (size 3.2 3.2) '
                                '(drill 3.2) (layers "*.Cu" "*.Mask") (uuid "%s"))',
                                flat(0, 0, 6.0, 'B.Cu', rb, '', kind='smd')])
        hole(rb, 'H4', 70, 10, net='unconnected-(H4-Pad1)')      # pin left open
        pad_via(rb, 'X1', 10, 30, 'GND', via_r=3.6)    # a via outside the pad
        pad_via(rb, 'X2', 30, 30, 'GND', via_drill=1.9)  # too big for a stitch
        part(rb, 'X3', 50, 30, [thru(0, 0, 6.0, 3.2, rb, 'GND'),  # copper off it
                                flat(1.5, 0, 1.0, 'F.Cu', rb, 'GND', kind='smd')])
        pad_via(rb, 'X4', 70, 30, 'GND', drill=1.8, size=3.6, via_r=1.2,
                via_drill=0.3)                         # a via-sized main drill
        p = write(rb, os.path.join(tmp, 'variants.kicad_pcb'))
        found = {h.ref: h for h in mounting_holes(parse(p))}
        ok('_Pad_Via, _Pad_TopBottom (plated and NPTH) and the unconnected-* hole '
           'are holes; the off-pad via, the oversized stitch, the off-hole copper '
           'and the small main drill are not',
           sorted(found) == ['H1', 'H2', 'H3', 'H4'], str(sorted(found)))
        h1 = found.get('H1')
        ok('_Pad_Via is centred on its screw hole with its drill',
           h1 is not None and abs(h1.x - 10) < 1e-6 and abs(h1.y - 10) < 1e-6
           and abs(h1.drill - 3.5) < 1e-9, str(h1))
        ok('the unconnected-* hole carries no net, the GND ones do',
           'H4' in found and not found['H4'].nets
           and bool(found.get('H1') and found['H1'].nets)
           and bool(found.get('H2') and found['H2'].nets))
        out = os.path.join(tmp, 'variants_areas.kicad_pcb')
        r = py(adder, p, out, '--name', 'MOUNT', '--mounting-holes', accept=True)
        kos = parse(out).board_info.keepouts
        names = sorted(k.get('name', '') for k in kos)
        ok('the unconnected-* hole and the net-less NPTH get an area; the GND '
           'holes are left open', names == ['MOUNT_H3', 'MOUNT_H4']
           and 'H1' in r.stdout and 'H2' in r.stdout and 'left open' in r.stdout,
           f'{names} {r.stdout[-200:]}')

        glasgow = evidence(os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb'),
                           'the in-repo _Pad_Via board')
        g = parse(glasgow)
        found = {h.ref: h for h in mounting_holes(g)}
        mk = sorted(r for r in found if r.startswith('MK'))
        ok("glasgow_revC's four _Pad_Via holes are found",
           mk == ['MK1', 'MK2', 'MK3', 'MK4'], str(sorted(found)))
        ok('each centred on its 3.5 mm drill',
           bool(mk) and all(abs(found[r].drill - 3.5) < 1e-9
               and any(p.drill == 3.5 and abs(p.global_x - found[r].x) < 1e-6
                       and abs(p.global_y - found[r].y) < 1e-6
                       for p in g.footprints[r].pads) for r in mk), str(mk))

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print('\nAll checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
