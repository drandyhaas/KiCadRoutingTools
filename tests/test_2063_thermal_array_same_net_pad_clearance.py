#!/usr/bin/env python3
"""--same-net-pad-clearance leaves an exposed pad its own thermal via array,
clear of the paste (#2063).

The plane step lays a thermal via ARRAY under an exposed pad (#487), and
nothing else in the chain places that pad's vias. `compute_thermal_via_array`
keeps a site only where the via map is open, and the #581 same-net pad
keep-out covers every same-net SMD pad -- the exposed pad included, so with
the flag set every lattice site was blocked and the pad ended with no via
("thermal array did not fit -- deferred to the route step").

The array is the pad's intended via site, so it is exempt from THAT pad's
keep-out. It is not exempt from the pad's solder-paste openings (#962), which
it must still clear at the same margin the paste keep-out uses
(via/2 + the same-net clearance), nor from any other pad's keep-out.

Checks, each through route_planes.py --nets GND --plane-layers B.Cu:
  A. a paste-less exposed pad: the array is laid with the flag as without it
     (on main: 9 vias without, 0 with)
  B. a windowpaned exposed pad (four paste-only panes): with the flag the
     array keeps only the sites between the panes -- fewer than without the
     flag, more than none -- and every via clears every paste opening by the
     same-net clearance (to half a grid step)
  C. an exposed pad whose own paste opening covers it: no site survives, and
     the "did not fit" line says the paste openings are why
  D. the flag still keeps the array off the neighbouring same-net pad's
     keep-out (a via never lands within the clearance of another GND pad)

    python3 tests/test_2063_thermal_array_same_net_pad_clearance.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence, tool    # noqa: E402

FAILS = []
SNPC = 0.25
VIA, DRILL, GRID = 0.6, 0.3, 0.1
EP = 4.0                      # exposed pad, mm square
UX, UY = 112.0, 110.0         # its centre


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


_N = [0]


def _uid():
    _N[0] += 1
    return '00000000-0000-4000-8000-%012d' % _N[0]


_HEAD = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "test_2063")\n'
         '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n\t)\n'
         '\t(paper "A4")\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
         '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
         '\t\t(13 "F.Paste" user)\n\t\t(15 "B.Paste" user)\n'
         '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
         '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n')


def _pad(num, net, x, y, w, h, layers):
    node = '\t\t\t(net "%s")\n' % net if net else ''
    return ('\t\t(pad "%s" smd rect\n\t\t\t(at %s %s)\n\t\t\t(size %s %s)\n'
            '\t\t\t(layers %s)\n%s\t\t\t(uuid "%s")\n\t\t)\n'
            % (num, x, y, w, h, ' '.join('"%s"' % l for l in layers), node, _uid()))


def _fp(ref, x, y, pads, name='TestPoint:TP'):
    return ('\t(footprint "%s"\n\t\t(layer "F.Cu")\n\t\t(uuid "%s")\n'
            '\t\t(at %s %s)\n\t\t(property "Reference" "%s"\n\t\t\t(at 0 -3 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "%s")\n\t\t)\n%s\t)\n'
            % (name, _uid(), x, y, ref, _uid(), ''.join(pads)))


def _outline(pts):
    return ''.join('\t(gr_line\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(stroke\n'
                   '\t\t\t(width 0.05)\n\t\t\t(type default)\n\t\t)\n'
                   '\t\t(layer "Edge.Cuts")\n\t\t(uuid "%s")\n\t)\n'
                   % (a[0], a[1], b[0], b[1], _uid())
                   for a, b in zip(pts, pts[1:] + pts[:1]))


def board(paste, neighbour=False):
    """U1: a GND exposed pad and one SIGB pin, with `paste` one of
    'none' (copper + mask only), 'panes' (four paste-only windowpanes that
    leave a cross of bare copper through the middle) or 'full' (the pad's
    own opening covers it). `neighbour` adds a second GND pad beside the
    exposed pad, inside reach of the array's edge column."""
    ep_layers = ['F.Cu', 'F.Mask'] + (['F.Paste'] if paste == 'full' else [])
    pads = [_pad('17', 'GND', 0, 0, EP, EP, ep_layers),
            _pad('2', 'SIGB', 2.75, 1.5, 0.5, 0.5, ['F.Cu', 'F.Mask', 'F.Paste'])]
    if paste == 'panes':
        for sx in (-1, 1):
            for sy in (-1, 1):
                pads.append(_pad('', None, 1.05 * sx, 1.05 * sy, 0.9, 0.9,
                                 ['F.Paste']))
    if neighbour:
        # 0.2 mm off the exposed pad's left edge: the array's left column
        # sits a via radius inside that edge, so those vias would land 0.2 mm
        # from the neighbour, inside its #581 keep-out.
        pads.append(_pad('3', 'GND', -2.45, 0, 0.5, 0.5,
                         ['F.Cu', 'F.Mask', 'F.Paste']))
    return (_HEAD
            + _fp('U1', UX, UY, pads, name='Package_DFN_QFN:QFN-16-1EP_4x4mm')
            + _fp('TP1', 104, 104, [_pad('1', 'GND', 0, 0, 1.0, 1.0,
                                         ['F.Cu', 'F.Mask', 'F.Paste'])])
            + _fp('TP2', 120, 116, [_pad('1', 'SIGB', 0, 0, 1.0, 1.0,
                                         ['F.Cu', 'F.Mask', 'F.Paste'])])
            + _outline([(100, 100), (124, 100), (124, 120), (100, 120)])
            + ')\n')


def project(name):
    cls = {'name': 'Default', 'clearance': 0.2, 'track_width': 0.2,
           'via_diameter': VIA, 'via_drill': DRILL, 'diff_pair_gap': 0.25,
           'diff_pair_width': 0.2}
    return {'board': {'design_settings': {'rules': {
                'min_clearance': 0.0, 'min_track_width': 0.15,
                'min_via_diameter': 0.5, 'min_through_hole_diameter': 0.3,
                'min_hole_to_hole': 0.25, 'min_hole_clearance': 0.25,
                'min_copper_edge_clearance': 0.5}, 'rule_severities': {}}},
            'meta': {'filename': name + '.kicad_pro', 'version': 3},
            'net_settings': {'classes': [cls], 'meta': {'version': 4}}}


def stage(d, name, text):
    pcb = os.path.join(d, name + '.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    with open(os.path.join(d, name + '.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(project(name), f, indent=2)
    return evidence(pcb, 'staged board')


def plane(d, name, text, snpc):
    """route_planes on a fresh copy of the board; (array vias, output, pcb)."""
    from kicad_parser import parse_kicad_pcb
    src = stage(d, name, text)
    out = os.path.join(d, name + '_out.kicad_pcb')
    argv = [sys.executable, '-X', 'utf8', tool('route_planes.py'), src, out,
            '--nets', 'GND', '--plane-layers', 'B.Cu',
            '--via-size', VIA, '--via-drill', DRILL, '--grid-step', GRID]
    if snpc is not None:
        argv += ['--same-net-pad-clearance', snpc]
    r = run_check(argv, accept=True)
    pcb = quiet(parse_kicad_pcb, evidence(out, 'route_planes output'))
    gnd = next(n for n, net in pcb.nets.items() if net.name == 'GND')
    h = EP / 2
    vias = [v for v in pcb.vias if v.net_id == gnd
            and abs(v.x - UX) <= h and abs(v.y - UY) <= h]
    return vias, r.stdout + r.stderr, pcb, gnd


def paste_gap(pcb, gnd, vias):
    """Smallest barrel-edge to paste-opening distance over the vias, mm."""
    from paste_apertures import apertures_for_net, aperture_distance
    aps = apertures_for_net(pcb, gnd)
    gaps = [aperture_distance(v.x, v.y, ap) - v.size / 2
            for v in vias for ap in aps]
    return (min(gaps) if gaps else float('inf')), len(aps)


def main():
    d = tempfile.mkdtemp(prefix='t2063_')
    try:
        print('A. a paste-less exposed pad keeps its array under the flag')
        off, _, _, _ = plane(d, 'a_off', board('none'), None)
        on, log, _, _ = plane(d, 'a_on', board('none'), SNPC)
        n_a = len(on)
        check('without the flag the array is laid (the path under test)',
              len(off) > 0, f'{len(off)} via(s)')
        check(f'with --same-net-pad-clearance {SNPC} the same array is laid',
              len(on) == len(off), f'{len(on)} vs {len(off)} via(s)')
        check('and no "did not fit" deferral', 'did not fit' not in log)

        print('B. a windowpaned exposed pad keeps the sites between its panes')
        off, _, _, _ = plane(d, 'b_off', board('panes'), None)
        on, log, pcb, gnd = plane(d, 'b_on', board('panes'), SNPC)
        gap, n_aps = paste_gap(pcb, gnd, on)
        check('the board carries the four paste-only panes (the path under test)',
              n_aps >= 4, f'{n_aps} opening(s) on GND')
        check('with the flag the array keeps some sites',
              len(on) > 0, f'{len(on)} via(s)')
        check('and fewer than without it (the panes cost sites)',
              len(on) < len(off), f'{len(on)} vs {len(off)}')
        check(f'every array via clears every paste opening by {SNPC} mm '
              f'(to half a grid step)', gap >= SNPC - GRID / 2 - 1e-9,
              f'closest {gap:.3f} mm')

        print('C. an exposed pad its own paste covers: no site, and the reason')
        on, log, _, _ = plane(d, 'c_on', board('full'), SNPC)
        check('no array via is laid in the paste', len(on) == 0, f'{len(on)} via(s)')
        line = next((l for l in log.splitlines() if 'did not fit' in l), '')
        check('the deferral says the paste openings cover the pad',
              'paste' in line, repr(line.strip()))

        print('D. the array still keeps off another same-net pad\'s keep-out')
        on, _, pcb, gnd = plane(d, 'd_on', board('none', neighbour=True), SNPC)
        nb = next(p for p in pcb.footprints['U1'].pads if p.pad_number == '3')
        from check_drc import point_to_pad_distance
        near = [round(point_to_pad_distance(v.x, v.y, nb) - v.size / 2, 3)
                for v in on]
        check('the array is laid (the path under test)', len(on) > 0,
              f'{len(on)} via(s)')
        check('the neighbour cost the array the sites in its keep-out',
              len(on) < n_a, f'{len(on)} vs {n_a}')
        check(f'no array via within {SNPC} mm of the neighbouring GND pad',
              all(g >= SNPC - GRID / 2 - 1e-9 for g in near), str(sorted(near)[:3]))
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        sys.exit(1)
    print('All checks passed.')


if __name__ == '__main__':
    main()
