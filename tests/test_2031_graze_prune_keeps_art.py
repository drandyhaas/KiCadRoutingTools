#!/usr/bin/env python3
"""A route step keeps the track that joins net-tagged copper art (#2031).

The board is the way an earlier route step leaves it: net TP has one pad and a
filled square of copper art (#337) that touches no pad, joined by one track
that ends on the art's corner. Net OTHER's track passes the art's side next to
that corner under the clearance. The graze prune took that art side off the
board it analyses (dropping one side of a closed ring leaves the net
connected), the ring became an open path, and the dead-end sweep then removed
TP's track: the art shipped unjoined, an unconnected item in KiCad's DRC.

  1. prune_grazing_segments leaves every art segment in place;
  2. route.py --nets '*' ships TP's track, and the art stays joined to J2.1.

    python3 tests/test_2031_graze_prune_keeps_art.py
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402

failures = []

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
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t)
'''

# The art: a 1.4 mm square whose corner (11.4, 14) is where TP's track lands.
ART = [(10.0, 14.0), (11.4, 14.0), (11.4, 15.4), (10.0, 15.4)]
# OTHER's run: its edge 0.19 mm off the art's outline edge (side x = 11.4,
# stroke 0.2), under the run's 0.2 mm clearance.
OTHER_X = 11.4 + 0.1 + 0.19 + 0.125


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def _fp(ref, x, y, net):
    return ('\t(footprint "Test:P"\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n\t\t\t(layer "F.SilkS")\n'
            '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n\t\t\t(layers "F.Cu" "F.Mask")\n'
            '\t\t\t(net "%s")\n\t\t)\n\t)\n' % (x, y, ref, net))


def _poly(pts, net, layer):
    xy = ' '.join('(xy %g %g)' % p for p in pts)
    return ('\t(gr_poly\n\t\t(pts %s)\n\t\t(stroke\n\t\t\t(width 0.2)\n\t\t\t(type solid)\n\t\t)\n'
            '\t\t(fill yes)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t)\n' % (xy, layer, net))


def _track(pts, net, w=0.25, layer='F.Cu'):
    return ''.join('\t(segment\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(width %g)\n'
                   '\t\t(layer "%s")\n\t\t(net "%s")\n\t)\n'
                   % (a[0], a[1], b[0], b[1], w, layer, net) for a, b in zip(pts, pts[1:]))


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(stroke\n\t\t\t(width 0.1)\n'
            '\t\t\t(type default)\n\t\t)\n\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def write_board(path):
    parts = [HEAD, _fp('J2', 11.4, 5, 'TP'), _poly(ART, 'TP', 'F.Cu'),
             _track([(11.4, 5), (11.4, 14.0)], 'TP'),
             _fp('R1', 18, 12, 'OTHER'), _fp('R2', 18, 18, 'OTHER'),
             _track([(18, 12), (OTHER_X, 14.2), (OTHER_X, 15.4), (18, 18)], 'OTHER'),
             _fp('R3', 25, 5, 'THIRD'), _fp('R4', 25, 18, 'THIRD')]
    c = [(0, 0), (30, 0), (30, 20), (0, 20)]
    parts += [_edge(*c[i], *c[(i + 1) % 4]) for i in range(4)]
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts) + ')\n')
    return run_utils.evidence(path, 'art board')


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def tp_state(pcb):
    """(TP's track count, whether every art cluster is in J2.1's cluster)."""
    from check_connected import check_net_connectivity
    from geometry_utils import UnionFind
    tp = next(k for k, n in pcb.nets.items() if n.name == 'TP')
    segs = [s for s in pcb.segments if s.net_id == tp]
    vias = [v for v in pcb.vias if v.net_id == tp]
    with contextlib.redirect_stdout(io.StringIO()):
        res = check_net_connectivity(tp, segs, vias, pcb.pads_by_net[tp], [],
                                     return_graph=True, pcb_data=pcb)
    uf = UnionFind()
    for a, b in res['graph']['edges']:
        uf.union(a, b)
    pad_root = uf.find(next(iter(res['graph']['pad_index_repr'].values())))
    art_roots = {uf.find(2 * i) for i, s in enumerate(segs) if s.graphic}
    return sum(not s.graphic for s in segs), art_roots == {pad_root}


def main():
    from pcb_modification import prune_grazing_segments
    with tempfile.TemporaryDirectory() as td:
        board = write_board(os.path.join(td, 'in.kicad_pcb'))
        pcb = parse(board)
        tp = next(k for k, n in pcb.nets.items() if n.name == 'TP')
        ok('the input joins the art to J2.1 with one track', tp_state(pcb) == (1, True),
           f'{tp_state(pcb)}')

        print('1. the graze prune leaves the art in place:')
        art = sorted(id(s) for s in pcb.segments if s.graphic and s.net_id == tp)
        with contextlib.redirect_stdout(io.StringIO()):
            prune_grazing_segments([], pcb, {tp}, clearance=0.2, check_foreign_segments=True)
        left = sorted(id(s) for s in pcb.segments if s.graphic and s.net_id == tp)
        ok('every art segment is still on the board', left == art,
           f'{len(art)} -> {len(left)}')

        print("2. route.py --nets '*' keeps TP's track to the art:")
        out = os.path.join(td, 'out.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                            '--nets', '*', '--clearance', '0.2', '--track-width', '0.25'],
                           capture_output=True, text=True, cwd=ROOT)
        ran = r.returncode == 0 and 'Traceback' not in r.stdout + r.stderr
        ok('route.py ran', ran, '' if ran else (r.stdout + r.stderr)[-300:])
        if os.path.exists(out):
            tracks, joined = tp_state(parse(run_utils.evidence(out, 'routed board')))
            ok("TP's track ships", tracks >= 1, f'{tracks} track(s)')
            ok('the art is joined to J2.1', joined)

    print(f"\n{'ALL PASS' if not failures else 'FAIL: ' + '; '.join(failures)}")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
