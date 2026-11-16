#!/usr/bin/env python3
"""A route step routes a net to its net-tagged copper art island (#659).

KiCad conducts through net-tagged copper art and links every copper cluster,
so art that reaches none of its net's pads is an open net in KiCad's DRC.
A corpus board's one-pad net carried a test land drawn as two filled 1.5 mm
discs, one per side, 12.8 mm from its pad; the human joins them to the pad
with a track and a via, and the router -- which never targeted art -- left
the net open on every run. This pins, on a board shaped like it:

  1. check_connected.classify_unconnected_link places a link end KiCad
     anchors at a filled disc's CENTRE (out of reach of its outline) on the
     disc: 'graphic', not 'unknown';
  2. art_terminals.add_art_terminals gives each island (one per side) a
     terminal on its copper, once (idempotent), and remove_art_terminals
     leaves the board as it found it;
  3. route.py --nets '*' joins the pad and both discs (KiCad's links, by our
     grade: none), and the written board carries no terminal footprint;
  4. a run whose --nets leaves the net out adds no terminal for it.

    python3 tests/test_659_art_terminals.py
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


def _disc(x, y, r, net, layer):
    return ('\t(gr_circle\n\t\t(center %g %g)\n\t\t(end %g %g)\n\t\t(stroke\n\t\t\t(width 0)\n'
            '\t\t\t(type solid)\n\t\t)\n\t\t(fill yes)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t)\n'
            % (x, y, x + r, y, layer, net))


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(stroke\n\t\t\t(width 0.1)\n'
            '\t\t\t(type default)\n\t\t)\n\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def write_board(path):
    parts = [HEAD, _fp('J2', 10, 5, 'TP'),
             _disc(10, 15, 0.75, 'TP', 'F.Cu'), _disc(10, 15, 0.75, 'TP', 'B.Cu'),
             _fp('R1', 20, 5, 'OTHER'), _fp('R2', 20, 15, 'OTHER')]
    c = [(0, 0), (30, 0), (30, 20), (0, 20)]
    parts += [_edge(*c[i], *c[(i + 1) % 4]) for i in range(4)]
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts) + ')\n')
    return run_utils.evidence(path, 'art-island board')


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def route(board, out, nets):
    return subprocess.run([sys.executable, '-X', 'utf8',
                           os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                           '--nets', *nets, '--clearance', '0.2', '--track-width', '0.25'],
                          capture_output=True, text=True, cwd=ROOT)


def main():
    from check_connected import classify_unconnected_link, check_net_connectivity
    from art_terminals import add_art_terminals, remove_art_terminals
    with tempfile.TemporaryDirectory() as td:
        board = write_board(os.path.join(td, 'in.kicad_pcb'))
        pcb = parse(board)
        tp = next(k for k, n in pcb.nets.items() if n.name == 'TP')

        print('1. a link end at a filled disc centre:')
        with contextlib.redirect_stdout(io.StringIO()):
            cls = classify_unconnected_link(pcb, tp, (10.0, 15.0, 'F.Cu', 'track'),
                                            (10.0, 5.0, 'F.Cu', 'pad'), 'track', 'pad')
        ok('reads graphic (the disc), the pad live', cls == ('graphic', 'live'), f'{cls}')

        print('2. one terminal per island, once, and taken back:')
        before = {n: len(v) for n, v in pcb.pads_by_net.items()}
        with contextlib.redirect_stdout(io.StringIO()):
            added = add_art_terminals(pcb, ['*'])
            again = add_art_terminals(pcb, ['*'])
        ok('two terminals (one per side), on TP', len(added) == 2
           and all(r.startswith('art~TP~') for r in added), f'{added}')
        ok('a second call adds none', again == [])
        pads = [p for p in pcb.pads_by_net[tp] if p.component_ref.startswith('art~')]
        ok('each sits at the disc centre, on its own layer',
           sorted(p.layers[0] for p in pads) == ['B.Cu', 'F.Cu']
           and all(abs(p.global_x - 10) < 1e-6 and abs(p.global_y - 15) < 1e-6 for p in pads))
        remove_art_terminals(pcb)
        ok('removal restores the board', {n: len(v) for n, v in pcb.pads_by_net.items()} == before
           and not any(r.startswith('art~') for r in pcb.footprints))

        print('3. route.py joins the pad and both discs:')
        out = os.path.join(td, 'out.kicad_pcb')
        r = route(board, out, ['*'])
        ok('route.py ran', r.returncode == 0 and 'Traceback' not in r.stdout + r.stderr,
           (r.stdout + r.stderr)[-300:])
        ok('it gave TP two art terminals', 'art~TP~1' in r.stdout and 'art~TP~2' in r.stdout)
        if os.path.exists(out):
            got = parse(out)
            tp2 = next(k for k, n in got.nets.items() if n.name == 'TP')
            segs = [s for s in got.segments if s.net_id == tp2]
            vias = [v for v in got.vias if v.net_id == tp2]
            ok('a via joins the two discs', len(vias) >= 1, f'{len(vias)} via(s)')
            with contextlib.redirect_stdout(io.StringIO()):
                res = check_net_connectivity(tp2, segs, vias, got.pads_by_net[tp2], [],
                                             return_graph=True, pcb_data=got)
            # every art cluster joins the pad's: KiCad's links are gone
            from geometry_utils import UnionFind
            uf = UnionFind()
            for a, b in res['graph']['edges']:
                uf.union(a, b)
            pad_root = uf.find(next(iter(res['graph']['pad_index_repr'].values())))
            art_roots = {uf.find(2 * i) for i, s in enumerate(segs) if s.graphic}
            ok('both discs are in the pad\'s copper cluster', art_roots == {pad_root},
               f'{len(art_roots)} art cluster(s)')
            ok('no terminal footprint is written',
               not any(r.startswith('art~') for r in got.footprints))

        print('4. a run whose --nets leaves TP out adds no terminal:')
        r2 = route(board, os.path.join(td, 'out2.kicad_pcb'), ['OTHER'])
        ok('no art terminal announced', 'Art islands' not in r2.stdout, r2.stdout[-200:])
    print('PASS' if not failures else f'FAIL ({len(failures)}): {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
