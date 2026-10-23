#!/usr/bin/env python3
"""A via the tool drills is written as a THROUGH via: `(layers "F.Cu" "B.Cu")`
(found reviewing #1268).

The writer gives a via no blind/micro word, so KiCad loads it as a through via
whatever its `(layers ...)` pair says. The BGA under-pad engine wrote the pair
`[layers[0], layers[-1]]` -- the ends of the escape layer list -- which the
back-side flip frame turns into `(layers "B.Cu" "In2.Cu")` on a 4-layer board
(`(layers "B.Cu" "In6.Cu")` on zynq7020_som's 8): the same copper, but a pair
anything reading it as the span took for a blind via. The fix changes the pair
only: on this fixture the tracks, via positions and sizes and failed nets are
identical before and after, on both sides. The structural half holds every
via producer in py_router off the list-ends pattern.

    python3 tests/test_fanout_through_via_layers.py
"""
import ast
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, HERE)

from kicad_parser import BoardInfo, Footprint  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
from synth import make_pad, make_net, make_pcb  # noqa: E402

CU = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
N, PITCH, BALL = 6, 0.5, 0.3     # too tight to surface-escape: inner balls drill
FAILS = []


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  [{detail}]' if not ok else ''))
    if not ok:
        FAILS.append(label)


def fixture(side):
    """A 6x6 0.5 mm BGA, every ball netted to a pad 16 mm east, on F.Cu or
    (mirrored) B.Cu of a 4-layer board."""
    cx = cy = CY = 10.0
    face = 'F.Cu' if side == 'F' else 'B.Cu'
    pads, nets, byid = [], {}, {}
    k = 0
    for r in range(N):
        for c in range(N):
            k += 1
            lx, ly = (c - (N - 1) / 2) * PITCH, (r - (N - 1) / 2) * PITCH
            gx, gy = cx + lx, cy + ly
            if side == 'B':
                gy = 2 * CY - gy
            p = make_pad(k, gx, gy, ref='U1', num=f'{chr(65 + r)}{c + 1}', net_name=f'N{k}',
                         size_x=BALL, size_y=BALL, shape='circle', layers=(face,),
                         local_x=gx - cx, local_y=gy - cy)
            pads.append(p)
            nets[k] = make_net(k, f'N{k}', pads=[p])
            byid[k] = [p]
    fp = Footprint(reference='U1', footprint_name='test:BGA64', x=cx, y=cy, rotation=0.0,
                   layer=face, pads=pads)
    far = []
    for i, nid in enumerate(sorted(nets)):
        fy = 1.0 + 0.6 * i
        gy = fy if side == 'F' else 2 * CY - fy
        q = make_pad(nid, 26.0, gy, ref='J1', num=str(i + 1), net_name=f'N{nid}',
                     size_x=0.4, size_y=0.4, layers=(face,), local_x=0.0, local_y=gy - 10.0)
        far.append(q)
        nets[nid].pads.append(q)
        byid[nid].append(q)
    j1 = Footprint(reference='J1', footprint_name='test:J', x=26.0, y=10.0, rotation=0.0,
                   layer=face, pads=far)
    bi = BoardInfo(layers={0: 'F.Cu', 1: 'In1.Cu', 2: 'In2.Cu', 31: 'B.Cu'},
                   copper_layers=list(CU), board_bounds=(0.0, -12.0, 30.0, 32.0))
    pcb = make_pcb(nets=nets, footprints={'U1': fp, 'J1': j1}, pads_by_net=byid,
                   board_info=bi, zones=[])
    pcb._fanout_all_foreign_immovable = True
    return pcb


def fan(side):
    pcb = fixture(side)
    with contextlib.redirect_stdout(io.StringIO()):
        tracks, vias, _rm, failed = generate_bga_fanout(
            pcb.footprints['U1'], pcb, net_filter=[n.name for n in pcb.nets.values()],
            layers=list(CU), track_width=0.1, clearance=0.1, via_size=0.25,
            via_drill=0.15, exit_margin=0.5, escape_method='underpad', plane_drop='off')
    return tracks, vias


def test_pairs():
    for side in ('F', 'B'):
        tracks, vias = fan(side)
        check(f"{side}-side BGA: the under-pad engine drills vias (the fixture "
              f"is not vacuous)", len(vias) >= 10, len(vias))
        pairs = sorted({tuple(sorted(v['layers'])) for v in vias})
        check(f"{side}-side BGA: every via it drills is written F.Cu/B.Cu",
              pairs == [('B.Cu', 'F.Cu')], pairs)


def test_no_end_pair_producer():
    """Structural: no via dict or Via() in py_router builds its pair from the
    two ENDS of a layer list (`[layers[0], layers[-1]]`), which is not the
    stack's ends once a list is reordered."""
    def is_end_pair(node):
        if not isinstance(node, ast.List) or len(node.elts) != 2:
            return False
        idx = []
        for e in node.elts:
            if not (isinstance(e, ast.Subscript) and isinstance(e.slice, (ast.Constant, ast.UnaryOp))):
                return False
            sl = e.slice
            val = (sl.value if isinstance(sl, ast.Constant)
                   else -sl.operand.value if isinstance(sl.operand, ast.Constant) else None)
            idx.append(val)
        return idx == [0, -1]
    hits = []
    for dp, _, fs in os.walk(os.path.join(ROOT_DIR, 'py_router')):
        for f in fs:
            if not f.endswith('.py'):
                continue
            p = os.path.join(dp, f)
            tree = ast.parse(open(p, encoding='utf-8').read())
            for n in ast.walk(tree):
                vals = []
                if isinstance(n, ast.Dict):
                    vals = [v for k, v in zip(n.keys, n.values)
                            if isinstance(k, ast.Constant) and k.value == 'layers']
                elif isinstance(n, ast.Call):
                    vals = [k.value for k in n.keywords if k.arg == 'layers']
                    if getattr(n.func, 'id', getattr(n.func, 'attr', '')) == 'generate_via_sexpr' \
                            and len(n.args) >= 5:
                        vals.append(n.args[4])
                for v in vals:
                    for sub in ast.walk(v):
                        if is_end_pair(sub):
                            hits.append(f"{os.path.relpath(p, ROOT_DIR)}:{n.lineno}")
    check("no via in py_router takes its layer pair from a list's two ends",
          not hits, hits)


def main():
    test_pairs()
    test_no_end_pair_producer()
    if FAILS:
        print(f"\n{len(FAILS)} FAILED")
        return 1
    print("\nall passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
