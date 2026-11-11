"""#1817: how many of a part's signal nets can leave it AT ONCE.

`check_reachability --pad` answers one pad at a time, so N pads sharing one
lane each read PASSABLE, and the lane ledger divides a span by a pitch. The
board below fences a row of four pads (U1, nets S1..S4, each also on X1
outside the fence) with another part's F.Cu copper (F1), the east wall opened
just wide enough for one track. `escape_flow.part_flow` (and
`check_reachability --part`) must read it short; a wider opening, or a pocket
inside the fence where a via reaches a B.Cu channel out, must raise the flow;
a rule area banning vias over that pocket must lower it again.

The lattice over-states capacity by design (a one-track gap straddling two
half-lane cells reads two), so the assertions are bounds: the one-track
opening must read SHORT and at least its true capacity, never a constant.

    python3 tests/test_1817_escape_flow.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import escape, escape_flow, reachability       # noqa: E402
from run_utils import check, evidence                         # noqa: E402

FAILS = []
RULES = dict(track_width=0.2, clearance=0.2, via_diameter=0.6, edge_clearance=0.3)
N = 4


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def board(path, gap, pocket=False, ban=False):
    """U1's row at x 10..13, y 10; F1's F.Cu fence round it, its east wall
    (x 15) open `gap` mm at y 10; F1's B.Cu sheet under the fenced area,
    leaving (with `pocket`) a channel west from x 9.7 out under the west
    wall; `ban` a rule area banning vias over that pocket."""
    nets = ''.join(f'  (net {k} "S{k}")\n' for k in range(1, N + 1))
    fence = N + 1
    nets += f'  (net {fence} "FENCE")\n'
    upads = ''.join(f'    (pad "{k}" smd rect (at {k - 1} 0) (size 0.3 0.3)'
                    f' (layers "F.Cu") (net {k} "S{k}"))\n' for k in range(1, N + 1))
    xpads = ''.join(f'    (pad "{k}" smd rect (at 0 {k - 1}) (size 0.3 0.3)'
                    f' (layers "F.Cu") (net {k} "S{k}"))\n' for k in range(1, N + 1))
    xl, xr, yt, yb, t = 8.0, 15.0, 8.0, 12.0, 0.6
    walls = [((xl + xr) / 2, yt, xr - xl + t, t), ((xl + xr) / 2, yb, xr - xl + t, t),
             (xl, (yt + yb) / 2, t, yb - yt + t)]
    run = (10.0 - gap / 2) - (yt - t / 2)
    walls += [(xr, (yt - t / 2) + run / 2, t, run), (xr, (yb + t / 2) - run / 2, t, run)]
    x0, x1, y0, y1 = xl - t / 2, xr + t / 2, yt - t / 2, yb + t / 2
    if pocket:
        sheet = [((x0 + x1) / 2, (y0 + 9.2) / 2, x1 - x0, 9.2 - y0),
                 ((x0 + x1) / 2, (10.8 + y1) / 2, x1 - x0, y1 - 10.8),
                 ((9.7 + x1) / 2, 10.0, x1 - 9.7, 1.6)]
    else:
        sheet = [((x0 + x1) / 2, 10.0, x1 - x0, y1 - y0)]
    fpads = ''.join(f'    (pad "W{i}" smd rect (at {x} {y}) (size {w} {h})'
                    f' (layers "F.Cu") (net {fence} "FENCE"))\n'
                    for i, (x, y, w, h) in enumerate(walls))
    fpads += ''.join(f'    (pad "B{i}" smd rect (at {x} {y}) (size {w} {h})'
                     f' (layers "B.Cu") (net {fence} "FENCE"))\n'
                     for i, (x, y, w, h) in enumerate(sheet))
    zone = ('  (zone (net 0) (net_name "") (layers "F.Cu" "B.Cu") (hatch edge 0.5)\n'
            '    (keepout (tracks allowed) (vias not_allowed) (pads allowed)'
            ' (copperpour allowed) (footprints allowed))\n'
            '    (polygon (pts (xy 8.2 9.1) (xy 9.8 9.1) (xy 9.8 10.9) (xy 8.2 10.9))))\n'
            if ban else '')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n' + nets +
                 '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))'
                 ' (layer "Edge.Cuts"))\n'
                 '  (footprint "t:ROW" (layer "F.Cu") (at 10 10)\n'
                 '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
                 + upads + '  )\n'
                 '  (footprint "t:FAR" (layer "F.Cu") (at 18 8.5)\n'
                 '    (property "Reference" "X1" (at 0 0) (layer "F.SilkS"))\n'
                 + xpads + '  )\n'
                 '  (footprint "t:FENCE" (layer "F.Cu") (at 0 0)\n'
                 '    (property "Reference" "F1" (at 0 0) (layer "F.SilkS"))\n'
                 + fpads + '  )\n' + zone + ')\n')
    return path


def flow(path):
    return escape_flow.part_flow(parse_kicad_pcb(path), 'U1', path, **RULES)


def main(td):
    one = flow(board(os.path.join(td, 'one.kicad_pcb'), gap=0.7))
    # the opening's free width for a track centreline is 0.7 - 2 x (0.2 + 0.1)
    # = 0.1 mm: one track. The lattice may read it as two, never as four.
    ok('a one-track opening reads SHORT (deficit > 0 is a proof)',
       one.demand == N and 1 <= one.flow < N and one.deficit == N - one.flow,
       one.to_dict())
    ok('the cut names the fence', one.cut_refs[:1] == ('F1',), one.cut_refs)
    two = flow(board(os.path.join(td, 'two.kicad_pcb'), gap=1.1))
    ok('an opening one lane wider raises the flow (not a constant)',
       two.flow > one.flow, (one.flow, two.flow))
    pocket = flow(board(os.path.join(td, 'pocket.kicad_pcb'), gap=0.7, pocket=True))
    ok('a pocket where a via reaches a B.Cu channel out raises the flow',
       pocket.flow > one.flow, (one.flow, pocket.flow))
    banned = flow(board(os.path.join(td, 'ban.kicad_pcb'), gap=0.7, pocket=True,
                        ban=True))
    ok('a rule area banning vias over the pocket lowers it again (the via mask '
       'is read)', banned.flow == one.flow, (pocket.flow, banned.flow))

    # control: the per-pad tool still reads every pad of the short row PASSABLE
    path = os.path.join(td, 'one.kicad_pcb')
    pcb = parse_kicad_pcb(path)
    verdicts = []
    for p in pcb.footprints['U1'].pads:
        r = reachability.pad_reachability(
            pcb, (p.global_x, p.global_y), net_id=p.net_id,
            view=(6.0, 6.0, 20.0, 13.0), track_mm=0.2, via_mm=0.6,
            base_clearance=0.2, step=0.02, edge_clearance=0.3)
        verdicts.append(r.to_dict()['verdict'])
    ok('control: each pad alone reads PASSABLE', verdicts == ['PASSABLE'] * N,
       verdicts)

    # control: the lane ledger is untouched
    tig = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
    tp = parse_kicad_pcb(tig)
    with contextlib.redirect_stdout(io.StringIO()):
        led = escape.escape_ledger(tp, pcb_file=tig, fine_at_board_rules=True)
    f = next(f for p in led for f in p.faces if (f.ref, f.face) == ('U3', 'south'))
    ok('control: the ledger still reads U3 south demand 10 supply 2 deficit 8',
       (f.demand, f.supply, f.deficit, f.deficit_floor) == (10, 2, 8, 0),
       (f.demand, f.supply, f.deficit, f.deficit_floor))

    # the CLI: a positive deficit exits 1, an unknown ref 2 for its reason
    cli = os.path.join(ROOT, 'py_tools', 'check_reachability.py')
    js = os.path.join(td, 'part.json')
    check([sys.executable, '-X', 'utf8', cli, path, '--part', 'U1', '--track', '0.2',
           '--clearance', '0.2', '--via', '0.6', '--json-out', js],
          refuse='PROVEN SHORT', code=1)
    d = json.load(open(evidence(js, 'check_reachability --part JSON')))
    ok('the JSON carries demand, flow and deficit',
       d['demand'] == N and d['deficit'] == N - d['flow'] > 0 and d['cut_refs'][:1] == ['F1'],
       d)
    check([sys.executable, '-X', 'utf8', cli, path, '--part', 'NOSUCH'],
          refuse="no part 'NOSUCH' on this board", code=2)
    print('  PASS  --part NOSUCH exits 2 naming the ref')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as td:
        main(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
