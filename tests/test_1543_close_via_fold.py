#!/usr/bin/env python3
"""#1543: two same-net vias this run laid under the drill hole-to-hole floor
are folded onto ONE via, with every layer's copper still joined to it.

The geometry is the issue's: two VCC3V3 through vias (0.3/0.15) 0.1 mm apart,
an F.Cu track into the first, a 0.1 mm In1.Cu hop between them, and B.Cu and
In2.Cu tracks out of the second. route.py shipped the pair (check_drc
VIA-DRILL-HOLE) and only warned (#1414).

Rows:
  1. The issue pair folds: one via left, no pair under the floor, the hop
     gone, every layer's track still ending on the kept via (directly or by
     a bridge from it), every pad connected, results == board.
  2. A bridge at the track's width that would graze a foreign pad is drawn
     inside the two via discs instead (the copper the dropped via had).
  3. A pair no bridge can join (vias apart, foreign copper between them on
     both layers) is left in place for the #1414 census.
  4. An input via is never the one dropped.
  5. The connectivity gate: a fold that strands a via-in-pad is refused and
     the other fold is taken.
  6. route.py's engine runs the fold before the #1414 census (the wall board
     of test_1414, whose pair cannot fold, still warns).

    python3 tests/test_1543_close_via_fold.py
"""
import contextlib
import io
import math
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from synth import make_pad, make_seg, make_via, make_pcb, make_net, RoutingBoard  # noqa: E402
from kicad_parser import BoardInfo                                   # noqa: E402
from routing_config import GridRouteConfig                           # noqa: E402
from check_connected import check_net_connectivity                   # noqa: E402
from check_weird import close_same_net_via_pairs                     # noqa: E402
import pcb_modification                                              # noqa: E402

FAILS = []
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


# Child for row 6: spy on the fold, then run route.py as its CLI runs.
SPY = r"""
import os, runpy, sys
root, src, out = sys.argv[1:4]
sys.path.insert(0, os.path.join(root, 'py_router'))
import pcb_modification
real = getattr(pcb_modification, 'merge_close_route_vias', None)
def spy(results, segs_by_net, vias_by_net, *a, **k):
    print('FOLD_CALL', sum(len(v) for v in vias_by_net.values()), flush=True)
    return real(results, segs_by_net, vias_by_net, *a, **k)
pcb_modification.merge_close_route_vias = spy
sys.argv = ['route.py', src, out, '--nets', 'SIG', '--layers', 'F.Cu', 'B.Cu',
            '--track-width', '0.2', '--clearance', '0.2', '--via-size', '0.6',
            '--via-drill', '0.3', '--hole-to-hole-clearance', '40.0',
            '--max-ripup', '0']
runpy.run_path(os.path.join(root, 'py_router', 'route.py'), run_name='__main__')
"""


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def cfg():
    return GridRouteConfig(track_width=0.1, clearance=0.09, via_size=0.45,
                           via_drill=0.3, grid_step=0.1,
                           hole_to_hole_clearance=0.25, layers=list(LAYERS))


def via(x, y):
    return make_via(x, y, size=0.3, drill=0.15, layers=('F.Cu', 'B.Cu'))


def seg(x1, y1, x2, y2, layer):
    return make_seg(x1, y1, x2, y2, layer=layer, width=0.3)


def board(segs, vias, pads, foreign_pads=()):
    by_net = {1: list(pads)}
    if foreign_pads:
        by_net[2] = list(foreign_pads)
    bi = BoardInfo(layers={0: 'F.Cu', 1: 'In1.Cu', 2: 'In2.Cu', 31: 'B.Cu'},
                   copper_layers=list(LAYERS))
    return make_pcb(nets={1: make_net(1, 'VCC3V3'), 2: make_net(2, 'OTHER')},
                    vias=vias, segments=segs, pads_by_net=by_net,
                    board_info=bi)


def issue_case(input_a=False, foreign_pads=()):
    """The issue's pair, with a pad at each track's far end."""
    a, b = via(170.1, 66.8), via(170.1, 66.9)
    f1 = seg(169.6, 66.3, 170.0, 66.7, 'F.Cu')
    f2 = seg(170.0, 66.7, 170.1, 66.8, 'F.Cu')
    hop = seg(170.1, 66.8, 170.1, 66.9, 'In1.Cu')
    bcu = seg(170.1, 66.9, 169.7, 67.3, 'B.Cu')
    in2a = seg(170.1, 66.9, 170.2, 67.0, 'In2.Cu')
    in2b = seg(170.2, 67.0, 170.9, 67.7, 'In2.Cu')
    pads = [make_pad(1, 169.6, 66.3, ref='P1', layers=('F.Cu',)),
            make_pad(1, 169.7, 67.3, ref='P2', layers=('B.Cu',)),
            make_pad(1, 170.9, 67.7, ref='P3', size_x=0.6, size_y=0.6,
                     shape='circle', layers=('*.Cu',), drill=0.3,
                     pad_type='thru_hole')]
    segs = [f1, f2, hop, bcu, in2a, in2b]
    vias = [a, b]
    pcb = board(segs, vias, pads, foreign_pads)
    new_vias = [b] if input_a else [a, b]
    new_segs = [s for s in segs if not (input_a and s in (f1, f2))]
    results = [{'new_vias': list(new_vias), 'new_segments': list(new_segs)}]
    return pcb, results, a, b, hop


def fold(pcb, results):
    segs_by_net = {1: [s for s in pcb.segments if s.net_id == 1]}
    vias_by_net = {1: [v for v in pcb.vias if v.net_id == 1]}
    merge = getattr(pcb_modification, 'merge_close_route_vias', None)
    if merge is None:       # no fold at all: every row below reads "unfolded"
        return 0, set(), segs_by_net, vias_by_net
    with contextlib.redirect_stdout(io.StringIO()):
        n, nets = merge(results, segs_by_net, vias_by_net, pcb, cfg())
    return n, nets, segs_by_net, vias_by_net


def connected(pcb):
    r = check_net_connectivity(1, [s for s in pcb.segments if s.net_id == 1],
                               [v for v in pcb.vias if v.net_id == 1],
                               pcb.pads_by_net[1], [], tolerance=0.02)
    return bool(r.get('connected')) and not r.get('disconnected_pads')


def ends_on(s, v):
    return any(abs(x - v.x) < 1e-6 and abs(y - v.y) < 1e-6
               for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y)))


def pairs(vias_by_net):
    return close_same_net_via_pairs(vias_by_net, lambda v: True, 0.25,
                                    lambda n: f'N{n}')


def main():
    # 1. The issue pair.
    pcb, results, a, b, hop = issue_case()
    check('1. before: the pair is under the floor and the net is connected',
          len(pairs({1: pcb.vias})) == 1 and connected(pcb))
    n, nets, sbn, vbn = fold(pcb, results)
    check('1. one via folded on VCC3V3', n == 1 and nets == {1}, f'n={n}')
    left = [v for v in pcb.vias if v.net_id == 1]
    check('1. one via left, no same-net pair under the floor',
          len(left) == 1 and not pairs({1: left}) and not pairs(vbn))
    check('1. the In1.Cu hop between the two vias is gone',
          all(s is not hop for s in pcb.segments))
    keep = left[0] if left else None
    for layer in ('F.Cu', 'B.Cu', 'In2.Cu'):
        check(f'1. {layer} copper still ends on the kept via',
              keep is not None and any(ends_on(s, keep) for s in pcb.segments
                                       if s.layer == layer))
    check('1. every pad connected after the fold', connected(pcb))
    wl_v = [v for r in results for v in r['new_vias']]
    wl_s = [s for r in results for s in r['new_segments']]
    check('1. results carry exactly the board copper (lockstep)',
          {id(v) for v in wl_v} == {id(v) for v in pcb.vias}
          and {id(s) for s in wl_s} == {id(s) for s in pcb.segments}
          and {id(s) for s in sbn[1]} == {id(s) for s in pcb.segments})

    # 2. A foreign pad beside the bridge's waist: the full-width bridge
    # would graze it, the bridge inside the two via discs does not.
    fp = make_pad(2, 170.437, 66.85, ref='X1', size_x=0.2, size_y=0.2,
                  layers=('F.Cu',))
    pcb, results, a, b, hop = issue_case(foreign_pads=[fp])
    n, nets, sbn, vbn = fold(pcb, results)
    bridges = [s for s in pcb.segments
               if s.layer == 'F.Cu' and ends_on(s, a) and ends_on(s, b)]
    waist = 2 * math.sqrt(0.15 ** 2 - 0.05 ** 2)
    check('2. folded with an F.Cu bridge between the two via centres',
          n == 1 and len(bridges) == 1, f'n={n} bridges={len(bridges)}')
    check('2. the bridge fits inside the two via discs (<= waist), not 0.3',
          len(bridges) == 1 and bridges[0].width <= waist + 1e-9,
          f'width={bridges[0].width if bridges else None} waist={waist:.4f}')
    check('2. every pad connected after the fold', connected(pcb))

    # 3. Apart vias, foreign copper between them on F.Cu and B.Cu: no bridge.
    a, b = via(10.0, 10.0), via(10.0, 10.35)
    segs = [seg(9.0, 9.0, 10.0, 10.0, 'F.Cu'), seg(10.0, 10.0, 10.0, 10.35, 'In1.Cu'),
            seg(10.0, 10.35, 11.0, 11.35, 'B.Cu')]
    walls = [make_seg(9.0, 10.175, 11.0, 10.175, layer=L, net_id=2, width=0.1)
             for L in ('F.Cu', 'B.Cu')]
    pads = [make_pad(1, 9.0, 9.0, ref='P1', layers=('F.Cu',)),
            make_pad(1, 11.0, 11.35, ref='P2', layers=('B.Cu',))]
    pcb = board(segs + walls, [a, b], pads)
    results = [{'new_vias': [a, b], 'new_segments': list(segs)}]
    before = (list(pcb.segments), list(pcb.vias))
    n, nets, sbn, vbn = fold(pcb, results)
    check('3. a pair no bridge can join is left in place',
          n == 0 and pcb.vias == before[1] and pcb.segments == before[0]
          and results[0]['new_vias'] == [a, b], f'n={n}')

    # 4. The input via survives; the new one is folded onto it.
    pcb, results, a, b, hop = issue_case(input_a=True)
    n, nets, sbn, vbn = fold(pcb, results)
    left = [v for v in pcb.vias if v.net_id == 1]
    check('4. the INPUT via is the one kept', n == 1 and left == [a],
          f'n={n} left={[(v.x, v.y) for v in left]}')
    check('4. B.Cu and In2.Cu are bridged to it, the hop dropped, connected',
          all(any(s.layer == L and ends_on(s, a) and ends_on(s, b)
                  for s in pcb.segments) for L in ('B.Cu', 'In2.Cu'))
          and all(s is not hop for s in pcb.segments) and connected(pcb))

    # 5. The gate: B sits in a B.Cu pad with no track. Dropping B needs no
    # bridge (the cheaper fold) but strands the pad; dropping A is taken.
    a, b = via(20.0, 20.0), via(20.0, 20.25)
    segs = [seg(19.0, 19.0, 20.0, 20.0, 'F.Cu'), seg(20.0, 20.0, 20.0, 20.25, 'In1.Cu')]
    pads = [make_pad(1, 19.0, 19.0, ref='P1', layers=('F.Cu',)),
            make_pad(1, 20.0, 20.4, ref='P2', size_x=0.4, size_y=0.4,
                     layers=('B.Cu',))]
    pcb = board(segs, [a, b], pads)
    results = [{'new_vias': [a, b], 'new_segments': list(segs)}]
    n, nets, sbn, vbn = fold(pcb, results)
    left = [v for v in pcb.vias if v.net_id == 1]
    check('5. the via in the pad is kept, the other folded onto it',
          n == 1 and left == [b], f'n={n} left={[(v.x, v.y) for v in left]}')
    check('5. every pad connected after the fold', connected(pcb))

    # 6. route.py runs the fold in its engine, before the #1414 census. In a
    # child process, so route.py's own exit echo stays out of this one's.
    bd = RoutingBoard(w=40, h=30)
    bd.pad('A', 10, 15, 'SIG')
    bd.pad('B', 30, 15, 'SIG')
    bd.seg(20, 0.5, 20, 29.5, 'WALL', w=1.0)
    tmp = tempfile.mkdtemp(prefix='t1543_')
    src = os.path.join(tmp, 'wall.kicad_pcb')
    out = os.path.join(tmp, 'out.kicad_pcb')
    open(src, 'w', encoding='utf-8').write(bd.text())
    r = subprocess.run([sys.executable, '-X', 'utf8', '-c', SPY, ROOT, src, out],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    text = r.stdout + r.stderr
    calls = [ln for ln in text.splitlines() if ln.startswith('FOLD_CALL ')]
    check("6. route.py called the fold with the run's vias",
          bool(calls) and int(calls[0].split()[1]) >= 2,
          f'calls={calls}' if calls else text[-400:])
    check('6. the unfoldable wall pair is still named by the #1414 census',
          'same-net via pair(s) closer than' in text)

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
