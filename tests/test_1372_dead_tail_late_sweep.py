#!/usr/bin/env python3
"""#1372: a dead tail on a net the run left finished is swept by the end-of-run
pass, not by a second identical run.

The in-run cleanup scopes an unfinished net out (#473: its fragments may be
the landing sites the next step welds to). A net the plane finalize or the
reconciliation finished afterwards therefore kept the tails the cleanup had
spared, until a second pass of the same command swept them as input copper:
rp2350's GND shipped a 0.85 mm In4.Cu tail KiCad reports as track_dangling.
`sweep_dangling_via_branches` (the #1166 late pass) now also removes dead
tails, under the same gate.

Rows, on `sweep_dangling_via_branches` and then through route.py:

  * a vertex-rooted tail on a finished net is removed; the trunk stays
  * a tail unwinds to the junction when it is several segments long
  * an UNFINISHED net (a pad not joined) keeps its tail
  * a locked tail, a tail on a net with a pour and a tail rooted on a pad
    are left alone
  * route.py ships the board without the tail, and check_weird reads clean

and the via-in-pad unblock shape (#535): a via that is the only thing joining
two F.Cu track ends 0.07 mm apart (their caps overlap, so the net grades
connected). The sweep's gate refused to remove it because dropping the via
graded the two ends as new dangling ends; it now removes the via and moves
one end onto the other (a weld), so the net stays joined and the via is gone.

    python3 tests/test_1372_dead_tail_late_sweep.py
"""
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from kicad_parser import Pad, Via, Segment, Net, PCBData, BoardInfo   # noqa: E402
from pcb_modification import sweep_dangling_via_branches              # noqa: E402
from synth import RoutingBoard                                        # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def pad(ref, x, y, layer='F.Cu'):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=1.0, size_y=1.0, shape='rect',
               layers=[layer], net_id=1, net_name='SIG', rotation=0.0)


def seg(x1, y1, x2, y2, layer='F.Cu', w=0.2, locked=False):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=1, locked=locked)


def board(segs, pads, zones=()):
    b = PCBData(footprints={}, nets={1: Net(1, 'SIG')}, segments=list(segs), vias=[],
                board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
                pads_by_net={1: list(pads)})
    b.zones = list(zones)
    return b


PADS = [pad('A', 10, 10), pad('B', 30, 10)]


def trunk():
    return [seg(10, 10, 20, 10), seg(20, 10, 30, 10)]


def main():
    # 1. The sweep.
    t = seg(20, 10, 20, 13)
    b = board(trunk() + [t], PADS)
    rs, rv = sweep_dangling_via_branches(b, {1})
    check('1. a vertex-rooted tail on a finished net is removed, the trunk stays',
          rs == [t] and len(b.segments) == 2, f'{len(rs)} removed, {len(b.segments)} left')

    t1, t2 = seg(20, 10, 20, 13), seg(20, 13, 23, 13)
    b = board(trunk() + [t1, t2], PADS)
    st = {}
    rs, _ = sweep_dangling_via_branches(b, {1}, stats=st)
    check('2. a two-segment tail unwinds to the junction', {id(x) for x in rs} == {id(t1), id(t2)}
          and st.get('tails') == 2, str(st))

    b = board([seg(10, 10, 20, 10)] + [seg(20, 10, 20, 13)], PADS)    # B never joined
    rs, _ = sweep_dangling_via_branches(b, {1})
    check('3. an unfinished net keeps its tail', not rs and len(b.segments) == 2)

    t = seg(20, 10, 20, 13, locked=True)
    b = board(trunk() + [t], PADS)
    rs, _ = sweep_dangling_via_branches(b, {1})
    check('4. a locked tail is left alone', not rs)

    from types import SimpleNamespace
    zone = SimpleNamespace(net_id=1, layer='B.Cu', polygon=[(0, 0), (40, 0), (40, 20), (0, 20)],
                           teardrop=False)
    b = board(trunk() + [seg(20, 10, 20, 13)], PADS, zones=[zone])
    rs, _ = sweep_dangling_via_branches(b, {1})
    check('5. a net with a pour is left alone', not rs)

    b = board(trunk() + [seg(30, 10, 33, 10)], PADS)        # a continuation past pad B's centre
    rs, _ = sweep_dangling_via_branches(b, {1})
    check('6. a stub that ends past a pad it is rooted on is not a tail the sweep takes',
          len(b.segments) >= 2)

    # 8. Two ends that met inside a dangling via's barrel are welded.
    via = Via(x=20.0, y=10.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    a, c = seg(10, 10, 19.965, 10), seg(20.035, 10, 30, 10)
    b = board([a, c], PADS)
    b.vias = [via]
    st = {}
    rs, rv = sweep_dangling_via_branches(b, {1}, stats=st)
    added = st.get('added') or []
    check('8. the via is removed and one end is welded onto the other',
          rv == [via] and len(added) == 1 and len(b.vias) == 0
          and st.get('welds') == 1, f'{len(rv)} via(s), {len(added)} weld(s), {st}')
    if added:
        w = added[0]
        pts = {(round(q.start_x, 3), round(q.start_y, 3), round(q.end_x, 3), round(q.end_y, 3))
               for q in b.segments}
        joined = any(abs(x.end_x - y.start_x) < 1e-6 and abs(x.end_y - y.start_y) < 1e-6
                     for x in b.segments for y in b.segments if x is not y)
        check('8. the two ends now coincide and the net is still two tracks',
              len(b.segments) == 2 and joined and w in b.segments, str(pts))
    a, c = seg(10, 10, 19.965, 10, locked=True), seg(20.035, 10, 30, 10)
    b = board([a, c], PADS)
    b.vias = [Via(x=20.0, y=10.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)]
    st = {}
    rs, rv = sweep_dangling_via_branches(b, {1}, stats=st)
    check('8. the unlocked track moves when the other is locked',
          len(rv) == 1 and st.get('welds') == 1 and a in b.segments, str(st))
    a, c = seg(10, 10, 19.965, 10, locked=True), seg(20.035, 10, 30, 10, locked=True)
    b = board([a, c], PADS)
    b.vias = [Via(x=20.0, y=10.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)]
    rs, rv = sweep_dangling_via_branches(b, {1})
    check('8. with both tracks locked nothing is welded or removed', not rv and not rs)
    a, c = seg(10, 10, 18.0, 10), seg(22.0, 10, 30, 10)        # ends 4 mm apart
    b = board([a, c], PADS)
    b.vias = [Via(x=20.0, y=10.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)]
    rs, rv = sweep_dangling_via_branches(b, {1})
    check('8. ends that did not meet inside the barrel are not welded', not rv)

    # 7. route.py on a board file.
    bd = RoutingBoard(w=40, h=30)
    bd.pad('A', 10, 15, 'SIG')
    bd.pad('B', 30, 15, 'SIG')
    bd.seg(10, 15, 20, 15, 'SIG', w=0.2)
    bd.seg(20, 15, 30, 15, 'SIG', w=0.2)
    bd.seg(20, 15, 20, 18, 'SIG', w=0.2)
    tmp = tempfile.mkdtemp(prefix='t1372_')
    pcb = os.path.join(tmp, 'b.kicad_pcb')
    out = os.path.join(tmp, 'o.kicad_pcb')
    open(pcb, 'w', encoding='utf-8').write(bd.text())
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', pcb, out,
                        '--nets', 'SIG', '--layers', 'F.Cu', 'B.Cu'], cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    log = r.stdout + r.stderr
    w = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_weird.py', out],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    check('7. route.py ships the board without the tail', os.path.isfile(out)
          and 'dangling-end' not in w.stdout and 'NO WEIRD THINGS' in w.stdout,
          (w.stdout + log)[-400:])
    inp = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_weird.py', pcb],
                         cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                         errors='replace')
    check('7. precondition: the input does carry the tail', 'dangling-end' in inp.stdout,
          inp.stdout[-300:])

    # 9. The same weld through route.py on a board file.
    bd = RoutingBoard(w=40, h=30)
    bd.pad('A', 10, 15, 'SIG')
    bd.pad('B', 30, 15, 'SIG')
    bd.seg(10, 15, 19.965, 15, 'SIG', w=0.2)
    bd.via(20, 15, 'SIG', size=0.6, drill=0.3)
    bd.seg(20.035, 15, 30, 15, 'SIG', w=0.2)
    pcb2 = os.path.join(tmp, 'v.kicad_pcb')
    out2 = os.path.join(tmp, 'vo.kicad_pcb')
    open(pcb2, 'w', encoding='utf-8').write(bd.text())
    inp2 = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_weird.py', pcb2],
                          cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                          errors='replace')
    r2 = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', pcb2, out2,
                         '--nets', 'SIG', '--layers', 'F.Cu', 'B.Cu'], cwd=ROOT,
                        capture_output=True, text=True, encoding='utf-8', errors='replace')
    w2 = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_weird.py', out2],
                        cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                        errors='replace')
    c2 = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_connected.py', out2],
                        cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                        errors='replace')
    check('9. precondition: the input carries the dangling via',
          'dangling-via: 1' in inp2.stdout, inp2.stdout[-300:])
    check('9. route.py ships the board without the via, welded, still connected',
          'NO WEIRD THINGS' in w2.stdout and 'ALL NETS FULLY CONNECTED' in c2.stdout
          and 'welded 1 pair' in (r2.stdout + r2.stderr),
          (w2.stdout + c2.stdout + r2.stdout)[-500:])

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
