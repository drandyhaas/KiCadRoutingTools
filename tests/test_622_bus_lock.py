#!/usr/bin/env python3
"""The lanes a bus step hands on are KiCad-locked (route_bus.lock_lanes): no later step strips them, whatever it names.
  python3 tests/test_622_bus_lock.py
The project's protection (#521) is lifted by a later step naming the net exactly -- a route.py --nets finishing a
pull's stub names it -- and a lock (#1261) by nothing. On a board of two lanes and one other net, a track, an arc and a
via each:
1. the named nets' tracks, arcs and vias get (locked yes) before their (layer / (layers, and nothing else does;
2. the parser reads them back locked, the other net's copper unlocked, every item still there;
3. an item already locked is left as it was (no second token), in either net dialect (ids or names);
4. a zone's polygon arc, which carries no net, is untouched.
"""
import contextlib
import io
import os
import sys
import tempfile
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import route_bus as rb  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
fails = []


def check(cond, msg):
    print(('PASS ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


HEAD = '''(kicad_pcb (version 20240108) (generator "pcbnew")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
  (net 1 "/LANE_A")
  (net 2 "/LANE_B")
  (net 3 "/OTHER")
'''


def item(net, i, locked=False, named=False):
    n = f'(net "{["", "/LANE_A", "/LANE_B", "/OTHER"][net]}")' if named else f'(net {net})'
    lk = ' (locked yes)' if locked else ''
    return (f'  (segment (start {i} 0) (end {i} 5) (width 0.15){lk} (layer "F.Cu") {n} (uuid "s{net}{i}"))\n'
            f'  (arc (start {i} 6) (mid {i + 0.5} 6.5) (end {i} 7) (width 0.15) (layer "B.Cu") {n} (uuid "a{net}{i}"))\n'
            f'  (via (at {i} 8) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") {n} (uuid "v{net}{i}"))\n')


ZONE = ('  (zone (net 0) (net_name "") (layer "F.Cu") (uuid "z") (polygon (pts (xy 0 20) (arc (start 0 20) (mid 1 21) '
        '(end 2 20)) (xy 2 22))))\n')


def board(named=False, pre_locked=False):
    return HEAD + item(1, 1, locked=pre_locked, named=named) + item(2, 2, named=named) + item(3, 3, named=named) \
        + ZONE + ')\n'


def parsed(txt):
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'b.kicad_pcb')
    with open(p, 'w') as f:
        f.write(txt)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(p)


for named in (False, True):
    dialect = 'names' if named else 'ids'
    txt = board(named=named)
    out, n = rb.lock_lanes(txt, {1, 2}, {'/LANE_A', '/LANE_B'})
    # 1. three items a lane (track, arc, via), each locked once, before its layer(s)
    check(n == 6, f'{dialect}: six items locked (got {n})')
    check(out.count('(locked yes)') == 6, f'{dialect}: six lock tokens in the text')
    check(out.count('(locked yes) (layer "F.Cu")') == 2 and out.count('(locked yes) (layers "F.Cu" "B.Cu")') == 2,
          f'{dialect}: the token stands before (layer / (layers')
    # 2. read back
    p = parsed(out)
    lk = {(s.net_id, bool(s.locked)) for s in p.segments if not getattr(s, 'graphic', False)}
    lv = {(v.net_id, bool(v.locked)) for v in p.vias}
    check(lk == {(1, True), (2, True), (3, False)}, f'{dialect}: tracks and arcs locked on the lanes alone {sorted(lk)}')
    check(lv == {(1, True), (2, True), (3, False)}, f'{dialect}: vias locked on the lanes alone {sorted(lv)}')
    p0 = parsed(txt)
    check((len(p.segments), len(p.vias)) == (len(p0.segments), len(p0.vias)), f'{dialect}: every item still read')
    # 4. the zone untouched
    check(ZONE.strip() in out, f'{dialect}: the zone polygon\'s arc untouched')

# 3. an item already locked keeps its one token
out, n = rb.lock_lanes(board(pre_locked=True), {1}, set())
check(n == 2 and out.count('(locked yes)') == 3, f'an already-locked track is left as it was (locked {n}, tokens '
                                                  f'{out.count("(locked yes)")})')

print(f'\n{"ALL PASS" if not fails else f"{len(fails)} FAILED"}')
sys.exit(1 if fails else 0)
