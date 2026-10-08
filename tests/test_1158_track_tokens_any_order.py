#!/usr/bin/env python3
"""#1158: a track, arc or via is read by its own tokens, wherever they sit.

extract_segments matched (segment ...) / (arc ...) with fixed-order patterns
that accepted (locked yes) only between width/layer or layer/net. KiCad's
reader takes the token anywhere, so a hand-stamped lock before (width) -- or
the bare `(segment locked (start ...` form -- made the track vanish from the
model while KiCad loaded it as copper (StickHub: 747 of 749 segments parsed,
a hand join graded broken). A lock after (net ...) was kept but read
locked=False, because the flag came from the match, which ended at the net.
Vias lost a block the same way for a token inside the geometry prefix.

Checks:
  1. segment / arc / via, numeric and named net, with the lock at every
     position KiCad accepts: parsed once, locked, every field intact.
  2. (locked no) is not a lock; an unlocked block stays unlocked.
  3. KiCad 9's mask-exposed track `(layers "F.Cu" "F.Mask")` is modelled on
     its copper layer, and remove_segments_from_content finds it.
  4. A track/via block that cannot be modelled is REPORTED on stderr with its
     line; a polygon's (arc ...) vertex is not a track and is not reported; a
     via with no (net ...) is modelled on net 0, as KiCad loads it.
  5. The canonical blocks KiCad writes parse exactly as before (field order
     and every value), on a tracked routed board.

    python3 tests/test_1158_track_tokens_any_order.py
"""
import contextlib
import io
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import kicad_parser as K                                    # noqa: E402
from kicad_writer import remove_segments_from_content       # noqa: E402

N2I = {'': 0, 'SIG': 7}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def wrap(body):
    return '(kicad_pcb\n (net 0 "")\n (net 7 "SIG")\n' + body + '\n)\n'


def parse(text):
    err = io.StringIO()
    with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
        segs = [s for s in K.extract_segments(text, N2I) if not s.graphic]
        vias = K.extract_vias(text, N2I)
    return segs, vias, err.getvalue()


def place(fields, lock, where):
    """Insert the lock token into a field list at a named position."""
    f = list(fields)
    if where == 'bare':
        return 'locked ' + ' '.join(f)
    idx = {'first': 0, 'end': len(f)}.get(where)
    if idx is None:
        idx = [i for i, x in enumerate(f) if x.startswith(where)][0]
    f.insert(idx, lock)
    return ' '.join(f)


NET = {'num': '(net 7)', 'named': '(net "SIG")'}
SEG = ['(start 1 2)', '(end 3 4)', '(width 0.2)', '(layer "F.Cu")', 'NET', '(uuid "u-seg")']
ARC = ['(start 0 0)', '(mid 1 1)', '(end 2 0)', '(width 0.25)', '(layer "B.Cu")', 'NET', '(uuid "u-arc")']
VIA = ['(at 5 6)', '(size 0.6)', '(drill 0.3)', '(layers "F.Cu" "B.Cu")', 'NET', '(uuid "u-via")']
# Positions KiCad accepts: before each field, first, after the last, bare.
POS = {'segment': ['first', '(start', '(end', '(width', '(layer', 'NET', '(uuid', 'end', 'bare'],
       'arc': ['first', '(mid', '(width', '(layer', 'NET', 'end', 'bare'],
       'via': ['first', '(at', '(size', '(drill', '(layers', 'NET', 'end', 'bare']}

print("1. the lock anywhere: parsed once, locked, fields intact")
for kind, fields in (('segment', SEG), ('arc', ARC), ('via', VIA)):
    for dialect, net in NET.items():
        bad = []
        for where in POS[kind]:
            body = place(fields, '(locked yes)', where).replace('NET', net)
            segs, vias, err = parse(wrap(f' ({kind} {body})'))
            if kind == 'via':
                ok = (len(vias) == 1 and vias[0].locked and vias[0].net_id == 7
                      and (vias[0].x, vias[0].y, vias[0].size, vias[0].drill) == (5, 6, 0.6, 0.3)
                      and vias[0].layers == ['F.Cu', 'B.Cu'] and vias[0].uuid == 'u-via')
            elif kind == 'segment':
                ok = (len(segs) == 1 and segs[0].locked and segs[0].net_id == 7
                      and (segs[0].start_x, segs[0].start_y, segs[0].end_x, segs[0].end_y)
                      == (1, 2, 3, 4) and segs[0].width == 0.2
                      and segs[0].layer == 'F.Cu' and segs[0].uuid == 'u-seg')
            else:
                ok = (len(segs) > 2 and all(s.locked and s.net_id == 7 and s.layer == 'B.Cu'
                                            and s.width == 0.25 and s.uuid == 'u-arc'
                                            for s in segs)
                      and (segs[0].start_x, segs[0].start_y) == (0, 0)
                      and (round(segs[-1].end_x, 9), round(segs[-1].end_y, 9)) == (2, 0))
            if not ok or err:
                bad.append(where)
        check(f"{kind} / {dialect} net, {len(POS[kind])} positions", not bad,
              f"failed at {bad}" if bad else '')

print("2. (locked no) and no token are unlocked")
for kind, fields in (('segment', SEG), ('via', VIA)):
    for lock in ('(locked no)', None):
        body = (place(fields, lock, 'first') if lock else ' '.join(fields)).replace('NET', NET['num'])
        segs, vias, _ = parse(wrap(f' ({kind} {body})'))
        items = vias if kind == 'via' else segs
        check(f"{kind} {lock or 'no token'}", len(items) == 1 and not items[0].locked)
segs, vias, _ = parse(wrap(' (segment ' + place(SEG, '(locked)', '(width').replace('NET', NET['num']) + ')'))
check("segment (locked) with no value is a lock", len(segs) == 1 and segs[0].locked)

print("3. mask-exposed track (layers \"F.Cu\" \"F.Mask\")")
masked = wrap(' (segment (start 1 2) (end 3 4) (width 0.32) (layers "F.Cu" "F.Mask") (net "SIG") (uuid "u-m"))')
segs, _, err = parse(masked)
check("modelled on F.Cu", len(segs) == 1 and segs[0].layer == 'F.Cu' and not err,
      f"{[(s.layer) for s in segs]} {err.strip()[:80]}")
out, n = remove_segments_from_content(masked, segs, {0: '', 7: 'SIG'})
check("the strip finds it", n == 1 and '(segment' not in out, f"removed {n}")

print("4. unreadable blocks are reported; polygon arc vertices are not tracks")
txt = wrap(' (segment (start 1 2) (end 3 4) (layer "F.Cu") (net 7))\n'
           ' (via (at 5 5) (size 0.6) (layers "F.Cu" "B.Cu") (net 7))\n'
           ' (gr_poly (pts (xy 0 0) (arc (start 0 0) (mid 1 1) (end 2 0))) (layer "F.Cu") (fill yes))')
segs, vias, err = parse(txt)
check("segment without width reported with its line",
      re.search(r'1 track block.*line 4 \(segment without width\)', err) is not None, err.strip()[:120])
check("via without drill reported with its line",
      re.search(r'1 via block.*line 5 \(via without drill\)', err) is not None)
check("the polygon arc vertex is not a track", 'arc' not in err)
segs, vias, err = parse(wrap(' (via (at 5 5) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (uuid "nn"))'))
check("a via with no (net) is a net-0 barrel, as KiCad loads it",
      [(v.uuid, v.net_id) for v in vias] == [('nn', 0)] and not err)

print("5. canonical blocks parse as before (a tracked routed board)")
board = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
content = open(board, encoding='utf-8').read()
with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()) as e:
    _nets, n2i = K.extract_nets(content)
    segs = [s for s in K.extract_segments(content, n2i) if not s.graphic]
canon = re.compile(
    r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+'
    r'\(width\s+([\d.-]+)\)\s+(?:\(locked\s+yes\)\s+)?\(layer\s+"([^"]+)"\)')
want = [(m.group(1), m.group(2), m.group(3), m.group(4), float(m.group(5)), m.group(6))
        for m in canon.finditer(content)]
got = [(s.start_x_str, s.start_y_str, s.end_x_str, s.end_y_str, s.width, s.layer) for s in segs]
check("every segment, in file order, with its source strings",
      len(want) > 100 and got == want and not e.getvalue(), f"{len(got)} vs {len(want)}")

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")
