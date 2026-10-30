#!/usr/bin/env python3
"""#1521: a part the clearance step could not move off a BGA's fanout copper
is held in a re-fan of that BGA.

The fanout lays vias and escapes on an unlocked 2-pad C/R/FB part, because
place_fanout_clearance moves such a part afterwards. Where it cannot, the
part ships shorted. autoplan then fans the BGA out again holding the part
(bga_fanout --immovable-refs), replays the later fanouts and runs the
clearance step again, keeping the board with fewer shorts, then fewer
dropped escapes.

1. Engine, on a synthetic 6x6 0.5 mm BGA with a back-side 2-pad part whose
   pads sit under two inner balls (their via sites):
   - unheld (the negative control): the under-pad fanout puts foreign copper
     IN CONTACT with the part's pads -- the fixture exercises the case;
   - held: no foreign via or track comes within the clearance of them, and
     every ball is either escaped or reported failed;
   - the held set is the call's own: a later call without it fans as before.
2. autoplan.held_parts: a short counts only when it is unaccepted, on a
   movable passive, and its copper is on a net of a fanned BGA's balls.
3. autoplan.refan with a scripted chain: no shorts left runs nothing (the
   control); shorts left re-fan the BGA from its own input with the part
   held, replay a later fanout at its accepted rung only, run the clearance
   step again, and keep the better board; a worse re-fan is not kept.

    python3 tests/test_1521_refan_held_parts.py
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, HERE)

import kicad_parser  # noqa: E402
from kicad_parser import BoardInfo, Footprint, Segment, Via  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
from check_drc import check_pad_segment_overlap, check_pad_via_overlap  # noqa: E402
import autoplan as AP  # noqa: E402
from synth import make_pad, make_net, make_pcb  # noqa: E402

RUN_ALL_TIMEOUT = 300
CU = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
N, PITCH, BALL = 6, 0.5, 0.3     # too tight to surface-escape: inner balls drill
CLEARANCE = 0.1
FAILS = []


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  [{detail}]' if not ok else ''))
    if not ok:
        FAILS.append(label)


def fixture():
    """A 6x6 0.5 mm BGA on F.Cu, every ball its own net to a pad 16 mm east,
    and C1, a 2-pad part on B.Cu whose pads sit exactly under two inner balls
    two pitches apart (C3 and C5, via sites of an under-pad escape), on two
    nets no ball carries."""
    cx = cy = 10.0
    pads, nets, byid = [], {}, {}
    k = 0
    for r in range(N):
        for c in range(N):
            k += 1
            lx, ly = (c - (N - 1) / 2) * PITCH, (r - (N - 1) / 2) * PITCH
            p = make_pad(k, cx + lx, cy + ly, ref='U1', num=f'{chr(65 + r)}{c + 1}',
                         net_name=f'N{k}', size_x=BALL, size_y=BALL, shape='circle',
                         layers=('F.Cu',), local_x=lx, local_y=ly)
            pads.append(p)
            nets[k] = make_net(k, f'N{k}', pads=[p])
            byid[k] = [p]
    u1 = Footprint(reference='U1', footprint_name='test:BGA36', x=cx, y=cy,
                   rotation=0.0, layer='F.Cu', pads=pads)
    far = []
    for i, nid in enumerate(sorted(nets)):
        q = make_pad(nid, 26.0, -6.0 + 0.4 * i, ref='J1', num=str(i + 1),
                     net_name=f'N{nid}', size_x=0.3, size_y=0.3, layers=('F.Cu',),
                     local_x=0.0, local_y=-6.0 + 0.4 * i - cy)
        far.append(q)
        nets[nid].pads.append(q)
        byid[nid].append(q)
    j1 = Footprint(reference='J1', footprint_name='test:J', x=26.0, y=cy,
                   rotation=0.0, layer='F.Cu', pads=far)
    # C1 on row C, its pads under balls C3 and C5
    y = cy + (2 - (N - 1) / 2) * PITCH
    c1p = []
    for num, nid, name, col in (('1', 101, 'P1V', 2), ('2', 102, 'GNDX', 4)):
        x = cx + (col - (N - 1) / 2) * PITCH
        p = make_pad(nid, x, y, ref='C1', num=num, net_name=name, size_x=0.3,
                     size_y=0.3, layers=('B.Cu',), local_x=x - cx - 0.25, local_y=0.0)
        c1p.append(p)
        nets[nid] = make_net(nid, name, pads=[p])
        byid[nid] = [p]
    c1 = Footprint(reference='C1', footprint_name='test:C0402', x=cx + 0.25, y=y,
                   rotation=0.0, layer='B.Cu', pads=c1p)
    bi = BoardInfo(layers={0: 'F.Cu', 1: 'In1.Cu', 2: 'In2.Cu', 31: 'B.Cu'},
                   copper_layers=list(CU), board_bounds=(0.0, -12.0, 30.0, 32.0))
    return make_pcb(nets=nets, footprints={'U1': u1, 'J1': j1, 'C1': c1},
                    pads_by_net=byid, board_info=bi, zones=[])


def fan(pcb, hold=None):
    with contextlib.redirect_stdout(io.StringIO()):
        tracks, vias, _rm, failed = generate_bga_fanout(
            pcb.footprints['U1'], pcb,
            net_filter=[f'N{k}' for k in range(1, N * N + 1)],
            layers=list(CU), track_width=0.1, clearance=CLEARANCE, via_size=0.25,
            via_drill=0.15, exit_margin=0.5, escape_method='underpad',
            plane_drop='off', immovable_refs=hold)
    return tracks, vias, failed


def on_part(pcb, tracks, vias, clearance):
    """Foreign vias and tracks within CLEARANCE of C1's pads (0: in contact)."""
    hits = []
    for p in pcb.footprints['C1'].pads:
        for v in vias:
            via = Via(x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                      layers=list(v['layers']), net_id=v['net_id'])
            if via.net_id != p.net_id and check_pad_via_overlap(p, via, clearance, CU, 0.0)[0]:
                hits.append(('via', p.pad_number, round(v['x'], 3), round(v['y'], 3)))
        for t in tracks:
            seg = Segment(start_x=t['start'][0], start_y=t['start'][1], end_x=t['end'][0],
                          end_y=t['end'][1], width=t['width'], layer=t['layer'],
                          net_id=t['net_id'])
            if seg.net_id != p.net_id and check_pad_segment_overlap(p, seg, clearance, CU, 0.0)[0]:
                hits.append(('track', p.pad_number, t['layer']))
    return hits


def test_engine():
    print("engine: --immovable-refs keeps the fanout off a movable passive")
    pcb = fixture()
    t0, v0, f0 = fan(pcb)
    touch = on_part(pcb, t0, v0, 0.0)
    check("unheld (negative control): the fanout puts foreign copper in contact "
          "with C1's pads", len(touch) > 0, touch)
    pcb = fixture()
    t1, v1, f1 = fan(pcb, hold=['C1'])
    near = on_part(pcb, t1, v1, CLEARANCE)
    check("held: no foreign via or track within the clearance of C1's pads",
          near == [], near)
    served = {v['net_id'] for v in v1} | {t['net_id'] for t in t1}
    names = {pcb.nets[n].name for n in served if n in pcb.nets}
    missing = [f'N{k}' for k in range(1, N * N + 1)
               if f'N{k}' not in names and f'N{k}' not in f1]
    check("held: every ball is escaped or reported failed", missing == [], missing)
    check("held: it drops at most the two balls whose via site C1 covers "
          "beyond what the unheld fan drops",
          len(f1) <= len(f0) + 2 and len(v1) >= 10, (len(f1), len(f0), len(v1)))
    t2, v2, f2 = fan(pcb)
    check("the held set is the call's own: a later call without it fans as the "
          "unheld one did",
          (len(t2), len(v2), sorted(f2)) == (len(t0), len(v0), sorted(f0)),
          ((len(t2), len(v2), len(f2)), (len(t0), len(v0), len(f0))))


def _item(pad_ref, net2, at=(10.0, 10.0), kind='pad-via', accepted=False):
    return {'type': kind, 'short': True, 'accepted': accepted, 'pad_ref': pad_ref,
            'net1': 'P1V', 'net2': net2, 'via_loc': list(at)}


@contextlib.contextmanager
def parsed_as(pcb):
    orig = kicad_parser.parse_kicad_pcb
    kicad_parser.parse_kicad_pcb = lambda path: pcb
    try:
        yield
    finally:
        kicad_parser.parse_kicad_pcb = orig


FANS = [{'ref': 'U1', 'tool': 'bga', 'input': 'u1_in',
         'ladder': [['{IN}', '--component', 'U1'],
                    ['{IN}', '--component', 'U1', '--escape-method', 'dogbone']],
         'best': {'escaped': 36, 'failed': 0, 'out': 'u1_out', 'rung': 0}},
        {'ref': 'U3', 'tool': 'qfn', 'input': 'u1_out',
         'ladder': [['{IN}', '--component', 'U3'],
                    ['{IN}', '--component', 'U3', '--escape-method', 'underpad']],
         'best': {'escaped': 10, 'failed': 0, 'out': 'u3_out', 'rung': 1}}]


def test_held_parts():
    print("autoplan.held_parts: which shorts name a part to hold")
    pcb = fixture()
    d = {'items': [_item('C1.1', 'N15'),                      # held
                   _item('C1.2', 'N16', accepted=True),       # accepted: not
                   _item('C1.2', 'NOT_A_BALL'),               # no BGA net: not
                   _item('J1.3', 'N15')]}                     # not a passive
    with parsed_as(pcb):
        got = AP.held_parts('board', d, FANS)
    check("an unaccepted short of a ball net on a movable passive holds it; "
          "an accepted one, a non-ball net and a non-passive do not",
          got == {'U1': ['C1']}, got)
    pcb.footprints['C1'].locked = True
    with parsed_as(pcb):
        got = AP.held_parts('board', d, FANS)
    check("a locked part is never held (the fanout already clears it)", got == {}, got)


class ScriptedChain:
    """Chain.run and Chain.check_drc scripted: each board's report is
    REPORTS[board]; a fanout run reports FAILED[tag] dropped escapes."""

    def __init__(self, cur, reports, failed=None):
        self.cur = cur
        self.reports = reports
        self.failed = failed or {}
        self.runs = []
        self.steps = []

    def run(self, kind, args, tag):
        out = f'{tag}.out'
        self.runs.append((kind, tag, args, self.cur))
        n = self.failed.get(tag, 0)
        return out, ({'escaped': 36 - n, 'failed': n} if kind in ('bga', 'qfn') else {}), {}

    def accept(self, out):
        if out:
            self.cur = out

    def check_drc(self, board):
        return self.reports.get(board, {'violations': 0, 'items': []})


def test_refan():
    print("autoplan.refan: re-fan, replay, clearance step again, keep the better")
    pcb = fixture()
    short = {'violations': 1, 'items': [_item('C1.1', 'N15')]}
    clean = {'violations': 0, 'items': []}

    ch = ScriptedChain('decap.out', {'decap.out': clean})
    with parsed_as(pcb):
        AP.refan(ch, FANS, ['{IN}', '{OUT}'])
    check("control: nothing shorted after the clearance step runs nothing",
          ch.runs == [] and ch.cur == 'decap.out', ch.runs)

    ch = ScriptedChain('decap.out', {'decap.out': short, 'decap_refan.out': clean},
                       failed={'fanout_U1_held': 2, 'fanout_U1_r1_held': 1})
    with parsed_as(pcb):
        AP.refan(ch, FANS, ['{IN}', '{OUT}'])
    tags = [r[1] for r in ch.runs]
    check("re-fan: U1's whole ladder held, U3 at its accepted rung, the "
          "clearance step again",
          tags == ['fanout_U1_held', 'fanout_U1_r1_held', 'fanout_U3_r1_refan',
                   'decap_refan'], tags)
    first = ch.runs[0]
    check("re-fan: U1 starts from the board it was first fanned on, holding C1",
          first[3] == 'u1_in' and first[2][-2:] == ['--immovable-refs', 'C1'],
          (first[3], first[2]))
    check("re-fan: U3 replays its accepted rung unheld",
          ch.runs[2][2] == FANS[1]['ladder'][1], ch.runs[2][2])
    check("re-fan: fewer shorts is kept, at the cost of a dropped escape",
          ch.cur == 'decap_refan.out' and ch.steps[-1]['kept']
          and ch.steps[-1]['after'] == {'shorts': 0, 'dropped': 1},
          (ch.cur, ch.steps[-1]))

    ch = ScriptedChain('decap.out', {'decap.out': short,
                                     'decap_refan.out': {'items': [_item('C1.1', 'N15')] * 2}})
    with parsed_as(pcb):
        AP.refan(ch, FANS, ['{IN}', '{OUT}'])
    check("a re-fan with more shorts is not kept",
          ch.cur == 'decap.out' and not ch.steps[-1]['kept'], (ch.cur, ch.steps[-1]))


def main():
    test_engine()
    test_held_parts()
    test_refan()
    if FAILS:
        print(f"\nFAIL: {len(FAILS)} check(s)")
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
