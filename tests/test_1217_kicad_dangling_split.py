#!/usr/bin/env python3
"""route.py no longer ships a joint KiCad's DRC reports as track_dangling
(#1217, check_weird's `kicad-dangling`).

Measured on a re-placed esp_prog (route_planes GND -> route_diff the USB pair
-> route.py all nets, --power-nets-widths 0.4): /+3.3V on F.Cu shipped

    B  (137.3586, 96.4)-(139.8, 96.4)      w0.325  the trunk's neck into U1.5
    C  (138.2741, 96.4)-(138.1, 96.5741)   w0.4    a T off B, then
    D  (138.1, 96.5741)-(138.1, 98.9)      w0.4    the vertical it feeds

and KiCad reported track_dangling at C's T end: B's copper reaches both ends
of C, and KiCad counts it only for the end nearer one of B's ENDPOINTS, which
is the far end. The octolinear smoother made C (a staircase leaving B at the
T became a diagonal + vertical); nothing removed it, rightly -- D stops
0.1741 mm off B's centreline, 11.6 um outside its 0.325 mm copper, so C is
the joint. _late_kicad_dangling_split1217 (pcb_modification.
split_kicad_dangling_hosts) now splits B at the T: both pieces end there and
count for it, and the copper is unchanged.

  1. the repro shape is flagged once, at the T end, and the strict collapse
     keeps C (it is load-bearing);
  2. the split: B becomes two collinear pieces covering it, check_weird is
     clean, connectivity is unchanged;
  3. two joints on one host: cut at the first alone, the second dangles, so
     the host is cut at both; and (3b) a host's dead stretch past its joint,
     which no removal could take while the host was whole, goes to the
     strict collapse that runs after the split;
  4. controls: a locked or read-only host is never split, and a stub whose
     own root is a dead end is not "fixed" by a split (the host stays whole);
  5. CLI: the late pass rewrites the written file; --keep-input-copper and
     an out-of-scope net leave it alone;
  6. GUI: this-run copper is replaced in its result list, input copper
     leaves through segments_to_remove and its pieces arrive as a result;
  7. route.py calls it before the strict collapse (which then grades each
     piece) and again after the #162 dedup.

    python3 tests/test_1217_kicad_dangling_split.py
"""
import copy
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import RoutingBoard                                  # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from improvement_gate import copper_signature                   # noqa: E402
from check_weird import check_weird, kicad_dangling_end         # noqa: E402
from check_connected import check_net_connectivity              # noqa: E402
from pcb_modification import (split_kicad_dangling_hosts,        # noqa: E402
                              collapse_strict_redundant, _pt_seg_dist)
from run_utils import evidence                                  # noqa: E402

FAILS = []
NET = '+3V3'
B = (137.3586, 96.4, 139.8, 96.4, 0.325)
C = (138.2741, 96.4, 138.1, 96.5741, 0.4)
D = (138.1, 96.5741, 138.1, 98.9, 0.4)


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def repro_board(tmp, tag, copper=True, locked_host=False):
    """The shipped /+3.3V copper around the joint, with a pad at each end."""
    rb = RoutingBoard(w=160, h=120)
    rb.pad('P1', 135.1, 96.4, NET)
    rb.pad('P2', 140.2, 96.4, NET, sx=0.8, sy=0.325)    # U1.5's place
    rb.pad('P3', 138.1, 98.9, NET)
    base = os.path.join(tmp, f'{tag}_base.kicad_pcb')
    with open(base, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    if copper:
        rb.seg(135.1, 96.4, 137.3586, 96.4, NET, w=0.4)
        rb.seg(*B[:4], NET, w=B[4], locked=locked_host)
        rb.seg(*C[:4], NET, w=C[4])
        rb.seg(*D[:4], NET, w=D[4])
    path = os.path.join(tmp, f'{tag}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    return base, path


def is_(s, spec):
    return (abs(s.start_x - spec[0]) < 1e-6 and abs(s.start_y - spec[1]) < 1e-6
            and abs(s.end_x - spec[2]) < 1e-6 and abs(s.end_y - spec[3]) < 1e-6)


def kd(pcb):
    return [f for f in check_weird(pcb, tolerance=0)[0]
            if f['category'] == 'kicad-dangling']


def connected(pcb):
    nid = next(i for i, n in pcb.nets.items() if n.name == NET)
    r = check_net_connectivity(nid, [s for s in pcb.segments if s.net_id == nid],
                               [], pcb.pads_by_net.get(nid, []), [])
    return bool(r.get('connected')), len(r.get('disconnected_pads') or [])


def sig_of(path):
    b = parse_kicad_pcb(evidence(path, 'the board'))
    return copper_signature(b.segments, b.vias,
                            {i: n.name for i, n in b.nets.items()}.get)


def shape(tmp):
    print('1. the repro shape: flagged once at the T end; C is the joint')
    _b, path = repro_board(tmp, 'shape')
    pcb = parse_kicad_pcb(evidence(path, 'the repro board'))
    f = kd(pcb)
    check('exactly one kicad-dangling, at C\'s T end (138.274, 96.400)',
          len(f) == 1 and abs(f[0]['x'] - 138.2741) < 1e-4
          and abs(f[0]['y'] - 96.4) < 1e-4, f"{[(x['x'], x['y']) for x in f]}")
    gap = _pt_seg_dist(D[0], D[1], *B[:4]) - B[4] / 2.0
    check('D stops outside B\'s copper, so C carries the joint',
          0.01 < gap < 0.012, f'{gap * 1000:.1f} um')
    n, _strip = collapse_strict_redundant([], pcb, None)
    check('the strict collapse removes nothing (C is load-bearing)', n == 0,
          f'removed {n}')


def split(tmp):
    print('2. the split clears it and moves no copper')
    _b, path = repro_board(tmp, 'split')
    pcb = parse_kicad_pcb(path)
    before = connected(pcb)
    host = next(s for s in pcb.segments if is_(s, B))
    st = {}
    pairs = split_kicad_dangling_hosts(pcb, None, stats=st)
    pieces = pairs[0][1] if len(pairs) == 1 else []
    check('one host split, and it is B', len(pairs) == 1 and pairs[0][0] is host
          and st == {'splits': 1, 'nets': 1, 'left': 0}, f'{st}')
    check('B becomes (137.3586->138.2741) + (138.2741->139.8)',
          len(pieces) == 2 and is_(pieces[0], (137.3586, 96.4, 138.2741, 96.4))
          and is_(pieces[1], (138.2741, 96.4, 139.8, 96.4)),
          f'{[(p.start_x, p.end_x) for p in pieces]}')
    check('the pieces keep B\'s width, layer and net, and B is off the board',
          all(p.width == host.width and p.layer == host.layer
              and p.net_id == host.net_id for p in pieces)
          and host not in pcb.segments and all(p in pcb.segments for p in pieces))
    check('C and D are untouched', sum(is_(s, C) or is_(s, D)
                                       for s in pcb.segments) == 2)
    w = check_weird(pcb, tolerance=0)[0]
    check('check_weird is clean after', not w,
          f"{[x['category'] for x in w]}")
    check('connectivity unchanged', connected(pcb) == before == (True, 0),
          f'{before} -> {connected(pcb)}')


def two_joints(tmp):
    print('3. two joints on one host: cut at both')
    rb = RoutingBoard(w=40, h=30)
    rb.pad('P1', 10, 10, 'V')
    rb.pad('P2', 20, 10, 'V')
    rb.pad('P3', 14.85, 13, 'V', sx=0.3, sy=0.3)
    rb.pad('P4', 15.45, 13, 'V', sx=0.3, sy=0.3)
    rb.seg(10, 10, 20, 10, 'V', w=0.4)
    rb.seg(15, 10, 14.85, 10.15, 'V', w=0.4)          # flagged at (15, 10)
    rb.seg(14.85, 10.15, 14.85, 13, 'V', w=0.4)
    rb.seg(15.6, 10, 15.45, 10.15, 'V', w=0.4)        # clean while H is whole
    rb.seg(15.45, 10.15, 15.45, 13, 'V', w=0.4)
    path = os.path.join(tmp, 'two.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    pcb = parse_kicad_pcb(path)
    f = kd(pcb)
    check('one joint flagged before', len(f) == 1 and abs(f[0]['x'] - 15) < 1e-6,
          f"{[(x['x'], x['y']) for x in f]}")
    # The witness: a cut at the first joint alone strands the second.
    trunk = next(s for s in pcb.segments if s.start_x == 10 and s.end_x == 20)
    a, b = copy.copy(trunk), copy.copy(trunk)
    a.end_x = b.start_x = 15.0
    second = next(s for s in pcb.segments if s.start_x == 15.6)
    rest = [s for s in pcb.segments if s is not trunk] + [a, b]
    check('witness: cut at (15, 10) alone, the second joint dangles',
          kicad_dangling_end(second, rest, [], pcb.pads_by_net.get(second.net_id, []),
                             [], ['F.Cu', 'B.Cu']) is not None)
    pairs = split_kicad_dangling_hosts(pcb, None)
    cuts = [(p.start_x, p.end_x) for _h, ps in pairs for p in ps]
    check('the trunk is cut at both joints', cuts == [(10, 15), (15, 15.6), (15.6, 20)],
          f'{cuts}')
    w = check_weird(pcb, tolerance=0)[0]
    check('check_weird is clean after', not w, f"{[x['category'] for x in w]}")


def exposed_tail(tmp):
    print('3b. a split exposes the host\'s dead stretch; the collapse after it '
          'takes it')
    # grp_ft A_s3's /OSCO (a routed corpus board): the 0.15 host runs on
    # 0.43 mm past the joint to nothing. Whole, no removal can take that
    # stretch (the host carries the joint); split, it is a separate segment.
    rb = RoutingBoard(w=160, h=120)
    rb.pad('U4', 115.758, 99.952, '/OSCO', sx=1.55, sy=0.3)
    rb.pad('Y1', 113.2, 100.8, '/OSCO', sx=0.4, sy=0.4)
    rb.seg(115.758, 99.952, 114.683, 99.952, '/OSCO', w=0.15)
    rb.seg(114.683, 99.952, 113.4, 99.0, '/OSCO', w=0.15)       # the host
    rb.seg(113.742133, 99.253867, 113.7, 99.3, '/OSCO', w=0.25)  # the joint
    rb.seg(113.7, 99.3, 113.6, 99.3, '/OSCO', w=0.25)
    rb.seg(113.6, 99.3, 113.2, 99.7, '/OSCO', w=0.25)
    rb.seg(113.2, 99.7, 113.2, 100.8, '/OSCO', w=0.25)
    path = os.path.join(tmp, 'osco.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    pcb = parse_kicad_pcb(path)
    check('flagged once before', [f['category'] for f in
                                  check_weird(pcb, tolerance=0)[0]] == ['kicad-dangling'])
    pcb = parse_kicad_pcb(path)
    collapse_strict_redundant([], pcb, None)
    split_kicad_dangling_hosts(pcb, None)
    w = [f['category'] for f in check_weird(pcb, tolerance=0)[0]]
    check('witness: collapse THEN split leaves the exposed stretch removable',
          w == ['removable-segment'], f'{w}')
    pcb = parse_kicad_pcb(path)
    split_kicad_dangling_hosts(pcb, None)
    n, _strip = collapse_strict_redundant([], pcb, None)
    split_kicad_dangling_hosts(pcb, None)
    w = check_weird(pcb, tolerance=0)[0]
    gone = not any(abs(s.end_x - 113.4) < 1e-6 or abs(s.start_x - 113.4) < 1e-6
                   for s in pcb.segments)
    check('split, collapse, split (route.py\'s order): the stretch goes, clean',
          n == 1 and gone and not w, f"removed {n}, {[x['category'] for x in w]}")


def controls(tmp):
    print('4. controls')
    _b, path = repro_board(tmp, 'lock', locked_host=True)
    pcb = parse_kicad_pcb(path)
    st = {}
    pairs = split_kicad_dangling_hosts(pcb, None, stats=st)
    check('a locked host is never split (the finding stays, counted as left)',
          not pairs and len(kd(pcb)) == 1 and st.get('left') == 1, f'{st}')
    _b, path = repro_board(tmp, 'ro')
    pcb = parse_kicad_pcb(path)
    host = next(s for s in pcb.segments if is_(s, B))
    pairs = split_kicad_dangling_hosts(pcb, None, readonly_ids={id(host)})
    check('a read-only host is never split', not pairs and host in pcb.segments)
    # check_weird row 23's stub: its root (0.03, 0.02) is a dead end inside
    # the track, so an endpoint at its T end cannot make it whole -- KiCad
    # would still call it dangling, so the track must stay as it was.
    rb = RoutingBoard(w=40, h=30)
    rb.pad('P1', 8, 10, 'S')
    rb.pad('P2', 12, 10, 'S')
    rb.seg(8, 10, 12, 10, 'S', w=0.25)
    rb.seg(10.03, 10.02, 10, 10, 'S', w=0.25)
    path = os.path.join(tmp, 'deadroot.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    pcb = parse_kicad_pcb(path)
    st = {}
    pairs = split_kicad_dangling_hosts(pcb, None, stats=st)
    check('a stub with a dead-end root: the trial split is put back',
          not pairs and len(pcb.segments) == 2 and len(kd(pcb)) == 1
          and st.get('left') == 1, f'{st}')


def cli(tmp):
    print('5. CLI: the late pass rewrites the written file')
    from route import _late_kicad_dangling_split1217
    base, path = repro_board(tmp, 'cli')
    n = _late_kicad_dangling_split1217(None, path, False, None, None,
                                       sig_of(base), {NET}, False)
    pcb = parse_kicad_pcb(evidence(path, 'the split board'))
    check('one track split in the file, one more segment, check_weird clean',
          n == 1 and len(pcb.segments) == 5 and not check_weird(pcb, tolerance=0)[0],
          f'split {n}, {len(pcb.segments)} segments')
    check('the pieces are in the file, B is not',
          not any(is_(s, B) for s in pcb.segments)
          and any(is_(s, (137.3586, 96.4, 138.2741, 96.4)) for s in pcb.segments)
          and any(is_(s, (138.2741, 96.4, 139.8, 96.4)) for s in pcb.segments))
    _b, path = repro_board(tmp, 'keep')
    txt = open(path, encoding='utf-8').read()
    n = _late_kicad_dangling_split1217(None, path, False, None, None,
                                       sig_of(path), {NET}, True)
    check('--keep-input-copper: input copper is read-only, the file unchanged',
          n == 0 and open(path, encoding='utf-8').read() == txt, f'split {n}')
    base, path = repro_board(tmp, 'scope')
    txt = open(path, encoding='utf-8').read()
    n = _late_kicad_dangling_split1217(None, path, False, None, None,
                                       sig_of(base), {'GND'}, False)
    check('a net outside the run\'s scope is left alone',
          n == 0 and open(path, encoding='utf-8').read() == txt, f'split {n}')


def gui(tmp):
    print('6. GUI: the write model')
    from route import _late_kicad_dangling_split1217
    base, path = repro_board(tmp, 'gui')
    pcb = parse_kicad_pcb(path)
    nid = next(i for i, n in pcb.nets.items() if n.name == NET)
    routed = list(pcb.segments)
    host = next(s for s in routed if is_(s, B))
    rd = {'results': [{'net_id': nid, 'new_segments': list(routed),
                       'new_vias': []}]}

    def model_this_run(r):
        return {nid: [s for x in r['results'] for s in x['new_segments']]}, {}
    n = _late_kicad_dangling_split1217(pcb, None, True, rd, model_this_run,
                                       sig_of(base), {NET}, False)
    segs = rd['results'][0]['new_segments']
    check('this-run copper: B replaced by its pieces in its own result list',
          n == 1 and host not in segs and len(segs) == 5
          and not rd.get('segments_to_remove') and len(rd['results']) == 1,
          f'split {n}, {len(segs)} in the result')
    check('...and in pcb_data', host not in pcb.segments and len(pcb.segments) == 5)

    pcb = parse_kicad_pcb(path)
    inputs = list(pcb.segments)
    host = next(s for s in inputs if is_(s, B))
    rd = {'results': []}

    def model_input(r):
        gone = {id(x) for x in r.get('segments_to_remove') or []}
        segs = [s for s in inputs if id(s) not in gone]
        segs += [s for x in r['results'] for s in x['new_segments']]
        return {nid: segs}, {}
    n = _late_kicad_dangling_split1217(pcb, None, True, rd, model_input,
                                       sig_of(path), {NET}, False)
    added = [s for x in rd['results'] for s in x['new_segments']]
    check('input copper: B leaves through segments_to_remove, its pieces '
          'arrive as a kicad_dangling_split result',
          n == 1 and rd.get('segments_to_remove') == [host]
          and [x.get('cleanup') for x in rd['results']] == ['kicad_dangling_split']
          and len(added) == 2 and host not in pcb.segments
          and all(p in pcb.segments for p in added),
          f'split {n}, removed {len(rd.get("segments_to_remove") or [])}, '
          f'added {len(added)}')


def order():
    print('7. route.py runs it before the collapse and again after the dedup')
    src = open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8').read()
    body = src[src.index('def batch_route('):]
    i_first = body.find('_late_kicad_dangling_split1217(')
    i_coll = body.find('_late_strict_collapse1063(')
    i_dedup = body.find('_late_stacked_dedup162(')
    i_again = body.find('_late_kicad_dangling_split1217(', i_first + 1)
    # The orphan sweep is also called from the early "nothing to route"
    # return, so the one that matters is the LAST call.
    i_orphan = body.rfind('_late_orphan_sweep659(')
    check('split -> strict collapse -> #162 dedup -> split -> last orphan sweep',
          0 <= i_first < i_coll < i_dedup < i_again < i_orphan,
          f'{i_first} {i_coll} {i_dedup} {i_again} {i_orphan}')


def main():
    with tempfile.TemporaryDirectory() as tmp:
        shape(tmp)
        split(tmp)
        two_joints(tmp)
        exposed_tail(tmp)
        controls(tmp)
        cli(tmp)
        gui(tmp)
    order()
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
