#!/usr/bin/env python3
"""#1798: the strict collapse keeps a via-centre stub when another track end
stays at that via's centre.

A fanned-out part's pad-to-via track ends at the centre of its via. A pair
that leaves that via on the same layer joins it through a stub of its own,
from the via's rim to its centre. The collapse refused to remove the LAST
track that ends at a via's centre on a layer; here the pad's track stays at
the centre, so the pair's stub was removed, the pair's track was left ending
on the rim, and the member measured a via radius shorter: a matched,
recorded pair failed its skew after a later `route.py` step.

The chain: an underpad QFN fanout, then route_diff matches and records the
pair CSI_CLK alone.

Rows:
  1. unit, on a three-segment net (pad track to the centre, a stub from the
     centre to the rim, a track carrying on from the stub): the model keeps
     the stub under keep_centre_stubs and removes it without; a duplicate of
     the stub is still removable; a redundant detour that leaves the centre
     through such a stub still goes, its outer run first and then the stub;
  2. the route_diff board carries the shape (a stub, a track carrying on
     from it that touches the via, another track end at that centre) on a
     pair graded within its recorded tolerance;
  3. check_weird does not list the stub removable, and the model keeps it on
     this net and removes it on a net no length grade reads;
  4. after `route.py --nets 'CSI_CLK*'` (nothing to route) the member's
     KiCad length is the input's and check_drc grades 0 nets out of range.

    python3 tests/test_1798_centre_stub_second_end.py
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb')


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def py(*argv):
    return run_check([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                     accept=True)


def graded(*argv):
    """A grader whose exit code carries findings: run it, refuse a crash."""
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError('BROKEN TEST: the grader crashed\n' + out[-1500:])
    return out


def model(segs, vias, pads, keep, layers=('F.Cu', 'B.Cu')):
    from pcb_modification import StrictRemovalModel
    m = StrictRemovalModel(7, segs, vias, pads, list(layers),
                           keep_centre_stubs=keep)
    if not m.valid:
        raise AssertionError('BROKEN TEST: the model does not grade this net')
    return m


def removable(segs, vias, pads, keep, layers=('F.Cu', 'B.Cu')):
    from pcb_modification import strict_removable_segments
    return strict_removable_segments(model(segs, vias, pads, keep, layers))


def unit_rows():
    from kicad_parser import Pad, Via, Segment

    def pad(ref, x, y):
        return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
                   local_x=0, local_y=0, size_x=0.6, size_y=0.6, shape='rect',
                   layers=['F.Cu'], net_id=7, net_name='/N7', rotation=0.0)

    def seg(x1, y1, x2, y2):
        return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=0.2,
                       layer='F.Cu', net_id=7)
    print("1. unit: the model")
    pads = [pad('U1', 0, 0), pad('U2', 10, 0.29)]
    via = Via(x=2.0, y=0.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=7)
    to_centre = seg(0, 0, 2, 0)             # the pad's track, ends at the centre
    stub = seg(2, 0, 2, 0.29)               # centre -> just inside the rim
    onward = seg(2, 0.29, 10, 0.29)         # carries on from the stub
    base = [to_centre, stub, onward]
    got = {k: removable(base, [via], pads, k) for k in (True, False)}
    check("the stub is kept under keep_centre_stubs, removable without",
          got == {True: set(), False: {1}}, str(got))
    twin = removable(base + [seg(2, 0, 2, 0.29)], [via], pads, True)
    check("with a second stub over the same span, either may go",
          twin == {1, 3}, str(twin))
    # Control: a redundant detour that leaves the centre through such a stub
    # and rejoins the far pad, beside the direct track. The rule does not
    # hold the detour: its outer run goes, and then the stub does, as the
    # collapse's next round finds it with nothing carrying on from it.
    from pcb_modification import strict_removable_segments
    direct = seg(2, 0, 10, 0.29)
    loop = [to_centre, direct, seg(2, 0, 2, 0.29), seg(2, 0.29, 6, 3),
            seg(6, 3, 10, 0.29)]
    m = model(loop, [via], pads, True)
    as_is = strict_removable_segments(m)
    then = m.trial({3, 4}, (), (2,))[0]
    check("a redundant detour from the centre still goes: its outer run, then "
          "the stub nothing carries on from", {3, 4} <= as_is and then,
          f"{as_is} then the stub: {then}")


def shape(path):
    """(net name, stub, carrying-on track, via) on `path`: an F.Cu segment
    with one end on a same-net via's centre whose far end is the end of
    another F.Cu segment touching that via, while a third F.Cu segment also
    ends at that centre. None when the board has no such shape."""
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(path)

    def ends(s):
        return ((s.start_x, s.start_y), (s.end_x, s.end_y))
    for v in p.vias:
        mine = [s for s in p.segments if s.net_id == v.net_id and s.layer == 'F.Cu']
        at = [s for s in mine
              if min(math.hypot(x - v.x, y - v.y) for x, y in ends(s)) < 1e-6]
        for s in at:
            far = max(ends(s), key=lambda e: math.hypot(e[0] - v.x, e[1] - v.y))
            if math.hypot(far[0] - v.x, far[1] - v.y) < 1e-6:
                continue
            on = [t for t in mine if t is not s and t not in at and any(
                math.hypot(x - far[0], y - far[1]) < 1e-6 for x, y in ends(t))]
            if on and len(at) >= 2:
                return p, p.nets[v.net_id].name, s, on[0], v
    return None


def net_len_mm(path, name):
    from kicad_parser import parse_kicad_pcb
    from net_queries import kicad_net_length_nm
    p = parse_kicad_pcb(path)
    i = next(k for k, n in p.nets.items() if n.name == name)
    return kicad_net_length_nm(p, [s for s in p.segments if s.net_id == i],
                               [v for v in p.vias if v.net_id == i],
                               p.nets[i].pads) / 1e6


def chain_rows(tmp):
    a, f, b, c = (os.path.join(tmp, f'{t}.kicad_pcb') for t in 'afbc')
    py(tool('copy_board.py'), BOARD, a)
    py(tool('qfn_fanout.py'), evidence(a, 'board copy'), '-o', f,
       '--component', 'U1', '--nets', 'CSI_*', '--escape-method', 'underpad',
       '--layer', 'In1.Cu', '--width', '0.127', '--via-size', '0.5',
       '--via-drill', '0.2', '--allow-via-in-pad')
    py(tool('route_diff.py'), evidence(f, 'fanout board'), b,
       '--nets', 'CSI_CLK*', '--layers', 'F.Cu', 'In1.Cu',
       '--track-width', '0.127', '--diff-pair-gap', '0.2', '--via-size', '0.6',
       '--diff-pair-intra-match')

    print("2. the matched board")
    out = graded(tool('check_drc.py'), evidence(b, 'route_diff board'), '-c', '0.1')
    if '1 group(s)/pair(s) graded, 0 net(s) out of range' not in out:
        print("  [BROKEN TEST] the pair is not graded within its recorded "
              "tolerance on the route_diff board:\n" + out[-800:])
        return False
    found = shape(b)
    if found is None:
        print("  [BROKEN TEST] no via on the route_diff board has a centre stub "
              "carrying a track on beside another track end at its centre: "
              "nothing here exercises the rule")
        return False
    pb, name, stub, onward, via = found
    print(f"  {name}: stub ({stub.start_x},{stub.start_y})->({stub.end_x},"
          f"{stub.end_y}) at the {via.size} mm via ({via.x},{via.y}); the track "
          f"carrying on ends ({onward.end_x},{onward.end_y})")

    print("3. check_weird and the model keep the stub")
    w = graded(tool('check_weird.py'), b)
    tag = f"({stub.start_x:.3f}, {stub.start_y:.3f})-({stub.end_x:.3f}, {stub.end_y:.3f})"
    check("check_weird does not list the stub removable", tag not in w, tag)
    from cleanup_pipeline import _skew_bound_net_ids
    nid = stub.net_id
    if nid not in _skew_bound_net_ids(pb):
        print(f"  [BROKEN TEST] no skew rule grades {name} on the route_diff "
              "board: the rule is not in force for it")
        return False
    segs = [s for s in pb.segments if s.net_id == nid]
    at = segs.index(stub)
    got = {k: at in removable(segs, [v for v in pb.vias if v.net_id == nid],
                              pb.pads_by_net.get(nid, []), k,
                              pb.board_info.copper_layers) for k in (True, False)}
    check("the model keeps it on this net and removes it on an ungraded one",
          got == {True: False, False: True}, str(got))

    print("4. the step with nothing to route")
    before = net_len_mm(b, name)
    py(tool('route.py'), b, c, '--nets', 'CSI_CLK*')
    after = net_len_mm(evidence(c, 'routed board'), name)
    check(f"{name}'s KiCad length is unchanged", abs(after - before) < 1e-6,
          f"{before:.4f} -> {after:.4f} mm")
    out = graded(tool('check_drc.py'), c, '-c', '0.1')
    check("check_drc grades the pair within its tolerance",
          '1 group(s)/pair(s) graded, 0 net(s) out of range' in out,
          next((ln.strip() for ln in out.splitlines() if 'skew: ' in ln), ''))
    return True


def main():
    unit_rows()
    tmp = tempfile.mkdtemp(prefix='t1798c_')
    try:
        if not chain_rows(tmp):
            return 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
