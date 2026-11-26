#!/usr/bin/env python3
"""#1971: a length-match group whose members are multipoint nets routed this
run names what it matched, and is graded on each net's total copper.

route.py routes a net of three or more pads as its longest MST edge first
(Phase 1), meanders that edge, and lays the remaining taps in Phase 3. The
group's `member_status` read `matched` with `length_mm` the edge's length,
named no pads, and said nothing of the taps; graded on the nets' total copper
(KiCad's length/skew DRC, `board_score --length-groups`' default, and the
measure a member routed by an earlier step is matched on) the same group was
23.9 mm out of a 0.5 mm tolerance.

The board is an in-repo one with no copper. The case: two nets of three pads
each, both named in one group; the control is two 2-pad nets of the same
board in one group.

Rows:
  1. the step exits 0;
  2. every multipoint member's record names its measure (`main_edge`) and
     the two pads its main edge joins, and the member line prints them;
  3. each member carries its total copper (`net_copper_mm`), which agrees
     with the grader's total-copper measure (`net_queries.net_match_length`)
     on the shipped board, and the group its spread;
  4. that spread is over the tolerance, so no multipoint member reads
     `matched` (`matched_main_edge`), and a line says the group is not
     matched on total copper;
  5. control: a group of two 2-pad nets carries `net_copper` records with no
     pads and no total-copper regrade;
  6. a main edge that ends on the free end of earlier copper (a stand-in,
     not a pad) is labelled by its position rather than crashing the step.

    python3 tests/test_1971_main_edge_disclosure.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
BOARD = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')
A, B = 'Net-(U1B-O)', 'Net-(U1D-O)'          # three pads each
EDGE = {A: {'U1.6', 'C2.1'}, B: {'U1.11', 'C4.1'}}
C, D = 'Net-(U1B-D)', 'Net-(U1D-D)'          # two pads each
TOL = 0.5


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def py(*argv):
    return run_check([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                     accept=True)


def route(inp, out, nets):
    r = py(tool('route.py'), inp, out, '--nets', *nets,
           '--length-match-group', *nets, '--length-match-tolerance', str(TOL))
    log = r.stdout + r.stderr
    line = next((l for l in log.splitlines() if l.startswith('JSON_SUMMARY:')), '')
    groups = json.loads(line.split(':', 1)[1]).get('length_match_groups') if line else []
    return r, log, (groups or [{}])[0]


def stub_end_row():
    from connectivity import _EndpointStub
    from kicad_parser import Pad
    from length_matching import main_edge_pads
    pad = Pad(component_ref='U1', pad_number='6', global_x=1.0, global_y=2.0,
              local_x=0.0, local_y=0.0, size_x=1.0, size_y=1.0, shape='rect',
              layers=['F.Cu'], net_id=1, net_name='/N')
    stub = _EndpointStub(3.25, 4.5, 'F.Cu', 1)
    res = {'is_multipoint': True, 'routed_pad_indices': [0, 1],
           'multipoint_pad_info': [(0, 0, 0, 1.0, 2.0, pad),
                                   (0, 0, 0, 3.25, 4.5, stub)]}
    try:
        got = main_edge_pads(res)
    except Exception as e:                      # the step used to die here
        got = repr(e)
    check("a stub end is labelled by position",
          got == ['U1.6', 'copper end (3.250, 4.500)'], str(got))


def main():
    stub_end_row()
    tmp = tempfile.mkdtemp(prefix='t1971_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        py(tool('copy_board.py'), BOARD, inp)
        out = os.path.join(tmp, 'mp.kicad_pcb')
        r, log, grp = route(evidence(inp, 'fixture copy'), out, [A, B])
        ms = grp.get('member_status') or {}
        if set(ms) != {A, B}:
            print(f"  [BROKEN TEST] the group did not form over both nets: {sorted(ms)}")
            return 1
        print("1. the step")
        check("route.py exits 0", r.returncode == 0)
        print("2. what was matched")
        for net in (A, B):
            rec = ms[net]
            check(f"{net} is measured on its main edge",
                  rec.get('measure') == 'main_edge', str(rec))
            check(f"{net} names the edge's two pads",
                  set(rec.get('pads') or ()) == EDGE[net], str(rec.get('pads')))
        a, b = sorted(EDGE[A])
        check("the member line prints the pads",
              any(A in l and 'between' in l and a in l and b in l
                  for l in log.splitlines()))
        print("3. each net's total copper")
        from kicad_parser import parse_kicad_pcb
        from net_queries import net_match_length
        pcb = parse_kicad_pcb(evidence(out, 'routed board'))
        nid = {n.name: i for i, n in pcb.nets.items()}
        graded = {net: net_match_length(pcb, nid[net], 'total') for net in (A, B)}
        for net in (A, B):
            got = ms[net].get('net_copper_mm')
            check(f"{net} net_copper_mm agrees with the grader",
                  got is not None and graded[net] is not None
                  and abs(got - graded[net]) < 0.05,
                  f"record {got} grader {graded[net]}")
        spread = grp.get('net_copper_spread_mm')
        want = abs(graded[A] - graded[B])
        check("the group carries the total-copper spread",
              spread is not None and abs(spread - want) < 0.1,
              f"record {spread} grader {want:.3f}")
        print("4. the verdict")
        check("the spread is over the tolerance (the case holds)", want > TOL,
              f"{want:.3f}")
        check("the group is not within tolerance on total copper",
              grp.get('net_copper_within_tolerance') is False,
              str(grp.get('net_copper_within_tolerance')))
        for net in (A, B):
            check(f"{net} does not read `matched`",
                  ms[net].get('status') == 'matched_main_edge', str(ms[net]))
        check("a line says the group is not matched on total copper",
              any('NOT matched on total copper' in l for l in log.splitlines()))
        print("5. control: two 2-pad nets")
        r2, log2, grp2 = route(inp, os.path.join(tmp, 'pp.kicad_pcb'), [C, D])
        ms2 = grp2.get('member_status') or {}
        if set(ms2) != {C, D}:
            print(f"  [BROKEN TEST] the control group did not form: {sorted(ms2)}")
            return 1
        check("control exits 0", r2.returncode == 0)
        check("control records are net_copper with no pads",
              all(m.get('measure') == 'net_copper' and 'pads' not in m
                  for m in ms2.values()), str(ms2))
        check("control is not regraded",
              'net_copper_spread_mm' not in grp2
              and not any('on total copper' in l for l in log2.splitlines()))
        check("control statuses are the matcher's",
              all(m.get('status') in ('matched', 'unmatched', 'unmeanderable')
                  for m in ms2.values()), str(ms2))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
