#!/usr/bin/env python3
"""#1851: a net a `.kicad_dru` length rule holds to a window is read-only to
the graze passes, and a late pass that moves it out of its window says so.

A skew-graded net had both protections (#1759, #1798). A net held to a
length window had neither: the post-route cleanup's length-changing passes
could re-bend its input copper, and when an end-of-run pass moved it out of
its window the step printed nothing and reported nothing. check_drc and
KiCad still graded the shipped board.

The chain is tests/test_1798_late_collapse_skew.py's: an underpad QFN
fanout, then route_diff. A redundant detour is added beside one segment of
CSI_D0_N (that test's `add_detour`), the pair record is taken off the board
and a length rule binds CSI_D0_N with a window 0.1 mm either side of its
length WITH the detour. The all-nets `route.py --nets '*'` step's end-of-run
collapse removes the detour, which shortens the net by more than 0.1 mm.

Rows:
  1. the input grades the net inside its window;
  2. the net is in the graze passes' read-only set, with the skew-graded
     nets, and the comparison (`length_moved_out_of_window`) reports a net
     moved from inside to outside, one already out and now further, and
     neither a net that stayed inside nor one that moved back in;
  3. CLI: the step names the net in a WARNING and in --json-out's
     `length_moved_by_cleanup`, with the lengths either side;
  4. the row is true: check_drc grades the shipped board's net out of range;
  5. the GUI leg (batch_route in memory) carries the same row.

    python3 tests/test_1851_length_window_cleanup.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence, tool                             # noqa: E402
import test_1798_late_collapse_skew as chain                     # noqa: E402

FAILS = []
NET = 'CSI_D0_N'


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def main():
    tmp = tempfile.mkdtemp(prefix='t1851_')
    try:
        a, f, b, e, le, lec = (os.path.join(tmp, f'{t}.kicad_pcb')
                               for t in ('a', 'f', 'b', 'e', 'le', 'lec'))
        chain.py(tool('copy_board.py'), chain.BOARD, a)
        chain.py(tool('qfn_fanout.py'), evidence(a, 'board copy'), '-o', f,
                 '--component', 'U1', '--nets', 'CSI_*', '--escape-method', 'underpad',
                 '--layer', 'In1.Cu', '--width', '0.127', '--via-size', '0.5',
                 '--via-drill', '0.2', '--allow-via-in-pad')
        chain.py(tool('route_diff.py'), evidence(f, 'fanout board'), b,
                 '--nets', 'CSI_D0*', 'CSI_CLK*', '--layers', 'F.Cu', 'In1.Cu',
                 '--track-width', '0.127', '--diff-pair-gap', '0.2', '--via-size', '0.6',
                 '--diff-pair-intra-match')
        chain.add_detour(evidence(b, 'route_diff board'), e)
        with_detour = chain.length_rule_board(evidence(e, 'detour board'), le, NET, NET)

        print("1. the input")
        out = chain.graded(tool('check_drc.py'), evidence(le, 'length-rule board'),
                           '-c', '0.1')
        if '1 net(s) graded, 0 out of range' not in out:
            print("  [BROKEN TEST] the length rule does not grade its net inside "
                  "the window on the input:\n" + out[-800:])
            return 1
        print(f"  {NET}: {with_detour:.4f} mm, window +-0.1 mm")

        print("2. the read-only set and the comparison")
        from kicad_parser import parse_kicad_pcb
        from cleanup_pipeline import (_length_window_net_ids, _skew_bound_net_ids,
                                      _centre_stub_net_ids)
        from check_drc import length_moved_out_of_window as moved
        pl = parse_kicad_pcb(le)
        nid = next(k for k, n in pl.nets.items() if n.name == NET)
        check("no skew rule grades the board's nets", not _skew_bound_net_ids(pl))
        check("the net is a length-window net, and keeps its centre stubs too",
              _length_window_net_ids(pl) == {nid} and nid in _centre_stub_net_ids(pl))

        def row(length, lo=10.0, hi=11.0):
            ex = max(lo - length, length - hi, 0.0)
            return {'net': 'N', 'rule': 'r', 'unit': 'mm', 'length': length,
                    'min': lo, 'max': hi, 'excess': ex}
        got = moved({'N': row(10.5)}, {'N': row(9.8)})
        check("inside -> outside is a row, was_within",
              len(got) == 1 and got[0]['was_within'] and got[0]['length_after'] == 9.8)
        got = moved({'N': row(9.9)}, {'N': row(9.5)})
        check("outside -> further out is a row, not was_within",
              len(got) == 1 and not got[0]['was_within'])
        check("inside -> inside, outside -> inside and an unmeasured net are not",
              moved({'N': row(10.5)}, {'N': row(10.2)}) == []
              and moved({'N': row(9.5)}, {'N': row(10.2)}) == []
              and moved({'N': row(10.5)}, {'N': None}) == [])

        print("3. the all-nets step (CLI)")
        js = os.path.join(tmp, 'lec.json')
        r = chain.py(tool('route.py'), le, lec, '--nets', '*', '--json-out', js)
        log = r.stdout + r.stderr
        if 'Strict collapse (#1063, end of run)' not in log:
            print("  [BROKEN TEST] the end-of-run collapse removed nothing: the "
                  "detour this row needs removed is not\n" + log[-800:])
            return 1
        with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
            doc = json.load(fh)
        rows = [x for x in doc.get('length_moved_by_cleanup') or [] if x['net'] == NET]
        check("--json-out names the net", len(rows) == 1 and rows[0]['was_within']
              and abs(rows[0]['length_before'] - with_detour) < 1e-3
              and rows[0]['length_after'] < rows[0]['min'],
              str(doc.get('length_moved_by_cleanup')))
        check("the WARNING names it", re.search(
            r"WARNING: the end-of-run cleanup moved " + NET
            + r" out of its length rule 'stub len'", log) is not None,
            ' | '.join(l.strip() for l in log.splitlines() if 'length rule' in l)[-300:])

        print("4. the row is true")
        out = chain.graded(tool('check_drc.py'), evidence(lec, 'shipped board'), '-c', '0.1')
        check("check_drc grades the shipped net out of range",
              '1 net(s) graded, 1 out of range' in out,
              (re.search(r'Length rules[^\n]*', out) or [''])[0][:160])

        print("5. the GUI leg")
        from route import batch_route
        pcb = parse_kicad_pcb(le)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _ok, _fail, _t, rd = batch_route(
                le, '', [n.name for n in pcb.nets.values() if n.name],
                pcb_data=pcb, return_results=True)
        rows = [x for x in rd.get('length_moved_by_cleanup') or [] if x['net'] == NET]
        check("results_data names the net", len(rows) == 1,
              str(rd.get('length_moved_by_cleanup')) + buf.getvalue()[-300:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}): " + '; '.join(FAILS))
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
