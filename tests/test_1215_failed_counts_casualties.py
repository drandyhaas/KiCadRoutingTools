#!/usr/bin/env python3
"""#1215: route.py's `failed` counts a net the run ripped and could not
re-route, and the headline names it as broken by this run.

`regrade_record` computed `failed = len(scope) - len(routed)` over pass 1's
single-ended list. A rip casualty is never in that list, so on rp2350 a run
that ripped /RP2354A/FPGA.~{RESET} and failed its re-route printed
`Single-ended: 2/2 routed`, "Broken outside the routing scope: ...RESET" and
`failed 0`, while its own improvement gate listed RESET as broken by the
run. A chain helper trusting `failed` stopped its rip loop early.

Checks:
  1. The issue's no-board repro: failed is 1 (B), successful 1 (A).
  2. `_final_regrade` on tracked routed_output with net B's copper removed
     from the "shipped" file and a summary that lists B as a failed re-route
     outside pass 1's scope: failed counts it, `broken_by_run` names it, and
     the headline says "Broken by this run", never "no worse than on the
     input".

    python3 tests/test_1215_failed_counts_casualties.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from route_summary import regrade_record                       # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def main():
    # 1. The issue's unit.
    s = {'routed_single': ['A'], 'failed_single': ['B'], 'open_single': [],
         'failed_multipoint': []}
    g = {'A': {'pads': 2, 'broken': False, 'copper': True, 'failed_pads': []},
         'B': {'pads': 2, 'broken': True, 'copper': False,
               'failed_pads': [{'x': 0, 'y': 0}]}}
    rg = regrade_record([s], g, ['A'], board='file')
    check('regrade_record: failed counts the casualty outside the scope',
          (rg['successful'], rg['failed'], rg['failed_single']) == (1, 1, ['B']),
          f"successful {rg['successful']} failed {rg['failed']}")

    # 2. End to end through _final_regrade.
    import route
    from copy_board import copy_board
    from kicad_parser import parse_kicad_pcb
    from check_connected import check_net_connectivity
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    segs, vias = {}, {}
    for x in pcb.segments:
        segs.setdefault(x.net_id, []).append(x)
    for v in pcb.vias:
        vias.setdefault(v.net_id, []).append(v)

    def connected_two_pad(n):
        pads = pcb.pads_by_net.get(n, [])
        if len(pads) != 2 or n not in segs or any(z.net_id == n for z in pcb.zones or []):
            return False
        c = check_net_connectivity(n, segs[n], vias.get(n, []), pads, [])
        return c['num_components'] == 1 and not c['disconnected_pads']
    two = [n for n in sorted(segs) if n and n in pcb.nets and connected_two_pad(n)]
    check('precondition: two connected two-pad nets', len(two) >= 2, str(len(two)))
    if len(two) < 2:
        return 1
    a_id, b_id = two[0], two[1]
    a, b = pcb.nets[a_id].name, pcb.nets[b_id].name

    tmp = tempfile.mkdtemp(prefix='t1215_')
    try:
        out = os.path.join(tmp, 'out.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(BOARD, out)
        text = open(out, encoding='utf-8').read()
        net_pat = r'\(net (?:%d|"%s")\)' % (b_id, re.escape(b))
        blocks = [m for m in re.finditer(r'\t\(segment\n(?:\t\t[^\n]*\n)+?\t\)\n', text)
                  if re.search(net_pat, m.group(0))]
        for m in reversed(blocks):
            text = text[:m.start()] + text[m.end():]
        open(out, 'w', encoding='utf-8').write(text)
        check('precondition: net B lost its copper in the shipped file',
              len(blocks) == len(segs[b_id]), f'{len(blocks)} of {len(segs[b_id])}')

        route._SUMMARY_SINK.clear()
        route._SUMMARY_SINK.append({'scope': 'run', 'routed_single': [a],
                                    'failed_single': [], 'open_single': [],
                                    'failed_multipoint': []})
        route._SUMMARY_SINK.append({'scope': 'reroute', 'routed_single': [],
                                    'failed_single': [b], 'open_single': [],
                                    'failed_multipoint': []})
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rec = route._final_regrade(pcb, out, False, None, None, [a], None,
                                       segs, vias)
        con = buf.getvalue()
        route._SUMMARY_SINK.clear()
        check('the record exists', rec is not None, con[-400:])
        if rec is None:
            return 1
        check('failed counts the ripped net', rec['failed'] == 1
              and rec['successful'] == 1, f"failed {rec['failed']} successful "
              f"{rec['successful']}")
        check('broken_by_run names it', rec.get('broken_by_run') == [b],
              str(rec.get('broken_by_run')))
        line = next((l for l in con.splitlines() if 'Broken by this run' in l), '')
        check('the headline calls it broken by this run', b in line, line)
        check('it is not called "no worse than on the input"',
              not any('no worse than on the input' in l and b in l
                      for l in con.splitlines()))
        check('the headline prints the failed count',
              'Ships broken: 1 net(s)' in con)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
