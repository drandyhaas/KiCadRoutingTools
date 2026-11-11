#!/usr/bin/env python3
"""#1812: `board_score --length-groups` grades a length limit, and a lane
through a series part end to end.

The lane J1.1 -> C1 -> J2.1 on the routed board is two nets, DPA_P and
DPB_P, of 8.33 mm each. A group `{"nets": [...], "max_mm": 12}` used to read
only the spread (0.0, pass) -- `max_mm` was not read -- and no grader summed
the two nets, so a 16.66 mm lane passed a 12 mm limit.

Rows (score_length on group files this test writes):
  1. a chain measures the sum of its nets' net_match_length, and the series
     part's own span is reported apart, not added;
  2. `max_mm: 12` on the chain fails the group and names the chain;
  3. `max_mm: 20` passes; a per-net `max_mm: 8` on `nets` fails;
  4. control: a group with only `nets` and `tolerance_mm` returns the dict
     it always did;
  5. a chain whose nets no 2-pad part joins fails with a reason naming it,
     a key the function does not read is named, and on the unrouted input
     the chain group is unmeasured (`ran` false), not failed;
  6. `"chains": "auto"` finds both sides of the AC-coupled pair.

    python3 tests/test_1812_length_chain_limits.py
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
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def main():
    from board_score import score_length
    from kicad_parser import parse_kicad_pcb
    from net_queries import net_match_length
    tmp = tempfile.mkdtemp(prefix='t1812_')
    try:
        a = os.path.join(tmp, 'in.kicad_pcb')
        r = os.path.join(tmp, 'r.kicad_pcb')
        run_check([sys.executable, '-B', '-X', 'utf8', tool('copy_board.py'),
                   os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), a],
                  accept=True)
        run_check([sys.executable, '-B', '-X', 'utf8', tool('route.py'),
                   evidence(a, 'board copy'), r, '--nets', 'DPA_P', 'DPB_P',
                   'DPA_N', 'DPB_N'], accept=True)
        pcb = parse_kicad_pcb(evidence(r, 'routed board'))
        ids = {n.name: i for i, n in pcb.nets.items()}
        la = net_match_length(pcb, ids['DPA_P'])
        lb = net_match_length(pcb, ids['DPB_P'])
        if la is None or lb is None:
            print("  [BROKEN TEST] the lane's nets are not routed")
            return 1

        def grade(spec, board=r):
            f = os.path.join(tmp, 'g.json')
            with open(f, 'w', encoding='utf-8') as fh:
                json.dump(spec, fh)
            return score_length(board, evidence(f, 'groups file'))

        print("1. the chain measure")
        s = grade({'LANE_P': {'chains': [['DPA_P', 'DPB_P']], 'tolerance_mm': 0.5,
                              'max_mm': 12}})
        g = s['groups']['LANE_P']
        ch = (g.get('chains') or {}).get('DPA_P + DPB_P') or {}
        check("the chain is its nets' lengths summed",
              ch.get('track_mm') is not None and abs(ch['track_mm'] - (la + lb)) < 1e-3,
              f"{ch.get('track_mm')} vs {la + lb:.4f}")
        part = (ch.get('series_parts') or [{}])[0]
        check("the series part and its span are reported apart",
              part.get('part') == 'C1' and part.get('span_mm', 0) > 0
              and ch.get('series_span_mm') == part.get('span_mm'), str(part))

        print("2. max_mm on the chain")
        check("count 1 and the chain named over max_mm 12",
              s['count'] == 1 and [o['member'] for o in g.get('out_of_bounds') or []]
              == ['DPA_P + DPB_P'] and g['out_of_bounds'][0]['limit_mm'] == 12,
              json.dumps(g)[:300])

        print("3. the other bounds")
        s = grade({'LANE_P': {'chains': [['DPA_P', 'DPB_P']], 'max_mm': 20}})
        check("max_mm 20 passes", s['count'] == 0, json.dumps(s['groups'])[:200])
        s = grade({'LANE_P': {'nets': ['DPA_P', 'DPB_P'], 'max_mm': 8}})
        check("a per-net max_mm 8 fails", s['count'] == 1
              and len(s['groups']['LANE_P'].get('out_of_bounds') or []) == 2,
              json.dumps(s['groups'])[:300])

        print("4. control: nets and tolerance only")
        s = grade({'OLD': {'nets': ['DPA_P', 'DPB_P'], 'tolerance_mm': 0.5}})
        check("the same dict as before", s['count'] == 0 and s['groups']['OLD'] == {
            'spread_mm': 0.0, 'tolerance_mm': 0.5, 'pass': True,
            'missing_nets': [], 'worst': 'DPA_P'}, str(s['groups']['OLD']))

        print("5. findings about the spec, and an unrouted board")
        s = grade({'BAD': {'chains': [['DPA_P', 'DPB_N']], 'maxlen': 3}})
        g = s['groups']['BAD']
        check("an unbridged chain fails with a reason",
              s['count'] == 1 and 'no part with two copper pads'
              in json.dumps(g.get('unbridged')), json.dumps(g)[:300])
        check("an unknown key is named", g.get('unknown_keys') == ['maxlen'])
        s = grade({'LANE_P': {'chains': [['DPA_P', 'DPB_P']], 'max_mm': 12}}, board=a)
        check("the unrouted input reads unmeasured, not failed",
              s.get('ran') is False and 'LANE_P' in (s.get('groups') or {}),
              json.dumps(s)[:300])

        print('6. "chains": "auto"')
        s = grade({'LANE': {'chains': 'auto', 'max_mm': 20, 'tolerance_mm': 5}})
        got = sorted((s['groups']['LANE'].get('chains') or {}))
        check("both sides of the AC-coupled pair",
              got == ['DPA_N + DPB_N', 'DPA_P + DPB_P'] and s['count'] == 0,
              str(got))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
