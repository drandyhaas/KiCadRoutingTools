#!/usr/bin/env python3
"""A via re-laid onto a paste opening is a site this run created (#1171).

glasgow_revC: a rip of +1V2 laid its via again 0.1 mm from the input's, with
the barrel 6 um into U36.2's F.Paste. `_input_match` reads any same-net input
via within half a diameter as THIS via, and route.py snapshotted its input vias
without the board, so `in_site` was None ("unknown") and the stamp kept the via
"as the input had it" -- although the input had it outside every opening.
check_drc --baseline then reported VIA-IN-PASTE 1 while board_score --baseline
kept it advisory.

Rows: the stamp decision with and without the board in the snapshot; every
routing step's snapshot passes the board; board_score counts the via in
BLOCKING under --baseline and keeps it advisory without.

    python3 tests/test_1171_via_snapshot_sites.py
"""
import ast
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from fab_notes import via_snapshot, via_protection_stamps      # noqa: E402
from test_962_check_drc_via_in_paste import board, via         # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


work = tempfile.mkdtemp(prefix='t1171_')
try:
    # U1.1 opens F.Paste over [9.5, 10.5]^2. The input via (0.4 mm) clears it
    # by 0.05 mm; the output's, 0.1 mm away, is 0.05 mm into it.
    inp_path = board(work, 'in', [via(10.0, 10.75, uid='v1')])
    out_path = board(work, 'out', [via(10.0, 10.65, uid='v1')])
    inp, out = parse_kicad_pcb(inp_path), parse_kicad_pcb(out_path)

    print('A. the stamp decision')
    st, rec = via_protection_stamps(out.vias, via_snapshot(inp.vias), out)
    check('control: without the board the via is kept "as the input had it"',
          not st and any('kept as the input had it' in u['why']
                         for u in rec.get('unprotected', [])), str(rec.get('unprotected')))
    snap = via_snapshot(inp.vias, inp)
    check('with the board the input via is known to be OUTSIDE solder',
          [e[5] for e in snap] == [False], str(snap))
    st, rec = via_protection_stamps(out.vias, snap, out)
    check('...so the re-laid via is a site this run created, and is stamped',
          len(st) == 1 and rec.get('stamped') == 1 and not rec.get('unprotected'),
          json.dumps({k: rec.get(k) for k in ('stamped', 'unprotected')}))

    print('B. every routing step snapshots WITH the board')
    sites = []
    for rel in ('py_router/route.py', 'py_router/route_diff.py',
                'py_router/route_planes.py', 'py_router/repair_planes.py',
                'py_placer/place_fanout_clearance.py'):
        tree = ast.parse(open(os.path.join(ROOT, rel), encoding='utf-8').read())
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, 'id', getattr(node.func, 'attr', ''))
                    in ('via_snapshot', '_via_snapshot962')):
                sites.append((rel, node.lineno, len(node.args) + len(node.keywords)))
    check('the snapshot sites were found', len(sites) >= 5, str(sites))
    check('each passes the board', all(n >= 2 for _r, _l, n in sites),
          str([s for s in sites if s[2] < 2]))

    print('C. board_score --baseline counts what check_drc did not inherit')

    def score(*extra):
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_tools', 'board_score.py'),
                            out_path, '--quiet', *extra],
                           capture_output=True, text=True, cwd=ROOT)
        line = next((ln for ln in r.stdout.splitlines() if ln.startswith('SCORE_JSON=')), None)
        return json.loads(line[len('SCORE_JSON='):]) if line else {'_err': r.stdout[-800:] + r.stderr[-800:]}

    s_base = score('--baseline', inp_path)
    check('with --baseline the via-in-paste is in blocking_by',
          (s_base.get('blocking_by') or {}).get('drc_via_in_paste') == 1
          and 'drc_via_in_paste' not in (s_base.get('advisory') or {}),
          json.dumps({k: s_base.get(k) for k in ('blocking_by', 'advisory', '_err')}))
    s_none = score()
    check('without --baseline it stays advisory',
          (s_none.get('advisory') or {}).get('drc_via_in_paste') == 1
          and 'drc_via_in_paste' not in (s_none.get('blocking_by') or {}),
          json.dumps({k: s_none.get(k) for k in ('blocking_by', 'advisory', '_err')}))
    check('...and blocking differs by exactly that via',
          s_base.get('blocking') is not None and s_none.get('blocking') is not None
          and s_base['blocking'] - s_none['blocking'] == 1,
          f"{s_base.get('blocking')} vs {s_none.get('blocking')}")
finally:
    shutil.rmtree(work, ignore_errors=True)

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
