#!/usr/bin/env python3
"""#1799: an AC-coupled chain route_diff leaves over its tolerance is named,
recorded, and graded end to end.

Two pairs joined by series caps are one signal, matched end to end by
`--ac-couple-match`. Its pass reverted its meanders and left the chain
0.1148 mm off a 0.01 mm tolerance, yet the run printed "matched end-to-end
by the AC-coupled pass" for both members, recorded no tolerance for the
chain (and dropped the per-side ones), and check_drc and board_score then
graded nothing -- on copper the run without the flag failed on.

Rows:
  1. over tolerance: no "matched end-to-end" line; the `ac_coupled_xnets`
     row is `unmatched` at `tolerance_mm` 0.01 with the shipped skew; the
     chain is in the board's records; check_drc prints the chain row at
     0.1148 mm; board_score counts it in `blocking`;
  2. within tolerance (0.2 mm): `within-tolerance`, and check_drc grades the
     chain with none out of tolerance;
  3. control: without --ac-couple-match there is no `ac_coupled_xnets` key,
     the two per-pair tolerances are recorded and check_drc prints the two
     per-pair rows, as before;
  4. the #1759 snapshot (the graze passes' and the end-of-run guard's
     measure) carries the chain as one group.

    python3 tests/test_1799_ac_chain_graded.py
"""
import json
import os
import re
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
NETS = ['DPA_P', 'DPA_N', 'DPB_P', 'DPB_N']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def graded(*argv):
    """A grader whose exit code carries findings: run it, refuse a crash."""
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError('BROKEN TEST: the grader crashed\n' + out[-1500:])
    return out


def arm(tmp, a, tag, *flags):
    out = os.path.join(tmp, f'{tag}.kicad_pcb')
    js = os.path.join(tmp, f'{tag}.json')
    r = run_check([sys.executable, '-B', '-X', 'utf8', tool('route_diff.py'), a, out,
                   '--nets', *NETS, '--layers', 'F.Cu', 'B.Cu', '--track-width', '0.15',
                   '--diff-pair-gap', '0.25', '--diff-pair-intra-match', *flags,
                   '--json-out', js], accept=True)
    with open(evidence(js, f'{tag} --json-out'), encoding='utf-8') as fh:
        doc = json.load(fh)
    with open(evidence(out[:-len('.kicad_pcb')] + '.kicad_pro', f'{tag} project'),
              encoding='utf-8') as fh:
        rec = (json.load(fh).get('kicad_routing_tools') or {}).get(
            'intra_pair_tolerance') or {}
    return evidence(out, f'{tag} board'), r.stdout + r.stderr, doc, rec


def main():
    tmp = tempfile.mkdtemp(prefix='t1799_')
    try:
        a = os.path.join(tmp, 'a.kicad_pcb')
        run_check([sys.executable, '-B', '-X', 'utf8', tool('copy_board.py'),
                   os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), a],
                  accept=True)

        print("1. a chain the pass left over its tolerance")
        b, log, doc, rec = arm(tmp, a, 'ac', '--ac-couple-match',
                               '--length-match-tolerance', '0.01')
        rows = doc.get('ac_coupled_xnets') or []
        if not rows or 'AC-couple DPA+DPB' not in log:
            print("  [BROKEN TEST] the AC-coupled pass found no chain:\n" + log[-800:])
            return 1
        row = rows[0]
        check("no 'matched end-to-end' line", 'matched end-to-end' not in log)
        check("the row is unmatched at tolerance 0.01 with the shipped skew",
              row.get('status') == 'unmatched' and row.get('tolerance_mm') == 0.01
              and abs((row.get('skew_mm') or 0) - 0.1148) < 1e-3, str(row))
        check("the run names the chain NOT matched",
              re.search(r'AC-coupled chain NOT matched: DPA\+DPB \(', log) is not None)
        ch = rec.get('ac-chain:DPA+DPB') or {}
        check("the chain is recorded with its tolerance",
              ch.get('tolerance_mm') == 0.01 and ch.get('p_nets') == ['DPA_P', 'DPB_P']
              and ch.get('n_nets') == ['DPA_N', 'DPB_N'], str(rec))
        out = graded(tool('check_drc.py'), b)
        check("check_drc prints the chain row at 0.1148 mm",
              re.search(r'DPA_N \+ DPB_N: 0\.1148mm shorter than DPA_P \+ DPB_P end '
                        r'to end', out) is not None
              and 'AC-coupled chains (recorded by route_diff, #1799): 1 graded end '
                  'to end, 1 out of tolerance' in out, out[-600:])
        sc = graded(os.path.join(ROOT, 'py_tools', 'board_score.py'), b)
        m = re.search(r'^BLOCKING=(\d+)', sc, re.M)
        check("board_score counts it in blocking",
              m is not None and int(m.group(1)) > 0 and 'diff_pairs=1' in sc,
              (m.group(0) if m else sc[-300:]))

        print("2. a chain inside its tolerance")
        b2, log2, doc2, _rec2 = arm(tmp, a, 'ok', '--ac-couple-match',
                                    '--length-match-tolerance', '0.2')
        row2 = (doc2.get('ac_coupled_xnets') or [{}])[0]
        check("within-tolerance", row2.get('status') == 'within-tolerance', str(row2))
        out = graded(tool('check_drc.py'), b2)
        check("check_drc grades it, none out of tolerance",
              'AC-coupled chains (recorded by route_diff, #1799): 1 graded end to '
              'end, 0 out of tolerance' in out, out[-400:])

        print("3. control: no --ac-couple-match")
        b3, log3, doc3, rec3 = arm(tmp, a, 'side', '--length-match-tolerance', '0.01')
        check("no ac_coupled_xnets key", 'ac_coupled_xnets' not in doc3)
        check("two per-pair tolerances, no chain",
              sorted(rec3) == ['DPA_P', 'DPB_P'], str(rec3))
        out = graded(tool('check_drc.py'), b3)
        check("check_drc prints the two per-pair rows",
              len(re.findall(r'DP[AB]_N: 0\.0574mm shorter than DP[AB]_P', out)) == 2
              and 'AC-coupled chains' not in out, out[-400:])

        print("4. the #1759 snapshot")
        from kicad_parser import parse_kicad_pcb
        from check_drc import skew_group_snapshot
        snap = skew_group_snapshot(parse_kicad_pcb(b), b)
        rows = [v for k, v in snap.items() if v and k[0] == 'chain DPA+DPB']
        check("the chain is one group, out of range",
              len(rows) == 1 and rows[0]['max_mm'] == 0.01
              and rows[0]['excess_mm'] > 0.1, str(snap))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
