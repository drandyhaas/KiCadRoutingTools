#!/usr/bin/env python3
"""#1980: an AC-coupled chain whose halves are each too short to couple.

`route_diff --ac-couple-match` defers each half of such a chain to
single-ended routing (electrically short), and the step's own single-ended
member fallback then routes each member net on its own. The end-to-end pass
read every member as ONE coupled result holding both nets, so it took the N
copper out of the P net's result: N read 0 mm, the pass reported
`no-room-for-meanders` and the chain shipped with its skew. Where a half has
no copper at all, the row said `member-not-routed`, naming the missing copper
rather than the deferral that left it so.

Rows:
  1. both halves short, both routed by the fallback: the pass measures each
     net from its own result, meanders the chain inside its tolerance, and
     check_drc grades the shipped chain with none out of tolerance; the board
     is DRC-clean and connected;
  2. a short half the fallback cannot connect: the chain row reads
     `member-deferred-electrically-short`, and the console says the member was
     deferred, not that it is unrouted;
  3. control: halves long enough to couple route coupled and keep their row
     exactly as before (the coupled path is unchanged).

    python3 tests/test_1980_ac_chain_short_halves.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
CHAIN = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
CHAIN_NETS = ['DPA_P', 'DPA_N', 'DPB_P', 'DPB_N']
CHAIN_GEOM = ['--layers', 'F.Cu', 'B.Cu', '--track-width', '0.15',
              '--diff-pair-gap', '0.25']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail and not cond else ''))


def graded(*argv):
    """A grader whose exit code carries findings: run it, refuse a crash."""
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError('BROKEN TEST: the grader crashed\n' + out[-1500:])
    return out


def arm(tmp, board, tag, nets, *flags):
    out = os.path.join(tmp, f'{tag}.kicad_pcb')
    js = os.path.join(tmp, f'{tag}.json')
    r = run_check([sys.executable, '-B', '-X', 'utf8', tool('route_diff.py'),
                   board, out, '--nets', *nets, '--diff-pair-intra-match',
                   '--ac-couple-match', *flags, '--json-out', js], accept=True)
    with open(evidence(js, f'{tag} --json-out'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return evidence(out, f'{tag} board'), r.stdout + r.stderr, doc


def outcomes(doc):
    return {p['pair']: (p.get('outcome'), p.get('failure_reason'))
            for p in doc.get('pair_reports') or []}


def main():
    tmp = tempfile.mkdtemp(prefix='t1980_')
    try:
        a = os.path.join(tmp, 'a.kicad_pcb')
        run_check([sys.executable, '-B', '-X', 'utf8', tool('copy_board.py'),
                   CHAIN, a], accept=True)

        print("1. both halves under the coupled floor, both routed single-ended")
        # A setback of 2 mm puts the coupled floor at 10 mm, above either
        # half of the chain (about 8 mm each) and below the chain itself.
        b, log, doc = arm(tmp, a, 'short', CHAIN_NETS, *CHAIN_GEOM,
                          '--diff-pair-centerline-setback', '2.0')
        oc = outcomes(doc)
        if (oc.get('DPA') != ('deferred', 'electrically-short')
                or oc.get('DPB') != ('deferred', 'electrically-short')):
            print("  [BROKEN TEST] the halves were not deferred as short: " + str(oc))
            return 1
        fb = doc.get('single_ended_fallback') or {}
        if sorted(fb.get('routed') or []) != sorted(CHAIN_NETS):
            print("  [BROKEN TEST] the fallback did not route every member: " + str(fb))
            return 1
        row = (doc.get('ac_coupled_xnets') or [{}])[0]
        check("each side measured from its own nets' copper (no 0 mm side)",
              'N=0.000mm' not in log and 'P=0.000mm' not in log
              and 'AC-couple DPA+DPB: end-to-end' in log, log[-600:])
        check("the chain is matched inside its tolerance on the shipped copper",
              row.get('status') in ('matched', 'within-tolerance')
              and row.get('skew_mm') is not None
              and row['skew_mm'] <= row.get('tolerance_mm', 0) + 1e-9, str(row))
        out = graded(tool('check_drc.py'), b)
        check("check_drc grades the chain, none out of tolerance",
              'AC-coupled chains (recorded by route_diff, #1799): 1 graded end to '
              'end, 0 out of tolerance' in out
              and 'NO DRC VIOLATIONS FOUND' in out, out[-600:])
        out = graded(tool('check_connected.py'), b)
        check("the board is connected", 'ALL NETS FULLY CONNECTED' in out, out[-300:])

        print("2. a short half the fallback cannot connect")
        ux = os.path.join(tmp, 'u.kicad_pcb')
        run_check([sys.executable, '-B', '-X', 'utf8', tool('copy_board.py'),
                   os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb'), ux],
                  accept=True)
        _b2, log2, doc2 = arm(tmp, ux, 'unrouted', ['FPDI_D0*', '/gpdi/GPDI_D0*'],
                              '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu')
        reps = {p['pair']: p for p in doc2.get('pair_reports') or []}
        gp = reps.get('/gpdi/GPDI_D0') or {}
        if (gp.get('outcome') != 'deferred'
                or gp.get('failure_reason') != 'electrically-short'
                or (gp.get('se_fallback') or {}).get('p') != 'failed'):
            print("  [BROKEN TEST] the case needs a deferred half the fallback "
                  "cannot connect: " + str(gp))
            return 1
        rows2 = doc2.get('ac_coupled_xnets') or []
        check("the chain row names the deferral",
              len(rows2) == 1 and rows2[0].get('status') == 'not-matchable'
              and rows2[0].get('reason') == 'member-deferred-electrically-short',
              str(rows2))
        check("the console says the member was deferred",
              "was deferred to single-ended routing (electrically-short" in log2
              and 'not fully routed; skipping' not in log2, log2[-600:])

        print("3. control: halves long enough to couple")
        _b3, _log3, doc3 = arm(tmp, a, 'long', CHAIN_NETS, *CHAIN_GEOM)
        check("both halves route coupled",
              sorted(doc3.get('routed_diff_pairs') or []) == ['DPA', 'DPB'],
              str(doc3.get('routed_diff_pairs')))
        row3 = (doc3.get('ac_coupled_xnets') or [{}])[0]
        check("the chain row is as before",
              (row3.get('status'), row3.get('reason'), row3.get('skew_mm'))
              == ('unmatched', 'meanders-would-worsen', 0.1148), str(row3))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("ALL PASS" if not FAILS else f"FAILED: {FAILS}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
