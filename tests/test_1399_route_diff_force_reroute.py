#!/usr/bin/env python3
"""#1399: route_diff --force-reroute re-routes an already-connected pair AS a
coupled pair.

route.py's --force-reroute is single-ended, so the skill's #521 override
recipe de-paired a pair route_diff had coupled; route_diff itself had no way to
re-route a connected-but-uncoupled pair ("nothing to route!", exit 0).

Asserts, on a board with one differential pair:
  * control: without the flag, a connected single-ended pair is skipped;
  * with the flag it comes back coupled (outcome `coupled`, most of its length
    coupled) with the original copper gone from the file;
  * the pair is then #521-protected: a glob does not override it, both members
    named exactly do;
  * a re-route that lands no copper gives the ORIGINAL copper back, byte for
    segment (the custody rule route.py's flag has);
  * the flag needs an explicit scope (exit 2, with its reason).

    python3 tests/test_1399_route_diff_force_reroute.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']
PAIR = ['CSI_D0_P', 'CSI_D0_N']


def summary(text):
    line = next(l for l in text.splitlines() if l.startswith('JSON_SUMMARY: '))
    return json.loads(line.split(': ', 1)[1])


def n_copper(path):
    with open(evidence(path, 'board'), encoding='utf-8') as f:
        return len(re.findall(r'^\s*\((segment|via)\b', f.read(), re.M))


def main():
    evidence(BOARD, 'diff-pair board')
    from copy_board import copy_board
    tmp = tempfile.mkdtemp(prefix='t1399_')
    try:
        inb = os.path.join(tmp, 'in.kicad_pcb')
        copy_board(BOARD, inb)
        se = os.path.join(tmp, 'se.kicad_pcb')
        check(PY + [tool('route.py'), inb, se, '--nets'] + PAIR, accept=True)

        # control: the connected pair is skipped without the flag
        r = check(PY + [tool('route_diff.py'), se, os.path.join(tmp, 'c.kicad_pcb'),
                        '--nets', 'CSI_D0_*'], accept=True)
        assert 'nothing to route' in r.stdout, r.stdout[-400:]

        # the flag re-routes it as a coupled pair
        fr = os.path.join(tmp, 'fr.kicad_pcb')
        r = check(PY + [tool('route_diff.py'), se, fr, '--nets', 'CSI_D0_*',
                        '--force-reroute'], accept=True)
        assert '--force-reroute: stripped' in r.stdout, r.stdout[-600:]
        rep = summary(r.stdout)['pair_reports'][0]
        cu = rep['copper']
        assert rep['outcome'] == 'coupled' and cu['coupled'], rep
        assert cu['coupled_mm'] > 0.5 * cu['length_p_mm'], cu
        # the single-ended originals are gone from the file (the re-route
        # replaced them; it did not stack on them)
        assert n_copper(fr) < 2 * n_copper(se), (n_copper(fr), n_copper(se))

        # now protected: a glob is no override, exact names are
        r = check(PY + [tool('route_diff.py'), fr, os.path.join(tmp, 'g.kicad_pcb'),
                        '--nets', 'CSI_D0_*', '--force-reroute'], accept=True)
        assert 'PROTECTED net(s) excluded from --force-reroute' in r.stdout, \
            r.stdout[-600:]
        r = check(PY + [tool('route_diff.py'), fr, os.path.join(tmp, 'e.kicad_pcb'),
                        '--nets'] + PAIR + ['--force-reroute'], accept=True)
        assert '--force-reroute: stripped' in r.stdout, r.stdout[-600:]

        # custody: a re-route that cannot land gives the original copper back
        cu_out = os.path.join(tmp, 'cu.kicad_pcb')
        r = check(PY + [tool('route_diff.py'), fr, cu_out, '--nets'] + PAIR
                  + ['--force-reroute', '--max-iterations', '5',
                     '--max-probe-iterations', '5'], accept=True)
        assert 'ORIGINAL copper restored' in r.stdout, r.stdout[-800:]
        assert n_copper(cu_out) == n_copper(fr), (n_copper(cu_out), n_copper(fr))

        # scope required
        check(PY + [tool('route_diff.py'), fr, os.path.join(tmp, 'x.kicad_pcb'),
                    '--force-reroute'],
              refuse='requires an explicit net scope', code=2,
              allow=('error: argument',))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('PASS')


if __name__ == '__main__':
    main()
