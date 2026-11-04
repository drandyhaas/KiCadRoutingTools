#!/usr/bin/env python3
"""
#1421: route_diff.py takes route.py's `--json-out FILE`.

It used to reject the flag at argparse (exit 2), so a chain that put
`--json-out` on its route_diff step -- the judge_seeds `{dir}` convention --
failed every seed with no board written.

Checks, on the in-repo LVDS board the fanout-and-route test also routes:
  1. a routing run exits 0 and writes FILE, holding the same JSON_SUMMARY the
     run printed (plus the via-in-pad record the printed line predates);
  2. a re-run over the routed board, where every pair is already connected,
     still writes FILE, with status 'already_connected', so a caller can tell
     "nothing to do" from a crash.

Run:
    python3 tests/test_1421_route_diff_json_out.py
"""

import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)

from run_utils import check, evidence, tool

BOARD = os.path.join(ROOT_DIR, 'kicad_files', 'routed_output.kicad_pcb')
OPTS = ['--nets', '*lvds_rx1_1*', '--impedance', '100',
        '--proximity-heuristic-factor', '0.0', '--clearance', '0.1',
        '--via-size', '0.3', '--via-drill', '0.2', '--track-width', '0.1',
        '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'B.Cu']


def _printed_summary(stdout):
    lines = [l for l in stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    assert lines, "the run printed no JSON_SUMMARY"
    return json.loads(lines[-1][len('JSON_SUMMARY: '):])


def main():
    evidence(BOARD, 'input board')
    tmp = tempfile.mkdtemp(prefix='t1421_')
    inp = os.path.join(tmp, 'in.kicad_pcb')
    # copy_board carries the sibling .kicad_pro (#441).
    subprocess.run([sys.executable, tool('copy_board.py'), BOARD, inp],
                   check=True, capture_output=True)
    route_diff = tool('route_diff.py')

    # 1. A run that routes.
    out1 = os.path.join(tmp, 'out1.kicad_pcb')
    js1 = os.path.join(tmp, 'route1.json')
    r = check([sys.executable, route_diff, inp, out1, *OPTS,
               '--json-out', js1], accept=True)
    evidence(js1, '--json-out file')
    doc = json.load(open(js1))
    printed = _printed_summary(r.stdout)
    assert doc.get('successful', 0) > 0, f"routed nothing: {doc}"
    assert {k: v for k, v in doc.items() if k != 'via_in_pad'} == printed, \
        "the --json-out file differs from the printed JSON_SUMMARY"
    print(f"  PASS routing run: file == printed JSON_SUMMARY "
          f"(successful={doc['successful']}, failed={doc['failed']})")

    # 2. Nothing left to route.
    out2 = os.path.join(tmp, 'out2.kicad_pcb')
    js2 = os.path.join(tmp, 'route2.json')
    check([sys.executable, route_diff, out1, out2, *OPTS,
           '--json-out', js2], accept=True)
    evidence(js2, '--json-out file (nothing to route)')
    doc2 = json.load(open(js2))
    assert doc2.get('status') == 'already_connected', doc2
    assert doc2.get('successful') == 0 and doc2.get('failed') == 0, doc2
    print("  PASS nothing-to-route run: status 'already_connected' written")

    # 3. The pattern matches nets but no pair (a ground net): the run exits non-zero, and the file
    #    still says why, so the caller can tell it from a crash.
    js3 = os.path.join(tmp, 'route3.json')
    r3 = subprocess.run([sys.executable, route_diff, inp,
                         os.path.join(tmp, 'out3.kicad_pcb'),
                         '--nets', '*GND*', '--json-out', js3],
                        capture_output=True, text=True)
    assert 'No differential pairs found matching' in r3.stdout, r3.stdout[-800:]
    evidence(js3, '--json-out file (no pair matched)')
    doc3 = json.load(open(js3))
    assert doc3.get('status') == 'no_diff_pairs', doc3
    print("  PASS no-pair-matched run: status 'no_diff_pairs' written")

    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
