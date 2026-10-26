#!/usr/bin/env python3
"""#1432: the length-window grade measures as the installed kicad-cli, and says so.

KiCad's length DRC measures differently from release to release within 10.0
(DATA_13 on routed_output: 15.149 mm in 10.0.0, 15.378 in 10.0.7, against a
15.25 mm max), so check_drc measures as the release of the kicad-cli that
grades the board (tests/test_1432_kicad_release_length.py checks each release)
and STATES which release that is.

  1. The release follows `kicad_oracle.kicad_cli_version`: an installed
     release in the modelled range is measured as itself; a newer one as the
     newest modelled, an older one as the oldest, and the note names it; with
     no kicad-cli, the newest modelled, and the note says so.
  2. The installed release is read once, not once per net measured.
  3. check_drc prints the release beside the length summary, and its summary
     carries `calibrated_to_kicad` and `installed_kicad`.

    python3 tests/test_1432_length_measure_calibration.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import net_queries                                             # noqa: E402
import kicad_oracle                                            # noqa: E402
from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def installed(version):
    """Make `version` the installed kicad-cli's release, as kicad_oracle
    reports it."""
    kicad_oracle.kicad_cli_version = lambda cli=None: version
    net_queries.installed_kicad_release.cache_clear()


def main():
    newest = net_queries.LENGTH_MEASURE_NEWEST
    oldest = net_queries.LENGTH_MEASURE_OLDEST
    real = kicad_oracle.kicad_cli_version
    try:
        for have, want, said in (('10.0.0', '10.0.0', None),
                                 ('10.0.5', '10.0.5', None),
                                 ('10.2.1', newest, 'newer than the newest'),
                                 ('9.0.7', oldest, 'older than the oldest'),
                                 (None, newest, 'no kicad-cli found')):
            installed(have)
            n = net_queries.length_measure_note()
            check(f'installed {have}: measured as {want}',
                  net_queries.length_measure_release() == want
                  and f'measured as KiCad {want} measures' in n
                  and (said in n if said else 'installed' not in n), n)

        calls = [0]

        def counting(cli=None):
            calls[0] += 1
            return '10.0.0'
        kicad_oracle.kicad_cli_version = counting
        net_queries.installed_kicad_release.cache_clear()
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(BOARD)
        lengths = net_queries.net_copper_lengths(pcb)
        check('the installed release is read once, not once per net',
              calls[0] == 1 and len(lengths) > 100,
              f'{calls[0]} kicad-cli version reads for {len(lengths)} nets')
    finally:
        kicad_oracle.kicad_cli_version = real
        net_queries.installed_kicad_release.cache_clear()

    want = net_queries.length_measure_release()
    have = net_queries.installed_kicad_release()
    tmp = tempfile.mkdtemp(prefix='t1432_')
    try:
        dst = os.path.join(tmp, 'ro.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(BOARD, dst)
        with open(os.path.join(tmp, 'ro.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "d13"\n'
                     '  (condition "A.NetName == \'Net-(U2A-DATA_13)\'")\n'
                     '  (constraint length (max 15.25mm)))\n')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'check_drc.py'), dst,
                            '--clearance-margin', '0.1'], cwd=ROOT,
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace')
        out = r.stdout + r.stderr
        ok = ('Length rules (.kicad_dru' in out
              and f'(length) lengths are measured as KiCad {want}' in out)
        check(f'check_drc prints the release beside the length summary ({want})',
              ok, '' if ok else out[-400:])

        import check_drc
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(dst)
            summary = check_drc._length_rule_pass(dst, pcb, None, [], True)
        check('the summary carries the release measured as and the one installed',
              summary and summary['calibrated_to_kicad'] == want
              and summary['installed_kicad'] == have, str(summary))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
