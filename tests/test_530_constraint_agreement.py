#!/usr/bin/env python3
"""run_all wrapper for tests/oracle/constraint_agreement.py (#530).

Runs every agreement row against the installed kicad-cli; SKIPS (exit 0,
with a line saying so) where KiCad is not installed, so a machine without
it reports "not measured" rather than "agreed".
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HARNESS = os.path.join(ROOT, 'tests', 'oracle', 'constraint_agreement.py')


def main():
    r = subprocess.run([sys.executable, HARNESS], capture_output=True, text=True, cwd=ROOT)
    tail = '\n'.join(r.stdout.strip().splitlines()[-30:])
    print(tail)
    if r.stdout.startswith('SKIP'):
        return 0
    if r.returncode != 0:
        print("FAIL: at least one constraint row disagrees with KiCad's DRC engine")
        return 1
    print("PASS: every constraint row agrees with KiCad's DRC engine")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
