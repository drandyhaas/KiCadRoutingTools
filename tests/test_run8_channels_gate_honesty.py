#!/usr/bin/env python3
"""A gate must not report a pass it did not measure, and a delta must see loss.

Both defects were found by running the placement skill on a damaged board and
having a watcher re-derive the close-out.

1. VACUOUS SUCCESS. `check_channels --gate` auto-detects fine-pitch parts. On a
   board with none it printed one parenthetical note and exited 0, and the
   close-out recorded "exit 0, no starved face" -- a clean result from an
   instrument that had examined nothing. A component nothing looked at is
   UNEXAMINED, never clean. It now exits 3 and says so.

2. THE DELTA FILTERED BY AN ABSOLUTE THRESHOLD. `--min-demand` (default 7)
   exists to keep the ABSOLUTE form quiet on healthy boards, where a lightly
   used face with no spare lane is ordinary. It was also applied to the
   BASELINE comparison, where it has no business: the baseline already carries
   the design's own habits, so a face that goes from supply 4 to supply 0 is
   new damage at any demand. Measured: a repair closed one part's north face
   (demand 6, supply 4 -> 0) and the gate exited 0 because 6 < 7.

Run: python3 -X utf8 tests/test_run8_channels_gate_honesty.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ['KRT_NO_BANNER'] = '1'

BOARD = os.path.join('kicad_files', 'splitflap_driver.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']

# The CLI resolves these from the board; the library needs them spelled out.
LANE = {'clearance': 0.15, 'track_width': 0.2, 'grid_step': 0.05}

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def run(*args):
    p = subprocess.run(PY + ['py_tools/check_channels.py', *args], capture_output=True,
                       text=True, encoding='utf-8', errors='replace', cwd=ROOT)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def ledgered_ref(board):
    """A ref this board can actually build a lane ledger for."""
    from kicad_parser import parse_kicad_pcb
    from placement import routability
    pcb = parse_kicad_pcb(board)
    for ref, fp in sorted(pcb.footprints.items()):
        if len(fp.pads or []) < 6:
            continue
        rows = routability.face_lane_ledger(pcb, ref, pcb_file=board, **LANE)
        if rows and any(r['demand_nets'] >= 1 and r['supply_finest_grid'] > 0
                        for r in rows):
            return ref, pcb
    return None, pcb


def main():
    print('a gate that measured nothing does not report a pass')
    code, out = run(BOARD, '--gate', '--refs')       # explicitly nothing
    check('exit is not 0 when no part was ledgered', code != 0, f'exit {code}')
    check('...and it says the gate did not run', 'GATE DID NOT RUN' in out)
    check('...and it does not call that a pass', 'This is not a pass' in out)

    print('a gate that DID measure still answers normally')
    ref, _pcb = ledgered_ref(BOARD)
    check('the reference board has a ledgerable part', ref is not None)
    if ref is None:
        return 1
    code, out = run(BOARD, '--gate', '--refs', ref)
    check(f'a healthy board with a real ledger passes ({ref})', code == 0,
          f'exit {code}')
    code, out = run(BOARD, '--baseline', BOARD, '--gate', '--refs', ref)
    check('a board against ITSELF is silent', code == 0 and '0 NEW' in out,
          f'exit {code}')

    print('the delta sees a face that loses its last lane')
    # Tested on the predicate, not by moving parts: constructing damage that
    # reliably closes a specific face is a fixture problem, and a fixture that
    # fails to close one proves nothing about the rule.
    from check_channels import lost_last_lane, _starved_faces

    def row(face, demand, supply):
        return {'face': face, 'demand_nets': demand,
                'supply_finest_grid': supply}

    base = {'U1': [row('N', 6, 4), row('S', 4, 15), row('E', 1, 11)]}
    after = {'U1': [row('N', 6, 0), row('S', 4, 15), row('E', 1, 11)]}
    lost = lost_last_lane(after, base)
    check('a face that went 4 -> 0 is reported',
          [(r, f) for r, f, _d, _b in lost] == [('U1', 'N')], str(lost))
    check('...with the demand and the supply it had',
          lost and lost[0][2] == 6 and lost[0][3] == 4, str(lost))
    check('...and --min-demand 7 does NOT hide it (demand is 6)',
          not _starved_faces(after, 7), 'the absolute form still says nothing')

    check('a face that was ALREADY starved is not reported again',
          not lost_last_lane({'U1': [row('N', 6, 0)]}, {'U1': [row('N', 6, 0)]}))
    check('a face with no demand is not reported',
          not lost_last_lane({'U1': [row('N', 0, 0)]}, {'U1': [row('N', 0, 4)]}))
    check('an unchanged ledger reports nothing', not lost_last_lane(base, base))
    check('a face that GAINED supply is not reported',
          not lost_last_lane(base, after))
    check('a face absent from the baseline is not reported',
          not lost_last_lane({'U2': [row('N', 6, 0)]}, base))

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
