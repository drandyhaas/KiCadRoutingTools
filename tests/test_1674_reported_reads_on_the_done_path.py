#!/usr/bin/env python3
"""#1674: the instruments the routing skill calls "never optional" have a step.

The table in plan-pcb-routing says `tests/stress/kicad_drc_compare.py` and
`check_cycles.py` are run, never optional, but no step ran either, and
`check_complete.py` does not. So what only they report never reached a finished
board. They are reported, not gating (#1334's decision). This pins:

  * each door that signs a board off carries a real command or the pointer to it,
    and says the exit code does not cover the channels;
  * the routing skill's step says what each exit code means, including NOT RUN;
  * check_cycles reports routed copper: a part's drawn copper shape, whose
    perimeter is a ring by construction, is not a loop. Otherwise the new step
    would report a loop on an unrouted board.

    python3 tests/test_1674_reported_reads_on_the_done_path.py
"""
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for p in (TESTS, os.path.join(REPO, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
SKILLS = os.path.join(REPO, '.claude', 'skills')
ROUTING = os.path.join(SKILLS, 'plan-pcb-routing', 'SKILL.md')
VERIFIER = os.path.join(SKILLS, 'plan-pcb-placement', 'references', 'verifier.md')
REVIEW = os.path.join(SKILLS, 'review-routed-board', 'SKILL.md')
COMPARE = os.path.join(TESTS, 'stress', 'kicad_drc_compare.py')
CYCLES = os.path.join(REPO, 'py_tools', 'check_cycles.py')
FAILS = []


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def read(path):
    with open(run_utils.evidence(path, 'skill'), encoding='utf-8') as f:
        return f.read()


def t_skills():
    routing, verifier, review = read(ROUTING), read(VERIFIER), read(REVIEW)
    start = routing.index('### Step 6 of the example chain: Verify Results')
    step = routing[start:routing.index('## Step 7:', start)]
    for label, text in (('the routing step', step), ('the verifier', verifier)):
        ok(f'{label} runs the compare',
           'python3 -X utf8 tests/stress/kicad_drc_compare.py ' in text)
        ok(f'{label} runs check_cycles',
           'python3 -X utf8 py_tools/check_cycles.py ' in text)
        for line in ('connwidth=', 'thermal=', 'KICAD-ONLY'):
            ok(f'{label} says to read {line}', line in text)
    ok('the routing step says it is not gating',
       'reported, not gating' in step, step[:300])
    ok('the routing step gives every exit code',
       all(f'| {c} |' in step for c in (0, 1, 2)) and 'NOT RUN' in step)
    ok('the verifier says it is not gating',
       '**report them, not gating**' in verifier)
    ok('the verifier names NOT RUN', 'NOT RUN' in verifier)
    ok('the review skill points at the step, without restating the exit codes',
       '"Verify Results"' in review and 'kicad_drc_compare.py' in review
       and '| 2 | NOT RUN' not in review)
    table = routing[routing.index('| how you use it |'):routing.index('Every runnable tool')]
    ok('the table at the top says where it is run, and who does not list it',
       '"Verify Results"' in table and '`krt_registry`' in table, table[:400])


def t_cycles():
    tmp = tempfile.mkdtemp(prefix='t1674_')
    try:
        pcb = os.path.join(tmp, 'b.kicad_pcb')
        with open(pcb, 'w', encoding='utf-8') as f:
            f.write(BOARD)
        r = run_utils.check(PY + [CYCLES, run_utils.evidence(pcb, 'fixture')], accept=True)
        ok("a part's drawn copper shape is no loop",
           'signal nets with loops: 0;' in r.stdout, r.stdout[-400:])
        with open(pcb, 'w', encoding='utf-8') as f:
            f.write(BOARD.replace(')\n;END', LOOP + ')\n;END'))
        r = run_utils.check(PY + [CYCLES, pcb], refuse='<== LOOPS', code=1)
        ok('control: a routed loop is still reported, exit 1',
           'signal nets with loops: 1;' in r.stdout, r.stdout[-400:])

        from kicad_oracle import find_kicad_cli
        if not find_kicad_cli(warn=False):
            print('  (kicad-cli not found: the compare half is not run)')
            return
        board = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
        stage = os.path.join(tmp, 'read_only.kicad_pcb')
        run_utils.check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), board, stage],
                        accept=True)
        with open(stage, 'rb') as f:
            before = hashlib.sha256(f.read()).hexdigest()
        # This board's own edge items make it exit 1; the exit is not the point.
        r = subprocess.run(PY + [COMPARE, stage], capture_output=True, text=True,
                           encoding='utf-8', errors='replace', cwd=REPO, timeout=600)
        acc = run_utils._accident((r.stdout or '') + (r.stderr or ''))
        if acc:
            raise AssertionError(f'BROKEN TEST: the compare died from {acc!r}\n'
                                 + (r.stdout or '')[-800:])
        with open(stage, 'rb') as f:
            after = hashlib.sha256(f.read()).hexdigest()
        ok('the compare prints the lines the step names',
           ' connwidth=' in r.stdout and ' thermal=' in r.stdout and ' dangling=' in r.stdout,
           r.stdout[-500:])
        ok('the compare leaves the board alone, as the verifier is told', before == after)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


#: one part with a square of drawn net-less copper around its one pad
BOARD = '''(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")
 (general (thickness 1.6))
 (paper "A4")
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
 (setup (pad_to_mask_clearance 0))
 (net 0 "") (net 1 "/A")
  (footprint "T:U1" (layer "F.Cu") (at 120 110)
    (property "Reference" "U1" (at 0 -3 0) (layer "F.SilkS") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
    (attr smd)
    (fp_poly (pts (xy -2 -2) (xy 2 -2) (xy 2 2) (xy -2 2)) (stroke (width 0.1) (type solid)) (fill solid) (layer "F.Cu"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "/A"))
  )
  (gr_line (start 100 100) (end 150 100) (layer "Edge.Cuts") (stroke (width 0.05) (type solid)))
  (gr_line (start 150 100) (end 150 125) (layer "Edge.Cuts") (stroke (width 0.05) (type solid)))
  (gr_line (start 150 125) (end 100 125) (layer "Edge.Cuts") (stroke (width 0.05) (type solid)))
  (gr_line (start 100 125) (end 100 100) (layer "Edge.Cuts") (stroke (width 0.05) (type solid)))
)
;END'''

#: a routed square on net 1
LOOP = ''.join(
    f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.2) (layer "B.Cu") (net 1))\n'
    for x1, y1, x2, y2 in ((130, 105, 135, 105), (135, 105, 135, 110),
                           (135, 110, 130, 110), (130, 110, 130, 105)))


def main():
    t_skills()
    t_cycles()
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
