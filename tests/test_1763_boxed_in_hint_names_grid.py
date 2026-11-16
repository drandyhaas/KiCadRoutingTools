#!/usr/bin/env python3
"""#1763: the failure hints say whose copper boxes a net in, and name the
finer grid that puts its off-grid pads on the grid.

The board: kicad_files/qfn_underpad_coupling's 0.5 mm-pitch QFN, eight nets
joining its top-row pads (x = n + 0.25, off the 0.1 grid) to its bottom row,
under a 0.28 mm .kicad_dru clearance floor. At --grid-step 0.1 route.py ends
4/8 routed; the final reconciliation's hint read "the blocking copper belongs
to 2 pre-existing net(s) committed by an earlier run/step ... Retry with
--rip-existing-nets 'N7' 'N2'" -- copper this same run laid, and a rip that
recovers nothing at that grid, while a scoped re-route at 0.05 routes 3 of 4.

Rows:
  1. unit: pads_off_grid -- pads at n + 0.25 / 13.4375 are off a 0.1 grid,
     and 0.05 is the coarsest finer step that puts more of them on it (on
     grid in x); pads on the grid report nothing.
  2. unit: laid_by_this_run reads the invocation's recorded input.
  3. unit: static_boxin_hint records on_grid_step with its verdict and names
     it in the hint.
  4. the route: the run's final hint names the blocking copper as this run's
     own, and the failed-net history names --grid-step 0.05 for the open nets;
     JSON_SUMMARY's boxed_in carries on_grid_step.

    python3 tests/test_1763_boxed_in_hint_names_grid.py
"""
import json
import os
import re
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def unit_rows():
    import routing_diagnostics as rd
    pad = lambda x, y: SimpleNamespace(global_x=x, global_y=y, pad_type='smd')
    off = SimpleNamespace(pads_by_net={1: [pad(9.25, 13.4375), pad(9.25, 8.5625)],
                                       2: [pad(9.2, 13.4), pad(9.3, 8.5)]},
                          board_info=SimpleNamespace(board_bounds=None))
    print("1. pads_off_grid")
    got = rd.pads_off_grid(off, 1, 0.1)
    check("off-grid pads: 0.05 puts them on grid in x", got == (2, 2, 0.05, 'x'), str(got))
    check("on-grid pads report nothing", rd.pads_off_grid(off, 2, 0.1)[2] is None)
    print("2. laid_by_this_run")
    seg = SimpleNamespace(net_id=1)
    board = SimpleNamespace(nets={1: SimpleNamespace(name='A'), 2: SimpleNamespace(name='B')},
                            segments=[seg], vias=[])
    saved = rd._INVOCATION_INPUT_NETS[0]
    try:
        rd.note_invocation_input(board)
        check("a net with no input copper was laid by this run",
              rd.laid_by_this_run(['A', 'B']) == ['B'])
        rd._INVOCATION_INPUT_NETS[0] = None
        check("an unrecorded input claims nothing", rd.laid_by_this_run(['B']) == [])
    finally:
        rd._INVOCATION_INPUT_NETS[0] = saved
    print("3. static_boxin_hint")
    cfg = SimpleNamespace(grid_step=0.1, clearance=0.28, track_width=0.1)
    hint, verdict = rd.static_boxin_hint({'iterations_forward': 2}, cfg, off,
                                         return_verdict=True, net_id=1)
    check("the verdict records the on-grid step",
          (verdict or {}).get('on_grid_step') == 0.05, str(verdict))
    check("the hint names it", '--grid-step 0.05 puts them on grid in x' in hint, hint)


def main():
    unit_rows()
    tmp = tempfile.mkdtemp(prefix='t1763_')
    try:
        base, qfn, out = (os.path.join(tmp, f'{t}.kicad_pcb') for t in ('base', 'qfn', 'out'))
        run_check([sys.executable, '-B', '-X', 'utf8', tool('copy_board.py'),
                   os.path.join(ROOT, 'kicad_files', 'qfn_underpad_coupling.kicad_pcb'),
                   base], accept=True)
        with open(evidence(base, 'board copy'), encoding='utf-8') as fh:
            s = fh.read()
        net_of = {}
        for k, top in enumerate(range(25, 33)):
            net_of[str(top)] = net_of[str(41 - top)] = f"N{k}"

        def add_net(m):
            if m.group(2) not in net_of:
                return m.group(0)
            return m.group(1) + f'\t\t\t(net "{net_of[m.group(2)]}")\n' + m.group(3)
        s = re.sub(r'(\(pad "(\d+)" smd roundrect\n(?:\t\t\t(?!\(uuid)[^\n]*\n)*)(\t\t\t\(uuid)',
                   add_net, s)
        if s.count('(net "N') != 16:
            print("  [BROKEN TEST] the QFN pads were not given their nets")
            return 1
        with open(qfn, 'w', encoding='utf-8') as fh:
            fh.write(s)
        with open(os.path.join(tmp, 'qfn.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "pin_clearance"\n  (constraint clearance (min 0.28mm)))\n')
        print("4. route.py at --grid-step 0.1")
        r = run_check([sys.executable, '-B', '-X', 'utf8', tool('route.py'), qfn, out,
                       '--nets', 'N*', '--track-width', '0.1', '--clearance', '0.28',
                       '--grid-step', '0.1', '--via-size', '0.3', '--via-drill', '0.15'],
                      accept=True)
        log = r.stdout + r.stderr
        line = next((l for l in log.splitlines() if l.startswith('JSON_SUMMARY:')), '')
        summ = json.loads(line.split(':', 1)[1]) if line else {}
        failed = summ.get('failed_single') or []
        if not failed:
            print("  [BROKEN TEST] every net routed at 0.1: the fixture boxes nothing in")
            return 1
        print(f"  open at grid 0.1: {failed}")
        tail = log[log.find('Final reconciliation'):] if 'Final reconciliation' in log else log
        check("the final hint calls the blocking copper this run's own",
              'net(s) this run routed itself' in tail
              and 'committed by an earlier run/step' not in tail)
        check("the failed-net history names --grid-step 0.05",
              re.search(r'Boxed in at this geometry .*re-route this net scoped '
                        r'\(--nets\) at --grid-step 0\.05', log) is not None)
        bx = summ.get('boxed_in') or []
        check("JSON_SUMMARY boxed_in carries on_grid_step 0.05",
              bool(bx) and all(b.get('on_grid_step') == 0.05 for b in bx), str(bx)[:300])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
