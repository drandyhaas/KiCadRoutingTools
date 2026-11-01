#!/usr/bin/env python3
"""#1562: bga_fanout --immovable-refs reads refs separated by commas as well
as spaces, and refuses a ref that is not a part on the board.

place_fanout_clearance prints the parts it could not clear as
`Unresolved (need manual attention): C17, C31, C6`, and that list pasted
after --immovable-refs used to hold `C17,` and `C31,` -- names no part has --
while `C17,C31,C6` held one part of that whole name. A ref that is on no
part was held too, and the log said so.

1. The shared resolver (bga_fanout.geometry.resolve_immovable_refs), which
   the engine calls, so the GUI's Hold parts field reads alike:
   space, comma and comma-space forms give the same refs; an unknown ref
   raises ValueError naming it (and only it); none gives [].
2. The engine: generate_bga_fanout refuses an unknown ref, and holds the
   parts a comma form names (the held set it records).
3. The CLI refuses an unknown ref before any work, naming it, in each form.

    python3 tests/test_1562_immovable_refs.py
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, HERE)

from bga_fanout import generate_bga_fanout  # noqa: E402
from bga_fanout.geometry import resolve_immovable_refs  # noqa: E402
from run_utils import check as run_check, evidence  # noqa: E402
from synth import make_pad, make_net, make_pcb  # noqa: E402
from kicad_parser import BoardInfo, Footprint  # noqa: E402

RUN_ALL_TIMEOUT = 300
FAILS = []
BOARD = os.path.join(ROOT_DIR, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  [{detail}]' if not ok else ''))
    if not ok:
        FAILS.append(label)


def fixture():
    """A 4x4 1 mm BGA U1 and three 2-pad parts C17, C31, C6 beside it."""
    pads, nets, byid = [], {}, {}
    k = 0
    for r in range(4):
        for c in range(4):
            k += 1
            lx, ly = c - 1.5, r - 1.5
            p = make_pad(k, 10 + lx, 10 + ly, ref='U1', num=f'{chr(65 + r)}{c + 1}',
                         net_name=f'N{k}', size_x=0.4, size_y=0.4, shape='circle',
                         layers=('F.Cu',), local_x=lx, local_y=ly)
            pads.append(p)
            nets[k] = make_net(k, f'N{k}', pads=[p])
            byid[k] = [p]
    fps = {'U1': Footprint(reference='U1', footprint_name='test:BGA16', x=10, y=10,
                           rotation=0.0, layer='F.Cu', pads=pads)}
    for i, ref in enumerate(('C17', 'C31', 'C6')):
        cp = []
        for j in (1, 2):
            nid = 100 + 2 * i + j
            p = make_pad(nid, 16 + 0.5 * j, 6 + 2 * i, ref=ref, num=str(j),
                         net_name=f'M{nid}', size_x=0.3, size_y=0.3, layers=('F.Cu',),
                         local_x=0.5 * j - 0.75, local_y=0.0)
            cp.append(p)
            nets[nid] = make_net(nid, f'M{nid}', pads=[p])
            byid[nid] = [p]
        fps[ref] = Footprint(reference=ref, footprint_name='test:C0402', x=16.75,
                             y=6 + 2 * i, rotation=0.0, layer='F.Cu', pads=cp)
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 30.0, 30.0))
    return make_pcb(nets=nets, footprints=fps, pads_by_net=byid, board_info=bi,
                    zones=[])


def refused(pcb, refs):
    try:
        resolve_immovable_refs(pcb, refs)
    except ValueError as exc:
        return str(exc)
    return None


def test_resolver():
    print("resolver: the forms a ref list arrives in, and an unknown ref")
    pcb = fixture()
    want = ['C17', 'C31', 'C6']
    for label, refs in (('space', ['C17', 'C31', 'C6']),
                        ('comma', ['C17,C31,C6']),
                        ('comma-space (the clearance step\'s printed form)',
                         ['C17,', 'C31,', 'C6']),
                        ('one quoted comma-space argument', ['C17, C31, C6'])):
        try:
            got = resolve_immovable_refs(pcb, refs)
        except ValueError as exc:
            got = f'refused: {exc}'
        check(f"{label} form reads as {want}", got == want, got)
    check("a repeated ref is held once",
          resolve_immovable_refs(pcb, ['C17', 'C17,C31']) == ['C17', 'C31'])
    check("none gives []", resolve_immovable_refs(pcb, None) == []
          and resolve_immovable_refs(pcb, []) == [])
    msg = refused(pcb, ['NOPE1'])
    check("a ref on no part is refused, naming it",
          msg is not None and 'not on the board: NOPE1' in msg, msg)
    msg = refused(pcb, ['C17,NOPE1,', 'C6'])
    check("in a mixed list only the unknown ref is named",
          msg is not None and msg.endswith('not on the board: NOPE1'), msg)


def test_engine():
    print("engine: generate_bga_fanout checks the refs it is handed")
    pcb = fixture()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            generate_bga_fanout(pcb.footprints['U1'], pcb, net_filter=['N*'],
                                layers=['F.Cu', 'B.Cu'], plane_drop='off',
                                immovable_refs=['NOPE1'])
        err = None
    except ValueError as exc:
        err = str(exc)
    check("an unknown ref is refused by the engine (the GUI's path)",
          err is not None and 'NOPE1' in err, err)
    pcb = fixture()
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        generate_bga_fanout(pcb.footprints['U1'], pcb, net_filter=['N*'],
                            layers=['F.Cu', 'B.Cu'], plane_drop='off',
                            immovable_refs=['C17,', 'C31,', 'C6'])
    held = getattr(pcb, '_fanout_immovable_refs', None)
    check("the comma-space form holds the three parts",
          held == frozenset({'C17', 'C31', 'C6'}), held)
    check("and the log names them as the board spells them",
          'Holding 3 part(s) immovable: C17, C31, C6' in out.getvalue(),
          [l for l in out.getvalue().splitlines() if 'Holding' in l])


def test_cli():
    print("CLI: an unknown ref is refused before any work")
    evidence(BOARD)
    cli = [sys.executable, os.path.join(ROOT_DIR, 'py_router', 'bga_fanout.py'),
           BOARD, '--output', os.devnull]
    for refs in (['NOPE1'], ['R1,NOPE1'], ['R1,', 'NOPE1']):
        try:
            run_check(cli + ['--immovable-refs', *refs],
                      refuse='not on the board: NOPE1', code=1, timeout=300)
            check(f"--immovable-refs {' '.join(refs)} refused naming NOPE1", True)
        except AssertionError as exc:
            check(f"--immovable-refs {' '.join(refs)} refused naming NOPE1", False,
                  str(exc)[-400:])


if __name__ == '__main__':
    test_resolver()
    test_engine()
    test_cli()
    print()
    if FAILS:
        print(f"FAILED {len(FAILS)}: {FAILS}")
        sys.exit(1)
    print("ALL PASS")
