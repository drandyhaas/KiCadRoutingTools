"""The pair-unit lap routes its staged board at the board's hole floor.

The lap copies the written board to ``<out>.pairlap.kicad_pcb`` and routes
that copy in a fresh batch_route. The copy carried no ``.kicad_pro``, so
resolve_hole_clearance read 0 there and the #1038 via-copper keep-out around
NPTH holes was off: the lap placed a 0.6/0.3 via 0.16 mm off an NPTH wall on
a board declaring min_hole_clearance 0.25.

Checks:
1. route._stage_lap_board exists (base: copies the board only, inline).
2. The staged copy resolves the source's declared hole floor (0.25).
3. Every sibling (.kicad_dru, .design-brief.json, ...) is staged too.
4. A source with no project stages the board alone (no error).
"""
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, ROOT)

FIX = os.path.join(HERE, 'fixtures', '1847')

fails = 0


def check(name, ok, detail=''):
    global fails
    print(f"{'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail else ''))
    if not ok:
        fails += 1


def main():
    import route
    from kicad_parser import parse_kicad_pcb
    from obstacle_map import resolve_hole_clearance

    stage = getattr(route, '_stage_lap_board', None)
    check('1. route._stage_lap_board exists', stage is not None)
    if stage is None:
        return
    d = tempfile.mkdtemp(prefix='t1038_')
    try:
        src = os.path.join(d, 'routed.kicad_pcb')
        shutil.copyfile(os.path.join(FIX, 'power_drop.kicad_pcb'), src)
        with open(os.path.join(FIX, 'power_drop.kicad_pro'), encoding='utf-8') as f:
            pro = json.load(f)
        pro['board']['design_settings']['rules']['min_hole_clearance'] = 0.25
        with open(os.path.join(d, 'routed.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump(pro, f, indent=2)
        with open(os.path.join(d, 'routed.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write('(version 1)\n')
        with open(os.path.join(d, 'routed.design-brief.json'), 'w',
                  encoding='utf-8') as f:
            json.dump({'keep_away': []}, f)
        src_floor = resolve_hole_clearance(parse_kicad_pcb(src), None)

        dst = os.path.join(d, 'routed.pairlap.kicad_pcb')
        stage(src, dst)
        got = resolve_hole_clearance(parse_kicad_pcb(dst), None)
        check('2. staged copy keeps the declared hole floor',
              abs(got - 0.25) < 1e-9 and abs(src_floor - 0.25) < 1e-9,
              f'source {src_floor}, staged {got}')
        from copy_board import SIBLING_EXTS
        missing = [e for e in ('.kicad_dru', '.design-brief.json')
                   if not os.path.exists(os.path.join(d, 'routed.pairlap' + e))]
        check('3. siblings staged (.kicad_dru, .design-brief.json)',
              not missing and '.design-brief.json' in SIBLING_EXTS,
              f'missing {missing}')

        bare = os.path.join(d, 'bare.kicad_pcb')
        shutil.copyfile(src, bare)
        dst2 = os.path.join(d, 'bare.pairlap.kicad_pcb')
        stage(bare, dst2)
        check('4. no project: board staged alone',
              os.path.exists(dst2)
              and not os.path.exists(os.path.join(d, 'bare.pairlap.kicad_pro')))
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    main()
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
