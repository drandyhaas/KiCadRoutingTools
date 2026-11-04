"""#1500: `heights[].under_cover` lets a part reached from above stand under a
cover, judged by its declared height.

A connector, switch or test point is "reached from above", so the height
model refused it under any declared cover, whatever height it declared: an
M.2 socket under an SO-DIMM module could not be declared at all. A
`heights[]` row with `under_cover: true` and its `height_mm` now has that one
part judged under a cover by that height; every reader names the override,
and an undeclared connector stays refused.

esp_prog: a module declared on CON2 (8.0 mm up), turned over CON1, which is
declared 1.0 mm tall.

    python3 tests/test_1500_under_cover.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

from placement import floorplan as fp  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def intent(underside=8.0, under_cover=None):
    con1 = {'ref': 'CON1', 'height_mm': 1.0, 'source': 'synthetic: a 1.0 mm SMD connector'}
    if under_cover is not None:
        con1['under_cover'] = under_cover
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': fp.UNDER_COVER_READER if under_cover else fp.COVERS_READER,
            'covers': [{'connector': 'CON2', 'side': 'F', 'axis_deg': 90, 'width_mm': 2.9,
                        'offset_mm': -8.08, 'length_mm': 11, 'underside_mm': underside,
                        'removable': True, 'source': 'synthetic: a module, stacking socket'}],
            'heights': [con1,
                        {'ref': 'R*', 'height_mm': 0.4, 'source': 'synthetic'},
                        {'ref': 'C*', 'height_mm': 0.9, 'source': 'synthetic'},
                        {'ref': 'Q*', 'height_mm': 1.1, 'source': 'synthetic'},
                        {'ref': 'U*', 'height_mm': 1.6, 'source': 'synthetic'},
                        {'ref': 'Y1', 'height_mm': 0.8, 'source': 'synthetic'}]}


def refusals():
    print('1. the key is checked at load')
    for label, row in (('a non-boolean', {'ref': 'J1', 'height_mm': 3.0, 'under_cover': 'yes',
                                          'source': 's'}),
                       ('without height_mm', {'ref': 'J1', 'underside_mm': 3.0,
                                              'under_cover': True, 'source': 's'})):
        try:
            fp.intent_from_dict({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                                 'min_reader': fp.UNDER_COVER_READER, 'heights': [row]}, '')
            check(f'{label} is refused', False, 'loaded')
        except fp.IntentError as ex:
            check(f'{label} is refused', 'under_cover' in str(ex), str(ex))


def arms(td):
    print('2. CON1 under the module on CON2')
    board = os.path.join(td, 'b.kicad_pcb')
    subprocess.run(PY + [os.path.join('py_router', 'copy_board.py'),
                         os.path.join('kicad_files', 'esp_prog.kicad_pcb'), board],
                   capture_output=True, cwd=REPO)
    run_utils.evidence(board)
    for name, doc, want in (('no key', intent(), 4), ('under_cover', intent(under_cover=True), 0),
                            ('under_cover, 0.8 mm underside', intent(0.8, True), 4)):
        path = os.path.join(td, name.replace(' ', '_').replace(',', '') + '.json')
        with open(path, 'w') as fh:
            json.dump(doc, fh)
        out = os.path.join(td, 'out.kicad_pcb')
        r = subprocess.run(PY + [os.path.join('py_placer', 'place_pose.py'), board, out,
                                 'rotate', 'CON2', '0', '--intent', path],
                           capture_output=True, text=True, cwd=REPO)
        refused = 'height_pairs new: CON1/CON2' in r.stdout + r.stderr
        check(f'{name}: exit {want}' + (', CON1 refused under the body' if want else ''),
              r.returncode == want and refused == bool(want),
              f'rc {r.returncode}: {(r.stdout + r.stderr)[-300:]}')
        if name == 'under_cover':
            a = subprocess.run(PY + [os.path.join('py_tools', 'check_assembly.py'), out,
                                     '--intent', path], capture_output=True, text=True, cwd=REPO)
            check('check_assembly names the override',
                  'CON1: declared under_cover at height_mm 1: judged under a cover' in a.stdout,
                  a.stdout[-400:])


def brief():
    print('3. a brief row carrying it compiles with min_reader 20')
    from placement import design_brief as db
    b = db.brief_from_dict({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                            'heights': [{'ref': 'J7', 'height_mm': 3.2, 'under_cover': True,
                                         'source': 'its datasheet'}]}, '')
    doc, _report = db.compile_brief(b, refs_known=False)
    check('min_reader is 20', doc.get('min_reader') == fp.UNDER_COVER_READER, json.dumps(doc)[:300])


def main():
    refusals()
    td = tempfile.mkdtemp(prefix='krt1500_')
    try:
        arms(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    brief()
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
