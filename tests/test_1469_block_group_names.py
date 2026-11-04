"""#1469: a `blocks[].group` that names a groups-file group.

`blocks[].group` takes only the groups the board defines (KiCad groups,
schematic sheets). A user zoning a group of their groups file writes its name
there and got two `block_unresolved` errors, neither saying what was wrong,
one pointing to a `--list-groups` flag check_floorplan does not have -- and
place_simple placed anyway, reporting the error only afterwards.

Pinned: ONE finding, saying the key takes board groups only and that the
parts go in `refs`, naming a tool that really lists the groups; and
place_simple refusing before any seed, with the groups file's refs ready to
paste.

    python3 tests/test_1469_block_group_names.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                               # noqa: E402
from placement.groups import parse_sources                    # noqa: E402
from run_utils import check as run_check, evidence, tool      # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
FAILS = []
PIC = ['U5', 'U1', 'U6', 'C6', 'C7', 'C8', 'P2', 'P3']


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _doc(blocks):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'blocks': blocks}


def test_one_finding_that_says_what_is_wrong():
    pcb = parse_kicad_pcb(BOARD)
    it = floorplan.intent_from_dict(_doc([{'name': 'pic_zone', 'group': 'pic',
                                           'zone': [172.7, 48.2, 217.8, 130.9]}]),
                                    '')
    _blocks, problems = floorplan.resolve_blocks(it, pcb, parse_sources('auto'))
    hits = [v for v in problems if v.rule == 'block_unresolved']
    check('one block_unresolved, not two', len(hits) == 1,
          [v.message for v in hits])
    msg = hits[0].message if hits else ''
    check('it says the key takes the board\'s own groups only',
          'takes only the groups the board itself defines' in msg, msg)
    check('...and that the parts go in `refs`', '`refs`' in msg, msg)
    named = re.findall(r'`(\w+\.py) [^`]*(--[\w-]+)`', msg)
    ok = bool(named)
    for script, flag in named:
        r = run_check([sys.executable, '-X', 'utf8', tool(script), '--help'],
                      accept=True)
        ok = ok and flag in r.stdout
    check('the tool it names really has the flag it names', ok, named)
    it = floorplan.intent_from_dict(_doc([{'name': 'typo',
                                           'refs': ['ZZ99*']}]), '')
    _b, problems = floorplan.resolve_blocks(it, pcb, parse_sources('auto'))
    check('a block whose refs match nothing keeps its own finding',
          [v.rule for v in problems] == ['block_unresolved']
          and 'matched no footprint' in problems[0].message)


def test_place_simple_refuses_before_any_seed(td):
    board = os.path.join(td, 'b.kicad_pcb')
    shutil.copy(BOARD, board)
    for ext in ('.kicad_pro', '.kicad_prl'):
        src = os.path.splitext(BOARD)[0] + ext
        if os.path.isfile(src):
            shutil.copy(src, os.path.splitext(board)[0] + ext)
    pcb = parse_kicad_pcb(board)
    rest = sorted(r for r in pcb.footprints if r not in PIC)
    groups = os.path.join(td, 'groups.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'groups': [
            {'name': 'pic', 'refs': PIC, 'anchor': 'U5'},
            {'name': 'main', 'refs': rest}]}, fh)
    intent = os.path.join(td, 'intent.json')
    with open(intent, 'w', encoding='utf-8') as fh:
        json.dump(_doc([{'name': 'pic_zone', 'group': 'pic',
                         'zone': [172.7, 48.2, 217.8, 130.9]}]), fh)
    evidence(groups, 'the groups file')
    evidence(intent, 'the intent')
    out = os.path.join(td, 's0.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', tool('place_simple.py'), board,
                   out, '--groups', groups, '--intent', intent, '--seed', '0'],
                  refuse='refused before any seed is placed', code=2)
    text = r.stdout + r.stderr
    check('the groups file\'s refs, ready to paste',
          '"refs": ' + json.dumps(PIC) in text, text[-1500:])
    check('nothing was written', not os.path.exists(out))


def main():
    test_one_finding_that_says_what_is_wrong()
    with tempfile.TemporaryDirectory(prefix='t1469_') as td:
        test_place_simple_refuses_before_any_seed(td)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
