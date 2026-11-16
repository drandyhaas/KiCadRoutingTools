"""#1819: an intent link can name the pads it is for.

A `links[]` row is measured courtyard to courtyard, so a capacitor at the far
end of a long header from the two supply pins it serves met a link meant for
those pins. `links[].pads` (`{REF: [pad numbers]}`, the spelling
`proximity[].pads` uses) measures a named part from those pads instead --
in `check_floorplan`'s `link` rule and in `place_simple`'s `within` term, one
number for both.

On the in-repo board below, C5 (the bulk cap, on the supply and ground) is
posed beside J1's supply pins (7, 8) and beside its far pins (1, 2). The
courtyard link reads both poses met; the pad link reads the far one past.
"""

import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan as fp                         # noqa: E402
from run_utils import check, evidence                         # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def tool(*p):
    return os.path.join(ROOT, *p)


def findings(td, board, intent):
    """The link findings check_floorplan grades, read from its --json report."""
    js = os.path.join(td, 'fp.json')
    if os.path.exists(js):
        os.remove(js)
    check(PY + [tool('py_tools', 'check_floorplan.py'), board, '--intent', intent,
                '--json', js], accept=True)
    rep = json.load(open(evidence(js, 'check_floorplan report')))
    return [v for v in rep.get('violations') or ()
            if str(v.get('rule', '')).startswith('link')]


def main(td):
    asbuilt = os.path.join(td, 'asbuilt.kicad_pcb')
    check(PY + [tool('py_router', 'copy_board.py'), BOARD, asbuilt], accept=True)
    base = os.path.join(td, 'base.json')
    check(PY + [tool('py_tools', 'check_floorplan.py'), asbuilt, '--emit-intent', base],
          accept=True)
    doc = json.load(open(evidence(base, 'emitted intent')))
    paths = {}
    for name, extra in (('court', {}), ('pads', {'pads': {'J1': ['7', '8']}}),
                        ('bad', {'pads': {'J1': ['7', '99']}})):
        d = dict(doc)
        d['links'] = [dict({'name': 'bulk', 'a': ['C5'], 'b': ['J1'], 'max_mm': 2}, **extra)]
        d['min_reader'] = max(int(d.get('min_reader') or 0), fp.LINK_PADS_READER)
        paths[name] = os.path.join(td, f'intent_{name}.json')
        json.dump(d, open(paths[name], 'w'))
    near = os.path.join(td, 'near.kicad_pcb')
    far = os.path.join(td, 'far.kicad_pcb')
    check(PY + [tool('py_placer', 'place_pose.py'), asbuilt, near,
                'set', 'C5', '87.4', '74.5', '--rot', '270'], accept=True)
    check(PY + [tool('py_placer', 'place_pose.py'), asbuilt, far,
                'set', 'C5', '87.4', '59.2', '--rot', '270'], accept=True)

    # controls: the courtyard row measures as before
    got = findings(td, asbuilt, paths['court'])
    ok('courtyard link: as built warns at 5.01 mm',
       [(v['rule'], abs(v['measured'].get('gap_mm') - 5.005) < 0.01) for v in got]
       == [('link', True)]
       and 'basis' not in got[0]['measured'], got)
    ok('courtyard link: near is met', findings(td, near, paths['court']) == [])
    ok('courtyard link: far is met too (the defect the pads fix)',
       findings(td, far, paths['court']) == [])

    # the pad row
    got = findings(td, far, paths['pads'])
    ok('pad link: far is past, one link finding', [v['rule'] for v in got] == ['link'], got)
    if got:
        m = got[0]
        ok('the finding names the pads and its basis',
           'J1 pads 7, 8' in m['message']
           and m['measured'].get('basis') == {'C5': 'courtyard', 'J1': 'pads'}, m)
        ok('the gap is measured to pads 7 and 8 (about 12.5 mm)',
           abs(m['measured']['gap_mm'] - 12.5) < 0.1, m['measured'])
    ok('pad link: near is met', findings(td, near, paths['pads']) == [])
    got = findings(td, near, paths['bad'])
    ok('a pad number the part does not have is link_unresolved',
       [v['rule'] for v in got] == ['link_unresolved'] and '99' in got[0]['message'], got)

    # the loader refuses a pad list for a part neither side names
    try:
        fp.intent_from_dict({'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
                             'links': [{'a': ['C5'], 'b': ['J1'], 'max_mm': 2,
                                        'pads': {'U1': ['1']}}]}, 't.json')
        why = None
    except fp.IntentError as e:
        why = str(e)
    ok('a pad list for a part the link does not name is refused',
       why is not None and 'neither side' in why, why)

    # the placer reads the same number: C5 free, everything else locked
    farlock = os.path.join(td, 'farlock.kicad_pcb')
    others = [r for r in parse_kicad_pcb(far).footprints if r != 'C5']
    check(PY + [tool('py_placer', 'place_pose.py'), far, farlock, 'lock', *others],
          accept=True)
    groups = os.path.join(td, 'groups.json')
    json.dump({'schema': 1, 'groups': [{'name': 'cap', 'refs': ['C5']}]}, open(groups, 'w'))
    placed = os.path.join(td, 'placed.kicad_pcb')
    js = os.path.join(td, 'placed.json')
    check(PY + [tool('py_placer', 'place_simple.py'), farlock, placed, '--groups', groups,
                '--intent', paths['pads'], '--seed', '0', '--json', js], accept=True)
    rep = json.load(open(evidence(js, 'place_simple report')))
    links = rep.get('links') or []
    ok('place_simple reports the link with its pads',
       len(links) == 1 and links[0].get('pads') == {'J1': ['7', '8']}, links)
    ok('place_simple pulls C5 to the pads, within max_mm',
       bool(links) and links[0]['gap_mm'] <= 2.0 + 1e-6, links)
    d = json.load(open(paths['pads']))
    d['links'][0]['max_mm'] = 0.0
    zero = os.path.join(td, 'intent_zero.json')
    json.dump(d, open(zero, 'w'))
    got = findings(td, placed, zero)
    ok('the grade reads the gap place_simple reports',
       bool(links) and len(got) == 1
       and abs(got[0]['measured']['gap_mm'] - links[0]['gap_mm']) < 2e-3,
       (got, links))


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as td:
        main(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
