"""#1477: `links[]` and `chains[]`, two sets of parts within a distance,
their groups unmerged.

A `proximity` row across two groups moves its subject into the partner's
group (#1467), and a groups-file `near` pulls until the groups touch, several
of them voting (#1471). A link holds two sets of parts within `max_mm`,
courtyard to courtyard (Manhattan), and a chain links consecutive stages.
`link` grades it; `place_simple` prices the same gap past `max_mm` at
`LINK_WEIGHT` per mm.
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
from test_1212_container_pins import _frame_board            # noqa: E402

PILE = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _intent(**extra):
    d = {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
         'min_reader': fp.LINKS_READER}
    d.update(extra)
    return fp.intent_from_dict(d, 't.json')


def _refused(**extra):
    try:
        _intent(**extra)
    except fp.IntentError as e:
        return str(e)
    return None


def test_load():
    it = _intent(links=[{'name': 'l', 'a': ['A'], 'b': ['B*'], 'max_mm': 4}],
                 chains=[{'name': 'c', 'stages': [['J1'], ['U1', 'C1'], ['U2']],
                          'max_mm': [5, 10]}])
    got = [(ln['name'], ln['a'], ln['b'], ln['max_mm'], ln['source']) for ln in it.links]
    ok('a chain is links between consecutive stages', got == [
        ('l', ('A',), ('B*',), 4.0, 'links[0]'),
        ('c:0', ('J1',), ('U1', 'C1'), 5.0, 'chains[0]'),
        ('c:1', ('U1', 'C1'), ('U2',), 10.0, 'chains[0]')], got)
    one = _intent(chains=[{'stages': [['A'], ['B'], ['C']], 'max_mm': 3}]).links
    ok('one max_mm serves every link', [ln['max_mm'] for ln in one] == [3.0, 3.0])
    ok('a chain of one stage is refused',
       'two or more' in (_refused(chains=[{'stages': [['A']], 'max_mm': 1}]) or ''))
    ok('one max_mm per link, or one',
       'one per link' in (_refused(chains=[{'stages': [['A'], ['B'], ['C']],
                                             'max_mm': [1]}]) or ''))
    ok('a link needs both sides',
       'non-empty' in (_refused(links=[{'a': ['A'], 'b': [], 'max_mm': 1}]) or ''))
    ok('a name is one link',
       'repeats' in (_refused(links=[{'name': 'x', 'a': ['A'], 'b': ['B'], 'max_mm': 1},
                                     {'name': 'x', 'a': ['A'], 'b': ['C'], 'max_mm': 1}]) or ''))


def test_grade(td):
    # A and B 1 mm squares at x 10 and 18: 6 mm apart, courtyard to courtyard
    path = _frame_board(td, [('A', 10, 15, 1.0), ('B', 18, 15, 1.0)])
    pcb = parse_kicad_pcb(path)

    def found(max_mm, b=('B',)):
        r = fp.grade(_intent(links=[{'name': 'l', 'a': ['A'], 'b': list(b),
                                     'max_mm': max_mm}]), pcb, path)
        return sorted((v.rule, v.severity, v.measured.get('gap_mm'))
                      for v in r.errors + r.warnings if v.rule.startswith('link'))
    ok('past max_mm: one warn, the gap named', found(5) == [('link', fp.WARN, 6.0)],
       found(5))
    ok('within it: nothing', found(6) == [])
    ok('a side matching no part is link_unresolved',
       found(5, ('Z*',)) == [('link_unresolved', fp.WARN, None)], found(5, ('Z*',)))


def test_place(td):
    groups = {'schema': 1, 'groups': [
        {'name': 'usb', 'refs': ['USB1', 'U2', 'C1', 'C3'], 'anchor': 'USB1'},
        {'name': 'core', 'refs': ['U1', 'Y1', 'C2', 'C4'], 'anchor': 'U1'},
        {'name': 'auto', 'refs': ['Q1', 'Q2', 'R1', 'R2'], 'anchor': 'Q1'},
        {'name': 'ports', 'refs': ['CON1', 'CON2', 'R3', 'R4', 'Ref*', 'Ref*~2'],
         'anchor': 'CON2'}]}
    gpath = os.path.join(td, 'groups.json')
    json.dump(groups, open(gpath, 'w'))
    doc = fp.emit_intent(parse_kicad_pcb(PILE), PILE)
    doc['links'] = [{'name': 'auto_by_usb', 'a': ['Q1', 'Q2', 'R1', 'R2'],
                     'b': ['USB1'], 'max_mm': 1.0}]
    doc['min_reader'] = max(fp.LINKS_READER, doc.get('min_reader') or 0)
    ipath = os.path.join(td, 'intent.json')
    json.dump(doc, open(ipath, 'w'))
    out = os.path.join(td, 'placed.kicad_pcb')
    js = os.path.join(td, 'placed.json')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_placer', 'place_simple.py'), PILE, out,
                        '--groups', gpath, '--intent', ipath, '--seed', '0', '--json', js],
                       capture_output=True, text=True, cwd=ROOT)
    ok('place_simple ran', r.returncode in (0, 4) and os.path.exists(js),
       r.stderr[-400:] or r.stdout[-400:])
    if not os.path.exists(js):
        return
    rep = json.load(open(js))
    links = {ln['link']: ln for ln in rep.get('links') or ()}
    ok('the link is priced and reported', 'auto_by_usb' in links, rep.get('links'))
    if 'auto_by_usb' not in links:
        return
    gap = links['auto_by_usb']['gap_mm']
    ok('the anneal holds it within max_mm', gap <= 1.0 + 1e-6, gap)
    names = [g if isinstance(g, str) else (g.get('name') if isinstance(g, dict) else g[0])
             for g in rep.get('groups') or ()]
    ok('the groups are not merged', 'auto' in names and 'usb' in names, names)
    graded = fp.grade(fp.load_intent(ipath), parse_kicad_pcb(out), out)
    found = [v for v in graded.errors + graded.warnings if v.rule == 'link']
    ok('the grade reads the same gap', not found, [v.message for v in found])


if __name__ == '__main__':
    test_load()
    with tempfile.TemporaryDirectory() as td:
        test_grade(td)
    with tempfile.TemporaryDirectory() as td:
        test_place(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
