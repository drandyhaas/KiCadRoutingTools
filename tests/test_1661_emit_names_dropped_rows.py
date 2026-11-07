#!/usr/bin/env python3
"""#1661: `--emit-intent` over an existing intent names the rows it drops.

An emit is built from the board, the brief and mechanical.json and never reads
the file it replaces, so an edge row, a `mating_face`, a disposition and a
waiver written into the intent were lost with exit 0 and no line saying so. It
still writes (the emit is what the skill's order relies on) and now names each
row the new document does not carry, whatever `--quiet` says.

Pinned here:
  1. the hand-written rows are named, with their values, and exit stays 0 with
     the new file written;
  2. an edited field on a row the emitter wrote is named as that field;
  3. a re-emit of an unchanged intent prints exactly what the first emit
     printed, on a placed board, on a pile and with --declare-classes;
  4. the comparison is by shape: a key nothing here knows is covered, a row in
     a list with no id key is covered, and rows the new document ADDS are not
     reported;
  5. an old file that is not an intent is said, and replaced;
  6. --quiet does not silence it.

    python3 tests/test_1661_emit_names_dropped_rows.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_tools', 'py_placer', 'py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check, evidence, tool                    # noqa: E402
from check_floorplan import intent_rows_not_carried            # noqa: E402

PY = [sys.executable, '-B', '-X', 'utf8']
FLOORPLAN = tool('check_floorplan.py')
COPY = tool('copy_board.py')
SAY = 'this emit replaces'
PLACED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
PILE_SRC = os.path.join(ROOT, 'tests', 'fixtures', '1603', 'edge_sma.kicad_pcb')
MAKE_UNPLACED = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                             'scripts', 'make_unplaced.py')


def emit(board, out, *flags):
    return check(PY + [FLOORPLAN, board, '--emit-intent', out, *flags],
                 accept=True, allow=('WARNING',))


def said(r):
    return (r.stdout or '') + (r.stderr or '')


def test_hand_rows_named(d):
    board = os.path.join(d, 'b.kicad_pcb')
    check(PY + [COPY, PLACED, board], accept=True)
    out = os.path.join(d, 'intent.json')
    emit(board, out, '--quiet')
    doc = json.load(open(evidence(out), encoding='utf-8'))
    doc['edge_connectors'].append(
        {'ref': 'USB1', 'edge': 'west', 'class': 'edge_receptacle',
         'mating_face': '+x', 'mating_face_source': 'datasheet p.3',
         'overhang_mm': {'min': 0.0, 'max': 1.0}})
    doc.setdefault('dispositions', {}).setdefault('rules', {})[
        'zone_containment'] = 'no region was given'
    doc.setdefault('overlap_waivers', []).append(
        {'pair': ['C1', 'C2'], 'reason': 'two footprints for one site'})
    json.dump(doc, open(out, 'w', encoding='utf-8'), indent=1, sort_keys=True)

    r = emit(board, out, '--quiet')           # 6: --quiet does not silence it
    text = said(r)
    assert SAY in text, text[-800:]
    for name in ('edge_connectors[USB1]', '"mating_face": "+x"',
                 'dispositions.rules.zone_containment = "no region was given"',
                 'overlap_waivers[C1,C2]'):
        assert name in text, f'{name} not named in:\n{text[-800:]}'
    after = json.load(open(out, encoding='utf-8'))
    assert not any(c.get('ref') == 'USB1' for c in after['edge_connectors']), \
        'the emit should still write its own document, not merge'
    assert 'dispositions' not in after and 'overlap_waivers' not in after
    print('  ok: an edge row, a disposition and a waiver are named; exit 0, new file written')


def test_edited_field_named(d):
    board = os.path.join(d, 'pile.kicad_pcb')
    out = os.path.join(d, 'intent.json')
    emit(board, out, '--allow-unplaced', '--quiet', '--declare-classes')
    doc = json.load(open(evidence(out), encoding='utf-8'))
    rows = doc['edge_connectors']
    assert rows, 'the fixture must emit an edge row with --declare-classes, or this tests nothing'
    ref = rows[0]['ref']
    rows[0]['overhang_mm'] = {'min': 33.25, 'max': 34.5}
    json.dump(doc, open(out, 'w', encoding='utf-8'), indent=1, sort_keys=True)
    text = said(emit(board, out, '--allow-unplaced', '--quiet', '--declare-classes'))
    assert f'edge_connectors[{ref}].overhang_mm.min' in text, text[-800:]
    assert '33.25' in text, text[-800:]
    print(f'  ok: an edited field on the emitter\'s own {ref} row is named as that field')


def test_unchanged_says_nothing(d):
    pile = os.path.join(d, 'pile.kicad_pcb')
    placed = os.path.join(d, 'placed.kicad_pcb')
    check(PY + [COPY, PLACED, placed], accept=True)
    for board, flags in ((placed, ()),
                         (pile, ('--allow-unplaced',)),
                         (pile, ('--allow-unplaced', '--declare-classes'))):
        out = os.path.join(d, 'same.json')
        if os.path.exists(out):
            os.remove(out)
        first, second = emit(board, out, *flags), emit(board, out, *flags)
        assert SAY not in said(second), said(second)[-800:]
        assert said(first).replace('\r\n', '\n') == said(second).replace('\r\n', '\n'), (
            f'{os.path.basename(board)} {flags}: an unchanged re-emit printed '
            f'something the first did not')
    print('  ok: an unchanged re-emit prints what the first printed (placed, pile, classes)')


def test_by_shape():
    new = {'kept': 1, 'rows': [{'ref': 'A', 'x': 1}], 'r': [1.0, 2.0], 'names': ['a'],
           'context': {'b': 2}, 'min_reader': 12, 'added': [{'ref': 'Z'}]}
    old = dict(new, never_heard_of={'deep': {'er': 'value'}},
               rows=[{'ref': 'A', 'x': 2, 'extra': 'kept?'}, {'ref': 'B', 'x': 3}],
               r=[1.0, 3.0], names=['a', 'b'], noid=[{'p': 1}, {'p': 2}],
               context={'whatever': 1}, min_reader=99, added=[], empty={},
               nothing=None)
    new_noid = dict(new, noid=[{'p': 2}])
    got = intent_rows_not_carried(old, new_noid)
    want = ['names[b]',
            'never_heard_of.deep.er = "value"',
            'noid[{"p": 1}] = {"p": 1}',
            'r: [1.0, 3.0] -> [1.0, 2.0]',
            'rows[A].extra = "kept?"',
            'rows[A].x: 2 -> 1',
            'rows[B] = {"ref": "B", "x": 3}']
    assert sorted(got) == want, got
    assert intent_rows_not_carried(new, new) == []
    print('  ok: compared by shape: an unknown key, a row with no id, values in full')


def test_unreadable_old_file(d):
    board = os.path.join(d, 'pile.kicad_pcb')
    out = os.path.join(d, 'broken.json')
    with open(out, 'w', encoding='utf-8') as fh:
        fh.write('not json {')
    text = said(emit(board, out, '--allow-unplaced', '--quiet'))
    assert SAY in text and 'could not be read' in text, text[-600:]
    assert json.load(open(out, encoding='utf-8')).get('kind') == 'floorplan-intent'
    print('  ok: an old file that is no intent is said, and replaced')


def main():
    d = tempfile.mkdtemp(prefix='t1661_')
    try:
        env = dict(os.environ)
        r = subprocess.run(PY + [MAKE_UNPLACED, PILE_SRC, os.path.join(d, 'pile.kicad_pcb')],
                           capture_output=True, text=True, env=env)
        evidence(os.path.join(d, 'pile.kicad_pcb'), f'the staged pile ({r.stderr[-300:]})')
        test_by_shape()
        test_hand_rows_named(d)
        test_edited_field_named(d)
        test_unchanged_says_nothing(d)
        test_unreadable_old_file(d)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
