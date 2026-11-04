#!/usr/bin/env python3
"""#1610: the rule roster owes the edge rule for connector-family parts.

The roster called `edge_connector` (and `pins_to_edge`) not applicable unless
some part classified as an edge receptacle or actuator. On a pile, where no
edge is read off a staging pose (#1103), a board whose cable connectors all
need an edge then read "nothing has an edge to claim": right-angle
wire-to-board connectors classify as `connector_affinity`, and a jack named
only by its part number classifies as nothing. The roster now owes the rule
for those parts too, and names them; `part_class` is unchanged (a header can
legitimately sit inside a board).

Cases:
* a board with a `connector_affinity` part and an unclassified part with a
  connector reference OWES `edge_connector`, naming both, and not the `X`
  crystal beside them; `pins_to_edge` applies but stays advisory;
* `edge_connectors[]` entries for them, or a written disposition, answer it;
* a board with no connector-family part still reads not applicable;
* `check_floorplan --intent --plan-only` prints the same OWED line.

    python3 -X utf8 tests/test_1610_connector_edge_roster.py [case ...]
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600

#: ref -> footprint name. J1 reads `connector_affinity`, J2 no class at all
#: (a part number, no pin functions), X1 is a crystal by its footprint name.
CABLE = 'Connector_Molex:Molex_PicoBlade_53261-0471_1x04-1MP_P1.25mm_Horizontal'
PARTS = {'U1': 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
         'J1': CABLE,
         'J2': 'test:CUI_SJ1-42534-SMT',
         'X1': 'Crystal:Crystal_HC49-U_Vertical'}


def _board(path, parts):
    fps = ''.join(
        f'  (footprint "{name}" (layer "F.Cu") (uuid "fp-{r}") '
        f'(at {2 + 4 * i} 2)\n'
        f'    (property "Reference" "{r}" (at 0 0))\n'
        f'    (pad "1" smd rect (at 0 0) (size 0.6 0.8) (layers "F.Cu") '
        f'(net 1 "/A") (uuid "p1-{r}"))\n'
        f'    (pad "2" smd rect (at 1 0) (size 0.6 0.8) (layers "F.Cu") '
        f'(net 2 "/B") (uuid "p2-{r}"))\n'
        '  )\n' for i, (r, name) in enumerate(sorted(parts.items())))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "test")\n'
                 '  (net 0 "")\n  (net 1 "/A")\n  (net 2 "/B")\n'
                 '  (gr_rect (start 0 0) (end 30 10) (layer "Edge.Cuts") '
                 '(uuid "e1"))\n' + fps + ')\n')
    return run_utils.evidence(path, 'the fixture board')


def _roster(path, **raw):
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan as fp
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        it = fp.intent_from_dict(dict(
            {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'},
            **raw), '')
        rows = fp.rule_roster(it, pcb, path)
    return {r['rule']: r for r in rows}, fp.roster_refusal_lines(rows)


def test_connector_family_parts_owe_the_edge_rule():
    with tempfile.TemporaryDirectory() as tmp:
        rows, owed = _roster(_board(os.path.join(tmp, 'b.kicad_pcb'), PARTS))
    e = rows['edge_connector']
    assert e['state'] == 'dark' and e['applicable'], e
    assert e['needs_disposition'], e
    why = e['applicability_reason']
    assert '(J1, J2)' in why and 'X1' not in why and 'U1' not in why, why
    line = [x for x in owed if x.startswith('edge_connector:')]
    assert len(line) == 1, owed
    assert '`edge_connectors[]`' in line[0], line
    assert 'dispositions.rules.edge_connector' in line[0], line
    p = rows['pins_to_edge']
    assert p['applicable'] and not p['gating'] and not p[
        'needs_disposition'], p
    print(f"  PASS: owed, naming J1 and J2, not the crystal ({why})")


def test_a_declaration_or_a_disposition_answers_it():
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(os.path.join(tmp, 'b.kicad_pcb'), PARTS)
        dec, _ = _roster(b, edge_connectors=[
            {'ref': 'J1', 'edge': 'west'}, {'ref': 'J2', 'edge': 'east'}])
        disp, _ = _roster(b, dispositions={'rules': {
            'edge_connector': 'both connectors take cables inside the box'}})
    assert dec['edge_connector']['state'] == 'armed', dec['edge_connector']
    assert not dec['edge_connector']['needs_disposition']
    assert not disp['edge_connector']['needs_disposition'], disp[
        'edge_connector']
    assert disp['edge_connector']['disposition']
    print("  PASS: declared edges arm the rule, a disposition answers it")


def test_no_connector_family_part_is_not_applicable():
    with tempfile.TemporaryDirectory() as tmp:
        rows, _ = _roster(_board(os.path.join(tmp, 'b.kicad_pcb'),
                                 {'U1': PARTS['U1'], 'X1': PARTS['X1']}))
    for name in ('edge_connector', 'pins_to_edge'):
        r = rows[name]
        assert not r['applicable'] and not r['needs_disposition'], r
        assert 'nothing has an edge to claim' in r[
            'applicability_reason'], r
    print("  PASS: a chip and a crystal leave the edge rules not applicable")


def test_the_cli_prints_the_owed_line():
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(os.path.join(tmp, 'b.kicad_pcb'), PARTS)
        intent = os.path.join(tmp, 'i.json')
        with open(intent, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent',
                       'units': 'mm'}, fh)
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), b,
             '--intent', intent, '--plan-only'],
            capture_output=True, text=True, encoding='utf-8', timeout=300)
    out = r.stdout + r.stderr
    line = [x for x in out.splitlines()
            if x.strip().startswith('OWED: edge_connector:')]
    assert len(line) == 1, out[-3000:]
    assert '(J1, J2)' in line[0] and 'X1' not in line[0], line
    assert 'dispositions.rules.edge_connector' in line[0], line
    print("  PASS: --plan-only prints the OWED line naming J1 and J2")


TESTS = [v for k, v in sorted(globals().items())
         if k.startswith('test_') and callable(v)]


if __name__ == '__main__':
    want = sys.argv[1:]
    n = 0
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(t.__name__)
        t()
        n += 1
    print(f"ALL PASS ({n} case(s))")
