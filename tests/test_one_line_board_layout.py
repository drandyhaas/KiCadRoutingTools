#!/usr/bin/env python3
"""A board written on one line parses exactly like the same board in KiCad's
multi-line layout.

SolderCAD's file validator writes boards on one line
(``...)(zone(net 1)...(layer "F.Cu" )...``), and KiCad reads that layout. This
parser's readers were written against the multi-line one: on one line the
zone iterator and the ``(layers`` table found nothing, and the unprettified
``(layer "F.Cu" )`` defeated the tight field patterns. Measured on a routed
board: zones 1 -> 0, copper layers 2 -> 0, bounds/outline gone, a footprint's
copper fp_poly dropped -- check_connected then called the poured GND broken.
`read_board_text` now reflows such a board before any reader sees it.

Checks:
  1. Each fixture, rewritten the way SolderCAD writes it, parses to the same
     nets, footprints, pads, copper (incl. footprint copper), vias, zones,
     keepouts, layers, stackup and outline as the original -- and the fields
     the one-line layout used to lose are non-empty, so equality means
     something.
  2. The reflow changes whitespace only: the token sequence is identical.
  3. A board that already has its lines is returned unchanged.

    python3 tests/test_one_line_board_layout.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import (_TOKEN_POS_RE, parse_kicad_pcb,  # noqa: E402
                          reflow_one_line_board)

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def one_line(text):
    """The board as SolderCAD's validator writes it: no line breaks or
    indentation, no space around parens, and KiCad's unprettified space
    before a string's closing paren."""
    text = re.sub(r'\s*\n\s*(?=[()])', '', text)
    text = re.sub(r'\s*\n\s*', ' ', text)
    return text.replace('")', '" )')


def summary(d):
    r = lambda v: round(v, 4)  # noqa: E731
    bi = d.board_info
    return {
        'nets': sorted((i, n.name) for i, n in d.nets.items()),
        'footprints': sorted((ref, r(f.x), r(f.y), r(f.rotation), f.layer, len(f.pads))
                             for ref, f in d.footprints.items()),
        'pads': sorted((p.component_ref, str(p.pad_number), p.net_id, r(p.global_x), r(p.global_y))
                       for ps in d.pads_by_net.values() for p in ps),
        'segments': sorted((r(s.start_x), r(s.start_y), r(s.end_x), r(s.end_y), s.layer, s.net_id)
                           for s in d.segments),
        'vias': sorted((r(v.x), r(v.y), v.net_id) for v in d.vias),
        'zones': sorted((z.net_id, z.layer, len(z.polygon)) for z in d.zones),
        'keepouts': len(bi.keepouts),
        'layers': sorted(bi.layers.items()),
        'copper_layers': bi.copper_layers,
        'stackup': [(s.name, s.layer_type) for s in bi.stackup],
        'board_outline': len(bi.board_outline),
    }


# fixture -> fields that must be non-empty in the one-line parse (the ones
# the layout used to lose).
FIXTURES = {
    # 2-layer, two GND pours, board outline, stackup
    'interf_u_routed.kicad_pcb': ('zones', 'copper_layers', 'stackup', 'board_outline'),
    # 4-layer with a keep-out rule area
    'glasgow_revC.kicad_pcb': ('keepouts', 'copper_layers', 'stackup'),
    # SOT-89 tab drawn as a copper fp_poly inside the footprint
    'esp_prog.kicad_pcb': ('segments', 'copper_layers'),
}


def parse_quietly(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def main():
    tmp = tempfile.mkdtemp(prefix='one_line_board_')
    try:
        for name, nonempty in FIXTURES.items():
            print(f"1. {name}: one-line layout parses like the original")
            src = os.path.join(ROOT, 'kicad_files', name)
            text = open(src, encoding='utf-8').read()
            flat = os.path.join(tmp, name)
            with open(flat, 'w', encoding='utf-8') as f:
                f.write(one_line(text))
            pro = src[:-len('.kicad_pcb')] + '.kicad_pro'   # #441: never strand the project
            if os.path.isfile(pro):
                shutil.copyfile(pro, flat[:-len('.kicad_pcb')] + '.kicad_pro')
            check('fixture really is one line', one_line(text).count('\n') == 0)
            want, got = summary(parse_quietly(src)), summary(parse_quietly(flat))
            for field in want:
                check(f'{field} identical', want[field] == got[field],
                      '' if want[field] == got[field] else f'{len(want[field]) if isinstance(want[field], list) else want[field]} -> '
                      f'{len(got[field]) if isinstance(got[field], list) else got[field]}')
            for field in nonempty:
                check(f'{field} present in the one-line parse', bool(got[field]))

        print("2. the reflow changes whitespace only")
        text = open(os.path.join(ROOT, 'kicad_files', 'interf_u_routed.kicad_pcb'), encoding='utf-8').read()
        flat = one_line(text)
        reflowed, changed = reflow_one_line_board(flat)
        check('one-line board is reflowed', changed and reflowed.count('\n') > 1000)
        check('token sequence unchanged', _TOKEN_POS_RE.findall(reflowed) == _TOKEN_POS_RE.findall(flat))

        print("3. a multi-line board is left alone")
        same, changed = reflow_one_line_board(text)
        check('multi-line board returned unchanged', not changed and same is text)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    if failures:
        print(f"FAILED: {len(failures)} check(s)")
        sys.exit(1)
    print("All checks passed.")


if __name__ == '__main__':
    main()
