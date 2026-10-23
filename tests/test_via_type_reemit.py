#!/usr/bin/env python3
"""A re-emitted blind, buried or micro via keeps its type word (found reviewing
#1268).

KiCad loads a via block with no `blind` / `buried` / `micro` word as a THROUGH
via whatever its `(layers ...)` says, and `generate_via_sexpr` wrote no word.
So every site that takes an existing via off the board and puts it back -- the
plane_io restore of a broken net, repair_planes' kept and restored copper, the
writer's re-placed vias, route.py's #666 re-emit -- shipped a microvia as a
through via: a fab change (a laser microvia becomes a drilled hole through the
whole stack) and copper on layers the via never had. repair_planes' writer
also forced every via to `(layers "F.Cu" "B.Cu")`.

A via the tool ADDS is a through via and must still be written exactly as
before: no type word, its own layer pair. Those rows are the controls.

The structural half -- every emit site passes `via_type=`, every copy of an
existing via's layers carries its type -- is in test_749.

    python3 tests/test_via_type_reemit.py
"""
import os
import shutil
import sys
import tempfile

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import Via, extract_vias, parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb, generate_via_sexpr  # noqa: E402

FAILS = []

BOARD = '''(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(4 "In1.Cu" signal)
\t\t(6 "In2.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(25 "Edge.Cuts" user)
\t)
\t(net 0 "")
\t(net 1 "/SIG")
\t(net 2 "/OTHER")
\t(gr_line (start 0 0) (end 40 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e1"))
\t(gr_line (start 40 0) (end 40 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e2"))
\t(gr_line (start 40 40) (end 0 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e3"))
\t(gr_line (start 0 40) (end 0 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e4"))
\t(segment (start 5 5) (end 10 5) (width 0.2) (layer "F.Cu") (net 1) (uuid "s1"))
\t(via micro (at 10 5) (size 0.3) (drill 0.1) (layers "F.Cu" "In1.Cu") (net 1) (uuid "vm"))
\t(segment (start 10 5) (end 15 5) (width 0.2) (layer "In1.Cu") (net 1) (uuid "s2"))
\t(via buried (at 15 5) (size 0.4) (drill 0.2) (layers "In1.Cu" "In2.Cu") (net 1) (uuid "vb"))
\t(segment (start 15 5) (end 20 5) (width 0.2) (layer "In2.Cu") (net 1) (uuid "s3"))
\t(via (at 20 5) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "vt"))
)
'''
N2N = {0: '', 1: '/SIG', 2: '/OTHER'}
NAMES = {'': 0, '/SIG': 1, '/OTHER': 2}


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  [{detail}]' if detail and not ok else ''))
    if not ok:
        FAILS.append(label)


def _by_pos(vias):
    return {(round(v.x, 3), round(v.y, 3)): v for v in vias}


def test_generate_via_sexpr():
    for vt in (None, 'through', '', 'nonsense'):
        s = generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1, via_type=vt)
        check(f"generate_via_sexpr(via_type={vt!r}) writes no type word (control)",
              s.lstrip().startswith('(via\n'), s.splitlines()[0])
    for vt in ('blind', 'buried', 'micro'):
        s = generate_via_sexpr(1, 2, 0.3, 0.1, ['F.Cu', 'In1.Cu'], 1, via_type=vt)
        got = extract_vias(s, None)
        check(f"generate_via_sexpr(via_type={vt!r}) writes `(via {vt}` and "
              f"parses back as {vt}",
              s.lstrip().startswith(f'(via {vt}\n') and len(got) == 1
              and got[0].via_type == vt, s.splitlines()[0])


def test_add_tracks_and_vias(tmp):
    src = os.path.join(tmp, 'in.kicad_pcb')
    dst = os.path.join(tmp, 'out.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    vias = [
        # re-emitted existing copper (route.py's #666 shape)
        {'x': 30.0, 'y': 30.0, 'size': 0.3, 'drill': 0.1,
         'layers': ['F.Cu', 'In1.Cu'], 'net_id': 1, 'via_type': 'micro',
         'tenting_attrs': {}, 'inherit_when_unspecified': True},
        {'x': 31.0, 'y': 30.0, 'size': 0.4, 'drill': 0.2,
         'layers': ['In1.Cu', 'In2.Cu'], 'net_id': 1, 'via_type': 'buried',
         'tenting_attrs': {}, 'inherit_when_unspecified': True},
        # a NEW via: no type key at all
        {'x': 32.0, 'y': 30.0, 'size': 0.6, 'drill': 0.3,
         'layers': ['F.Cu', 'B.Cu'], 'net_id': 1},
    ]
    assert add_tracks_and_vias_to_pcb(src, dst, [], vias, net_id_to_name=N2N)
    with open(dst, encoding='utf-8') as f:
        got = _by_pos(extract_vias(f.read(), NAMES))
    m, b, n = got.get((30.0, 30.0)), got.get((31.0, 30.0)), got.get((32.0, 30.0))
    check("add_tracks_and_vias_to_pcb: a re-emitted microvia stays micro on "
          "F.Cu..In1.Cu", m is not None and m.via_type == 'micro'
          and m.layers == ['F.Cu', 'In1.Cu'], m and (m.via_type, m.layers))
    check("add_tracks_and_vias_to_pcb: a re-emitted buried via stays buried",
          b is not None and b.via_type == 'buried', b and b.via_type)
    check("add_tracks_and_vias_to_pcb: a NEW via is still a plain through via "
          "(control)", n is not None and n.via_type == 'through'
          and n.layers == ['F.Cu', 'B.Cu'], n and (n.via_type, n.layers))


def test_plane_io_restore(tmp):
    """The restore of a net a plane repair broke puts the INPUT's copper back."""
    from plane_io import restore_failed_reroute_nets
    src = os.path.join(tmp, 'r_in.kicad_pcb')
    dst = os.path.join(tmp, 'r_out.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    # The output: the net's copper ripped (no segments, no vias).
    body = BOARD[:BOARD.index('\t(segment')] + ')\n'
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(body)
    restore_failed_reroute_nets(src, dst, [1], [], N2N, 0.6, 0.2)
    with open(dst, encoding='utf-8') as f:
        got = _by_pos(extract_vias(f.read(), NAMES))
    check("plane_io restore: three vias come back", len(got) == 3, len(got))
    m, b, t = got.get((10.0, 5.0)), got.get((15.0, 5.0)), got.get((20.0, 5.0))
    check("plane_io restore: the microvia is restored as a microvia",
          m is not None and m.via_type == 'micro' and m.layers == ['F.Cu', 'In1.Cu'],
          m and (m.via_type, m.layers))
    check("plane_io restore: the buried via is restored as a buried via",
          b is not None and b.via_type == 'buried', b and b.via_type)
    check("plane_io restore: the through via is restored as a through via (control)",
          t is not None and t.via_type == 'through', t and t.via_type)


def test_repair_planes_write(tmp):
    """repair_planes' writer forced every via to F.Cu/B.Cu, untyped."""
    from repair_planes import _write_output
    src = os.path.join(tmp, 'p_in.kicad_pcb')
    dst = os.path.join(tmp, 'p_out.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    vias = [{'x': 30.0, 'y': 35.0, 'size': 0.3, 'drill': 0.1,
             'layers': ['F.Cu', 'In1.Cu'], 'net_id': 2, 'via_type': 'micro'},
            {'x': 31.0, 'y': 35.0, 'size': 0.6, 'drill': 0.3,
             'layers': ['F.Cu', 'B.Cu'], 'net_id': 2, 'via_type': 'through'},
            {'x': 32.0, 'y': 35.0, 'size': 0.6, 'drill': 0.3,
             'layers': ['F.Cu', 'B.Cu'], 'net_id': 2}]
    _write_output(src, dst, [], vias, net_id_to_name=N2N)
    got = _by_pos(parse_kicad_pcb(dst).vias)
    m = got.get((30.0, 35.0))
    check("repair_planes._write_output: a kept microvia keeps its type and span",
          m is not None and m.via_type == 'micro' and m.layers == ['F.Cu', 'In1.Cu'],
          m and (m.via_type, m.layers))
    for x in (31.0, 32.0):
        t = got.get((x, 35.0))
        check(f"repair_planes._write_output: a repair via at x={x} is still a "
              f"through via on F.Cu/B.Cu (control)",
              t is not None and t.via_type == 'through' and t.layers == ['F.Cu', 'B.Cu'],
              t and (t.via_type, t.layers))


def test_output_writer():
    from output_writer import _generate_routing_text
    micro = Via(x=1.0, y=1.0, size=0.3, drill=0.1, layers=['F.Cu', 'In1.Cu'],
                net_id=1, via_type='micro')
    new = Via(x=2.0, y=1.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    text = _generate_routing_text([{'new_segments': [], 'new_vias': [micro, new]}],
                                  [], N2N)
    got = _by_pos(extract_vias(text, NAMES))
    check("output_writer: a re-placed microvia in a result keeps its type",
          got.get((1.0, 1.0)) is not None and got[(1.0, 1.0)].via_type == 'micro')
    check("output_writer: a routed (new) via is still through (control)",
          got.get((2.0, 1.0)) is not None and got[(2.0, 1.0)].via_type == 'through')


def main():
    try:
        test_generate_via_sexpr()
    except TypeError as e:
        check("generate_via_sexpr takes via_type= (it had no way to write a "
              "type word at all)", False, str(e))
    tmp = tempfile.mkdtemp(prefix='tviatype_')
    try:
        test_add_tracks_and_vias(tmp)
        test_plane_io_restore(tmp)
        test_repair_planes_write(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    test_output_writer()
    if FAILS:
        print(f"\n{len(FAILS)} FAILED")
        return 1
    print("\nall passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
