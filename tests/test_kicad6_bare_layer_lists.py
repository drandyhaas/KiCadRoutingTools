#!/usr/bin/env python3
"""KiCad 6 writes a pad's layer list BARE, and every text reader must accept it.

KiCad 7+ quotes each name, `(layers "*.Cu" "*.Mask")`; KiCad 6 (file format
20211014) writes pads as `(layers *.Cu *.Mask)`, and a rule area spanning both
faces as `(layers F&B.Cu)`. The pad parser collected quoted names only, so a
KiCad 6 board parsed with `layers == []` on EVERY pad: duodyne_z80_proc's raw
source (1289 through-hole pads) routed nothing at all, each net failing at its
first step with "8/8 neighbors blocked". The corpus never showed it, because
prep round-trips every board through current pcbnew, which quotes the names;
the GUI never showed it, because it builds pads from pcbnew. Only a CLI run on
a KiCad 6 file did.

Pinned here, one test per reader that took the quoted-only shortcut:
  1. the tokenizer itself, and its spelling-preserving rewrite;
  2. the pad parser -- a KiCad 6 board and its quoted twin parse the same;
  3. end to end -- the KiCad 6 board routes and connects;
  4. the placement writer's side flip -- a bare list is mirrored too, and
     stays bare (it used to leave the pad on the face the part had left);
  5. enable_used_layers -- a layer named only in a bare pad list is found;
  6. plane_io._zone_layer_span -- a bare `(layers F&B.Cu)` is read.

    python3 tests/test_kicad6_bare_layer_lists.py
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

# Two DIP-pitch through-hole parts and one SMD part. SIG runs U1.1 -> U2.1,
# OTHER U1.2 -> U2.2 -> R1.1. The rule area is KiCad 6's two-face spelling.
KICAD6_BOARD = """(kicad_pcb (version 20211014) (generator pcbnew)
  (general (thickness 1.6))
  (paper "A4")
  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (34 "B.Paste" user)
    (35 "F.Paste" user)
    (36 "B.SilkS" user)
    (37 "F.SilkS" user)
    (38 "B.Mask" user)
    (39 "F.Mask" user)
    (44 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "SIG")
  (net 2 "OTHER")
  (footprint "test:DIP2" (layer "F.Cu") (tstamp a1) (at 10 10)
    (fp_text reference "U1" (at 0 -2) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))) (tstamp a2))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers *.Cu *.Mask) (net 1 "SIG") (tstamp a3))
    (pad "2" thru_hole circle (at 0 2.54) (size 1.6 1.6) (drill 0.8) (layers *.Cu *.Mask) (net 2 "OTHER") (tstamp a4))
  )
  (footprint "test:DIP2" (layer "F.Cu") (tstamp b1) (at 30 10)
    (fp_text reference "U2" (at 0 -2) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))) (tstamp b2))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers *.Cu *.Mask) (net 1 "SIG") (tstamp b3))
    (pad "2" thru_hole circle (at 0 2.54) (size 1.6 1.6) (drill 0.8) (layers *.Cu *.Mask) (net 2 "OTHER") (tstamp b4))
  )
  (footprint "test:R0805" (layer "F.Cu") (tstamp c1) (at 20 22)
    (fp_text reference "R1" (at 0 -2) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))) (tstamp c2))
    (pad "1" smd rect (at -1 0) (size 1 1.2) (layers F.Cu F.Paste F.Mask) (net 2 "OTHER") (tstamp c3))
    (pad "2" smd rect (at 1 0) (size 1 1.2) (layers F.Cu F.Paste F.Mask) (tstamp c4))
  )
  (zone (net 0) (net_name "") (layers F&B.Cu) (tstamp d1) (hatch edge 0.508)
    (connect_pads (clearance 0)) (min_thickness 0.254)
    (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))
    (fill (thermal_gap 0.508) (thermal_bridge_width 0.508))
    (polygon (pts (xy 1 1) (xy 3 1) (xy 3 3) (xy 1 3)))
  )
  (gr_line (start 0 0) (end 40 0) (layer "Edge.Cuts") (width 0.1) (tstamp e1))
  (gr_line (start 40 0) (end 40 30) (layer "Edge.Cuts") (width 0.1) (tstamp e2))
  (gr_line (start 40 30) (end 0 30) (layer "Edge.Cuts") (width 0.1) (tstamp e3))
  (gr_line (start 0 30) (end 0 0) (layer "Edge.Cuts") (width 0.1) (tstamp e4))
)
"""


def _quoted_twin(text):
    """The same board as KiCad 7+ writes it: every bare layer-list name quoted."""
    def q(m):
        names = m.group(1).split()
        return '(layers ' + ' '.join('"%s"' % n for n in names) + ')'
    return re.sub(r'\(layers ((?:[^\s"()]+\s*)+)\)', q,
                  text.replace('(version 20211014)', '(version 20221018)'))


def _write(tmp, name, text):
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)
    return path


def test_tokenizer():
    from kicad_parser import layer_list_tokens, map_layer_list_tokens, flip_layer_token
    assert layer_list_tokens(' *.Cu *.Mask)') == ['*.Cu', '*.Mask']
    assert layer_list_tokens(' "*.Cu" "*.Mask")') == ['*.Cu', '*.Mask']
    assert layer_list_tokens(' F.Cu "F.Paste" F.Mask') == ['F.Cu', 'F.Paste', 'F.Mask']
    assert layer_list_tokens(' F&B.Cu)') == ['F&B.Cu']
    assert (map_layer_list_tokens(' F.Cu "F.Paste" F.Mask)', flip_layer_token)
            == ' B.Cu "B.Paste" B.Mask)'), 'each name must keep its own spelling'
    print('  tokenizer: bare, quoted and mixed lists; the rewrite keeps each spelling')


def test_parser_parity(tmp):
    from kicad_parser import parse_kicad_pcb
    v6 = parse_kicad_pcb(_write(tmp, 'v6.kicad_pcb', KICAD6_BOARD))
    v7 = parse_kicad_pcb(_write(tmp, 'v7.kicad_pcb', _quoted_twin(KICAD6_BOARD)))

    def layers_by_pad(pcb):
        return {(ref, p.pad_number): list(p.layers)
                for ref, fp in pcb.footprints.items() for p in fp.pads}
    a, b = layers_by_pad(v6), layers_by_pad(v7)
    assert a == b, f'KiCad 6 and its quoted twin disagree:\n  {a}\n  {b}'
    assert a[('U1', '1')] == ['*.Cu', '*.Mask'], a[('U1', '1')]
    assert a[('R1', '1')] == ['F.Cu', 'F.Paste', 'F.Mask'], a[('R1', '1')]
    print(f'  parser: {len(a)} pads, identical layer lists in both spellings')


def test_routes_end_to_end(tmp):
    board = _write(tmp, 'route_v6.kicad_pcb', KICAD6_BOARD)
    out = os.path.join(tmp, 'route_v6_routed.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route.py'),
                        board, out, '--nets', 'SIG', 'OTHER'],
                       capture_output=True, text=True, encoding='utf-8', cwd=ROOT)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert '"failed": 0' in r.stdout, \
        'a KiCad 6 board must route: ' + r.stdout[-2000:]
    c = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'),
                        out, '--nets', 'SIG', 'OTHER'],
                       capture_output=True, text=True, encoding='utf-8', cwd=ROOT)
    assert c.returncode == 0 and 'ALL NETS FULLY CONNECTED' in c.stdout, \
        c.stdout[-2000:] + c.stderr[-2000:]
    print('  e2e: the KiCad 6 board routes and both nets connect')


def test_flip_mirrors_a_bare_list(tmp):
    from kicad_parser import parse_kicad_pcb, iter_footprint_blocks
    from placement.writer import write_placed_output
    src = _write(tmp, 'flip_v6.kicad_pcb', KICAD6_BOARD)
    dst = os.path.join(tmp, 'flip_v6_out.kicad_pcb')
    fp = parse_kicad_pcb(src).footprints['R1']
    write_placed_output(src, dst, [{
        'reference': 'R1', 'new_x': fp.x, 'new_y': fp.y,
        'new_rotation': fp.rotation, 'new_side': 'B'}])
    text = open(dst, encoding='utf-8').read()
    blocks = {key: text[s:e] for s, e, _t, _raw, key in iter_footprint_blocks(text)}
    r1 = blocks['R1']
    assert r1.count('(layers B.Cu B.Paste B.Mask)') == 2, \
        'both R1 pads must be mirrored to the back, bare as written:\n' + r1
    assert 'F.Paste' not in r1 and 'F.Mask' not in r1, r1
    assert blocks['U1'].count('(layers *.Cu *.Mask)') == 2, \
        'an unflipped part must be left exactly as it was'
    flipped = parse_kicad_pcb(dst).footprints['R1']
    assert all(p.layers == ['B.Cu', 'B.Paste', 'B.Mask'] for p in flipped.pads), \
        [p.layers for p in flipped.pads]
    print('  flip: a bare pad list is mirrored to B.* and stays bare')


def test_enable_used_layers_reads_a_bare_list(tmp):
    from fix_kicad_drc_settings import enable_used_layers
    text = KICAD6_BOARD.replace('    (35 "F.Paste" user)\n', '')
    assert '"F.Paste"' not in text and 'F.Paste' in text, \
        'fixture: F.Paste must be named only by the bare pad lists'
    path = _write(tmp, 'used_v6.kicad_pcb', text)
    added = enable_used_layers(path, verbose=False)
    assert added == ['F.Paste'], added
    print('  enable_used_layers: a layer named only in a bare pad list is re-added')


def test_zone_layer_span_reads_a_bare_list():
    from plane_io import _zone_layer_span
    zone = re.search(r'\(zone .*?\n  \)', KICAD6_BOARD, re.S).group(0)
    assert _zone_layer_span(zone) == {'F&B.Cu'}, _zone_layer_span(zone)
    assert _zone_layer_span(_quoted_twin(zone)) == {'F&B.Cu'}
    print('  plane_io: a bare (layers F&B.Cu) zone span is read')


def main():
    tmp = tempfile.mkdtemp(prefix='kicad6_layers_')
    try:
        test_tokenizer()
        test_parser_parity(tmp)
        test_routes_end_to_end(tmp)
        test_flip_mirrors_a_bare_list(tmp)
        test_enable_used_layers_reads_a_bare_list(tmp)
        test_zone_layer_span_reads_a_bare_list()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('OK: every layer-list reader accepts KiCad 6 bare names')


if __name__ == '__main__':
    main()
