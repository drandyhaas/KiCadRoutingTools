#!/usr/bin/env python3
"""A KiCad 6-era board parses the way pcbnew loads it.

pcbnew converts six KiCad 6-era conventions on load, and the GUI, which
builds from pcbnew, has always seen the converted board. The text parser kept
the file's spelling. Parsing all 89 KiCad 6 corpus sources raw and after a
pcbnew round trip found each one; the cutoffs are KiCad 10.0.0's own
(pcb_io_kicad_sexpr_parser.cpp, pcb_io_kicad_sexpr.h, kiid.cpp,
string_utils.cpp):

  1. A zone on `(layers F&B.Cu)` or `(layers *.Cu)` is F.Cu + B.Cu, or every
     copper layer. Kept literally it became ONE zone on a layer nothing
     recognizes, and the pour vanished from the model (27 corpus boards).
  2. Up to version 20210925 an arc is (start CENTER) (end ARC-START)
     (angle SWEEP). Every reader matched only start/mid/end, so legacy arcs
     were dropped: rounded outlines lost their corners, and one corpus board
     parsed with no outline at all (6 boards).
  3. Up to version 20220815 a net tie is declared by keyword: a footprint
     whose (tags ...) start with "net tie" gets one pad group of every pad
     (NetTie parts and bridged solder jumpers, 10 boards).
  4. KiCad 6 writes an item's id as (tstamp ...); an id of at most 8 hex
     digits is a legacy timestamp. Footprint, track, via and zone uuids were
     all empty (every KiCad 6 board).
  5. Before version 20210606 an overbar is ~X~, now ~{X}; `--nets "/~{RST}"`,
     as KiCad shows the net, matched nothing on the CLI.
  6. KiCad 6 writes a lock as a bare word: `(footprint "X" locked (layer ...`
     and `(gr_line locked (start ...`. A locked footprint read as UNLOCKED
     (placement could move it, and the seeder's unlock left it locked in
     KiCad), and a locked outline line was dropped -- one corpus board parsed
     with no outer edge at all.

The expected values below are what pcbnew 10.0.3 produced for this exact
fixture (load + save), so the test needs no KiCad.

    python3 tests/test_kicad6_legacy_format_parity.py
"""
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

LEGACY_BOARD = """(kicad_pcb (version 20210228) (generator pcbnew)
  (general (thickness 1.6))
  (paper "A4")
  (layers
    (0 "F.Cu" signal)
    (1 "In1.Cu" signal)
    (2 "In2.Cu" signal)
    (31 "B.Cu" signal)
    (37 "F.SilkS" user)
    (39 "F.Mask" user)
    (44 "Edge.Cuts" user)
    (47 "F.CrtYd" user)
  )
  (net 0 "")
  (net 1 "GND")
  (net 2 "/~RST~")
  (net 3 "A~B~C")
  (net 4 "/~USER BTN~")
  (footprint "NetTie:NetTie-2_SMD_Pad0.5mm" locked (layer "F.Cu") (tedit 5A1DB3E7) (tstamp 0a1b2c3d-1111-2222-3333-444455556666)
    (at 20 20)
    (descr "Net tie, 2 pin, 0.5mm square SMD pads")
    (tags "net tie")
    (fp_text reference "NT1" (at 0 -1.2) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))) (tstamp 5c5c5c5c-0000-0000-0000-000000000001))
    (pad "1" smd circle (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "GND") (tstamp 5c5c5c5c-0000-0000-0000-000000000002))
    (pad "2" smd circle (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "/~RST~") (tstamp 5c5c5c5c-0000-0000-0000-000000000003))
  )
  (footprint "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" (layer "F.Cu") (tedit 5A3EABFC) (tstamp 5E3F1A2B)
    (at 30 20)
    (tags "solder jumper open")
    (fp_text reference "JP1" (at 0 -1.8) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))) (tstamp 5c5c5c5c-0000-0000-0000-000000000004))
    (pad "1" smd rect (at -0.65 0) (size 1 1.5) (layers "F.Cu" "F.Mask") (net 3 "A~B~C") (tstamp 5c5c5c5c-0000-0000-0000-000000000005))
    (pad "2" smd rect (at 0.65 0) (size 1 1.5) (layers "F.Cu" "F.Mask") (net 4 "/~USER BTN~") (tstamp 5c5c5c5c-0000-0000-0000-000000000006))
    (fp_arc (start 0 0) (end 1.5 0) (angle 90) (layer "F.CrtYd") (width 0.05))
    (fp_line (start 0 1.5) (end -1.5 0) (layer "F.CrtYd") (width 0.05))
  )
  (gr_line locked (start 12 10) (end 48 10) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000001))
  (gr_line locked (start 50 12) (end 50 38) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000002))
  (gr_line locked (start 48 40) (end 12 40) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000003))
  (gr_line locked (start 10 38) (end 10 12) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000004))
  (gr_arc (start 48 12) (end 48 10) (angle 90) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000005))
  (gr_arc (start 48 38) (end 48 40) (angle -90) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000006))
  (gr_arc (start 12 38) (end 10 38) (angle -90) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000007))
  (gr_arc (start 12 12) (end 10 12) (angle 90) (layer "Edge.Cuts") (width 0.1) (tstamp 1e1e1e1e-0000-0000-0000-000000000008))
  (segment (start 19.5 20) (end 15 25) (width 0.25) (layer "F.Cu") (net 1) (tstamp 2f2f2f2f-0000-0000-0000-000000000001))
  (via (at 15 25) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 1) (tstamp 3a3a3a3a-0000-0000-0000-000000000001))
  (zone (net 1) (net_name "GND") (layers F&B.Cu) (tstamp 4b4b4b4b-0000-0000-0000-000000000001) (hatch edge 0.508)
    (connect_pads (clearance 0.3))
    (min_thickness 0.254)
    (fill yes (thermal_gap 0.508) (thermal_bridge_width 0.508))
    (polygon (pts (xy 11 11) (xy 25 11) (xy 25 39) (xy 11 39)))
  )
  (zone (net 1) (net_name "GND") (layers *.Cu) (tstamp 4b4b4b4b-0000-0000-0000-000000000002) (hatch edge 0.508)
    (connect_pads (clearance 0.3))
    (min_thickness 0.254)
    (fill yes (thermal_gap 0.508) (thermal_bridge_width 0.508))
    (polygon (pts (xy 35 11) (xy 49 11) (xy 49 39) (xy 35 39)))
  )
)
"""

# pcbnew 10.0.3's re-save of each legacy arc above, start/mid/end.
PCBNEW_ARCS = {
    ('gr_arc', 48, 12): ((48, 10), (49.414214, 10.585786), (50, 12)),
    ('gr_arc', 48, 38): ((50, 38), (49.414214, 39.414214), (48, 40)),
    ('gr_arc', 12, 38): ((12, 40), (10.585786, 39.414214), (10, 38)),
    ('gr_arc', 12, 12): ((10, 12), (10.585786, 10.585786), (12, 10)),
    ('fp_arc', 0, 0): ((1.5, 0), (1.06066, 1.06066), (0, 1.5)),
}


def _write(tmp, name, text):
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)
    return path


def _restamp(text, version):
    return text.replace('(version 20210228)', '(version %d)' % version)


def test_legacy_arcs():
    from kicad_parser import upgrade_legacy_arcs
    up = upgrade_legacy_arcs(LEGACY_BOARD)
    got = re.findall(r'\((gr_arc|fp_arc)\s+\(start ([-\d.]+) ([-\d.]+)\) \(mid ([-\d.]+) ([-\d.]+)\) '
                     r'\(end ([-\d.]+) ([-\d.]+)\)', up)
    assert len(got) == 5 and '(angle' not in up, up
    centers = re.findall(r'\((gr_arc|fp_arc)\s+\(start ([-\d.]+) ([-\d.]+)\) \(end', LEGACY_BOARD)
    for (kind, *nums), (ckind, cx, cy) in zip(got, centers):
        key = (ckind, float(cx), float(cy))
        want = PCBNEW_ARCS[key]
        assert kind == key[0], (kind, key)
        pts = [float(n) for n in nums]
        flat = [c for p in want for c in p]
        assert all(abs(a - b) < 1e-5 for a, b in zip(pts, flat)), (key, pts, want)
    assert upgrade_legacy_arcs(_restamp(LEGACY_BOARD, 20211014)) == _restamp(LEGACY_BOARD, 20211014), \
        'a file past LEGACY_ARC_FORMATTING must be left exactly as written'
    print('  arcs: center/angle arcs land on pcbnew\'s start/mid/end; 6.0 files untouched')


def test_outline_and_courtyard(tmp):
    from kicad_parser import parse_kicad_pcb
    from placement.parser import extract_courtyard_bboxes
    path = _write(tmp, 'legacy.kicad_pcb', LEGACY_BOARD)
    bi = parse_kicad_pcb(path).board_info
    assert len(bi.board_outline) == 68, len(bi.board_outline)   # pcbnew's re-save parses to 68
    assert bi.board_bounds == (10.0, 10.0, 50.0, 40.0), bi.board_bounds
    cy = extract_courtyard_bboxes(path)['JP1']
    assert abs(cy[2] - 1.5) < 1e-6, f'the legacy fp_arc must reach x=1.5: {cy}'
    print('  outline: rounded corners kept (68 vertices); the courtyard keeps its arc')


def test_zone_layer_sets(tmp):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(_write(tmp, 'zones.kicad_pcb', LEGACY_BOARD))
    assert all(z.uuid for z in pcb.zones), 'each zone keeps its (tstamp ...) as its uuid'
    got = sorted((z.uuid[-1], z.layer) for z in pcb.zones)
    assert got == [('1', 'B.Cu'), ('1', 'F.Cu'), ('2', 'B.Cu'), ('2', 'F.Cu'),
                   ('2', 'In1.Cu'), ('2', 'In2.Cu')], got
    print('  zones: F&B.Cu -> F.Cu + B.Cu; *.Cu -> every copper layer')


def test_legacy_net_ties(tmp):
    from kicad_parser import parse_kicad_pcb
    fps = parse_kicad_pcb(_write(tmp, 'ties.kicad_pcb', LEGACY_BOARD)).footprints
    assert fps['NT1'].net_tie_groups == [['1', '2']], fps['NT1'].net_tie_groups
    assert fps['JP1'].net_tie_groups == [], 'tags not starting "net tie" are no tie'
    later = parse_kicad_pcb(_write(tmp, 'ties7.kicad_pcb', _restamp(LEGACY_BOARD, 20221018)))
    assert later.footprints['NT1'].net_tie_groups == [], \
        'past LEGACY_NET_TIES only (net_tie_pad_groups ...) declares a tie'
    print('  net ties: a legacy "net tie" footprint gets its pad group, a 7.0 file does not')


def test_tstamp_ids(tmp):
    from kicad_parser import parse_kicad_pcb, kiid_from_tstamp
    assert kiid_from_tstamp('5E3F1A2B') == '00000000-0000-0000-0000-00005e3f1a2b'
    assert kiid_from_tstamp('1A2B') == '00000000-0000-0000-0000-000000001a2b'
    assert kiid_from_tstamp('0a1b2c3d-1111-2222-3333-444455556666') == \
        '0a1b2c3d-1111-2222-3333-444455556666'
    pcb = parse_kicad_pcb(_write(tmp, 'ids.kicad_pcb', LEGACY_BOARD))
    assert pcb.footprints['NT1'].uuid == '0a1b2c3d-1111-2222-3333-444455556666'
    assert pcb.footprints['JP1'].uuid == '00000000-0000-0000-0000-00005e3f1a2b'
    assert [s.uuid for s in pcb.segments if not getattr(s, 'graphic', False)] == \
        ['2f2f2f2f-0000-0000-0000-000000000001']
    assert [v.uuid for v in pcb.vias] == ['3a3a3a3a-0000-0000-0000-000000000001']
    print('  ids: (tstamp ...) is the uuid, a legacy 8-digit stamp as KIID expands it')


def test_overbar_notation(tmp):
    from kicad_parser import parse_kicad_pcb, convert_to_new_overbar_notation as cv
    cases = {'~RST~': '~{RST}', 'A~B~C': 'A~{B}C', '/~USER BTN~': '/~{USER} BTN~{}',
             'ok~~tilde': 'ok~tilde', '~': '~', 'x~{y}': 'x~{y}', '~open': '~{open}'}
    for old, new in cases.items():
        assert cv(old) == new, (old, cv(old), new)
    pcb = parse_kicad_pcb(_write(tmp, 'ob.kicad_pcb', LEGACY_BOARD))
    assert {n.name for n in pcb.nets.values()} >= {'/~{RST}', 'A~{B}C', '/~{USER} BTN~{}'}
    assert pcb.footprints['NT1'].pads[1].net_name == '/~{RST}'
    later = parse_kicad_pcb(_write(tmp, 'ob6.kicad_pcb', _restamp(LEGACY_BOARD, 20210606)))
    assert '/~RST~' in {n.name for n in later.nets.values()}, \
        'from NEW_OVERBAR_NOTATION on, ~X~ is literal text'
    print('  overbar: ~X~ net names read as ~{X} before 20210606, literally after')


def test_bare_locks(tmp):
    from kicad_parser import parse_kicad_pcb
    from placement.parser import extract_locked_refs
    from placement.seeder import stamp_locked, stamp_unlocked
    path = _write(tmp, 'locks.kicad_pcb', LEGACY_BOARD)
    pcb = parse_kicad_pcb(path)
    assert pcb.footprints['NT1'].locked and not pcb.footprints['JP1'].locked
    assert extract_locked_refs(path) == {'NT1'}, extract_locked_refs(path)
    assert stamp_locked(path, ['NT1']) == 0, 'a bare lock is already a lock'
    assert stamp_unlocked(path, ['NT1']) == 1
    text = open(path, encoding='utf-8').read()
    assert '(footprint "NetTie:NetTie-2_SMD_Pad0.5mm" (layer' in text,         'the unlock must remove the bare word, or KiCad keeps the part locked'
    assert extract_locked_refs(path) == set()
    print('  locks: a bare footprint lock reads, stamps and unlocks; a locked outline line counts')


def main():
    tmp = tempfile.mkdtemp(prefix='kicad6_legacy_')
    try:
        test_legacy_arcs()
        test_outline_and_courtyard(tmp)
        test_zone_layer_sets(tmp)
        test_legacy_net_ties(tmp)
        test_tstamp_ids(tmp)
        test_overbar_notation(tmp)
        test_bare_locks(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('OK: KiCad 6-era boards parse the way pcbnew loads them')


if __name__ == '__main__':
    main()
