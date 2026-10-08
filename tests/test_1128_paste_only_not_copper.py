#!/usr/bin/env python3
"""#1128: a pad on no copper layer is not copper to the placement graders.

`legality.occupancy_shape` (a part's courtyard united with its pad copper,
the courtyard census's occupancy) and `legality.pad_copper_overrun_mm` (THE
gating measure for pad copper past the outline, read by check_assembly and
render_placement's `--gate`) skipped NPTH pads but read `pad.layers` nowhere,
so a paste-only aperture counted as copper. Both now skip every pad that puts
no copper on a copper layer (`legality._pad_carries_copper`, the predicate
`PartPads` already builds its pad list from).

* the issue's case: a 0.6 x 0.4 F.Cu pad grazing the right edge by 0.01 mm
  and a 1 x 1 F.Paste-only pad 3 mm further out -- the overrun is 0.01, not
  the paste pad's 3.21;
* a paste-only pad outside the courtyard does not enlarge the occupancy;
* controls: an F.Cu+F.Paste pad and a through-hole `*.Cu` pad outside the
  courtyard still do, an NPTH pad still does not, and an F.Cu pad past the
  edge still overruns;
* jetson-agx-thor's H5-H8 (KiCad 10 demo, skipped without it): occupancy
  62.41 mm2 either way -- their drawn paste apertures lie inside the hole
  pad's copper, which is why tests/measure_1128_paste_only_pads.py finds no
  real board the fix moves.

    python3 -X utf8 tests/test_1128_paste_only_not_copper.py [case ...]
"""
import os
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

_TMP = tempfile.TemporaryDirectory(prefix='t1128_')

#: courtyard [-0.5, 0.5]^2 around the anchor, so a pad at local x = 3 is
#: wholly outside it.
_COURT = ('    (fp_rect (start -0.5 -0.5) (end 0.5 0.5) (stroke (width 0.05) '
          '(type default)) (layer "F.CrtYd"))\n')


def _pad(num, x, w, h, layers, kind='smd', drill=None):
    dr = f' (drill {drill})' if drill else ''
    net = '' if kind == 'np_thru_hole' else ' (net 1 "N1")'
    return (f'    (pad "{num}" {kind} rect (at {x} 0) (size {w} {h}){dr} '
            f'(layers {layers}){net})\n')


def _board(ref, x, y, pads, court=True):
    wd = tempfile.mkdtemp(dir=_TMP.name)
    path = os.path.join(wd, 'b.kicad_pcb')
    fp = (f'  (footprint "t:{ref}" (layer "F.Cu") (at {x} {y})\n'
          f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
          + (_COURT if court else '') + ''.join(pads) + '  )\n')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                 '(35 "F.Paste" user) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1")\n'
                 '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) '
                 '(type default)) (layer "Edge.Cuts"))\n' + fp + ')\n')
    return path


def _parse(path):
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(path)


def _overrun(path, ref='U1'):
    from placement.legality import BoardOutlineGate, pad_copper_overrun_mm
    pcb = _parse(path)
    return pad_copper_overrun_mm(pcb.footprints[ref].pads,
                                 BoardOutlineGate(pcb.board_info, 0.0))


def _occupancy(path, ref='U1'):
    from placement.legality import graded_parts_from_file
    pcb = _parse(path)
    (g,) = [g for g in graded_parts_from_file(pcb, path) if g.ref == ref]
    assert g.poly is not None, 'no occupancy shape: the courtyard rung failed'
    return g.poly.area


def test_the_issues_overrun_is_the_copper_not_the_paste():
    """The F.Cu pad spans x [29.41, 30.01] (0.01 past the edge at 30); the
    paste pad spans [32.21, 33.21] (3.21 past it)."""
    path = _board('U1', 29.71, 15, [
        _pad('1', 0, 0.6, 0.4, '"F.Cu"'),
        _pad('2', 3.0, 1, 1, '"F.Paste"')])
    got = _overrun(path)
    assert abs(got - 0.01) < 1e-6, got
    # and through the gate check_assembly reads
    from placement.legality import grade_pad_legality
    g = grade_pad_legality(_parse(path), 0.2, pcb_file=path)
    worst = (g.get('oob_pad_copper_overrun_mm') or {}).get('U1')
    assert worst is not None and abs(worst - 0.01) < 1e-6, g.get(
        'oob_pad_copper_overrun_mm')
    print(f"  PASS: overrun {got:.3f} mm (the paste pad read 3.21)")


def test_a_paste_pad_outside_the_courtyard_is_not_occupancy():
    base = _occupancy(_board('U1', 15, 15, [_pad('1', 0, 0.6, 0.4,
                                                 '"F.Cu"')]))
    got = _occupancy(_board('U1', 15, 15, [
        _pad('1', 0, 0.6, 0.4, '"F.Cu"'),
        _pad('2', 3.0, 1, 1, '"F.Paste"')]))
    assert abs(base - 1.0) < 1e-6, base          # the courtyard alone
    assert abs(got - base) < 1e-6, (got, base)
    print(f"  PASS: occupancy {got:.3f} mm2 with a paste-only pad outside "
          f"the courtyard (its 1 mm2 not added)")


def test_copper_pads_outside_still_count():
    """Controls: the predicate is copper, not 'SMD on F.Cu only'."""
    smd = _occupancy(_board('U1', 15, 15, [
        _pad('1', 0, 0.6, 0.4, '"F.Cu"'),
        _pad('2', 3.0, 1, 1, '"F.Cu" "F.Paste"')]))
    tht = _occupancy(_board('U1', 15, 15, [
        _pad('1', 0, 0.6, 0.4, '"F.Cu"'),
        _pad('2', 3.0, 1, 1, '"*.Cu" "*.Mask"', kind='thru_hole',
             drill=0.5)]))
    npth = _occupancy(_board('U1', 15, 15, [
        _pad('1', 0, 0.6, 0.4, '"F.Cu"'),
        _pad('2', 3.0, 1, 1, '"*.Cu" "*.Mask"', kind='np_thru_hole',
             drill=0.9)]))
    assert abs(smd - 2.0) < 1e-6, smd
    assert abs(tht - 2.0) < 1e-6, tht
    assert abs(npth - 1.0) < 1e-6, npth
    over = _overrun(_board('U1', 29.71, 15, [
        _pad('1', 0, 0.6, 0.4, '"F.Cu"'),
        _pad('2', 3.0, 1, 1, '"F.Cu" "F.Paste"')]))
    assert abs(over - 3.21) < 1e-6, over
    hole = _overrun(_board('U1', 29.71, 15, [
        _pad('1', 0, 0.6, 0.4, '"F.Cu"'),
        _pad('2', 3.0, 1, 1, '"*.Cu" "*.Mask"', kind='np_thru_hole',
             drill=0.9)]))
    assert abs(hole - 0.01) < 1e-6, hole
    print(f"  PASS: F.Cu+F.Paste {smd:.1f} and THT {tht:.1f} mm2 still "
          f"counted, NPTH {npth:.1f}; a copper pad 3.21 mm out still "
          f"overruns, an NPTH hole there does not ({hole:.2f})")


DEMOS = os.environ.get('KICAD_DEMOS_DIR') or next(
    (d for d in (r'C:\Program Files\KiCad\10.0\share\kicad\demos',
                 '/Applications/KiCad/KiCad.app/Contents/SharedSupport/demos',
                 '/usr/share/kicad/demos') if os.path.isdir(d)), None)


def test_jetson_spacers_do_not_move():
    if not DEMOS:
        print("  SKIP: no KiCad demos directory (KICAD_DEMOS_DIR)")
        return
    jet = os.path.join(DEMOS, 'jetson-agx-thor-baseboard',
                       'jetson-agx-thor-baseboard.kicad_pcb')
    if not os.path.isfile(jet):
        print(f"  SKIP: {jet} absent")
        return
    from placement.legality import _pad_carries_copper, graded_parts_from_file
    pcb = _parse(jet)
    got = {g.ref: g.poly.area for g in graded_parts_from_file(pcb, jet)
           if g.ref in ('H5', 'H6', 'H7', 'H8')}
    assert sorted(got) == ['H5', 'H6', 'H7', 'H8'], got
    for ref, area in sorted(got.items()):
        paste = [p for p in pcb.footprints[ref].pads
                 if not _pad_carries_copper(p)]
        assert len(paste) == 4, (ref, len(paste))
        assert abs(area - 62.41) < 0.005, (ref, area)
    print("  PASS: jetson H5-H8 each carry 4 paste-only pads and occupy "
          "62.41 mm2 either way")


TESTS = [
    test_the_issues_overrun_is_the_copper_not_the_paste,
    test_a_paste_pad_outside_the_courtyard_is_not_occupancy,
    test_copper_pads_outside_still_count,
    test_jetson_spacers_do_not_move,
]


if __name__ == '__main__':
    want = sys.argv[1:]
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
