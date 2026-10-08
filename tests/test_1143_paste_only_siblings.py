#!/usr/bin/env python3
"""#1143: an aperture-only pad is not a pad to the placement measures.

#1128 taught `occupancy_shape` and `pad_copper_overrun_mm` that a pad on no
copper layer is not copper. #1143 named three more measures that still read
it -- the occupancy rect, `pad_area_balance` and `assembly_census` -- and the
sweep for it found the same class at about twenty more sites (the escape
pitch, the chip bounds the decap election measures to, part_class, the copper
geometry fallback, part_centre, the "has pads" gates of the census,
placement_state, recovery, lock_advisor, the board tools ...). Each now reads
`kicad_parser.non_aperture_pads` / `pad_is_aperture_only`, or
`paste_apertures.pad_has_copper` where the question is copper -- except the
MOVERS (the quench's zero-pad branch, reconcile's anchors, the drift default,
portfolio and the agent grade's poses), which keep an aperture-only part
MOVABLE on purpose (`test_quench_keeps_an_aperture_only_part_movable`,
`test_a_mechanical_aperture_only_part_is_anchored`). The router's own
readers followed in #1148 (`tests/test_1148_router_pad_readers.py`).

An aperture-only pad is: not NPTH, no drill, and no `*.Cu` layer. NPTH and
drilled pads are KEPT -- a mounting hole is physical extent; dropping it moves
splitflap H6/H7 and test_837's census, which is the mistake the predicate's
first control below guards.

Each case puts an F.Cu pad and an F.Paste-only pad side by side and asserts
the site measures the F.Cu pad alone, with controls that an F.Cu+F.Paste pad,
an NPTH hole and a drilled pad still count. A footprint whose ONLY pads are
apertures reads as pad-less to every measure tested here (the quench's
movability aside), and no site crashes on it.

Corpus witness: tigard C25 measures 2.93 mm to J1's copper (2.43 mm to the
box its 8 paste windows drew). Demo witness (skipped without KiCad 10's
demos): jetson-agx-thor H5's occupancy rect no longer sits 2.664 mm past the
outline.

    python3 -X utf8 tests/test_1143_paste_only_siblings.py [case ...]
"""
import contextlib
import io
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

RUN_ALL_TIMEOUT = 900

_TMP = tempfile.TemporaryDirectory(prefix='t1143_')

#: A RECTANGULAR outline: `pad_area_balance` abstains on a square board.
W, H = 40, 20
_COURT = ('    (fp_rect (start -0.5 -0.5) (end 0.5 0.5) (stroke (width 0.05) '
          '(type default)) (layer "F.CrtYd"))\n')


def _pad(num, x, w, h, layers, kind='smd', drill=None, y=0, net=1):
    dr = f' (drill {drill})' if drill else ''
    nt = ('' if kind == 'np_thru_hole' or not net
          else f' (net {net} "N{net}")')
    return (f'    (pad "{num}" {kind} rect (at {x} {y}) (size {w} {h}){dr} '
            f'(layers {layers}){nt})\n')


def _fp(ref, x, y, pads, court=True, name=None):
    return (f'  (footprint "t:{name or ref}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            + (_COURT if court else '') + ''.join(pads) + '  )\n')


def _board(*fps):
    wd = tempfile.mkdtemp(dir=_TMP.name)
    path = os.path.join(wd, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                 '(35 "F.Paste" user) (39 "F.Mask" user) '
                 '(44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
                 f'  (gr_rect (start 0 0) (end {W} {H}) (stroke (width 0.1) '
                 '(type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(fps) + ')\n')
    return path


def _parse(path):
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


CU = '"F.Cu"'
PASTE = '"F.Paste"'
CU_PASTE = '"F.Cu" "F.Paste"'
NPTH_LAYERS = '"*.Cu" "*.Mask"'


def _one(pads, ref='U1', x=10, y=10, court=True):
    path = _board(_fp(ref, x, y, pads, court=court))
    return path, _parse(path)


def test_the_predicate_keeps_npth_and_drilled_pads():
    """THE control: `_pad_carries_copper` alone also drops NPTH, which moves
    the A/B. The aperture predicate must keep NPTH and drilled pads."""
    from kicad_parser import pad_is_aperture_only
    _p, pcb = _one([
        _pad('1', 0, 0.6, 0.4, CU),
        _pad('2', 3, 1, 1, PASTE),
        _pad('3', 6, 1, 1, '"F.Mask"'),
        _pad('4', 9, 1, 1, CU_PASTE),
        _pad('5', 12, 1, 1, NPTH_LAYERS, kind='np_thru_hole', drill=1),
        _pad('6', 15, 1.6, 1.6, '"*.Cu" "*.Mask"', kind='thru_hole',
             drill=0.8),
        # a drilled pad that names no copper layer: still a hole, so kept
        _pad('7', 18, 1.6, 1.6, '"*.Mask"', kind='thru_hole', drill=0.8),
        # an NPTH with no drill and no copper layer: the NPTH clause alone
        # keeps it (the drill clause cannot)
        _pad('8', 21, 1, 1, '"*.Mask"', kind='np_thru_hole')])
    got = {p.pad_number: pad_is_aperture_only(p)
           for p in pcb.footprints['U1'].pads}
    want = {'1': False, '2': True, '3': True, '4': False, '5': False,
            '6': False, '7': False, '8': False}
    assert got == want, got
    # A stand-in with no `layers` at all (arrays.pose_free_chip_refs' pads,
    # test stubs) is NOT an aperture: nothing says it is one.
    from types import SimpleNamespace
    assert pad_is_aperture_only(SimpleNamespace(global_x=0.0,
                                                global_y=0.0)) is False
    print(f"  PASS: aperture-only = paste/mask windows only {got}; a pad "
          f"with no layers is not one")


def test_the_four_copper_predicates_agree():
    """`paste_apertures.pad_has_copper`, `legality._pad_carries_copper`,
    `check_drc._pad_has_no_copper` (negated) and `obstacle_map._pad_has_copper`
    answer one question; on the synthetic set and on two corpus boards that
    carry aperture pads they must agree pad for pad. (They differ on
    `layers=None`, which check_drc and obstacle_map raise on and the two
    placement predicates read as no copper -- not reached by either parser.)"""
    from paste_apertures import pad_has_copper
    from placement.legality import _pad_carries_copper
    from check_drc import _pad_has_no_copper
    from obstacle_map import _pad_has_copper as om_has_copper
    _p, pcb = _one([
        _pad('1', 0, 0.6, 0.4, CU), _pad('2', 3, 1, 1, PASTE),
        _pad('3', 6, 1, 1, '"F&B.Cu"'), _pad('4', 9, 1, 1, '"In1.Cu"'),
        _pad('5', 12, 1, 1, NPTH_LAYERS, kind='np_thru_hole', drill=1),
        _pad('6', 15, 1, 1, '"F.Mask"')])
    pads = list(pcb.footprints['U1'].pads)
    n_corpus = 0
    for b in ('tigard', 'orangecrab_ext_pll'):
        bp = os.path.join(ROOT, 'kicad_files', b + '.kicad_pcb')
        for fp in _parse(bp).footprints.values():
            pads.extend(fp.pads or ())
            n_corpus += len(fp.pads or ())
    bad = [(p.pad_number, p.layers) for p in pads
           if len({pad_has_copper(p), _pad_carries_copper(p),
                   not _pad_has_no_copper(p), om_has_copper(p)}) != 1]
    assert not bad, bad[:5]
    assert n_corpus > 1000, n_corpus
    print(f"  PASS: four predicates agree on {len(pads)} pads "
          f"({n_corpus} corpus)")


def test_bbox_local_skips_the_aperture_not_npth():
    from placement.utility import compute_footprint_bbox_local as bb
    _p, pcb = _one([_pad('1', 0, 0.6, 0.4, CU), _pad('2', 3, 1, 1, PASTE)])
    got = bb(pcb.footprints['U1'])
    assert abs(got[2] - 0.3) < 1e-9, got           # not 3.5
    _p, pcb = _one([_pad('1', 0, 0.6, 0.4, CU), _pad('2', 3, 1, 1, CU_PASTE)])
    assert abs(bb(pcb.footprints['U1'])[2] - 3.5) < 1e-9
    _p, pcb = _one([_pad('1', 0, 0.6, 0.4, CU),
                    _pad('2', 3, 1, 1, NPTH_LAYERS, kind='np_thru_hole',
                         drill=1)])
    assert abs(bb(pcb.footprints['U1'])[2] - 3.5) < 1e-9
    _p, pcb = _one([_pad('1', 0, 1, 1, PASTE)])
    assert bb(pcb.footprints['U1']) == (-0.5, -0.5, 0.5, 0.5)
    print(f"  PASS: bbox max x {got[2]:.2f} (paste at 3.5 skipped; "
          f"copper and NPTH at 3.5 kept; aperture-only -> fallback)")


def test_the_occupancy_rect_and_pads_rung():
    """No courtyard: the occupancy ladder falls to the PADS rung, which must
    be the copper pad's box. An aperture-only part gets no pads rung."""
    from placement.legality import graded_parts_from_file
    path = _board(_fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU),
                                     _pad('2', 3, 1, 1, PASTE)], court=False),
                  _fp('U2', 30, 10, [_pad('1', 0, 1, 1, PASTE)], court=False))
    pcb = _parse(path)
    g = {x.ref: x for x in graded_parts_from_file(pcb, path)}
    assert abs(g['U1'].rect[2] - 10.3) < 1e-6, g['U1'].rect
    from placement.body import board_bodies, SOURCE_NONE
    bb = board_bodies(pcb, path)
    # EXACTLY no source: an aperture-only part has no pads rung, and with no
    # pads it may not claim a silk body either (Rule 2). `!= 'pads'` alone
    # let a mutant that gave it a pads rung through the silk union pass.
    assert bb['U2'].source == SOURCE_NONE, bb['U2']
    assert bb['U2'].occupancy_local is None, bb['U2']
    print(f"  PASS: U1 rect {tuple(round(v, 3) for v in g['U1'].rect)}; "
          f"aperture-only U2 source {bb['U2'].source!r}")


def test_assembly_census_aperture_only_part_is_zero_pad():
    from placement.legality import assembly_census
    path = _board(_fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU),
                                     _pad('2', 3, 1, 1, PASTE)]),
                  _fp('U2', 30, 10, [_pad('1', 0, 1, 1, PASTE)]),
                  _fp('H1', 20, 15, [_pad('', 0, 2, 2, NPTH_LAYERS,
                                          kind='np_thru_hole', drill=2),
                                     _pad('1', 2, 1, 1, PASTE)]))
    c = assembly_census(_parse(path))
    assert c['zero_pad']['F'] == ['U2'], c['zero_pad']
    assert c['pad_bearing']['F'] == 2, c['pad_bearing']
    assert c['smd']['F'] == 1 and c['unsoldered']['F'] == 1, c
    assert 'aperture' in c['basis'], c['basis']
    print(f"  PASS: zero_pad {c['zero_pad']['F']}, smd {c['smd']['F']}, "
          f"NPTH+paste H1 unsoldered {c['unsoldered']['F']}")


def test_pad_area_balance_weighs_copper_and_names_its_basis():
    import placement_score as PS
    a = PS.pad_area_balance(_parse(_board(
        _fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU)]),
        _fp('U2', 30, 10, [_pad('1', 0, 0.6, 0.4, CU)]))))
    b = PS.pad_area_balance(_parse(_board(
        _fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU)]),
        _fp('U2', 30, 10, [_pad('1', 0, 0.6, 0.4, CU),
                           _pad('2', 5, 3, 3, PASTE)]))))
    c = PS.pad_area_balance(_parse(_board(
        _fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU)]),
        _fp('U2', 30, 10, [_pad('1', 0, 0.6, 0.4, CU),
                           _pad('2', 5, 3, 3, CU_PASTE)]))))
    assert a['ran'] and b['ran'] and c['ran'], (a, b, c)
    assert a['value'] == b['value'], (a['value'], b['value'])
    assert c['value'] > a['value'] + 0.05, (a['value'], c['value'])
    assert b['aperture_pads_excluded'] == 1, b
    assert b['basis'] == [PS.BALANCE_BASIS], b['basis']
    print(f"  PASS: balance {a['value']} with or without the 3x3 paste "
          f"window (copper there: {c['value']}); basis named")


def test_balance_refuses_to_compare_across_the_basis():
    """A lap scored before #1143 published `basis: None`; comparing it with
    one scored after would report a balance change no placement caused."""
    import placement_score as PS
    old = {'balance': {'ran': True, 'value': 0.0328, 'unit': 'frac',
                       'direction': 'lower', 'basis': None}}
    new = {'balance': {'ran': True, 'value': 0.0207, 'unit': 'frac',
                       'direction': 'lower', 'basis': [PS.BALANCE_BASIS]}}
    rows = {r['term']: r for r in PS.term_deltas(old, new)}
    assert 'balance' in rows, sorted(rows)
    r = rows['balance']
    assert r.get('judgement') == 'not-comparable', r
    print(f"  PASS: old-basis balance -> {r['judgement']} ({r.get('why')})")


def test_pad_pitch_reads_the_pin_lattice():
    """A thermal pad's split paste windows sit 0.05 mm apart; read as pads
    they collapse the pitch (tigard U3 0.033 vs 0.108)."""
    from placement.escape import pad_pitch, _part_rect, fine_pitch_parts
    pins = [_pad(str(i), round(i * 0.5, 3), 0.25, 0.8, CU, y=-2)
            for i in range(8)]
    windows = [_pad(f'W{i}', round(1.5 + i * 0.05, 3), 0.04, 0.04, PASTE)
               for i in range(4)]
    _p, pcb = _one(pins + windows)
    got = pad_pitch(pcb.footprints['U1'])
    assert abs(got - 0.5) < 1e-6, got
    _p, pcb = _one([_pad('1', 0, 1, 1, PASTE), _pad('2', 1, 1, 1, PASTE)],
                   ref='U2', x=30, y=10)
    fp = pcb.footprints['U2']
    assert pad_pitch(fp) == float('inf')
    assert _part_rect(fp) == (30, 10, 30, 10), _part_rect(fp)
    assert fine_pitch_parts(pcb) == []
    print(f"  PASS: pitch {got} (windows at 0.05 skipped); an aperture-only "
          f"part has no pitch and no crash")


def test_copper_geometry_and_centre_skip_an_aperture_only_part():
    from placement.legality import part_copper_geometry
    from placement.pose_ops import part_centre, part_faces, PoseRefusal
    path = _board(_fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU),
                                     _pad('2', 3, 1, 1, PASTE)]),
                  _fp('U2', 30, 10, [_pad('1', 2, 1, 1, PASTE)]))
    pcb = _parse(path)
    geo = part_copper_geometry(pcb.footprints, 0.2)
    assert 'U2' not in geo and 'U1' in geo, sorted(geo)
    assert part_centre(pcb, 'U2') == (30.0, 10.0)
    assert part_centre(pcb, 'U1') == (10.0, 10.0), part_centre(pcb, 'U1')
    try:
        part_faces(pcb, 'U2', clearance=0.2, track_width=0.2)
    except PoseRefusal as exc:
        assert 'no pads' in str(exc), exc
    else:
        raise AssertionError('part_faces aimed an aperture-only part')
    print("  PASS: copper geometry, centre and faces read U1's copper; "
          "U2 (apertures only) is pad-less")


def test_part_class_and_band():
    """A mounting hole with a paste ring is still NPTH-only; a receptacle's
    band is its pad field's, not its paste windows'."""
    from placement.part_class import classify_part, default_band
    _p, pcb = _one([_pad('', 0, 3, 3, NPTH_LAYERS, kind='np_thru_hole',
                         drill=3),
                    _pad('1', 2, 1, 1, PASTE)], ref='H1')
    c = classify_part(pcb.footprints['H1'], 'H1')
    assert c.name == 'mount_hole' and c.confidence == 'high', c
    _p, pcb = _one([_pad('1', 0, 1, 1, CU), _pad('2', 0.8, 1, 1, CU, y=0.8),
                    _pad('3', 6, 1, 1, PASTE, y=6)], ref='J1')
    band = default_band('edge_receptacle', pcb.footprints['J1'])
    assert band['max'] == 0.5, band       # half of 0.8, floored at 0.5
    print(f"  PASS: NPTH+paste -> {c.name}; band {band}")


def test_chip_bounds_and_centroid_skip_paste():
    from placement import groups
    pins = [_pad(str(i), x, 0.4, 0.4, CU, y=y)
            for i, (x, y) in enumerate([(0, 0), (1, 0), (0, 1), (1, 1)])]
    _p, pcb = _one(pins + [_pad('W', 5, 0.4, 0.4, PASTE, y=5)])
    fp = pcb.footprints['U1']
    b = groups.chip_bounds_of(fp)
    assert abs(b[2] - 11.5) < 1e-6 and abs(b[3] - 11.5) < 1e-6, b
    assert groups._centroid(fp) == (10.5, 10.5), groups._centroid(fp)
    print(f"  PASS: chip bounds {b} (the window at 15,15 skipped)")


def test_quench_keeps_an_aperture_only_part_movable():
    """The DELIBERATE exclusion: the quench's zero-pad branch still reads
    `fp.pads`, so a part whose only pads are apertures (a logo) stays a
    MOVABLE quench part with a pad box, never an unmovable obstacle. The
    seeder's fixed-pose stage seats such a part by its courtyard
    (test_1051_hardening's copperless logo); locking it made that stage
    refuse it as "already placed" (Phase-1 verifier). Its pad box is the
    bbox fallback, because its apertures are not extent."""
    from placement.quench import QuenchState
    path = _board(_fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU)]),
                  _fp('U2', 30, 10, [_pad('1', 2, 1, 1, PASTE)]))
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        st = QuenchState(_parse(path), path, 0.2, 0.55, 10.0, 0.5, 0.25,
                         2.0, 2.0, 2.0, 0.1, 1.0)
    assert not st.parts['U2'].locked, 'an aperture-only part was locked'
    assert st.parts['U2'].padbox_local == (-0.5, -0.5, 0.5, 0.5), (
        st.parts['U2'].padbox_local)
    assert not st.parts['U1'].locked
    print("  PASS: U2 (apertures only) movable, pad box the fallback; "
          "U1 movable")


def _gate_board():
    """U1: a 1x4 copper row along x plus one 1x1 paste window OFF the line;
    C1: a two-pad cap with two paste windows; U2: apertures only."""
    pins = [_pad(str(i), i * 1.0, 0.5, 0.5, CU) for i in range(4)]
    return _board(
        _fp('U1', 10, 10, pins + [_pad('W', 1.5, 1, 1, PASTE, y=3)]),
        _fp('C1', 20, 10, [_pad('1', 0, 0.5, 0.5, CU),
                           _pad('2', 1, 0.5, 0.5, CU, net=2),
                           _pad('3', 0, 0.2, 0.2, PASTE, y=0.5),
                           _pad('4', 1, 0.2, 0.2, PASTE, y=0.5)]),
        _fp('U2', 30, 10, [_pad('1', 0, 1, 1, PASTE)]))


def test_every_gate_reads_pins_not_apertures():
    """One assertion per site the Phase-1 verifier found untested (#1143):
    each reads U1's four pins, never its paste window, and none counts the
    aperture-only U2 as a pad-bearing part."""
    import importlib.util
    from placement import (edge_facing, escape, floorplan, groups, legality,
                           placement_state, portfolio, recovery, utility)
    path = _gate_board()
    pcb = _parse(path)
    u1, c1 = pcb.footprints['U1'], pcb.footprints['C1']
    got = {}
    got['floorplan._copper_pads'] = len(floorplan._copper_pads(u1))
    _s, partners, _d = floorplan.proximity_pads(
        {'ref': 'U1', 'near': 'C1'}, u1, c1)
    got['floorplan.proximity_pads'] = len(partners)
    got['groups._pads_are_collinear'] = groups._pads_are_collinear(u1)
    got['legality.bodyless_pad_shape'] = round(
        legality.bodyless_pad_shape(u1).area, 6)
    _pads, rect, _p, _c, _pitch = edge_facing.part_inputs(pcb, 'U1')
    got['edge_facing.rect_maxy'] = round(rect[3], 6)
    asg = escape.assign_faces(u1, None, lane_mm=0.3,
                              fallback_rect=escape._part_rect(u1))
    got['escape.assign_faces'] = len(asg.faces)
    with contextlib.redirect_stdout(io.StringIO()):
        st = placement_state.assess_placement(pcb, path)
    got['placement_state.n_footprints'] = st.n_footprints
    got['portfolio._final_poses'] = sorted(portfolio._final_poses(pcb, []))
    with contextlib.redirect_stdout(io.StringIO()):
        got['portfolio.free_refs'] = sorted(portfolio.free_refs(pcb, path))
    from placement import arrays
    # U1 is a 1x4 ROW once its off-line paste window is ignored: neither the
    # posed nor the pose-free chip test may call it a chip.
    got['groups.chip_refs'] = sorted(groups.chip_refs(pcb))
    got['arrays.pose_free_chip_refs'] = sorted(
        arrays.pose_free_chip_refs(pcb))
    got['recovery.board_poses'] = sorted(recovery.board_poses(pcb))
    got['recovery.part_displacement'] = round(recovery.part_displacement(
        u1, (10.0, 10.0, 0.0), (10.0, 10.0, 180.0)), 6)
    # a rect around U1's paste window alone (global (11.5, 13))
    got['utility.refs_in_rect'] = utility.refs_in_rect(
        pcb, (11.0, 12.5, 12.0, 13.5), by='pad')
    import check_pockets
    bins = check_pockets.copper_touched_bins(pcb, 1.0)
    got['check_pockets.u2_bin'] = (30, 10) in bins
    import board_context
    with contextlib.redirect_stdout(io.StringIO()):
        doc = board_context.build_context(pcb, path, clearance=0.2,
                                          track_width=0.2)
    rows = []

    def walk(o):
        if isinstance(o, dict):
            if o.get('ref') == 'U1' and 'pads' in o:
                rows.append(o['pads'])
            for v in o.values():
                walk(v)
        elif isinstance(o, list):
            for v in o:
                walk(v)
    walk(doc)
    got['board_context.pads'] = rows[:1]
    spec = importlib.util.spec_from_file_location(
        'agent_grade', os.path.join(ROOT, '.claude', 'skills',
                                    'pcb-free-agent', 'scripts', 'grade.py'))
    grade = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(grade)
    with contextlib.redirect_stdout(io.StringIO()):
        got['agent_grade._poses'] = sorted(grade._poses(path))
    # the four pins at x 0..3 about the origin, turned 180 about it
    rms = (sum((2 * i) ** 2 for i in range(4)) / 4) ** 0.5
    want = {
        'floorplan._copper_pads': 4,
        'floorplan.proximity_pads': 2,
        'groups._pads_are_collinear': True,
        'legality.bodyless_pad_shape': 1.0,
        'edge_facing.rect_maxy': 10.0,
        'escape.assign_faces': 4,
        'placement_state.n_footprints': 2,
        # The "can a run MOVE it" sites follow the quench, which keeps the
        # aperture-only U2 movable (the deliberate exclusion): U2 is IN.
        'portfolio._final_poses': ['C1', 'U1', 'U2'],
        'portfolio.free_refs': ['C1', 'U1', 'U2'],
        'groups.chip_refs': [],
        'arrays.pose_free_chip_refs': [],
        'recovery.board_poses': ['C1', 'U1'],
        'recovery.part_displacement': round(rms, 6),
        'utility.refs_in_rect': [],
        'check_pockets.u2_bin': False,
        'board_context.pads': [4],
        'agent_grade._poses': ['C1', 'U1', 'U2'],
    }
    bad = {k: (got.get(k), v) for k, v in want.items() if got.get(k) != v}
    assert not bad, bad
    print(f"  PASS: {len(want)} sites read U1's pins; U2 is pad-less to the "
          f"measures and movable to the movers")


def test_a_mechanical_aperture_only_part_is_anchored():
    """reconcile.anchor_blocks asks the quench's question -- does the seeder
    PLACE this part -- so an aperture-only part with a mechanical pose is
    anchored like any placed part, not skipped as "pad-less: the seeder never
    places it" (Phase-1 second-pass verifier: under that skip a G1 the seeder
    did place drifted 3.16 mm without an anchor)."""
    from placement import reconcile
    path = _board(_fp('U1', 10, 10, [_pad('1', 0, 0.6, 0.4, CU)]),
                  _fp('G1', 30, 10, [_pad('1', 0, 1, 1, PASTE)]))
    pcb = _parse(path)
    mech = {'poses': {'G1': {'x': 30.0, 'y': 10.0, 'rot': 0.0}}}
    with contextlib.redirect_stdout(io.StringIO()):
        blocks, skipped = reconcile.anchor_blocks(pcb, path, mech)
    assert 'G1' not in skipped, skipped
    assert any('G1' in (b.get('refs') or ()) for b in blocks), blocks
    print("  PASS: the aperture-only G1 is anchored, not skipped")


def test_a_short_names_the_copper_pad_after_a_paste_window():
    """`pad_intersection_pairs` indexes copper pads, so its short label must
    too: with a paste window first in U1's pad list, a short on U1's pad 2
    printed as U1.1 (final review)."""
    from placement.legality import pad_intersection_pairs
    path = _board(
        _fp('U1', 10, 10, [_pad('', -3, 1, 1, PASTE),
                           _pad('1', 0, 0.6, 0.6, CU),
                           _pad('2', 1.5, 0.6, 0.6, CU, net=2)],
            court=False),
        _fp('U2', 11.5, 10.2, [_pad('7', 0, 0.6, 0.6, CU)], court=False))
    pairs = pad_intersection_pairs(_parse(path), 0.2)
    shorts = [s for q in pairs for s in (q.shorts or ())]
    assert shorts and all('U1.2' in s for s in shorts), shorts
    print(f"  PASS: {shorts[0]}")


def test_tigard_c25_measures_to_j1_copper():
    from placement import groups
    pcb = _parse(os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb'))
    got = {c: (ic, d) for c, ic, d in groups._elect_tethers(pcb)}
    ic, d = got['C25']
    assert ic == 'J1' and abs(d - 2.93) < 0.005, (ic, d)
    print(f"  PASS: tigard C25 -> {ic} {d:.3f} mm (its paste windows read "
          f"2.43)")


def test_jetson_spacers_are_inside_the_outline():
    demos = os.environ.get('KICAD_DEMOS_DIR') or next(
        (d for d in (r'C:\Program Files\KiCad\10.0\share\kicad\demos',
                     '/usr/share/kicad/demos',
                     '/Applications/KiCad/KiCad.app/Contents/SharedSupport/'
                     'demos') if os.path.isdir(d)), None)
    path = demos and os.path.join(demos, 'jetson-agx-thor-baseboard',
                                  'jetson-agx-thor-baseboard.kicad_pcb')
    if not path or not os.path.isfile(path):
        print("  SKIP: KiCad 10 demos not installed (KICAD_DEMOS_DIR)")
        return
    from placement import legality as L
    pcb = _parse(path)
    gate = L.BoardOutlineGate(pcb.board_info, 0.0)
    g = {x.ref: x for x in L.graded_parts_from_file(pcb, path)}
    over = {r: gate.rect_outside_amount(g[r].rect)
            for r in ('H5', 'H6', 'H7', 'H8')}
    assert all(v < 1e-6 for v in over.values()), over
    print(f"  PASS: jetson H5-H8 occupancy rect past the outline {over}")


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
