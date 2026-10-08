#!/usr/bin/env python3
"""#1065: render_placement's pad-clearance checklist is the grader's census.

`checklist.b_pad_clearance_pairs` (and the `--gate` key, the red-ring overlay,
the panels and the narrative, which all read the same list) came from
`LegalityContext.pair_shortfall`, which charges BOUNDING-BOX gaps. On an oval
pad that is not the copper: run 33's U1.1 against USB1's oval GND pad sat
0.277 mm apart, the boxes 0.2285, and at 0.25 render flagged a pose that
`grade_pad_legality`, `place_pose` and `check_drc` all graded clean. The run
gated its search on render, so it rejected legal poses.

Render now nominates with that box test (a NECESSARY condition) and lets the
grader's own per-pair census, `legality.pad_pair_conflict`, decide and give
the mm, on the copper at the MODEL's pose. Every arm checks render against
`grade_pad_legality` on the same board, so a refactor that broke the grader
the same way would still be caught by the pinned numbers.

The fixture is cand_r1's relative geometry: U1 pad 1, SMD rect 0.325 x 1.27
at (15, 10), no net; USB1 pad 0, oval 1.8 x 1.2 at (13.709, 10.923), GND.

Run: python3 -X utf8 tests/test_1065_render_pad_clearance_exact.py [case ...]
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

from kicad_parser import parse_kicad_pcb  # noqa: E402

RUN_ALL_TIMEOUT = 900

CLEARANCE = 0.25
USB_AT = (13.709, 10.923)


def _board(path, usb_x=USB_AT[0], u1_net=None, u1_clearance=None):
    net = ' (net 1 "GND")' if u1_net == 'GND' else ''
    clr = f' (clearance {u1_clearance})' if u1_clearance else ''
    text = (
        '(kicad_pcb (version 20240108) (generator "t")\n'
        ' (general (thickness 1.6))\n'
        ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal)\n'
        '  (44 "Edge.Cuts" user) (37 "F.SilkS" user) (39 "F.Mask" user)\n'
        '  (35 "F.Paste" user) (49 "F.Fab" user) (47 "F.CrtYd" user))\n'
        ' (setup)\n (net 0 "")\n (net 1 "GND")\n'
        ' (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) '
        '(type solid)) (fill no) (layer "Edge.Cuts"))\n'
        '  (footprint "U:QFN" (layer "F.Cu") (uuid "u1") (at 15 10 0)\n'
        '    (property "Reference" "U1" (at 0 -1.5 0) (layer "F.SilkS"))\n'
        '    (fp_rect (start -0.25 -0.7) (end 0.25 0.7) (stroke (width 0.05) '
        '(type solid)) (fill none) (layer "F.CrtYd"))\n'
        f'    (pad "1" smd rect (at 0 0) (size 0.325 1.27) '
        f'(layers "F.Cu" "F.Mask" "F.Paste"){net}{clr}))\n'
        f'  (footprint "J:USB" (layer "F.Cu") (uuid "u2") '
        f'(at {usb_x} {USB_AT[1]} 0)\n'
        '    (property "Reference" "USB1" (at 0 -1.5 0) (layer "F.SilkS"))\n'
        '    (fp_rect (start -0.95 -0.65) (end 0.95 0.65) (stroke (width 0.05) '
        '(type solid)) (fill none) (layer "F.CrtYd"))\n'
        '    (pad "0" smd oval (at 0 0) (size 1.8 1.2) '
        '(layers "F.Cu" "F.Mask" "F.Paste") (net 1 "GND")))\n'
        ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def _tmp_board(**kw):
    wd = tempfile.mkdtemp(prefix='t1065_')
    return _board(os.path.join(wd, 'b.kicad_pcb'), **kw)


def _render(path, clearance=CLEARANCE, move=None):
    """render's pad rows (list of [a, b, mm]) and the model, at `clearance`;
    `move` = (ref, x, y, rot) applied to the MODEL before the findings."""
    import render_placement as RP
    model = RP.PlacementModel(parse_kicad_pcb(path), path, exact=True,
                              quench_kwargs={'clearance': clearance,
                                             'ignore_net_ids': None})
    if move is not None:
        model.state.apply_move(*move)
        model._legality_findings = None
    return [list(r) for r in RP.legality_findings(model)
            ['pad_conflict_pairs_refs']], model


def _grade(path, clearance=CLEARANCE):
    from placement.legality import grade_pad_legality
    g = grade_pad_legality(parse_kicad_pcb(path), clearance, pcb_file=path,
                           worst_n=0)
    return [list(w) for w in g['worst']]


def _box_rows(model):
    """What render reported before #1065: the box term alone."""
    ctx = model.state.legality_ctx
    refs = sorted(ctx.parts)
    return [[a, b, round(ctx.pair_shortfall(a, b).pad, 4)]
            for i, a in enumerate(refs) for b in refs[i + 1:]
            if ctx.pair_shortfall(a, b).pad > 1e-6]


def test_an_oval_graze_is_clean():
    """The #1065 pose: the boxes are 0.0215 short, the copper is 0.027 clear."""
    path = _tmp_board()
    got, model = _render(path)
    assert _box_rows(model) == [['U1', 'USB1', 0.0215]], _box_rows(model)
    assert got == [] and _grade(path) == [], (got, _grade(path))
    print("  box 0.0215 short; render and grade: clean")


def test_a_real_shortfall_is_the_graders_mm():
    """USB1 0.06 mm closer: the copper really is short, and render reports
    the grader's mm, not the box's."""
    path = _tmp_board(usb_x=USB_AT[0] + 0.06)
    got, model = _render(path)
    want = _grade(path)
    assert len(want) == 1 and got == want, (got, want)
    box = _box_rows(model)[0][2]
    assert got[0][2] < box - 0.02, (got, box)
    print(f"  render {got} == grade; the box read {box}")


def test_a_pad_override_sets_the_requirement():
    """U1's pad declares `(clearance 0.3)`: the requirement is the pad's, so
    the 0.277 mm copper gap IS short -- found only with the per-pair model."""
    path = _tmp_board(u1_clearance=0.3)
    got, _m = _render(path)
    want = _grade(path)
    assert len(want) == 1 and got == want, (got, want)
    assert 0.015 < got[0][2] < 0.03, got
    print(f"  pad override 0.3: render {got} == grade")


def test_same_net_pads_never_conflict():
    """U1's pad on GND, at the real-shortfall pose: same-net copper is not a
    clearance pair, in render or in the grade."""
    path = _tmp_board(usb_x=USB_AT[0] + 0.06, u1_net='GND')
    got, _m = _render(path)
    assert got == [] and _grade(path) == [], (got, _grade(path))
    print("  same net: clean in both")


def test_the_copper_is_read_at_the_models_pose():
    """render draws PROPOSED poses (a move applied to the model, nothing
    written): the clean board with USB1 moved 0.06 mm closer in the model
    reports what the board written at that pose grades."""
    clean = _tmp_board()
    got, _m = _render(clean, move=('USB1', USB_AT[0] + 0.06, USB_AT[1], 0.0))
    want = _grade(_tmp_board(usb_x=USB_AT[0] + 0.06))
    assert len(want) == 1 and got == want, (got, want)
    print(f"  moved in the model: render {got} == the written pose's grade")


#: grade_pad_legality's `worst` on four tracked boards, recorded at
#: 457959b7 (before #1065, and before the census was lifted out of it), at
#: the clearance render resolves for each board. Pinned so a refactor that
#: broke render AND the grade alike would still fail here.
CORPUS = {
    'esp_prog': [],
    'glasgow_revC': [['C21', 'C77', 0.04]],
    'rp2350_fpga_eensy_prePlane': [['C18', 'C19', 0.15], ['R4', 'R5', 0.05]],
    'orangecrab_ext_pll': [['C67', 'TP30', 0.0293], ['R21', 'R29', 0.05],
                           ['TP19', 'TP20', 0.0429], ['TP28', 'U8', 0.025]],
}


def test_the_corpus_agrees_with_the_grade():
    for name, want in CORPUS.items():
        path = run_utils.evidence(os.path.join(
            ROOT, 'kicad_files', name + '.kicad_pcb'))
        got, model = _render(path, clearance=None)
        clr = (model.floor_knobs.get('clearance') or {}).get('value')
        grade = _grade(path, clr)
        assert sorted(got) == sorted(grade) == sorted(want), (
            name, got, grade, want)
        print(f"  {name}: {len(got)} pair(s), render == grade == pinned")


def test_the_caption_counts_the_graders_pairs():
    """The panel caption's `pad-conflicts` is the checklist's count, not
    `metrics.pad_conflict_pairs` -- the optimizer's bounding-box currency,
    which stays in the JSON. On the #1065 pose the metric reads 1 and the
    caption 0 (the Phase-5 verifier's glasgow case: 10 against 1)."""
    import render_placement as RP
    path = _tmp_board()
    _rows, model = _render(path)
    assert model.metrics.get('pad_conflict_pairs') == 1, model.metrics.get(
        'pad_conflict_pairs')
    cap = RP.caption(RP.PanelSpec(model=model, label='after'))
    assert 'pad-conflicts 0' in cap, cap
    print(f"  caption {cap.split('|')[-2].strip()!r}; metric 1")


def test_the_gate_reads_the_graders_pairs():
    """`--gate` passes the #1065 pose and fails the real shortfall, naming the
    key."""
    tool = os.path.join(ROOT, 'py_tools', 'render_placement.py')
    wd = tempfile.mkdtemp(prefix='t1065_gate_')
    clean = _board(os.path.join(wd, 'clean.kicad_pcb'))
    short = _board(os.path.join(wd, 'short.kicad_pcb'),
                   usb_x=USB_AT[0] + 0.06)
    for path, kw in ((clean, dict(accept=True)),
                     (short, dict(refuse='b_pad_clearance_pairs=1', code=4))):
        r = run_utils.check(
            [sys.executable, '-X', 'utf8', tool, path,
             '-o', path[:-10] + '.png', '--clearance', str(CLEARANCE),
             '--gate'], **kw)
        if kw.get('accept'):
            both = (r.stdout or '') + (r.stderr or '')
            assert 'GATE: PASS' in both, both[-1500:]
    print("  --gate: the #1065 pose PASSES; the real shortfall FAILS")


TESTS = [
    test_an_oval_graze_is_clean,
    test_a_real_shortfall_is_the_graders_mm,
    test_a_pad_override_sets_the_requirement,
    test_same_net_pads_never_conflict,
    test_the_copper_is_read_at_the_models_pose,
    test_the_corpus_agrees_with_the_grade,
    test_the_caption_counts_the_graders_pairs,
    test_the_gate_reads_the_graders_pairs,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
