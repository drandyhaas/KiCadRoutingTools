#!/usr/bin/env python3
"""Question-scoped crops: --view on the renderers, --render on check_drc.

Run 5 read zero images because the image types worth reading did not exist:
nothing drew DRC violation positions, and the crop seam (BoardRenderer.view)
was library-only. These tests pin the new surfaces: parse_view's contract,
the route_render --view CLI, and check_drc's violation-cluster panels
(clustering, red-ring markers, caption, accepted-skip, panel cap).
"""
import os
import subprocess
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for p in (ROOT, _TESTS, os.path.join(ROOT, 'rust_router'),
          os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools'),
          os.path.join(ROOT, 'py_placer')):   # #522: the CLIs moved into packages
    if p not in sys.path:
        sys.path.insert(0, p)
from run_utils import tool as _tool  # #522: resolve a moved CLI, loudly

# #522 reorg + skill merge: engine -> py_router/, placer -> py_placer/,
# board_score.py -> the placement-and-routing skill. Without these roots the
# imports below raise and the test never runs (it reports as a failure while
# asserting nothing).
_R522 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p522 in ('py_router', 'py_placer',
              os.path.join('.claude', 'skills',
                           'plan-pcb-placement-and-routing', 'scripts')):
    _d522 = os.path.join(_R522, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)


BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def test_parse_view_contract():
    from route_render import parse_view
    assert parse_view(None) is None
    assert parse_view('1,2,3,4') == (1.0, 2.0, 3.0, 4.0)
    # corner order normalizes; spaces work like commas
    assert parse_view('3 4 1 2') == (1.0, 2.0, 3.0, 4.0)
    try:
        parse_view('1,2,3')
    except ValueError:
        pass
    else:
        raise AssertionError('3-value view must be refused')
    print("  PASS: parse_view normalizes corners and refuses malformed rects")


def test_ref_and_ruler_overlays_are_scale_gated():
    """Refs/ruler default ON for a crop, OFF whole-board -- and the ref
    overlay draws nothing when zoomed out (labels would blanket the board)."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from kicad_parser import parse_kicad_pcb
    from route_render import BoardRenderer, ref_label_overlay
    pcb = parse_kicad_pcb(BOARD)
    calls = []

    class _Probe:
        mode = 'RGB'
        def line(self, *a, **k): calls.append('line')
        def rectangle(self, *a, **k): calls.append('rect')
        def text(self, *a, **k): calls.append('text')
        def textbbox(self, *a, **k): return (0, 0, 10, 10)

    fn = ref_label_overlay(pcb)
    fn(_Probe(), BoardRenderer(pcb, size=800))            # whole board: gated
    assert not calls, "whole-board scale must draw no ref labels"
    fn(_Probe(), BoardRenderer(pcb, size=800, view=(60, 30, 70, 40)))
    assert 'text' in calls, "crop scale must draw ref labels"
    print("  PASS: ref labels gate on scale (crop yes, whole-board no)")


def test_route_render_view_cli():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'crop.png')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            _tool('route_render.py'), BOARD,
                            '-o', out, '--view', '60,30,80,45'],
                           capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr[-500:]
        assert os.path.isfile(out) and os.path.getsize(out) > 1000
    print("  PASS: route_render --view writes a labeled crop")


def test_drc_panels_cluster_mark_and_cap():
    """render_violation_panels off hand-made records: two spatial clusters
    become two panels; 'accepted' records are skipped; the cap prints what it
    dropped."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from check_drc import render_violation_panels, _violation_xy
    # _violation_xy: every record shape resolves to a point
    assert _violation_xy({'closest_pt1': (1.0, 2.0)}) == (1.0, 2.0)
    assert _violation_xy({'via_loc': (3.0, 4.0)}) == (3.0, 4.0)
    assert _violation_xy({'seg_loc': (0.0, 0.0, 2.0, 4.0)}) == (1.0, 2.0)
    assert _violation_xy({'type': 'x'}) is None

    def v(x, y, typ='segment-segment', accepted=False):
        d = {'type': typ, 'net1': 'A', 'net2': 'B', 'closest_pt1': (x, y)}
        if accepted:
            d['accepted'] = True
        return d

    # two clusters ~40mm apart on the real fixture's board area
    viols = [v(70.0, 35.0), v(70.5, 35.2), v(71.0, 35.4),
             v(110.0, 35.0), v(110.3, 35.1),
             v(70.2, 35.1, accepted=True)]           # waived: must not render
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'panels')
        paths = render_violation_panels(BOARD, viols, out)
        assert len(paths) == 2, paths
        assert all(os.path.getsize(p) > 1000 for p in paths)
        # largest cluster first: 3 items (the accepted one excluded)
        assert 'cluster1' in paths[0]
        # cap: same records with max_panels=1 renders one and reports the rest
        import io
        from contextlib import redirect_stdout
        buf = io.StringIO()
        with redirect_stdout(buf):
            paths2 = render_violation_panels(BOARD, viols,
                                             os.path.join(td, 'capped'),
                                             max_panels=1)
        assert len(paths2) == 1
        assert 'NOT rendered' in buf.getvalue()
    print("  PASS: 2 clusters -> 2 panels, waived skipped, cap reported")


def test_quiet_keeps_keys_off_stdout_when_json_out_carries_them():
    """Run 24, finding A-1: the review gates order 'view the image BEFORE the
    keys', then prescribe a render command whose unconditional JSON_SUMMARY
    echo put the keys on screen first. --quiet with --json-out now suppresses
    the echo and the describe text (the file carries both); --quiet WITHOUT
    --json-out must still print the summary -- data is never silenced into
    nowhere."""
    rp = _tool('render_placement.py')
    board = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'r.json')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', rp, board, '--clearance', '0.15',
             '--json-out', out, '--quiet', '-o', os.path.join(td, 'r.png')],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stdout[-400:]
        assert 'JSON_SUMMARY' not in r.stdout, 'the echo defeats blind-first'
        assert 'WHAT THIS PANEL SHOWS' not in r.stdout
        assert os.path.getsize(out) > 100, 'the file must still carry the doc'
        r2 = subprocess.run(
            [sys.executable, '-X', 'utf8', rp, board, '--clearance', '0.15',
             '--json', '--quiet'],
            capture_output=True, text=True, cwd=ROOT)
        assert 'JSON_SUMMARY' in r2.stdout, \
            'without --json-out the summary must still print somewhere'
    print("  PASS: --quiet + --json-out is blind; --quiet alone never "
          "silences data into nowhere")


if __name__ == '__main__':
    for k, fn in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            fn()
    print("ALL PASS")
