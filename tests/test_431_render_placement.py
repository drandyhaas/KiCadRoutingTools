"""`render_placement.py` -- the placement still (#431).

Geometry and metrics are asserted as NUMBERS, separately from the drawing.
Pixels are asserted as INVARIANTS (a toggle changes the image, an overlay lands
where the transform says), never as a committed checksum: Pillow's LANCZOS and
default-font rendering drift between versions and a stored hash would red-fail a
contributor's box for reasons unrelated to this code.

The load-bearing one is `test_rects_come_from_the_optimizers_own_model`. The
renderer must not own any coordinate arithmetic -- the moment someone inlines a
courtyard transform here, the picture starts disagreeing with the grader that
decided the placement, and every later "why does the render show X but the
optimizer say Y" is unanswerable.
"""

import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from PIL import ImageChops  # noqa: E402

import render_placement as RP  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(ROOT, 'kicad_files')
SEED = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
PLACED = os.path.join(KF, 'interf_u_unrouted_placed.kicad_pcb')
ULX = os.path.join(KF, 'ulx3s.kicad_pcb')


def _model(board=PLACED):
    return RP.PlacementModel(parse_kicad_pcb(board), board)


def _run(*args):
    return subprocess.run([sys.executable, '-X', 'utf8',
                           os.path.join(ROOT, 'py_tools', 'render_placement.py')] + list(args),
                          capture_output=True, text=True, cwd=ROOT, timeout=1800)


# --- model / geometry: numbers, not pixels ----------------------------------

def test_moved_parts_finds_the_tracked_delta():
    """interf_u ships a real 22-part placement delta; C4 moves 95.654mm."""
    moves = RP.moved_parts(parse_kicad_pcb(SEED), parse_kicad_pcb(PLACED))
    assert len(moves) == 22, len(moves)
    c4 = next(m for m in moves if m['reference'] == 'C4')
    assert abs(c4['dist'] - 95.654) < 1e-3, c4['dist']
    assert [m['reference'] for m in moves] == sorted(m['reference'] for m in moves), \
        "moves must be sorted (#457)"


def test_rects_come_from_the_optimizers_own_model():
    """THE anti-duplication assertion. Fails the day a courtyard transform is
    inlined into the renderer."""
    m = _model()
    for ref in sorted(m.parts())[:8]:
        assert m.rect(ref) == m.state.parts[ref].rect(), ref


def test_metrics_are_the_quenchs_own_numbers():
    m = _model()
    cost = m.state.total_cost()
    leg = m.state.legality_metrics()
    for k in ('crossings', 'hpwl', 'length'):
        assert m.metrics[k] == cost[k], k
    for k in ('overlap_area', 'oob_count'):
        assert m.metrics[k] == leg[k], k


def test_far_side_of_a_through_hole_part_is_its_drilled_pad_box():
    """`legality.rect_on` gates on this; drawing the courtyard on both sides
    would make the picture disagree with the grader."""
    m = _model()
    tht = [r for r in sorted(m.parts())
           if getattr(m.state.parts[r], 'has_tht', False)]
    assert tht, "fixture: expected through-hole parts"
    ref = tht[0]
    far, own = m.far_rect(ref), m.rect(ref)
    assert far is not None and far != own
    assert m.sides(ref) == {'F', 'B'}


def test_union_view_pads_and_floors():
    v = RP.union_view([(10.0, 10.0, 10.2, 10.2)], pad_mm=1.0, min_size=8.0)
    assert (v[2] - v[0]) >= 8.0 and (v[3] - v[1]) >= 8.0, \
        "one nudged 0402 must not fill the screen"
    cx = (v[0] + v[2]) / 2
    assert abs(cx - 10.1) < 1e-9, "the floor must expand about the centre"
    assert RP.union_view([]) is None


def test_clusters_are_deterministic_and_ordered():
    pts = [(0, 0), (0.5, 0), (40, 40), (40.5, 40), (41, 40)]
    a = RP.cluster_points(pts, 2.0)
    b = RP.cluster_points(list(reversed(pts)), 2.0)
    assert a == b, "cluster order must not depend on input order (#457)"
    assert len(a) == 2
    assert len(a[0]) == 3, "largest cluster first"


def test_zoom_group_resolves_exactly_like_route_py():
    """A name typed for `route.py --group` must work here unchanged."""
    from group_routing import block_refs
    from placement.groups import parse_sources
    pcb = parse_kicad_pcb(ULX)
    srcs = parse_sources('sheet')
    refs = block_refs(pcb, '58d913ec', srcs)
    assert refs and refs == block_refs(pcb, 'sheet:58d913ec', srcs)


def test_no_edge_cuts_board_reports_oob_unavailable():
    """The optimizer refuses a board with no outline, deliberately. The RENDERER
    must still work -- but must not print a 0 that reads as "clean"."""
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'noedge.kicad_pcb')
        txt = open(SEED, encoding='utf-8').read()
        # drop every Edge.Cuts graphic
        import re
        txt = re.sub(r'\(gr_(line|arc|circle|rect|poly)[^()]*?'
                     r'\(layer "Edge\.Cuts"\)[\s\S]*?\)\s*\)', '', txt)
        open(out, 'w', encoding='utf-8', newline='').write(txt)
        pcb = parse_kicad_pcb(out)
        if pcb.board_info.board_bounds is not None:
            pcb.board_info.board_bounds = None      # force the path
        m = RP.PlacementModel(pcb, out)
        assert m.no_outline is True
        assert m.metrics['oob_count'] is None, "oob must be unavailable, not 0"
        assert m.metrics['crossings'] is not None, "everything else stays exact"
    finally:
        shutil.rmtree(d, ignore_errors=True)


# --- pixels: invariants -----------------------------------------------------

def _panel(**kw):
    m = _model()
    o = {'borders': True, 'labels': True, 'ratsnest': True, 'pads': True,
         'ghosts': True, 'arrows': True, 'delta_first': True,
         'ratsnest_all': False}
    o.update(kw.pop('opts', {}))
    spec = RP.PanelSpec(m, opts=o, **kw)
    return RP.render_panel(spec, size=420, supersample=1)


def test_toggles_measurably_change_the_image():
    base = _panel()
    for off in ('borders', 'labels', 'ratsnest', 'pads'):
        assert ImageChops.difference(base, _panel(opts={off: False})).getbbox() \
            is not None, f"--no-{off} changed nothing"


def test_ghosts_and_arrows_need_a_before_board():
    """`_Part.seed_x/seed_y` equals the file position for a freshly parsed
    board, so a delta is NOT free from one file -- it needs --before."""
    m = _model()
    o = dict(borders=True, labels=False, ratsnest=False, pads=False,
             ghosts=True, arrows=True, delta_first=True, ratsnest_all=False)
    plain = RP.render_panel(RP.PanelSpec(m, opts=o), size=420, supersample=1)
    moves = RP.moved_parts(parse_kicad_pcb(SEED), parse_kicad_pcb(PLACED))
    with_delta = RP.render_panel(RP.PanelSpec(m, moves=moves, opts=o),
                                 size=420, supersample=1)
    assert ImageChops.difference(plain, with_delta).getbbox() is not None


def test_rendering_is_reproducible_in_this_environment():
    """The checksum test worth having: identical bytes twice, and across two
    subprocesses with different PYTHONHASHSEED. Catches set-iteration order
    reaching the drawing layer (#457) without pinning a Pillow version."""
    assert _panel().tobytes() == _panel().tobytes()
    d = tempfile.mkdtemp()
    try:
        outs = []
        for seed in ('0', '12345'):
            env = dict(os.environ, PYTHONHASHSEED=seed)
            p = os.path.join(d, f'r{seed}.png')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'render_placement.py'), PLACED,
                 '-o', p, '--size', '300', '--supersample', '1', '--quiet'],
                capture_output=True, text=True, cwd=ROOT, env=env, timeout=1800)
            assert r.returncode == 0, r.stderr[-400:]
            outs.append(open(p, 'rb').read())
        assert outs[0] == outs[1], "render differs under a different hash seed"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_per_side_panels_differ_on_a_two_sided_board():
    d = tempfile.mkdtemp()
    try:
        r = _run(ULX, '--per-side', '--size', '320', '--supersample', '1',
                 '-o', d, '--quiet')
        assert r.returncode == 0, r.stderr[-500:]
        pngs = sorted(f for f in os.listdir(d) if f.endswith('.png'))
        assert len(pngs) == 2, pngs
        a = open(os.path.join(d, pngs[0]), 'rb').read()
        b = open(os.path.join(d, pngs[1]), 'rb').read()
        assert a != b, "F and B panels are identical"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_caption_carries_the_verdict_not_just_a_title():
    """#431 limit 3: geometry is not causality. Without these numbers a reader
    adopts the wrong heuristic -- "lots moved, looks broken" and "barely moved,
    looks safe" are both wrong."""
    m = _model()
    moves = RP.moved_parts(parse_kicad_pcb(SEED), parse_kicad_pcb(PLACED))
    cap = RP.caption(RP.PanelSpec(m, moves=moves, label='x'),
                     {'failures': 3, 'iterations': 1234, 'vias': 7})
    for token in ('crossings', 'hpwl', 'overlap', 'failures 3', '22 moved'):
        assert token in cap, (token, cap)


def test_cli_writes_a_png_and_a_machine_readable_summary():
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'delta.png')
        r = _run(PLACED, '--before', SEED, '-o', out, '--json',
                 '--size', '320', '--supersample', '1')
        assert r.returncode == 0, r.stderr[-500:]
        assert os.path.getsize(out) > 1000
        line = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY:')]
        assert line, r.stdout[-400:]
        import json
        doc = json.loads(line[0].split('JSON_SUMMARY:', 1)[1])
        assert doc['moved'] == 22
        assert doc['metrics']['crossings'] > 0
        assert doc['panels'][0]['path'] == out
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_an_unplaced_board_still_renders_and_says_so():
    """The renderer WARNS where the placement CLIs refuse -- seeing the state is
    the point -- and frames the parts, or a pile renders as a dot in a corner."""
    d = tempfile.mkdtemp()
    try:
        from placement.writer import write_placed_output
        src = os.path.join(KF, 'lvds_converter_dualclk.kicad_pcb')
        pcb = parse_kicad_pcb(src)
        bb = pcb.board_info.board_bounds
        pile = os.path.join(d, 'pile.kicad_pcb')
        write_placed_output(src, pile, [{'reference': r,
                                         'new_x': (bb[0] + bb[2]) / 2,
                                         'new_y': (bb[1] + bb[3]) / 2,
                                         'new_rotation': 0.0}
                                        for r in pcb.footprints])
        out = os.path.join(d, 'pile.png')
        r = _run(pile, '-o', out, '--size', '320', '--supersample', '1', '--json')
        assert r.returncode == 0, r.stderr[-500:]
        assert 'does not look PLACED' in r.stdout
        assert os.path.getsize(out) > 500
        import json
        doc = json.loads([l for l in r.stdout.splitlines()
                          if l.startswith('JSON_SUMMARY:')][0].split(':', 1)[1])
        assert doc['unplaced'] is True
        assert doc['panels'][0]['view'] is not None, \
            "an unplaced board must be framed to the PARTS, not the outline"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_ratsnest_nets_draws_only_the_named_nets():
    """Between "the nets that moved" and "every net on the board" is the case
    that actually comes up: the handful you are chasing. Naming them also makes
    their parts prominent -- a set of wires with nothing identified is not much
    of an answer."""
    m = _model()
    all_ids = {n.net_id for n in m.pcb.nets.values()
               if n.net_id > 0 and len(n.pads) > 1}
    pick = m.net_ids_matching(['/BIT*'])
    assert pick, "fixture: expected some matching nets"
    assert pick < all_ids, "the pattern must select a strict subset"

    o = dict(borders=False, labels=False, ratsnest=True, pads=False,
             ghosts=False, arrows=False, delta_first=True, ratsnest_all=False)
    plain = RP.render_panel(RP.PanelSpec(m, opts=o), size=420, supersample=1)
    named = RP.render_panel(RP.PanelSpec(m, pick_nets=pick, opts=o),
                            size=420, supersample=1)
    assert ImageChops.difference(plain, named).getbbox() is not None
    # the picked colour must actually appear, and only when asked for
    assert RP.C_AIR_PICK in {c for _n, c in named.convert('RGB').getcolors(1 << 20)}
    assert RP.C_AIR_PICK not in {c for _n, c in plain.convert('RGB').getcolors(1 << 20)}


def test_ratsnest_nets_uses_the_shared_net_filter():
    """Same glob semantics as route.py --nets, exclusions included. A second
    matcher in a viewer would be a second set of surprises."""
    m = _model()
    from net_queries import matches_net_filter
    pats = ['*', '!/BIT*']
    got = m.net_ids_matching(pats)
    want = {n.net_id for n in m.pcb.nets.values()
            if n.net_id > 0 and n.name and matches_net_filter(n.name, pats)}
    assert got == want and got, "exclusion patterns must behave as elsewhere"
    assert not (got & m.net_ids_matching(['/BIT*']))


def test_ratsnest_nets_cli_reports_an_empty_match():
    """Silently rendering nothing would read as "this net has no airwires"."""
    d = tempfile.mkdtemp()
    try:
        r = _run(PLACED, '-o', os.path.join(d, 'x.png'), '--size', '260',
                 '--supersample', '1', '--ratsnest-nets', 'NOSUCHNET*')
        assert r.returncode == 0, r.stderr[-300:]
        assert 'no nets match' in (r.stdout + r.stderr)
    finally:
        shutil.rmtree(d, ignore_errors=True)



def test_single_panel_accepts_a_directory_target():
    """`-o` is documented as "PNG path (one panel) or directory", but directory
    handling used to be gated on len(panels) > 1, so every ONE-panel run with a
    directory died in PIL with "unknown file extension". That is the plain render
    and --focus, i.e. two of the four situations the skill says to render in."""
    d = tempfile.mkdtemp()
    try:
        for extra in ([], ['--focus']):
            sub = os.path.join(d, 'out' + str(len(extra)))
            os.makedirs(sub)
            r = _run(PLACED, '-o', sub + os.sep, '--size', '320',
                     '--supersample', '1', *extra)
            assert r.returncode == 0, (extra, r.stderr[-600:])
            pngs = [f for f in os.listdir(sub) if f.endswith('.png')]
            assert pngs, f"{extra}: nothing written into the directory"
        # and a plain file target is still a file, not a directory
        f = os.path.join(d, 'one.png')
        r = _run(PLACED, '-o', f, '--size', '320', '--supersample', '1')
        assert r.returncode == 0, r.stderr[-600:]
        assert os.path.isfile(f)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_ignore_nets_reproduces_place_optimize_exactly():
    """The skill calls this the re-measurement channel: render the WRITTEN board
    and the metrics must reproduce place_optimize's JSON_SUMMARY. They cannot
    unless both exclude the same nets -- place_optimize is always given
    --ignore-nets for the plane rails, and this tool had no such flag, so the
    check compared two different net sets and always "failed"."""
    import json
    d = tempfile.mkdtemp()
    try:
        placed = os.path.join(d, 'p.kicad_pcb')
        opt = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_placer', 'place_optimize.py'),
             SEED, placed, '--max-displacement', '1', '--ignore-nets', 'GND'],
            capture_output=True, text=True, cwd=ROOT, timeout=1800)
        assert opt.returncode == 0, opt.stderr[-600:]
        # Line-based parse (the documented convention): the B1 banner appends
        # an EXIT= line after JSON_SUMMARY, so a whole-stdout split no longer
        # yields bare JSON.
        q = json.loads([l for l in opt.stdout.splitlines()
                        if l.startswith('JSON_SUMMARY:')][0]
                       .split('JSON_SUMMARY:', 1)[1])

        def rendered(*extra):
            r = _run(placed, '--json', '-o', os.path.join(d, 'r.png'),
                     '--size', '320', '--supersample', '1', *extra)
            assert r.returncode == 0, r.stderr[-600:]
            line = [l for l in r.stdout.splitlines()
                    if l.startswith('JSON_SUMMARY:')][0]
            return json.loads(line.split('JSON_SUMMARY:', 1)[1])['metrics']

        same = rendered('--ignore-nets', 'GND')
        assert same['crossings'] == q['crossings_after'], (same, q)
        assert abs(same['hpwl'] - q['hpwl_after']) < 1e-9, (same, q)
        # and without the flag it legitimately differs, or the test proves nothing
        assert rendered()['crossings'] != q['crossings_after']
    finally:
        shutil.rmtree(d, ignore_errors=True)


TESTS = [
    test_moved_parts_finds_the_tracked_delta,
    test_rects_come_from_the_optimizers_own_model,
    test_metrics_are_the_quenchs_own_numbers,
    test_far_side_of_a_through_hole_part_is_its_drilled_pad_box,
    test_union_view_pads_and_floors,
    test_clusters_are_deterministic_and_ordered,
    test_zoom_group_resolves_exactly_like_route_py,
    test_no_edge_cuts_board_reports_oob_unavailable,
    test_toggles_measurably_change_the_image,
    test_ghosts_and_arrows_need_a_before_board,
    test_rendering_is_reproducible_in_this_environment,
    test_per_side_panels_differ_on_a_two_sided_board,
    test_the_caption_carries_the_verdict_not_just_a_title,
    test_cli_writes_a_png_and_a_machine_readable_summary,
    test_an_unplaced_board_still_renders_and_says_so,
    test_ratsnest_nets_draws_only_the_named_nets,
    test_ratsnest_nets_uses_the_shared_net_filter,
    test_ratsnest_nets_cli_reports_an_empty_match,
    test_single_panel_accepts_a_directory_target,
    test_ignore_nets_reproduces_place_optimize_exactly,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")


# --- run-4 G additions: file JSON, instrument echo, checklist, -o siblings --

def test_per_side_png_target_writes_sibling_files():
    """Run-4 G4: `-o wk/x.png --per-side` used to os.makedirs('wk/x.png') --
    a DIRECTORY literally named x.png -- which make_film globbed as a card
    and died reading. A .png target now yields stem-suffixed sibling FILES."""
    d = tempfile.mkdtemp()
    try:
        f = os.path.join(d, 'x.png')
        r = _run(ULX, '-o', f, '--per-side', '--size', '320',
                 '--supersample', '1')
        assert r.returncode == 0, r.stderr[-600:]
        assert not os.path.isdir(f), "-o file target must not become a dir"
        sibs = sorted(os.listdir(d))
        assert 'x_F.png' in sibs and 'x_B.png' in sibs, sibs
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_json_out_writes_a_file_with_instrument_and_checklist():
    """Run-4 G1/G2/G5: --json-out writes the document to a FILE (no more
    grep-and-strip), the instrument block makes a before/after series
    provably same-instrument, and the checklist carries mandate 8's four
    answers with refs, channel-labelled."""
    import json as _json
    d = tempfile.mkdtemp()
    try:
        js = os.path.join(d, 'r.json')
        r = _run(PLACED, '--before', SEED, '-o', os.path.join(d, 'r.png'),
                 '--json-out', js, '--size', '320', '--supersample', '1',
                 '--ignore-nets', 'GND', '--expect-moved', '22')
        assert r.returncode == 0, r.stderr[-600:]
        doc = _json.load(open(js, encoding='utf-8'))
        inst = doc['instrument']
        assert inst['ignore_nets'] == ['GND']
        assert inst['board'].endswith('interf_u_unrouted_placed.kicad_pcb')
        assert inst['before'].endswith('interf_u_unrouted.kicad_pcb')
        assert inst['size'] == 320
        cl = doc['checklist']
        # run-6: 'b_overlap_pairs' was renamed to its honest channel name
        # (it carried pad CLEARANCE pairs) and the body channel was added.
        assert set(cl) == {'a_off_outline', 'b_pad_clearance_pairs',
                           'b_body_overlap_pairs',
                           'c_hole_conflicts', 'c_locked_refs', 'd_moved'}
        assert cl['d_moved'] == {'moved': 22, 'expected': 22, 'match': True}
        assert doc['moved_refs'] and all(
            set(m) == {'reference', 'dist'} for m in doc['moved_refs'])
        # channels are labelled -- the run-3 confusion was an unlabelled
        # two-channel disagreement
        assert set(cl['a_off_outline']) == {'pad_copper', 'courtyard'}
        # and the stdout line still exists for back-compat
        assert any(l.startswith('JSON_SUMMARY:')
                   for l in r.stdout.splitlines())
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_expect_moved_mismatch_is_reported_not_fatal():
    import json as _json
    d = tempfile.mkdtemp()
    try:
        js = os.path.join(d, 'r.json')
        r = _run(PLACED, '--before', SEED, '-o', os.path.join(d, 'r.png'),
                 '--json-out', js, '--size', '320', '--supersample', '1',
                 '--expect-moved', '3')
        assert r.returncode == 0
        cl = _json.load(open(js, encoding='utf-8'))['checklist']
        assert cl['d_moved']['match'] is False
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_focus_without_summary_json_clusters_legality_findings():
    """--focus with no --summary-json used to warn and emit nothing (run-4
    G7); it now clusters the LEGALITY findings -- which need no route summary
    -- into pocket panels instead of refusing. Pin the refusal-free path: no
    warning, and the pocket siblings actually land next to the base render."""
    d = tempfile.mkdtemp()
    try:
        r = _run(PLACED, '-o', os.path.join(d, 'r.png'), '--focus',
                 '--size', '320', '--supersample', '1')
        assert r.returncode == 0
        assert '--focus emits nothing without --summary-json' not in r.stderr
        assert os.path.exists(os.path.join(d, 'r.png'))
        pockets = sorted(f for f in os.listdir(d) if 'pocket' in f)
        assert pockets, (r.stdout, r.stderr)
        # This fixture HAS legality findings, so the empty-board NOTE must
        # not fire either -- the pockets ARE the focus output.
        assert 'found nothing to focus on' not in r.stderr, r.stderr
    finally:
        shutil.rmtree(d, ignore_errors=True)


# --- D8: the render must be able to say its net list missed -----------------
#
# There was NO field in the instrument block where "61 requested, 51 matched"
# could ever appear, and that absence is why a mangled net list went undetected:
# board_score published `--ignore-nets` candidates double-escaped, 10 of 61
# matched nothing, and the render reported hpwl +45.6% / crossings +129.5% on an
# identical board. Both renders were internally consistent. The operator had to
# compare declared against matched by hand, outside the tool.

def test_net_pattern_report_counts_declared_against_matched():
    """The unit the JSON field is built from -- no rendering needed."""
    pcb = parse_kicad_pcb(PLACED)
    names = [n.name for n in pcb.nets.values() if n.name]
    real = names[0]
    rep = RP.net_pattern_report(pcb, [real, 'NOSUCHNET', 'ALSO_NOT_HERE*'],
                                '--ignore-nets')
    assert rep['requested'] == 3, rep
    assert rep['matched'] == 1, rep
    assert rep['unmatched'] == ['ALSO_NOT_HERE*', 'NOSUCHNET'], rep
    assert rep['nets_matched'] >= 1, rep
    # An empty list is not a failure, and must still produce the fields --
    # a key that appears only on failure is a key no reader looks for.
    empty = RP.net_pattern_report(pcb, None, '--ratsnest-nets')
    assert empty == {'flag': '--ratsnest-nets', 'requested': 0, 'matched': 0,
                     'unmatched': [], 'nets_matched': 0}, empty


def test_a_net_list_that_missed_is_reported_in_the_json_and_on_stderr():
    """The single change that would have caught the double-escaping."""
    import json as _json
    d = tempfile.mkdtemp()
    try:
        js = os.path.join(d, 'r.json')
        # 'GND' exists on this board; the other two cannot. The second is the
        # measured shape exactly -- a real net name carrying one backslash too
        # many, which is indistinguishable from a net that does not exist.
        r = _run(PLACED, '-o', os.path.join(d, 'r.png'), '--json-out', js,
                 '--size', '300', '--supersample', '1',
                 '--ignore-nets', 'GND', 'NOSUCHNET', '/GPIO10\\\\OE3#')
        assert r.returncode == 0, r.stderr[-600:]

        rep = _json.load(open(js, encoding='utf-8'))['instrument']['net_lists']
        ig = rep['ignore_nets']
        assert (ig['requested'], ig['matched']) == (3, 1), ig
        assert ig['unmatched'] == ['/GPIO10\\\\OE3#', 'NOSUCHNET'], ig
        assert 'ratsnest_nets' in rep, 'both net lists must be reported'

        # ...and it must SAY so, unprompted. A JSON field nobody opens is not
        # a warning.
        assert '3 requested, 1 matched' in r.stderr, r.stderr[-600:]
        assert 'NOSUCHNET' in r.stderr, r.stderr[-600:]
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_fully_matching_net_list_stays_quiet():
    """The warning has to mean something, so it must not fire on a good run."""
    d = tempfile.mkdtemp()
    try:
        r = _run(PLACED, '-o', os.path.join(d, 'r.png'), '--size', '300',
                 '--supersample', '1', '--ignore-nets', 'GND')
        assert r.returncode == 0, r.stderr[-600:]
        assert 'matched NO net' not in r.stderr, r.stderr[-400:]
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_legality_findings_cached_once_per_model():
    m = _model()
    a = RP.legality_findings(m)
    assert RP.legality_findings(m) is a, "findings must be computed once"
    for key in ('oob_refs_pad_copper', 'oob_refs_courtyard',
                'pad_conflict_pairs_refs', 'hole_conflict_pairs_refs',
                'locked_refs'):
        assert key in a
