#!/usr/bin/env python3
"""Board-declared DRC floors resolve the same way on every consumer.

A declared 0 is UNSET (KiCad writes 0 for "not configured"), an explicit
value wins, and the fallback is only used when the board declares nothing --
with the source announced. Pins list_nets.board_floor / resolve_cli_floor,
the four routing mains' floor reads, and the qfn underpad drill floor wrap.

Run: python3 -X utf8 tests/test_board_floors.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522 layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522 layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522 layout
os.environ.setdefault('KRT_NO_BANNER', '1')

import routing_defaults as defaults                            # noqa: E402

FAILURES = []

def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


# --- D9: three more instruments that substituted a constant for the board ----
#
# check_channels and check_assembly defaulted --clearance to routing_defaults
# 0.25 and never read the board; render_placement documented no default at all.
# Measured on a board whose floor is 0.2 with a 0.254 track: check_channels at
# its old constants reported 334 escape lanes where the board's own floor gives
# 399 -- a 65-lane understatement, and an invented deficit on a face that had
# none. A phantom deficit steers a placement search at the thing that is not
# wrong.

def _fixture(tmp, clearance=0.1, track=0.15, extra_rules=None):
    """A board that DECLARES a floor, built to order.

    Most boards in kicad_files/ declare nothing -- but NOT all of them, which
    is the correction in the module docstring: flat_hierarchy and
    routed_output ship committed .kicad_pro siblings. Those two are used
    directly by _q5_committed_projects_are_real; this fixture exists so the
    resolver's OTHER branches (a value the corpus does not happen to declare,
    the 0.0 trap, a malformed project) are reachable at all."""
    import shutil
    src = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    dst = os.path.join(tmp, 'declared.kicad_pcb')
    shutil.copyfile(src, dst)
    import json as _json
    rules = {'min_clearance': 0.0}        # the trap: KiCad writes 0 for "unset"
    rules.update(extra_rules or {})
    with open(os.path.splitext(dst)[0] + '.kicad_pro', 'w',
              encoding='utf-8') as f:
        _json.dump({'net_settings': {'classes': [
            {'name': 'Default', 'clearance': clearance, 'track_width': track}]},
            'board': {'design_settings': {'rules': rules}}}, f)
    return dst


def _d9_shared_resolver():
    """One helper, so the three instruments cannot each be wrong differently."""
    import tempfile
    from list_nets import board_floor
    with tempfile.TemporaryDirectory() as tmp:
        b = _fixture(tmp)
        print('board_floor resolves board-first')
        check('an explicit value wins and is labelled cli',
              board_floor(b, 'clearance', 0.42, 0.25) == (0.42, 'cli'))
        check('unset reads the board netclass',
              board_floor(b, 'clearance', None, 0.25) == (0.1, 'board netclass'),
              str(board_floor(b, 'clearance', None, 0.25)))
        check('track width too',
              board_floor(b, 'track_width', None, 0.3) == (0.15, 'board netclass'))
        # THE min_clearance TRAP. The fixture declares min_clearance 0.0, which
        # KiCad writes for "not configured". Reading it as a real floor would
        # resolve a board asking for 0.1 down to 0.0 and relax every consumer
        # to nothing -- so `clearance` must never fall back to that constraint.
        check('min_clearance 0.0 does NOT override the netclass',
              board_floor(b, 'clearance', None, 0.25)[0] == 0.1)
        # A constraint-only floor, which no netclass expresses (D11's case).
        b2 = _fixture(tmp, extra_rules={'min_hole_clearance': 0.25})
        check('a hole floor comes from the board constraint',
              board_floor(b2, 'hole_clearance', None, 0.2)
              == (0.25, 'board constraint'),
              str(board_floor(b2, 'hole_clearance', None, 0.2)))

    print('a board declaring nothing still falls back, and says so')
    nodecl = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
    check('fallback is labelled fixed default',
          board_floor(nodecl, 'clearance', None, 0.25) == (0.25, 'fixed default'))
    check('"could not read the project" is not "declares nothing"',
          board_floor(os.path.join(tmp, 'nope.kicad_pcb'), 'clearance',
                      None, 0.25, design_rules=_Boom())[1] == 'unreadable project')

    # THE 0.0 TRAP, in the OLDER helper. board_floor guards it; board_floor_knobs
    # did not, and render_placement is wired to board_floor_knobs -- so a project
    # declaring `min_copper_edge_clearance: 0.0` (KiCad's "not configured") gave
    # the placement model a REAL edge floor of zero where it had used 0.55.
    #
    # WHAT THAT ACTUALLY COSTS, measured on interf_u_unrouted_placed with a
    # 0.0-declaring project (NOT the "every edge/halo term collapses" I first
    # wrote in the commit message -- quench takes max(clearance, edge), so 0.0
    # degrades to the 0.2 pad clearance rather than to nothing, and `edge` and
    # `halo` do not move at all):
    #
    #     edge  15.516 -> 15.516     halo 303.803 -> 303.803   (unchanged)
    #     oob_amount  22.039 -> 24.489      oob_area 398.51 -> 445.44
    #
    # i.e. it UNDER-REPORTS off-board pad copper by ~11%, which CLAUDE.md names
    # as the top-priority placement defect ("converts one-for-one into unrouted
    # and broken"). Under-reporting that is the harm; it is smaller and more
    # specific than the sweeping claim, and the number is the point.
    # The two helpers must agree about the one trap this module documents.
    print('a declared 0.0 is UNSET in both floor helpers, not a floor of zero')
    from list_nets import board_floor_knobs
    import tempfile as _tf
    with _tf.TemporaryDirectory() as z:
        b = _fixture(z, clearance=0.2,
                     extra_rules={'min_copper_edge_clearance': 0.0})
        _clr, _edge, knobs = board_floor_knobs(b)
        check('board_floor_knobs does not return a 0.0 edge floor',
              _edge == 0.55 and knobs['board_edge_clearance']['source']
              == 'fixed default', str(knobs['board_edge_clearance']))
        check('board_floor agrees',
              board_floor(b, 'board_edge_clearance', None, 0.55)
              == (0.55, 'fixed default'))


# --- Q3: the ROUTING half read the same two constraints its own way ---------
#
# D9 put the `> 0` guard in the placement helpers and stopped there. The four
# routing mains each kept a private copy of
# `board_constraint(...) if ... is not None else <default>` -- eight sites, an
# `is not None` test with no positivity guard -- so one declared floor got two
# answers depending on which half of the loop asked.
#
# On a board declaring `min_copper_edge_clearance: 0.0` (KiCad's "not
# configured"): placement / floorplan / render resolved 0.55 [fixed default];
# route.py took a REAL floor of 0.0 while printing "using the board
# min_copper_edge_clearance 0.0mm" -- announcing a declaration that was not
# one. The plane engines were worse than misleading: their inset fell from
# PLANE_EDGE_CLEARANCE 0.5 to 0.0, on the very comment that says they keep 0.5
# "only when the board declares no edge rule of its own". The GUI's plane tab
# was already right (swig_gui._effective_plane_edge_clearance guards `> 1e-9`),
# so this was a CLI-only divergence from the GUI as well.

_CLI_FLOOR_SITES = ('py_router/route.py', 'py_router/route_diff.py',
                    'py_router/route_planes.py', 'py_router/repair_planes.py')
_RAW_KEYS = ('min_hole_to_hole', 'min_copper_edge_clearance')


def _q3_routing_half_uses_the_same_resolver():
    import io
    import contextlib
    import tempfile
    from list_nets import (board_floor, board_floor_knobs, resolve_cli_floor)

    print('one declared floor, one number, on both halves of the loop')
    with tempfile.TemporaryDirectory() as tmp:
        # A board that DECLARES a positive edge rule. Both halves must land on
        # it -- this is the "one number" claim in its testable form.
        decl = _fixture(tmp, extra_rules={'min_copper_edge_clearance': 0.3,
                                          'min_hole_to_hole': 0.3})
        _c, edge_place, knobs = board_floor_knobs(decl)
        with contextlib.redirect_stdout(io.StringIO()):
            edge_sig = resolve_cli_floor(decl, 'board_edge_clearance', None,
                                         defaults.BOARD_EDGE_CLEARANCE, '--x')
            edge_pln = resolve_cli_floor(decl, 'board_edge_clearance', None,
                                         defaults.PLANE_EDGE_CLEARANCE, '--x')
        check('a DECLARED 0.3 edge rule is 0.3 on the placement half',
              edge_place == 0.3
              and knobs['board_edge_clearance']['source'] == 'board constraint',
              str(knobs['board_edge_clearance']))
        check('...and 0.3 on the routing half, signal and plane alike',
              edge_sig == 0.3 and edge_pln == 0.3,
              f'signal {edge_sig}, plane {edge_pln}')

    print('a DECLARED 0.0 is UNSET on the routing half too')
    with tempfile.TemporaryDirectory() as tmp:
        z = _fixture(tmp, extra_rules={'min_copper_edge_clearance': 0.0,
                                       'min_hole_to_hole': 0.0})
        _c, edge_place, knobs = board_floor_knobs(z)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            edge_sig = resolve_cli_floor(z, 'board_edge_clearance', None,
                                         defaults.BOARD_EDGE_CLEARANCE, '--x')
            edge_pln = resolve_cli_floor(z, 'board_edge_clearance', None,
                                         defaults.PLANE_EDGE_CLEARANCE, '--x')
            h2h = resolve_cli_floor(z, 'hole_to_hole', None,
                                    defaults.HOLE_TO_HOLE_CLEARANCE, '--y')
        said = buf.getvalue()

        # BOTH halves must reach the same VERDICT about the board: it declared
        # nothing. The fallbacks then legitimately differ -- BOARD_EDGE_CLEARANCE
        # 0.0 is a deliberate "no edge rule, use the copper-copper clearance"
        # sentinel (route.py:896, obstacle_map.py:797), the plane engines want
        # 0.5, the placement model 0.55 -- and are NOT unified here.
        check('placement half: declared 0.0 reads as fixed default',
              edge_place == 0.55
              and knobs['board_edge_clearance']['source'] == 'fixed default',
              str(knobs['board_edge_clearance']))
        check('routing half agrees the board declared nothing',
              board_floor(z, 'board_edge_clearance', None, 0.0)[1]
              == 'fixed default'
              and board_floor(z, 'hole_to_hole', None, 0.2)[1]
              == 'fixed default')
        # The value fixes, each measured against what the old raw read gave.
        check('plane inset stays PLANE_EDGE_CLEARANCE (was 0.0)',
              edge_pln == defaults.PLANE_EDGE_CLEARANCE, f'got {edge_pln}')
        check('hole-to-hole stays the packaged floor (was 0.0)',
              h2h == defaults.HOLE_TO_HOLE_CLEARANCE, f'got {h2h}')
        check('the signal sentinel is unchanged at 0.0',
              edge_sig == defaults.BOARD_EDGE_CLEARANCE, f'got {edge_sig}')
        # And it must stop claiming the board said so.
        check('the print no longer attributes 0.0 to the board',
              'min_copper_edge_clearance' not in said
              and 'min_hole_to_hole' not in said, said.strip())
        check('the print names its source in the placement vocabulary',
              said.count('[fixed default]') == 3, said.strip())

    # THE WIRING. Every check above passes with all eight call sites reverted
    # to the raw read -- the exact hole D10 was pulled up for. So assert the
    # mains actually route through the resolver, and that the raw read is gone.
    print('all four routing mains go through the one resolver')
    import ast
    for fn in _CLI_FLOOR_SITES:
        src = open(os.path.join(ROOT, fn), encoding='utf-8').read()
        tree = ast.parse(src)
        floors = set()
        raw = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            fname = getattr(node.func, 'id', None) or getattr(
                node.func, 'attr', None)
            first_str = [a.value for a in node.args
                         if isinstance(a, ast.Constant)
                         and isinstance(a.value, str)]
            if fname == 'resolve_cli_floor':
                floors.update(first_str)
            elif fname == 'board_constraint':
                raw += [s for s in first_str if s in _RAW_KEYS]
        check(f'{fn}: both floors via resolve_cli_floor',
              {'hole_to_hole', 'board_edge_clearance'} <= floors,
              f'found {sorted(floors)}')
        check(f'{fn}: no raw board_constraint read of either key',
              not raw, f'raw reads: {raw}')

    # ...and the mains ACTUALLY PRINT it, which the AST check cannot show. Each
    # runs in ~1.3s on a net that matches nothing, so this is the CLI's own
    # answer rather than a model of it. At 62f7c13^ these lines read "using the
    # board min_copper_edge_clearance 0.0mm" and the plane engines resolved 0.0.
    print('...and each main prints the resolved floor, crediting the right source')
    import subprocess
    import tempfile
    # (flag-shape, edge-floor-expected) per main; hole-to-hole is 0.2 for all.
    mains = [
        ('py_router/route.py', lambda i, o: [i, o, '--nets', '__none__'], 0.0),
        ('py_router/route_diff.py', lambda i, o: [i, '--output', o, '--nets', '__none__'], 0.0),
        ('py_router/route_planes.py', lambda i, o: [i, '--output', o, '--nets', '__none__',
                                          '--plane-layers', 'B.Cu'], 0.5),
        ('py_router/repair_planes.py',
         lambda i, o: [i, '--output', o, '--nets', '__none__',
                       '--plane-layers', 'B.Cu'], 0.5),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        z = _fixture(tmp, extra_rules={'min_copper_edge_clearance': 0.0,
                                       'min_hole_to_hole': 0.0})
        for fn, argv, edge in mains:
            out = os.path.join(tmp, 'o_' + fn.replace('.py', '') + '.kicad_pcb')
            r = subprocess.run([sys.executable, '-X', 'utf8', fn] + argv(z, out),
                               cwd=ROOT, capture_output=True, text=True,
                               timeout=300)
            lines = [ln.strip() for ln in (r.stdout or '').splitlines()
                     if ln.startswith('--hole-to-hole-clearance not given')
                     or ln.startswith('--board-edge-clearance not given')]
            said = ' '.join(lines)
            check(f'{fn}: prints both resolved floors',
                  len(lines) == 2, said or (r.stderr or '')[-300:])
            check(f'{fn}: hole-to-hole 0.2 [fixed default], not a declared 0.0',
                  f'--hole-to-hole-clearance not given; using 0.2mm '
                  f'[fixed default].' in said, said)
            check(f'{fn}: board edge {edge} [fixed default]',
                  f'--board-edge-clearance not given; using {edge}mm '
                  f'[fixed default].' in said, said)
            check(f'{fn}: does not credit the board for what it left unset',
                  'min_copper_edge_clearance' not in said
                  and 'min_hole_to_hole' not in said, said)


# --- Q4: board_floor is NOT raise-only, and a drill consumer must wrap it ---
#
# board_floor returns the declared value whenever it is positive, with no
# max() against the fallback. For most of its table that is exactly right --
# check_channels / check_assembly must grade at the board's own clearance even
# when it is BELOW their default. For a DRILL floor it is a fab hazard, and the
# qfn underpad escape claimed "Raise-only in practice" while doing no such
# thing: a project declaring `min_hole_to_hole: 0.10` spaced that run's drills
# at 0.10, under the 0.20 JLC floor. `resolve_hole_clearance` is raise-only
# only because ITS consumers wrap it; this is the same wrap, and the same
# demonstration.

def _qfn_fixture(tmp, h2h):
    """tigard (a real QFN board) with a project declaring min_hole_to_hole."""
    import shutil
    import json as _json
    src = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
    if not os.path.exists(src):
        return None
    dst = os.path.join(tmp, 'h2h.kicad_pcb')
    shutil.copyfile(src, dst)
    with open(os.path.splitext(dst)[0] + '.kicad_pro', 'w',
              encoding='utf-8') as f:
        _json.dump({'net_settings': {'classes': [
            {'name': 'Default', 'clearance': 0.2, 'track_width': 0.25}]},
            'board': {'design_settings': {'rules': {'min_hole_to_hole': h2h}}}},
            f)
    return dst


def _qfn_effective_h2h(board):
    """The hole_to_hole the underpad escape actually routes with.

    Read off what the engine FEEDS run_output_conflict, which is where
    f1dd280 said the value goes -- not re-derived, or the test would only be
    checking arithmetic it wrote itself.
    """
    import io
    import contextlib
    from kicad_parser import parse_kicad_pcb
    import qfn_fanout as Q

    seen = []
    real = Q.run_output_conflict

    def spy(*a, **kw):
        seen.append(kw.get('hole_to_hole'))
        return real(*a, **kw)

    Q.run_output_conflict = spy
    try:
        pcb = parse_kicad_pcb(board)
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            Q.generate_qfn_fanout(
                pcb.footprints["U3"], pcb, net_filter=["/USB_DP", "/USB_DN"],
                layer="F.Cu", track_width=0.1, clearance=0.1, grid_step=0.05,
                escape_method="underpad", via_size=0.45, via_drill=0.25)
    finally:
        Q.run_output_conflict = real
    return (set(v for v in seen if v is not None), buf.getvalue())


def _q4_a_declared_value_must_not_relax_a_fab_floor():
    import tempfile
    from list_nets import board_floor
    from fab_tiers import fab_floor_min

    print('board_floor is board-authoritative, NOT raise-only')
    with tempfile.TemporaryDirectory() as tmp:
        b = _fixture(tmp, extra_rules={'min_hole_to_hole': 0.10})
        # The CLAIM, corrected: this is the documented behaviour, not a bug to
        # fix in the helper -- check_channels needs exactly this freedom.
        check('a declared 0.10 resolves DOWN, and says it came from the board',
              board_floor(b, 'hole_to_hole', None, 0.2)
              == (0.1, 'board constraint'),
              str(board_floor(b, 'hole_to_hole', None, 0.2)))

    fab = fab_floor_min(4).get('hole_to_hole')
    check('the fab hole-to-hole floor is the thing being protected',
          fab == 0.20, f'got {fab}')

    print('...so the qfn DRILL consumer wraps it, and discloses the pin')
    with tempfile.TemporaryDirectory() as tmp:
        b = _qfn_fixture(tmp, 0.10)
        if b is None:
            print('  SKIP  no tigard board (qfn drill floor)')
            return
        got, said = _qfn_effective_h2h(b)
        check('a declared 0.10 does NOT space this run\'s drills at 0.10',
              got == {0.20}, f'engine used {sorted(got)}')
        check('...and it is not relaxed SILENTLY',
              'below the 0.2mm fab hole-to-hole floor' in said, said.strip())

    print('...while a declared value ABOVE the floor still wins')
    with tempfile.TemporaryDirectory() as tmp:
        b = _qfn_fixture(tmp, 0.30)
        got, said = _qfn_effective_h2h(b)
        check('a declared 0.30 raises the drill spacing to 0.30',
              got == {0.30}, f'engine used {sorted(got)}')
        check('...and says the board is where it came from',
              "from the board's own" in said, said.strip())

    print('...and a board declaring nothing is byte-identical')
    with tempfile.TemporaryDirectory() as tmp:
        import shutil
        src = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
        dst = os.path.join(tmp, 'silent.kicad_pcb')
        shutil.copyfile(src, dst)          # no .kicad_pro sibling at all
        got, said = _qfn_effective_h2h(dst)
        check('the packaged default stands, unannounced',
              got == {defaults.HOLE_TO_HOLE_CLEARANCE}
              and 'hole-to-hole' not in said.lower(), f'{sorted(got)}')


class _Boom(dict):
    """A design_rules stand-in that raises when read, to exercise the
    'I could not look' branch without corrupting a real project file."""
    def get(self, *a, **k):
        raise OSError('unreadable')



def main():
    _d9_shared_resolver()
    _q3_routing_half_uses_the_same_resolver()
    _q4_a_declared_value_must_not_relax_a_fab_floor()
    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
