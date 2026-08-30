"""A geometry-floor override must not leak from one plan step into the next.

`_effective_geometry_floor` returns the board's Default net-class value when a
floor's override checkbox is UNCHECKED, and the typed control value when it is
CHECKED. That mirrors the CLI exactly: an omitted `--track-width` /
`--clearance` / `--via-*` makes the CLI resolve the board's own value.

`ai_plan._enable_geometry_override` checks the box whenever a plan step supplies
the param — correct — and `reset_params_to_defaults` un-checks every one of them
again (#439, the loop near the end of that method). So the design was right and
the code was right.

What broke it was 80 lines above that loop: `impedance_value.SetValue(50.0)`
passed a float to a wx.SpinCtrl (integer-only), which raises `TypeError` on
wxPython 4.2 — and `ai_plan` CATCHES it, logging "per-step reset skipped". The
reset therefore ABANDONED itself partway, and every control below that line,
including the whole #439 un-check loop, silently kept the previous step's state.

The visible symptom was `test_gui_engine_parity` reporting CLI=1307 / GUI=1308
segments, with 58 GND plane taps at 0.3 mm where the CLI laid them at 0.127 —
same coordinates, 2.4x the copper width — while still printing "VERDICT: PARITY",
because the graded metrics (DRC, unconnected) did not move.

The lesson this gate encodes: a caught-and-logged exception in a reset is
indistinguishable from a reset that worked, so assert the OBSERVABLE state
(what each floor resolves to) rather than trusting the reset ran.

Needs wx; re-execs into KiCad's python.

    python3 tests/gui_parity/test_geometry_floor_leak.py [board.kicad_pcb]
"""

import os
import subprocess
import sys

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.dirname(REPO))
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522 layout
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

# Shared with the engine (#647): versioned Windows layouts, sorted by NUMERIC
# version (the local `sorted(..., reverse=True)` this replaced put "9.0" above
# "10.0"). The loop below still verifies wx+pcbnew itself.
from kicad_exact_fill import kicad_python_candidates  # noqa: E402

KICAD_PYTHONS = kicad_python_candidates()

FLOORS = ('track_width', 'clearance', 'via_size', 'via_drill',
          'hole_to_hole_clearance')


def _reexec():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
        return
    except ImportError:
        pass
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import wx, pcbnew'],
                          capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with wx+pcbnew found")
    sys.exit(2)


def main():
    _reexec()
    import pcbnew
    import wx

    from kicad_parser import build_pcb_data_from_board
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    from kicad_routing_plugin import ai_plan, routing_dialog as swig_gui

    board_path = (sys.argv[1] if len(sys.argv) > 1
                  else os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb'))
    # Bind it: an unreferenced wx.App is collected before the dialog is built,
    # and RoutingDialog then dies with "The wx.App object must be created first!"
    _app = wx.App(False)
    assert _app is not None
    board = pcbnew.LoadBoard(board_path)
    pcbnew.GetBoard = lambda: board
    dlg = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), board_path)

    bad = []
    # What a step that omits every floor must resolve: the board's own values.
    board_vals = {n: dlg._effective_geometry_floor(n) for n in FLOORS}
    for n, v in board_vals.items():
        if getattr(dlg, n + '_check').GetValue():
            bad.append((n, "override box is CHECKED on a fresh dialog"))

    # A step that SUPPLIES them (parity plan step 1) checks the boxes.
    ai_plan.apply_step_params(
        {'action': 'route',
         'params': {'track_width': 0.127, 'clearance': 0.15,
                    'via_size': 0.45, 'via_drill': 0.2}}, dlg)
    if not dlg.track_width_check.GetValue():
        bad.append(('track_width', "a supplied param did NOT check its box; "
                                   "the typed value would be ignored"))

    # The executor resets before the next step. This must not raise: ai_plan
    # catches it and logs "per-step reset skipped", leaving the reset half-done.
    try:
        dlg.reset_params_to_defaults()
    except Exception as e:
        bad.append(('<reset>', f"reset_params_to_defaults raised {type(e).__name__}: {e} "
                               f"-- ai_plan catches this, so every control after "
                               f"that line keeps the previous step's value"))

    # A step that OMITS them (parity plan step 2, route_planes) must be back to
    # the board's values -- what omitting the CLI flag means.
    for n in FLOORS:
        got = dlg._effective_geometry_floor(n)
        if abs(got - board_vals[n]) > 1e-9:
            bad.append((n, f"leaked: board {board_vals[n]} but resolves {got} "
                           f"after a reset (CLI would use {board_vals[n]})"))

    bad += _declared_zero_is_unset(dlg)

    print(f"board: {os.path.basename(board_path)}")
    for n in FLOORS:
        print(f"  {n:26s} board={board_vals[n]:<8} "
              f"after-reset={dlg._effective_geometry_floor(n)}")
    print(f"\nGeometry-floor leak: {len(bad)} problem(s).")
    for n, why in bad:
        print(f"    {n}: {why}")
    return 1 if bad else 0


def _declared_zero_is_unset(dlg):
    """A board declaring 0 must be UNSET on this side too, as on the CLI.

    `_effective_geometry_floor`'s unchecked branch took the board value on
    `is not None` alone. KiCad writes 0 into these fields for "not
    configured", and every other resolver in the tree treats that as unset --
    list_nets.board_floor, board_floor_knobs, resolve_cli_floor, and
    `_effective_plane_edge_clearance` ten lines above it, which already
    guarded `> 1e-9`. This branch did not.

    It is MASKED in the default fab tier: `_fab_floored` pins a 0.0 up to the
    0.20 fab hole-to-hole floor, and the CLI lands on 0.20 as well, so the two
    agree by accident. Move the fab floor and they stop agreeing. With a
    fab-overrides declaring hole_to_hole 0.10 -- reachable from the GUI via the
    fab_tier / fab_overrides_path controls, and collapsing fab_floor_ladder to
    that single hard rung -- a board declaring 0.0 gave GUI 0.10 against CLI
    0.20: the GUI drilling twice as close as the CLI on the same board.

    Driven through the REAL dialog rather than a mirrored config, per the
    CLAUDE.md note that hand-mirrored parity maps silently drift.
    """
    import tempfile
    from kicad_routing_plugin import routing_dialog as swig_gui
    out = []
    # Force the "board declares 0.0" case without needing such a board.
    real = swig_gui._get_board_minimum_constraints
    swig_gui._get_board_minimum_constraints = lambda *a, **k: {
        'min_hole_to_hole': 0.0}
    saved_path = dlg.fab_overrides_path.GetValue()
    tmp = tempfile.mkdtemp()
    ovr = os.path.join(tmp, 'fine.fab')
    with open(ovr, 'w', encoding='utf-8') as f:
        f.write("# a fab that really can drill closer\n"
                "hole_to_hole = 0.10\n")
    try:
        # 1. Default tier: both sides land on the 0.20 fab floor, which is why
        #    this divergence is invisible until the fab floor moves.
        got_default = dlg._effective_geometry_floor('hole_to_hole_clearance')
        if abs(got_default - 0.20) > 1e-9:
            out.append(('hole_to_hole_clearance',
                        f"declared 0.0 at the default tier resolves "
                        f"{got_default}, expected the 0.20 fab floor"))
        # 2. The unmasking case, driven through the REAL control a user sets:
        #    an override FILE lowering the fab hole-to-hole floor to 0.10.
        #    _fab_floor_for_ctrl reads fab_overrides_path and parses it, so
        #    set_default_fab_tier alone does NOT reach this path.
        dlg.fab_overrides_path.SetValue(ovr)
        floor = dlg._fab_floor_for_ctrl('hole_to_hole_clearance')
        if floor is None or abs(floor - 0.10) > 1e-9:
            out.append(('<fixture>',
                        f"the override file did not lower the fab floor "
                        f"(got {floor}); this check would prove nothing"))
        got = dlg._effective_geometry_floor('hole_to_hole_clearance')
        # The CLI resolves 0.2 here: board_floor calls a declared 0.0 unset and
        # falls back to HOLE_TO_HOLE_CLEARANCE 0.2, which enforce_fab_floors
        # only ever raises.
        if abs(got - 0.20) > 1e-9:
            out.append(('hole_to_hole_clearance',
                        f"declared 0.0 under a 0.10 fab override resolves "
                        f"{got}; the CLI resolves 0.2, so the GUI would drill "
                        f"{0.2 / got:.1f}x closer on the same board"))
        print(f"  declared-0.0 hole_to_hole, fab floor {floor} -> GUI {got} "
              f"(CLI 0.2)")
    finally:
        swig_gui._get_board_minimum_constraints = real
        dlg.fab_overrides_path.SetValue(saved_path)
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)
    return out


if __name__ == '__main__':
    sys.exit(main())
