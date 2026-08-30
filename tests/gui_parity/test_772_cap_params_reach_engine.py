#!/usr/bin/env python3
"""#772 on the REAL headless dialog: does an `optimize_caps` plan step's params
reach the ENGINE?

    python3 tests/gui_parity/test_772_cap_params_reach_engine.py

(re-execs into KiCad's bundled python automatically, like its siblings)

THIS FILE EXISTS BECAUSE BOTH EXISTING HALVES WERE GREEN WHILE ALL TEN CAP
KNOBS WERE UNREACHABLE.

  * `tests/gui_parity/test_manifest_plan_parity.py`'s `check_cap_flags` asserts
    CONVERSION: the flag survives into `step['params']`. It did.
  * its `check_param_resolution` asserts the param NAME matches some
    `self.X = ...` somewhere across the four GUI source files. Every `cap_*`
    control has always existed, so that half could never go red.

Neither knows about OWNERS. `ai_plan._owners()` returned `[dialog]` for any
action other than route_diff / fanout / route_planes / repair_planes,
`optimize_caps` was not in that list, and every `cap_*` control lives on
`fanout_tab.bga_options`. So the generic loop logged "no control for
cap_capture_radius, ignored" ten times a step and the engine ran at its
signature defaults. Measured before the fix, from a step carrying the issue's
numbers:

    ENGINE capture_radius = 2.0 (plan 5.0)    near_margin = 1.0 (plan 1.5)
    ENGINE max_passes = 30 (plan 7)           cap_prefix = 'C,R,FB' (plan 'C')
    ENGINE allow_rotations = True (plan False)
    ENGINE board_edge_clearance = None (plan 0.85)
    10 of 11 params did not arrive

And `--board-edge-clearance` DID resolve -- onto the Basic tab's SIGNAL
copper-to-edge control, ticking `edge_clearance_check`, which then stayed
ticked for the NEXT step. Wrong knob, wrong quantity, and a leak.

So this gate asserts DELIVERY, not conversion: it drives the real
`PlanExecutor._next_step()` on a real headless `RoutingDialog` and reads the
kwargs the engine is handed.

HOW THE KWARGS ARE CAPTURED. There is no `KICAD_DUMP_*` env hook for
`repair_fanout_clearance` (`env_knobs.DUMP_BATCH_KWARGS` is read only by
route.py, route_diff.py and bga_fanout), and adding one would be production
code written for a test. The spy therefore sits on the SOURCE module,
`placement.fanout_clearance`, and NOT on `fanout_gui`: `FanoutTab.
_optimize_decoupling_caps` imports the engine INSIDE the method body, so it
re-resolves the source module's attribute on every call and a `fanout_gui`
patch would record nothing at all -- silently, which is the failure mode this
file is here to guard against. It RETURNS an engine-shaped empty result rather
than raising, so the apply loops and the summary run for real, and it asserts
the engine was reached at all: with no such check every kwarg reads back
`<absent>` and the gate passes vacuously.
"""
import inspect
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    r'C:\Program Files\KiCad\10.0\bin\python.exe',
    r'C:\Program Files\KiCad\9.0\bin\python.exe',
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    # Exit 0 like every sibling here, but say plainly that the gate did NOT run.
    # CLAUDE.md: probe before recording it as not-run --
    #   <kicad>/bin/python3 -c "import pcbnew, wx; print(wx.version())"
    print('SKIP: no python with pcbnew+wx found -- THIS GATE DID NOT RUN, and '
          'nothing else measures what the cap engine is actually handed')
    sys.exit(0)


# The issue's numbers, deliberately all different from the panel defaults so a
# dropped param cannot coincide with a delivered one.
PLAN = {
    'cap_capture_radius': 5.0,
    'cap_near_margin': 1.5,
    'cap_step': 0.35,
    'cap_max_displacement': 4.0,
    'cap_max_displacement_cap': 6.0,
    'cap_displacement_growth': 2.0,
    'cap_board_edge_clearance': 0.85,
    'cap_max_passes': 7,
    'cap_prefix': 'C',
    # #742: differs from BOTH the panel default (0.3) and the Basic tab's
    # via_size, so a step that took either instead would show up here.
    'cap_default_via_size': 0.42,
    'cap_allow_rotation': False,
}
# plan param -> the engine kwarg it must arrive as
TO_ENGINE = {
    'cap_capture_radius': 'capture_radius',
    'cap_near_margin': 'near_margin',
    'cap_step': 'step',
    'cap_max_displacement': 'max_displacement',
    'cap_max_displacement_cap': 'max_displacement_cap',
    'cap_displacement_growth': 'displacement_growth',
    'cap_board_edge_clearance': 'board_edge_clearance',
    'cap_max_passes': 'max_passes',
    'cap_prefix': 'cap_prefix',
    'cap_default_via_size': 'default_via_size',
    'cap_allow_rotation': 'allow_rotations',
}
ABSENT = '<<absent>>'
# the shape the engine's own early returns carry, plus the four nudge keys, so
# the apply loops and cap_optimization_summary run instead of being skipped
EMPTY = dict(placements=[], resolved=[], unresolved=[], bga_refs=[],
             required=[], clearance_notes=[], via_moves=[], new_segments=[],
             via_resolved=[], regrazed=[])


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    import pcbnew
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
    sys.path.insert(0, os.path.dirname(REPO))

    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin.fanout_gui import BGAOptionsPanel
    from kicad_routing_plugin import ai_plan
    from placement import fanout_clearance as _fc
    from placement.fanout_clearance import (CAP_EDGE_CLEARANCE,
                                            resolve_cap_edge_clearance)

    real_engine = _fc.repair_fanout_clearance      # captured BEFORE patching
    app = wx.App(False)  # noqa: F841
    board = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    failures = []

    def check(name, ok, detail=''):
        if not ok:
            failures.append(name)
        print(('  PASS ' if ok else '  FAIL ') + name
              + (('  ' + detail) if detail and not ok else ''))

    def close(a, b):
        return isinstance(a, float) and abs(a - b) < 1e-9

    # pcbnew.GetBoard() is None outside the KiCad process and
    # run_cap_optimization returns early on None -- the engine would never be
    # reached, every kwarg would read back absent, and that is a false pass
    # rather than a check.
    live = pcbnew.GetBoard() or pcbnew.LoadBoard(board)
    if live is None:
        print('SKIP: pcbnew could not load the fixture board')
        return 0
    pcbnew.GetBoard = lambda: live

    seen = {}

    def _spy(pcb_data, **kw):
        seen.clear()
        seen.update(kw)
        return dict(EMPTY)

    def drive(steps):
        """Run `steps` through the REAL PlanExecutor; one kwarg dict per step.

        Not apply_step_params alone: _next_step also runs the #768 clearance
        rule, the #772 scoped cap reset and the _action_parts dispatch, and this
        file is about what the ENGINE gets after all three.
        """
        out = []
        _fc.repair_fanout_clearance = _spy
        try:
            ex = ai_plan.PlanExecutor(
                dlg, steps, list(range(len(steps))),
                lambda i, s: None, lambda c, a: None, log=lambda m: None)
            ex._queue = list(range(len(steps)))
            for _ in steps:
                seen.clear()
                ex._next_step()
                if not seen:
                    raise AssertionError(
                        'the engine was not reached -- the step returned '
                        'early, so no kwarg below would mean anything')
                out.append(dict(seen))
        finally:
            _fc.repair_fanout_clearance = real_engine
        return out

    # == 1. DELIVERY: every cap knob a plan step names reaches the engine =====
    dlg.reset_params_to_defaults()
    kw = drive([{'action': 'optimize_caps', 'params': dict(PLAN)}])[0]
    for p in sorted(PLAN):
        e, want = TO_ENGINE[p], PLAN[p]
        got = kw.get(e, ABSENT)
        ok = close(got, want) if isinstance(want, float) else got == want
        check('plan %s=%r -> engine %s' % (p, want, e), ok, 'got %r' % (got,))

    # == 2. and NOT through the Basic tab's SIGNAL edge control ==============
    # The #772 leak: the cap margin used to tick edge_clearance_check, which
    # then stayed ticked and re-priced the NEXT step's routing keep-out.
    check('the Basic tab edge override is still UNTICKED after a cap step',
          dlg.edge_clearance_check.GetValue() is False,
          'edge_clearance_check=%r board_edge_clearance=%r -- the cap margin '
          'landed on the SIGNAL keep-out and leaks into the next step'
          % (dlg.edge_clearance_check.GetValue(),
             dlg.board_edge_clearance.GetValue()))

    # == 3. LEGACY spelling (plans converted before #772) ====================
    dlg.reset_params_to_defaults()
    kw = drive([{'action': 'optimize_caps',
                 'params': {'board_edge_clearance': 0.85}}])[0]
    check('a legacy `board_edge_clearance` reaches the CAP knob',
          close(kw.get('board_edge_clearance'), 0.85),
          'got %r' % (kw.get('board_edge_clearance', ABSENT),))
    check('...and still does not tick the signal override',
          dlg.edge_clearance_check.GetValue() is False)

    # == 4. ZERO is UNSET, on both fronts ====================================
    # get_config maps a 0 spin to None; resolve_cap_edge_clearance applies the
    # SAME non-positive-is-unset rule to an explicit CLI value. So a plan
    # carrying 0 must land on the resolved default, not a margin of zero.
    dlg.reset_params_to_defaults()
    kw = drive([{'action': 'optimize_caps',
                 'params': {'cap_board_edge_clearance': 0.0}}])[0]
    check('an explicit 0 arrives as None (UNSET), not 0.0',
          kw.get('board_edge_clearance', ABSENT) is None,
          'got %r' % (kw.get('board_edge_clearance', ABSENT),))
    _resolved = resolve_cap_edge_clearance(board,
                                           kw.get('board_edge_clearance'))
    # the CONSTANT, not a copy of its current value: a gate that hard-codes
    # 0.55 goes stale the day the placement default moves
    check('...and the engine then resolves the placement default',
          close(_resolved[0], CAP_EDGE_CLEARANCE),
          'got %r want %r' % (_resolved, CAP_EDGE_CLEARANCE))

    # == 5. NO LEAK between two consecutive cap steps ========================
    # The per-step reset is SKIPPED for optimize_caps by design, so without the
    # scoped cap reset step B inherits step A's knobs -- and a recorded CLI
    # `--capture-radius 5` gives every OTHER flag its argparse default, not the
    # previous command's value.
    dlg.reset_params_to_defaults()
    kws = drive([
        {'action': 'optimize_caps',
         'params': {'cap_near_margin': 1.5, 'cap_prefix': 'C',
                    'cap_max_passes': 7}},
        {'action': 'optimize_caps', 'params': {'cap_capture_radius': 5.0}},
        # STEP C NAMES NO CAP KNOB AT ALL, only Basic-tab params -- exactly
        # what `place_fanout_clearance.py --clearance 0.1 --grid-step 0.05`
        # converts to. An adversarial review measured that the first version
        # of the fix skipped its reset (the condition was "names a cap
        # knob", not "the plan specified this step") and it inherited step
        # A's knobs. The --grid-step row this branch adds makes the shape
        # more reachable, and arm 5 originally could not see it because
        # every step it drove named one.
        {'action': 'optimize_caps',
         'params': {'clearance': 0.1, 'grid_step': 0.05}},
    ])
    check('step A: its own knobs are delivered',
          close(kws[0].get('near_margin'), 1.5)
          and kws[0].get('cap_prefix') == 'C')
    check('step B: the knob it NAMES is delivered',
          close(kws[1].get('capture_radius'), 5.0),
          'got %r' % (kws[1].get('capture_radius', ABSENT),))
    check('step B: a knob it does NOT name is the CLI default, not step A\'s',
          close(kws[1].get('near_margin'), 1.0),
          'near_margin=%r -- step A\'s 1.5 leaked into step B'
          % (kws[1].get('near_margin', ABSENT),))
    check('step B: and the same for cap_prefix',
          kws[1].get('cap_prefix') == 'C,R,FB',
          'cap_prefix=%r -- step A\'s value leaked'
          % (kws[1].get('cap_prefix'),))
    # The knob to watch is CAPTURE_RADIUS: step B named it, so step B's own
    # reset left it at 5.0 while returning everything else to the defaults.
    # It is therefore the only non-default value standing when step C runs,
    # and the only one that can demonstrate the leak. Checking near_margin
    # here would pass either way -- step B already restored it -- which is
    # how the first version of this arm managed to be green against the
    # very defect it was added for.
    check('step C names NO cap knob and still starts from the defaults',
          close(kws[2].get('capture_radius'), 2.0),
          'capture_radius=%r -- a step whose params are all Basic-tab '
          'ones inherited the earlier cap step (want the CLI default 2.0)'
          % (kws[2].get('capture_radius', ABSENT),))
    check('...while its OWN Basic-tab params still arrive',
          close(kws[2].get('clearance'), 0.1)
          and close(kws[2].get('grid_step'), 0.05),
          'clearance=%r grid_step=%r' % (kws[2].get('clearance', ABSENT),
                                         kws[2].get('grid_step', ABSENT)))

    # == 6. a step with NO PARAMS is left alone ==============================
    # The auto-inserted step (_insert_cap_optimization) carries no `params`
    # key at all, and the scoped reset must not fire for it.
    #
    # WHAT IT THEN INHERITS IS NARROWER THAN THIS ARM ORIGINALLY CLAIMED, and
    # the correction came from an adversarial review. It used to set the
    # panel by hand and drive a ONE-step plan, then say the operator's tweaks
    # "must survive". In a real plan a `fanout` step precedes the cap step,
    # and that step gets the FULL per-step reset -- which, since the cap
    # knobs joined reset_params_to_defaults, returns them to the CLI
    # defaults. So the honest claim is: the scoped reset does not fire, and
    # the step sees whatever the panel holds AT THAT MOMENT. Both halves are
    # asserted below, the second one through a preceding step rather than a
    # scenario no plan produces.
    dlg.reset_params_to_defaults()
    dlg.fanout_tab.bga_options.cap_near_margin.SetValue(1.5)
    dlg.fanout_tab.bga_options.cap_prefix.SetValue('C')
    kw = drive([{'action': 'optimize_caps'}])[0]
    check('a bare optimize_caps step does not trigger the scoped reset',
          close(kw.get('near_margin'), 1.5)
          and kw.get('cap_prefix') == 'C',
          'near_margin=%r cap_prefix=%r -- the reset fired on a step that '
          'specified nothing'
          % (kw.get('near_margin', ABSENT), kw.get('cap_prefix', ABSENT)))

    # == 7. the SHARED Basic-tab knobs keep inheriting =======================
    # The scoped reset must touch the cap panel ONLY: clearance / grid_step /
    # via_size come from the Basic tab and the #768 rationale is about those.
    dlg.reset_params_to_defaults()
    dlg.grid_step.SetValue(0.07)
    kw = drive([{'action': 'optimize_caps',
                 'params': {'cap_near_margin': 1.5}}])[0]
    check('a scoped cap reset leaves the Basic tab grid step alone',
          close(kw.get('grid_step'), 0.07),
          'grid_step=%r -- the reset was not scoped to the cap panel'
          % (kw.get('grid_step', ABSENT),))

    # == 8. #768 still holds: clearance arrives, and so does the ceiling =====
    dlg.reset_params_to_defaults()
    kw = drive([{'action': 'optimize_caps', 'params': {'clearance': 0.1}}])[0]
    check('a cap step\'s clearance reaches the engine',
          close(kw.get('clearance'), 0.1),
          'got %r' % (kw.get('clearance', ABSENT),))
    check('...and PRESENCE switches the netclass ceiling on (#768)',
          close(kw.get('netclass_ceiling'), 0.1),
          'netclass_ceiling=%r' % (kw.get('netclass_ceiling', ABSENT),))

    # == 9. reset_params_to_defaults covers all TEN cap knobs ================
    # CLAUDE.md: "add it to reset_params_to_defaults ... or the param leaks
    # between steps". Before #772 only three of the ten were in it.
    opts = dlg.fanout_tab.bga_options
    for name, _default in BGAOptionsPanel.CAP_PARAM_DEFAULTS:
        ctl = getattr(opts, name, None)
        if ctl is None:
            continue
        cur = ctl.GetValue()
        ctl.SetValue((not cur) if isinstance(cur, bool)
                     else ('ZZZ' if isinstance(cur, str) else cur + 1))
    dlg.reset_params_to_defaults()
    for name, default in BGAOptionsPanel.CAP_PARAM_DEFAULTS:
        got = getattr(opts, name).GetValue()
        ok = close(got, default) if isinstance(default, float) \
            else got == default
        check('reset_params_to_defaults restores %s' % name, ok,
              'got %r want %r' % (got, default))

    # == 10. the table is TRUE of a freshly constructed panel ================
    # CAP_PARAM_DEFAULTS is hand-written next to the _cap_spin calls; this is
    # what stops it drifting from them.
    frame = wx.Frame(None)
    fresh = BGAOptionsPanel(frame)
    for name, default in BGAOptionsPanel.CAP_PARAM_DEFAULTS:
        ctl = getattr(fresh, name, None)
        check('CAP_PARAM_DEFAULTS names a real control: %s' % name,
              ctl is not None)
        if ctl is None:
            continue
        got = ctl.GetValue()
        ok = close(got, default) if isinstance(default, float) \
            else got == default
        check('...and its creation value matches the table: %s' % name, ok,
              'control=%r table=%r' % (got, default))
    frame.Destroy()

    # == 11. and the table equals the CLI/ENGINE defaults ====================
    # place_fanout_clearance.py passes its argparse values straight through, so
    # repair_fanout_clearance's SIGNATURE is the CLI default set. A GUI default
    # that drifts from it means "reset the cap panel, then apply the step's
    # params" stops being "run the CLI with exactly these flags" -- the rule the
    # scoped reset is justified by.
    sig = inspect.signature(real_engine).parameters
    table = dict(BGAOptionsPanel.CAP_PARAM_DEFAULTS)
    for gui_name, engine_name in sorted(TO_ENGINE.items()):
        eng = sig[engine_name].default
        if gui_name == 'cap_board_edge_clearance':
            check('engine default for %s is None (== the panel 0 = UNSET)'
                  % engine_name, eng is None, 'got %r' % (eng,))
            continue
        want = table[gui_name]
        ok = close(eng, want) if isinstance(want, float) else eng == want
        check('panel default %s == engine default %s'
              % (gui_name, engine_name), ok,
              'panel=%r engine=%r' % (want, eng))

    print()
    if failures:
        print('FAILED (%d): %s' % (len(failures), ', '.join(failures)))
        return 1
    print('ALL PASS -- every cap param a plan step names reaches the engine, '
          'on the cap knob, without leaking into the next step')
    return 0


if __name__ == '__main__':
    sys.exit(main())
