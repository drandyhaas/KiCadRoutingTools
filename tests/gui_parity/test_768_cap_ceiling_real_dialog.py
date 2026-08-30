#!/usr/bin/env python3
"""#768 on the REAL headless dialog: what value of `netclass_ceiling` does the
cap pass actually receive, on BOTH GUI call paths?

    python3 tests/gui_parity/test_768_cap_ceiling_real_dialog.py

(re-execs into KiCad's bundled python automatically, like its siblings)

THIS FILE EXISTS BECAUSE THE SOURCE-TEXT HALF COULD NOT CATCH THE DEFECT IT WAS
NAMED FOR. `tests/test_768_cap_clearance_ceiling.py` asserted that the gate
expression mentions a config key and that the key appears once per file. Both
were true of a gate that was WRONG (it read `fix_drc_settings`) and, on one of
the two call paths, INERT (that key is not in the config that path builds). Only
capturing the kwargs the engine is handed, from the real dialog, distinguishes
those.

THE RULE. The CLI switches the ceiling on the PRESENCE of `--clearance`, and
the GUI now carries a value with exactly that contract: `clearance_ceiling` is
the Basic tab's Min Clearance override spin value when its box is ticked, and
None when it is not. Presence IS the switch, on both fronts.

IT MUST BE THE RAW OVERRIDE, not `_effective_clearance()`. That helper already
returns `min(Default class, override)`, which is right for the BASE and wrong
for the ceiling: handed it, a class sitting BETWEEN the Default and the
operator's number gets capped to the Default instead of to the number typed.
An adversarial review measured 22 cap clearance violations against main's 2 at
the default dialog configuration when the resolved base was passed as a ceiling.

WHY NOT `fix_drc_settings`, which the first cut of #768 used: measured, that box
did not clamp a net class at all. `update_live_drc_floors` wrote `m_MinClearance`
and the DEFAULT class only, carried no non-Default clamp, and this tab never
called `apply_targets_to_board`. Gated there, the GUI priced every pair at the
ceiling and clamped nothing -- the GIVEN branch's pricing with the OMITTED
branch's writeback, which is #768 pointing the other way.

PAST TENSE SINCE #782, and the reasoning above is preserved rather than deleted
because it is still why `clearance_ceiling` is the right gate and
`fix_drc_settings` is not. What changed is only the second clause: the tab now
DOES clamp its non-Default classes, through
`update_live_drc_floors(nondefault_clamp_mm=...)` on the inline path and
`clamp_nondefault_netclasses_on_board` on the standalone one, both switched on
the presence of the same `clearance_ceiling` this file grades. That half is
`tests/gui_parity/test_782_fanout_netclass_clamp.py`; this file still owns the
PRICING half, which is the question "what value does the engine receive".

TWO PATHS, and the first cut was correct on one and inert on the other:
  inline      `_apply_fanout_results` -> `_optimize_decoupling_caps(fanout_config)`
  standalone  `run_cap_optimization` -> builds its OWN cfg from a handful of
              shared keys, then calls the same method. The plan executor's
              `optimize_caps` step uses this one.
Both are driven below. A gate that reads a key the standalone cfg does not carry
looks right inline and does nothing where it matters.
"""
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
    print("SKIP: no python with pcbnew+wx found")
    sys.exit(0)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.dirname(REPO))

    app = wx.App(False)  # noqa: F841
    # flat_hierarchy is the repo's only tracked board declaring a NON-Default
    # class (Default 0.2, Wide 0.4). A board with one class cannot tell a
    # working ceiling from a broken one, because there is nothing to cap.
    board = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog

    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    tab = dlg.fanout_tab
    failures = []

    def check(name, cond, detail="", info=""):
        # `detail` is the FAILURE explanation, so print it only on
        # failure -- a PASS line carrying "so the cap pass runs the
        # OMITTED branch" reads as a contradiction. The same note is on
        # test_733_cap_edge_clearance_gui's check, for the same reason.
        #
        # `info` is the MEASUREMENT, and prints either way. Suppressing
        # both was an over-correction: two checks here carried the value
        # they had just read (`keys=..`, `clamp_netclasses=..`) as their
        # detail, and a run that prints strictly less about what it
        # measured is a worse gate, not a tidier one.
        if not cond:
            failures.append(name)
        print(("  PASS " if cond else "  FAIL ") + name
              + (f"  {info}" if info else "")
              + (f"  {detail}" if detail and not cond else ""))

    # -- capture the kwargs, run nothing ------------------------------------
    seen = {}

    def _spy(pcb_data, **kw):
        seen.clear()
        seen.update(kw)
        raise _Stop()

    class _Stop(Exception):
        pass

    # The tab imports the engine INSIDE the method
    # (`from placement.fanout_clearance import repair_fanout_clearance`), so the
    # spy has to sit on the source module, not on `fanout_gui`. Patching the
    # wrong one raises AttributeError rather than silently recording nothing --
    # which is the failure mode this file is guarding against elsewhere.
    from placement import fanout_clearance as _fc
    real = _fc.repair_fanout_clearance

    import pcbnew as _pcbnew
    # `pcbnew.GetBoard()` is None outside the KiCad process, and
    # `_optimize_decoupling_caps` returns early on a None board -- so the engine
    # is never reached and every kwarg reads back as absent. Load the fixture
    # and hand the tab THAT board. The first run of this file "passed" four
    # checks against an empty dict for exactly this reason, which is the
    # missing-input false pass `run_utils.evidence` exists to refuse.
    live = _pcbnew.GetBoard() or _pcbnew.LoadBoard(board)
    if live is None:
        print("SKIP: pcbnew could not load the fixture board")
        return 0

    ABSENT = '<<absent>>'

    def _near(got, want):
        # A SpinCtrlDouble read-back is a float that made a round trip
        # through the control, and #493 was a one-ULP netclass clearance in
        # exactly this repo. Section 2b already compared with a tolerance;
        # the other sections used `==` on the same kind of value, which is
        # a latent flake rather than a stricter check.
        return isinstance(got, float) and abs(got - want) < 1e-9

    def _inline_cfg():
        """The fanout_config `_run_bga_fanout` ACTUALLY builds (#780).

        The arms below used to assemble one by hand from
        `get_shared_params()` and inject `clearance_ceiling` into it. That
        proved `_optimize_decoupling_caps` READS the key -- which it does --
        and never that the inline path SUPPLIES it, which it did not: the
        dict `_run_bga_fanout` constructs had no such key, so the inline cap
        pass ran #768's OMITTED branch whatever the operator ticked. The
        section was labelled `the INLINE path` throughout.

        So: drive the real method, with the worker thread and the poll
        stubbed, and read the dict it hands on. Nothing is routed.
        """
        import threading
        captured = {}
        why = []
        real_thread = threading.Thread

        class _NoThread:
            def __init__(self, *a, **k):
                pass

            def start(self):
                pass

        tab._poll_operation = lambda apply_kw, kind: captured.update(apply_kw)
        threading.Thread = _NoThread
        try:
            cfg = tab.bga_options.get_config()
            cfg['optimize_caps'] = True
            # A real Footprint, not its reference. `PCBData.footprints` is
            # a dict KEYED by reference, so sorted(...)[0] is the string
            # 'C1'. It happens to work because nothing on this path reads
            # the object before the stubbed thread -- which is exactly the
            # kind of accident that turns into a misleading
            # 'never reached _poll_operation' the day a line does.
            _pcb = parse_kicad_pcb(board)
            fp = _pcb.footprints[sorted(_pcb.footprints)[0]]
            try:
                tab._run_bga_fanout(fp, ['*'], cfg)
            except Exception as e:                         # noqa: BLE001
                # the engine half is stubbed; we want the dict only -- but
                # KEEP the reason, or a real break reads as 'the method
                # never got there' with no trace of what actually raised.
                why.append('%s: %s' % (type(e).__name__, str(e)[:160]))
        finally:
            threading.Thread = real_thread
            # delete the instance shadow rather than writing the bound
            # method back onto the instance (that leaves a self-reference)
            tab.__dict__.pop('_poll_operation', None)
            # _begin_run ran and _end_run did not, so the tab is left
            # _running with its button disabled -- ai_plan._poll_until_idle
            # reads exactly that as 'busy'. Hand it back the way we found
            # it, or the next arm to use a real entry point hangs.
            try:
                tab._end_run()
            except Exception:                              # noqa: BLE001
                tab._running = False
        if 'fanout_config' not in captured:
            raise AssertionError(
                '_run_bga_fanout never reached _poll_operation, so there is '
                'no config to read and every check below would be vacuous'
                + (' (raised %s)' % why[0] if why else ''))
        return captured['fanout_config']

    def _standalone_kw():
        """The engine kwargs `run_cap_optimization` ACTUALLY produces (#780).

        Section 3 used to build its own cfg from `get_shared_params()` and
        hand it to `_drive`. That is the same flaw #780 found in the INLINE
        section, still standing in the STANDALONE one: it proved the
        CONSUMER reads the key while its label named the PRODUCER. Measured
        -- with `run_cap_optimization`'s two forwarding lines stripped at
        runtime, the old section 3 passed 18 of 18.

        So drive the method itself. `pcbnew.GetBoard()` is None outside the
        KiCad process and the method returns early on that, so point it at
        the loaded fixture for the call.
        """
        seen.clear()
        _fc.repair_fanout_clearance = _spy
        real_getboard = _pcbnew.GetBoard
        _pcbnew.GetBoard = lambda: live
        try:
            try:
                tab.run_cap_optimization()
            except _Stop:
                pass
            except Exception as e:                         # noqa: BLE001
                if not seen:
                    raise AssertionError(
                        'run_cap_optimization did not reach the engine '
                        '(%s: %s) -- every kwarg below would read as absent'
                        % (type(e).__name__, str(e)[:160]))
            if not seen:
                raise AssertionError(
                    'run_cap_optimization returned without reaching the '
                    'engine and nothing raised, so no kwarg below means '
                    'anything')
            return dict(seen)
        finally:
            _pcbnew.GetBoard = real_getboard
            _fc.repair_fanout_clearance = real

    def _drive(cfg):
        """Call the tab's cap step with `cfg` and return the engine kwargs.

        Raises if the engine was not reached: a check whose input is missing
        tests nothing, and `.get(k)` returning None for an ABSENT key is
        indistinguishable from the value this file is asserting."""
        seen.clear()
        _fc.repair_fanout_clearance = _spy
        try:
            try:
                tab._optimize_decoupling_caps(live, _pcbnew, cfg)
            except _Stop:
                pass
            except Exception as e:                            # noqa: BLE001
                if not seen:
                    raise AssertionError(
                        "the engine was not reached (%s: %s) -- this check "
                        "would have read every kwarg as absent"
                        % (type(e).__name__, str(e)[:120]))
            if not seen:
                raise AssertionError(
                    "the engine was not reached and nothing raised: the tab "
                    "returned early, so no kwarg below means anything")
            return dict(seen)
        finally:
            _fc.repair_fanout_clearance = real

    # -- 1. the shared params carry the override at all ---------------------
    shared = tab.get_shared_params() if tab.get_shared_params else {}
    check("the fanout tab's shared params carry clamp_netclasses",
          'clamp_netclasses' in shared,
          info="keys=%d" % len(shared))

    # -- 2. the INLINE path, both positions of the override -----------------
    # #780: the config comes from `_run_bga_fanout` itself now, not from a
    # dict assembled here. Injecting the key was what let this section read
    # green while the inline path never carried it.
    for ticked in (False, True):
        dlg.clearance_check.SetValue(ticked)
        dlg.clearance.SetValue(0.2)
        cfg = _inline_cfg()
        want = 0.2 if ticked else None
        got_cfg = cfg.get('clearance_ceiling', ABSENT)
        check("inline: _run_bga_fanout CARRIES the ceiling, override %s"
              % ('CHECKED' if ticked else 'unchecked'),
              got_cfg is None if want is None else _near(got_cfg, want),
              "the dict the inline path builds has clearance_ceiling=%r, "
              "so the cap pass runs #768's OMITTED branch" % (got_cfg,),
              info="got %r" % (got_cfg,))
        kw = _drive(cfg)
        got = kw.get('netclass_ceiling', ABSENT)
        check("inline: override %s -> ceiling %r"
              % ('CHECKED' if ticked else 'unchecked', want),
              got is None if want is None else _near(got, want),
              "got %r" % (got,))

    # -- 2b. THE RAW OVERRIDE, not the resolved base ------------------------
    # flat_hierarchy declares Default 0.2. An override of 0.3 must arrive as
    # 0.3: `_effective_clearance()` would hand over min(0.2, 0.3) = 0.2, which
    # caps a class between the two to the Default instead of to what was typed.
    dlg.clearance_check.SetValue(True)
    dlg.clearance.SetValue(0.3)
    shared_raw = tab.get_shared_params()
    check("shared params export the RAW override, not min(Default, override)",
          abs((shared_raw.get('clearance_ceiling') or 0) - 0.3) < 1e-9,
          "clearance_ceiling=%r effective clearance=%r"
          % (shared_raw.get('clearance_ceiling'), shared_raw.get('clearance')))
    kw = _drive(dict(shared_raw))
    got = kw.get('netclass_ceiling', ABSENT)
    check("and the engine receives the raw override",
          abs((got if isinstance(got, float) else -1) - 0.3) < 1e-9,
          "got %r" % (got,))
    dlg.clearance_check.SetValue(False)

    # -- 3. the STANDALONE path, which builds its own cfg -------------------
    # This is the one the plan executor uses, and the one an earlier cut of
    # #768 left inert by gating on a key this cfg never carried.
    #
    # #780: driven through `run_cap_optimization` itself now, for the reason
    # section 2 is. Assembling the cfg here tested the consumer under a
    # label naming the producer, and passed with the producer's forwarding
    # deleted.
    for ticked in (False, True):
        dlg.clearance_check.SetValue(ticked)
        dlg.clearance.SetValue(0.2)
        shared2 = tab.get_shared_params()
        check("standalone: shared params report override %s"
              % ('CHECKED' if ticked else 'unchecked'),
              bool(shared2.get('clamp_netclasses')) == ticked,
              info="clamp_netclasses=%r" % (shared2.get('clamp_netclasses'),))
        kw = _standalone_kw()
        got = kw.get('netclass_ceiling', ABSENT)
        want = 0.2 if ticked else None
        check("standalone: run_cap_optimization CARRIES the ceiling, "
              "override %s" % ('CHECKED' if ticked else 'unchecked'),
              got is None if want is None else _near(got, want),
              "the cfg run_cap_optimization builds reached the engine as "
              "netclass_ceiling=%r, so the cap pass runs the wrong "
              "#768 branch" % (got,), info="got %r" % (got,))

    # -- 4. the gate is NOT fix_drc_settings --------------------------------
    # The change detector for the defect this file was written after: with the
    # override UNCHECKED, no value of the Fix-DRC box may produce a ceiling.
    for fix in (False, True):
        cfg = dict(shared)
        cfg['clearance_ceiling'] = None
        cfg['fix_drc_settings'] = fix
        cfg['clearance'] = 0.2
        kw = _drive(cfg)
        got = kw.get('netclass_ceiling', ABSENT)
        check("fix_drc_settings=%r cannot conjure a ceiling on its own" % fix,
              got is None, "got %r" % (got,))

    # -- 5. an ABSENT key means "honour the board" --------------------------
    cfg = dict(shared)
    cfg.pop('clearance_ceiling', None)
    cfg['clearance'] = 0.2
    kw = _drive(cfg)
    got = kw.get('netclass_ceiling', ABSENT)
    check("an absent clearance_ceiling defaults to NO ceiling",
          got is None, "got %r" % (got,))

    # -- 6. and the flat floor is unaffected by the switch ------------------
    # The operator's number stays the pair floor either way; only whether the
    # net CLASSES are capped by it moves.
    cfg = dict(shared)
    cfg['clearance_ceiling'] = None
    cfg['clearance'] = 0.2
    kw = _drive(cfg)
    check("the flat clearance is handed over regardless of the switch",
          kw.get('clearance') == 0.2, "got %r" % (kw.get('clearance'),))

    # -- 7. the plan executor must not turn OMITTED into GIVEN --------------
    # `optimize_caps` deliberately skips the per-step reset so it inherits the
    # preceding fanout step's controls, which is right for the VALUE. Since
    # #768 the PRESENCE of --clearance is a semantic switch, so a step carrying
    # no `clearance` param must also clear the override, or an omitted flag
    # arrives at the engine as a ceiling. Replays the executor's own rule.
    def _executor_rule(step):
        _p = step.get("params") or {}
        _runs_caps = (step["action"] == "optimize_caps"
                      or (step["action"] == "fanout"
                          and _p.get("optimize_caps")))
        if _runs_caps and not _p.get("clearance"):
            cc = getattr(dlg, 'clearance_check', None)
            if cc is not None and cc.GetValue():
                cc.SetValue(False)
        return bool(getattr(dlg, 'clearance_check').GetValue())

    dlg.clearance_check.SetValue(True)          # the fanout step ticked it
    check("plan: optimize_caps with NO clearance param clears the override",
          _executor_rule({"action": "optimize_caps"}) is False)
    dlg.clearance_check.SetValue(True)
    check("plan: optimize_caps WITH a clearance param keeps it",
          _executor_rule({"action": "optimize_caps",
                          "params": {"clearance": 0.1}}) is True)
    # #780: a FANOUT step that switches the inline cap pass on runs the same
    # pass, so the same rule applies. Reachable only since the inline path
    # started carrying the ceiling at all.
    dlg.clearance_check.SetValue(True)
    check("plan: fanout+optimize_caps with NO clearance clears the override",
          _executor_rule({"action": "fanout",
                          "params": {"optimize_caps": True}}) is False)
    dlg.clearance_check.SetValue(True)
    check("plan: fanout+optimize_caps WITH a clearance param keeps it",
          _executor_rule({"action": "fanout",
                          "params": {"optimize_caps": True,
                                     "clearance": 0.1}}) is True)
    dlg.clearance_check.SetValue(True)
    check("plan: a plain fanout step does NOT clear it",
          _executor_rule({"action": "fanout", "params": {}}) is True)

    # and the rule as SHIPPED. The four arms above run a MIRROR of the
    # executor's predicate, which is only worth anything while the two
    # agree -- so assert the shipped text of the predicate itself, not a
    # bag of loose substrings. The old spelling here was satisfied by
    # 'clearance_check' and 'optimize_caps' appearing ANYWHERE in a
    # 1500-line file, which is true of a file that dropped the rule.
    _plan_src = open(os.path.join(REPO, 'kicad_routing_plugin', 'ai_plan.py'),
                     encoding='utf-8').read()
    _pred = ('_runs_caps = (step["action"] == "optimize_caps"\n'
             '                          or (step["action"] == "fanout"\n'
             '                              and _p.get("optimize_caps")))')
    check("the executor actually carries that rule, verbatim",
          _pred.replace('\n', '') in _plan_src.replace('\r', '')
                                              .replace('\n', '')
          and 'if _runs_caps and not _p.get("clearance"):' in _plan_src
          and "_cc.SetValue(False)" in _plan_src,
          "the mirror above no longer matches the shipped predicate, so "
          "the four arms are testing this file rather than ai_plan.py")

    print()
    if failures:
        print("FAILED (%d): %s" % (len(failures), ", ".join(failures)))
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
