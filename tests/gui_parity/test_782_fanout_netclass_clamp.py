#!/usr/bin/env python3
"""#782 on a REAL pcbnew board: does the fanout tab's cap pass actually LOWER the
non-Default net classes it priced at the #768 ceiling?

    python3 tests/gui_parity/test_782_fanout_netclass_clamp.py

(re-execs into KiCad's bundled python automatically, like its siblings)

THE DEFECT. #768 gave the cap pass a netclass ceiling with the CLI's contract:
GIVEN a `--clearance`, price every class at min(class, it) AND clamp the output
down to it. The fanout tab did the first half only. It finishes with
`gui_utils.update_live_drc_floors`, which writes `m_MinClearance` and the DEFAULT
class, and it reached `apply_targets_to_board` -- where the non-Default clamp
lived -- from nowhere. So a Wide-class pair was priced at min(0.4, 0.2) and then
graded by KiCad at the still-0.4 class: violations on copper the pass considered
legal.

WHY THIS FILE AND NOT THE WX-FREE ONE. `tests/test_782_nondefault_netclass_clamp.py`
grades the clamp's semantics against FAKE net-class objects and asserts the two
call sites by source text. Neither is the risky part. The risky parts are the
ones only a real board exercises:

  * the pcbnew net-class ENUMERATION, whose API varies by KiCad version -- the
    helper probes `m_NetSettings.GetNetclasses` then `GetNetClasses` and no-ops
    on an unknown shape, which is indistinguishable from "worked" unless a real
    board proves a class actually moved;
  * that `update_live_drc_floors` REACHES the clamp at all. Its body opens with
    `import pcbnew` inside the try, so with no KiCad python the whole function
    silently no-ops and a wx-free arm would grade a function that never ran.

THE FIXTURE. `flat_hierarchy` is the repo's only tracked board declaring a
NON-Default class (Default 0.2, Wide 0.4) -- the same reason the #768 gate uses
it. A single-class board cannot tell a working clamp from a broken one, because
there is nothing to lower.

PRESENCE IS THE SWITCH, and both directions are graded. A gate that only checked
"the ceiling clamps" would pass a clamp that fires unconditionally, which is
#768's OMITTED branch broken the other way -- it would silently rewrite the
classes of a board whose spec the operator chose to honour.

WHAT THIS GATE DOES NOT COVER, stated rather than implied. Verified as a change
detector: with both halves of the fix reverted it fails arms 1 and 3 by name and
the two PRESERVE arms correctly still pass. But arm 1 drives
`update_live_drc_floors` DIRECTLY, so it proves the mechanism the inline path
uses and NOT that `_apply_fanout_results` passes `clearance_ceiling` into it --
that one line is held only by a source-text guard in the wx-free half. Driving it
for real needs a whole BGA fanout apply, which is a different and much slower
gate. The standalone path IS driven end to end (arms 3/4), so the two call sites
are not covered to the same depth; #780 is the reminder that a source guard is
the weaker instrument, since the arm it replaced there had been green throughout
the period the defect existed.
"""
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    r'C:\Program Files\KiCad\10.0\bin\python.exe',
]

MM = 1e6
CEILING = 0.3          # between Default 0.2 and Wide 0.4 -- see below
WIDE_DECLARED = 0.4
DEFAULT_DECLARED = 0.2


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
    import pcbnew as _pcbnew
    board_path = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')

    failures = []

    def check(name, cond, detail="", info=""):
        # `detail` is the FAILURE explanation and prints only on failure;
        # `info` is the MEASUREMENT and prints either way -- the convention
        # the #768 and #733 gates settled on.
        if not cond:
            failures.append(name)
        print(("  PASS " if cond else "  FAIL ") + name
              + (f"  {info}" if info else "")
              + (f"  {detail}" if detail and not cond else ""))

    _tmpdirs = []

    def fresh_board(tag):
        """A board per arm, at its OWN PATH. Measured, and it is not paranoia:

        `pcbnew.LoadBoard(same_path)` returns a DIFFERENT BOARD object whose net
        classes are the SAME ones -- KiCad holds project settings per project in
        memory (which is the premise `update_live_drc_floors`' own docstring
        rests on), so a second load of one path hands back the first load's
        mutated classes. Probed directly: load, lower Wide 0.4 -> 0.3, re-load
        the same path, read 0.3 with `b2 is b1` False.

        The clamp under test MUTATES net classes, so with a shared path arms 1
        and 3 would pre-satisfy arms 2 and 4 and the PRESERVE direction would
        report PASS having measured the previous arm's leftovers -- the
        order-dependent green this file's own "both directions" rule exists to
        prevent. Copying to a fresh path per arm keys them separately; verified
        by the same probe.

        The sibling `.kicad_pro` goes with it, always: it carries the net-class
        declarations, and a board copied without it loads with the STOCK classes
        (#441). That would make every arm read a Wide that is not the fixture's.
        """
        d = tempfile.mkdtemp(prefix=f'test782_{tag}_')
        _tmpdirs.append(d)
        dst = os.path.join(d, 'b.kicad_pcb')
        shutil.copyfile(board_path, dst)
        pro = os.path.splitext(board_path)[0] + '.kicad_pro'
        if os.path.isfile(pro):
            shutil.copyfile(pro, os.path.splitext(dst)[0] + '.kicad_pro')
        return _pcbnew.LoadBoard(dst)

    def classes_of(board):
        """{name: clearance in mm} for every class the board declares.

        TWO SOURCES, and that is a finding rather than defensiveness. On the
        KiCad this runs against (10.0.0), `m_NetSettings.GetNetclasses()`
        returns the NON-Default classes ONLY -- the Default is reachable just
        through `GetDefaultNetclass()`. The first version of this reader used
        the enumeration alone, read `{Wide: 0.4}`, and refused its own fixture.

        Worth stating because the clamp under test skips the Default class by
        identity AND by name: on this build neither guard is what saves it, the
        enumeration simply never offers it. The guards still earn their place
        for the older shape (`bds.GetNetClasses()`, which DOES include it) --
        they are just not what this gate is exercising.

        Keys are normalised with `str()`: pcbnew hands back `wxString`, which
        prints as `wxString('Wide')` and makes a mismatch report unreadable.
        """
        bds = board.GetDesignSettings()
        ns = getattr(bds, 'm_NetSettings', None)
        out = {}
        for getter in ('GetNetclasses', 'GetNetClasses'):
            src = (ns if ns is not None and hasattr(ns, getter)
                   else (bds if hasattr(bds, getter) else None))
            if src is None:
                continue
            try:
                m = getattr(src, getter)()
                items = (dict(m.items()) if hasattr(m, 'items')
                         else {k: m[k] for k in m.keys()})
                for k, nc in items.items():
                    try:
                        out[str(k)] = nc.GetClearance() / MM
                    except Exception:
                        pass
                if out:
                    break
            except Exception:
                pass
        if 'Default' not in out:
            for getter in ('GetDefaultNetclass',):
                src = (ns if ns is not None and hasattr(ns, getter)
                       else (bds if hasattr(bds, getter) else None))
                if src is None:
                    continue
                try:
                    out['Default'] = getattr(src, getter)().GetClearance() / MM
                except Exception:
                    pass
        return out

    # ---------------------------------------------------------------- fixture
    probe = fresh_board('probe')
    if probe is None:
        print("SKIP: pcbnew could not load the fixture board")
        return 0
    declared = classes_of(probe)
    print(f"\nfixture net classes as loaded: {declared}")
    # EVIDENCE BEFORE VERDICT: if the fixture does not actually declare the two
    # classes, every arm below is vacuous and would report PASS by measuring
    # nothing. Refuse instead (the run_utils.evidence rule, applied to a board).
    if abs(declared.get('Wide', -1) - WIDE_DECLARED) > 1e-9 \
            or abs(declared.get('Default', -1) - DEFAULT_DECLARED) > 1e-9:
        print(f"BROKEN FIXTURE: expected Default {DEFAULT_DECLARED} / Wide "
              f"{WIDE_DECLARED}, read {declared}. Every arm below would be "
              f"vacuous, so this is a refusal rather than a skip.")
        return 2

    from kicad_routing_plugin.gui_utils import update_live_drc_floors

    # -- 1. update_live_drc_floors, ceiling GIVEN ---------------------------
    # The delegation, on the real enumeration API.
    print("\n-- 1. update_live_drc_floors WITH a ceiling --")
    b1 = fresh_board('arm1_ceiling')
    update_live_drc_floors(b1, clearance=DEFAULT_DECLARED,
                           nondefault_clamp_mm=CEILING)
    c1 = classes_of(b1)
    check("the non-Default class is lowered to the ceiling",
          abs(c1.get('Wide', -1) - CEILING) < 1e-9,
          f"Wide is {c1.get('Wide')}, expected {CEILING} -- the clamp did not "
          f"reach the real pcbnew net-class enumeration",
          info=f"Wide {WIDE_DECLARED} -> {c1.get('Wide')}")
    # The ceiling is deliberately ABOVE the Default class, which is the case
    # #768 says must not be clamped to the effective clearance: min(Default,
    # ceiling) is 0.2, and clamping Wide to 0.2 would ship a class BELOW what
    # the pass priced it at. That is #768's own shape in the safe direction,
    # and it is still #768's shape.
    check("...to the CEILING, not to the effective clearance",
          abs(c1.get('Wide', -1) - DEFAULT_DECLARED) > 1e-9,
          f"Wide is {c1.get('Wide')} = the effective clearance, so the caller "
          f"passed `clearance` where it owed `clearance_ceiling`")
    check("the Default class is not RAISED by the clamp",
          c1.get('Default', 9) <= DEFAULT_DECLARED + 1e-9,
          f"Default moved up to {c1.get('Default')}",
          info=f"Default {c1.get('Default')}")

    # -- 2. update_live_drc_floors, ceiling OMITTED --------------------------
    print("\n-- 2. update_live_drc_floors with NO ceiling (#768 OMITTED) --")
    b2 = fresh_board('arm2_omitted')
    update_live_drc_floors(b2, clearance=DEFAULT_DECLARED,
                           nondefault_clamp_mm=None)
    c2 = classes_of(b2)
    check("the non-Default class is PRESERVED",
          abs(c2.get('Wide', -1) - WIDE_DECLARED) < 1e-9,
          f"Wide is {c2.get('Wide')}, expected the declared {WIDE_DECLARED} -- "
          f"a clamp that fires without a ceiling rewrites the spec of a board "
          f"whose classes the operator chose to honour",
          info=f"Wide stays {c2.get('Wide')}")

    # -- 3/4. the STANDALONE cap button, through the real dialog -------------
    # The second interactive path. It builds its own cfg from get_shared_params
    # and wrote no DRC settings at all before #782, so which button the operator
    # pressed decided what the board shipped.
    #
    # `_optimize_decoupling_caps` is stubbed: this gate is about the WRITEBACK
    # that runs after it, and letting the engine actually move caps would make
    # the arm slow and its failure ambiguous between "did not clamp" and "the
    # cap pass errored". `pcbnew.GetBoard()` is None outside the KiCad process,
    # so the tab is handed the loaded fixture the same way the #768 gate does.
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog

    for ticked, label, want in ((True, "override CHECKED", CEILING),
                                (False, "override unchecked", WIDE_DECLARED)):
        print(f"\n-- {'3' if ticked else '4'}. standalone run_cap_optimization,"
              f" {label} --")
        dlg = RoutingDialog(None, parse_kicad_pcb(board_path), board_path)
        tab = dlg.fanout_tab
        dlg.clearance_check.SetValue(ticked)
        dlg.clearance.SetValue(CEILING)
        live = fresh_board('arm3_checked' if ticked else 'arm4_unchecked')
        seen_cfg = {}

        def _stub(_board, _pcbnew_mod, cfg):
            seen_cfg.clear()
            seen_cfg.update(cfg)
            return "stubbed cap pass"

        tab._optimize_decoupling_caps = _stub
        real_getboard = _pcbnew.GetBoard
        _pcbnew.GetBoard = lambda: live
        try:
            tab.run_cap_optimization()
        finally:
            _pcbnew.GetBoard = real_getboard
        got = classes_of(live)
        # Reaching the engine is asserted separately from the clamp, so a stub
        # that never ran cannot read as "the clamp preserved the class".
        check(f"{label}: the cap pass was actually reached",
              bool(seen_cfg),
              "run_cap_optimization returned without calling the cap pass, so "
              "the class reading below measures nothing")
        check(f"{label}: cfg carries the ceiling {('0.3' if ticked else 'None')}",
              (abs((seen_cfg.get('clearance_ceiling') or -1) - CEILING) < 1e-9
               if ticked else seen_cfg.get('clearance_ceiling') is None),
              f"clearance_ceiling={seen_cfg.get('clearance_ceiling')!r}",
              info=f"clearance_ceiling={seen_cfg.get('clearance_ceiling')!r}")
        check(f"{label}: Wide -> {want}",
              abs(got.get('Wide', -1) - want) < 1e-9,
              f"Wide is {got.get('Wide')}, expected {want}",
              info=f"Wide {WIDE_DECLARED} -> {got.get('Wide')}")
        dlg.Destroy()

    for _d in _tmpdirs:
        shutil.rmtree(_d, ignore_errors=True)

    print()
    if failures:
        print(f"FAILED ({len(failures)}): " + ", ".join(failures))
        return 1
    print("ALL PASS -- the cap pass clamps the classes it priced at the "
          "ceiling, on both interactive paths, and preserves them without one")
    return 0


if __name__ == '__main__':
    sys.exit(main())
