#!/usr/bin/env python3
"""#900 on a REAL pcbnew board: does `apply_targets_to_board` write the net
classes at the ROUTED clearance, or at the capped rule floor?

    python3 tests/gui_parity/test_900_live_class_clearance.py

(re-execs into KiCad's bundled python automatically, like its siblings)

THE DEFECT, and why it is a GUI defect and not only a CLI one. `compute_targets`
caps `rules.min_clearance` at the smallest copper-pad clearance override on the
board (#530), because KiCad floors an override there. `apply_targets_to_board`
then read that same capped key for the Default net class, and
`clamp_nondefault_netclasses_on_board` for every other class. The signal, planes
and differential tabs all call `apply_targets_to_board` with
`minima=board_minima_from_live(board)` -- which carries the override -- and only
afterwards call `gui_utils.update_live_drc_floors`, whose class write is correct
but ONLY-LOWER, so it could never undo the capped value written moments before.
One part with a 2 mil library override therefore shipped a 0.0508 mm board out
of the GUI as well.

WHY THIS FILE AND NOT THE WX-FREE ONE. `tests/test_900_class_clearance_not_capped.py`
drives the FILE writeback end to end, the non-Default clamp against fake net
classes, and `apply_targets_to_board` by SOURCE TEXT only -- that function opens
with `import pcbnew`. A source guard is the weaker instrument (#780: the arm it
replaced there had been green throughout the period the defect existed), and it
is spelling-sensitive: a revert written `(targets or {}).get("min_clearance")`
would not trip its absence half. This file grades the behaviour.

THE FIXTURE. `flat_hierarchy` is the repo's only tracked board declaring a
NON-Default class (Default 0.2, Wide 0.4) -- the same reason the #768 and #782
gates use it -- and it carries NO pad clearance override, which is exactly why
neither of those gates could see this bug. The override is set here, on the
loaded board, through pcbnew's own setter: the quantity under test is what
`board_minima_from_live` reads back, so setting it any other way would be
testing the text parser instead.

ORDERING: 0.0508 (the override) < 0.15 (the routed clearance) < 0.2 (Default)
< 0.4 (Wide). Every class is above the routed value and the routed value is
above the override, so a class landing on 0.0508 and a class landing on 0.15 are
distinguishable, and so is a class that was not written at all.
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
OVERRIDE = 0.0508
ROUTED = 0.15
DEFAULT_DECLARED = 0.2
WIDE_DECLARED = 0.4


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


def _set_override(pad, mm):
    """pcbnew's local-clearance setter, across the shapes KiCad ships."""
    iu = int(round(mm * MM))
    for name in ('SetLocalClearance', 'SetClearance'):
        fn = getattr(pad, name, None)
        if fn is None:
            continue
        try:
            fn(iu)
            return True
        except Exception:                                      # noqa: BLE001
            try:                       # KiCad 10: std::optional<int>
                fn(pcbnew.optional_int(iu))                    # noqa: F821
                return True
            except Exception:                                  # noqa: BLE001
                continue
    return False


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import pcbnew
    globals()['pcbnew'] = pcbnew
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.dirname(REPO))

    from fix_kicad_drc_settings import compute_targets, apply_targets_to_board
    from kicad_routing_plugin.gui_utils import board_minima_from_live

    failures = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}"
              + (f"   [{detail}]" if not cond and detail != '' else ''))
        if not cond:
            failures.append(name)

    src = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    td = tempfile.mkdtemp()
    dst = os.path.join(td, 'b.kicad_pcb')
    shutil.copyfile(src, dst)
    for ext in ('.kicad_pro', '.kicad_dru'):
        s = os.path.splitext(src)[0] + ext
        if os.path.isfile(s):
            shutil.copyfile(s, os.path.splitext(dst)[0] + ext)

    board = pcbnew.LoadBoard(dst)
    bds = board.GetDesignSettings()
    ns = bds.m_NetSettings
    default_nc = ns.GetDefaultNetclass()

    def _classes():
        out = {}
        for getter in ('GetNetclasses', 'GetNetClasses'):
            m = getattr(ns, getter, None) or getattr(bds, getter, None)
            if m is None:
                continue
            try:
                d = {str(k): v for k, v in m().items()}
            except Exception:                                  # noqa: BLE001
                continue
            if d:
                out = d
                break
        return out

    # ---- ON THE BRANCH -----------------------------------------------------
    print('\n-- 0. the fixture really is on the branch --')
    others = {k: v for k, v in _classes().items()
              if k != 'Default' and v is not default_nc}
    check('the board declares a non-Default class', bool(others),
          sorted(_classes()))
    wide = others.get('Wide')
    check('Wide is declared at 0.4',
          wide is not None and abs(wide.GetClearance() / MM - WIDE_DECLARED) < 1e-6,
          None if wide is None else wide.GetClearance() / MM)
    check('Default is declared at 0.2',
          abs(default_nc.GetClearance() / MM - DEFAULT_DECLARED) < 1e-6,
          default_nc.GetClearance() / MM)

    placed = False
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            # Any COPPER pad that is not NPTH -- the population
            # `board_minima_from_live` scans. Restricting this to SMD found no
            # pad at all on this fixture and the arm reported a fixture fault.
            if (pad.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH
                    and _set_override(pad, OVERRIDE)):
                placed = True
                break
        if placed:
            break
    check('a pad clearance override could be set through pcbnew', placed)
    minima = board_minima_from_live(board)
    check('board_minima_from_live reads it back',
          abs((minima.get('min_pad_clearance_override') or 0) - OVERRIDE) < 1e-6,
          minima.get('min_pad_clearance_override'))

    if failures:
        print('\nFAIL: the fixture is not on the branch; the arms below would '
              'grade nothing.')
        return 1

    # ---- THE ARM -----------------------------------------------------------
    print('\n-- 1. apply_targets_to_board with the override present --')
    targets = compute_targets(clearance=ROUTED, minima=minima)
    check('compute_targets capped the RULE floor',
          abs(targets['min_clearance'] - OVERRIDE) < 1e-9, targets['min_clearance'])
    apply_targets_to_board(board, targets, {}, clamp_nondefault_netclasses=True)

    check('m_MinClearance is the capped floor (#530 kept)',
          abs(bds.m_MinClearance / MM - OVERRIDE) < 1e-6, bds.m_MinClearance / MM)
    check('the Default class is the ROUTED 0.15, not the cap',
          abs(default_nc.GetClearance() / MM - ROUTED) < 1e-6,
          default_nc.GetClearance() / MM)
    check('the Wide class clamps to the routed 0.15, not the cap',
          abs(wide.GetClearance() / MM - ROUTED) < 1e-6, wide.GetClearance() / MM)

    # ---- THE OFF ARM -------------------------------------------------------
    # A gate that only checked "the classes land at the routed value" would pass
    # a writer that ignores its argument and always writes 0.15.
    print('\n-- 2. control: a DIFFERENT routed value moves them with it --')
    board2 = pcbnew.LoadBoard(dst)
    bds2 = board2.GetDesignSettings()
    nc2 = bds2.m_NetSettings.GetDefaultNetclass()
    for fp in board2.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH:
                _set_override(pad, OVERRIDE)
                break
        break
    apply_targets_to_board(
        board2, compute_targets(clearance=0.09,
                                minima=board_minima_from_live(board2)),
        {}, clamp_nondefault_netclasses=True)
    check('the Default class follows the routed value (0.09)',
          abs(nc2.GetClearance() / MM - 0.09) < 1e-6, nc2.GetClearance() / MM)

    print()
    if failures:
        print(f"FAIL: {len(failures)} check(s): {failures}")
        return 1
    print('PASS: the live-board writeback caps m_MinClearance and writes the '
          'net classes at the routed clearance (#900)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
