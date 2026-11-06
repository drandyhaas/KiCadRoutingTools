#!/usr/bin/env python3
"""Placement tab Apply makes every footprint's KiCad LOCK equal the result
board's (#1054).

The result file of a placement run carries locks the run decided: the seeder
stamps `(locked yes)` on the fixed_poses it seats (#1054), `place_pose lock`
stamps one, and `place_pose unlock` -- "the only way to move a KiCad-locked
part" -- strips one. Apply used to diff pose and side only, so the live board
kept its own locks: a seated part came back unlocked, and a part the run had
unlocked to move came back moved AND still locked. A lock that changes
without a move is the case a pose diff cannot see at all, so it is asserted
on its own (U9).

Drives the REAL `PlacementTab._on_apply` on a real pcbnew board, with only
the modal confirmation, `pcbnew.GetBoard`/`Refresh` and the zone refill
stubbed. It runs on the sys.path the REAL plugin loader builds -- the repo
root plus py_router / py_placer / py_tools, and NOT kicad_routing_plugin/ --
which is asserted, because every other GUI gate adds the plugin directory and
so could not see that an absolute `from gui_utils import` inside Apply raised
ModuleNotFoundError in KiCad itself.

Needs pcbnew; re-execs into KiCad's python automatically. Does not
self-skip: a gate that exits 0 without running reports what it guards as
checked.

    python3 tests/gui_parity/test_placement_apply_locks.py
"""
import glob
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# The real loader's shape (root __init__.py + action_plugin.py): repo root and
# py_router first, py_placer / py_tools appended. Never kicad_routing_plugin/.
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
for _d in ('py_placer', 'py_tools'):
    sys.path.append(os.path.join(REPO, _d))

from kicad_locate import path_version_key  # noqa: E402
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]

FIXTURE = os.path.join(REPO, 'kicad_files', 'interf_u_unrouted.kicad_pcb')
#: unlocked in the input; LOCKED in the result at the SAME pose -- the
#: seeder's seated fixed_pose. No pose diff, so only a lock pass reaches it.
LOCK_IN_PLACE = 'U9'
#: unlocked in the input; moved AND locked in the result.
MOVE_AND_LOCK = 'C3'
#: LOCKED in the input; unlocked and moved in the result (`place_pose unlock`).
UNLOCK_AND_MOVE = 'P1'
#: locked in both, never moved: the control.
STAYS_LOCKED = 'BUS1'


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        r = subprocess.run([cand, '-c', 'import pcbnew, wx'], capture_output=True)
        if r.returncode == 0:
            # os.execv re-splits argv on spaces on Windows ("Program Files").
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew+wx found; this gate does not self-skip")
    sys.exit(2)


def run():
    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import pcbnew
    import wx
    from kicad_parser import mm_to_iu, parse_kicad_pcb

    failures = []

    def check(label, ok, detail=""):
        print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f" ({detail})" if detail else ""))
        if not ok:
            failures.append(label)

    # The premise this gate's sys.path stands for. If the plugin directory
    # were importable here the gate would be measuring the other gates' shape.
    _premise = importlib.util.find_spec('gui_utils') is None
    check("plugin dir is NOT on sys.path (the real loader's shape)", _premise,
          "" if _premise else "gui_utils resolves top-level, so this gate "
                              "cannot see an absolute sibling import")

    app = wx.App(False)  # noqa: F841  (before any wx object)
    wx.Log.SetActiveTarget(wx.LogStderr())
    from kicad_routing_plugin import gui_utils
    from kicad_routing_plugin.placement_gui import PlacementTab

    tmp = tempfile.mkdtemp(prefix='krt_apply_locks_')
    try:
        live_path = os.path.join(tmp, 'live.kicad_pcb')
        result_path = os.path.join(tmp, 'final.kicad_pcb')
        shutil.copyfile(FIXTURE, live_path)

        # The RESULT board, authored by pcbnew itself so its lock tokens are
        # the ones KiCad writes.
        res = pcbnew.LoadBoard(live_path)
        by_ref = {f.GetReference(): f for f in res.GetFootprints()}

        def nudge(fp, dx_mm):
            p = fp.GetPosition()
            fp.SetPosition(pcbnew.VECTOR2I(p.x + mm_to_iu(dx_mm), p.y))
        by_ref[LOCK_IN_PLACE].SetLocked(True)
        nudge(by_ref[MOVE_AND_LOCK], 2.0)
        by_ref[MOVE_AND_LOCK].SetLocked(True)
        by_ref[UNLOCK_AND_MOVE].SetLocked(False)
        nudge(by_ref[UNLOCK_AND_MOVE], -1.5)
        pcbnew.SaveBoard(result_path, res)

        # The fixture must say what the test claims, through the SAME parse
        # Apply uses -- or the lock checks below are vacuous.
        parsed = parse_kicad_pcb(result_path)
        want = {r: bool(fp.locked) for r, fp in parsed.footprints.items()}
        inp = {r: bool(fp.locked) for r, fp in parse_kicad_pcb(live_path).footprints.items()}
        check("fixture: input locks as declared",
              inp[STAYS_LOCKED] and inp[UNLOCK_AND_MOVE]
              and not inp[LOCK_IN_PLACE] and not inp[MOVE_AND_LOCK],
              str({r: inp[r] for r in (LOCK_IN_PLACE, MOVE_AND_LOCK,
                                        UNLOCK_AND_MOVE, STAYS_LOCKED)}))
        check("fixture: result locks as authored (text parse of pcbnew's file)",
              want[STAYS_LOCKED] and want[LOCK_IN_PLACE] and want[MOVE_AND_LOCK]
              and not want[UNLOCK_AND_MOVE],
              str({r: want[r] for r in (LOCK_IN_PLACE, MOVE_AND_LOCK,
                                         UNLOCK_AND_MOVE, STAYS_LOCKED)}))

        live = pcbnew.LoadBoard(live_path)     # in memory only; never saved
        live_by_ref = {f.GetReference(): f for f in live.GetFootprints()}
        before = {r: (pcbnew.ToMM(f.GetPosition().x), pcbnew.ToMM(f.GetPosition().y))
                  for r, f in live_by_ref.items()}

        frame = wx.Frame(None)
        logs = []
        tab = PlacementTab(frame, parse_kicad_pcb(live_path), live_path,
                           append_log=logs.append)
        tab._pending_result = {"board": result_path}
        tab._mode = "place"

        asked = []
        saved = (wx.MessageBox, pcbnew.GetBoard, pcbnew.Refresh,
                 gui_utils.refill_all_zones)

        def _yes(message, *a, **k):
            asked.append(message)
            return wx.YES
        wx.MessageBox = _yes
        pcbnew.GetBoard = lambda: live
        pcbnew.Refresh = lambda: None
        gui_utils.refill_all_zones = lambda board: 0
        raised = None
        try:
            tab._on_apply(None)
        except Exception as e:                            # noqa: BLE001
            import traceback
            traceback.print_exc()
            raised = f"{type(e).__name__}: {e}"
        finally:
            (wx.MessageBox, pcbnew.GetBoard, pcbnew.Refresh,
             gui_utils.refill_all_zones) = saved
        check("Apply ran to completion", raised is None, raised or "")
        check("Apply asked for confirmation", len(asked) == 1, f"{len(asked)} dialogs")

        got = {r: bool(f.IsLocked()) for r, f in live_by_ref.items()}
        differ = sorted(r for r in want if r in got and got[r] != want[r])
        check("EVERY live lock equals the result board's", not differ,
              ", ".join(f"{r}: live {got[r]} result {want[r]}" for r in differ))
        check(f"{LOCK_IN_PLACE} locked although it did not move (no pose diff)",
              got[LOCK_IN_PLACE]
              and before[LOCK_IN_PLACE] == (pcbnew.ToMM(live_by_ref[LOCK_IN_PLACE].GetPosition().x),
                                            pcbnew.ToMM(live_by_ref[LOCK_IN_PLACE].GetPosition().y)))
        moved_c = pcbnew.ToMM(live_by_ref[MOVE_AND_LOCK].GetPosition().x) - before[MOVE_AND_LOCK][0]
        check(f"{MOVE_AND_LOCK} moved and locked",
              got[MOVE_AND_LOCK] and abs(moved_c - 2.0) < 1e-6, f"dx {moved_c:.4f}")
        moved_p = pcbnew.ToMM(live_by_ref[UNLOCK_AND_MOVE].GetPosition().x) - before[UNLOCK_AND_MOVE][0]
        check(f"{UNLOCK_AND_MOVE} moved and UNLOCKED, as the result board says",
              not got[UNLOCK_AND_MOVE] and abs(moved_p + 1.5) < 1e-6,
              f"locked {got[UNLOCK_AND_MOVE]}, dx {moved_p:.4f}")
        check(f"{STAYS_LOCKED} still locked (control)", got[STAYS_LOCKED])

        # Disclosed BEFORE anything changes: an unlock the user did not see
        # coming is the change this dialog exists to show.
        msg = asked[0] if asked else ""
        check("the dialog names the parts it will lock",
              LOCK_IN_PLACE in msg and MOVE_AND_LOCK in msg and "lock" in msg.lower())
        check("the dialog names the part it will UNLOCK",
              "unlock" in msg.lower() and UNLOCK_AND_MOVE in msg)
        status = tab.status_text.GetLabel().replace("\n", " ")
        check("the status line counts the lock changes", "lock" in status.lower(),
              status)
        frame.Destroy()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nVERDICT: FAIL ({len(failures)}): " + "; ".join(failures))
        return 1
    print("\nVERDICT: placement Apply mirrors the result board's footprint locks")
    return 0


def main():
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(run())


if __name__ == '__main__':
    main()
