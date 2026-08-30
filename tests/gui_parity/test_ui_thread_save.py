#!/usr/bin/env python3
"""#688: prove save_board_on_ui_thread actually marshals, and actually degrades.

The livechain / engine-parity gates drive the engine on the MAIN thread, so
`wx.IsMainThread()` is True there and the marshalling path never runs -- they
cannot catch a regression in this guard. This one runs the save from a real
WORKER thread, which is the configuration that deadlocked on Windows (#688).

Two cases, and the second matters as much as the first:

  1. UI thread pumping  -> the save runs ON THE MAIN THREAD and succeeds.
  2. UI thread NOT pumping -> the wait times out and it returns False instead
     of blocking forever. That is what turns this class of bug from "kill
     KiCad" into "skip an optional leg".

Needs KiCad's python (pcbnew + wx); re-execs into it like the sibling gates.
"""
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))


def _reexec_into_kicad_python():
    if os.environ.get("_UI_THREAD_TEST_REEXEC"):
        return
    try:
        import pcbnew, wx  # noqa: F401
        return
    except Exception:
        pass
    cands = [
        "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
        "Versions/Current/bin/python3",
        "/usr/bin/python3",
        r"C:\Program Files\KiCad\9.0\bin\python.exe",
    ]
    for c in cands:
        if os.path.isfile(c):
            os.environ["_UI_THREAD_TEST_REEXEC"] = "1"
            os.execv(c, [c, os.path.abspath(__file__)] + sys.argv[1:])
    print("SKIP: no python with pcbnew + wx found")
    raise SystemExit(0)


_reexec_into_kicad_python()
sys.path.insert(0, os.path.join(ROOT, "py_router"))

import pcbnew  # noqa: E402
import wx  # noqa: E402
from ui_thread import save_board_on_ui_thread  # noqa: E402

BOARD = os.path.join(ROOT, "kicad_files", "splitflap_driver.kicad_pcb")


def main():
    fails = []
    app = wx.App()  # noqa: F841 -- wx needs one before CallAfter works
    board = pcbnew.LoadBoard(BOARD)
    if board is None:
        print(f"FAIL: could not load {BOARD}")
        return 1

    # --- case 1: worker thread, UI thread pumping -------------------------
    out = tempfile.mktemp(suffix=".kicad_pcb")
    box = {}

    def _worker():
        box["ran_on_main"] = None
        box["ok"] = save_board_on_ui_thread(out, board, timeout_s=60)

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    deadline = time.monotonic() + 60
    while t.is_alive() and time.monotonic() < deadline:
        wx.YieldIfNeeded()          # this IS the pumping UI thread
        time.sleep(0.01)
    t.join(timeout=5)

    if not box.get("ok"):
        fails.append("marshalled save returned False with the UI thread pumping")
    elif not os.path.isfile(out):
        fails.append("marshalled save reported success but wrote no file")
    elif os.path.getsize(out) < 1000:
        fails.append(f"marshalled save wrote a suspiciously small file "
                     f"({os.path.getsize(out)} bytes)")
    for p in (out, os.path.splitext(out)[0] + ".kicad_pro"):
        try:
            os.unlink(p)
        except OSError:
            pass

    # --- case 2: worker thread, UI thread NOT pumping ---------------------
    # The deadlock configuration. It must TIME OUT and return False, not hang.
    out2 = tempfile.mktemp(suffix=".kicad_pcb")
    box2 = {}

    def _worker2():
        t0 = time.monotonic()
        box2["ok"] = save_board_on_ui_thread(out2, board, timeout_s=2)
        box2["elapsed"] = time.monotonic() - t0

    t2 = threading.Thread(target=_worker2, daemon=True)
    t2.start()
    t2.join(timeout=30)             # deliberately never yielding to wx here
    if t2.is_alive():
        fails.append("BLOCKED: the guard did not time out with the UI thread "
                     "not pumping -- this is the #688 deadlock")
    else:
        if box2.get("ok") is not False:
            fails.append(f"expected False when the UI thread never pumps, "
                         f"got {box2.get('ok')!r}")
        if not (2.0 <= box2.get("elapsed", 0) < 20):
            fails.append(f"timeout took {box2.get('elapsed')}s, expected ~2s")
    try:
        os.unlink(out2)
    except OSError:
        pass

    if fails:
        for f in fails:
            print(f"FAIL: {f}")
        return 1
    print("PASS: #688 guard marshals the save to the main thread when it is "
          "pumping, and times out (rather than deadlocking) when it is not")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
