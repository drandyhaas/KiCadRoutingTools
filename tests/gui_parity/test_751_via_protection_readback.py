#!/usr/bin/env python3
"""#751: the GUI read half of the via-protection round trip, on a pcbnew whose
SWIG wrapper does NOT export the protection enums.

`_pcbnew_via_protection_attrs` compares each mode against `pcbnew.TENTING_MODE_*`
and friends. The shipping KiCad 10.0.0 wrapper exports the SETTERS but not those
values, and hands back an opaque SwigPyObject from the getters -- so building the
comparison table raised inside the function's own `try` and it returned {} for
EVERY via. `build_pcb_data_from_board` therefore reported no spec at all, and a
user who set via-in-pad protection in the GUI lost it to any pass that re-places
a via (the #313 cap nudge, a rip-up, a tap relocation).

It is BUILD-dependent, not version-dependent -- 10.0.0-103-gacbf1898e0 on macOS
exports all ten constants and returns plain ints -- so this gate does not assume
either environment. It runs BOTH:

  * the wrapper as installed, whatever it is; and
  * a SIMULATED blind wrapper (the ten constants deleted from the module), which
    is the reporter's environment reproduced on any machine.

The second arm carries its own NEGATIVE CONTROL: it first asserts that the raw
live-object reader really does go blind (returns {}), so the arm cannot pass on
`{} == {}` -- the exact vacuity #751 warns about -- and only then asserts that
the resolver still delivers the full spec from the board file.

Needs pcbnew; re-execs into KiCad's python automatically.

    python3 tests/gui_parity/test_751_via_protection_readback.py
"""
import glob
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

# IPC-4761 Type VII (filled + capped + plated), the #489 s8 motivating spec:
# every token in the family, so a partial reader is caught as well as a blind one.
TYPE_VII = {
    'tenting': '(front no) (back no)',
    'covering': '(front yes) (back yes)',
    'plugging': '(front yes) (back yes)',
    'capping': 'yes',
    'filling': 'yes',
}
VIA_UUID = "22222222-2222-2222-2222-222222222222"

BOARD = '''(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(44 "Edge.Cuts" user)
\t)
\t(setup (pad_to_mask_clearance 0))
\t(net 0 "")
\t(net 1 "/SIG")
\t(gr_line (start 0 0) (end 40 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "11111111-1111-1111-1111-111111111111"))
\t(gr_line (start 40 0) (end 40 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "11111111-1111-1111-1111-111111111112"))
\t(gr_line (start 40 40) (end 0 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "11111111-1111-1111-1111-111111111113"))
\t(gr_line (start 0 40) (end 0 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "11111111-1111-1111-1111-111111111114"))
\t(via
\t\t(at 10.5 10.5)
\t\t(size 0.6)
\t\t(drill 0.3)
\t\t(layers "F.Cu" "B.Cu")
\t\t(tenting (front no) (back no))
\t\t(covering (front yes) (back yes))
\t\t(plugging (front yes) (back yes))
\t\t(capping yes)
\t\t(filling yes)
\t\t(net 1)
\t\t(uuid "''' + VIA_UUID + '''")
\t)
)
'''


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable:
            continue
        if os.path.exists(cand):
            r = subprocess.run([cand, '-c', 'import pcbnew'], capture_output=True)
            if r.returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("ERROR: no python with pcbnew found")
    sys.exit(2)


def _blind_pcbnew():
    """Delete the ten protection enums from the live pcbnew module.

    This is the reporter's KiCad 10.0.0 reproduced anywhere: the setters stay,
    the constants go. Returns a restore callable.
    """
    import pcbnew
    from kicad_parser import _PROTECTION_ENUM_NAMES
    saved = {}
    for n in _PROTECTION_ENUM_NAMES:
        if hasattr(pcbnew, n):
            saved[n] = getattr(pcbnew, n)
            delattr(pcbnew, n)

    def restore():
        for n, v in saved.items():
            setattr(pcbnew, n, v)
    return restore, saved


def run():
    import pcbnew
    from kicad_parser import (_extract_via_protection_attrs,
                              _pcbnew_via_protection_attrs,
                              build_pcb_data_from_board,
                              pcbnew_protection_accessors_usable,
                              pcbnew_via_protection_attrs)

    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    workdir = tempfile.mkdtemp(prefix='t751_')
    path = os.path.join(workdir, 'via751.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)

    # The CLI's answer, which is the contract both halves must meet.
    text = _extract_via_protection_attrs(BOARD)
    check(text.get(VIA_UUID) == TYPE_VII,
          f"text parser lost the spec (got {text.get(VIA_UUID)}); the board "
          f"fixture, not the GUI half, is wrong")

    print(f"pcbnew {pcbnew.GetBuildVersion()}")
    print(f"protection enums exported: {pcbnew_protection_accessors_usable()}")

    # --- arm 1: the wrapper as installed -------------------------------------
    board = pcbnew.LoadBoard(path)
    gui = build_pcb_data_from_board(board)
    check(len(gui.vias) == 1, f"builder saw {len(gui.vias)} vias, expected 1")
    if gui.vias:
        got = gui.vias[0].tenting_attrs
        check(got == TYPE_VII,
              f"AS INSTALLED: build_pcb_data_from_board reported {got}, "
              f"expected the full Type-VII spec {TYPE_VII}")

    # --- arm 2: a wrapper that cannot be asked -------------------------------
    restore, saved = _blind_pcbnew()
    try:
        check(not pcbnew_protection_accessors_usable(),
              "the blind-wrapper simulation did not take: the probe still "
              "reports the accessors usable with the constants deleted")

        live_via = None
        for t in board.GetTracks():
            if t.GetClass() == 'PCB_VIA':
                live_via = t
                break
        check(live_via is not None, "no PCB_VIA on the loaded board")

        # NEGATIVE CONTROL. Without this the next assertion could pass on a
        # reader that never went blind, and the arm would be testing nothing.
        if live_via is not None:
            blind = _pcbnew_via_protection_attrs(live_via)
            check(blind == {},
                  f"negative control FAILED: the raw live-object reader "
                  f"returned {blind} on a wrapper with no protection enums, so "
                  f"this arm is not reproducing #751 and proves nothing")

        board2 = pcbnew.LoadBoard(path)
        gui2 = build_pcb_data_from_board(board2)
        check(len(gui2.vias) == 1,
              f"blind wrapper: builder saw {len(gui2.vias)} vias, expected 1")
        if gui2.vias:
            got = gui2.vias[0].tenting_attrs
            check(got == TYPE_VII,
                  f"BLIND WRAPPER: build_pcb_data_from_board reported {got}, "
                  f"expected the full Type-VII spec from the board file")

        if live_via is not None:
            from kicad_parser import via_protection_attrs_from_board_file
            got = pcbnew_via_protection_attrs(
                live_via, via_protection_attrs_from_board_file(board))
            check(got == TYPE_VII,
                  f"BLIND WRAPPER: the resolver returned {got}, expected "
                  f"{TYPE_VII}")
    finally:
        restore()

    check(pcbnew_protection_accessors_usable() == bool(saved) or not saved,
          "the blind-wrapper simulation did not restore the constants")

    # --- the write half, which #751 says already works -----------------------
    from kicad_routing_plugin.gui_utils import apply_via_protection
    nv = pcbnew.PCB_VIA(board)
    check(apply_via_protection(nv, TYPE_VII) is True,
          "apply_via_protection refused a full Type-VII spec; the write half "
          "of the round trip is broken too")
    check(apply_via_protection(nv, {}) is False,
          "apply_via_protection must return early on an empty spec -- that is "
          "what makes an inheriting via keep inheriting")

    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} check(s) FAILED")
        return False
    print("PASS  #751 via-protection readback (as installed + blind wrapper, "
          "with negative control)")
    return True


if __name__ == '__main__':
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(0 if run() else 1)
