#!/usr/bin/env python3
"""#733: the cap-placement edge margin, read off the REAL headless dialog.

`tests/test_733_fanout_clearance_edge_margin.py` covers the engine and greps
`fanout_gui.py` for the kwarg, but the `swig_gui.py` half -- the resolver and
the shared-params key -- was gated by NOTHING. Measured by the #733 review:
deleting `'cap_board_edge_clearance': self._effective_placement_edge_clearance()`
from `get_shared_params`, or making that method `return None` always, left every
one of those tests green while the operator's override silently stopped
travelling.

The three states this asserts, and why each is load-bearing:

  * UNCHECKED -> None, so the engine resolves (CLI parity with an omitted
    --board-edge-clearance). Not `_effective_board_edge_clearance`, which is the
    SIGNAL keep-out: it answers 0.0 on a board with no rule and is then
    fab-floored to 0.20, so reusing it would inset caps at 0.20 instead of 0.55
    -- a 0.35mm relaxation shipped inside the parity fix. The signal key is read
    here too, as the negative control that proves the two are different numbers.
  * CHECKED with a real value -> that value, honoured as typed.
  * CHECKED with ZERO -> None. This is the footgun the review found: the control
    is CREATED at defaults.BOARD_EDGE_CLEARANCE (0.0) with a range minimum of
    0.0, so "ticked the box, typed nothing" is a reachable state, and passing it
    on would inset caps at the bare clearance -- LOOSER than the 0.55 this tab
    used before #733.

Needs KiCad's python (wx + pcbnew); re-execs into it automatically, like its
siblings. Constructs the dialog headless and reads dict values. No routing runs;
seconds, not minutes.

Run: python3 tests/gui_parity/test_733_cap_edge_clearance_gui.py
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
    # Exit 0, like every sibling in this directory -- these gates are run by
    # hand and by contributors who may not have KiCad installed, and a hard
    # failure there is noise, not signal. But say plainly that the gate did NOT
    # run: this file exists because the swig_gui half was covered by nothing,
    # and a silent skip re-creates exactly that state on a machine without wx.
    # CLAUDE.md: probe before recording it as not-run --
    #   <kicad>/bin/python3 -c "import pcbnew, wx; print(wx.version())"
    print("SKIP: no python with pcbnew+wx found -- THIS GATE DID NOT RUN, and "
          "nothing else covers swig_gui._effective_placement_edge_clearance")
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
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'py_placer'))
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.path.insert(0, os.path.dirname(REPO))

    app = wx.App(False)  # noqa: F841  (before any wx object)
    board = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
    from kicad_parser import parse_kicad_pcb
    # routing_dialog is this branch's swig_gui (renamed by the IPC port).
    from kicad_routing_plugin.routing_dialog import RoutingDialog
    from placement.fanout_clearance import (CAP_EDGE_CLEARANCE,
                                            resolve_cap_edge_clearance)

    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    failures = []

    def check(what, ok, detail=''):
        # `detail` is the FAILURE explanation, so print it only on failure --
        # an OK line carrying "the key is absent" reads as a contradiction.
        print(('  OK   ' if ok else '  FAIL ') + what
              + ('' if ok or not detail else ' -- ' + detail))
        if not ok:
            failures.append(what)

    def shared():
        return dlg.fanout_tab.get_shared_params()

    # THESE TWO ARMS WERE RED, and had been since the #733 FOLLOW-UP.
    #
    # They asserted that ticking the Basic tab's SIGNAL edge control makes
    # get_shared_params carry `cap_board_edge_clearance` -- true when this
    # file was written, and deliberately false since `cd623938` gave the cap
    # margin its own control on the BGA panel and stopped the shared params
    # emitting the key at all (swig_gui.py's comment says so in as many
    # words). Its green sibling test_733_cap_edge_control_real_dialog.py
    # asserts the exact inverse. So the gate was not merely stale: it was
    # demanding the behaviour another gate forbids.
    #
    # Measured on `fix/775-fanout-clearance-via-prune`, before any change:
    #     FAIL get_shared_params carries cap_board_edge_clearance
    #     FAIL checked 0.8 -> the typed value travels -- None
    #     VERDICT: 2 FAILURE(S)
    #
    # RE-AIMED rather than deleted. The decoupling is still worth a change
    # detector; it just points the other way now -- the signal control must
    # NOT be able to move the cap margin.
    dlg.edge_clearance_check.SetValue(True)
    dlg.board_edge_clearance.SetValue(0.8)
    s = shared()
    check('get_shared_params does NOT carry cap_board_edge_clearance',
          'cap_board_edge_clearance' not in s,
          'the shared SIGNAL control is feeding the cap PLACEMENT margin '
          'again -- the #733 follow-up decoupled them on purpose')
    check('a signal override of 0.8 leaves the cap knob alone',
          dlg.fanout_tab.bga_options.get_config()
          .get('cap_board_edge_clearance') is None,
          repr(dlg.fanout_tab.bga_options.get_config()
               .get('cap_board_edge_clearance')))

    dlg.edge_clearance_check.SetValue(False)
    s = shared()
    cap = dlg.fanout_tab.bga_options.get_config().get(
        'cap_board_edge_clearance')
    sig = s.get('board_edge_clearance')
    check('the cap knob at its own default -> None, so the ENGINE resolves it',
          cap is None, repr(cap))
    check('...and the engine then answers the placement default',
          resolve_cap_edge_clearance(board, cap)
          == (CAP_EDGE_CLEARANCE, 'fixed default'))
    # NEGATIVE CONTROL: the SIGNAL twin on the same control answers something
    # else entirely, and something SMALLER. Not `sig != cap` -- with cap None
    # that is true of any float and would pass on a build that had wired the
    # two together in the other direction.
    check('the signal keep-out is a real number, not None',
          isinstance(sig, (int, float)),
          f'signal={sig!r} -- the control this compares against is gone')
    check('...and it is BELOW the placement default, so reusing it would have '
          'inset caps closer to the edge',
          isinstance(sig, (int, float)) and sig < CAP_EDGE_CLEARANCE,
          f'signal={sig!r} vs placement default {CAP_EDGE_CLEARANCE}')

    # The footgun: ticked, but never typed in. SetValue(0.0) reproduces the
    # control's own creation state (defaults.BOARD_EDGE_CLEARANCE, range min 0).
    dlg.edge_clearance_check.SetValue(True)
    dlg.board_edge_clearance.SetValue(0.0)
    cap = shared().get('cap_board_edge_clearance')
    check('checked but ZERO -> None (an empty override is UNSET)',
          cap is None, repr(cap))
    check('...so the margin does not fall below the placement default',
          resolve_cap_edge_clearance(board, cap)[0] == CAP_EDGE_CLEARANCE)

    dlg.Destroy()
    print()
    if failures:
        print(f"VERDICT: {len(failures)} FAILURE(S): {', '.join(failures)}")
        return 1
    print("VERDICT: cap edge margin reaches the engine from the real dialog")
    return 0


if __name__ == '__main__':
    sys.exit(main())
