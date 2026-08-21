#!/usr/bin/env python3
"""Run a GUI routing plan with no GUI: the REAL plugin dialog, headless (#507).

A plan JSON (what the AI tab's Save.../Load... buttons write and read, and
what ``make_plan.py`` converts a recorded manifest into) describes a routing
chain as steps. This module executes one exactly the way a person would --

    open the board -> AI tab -> Load... -> Run All Selected Steps

-- but with no window and no clicks: ``swig_gui.RoutingDialog`` is constructed
with parent None and never shown, and the real ``ai_plan.PlanExecutor``
drives its tabs inside a wx MainLoop. Nothing is re-implemented or mirrored, so
what runs here is what the buttons run.

It therefore needs a Python with **pcbnew and wx** -- KiCad's bundled one.
``reexec_into_kicad()`` finds it and re-execs, so a plain ``python3 run_plan.py``
still works.

Consumers: ``run_plan.py`` (the CLI) and
``tests/gui_parity/replay_plan_vs_run.py`` (the parity harness). Keeping one
driver means the harness that proves GUI/CLI parity exercises the same code the
user-facing tool runs.
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

# wx debug builds assert on about_tab's wxEXPAND|wxALIGN_* sizer flags, which is
# harmless in the real plugin but fatal when constructing the dialog headless.
# Must be set before wx is imported (and before any re-exec, so a child sees it).
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
# kicad_routing_plugin lives at the REPO root, one level up (#522 layout)
_REPO_DIR = os.path.abspath(os.path.join(ROOT_DIR, '..'))
if _REPO_DIR not in sys.path:
    sys.path.insert(0, _REPO_DIR)

# Where KiCad's bundled python (the one with pcbnew + wx) lives per platform.
# Shared with kicad_exact_fill so the install layouts -- notably Windows'
# VERSIONED directory, which the bare C:\Program Files\KiCad\bin path missed
# for every KiCad >= 6 (#647) -- are described in exactly one place.
from kicad_exact_fill import kicad_python_candidates  # noqa: E402

KICAD_PYTHONS = kicad_python_candidates()


def have_kicad_python() -> bool:
    """True when THIS interpreter can drive the plugin (pcbnew + wx present)."""
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
        return True
    except Exception:
        return False


def reexec_into_kicad(script=None, argv=None):
    """Re-exec the current script under a python that has pcbnew + wx.

    Returns (never returns on success) only if no such interpreter is found.
    """
    script = os.path.abspath(script or sys.argv[0])
    argv = list(sys.argv[1:] if argv is None else argv)
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            child = [cand, script] + argv
            if os.name == 'nt':
                # os.execv goes through the CRT on Windows, which re-splits the
                # argument vector on spaces: "C:\Program Files\KiCad\10.0\bin\
                # python.exe" arrives torn in two and the run dies with a bogus
                # "can't open file ...\Files\KiCad\...". subprocess quotes it.
                sys.exit(subprocess.run(child).returncode)
            os.execv(cand, child)
    return False


def load_plan(path):
    """(steps, messages) from a plan JSON file -- the same parse the AI
    tab's Load... button uses, so a file that loads there loads here."""
    from kicad_routing_plugin.ai_plan import parse_plan_result
    with open(path, encoding='utf-8') as f:
        text = f.read()
    return parse_plan_result(text)


def step_labels(steps):
    from kicad_routing_plugin.ai_plan import step_label
    return [step_label(i + 1, s) for i, s in enumerate(steps)]


SNAPSHOT_FORMAT = '{prefix}{n:02d}_{action}.kicad_pcb'


def run_plan(board_path, steps, indices=None, snapshot_dir=None,
             snapshot_prefix='step', snapshot_format=SNAPSHOT_FORMAT,
             timeout=7200, on_status=None, quiet=True, save=True):
    """Execute ``steps`` against ``board_path`` through the real plugin.

    The board is routed IN PLACE (load -> run -> save back to ``board_path``),
    so callers copy their input first -- with its siblings, since the
    ``.kicad_pro`` carries the DRC floor (#441).

    Args:
        indices: which steps to run (default: all, in order).
        snapshot_dir: save the board after each completed step as
            ``<prefix>NN_<action>.kicad_pcb`` here -- the chain a routing movie
            animates (``make_movie.py``).
        snapshot_format: override that naming ('{prefix}', '{n}' = 1-based step,
            '{action}'); the parity harness pins its own names.
        timeout: seconds before the run is abandoned (the plan cannot block on a
            dialog: modal popups are stubbed out, but a wedged step would
            otherwise hang forever).
        on_status: callback(index, 'running'|'done'|'failed'|'stopped').

    Returns a dict: board, steps (per-step status + seconds), completed,
    aborted, timed_out, snapshots, log, seconds.
    """
    import wx
    import pcbnew

    board_path = os.path.abspath(board_path)
    indices = list(range(len(steps))) if indices is None else list(indices)

    app = wx.App(False)
    # No user is present: a modal popup would hang the run forever. quiet=True
    # already suppresses the per-step completion popups; this covers the rest
    # (validation warnings, the plane-offer prompt's cousins).
    wx.MessageBox = lambda *a, **k: wx.OK

    class _NoModal:
        def __init__(self, *a, **k):
            pass

        def ShowModal(self):
            return wx.ID_OK

        def Destroy(self):
            pass

        def __getattr__(self, _n):
            return lambda *a, **k: None

    wx.MessageDialog = _NoModal

    board = pcbnew.LoadBoard(board_path)
    pcbnew.GetBoard = lambda: board   # the plugin reads the "live" board through this

    # LoadBoard's net propagation reassigns netcodes of copper touching STALE
    # zone fills (a mid-chain input: pours poured before the copper existed).
    # Restore the file's stored nets, then refill so the fills carry real
    # knockouts and later connectivity rebuilds cannot flip them again.
    try:
        from kicad_exact_fill import repin_netcodes_from_file
        _n_repin = repin_netcodes_from_file(board, board_path)
        if _n_repin:
            print(f"run_plan: restored {_n_repin} netcode(s) the board load "
                  f"flipped over stale fills")
            from kicad_routing_plugin.gui_utils import refill_all_zones
            refill_all_zones(board)
    except Exception as _e:
        print(f"run_plan: (net repin skipped: {_e})")

    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui
    from kicad_routing_plugin.ai_plan import PlanExecutor, step_label

    pcb_data = build_pcb_data_from_board(board)
    dialog = swig_gui.RoutingDialog(None, pcb_data, board_path)

    result = {'board': board_path, 'steps': [], 'completed': 0, 'aborted': None,
              'timed_out': False, 'snapshots': [], 'log': []}
    started = {}

    if snapshot_dir:
        os.makedirs(snapshot_dir, exist_ok=True)

    def _status(index, status):
        if status == 'running':
            started[index] = time.time()
        else:
            elapsed = time.time() - started.get(index, time.time())
            result['steps'].append(
                {'index': index, 'action': steps[index].get('action'),
                 'label': step_label(index + 1, steps[index]),
                 'status': status, 'seconds': round(elapsed, 1)})
        if on_status is not None:
            on_status(index, status)
        if status == 'done' and snapshot_dir:
            action = steps[index].get('action') or 'step'
            snap = os.path.join(snapshot_dir, snapshot_format.format(
                prefix=snapshot_prefix, n=index + 1, action=action))
            try:
                # aSkipSettings: per-step copper snapshot, no .kicad_pro
                # wanted (KiCad 10's settings save merges the pre-migration
                # on-disk project JSON with its migrated in-memory view;
                # an uncaught type_error aborts the process on any
                # pre-KiCad-10 project).
                pcbnew.SaveBoard(snap, board, aSkipSettings=True)
                result['snapshots'].append(snap)
            except Exception as e:
                result['log'].append(f"snapshot step {index + 1} failed: {e}")

    def _finished(completed, reason):
        result['completed'] = completed
        result['aborted'] = reason
        if save:
            try:
                # Save back to the same path, beside the .kicad_pro that
                # PlanExecutor._write_drc_floors just stamped with the routed
                # floors -- a board without its floor grades wrong (#441).
                # aSkipSettings for two reasons: the implicit settings save
                # would REWRITE that just-stamped .kicad_pro from KiCad's
                # in-memory (pre-stamp) view, silently dropping the floors;
                # and on pre-KiCad-10 projects it aborts the process outright
                # (uncaught type_error merging the pre-migration file with
                # the migrated in-memory settings).
                pcbnew.SaveBoard(board_path, board, aSkipSettings=True)
            except Exception as e:
                result['log'].append(f"final save failed: {e}")
        app.ExitMainLoop()

    def _on_timeout():
        result['timed_out'] = True
        result['log'].append(f"plan exceeded timeout {timeout}s")
        app.ExitMainLoop()

    executor = PlanExecutor(dialog, steps, indices, _status, _finished,
                            log=lambda m: result['log'].append(m), quiet=quiet)
    t0 = time.time()
    wx.CallAfter(executor.start)
    timer = wx.CallLater(int(timeout * 1000), _on_timeout)
    app.MainLoop()
    result['seconds'] = round(time.time() - t0, 1)

    # Deterministic wx teardown. Without this the run segfaults AFTER finishing
    # -- measured 2 of 3 runs exiting 139 on a 1-step replay, and one probe run
    # died before printing its results, so a crash here can silently cost a
    # measurement rather than just look untidy.
    #
    # Two causes, both "leave it to the interpreter":
    #   * the wx.CallLater timeout timer was NEVER cancelled. On a normal finish
    #     it is still pending (default deadline 7200s) holding a callback into
    #     this frame, and wxWidgets tears it down at an arbitrary later point.
    #   * the dialog was never Destroy()ed. Its C++ side then gets finalized
    #     during interpreter shutdown, after the wx.App it belongs to is gone.
    # Either way the heap is corrupt by the time CPython's GC next walks it,
    # which is why the crash surfaces as SIGBUS/SIGSEGV inside visit_decref /
    # dict_traverse / collect with a traceback pointing at unrelated code.
    #
    # Order matters: stop the timer first (so nothing can fire into a
    # half-destroyed dialog), destroy the dialog, then let wx process the
    # pending destroy events while the App is still alive.
    try:
        if timer is not None and timer.IsRunning():
            timer.Stop()
    except Exception:
        pass
    try:
        dialog.Destroy()
    except Exception:
        pass
    try:
        app.ProcessPendingEvents()
    except Exception:
        pass
    del dialog
    del app
    return result
