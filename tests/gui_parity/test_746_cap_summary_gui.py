#!/usr/bin/env python3
"""#746: the plugin's cap-optimization summary, driven with real result dicts.

`tests/test_746_fanout_clearance_resolved_credit.py` covers the engine. The
GUI half was gated by NOTHING, and had been wrong for a long time because of
it: `fanout_gui.py` rendered `unresolved` as "could not clear a foreign via"
while `graze_penalty` has always been via (#130) + track (#278) + pad (#275).
It named one of the three channels. That was wrong before #736 and more
visibly wrong after it, which made the track channel reachable in that verdict
for the first time.

The reason nothing tested it is structural, not an oversight: the text was
built inline inside `_optimize_decoupling_caps`, which needs a live pcbnew
board, a fanned-out design and a dialog. #746 lifts it to a module-level
`cap_optimization_summary(result)` taking a plain dict, so this gate drives the
REAL function with the REAL dicts the engine returns -- no board, no dialog,
milliseconds.

The dicts below are not invented. They are the shapes measured on the two rigs
and the one tracked board in the engine test file, plus the six-key shape the
engine's two early returns carry.

Needs KiCad's python (fanout_gui imports wx at module level); re-execs into it
automatically, like its siblings.

Run: python3 tests/gui_parity/test_746_cap_summary_gui.py
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
    # Exit 0, like every sibling here -- but say plainly that the gate did NOT
    # run. CLAUDE.md: probe before recording it as not-run --
    #   <kicad>/bin/python3 -c "import pcbnew, wx; print(wx.version())"
    print("SKIP: no python with pcbnew+wx found -- THIS GATE DID NOT RUN, and "
          "nothing else covers fanout_gui.cap_optimization_summary")
    sys.exit(0)


# --- the result dicts, as MEASURED by tests/test_746_... ------------------
# arm A: a boxed cap only the #313 via-nudge could free.
ARM_A = {'placements': [], 'resolved': ['C1'], 'unresolved': [],
         'via_resolved': ['C1'], 'regrazed': [],
         'via_moves': [(11.175, 10.0, {})], 'new_segments': [{}]}
# arm B: a cap the sweep cleaned, re-grazed by the connector this pass drew.
ARM_B = {'placements': [{}, {}], 'resolved': [], 'unresolved': ['C1', 'C2'],
         'via_resolved': [], 'regrazed': ['C2'],
         'via_moves': [(10.0, 12.0, {})], 'new_segments': [{}]}
# the sweep-only path: the nudger is never called.
SWEEP = {'placements': [{}], 'resolved': ['C1'], 'unresolved': [],
         'via_resolved': [], 'regrazed': [], 'via_moves': [],
         'new_segments': []}
# orangecrab_ext_pll @0.1, fallback off -- the tracked board that reaches it.
REAL = {'placements': [{}] * 18, 'resolved': ['C19', 'C44', 'C45', 'C67'],
        'unresolved': ['R19', 'C1', 'C25', 'C7', 'C52', 'C53', 'C68', 'C55',
                       'C28', 'C59'],
        'via_resolved': ['C19', 'C44', 'C45'], 'regrazed': [],
        'via_moves': [(0, 0, {})] * 9, 'new_segments': [{}] * 17}
# what the engine's TWO early returns carry: six keys, none of the nudge four.
EARLY = {'placements': [], 'resolved': [], 'unresolved': [], 'bga_refs': [],
         'required': [], 'clearance_notes': []}


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'py_placer'))
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.path.insert(0, os.path.dirname(REPO))

    from kicad_routing_plugin.fanout_gui import cap_optimization_summary as S

    failures = []

    def check(what, ok, detail=''):
        print(('  OK   ' if ok else '  FAIL ') + what
              + ('' if ok or not detail else ' -- ' + detail))
        if not ok:
            failures.append(what)

    # ON THE GATE: the function must be reachable and must produce text, or
    # every assertion below is about an empty string and passes vacuously.
    check('cap_optimization_summary is importable without a board or dialog',
          callable(S))
    # In a try, because the most likely defect here is a KeyError on the
    # early-return shape: unguarded, that aborts the gate with a traceback
    # and no FAIL line, which is a non-zero exit that does not say WHY
    # (CLAUDE.md: assert the reason, not the exit code).
    try:
        _early = S(EARLY)
        check('it returns text for the simplest dict',
              isinstance(_early, str)
              and _early.startswith('Decoupling caps'), repr(_early))
    except Exception as e:                                       # noqa: BLE001
        check('it returns text for the simplest dict', False,
              'raised %s: %s -- a read is not defensive, and the plugin '
              'crashes on a board with no vias' % (type(e).__name__, e))

    # 1. The wording defect this issue names "while here".
    for name, d in (('armA', ARM_A), ('armB', ARM_B), ('real', REAL)):
        check('%s: the verdict no longer says "a foreign via" for a via +'
              ' track + pad grade' % name,
              'could not clear a foreign via' not in S(d), S(d))
    check('the unresolved clause names all three channels',
          'still grazing foreign copper (via/track/pad)' in S(ARM_B), S(ARM_B))

    # 2. The credit the operator could not previously see.
    check('armA: the nudge is credited by name',
          '1 cap(s) freed by that nudge' in S(ARM_A), S(ARM_A))
    check('real: three caps credited to the nudge, not to the sweep',
          '3 cap(s) freed by that nudge' in S(REAL), S(REAL))

    # 3. The cause of a re-graze is named rather than left anonymous.
    check('armB: the pass says it broke C2 itself',
          "re-grazed by this pass's own connector copper: C2" in S(ARM_B),
          S(ARM_B))
    check('armB: and C2 is still reported unresolved',
          'manual: C1, C2' in S(ARM_B), S(ARM_B))

    # 4. SUPPRESSION. This is the arm that protects the 22 tracked boards,
    # where the nudger is never called at the shipped defaults: the line must
    # be what it always was, not the new one with empty counts in it.
    check('sweep-only: the summary is exactly the pre-#746 line',
          S(SWEEP) == 'Decoupling caps optimized: 1 moved', S(SWEEP))
    for clause in ('freed by that nudge', 're-grazed', 'still grazing'):
        check('sweep-only: no %r clause is emitted' % clause,
              clause not in S(SWEEP), S(SWEEP))

    # 5. The EARLY-RETURN shape. The engine's two early returns carry neither
    # 'via_moves'/'new_segments' nor 'via_resolved'/'regrazed', so every read
    # must be a .get. A KeyError here is a crash in the plugin, on the most
    # ordinary board there is: one with no vias.
    try:
        early = S(EARLY)
        check('a board with no vias does not raise', True)
        check('...and reports a plain no-op',
              early == 'Decoupling caps optimized: 0 moved', early)
    except Exception as e:                                       # noqa: BLE001
        check('a board with no vias does not raise', False,
              '%s: %s' % (type(e).__name__, e))

    # 6. A None where a list is expected -- the engine never does this, but
    # `.get(k) or []` is what makes the reads defensive and a bare `.get(k)`
    # would pass every arm above and crash only here.
    try:
        check('None values are tolerated like absent ones',
              S({'placements': None, 'unresolved': None, 'via_moves': None,
                 'via_resolved': None, 'regrazed': None})
              == 'Decoupling caps optimized: 0 moved')
    except Exception as e:                                       # noqa: BLE001
        check('None values are tolerated like absent ones', False,
              '%s: %s' % (type(e).__name__, e))

    # 7. The method still routes through this function rather than carrying a
    # second copy of the wording -- the CLI/GUI drift shape CLAUDE.md warns
    # about, one front silently keeping its own text.
    import inspect
    from kicad_routing_plugin import fanout_gui as FG
    body = inspect.getsource(FG.FanoutTab._optimize_decoupling_caps)
    check('_optimize_decoupling_caps delegates instead of rebuilding the text',
          'cap_optimization_summary(result)' in body
          and 'Decoupling caps optimized' not in body)

    print()
    if failures:
        print(f"VERDICT: {len(failures)} FAILURE(S): {'; '.join(failures)}")
        return 1
    print("VERDICT: the plugin's cap summary names the right channels, "
          "credits the nudge, and is unchanged where nothing nudged")
    return 0


if __name__ == '__main__':
    sys.exit(main())
