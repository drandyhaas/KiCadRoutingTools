"""The #1067 mutation battery: place_fanout_clearance holds the decap limits.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore a defect somebody measured:

  * `cost-skips-gate` -- run 34: C63 moved from 2.00 to 2.96 mm from its
    +3V3 ball U30.C10 while the tool logged "0 unresolved";
  * `gui-drops-the-intent` -- the GUI is a second front: a plan step's
    `cap_intent_path` must reach the same engine call.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed. The GUI rows' witnesses need KiCad's
python (they re-exec into it, and exit 2 when it is absent, so a missing
KiCad reads as REFUSING, not as killed).

    python3 tests/mutate_1067.py
    python3 tests/mutate_1067.py --row cost-skips-gate

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`.

Not covered by a row: `hard_blocked`'s gate clause (`hard_blocked` has no
production caller; it mirrors `cost()` for tests) and the proximity filter
(`FANOUT_TETHER_RULES`), which no fixture intent declares.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
_PP = os.path.join(_ROOT, 'py_placer')
_PLUGIN = os.path.join(_ROOT, 'kicad_routing_plugin')

TARGETS = {
    'fc': os.path.join(_PP, 'placement', 'fanout_clearance.py'),
    'quench': os.path.join(_PP, 'placement', 'quench.py'),
    'cli': os.path.join(_PP, 'place_fanout_clearance.py'),
    'gui': os.path.join(_PLUGIN, 'fanout_gui.py'),
    'settings': os.path.join(_PLUGIN, 'settings_persistence.py'),
    'man': os.path.join(_TESTS, 'stress', 'manifest_to_plan.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T = 'test_1067_fanout_clearance_decap.py'
ON = _t(T, 'on_holds_every_claim')
ARMS = _t(T, 'better_arm_is_kept')
GRAZE = _t(T, 'cap_left_grazing')
OFF = _t(T, 'off_still_breaks')
QGATE = _t(T, 'gate_is_the_quench')
CACHE = _t(T, 'invalidates_the_gate')
BAD = _t(T, 'unreadable_intent')
NONE = _t(T, 'no_intent_changes')
GUI = _t(os.path.join('gui_parity', 'test_1067_cap_intent_gui.py'))
GUI772 = _t(os.path.join('gui_parity', 'test_772_cap_params_reach_engine.py'))
AIPLAN = _t('test_772_ai_plan_cap_params.py')

# (name, target, old, new, tests, expect)
ROWS = [
    ('cost-skips-gate', 'fc',
     "        if self._tether_refuses(ref, x, y, rot):",
     "        if False:",
     (ON,), 'KILLED'),
    ('gate-never-armed', 'fc',
     "        st._tethers = view",
     "        pass",
     (ON,), 'KILLED'),
    ('cache-not-invalidated', 'fc',
     "            self._tethers.note_move()",
     "            pass",
     (CACHE,), 'KILLED'),
    ('ladder-never-falls-back', 'fc',
     "                if best is None and gate_refused:",
     "                if False:",
     (ON,), 'KILLED'),
    ('broken-not-recorded', 'fc',
     "                        st.tether_broken[ref] = _tg.tether_failures(",
     "                        _unused = _tg.tether_failures(",
     (ON,), 'KILLED'),
    ('disclosed-without-intent', 'fc',
     "    if decap_report is not None:",
     "    if True:",
     (NONE, OFF), 'KILLED'),
    ('ungated-arm-never-kept', 'fc',
     "    kept = 'ungated' if kf < kg else 'gated'",
     "    kept = 'gated'",
     (ARMS,), 'KILLED'),
    ('arms-compared-by-count-only', 'fc',
     "    kg = (len(gated['unresolved']), len(worse_g))",
     "    kg = (len(gated['unresolved']), 0)",
     (ARMS,), 'KILLED'),
    ('view-not-bound', 'quench',
     "    _tether_measure = QuenchState._tether_measure",
     "    _tether_measure = staticmethod(lambda *a, **k: 0.0)",
     (QGATE,), 'KILLED'),
    ('cli-drops-the-intent', 'cli',
     "        intent=intent,",
     "        intent=None,",
     (ON,), 'KILLED'),
    ('unreadable-intent-runs-anyway', 'cli',
     "    if _rc:",
     "    if False:",
     (BAD,), 'KILLED'),
    ('json-summary-dropped', 'cli',
     "        print(\"JSON_SUMMARY: \" + json.dumps({",
     "        print(\"NO_SUMMARY: \" + json.dumps({",
     (ON,), 'KILLED'),
    ('gui-drops-the-intent', 'gui',
     "                intent=_cap_intent,",
     "                intent=None,",
     (GUI,), 'KILLED'),
    ('gui-unloadable-runs-ungated', 'gui',
     "                    _cap_intent = _fp1067.load_intent(_ip)",
     "                    _cap_intent = None",
     (GUI,), 'KILLED'),
    ('gui-summary-drops-broken', 'gui',
     "    if decap_broken:",
     "    if False:",
     (GUI,), 'KILLED'),
    # ('gui-refusal-still-clamps' dropped on ipc-migration: it mutates the
    # SWIG standalone cap path's `_refused` gate on its net-class clamp and
    # zone refill, and the IPC path has neither -- the clamp is a recorded
    # known gap there and KiCad refills server-side -- so there is no gate to
    # mutate. A refused step writing nothing is graded by test_1067_cap_intent_gui
    # arm 1b, which compares the board and project bytes.)
    ('cli-reads-a-bad-limit-late', 'cli',
     "            _fp1067.tether_gate_spec(intent)",
     "            pass",
     (BAD,), 'KILLED'),
    ('intent-not-in-defaults', 'gui',
     "        ('cap_intent_path', ''),",
     "",
     (AIPLAN,), 'KILLED'),
    ('get-config-drops-intent', 'gui',
     "            'cap_intent_path': self.cap_intent_path.GetValue().strip(),",
     "",
     (GUI772,), 'KILLED'),
    ('settings-save-dropped', 'settings',
     "        'fanout_bga_cap_intent_path': dialog.fanout_tab.bga_options.cap_intent_path.GetValue(),  # #1067",
     "",
     (GUI,), 'KILLED'),
    ('compared-only-on-a-broken-claim', 'fc',
     "    if ((not rep.get('broken') and not gated.get('unresolved'))",
     "    if ((not rep.get('broken'))",
     (GRAZE,), 'KILLED'),
    ('discarded-run-printed-too', 'fc',
     "    print(buf.getvalue(), end='')",
     "    print(gated_out + buf.getvalue(), end='')",
     (GRAZE,), 'KILLED'),
    ('manifest-intent-not-absolute', 'man',
     "            if (CAP_FLAG_PARAMS[a] == 'cap_intent_path' and cwd",
     "            if (False and cwd",
     (AIPLAN,), 'KILLED'),
    ('manifest-drops-intent', 'man',
     "    '--intent': 'cap_intent_path',",
     "",
     (AIPLAN,), 'KILLED'),
]

sys.path.insert(0, _TESTS)
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _run_tests(tests):
    failed = []
    for t in tests:
        p = subprocess.run([sys.executable, '-X', 'utf8', t[0]] + list(t[1:]),
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT)
        if p.returncode != 0:
            failed.append((os.path.basename(t[0]) + ':' + ','.join(t[1:]),
                           p.returncode,
                           [ln.strip()[:90] for ln in
                            ((p.stdout or '') + (p.stderr or '')).splitlines()
                            if 'FAIL' in ln or 'Error' in ln][:2]))
    return failed


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2
    # THE UNMUTATED BASELINE: every witness must pass as the code stands,
    # or a row it "kills" proves nothing.
    witnesses = sorted({t for r in rows for t in r[4]})
    base_fail = _run_tests(witnesses)
    if base_fail:
        print('REFUSING: witnesses fail UNMUTATED -- %s' % base_fail)
        return 2
    print('baseline: %d witnesses pass unmutated' % len(witnesses))
    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            o, n = old, new
            if '\r\n' in base:
                o, n = o.replace('\n', '\r\n'), n.replace('\n', '\r\n')
            if base.count(o) != 1 or o == n:
                results.append((name, 'BROKEN', expect,
                                ['anchor matched %d times' % base.count(o)]))
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(o, n, 1))
            try:
                failed = _run_tests(tests)
            finally:
                io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if failed else 'SURVIVED',
                            expect, [str(f)[:150] for f in failed[:2]]))
            print('%-36s %s' % (name, results[-1][1]), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
    wrong = [r for r in results if r[1] != r[2]]
    print('')
    for name, verdict, expect, why in results:
        print('%-36s %-9s%s' % (name, verdict, '' if verdict == expect else
                                '   <-- WRONG, expected %s' % expect))
        for w in why:
            print('      %s' % w)
    print('\n%d rows: %d killed, %d survived, %d broken'
          % (len(results), sum(r[1] == 'KILLED' for r in results),
             sum(r[1] == 'SURVIVED' for r in results),
             sum(r[1] == 'BROKEN' for r in results)))
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', default=None, help='run only this row')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
