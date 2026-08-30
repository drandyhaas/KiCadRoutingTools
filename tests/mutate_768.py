#!/usr/bin/env python3
"""The #768/#769 mutation battery, shipped so its numbers can be re-derived.

`tests/test_768_cap_clearance_ceiling.py` records what each arm kills. A count
is only checkable if the exact source edit is written down -- two reviewers of
the #746 branch reconstructed its rows from their names and both got the wrong
answer, because a plausible-looking reconstruction of one row was semantically
inert. So the edits live here, as data, next to the numbers they produced.

Every row carries an EXPECTATION. Some mutations are deliberately inert, and an
inert row recorded as an expected survivor is a finding, while an inert row
quietly deleted is a hole.

THERE IS NO EXPECTED SURVIVOR IN THIS TABLE TODAY, and saying so is the
point of the paragraph. It named `writeback-spends-the-flag-not-the-priced-
value` -- a row that is not in ROWS and was already absent at the #768
branch tip that shipped this prose. So the file described a hole it did not
have, which is the same failure as an inert row deleted, one level up:
prose about the table rotting instead of the table. If a row here ever does
go inert, record it as SURVIVED with its reason and name it here again.

SEVEN targets, four more than any previous copy: the change spans the model
(`legality.py`), the engine (`fanout_clearance.py`), the CLI main
(`place_fanout_clearance.py`), the GUI call site (`fanout_gui.py`), the tab's
shared params (`swig_gui.py`), the plan executor (`ai_plan.py`) and the manifest
converter. That spread is the point -- CLAUDE.md's CLI/GUI rule says a fix to
one front is not automatically a fix to the other, and a parity review proved it
here the hard way: the first cut of the GUI gate read a checkbox that clamps no
net class at all, and TWO string-count assertions passed it. The GUI rows now
name the real-dialog gate, which captures the value the engine is handed.

This is the FOURTH copy of this runner (`mutate_730.py`, `mutate_750.py`,
`mutate_756.py`, `mutate_761.py`). It is not refactored into a shared one
deliberately: that would rewrite four shipped batteries whose recorded counts
are the evidence for four merged reviews, which is a change to make on its own,
not inside a fix.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. The file refuses to start on a
dirty target, because restoring would write the COMMITTED text back over
uncommitted work.

    python3 tests/mutate_768.py
    python3 tests/mutate_768.py --row cap-is-assignment-not-min

A row is KILLED by a FAILURE **or an ERROR**: several of these mutations make an
arm raise rather than fail (inverting the `ceiling is not None` guard reaches
`min(v, None)`), and a battery that counted only failures would call that a
survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor.

Python `str.replace(old, new, 1)`, never `sed`: commit `bb8f4477` records two
rows of `mutate_761` left a `SyntaxError` behind because `sed` ate an unescaped
metacharacter, and a battery that cannot start reports nothing at all.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

LEG = os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py')
FC = os.path.join(_ROOT, 'py_placer', 'placement', 'fanout_clearance.py')
CLI = os.path.join(_ROOT, 'py_placer', 'place_fanout_clearance.py')
GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'fanout_gui.py')
SWIG = os.path.join(_ROOT, 'kicad_routing_plugin', 'swig_gui.py')
PLAN = os.path.join(_ROOT, 'kicad_routing_plugin', 'ai_plan.py')
MAN = os.path.join(_ROOT, 'tests', 'stress', 'manifest_to_plan.py')
TARGETS = {'leg': LEG, 'fc': FC, 'cli': CLI, 'gui': GUI, 'swig': SWIG,
           'plan': PLAN, 'man': MAN}

T768 = os.path.join(_TESTS, 'test_768_cap_clearance_ceiling.py')
# The real headless dialog. Not a unittest -- it exits non-zero on failure,
# which is all the runner's kill rule needs. Rows touching the GUI name it,
# because the wx-free half deliberately cannot decide a VALUE.
TDLG = os.path.join(_TESTS, 'gui_parity',
                    'test_768_cap_ceiling_real_dialog.py')
T697 = os.path.join(_TESTS, 'test_697_placement_pad_clearance.py')
# #780: the only gate that drives an optimize_caps step END TO END, from
# the plan through the real dialog to the engine kwargs. It is what still
# kills the standalone row once the two source greps are made specific.
T772 = os.path.join(_TESTS, 'gui_parity',
                   'test_772_cap_params_reach_engine.py')
T725 = os.path.join(_TESTS, 'test_725_fanout_clearance_pad_floors.py')

# The disclosure block, quoted once because three rows touch it.
_DISCLOSE = """                    if _capped:
                        notes.append(
                            'net classes capped at the %gmm --clearance '
                            'ceiling: %s' % (ceiling, ', '.join(
                                '%g -> %g (%d net%s)'
                                % (_v, ceiling, _c, '' if _c == 1 else 's')
                                for _v, _c in sorted(_capped.items()))))
"""

_HOLE_KW = "                hole_clearance=_hc,\n"

_GUI_CEILING = ("                netclass_ceiling="
                "fanout_config.get('clearance_ceiling'),\n")

# RE-ANCHORED (#780). The predicate grew a second shape -- a FANOUT step
# that switches the inline cap pass on runs the same pass -- so the old
# five-line quote matched ZERO times and the row reported BROKEN. Quote
# the guard as it now stands; deleting it is still the mutation.
_PLAN_UNTICK = """            _p = step.get("params") or {}
            _runs_caps = (step["action"] == "optimize_caps"
                          or (step["action"] == "fanout"
                              and _p.get("optimize_caps")))
            if _runs_caps and not _p.get("clearance"):
                _cc = getattr(self.dialog, 'clearance_check', None)
                if _cc is not None and _cc.GetValue():
                    _cc.SetValue(False)
"""

ROWS = [
    # ---- the cap itself, in for_board's netclass admission map ----------
    ('cap-is-assignment-not-min', 'leg',
     '                    by_name = {n: min(v, ceiling) for n, v in by_name.items()}',
     '                    by_name = {n: ceiling for n, v in by_name.items()}',
     (T768,), 'KILLED'),
    ('cap-is-max-not-min', 'leg',
     '                    by_name = {n: min(v, ceiling) for n, v in by_name.items()}',
     '                    by_name = {n: max(v, ceiling) for n, v in by_name.items()}',
     (T768,), 'KILLED'),
    ('cap-removed', 'leg',
     '                    by_name = {n: min(v, ceiling) for n, v in by_name.items()}',
     '                    pass',
     (T768,), 'KILLED'),
    # Inverting the guard, NOT deleting it: deleting it leaves `ceiling` unbound
    # in the None case only by luck of the comparison order. Inverted, the
    # uncapped path takes the capped branch and reaches `min(v, None)`, which
    # is the ERROR half of the kill rule.
    ('cap-guard-inverted', 'leg',
     '                if ceiling is not None:',
     '                if ceiling is None:',
     (T768, T697), 'KILLED'),
    ('cap-not-disclosed', 'leg',
     _DISCLOSE, '',
     (T768,), 'KILLED'),
    ('ceiling-not-recorded-on-the-model', 'leg',
     '                    has_overrides=has_overrides, ceiling=ceiling)',
     '                    has_overrides=has_overrides, ceiling=None)',
     (T768,), 'KILLED'),

    # ---- resolve_pair_clearance: the two documented branches ------------
    ('omitted-ignores-the-board', 'fc',
     "            return float(declared), 'board netclass'",
     "            return float(_defaults.CLEARANCE), 'board netclass'",
     (T768,), 'KILLED'),
    ('base-ignores-the-declaration', 'fc',
     "        return min(float(declared), float(clearance)), 'cli'",
     "        return float(clearance), 'cli'",
     (T768,), 'KILLED'),
    ('zero-declaration-honoured', 'fc',
     '    if declared is not None and declared <= 0:',
     '    if False:',
     (T768,), 'KILLED'),

    # ---- the layer axis --------------------------------------------------
    ('layer-fallback-dropped', 'fc',
     "    n = len([l for l in (_cu or []) if str(l).endswith('.Cu')]) or 2",
     "    n = len([l for l in (_cu or []) if str(l).endswith('.Cu')])",
     (T768,), 'KILLED'),
    ('layer-filter-dropped', 'fc',
     "    n = len([l for l in (_cu or []) if str(l).endswith('.Cu')]) or 2",
     "    n = len(_cu or []) or 2",
     (T768,), 'KILLED'),
    # MATCH THE GRADER. check_drc does not fab-floor copper clearance, so a
    # pass that raised to it would refuse landings its own checker passes.
    ('sub-fab-clamped', 'fc',
     '    if clearance < _fab_clr - 1e-9:',
     '    clearance = max(clearance, _fab_clr)\n    if False:',
     (T768,), 'KILLED'),

    # ---- the engine's resolve + disclosure -------------------------------
    ('engine-does-not-resolve', 'fc',
     '    clearance, _clr_src = resolve_pair_clearance(pcb_file, clearance)',
     "    _clr_src = 'cli'",
     (T768,), 'KILLED'),
    ('priced-not-disclosed', 'fc',
     '    print(f"  cap pair clearance: {clearance}mm ({_clr_src})")',
     '    pass',
     (T768,), 'KILLED'),
    ('engine-drops-the-ceiling', 'fc',
     '                                             ceiling=netclass_ceiling)',
     '                                             ceiling=None)',
     (T768,), 'KILLED'),

    # ---- the CLI: one rule, every exit -----------------------------------
    ('cli-drops-the-ceiling', 'cli',
     '        netclass_ceiling=args.clearance,',
     '        netclass_ceiling=None,',
     (T768,), 'KILLED'),
    ('omitted-clamps-anyway', 'cli',
     '                clamp_nondefault_netclasses=args.clearance is not None)',
     '                clamp_nondefault_netclasses=True)',
     (T768,), 'KILLED'),
    ('hole-clearance-rides-the-ceiling', 'cli',
     _HOLE_KW, '',
     (T768,), 'KILLED'),
    # #769 proper: the clamp used to live only in the branch that moved a cap.
    ('769-copy-branch-unwritten', 'cli',
     '        print(f"Wrote {args.output_file} (unchanged copy)")\n'
     '        _write_drc_floors()\n',
     '        print(f"Wrote {args.output_file} (unchanged copy)")\n',
     (T768,), 'KILLED'),
    # This row REPLACES an expected survivor, and the replacement is the
    # finding. The shipped code used to pass `_priced` (= min(Default, ceiling))
    # and the battery's row for it SURVIVED, correctly: on every board it could
    # reach, Default >= ceiling and the two are equal. An adversarial review
    # found the case that is not -- a class BETWEEN the Default and the ceiling
    # is priced at the ceiling and was shipped at the Default. The writeback is
    # lower-only, so passing the CEILING leaves each class at min(its own,
    # ceiling), which is what was priced, per class.
    ('writeback-clamps-to-the-base-not-the-ceiling', 'cli',
     '            _target = args.clearance if args.clearance is not None else _priced',
     '            _target = _priced',
     (T768,), 'KILLED'),
    ('hole-clearance-invented-from-the-ceiling', 'cli',
     '''            if _hc is None:
                try:
                    _hc = resolve_npth_floor(pcb_data, args.input_file)
                except Exception:                              # noqa: BLE001
                    _hc = None
''',
     '',
     (T768,), 'KILLED'),
    ('nan-ceiling-accepted', 'cli',
     '    if args.clearance is not None and not math.isfinite(args.clearance):',
     '    if False:',
     (T768,), 'KILLED'),

    # ---- the GUI half (CLAUDE.md: a CLI fix is not a GUI fix) ------------
    ('gui-no-ceiling', 'gui',
     _GUI_CEILING, '',
     (T768, TDLG), 'KILLED'),
    # THE DEFECT THE FIRST CUT SHIPPED. `fix_drc_settings` clamps no net class
    # at all -- update_live_drc_floors writes m_MinClearance and the Default
    # class only -- so gated there the GUI prices on the GIVEN branch and writes
    # back on the OMITTED one. Found by a parity review measuring the value,
    # after two string-count assertions passed it.
    ('gui-gate-is-fix-drc-settings', 'gui',
     "netclass_ceiling=fanout_config.get('clearance_ceiling'),",
     "netclass_ceiling=(fanout_config.get('clearance')\n"
     "                                  if fanout_config.get("
     "'fix_drc_settings', True) else None),",
     (T768, TDLG), 'KILLED'),
    # THE SECOND CUT'S DEFECT, found by an adversarial review: the RESOLVED base
    # is already min(Default class, override), so handing it over as a ceiling
    # caps a class BETWEEN the two down to the Default. Measured 22 cap
    # clearance violations against main's 2 at the default dialog config.
    ('gui-ceiling-is-the-resolved-base', 'gui',
     "netclass_ceiling=fanout_config.get('clearance_ceiling'),",
     "netclass_ceiling=fanout_config.get('clearance'),",
     (TDLG,), 'KILLED'),
    # RE-ANCHORED AND RE-GRADED (#780). The inline cap config gained the
    # same pair of lines at a DEEPER indent, and 12 spaces + the text is a
    # substring of 16 spaces + the text -- so the bare line matched TWICE
    # and the runner reported BROKEN. Anchor on the standalone pair's
    # NEIGHBOUR instead, which the inline copy separates with a comment.
    #
    # T768 and TDLG were also both VACUOUS against this row until #780:
    # T768's arm stripped the indentation, so the inline copy satisfied it,
    # and TDLG's standalone section assembled its own cfg rather than
    # driving run_cap_optimization. Both are fixed in the same commit
    # range; T772 is added because it drives the step end to end.
    ('gui-standalone-cfg-drops-the-key', 'gui',
     "            'clearance_ceiling': shared.get('clearance_ceiling'),\n"
     "            'fix_drc_settings': shared.get('fix_drc_settings', True),\n",
     "            'fix_drc_settings': shared.get('fix_drc_settings', True),\n",
     (T768, TDLG, T772), 'KILLED'),
    # #780's own half of the same defect, on the INLINE dict. Its only
    # behavioural killer is TDLG's section 2, which drives _run_bga_fanout.
    ('gui-inline-cfg-drops-the-key', 'gui',
     "                'clearance_ceiling': shared.get('clearance_ceiling'),\n",
     '',
     (TDLG,), 'KILLED'),
    ('fanout-tab-exports-the-RESOLVED-clearance-as-the-ceiling', 'swig',
     "                'clearance_ceiling': (self.clearance.GetValue()",
     "                'clearance_ceiling': (self._effective_clearance()",
     (T768, TDLG), 'KILLED'),
    # Anchored on the #768 comment above the row, not the row itself:
    # `clamp_netclasses` is exported by the signal, planes and now fanout
    # tabs, so the bare line matched TWICE and the runner reported BROKEN.
    # That is the runner behaving exactly as designed.
    ('fanout-tab-stops-exporting-the-override', 'swig',
     "                # what the pass must price at.\n"
     "                'clamp_netclasses': self.clearance_check.GetValue(),\n",
     '',
     (T768, TDLG), 'KILLED'),
    # #768's plan-executor half: an OMITTED --clearance must not inherit the
    # preceding fanout step's override and become a GIVEN one.
    ('plan-inherits-the-given-branch', 'plan',
     _PLAN_UNTICK, '',
     (TDLG,), 'KILLED'),

    # ---- the recorded-manifest round trip --------------------------------
    # Anchored on the comment, not the row: `'--clearance': 'clearance',` is
    # ALSO FLAG_PARAMS' own row, so the bare string matched twice and the
    # battery reported BROKEN -- which is the runner behaving correctly.
    ('manifest-drops-clearance', 'man',
     "    # one parameter whose two branches decide whether the step clamps the\n"
     "    # project at all.\n"
     "    '--clearance': 'clearance',\n",
     "    # one parameter whose two branches decide whether the step clamps the\n"
     "    # project at all.\n",
     (T768,), 'KILLED'),
]


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


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

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            edits = old if isinstance(old, list) else [(old, new)]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                continue
            mutated = base
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(path, 'w', encoding='utf-8', newline='').write(mutated)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=2400, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += ['%s::%s' % (os.path.basename(t)[5:8],
                                       l.split('(')[0].replace('FAIL: ', '')
                                       .replace('ERROR: ', '').strip())
                           for l in out.splitlines()
                           if l.startswith(('FAIL:', 'ERROR:'))]
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])

    w = max(len(r[0]) for r in results)
    wrong = 0
    for name, verdict, expect, cnt, failed in results:
        mark = ''
        if verdict != expect:
            mark = '   <-- WRONG, expected %s' % expect
            wrong += 1
        print('%-*s  %-9s  %-3s%s' % (w, name, verdict, cnt, mark))
        for f in failed:
            print('%s      %s' % (' ' * w, f))
    killed = sum(1 for r in results if r[1] == 'KILLED')
    survived = sum(1 for r in results if r[1] == 'SURVIVED')
    broken = sum(1 for r in results if r[1] == 'BROKEN')
    print('\n%d rows: %d killed, %d survived (%d of them expected), %d broken'
          % (len(results), killed, survived,
             sum(1 for r in results if r[1] == r[2] == 'SURVIVED'), broken))
    if wrong or broken:
        print('%d row(s) did not match their expectation' % (wrong + broken))
    return 1 if (wrong or broken) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', help='run a single row by name')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
