#!/usr/bin/env python3
"""The #946 mutation battery, shipped so its numbers can be re-derived.

One row per kept phase of the render design system, each reverting that phase's
single load-bearing line. A count is only checkable if the exact source edit is
written down: two reviewers of an earlier branch reconstructed a battery's rows
from their names and both got the wrong answer, because a plausible-looking
reconstruction of one row was semantically inert. So the edits live here, as
data, next to the numbers they produced.

Every row carries an EXPECTATION. Some mutations are deliberately inert, and an
inert row recorded as an expected survivor is a finding, while an inert row
quietly deleted is a hole. A row whose verdict does not match its expectation is
reported as WRONG.

**THE ROWS TO LOOK AT FIRST if this file ever goes red** are the ones that
restore the defect #946 opened on:

  * `restore-back-to-the-red-green-axis` -- the rip/restore pair back to 76
    deuteranope separation, which is two thirds of the separation gone and what
    is left is lightness;
  * `rip-loses-its-second-channel` -- colour alone again, at 1-2 px for two
    frames;
  * `defect-back-in-the-red-family` -- a pad/hole conflict 2.8 from the rip,
    indistinguishable and opposite in kind;
  * `crossings-blend-again` -- the 19 two-layer crossings that land within 34
    of some third layer's solo appearance.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
target, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 tests/mutate_946.py
    python3 tests/mutate_946.py --row rip-loses-its-second-channel

A row is KILLED by a FAILURE **or an ERROR**: several of these make a renderer
raise rather than merely draw the wrong thing, and a battery that counted only
failures would call that a survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor. `preflight()` runs right after `ROWS` so that costs one second rather
than a whole battery's witnesses (#877).

Edits are applied with `str.replace(old, new, 1)`, never `sed`: a shipped
battery once left two rows as a SyntaxError from unescaped `sed` metacharacters,
and a battery that cannot start reports nothing at all.

Anchors are written with LF and translated to the target's own ending. Every
target is LF in git (`.gitattributes` has `*.py text eol=lf`), so from a clean
checkout the translation is a no-op; it is here because a WORKING TREE can
still be CRLF.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

TARGETS = {
    'theme': os.path.join(_ROOT, 'py_router', 'render_theme.py'),
    'render': os.path.join(_ROOT, 'py_router', 'route_render.py'),
    'plan': os.path.join(_ROOT, 'py_router', 'render_plan.py'),
}


def _t(name):
    return os.path.join(_TESTS, name)


T_PAL = _t('test_946_palette_measures.py')
T_THEME = _t('test_946_theme_completeness.py')
T_XING = _t('test_946_opaque_crossings.py')
T_PLAN = _t('test_946_plan_overlay.py')

# (name, target, old, new, tests, expect)
ROWS = [
    # --- the palettes -----------------------------------------------------
    # The defect #946 opened on: a defect colour 2.8 from the rip, so red means
    # "copper was removed" and "this board is wrong" at the same time.
    ('defect-back-in-the-red-family', 'theme',
     "    'defect_conflict':    (255, 140, 0),",
     "    'defect_conflict':    (255, 64, 64),",
     (T_PAL,), 'KILLED'),

    ('light-layer-palette-back-to-the-dark-one', 'theme',
     "LIGHT = Theme('light', _LIGHT, _DARK_MARKS, _LIGHT_LAYERS, 205)",
     "LIGHT = Theme('light', _LIGHT, _DARK_MARKS, _LAYERS, 150)",
     (T_PAL,), 'KILLED'),

    # --- cyan, and the dash ------------------------------------------------
    ('restore-back-to-the-red-green-axis', 'theme',
     "    'event_restored':     (80, 215, 230),",
     "    'event_restored':     (86, 224, 96),",
     (T_PAL,), 'KILLED'),

    ('rip-loses-its-second-channel', 'theme',
     "    'event_ripped': 'dashed',",
     "    'event_ripped': 'solid',",
     (T_PAL,), 'KILLED'),

    # --- the theme is COMPLETE and ordered ---------------------------------
    ('a-role-vanishes-from-the-light-theme', 'theme',
     "    'event_restored':        (0, 60, 150),\n",
     "",
     (T_THEME, T_PAL), 'KILLED'),

    # --- crossings are opaque ----------------------------------------------
    ('crossings-blend-again', 'render',
     "        self.opaque_crossings = True",
     "        self.opaque_crossings = False",
     (T_XING,), 'KILLED'),

    # --- the declared plan -------------------------------------------------
    # The FULL diagonals, which is what the first version of this drew. The
    # clip is implemented in three cooperating places -- the loop bound, the
    # 45-degree adjust-and-skip, and the clamp on the line itself -- and
    # removing any ONE of them changes nothing, which two attempts at this row
    # measured. So the row removes the clip entirely, which is the change that
    # restores the defect: a prohibition drawn bigger than it was declared.
    ('keepout-hatch-escapes-its-own-box', 'plan',
     "            for i in range(int(y0 - (x1 - x0)), int(y1), step):\n"
     "                ax, ay, bx, by = x0, i + (x1 - x0), x1, i\n"
     "                # clamp the segment to y in [y0, y1] along its "
     "45-degree slope\n"
     "                if ay > y1:\n"
     "                    ax += (ay - y1); ay = y1\n"
     "                if by < y0:\n"
     "                    bx -= (y0 - by); by = y0\n"
     "                if ax > bx or ay < y0 or by > y1:\n"
     "                    continue\n"
     "                d.line([max(x0, ax), min(y1, ay), min(x1, bx), "
     "max(y0, by)],\n"
     "                       fill=keep_c, width=1)",
     "            for i in range(int(y0 - (x1 - x0)), int(y1), step):\n"
     "                d.line([x0, i + (x1 - x0), x1, i], fill=keep_c, "
     "width=1)",
     (T_PLAN,), 'KILLED'),
]

# Every anchor must match its target exactly once BEFORE anything is rewritten.
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


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
            if '\r\n' in base:
                edits = [(o.replace('\n', '\r\n'), n.replace('\n', '\r\n'))
                         for o, n in edits]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits) or old == new:
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times%s'
                                % (counts, ', and the edit is a no-op'
                                   if old == new else ''), []))
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
                                   timeout=1800, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += ['%s::%s' % (os.path.basename(t)[9:17],
                                       l.split('(')[0].replace('FAIL: ', '')
                                       .replace('ERROR: ', '').strip()[:70])
                           for l in out.splitlines()
                           if l.strip().startswith(('FAIL:', 'ERROR:'))]
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:3]))
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
    if wrong:
        print('%d row(s) did not match their expectation' % wrong)
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', help='run a single row by name')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
