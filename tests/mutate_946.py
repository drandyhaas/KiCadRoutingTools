#!/usr/bin/env python3
"""The #946 mutation battery, shipped so its numbers can be re-derived.

One row per phase of the render design system, each reverting that phase's
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
    of some third layer's solo appearance;
  * `frame-height-only-even` -- the silent pixel-column crop `_write_mp4` has
    been doing to every tall board's mp4.

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
    'anim': os.path.join(_ROOT, 'py_router', 'animate_route.py'),
    'chrome': os.path.join(_ROOT, 'py_router', 'render_chrome.py'),
    'plan': os.path.join(_ROOT, 'py_router', 'render_plan.py'),
    'layout': os.path.join(_ROOT, 'py_router', 'frame_layout.py'),
    'panels': os.path.join(_ROOT, 'py_router', 'render_panels.py'),
    'attempts': os.path.join(_ROOT, 'py_router', 'movie_attempts.py'),
    'motion': os.path.join(_ROOT, 'py_router', 'copper_motion.py'),
    'iso': os.path.join(_ROOT, 'py_router', 'movie_panels.py'),
}


def _t(name):
    return os.path.join(_TESTS, name)


T_PAL = _t('test_946_palette_measures.py')
T_THEME = _t('test_946_theme_completeness.py')
T_KEY = _t('test_946_event_key.py')
T_XING = _t('test_946_opaque_crossings.py')
T_ISO = _t('test_946_iso_gate.py')
T_PLAN = _t('test_946_plan_overlay.py')
T_LAY = _t('test_946_frame_layout.py')
T_CAP = _t('test_946_caption_split.py')
T_BOX = _t('test_946_lower_box.py')
T_ATT = _t('test_946_movie_attempts.py')
T_MOT = _t('test_946_retract_and_grow.py')
T_FMT = _t('test_946_format_defaults.py')

# (name, target, old, new, tests, expect)
ROWS = [
    # --- P2: the palettes ---------------------------------------------------
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

    # --- P3: cyan, and the dash --------------------------------------------
    ('restore-back-to-the-red-green-axis', 'theme',
     "    'event_restored':     (80, 215, 230),",
     "    'event_restored':     (86, 224, 96),",
     (T_PAL,), 'KILLED'),

    ('rip-loses-its-second-channel', 'theme',
     "    'event_ripped': 'dashed',",
     "    'event_ripped': 'solid',",
     (T_PAL, T_KEY), 'KILLED'),

    # --- P1/P2: the theme is COMPLETE and ordered ---------------------------
    ('a-role-vanishes-from-the-light-theme', 'theme',
     "    'event_restored':        (0, 60, 150),\n",
     "",
     (T_THEME, T_PAL), 'KILLED'),

    # --- P4: the key reports the RUN, not the palette -----------------------
    ('key-advertises-every-colour-whatever-happened', 'chrome',
     "    wanted = [(r, lbl) for r, lbl in EVENT_KEY if seen is None or r in seen]",
     "    wanted = list(EVENT_KEY)",
     (T_KEY,), 'KILLED'),

    ('key-reads-its-mark-from-somewhere-other-than-the-theme', 'chrome',
     "    return [(theme.rgb(role), theme.mark(role), label) for role, label in roles]",
     "    return [(theme.rgb(role), 'solid', label) for role, label in roles]",
     (T_KEY,), 'KILLED'),

    # --- P5: crossings are opaque ------------------------------------------
    ('crossings-blend-again', 'render',
     "        self.opaque_crossings = True",
     "        self.opaque_crossings = False",
     (T_XING,), 'KILLED'),

    # --- P6: the iso gate ---------------------------------------------------
    ('iso-panel-drawn-on-a-board-with-no-models', 'iso',
     "        self.require_models = bool(require_models)",
     "        self.require_models = False",
     (T_ISO,), 'KILLED'),

    # --- P7: the declared plan ---------------------------------------------
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

    # --- P8: the frame is decided ONCE -------------------------------------
    # The bug this phase found shipped: `_write_mp4` crops BOTH axes and only
    # the height was ever forced even, so every tall board's mp4 silently lost
    # a pixel column.
    ('frame-height-only-even', 'layout',
     "    W, H = even(W), even(H)",
     "    H = even(H)",
     (T_LAY,), 'KILLED'),

    ('auto-infers-a-stance-it-must-never-infer', 'layout',
     "    if a > ADAPTIVE_ASPECT_CUT:\n        return 'sidebar', ('adaptive: board aspect %.2f > %.2f'\n                           % (a, ADAPTIVE_ASPECT_CUT))",
     "    if a > ADAPTIVE_ASPECT_CUT:\n        return 'split', ('adaptive: board aspect %.2f > %.2f'\n                         % (a, ADAPTIVE_ASPECT_CUT))",
     (T_LAY,), 'KILLED'),

    ('mixed-frame-sizes-are-squashed-silently', 'anim',
     "        frame_layout.assert_frames_uniform(sorted(sizes))",
     "        return frames",
     (T_LAY,), 'KILLED'),

    # --- P9: the caption splits --------------------------------------------
    ('totals-drop-a-field-when-the-foot-is-short', 'chrome',
     "        t = _fit(d, '  -  '.join(parts), font, box.w - 2 * pad)",
     "        t = _fit(d, parts[0], font, box.w - 2 * pad)",
     (T_CAP,), 'KILLED'),

    ('the-rail-counts-steps-again', 'chrome',
     "    out = 'lap %d' % lap",
     "    return label or ''\n    out = 'lap %d' % lap",
     (T_CAP,), 'KILLED'),

    # --- P10: one box, four contents ---------------------------------------
    ('the-strip-counts-copper-that-is-not-its-own', 'panels',
     "            for s in by_layer.get(ln, ()):",
     "            for s in segments:",
     (T_BOX,), 'KILLED'),

    ('the-strip-reports-a-tally-it-did-not-draw', 'panels',
     "                drew += 1",
     "                pass",
     (T_BOX,), 'KILLED'),

    ('the-count-is-stamped-on-the-layer-name-again', 'panels',
     "            if d.textlength(num, font=font) > room:\n                num = ''",
     "            if False:\n                num = ''",
     (T_BOX,), 'KILLED'),

    ('cells-shrink-below-legibility', 'panels',
     "    if cw < CELL_MIN_W:",
     "    if False:",
     (T_BOX,), 'KILLED'),

    ('an-unplaced-board-pretends-it-was-repaired', 'panels',
     "    if unplaced:\n        return 'seeding'",
     "    pass",
     (T_BOX,), 'KILLED'),

    # --- P11: the attempts band --------------------------------------------
    ('the-axis-becomes-vias-instead-of-the-accept-rule', 'attempts',
     "    key = 'accept_score' if use_accept else 'failures'",
     "    key = 'vias'",
     (T_ATT,), 'KILLED'),

    ('a-screened-round-is-scored-zero', 'attempts',
     "            score=None if sc is None else float(sc),",
     "            score=float(sc or 0),",
     (T_ATT,), 'KILLED'),

    ('a-null-blocking-row-is-dropped', 'attempts',
     "        b = sc.get('blocking') if sc else None",
     "        b = (sc.get('blocking') if sc else None)\n        if b is None:\n            continue",
     (T_ATT,), 'KILLED'),

    ('a-rejected-attempt-can-set-the-record', 'attempts',
     "                and (a.accepted or not require_accepted)",
     "                and True",
     (T_ATT,), 'KILLED'),

    ('the-off-arm-rebuilds-the-frame-list', 'attempts',
     "        report['why'] = ('no loop_round*.json sidecars and no converge '\n                         'ledger: this chain is one attempt')\n        return frames, report",
     "        report['why'] = ('no loop_round*.json sidecars and no converge '\n                         'ledger: this chain is one attempt')\n        return list(frames), report",
     (T_ATT,), 'KILLED'),

    ('attempts-are-synthesised-from-the-boards', 'attempts',
     "    t = attempts_from_loop_dir(d)\n    if t:\n        return t",
     "    t = attempts_from_loop_dir(d)\n    if t:\n        return t\n    import glob as _g\n    _b = sorted(_g.glob(os.path.join(d, '*.kicad_pcb')))\n    if len(_b) > 1:\n        return Track(tuple(Attempt(i, b, 'round', i - 1 or None, True, False,\n                                   float(len(_b) - i), False, b)\n                           for i, b in enumerate(_b)),\n                     'boards', 'loop', 'synthesised')",
     (T_ATT,), 'KILLED'),

    # --- P12: retract and grow ---------------------------------------------
    ('copper-dissolves-instead-of-retracting', 'motion',
     "    ordered = order_from(rows, a)",
     "    ordered = [list(r) for r in rows]",
     (T_MOT,), 'KILLED'),

    ('the-last-retract-stage-still-holds-copper', 'motion',
     "        f = ((i + 1) / float(n)) if grow else (1.0 - (i + 1) / float(n))",
     "        f = ((i + 1) / float(n)) if grow else (1.0 - i / float(n))",
     (T_MOT,), 'KILLED'),

    ('rip-hold-zero-animates-anyway', 'anim',
     "        self.motion = rip_hold > 0",
     "        self.motion = True",
     (T_MOT,), 'KILLED'),

    ('every-add-grows-not-just-a-restore', 'anim',
     "            if self.motion and role == 'event_restored' and new_s:",
     "            if self.motion and new_s:",
     (T_MOT,), 'KILLED'),

    ('a-growth-stage-has-its-finished-self-underneath-it', 'anim',
     "                                    col, label, base_s=base)",
     "                                    col, label)",
     (T_MOT,), 'KILLED'),

]

# `a-growth-stage-has-its-finished-self-underneath-it` shipped for one commit
# as an EXPECTED SURVIVOR, with the note that closing it needed a pixel probe
# of a growth stage rather than another count. It has one now
# (`test_a_growth_stage_is_not_drawn_over_its_finished_self`, asserting the
# restore ink grows stage by stage), so the row is an ordinary KILLED.
#
# #946 SECTION 4 has NO row here on purpose. Its refusal lives in
# `make_movie.DEFAULT_SIZE` / `DEFAULT_FPS`, which this battery does not own,
# and `tests/test_946_format_defaults.py` gates it by RE-MEASURING rather than
# by a source edit -- a mutation of a default is just a different default, and
# the gate's whole point is that the number has to come past a measurement.

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
