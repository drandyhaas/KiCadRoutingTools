#!/usr/bin/env python3
"""#887 mutation battery: are the reader, the run clock and the panel covered,
or do their tests merely run?

    python3 tests/mutate_887.py
    python3 tests/mutate_887.py --list          # anchor PRE-FLIGHT, ~1 second
    python3 tests/mutate_887.py --row the-order-of-the-stage-rules-is-reversed

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine files
in place and restores them, and a suite running beside it would grade a mutated
tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and an
ERROR count the same, because a mutation that makes the graders crash is still a
mutation the graders noticed. **A row whose anchor does not match EXACTLY ONCE
is BROKEN, not skipped**: an anchor that silently matches nothing reports every
mutation as killed and is the most flattering possible bug. `--list` runs that
check on its own, in about a second, so a stale anchor is found before a full
run rather than fifty minutes into one.

Expected SURVIVORS are declared WITH THE REASON they are not a test hole.

The measured table is in the header of `tests/test_887_cmd_timing_reader.py`,
written from a run, and is never edited to match a prediction.

One row here is worth reading even if you skip the rest.
`the-order-of-the-stage-rules-is-reversed` is an expected SURVIVOR, and that is
the finding: the first version of cmd_timing.py claimed that order was
load-bearing, three reordering mutations survived a full suite, and the claim
was simply false -- the prefixes are pairwise incomparable, so nothing can
shadow anything. The row stays, expecting SURVIVED, so the day someone adds a
prefix that DOES shadow another it will start disagreeing.
"""
import argparse
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

TIMING = os.path.join(ROOT, 'py_router', 'cmd_timing.py')
PANELS = os.path.join(ROOT, 'py_router', 'movie_panels.py')
ISO = os.path.join(ROOT, 'py_router', 'kicad_iso_render.py')
TARGETS = {'t': TIMING, 'p': PANELS, 'i': ISO}

T_READER = os.path.join(TESTS, 'test_887_cmd_timing_reader.py')
T_CLOCK = os.path.join(TESTS, 'test_887_frame_clock.py')
T_SMALL = os.path.join(TESTS, 'test_887_small_items.py')
T_PANEL = os.path.join(TESTS, 'test_887_two_panel_frame.py')
#: The kicad-cli arm. Rows naming it are killed by EITHER file, and this one
#: self-skips (exit 77, which `run` reads as "not killed") on a machine without
#: kicad-cli -- so a row that only this file can kill must also name T_PANEL, or
#: it would report SURVIVED there for an environmental reason.
T_ISO = os.path.join(TESTS, 'test_887_iso_render.py')

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the bucketer ----------------------------------------------------
    ('pclose-is-routed-to-close-out', 't',
     "    ('close-out', ('fence', 'close')),\n    ('P*',        ('P',)),",
     "    ('close-out', ('fence', 'close', 'Pclose')),\n    ('P*',        ('P',)),",
     (T_READER,), 'KILLED'),
    ('fence-is-no-longer-close-out', 't',
     "    ('close-out', ('fence', 'close')),",
     "    ('close-out', ('close',)),",
     (T_READER,), 'KILLED'),
    ('stage_of-becomes-case-insensitive', 't',
     "        for pre in prefixes:\n            if s.startswith(pre):",
     "        for pre in prefixes:\n            if s.lower().startswith(pre.lower()):",
     (T_READER,), 'KILLED'),
    # THE ROW WORTH READING. See the module docstring: the order is inert, the
    # original comment said otherwise, and this survivor is the proof. It stays
    # so that adding a shadowing prefix starts disagreeing.
    ('the-order-of-the-stage-rules-is-reversed', 't',
     "STAGE_RULES = (\n    ('staging',   ('staging',)),\n"
     "    ('close-out', ('fence', 'close')),",
     "STAGE_RULES = (\n    ('close-out', ('fence', 'close')),\n"
     "    ('staging',   ('staging',)),",
     (T_READER,), 'SURVIVED'),   # the prefixes cannot shadow each other
    ('the-other-bucket-is-dropped-from-the-report', 't',
     "STAGE_ORDER = ('staging', 'P*', 'L*', 'R*', 'V*', 'close-out', OTHER)",
     "STAGE_ORDER = ('staging', 'P*', 'L*', 'R*', 'V*', 'close-out')",
     (T_READER,), 'KILLED'),
    ('unmatched-labels-stop-being-announced', 't',
     "    odd = [r.get('label') for r in rows if stage_of(r.get('label')) == OTHER]\n"
     "    if not odd:\n        return ''",
     "    odd = []\n    if not odd:\n        return ''",
     (T_READER,), 'KILLED'),

    # ---- reading and totals ---------------------------------------------
    ('rows-are-no-longer-sorted-by-t_start', 't',
     "    rows.sort(key=lambda r: (r['t_start'] is None, r['t_start'] or 0.0))",
     "    pass",
     (T_READER,), 'KILLED'),
    ('the-span-uses-the-last-rows-t_end', 't',
     "    t0, t1 = min(starts), max(ends)",
     "    t0, t1 = min(starts), ends[-1]",
     (T_READER,), 'KILLED'),
    ('outside-the-tools-is-clamped-at-zero', 't',
     "    return Totals(n, tool_s, t0, t1, run_s, round(run_s - tool_s, 3),",
     "    return Totals(n, tool_s, t0, t1, run_s, max(0.0, round(run_s - tool_s, 3)),",
     (T_READER,), 'KILLED'),
    ('numeric-coercion-is-removed', 't',
     "    return f if math.isfinite(f) else None",
     "    return v",
     (T_READER,), 'KILLED'),
    ('non-finite-values-are-passed-through', 't',
     "    return f if math.isfinite(f) else None",
     "    return f",
     (T_READER,), 'KILLED'),
    ('the-utf8-bom-codec-is-reverted', 't',
     "    with open(path, encoding='utf-8-sig') as f:",
     "    with open(path, encoding='utf-8') as f:",
     (T_READER,), 'KILLED'),
    ('a-label-less-row-is-left-without-one', 't',
     "            row.setdefault('label', 'unlabelled')",
     "            pass",
     (T_READER,), 'KILLED'),
    ('longest-breaks-ties-the-other-way', 't',
     "                                   r.get('t_start') if r.get('t_start') is not None\n"
     "                                   else float('inf')))",
     "                                   -(r.get('t_start') or 0.0)))",
     (T_READER,), 'KILLED'),
    ('find_ledger-stops-walking-up', 't',
     "    for _ in range(max(0, int(max_up or 0)) + 1):",
     "    for _ in range(1):",
     (T_READER,), 'KILLED'),
    ('fmt_hms-truncates-instead-of-rounding', 't',
     "    total = int(math.floor(float(seconds) + 0.5))",
     "    total = int(float(seconds))",
     (T_READER,), 'KILLED'),
    ('fmt_hms-uses-bankers-rounding', 't',
     "    total = int(math.floor(float(seconds) + 0.5))",
     "    total = int(round(float(seconds)))",
     (T_READER,), 'KILLED'),

    # ---- the report -----------------------------------------------------
    ('the-step-tables-exit-column-is-dropped', 't',
     "    out.append('| # | label | started | wall s | exit |')",
     "    out.append('| # | label | started | wall s |')",
     (T_READER,), 'KILLED'),
    ('the-step-table-loses-its-precision', 't',
     "                      ('%.3f' % r['wall_s']) if r.get('wall_s') is not None else '-',",
     "                      ('%.1f' % r['wall_s']) if r.get('wall_s') is not None else '-',",
     (T_READER,), 'KILLED'),
    ('the-tool-time-line-is-elided', 't',
     "    out.append('- Tool time (sum of wall_s): %.1f s = **%s**'\n"
     "               % (tot.tool_s, fmt_hms(tot.tool_s)))",
     "    pass",
     (T_READER,), 'KILLED'),
    ('the-stage-titles-are-crossed', 't',
     "    'R*': 'R* route',",
     "    'R*': 'L* driver',",
     (T_READER,), 'KILLED'),
    ('the-subtotals-columns-round-separately', 't',
     "        shown = round(wall, 1)\n"
     "        out.append('| %s | %d | %.1f | %s |'\n"
     "                   % (STAGE_TITLES[stage], n, shown, fmt_hms(shown)))",
     "        out.append('| %s | %d | %.1f | %s |'\n"
     "                   % (STAGE_TITLES[stage], n, wall, fmt_hms(wall)))",
     (T_READER,), 'KILLED'),
    ('report_data-loses-its-step-list', 't',
     "        'steps': [{'label': r.get('label'), 'iso_start': r.get('iso_start'),",
     "        'steps': [] and [{'label': r.get('label'), 'iso_start': r.get('iso_start'),",
     (T_READER,), 'KILLED'),

    # ---- the run clock ---------------------------------------------------
    ('the-mtime-witness-is-removed', 't',
     "            inside = [r for r in rows if r['t_start'] <= mt <= r['t_end']]",
     "            inside = []",
     (T_CLOCK,), 'KILLED'),
    ('argv-matching-ignores-the-exit-code', 't',
     "            clean = [r for r in hits if r.get('exit') in (0, '0')]",
     "            clean = list(hits)",
     (T_CLOCK,), 'KILLED'),
    ('argv-matching-becomes-a-substring-test', 't',
     "            hits = [r for r in rows if base in _basenames(r)]",
     "            hits = [r for r in rows if any(base in b for b in _basenames(r))]",
     (T_CLOCK,), 'KILLED'),
    ('the-monotone-clamp-is-removed', 't',
     "        if a.t is not None and prev is not None and a.t < prev:\n"
     "            a = a._replace(t=prev, basis=a.basis + '+clamped')",
     "        pass",
     (T_CLOCK,), 'KILLED'),
    ('a-clamped-instant-stops-saying-so', 't',
     "            a = a._replace(t=prev, basis=a.basis + '+clamped')",
     "            a = a._replace(t=prev)",
     (T_CLOCK,), 'KILLED'),
    ('the-pre-run-branch-is-removed', 't',
     "            t, basis = min(r['t_start'] for r in rows), 'pre-run'",
     "            pass",
     (T_CLOCK,), 'KILLED'),
    ('coverage-stops-requiring-every-beat', 't',
     "        if not self.anchors or len(self.resolved) != len(self.anchors):\n"
     "            return False",
     "        if not self.anchors:\n            return False",
     (T_CLOCK,), 'KILLED'),
    ('a-remaining-figure-is-always-offered', 't',
     "        if not self.covered:\n            return None\n"
     "        return max(0.0, self.tot.t1 - inst)",
     "        return max(0.0, self.tot.t1 - inst)",
     (T_CLOCK,), 'KILLED'),
    ('the-remaining-line-loses-its-qualifier', 't',
     "            out.append('remaining  %s  (exact, post-hoc: the run is over; this '\n"
     "                       'is a recorded total)' % fmt_hms(r.remaining_s))",
     "            out.append('remaining  %s' % fmt_hms(r.remaining_s))",
     (T_CLOCK,), 'KILLED'),
    ('an-interpolated-reading-stops-admitting-it', 't',
     "        interp = frac not in (0.0,)",
     "        interp = False",
     (T_CLOCK,), 'KILLED'),
    ('the-frame-stops-naming-its-basis', 't',
     "        out.append('basis  cmd_timing.jsonl - %d wrapped commands, %s'\n"
     "                   % (t.n if t else 0, how))",
     "        pass",
     (T_CLOCK,), 'KILLED'),
    ('the-overlay-stops-wrapping', 't',
     "            if cur and _w(cand) > avail:\n                wrapped.append(cur)\n"
     "                cur = word",
     "            if False:\n                wrapped.append(cur)\n                cur = word",
     (T_CLOCK,), 'KILLED'),
    ('the-png-block-gains-a-progress-key', 't',
     "            'krt:ledger_rows': t.n if t else 0,",
     "            'krt:ledger_rows': t.n if t else 0,\n            'krt:progress': 0.5,",
     (T_CLOCK,), 'KILLED'),
    # ---- the two-panel composer -----------------------------------------
    # Every row here is a mutation that SURVIVED the first panel review, i.e.
    # a hole the tests did not cover until this commit.
    ('the-composed-height-is-not-forced-even', 'p',
     "    total = H_top + H_iso\n    if total % 2:\n        H_iso += 1\n"
     "        total += 1",
     "    total = H_top + H_iso",
     (T_PANEL,), 'KILLED'),
    ('a-shot-may-be-shorter-than-a-beat', 'p',
     "            if best_len < 2 * MIN_SHOT_FRAMES:",
     "            if best_len < 2:",
     (T_PANEL,), 'KILLED'),
    ('the-head-fill-skips-an-empty-first-mark', 'p',
     "    head = all_marks[0][1]\n"
     "    for i in range(min(n_frames, max(0, all_marks[0][2]))):",
     "    head = spans[0][1] if spans else all_marks[0][1]\n"
     "    for i in range(min(n_frames, max(0, (spans or all_marks)[0][2]))):",
     (T_PANEL,), 'KILLED'),
    ('every-caption-names-the-opening-board', 'p',
     "            _m, note = _note_for(shot.board)",
     "            _m, note = _note_for(shots[0].board)",
     (T_PANEL,), 'KILLED'),
    ('the-probe-render-is-skipped-entirely', 'p',
     "        if not got:\n            return frames, _report('error', err)",
     "        if False:\n            return frames, _report('error', err)",
     (T_PANEL,), 'KILLED'),
    ('a-panel-that-cannot-be-read-is-not-counted', 'p',
     "        failed = sum(1 for e in errors.values() if e)",
     "        failed = sum(1 for k2 in results if not results[k2][0])",
     (T_PANEL,), 'KILLED'),
    ('the-failure-reason-is-not-drawn-into-the-panel', 'p',
     "            _wrapped_text(d, font, drawn_error,\n"
     "                          10, max(8, H // 3), W - 20, (196, 128, 128))",
     "            pass",
     (T_PANEL,), 'KILLED'),
    ('the-temp-dir-failure-escapes-as-a-traceback', 'p',
     "    except OSError as exc:\n"
     "        return frames, _report('error',\n"
     "                               'could not make a directory for the renders (%s)'\n"
     "                               % exc)",
     "    except ZeroDivisionError as exc:\n"
     "        return frames, _report('error', str(exc))",
     (T_PANEL,), 'KILLED'),
    ('a-non-finite-tuning-value-is-trusted', 'p',
     "    if v is None or not math.isfinite(v):",
     "    if v is None:",
     (T_PANEL,), 'KILLED'),
    ('the-disabled-message-hardcodes-zero', 'p',
     "                               '--iso-max-renders %d' % opts.max_renders)",
     "                               '--iso-max-renders 0')",
     (T_PANEL,), 'KILLED'),
    ('render-results-come-back-in-completion-order', 'i',
     "    with ThreadPoolExecutor(max_workers=n) as ex:\n"
     "        for k, v in ex.map(one, jobs):\n            out[k] = v",
     "    with ThreadPoolExecutor(max_workers=n) as ex:\n"
     "        for i2, (k, v) in enumerate(ex.map(one, jobs)):\n"
     "            out[jobs[-1 - i2][0]] = v",
     (T_PANEL, T_ISO), 'KILLED'),
    ('a-relative-model-path-is-resolved-against-the-cwd', 'i',
     "        if not os.path.isabs(path):\n"
     "            path = os.path.join(proj, path).replace('\\\\', '/')",
     "        pass",
     (T_PANEL,), 'KILLED'),
    ('models-note-prints-total-over-found', 'i',
     "    note = '3D models %d/%d' % (f, t)",
     "    note = '3D models %d/%d' % (t, f)",
     (T_PANEL,), 'KILLED'),
    ('a-mostly-bare-board-gets-no-warning', 'i',
     "    elif f < t * MOSTLY_BARE_FRACTION:",
     "    elif False:",
     (T_PANEL,), 'KILLED'),
    ('model_dirs-stops-defining-KIPRJMOD', 'i',
     "    if board_path:\n        dirs['KIPRJMOD'] = os.path.dirname(os.path.abspath(board_path))",
     "    if False:\n        dirs['KIPRJMOD'] = os.path.dirname(os.path.abspath(board_path))",
     (T_PANEL,), 'KILLED'),
    ('an-unreadable-png-is-accepted-as-a-render', 'i',
     "        with Image.open(out_png) as probe:\n            probe.verify()",
     "        pass",
     (T_PANEL, T_ISO), 'KILLED'),
    ('the-error-tail-is-sliced-from-the-end-again', 'i',
     "        head = blob.splitlines()[0].strip() if blob else ''",
     "        head = blob.replace('\\n', ' ')[-160:] if blob else ''",
     (T_PANEL, T_ISO), 'SURVIVED'),   # a one-line stderr reads the same

    ('the-png-block-keeps-remaining-without-coverage', 't',
     "        if r.remaining_s is not None:\n"
     "            m['krt:remaining_s'] = round(r.remaining_s, 1)",
     "        if True:\n"
     "            m['krt:remaining_s'] = round(r.remaining_s or 0.0, 1)",
     (T_CLOCK,), 'KILLED'),
]


def run(tests):
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True)
        if r.returncode != 0:
            return True, os.path.basename(t)
    return False, ''


def preflight():
    """Every anchor must match exactly once. Returns the number that do not."""
    src = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    bad = 0
    for name, tgt, old, _new, tests, exp in ROWS:
        n = src[tgt].count(old)
        flag = '' if n == 1 else '   *** BROKEN: matched %dx' % n
        if n != 1:
            bad += 1
        print('  %-46s %-16s %-9s%s'
              % (name, os.path.basename(TARGETS[tgt]), exp, flag))
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true',
                    help='anchor pre-flight only: no mutation, no tests')
    a = ap.parse_args()

    if a.list:
        bad = preflight()
        print('\n%d row(s), %d with a stale anchor' % (len(ROWS), bad))
        return 1 if bad else 0

    # A dirty engine tree would be RESTORED to its committed text, silently
    # destroying uncommitted work. Refuse rather than help.
    dirty = subprocess.run(['git', 'diff', '--quiet', '--'] + list(TARGETS.values()),
                           cwd=ROOT).returncode
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes.\nRestoring them would write the COMMITTED text back '
              'over your work. Commit first.')
        return 2

    rows = [r for r in ROWS if not a.row or r[0] == a.row]
    if not rows:
        print('no row named %r' % a.row)
        return 2
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print('  %-46s BROKEN (anchor matched %dx)' % (name, src.count(old)))
                broken += 1
                continue
            with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new))
            try:
                died, by = run(tests)
            finally:
                with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                    fh.write(src)
            got = 'KILLED' if died else 'SURVIVED'
            mark = '' if got == exp else '   *** DISAGREES with ' + exp
            if got != exp:
                disagree += 1
            killed += died
            survived += not died
            print('  %-46s %-9s %s%s' % (name, got, by, mark))
    finally:
        for k, v in TARGETS.items():
            with open(v, 'w', encoding='utf-8', newline='') as fh:
                fh.write(originals[k])
    print('\n%d row(s): %d killed, %d survived, %d broken, %d disagreeing '
          'with expectation' % (len(rows), killed, survived, broken, disagree))
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
