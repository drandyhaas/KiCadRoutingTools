"""The #1081 mutation battery: the stage3d film and the defects it closed.

One row per load-bearing line of the PR, each reverting it; every row names
the test that must fail. A count is only checkable if the exact source edit is
written down, so the edits live here as data, next to the verdicts they
produce (see `mutate_946.py` for why that matters).

**THE ROWS TO LOOK AT FIRST if this file ever goes red** restore a defect
somebody measured:

  * `flip-skips-its-chrome` (#1082) -- the rail and layer strip of 38 frames
    drawn from another frame's record;
  * `copper-after-the-flip-unmirrored` (#1085) -- the film reads B, F, B;
  * `crossing-on-a-rejected-lap` -- WORKING on a lap the run threw away
    (three real ledgers, phase-4 verification);
  * `back-bodies-inside-the-board` -- 83 of 83 back parts on orangecrab.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1081.py
    python3 tests/mutate_1081.py --row crossing-on-a-rejected-lap

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
_S3D = os.path.join(_ROOT, 'py_router', 'stage3d')

TARGETS = {
    'layout': os.path.join(_ROOT, 'py_router', 'frame_layout.py'),
    'anim': os.path.join(_ROOT, 'py_router', 'animate_route.py'),
    'cam': os.path.join(_ROOT, 'py_router', 'movie_camera.py'),
    'ledger': os.path.join(_ROOT, 'py_router', 'ledger_score.py'),
    'bench': os.path.join(_ROOT, 'py_router', 'movie_benchmark.py'),
    'tl': os.path.join(_S3D, 'timeline.py'),
    'film': os.path.join(_S3D, 'film.py'),
    'page': os.path.join(_S3D, 'page', 'stage3d.mjs'),
    'env': os.path.join(_ROOT, 'py_router', 'env_knobs.py'),
    'render': os.path.join(_ROOT, 'py_router', 'route_render.py'),
    'r3d': os.path.join(_S3D, 'render3d.py'),
    'fp': os.path.join(_ROOT, 'py_router', 'film_passes.py'),
    'scene': os.path.join(_S3D, 'scene.py'),
}


def _t(name):
    return os.path.join(_TESTS, name)


T_REC = _t('test_1081_frame_record.py')
T_ROT = _t('test_1081_rotation_glide.py')
T_LAY = _t('test_1081_stage3d_layout.py')
T_BAND = _t('test_1081_benchmark_band.py')
T_TL = _t('test_1081_timeline.py')
T_R3D = _t('test_1081_render3d.py')
T_E2E = _t('test_1081_e2e.py')
T_DEF = _t('test_1081_defaults.py')
T_BL = _t('test_1081_blender.py')
T_1042 = _t('test_1042_placement_panels.py')

# (name, target, old, new, tests, expect)
ROWS = [
    # --- the one frame path (#1082-#1085) ----------------------------------
    ('flip-skips-its-chrome', 'cam',
     "                                       record_chrome=True)",
     "                                       record_chrome=False)",
     (T_REC,), 'KILLED'),
    ('flip-stamps-over-the-rail', 'anim',
     "            if self.split_caption:\n"
     "                label = None\n"
     "        if kind == 'flip' and label:",
     "        if kind == 'flip' and label:",
     (T_REC,), 'KILLED'),
    ('back-glide-drops-its-ghost', 'anim',
     "                            overlays=[self.overlay] if self.overlay else None,",
     "                            overlays=None,",
     (T_REC,), 'KILLED'),
    ('copper-after-the-flip-unmirrored', 'anim',
     "                            mirror=True,",
     "                            mirror=False,",
     (T_REC,), 'KILLED'),
    # 'key-drawn-before-the-mirror' went with the in-frame key after the flip:
    # only a rail-less (legacy) film frame had one; film frames are all railed.
    ('epoch-caught-mid-glide', 'anim',
     "            self.stage_epochs.append(_pose_table(pcb, self.moving_rest))",
     "            self.stage_epochs.append(_pose_table(pcb))",
     (T_REC, T_ROT), 'KILLED'),

    # --- the turning glide (#1086) -----------------------------------------
    ('rotation-snaps-again', 'cam',
     "            turns[ref] = turn_deg(m)",
     "            turns[ref] = 0.0",
     (T_ROT,), 'KILLED'),
    ('turned-pad-left-unfolded', 'cam',
     "            p.rect_rotation, p.size_x, p.size_y = fold_rect(rr - phi, sx, sy)",
     "            p.rect_rotation, p.size_x, p.size_y = rr - phi, sx, sy",
     (T_ROT,), 'KILLED'),

    # --- the layout ----------------------------------------------------------
    # 'auto-picks-stage3d' went with 'auto': stage3d is the only layout.
    # Its replacement pins the one frame-shape decision left -- a frame too
    # extreme to hold a column is BOARD-ONLY, not a squeezed column.
    ('an-extreme-frame-keeps-a-column', 'layout',
     "        board, panel_box = Box(0, inner_y, W, inner_h), None",
     "        board, panel_box, _w = _stage3d_boxes(W, H, inner_y, inner_h)",
     (T_LAY,), 'KILLED'),
    ('board-below-seventy', 'layout',
     "        bw = min(W - 2, _up_even(STAGE3D_BOARD_W_FRAC * W))",
     "        bw = min(W - 2, _up_even(0.6 * W))",
     (T_LAY,), 'KILLED'),

    # --- the benchmark band ---------------------------------------------------
    ('row-done-ignores-a-fail-lens', 'ledger',
     "    if 'FAIL' in lens_verdicts(row).values():",
     "    if False:",
     (T_BAND,), 'KILLED'),
    ('row-done-ignores-unknown', 'ledger',
     "        return False                # any value: RAN and could not answer",
     "        pass",
     (T_BAND,), 'KILLED'),
    ('crossing-on-a-rejected-lap', 'bench',
     "        if not p.accepted or p.blocking is None:",
     "        if p.blocking is None:",
     (T_BAND,), 'KILLED'),
    ('a-weighted-sum-below-the-line', 'bench',
     "        return (0, 0, p.key)",
     "        return (0, 0, (sum(float(v) for v in p.key if LS.plottable(v)),))",
     (T_BAND,), 'KILLED'),
    ('gold-against-a-broken-benchmark', 'bench',
     "    if (bench is not None and bench.blocking == 0",
     "    if (bench is not None",
     (T_BAND,), 'KILLED'),
    ('a-foreign-score-believed', 'bench',
     "        elif os.path.isfile(board) and sha != _sha256(board):",
     "        elif False:",
     (T_BAND,), 'KILLED'),
    ('deciding-term-backwards', 'ledger',
     "            return (name, float(b) - float(a))",
     "            return (name, float(a) - float(b))",
     (T_BAND,), 'KILLED'),

    # --- the timeline and the 3D board ----------------------------------------
    ('timeline-drops-the-growth-hide', 'tl',
     "            hide = sorted(k2i[k] for k in r['hide'] if k in k2i)",
     "            hide = []",
     (T_TL,), 'KILLED'),
    ('copper-dies-a-step-late', 'tl',
     "            if it[-2] < n and (it[-1] < 0 or it[-1] >= n) and i not in h]",
     "            if it[-2] < n and (it[-1] < 0 or it[-1] > n) and i not in h]",
     (T_TL,), 'KILLED'),
    ('copper-does-not-turn-the-board', 'tl',
     "            w = 'B' if a == 'B.Cu' else ('F' if a == 'F.Cu' else None)",
     "            w = None",
     (T_TL,), 'KILLED'),
    ('the-turn-cuts-a-glide', 'tl',
     "        if w == side and glide[last - 1]:",
     "        if False:",
     (T_TL,), 'KILLED'),
    # 'the-column-swaps-panels-again' went with the phase switch itself: the
    # column has one content, so there is nothing left to swap to.
    ('glide-poses-not-recorded', 'anim',
     "                    moving[ref] = (fp.x, fp.y, fp.rotation or 0.0)",
     "                    pass",
     (T_TL,), 'KILLED'),
    ('pours-never-revealed', 'tl',
     "              'zones': sorted(int(z) for z in (r.get('zones') or ())),",
     "              'zones': [],",
     (T_TL, T_R3D), 'KILLED'),
    ('custom-pads-back-to-boxes', 'page',
     "        if (polys) {",
     "        if (false) {",
     (T_R3D,), 'KILLED'),
    ('back-bodies-inside-the-board', 'page',
     "      part.body.position.y = back ? -(h / 2 + 0.06) : h / 2 + 0.06;",
     "      part.body.position.y = h / 2 + 0.06;",
     (T_R3D,), 'KILLED'),
    # --- the defaults (#1081), Blender (#1089), one pipeline (#1087) -------
    # 'the-film-default-back-to-legacy' went with KICAD_MOVIE_LAYOUT: there
    # is no layout to default to. What replaces it is the retired knob's
    # warning -- a value set in a shell must be SAID, not ignored.
    ('a-retired-knob-goes-silent', 'layout',
     "    print(line, file=sys.stderr)\n    return line",
     "    return line",
     (T_DEF,), 'KILLED'),
    # A retired layout NAME given as the aspect (`--aspect stacked`,
    # `$KICAD_MOVIE_ASPECT=sidebar`) raised out of build_boards and lost
    # the movie; it is said once and declares nothing.
    ('a-retired-aspect-name-kills-the-movie', 'layout',
     "    if key in RETIRED_LAYOUTS:",
     "    if False:",
     (T_LAY,), 'KILLED'),
    ('a-retired-aspect-name-goes-silent', 'layout',
     "        warn_retired_aspect(key)",
     "        pass",
     (T_LAY,), 'KILLED'),
    ('a-retired-aspect-name-said-every-call', 'layout',
     "    if name in _RETIRED_ASPECT_SAID:",
     "    if False:",
     (T_LAY,), 'KILLED'),
    ('the-theme-default-back-to-dark', 'env',
     "    g['RENDER_THEME'] = _s('KICAD_RENDER_THEME', 'light')",
     "    g['RENDER_THEME'] = _s('KICAD_RENDER_THEME', 'dark')",
     (T_DEF,), 'KILLED'),
    ('an-unthemed-renderer-hardwired-dark', 'render',
     "            else _default_theme()",
     "            else _THEME_DARK",
     (T_DEF,), 'KILLED'),
    ('blender-keeps-its-render-time-metadata', 'r3d',
     "        px.save(p, format='PNG')",
     "        pass",
     (T_BL,), 'KILLED'),
    # 'the-pipeline-drops-the-verdict-band' went with the verdict band (no
    # stage3d film draws it). The pipeline's other band still must land.
    ('the-pipeline-drops-the-placement-panels', 'fp',
     "            frames = movie_placement.compose(frames, box, ptrack, marks,\n"
     "                                             theme, geom.frame.h)",
     "            pass",
     (T_1042,), 'KILLED'),
    ('a-2d-fallback-that-says-nothing', 'film',
     "        _say('stage3d: 2D X-ray in the board box -- %s' % why, notes)",
     "        pass",
     (T_E2E,), 'KILLED'),
    # the missing-tool arm expects the reason for whichever tool is missing
    # FIRST (playwright-core on a machine with no `npm ci`), so a fallback
    # that names no tool must still fail it
    ('a-2d-fallback-that-names-no-tool', 'film',
     "        _say('stage3d: 2D X-ray in the board box -- %s' % why, notes)",
     "        _say('stage3d: 2D X-ray in the board box -- a tool is "
     "missing', notes)",
     (T_E2E,), 'KILLED'),
    # --- found by the esp_prog film (run 35) -------------------------------
    ('the-camera-fitted-once-for-the-film', 'page',
     "  fitCamera();",
     "  // fitCamera();",
     (T_E2E,), 'KILLED'),
    ('the-caption-no-obstacle', 'bench',
     "        occupied.append((bb[0] - 2, bb[1] - 1, bb[2] + 2, bb[3] + 1))",
     "        pass",
     (T_BAND,), 'KILLED'),
    ('a-pour-outline-sets-the-fit', 'page',
     "    if (!o.isMesh || o.isInstancedMesh || !o.geometry || o.userData.noFit) return;",
     "    if (!o.isMesh || o.isInstancedMesh || !o.geometry) return;",
     (T_R3D,), 'KILLED'),
    ('a-label-off-the-bands-left-edge', 'bench',
     "            if bb[0] < x0:",
     "            if False:",
     (T_BAND,), 'KILLED'),
    ('a-flat-model-path-not-looked-up', 'scene',
     "        found = _by_name(path, dirs)",
     "        found = None",
     (T_TL,), 'KILLED'),
    ('a-copper-only-part-gets-a-box', 'scene',
     "            & {'board_only', 'exclude_from_pos_files'}}",
     "            & set()}",
     (T_TL,), 'KILLED'),
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
        p = subprocess.run([sys.executable, '-X', 'utf8', t],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT)
        if p.returncode not in (0,):
            failed.append((os.path.basename(t), p.returncode,
                           [l.strip()[:90] for l in
                            ((p.stdout or '') + (p.stderr or '')).splitlines()
                            if 'FAIL' in l or 'Error' in l][:2]))
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
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', help='run a single row by name')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
