#!/usr/bin/env python3
"""#962 mutation battery: does a test notice when each #962 behaviour is undone?

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine files
in place and restores them, and a suite running beside it would grade a mutated
tree. One writer per tree -- run this in its own worktree. Never import it.

A row is KILLED when any named test exits non-zero. An anchor must match its
target EXACTLY ONCE (`mutation_anchors.preflight`, run before anything is
mutated): an anchor that matches nothing mutates nothing, and every test it
names then passes for free, which reads as KILLED-or-SURVIVED for the wrong
reason (#877). The unmutated tree is run first; a red baseline refuses.

Rows cover the five areas of the PR:
  - graphic copper vs the outline (the waiver's scope, the census, the
    baseline origin, the placement channel and its consumers);
  - the paste model (margin resolution, the zero-is-unset version gate, the
    fill rule, which nets an opening concerns);
  - the keep-out under --same-net-pad-clearance;
  - the Type VII stamp (who is stamped, who is restored, the format gate, the
    BGA path);
  - check_drc's via-in-paste class and its consumers.

    python3 -X utf8 tests/mutate_962.py
    python3 -X utf8 tests/mutate_962.py --list
    python3 -X utf8 tests/mutate_962.py --row lock-waives-graphic
    python3 -X utf8 tests/mutate_962.py --verify-anchors
"""
import argparse
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

DRC = os.path.join(ROOT, 'py_router', 'check_drc.py')
FAB = os.path.join(ROOT, 'py_router', 'fab_notes.py')
PASTE = os.path.join(ROOT, 'py_router', 'paste_apertures.py')
PARSER = os.path.join(ROOT, 'py_router', 'kicad_parser.py')
OBST = os.path.join(ROOT, 'py_router', 'obstacle_map.py')
PLANE = os.path.join(ROOT, 'py_router', 'plane_obstacle_builder.py')
BGA = os.path.join(ROOT, 'py_router', 'bga_fanout', '__init__.py')
LEG = os.path.join(ROOT, 'py_placer', 'placement', 'legality.py')
OPS = os.path.join(ROOT, 'py_placer', 'placement', 'pose_ops.py')
KDC = os.path.join(ROOT, 'tests', 'stress', 'kicad_drc_compare.py')
RENDER = os.path.join(ROOT, 'py_tools', 'render_placement.py')
SCORE = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing',
                     'scripts', 'board_score.py')

TARGETS = {'d': DRC, 'f': FAB, 'p': PASTE, 'k': PARSER, 'o': OBST, 'b': PLANE,
           'g': BGA, 'l': LEG, 'q': OPS, 'c': KDC, 'r': RENDER, 's': SCORE}

T_WAIVER = os.path.join(TESTS, 'test_962_graphic_waiver.py')
T_OOB = os.path.join(TESTS, 'test_962_graphic_oob_channel.py')
T_POSE = os.path.join(TESTS, 'test_962_place_pose_graphic.py')
T_PARSE = os.path.join(TESTS, 'test_962_paste_aperture_parse.py')
T_KEEP = os.path.join(TESTS, 'test_962_aperture_keepout.py')
T_STAMP = os.path.join(TESTS, 'test_962_type_vii_stamp.py')
T_VIP = os.path.join(TESTS, 'test_962_check_drc_via_in_paste.py')
T_CENSUS = os.path.join(TESTS, 'test_962_census_via_in_paste.py')

#: (name, target key, anchor, replacement, tests that must notice, expectation)
ROWS = [
    # ---- graphic copper vs the outline -----------------------------------
    ('lock-waives-graphic', 'd',
     "GRAPHIC_WAIVED_STATES = ('board-level', 'board-outline')",
     "GRAPHIC_WAIVED_STATES = ('board-level', 'board-outline', 'locked')",
     (T_WAIVER,), 'KILLED'),
    ('unresolved-owner-waived', 'd',
     "        return 'unresolved'",
     "        return 'board-level'",
     (T_WAIVER,), 'KILLED'),
    ('off-board-row-never-emitted', 'd',
     "                and _row['owner_state'] not in GRAPHIC_WAIVED_STATES):",
     "                and False):",
     (T_WAIVER,), 'KILLED'),
    ('filled-interior-ignored', 'd',
     "        best, worst_seg, worst_pt = _filled_interior_edge_depth(shape, dist_rings)",
     "        pass",
     (T_WAIVER,), 'KILLED'),
    ('keyhole-split-at-its-start', 'd',
     "        same_shape = bool(cur and getattr(sg, 'uuid', '')",
     "        same_shape = False and bool(cur and getattr(sg, 'uuid', '')",
     (T_WAIVER,), 'KILLED'),
    ('true-circle-replaced-by-16-gon', 'd',
     "    if circ is not None:\n        cx, cy, r = circ\n"
     "        n = max(32, int(2 * math.pi * r / step) + 1)",
     "    if False:\n        cx, cy, r = circ\n"
     "        n = max(32, int(2 * math.pi * r / step) + 1)",
     (T_WAIVER,), 'KILLED'),
    ('moved-side-graded-on-stale-copper', 'd',
     "        if moved is False:",
     "        if False:",
     (T_WAIVER,), 'KILLED'),
    ('no-outline-is-silent', 'd',
     "    no_outline = not (geom[0] or geom[1] or geom[4])",
     "    no_outline = False",
     (T_WAIVER,), 'KILLED'),
    ('baseline-ignores-rotation', 'd',
     "                and abs(dr) < 1e-6 and fp.layer == old[3])",
     "                and fp.layer == old[3])",
     (T_WAIVER,), 'KILLED'),
    ('baseline-ignores-side', 'd',
     "                and abs(dr) < 1e-6 and fp.layer == old[3])",
     "                and abs(dr) < 1e-6)",
     (T_WAIVER,), 'KILLED'),
    ('legality-counts-waived-parts', 'l',
     "            if row['owner_state'] in GRAPHIC_WAIVED_STATES:",
     "            if False:",
     (T_WAIVER, T_OOB), 'KILLED'),
    ('place-pose-ignores-graphic-count', 'q',
     "                 'oob_graphic_copper_count')",
     "                 )",
     (T_POSE,), 'KILLED'),
    ('swap-launders-an-overrun', 'q',
     "        if new and not improved:",
     "        if False:",
     (T_POSE,), 'KILLED'),
    ('strict-improvement-refused', 'q',
     "        if new and not improved:",
     "        if new:",
     (T_POSE,), 'KILLED'),
    ('render-ignores-the-proposal', 'r',
     "            fps[k] = _dc.replace(fp, x=p.x, y=p.y, rotation=p.rot, layer=layer)",
     "            fps[k] = fp",
     (T_OOB,), 'KILLED'),
    ('compare-drops-graphic-types', 'c',
     '                 "graphic-off-board", "graphic-board-edge"}',
     '                 }',
     (T_WAIVER,), 'KILLED'),

    # ---- the paste model --------------------------------------------------
    ('margin-clamp-dropped', 'p',
     "    mx = max(mx, -pad.size_x / 2.0)",
     "    mx = mx",
     (T_PARSE,), 'KILLED'),
    ('custom-ratio-from-primitive-extent', 'p',
     "        ax, ay = getattr(pad, 'anchor_size', None) or (pad.size_x, pad.size_y)",
     "        ax, ay = (pad.size_x, pad.size_y)",
     (T_PARSE,), 'KILLED'),
    ('opening-ignores-owner-pads', 'p',
     "        if _pad_overlaps(ap, pad):\n            nets.add(pad.net_id)",
     "        if False:\n            nets.add(pad.net_id)",
     (T_PARSE, T_KEEP), 'KILLED'),
    ('explicit-zero-always-unset', 'k',
     "PASTE_ZERO_IS_UNSET_MAX_VERSION = 20240201",
     "PASTE_ZERO_IS_UNSET_MAX_VERSION = 99999999",
     (T_PARSE,), 'KILLED'),
    ('stroked-rect-read-filled', 'k',
     "        return width <= 0",
     "        return True",
     (T_PARSE,), 'KILLED'),

    # ---- the keep-out -----------------------------------------------------
    ('keepout-drops-the-aperture-half', 'o',
     "    if apertures is None and pads is None:\n"
     "        _ap_cells = paste_aperture_keepout_cells(pcb_data, net_id, config, snpc)",
     "    if apertures is None and pads is None:\n"
     "        _ap_cells = np.empty((0, 2), dtype=np.int32)",
     (T_KEEP,), 'KILLED'),
    ('plane-map-keepout-at-zero', 'b',
     "    if exclude_net_id is not None and same_net_pad_clearance > 0:",
     "    if exclude_net_id is not None and same_net_pad_clearance >= 0:",
     (T_KEEP,), 'KILLED'),

    # ---- the Type VII stamp -----------------------------------------------
    ('stamp-overrides-own-spec', 'f',
     "        if own:\n            _unprot('own spec kept')",
     "        if False:\n            _unprot('own spec kept')",
     (T_STAMP,), 'KILLED'),
    ('stamp-ignores-a-filled-board', 'f',
     "        if is_filled_and_capped(effective_via_protection(own, setup)):",
     "        if False:",
     (T_STAMP,), 'KILLED'),
    ('relaid-via-loses-its-spec', 'f',
     "            if spec:",
     "            if False:",
     (T_STAMP,), 'KILLED'),
    ('format-gate-off', 'f',
     "    return not (0 < ver < PER_VIA_PROTECTION_MIN_VERSION)",
     "    return True",
     (T_STAMP,), 'KILLED'),
    ('bga-fanout-unstamped', 'g',
     "    _st962, _rec962 = via_protection_stamps(vias_to_add, [], pcb_data)",
     "    _st962, _rec962 = [], {}",
     (T_STAMP,), 'KILLED'),

    # ---- check_drc via-in-paste -------------------------------------------
    ('via-in-paste-pass-off', 'd',
     "    _via_in_paste_pass(pcb_data, matching_via_nets, _baseline_pd, violations,",
     "    (lambda *a, **k: None)(pcb_data, matching_via_nets, _baseline_pd, violations,",
     (T_VIP, T_CENSUS), 'KILLED'),
    ('protected-via-counted', 'd',
     "        if is_filled_and_capped(eff):",
     "        if False:",
     (T_VIP, T_STAMP), 'KILLED'),
    ('baseline-never-inherits-a-via', 'd',
     "        elif snap_by_net is not None and _preexisting(v, snap_by_net):",
     "        elif False:",
     (T_VIP, T_CENSUS), 'KILLED'),
    ('compare-matches-via-in-paste-as-copper', 'c',
     '    cd = [c for c in cd if c["type"] not in CD_VIA_PASTE_TYPES]',
     '    cd = cd',
     (T_VIP,), 'KILLED'),
    ('board-score-blocks-on-via-in-paste', 's',
     "             and t not in VIA_PASTE_TYPES}",
     "             }",
     (T_OOB,), 'KILLED'),
]

from mutation_anchors import preflight  # noqa: E402
preflight(__file__)


def _uncache(path):
    """Delete the target's cached bytecode: a same-size mutated and restored
    file within one second leaves the MUTATED .pyc valid (measured, #892)."""
    import importlib
    import importlib.util
    try:
        cached = importlib.util.cache_from_source(path)
        if os.path.exists(cached):
            os.remove(cached)
    except (OSError, ValueError, NotImplementedError):
        pass
    importlib.invalidate_caches()


def _write(path, text):
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    _uncache(path)


def run(tests):
    env = dict(os.environ)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['PYTHONHASHSEED'] = '0'
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True, env=env)
        if r.returncode != 0:
            return True, os.path.basename(t)
    return False, ''


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for n, t, _o, _w, _tests, exp in ROWS:
            print('  %-44s %-26s %s' % (n, os.path.basename(TARGETS[t]), exp))
        return 0

    # A dirty engine tree would be RESTORED to its committed text, silently
    # destroying uncommitted work. Refuse rather than help.
    dirty = subprocess.run(['git', 'diff', '--quiet', '--']
                           + list(TARGETS.values()), cwd=ROOT).returncode
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes. Restoring them would write the COMMITTED text back '
              'over your work. Commit first.')
        return 2

    rows = [r for r in ROWS if not a.row or r[0] == a.row]
    if not rows:
        print('no row named %r' % a.row)
        return 2
    originals = {k: open(v, encoding='utf-8', newline='').read()
                 for k, v in TARGETS.items()}
    # The BASELINE, run once: a target test that is red before any mutation
    # scores every row KILLED and the battery exits 0 on a lie.
    died, by = run(sorted({t for r in rows for t in r[4]}))
    if died:
        print('REFUSED: %s is already failing on the UNMUTATED tree, so every '
              'row below would report KILLED for the wrong reason.' % by)
        return 2

    killed = survived = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            _write(TARGETS[tgt], src.replace(old, new))
            try:
                died, by = run(tests)
            finally:
                _write(TARGETS[tgt], src)
            got = 'KILLED' if died else 'SURVIVED'
            killed += died
            survived += not died
            mark = '' if got == exp else '   <-- expected %s' % exp
            disagree += bool(mark)
            print('  %-44s %-8s %s%s' % (name, got, by, mark), flush=True)
    finally:
        for k, v in TARGETS.items():
            _write(v, originals[k])
    print('\n%d row(s): %d killed, %d survived, %d disagree with the expectation'
          % (len(rows), killed, survived, disagree))
    return 1 if disagree else 0


if __name__ == '__main__':
    sys.exit(main())
