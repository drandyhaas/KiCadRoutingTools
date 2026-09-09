#!/usr/bin/env python3
"""The #894 mutation battery, shipped so its numbers can be re-derived.

Same contract as `tests/mutate_799.py`, which this copies: a row is KILLED by a
failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN,
never silently skipped; `str.replace(old, new, 1)`, never `sed`; originals
restored in a `finally`; and it REFUSES to start on a dirty engine, because
restoring would write the committed text back over uncommitted work.

Every row here reverses a defect an adversarial review actually measured in
this work, so the battery is a record of what went wrong as much as a gate:

  * the plane term credited the whole net span rather than the part inside the
    blocker, so an 80mm graze outscored a 15mm cut 5.3 to 1;
  * it skipped nets with an endpoint inside the blocker, which made a BIGGER
    part obstruct LESS (measured: -13% for +1mm a side, then saturation);
  * it counted the ground and rail nets, which on two layers ARE the
    reference copper (with the filter off today they are 24.4 of 46.7mm on
    one board -- the figures below are of the DEFECT, measured before the
    clip landed, and are not what the term reports now);
  * `balance` weighed NPTH pads, whose `size` is a mask opening and not
    copper, biasing the result by 1.7x the signal it is the only term able to
    resolve;
  * the comparison summed or ordered terms in different currencies instead of
    refusing to judge across a moved population;
  * the placement tier reached the ROUTING half.

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. One writer per tree.

    python3 tests/mutate_894.py
    python3 tests/mutate_894.py --row plane-cut-credits-the-whole-chord
    python3 tests/mutate_894.py --list
"""
import argparse
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
sys.path.insert(0, _TESTS)

SCORE = os.path.join(_ROOT, 'py_placer', 'placement_score.py')
CONVERGE = os.path.join(_ROOT, 'py_placer', 'converge.py')
TARGETS = {'ps': SCORE, 'cv': CONVERGE}

T_TERMS = os.path.join(_TESTS, 'test_894_placement_terms.py')
T_RANK = os.path.join(_TESTS, 'test_894_placement_ranking.py')

KILLED = 'KILLED'

ROWS = [
    # ---- the plane term measured the wrong quantity, three ways ------------
    ('plane-cut-credits-the-whole-chord', 'ps',
     "            inside = _clip_len(a, b, rect)\n",
     "            inside = (math.hypot(b[0] - a[0], b[1] - a[1])\n"
     "                      if _clip_len(a, b, rect) > 1e-9 else 0.0)\n",
     (T_TERMS,), KILLED),

    ('plane-cut-skips-a-pad-under-the-part', 'ps',
     "        if p == 0:\n"
     "            if q < 0:\n"
     "                return 0.0        # parallel to this edge and outside it\n"
     "            continue\n",
     "        if p == 0:\n"
     "            if q < 0:\n"
     "                return 0.0        # parallel to this edge and outside it\n"
     "            continue\n"
     "        if not (0.0 < q / p < 1.0):\n"
     "            return 0.0\n",
     (T_TERMS,), KILLED),

    ('plane-cut-counts-the-reference-net-again', 'ps',
     "        if is_ground_net_name(name) or is_power_net_name(name):\n",
     "        if False:\n",
     (T_TERMS,), KILLED),

    # ---- balance weighed mask openings as if they were copper --------------
    ('balance-weighs-npth-pads-again', 'ps',
     "            if getattr(pad, 'pad_type', '') == 'np_thru_hole':\n",
     "            if False:\n",
     (T_TERMS,), KILLED),

    # ---- the comparison ----------------------------------------------------
    # A moved population must not be judged. Without this the plane term's
    # blocker set going 11 -> 1 reads as a spectacular improvement.
    # Gated on the TERMS test, which is where the moved-basis assertion lives.
    # Pointed at the ranking test first, this row SURVIVED -- a gate that does
    # not contain the assertion passes for free, which is the same defect as a
    # stale anchor wearing different clothes.
    ('compare-judges-across-a-moved-basis', 'ps',
     "        if comparable and a.get('basis') != b.get('basis'):\n",
     "        if False:\n",
     (T_TERMS,), KILLED),

    # Pareto, not "any improvement wins": a lap that trades one term for
    # another is not an improvement.
    ('compare-credits-a-trade-as-an-improvement', 'ps',
     "    if better and worse:\n        return 'mixed', detail\n",
     "    if better and worse:\n        return 'better', detail\n",
     (T_RANK,), KILLED),

    # ---- the tier's containment -------------------------------------------
    # It may only ever turn a tied PLACEMENT window from plateau to improving.
    ('placement-tier-reaches-the-routing-half', 'cv',
     "    _place = _placement_movement(runs_pairs) if half == 'placement' else None\n",
     "    _place = _placement_movement(runs_pairs)\n",
     (T_RANK,), KILLED),

    # `parent_sha` resolving to two rows is "I could not tell", never an answer.
    ('parent-score-guesses-an-ambiguous-parent', 'cv',
     "    if len(hits) != 1:\n        return None\n",
     "    if not hits:\n        return None\n",
     (T_RANK,), KILLED),
]

from mutation_anchors import preflight                        # noqa: E402
preflight(__file__)


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=_ROOT)
    return r.returncode == 0


def _run(test):
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8', test],
                       cwd=_ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=1800)
    return r.returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--row', action='append', default=[])
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for name, tgt, _o, _n, gates, exp in ROWS:
            print(f'{name}  [{tgt}]  gates={[os.path.basename(g) for g in gates]}'
                  f'  expect={exp}')
        return 0
    rows = ROWS
    if a.row:
        known = {r[0] for r in ROWS}
        bad = [r for r in a.row if r not in known]
        if bad:
            print(f'no such row(s): {bad}\nknown: {sorted(known)}',
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in a.row]
    if not _git_clean(sorted(set(TARGETS.values()))):
        print('REFUSING: the target tree is dirty. This restores the ORIGINAL '
              'text from disk and would write committed text over uncommitted '
              'work.', file=sys.stderr)
        return 2
    killed = survived = broken = 0
    for name, tgt, old, new, gates, expect in rows:
        path = TARGETS[tgt]
        src = open(path, encoding='utf-8').read()
        if src.count(old) != 1:
            print(f'  BROKEN    {name} -- anchor matched {src.count(old)} time(s)')
            broken += 1
            continue
        try:
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new, 1))
            rcs = [(os.path.basename(g), _run(g)) for g in gates]
        finally:
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(src)
        got = KILLED if any(rc != 0 for _g, rc in rcs) else 'SURVIVED'
        mark = 'ok  ' if got == expect else 'DISAGREES'
        print(f'  {got:<9} {mark} {name} -- '
              + ', '.join(f'{g} exit {rc}' for g, rc in rcs))
        if got == KILLED:
            killed += 1
        else:
            survived += 1
    print(f'\n{len(rows)} row(s): {killed} killed, {survived} survived, '
          f'{broken} broken')
    return 1 if (survived or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
