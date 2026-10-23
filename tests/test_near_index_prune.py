#!/usr/bin/env python3
"""quench's `_NearIndex` is a PRUNE, never a verdict.

`violation_parts` and `candidate_valid` sweep every part on the board; on a
board of `_NEAR_INDEX_MIN_PARTS` or more parts the sweep visits only the parts
`_NearIndex` cannot prove clear. Its docstring claims the survivors come back
in `parts` order, so the loops see the same first violator, the same float sum
and the same early exit. What this pins:

* on splitflap_driver (above the threshold), both sweeps answer EXACTLY what
  the full sweep answers, pose for pose, through random moves of other parts
  (a moved part is re-bucketed) and a rotation-table entry added after the
  index was built (a 45-degree pose: `ensure_rotation` marks it stale);
* the test is not vacuous: the index exists and dropped parts from at least
  some sweeps;
* every field `rect()` / `tht_rect()` read is one whose write marks a part
  stale (`_NearWatched._GEOM_FIELDS`, kept by hand -- read off their source
  here, so a new read without a new entry fails);
* a superseded index stops watching: after a rebuild every part watches one
  dirty set, the live index's.

    python3 tests/test_near_index_prune.py
"""
import inspect
import os
import random
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb    # noqa: E402
import pose_score                           # noqa: E402
from placement import quench                # noqa: E402

RUN_ALL_TIMEOUT = 900

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _state():
    pcb = parse_kicad_pcb(BOARD)
    return pose_score.make_state(pcb, BOARD, clearance=0.2,
                                 board_edge_clearance=0.3)


def _answers(st, ref, x, y, rot):
    return st.violation_parts(ref, x, y, rot), st.candidate_valid(ref, x, y,
                                                                  rot)


def test_pruned_sweeps_answer_as_the_full_sweep():
    st = _state()
    assert len(st.parts) >= quench._NEAR_INDEX_MIN_PARTS, \
        f"BROKEN TEST: {len(st.parts)} parts never reach the index"
    real = st._sweep_others
    seen = {'calls': 0, 'pruned': 0}

    def counting(rects, clr):
        got = list(real(rects, clr))
        if st._near_ix:
            seen['calls'] += 1
            seen['pruned'] += len(got) < len(st.parts)
        return got
    st._sweep_others = counting

    rng = random.Random(1214)
    movable = sorted(r for r, p in st.parts.items() if not p.locked)
    assert len(movable) > 10, "BROKEN TEST: nothing to move"
    checked = moved = turned = 0
    for i in range(240):
        if i % 6 == 5:
            # Move some OTHER part: the index must re-bucket it.
            r = rng.choice(movable)
            p = st.parts[r]
            st.apply_move(r, p.x + rng.uniform(-4, 4),
                          p.y + rng.uniform(-4, 4), p.rot)
            moved += 1
        if i == 120:
            # A rotation-table entry added after the index was built, for a
            # part already AT that angle: its rect changes under the index.
            r = movable[0]
            p = st.parts[r]
            st.apply_move(r, p.x, p.y, 45.0)
            p.ensure_rotation(45.0)
            turned += 1
        ref = rng.choice(movable)
        p = st.parts[ref]
        rot = rng.choice((0.0, 90.0, 180.0, 270.0, 45.0))
        p.ensure_rotation(rot)
        x, y = p.x + rng.uniform(-3, 3), p.y + rng.uniform(-3, 3)
        with_ix = _answers(st, ref, x, y, rot)
        ix, st._near_ix = st._near_ix, False        # False: sweep everything
        try:
            full = _answers(st, ref, x, y, rot)
        finally:
            st._near_ix = ix
        assert with_ix == full, (
            f"pose {i}: {ref} at ({x:.4f}, {y:.4f}, {rot:g}) -- pruned "
            f"{with_ix} vs full sweep {full}")
        checked += 1
    assert isinstance(st._near_ix, quench._NearIndex), \
        f"BROKEN TEST: no index was built ({st._near_ix!r})"
    assert seen['calls'] and seen['pruned'], \
        f"BROKEN TEST: the index never pruned a sweep ({seen})"
    print(f"  PASS: {checked} poses, pruned == full sweep ({seen['pruned']} "
          f"of {seen['calls']} indexed sweeps dropped parts; {moved} moves, "
          f"{turned} late rotation entry)")


def test_every_field_rect_reads_marks_a_part_stale():
    reads = set()
    for fn in (quench._Part.rect, quench._Part.tht_rect):
        src = inspect.getsource(fn)
        reads |= set(re.findall(r'self\.(\w+)\b(?!\s*\()', src))
    assert {'x', 'y', 'rot', 'bounds_by_rot', 'tht_by_rot'} <= reads, \
        f"BROKEN TEST: the source scan found only {sorted(reads)}"
    missing = reads - quench._NearWatched._GEOM_FIELDS
    assert not missing, (
        f"rect()/tht_rect() read {sorted(missing)}, which _GEOM_FIELDS does "
        f"not list: a write to them would leave a _NearIndex footprint stale")
    print(f"  PASS: rect()/tht_rect() read {sorted(reads)}, all watched")


def test_a_superseded_index_stops_watching():
    st = _state()
    p0 = next(iter(st.parts.values()))
    st.violation_parts(p0.ref)                       # builds the index
    old = st._near_ix
    assert isinstance(old, quench._NearIndex), f"BROKEN TEST: {old!r}"
    st.parts = dict(st.parts)                        # a new dict: not current
    st.violation_parts(p0.ref)
    new = st._near_ix
    assert isinstance(new, quench._NearIndex) and new is not old, new
    for ref, p in st.parts.items():
        watch = p._ix_watch
        assert len(watch) == 1 and watch[0] is new.dirty, \
            f"{ref} watches {len(watch)} dirty set(s) after a rebuild"
    print(f"  PASS: after a rebuild all {len(st.parts)} parts watch only the "
          f"live index")


TESTS = [
    test_pruned_sweeps_answer_as_the_full_sweep,
    test_every_field_rect_reads_marks_a_part_stale,
    test_a_superseded_index_stops_watching,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
