#!/usr/bin/env python3
"""#836's finding, kept from rotting -- the two mechanisms behind it, re-measured.

`tests/measure_836_flip_vs_reseat.py` answers #836's pre-registered question
and takes tens of minutes. This is the seconds-long gate that fails when the
engine moves the numbers the answer rests on, so the finding is a TEST rather
than a sentence in a doc that nobody re-runs.

It checks two mechanisms, not the conclusion:

1. **The objective prices a flip as free relief.** `_halo_pair_penalty` returns
   0.0 for a cross-side SMD pair, so every same-side SMD halo charge a part
   pays is relief a flip grants it for nothing. Measured: 207 of ulx3s's 226
   movable parts carry such a charge. That is why the measurement's verdict
   cannot be a cost delta -- and if an engine change ever made a flip cost
   something, THIS is the number that moves and this test is the reopening
   signal for #836.

2. **`reseat.py` sees a minority of any board.** `clusters_from_tethers`
   sources clusters from `groups.decap_tethers` and nothing else, so comparing
   a flip against native reseat alone would manufacture a win on every part
   reseat cannot see. Measured: 23% of ulx3s, 35% of glasgow_revC.

Both are re-derived here from the engine, never read from a constant. The
expected values are hand-stated with a tolerance band, because a number copied
out of the module it guards is not a change detector.

    python3 -X utf8 tests/test_836_flip_census.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
                HERE]

RUN_ALL_TIMEOUT = 300

#: (stem, halo-relief parts, movable, reseat members). Measured at the commit
#: that recorded the #836 answer. Bands rather than exact equality: a board
#: edit or a courtyard fix may move a part or two, and a gate that fires on
#: that gets deleted. A MECHANISM change moves these by far more.
EXPECT = [
    ('ulx3s',        207, 226,  53),
    ('glasgow_revC', 237, 243,  84),
]
#: How far a number may drift before this is a finding rather than noise.
TOL_PARTS = 8
#: The claims themselves, which no drift may cross.
MIN_HALO_FRAC = 0.85      # "a flip is free relief for almost every part"
MAX_RESEAT_COVERAGE = 0.60   # "reseat sees a minority of the board"

FAILURES = []


def check(name, cond, detail=''):
    print(f"  {'ok  ' if cond else 'FAIL'} {name}"
          + (f": {detail}" if not cond else ''))
    if not cond:
        FAILURES.append(f"{name}: {detail}")


def main():
    from kicad_parser import parse_kicad_pcb
    sys.path.insert(0, HERE)
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'm836', os.path.join(HERE, 'measure_836_flip_vs_reseat.py'))
    m836 = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m836)
    from pose_score import make_state

    print("#836 flip-vs-reseat mechanisms")
    for stem, e_halo, e_movable, e_reseat in EXPECT:
        path = os.path.join(ROOT, 'kicad_files', f'{stem}.kicad_pcb')
        if not os.path.isfile(path):
            check(f"{stem}: fixture present", False, path)
            continue
        pcb = parse_kicad_pcb(path)
        st = make_state(pcb, path)
        movable = [r for r, p in st.parts.items() if not p.locked]
        _per, positive = m836.halo_relief(st)
        clusters = m836._reseat_clusters(pcb, st)
        members = {m for c in clusters for m in c.members}

        check(f"{stem}: {e_movable} movable parts (+-{TOL_PARTS})",
              abs(len(movable) - e_movable) <= TOL_PARTS,
              f"got {len(movable)}")
        check(f"{stem}: {e_halo} parts get free halo relief (+-{TOL_PARTS})",
              abs(positive - e_halo) <= TOL_PARTS, f"got {positive}")
        # The CLAIM, not the number: this is what makes a cost delta useless
        # as a verdict, and an engine change that fixes it reopens #836.
        frac = positive / len(movable) if movable else 0.0
        check(f"{stem}: ...which is >= {MIN_HALO_FRAC:.0%} of the board -- the "
              f"reason the verdict cannot be a cost delta",
              frac >= MIN_HALO_FRAC,
              f"got {frac:.1%}. If this has FALLEN, the objective now charges "
              f"something for a flip and #836 should be re-opened and "
              f"re-measured")
        check(f"{stem}: {e_reseat} parts reachable by native reseat "
              f"(+-{TOL_PARTS})",
              abs(len(members) - e_reseat) <= TOL_PARTS,
              f"got {len(members)}")
        cov = len(members) / len(movable) if movable else 0.0
        check(f"{stem}: ...which is <= {MAX_RESEAT_COVERAGE:.0%} -- the reason "
              f"the move arm cannot be native reseat alone",
              cov <= MAX_RESEAT_COVERAGE,
              f"got {cov:.1%}. If reseat now covers the board, the "
              f"measurement's move arm should be re-derived")

    # The rig's own pre-registration must still be the one the answer was
    # taken against. Editing a threshold after the fact is the failure the
    # whole design is written around.
    pre = m836.PREREGISTRATION
    check("the pre-registered thresholds are unchanged",
          (pre['E_close'], pre['E_build'], pre['E_material_build'],
           pre['C4_ratio']) == (1, 6, 3, 0.5), str(pre))
    check("and it still names three distinct boards",
          len(set(pre['boards'])) == 3, str(pre['boards']))

    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #836 mechanisms, "
          f"{len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f"  - {f}")
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
