"""#553: run the real `diagnose_round` on real boards, because the fakes lied.

`test_553_diagnosis_rank.py` exercises the ranking through hand-built fakes.
That is the right shape for the algebra, and it has one blind spot which cost a
shipped crash: the fake `_Part` has no `locked` attribute, so
`getattr(p, 'locked', False)` is False in every unit test. The KiCad-locked cut
was therefore written, reviewed, committed -- and never once executed on a part
that is actually locked.

What it did on a real board: `glasgow_revC`'s `decap:U3` is `['C15', 'U3']`,
and BOTH are `(locked yes)`. The cut removed them from the candidate universe
while the raw `blocks` dict still went to `block_displacements`, which returned
a row for a key that no longer had a candidate entry -- `KeyError: 'decap:U3'`,
raised under exactly the `--group-by auto,decap` the README prints as its
example, after round 0 had already routed the whole board.

So this file runs the real thing on tracked boards. It is deliberately thin --
it asserts that the pipeline COMPLETES and that its output is coherent, not
what it ranks -- because a test that pinned the ranking on a real board would
break on every legality-grader change and get deleted.

    python3 -X utf8 tests/test_553_diagnosis_real_boards.py
"""

import json
import os
import sys
import time

RUN_ALL_TIMEOUT = 900

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # placement split

from run_utils import evidence  # noqa: E402

#: glasgow_revC first and by name: it is the board that crashed, and it is the
#: only tracked board with KiCad-locked footprints, so it is the only one that
#: exercises the cut at all. splitflap and watchy are ordinary boards with real
#: decap blocks; esp_prog derives almost nothing and is the degenerate end.
BOARDS = (
    ('glasgow_revC', ('decap',)),
    ('glasgow_revC', ('kicad', 'sheet', 'decap')),
    ('splitflap_driver', ('decap',)),
    ('watchy', ('decap',)),
    ('esp_prog', ('kicad', 'sheet')),
)

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.groups import derive_groups
    from placement import diagnosis as D
    import place_route_loop as prl

    seen_locked = False
    for name, sources in BOARDS:
        path = evidence(os.path.join(ROOT, 'kicad_files', f'{name}.kicad_pcb'),
                        f'the {name} board')
        pcb = parse_kicad_pcb(path)
        blocks = derive_groups(pcb, sources)
        locked = sorted(r for r, fp in (pcb.footprints or {}).items()
                        if getattr(fp, 'locked', False))
        seen_locked = seen_locked or bool(locked)
        label = f'{name} --group-by {",".join(sources)}'
        t0 = time.time()
        try:
            diag = prl.diagnose_round(pcb, path, blocks,
                                      {'blocker_report': None},
                                      budget=8, top_k=3)
        except Exception as e:                        # noqa: BLE001
            check(False, f'{label}: raised {type(e).__name__}: {e}')
            continue
        dt = time.time() - t0
        check(True, f'{label}: completed in {dt:.2f}s '
                    f'({len(blocks)} block(s), {len(locked)} locked part(s), '
                    f'{len(diag.selected)} selected)')

        # Coherence, not content. Each of these was violated by a real defect.
        keys = {c.key for c in diag.candidates}
        check(set(diag.selected_keys) <= keys,
              f'{label}: every selected key is a candidate')
        check(all(r in (pcb.footprints or {}) for r in diag.selected),
              f'{label}: every selected ref is a footprint on this board')
        check(not (set(diag.selected) & set(locked)),
              f'{label}: no KiCad-locked part was selected '
              f'({sorted(set(diag.selected) & set(locked))})')
        blob = json.dumps(D.to_json(diag), sort_keys=True)
        check(isinstance(json.loads(blob), dict),
              f'{label}: the report round-trips through JSON')
        check(D.NO_EFFICACY_CLAIM in blob,
              f'{label}: and carries the no-efficacy sentence')
        check(isinstance(D.format_text(diag), str),
              f'{label}: the operator table renders')
        if diag.degenerate:
            check(bool(diag.fallback_reason()),
                  f'{label}: a degenerate diagnosis names its reason')

    check(seen_locked,
          'at least one board in this file actually HAS a locked footprint -- '
          'without that the locked cut is untested here too, which is exactly '
          'how it shipped broken')

    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_553_diagnosis_real_boards: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
