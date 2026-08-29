#!/usr/bin/env python3
"""Where `--reseat-min-gain`'s default comes from (#698), shipped so the number
can be re-derived rather than quoted.

The only basis a sideways shuffle can score on is `scope_hpwl`, so the question
is: on real boards, what gain does an explicit re-seat actually produce? Parts
are chosen by pad count -- a property of the BOARD, not of the answer -- so the
sample cannot be fitted to the conclusion.

NOT named `test_*.py`: `tests/run_all.py` does not collect it, because it runs
16 full re-seats on corpus boards (minutes, not seconds) and asserts nothing.
It is a measurement, and its output is quoted in `placement/README.md`.

    python3 -X utf8 tests/measure_698_min_gain.py

**Read the `pre_prune` and `moved` columns before drawing the obvious
conclusion.** The first write-up of this measurement said the zero rows were the
search "re-seating the part where it already was". That is false, and these
columns are what show it: the search relocates, to a pose that is hpwl-WORSE,
and `prune_assignment` reverts it -- so `after` is measured at the restored
input pose and the gain is trivially 0.000. The zeros are prune artifacts, not
no-ops, and the bimodality is therefore partly STRUCTURAL: anything that
survives prune has improved hpwl by construction.

`moved` is printed rather than described because the description was wrong too.
The displacements are NOT uniformly large -- on the recorded run they run 0.14
to 63.79 mm, and two of the nine reverted rows (tigard U5, U6) move well under a
millimetre, which are exactly the micro-shuffles the paragraph above says the
zeros are not. Most are large; some are not; the column says which.

What the sample does support is narrower and still enough: every re-seat whose
gain was NON-ZERO gained at least ~1 mm, so a non-zero default would buy nothing
here while risking a genuine small win.
"""
from __future__ import annotations

import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in (_ROOT, os.path.join(_ROOT, 'py_router'),
           os.path.join(_ROOT, 'py_tools'), os.path.join(_ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import floorplan, reconstruct, seeder         # noqa: E402

BOARDS = ['splitflap_driver.kicad_pcb', 'tigard.kicad_pcb',
          'watchy.kicad_pcb', 'esp_prog.kicad_pcb']
TOPN = 4


def rows_for(name):
    path = os.path.join(_ROOT, 'kicad_files', name)
    if not os.path.exists(path):
        return []
    pcb = parse_kicad_pcb(path)
    intent = floorplan.empty_intent(path)
    ranked = sorted(pcb.footprints.items(),
                    key=lambda kv: (-len(kv[1].pads), kv[0]))
    out = []
    for ref, _fp in ranked[:TOPN]:
        seen = {}
        real = reconstruct.prune_assignment

        def spy(state, old, notes=None, _seen=seen, _real=real, _ref=ref, **kw):
            # The SEARCH's pose, before the sweep can revert it. Without this
            # the zero rows are unreadable.
            _seen['moved'] = max(
                (abs(state.parts[r].x - old[r][0])
                 + abs(state.parts[r].y - old[r][1])) for r in old)
            _seen['pre'] = seeder.scope_hpwl(state, {_ref})
            res = _real(state, old, notes, **kw)
            _seen['pruned'] = list(res)
            return res

        reconstruct.prune_assignment = spy
        try:
            rep = seeder.reseat_scope(parse_kicad_pcb(path), path, intent,
                                      refs=[ref], group_sources=(),
                                      clearance=0.2, board_edge_clearance=0.5,
                                      grid_step=0.1, seed=0)
        except Exception as e:                          # noqa: BLE001
            out.append((name, ref, None, None, None, None, None,
                        f'ERR {type(e).__name__}: {e}'))
            continue
        finally:
            reconstruct.prune_assignment = real
        ab = rep.get('accept_basis') or {}
        t = next((t for t in ab.get('terms') or []
                  if t['term'] == 'scope_hpwl'), {})
        out.append((name, ref, t.get('before'), t.get('after'),
                    seen.get('pre'), seen.get('moved'), seen.get('pruned'),
                    ('ACCEPT ' + str(ab.get('fired'))) if rep['accepted']
                    else 'refuse'))
    return out


def main():
    allrows = []
    for b in BOARDS:
        allrows += rows_for(b)
    print(f"{'board':<28} {'ref':<8} {'hpwl_b':>10} {'hpwl_a':>10} "
          f"{'pre_prune':>10} {'moved':>8} {'gain':>9} {'pruned':>7}"
          f"  verdict")
    gains = []
    for name, ref, b, a, pre, moved, pruned, verdict in allrows:
        if b is None:
            print(f"{name:<28} {ref:<8} {'-':>10} {'-':>10} {'-':>10} "
                  f"{'-':>8} {'-':>9} {'-':>7}  {verdict}")
            continue
        g = b - a
        gains.append(g)
        print(f"{name:<28} {ref:<8} {b:>10.3f} {a:>10.3f} "
              f"{(f'{pre:.3f}' if pre is not None else '-'):>10} "
              f"{(f'{moved:.2f}' if moved is not None else '-'):>8} "
              f"{g:>9.3f} {str(bool(pruned)):>7}  {verdict}")
    pos = sorted(g for g in gains if g > 1e-9)
    zero = [g for g in gains if abs(g) <= 1e-9]
    print(f"\n{len(gains)} explicit re-seats measured on {len(BOARDS)} boards; "
          f"{len(pos)} produced a positive scope_hpwl gain, "
          f"{len(zero)} exactly zero")
    if pos:
        print(f"smallest positive gain: {pos[0]:.3f} mm; "
              f"largest: {pos[-1]:.3f} mm")
    print("The zero rows are reverted seats, not no-ops -- compare "
          "`pre_prune` against `hpwl_b`, read `moved` for how far the search "
          "actually went, and read the module docstring before quoting this.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
