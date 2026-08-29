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

**Read the `pre_prune` column before drawing the obvious conclusion.** The first
write-up of this measurement said the zero rows were the search "re-seating the
part where it already was". That is false, and this column is what shows it: the
search relocates by tens of millimetres, to a pose that is hpwl-WORSE, and
`prune_assignment` reverts it -- so `after` is measured at the restored input
pose and the gain is trivially 0.000. The zeros are prune artifacts, not
no-ops, and the bimodality is therefore partly STRUCTURAL: anything that
survives prune has improved hpwl by construction.

What the sample does support is narrower and still enough: among the re-seats
that reach the gate at all, the smallest gain is ~1 mm, so a non-zero default
would buy nothing here while risking a genuine small win.
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
            out.append((name, ref, None, None, None, None,
                        f'ERR {type(e).__name__}: {e}'))
            continue
        finally:
            reconstruct.prune_assignment = real
        ab = rep.get('accept_basis') or {}
        t = next((t for t in ab.get('terms') or []
                  if t['term'] == 'scope_hpwl'), {})
        out.append((name, ref, t.get('before'), t.get('after'),
                    seen.get('pre'), seen.get('pruned'),
                    ('ACCEPT ' + str(ab.get('fired'))) if rep['accepted']
                    else 'refuse'))
    return out


def main():
    allrows = []
    for b in BOARDS:
        allrows += rows_for(b)
    print(f"{'board':<28} {'ref':<8} {'hpwl_b':>10} {'hpwl_a':>10} "
          f"{'pre_prune':>10} {'gain':>9} {'pruned':>7}  verdict")
    gains = []
    for name, ref, b, a, pre, pruned, verdict in allrows:
        if b is None:
            print(f"{name:<28} {ref:<8} {'-':>10} {'-':>10} {'-':>10} "
                  f"{'-':>9} {'-':>7}  {verdict}")
            continue
        g = b - a
        gains.append(g)
        print(f"{name:<28} {ref:<8} {b:>10.3f} {a:>10.3f} "
              f"{(f'{pre:.3f}' if pre is not None else '-'):>10} {g:>9.3f} "
              f"{str(bool(pruned)):>7}  {verdict}")
    pos = sorted(g for g in gains if g > 1e-9)
    zero = [g for g in gains if abs(g) <= 1e-9]
    print(f"\n{len(gains)} explicit re-seats measured on {len(BOARDS)} boards; "
          f"{len(pos)} produced a positive scope_hpwl gain, "
          f"{len(zero)} exactly zero")
    if pos:
        print(f"smallest positive gain: {pos[0]:.3f} mm; "
              f"largest: {pos[-1]:.3f} mm")
    print("The zero rows are reverted seats, not no-ops -- compare `pre_prune` "
          "against `hpwl_b` and read the module docstring before quoting this.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
