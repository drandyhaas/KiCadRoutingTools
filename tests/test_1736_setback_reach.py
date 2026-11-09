#!/usr/bin/env python3
"""A pad-edge launch has far setback rungs, tried after the whole ladder.

A diff pair launching from a long bare pad starts at the pad's EDGE (#165),
and its setback is pulled in by that launch shift so the setback point stands
where a centre launch would put it (#166). The ladder's outer rungs (#90),
1.5x and 2x, are scaled off the pulled-in setback, so they reach far less past
the pad than a centre launch's do: with a 0.68 mm setback pulled in by
0.525 mm to 0.155 mm, the rungs are 0.23 and 0.31 mm -- both under the
geometric floor -- and the whole ladder stops 0.34 mm past the pad edge. A
pair whose members are priced at their layer width (the obstacle stamps of an
impedance run) clears the pads beside its own only farther out, and found no
launch at any setback.

Such a launch gets two FAR rungs where a centre launch's 1.5x and 2x stand.
They are tried only once the ladder has failed straight AND rotated: tried
with the ladder, a far launch beat the rotated launch that routed a pair
better (measured on a corpus board: a pair looped, with vias beside the
connector it ran to, where the rotated launch ran straight).

Rows:
  - a pulled-in setback: the ladder is the one it always was, and the far
    rungs stand at 1.5x and 2x of the configured setback from the pad CENTRE;
  - no launch shift: no far rungs;
  - the laddered search: a terminal that opens both on a rotated ladder rung
    and on a far rung straight ahead launches rotated, on the ladder; one
    that opens only on a far rung launches there.

    python3 tests/test_1736_setback_reach.py
"""
import inspect
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

from routing_config import GridRouteConfig   # noqa: E402
import diff_pair_routing as dpr              # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def split(res):
    """(ladder, far) on whichever return the code has: a ladder with no far
    rungs of its own is still measured."""
    if isinstance(res, tuple):
        return list(res[0]), list(res[1])
    return list(res), []


def ladder_of(setback, spacing, pad_gap_half, cfg, shift):
    if 'launch_shift' in inspect.signature(dpr._setback_ladder).parameters:
        return split(dpr._setback_ladder(setback, spacing, pad_gap_half, cfg,
                                         launch_shift=shift))
    return split(dpr._setback_ladder(setback, spacing, pad_gap_half, cfg))


def plain_ladder(setback, spacing, pad_gap_half, cfg):
    floor = max(2 * spacing, abs(pad_gap_half - spacing) + cfg.grid_step)
    want = [setback]
    for s in (0.75 * setback, 0.5 * setback, floor, 1.5 * setback, 2 * setback):
        s = max(s, floor)
        if all(abs(s - e) > 1e-6 for e in want):
            want.append(s)
    return want


def search(cfg, setback, shift, opens):
    """Run the laddered search with `_find_open_positions` replaced by
    `opens(rotated_deg, s)` -> bool; returns (setback used, rotated)."""
    real = dpr._find_open_positions

    def fake(cx, cy, dx, dy, layer_idx, s, *a, **k):
        deg = round(math.degrees(math.atan2(dy, dx))) % 360
        return [(0, 0)] if opens(deg, s) else []
    dpr._find_open_positions = fake
    kw = {'launch_shift': shift} if 'launch_shift' in inspect.signature(
        dpr._find_open_positions_laddered).parameters else {}
    try:
        cands, used, rotated = dpr._find_open_positions_laddered(
            0.0, 0.0, 1.0, 0.0, 0, setback, 0.25, 'source', ['F.Cu'], 0.17, cfg,
            None, None, None, [], quiet=True, **kw)
    finally:
        dpr._find_open_positions = real
    return (used if cands else None), rotated


def main():
    cfg = GridRouteConfig(track_width=0.15, clearance=0.15, grid_step=0.05,
                          diff_pair_gap=0.19, layers=['F.Cu', 'B.Cu'])
    spacing = (0.15 + 0.19) / 2          # the call's half-pitch, 0.17
    configured = 4 * spacing             # the default setback, 0.68
    shift = 0.525                        # a pad-edge launch on a long pad
    pulled = configured - shift          # what #166 routes rung 1 at
    pad_gap_half = 0.25

    ladder, far = ladder_of(pulled, spacing, pad_gap_half, cfg, shift)
    print(f"  pulled in by {shift} mm: ladder {[round(s, 3) for s in ladder]}, "
          f"far {[round(s, 3) for s in far]}")
    check("the ladder of a pulled-in launch is the one it always was",
          [round(s, 9) for s in ladder] ==
          [round(s, 9) for s in plain_ladder(pulled, spacing, pad_gap_half, cfg)])
    for w in (1.5 * configured - shift, 2 * configured - shift):
        check(f"a far rung stands {w:.3f} mm past the pad edge "
              f"({w + shift:.2f} mm from its centre, as a centre launch's)",
              any(abs(s - w) < 1e-6 for s in far),
              f"far rungs {[round(s, 3) for s in far]}")

    lad0, far0 = ladder_of(configured, spacing, pad_gap_half, cfg, 0.0)
    check("a centre launch has no far rungs, and its ladder is unchanged",
          not far0 and [round(s, 9) for s in lad0] ==
          [round(s, 9) for s in plain_ladder(configured, spacing, pad_gap_half, cfg)],
          f"{[round(s, 3) for s in lad0]} far {far0}")

    far_rung = 2 * configured - shift
    on_far = lambda s: abs(s - far_rung) < 1e-6   # noqa: E731
    # opens rotated 90 deg at the ladder's first rung, and straight at a far rung
    used, rotated = search(cfg, pulled, shift,
                           lambda deg, s: (deg == 90 and abs(s - pulled) < 1e-6)
                           or (deg == 0 and on_far(s)))
    check("a rotated ladder launch wins over a straight far one",
          used is not None and rotated and abs(used - pulled) < 1e-6,
          f"used {used}, rotated {rotated}")
    # opens only straight at a far rung
    used, rotated = search(cfg, pulled, shift, lambda deg, s: deg == 0 and on_far(s))
    check("a terminal that opens only at a far rung launches there",
          used is not None and not rotated and abs(used - far_rung) < 1e-6,
          f"used {used}, rotated {rotated}")

    if FAILS:
        print(f"\nFAIL: {len(FAILS)} row(s): {FAILS}")
        return 1
    print("\nfar setback rungs reach where a centre launch's do, after the ladder. OK")
    return 0


if __name__ == '__main__':
    sys.exit(main())
