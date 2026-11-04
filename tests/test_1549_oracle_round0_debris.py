#!/usr/bin/env python3
"""#1549: the oracle's stranded-fragment sweep runs on the round-0 exit.

When KiCad reports every processed net complete, oracle_reconnect leaves its
rounds loop before the first round's body runs, so nothing that body binds
exists. The sweep after the loop still runs whenever the link report names
ANY net (#659: debris is cleared whether or not the weld had work), and on
deleting a fragment it read the board's net dialect off `v10` -- bound only
inside a round that ran. The call died on an unbound local, and because
route.py's plane finalize wraps it in one try, the finalize died with it:
"plane finalize pass FAILED: cannot access local variable 'v10'", every zone
net left unverified by the oracle (a stress board's 3v3 and GND). #678 had
fixed the same exit for `name_to_id`. The sweep now reads the dialect off the
file it edits.

Stubbed link source, no KiCad: the scope net A is complete, and the report
names one link on another net C, from C's pad to a pad-less fragment. On
both net dialects the call must return normally with the fragment deleted
from the file and handed back in removed_segments.

    python3 tests/test_1549_oracle_round0_debris.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

FAILS = []


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name
          + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


PADS = [('R1', 5, 10, 'A'), ('R2', 15, 10, 'A'), ('R3', 32, 32, 'C')]
SEGS = [(5, 10, 15, 10, 'A'),          # A: complete
        (25, 25, 27, 25, 'C')]         # C: the pad-less fragment
NUM = {'A': 1, 'C': 2}


def board(path, by_name):
    """KiCad 10 nets by NAME, or the older numbered net table."""
    def net(n):
        return f'(net "{n}")' if by_name else f'(net {NUM[n]} "{n}")'
    table = '' if by_name else ''.join(
        f'\n  (net {i} "{n}")' for n, i in [('', 0)] + sorted(NUM.items()))
    fps = ''.join(f'''
  (footprint "T:P" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") {net(n)}))'''
                  for ref, x, y, n in PADS)
    segs = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width 0.2) '
                   f'(layer "F.Cu") '
                   + (f'(net "{n}")' if by_name else f'(net {NUM[n]})') + ')'
                   for a, b, c, d, n in SEGS)
    ver = ('(version 20250114) (generator "pcbnew") (generator_version "10.0")'
           if by_name else
           '(version 20241229) (generator "pcbnew") (generator_version "9.0")')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb {ver}
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0)){table}
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts")){fps}{segs}
)
''')


def run(by_name):
    import kicad_oracle
    import kicad_exact_fill
    from kicad_parser import parse_kicad_pcb, board_uses_name_nets
    from routing_config import GridRouteConfig

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        return [('C', (32.0, 32.0, 'F.Cu', 'pad'),
                 (26.0, 25.0, 'F.Cu', 'track'))]

    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    try:
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'b.kicad_pcb')
            board(path, by_name)
            with open(path, encoding='utf-8') as f:
                dialect = board_uses_name_nets(f.read())
            cfg = GridRouteConfig(clearance=0.2, track_width=0.2,
                                  via_size=0.6, via_drill=0.3,
                                  layers=['F.Cu', 'B.Cu'])
            out = io.StringIO()
            err = None
            res = None
            try:
                with contextlib.redirect_stdout(out):
                    res = kicad_oracle.oracle_reconnect(
                        path, ['A'], cfg, track_via_clearance=0.2,
                        hole_to_hole_clearance=0.2, max_rounds=3)
            except Exception as e:                          # noqa: BLE001
                err = e
            left = [(s.start_x, s.end_x) for s in parse_kicad_pcb(path).segments]
    finally:
        (kicad_exact_fill.exact_unconnected,
         kicad_oracle.kicad_unconnected) = saved
    return dialect, out.getvalue(), err, res, left


def main():
    for by_name, label in ((True, 'nets by name (KiCad 10)'),
                           (False, 'numbered nets')):
        print(f"--- {label}")
        dialect, log, err, res, left = run(by_name)
        check("fixture: the board is in the intended dialect",
              dialect == by_name, f"board_uses_name_nets={dialect}")
        round0 = 'all processed nets complete' in log
        check("fixture: the call takes the round-0 exit", round0,
              '' if round0 else log.strip()[-300:])
        check("the call returns instead of raising", err is None, repr(err))
        if err is not None or res is None:
            continue
        check("it ran no round", res.get('rounds') == 0, str(res.get('rounds')))
        rem = res.get('removed_segments') or []
        check("the fragment comes back in removed_segments",
              [(s.start_x, s.end_x) for s in rem] == [(25.0, 27.0)],
              str([(s.start_x, s.end_x) for s in rem]))
        check("and is gone from the file, A's copper kept",
              left == [(5.0, 15.0)], str(left))
    if FAILS:
        print(f"\nFAIL: {len(FAILS)} check(s): {FAILS}")
        sys.exit(1)
    print("\nALL PASS")


if __name__ == '__main__':
    main()
