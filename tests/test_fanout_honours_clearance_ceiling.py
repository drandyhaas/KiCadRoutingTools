#!/usr/bin/env python3
"""The fanout fronts HONOUR --clearance-ceiling instead of accepting and ignoring it.

    python3 tests/test_fanout_honours_clearance_ceiling.py

`fab_tiers.add_fab_tier_args` registers `--clearance-ceiling` on every tool that
takes `--escalation`, which is `bga_fanout` and `qfn_fanout` too. Only the
ROUTING mains ever read it, so those two ACCEPTED the flag and ignored it.
Nothing warned -- and CLAUDE.md's chain doctrine says to pass the ceiling
rather than a bare `--clearance`, so the RECOMMENDED spelling was the one that
silently did nothing.

MEASURED on butterstick (381-ball 0.8mm BGA), ceiling-only vs the same number
spelled `--clearance`:

    --clearance-ceiling 0.09    3232 grazes, 2 dropped balls, 112 failed plane
                                drops, 3 of 85 GND balls pour-served
    --clearance 0.09              19 grazes, 0 dropped, 85/85 pour-served

and on orangecrab U3 the ceiling-only run left the escape field at
`BGA_CLEARANCE` 0.25 -- the generic default -- rather than the 0.09 asked for.

These fronts route to ONE scalar, so the ceiling's "cap every net class"
reading collapses to `min(requested, ceiling)`: TIGHTEN-ONLY, exactly as it is
for the Default class on the routing mains. The WRITEBACK half already matched
(both pass `clamp_nondefault_netclasses=True` unconditionally), so only the run
clearance was missing.

Rows:
 1. The resolver itself: caps below, never raises above, silent when omitted,
    and handles a None starting clearance.
 2. BOTH fronts call it, and BEFORE anything that reads args.clearance --
    on bga_fanout that means before `enforce_fab_floors`, which would
    otherwise pin the UNCAPPED value up to a tier floor and leave the cap
    applied to a number nothing routed to.
 3. End to end through the real CLI, when a board is available: the ceiling
    reaches the run and is announced.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

QFN = os.path.join(ROOT, 'py_router', 'qfn_fanout', '__init__.py')
BGA = os.path.join(ROOT, 'py_router', 'bga_fanout', '__init__.py')
STRESS = os.path.expanduser('~/Documents/kicad_stress_test')

fails = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f"   [{detail}]" if detail else ''))
    if not ok:
        fails.append(name)


class A:
    """Minimal args stand-in: the resolver reads two attributes."""
    def __init__(self, clearance=None, ceiling=None):
        self.clearance = clearance
        self.clearance_ceiling = ceiling


def main():
    import fab_tiers

    print("1. the resolver: tighten-only, and silent when the flag is omitted")
    fn = getattr(fab_tiers, 'apply_clearance_ceiling', None)
    check("fab_tiers.apply_clearance_ceiling exists", fn is not None,
          '' if fn else "absent: the fronts have nothing to call")
    if fn is None:
        # Do NOT stop here. The fronts ignoring the flag is a SEPARATE defect
        # from the resolver missing, and a battery that returns at the first
        # failure reports one gap where there are three.
        fn = lambda _a, _t='': None      # noqa: E731 -- rows below then fail on their own terms

    a = A(clearance=0.25, ceiling=0.09)
    fn(a, 'probe')
    check("a ceiling BELOW --clearance caps it", abs(a.clearance - 0.09) < 1e-12,
          f"clearance={a.clearance}")

    a = A(clearance=0.09, ceiling=0.2)
    fn(a, 'probe')
    check("a ceiling ABOVE --clearance does NOT raise it (tighten-only)",
          abs(a.clearance - 0.09) < 1e-12, f"clearance={a.clearance}")

    a = A(clearance=0.09, ceiling=None)
    r = fn(a, 'probe')
    check("omitted: returns None and changes nothing",
          r is None and abs(a.clearance - 0.09) < 1e-12, f"ret={r} clearance={a.clearance}")

    a = A(clearance=None, ceiling=0.12)
    fn(a, 'probe')
    check("a None starting clearance takes the ceiling",
          a.clearance is not None and abs(a.clearance - 0.12) < 1e-12,
          f"clearance={a.clearance}")

    print("2. both fronts call it, and early enough to matter")
    for label, path in (('qfn_fanout', QFN), ('bga_fanout', BGA)):
        src = open(path, encoding='utf-8', errors='replace').read()
        check(f"{label} calls apply_clearance_ceiling",
              'apply_clearance_ceiling' in src)
    bsrc = open(BGA, encoding='utf-8', errors='replace').read()
    i_cap = bsrc.find('apply_clearance_ceiling')
    i_floor = bsrc.find('enforce_fab_floors(')
    check("bga_fanout caps BEFORE enforce_fab_floors reads args.clearance",
          i_cap != -1 and i_floor != -1 and i_cap < i_floor,
          f"cap@{i_cap} floors@{i_floor}")

    print("3. end to end, through the real CLI")
    board = None
    for cand in (f'{STRESS}/boards_unrouted_set1/neo6502.kicad_pcb',):
        if os.path.isfile(cand):
            board = cand
            break
    if board is None:
        print("  SKIP: no stress board available -- rows 1/2 still discriminate")
    else:
        import tempfile
        d = tempfile.mkdtemp(prefix='ceil_')
        cmd = [sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'py_router', 'qfn_fanout.py'), board,
               '-c', 'U2', '--output', os.path.join(d, 'o.kicad_pcb'),
               '--clearance-ceiling', '0.09', '--grid-step', '0.05']
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=900, cwd=d)
        out = p.stdout + p.stderr
        m = re.search(r'--clearance-ceiling 0\.09: clearance capped at it', out)
        check("the CLI announces the cap", bool(m),
              'no cap line' if not m else '')
        check("...and names the value it replaced",
              bool(re.search(r'capped at it for this run \(was [0-9.]+\)', out)))

    print(f"\n{len(fails)} failed")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
