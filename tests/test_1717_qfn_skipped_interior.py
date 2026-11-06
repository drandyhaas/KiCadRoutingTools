#!/usr/bin/env python3
"""#1717: every pad a qfn_fanout --nets filter selects and the stub escape
skips as interior is named, also when its pattern selected other pads.

#1641 lists a pattern in `unmatched` only when it selects NO pad; a glob
that selected interior pads AND a perimeter pad ended `failed: 0,
unmatched: []`, exit 0, with the interior pads given no copper and named
nowhere.

On kicad_files/qfn_interior_pads.kicad_pcb (U1: interior GND pads 33/36/37,
perimeter two-pad net FOREIGN on 31/32):

  1. --nets '*' (stub): FOREIGN escapes, `unmatched` stays [] (the pattern
     selected a pad), and `skipped_interior` names U1.33/36/37 and the
     console says so with the underpad hint, while those pads carry no copper.
  2. --nets GND (stub): the per-pattern miss is unchanged and the same pads
     are in `skipped_interior`.
  3. --nets '*' --escape-method underpad: `skipped_interior` is [] (the
     underpad escape tries them; its own failures are in failed).
  4. No --nets: [] -- an unscoped run skips the exposed pad by design and
     never via-drops it.
  5. The engine both fronts call publishes LAST_SKIPPED_INTERIOR.

    python3 tests/test_1717_qfn_skipped_interior.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

QFN = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
INTERIOR = ['U1.33', 'U1.36', 'U1.37']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def summary(out):
    m = re.search(r'^JSON_SUMMARY: (.*)$', out, re.M)
    return json.loads(m.group(1)) if m else None


def copper_pads(path):
    """The U1 pads some copper of the output touches, by its net."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(path))
    live = {s.net_id for s in pcb.segments} | {v.net_id for v in pcb.vias}
    return {f"U1.{p.pad_number}" for p in pcb.footprints['U1'].pads
            if p.net_id and p.net_id in live}


def cli(tmp):
    base = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'qfn_fanout.py')]
    b = os.path.join(tmp, 'b.kicad_pcb')
    shutil.copyfile(QFN, b)

    def run(name, *extra):
        out = os.path.join(tmp, f'{name}.kicad_pcb')
        r = run_check(base + [b, '--component', 'U1', '--output', out] + list(extra),
                      accept=True)
        s = summary(r.stdout)
        if s is None:
            check(f"{name}: a JSON_SUMMARY", False, r.stdout[-400:])
            return {}, r.stdout, out
        return s, r.stdout, out

    print("1. --nets '*', stub escape")
    s, log, out = run('glob', '--nets', '*')
    check("FOREIGN escaped, nothing failed",
          s.get('escaped') == 1 and s.get('failed') == 0,
          f"escaped={s.get('escaped')} failed={s.get('failed')}")
    check("unmatched stays [] (the pattern selected a pad)",
          s.get('unmatched') == [], repr(s.get('unmatched')))
    check("skipped_interior names the three interior pads",
          s.get('skipped_interior') == INTERIOR, repr(s.get('skipped_interior')))
    check("the console names them with the underpad hint",
          'given no copper: U1.33, U1.36, U1.37 are interior' in log
          and '--escape-method underpad reaches them' in log)
    check("and they really carry no copper",
          not (copper_pads(out) & set(INTERIOR)), repr(sorted(copper_pads(out))))

    print("2. --nets GND, stub escape")
    s, _log, _out = run('gnd', '--nets', 'GND')
    miss = s.get('unmatched') or []
    check("the per-pattern interior miss is unchanged",
          len(miss) == 1 and miss[0]['reason'] == 'interior'
          and miss[0]['pads'] == INTERIOR, repr(miss))
    check("skipped_interior names the same pads",
          s.get('skipped_interior') == INTERIOR, repr(s.get('skipped_interior')))

    print("3. --nets '*', underpad escape")
    s, _log, _out = run('under', '--nets', '*', '--escape-method', 'underpad',
                        '--allow-via-in-pad')
    check("skipped_interior is [] (the underpad escape tries them)",
          s.get('skipped_interior') == [], repr(s.get('skipped_interior')))

    print("4. no --nets, stub escape")
    s, log, _out = run('all')
    check("skipped_interior is [] on an unscoped run",
          s.get('skipped_interior') == [] and 'given no copper' not in log,
          repr(s.get('skipped_interior')))


def engine():
    print("5. the engine (the fanout tab's call)")
    import qfn_fanout
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(QFN)
        qfn_fanout.generate_qfn_fanout(
            pcb.footprints['U1'], pcb, net_filter=['*'], layer='F.Cu',
            track_width=0.1, clearance=0.1, grid_step=0.05, escape_method='stub')
    got = list(qfn_fanout.LAST_SKIPPED_INTERIOR)
    check("LAST_SKIPPED_INTERIOR names the interior pads", got == INTERIOR, repr(got))
    check("and no pattern is unmatched",
          list(qfn_fanout.LAST_FILTER_MISSES) == [], repr(qfn_fanout.LAST_FILTER_MISSES))


def main():
    tmp = tempfile.mkdtemp(prefix='t1717_')
    try:
        cli(tmp)
        engine()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
