#!/usr/bin/env python3
"""#1717: a qfn_fanout stub run via-drops the interior pads its --nets filter
selects, and names each one it could not serve.

The stub escape used to drop every interior pad before counting it, and
#1641's disclosure named a pattern only when it selected NO pad, so a glob
that selected interior pads AND perimeter pads ended `failed: 0,
unmatched: []`, exit 0, with the interior pads given no copper and named
nowhere. Now a net-scoped stub run gives those pads the underpad escape's
via-drop after its surface fan; a pad the via-drop cannot place is named in
JSON_SUMMARY `skipped_interior` (one console WARNING) and its net is failed,
as an underpad run fails it. An unscoped run still never via-drops the
exposed pad.

On kicad_files/qfn_interior_pads.kicad_pcb (U1: interior GND pads 33/36/37,
perimeter two-pad net FOREIGN on 31/32), and on a copy with a partner part
that gives the interior pads 34/35 (INT1, INT2) and perimeter pads 23/24 a
second pad each:

  1. partner copy, --nets '*' '!GND' --allow-via-in-pad: INT1/INT2 get a via
     and copper, the perimeter nets escape, nothing failed or skipped.
  2. partner copy, the same without via-in-pad: the via-drop finds no site
     for U1.34/35, so they are named in `skipped_interior` and their nets
     are failed, with the console line.
  3. --nets '*' on the tracked board: GND's pads are tried; the two the
     via-drop cannot place are named, GND is failed, FOREIGN escapes.
  4. No --nets: no via, no interior copper, nothing named.
  5. Every output grades DRC-clean, and the engine both fronts call
     publishes LAST_SKIPPED_INTERIOR.

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
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def summary(out):
    m = re.search(r'^JSON_SUMMARY: (.*)$', out, re.M)
    return json.loads(m.group(1)) if m else None


def with_partner(dst):
    """The tracked board plus a part giving INT1, INT2, DP1_P and DP1_N a
    second pad, so all four are multi-pad signal nets."""
    with open(QFN, encoding='utf-8') as f:
        text = f.read()
    rows = ''.join(
        f'    (pad "{i}" smd rect (at {x} 0) (size 0.6 0.6) (layers "F.Cu") (net "{n}"))\n'
        for i, (x, n) in enumerate([(-3, 'INT1'), (-1, 'INT2'), (1, 'DP1_P'),
                                    (3, 'DP1_N')], 1))
    part = ('  (footprint "fx:partner" (layer "F.Cu") (at 11 18.5)\n'
            '    (property "Reference" "P1" (at 0 -1.5 0) (layer "F.SilkS") (hide yes)'
            ' (effects (font (size 1 1) (thickness 0.15))))\n'
            '    (attr smd)\n' + rows + '  )\n')
    k = text.rindex('\t(embedded_fonts no)')
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(text[:k] + part + text[k:])
    return evidence(dst)


def served(path):
    """{U1 pad: True when some copper of its net lands on it} and the via count."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(path))
    ends = {}
    for s in pcb.segments:
        ends.setdefault(s.net_id, []).extend([(s.start_x, s.start_y), (s.end_x, s.end_y)])
    for v in pcb.vias:
        ends.setdefault(v.net_id, []).append((v.x, v.y))
    out = {}
    for p in pcb.footprints['U1'].pads:
        hx, hy = max(p.size_x, p.size_y) / 2, max(p.size_x, p.size_y) / 2
        out[f"U1.{p.pad_number}"] = any(
            abs(x - p.global_x) <= hx and abs(y - p.global_y) <= hy
            for x, y in ends.get(p.net_id, ()))
    return out, len(pcb.vias)


def drc_clean(path):
    """check_drc.py's verdict on PATH: [] when clean, else its tail."""
    r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                   evidence(path), '--clearance', '0.1'], accept=True)
    return [] if 'NO DRC VIOLATIONS FOUND' in r.stdout else r.stdout.splitlines()[-6:]


def cli(tmp):
    base = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'qfn_fanout.py')]
    b = os.path.join(tmp, 'b.kicad_pcb')
    shutil.copyfile(QFN, b)
    bp = with_partner(os.path.join(tmp, 'bp.kicad_pcb'))
    outs = []

    def run(board, name, *extra):
        out = os.path.join(tmp, f'{name}.kicad_pcb')
        r = run_check(base + [board, '--component', 'U1', '--output', out] + list(extra),
                      accept=True)
        s = summary(r.stdout)
        if s is None:
            check(f"{name}: a JSON_SUMMARY", False, r.stdout[-400:])
            return {}, r.stdout, out
        outs.append(out)
        return s, r.stdout, out

    print("1. interior signal pads, via-in-pad allowed")
    s, _log, out = run(bp, 'vip', '--nets', '*', '!GND', '--allow-via-in-pad')
    got, nvias = served(out)
    check("INT1/INT2 escaped with the perimeter nets, nothing failed",
          s.get('failed') == 0 and s.get('escaped') == s.get('requested') == 5,
          f"requested={s.get('requested')} escaped={s.get('escaped')} failed={s.get('failed')}")
    check("U1.34 and U1.35 carry copper, through a via",
          got.get('U1.34') and got.get('U1.35') and nvias >= 2, f"{nvias} via(s)")
    check("nothing is named in skipped_interior",
          s.get('skipped_interior') == [], repr(s.get('skipped_interior')))

    print("2. interior signal pads, no via-in-pad")
    s, log, _out = run(bp, 'novip', '--nets', '*', '!GND')
    check("the unplaced pads are named", s.get('skipped_interior') == ['U1.34', 'U1.35'],
          repr(s.get('skipped_interior')))
    check("and their nets are failed",
          s.get('unescaped_nets') == ['INT1', 'INT2'] and s.get('failed') == 2,
          f"failed={s.get('failed')} {s.get('unescaped_nets')}")
    check("the console says so", 'interior pad(s) given no copper: U1.34, U1.35' in log)

    print("3. --nets '*' on the tracked board")
    s, _log, _out = run(b, 'glob', '--nets', '*')
    check("GND's two unplaced pads are named, GND failed, FOREIGN escaped",
          s.get('skipped_interior') == ['U1.36', 'U1.37']
          and s.get('unescaped_nets') == ['GND'] and s.get('escaped') == 1,
          f"{s.get('skipped_interior')} {s.get('unescaped_nets')} escaped={s.get('escaped')}")
    check("unmatched stays []", s.get('unmatched') == [], repr(s.get('unmatched')))

    print("4. no --nets")
    s, log, out = run(b, 'all')
    got, nvias = served(out)
    check("no via, no interior copper, nothing named",
          nvias == 0 and not any(got[p] for p in ('U1.33', 'U1.36', 'U1.37'))
          and s.get('skipped_interior') == [] and 'given no copper' not in log,
          f"{nvias} via(s), {s.get('skipped_interior')}")

    print("5. every output grades DRC-clean")
    for out in outs:
        rows = drc_clean(out)
        check(f"{os.path.basename(out)}: no DRC row", rows == [], repr(rows[:4]))


def engine():
    print("6. the engine (the fanout tab's call)")
    import qfn_fanout
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(QFN)
        _t, vias, failed = qfn_fanout.generate_qfn_fanout(
            pcb.footprints['U1'], pcb, net_filter=['*'], layer='F.Cu',
            track_width=0.1, clearance=0.1, grid_step=0.05, escape_method='stub',
            via_size=0.45, via_drill=0.25)
    got = list(qfn_fanout.LAST_SKIPPED_INTERIOR)
    check("LAST_SKIPPED_INTERIOR names the unplaced pads", got == ['U1.36', 'U1.37'], repr(got))
    check("the via-drop's via is returned and GND is failed",
          len(vias) == 1 and failed == ['GND'], f"{len(vias)} via(s), failed {failed}")


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
