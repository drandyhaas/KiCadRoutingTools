#!/usr/bin/env python3
"""#1737: check_drc reports a net-tie contact when KiCad does.

Measured on kicad-cli 10.0.6: at a net-tie PARTNER pad that is not a plain
rect (a roundrect, as Kelvin-shunt footprints draw it), KiCad waives the tied
net's contact on its own pad only when the track's uuid sorts BEFORE the
partner pad's; when it sorts after, KiCad reports the contact as
shorting_items, even a landing wholly on the own pad. A rect partner is
waived in both orders. check_drc used to waive every such contact, so a
shunt landing graded clean while KiCad reported a shorting_items at it.

Fixture: shunt R1, VBUS pad 1 (roundrect, or rect for the control) with the
SENSE pad 2 NESTED in it (pad 2's copper lies wholly on pad 1, so no landing
avoids the partner), tied by net_tie_pad_groups; and the same with pad 2
straddling pad 1's edge. One SENSE landing, written with a uuid below or
above pad 1's.

  1. The text parser reads each pad's own uuid.
  2. Nested, uuid below: no row. Nested, uuid above: one accepted
     `net-tie-order` row (footprint-inherent: no landing avoids the
     contact), 0 violations, JSON `net_tie_order` 1.
  3. Straddling, uuid above: a pad-segment violation (a landing clear of
     the partner exists); uuid below: waived.
  4. A rect partner, uuid above: waived (nested and straddling).
  5. With kicad-cli on PATH or at the macOS app path: KiCad reports an item
     at R1 in exactly the cases check_drc names or counts (skipped without
     it).

    python3 tests/test_1737_net_tie_landing_order.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

PARTNER_UUID = '80000000-0000-4000-8000-000000000001'
BEFORE, AFTER = '10000000-0000-4000-8000-000000000002', 'f0000000-0000-4000-8000-000000000003'
BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "VBUS")
  (net 2 "SENSE")
  (footprint "test:kelvin_shunt" (layer "F.Cu")
    (uuid "20000000-0000-4000-8000-000000000004")
    (at 10 10)
    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (uuid "20000000-0000-4000-8000-000000000005") (effects (font (size 1 1) (thickness 0.15))))
    (net_tie_pad_groups "1, 2")
    (attr smd)
    (pad "1" smd {shape} (at 0 0.325) (size 0.95 0.8){rratio} (layers "F.Cu") (net 1 "VBUS") (uuid "{partner}"))
    (pad "2" smd rect (at 0 {sense_y}) (size 0.5 0.15) (layers "F.Cu") (net 2 "SENSE") (uuid "30000000-0000-4000-8000-000000000006"))
  )
  (segment (start 10 8.5) (end 10 {land_y}) (width 0.1) (layer "F.Cu") (net 2) (uuid "{seg}"))
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)) (uuid "20000000-0000-4000-8000-000000000007"))
)
"""
# nested: pad 2 spans y 9.925..10.075, on pad 1 (y 9.925..10.725); the
# landing ends at its centre. straddle: pad 2 spans y 9.875..10.025, across
# pad 1's edge; the landing's copper reaches y 10.0, so it touches pad 1
# only where it lies on pad 2 (the waiver's own case).
CASES = {'nested': (0, 10.0), 'straddle': (-0.05, 9.95)}
SHAPES = {'roundrect': ('roundrect', ' (roundrect_rratio 0.25)'), 'rect': ('rect', '')}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(tmp, case, seg_uuid, shape='roundrect'):
    sense_y, land_y = CASES[case]
    path = os.path.join(tmp, f'{shape}_{case}_{seg_uuid[:1]}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD.format(partner=PARTNER_UUID, sense_y=sense_y, land_y=land_y,
                             seg=seg_uuid, shape=SHAPES[shape][0], rratio=SHAPES[shape][1]))
    return evidence(path)


def grade(path, flagged=False):
    """check_drc.py's JSON for PATH: (violations, net_tie_order, by_type).
    FLAGGED: the run is expected to report a pad-segment violation (exit 1)."""
    js = path[:-len('.kicad_pcb')] + '.drc.json'
    argv = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
            path, '--clearance', '0.1', '--json', js]
    if flagged:
        run_check(argv, refuse='PAD-SEGMENT', code=1)
    else:
        run_check(argv, accept=True)
    with open(evidence(js), encoding='utf-8') as f:
        d = json.load(f)
    return d['violations'], d.get('net_tie_order'), d.get('by_type', {})


def kicad_cli():
    for c in (shutil.which('kicad-cli'),
              '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'):
        if c and os.path.exists(c):
            return c
    return None


def kicad_items(cli, path):
    js = path[:-len('.kicad_pcb')] + '.kicad.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', js, path],
                   capture_output=True)
    with open(evidence(js), encoding='utf-8') as f:
        return [v['type'] for v in json.load(f)['violations']
                if v['type'] in ('shorting_items', 'clearance')
                and any('of R1' in i['description'] for i in v['items'])]


def main():
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory(prefix='t1737_') as tmp:
        print("1. the parser")
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(board(tmp, 'nested', BEFORE))
        uu = {p.pad_number: getattr(p, 'uuid', None) for p in pcb.footprints['R1'].pads}
        check("each pad's own uuid is read", uu.get('1') == PARTNER_UUID
              and (uu.get('2') or '').startswith('30000000'), repr(uu))

        print("2. a landing on a pad nested in its roundrect partner")
        v, n, _ = grade(board(tmp, 'nested', BEFORE))
        check("uuid below the partner's: no row", v == 0 and n == 0,
              f"violations={v} net_tie_order={n}")
        v, n, _ = grade(board(tmp, 'nested', AFTER))
        check("uuid above: one accepted net-tie-order row, no violation",
              v == 0 and n == 1, f"violations={v} net_tie_order={n}")

        print("3. a tie pad straddling its roundrect partner's edge")
        v, n, by = grade(board(tmp, 'straddle', AFTER), flagged=True)
        check("uuid above: a pad-segment violation", v >= 1 and n == 0
              and by.get('pad-segment', 0) >= 1, f"violations={v} by_type={by}")
        v, n, _ = grade(board(tmp, 'straddle', BEFORE))
        check("uuid below: waived", v == 0 and n == 0, f"violations={v} net_tie_order={n}")

        print("4. a rect partner")
        for case in CASES:
            v, n, _ = grade(board(tmp, case, AFTER, 'rect'))
            check(f"{case}, uuid above: waived", v == 0 and n == 0,
                  f"violations={v} net_tie_order={n}")

        cli = kicad_cli()
        print("5. KiCad" + ('' if cli else ': kicad-cli not found, skipped'))
        if cli:
            for shape in SHAPES:
                for case in CASES:
                    for u in (BEFORE, AFTER):
                        flagged = shape == 'roundrect' and u == AFTER
                        got = kicad_items(cli, board(tmp, case, u, shape))
                        check(f"{shape} {case}, uuid {'above' if u == AFTER else 'below'}: "
                              f"KiCad {'reports' if flagged else 'accepts'} it",
                              bool(got) == flagged, repr(got))
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
