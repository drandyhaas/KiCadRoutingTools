#!/usr/bin/env python3
"""check_connected names two more unroutable opens apart, outside the open tally.

A net the BOARD makes unroutable is no routing failure, so it does not count
as an open net; it is named in a line of its own instead:

  1. SHORTED AS DRAWN -- a pad the design already shorts to another net
     (`check_pads.pads_shorted_as_drawn`, route.py's `unroutable_as_designed`:
     another net's pad on it, two parts' pads touching) cannot be joined
     without the short. A net complete without such pads, or holding fewer
     than two others, is named apart. A same-net overlap blocks nothing, so
     its net still counts (control). check_drc grades the short itself.
  2. ART ISLAND (#659) -- KiCad links every copper cluster, and net-tagged
     copper art reaching none of the net's pads is one: the open is real but
     the art is immutable and no routing pass targets it. A net whose every
     KiCad-only link has such an art end is named apart; a link between two
     LIVE ends (copper our grade joins and KiCad does not) still counts
     (control). The KiCad oracle is stubbed here, as the suite image has none.

`ab_replay_grade` records the two as `nets_overlap` and `nets_art`, outside
`nets_incomplete`.

    python3 tests/test_unroutable_open_buckets.py
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(TESTS_DIR, 'stress'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402
import ab_replay_grade                                           # noqa: E402

failures = []

HEAD = '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t)
'''


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def _pad(ref, x, y, net, size=1.0):
    net_s = '\t\t\t(net "%s")\n' % net if net else ''
    return ('\t(footprint "Test:Pad"\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size %g %g)\n'
            '\t\t\t(layers "F.Cu" "F.Mask")\n%s\t\t)\n\t)\n'
            % (x, y, ref, size, size, net_s))


def _track(x1, y1, x2, y2, net):
    return ('\t(segment\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(width 0.25)\n'
            '\t\t(layer "F.Cu")\n\t\t(net "%s")\n\t)\n' % (x1, y1, x2, y2, net))


def _art(x1, y1, x2, y2, net):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.3)\n\t\t\t(type solid)\n\t\t)\n'
            '\t\t(layer "F.Cu")\n\t\t(net "%s")\n\t)\n' % (x1, y1, x2, y2, net))


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def _zone(net, ring):
    pts = ' '.join('(xy %g %g)' % p for p in ring)
    return ('\t(zone\n\t\t(net "%s")\n\t\t(layer "B.Cu")\n'
            '\t\t(uuid "11111111-0000-4000-8000-0000000000aa")\n'
            '\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0.5)\n\t\t)\n'
            '\t\t(min_thickness 0.25)\n\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n'
            '\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)\n' % (net, pts))


def write_board(path, parts):
    corners = [(0, 0), (30, 0), (30, 20), (0, 20)]
    body = [HEAD] + parts
    for i in range(4):
        body.append(_edge(*corners[i], *corners[(i + 1) % 4]))
    body.append(')\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(body))
    return run_utils.evidence(path, 'unroutable-bucket board')


def check_overlap(td):
    print('design overlap:')
    board = write_board(os.path.join(td, 'overlap.kicad_pcb'), [
        # OVL: R1 lies under P1, a one-pad net OTHER's pad -> bucketed
        _pad('R1', 5, 4, 'OVL'), _pad('R2', 20, 4, 'OVL'), _pad('P1', 5.3, 4, 'OTHER'),
        # SAME: R3 overlaps a pad of its OWN net -> blocks nothing, still open
        _pad('R3', 5, 10, 'SAME'), _pad('R4', 20, 10, 'SAME'), _pad('R5', 5.3, 10, 'SAME'),
    ])
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'), board],
                       capture_output=True, text=True, cwd=ROOT)
    text = r.stdout + r.stderr
    ok('check_connected ran', 'Traceback' not in text, text[-400:])
    line = next((ln for ln in text.splitlines() if 'pads the design shorts' in ln), '')
    ok('OVL is named in the overlap bucket, with the pad pair',
       line.strip().startswith('1 net(s)') and 'OVL (R1.1 x P1.1)' in line, line.strip())
    ok('SAME stays an open net', 'SAME' in text.split('FOUND')[-1] if 'FOUND' in text else False)
    ok('OVL is not reported open', 'OVL (' not in text.split('FOUND')[-1])
    ok('ab_replay_grade reads nets_overlap = 1', ab_replay_grade._bucket_count(
        text, r"net\(s\) open only at pads the design shorts") == 1)
    _, incomplete, _ = ab_replay_grade._completion(text)
    ok('nets_incomplete counts SAME only', incomplete == 1, f'incomplete={incomplete}')


def check_art(td):
    print('art island (#659), KiCad oracle stubbed:')
    board = write_board(os.path.join(td, 'art.kicad_pcb'), [
        # ART: complete in copper; a net-tagged art stroke touching nothing
        _pad('R1', 5, 4, 'ART'), _pad('R2', 20, 4, 'ART'), _track(5, 4, 20, 4, 'ART'),
        _art(5, 12, 10, 12, 'ART'),
        # LIVE: complete in copper; KiCad reports a link between two live ends
        _pad('R3', 5, 16, 'LIVE'), _pad('R4', 20, 16, 'LIVE'), _track(5, 16, 20, 16, 'LIVE'),
        # a pour, so the KiCad reconcile runs at all
        _pad('R5', 25, 8, 'GND'), _pad('R6', 25, 12, 'GND'),
        _zone('GND', [(22, 6), (29, 6), (29, 14), (22, 14)]),
    ])
    import kicad_oracle
    from check_connected import run_connectivity_check
    links = [('ART', (20.0, 4.0, None, 'pad'), (7.5, 12.0, 'F.Cu', 'track')),
             ('LIVE', (5.0, 16.0, None, 'pad'), (20.0, 16.0, None, 'pad'))]
    saved = (kicad_oracle.find_kicad_cli, kicad_oracle.kicad_unconnected)
    kicad_oracle.find_kicad_cli = lambda *a, **k: '/stub/kicad-cli'
    kicad_oracle.kicad_unconnected = lambda *a, **k: list(links)
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            issues = run_connectivity_check(board)
    finally:
        kicad_oracle.find_kicad_cli, kicad_oracle.kicad_unconnected = saved
    text = buf.getvalue()
    names = sorted(i['net_name'] for i in issues)
    line = next((ln for ln in text.splitlines() if 'net-tagged copper art' in ln), '')
    ok('ART is named in the art bucket', line.strip().startswith('1 net(s)')
       and line.rstrip().endswith('ART'), line.strip())
    ok('LIVE (two live ends) stays an open net, ART does not',
       names == ['LIVE'], f'issues={names}')
    ok('ab_replay_grade reads nets_art = 1', ab_replay_grade._bucket_count(
        text, r"net\(s\) open in KiCad only where net-tagged copper art") == 1)


def main():
    with tempfile.TemporaryDirectory() as td:
        check_overlap(td)
        check_art(td)
    print('PASS' if not failures else f'FAIL ({len(failures)}): {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
