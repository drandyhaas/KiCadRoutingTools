#!/usr/bin/env python3
"""check_connected names an unroutable open apart, and labels an open at art.

A net the BOARD makes unroutable is no routing failure, so it does not count
as an open net; it is named in a line of its own instead:

  1. SHORTED AS DRAWN -- a pad the design already shorts to another net
     (`check_pads.pads_shorted_as_drawn`, route.py's `unroutable_as_designed`:
     another net's pad on it, two parts' pads touching) cannot be joined
     without the short. A net complete without such pads, or holding fewer
     than two others, is named apart. A same-net overlap blocks nothing, and
     a net tie's pad on an ordinary part's copper of the net the tie joins it
     to (a Kelvin tie on its shunt) is the tie's purpose: both nets still
     count (controls). check_drc grades the short itself.
  2. ART ISLAND (#659) -- KiCad links every copper cluster, and net-tagged
     copper art reaching none of the net's pads is one. A route step gives
     such an island a terminal (art_terminals), so the net is ROUTABLE: it
     counts as open, on a line of its own naming it as an art open. A link
     between two LIVE ends counts too, unlabelled (control). The KiCad oracle
     is stubbed here, as the suite image has none.

`ab_replay_grade` records `nets_overlap` outside `nets_incomplete`, and
`nets_art` inside it.

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


def _tie(ref, x, y, net1, net2):
    """A two-pad net tie: pad 1 at (x, y), pad 2 1.5 mm to its left."""
    return ('\t(footprint "Test:NetTie"\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(net_tie_pad_groups "1, 2")\n'
            '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n'
            '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "%s")\n\t\t)\n'
            '\t\t(pad "2" smd rect\n\t\t\t(at -1.5 0)\n\t\t\t(size 1 1)\n'
            '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "%s")\n\t\t)\n\t)\n'
            % (x, y, ref, net1, net2))


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
        # SENSE: a Kelvin net tie NT1 (pads 1 SENSE, 2 PWR, tied) whose pad 1
        # overlaps the shunt RS's PWR pad by design -> routable, still open
        _tie('NT1', 5, 16, 'SENSE', 'PWR'), _pad('RS', 5.3, 16, 'PWR'),
        _pad('R6', 20, 16, 'SENSE'), _pad('R7', 25, 16, 'PWR'),
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
    ok('SENSE (a net tie on its own partner net, by design) stays an open net',
       'SENSE' in text.split('FOUND')[-1] and 'SENSE (' not in line, line.strip())
    ok('OVL is not reported open', 'OVL (' not in text.split('FOUND')[-1])
    ok('ab_replay_grade reads nets_overlap = 1', ab_replay_grade._bucket_count(
        text, r"net\(s\) open only at pads the design shorts") == 1)
    _, incomplete, _ = ab_replay_grade._completion(text)
    ok('nets_incomplete counts SAME and SENSE (and PWR) only', incomplete == 3,
       f'incomplete={incomplete}')


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
    ok('ART is named on the art line', line.strip().startswith('1 net(s)')
       and line.rstrip().endswith('ART'), line.strip())
    ok('ART and LIVE are both open nets (art is routable)',
       names == ['ART', 'LIVE'], f'issues={names}')
    art = next((i for i in issues if i['net_name'] == 'ART'), {})
    ok('the ART issue carries art_only, LIVE does not',
       art.get('art_only') is True and not next(
           i for i in issues if i['net_name'] == 'LIVE').get('art_only'))
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
