#!/usr/bin/env python3
"""#1232: pad solder-mask openings reach PCBData, and check_via_mask grades vias against them.

TEXT parse path only; tests/gui_parity/test_1232_mask_parity.py gates the
pcbnew path against pcbnew's own GetSolderMaskExpansion.

1. Mask-margin precedence, pinned to pcbnew 10.0.7's resolved expansion
   (probed on boards built exactly like these): pad, then footprint, then
   board `pad_to_mask_clearance`; an explicit 0 is unset in a 20240108 file
   and an override in a 20240202 one; a negative margin clamps at
   -min(size)/2; a pad listing no copper layer gets 0; `*.Mask` opens both
   sides.
2. via-mask-contact: a via ring reaching into a pad's opening, centre outside
   the copper and barrel clear of the paste opening; NOT for the same via
   moved clear, NOT against a pad whose same-net paste opening holds the
   barrel (check_drc's via-in-paste owns that one), but STILL against a
   foreign-net pad beside it on either side; NOT for a via centred in the pad.
   An opening KiCad shapes differently (a trapezoid) says so in
   `approximation`.
3. A pad's own margin moves the verdict: margin 0 opens no band around the
   copper, so the same via is clean. A positive margin rounds the opening's
   corners at the margin, as KiCad plots it (a RoundRect aperture): a via off
   the pad's corner that a square-cornered growth would reach is clean, and
   moved in by 0.03 mm it is a contact.
4. via-mask-web: a soldermask-plugged via within the web of a pad opening;
   the same via unplugged is clean; --plug-web moves the limit.
5. --baseline: a finding the input already had is accepted, a new one is not.
6. The CLI: exit 1 on a finding, 0 when all are accepted, and --json counts.

    python3 tests/test_1232_via_mask.py
"""

import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from mask_apertures import resolve_mask_margin, pad_mask_layers  # noqa: E402
from check_via_mask import via_mask_findings, split_inherited, counts  # noqa: E402

RUN_ALL_FAST_OK = True

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _board(body, setup='', version=20240202):
    return ('(kicad_pcb (version %d) (generator "t")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (35 "F.Paste" user)'
            ' (34 "B.Paste" user) (39 "F.Mask" user) (38 "B.Mask" user)'
            ' (44 "Edge.Cuts" user))\n'
            ' (setup %s)\n'
            ' (net 0 "")\n (net 1 "/A")\n (net 2 "/B")\n%s\n)'
            % (version, setup, body))


def _fp(pad_extra='', header='', layers='"F.Cu" "F.Paste" "F.Mask"',
        size='1 0.5', kind='smd rect', net='(net 1 "/A")'):
    return ('(footprint "L:P" (layer "F.Cu") (at 100 100)\n'
            '   (property "Reference" "U1")\n%s\n'
            '   (pad "1" %s (at 0 0) (size %s) (layers %s) %s %s))'
            % (header, kind, size, layers, net, pad_extra))


def _via(x, y, net=1, extra=''):
    return ('(via (at %g %g) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") '
            '(net %d) %s)' % (x, y, net, extra))


def _parse(text):
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    with open(path, 'w') as fh:
        fh.write(text)
    try:
        return parse_kicad_pcb(path), path
    except Exception:
        os.remove(path)
        raise


def _margin(text):
    pcb, path = _parse(text)
    os.remove(path)
    fp = pcb.footprints['U1']
    return resolve_mask_margin(fp.pads[0], fp, pcb.board_info)


def main():
    # ---- 1. precedence, as pcbnew 10.0.7 resolves it -------------------------
    B = '(pad_to_mask_clearance 0.05)'
    rows = [  # (version, board, footprint header, pad extra, expected)
        (20240202, '', '', '', 0.0),
        (20240202, B, '', '', 0.05),
        (20240202, B, '(solder_mask_margin 0.07)', '', 0.07),
        (20240202, B, '(solder_mask_margin 0.07)', '(solder_mask_margin 0.11)', 0.11),
        (20240202, B, '(solder_mask_margin 0.07)', '(solder_mask_margin 0)', 0.0),
        (20240202, B, '(solder_mask_margin 0)', '', 0.0),
        (20240108, B, '(solder_mask_margin 0.07)', '(solder_mask_margin 0)', 0.07),
        (20240108, B, '(solder_mask_margin 0)', '', 0.05),
        (20240108, B, '(solder_mask_margin 0)', '(solder_mask_margin 0)', 0.05),
    ]
    for ver, setup, header, extra, want in rows:
        got = _margin(_board(_fp(pad_extra=extra, header=header), setup, ver))
        check(f"1. v{ver} board={setup or '-'} fp={header or '-'} "
              f"pad={extra or '-'} -> {want}", abs(got - want) < 1e-9, str(got))
    got = _margin(_board(_fp(pad_extra='(solder_mask_margin -0.1)'), B))
    check('1. a negative margin applies', abs(got + 0.1) < 1e-9, str(got))
    got = _margin(_board(_fp(pad_extra='(solder_mask_margin -0.4)'), B))
    check('1. a negative margin clamps at -min(size)/2', abs(got + 0.25) < 1e-9, str(got))
    got = _margin(_board(_fp(pad_extra='(solder_mask_margin 0.1)', layers='"F.Mask"'), B))
    check('1. a pad listing no copper layer gets 0', got == 0.0, str(got))
    pcb, path = _parse(_board(_fp(layers='"*.Cu" "*.Mask"', kind='thru_hole circle',
                                  size='1 1', pad_extra='(drill 0.5)'), B))
    os.remove(path)
    check('1. *.Mask opens both sides',
          pad_mask_layers(pcb.footprints['U1'].pads[0]) == ['F.Mask', 'B.Mask'])
    check('1. the board setup is read', pcb.board_info.pad_to_mask_clearance == 0.05)

    # ---- 2. via-mask-contact --------------------------------------------------
    # The pad's copper edge is x = 100.5; with a 0.1 margin the opening reaches
    # 100.6. A 0.5 via at x = 100.8 has its ring at 100.55: inside the
    # opening, 0.05 clear of the copper (and of the paste opening, which is
    # the copper here).
    setup = '(pad_to_mask_clearance 0.1)'
    pcb, path = _parse(_board(_fp() + _via(100.8, 100), setup))
    os.remove(path)
    f = via_mask_findings(pcb)
    check('2. a ring in the opening is a contact',
          [x['category'] for x in f] == ['via-mask-contact'], str(f))
    check('2. the finding names the pad and side',
          f and f[0]['pad'] == 'U1.1' and f[0]['side'] == 'F.Mask', str(f))
    pcb, path = _parse(_board(_fp() + _via(101.0, 100), setup))
    os.remove(path)
    check('2. the same via clear of the opening is clean', via_mask_findings(pcb) == [])
    pcb, path = _parse(_board(_fp() + _via(100.6, 100), setup))
    os.remove(path)
    check('2. a barrel in a same-net paste opening is left to via-in-paste',
          via_mask_findings(pcb) == [], str(via_mask_findings(pcb)))
    # A via-in-paste of U1 (barrel over U1's copper corner) beside a /B pad
    # whose opening its ring reaches: 0.05 mm of copper gap, 0.1 mm of growth.
    for layers, side in (('"F.Cu" "F.Paste" "F.Mask"', 'F.Mask'),
                         ('"B.Cu" "B.Paste" "B.Mask"', 'B.Mask')):
        other = ('(footprint "L:P" (layer "F.Cu") (at 100.6 100.9)\n'
                 '   (property "Reference" "U2")\n'
                 '   (pad "1" smd rect (at 0 0) (size 1 0.5) (layers %s) (net 2 "/B")))'
                 % layers)
        pcb, path = _parse(_board(_fp() + other + _via(100.6, 100.35), setup))
        os.remove(path)
        f = via_mask_findings(pcb)
        check(f'2. ...but its contact with a foreign pad on {side} is reported',
              [(x['category'], x['pad'], x['side']) for x in f]
              == [('via-mask-contact', 'U2.1', side)], str(f))
    pcb, path = _parse(_board(_fp(layers='"F.Cu" "F.Mask"') + _via(100.3, 100), setup))
    os.remove(path)
    check('2. a via centred in the pad is via-in-pad, not this',
          via_mask_findings(pcb) == [], str(via_mask_findings(pcb)))
    pcb, path = _parse(_board(_fp() + _via(100.8, 100, net=2), setup))
    os.remove(path)
    check('2. a foreign-net via counts too',
          [x['category'] for x in via_mask_findings(pcb)] == ['via-mask-contact'])
    check('2. an ordinary rect opening is exact',
          via_mask_findings(pcb)[0]['approximation'] == '')
    trap = _fp(kind='smd trapezoid', pad_extra='(rect_delta 0 0.2)')
    pcb, path = _parse(_board(trap + _via(100.8, 100), setup))
    os.remove(path)
    f = via_mask_findings(pcb)
    check('2. a trapezoid opening says it is approximate',
          len(f) == 1 and 'trapezoid' in f[0]['approximation'], str(f))

    # ---- 3. a pad's own margin moves the verdict ------------------------------
    pcb, path = _parse(_board(_fp(pad_extra='(solder_mask_margin 0)') + _via(100.8, 100), setup))
    os.remove(path)
    check("3. the pad's margin 0 closes the band", via_mask_findings(pcb) == [])
    # The copper corner is (100.5, 100.25). On the diagonal, a ring 0.12 mm
    # from it lies inside the square growth (100.6, 100.35) but outside the
    # rounded 0.1 mm opening; 0.09 mm from it, it is inside both.
    for ring_gap, want in ((0.12, []), (0.09, ['via-mask-contact'])):
        off = (ring_gap + 0.25) / 2 ** 0.5
        pcb, path = _parse(_board(_fp() + _via(100.5 + off, 100.25 + off), setup))
        os.remove(path)
        check(f"3. a via {ring_gap} mm off the corner -> {want or 'clean'}",
              [x['category'] for x in via_mask_findings(pcb)] == want,
              str(via_mask_findings(pcb)))

    # ---- 4. via-mask-web --------------------------------------------------------
    plug = '(plugging (front yes) (back no))'
    pcb, path = _parse(_board(_fp() + _via(101.0, 100, extra=plug), setup, 20260206))
    os.remove(path)
    f = via_mask_findings(pcb)
    check('4. a plugged via 0.15 mm from the opening is under the 0.35 web',
          [x['category'] for x in f] == ['via-mask-web'], str(f))
    check('4. its distance is reported',
          f and abs(f[0]['distance_mm'] - 0.15) < 1e-6, str(f))
    check('4. --plug-web 0.1 clears it', via_mask_findings(pcb, plug_web=0.1) == [])
    pcb, path = _parse(_board(_fp() + _via(101.0, 100), setup, 20260206))
    os.remove(path)
    check('4. the same via unplugged is clean', via_mask_findings(pcb) == [])
    pcb, path = _parse(_board(_fp() + _via(101.0, 100, extra='(plugging (front no) (back yes))'),
                              setup, 20260206))
    os.remove(path)
    check('4. plugging on the far side only is clean', via_mask_findings(pcb) == [])

    # ---- 5. baseline ------------------------------------------------------------
    pcb_in, p_in = _parse(_board(_fp() + _via(100.8, 100), setup))
    pcb_out, p_out = _parse(_board(_fp() + _via(100.8, 100) + _via(100, 100.6), setup))
    new, inh = split_inherited(via_mask_findings(pcb_out), via_mask_findings(pcb_in))
    check('5. the input\'s contact is accepted', len(inh) == 1, str(inh))
    check('5. the new contact is not', len(new) == 1 and new[0]['y'] == 100.6, str(new))
    check('5. counts() reads the categories',
          counts(new) == {'via-mask-contact': 1, 'via-mask-web': 0}, str(counts(new)))

    # ---- 6. the CLI -------------------------------------------------------------
    tool = os.path.join(ROOT_DIR, 'py_tools', 'check_via_mask.py')
    fd, js = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    r = subprocess.run([sys.executable, tool, p_out, '--json', js],
                       capture_output=True, text=True)
    check('6. exit 1 on a finding', r.returncode == 1, r.stdout + r.stderr)
    with open(js) as fh:
        doc = json.load(fh)
    check('6. --json counts both contacts',
          doc['counts'] == {'via-mask-contact': 2, 'via-mask-web': 0}, str(doc['counts']))
    r = subprocess.run([sys.executable, tool, p_in, '--baseline', p_in],
                       capture_output=True, text=True)
    check('6. exit 0 when every finding is accepted', r.returncode == 0, r.stdout + r.stderr)
    for p in (p_in, p_out, js):
        os.remove(p)

    print(f"\n{'ALL PASS' if not FAILS else '%d FAIL' % len(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
