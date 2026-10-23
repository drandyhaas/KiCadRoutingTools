#!/usr/bin/env python3
"""make_unplaced's pile carries no text angle that hands back a rotation (#1163).

The pile sets every moved part to rotation 0, and the writer composes every
text angle with the rotation change, as pcbnew does. KiCad stores a text angle
as an ABSOLUTE board angle, so the pile's text read `T - R` (human text angle
minus human part rotation); humans keep labels upright, so that handed back
`R`. Measured on the tracked boards before the fix: the Reference alone
recovered the rotation exactly for 7 parts of esp_prog, 13 of watchy, 24 of
tigard and 14 of splitflap_driver.

The fixture is the issue's minimal input on esp_prog: C1 (at ... 90) with its
texts turned upright (absolute 0), plus the issue's second channel: a GND zone
drawn tight around one C1 pad, a keep-out (rule area) tight around a pad of a
second part, and a board-wide GND pour.

Checks:
  1. NEGATIVE CONTROL: the pile written the old way (no `new_text_angle`)
     reads C1's Reference at 270 = -R_h, and `text_angle_leaks` flags it --
     so the check below can fail;
  2. a REAL rotation still composes (C1 90 -> 180 turns its texts by 90);
  3. make_unplaced's pile: every text of every moved part is written
     `(at x y 0)`, token present, its summary says so, C1's is 0;
  4. the zones: a board-level zone that outlines the placement (under a
     quarter of the bounding box, holding a moved part's pad) is DROPPED --
     the local pour and the local keep-out both -- and the board-wide pour is
     kept as it was; the summary names the parts; with --keep-local-zones
     every zone is kept and the summary still names them;
  5. `new_text_angle` refuses a non-number at the boundary.

    python3 tests/test_1163_pile_text_angles.py
"""
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts')
for _d in (os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'tests'), SCRIPTS):
    sys.path.insert(0, _d)

import run_utils                                              # noqa: E402
from kicad_parser import parse_kicad_pcb, iter_footprint_blocks  # noqa: E402
from placement.writer import write_placed_output, text_node_angles  # noqa: E402
import make_unplaced                                          # noqa: E402

ESP = run_utils.evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'))
UNPLACED = os.path.join(SCRIPTS, 'make_unplaced.py')
PY = [sys.executable, '-X', 'utf8']
ANCHOR = '\t(embedded_fonts no)\n)'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f" ({detail})" if detail else ''))
    if not ok:
        failures.append(label)


def _block(text, ref):
    for _s, _e, fp_text, _raw, key in iter_footprint_blocks(text):
        if key == ref:
            return fp_text
    raise AssertionError(f'BROKEN FIXTURE: {ref} not found')


def _zone(uid, ring):
    pts = ' '.join('(xy %.4f %.4f)' % p for p in ring)
    return ('\t(zone\n\t\t(net "GND")\n\t\t(layer "B.Cu")\n\t\t(uuid "%s")\n'
            '\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0.5)\n\t\t)\n'
            '\t\t(min_thickness 0.25)\n\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n'
            '\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)\n' % (uid, pts))


def _keepout(uid, ring):
    pts = ' '.join('(xy %.4f %.4f)' % p for p in ring)
    return ('\t(zone\n\t\t(layers "F.Cu" "B.Cu")\n\t\t(uuid "%s")\n'
            '\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0)\n\t\t)\n'
            '\t\t(min_thickness 0.25)\n\t\t(keepout\n\t\t\t(tracks not_allowed)\n'
            '\t\t\t(vias not_allowed)\n\t\t\t(pads allowed)\n'
            '\t\t\t(copperpour not_allowed)\n\t\t\t(footprints allowed)\n\t\t)\n'
            '\t\t(fill\n\t\t\t(thermal_gap 0.5)\n\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)\n' % (uid, pts))


def _box(pad):
    return [(pad.global_x - 1, pad.global_y - 1), (pad.global_x + 1, pad.global_y - 1),
            (pad.global_x + 1, pad.global_y + 1), (pad.global_x - 1, pad.global_y + 1)]


LOCAL_POUR = '11111111-0000-4000-8000-000000000001'
WIDE_POUR = '11111111-0000-4000-8000-000000000002'
LOCAL_KEEPOUT = '11111111-0000-4000-8000-000000000003'


def _zone_uuids(path):
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    return sorted(text[s:e].split('(uuid "', 1)[1].split('"', 1)[0]
                  for s, e, _r in make_unplaced.board_zone_blocks(text))


def build(td):
    with open(ESP, encoding='utf-8') as fh:
        text = fh.read()
    c1 = _block(text, 'C1')
    if not re.search(r'\n\t\t\(at [^ ]+ [^ ]+ 90\)\n', c1):
        raise AssertionError('BROKEN FIXTURE: esp_prog C1 is no longer at 90')
    # the human keeps C1's labels upright: every text at absolute 0
    upright = re.sub(r'(\n\t\t\((?:property|fp_text) [^\n]*\n\t\t\t\(at [^ ]+ [^ )]+) 90\)',
                     r'\1 0)', c1)
    if upright == c1:
        raise AssertionError('BROKEN FIXTURE: no C1 text at 90 to turn upright')
    text = text.replace(c1, upright)
    pcb = parse_kicad_pcb(ESP)
    pad = pcb.footprints['C1'].pads[0]
    x0, y0, x1, y1 = pcb.board_info.board_bounds
    local = _box(pad)
    # the keep-out sits around a pad of the part farthest from C1, so the two
    # local zones name different parts
    other = max((r for r, f in pcb.footprints.items() if f.pads and r != 'C1'),
                key=lambda r: (pcb.footprints[r].x - pad.global_x) ** 2
                + (pcb.footprints[r].y - pad.global_y) ** 2)
    wide = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    text = text.replace(ANCHOR, _zone(LOCAL_POUR, local) + _zone(WIDE_POUR, wide)
                        + _keepout(LOCAL_KEEPOUT, _box(pcb.footprints[other].pads[0]))
                        + ANCHOR)
    src = os.path.join(td, 'human.kicad_pcb')
    with open(src, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    return src, text, other


def main():
    with tempfile.TemporaryDirectory() as td:
        src, text, other = build(td)
        c1_src = text_node_angles(_block(text, 'C1'))
        check('fixture: C1 is at 90 and every text of it at absolute 0',
              parse_kicad_pcb(src).footprints['C1'].rotation == 90
              and c1_src and all(a == 0 for a in c1_src), c1_src)

        # 1. the old write, as the negative control
        old = os.path.join(td, 'old_pile.kicad_pcb')
        write_placed_output(src, old, [{'reference': 'C1', 'new_x': 130.0,
                                        'new_y': 120.0, 'new_rotation': 0.0}])
        with open(old, encoding='utf-8') as fh:
            c1_old = text_node_angles(_block(fh.read(), 'C1'))
        check('control: composed into the pile, C1 Reference reads 270 = -R_h',
              c1_old and c1_old[0] == 270, c1_old)
        leaks, _n, _good = make_unplaced.text_angle_leaks(old, {'C1'})
        check('control: text_angle_leaks flags the old pile', 'C1' in leaks, leaks)

        # 2. a real rotation is untouched by the fix
        rot = os.path.join(td, 'rotated.kicad_pcb')
        write_placed_output(src, rot, [{'reference': 'C1', 'new_x': 130.0,
                                        'new_y': 100.0, 'new_rotation': 180.0}])
        with open(rot, encoding='utf-8') as fh:
            c1_rot = text_node_angles(_block(fh.read(), 'C1'))
        check('a real rotation 90 -> 180 still composes every text by +90',
              c1_rot and all(a == 90 for a in c1_rot), c1_rot)

        # 3. the pile
        pile = os.path.join(td, 'pile.kicad_pcb')
        r = run_utils.check(PY + [UNPLACED, src, pile], accept=True)
        with open(run_utils.evidence(pile), encoding='utf-8') as fh:
            ptext = fh.read()
        human = parse_kicad_pcb(src)
        moved = {k for k, f in parse_kicad_pcb(pile).footprints.items()
                 if f.pads and (round(f.x, 4), round(f.y, 4))
                 != (round(human.footprints[k].x, 4), round(human.footprints[k].y, 4))}
        check('control: the pile moved C1 and most parts', 'C1' in moved
              and len(moved) >= 15, sorted(moved))
        tokens = {}
        for _s, _e, fp_text, _raw, key in iter_footprint_blocks(ptext):
            if key in moved:
                tokens[key] = re.findall(
                    r'\n\t\t\((?:property|fp_text) [^\n]*\n\t\t\t(\(at [^)]*\))', fp_text)
        bad = {k: v for k, v in tokens.items()
               if not v or any(not re.fullmatch(r'\(at \S+ \S+ 0\)', t) for t in v)}
        check('every text of every moved part is written (at x y 0)', not bad,
              dict(list(bad.items())[:3]))
        leaks, n, good = make_unplaced.text_angle_leaks(pile, moved)
        check('text_angle_leaks: none', not leaks and n == good > 0, (leaks, n, good))
        line = [ln for ln in r.stdout.splitlines() if ln.startswith('texts')]
        check('the summary states it', line == [f'texts      {n} of {n} text(s) on '
                                                f'{len(moved)} moved part(s) at absolute 0'],
              line)

        # 4. the zones
        check('fixture: three board-level zones, two of them local',
              _zone_uuids(src) == sorted([LOCAL_POUR, WIDE_POUR, LOCAL_KEEPOUT]))
        zl = [ln for ln in r.stdout.splitlines() if ln.startswith('zones')]
        check('the zone line says both local zones were dropped and names '
              'the parts', zl and zl[0].startswith(
                  'zones      1 of 3 board-level zone(s)/rule area(s) kept; '
                  'dropped 2 that cover under 25%')
              and f'2 moved part(s) ({", ".join(sorted(["C1", other]))})' in zl[0],
              zl)
        check('the local pour and the local keep-out are gone, the board-wide '
              'pour stays', _zone_uuids(pile) == [WIDE_POUR], _zone_uuids(pile))
        hz = [(z.uuid, tuple(z.polygon)) for z in human.zones if z.uuid == WIDE_POUR]
        pz = [(z.uuid, tuple(z.polygon)) for z in parse_kicad_pcb(pile).zones]
        check('the kept pour is exactly as it was', hz == pz and len(hz) == 1)
        # --keep-local-zones: kept, and still disclosed
        kept = os.path.join(td, 'kept.kicad_pcb')
        rk = run_utils.check(PY + [UNPLACED, src, kept, '--keep-local-zones'],
                             accept=True)
        zk = [ln for ln in rk.stdout.splitlines() if ln.startswith('zones')]
        check('--keep-local-zones keeps every zone',
              _zone_uuids(run_utils.evidence(kept))
              == sorted([LOCAL_POUR, WIDE_POUR, LOCAL_KEEPOUT]),
              _zone_uuids(kept))
        check('...and its zone line still names the parts they outline',
              zk and '(--keep-local-zones); 2 cover under 25%' in zk[0]
              and 'C1' in zk[0] and other in zk[0], zk)

        # 5. the boundary
        for bad_angle in ('0', True):
            try:
                write_placed_output(src, os.path.join(td, 'x.kicad_pcb'),
                                    [{'reference': 'C1', 'new_x': 1.0, 'new_y': 1.0,
                                      'new_rotation': 0.0,
                                      'new_text_angle': bad_angle}])
                check(f'new_text_angle={bad_angle!r} refused', False)
            except ValueError as e:
                check(f'new_text_angle={bad_angle!r} refused', 'new_text_angle' in str(e))

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nPASS: a pile's texts read absolute 0 and its placement-local zones are dropped")
    return 0


if __name__ == '__main__':
    sys.exit(main())
