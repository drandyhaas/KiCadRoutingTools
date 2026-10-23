#!/usr/bin/env python3
"""A zone drawn past the board renders only inside the outline (#1205).

KiCad clips a pour's FILL to the board, so drawing a GND zone as a rectangle
larger than the board is the usual habit. `BoardRenderer` painted each zone's
stored outline polygon over the board outline unclipped, so a non-rectangular
board rendered as its bounding box: CM5_MINIMA_3's notch, ears and edge stroke
were covered, and an agent placed edge parts against the bbox. The same
renderer backs render_placement. board_brief's text line printed the bbox as
"outline ... (bbox area)" and showed the real area only when cutouts existed.

The fixture is esp_prog with an EAR added on its right edge (so the two
corners beside the ear are inside the bounding box and outside the board, with
no part near them) and a GND pour on F.Cu drawn well past the board.

Checks, by pixel probe (supersample 1, so no antialiasing):
  1. static base (render_placement's path): the corners beside the ear and a
     point past the bbox corner render as background, a point on the board and
     one in the ear as zone tint, and the edge stroke shows over the pour;
  2. render_placement's own panel agrees at the same probes;
  3. control: without the pour the background probes are background and the
     board probes are board body -- so 1-2 test the clip, not the fixture;
  4. board_brief's text line prints the real outline area (bbox differs).

    python3 tests/test_1205_zone_clip_render.py
"""
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_tools', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import run_utils                                              # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from route_render import BoardRenderer                        # noqa: E402

ESP = run_utils.evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'))
ANCHOR = '\t(embedded_fonts no)\n)'
#: esp_prog is 114..145.75 x 91..105.5; the ear adds 145.75..150.75 x 95..101.5.
OUTLINE = [(114, 91), (145.75, 91), (145.75, 95), (150.75, 95), (150.75, 101.5),
           (145.75, 101.5), (145.75, 105.5), (114, 105.5)]
POUR = [(110, 87), (155, 87), (155, 110), (110, 110)]
#: name -> (x, y, what it must render as)
PROBES = {
    'beside_ear_top': (148.25, 93.0, 'ground'),
    'beside_ear_bottom': (148.25, 103.5, 'ground'),
    'past_corner': (151.05, 105.8, 'ground'),
    'on_board': (122.0, 92.0, 'zone'),
    'in_ear': (148.25, 98.25, 'zone'),
    'edge': (145.75, 93.0, 'board_edge'),
}
SIZE = 800
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f" ({detail})" if detail else ''))
    if not ok:
        failures.append(label)


def _edge(a, b, n):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(stroke\n'
            '\t\t\t(width 0.05)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "12050000-0000-4000-8000-%012d")\n\t)\n'
            % (a[0], a[1], b[0], b[1], n))


def build(td, pour=True):
    with open(ESP, encoding='utf-8') as fh:
        text = fh.read()
    lines = re.findall(r'\t\(gr_line\n(?:\t\t[^\n]*\n)*?\t\t\(layer "Edge\.Cuts"\)\n'
                       r'(?:\t\t[^\n]*\n)*?\t\)\n', text)
    if len(lines) != 4 or ANCHOR not in text:
        raise AssertionError('BROKEN FIXTURE: esp_prog no longer has 4 Edge.Cuts '
                             'lines and the expected tail')
    for ln in lines:
        text = text.replace(ln, '')
    edges = ''.join(_edge(OUTLINE[i], OUTLINE[(i + 1) % len(OUTLINE)], i + 1)
                    for i in range(len(OUTLINE)))
    zone = ''
    if pour:
        zone = ('\t(zone\n\t\t(net "GND")\n\t\t(layer "F.Cu")\n'
                '\t\t(uuid "12050000-0000-4000-8000-000000000100")\n'
                '\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0.5)\n\t\t)\n'
                '\t\t(min_thickness 0.25)\n\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n'
                '\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t'
                + ' '.join('(xy %g %g)' % p for p in POUR)
                + '\n\t\t\t)\n\t\t)\n\t)\n')
    text = text.replace(ANCHOR, edges + zone + ANCHOR)
    path = os.path.join(td, 'eared%s.kicad_pcb' % ('' if pour else '_nopour'))
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    return path


def expected(r, what):
    if what == 'ground':
        return r.bg
    if what == 'zone':
        base = r.palette.get('F.Cu', r.theme.rgb('zone_tint'))
        body = r.theme.rgb('board_body')
        return tuple(int(body[i] * 0.55 + base[i] * 0.45) for i in range(3))
    return r.theme.rgb(what)


def probe(img, r):
    return {k: img.getpixel(tuple(int(round(v)) for v in r.tf.pt(x, y)))
            for k, (x, y, _w) in PROBES.items()}


def grade(label, img, r, want=None):
    got = probe(img, r)
    want = want or {k: expected(r, w) for k, (_x, _y, w) in PROBES.items()}
    bad = {k: (got[k], want[k]) for k in PROBES if tuple(got[k][:3]) != tuple(want[k])}
    check(label, not bad, bad or '')


def main():
    with tempfile.TemporaryDirectory() as td:
        board = build(td)
        pcb = parse_kicad_pcb(board)
        ring = pcb.board_info.board_outlines or [pcb.board_info.board_outline]
        check('fixture: the outline parses as the eared ring and the pour as a zone',
              len(ring) == 1 and len(ring[0]) == len(OUTLINE)
              and pcb.board_info.board_bounds == (114.0, 91.0, 150.75, 105.5)
              and any(z.net_name == 'GND' and z.layer == 'F.Cu' for z in pcb.zones),
              (pcb.board_info.board_bounds, len(ring[0]) if ring and ring[0] else 0))

        # 1. the static base: render_placement and render_board_file
        r = BoardRenderer(pcb, size=SIZE, supersample=1, show_pads=False)
        grade('static base: pour clipped to the outline, edge drawn over it',
              r.render(), r)

        # 2. render_placement's panel at the same probes
        import render_placement as RP
        m = RP.PlacementModel(pcb, board)
        img = RP.render_panel(RP.PanelSpec(m), size=SIZE, supersample=1)
        rp_got = probe(img, r)
        rp_bad = {k: rp_got[k] for k in ('beside_ear_top', 'beside_ear_bottom', 'past_corner')
                  if tuple(rp_got[k][:3]) != tuple(r.bg)}
        check('render_placement panel: the corners beside the ear and past the bbox '
              'are background', not rp_bad, rp_bad or '')

        # 3. control: without the pour
        bare = parse_kicad_pcb(build(td, pour=False))
        rb = BoardRenderer(bare, size=SIZE, supersample=1, show_pads=False)
        grade('control: no pour -> background outside, board body inside',
              rb.render(), rb, {k: expected(rb, 'board_body' if w == 'zone' else w)
                                for k, (_x, _y, w) in PROBES.items()})

        # 4. board_brief's text line
        r5 = run_utils.check([sys.executable, '-X', 'utf8',
                              os.path.join(ROOT, 'py_tools', 'board_brief.py'), board],
                             accept=True)
        line = [ln for ln in r5.stdout.splitlines() if ln.strip().startswith('outline')]
        bbox = round(36.75 * 14.5, 2)
        area = round(31.75 * 14.5 + 5 * 6.5, 2)
        check('board_brief prints the real outline area beside the bbox',
              line and f'({bbox} mm2)' in line[0] and f'outline area {area} mm2' in line[0],
              line)
        plain = run_utils.check([sys.executable, '-X', 'utf8',
                                 os.path.join(ROOT, 'py_tools', 'board_brief.py'), ESP],
                                accept=True)
        pline = [ln for ln in plain.stdout.splitlines() if ln.strip().startswith('outline')]
        check('...and keeps the old line on a rectangle, where they agree',
              pline and 'outline area' not in pline[0], pline)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nPASS: pours render inside the outline on every path; the brief names the real area")
    return 0


if __name__ == '__main__':
    sys.exit(main())
