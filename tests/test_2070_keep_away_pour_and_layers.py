#!/usr/bin/env python3
"""#2070: a keep-away sees the other side's POUR, and, asked, the next layer.

`--keep-away AGG:VICTIM:GAP` used to price and report the victim against the
other side's tracks, vias and pads on the victim's own layer only. A pour of
the aggressor gives way to the routed track, so the track ran through its
outline and KiCad refilled it 0.25 mm from the victim, while the step said
"0 of 1 net(s) run 0 mm inside a band". A track of the aggressor on the next
layer was run over (0 mm in plan view), with the same line.

Fixture: kicad_files/cap_chain plus one DPB_N item 0.7 mm (centre to centre)
beside DPA_P's straight J1.1-C1.1 line, in three variants built in a temp
dir: a 0.25 mm F.Cu track (track_F, the control), the same track on B.Cu
(track_B), and a 1 mm wide F.Cu pour whose edge lies where the track's edge
does (pour_F). Each routes DPA_P on F.Cu under `DPB_N:DPA_P:1.0`.

Rows:
  - the rule syntax: `:adjacent` round-trips through the parser, the brief
    reader and `spec`; an unknown fourth field is refused by name;
  - the band (no KiCad): DPA_P's priced cells cover the DPB_N pour's outline
    and a GAP around it, on F.Cu only for the plain rule;
  - pour_F routed: DPA_P stays GAP from the pour's outline, or the report
    names DPA_P; the report says the pour was measured on its outline;
  - track_B: the plain rule runs over the B.Cu track and its report says
    other layers were not measured; with `:adjacent` DPA_P is kept GAP from
    it in plan view, or reported;
  - track_F (control): 1.005 mm, `0 of 2 net(s)`, as before;
  - with KiCad: the delivered fill (py_tools/fill_for_delivery.py) of pour_F
    lies GAP from DPA_P, or the report names DPA_P, and the shipped report
    (--json-out) was measured on the run's own KiCad refill. Without KiCad
    this row is skipped and the test exits 77 after the rows above PASS.

    python3 tests/test_2070_keep_away_pour_and_layers.py
"""

import contextlib
import io
import json
import os
import re
import sys
import tempfile
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import check, evidence  # noqa: E402

import keep_away as ka  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from shapely.geometry import LineString, Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
GAP = 1.0
RULE = f'DPB_N:DPA_P:{GAP:g}'
TOL = 0.01            # keep_away.REPORT_TOL


def _build(d):
    """The issue's three variants of cap_chain, written into `d`."""
    t = open(evidence(BOARD), encoding='utf8').read()
    end = t.rindex('\t(embedded_fonts no)')
    a, b = (12.18, 15.14), (16.17, 14.08)

    def seg(p, q, layer):
        return (f'\t(segment (start {p[0]} {p[1]}) (end {q[0]} {q[1]}) '
                f'(width 0.25) (layer "{layer}") (net "DPB_N") '
                f'(uuid "{uuid.uuid4()}"))\n')

    def track(layer):   # joined to J2.2 (through-hole): not swept as an island
        return (seg(a, b, layer) + seg(b, (19.63, 17.54), layer)
                + seg((19.63, 17.54), (26.0, 17.54), layer))
    n = (0.2570, 0.9664)
    ring = [(round(p[0] + k * n[0], 3), round(p[1] + k * n[1], 3))
            for p, k in ((a, -0.125), (b, -0.125), (b, 0.875), (a, 0.875))]
    pts = ' '.join(f'(xy {x} {y})' for x, y in ring)
    zone = (f'\t(zone (net "DPB_N") (layer "F.Cu") (uuid "{uuid.uuid4()}") '
            f'(hatch edge 0.5)\n'
            f'\t\t(connect_pads yes (clearance 0.2)) (min_thickness 0.1)\n'
            f'\t\t(fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) '
            f'(island_removal_mode 1))\n'
            f'\t\t(polygon (pts {pts})))\n')
    out = {}
    for name, add in (('track_F', track('F.Cu')), ('track_B', track('B.Cu')),
                      ('pour_F', zone)):
        out[name] = os.path.join(d, f'{name}.kicad_pcb')
        with open(out[name], 'w', encoding='utf8') as fh:
            fh.write(t[:end] + add + t[end:])
    return out, ring


def _parse(path):
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(evidence(path))


def _victim(pcb):
    ids = {n.name: i for i, n in pcb.nets.items()}
    caps = [LineString([(s.start_x, s.start_y), (s.end_x, s.end_y)])
            .buffer(s.width / 2) for s in pcb.segments
            if s.net_id == ids['DPA_P'] and s.layer == 'F.Cu']
    assert caps, "DPA_P has no F.Cu track on the routed board"
    return unary_union(caps), ids


def _aggressor_tracks(pcb, ids, layer):
    caps = [LineString([(s.start_x, s.start_y), (s.end_x, s.end_y)])
            .buffer(s.width / 2) for s in pcb.segments
            if s.net_id == ids['DPB_N'] and s.layer == layer]
    assert caps, f"no DPB_N track on {layer}"
    return unary_union(caps)


def _route(src, out, rule, td):
    jout = os.path.splitext(out)[0] + '.json'
    r = check([sys.executable, '-X', 'utf8', 'py_router/route.py', src, out,
               '--nets', 'DPA_P', '--layers', 'F.Cu', '--keep-away', rule,
               '--json-out', jout], accept=True)
    log = r.stdout + r.stderr
    line = re.findall(r'^Keep-away \(.*$', log, re.M)
    assert line, f"no Keep-away report line in the log:\n{log[-1500:]}"
    rep = json.loads(open(evidence(jout), encoding='utf8').read())['keep_away']
    return line[0], rep, log


def test_rule_syntax():
    (r,) = ka.parse_keep_away_rules('DPB_N:DPA_P:1:adjacent')
    assert r.adjacent and r.gap == 1.0 and r.spec == 'DPB_N:DPA_P:1:adjacent', r
    (r,) = ka.parse_keep_away_rules('DPB_N:DPA_P:1')
    assert not r.adjacent and r.spec == 'DPB_N:DPA_P:1', r
    assert ka.normalize_keep_away_specs('a,b:c:0.5:ADJACENT') == (
        'a,b:c:0.5:adjacent',)
    rows = ka.brief_rule_rows(['DPB_N:DPA_P:1.0:adjacent',
                               {'rule': 'DPB_N:DPA_P:1.0', 'why': 'w'}])
    assert [x['rule'] for x in rows] == ['DPB_N:DPA_P:1:adjacent',
                                          'DPB_N:DPA_P:1'], rows
    try:
        ka.parse_keep_away_rules('DPB_N:DPA_P:1:below')
    except ValueError as e:
        assert "unknown option 'below'" in str(e), e
    else:
        raise AssertionError("an unknown fourth field was accepted")


def test_band_covers_the_pour(variants, ring):
    """KiCad-free: the cells DPA_P is priced on cover the pour's outline and
    a GAP around it, on the pour's layer."""
    pcb = _parse(variants['pour_F'])
    ids = {n.name: i for i, n in pcb.nets.items()}
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.25,
                          keep_away=ka.normalize_keep_away_specs(RULE),
                          keep_away_free=0.0, keep_away_cost=0.5)
    with contextlib.redirect_stdout(io.StringIO()):
        rows = ka.keep_away_rows(cfg, pcb, ids['DPA_P'])
    assert rows is not None, "no band at all for DPA_P against the DPB_N pour"
    cells = {(int(r[0]), int(r[1]), int(r[2])) for r in rows}
    step = cfg.grid_step
    poly = Polygon(ring)
    cx, cy = poly.centroid.x, poly.centroid.y
    # A point 0.9 mm outside the outline's long DPA_P-side edge: inside
    # GAP + half the track width of it.
    (x0, y0), (x1, y1) = ring[0], ring[1]
    nx, ny = (0.2570, 0.9664)
    mx, my = (x0 + x1) / 2 - 0.9 * nx, (y0 + y1) / 2 - 0.9 * ny
    for what, (x, y) in (('inside the pour outline', (cx, cy)),
                         ('0.9 mm outside it', (mx, my))):
        c = (0, round(x / step), round(y / step))
        assert c in cells, (f"the F.Cu cell {what} ({x:.2f}, {y:.2f}) is not "
                            f"priced: the band does not see the DPB_N pour")
        assert (1, c[1], c[2]) not in cells, (
            f"the plain rule priced B.Cu {what}: a pour on F.Cu is F.Cu's")


def test_pour_routed(variants, ring, td):
    out = os.path.join(td, 'pour_F.out.kicad_pcb')
    line, rep, _log = _route(variants['pour_F'], out, RULE, td)
    vic, _ids = _victim(_parse(out))
    d = vic.distance(Polygon(ring))
    named = 'DPA_P' in rep['nets']
    print(f"    pour_F: DPA_P {d:.3f} mm from the DPB_N pour outline; {line}")
    assert d >= GAP - TOL or named, (
        f"DPA_P runs {d:.3f} mm from the DPB_N pour's outline (gap {GAP:g}) "
        f"and the report names no net: the pour is neither priced nor "
        f"measured ({line})")
    assert rep.get('pour_measured_on') in ('outline', 'kicad_refill'), rep
    return out, d


def test_track_b(variants, td):
    # The plain rule: same-layer by design (#1146), and it says so.
    out = os.path.join(td, 'track_B.plain.kicad_pcb')
    line, rep, log = _route(variants['track_B'], out, RULE, td)
    pcb = _parse(out)
    vic, ids = _victim(pcb)
    d_plain = vic.distance(_aggressor_tracks(pcb, ids, 'B.Cu'))
    print(f"    track_B plain: {d_plain:.3f} mm in plan view; {line}")
    assert d_plain < GAP - TOL, (
        f"the plain rule kept DPA_P {d_plain:.3f} mm from a B.Cu track: it "
        f"is same-layer by design")
    assert ka.NOT_MEASURED_OTHER_LAYERS in rep.get('not_measured', []), rep
    assert 'not measured: copper on other layers' in log, log[-1500:]
    # `:adjacent`: B.Cu is next to F.Cu on a 2-layer board.
    out = os.path.join(td, 'track_B.adjacent.kicad_pcb')
    line, rep, _log = _route(variants['track_B'], out, RULE + ':adjacent', td)
    pcb = _parse(out)
    vic, ids = _victim(pcb)
    d_adj = vic.distance(_aggressor_tracks(pcb, ids, 'B.Cu'))
    print(f"    track_B :adjacent: {d_adj:.3f} mm in plan view; {line}")
    assert d_adj >= GAP - TOL or 'DPA_P' in rep['nets'], (
        f"with ':adjacent' DPA_P runs {d_adj:.3f} mm in plan view from the "
        f"DPB_N track on B.Cu and the report names no net ({line})")
    assert ka.NOT_MEASURED_OTHER_LAYERS not in rep.get('not_measured', []), rep


def test_track_f_control(variants, td):
    out = os.path.join(td, 'track_F.out.kicad_pcb')
    line, rep, _log = _route(variants['track_F'], out, RULE, td)
    pcb = _parse(out)
    vic, ids = _victim(pcb)
    d = vic.distance(_aggressor_tracks(pcb, ids, 'F.Cu'))
    print(f"    track_F: {d:.3f} mm; {line}")
    assert abs(d - 1.005) < 0.01, f"control moved: {d:.3f} mm, was 1.005"
    assert '0 of 2 net(s)' in line, line
    assert rep['nets_in_band'] == 0, rep


def test_kicad_fill(routed, td):
    """With KiCad: the delivered fill lies GAP from DPA_P, or the report
    names DPA_P, and the shipped report read the run's own refill."""
    from kicad_exact_fill import find_kicad_python
    if find_kicad_python() is None:
        return False
    jout = os.path.splitext(routed)[0] + '.json'
    rep = json.loads(open(evidence(jout), encoding='utf8').read())['keep_away']
    assert rep.get('pour_measured_on') == 'kicad_refill', (
        f"the shipped report did not read the run's KiCad refill: {rep}")
    filled = os.path.join(td, 'pour_F.filled.kicad_pcb')
    check([sys.executable, '-X', 'utf8', 'py_tools/fill_for_delivery.py',
           routed, '-o', filled], accept=True)
    text = open(evidence(filled), encoding='utf8').read()
    polys = []
    for m in re.finditer(r'\(filled_polygon\s*\(layer "F\.Cu"\)\s*\(pts(.*?)\)'
                         r'\s*\)', text, re.S):
        xy = [tuple(map(float, p)) for p in
              re.findall(r'\(xy ([-\d.]+) ([-\d.]+)\)', m.group(1))]
        polys.append(Polygon(xy).buffer(0))
    assert polys, "KiCad's refill poured nothing for DPB_N on F.Cu"
    vic, _ids = _victim(_parse(filled))
    d = vic.distance(unary_union(polys))
    print(f"    pour_F delivered fill: {d:.3f} mm from DPA_P")
    assert d >= GAP - TOL or 'DPA_P' in rep['nets'], (
        f"KiCad's fill lies {d:.3f} mm from DPA_P (gap {GAP:g}) and the "
        f"report names no net")
    return True


def main():
    test_rule_syntax()
    print("  PASS test_rule_syntax")
    with tempfile.TemporaryDirectory() as td:
        variants, ring = _build(td)
        test_band_covers_the_pour(variants, ring)
        print("  PASS test_band_covers_the_pour")
        routed, _d = test_pour_routed(variants, ring, td)
        print("  PASS test_pour_routed")
        test_track_b(variants, td)
        print("  PASS test_track_b")
        test_track_f_control(variants, td)
        print("  PASS test_track_f_control")
        if not test_kicad_fill(routed, td):
            print("  SKIP test_kicad_fill: no KiCad python (the rows above "
                  "ran and passed)")
            sys.exit(77)
        print("  PASS test_kicad_fill")
    print("ALL PASS")


if __name__ == '__main__':
    main()
