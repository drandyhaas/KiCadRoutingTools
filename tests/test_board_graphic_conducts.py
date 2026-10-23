#!/usr/bin/env python3
"""Board-level copper graphics CONDUCT, as KiCad 10 has it.

#513 item 6 concluded KiCad never credits a net-tagged copper graphic and made
the connectivity graph skip them. KiCad 10 does credit them: when an
inductor pad sits inside its net's own filled B.Cu gr_poly, pcbnew lists the
shape among the pad's connected items and kicad-cli reports the
net complete, while check_connected graded it open and the route step spent
three terminal-escalation rungs and a reconciliation lap trying to route it.

Each row below is one net: two pads joined (or not) only through the drawn
copper. EXPECTED is KiCad's verdict, measured on this exact geometry with
pcbnew 10.0.0 (GetConnectedItems) and kicad-cli 10.0.0 (DRC unconnected
items); the KiCad half of this file re-measures it whenever KiCad is
installed. What KiCad does:

  * a stroke (gr_line, gr_arc, the outline of gr_rect / gr_poly / gr_circle)
    joins the same-net copper it touches, a track crossing it mid-span too;
  * a FILLED shape joins across its whole interior (pads, vias, track ends
    inside it), a circle on its true radius, a stroke-0 one at its drawn
    edge (not the fab track width the parser gives its outline);
  * an UNFILLED outline does not conduct through its empty middle;
  * a footprint's own copper (an fp_line between its pads) conducts nothing.

Checks:
  1. text parse: check_net_connectivity agrees with EXPECTED on every row;
  2. the route step's terminal grouping (get_terminal_component_info) puts
     pads joined by art in one group and never offers the art as a launch
     surface; a KiCad link whose end is on a cluster that reaches a pad
     through art classifies 'live', not 'graphic';
  3. under KiCad's python: pcbnew's own verdict equals EXPECTED, and the
     pcbnew-built PCBData grades the same as the text parse (#337 parity).

    python3 tests/test_board_graphic_conducts.py
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
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence  # noqa: E402

#: The only shell-out is KiCad's python for the KiCad half, when THIS python
#: has no pcbnew: one interpreter start, seconds.
RUN_ALL_FAST_OK = True

fails = []


def check(label, ok, detail=''):
    print(f"  {'ok  ' if ok else 'FAIL'} {label}{(': ' + detail) if detail else ''}")
    if not ok:
        fails.append(label)


HEADER = """(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6) (legacy_teardrops no))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(25 "Edge.Cuts" user)
\t\t(35 "F.Fab" user)
\t)
\t(setup (pad_to_mask_clearance 0))
"""

_UID = [0]


def uid():
    _UID[0] += 1
    return f"00000000-0000-4000-8000-{_UID[0]:012d}"


def fp(ref, y, net, pads, extra=''):
    """A footprint at (0, y) whose pads are (x, dy, size)."""
    body = []
    for i, (x, dy, size) in enumerate(pads):
        body.append(f'\t\t(pad "{i + 1}" smd rect (at {x} {dy}) '
                    f'(size {size} {size}) (layers "F.Cu") (net "{net}") '
                    f'(uuid "{uid()}"))')
    return (f'\t(footprint "T:T" (layer "F.Cu") (uuid "{uid()}") (at 0 {y})\n'
            f'\t\t(property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS") '
            f'(uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            + extra + '\n'.join(body) + '\n\t)\n')


def stroke(w):
    return f'(stroke (width {w}) (type solid))'


def shape(kind, net, geo, w=0.2, fill=None, layer='F.Cu'):
    f = f' (fill {fill})' if fill else ''
    n = f' (net "{net}")' if net else ''
    return (f'\t({kind} {geo} {stroke(w)}{f} (layer "{layer}"){n} '
            f'(uuid "{uid()}"))\n')


def rect_pts(x0, y0, x1, y1):
    return f'(pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1}))'


def track(net, a, b, w=0.2, layer='F.Cu'):
    return (f'\t(segment (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (width {w}) '
            f'(layer "{layer}") (net "{net}") (uuid "{uid()}"))\n')


def via(net, x, y):
    return (f'\t(via (at {x} {y}) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
            f'(net "{net}") (uuid "{uid()}"))\n')


# name -> (KiCad 10's verdict, board text). Row k sits at y = 10 k.
ROWS = {}


def row(name, expected, text):
    ROWS[name] = (expected, text)


def build():
    def y_of(k):
        return 10 * k
    k = 0

    def nxt():
        nonlocal k
        k += 1
        return y_of(k)

    y = nxt()
    row('LINE', True, fp('A1', y, 'LINE', [(0, 0, 1), (10, 0, 1)])
        + shape('gr_line', 'LINE', f'(start 0 {y}) (end 10 {y})'))
    y = nxt()
    row('ARC', True, fp('A2', y, 'ARC', [(0, 0, 1), (10, 0, 1)])
        + shape('gr_arc', 'ARC', f'(start 0 {y}) (mid 5 {y - 3}) (end 10 {y})'))
    y = nxt()
    row('RECT_OUTLINE_EDGE', True,
        fp('A3', y, 'RECT_OUTLINE_EDGE', [(0, 0, 1), (10, 0, 1)])
        + shape('gr_rect', 'RECT_OUTLINE_EDGE',
                f'(start 0 {y - 2}) (end 10 {y + 2})', fill='no'))
    y = nxt()
    row('RECT_OUTLINE_INSIDE', False,
        fp('A4', y, 'RECT_OUTLINE_INSIDE', [(2, 0, 1), (8, 0, 1)])
        + shape('gr_rect', 'RECT_OUTLINE_INSIDE',
                f'(start 0 {y - 3}) (end 10 {y + 3})', fill='no'))
    y = nxt()
    row('RECT_FILLED_INSIDE', True,
        fp('A5', y, 'RECT_FILLED_INSIDE', [(2, 0, 1), (8, 0, 1)])
        + shape('gr_rect', 'RECT_FILLED_INSIDE',
                f'(start 0 {y - 3}) (end 10 {y + 3})', fill='yes'))
    y = nxt()
    row('POLY_FILLED_INSIDE', True,
        fp('A6', y, 'POLY_FILLED_INSIDE', [(2, 0, 1), (8, 0, 1)])
        + shape('gr_poly', 'POLY_FILLED_INSIDE', rect_pts(0, y - 3, 10, y + 3),
                fill='yes'))
    y = nxt()
    row('POLY_OUTLINE_INSIDE', False,
        fp('A7', y, 'POLY_OUTLINE_INSIDE', [(2, 0, 1), (8, 0, 1)])
        + shape('gr_poly', 'POLY_OUTLINE_INSIDE', rect_pts(0, y - 3, 10, y + 3),
                fill='no'))
    y = nxt()
    row('POLY_OUTLINE_EDGE', True,
        fp('A8', y, 'POLY_OUTLINE_EDGE', [(0, 0, 1), (10, 0, 1)])
        + shape('gr_poly', 'POLY_OUTLINE_EDGE', rect_pts(0, y - 3, 10, y + 3),
                fill='no'))
    y = nxt()
    row('CIRCLE_FILLED_INSIDE', True,
        fp('A9', y, 'CIRCLE_FILLED_INSIDE', [(3, 0, 1), (7, 0, 1)])
        + shape('gr_circle', 'CIRCLE_FILLED_INSIDE',
                f'(center 5 {y}) (end 9 {y})', fill='yes'))
    y = nxt()
    row('CIRCLE_OUTLINE_INSIDE', False,
        fp('A10', y, 'CIRCLE_OUTLINE_INSIDE', [(3.5, 0, 1), (6.5, 0, 1)])
        + shape('gr_circle', 'CIRCLE_OUTLINE_INSIDE',
                f'(center 5 {y}) (end 9 {y})', fill='no'))
    y = nxt()
    row('CIRCLE_OUTLINE_RING', True,
        fp('A11', y, 'CIRCLE_OUTLINE_RING', [(1, 0, 1), (9, 0, 1)])
        + shape('gr_circle', 'CIRCLE_OUTLINE_RING',
                f'(center 5 {y}) (end 9 {y})', fill='no'))
    y = nxt()
    row('TRACK_INTO_FILLED', True,
        fp('A12', y, 'TRACK_INTO_FILLED', [(-6, 0, 1), (16, 0, 1)])
        + shape('gr_poly', 'TRACK_INTO_FILLED', rect_pts(0, y - 3, 10, y + 3),
                fill='yes')
        + track('TRACK_INTO_FILLED', (-6, y), (3, y))
        + track('TRACK_INTO_FILLED', (16, y), (7, y)))
    y = nxt()
    row('TRACK_CROSSES_STROKE', True,
        fp('A13', y, 'TRACK_CROSSES_STROKE', [(-6, 0, 1), (16, 0, 1)])
        + shape('gr_poly', 'TRACK_CROSSES_STROKE', rect_pts(0, y - 3, 10, y + 3),
                fill='no')
        + track('TRACK_CROSSES_STROKE', (-6, y), (3, y))
        + track('TRACK_CROSSES_STROKE', (16, y), (7, y)))
    y = nxt()
    row('TRACK_IN_OUTLINE_NO_STROKE', False,
        fp('A14', y, 'TRACK_IN_OUTLINE_NO_STROKE', [(2, 0, 1), (8, 0, 1)])
        + shape('gr_poly', 'TRACK_IN_OUTLINE_NO_STROKE',
                rect_pts(0, y - 3, 10, y + 3), fill='no')
        + track('TRACK_IN_OUTLINE_NO_STROKE', (2, y), (4, y))
        + track('TRACK_IN_OUTLINE_NO_STROKE', (8, y), (6, y)))
    y = nxt()
    row('TRACK_T_LINE', True,
        fp('A15', y, 'TRACK_T_LINE', [(0, 0, 1), (5, 5, 1)])
        + shape('gr_line', 'TRACK_T_LINE', f'(start 0 {y}) (end 10 {y})')
        + track('TRACK_T_LINE', (5, y + 5), (5, y)))
    y = nxt()
    row('LINE_OTHER_LAYER', False,
        fp('A16', y, 'LINE_OTHER_LAYER', [(0, 0, 1), (10, 0, 1)])
        + shape('gr_line', 'LINE_OTHER_LAYER', f'(start 0 {y}) (end 10 {y})',
                layer='B.Cu'))
    y = nxt()
    row('VIA_IN_FILLED', True,
        fp('A17', y, 'VIA_IN_FILLED', [(-6, 0, 1), (16, 0, 1)])
        + shape('gr_poly', 'VIA_IN_FILLED', rect_pts(0, y - 3, 10, y + 3),
                fill='yes', layer='B.Cu')
        + via('VIA_IN_FILLED', 3, y) + via('VIA_IN_FILLED', 7, y)
        + track('VIA_IN_FILLED', (-6, y), (3, y))
        + track('VIA_IN_FILLED', (16, y), (7, y)))
    y = nxt()
    row('TRACK_END_IN_FILLED', True,
        fp('A18', y, 'TRACK_END_IN_FILLED', [(5, 0, 1), (5, 6, 1)])
        + shape('gr_poly', 'TRACK_END_IN_FILLED', rect_pts(0, y - 2, 10, y + 2),
                fill='yes')
        + track('TRACK_END_IN_FILLED', (5, y + 6), (8, y + 1)))
    y = nxt()
    row('FP_LINE', False,
        fp('A19', y, 'FP_LINE', [(0, 0, 1), (10, 0, 1)],
           extra=f'\t\t(fp_line (start 0 0) (end 10 0) {stroke(0.3)} '
                 f'(layer "F.Cu") (uuid "{uid()}"))\n'))
    y = nxt()
    row('CONTROL_OPEN', False,
        fp('A20', y, 'CONTROL_OPEN', [(0, 0, 1), (10, 0, 1)]))
    # A 0.04 mm pad 0.03 mm inside a filled circle's true edge, at the angle
    # of the parser's 16-gon chord midpoint (0.077 mm outside the chord): the
    # true radius credits it, the polygon would not.
    y = nxt()
    import math
    a = math.pi / 16
    px, py = round(4 * math.cos(a) * 0.9925, 4), round(4 * math.sin(a) * 0.9925, 4)
    row('CIRCLE_TRUE_RADIUS', True,
        fp('A21', y, 'CIRCLE_TRUE_RADIUS', [(5, 0, 1), (5 + px, py, 0.04)])
        + shape('gr_circle', 'CIRCLE_TRUE_RADIUS', f'(center 5 {y}) (end 9 {y})',
                w=0, fill='yes'))
    # A stroke-0 filled poly: a 0.15 track ending 0.1 mm outside its edge
    # leaves a 0.025 mm gap. The parser models the outline at the fab track
    # width (0.3 mm), which would have credited it.
    y = nxt()
    row('DRAWN_EDGE_GAP', False,
        fp('A22', y, 'DRAWN_EDGE_GAP', [(5, 0, 1), (5, 6, 1)])
        + shape('gr_poly', 'DRAWN_EDGE_GAP', rect_pts(0, y - 2, 10, y + 2),
                w=0, fill='yes')
        + track('DRAWN_EDGE_GAP', (5, y + 6), (5, y + 2.1), w=0.15))
    y = nxt()
    edge = (shape('gr_rect', None, f'(start -12 0) (end 25 {y + 10})', w=0.1,
                  layer='Edge.Cuts'))
    return (HEADER + ''.join(t for _e, t in ROWS.values()) + edge + ')\n')


def grade(pcb):
    """{net name: connected} from check_net_connectivity, per row."""
    from check_connected import check_net_connectivity
    out = {}
    for nid, net in pcb.nets.items():
        if net.name not in ROWS:
            continue
        r = check_net_connectivity(
            nid, [s for s in pcb.segments if s.net_id == nid],
            [v for v in pcb.vias if v.net_id == nid],
            pcb.pads_by_net.get(nid, []),
            [z for z in pcb.zones if z.net_id == nid], pcb_data=pcb)
        out[net.name] = bool(r['connected'])
    return out


def text_half(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    got = grade(pcb)
    print("1. text parse: check_net_connectivity against KiCad 10's verdict")
    check("every row graded", set(got) == set(ROWS),
          f"missing {sorted(set(ROWS) - set(got))}")
    for name, (exp, _t) in ROWS.items():
        if name in got:
            check(f"{name}: {'connected' if exp else 'open'}",
                  got[name] == exp, f"graded {'connected' if got[name] else 'open'}")

    print("2. the route step's view")
    from connectivity import get_terminal_component_info
    nid = next(i for i, n in pcb.nets.items() if n.name == 'POLY_FILLED_INSIDE')
    pads = pcb.pads_by_net[nid]
    info = [(None, None, None, p.global_x, p.global_y, p) for p in pads]
    comps, count, segs_by, _vias = get_terminal_component_info(pcb, nid, info)
    check("pads joined only by a filled board graphic are ONE terminal group",
          len(set(comps.values())) == 1, str(comps))
    check("...and the art is never a launch surface",
          not any(s.graphic for ss in segs_by.values() for s in ss)
          and not count, f"{count}")
    from check_connected import classify_unconnected_link
    nid = next(i for i, n in pcb.nets.items() if n.name == 'TRACK_INTO_FILLED')
    y = 10 * (list(ROWS).index('TRACK_INTO_FILLED') + 1)
    a, b = classify_unconnected_link(pcb, nid, (0.0, y - 3, 'F.Cu', 'track'),
                                     (3.0, y, 'F.Cu', 'track'), 'track', 'track')
    check("a link end on art that reaches a pad classifies 'live' (weldable), "
          "not 'graphic'", (a, b) == ('live', 'live'), f"{a}/{b}")
    return got


def kicad_half(path):
    """Under KiCad's python: pcbnew's own verdict per row, and the pcbnew
    parse path graded by the same check_net_connectivity."""
    import pcbnew
    from kicad_parser import build_pcb_data_from_board
    print("3. KiCad: pcbnew's connectivity and the pcbnew parse path")
    b = pcbnew.LoadBoard(path)
    b.BuildConnectivity()
    conn = b.GetConnectivity()
    for f in b.GetFootprints():
        pads = list(f.Pads())
        name = pads[0].GetNetname()
        if name not in ROWS:
            continue
        ids = {i.m_Uuid.AsString() for i in conn.GetConnectedItems(pads[0])}
        ok = pads[1].m_Uuid.AsString() in ids
        check(f"KiCad: {name} {'connected' if ok else 'open'}",
              ok == ROWS[name][0],
              '' if ok == ROWS[name][0] else 'EXPECTED is wrong for this KiCad')
    with contextlib.redirect_stdout(io.StringIO()):
        g = build_pcb_data_from_board(b)
    got = grade(g)
    for name, (exp, _t) in ROWS.items():
        check(f"pcbnew parse path: {name}", got.get(name) == exp,
              f"graded {got.get(name)}")


BOARD_TEXT = build()
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, 'graphic_conduction.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD_TEXT)
    evidence(path, 'generated board')
    KICAD_SKIPPED = None
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        pcbnew = None
    if pcbnew is not None:
        kicad_half(path)
    else:
        text_half(path)
        from kicad_exact_fill import find_kicad_python
        _kp = find_kicad_python()
        if _kp is None:
            KICAD_SKIPPED = "no python with pcbnew was found (KICAD_PYTHON names one)"
        else:
            r = subprocess.run([_kp, '-X', 'utf8', os.path.abspath(__file__)],
                               capture_output=True, text=True, encoding='utf-8',
                               errors='replace', timeout=600)
            for ln in r.stdout.splitlines():
                if ln.startswith('  ') or ln.startswith('3.'):
                    print('  [KiCad python]' + ln)
            ran = 'KiCad: LINE' in r.stdout
            check(f"KiCad half ran under {_kp}: exit {r.returncode}",
                  r.returncode == 0 and ran)
            if r.returncode != 0 or not ran:
                print(r.stdout[-2000:] + r.stderr[-2000:])

print('-' * 60)
if fails:
    print(f"{len(fails)} FAILED: " + '; '.join(fails))
    sys.exit(1)
if KICAD_SKIPPED:
    print(f"SKIP: the KiCad half did not run -- {KICAD_SKIPPED}")
    sys.exit(77)
print('ALL PASS')
