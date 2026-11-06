#!/usr/bin/env python3
"""#1682: a BGA or QFN fanout stays out of another net's filled graphic.

A fanout tests foreign copper by its own means. Its via gates measured to a
graphic's perimeter strokes, so a via wholly inside a filled shape touched none
of them, and the shape's interior reached the under-pad grid nowhere; and copper
of the nets being fanned, a graphic's included, was skipped as "future" copper,
so the strokes of a fanned net's shape did not reach it either. The step reported
failed 0 over a via that KiCad calls a short.

The fixture: a 6 x 6 BGA with every ball fanned, and a filled B.Cu rectangle under
the middle of the array, tagged with one of the fanned nets.

Checks, for each engine:
  1. No via and no track of another net lands in the shape or inside its
     clearance.
  2. The shape's own net may: its ball's via stands in the rect.
  3. Every ball escapes or is named failed.
  4. Control: `(fill no)` leaves only the outline, and a foreign via stands where
     the interior was: the interior is what the test is about.
  5. A rotated part and a flipped part fan out in their own frames: the interior
     must travel with the strokes (`transform_graphic_shapes`).
  6. A shape the same net tagged but LOCKED holds the same.
  7. A net tie's copper: a via of a tied net stays off it (a drill cuts the
     bridge), as the obstacle map holds it.
  8. A QFN under-pad via and stub stay out too.

    python3 tests/test_1682_fanout_filled_graphic.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb                 # noqa: E402
from bga_fanout import generate_bga_fanout               # noqa: E402
from check_drc import filled_graphic_shapes              # noqa: E402

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f' -- {detail}' if detail else ''))
    if not cond:
        failures.append(name)


N, PITCH = 6, 0.8
X0, Y0 = 30.0, 30.0      # the array's centre
OWN = 'N2_2'
# Under the balls of the array's middle
RECT = (X0 - 1.0, Y0 - 1.0, X0 + 1.0, Y0 + 1.0)


def bga_text(fill='yes', rot=0, layer='F.Cu', extra_shape='', locked=''):
    nets = [(0, '')]
    pads, parts = [], []
    other = 'F.Mask' if layer == 'F.Cu' else 'B.Mask'
    nid = 1
    for r in range(N):
        for c in range(N):
            nets.append((nid, f'N{r}_{c}'))
            net = f'(net {nid} "N{r}_{c}")'
            pads.append(f'  (pad "{r}_{c}" smd circle (at {(c - 2.5) * PITCH} {(r - 2.5) * PITCH}) '
                        f'(size 0.55 0.55) (layers "{layer}" "{other}") {net})')
            parts.append(f' (footprint "t:R" (layer "F.Cu") (at {4 + (nid % 18) * 3} {5 + (nid // 18) * 3})\n'
                         f'  (property "Reference" "R{nid}" (at 0 0) (layer "F.SilkS"))\n'
                         f'  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask") {net}))')
            nid += 1
    x0, y0, x1, y1 = RECT
    back = 'F.Cu' if layer == 'B.Cu' else 'B.Cu'
    shape = (f' (gr_rect {locked} (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05) (type default))'
             f' (fill {fill}) (layer "{back}") (net "{OWN}"))\n')
    net_lines = '\n'.join(f' (net {i} "{n}")' for i, n in nets)
    at = f'(at {X0} {Y0} {rot})' if rot else f'(at {X0} {Y0})'
    return ('(kicad_pcb (version 20260206) (generator test)\n'
            ' (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal)'
            ' (25 "Edge.Cuts" user))\n'
            f'{net_lines}\n'
            ' (gr_rect (start 0 0) (end 70 70) (stroke (width 0.05) (type default)) (fill no)'
            ' (layer "Edge.Cuts"))\n'
            f'{shape}{extra_shape}'
            f' (footprint "t:BGA" (layer "{layer}") {at}\n'
            '  (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
            + '\n'.join(pads) + '\n )\n' + '\n'.join(parts) + '\n)\n')


def fan(td, name, text, method, track=0.1, via=0.3, drill=0.15):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        tracks, vias, _rm, failed = generate_bga_fanout(
            pcb.footprints['U1'], pcb, layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
            net_filter=['N*'], track_width=track, clearance=0.1, via_size=via, via_drill=drill,
            escape_method=method, plane_drop='off')
    return pcb, tracks, vias, failed


def in_rect(x, y, keep=0.0):
    """Is (x, y) inside the rect, or within `keep` of it?"""
    x0, y0, x1, y1 = RECT
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy) < keep or (dx == 0.0 and dy == 0.0)


def main():
    print('#1682: a fanout stays out of a foreign filled graphic')
    with tempfile.TemporaryDirectory() as td:
        for method in ('underpad', 'dogbone', 'channel'):
            print(f'-- {method}')
            pcb, tracks, vias, failed = fan(td, f'a_{method}', bga_text(), method)
            own = next(i for i, n in pcb.nets.items() if n.name == OWN)
            names = {i: n.name for i, n in pcb.nets.items()}
            bad_v = [v for v in vias if v['net_id'] != own
                     and in_rect(v['x'], v['y'], v['size'] / 2 + 0.1 + 0.025 - 1e-3)]
            ok('no via of another net in the shape or its clearance', not bad_v,
               f"{[(names[v['net_id']], round(v['x'], 3), round(v['y'], 3)) for v in bad_v][:4]}")
            bad_t = [t for t in tracks if t['net_id'] != own and t['layer'] == 'B.Cu'
                     and (in_rect(*t['start']) or in_rect(*t['end']))]
            ok('no track of another net in it', not bad_t, f'{len(bad_t)}')
            got = {names[t['net_id']] for t in tracks} | {names[v['net_id']] for v in vias}
            missing = {f'N{r}_{c}' for r in range(N) for c in range(N)} - got - set(failed)
            ok('every ball escapes or is named failed', not missing, f'{sorted(missing)[:4]}')
            print(f'     escaped {36 - len(failed)} of 36; failed {failed}')
        print('-- own net and the control')
        pcb, tracks, vias, failed = fan(td, 'own', bga_text(), 'underpad')
        own = next(i for i, n in pcb.nets.items() if n.name == OWN)
        ok("the shape's own net has a via in it",
           any(v['net_id'] == own and in_rect(v['x'], v['y']) for v in vias),
           f"{[(round(v['x'], 2), round(v['y'], 2)) for v in vias if v['net_id'] == own]}")
        p_c, t_c, v_c, f_c = fan(td, 'ctl', bga_text(fill='no'), 'underpad')
        ok('control: (fill no) has no filled shape', not filled_graphic_shapes(p_c))
        own_c = next(i for i, n in p_c.nets.items() if n.name == OWN)
        foreign = [v for v in v_c if v['net_id'] != own_c and in_rect(v['x'], v['y'])]
        ok('control: with (fill no) foreign vias stand inside the outline', foreign,
           f'{len(foreign)}')
        ok('control: with (fill no) the balls that lost the shape escape',
           len(f_c) <= len(failed), f'{len(f_c)} failed, {len(failed)} with the fill')
        print('-- the part in its own frame')
        for label, text, method in (
                ('rotated 30', bga_text(rot=30), 'underpad'),
                ('flipped', bga_text(layer='B.Cu'), 'underpad')):
            pcb, tracks, vias, failed = fan(td, label.replace(' ', '_'), text, method)
            own = next(i for i, n in pcb.nets.items() if n.name == OWN)
            bad = [v for v in vias if v['net_id'] != own and in_rect(v['x'], v['y'])]
            ok(f'{label}: no foreign via in the shape', not bad,
               f"{[(round(v['x'], 3), round(v['y'], 3)) for v in bad][:4]}")
        print('-- a locked shape')
        pcb, tracks, vias, failed = fan(td, 'lock', bga_text(locked='(locked yes)'), 'underpad')
        own = next(i for i, n in pcb.nets.items() if n.name == OWN)
        ok('locked: no foreign via in the shape',
           not [v for v in vias if v['net_id'] != own and in_rect(v['x'], v['y'])])
        print('-- the shared gate')
        from check_drc import FilledCopperGate
        pcb, *_ = fan(td, 'gate', bga_text(), 'underpad')
        gate = FilledCopperGate(pcb)
        own = next(i for i, n in pcb.nets.items() if n.name == OWN)
        other = next(i for i, n in pcb.nets.items() if n.name == 'N0_0')
        ok('a foreign via in the middle is refused',
           gate.via_hit(X0, Y0, 0.3, other, 0.1) is not None)
        ok('a foreign via outside by its clearance is not',
           gate.via_hit(X0 + 1.0 + 0.15 + 0.025 + 0.1 + 0.01, Y0, 0.3, other, 0.1) is None)
        ok('a foreign via inside the clearance band is',
           gate.via_hit(X0 + 1.0 + 0.15 + 0.05, Y0, 0.3, other, 0.1) is not None)
        ok("the shape's own via is not", gate.via_hit(X0, Y0, 0.3, own, 0.1) is None)
        ok('a foreign track across it is refused',
           gate.segment_hit(X0 - 3, Y0, X0 + 3, Y0, 0.1, 'B.Cu', other, 0.1) is not None)
        ok('on another layer it is not',
           gate.segment_hit(X0 - 3, Y0, X0 + 3, Y0, 0.1, 'In1.Cu', other, 0.1) is None)
        ok('a foreign track wholly inside it is refused',
           gate.segment_hit(X0 - 0.1, Y0, X0 + 0.1, Y0, 0.1, 'B.Cu', other, 0.1) is not None)
        ok("the shape's own track is not",
           gate.segment_hit(X0 - 3, Y0, X0 + 3, Y0, 0.1, 'B.Cu', own, 0.1) is None)
        tie = net_tie(td)
        print('-- a net tie')
        from check_drc import FilledCopperGate
        tp, sh, a, b, c = tie
        g = FilledCopperGate(tp)
        ok("a tied net's track may enter the bridge", b in g.track_nets(sh) and a in g.track_nets(sh))
        ok("a tied net's via may not (the drill cuts it)", a not in g.via_nets(sh))
        ok('a third net may not enter at all', c not in g.track_nets(sh))
        qfn_check(td)
    print('FAILED: ' + '; '.join(failures) if failures else 'All checks passed')
    return 1 if failures else 0


def net_tie(td):
    text = ('(kicad_pcb (version 20260206) (generator test)\n'
            ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
            ' (net 0 "") (net 1 "A") (net 2 "B") (net 3 "C")\n'
            ' (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) (type default)) (fill no)'
            ' (layer "Edge.Cuts"))\n'
            ' (footprint "t:NT" (layer "F.Cu") (at 10 10)\n'
            '  (net_tie_pad_groups "1, 2")\n'
            '  (property "Reference" "NT1" (at 0 0) (layer "F.SilkS"))\n'
            '  (fp_rect (start -0.5 -0.2) (end 0.5 0.2) (stroke (width 0) (type default)) (fill yes)'
            ' (layer "F.Cu"))\n'
            '  (pad "1" smd rect (at -0.5 0) (size 0.4 0.4) (layers "F.Cu" "F.Mask") (net 1 "A"))\n'
            '  (pad "2" smd rect (at 0.5 0) (size 0.4 0.4) (layers "F.Cu" "F.Mask") (net 2 "B")))\n'
            ' (footprint "t:R" (layer "F.Cu") (at 20 10)\n'
            '  (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))\n'
            '  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask") (net 3 "C"))))\n')
    path = os.path.join(td, 'tie.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    sh = filled_graphic_shapes(pcb)[0]
    return pcb, sh, 1, 2, 3


def qfn_text():
    pads, nets, parts = [], [(0, '')], []
    nid = 1
    pitch, half = 0.5, 2.0
    for i in range(8):
        off = (i - 3.5) * pitch
        for side, (x, y, w, h) in enumerate(((off, half, 0.3, 0.8), (off, -half, 0.3, 0.8),
                                             (half, off, 0.8, 0.3), (-half, off, 0.8, 0.3))):
            name = f'S{side}_{i}'
            nets.append((nid, name))
            net = f'(net {nid} "{name}")'
            pads.append(f'  (pad "{nid}" smd rect (at {x} {y}) (size {w} {h}) '
                        f'(layers "F.Cu" "F.Mask") {net})')
            parts.append(f' (footprint "t:R" (layer "F.Cu") (at {4 + (nid % 12) * 4} {55 + (nid // 12) * 3})\n'
                         f'  (property "Reference" "R{nid}" (at 0 0) (layer "F.SilkS"))\n'
                         f'  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask") {net}))')
            nid += 1
    # A filled B.Cu ring of one fanned net around the part: every pad's via
    # stub lands in it.
    shape = (f' (gr_rect (start 25 25) (end 35 35) (stroke (width 0.05) (type default))'
             f' (fill yes) (layer "B.Cu") (net "S0_0"))\n')
    net_lines = '\n'.join(f' (net {i} "{n}")' for i, n in nets)
    return ('(kicad_pcb (version 20260206) (generator test)\n'
            ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
            f'{net_lines}\n'
            ' (gr_rect (start 0 0) (end 70 70) (stroke (width 0.05) (type default)) (fill no)'
            ' (layer "Edge.Cuts"))\n'
            f'{shape}'
            ' (footprint "t:QFN" (layer "F.Cu") (at 30 30)\n'
            '  (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
            + '\n'.join(pads) + '\n )\n' + '\n'.join(parts) + '\n)\n')


def qfn_check(td):
    print('-- a QFN under-pad escape')
    from qfn_fanout import generate_qfn_fanout
    path = os.path.join(td, 'qfn.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(qfn_text())
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        tracks, vias, failed = generate_qfn_fanout(
            pcb.footprints['U1'], pcb, track_width=0.1, clearance=0.1, via_size=0.3,
            via_drill=0.15, escape_method='underpad', allow_via_in_pad=True)
    bad = [v for v in vias if v['net_id'] != 1 and 25 < v['x'] < 35 and 25 < v['y'] < 35]
    ok('no via of another net in the shape', not bad, f'{len(bad)} of {len(vias)}')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(qfn_text().replace('(fill yes)', '(fill no)'))
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        tracks, vias, failed = generate_qfn_fanout(
            pcb.footprints['U1'], pcb, track_width=0.1, clearance=0.1, via_size=0.3,
            via_drill=0.15, escape_method='underpad', allow_via_in_pad=True)
    ok('control: with (fill no) the vias stand there',
       len([v for v in vias if 25 < v['x'] < 35 and 25 < v['y'] < 35]) >= 20,
       f'{len(vias)} via(s), {len(failed)} failed')


if __name__ == '__main__':
    sys.exit(main())
