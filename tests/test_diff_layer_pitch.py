#!/usr/bin/env python3
"""A diff pair's P/N centre pitch follows the layer's own width: with per-layer
widths F.Cu 0.197 / In2.Cu 0.154 and a 0.15 mm gap, the pair runs at 0.347 mm
on F.Cu and 0.304 mm on In2.Cu (it kept 0.347 everywhere, a 0.193 mm edge gap
on In2, cm5-io 2026-10-05).

The via placer (`_process_via_positions`) follows it too. It rewrites the
centreline point at each layer change into an approach point on the old layer
and an exit point on the new one; at the call's scalar pitch the whole segment
into and out of every via tapered back to that pitch, so a layer reached only
through vias kept the scalar pitch at both ends, and a layer WIDER than the
call's track width closed the P/N gap to 0.05 mm at the approach. Rows:
  - straight transition, narrow inner layer: every segment on each layer at
    that layer's gap, every track clear of the partner via;
  - the same with the outer layer wider than the call's track width;
  - no per-layer widths: the approach and exit keep the scalar pitch (the
    path a board without layer widths takes, unchanged);
  - every layer narrower than the call's track width under a 1.5x
    track-to-via margin: the via offset still leaves each layer's track its
    margin-scaled offset from the partner via.

The centreline obstacle map carries one extra clearance for every layer, the
call's P/N half-pitch, so on a layer wider than the call's track width the
outer track, now at that layer's wider pitch, can pass a neighbour inside
clearance. Such a segment is necked where it grazes. Rows:
  - no per-layer widths, or no segment wider than the call's track width:
    that pass does not run (the terminal neck is the only one);
  - routed: cap_chain with a 2-layer stackup, --impedance 80 (0.407 mm on both
    layers over a 0.25 mm --track-width) and a foreign track ending just above
    the pair's path. Without the neck the pair grazed it (0.226 mm against
    0.25); here it routes DRC-clean, at 0.407 away from the neighbour.

    python3 tests/test_diff_layer_pitch.py
"""
import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "py_router"))
from diff_pair_routing import (create_parallel_path_from_float, layer_pair_spacing,  # noqa: E402
                               _process_via_positions)
from routing_config import GridRouteConfig, GridCoord  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _pt_seg(p, a, b):
    L = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * (b[0] - a[0])
                                             + (p[1] - a[1]) * (b[1] - a[1])) / L))
    return math.hypot(p[0] - a[0] - t * (b[0] - a[0]), p[1] - a[1] - t * (b[1] - a[1]))


def _seg_seg(s, t):
    return min(_pt_seg(s[0], *t), _pt_seg(s[1], *t), _pt_seg(t[0], *s), _pt_seg(t[1], *s))


def _transition(tw, widths, gap=0.15, clr=0.15, via=0.4, margin=1.0):
    """Lay a straight centreline F.Cu -> In1.Cu through one layer change and
    return (config, P path, N path) after the via placer."""
    cfg = GridRouteConfig(track_width=tw, clearance=clr, via_size=via, via_drill=via / 2,
                          grid_step=0.05, layers=['F.Cu', 'In1.Cu'],
                          layer_widths=dict(widths), diff_pair_gap=gap,
                          routing_clearance_margin=margin, reserve_layer_widths=True)
    coord = GridCoord(cfg.grid_step)
    grid = [(0, 0, 0), (100, 0, 0), (100, 0, 1), (200, 0, 1)]
    flt = [coord.to_float(x, y) + (l,) for x, y, l in grid]
    sp = layer_pair_spacing(cfg)
    p = create_parallel_path_from_float(flt, 1, sp)
    n = create_parallel_path_from_float(flt, -1, sp)
    p, n = _process_via_positions(grid, list(p), list(n), coord, cfg, 1, -1,
                                  (tw + gap) / 2, None, None)
    return cfg, p, n


def _geometry(cfg, p, n):
    """Per layer the smallest P/N edge gap and the smallest edge gap from a
    track to its partner's via; the centre pitch at the approach and the exit."""
    def segs(path):
        return [((a[0], a[1]), (b[0], b[1]), cfg.layers[a[2]])
                for a, b in zip(path, path[1:]) if a[2] == b[2] and a[:2] != b[:2]]

    def vias(path):
        return [(a[0], a[1]) for a, b in zip(path, path[1:]) if a[2] != b[2]]
    gaps = {}
    for s in segs(p):
        for t in segs(n):
            if s[2] == t[2]:
                g = _seg_seg(s[:2], t[:2]) - cfg.get_track_width(s[2])
                gaps[s[2]] = min(gaps.get(s[2], 9.0), g)
    via_gap = {}
    for path, other in ((p, n), (n, p)):
        for s in segs(path):
            for v in vias(other):
                g = _pt_seg(v, *s[:2]) - cfg.via_size / 2 - cfg.get_track_width(s[2]) / 2
                via_gap[s[2]] = min(via_gap.get(s[2], 9.0), g)
    # [start, approach, via (layer 0), via (layer 1), exit, end]
    approach = math.dist(p[1][:2], n[1][:2])
    exit_ = math.dist(p[4][:2], n[4][:2])
    return gaps, via_gap, approach, exit_


def test_layer_spacing():
    widths = {"F.Cu": 0.197, "In2.Cu": 0.154, "B.Cu": 0.197}
    cfg = SimpleNamespace(track_width=0.197, diff_pair_gap=0.15, layers=["F.Cu", "In2.Cu", "B.Cu"],
                          layer_widths=widths, get_track_width=lambda l: widths.get(l, 0.197))
    sp = layer_pair_spacing(cfg)
    check('half pitch per layer', abs(sp(0) - 0.1735) < 1e-9 and abs(sp(1) - 0.152) < 1e-9,
          f'{sp(0):.4f} {sp(1):.4f}')
    for layer, pitch in ((0, 0.347), (1, 0.304)):
        line = [(0.0, 0.0, layer), (5.0, 0.0, layer)]
        p = create_parallel_path_from_float(line, 1, sp)
        n = create_parallel_path_from_float(line, -1, sp)
        got = math.dist(p[0][:2], n[0][:2])
        check(f'layer {layer} offset at {pitch}', abs(got - pitch) < 1e-9, f'{got:.4f}')
    flat = SimpleNamespace(track_width=0.197, diff_pair_gap=0.15, layers=["F.Cu"], layer_widths={})
    check('no layer widths: the scalar', layer_pair_spacing(flat) == (0.197 + 0.15) / 2)


def test_via_transitions():
    gap, clr = 0.15, 0.15
    for label, tw, widths in (
            ('narrow inner layer', 0.197, {'F.Cu': 0.197, 'In1.Cu': 0.154}),
            ('outer layer wider than the call width', 0.1, {'F.Cu': 0.2, 'In1.Cu': 0.1})):
        cfg, p, n = _transition(tw, widths, gap, clr)
        gaps, via_gap, approach, exit_ = _geometry(cfg, p, n)
        want_in = widths['F.Cu'] + gap
        want_out = widths['In1.Cu'] + gap
        check(f'{label}: approach at the F.Cu pitch {want_in:.3f}',
              abs(approach - want_in) < 1e-6, f'{approach:.4f}')
        check(f'{label}: exit at the In1.Cu pitch {want_out:.3f}',
              abs(exit_ - want_out) < 1e-6, f'{exit_:.4f}')
        # (a lateral jog of the pair into the via keeps its pitch to within
        # cos(jog angle): 18 nm on this 5 mm run)
        check(f'{label}: P/N at the declared gap on every layer',
              all(abs(g - gap) < 1e-4 for g in gaps.values()),
              ', '.join(f'{k} {v:.5f}' for k, v in sorted(gaps.items())))
        check(f'{label}: every track clears the partner via',
              min(via_gap.values()) >= clr - 1e-6,
              ', '.join(f'{k} {v:.4f}' for k, v in sorted(via_gap.items())))

    # control: no per-layer widths -> the scalar pitch, as before
    cfg, p, n = _transition(0.2, {}, gap, clr)
    _gaps, via_gap, approach, exit_ = _geometry(cfg, p, n)
    check('no layer widths: approach and exit at the scalar pitch 0.35',
          abs(approach - 0.35) < 1e-9 and abs(exit_ - 0.35) < 1e-9,
          f'{approach:.4f} {exit_:.4f}')
    check('no layer widths: tracks clear the partner via',
          min(via_gap.values()) >= clr - 1e-6, str(via_gap))

    # every layer narrower than the call's width, 1.5x track-to-via margin:
    # each layer's track keeps its margin-scaled offset from the partner via
    m = 1.5
    widths = {'F.Cu': 0.15, 'In1.Cu': 0.2}
    cfg, p, n = _transition(0.3, widths, gap, clr, margin=m)
    _gaps, via_gap, _a, _e = _geometry(cfg, p, n)
    need = {l: (clr + w / 2 + cfg.via_size / 2) * m - cfg.via_size / 2 - w / 2
            for l, w in widths.items()}
    check('narrower layers under a 1.5x margin: the margin holds on each layer',
          all(via_gap[l] >= need[l] - 1e-6 for l in need),
          ', '.join(f'{l} {via_gap[l]:.4f} >= {need[l]:.4f}' for l in sorted(need)))


def test_wide_neck_pass_runs_only_for_wide_layers():
    import contextlib
    import io
    import single_ended_routing
    from diff_pair_routing import _float_path_to_geometry
    from kicad_parser import parse_kicad_pcb
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(os.path.join(root, 'kicad_files', 'cap_chain.kicad_pcb'))
    real, calls = single_ended_routing._neck_terminal_grazes, []

    def counting(segs, *a, **k):
        calls.append(len(segs))
        return real(segs, *a, **k)
    single_ended_routing._neck_terminal_grazes = counting
    try:
        for label, widths, want in (
                ('no layer widths', {}, 1),
                ('no layer wider than the call width', {'F.Cu': 0.25, 'B.Cu': 0.2}, 1),
                ('a layer wider than the call width', {'F.Cu': 0.4, 'B.Cu': 0.2}, 2)):
            cfg = GridRouteConfig(track_width=0.25, clearance=0.25, diff_pair_gap=0.25,
                                  layers=['F.Cu', 'B.Cu'], layer_widths=widths)
            del calls[:]
            _float_path_to_geometry([(20.0, 8.0, 0), (25.0, 8.0, 0)], 1, None, None, 1,
                                    (1, 0), (1, 0), 0, 0, cfg, cfg.layers, pcb_data=pcb)
            check(f'{label}: {want} neck pass(es)', len(calls) == want, str(calls))
    finally:
        single_ended_routing._neck_terminal_grazes = real


STACKUP = ('\t\t(stackup\n'
           '\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t\t(layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4") '
           '(epsilon_r 4.5) (loss_tangent 0.02))\n'
           '\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t)\n')
# a DPB_P track on both layers ending just above DPA's path
WALL = ''.join('\t(segment (start 14 5.5) (end 14 15.45) (width 0.25) (layer "%s") '
               '(net "DPB_P") (uuid "00000000-0000-0000-0000-00000000000%d"))\n' % (lay, k)
               for k, lay in enumerate(('F.Cu', 'B.Cu')))
PRO = {"net_settings": {"classes": [{
    "name": "Default", "clearance": 0.25, "track_width": 0.25, "diff_pair_gap": 0.25,
    "diff_pair_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3}]}}


def test_wide_layer_run_keeps_clear_of_its_neighbour():
    import contextlib
    import io
    import json
    import tempfile
    import route_diff
    from kicad_parser import parse_kicad_pcb
    from check_drc import run_drc
    from check_connected import run_connectivity_check
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    src = open(os.path.join(root, 'kicad_files', 'cap_chain.kicad_pcb'), encoding='utf-8').read()
    i = src.index('\n', src.index('(setup')) + 1
    txt = src[:i] + STACKUP + src[i:]
    k = txt.rstrip().rfind(')')
    txt = txt[:k] + WALL + txt[k:]
    with tempfile.TemporaryDirectory() as td:
        dst, out = os.path.join(td, 'cc.kicad_pcb'), os.path.join(td, 'o.kicad_pcb')
        with open(dst, 'w', encoding='utf-8', newline='') as fh:
            fh.write(txt)
        for stem in ('cc', 'o'):
            with open(os.path.join(td, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
                json.dump(PRO, fh)
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            route_diff.batch_route_diff_pairs(dst, out, ['DPA_*'], impedance=80,
                                              track_width=0.25, diff_pair_gap=0.25,
                                              clearance=0.25, via_size=0.6, via_drill=0.3,
                                              layers=['F.Cu', 'B.Cu'])
        if not os.path.isfile(out):
            check('routed: an output board', False, log.getvalue()[-400:])
            return
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(out)
            drc = run_drc(out, clearance=0.25, quiet=True, print_summary=False,
                          check_sizes=False, clearance_margin=0.0)
            conn = run_connectivity_check(out, net_patterns=['DPA_*'], quiet=True)
    # Margin 0 past the writer's 1 nm coordinate rounding: the pair is coupled
    # exactly at the rule at an off-grid pitch, which the file can read up to
    # sqrt(2) nm inside it.
    from kicad_writer import EDGE_ROUNDING_MM
    drc = [v for v in drc if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM]
    names = {n.net_id: n.name for n in pcb.nets.values()}
    widths = sorted({round(s.width, 3) for s in pcb.segments
                     if names.get(s.net_id, '').startswith('DPA')})
    check('routed: the pair runs at the 0.407 impedance width over a 0.25 call width, '
          'and the map reserves the call\'s half-pitch',
          bool(widths) and max(widths) >= 0.4 and 'extra clearance: 0.250mm' in log.getvalue(),
          str(widths))
    check('routed: DRC clean at the 0.25 clearance', list(drc) == [], str(drc)[:300])
    check('routed: the pair is connected', list(conn) == [], str(conn)[:200])


def main():
    test_layer_spacing()
    test_via_transitions()
    test_wide_neck_pass_runs_only_for_wide_layers()
    test_wide_layer_run_keeps_clear_of_its_neighbour()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nPASS')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
