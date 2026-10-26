#!/usr/bin/env python3
"""#1452: under --escalation off a wide net reaches a fine-pitch pin over a
long edge, at its width everywhere but a pad entry of at most
PAD_ENTRY_NECK_MM.

Since #1282, ``off`` narrows a power edge only when its span is at most the
pad-entry allowance, so a pin that needs a narrow track only where it passes
between its neighbours -- 0.5 mm pitch, a 0.5 mm net, a 0.25 mm request --
was never reached over a longer edge, and the net shipped open with a hint
advising a smaller track. Now the edge is laid at the net's width up to a
landing near the pin and narrow only for the last stretch into it; when no
such route exists the failure names the allowance and the edge's span.

Fixture: a 5 x 5 mm QFN (0.5 mm pitch) whose west pin is on the power net,
two 0805 capacitors on the same net 6-10 mm away, F.Cu only.

Checks:
  1. Unit: the pad-entry zone under ``off`` is the allowance less the taper
     (so zone + taper never exceed it); other policies keep neckdown_length.
  2. Unit: a failure the policy refused gets a hint naming the allowance and
     the span, not a smaller track; any other boxed-in failure keeps its hint.
  3. End to end, ``off``, net at 0.5 mm: all three pads connected, DRC clean
     at the routed 0.2 mm, copper at 0.5 mm, nothing below the 0.25 mm
     request, and every piece under 0.5 mm within the allowance of a pad of
     the net, at most the allowance of it at any one pad.
  4. End to end, ``off``, net at 1.2 mm (no 1.2 mm track can land within the
     allowance of the pin): the pin stays open, the log says why (span and
     allowance), and no hint advises a smaller track.

    python3 tests/test_1452_pad_entry_neck_long_edge.py
"""
import math
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import fab_tiers                                             # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from routing_config import GridRouteConfig                   # noqa: E402
from routing_defaults import PAD_ENTRY_NECK_MM               # noqa: E402
from routing_diagnostics import static_boxin_hint            # noqa: E402
from run_utils import check as run_check, evidence, tool     # noqa: E402
from single_ended_routing import _pad_entry_zone_mm          # noqa: E402

RUN_ALL_TIMEOUT = 300
FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', '1452', 'neck.kicad_pcb')
NET_W, TRACK_W, CLEARANCE = 0.5, 0.25, 0.2
fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


# ---- 1. the zone ------------------------------------------------------------
print("1. _pad_entry_zone_mm")
prev = fab_tiers.get_escalation_policy()
try:
    cfg = GridRouteConfig()
    fab_tiers.set_escalation_policy('off')
    z = _pad_entry_zone_mm(cfg)
    check("off: zone + taper == the allowance",
          abs(z + cfg.neckdown_taper_length - PAD_ENTRY_NECK_MM) < 1e-9,
          f"zone {z}, taper {cfg.neckdown_taper_length}")
    cfg0 = GridRouteConfig(neckdown_taper_length=0.0)
    check("off, no taper: the zone is the allowance",
          abs(_pad_entry_zone_mm(cfg0) - min(cfg0.neckdown_length, PAD_ENTRY_NECK_MM)) < 1e-9)
    for pol in ('fab', 'board'):
        fab_tiers.set_escalation_policy(pol)
        check(f"{pol}: the zone is neckdown_length",
              _pad_entry_zone_mm(cfg) == cfg.neckdown_length)
finally:
    fab_tiers.set_escalation_policy(*prev)

# ---- 2. the hint ------------------------------------------------------------
print("2. static_boxin_hint")
cfg = GridRouteConfig(track_width=TRACK_W, clearance=CLEARANCE)
base = {'failed': True, 'iterations_forward': 5000, 'iterations_backward': 1}
refused = dict(base, pad_entry_refused={
    'span_mm': 7.5, 'allowance_mm': PAD_ENTRY_NECK_MM,
    'net_width_mm': NET_W, 'neck_width_mm': TRACK_W})
hint, verdict = static_boxin_hint(refused, cfg, return_verdict=True)
check("a refused neck's hint names the allowance and the span",
      f"{PAD_ENTRY_NECK_MM:g} mm" in hint and '7.5 mm' in hint, hint)
check("... and does not advise a smaller track", '--track-width' not in hint, hint)
check("... and its verdict says why",
      (verdict or {}).get('verdict') == 'pad_entry_allowance', str(verdict))
hint0, verdict0 = static_boxin_hint(base, cfg, return_verdict=True)
check("control: an ordinary box-in keeps the static hint",
      (verdict0 or {}).get('verdict') == 'boxed_in_static' and 'finer grid' in hint0,
      hint0)


# ---- 3./4. end to end -------------------------------------------------------
def route(tmp, width):
    out = os.path.join(tmp, f'out_{width:g}.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', tool('route.py'), FIXTURE, out,
                   '--nets', 'VCC', '--layers', 'F.Cu',
                   '--track-width', str(TRACK_W), '--clearance', str(CLEARANCE),
                   '--via-size', '0.6', '--via-drill', '0.3',
                   '--escalation', 'off',
                   '--power-nets', 'VCC', '--power-nets-widths', str(width)],
                  accept=True)
    return evidence(out, 'routed board'), r.stdout + r.stderr


def seg_len(s):
    return math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)


def dist_point_seg(px, py, s):
    dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - s.start_x) * dx + (py - s.start_y) * dy) / L2))
    return math.hypot(px - (s.start_x + t * dx), py - (s.start_y + t * dy))


tmp = tempfile.mkdtemp(prefix='t1452_')
try:
    print("3. off, net at 0.5 mm")
    out, log = route(tmp, NET_W)
    r = run_check([sys.executable, '-X', 'utf8', tool('check_connected.py'), out,
                   '--nets', 'VCC'], accept=True)
    check("all VCC pads connected (check_connected)",
          'ALL NETS FULLY CONNECTED' in r.stdout, r.stdout[-300:])
    r = run_check([sys.executable, '-X', 'utf8', tool('check_drc.py'), out,
                   '--clearance', str(CLEARANCE)], accept=True)
    check("DRC clean at the routed clearance", 'NO DRC VIOLATIONS' in r.stdout,
          r.stdout[-300:])
    check("the log names the pad-entry neck", 'pad-entry neck' in log)
    pcb = parse_kicad_pcb(out)
    vcc = next(n for n, net in pcb.nets.items() if net.name == 'VCC')
    segs = [s for s in pcb.segments if s.net_id == vcc]
    pads = [p for p in pcb.pads_by_net.get(vcc, [])]
    check("three VCC pads in the fixture", len(pads) == 3, str(len(pads)))
    check("the trunk is at the net's width",
          any(abs(s.width - NET_W) < 1e-6 for s in segs))
    check("nothing narrower than --track-width",
          min(s.width for s in segs) >= TRACK_W - 1e-6,
          str(min(s.width for s in segs)))
    narrow = [s for s in segs if s.width < NET_W - 1e-6]
    total = sum(seg_len(s) for s in narrow)
    print(f"   under {NET_W} mm: {total:.3f} mm in {len(narrow)} piece(s): "
          + ', '.join(f"{s.width:g}x{seg_len(s):.2f}" for s in narrow))
    far = [s for s in narrow
           if min(max(dist_point_seg(p.global_x, p.global_y, s),
                      math.hypot(s.start_x - p.global_x, s.start_y - p.global_y),
                      math.hypot(s.end_x - p.global_x, s.end_y - p.global_y))
                  for p in pads) > PAD_ENTRY_NECK_MM + 1e-6]
    check("every narrow piece lies within the allowance of a VCC pad",
          not far, str([(s.start_x, s.start_y, s.end_x, s.end_y) for s in far]))
    per_pad = {f"{p.component_ref}.{p.pad_number}": sum(
        seg_len(s) for s in narrow
        if max(math.hypot(s.start_x - p.global_x, s.start_y - p.global_y),
               math.hypot(s.end_x - p.global_x, s.end_y - p.global_y))
        <= PAD_ENTRY_NECK_MM + 1e-6) for p in pads}
    check("at most the allowance of narrow copper at any pad",
          all(v <= PAD_ENTRY_NECK_MM + 1e-6 for v in per_pad.values()), str(per_pad))
    fine = next(p for p in pads if p.component_ref == 'U1')
    check("the fine-pitch pin's entry is the narrow part",
          per_pad[f"U1.{fine.pad_number}"] > 0, str(per_pad))

    print("4. off, net at 1.2 mm: refused, and said so")
    out, log = route(tmp, 1.2)
    r = run_check([sys.executable, '-X', 'utf8', tool('check_connected.py'), out,
                   '--nets', 'VCC'])
    check("the pin stays open", 'Disconnected' in r.stdout, r.stdout[-300:])
    check("the log names the refusal, the span and the allowance",
          'Neck-down refused (--escalation off)' in log and 'spans' in log
          and f"{PAD_ENTRY_NECK_MM:g}mm pad-entry allowance" in log)
    hints = [ln for ln in log.splitlines() if 'Hint:' in ln]
    check("a hint names the pad-entry allowance",
          any('narrow only within' in h for h in hints), '\n'.join(hints))
    check("no hint advises a smaller track",
          not any('--track-width' in h for h in hints), '\n'.join(hints))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print()
if fails:
    print(f"FAILED {len(fails)}: " + '; '.join(fails))
    sys.exit(1)
print("ALL PASS")
