"""Corpus scan: how well does route_planes.is_thermal_pad target true EPs?

For every unrouted corpus board, bucket every SMD pad >= THERMAL_PAD_MIN_MM:
  SELECTED            -- is_thermal_pad True (would get an array on its plane net)
  excluded_prefix     -- big pad but passive/connector reference (C/R/J/...)
  excluded_signature  -- active reference but fails the EP/tab dominance test
Also flags SELECTED pads whose net is not ground/power-ish (suspicious).
Dedupes boards by basename (sets share boards).
"""
import glob
import os
import sys
import time

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _REPO)
sys.path.insert(0, os.path.join(_REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(_REPO, 'py_placer'))  # #522
sys.path.insert(0, os.path.join(_REPO, 'py_tools'))  # #522
from kicad_parser import parse_kicad_pcb
from route_planes import is_thermal_pad
import routing_defaults as d

boards = {}
for p in sorted(glob.glob(os.path.expanduser(
        '~/Documents/kicad_stress_test/boards_unrouted_set*/*.kicad_pcb'))):
    boards.setdefault(os.path.basename(p), p)

sel_rows, pref_rows, sig_rows = [], [], []
suspicious = []
t0 = time.time()
for i, (name, path) in enumerate(sorted(boards.items()), 1):
    try:
        pcb = parse_kicad_pcb(path)
    except Exception as e:
        print(f"PARSE FAIL {name}: {e}", flush=True)
        continue
    import re as _re
    for fp in pcb.footprints.values():
        for pad in fp.pads:
            if pad.drill != 0 or not pad.size_x or not pad.size_y:
                continue
            if min(pad.size_x, pad.size_y) < d.THERMAL_PAD_MIN_MM:
                continue
            row = (name, f"{fp.reference}.{pad.pad_number}",
                   f"{pad.size_x:.1f}x{pad.size_y:.1f}",
                   (fp.footprint_name or '')[-45:], pad.net_name or '-')
            if is_thermal_pad(pad, pcb):
                sel_rows.append(row)
                nn = (pad.net_name or '').upper()
                if not any(k in nn for k in ('GND', 'VSS', 'PGND', 'AGND', 'EARTH',
                                             'VCC', 'VDD', 'PWR', 'V3', 'V5', '3V',
                                             '5V', '12V', 'VIN', 'VBUS', 'POWER',
                                             'BATT', 'PVDD', 'SW', 'VOUT', 'PP')):
                    suspicious.append(row)
            else:
                m = _re.match(r'([A-Za-z]+)', fp.reference or '')
                from route_planes import _THERMAL_REF_PREFIXES
                if not m or m.group(1).upper() not in _THERMAL_REF_PREFIXES:
                    pref_rows.append(row)
                else:
                    sig_rows.append(row)
    if i % 25 == 0:
        print(f"[{i}/{len(boards)}] {time.time()-t0:.0f}s", flush=True)

print(f"\n=== {len(boards)} unique boards scanned in {time.time()-t0:.0f}s ===")
print(f"SELECTED (array-eligible):        {len(sel_rows)} pads on "
      f"{len({r[0] for r in sel_rows})} boards")
print(f"excluded by reference prefix:     {len(pref_rows)} pads")
print(f"excluded by EP-signature test:    {len(sig_rows)} pads")
print(f"SELECTED with non-GND/power net:  {len(suspicious)} pads")

def dump(title, rows, n):
    print(f"\n-- {title} (showing {min(n, len(rows))}/{len(rows)}) --")
    for r in rows[:n]:
        print("  " + " | ".join(r))

dump("SELECTED", sel_rows, 60)
dump("SELECTED but non-GND/power net (suspicious)", suspicious, 30)
dump("active ref, failed EP signature (possible under-trigger)", sig_rows, 40)
dump("excluded by prefix (should be passives/connectors)", pref_rows, 30)
