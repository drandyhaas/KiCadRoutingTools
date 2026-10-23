#!/usr/bin/env python3
"""Where do HUMANS put each functional group? Measured over the stress corpus.

The group priors in `.claude/skills/plan-pcb-placement/references/placement-lessons.md`
("Functional groups") come from this tool, run over the human-placed boards
of stress sets 1-30 (`boards_set<N>/` under the stress dir; building them is
in tests/stress/RUNBOOK.md).

    # one board -> one JSON row
    python3 tests/stress/placement_survey.py board kicad_files/esp_prog.kicad_pcb
    # the corpus -> rows.jsonl (one subprocess per board, so one bad board
    # costs its own row, never the run)
    python3 tests/stress/placement_survey.py run --sets 1-30 --jobs 6 --out rows.jsonl
    # the roll-up the reference page quotes
    python3 tests/stress/placement_survey.py report rows.jsonl

What a row measures:
- **Power conditioners** -- linear regulators, switchers, chargers -- found by
  part number, by a VIN + supply-rail VOUT pin pair, or by a SW/LX pin on an
  inductor's net. Each is measured against the board edge, the connector its
  input is traced to (through at most two series 2-pad parts: fuse, ferrite,
  diode, 0-ohm), the main IC (most pads) and its own input/output caps.
- **Crystals** against their IC and load caps; **ESD/TVS** parts against their
  connector; **decaps** against the supply pin they serve; **LEDs** against the
  edge and their series resistor; **buttons**, **connectors** and **mounting
  holes** against the edge.
- **Habits**: rotations at 90-degree multiples, 2-pad passives on one axis,
  passives aligned in a row or column, use of the back side.

The classifier is heuristic by design -- part-number regexes, pin names, and
net-name voltages to orient a regulator whose pins carry no function -- and it
says "unknown" rather than guess a switcher's input. A part it misreads is an
entry for part-roles.md (what parts are) as well as a fix here.
"""
import argparse
import glob
import json
import math
import os
import re
import statistics as st
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in ('py_placer', 'py_tools', 'py_router'):
    _d = os.path.join(REPO, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)

DEFAULT_STRESS = os.path.expanduser('~/Documents/kicad_stress_test')

LDO_RX = re.compile(
    r'(?<![A-Za-z])(AMS1117|LM1117|LD1117|NCP1117|AZ1117|SPX1117|AP1117|TLV1117|AP211\d|'
    r'AP212\d|AP73\d\d|ME62\d\d|XC62\d\d|MCP17\d\d|MCP18\d\d|LP29\d\d|'
    r'LP59\d\d|LP38\d\d|TLV7\d+|TPS7\d+|RT90\d\d|HT7\d\d\d|L?M?7[89][LM]?\d\d|'
    r'LM317|LM337|LM108\d|LM2937|LM3940|SPX38\d\d|SPX29\d\d|MIC5\d{3}|'
    r'MIC29\d\d|ADP1[2-7]\d|LT17\d\d|LT176\d|LT30\d\d|BU\d\d|BD\d\dG|'
    r'NCP11\d|NCP7\d+|TC1262|TC1185|LM39\d\d|S-1[12]\d\d|TAR5|LDO)', re.I)
SW_RX = re.compile(
    r'(?<![A-Za-z])(TPS5\d+|TPS6\d+|TPS40\d+|MP1584|MP2\d{3}|MP1\d{3}|MP3\d{3}|LM259\d|'
    r'LM2576|LM26\d\d|LMR\d+|LT36\d\d|LT86\d\d|LTC3\d+|AP3012|AP6\d+|'
    r'MT3608|SY80\d\d|SY82\d\d|RT80\d\d|RT88\d\d|RT6\d{3}|XL[46]0\d\d|'
    r'MAX17\d+|MAX16\d+|TLV62\d+|MCP16\d\d|ADP2\d{3}|LM5\d{3}|ISL8\d+|'
    r'AOZ\d+|BUCK|BOOST|DC-?DC)', re.I)
CHG_RX = re.compile(r'(?<![A-Za-z])(MCP7383|TP40\d\d|BQ2[45]\d+|LTC405\d|MAX155\d|'
                    r'IP53\d\d|CN30\d\d|LTC4\d{3}|charger)', re.I)
VIN_FN = re.compile(r'^(VIN\d?|IN\d?|VI|PVIN\d?|VINA|VBUS|VCC_?IN|VDD_?IN|'
                    r'INPUT|VBAT_?IN|BAT_?IN|V_?IN)$', re.I)
VOUT_FN = re.compile(r'^(VOUT\d?|OUT\d?|VO)$', re.I)
SUPPLY_NET = re.compile(r'(^|/)(\+|-)?(\d+(\.\d+)?V\d*|V\d+V\d*|VCC|VDD|AVDD|DVDD|'
                        r'VBUS|VBAT|VSYS|VIN|VREF|P\d+V\d*)', re.I)
CONN_PREFIX = ('J', 'P', 'CN', 'CON', 'USB', 'BAT', 'BT', 'PWR', 'JACK', 'X',
               'XS', 'XP', 'K')
POWER_CONN = re.compile(r'(usb|barrel|jack|dc_|terminal|screw|battery|bat|jst|xt30|'
                        r'xt60|molex|power|pwr|phoenix|wago|banana|vbus)', re.I)
# A footprint keyword never makes these a connector: `D_SMA` is a diode
# package, not an SMA jack.
NOT_CONN_PREFIX = ('D', 'R', 'C', 'L', 'U', 'Q', 'F', 'FB', 'TP', 'Y', 'SW', 'LED',
                   'IC', 'RN', 'FL')
SERIES_PREFIX = ('F', 'FB', 'L', 'D', 'R', 'JP', 'SJ', 'FL', 'PTC', 'RV')


def volts(name):
    """A supply voltage read off a net NAME, or None. VIN/RAW/PWR_IN read as
    an input of unknown (high) voltage."""
    n = (name or '').upper().lstrip('/').split('/')[-1]
    if re.search(r'(^|[^A-Z])(VIN|RAW|PWR_?IN|V_?IN|DC_?IN)($|[^A-Z])', n):
        return 99.0
    if 'VBUS' in n or 'USB_5V' in n:
        return 5.0
    if re.search(r'VBAT|BATT', n):
        return 3.7
    m = re.search(r'(\d+)V(\d+)', n)
    if m:
        return float(f"{m.group(1)}.{m.group(2)}")
    m = re.search(r'(\d+(?:\.\d+)?)V', n)
    if m:
        return float(m.group(1))
    return None


def prefix(ref):
    m = re.match(r'[A-Za-z#*]+', ref)
    return (m.group(0) if m else '').upper()


def cu_pads(fp):
    return [p for p in fp.pads if p.pad_type != 'np_thru_hole'
            and any(lay.endswith('.Cu') or lay == '*.Cu' for lay in (p.layers or []))]


def fn(pad):
    return (getattr(pad, 'pinfunction', '') or '').strip()


def outline_geom(pcb, text):
    from shapely.geometry import Polygon, box
    from kicad_parser import extract_board_contours
    try:
        outers, cutouts = extract_board_contours(text)
    except Exception:
        outers, cutouts = [], []
    if outers:
        try:
            poly = Polygon(outers[0], holes=list(cutouts))
            if poly.is_valid and poly.area > 1:
                return poly
            poly = Polygon(outers[0]).buffer(0)
            if poly.area > 1:
                return poly
        except Exception:
            pass
    bb = pcb.board_info.board_bounds
    return box(*bb) if bb else None


def is_connector(ref, fp, cls):
    name = (fp.footprint_name or '').lower()
    pref = prefix(ref)
    if cls in ('testpoint', 'fiducial', 'mount_hole'):
        return False
    if any(k in name for k in ('crystal', 'xtal', 'resonator', 'testpoint',
                               'test_point')):
        return False
    if pref in NOT_CONN_PREFIX:
        return False
    if any(k in name for k in ('connector', 'conn_', 'pinheader', 'pin_header',
                               'socket', 'usb', 'jack', 'terminal', 'barrel',
                               'jst', 'molex', 'header', 'idc', 'battery',
                               'wire', 'screw', 'dc_', 'rj45', 'sma')):
        return len(cu_pads(fp)) >= 2
    return pref in CONN_PREFIX and len(cu_pads(fp)) >= 2 and pref != 'X' \
        or (pref == 'X' and 'crystal' not in name and len(cu_pads(fp)) >= 3)


def _reach(sub_fp, sub_ref, near_fp, near_ref, sub_pads, near_pads):
    """Pad-edge gaps, one per subject pad, exactly as a `proximity[]` claim
    naming these pads is graded (`floorplan.proximity_pads` and
    `proximity_reaches`)."""
    from placement import floorplan
    if not sub_pads or not near_pads:
        return []
    claim = {'ref': sub_ref, 'near': near_ref,
             'pads': {sub_ref: sorted(set(sub_pads)), near_ref: sorted(set(near_pads))}}
    subject, partners, declared = floorplan.proximity_pads(claim, sub_fp, near_fp)
    return [g for g, *_ in floorplan.proximity_reaches(subject, partners, declared)]


def survey(path, stress_dir=DEFAULT_STRESS):
    """One JSON-able row for the board at `path` (see the module docstring)."""
    from shapely.geometry import Point, box
    from kicad_parser import parse_kicad_pcb
    from net_queries import (is_ground_net_name, is_supply_pintype,
                             is_supply_pinfunction)
    from placement.part_class import classify_part

    text = open(path, encoding='utf-8', errors='replace').read()
    pcb = parse_kicad_pcb(path)
    key = os.path.relpath(path, stress_dir) if os.path.abspath(path).startswith(
        os.path.abspath(stress_dir) + os.sep) else path
    poly = outline_geom(pcb, text)
    if poly is None:
        return {'board': key, 'error': 'no outline'}
    minx, miny, maxx, maxy = poly.bounds
    w, h = maxx - minx, maxy - miny
    hm = max(min(w, h) / 2.0, 1e-6)
    diag = math.hypot(w, h)
    bnd = poly.exterior if hasattr(poly, 'exterior') else poly.boundary
    corners = [(minx, miny), (minx, maxy), (maxx, miny), (maxx, maxy)]

    fps = {r: fp for r, fp in pcb.footprints.items() if cu_pads(fp)}
    net_name = {nid: n.name for nid, n in pcb.nets.items()}
    gnd = {nid for nid, nm in net_name.items() if nm and is_ground_net_name(nm)}
    cls = {r: classify_part(fp, r).name for r, fp in fps.items()}

    def pts(ref):
        return [(p.global_x, p.global_y) for p in cu_pads(fps[ref])]

    def ctr(ref):
        q = pts(ref)
        return (sum(x for x, _ in q) / len(q), sum(y for _, y in q) / len(q))

    def onboard(ref):
        return poly.buffer(1.0).contains(Point(*ctr(ref)))

    fps = {r: fp for r, fp in fps.items() if onboard(r)}
    if len(fps) < 4:
        return {'board': key, 'error': 'too few placed parts'}

    def edge_d(ref):
        return min(bnd.distance(Point(x, y)) for x, y in pts(ref))

    def edge_n(ref):
        return min(1.0, bnd.distance(Point(*ctr(ref))) / hm)

    def edge_nm(ref):
        return min(1.0, edge_d(ref) / hm)

    def dmin(a, b):
        return min(math.dist(p, q) for p in pts(a) for q in pts(b))

    def dpads(pa, pb):
        return min(math.dist((p.global_x, p.global_y), (q.global_x, q.global_y))
                   for p in pa for q in pb)

    net_parts = {}
    for r, fp in fps.items():
        for p in cu_pads(fp):
            if p.net_id > 0:
                net_parts.setdefault(p.net_id, set()).add(r)

    def npads(r):
        return len(cu_pads(fps[r]))

    def val(r):
        return (getattr(fps[r], 'value', '') or '')

    two_pad = {r for r in fps if npads(r) == 2}
    caps = {r for r in two_pad if prefix(r) == 'C'}
    conns = {r for r in fps if is_connector(r, fps[r], cls.get(r))}

    # ---- power conditioners -------------------------------------------------
    cond = {}
    for r, fp in fps.items():
        n = npads(r)
        if prefix(r) not in ('U', 'IC', 'VR', 'REG', 'PS', 'PSU', 'DC'):
            continue
        if CHG_RX.search(val(r) + ' ' + (fp.footprint_name or '')):
            cond[r] = 'charger'
        elif SW_RX.search(val(r)) and n <= 40:
            cond[r] = 'switcher'
        elif LDO_RX.search(val(r)) and n <= 12:
            cond[r] = 'linear'
        elif n <= 8 and any(VIN_FN.match(fn(p)) for p in fp.pads) and any(
                VOUT_FN.match(fn(p)) and p.net_id > 0
                and SUPPLY_NET.search(net_name.get(p.net_id, '') or '') for p in fp.pads):
            # IN/OUT pins alone also name amplifiers: the OUT net must be a rail
            cond[r] = 'linear'
    # an IC with a SW/LX pin on an inductor's (non-ground) net is a switcher
    for L in [r for r in two_pad if prefix(r) == 'L']:
        for p in cu_pads(fps[L]):
            if p.net_id <= 0 or p.net_id in gnd:
                continue
            for r in net_parts.get(p.net_id, ()):
                if r in cond or r == L or r not in fps:
                    continue
                if prefix(r) in ('U', 'IC') and 5 <= npads(r) <= 40 and any(
                        re.match(r'^(SW\d?|LX\d?|PH|SWN)$', fn(q), re.I)
                        for q in cu_pads(fps[r]) if q.net_id == p.net_id):
                    cond[r] = 'switcher'

    def in_out_nets(r):
        pads = cu_pads(fps[r])
        vin = {p.net_id for p in pads if VIN_FN.match(fn(p)) and p.net_id > 0}
        vout = {p.net_id for p in pads if re.match(r'^(VOUT\d?|OUT\d?|VO|BAT|VBAT)$',
                                                   fn(p), re.I) and p.net_id > 0}
        if not vin:
            if cond.get(r) != 'linear' or len(pads) > 5:
                return set(), vout   # no pin functions: never guess a switcher's input
            nets = sorted({p.net_id for p in pads if p.net_id > 0 and p.net_id not in gnd})
            if len(nets) != 2:
                return set(), vout
            va = [volts(net_name.get(n, '')) for n in nets]
            if va[0] is None or va[1] is None or va[0] == va[1]:
                return set(), vout   # the names do not say which way it points
            hi = 0 if va[0] > va[1] else 1
            vin, vout = {nets[hi]}, {nets[1 - hi]}
        return vin, vout - vin

    def series_ok(r):
        if r not in two_pad:
            return False
        if prefix(r) == 'R':
            return bool(re.match(r'^0([RΩ]|R0+|\s*ohm)?$|^0\.0', val(r).strip(), re.I))
        return prefix(r) in SERIES_PREFIX

    def entries_for(nets):
        """Connectors reachable from `nets` through <=2 series 2-pad parts,
        with the hop count."""
        seen = set(nets)
        frontier = set(nets)
        found = {}
        for hop in range(3):
            for n in frontier:
                for r in net_parts.get(n, ()):
                    if r in conns and r not in found:
                        found[r] = hop
            if hop == 2:
                break
            nxt = set()
            for n in frontier:
                for r in net_parts.get(n, ()):
                    if r in cond or not series_ok(r):
                        continue
                    for p in cu_pads(fps[r]):
                        if p.net_id > 0 and p.net_id not in seen and p.net_id not in gnd:
                            nxt.add(p.net_id)
            seen |= nxt
            frontier = nxt
        return found

    # main IC: most copper pads among non-conditioner U/IC/A/M parts, >= 8 pads
    ics = [r for r in fps if prefix(r) in ('U', 'IC', 'A', 'M', 'MOD') and r not in cond
           and npads(r) >= 8 and r not in conns]
    main = max(ics, key=lambda r: (npads(r), r), default=None)

    out_by_net = {}
    for r in cond:
        _, vo = in_out_nets(r)
        for n in vo:
            out_by_net.setdefault(n, []).append(r)

    rows_cond = []
    for r, kind in sorted(cond.items()):
        vin, vout = in_out_nets(r)
        ent = entries_for(vin) if vin else {}
        ent.pop(r, None)
        upstream = sorted({u for n in vin for u in out_by_net.get(n, ()) if u != r})
        row = {'ref': r, 'kind': kind, 'value': val(r)[:24], 'pads': npads(r),
               'edge_mm': round(edge_d(r), 2), 'edge_n': round(edge_n(r), 3),
               'edge_nm': round(edge_nm(r), 3), 'side': fps[r].layer, 'upstream': upstream,
               'vin': [net_name.get(n, '') for n in vin][:3],
               'vout': [net_name.get(n, '') for n in vout][:3]}
        if ent:
            e = min(ent, key=lambda c: (dmin(r, c), c))
            de = dmin(r, e)
            others = [x for x in fps if x not in (r, e)]
            closer = sum(1 for x in others if dmin(x, e) < de)
            pw = [c for c in ent if POWER_CONN.search((fps[c].footprint_name or '') + ' ' + val(c))]
            if pw:
                pe = min(pw, key=lambda c: (dmin(r, c), c))
                row.update({'pentry': pe, 'd_pentry': round(dmin(r, pe), 2),
                            'd_pentry_n': round(dmin(r, pe) / diag, 3),
                            'pentry_edge_mm': round(edge_d(pe), 2)})
                if main:
                    row['t_pentry_main'] = round(
                        dmin(r, pe) / max(1e-6, dmin(r, pe) + dmin(r, main)), 3)
            row.update({'n_entries': len(ent),
                        'entry': e, 'entry_hops': ent[e], 'd_entry': round(de, 2),
                        'd_entry_n': round(de / diag, 3),
                        'entry_rank_pct': round(closer / max(1, len(others)), 3),
                        'entry_edge_mm': round(edge_d(e), 2)})
        row['corner_n'] = round(min(math.dist(ctr(r), c) for c in corners) / diag, 3)
        if upstream:
            u = min(upstream, key=lambda u: (dmin(r, u), u))
            row['d_up'] = round(dmin(r, u), 2)
        loads = [x for n in vout for x in net_parts.get(n, ())
                 if x != r and x not in conns and x not in cond and npads(x) >= 6]
        if loads:
            ld = min(loads, key=lambda x: (dmin(r, x), x))
            row['d_load'] = round(dmin(r, ld), 2)
            big = max(loads, key=lambda x: (npads(x), x))
            row['d_bigload'] = round(dmin(r, big), 2)
            if 'd_entry' in row:
                row['t_entry_load'] = round(
                    row['d_entry'] / max(1e-6, row['d_entry'] + row['d_bigload']), 3)
        if main:
            dm = dmin(r, main)
            row['d_main'] = round(dm, 2)
            row['d_main_n'] = round(dm / diag, 3)
            if 'd_entry' in row:
                row['t_entry_main'] = round(row['d_entry'] / max(1e-6, row['d_entry'] + dm), 3)
        # the group: nearest input/output caps, and a switcher's inductor
        pads = cu_pads(fps[r])
        members = {r}
        for nets, key_ in ((vin, 'cin_mm'), (vout, 'cout_mm')):
            cs = [c for c in caps if any(p.net_id in nets for p in cu_pads(fps[c]))
                  and any(p.net_id in gnd for p in cu_pads(fps[c]))]
            if cs:
                own = [p for p in pads if p.net_id in nets]
                c = min(cs, key=lambda c: (dpads(own, cu_pads(fps[c])), c))
                row[key_] = round(dpads(own, cu_pads(fps[c])), 2)
                members.add(c)
        if kind == 'switcher':
            ls = [L for L in two_pad if prefix(L) == 'L' and
                  any(p.net_id in {q.net_id for q in pads} and p.net_id not in gnd
                      and p.net_id not in vin for p in cu_pads(fps[L]))]
            if ls:
                L = min(ls, key=lambda L: (dmin(r, L), L))
                row['l_mm'] = round(dmin(r, L), 2)
                members.add(L)
        if len(members) >= 2:
            xs = [x for m in members for x, _ in pts(m)]
            ys = [y for m in members for _, y in pts(m)]
            gb = box(min(xs), min(ys), max(xs), max(ys))
            intr = [x for x in fps if x not in members and gb.contains(Point(*ctr(x)))]
            row['group_n'] = len(members)
            row['group_span_mm'] = round(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 2)
            row['group_intruders'] = len(intr)
        rows_cond.append(row)

    # ---- other roles -------------------------------------------------------
    def ic_like(r):
        return npads(r) >= 4 and r not in conns

    crystals = []
    for r, fp in fps.items():
        name = (fp.footprint_name or '').lower()
        if not (prefix(r) in ('Y', 'X', 'XTAL', 'Q', 'OSC') and
                any(k in name for k in ('crystal', 'xtal', 'resonator', 'oscillator'))):
            continue
        sig = {p.net_id for p in cu_pads(fp) if p.net_id > 0 and p.net_id not in gnd}
        tgt = [x for n in sig for x in net_parts.get(n, ()) if x != r and ic_like(x)]
        if not tgt:
            continue
        ic = min(tgt, key=lambda x: (dmin(r, x), x))
        lc = []
        for n in sig:
            if len(net_parts.get(n, ())) > 5:
                continue  # a supply rail, not an oscillator net
            cs = [c for c in net_parts.get(n, ()) if c in caps]
            if cs:
                lc.append(min(cs, key=lambda c: (dmin(r, c), c)))
        # The two numbers a `proximity[]` claim grades, measured by the
        # grader's own functions: each crystal leg pad edge to the IC pad on
        # its net, and each load cap's crystal-net pad to the crystal's pad
        # on that net. The farthest leg and the farthest cap, because a claim
        # holds for every leg.
        legs = _reach(fp, r, fps[ic], ic,
                      [p.pad_number for p in cu_pads(fp) if p.net_id in sig],
                      [q.pad_number for q in cu_pads(fps[ic]) if q.net_id in sig])
        cap_gaps = []
        for c in lc:
            cn = {q.net_id for q in cu_pads(fps[c])} & sig
            cap_gaps += _reach(fps[c], c, fp, r,
                               [q.pad_number for q in cu_pads(fps[c]) if q.net_id in cn],
                               [q.pad_number for q in cu_pads(fp) if q.net_id in cn])
        crystals.append({'ref': r, 'ic': ic, 'd_ic': round(dmin(r, ic), 2),
                         'd_ic_rel': round(dmin(r, ic) / max(1e-6, math.sqrt(
                             max(1, npads(ic)))), 3),
                         'load_cap_mm': round(max((dmin(r, c) for c in lc), default=-1), 2),
                         'leg_gap_mm': round(max(legs), 2) if legs else -1,
                         'cap_gap_mm': round(max(cap_gaps), 2) if cap_gaps else -1,
                         'edge_n': round(edge_n(r), 3)})

    esd = []
    for r, fp in fps.items():
        v = (val(r) + ' ' + (fp.footprint_name or '')).lower()
        if not re.search(r'(esd|tvs|usblc|prtr|tpd\d|sp05|pesd|smf|smbj|smaj|'
                         r'rclamp|ip4220|srv05)', v):
            continue
        sig = {p.net_id for p in cu_pads(fp) if p.net_id > 0 and p.net_id not in gnd}
        cs = [x for n in sig for x in net_parts.get(n, ()) if x in conns]
        if cs:
            c = min(cs, key=lambda x: (dmin(r, x), x))
            esd.append({'ref': r, 'conn': c, 'd_conn': round(dmin(r, c), 2),
                        'd_main': round(dmin(r, main), 2) if main else None})

    decaps = []
    for c in caps:
        pads = cu_pads(fps[c])
        if not any(p.net_id in gnd for p in pads):
            continue
        rail = [p for p in pads if p.net_id > 0 and p.net_id not in gnd]
        if not rail:
            continue
        n = rail[0].net_id

        def supply_pads(o, n=n):
            return [q for q in cu_pads(fps[o]) if q.net_id == n and (
                is_supply_pintype(getattr(q, 'pintype', '') or '') or
                is_supply_pinfunction(fn(q)) or SUPPLY_NET.search(net_name.get(n, '') or ''))]
        owners = [x for x in net_parts.get(n, ()) if x != c and npads(x) >= 3
                  and x not in conns and supply_pads(x)]
        if not owners:
            continue
        best = min(((dpads(rail, supply_pads(o)), o) for o in owners))
        decaps.append({'ref': c, 'ic': best[1], 'd_pin': round(best[0], 2),
                       'same_side': fps[c].layer == fps[best[1]].layer})

    leds = []
    for r, fp in fps.items():
        v = (val(r) + ' ' + (fp.footprint_name or '')).lower()
        if prefix(r) not in ('D', 'LED', 'DS') or 'led' not in v or npads(r) > 4:
            continue
        sig = {p.net_id for p in cu_pads(fp) if p.net_id > 0 and p.net_id not in gnd}
        rs = [x for n in sig for x in net_parts.get(n, ())
              if x in two_pad and prefix(x) == 'R' and len(net_parts.get(n, ())) <= 3]
        leds.append({'ref': r, 'edge_mm': round(edge_d(r), 2), 'edge_n': round(edge_n(r), 3),
                     'edge_nm': round(edge_nm(r), 3),
                     'res_mm': round(min((dmin(r, x) for x in rs), default=-1), 2)})

    buttons = []
    for r, fp in fps.items():
        name = (fp.footprint_name or '').lower()
        if prefix(r) in ('SW', 'S', 'BTN', 'K', 'B') and re.search(
                r'(button|switch|tact|push|sw_|b3u|evqp|kmr|ts-)', name) and \
                'slide' not in name:
            buttons.append({'ref': r, 'edge_mm': round(edge_d(r), 2),
                            'edge_n': round(edge_n(r), 3), 'edge_nm': round(edge_nm(r), 3)})

    conn_rows = [{'ref': r, 'edge_mm': round(edge_d(r), 2), 'edge_n': round(edge_n(r), 3),
                  'edge_nm': round(edge_nm(r), 3), 'pads': npads(r)} for r in sorted(conns)]
    holes, holes_mm = [], []
    for r, fp in pcb.footprints.items():
        try:
            if classify_part(fp, r).name == 'mount_hole':
                holes.append(round(min(math.dist((fp.x, fp.y), c) for c in corners) / diag, 3))
                # the larger axis offset from its nearest corner, in mm: "within
                # 8 mm of a corner" on both axes (placement/mount_holes.py)
                holes_mm.append(round(min(max(abs(fp.x - c[0]), abs(fp.y - c[1]))
                                          for c in corners), 2))
        except Exception:
            pass

    allparts = sorted(fps)
    passives = [r for r in two_pad if prefix(r) in ('R', 'C', 'L', 'D', 'FB', 'LED')]

    def axis(r):
        rot = fps[r].rotation or 0
        return round(rot % 180) // 90 if rot % 90 == 0 else None

    axes = [axis(r) for r in passives if axis(r) is not None]
    maj = max(axes.count(0), axes.count(1)) / len(axes) if axes else None
    aligned = 0
    for r in passives:
        xr, yr = fps[r].x, fps[r].y
        if any(o != r and math.dist((xr, yr), (fps[o].x, fps[o].y)) < 4.0 and
               (abs(fps[o].x - xr) < 0.05 or abs(fps[o].y - yr) < 0.05) for o in passives):
            aligned += 1

    return {
        'board': key,
        'w': round(w, 2), 'h': round(h, 2), 'parts': len(allparts),
        'layers': len(pcb.board_info.copper_layers),
        'main': main, 'main_pads': npads(main) if main else 0,
        'main_edge_n': round(edge_n(main), 3) if main else None,
        'all_edge_n': [round(edge_n(r), 3) for r in allparts],
        'all_edge_nm': [round(edge_nm(r), 3) for r in allparts],
        'all_corner_n': [round(min(math.dist(ctr(r), c) for c in corners) / diag, 3)
                         for r in allparts],
        'cond': rows_cond, 'crystals': crystals, 'esd': esd, 'leds': leds,
        'buttons': buttons, 'conns': conn_rows, 'holes_corner_n': holes,
        'holes_corner_mm': holes_mm,
        'decap_d_pin': [d['d_pin'] for d in decaps],
        'decap_same_side': sum(1 for d in decaps if d['same_side']),
        'decap_n': len(decaps),
        'rot90_frac': round(sum(1 for r in allparts if (fps[r].rotation or 0) % 90 == 0)
                            / len(allparts), 3),
        'passive_axis_majority': round(maj, 3) if maj is not None else None,
        'passive_aligned_frac': round(aligned / len(passives), 3) if passives else None,
        'n_passives': len(passives),
        'b_side_frac': round(sum(1 for r in allparts if fps[r].layer == 'B.Cu')
                             / len(allparts), 3),
    }


# ---------------------------------------------------------------------------
# run: the corpus, one subprocess per board
# ---------------------------------------------------------------------------

def parse_sets(spec):
    out = []
    for part in spec.split(','):
        part = part.strip().lower().replace('set', '')
        if '-' in part:
            a, b = part.split('-', 1)
            out.extend(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def run(args):
    boards = sorted(b for s in parse_sets(args.sets) for b in glob.glob(
        os.path.join(args.stress_dir, f'boards_set{s}', '*.kicad_pcb')))
    if not boards:
        print(f'no boards under {args.stress_dir}/boards_set<{args.sets}>/', file=sys.stderr)
        return 2

    def one(b):
        try:
            r = subprocess.run([sys.executable, '-X', 'utf8', os.path.abspath(__file__),
                                'board', b, '--stress-dir', args.stress_dir],
                               capture_output=True, text=True, timeout=args.timeout)
            lines = r.stdout.strip().splitlines()
            return lines[-1] if lines else json.dumps(
                {'board': b, 'error': f'no output rc={r.returncode}'})
        except subprocess.TimeoutExpired:
            return json.dumps({'board': b, 'error': 'timeout'})

    with ThreadPoolExecutor(args.jobs) as ex, open(args.out, 'w') as f:
        for line in ex.map(one, boards):
            f.write(line + '\n')
    bad = sum(1 for line in open(args.out) if '"error"' in line)
    print(f'{len(boards)} board(s) -> {args.out}; {bad} with an error row')
    return 0


# ---------------------------------------------------------------------------
# report: the roll-up
# ---------------------------------------------------------------------------

def _q(xs, nd=2):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return 'n=0'

    def pct(p):
        return xs[min(len(xs) - 1, int(round(p * (len(xs) - 1))))]
    return (f"n={len(xs)} median {round(st.median(xs), nd)} "
            f"[p25 {round(pct(.25), nd)} p75 {round(pct(.75), nd)} p90 {round(pct(.9), nd)}]")


def _frac(xs, pred):
    xs = [x for x in xs if x is not None]
    k = sum(1 for x in xs if pred(x))
    return f"{k}/{len(xs)} = {100 * k / max(1, len(xs)):.0f}%"


def report(rows, out=print):
    q, frac = _q, _frac
    rows = [r for r in rows if 'error' not in r]
    out(f"BOARDS {len(rows)}  (2L {sum(1 for r in rows if r['layers'] == 2)}, "
        f"4L+ {sum(1 for r in rows if r['layers'] >= 4)}); parts {q([r['parts'] for r in rows], 0)}")
    all_nm = [x for r in rows for x in r['all_edge_nm']]
    all_n = [x for r in rows for x in r['all_edge_n']]
    out(f"\nBASELINE all parts: edge_nm (nearest pad to edge / half the short side) {q(all_nm)}")
    out(f"                    edge_n  (centroid)                                  {q(all_n)}")

    cond = [dict(c, _parts=r['parts']) for r in rows for c in r['cond']]
    kinds = {k: sum(1 for c in cond if c['kind'] == k) for k in ('linear', 'switcher', 'charger')}
    out(f"\n=== POWER CONDITIONERS: {len(cond)} on {sum(1 for r in rows if r['cond'])} boards ({kinds})")
    for kind in ('ALL', 'linear', 'switcher', 'charger'):
        cs = cond if kind == 'ALL' else [c for c in cond if c['kind'] == kind]
        if not cs:
            continue
        out(f"\n-- {kind} ({len(cs)})")
        out(f"  edge_nm            {q([c['edge_nm'] for c in cs])}")
        out(f"  edge_mm            {q([c['edge_mm'] for c in cs], 1)}")
        out(f"  within 3 mm of edge {frac([c['edge_mm'] for c in cs], lambda x: x <= 3)}   "
            f"(all parts edge_nm<=0.2: {frac(all_nm, lambda x: x <= 0.2)}; this kind: "
            f"{frac([c['edge_nm'] for c in cs], lambda x: x <= 0.2)})")
        ent = [c for c in cs if 'd_entry' in c]
        out(f"  input traced to a connector: {len(ent)}/{len(cs)}; cascaded (fed by another "
            f"conditioner): {sum(1 for c in cs if c['upstream'])}/{len(cs)}")
        out(f"  d_entry mm         {q([c['d_entry'] for c in ent], 1)}")
        out(f"  d_entry / diag     {q([c['d_entry_n'] for c in ent], 3)}")
        out(f"  entry_rank_pct     {q([c['entry_rank_pct'] for c in ent], 3)}  "
            f"(share of parts nearer the entry than the conditioner)")
        out(f"  t = d_entry/(d_entry+d_main) {q([c.get('t_entry_main') for c in ent], 3)}; "
            f"nearer entry than main IC: {frac([c.get('t_entry_main') for c in ent], lambda x: x < .5)}")
        pe = [c for c in cs if 'd_pentry' in c]
        out(f"  POWER connector (usb/barrel/terminal/battery..) found: {len(pe)}; d_pentry mm "
            f"{q([c['d_pentry'] for c in pe], 1)}; d/diag {q([c['d_pentry_n'] for c in pe], 3)}")
        out(f"     t_pentry_main {q([c.get('t_pentry_main') for c in pe], 3)}; nearer power "
            f"connector than main IC: {frac([c.get('t_pentry_main') for c in pe], lambda x: x < .5)}")
        out(f"  d_main mm          {q([c.get('d_main') for c in cs], 1)};  d_main/diag "
            f"{q([c.get('d_main_n') for c in cs], 3)}")
        out(f"  input cap mm       {q([c.get('cin_mm') for c in cs])};  output cap mm "
            f"{q([c.get('cout_mm') for c in cs])}")
        if kind == 'switcher':
            out(f"  inductor mm        {q([c.get('l_mm') for c in cs])}")
        out(f"  group span mm      {q([c.get('group_span_mm') for c in cs], 1)};  intruders 0: "
            f"{frac([c.get('group_intruders') for c in cs], lambda x: x == 0)}; intruders "
            f"{q([c.get('group_intruders') for c in cs], 1)}")
        out(f"  on B side          {frac([c['side'] for c in cs], lambda x: x == 'B.Cu')}")

    casc = [c for c in cond if c['upstream']]
    pairs = [(c.get('d_up'), c.get('d_bigload')) for c in casc
             if c.get('d_up') is not None and c.get('d_bigload') is not None]
    out(f"\n-- cascaded stage-2 conditioners: {len(casc)}: d_up (to the conditioner feeding it) "
        f"{q([c.get('d_up') for c in casc], 1)}; d_bigload {q([c.get('d_bigload') for c in casc], 1)}; "
        f"nearer upstream than its biggest load {frac(pairs, lambda t: t[0] < t[1])}")
    all_corner = [x for r in rows for x in r['all_corner_n']]
    out(f"\n-- corner: conditioner centroid to nearest outline corner / diag "
        f"{q([c.get('corner_n') for c in cond], 3)} vs all parts {q(all_corner, 3)}")
    out(f"-- conditioner vs its BIGGEST LOAD: d_bigload {q([c.get('d_bigload') for c in cond], 1)}; "
        f"t_entry_load {q([c.get('t_entry_load') for c in cond], 3)}; nearer entry than biggest load "
        f"{frac([c.get('t_entry_load') for c in cond], lambda x: x < .5)}")
    out(f"-- conditioner vs its NEAREST LOAD (any IC on its output): d_load "
        f"{q([c.get('d_load') for c in cond], 1)}")
    out(f"-- power entry connector itself: edge_mm "
        f"{q([c.get('entry_edge_mm') for c in cond if 'entry' in c], 1)}; power-type entry edge_mm "
        f"{q([c.get('pentry_edge_mm') for c in cond if 'pentry' in c], 1)}")
    for lo, hi, nm in ((0, 40, '<40 parts'), (40, 120, '40-120 parts'), (120, 10 ** 6, '>120 parts')):
        cs = [c for c in cond if lo <= c['_parts'] < hi and 'entry_rank_pct' in c]
        out(f"-- {nm}: entry_rank_pct {q([c['entry_rank_pct'] for c in cs], 3)}; nearer entry than "
            f"main {frac([c.get('t_entry_main') for c in cs], lambda x: x < .5)}; edge_nm "
            f"{q([c['edge_nm'] for c in cond if lo <= c['_parts'] < hi])}")

    out("\n=== MAIN IC (most pads)")
    out(f"  edge_n (centroid) {q([r['main_edge_n'] for r in rows if r['main']])} vs all parts {q(all_n)}")
    out(f"  central (edge_n>=0.6): {frac([r['main_edge_n'] for r in rows if r['main']], lambda x: x >= .6)}; "
        f"all parts: {frac(all_n, lambda x: x >= .6)}")

    cr = [c for r in rows for c in r['crystals']]
    out(f"\n=== CRYSTALS {len(cr)}: d_ic (pad gap to the IC) {q([c['d_ic'] for c in cr])}; "
        f"<= 3 mm {frac([c['d_ic'] for c in cr], lambda x: x <= 3)}; load caps (farthest) "
        f"{q([c['load_cap_mm'] for c in cr if c['load_cap_mm'] >= 0])}")
    lg = [c['leg_gap_mm'] for c in cr if c.get('leg_gap_mm', -1) >= 0]
    cg = [c['cap_gap_mm'] for c in cr if c.get('cap_gap_mm', -1) >= 0]
    out(f"  as a proximity[] claim grades them, pad edge to the pad on the same net: "
        f"farthest leg to the IC {q(lg)} (n={len(lg)}); farthest load cap to the "
        f"crystal {q(cg)} (n={len(cg)})")

    es = [e for r in rows for e in r['esd']]
    out(f"\n=== ESD/TVS {len(es)}: d_conn {q([e['d_conn'] for e in es])}; d_main "
        f"{q([e['d_main'] for e in es], 1)}; nearer connector than main IC "
        f"{frac([(e['d_conn'], e['d_main']) for e in es if e['d_main'] is not None], lambda t: t[0] < t[1])}")

    dp = [x for r in rows for x in r['decap_d_pin']]
    out(f"\n=== DECAPS {len(dp)}: pad gap to the supply pin {q(dp)}; <= 2 mm {frac(dp, lambda x: x <= 2)}; "
        f"<= 5 mm {frac(dp, lambda x: x <= 5)}; same side as IC "
        f"{sum(r['decap_same_side'] for r in rows)}/{sum(r['decap_n'] for r in rows)}")
    dp4 = [x for r in rows if r['layers'] >= 4 for x in r['decap_d_pin']]
    dp2 = [x for r in rows if r['layers'] == 2 for x in r['decap_d_pin']]
    out(f"   2L {q(dp2)};  4L+ {q(dp4)}")
    out(f"   same side 2L {sum(r['decap_same_side'] for r in rows if r['layers'] == 2)}/"
        f"{sum(r['decap_n'] for r in rows if r['layers'] == 2)};  4L+ "
        f"{sum(r['decap_same_side'] for r in rows if r['layers'] >= 4)}/"
        f"{sum(r['decap_n'] for r in rows if r['layers'] >= 4)}")

    le = [led for r in rows for led in r['leds']]
    out(f"\n=== LEDS {len(le)}: edge_mm {q([x['edge_mm'] for x in le], 1)}; edge_nm "
        f"{q([x.get('edge_nm') for x in le])}; edge_n {q([x['edge_n'] for x in le])}; "
        f"within 3 mm of edge {frac([x['edge_mm'] for x in le], lambda x: x <= 3)}")
    out(f"   series resistor gap {q([x['res_mm'] for x in le if x['res_mm'] >= 0])}; <= 2 mm "
        f"{frac([x['res_mm'] for x in le if x['res_mm'] >= 0], lambda x: x <= 2)}")

    bt = [b for r in rows for b in r['buttons']]
    out(f"\n=== BUTTONS {len(bt)}: edge_mm {q([b['edge_mm'] for b in bt], 1)}; edge_nm "
        f"{q([b.get('edge_nm') for b in bt])}; edge_n {q([b['edge_n'] for b in bt])}; "
        f"within 3 mm {frac([b['edge_mm'] for b in bt], lambda x: x <= 3)}")

    cn = [c for r in rows for c in r['conns']]
    out(f"\n=== CONNECTORS {len(cn)}: edge_mm {q([c['edge_mm'] for c in cn], 1)}; within 2 mm "
        f"{frac([c['edge_mm'] for c in cn], lambda x: x <= 2)}; within 5 mm "
        f"{frac([c['edge_mm'] for c in cn], lambda x: x <= 5)}")
    big = [c for c in cn if c['pads'] >= 4]
    out(f"   (>=4 pads: within 2 mm {frac([c['edge_mm'] for c in big], lambda x: x <= 2)}; "
        f"within 5 mm {frac([c['edge_mm'] for c in big], lambda x: x <= 5)})")

    ho = [h for r in rows for h in r['holes_corner_n']]
    out(f"\n=== MOUNTING HOLES {len(ho)}: distance to nearest outline corner / diag {q(ho, 3)}; "
        f"<= 0.1 diag {frac(ho, lambda x: x <= .1)}")
    by_count = {}
    for r in rows:
        hm = r.get('holes_corner_mm') or []
        if hm:
            by_count.setdefault(len(hm), []).append(all(h <= 8 for h in hm))
    out("   boards whose holes ALL sit within 8 mm of a corner (both axes), by hole count: "
        + '; '.join(f"{n}: {sum(v)}/{len(v)}" for n, v in sorted(by_count.items())
                    if n <= 8))

    out(f"\n=== HABITS: rot multiple of 90 {q([r['rot90_frac'] for r in rows], 3)}; boards 100% "
        f"{frac([r['rot90_frac'] for r in rows], lambda x: x >= .999)}")
    out(f"   2-pad passives on the majority axis {q([r['passive_axis_majority'] for r in rows], 3)}")
    out(f"   2-pad passives row/column-aligned with a neighbour within 4 mm "
        f"{q([r['passive_aligned_frac'] for r in rows], 3)}")
    out(f"   B-side parts {q([r['b_side_frac'] for r in rows], 3)}; boards with any B part "
        f"{frac([r['b_side_frac'] for r in rows], lambda x: x > 0)}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('board', help='measure one board, print one JSON row')
    b.add_argument('board')
    b.add_argument('--stress-dir', default=DEFAULT_STRESS,
                   help='rows name a board relative to this dir (default: %(default)s)')
    r = sub.add_parser('run', help='measure the corpus into a JSONL file')
    r.add_argument('--stress-dir', default=DEFAULT_STRESS,
                   help='holds boards_set<N>/ (default: %(default)s)')
    r.add_argument('--sets', default='1-30', help='set numbers, e.g. 1-30 or 1,3,5-7 '
                   '(default: %(default)s)')
    r.add_argument('--jobs', type=int, default=6, help='boards in flight (default: %(default)s)')
    r.add_argument('--timeout', type=int, default=180,
                   help='seconds per board before its row says timeout (default: %(default)s)')
    r.add_argument('--out', required=True, help='rows JSONL to write')
    p = sub.add_parser('report', help='roll a rows JSONL up into the habits report')
    p.add_argument('rows')
    args = ap.parse_args(argv)

    if args.cmd == 'board':
        try:
            print(json.dumps(survey(args.board, args.stress_dir)))
        except Exception as ex:  # one board must never stop a corpus run
            print(json.dumps({'board': args.board, 'error': f'{type(ex).__name__}: {ex}'}))
        return 0
    if args.cmd == 'run':
        return run(args)
    report([json.loads(line) for line in open(args.rows)])
    return 0


if __name__ == '__main__':
    sys.exit(main())
