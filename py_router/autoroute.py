#!/usr/bin/env python3
"""Route a board through a deterministic plan built from the board itself.

    python3 py_router/autoroute.py board.kicad_pcb routed.kicad_pcb
    python3 py_router/autoroute.py board.kicad_pcb routed.kicad_pcb --dry-run

The plan-pcb-routing skill has an agent read a board and choose a command
chain. This makes the same kinds of choices from board facts alone -- the
footprints, net names, layer count and the board's own design rules -- and
runs the chain, so routing without an agent gets the steps that remove most
congestion before the router starts:

  1. design rules  the fab clearance floor for every step (a net class above
                   it is aspirational, so the route step caps every class
                   at it), the fab track floor, the working via (fine via
                   for fine-pitch escapes), all from
                   list_nets.read_design_rules
  2. planes        on 4+ layer boards the fewest inner layers that put a
                   plane next to every signal layer (In1+In2 on 4 layers,
                   In1+In4 on 6, In1+In3+In6 on 8): GND, then the dominant
                   rail, then GND again; GND on B.Cu on 2-layer boards;
                   poured first, and signals kept off the inner planes
                   until the route step's later passes
  3. fanout        every grid array with interior depth or fine pitch
                   (bga_fanout) and every fine-pitch four-sided perimeter
                   package (qfn_fanout), and every dense mezzanine (a
                   CM4/CM5's 200+ pads at 0.4 mm, qfn_fanout under the
                   pads), read from the pads and their pitch
                   along a row -- never a connector, socket or magnetics --
                   plane nets excluded; a part that drops escapes or leaves
                   grazes is retried with the fine via (BGA), then with the
                   dogbone (BGA) or under-pad (QFN) method, keeping the attempt
                   that escaped most, then grazed least
  4. decaps        place_fanout_clearance once, after the fanouts; a part it
                   leaves shorted to a BGA's fanout copper is held in a re-fan
                   of that BGA (bga_fanout --immovable-refs, the BGA's whole
                   ladder, every later fanout replayed at its accepted rung),
                   then place_fanout_clearance again, keeping the board with
                   fewer shorts, then fewer dropped escapes
  5. diff pairs    name-detected pairs routed first with route_diff; a pair
                   net it leaves in a DRC violation is ripped and routed
                   again single-ended
  6. route         route.py over every net with the poured nets and rails as
                   power nets, a first pass that keeps signals off the planes,
                   a finer grid around fine-pitch parts, deeper rip-up on
                   congested boards, and leave to rip the stubs of any pair
                   route_diff could not couple; up to two further passes on the result
                   while nets are left open, which may cross the planes at
                   PLANE_FALLBACK_COST, shipping the best pass

--finish runs step 6 alone on a board another chain has already prepared and
routed (an agent's chain, say): the poured planes are read from the board's
pours, and the input board is itself a candidate, so a pass ships only if it
beats it. route.py works on the nets still open, but like any route step it
may rip and reroute existing copper and runs its plane finalize.

Every step runs as its own process in <output>.autoroute/, so a step that
fails is skipped and the chain continues from the last good board. Each route
pass is graded: open nets as check_connected reports them (the KiCad
cross-check off, so the grade is deterministic) and real violations as
check_drc reports them. A pass is kept when it ranks above the best board so
far, completion first: fewer open nets, then fewer violations at the same open
count. The first pass that is not kept ends the passes. A further pass
starts only while the router iterations spent, plus the last pass's again,
stay within --effort-factor times the first pass's -- a forecast from counts
the router reproduces exactly, not a cap on the pass itself -- and not at all
when a pass reported no iterations. If no route pass succeeds, the best board
the chain has ships (with --finish, the input board itself). The chain and
every grade are written to <output>.autoroute/plan.json.

Besides its prose the run prints machine-readable lines, one JSON object
each, which the plugin's Autorouter tab reads (and a script may):

  AUTOROUTE_ANALYSIS: {...}  what analyze() read off the board
  AUTOROUTE_PLAN: {...}      the steps it means to run, in order, each with
                             what it does and why; a row marked optional runs
                             only if the passes before it leave nets open
  AUTOROUTE_STEP: {...}      a step starting (state "running") and ending
                             (state "done" or "failed"), one of each per step
                             run, or a planned step that never ran (state
                             "skipped", with the reason). An ending carries
                             the step's seconds, exit code, a short result,
                             its own summary, and on failure the tail of its
                             log. Steps the run adds (a fanout's next rung, a
                             re-fan, the pair re-route) appear as they start,
                             with the planned row they belong to as `parent`
  AUTOROUTE_SUMMARY: {...}   the shipped board and its grade, last
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'actor'}

import argparse
import fnmatch
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

TOOLS = {
    'planes': os.path.join(HERE, 'route_planes.py'),
    'bga': os.path.join(HERE, 'bga_fanout.py'),
    'qfn': os.path.join(HERE, 'qfn_fanout.py'),
    'decap': os.path.join(REPO, 'py_placer', 'place_fanout_clearance.py'),
    'diff': os.path.join(HERE, 'route_diff.py'),
    'route': os.path.join(HERE, 'route.py'),
}

GRID_FINE_PITCH = 0.5        # a part at or under this pitch routes on 0.05
CONGESTED_NETS = 150         # skill: CONGESTED = fine BGA or > 150 nets
RAIL_WIDTH = 0.3
PLANE_LAYER_COST = -1.0      # inner planes stay solid; a positive price lets signals cross
PLANE_FALLBACK_COST = 3.0    # a later pass's price on a solid plane, the route step's in the plan skill
ROUTE_PASSES = 3             # the first pass plus up to two on its result
EFFORT_FACTOR = 2.0          # --effort-factor's default


def _num(x):
    return round(float(x), 4)


_DASH_DIGIT = re.compile(r'^-\d')


def _exact_ok(name):
    """A net name the tools' argparse accepts as an exact value: it may start
    with '-' only before a digit (cli_nets.pin_dash_digit_values)."""
    return not name.startswith('-') or bool(_DASH_DIGIT.match(name))


def _glob(name):
    """A pattern that selects exactly this net through every selector the
    chain's tools use, and that argparse passes as a value.

    The selectors (net_queries) match a pattern with no '/' against a net's
    last sheet component too ('GND' selects '/Analog/GND'), and read a leading
    '!' as an exclusion. A sheet-qualified name ('/Analog/GND') is already
    full-matched only. Any other name gets its LAST character as a class that
    also holds '/' -- a net name never ends in '/', so the class admits no
    other net -- and a leading '-' or '!' becomes a class that is not read as
    an option or an exclusion: '[--]' (only '-') and '[ -!]' (a space or '!';
    no net name starts with a space). 'GND' -> 'GN[D/]', '-BATT' ->
    '[--]BAT[T/]', '!RESET' -> '[ -!]RESE[T/]'. Glob metacharacters are
    bracketed. `_selects_exactly` still checks each pattern against the
    board's own nets."""
    def esc(c):
        return f'[{c}]' if c in '[]*?' else c

    def tail(c):
        return {']': '[]/]', '!': '[/!]'}.get(c, f'[{c}/]')

    if name.startswith('/'):
        return ''.join(esc(c) for c in name)
    head = {'-': '[--]', '!': '[ -!]'}.get(name[0])
    if len(name) == 1:
        return head or tail(name)
    return (head or esc(name[0])) + ''.join(esc(c) for c in name[1:-1]) + tail(name[-1])


def _selects_exactly(pattern, name, names):
    """True when PATTERN selects NAME and no other of NAMES, read the ways the
    chain's tools read it: net_queries.net_pattern_matches (--nets, the
    fanouts' filters and their '!' exclusions) and plain fnmatchcase
    (route.py --power-nets)."""
    from net_queries import net_pattern_matches
    if pattern.startswith(('!', '-')):
        return False
    return all(net_pattern_matches(n, pattern) == (n == name)
               and fnmatch.fnmatchcase(n, pattern) == (n == name) for n in names)


# --------------------------------------------------------------------------
# Board analysis


#: Footprint-name fragments of parts that are never escape-routed: connectors,
#: sockets and headers (pins reached in place) and magnetics. Matched against
#: the upper-cased library:footprint name.
MEZZANINE_PADS = 200         # the plan skill's dense mezzanine (CM4/CM5): 200+ pads,
MEZZANINE_PITCH = 0.4        # in rows at 0.4 mm, escaped under the pads
NOT_FANOUT_NAMES = ('CONN', 'SOCKET', 'HEADER', 'PINHEADER', 'BUS_M.2', 'M.2_',
                    'USB', 'RJ45', 'HDMI', 'SFP', 'TRANSFORMER', 'TRANS_', 'XFMR',
                    'MAGJACK', 'MAGNETIC')


def _mode(values):
    """The most common value after rounding to 1 um, the smallest on a tie."""
    from collections import Counter
    c = Counter(round(v, 3) for v in values if v > 1e-3)
    return min(c, key=lambda v: (-c[v], v)) if c else None


def pad_shape(fp):
    """('grid', pitch, depth), ('perimeter', pitch, None) or None for FP's pads.

    Read in the footprint's own frame (local_x/local_y), from the copper pads
    (a paste-only aperture is not one; a pin drawn as several pads is its
    largest) left once each marker (a net-less pad unlike the commonest pad,
    such as a BGA's corner outline pads off the ball lattice), each exposed
    pad (a pad over 3x the median area) and anything inside one (its sub-pads)
    is set aside. An unused ball or pin is net-less too, but keeps the common
    size.

    grid: no exposed pad, >= 3 rows and >= 3 columns of 3 or more pads, at
    least one pad off the bounding edges, and the columns and rows each on a lattice of their
    own pitch: the commonest spacing between neighbouring pads of one row (x)
    or one column (y), the two within 2x of each other. The part's pitch is
    the finer of the two; depth is rows from the edge to the centre.
    perimeter (when not a grid): at least two pads on each of the four
    bounding edges away from its corners, and every other pad within 1.5
    pitches of an edge -- a second staggered ring (aQFN) or corner pads set
    in. The pitch is the median spacing between neighbours along a side.
    Anything else -- two rows (a connector, a DFN, a transformer), one row, a
    scatter -- is neither."""
    from collections import Counter
    pads = [p for p in fp.pads if p.pad_type != 'np_thru_hole'
            and any(l.endswith('.Cu') for l in p.layers)]
    dims = lambda p: tuple(sorted((round(p.size_x, 3), round(p.size_y, 3))))
    if pads:
        common = Counter(map(dims, pads)).most_common(1)[0][0]
        pads = [p for p in pads if p.net_id or dims(p) == common]
    if len(pads) < 6:
        return None
    area = sorted(p.size_x * p.size_y for p in pads)
    med = area[len(area) // 2]
    big = [p for p in pads if p.size_x * p.size_y > 3 * med]
    # Pad sizes are in board space; local_x/local_y in the footprint's frame.
    turned = round(fp.rotation / 90) % 2 == 1

    def copper(p):
        return {l for l in p.layers if l.endswith('.Cu')}

    def under(p, q):
        hx, hy = (q.size_y, q.size_x) if turned else (q.size_x, q.size_y)
        shared = '*.Cu' in copper(p) | copper(q) or copper(p) & copper(q)
        return (shared and abs(p.local_x - q.local_x) <= hx / 2
                and abs(p.local_y - q.local_y) <= hy / 2)
    kept = [p for p in pads if p not in big and not any(under(p, q) for q in big)]
    # One land per pin: a pin drawn as two overlapping pads is one pad, and
    # an exposed pad's fingers out to the edge (which carry its number) are
    # one pad, not a side. The number alone is no test: libraries number the
    # exposed pad like a pin (orangecrab's RGY-24 calls it 12).
    largest = {}
    for p in kept:
        if p.pad_number and (p.pad_number not in largest or
                             p.size_x * p.size_y > largest[p.pad_number].size_x * largest[p.pad_number].size_y):
            largest[p.pad_number] = p
    kept = [p for p in kept if not p.pad_number or largest[p.pad_number] is p]
    if len(kept) < 6:
        return None
    # The exposed pad is a big pad in the middle of the part; big pads at its
    # corners (a sensor's mounting lands) are not one.
    kx = [p.local_x for p in kept]
    ky = [p.local_y for p in kept]
    cx, cy = (min(kx) + max(kx)) / 2, (min(ky) + max(ky)) / 2
    exposed = any(abs(q.local_x - cx) <= (max(kx) - min(kx)) / 4
                  and abs(q.local_y - cy) <= (max(ky) - min(ky)) / 4 for q in big)
    tol = 0.02

    def snap(vals):
        # Library coordinates jitter by a micron or two (IMX283: -9.001 and
        # -9.0 are one column); within tol, a value joins the one before it.
        out = {}
        for v in sorted(vals):
            prev = out[max(out)] if out else None
            out[v] = prev if prev is not None and v - prev <= tol else v
        return out
    sx = snap({p.local_x for p in kept})
    sy = snap({p.local_y for p in kept})
    pts = sorted({(sx[p.local_x], sy[p.local_y]) for p in kept})
    if len(pts) < 6:
        return None
    xs = sorted({x for x, _ in pts})
    ys = sorted({y for _, y in pts})
    x0, x1, y0, y1 = xs[0], xs[-1], ys[0], ys[-1]

    def steps(vals):
        v = sorted(vals)
        return [b - a for a, b in zip(v, v[1:])]

    def on_lattice(vals, pitch):
        return all(round(d / pitch) >= 1 and abs(d / pitch - round(d / pitch)) < 0.1
                   for d in steps(vals))

    interior = [(x, y) for x, y in pts
                if x0 + tol < x < x1 - tol and y0 + tol < y < y1 - tol]
    rows = {}
    cols = {}
    for x, y in pts:
        rows.setdefault(y, []).append(x)
        cols.setdefault(x, []).append(y)
    # An exposed pad makes it a QFN, however many rings of pads surround it.
    if (len(xs) >= 3 and len(ys) >= 3 and interior and not exposed
            and sum(len(r) >= 3 for r in rows.values()) >= 3
            and sum(len(c) >= 3 for c in cols.values()) >= 3):
        # Each axis has its own pitch (LPDDR packages: 0.8 x 0.65 mm); the
        # finer one is the part's escape pitch.
        px = _mode([d for r in rows.values() for d in steps(r)])
        py = _mode([d for c in cols.values() for d in steps(c)])
        # Two parallel pad rows a module's width apart (a compute module's
        # board-to-board connectors) also sit on a lattice, just not a square one.
        if (px and py and max(px, py) <= 2 * min(px, py)
                and on_lattice(xs, px) and on_lattice(ys, py)):
            return ('grid', min(px, py), min(len(xs), len(ys)) // 2)
    # A side counts only the pads strictly between the two perpendicular
    # edges: a two-row part's end pads sit on a corner, not on a side.
    mid_x = lambda x: x0 + tol < x < x1 - tol
    mid_y = lambda y: y0 + tol < y < y1 - tol
    sides = {'l': [y for x, y in pts if abs(x - x0) <= tol and mid_y(y)],
             'r': [y for x, y in pts if abs(x - x1) <= tol and mid_y(y)],
             't': [x for x, y in pts if abs(y - y0) <= tol and mid_x(x)],
             'b': [x for x, y in pts if abs(y - y1) <= tol and mid_x(x)]}
    if any(len(v) < 2 for v in sides.values()):
        return None
    d = sorted(s for v in sides.values() for s in steps(v) if s > 1e-3)
    if not d:
        return None
    pitch = d[len(d) // 2]
    depth = max(min(x - x0, x1 - x, y - y0, y1 - y) for x, y in pts)
    return ('perimeter', round(pitch, 3), None) if depth <= 1.5 * pitch + tol else None


def mezzanine_pitch(fp):
    """The pitch of FP when it is a dense board-to-board mezzanine -- at least
    MEZZANINE_PADS surface pads on at most four lines of one axis (two
    connectors of two rows), each line at MEZZANINE_PITCH or finer -- else
    None. Read in the footprint's own frame."""
    pads = [p for p in fp.pads if p.pad_type != 'np_thru_hole'
            and any(l.endswith('.Cu') for l in p.layers)]
    if len(pads) < MEZZANINE_PADS or any((p.drill or 0) > 0 for p in pads):
        return None
    for across, along in (('local_x', 'local_y'), ('local_y', 'local_x')):
        lines = {}
        for p in pads:
            lines.setdefault(round(getattr(p, across), 2), []).append(getattr(p, along))
        if len(lines) > 4:
            continue
        steps = sorted(b - a for v in lines.values()
                       for a, b in zip(sorted(v), sorted(v)[1:]))
        if steps and steps[len(steps) // 2] <= MEZZANINE_PITCH + 1e-3:
            return round(steps[len(steps) // 2], 3)
    return None


def fanout_census(pcb):
    """{ref: (tool or None, pitch or None, why)} for every footprint: the
    decision fanout_parts takes on it and the reason, in words."""
    out = {}
    for ref, fp in pcb.footprints.items():
        name = (fp.footprint_name or '').upper()
        hit = next((k for k in NOT_FANOUT_NAMES if k in name), None)
        if hit:
            out[ref] = (None, None, f'a connector, socket or magnetics by name ({hit})')
            continue
        pads = [p for p in fp.pads if p.pad_type != 'np_thru_hole']
        th = sum(1 for p in pads if (p.drill or 0) > 0)
        if th > len(pads) - th and 'PGA' not in name:
            out[ref] = (None, None, 'mostly through-hole pins')
            continue
        shape = pad_shape(fp)
        if shape is None:
            pitch = mezzanine_pitch(fp)
            if pitch:
                out[ref] = ('mezz', pitch, f'a dense mezzanine ({MEZZANINE_PADS}+ pads '
                                          f'at {pitch:g} mm): escaped under the pads')
            else:
                out[ref] = (None, None, 'neither a grid array nor a four-sided package')
            continue
        kind, pitch, depth = shape
        if kind == 'grid':
            if pitch < 2.0 and (depth > 2 or pitch <= 0.5):
                out[ref] = ('bga', pitch, f'a grid array at {pitch:g} mm pitch, '
                                          f'{depth} rows deep')
            else:
                out[ref] = (None, pitch, f'a grid array at {pitch:g} mm pitch only '
                                         f'{depth} rows deep: routed from its pads')
        elif pitch < 1.27 and (pitch <= 0.5 or (len(pads) > 40 and pitch <= 0.8)):
            out[ref] = ('qfn', pitch, f'a four-sided package at {pitch:g} mm pitch')
        else:
            out[ref] = (None, pitch, f'a four-sided package at {pitch:g} mm pitch: '
                                     f'coarse enough to route from its pads')
    return out


def fanout_parts(pcb):
    """[(ref, tool, pitch)] for the parts the skill fans out (Step 3): a grid
    array (bga_fanout), a four-sided perimeter package (qfn_fanout) or a dense
    mezzanine ('mezz', qfn_fanout under the pads), read from the pads
    themselves -- never a connector or magnetics by name."""
    return sorted((ref, tool, pitch) for ref, (tool, pitch, _why)
                  in fanout_census(pcb).items() if tool)


#: The smallest part the plan names when it is NOT fanned out, with why: a
#: two-pad passive is never a candidate, and listing them all says nothing.
NOT_FANNED_MIN_PADS = 16


def plane_layers(n):
    """Indices of the fewest inner layers of an N-layer stack such that
    every other layer is next to one: each signal layer then has a reference
    plane. The first such set in order, so the same stack always gets the
    same planes."""
    from itertools import combinations
    for k in range(1, n - 1):
        for combo in combinations(range(1, n - 1), k):
            if all(i in combo or i - 1 in combo or i + 1 in combo for i in range(n)):
                return list(combo)
    return list(range(1, n - 1))


def analyze(board, finish=False, plane_cost=PLANE_LAYER_COST):
    from kicad_parser import parse_kicad_pcb, pour_zones
    from list_nets import (read_design_rules, effective_floors,
                           find_power_nets, find_differential_pairs,
                           power_pin_parts, power_net_order)

    pcb = parse_kicad_pcb(board)
    layers = list(pcb.board_info.copper_layers)
    n_layers = len(layers)
    rules = read_design_rules(board)
    eff = rules.get('effective') or effective_floors(rules.get('constraints') or {}, n_layers)

    # A finished board meets the fab floor everywhere; a declared net class
    # above it is aspirational, so every step routes at the floor (#1522).
    clearance = _num(eff['drc_clearance'])
    track = _num(max(eff['fab_track_width'], 0.1))
    via, drill = _num(eff['working_via_diameter']), _num(eff['working_via_drill'])
    fine_via, fine_drill = _num(eff['fine_via_diameter']), _num(eff['fine_via_drill'])

    pairs = []
    for pr in find_differential_pairs(pcb) or []:
        p, n = (pr[0], pr[1]) if isinstance(pr, (tuple, list)) else (pr.p, pr.n)
        pairs += [getattr(p, 'name', p), getattr(n, 'name', n)]

    # Name-matched grounds and supplies, not find_power_nets' pad-count
    # candidates: those include keyboard matrix rows and unconnected pins.
    gnd, vcc, _cand = find_power_nets(pcb)
    # A rail no pattern names (V+, /CM5_3V3, /M2_3v3) still carries power
    # pins of two or more parts; power pins of one part are that part's own
    # supply tap, an enable or a connector's misdeclared pins, and a diff
    # pair is a pair whatever its pins say.
    named = {n for n, _ in gnd + vcc} | set(pairs)
    vcc = vcc + [(n.name, len(n.pads)) for n in pcb.nets.values()
                 if n.name and n.name not in named
                 and len(power_pin_parts(n)) >= 2]
    real = lambda t: t[1] >= 3 and not t[0].lower().startswith('unconnected-')
    # ties by name, never by board file order (#1755)
    gnd = sorted(filter(real, gnd), key=power_net_order)
    rails = sorted(filter(real, vcc), key=power_net_order)
    planes = []                         # [(net, layer)]
    if gnd:
        if n_layers >= 4:
            nets = [gnd[0][0]] * len(plane_layers(n_layers))
            if rails and rails[0][1] >= 10 and len(nets) > 1:
                nets[1] = rails[0][0]
            planes = [(n, layers[i]) for n, i in zip(nets, plane_layers(n_layers))]
        else:
            planes.append((gnd[0][0], layers[-1]))
    # route_planes takes EXACT names: a name argparse cannot carry stays a
    # power net (passed as a pattern) instead of a plane.
    planes = [(n, l) for n, l in planes if _exact_ok(n)]
    plane_nets = list(dict.fromkeys(n for n, _ in planes))
    power_nets = plane_nets + [n for n, _ in rails if n not in plane_nets]
    power_nets += [n for n, _ in gnd[1:] if n not in power_nets]

    if finish:
        # The board's own pours: their nets are power nets, and an INNER
        # layer carrying one is a plane the signals should keep off. Teardrops
        # are zones too, but routing output on a signal net, not a pour.
        zones = [z for z in pour_zones(pcb.zones) if not z.in_footprint and z.net_name]
        planes = sorted({(z.net_name, z.layer) for z in zones})
        plane_nets = sorted({n for n, _ in planes})
        power_nets = plane_nets + [n for n, _ in rails if n not in plane_nets]
        power_nets += [n for n, _ in gnd if n not in power_nets]
    solid = {l for _, l in planes if l not in (layers[0], layers[-1])}
    if n_layers == 2:
        layer_costs = [1.0, 1.5]
    else:
        layer_costs = [plane_cost if l in solid else 1.0 for l in layers]

    census = fanout_census(pcb)
    parts = sorted((ref, tool, pitch) for ref, (tool, pitch, _w) in census.items() if tool)
    fanout_why = {ref: census[ref][2] for ref, _t, _p in parts}
    not_fanned = sorted(
        ({'ref': ref, 'pads': n, 'why': census[ref][2]}
         for ref, fp in pcb.footprints.items() if not census[ref][0]
         for n in [sum(1 for p in fp.pads if p.pad_type != 'np_thru_hole')]
         if n >= NOT_FANNED_MIN_PADS),
        key=lambda d: (-d['pads'], d['ref']))
    fine = any(p <= GRID_FINE_PITCH for _, _, p in parts)
    grid = 0.05 if fine else 0.1
    n_nets = sum(1 for n in pcb.nets.values() if n.name and len(n.pads) >= 2)
    congested = n_nets > CONGESTED_NETS or any(t == 'bga' and p <= 0.8 for _, t, p in parts)

    # Every net the chain names by pattern, with the pattern that selects it
    # alone on this board. A net no pattern isolates is left out of the
    # lists that name it (a diff pair goes as a pair) and reported.
    names = sorted({n.name for n in pcb.nets.values() if n.name})
    globs, unnamed = {}, []
    for n in dict.fromkeys(plane_nets + power_nets + pairs):
        g = _glob(n)
        if _selects_exactly(g, n, names):
            globs[n] = g
        else:
            unnamed.append(n)
    if unnamed:
        print(f"AUTOROUTE: no pattern selects only {', '.join(unnamed)} on this "
              f"board; left out of the power, exclusion and pair lists", flush=True)
    power_nets = [n for n in power_nets if n in globs]
    pairs = [n for i in range(0, len(pairs) - 1, 2) for n in pairs[i:i + 2]
             if pairs[i] in globs and pairs[i + 1] in globs]

    return dict(layers=layers, clearance=clearance, track=track, via=via,
                drill=drill, fine_via=fine_via, fine_drill=fine_drill, planes=planes,
                plane_nets=plane_nets, power_nets=power_nets,
                layer_costs=layer_costs, parts=parts, grid=grid,
                congested=congested, n_nets=n_nets, pair_nets=pairs,
                globs=globs,
                # Why, for a reader (AUTOROUTE_PLAN, the plugin's tab): where
                # the rules came from, why each part is fanned out or not,
                # and the grounds and rails found by name and power pins.
                rules_source=(os.path.basename(rules['source']) if rules.get('source')
                              else None),
                fanout_why=fanout_why, not_fanned=not_fanned,
                grounds=[n for n, _ in gnd], rails=[n for n, _ in rails],
                unselectable=unnamed)


# --------------------------------------------------------------------------
# Running steps


#: Prints the nets check_connected leaves incomplete (unrouted or broken),
#: one JSON list on its own line; run in a child so its output and state stay
#: out of this process.
_CONN_PROBE = (
    "import contextlib, io, json, sys\n"
    "sys.path.insert(0, sys.argv[1])\n"
    "from check_connected import run_connectivity_check\n"
    "with contextlib.redirect_stdout(io.StringIO()):\n"
    "    issues = run_connectivity_check(sys.argv[2], quiet=True)\n"
    "print('AUTOROUTE_OPEN: ' + json.dumps(sorted({i['net_name'] for i in issues"
    " if i.get('net_name')})))\n")


def _recorded_clearance(board):
    """The clearance BOARD's .kicad_pro records, or None. check_drc grades at
    the board's own recorded floors when there is one -- copper a step routed
    below the chain's clearance (a fine tap) is legal there -- and only
    otherwise at the chain's clearance, as the corpus grader does."""
    from fix_kicad_drc_settings import project_copper_clearance
    try:
        with open(board[:-len('.kicad_pcb')] + '.kicad_pro') as fh:
            return project_copper_clearance(json.load(fh))
    except (OSError, ValueError):
        return None


def _child_env():
    env = dict(os.environ)
    env.pop('REDO_MANIFEST', None)   # the parent records the one invocation
    return env


def emit(token, payload):
    """One machine-readable line: TOKEN, a colon, PAYLOAD as JSON."""
    print(f"{token}: " + json.dumps(payload, sort_keys=True, default=str), flush=True)


#: Lines of a failed step's log its ending event carries.
ERROR_TAIL_LINES = 15

#: Fault injection for the tests of a failed step (this file's and the
#: plugin tab's): the steps named in KRT_AUTOROUTE_FAIL_STEPS (tags, comma
#: separated) run a stand-in that prints FAIL_STEP_MESSAGE and exits 3, so
#: the chain's failure path runs on a real board. Unset, nothing changes.
FAIL_STEPS_ENV = 'KRT_AUTOROUTE_FAIL_STEPS'
FAIL_STEP_MESSAGE = 'step failed on purpose (KRT_AUTOROUTE_FAIL_STEPS)'


def _fail_steps():
    return {s.strip() for s in os.environ.get(FAIL_STEPS_ENV, '').split(',') if s.strip()}

#: place_fanout_clearance's own summary line (pinned byte for byte by its
#: tests); it prints a JSON_SUMMARY only when given an intent.
_DECAP_MOVED = re.compile(r'^Moved (\d+) cap\(s\); resolved (\d+)/(\d+) initial '
                          r'violations; (\d+) unresolved', re.M)


def _tail(text, n=ERROR_TAIL_LINES):
    lines = [ln for ln in (text or '').splitlines() if ln.strip()]
    return lines[-n:]


def step_summary(kind, text, summary, summary_min):
    """(summary dict, one-line result) for a finished step of tool KIND, from
    its log TEXT and the JSON_SUMMARY / JSON_SUMMARY_MIN it printed: the
    numbers a reader needs to judge the step, not the whole summary."""
    s = summary or {}
    if kind in ('bga', 'qfn'):
        d = {k: s[k] for k in ('requested', 'escaped', 'failed', 'escape_method',
                               'via_size', 'via_drill') if k in s}
        d['grazes'] = rung_grazes(s)
        d['unescaped_nets'] = len(s.get('unescaped_nets') or ())
        if not s:
            return d, 'no fanout summary printed'
        return d, (f"escaped {d.get('escaped', 0)}/{d.get('requested', 0)}, "
                   f"{d.get('failed', 0)} dropped, {d['grazes']} graze(s)")
    if kind == 'planes':
        zones = [f"{z.get('net')} on {z.get('layer')}" for z in s.get('plane_zones') or ()]
        d = {'zones': zones, 'status': s.get('status')}
        return d, ('poured ' + ', '.join(zones)) if zones else 'no zone poured'
    if kind == 'decap':
        m = _DECAP_MOVED.search(text or '')
        if not m:
            return {}, 'no summary line'
        d = dict(zip(('moved', 'resolved', 'initial', 'unresolved'),
                     map(int, m.groups())))
        return d, (f"moved {d['moved']} cap(s), resolved {d['resolved']}/"
                   f"{d['initial']}, {d['unresolved']} unresolved")
    if kind == 'diff':
        if not s:
            if 'nothing to route' in (text or ''):
                return {}, 'nothing to route: every pair already connected'
            return {}, 'no summary printed'

        def n(k):
            return len(s.get(k) or ())
        d = {'routed': n('routed_diff_pairs'), 'failed': n('failed_diff_pairs'),
             'single_ended': n('single_ended_diff_pairs'),
             'uncoupled': n('uncoupled_diff_pairs'),
             'followup_nets': list(s.get('single_ended_followup_nets') or ()),
             'vias': s.get('total_vias')}
        return d, (f"{d['routed']} pair(s) routed, {d['failed']} failed"
                   + (f", {d['single_ended']} left to the route step"
                      if d['single_ended'] else ''))
    if kind == 'route':
        m = summary_min or {}
        d = {k: m[k] for k in ('routed', 'failed', 'vias', 'failed_single',
                               'open_single', 'multipoint_deficit') if k in m}
        if 'pad_pairs_open' in m:
            d['open_nets'] = list((m['pad_pairs_open'] or {}).get('nets') or ())
        if not m:
            return d, 'no route summary printed'
        return d, (f"routed {d.get('routed', 0)}, {d.get('failed', 0)} failed, "
                   f"{d.get('vias', 0)} via(s)")
    return {}, ''


class ChainEvents:
    """A chain's event side: the steps that started, the planned steps that
    will not and why, the decisions it made, and a step whose ending waits on
    its grade. Its state is made on first use, so a scripted chain in a test
    gets it by inheriting this class alone."""

    held = None                     # a step whose ending waits on its grade

    @property
    def started(self):              # tags with a `running` event
        return self.__dict__.setdefault('_started', set())

    @property
    def skipped(self):              # planned tag -> why it did not run
        return self.__dict__.setdefault('_skipped', {})

    @property
    def decisions(self):            # AUTOROUTE_DECISION payloads, in order
        return self.__dict__.setdefault('_decisions', [])

    def skip(self, tag, reason):
        """A planned step that will not run, and why (emitted by main)."""
        self.skipped.setdefault(tag, reason)

    def decide(self, tag, kept, why, **extra):
        """A choice the chain made about step TAG -- the fanout rung it kept,
        a re-fan or route pass kept or not -- and why: AUTOROUTE_DECISION."""
        self.end_step()
        d = dict(extra, tag=tag, kept=kept, why=why)
        self.decisions.append(d)
        emit('AUTOROUTE_DECISION', d)

    def end_step(self, result=None, **extra):
        """Emit the held step's ending, with RESULT and EXTRA (a grade)."""
        ev, self.held = self.held, None
        if ev is None:
            return
        if result is not None:
            ev['result'] = result
        ev.update(extra)
        self.steps[ev.pop('_index')].update(
            {k: ev[k] for k in ('result', 'kept') if k in ev})
        emit('AUTOROUTE_STEP', ev)


class Chain(ChainEvents):
    def __init__(self, board, work, clearance=None, baseline=None):
        self.work = work
        self.cur = os.path.abspath(board)
        self.n = 0
        self.steps = []
        self.last_iterations = None
        self.clearance = clearance
        self.baseline = baseline

    def grade(self, board):
        """{'open': n, 'open_nets': [...], 'drc': n} for BOARD, or None when a
        checker could not grade it.

        open: the nets check_connected reports unrouted or broken, every one
        (no capped name list, no pad counts), with its KiCad cross-check off
        so the same board always grades the same. drc: check_drc's real
        violations at the board's recorded floors (else the chain's
        clearance), the chain's input as baseline."""
        env = _child_env()
        env['KICAD_NO_GRADE_RECONCILE'] = '1'
        r = subprocess.run([sys.executable, '-X', 'utf8', '-c', _CONN_PROBE, HERE, board],
                           capture_output=True, text=True, env=env)
        m = re.search(r'^AUTOROUTE_OPEN: (\[.*\])$', r.stdout, re.M)
        if r.returncode != 0 or not m:
            print(f"  grade: check_connected failed on {os.path.basename(board)}",
                  flush=True)
            return None
        open_nets = json.loads(m.group(1))
        d = self.check_drc(board)
        if d is None:
            return None
        return {'open': len(open_nets), 'open_nets': open_nets, 'drc': int(d['violations']),
                'drc_items': _named_items(d)}

    def check_drc(self, board):
        """check_drc's JSON report for BOARD at the board's recorded floors
        (else the chain's clearance), the chain's input as baseline; None when
        it could not grade."""
        fd, jpath = tempfile.mkstemp(suffix='.json', dir=self.work)
        os.close(fd)
        argv = [sys.executable, '-X', 'utf8', os.path.join(HERE, 'check_drc.py'),
                board, '--quiet', '--json', jpath]
        if self.clearance is not None and _recorded_clearance(board) is None:
            argv += ['--clearance', str(self.clearance)]
        if self.baseline:
            argv += ['--baseline', self.baseline]
        subprocess.run(argv, capture_output=True, text=True, env=_child_env())
        try:
            with open(jpath) as fh:
                d = json.load(fh)
            int(d['violations'])
            return d
        except (OSError, ValueError, KeyError, TypeError):
            print(f"  grade: check_drc failed on {os.path.basename(board)}", flush=True)
            return None
        finally:
            os.remove(jpath)

    def drc_nets(self, board):
        """The nets named in BOARD's DRC violations that involve a track or
        via -- copper a routing step lays -- or None when ungraded. An item
        check_drc accepts (a protected via in a pad) is no violation."""
        d = self.check_drc(board)
        if d is None:
            return None
        return {n for it in d.get('items') or []
                if not it.get('accepted')
                and any(k in it.get('type', '') for k in ('segment', 'via', 'track'))
                for n in (it.get('net1'), it.get('net2')) if n}

    def run(self, kind, args, tag, parent=None, hold=False):
        """Run one tool from the current board to a fresh output; returns
        (output path or None, JSON_SUMMARY dict or None, JSON_SUMMARY_MIN).
        ``last_iterations`` is the router iterations its JSON_SUMMARY lines
        report, summed -- the same effort count the replay harness uses --
        or None when the run printed no JSON_SUMMARY at all.

        Emits the step's `running` event, and its ending -- unless HOLD and
        the step produced a board, when the caller ends it with end_step
        once it is graded. PARENT names the planned row a step the plan did
        not list belongs to (a fanout rung, a re-fan)."""
        self.end_step()
        self.n += 1
        out = os.path.join(self.work, f'step{self.n:02d}_{tag}.kicad_pcb')
        argv = [sys.executable, '-u', '-X', 'utf8', TOOLS[kind]] + \
            [a.replace('{IN}', self.cur).replace('{OUT}', out) for a in args]
        log = out[:-len('.kicad_pcb')] + '.log'
        ev = {'n': self.n, 'tag': tag, 'tool': kind, 'parent': parent,
              'cmd': [os.path.basename(TOOLS[kind])] + argv[5:], 'log': log,
              'board': out, 'input': self.cur}
        self.started.add(tag)
        emit('AUTOROUTE_STEP', dict(ev, state='running', t=round(time.time(), 3)))
        if tag in _fail_steps():
            argv = [sys.executable, '-c', 'import sys; print("' + FAIL_STEP_MESSAGE
                    + '", file=sys.stderr); sys.exit(3)']
        t0 = time.time()
        with open(log, 'w') as fh:
            rc = subprocess.run(argv, cwd=self.work, stdout=fh,
                                stderr=subprocess.STDOUT, env=_child_env()).returncode
        seconds = round(time.time() - t0, 1)
        text = open(log, errors='replace').read()
        summary = summary_min = None
        self.last_iterations = None
        for m in re.finditer(r'^JSON_SUMMARY: (\{.*\})$', text, re.M):
            try:
                summary = json.loads(m.group(1))
            except ValueError:
                continue
            self.last_iterations = ((self.last_iterations or 0)
                                    + int(summary.get('total_iterations') or 0))
        m = re.search(r'^JSON_SUMMARY_MIN: (\{.*\})$', text, re.M)
        if m:
            try:
                summary_min = json.loads(m.group(1))
            except ValueError:
                pass
        ok = rc == 0 and os.path.exists(out)
        compact, result = step_summary(kind, text, summary, summary_min)
        if not ok:
            # A failed step's numbers describe no board the chain keeps.
            compact = {}
            result = ((f"exit code {rc}" if rc else "exit code 0 but no output board")
                      + "; the chain continues from the board before it")
        self.steps.append({'tool': kind, 'tag': tag, 'rc': rc, 'ok': ok,
                           'iterations': self.last_iterations,
                           'argv': argv[4:], 'log': os.path.basename(log),
                           'seconds': seconds, 'summary': compact,
                           'result': result})
        print(f"  [{self.n:02d}] {tag}: rc={rc}{'' if ok else ' (skipped)'}",
              flush=True)
        end = dict(ev, state='done' if ok else 'failed', rc=rc, seconds=seconds,
                   summary=compact, result=result, iterations=self.last_iterations,
                   t=round(time.time(), 3))
        if not ok:
            end['board'] = None
            end['error_tail'] = _tail(text)
        if hold and ok:
            self.held = dict(end, _index=len(self.steps) - 1)
        else:
            emit('AUTOROUTE_STEP', end)
        return (out if ok else None), summary, summary_min

    def accept(self, out):
        if out:
            self.cur = out


def plan_steps(a, finish=False, effort_factor=2.0):
    """AUTOROUTE_PLAN's payload: the design rules the chain routes at and the
    steps it means to run, in order, as {tag, tool, what, why, optional}. A
    step marked optional runs only if what comes before it calls for it;
    steps a run adds (a fanout's next rung, a re-fan, the pair re-route)
    are not listed, and appear in the events with their planned row as
    parent."""
    rows = []

    def row(tag, tool, what, why, optional=False):
        rows.append({'tag': tag, 'tool': tool, 'what': what, 'why': why,
                     'optional': optional})
    pours = ', '.join(f'{n} on {l}' for n, l in a['planes'])
    if finish:
        row('input', 'grade', 'grade the input board',
            'it is itself a candidate: a route pass ships only if it beats it')
    else:
        if a['planes']:
            row('planes', 'planes', f'pour {pours}',
                'a GND pour on the bottom of a 2-layer board' if len(a['layers']) == 2
                else 'the fewest inner planes that put one next to every signal '
                     'layer; GND, then the dominant rail')
        for ref, tool, pitch in a['parts']:
            row(f'fanout_{ref}', 'bga' if tool == 'bga' else 'qfn',
                f"fan out {ref} with {'bga_fanout' if tool == 'bga' else 'qfn_fanout'}"
                + (' under the pads' if tool == 'mezz' else ''),
                a.get('fanout_why', {}).get(ref, '') + '; a rung that drops escapes '
                'or grazes is retried with the next')
        if a['parts']:
            row('decap', 'decap', 'move decoupling caps off the fanout copper '
                '(place_fanout_clearance)',
                'once, after the fanouts, when one escaped a ball; a part it leaves '
                'shorted to a BGA\'s copper re-fans that BGA holding the part',
                optional=True)
        if a['pair_nets']:
            row('diffpairs', 'diff',
                f"route {len(a['pair_nets']) // 2} differential pair(s) with route_diff",
                'pairs found by name, routed before the single nets; a pair net '
                'left in a DRC violation is routed again single-ended')
    row('route', 'route', 'route every net' + (' still open' if finish else ''),
        f"grid {a['grid']:g} mm"
        + (', power nets ' + ', '.join(a['power_nets'][:6])
           + (' ...' if len(a['power_nets']) > 6 else '') + f' at {RAIL_WIDTH:g} mm'
           if a['power_nets'] else '')
        + (', deeper rip-up (congested)' if a['congested'] else '')
        + ('; signals kept off the inner planes' if any(
            float(c) < 0 for c in a['layer_costs']) else ''))
    for i in range(2, ROUTE_PASSES + 1):
        row(f'route_p{i}', 'route', f'route pass {i} on the best board so far',
            f'only while nets stay open, the last pass was kept, and the router '
            f'effort stays within --effort-factor {effort_factor:g} x the first '
            f"pass's; may cross the planes at {PLANE_FALLBACK_COST:g}", optional=True)
    rules = {k: a[k] for k in ('clearance', 'track', 'via', 'drill', 'fine_via',
                               'fine_drill', 'grid')}
    return {'mode': 'finish' if finish else 'full', 'rules': rules,
            'rules_source': a.get('rules_source'),
            'rules_why': ('the fab floor for every step: a net class above it is '
                          'aspirational, so the route steps cap every class at it; '
                          'read from ' + (a.get('rules_source') or
                                          "the fab defaults (the board has no .kicad_pro)")),
            'layers': a['layers'],
            'layer_costs': a['layer_costs'], 'planes': a['planes'],
            'power_nets': a['power_nets'], 'pair_nets': a['pair_nets'],
            'parts': a['parts'], 'not_fanned': a.get('not_fanned', []),
            'unselectable': a.get('unselectable', []),
            'congested': a['congested'], 'n_nets': a['n_nets'],
            'effort_factor': effort_factor, 'steps': rows}


def run_plan(board, output, a, work, finish=False, effort_factor=2.0):
    ch = Chain(board, work, clearance=a['clearance'], baseline=os.path.abspath(board))
    L = a['layers']
    costs = [str(c) for c in a['layer_costs']]
    if finish:
        return route_passes(ch, a, L, costs, effort_factor, finish=True)

    # 2. planes, first
    if a['planes']:
        out, _, _ = ch.run('planes', [
            '{IN}', '{OUT}', '--nets', *[n for n, _ in a['planes']],
            '--plane-layers', *[l for _, l in a['planes']],
            '--clearance', str(a['clearance'])], 'planes')
        ch.accept(out)

    # 3. fanout, each part, with a retry ladder keeping the best escape
    excl = [f"!{a['globs'][n]}" for n in a['plane_nets'] if n in a['globs']]
    fanned = False
    fans = []
    for ref, tool, pitch in a['parts']:
        fine = pitch <= 0.65
        v, d = (a['fine_via'], a['fine_drill']) if fine else (a['via'], a['drill'])
        tw = str(a['track'])
        if tool == 'bga':
            base = ['{IN}', '--output', '{OUT}', '--component', ref,
                    '--nets', '*', *excl, '--layers', *L, '--layer-costs', *costs,
                    '--track-width', tw, '--grid-step', str(a['grid']),
                    '--clearance', str(a['clearance']),
                    # the ceiling too, as on the route steps: bga_fanout honours
                    # and keeps the net classes without one (#1529)
                    '--clearance-ceiling', str(a['clearance'])]
            # The working via, then the fine via (one rung when they are the
            # same via), then the fine via with dogbone escapes.
            vias = dict.fromkeys([(v, d), (a['fine_via'], a['fine_drill'])])
            ladder = [base + ['--via-size', str(sv), '--via-drill', str(sd)]
                      for sv, sd in vias]
            ladder.append(ladder[-1] + ['--escape-method', 'dogbone'])
            names = [f'{sv:g}/{sd:g} via' for sv, sd in vias]
            names.append(names[-1] + ', dogbone')
        else:
            base = ['{IN}', '--output', '{OUT}', '--component', ref,
                    '--nets', '*', *excl, '--width', tw,
                    '--grid-step', str(a['grid']), '--clearance', str(a['clearance']),
                    # the ceiling too, as on the route steps: qfn_fanout honours
                    # and keeps the net classes without one (#1529)
                    '--clearance-ceiling', str(a['clearance'])]
            ladder = [
                base,
                base + ['--escape-method', 'underpad',
                        '--via-size', str(a['fine_via']), '--via-drill', str(a['fine_drill']),
                        '--allow-via-in-pad'],
            ]
            names = ['stub escape', f"under-pad escape, {a['fine_via']:g}/"
                                    f"{a['fine_drill']:g} via in pad"]
            if tool == 'mezz':
                ladder, names = ladder[1:], names[1:]
        fan = {'ref': ref, 'tool': tool, 'ladder': ladder, 'input': ch.cur,
               'rung_names': names}
        best = fan_ladder(ch, fan)
        fan['best'] = best if best and best['escaped'] > 0 else None
        fans.append(fan)
        if fan['best']:
            ch.accept(best['out'])
            fanned = True
            ch.decide(f'fanout_{ref}', True,
                      f"kept rung {best['rung']} ({names[best['rung']]}): escaped "
                      f"{best['escaped']}, {best['failed']} dropped, "
                      f"{best['grazes']} graze(s)", board=best['out'])
        else:
            ch.decide(f'fanout_{ref}', False,
                      'no rung escaped a ball; the route step routes the part '
                      'from its pads' if best else
                      'no rung produced a board; the route step routes the part '
                      'from its pads')

    # 4. decaps, once, after the fanouts -- and again after a re-fan that
    # holds the parts it left shorted
    if fanned:
        decap = ['{IN}', '{OUT}', '--clearance', str(a['clearance'])]
        out, _, _ = ch.run('decap', decap, 'decap')
        ch.accept(out)
        if out:
            refan(ch, fans, decap)
    elif a['parts']:
        ch.skip('decap', 'no fanout escaped a ball, so there is no fanout copper '
                         'to clear the caps off')

    # 5. diff pairs first
    rip, reroute = [], []
    if a['pair_nets']:
        before = ch.drc_nets(ch.cur)
        out, s, _ = ch.run('diff', [
            '{IN}', '{OUT}', '--nets', *[a['globs'][n] for n in a['pair_nets']],
            '--track-width', str(max(0.1, a['track'])),
            '--diff-pair-gap', str(max(0.1, a['clearance'])),
            '--clearance-ceiling', str(a['clearance']),
            '--via-size', str(a['via']), '--via-drill', str(a['drill']),
            '--layers', *L, '--layer-costs', *costs,
            '--grid-step', str(a['grid'])], 'diffpairs')
        ch.accept(out)
        # A pair it could not couple is left to the route step, with whatever
        # stubs it laid; those stubs block the partner unless they may be ripped.
        if out:
            rip = [a['globs'][n] for n in (s or {}).get('single_ended_followup_nets') or []
                   if n in a['globs']]
            # A pair net the step left in a DRC violation is laid again, single-ended.
            after = ch.drc_nets(out)
            if None not in (before, after):
                reroute = sorted((after - before) & set(a['pair_nets']))
            ch.decide('diffpairs', True,
                      '; '.join(x for x in (
                          f"{len(rip)} uncoupled pair net(s) the route step may rip"
                          if rip else '',
                          f"{len(reroute)} pair net(s) left in a DRC violation, "
                          f"routed again single-ended: {', '.join(reroute)}"
                          if reroute else '') if x) or 'every pair kept as routed',
                      rip=rip, reroute=reroute)

    return route_passes(ch, a, L, costs, effort_factor, rip=rip, reroute=reroute)


def rung_grazes(summary):
    """The clearance rows a fanout rung's own copper left that no later step
    clears: its JSON_SUMMARY drc_grazes, less those on a part the decap step
    moves. A grade that errored or never ran counts none: it measured nothing."""
    g = (summary or {}).get('drc_grazes') or {}
    return int(g.get('total') or 0) - int(g.get('deferred_to_decap') or 0)


def fan_ladder(ch, fan, rungs=None, hold=(), tag=''):
    """Run FAN's fanout ladder (or the RUNGS of it named) until a rung escapes
    every ball and leaves no graze; returns the rung that escaped most, then
    grazed least (#1685), the earliest of equals, as {'escaped', 'failed',
    'grazes', 'out', 'rung'}, or None when no rung produced a board. HOLD names
    the parts the fanout keeps off as if locked (bga_fanout --immovable-refs)."""
    ref = fan['ref']
    best = None
    for i in (range(len(fan['ladder'])) if rungs is None else rungs):
        args = fan['ladder'][i] + (['--immovable-refs', *hold] if hold else [])
        step = f'fanout_{ref}' + (f'_r{i}' if i else '') + tag
        out, s, _ = ch.run('bga' if fan['tool'] == 'bga' else 'qfn', args, step,
                           parent='refan' if tag else (f'fanout_{ref}' if i else None))
        if not out:
            continue
        escaped = int((s or {}).get('escaped', 0))
        failed = int((s or {}).get('failed', 0) or 0)
        grazes = rung_grazes(s)
        if best is None or (escaped, -grazes) > (best['escaped'], -best['grazes']):
            best = {'escaped': escaped, 'failed': failed, 'grazes': grazes,
                    'out': out, 'rung': i}
        if s is not None and failed == 0 and grazes == 0:
            break
    return best


def _named_items(d, cap=20):
    """check_drc report D's real items, one short line each, so the run's last
    line says WHAT is left and not only how many (#1681): a part the clearance
    step could not clear ships as `R6.2 pad-via GND 0.07`."""
    names = []
    for it in d.get('items') or []:
        if it.get('accepted'):
            continue
        over = it.get('overlap_mm')
        names.append(' '.join(str(x) for x in (
            it.get('pad_ref') or it.get('net1'), it.get('type'), it.get('net2'),
            round(over, 3) if isinstance(over, (int, float)) else None,
            'SHORT' if it.get('short') else None) if x is not None))
    if len(names) > cap:
        return names[:cap] + [f'+{len(names) - cap} more']
    return names


def _shorts(d):
    """check_drc report D's shorts: the items it does not accept that put
    copper of two nets in contact (check_drc sets `short` on copper pairs
    only, never on a hole, edge or size item, #1558)."""
    return [it for it in d.get('items') or []
            if it.get('short') and not it.get('accepted')]


def held_parts(board, d, fans):
    """{BGA ref: [part refs]}: the parts BOARD's check_drc report D still
    shorts to a fanned BGA's copper, where the fanout laid its copper on the
    part because the clearance step moves such a part (an unlocked 2-pad
    C/R/FB, bga_fanout's movable passive) and that step could not.

    A short is the BGA's when the copper's net is on one of its balls (its
    escapes and plane drops alike); of two such BGAs, the one with a ball of
    that net nearest the copper."""
    from kicad_parser import parse_kicad_pcb
    from bga_fanout.geometry import movable_passive
    bgas = [f['ref'] for f in fans if f['tool'] == 'bga' and f['best']]
    items = [it for it in _shorts(d)
             if it.get('type') in ('pad-via', 'pad-segment') and it.get('pad_ref')]
    if not bgas or not items:
        return {}
    pcb = parse_kicad_pcb(board)
    balls = {}                          # net name -> [(BGA ref, x, y)]
    for ref in bgas:
        fp = pcb.footprints.get(ref)
        for p in (fp.pads if fp else ()):
            if p.net_name:
                balls.setdefault(p.net_name, []).append((ref, p.global_x, p.global_y))
    held = {}
    for it in items:
        part = it['pad_ref'].rsplit('.', 1)[0]
        fp = pcb.footprints.get(part)
        if fp is None or part in bgas:
            continue
        copper = [p for p in fp.pads if any(str(l).endswith('.Cu') for l in p.layers)]
        if not movable_passive(fp, pcb, copper):
            continue
        at = it.get('via_loc') or it.get('seg_loc') or it.get('pad_loc')
        x = sum(at[0::2]) / len(at[0::2])
        y = sum(at[1::2]) / len(at[1::2])
        near = min(balls.get(it.get('net2'), ()), default=None,
                   key=lambda b: math.hypot(b[1] - x, b[2] - y))
        if near:
            held.setdefault(near[0], set()).add(part)
    return {ref: sorted(parts) for ref, parts in held.items()}


def refan(ch, fans, decap):
    """The clearance step moves a part off the fanout's copper, so a fanout
    lays vias and escapes on such parts. A part it could not clear is still
    shorted to that copper, so the BGA is fanned out again from the board it
    started on, holding those parts (dropping a ball it cannot escape, which
    the route step routes from the bare ball), every later fanout replayed
    with the rung it was accepted at, and the clearance step run again. The
    board with fewer shorts, then fewer dropped escapes, is kept."""
    d = ch.check_drc(ch.cur)
    if d is None:
        return
    held = held_parts(ch.cur, d, fans)
    if not held:
        return
    before = (len(_shorts(d)), sum(f['best']['failed'] for f in fans if f['best']))
    print(f"  decap left {before[0]} short(s); re-fanning "
          + '; '.join(f"{r} holding {' '.join(p)}" for r, p in held.items()), flush=True)
    first = min(i for i, f in enumerate(fans) if f['ref'] in held)
    kept, ch.cur = ch.cur, fans[first]['input']
    failed = sum(f['best']['failed'] for f in fans[:first] if f['best'])
    for f in fans[first:]:
        if not f['best']:
            continue
        if f['ref'] in held:
            best = fan_ladder(ch, f, hold=held[f['ref']], tag='_held')
        else:
            best = fan_ladder(ch, f, rungs=[f['best']['rung']], tag='_refan')
        if not best or best['escaped'] == 0:
            ch.cur = kept
            ch.decide('refan', False, f"the re-fan of {f['ref']} escaped no ball; "
                                      f"the board before the re-fan is kept",
                      held=held)
            return
        ch.accept(best['out'])
        failed += best['failed']
    out, _, _ = ch.run('decap', decap, 'decap_refan', parent='refan')
    d2 = ch.check_drc(out) if out else None
    after = None if d2 is None else (len(_shorts(d2)), failed)
    ch.steps.append({'tool': 'refan', 'tag': 'refan', 'held': held,
                     'before': {'shorts': before[0], 'dropped': before[1]},
                     'after': after and {'shorts': after[0], 'dropped': after[1]},
                     'kept': bool(after and after < before)})
    if after and after < before:
        ch.accept(out)
        print(f"  re-fan kept: {after[0]} short(s), {after[1]} dropped escape(s) "
              f"(was {before[0]}, {before[1]})", flush=True)
    else:
        ch.cur = kept
        print(f"  re-fan not kept: "
              + (f"{after[0]} short(s), {after[1]} dropped escape(s)" if after else 'ungraded')
              + f" (was {before[0]}, {before[1]})", flush=True)
    ch.decide('refan', bool(after and after < before),
              (f"{after[0]} short(s), {after[1]} dropped escape(s)" if after
               else 'ungraded') + f" against {before[0]} short(s), {before[1]} "
              f"dropped before the re-fan; kept the "
              + ('re-fanned board' if after and after < before else 'board before it'),
              held=held, board=out if after and after < before else kept)


def _improves(g, best):
    """G ranks above BEST completion first: fewer open nets, then, at the same
    open count, fewer DRC violations. A pass that closes nets at the cost of
    new violations is kept, and its violations are disclosed by the grade."""
    return (g['open'], g['drc']) < (best['open'], best['drc'])


def route_passes(ch, a, L, costs, effort_factor=2.0, finish=False, rip=(), reroute=()):
    """6. route every net; up to two more passes while nets are open, each
    pass improves on the best board so far, and the effort stays within
    budget. Only the first pass keeps signals off the solid planes; the later
    ones may cross them at PLANE_FALLBACK_COST, so a plane carries only what
    could not route without it. The REROUTE nets are first ripped and routed
    again on their own. Returns (chain, best board or None,
    its grade or None)."""
    def route_args(costs):
        return base + ['--layers', *L, '--layer-costs', *costs]
    base = ['{IN}', '{OUT}', '--nets', '*', '--no-bga-zones',
            '--clearance-ceiling', str(a['clearance']),
            '--track-width', str(a['track']),
            '--via-size', str(a['via']), '--via-drill', str(a['drill']),
            '--grid-step', str(a['grid'])]
    if a['power_nets']:
        base += ['--power-nets', *[a['globs'][n] for n in a['power_nets']],
                 '--power-nets-widths',
                 *[str(RAIL_WIDTH)] * len(a['power_nets'])]
    if a['congested']:
        base += ['--max-ripup', '5']
    if rip:
        base += ['--rip-existing-nets', *rip]
    best = None                        # (grade, board)
    passes = ['route' + (f'_p{i + 1}' if i else '') for i in range(ROUTE_PASSES)]

    def skip_rest(i, why):
        for tag in passes[i:]:
            ch.skip(tag, why)
    if finish:
        ev = {'n': 0, 'tag': 'input', 'tool': 'grade', 'parent': None,
              'board': ch.cur, 'input': ch.cur, 'cmd': [], 'log': None}
        ch.started.add('input')
        emit('AUTOROUTE_STEP', dict(ev, state='running', t=round(time.time(), 3)))
        t0 = time.time()
        g = ch.grade(ch.cur)
        end = dict(ev, seconds=round(time.time() - t0, 1), t=round(time.time(), 3),
                   rc=0, summary={}, iterations=None)
        if g is None:
            emit('AUTOROUTE_STEP', dict(end, state='failed', board=None,
                                        result='a checker could not grade the input',
                                        error_tail=[]))
            print("  the input could not be graded: no route pass runs", flush=True)
            skip_rest(0, 'the input could not be graded')
            return ch, None, None
        emit('AUTOROUTE_STEP', dict(end, state='done', grade=g,
                                    result=f"{g['open']} open, {g['drc']} DRC"))
        ch.steps.append({'tool': 'grade', 'tag': 'input', 'grade': g})
        best = (g, ch.cur)
        print(f"  input: {g['open']} open, {g['drc']} DRC", flush=True)
        if g['open'] == 0:
            skip_rest(0, 'the input has no open net')
            return ch, ch.cur, g
    if reroute:
        # A net whose new route lays no copper gets its old copper back, and
        # the passes below route whatever is left open.
        out, _, _ = ch.run('route', ['{IN}', '{OUT}', '--nets', *[a['globs'][n] for n in reroute],
                                     '--force-reroute', *base[4:], '--layers', *L,
                                     '--layer-costs', *costs], 'reroute_pairs',
                           parent='diffpairs')
        ch.accept(out)
    crossing = [str(PLANE_FALLBACK_COST) if float(c) < 0 else c for c in costs]
    first = spent = None
    for i in range(ROUTE_PASSES):
        if i:
            if ch.last_iterations is None:
                print(f"  route pass {i + 1} not started: the last pass reported "
                      f"no iterations to budget from", flush=True)
                skip_rest(i, 'the last pass reported no router iterations to '
                             'budget the next from')
                break
            if spent + ch.last_iterations > effort_factor * first:
                print(f"  route pass {i + 1} not started: {spent} iterations "
                      f"spent, the last pass took {ch.last_iterations}, budget "
                      f"{effort_factor:g} x {first}", flush=True)
                skip_rest(i, f"effort budget: {spent} router iterations spent and "
                             f"the last pass took {ch.last_iterations}, over "
                             f"--effort-factor {effort_factor:g} x the first "
                             f"pass's {first}")
                break
        out, _, _ = ch.run('route', route_args(crossing if i else costs),
                           passes[i], hold=True)
        if ch.last_iterations is not None:
            spent = (spent or 0) + ch.last_iterations
            if first is None:
                first = ch.last_iterations
        if not out:
            skip_rest(i + 1, f'{passes[i]} failed')
            break
        g = ch.grade(out)
        ch.steps[-1]['grade'] = g
        if g is None:
            if best is None:           # still the routed board, just ungraded
                best = (None, out)
                ch.accept(out)
            ch.end_step(result='routed, but a checker could not grade it',
                        grade=None, kept=best[1] == out)
            skip_rest(i + 1, f'{passes[i]} could not be graded')
            break
        print(f"  {os.path.basename(out)}: {g['open']} open, {g['drc']} DRC", flush=True)
        if best is not None and not _improves(g, best[0]):
            print(f"  not kept: no better than {os.path.basename(best[1])} "
                  f"({best[0]['open']} open, {best[0]['drc']} DRC)", flush=True)
            ch.end_step(result=f"{g['open']} open, {g['drc']} DRC: not kept",
                        grade=g, kept=False)
            ch.decide(passes[i], False,
                      f"{g['open']} open, {g['drc']} DRC is no better than "
                      f"{os.path.basename(best[1])} ({best[0]['open']} open, "
                      f"{best[0]['drc']} DRC)")
            skip_rest(i + 1, f'{passes[i]} was not kept, which ends the passes')
            break
        best = (g, out)
        ch.accept(out)
        ch.end_step(result=f"{g['open']} open, {g['drc']} DRC: best so far",
                    grade=g, kept=True)
        if g['open'] == 0:
            skip_rest(i + 1, 'no open net left')
            break
    ch.end_step()
    return ch, (best[1] if best else None), (best[0] if best else None)


def _effort_factor(text):
    v = float(text)
    if not math.isfinite(v) or v < 0:
        raise argparse.ArgumentTypeError(f'must be a finite number >= 0, not {text}')
    return v


def _plane_layer_cost(text):
    v = float(text)
    if not math.isfinite(v) or v == 0:
        raise argparse.ArgumentTypeError(f'must be a finite nonzero number, not {text}')
    return v


def _ship(src, dst):
    """Copy board SRC and its siblings to DST, removing any sibling DST has
    that SRC does not, so the shipped set is exactly SRC's."""
    from copy_board import copy_board, SIBLING_EXTS
    sb, db = src[:-len('.kicad_pcb')], dst[:-len('.kicad_pcb')]
    for ext in SIBLING_EXTS:
        if os.path.exists(db + ext) and not os.path.exists(sb + ext):
            os.remove(db + ext)
    copy_board(src, dst)


def main():
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('input_file')
    ap.add_argument('output_file')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the analysis and exit')
    ap.add_argument('--finish', action='store_true',
                    help='only the final route passes, on a board already '
                         'prepared and routed by another chain')
    ap.add_argument('--effort-factor', type=_effort_factor, default=EFFORT_FACTOR,
                    help='a route pass after the first starts only while the '
                         'router iterations spent, plus the last pass\'s '
                         'again, stay within this many times the first '
                         'pass\'s (default 2). A forecast, not a cap: a pass '
                         'that starts runs to its end')
    ap.add_argument('--plane-layer-cost', type=_plane_layer_cost, default=PLANE_LAYER_COST,
                    help='the --layer-costs entry of every inner plane layer in the '
                         'fanout, diff pair and first route steps. Negative (default -1) '
                         'keeps signals off the planes, so each stays a solid '
                         'reference, and only the later route passes may cross them '
                         'at 3.0; a positive price lets every step cross them '
                         '(the skill\'s route step uses 3.0), trading reference '
                         'integrity for routability')
    args = ap.parse_args()

    t_start = time.time()
    a = analyze(args.input_file, finish=args.finish, plane_cost=args.plane_layer_cost)
    print("AUTOROUTE_ANALYSIS: " + json.dumps(a, sort_keys=True), flush=True)
    plan = plan_steps(a, finish=args.finish, effort_factor=args.effort_factor)
    emit('AUTOROUTE_PLAN', plan)
    if args.dry_run:
        return 0
    out = os.path.abspath(args.output_file)
    work = out + '.autoroute'
    from copy_board import copy_board
    # Stage the input before clearing the work directory: it may live there
    # (a previous run's step board).
    stage = tempfile.mkdtemp(prefix='.autoroute_input_', dir=os.path.dirname(out))
    try:
        staged = os.path.join(stage, 'input.kicad_pcb')
        copy_board(os.path.abspath(args.input_file), staged)
        if os.path.isdir(work):
            shutil.rmtree(work)
        os.makedirs(work)
        src = os.path.join(work, 'input.kicad_pcb')
        copy_board(staged, src)
    finally:
        shutil.rmtree(stage, ignore_errors=True)
    ch, final, grade = run_plan(src, out, a, work, finish=args.finish,
                                effort_factor=args.effort_factor)
    ch.end_step()
    note = None
    if not final and (args.finish or ch.cur != src):
        # No route pass finished: ship the best board the chain has -- the
        # input itself for --finish, else the last step that succeeded.
        final = ch.cur
        note = 'no route pass finished; shipping the last board a step produced'
        print(f"AUTOROUTE: no route pass finished; shipping "
              f"{os.path.basename(final)}", flush=True)
    # Every planned step that never started, with why.
    for r in plan['steps']:
        if r['tag'] not in ch.started:
            emit('AUTOROUTE_STEP', {'tag': r['tag'], 'tool': r['tool'], 'parent': None,
                                    'state': 'skipped', 't': round(time.time(), 3),
                                    'reason': ch.skipped.get(r['tag'], 'not reached')})
    with open(os.path.join(work, 'plan.json'), 'w') as fh:
        json.dump({'analysis': a, 'plan': plan, 'steps': ch.steps,
                   'decisions': ch.decisions, 'final': final,
                   'grade': grade}, fh, indent=1)
    final_tag = None
    if final:
        final_tag = next((s['tag'] for s in ch.steps
                          if s.get('log') and final.endswith(
                              s['log'][:-len('.log')] + '.kicad_pcb')),
                         'input' if final == src else None)
    summary = {'final_step': os.path.basename(final) if final else None,
               'final_tag': final_tag, 'output': out if final else None,
               'work': work, 'note': note,
               'open': grade['open'] if grade else None,
               'open_nets': grade['open_nets'] if grade else None,
               'drc': grade['drc'] if grade else None,
               'drc_items': grade['drc_items'] if grade else None,
               'steps': len(ch.steps),
               'seconds': round(time.time() - t_start, 1),
               'skipped': [s['tag'] for s in ch.steps if not s.get('ok', True)]}
    if not final:
        print("AUTOROUTE: no route pass produced a board", flush=True)
        summary['error'] = 'no route pass produced a board'
        print("AUTOROUTE_SUMMARY: " + json.dumps(summary), flush=True)
        return 1
    _ship(final, out)
    print("AUTOROUTE_SUMMARY: " + json.dumps(summary), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
