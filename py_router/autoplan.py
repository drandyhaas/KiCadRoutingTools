#!/usr/bin/env python3
"""Route a board through a deterministic plan built from the board itself.

    python3 py_router/autoplan.py board.kicad_pcb routed.kicad_pcb
    python3 py_router/autoplan.py board.kicad_pcb routed.kicad_pcb --dry-run

The plan-pcb-routing skill has an agent read a board and choose a command
chain. This makes the same kinds of choices from board facts alone -- the
footprints, net names, layer count and the board's own design rules -- and
runs the chain, so routing without an agent gets the steps that remove most
congestion before the router starts:

  1. design rules  the board's Default class clearance as a ceiling, the fab
                   track floor, the working via (fine via for fine-pitch
                   escapes), all from list_nets.read_design_rules
  2. planes        GND on In1.Cu and the dominant rail on In2.Cu on 4+ layer
                   boards, GND on B.Cu on 2-layer boards; poured first
  3. fanout        every grid array with interior depth or fine pitch
                   (bga_fanout) and every fine-pitch four-sided perimeter
                   package (qfn_fanout), read from the pads and their pitch
                   along a row -- never a connector, socket or magnetics --
                   plane nets excluded; a part that drops escapes is retried
                   at the fab clearance with the fine via, then with the
                   dogbone (BGA) or under-pad (QFN) method, keeping the
                   attempt that escaped most
  4. decaps        place_fanout_clearance once, after the fanouts
  5. diff pairs    name-detected pairs routed first with route_diff
  6. route         route.py over every net with the poured nets and rails as
                   power nets, layer costs that keep signals off the planes,
                   a finer grid around fine-pitch parts and deeper rip-up on
                   congested boards; up to two further passes on the result
                   while nets are left open, shipping the best pass

--finish runs step 6 alone on a board another chain has already prepared and
routed (an agent's chain, say): the poured planes are read from the board's
pours, and the input board is itself a candidate, so a pass ships only if it
beats it. route.py works on the nets still open, but like any route step it
may rip and reroute existing copper and runs its plane finalize.

Every step runs as its own process in <output>.autoplan/, so a step that
fails is skipped and the chain continues from the last good board. Each route
pass is graded: open nets as check_connected reports them (the KiCad
cross-check off, so the grade is deterministic) and real violations as
check_drc reports them. A pass is kept only if it is no worse on both and
better on one; the first pass that is not kept ends the passes. A further pass
starts only while the router iterations spent, plus the last pass's again,
stay within --effort-factor times the first pass's -- a forecast from counts
the router reproduces exactly, not a cap on the pass itself -- and not at all
when a pass reported no iterations. If no route pass succeeds, the best board
the chain has ships (with --finish, the input board itself). The chain and
every grade are written to <output>.autoplan/plan.json.
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
ROUTE_PASSES = 3             # the first pass plus up to two on its result


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
    left once each marker (a net-less pad unlike the commonest pad, such as a
    BGA's corner outline pads off the ball lattice), each exposed pad (a pad
    over 3x the median area) and anything inside one (its sub-pads) is set
    aside. An unused ball or pin is net-less too, but keeps the common size.

    grid: >= 3 rows and >= 3 columns of 3 or more pads, at least one pad off
    the bounding edges, and the columns and rows each on a lattice of their
    own pitch: the commonest spacing between neighbouring pads of one row (x)
    or one column (y), the two within 2x of each other. The part's pitch is
    the finer of the two; depth is rows from the edge to the centre.
    perimeter: every pad on one of the four bounding edges, at least two on
    each side away from its corners. The pitch is the median spacing between neighbours along a side.
    Anything else -- two rows (a connector, a DFN, a transformer), one row, a
    scatter -- is neither."""
    from collections import Counter
    pads = [p for p in fp.pads if p.pad_type != 'np_thru_hole']
    dims = lambda p: tuple(sorted((round(p.size_x, 3), round(p.size_y, 3))))
    if pads:
        common = Counter(map(dims, pads)).most_common(1)[0][0]
        pads = [p for p in pads if p.net_id or dims(p) == common]
    if len(pads) < 6:
        return None
    area = sorted(p.size_x * p.size_y for p in pads)
    med = area[len(area) // 2]
    big = [p for p in pads if p.size_x * p.size_y > 3 * med]

    def under(p, q):
        r = max(q.size_x, q.size_y) / 2
        return abs(p.local_x - q.local_x) <= r and abs(p.local_y - q.local_y) <= r
    kept = [p for p in pads if p not in big and not any(under(p, q) for q in big)]
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
    if len(xs) >= 3 and len(ys) >= 3 and interior:
        rows = {}
        cols = {}
        for x, y in pts:
            rows.setdefault(y, []).append(x)
            cols.setdefault(x, []).append(y)
        # Each axis has its own pitch (LPDDR packages: 0.8 x 0.65 mm); the
        # finer one is the part's escape pitch.
        if (sum(len(r) >= 3 for r in rows.values()) < 3
                or sum(len(c) >= 3 for c in cols.values()) < 3):
            return None
        px = _mode([d for r in rows.values() for d in steps(r)])
        py = _mode([d for c in cols.values() for d in steps(c)])
        # Two parallel pad rows a module's width apart (a compute module's
        # board-to-board connectors) also sit on a lattice, just not a square one.
        if (px and py and max(px, py) <= 2 * min(px, py)
                and on_lattice(xs, px) and on_lattice(ys, py)):
            return ('grid', min(px, py), min(len(xs), len(ys)) // 2)
        return None
    # A side counts only the pads strictly between the two perpendicular
    # edges: a two-row part's end pads sit on a corner, not on a side.
    mid_x = lambda x: x0 + tol < x < x1 - tol
    mid_y = lambda y: y0 + tol < y < y1 - tol
    sides = {'l': [y for x, y in pts if abs(x - x0) <= tol and mid_y(y)],
             'r': [y for x, y in pts if abs(x - x1) <= tol and mid_y(y)],
             't': [x for x, y in pts if abs(y - y0) <= tol and mid_x(x)],
             'b': [x for x, y in pts if abs(y - y1) <= tol and mid_x(x)]}
    on_edge = sum(1 for x, y in pts if abs(x - x0) <= tol or abs(x - x1) <= tol
                  or abs(y - y0) <= tol or abs(y - y1) <= tol)
    if on_edge != len(pts) or any(len(v) < 2 for v in sides.values()):
        return None
    d = sorted(s for v in sides.values() for s in steps(v) if s > 1e-3)
    return ('perimeter', round(d[len(d) // 2], 3), None) if d else None


def fanout_parts(pcb):
    """[(ref, tool, pitch)] for the parts the skill fans out (Step 3): a grid
    array (bga_fanout) or a four-sided perimeter package (qfn_fanout), read
    from the pads themselves -- never a connector or magnetics by name."""
    out = []
    for ref, fp in pcb.footprints.items():
        name = (fp.footprint_name or '').upper()
        if any(k in name for k in NOT_FANOUT_NAMES):
            continue
        pads = [p for p in fp.pads if p.pad_type != 'np_thru_hole']
        th = sum(1 for p in pads if (p.drill or 0) > 0)
        if th > len(pads) - th and 'PGA' not in name:
            continue
        shape = pad_shape(fp)
        if shape is None:
            continue
        kind, pitch, depth = shape
        if kind == 'grid':
            if pitch < 2.0 and (depth > 2 or pitch <= 0.5):
                out.append((ref, 'bga', pitch))
        elif pitch < 1.27 and (pitch <= 0.5 or (len(pads) > 40 and pitch <= 0.8)):
            out.append((ref, 'qfn', pitch))
    return sorted(out)


def analyze(board, finish=False):
    from kicad_parser import parse_kicad_pcb, pour_zones
    from list_nets import (read_design_rules, effective_floors,
                           find_power_nets, find_differential_pairs,
                           board_default_netclass_clearance)

    pcb = parse_kicad_pcb(board)
    layers = list(pcb.board_info.copper_layers)
    n_layers = len(layers)
    rules = read_design_rules(board)
    eff = effective_floors(rules.get('constraints') or {}, n_layers)

    clearance = board_default_netclass_clearance(board) or eff['drc_clearance']
    clearance = _num(max(clearance, eff['drc_clearance']))
    track = _num(max(eff['fab_track_width'], 0.1))
    via, drill = _num(eff['working_via_diameter']), _num(eff['working_via_drill'])
    fine_via, fine_drill = _num(eff['fine_via_diameter']), _num(eff['fine_via_drill'])

    # Name-matched grounds and supplies only: find_power_nets' pad-count
    # candidates include keyboard matrix rows and unconnected pins.
    gnd, vcc, _cand = find_power_nets(pcb)
    real = lambda t: t[1] >= 3 and not t[0].lower().startswith('unconnected-')
    gnd = sorted(filter(real, gnd), key=lambda t: -t[1])
    rails = sorted(filter(real, vcc), key=lambda t: -t[1])
    planes = []                         # [(net, layer)]
    if gnd:
        if n_layers >= 4:
            planes.append((gnd[0][0], layers[1]))
            if rails and rails[0][1] >= 10:
                planes.append((rails[0][0], layers[2]))
        else:
            planes.append((gnd[0][0], layers[-1]))
    # route_planes takes EXACT names: a name argparse cannot carry stays a
    # power net (passed as a pattern) instead of a plane.
    planes = [(n, l) for n, l in planes if _exact_ok(n)]
    plane_nets = [n for n, _ in planes]
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
        layer_costs = [3.0 if l in solid else 1.0 for l in layers]

    parts = fanout_parts(pcb)
    fine = any(p <= GRID_FINE_PITCH for _, _, p in parts)
    grid = 0.05 if fine else 0.1
    n_nets = sum(1 for n in pcb.nets.values() if n.name and len(n.pads) >= 2)
    congested = n_nets > CONGESTED_NETS or any(t == 'bga' and p <= 0.8 for _, t, p in parts)

    pairs = []
    for pr in find_differential_pairs(pcb) or []:
        p, n = (pr[0], pr[1]) if isinstance(pr, (tuple, list)) else (pr.p, pr.n)
        pairs += [getattr(p, 'name', p), getattr(n, 'name', n)]

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
        print(f"AUTOPLAN: no pattern selects only {', '.join(unnamed)} on this "
              f"board; left out of the power, exclusion and pair lists", flush=True)
    power_nets = [n for n in power_nets if n in globs]
    pairs = [n for i in range(0, len(pairs) - 1, 2) for n in pairs[i:i + 2]
             if pairs[i] in globs and pairs[i + 1] in globs]

    return dict(layers=layers, clearance=clearance, track=track, via=via,
                drill=drill, fine_via=fine_via, fine_drill=fine_drill,
                fab_clearance=_num(eff['drc_clearance']), planes=planes,
                plane_nets=plane_nets, power_nets=power_nets,
                layer_costs=layer_costs, parts=parts, grid=grid,
                congested=congested, n_nets=n_nets, pair_nets=pairs,
                globs=globs)


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
    "print('AUTOPLAN_OPEN: ' + json.dumps(sorted({i['net_name'] for i in issues"
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


class Chain:
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
        m = re.search(r'^AUTOPLAN_OPEN: (\[.*\])$', r.stdout, re.M)
        if r.returncode != 0 or not m:
            print(f"  grade: check_connected failed on {os.path.basename(board)}",
                  flush=True)
            return None
        open_nets = json.loads(m.group(1))
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
                drc = int(json.load(fh)['violations'])
        except (OSError, ValueError, KeyError):
            print(f"  grade: check_drc failed on {os.path.basename(board)}", flush=True)
            return None
        finally:
            os.remove(jpath)
        return {'open': len(open_nets), 'open_nets': open_nets, 'drc': drc}

    def run(self, kind, args, tag):
        """Run one tool from the current board to a fresh output; returns
        (output path or None, JSON_SUMMARY dict or None, JSON_SUMMARY_MIN).
        ``last_iterations`` is the router iterations its JSON_SUMMARY lines
        report, summed -- the same effort count the replay harness uses --
        or None when the run printed no JSON_SUMMARY at all."""
        self.n += 1
        out = os.path.join(self.work, f'step{self.n:02d}_{tag}.kicad_pcb')
        argv = [sys.executable, '-u', '-X', 'utf8', TOOLS[kind]] + \
            [a.replace('{IN}', self.cur).replace('{OUT}', out) for a in args]
        log = out[:-len('.kicad_pcb')] + '.log'
        with open(log, 'w') as fh:
            rc = subprocess.run(argv, cwd=self.work, stdout=fh,
                                stderr=subprocess.STDOUT, env=_child_env()).returncode
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
        self.steps.append({'tool': kind, 'tag': tag, 'rc': rc, 'ok': ok,
                           'iterations': self.last_iterations,
                           'argv': argv[4:], 'log': os.path.basename(log)})
        print(f"  [{self.n:02d}] {tag}: rc={rc}{'' if ok else ' (skipped)'}",
              flush=True)
        return (out if ok else None), summary, summary_min

    def accept(self, out):
        if out:
            self.cur = out


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
    for ref, tool, pitch in a['parts']:
        fine = pitch <= 0.65
        v, d = (a['fine_via'], a['fine_drill']) if fine else (a['via'], a['drill'])
        tw = str(a['track'])
        if tool == 'bga':
            base = ['{IN}', '--output', '{OUT}', '--component', ref,
                    '--nets', '*', *excl, '--layers', *L,
                    '--track-width', tw, '--grid-step', str(a['grid'])]
            ladder = [
                base + ['--clearance', str(a['clearance']),
                        '--via-size', str(v), '--via-drill', str(d)],
                base + ['--clearance', str(a['fab_clearance']),
                        '--via-size', str(a['fine_via']), '--via-drill', str(a['fine_drill'])],
                base + ['--clearance', str(a['fab_clearance']),
                        '--via-size', str(a['fine_via']), '--via-drill', str(a['fine_drill']),
                        '--escape-method', 'dogbone'],
            ]
        else:
            base = ['{IN}', '--output', '{OUT}', '--component', ref,
                    '--nets', '*', *excl, '--width', tw,
                    '--grid-step', str(a['grid'])]
            ladder = [
                base + ['--clearance', str(a['clearance'])],
                base + ['--clearance', str(a['fab_clearance'])],
                base + ['--clearance', str(a['fab_clearance']),
                        '--escape-method', 'underpad',
                        '--via-size', str(a['fine_via']), '--via-drill', str(a['fine_drill']),
                        '--allow-via-in-pad'],
            ]
        best = None                    # (escaped, out)
        for i, args in enumerate(ladder):
            out, s, _ = ch.run(tool, args, f'fanout_{ref}' + (f'_r{i}' if i else ''))
            if not out:
                continue
            escaped = int((s or {}).get('escaped', 0))
            if best is None or escaped > best[0]:
                best = (escaped, out)
            if s is not None and int(s.get('failed', 0) or 0) == 0:
                break
        if best and best[0] > 0:
            ch.accept(best[1])
            fanned = True

    # 4. decaps, once, after the fanouts
    if fanned:
        out, _, _ = ch.run('decap', ['{IN}', '{OUT}', '--clearance',
                                     str(a['clearance'])], 'decap')
        ch.accept(out)

    # 5. diff pairs first
    if a['pair_nets']:
        out, s, _ = ch.run('diff', [
            '{IN}', '{OUT}', '--nets', *[a['globs'][n] for n in a['pair_nets']],
            '--track-width', str(max(0.1, a['track'])),
            '--diff-pair-gap', str(max(0.1, a['clearance'])),
            '--clearance-ceiling', str(a['clearance']),
            '--via-size', str(a['via']), '--via-drill', str(a['drill']),
            '--layers', *L, '--layer-costs', *costs,
            '--grid-step', str(a['grid'])], 'diffpairs')
        ch.accept(out)

    return route_passes(ch, a, L, costs, effort_factor)


def _improves(g, best):
    """G is no worse than BEST on open nets and on DRC, and better on one."""
    return (g['open'] <= best['open'] and g['drc'] <= best['drc']
            and (g['open'], g['drc']) != (best['open'], best['drc']))


def route_passes(ch, a, L, costs, effort_factor=2.0, finish=False):
    """6. route every net; up to two more passes while nets are open, each
    pass improves on the best board so far, and the effort stays within
    budget. Returns (chain, best board or None, its grade or None)."""
    args = ['{IN}', '{OUT}', '--nets', '*', '--no-bga-zones',
            '--clearance-ceiling', str(a['clearance']),
            '--track-width', str(a['track']),
            '--via-size', str(a['via']), '--via-drill', str(a['drill']),
            '--layers', *L, '--layer-costs', *costs,
            '--grid-step', str(a['grid'])]
    if a['power_nets']:
        args += ['--power-nets', *[a['globs'][n] for n in a['power_nets']],
                 '--power-nets-widths',
                 *[str(RAIL_WIDTH)] * len(a['power_nets'])]
    if a['congested']:
        args += ['--max-ripup', '5']
    best = None                        # (grade, board)
    if finish:
        g = ch.grade(ch.cur)
        if g is None:
            print("  the input could not be graded: no route pass runs", flush=True)
            return ch, None, None
        ch.steps.append({'tool': 'grade', 'tag': 'input', 'grade': g})
        best = (g, ch.cur)
        print(f"  input: {g['open']} open, {g['drc']} DRC", flush=True)
        if g['open'] == 0:
            return ch, ch.cur, g
    first = spent = None
    for i in range(ROUTE_PASSES):
        if i:
            if ch.last_iterations is None:
                print(f"  route pass {i + 1} not started: the last pass reported "
                      f"no iterations to budget from", flush=True)
                break
            if spent + ch.last_iterations > effort_factor * first:
                print(f"  route pass {i + 1} not started: {spent} iterations "
                      f"spent, the last pass took {ch.last_iterations}, budget "
                      f"{effort_factor:g} x {first}", flush=True)
                break
        out, _, _ = ch.run('route', args, 'route' + (f'_p{i + 1}' if i else ''))
        if ch.last_iterations is not None:
            spent = (spent or 0) + ch.last_iterations
            if first is None:
                first = ch.last_iterations
        if not out:
            break
        g = ch.grade(out)
        ch.steps[-1]['grade'] = g
        if g is None:
            if best is None:           # still the routed board, just ungraded
                best = (None, out)
                ch.accept(out)
            break
        print(f"  {os.path.basename(out)}: {g['open']} open, {g['drc']} DRC", flush=True)
        if best is not None and not _improves(g, best[0]):
            print(f"  not kept: no better than {os.path.basename(best[1])} "
                  f"({best[0]['open']} open, {best[0]['drc']} DRC)", flush=True)
            break
        best = (g, out)
        ch.accept(out)
        if g['open'] == 0:
            break
    return ch, (best[1] if best else None), (best[0] if best else None)


def _effort_factor(text):
    v = float(text)
    if not math.isfinite(v) or v < 0:
        raise argparse.ArgumentTypeError(f'must be a finite number >= 0, not {text}')
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
    ap.add_argument('--effort-factor', type=_effort_factor, default=2.0,
                    help='a route pass after the first starts only while the '
                         'router iterations spent, plus the last pass\'s '
                         'again, stay within this many times the first '
                         'pass\'s (default 2). A forecast, not a cap: a pass '
                         'that starts runs to its end')
    args = ap.parse_args()

    a = analyze(args.input_file, finish=args.finish)
    print("AUTOPLAN_ANALYSIS: " + json.dumps(a, sort_keys=True), flush=True)
    if args.dry_run:
        return 0
    out = os.path.abspath(args.output_file)
    work = out + '.autoplan'
    from copy_board import copy_board
    # Stage the input before clearing the work directory: it may live there
    # (a previous run's step board).
    stage = tempfile.mkdtemp(prefix='.autoplan_input_', dir=os.path.dirname(out))
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
    if not final and (args.finish or ch.cur != src):
        # No route pass finished: ship the best board the chain has -- the
        # input itself for --finish, else the last step that succeeded.
        final = ch.cur
        print(f"AUTOPLAN: no route pass finished; shipping "
              f"{os.path.basename(final)}", flush=True)
    with open(os.path.join(work, 'plan.json'), 'w') as fh:
        json.dump({'analysis': a, 'steps': ch.steps, 'final': final,
                   'grade': grade}, fh, indent=1)
    if not final:
        print("AUTOPLAN: no route pass produced a board", flush=True)
        return 1
    _ship(final, out)
    print("AUTOPLAN_SUMMARY: " + json.dumps(
        {'final_step': os.path.basename(final),
         'open': grade['open'] if grade else None,
         'drc': grade['drc'] if grade else None,
         'steps': len(ch.steps),
         'skipped': [s['tag'] for s in ch.steps if not s.get('ok', True)]}), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
