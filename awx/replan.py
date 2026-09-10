#!/usr/bin/env python3
"""replan.py -- the braid's VERDICT re-plans the ends (#622, 2026-09-10).

The plan's model cannot price a swimmer at K51 (rank agreement 0.2-0.4
with the braid over 16 routed boards), so every chooser judged by it
lost at the braid. This driver uses the ROUTE ITSELF as the judge and
the PREVIOUS route as the price. Round r, on a fanout board F (both ends
laid, its plan sidecar beside it) and its routed board R:

  1. VERDICT off R: per net its class in the braid's own schedule (page
     F / page B / swimmer), its real vias, refused and the boxed end
     (`walled_at`), its launch and target rank. The plan model's
     prediction on the plan F carries is recomputed with the residual
     off, and real - predicted per net becomes `plan_ends.RESIDUAL`:
     the learned price every judge below runs with.
  2. WORST: the refused nets, then the swimmers by real vias.
  3. CANDIDATES per worst net at each end the verdict allows (a lane
     boxed at its tooth moves its tooth; a swimmer may move either): the
     destination menu on the bare array (a class not banned and not the
     current one, its lane free of the other berths), the source menu
     with CLIMBS (`SRC_CLIMB`, the honest menu: legs clear of every other
     net's copper); each ranked by the plan's judged cost with every
     other net fixed at what the board carries.
  4. PROBE each candidate with the real router on R: the net stripped
     to its tooth, the asked end re-fanned by the production engine
     against everything else's copper (audited: face / gap / layer /
     kind), the net braided ALONE, the board graded whole. A candidate
     that grades better than R stands (opens first, then vias, no DRC).
  5. APPLY the standing candidates to F: source moves realized
     (`source_realize`, audited), berths re-fanned incrementally
     (`fanout_once`, audited), the UNMOVED teeth and berths checked
     unchanged from the board, the sidecar rewritten from the board it
     sits beside; then the BRAID on the new F, graded, its verdict read.
     Kept iff every move was laid in its asked class AND the board
     grades better; else the moves tried are banned and F stays.

Nothing here reads a face, a ref or a board name: the classes come off
the board, the candidates from the menus, the verdicts from the engines.

usage: replan.py TAG K [--board=BASE] [--dest=REF] [--rounds=4] [--worst=3]
                       [--climb=8] [--probes=1] [--min-vias=3] [--out=TAG]
  reads  tmp/TAG_fo_kK.kicad_pcb (+ .plan.json), tmp/TAG_kK.kicad_pcb
         (+ .log, .pack.json, _refusals.json)
  writes tmp/OUT_rp_kK_r<N>_* per round, tmp/OUT_rp_kK.kicad_pcb (best)
"""
import contextlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from collections import Counter

ARGV = [a for a in sys.argv[1:] if not a.startswith('--')]
OPTS = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--'))
# the source menu offers climbs (fanout_from_plan reads this at import)
os.environ.setdefault('SRC_CLIMB', OPTS.get('climb', '14'))

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import fanout_from_plan as fp  # noqa: E402
import source_realize as sr  # noqa: E402
import plan_ends as pe  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
from coherent_nets import coherent_nets  # noqa: E402

LAYERS = ('F.Cu', 'B.Cu')


_ECO = re.compile(r'\s*\(gr_line(?:[^()]|\([^()]*\))*\)')


def strip_eco(txt):
    """The braid's Eco overlay (planned centrelines, end marks) off a board
    text, so successive one-net runs do not stack drawings."""
    return _ECO.sub(lambda m: '' if 'Eco1.User' in m.group(0)
                    or 'Eco2.User' in m.group(0) else m.group(0), txt)


def copy_board(src, dst, eco=False):
    if eco:
        shutil.copy(src, dst)
    else:
        with open(src, encoding='utf-8') as f:
            txt = f.read()
        with open(dst, 'w', encoding='utf-8') as f:
            f.write(strip_eco(txt))
    fp.copy_pro(src, dst)


def source_view(fo, out, names, byname, dref, pad_mm=2.0):
    """The fanout board with the run's DESTINATION copper stripped (a
    window round the array, as fanout_once strips for a re-lay): the board
    the plan's state is read from."""
    pcb = parse_kicad_pcb(fo)
    n2n = {i: n.name for i, n in pcb.nets.items()}
    x0, y0, x1, y1 = em.grid_of(pcb.footprints[dref]).bbox
    x0, y0, x1, y1 = x0 - pad_mm, y0 - pad_mm, x1 + pad_mm, y1 + pad_mm
    nids = {byname[nm][0] for nm in names}
    segs = [s for s in pcb.segments if s.net_id in nids
            and x0 <= min(s.start_x, s.end_x) and max(s.start_x, s.end_x) <= x1
            and y0 <= min(s.start_y, s.end_y) and max(s.start_y, s.end_y) <= y1]
    vias = [v for v in pcb.vias if v.net_id in nids
            and x0 <= v.x <= x1 and y0 <= v.y <= y1]
    content = open(fo, encoding='utf-8').read()
    content, n_s = sr.remove_segments_from_content(content, segs, n2n)
    content, n_v = sr.remove_vias_from_content(content, vias, n2n)
    if n_s != len(segs) or n_v != len(vias):
        print(f'  source view: WARNING strip matched {n_s}/{len(segs)} segments, '
              f'{n_v}/{len(vias)} vias')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(content)
    fp.copy_pro(fo, out)


def grade(board, K, base):
    """(opens, drc, vias) of the run's nets on `board`, by grade_k."""
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'), board,
                        ','.join(coherent_nets(K, base))], capture_output=True, text=True)
    line = next((l for l in (r.stdout + r.stderr).splitlines() if l.startswith('GRADE')), '')
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', line)
    opens = sorted(line.split('open: ')[1].split(',')) if 'open: ' in line else []
    return ([opens, int(m.group(2)), int(m.group(3))] if m else [None, None, None]), line


def count_copper(board, nid):
    pcb = parse_kicad_pcb(board)
    return (sum(1 for v in pcb.vias if v.net_id == nid),
            sum(1 for s in pcb.segments if s.net_id == nid))


PAD_MM = 2.0          # the window round an array (fanout_once's re-lay strip)
TOL = 0.002           # mm: a text block is the same copper as a parsed item
_NUM = r'(-?[\d.]+)'


# ---------------------------------------------------------------- text
def dmenu_full(st):
    """The destination menus of a plan state: the full bare-array menu
    (`dmenu` in plan_state as committed; a `dmenu_full` when a pin or a
    force narrows `dmenu`)."""
    return st.get('dmenu_full', st['dmenu'])


def walk_blocks(txt, token, remove):
    """Every top-level `(token ...)` block: `remove(block)` True drops it.
    Paren-balanced, either net dialect (te.strip_net_items's walk)."""
    out, i = [], 0
    while True:
        j = txt.find('(' + token, i)
        while j >= 0 and txt[j + 1 + len(token)] not in ' \n\t':
            j = txt.find('(' + token, j + 1)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = txt[j:k + 1]
        if remove(block):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def block_net(block, nid, name):
    m = re.search(r'\(net (\d+)\)', block)
    m2 = re.search(r'\(net "([^"]+)"\)', block)
    return (m and int(m.group(1)) == nid) or (m2 and m2.group(1) == name)


def block_geom(block, token):
    if token == 'segment':
        s = re.search(r'\(start ' + _NUM + ' ' + _NUM + r'\)', block)
        e = re.search(r'\(end ' + _NUM + ' ' + _NUM + r'\)', block)
        L = re.search(r'\(layer "?([^")\s]+)"?\)', block)
        return ((float(s.group(1)), float(s.group(2))), (float(e.group(1)), float(e.group(2))), L.group(1))
    a = re.search(r'\(at ' + _NUM + ' ' + _NUM + r'\)', block)
    return ((float(a.group(1)), float(a.group(2))), None, None)


def _near(p, q):
    return abs(p[0] - q[0]) < TOL and abs(p[1] - q[1]) < TOL


def in_box(p, box):
    return box[0] <= p[0] <= box[2] and box[1] <= p[1] <= box[3]


def strip_to_fanout_copper(txt, nm, nid, name, pcb_f, box):
    """`txt` (a routed board) with net `nm` reduced to the copper the
    fanout board `pcb_f` carries for it inside `box` -- its TOOTH: the
    lane and the berth go, the tooth stays as the fanout laid it (the
    braid keeps its input copper verbatim)."""
    segs = [s for s in pcb_f.segments if s.net_id == nid
            and in_box((s.start_x, s.start_y), box) and in_box((s.end_x, s.end_y), box)]
    vias = [v for v in pcb_f.vias if v.net_id == nid and in_box((v.x, v.y), box)]

    def rm_seg(block):
        if not block_net(block, nid, name):
            return False
        a, b, L = block_geom(block, 'segment')
        return not any(s.layer == L and ((_near(a, (s.start_x, s.start_y)) and _near(b, (s.end_x, s.end_y)))
                                         or (_near(a, (s.end_x, s.end_y)) and _near(b, (s.start_x, s.start_y))))
                       for s in segs)

    def rm_via(block):
        if not block_net(block, nid, name):
            return False
        a, _, _ = block_geom(block, 'via')
        return not any(_near(a, (v.x, v.y)) for v in vias)
    txt = walk_blocks(txt, 'segment', rm_seg)
    return walk_blocks(txt, 'via', rm_via)


def strip_window(txt, nets, byname, box):
    """`txt` with those nets' copper inside `box` removed (a segment
    wholly inside, a via inside): fanout_once's re-lay strip, as text."""
    ids = {byname[nm][0]: byname[nm][1].name for nm in nets}

    def rm(token):
        def f(block):
            m = re.search(r'\(net (\d+)\)', block)
            m2 = re.search(r'\(net "([^"]+)"\)', block)
            nid = int(m.group(1)) if m else next((i for i, n in ids.items() if m2 and n == m2.group(1)), None)
            if nid not in ids:
                return False
            a, b, _ = block_geom(block, token)
            return in_box(a, box) and (b is None or in_box(b, box))
        return f
    txt = walk_blocks(txt, 'segment', rm('segment'))
    return walk_blocks(txt, 'via', rm('via'))


def write_board(txt, out, pro_from):
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt)
    fp.copy_pro(pro_from, out)


# ------------------------------------------------------------- verdict
def _names(s):
    return [x.strip().strip("'") for x in s.split(',') if x.strip()]


def verdict(stem):
    """What the braid said, per net, off its log, pack sidecar and
    refusal record: class, lane vias, refused + boxed end, ranks."""
    V = {}
    log = open(stem + '.log', encoding='utf-8').read()
    for m in re.finditer(r'^  page F: \[(.*?)\]$', log, re.M):
        for n in _names(m.group(1)):
            V.setdefault(n, {})['cls'] = 'F'
    for m in re.finditer(r'^  page B: \[(.*?)\]\s+swimmers: \[(.*?)\]$', log, re.M):
        for n in _names(m.group(1)):
            V.setdefault(n, {})['cls'] = 'B'
        for n in _names(m.group(2)):
            V.setdefault(n, {})['cls'] = 'swim'
    lo = re.findall(r'^  launch order: \[(.*?)\]$', log, re.M)
    to = re.findall(r'^  target order: \[(.*?)\]', log, re.M)
    for l, t in zip(lo, to):
        for i, n in enumerate(_names(l)):
            V.setdefault(n, {})['lrank'] = i
        for i, n in enumerate(_names(t)):
            V.setdefault(n, {})['trank'] = i
    for m in re.finditer(r'last call routed: (\S+) \((\d+) via', log):
        V.setdefault(m.group(1), {})['last_call'] = int(m.group(2))
    for m in re.finditer(r"rip \[(.*?)\]: (\S+) routed \((\d+) via", log):
        V.setdefault(m.group(2), {})['rip'] = _names(m.group(1))
    # THE BRAID'S OWN CENSUS of what walls a lane: the lanes of the run on
    # its blocked frontier (cells each), and the min-cut probe's crossing
    # set -- the good nets in the way of this bad one, by the real router
    for m in re.finditer(r'rip for (\S+): frontier \d+ cells; lanes of this run on it: (.*?)\s+\(', log):
        fr = re.findall(r'([A-Za-z0-9_]+)\((\d+)\)', m.group(2))
        V.setdefault(m.group(1), {})['frontier'] = [(n, int(c)) for n, c in fr]
    for m in re.finditer(r'rip for (\S+): min-cut probe \d+ via\(s\) crosses \[(.*?)\]', log):
        V.setdefault(m.group(1), {})['mincut'] = _names(m.group(2))
    pk = json.load(open(stem + '.pack.json'))
    for n, l in pk['lanes'].items():
        V.setdefault(n, {})['lane_vias'] = len(l['vias'])
        V[n]['lane_mm'] = round(sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in l['segs']), 1)
    rp = stem + '_refusals.json'
    if os.path.exists(rp):
        for n, info in json.load(open(rp)).items():
            V.setdefault(n, {}).update(refused=True, walled_at=info.get('walled_at'),
                                       pocket=info.get('pocket_mm'))
    # the census carried from earlier rounds and runs (replan writes it
    # beside every board it keeps): an incremental board's own log names
    # only the nets its local braid re-laid
    cp = stem + '.census.json'
    if os.path.exists(cp):
        for n, bl in json.load(open(cp)).items():
            V.setdefault(n, {}).setdefault('census_prev', bl)
    return V


def fmt_v(nm, v, real):
    return (f'{nm:6s} {v.get("cls", "?"):4s} L{v.get("lrank", "?")!s:>3}->T{v.get("trank", "?")!s:<3} '
            f'real {real!s:>2} lane {v.get("lane_vias", 0)}'
            + (f' REFUSED walled at the {v.get("walled_at")}' if v.get('refused') else '')
            + (f' last-call {v["last_call"]}v' if 'last_call' in v else '')
            + (f' rip {v["rip"]}' if 'rip' in v else ''))


# --------------------------------------------------------------- state
class Board:
    """A fanout board F with its plan state read off its source view,
    both ends measured, the menus, the current moves."""

    def __init__(self, F, names, dref, banned=frozenset(), R=None):
        self.F = F
        stem = F[:-len('.kicad_pcb')]
        pcb = parse_kicad_pcb(F)
        self.pcb = pcb
        byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
        self.sv = stem + '_srcview.kicad_pcb'
        source_view(F, self.sv, names, byname, dref)
        self.st = st = fp.plan_state(parse_kicad_pcb(self.sv), names, banned)
        self.byname = st['byname']
        self.names = names
        self.ends = {}
        for nm in names:
            self.ends[nm] = {
                'dst': sr.measure_tooth(pcb, nm, st['dst_pad'][nm], self.byname, dest_ref=st['dref']),
                'src': sr.measure_tooth(pcb, nm, st['src_pad'][nm], self.byname, dest_ref=st['dref'], which='src')}
        self.cur_dst = {nm: fp._menu_match(dmenu_full(st)[nm], self.ends[nm]['dst']) for nm in names}
        self.cur_src = {nm: fp._menu_match(st['smenu'].get(nm, []), self.ends[nm]['src']) for nm in names}
        self.choice = {nm: m for nm, m in self.cur_dst.items() if m is not None}
        self.pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y) for nm in names}
        self.lanes = lane_items(parse_kicad_pcb(R), pcb, names, self.byname) if R else {}
        self.berths = berth_items(pcb, names, self.byname, _pad(st['dgrid'].bbox))

    def cls(self, nm, end):
        g = self.ends[nm][end]
        return (g['direction'], g['layer']) if g else None

    def advance(self, nm, pr):
        """After a standing probe: the moved net's ends (and the co-moved
        berths') as the engine LAID them, so the next net's candidates are
        ranked against the board as it stands -- measured: SBA2's climb
        was asked to the exit SA8's new tooth had just taken."""
        got = {}
        if pr.get('src_got'):
            got[(nm, 'src')] = pr['src_got']
        if pr.get('dst_got'):
            got[(nm, 'dst')] = pr['dst_got']
        for o, g in (pr.get('comove_got') or {}).items():
            if g:
                got[(o, 'dst')] = g
        for (o, end), g in got.items():
            self.ends[o][end] = g
            if end == 'dst':
                self.cur_dst[o] = fp._menu_match(dmenu_full(self.st)[o], g)
                if self.cur_dst[o] is not None:
                    self.choice[o] = self.cur_dst[o]
                else:
                    self.choice.pop(o, None)
            else:
                self.cur_src[o] = fp._menu_match(self.st['smenu'].get(o, []), g)
                self.st['launch'][o] = tuple(g['tooth'])
                self.st['tooth0'][o] = g['layer']
                self.st['tooth_vias'][o] = g['vias']

    def dmenu_honest(self, nm):
        """Every berth move `nm` has on THIS fanout board with the other
        berths as copper (its own excluded): the plan's menu is drawn on a
        bare array, and a site or run the other berths' vias block is one
        the engine will not lay (SA6: 'exact move infeasible even alone'
        for a dogbone the bare menu offered)."""
        import escape_moves as em
        nid = self.byname[nm][0]
        obs = {L: te.build_obstacles(self.pcb, nid, {nid}, L) for L in LAYERS}
        return em.enumerate_moves(
            self.st['dst_pad'][nm], self.st['dgrid'], LAYERS,
            lambda p, q, L: obs[L].seg_clear(p, q),
            lambda p, L: not (obs[L].point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])

    def buses(self):
        return fp.planned_buses(self.st, self.choice)

    def cost(self, choice, launch, tl, tv, buses, cache):
        st = self.st
        return pe.judged_cost(choice, launch, st['dgrid'].bbox, cache, st['sgrid'].bbox,
                              tl, tv, buses, chi=st['chi'])

    def model(self, named):
        """The plan model's prediction per net for the plan this board
        carries (its sidecar's nets), RESIDUAL off: what the sidecar's
        judge said. Returns (pred, bp)."""
        keep, pe.RESIDUAL = pe.RESIDUAL, {}
        try:
            choice = {nm: m for nm, m in self.choice.items() if nm in named}
            achieved = {nm: self.ends[nm]['dst'] for nm in choice}
            _c, pred, bp, _plan = fp.judge_by_braid(self.st, choice, self.F, achieved)
        finally:
            pe.RESIDUAL = keep
        return pred, bp

    def write_sidecar(self, named):
        """The sidecar rewritten from THIS board: the achieved ends and
        layers of every net the previous sidecar named (plus the moved
        ones), as fanout_destination writes it. Returns (pred, bp)."""
        choice = {nm: m for nm, m in self.choice.items() if nm in named}
        achieved = {nm: self.ends[nm]['dst'] for nm in choice}
        keep, pe.RESIDUAL = pe.RESIDUAL, {}
        try:
            _c, pred, bp, plan = fp.judge_by_braid(self.st, choice, self.F, achieved)
        finally:
            pe.RESIDUAL = keep
        side = self.F[:-len('.kicad_pcb')] + '.plan.json'
        with open(side, 'w', encoding='utf-8') as f:
            json.dump(plan, f, indent=1, sort_keys=True)
        return pred, bp, side


# ---------------------------------------------------------- candidates
def rank_dest(B, nm, bans, buses, cache, top):
    """Destination moves for `nm` on the bare array, not banned, not its
    current class, lane free of the other berths as laid; ranked by the
    judged cost with the residual on."""
    st = B.st
    cur = B.cls(nm, 'dst')
    others = {o: m for o, m in B.choice.items() if o != nm}
    pitch = st['dgrid'].pitch_y if True else 0
    pad = st['dst_pad'][nm]

    def depth(m):
        # ball rows (or columns) the move's run crosses on the way out
        ax = 1 if m.direction in ('up', 'down') else 0
        p = st['dgrid'].pitch_y if ax else st['dgrid'].pitch_x
        return abs(m.exit_pt[ax] - (pad.global_y if ax else pad.global_x)) / p
    menu = [m for m in dmenu_full(st)[nm]
            if (m.direction, m.layer) not in bans and (m.direction, m.layer) != cur
            and not legs_cross(m, others)
            and not (m.kind == 'surface' and depth(m) > MAX_DEPTH + 0.6)]
    out = []
    for m in menu:
        # the berths in the way of this move (the bare menu cannot see
        # them): re-fanned WITH the net, the engine negotiating -- the
        # destination co-move. Ranked after the cost: fewest first.
        N = conflicts(m, B.berths, nm)
        if len(N) > MAX_COMOVE:
            continue
        ch = dict(B.choice)
        ch[nm] = m
        out.append((B.cost(ch, st['launch'], st['tooth0'], st['tooth_vias'], buses, cache), len(N), m, N))
    out.sort(key=lambda t: (t[0], t[1], t[2].vias, t[2].climb))
    # one candidate per class: the cheapest of each (face, layer)
    seen, res = set(), []
    for c, n, m, N in out:
        k = (m.direction, m.layer)
        if k in seen:
            continue
        seen.add(k)
        m.comove = sorted(N)
        res.append((c, m))
        if len(res) >= top:
            break
    return res, len(menu)


def rank_src(B, nm, buses, cache, top):
    """Source moves for `nm` (the honest menu, climbs included; the
    banned signatures already left it in plan_state), not its current
    tooth; ranked by the judged cost with the residual on."""
    st = B.st
    g = B.ends[nm]['src']
    menu = st['smenu'].get(nm, [])
    if g:
        menu = [m for m in menu if not (m.layer == g['layer']
                                        and abs(m.exit_pt[0] - g['tooth'][0]) < 0.35
                                        and abs(m.exit_pt[1] - g['tooth'][1]) < 0.35)]
    # two teeth cannot share one exit point whatever their layers (the
    # braid orders lanes by their offset at the array): every other
    # tooth's exit as MEASURED, since the menu names only 30 of 47
    others_exit = [B.ends[o]['src']['tooth'] for o in B.names if o != nm and B.ends[o]['src']]
    others_mv = {o: m for o, m in B.cur_src.items() if o != nm and m is not None}
    menu = [m for m in menu
            if all(abs(m.exit_pt[0] - e[0]) >= pe.sm._EXIT_TOL or abs(m.exit_pt[1] - e[1]) >= pe.sm._EXIT_TOL
                   for e in others_exit)]
    out = []
    for m in menu:
        launch = dict(st['launch'])
        launch[nm] = m.exit_pt
        tl = dict(st['tooth0'])
        tl[nm] = m.layer
        tv = dict(st['tooth_vias'])
        tv[nm] = m.vias
        out.append((B.cost(B.choice, launch, tl, tv, buses, cache), m))
    out.sort(key=lambda t: (t[0], t[1].vias, t[1].climb))
    seen, res = set(), []
    for c, m in out:
        k = (m.direction, m.layer, m.kind, m.climb)
        if k in seen:
            continue
        seen.add(k)
        res.append((c, m))
        if len(res) >= top:
            break
    return res, len(menu)


# ------------------------------------------------------- lanes, conflicts
END_AGREE = 0.5       # mm: an end laid to one ask on two boards counts as the same end
MAX_COMOVE = 3        # berths re-fanned with a destination candidate, at most
MAX_DEPTH = 3         # ball rows a SURFACE berth run may cross: deeper ones the
                      # engine's plan-follow does not lay (SA12: a 7-row run on F
                      # asked, 'infeasible even alone', the original re-laid)
TRACK_CLEAR = 0.30    # centre to centre: track + clearance + the engine's grid
VIA_CLEAR = 0.35      # via radius + clearance + half a track + the grid


def _pt_seg_d(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _seg_seg_d(a, b, c, d):
    if pe.sm._proper_cross(a, b, c, d):
        return 0.0
    return min(_pt_seg_d(a, c, d), _pt_seg_d(b, c, d), _pt_seg_d(c, a, b), _pt_seg_d(d, a, b))


def lane_items(pcb_r, pcb_f, names, byname):
    """Per net, the BRAID's copper on the routed board: its segments and
    vias that the fanout board does not carry (the fanout's own copper
    stays verbatim through the braid)."""
    out = {}
    for nm in names:
        nid = byname[nm][0]
        fs = [s for s in pcb_f.segments if s.net_id == nid]
        fv = [v for v in pcb_f.vias if v.net_id == nid]
        segs = [s for s in pcb_r.segments if s.net_id == nid
                and not any(t.layer == s.layer and
                            ((_near((s.start_x, s.start_y), (t.start_x, t.start_y))
                              and _near((s.end_x, s.end_y), (t.end_x, t.end_y)))
                             or (_near((s.start_x, s.start_y), (t.end_x, t.end_y))
                                 and _near((s.end_x, s.end_y), (t.start_x, t.start_y))))
                            for t in fs)]
        vias = [v for v in pcb_r.vias if v.net_id == nid
                and not any(_near((v.x, v.y), (t.x, t.y)) for t in fv)]
        out[nm] = (segs, vias)
    return out


def conflicts(move, lanes, me):
    """The nets whose LANE copper is in the way of `move`'s legs (same
    layer, within a track's clearance) or of its via site (any layer):
    the lanes that must be re-laid for the engine to lay the move."""
    out = set()
    for nm, (segs, vias) in lanes.items():
        if nm == me:
            continue
        hit = False
        for p, q, L in move.legs:
            for s in segs:
                if s.layer == L and _seg_seg_d(p, q, (s.start_x, s.start_y), (s.end_x, s.end_y)) < TRACK_CLEAR:
                    hit = True
                    break
            if hit:
                break
            for v in vias:
                if _pt_seg_d((v.x, v.y), p, q) < VIA_CLEAR:
                    hit = True
                    break
            if hit:
                break
        if not hit and move.site is not None:
            for s in segs:
                if _pt_seg_d(move.site, (s.start_x, s.start_y), (s.end_x, s.end_y)) < VIA_CLEAR:
                    hit = True
                    break
            if not hit and any(math.hypot(v.x - move.site[0], v.y - move.site[1]) < 2 * VIA_CLEAR for v in vias):
                hit = True
        if hit:
            out.add(nm)
    return out


def berth_items(pcb_f, names, byname, box):
    """Per net, its BERTH copper on the fanout board: segments and vias
    inside the destination window."""
    out = {}
    for nm in names:
        nid = byname[nm][0]
        out[nm] = ([s for s in pcb_f.segments if s.net_id == nid
                    and in_box((s.start_x, s.start_y), box) and in_box((s.end_x, s.end_y), box)],
                   [v for v in pcb_f.vias if v.net_id == nid and in_box((v.x, v.y), box)])
    return out


def engine_lays(B, nm, move, end, others=None):
    """THE ENGINE'S OWN ANSWER to one candidate, in memory: the net's copper
    at that end taken off the parsed fanout board, the production fanout
    asked for the move (with `others` {net: move} laid in the same call,
    a co-move group), nothing written. Returns (achieved dict as
    measure_tooth reports it, exact, in_class, seconds) -- the plan's menu
    is drawn on a model whose clearance is exact geometry and whose array
    is bare; the engine's occupancy grid, keep-outs and deepest-first claim
    order refuse a share of its moves ('OTHER CLASS' in the probes).
    Screening candidates here costs ~1 s each against 10-20 s a probe."""
    from pcb_modification import remove_net_from_pcb_data
    from bga_fanout import generate_bga_fanout
    t0 = time.time()
    st, byname = B.st, B.byname
    pcb = B.pcb
    ref = st['sref'] if end == 'src' else st['dref']
    pads = st['src_pad'] if end == 'src' else st['dst_pad']
    group = {nm: move}
    group.update(others or {})
    box = _pad(st['sgrid'].bbox if end == 'src' else st['dgrid'].bbox)
    removed = {}
    for o in group:
        nid = byname[o][0]
        segs = [s_ for s_ in pcb.segments if s_.net_id == nid
                and in_box((s_.start_x, s_.start_y), box) and in_box((s_.end_x, s_.end_y), box)]
        vias = [v for v in pcb.vias if v.net_id == nid and in_box((v.x, v.y), box)]
        removed[o] = (segs, vias)
        rm_s, rm_v = set(map(id, segs)), set(map(id, vias))
        pcb.segments = [s_ for s_ in pcb.segments if id(s_) not in rm_s]
        pcb.vias = [v for v in pcb.vias if id(v) not in rm_v]
    hints = {}
    for o, m in group.items():
        p = pads[o]
        hints[(round(p.global_x, 3), round(p.global_y, 3))] = sr.full_move(m)
    pcb._fanout_all_foreign_immovable = True
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            tracks, vias_add, vias_rm, failed = generate_bga_fanout(
                pcb.footprints[ref], pcb, net_filter=list(group), layers=list(LAYERS),
                track_width=0.1, clearance=0.1, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
                exit_margin=0.5, escape_method='underpad', plane_drop='off',
                escape_dir_hints=hints)
    finally:
        for o, (segs, vias) in removed.items():
            pcb.segments.extend(segs)
            pcb.vias.extend(vias)
    nid = byname[nm][0]
    mine = [t for t in tracks if t['net_id'] == nid]
    if not mine:
        return None, False, False, time.time() - t0
    # the laid end: the segment endpoint used once, away from the pad and
    # the vias (te.endpoints' rule), measured as measure_tooth measures
    cnt = Counter()
    for t in mine:
        cnt[(round(t['start'][0], 3), round(t['start'][1], 3))] += 1
        cnt[(round(t['end'][0], 3), round(t['end'][1], 3))] += 1
    p = pads[nm]
    myv = [v for v in vias_add if v.get('net_id') == nid]
    anchors = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)] + \
        [(v['x'], v['y'], te.VIA_SIZE / 2) for v in myv]
    free = [pt for pt, c in cnt.items() if c == 1
            and all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar) for ax, ay, ar in anchors)]
    if not free:
        return None, False, False, time.time() - t0
    tip = max(free, key=lambda q: (q[0] - p.global_x) ** 2 + (q[1] - p.global_y) ** 2)
    seg = next(t for t in mine if _near(tip, t['start']) or _near(tip, t['end']))
    g = st['sgrid'] if end == 'src' else st['dgrid']
    x0, y0, x1, y1 = g.bbox
    hx, hy = g.pitch_x / 2, g.pitch_y / 2
    beyond = [f for f, ok in (('right', tip[0] > x1 + hx * 0.5), ('left', tip[0] < x0 - hx * 0.5),
                              ('down', tip[1] > y1 + hy * 0.5), ('up', tip[1] < y0 - hy * 0.5)) if ok]
    face = beyond[0] if beyond else move.direction
    pad_r = max(p.size_x, p.size_y) / 2
    kind = ('via_in_pad' if any(math.hypot(v['x'] - p.global_x, v['y'] - p.global_y) <= pad_r + 0.01 for v in myv)
            else 'dogbone' if myv else 'surface')
    got = {'tooth': (round(tip[0], 3), round(tip[1], 3)), 'layer': seg['layer'], 'vias': len(myv),
           'kind': kind, 'direction': face, 'bearing': face,
           'site': ((round(myv[0]['x'], 3), round(myv[0]['y'], 3)) if myv else None)}
    ax = 0 if move.direction in ('up', 'down') else 1
    gap = abs(got['tooth'][ax] - move.exit_pt[ax])
    in_cls = (face, seg['layer']) == (move.direction, move.layer)
    exact = in_cls and kind == move.kind and gap <= sr.GAP_TOL
    return got, exact, in_cls, time.time() - t0


SCREEN_MAX = 8        # dry runs per net and end, at most


def synth_move(nm, got):
    """A Move the engine can be asked for again, from the end it LAID
    (measure_tooth's dict): face, exit, layer, kind, site -- what
    `source_realize.full_move` hands the engine. For an engine substitute
    the plan's menu does not name (SBA2's south-face surface escape, six
    gaps west of any menu move: 109 -> 106 vias)."""
    import escape_moves as em
    if not got or got.get('direction') not in ('left', 'right', 'up', 'down'):
        return None
    return em.Move(nm, got['kind'], got['direction'], got['layer'], tuple(got['tooth']),
                   got.get('vias', 0), [], site=(tuple(got['site']) if got.get('site') else None))


def screen(B, nm, ranked, end, top, log=None):
    """The engine's dry run over the model's ranked candidates: the first
    `top` the engine lays as asked (exact or in class) go forward; where it
    lays ANOTHER class, that laid move (menu-matched) goes forward instead
    -- the engine's own realistic version of the ask. Returns [(cost, move)]."""
    out, seen, n_run, n_ok, n_sub, n_no = [], set(), 0, 0, 0, 0
    for c, m in ranked:
        if len(out) >= top or n_run >= SCREEN_MAX:
            break
        n_run += 1
        others = ({o: B.cur_dst[o] for o in (getattr(m, 'comove', []) or []) if B.cur_dst.get(o)}
                  if end == 'dst' else None)
        got, exact, in_cls, _dt = engine_lays(B, nm, m, end, others)
        if got is None:
            n_no += 1
            continue
        if in_cls:
            n_ok += 1
            key = (m.direction, m.layer, m.kind, round(m.exit_pt[0], 2), round(m.exit_pt[1], 2))
            if key not in seen:
                seen.add(key)
                out.append((c, m))
            continue
        menu_ = B.st['smenu'].get(nm, []) if end == 'src' else dmenu_full(B.st)[nm]
        sub = fp._menu_match(menu_, got) or synth_move(nm, got)
        if sub is None:
            n_no += 1
            continue
        n_sub += 1
        sub.comove = list(getattr(m, 'comove', []) or [])
        key = (sub.direction, sub.layer, sub.kind, round(sub.exit_pt[0], 2), round(sub.exit_pt[1], 2))
        if key not in seen:
            seen.add(key)
            out.append((c, sub))
    if log:
        log(f'    engine screen {nm} {end}: {n_run} dry run(s): {n_ok} as asked, {n_sub} substituted, '
            f'{n_no} not laid -> {len(out)} candidate(s)')
    return out


def legs_cross(m, others):
    """Does a leg of `m` properly cross a leg of another move on the same
    layer? The engine cannot lay that (a surface run across another
    surface run); the selector's non-strict conflict test lets it through
    (measured: SA6's south-face berth asked across the east-face berths'
    row runs, 'exact move infeasible even alone')."""
    for p, q, L in m.legs:
        for om in others.values():
            for r, s_, oL in om.legs:
                if oL == L and pe.sm._proper_cross(p, q, r, s_):
                    return True
    return False


# --------------------------------------------------------------- probe
PROBE_ATTEMPTS = os.environ.get('PROBE_ATTEMPTS', '1')   # the probe braid's attempt ladder
PROBE_BUDGET_X = os.environ.get('PROBE_BUDGET_X', '4')   # its rescue / last-call budget (2 loses nets: measured)


def braid_run(board, out_stem, nets, dref, log_to, probe=False):
    t0 = time.time()
    env = dict(os.environ)
    if probe:
        env['BRAID_ATTEMPTS'] = PROBE_ATTEMPTS
        env['BRAID_BUDGET_X'] = PROBE_BUDGET_X
    r = subprocess.run([sys.executable, '-u', os.path.join(HERE, 'braid.py'),
                        '--board', board, '--dest', dref, '--nets', nets, '--out', out_stem],
                       capture_output=True, text=True, env=env)
    with open(log_to, 'w') as f:
        f.write(r.stdout + r.stderr)
    return os.path.exists(out_stem + '.kicad_pcb'), time.time() - t0


def probe(B, R, nm, src_move, dst_move, tag, K, base, nets_csv, log, extra_relay=None):
    """The real router's answer to ONE move on the routed board R: the
    net stripped to its tooth, the asked end(s) re-fanned against the
    frozen copper, braided alone, graded whole. Returns a dict."""
    t0 = time.time()
    st, byname = B.st, B.byname
    nid, net = byname[nm]
    res = {'net': nm, 'src': src_move, 'dst': dst_move}
    txt = strip_eco(open(R, encoding='utf-8').read())
    txt = strip_to_fanout_copper(txt, nm, nid, net.name, B.pcb, _pad(st['sgrid'].bbox))
    # the LOCAL RE-BRAID: the lanes in the way of the new end are stripped
    # (their teeth and berths stay) and re-laid with the moved net
    C = conflicts(src_move if src_move is not None else dst_move, B.lanes, nm)
    N = list(getattr(dst_move, 'comove', []) or [])
    C |= set(N)           # a re-fanned neighbour's lane is re-laid too
    C |= set((getattr(B, 'blockers', {}) or {}).get(nm, []))   # the braid's census: what walled it
    C |= set(extra_relay or [])
    C.discard(nm)
    whole = (-1e9, -1e9, 1e9, 1e9)
    for c in sorted(C):
        cid, cnet = byname[c]
        txt = strip_to_fanout_copper(txt, c, cid, cnet.name, B.pcb, whole)
    res['relaid'] = sorted(C)
    cur = tag + '_bare.kicad_pcb'
    write_board(txt, cur, R)
    lines = []
    if src_move is not None:
        b1 = tag + '_src.kicad_pcb'
        buf0 = io.StringIO()
        with contextlib.redirect_stdout(buf0), contextlib.redirect_stderr(buf0):
            r = sr.realize(cur, {nm: src_move}, st['src_pad'], byname, st['sref'], b1,
                           log=lines.append, guard_names=())
        with open(tag + '_src.fanout.log', 'w') as f:
            f.write(buf0.getvalue() + '\n'.join(lines))
        a = r['audit'].get(nm, {})
        res['src_laid'] = nm in r['ok']
        res['src_exact'] = bool(a.get('exact'))
        res['src_got'] = a.get('achieved')
        res['src_verdict'] = a.get('verdict')
        if r['rejected'] or nm not in r['ok']:
            res['fail'] = 'source: ' + (r['rejected'] or 'engine refused the tooth')
            res['seconds'] = time.time() - t0
            return res
        cur = b1
    b2 = tag + '_dst.kicad_pcb'
    copy_board(cur, b2)
    move = dst_move if dst_move is not None else B.cur_dst[nm]
    ask = None if move is not None else {nm: B.ends[nm]['dst']['direction']}
    group = [nm] + N
    choice = ({nm: move} if move else {})
    for o in N:
        if B.cur_dst.get(o) is not None:
            choice[o] = B.cur_dst[o]
        else:
            ask = dict(ask or {})
            ask[o] = B.ends[o]['dst']['direction']
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        laid, audit_d, ok = fp.fanout_once(b2, group, choice, st['dst_pad'],
                                           st['dref'], byname, cur, relay=group, already=(),
                                           face_asks=ask)
    res['comove'] = {o: (audit_d.get(o, {}).get('verdict'), bool(audit_d.get(o, {}).get('exact')))
                     for o in N}
    # what the engine LAID for the group: the ask the apply step repeats,
    # so a slot exchange the engine negotiated here is asked for outright
    # there (measured: the same group re-fanned on the fanout board in the
    # engine's deepest-first order kept the neighbour exact and dropped
    # the moved net to its fallback)
    res['comove_got'] = {o: (fp.fanout_once.achieved or {}).get(o) for o in N}
    with open(tag + '_dst.fanout.log', 'w') as f:
        f.write(buf.getvalue())
    got = (fp.fanout_once.achieved or {}).get(nm)
    res['dst_laid'] = got is not None
    res['dst_exact'] = bool(audit_d.get(nm, {}).get('exact')) if move else (got is not None)
    res['dst_got'] = got
    res['dst_verdict'] = audit_d.get(nm, {}).get('verdict')
    res['dst_clean'] = ok
    if got is None:
        res['fail'] = 'destination: engine laid no berth'
        res['seconds'] = time.time() - t0
        return res
    if not ok:
        # a re-fanned berth that grazes a LANE routed against the old one:
        # that lane is re-laid with the group (its net named by the DRC pair)
        pairs = sr.drc_pairs(b2)
        extra = set()
        for ln in pairs:
            for tok in re.findall(r'/([A-Za-z0-9_]+)', ln):
                if tok in byname and tok not in group and tok in B.lanes:
                    extra.add(tok)
        if extra:
            txt2 = open(b2, encoding='utf-8').read()
            for c in sorted(extra):
                cid, cnet = byname[c]
                txt2 = strip_to_fanout_copper(txt2, c, cid, cnet.name, B.pcb, whole)
            write_board(txt2, b2, cur)
            C |= extra
            res['relaid'] = sorted(C)
            ok = not sr.drc_pairs(b2)
        if not ok:
            res['fail'] = 'destination: fanout board not clean/complete (' + '; '.join(pairs[:3]) + ')'
            res['seconds'] = time.time() - t0
            return res
    okb, tb = braid_run(b2, tag + '_rb', ','.join([nm] + sorted(C)), st['dref'], tag + '_rb.log',
                        probe=True)
    res['braid_s'] = tb
    rb = tag + '_rb.kicad_pcb'
    if not okb:
        res['fail'] = 'braid: no board'
        res['seconds'] = time.time() - t0
        return res
    rj = tag + '_rb_refusals.json'
    rf = json.load(open(rj)) if os.path.exists(rj) else {}
    if rf:
        # a refusal with everything else frozen -- the moved net's or a
        # re-laid neighbour's -- is NOT a verdict on the move: the full
        # braid rips and re-lays what this probe cannot (measured: a
        # re-braid of every swimmer on frozen page lanes lost three nets
        # the full braid had closed; SA14's climb routed SA14 at 4 vias
        # and the re-laid SDQ0 refused). Unjudged; the full braid decides.
        res['refused_nets'] = sorted(rf)
        res['unjudged'] = True
        if nm in rf:
            res['refused'] = True
            res['walled_at'] = rf[nm].get('walled_at')
    g, line = grade(rb, K, base)
    res['grade'] = g
    res['board'] = rb
    res['vias_net'] = count_copper(rb, nid)[0]
    res['seconds'] = time.time() - t0
    return res


def _pad(box, m=PAD_MM):
    return (box[0] - m, box[1] - m, box[2] + m, box[3] + m)


def _end_on_routed(pcb_r, B, nm, end):
    """The tooth (end='src') or berth ('dst') of `nm` on a ROUTED board:
    the free end the fanout board's copper would have, found among the
    routed board's segments that match the fanout board's -- the braid
    keeps its input copper verbatim, so the end is the fanout segment
    endpoint the lane continues from."""
    nid = B.byname[nm][0]
    box = _pad(B.st['sgrid'].bbox) if end == 'src' else _pad(B.st['dgrid'].bbox)
    fs = [s_ for s_ in B.pcb.segments if s_.net_id == nid
          and in_box((s_.start_x, s_.start_y), box) and in_box((s_.end_x, s_.end_y), box)]
    rs = [s_ for s_ in pcb_r.segments if s_.net_id == nid
          and any(t.layer == s_.layer and ((_near((s_.start_x, s_.start_y), (t.start_x, t.start_y))
                                           and _near((s_.end_x, s_.end_y), (t.end_x, t.end_y)))
                                          or (_near((s_.start_x, s_.start_y), (t.end_x, t.end_y))
                                              and _near((s_.end_x, s_.end_y), (t.start_x, t.start_y))))
                  for t in fs)]
    if not rs:
        return None
    cnt = Counter()
    for s_ in rs:
        cnt[(round(s_.start_x, 3), round(s_.start_y, 3))] += 1
        cnt[(round(s_.end_x, 3), round(s_.end_y, 3))] += 1
    g = B1_ends_ref = B.ends[nm][end]
    # the fanout copper's free end on the routed board is the one the lane
    # continues from: the endpoint used once that is nearest the fanout
    # board's own measured end
    ends = [pt for pt, c in cnt.items() if c == 1]
    if not ends or g is None:
        return None
    pt = min(ends, key=lambda q: (q[0] - g['tooth'][0]) ** 2 + (q[1] - g['tooth'][1]) ** 2)
    seg = next((s_ for s_ in rs if _near((s_.start_x, s_.start_y), pt) or _near((s_.end_x, s_.end_y), pt)), None)
    return {'tooth': (round(pt[0], 3), round(pt[1], 3)), 'layer': seg.layer if seg else '?',
            'vias': g['vias'], 'kind': g['kind'], 'direction': g['direction']}


def better(g, ref):
    """(opens, drc, vias) lexicographic: fewer opens; no DRC; fewer vias."""
    if g is None or g[0] is None:
        return False
    if g[1] != 0:
        return False
    if len(g[0]) != len(ref[0]):
        return len(g[0]) < len(ref[0])
    return g[2] < ref[2]


def fmt_move(m):
    if m is None:
        return '(engine)'
    if isinstance(m, tuple):
        return f'tooth {fmt_move(m[0])} + berth {fmt_move(m[1])}'
    return sr.fmt_ask(m) + (f' climb {m.climb}' if m.climb else '')


# ---------------------------------------------------------------- main
def main():
    tag, K = ARGV[0], int(ARGV[1])
    if '/' not in tag:
        tag = os.path.join('tmp', tag)
    out_tag = OPTS.get('out', os.path.basename(tag))
    if '/' not in out_tag:
        out_tag = os.path.join('tmp', out_tag)
    base = OPTS.get('board', os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb'))
    dest = OPTS.get('dest', 'DU1')
    ROUNDS = int(OPTS.get('rounds', 4))
    WORST = int(OPTS.get('worst', 3))
    PROBES = int(OPTS.get('probes', 1))
    MIN_VIAS = int(OPTS.get('min-vias', 3))
    MODE = OPTS.get('mode', 'incremental')      # incremental | rebraid
    JOINT = OPTS.get('joint', '1') == '1'        # tooth-and-berth pairs probed too
    GATE_MIN = int(OPTS.get('gate-min', 2))      # bad nets a good one must block to be re-planned
    GATES = int(OPTS.get('gates', 3))            # gatekeepers re-planned per round, at most
    CENSUS = int(OPTS.get('census', 4))          # blockers per bad net taken from the braid's census
    stem = f'{out_tag}_rp_k{K}'
    log = print
    t_all = time.time()
    nets_all = coherent_nets(K, base)
    nets_csv = ','.join(nets_all)
    F0 = f'{tag}_fo_k{K}.kicad_pcb'
    R0 = f'{tag}_k{K}.kicad_pcb'
    if OPTS.get('from'):
        # continue from a previous run's boards: STEM_fo.kicad_pcb (+ its
        # plan sidecar) and STEM.kicad_pcb (+ its braid log and pack)
        F0 = OPTS['from'] + '_fo.kicad_pcb'
        R0 = OPTS['from'] + '.kicad_pcb'
    # the nets planned here: those ending on the destination array
    pcb0 = parse_kicad_pcb(base)
    byname0 = {n.name.split('/')[-1]: (i, n) for i, n in pcb0.nets.items()}
    ends0 = te.endpoints(pcb0, nets_all, byname0)
    dref = Counter(ends0[nm][2] for nm in nets_all if nm in ends0).most_common(1)[0][0]
    names = [nm for nm in nets_all if nm in ends0 and ends0[nm][2] == dref]
    named = set(json.load(open(F0[:-len('.kicad_pcb')] + '.plan.json')).get('ends', {}))
    log(f'replan: K{K} {len(names)} nets on {dref} ({len(nets_all) - len(names)} elsewhere), '
        f'from {os.path.basename(F0)} / {os.path.basename(R0)}; {ROUNDS} round(s), '
        f'{WORST} worst, {PROBES} probe(s) per end, SRC_CLIMB={os.environ["SRC_CLIMB"]}')

    # round 0: the recorded boards as they stand
    F, R = f'{stem}_r0_fo.kicad_pcb', f'{stem}_r0.kicad_pcb'
    copy_board(F0, F)
    shutil.copy(F0[:-len('.kicad_pcb')] + '.plan.json', F[:-len('.kicad_pcb')] + '.plan.json')
    copy_board(R0, R, eco=True)
    for ext in ('.log', '.pack.json', '_refusals.json', '.census.json'):
        src = R0[:-len('.kicad_pcb')] + ext
        if os.path.exists(src):
            shutil.copy(src, R[:-len('.kicad_pcb')] + ext)
    best_g, line = grade(R, K, base)
    log(f'  start: open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]}')
    bans_d = {nm: set() for nm in names}
    bans_s = set()             # (net, move sig) at the source
    bans_pair = {nm: set() for nm in names}   # (source sig, berth face, berth layer) tried together
    tried = Counter()
    prevV = None
    census_hist = {}
    import gc
    import resource
    for rnd in range(1, ROUNDS + 1):
        t_r = time.time()
        gc.collect()
        log(f'\n=== round {rnd}: verdict off {os.path.basename(R)}  '
            f'(peak rss {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1048576:.0f} MB)')
        B = Board(F, names, dref, banned=frozenset(bans_s), R=R)
        V = verdict(R[:-len('.kicad_pcb')])
        if prevV is not None:
            # an incremental board's log names only the re-laid nets: the
            # others keep the class the last full verdict gave them
            for nm in names:
                if 'cls' not in V.get(nm, {}) and 'cls' in prevV.get(nm, {}):
                    V.setdefault(nm, {})['cls'] = prevV[nm]['cls']
        pcbR = parse_kicad_pcb(R)
        real = {nm: sum(1 for v in pcbR.vias if v.net_id == B.byname[nm][0]) for nm in names}
        for nm in names:
            # the LANE's vias, off the board: the ends' vias taken off
            ev = sum((B.ends[nm][e] or {}).get('vias', 0) for e in ('src', 'dst'))
            V.setdefault(nm, {})['lane_vias'] = max(0, real[nm] - ev)
        prevV = V
        pred, bp = B.model(named)
        # THE LEARNED PRICE: real - predicted, per net, on the plan routed
        resid = {}
        for nm in names:
            if nm in pred and not V.get(nm, {}).get('refused'):
                resid[nm] = real[nm] - pred[nm]
        pe.RESIDUAL = resid
        B.swimmers = {nm for nm in names if V.get(nm, {}).get('cls') == 'swim' and not V[nm].get('refused')}
        nsw_plan = sum(1 for nm in names if nm in bp and bp[nm]['page'] is None)
        nsw_braid = sum(1 for nm in names if V.get(nm, {}).get('cls') == 'swim')
        log(f'  plan model on this fanout: {sum(pred.values())} vias over {len(pred)} nets, '
            f'{nsw_plan} swimmers; the braid: {sum(real.values())} vias, {nsw_braid} swimmers, '
            f'refused {[n for n in names if V.get(n, {}).get("refused")]}; '
            f'residual real-pred: total {sum(resid.values()):+d}, '
            f'|.| {sum(abs(x) for x in resid.values())} over {len(resid)} nets')
        worst = [nm for nm in names if V.get(nm, {}).get('refused')]
        sw = sorted((nm for nm in names if not V.get(nm, {}).get('refused')
                     and V[nm].get('lane_vias', 0) >= MIN_VIAS),
                    key=lambda n: (-V[n]['lane_vias'], V[n].get('cls') != 'swim'))
        worst += sw[:WORST]
        bad = set(worst)
        log(f'  worst: ' + '; '.join(fmt_v(nm, V[nm], real[nm]) for nm in worst))
        # THE BLOCKER CENSUS: for each bad net, the lanes the braid found
        # in its way (this round's log, and every earlier round's); a good
        # net in the way of GATE_MIN bad ones is a GATEKEEPER -- not frozen:
        # its lane is re-laid in their probes and it is re-planned itself
        for nm in names:
            v = V.get(nm, {})
            bl = list(v.get('mincut', [])) + [n for n, c in sorted(v.get('frontier', []), key=lambda t: -t[1])]
            seen_b = []
            for b_ in bl:
                if b_ != nm and b_ in B.byname and b_ not in seen_b:
                    seen_b.append(b_)
            for b_ in v.get('census_prev', []):
                if b_ not in seen_b and b_ != nm and b_ in B.byname:
                    seen_b.append(b_)
            if seen_b:
                census_hist.setdefault(nm, [])
                census_hist[nm] = seen_b[:CENSUS] + [b_ for b_ in census_hist[nm] if b_ not in seen_b][:CENSUS]
        blockers = {nm: [b_ for b_ in census_hist.get(nm, []) if b_ not in bad][:CENSUS] for nm in worst}
        gate_score = Counter(b_ for nm in worst for b_ in blockers[nm])
        gates = [g for g, c in gate_score.most_common() if c >= GATE_MIN][:GATES]
        log('  census (the braid\'s own): ' + '; '.join(f'{nm} <- {blockers[nm]}' for nm in worst if blockers[nm]))
        log(f'  gatekeepers (good nets in the way of >= {GATE_MIN} bad ones): '
            + (', '.join(f'{g} ({gate_score[g]})' for g in gates) or 'none'))
        B.blockers = blockers
        buses = B.buses()
        cache = {}
        stand = {}          # net -> (src_move, dst_move, probe)
        unjudged = {}       # net -> (end, move, probe): laid in class, refused with everything frozen
        R_cur = R           # the routed board the probes run on (advances in incremental mode)
        g_round0 = list(best_g)
        for nm in worst:
            v = V[nm]
            ends_try = []
            if v.get('refused'):
                w = v.get('walled_at')
                ends_try = ['src'] if w == 'tooth' else ['dst'] if w == 'berth' else ['dst', 'src']
            else:
                ends_try = ['dst', 'src']
            ref_g = best_g
            cands = []
            if len(stand) >= WORST + 2:
                break
            screened = {}
            for end in ends_try:
                if end == 'dst':
                    ranked, n_menu = rank_dest(B, nm, bans_d[nm], buses, cache, SCREEN_MAX)
                else:
                    ranked, n_menu = rank_src(B, nm, buses, cache, SCREEN_MAX)
                ranked = screen(B, nm, ranked, end, PROBES + 1, log)
                screened[end] = ranked
                log(f'  {nm} {end}: {n_menu} move(s) in the menu, current '
                    f'{sr.fmt(B.ends[nm][end])}; candidates: '
                    + ('; '.join(f'{fmt_move(m)} judged {c:.1f}' for c, m in ranked[:PROBES]) or 'none'))
                for c, m in ranked[:PROBES]:
                    cands.append((end, m, c))
            if 'src' in ends_try and 'dst' in ends_try and JOINT:
                # JOINT candidates: a tooth AND a berth moved together -- a
                # net whose launch and target ranks are both wrong looks
                # worse after either end alone (SA12's win took two rounds,
                # tooth south then berth east, because the first happened to
                # pay by itself); ranked by the judged cost with both applied
                rs_ = screened.get('src', [])
                rd_ = screened.get('dst', [])
                pairs = []
                for cs, ms in rs_:
                    for cd, md in rd_:
                        if (sr.move_sig(ms), md.direction, md.layer) in bans_pair[nm]:
                            continue
                        launch = dict(B.st['launch'])
                        launch[nm] = ms.exit_pt
                        tl = dict(B.st['tooth0'])
                        tl[nm] = ms.layer
                        tv = dict(B.st['tooth_vias'])
                        tv[nm] = ms.vias
                        ch = dict(B.choice)
                        ch[nm] = md
                        pairs.append((B.cost(ch, launch, tl, tv, buses, cache), ms, md))
                pairs.sort(key=lambda t: t[0])
                if pairs:
                    log(f'  {nm} both: {len(pairs)} pair(s); best: '
                        + '; '.join(f'{fmt_move(ms)} + {fmt_move(md)} judged {c:.1f}' for c, ms, md in pairs[:PROBES]))
                for c, ms, md in pairs[:PROBES]:
                    cands.append(('both', (ms, md), c))
            results = []
            for end, m, c in cands:
                tried[nm] += 1
                ptag = f'{stem}_r{rnd}_{nm}_{end}{tried[nm]}'
                if end == 'both':
                    pr = probe(B, R_cur, nm, m[0], m[1], ptag, K, base, nets_csv, log)
                else:
                    pr = probe(B, R_cur, nm, m if end == 'src' else None, m if end == 'dst' else None,
                               ptag, K, base, nets_csv, log)
                if 'fail' in pr:
                    log(f'    probe {end} {fmt_move(m)}: FAILED ({pr["fail"]}; '
                        f'{pr.get("src_verdict") or pr.get("dst_verdict") or ""}) {pr["seconds"]:.0f} s')
                    if end == 'dst':
                        bans_d[nm].add((m.direction, m.layer))
                    elif end == 'src':
                        bans_s.add((nm, sr.move_sig(m)))
                    else:
                        bans_pair[nm].add((sr.move_sig(m[0]), m[1].direction, m[1].layer))
                    continue
                g = pr['grade']
                if end == 'both':
                    ms, md = m
                    faith = bool(pr.get('src_exact')) and bool(pr.get('dst_exact'))
                    in_cls = ((pr['src_got']['direction'], pr['src_got']['layer']) == (ms.direction, ms.layer)
                              and (pr['dst_got']['direction'], pr['dst_got']['layer']) == (md.direction, md.layer))
                    verdict_s = f'{pr.get("src_verdict")} / {pr.get("dst_verdict")}'
                    laid_cls = None
                else:
                    faith = ((pr.get('src_exact', True) if end == 'src' else True)
                             and (pr.get('dst_exact', True) if end == 'dst' else True))
                    laid_cls = ((pr['src_got']['direction'], pr['src_got']['layer']) if end == 'src'
                                else (pr['dst_got']['direction'], pr['dst_got']['layer']))
                    in_cls = laid_cls == (m.direction, m.layer)
                    verdict_s = (pr.get('src_verdict') if end == 'src' else pr.get('dst_verdict')) or ''
                # the candidate's OWN net must have routed: a board that
                # grades better because the re-laid neighbours came out
                # cheaper is no verdict on the move (measured: SA11's berth
                # 'stood' at 107 vias with SA11 still refused)
                # the engine may have laid another class than asked: if the
                # board is better all the same, the LAID move (menu-matched)
                # is what the apply step asks for, and the asked class, which
                # the engine would not lay, is banned
                substitute = None
                if end != 'both' and not in_cls and better(g, ref_g) and not pr.get('refused') and not pr.get('unjudged'):
                    got = pr['src_got'] if end == 'src' else pr['dst_got']
                    menu_ = B.st['smenu'].get(nm, []) if end == 'src' else dmenu_full(B.st)[nm]
                    substitute = fp._menu_match(menu_, got) or synth_move(nm, got)
                    if substitute is not None:
                        substitute.comove = list(getattr(m, 'comove', []) or [])
                ok = better(g, ref_g) and (in_cls or substitute is not None) and not pr.get('refused')
                if substitute is not None:
                    if end == 'dst':
                        bans_d[nm].add((m.direction, m.layer))
                    else:
                        bans_s.add((nm, sr.move_sig(m)))
                    m = substitute
                if pr.get('unjudged') and in_cls:
                    # the best unjudged per net: its own net routed first,
                    # then the board's opens and vias
                    key = (pr.get('refused', False), len(g[0]), g[2])
                    if nm not in unjudged or key < unjudged[nm][3]:
                        unjudged[nm] = (end, m, pr, key)
                laid_s = (f'{sr.fmt(pr["src_got"])} + {sr.fmt(pr["dst_got"])}' if end == 'both'
                          else sr.fmt(pr["src_got"] if end == "src" else pr["dst_got"]))
                log(f'    probe {end} {fmt_move(m)}: laid {laid_s} '
                    f'[{"exact" if faith else ("in class" if in_cls else "OTHER CLASS")}]'
                    + (f' -- {verdict_s}' if not faith else '')
                    + (f'; REFUSED (walled at the {pr.get("walled_at")})' if pr.get('refused') else
                       f'; routed: net {pr["vias_net"]} v (was {real[nm]})')
                    + (f' with {pr["relaid"]} re-laid' if pr.get('relaid') else ' alone')
                    + (f', co-moved {pr["comove"]}' if pr.get('comove') else '')
                    + (f', refused {pr["refused_nets"]}' if pr.get('refused_nets') else '')
                    + f'; board open {g[0]} drc {g[1]} vias {g[2]} (ref {len(ref_g[0])}/{ref_g[2]})'
                    f' -> {"STANDS" if ok else ("unjudged" if pr.get("unjudged") and in_cls else "rejected")}'
                    + (f' (the engine\'s substitute {fmt_move(m)} is the ask)' if substitute is not None else '')
                    + f' ({pr["seconds"]:.0f} s)')
                if ok:
                    results.append((len(g[0]), g[2], end, m, pr))
                elif not (pr.get('unjudged') and in_cls) and substitute is None:
                    if end == 'dst':
                        bans_d[nm].add((m.direction, m.layer))
                    elif end == 'src':
                        bans_s.add((nm, sr.move_sig(m)))
                    else:
                        bans_pair[nm].add((sr.move_sig(m[0]), m[1].direction, m[1].layer))
            if results:
                results.sort(key=lambda t: (t[0], t[1]))
                _o, _v, end, m, pr = results[0]
                stand[nm] = ((m[0], m[1], pr) if end == 'both'
                             else (m if end == 'src' else None, m if end == 'dst' else None, pr))
                if MODE == 'incremental':
                    # the probe's board IS a routed board (graded whole):
                    # the next net is probed on it, and the round ships it
                    R_cur = pr['board']
                    best_g = list(pr['grade'])
                    B.lanes = lane_items(parse_kicad_pcb(R_cur), B.pcb, names, B.byname)
                    B.advance(nm, pr)
                    log(f'    {nm}: {os.path.basename(R_cur)} is the board now '
                        f'(open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]})')
        # PHASE 2: each gatekeeper tried at another class of its own,
        # judged by the local braid with the bad nets it blocks re-laid
        for g in gates:
            if g in stand or g in bad:
                continue
            blocked_by_g = [nm for nm in worst if g in blockers.get(nm, [])]
            ref_g = best_g
            cands = []
            for end in ('dst', 'src'):
                if end == 'dst':
                    ranked, n_menu = rank_dest(B, g, bans_d[g], buses, cache, SCREEN_MAX)
                else:
                    ranked, n_menu = rank_src(B, g, buses, cache, SCREEN_MAX)
                ranked = screen(B, g, ranked, end, PROBES, log)
                log(f'  gatekeeper {g} {end}: {n_menu} move(s), current {sr.fmt(B.ends[g][end])}, blocks '
                    f'{blocked_by_g}; candidates: '
                    + ('; '.join(f'{fmt_move(m)} judged {c:.1f}' for c, m in ranked) or 'none'))
                for c, m in ranked:
                    cands.append((end, m, c))
            results = []
            for end, m, c in cands:
                tried[g] += 1
                ptag = f'{stem}_r{rnd}_{g}_{end}{tried[g]}'
                pr = probe(B, R_cur, g, m if end == 'src' else None, m if end == 'dst' else None,
                           ptag, K, base, nets_csv, log, extra_relay=blocked_by_g)
                if 'fail' in pr:
                    log(f'    probe {end} {fmt_move(m)}: FAILED ({pr["fail"]}) {pr["seconds"]:.0f} s')
                    if end == 'dst':
                        bans_d[g].add((m.direction, m.layer))
                    else:
                        bans_s.add((g, sr.move_sig(m)))
                    continue
                gg = pr['grade']
                laid_cls = ((pr['src_got']['direction'], pr['src_got']['layer']) if end == 'src'
                            else (pr['dst_got']['direction'], pr['dst_got']['layer']))
                in_cls = laid_cls == (m.direction, m.layer)
                ok = better(gg, ref_g) and in_cls and not pr.get('refused')
                log(f'    probe {end} {fmt_move(m)}: laid {sr.fmt(pr["src_got"] if end == "src" else pr["dst_got"])} '
                    f'[{"in class" if in_cls else "OTHER CLASS"}]'
                    + (f'; REFUSED' if pr.get('refused') else f'; routed: net {pr["vias_net"]} v (was {real[g]})')
                    + f' with {pr.get("relaid")} re-laid'
                    + (f', refused {pr["refused_nets"]}' if pr.get('refused_nets') else '')
                    + f'; board open {gg[0]} drc {gg[1]} vias {gg[2]} (ref {len(ref_g[0])}/{ref_g[2]})'
                    f' -> {"STANDS" if ok else ("unjudged" if pr.get("unjudged") and in_cls else "rejected")}'
                    f' ({pr["seconds"]:.0f} s)')
                if ok:
                    results.append((len(gg[0]), gg[2], end, m, pr))
                elif not (pr.get('unjudged') and in_cls):
                    if end == 'dst':
                        bans_d[g].add((m.direction, m.layer))
                    else:
                        bans_s.add((g, sr.move_sig(m)))
            if results:
                results.sort(key=lambda t: (t[0], t[1]))
                _o, _v, end, m, pr = results[0]
                stand[g] = (m if end == 'src' else None, m if end == 'dst' else None, pr)
                if MODE == 'incremental':
                    R_cur = pr['board']
                    best_g = list(pr['grade'])
                    B.lanes = lane_items(parse_kicad_pcb(R_cur), B.pcb, names, B.byname)
                    B.advance(g, pr)
                    log(f'    {g}: {os.path.basename(R_cur)} is the board now '
                        f'(open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]})')
        if not stand and unjudged and MODE == 'incremental':
            log(f'  round {rnd}: nothing judged better; {len(unjudged)} unjudged move(s) would need the '
                f'full braid (--mode=rebraid) -- stopping ({time.time() - t_r:.0f} s)')
            break
        if not stand and unjudged:
            # nothing judged better: the unjudged moves go to the full
            # braid, which is the only judge left for them
            for nm, (end, m, pr, _k) in unjudged.items():
                stand[nm] = ((m[0], m[1], pr) if end == 'both'
                             else (m if end == 'src' else None, m if end == 'dst' else None, pr))
            log(f'  round {rnd}: nothing judged better; applying the unjudged moves for the full braid: '
                + ', '.join(f'{nm} {end} {fmt_move(m)}' for nm, (end, m, pr, _k) in unjudged.items()))
        if not stand:
            log(f'  round {rnd}: no candidate stands -- stopping ({time.time() - t_r:.0f} s)')
            break
        # standing moves must not CONFLICT with each other (two nets asked
        # for one berth slot: the engine laid neither as asked, measured):
        # the better-graded one keeps its move, the other waits a round
        def _grade_key(t):
            pr = t[2]
            g = pr.get('grade') or [[None] * 99, 0, 10 ** 6]
            return (pr.get('refused', False), len(g[0]), g[2])
        kept = {}
        for nm in sorted(stand, key=lambda n: _grade_key(stand[n])):
            sm_, dm_, pr_ = stand[nm]
            clash = None
            for o, (so, do, _p) in kept.items():
                if dm_ is not None and do is not None and pe.sm._conflict(dm_, do, strict=True):
                    clash = o
                if sm_ is not None and so is not None and pe.sm._conflict(sm_, so, strict=True):
                    clash = o
            if clash:
                log(f'  {nm}: its move conflicts with {clash}\'s -- deferred to a later round')
                continue
            kept[nm] = stand[nm]
        stand = kept
        # ---- APPLY to the fanout board
        src_moves = {nm: s for nm, (s, d, _) in stand.items() if s is not None}
        dst_moves = {nm: d for nm, (s, d, _) in stand.items() if d is not None}
        log(f'  applying to {os.path.basename(F)}: source {list(src_moves)}, destination {list(dst_moves)}')
        F1 = f'{stem}_r{rnd}_fo.kicad_pcb'
        txt = open(F, encoding='utf-8').read()
        unfaithful = []
        if src_moves:
            # the moved nets' berths go first (one free end for the
            # measure; the berth is re-laid with the others below)
            txt = strip_window(txt, list(src_moves), B.byname, _pad(B.st['dgrid'].bbox))
            f1s = f'{stem}_r{rnd}_fo_srcstrip.kicad_pcb'
            write_board(txt, f1s, F)
            lines = []
            r = sr.realize(f1s, src_moves, B.st['src_pad'], B.byname, B.st['sref'],
                           f'{stem}_r{rnd}_fo_src.kicad_pcb', log=lines.append, guard_names=names)
            for l in lines:
                if any(nm in l for nm in src_moves) or 'audit' in l or 'unmoved' in l or 'REJECTED' in l:
                    log('    ' + l.strip())
            if r['rejected']:
                best_g = g_round0
                log(f'  round {rnd}: source realize REJECTED ({r["rejected"]}) -- moves banned, F stays')
                for nm, m in src_moves.items():
                    bans_s.add((nm, sr.move_sig(m)))
                continue
            for nm, m in src_moves.items():
                a = r['audit'][nm]
                g = a.get('achieved')
                if nm not in r['ok'] or g is None or (g['direction'], g['layer']) != (m.direction, m.layer):
                    unfaithful.append(f'{nm} tooth: asked {fmt_move(m)}, got {sr.fmt(g)} -- {a.get("verdict")}')
                    bans_s.add((nm, sr.move_sig(m)))
                elif not a['exact']:
                    log(f'    {nm} tooth laid in class, not exact: {a.get("verdict")}')
            txt = open(r['board'], encoding='utf-8').read()
        write_board(txt, F1, F)
        # every changed net's berth re-laid against the rest (a source-moved
        # net's berth was stripped above)
        comoved = sorted({o for m in dst_moves.values() for o in (getattr(m, 'comove', []) or [])}
                         - set(dst_moves) - set(src_moves))
        changed = sorted(set(src_moves) | set(dst_moves) | set(comoved))
        if comoved:
            log(f'  co-moved berths (re-fanned with their neighbour, the engine negotiating): {comoved}')
        choice_all = dict(B.choice)
        choice_all.update(dst_moves)
        harvested = []
        for nm, (s_, d_, pr) in stand.items():
            if d_ is None:
                continue
            for o, got in (pr.get('comove_got') or {}).items():
                mv = (fp._menu_match(dmenu_full(B.st)[o], got) or synth_move(o, got)) if got else None
                if mv is not None and sr.move_sig(mv) != sr.move_sig(choice_all.get(o, mv)):
                    choice_all[o] = mv
                    harvested.append(f'{o} -> {fmt_move(mv)}')
            got = pr.get('dst_got')
            if got and not pr.get('dst_exact'):
                mv = fp._menu_match(dmenu_full(B.st)[nm], got) or synth_move(nm, got)
                if mv is not None and (mv.direction, mv.layer) == (d_.direction, d_.layer):
                    choice_all[nm] = mv
                    dst_moves[nm] = mv
                    harvested.append(f'{nm} -> {fmt_move(mv)} (the probe\'s own laid gap)')
        if harvested:
            log(f'  asks harvested from the probes\' laid berths: {harvested}')
        ask = {nm: B.ends[nm]['dst']['direction'] for nm in changed if nm not in choice_all}
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            laid, audit_d, ok = fp.fanout_once(F1, names, choice_all, B.st['dst_pad'], dref,
                                               B.byname, F, relay=changed,
                                               already=[nm for nm in names if nm not in changed],
                                               face_asks=ask or None)
        with open(f'{stem}_r{rnd}_fo.log', 'w') as f:
            f.write(buf.getvalue())
        achieved = fp.fanout_once.achieved or {}
        for nm in changed:
            a = audit_d.get(nm, {})
            g = achieved.get(nm)
            want = choice_all.get(nm)
            if g is None:
                unfaithful.append(f'{nm} berth: engine laid none')
            elif nm in comoved and (g['direction'], g['layer']) != (want.direction, want.layer):
                log(f'    {nm} berth CO-MOVED by the engine: {fmt_move(want)} -> {sr.fmt(g)}')
            elif want is not None and (g['direction'], g['layer']) != (want.direction, want.layer):
                unfaithful.append(f'{nm} berth: asked {fmt_move(want)}, got {sr.fmt(g)} -- {a.get("verdict")}')
                if nm in dst_moves:
                    bans_d[nm].add((want.direction, want.layer))
            elif want is not None and not a.get('exact'):
                log(f'    {nm} berth laid in class, not exact: {a.get("verdict")}')
            else:
                log(f'    {nm} berth {sr.fmt(g)}' + (' exact' if a.get('exact') else ' (engine\'s own)'))
        log(f'  fanout board {os.path.basename(F1)}: {"clean and complete" if ok else "NOT clean/complete"}'
            + (f'; UNFAITHFUL: ' + ' | '.join(unfaithful) if unfaithful else '; every move laid in its class'))
        if not ok or unfaithful:
            best_g = g_round0
            log(f'  round {rnd}: the fanout board is {"not clean" if not ok else "unfaithful"} -- '
                f'no braid; the moves not laid as asked are banned, F stays')
            for nm, m in dst_moves.items():
                if not ok or any(u.startswith(nm + ' ') for u in unfaithful):
                    bans_d[nm].add((m.direction, m.layer))
            for nm, m in src_moves.items():
                if not ok or any(u.startswith(nm + ' ') for u in unfaithful):
                    bans_s.add((nm, sr.move_sig(m)))
            continue
        # the UNMOVED ends must be where they were
        B1 = Board(F1, names, dref, banned=frozenset(bans_s))
        drift = []
        for nm in names:
            if nm in changed:
                continue
            for end in ('src', 'dst'):
                a, b = B.ends[nm][end], B1.ends[nm][end]
                if (a is None) != (b is None) or (a and (a['tooth'] != b['tooth'] or a['layer'] != b['layer']
                                                         or a['vias'] != b['vias'])):
                    drift.append(f'{nm}.{end}')
        log(f'  unmoved ends: {2 * (len(names) - len(changed)) - len(drift)}/{2 * (len(names) - len(changed))} '
            f'unchanged' + (f'; DRIFTED {drift}' if drift else ''))
        named1 = (named | set(changed)) & set(B1.choice)
        pred1, bp1, side = B1.write_sidecar(named1)
        nsw1 = sum(1 for nm in names if nm in bp1 and bp1[nm]['page'] is None)
        log(f'  sidecar {os.path.basename(side)}: {len(named1)} nets named; plan model '
            f'{sum(pred1.values())} vias (residual off), {nsw1} swimmers on paper')
        if MODE == 'incremental' and stand and R_cur != R:
            # the fanout board and the routed board must carry the SAME
            # ends for every changed net (both laid by the engine to the
            # same asks): then the probes' board ships as this round's
            # the routed board's ends for the changed nets are what the
            # probes' engine calls LAID there (measured then); the fanout
            # board's are measured now. Same face and layer, the gap within
            # END_AGREE (two engine runs to one ask land a gap apart)
            laid_on_r = {}
            for nm_, (s_, d_, pr) in stand.items():
                if pr.get('src_got'):
                    laid_on_r[(nm_, 'src')] = pr['src_got']
                if pr.get('dst_got'):
                    laid_on_r[(nm_, 'dst')] = pr['dst_got']
                for o, g in (pr.get('comove_got') or {}).items():
                    if g:
                        laid_on_r[(o, 'dst')] = g
            mismatch = []
            for nm in changed:
                for e in ('src', 'dst'):
                    a = B1.ends[nm][e]
                    b = laid_on_r.get((nm, e), B.ends[nm][e])   # unchanged at this end: as it was
                    if a is None or b is None or a['layer'] != b['layer'] \
                            or a['direction'] != b['direction'] \
                            or math.hypot(a['tooth'][0] - b['tooth'][0], a['tooth'][1] - b['tooth'][1]) > END_AGREE:
                        mismatch.append(f'{nm}.{e}: fanout {sr.fmt(a)} vs routed {sr.fmt(b)}')
            if mismatch:
                log(f'  ends DIFFER between the fanout board and the routed board: {mismatch} '
                    f'-> the full braid decides')
            else:
                R1 = f'{stem}_r{rnd}.kicad_pcb'
                copy_board(R_cur, R1, eco=True)
                for ext in ('.log', '.pack.json', '_refusals.json'):
                    src = R_cur[:-len('.kicad_pcb')] + ext
                    if os.path.exists(src):
                        shutil.copy(src, R1[:-len('.kicad_pcb')] + ext)
                with open(R1[:-len('.kicad_pcb')] + '.census.json', 'w') as f:
                    json.dump(census_hist, f, indent=1, sort_keys=True)
                g1, line = grade(R1, K, base)
                keep = better(g1, g_round0) and not drift
                log(f'  round {rnd}: {"KEPT" if keep else "rejected"} incremental -- open {g1[0]}, drc {g1[1]}, '
                    f'vias {g1[2]} (round start {g_round0[0]}/{g_round0[2]}); ends of {len(changed)} changed '
                    f'net(s) agree on both boards ({time.time() - t_r:.0f} s)')
                if keep:
                    F, R, best_g = F1, R1, g1
                else:
                    best_g = g_round0
                continue
        # ---- BRAID
        R1 = f'{stem}_r{rnd}'
        okb, tb = braid_run(F1, R1, nets_csv, dest, R1 + '.log')
        R1 += '.kicad_pcb'
        if not okb:
            log(f'  round {rnd}: braid produced no board ({tb:.0f} s) -- F stays')
            continue
        g1, line = grade(R1, K, base)
        V1 = verdict(R1[:-len('.kicad_pcb')])
        nsw_b1 = sum(1 for nm in names if V1.get(nm, {}).get('cls') == 'swim')
        pl = re.search(r'^plan from .*$', open(R1[:-len('.kicad_pcb')] + '.log').read(), re.M)
        log(f'  braid {tb:.0f} s: open {g1[0]}, drc {g1[1]}, vias {g1[2]}, {nsw_b1} swimmers '
            f'(before: open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]}, {nsw_braid}); '
            f'{pl.group(0) if pl else "no plan line in the braid log"}')
        for nm in changed:
            log(f'    {fmt_v(nm, V1.get(nm, {}), count_copper(R1, B.byname[nm][0])[0])}')
        best_g = g_round0
        keep = better(g1, best_g) and not unfaithful and not drift
        log(f'  round {rnd}: {"KEPT" if keep else "rejected"} '
            f'({"better" if better(g1, best_g) else "not better"}'
            f'{", unfaithful" if unfaithful else ""}{", drift" if drift else ""}) '
            f'({time.time() - t_r:.0f} s)')
        if keep:
            F, R, best_g = F1, R1, g1
        else:
            for nm, m in dst_moves.items():
                bans_d[nm].add((m.direction, m.layer))
            for nm, m in src_moves.items():
                bans_s.add((nm, sr.move_sig(m)))
    final = stem + '.kicad_pcb'
    copy_board(R, final, eco=True)
    for ext in ('.log', '.pack.json', '_refusals.json', '.census.json'):
        src = R[:-len('.kicad_pcb')] + ext
        if os.path.exists(src):
            shutil.copy(src, final[:-len('.kicad_pcb')] + ext)
    with open(final[:-len('.kicad_pcb')] + '.census.json', 'w') as f:
        json.dump(census_hist, f, indent=1, sort_keys=True)
    fo_final = stem + '_fo.kicad_pcb'
    copy_board(F, fo_final)
    shutil.copy(F[:-len('.kicad_pcb')] + '.plan.json', fo_final[:-len('.kicad_pcb')] + '.plan.json')
    log(f'\nreplan: best open {best_g[0]} drc {best_g[1]} vias {best_g[2]} -> {os.path.basename(final)} '
        f'(fanout {os.path.basename(fo_final)}) ({time.time() - t_all:.0f} s)')
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'), final, nets_csv],
                       capture_output=True, text=True)
    print((r.stdout + r.stderr).strip())
    return 0


if __name__ == '__main__':
    sys.exit(main())
