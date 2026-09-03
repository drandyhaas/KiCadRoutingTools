#!/usr/bin/env python3
"""#622 the SURGICAL toolkit: one net's copper, swapped or stripped,
braided ALONE against everyone else's frozen copper, graded, judged.

Factored out of floor_evolve.py (byte-for-byte the same text
surgery) so the floor descent and the slack harvest share one set of
primitives. Nothing here decides anything: it moves copper between
boards, runs the braid on one net (or a few), grades, and enumerates
the geometry of the assignment menu. The loops decide.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

PY = sys.executable


def k_nets(k):
    return subprocess.run(
        [PY, os.path.join(HERE, 'coherent_nets.py'), str(k)],
        capture_output=True, text=True).stdout.strip()


def _walk_strip(txt, token, match):
    out, i = [], 0
    pat = '(' + token
    while True:
        j = txt.find(pat, i)
        while j >= 0 and j + len(pat) < len(txt) \
                and txt[j + len(pat)] not in ' \n\t(':
            j = txt.find(pat, j + 1)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            ch = txt[k]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        if match(txt[j:k + 1]):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def _collect(txt, token, match):
    out, i = [], 0
    pat = '(' + token
    while True:
        j = txt.find(pat, i)
        while j >= 0 and j + len(pat) < len(txt) \
                and txt[j + len(pat)] not in ' \n\t(':
            j = txt.find(pat, j + 1)
        if j < 0:
            break
        k, depth = j, 0
        while True:
            ch = txt[k]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        if match(txt[j:k + 1]):
            out.append(txt[j:k + 1])
        i = k + 1
    return out


def _matcher(nid, name):
    def match(block):
        m = re.search(r'\(net (\d+)\)', block)
        if m:
            return int(m.group(1)) == nid
        m = re.search(r'\(net "([^"]+)"\)', block)
        return bool(m and m.group(1) == name)
    return match


def net_ids(board):
    p = parse_kicad_pcb(board)
    return {n.name.split('/')[-1]: (i, n.name)
            for i, n in p.nets.items()}


def swap_stub(best, src_board, n, out, strip_only_lane=False):
    """best with net n's copper replaced by src_board's copy of n.
    With strip_only_lane, n's copper in `best` is stripped and
    src_board must BE best (in-place re-braid harvest)."""
    ids = net_ids(src_board)
    nid, name = ids[n]
    match = _matcher(nid, name)
    blocks = []
    if not strip_only_lane:
        src_txt = open(src_board, encoding='utf-8').read()
        blocks = (_collect(src_txt, 'segment', match)
                  + _collect(src_txt, 'via', match))
    ids_b = net_ids(best)
    match_b = _matcher(ids_b[n][0], ids_b[n][1])
    txt = open(best, encoding='utf-8').read()
    txt = _walk_strip(txt, 'segment', match_b)
    txt = _walk_strip(txt, 'via', match_b)
    pos = txt.rfind(')')
    open(out, 'w', encoding='utf-8').write(
        txt[:pos] + '\n'.join(blocks) + '\n' + txt[pos:])


def grade(board, nets_csv):
    """(open, drc, vias) among the run's nets; (99, 99, 999) when the
    grader could not read a board."""
    return grade_full(board, nets_csv)[0]


def grade_full(board, nets_csv):
    """((open, drc, vias), [open net short names])."""
    g = subprocess.run(
        [PY, os.path.join(HERE, 'grade_k.py'), board, nets_csv],
        capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', g)
    if not m:
        return (99, 99, 999), []
    mo = re.search(r'open: (\S+)', g)
    opens = mo.group(1).split(',') if mo else []
    return tuple(int(x) for x in m.groups()), opens


def braid_one(board, nets_csv, out, dst='DU1', env=None, pages=None):
    """Braid the named net(s) alone on `board` (everything else is
    frozen copper). True when the braid wrote out.kicad_pcb and
    refused nothing. `pages` ({net: 'F.Cu'|'B.Cu'|None}) pins the
    ride through the board's pages sidecar (a surgical PAGE FLIP --
    the move class the K35 floor gap lives in: the human rides SDQ7
    and the SDQM/SDQ2/SDQ13 group on a constant layer, 0 changes);
    None = free braid (any stale sidecar removed)."""
    side = os.path.splitext(board)[0] + '.pages.json'
    if os.path.exists(side):
        os.remove(side)
    if pages is not None:
        import json
        json.dump(pages, open(side, 'w'), indent=1)
    env = dict(env or os.environ, TWO_PAGE='1')
    r = subprocess.run(
        [PY, os.path.join(HERE, 'braid.py'), '--board', board,
         '--dest', dst, '--nets', nets_csv, '--out', out],
        env=env, capture_output=True, text=True, cwd=HERE)
    return (os.path.exists(out + '.kicad_pcb')
            and 'REFUSED' not in r.stdout)


def relay(fo, n, largs, out):
    """relay_net on the fanout board; returns the DELIVERED key
    (side, layer, coord) or None when the relay missed / wrote
    nothing."""
    r = subprocess.run(
        [PY, os.path.join(HERE, 'relay_net.py'), fo, n, '--out', out]
        + largs, capture_output=True, text=True, cwd=HERE)
    if not os.path.exists(out) or 'MISSED' in r.stdout:
        return None
    md = re.search(r'DELIVERED (\w+)/([\w.]+)@([-\d.]+)', r.stdout)
    if md:
        return (md.group(1), md.group(2), round(float(md.group(3)), 1))
    return ('?', '?', 0.0)


class Menu:
    """The assignment menu's geometry on a fanout board: for a net,
    every (label, relay_net args) -- alt faces x layers at each end,
    plus keep-position layer flips."""

    def __init__(self, fo, src='U1', dst='DU1'):
        self.src, self.dst = src, dst
        self.pcb = parse_kicad_pcb(fo)
        self.BBs = {src: self.bbox(src), dst: self.bbox(dst)}

    def bbox(self, ref):
        fp = self.pcb.footprints[ref]
        xs = [p.global_x for p in fp.pads]
        ys = [p.global_y for p in fp.pads]
        return (min(xs) - 0.25, min(ys) - 0.25,
                max(xs) + 0.25, max(ys) + 0.25)

    def ball(self, ref, nm):
        fp = self.pcb.footprints[ref]
        nid = next((i for i, n2 in self.pcb.nets.items()
                    if n2.name.rsplit('/', 1)[-1] == nm), None)
        p = next((p2 for p2 in fp.pads if p2.net_id == nid), None)
        return (p.global_x, p.global_y) if p else None

    def keep_pos(self, nm, ref, layer):
        return (f'{ref[0]}kp{layer[0]}',
                (['--ref', self.dst] if ref == self.dst else [])
                + ['--keep-pos', '--layer', layer])

    def faces(self, nm, ref):
        out = []
        b = self.ball(ref, nm)
        o = self.ball(self.dst if ref == self.src else self.src, nm)
        if not b or not o:
            return out
        W = self.BBs[ref]
        dx, dy = o[0] - b[0], o[1] - b[1]
        for side in ('up', 'down', 'left', 'right'):
            if side in ('left', 'right'):
                edge = W[0] if side == 'left' else W[2]
                t = (edge - b[0]) / dx if abs(dx) > 1e-9 else -1
                raw = b[1] + t * dy if t > 1e-9 else o[1]
                span = (W[1], W[3])
            else:
                edge = W[1] if side == 'up' else W[3]
                t = (edge - b[1]) / dy if abs(dy) > 1e-9 else -1
                raw = b[0] + t * dx if t > 1e-9 else o[0]
                span = (W[0], W[2])
            coord = min(max(raw, span[0] + 0.3), span[1] - 0.3)
            for L in ('F.Cu', 'B.Cu'):
                out.append((f'{ref[0]}{side[0]}{L[0]}',
                            (['--ref', self.dst] if ref == self.dst
                             else [])
                            + ['--side', side, '--coord',
                               f'{coord:.2f}', '--layer', L]))
        return out

    def cands(self, nm):
        """floor_evolve's sweep order: per end, faces x layers then
        the keep-position flips."""
        out = []
        for ref in (self.src, self.dst):
            if not self.ball(ref, nm):
                continue
            out.extend(self.faces(nm, ref))
            for L in ('F.Cu', 'B.Cu'):
                out.append(self.keep_pos(nm, ref, L))
        return out



def rescue_close(cand, fo, base_open, nets_csv, tag, dst='DU1'):
    """COMPLETION RESCUE (the user's rule, 0902: moves get vetoed on
    completion loss, so fixing completion frees the slack/floor moves
    behind the veto). For every K net open on `cand` but not on the
    base board, braid it ALONE against the candidate's frozen copper
    (stubs restored from fo). Returns (board, grade, opens) -- the
    rescued board when every stranded net closed, else the candidate
    as it was. Measured origin: SA2, K41's chronic open, closed this
    way at +5 vias / 0 drc on the 79v record."""
    g, opens = grade_full(cand, nets_csv)
    stranded = [m for m in opens if m not in base_open]
    if not stranded:
        return cand, g, opens
    cur = cand
    for i, m in enumerate(stranded):
        scr = f'tmp/{tag}_rsc_scr.kicad_pcb'
        swap_stub(cur, fo, m, scr)
        stem = f'tmp/{tag}_rsc{i}_{m}'
        if not braid_one(scr, m, stem, dst=dst):
            cur = None
            break
        cur = stem + '.kicad_pcb'
    if cur is not None:
        g2, opens2 = grade_full(cur, nets_csv)
        if len(opens2) < len(opens):
            return cur, g2, opens2
    # tier 2: the stranded nets as ONE group braid (schedule + last
    # call + rip-assist over all of them). Measured on the K35 batch
    # boards: sequential closes reach 2 of 4 before a refusal, the
    # group reaches 3 of 4 -- strictly further, one braid. A net the
    # batch's own wraps wall in (SA9 there) refuses either way.
    cur = cand
    for i, m in enumerate(stranded):
        nxt = f'tmp/{tag}_rscg_scr{i}.kicad_pcb'
        swap_stub(cur, fo, m, nxt)
        cur = nxt
    stem = f'tmp/{tag}_rscg'
    braid_one(cur, ','.join(stranded), stem, dst=dst)
    if os.path.exists(stem + '.kicad_pcb'):
        g3, opens3 = grade_full(stem + '.kicad_pcb', nets_csv)
        if len(opens3) < len(opens):
            return stem + '.kicad_pcb', g3, opens3
    return cand, g, opens


def relay_surgical(best, fo, nm, largs, stem):
    """Relay net nm's stub IN THE RECORD'S WORLD: strip its lane from
    `best` (stubs restored from fo), run relay_net on THAT board, so
    the engine and the collision check see every other net's frozen
    LANE, not just its fanout stub. Measured need (K35 SA5 north/B):
    relayed on the fanout board the stub was DRC-clean, swapped into
    the record it lay under SCKE0's B lane -- 9 contact overlaps, the
    floor drop 62 -> 60 vetoed on drc in every single and pair that
    carried it. Returns (delivered_key, rel_board, new_fo) -- rel is
    the braid-ready candidate world (nm has stubs only), new_fo the
    fanout lineage carrying nm's new stubs -- or None."""
    scr0 = stem + '_scr0.kicad_pcb'
    swap_stub(best, fo, nm, scr0)
    rel = stem + '_rel.kicad_pcb'
    key = relay(scr0, nm, largs, rel)
    if key is None:
        return None
    newfo = stem + '_fo.kicad_pcb'
    swap_stub(fo, rel, nm, newfo)
    pro = os.path.splitext(fo)[0] + '.kicad_pro'
    if os.path.exists(pro):
        import shutil
        shutil.copy(pro, os.path.splitext(newfo)[0] + '.kicad_pro')
    return key, rel, newfo


def drc_partners(board, nm):
    """Nets whose copper check_drc pairs with nm's on `board`, from
    the checker's own report (grade at the loops' 0.1 / margin 0.1)."""
    r = subprocess.run(
        [PY, os.path.join(HERE, '..', 'py_router', 'check_drc.py'), board,
         '--clearance', '0.1', '--clearance-margin', '0.1'],
        capture_output=True, text=True)
    out = set()
    for m in re.finditer(r'^\s*(/\S.*?) <-> (/.*?)\s*$', r.stdout, re.M):
        a_, b_ = (x.rsplit('/', 1)[-1] for x in m.groups())
        if a_ == nm and b_ != nm:
            out.add(b_)
        elif b_ == nm and a_ != nm:
            out.add(a_)
    return sorted(out)


def realize_relay(best, fo, nm, relfo, stem, dst='DU1', max_blockers=2):
    """Realize a relayed stub SURGICALLY, with its BLOCKERS: swap nm's
    new stubs into the record; if check_drc pairs them with other
    nets' frozen copper (the stub was laid on the fanout board, where
    those nets have only stubs -- K35 SA5 north/B lay under SCKE0's B
    lane, 9 contact overlaps), rip those nets' lanes too (stubs
    restored from fo) and braid the mover WITH them as one group.
    Returns (board, moved_nets) or None. A relay whose stub collides
    with more than max_blockers lanes is refused (that is a re-plan,
    not a move)."""
    scr = stem + '_scr.kicad_pcb'
    swap_stub(best, relfo, nm, scr)
    blockers = drc_partners(scr, nm)
    if len(blockers) > max_blockers:
        return None
    for b in blockers:
        nxt = stem + f'_scr_{b}.kicad_pcb'
        swap_stub(scr, fo, b, nxt)
        scr = nxt
    group = [nm] + blockers
    if not braid_one(scr, ','.join(group), stem + '_b1', dst=dst):
        return None
    return stem + '_b1.kicad_pcb', group


def stub_asks(board, names, refs=('U1', 'DU1'), reach=1.6):
    """Each net's delivered escape ask at each array, read off the
    copper: {net: {ref: (side, coord, layer)}} -- the stub's free end
    inside the array's rip window (degree-1 vertex farthest from the
    array centre), its layer, the face it is nearest, and its
    coordinate along that face. The shape a plan can be ANCHORED to."""
    p = parse_kicad_pcb(board)
    short = {i: n.name.rsplit('/', 1)[-1] for i, n in p.nets.items()}
    want = set(names)
    out = {}
    for ref in refs:
        fp = p.footprints.get(ref)
        if fp is None:
            continue
        xs = [q.global_x for q in fp.pads]
        ys = [q.global_y for q in fp.pads]
        bb = (min(xs), min(ys), max(xs), max(ys))
        RW = (bb[0] - reach, bb[1] - reach, bb[2] + reach, bb[3] + reach)
        cx, cy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
        nids = {q.net_id for q in fp.pads if short.get(q.net_id) in want}
        deg, lay = {}, {}
        for s in p.segments:
            if s.net_id not in nids:
                continue
            for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
                if not (RW[0] <= x <= RW[2] and RW[1] <= y <= RW[3]):
                    continue
                k = (s.net_id, round(x, 3), round(y, 3))
                deg[k] = deg.get(k, 0) + 1
                lay[k] = s.layer
        for v in p.vias:
            if v.net_id in nids and RW[0] <= v.x <= RW[2] \
                    and RW[1] <= v.y <= RW[3]:
                k = (v.net_id, round(v.x, 3), round(v.y, 3))
                deg[k] = deg.get(k, 0) + 2
        ends = {}
        for (nid, x, y), d in deg.items():
            if d != 1:
                continue
            r = (x - cx) ** 2 + (y - cy) ** 2
            if nid not in ends or r > ends[nid][0]:
                ends[nid] = (r, x, y, lay.get((nid, x, y), 'F.Cu'))
        for nid, (_r, x, y, L) in ends.items():
            dist = {'left': abs(x - bb[0]), 'right': abs(x - bb[2]),
                    'up': abs(y - bb[1]), 'down': abs(y - bb[3])}
            side = min(dist, key=dist.get)
            coord = y if side in ('left', 'right') else x
            out.setdefault(short[nid], {})[ref] = (side, round(coord, 3), L)
    return out
