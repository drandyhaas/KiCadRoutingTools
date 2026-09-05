#!/usr/bin/env python3
"""negotiate.py -- WHY a lane is refused, and the rip-up that answers it.

A refused lane is one of three things, and each needs a different
answer, so the first job is to tell them apart with the router itself
rather than guess:

  MISSED    the lane routes against the board as it stands (wider
            window, bigger budget): the refusing search was starved,
            not walled. Answer: route it.
  WALLED    the lane does not route even with EVERY other lane of the
            run removed -- its end is boxed by static copper (a
            fanout stub, a pad, the neighbouring escapes). No rip-up
            inside the braid can fix it; the answer lives at the
            fanout (a different berth or tooth) and the diagnosis
            names the copper that boxes it.
  BLOCKED   it routes when some lanes are removed. WHICH lanes is what
            a rip-up needs to know, and it is discovered rather than
            guessed: the lane is routed with every other lane as SOFT
            copper -- a price per cell, not a wall (connect(soft=...))
            -- so the search returns the path that crosses the fewest
            lanes, and the lanes that path conflicts with are the
            blockers, ordered by how much of the path they carry.

The rip-up is then blocker-directed: rip the top-k blockers, route the
refused net against the rest, re-route the victims one by one against
the new lane, and keep the result only if MORE lanes are routed than
before (a lexicographic improvement: fewer open, then fewer vias). A
victim that fails is itself negotiated one level down with the net
just placed PROTECTED, which is the negotiation of PathFinder-style
routers in the braid's own vocabulary. `history` counts how often a
net has been ripped, and a much-ripped net is a worse victim next
time: that is the learning between rounds, and it is what stops two
lanes trading the same corridor forever.

Everything here works on a STATE: {net name: (segments, vias)} of the
run's nets over a board whose other copper is fixed, plus each net's
FANOUT copper (its state when un-routed: the stubs), so 'rip' means
'reset the net to its fanout copper' and a trimmed berth is restored
with it. Nothing reads a face, an axis or a board name.
"""
import contextlib
import copy
import io
import os
import re
from collections import Counter

import connect as cn
import topo_strings as ts

RIP_RE = re.compile(r'Blocking obstacles: (.*)')
NAME_RE = re.compile(r'([^\s,()]+)\((\d+) track')


def _key_seg(s):
    return (round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
            round(s.end_y, 4), s.layer)


def _key_via(v):
    return (round(v.x, 4), round(v.y, 4))


def split_copper(pcb_fo, pcb_b, nids):
    """Per net id: its FANOUT copper (from pcb_fo) and its CURRENT copper
    (from pcb_b), plus the board's other copper (everything in pcb_b
    that belongs to none of `nids`)."""
    fo = {nid: ([s for s in pcb_fo.segments if s.net_id == nid],
                [v for v in pcb_fo.vias if v.net_id == nid]) for nid in nids}
    cur = {nid: ([s for s in pcb_b.segments if s.net_id == nid],
                 [v for v in pcb_b.vias if v.net_id == nid]) for nid in nids}
    rest = ([s for s in pcb_b.segments if s.net_id not in nids],
            [v for v in pcb_b.vias if v.net_id not in nids])
    return fo, cur, rest


def lane_part(fo_copper, cur_copper):
    """The copper a net gained since its fanout: its lane, plus any
    berth the lane trimmed (a trimmed stub is a different segment)."""
    ks = {_key_seg(s) for s in fo_copper[0]}
    kv = {_key_via(v) for v in fo_copper[1]}
    return ([s for s in cur_copper[0] if _key_seg(s) not in ks],
            [v for v in cur_copper[1] if _key_via(v) not in kv])


def board_with(pcb, rest, state, only=None, skip=()):
    """A shallow copy of `pcb` carrying `rest` plus the state of every
    net in `only` (default all) except those in `skip`."""
    b = copy.copy(pcb)
    segs = list(rest[0])
    vias = list(rest[1])
    for nid, (s, v) in state.items():
        if nid in skip or (only is not None and nid not in only):
            continue
        segs.extend(s)
        vias.extend(v)
    b.segments, b.vias = segs, vias
    return b


def big_cfg(cfg, factor=4):
    big = copy.copy(cfg)
    big.max_iterations = factor * max(cfg.max_iterations, 50_000)
    return big


def hard_route(pcb, nid, a, al, b, bl, cfg, margins=(2.0, 4.0, 6.0),
               window_pts=None, b_alts=None, virtual=None,
               virtual_vias=None, capture=None):
    """The last-call ladder: band-free, window margins escalating, the
    x4 budget. `capture` (a list) receives the router's diagnostics of
    the LAST failing margin, so a walled end can be read back."""
    big = big_cfg(cfg)
    for mg in margins:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res = cn.connect(pcb, nid, a, al, b, bl, big, band=None,
                             margin=mg, window_pts=window_pts, b_alts=b_alts,
                             virtual=virtual, virtual_vias=virtual_vias)
        if res is not None:
            return res
        if capture is not None:
            capture[:] = [buf.getvalue()]
    return None


def stuck_blockers(text):
    """Net names the router blamed at a stuck start/end cell."""
    out = Counter()
    for m in RIP_RE.finditer(text or ''):
        for nm, n in NAME_RE.findall(m.group(1)):
            out[nm.split('/')[-1]] += int(n)
    return out


def conflicts(path, others, cfg):
    """Which of `others` {name: (segs, vias)} the copper `path` (segs,
    vias) would violate: the obstacle model's own distances (same-layer
    track/track, via/track on any layer, via/via)."""
    clr = cfg.clearance - 1e-6
    psegs, pvias = path
    out = Counter()
    for nm, (osegs, ovias) in others.items():
        n = 0
        for s in psegs:
            a, b = (s.start_x, s.start_y), (s.end_x, s.end_y)
            for o in osegs:
                if o.layer != s.layer:
                    continue
                if ts.seg_seg_dist(a, b, (o.start_x, o.start_y),
                                   (o.end_x, o.end_y)) \
                        < (s.width + o.width) / 2 + clr:
                    n += 1
            for v in ovias:
                if ts.seg_pt_dist(a, b, (v.x, v.y)) < v.size / 2 + s.width / 2 + clr:
                    n += 1
        for v in pvias:
            for o in osegs:
                if ts.seg_pt_dist((o.start_x, o.start_y), (o.end_x, o.end_y),
                                  (v.x, v.y)) < v.size / 2 + o.width / 2 + clr:
                    n += 1
            for w in ovias:
                if ts.d2((v.x, v.y), (w.x, w.y)) ** 0.5 < (v.size + w.size) / 2 + clr:
                    n += 1
        if n:
            out[nm] = n
    return out


STUB_COST = 6.0     # mm-equivalent per cell through a STUB: three lanes'
                    # worth -- moving a stub is a re-berth, not a re-lay


def soft_probe(pcb_base, nid, a, al, b, bl, cfg, lanes, margins=(2.0, 4.0, 6.0),
               window_pts=None, b_alts=None, soft_cost=2.0, stubs=None):
    """Route `nid` over `pcb_base` (NO lane copper of the run's other
    nets on it) with `lanes` {name: (segs, vias)} priced instead of
    blocked -- and, with `stubs` {name: (segs, vias)} (the other nets'
    FANOUT copper, absent from pcb_base too), those priced at STUB_COST.
    Returns (path, lane blockers, stub blockers) or (None, None, None)."""
    soft = [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer, s.width)
            for segs, _ in lanes.values() for s in segs]
    soft_v = [(v.x, v.y, v.size) for _, vias in lanes.values() for v in vias]
    if stubs:
        soft += [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer, s.width,
                  STUB_COST) for segs, _ in stubs.values() for s in segs]
        soft_v += [(v.x, v.y, v.size, STUB_COST)
                   for _, vias in stubs.values() for v in vias]
    big = big_cfg(cfg)
    for mg in margins:
        with contextlib.redirect_stdout(io.StringIO()):
            res = cn.connect(pcb_base, nid, a, al, b, bl, big, band=None,
                             margin=mg, window_pts=window_pts, b_alts=b_alts,
                             soft=soft, soft_vias=soft_v, soft_cost=soft_cost)
        if res is not None:
            return (res, conflicts(res, lanes, cfg),
                    conflicts(res, stubs, cfg) if stubs else Counter())
    return None, None, None


class Negotiator:
    """The state machine over one run's nets.

    ends[name] = ((ax, ay), a_layer, (bx, by), b_layer)
    fo[name]   = the net's fanout copper (segs, vias)
    state[name]= its current copper (segs, vias); == fo when unrouted
    rest       = every other piece of copper on the board (fixed)
    """

    def __init__(self, pcb, cfg, byname, ends, fo, state, rest, log=print,
                 window_pts=None, b_alts=None, protected=(), chains=None,
                 board_path=None, dst='DU1', reberth=False):
        self.pcb, self.cfg, self.byname = pcb, cfg, byname
        self.ends, self.fo, self.state, self.rest = ends, fo, state, rest
        self.log = log
        self.window_pts = window_pts or {}
        self.b_alts = b_alts or {}
        # chains[nm]: the berth stub walked from its tip toward the
        # pad as [(seg, tip_pt, pad_pt), ...] (braid ctx.dest_chain).
        # A rip restores the net's fanout copper, but the braid TRIMS
        # a berth once a lane joins it short of the tip, and a later
        # lane may have been laid in the vacated space: the restored
        # tip must be cut back to where it is clear, and the re-lay
        # aimed at the surviving tip (K41 SA4's restored tip on
        # SA11's lane: 2 DRC). Same trim after a join, as the braid.
        self.chains = chains or {}
        self.tgt = {}                 # nm -> (b, b_alts) after a trim
        self.protected = set(protected)
        self.history = Counter()
        self.rungs = Counter()
        self.nid = {nm: byname[nm][0] for nm in ends}
        # RE-BERTH (reberth=True, needs board_path: the fanout board the
        # state was read from): a stub is a wall to every lane, so a
        # net boxed by another net's STUB is 'walled' to the lane-only
        # probe. With stubs priced too, the probe can name the stub,
        # and the remedy is to move that net's berth through its escape
        # menu (relay_net: other faces, layers, keep-position flips)
        # and re-lay its lane -- the rip-up of a berth.
        self.board_path = board_path
        self.dst = dst
        self.reberth = reberth and board_path is not None
        self.reberthed = {}          # nm -> (new fo segs, new fo vias)
        self._bare_txt = None

    # -- views -------------------------------------------------------
    def lane(self, nm):
        return lane_part(self.fo[nm], self.state[nm])

    def routed(self, nm):
        segs, vias = self.lane(nm)
        return bool(segs or vias)

    def n_routed(self):
        return sum(1 for nm in self.state if self.routed(nm))

    def n_vias(self):
        return sum(len(self.lane(nm)[1]) for nm in self.state)

    def board(self, skip=(), lanes_of=None):
        st = {self.nid[nm]: v for nm, v in self.state.items()}
        skip_ids = {self.nid[nm] for nm in skip}
        return board_with(self.pcb, self.rest, st, skip=skip_ids)

    def base_board(self, bare_others=False, keep=()):
        """Every run net at its FANOUT copper: the board with no lane.
        bare_others: every net's fanout copper off the board too, except
        the nets in `keep` (for the stub-priced probe)."""
        st = {self.nid[nm]: self.fo[nm] for nm in self.state
              if not bare_others or nm in keep}
        return board_with(self.pcb, self.rest, st)

    # -- re-berth machinery -------------------------------------------
    def bare_text(self):
        """The fanout board's text with every run net's copper stripped:
        what a state is written onto."""
        if self._bare_txt is None:
            import surgical as sg
            txt = open(self.board_path, encoding='utf-8').read()
            for nm in self.state:
                nid, net = self.byname[nm]
                m = sg._matcher(nid, net.name)
                txt = sg._walk_strip(txt, 'segment', m)
                txt = sg._walk_strip(txt, 'via', m)
            self._bare_txt = txt
        return self._bare_txt

    def write_state(self, out, override=None):
        """Write the current state (or `override` {nm: (segs, vias)}
        for some nets) as a board next to board_path."""
        import os
        import shutil
        from kicad_writer import add_tracks_and_vias_to_pcb
        tmp = out + '.bare.kicad_pcb'
        open(tmp, 'w', encoding='utf-8').write(self.bare_text())
        tracks, vias = [], []
        for nm in self.state:
            segs, vs = (override or {}).get(nm, self.state[nm])
            nid = self.nid[nm]
            for s in segs:
                tracks.append(dict(start=(s.start_x, s.start_y),
                                   end=(s.end_x, s.end_y), width=s.width,
                                   layer=s.layer, net_id=nid))
            for v in vs:
                vias.append(dict(x=v.x, y=v.y, size=v.size, drill=v.drill,
                                 layers=list(v.layers), net_id=nid))
        add_tracks_and_vias_to_pcb(tmp, out, tracks, vias,
                                   net_id_to_name={i: n.name for i, n
                                                   in self.pcb.nets.items()})
        os.remove(tmp)
        for ext in ('.kicad_pro', '.kicad_prl'):
            src = os.path.splitext(self.board_path)[0] + ext
            if os.path.exists(src):
                shutil.copy(src, os.path.splitext(out)[0] + ext)
        return out

    def relay_candidates(self, om):
        """The berth menu for `om` at the destination: faces x layers
        and the keep-position layer flips (surgical.Menu)."""
        import surgical as sg
        menu = sg.Menu(self.board_path, dst=self.dst)
        return [c for c in menu.cands(om) if '--ref' in c[1]]

    def try_reberth(self, om, largs, stem):
        """Relay om's berth on a board carrying the CURRENT lanes of
        every other net (om itself at its fanout copper), read the new
        stub back, and install it: fo[om], ends[om] (new tip + layer),
        chains[om] (none), state[om] = the bare new stub. Returns the
        delivered key or None; the caller restores on failure."""
        import os
        import surgical as sg
        import braid as te
        from kicad_parser import parse_kicad_pcb
        board = self.write_state(stem + '_rb_in.kicad_pcb',
                                 override={om: self.fo[om]})
        out = stem + '_rb_out.kicad_pcb'
        key = sg.relay(board, om, largs, out)
        if key is None or not os.path.exists(out):
            return None
        pr = parse_kicad_pcb(out)
        nid = self.nid[om]
        segs = [x for x in pr.segments if x.net_id == nid]
        vias = [x for x in pr.vias if x.net_id == nid]
        byn = {n.name.split('/')[-1]: (i, n) for i, n in pr.nets.items()}
        try:
            e = te.endpoints(pr, [om], byn, dest_ref=self.dst)[om]
        except AssertionError:
            return None
        tip = e[1]
        lay = te._layer_at(pr, nid, tip, self.ends[om][3])
        # the relay was laid on a board carrying every other lane, but
        # its own collision check is the engine's: grade the new stub
        # against the state's copper here (K41 pp41: a relayed SCKE0
        # stub grazed SA2 by 0.013 mm on F -- accepted, shipped 2 drc)
        others = self.others_of(om)
        if conflicts((segs, vias), others, self.cfg):
            return None
        a, al, _b, _bl = self.ends[om]
        self.fo[om] = (segs, vias)
        self.state[om] = (list(segs), list(vias))
        self.ends[om] = (a, al, tip, lay)
        self.chains[om] = []
        self.tgt.pop(om, None)
        self.b_alts[om] = []
        self.reberthed[om] = (segs, vias)
        return key

    def others_of(self, nm):
        d = {om: self.state[om] for om in self.state if om != nm}
        d['<rest>'] = self.rest
        return d

    def rip(self, nm):
        """Reset `nm` to its fanout copper, trimmed where the berth's
        tip-side segments now collide with copper laid in the space a
        braid trim had vacated; re-aim its target at the survivor."""
        segs, vias = self.fo[nm]
        chain = self.chains.get(nm) or []
        cut = 0
        if chain:
            others = self.others_of(nm)
            for k, (sg, _t, _p) in enumerate(chain):
                if conflicts(([sg], []), others, self.cfg):
                    cut = k + 1
        if cut:
            gone = {id(sg) for (sg, _t, _p) in chain[:cut]}
            segs = [x for x in segs if id(x) not in gone]
            if cut < len(chain):
                b = chain[cut][1]
                alts = [(p_[0], p_[1], self.ends[nm][3])
                        for (_s, _t, p_) in chain[cut:]]
            else:
                b = chain[-1][2]
                alts = []
            self.tgt[nm] = (b, alts)
            self.log(f'      rip {nm}: berth trimmed {cut} seg(s) '
                     f'(space taken), target -> ({b[0]:.2f},{b[1]:.2f})')
        else:
            self.tgt.pop(nm, None)
        self.state[nm] = (list(segs), list(vias))

    def lay(self, nm, res):
        """Accept a routed lane for `nm`: own-via reuse (a new via
        inside an existing own barrel is that barrel), then the
        braid's join trim (tip-side chain segments the lane bypassed
        are dead copper)."""
        segs0, vias0 = self.state[nm]
        new_v = []
        for v in res[1]:
            if any(ts.d2((v.x, v.y), (w.x, w.y)) ** 0.5 < w.size / 2 - 1e-6
                   for w in vias0):
                continue
            new_v.append(v)
        segs = list(segs0) + list(res[0])
        vias = list(vias0) + new_v
        chain = self.chains.get(nm) or []
        if chain:
            endpts = set()
            for x in res[0]:
                endpts.add((round(x.start_x, 3), round(x.start_y, 3)))
                endpts.add((round(x.end_x, 3), round(x.end_y, 3)))
            cut = None
            for k, (_s, t, p_) in enumerate(chain):
                if t in endpts:
                    cut = k
                    break
                if p_ in endpts:
                    cut = k + 1
                    break
            if cut:
                gone = {id(sg) for (sg, _t, _p) in chain[:cut]}
                segs = [x for x in segs if id(x) not in gone]
        self.state[nm] = (segs, vias)

    # -- probes ------------------------------------------------------
    DETOUR = (1.6, 4.0)           # a lane may run at most k*chord + c mm

    def try_hard(self, nm, skip=(), capture=None):
        (a, al, b, bl) = self.ends[nm]
        b, alts = self.tgt.get(nm, (b, self.b_alts.get(nm)))
        # the net's OWN copper stays on the board: its fanout stubs and
        # vias are what the search lands on and what the same-net via
        # rule keeps a new via clear of (a via 0.14 mm from its own
        # fanout via, K51 SBA1, was placed with that copper absent)
        # the PLANNED lane as a tube first (0.5, 1.0, 2.0 mm), the free
        # window last: a re-lay confined to its plan cannot take another
        # net's landing (rungs counted in self.rungs)
        wp = self.window_pts.get(nm) or [a, b]
        board = self.board(skip=set(skip))
        res = None
        big = big_cfg(self.cfg)
        for hw in (0.5, 1.0, 2.0):
            with contextlib.redirect_stdout(io.StringIO()):
                res = cn.connect(board, self.nid[nm], a, al, b, bl, big,
                                 band=cn.tube_band(wp, hw), margin=1.0,
                                 window_pts=wp, b_alts=alts)
            if res is not None:
                self.rungs[f'tube{hw}'] += 1
                break
        if res is None:
            res = hard_route(board, self.nid[nm], a, al, b, bl, self.cfg,
                             window_pts=wp, b_alts=alts, capture=capture)
            if res is not None:
                self.rungs['free'] += 1
        if res is not None:
            # a band-free search with a wide window will wrap a whole
            # array to complete (K41 SA3 re-laid round the destination
            # after five rips): that is not a lane, and it steals the
            # room of every net it circles. Refused instead.
            ln = sum(((x.end_x - x.start_x) ** 2 + (x.end_y - x.start_y) ** 2) ** 0.5
                     for x in res[0])
            chord = ts.d2(a, b) ** 0.5
            k, c = self.DETOUR
            if ln > k * chord + c:
                self.log(f'      {nm}: {ln:.1f} mm for a {chord:.1f} mm chord '
                         f'-- detour refused')
                return None
        return res

    def classify(self, nm):
        """One refused net's story: ('MISSED', path) / ('WALLED',
        blockers) / ('BLOCKED', path, blockers) / ('NO_PATH', None)."""
        res = self.try_hard(nm)
        if res is not None:
            return 'MISSED', res, None
        cap = []
        (a, al, b, bl) = self.ends[nm]
        b, _alts = self.tgt.get(nm, (b, None))
        base = self.base_board()
        r0 = hard_route(base, self.nid[nm], a, al, b, bl, self.cfg,
                        window_pts=self.window_pts.get(nm),
                        b_alts=self.b_alts.get(nm), capture=cap)
        if r0 is None:
            return 'WALLED', None, stuck_blockers(cap[0] if cap else '')
        lanes = {om: self.lane(om) for om in self.state
                 if om != nm and self.routed(om)}
        path, bl_, _sb = soft_probe(base, self.nid[nm], a, al, b, bl, self.cfg,
                                    lanes, window_pts=self.window_pts.get(nm),
                                    b_alts=self.b_alts.get(nm))
        if path is None:
            return 'NO_PATH', r0, None
        return 'BLOCKED', path, bl_

    def stub_probe(self, nm):
        """The probe with the other nets' STUBS priced as well (their
        lanes too): (path, lane blockers, stub blockers) or Nones."""
        (a, al, b, bl) = self.ends[nm]
        b, _alts = self.tgt.get(nm, (b, None))
        base = self.base_board(bare_others=True, keep={nm})
        lanes = {om: self.lane(om) for om in self.state
                 if om != nm and self.routed(om)}
        stubs = {om: self.fo[om] for om in self.state if om != nm}
        return soft_probe(base, self.nid[nm], a, al, b, bl, self.cfg, lanes,
                          window_pts=self.window_pts.get(nm),
                          b_alts=self.b_alts.get(nm), stubs=stubs)

    # -- the rip-up --------------------------------------------------
    def negotiate(self, nm, max_victims=4, depth=2, _protect=frozenset()):
        """Blocker-directed rip-up for `nm`. Mutates self.state only on
        an accepted improvement; returns (accepted: bool, story: str)."""
        log = self.log
        before = (-self.n_routed(), self.n_vias())
        kind, path, blk = self.classify(nm)
        if kind == 'MISSED':
            self.lay(nm, path)
            log(f'    negotiate {nm}: MISSED -> routed ({len(path[1])} via)')
            return True, 'missed'
        if kind == 'WALLED':
            log(f'    negotiate {nm}: WALLED by static copper '
                f'{dict(blk) if blk else "(unnamed)"} -- a fanout matter')
            if self.reberth:
                return self.negotiate_stubs(nm, before, _protect)
            return False, 'walled'
        if kind == 'NO_PATH':
            log(f'    negotiate {nm}: no soft path found')
            return False, 'no_path'
        victims = [v for v, _ in sorted(
            blk.items(), key=lambda kv: -kv[1] / (1.0 + self.history[kv[0]]))
            if v not in _protect and v not in self.protected]
        log(f'    negotiate {nm}: BLOCKED by {dict(blk)}'
            + (f' (protected: {sorted(set(blk) & (_protect | self.protected))})'
               if set(blk) & (_protect | self.protected) else ''))
        if not victims:
            return False, 'all_protected'
        saved = {k: v for k, v in self.state.items()}
        best = None
        ks = list(range(1, min(len(victims), max_victims) + 1))
        if len(victims) > max_victims:
            # the crossing set is SUFFICIENT by construction (rip it
            # all and the soft path is a legal path), so a deeply
            # buried lane gets one try at the whole set after the
            # capped ladder
            ks.append(len(victims))
        for k in ks:
            V = victims[:k]
            self.state = dict(saved)
            saved_tgt = dict(self.tgt)
            for om in V:
                self.rip(om)
            res = self.try_hard(nm)
            if res is None:
                log(f'      rip {V}: {nm} still refused')
                self.tgt = saved_tgt
                continue
            self.lay(nm, res)
            fails = []
            for om in sorted(V, key=lambda o: -len(saved[o][1])):
                r2 = self.try_hard(om)
                if r2 is None:
                    fails.append(om)
                else:
                    self.lay(om, r2)
            if fails and depth > 1:
                for om in list(fails):
                    ok, _ = self.negotiate(om, max_victims=max_victims,
                                           depth=depth - 1,
                                           _protect=_protect | {nm})
                    if ok:
                        fails.remove(om)
            after = (-self.n_routed(), self.n_vias())
            log(f'      rip {V}: {nm} routed ({len(res[1])} via); '
                f're-laid {[o for o in V if o not in fails]}'
                + (f', still open {fails}' if fails else '')
                + f'  -> routed {-after[0]} vias {after[1]}')
            for om in V:
                self.history[om] += 1
            if after < before and (best is None or after < best[0]):
                best = (after, dict(self.state), dict(self.tgt))
            self.tgt = saved_tgt
            if not fails:
                break
        if best is not None:
            self.state, self.tgt = best[1], best[2]
            return True, 'ripped'
        self.state = saved
        if self.reberth:
            return self.negotiate_stubs(nm, before, _protect)
        return False, 'contested'

    def negotiate_stubs(self, nm, before, _protect=frozenset(), max_victims=2,
                        max_cands=6):
        """The berth rip-up: name the STUBS in the way (stub-priced
        probe), and for each (worst first) try its berth menu: relay,
        lay `nm`, re-lay the victim to its new tip, keep a strict
        improvement."""
        import copy as _copy
        log = self.log
        path, lb, sb = self.stub_probe(nm)
        if path is None or not sb:
            log(f'    reberth {nm}: no stub in the way'
                + (f' (lanes {dict(lb)})' if lb else ''))
            return False, 'no_stub'
        victims = [v for v, _ in sorted(sb.items(), key=lambda kv: -kv[1])
                   if v not in _protect and v not in self.protected][:max_victims]
        log(f'    reberth {nm}: stubs in the way {dict(sb)}'
            + (f', lanes {dict(lb)}' if lb else ''))
        stem = os.path.splitext(self.board_path)[0] + f'_neg_{nm}'
        for om in victims:
            saved = (dict(self.state), dict(self.fo), dict(self.ends),
                     dict(self.chains), dict(self.tgt), dict(self.b_alts),
                     dict(self.reberthed))
            for k, (label, largs) in enumerate(self.relay_candidates(om)[:max_cands]):
                key = self.try_reberth(om, largs, stem + f'_{om}_{label}')
                if key is None:
                    continue
                res = self.try_hard(nm)
                story = f'{om} -> {label} {key}: '
                if res is None:
                    story += f'{nm} still refused'
                else:
                    self.lay(nm, res)
                    r2 = self.try_hard(om)
                    if r2 is not None:
                        self.lay(om, r2)
                    else:
                        # the mover took the freed row: the victim may
                        # still land by ripping LANES of its own, the
                        # mover protected
                        ok2, _ = self.negotiate(om, max_victims=3, depth=1,
                                                _protect=_protect | {nm})
                        r2 = True if ok2 else None
                    after = (-self.n_routed(), self.n_vias())
                    story += (f'{nm} routed ({len(res[1])} via), {om} '
                              + ('re-laid' if r2 is not None else 'OPEN')
                              + f' -> routed {-after[0]} vias {after[1]}')
                    if after < before:
                        log(f'      reberth {story}  ACCEPTED')
                        self.history[om] += 1
                        return True, 'reberthed'
                log(f'      reberth {story}')
                (self.state, self.fo, self.ends, self.chains, self.tgt,
                 self.b_alts, self.reberthed) = [dict(x) for x in saved]
        return False, 'reberth_failed'

    def run(self, refused, rounds=3, max_victims=4):
        """Negotiate every refused net, repeatedly, until a round makes
        no progress. Returns the names still open."""
        open_ = list(refused)
        for rnd in range(rounds):
            progress = False
            for nm in list(open_):
                if self.routed(nm):
                    open_.remove(nm)
                    continue
                ok, story = self.negotiate(nm, max_victims=max_victims)
                if ok:
                    progress = True
                open_ = [o for o in self.state if not self.routed(o)]
            self.log(f'  negotiate round {rnd}: {len(open_)} open '
                     f'{sorted(open_)}, {self.n_vias()} vias; rungs {dict(self.rungs)}')
            if not open_ or not progress:
                break
        return open_

