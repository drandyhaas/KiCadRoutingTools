#!/usr/bin/env python3
"""Print the first K nets of the COHERENT K-ladder (k_ladder_coherent.txt).

That file's own header is the point: a prefix K must never split a
river. Taking "the first K nets by launch y" instead mixes singletons
into small K and measures a harder problem than the campaign's -- at
K=8 the braid emits 593 DRC violations on the launch-y prefix and 0 on
this one."""
import os
import re
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb')


def net_keys(src):
    """{net name: the whole route's name for the net} for every named net of `src` (a board, or its by_key map): the
    name's last path part -- the sheet path off, as a ladder, a plan and a log name a bus's nets -- where no other net's
    name ends in that part, else the name whole (ENABLE and TEST/ENABLE: each by its full name, or the one is fanned
    out, graded and laid for the other)"""
    if not hasattr(src, 'nets'):
        return {n.name: k for k, (_i, n) in src.items() if n.name}
    from net_queries import short_net_names
    return short_net_names(src)


_BLOCK_NET = re.compile(r'\(net "((?:[^"\\]|\\.)*)"\)')


def block_net_name(block):
    """The net NAME a board file's block -- a segment's, a via's -- carries, as the parser names the net: UNESCAPED
    (kicad_parser's rule: a backslash and a quote are spelled escaped in the file). None where the block carries a
    net's number, or none. Compared as the file spells it, a net with a backslash in its name matched nothing: a
    merge left its copper of the shared field in place and took none of what its bus had routed -- one bus net open
    and shorted on a board whose bus had routed it clean"""
    m = _BLOCK_NET.search(block)
    return m.group(1).replace('\\\\', '\\').replace('\\"', '"') if m else None


def key_of(name, keys=None):
    """A net's name, as a checker's log or a pad gives it, by the whole route's name for it: `keys` (net_keys) when
    given, else -- or for a name not among them -- its last path part"""
    return keys[name] if keys and name in keys else name.split('/')[-1]


def namer(src):
    """key_of over the nets of `src` -- a board, or its by_key map -- as a function: a net name, full or already the
    whole route's, by the whole route's name for it"""
    keys = net_keys(src)
    return lambda name: key_of(name, keys)


def by_key(pcb):
    """{the whole route's name (net_keys): (net id, net)} for every net of `pcb` (the unnamed by '')"""
    keys = net_keys(pcb)
    return {keys.get(n.name, n.name): (i, n) for i, n in pcb.nets.items()}


def net_patterns(names, keys=None):
    """The checkers' --nets patterns (fnmatchcase) for the nets `names`: each net as named and under any sheet path
    (NAME, */NAME) -- not *NAME, which takes every net whose name merely ENDS so (a bus net CK took SCK and ADC_CK: an
    open one failed the bus's connectivity, its violations counted as the bus's) -- and a net the board names by its
    full name (`keys`, net_keys) as named only. A glob character in a name is bracketed, so it matches only itself
    (DATA[0])."""
    whole = {k for n, k in (keys or {}).items() if k == n}
    out = []
    for n in names:
        q = re.sub(r'([*?\[])', r'[\1]', n)
        out += [q] if '/' in n or n in whole else [q, '*/' + q]
    return out


def engine_filter(names, pcb):
    """The fanout engine's net_filter for the nets `names` (full names or the whole route's, net_keys) on `pcb`: each
    net by its full name, a glob character in it bracketed -- the engine takes a name that is a net's own as that net
    alone (net_queries.named_net)"""
    from net_queries import literal_net_pattern
    keys = net_keys(pcb)
    full = {k: n for n, k in keys.items()}
    return [literal_net_pattern(n if n in keys else full.get(n, n)) for n in names]


def ladder_of(board=BENCH):
    """The ladder file for a bench board: `<board stem>.ladder.txt` beside
    it when there is one (a second array pair carries its own rivers),
    else the bench's `k_ladder_coherent.txt`."""
    side = os.path.splitext(board)[0] + '.ladder.txt'
    return side if os.path.exists(side) else os.path.join(HERE, 'k_ladder_coherent.txt')


def _rivers(board=BENCH):
    rivers = []
    for line in open(ladder_of(board)):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        rivers.append(line.split())
    return rivers


def admissible(pcb, net):
    """A net the bus router can route ball to ball: two pads on two parts
    -- or more, when every extra pad is SERVED UNDER one of the two by a
    via-in-pad (pairs.under_pad: a back-side termination resistor under a
    DDR clock ball) or is a pad of a two-pad part whose other pad is on the
    PARTNER leg of a differential pair (the pair's termination, which the
    pair passes through: pairs.pair_waypoints -- the zynq's R20 on CK).
    make_bench selects the bench's nets by it and coherent_nets keeps them,
    so a net the bench fans out is one the ladder admits."""
    import pairs as _pairs
    import rules as _rules
    import awx_settings
    # (a pull pad the bus step's escape fanout joined to the lane -- BUS_ATTACHED, route_bus.attach_pulls -- is none
    # of the net's ends: its stub stands on the board, connected when the lane is)
    attached = {h for h in (awx_settings.get('BUS_ATTACHED') or '').split(',') if h}
    keys = net_keys(pcb)
    prs = _pairs.pair_names(list(set(keys.values())), admit_all=True)   # admission ignores BRAID_PAIR_ONLY
    partner = {}
    for _b, (_pn, _nn) in prs.items():
        partner[_pn], partner[_nn] = _nn, _pn
    pads = [p for p in net.pads if f'{p.component_ref}.{p.pad_number}' not in attached]
    if len({p.component_ref for p in pads}) < 2:
        return False
    if len(pads) == 2:
        return True
    mate = partner.get(keys.get(net.name, net.name))

    def waypoint(p):
        fp = pcb.footprints.get(p.component_ref)
        if fp is None or mate is None or len(fp.pads) != 2:
            return False
        other = [q for q in fp.pads if q is not p][0]
        return keys.get(pcb.nets[other.net_id].name) == mate if other.net_id in pcb.nets else False
    via_size = _rules.active().via_size
    ends = [p for p in pads if not waypoint(p)
            and not any(_pairs.under_pad(q, p, via_size) for q in pads if q is not p)]
    return len(ends) == 2 and len({p.component_ref for p in ends}) == 2


def coherent_nets(K, board=BENCH, bare_ok=None):
    """The first K routable nets of the coherent ladder: two-pad nets
    between two components that are fanned out on `board` (a free
    stub end exists at the source) -- or, with `bare_ok` (default: the
    escape fanout, FANOUT_ESCAPE), bare at both (braid.bare_lane: the
    escape fanout lays both of its ends from the pads)."""
    if bare_ok is None:
        sys.path.insert(0, HERE)
        import awx_settings
        bare_ok = awx_settings.get('FANOUT_ESCAPE') == '1'
    flat = [n for r in _rivers(board) for n in r]
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    sys.path.insert(0, HERE)
    import contextlib
    from kicad_parser import parse_kicad_pcb
    import braid as te
    # the parser prints board warnings (net-tagged graphics, duplicate
    # references) on STDOUT; this CLI's stdout is the net list a chain
    # captures, so those go to stderr here
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    bn = by_key(pcb)
    by = {k: n for k, (_i, n) in bn.items()}
    two_ended = lambda net: admissible(pcb, net)
    flat = [n for n in flat if n in by and two_ended(by[n])]
    ok = []
    for n in flat:
        try:
            te.endpoints(pcb, [n], bn)
            ok.append(n)
        except AssertionError:
            if bare_ok and te.bare_lane(pcb, n, bn):
                ok.append(n)
    # A SHORT COUNT IS A DIFFERENT PROBLEM, and it was silent: "K51" is
    # 48 nets here, so the human's 81 (48 nets) and 85 (51) are both
    # right and both were in circulation for one label. Pointing --board
    # at a ROUTED board is worse: a routed net has no free stub end, so
    # K41 returned the 7 UNROUTED nets instead of the 41.
    if len(ok) < K:
        print(f'  coherent_nets: asked K={K}, {len(ok)} net(s) qualify on '
              f'{os.path.basename(board)} -- this is a {len(ok)}-net problem',
              file=sys.stderr)
    return whole_pairs(ok, K, board)


def whole_pairs(ok, K, board=''):
    """The first K of `ok` with no differential pair CUT: a leg whose
    partner qualifies but falls beyond K is left out with it, so the
    prefix is a little shorter than K rather than a problem with a pair
    run as one leg alone (the zynq LVDS bus's escape ladder at K=40 took
    TX_D3_P, TX_D4_P and TX_D5_P without their N legs). Said on stderr."""
    import pairs as _pairs
    mate = {}
    for p_, n_ in _pairs.pair_names(ok, admit_all=True).values():
        mate[p_], mate[n_] = n_, p_
    head = ok[:K]
    have = set(head)
    cut = [n for n in head if n in mate and mate[n] not in have]
    if cut:
        print(f'  coherent_nets: K={K} on {os.path.basename(board)} would '
              f'cut pair(s) {", ".join(f"{n}/{mate[n]}" for n in cut)} -- '
              f'their leg(s) left out: a {len(head) - len(cut)}-net problem',
              file=sys.stderr)
    return [n for n in head if n not in cut]


if __name__ == '__main__':
    board = next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--board=')), BENCH)
    rivers = _rivers(board)
    flat = [n for r in rivers for n in r]
    if '--checkpoints' in sys.argv:
        tot = 0
        out = []
        for r in rivers:
            tot += len(r)
            out.append(str(tot))
        print(' '.join(out))
        sys.exit(0)
    K = int(sys.argv[1]) if len(sys.argv) > 1 else 51
    print(','.join(flat[:K] if '--raw' in sys.argv else coherent_nets(K, board)))
