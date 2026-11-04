"""Repeat one placed hierarchical-sheet channel onto its sibling instances
(#1211) -- the engine half of `place_pose replicate`.

A board drawn from one hierarchical sheet instantiated N times (an audio
mixer's channel strips, one I/O buffer per bank) already carries the
correspondence between the copies: every footprint's `(path "/<sheet>/
<symbol>")` ends in the SAME symbol uuid in every instance, and KiCad's own
"Repeat Layout" maps by it. Nothing in the placement stack used it --
`arrays` (#1051) rows identical parts on ONE sheet -- so an agent that wanted
four channels laid out alike wrote its own helper.

This module

1. resolves the sheets a caller named, by sheet NAME (what KiCad shows) or by
   path prefix (`/<uuid>`), refusing an unknown or ambiguous one and listing
   the candidates;
2. maps source -> target footprints by the path BELOW the instance prefix:
   the symbol uuid, plus the uuids of any nested sub-sheets, which every
   instance shares because the sub-sheet symbol lives in the shared file;
3. refuses unless the copy is a pure relabelling -- the same parts, the same
   footprints and pads, and a consistent net correspondence (THE NET RULE);
4. returns the target poses: the source's poses translated by one offset,
   same rotation, same side.

Channels drawn as copies INSIDE one sheet (three line drivers, an I/Q pair)
share no instance prefix, so nothing pairs them by path. For those the
caller names the correspondence (#1480): `channels`, ordered lists of refs,
the first the source and each other a target matched to it index for index
(`channel_members`). Every check below runs on them unchanged -- the same
parts, footprints and pads, THE NET RULE, one translation -- with each
channel standing in for a sheet instance. `suggest_channels` proposes them.

It moves nothing and grades nothing. `pose_ops.resolve_ops` turns its rows
into the writer's placements, and `apply_poses` grades them with the verdict
every other `place_pose` verb gets -- the AI declares the structure, the
tools do the geometry (#1051).

THE NET RULE. Instances of one sheet differ in their LOCAL nets (`/CH1/IN`
vs `/CH2/IN`) and share their GLOBAL ones (GND, the rails), so equal names
cannot be asked for. What is asked for, over every mapped pad pair, is

* a function: a source net reaches ONE target net;
* injective: a target net is reached from ONE source net;
* a net carried by pads of BOTH instances (a rail, a bus both channels sit
  on) maps to itself;
* no net maps to no net.

The first two are judged against ONE net map, chosen over all the pad pairs
at once so that the most of them agree (`net_map`, #1277): which pads break
it is then the fewest that can, and a strap names itself however early its
pad comes in pair order.

A pad pair that breaks one is named -- refs, pads and nets -- and the copy is
refused: the two instances are not interchangeable there, so a geometric
copy would seat a target pad where the source's DIFFERENT connection wanted
it. The refusal spells the waiver for each such pad: `--allow-net-mismatch
REF.PAD` (one pad per flag, never a blanket switch) leaves a pad wired
differently BY DESIGN -- glasgow's per-instance I2C address straps -- out of
the rule, and the summary reports every waived pad and whether the rule
needed it.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Set, Tuple

from placement.arrays import natural_key
from placement.groups import _sheet_of
from placement.pose_ops import PoseRefusal

#: How many offending items a refusal sentence names before it summarises.
#: The summary carries every one (up to `LISTED`).
NAMED = 6
LISTED = 200

#: The shortest leading part of a sheet path accepted in place of the whole.
MIN_PATH_START = 4

#: Equal poses, for "did this part move". The writer's resolution is far
#: finer; this is the tolerance `resolve_ops` compares notes at (4 places).
POSE_EPS = 1e-4


def norm_name(text: str) -> str:
    """A sheet name or path with its slashes trimmed: KiCad 9 spells the
    sheet `/CH1/`, KiCad 7 `CH1`, and a person types either."""
    return (text or '').strip().strip('/').strip()


def _leaf(name: str) -> str:
    return name.rsplit('/', 1)[-1]


def sheet_instances(pcb) -> Dict[str, Dict]:
    """`{prefix: {'names': [...], 'files': [...], 'refs': [...]}}` for every
    NON-ROOT sheet prefix on the board, ancestors of nested sheets included.

    `refs` are every footprint at or below the prefix; `names` and `files`
    are the sheet names and sheet files the prefix's OWN parts declare
    (`Footprint.sheet_name` / `sheet_file`, which both parse paths fill). A
    sheet holding only sub-sheets has none -- it is still reachable by
    path."""
    out: Dict[str, Dict] = {}
    for ref, fp in pcb.footprints.items():
        prefix = _sheet_of(fp)
        if not prefix:
            continue
        parts = prefix.split('/')
        for i in range(1, len(parts) + 1):
            row = out.setdefault('/'.join(parts[:i]),
                                 {'names': set(), 'files': set(), 'refs': []})
            row['refs'].append(ref)
        name = norm_name(getattr(fp, 'sheet_name', '') or '')
        if name:
            out[prefix]['names'].add(name)
        sfile = (getattr(fp, 'sheet_file', '') or '').strip()
        if sfile:
            out[prefix]['files'].add(sfile)
    for row in out.values():
        row['names'] = sorted(row['names'])
        row['files'] = sorted(row['files'])
        row['refs'] = sorted(row['refs'], key=natural_key)
    return out


def describe(prefix: str, row: Dict) -> Dict:
    """One sheet instance as a refusal or a summary names it."""
    return {'name': row['names'][0] if row['names'] else None,
            'path': '/' + prefix,
            'file': row['files'][0] if row.get('files') else None,
            'parts': len(row['refs']),
            'example': row['refs'][:3]}


def _phrase(d: Dict) -> str:
    return '%s (%s, %d parts: %s)' % (
        d['name'] or '<unnamed>', d['path'], d['parts'],
        ', '.join(d['example']) + (', ...' if d['parts'] > 3 else ''))


def resolve_sheet(pcb, token: str, role: str,
                  instances: Optional[Dict[str, Dict]] = None) -> str:
    """The sheet prefix `token` names, or a refusal (exit 2) listing the
    candidates.

    Tried in order, the first rung with exactly one answer wins and a rung
    with several is AMBIGUOUS (never resolved by a later rung): the sheet
    name, whole or by its last component (`CH1`, `/CH1/`, or `sub` for
    `/CH1/sub/`); then the same case-insensitively; then the path prefix,
    whole or by a unique leading part of it (`/28f12a69`) at least
    `MIN_PATH_START` characters long, so a short mistyped NAME cannot land
    on whichever uuid happens to start with it."""
    inst = instances if instances is not None else sheet_instances(pcb)
    listing = sorted((describe(p, r) for p, r in inst.items()),
                     key=lambda d: (d['name'] is None, d['name'] or '',
                                    d['path']))

    def _refuse(why, matches=None):
        shown = [describe(p, inst[p]) for p in matches] if matches else listing
        raise PoseRefusal(
            '%s %r %s. %s: %s' % (
                role, token, why,
                'It matches' if matches else
                ('Sheet instances on this board' if listing else
                 'This board has no hierarchical sheet instances'),
                '; '.join(_phrase(d) for d in shown[:12])
                + ('; +%d more' % (len(shown) - 12) if len(shown) > 12
                   else '')),
            code=2, sheets=shown[:LISTED])

    q = norm_name(token)
    if not q:
        _refuse('names the ROOT sheet, which is not an instance of a '
                'hierarchical sheet; name one of its sheets')
    for fold in (False, True):
        key = q.casefold() if fold else q

        def _eq(name):
            name = name.casefold() if fold else name
            return key == name or key == _leaf(name)
        hits = sorted(p for p, r in inst.items() if any(_eq(n) for n in r['names']))
        if len(hits) == 1:
            return hits[0]
        if hits:
            _refuse('is ambiguous: %d sheet instances carry that name. Name '
                    'one by its path' % len(hits), hits)
    path = q.lower()
    if path in inst:
        return path
    hits = (sorted(p for p in inst if p.lower().startswith(path))
            if len(path) >= MIN_PATH_START else [])
    if len(hits) == 1:
        return hits[0]
    if hits:
        _refuse('is ambiguous: it is the start of %d sheet paths' % len(hits),
                hits)
    _refuse('is neither a sheet name nor a sheet path on this board')
    raise AssertionError('unreachable')                    # pragma: no cover


def members(pcb, prefix: str) -> Dict[str, str]:
    """`{path below the instance: ref}` for every footprint at or below
    `prefix`. Two footprints claiming one path (a duplicated block) is
    refused: the correspondence would have to pick one."""
    depth = len(prefix.split('/'))
    out: Dict[str, str] = {}
    dup: Dict[str, List[str]] = {}
    for ref in sorted(pcb.footprints, key=natural_key):
        fp = pcb.footprints[ref]
        sheet = _sheet_of(fp)
        if sheet != prefix and not sheet.startswith(prefix + '/'):
            continue
        parts = [q for q in (fp.sheet_path or '').split('/') if q]
        key = '/'.join(parts[depth:])
        if key in out:
            dup.setdefault(key, [out[key]]).append(ref)
        else:
            out[key] = ref
    if dup:
        raise PoseRefusal(
            'sheet /%s has footprints that claim the same schematic symbol '
            '(%s), so a copy cannot tell which one a counterpart maps to'
            % (prefix, '; '.join('/'.join(v) for v in list(dup.values())[:NAMED])),
            duplicates=sorted(dup.values()))
    return out


#: The prefix of a channel named by its refs (#1480), standing in for a
#: sheet instance's path in every message and report.
CHANNEL_PREFIX = 'channel:'


def channel_members(pcb, channels: Sequence[Sequence[str]]
                    ) -> Tuple[str, List[str], Dict[str, Dict[str, str]],
                               Dict[str, Dict]]:
    """`(source, targets, members, instances)` for channels named by their
    refs (#1480): each channel a pseudo-instance `channel:<first ref>`, its
    members keyed by POSITION, so the parts at one index correspond. Refused
    (exit 2) unless there are two or more channels, each the same length,
    every ref on the board and in one channel only."""
    chans = [[str(r) for r in c] for c in (channels or ())]
    if len(chans) < 2:
        raise PoseRefusal('channels: name two or more channels (the first is '
                          'the source, the others copies of it)', code=2)
    if len({len(c) for c in chans}) != 1 or not chans[0]:
        raise PoseRefusal('channels: every channel lists its parts in the same '
                          'order, so they must be the same length; got %s' % (
                              ', '.join(str(len(c)) for c in chans)), code=2)
    fps = pcb.footprints
    missing = sorted({r for c in chans for r in c if r not in fps}, key=natural_key)
    if missing:
        raise PoseRefusal('channels: not on this board: %s' % ', '.join(
            missing[:NAMED * 2]), code=2, missing=missing)
    seen: Dict[str, int] = {}
    twice = []
    for i, c in enumerate(chans):
        for r in c:
            if r in seen and r not in twice:
                twice.append(r)
            seen.setdefault(r, i)
    if twice:
        raise PoseRefusal('channels: a part is in one channel only; named more '
                          'than once: %s' % ', '.join(twice[:NAMED * 2]), code=2)
    names = [CHANNEL_PREFIX + c[0] for c in chans]
    mem = {n: {str(k): r for k, r in enumerate(c)} for n, c in zip(names, chans)}
    inst = {n: {'names': [n], 'files': [], 'refs': sorted(c, key=natural_key)}
            for n, c in zip(names, chans)}
    return names[0], names[1:], mem, inst


def _mem(pcb, prefix: str, chan=None) -> Dict[str, str]:
    """A sheet instance's members (`members`), or a named channel's."""
    if chan is not None and prefix in chan:
        return chan[prefix]
    return members(pcb, prefix)


def net_map(votes: Dict[Tuple[int, int], int],
            rank: Dict[Tuple[int, int], int]) -> List[Tuple[int, int]]:
    """The source -> target net map THE NET RULE judges pads against (#1277):
    `[(source net, target net), ...]`, one to one.

    `votes[(a, b)]` is how many mapped pad pairs wire source net `a` where
    the target wires `b`, and `rank[(a, b)]` the position of the first of
    them in pair order. The map is the one under which the MOST pad pairs
    agree -- a maximum-weight matching of the votes, so the fewest pads are
    named -- and among maps that tie, the one whose pairings are met first
    (taken in rank order, each kept when an optimal map still includes it).

    A map grown first-come instead let one strap claim its net: R2.1
    strapped to /S2/OUT, met before the two pairs wiring /S1/IN to /S2/IN
    alike, mapped /S1/IN to /S2/OUT and named the pads that ARE alike. A
    per-net majority would need tie-breaks across the sides, because a
    net's pairs can split evenly (there, /S1/IN's split 1 to 1 and only
    /S1/OUT's two pairs on /S2/OUT settle it); the matching weighs both
    sides at once, and its one tie-break is the order the pairs are met.

    Only a connected group of COMPETING pairings is solved, so a map with
    no strap -- every net wired to one net -- is the votes as they stand,
    and no solver runs."""
    parent: Dict[Tuple[str, int], Tuple[str, int]] = {}

    def _root(n):
        while parent.setdefault(n, n) != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    edges = sorted(votes, key=lambda e: rank[e])
    for a, b in edges:
        parent[_root(('s', a))] = _root(('t', b))
    groups: Dict[Tuple[str, int], List[Tuple[int, int]]] = {}
    for e in edges:
        groups.setdefault(_root(('s', e[0])), []).append(e)
    out: List[Tuple[int, int]] = []
    for group in groups.values():
        out += group if len(group) == 1 else _first_best(group, votes)
    return out


def _first_best(edges: List[Tuple[int, int]],
                votes: Dict[Tuple[int, int], int]) -> List[Tuple[int, int]]:
    """`net_map` on one group of competing pairings, `edges` in rank order:
    of the maximum-weight matchings, the one that takes the earliest edges."""
    import numpy as np
    from scipy.optimize import linear_sum_assignment

    def _best(es):
        if not es:
            return 0
        src = {a: i for i, a in enumerate(sorted({a for a, _b in es}))}
        dst = {b: i for i, b in enumerate(sorted({b for _a, b in es}))}
        w = np.zeros((len(src), len(dst)))
        for a, b in es:
            w[src[a], dst[b]] = votes[(a, b)]
        r, c = linear_sum_assignment(w, maximize=True)
        return int(round(float(w[r, c].sum())))

    target = _best(edges)
    live = list(edges)
    kept: List[Tuple[int, int]] = []
    got = 0
    for e in edges:
        if e not in live:
            continue
        rest = [f for f in live if f[0] != e[0] and f[1] != e[1]]
        if got + votes[e] + _best(rest) == target:
            kept.append(e)
            got += votes[e]
            live = rest
        else:
            live.remove(e)
    return kept


def correspond(pcb, src: str, dst: str, *,
               waive: Optional[Dict[Tuple[str, str], str]] = None,
               keys: Optional[Set[str]] = None, chan=None
               ) -> Tuple[List[Tuple[str, str]], Dict]:
    """`([(source ref, target ref), ...], nets)`, or a refusal (exit 4).

    Refused when the instances do not hold the same parts, when a mapped
    pair differs in footprint or pad list, or when a mapped pad pair breaks
    THE NET RULE (module docstring). Pads are paired by INDEX, after the pad
    numbers have been checked equal index for index: a pad number is not
    unique within a footprint (#892), and one library footprint writes its
    pads in one order.

    `waive` (`--allow-net-mismatch`): `{(symbol key, pad number): as given}`
    -- pads wired differently BY DESIGN (glasgow's I2C address straps), left
    out of the net rule entirely: they neither break it, nor make a net look
    shared, nor vote on the net map. Each is reported back with whether the
    rule needed it
    (`nets['waived']`). A refusal names the pads it would take, as flags.

    `keys` (`--only` / `--skip`): the symbol keys this copy MOVES; None is
    every one. The three checks run over those alone -- a part left out
    needs no counterpart, may differ in footprint, and cannot break the net
    rule -- but a net is SHARED when any pad of either whole instance
    carries it (the waived pads aside), so leaving a part out never turns a
    rail into a channel-local net."""
    waive = waive or {}
    fps = pcb.footprints
    sm, tm = _mem(pcb, src, chan), _mem(pcb, dst, chan)
    sel = set(sm) | set(tm) if keys is None else set(keys)
    only_s = sorted((sm[k] for k in sm.keys() - tm.keys() if k in sel),
                    key=natural_key)
    only_t = sorted((tm[k] for k in tm.keys() - sm.keys() if k in sel),
                    key=natural_key)
    if only_s or only_t:
        raise PoseRefusal(
            'the two sheet instances do not hold the same parts (matched by '
            'schematic symbol): only in /%s: %s; only in /%s: %s. A copy '
            'needs a counterpart for every part it moves (--skip leaves '
            'named parts out)' % (
                src, ', '.join(only_s[:NAMED * 2]) or 'none',
                dst, ', '.join(only_t[:NAMED * 2]) or 'none'),
            only_in_source=only_s, only_in_target=only_t)
    pairs = sorted(((sm[k], tm[k]) for k in sm if k in tm and k in sel),
                   key=lambda p: natural_key(p[0]))

    differ = []
    for s, t in pairs:
        fs, ft = fps[s], fps[t]
        if fs.footprint_name != ft.footprint_name:
            differ.append('%s is %s but %s is %s' % (
                s, fs.footprint_name, t, ft.footprint_name))
        elif ([p.pad_number for p in fs.pads or ()]
              != [p.pad_number for p in ft.pads or ()]):
            differ.append('%s and %s are both %s but list different pads '
                          '(%d vs %d)' % (s, t, fs.footprint_name,
                                          len(fs.pads or ()), len(ft.pads or ())))
    if differ:
        raise PoseRefusal(
            'mapped parts differ, so the source arrangement does not fit the '
            'target: %s%s' % ('; '.join(differ[:NAMED]),
                              '; +%d more' % (len(differ) - NAMED)
                              if len(differ) > NAMED else ''),
            footprints_differ=differ[:LISTED])

    def _name(nid):
        net = (pcb.nets or {}).get(nid)
        return (getattr(net, 'name', '') or '') if nid else '<no net>'

    key_of = {sm[k]: k for k in sm}
    rows, waived_rows = [], []
    for s, t in pairs:
        for sp, tp in zip(fps[s].pads or (), fps[t].pads or ()):
            given = waive.get((key_of[s], sp.pad_number))
            (waived_rows if given is not None else rows).append(
                (s, sp, t, tp, given))

    def _instance_nets(mem):
        return {p.net_id for k, ref in mem.items()
                for p in fps[ref].pads or ()
                if (k, p.pad_number) not in waive}
    s_nets, t_nets = _instance_nets(sm), _instance_nets(tm)

    # The map, filled below. EMPTY while the votes are counted, so `_why`
    # there gives only the reasons a pad pair breaks under ANY map.
    fwd: Dict[int, Tuple[int, str]] = {}
    bwd: Dict[int, Tuple[int, str]] = {}

    def _at(s, sp, t, tp):
        return '%s.%s %s -> %s.%s %s' % (s, sp.pad_number, _name(sp.net_id),
                                         t, tp.pad_number, _name(tp.net_id))

    def _why(a, b):
        # The pair's own reasons first: they hold under any map (#1277).
        if (a == 0) != (b == 0):
            return 'one pad has no net and the other has one'
        if a != b and a in t_nets:
            return ('%s is on pads of BOTH instances (a shared net), so it '
                    'must map to itself' % _name(a))
        if a != b and b in s_nets:
            return ('%s is on pads of BOTH instances (a shared net), so only '
                    'itself may map to it' % _name(b))
        if a in fwd and fwd[a][0] != b:
            return '%s maps to %s (%s)' % (_name(a), _name(fwd[a][0]),
                                           fwd[a][1])
        if b in bwd and bwd[b][0] != a:
            return '%s is reached from %s (%s)' % (
                _name(b), _name(bwd[b][0]), bwd[b][1])
        return None

    # THE NET MAP (#1277) is chosen over ALL the pad pairs at once
    # (`net_map`), not grown first-come: a strap met first claimed its net
    # (R2.1 strapped to /S2/OUT mapped /S1/IN there), and the refusal named
    # the pads wired alike. A pair that breaks the rule under any map does
    # not vote. Each map entry cites the first pad pair that wires it.
    votes: Dict[Tuple[int, int], int] = {}
    first: Dict[Tuple[int, int], Tuple[int, str, str]] = {}
    for i, (s, sp, t, tp, _g) in enumerate(rows):
        a, b = sp.net_id, tp.net_id
        if _why(a, b) is None:
            votes[(a, b)] = votes.get((a, b), 0) + 1
            first.setdefault((a, b), (i, '%s.%s' % (s, sp.pad_number),
                                      '%s.%s' % (t, tp.pad_number)))
    for a, b in net_map(votes, {e: r[0] for e, r in first.items()}):
        fwd[a] = (b, first[(a, b)][1])
        bwd[b] = (a, first[(a, b)][2])

    breaks = []
    for s, sp, t, tp, _g in rows:
        why = _why(sp.net_id, tp.net_id)
        if why:
            breaks.append({'pads': _at(s, sp, t, tp), 'why': why,
                           'source_pad': '%s.%s' % (s, sp.pad_number),
                           'target_pad': '%s.%s' % (t, tp.pad_number)})
    if breaks:
        flags = []
        for b in breaks:
            if b['source_pad'] not in flags:
                flags.append(b['source_pad'])
        raise PoseRefusal(
            'the two sheet instances are not wired alike: %d mapped pad '
            'pair(s) break a consistent net correspondence (each source net '
            'must reach one target net, each target net come from one source '
            'net, and a net on both instances map to itself) -- %s%s. If '
            'those pads are wired differently BY DESIGN (an address strap, a '
            'per-channel ID), waive each one: %s' % (
                len(breaks),
                '; '.join('%s: %s' % (b['pads'], b['why'])
                          for b in breaks[:NAMED]),
                '; +%d more' % (len(breaks) - NAMED)
                if len(breaks) > NAMED else '',
                ' '.join('--allow-net-mismatch %s' % f
                         for f in flags[:NAMED * 2])
                + (' ... (%d in all, listed under suggest_waivers)'
                   % len(flags) if len(flags) > NAMED * 2 else '')),
            net_breaks=breaks[:LISTED], suggest_waivers=flags[:LISTED])
    # A waiver the rule did not need is reported, not refused: the pads are
    # wired alike after all, and the copy is the same either way.
    waived = [{'pads': _at(s, sp, t, tp), 'as_given': g,
               'needed': _why(sp.net_id, tp.net_id) is not None}
              for s, sp, t, tp, g in waived_rows]
    renamed = {_name(a): _name(b) for a, (b, _w) in sorted(fwd.items())
               if a != b}
    shared = sorted(_name(a) for a, (b, _w) in fwd.items() if a == b and a)
    return pairs, {'renamed': renamed, 'shared': shared, 'waived': waived}


def _anchor_rank(fp) -> Tuple:
    """Bigger first: most pads, then the widest pad-copper extent."""
    from kicad_parser import non_aperture_pads
    pads = non_aperture_pads(fp)
    if not pads:
        return (0, 0.0)
    xs = [p.global_x for p in pads]
    ys = [p.global_y for p in pads]
    return (len(pads), (max(xs) - min(xs)) * (max(ys) - min(ys)))


def _locate_ref(pcb, ref: str, src: str, dsts: Sequence[str],
                flag: str, chan=None) -> Tuple[str, List[str]]:
    """`(symbol key, [the target instances it applies to])` for a ref named
    on the SOURCE instance (every target) or on one TARGET instance (that
    one only), else a refusal (exit 2) listing where it looked."""
    for prefix in [src] + list(dsts):
        hit = [k for k, r in _mem(pcb, prefix, chan).items() if r == ref]
        if hit:
            return hit[0], (list(dsts) if prefix == src else [prefix])
    raise PoseRefusal(
        '%s %s: %s is in neither the --from sheet nor a --to sheet (%s)' % (
            flag, ref, ref, ', '.join('/' + p for p in [src] + list(dsts))),
        code=2)


def resolve_waivers(pcb, tokens: Sequence[str], src: str,
                    dsts: Sequence[str], chan=None
                    ) -> Dict[str, Dict[Tuple[str, str], str]]:
    """`--allow-net-mismatch REF.PAD ...` -> `{target prefix: {(symbol key,
    pad number): token}}`.

    REF is a part of the source instance (the pad is waived in every target)
    or of one target instance (that target only). The split is at the FIRST
    dot, since a reference has none and a pad number may (`1.1`). A pad
    number repeated within a footprint (#892) waives every pad carrying it.
    An unknown ref or pad is refused at exit 2: a waiver that names nothing
    would read as one that was granted."""
    out: Dict[str, Dict[Tuple[str, str], str]] = {d: {} for d in dsts}
    for tok in tokens or ():
        ref, dot, pad = str(tok).partition('.')
        if not dot or not ref or not pad:
            raise PoseRefusal(
                '--allow-net-mismatch %r: expected REF.PAD, one pad per flag '
                '(e.g. U19.2)' % tok, code=2)
        key, applies = _locate_ref(pcb, ref, src, dsts,
                                   '--allow-net-mismatch', chan)
        numbers = [p.pad_number for p in pcb.footprints[ref].pads or ()]
        if pad not in numbers:
            raise PoseRefusal(
                '--allow-net-mismatch %s: %s has no pad %r (its pads: %s)' % (
                    tok, ref, pad, ', '.join(sorted(set(numbers),
                                                    key=natural_key)[:24])),
                code=2)
        for d in applies:
            out[d][(key, pad)] = str(tok)
    return out


def resolve_selection(pcb, only: Sequence[str], skip: Sequence[str],
                      src: str, dsts: Sequence[str], chan=None
                      ) -> Dict[str, Optional[Set[str]]]:
    """`--only` / `--skip` -> `{target prefix: the symbol keys it copies}`,
    None meaning every one.

    A ref on the SOURCE instance names its symbol in every target; a ref on
    one TARGET instance names that part only -- so `--skip RV1` leaves every
    channel's counterpart of RV1 where it is, and `--skip RV9` only CH3's.
    The two flags are exclusive (exit 2: `--only A --skip B` has two
    readings), and a ref outside the named sheets is refused at exit 2."""
    if only and skip:
        raise PoseRefusal('replicate was given both --only and --skip; pass '
                          'one (the parts to copy, or the parts to leave)',
                          code=2)
    flag, refs = ('--only', only) if only else ('--skip', skip)
    if not refs:
        return {d: None for d in dsts}
    named: Dict[str, Set[str]] = {d: set() for d in dsts}
    for ref in refs:
        key, applies = _locate_ref(pcb, ref, src, dsts, flag, chan)
        for d in applies:
            named[d].add(key)
    if flag == '--only':
        return named
    sm = set(_mem(pcb, src, chan))
    return {d: (sm | set(_mem(pcb, d, chan))) - named[d] for d in dsts}


def plan(pcb, src_token: str, dst_tokens: Sequence[str], *, anchor: Optional[str] = None,
         offset: Optional[Sequence[float]] = None,
         allow_net_mismatch: Sequence[str] = (),
         only: Sequence[str] = (), skip: Sequence[str] = (),
         channels: Optional[Sequence[Sequence[str]]] = None
         ) -> Tuple[List[Dict], Dict]:
    """`(rows, report)` for one `replicate` op.

    `rows`: one per target part -- `{'source', 'ref', 'x', 'y', 'rot',
    'side', 'flip'}`, the pose it should take. `report`: what was mapped and
    how the offset was chosen, for the summary.

    The OFFSET: `offset` (dx, dy) when given -- one target only, the CLI
    refuses it with several; otherwise each target keeps its ANCHOR where
    it is, so a channel is re-arranged in place rather than moved. The
    anchor is `anchor` (a ref in the source sheet or in any target sheet,
    standing for its counterparts), else the source part with the most pads
    (then the widest pad extent, then the first ref): on a channel strip,
    the op-amp. The anchor keeps its (at x y); its rotation and side become
    the source's like every other part's.

    `allow_net_mismatch`: `REF.PAD` tokens left out of THE NET RULE
    (`resolve_waivers`, `correspond`).

    `only` / `skip`: a PARTIAL copy (`resolve_selection`) -- the parts left
    out stay where they are, and the checks run over the parts copied. The
    default anchor is then the largest source part copied into EVERY target
    (else present in every target); an explicit `anchor` may be one that is
    left out, and still fixes the offset.

    `channels` (#1480): the instances named by their refs instead
    (`channel_members`); `src_token` and `dst_tokens` are then ignored.
    """
    chan = None
    if channels is not None:
        src, dsts, chan, inst = channel_members(pcb, channels)
        src_token, dst_tokens = src, list(dsts)
    else:
        inst = sheet_instances(pcb)
        src = resolve_sheet(pcb, src_token, '--from', inst)
        dsts = [resolve_sheet(pcb, t, '--to', inst) for t in dst_tokens]
    for t, d in zip(dst_tokens, dsts):
        if d == src:
            raise PoseRefusal('--to %r is the --from sheet itself (/%s); a '
                              'sheet is not copied onto itself' % (t, src),
                              code=2)
        if d.startswith(src + '/') or src.startswith(d + '/'):
            raise PoseRefusal(
                '--to %r (/%s) and --from %r (/%s) are nested, one inside '
                'the other, so they share parts; replicate copies between '
                'SIBLING instances' % (t, d, src_token, src), code=2)
    if len(set(dsts)) != len(dsts):
        raise PoseRefusal('--to names one sheet instance twice (%s)' % ', '.join(
            '/' + d for d in dsts), code=2)
    for i, a in enumerate(dsts):
        for b in dsts[i + 1:]:
            if a.startswith(b + '/') or b.startswith(a + '/'):
                raise PoseRefusal('--to sheets /%s and /%s are nested, one '
                                  'inside the other' % (a, b), code=2)

    from placement.legality import footprint_side
    fps = pcb.footprints
    anchor_key = None
    if anchor is not None and offset is None:
        for prefix in [src] + dsts:
            hit = [k for k, r in _mem(pcb, prefix, chan).items() if r == anchor]
            if hit:
                anchor_key = hit[0]
                break
        if anchor_key is None:
            raise PoseRefusal(
                '--anchor %s is in neither the --from sheet nor a --to sheet '
                '(%s)' % (anchor, ', '.join('/' + p for p in [src] + dsts)),
                code=2)

    waivers = resolve_waivers(pcb, allow_net_mismatch, src, dsts, chan)
    selected = resolve_selection(pcb, only, skip, src, dsts, chan)
    rows: List[Dict] = []
    targets = []
    src_members = _mem(pcb, src, chan)
    dst_members = {d: _mem(pcb, d, chan) for d in dsts}
    if offset is None and anchor_key is None:
        everywhere = [k for k in src_members
                      if all(k in dst_members[d] for d in dsts)]
        copied = [k for k in everywhere
                  if all(selected[d] is None or k in selected[d]
                         for d in dsts)]
        pool = copied or everywhere
        if not pool:
            raise PoseRefusal(
                'no part of /%s has a counterpart in every --to sheet, so no '
                'anchor can be chosen; give --anchor or --offset' % src,
                code=4)
        anchor_key = min(pool, key=lambda k: (
            tuple(-v for v in _anchor_rank(fps[src_members[k]])),
            natural_key(src_members[k])))
    for token, dst in zip(dst_tokens, dsts):
        pairs, nets = correspond(pcb, src, dst, waive=waivers[dst],
                                 keys=selected[dst], chan=chan)
        if not pairs:
            raise PoseRefusal(
                '%s leaves nothing to copy into /%s' % (
                    '--only' if only else '--skip', dst), code=2)
        if offset is not None:
            dx, dy = float(offset[0]), float(offset[1])
            basis = {'kind': 'explicit'}
        else:
            if (anchor_key not in src_members
                    or anchor_key not in dst_members[dst]):
                raise PoseRefusal(
                    'the anchor (%s) has no counterpart in %s, so it cannot '
                    'fix where that copy goes; name another --anchor' % (
                        anchor or src_members.get(anchor_key),
                        '/' + (src if anchor_key not in src_members
                               else dst)), code=4)
            s_ref = src_members[anchor_key]
            t_ref = dst_members[dst][anchor_key]
            dx = float(fps[t_ref].x) - float(fps[s_ref].x)
            dy = float(fps[t_ref].y) - float(fps[s_ref].y)
            basis = {'kind': 'anchor', 'source': s_ref, 'target': t_ref,
                     'chosen': 'given' if anchor else 'most pads'}
        flips = []
        moved = 0
        for s, t in pairs:
            fs, ft = fps[s], fps[t]
            side_s, side_t = footprint_side(fs), footprint_side(ft)
            row = {'source': s, 'ref': t, 'target': dst,
                   'x': round(float(fs.x) + dx, 6),
                   'y': round(float(fs.y) + dy, 6),
                   'rot': float(fs.rotation or 0.0) % 360.0,
                   'side': side_s, 'flip': side_s != side_t}
            if basis.get('source') == s:
                # The anchor keeps its own coordinates EXACTLY -- `a + (b - a)`
                # is not always `b` in floating point.
                row['x'], row['y'] = float(ft.x), float(ft.y)
            rot_t = float(ft.rotation or 0.0) % 360.0
            row['moved'] = (abs(row['x'] - float(ft.x)) > POSE_EPS
                            or abs(row['y'] - float(ft.y)) > POSE_EPS
                            or _rot_differs(row['rot'], rot_t)
                            or row['flip'])
            moved += row['moved']
            if row['flip']:
                flips.append('%s %s->%s' % (t, side_t, side_s))
            rows.append(row)
        d = describe(dst, inst[dst])
        d['as_given'] = token
        copied = {t for _s, t in pairs}
        targets.append({'sheet': d, 'pairs': len(pairs),
                        'moved': moved, 'offset': [round(dx, 6), round(dy, 6)],
                        'offset_basis': basis, 'side_flips': flips,
                        'nets_renamed': nets['renamed'],
                        'nets_shared': nets['shared'],
                        'nets_waived': nets['waived'],
                        # The target parts a partial copy leaves where they
                        # are; [] for a whole copy.
                        'left_in_place': sorted(
                            (r for r in dst_members[dst].values()
                             if r not in copied), key=natural_key)})
    s_desc = describe(src, inst[src])
    s_desc['as_given'] = src_token
    report = {'from': s_desc, 'to': targets,
              'source_refs': sorted(src_members.values(), key=natural_key)}
    if only or skip:
        report['selection'] = {'only' if only else 'skip':
                               list(only or skip)}
    return rows, report


def _rot_differs(a: float, b: float) -> bool:
    d = abs((a - b) % 360.0)
    return min(d, 360.0 - d) > 1e-6


# ---------------------------------------------------------------------------
# the intent's `replicate[]` declaration (#1211)
# ---------------------------------------------------------------------------

def declared_op(decl: Dict) -> Dict:
    """The `place_pose` op one `replicate[]` declaration stands for -- the
    same dict `place_pose replicate` builds from its flags, so a declaration
    and a command line take one path through `pose_ops.resolve_ops`."""
    return {'kind': 'replicate', 'from': decl.get('from'),
            'to': list(decl.get('to') or ()), 'anchor': decl.get('anchor'),
            'channels': decl.get('channels'),
            'offset': None,
            'allow_net_mismatch': list(decl.get('allow_net_mismatch') or ()),
            'only': list(decl.get('only') or ()),
            'skip': list(decl.get('skip') or ()),
            'search': decl.get('search_mm'), 'search_step': None,
            'from_intent': True}


def _median(values: Sequence[float]) -> float:
    v = sorted(values)
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2.0


def measure_declared(pcb, decl: Dict) -> List[Dict]:
    """How far each target of one declaration sits from the source's
    arrangement, one row per target -- the `replicate` rule's measurement.

    The parts are matched exactly as `place_pose replicate` would match
    them (`plan`, with the declaration's waivers and `only` / `skip`), so a
    declaration the verb would refuse raises the verb's own `PoseRefusal`.
    For each target, the part offsets `target - source` are fitted by ONE
    translation, their component-wise MEDIAN -- so a copy laid around any
    anchor reads 0, and one stray part reads as itself instead of smearing
    the fit -- and each part's residual is its distance from that fit. A
    rotation or a side that differs from its source is reported apart: a
    translation cannot fix it. Parts the declaration leaves out are not
    measured."""
    rows, report = plan(pcb, decl.get('from'), decl.get('to') or (),
                        anchor=decl.get('anchor'),
                        allow_net_mismatch=decl.get('allow_net_mismatch') or (),
                        only=decl.get('only') or (),
                        skip=decl.get('skip') or (),
                        channels=decl.get('channels'))
    from placement.legality import footprint_side
    fps = pcb.footprints
    out = []
    for t in report['to']:
        prefix = t['sheet']['path'].lstrip('/')
        pairs = [(r['source'], r['ref']) for r in rows
                 if r['target'] == prefix]
        d = [(float(fps[tr].x) - float(fps[sr].x),
              float(fps[tr].y) - float(fps[sr].y)) for sr, tr in pairs]
        fx, fy = _median([a for a, _b in d]), _median([b for _a, b in d])
        parts = []
        for (sr, tr), (dx, dy) in zip(pairs, d):
            fs, ft = fps[sr], fps[tr]
            parts.append({
                'ref': tr, 'source': sr,
                'residual_mm': round(math.hypot(dx - fx, dy - fy), 4),
                'rot_differs': _rot_differs(float(fs.rotation or 0.0),
                                            float(ft.rotation or 0.0)),
                'side_differs': footprint_side(fs) != footprint_side(ft)})
        parts.sort(key=lambda p: (-p['residual_mm'], natural_key(p['ref'])))
        res = [p['residual_mm'] for p in parts]
        out.append({
            'from': report['from']['name'] or report['from']['path'],
            'to': t['sheet']['name'] or t['sheet']['path'],
            'to_path': t['sheet']['path'],
            'pairs': len(pairs),
            'fit_offset': [round(fx, 4), round(fy, 4)],
            'max_residual_mm': round(max(res), 4) if res else 0.0,
            'rms_residual_mm': (round(math.sqrt(sum(r * r for r in res)
                                                / len(res)), 4)
                                if res else 0.0),
            'rotated': sum(p['rot_differs'] for p in parts),
            'flipped': sum(p['side_differs'] for p in parts),
            'left_in_place': list(t['left_in_place']),
            'parts': parts})
    return out




# ---------------------------------------------------------------------------
# channels inside one sheet, proposed (#1480)
# ---------------------------------------------------------------------------

#: A channel's chip has at least this many connected pads: a line driver, an
#: op-amp, a mixer -- what a channel is built round.
CHIP_MIN_PADS = 5


def _part_sig(fp, chip_pad_of) -> Tuple:
    """What a chip's neighbour IS in its channel: its footprint, its value,
    and per pad (in pad order) the chip pad it shares a net with -- the same
    in every channel when the channel is a copy."""
    return (fp.footprint_name, fp.value or '',
            tuple(chip_pad_of.get(p.net_id) for p in fp.pads or ()))


def suggest_channels(pcb, declined: Optional[List[Dict]] = None) -> List[Dict]:
    """Copies of one channel drawn inside ONE sheet, proposed with their
    correspondence (#1480), pose-blind: for every footprint + value that
    appears as a chip (`CHIP_MIN_PADS` connected pads) two or more times in
    one sheet, each chip's channel is the parts on its LOCAL nets (no rail,
    no net that reaches another of the chips), one hop; a part is matched
    across the chips by its footprint, value and the chip pads its pads
    share nets with (`_part_sig`). A part whose signature is not unique in
    every channel is left out and named. A proposal is kept only when
    `plan` accepts it -- the same parts, footprints, pads and THE NET RULE --
    else appended to `declined` with `plan`'s reason, as a chip group with no
    matched part is. Each kept one carries the ready `replicate[]` row."""
    from placement.grouping import BoardView
    fps = pcb.footprints
    rails = set(BoardView(pcb).rails)
    by: Dict[Tuple, List[str]] = {}
    for ref, fp in fps.items():
        conn = [p for p in fp.pads or () if p.net_id]
        if len(conn) >= CHIP_MIN_PADS:
            by.setdefault((_sheet_of(fp), fp.footprint_name, fp.value or ''),
                          []).append(ref)
    out = []
    for (sheet, name, value), chips in sorted(by.items()):
        if len(chips) < 2:
            continue
        chips = sorted(chips, key=natural_key)
        net_chips: Dict[int, Set[str]] = {}
        for c in chips:
            for p in fps[c].pads or ():
                if p.net_id:
                    net_chips.setdefault(p.net_id, set()).add(c)
        chans: Dict[str, Dict[Tuple, List[str]]] = {}
        for c in chips:
            local = {p.net_id: p.pad_number for p in fps[c].pads or ()
                     if p.net_id and p.net_id not in rails
                     and net_chips.get(p.net_id) == {c}}
            sigs: Dict[Tuple, List[str]] = {}
            for ref, fp in fps.items():
                if ref in chips or not any(p.net_id in local for p in fp.pads or ()):
                    continue
                sigs.setdefault(_part_sig(fp, local), []).append(ref)
            chans[c] = sigs
        common = set.intersection(*(set(s) for s in chans.values()))
        keep = sorted((k for k in common
                       if all(len(chans[c][k]) == 1 for c in chips)), key=repr)
        left = {c: sorted((r for k, rs in chans[c].items() if k not in keep for r in rs),
                          key=natural_key) for c in chips}
        channels = [[c] + [chans[c][k][0] for k in keep] for c in chips]
        rec = {'sheet': '/' + sheet if sheet else '/', 'chip': name, 'value': value,
               'chips': chips, 'channels': channels,
               'left_out': {c: rs for c, rs in left.items() if rs}}
        if not keep:
            rec['refused'] = 'no part of a channel has a counterpart in every other'
        else:
            try:
                plan(pcb, None, (), channels=channels)
                rec['row'] = {'channels': channels,
                              'why': f"{len(chips)} copies of the {value or name} "
                                     f"channel ({', '.join(chips)}), matched by footprint, "
                                     f"value and chip pin"}
            except PoseRefusal as e:
                rec['refused'] = str(e)
        if 'row' in rec:
            out.append(rec)
        elif declined is not None:
            declined.append(rec)
    return out
