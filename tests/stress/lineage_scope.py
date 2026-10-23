#!/usr/bin/env python3
"""Which routable nets does NO step of a board's final lineage scope?

    python3 tests/stress/lineage_scope.py RUN/redo_commands.sh [...] [--json OUT]

A replay grades the board its chain ends on, and an open net there is the
engine's only if some step of the chain that made that board was ASKED to
route it. One recorded board taught this. Its recorded chain
routed GND alone in one step and leaned on that step's end-of-run
reconciliation, which in those days enrolled every pad-deficit net on the
board -- 35 of them, the whole RP2040 side. #734 scoped the reconciliation to
the requested nets, so its replays since then ran no lineage step that
routes those nets: 34 shipped open while every step reported success, and
the grade booked them against the engine.

For one manifest this reads:

  * the LINEAGE -- the commands `redo_stress_test.compute_prune_keep` keeps,
    which are the ones a replay runs to make the final board;
  * what each lineage step SCOPES, from the tool's own argparse
    (`cli_argv_check.parsed_args`) and the same expansion its main applies:
      route.py         positional patterns + --nets, '*' when it is given
                       neither nor --component/--group, expanded by
                       `net_queries.expand_net_patterns` ('!' excludes,
                       globs, sheet-path leaf matching, no 'unconnected-*'
                       in a wildcard), then intersected with --component /
                       --group as main() does; --undo routes nothing
      route_diff.py    the differential PAIRS `find_differential_pairs`
                       selects from its expanded patterns -- it skips every
                       other net
      route_planes.py  the poured --nets ('|' joins several on one layer).
                       Since #562 a pour places no tap via and draws no
                       track, so a net only a pour scopes is `pour_only`
      repair_planes.py --nets with --plane-layers, else the zones on the
                       board at that step (filtered by --nets/--plane-layers)
      qfn/bga_fanout   escape stubs on the part's pads: context only
                       (`escaped_by`), never routing -- a stub joins no pads
      place_fanout_clearance.py, copy_board.py, cp, mv: no net
  * the board's ROUTABLE nets, the ones check_connected grades: named, >= 2
    pads, not an auto-named 'unconnected-*', pads not already joined by
    overlapping copper -- read off the chain's unrouted input board.

and REPORTS (it never fails a board):

  unscoped               routable nets no routing step of the lineage scopes,
                         on which the input board carries no copper at all
  unscoped_input_copper  the same, but the input already has a track, via or
                         zone on the net, so it may arrive connected
  pour_only              poured, and scoped by no routing step
  unmodelled             lineage steps whose scope this cannot read (a tool
                         it does not know, a parser it cannot build, a --group
                         that does not resolve). While one is listed, the
                         lists above are an UPPER bound.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for _d in (str(HERE), str(REPO / 'py_router'), str(REPO / 'py_placer'),
           str(REPO / 'py_tools')):
    if _d not in sys.path:
        sys.path.insert(0, _d)

import cli_argv_check  # noqa: E402
from redo_stress_test import (board_io, compute_prune_keep,  # noqa: E402
                              is_check_cmd, parse_manifest)

#: Steps that move or copy boards and scope no net.
NO_SCOPE_TOOLS = {'place_fanout_clearance.py', 'copy_board.py', 'cp', 'mv'}
FANOUT_TOOLS = {'qfn_fanout.py', 'bga_fanout.py'}
REPAIR_TOOLS = {'repair_planes.py', 'route_disconnected_planes.py'}
#: Step kinds that connect pads (a net they scope is routed by the chain).
ROUTING_KINDS = {'route', 'pair', 'repair'}


def _quiet(fn, *a, **k):
    """The parser and the expansion helpers print match counts and board
    warnings; the report is the output here. Exceptions still propagate."""
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def tool_name(argv):
    """The step's tool: the first `.py` token, past any `NAME=value` env
    prefix and the interpreter; a shell command's own name otherwise."""
    for a in argv:
        if a.endswith('.py'):
            return os.path.basename(a)
    for a in argv:
        if '=' not in a:
            return os.path.basename(a)
    return '?'


def routable_nets(pcb):
    """{name: (net_id, pad count)} for every net check_connected grades."""
    from check_connected import _net_pads_connected_by_overlap
    layers = pcb.board_info.copper_layers or ['F.Cu', 'B.Cu']
    out = {}
    for nid, net in pcb.nets.items():
        name = net.name
        if not name or nid == 0 or name.lower().startswith('unconnected-'):
            continue
        pads = pcb.pads_by_net.get(nid, [])
        if len(pads) < 2 or _net_pads_connected_by_overlap(pads, layers):
            continue
        out[name] = (nid, len(pads))
    return out


def _input_copper_net_ids(pcb):
    ids = {s.net_id for s in pcb.segments if s.net_id and not getattr(s, 'graphic', False)}
    ids |= {v.net_id for v in pcb.vias if v.net_id}
    ids |= {z.net_id for z in pcb.zones if z.net_id}
    return ids


def _route_scope(ns, pcb):
    from net_queries import expand_net_patterns, nets_for_components
    from routing_constants import POWER_NET_EXCLUSION_PATTERNS
    if ns.get('undo'):
        return 'unroute', set(), '--undo strips copper'
    pats = [p.strip() for p in list(ns.get('net_patterns') or [])
            + list(ns.get('nets') or []) if p.strip()]
    comps = list(ns.get('component') or [])
    group = ns.get('group')
    if not pats and not comps and not group:
        pats = ['*']
    names = _quiet(expand_net_patterns, pcb, pats) if pats else []
    if comps:
        sel = _quiet(nets_for_components, pcb, comps,
                     exclude_patterns=None if pats else POWER_NET_EXCLUSION_PATTERNS)
        names = ([n for n in names if n in set(sel.net_names)] if pats
                 else list(sel.net_names))
    if group:
        try:
            from group_routing import block_net_names, block_refs
            import _placer_path  # noqa: F401
            from placement.groups import parse_sources
            refs = block_refs(pcb, group, parse_sources(ns.get('group_by') or 'auto'))
            gnets = set(block_net_names(pcb, refs, ns.get('group_scope') or 'touching'))
        except Exception as e:  # noqa: BLE001 -- unreadable scope, not a crash
            return 'unmodelled', set(), f'--group {group!r}: {type(e).__name__}: {e}'
        names = [n for n in names if n in gnets] if pats else sorted(gnets)
    return 'route', set(names), ''


def _diff_scope(ns, pcb):
    from net_queries import expand_net_patterns, find_differential_pairs
    pats = [p.strip() for tok in list(ns.get('net_patterns') or [])
            + list(ns.get('nets') or []) for p in tok.split(',') if p.strip()]
    names = _quiet(expand_net_patterns, pcb, pats or ['*'])
    pairs = _quiet(find_differential_pairs, pcb, names)
    out = set()
    for pr in pairs.values():
        for nid, nm in ((pr.p_net_id, pr.p_net_name), (pr.n_net_id, pr.n_net_name)):
            nm = nm or (pcb.nets[nid].name if nid in pcb.nets else None)
            if nm:
                out.add(nm)
    return 'pair', out, ''


def _split_bar(values):
    return [n.strip() for v in values or [] for n in str(v).split('|') if n.strip()]


def step_scope(argv, cwd, pcb, zones):
    """(kind, nets, note) for one lineage command. `zones` is the set of
    (net, layer) zones on the board when the step runs; a pour adds its own."""
    tool = tool_name(argv)
    if tool in NO_SCOPE_TOOLS:
        return 'none', set(), ''
    rep = cli_argv_check.parsed_args(argv, cwd)
    if rep is None:
        return 'unmodelled', set(), f'{tool}: no parser for it here'
    if not rep['ok']:
        return 'refused', set(), rep.get('why') or ''
    ns = rep['ns']
    board_nets = {n.name for n in pcb.nets.values() if n.name}
    if tool == 'route.py':
        return _route_scope(ns, pcb)
    if tool == 'route_diff.py':
        return _diff_scope(ns, pcb)
    if tool == 'route_planes.py':
        # plane_io.resolve_net_id: an exact name match. --nets pairs with
        # --plane-layers by position, '|' putting several nets on one layer.
        for arg, ly in zip(ns.get('nets') or [], ns.get('plane_layers') or []):
            zones |= {(n, ly) for n in _split_bar([arg])}
        return 'pour', {n for n in _split_bar(ns.get('nets')) if n in board_nets}, ''
    if tool in REPAIR_TOOLS:
        nets, layers = _split_bar(ns.get('nets')), list(ns.get('plane_layers') or [])
        if nets and layers:
            return 'repair', {n for n in nets if n in board_nets}, ''
        return 'repair', {n for n, ly in zones
                          if (not nets or n in nets) and (not layers or ly in layers)}, ''
    if tool in FANOUT_TOOLS:
        from net_queries import matches_net_filter
        ref = ns.get('component')
        fp = pcb.footprints.get(ref) if ref else None
        if fp is None:
            return 'escape', set(), f'component {ref or "auto-detected"}'
        nets = {p.net_name for p in fp.pads if p.net_id and p.net_name}
        if ns.get('nets'):
            nets = {n for n in nets if matches_net_filter(n, ns['nets'])}
        return 'escape', nets, ref
    return 'unmodelled', set(), f'{tool}: scope not modelled'


def input_board(cmds, keep, manifest):
    """The chain's unrouted input: the first lineage command's first board,
    resolved against its cwd, then the manifest's folder."""
    first = next((i for i in sorted(keep) if not is_check_cmd(cmds[i][1])), None)
    if first is None:
        return None
    cwd, argv = cmds[first]
    ins, _out = board_io(argv)
    vals = [a.split('=', 1)[1] if a.startswith('-') and '=' in a else a for a in argv]
    tok = next((v for v in vals if ins and os.path.basename(v) == ins[0]), None)
    if tok is None:
        return None
    for cand in (tok, os.path.join(cwd or '', tok),
                 os.path.join(os.path.dirname(os.path.abspath(manifest)), tok)):
        if os.path.isfile(cand):
            return cand
    return tok


def lineage_scope(manifest, board=None):
    """The report for one manifest (see the module docstring). `board` is the
    chain's unrouted input; by default the first lineage command's input."""
    from kicad_parser import parse_kicad_pcb
    cmds = parse_manifest(manifest)
    keep, info = compute_prune_keep(cmds)
    board = board or input_board(cmds, keep, manifest)
    rep = {'manifest': str(manifest), 'board': board,
           'final': info.get('final_board'), 'lineage': [],
           'unscoped': [], 'unscoped_input_copper': [], 'pour_only': [],
           'unmodelled': []}
    if not board or not os.path.isfile(board):
        rep['error'] = f'input board not found: {board}'
        return rep
    pcb = _quiet(parse_kicad_pcb, board)
    routable = routable_nets(pcb)
    copper = _input_copper_net_ids(pcb)
    zones = {(z.net_name, z.layer) for z in pcb.zones if z.net_name}
    routed, poured, escaped = set(), set(), {}
    for i in sorted(keep):
        cwd, argv = cmds[i]
        if is_check_cmd(argv):
            continue
        tool = tool_name(argv)
        kind, nets, note = step_scope(argv, cwd, pcb, zones)
        step = {'index': i + 1, 'tool': tool, 'kind': kind, 'nets': len(nets)}
        if note:
            step['note'] = note
        rep['lineage'].append(step)
        if kind in ROUTING_KINDS:
            routed |= nets
        elif kind == 'pour':
            poured |= nets
        elif kind == 'escape':
            for n in nets:
                escaped.setdefault(n, []).append(f'{tool} {note}'.strip())
        elif kind in ('unmodelled', 'refused'):
            rep['unmodelled'].append(f'step {i + 1} {tool}: {kind}'
                                     + (f' ({note})' if note else ''))
    rep['routable_nets'] = len(routable)
    for name, (nid, npads) in sorted(routable.items()):
        if name in routed:
            continue
        row = {'net': name, 'pads': npads}
        if name in escaped:
            row['escaped_by'] = escaped[name]
        if name in poured:
            rep['pour_only'].append(row)
        elif nid in copper:
            rep['unscoped_input_copper'].append(row)
        else:
            rep['unscoped'].append(row)
    return rep


def compact(rep):
    """The fields a grade row carries: net names only, per list."""
    out = {k: [r['net'] for r in rep.get(k, [])]
           for k in ('unscoped', 'unscoped_input_copper', 'pour_only')}
    out['unmodelled'] = list(rep.get('unmodelled', []))
    if rep.get('error'):
        out['error'] = rep['error']
    return out


def describe(rep, limit=8):
    """One line for a grade log, or '' when every routable net is scoped."""
    un = rep.get('unscoped') or []
    if not un:
        return ''
    names = [r['net'] if isinstance(r, dict) else r for r in un]
    more = f' (+{len(names) - limit} more)' if len(names) > limit else ''
    return (f"{len(names)} routable net(s) NO step of the final lineage routes"
            f" -- open there is the chain's, not the engine's: "
            f"{', '.join(names[:limit])}{more}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('manifests', nargs='+', help='redo_commands.sh file(s)')
    ap.add_argument('--board', help="the chain's unrouted input (one manifest "
                                    "only; default: the first lineage step's input)")
    ap.add_argument('--json', dest='json_out', help='write every report here')
    a = ap.parse_args()
    if a.board and len(a.manifests) > 1:
        ap.error('--board names ONE manifest\'s input board')
    reps, flagged = [], 0
    for m in a.manifests:
        try:
            rep = lineage_scope(m, board=a.board)
        except Exception as e:  # noqa: BLE001 -- a scan reports, never dies
            rep = {'manifest': m, 'error': f'{type(e).__name__}: {e}'}
        reps.append(rep)
        board = os.path.basename(os.path.dirname(os.path.abspath(m)))
        n = {k: len(rep.get(k) or []) for k in
             ('unscoped', 'unscoped_input_copper', 'pour_only', 'unmodelled')}
        if rep.get('error'):
            print(f"{board}: ERROR {rep['error']}")
        elif any(n.values()):
            flagged += 1
            print(f"{board}: unscoped={n['unscoped']} "
                  f"unscoped_input_copper={n['unscoped_input_copper']} "
                  f"pour_only={n['pour_only']} unmodelled={n['unmodelled']}  "
                  f"[{m}]")
            for k in ('unscoped', 'unscoped_input_copper', 'pour_only'):
                for r in rep.get(k) or []:
                    esc = f"  escaped by {', '.join(r['escaped_by'])}" if r.get('escaped_by') else ''
                    print(f"    {k}: {r['net']} ({r['pads']} pads){esc}")
            for u in rep.get('unmodelled') or []:
                print(f"    unmodelled: {u}")
    print(f"{len(reps)} manifest(s): {flagged} with nets their final lineage "
          f"does not scope")
    if a.json_out:
        with open(a.json_out, 'w', encoding='utf-8') as fh:
            json.dump(reps, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
