#!/usr/bin/env python3
"""#600: the improvement gate must not ship a board a run made worse.

Covers the decision logic (compare_connectivity + gate_verdict) against the
shapes the sets-21-27 wave actually produced, and the connectivity map against
a synthetic board so a change to the underlying checker cannot silently flip
the gate's inputs.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_router'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from improvement_gate import (net_connectivity_map, compare_connectivity,
                              gate_verdict, format_report)
from synth import make_pad, make_seg, make_pcb, make_net
from kicad_parser import BoardInfo

_failures = []


def check(label, cond):
    print(f"{'PASS' if cond else 'FAIL'}: {label}")
    if not cond:
        _failures.append(label)


def _cmp(before, after, names=None):
    names = names or {}
    return compare_connectivity(before, after, lambda nid: names.get(nid, f"n{nid}"))


# ---------------------------------------------------------------- decisions
# (connected, disconnected_pad_count) per net id.

# bms_sensor's wave shape: a retry scoped to fix 3 pads ripped 9 unrelated
# fully-routed nets and could not restore them (3 -> 20 disconnected pads).
before = {i: (True, 0) for i in range(1, 11)}
before[11] = (False, 3)
after = {i: (False, 2) for i in range(1, 10)}
after[10] = (True, 0)
after[11] = (True, 0)
c = _cmp(before, after)
check("bms shape: 9 lost vs 1 gained is rejected",
      len(c['lost']) == 9 and len(c['gained']) == 1
      and gate_verdict(c) == 'reject')

# spartan6's shape: one net worse on the net count is enough.
c = _cmp({1: (True, 0), 2: (True, 0)}, {1: (True, 0), 2: (False, 4)})
check("one net lost, none gained -> reject", gate_verdict(c) == 'reject')

# Progress with collateral: five closed, one broken, and the pad count falls.
# Discarding this would throw away more than it saves.
_b5 = {i: (False, 2) for i in range(1, 6)}
_b5[9] = (True, 0)
_a5 = {i: (True, 0) for i in range(1, 6)}
_a5[9] = (False, 1)
c = _cmp(_b5, _a5)
check("5 gained vs 1 lost, pads fall -> accept", gate_verdict(c) == 'accept')

# spartan6_4layer's REAL numbers, re-running its wave command (2026-08-11):
# the net count says better by 7 while the board's open pads nearly DOUBLE,
# because the 36 nets it broke are far bigger than the 43 it closed. Neither
# axis may outvote the other -- an earlier lexicographic rule with nets first
# SHIPPED this board.
_bs = {i: (True, 0) for i in range(1, 37)}          # 36 connected -> broken
_bs.update({100 + i: (False, 2) for i in range(43)})  # 43 broken -> connected
_as = {i: (False, 4) for i in range(1, 37)}
_as.update({100 + i: (True, 0) for i in range(43)})
c = _cmp(_bs, _as)
check("spartan6 shape: net count better, pads worse -> REJECT",
      len(c['gained']) > len(c['lost'])
      and c['disconnected_pads_after'] > c['disconnected_pads_before']
      and gate_verdict(c) == 'reject')

# An equal trade that leaves the pad count untouched is a genuine lateral move
# (bms_sensor's retry reproduced today) -- it ships, loudly reported.
c = _cmp({1: (True, 0), 2: (True, 0), 3: (False, 3)},
         {1: (False, 3), 2: (True, 0), 3: (True, 0)})
check("equal trade, equal pads -> accept (and is reported)",
      len(c['lost']) == 1 and len(c['gained']) == 1
      and gate_verdict(c) == 'accept')
c = _cmp({1: (True, 0), 2: (False, 1)},
         {1: (False, 9), 2: (True, 0)})
check("equal trade but MORE disconnected pads -> reject",
      gate_verdict(c) == 'reject')
c = _cmp({1: (True, 0), 2: (False, 9)},
         {1: (False, 1), 2: (True, 0)})
check("equal trade with FEWER disconnected pads -> accept",
      gate_verdict(c) == 'accept')

# A clean run must never trip the gate.
c = _cmp({1: (True, 0), 2: (True, 0)}, {1: (True, 0), 2: (True, 0)})
check("no change -> accept, nothing reported",
      gate_verdict(c) == 'accept' and not c['lost'] and not c['gained'])

# Nets present in only one reading are a scope/parse difference, never a
# routing outcome -- they must not be able to trip the gate.
c = _cmp({1: (True, 0), 2: (True, 0)}, {1: (True, 0)})
check("net missing from the 'after' reading is ignored",
      not c['lost'] and gate_verdict(c) == 'accept' and c['nets_compared'] == 1)

# The report names the nets: a count alone is not actionable.
c = _cmp({1: (True, 0)}, {1: (False, 2)}, names={1: '/GND'})
r = format_report(c, gate_verdict(c), 'REVERTED')
check("report names the broken net", '/GND' in r and 'REVERT' in r)

# #1032: a PAD-COUNT-ONLY rejection must still name the net. GND already had
# open pads before the run, so it is not `lost`; before the fix the head line
# read "broke 0 ... connected 1 -- REJECTED" and named nothing.
c = _cmp({1: (False, 30), 2: (False, 2)}, {1: (False, 36), 2: (True, 0)},
         names={1: 'GND', 2: '/SIG'})
r = format_report(c, gate_verdict(c), 'REVERTED')
_head = r.splitlines()[0]
check("#1032 pad-count-only rejection: verdict is reject",
      gate_verdict(c) == 'reject' and not c['lost'])
check("#1032 pad-count-only rejection names GND in the head line",
      'GND' in _head and 'REJECTED' in _head)
check("#1032 worsened carries name + before->after",
      c['worsened'] == [('GND', 30, 36)] and 'GND 30->36' in r)

# #1032: a net EXCLUDED by plan (a pour outside --nets, which the finalize was
# told not to repair) cannot reject -- same shapes, GND excluded -> accept, and
# the damage is still reported on its own line.
c = _cmp({1: (True, 0), 2: (False, 2)}, {1: (False, 3), 2: (True, 0)},
         names={1: 'GND', 2: '/SIG'})
check("#1032 control: GND lost without exclusion -> reject",
      c['lost'] == ['GND'] and gate_verdict(c) == 'reject')
c = compare_connectivity({1: (True, 0), 2: (False, 2)},
                         {1: (False, 3), 2: (True, 0)},
                         lambda nid: {1: 'GND', 2: '/SIG'}[nid],
                         excluded_ids={1})
r = format_report(c, gate_verdict(c), 'shipped')
check("#1032 excluded net cannot reject",
      gate_verdict(c) == 'accept' and not c['lost'] and not c['worsened']
      and c['nets_compared'] == 1
      and c['disconnected_pads_before'] == 2
      and c['disconnected_pads_after'] == 0)
check("#1032 excluded net is reported, not hidden",
      c['excluded_by_plan'] == [('GND', 0, 3)]
      and 'excluded' in r and 'GND 0->3' in r)

# The head line caps its list.
c = _cmp({i: (True, 0) for i in range(10)}, {i: (False, 2) for i in range(10)})
_head = format_report(c, gate_verdict(c), 'x').splitlines()[0]
check("#1032 head line caps the named nets", '+4 more' in _head)

# The fallback derivation uses the finalize's own rule.
from improvement_gate import plan_excluded_net_names
check("#1032 plan exclusion: zone nets outside --nets",
      plan_excluded_net_names(['GND', '+3V3'], ['/D1', '+3V3']) == ['GND'])
check("#1032 plan exclusion: globs follow matches_net_filter",
      plan_excluded_net_names(['GND', '+3V3'], ['*']) == []
      and plan_excluded_net_names(['GND', '+3V3'], ['*', '!GND']) == ['GND'])
check("#1032 plan exclusion: an unscoped call excludes nothing",
      plan_excluded_net_names(['GND'], []) == []
      and plan_excluded_net_names(['GND'], None) == [])


# ------------------------------------------------------- connectivity map
# Two pads joined by one segment = connected; the same pads with the segment
# removed = broken. This is the input the whole gate rests on.
nets = {1: make_net(1, '/A')}
pads = [make_pad(1, 0.0, 0.0, ref='U1', num='1', net_name='/A'),
        make_pad(1, 5.0, 0.0, ref='U2', num='1', net_name='/A')]
bi = BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
               board_bounds=(-1.0, -1.0, 6.0, 1.0))
joined = make_pcb(nets=nets, pads_by_net={1: pads}, board_info=bi,
                  segments=[make_seg(0.0, 0.0, 5.0, 0.0, net_id=1)])
bare = make_pcb(nets=nets, pads_by_net={1: pads}, board_info=bi, segments=[])

m_join = net_connectivity_map(joined)
m_bare = net_connectivity_map(bare)
check("joined pads read as connected", m_join.get(1, (False, 0))[0] is True)
check("bare pads read as disconnected", m_bare.get(1, (True, 0))[0] is False)
c = _cmp(m_join, m_bare, names={1: '/A'})
check("losing the only segment is a rejected regression",
      c['lost'] == ['/A'] and gate_verdict(c) == 'reject')
c = _cmp(m_bare, m_join, names={1: '/A'})
check("routing it is a gain", c['gained'] == ['/A']
      and gate_verdict(c) == 'accept')

# Single-pad nets are trivially connected and must not appear at all.
one = make_pcb(nets={2: make_net(2, '/B')}, board_info=bi,
               pads_by_net={2: [make_pad(2, 0.0, 0.0, net_name='/B')]})
check("single-pad net is not measured", 2 not in net_connectivity_map(one))


# ------------------------------------------------------------ the ACTION
# The decision logic above is pure; this exercises what the engine DOES with a
# rejection, on BOTH fronts. The verdict is forced (a rejection needs a run
# that genuinely breaks a board, which is not something to synthesise), so what
# is under test here is the rollback plumbing, not the decision.
import shutil
import subprocess
import tempfile

import improvement_gate as _ig
import route as _route

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_BOARD = os.path.join(_REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')

if os.path.isfile(_BOARD):
    _tmp = tempfile.mkdtemp(prefix='gate600_')
    _seed = os.path.join(_tmp, 'seed.kicad_pcb')
    shutil.copy(_BOARD, _seed)
    # Step 1: put copper on the board (the gate is inert with no input copper).
    _routed = os.path.join(_tmp, 'routed.kicad_pcb')
    _r1 = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(_REPO, 'py_router', 'route.py'),
         _seed, _routed, '--nets', '*', '!GND', '--track-width', '0.2',
         '--clearance', '0.2', '--grid-step', '0.1'],
        capture_output=True, text=True, timeout=1200)
    check("seed route produced a board", os.path.isfile(_routed))

    if os.path.isfile(_routed):
        _before = open(_routed, 'rb').read()
        _out = os.path.join(_tmp, 'out.kicad_pcb')
        # batch_route takes REAL net names -- '*'/'!GND' are patterns main()
        # expands, and passing them here routes NOTHING, which makes both
        # checks below pass vacuously (the no-valid-nets path also
        # passthrough-copies the board). Name nets that actually have copper.
        from kicad_parser import parse_kicad_pcb as _pk
        _rp = _pk(_routed)
        _with_copper = {s.net_id for s in _rp.segments}
        _names = [n.name for nid, n in _rp.nets.items()
                  if nid in _with_copper and n.name and n.name != 'GND'][:6]
        check("test names real routed nets (not patterns)", len(_names) >= 2)
        _orig_verdict = _ig.gate_verdict
        try:
            _ig.gate_verdict = lambda cmp: 'reject'
            # CLI front: the output file must come back as the INPUT board.
            _route.batch_route(_routed, _out, _names, track_width=0.2,
                               clearance=0.2, grid_step=0.1,
                               force_reroute=True)
            check("CLI: a rejected run reverts the output to the input board",
                  os.path.isfile(_out)
                  and open(_out, 'rb').read() == _before)
            # GUI front: the applier must be handed nothing to apply.
            _ok, _f, _t, _data = _route.batch_route(
                _routed, '', _names, track_width=0.2, clearance=0.2,
                grid_step=0.1, force_reroute=True, return_results=True)
            check("GUI: a rejected run returns an empty change-set",
                  not _data.get('results')
                  and not _data.get('segments_to_remove')
                  and not _data.get('vias_to_remove')
                  and not _data.get('all_swap_vias'))
            check("GUI: the verdict rides along for the caller",
                  (_data.get('improvement_gate') or {}).get('verdict') == 'reject')
            check("GUI: diagnostics survive the rejection",
                  'blockers' in _data and 'pad_pairs_open' in _data)
        finally:
            _ig.gate_verdict = _orig_verdict
            shutil.rmtree(_tmp, ignore_errors=True)

else:
    print("SKIP: in-repo splitflap board missing -- action checks not run")

print()
if _failures:
    print(f"{len(_failures)} FAILURE(S): " + "; ".join(_failures))
    sys.exit(1)
print("All improvement-gate checks passed")
