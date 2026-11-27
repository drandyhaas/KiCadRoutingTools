#!/usr/bin/env python3
"""Board-wide rescue maps derive from one shared no-net base instead of a
whole-board build per net, and the base follows routed copper across copper
epochs instead of being rebuilt.

The rescue, the own-requirement retry and the terminal escalation ask
`_pristine_rescue_map` for a board-wide map for every failed net. Each was a
full `build_base_obstacle_map(pcb, cfg, [net])`: every pad, track and via on
the board stamped again per net, and again for every net after each routed
net bumped the copper epoch. The fix builds the map for NO net once per
(geometry, clearance floor), derives a net's map as

    map(N) = base - stamps(N's own copper) + N's sense branches

and carries the base to a new epoch by subtracting the stamps of the copper
removed since and adding those of the copper added.

Checks (a synthetic two-layer board: a row of nets with pads, tracks and
vias, several failed nets asked for at two copper epochs):
  1. every map equals a fresh full build for that net, cell for cell
     (`content_digest`), at both epochs;
  2. at most one whole-board build serves all nets at the first epoch, and
     none at the second (the base is carried over, not rebuilt);
  3. KICAD_NET_MAP_VERIFY=1 runs the same requests with every derived map
     and every carried base checked against a fresh build, without a raise;
  4. a board with scoped .kicad_dru clearance rules is never derived;
  5. an escalation walking every net through a full ladder of geometries
     builds one base per geometry, not one per net and rung;
  6. a pad, zone or config input changed before a new epoch rebuilds the
     base instead of carrying it over;
  7. so do in-place pad edits (a net swap across clearance classes, a pad
     moved through global_x), which keep the pad lists' identity and length;
  8. an input compared by identity is held, so a new one at a freed address
     is not taken for the old one;
  9. with KICAD_NET_MAP_VERIFY=1, copper edited without bump_copper_epoch
     raises, naming the contract;
 10. a power net's rungs (its own width override popped) are bases of
     their own that do not evict the rungs every other net shares.

    python3 tests/test_2078_incremental_rescue_maps.py
"""
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import env_knobs  # noqa: E402
import net_rescue  # noqa: E402
import obstacle_map  # noqa: E402
from kicad_parser import BoardInfo, Net  # noqa: E402
from pcb_modification import bump_copper_epoch  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from synth import make_pad, make_pcb, make_seg, make_via  # noqa: E402

LAYERS = ['F.Cu', 'B.Cu']
N = 8
fails = []


def ok(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def board():
    """N nets in a row 1.5 mm apart, each a pad pair joined by a track with a
    via part way; a few more pads per net so its own-pad stamps matter."""
    nets = {i: Net(i, f'/N{i}') for i in range(1, N + 1)}
    pads_by_net, segs, vias = {}, [], []
    for i in range(1, N + 1):
        x = 1.5 * i
        pads_by_net[i] = [make_pad(i, x, 2.0, ref=f'U{i}', num='1'),
                          make_pad(i, x, 12.0, ref=f'U{i}', num='2'),
                          make_pad(i, x + 0.6, 7.0, ref=f'R{i}', num='1',
                                   layers=('B.Cu',))]
        if i % 2:
            segs += [make_seg(x, 2.0, x, 6.0, net_id=i),
                     make_seg(x, 6.0, x, 12.0, net_id=i, layer='B.Cu')]
            vias.append(make_via(x, 6.0, net_id=i))
    bi = BoardInfo(layers={}, board_bounds=(0, 0, 1.5 * N + 3, 14),
                   copper_layers=list(LAYERS))
    return make_pcb(board_info=bi, nets=nets, pads_by_net=pads_by_net,
                    segments=segs, vias=vias)


def full_build_counter():
    """Counts whole-board builds: neither a single net's stamps nor a copper
    delta, i.e. every pad, track and via on the board stamped."""
    real = obstacle_map.build_base_obstacle_map
    calls = []

    def counted(*a, **kw):
        if kw.get('_only_nets') is None and kw.get('_only_copper') is None:
            calls.append(tuple(a[2]) if len(a) > 2 else tuple(kw.get('nets_to_route', ())))
        return real(*a, **kw)
    return real, counted, calls


def digest(m):
    return m.content_digest() if hasattr(m, 'content_digest') else None


def run(label, counts=True):
    pcb = board()
    cfg = GridRouteConfig(layers=list(LAYERS))
    failed = list(range(2, N + 1, 2)) + [1, 3]
    real, counted, calls = full_build_counter()
    obstacle_map.build_base_obstacle_map = counted
    per_epoch, equal = [], True
    try:
        for epoch_i in range(2):
            if epoch_i:
                # a routed net lands: new copper, and one track ripped
                pcb.segments.append(make_seg(3.0, 2.0, 3.0, 12.0, net_id=2))
                pcb.vias.append(make_via(4.5, 9.0, net_id=3))
                pcb.segments.pop(0)
                bump_copper_epoch(pcb)
            before = len(calls)
            for n in failed:
                got = net_rescue._pristine_rescue_map(pcb, pcb, cfg, n, {}, ('full',))
                n_calls = len(calls)
                fresh = real(pcb, cfg, [n], net_clearances={})
                del calls[n_calls:]
                if digest(got) is None or digest(got) != digest(fresh):
                    equal = False
            per_epoch.append(len(calls) - before)
    finally:
        obstacle_map.build_base_obstacle_map = real
    ok(f'{label} 1: every rescue map equals a fresh build for its net', equal)
    if not counts:
        return
    ok(f'{label} 2a: one whole-board build serves {len(failed)} nets', per_epoch[0] <= 1,
       f'{per_epoch[0]} builds')
    ok(f'{label} 2b: the next epoch carries the base over (no whole-board build)',
       per_epoch[1] == 0, f'{per_epoch[1]} builds')


run('plain')
env_knobs.NET_MAP_VERIFY = True
try:
    run('verify', counts=False)  # verify's own fresh builds count too
except AssertionError as exc:
    ok('verify 3: no derived map or carried base differs from a fresh build', False, str(exc))
finally:
    env_knobs.NET_MAP_VERIFY = False

# 4. A board with scoped .kicad_dru rules (#1491) is built in full: those
# rules are installed for the routed net set, so a no-net base would price
# foreign copper differently from a build for [net].
_cfg = GridRouteConfig(layers=list(LAYERS))
_cfg.scoped_clearance = object()
_eligible = getattr(obstacle_map, 'incremental_net_eligible', None)
ok('4: scoped clearance rules make the delta ineligible',
   _eligible is not None and not _eligible(board(), _cfg),
   '' if _eligible else 'obstacle_map has no incremental_net_eligible')

# 5. Terminal escalation walks each net through every rung of its ladder in
# turn; the base cache must hold the whole ladder or each net evicts the
# base the next one needs first.
from dataclasses import replace as _replace  # noqa: E402
_pcb = board()
_base = GridRouteConfig(layers=list(LAYERS))
_ladder = [_replace(_base, track_width=w) for w in (0.2, 0.18, 0.16, 0.14)]
_real, _counted, _calls = full_build_counter()
obstacle_map.build_base_obstacle_map = _counted
try:
    for _n in range(1, 5):
        for _cfg in _ladder:
            net_rescue._pristine_rescue_map(_pcb, _pcb, _cfg, _n, {}, ('full',))
finally:
    obstacle_map.build_base_obstacle_map = _real
ok(f'5: a {len(_ladder)}-rung escalation over 4 nets builds one base per rung',
   len(_calls) == len(_ladder), f'{len(_calls)} builds')

# 6. A base carried to a new copper epoch is stamped with copper only, so a
# pad, zone or config input that changed in between (a BGA keep-out, a pad
# moved to another net, a pad added) must rebuild it, never rebase it.
def _changed_inputs(label, change):
    pcb = board()
    cfg = GridRouteConfig(layers=list(LAYERS))
    name = f'6 {label}: the next epoch rebuilds, equal to a fresh build'
    env_knobs.NET_MAP_VERIFY = True
    try:
        net_rescue._pristine_rescue_map(pcb, pcb, cfg, 2, {}, ('full',))
        change(pcb, cfg)
        pcb.segments.append(make_seg(3.0, 2.0, 3.0, 12.0, net_id=2))
        bump_copper_epoch(pcb)
        got = net_rescue._pristine_rescue_map(pcb, pcb, cfg, 4, {}, ('full',))
        fresh = obstacle_map.build_base_obstacle_map(pcb, cfg, [4], net_clearances={})
        ok(name, digest(got) is not None and digest(got) == digest(fresh))
    except AssertionError as exc:
        ok(name, False, str(exc))
    finally:
        env_knobs.NET_MAP_VERIFY = False


def _bga(pcb, cfg):
    cfg.bga_exclusion_zones = [(4.0, 4.0, 8.0, 10.0)]


def _pad_moved(pcb, cfg):
    pad = pcb.pads_by_net[5].pop()
    pad.net_id = 6
    pcb.pads_by_net[6] = pcb.pads_by_net[6] + [pad]


def _pad_added(pcb, cfg):
    pcb.pads_by_net[7] = pcb.pads_by_net[7] + [make_pad(7, 9.0, 9.0, ref='T1', num='1')]


_changed_inputs('BGA keep-out set', _bga)
_changed_inputs('pad moved to another net', _pad_moved)
_changed_inputs('pad added', _pad_added)


# 7. In-place edits keep a pad list's identity and length, so only the pads'
# content can tell the board changed. Net 5 carries a wider class clearance;
# requests run at net 4's floor.
def _swap_across_classes(pcb, cfg):
    from pcb_modification import swap_pad_nets_in_pcb_data
    swap_pad_nets_in_pcb_data(pcb, pcb.pads_by_net[5][0], pcb.pads_by_net[4][2])


def _pad_moved_in_place(pcb, cfg):
    pcb.pads_by_net[5][0].global_x += 0.5


def _in_place(label, change):
    pcb = board()
    cfg = GridRouteConfig(layers=list(LAYERS))
    ncl = {5: 0.4}
    name = f'7 {label}: the next epoch rebuilds, equal to a fresh build'
    env_knobs.NET_MAP_VERIFY = True
    try:
        net_rescue._pristine_rescue_map(pcb, pcb, cfg, 2, ncl, ('full',))
        change(pcb, cfg)
        pcb.segments.append(make_seg(3.0, 2.0, 3.0, 12.0, net_id=2))
        bump_copper_epoch(pcb)
        got = net_rescue._pristine_rescue_map(pcb, pcb, cfg, 6, ncl, ('full',))
        fresh = obstacle_map.build_base_obstacle_map(pcb, cfg, [6], net_clearances=ncl)
        ok(name, digest(got) is not None and digest(got) == digest(fresh))
    except AssertionError as exc:
        ok(name, False, str(exc))
    finally:
        env_knobs.NET_MAP_VERIFY = False


_in_place('pad nets swapped across clearance classes', _swap_across_classes)
_in_place('pad moved in place', _pad_moved_in_place)


# 8. An input compared by identity must be held: once freed, a new object can
# take its address (#977). Free the board outline, allocate replacements with
# other bounds until one lands at the old address (or a fixed number of tries,
# when the cache holds the old one and none can), then route and bump.
def _id_reuse():
    pcb = board()
    cfg = GridRouteConfig(layers=list(LAYERS))
    name = '8: a new board_info at a freed address rebuilds, equal to a fresh build'
    env_knobs.NET_MAP_VERIFY = True
    try:
        net_rescue._pristine_rescue_map(pcb, pcb, cfg, 2, {}, ('full',))
        # the arguments first: only the instance may take the freed block
        kw = dict(layers={}, board_bounds=(0, 0, 1.5 * N + 3, 18),
                  copper_layers=list(LAYERS))
        made, reused = [], False
        old_id = id(pcb.board_info)
        pcb.board_info = None
        for _ in range(2000):
            bi = BoardInfo(**kw)
            made.append(bi)
            if id(bi) == old_id:
                reused = True
                break
        pcb.board_info = made[-1]
        del made
        pcb.segments.append(make_seg(3.0, 2.0, 3.0, 12.0, net_id=2))
        bump_copper_epoch(pcb)
        got = net_rescue._pristine_rescue_map(pcb, pcb, cfg, 4, {}, ('full',))
        fresh = obstacle_map.build_base_obstacle_map(pcb, cfg, [4], net_clearances={})
        ok(name, digest(got) is not None and digest(got) == digest(fresh),
           f'address reused: {reused}')
    except AssertionError as exc:
        ok(name, False, str(exc))
    finally:
        env_knobs.NET_MAP_VERIFY = False


_id_reuse()


# 9. Tracks edited in place without bump_copper_epoch: the shared base is
# served as is, so KICAD_NET_MAP_VERIFY=1 must name the broken contract.
def _no_bump():
    pcb = board()
    cfg = GridRouteConfig(layers=list(LAYERS))
    env_knobs.NET_MAP_VERIFY = True
    msg = ''
    try:
        net_rescue._pristine_rescue_map(pcb, pcb, cfg, 2, {}, ('full',))
        pcb.segments[0].end_y = 4.0
        net_rescue._pristine_rescue_map(pcb, pcb, cfg, 4, {}, ('full',))
    except AssertionError as exc:
        msg = str(exc)
    finally:
        env_knobs.NET_MAP_VERIFY = False
    ok('9: verify mode names copper edited without bump_copper_epoch',
       'bump_copper_epoch' in msg, msg)


_no_bump()


# 10. The escalation pops a power or impedance net's own width override from
# its rungs (`_escalation_ladder`), so that net's rungs differ in config from
# every other net's. Its bases must not evict the rungs the others share.
def _power_net_churn():
    pcb = board()
    base = GridRouteConfig(layers=list(LAYERS), power_net_widths={3: 0.5})
    widths = (0.2, 0.18, 0.16, 0.14)
    real, counted, calls = full_build_counter()
    obstacle_map.build_base_obstacle_map = counted
    try:
        for n in range(1, 6):
            pw = dict(base.power_net_widths)
            pw.pop(n, None)
            for w in widths:
                net_rescue._pristine_rescue_map(
                    pcb, pcb, _replace(base, track_width=w, power_net_widths=pw),
                    n, {}, ('full',))
    finally:
        obstacle_map.build_base_obstacle_map = real
    ok('10: the own rungs of a power net leave the shared rungs cached',
       len(calls) == 2 * len(widths), f'{len(calls)} builds, want {2 * len(widths)}')


_power_net_churn()

print(f'\n{len(fails)} failure(s)' if fails else '\nall passed')
sys.exit(1 if fails else 0)
