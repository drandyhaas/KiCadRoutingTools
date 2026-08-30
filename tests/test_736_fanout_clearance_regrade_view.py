"""#736: the post-nudge re-grade must see the copper the pass itself drew.

`repair_fanout_clearance` ends by asking `nudge_vias_for_unresolved` to move
offending vias, then re-grades to produce the final `unresolved` list. The
nudger does TWO things to the board: it moves vias, and it APPENDS new
connector `Segment`s to `pcb_data.segments` to restore continuity back to each
moved via's stub start.

The re-grade refreshed only the via view, and by pointing every cap at the
whole board list (spelled `refresh_cap_vias` since #775 gave that its own
method and made it re-prune).
`st.segments`, `st.cap_segs` and `st._seg_floor_by_id` were snapshotted in
`_Repair.__init__`, before those connectors existed, and nothing rebuilt them --
so `_seg_shortfalls`, the PAD-SEGMENT channel of `graze_penalty`, and
`required_rows`' disclosure both read a stale track view. Both front ends WRITE
that copper (placement/writer.py on the CLI, the plugin's pcbnew mirror in the
GUI), so the summary is a statement about a board that already carries it.

THE HEADLINE, measured on the rig below at `--clearance 0.1` and graded by the
AUTHORITATIVE checker rather than by the repair pass's own grader:

    arm B  through-hole cap pads, connector on B.Cu
      before   1 via move, 1 connector, `unresolved == []`      <- RESOLVED
      after    same move, same connector, `unresolved == ['C1']`
      check_drc.check_pad_segment_overlap(C1.2, the connector)
               -> violation, overlap 0.1200mm

That overlap IS the requirement: the connector's copper reaches 0.020mm INSIDE
the pad (0.130 centre-gap minus its 0.150 half-width), and the repair's own
grader charges the same 0.1200.

WHICH WINDOW THIS RUNS THROUGH, stated precisely because the obvious reading is
wrong. On the connector's OWN layer the draw gate and the accept gate are the
same number -- `connector_clear` charges `hw + req(pfl, cfl)` and `_seg_effs`
charges `halves[j] + _pair_or_flat(fa, floors[j])` with `halves[j] == hw`, the
same two floors through the same resolver -- and `connector_clear` is the
STRICTER of the two, by EPS. So an ACCEPTED same-layer connector can never be
charged here, and `TestTheDrawGateAndTheGraderAgreeOnTheSameLayer` measures
that rather than asserting it. Two windows remain:

  * arm B, the #738 window: `connector_clear` scopes a cap pad by the
    FOOTPRINT's side (`cl != layer`), so it never tests a back-side or
    inner-layer connector against a THROUGH-HOLE cap pad, whose copper spans
    every layer. This is a REAL violation -- check_drc agrees, above.
  * arm C, a window #738's own proposed fix does NOT close: a connector on a
    layer absent from `_all_cu` is charged by `_seg_shares` (which charges an
    unknown layer on purpose) while `connector_clear` skips it. Measured
    identical numbers to arm B with no through-hole pad anywhere -- BUT
    check_drc grades that pair CLEAN. It is the module's documented
    over-block, not a shipped violation, and its test says so.

An earlier draft of this file had that backwards and called arm C the durable
demonstration of a shipped violation. It is durable; it is not a violation.

WHY THE RIG IS SYNTHETIC. Measured over the 22 boards `git ls-files
kicad_files/*.kicad_pcb` returns, at `--clearance` 0.25 and 0.10: 2 run the
full pass, 1 reaches the via-nudger, and **0 emit a connector**. The one in-repo
configuration that does is the FALLBACK-OFF leg of the frozen sweep
`test_fanout_clearance.py` uses -- orangecrab_ext_pll at 0.10, 9 via moves and
17 connectors on B.Cu / F.Cu / In1.Cu / In2.Cu -- and after the fix all 17
enter the grader's view and NOTHING changes (`unresolved` 10 -> 10, every
per-cap shortfall `{}`). Both facts are asserted by
`TestInertOnTheTrackedCorpus`, the second so the first cannot be a statement
about unreachable code.

Which leg, precisely, because the obvious answer is wrong: that test runs both,
and `via_clear_fallback=False` is the load-bearing flag. With the fallback ON
the same board reports 0 unresolved, so the nudger is never called at all -- 0
moves, 0 connectors. `allow_rotations` makes no difference either way.
Measured, all four combinations.

THE ZONE IS THE TRICK. `conn_layers` can be fed by a same-net segment, a
via-in-pad pad, or a ZONE. A stub `Segment` terminating at the via necessarily
starts inside the cap's keep-out on its own layer and destroys the isolation; a
via-in-pad `Pad` enters `foreign_pads` and charges the pad channel.
`_Repair.__init__` reads `pcb_data.zones` NOWHERE -- the module's only read is
inside `nudge_vias_for_unresolved`'s pour-tie loop -- so a zone carries the
connector's LAYER in as a free parameter while leaving every keep-out channel
empty. The board therefore has ZERO segments until the connector exists, and
every arm's seed `seg_penalty` is exactly 0.0000.

Conventions this file follows (#697/#725/#731/#732/#733/#737 and CLAUDE.md):

  * REAL parser dataclasses, and a real file on disk -- `_Repair.__init__`
    reads courtyards, locked refs and the clearance model from `pcb_file`.
  * Every assertion names the single-line MUTATION that must kill it.
  * Assert you are ON THE BRANCH before asserting about it.
  * Every "is refused" is paired with a NEGATIVE CONTROL that still moves, and
    every "is charged" with one that is drawn and correctly NOT charged.
  * A refusal is asserted from the PRINTED OUTPUT before the counts -- a
    refusal that prints nothing is indistinguishable from a pass that never
    looked (#732's silent-failure lesson).
  * No fixture sits within 0.05mm of the quantity its assertion measures. The
    two gaps are 0.130 and 0.375 against a 0.250 keep-out, i.e. 0.120 and 0.125
    of margin. THREE tighter boundaries exist and are named here rather than
    left for a reviewer to find:
      - the landing (11.330) clears the VIA keep-out (11.300) by 0.030. That is
        the spiral's own 0.05mm radial quantum, and no assertion measures it --
        the landings are asserted EXACTLY, so they are change detectors for
        precisely that.
      - C2's probe pose sits 0.100 past the plateau edge at dy = 1.70 (where
        its pad's y-range starts covering the connector). Asserted on the
        plateau, at dy = 1.80.
      - the declaring arm's `t[5]` (0.350) and its `effs` cell (0.650) differ
        by 0.300 ON PURPOSE: without that inequality the two likeliest
        mutations -- dropping `_item_reach`, and omitting the floor map --
        cancel and both survive.

MUTATION BATTERY: 22 single-line mutations of the fix, 22 killed, each named
by the arm that kills it under `TestOneConstructionSiteOnePrunePredicate`. Two
false-positive probes -- a trailing comment naming a guarded literal, and a
comment line naming the pricing spelling -- are correctly inert.

Four of the twenty-two (19-22) exist because an adversarial review of this
branch found a defect the fix INTRODUCED: `required_rows` grades the seed pose
as well as the final one, and a connector the pass DREW did not exist at the
seed, so charging it there invented a `track <net>` row describing a pair no
board ever had -- the same class of phantom #731 removed from that very report.
`TestTheSeedHalfOfTheDisclosureIgnoresPassCreatedCopper` is that finding.

Two more exist because the FIRST run of the battery let them through, and both
are recorded rather than quietly fixed:

  * `if via_moves:` -> `if True:` changes no NUMBER, because the via prune is
    exact and re-pruning and de-pruning grade identically. This mutation has
    now escaped TWO spellings of its killer. The first version of
    `TestTheRefreshIsSkippedWhenNothingMoved` asserted only the track half
    and it survived. The second caught it on `cap_vias[ref] is not st.vias`,
    which #775 made VACUOUS -- a re-pruning refresh never yields the board
    list either -- measured, not predicted: with #775 in and the guard
    mutated, this file passed. It is now caught on the identity of the list
    `__init__` built, snapshotted at the seed frame through `on_move`, with
    a positive control beside it so a DELETED refresh cannot satisfy it.
    The consequence it guards is unchanged: an eff-matrix rebuild of
    n_pads x n_ALL_vias per cap, on every board that reaches the nudger and
    moves nothing, i.e. every tracked board at the shipped defaults.
  * anchoring the prune on `cap.rect()` instead of `self._cap_geom` survived
    because every arm here freezes the caps (`max_displacement=0.0`), so no
    cap ever moves. `TestThePruneIsAnchoredOnTheSEEDPose` moves one by hand.

ONE mutation is DECLARED unkillable rather than papered over: `if via_moves:`
-> `if new_segs:`. A via move whose `conn_layers` is empty registers nothing
either way, so no arm can separate them -- recorded as test_737 records its own
surviving spelling.

Runtime ~40s, in-process apart from one `git ls-files`.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
import json
import os
import re
import sys
import tempfile
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import routing_defaults as defaults
import run_utils
from check_drc import check_pad_segment_overlap
from kicad_parser import BoardInfo, Footprint, Zone, parse_kicad_pcb
from placement import fanout_clearance as FC
from placement.fanout_clearance import _Repair, repair_fanout_clearance
from synth import make_net, make_pad, make_pcb, make_via

# --- the rig, and every number it is worth ---------------------------------
CLEAR = 0.1                 # place_fanout_clearance.py's own --help example
CU = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
BOUNDS = (0.0, 0.0, 60.0, 20.0)
TRACK_W = defaults.TRACK_WIDTH      # 0.3 -- the width the pour-tie path falls
HW = TRACK_W / 2.0                  # back to; imported, never mirrored
V_SIZE, V_DRILL = 0.8, 0.3
VR = V_SIZE / 2.0                   # 0.40

NET_V = 3                   # the foreign net the via and its connector carry
NET_A, NET_B = 11, 12       # C1's two pads
CAP_XY = (10.0, 10.0)
CAP2_XY = (10.0, 8.0)
BGA_XY = (7.6, 10.0)

PAD_HALF = 0.3              # 0.6mm square pads at local +-0.5 ...
PAD2_EDGE = CAP_XY[0] + 0.5 + PAD_HALF          # ... so pad 2's right edge

SEG_KEEPOUT = HW + CLEAR                        # 0.25, what the GRADER charges
VIA_KEEPOUT = VR + CLEAR                        # 0.40, what valid_via_pos needs

VIA_DEEP = 10.930           # gap 0.130 -> charged SEG_KEEPOUT - 0.130
VIA_CLEAR = 11.175          # gap 0.375 -> clear by 0.125
GAP_DEEP = VIA_DEEP - PAD2_EDGE                 # 0.130
CHARGE = SEG_KEEPOUT - GAP_DEEP                 # 0.1200

LAND_DEEP = (11.330, 10.0)
LAND_CLEAR = (11.325, 10.0)
# The connector runs from the via's OLD position to its new one, so its
# start is VIA_CLEAR itself. Spelled that way rather than as
# `LAND_CLEAR[0] - 0.15` -- the same point in arithmetic, and one ULP
# below it in float.
CONN_CLEAR = (VIA_CLEAR, CAP_XY[1], LAND_CLEAR[0], CAP_XY[1])

# The per-cap track prune keeps anything within
#   max_displacement_cap + 2*span + clearance   (+ the track's own over-reach)
# of the cap's SEED centre: 5.058801mm here (3.0 + 2*0.854400 + 0.1 + 0.25).
# FAR_D is inside it and FAR_D - WALK_BACK is outside it, both by more than the
# 0.05 this file allows itself, so a prune anchored on the MOVED pose drops a
# track the seed-anchored one keeps.
FAR_D = 4.90
FAR_SEG = (CAP_XY[0] + FAR_D, CAP_XY[1],
           CAP_XY[0] + FAR_D + 0.15, CAP_XY[1])
WALK_BACK = -0.30

# C1 walked +0.30 in x puts its pad edge 0.075 from the connector.
PROBE_C1 = (0.30, 0.0)
PROBE_C2 = (0.30, 1.80)
PROBE_CHARGE = 0.1750

DECL_CLASS = [{'name': 'Default', 'clearance': 0.2, 'track_width': 0.2,
               'via_diameter': 0.6, 'via_drill': 0.4,
               'priority': 2147483647}]
DECL_LC = 0.5               # a pad override, so pair() != _item_reach()
DECL_T5 = 0.35              # HW + _item_reach(seg floor)  = 0.15 + max(0.1,0.2)
DECL_EFF = 0.65             # HW + pair(cap floor, seg floor) = 0.15 + 0.5

# A connector hugging C1's SEED pads: 0.100 off pad 2's seed edge against the
# 0.650 declared keep-out, so it IS charged there -- and 1.900 clear of the
# pads once the cap has walked SEED_HUG_WALK away, so it is NOT charged at the
# pose the pass left it in. That asymmetry is the whole of the seed-half
# disclosure defect an adversarial review found in this branch.
SEED_HUG = {'start': (10.90, CAP_XY[1]), 'end': (11.05, CAP_XY[1]),
            'width': TRACK_W, 'layer': 'F.Cu', 'net_id': NET_V}
SEED_HUG_WALK = 2.5


def _bga():
    """detect_package_type keys on the footprint NAME, so a 3x3 grid whose
    name says BGA is enough to give _Repair a ball field to sit caps near."""
    pads = []
    for i, dx in enumerate((-0.8, 0.0, 0.8)):
        for j, dy in enumerate((-0.8, 0.0, 0.8)):
            pads.append(make_pad(net_id=100 + 3 * i + j,
                                 x=BGA_XY[0] + dx, y=BGA_XY[1] + dy,
                                 ref='U1', num='%d%d' % (i, j),
                                 size_x=0.4, size_y=0.4, shape='circle',
                                 layers=['F.Cu'], local_x=dx, local_y=dy))
    return Footprint(reference='U1', footprint_name='lib:BGA-9_3x3_0.8mm',
                     x=BGA_XY[0], y=BGA_XY[1], rotation=0.0, layer='F.Cu',
                     pads=pads)


def _cap(ref, cx, cy, through=False):
    """A C-prefixed 2-copper-pad part -- what _Repair's cap detection admits.

    `through` is the ONE variable that separates arm B from its control: the
    pads become `*.Cu` plated through-hole, so their copper spans every layer
    while the FOOTPRINT still sits on F.Cu. That is the whole of the #738
    window, expressed as one keyword.
    """
    kw = dict(size_x=2 * PAD_HALF, size_y=2 * PAD_HALF, shape='rect')
    if through:
        kw.update(layers=['*.Cu', '*.Mask'], drill=0.3, pad_type='thru_hole')
    else:
        kw.update(layers=['F.Cu'])
    pads = [make_pad(net_id=NET_A, x=cx - 0.5, y=cy, ref=ref, num='1',
                     local_x=-0.5, local_y=0.0, **kw),
            make_pad(net_id=NET_B, x=cx + 0.5, y=cy, ref=ref, num='2',
                     local_x=0.5, local_y=0.0, **kw)]
    return Footprint(reference=ref, footprint_name='lib:C_0402_1005Metric',
                     x=cx, y=cy, rotation=0.0, layer='F.Cu', pads=pads)


def _board(via_x, zone_layer, through=False, second_cap=False, lc=0.0):
    bi = BoardInfo(layers={}, copper_layers=list(CU), board_bounds=BOUNDS)
    v = make_via(via_x, CAP_XY[1], net_id=NET_V, size=V_SIZE, drill=V_DRILL)
    fps = {'U1': _bga(), 'C1': _cap('C1', *CAP_XY, through=through)}
    if second_cap:
        fps['C2'] = _cap('C2', *CAP2_XY, through=through)
    if lc:
        for p in fps['C1'].pads:
            p.local_clearance = lc
    zone = Zone(net_id=NET_V, net_name='N3', layer=zone_layer,
                polygon=[(via_x - 1.0, CAP_XY[1] - 1.0),
                         (via_x + 1.0, CAP_XY[1] - 1.0),
                         (via_x + 1.0, CAP_XY[1] + 1.0),
                         (via_x - 1.0, CAP_XY[1] + 1.0)])
    pads_by_net = {}
    for fp in fps.values():
        for p in fp.pads:
            pads_by_net.setdefault(p.net_id, []).append(p)
    nets = {0: make_net(0, '')}
    for k in list(pads_by_net) + [NET_V]:
        nets.setdefault(k, make_net(k, 'N%d' % k))
    pcb = make_pcb(board_info=bi, nets=nets, vias=[v], segments=[],
                   footprints=fps, pads_by_net=pads_by_net, zones=[zone])
    return pcb, v


@contextlib.contextmanager
def _stub(classes=None):
    """A bare board FILE so the path-reading collaborators degrade cleanly
    (courtyards -> {}, locked refs -> set(), PadClearanceModel -> INERT unless
    `classes` declares one). #617's idiom."""
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with io.open(path, 'w', encoding='utf-8') as f:
            f.write('(kicad_pcb (version 20240108))\n')
        if classes is not None:
            with io.open(os.path.join(td, 'b.kicad_pro'), 'w',
                         encoding='utf-8') as f:
                json.dump({'net_settings': {'classes': classes,
                                            'netclass_assignments': {},
                                            'netclass_patterns': []}}, f)
        yield path


def _run(pcb, path, **kw):
    """Drive the WHOLE pass, capturing stdout and the internal `_Repair`.

    `on_move` is the supported hook the animator uses: it is handed the state
    at the seed frame, and every frame is the SAME object, so it is the only
    way to see cap_segs / segments / _seg_floor_by_id from outside without
    monkeypatching.

    max_displacement=0.0 with max_passes=1 is the BOXED cap of #313 expressed
    as a budget rather than as a wall of obstacle geometry -- the same rig
    tests/test_fanout_clearance.py's via-clear-fallback check uses.

    THE FOURTH RETURN is the SEED-FRAME snapshot of the per-cap via lists
    (#775). `on_move` fires at the seed placement, BEFORE the nudge, with the
    live object, so it is the only way to see what __init__ built. It holds
    the LIST OBJECTS and not their ids: the refresh frees what it replaces,
    and CPython recycles a freed list's id readily, so an id-keyed snapshot
    can report "unchanged" about two different objects.
    """
    seen, pre, buf = [], {}, io.StringIO()

    def _hook(st):
        seen.append(st)
        if len(seen) == 1:
            pre.update({r: st.cap_vias[r] for r in st.caps})

    with contextlib.redirect_stdout(buf):
        res = repair_fanout_clearance(
            pcb, path, clearance=CLEAR, cap_prefix='C',
            max_displacement=0.0, max_passes=1, allow_rotations=False,
            via_clear_fallback=False, on_move=_hook, **kw)
    return res, (seen[0] if seen else None), buf.getvalue(), pre


def _arm(via_x, zone_layer, **kw):
    with _stub(kw.pop('classes', None)) as path:
        pcb, v = _board(via_x, zone_layer, **kw)
        res, st, out, pre = _run(pcb, path)
        return pcb, v, res, st, out, pre


class TestTheDrawGateAndTheGraderAgreeOnTheSameLayer(unittest.TestCase):
    """The measurement that LICENSES this file's framing: on the connector's
    own layer the two gates are the same number, so #736 cannot ship a false
    RESOLVED there. Without this, every other arm reads as a lucky fixture."""

    def test_a_same_layer_connector_in_the_keepout_is_REFUSED_at_draw_time(self):
        pcb, v, res, st, out, _ = _arm(VIA_DEEP, 'F.Cu')
        # ON THE BRANCH: the cap really is a violator, so the nudger really ran
        self.assertIn('Caps initially grazing foreign copper', out)
        self.assertEqual(sorted(st.caps), ['C1'],
                         'U1 leaked into the cap set; cap_prefix is wrong')
        self.assertLess(GAP_DEEP, SEG_KEEPOUT - 0.05,
                        'the fixture no longer sits inside the keep-out')
        # the PRINTED refusal first (#732): a refusal that prints nothing is
        # indistinguishable from a pass that never looked
        self.assertIn("via-nudge: no clear spot for C1's offending via", out)
        self.assertEqual(res['via_moves'], [])
        self.assertEqual(res['new_segments'], [])
        self.assertEqual((v.x, v.y), (VIA_DEEP, CAP_XY[1]))
        self.assertEqual(len(pcb.segments), 0)
    # MUTATION: none -- this arm licenses the claim rather than testing the
    # fix, and it fails loudly if the draw gate is ever weakened.

    def test_the_NEGATIVE_control_with_room_still_moves(self):
        pcb, v, res, st, out, _ = _arm(VIA_CLEAR, 'F.Cu')
        self.assertIn('via-nudge: moved', out)
        self.assertEqual(len(res['via_moves']), 1)
        self.assertEqual((round(v.x, 4), round(v.y, 4)), LAND_CLEAR)
    # MUTATION: none (control).

    def test_the_two_gates_resolve_to_the_SAME_requirement(self):
        """Arithmetic, not geometry: `connector_clear` charges
        `hw + req(pfl, cfl)` and `_seg_effs` charges `halves + pair(fa, fb)`
        from the same two floors. Compared on a DECLARING board, where both
        are free to be wrong differently."""
        with _stub(DECL_CLASS) as path:
            pcb, v = _board(VIA_CLEAR, 'F.Cu', lc=DECL_LC)
            st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C',
                         set())
            self.assertIsNotNone(st._floors, 'the model is INERT -- this arm '
                                             'would compare 0.1 with 0.1')
            cap = st.caps['C1']
            st.register_new_segments([{'start': CONN_CLEAR[:2],
                                       'end': CONN_CLEAR[2:],
                                       'width': TRACK_W, 'layer': 'F.Cu',
                                       'net_id': NET_V}])
            cfl = st.seg_floor(NET_V, 'F.Cu')
            drawn = HW + st.required(cap.pad_floors[1], cfl)
            graded = st._seg_effs('C1', cap)[1][-1]
            self.assertAlmostEqual(drawn, graded, places=12)
            self.assertAlmostEqual(graded, DECL_EFF, places=9)
    # MUTATION: price the tuple at `HW + self.clearance` -> graded 0.25 != drawn.


class TestTheConnectorEntersTheGradersView(unittest.TestCase):
    """LOAD-BEARING, and deliberately built on a connector the draw gate
    genuinely ACCEPTS on the cap's own layer. Nothing here depends on #738 or
    on any layer-scope divergence, so #738 landing cannot touch it."""

    def setUp(self):
        self.pcb, self.v, self.res, self.st, self.out, _ = _arm(
            VIA_CLEAR, 'F.Cu', second_cap=True)
        # ON THE BRANCH: the pass reached the nudger and really drew copper
        self.assertEqual(len(self.res['via_moves']), 1, self.out)
        self.assertEqual([(s['layer'], s['width'])
                          for s in self.res['new_segments']],
                         [('F.Cu', TRACK_W)])
        self.assertIn('via-nudge: moved', self.out)
        self.seg = self.pcb.segments[-1]

    def test_the_track_view_gained_exactly_the_connector(self):
        self.assertEqual(len(self.st.segments), len(self.pcb.segments))
        self.assertEqual(len(self.st.segments), 1)
    # MUTATION: drop the register_new_segments call -> 0 != 1.

    def test_the_tuple_is_priced_and_layered_as_init_would_have(self):
        t = self.st.cap_segs['C1'][-1]
        self.assertEqual(t[:5], (CONN_CLEAR[0], CONN_CLEAR[1], CONN_CLEAR[2],
                                 CONN_CLEAR[3], NET_V))
        self.assertEqual(t[6], 'F.Cu',
                         'element 6 is a board SIDE again -- the pre-#731 '
                         'collapse')
        self.assertAlmostEqual(
            t[5], self.seg.width / 2.0 + self.st._item_reach(
                self.st.seg_floor(self.seg.net_id, self.seg.layer)), places=12)
        self.assertAlmostEqual(t[5], SEG_KEEPOUT, places=9)
    # MUTATION: `t[6] = _seg_side(layer)`; drop `_item_reach`; drop the `/2`.

    def test_the_floor_map_holds_no_orphan_id(self):
        live = {id(t) for t in self.st.segments}
        self.assertLessEqual(set(self.st._seg_floor_by_id), live,
                             'an entry keys a tuple that is not in the live '
                             'list -- a recycled id would return ANOTHER '
                             "track's floor, silently")
    # MUTATION: build the tuple without filing it in self.segments.

    def test_every_cap_in_reach_was_refreshed_with_a_NEW_list(self):
        for ref in ('C1', 'C2'):
            self.assertEqual(len(self.st.cap_segs[ref]), 1,
                             '%s did not keep the connector' % ref)
        effs = self.st._seg_effs('C1', self.st.caps['C1'])
        self.assertTrue(all(len(row) == len(self.st.cap_segs['C1'])
                            for row in effs),
                        'the _cap_seg_eff memo is one column short -- '
                        'cap_segs was appended to IN PLACE, so its identity '
                        'never changed and the memo was never rebuilt')
    # MUTATION: `cap_segs[ref].append(t)`; refresh only the repaired cap.

    def test_the_pass_did_not_INVENT_a_charge_for_clear_copper(self):
        for ref in ('C1', 'C2'):
            cap = self.st.caps[ref]
            self.assertAlmostEqual(
                self.st.seg_penalty(ref, cap, cap.x, cap.y, cap.rot), 0.0,
                places=9, msg='%s is charged for a connector the draw gate '
                              'accepted -- the fix OVER-blocks' % ref)
        self.assertEqual(self.res['unresolved'], [])
    # MUTATION: charge the tuple at the prune over-reach instead of the pair.

    def test_graze_penalty_RISES_for_a_pose_inside_the_connectors_keepout(self):
        """The behavioural half, with no layer-scope dependence at all: walk
        each cap toward the connector and the charge must appear."""
        for ref, (dx, dy) in (('C1', PROBE_C1), ('C2', PROBE_C2)):
            cap = self.st.caps[ref]
            self.assertAlmostEqual(
                sum(self.st._seg_shortfalls(
                    ref, cap, cap.x + dx, cap.y + dy, cap.rot).values()),
                PROBE_CHARGE, places=4,
                msg='%s: the connector is not in the view, or is priced wrong'
                    % ref)
    # MUTATION: any of the above; also "build from st.segments instead of the
    # nudger's dicts" (nothing to build from) and "call it BEFORE the nudge".


class TestTheStaleViewShipsAFalseRESOLVED(unittest.TestCase):
    """THE HEADLINE. A through-hole cap pad and a back-side connector: the
    draw gate never tests the pair (#738), the grader charges it, and
    check_drc -- the authority -- agrees the copper really is in violation."""

    def setUp(self):
        self.pcb, self.v, self.res, self.st, self.out, _ = _arm(
            VIA_DEEP, 'B.Cu', through=True)

    def test_the_connector_is_DRAWN_and_now_CHARGED(self):
        # draw side FIRST, and exactly -- see the CHANGE DETECTOR note below
        self.assertEqual([(s['layer'], s['width'])
                          for s in self.res['new_segments']],
                         [('B.Cu', TRACK_W)],
                         'the off-side connector was NOT drawn. If #738 has '
                         'landed that is CORRECT: delete this class, keep '
                         'TestTheConnectorEntersTheGradersView, and record in '
                         'the #738 PR that #736\'s behavioural window closed.')
        self.assertEqual((round(self.v.x, 4), round(self.v.y, 4)), LAND_DEEP)
        cap = self.st.caps['C1']
        self.assertAlmostEqual(
            self.st.seg_penalty('C1', cap, cap.x, cap.y, cap.rot), CHARGE,
            places=4)
        self.assertEqual(self.res['unresolved'], ['C1'])
    # MUTATION: delete the refresh -> unresolved == [] and the pass reports a
    # cap RESOLVED with a connector 0.020mm INSIDE one of its pads.

    def test_check_drc_agrees_it_is_a_real_violation(self):
        """Not the repair pass's own arithmetic: the authoritative checker, at
        margin 0 so the number is the requirement and not a tolerance band."""
        pad2 = self.pcb.footprints['C1'].pads[1]
        viol, overlap, _pt = check_pad_segment_overlap(
            pad2, self.pcb.segments[-1], CLEAR, CU, clearance_margin=0.0)
        self.assertTrue(viol, 'check_drc grades this pair clean -- the fix is '
                              'reporting a phantom, not a violation')
        self.assertAlmostEqual(overlap, CHARGE, places=4)
    # MUTATION: none -- this is the arm that makes the headline a fact rather
    # than a claim about one grader agreeing with itself.

    def test_the_NEGATIVE_control_SMD_pads_is_drawn_and_NOT_charged(self):
        """One variable away from the arm above: the same B.Cu connector
        against SMD F.Cu pads, which share no copper with it."""
        pcb, v, res, st, out, _ = _arm(VIA_DEEP, 'B.Cu', through=False)
        self.assertEqual([(s['layer'], s['width'])
                          for s in res['new_segments']], [('B.Cu', TRACK_W)])
        cap = st.caps['C1']
        self.assertAlmostEqual(st.seg_penalty('C1', cap, cap.x, cap.y,
                                              cap.rot), 0.0, places=9)
        self.assertEqual(res['unresolved'], [])
        pad2 = pcb.footprints['C1'].pads[1]
        viol, _ov, _pt = check_pad_segment_overlap(
            pad2, pcb.segments[-1], CLEAR, CU, clearance_margin=0.0)
        self.assertFalse(viol)
    # MUTATION: drop the layer gate from the rebuilt prune -> charged, and
    # check_drc says it should not be.


class TestTheLayerScopeSurvivesTheRebuild(unittest.TestCase):
    """#731's regression guard, and the ONLY arm that kills one mutation."""

    def test_a_DECLARED_layer_the_cap_pads_do_not_share_is_excluded(self):
        pcb, v, res, st, out, _ = _arm(VIA_DEEP, 'In1.Cu')
        # the connector IS drawn and IS on the board ...
        self.assertEqual([(s['layer'], s['width'])
                          for s in res['new_segments']], [('In1.Cu', TRACK_W)])
        self.assertEqual(len(st.segments), 1)
        self.assertIn('In1.Cu', st._all_cu, 'the layer is UNKNOWN to the '
                                            'board, so _seg_shares charges it '
                                            'and this arm tests nothing')
        # ... and the F.Cu SMD cap pads share no copper with it
        self.assertEqual(len(st.cap_segs['C1']), 0)
        cap = st.caps['C1']
        self.assertAlmostEqual(st.seg_penalty('C1', cap, cap.x, cap.y,
                                              cap.rot), 0.0, places=9)
        self.assertEqual(res['unresolved'], [])
    # MUTATION: `t[6] = _seg_side(s.layer)` -- the pre-#731 collapse. 'F' is
    # not in _all_cu, so _seg_shares charges it and a PHANTOM appears. This is
    # the only arm that catches it: on the F.Cu and B.Cu arms the collapse is
    # numerically invisible.


class TestTheUnknownLayerOverBlockIsDELIBERATE(unittest.TestCase):
    """Arm C. A connector on a layer the board never declared is charged --
    by policy, not by accident (`_seg_shares`: "over-blocking is the only
    direction that can never ship a violation").

    It is pinned HERE, apart from the headline, because check_drc grades the
    pair CLEAN. Reading it as a violation is the mistake an earlier draft of
    this file made."""

    def test_it_is_charged_even_though_the_checker_calls_it_clean(self):
        pcb, v, res, st, out, _ = _arm(VIA_DEEP, 'In3.Cu')
        self.assertNotIn('In3.Cu', st._all_cu,
                         'In3.Cu became a declared layer; this arm now tests '
                         'the same thing as the In1.Cu one')
        self.assertEqual([(s['layer'], s['width'])
                          for s in res['new_segments']], [('In3.Cu', TRACK_W)])
        cap = st.caps['C1']
        self.assertAlmostEqual(st.seg_penalty('C1', cap, cap.x, cap.y,
                                              cap.rot), CHARGE, places=4)
        self.assertEqual(res['unresolved'], ['C1'])
        # ... and the authority disagrees, which is the point of this class
        pad2 = pcb.footprints['C1'].pads[1]
        viol, _ov, _pt = check_pad_segment_overlap(
            pad2, pcb.segments[-1], CLEAR, CU, clearance_margin=0.0)
        self.assertFalse(viol, 'check_drc now flags this -- the over-block '
                               'has become a real rule and this class should '
                               'be re-read as a violation arm')
    # MUTATION: drop the `_seg_shares` fallback from the rebuilt prune.


class TestTheRebuildPricesTheConnectorCorrectly(unittest.TestCase):
    """A DECLARING board, where `t[5]` and the eff cell are free to differ --
    0.350 against 0.650 here. Without that inequality the two likeliest
    mutations cancel exactly and both survive."""

    def _st(self, path, pcb):
        st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C', set())
        st.register_new_segments([{'start': CONN_CLEAR[:2],
                                   'end': CONN_CLEAR[2:], 'width': TRACK_W,
                                   'layer': 'F.Cu', 'net_id': NET_V}])
        return st

    def test_the_two_prices_are_separated_and_both_correct(self):
        with _stub(DECL_CLASS) as path:
            pcb, v = _board(VIA_CLEAR, 'F.Cu', lc=DECL_LC)
            st = self._st(path, pcb)
            self.assertIsNotNone(st._floors, 'the model is INERT')
            t = st.segments[-1]
            self.assertAlmostEqual(t[5], DECL_T5, places=9)
            self.assertAlmostEqual(st._seg_effs('C1', st.caps['C1'])[1][-1],
                                   DECL_EFF, places=9)
            self.assertGreater(abs(DECL_EFF - DECL_T5), 0.05,
                               'the two prices coincide, so a mutation that '
                               'swaps them survives')
    # MUTATION: drop `_item_reach` -> t[5] 0.25; omit the floor map -> the eff
    # cell moves.

    def test_the_floor_filed_is_the_TRACK_floor_object_itself(self):
        """Identity, not value: `_seg_floor_for` memoises per (net, layer), so
        the right floor is the same OBJECT. A via floor is scoped to ALL
        copper and would diverge only under a layer-scoped .kicad_dru rule --
        which no behavioural arm here has, so this is the only killer."""
        with _stub(DECL_CLASS) as path:
            pcb, v = _board(VIA_CLEAR, 'F.Cu', lc=DECL_LC)
            st = self._st(path, pcb)
            t = st.segments[-1]
            fl = st.seg_floor(NET_V, 'F.Cu')
            self.assertIsNotNone(fl, 'an inert board makes this None is None')
            self.assertIs(st._seg_floor_by_id.get(id(t)), fl)
    # MUTATION: file `_via_floor_for(net)` instead of `_seg_floor_for(net,
    # layer)` -- behaviourally invisible without a dru rule.

    def test_an_INERT_board_files_no_floor_at_all(self):
        with _stub() as path:
            pcb, v = _board(VIA_CLEAR, 'F.Cu')
            st = self._st(path, pcb)
            self.assertIsNone(st._floors)
            t = st.segments[-1]
            self.assertNotIn(id(t), st._seg_floor_by_id)
            self.assertAlmostEqual(t[5], SEG_KEEPOUT, places=9)
    # MUTATION: file the floor unconditionally (drop `if fl is not None`).


class TestTheRefreshIsSkippedWhenNothingMoved(unittest.TestCase):
    """The refresh is guarded, and the guard is not decorative: EVERY tracked
    board that reaches the nudger at the shipped defaults makes zero moves."""

    def test_a_run_that_moved_no_via_leaves_the_track_view_alone(self):
        with _stub() as path:
            pcb, v = _board(VIA_DEEP, 'F.Cu')
            st_probe = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3,
                               'C', set())
            before = {r: st_probe.cap_segs[r] for r in st_probe.caps}
            segs_before = st_probe.segments

            res, st, out, pre = _run(pcb, path)
        self.assertIn('no clear spot', out)
        self.assertEqual(res['via_moves'], [])
        # the run built its OWN _Repair, so compare the live one's identities
        self.assertEqual(len(st.segments), 0)
        self.assertTrue(pre, 'the seed frame never fired, so every identity '
                             'assertion below is vacuous')
        for ref in st.caps:
            self.assertEqual(st.cap_segs[ref], [])
            # ...and the VIA half of the refresh did not run either. This is
            # the only observable consequence of an unguarded refresh, and
            # since #775 it is no longer `is not st.vias`: the refresh
            # RE-prunes, so a pruned list is not the board list whether it ran
            # or not, and that spelling went vacuous. MEASURED, not predicted
            # -- with #775 in and the guard mutated to `if True:`, this file
            # passed. What still discriminates is the identity of the list
            # __init__ built: the refresh ASSIGNS a new one per cap,
            # unconditionally, and every _cap_via_eff memo dies with the old
            # one. That is the cost the guard exists to avoid -- an eff-matrix
            # rebuild of n_pads x n_ALL_vias per cap on every board that
            # reaches the nudger and moves nothing, which is every tracked
            # board at the shipped defaults.
            self.assertIs(st.cap_vias[ref], pre[ref],
                          'the per-cap via list for %s was REBUILT on a run '
                          'that moved NO via -- the refresh is unguarded'
                          % ref)
        self.assertEqual(len(segs_before), 0)
        self.assertEqual(sorted(before), sorted(st.caps))
    # MUTATION: `if via_moves:` -> `if True:`. It changes no number -- the
    # via prune is exact, so re-pruning and de-pruning both grade the same --
    # which is why the assertion above is on list IDENTITY. The first version
    # of this class asserted only the track half and the mutation SURVIVED
    # the battery; the second asserted `is not st.vias`, which #775 made
    # vacuous.

    def test_a_run_that_DID_move_a_via_rebuilds_every_cap_via_list(self):
        """POSITIVE CONTROL for the arm above (#775). Without it, a refresh
        that had been deleted outright would satisfy every identity assertion
        in this class -- the lists would be untouched because nothing touches
        them -- and the guard would read as held."""
        pcb, v, res, st, out, pre = _arm(VIA_CLEAR, 'F.Cu', second_cap=True)
        self.assertTrue(res['via_moves'], out)
        self.assertTrue(pre)
        for ref in st.caps:
            self.assertIsNot(st.cap_vias[ref], pre[ref],
                             'cap %s kept its construction-time list through '
                             'a run that DID move a via' % ref)
            self.assertIsNot(st.cap_vias[ref], st.vias,
                             'the refresh de-pruned instead of re-pruning')
    # MUTATION: delete the refresh call.


class TestTheSeedHalfOfTheDisclosureIgnoresPassCreatedCopper(unittest.TestCase):
    """`required_rows` grades `_seg_shortfalls` at the SEED pose as well as the
    final one -- deliberately, so that a pass which SUCCEEDS still discloses
    the raised requirement that did the work.

    A connector this pass DREW did not exist at the seed. Charging it there
    invents a `track <net>` row describing a pair no board ever had, which is
    exactly the class of phantom #731 removed from this very report. Measured
    on the rig below before the scope was added: `{3: 0.550}` charged at the
    seed against `{}` at the final pose -- 0.650 (the declared eff cell) minus
    the 0.100 the connector sits off pad 2's SEED edge.

    Found by an adversarial review of this branch, not by me.
    """

    def test_a_connector_hugging_the_SEED_pads_produces_no_track_row(self):
        with _stub(DECL_CLASS) as path:
            pcb, v = _board(VIA_CLEAR, 'F.Cu', lc=DECL_LC)
            st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C',
                         set())
            self.assertIsNotNone(st._floors, 'an INERT board discloses '
                                             'nothing, so this arm would pass '
                                             'on an empty report')
            cap = st.caps['C1']
            seed_n = st._seed_seg_n['C1']
            cap.x = cap.seed_x - SEED_HUG_WALK      # the pose the pass left
            self.assertEqual(st.register_new_segments([SEED_HUG]), 1)
            self.assertEqual(len(st.cap_segs['C1']), seed_n + 1)

            # ON THE BRANCH: the fixture really does reach the phantom -- the
            # connector is charged at the seed pose over ALL copper, and clear
            # at the pose the cap is actually in.
            self.assertEqual(
                st._seg_shortfalls('C1', cap, cap.x, cap.y, cap.rot), {})
            self.assertIn(
                NET_V, st._seg_shortfalls('C1', cap, cap.seed_x, cap.seed_y,
                                          cap.seed_rot))
            # ...and scoped to the seed-era copper it is not
            self.assertEqual(
                st._seg_shortfalls('C1', cap, cap.seed_x, cap.seed_y,
                                   cap.seed_rot, upto=seed_n), {})

            tracks = [r for r in st.required_rows({})
                      if str(r[1]).startswith('track ')]
            self.assertEqual(tracks, [],
                             'the disclosure names a track pair that exists '
                             'on neither the seed board nor the final one')
    # MUTATION: drop the `upto` slice from _seg_shortfalls; record
    # _seed_seg_n AFTER the append instead of in __init__; pass no seed_kw to
    # both() for the track kind.


class TestThePruneIsAnchoredOnTheSEEDPose(unittest.TestCase):
    """By the time a connector is registered the caps have already moved, so
    `_prune_segs` is handed `self._cap_geom` -- the SEED-pose geometry -- and
    not `cap.rect()`.

    That is not a stylistic choice. The reach bound is the reachable-DISK
    argument (a cap moves at most `max_displacement_cap` from its SEED), which
    is what makes the prune EXACT rather than approximate; and
    `required_rows` grades `_seg_shortfalls` at the SEED pose as well as the
    final one, so a moved-pose radius is a superset for neither.
    """

    def test_a_track_in_reach_of_the_SEED_survives_a_cap_that_moved_away(self):
        with _stub() as path:
            pcb, v = _board(VIA_CLEAR, 'F.Cu')
            st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C',
                         set())
            cap = st.caps['C1']
            _cx, _cy, span, _r = st._cap_geom['C1']
            thr = (st._max_disp_cap + 2 * span + st.clearance
                   + max(0.0, cap.max_floor - st.clearance)
                   + HW + st.clearance)
            # ON THE BRANCH: the fixture straddles the real threshold, both
            # ways, by more than the 0.05mm this file allows itself
            self.assertGreater(thr - FAR_D, 0.05, 'the track is not in reach '
                                                  'of the seed after all')
            self.assertGreater((FAR_D - WALK_BACK) - thr, 0.05,
                               'the moved pose still reaches it, so a '
                               'moved-pose prune would keep it too')
            cap.x += WALK_BACK              # the pose the sweep left it in
            self.assertNotEqual(cap.x, cap.seed_x)

            st.register_new_segments([{'start': FAR_SEG[:2],
                                       'end': FAR_SEG[2:], 'width': TRACK_W,
                                       'layer': 'F.Cu', 'net_id': NET_V}])
            self.assertEqual(
                len(st.cap_segs['C1']), 1,
                'the connector was pruned away: the reach was measured from '
                "the cap's CURRENT pose, not its seed, which silently "
                'redefines what "exact" means for this prune')
    # MUTATION: recompute (ccx, ccy, span) from cap.rect() in the registrar
    # instead of reading self._cap_geom.


class TestOneConstructionSiteOnePrunePredicate(unittest.TestCase):
    """SOURCE GUARD. #736 exists because both halves of "how a track becomes a
    graded tuple" were inline in `__init__` and existed nowhere else, so
    nothing could register a track that appeared later. Adding a second
    spelling is that defect again, and no fixture can catch it.

    CODE only -- `l.split('#')[0]` drops a trailing comment as well as a
    full-line one, because the engine's comment blocks name every one of these
    symbols repeatedly. Reading raw lines both invents sites and lets a real
    one hide behind a trailing comment (#737's lesson). Offending line numbers
    are REPORTED; never `assertIn` over the whole source (#732 measured a
    393KB failure message from that).

    THE BATTERY, as RUN: 22 mutations, 22 killed, plus two inert probes.

      1  delete the refresh call ............ graze_penalty_RISES + 3 others
      2  call it BEFORE the nudge ........... graze_penalty_RISES + 3 others
      3  hand the registrar an empty list ... graze_penalty_RISES + 3 others
      4  register but never prune per cap ... every_cap_in_reach, SEEDPose
      5  prune but never file the tuple ..... CONSTRUCTED_in_exactly_one_place
      6  refresh AFTER the unresolved recompute  connector_is_DRAWN_and_CHARGED
      7  drop `_item_reach` ................. two_prices_are_separated
      8  drop the `/2` ...................... tuple_is_priced_and_layered
      9  omit the floor map ................. floor_filed_is_the_TRACK_floor
      10 file `_via_floor_for` instead ...... floor_filed_is_the_TRACK_floor
      11 `t[6] = _seg_side(layer)` .......... LayerScopeSurvivesTheRebuild
      12 extend `cap_segs[ref]` IN PLACE .... cap_segs_is_ASSIGNED + 3 others
      13 file the floor under a stale id .... floor_filed_is_the_TRACK_floor
      14 refresh only the first cap ......... every_cap_in_reach (C2)
      15 drop the layer filter .............. LayerScopeSurvivesTheRebuild
      16 drop the distance filter ........... prune_predicate_has_ONE_spelling
      17 make the refresh unconditional ..... RefreshIsSkippedWhenNothingMoved
      18 prune from the MOVED pose .......... PruneIsAnchoredOnTheSEEDPose
      19 ignore the `upto` seed scope ....... SeedHalfIgnoresPassCreatedCopper
      20 record `_seed_seg_n` after the append  same
      21 grade the seed half over ALL copper .. same
      22 registrar reports the wrong count ... same

    TEN of them -- 10, 11, 15, 16, 17, 18 and each of 19-22 -- have exactly
    ONE killer, which is why those arms exist at all; the
    module docstring records that 17 and 18 SURVIVED the first run of this
    battery, and that 19-22 exist because a review found a defect the fix
    itself introduced.

    INERT PROBES, which must change nothing: a trailing comment naming
    `self._seg_floor_by_id[`, and a comment line naming the pricing spelling
    and an in-place `cap_segs[r] +=`. Both pass, so the comment stripper is
    doing its job rather than the guard being loose.

    DECLARED UNKILLABLE, rather than papered over: `if via_moves:` ->
    `if new_segs:`. A via move whose `conn_layers` is empty registers nothing
    either way, so no arm can separate them. Recorded as test_737 records its
    own surviving spelling.
    """

    @staticmethod
    def _code(obj):
        return [l.split('#')[0] for l in inspect.getsource(obj).splitlines()]

    def _sites(self, lines, literal):
        return [i + 1 for i, l in enumerate(lines) if literal in l]

    def setUp(self):
        self.mod = self._code(FC)
        self.rep = self._code(FC._Repair)
        self.builder = self._code(FC._Repair._register_segment)
        self.pruner = self._code(FC._Repair._prune_segs)
        self.nudger = self._code(FC.nudge_vias_for_unresolved)

    def test_a_track_tuple_is_CONSTRUCTED_in_exactly_one_place(self):
        for lit, where in (('self.segments.append(', self.builder),
                           ('self._seg_floor_by_id[', self.builder),
                           ('width / 2.0 + self._item_reach(', self.builder)):
            sites = self._sites(self.mod, lit)
            self.assertEqual(
                len(sites), 1,
                '%r occurs at %d sites (module-relative line(s) %s). #736 '
                'exists because the one in __init__ was the only one, so '
                'nothing could register the connectors the pass draws. A '
                'second is that defect again.' % (lit, len(sites), sites))
            self.assertTrue(any(lit in l for l in where),
                            '%r moved out of _register_segment' % lit)
    # MUTATION: re-inline the tuple build in __init__ or in the registrar.

    def test_the_prune_predicate_has_exactly_one_spelling(self):
        sites = self._sites(self.rep, '_point_to_seg_dist(')
        self.assertEqual(len(sites), 1,
                         '_Repair measures a point-to-track distance at %d '
                         'sites (class-relative line(s) %s); the prune must '
                         'have one' % (len(sites), sites))
        self.assertTrue(any('_point_to_seg_dist(' in l for l in self.pruner))
        calls = self._sites(self.mod, 'self._prune_segs(')
        self.assertEqual(len(calls), 2,
                         'expected exactly two callers of the prune -- '
                         '__init__ and the registrar; found %d at %s'
                         % (len(calls), calls))
    # MUTATION: re-inline the prune loop in either caller.

    def test_cap_segs_is_ASSIGNED_and_never_mutated_in_place(self):
        asg = re.compile(r'self\.cap_segs\[[^\]]+\]\s*=[^=]')
        writes = [i + 1 for i, l in enumerate(self.mod) if asg.search(l)]
        self.assertEqual(len(writes), 2,
                         'expected two cap_segs assignments (__init__ and the '
                         'registrar); found %d at %s' % (len(writes), writes))
        inplace = re.compile(r'cap_segs\[[^\]]*\]\s*(?:\.append|\.extend|\+=)')
        bad = [i + 1 for i, l in enumerate(self.mod) if inplace.search(l)]
        self.assertEqual(bad, [],
                         'cap_segs is mutated IN PLACE at line(s) %s -- the '
                         '_cap_seg_eff memo revalidates on the list IDENTITY, '
                         'so the rows are never rebuilt and _seg_shortfalls '
                         'indexes past the end of its row' % bad)
    # MUTATION: `cap_segs[ref].append(t)` / `.extend(kept)` / `+= kept`.

    def test_the_registrar_is_not_reached_from_the_duck_typed_nudger(self):
        """`nudge_vias_for_unresolved` takes a duck-typed `st` -- five test
        files pass a stand-in carrying only caps / vias / graze_penalty. A
        resolver there has an honest flat fallback; a MUTATION does not,
        because "silently do nothing" IS this defect."""
        self.assertEqual(self._sites(self.nudger, 'register_new_segments('), [])
        calls = [i + 1 for i, l in enumerate(self.mod)
                 if 'register_new_segments(' in l and 'def ' not in l]
        self.assertEqual(len(calls), 1,
                         'expected exactly one call site; found %d at %s'
                         % (len(calls), calls))
    # MUTATION: move the call inside the nudger -> five test files AttributeError.

    def test_the_zero_count_arms_are_not_searching_for_dead_strings(self):
        """Every `== 0` arm above passes after a rename. Assert the positive
        controls exist, so a rename fails HERE rather than disarming them."""
        for token, floor in (('cap_segs', 6), ('self.segments', 3),
                             ('_seg_floor_by_id', 4),
                             ('pcb_data.segments', 5)):
            n = sum(1 for l in self.mod if token in l)
            self.assertGreaterEqual(
                n, floor, '%r appears %d times in CODE (expected >= %d) -- it '
                          'was renamed, and the zero-count arms of this file '
                          'are now vacuous' % (token, n, floor))
    # MUTATION: rename any of those identifiers.

    def test_a_trailing_comment_cannot_arm_or_disarm_the_guard(self):
        """False-positive probe: the stripper must drop a trailing comment, so
        a comment MENTIONING a guarded literal invents no site."""
        probe = ['        x = 1  # self.segments.append( and cap_segs[r] += q',
                 '        # width / 2.0 + self._item_reach(fl)']
        stripped = [l.split('#')[0] for l in probe]
        self.assertEqual(self._sites(stripped, 'self.segments.append('), [])
        self.assertEqual(
            self._sites(stripped, 'width / 2.0 + self._item_reach('), [])
    # MUTATION: read raw lines instead of stripping comments.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """Two arms. The first says no tracked board emits a connector at the
    shipped defaults, so the change is inert there. The second is what stops
    that from being a statement about unreachable code."""

    @staticmethod
    def _reaching():
        """Boards carrying BOTH vias and a movable near-BGA cap -- the only
        ones that get past repair_fanout_clearance's two early returns."""
        boards = run_utils.corpus_boards()
        if not boards:
            return None
        out = []
        for b in boards:
            try:
                pcb = parse_kicad_pcb(b)
            except Exception:                                    # noqa: BLE001
                continue
            if not pcb.vias:
                continue
            if not any('BGA' in (f.footprint_name or '').upper()
                       for f in pcb.footprints.values()):
                continue
            out.append(b)
        return out

    def test_no_tracked_board_EMITS_a_connector_at_the_shipped_defaults(self):
        reaching = self._reaching()
        if reaching is None:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(len(reaching), 3,
                                'the candidate set collapsed; the filter is '
                                'wrong and this arm proves nothing')
        emitters = []
        for b in reaching:
            for clr in (0.25, 0.10):
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    r = repair_fanout_clearance(parse_kicad_pcb(b), b,
                                                clearance=clr)
                if r.get('new_segments'):
                    emitters.append((os.path.basename(b), clr,
                                     len(r['new_segments'])))
        self.assertEqual(emitters, [],
                         'a tracked board now emits connector copper: %r. The '
                         '"inert on the corpus" claim in the #736 PR has '
                         'EXPIRED -- re-run the before/after and record the '
                         'new unresolved lists.' % (emitters,))
    # MUTATION: none -- a self-expiring corpus bound, not a fix assertion.

    def test_the_one_configuration_that_DOES_emit_connectors_is_still_clean(self):
        """The FALLBACK-OFF leg of tests/test_fanout_clearance.py's frozen
        sweep, on the one board that reaches the nudger with it: 9 via moves
        and 17 connectors on four different layers, all registered, and NOT
        ONE of them lands in any cap's keep-out. This is the whole in-repo
        real-board A/B for #736, and it is what makes the synthetic rig
        necessary rather than lazy.

        `via_clear_fallback=False` is not decoration: with the fallback ON the
        same board reports 0 unresolved, so the nudger is never called and
        this arm would assert about nothing."""
        board = os.path.join(_ROOT, 'kicad_files',
                             'orangecrab_ext_pll.kicad_pcb')
        if not os.path.isfile(board):
            print('SKIP: %s is absent' % board)
            self.skipTest('fixture absent')
        pcb = parse_kicad_pcb(board)
        seen, buf = [], io.StringIO()
        with contextlib.redirect_stdout(buf):
            r = repair_fanout_clearance(pcb, board, clearance=0.1,
                                        max_displacement=0.0, max_passes=1,
                                        via_clear_fallback=False,
                                        on_move=lambda st: seen.append(st))
        st = seen[0]
        self.assertGreater(len(r['new_segments']), 10,
                           'this configuration stopped emitting connectors, '
                           'so the arm below is vacuous')
        self.assertGreater(len({s['layer'] for s in r['new_segments']}), 2)
        registered = {(round(s['start'][0], 6), round(s['start'][1], 6),
                       round(s['end'][0], 6), round(s['end'][1], 6),
                       s['net_id'], s['layer'])
                      for s in r['new_segments']}
        live = {(round(t[0], 6), round(t[1], 6), round(t[2], 6),
                 round(t[3], 6), t[4], t[6]) for t in st.segments}
        self.assertLessEqual(registered, live,
                             'a connector the pass drew never reached the '
                             'track view')
        dirty = {ref: st._seg_shortfalls(ref, c, c.x, c.y, c.rot)
                 for ref, c in st.caps.items()}
        self.assertEqual({k: v for k, v in dirty.items() if v}, {},
                         'a connector now grazes a cap on a REAL board -- '
                         'grade it with check_drc before believing it')
    # MUTATION: delete the refresh -> `registered <= live` fails.


if __name__ == '__main__':
    unittest.main(verbosity=2)
