"""#725: the fanout-clearance repair pass must price each pair at its REAL
required clearance -- netclass, `.kicad_dru` layer rule, pad `(clearance ...)`
override -- exactly as check_drc grades it, and stay byte-identical on a board
that declares none of them.

Conventions this file follows (from #697's test file, and CLAUDE.md):

  * REAL parser dataclasses and REAL boards. `_Repair.__init__` reads courtyards
    and locked refs from the file on disk, so even a synthetic case needs a file.
  * Every assertion names the single-line MUTATION that must kill it.
  * Assert you are ON the branch before asserting about it -- each test spies the
    value its branch keys on, with a detail string saying why.

Nothing here shells out or drives a CLI: it runs entirely in-process in ~22 s.
`run_all.is_integration` classifies by grepping the SOURCE for `run_utils` and
the name of the standard sub-process module, and this file NAMES those markers
in prose (including in this sentence) -- which is exactly how the first version
of it got bucketed as integration and silently skipped under `--fast`, a test
file that proves nothing being worse than no test file at all. Hence the
explicit opt-out below.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import copy
import io
import json
import math
import os
import shutil
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

from kicad_parser import parse_kicad_pcb
from copy_board import copy_board
from synth import make_via
import placement.legality as L
from placement.legality import PadClearanceModel
from placement.fanout_clearance import (_Cap, _Repair, _pad_pair_shortfall,
                                        _point_to_rect_dist, _rect_gap,
                                        nudge_vias_for_unresolved,
                                        repair_fanout_clearance)

BOARD = os.path.join(_ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
INERT = os.path.join(_ROOT, 'kicad_files',
                     'rp2350_fpga_eensy_prePlane.kicad_pcb')
CLEAR = 0.1

# Measured anchors on the stock board at CLEAR (see the PR body). TP32.1 is a
# B.Cu test point on net 69 sitting 0.1346mm from C20's pad -- above the flat
# 0.1 and below anything a declaration raises it to, which is precisely the
# band the flat scalar cannot see.
ANCHOR_CAP = 'C20'
ANCHOR_NET = 69
ANCHOR_GAP = 0.1346
MOVER_A, MOVER_B = 'C49', 'C62'

# The inert board's recorded result. #725 did not move it; #731 did, on
# purpose and by a lot -- 2 placements and 7 unresolved caps became 1 and 1.
#
# rp2350 declares no netclass, no .kicad_dru and no pad override, so the
# clearance model is INERT and the pre-#731 layer side-channels were both
# empty here. It is also 6 copper layers, and 2342 of its 4331 pruned
# pad x track pairs were OFF-LAYER, carrying 4.6685mm of phantom seed graze
# against 0.0142mm of real. Per-cap seed track graze, on-layer / off-layer:
#
#   C16  0.0000 / 0.7046      C20  0.0000 / 0.2900
#   C17  0.0000 / 0.4913      C22  0.0000 / 0.9160
#   C18  0.0106 / 0.0000      C23  0.0036 / 0.5836
#   C19  0.0000 / 0.7851      C24  0.0000 / 0.8979
#
# So six of the eight caps were violators purely on phantom, and only C18 and
# C23 have anything real. C18 is resolved; C23 is not, and is the one cap that
# legitimately remains unresolved. The old run also SHIPPED a violation for
# this: it moved C19 to clear a phantom and put its GND pad 0.145mm inside a
# real F.Cu track, so check_drc flagged a PAD-SEGMENT on the output of a board
# that is clean as it ships. It is clean again now.
INERT_UNRESOLVED = ['C23']
INERT_PLACEMENTS = 1


def _repair(path, clearance=CLEAR, prefix='C,R,FB', pcb=None):
    """The 10-POSITIONAL construction every other test in this family uses.
    Calling it positionally is itself part of the #725 shape contract."""
    return _Repair(pcb if pcb is not None else parse_kicad_pcb(path), path,
                   clearance, 0.1, 0.55, 1.0, 2.0, 0.3, prefix, set())


def _stage(tmp, name, classes=None, dru=None, src=None):
    """A COPY of an in-repo board (siblings carried by copy_board), plus an
    optional netclass project and/or an optional .kicad_dru."""
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, 'b.kicad_pcb')
    copy_board(src or BOARD, dst)
    pro = os.path.splitext(dst)[0] + '.kicad_pro'
    if classes is not None:
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump({'net_settings': {'classes': classes,
                                        'netclass_assignments': {},
                                        'netclass_patterns': []}}, f)
    elif os.path.exists(pro):
        os.remove(pro)
    if dru is not None:
        with open(os.path.splitext(dst)[0] + '.kicad_dru', 'w',
                  encoding='utf-8') as f:
            f.write(dru)
    return dst


def _default_class(clearance):
    return [{'name': 'Default', 'clearance': clearance, 'track_width': 0.2,
             'via_diameter': 0.6, 'via_drill': 0.4, 'priority': 2147483647}]


def _dru(layer, mm):
    return ('(version 1)\n(rule r (layer "%s") '
            '(constraint clearance (min %gmm)))\n' % (layer, mm))


def _pad_rect(p):
    """The pad rect exactly as _Repair builds it (rect_rotation folded in)."""
    t = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
    c, s = abs(math.cos(t)), abs(math.sin(t))
    hx, hy = p.size_x / 2.0, p.size_y / 2.0
    ex, ey = hx * c + hy * s, hx * s + hy * c
    return (p.global_x - ex, p.global_y - ey, p.global_x + ex, p.global_y + ey)


def _anchor_pad(pcb):
    """The parser Pad behind the ANCHOR_GAP tuple -- located by matching the
    rect the module itself computes, so this cannot drift from the geometry."""
    st = _repair(BOARD, pcb=pcb)
    cap = st.caps[ANCHOR_CAP]
    r = cap.pad_rects()[0]
    for t in st.cap_foreign_pads[ANCHOR_CAP]:
        if t[4] == r[4] or t[4] == -1:
            continue
        if t[5] is not None and t[5] != cap.side:
            continue
        gap = _rect_gap(r[:4], t[:4])
        if abs(gap - ANCHOR_GAP) > 5e-3:
            continue
        for fref, fp in pcb.footprints.items():
            for p in fp.pads:
                if p.net_id != t[4]:
                    continue
                if max(abs(a - b) for a, b in zip(_pad_rect(p), t[:4])) < 1e-9:
                    return fref, p, gap
    raise AssertionError('the ANCHOR_GAP pair is gone -- the fixture moved')


def _short(st, ref):
    cap = st.caps[ref]
    return st._pad_shortfalls(ref, cap, cap.x, cap.y, cap.rot)


class TestSourcesReachTheEngine(unittest.TestCase):
    """Each of the three declaration channels must arrive, and a board with
    none of them must not build a model at all."""

    def test_pad_override_raises_a_pad_pair(self):
        pcb = parse_kicad_pcb(BOARD)
        owner, pad, gap = _anchor_pad(pcb)
        # ON THE BRANCH: the pair must sit in the band the flat scalar misses,
        # or every assertion below passes for the wrong reason.
        self.assertTrue(CLEAR < gap < 0.5,
                        'gap %.4f is not in (%.2f, 0.5) -- this test would '
                        'pass vacuously' % (gap, CLEAR))
        self.assertEqual(_short(_repair(BOARD, pcb=pcb), ANCHOR_CAP), {},
                         'the stock board must not charge this pair')
        pad.local_clearance = 0.5
        st = _repair(BOARD, pcb=pcb)
        # NB: NOT `assertIsNotNone(st._floors)` -- this board's 6 fiducials
        # already carry local_clearance 0.375, so the model is active before
        # this test writes anything, and that assertion would credit the setup
        # for a state it did not create.
        self.assertAlmostEqual(st.caps[ANCHOR_CAP].max_floor, 0.0, places=9,
                               msg='the CAP carries a floor of its own -- the '
                                   'shortfall below would not isolate the '
                                   'foreign override')
        got = _short(st, ANCHOR_CAP)
        self.assertIn(pad.net_id,
                      got, 'the %s override never reached the shortfall' % owner)
        self.assertAlmostEqual(got[pad.net_id], 0.5 - gap, places=4)
    # MUTATION: `_pad_shortfalls` back to `if gap < self.clearance - EPS:`
    # (with the matching `self.clearance - gap`) -> got == {}.

    def test_netclass_from_a_sibling_project_raises_a_pad_pair(self):
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'ncl', classes=_default_class(0.4))
            pcb = parse_kicad_pcb(p)
            m = PadClearanceModel.for_board(pcb, CLEAR, p)
            named = sum(1 for n in pcb.nets.values() if n.name)
            # ON THE BRANCH: every named net must have arrived, at 0.4.
            self.assertEqual(len(m.net_floor), named,
                             'the project reached %d of %d named nets'
                             % (len(m.net_floor), named))
            self.assertEqual({round(v, 6) for v in m.net_floor.values()}, {0.4})
            self.assertIn(ANCHOR_NET, _short(_repair(p, pcb=pcb), ANCHOR_CAP),
                          'the netclass never reached the shortfall')
    # MUTATION: drop the `PadClearanceModel.for_board` call in
    # `_Repair.__init__` -> the shortfall is empty.

    def test_dru_layer_rule_raises_a_pad_pair(self):
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'dru', dru=_dru('B.Cu', 0.5))
            pcb = parse_kicad_pcb(p)
            m = PadClearanceModel.for_board(pcb, CLEAR, p)
            # ON THE BRANCH: the rule must have parsed, on the anchor's layer.
            self.assertEqual({k: round(v, 6) for k, v in m.layer_rules.items()},
                             {'B.Cu': 0.5}, 'the .kicad_dru did not parse')
            self.assertIn(ANCHOR_NET, _short(_repair(p, pcb=pcb), ANCHOR_CAP),
                          'the layer rule never reached the shortfall')
    # MUTATION: drop `read_board_layer_clearances` from the model -> empty.

    def test_an_override_on_a_COPPERLESS_pad_is_not_charged(self):
        """`_Cap`'s pad filter is deliberately loose (`endswith('.Cu')`), which
        admits an np_thru_hole pad -- it lists *.Cu and carries no copper. A
        FLOOR must not be that loose: PadClearanceModel.pad_floor reads
        local_clearance unconditionally, so charging it would move a cap to
        clear copper that does not exist, and would contradict the model's own
        inertness rule, which refuses to ACTIVATE for an NPTH-only override."""
        pcb = parse_kicad_pcb(BOARD)
        model = PadClearanceModel.for_board(pcb, CLEAR, BOARD)
        self.assertTrue(model.active)
        fp = copy.deepcopy(pcb.footprints[ANCHOR_CAP])
        # the footprint carries paste-only apertures too; take a REAL copper pad
        real = next(p for p in fp.pads if L._pad_carries_copper(p))
        real.local_clearance = 0.5                # copper, must be charged
        hole = copy.deepcopy(real)
        hole.pad_number = 'H1'
        hole.pad_type = 'np_thru_hole'
        hole.drill = 1.0
        hole.layers = ['*.Cu', '*.Mask']
        hole.net_id = 0
        hole.local_clearance = 3.0                # absurd, and must be ignored
        fp.pads.append(hole)
        # _Cap directly, not through _Repair: appending a pad would push this
        # footprint past the `n_copper <= 2` cap test and it would stop being a
        # mover at all -- the very coupling the loose filter's comment warns
        # about.
        cap = _Cap(fp, (-0.5, -0.5, 0.5, 0.5), model)
        # ON THE BRANCH: the loose GEOMETRY filter must still admit the hole
        # (it lists *.Cu), or this tests nothing. The footprint also carries
        # paste-only apertures, which the filter correctly drops.
        loose = [p for p in fp.pads
                 if any(str(l).endswith('.Cu') for l in p.layers)]
        self.assertIn(hole, loose, 'the hole was dropped by the geometry '
                                   'filter -- this test would pass vacuously')
        self.assertEqual(len(cap.pads), len(loose))
        self.assertEqual(len(cap.pad_floors), len(cap.pads))
        self.assertEqual(cap.pad_floors[-1].lc, 0.0,
                         'the copper-less pad was given a clearance floor')
        self.assertAlmostEqual(cap.max_floor, 0.5, places=6,
                               msg='the NPTH override reached max_floor and '
                                   'would widen every prune and the prescreen')
        # ...and the pad's LAYER SET is empty, which is the marker every eff
        # builder keys "grade this pad flat" on.
        self.assertEqual(cap.pad_layers[-1], frozenset())
        self.assertTrue(cap.pad_layers[0], 'the copper pad lost its layers')
    # MUTATION: drop the `_pad_carries_copper` guard on `model.pad_floor(p)` in
    # `_Cap.__init__` -> max_floor becomes 3.0.

    def test_a_COPPERLESS_pad_is_graded_flat_in_EVERY_channel(self):
        """Zeroing the copper-less pad's own override is not enough: the
        PARTNER's netclass still flows through `pair()`, charging a keep-out
        against a pad check_drc does not grade at all. Same defect as the
        off-layer phantom, in the pad / via / cap-cap channels.

        Measured before this guard, with C20's second copper pad replaced by an
        np_thru_hole at lc=1.0 on a Default-0.4 board: that pad was priced at
        0.4 against every foreign pad, and C20's foreign-pad shortfall read
        0.6950mm where the flat scalar sees 0.2450."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'npthflat', classes=_default_class(0.4))
            pcb = parse_kicad_pcb(p)
            fp = pcb.footprints[ANCHOR_CAP]
            real = [q for q in fp.pads if L._pad_carries_copper(q)]
            self.assertGreaterEqual(len(real), 2, 'need a 2-copper-pad cap')
            hole = copy.deepcopy(real[1])
            hole.pad_type = 'np_thru_hole'
            hole.drill = 1.0
            hole.layers = ['*.Cu', '*.Mask']
            hole.local_clearance = 1.0
            fp.pads[fp.pads.index(real[1])] = hole
            st = _repair(p, pcb=pcb)
            cap = st.caps[ANCHOR_CAP]
            flat_i = [k for k in range(len(cap.pad_floors))
                      if not cap.pad_layers[k]]
            live_i = [k for k in range(len(cap.pad_floors))
                      if cap.pad_layers[k]]
            # ON THE BRANCH: one of each, or this proves nothing.
            self.assertEqual(len(flat_i), 1)
            self.assertTrue(live_i)
            fi, li = flat_i[0], live_i[0]

            pad_rows = st._pad_effs(ANCHOR_CAP, cap)
            self.assertEqual({round(v, 6) for v in pad_rows[fi]}, {CLEAR})
            self.assertEqual({round(v, 6) for v in pad_rows[li]}, {0.4},
                             'the COPPER pad must still be charged at 0.4')

            vias = st.cap_vias[ANCHOR_CAP]
            self.assertTrue(vias)
            via_rows = st._via_effs(ANCHOR_CAP, cap, vias)
            for j, v in enumerate(vias):
                radius = st._via_radius_by_id[id(v)][1]
                self.assertAlmostEqual(via_rows[fi][j], radius + CLEAR,
                                       places=9)
                self.assertAlmostEqual(via_rows[li][j], radius + 0.4, places=9)

            oref = next((o for o in st.cap_caps[ANCHOR_CAP]
                         if st._cap_floors_ok(st.caps[o])), None)
            self.assertIsNotNone(oref, 'no mover neighbour to pair with')
            pair_rows = st._pair_effs(ANCHOR_CAP, cap, oref, st.caps[oref])
            self.assertEqual({round(v, 6) for v in pair_rows[fi]}, {CLEAR})
            self.assertEqual({round(v, 6) for v in pair_rows[li]}, {0.4})
    # MUTATION: drop the `_flat_pad` test in `_pad_effs` / `_via_effs` /
    # `_pair_effs` -> the copper-less row comes back at 0.4.

    def test_a_board_declaring_nothing_builds_no_model(self):
        pcb = parse_kicad_pcb(INERT)
        self.assertFalse(PadClearanceModel.for_board(pcb, CLEAR, INERT).active)
        self.assertIsNone(_repair(INERT, pcb=pcb)._floors,
                          'an inert board must take the flat path')
    # MUTATION: drop the `.active` gate -> `_floors` is a model.


class TestChannels(unittest.TestCase):
    """pad<->pad, via<->pad and track<->pad, each priced at the requirement."""

    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.mkdtemp()
        cls.path = _stage(cls.td, 'chan', dru=_dru('B.Cu', 0.5))
        cls.st = _repair(cls.path)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.td, ignore_errors=True)

    def test_static_foreign_pad_is_charged(self):
        got = _short(self.st, ANCHOR_CAP)
        self.assertIn(ANCHOR_NET, got)
        self.assertAlmostEqual(got[ANCHOR_NET], 0.5 - ANCHOR_GAP, places=3)
    # MUTATION: `_pad_shortfalls` back to the flat scalar -> {}.

    def test_mover_vs_mover_pad_is_charged(self):
        a, b = self.st.caps[MOVER_A], self.st.caps[MOVER_B]
        gap = min(_rect_gap(ra[:4], rb[:4]) for ra in a.pad_rects()
                  for rb in b.pad_rects() if ra[4] != rb[4])
        # ON THE BRANCH: the pair must sit in the uncharged band.
        self.assertTrue(CLEAR < gap < 0.5,
                        '%s<->%s gap %.4f is outside the band -- this test '
                        'would pass vacuously' % (MOVER_A, MOVER_B, gap))
        effs = self.st._pair_effs(MOVER_A, a, MOVER_B, b)
        self.assertIsNotNone(effs, 'no per-pair requirements were resolved')
        self.assertEqual(
            _pad_pair_shortfall(a.pad_rects(), b.pad_rects(), CLEAR), 0.0,
            'the flat scalar must see nothing here')
        self.assertGreater(
            _pad_pair_shortfall(a.pad_rects(), b.pad_rects(), CLEAR, effs), 0.0)
    # MUTATION: `_pad_pair_shortfall` back to `if gap < clearance - EPS:` -> 0.0.

    def test_via_dru_rule_binds_through_the_span(self):
        """A via has no pad layers of its own, so its PadFloor must be scoped to
        ALL copper. Built with `layers=None`, PadClearanceModel.pair does
        `fb.layers or ()`, the shared set is empty, and EVERY dru rule is
        silently discarded -- a wrong answer that passes every other test."""
        fl = self.st.via_floor(0)
        self.assertIsNotNone(fl)
        self.assertTrue(fl.layers, 'the via floor carries no layer scope')
        self.assertIn('B.Cu', fl.layers)
        pf = self.st.caps[ANCHOR_CAP].pad_floors[0]
        self.assertAlmostEqual(self.st.via_required(pf, 0), 0.5, places=6)
    # MUTATION: build the via PadFloor with `layers=None` -> via_required
    # returns the flat 0.1.

    def test_track_is_scoped_to_the_SEGMENTS_own_layer(self):
        """check_drc resolves pad-segment as a single-layer REPLACE on
        `seg.layer`, NOT over the F/B `side` the tuple carries (which files an
        In1.Cu track under 'F')."""
        pf = self.st.caps[ANCHOR_CAP].pad_floors[0]      # a B-side cap pad
        self.assertAlmostEqual(
            self.st.required(pf, self.st.seg_floor(0, 'B.Cu')), 0.5, places=6)
        self.assertAlmostEqual(
            self.st.required(pf, self.st.seg_floor(0, 'F.Cu')), CLEAR, places=6)
        self.assertAlmostEqual(
            self.st.required(pf, self.st.seg_floor(0, 'In1.Cu')), CLEAR,
            places=6)
    # MUTATION: scope the segment floor to `self._all_cu` -> the F.Cu and
    # In1.Cu arms return 0.5 too.

    def test_an_OFF_LAYER_track_pair_is_not_in_cap_segs_AT_ALL(self):
        """#725 charged an off-layer pair the FLAT scalar so a netclass could
        not amplify it; #731 removed the pair instead, which is what check_drc
        does (check_pad_segment_overlap returns early off-layer).

        This test used to assert the flat price and to REQUIRE such a pair to
        exist (`assertGreater(off, 0)`). Under #731 that is structurally
        impossible, so the spy moved: it recomputes the OLD 'F'/'B' side prune
        and asserts THAT still admits off-layer pairs, then asserts the real
        `cap_segs` admits none. Without the recomputed spy this test would pass
        just as well on a build that had deleted track grading outright, which
        is why the on-layer arm below is kept unchanged.

        Measured on this board at a Default class of 0.3 before #731:
        R17/R18/R5's entire graze WAS the phantom -- 0.0821mm each flat,
        0.6991/0.6991/0.5686 raised."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'offlayer', classes=_default_class(0.4))
            st = _repair(p)
            # ON THE BRANCH: the OLD side prune really did admit off-layer
            # pairs on this board, so "0 now" is a change, not a tautology.
            old_off = 0
            for ref, cap in st.caps.items():
                for t in st.segments:
                    if st._seg_side(t[6]) != cap.side:
                        continue
                    if not any(t[6] in pl for pl in cap.pad_layers):
                        old_off += 1
            self.assertGreater(old_off, 0,
                               'the OLD side prune admitted no off-layer pair '
                               'on this board -- the spy proves nothing')
            checked = off = on = 0
            for ref, cap in st.caps.items():
                rows = st._seg_effs(ref, cap)
                if rows is None:
                    continue
                for i in range(len(cap.pad_layers)):
                    mine = cap.pad_layers[i]
                    for j, t in enumerate(st.cap_segs[ref]):
                        layer = t[6]
                        half = t[5] - st._item_reach(
                            st._seg_floor_by_id.get(id(t)))
                        checked += 1
                        if layer in mine:
                            on += rows[i][j] > half + CLEAR + 1e-9
                        else:
                            off += 1
            self.assertEqual(off, 0,
                             '%d off-layer pair(s) survived the prune' % off)
            # ON THE BRANCH: on-layer pairs must still be raised, or the whole
            # track channel is simply gone and `off == 0` is vacuous.
            self.assertGreater(on, 0, 'no on-layer pair was raised')
            self.assertGreater(checked, 0, 'no pair was examined at all')
    # MUTATION: revert the prune to `if seg[6] != cap.side: continue` -> `off`
    # is no longer 0. MUTATION: make `_seg_shares` return False always -> `on`
    # drops to 0 and the vacuity guard fires.

    def test_a_board_with_NO_copper_layers_RESOLVES_one(self):
        """If `board_info.copper_layers` is empty, `pad_copper_layers` resolves
        NOTHING for a `*.Cu` pad -- so every such pad reads copper-less and the
        scoping would grade the board at the flat scalar. #725 handled that by
        switching scoping OFF entirely, on the reasoning that the board only
        ever over-blocked before.

        #731 corrected the reasoning. Switching off falls back to the 'F'/'B'
        side collapse, and check_drc does NOT switch off: it resolves the same
        situation with a three-level fallback (filter to `.Cu`, then the layers
        segments sit on, then ['F.Cu','B.Cu']) and still expands and grades a
        `*.Cu` pad over the result. The side collapse drops every B.Cu and
        inner track a through-hole pad shares copper with -- an UNDER-block,
        which is what actually ships a violation. Measured on rp2350 with the
        copper list cleared and cap pads made through-hole: 280 pairs check_drc
        grades that the old fallback ignored.

        So `_Repair` now resolves a copper list the same way check_drc does,
        and `_all_cu` is never empty on a board that has any copper at all."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'nocu', classes=_default_class(0.4))
            pcb = parse_kicad_pcb(p)
            real = list(pcb.board_info.copper_layers)
            pcb.board_info.copper_layers = []
            st = _repair(p, pcb=pcb)
            # ON THE BRANCH: the model must still be active, or nothing is
            # scoped anyway and this passes for the wrong reason.
            self.assertIsNotNone(st._floors)
            self.assertEqual(pcb.board_info.copper_layers, [])
            # ...and the fallback reconstructed a real list from the segments.
            self.assertTrue(st._all_cu, 'the fallback resolved no copper')
            self.assertTrue(st._all_cu <= set(real),
                            'the fallback invented a layer: %r'
                            % (st._all_cu - set(real),))
            cap = st.caps[ANCHOR_CAP]
            self.assertIsNotNone(st._cap_pad_layers(cap),
                                 'layer scoping switched itself off')
            rows = st._pad_effs(ANCHOR_CAP, cap)
            self.assertTrue(rows)
            self.assertIn(0.4, {round(v, 6) for r in rows for v in r},
                          'every pair fell back to the flat scalar -- the '
                          'board is now UNDER-blocked')
    # MUTATION: drop the segment-derived fallback for `_all_cu` -> it is empty,
    # `_cap_pad_layers` returns None and the assertIsNotNone fires.

    def test_a_relaxing_rule_REPLACES_downward(self):
        """A dru rule REPLACES, so it can lower a pair below the netclass.
        Pinning this is what proves the compare uses pair(), not max_floor()."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'relax', classes=_default_class(0.4),
                       dru=_dru('B.Cu', 0.15))
            st = _repair(p)
            cap = st.caps[ANCHOR_CAP]
            pf = cap.pad_floors[0]
            # ON THE BRANCH: the pad's own netclass must be the 0.4 the rule is
            # about to replace, or "replaced downward" is not what is tested.
            self.assertAlmostEqual(pf.ncl, 0.4, places=6)
            self.assertAlmostEqual(st.required(pf, st.seg_floor(0, 'B.Cu')),
                                   0.15, places=6)
            # ...while the BROAD-PHASE bound keeps the 0.4: a relaxing rule
            # REPLACES a pair value but must never shrink a prune, which may
            # over-reach and must never under-reach.
            self.assertGreaterEqual(
                cap.max_floor, 0.4,
                'the broad-phase bound was lowered by a relaxing rule')
    # MUTATION: `_pair_or_flat` -> `max(self.clearance, model.max_floor(fa),
    # model.max_floor(fb))` -> the `required(...)` assertion reads 0.4 instead
    # of the replaced-down 0.15. (A mutation confined to the eff builders does
    # NOT reach this test -- it reads the resolver directly.)

    def test_the_track_bbox_reject_widens_with_the_requirement(self):
        """`_seg_shortfalls`' cheap bbox reject is a SKIP keyed on the same
        half-width the compare uses; left flat it drops raised pairs."""
        ref = ANCHOR_CAP
        cap = self.st.caps[ref]
        segs = self.st.cap_segs[ref]
        self.assertTrue(segs, 'no tracks in reach of %s' % ref)
        rows = self.st._seg_effs(ref, cap)
        self.assertIsNotNone(rows)
        widened = sum(1 for i in range(len(rows))
                      for j, t in enumerate(segs)
                      if rows[i][j] > t[5] - self.st._item_reach(
                          self.st._seg_floor_by_id.get(id(t))) + CLEAR + 1e-9)
        self.assertGreater(widened, 0,
                           'no track keep-out was raised -- this test would '
                           'pass vacuously')
    # MUTATION: `_seg_effs`' `halves[j] + eff(...)` -> `halves[j] +
    # self.clearance` -> no row exceeds the flat half-width.
    # NB this reads the eff ROWS; `_seg_shortfalls`' own widened bbox reject is
    # a separate site and is NOT what this test covers.


class TestPruneRadiiStayExact(unittest.TestCase):
    """The pruned neighbour lists are documented as EXACT, not approximate.
    That claim dies the moment a requirement exceeds the flat scalar."""

    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.mkdtemp()
        cls.path = _stage(cls.td, 'prune', classes=_default_class(0.6))
        cls.st = _repair(cls.path)
        cls.flat = _repair(BOARD)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.td, ignore_errors=True)

    def test_no_pruned_list_shrinks_at_a_raised_requirement(self):
        """Every list must grow, or stay equal where nothing sits in the band --
        never shrink. A shrunk list is a pair the pass can no longer see."""
        grew = {}
        for ref in self.st.caps:
            for name in ('cap_foreign_pads', 'cap_caps', 'cap_segs', 'cap_vias'):
                a = len(getattr(self.flat, name)[ref])
                b = len(getattr(self.st, name)[ref])
                self.assertGreaterEqual(
                    b, a, '%s[%s] SHRANK %d -> %d at a raised requirement'
                          % (name, ref, a, b))
                if b > a:
                    grew[name] = grew.get(name, 0) + 1
        # ON THE BRANCH: each reach must actually have been raised, or this
        # passes on a no-op change.
        for name in ('cap_foreign_pads', 'cap_caps', 'cap_segs', 'cap_vias'):
            self.assertIn(name, grew,
                          '%s never grew -- its reach was not raised' % name)
    # MUTATION: revert the foreign-pad radius or the cap-cap radius -> that
    # list shrinks and its name is missing from `grew`.
    # NB the segment and via radii are NOT covered here: on a uniform-class
    # fixture the ITEMS' own floors already grow those two lists, so removing
    # the cap-side slack leaves `grew` intact. They are covered by
    # test_the_cap_side_slack_raises_the_via_and_track_reaches.

    def test_a_foreign_pad_beyond_the_OLD_reach_is_still_kept(self):
        cap = self.st.caps[ANCHOR_CAP]
        r = cap.rect()
        cx, cy = (r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0
        span = math.hypot(r[2] - cx, r[3] - cy)
        old = 3.0 + span + CLEAR          # max_displacement_cap + span + flat
        beyond = 0
        for t in self.st.cap_foreign_pads[ANCHOR_CAP]:
            px, py = (t[0] + t[2]) / 2.0, (t[1] + t[3]) / 2.0
            ph = math.hypot(t[2] - px, t[3] - py)
            if math.hypot(px - cx, py - cy) > old + ph:
                beyond += 1
        self.assertGreater(beyond, 0,
                           'no foreign pad sits beyond the old reach %.3f -- '
                           'this test would pass vacuously' % old)
    # MUTATION: `:466` back to `max_displacement_cap + span + clearance` ->
    # nothing beyond the old reach survives.

    def test_the_bbox_prescreen_widens_with_the_requirement(self):
        """`_blocked_geom`'s pad-bbox prescreen is a SKIP: left at the flat
        scalar it silently drops exactly the pairs a declaration raises.

        Behavioural, not arithmetic: it walks a cap toward each neighbour until
        the pair is charged beyond its seed baseline while the pad BBOXES are
        still further apart than the flat scalar, then asserts _blocked_geom
        actually vetoes that pose."""
        pcb = parse_kicad_pcb(BOARD)
        for p in pcb.footprints[ANCHOR_CAP].pads:
            p.local_clearance = 1.2      # raise only this mover's own pads
        st = _repair(BOARD, pcb=pcb)
        cap = st.caps[ANCHOR_CAP]
        found = None
        for oref in st.cap_caps[ANCHOR_CAP]:
            other = st.caps[oref]
            base = st.base_cap_pad.get(frozenset((ANCHOR_CAP, oref)), 0.0)
            effs = st._pair_effs(ANCHOR_CAP, cap, oref, other)
            orect = other.rect()
            ocx, ocy = (orect[0] + orect[2]) / 2.0, (orect[1] + orect[3]) / 2.0
            d = math.hypot(ocx - cap.x, ocy - cap.y)
            for k in range(1, 25):
                step = k * 0.1
                if step >= d:
                    break
                nx = cap.x + (ocx - cap.x) * step / d
                ny = cap.y + (ocy - cap.y) * step / d
                pads = cap.pad_rects(nx, ny, cap.rot)
                bbox_gap = _rect_gap(cap.pad_bbox(nx, ny, cap.rot),
                                     other.pad_bbox())
                charged = _pad_pair_shortfall(pads, other.pad_rects(),
                                              st.clearance, effs)
                flat = _pad_pair_shortfall(pads, other.pad_rects(), st.clearance)
                if (bbox_gap >= st.clearance and charged > base + 1e-6
                        and flat == 0.0):
                    found = (oref, nx, ny, bbox_gap, charged, base)
                    break
            if found:
                break
        # ON THE BRANCH: the pose must be one the OLD prescreen would skip, and
        # one the flat pricing scores at exactly zero.
        self.assertIsNotNone(found, 'no pose sits beyond the flat prescreen '
                                    'while being charged -- this test would '
                                    'pass vacuously')
        oref, nx, ny, bbox_gap, charged, base = found
        self.assertGreaterEqual(bbox_gap, st.clearance)
        self.assertGreater(charged, base)
        self.assertTrue(
            st._blocked_geom(ANCHOR_CAP, cap, nx, ny, cap.rot),
            '_blocked_geom let a charged %s<->%s pose through (bbox gap %.4f '
            '>= the flat %.2f, so the prescreen skipped it)'
            % (ANCHOR_CAP, oref, bbox_gap, st.clearance))
    # MUTATION: `_blocked_geom`'s prescreen back to `>= self.clearance` -> the
    # pair is skipped before its shortfall is computed and the pose is allowed.

    def test_the_cap_side_slack_raises_the_via_and_track_reaches(self):
        """`cap_segs`/`cap_vias` add the CAP's own excess over the flat scalar.
        A uniform netclass hides this -- the item's own floor already grew the
        list -- so raise one mover's pads and leave every via and track at the
        flat value."""
        pcb = parse_kicad_pcb(BOARD)
        for p in pcb.footprints[ANCHOR_CAP].pads:
            p.local_clearance = 1.2
        st = _repair(BOARD, pcb=pcb)
        # ON THE BRANCH: the cap is raised and the items are NOT.
        self.assertAlmostEqual(st.caps[ANCHOR_CAP].max_floor, 1.2, places=6)
        self.assertEqual(st._item_reach(st.via_floor(0)), CLEAR)
        for name in ('cap_segs', 'cap_vias'):
            a = len(getattr(self.flat, name)[ANCHOR_CAP])
            b = len(getattr(st, name)[ANCHOR_CAP])
            self.assertGreater(b, a, '%s did not grow with the cap-side slack '
                                     '(%d -> %d)' % (name, a, b))
    # MUTATION: `via_slack = 0.0` in `_prune_vias`, or drop
    # `max(0.0, cap.max_floor - self.clearance)` from `_prune_segs`'
    # `seg_reach` -> that list stops growing. Since #775 the via term has ONE
    # spelling, so this note now covers the construction prune AND the
    # post-nudge refresh; before it, a mutation could have zeroed one and
    # left the other intact.

    def test_the_seed_baseline_is_in_the_SAME_currency(self):
        """A baseline priced flat while candidates price at the requirement
        makes `_worsens_any_net` compare two different units, so the accept
        gate reads a pose as worse-than-seed on a pair that did not change.
        Measured by reverting that line at HEAD, the placement count moves in a
        CLASS-DEPENDENT direction and not by much -- which is exactly why this
        asserts the CURRENCY rather than a placement count."""
        raised = [k for k, v in self.st.base_cap_pad.items() if v > 0.0]
        self.assertTrue(raised, 'no mover pair has a non-zero seed baseline')
        for key in raised:
            ref, oref = tuple(key) if len(key) == 2 else (tuple(key)[0],) * 2
            cap, other = self.st.caps[ref], self.st.caps[oref]
            self.assertAlmostEqual(
                self.st.base_cap_pad[key],
                _pad_pair_shortfall(cap.pad_rects(), other.pad_rects(),
                                    self.st.clearance,
                                    self.st._pair_effs(ref, cap, oref, other)),
                places=9,
                msg='%s<->%s baseline is not in the candidates currency'
                    % (ref, oref))
        # and the flat pricing must NOT reproduce them, or the two currencies
        # are indistinguishable here.
        flat_zero = sum(
            1 for key in raised
            for ref, oref in [tuple(key)]
            if _pad_pair_shortfall(self.st.caps[ref].pad_rects(),
                                   self.st.caps[oref].pad_rects(),
                                   self.st.clearance) == 0.0)
        self.assertGreater(flat_zero, 0,
                           'every raised baseline is also non-zero flat -- this '
                           'test would pass vacuously')
    # MUTATION: `base_cap_pad` back to the 3-argument `_pad_pair_shortfall` ->
    # the raised baselines read 0.0.


class TestNudgerGraderConsistency(unittest.TestCase):
    """`nudge_vias_for_unresolved`'s offender test and `via_penalty`'s grazing
    test must resolve the SAME number for the same (cap pad, via) pair. If they
    diverge, the nudger returns an empty offender list for a cap the grader
    flagged and the cap stays unresolved forever with nothing printed."""

    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.mkdtemp()
        cls.path = _stage(cls.td, 'nudge', classes=_default_class(0.4))
        cls.st = _repair(cls.path)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.td, ignore_errors=True)

    def test_the_grader_rows_are_built_from_via_required(self):
        ref = ANCHOR_CAP
        cap = self.st.caps[ref]
        vias = self.st.cap_vias[ref]
        self.assertTrue(vias, 'no vias in reach of %s' % ref)
        rows = self.st._via_effs(ref, cap, vias)
        self.assertIsNotNone(rows)
        raised = 0
        for i, fa in enumerate(cap.pad_floors):
            for j, v in enumerate(vias):
                radius = v[3] - self.st._item_reach(self.st.via_floor(v[2]))
                want = self.st.via_required(fa, v[2])
                self.assertAlmostEqual(rows[i][j], radius + want, places=9)
                raised += (want > self.st.clearance + 1e-9)
        # ON THE BRANCH: some pair must actually be raised, or the loop above
        # compares the flat scalar with itself.
        self.assertGreater(raised, 0, 'no (pad, via) pair was raised -- this '
                                      'test would pass vacuously')
    # MUTATION: build the via eff rows from anything but `via_required` -> the
    # equality fails.

    def test_the_offender_loop_RESOLVES_rather_than_assuming_the_scalar(self):
        """Drives the real `nudge_vias_for_unresolved` and counts how often its
        offender loop consults `via_required`.

        Every via is parked far from every cap, so there are no offenders and
        the function returns empty either way -- but the loop must still have
        EVALUATED the predicate through the resolver. Priced at the flat scalar
        instead, the resolver is never called at all, and a cap unresolved
        because of a RAISED via requirement yields an empty offender list: the
        pass then reports it unresolved forever and prints nothing."""
        pcb = parse_kicad_pcb(self.path)
        st = _repair(self.path, pcb=pcb)
        bounds = pcb.board_info.board_bounds
        far = make_via(bounds[0] + 2.0, bounds[1] + 2.0, net_id=1)
        pcb.vias[:] = [far]
        st.vias = [(far.x, far.y, 1, 0.25 + st._item_reach(st.via_floor(1)))]
        st.cap_vias = {r: st.vias for r in st.caps}
        unresolved = [r for r in st.caps
                      if st.graze_penalty(r, st.caps[r], st.caps[r].x,
                                          st.caps[r].y, st.caps[r].rot) > 1e-6]
        # ON THE BRANCH: the loop only runs at all if some cap is unresolved,
        # and only reaches the resolver if a via is in the list.
        self.assertTrue(unresolved, 'no cap is unresolved -- the offender loop '
                                    'never runs and this proves nothing')
        calls = [0]
        real = st.via_required

        def counting(pad_floor, via_net):
            calls[0] += 1
            return real(pad_floor, via_net)
        st.via_required = counting
        moves, segs = nudge_vias_for_unresolved(st, pcb, CLEAR)
        self.assertEqual((moves, segs), ([], []),
                         'the far-away via should offend nobody')
        self.assertGreater(calls[0], 0,
                           'the offender loop never consulted via_required -- '
                           'it is pricing at the flat scalar')
    # MUTATION: the offender test in `nudge_vias_for_unresolved` back to
    # `vr + clearance - EPS` -> calls[0] == 0.

    def test_a_RELOCATED_via_keeps_its_radius(self):
        """`_Repair.relocate_vias` replaces the tuple of a moved via, so a
        moved via is a NEW tuple. Without carrying its entry across, it drops
        out of the radius map and is graded at its keep-out SLOT -- the prune
        over-reach -- rather
        than at the pair's requirement. Conservative, but wrong, and it is the
        one map that gains entries after __init__, so the entry must also hold
        its tuple or a recycled id hands back another via's radius.

        No tracked board relocates a via at the shipped 0.6mm budget, so this
        rigs one: a single foreign via just outside a cap pad, a cleared
        neighbourhood, and a wider `max_shift`."""
        pcb = parse_kicad_pcb(BOARD)
        st = _repair(BOARD, pcb=pcb)
        cap = st.caps['C67']
        r = cap.pad_rects()[0]
        own = {q[4] for q in cap.pads}
        fnet = next(n for n in pcb.nets if n and n not in own)
        vx, vy = r[2] + 0.20, (r[1] + r[3]) / 2.0
        pcb.vias[:] = [make_via(vx, vy, net_id=fnet, size=0.5, drill=0.3)]
        pcb.segments[:] = []
        t0 = (vx, vy, fnet, 0.25 + st._item_reach(st.via_floor(fnet)))
        st.vias = [t0]
        st._via_radius_by_id = {id(t0): (t0, 0.25)}
        st.cap_vias = {k: st.vias for k in st.caps}
        st.segments = []
        st.cap_segs = {k: [] for k in st.caps}
        moves, _segs = nudge_vias_for_unresolved(st, pcb, CLEAR, max_shift=4.0)
        # ON THE BRANCH: the via must actually have moved, or the carry-over
        # branch never ran and this test would pass vacuously.
        self.assertEqual(len(moves), 1, 'no via was relocated')
        # #747: the nudger REPORTS, _Repair applies. Both halves are asserted
        # -- that the report left the graded view alone, and that handing it
        # to the registrar performs the carry-over this test is about.
        self.assertIs(st.vias[0], t0,
                      'the nudger mutated the graded via view')
        self.assertEqual(st.relocate_vias(moves), 1)
        self.assertEqual(len(st.vias), 1)
        moved = st.vias[0]
        self.assertNotEqual((moved[0], moved[1]), (t0[0], t0[1]))
        rec = st._via_radius_by_id.get(id(moved))
        self.assertIsNotNone(rec, 'the relocated via lost its radius')
        self.assertAlmostEqual(rec[1], 0.25, places=9)
        self.assertIs(rec[0], moved,
                      'the map does not hold the tuple it keys on -- a '
                      'recycled id would return another via\'s radius')
    # MUTATION: drop the `radius=` carry-over in `_Repair._register_via`'s
    # relocation caller -> rec is None. (#747 moved it there; before that it
    # was a hand-written carry-over inside the nudger.)

    def test_the_nudger_grades_a_COPPERLESS_cap_pad_flat_too(self):
        """The copper-less-pad rule must not stop at the eff builders. The
        nudger resolves through the same helpers, so without the marker it
        charges a phantom the grader does not -- handing itself a via nothing
        flagged and printing an operator-facing line naming a non-violation."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'nudgeflat', classes=_default_class(0.4))
            pcb = parse_kicad_pcb(p)
            fp = pcb.footprints[ANCHOR_CAP]
            real = [q for q in fp.pads if L._pad_carries_copper(q)]
            hole = copy.deepcopy(real[1])
            hole.pad_type = 'np_thru_hole'
            hole.drill = 1.0
            hole.layers = ['*.Cu', '*.Mask']
            hole.local_clearance = 1.0
            fp.pads[fp.pads.index(real[1])] = hole
            st = _repair(p, pcb=pcb)
            cap = st.caps[ANCHOR_CAP]
            flat_i = [k for k in range(len(cap.pad_floors))
                      if not cap.pad_layers[k]]
            # ON THE BRANCH: there must be exactly one copper-less pad, and the
            # board must be declaring, or the phantom cannot arise.
            self.assertEqual(len(flat_i), 1)
            self.assertIsNotNone(st._floors)
            seen = []
            real_req = st.via_required

            def recording(pad_floor, via_net):
                seen.append(pad_floor)
                return real_req(pad_floor, via_net)
            st.via_required = recording
            nudge_vias_for_unresolved(st, pcb, CLEAR)
            self.assertTrue(seen, 'the nudger never consulted via_required')
            bad = [f for f in seen if f is cap.pad_floors[flat_i[0]]]
            self.assertEqual(bad, [], 'the nudger priced the copper-less pad '
                                      'at its own floor instead of flat')
    # MUTATION: drop the `pad_layers` check in `cap_floors_of` -> the
    # copper-less pad's floor reaches via_required.

    def test_a_duck_typed_state_grades_flat_instead_of_crashing(self):
        """test_617 and test_370 drive the nudger with a _FakeSt/_FakeCap that
        carries none of this machinery.

        The cap and the UNRESOLVED verdict are both load-bearing: with no caps
        the function returns at its pre-existing early return, 58 lines above
        the first `getattr(st, ...)` this change added, and the test proves
        nothing at all. It did exactly that until an audit measured the line
        coverage."""
        pcb = parse_kicad_pcb(INERT)

        class _FakeCap(object):
            pads = [(0.0, 0.0, 0.1, 0.1, 1)]
            x = y = rot = 0.0
            side = 'F'

            def pad_rects(self, *a):
                b = pcb.board_info.board_bounds
                return [(b[0] + 1.0, b[1] + 1.0, b[0] + 1.2, b[1] + 1.2, 1)]

        class _FakeSt(object):
            caps = {'C1': _FakeCap()}
            locked_refs = set()

            def graze_penalty(self, *a):
                return 1.0          # unresolved, so the offender loop RUNS
        st = _FakeSt()
        moves, segs = nudge_vias_for_unresolved(st, pcb, CLEAR)
        self.assertEqual((moves, segs), ([], []))
        # ON THE BRANCH: prove the guarded reads were reached, not skipped.
        self.assertIsNone(getattr(st, '_floors', None))
        self.assertFalse(hasattr(_FakeCap, 'pad_floors'))
    # MUTATION: read `st.required` / `cap.pad_floors` directly instead of via
    # getattr -> AttributeError here, and in test_617 / test_370.


class TestShapeContract(unittest.TestCase):
    """Four tuple shapes are unpacked positionally outside this module. #725
    carries floors in parallel lists and identity maps precisely so that none
    of them had to widen."""

    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.mkdtemp()
        cls.path = _stage(cls.td, 'shape', classes=_default_class(0.4))
        cls.st = _repair(cls.path)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.td, ignore_errors=True)

    def test_the_four_pinned_tuple_widths(self):
        # ON THE BRANCH: grading the FLAT path proves nothing about the shapes.
        self.assertIsNotNone(self.st._floors)
        # every all() below is vacuous on an empty collection, so gate each
        for what, coll, width in (('vias', self.st.vias, 4),
                                  ('segments', self.st.segments, 7),
                                  ('foreign_pads', self.st.foreign_pads, 6)):
            self.assertTrue(coll, 'no %s on this fixture -- the width '
                                  'assertion would be vacuous' % what)
            self.assertTrue(all(len(t) == width for t in coll),
                            '%s is not %d wide' % (what, width))
        self.assertTrue(self.st.caps)
        for cap in self.st.caps.values():
            rects = cap.pad_rects()
            self.assertTrue(rects)
            self.assertTrue(all(len(r) == 5 for r in rects))
    # MUTATION: widen any of them -> animate_fanout_clearance.py and three
    # existing test files stop unpacking.

    def test_pad_floors_stay_index_aligned_through_every_rotation(self):
        cap = self.st.caps[ANCHOR_CAP]
        self.assertEqual(len(cap.pad_floors), len(cap.pads))
        base = [r[4] for r in cap.pad_rects(cap.x, cap.y, 0.0)]
        for rot in (0.0, 90.0, 180.0, 270.0):
            rects = cap.pad_rects(cap.x, cap.y, rot)
            self.assertEqual(len(rects), len(cap.pad_floors))
            self.assertEqual([r[4] for r in rects], base,
                             'pad order changed at rot %g -- floors would '
                             'address the wrong pad' % rot)
    # MUTATION: FILTER inside `_pad_cache_for` (drop a pad) -> the length
    # assertions fail. NB a deterministic SORT is not covered: `base` comes
    # from the same function, so both sides reorder together.

    def test_injecting_st_vias_wholesale_is_graded_AS_INJECTED(self):
        """`st.vias = [...]` is the idiom tests/test_fanout_clearance.py uses.
        The injected tuple carries its own keep-out convention, so it must be
        used verbatim -- deriving the radius by subtracting `_item_reach` off
        element 3 and adding the pair requirement back silently re-prices it.

        Staged with a netclass AND an inner-layer dru rule on purpose: that is
        what makes `_item_reach` (0.5, raised by the rule on the via's all-copper
        scope) differ from `pair` (0.4, the netclass, since the cap pad does not
        share the ruled layer). On a fixture where they coincide, a derived
        radius round-trips by luck and this test would pass either way."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'inject', classes=_default_class(0.4),
                       dru=_dru('In1.Cu', 0.5))
            st = _repair(p)
            cap = st.caps[ANCHOR_CAP]
            own = {q[4] for q in cap.pads}
            foreign = next(n for n in range(1, 50) if n not in own)
            r = cap.pad_rects()[0]
            injected = ((r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0, foreign,
                        0.35 / 2 + CLEAR)
            st.vias = [injected]
            st.cap_vias = {ref: st.vias for ref in st.caps}
            # ON THE BRANCH: the two must actually differ here.
            reach = st._item_reach(st.via_floor(foreign))
            pair = st.via_required(cap.pad_floors[0], foreign)
            self.assertNotAlmostEqual(
                reach, pair, places=6,
                msg='_item_reach == pair on this fixture -- a derived radius '
                    'would round-trip by luck and this test would pass '
                    'whether or not the identity map is used')
            rows = st._via_effs(ANCHOR_CAP, cap, st.cap_vias[ANCHOR_CAP])
            self.assertAlmostEqual(rows[0][0], injected[3], places=9,
                                   msg='the injected keep-out was re-priced')
            self.assertGreater(
                st.via_penalty(cap, cap.x, cap.y, cap.rot,
                               st.cap_vias[ANCHOR_CAP], ref=ANCHOR_CAP), 0.0)
    # MUTATION: derive the radius from `v[3] - _item_reach(...)` instead of the
    # identity map -> rows[0][0] comes out 0.1mm low.

    def test_injecting_cap_foreign_pads_grades_without_misindexing(self):
        """A tuple a test injected carries no registered floor, so it must be
        graded at the FLAT scalar. Without the source-list identity guard the
        memo built during __init__ is reused, and row[0] silently supplies the
        requirement of a completely different pad -- a wrong number that raises
        no error and looks entirely plausible."""
        st = _repair(self.path)
        cap = st.caps[ANCHOR_CAP]
        own = {p[4] for p in cap.pads}
        foreign = next(n for n in range(1, 50) if n not in own)
        r = cap.pad_rects()[0]
        st.cap_foreign_pads[ANCHOR_CAP] = [
            (r[0] - 0.15, r[1], r[0] - 0.05, r[3], foreign, cap.side)]
        rect = st.cap_foreign_pads[ANCHOR_CAP][0][:4]
        got = _short(st, ANCHOR_CAP)
        self.assertIn(foreign, got)
        # Summed over EVERY cap pad, at the FLAT scalar. Graded off a stale row
        # it would come out at the board's declared 0.4 instead.
        flat = sum(max(0.0, CLEAR - _rect_gap(p[:4], rect))
                   for p in cap.pad_rects())
        declared = sum(max(0.0, 0.4 - _rect_gap(p[:4], rect))
                       for p in cap.pad_rects())
        self.assertNotAlmostEqual(flat, declared, places=6)   # not vacuous
        self.assertAlmostEqual(got[foreign], flat, places=6,
                               msg='the injected pad was graded at a stale '
                                   'row rather than the flat scalar')
    # MUTATION: drop the source-list identity guard in `_pad_effs` -> the
    # shortfall reads 0.35.

    def test_via_penalty_without_a_ref_grades_FLAT_as_documented(self):
        """`via_penalty(cap, x, y, rot)` is a public 4-positional path existing
        tests use. Its docstring says the flat scalar is used there. #725
        redefined the tuple's keep-out slot as the prune OVER-reach, so grading
        that slot directly charges more than any pair needs -- and more than
        upstream's slot, which really was radius + clearance."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'noref', classes=_default_class(0.4),
                       dru=_dru('In1.Cu', 0.9))
            st = _repair(p)
            ref = 'C7'          # C20's vias clear both ways; C7's do not
            cap = st.caps[ref]
            vias = st.cap_vias[ref]
            self.assertTrue(vias)
            flat = st.via_penalty(cap, cap.x, cap.y, cap.rot, vias)
            expect = over = 0.0
            for (bx0, by0, bx1, by1, net) in cap.pad_rects():
                for v in vias:
                    if v[2] == net:
                        continue
                    d = _point_to_rect_dist(v[0], v[1], (bx0, by0, bx1, by1))
                    ko = st._via_radius_by_id[id(v)][1] + CLEAR
                    if d < ko - 1e-6:
                        expect += ko - d
                    if d < v[3] - 1e-6:          # the tuple's over-reach slot
                        over += v[3] - d
            # ON THE BRANCH: the two must actually differ on this cap, or the
            # assertion below holds whichever quantity is graded.
            self.assertGreater(over, expect + 1.0,
                               'flat and over-reach coincide here (%.4f vs '
                               '%.4f) -- this would pass vacuously'
                               % (expect, over))
            self.assertAlmostEqual(flat, expect, places=9,
                                   msg='the no-ref path graded at the prune '
                                       'over-reach, not the flat scalar')
    # MUTATION: grade `keepout` directly in via_penalty's flat branch -> the
    # penalty comes out far above the flat scalar.

    def test_ten_positional_arguments_bind_to_the_documented_params(self):
        """Four existing test files construct _Repair with 10 positionals. A
        bare construction asserts nothing -- inserting a DEFAULTED parameter
        still constructs fine and binds every argument to the wrong name. Check
        the signature, then check the values landed."""
        import inspect
        names = list(inspect.signature(_Repair.__init__).parameters)[1:11]
        self.assertEqual(names, ['pcb_data', 'pcb_file', 'clearance',
                                 'grid_step', 'board_edge_clearance',
                                 'near_margin', 'capture_radius',
                                 'default_via_size', 'cap_prefix',
                                 'extra_locked'])
        st = _Repair(parse_kicad_pcb(BOARD), BOARD, CLEAR, 0.1, 0.55, 1.0, 2.0,
                     0.3, 'C', set())
        self.assertEqual(st.clearance, CLEAR)
        self.assertEqual(st.grid_step, 0.1)
        self.assertEqual(st.capture_radius, 2.0)
        self.assertEqual(st._cap_prefixes, ('C',))
    # MUTATION: insert ANY parameter (defaulted or not) before `extra_locked`
    # -> the name list differs, and the values land on the wrong attributes.


class TestInertness(unittest.TestCase):
    """A board declaring nothing must take the IDENTICAL flat path -- a
    stronger claim than 'it gives the same answer'."""

    @staticmethod
    def _counting():
        hits = [0]
        orig = PadClearanceModel.pair

        def counting(self, a, b):
            hits[0] += 1
            return orig(self, a, b)
        return hits, orig, counting

    def test_the_model_is_never_consulted_on_an_inert_board(self):
        """A result comparison alone passes even if the model runs and returns
        `base` every time, which would be a real (perf) regression hiding
        behind a correct answer."""
        hits, orig, counting = self._counting()
        PadClearanceModel.pair = counting
        try:
            r = repair_fanout_clearance(parse_kicad_pcb(INERT), INERT,
                                        clearance=CLEAR)
        finally:
            PadClearanceModel.pair = orig
        self.assertEqual(hits[0], 0,
                         'the model was consulted %d times on a board that '
                         'declares nothing' % hits[0])
        self.assertEqual(len(r['placements']), INERT_PLACEMENTS)
        self.assertEqual(sorted(r['unresolved']), INERT_UNRESOLVED)
        self.assertEqual(r['required'], [])
        self.assertEqual(r['clearance_notes'], [])
    # MUTATION: drop the `.active` gate in `_Repair.__init__` -> hits > 0.

    def test_the_locked_warning_still_names_a_COPPERLESS_NPTH_pad(self):
        """The locked-part warning exists to surface a foreign via inside a
        locked pad's keep-out. A via inside a 1.7mm mounting hole is exactly
        that, and upstream reported it. #725's copper predicate must decide how
        the pair is PRICED, not whether it is seen -- dropping such pads removed
        real rows on a board that declares nothing at all, which would break
        the inertness claim in this channel."""
        pcb = parse_kicad_pcb(INERT)
        st = _repair(INERT, pcb=pcb)
        # ON THE BRANCH: an inert board, and a locked part to convert.
        self.assertIsNone(st._floors)
        locked = sorted(st.locked_refs)
        self.assertTrue(locked, 'no locked part on this fixture')

        def warn_rows(board):
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                repair_fanout_clearance(board, INERT, clearance=CLEAR)
            return [ln.strip() for ln in buf.getvalue().splitlines()
                    if '<-> via at' in ln]

        # ON THE BRANCH: convert the pad that actually HAS a via in its
        # keep-out, or the row it should keep does not exist either way.
        # (Match the ref token exactly -- 'U8.16'.startswith('U8.1') is True,
        # and a prefix test silently passed on the wrong row while checking
        # this fix.)
        before = warn_rows(pcb)
        self.assertTrue(before, 'no locked-pad warning at all on this fixture')
        ref, pnum = before[0].split(' ')[0].split('.', 1)
        fp = pcb.footprints[ref]
        pad = next(q for q in fp.pads if str(q.pad_number) == pnum)
        pad.pad_type = 'np_thru_hole'
        pad.drill = 1.7
        pad.layers = ['*.Cu', '*.Mask']
        self.assertFalse(L._pad_carries_copper(pad))
        after = warn_rows(pcb)
        named = [r for r in after if r.split(' ')[0] == '%s.%s' % (ref, pnum)]
        self.assertTrue(named,
                        'the copper-less locked pad vanished from the warning: '
                        '%r -> %r' % (before, after))
    # MUTATION: `continue` on `not _pad_carries_copper(p)` in the locked
    # warning -> those rows disappear, on an INERT board.

    def test_the_counter_itself_fires_on_a_declaring_board(self):
        """Without this, the test above also passes if `pair` is simply never
        the method the fix calls."""
        hits, orig, counting = self._counting()
        PadClearanceModel.pair = counting
        try:
            with tempfile.TemporaryDirectory() as td:
                _repair(_stage(td, 'ctr', classes=_default_class(0.4)))
        finally:
            PadClearanceModel.pair = orig
        self.assertGreater(hits[0], 0)
    # MUTATION: make the counter patch a method the fix does not call (e.g.
    # `pair_with_source`) -> this control goes red while the inertness test
    # above still passes, which is the point of having it.


class TestReport(unittest.TestCase):
    def test_required_rows_name_the_source_and_render(self):
        with tempfile.TemporaryDirectory() as td:
            st = _repair(_stage(td, 'rep', classes=_default_class(0.4)))
            rows = st.required_rows()
            self.assertTrue(rows, 'nothing was reported as raised')
            for ref, who, mm, src in rows:
                self.assertIn(src, ('netclass', 'layer rule', 'pad override'))
                self.assertGreater(mm, st.clearance)
            # every source this fixture can produce is 'netclass'; assert that
            # rather than a 3-way `assertIn` that one value satisfies
            self.assertEqual({r[3] for r in rows}, {'netclass'})
            clause = L.format_required_clause({'required': rows})
            self.assertIn('requires', clause)
            self.assertIn('netclass', clause)
            # the mover-vs-mover leg must contribute, or deleting it passes
            self.assertTrue([r for r in rows if r[1].startswith('cap ')],
                            'no mover-vs-mover row -- deleting that leg would '
                            'leave this test green')
    # MUTATION: delete the mover-vs-mover leg of `required_rows` -> the last
    # assertion fails (it left 59 of 77 rows and passed before this guard).

    def test_only_CHARGED_pairs_are_reported(self):
        """An in-reach pair that happens to be clear is not a finding, it is the
        normal case on a declaring board. Reporting all of them buries the real
        ones under hundreds of rows."""
        with tempfile.TemporaryDirectory() as td:
            st = _repair(_stage(td, 'rep2', classes=_default_class(0.4)))
            in_reach = sum(len(st.cap_foreign_pads[r]) + len(st.cap_vias[r])
                           + len(st.cap_segs[r]) for r in st.caps)
            self.assertGreater(in_reach, 1000, 'the fixture got small')
            self.assertLess(len(st.required_rows()), 100,
                            'required_rows is reporting in-reach pairs rather '
                            'than charged ones')
    # MUTATION: drop the `charged` filter in `required_rows` -> hundreds of rows.

    def test_the_report_never_names_an_OFF_LAYER_track_pair(self):
        """`required_rows` must MIRROR what was charged, not re-derive it.
        Before #731 the F/B side collapse put pairs sharing no copper layer
        into `cap_segs` at all, `_seg_effs` priced them at the flat scalar, and
        a report resolving the requirement independently named them anyway, at
        the netclass -- up to 43% of the disclosure on rp2350, every row
        sending the reader after a violation check_drc does not grade.

        #731 removes the pair rather than repricing it, so the guard in
        `best()` is now belt-and-braces for the real prune and load-bearing
        only for a MIXED-layer cap (an SMD pad beside a through-hole one),
        where the union prune keeps a track only some pads share. The spy that
        used to require an off-layer pair to exist therefore moved to the OLD
        side prune; the per-row `shared` assertion below is unchanged and is
        what still discriminates."""
        with tempfile.TemporaryDirectory() as td:
            p = _stage(td, 'offrep', classes=_default_class(0.4), src=INERT)
            pcb = parse_kicad_pcb(p)
            st = _repair(p, pcb=pcb)
            names = {n.net_id: n.name for n in pcb.nets.values()}
            by_name = {v: k for k, v in names.items()}
            rows = st.required_rows(names)
            tracks = [r for r in rows if r[1].startswith('track ')]
            # ON THE BRANCH: there must BE track rows, or this proves nothing.
            self.assertTrue(tracks, 'no track rows at all on this fixture')
            # ...and the OLD prune must have admitted off-layer pairs here, or
            # the phantom this guard is about never existed on this fixture.
            old_off = sum(
                1 for ref, cap in st.caps.items()
                for t in st.segments
                if st._seg_side(t[6]) == cap.side
                and not any(t[6] in pl for pl in cap.pad_layers))
            self.assertGreater(old_off, 0, 'the OLD side prune admitted no '
                                           'off-layer pair -- spy proves nothing')
            # ...and the NEW prune must admit none.
            off_pairs = sum(
                1 for ref, cap in st.caps.items()
                for t in st.cap_segs[ref]
                if not any(t[6] in pl for pl in cap.pad_layers))
            self.assertEqual(off_pairs, 0,
                             '%d off-layer pair(s) survived the prune' % off_pairs)
            for ref, who, mm, src in tracks:
                nid = by_name.get(who[6:])
                cap = st.caps[ref]
                shared = any(
                    any(t[6] in pl for pl in cap.pad_layers)
                    for t in st.cap_segs[ref] if t[4] == nid)
                self.assertTrue(shared,
                                '%s <-> %s shares no copper layer, so it is '
                                'not charged and must not be reported' % (ref, who))
    # MUTATION: drop the `_seg_shares` guard in `required_rows`' `best()` ->
    # a mixed-layer cap's phantom rows come back. MUTATION: revert the prune to
    # the side collapse -> `off_pairs` is no longer 0.

    def test_an_unreadable_declaration_is_disclosed_not_silent(self):
        """A FAILED read is exactly what makes a model look INERT, so the notes
        must be captured BEFORE the active-drop.

        Staged on the board with NO pad overrides on purpose: on a board that
        has some, the failed netclass read still leaves the model active and
        the notes survive by accident, so the ordering is never tested."""
        import list_nets
        orig = list_nets.net_clearance_map

        def boom(*a, **k):
            raise IOError('staged failure')
        list_nets.net_clearance_map = boom
        try:
            with tempfile.TemporaryDirectory() as td:
                p = _stage(td, 'unread', classes=_default_class(0.4), src=INERT)
                st = _repair(p)
                # ON THE BRANCH: the failure must leave the model INACTIVE, or
                # the notes survive for the wrong reason.
                self.assertIsNone(st._floors,
                                  'the model stayed active -- the ordering is '
                                  'not under test on this fixture')
                self.assertTrue(st.clearance_notes,
                                'the failed netclass read was silent')
        finally:
            list_nets.net_clearance_map = orig
    # MUTATION: read `model.notes` after `if not model.active: model = None`
    # -> AttributeError, or the notes are lost.


class TestAnimatorRecorderStillUnpacks(unittest.TestCase):
    """tests/test_431_animator_port.py drives render_gif with a SYNTHETIC
    recorder and never touches the real `_Recorder`, so it passes even if a
    shape change breaks the shipped animator. This is the only coverage."""

    def test_the_real_on_move_recorder_survives(self):
        import animate_fanout_clearance as AFC
        rec = AFC._Recorder()
        repair_fanout_clearance(parse_kicad_pcb(INERT), INERT, clearance=CLEAR,
                                max_displacement=0.0, max_passes=1, on_move=rec)
        self.assertTrue(rec.static['vias'], 'no vias were recorded')
        self.assertTrue(all(len(v) == 4 for v in rec.static['vias']))
    # MUTATION: widen st.vias to 5-tuples -> ValueError inside _Recorder.


class TestParityGateRegistration(unittest.TestCase):
    def test_place_fanout_clearance_is_a_scanned_CLI_main(self):
        """tests/gui_parity/** is NOT collected by run_all.py's flat glob, so
        nothing in the default suite runs that lint. Assert it here."""
        with open(os.path.join(_TESTS, 'gui_parity',
                               'test_cli_postpass_coverage.py'),
                  encoding='utf-8') as fh:
            self.assertIn('py_placer/place_fanout_clearance.py', fh.read())
    # MUTATION: remove it from CLI_MAINS -> the cap-repair CLI's post-passes go
    # unscanned again.


if __name__ == '__main__':
    unittest.main(verbosity=2)
