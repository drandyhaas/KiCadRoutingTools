# -*- coding: utf-8 -*-
"""#741: a nudged via must keep EXACTLY the protection the board gave it.

THE DEFECT, in one sentence with two halves. The #313 via-nudge relocates a
REAL, pre-existing via to free a boxed-in decoupling cap: the writer deletes it
from the board text (`plane_io._remove_vias_at_positions`) and re-emits it
0.15mm away. A via that comes off the board and goes back on must return with
the protection the board gave it -- NO MORE and NO LESS. Both halves were
wrong.

MEASURED on the rig below, at HEAD~1 (the defect) and HEAD (the fix):

    via   input spec                    BEFORE                      AFTER
    ----  ----------------------------  --------------------------  -----------
    VF1   covering (front no)(back no)  {'tenting':                 == input,
          plugging (front no)(back no)   '(front yes) (back yes)'}   token for
          capping yes / filling yes     -- the WHOLE spec destroyed  token
    VF2   {} (inherits the board)       {'tenting':                 {}
                                         '(front yes) (back yes)'}
                                        -- GAINED a fab attribute
    VF3   Type VII, never nudged        unchanged                   unchanged

So a via-in-pad needing IPC-4761 Type VII (filled + capped + plated) shipped
TENTED, silently, from a pass whose stated job is to tidy clearance (half one);
and a via that had been inheriting the board's `(setup (tenting ...))` came back
with an explicit token it never carried (half two).

Half two is also a CLI/GUI DIVERGENCE, which is what makes it part of this
failure mode rather than a separate taste question: `gui_utils
.apply_via_protection` returns early on an empty spec, because pcbnew's
`*_MODE_FROM_BOARD` already means inherit. The GUI has never re-stamped such a
via. Only the text writer did.

THE AUTHORITY here is a RE-PARSE of the file the writer actually produced --
never the pass's own `via_moves` dict, which is half of the thing being fixed.
Deliberately NOT check_drc: DRC grades geometry, and a tented via and an
untented one are geometrically identical, so DRC is blind to this by
construction. Geometry is pinned separately, by the exact landings in every
setUp and by the whole-file-minus-via-blocks diff.

WHY THE FIXTURE IS AN AUTHORED BOARD TEXT, and not a tracked board or a
`synth.make_pcb()`. Three measured reasons, all of which would otherwise make
this file lie:

  * NO tracked board carries a non-tenting per-via spec. Exactly two carry a
    per-via spec at all -- orangecrab_ext_pll (136 vias) and
    rp2350_fpga_eensy_prePlane (70) -- and BOTH carry only
    `(tenting (front yes) (back yes))`, which is byte-for-byte
    `kicad_writer.DEFAULT_VIA_TENTING`. On the corpus the defect's output and
    the correct output are THE SAME STRING. The only Type-VII example in the
    repo is the inline `REAL_VIA` string in tests/test_489_via_tenting.py,
    which is not a board.
  * A `synth.make_pcb()` board leaves `net_id_to_name` empty, so
    `placement/writer.py`'s `n2n` is falsy and every via is emitted with a
    NUMERIC net token. Note what that does and does not break, because the
    obvious reading is wrong: a real spec is still emitted (the `net_name`
    gate guards only the DEFAULT), so half one would appear to work -- but the
    numeric-net parser defect described below then drops the whole via at
    re-parse, and the spec-less half collapses to silence. An arm built that
    way grades a via that is not in the model.
  * The vias MUST carry `(uuid "...")`. `kicad_parser
    ._extract_via_protection_attrs` keys specs by uuid and `continue`s on a
    uuid-less via, so the spec would arrive `{}` and every assertion below
    would pass on `{} == {}`. Every setUp asserts the INPUT spec first.

And the vias use the KiCad-10 `(net "NAME")` dialect deliberately. Measured
while building this rig, and filed separately: the KiCad-9 NUMERIC via regex in
`kicad_parser.extract_vias` has strict field ordering, while its v10 twin was
given a flexible `.*?` gap for exactly these tokens -- so a numeric-net via
carrying ANY protection token does not merely lose its spec, it VANISHES from
the model entirely. Zero tracked boards are affected, so it is latent, but this
file must not be the place it hides.

THE RIG, and every margin it is worth.

    board            60 x 20, four Edge.Cuts lines
    U1               BGA-9, 3x3 @ 0.8mm, 0.4mm balls, centred (7.6, 10.0)
                     -> ball-field copper ends at x = 8.6
    C1               (10.0,  9.5), 0.6mm square pads at local +-0.5
    C2               (10.0, 10.5), same
    VF1              (11.175,  9.5) Type-VII spec, uuid 1111...   -> NUDGED
    VF2              (11.175, 10.5) NO spec,       uuid 2222...   -> NUDGED
    VF3              (30.0,  10.0) Type-VII spec,  uuid 3333...   -> untouched
    via size/drill   0.8 / 0.3  ->  copper radius 0.40

  * the offending gap is 0.375 (C1 pad 2's edge at x=10.8 to the via centre)
    against a keep-out of 0.40 + 0.10 = 0.500: 0.125 INSIDE, so 0.125 of
    margin. The refusal arm runs the same board at clearance 0.7; measured, the
    refusal begins between 0.50 and 0.55, so that arm has ~0.15 of margin. Both
    are outside the 0.05mm fixture rule.
  * the landing (11.325) clears the keep-out edge (11.300) by 0.025. That is
    INSIDE the 0.05 rule and it is disclosed rather than hidden: it is the
    spiral search's own radial quantum (`r += 0.05`), not a fixture choice, and
    NO assertion measures that clearance. The landings are asserted EXACTLY, so
    they are change detectors for precisely that number.
  * C1's pads start at x=9.2 and the ball field ends at x=8.6 -> 0.4 of slack
    against `near_margin=1.0`. Nothing asserts the near-BGA test; every setUp
    asserts `sorted(st.caps) == ['C1','C2']` so a change fails loudly instead.
  * C1 and C2's pad rows are 0.4mm apart. Nothing measures that.

`max_displacement_cap=0.0` is load-bearing: without it the `via_clear_fallback`
leg searches `_candidate_positions(cap, max_displacement_cap, ...)` -- the CAP
budget, not the per-cap one -- relocates the cap, and the nudger is never
called, so every arm here would assert about nothing. `via_clear_fallback=False`
belts the same brace in-process; the CLI arm cannot pass it and passes
`--max-displacement-cap 0` instead.

TWO HAZARDS this file must not fall into, both measured:

  * NET IDS ARE NOT STABLE ACROSS THE WRITE. The writer deletes the moved vias
    and APPENDS them, so the output via order is VF3, VF1, VF2 and the ids are
    re-synthesised. Every lookup here is by net NAME through
    `pcb.nets[v.net_id].name` -- never by index, never by uuid.
  * THE RE-EMITTED VIA GETS A FRESH UUID (`generate_via_sexpr` mints one),
    which is why CLAUDE.md forbids whole-file diffing a `.kicad_pcb` and why
    the "nothing else changed" arm diffs `_strip_via_blocks(out)` against
    `_strip_via_blocks(inp)` instead.

THE BATTERY: 32 mutations, RUN rather than reasoned about, with the killer
named beside each arm and the full table at the bottom of this file. 29 killed,
THREE declared survivors -- recorded, not papered over. Two further survivors
were found by an adversarial review of this file and a mutation run, and were
CLOSED by arms added afterwards rather than declared away. Three arms overlap
tests/test_489_via_tenting.py, which owns those values; they are kept only as
end-to-end change detectors and are marked as such.

RUNTIME: ~14s warm, and it is almost all one class -- measured 13.8/13.9/13.8
over three runs, of which TestInertOnTheTrackedCorpus is 13.5. A cold cache is
much worse: an independent reviewer measured 57s on a fresh checkout, because
that class parses and drives the engine over 22 boards x 2 clearances. That is
why this file does NOT declare RUN_ALL_FAST_OK -- run_all.py's rule is that a
test driving a routing chain belongs in the integration bucket whatever it
imports. The rest (rig, 34 other arms, one CLI subprocess) is under a second.
"""

# NO RUN_ALL_FAST_OK. Measured six runs: 54-67s, of which
# TestInertOnTheTrackedCorpus is ~50s -- it drives the whole engine over 22
# boards x 2 clearances. run_all.py's own rule is that a test which drives a
# routing chain belongs in the integration bucket whatever it imports, and the
# opt-out is documented for ~4s unit tests. Declaring both the opt-out and a
# 900s timeout would have been the two declarations disagreeing out loud.
RUN_ALL_TIMEOUT = 900

import contextlib
import glob
import inspect
import io
import os
import re
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p) if _p else _ROOT
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import run_utils                                                # noqa: E402
import kicad_parser as KP                                       # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from kicad_writer import (DEFAULT_VIA_TENTING,                  # noqa: E402
                          VIA_PROTECTION_TOKEN_ORDER,
                          generate_via_sexpr)
from placement import fanout_clearance as FC                    # noqa: E402
from placement.fanout_clearance import repair_fanout_clearance  # noqa: E402
from placement.writer import write_placed_output                # noqa: E402
from synth import make_via                                      # noqa: E402

# --------------------------------------------------------------------- the rig

CLEAR = 0.1          # the value place_fanout_clearance.py's own help example uses
CLEAR_BOXED = 0.7    # the refusal arm; measured, refusal begins between .50/.55
V_SIZE, V_DRILL = 0.8, 0.3
VR = V_SIZE / 2.0                    # 0.40 copper radius
VIA_KEEPOUT = VR + CLEAR             # 0.50, what valid_via_pos needs
PAD_HALF = 0.3                       # 0.6mm square cap pads at local +-0.5

BGA_XY = (7.6, 10.0)
CAP1_XY = (10.0, 9.5)
CAP2_XY = (10.0, 10.5)
VIA_X = 11.175                       # gap 0.375 to pad 2's edge (10.8)
CTRL_XY = (30.0, 10.0)
LAND_VF1 = (11.325, 9.5)             # MEASURED
LAND_VF2 = (11.325, 10.5)            # MEASURED

UUID_VF1 = '11111111-1111-1111-1111-111111111111'
UUID_VF2 = '22222222-2222-2222-2222-222222222222'
UUID_VF3 = '33333333-3333-3333-3333-333333333333'

# MULTI-LINE on purpose: that is the shape real KiCad writes (see the hackrf
# block in tests/test_489_via_tenting.py), so it exercises
# _balanced_token_text's whitespace collapse on the way IN and
# via_protection_sexpr's on the way OUT.
TYPE_VII_TEXT = (
    '\t\t(covering\n\t\t\t(front no)\n\t\t\t(back no)\n\t\t)\n'
    '\t\t(plugging\n\t\t\t(front no)\n\t\t\t(back no)\n\t\t)\n'
    '\t\t(capping yes)\n'
    '\t\t(filling yes)')

# ...and this is what the PARSER yields from it (it normalises whitespace
# itself), so input and output specs compare with a plain ==.
TYPE_VII = {'covering': '(front no) (back no)',
            'plugging': '(front no) (back no)',
            'capping': 'yes',
            'filling': 'yes'}

_LAYERS = '\n'.join([
    '\t(layers',
    '\t\t(0 "F.Cu" signal)',
    '\t\t(2 "B.Cu" signal)',
    '\t\t(9 "F.Adhes" user "F.Adhesive")',
    '\t\t(11 "F.Paste" user)',
    '\t\t(13 "F.SilkS" user "F.Silkscreen")',
    '\t\t(15 "F.Mask" user)',
    '\t\t(17 "B.Mask" user)',
    '\t\t(44 "Edge.Cuts" user)',
    '\t\t(46 "F.CrtYd" user "F.Courtyard")',
    '\t\t(49 "F.Fab" user)',
    '\t)'])


def _edge(x1, y1, x2, y2, uu):
    return ('\t(gr_line\n\t\t(start %s %s)\n\t\t(end %s %s)\n'
            '\t\t(stroke (width 0.05) (type solid))\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "%s")\n\t)'
            % (x1, y1, x2, y2, uu))


def _pad(num, lx, ly, w, h, net_id, net_name, shape='rect'):
    return ('\t\t(pad "%s" smd %s\n\t\t\t(at %s %s)\n\t\t\t(size %s %s)\n'
            '\t\t\t(layers "F.Cu" "F.Paste" "F.Mask")\n'
            '\t\t\t(net %d "%s")\n\t\t)'
            % (num, shape, lx, ly, w, h, net_id, net_name))


def _cap(ref, x, y, na, nb, ia, ib, uu):
    # No F.CrtYd polygon: extract_courtyard_bboxes then returns {} and every cap
    # falls back to its pad bbox -- the same regime test_736's one-line _stub()
    # produces, so the two files' rigs are comparable.
    return '\n'.join([
        '\t(footprint "Capacitor_SMD:C_0402_1005Metric"',
        '\t\t(layer "F.Cu")',
        '\t\t(uuid "%s")' % uu,
        '\t\t(at %s %s)' % (x, y),
        '\t\t(property "Reference" "%s" (at 0 -1 0) (layer "F.SilkS")'
        ' (uuid "%s-r") (effects (font (size 0.5 0.5) (thickness 0.1))))'
        % (ref, uu),
        _pad('1', -0.5, 0, 0.6, 0.6, ia, na),
        _pad('2', 0.5, 0, 0.6, 0.6, ib, nb),
        '\t)'])


def _bga(ref, x, y, uu):
    # Named BGA-* so detect_package_type returns 'BGA' by NAME, which is what
    # puts it in st.bga_refs and makes C1/C2 "movable near-BGA caps" at all.
    rows, n = [], 10
    for i in range(3):
        for j in range(3):
            rows.append(_pad('%s%d' % ('ABC'[i], j + 1),
                             (j - 1) * 0.8, (i - 1) * 0.8, 0.4, 0.4,
                             n, 'BALL%d' % n, shape='circle'))
            n += 1
    return '\n'.join([
        '\t(footprint "Package_BGA:BGA-9_1.4x1.4mm_Layout3x3_P0.8mm"',
        '\t\t(layer "F.Cu")',
        '\t\t(uuid "%s")' % uu,
        '\t\t(at %s %s)' % (x, y),
        '\t\t(property "Reference" "%s" (at 0 -1.5 0) (layer "F.SilkS")'
        ' (uuid "%s-r") (effects (font (size 0.5 0.5) (thickness 0.1))))'
        % (ref, uu),
    ] + rows + ['\t)'])


def _via(x, y, net, uu, spec=''):
    s = ('\n' + spec) if spec else ''
    return ('\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n'
            '\t\t(layers "F.Cu" "B.Cu")%s\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
            % (x, y, V_SIZE, V_DRILL, s, net, uu))


def _board_text():
    return '\n'.join([
        '(kicad_pcb',
        '\t(version 20240108)',
        '\t(generator "pcbnew")',
        '\t(generator_version "8.0")',
        '\t(general\n\t\t(thickness 1.6)\n\t)',
        '\t(paper "A4")',
        _LAYERS,
        # NO board-level (tenting ...) default on purpose: the defect is about
        # the PER-VIA spec, and a board default would hand a reader a second
        # explanation for the same output.
        '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)',
        '\t(net 0 "")',
        '\t(net 1 "NA1")',
        '\t(net 2 "NB1")',
        '\t(net 3 "NA2")',
        '\t(net 4 "NB2")',
        # VF1/VF2/VF3 are FOREIGN to every cap pad -- graze_penalty only counts
        # foreign copper, so a via sharing a cap pad's net is not an offender
        # and the nudger is never reached.
        '\t(net 5 "VF1")',
        '\t(net 6 "VF2")',
        '\t(net 7 "VF3")',
    ] + ['\t(net %d "BALL%d")' % (n, n) for n in range(10, 19)] + [
        _edge(0, 0, 60, 0, 'e1'), _edge(60, 0, 60, 20, 'e2'),
        _edge(60, 20, 0, 20, 'e3'), _edge(0, 20, 0, 0, 'e4'),
        _bga('U1', BGA_XY[0], BGA_XY[1], 'aaaaaaaa-0000-0000-0000-00000000u001'),
        _cap('C1', CAP1_XY[0], CAP1_XY[1], 'NA1', 'NB1', 1, 2,
             'cccccccc-0000-0000-0000-0000000000c1'),
        _cap('C2', CAP2_XY[0], CAP2_XY[1], 'NA2', 'NB2', 3, 4,
             'cccccccc-0000-0000-0000-0000000000c2'),
        _via(VIA_X, 9.5, 'VF1', UUID_VF1, TYPE_VII_TEXT),
        _via(VIA_X, 10.5, 'VF2', UUID_VF2),
        _via(CTRL_XY[0], CTRL_XY[1], 'VF3', UUID_VF3, TYPE_VII_TEXT),
        ')'])


RIG_BOARD = _board_text()

# --------------------------------------------------------------------- helpers

_TD = None
_RIGS = {}


def setUpModule():
    global _TD, _RIGS
    _TD, _RIGS = tempfile.TemporaryDirectory(), {}


def tearDownModule():
    _TD.cleanup()


class _Rig(object):
    pass


def _stage(subdir):
    d = os.path.join(_TD.name, subdir)
    os.makedirs(d, exist_ok=True)
    src = os.path.join(d, 'rig.kicad_pcb')
    with io.open(src, 'w', encoding='utf-8', newline='') as f:
        f.write(RIG_BOARD)
    # A check whose INPUT is missing tests nothing (CLAUDE.md / run_utils).
    run_utils.evidence(src, 'the authored #741 rig board')
    return d, src


def _rig(clearance=CLEAR):
    """Stage RIG_BOARD, run the REAL pass, write through the REAL writer,
    re-parse. Memoised per clearance: it is well under a second, but every
    class wants it and re-running would make the numbers look run-dependent."""
    if clearance in _RIGS:
        return _RIGS[clearance]
    d, src = _stage('c%g' % clearance)
    pcb = parse_kicad_pcb(src)
    seen, buf = [], io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = repair_fanout_clearance(
            pcb, src, clearance=clearance, cap_prefix='C',
            max_displacement=0.0, max_displacement_cap=0.0, max_passes=1,
            allow_rotations=False, via_clear_fallback=False,
            on_move=lambda st: seen.append(st))
    dst = os.path.join(d, 'out.kicad_pcb')
    wrote = write_placed_output(src, dst, res['placements'],
                                via_moves=res.get('via_moves'),
                                new_segments=res.get('new_segments'),
                                pcb_data=pcb)
    r = _Rig()
    r.src, r.dst, r.pcb, r.res, r.wrote = src, dst, pcb, res, wrote
    r.st = seen[0] if seen else None
    r.out = buf.getvalue()
    r.in_text = RIG_BOARD
    if wrote:
        with io.open(dst, encoding='utf-8') as f:
            r.out_text = f.read()
    else:
        r.out_text = None
    r.out_pcb = (parse_kicad_pcb(dst) if wrote else None)
    _RIGS[clearance] = r
    return r


def _via_by_net(pcb, name):
    """Identity is by NET NAME. Never by index (the writer reorders) and never
    by uuid (the re-emitted via gets a fresh one)."""
    hits = [v for v in pcb.vias
            if v.net_id in pcb.nets and pcb.nets[v.net_id].name == name]
    assert len(hits) == 1, '%d vias on net %r, expected exactly 1' % (
        len(hits), name)
    return hits[0]


def _via_blocks(text):
    """[(text, net_name_or_None)] for every balanced `(via ...)` block."""
    out = []
    for m in re.finditer(r'\(via(?=[\s(])', text):
        depth = 0
        for i, c in enumerate(text[m.start():]):
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    blk = text[m.start():m.start() + i + 1]
                    nm = re.search(r'\(net\s+"([^"]*)"\)', blk)
                    out.append((blk, nm.group(1) if nm else None))
                    break
    return out


def _via_block(text, net_name):
    hits = [b for b, n in _via_blocks(text) if n == net_name]
    assert len(hits) == 1, '%d via blocks for %r' % (len(hits), net_name)
    return hits[0]


def _strip_via_blocks(text):
    for blk, _ in _via_blocks(text):
        text = text.replace(blk, '')
    return text


def _read(path):
    with io.open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def _prot_tokens(block):
    """The protection tokens present in one raw `(via ...)` block, in order."""
    return [m.group(1) for m in re.finditer(
        r'\((tenting|covering|plugging|capping|filling)[\s)]', block)]


def _code(obj):
    """Source lines with trailing comments stripped, line numbering preserved.
    placement/writer.py's comment block names `generate_via_sexpr` in prose;
    reading raw lines would let a comment arm or disarm every guard below."""
    return [l.split('#')[0] for l in inspect.getsource(obj).splitlines()]


def _calls(lines, name):
    """[(line, argument text)] per `name(...)` call. writer.py's call is
    MULTI-LINE, so a line-wise `in` test cannot see its argument list."""
    text, out = '\n'.join(lines), []
    for m in re.finditer(re.escape(name) + r'\s*\(', text):
        i = text.index('(', m.start() + len(name))
        depth = 0
        for j in range(i, len(text)):
            if text[j] == '(':
                depth += 1
            elif text[j] == ')':
                depth -= 1
                if depth == 0:
                    out.append((text.count('\n', 0, m.start()) + 1,
                                text[i + 1:j]))
                    break
    return out


# -------------------------------------------------------------------- the arms

class TestATypeVIIViaSurvivesTheNudge(unittest.TestCase):
    """THE HEADLINE. A via carrying IPC-4761 Type VII -- filled + capped +
    plated, and deliberately NOT tented -- is nudged 0.15mm to tidy C1's
    clearance, and came back TENTED with its whole spec gone.

    MEASURED at HEAD~1: {'tenting': '(front yes) (back yes)'} in place of the
    four tokens the board carried."""

    def setUp(self):
        self.rig = _rig(CLEAR)
        out = self.rig.out
        # PRINTED OUTPUT FIRST (#732's lesson): a pass that never looked prints
        # nothing, and is otherwise indistinguishable from one that looked and
        # agreed.
        self.assertIn('via-nudge: moved VF1 via (11.175,9.500) -> '
                      '(11.325,9.500) to free C1', out)
        self.assertEqual(sorted(self.rig.st.caps), ['C1', 'C2'],
                         'the cap set changed: cap_prefix or the near-BGA test '
                         'moved, and this rig no longer proves what it says')
        self.assertEqual(len(self.rig.res['via_moves']), 2, out)
        self.assertEqual(self.rig.res['new_segments'], [],
                         'a connector appeared: conn_layers is no longer empty, '
                         'so this rig now also exercises #736 and its margins '
                         'have to be re-derived')
        self.assertTrue(self.rig.wrote, 'the writer refused; nothing to grade')
        # VACUITY GUARD: the INPUT really carries a spec. Without the uuid,
        # _extract_via_protection_attrs skips the via and every assertion below
        # would pass on {} == {}.
        # Re-parsed from disk, NOT rig.pcb: the pass mutates that object in
        # place (v.x, v.y = nx, ny), so calling it "the input" would be reading
        # post-pass state and asserting a spec is unmutated on the very object
        # a mutation could have mutated.
        src = _via_by_net(parse_kicad_pcb(self.rig.src), 'VF1')
        self.assertEqual(src.tenting_attrs, TYPE_VII)
        self.assertNotIn('tenting', src.tenting_attrs)

    def test_the_reparsed_output_via_keeps_its_spec_token_for_token(self):
        v = _via_by_net(self.rig.out_pcb, 'VF1')
        self.assertEqual((round(v.x, 4), round(v.y, 4)), LAND_VF1,
                         'the via did not land where it lands today -- assert '
                         'the geometry before asserting about the spec')
        self.assertEqual(
            v.tenting_attrs, TYPE_VII,
            'the nudge re-stamped the via: a via-in-pad needing IPC-4761 Type '
            'VII shipped with a different fab spec after a move whose stated '
            'job was to tidy clearance')
        self.assertNotIn('tenting', v.tenting_attrs)
    # MUTATION 1: drop the 'tenting_attrs' key from the move dict.
    # MUTATION 2: drop `tenting_attrs=` at writer.py's emit.
    # MUTATION 3: pass None there.  MUTATION 4: pass {} there.

    def test_the_emitted_block_collapses_the_spec_and_orders_it_canonically(self):
        """`via_protection_sexpr` collapses the parsed inner text onto one line
        and emits VIA_PROTECTION_TOKEN_ORDER. Asserted from RAW TEXT, because
        the parser normalises whitespace on the way back in and a re-parse can
        see neither.

        OVERLAP, recorded rather than hidden: tests/test_489_via_tenting.py
        owns both contracts at the generate_via_sexpr UNIT level and kills both
        mutations too. This arm is the END-TO-END change detector -- the only
        one that would catch py_placer growing an emitter of its own."""
        blk = _via_block(self.rig.out_text, 'VF1')
        self.assertNotIn('(tenting', blk, blk)
        self.assertIn('(covering (front no) (back no))', blk, blk)
        self.assertIn('(plugging (front no) (back no))', blk, blk)
        self.assertEqual(_prot_tokens(blk),
                         ['covering', 'plugging', 'capping', 'filling'], blk)
    # MUTATION 14: reorder VIA_PROTECTION_TOKEN_ORDER.
    # MUTATION 15: drop the `" ".join(inner.split())` collapse.

    def test_the_output_has_a_fresh_uuid_and_still_only_three_vias(self):
        """Two failure modes that would make the headline lie in OPPOSITE
        directions: a re-emitted via with no uuid loses its spec at the next
        parse, and a removal that missed leaves a STACKED duplicate -- one
        tented, one not -- which no DRC will ever flag."""
        self.assertEqual(len(self.rig.out_pcb.vias), 3,
                         'the moved via was appended without removing the '
                         'original: a stacked same-net barrel pair')
        blk = _via_block(self.rig.out_text, 'VF1')
        m = re.search(r'\(uuid "([^"]+)"\)', blk)
        # presence FIRST: without it, deleting the (uuid ...) line reports an
        # AttributeError on None instead of this arm's authored reason.
        self.assertIsNotNone(m, 'the re-emitted via carries NO uuid, so it can '
                                'never receive a protection spec at the next '
                                'parse (_extract_via_protection_attrs keys by '
                                'uuid)')
        got = m.group(1)
        self.assertTrue(got)
        self.assertNotEqual(got, UUID_VF1,
                            'the re-emitted via reuses the input uuid; two '
                            'elements now claim one identity')
    # MUTATION 8: drop the (uuid ...) line from generate_via_sexpr.
    # MUTATION 9: re-emit the uuid unchanged.
    # MUTATION 10: skip the _remove_vias_at_positions call.

    def test_NOTHING_outside_the_rewritten_via_blocks_changed(self):
        """CLAUDE.md forbids whole-file diffing a .kicad_pcb because outputs
        carry per-run uuids. Scoped so it is legitimate: the via blocks are the
        only place a fresh uuid appears here, and they are compared field for
        field by the other arms."""
        self.assertEqual(_strip_via_blocks(self.rig.out_text),
                         _strip_via_blocks(self.rig.in_text))
    # MUTATION: none (control). The whole-file statement that this fix changes
    # no geometry and nothing else.


class TestASpecLessViaEmitsNothingAndInherits(unittest.TestCase):
    """The OTHER half, and the one that is a CLI/GUI divergence rather than a
    lost attribute. VF2 is nudged on the same board, the same distance, in the
    same pass -- and says nothing about protection, because it inherits the
    board's `(setup (tenting ...))`.

    MEASURED at HEAD~1: it came back carrying an explicit
    {'tenting': '(front yes) (back yes)'} it never had. The GUI twin has never
    done that -- gui_utils.apply_via_protection returns early on an empty spec
    because pcbnew's *_MODE_FROM_BOARD already means inherit."""

    def setUp(self):
        self.rig = _rig(CLEAR)
        self.assertIn('via-nudge: moved VF2 via (11.175,10.500) -> '
                      '(11.325,10.500) to free C2', self.rig.out)
        self.assertTrue(self.rig.wrote)
        # re-parsed from disk; see the note in the headline class's setUp
        self.assertEqual(
            _via_by_net(parse_kicad_pcb(self.rig.src), 'VF2').tenting_attrs,
            {}, 'the INPUT via gained a spec; this is no longer the spec-less '
                'case')

    def test_a_via_that_inherited_still_inherits(self):
        v = _via_by_net(self.rig.out_pcb, 'VF2')
        self.assertEqual((round(v.x, 4), round(v.y, 4)), LAND_VF2)
        self.assertEqual(v.tenting_attrs, {})
        self.assertEqual(_prot_tokens(_via_block(self.rig.out_text, 'VF2')), [],
                         'the re-placed via was stamped with a protection '
                         'token the board never gave it')
    # MUTATION 12: drop `inherit_when_unspecified=True` at writer.py's emit.
    # MUTATION 13: make the inherit branch fall through to the default.

    def test_the_KiCad10_name_net_survives_so_this_arm_cannot_pass_wrongly(self):
        """`via_protection_sexpr` goes SILENT, not default, when net_name is
        None. Without this line the arm above would pass for entirely the wrong
        reason on a via that had lost its net."""
        self.assertIn('(net "VF2")', _via_block(self.rig.out_text, 'VF2'))
    # MUTATION 11: drop `net_name=nm` at writer.py's emit.

    def test_the_engine_dict_says_present_and_empty_not_absent(self):
        """Absent and empty are different facts for a caller that distinguishes
        'inherit' from 'nothing to say' -- which is the whole content of this
        fix."""
        by = {self.rig.pcb.nets[d['net_id']].name: d
              for _x, _y, d in self.rig.res['via_moves']}
        self.assertIn('tenting_attrs', by['VF2'])
        self.assertEqual(by['VF2']['tenting_attrs'], {})
    # MUTATION 4b: set the key only when the source spec is non-empty.


class TestTheHash489DefaultMovedAndThisFileFollowsIt(unittest.TestCase):
    """THE #489 DEFAULT MOVED, DELIBERATELY, AFTER THIS FILE WAS WRITTEN.

    These two arms were written when a caller with no opinion still got the
    #489 s8 default -- front+back tenting on KiCad-10 output -- and they
    asserted that the via-nudge's new "inherit" state had not leaked into it.
    #748/#749/#751 then RETIRED that default: an unspecified via now emits no
    protection token at all, in every case, so it inherits the board's own
    `(setup ...)` policy.

    That was not a tidy-up. Probed against pcbnew 10.0.0, a via left at
    `*_MODE_FROM_BOARD` serialises with NO token and a token appears only for
    an explicit override -- so stamping one turned an inheriting via into an
    override. Measured over 886 corpus boards, three (nanovoltmeter_marge,
    hexberry_fpga, pedal_404) declare `(tenting (front no) (back no))`
    board-wide and had every added via stamped tented: a fab error, hidden
    because KiCad's FACTORY policy is tented so the two agree on an ordinary
    board.

    So the arms are KEPT and INVERTED rather than deleted. What they can still
    detect changed, and saying so is the point: the empty-dict and no-attrs
    cases are now indistinguishable from "inherit" IN THE OUTPUT, by design,
    so these no longer discriminate the sentinel from the default. They are
    change detectors for the RETIREMENT still being in force -- a revert of it
    turns them red here as well as in the owning file.

    tests/test_489_via_tenting.py OWNS these values ("nothing to say -> say
    NOTHING, either way"); this file follows it. If the two ever disagree,
    test_489 is right."""

    def test_no_tenting_attrs_emits_NO_token_on_KiCad10(self):
        """Retired #489 s8 default: was front+back tenting, now nothing."""
        v10 = generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5,
                                 net_name='GND')
        self.assertNotIn('(tenting', v10)

    def test_an_empty_dict_also_emits_NO_token(self):
        """`{}` and the inherit keyword now agree, deliberately: an empty spec
        is "the board said nothing about this via", and the answer to that is
        to say nothing back."""
        self.assertNotIn('(tenting',
                         generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5,
                                            net_name='GND', tenting_attrs={}))

    def test_a_COPIED_spec_still_inherits_which_is_why_this_is_a_KEYWORD(self):
        """WHY THE DECISION IS A KEYWORD AND NOT A SENTINEL VALUE.

        The first version of this fix used a sentinel object passed as
        `tenting_attrs`. It survived copy, deepcopy and pickle -- and was
        destroyed by `dict()`, `.copy()` and `{**s}`, all of which turn a
        dict-shaped sentinel into a plain `{}` that lands straight back on the
        front+back default. That matters because `dict(...)` is EXACTLY the
        idiom this repo uses to carry a spec: fanout_clearance.py's move dict
        and plane_io.py:783 both spell it that way, and CLAUDE.md sends the
        next author to those very sites.

        A flag carried beside the value cannot be undone by copying the value.
        Found by a cold review of this branch."""
        for make, how in ((lambda d: dict(d), 'dict()'),
                          (lambda d: d.copy(), '.copy()'),
                          (lambda d: {**d}, '{**d}'),
                          (lambda d: d, 'verbatim')):
            self.assertEqual(
                _prot_tokens(generate_via_sexpr(
                    1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, net_name='GND',
                    tenting_attrs=make({}),
                    inherit_when_unspecified=True)), [],
                'a spec carried through %s stopped inheriting and fell back to '
                'the front+back default -- the #741 bug, restored by a copy'
                % how)
            self.assertEqual(
                _prot_tokens(generate_via_sexpr(
                    1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, net_name='GND',
                    tenting_attrs=make(dict(TYPE_VII)),
                    inherit_when_unspecified=True)),
                ['covering', 'plugging', 'capping', 'filling'],
                'a REAL spec carried through %s was lost' % how)
    # MUTATION 26: make the flag a sentinel VALUE again -> the dict() arm fires.

    def test_a_spec_the_TEXT_parser_never_touched_is_still_collapsed(self):
        """KILLS the whitespace-collapse survivor. `via_protection_sexpr._one`
        collapses its inner text onto one line -- and NO fixture in this file
        or in test_489 could see that, because `_balanced_token_text` already
        collapses on the way IN, so a spec that came from the text parser is
        one line before the writer ever sees it.

        The pcbnew path does not go through that parser, and neither does a
        hand-built spec. Build one with real newlines and tabs, which is what
        a caller assembling a spec by hand produces.

        Found by a mutation run: dropping the collapse left this file AND
        test_489_via_tenting.py green."""
        ragged = {'covering': '(front no)\n\t\t\t(back no)',
                  'capping': '\n\t\tyes\n\t'}
        out = generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5,
                                 net_name='GND', tenting_attrs=ragged)
        self.assertIn('(covering (front no) (back no))', out, out)
        self.assertIn('(capping yes)', out, out)
        for line in out.splitlines():
            self.assertLessEqual(
                line.count('('), 3,
                'a protection token spilled across lines: %r' % line)
    # MUTATION 15: drop `inner = " ".join((inner or '').split())` in _one.

    def test_a_token_KiCad_adds_LATER_is_re_emitted_rather_than_dropped(self):
        """KILLS the unrecognised-token survivor. `via_protection_sexpr` emits
        spec keys outside VIA_PROTECTION_TOKEN_ORDER after the known ones,
        deliberately -- losing a spec is the bug this whole area exists to
        prevent, and a KiCad that adds a sixth protection token must not have
        it silently dropped by a re-placement.

        No board in the corpus and no fixture in this file carries such a
        token, so the line was uncovered. Found by a mutation run."""
        spec = dict(TYPE_VII)
        spec['electroplating'] = 'yes'      # a token this repo does not know
        out = generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5,
                                 net_name='GND', tenting_attrs=spec)
        self.assertIn('(electroplating yes)', out, out)
        self.assertEqual(_prot_tokens(out),
                         ['covering', 'plugging', 'capping', 'filling'],
                         'the KNOWN tokens must still lead, in canonical '
                         'order, with the unknown one appended after')
    # MUTATION B: delete the `parts += [...]` unrecognised-token line.

    def test_the_flag_emits_nothing_and_a_real_spec_still_wins(self):
        self.assertEqual(
            _prot_tokens(generate_via_sexpr(
                1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, net_name='GND',
                inherit_when_unspecified=True)), [])
        self.assertEqual(
            _prot_tokens(generate_via_sexpr(
                1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, net_name='GND',
                tenting_attrs=dict(TYPE_VII))),
            ['covering', 'plugging', 'capping', 'filling'])
        # ...and the flag never OVERRIDES a real spec
        self.assertEqual(
            _prot_tokens(generate_via_sexpr(
                1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, net_name='GND',
                tenting_attrs=dict(TYPE_VII),
                inherit_when_unspecified=True)),
            ['covering', 'plugging', 'capping', 'filling'])

    def test_the_numeric_net_dialect_is_still_silent_either_way(self):
        """#489 pins that a KiCad-9 numeric-net via emits no protection at all.
        The sentinel must not have turned that into a default."""
        self.assertEqual(
            _prot_tokens(generate_via_sexpr(1, 2, 0.6, 0.3,
                                            ['F.Cu', 'B.Cu'], 5)), [])
        self.assertEqual(
            _prot_tokens(generate_via_sexpr(
                1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5,
                inherit_when_unspecified=True)), [])

    def test_a_real_spec_always_wins_over_the_flag(self):
        """Branch ORDER: the spec is tested first, so a caller that sets the
        flag defensively on every re-placement -- which is what CLAUDE.md now
        tells them to do -- never loses a real spec by doing so."""
        for flag in (True, False):
            self.assertEqual(
                _prot_tokens(generate_via_sexpr(
                    1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, net_name='GND',
                    tenting_attrs={'capping': 'yes'},
                    inherit_when_unspecified=flag)), ['capping'])
    # MUTATION 27: test inherit_when_unspecified BEFORE the spec.

    def test_the_imported_default_is_still_what_489_documented(self):
        """ANTI-ROT: the arms above follow the symbol, so they would keep
        passing if the value changed. This makes that change VISIBLE here
        instead of silent."""
        self.assertEqual(DEFAULT_VIA_TENTING,
                         {'tenting': '(front yes) (back yes)'},
                         'DEFAULT_VIA_TENTING changed. test_489_via_tenting.py '
                         'owns this value; re-read #489 s8 and re-measure this '
                         'file before touching it.')
        # Deliberately NOT re-pinning VIA_PROTECTION_TOKEN_ORDER as a literal
        # tuple. test_489 owns the EMITTED order; moving 'tenting' within that
        # constant is an in-scope #489 change that would have turned this file
        # red and left test_489 green -- a value this class says it does not
        # own, pinned in the one direction its owner cannot cover. Assert only
        # the property this file depends on: that the four Type-VII tokens
        # keep a stable relative order.
        self.assertEqual(
            [t for t in VIA_PROTECTION_TOKEN_ORDER if t in TYPE_VII],
            ['covering', 'plugging', 'capping', 'filling'])


class TestAViaTheNudgeNeverTouchedIsUnchanged(unittest.TestCase):
    """VF3 sits 18mm away and is nobody's offender."""

    def setUp(self):
        self.rig = _rig(CLEAR)
        # ON THE BRANCH. Without this, an engine that never reached the nudger
        # satisfies every assertion below -- measured: with
        # nudge_vias_for_unresolved neutered to `return [], []` this class
        # alone passed in 0.29s. Found by an adversarial review of this file.
        self.assertIn('via-nudge: moved VF1 via', self.rig.out)
        self.assertEqual(len(self.rig.res['via_moves']), 2, self.rig.out)
        self.assertTrue(self.rig.wrote)

    def test_the_control_vias_block_is_BYTE_identical(self):
        rig = self.rig
        self.assertEqual(_via_block(rig.out_text, 'VF3'),
                         _via_block(rig.in_text, 'VF3'),
                         'a via nobody moved was rewritten -- including its '
                         'uuid, which no consumer can follow across that')
        v = _via_by_net(rig.out_pcb, 'VF3')
        self.assertEqual(v.tenting_attrs, TYPE_VII)
        self.assertEqual((round(v.x, 4), round(v.y, 4)), CTRL_XY)
    # MUTATION: a writer that re-emits every via instead of the moved ones.
    #
    # NOT this arm: net-agnostic removal (dropping net_ids=/net_names= from
    # writer.py). MEASURED to leave this file entirely green, because nothing
    # on the rig sits within _remove_vias_at_positions' 2e-3 tolerance of a
    # moved via's old position -- and a coincident via cannot be ADDED to the
    # rig, because at 0.15mm it fails the hole-to-hole gate and the nudge then
    # refuses (measured: VF1 stops moving). That mutation is owned by
    # tests/test_via_move_removal_net_name.py, which drives
    # _remove_vias_at_positions directly with a positive and a negative net
    # match -- the #344 stacked-via regression.


class TestTheMoveDictCarriesTheSpecForTheGUI(unittest.TestCase):
    """kicad_routing_plugin/fanout_gui.py already reads `vd.get('tenting_attrs')`
    off this dict -- written for #489 s8 and DEAD since, because the engine
    never put the key there.

    Asserted at the ENGINE and not through the GUI on purpose. In the GUI,
    `pcb_data.vias[i].tenting_attrs` is itself built by
    `_pcbnew_via_protection_attrs(track)`, which is the SAME call the GUI's own
    fallback makes -- so the fallback re-derives an identical value and would
    MASK an engine revert. The GUI cannot be its own regression detector here."""

    def setUp(self):
        self.rig = _rig(CLEAR)
        self.assertIn('via-nudge: moved', self.rig.out)
        self.assertEqual(len(self.rig.res['via_moves']), 2, self.rig.out)
        self.by = {self.rig.pcb.nets[d['net_id']].name: d
                   for _x, _y, d in self.rig.res['via_moves']}
        self.assertEqual(sorted(self.by), ['VF1', 'VF2'])

    def test_the_dict_carries_the_key_with_the_SOURCE_vias_spec(self):
        self.assertIn('tenting_attrs', self.by['VF1'],
                      "the GUI's read is still dead: it re-tents every via it "
                      "nudges, and the CLI half of the fix hides that")
        self.assertEqual(self.by['VF1']['tenting_attrs'], TYPE_VII)
    # MUTATION 1: delete the dict key.

    def test_the_dict_holds_a_COPY_not_the_live_Vias_attribute(self):
        """The ONLY killer for `dict(getattr(...))` -> `getattr(...)`, and it
        exists for that alone. Nothing in the shipped path mutates the spec, so
        the mutation is invisible to every other arm -- while the via itself IS
        mutated a few lines earlier (`v.x, v.y = nx, ny`), which is exactly the
        shape of code where a shared reference eventually bites."""
        src = _via_by_net(self.rig.pcb, 'VF1')
        d = self.by['VF1']
        # presence FIRST: without it, mutation 1 (drop the key) reports a bare
        # KeyError traceback here instead of this arm's authored reason.
        self.assertIn('tenting_attrs', d)
        self.assertEqual(d['tenting_attrs'], src.tenting_attrs)
        self.assertIsNot(d['tenting_attrs'], src.tenting_attrs)
    # MUTATION 5: `dict(getattr(v,'tenting_attrs',{}) or {})` ->
    #             `getattr(v,'tenting_attrs',{})`.

    def test_the_key_set_is_exactly_what_the_contract_docstring_says(self):
        """Two consumers index this dict (placement/writer.py and the plugin's
        pcbnew mirror) and #741 IS a key that went unnamed, so the shape is
        pinned here and spelled in the docstring."""
        self.assertEqual(set(self.by['VF1']),
                         {'x', 'y', 'size', 'drill', 'layers', 'net_id',
                          'tenting_attrs'})
        doc = inspect.getdoc(FC.nudge_vias_for_unresolved) or ''
        for k in ('tenting_attrs', 'net_id', 'layers'):
            self.assertIn(k, doc,
                          'the contract docstring no longer names %r' % k)
    # MUTATION: rename the key; add a second, differently-spelled one.

    def test_a_synth_Via_round_trips_through_the_writer_and_parser(self):
        """The previous version of this arm asserted
        `Via(tenting_attrs=X).tenting_attrs == X` -- a value compared to itself
        through a constructor, which no change to any of the three files under
        test could break. Replaced with the round trip it was pretending to
        be: synth Via -> generate_via_sexpr -> _extract_via_protection_attrs.
        Found by an adversarial review of this file."""
        probe = make_via(0.0, 0.0, net_id=1, size=V_SIZE, drill=V_DRILL,
                         tenting_attrs=dict(TYPE_VII))
        probe.uuid = 'deadbeef-0000-0000-0000-00000000ffff'
        text = generate_via_sexpr(probe.x, probe.y, probe.size, probe.drill,
                                  probe.layers, probe.net_id, net_name='NB1',
                                  tenting_attrs=probe.tenting_attrs)
        back = KP._extract_via_protection_attrs(text)
        self.assertEqual(len(back), 1, text)
        self.assertEqual(list(back.values())[0], TYPE_VII, text)
    # MUTATION A: `_one` emits `({token})` and drops the inner text.
    # MUTATION A2: `parts = parts[:1]` -- emit only the first token.


class TestANoNetViaIsNotTurnedIntoInvisibleCopper(unittest.TestCase):
    """THE REGRESSION THIS FIX ITSELF INTRODUCED, caught by an independent
    reachability review rather than by me.

    `via_protection_sexpr` picks the net dialect from `net_name` and the
    protection tokens from `tenting_attrs` INDEPENDENTLY. So a via emitted with
    a numeric `(net N)` token AND a spec is a shape `kicad_parser.extract_vias`
    cannot match -- its KiCad-9 pattern has strict field ordering and no gap
    for the protection tokens, while its KiCad-10 twin was given one. Such a
    via does not merely lose its spec: it VANISHES from the model. Invisible
    copper the router then plans tracks through, which is the hazard PR #534
    was written against, and strictly worse than the wrong-tenting bug #741
    set out to fix.

    Before this fix the placement writer passed no `tenting_attrs`, so it could
    never produce that pairing. After it, one input does: a via whose net id
    does not resolve to a name. That is not hypothetical -- `extract_nets`
    records `name_to_id[""] = 0` but never creates a `Net` for id 0, so a
    legitimate no-net `(net "")` via misses the lookup on EVERY board.
    Measured: kicad_files/orangecrab_ext_pll.kicad_pcb resolves 169 nets and
    has no key 0.

    MEASURED on the call below, one via move with net_id 0 and a Type-VII
    spec, on a pcb_data whose map has had key 0 removed to match a real board:

        0cdafd7a  (before #741)      (net 0)   no tokens    3 in file, 3 parsed
        the #741 fix, no guard       (net 0)   4 tokens     3 in file, 2 PARSED
        with the guard               (net "")  4 tokens     3 in file, 3 parsed

    The middle row is the regression. The third keeps the spec AND the via.

    Driven through `write_placed_output` directly rather than the rig, because
    the rig declares a numeric net table and therefore HAS a key 0 -- it cannot
    reach this at all, which is exactly why the defect survived the first
    review round."""

    def _emit(self, net_id, spec):
        d, src = _stage('net0-%s' % net_id)
        pcb = parse_kicad_pcb(src)
        # Match a real KiCad-10 board: nets resolved by name, no entry for 0.
        pcb.net_id_to_name.pop(0, None)
        self.assertNotIn(0, pcb.net_id_to_name,
                         'the map still has a key 0, so this arm is not '
                         'reproducing a real board and proves nothing')
        move = (VIA_X, 9.5, {'x': LAND_VF1[0], 'y': LAND_VF1[1],
                             'size': V_SIZE, 'drill': V_DRILL,
                             'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id,
                             'tenting_attrs': dict(spec)})
        dst = os.path.join(d, 'out.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            wrote = write_placed_output(src, dst, [], via_moves=[move],
                                        new_segments=[], pcb_data=pcb)
        self.assertTrue(wrote)
        text = _read(dst)
        moved = [b for b, _n in _via_blocks(text)
                 if ('%.6f' % LAND_VF1[0]) in b]
        self.assertEqual(len(moved), 1, text)
        return text, moved[0], parse_kicad_pcb(dst)

    def test_a_no_net_via_with_a_spec_survives_the_round_trip(self):
        text, blk, out = self._emit(0, TYPE_VII)
        n_in_file = len(_via_blocks(text))
        self.assertEqual(
            len(out.vias), n_in_file,
            'the written board has %d vias and the parser reads back %d. A '
            'via was emitted in a dialect our own parser cannot match, so it '
            'is invisible copper -- worse than the spec loss #741 fixes.'
            % (n_in_file, len(out.vias)))
    # MUTATION 23: neutralise BOTH guards (the net-0 naming AND the belt)
    # -> (net 0) plus four protection tokens, and the via is gone: 3 in file,
    # 2 parsed. Either guard alone still saves it, which is why 24 and 25 are
    # separate rows.

    def test_it_is_emitted_in_the_NAME_dialect_and_keeps_its_spec(self):
        """The guard must not buy the via back by throwing the spec away --
        that would re-open #741 for exactly this via."""
        text, blk, out = self._emit(0, TYPE_VII)
        self.assertIn('(net "")', blk, blk)
        self.assertNotIn('(net 0)', blk, blk)
        self.assertEqual(_prot_tokens(blk),
                         ['covering', 'plugging', 'capping', 'filling'], blk)
        v = [x for x in out.vias if x.net_id == 0]
        self.assertEqual(len(v), 1)
        self.assertEqual(v[0].tenting_attrs, TYPE_VII)
    # MUTATION 24: make the guard suppress the spec instead of naming the net
    # (nm stays None, spec -> INHERIT) -> the via survives but ships bare.

    def test_the_belt_never_emits_a_numeric_net_ALONGSIDE_a_spec(self):
        """The residue guard, for an id that is not 0 and does not resolve.
        No board produces this today -- per-via protection is a KiCad-10
        feature and those boards are name-net -- so it is asserted at the
        writer rather than through a fixture. Losing the spec here is the
        deliberate trade: losing the VIA is worse."""
        text, blk, out = self._emit(9999, TYPE_VII)
        numeric = '(net 9999)' in blk
        if numeric:
            self.assertEqual(_prot_tokens(blk), [],
                             'a numeric net token was emitted ALONGSIDE a '
                             'protection spec -- the exact pairing extract_'
                             'vias cannot read back')
        self.assertEqual(len(out.vias), len(_via_blocks(text)),
                         'a via was lost at re-parse')
    # MUTATION 25: drop the `if nm is None and spec is not INHERIT...` belt.


class TestARefusedNudgeLeavesTheBoardTextAlone(unittest.TestCase):
    """The same board at clearance 0.7, where no clear spot exists inside the
    0.6mm search. This licenses the framing: a via's spec is only ever at risk
    when the via is actually re-emitted."""

    def test_a_boxed_via_is_REFUSED_and_every_spec_is_untouched(self):
        rig = _rig(CLEAR_BOXED)
        # the PRINTED refusal FIRST -- a refusal that prints nothing is
        # indistinguishable from a pass that never looked.
        self.assertIn("via-nudge: no clear spot for C1's offending via "
                      "at (11.18, 9.50) within 0.6mm", rig.out)
        self.assertEqual(rig.res['via_moves'], [])
        self.assertEqual(rig.res['new_segments'], [])
        for net, spec in (('VF1', TYPE_VII), ('VF2', {}), ('VF3', TYPE_VII)):
            self.assertEqual(_via_by_net(rig.pcb, net).tenting_attrs, spec)
        self.assertTrue(rig.wrote,
                        'the writer refused, so the text comparison below '
                        'would silently vanish while the printed-refusal half '
                        'still looked satisfied')
        self.assertEqual(_strip_via_blocks(rig.out_text),
                         _strip_via_blocks(rig.in_text))
    # MUTATION 19: weaken the draw gate so a boxed via moves anyway.

    def test_the_NEGATIVE_control_at_the_shipped_clearance_still_moves(self):
        rig = _rig(CLEAR)
        self.assertIn('via-nudge: moved', rig.out)
        self.assertEqual(len(rig.res['via_moves']), 2)
    # MUTATION: none (control).


class TestTheShippedCLIPathKeepsTheSpec(unittest.TestCase):
    """The one subprocess arm. It earns its cost on a mutation nothing
    in-process can kill: drop `pcb_data=pcb_data` at
    place_fanout_clearance.py's write_placed_output call. Then `n2n` is None,
    `nm` is None, the via emits a NUMERIC `(net 7)` token into a board whose
    other copper is name-style, AND via_protection_sexpr goes silent entirely.
    Every in-process arm passes pcb_data itself and cannot see it.

    MEASURED, because my first statement of the mechanism was backwards: the
    spec is NOT silenced. `via_protection_sexpr`'s `net_name` gate guards only
    the DEFAULT, so a real spec is still emitted -- next to a numeric net
    token, which is the shape `extract_vias`' KiCad-9 pattern cannot match, so
    the via VANISHES from the re-parse. The arm below therefore asserts the via
    is still THERE before asserting anything about its spec, or the failure
    arrives as a bare helper assertion instead of this arm's own reason.

    Note the CLI's own guard is `if result['placements'] or
    result.get('via_moves') or ...`, and on this rig `placements == []` -- so
    via_moves alone is what triggers the write, which is exactly the path under
    test."""

    def test_place_fanout_clearance_py_writes_both_answers_through(self):
        d, src = _stage('cli')
        dst = os.path.join(d, 'out.kicad_pcb')
        r = run_utils.check(
            [sys.executable, run_utils.tool('place_fanout_clearance.py'),
             src, dst, '--clearance', str(CLEAR),
             '--max-displacement', '0', '--max-displacement-cap', '0',
             '--max-passes', '1', '--no-rotate'], accept=True)
        self.assertIn('via-nudge: moved VF1 via', (r.stdout or '') + (r.stderr or ''))
        run_utils.evidence(dst, 'the CLI output board')
        out = parse_kicad_pcb(dst)
        self.assertEqual(
            sorted(out.nets[v.net_id].name for v in out.vias),
            ['VF1', 'VF2', 'VF3'],
            'a via is MISSING from the re-parse. The likeliest cause is a via '
            'emitted with a numeric net token AND a protection spec, which '
            "extract_vias' KiCad-9 pattern cannot match -- so the barrel is "
            'gone from the model, not merely its spec.')
        self.assertEqual(_via_by_net(out, 'VF1').tenting_attrs, TYPE_VII)
        self.assertEqual(_via_by_net(out, 'VF2').tenting_attrs, {})
        self.assertEqual(_via_by_net(out, 'VF3').tenting_attrs, TYPE_VII)
        with io.open(dst, encoding='utf-8') as f:
            text = f.read()
        self.assertIn('(net "VF1")', _via_block(text, 'VF1'))
        self.assertEqual(_prot_tokens(_via_block(text, 'VF2')), [])
    # MUTATION 18: drop pcb_data= at place_fanout_clearance.py's writer call.


class TestOneEmitSiteOneSpecArgument(unittest.TestCase):
    """Source-structure guard: the mutations no fixture can reach.

    Reports LINE NUMBERS, never source text -- #732 measured a 393KB failure
    message from an assertIn over a whole module."""

    def setUp(self):
        from placement import writer as W
        self.W = W
        self.writer = _code(W)
        self.nudger = _code(FC.nudge_vias_for_unresolved)

    def test_py_placer_emits_a_via_at_ONE_site_that_passes_spec_and_sentinel(self):
        calls = _calls(self.writer, 'generate_via_sexpr')
        self.assertEqual(
            len(calls), 1,
            'generate_via_sexpr is called at %d sites in placement/writer.py '
            '(line(s) %s). #741 exists because the single site forgot one '
            'argument; a second site is that defect with somewhere to hide.'
            % (len(calls), [ln for ln, _ in calls]))
        bad = [ln for ln, args in calls if 'tenting_attrs' not in args]
        self.assertEqual(
            bad, [],
            'generate_via_sexpr at line(s) %s emits a via with no '
            'tenting_attrs= argument, so via_protection_sexpr falls through to '
            'DEFAULT_VIA_TENTING and every nudged via ships tented (#489 s8, '
            '#741).' % bad)
        noflag = [ln for ln, args in calls
                  if 'inherit_when_unspecified' not in args]
        self.assertEqual(
            noflag, [],
            'the emit at line(s) %s passes a spec but not '
            'inherit_when_unspecified, so a via that carried NO spec is '
            'stamped with front+back tenting it never had -- the second half '
            'of #741.' % noflag)
        # and the net-0 / numeric-dialect guard, which is what stops the same
        # call turning a via into copper the parser cannot see.
        self.assertEqual(
            len([1 for l in self.writer if 'def _net_name(' in l]), 1,
            'placement/writer.py no longer resolves the net name through one '
            'helper; the emit and the REMOVAL can now disagree about net 0')
    # MUTATIONS 2, 12, 16.

    def test_no_OTHER_file_under_py_placer_emits_a_via(self):
        """A directory sweep, not inspect: the point is that a NEW module
        cannot grow a second emitter without landing here."""
        hits = sorted(
            os.path.relpath(p, _ROOT).replace('\\', '/')
            for p in glob.glob(os.path.join(_ROOT, 'py_placer', '**', '*.py'),
                               recursive=True)
            if 'generate_via_sexpr' in _read(p))
        self.assertEqual(hits, ['py_placer/placement/writer.py'],
                         'py_placer gained a via emitter at %s' % hits)
    # MUTATION 16: add an unguarded via emitter anywhere else under py_placer/.

    def test_the_move_dict_is_BUILT_in_exactly_one_place(self):
        appends = [i + 1 for i, l in enumerate(self.nudger)
                   if 'via_moves.append(' in l]
        self.assertEqual(len(appends), 1,
                         'the via-move dict is constructed at %d sites '
                         '(function-relative line(s) %s)'
                         % (len(appends), appends))
        # The KEY spelling, with its colon: `'tenting_attrs'` alone also
        # matches the contract docstring (which names it deliberately) and the
        # `getattr(v, 'tenting_attrs', ...)` continuation line, so counting
        # that would be counting three different things.
        keys = [i + 1 for i, l in enumerate(self.nudger)
                if "'tenting_attrs':" in l]
        self.assertEqual(len(keys), 1,
                         "the move dict sets 'tenting_attrs' at %d CODE "
                         "line(s) of the nudger (function-relative %s); "
                         "expected exactly one" % (len(keys), keys))
    # MUTATION: build the dict in two places; drop the key.

    def test_the_pattern_this_fix_COPIES_still_exists(self):
        """#741's fix is not novel: py_router/plane_io.py already carries the
        identical pair, for the identical reason (a via that came OFF the board
        and goes back on). If plane_io drops it, this file's docstring claim
        that it is a house pattern has rotted, and so has half of #489."""
        import plane_io
        pl = _code(plane_io)
        self.assertEqual(
            len([1 for l in pl if "'tenting_attrs': dict(getattr(" in l]), 1,
            'plane_io no longer carries the move-dict half of the pattern')
        self.assertGreaterEqual(
            len([ln for ln, a in _calls(pl, 'generate_via_sexpr')
                 if 'tenting_attrs' in a]), 2,
            'plane_io no longer carries the emit half of the pattern')
    # MUTATION 22: strip the pair out of plane_io.

    def test_the_count_arms_are_not_searching_for_dead_strings(self):
        """ANTI-ROT. Every count arm above passes after a rename. Assert the
        positive controls so a rename fails HERE instead of disarming them."""
        for token, lines, floor in (
                ('generate_via_sexpr', self.writer, 2),
                ('tenting_attrs', self.writer, 1),
                ('inherit_when_unspecified', self.writer, 1),
                ('_net_name', self.writer, 3),
                ('_remove_vias_at_positions', self.writer, 2),
                ('via_moves', self.nudger, 3),
                ('tenting_attrs', self.nudger, 1)):
            n = sum(1 for l in lines if token in l)
            self.assertGreaterEqual(
                n, floor,
                '%r appears %d times in CODE (expected >= %d): it was renamed, '
                'and the count arms of this class are now vacuous'
                % (token, n, floor))

    def test_a_trailing_comment_cannot_arm_or_disarm_the_guard(self):
        """FALSE-POSITIVE PROBE. placement/writer.py's comment block names
        `generate_via_sexpr` and `_remove_vias_at_positions` in prose; the
        stripper must drop them (#737's lesson)."""
        def _probe():
            x = 1  # generate_via_sexpr(a, b) and via_moves.append(
            # 'tenting_attrs': dict(getattr(v, 'tenting_attrs', {}))
            return x

        # Through the REAL helper. Re-implementing the stripper inline left a
        # change to _code invisible to its own probe.
        stripped = _code(_probe)
        self.assertEqual(_calls(stripped, 'generate_via_sexpr'), [])
        self.assertEqual([l for l in stripped if "'tenting_attrs'" in l], [])

    def test_no_sentinel_VALUE_came_back(self):
        """The decision must stay a keyword. A sentinel passed as
        `tenting_attrs` is destroyed by `dict()`, which is the repo's own idiom
        for carrying a spec -- see the dict()/copy()/{**} arm above."""
        import kicad_writer as KW
        src = '\n'.join(_code(KW))
        self.assertNotIn('INHERIT_VIA_PROTECTION', src,
                         'a sentinel VALUE is back in kicad_writer; it cannot '
                         'survive dict(), which is how this repo copies a spec')
        self.assertIn('inherit_when_unspecified', src)
    # MUTATION 26: reintroduce the sentinel.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """Three arms, and the DISCLOSURE they carry matters more than the passes.

    "INERT" here means the fix changes no GEOMETRY, and on the tracked corpus
    it changes no BYTES either. Not, note, because the corpus never reaches the
    nudger -- an earlier draft of this class asserted that and the run
    falsified it. MEASURED over the 22 boards `git ls-files
    kicad_files/*.kicad_pcb` returns, at clearance 0.25 and 0.10 with
    displacement pinned to 0 and the cap fallback OFF (the configuration that
    maximally FORCES the nudger):

        DISTINCT boards that ran the whole pass ...........  2
        board-clearance PAIRS that ran it .................  4
        pairs that moved a via ............................  1
          orangecrab_ext_pll.kicad_pcb @ 0.10 .............  9 vias
        distinct source specs among those 9 ...............  exactly
                                            {'tenting': '(front yes) (back yes)'}

    That last line is the whole inertness argument, and it is what the second
    arm asserts: every via the corpus moves already carries byte-for-byte
    `kicad_writer.DEFAULT_VIA_TENTING`, so the string this fix emits and the
    string the defect emitted are THE SAME STRING. Neither half of #741 can
    fire on it -- the spec is not lost because it is re-emitted verbatim, and
    it is not gained because it was already there.

    Which is also exactly why the corpus can never be the DEMONSTRATION, and
    why the synthetic rig above is necessary rather than lazy. Both arms are
    self-expiring: they fail the day a board with a real Type-VII via is
    tracked, which is the day this file should be re-measured against it."""

    @staticmethod
    def _boards():
        return run_utils.corpus_boards()

    def _skip_without_git(self, boards):
        if not boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('git ls-files returned nothing')

    def test_every_via_the_corpus_MOVES_re_emits_to_the_same_string(self):
        """THE inertness arm. Run the pass over every tracked board in the
        configuration that most forces a nudge, and assert that each via it
        relocates carries a spec the writer would have produced anyway."""
        boards = self._boards()
        self._skip_without_git(boards)
        full, moved, specs, blew = set(), 0, set(), []
        for b in boards:
            for clr in (0.25, 0.10):
                try:
                    pcb = parse_kicad_pcb(b)
                    with contextlib.redirect_stdout(io.StringIO()):
                        res = repair_fanout_clearance(
                            pcb, b, clearance=clr, max_displacement=0.0,
                            max_displacement_cap=0.0, max_passes=1,
                            allow_rotations=False, via_clear_fallback=False)
                except Exception as e:
                    # COLLECTED, not swallowed. `continue` alone meant an
                    # engine regression that started raising on 20 of the 22
                    # boards left both corpus arms green. Measured today: zero.
                    blew.append('%s @ %s: %s: %s'
                                % (os.path.basename(b), clr,
                                   type(e).__name__, e))
                    continue
                # The two EARLY-RETURN dicts (no BGA / no movable cap) carry no
                # 'via_moves' key at all, so only a board that ran the whole
                # pass can answer this. Do NOT spell it
                # `res.get('via_moves') or []`: that reads "never reached the
                # nudger" as "reached it and moved nothing", and the floor
                # below would then be counting the wrong set.
                if 'via_moves' not in res:
                    continue
                full.add(b)
                for _x, _y, d in res['via_moves']:
                    moved += 1
                    self.assertIn('tenting_attrs', d,
                                  '%s @ %s: the move dict lost the key on a '
                                  'REAL board' % (b, clr))
                    specs.add(tuple(sorted((d['tenting_attrs'] or {}).items())))
        self.assertEqual(blew, [], 'the pass RAISED on tracked board(s); '
                                   'that is a finding, not something to skip')
        self.assertGreaterEqual(len(full), 2,
                                'only %d tracked BOARD(s) ran the full pass '
                                '(2 measured, over 4 board-clearance pairs); '
                                'this arm is no longer measuring anything'
                                % len(full))
        # A FLOOR OF ONE, not of nine. Nine is what the corpus moves today
        # (orangecrab_ext_pll @ 0.10) and it is disclosed rather than asserted:
        # pinning it would turn a *tenting* test red for a *geometry* reason
        # the day a legitimate nudge change relocates eight vias instead.
        print('corpus disclosure: %d via(s) moved across %d board(s); '
              'today that is 9 on orangecrab_ext_pll @ 0.10'
              % (moved, len(full)))
        self.assertGreaterEqual(moved, 1,
                                'the corpus moved NO vias, so the spec '
                                'assertion below grades an empty set')
        self.assertEqual(
            specs, {tuple(sorted(DEFAULT_VIA_TENTING.items()))},
            'a tracked board now MOVES a via whose spec is not the writer '
            'default: %s. The corpus is no longer inert under this fix -- '
            'which also means it can now demonstrate #741. Re-measure, and '
            'consider replacing this file\'s synthetic rig with that board.'
            % sorted(specs))
    # MUTATION 1 on a REAL board: the assertIn fires.
    # Otherwise a self-expiring corpus bound.

    def test_every_tracked_per_via_spec_is_one_the_default_would_produce(self):
        """A TEXT scan, not a repair run. Any tracked via whose spec is not
        DEFAULT_VIA_TENTING is a via the shipped defect would have CORRUPTED,
        and the day one appears this file's rig should be replaced by it."""
        boards = self._boards()
        self._skip_without_git(boards)
        specs, carriers = set(), {}
        for b in boards:
            with io.open(b, encoding='utf-8', errors='replace') as f:
                text = f.read()
            attrs = KP._extract_via_protection_attrs(text)
            if attrs:
                carriers[os.path.basename(b)] = len(attrs)
            for spec in attrs.values():
                specs.add(tuple(sorted(spec.items())))
        want = tuple(sorted(DEFAULT_VIA_TENTING.items()))
        self.assertTrue(specs, 'no tracked board carries a per-via spec at '
                               'all; the parse path may have regressed')
        self.assertEqual(
            specs, {want},
            'a tracked board now carries a per-via spec the writer default '
            'would NOT produce: %s. This file says the corpus cannot '
            'demonstrate #741 -- that is no longer true. Use the real board.'
            % sorted(specs))
        self.assertGreaterEqual(len(carriers), 2, carriers)
    # MUTATION: none -- self-expiring. If it fires, re-measure; do not relax.

    def test_the_two_spec_carrying_boards_are_still_found(self):
        """Guards the arm above against a parse regression that would empty the
        set and make it pass vacuously."""
        boards = self._boards()
        self._skip_without_git(boards)
        counts = {}
        for b in boards:
            with io.open(b, encoding='utf-8', errors='replace') as f:
                text = f.read()
            n = len(KP._extract_via_protection_attrs(text))
            if n:
                counts[os.path.basename(b)] = n
        self.assertIn('orangecrab_ext_pll.kicad_pcb', counts, counts)
        self.assertIn('rp2350_fpga_eensy_prePlane.kicad_pcb', counts, counts)


# THE BATTERY, as RUN -- every row below was applied to the tree and the file
# re-run, not reasoned about. 32 mutations, 29 killed, 3 survivors, 0 BROKEN
# (no mutation made this file die before it checked anything). The killer per
# row is also named in the trailing `# MUTATION n:` comment beside the arm.
#
#    1  drop the move-dict key ......... headline, collapse, move-dict x4, the
#                                         source arm, the CLI arm, the corpus arm
#    2  drop tenting_attrs= at the emit  headline, collapse, spec-less, source,
#                                         anti-rot, CLI
#    3  pass None at the emit .......... the same six as 2 -- the source arm DOES
#                                         fire, because it requires the SENTINEL
#                                         in the argument text, not just the kwarg
#    4  pass {} at the emit ............ the same six as 2. NOT the move-dict
#                                         arms: a writer-side mutation cannot
#                                         reach the engine dict
#   4b  key only when spec non-empty ... the present-and-empty arm, ALONE
#    5  share the dict, don't copy ..... TestTheDictHoldsACOPY, ALONE
#    6  drop the `or {}` .............. SURVIVES -- declared, see below
#    7  copy the spec to the wrong via . headline + spec-less together
#    8  emit no uuid .................. headline + the uuid arm
#   10  skip _remove_vias_at_positions . the uuid/count arm, the whole-file diff,
#                                         and five more
#   11  drop net_name=nm ............... the (net "VF2") arm + 5
#   12  drop inherit_when_unspecified=True  spec-less, source, anti-rot, CLI
#  12b  pass the sentinel UNCONDITIONALLY  headline, collapse, CLI
#  13b  inherit branch -> the default ... spec-less arm, both flag arms, CLI
#   26  make the decision a sentinel VALUE  the dict()/copy()/{**} arm and the
#                                         no-sentinel source arm. This is the
#                                         design the first draft shipped; a
#                                         cold review measured that dict() --
#                                         the repo's OWN idiom for copying a
#                                         spec -- destroys it
#   27  test the flag BEFORE the spec ... the real-spec-wins arm
#   14  reorder VIA_PROTECTION_TOKEN_ORDER  the canonical-order arm + 2
#   15  drop the whitespace collapse ... the ragged-spec arm (ADDED after a
#                                         mutation run showed this file AND
#                                         test_489 both green without it)
#   16  a second emit site in py_placer/  the source arm, ALONE
#   17  rename any guarded identifier .. the anti-rot arm, ALONE
#   18  drop pcb_data= in the CLI main .. the CLI subprocess arm, ALONE
#   19  weaken the draw gate ........... the refusal arm, ALONE
#   20  spiral step 0.05 -> 0.1 ........ SURVIVES -- declared, see below
#    A  _one drops the inner text ...... headline, collapse, both #489 default
#                                         arms, CLI
#   A2  emit only the FIRST token ...... headline, collapse, CLI (the VF1-only
#                                         mutation: VF2 emits nothing either
#                                         way and every #489 arm has one token)
#    B  drop the unrecognised-token line  the later-token arm (ADDED; it
#                                         survived before)
#    C  change the emitted indentation .. SURVIVES -- declared, see below
#   22  strip the pattern out of plane_io  the house-pattern arm
#   23  drop BOTH net-0 guards .......... all three no-net arms -- the via is
#                                          LOST (3 in file, 2 parsed). This is
#                                          the REGRESSION the #741 fix itself
#                                          introduced. Removing only ONE guard
#                                          does not lose the via, because the
#                                          other still catches it; that is the
#                                          point of having two, and it is why
#                                          rows 24 and 25 exist separately.
#   24  net-0 guard does nothing ........ the name-dialect arm (the belt then
#                                          saves the via but it ships BARE)
#   25  drop the numeric+spec belt ...... the belt arm
#
# NOT IN THIS FILE, and named so it does not read as uncovered: net-agnostic
# removal (dropping net_ids=/net_names= at the writer). It leaves this file
# green -- nothing on the rig is within _remove_vias_at_positions' 2e-3
# tolerance of a moved via's old position, and a coincident via CANNOT be added
# to the rig because at 0.15mm it fails the hole-to-hole gate and the nudge
# then refuses (measured). It is owned by tests/test_via_move_removal_net_name
# .py, the #344 stacked-via regression, which drives that function directly
# with a positive and a negative net match.
#
# THE THREE SURVIVORS, recorded rather than papered over. Two FURTHER
# survivors -- #15 (the whitespace collapse) and #B (the unrecognised-token
# emit) -- were found by an adversarial review of this file and a mutation run,
# and are NOT in this list: arms were added to kill them, and both kills are
# verified. Rows 15 and B name those arms.
#
#   #6, drop the `or {}` in dict(getattr(v, 'tenting_attrs', {}) or {}).
#   Via.tenting_attrs is field(default_factory=dict) and
#   _pcbnew_via_protection_attrs returns {} or a dict, so NO parse path in this
#   repo produces None and the two spellings are identical everywhere
#   reachable. A killer could be manufactured with a via-like carrying None,
#   but that asserts about a state no caller produces -- the phantom fixture
#   #731 removed from a different report. The spelling stays because
#   plane_io.py spells it that way, and one spelling beats one branch.
#
#   #20, the spiral quantum (r += 0.05 -> 0.1). The rig's clear spot is at
#   radius exactly 0.15, which both step sizes reach, and the 0.7 refusal is
#   unchanged because 0.65 still exceeds max_shift 0.6. So the landings this
#   file asserts are genuinely identical. On a REAL board a via whose clear
#   spot is at 0.10 would be pushed to 0.15 -- a geometry change this rig
#   cannot see. The claim in row 20 is therefore narrowed: the landings catch a
#   change that MOVES the landing, not every geometry change.
#
#   #C, the indentation of the emitted fragment. Semantically the same s-expr;
#   uncovered in the "records what is pinned" sense only.
#
#   Net-agnostic removal is NOT in this list either: see the note above -- it
#   is owned by tests/test_via_move_removal_net_name.py rather than uncovered.
#
# RECORDED AS UNCOVERED: the GUI mirror (fanout_gui.py's nudge block ->
# gui_utils.apply_via_protection) needs pcbnew, and tests/gui_parity/** is not
# collected by run_all.py's flat glob. TestTheMoveDictCarriesTheSpecForTheGUI
# pins the CONTRACT that block reads; the pcbnew half is not exercised here.


if __name__ == '__main__':
    unittest.main(verbosity=2)
