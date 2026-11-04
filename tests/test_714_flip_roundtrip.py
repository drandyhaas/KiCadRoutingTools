#!/usr/bin/env python3
"""#714 RT: flip, flip back, and the bytes must come back exactly.

The issue proposes this as an acceptance criterion, and it is the only gate in
the set that can see a transform which is *self-consistent but not an
involution* -- a coordinate negated through `float` instead of by toggling the
sign character, which rewrites `0.500` as `0.5`, or a `(justify mirror)` token
that is added on the way out and never removed on the way back.

WHAT "BYTE-IDENTICAL" IS MEASURED AGAINST, and it is not the input board.
`write_placed_output` runs `move_copper_text_to_silkscreen` over the whole file
and reformats `(at)` to six decimals for every ref it is handed, so a raw
comparison fails for reasons that have nothing to do with the flip. The baseline
is the DOSE-0 PASS OF THE SAME WRITER -- exactly the control board
`perturb.perturb` builds before it perturbs anything, and for exactly this
reason.

WHY IT IS NOT VACUOUS, in four arms, all required:

  1. `sha256(flipped) != sha256(control)`, ASSERTED FIRST. Without this a writer
     that ignores the side key entirely round-trips perfectly. It is the single
     most important line in the file, and it is the arm that is red today.
  2. The delta between control and flipped is CONFINED to the target block's
     span. Nothing else in the file may move.
  3. A FLOOR on how much of the block changed, per surface. A flip that rewrote
     only the footprint's `(layer ...)` line round-trips flawlessly and is the
     exact silent failure #714 is about.
  4. It runs on fixtures that already carry `(justify mirror)`, so the
     add-but-never-remove asymmetry has somewhere to show up.

A PAD CHILD NO TRACKED BOARD CARRIES is exercised on an injected copy: the
pad-to-die `(die_length ...)` / `(die_delay ...)` (#1248) on tigard J7. The
part must flip (it used to refuse `die_delay`), keep every value pad for pad
and token for token, and flip back byte-identically.

DIRECTION IS PINNED. Measured (`tests/measure_714_mirror_convention.py`
section F): `FLIP_DIRECTION_TOP_BOTTOM` is an involution on 15/15 fixtures;
`LEFT_RIGHT` is not, on 4 of them, because its text-angle rule collapses to
`a mod 180` (glasgow_revC SW1 180 -> 0, orangecrab_ext_pll J4 270 -> 90). So
byte-identity is statable only for TOP_BOTTOM -- which is the argument for the
writer supporting no other direction.

    python3 tests/test_714_flip_roundtrip.py
"""
from __future__ import annotations

import hashlib
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

RUN_ALL_TIMEOUT = 600

# `orangecrab_ext_pll J4` and `ulx3s BAT1` are already on B.Cu and already
# carry `(justify mirror)`, so the REMOVAL branch is exercised rather than
# only the insertion branch. `rp2350 J2` carries a footprint-internal
# `(zone ...)` and `orangecrab U9` custom-pad primitives. A fixture the
# transform refuses cannot round-trip, so none of these may be a refusal
# witness (tests/test_714_refusals.py).
FIXTURES = [
    ('tigard', 'J7'),
    ('tigard', 'J1'),
    ('glasgow_revC', 'SW1'),
    ('orangecrab_ext_pll', 'J4'),
    ('ulx3s', 'BAT1'),
    ('rp2350_fpga_eensy_prePlane', 'SW1'),
    ('rp2350_fpga_eensy_prePlane', 'J2'),
    ('orangecrab_ext_pll', 'U9'),
]
# Every part of the corpus-witness fixture the pcbnew parity gate compares:
# primitives, chamfered corners, absolute zones and points (here flipped at a
# HELD rotation, so their mirror is composed with a rotation and undone),
# groups, embedded files and the other passed-through constructs.
WITNESS_BOARD = 'tests/fixtures/714/flip_witnesses'
FIXTURES += [(WITNESS_BOARD, r) for r in (
    [f'WPRIM{i}' for i in range(1, 7)] + [f'WCHAM{i}' for i in range(1, 5)]
    + [f'W{t}{i}' for t in ('ZONE', 'SPMR', 'KEL', 'GRP', 'PT', 'PLC', 'EMB')
       for i in range(1, 4)]
    + ['WCCL1', 'WCCL2'])]

# Per-surface floors on how much of the block a real flip must change. A flip
# that rewrote one line would satisfy an involution perfectly.
MIN_CHANGED_LINES = 4


def _sha(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _control(src, dst):
    """The dose-0 pass of the same writer: every part at the pose it has."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(src)
    write_placed_output(src, dst, [
        {'reference': r, 'new_x': round(f.x, 6), 'new_y': round(f.y, 6),
         'new_rotation': f.rotation} for r, f in sorted(pcb.footprints.items())])
    return dst


def _flip(src, dst, ref, side):
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    fp = parse_kicad_pcb(src).footprints[ref]
    write_placed_output(src, dst, [{
        'reference': ref, 'new_x': round(fp.x, 6), 'new_y': round(fp.y, 6),
        'new_rotation': fp.rotation, 'new_side': side}])
    return dst


def _block_span(path, ref):
    from kicad_parser import iter_footprint_blocks
    text = open(path, encoding='utf-8').read()
    for s, e, _t, _raw, key in iter_footprint_blocks(text):
        if key == ref:
            return text, s, e
    return text, None, None


# #1248: a pad's pad-to-die length and delay. No tracked board carries either
# token, so they are INJECTED into tigard J7, after each pad's `(net ...)`
# where pcbnew 10 writes them: both on pad 1, one each on pads 2 and 3. A flip
# leaves both per-pad scalars alone -- pcbnew's FOOTPRINT::Flip does, and
# `tests/gui_parity/test_714_mirror_pcbnew_parity.py` holds this writer to it
# on the same injection. Before `die_delay` was whitelisted the writer refused
# to flip such a part at all.
DIE_REF = 'J7'
DIE_TOKENS = {'1': ('(die_length 1.25)\n\t\t\t(die_delay 7.5)', 1.25, 7.5),
              '2': ('(die_length 0.333)', 0.333, 0.0),
              '3': ('(die_delay 12.345678)', 0.0, 12.345678)}


def _inject_die(src, dst):
    """tigard with DIE_TOKENS added inside J7's pads, each after the pad's own
    `(net ...)`. Raises when an anchor is missing, so the arm cannot grade a
    board that carries no token."""
    import re
    from kicad_parser import iter_footprint_blocks
    # Universal newlines: the anchor below is LF-only, and core.autocrlf
    # checks tigard out with CRLF (so does `git archive` for the cloud image).
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    for s, e, body, _raw, key in iter_footprint_blocks(text):
        if key != DIE_REF:
            continue
        left = dict(DIE_TOKENS)
        for m in reversed(list(re.finditer(r'\(pad\s+"([^"]*)"', body))):
            if m.group(1) not in left:
                continue
            net = re.compile(r'\(net\s+"[^"]*"\)\n').search(body, m.start())
            nxt = body.find('(pad ', m.end())
            if net is None or (nxt != -1 and net.start() > nxt):
                raise AssertionError(f"{DIE_REF} pad {m.group(1)}: no (net ...)")
            body = (body[:net.end()] + '\t\t\t' + left.pop(m.group(1))[0]
                    + '\n' + body[net.end():])
        if left:
            raise AssertionError(f"{DIE_REF} pads {sorted(left)} not found")
        with open(dst, 'w', encoding='utf-8', newline='') as fh:
            fh.write(text[:s] + body + text[e:])
        return dst
    raise AssertionError(f"{DIE_REF} not in {src}")


def _die_arm(tmp, problems):
    """The pad-to-die tokens survive a flip unchanged, and flip back exactly.
    Returns True when the arm graded a flip."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import SideFlipUnsupported
    src = os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')
    try:
        inj = _inject_die(src, os.path.join(tmp, 'die_inj.kicad_pcb'))
    except AssertionError as exc:
        problems.append(f"die arm: injection failed ({exc}) -- re-anchor it")
        return False
    want = {n: (L, D) for n, (_t, L, D) in DIE_TOKENS.items()}
    got0 = {p.pad_number: (p.die_length, p.die_delay)
            for p in parse_kicad_pcb(inj).footprints[DIE_REF].pads
            if p.pad_number in want}
    if got0 != want:
        problems.append(f"die arm: the injected board parses {got0}, not {want} "
                        f"-- the fixture carries the wrong tokens")
        return False
    ctrl = _control(inj, os.path.join(tmp, 'die_ctrl.kicad_pcb'))
    try:
        f1 = _flip(ctrl, os.path.join(tmp, 'die_f1.kicad_pcb'), DIE_REF, 'B')
    except SideFlipUnsupported as exc:
        problems.append(f"die arm: the flip REFUSED a part carrying die_length/"
                        f"die_delay: {exc}")
        return False
    after = parse_kicad_pcb(f1).footprints[DIE_REF]
    if not (after.layer or '').startswith('B'):
        problems.append(f"die arm: asked for side B, got {after.layer!r} -- "
                        f"nothing was flipped, so the arm would pass for the "
                        f"wrong reason")
        return False
    got1 = {p.pad_number: (p.die_length, p.die_delay) for p in after.pads
            if p.pad_number in want}
    if got1 != want:
        problems.append(f"die arm: the flip changed the pad-to-die values "
                        f"{want} -> {got1}; pcbnew's flip leaves them alone")
    t1, s1, e1 = _block_span(f1, DIE_REF)
    block = t1[s1:e1] if s1 is not None else ''
    for tok in ('(die_length 1.25)', '(die_delay 7.5)', '(die_length 0.333)',
                '(die_delay 12.345678)'):
        if block.count(tok) != 1:
            problems.append(f"die arm: {tok} appears {block.count(tok)} times in "
                            f"the flipped block, not once, verbatim")
    f2 = _flip(f1, os.path.join(tmp, 'die_f2.kicad_pcb'), DIE_REF, 'F')
    if _sha(f2) != _sha(ctrl):
        problems.append("die arm: flip-and-flip-back of the part carrying "
                        "die_length/die_delay is NOT byte-identical")
    return True


def main():
    from kicad_parser import parse_kicad_pcb

    tmp = tempfile.mkdtemp(prefix='krt714rt')
    problems, checked = [], 0
    try:
        for board, ref in FIXTURES:
            src = (os.path.join(REPO, board + '.kicad_pcb') if '/' in board
                   else os.path.join(REPO, 'kicad_files', board + '.kicad_pcb'))
            if not os.path.isfile(src):
                problems.append(f"{board}: board not tracked")
                continue
            fp0 = parse_kicad_pcb(src).footprints.get(ref)
            if fp0 is None:
                problems.append(f"{board}:{ref} missing -- fixture stale")
                continue
            here = (fp0.layer or 'F.Cu')[0]
            other = 'F' if here == 'B' else 'B'

            ctrl = _control(src, os.path.join(tmp, f"{os.path.basename(board)}_{ref}_ctrl.kicad_pcb"))
            f1 = _flip(ctrl, os.path.join(tmp, f"{os.path.basename(board)}_{ref}_f1.kicad_pcb"),
                       ref, other)
            f2 = _flip(f1, os.path.join(tmp, f"{os.path.basename(board)}_{ref}_f2.kicad_pcb"),
                       ref, here)

            h_ctrl, h_f1, h_f2 = _sha(ctrl), _sha(f1), _sha(f2)

            # --- arm 1: the flip did something. Red today; without it every
            # other arm passes on a writer that ignores the side key.
            if h_f1 == h_ctrl:
                problems.append(
                    f"{board}:{ref} the flipped board is byte-identical to the "
                    f"control -- nothing was flipped, so the round trip below "
                    f"would pass for the wrong reason")
                continue
            checked += 1

            # --- arm 2: the delta is confined to this block
            t_ctrl, cs, ce = _block_span(ctrl, ref)
            t_f1, fs, fe = _block_span(f1, ref)
            if cs is None or fs is None:
                problems.append(f"{board}:{ref} block not locatable in the output")
            else:
                if t_ctrl[:cs] != t_f1[:fs] or t_ctrl[ce:] != t_f1[fe:]:
                    problems.append(
                        f"{board}:{ref} the flip changed bytes OUTSIDE its own "
                        f"footprint block")
                # --- arm 3: enough of the block actually changed
                a = t_ctrl[cs:ce].splitlines()
                b = t_f1[fs:fe].splitlines()
                changed = sum(1 for x, y in zip(a, b) if x != y) + abs(len(a) - len(b))
                if changed < MIN_CHANGED_LINES:
                    problems.append(
                        f"{board}:{ref} only {changed} line(s) of the block "
                        f"changed -- a flip that rewrites one line round-trips "
                        f"perfectly and is the silent failure this gate exists "
                        f"for")

            # --- arm 4: and back again, exactly
            if h_f2 != h_ctrl:
                problems.append(
                    f"{board}:{ref} flip-and-flip-back is NOT byte-identical\n"
                    f"      control {h_ctrl}\n      round   {h_f2}")
            # --- arm 5, on the FIRST fixture only: a coordinate that is not
            # already in `_fmt_mm`'s minimal form.
            #
            # This exists because the mutation battery said it had to. The row
            # `the-negation-goes-through-float` replaces the sign toggle with
            # `_fmt_mm(-float(tok))` and SURVIVED every gate here -- and the
            # reason is a fact about the corpus, not about the code: measured,
            # all 148417 coordinate literals on the 22 tracked boards are
            # ALREADY in minimal form and none carries more than six decimals,
            # so on this input the float round trip happens to be an
            # involution too.
            #
            # That makes the sign toggle's advantage real but undemonstrated:
            # it is exact BY CONSTRUCTION over the declared grammar, whereas
            # the float path is exact only because of a property of these
            # particular files that another tool's board can break. So the
            # discriminating input is synthesised rather than found.
            if checked == 1:
                nm = os.path.join(tmp, 'nonminimal.kicad_pcb')
                # newline='' on BOTH sides: without it the read translates
                # CRLF to LF and the rewrite emits LF, so the round trip
                # compares two different line endings and fails for a reason
                # that has nothing to do with the mirror. (It did.)
                with open(ctrl, encoding='utf-8', newline='') as fh:
                    text = fh.read()
                # Scoped to THIS block. `(at -1.5 -2)` is an ordinary pad
                # coordinate and occurs on other footprints too; a whole-file
                # replace can land in a block the flip never touches, and the
                # arm then round-trips trivially.
                #
                # The span is computed on THIS string, not via `_block_span`,
                # which does its own newline-translating read: on Windows a
                # `newline=''` read keeps CRLF and a default read collapses it,
                # so the two disagree by one byte per line and the window comes
                # out shifted. (It did, and the arm reported a lost anchor.)
                from kicad_parser import iter_footprint_blocks
                bs = be = None
                for _s, _e, _t, _raw, _k in iter_footprint_blocks(text):
                    if _k == ref:
                        bs, be = _s, _e
                        break
                marker = '(at -1.5 -2)'
                if bs is None or marker not in text[bs:be]:
                    problems.append(
                        f"the non-minimal-literal arm lost its anchor "
                        f"{marker!r} inside {ref} -- re-anchor it rather than "
                        f"dropping it, or the float path stops being "
                        f"distinguishable")
                else:
                    block = text[bs:be].replace(marker, '(at -1.5 -2.500000)', 1)
                    with open(nm, 'w', encoding='utf-8', newline='') as fh:
                        fh.write(text[:bs] + block + text[be:])
                    g1 = _flip(nm, os.path.join(tmp, 'nm_f1.kicad_pcb'),
                               'J7', 'B')
                    g2 = _flip(g1, os.path.join(tmp, 'nm_f2.kicad_pcb'),
                               'J7', 'F')
                    if _sha(g2) != _sha(nm):
                        problems.append(
                            "a coordinate written non-minimally "
                            "(`-2.500000`) does not survive flip-and-flip-back "
                            "-- the transform is reformatting numbers instead "
                            "of toggling their sign, so it is an involution "
                            "only for input that was already minimal")
        if _die_arm(tmp, problems):
            checked += 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if checked == 0 and not problems:
        problems.append("no fixture round-tripped -- nothing was graded")

    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for p in problems[:25]:
            print("  " + p)
        return 1
    print(f"PASS: {checked} fixtures flip and flip back byte-identically")
    return 0


if __name__ == '__main__':
    sys.exit(main())
