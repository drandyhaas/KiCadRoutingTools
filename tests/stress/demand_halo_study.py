#!/usr/bin/env python3
"""#700 item 3, measured: would a demand-derived halo buy anything?

Issue #700's third suggestion is "only then consider a demand-derived halo,
and only as an objective term behind a default-off flag". This script is the
CONSIDERATION, committed so the numbers in docs/placement-predictors.md have a
derivation someone can re-run rather than a claim someone has to trust.

The proposed term replaces the quench's shape halo

    halo_i = halo_base + halo_coef * sqrt(pin_count)          (quench.py:581)

with, or rather takes the max against,

    demand_i * lane_pitch / copper_layers

where `demand_i` is the largest per-face net count on the part. The formula is
not invented here -- .claude/skills/plan-pcb-placement/SKILL.md:430-437 already
states `cut_nets x (track_width + clearance) / copper_layers` verbatim, layer
divisor included.

Two questions, and neither is "does the number change":

  A. Does the term reach any blocker the shipped halo does not already charge?
     The escape ledger NAMES the parts eating a deficit face (`blockers`,
     escape.py:308). For each such pair, compare the pad-rect gap against the
     shipped pair requirement `halo_a + halo_b` at the A/B harness's own
     coefficients (tests/test_placement_ab.py:312-316, coef 0.15). A pair
     already inside that requirement is one the OFF arm already repels; the
     term can only charge it MORE, which is a magnitude change, not a new
     finding.

  B. Where does the term fire, against where the grader can see? The only
     independent escape instrument the placement A/B has is
     `health_escape_deficit_parts` / `health_escape_worst_deficit`
     (routability.py:718-722 -> floorplan.py:2455-2458). A term that is large
     where those read zero, and inert where they read their maximum, cannot be
     graded by them however the run comes out.

Run:  python3 -X utf8 tests/stress/demand_halo_study.py
No routing, no LLM, no board written. Seconds.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import escape                                # noqa: E402
from placement.legality import rect_gap                     # noqa: E402

# tests/test_placement_ab.py:312-316 QUENCH_BASE -- the arm this term would
# have to beat. NOT quench()'s own 0.25 default: the harness that decides
# whether a placement term ships is the one whose numbers matter.
HALO_BASE, HALO_COEF = 0.5, 0.15

# Boards with fine-pitch parts, so the ledger has something to say. The four
# the A/B harness itself uses, plus the two densest others.
BOARDS = ('rp2350_fpga_eensy_prePlane', 'orangecrab_ext_pll', 'ulx3s',
          'kit-dev-coldfire-xilinx_5213', 'watchy', 'glasgow_revC', 'tigard',
          'esp_prog')


def _shape_halo(pin_count):
    return HALO_BASE + HALO_COEF * math.sqrt(max(pin_count, 1))


def _face_demand(fp, lane):
    """Largest per-face distinct-net count, by the ledger's own face rule."""
    rect = escape._part_rect(fp)
    pitch = escape.pad_pitch(fp)
    per = {f: set() for f in escape.FACES}
    for pad in fp.pads:
        if not getattr(pad, 'net_id', 0):
            continue
        face = escape._face_of(pad, rect,
                               pitch if pitch != float('inf') else lane)
        if face:
            per[face].add(pad.net_id)
    return max((len(v) for v in per.values()), default=0)


def _load(name):
    path = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
    if not os.path.isfile(path):
        return None, None
    return parse_kicad_pcb(path), path


def question_a():
    """Does the term reach a blocker the shipped halo does not already charge?"""
    print('A. Blockers the escape ledger NAMES, against the shipped halo')
    print('   (pair charged = pad-rect gap < halo_a + halo_b at coef %.2f)'
          % HALO_COEF)
    total = charged = 0
    uncharged = []
    for name in BOARDS:
        pcb, path = _load(name)
        if pcb is None:
            continue
        led = escape.escape_ledger(pcb, pcb_file=path)
        rects = {r: escape._part_rect(f)
                 for r, f in pcb.footprints.items() if f.pads}
        pins = {r: len([p for p in f.pads if p.net_id > 0])
                for r, f in pcb.footprints.items() if f.pads}
        t = c = 0
        for pe in led:
            for face in pe.faces:
                if face.deficit <= 0:
                    continue
                for b in face.blockers:
                    if pe.ref not in rects or b not in rects:
                        continue
                    t += 1
                    # `rect_gap` returns a float always -- negative when the
                    # rects overlap -- so there is no None case to guard.
                    gap = rect_gap(rects[pe.ref], rects[b])
                    need = _shape_halo(pins[pe.ref]) + _shape_halo(pins[b])
                    if gap < need:
                        c += 1
                    else:
                        uncharged.append((name, pe.ref, b, gap, need))
        if t:
            print('   %-30s %3d blockers, %3d already charged (%.1f%%)'
                  % (name, t, c, 100.0 * c / t))
        total += t
        charged += c
    print('   %-30s %3d blockers, %3d already charged (%.1f%%)'
          % ('TOTAL', total, charged, 100.0 * charged / max(1, total)))

    # The whole question is whether the term reaches the REMAINDER.
    print('\n   the %d pairs the shipped halo does NOT charge, and whether the'
          ' demand term would:' % len(uncharged))
    fired = 0
    seen = set()
    for name, a, b, gap, need in uncharged:
        pcb, path = _load(name)
        ncu = len(pcb.board_info.copper_layers or ['F.Cu', 'B.Cu'])
        lane = escape.lane_pitch(pcb, path)
        hits = []
        for ref in (a, b):
            fp = pcb.footprints[ref]
            pin = len([p for p in fp.pads if p.net_id > 0])
            want = _face_demand(fp, lane) * lane / ncu
            if want > _shape_halo(pin) + 1e-9:
                hits.append(ref)
            seen.add((name, ref))
        fired += len(hits)
        print('     %-28s %-6s/%-6s gap %.3f vs %.3f  demand term fires on: %s'
              % (name, a, b, gap, need, ', '.join(hits) or 'NEITHER'))
    print('\n   the demand term fires on %d of the %d parts in those pairs.'
          % (fired, len(seen)))
    return total, charged, len(uncharged), fired


def question_b():
    """Where the term fires, against where the grader can see."""
    print('\nB. Where the term fires vs where the escape grader can see')
    print('   %-30s %-8s %-8s %s' % ('board', 'deficit', 'worst', 'term fires on'))
    for name in BOARDS:
        pcb, path = _load(name)
        if pcb is None:
            continue
        led = escape.escape_ledger(pcb, pcb_file=path)
        short = [p for p in led if p.worst]
        worst = max([p.worst.deficit for p in led if p.worst], default=0)

        # The SHIPPED resolver, not a second opinion: reimplementing the
        # plane test here is how a study and the tool it is about come to
        # disagree about the board they are both reading.
        nsig, _source, planes = escape.signal_layer_count(pcb)
        lane = escape.lane_pitch(pcb, path)
        fires = parts = 0
        worst_ask = (0.0, '')
        for ref, fp in pcb.footprints.items():
            pin = len([p for p in fp.pads if p.net_id > 0])
            if not pin:
                continue
            parts += 1
            want = _face_demand(fp, lane) * lane / nsig
            if want > _shape_halo(pin) + 1e-9:
                fires += 1
                if want > worst_ask[0]:
                    worst_ask = (want, ref)
        print('   %-30s %-8s %-8s %d/%d (%.0f%%)  [%d Cu, %d plane layer(s)]'
              % (name, len(short), worst, fires, parts,
                 100.0 * fires / max(1, parts),
                 len(pcb.board_info.copper_layers or ()), len(planes))
              + ('' if not worst_ask[1] else
                 '  worst ask %.2fmm (%s)' % worst_ask))


def main():
    total, charged, n_uncharged, fired = question_a()
    question_b()
    print('\nVERDICT INPUTS')
    print('  %.1f%% of named escape blockers are already charged by the '
          'shipped halo.' % (100.0 * charged / max(1, total)))
    print('  Of the %d it does not charge, the demand term reaches %d.'
          % (n_uncharged, fired))
    print('  See docs/placement-predictors.md for what was concluded and why.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
