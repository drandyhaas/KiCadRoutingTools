#!/usr/bin/env python3
"""#836's own kill switch, run: how many parts does a FLIP help that a MOVE does not?

Issue #836 asks for a side-flip move in the placement search and pre-registers
the check that decides whether it should exist at all, verbatim:

    A cheap pre-registered check: on at least three corpus boards, how many
    parts does a flip help that a re-seat does not? If the answer is near
    zero, this issue should be closed rather than built.

This is that check. It asserts nothing and is not collected by
`tests/run_all.py` (not named `test_*`); `tests/test_836_flip_census.py` is the
change detector that keeps its numbers from rotting.

    python3 -X utf8 tests/measure_836_flip_vs_reseat.py --table ABD
    python3 -X utf8 tests/measure_836_flip_vs_reseat.py --out after.json
    python3 -X utf8 tests/measure_836_flip_vs_reseat.py --diff before.json after.json

THE OBJECTIVE MAY NOT VOTE, AND THAT IS THE WHOLE DESIGN.

#836 says "the objective cannot price the move -- the only term that responds
to a flip at all is 2-D MST length". Measured against the code, that is
backwards, and the correction is what this rig is built around:

  * `quench._halo_pair_penalty` returns 0.0 for a cross-side SMD pair, so a
    flip ZEROES a part's halo charge against every same-side SMD neighbour at
    once;
  * `candidate_valid`'s pair loop skips cross-side pairs entirely, and
    `pair_min_gap` returns None for an unshared side, so a flipped part stops
    colliding with its own face;
  * `align` has no side filter at all, so an F/B pair of one footprint stays
    "aligned";
  * length and crossings carry no layer column (airwires are x1,y1,x2,y2,net),
    so they barely move for a two-pad passive mirrored about its own origin.

Every one of those responds in ONE direction. A flip to the emptier face is
priced as nearly-free relief and is strictly more legal, so a cost delta would
report that flipping helps almost every part on the board -- including boards
whose back face is empty, where the answer is meaningless. Table B measures
that artefact and publishes it as `blind_delta`; nothing here votes on it.

The verdict instruments are the ones that charge the DESTINATION face on the
same ledger they credit the source face: the assembly gate
(`grade_body_overlap` + `grade_pad_legality`), the escape ledger's deficit
lanes (#835/#848/#850 made `_blocked_span` symmetric), and `grow_board`'s
per-side utilisation -- which is exactly the credit #836 says is "already
granted and never earned".

WHAT THE FLIP IS COMPARED AGAINST. Not `reseat.py` as shipped:
`clusters_from_tethers` sources clusters from `groups.decap_tethers` and
NOTHING else, so it reaches a minority of any board's movable parts (table A
measures the fraction). Comparing against it alone would manufacture a win on
every part it cannot see. The move arm is the union of native reseat and
`pose_score.rank_poses`, which enumerates every legal pose of the part on the
rotation lattice within a radius and so strictly dominates reseat's slot pool
for a single part. `EXCLUSIVE` therefore means "no same-side move within reach
helps this part", which is #836's actual question.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'tests')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                             # noqa: E402

# --- pre-registration -------------------------------------------------------
#
# Written down BEFORE the first full run and committed in its own commit, so
# git makes the order checkable. A threshold chosen after seeing the numbers is
# not a threshold.
PREREGISTRATION = {
    'question': ('how many parts does a flip help that no same-side move '
                 'within reach helps'),
    'boards': ['ulx3s', 'glasgow_revC', 'orangecrab_ext_pll'],
    'E_close': 1,          # E <= this AND E_material == 0  -> close #836
    'E_build': 6,          # E >= this, on all three boards -> build it
    'E_material_build': 3,
    'C4_ratio': 0.5,       # rate(B->F) must be <= this * rate(F->B)
    'V1_floor': 0.85,      # blind_delta must help >= this fraction, or the
                           # reading of the objective above is wrong and the
                           # run decides nothing
    'gate_keys': ['blocking', 'containment_blocking', 'courtyard_blocking',
                  'pad_conflicts', 'hole_conflicts', 'oob_pad_count'],
}

#: The three primaries, and the reason each is here rather than convenient.
PRIMARY = ('ulx3s', 'glasgow_revC', 'orangecrab_ext_pll')
#: Zero back-side pad copper. Their `helps_flip` rate is the ARTEFACT rate:
#: every flip lands in a perfectly empty face, so the gate trivially clears.
#: A finding on a primary is only a finding if it beats this.
CONTROL = ('splitflap_driver', 'watchy', 'flat_hierarchy')

CLEARANCE = 0.2
EDGE_CLEARANCE = 0.5
RESEAT_RADIUS_MM = 5.0
#: The move arm's reach. Pre-registered, and BOUNDED: `rank_poses` pays a full
#: `total_cost()` per (offset, rotation), so 3.0mm at 0.5mm is 452 evaluations
#: per candidate and minutes per part on a 235-part board. 1.5mm is 116, which
#: is still a strict superset of `reseat.slot_pool`'s 32 positions x 4
#: rotations for a single part. A run that cannot finish measures nothing, and
#: the sensitivity of the answer to this number is reported rather than
#: assumed: `--rank-radius` re-runs it on the EXCLUSIVE set alone.
RANK_RADIUS_MM = 1.5
RANK_STEP_MM = 0.5
RANK_LIMIT = 6


def _sha(path):
    import hashlib
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def engine_sha():
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                              capture_output=True, text=True).stdout.strip()
    except Exception:                                          # noqa: BLE001
        return 'unknown'


def io_read(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def board_path(stem):
    return os.path.join(ROOT, 'kicad_files', f'{stem}.kicad_pcb')


def _identity_placements(state):
    """Every movable part at its current pose. `perturb._all_at_current`'s
    shape, rebuilt here so this rig does not depend on a damage stager."""
    return [{'reference': r, 'new_x': round(p.x, 6), 'new_y': round(p.y, 6),
             'new_rotation': p.rot}
            for r, p in state.parts.items()]


def _write(src, dst, placements):
    from placement.writer import write_placed_output
    write_placed_output(src, dst, placements)
    # Siblings travel (#441): a board written without its `.kicad_pro`
    # resolves its floor from the STOCK netclass and manufactures phantom DRC.
    base = os.path.splitext(src)[0]
    for ext in ('.kicad_pro', '.kicad_dru'):
        if os.path.isfile(base + ext):
            shutil.copyfile(base + ext, os.path.splitext(dst)[0] + ext)
    return dst


def gate_of(path):
    """The assembly gate's own currency, on a board file."""
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_body_overlap, grade_pad_legality
    pcb = parse_kicad_pcb(path)
    g = grade_body_overlap(pcb, CLEARANCE, pcb_file=path)
    leg = grade_pad_legality(pcb, CLEARANCE, worst_n=0, pcb_file=path)
    return {
        'blocking': g['blocking'],
        'containment_blocking': g['containment_blocking'],
        'courtyard_blocking': g['courtyard_blocking'],
        'pad_conflicts': leg['pad_conflicts'],
        'hole_conflicts': leg['hole_conflicts'],
        'oob_pad_count': leg['oob_pad_count'],
    }


def escape_lanes(path):
    """Deficit lanes, both the plain and the layer-aware floor (#700)."""
    from kicad_parser import parse_kicad_pcb
    from placement.escape import escape_ledger
    from placement.options import deficit_totals
    pcb = parse_kicad_pcb(path)
    led = escape_ledger(pcb, pcb_file=path, clearance=CLEARANCE)
    return {'deficit': deficit_totals(led)['lanes'],
            'deficit_floor': deficit_totals(led, field='deficit_floor')['lanes']}


def utilisation(path, sides=None):
    from kicad_parser import parse_kicad_pcb
    from placement.options import grow_board
    g = grow_board(parse_kicad_pcb(path), path, clearance=CLEARANCE,
                   board_edge_clearance=EDGE_CLEARANCE, assembly_sides=sides)
    m = g.get('measured') or {}
    return {'utilisation': m.get('utilisation'),
            'charged_area_mm2': m.get('charged_area_mm2'),
            'fits': g.get('fits_by_area')}


def _worse(a, b, keys):
    """Is `b` worse than `a` on any key?"""
    return any((b.get(k) or 0) > (a.get(k) or 0) for k in keys)


def _better(a, b, keys):
    """Is `b` better than `a` on some key and worse on none?"""
    return (not _worse(a, b, keys)
            and any((b.get(k) or 0) < (a.get(k) or 0) for k in keys))


# --- table A: the census ----------------------------------------------------

def table_a(stems):
    """Eligibility, writer refusals, and how much of the board a reseat sees."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import SideFlipUnsupported
    from pose_score import make_state
    rows = {}
    for stem in stems:
        path = board_path(stem)
        run_utils.evidence(path, f'the {stem} board')
        pcb = parse_kicad_pcb(path)
        st = make_state(pcb, path)
        movable = [r for r, p in st.parts.items() if not p.locked]
        refused, reasons = [], {}
        # The refusal census asks `_flip_footprint_block` directly rather than
        # writing the whole board once per part. Same function, same raise --
        # `write_placed_output` reaches it through a loop over blocks -- but a
        # whole-board write costs ~1s on a 1.9MB board, which is 4 minutes per
        # board to learn something the block transform answers in milliseconds.
        from placement.writer import _flip_footprint_block
        from kicad_parser import iter_footprint_blocks
        content = io_read(path)
        for _s, _e, fp_text, _raw, key in iter_footprint_blocks(content):
            if key not in st.parts:
                continue
            rot = st.parts[key].rot
            try:
                _flip_footprint_block(fp_text, key, rot, rot)
            except SideFlipUnsupported as exc:
                refused.append(key)
                reasons[key] = str(exc)[:120]
            except Exception as exc:                           # noqa: BLE001
                refused.append(key)
                reasons[key] = f"{type(exc).__name__}: {str(exc)[:100]}"
        cl = _reseat_clusters(pcb, st)
        covered = {m for c in cl for m in c.members}
        rows[stem] = {
            'parts': len(st.parts),
            'movable': len(movable),
            'flip_refused': len(refused),
            'refusal_reasons': reasons,
            'eligible': len(movable) - len(refused),
            'reseat_clusters': len(cl),
            'reseat_members': len(covered),
            'reseat_native_coverage': (round(len(covered) / len(movable), 4)
                                       if movable else None),
        }
    return rows


def halo_relief(state):
    """The charge a flip would ZERO, per part -- the artefact, quantified.

    `_halo_pair_penalty` returns 0.0 the moment two SMD parts are on opposite
    faces, so every same-side SMD halo charge a part pays is relief a flip
    grants it for free. This is computed and PUBLISHED and votes on nothing:
    its size is the reason the verdict has to come from somewhere else.
    """
    per_part, positive = {}, 0
    for ref, part in state.parts.items():
        # The engine's own loop shape (`part_geometry_cost`), calling the
        # engine's own penalty. A re-implementation here would be measuring my
        # arithmetic rather than the objective's.
        rects = part.rects(part.x, part.y, part.rot)
        rect = rects[0]
        if state._neighbors is not None and ref in state._neighbors:
            others = ((o, state.parts[o]) for o in state._neighbors[ref])
        else:
            others = state.parts.items()
        tot = 0.0
        for other_ref, other in others:
            if other_ref == ref:
                continue
            # Only the charge a FLIP would zero: same-side, and neither part
            # through-hole (the penalty falls through to `gap_to` when either
            # is, and a THT part reaches both faces anyway).
            if other.side != part.side or part.has_tht or other.has_tht:
                continue
            tot += state._halo_pair_penalty(part, rect, other, other.rect(),
                                            rects_a=rects)
        per_part[ref] = round(tot, 6)
        positive += (tot > 0)
    return per_part, positive


def _reseat_clusters(pcb, state):
    from placement.reseat import clusters_from_tethers
    try:
        return clusters_from_tethers(pcb, state, RESEAT_RADIUS_MM)
    except Exception:                                          # noqa: BLE001
        return []


def _instruments(path):
    """The INDEPENDENT verdict currency. Never the quench objective.

    Two channels, kept APART on purpose, because they have very different
    standing:

    * the GATE keys are real defects -- pads intersecting, a body contained,
      copper off the board. An improvement there is a board that got more
      buildable, and it is what `EXCLUSIVE` is defined on.
    * `busiest_area_mm2` is the per-side area credit #836 names. It is
      reported and NEVER decides, because on any board whose back face is
      emptier than its front -- which is every board in this corpus, by area
      -- moving ANY part to the back reduces it. That is arithmetic, not a
      finding, and folding it into the verdict would report "a flip helps"
      for every part on every board.

    The area is charged on the `both` (max-per-side) basis deliberately, even
    on a board whose observed policy is one face: under a one-face policy the
    charge is the SUM, which is invariant under a flip, so the instrument
    would be silently inert rather than merely weak.
    """
    g = gate_of(path)
    g['busiest_area_mm2'] = utilisation(path, 'both')['charged_area_mm2']
    return g


def table_b(stems, limit=None, quiet=True):
    """The screen: does a flip help on an instrument a same-side move cannot?"""
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    from placement.writer import SideFlipUnsupported
    from pose_score import make_state, rank_poses

    rows = {}
    gk = PREREGISTRATION['gate_keys']
    for stem in stems:
        path = board_path(stem)
        run_utils.evidence(path, f'the {stem} board')
        pcb = parse_kicad_pcb(path)
        st = make_state(pcb, path)
        from placement.legality import assembly_census
        sides = assembly_census(pcb)['sides']
        halo, halo_positive = halo_relief(st)
        movable = [r for r, p in st.parts.items() if not p.locked]
        if limit:
            movable = movable[:limit]

        tmp = tempfile.mkdtemp(prefix='m836b')
        buf = io.StringIO()
        try:
            base_pl = _identity_placements(st)
            # C0: the identity write must be inert. If it is not, every
            # candidate below is measured against a board this rig moved.
            c0 = os.path.join(tmp, 'c0.kicad_pcb')
            with contextlib.redirect_stdout(buf):
                _write(path, c0, base_pl)
            ctrl = _instruments(c0)
            orig = _instruments(path)
            c0_inert = (ctrl == orig)

            helps_flip, helps_move, exclusive, refused = [], [], [], []
            area_relief = []
            for _i, ref in enumerate(movable):
                if _i and _i % 25 == 0:
                    print(f"    {stem}: {_i}/{len(movable)}", flush=True)
                pl = [dict(d) for d in base_pl]
                for d in pl:
                    if d['reference'] == ref:
                        d['new_side'] = 'B' if st.parts[ref].side == 'F' else 'F'
                fpath = os.path.join(tmp, 'flip.kicad_pcb')
                try:
                    with contextlib.redirect_stdout(buf):
                        _write(path, fpath, pl)
                except (SideFlipUnsupported, Exception):       # noqa: BLE001
                    refused.append(ref)
                    continue
                f_inst = _instruments(fpath)
                f_better = _better(ctrl, f_inst, gk)
                if f_inst['busiest_area_mm2'] < ctrl['busiest_area_mm2']:
                    area_relief.append(ref)

                # The MOVE arm, and it is evaluated the same way -- same
                # writer, same graders, same board file -- so the two arms
                # cannot differ by their measurement.
                m_better = False
                if f_better:
                    poses = rank_poses(pcb, path, ref, radius=RANK_RADIUS_MM,
                                       step=RANK_STEP_MM, limit=RANK_LIMIT,
                                       state=st)
                    for cand in poses[:RANK_LIMIT]:
                        mpl = [dict(d) for d in base_pl]
                        for d in mpl:
                            if d['reference'] == ref:
                                d['new_x'] = round(cand['x'], 6)
                                d['new_y'] = round(cand['y'], 6)
                                d['new_rotation'] = cand['rot']
                        mpath = os.path.join(tmp, 'move.kicad_pcb')
                        try:
                            with contextlib.redirect_stdout(buf):
                                _write(path, mpath, mpl)
                        except Exception:                      # noqa: BLE001
                            continue
                        if _better(ctrl, _instruments(mpath), gk):
                            m_better = True
                            break
                if f_better:
                    helps_flip.append(ref)
                    if m_better:
                        helps_move.append(ref)
                    else:
                        exclusive.append(ref)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

        rows[stem] = {
            'sides_observed': sides,
            'movable': len(movable),
            'flip_refused': len(refused),
            'control_write_inert': c0_inert,
            'control_instruments': ctrl,
            # The artefact. Published, and it votes on nothing.
            'blind_halo_relief_parts': halo_positive,
            'blind_halo_relief_frac': (round(halo_positive / len(st.parts), 4)
                                       if st.parts else None),
            'blind_halo_relief_sum': round(sum(halo.values()), 3),
            # Reported, never decisive: see `_instruments`.
            'area_relief_parts': len(area_relief),
            'helps_flip': len(helps_flip),
            'helps_move': len(helps_move),
            'exclusive': len(exclusive),
            'exclusive_refs': sorted(exclusive)[:40],
        }
        print(f"{stem}: helps_flip={len(helps_flip)} helps_move="
              f"{len(helps_move)} EXCLUSIVE={len(exclusive)} "
              f"halo-relief parts={halo_positive}/{len(st.parts)} "
              f"c0_inert={c0_inert}", flush=True)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--table', default='AB')
    ap.add_argument('--boards', nargs='+', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--diff', nargs=2, default=None, metavar=('BEFORE', 'AFTER'))
    ap.add_argument('--limit', type=int, default=None,
                    help='candidates per board (a SMOKE bound, never a result)')
    a = ap.parse_args(argv)
    if a.diff:
        return _diff(a.diff[0], a.diff[1])
    stems = a.boards or list(PRIMARY)
    doc = {'preregistration': PREREGISTRATION, 'engine_sha': engine_sha(),
           'clearance': CLEARANCE, 'generated_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ',
                                                                 time.gmtime())}
    if 'A' in a.table:
        doc['census'] = table_a(stems)
        print(json.dumps(doc['census'], indent=1, sort_keys=True))
    if 'B' in a.table:
        doc['screen'] = table_b(stems, limit=a.limit)
        doc['limited'] = a.limit
        print(json.dumps(doc['screen'], indent=1, sort_keys=True))
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
        print(f"wrote {a.out}")
    return 0


def _diff(before, after):
    b = json.load(open(before, encoding='utf-8'))
    c = json.load(open(after, encoding='utf-8'))
    for section in sorted(set(b) | set(c)):
        if not isinstance(b.get(section), dict):
            continue
        for k in sorted(set(b.get(section, {})) | set(c.get(section, {}))):
            if b.get(section, {}).get(k) != c.get(section, {}).get(k):
                print(f"{section}.{k}: {b.get(section, {}).get(k)} -> "
                      f"{c.get(section, {}).get(k)}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
