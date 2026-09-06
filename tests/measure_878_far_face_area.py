#!/usr/bin/env python3
"""#878: what a through-hole part's leads cost on the FAR face, measured.

`options.grow_board` charges a part's area to its footprint layer only
(`options.py:245-246`). A drilled part's leads occupy the other face too, and
that area is charged to nobody. Three instruments in this tree charge a drilled
part per face and they do not agree about WHAT the far face costs, so #878 asks
for the currency to be DECIDED BY MEASUREMENT before the engine changes. This
is that measurement.

    python3 -X utf8 tests/measure_878_far_face_area.py
    python3 -X utf8 tests/measure_878_far_face_area.py --table control,charge
    python3 -X utf8 tests/measure_878_far_face_area.py --boards flat_hierarchy tigard
    python3 -X utf8 tests/measure_878_far_face_area.py --out after.json
    python3 -X utf8 tests/measure_878_far_face_area.py --diff before.json after.json

BOTH ARMS ARE MEASURED IN ONE PROCESS. This file carries its own copy of
`grow_board`'s charging loop, parametrised by currency, and prints all three
side by side -- so no column depends on checking out a parent commit, and the
"before" column re-measures when the next change lands. It is the discipline
`tests/measure_850_848_faces.py` states, for the same reason.

The copy CALLS the engine's helpers rather than re-deriving them
(`extract_courtyard_bboxes`, `compute_footprint_bbox_local`,
`rotate_local_bounds`, `footprint_side`, `hosts_the_design`, `CONTAINER_RATIO`,
`footprint_has_through_pads`, `part_local_bounds`), and it reaches nothing
private: the far-face box comes from `legality.part_local_bounds(...).tht_local`,
the public reader, not from `quench._through_pad_bounds_local`.

TWO NEGATIVE CONTROLS, and a failure of either REFUSES the whole report:

  NC1  the arm the ENGINE implements must reproduce `options.grow_board`
       EXACTLY -- nine fields per (board, basis), not just the answer. This is
       what proves the harness measures the engine and not itself, and since
       #878 landed it also pins WHICH currency ships. A report whose control
       did not reproduce is a page of numbers about nothing.
  NC2  a board carrying NO drilled pad must be bit-identical under all three
       currencies on every basis. If a currency moves one of those, the harness
       is charging something other than the far face and every cell is suspect.

THE OBSTRUCTION PREDICATE IS NOT THE ASSEMBLY PREDICATE, and conflating them is
the easiest way to get this wrong. Which parts reach the far face is
`footprint_has_through_pads` -- plain `drill > 0`, because an unplated hole
blocks the far side exactly as a plated one does (`legality.py:245-252`). Which
FACES a board is populated on is `assembly_census`, which asks
`pad_is_plated_through`, because an NPTH alignment post is not a soldered pin
(`legality.py:296-305`). #878's own per-board figures were counted with the
second rule; this file reports both columns so they cannot be read as one.

Boards come from `run_utils.corpus_boards()` -- the git-TRACKED set, 22 today.
A plain glob of `kicad_files/` also returns generated boards and gives 27-33.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
for _p in (_HERE, ROOT, os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools'),
           os.path.join(ROOT, 'py_placer', 'placement')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                             # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import legality as L                          # noqa: E402
from placement import options as O                           # noqa: E402
from placement.parser import extract_courtyard_bboxes        # noqa: E402
from placement.parser import extract_courtyard_sides         # noqa: E402
from placement.utility import compute_footprint_bbox_local   # noqa: E402

SKIP_EXIT = 77

#: ONE BASIS, stated rather than defaulted. 0.2 / 0.5 are what
#: `tests/test_capacity_options.py` grades `grow_board` at, so every number
#: here is comparable with the assertions that pin the engine.
#: `check_capacity.py`'s own default edge clearance is 0.55, which is why the
#: robustness arm re-runs the verdict there (H5).
CLEARANCE = 0.2
BOARD_EDGE_CLEARANCE = 0.5
EDGE_SENSITIVITY = 0.55

CURRENCIES = ('none', 'tht', 'courtyard')

#: The three bases, and which of them the pre-registered rule is allowed to
#: read. `one_face_declared` exists because `--assembly-sides F` is a supported
#: flag a designer can point at ANY board to ask "could this be built on one
#: side?" -- but on a board the census calls two-sided that is a counterfactual,
#: and a counterfactual printed beside real numbers WILL be quoted as evidence.
#: So it is measured, labelled `hypothetical`, and pre-registered as NOT
#: decision-bearing.
BASES = ('busiest', 'one_face_observed', 'one_face_declared')
DECISION_BASES = ('busiest', 'one_face_observed')

PREREGISTRATION = {
    'question': ('which rect does a through-hole part present on the FAR face '
                 'when grow_board charges part area: the whole courtyard, the '
                 'drilled-pad rect, or nothing'),
    'currencies': list(CURRENCIES),
    'bases': list(BASES),
    'decision_bases': list(DECISION_BASES),
    'basis_population': {
        'busiest': 'every tracked board; charged = max(per_side)',
        'one_face_observed':
            "boards whose assembly_census['sides'] is 'F' or 'B'; charged = "
            'sum(per_side)',
        'one_face_declared':
            'every tracked board, declaring the census face where there is '
            'one and the majority pad-bearing face otherwise; REPORTED ONLY, '
            'never read by the rule -- on a two-sided board it is a '
            'counterfactual',
    },
    'obstruction_predicate': 'legality.footprint_has_through_pads (drill > 0)',
    'assembly_predicate': 'legality.assembly_census (pad_is_plated_through)',
    # The granularity of grow_board's OWN output: its action string prints
    # `{util:.0%}` (options.py:358), so 0.005 is exactly where the number a
    # human reads changes. Not a round number chosen to reach a conclusion.
    'MOVE_EPS': 0.005,
    # A one-face board's `charged` is sum(per_side). A currency that charges
    # the far face as much as the near one is putting the same physical area
    # into that sum twice. The bound carries its own counterexample and the
    # counterexample is knowable from the part census WITHOUT running the
    # sweep: flat_hierarchy is 64 pad-bearing parts, all 64 drilled, census
    # 'F'. Under `courtyard` the far charge IS the near charge part for part,
    # so far/near == 1.000 and the utilisation exactly doubles. 0.9 sits below
    # that provable 1.0 and above what a mixed board reaches.
    'F1_DOUBLE_MAX': 0.9,
    'F1_WITNESS': 'flat_hierarchy',
    'F1_WITNESS_PREDICTION': 1.0,
    # Boards carrying no drilled pad at all: NC2's population, known a priori.
    'NC2_BOARDS': ['haasoscope_pro_max_test', 'qfn_diffpair_escape',
                   'qfn_interior_pads', 'qfn_underpad_coupling',
                   'routed_output'],
    'default_currency': 'tht',
    'default_reason': (
        'legality.rect_on is the only place in the tree where the far-face '
        'rect is DECIDED rather than sidestepped, and it answers tht_rect. '
        'check_pockets.courtyard_cover is not a competing decision: it answers '
        'a per-window COVER question where summing never arises. grow_board '
        'giving a third answer is a second answer to a settled question.'),
    'rules': [
        'STEP 0  if NC1 or NC2 fails on any row, the sweep decides NOTHING.',
        'STEP 0b if courtyard_sides_differ > 0, the `courtyard` arm is '
        'confounded by the both-sides union grow_board reads and must be '
        'split into union/own-side before it may be selected.',
        'STEP 1  F1: on a one-face basis, a currency with far_over_near >= '
        'F1_DOUBLE_MAX on any board is REJECTED for that basis.',
        'STEP 2  F2: a currency with 0 flips and 0 moves on every decision '
        'basis is INDISTINGUISHABLE from `none` on this corpus.',
        'STEP 3  per decision basis, among currencies neither rejected nor '
        'inert: `tht` is selected on default_reason. `courtyard` displaces it '
        'only if it survives F1 on that basis AND flips strictly more boards. '
        'If only `none` survives, that basis keeps `none` and the '
        'not_modelled disclosure becomes basis-specific.',
        'STEP 4  if the robustness arm reports NOT ROBUST, a selection may '
        'not be taken from flips; it falls back to default_currency on '
        'default_reason alone, and the verdict says so.',
    ],
}


# --- the charging loop, parametrised by currency ----------------------------

def _board_geometry(pcb, path):
    """Per-part rows: the near box grow_board charges, and the far box.

    Deliberately one pass that yields BOTH boxes, so no arm can differ from
    another by geometry source rather than by currency.
    """
    cy = extract_courtyard_bboxes(path) or {}
    lb = L.part_local_bounds(pcb, path)
    bb = pcb.board_info.board_bounds
    x0, y0, x1, y1 = bb
    outline_area = (x1 - x0) * (y1 - y0)
    rows, containers, from_courtyard, from_pads, no_geometry = [], [], 0, 0, 0
    # `sorted(...)` matches options.py:203 so float accumulation order matches.
    for ref, fp in sorted(pcb.footprints.items()):
        box = cy.get(ref)
        if box:
            from_courtyard += 1
        elif fp.pads:
            box = compute_footprint_bbox_local(fp)
            from_pads += 1
        else:
            no_geometry += 1
            continue
        gx0, gy0, gx1, gy1 = L.rotate_local_bounds(*box, fp.rotation or 0.0)
        w, h = gx1 - gx0, gy1 - gy0
        if (outline_area > 0 and (w * h) >= L.CONTAINER_RATIO * outline_area
                and O.hosts_the_design(ref, gx0, gy0, gx1, gy1, fp,
                                       pcb.footprints)):
            containers.append((round(w * h, 2), ref))
            continue
        own = 'B.Cu' if L.footprint_side(fp) == 'B' else 'F.Cu'
        far = 'F.Cu' if own == 'B.Cu' else 'B.Cu'
        near = (w + CLEARANCE) * (h + CLEARANCE)
        # The far box, through the PUBLIC reader. `.tht_local` only -- `.local`
        # is NOT grow_board's near box (it uses courtyard_for_side rather than
        # the union, and keeps zero-pad parts as `synthetic` where grow_board
        # drops them), so swapping it wholesale would make every arm differ by
        # geometry source. NC1 would catch that; this comment is so nobody has
        # to learn it from a refusal.
        has_tht = L.footprint_has_through_pads(fp)
        tw = th = 0.0
        rec = lb.get(ref)
        tloc = getattr(rec, 'tht_local', None) if rec is not None else None
        if has_tht and tloc is not None:
            tx0, ty0, tx1, ty1 = L.rotate_local_bounds(*tloc, fp.rotation or 0.0)
            tw, th = tx1 - tx0, ty1 - ty0
        rows.append({'ref': ref, 'own': own, 'far': far, 'near': near,
                     'has_tht': has_tht, 'tht_w': tw, 'tht_h': th,
                     'tht_box': (has_tht and tloc is not None)})
    return {'rows': rows, 'containers': containers,
            'from_courtyard': from_courtyard, 'from_pads': from_pads,
            'no_geometry': no_geometry, 'outline_area': outline_area,
            'bounds': bb}


def _per_side(geom, currency, *, bare_far=False):
    """The POPULATION and OBSTRUCTION dicts under one currency.

    `pop` is each part once on its own face -- what `options.py:245-246` has
    always built, identical under every currency. `per` adds the far-face
    charge this currency says a drilled part presents. They are returned
    separately because the engine keeps them separately (#878), and a control
    that compared one against the other would report a mismatch on every board
    carrying a through-hole part.
    """
    per = {'F.Cu': 0.0, 'B.Cu': 0.0}
    pop = {'F.Cu': 0.0, 'B.Cu': 0.0}
    near_total = far_total = 0.0
    for r in geom['rows']:
        per[r['own']] += r['near']
        pop[r['own']] += r['near']
        near_total += r['near']
        if currency == 'none' or not r['has_tht']:
            continue
        if currency == 'courtyard':
            # THT-GATED, exactly like check_pockets.courtyard_cover, whose
            # `_side_key` is GradedPart.sides == sides_occupied(side, has_tht).
            # Charging an SMD part's courtyard to the far face would not be
            # this currency; it would be a bug that made this the biggest arm
            # for the wrong reason.
            add = r['near']
        elif currency == 'tht':
            if not r['tht_box']:
                continue
            add = (r['tht_w'] * r['tht_h'] if bare_far else
                   (r['tht_w'] + CLEARANCE) * (r['tht_h'] + CLEARANCE))
        else:
            raise ValueError(currency)
        per[r['far']] += add
        far_total += add
    return per, pop, near_total, far_total


def _usable(bounds, edge):
    x0, y0, x1, y1 = bounds
    return (max(0.0, (x1 - x0) - 2 * edge) * max(0.0, (y1 - y0) - 2 * edge))


def _arm(geom, currency, basis, *, edge=BOARD_EDGE_CLEARANCE, bare_far=False):
    per, pop, near_total, far_total = _per_side(geom, currency,
                                                bare_far=bare_far)
    usable = _usable(geom['bounds'], edge)
    busiest = max(per.values()) if per else 0.0
    one_face = basis != 'busiest'
    charged = sum(per.values()) if one_face else busiest
    util = (charged / usable) if usable > 0 else float('inf')
    binding = max(per, key=lambda k: per[k])
    return {
        'charged_area_mm2': round(charged, 2),
        'utilisation': round(util, 4),
        'fits_by_area': charged <= usable,
        # The engine's two dicts, kept apart for the same reason it keeps them
        # apart: one is the population, one is the obstruction.
        'part_area_by_side_mm2': {k: round(v, 2) for k, v in sorted(pop.items())},
        'obstructed_area_by_side_mm2': {k: round(v, 2)
                                        for k, v in sorted(per.items())},
        'busiest_side_area_mm2': round(busiest, 2),
        'near_charge_mm2': round(near_total, 2),
        'far_charge_mm2': round(far_total, 2),
        'far_over_near': round(far_total / near_total, 4) if near_total else 0.0,
        # "Report WHICH term binds": on a board whose back face is barely
        # populated, a far-face charge can make the UNBUILT face the binding
        # one. That has to be visible, not inferable.
        'binding_side': binding,
        'binding_rule': 'sum' if one_face else 'max',
        'usable_area_mm2': round(usable, 2),
    }


# --- the negative controls ---------------------------------------------------

#: The currency the shipping engine implements. NC1 compares
#: `options.grow_board` against THIS, not against a fixed `none`.
#:
#: Before #878 it was `none`, and NC1 read "the harness reproduces the engine".
#: After, it reads "the harness and the engine agree about which currency
#: SHIPS" -- strictly more, and still a proof the harness is not measuring
#: itself, because the harness computes every arm without consulting
#: `grow_board` at all.
ENGINE_CURRENCY = 'tht'


def _engine_shaped_arm(geom, basis):
    """The arm the ENGINE actually computes, which is not one of the study arms.

    The distinction is the whole of #878's post-hoc finding, so it is modelled
    rather than asserted. The engine charges the far face into `obstructed`
    ALWAYS -- the dict does not depend on the basis -- and then takes
    `max(obstructed)` on the busiest basis but `sum(per_side)` on the one-face
    one, because that sum is defined as each part exactly once.

    The study arms deliberately do the other thing on the one-face basis
    (`sum(obstructed)`), which is what measured the double count. So a control
    comparing the engine to a study arm would report a mismatch on every board
    with a through-hole part -- as it did, before this function existed.
    """
    a = _arm(geom, ENGINE_CURRENCY, basis)
    if basis != 'busiest':
        pop = _arm(geom, 'none', basis)
        a = dict(a)
        for k in ('charged_area_mm2', 'utilisation', 'fits_by_area',
                  'binding_side'):
            a[k] = pop[k]
    return a


#: The nine fields NC1 compares. Chosen to pin the CHAIN, not just the answer:
#: each one fails for a different reason.
NC1_FIELDS = ('charged_area_mm2', 'utilisation', 'fits_by_area',
              'busiest_side_area_mm2', 'part_area_by_side_mm2',
              'obstructed_area_by_side_mm2',
              'containers_excluded', 'extent_from_courtyard',
              'extent_from_pad_bbox')


def _control_row(pcb, path, geom, basis, declared):
    """The arm the engine implements (`ENGINE_ARM`) against the real engine."""
    eng = O.grow_board(pcb, path, clearance=CLEARANCE,
                       board_edge_clearance=BOARD_EDGE_CLEARANCE,
                       assembly_sides=declared)
    if not eng.get('ran'):
        return {'basis': basis, 'skipped': eng.get('reason', 'did not run')}
    m = eng['measured']
    mine = _engine_shaped_arm(geom, basis)
    mine_extra = {
        'containers_excluded': [r for _a, r in sorted(geom['containers'],
                                                      reverse=True)],
        'extent_from_courtyard': geom['from_courtyard'],
        'extent_from_pad_bbox': geom['from_pads'],
    }
    got = dict(mine)
    got.update(mine_extra)
    bad = []
    for f in NC1_FIELDS:
        a, b = m.get(f, eng.get(f)), got.get(f)
        if f == 'fits_by_area':
            a = eng.get('fits_by_area')
        if a != b:
            bad.append({'field': f, 'engine': a, 'harness': b})
    return {'basis': basis, 'engine_charged': m.get('charged_area_mm2'),
            'harness_charged': got['charged_area_mm2'],
            'engine_util': m.get('utilisation'),
            'harness_util': got['utilisation'],
            'engine_fits': eng.get('fits_by_area'),
            'harness_fits': got['fits_by_area'], 'mismatches': bad}


# --- per board ---------------------------------------------------------------

def measure_board(path):
    pcb = parse_kicad_pcb(path)
    if pcb.board_info.board_bounds is None:
        return None
    geom = _board_geometry(pcb, path)
    cen = L.assembly_census(pcb)
    sides = cen['sides']
    pad_bearing = cen['pad_bearing']
    observed = sides if sides in ('F', 'B') else None
    majority = 'B' if pad_bearing.get('B', 0) > pad_bearing.get('F', 0) else 'F'
    tht_obstruct = sum(1 for r in geom['rows'] if r['has_tht'])

    arms = {}
    for basis in BASES:
        if basis == 'one_face_observed' and observed is None:
            arms[basis] = None
            continue
        arms[basis] = {c: _arm(geom, c, basis) for c in CURRENCIES}
        for c in CURRENCIES:
            arms[basis][c]['far_charge_bare_mm2'] = _arm(
                geom, c, basis, bare_far=True)['far_charge_mm2']
            arms[basis][c]['fits_at_edge_055'] = _arm(
                geom, c, basis, edge=EDGE_SENSITIVITY)['fits_by_area']

    # The union confound: how many footprints draw a courtyard on both sides,
    # and on how many do the two boxes actually DIFFER. grow_board reads the
    # union (parser.py:198-214) where legality reads the per-side view
    # (parser.py:217-224); the arm is only clean while `differ` is 0.
    try:
        cs = extract_courtyard_sides(path) or {}
    except Exception:                                        # noqa: BLE001
        cs = {}
    drawn = sum(1 for d in cs.values() if len(d) > 1)
    differ = sum(1 for d in cs.values()
                 if len(d) > 1 and len(set(map(tuple, d.values()))) > 1)

    controls = [_control_row(pcb, path, geom, 'busiest', None)]
    if observed is not None:
        controls.append(_control_row(pcb, path, geom,
                                     'one_face_observed', observed))
    return {
        'census_sides': sides,
        'pad_bearing': pad_bearing,
        'blocks': cen['blocks'],
        'through_hole_plated': cen['through_hole'],
        'tht_obstructing': tht_obstruct,
        'parts_charged': len(geom['rows']),
        'declared_face': observed or majority,
        'declared_is_hypothetical': observed is None,
        'outline_area_mm2': round(geom['outline_area'], 2),
        'usable_area_mm2': round(_usable(geom['bounds'],
                                         BOARD_EDGE_CLEARANCE), 2),
        'containers_excluded': [r for _a, r in sorted(geom['containers'],
                                                      reverse=True)],
        'courtyard_sides_drawn': drawn,
        'courtyard_sides_differ': differ,
        'arms': arms,
        'control': controls,
    }


def boards(only=None):
    tracked = run_utils.corpus_boards()
    if not tracked:
        return None
    paths = {os.path.splitext(os.path.basename(p))[0]: p for p in tracked}
    if only:
        paths = {k: v for k, v in paths.items() if k in set(only)}
    return paths


# --- the rule, applied mechanically -----------------------------------------

def apply_rule(doc):
    """The pre-registered rule. Pure, so the gate can re-run it."""
    pre = doc['preregistration']
    out = {'selected': {}, 'bound_by': {}, 'f1_rejected': {}, 'f2_inert': [],
           'why': []}

    nc1 = sum(len(r['mismatches']) for b in doc['boards'].values()
              for r in b['control'] if 'mismatches' in r)
    nc2 = doc['control']['nc2_mismatches']
    if nc1 or nc2:
        out['why'].append('STEP 0: a negative control failed; decides nothing')
        return out
    confounded = any(b['courtyard_sides_differ']
                     for b in doc['boards'].values())
    if confounded:
        out['why'].append('STEP 0b: courtyard_sides_differ > 0; the '
                          '`courtyard` arm is confounded and unavailable')

    hl = doc['headline']
    for basis in pre['decision_bases']:
        rejected = []
        if basis.startswith('one_face'):
            for c in ('tht', 'courtyard'):
                worst = max((v for _b, v in hl['double'].get(
                    '%s.%s' % (basis, c), [])), default=0.0)
                if worst >= pre['F1_DOUBLE_MAX']:
                    rejected.append(c)
        if confounded and 'courtyard' not in rejected:
            rejected.append('courtyard')
        out['f1_rejected'][basis] = rejected

        alive = []
        for c in ('tht', 'courtyard'):
            if c in rejected:
                continue
            flips = len(hl['flips'].get('%s.%s' % (basis, c), []))
            moves = hl['moves'].get('%s.%s' % (basis, c), {}).get('n', 0)
            if flips == 0 and moves == 0:
                continue
            alive.append(c)

        robust = not hl['robustness']['not_robust']
        if not alive:
            out['selected'][basis] = 'none'
            out['bound_by'][basis] = (
                'every currency rejected or inert on this basis')
        elif not robust:
            out['selected'][basis] = pre['default_currency']
            out['bound_by'][basis] = (
                'STEP 4: not robust, so default_reason alone')
        else:
            sel = pre['default_currency'] if pre['default_currency'] in alive \
                else alive[0]
            if 'courtyard' in alive and 'tht' in alive:
                if (len(hl['flips'].get('%s.courtyard' % basis, []))
                        > len(hl['flips'].get('%s.tht' % basis, []))):
                    sel = 'courtyard'
            out['selected'][basis] = sel
            out['bound_by'][basis] = (
                'default_reason (legality.rect_on)' if sel == 'tht'
                else 'STEP 3: strictly more flips than tht')

    # Inertness is a corpus-wide statement, reported apart from selection.
    for c in ('tht', 'courtyard'):
        if all(len(hl['flips'].get('%s.%s' % (b, c), [])) == 0
               and hl['moves'].get('%s.%s' % (b, c), {}).get('n', 0) == 0
               for b in pre['decision_bases']):
            out['f2_inert'].append(c)
    return out


#: POST-HOC, and labelled so, because it was reasoned AFTER the numbers were
#: visible and is therefore not part of what `PREREGISTRATION` notarised.
#:
#: F1 rejects a currency on a one-face basis at a THRESHOLD (far/near >= 0.9),
#: and on this corpus it fires for `courtyard` (flat_hierarchy 1.000) and not
#: for `tht` (worst 0.5866). But the mechanism F1 is a proxy for is not a
#: threshold at all, it is structural: under a one-face policy `charged` is
#: `sum(per_side)`, which is defined as EACH PART EXACTLY ONCE -- the demand on
#: the single face the fab populates. A through-hole part's leads come out on
#: the face nobody populates, so they compete with nothing there. ANY far-face
#: charge entering that sum is the same area counted twice, `tht` included.
#:
#: So the implementation charges the far face into `busiest` ONLY, which is
#: strictly more conservative than the rule selected. `one_face_charged_once`
#: is the check that says so in numbers rather than in prose: it is True
#: exactly when a currency leaves the one-face sum equal to `part_area_mm2`.
#: Recording it here rather than editing the pre-registration is the point --
#: a threshold moved after seeing the answers is how a measurement stops
#: meaning anything.
POST_HOC = {
    'finding': ('no far-face charge belongs in the one-face SUM; the far face '
                'is charged into `busiest` only'),
    'why': ('sum(per_side) is each part exactly once, the demand on the one '
            'populated face. Leads on the unpopulated face compete with '
            'nothing, so any far charge there is the same area twice.'),
    'generalises': ('F1, which is a threshold on the same mechanism and fires '
                    'only for `courtyard` on this corpus'),
    'status': ('POST-HOC -- reasoned after the numbers were visible, NOT part '
               'of PREREGISTRATION, and it only makes the change more '
               'conservative than the rule required'),
}


def one_face_charged_once(rows):
    """Per currency: does the one-face sum stay `part_area_mm2` (each part once)?

    True only for `none`. This is the POST_HOC finding as a measurement.
    """
    out = {}
    for c in CURRENCIES:
        offenders = []
        for name, d in rows:
            arm = d['arms'].get('one_face_observed')
            if not arm:
                continue
            once = arm['none']['charged_area_mm2']
            got = arm[c]['charged_area_mm2']
            if abs(got - once) > 0.011:
                offenders.append([name, once, got])
        out[c] = {'charged_once': not offenders,
                  'boards_double_charged': sorted(offenders,
                                                  key=lambda r: r[1] - r[2])}
    return out


def headline(rows):
    flips, moves, double = {}, {}, {}
    for basis in BASES:
        for c in ('tht', 'courtyard'):
            key = '%s.%s' % (basis, c)
            f, mv, dd = [], [], []
            for name, d in rows:
                arm = (d['arms'].get(basis) or {})
                if not arm:
                    continue
                base, cur = arm['none'], arm[c]
                if base['fits_by_area'] != cur['fits_by_area']:
                    f.append([name, base['utilisation'], cur['utilisation'],
                              d['usable_area_mm2']])
                delta = cur['utilisation'] - base['utilisation']
                if abs(delta) >= PREREGISTRATION['MOVE_EPS']:
                    mv.append([name, round(delta, 4)])
                if cur['far_over_near']:
                    dd.append([name, cur['far_over_near']])
            flips[key] = f
            deltas = [d for _n, d in mv]
            moves[key] = {'n': len(mv), 'max': max(deltas, default=0.0),
                          'median': (round(statistics.median(deltas), 4)
                                     if deltas else 0.0),
                          'boards': sorted(mv, key=lambda r: -abs(r[1]))}
            double[key] = sorted(dd, key=lambda r: -r[1])

    # H5: the same flip sets under two named perturbations.
    not_robust = []
    for basis in DECISION_BASES:
        for c in ('tht', 'courtyard'):
            key = '%s.%s' % (basis, c)
            names = {r[0] for r in flips[key]}
            edge = set()
            for name, d in rows:
                arm = (d['arms'].get(basis) or {})
                if not arm:
                    continue
                if arm['none']['fits_at_edge_055'] != arm[c]['fits_at_edge_055']:
                    edge.add(name)
            if edge != names:
                not_robust.append({'arm': key, 'perturbation': 'edge 0.55',
                                   'primary': sorted(names),
                                   'perturbed': sorted(edge)})
    return {'flips': flips, 'moves': moves, 'double': double,
            'robustness': {'not_robust': not_robust,
                           'unmeasured': ['per-board clearance from a sibling '
                                          '.kicad_pro (#441): most tracked '
                                          'boards carry none']}}


# --- tables ------------------------------------------------------------------

def table_control(doc, rows):
    print('\n== control: the engine-shaped arm against the engine, %d fields =='
          % len(NC1_FIELDS))
    n = bad = 0
    for name, d in rows:
        for r in d['control']:
            if 'mismatches' not in r:
                continue
            n += 1
            bad += len(r['mismatches'])
            if r['mismatches']:
                print('  %-30s %-18s MISMATCH %s'
                      % (name, r['basis'], r['mismatches'][:2]))
    print('NC1: %d (board, basis) rows, %d fields each, %d mismatch(es).'
          % (n, len(NC1_FIELDS), bad))
    c = doc['control']
    print('NC2: %d board(s) with no drilled pad; %d differ across currencies.'
          % (len(c['nc2_boards']), len(c['nc2_mismatches'])))
    print('confound: %d footprint(s) draw both courtyards, %d differ.'
          % (doc['confound']['courtyard_sides_drawn'],
             doc['confound']['courtyard_sides_differ']))


def table_census(doc, rows):
    print('\n== census: the two predicates, side by side ==')
    print('%-30s %5s %5s %6s %6s %6s' % ('board', 'sides', 'parts',
                                         'THTobs', 'THTpl', 'B.pads'))
    for name, d in rows:
        print('%-30s %5s %5d %6d %6d %6d'
              % (name, d['census_sides'], d['parts_charged'],
                 d['tht_obstructing'], d['through_hole_plated'],
                 d['pad_bearing'].get('B', 0)))
    print('THTobs = footprint_has_through_pads (drill>0), the OBSTRUCTION rule '
          'this fix uses.')
    print('THTpl  = assembly_census (pad_is_plated_through), the ASSEMBLY rule '
          '#878 counted with.')


def table_charge(doc, rows):
    for basis in BASES:
        pop = [(n, d) for n, d in rows if d['arms'].get(basis)]
        note = ''
        if basis == 'one_face_declared':
            note = '  [REPORTED ONLY -- not decision-bearing]'
        print('\n== charge, basis %s (%d boards)%s ==' % (basis, len(pop), note))
        print('%-30s %5s | %8s %4s | %8s %4s | %8s %4s | %8s'
              % ('board', 'THT', 'none', 'fit', 'tht', 'fit',
                 'crtyd', 'fit', 'usable'))
        for name, d in pop:
            a = d['arms'][basis]
            flag = ' *' if a['none']['binding_side'] != a['tht']['binding_side'] \
                else ''
            print('%-30s %5d | %8.4f %4s | %8.4f %4s | %8.4f %4s | %8.1f%s'
                  % (name, d['tht_obstructing'],
                     a['none']['utilisation'],
                     'Y' if a['none']['fits_by_area'] else 'N',
                     a['tht']['utilisation'],
                     'Y' if a['tht']['fits_by_area'] else 'N',
                     a['courtyard']['utilisation'],
                     'Y' if a['courtyard']['fits_by_area'] else 'N',
                     d['usable_area_mm2'], flag))
        print('  * = the binding side moved between `none` and `tht`.')


def table_headline(doc, rows):
    hl = doc['headline']
    print('\n== headline ==')
    for basis in BASES:
        for c in ('tht', 'courtyard'):
            k = '%s.%s' % (basis, c)
            f, m = hl['flips'][k], hl['moves'][k]
            tag = '' if basis in DECISION_BASES else '  [reported only]'
            print('  %-28s flips=%-2d moves=%-2d max|d util|=%.4f%s'
                  % (k, len(f), m['n'], abs(m['max']), tag))
            for row in f:
                print('        FLIP %-26s util %.4f -> %.4f (usable %.1f)'
                      % (row[0], row[1], row[2], row[3]))
    print('\n  double-count (far/near) on the one-face bases, worst first:')
    for basis in ('one_face_observed', 'one_face_declared'):
        for c in ('tht', 'courtyard'):
            dd = hl['double'].get('%s.%s' % (basis, c), [])[:3]
            if dd:
                print('    %-28s %s' % ('%s.%s' % (basis, c), dd))
    ph = doc['post_hoc']['one_face_charged_once']
    print('\n  POST-HOC (not pre-registered): the one-face sum is each '
          'part ONCE only under:')
    print('    %s' % [c for c in CURRENCIES if ph[c]['charged_once']])
    for c in CURRENCIES:
        bad = ph[c]['boards_double_charged']
        if bad:
            print('    %-10s double-charges %d board(s), worst %s'
                  % (c, len(bad), bad[0]))
    nr = hl['robustness']['not_robust']
    print('\n  robustness: %s' % ('ROBUST' if not nr else 'NOT ROBUST'))
    for r in nr:
        print('    %s under %s: %s vs %s'
              % (r['arm'], r['perturbation'], r['primary'], r['perturbed']))


def table_verdict(doc, rows):
    v = doc['verdict']
    print('\n== verdict (the pre-registered rule, applied) ==')
    for r in doc['preregistration']['rules']:
        print('  %s' % r)
    print()
    for basis in DECISION_BASES:
        print('  %-20s -> %-10s  (%s)'
              % (basis, v['selected'].get(basis, '?'),
                 v['bound_by'].get(basis, '')))
    if v['f1_rejected']:
        print('  F1 rejected: %s' % v['f1_rejected'])
    if v['f2_inert']:
        print('  F2 inert (indistinguishable from `none`): %s' % v['f2_inert'])
    for w in v['why']:
        print('  %s' % w)


TABLES = {'control': table_control, 'census': table_census,
          'charge': table_charge, 'headline': table_headline,
          'verdict': table_verdict}


# --- assembly ----------------------------------------------------------------

def build(paths):
    rows, failed = [], []
    for name, path in sorted(paths.items()):
        try:
            d = measure_board(path)
        except Exception as exc:                             # noqa: BLE001
            failed.append(name)
            print('  %-30s FAILED: %r' % (name, exc))
            continue
        if d is None:
            failed.append(name)
            print('  %-30s FAILED: no board outline' % name)
            continue
        rows.append((name, d))

    nc2_boards = [n for n, d in rows if d['tht_obstructing'] == 0]
    nc2_bad = []
    for n, d in rows:
        if d['tht_obstructing']:
            continue
        for basis in BASES:
            arm = d['arms'].get(basis)
            if not arm:
                continue
            ref = arm['none']['charged_area_mm2']
            for c in ('tht', 'courtyard'):
                if arm[c]['charged_area_mm2'] != ref:
                    nc2_bad.append({'board': n, 'basis': basis, 'currency': c,
                                    'none': ref,
                                    'got': arm[c]['charged_area_mm2']})
    sha = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                         capture_output=True, text=True).stdout.strip()
    doc = {
        'engine_sha': sha,
        'preregistration': PREREGISTRATION,
        'basis': {'clearance': CLEARANCE,
                  'board_edge_clearance': BOARD_EDGE_CLEARANCE,
                  'board_edge_clearance_sensitivity': EDGE_SENSITIVITY,
                  'corpus': 'run_utils.corpus_boards() -- the git-TRACKED set',
                  'n_boards': len(rows)},
        'control': {'nc2_boards': nc2_boards, 'nc2_mismatches': nc2_bad},
        'confound': {
            'courtyard_sides_drawn': sum(d['courtyard_sides_drawn']
                                         for _n, d in rows),
            'courtyard_sides_differ': sum(d['courtyard_sides_differ']
                                          for _n, d in rows)},
        'boards': {n: d for n, d in rows},
    }
    doc['headline'] = headline(rows)
    doc['post_hoc'] = dict(POST_HOC)
    doc['post_hoc']['one_face_charged_once'] = one_face_charged_once(rows)
    doc['verdict'] = apply_rule(doc)
    return doc, rows, failed


DIFF_KEYS = ('utilisation', 'fits_by_area', 'charged_area_mm2',
             'far_charge_mm2', 'binding_side')

#: What a COMMITTED run keeps per (board, basis, currency). The full in-process
#: record carries fourteen fields per arm and the eight-field control row for
#: every board; dumping all of it is 126 KB of numbers the gate re-derives in
#: 17 seconds anyway, and this repo has already had one 276 KB regenerable
#: table asked back out of a PR. So the artifact keeps the cells the verdict
#: actually rests on, and `--out-full` still exists for a debugging dump.
KEEP_PER_ARM = ('utilisation', 'fits_by_area', 'charged_area_mm2',
                'far_charge_mm2', 'far_over_near', 'binding_side')
KEEP_PER_BOARD = ('census_sides', 'parts_charged', 'tht_obstructing',
                  'through_hole_plated', 'declared_face',
                  'declared_is_hypothetical', 'usable_area_mm2',
                  'containers_excluded', 'courtyard_sides_drawn',
                  'courtyard_sides_differ')


def compact(doc):
    """The committed shape: the deciding cells, not the whole record."""
    out = {k: doc[k] for k in ('engine_sha', 'preregistration', 'basis',
                               'confound', 'headline', 'post_hoc',
                               'verdict')}
    out['control'] = {
        'nc1_rows': sum(1 for b in doc['boards'].values()
                        for r in b['control'] if 'mismatches' in r),
        'nc1_fields_per_row': len(NC1_FIELDS),
        'nc1_mismatches': [m for b in doc['boards'].values()
                           for r in b['control']
                           for m in r.get('mismatches', ())],
        'nc2_boards': doc['control']['nc2_boards'],
        'nc2_mismatches': doc['control']['nc2_mismatches'],
    }
    boards = {}
    for name, d in doc['boards'].items():
        rec = {k: d[k] for k in KEEP_PER_BOARD}
        rec['pad_bearing_B'] = d['pad_bearing'].get('B', 0)
        rec['arms'] = {
            basis: (None if d['arms'].get(basis) is None else
                    {c: {k: d['arms'][basis][c][k] for k in KEEP_PER_ARM}
                     for c in CURRENCIES})
            for basis in BASES}
        boards[name] = rec
    out['boards'] = boards
    return out


def _diff(before, after):
    b = json.load(open(run_utils.evidence(before, 'the BEFORE run'),
                       encoding='utf-8'))
    a = json.load(open(run_utils.evidence(after, 'the AFTER run'),
                       encoding='utf-8'))
    moved = 0
    for board in sorted(set(b['boards']) | set(a['boards'])):
        if board not in b['boards'] or board not in a['boards']:
            print('  %-30s %s' % (board, 'ADDED' if board in a['boards']
                                  else 'REMOVED'))
            moved += 1
            continue
        for basis in BASES:
            ba = b['boards'][board]['arms'].get(basis)
            aa = a['boards'][board]['arms'].get(basis)
            if (ba is None) != (aa is None):
                print('  %-30s %-20s basis appeared/vanished' % (board, basis))
                moved += 1
                continue
            if ba is None:
                continue
            for c in CURRENCIES:
                for k in DIFF_KEYS:
                    x, y = ba[c].get(k), aa[c].get(k)
                    if x != y:
                        print('  %-30s %-28s %r -> %r'
                              % (board, '%s.%s.%s' % (basis, c, k), x, y))
                        moved += 1
    for k in ('selected', 'bound_by'):
        if b['verdict'].get(k) != a['verdict'].get(k):
            print('  VERDICT %s: %r -> %r'
                  % (k, b['verdict'].get(k), a['verdict'].get(k)))
            moved += 1
    print('\n%d value(s) moved' % moved)
    if b.get('engine_sha') != a.get('engine_sha'):
        print('engine %s -> %s' % (b.get('engine_sha'), a.get('engine_sha')))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--table', default='control,census,charge,headline,verdict',
                    help='comma-separated: %s' % ', '.join(TABLES))
    ap.add_argument('--boards', nargs='*', help='basenames to restrict to')
    ap.add_argument('--out', help='write the committed figures as json')
    ap.add_argument('--out-full', help='write the UNCOMPACTED record (a '
                                       'debugging dump, not for committing)')
    ap.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'))
    args = ap.parse_args()

    if args.diff:
        return _diff(*args.diff)

    paths = boards(args.boards)
    if paths is None:
        print('SKIP: git could not name the tracked corpus')
        return SKIP_EXIT
    if not paths:
        print('no tracked board matched %r' % (args.boards,))
        return 2

    doc, rows, failed = build(paths)

    # REFUSE rather than print a clean page over nothing.
    if failed or not rows:
        print('\nREFUSING to report: %d of %d board(s) did not measure (%s).'
              % (len(failed), len(failed) + len(rows),
                 ', '.join(failed[:6]) or 'none measured'))
        return 2

    wanted = [t.strip() for t in args.table.split(',') if t.strip()]
    for t in wanted:
        if t not in TABLES:
            print('unknown table %r; have %s' % (t, ', '.join(TABLES)))
            return 2

    nc1 = sum(len(r['mismatches']) for _n, d in rows for r in d['control']
              if 'mismatches' in r)
    nc2 = len(doc['control']['nc2_mismatches'])
    if 'control' in wanted or nc1 or nc2:
        table_control(doc, rows)
    if nc1 or nc2:
        print('\nREFUSING to report: the negative control did not reproduce '
              '(NC1 %d mismatch(es), NC2 %d). Every table below would be a '
              'page of numbers about a harness, not about the engine.'
              % (nc1, nc2))
        return 2

    for t in wanted:
        if t == 'control':
            continue
        TABLES[t](doc, rows)

    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump(compact(doc), f, indent=1, sort_keys=True)
        print('\nwrote %s' % args.out)
    if args.out_full:
        with open(args.out_full, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
        print('wrote %s (uncompacted)' % args.out_full)
    return 0


if __name__ == '__main__':
    sys.exit(main())
