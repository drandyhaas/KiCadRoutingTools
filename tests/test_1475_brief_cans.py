#!/usr/bin/env python3
"""#1475 step 3: a design brief's `cans[]` row compiles a shield can into the
rows the intent already grades -- a block per listed face, a keep-out over
the zone that lets in only the can's own parts, and the room under the lid.

Each section names the defect it is there to catch:

  1. a drawn can is not a stranger in its own keep-out (the can's ref in
     `allow`), and a stranger at the centre still is;
  2. a through-hole member listed on F is not refused by the B keep-out (every
     face's parts on every face's keep-out);
  3. one height rule drawn and undrawn: 2.0 mm under 2.5 mm of room fits both
     ways, 2.01 mm fits neither (the cover margin taken off the undrawn
     keep-out's limit), and the same at a 1.2 mm lid (0.7 / 0.71 mm); the
     refusal names the limit and the height, and the plan check place_simple
     runs first refuses the run;
  4. a member 0.3 mm past the zone is out (the block's tolerance is 0);
  5. a drawn can whose courtyard is a ring gives room under it; one whose
     courtyard covers the zone gives none and says why (withheld, read as
     drawn, never by its bounding box); nor does a zone that is not under
     the can, or a can whose outline cannot be read;
  6. the merge takes a can's parts out of an inferred block and says so,
     `either`-only cans included; the can's block inside that block's zone
     is no `intent_zone_overlap`, since the can's keep-out closes the shared
     area to the other block's parts (and without it, it is);
  7. drift: none when equal, the can's clause when its zone changes, when
     only a block changes (a part moved from `in` to `either`), and when an
     undrawn can's lid height changes;
  8. refusals, each by its reason;
  9. min_reader is the fence's reader, for every can row;
 10. place_simple, from a pile, over three seeds, on a netted board: with the
     row the `in` parts land in the zone, the strangers wired to the chip stay
     out of it and its wall, and an `either` part too tall for the lid stays
     out from under it -- and without the row the same seeds break each of
     the three, so the checks can fail.

Run: python3 tests/test_1475_brief_cans.py
"""
import copy
import json
import os
import sys
import tempfile
import uuid

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tests'))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))

import test_1462_declared_underside as t1462  # noqa: E402
from test_1462_declared_underside import _fp, check, failures, quiet  # noqa: E402
from placement import design_brief as db  # noqa: E402
from placement import floorplan as fp  # noqa: E402
from placement import heights  # noqa: E402
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 300
ZONE = [11, 11, 29, 29]


def _crtyd(x0, y0, x1, y1):
    return ('\t\t(fp_rect\n\t\t\t(start %g %g)\n\t\t\t(end %g %g)\n\t\t\t(stroke\n'
            '\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n\t\t\t(fill no)\n'
            '\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "%s")\n\t\t)\n'
            % (x0, y0, x1, y1, uuid.uuid4()))


def _tht(ref, x, y):
    """A 2-pin through-hole part, its leads on both faces."""
    s = _fp('Test:THT_2', ref, x, y, [], crt=(1.5, 1.0))
    pads = ''.join(f'\t\t(pad "{n}" thru_hole circle\n\t\t\t(at {px} 0)\n'
                   f'\t\t\t(size 1.2 1.2)\n\t\t\t(drill 0.7)\n'
                   f'\t\t\t(layers "*.Cu" "*.Mask")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n'
                   for n, px in (('1', -0.8), ('2', 0.8)))
    return s[:-3] + pads + '\t)\n'


def board(tmp, name, *, can='none', drawn=True, extra=''):
    """test_1462's 40 x 40 board (can SH1 at (20, 20) over C1, C2, U1), its
    courtyard 'none' / 'ring' (outer + inner outline, as KiCad's
    RF_Shielding footprints) / 'full'; `drawn=False` drops SH1; `extra`
    adds footprints."""
    s = open(t1462.write_can(tmp, 'SH1')).read()
    crt = {'none': '', 'full': _crtyd(-10.5, -10.5, 10.5, 10.5),
           'ring': _crtyd(-10.5, -10.5, 10.5, 10.5) + _crtyd(-8.5, -8.5, 8.5, 8.5)}[can]
    head = s.index('\t(footprint "Test:ShieldCan_20x20mm"')
    end = s.index('\t(footprint "Test:C_0603"')
    can_block = s[head:end]
    if crt:
        j = can_block.index('\t\t(fp_rect')
        can_block = can_block[:j] + crt + can_block[j:]
    s = s[:head] + (can_block if drawn else '') + s[end:]
    j = s.index('\t(gr_rect')
    s = s[:j] + extra + s[j:]
    path = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(s)
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, path)
    pcb.source_path = path
    return pcb, path


def brief(cans, hts=(('C1', 0.9), ('C2', 0.9), ('U1', 0.9))):
    return db.brief_from_dict({
        'schema': 1, 'kind': 'design-brief', 'cans': cans,
        'heights': [{'ref': r, 'height_mm': h, 'source': 'test'} for r, h in hts]})


def can_row(**kw):
    row = {'name': 'RF', 'ref': 'SH1', 'zone': list(ZONE), 'side': 'F',
           'interior_height_mm': 2.5, 'source': 'test datasheet',
           'members': {'F': {'in': ['C1', 'U1'], 'either': ['C2']}}}
    row.update(kw)
    return {k: v for k, v in row.items() if v is not None}


def intent_for(b, pcb=None, path='', emitted=None):
    frag, rep = db.compile_with_consequences(b, pcb, path)
    base = emitted or {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm'}
    return db.merge_into_intent(base, frag, rep), frag, rep


def grade(doc, pcb, path):
    it = fp.intent_from_dict(copy.deepcopy(doc), 't.json')
    r = quiet(fp.grade, it, pcb, path)
    return [(v.rule, v.ref, str(v.message)) for v in r.errors + r.warnings]


def by_rule(found, rule):
    return sorted({ref for r, ref, _m in found if r == rule})


E2E_NETS = ('GND', 'N1', 'N2', 'N3', 'N4')


def _netted(ref, x, y, pads, hw, hh, locked=False):
    lk = '(locked yes) ' if locked else ''
    s = (f'  (footprint "t:{ref[0]}" {lk}(layer "F.Cu") (at {x} {y})\n'
         f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
         f'    (fp_rect (start {-hw} {-hh}) (end {hw} {hh}) (stroke (width 0.05)'
         f' (type default)) (layer "F.CrtYd"))\n')
    for n, (px, py, net) in enumerate(pads, 1):
        s += (f'    (pad "{n}" smd rect (at {px} {py}) (size 0.8 0.8) (layers "F.Cu")'
              f' (net {E2E_NETS.index(net) + 1} "{net}"))\n')
    return s + '  )\n'


def netted_board(path):
    """A 40 x 40 board: connector J1 locked at the west edge, and a pile off
    the outline -- chip U1, its caps C1 and C2, and R9 and R8, which are wired
    to U1 and J1 but belong to no can."""
    two = lambda a, b: [(-0.8, 0, a), (0.8, 0, b)]  # noqa: E731
    parts = (_netted('U1', 52, 10, [(-1.5, 0, 'N1'), (-0.5, 0, 'N2'),
                                    (0.5, 0, 'N3'), (1.5, 0, 'N4')], 2.2, 2.2)
             + _netted('C1', 52, 16, two('N1', 'GND'), 1.5, 0.75)
             + _netted('C2', 56, 16, two('N2', 'GND'), 1.5, 0.75)
             + _netted('R9', 52, 22, two('N3', 'N4'), 1.5, 0.75)
             + _netted('R8', 56, 22, two('N4', 'GND'), 1.5, 0.75)
             + _netted('J1', 3, 20, [(0, -1, 'GND'), (0, 1, 'N4')], 1.5, 2.5,
                       locked=True))
    nets = ''.join(f'  (net {i + 1} "{n}")\n' for i, n in enumerate(E2E_NETS))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n' + nets + '  (gr_rect (start 0 0) (end 40 40)'
                 ' (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
                 + parts + ')\n')
    return path


def pads_in(pcb, ref, box, every):
    hits = [box[0] <= p.global_x <= box[2] and box[1] <= p.global_y <= box[3]
            for p in pcb.footprints[ref].pads]
    return all(hits) if every else any(hits)


def main():
    with tempfile.TemporaryDirectory(prefix='t1475_') as tmp:
        return _main(tmp)


def _main(tmp):

    print('1. the drawn can is not a stranger in its own keep-out')
    pcb, path = board(tmp, 'b1', extra=_fp('Test:R_0603', 'R9', 20, 20,
                                           [('1', -0.8, 0, 0.9, 0.95),
                                            ('2', 0.8, 0, 0.9, 0.95)],
                                           crt=(1.5, 0.75)))
    doc, _f, _r = intent_for(brief([can_row()]), pcb, path)
    found = grade(doc, pcb, path)
    ko = by_rule(found, 'keepout')
    check('SH1 not inside its own keep-out', 'SH1' not in ko, str(ko))
    check('the stranger R9 at the centre is', ko == ['R9'], str(ko))
    # `either` may stand inside: no rule at all, the block's included
    c2 = [f for f in found if f[1] == 'C2']
    check('the either part C2 inside the can has no finding', c2 == [], str(c2))

    print('2. a through-hole member is not refused on its other face')
    hts = (('C1', 0.9), ('C2', 0.9), ('U1', 0.9), ('R20', 1.0))
    pcb, path = board(tmp, 'b2', extra=_tht('R20', 20, 18))
    row = can_row(members={'F': {'in': ['C1', 'U1', 'R20']},
                           'B': {'either': ['C2']}})
    doc, _f, _r = intent_for(brief([row], hts), pcb, path)
    names = {k['name']: k for k in doc['keepouts']}
    check('a keep-out per listed face', {'can:RF:F', 'can:RF:B'} <= set(names),
          str(sorted(names)))
    ko = by_rule(grade(doc, pcb, path), 'keepout')
    check('R20 not refused by the B keep-out', 'R20' not in ko, str(ko))
    hh = heights.with_declared(heights.EMPTY,
                               {'covers': [], 'heights': doc['heights']}, pcb, path)
    p = t1462.pair(hh, pcb, 'SH1', 'R20')
    check(f"R20 fits under SH1 too (pair {p})", p == 0.0)
    # under a can not drawn yet its far face's height is not known, so the
    # height keep-out refuses it, and says why
    pcb, path = board(tmp, 'b2u', drawn=False, extra=_tht('R20', 20, 18))
    doc, _f, _r = intent_for(brief([can_row(ref=None, wall_mm=0.8, members={
        'F': {'in': ['C1', 'U1', 'R20']}})], hts), pcb, path)
    msg = [m for r, ref, m in grade(doc, pcb, path) if r == 'keepout' and ref == 'R20']
    check('undrawn: R20 refused, as standing on both faces',
          len(msg) == 1 and 'stands on both faces' in msg[0], str(msg))
    # a stranger on B under the zone: refused when the row lists B, not
    # when it lists F alone (a face not listed claims nothing)
    r7 = _fp('Test:R_0603', 'R7', 20, 24, [('1', -0.8, 0, 0.9, 0.95),
                                            ('2', 0.8, 0, 0.9, 0.95)],
             crt=(1.5, 0.75)).replace('F.Cu', 'B.Cu').replace('F.CrtYd', 'B.CrtYd')
    pcb, path = board(tmp, 'b2b', extra=r7)
    for faces, want in (({'F': {'in': ['C1', 'U1'], 'either': ['C2']},
                          'B': {'either': ['C9']}}, ['R7']),
                        ({'F': {'in': ['C1', 'U1'], 'either': ['C2']}}, [])):
        doc, _f, _r = intent_for(brief([can_row(members=faces)]), pcb, path)
        ko = by_rule(grade(doc, pcb, path), 'keepout')
        check(f"the B stranger R7 with faces {sorted(faces)}: "
              f"{'refused' if want else 'not claimed'}", ko == want, str(ko))
    # an F can's lid limits only F: a 3 mm part on B under it is not refused
    pcb, path = board(tmp, 'b2bu', drawn=False, extra=r7)
    doc, _f, _r = intent_for(brief([can_row(ref=None, wall_mm=0.8)],
                                   (('C1', 0.9), ('C2', 0.9), ('U1', 0.9), ('R7', 3.0))),
                             pcb, path)
    ko = by_rule(grade(doc, pcb, path), 'keepout')
    check('undrawn F can: the 3 mm R7 on B is not under its lid', ko == [], str(ko))

    print('3. one height rule, drawn and undrawn, a low lid too')
    for lid, h, fits in ((2.5, 2.0, True), (2.5, 2.01, False),
                         (1.2, 0.7, True), (1.2, 0.71, False)):
        hts = (('C1', h), ('C2', 0.5), ('U1', 0.5))
        # undrawn: the can's fence and its height keep-out
        pcb, path = board(tmp, f'b3u{lid}_{h}', drawn=False)
        doc, frag, _r = intent_for(brief([can_row(ref=None, wall_mm=0.8,
                                                  interior_height_mm=lid)], hts),
                                   pcb, path)
        lim = next(k['max_height_mm'] for k in frag['keepouts']
                   if k['name'] == 'can:RF:height')
        found = grade(doc, pcb, path)
        flagged = 'C1' in by_rule(found, 'keepout')
        check(f"undrawn: C1 at {h} mm {'fits' if fits else 'refused'} "
              f"(limit {lim:g})", flagged != fits, str(by_rule(found, 'keepout')))
        if not fits:
            msg = next(m for r, ref, m in found if r == 'keepout' and ref == 'C1')
            check('the refusal names the limit and the height',
                  f'no taller than {lim:g}mm' in msg and f'C1 is {h:g}mm' in msg,
                  msg)
        # what place_simple stops on before placing anything (#1468)
        it = fp.intent_from_dict(copy.deepcopy(doc), 't.json')
        plan, _m = quiet(fp.plan_check, it, pcb, path)
        rules = sorted({v.rule for v in plan if v.severity == fp.ERROR})
        check(f"undrawn at {h}: the plan check "
              f"{'passes' if fits else 'refuses the run'} ({rules})",
              rules == ([] if fits else ['intent_zone_in_keepout']))
        # drawn: SH1 is a cover with the declared room
        pcb, path = board(tmp, f'b3d{lid}_{h}')
        doc, frag, _r = intent_for(brief([can_row(interior_height_mm=lid)], hts),
                                   pcb, path)
        hh = heights.with_declared(heights.EMPTY,
                                   {'covers': [], 'heights': doc['heights']},
                                   pcb, path)
        p = t1462.pair(hh, pcb, 'SH1', 'C1')
        check(f"drawn: C1 at {h} mm {'fits' if fits else 'refused'} "
              f"under SH1 (pair {p})",
              (p == 0.0) if fits else (p is not None and p > 0))
    # a member with no known height is refused, and the finding says so
    pcb, path = board(tmp, 'b3n', drawn=False)
    doc, _f, _r = intent_for(brief([can_row(ref=None, wall_mm=0.8)],
                                   (('C2', 0.5), ('U1', 0.5))), pcb, path)
    msg = [m for r, ref, m in grade(doc, pcb, path) if r == 'keepout' and ref == 'C1']
    check('undrawn: C1 of no known height refused, and named so',
          len(msg) == 1 and 'C1 has no known height' in msg[0], str(msg))

    print('4. the block holds its members to the zone, with no tolerance')
    pcb, path = board(tmp, 'b4')
    # C2 (24.25..25.75 pads, courtyard 23.5..26.5) 0.3 mm past x1 = 26.2
    row = can_row(zone=[11, 11, 26.2, 29],
                  members={'F': {'in': ['C1', 'U1', 'C2']}})
    doc, _f, _r = intent_for(brief([row]), pcb, path)
    blk = next(b for b in doc['blocks'] if b['name'] == 'can:RF:F')
    check('tolerance_mm 0.0', blk.get('tolerance_mm') == 0.0, str(blk))
    zc = by_rule(grade(doc, pcb, path), 'zone_containment')
    check('C2 0.3 mm past the zone is out', zc == ['C2'], str(zc))

    print('5. a drawn courtyard: a ring gives room, a full one does not')
    for can, room in (('ring', True), ('full', False), ('none', True)):
        pcb, path = board(tmp, f'b5{can}', can=can)
        doc, frag, rep = intent_for(brief([can_row()]), pcb, path)
        rows = [c for c in rep.get('consequences') or ()
                if c['id'] == 'cans[RF].interior_height_mm']
        compiled = any(h.get('ref') == 'SH1' for h in doc.get('heights') or ())
        check(f"{can}: room {'compiled' if room else 'withheld'}",
              compiled == room and len(rows) == 1
              and rows[0]['status'] == ('compiled' if room else 'withheld'),
              str(rows))
        if room:
            check(f"{can}: the unlocked SH1 is named",
                  'SH1 is not locked' in rows[0]['why'], rows[0]['why'])
        hh = heights.with_declared(heights.EMPTY,
                                   {'covers': [], 'heights': doc.get('heights') or []},
                                   pcb, path)
        p = t1462.pair(hh, pcb, 'SH1', 'U1')
        # withheld, SH1 is no cover at all (pair None): U1 inside its
        # courtyard is then a courtyard overlap, as KiCad grades it
        check(f"{can}: U1 under SH1 {'fits' if room else 'gets no room'} "
              f"(pair {p})", (p == 0.0) == room)
    pcb, path = board(tmp, 'b5w', can='full')
    _d, _f, rep = intent_for(brief([can_row()]), pcb, path)
    why = next(c['why'] for c in rep['consequences']
               if c['id'] == 'cans[RF].interior_height_mm')
    check('the withheld row says to draw the courtyard as a ring',
          'as a ring' in why, why)
    pcb, path = board(tmp, 'b5s')
    _d, _f, rep = intent_for(brief([can_row(side='B')]), pcb, path)
    why = next(c['why'] for c in rep['consequences']
               if c['id'] == 'cans[RF].interior_height_mm')
    check('a can declared on the wrong face is withheld', 'F face' in why, why)
    # SH1 spans 10..30: a zone moved 10 mm east is not under it
    for zone, room in (([11, 11, 29, 29], True), ([21, 11, 39, 29], False)):
        _d, _f, rep = intent_for(brief([can_row(zone=zone)]), pcb, path)
        row = next(c for c in rep['consequences']
                   if c['id'] == 'cans[RF].interior_height_mm')
        check(f"zone {zone}: {'under the can' if room else 'withheld, not under it'}",
              (row['status'] == 'compiled') == room
              and (room or 'past SH1' in row['why']), row['why'])
    import placement.body as body_mod
    real = body_mod.board_bodies

    def broken(*_a, **_k):
        raise ValueError('no outline')
    body_mod.board_bodies = broken
    try:
        _d, _f, rep = intent_for(brief([can_row()]), pcb, path)
    finally:
        body_mod.board_bodies = real
    row = next(c for c in rep['consequences']
               if c['id'] == 'cans[RF].interior_height_mm')
    check('an outline that cannot be read is withheld and named',
          row['status'] == 'withheld' and 'no outline' in row['why'], row['why'])

    print('6. the merge takes a can\'s parts out of an inferred block')
    emitted = {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
               'blocks': [{'name': 'sheetA', 'group': 'sheetA',
                           'refs': ['C1', 'C2', 'R5'], 'zone': [0, 0, 40, 40]},
                          {'name': 'sheetB', 'group': 'sheetB',
                           'refs': ['U1'], 'zone': [0, 0, 40, 40]}]}
    doc, _f, rep = intent_for(brief([can_row()]), None, '', emitted)
    blocks = {b['name']: b for b in doc['blocks']}
    check('sheetA keeps R5 and its zone, without its group',
          blocks.get('sheetA', {}).get('refs') == ['R5']
          and 'group' not in blocks['sheetA'] and blocks['sheetA'].get('zone'),
          str(blocks.get('sheetA')))
    check('sheetB, left with nothing, is dropped', 'sheetB' not in blocks)
    check('the can block is appended', 'can:RF:F' in blocks)
    check('each change is a contradiction',
          sum('under a can' in c for c in rep['contradictions']) == 2,
          str(rep['contradictions']))
    # the can's block lies inside sheetA's zone, and the can's keep-out shuts
    # sheetA's own part out of it: the shared area is not "either's"
    def overlaps(d):
        return [v.block for v in fp.validate_intent(
            fp.intent_from_dict(copy.deepcopy(d), 't.json'))
            if v.rule == 'intent_zone_overlap']
    check('no zone-overlap warning where the can\'s keep-out closes the area',
          overlaps(doc) == [], str(overlaps(doc)))
    bare = dict(doc, keepouts=[k for k in doc.get('keepouts') or ()
                               if not k['name'].startswith('can:')])
    check('without the keep-out the overlap is warned (the control)',
          overlaps(bare) != [], str(overlaps(bare)))
    # a can of `either` parts only compiles no block, and still takes them
    doc, _f, rep = intent_for(brief([can_row(members={'F': {'either': ['C1', 'C2']}})]),
                              None, '', copy.deepcopy(emitted))
    blocks = {b['name']: b for b in doc['blocks']}
    check('an either-only can takes its parts out too',
          blocks.get('sheetA', {}).get('refs') == ['R5'] and 'can:RF:F' not in blocks
          and blocks.get('sheetB', {}).get('refs') == ['U1'], str(blocks))

    print('7. drift')
    b0 = brief([can_row()])
    doc, frag, _r = intent_for(b0)
    check('none when the intent is the compiled brief',
          db.drift_pairs(doc, frag) == [], str(db.drift_pairs(doc, frag)))
    _d, frag2, _r = intent_for(brief([can_row(zone=[11, 11, 28, 29])]))
    ids = db.drifted_clause_ids(doc, frag2)
    check('a moved zone drifts as cans[RF]', ids == ['cans[RF]'], str(ids))
    _d, frag3, _r = intent_for(brief([can_row(
        members={'F': {'in': ['C1'], 'either': ['C2', 'U1']}})]))
    lines = db.drift_pairs(doc, frag3)
    check('U1 moved from in to either drifts on the block alone',
          lines and all(cid == 'cans[RF]' and 'block' in ln for cid, ln in lines),
          str(lines))
    und = dict(ref=None, wall_mm=0.8)
    doc_u, frag_u, _r = intent_for(brief([can_row(**und)]))
    _d, frag_u2, _r = intent_for(brief([can_row(interior_height_mm=3.0, **und)]))
    lines = db.drift_pairs(doc_u, frag_u2)
    check('an undrawn can\'s new lid height drifts on its height keep-out',
          db.drift_pairs(doc_u, frag_u) == [] and any(
              cid == 'cans[RF]' and 'max_height_mm' in ln for cid, ln in lines),
          str(lines))
    _d, frag_u3, _r = intent_for(brief([can_row(ref=None, wall_mm=1.2)]))
    lines = db.drift_pairs(doc_u, frag_u3)
    check('an undrawn can\'s new wall drifts on its block\'s fence_mm',
          any(cid == 'cans[RF]' and 'fence_mm' in ln for cid, ln in lines),
          str(lines))
    hand = copy.deepcopy(doc)
    next(b for b in hand['blocks'] if b['name'] == 'can:RF:F')['zone'] = [11, 11, 27, 29]
    lines = db.drift_pairs(hand, frag)
    check('a can block\'s zone edited by hand in the intent drifts',
          any(cid == 'cans[RF]' and 'zone' in ln for cid, ln in lines), str(lines))
    _d, frag4, _r = intent_for(brief([]))
    lines = db.drift_pairs(doc, frag4)
    check('a can the brief drops leaves its block, keep-outs and room as drift',
          {'block', 'keepout', 'heights'} <= {ln.split()[0].split('[')[0]
                                              for _c, ln in lines}, str(lines))

    print('8. refusals, by reason')
    for label, row, why in (
            ('no zone', can_row(zone=None), 'needs `zone`'),
            ('no source', can_row(source=None), 'needs `source`'),
            ('no height', can_row(interior_height_mm=None),
             'needs `interior_height_mm`'),
            ('height under the margin', can_row(interior_height_mm=0.4),
             'leaves no room'),
            ('no ref and no wall', can_row(ref=None), 'needs `ref`'),
            ('a wall on a drawn can', can_row(wall_mm=1.0), '`wall_mm` is for'),
            ('a glob member', can_row(members={'F': {'in': ['C*']}}), 'pattern'),
            ('the can as a member', can_row(members={'F': {'in': ['SH1']}}),
             'the can itself'),
            ('in and either', can_row(members={'F': {'in': ['C1'],
                                                     'either': ['C1']}}),
             'one claim per part'),
            ('an unknown key', can_row(height_mm=2), 'height_mm'),
            ('an unknown face', can_row(members={'X': {'in': ['C1']}}),
             "'F' or 'B'"),
            ('no members', can_row(members={}), 'needs `members`'),
            ('undrawn, no in on its face',
             can_row(ref=None, wall_mm=1, members={'F': {'either': ['C1']}}),
             'needs `in` parts')):
        try:
            brief([row])
            check(f"refused: {label}", False, 'accepted')
        except db.BriefError as exc:
            check(f"refused: {label}", why in str(exc), str(exc))
    for label, cans, hts, why in (
            ('a part in two cans',
             [can_row(), can_row(name='RF2', ref='SH2',
                                 members={'F': {'in': ['C1']}})], (),
             'one can per part'),
            ('the can also in heights[]', [can_row()], (('SH1', 3.0),),
             'also in heights[]'),
            ('two cans over one area of one face',
             [can_row(), can_row(name='RF2', ref='SH2', zone=[20, 20, 35, 35],
                                 members={'F': {'in': ['R5']}})], (),
             'overlaps cans[0] (RF) on face F')):
        try:
            brief(cans, hts)
            check(f"refused: {label}", False, 'accepted')
        except db.BriefError as exc:
            check(f"refused: {label}", why in str(exc), str(exc))
    try:
        brief([can_row(), can_row(name='RF2', ref='SH2', zone=[20, 20, 35, 35],
                                  members={'B': {'in': ['R5']}})])
        check('two cans over one area of different faces are accepted', True)
    except db.BriefError as exc:
        check('two cans over one area of different faces are accepted', False,
              str(exc))
    try:
        db.brief_from_dict({'schema': 1, 'kind': 'design-brief',
                            'keepouts': [{'name': 'can:x', 'rect': ZONE}]})
        check('refused: a hand-written can: keep-out', False, 'accepted')
    except db.BriefError as exc:
        check('refused: a hand-written can: keep-out', 'reserved' in str(exc),
              str(exc))

    pcb, path = board(tmp, 'b8')
    _d, _f, rep = intent_for(brief([can_row(members={'F': {'in': ['C1', 'U7']}})]),
                             pcb, path)
    check('a member the board does not have is reported unmatched',
          'U7' in rep.get('unmatched', ()), str(rep.get('unmatched')))

    print('9. min_reader')
    _d, frag, _r = intent_for(brief([can_row(ref=None, wall_mm=1.0)]))
    check('the fence reader', frag.get('min_reader') == fp.FENCE_READER,
          str(frag.get('min_reader')))
    _d, frag, _r = intent_for(brief([can_row(members={'F': {'either': ['C1']}})]))
    check('the same reader for a can with no block',
          frag.get('min_reader') == fp.FENCE_READER, str(frag.get('min_reader')))

    print('10. place_simple from a pile, with the row and without it')
    from kicad_parser import parse_kicad_pcb
    path = netted_board(os.path.join(tmp, 'e2e.kicad_pcb'))
    pcb = quiet(parse_kicad_pcb, path)
    pcb.source_path = path
    zone, wall = [7, 10, 21, 30], 0.8
    banded = [zone[0] - wall, zone[1] - wall, zone[2] + wall, zone[3] + wall]
    row = {'name': 'RF', 'zone': zone, 'side': 'F', 'wall_mm': wall,
           'interior_height_mm': 2.5, 'source': 'test',
           'members': {'F': {'in': ['C1', 'U1'], 'either': ['C2']}}}
    hts = (('C1', 0.9), ('C2', 3.0), ('U1', 0.9), ('R9', 0.5), ('R8', 0.5),
           ('J1', 5.0))
    on, _f, _r = intent_for(brief([row], hts), pcb, path)
    off = {k: v for k, v in on.items() if k not in ('blocks', 'keepouts')}
    groups = os.path.join(tmp, 'e2e_groups.json')
    with open(groups, 'w') as fh:
        json.dump({'schema': 1, 'groups': [{'name': 'all',
                                            'refs': ['C1', 'C2', 'U1', 'R9', 'R8']}]}, fh)
    broke = {'in parts out': 0, 'a stranger in': 0, 'the tall part under': 0}
    for arm, doc in (('with', on), ('without', off)):
        ip = os.path.join(tmp, f'e2e_{arm}.json')
        with open(ip, 'w') as fh:
            json.dump(doc, fh)
        for seed in (0, 1, 2):
            out = os.path.join(tmp, f'e2e_{arm}{seed}.kicad_pcb')
            run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('place_simple.py'), path, out,
                             '--groups', groups, '--intent', ip,
                             '--seed', str(seed)], accept=True)
            run_utils.evidence(out, 'the placed board')
            got = quiet(parse_kicad_pcb, out)
            got.source_path = out
            res = {'in parts out': not all(pads_in(got, r, zone, True)
                                           for r in ('C1', 'U1')),
                   'a stranger in': any(pads_in(got, r, banded, False)
                                        for r in ('R9', 'R8')),
                   'the tall part under': pads_in(got, 'C2', zone, False)}
            if arm == 'with':
                errs = [v for v in grade(on, got, out)]
                check(f"seed {seed}: in parts in, strangers and the tall part out,"
                      f" no finding", not any(res.values()) and errs == [],
                      f"{res} {errs}")
            else:
                for k, v in res.items():
                    broke[k] += v
    # The control reads what place_simple does with no can at all. If a
    # placer change keeps all three right on every seed, this fails with
    # nothing wrong in cans[]: pick seeds or parts that break them again.
    check('without the row the same seeds break each check (the control)',
          all(broke.values()), str(broke))

    print(f"\n{'FAIL' if failures else 'PASS'}: test_1475_brief_cans "
          f"({len(failures)} failure(s))")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
