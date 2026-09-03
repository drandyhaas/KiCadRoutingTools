"""#850: both lane ledgers bucket a pad to a face with the SAME rule.

`escape` measured each pad by its own COPPER EDGE against the part's copper
box, within `max(pad_pitch/2, INTERIOR_EPS)`, and reported a pad boxed in on
every side as INTERIOR -- it cannot leave sideways at any pitch, it needs a
via, and rolling it into a face's demand blames the face for a fanout problem.
`routability.face_lane_ledger` took `min` over `abs(pad CENTRE - extent edge)`:
no tolerance, no interior case, so EVERY netted pad was demand on some face.
Neither file recorded that the other existed.

What is asserted, and why each arm is here rather than implied:

 1. THE SPY, on the upstream producer. Asserting the published row against the
    resolver that filled the row is true by construction and sees nothing --
    the mistake that passed for six commits on #841's branch. This arm patches
    `escape.face_of` and inspects what it was HANDED: the rect by OBJECT
    IDENTITY against the netted-pad box (not `==`: on a part with no NPTH and
    no unnetted pad the three candidate boxes coincide, so equality cannot
    bite), the pitch, and a real `pad_box` for every netted pad.

 2. CONSERVATION. Every netted pad is on exactly one face or interior, and the
    published `interior_pads` is the count the spy saw. Arm 1 checks what the
    kernel was asked; this checks what was done with the answer. It is what
    catches a pad dropped on the floor.

 3. THE RECONCILIATION #850 ASKS FOR, and the reason this PR exists:
    `interior_pads` on the row EQUALS `escape.PartEscape.interior_pads` for
    the same ref at the same clearance, on every fine-pitch ref of every
    tracked board. Exact, not approximate. It holds only because the
    classification is geometric FIRST and the owner filter second, and because
    both sides are handed the same `CopperGeometry` at the same clearance.

 4. A GOLDEN RECORDED FROM THE PARENT COMMIT e239e067. Arms 1-3 all run
    through the same shared function, so a mutation to it moves both sides of
    any equivalence equally and they still pass. A value from before the
    change is the only oracle that does not move with it.

 5. ISOLATION. `length_mm`, `supply_*`, `eaten_by` and the band fields are
    unchanged by #850 on every row of every board -- measured by toggling the
    face rule in one process. #850 changed the demand model and provably
    nothing else; without this arm, a change that also moved the face GEOMETRY
    would satisfy arm 4 with a compensating supply change.

5b. ...and the face GEOMETRY is still `pp.extent`, asserted on the five refs
    where the extent and the copper box actually differ. Arm 5 cannot see
    that: a mutation moving `ext` to the copper box moves BOTH of its arms
    equally. The mutation battery found this absence -- see the arm.

 6. THE FACE MAP, on a fixture with a DIFFERENT count on each face. An
    inverted `{'north': 'S'}` preserves every total, every conservation law
    and the interior equality; only an asymmetric per-face expectation catches
    it.

 7. THE TIE-BREAK. #850 also moves ties from this ledger's dict order
    (N, S, W, E) to `escape.FACES` order (N, E, S, W). Invisible on a corpus
    board where no tie is exact.

 8. THE UNNETTED-PAD BOX. Eight unnetted alignment marks on ulx3s U1 set the
    box for 379 netted balls and made every one of them interior. This arm is
    the true positive for `_assignment_rect`.

 9. THE GATE DID NOT GO QUIET. All three `check_channels` gate predicates read
    `demand_nets` and two threshold at `GATE_MIN_DEMAND = 7`, so a demand-only
    fall can silence a true positive with no output line saying a face left
    the gate. The tracked `tests/fixtures/run23` tigard pair is the only
    clean-clone positive `--gate` has.

10. THE STARVED BUDGET. Arm 9 shows the positive survived; this shows it was
    not bought by inventing negatives elsewhere.

Run: python3 -X utf8 tests/test_850_demand_face_of.py
"""
import os
import sys

RUN_ALL_TIMEOUT = 900

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT, os.path.join(_ROOT, 'py_placer'),
           os.path.join(_ROOT, 'py_router'), os.path.join(_ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                             # noqa: E402
import synth                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import escape as E                            # noqa: E402
from placement import legality as L                          # noqa: E402
from placement import routability as R                       # noqa: E402

SKIP_EXIT = 77
FAILURES = []

#: One basis, stated. `part_copper_geometry`'s NPTH hole extents grow below
#: 0.20mm clearance, so a census at another clearance is a different census.
CLR, TRK, GRID = 0.2, 0.2, 0.05
LANE = dict(clearance=CLR, track_width=TRK, grid_step=GRID)

#: `{board:ref: (demand at e239e067, demand now, interior_pads,
#:   interior_demand_nets)}` -- the parent-commit golden (arm 4).
#:
#: Read the rows, they are the finding:
#:   rp2350 U2   31 nets -> 17, 25 interior pads, 13 nets left the faces.
#:   tigard U3   UNCHANGED, with 18 interior pads and 0 nets lost -- every one
#:               of those nets has another pad that IS on a face. The interior
#:               bucket takes a net's demand only when it has nowhere else to
#:               go. That is the whole difference between reclassifying and
#:               not looking, and it is why `interior_demand_nets` is
#:               published beside `interior_pads`.
#:   glasgow U1  UNCHANGED, same reason, with 27 interior pads.
#:   ulx3s U1    214 -> 61, the largest fall on the corpus, and the row that
#:               nearly shipped as 214 -> 0: see arm 8.
#:   watchy U1   N 1->2, W 1->0 -- a net MOVED between faces, which the
#:               tolerance and the `escape.FACES` tie order both do and the
#:               interior bucket cannot. The part total falls by 1 while one
#:               face RISES, which is why nothing here asserts a per-face
#:               ceiling.
GOLDEN = {
    'rp2350_fpga_eensy_prePlane:U2': ({'N': 7, 'S': 13, 'W': 5, 'E': 6},
                                      {'N': 5, 'S': 6, 'W': 2, 'E': 4}, 25, 13),
    'tigard:U3': ({'N': 5, 'S': 8, 'W': 10, 'E': 9},
                  {'N': 5, 'S': 8, 'W': 10, 'E': 9}, 18, 0),
    'glasgow_revC:U1': ({'N': 7, 'S': 10, 'W': 7, 'E': 12},
                        {'N': 7, 'S': 10, 'W': 7, 'E': 12}, 27, 0),
    'ulx3s:U1': ({'N': 64, 'S': 20, 'W': 63, 'E': 67},
                 {'N': 17, 'S': 10, 'W': 18, 'E': 16}, 312, 145),
    'watchy:U1': ({'N': 1, 'S': 1, 'W': 1, 'E': 0},
                  {'N': 2, 'S': 1, 'W': 0, 'E': 0}, 0, 0),
}

#: Refs whose ASSIGNMENT box is set by an unnetted pad (arm 8), and how many
#: of their netted pads that pushed interior. Only ulx3s U1 is inverted by it.
UNNETTED_BOX = {
    'ulx3s:U1': (379, 312),
    'qfn_interior_pads:U1': (5, 1),
    'qfn_csi_underpad_diff:U1': (0, 0),
    'qfn_diffpair_escape:U1': (0, 0),
    'qfn_underpad_coupling:U1': (0, 0),
    'watchy:J3': (0, 0),
}


def check(name, cond, detail=''):
    if cond:
        print('  PASS  %s' % name)
    else:
        FAILURES.append(name)
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


def _boards():
    return {os.path.splitext(os.path.basename(p))[0]: os.path.join(_ROOT, p)
            for p in run_utils.corpus_boards()}


def _rows(pcb, ref, path, ctx=None):
    return R.face_lane_ledger(pcb, ref, pcb_file=path, context=ctx, **LANE)


# --- 1-2. the spy and conservation ------------------------------------------

def the_spy_and_conservation(boards):
    # rp2350 J2 and watchy SW1-SW4 are the corpus's NPTH parts: on them
    # `CopperGeometry.rect` and `.copper` genuinely differ, so an identity
    # check on the rect has something to catch. ulx3s U1 is the unnetted-pad
    # part, where the netted box differs from `.copper` as well.
    for name, ref in (('rp2350_fpga_eensy_prePlane', 'U2'), ('tigard', 'U3'),
                      ('ulx3s', 'U1'), ('watchy', 'J3')):
        path = boards.get(name)
        if path is None:
            check('spy: %s present' % name, False)
            continue
        pcb = parse_kicad_pcb(path)
        fp = pcb.footprints[ref]
        ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
        seen = []
        real = E.face_of

        def spy(pad, rect, pitch, pad_box=None):
            seen.append((pad, rect, pitch, pad_box))
            return real(pad, rect, pitch, pad_box=pad_box)

        E.face_of = spy
        try:
            rows = _rows(pcb, ref, path, ctx)
        finally:
            E.face_of = real
        if not rows:
            check('spy: %s %s has a ledger' % (name, ref), False)
            continue
        check('spy: %s %s -- the kernel was called once per pad' % (name, ref),
              len(seen) == len(fp.pads),
              '%d calls for %d pads' % (len(seen), len(fp.pads)))
        # THE PAIRING, checked TWO ways, because the obvious one is true by
        # construction. Calling `_assignment_rect` to build the expectation
        # makes this arm blind to a mutation OF `_assignment_rect` -- measured:
        # replacing it with `geom.rect` or `geom.copper` leaves this equality
        # passing (the golden and the unnetted-box arms are what kill those).
        # What it DOES catch, and is here for, is `face_lane_ledger` growing
        # its own box back and handing the kernel something else.
        want = E._assignment_rect(
            [(p, L.pad_box(ctx.geom[ref], p)) for p in fp.pads],
            ctx.geom[ref], None)
        check('spy: %s %s -- the ledger did not build its own box' % (name, ref),
              all(r == want for _p, r, _pi, _b in seen),
              'want %r, saw %r' % (want, {r for _p, r, _pi, _b in seen}))
        # ...and the INVARIANT, derived from the pads rather than from the
        # function: every edge of the box `face_of` measures against is
        # attained by some NETTED pad, and no netted pad lies outside it. That
        # is the property `face_of`'s docstring rests on -- it is what puts an
        # edge pad at distance exactly 0 -- and it is FALSE for `geom.copper`
        # on a part whose box is set by an unnetted pad, which is #850's own
        # ulx3s finding.
        rects = [r for _p, r, _pi, _b in seen]
        nb = [L.pad_box(ctx.geom[ref], p) for p in fp.pads if p.net_id]
        nb = [b for b in nb if b is not None]
        if rects and nb:
            r = rects[0]
            attained = all(
                any(abs(b[i] - r[i]) < 1e-9 for b in nb) for i in range(4))
            inside = all(b[0] >= r[0] - 1e-9 and b[1] >= r[1] - 1e-9
                         and b[2] <= r[2] + 1e-9 and b[3] <= r[3] + 1e-9
                         for b in nb)
            check('spy: %s %s -- every edge of that box is attained by a '
                  'NETTED pad, and none lies outside it' % (name, ref),
                  attained and inside,
                  'rect %r attained=%s inside=%s' % (r, attained, inside))
        check('spy: %s %s -- at the part\'s own pad pitch' % (name, ref),
              all(abs(pi - E.pad_pitch(fp)) < 1e-12 for _p, _r, pi, _b in seen)
              and abs(rows[0]['face_pitch_mm'] - round(E.pad_pitch(fp), 4)) < 1e-9,
              'pad_pitch %r, row %r' % (E.pad_pitch(fp), rows[0]['face_pitch_mm']))
        check('spy: %s %s -- every netted pad got its own copper box'
              % (name, ref),
              all(b is not None for p, _r, _pi, b in seen if p.net_id),
              '%d netted pads with no box'
              % sum(1 for p, _r, _pi, b in seen if p.net_id and b is None))
        # 2. CONSERVATION.
        faced = sum(1 for (_p, _r, _pi, _b), (pad, f)
                    in zip(seen, E.assign_faces(fp, ctx.geom[ref],
                                                lane_mm=1.0).faces)
                    if pad.net_id and f is not None)
        interior = sum(1 for pad, f in E.assign_faces(
            fp, ctx.geom[ref], lane_mm=1.0).faces if pad.net_id and f is None)
        netted = sum(1 for p in fp.pads if p.net_id)
        check('conservation: %s %s -- faced + interior == netted pads'
              % (name, ref), faced + interior == netted,
              '%d + %d != %d' % (faced, interior, netted))
        check('conservation: %s %s -- the row publishes that interior count'
              % (name, ref), rows[0]['interior_pads'] == interior,
              '%r != %d' % (rows[0]['interior_pads'], interior))


# --- 3. the reconciliation --------------------------------------------------

def interior_equals_escape(boards):
    agree = 0
    bad = []
    for name, path in sorted(boards.items()):
        pcb = parse_kicad_pcb(path)
        ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
        sides = E.board_side_map(pcb)
        cont = E.board_container_refs(pcb, path)
        for ref in E.fine_pitch_parts(pcb):
            rows = _rows(pcb, ref, path, ctx)
            if not rows:
                continue
            # `ignore_net_ids=()` is the POPULATION control -- escape_ledger's
            # caller may drop plane rails, and then the two count different
            # pads. The same `geom` at the same clearance is the second
            # control: below 0.20mm the NPTH hole extents differ.
            pe = E.part_escape(pcb, ref, ignore_net_ids=(),
                               obstruction_rects=ctx.geom, sides=sides,
                               containers=cont, clearance=CLR)
            if rows[0]['interior_pads'] == pe.interior_pads:
                agree += 1
            else:
                bad.append((name, ref, rows[0]['interior_pads'],
                            pe.interior_pads))
    check('reconciliation: interior_pads is EQUAL on every fine-pitch ref',
          not bad and agree > 80,
          'agreed on %d, disagreed on %d: %r' % (agree, len(bad), bad[:6]))


# --- 4. the parent-commit golden --------------------------------------------

def the_golden(boards):
    fell = 0
    for key, (pre, post, ipads, ilost) in sorted(GOLDEN.items()):
        name, ref = key.split(':')
        path = boards.get(name)
        if path is None:
            check('golden: %s present' % name, False)
            continue
        rows = _rows(parse_kicad_pcb(path), ref, path)
        got = {r['face']: r['demand_nets'] for r in rows}
        check('golden: %s demand is the recorded transition' % key, got == post,
              '%r != %r  (was %r at e239e067)' % (got, post, pre))
        check('golden: %s interior_pads / interior_demand_nets' % key,
              (rows[0]['interior_pads'], rows[0]['interior_demand_nets'])
              == (ipads, ilost),
              '%r != %r' % ((rows[0]['interior_pads'],
                             rows[0]['interior_demand_nets']), (ipads, ilost)))
        check('golden: %s the part total did not grow' % key,
              sum(got.values()) <= sum(pre.values()),
              '%d > %d' % (sum(got.values()), sum(pre.values())))
        if sum(got.values()) < sum(pre.values()):
            fell += 1
    # ...and the file is not measuring nothing: some ref must actually move,
    # or every assertion above is satisfied by an inert change.
    check('golden: the change moved at least two of the pinned refs', fell >= 2,
          '%d fell' % fell)


# --- 5. isolation -----------------------------------------------------------

STATIC = ('length_mm', 'supply_routed_grid', 'supply_finest_grid', 'eaten_by',
          'escape_band_mm', 'escape_band_source', 'escape_band_basis',
          'taps_not_modeled')


def only_demand_moved(boards):
    """Toggle the face rule in ONE process and diff every other key.

    The old rule is carried here rather than checked out, so this arm needs no
    second tree and cannot drift out of date with the commit it describes.
    """
    real = E.assign_faces

    def old_rule(fp, geom, *, lane_mm, fallback_rect=None):
        # `min` over |pad CENTRE - extent edge|, no tolerance, no interior --
        # `face_lane_ledger` as it stood at e239e067.
        ext = fallback_rect
        out = []
        for pad in (fp.pads or []):
            d = {'north': abs(pad.global_y - ext[1]),
                 'south': abs(pad.global_y - ext[3]),
                 'west': abs(pad.global_x - ext[0]),
                 'east': abs(pad.global_x - ext[2])}
            out.append((pad, min(d, key=d.get)))
        return E.FaceAssignment(faces=tuple(out), pitch_mm=0.0,
                                pitch_source='test_old_rule')

    drift, moved = [], 0
    for name, path in sorted(boards.items()):
        pcb = parse_kicad_pcb(path)
        ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
        refs = list(E.fine_pitch_parts(pcb))
        new = {r: _rows(pcb, r, path, ctx) for r in refs}
        E.assign_faces = old_rule
        try:
            old = {r: _rows(pcb, r, path, ctx) for r in refs}
        finally:
            E.assign_faces = real
        for ref in refs:
            for a, b in zip(old[ref], new[ref]):
                for k in STATIC:
                    if a[k] != b[k]:
                        drift.append((name, ref, a['face'], k, a[k], b[k]))
                if a['demand_nets'] != b['demand_nets']:
                    moved += 1
    check('isolation: only demand and deficit moved -- no supply, span or '
          'blocker key did', not drift, '%d drifting: %r' % (len(drift),
                                                             drift[:5]))
    check('isolation: ...and demand DID move, so the arm is not vacuous',
          moved > 50, '%d faces moved' % moved)


#: Fine-pitch refs whose `CopperGeometry.rect` and `.copper` actually DIFFER,
#: with the face length the WHOLE-PART extent gives, in row order N, S, W, E.
#: Five refs corpus-wide; everywhere else the two boxes coincide and no
#: assertion can tell them apart.
EXTENT_FACES = {
    'rp2350_fpga_eensy_prePlane:J2': [6.071, 6.071, 3.023, 3.023],
    'watchy:SW1': [3.5, 3.5, 6.8, 6.8],
    'watchy:SW2': [3.5, 3.5, 6.8, 6.8],
    'watchy:SW3': [3.5, 3.5, 6.8, 6.8],
    'watchy:SW4': [3.5, 3.5, 6.8, 6.8],
}


def the_face_geometry_is_still_the_extent(boards):
    """`length_mm` comes from `pp.extent`, NOT from the copper box.

    THIS ARM EXISTS BECAUSE THE MUTATION BATTERY FOUND ITS ABSENCE. The row
    `the-face-geometry-moves-to-the-copper-box` -- which builds `faces` from
    `ctx.geom[ref].copper` instead of `pp.extent` -- SURVIVED the first run,
    against two witnesses that both should have caught it:

    * `only_demand_moved` above toggles the face RULE and diffs the static
      keys. The mutation moves `length_mm` in BOTH arms equally, so the diff
      stays empty. That is #849's own warning -- an equivalence check cannot
      see a change that moves both of its sides -- landing in my arm.
    * `test_849_lane_context`'s `GOLDEN_PRE_HOIST` pins `supply_*` and
      `eaten_by` against b5c567c7 on rp2350 U2 and tigard U3. Neither part
      carries an NPTH hole, so `rect == copper` on both and the mutation is a
      value no-op on exactly the two refs that are pinned.

    So the property needs asserting where the two boxes differ, and there are
    only five such refs on the whole tracked corpus (the 12-edge NPTH census
    in `CopperGeometry.copper`'s docstring). On rp2350 J2 the extent is up to
    1.372mm wider per side than the copper.
    """
    for key, want in sorted(EXTENT_FACES.items()):
        name, ref = key.split(':')
        path = boards.get(name)
        if path is None:
            check('extent faces: %s present' % name, False)
            continue
        pcb = parse_kicad_pcb(path)
        ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
        g = ctx.geom.get(ref)
        rows = _rows(pcb, ref, path, ctx)
        if g is None or not rows:
            check('extent faces: %s has geometry and a ledger' % key, False)
            continue
        # The precondition, asserted rather than assumed: if a future corpus
        # or clearance change made these two boxes equal, this arm would pass
        # while measuring nothing.
        check('extent faces: %s -- rect and copper DIFFER, so the arm bites'
              % key, g.rect != g.copper, 'both %r' % (g.rect,))
        got = [r['length_mm'] for r in rows]
        check('extent faces: %s -- length_mm is the EXTENT face, not the '
              'copper one' % key, got == want, '%r != %r' % (got, want))


# --- 6-7. the face map and the tie-break, on fixtures -----------------------

def _pad(num, x, y, *, w=0.4, h=0.4, net=0):
    return ('\t\t(pad "%s" smd rect\n\t\t\t(at %s %s)\n\t\t\t(size %s %s)\n'
            '\t\t\t(layers "F.Cu")\n\t\t\t(net %d "N%d")\n'
            '\t\t\t(uuid "q%s")\n\t\t)\n' % (num, x, y, w, h, net, net, num))


def _two_part_board(pads, tmpdir, name='f850.kicad_pcb'):
    """U1 carrying `pads`, plus a far-away sink so every net has 2 owners."""
    sink = ''.join(_pad('s%d' % n, 0.5 * n, 0.0, net=n) for n in range(1, 12))
    fps = (synth.footprint_text('U1', 50.0, 50.0, pads=0, extra=''.join(pads))
           + synth.footprint_text('U9', 120.0, 120.0, pads=0, extra=sink))
    path = os.path.join(tmpdir, name)
    synth.write_board(synth.board_text(fps, nets=range(0, 12)), path)
    return path


def the_face_map(tmpdir):
    """A 5 x 5 box with 1 / 2 / 3 / 4 pads hard against N / E / S / W.

    Every count differs, and each pad carries its OWN net, so the expectation
    is a four-way asymmetric equality. That is the only shape that catches an
    inverted `FACE_LETTER`: swap 'north' and 'south' in it and every total,
    every conservation law and the interior equality are preserved -- the four
    numbers just change places.
    """
    pads = [_pad('n1', 0.0, -2.5, net=1)]
    pads += [_pad('e%d' % i, 2.5, -1.0 + i, net=2 + i) for i in range(2)]
    pads += [_pad('s%d' % i, -1.0 + i, 2.5, net=4 + i) for i in range(3)]
    pads += [_pad('w%d' % i, -2.5, -1.5 + i, net=7 + i) for i in range(4)]
    path = _two_part_board(pads, tmpdir)
    pcb = parse_kicad_pcb(run_utils.evidence(path, 'the #850 face-map fixture'))
    rows = {r['face']: r['demand_nets'] for r in _rows(pcb, 'U1', path)}
    check('face map: N/E/S/W carry exactly 1/2/3/4 nets, each face its own',
          rows == {'N': 1, 'E': 2, 'S': 3, 'W': 4}, 'rows=%r' % (rows,))
    # ...and the letters are the four the schema publishes, in the row order
    # `check_channels` prints and `GOLDEN_PRE_HOIST` pins.
    order = [r['face'] for r in _rows(pcb, 'U1', path)]
    check('face map: the rows are still N, S, W, E in that order',
          order == ['N', 'S', 'W', 'E'], 'order=%r' % (order,))
    return rows


def the_tie_break(tmpdir):
    # A pad at the exact corner of a square box is equidistant from two faces.
    # escape.FACES order is N, E, S, W; the old dict order was N, S, W, E, and
    # both put N first -- so the discriminating tie is S-vs-E, where FACES
    # gives EAST and the dict order gives SOUTH.
    pads = [_pad('a', -2.0, -2.0, net=1), _pad('b', 2.0, -2.0, net=2),
            _pad('c', -2.0, 2.0, net=3),
            _pad('t', 2.0, 2.0, net=5)]        # the SE corner: south vs east
    path = _two_part_board(pads, tmpdir, 'f850tie.kicad_pcb')
    pcb = parse_kicad_pcb(path)
    fp = pcb.footprints['U1']
    geom = L.part_copper_geometry(pcb.footprints, CLR)['U1']
    faces = {p.pad_number: f
             for p, f in E.assign_faces(fp, geom, lane_mm=0.4).faces}
    check('tie-break: an exact SE corner resolves EAST (escape.FACES order), '
          'not SOUTH (the ledger\'s old dict order)',
          faces.get('t') == 'east', 'pad t -> %r, all %r' % (faces.get('t'),
                                                             faces))


# --- 8. the unnetted-pad box ------------------------------------------------

def the_unnetted_box(boards):
    seen = {}
    for key, want in sorted(UNNETTED_BOX.items()):
        name, ref = key.split(':')
        path = boards.get(name)
        if path is None:
            check('unnetted box: %s present' % name, False)
            continue
        pcb = parse_kicad_pcb(path)
        fp = pcb.footprints[ref]
        geom = L.part_copper_geometry(pcb.footprints, CLR)[ref]
        boxes = [(p, L.pad_box(geom, p)) for p in fp.pads]
        netted_box = E._assignment_rect(boxes, geom, None)
        check('unnetted box: %s\'s box is INSET from the all-pad box' % key,
              netted_box != geom.copper,
              '%r == %r' % (netted_box, geom.copper))
        n_all = sum(1 for p, f in E.assign_faces(
            fp, geom, lane_mm=0.4).faces if p.net_id and f is None)
        # ...and the same part measured against the all-pad box, which is
        # what shipped before this commit.
        n_old = sum(1 for p in fp.pads if p.net_id
                    and E.face_of(p, geom.copper, E.pad_pitch(fp),
                                  pad_box=L.pad_box(geom, p)) is None)
        seen[key] = (n_old, n_all)
    check('unnetted box: the recorded netted-pad-interior census',
          seen == UNNETTED_BOX, '%r != %r' % (seen, UNNETTED_BOX))
    check('unnetted box: ulx3s U1 was ALL-interior and is not any more',
          seen.get('ulx3s:U1', (0, 0))[0] == 379
          and seen.get('ulx3s:U1', (0, 1))[1] < 379,
          '%r' % (seen.get('ulx3s:U1'),))


# --- 9-10. the gate ---------------------------------------------------------

def the_gate_still_fires():
    fix = os.path.join(_ROOT, 'tests', 'fixtures', 'run23')
    dmg = os.path.join(fix, 'tigard_damaged.kicad_pcb')
    ok = os.path.join(fix, 'tigard_placed.kicad_pcb')
    if not (os.path.isfile(dmg) and os.path.isfile(ok)):
        check('gate: the tracked run23 pair is present', False)
        return
    run_utils.evidence(dmg, 'the damaged tigard')
    run_utils.evidence(ok, 'the placed tigard')
    r = run_utils.check([sys.executable, '-X', 'utf8',
                         os.path.join(_ROOT, 'py_tools', 'check_channels.py'),
                         dmg, '--baseline', ok, '--gate',
                         '--track-width', '0.15', '--clearance', '0.15'],
                        code=4, refuse='NEW')
    out = r.stdout or ''
    check('gate: it still names U3 E at demand 9 -- the demand survived the '
          'interior bucket', 'U3 E: demand 9' in out,
          out[-600:])
    check('gate: and U3 W at demand 10', 'U3 W: demand 10' in out, out[-600:])


def the_starved_budget(boards):
    import check_channels as CC
    grew = {}
    for name, path in sorted(boards.items()):
        pcb = parse_kicad_pcb(path)
        ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
        led = {}
        for ref in E.fine_pitch_parts(pcb):
            rows = _rows(pcb, ref, path, ctx)
            if rows:
                led[ref] = rows
        n = len(CC._starved_faces(led, CC.GATE_MIN_DEMAND))
        if n:
            grew[name] = n
    # Recorded, because "did not grow" needs a number to be a claim.
    check('starved budget: the boards with a starved face, and how many',
          grew == {'orangecrab_ext_pll': 1, 'rp2350_fpga_eensy_prePlane': 1},
          '%r' % (grew,))


def main():
    boards = _boards()
    if not boards:
        print('SKIP: git could not name the tracked corpus')
        return SKIP_EXIT
    import tempfile
    print('1-2. the spy on the kernel, and conservation')
    the_spy_and_conservation(boards)
    print('3. the reconciliation #850 asks for')
    interior_equals_escape(boards)
    print('4. the golden recorded from e239e067')
    the_golden(boards)
    print('5. isolation: only demand moved')
    only_demand_moved(boards)
    print('5b. the face geometry is still the extent')
    the_face_geometry_is_still_the_extent(boards)
    print('6-7. the face map and the tie-break')
    with tempfile.TemporaryDirectory() as td:
        the_face_map(td)
        the_tie_break(td)
    print('8. the unnetted-pad assignment box')
    the_unnetted_box(boards)
    print('9. the gate still fires')
    the_gate_still_fires()
    print('10. the starved budget')
    the_starved_budget(boards)
    if FAILURES:
        print('\nFAIL: %d check(s): %s' % (len(FAILURES), ', '.join(FAILURES)))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
