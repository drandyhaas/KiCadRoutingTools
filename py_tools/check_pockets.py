#!/usr/bin/env python3
"""Pocket census: windowed routing demand vs free copper area (run-23).

Every per-net handoff instrument passed run 23's board -- check_channels: 0
starved faces; check_reachability: every failing pad PASSABLE copper-free;
crossings below the damaged baseline -- and two nets then died across ~20
route laps in ONE pocket (the RN7/U6/J2 region), where committed copper
finally wrapped RN7 at 0.095mm against a 0.45mm corridor need. Per-net tests
are blind to SIMULTANEOUS routability by construction; this tool reports the
aggregate: the board binned at --bin mm, each bin's distinct demanding nets
against its free area, top offenders named with their nets and parts.

#709 -- IT COULD NOT SAY "EMPTY". `congestion_bins` keyed its result off the
`owners` map, which only gains a key where some net has a terminal, so a
window with nothing in it never entered the dict: measured on esp_prog at the
default 2mm bin the census reported 44 windows where 128 lie in the outline,
and on glasgow_revC 457 of 1000. The finding a placement reviewer most wants
-- "there is a clear empty pocket here, a 6-8mm band" -- was unproducible by
construction. So the census now enumerates every in-outline window, reports
COLD regions (in-outline, no demand, no copper, no part) ranked by CONTIGUOUS
AREA rather than by window count, and prints the arrangement census that
belongs beside them: where the part mass sits against where the demand sits.

Three things that are deliberate and easy to get wrong:

* **The lattice is the ROUTER'S lattice.** Bins are keyed off absolute board
  mm (`int(x // bin)`), and `congestion2_rows` maps them back the same way to
  price A* cells. Re-origining the grid on the outline would put this report
  on a lattice the router does not use -- silently, since congestion-v2 is
  off by default -- so an edge-straddling window is reported with its
  IN-OUTLINE AREA FRACTION instead of being aligned away.
* **Cold is not the same as unreported.** Most windows the old census could
  not see sit under a part courtyard, and a window under a part is not a
  pocket. Nor is part-free the same as empty: on a routed board, windows with
  no part still carry copper. Cold means all four.
* **The centroid is weighted by COURTYARD AREA, not part count.** The count
  form is the intuitive one and it points the wrong way; it is computed and
  printed as a labelled control, never as the answer.

REPORT-ONLY, deliberately: a new metric mis-thresholded is noise, so nothing
gates on it. The boundary review (the blind-first visual gate) must
DISPOSITION every window this prints -- in writing, against a named plan --
or refuse the handoff itself.

    python3 -X utf8 py_tools/check_pockets.py <board> [--nets GLOB...]
        [--bin MM] [--top N] [--threshold NETS_PER_MM2] [--json PATH]
        [--no-cold] [--cold-cover FRAC] [--outline-samples K]
        [--no-arrangement]

Exit codes: 0 always (report), 2 usage/load error.
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import math
import sys

#: Half-open containment, so a point on a shared boundary belongs to exactly
#: one window. Every consumer of a census rectangle must use the same rule --
#: `placement.utility.refs_in_rect` is the one implementation, and
#: `place_seed --reseat-region` resolves through it, so the rectangle this
#: tool prints names the same parts the mover lifts.
from placement.utility import refs_in_rect          # noqa: E402


def lattice_windows(bounds, bin_mm):
    """Every (bx, by) on the ABSOLUTE lattice whose window meets `bounds`.

    floor/ceil, not `int(hi // bin) + 1`: when a bound lands exactly on a
    lattice line the +1 form appends a row of zero-width windows. Measured on
    glasgow_revC (y1 = 120.0 at bin 2.0) that is 40 phantom windows, and the
    invisible-window count this tool exists to report would be overstated by
    exactly those 40.
    """
    def _lo(v):
        return int(math.floor(v / bin_mm))

    def _hi(v):
        # `math.ceil(21.0 / 0.7)` is 31, not 30: the quotient is
        # 30.000000000000004. That is the SAME defect floor/ceil replaced,
        # moved from the `+1` form into float division -- measured, it
        # invented 59 off-board windows on a 20x20mm rectangular board at
        # --bin 0.7. Snap the quotient to an integer it is within an ulp of
        # before taking the ceiling.
        q = v / bin_mm
        r = round(q)
        return int(r if abs(q - r) < 1e-9 else math.ceil(q))

    x0, y0, x1, y1 = bounds
    return [(bx, by)
            for bx in range(_lo(x0), _hi(x1))
            for by in range(_lo(y0), _hi(y1))]


def window_rect(b, bin_mm):
    bx, by = b
    return (round(bx * bin_mm, 6), round(by * bin_mm, 6),
            round((bx + 1) * bin_mm, 6), round((by + 1) * bin_mm, 6))


def in_outline_fraction(win, bounds, gate, samples, leg):
    """How much of this window is inside the board, in [0, 1].

    Three tiers, and the ORDER of the first two is load-bearing:

    1. No intersection with the bbox -> 0.0, before any ring work.
    2. `gate.active` is False -> the outline IS the bbox, and the bbox
       overlap is EXACT. This is not a rare fallback: `extract_board_contours`
       deliberately returns no rings for a simple axis-aligned rectangle
       ("use bounding box"), so most boards land here -- and on such a board
       the gate's point test answers True everywhere, which is why `active`
       must be consulted before it, not after.
    3. Otherwise ask the gate. A window it calls fully legal is 1.0; one it
       does not is sub-sampled, because a census window needs HOW MUCH is
       inside, not the "is it entirely inside" a part centre needs.
    """
    inb = leg.rect_overlap_area(win, bounds)
    if inb <= 1e-9:
        return 0.0
    cell = (win[2] - win[0]) * (win[3] - win[1])
    if not gate.active:
        return min(1.0, inb / cell) if cell > 0 else 0.0
    if gate.rect_outside_amount(win) <= 1e-9:
        return 1.0
    hits = 0
    for i in range(samples):
        px = win[0] + (i + 0.5) * (win[2] - win[0]) / samples
        for j in range(samples):
            py = win[1] + (j + 0.5) * (win[3] - win[1]) / samples
            if gate.rect_outside_amount((px, py, px, py)) <= 1e-9:
                hits += 1
    return hits / float(samples * samples)


def flood_regions(cold_keys):
    """4-connected components of a sparse set of (bx, by) keys.

    Same semantics as `plane_fill_model._label_components` (labels 1..n,
    4-connected) and deliberately not that function: importing
    `plane_fill_model` costs ~1.3s and pulls the routing stack into a report
    tool, to label a set of tens of windows.

    4-connected, not 8: diagonal joins merge a band into the blob next to it,
    and the band IS the finding ("an empty band south of CON2's eastern
    half"). Output order is sorted, never dict order.
    """
    remaining = set(cold_keys)
    out = []
    for seed in sorted(remaining):
        if seed not in remaining:
            continue
        remaining.discard(seed)
        comp, stack = [seed], [seed]
        while stack:
            bx, by = stack.pop()
            for nb in ((bx + 1, by), (bx - 1, by), (bx, by + 1), (bx, by - 1)):
                if nb in remaining:
                    remaining.discard(nb)
                    comp.append(nb)
                    stack.append(nb)
        out.append(sorted(comp))
    return out


def largest_band(cells):
    """Largest all-cold axis-aligned rectangle, as (bx0, by0, bx1, by1) incl.

    Maximal rectangle in a binary matrix (histogram + monotone stack). This is
    what turns a region into the reviewer's own sentence: a 24-window blob
    says nothing, "a 6.0 x 2.0mm band at [140,94]-[146,96]" is a place.
    """
    if not cells:
        return None
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    present = set(cells)
    best = (0, None)
    heights = [0] * (x1 - x0 + 1)
    for by in range(y0, y1 + 1):
        for i in range(len(heights)):
            heights[i] = heights[i] + 1 if (x0 + i, by) in present else 0
        stack = []                      # (start_index, height)
        for i, h in enumerate(heights + [0]):
            start = i
            while stack and stack[-1][1] >= h:
                s, sh = stack.pop()
                area = sh * (i - s)
                if area > best[0]:
                    best = (area, (x0 + s, by - sh + 1, x0 + i - 1, by))
                start = s
            stack.append((start, h))
    return best[1]


def copper_touched_bins(pcb, bin_mm):
    """Every bin any copper geometry ACTUALLY overlaps, not just its midpoint.

    `congestion_bins` charges a segment's whole area to the bin containing its
    MIDPOINT, which is the right model for a demand/capacity ratio and the
    wrong one for "is this window empty": a track running straight through a
    window, with its midpoint elsewhere, leaves `free == bin_area_total` and
    the window reads COLD. Measured on rp2350_fpga_eensy_prePlane at --bin 1.0
    (the bin the docs' own example uses), 27 of 159 cold windows -- 17% -- had
    a track crossing them.

    So the copper test the cold predicate uses is this one: walk each segment
    and stamp the bins its swept width touches, and stamp each via's and pad's
    bounding box. Half a bin is the walk step, which cannot skip a bin.
    """
    hit = set()

    def stamp_box(x0, y0, x1, y1):
        for bx in range(int(math.floor(x0 / bin_mm)),
                        int(math.floor(x1 / bin_mm)) + 1):
            for by in range(int(math.floor(y0 / bin_mm)),
                            int(math.floor(y1 / bin_mm)) + 1):
                hit.add((bx, by))

    for s in pcb.segments:
        r = max(s.width, 0.0) / 2.0
        n = max(1, int(math.hypot(s.end_x - s.start_x,
                                  s.end_y - s.start_y) / (bin_mm / 2.0)) + 1)
        for i in range(n + 1):
            t = i / float(n)
            x = s.start_x + (s.end_x - s.start_x) * t
            y = s.start_y + (s.end_y - s.start_y) * t
            stamp_box(x - r, y - r, x + r, y + r)
    for v in pcb.vias:
        r = max(v.size, 0.0) / 2.0
        stamp_box(v.x - r, v.y - r, v.x + r, v.y + r)
    for plist in (getattr(pcb, 'pads_by_net', {}) or {}).values():
        for p in plist:
            hx, hy = p.size_x / 2.0, p.size_y / 2.0
            stamp_box(p.global_x - hx, p.global_y - hy,
                      p.global_x + hx, p.global_y + hy)
    return hit


def courtyard_cover(parts, containers, windows, bin_mm, leg):
    """{window: {'F': mm2, 'B': mm2}} of courtyard area, charged PER SIDE.

    A through-hole part is charged to BOTH faces, because `GradedPart.sides`
    says its leads really are on both and a window under it is not clear on
    either. Charging only `gp.side` is nearly an equivalent mutation -- the two
    disagree only where a B-side part ALSO covers the window, so that the
    far-side sum overtakes the near one -- which is exactly why it is asserted
    here directly rather than through a cold-window count that no in-repo board
    happens to move.
    """
    cover = {}
    for gp in parts:
        if gp.ref in containers:
            continue
        for bx in range(int(math.floor(gp.rect[0] / bin_mm)),
                        int(math.ceil(gp.rect[2] / bin_mm))):
            for by in range(int(math.floor(gp.rect[1] / bin_mm)),
                            int(math.ceil(gp.rect[3] / bin_mm))):
                b = (bx, by)
                if b not in windows:
                    continue
                a = leg.rect_overlap_area(window_rect(b, bin_mm), gp.rect)
                if a <= 0:
                    continue
                slot = cover.setdefault(b, {'F': 0.0, 'B': 0.0})
                for s in _side_key(gp):
                    slot[s] = slot.get(s, 0.0) + a
    return cover


def _side_key(gp):
    return getattr(gp, 'sides', None) or frozenset((gp.side,))


def _containers(parts, footprints, board_area, leg, opts):
    """Refs whose courtyard is a FRAME around the design, not a body.

    Area alone is not the test -- `options` records that on a synthetically
    shrunk board the bare ratio classed 12 plain connectors as containers --
    so the hosting guard decides, and it is CALLED rather than mirrored.
    """
    if not board_area or opts is None:
        return set()
    out = set()
    for gp in parts:
        if leg.rect_area(gp.rect) < leg.CONTAINER_RATIO * board_area:
            continue
        fp = footprints.get(gp.ref)
        if fp is None:
            continue
        try:
            hosts = opts.hosts_the_design(
                gp.ref, gp.rect[0] - fp.x, gp.rect[1] - fp.y,
                gp.rect[2] - fp.x, gp.rect[3] - fp.y, fp, footprints)
        except Exception:                                       # noqa: BLE001
            hosts = False
        if hosts:
            out.add(gp.ref)
    return out


QUADRANTS = ('NW', 'NE', 'SW', 'SE')


def quadrant_of(x, y, bounds):
    """`perturb._region_unit`'s split, verbatim: 0=NW 1=NE 2=SW 3=SE.

    Reusing the numbering is what makes the output a reseat TARGET rather than
    a weight -- `--block region:q0` is a runnable argument, and it only names
    the same parts if the membership rule is the same one (half-open, on the
    FOOTPRINT ORIGIN, which is what `_region_unit` filters on).
    """
    x0, y0, x1, y1 = bounds
    mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return (0 if x < mx else 1) + (0 if y < my else 2)


def arrangement_census(pcb, parts, containers, bins, bin_mm, bounds, leg):
    """Where the part MASS sits, against where the demand sits.

    Weighted by courtyard AREA. The count-weighted form is reported beside it
    as a control and never as the answer: it is the intuitive one and it
    points the wrong way -- on esp_prog it reads 13.4% of span where the area
    form reads 2.7%.

    Refuses rather than invents: no `board_bounds` means no span and no board
    centre, so there is no offset to report and this returns None.
    """
    if not bounds:
        return None
    x0, y0, x1, y1 = bounds
    span_x, span_y = x1 - x0, y1 - y0
    cx0, cy0 = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    weighed = [p for p in parts if p.ref not in containers]

    sides = {}
    for side in ('F', 'B'):
        mine = [p for p in weighed if side in _side_key(p)]
        area = sum(leg.rect_area(p.rect) for p in mine)
        if not mine or area <= 0:
            continue                      # absent, never a zero
        ax = sum(leg.rect_area(p.rect) * (p.rect[0] + p.rect[2]) / 2.0
                 for p in mine) / area
        ay = sum(leg.rect_area(p.rect) * (p.rect[1] + p.rect[3]) / 2.0
                 for p in mine) / area
        nx = sum((p.rect[0] + p.rect[2]) / 2.0 for p in mine) / len(mine)
        ny = sum((p.rect[1] + p.rect[3]) / 2.0 for p in mine) / len(mine)
        sides[side] = {
            'parts': len(mine),
            'courtyard_area_mm2': round(area, 3),
            'offset_mm': [round(ax - cx0, 3), round(ay - cy0, 3)],
            'offset_frac_span': [
                round(abs(ax - cx0) / span_x, 4) if span_x > 0 else None,
                round(abs(ay - cy0) / span_y, 4) if span_y > 0 else None],
            'count_control_offset_mm': [round(nx - cx0, 3),
                                        round(ny - cy0, 3)],
            'count_control_frac_span': [
                round(abs(nx - cx0) / span_x, 4) if span_x > 0 else None,
                round(abs(ny - cy0) / span_y, 4) if span_y > 0 else None],
        }
    if not sides:
        return None
    headline = max(sides, key=lambda s: sides[s]['courtyard_area_mm2'])

    quads = [{'name': QUADRANTS[q], 'index': q, 'parts': 0,
              'part_area_mm2': 0.0, 'demand_nets': 0, 'distinct_nets': 0,
              'cold_windows': 0} for q in range(4)]
    #: `_region_unit` filters on the footprint ORIGIN, so this count uses the
    #: origin too and the quadrant it names is the one `--block region:qN`
    #: would act on. It counts the parts the placer would SEE: a graphic-only
    #: footprint (no courtyard, no pads -- the +/-0.5mm fiction) is not a part
    #: and is not in the quench's own part set either, so counting it here
    #: made the number 20 where `region:q*` totalled 17 on esp_prog. What the
    #: census cannot know is which of them a given run LOCKS, so this is an
    #: upper bound on the movable set, never a promise about it.
    seat = {gp.ref for gp in parts if gp.ref not in containers}
    for ref, fp in pcb.footprints.items():
        if ref not in seat:
            continue
        quads[quadrant_of(fp.x, fp.y, bounds)]['parts'] += 1
    qrects = []
    mx, my = cx0, cy0
    for q in range(4):
        qx = (x0, mx) if q in (0, 2) else (mx, x1)
        qy = (y0, my) if q in (0, 1) else (my, y1)
        qrects.append((qx[0], qy[0], qx[1], qy[1]))
    for gp in parts:
        if gp.ref in containers:
            continue
        for q in range(4):
            quads[q]['part_area_mm2'] += leg.rect_overlap_area(gp.rect,
                                                               qrects[q])
    seen = [set() for _ in range(4)]
    for (bx, by), (_free, own) in bins.items():
        if not own:
            continue
        q = quadrant_of((bx + 0.5) * bin_mm, (by + 0.5) * bin_mm, bounds)
        #: Two numbers, not one: summing len(owners) counts a net once per
        #: window it spans, which is the right measure of pressure; the union
        #: is the right measure of how many nets are actually involved.
        quads[q]['demand_nets'] += len(own)
        seen[q] |= set(own)
    for q in range(4):
        quads[q]['distinct_nets'] = len(seen[q])
        quads[q]['part_area_mm2'] = round(quads[q]['part_area_mm2'], 3)

    return {'weight': 'courtyard_area', 'headline_side': headline,
            'board_centre': [round(cx0, 3), round(cy0, 3)],
            'span_mm': [round(span_x, 3), round(span_y, 3)],
            'sides': sides, 'quadrants': quads}


def _compass(dx, dy):
    """Board compass. +y is SOUTH in KiCad's coordinate system."""
    parts = []
    if abs(dy) > 1e-9:
        parts.append('S' if dy > 0 else 'N')
    if abs(dx) > 1e-9:
        parts.append('E' if dx > 0 else 'W')
    return ''.join(parts) or 'centre'


def pocket_census(pcb, board_path, *, nets=('*',), bin_mm=2.0, top=8,
                  threshold=None, cold=True, cold_cover=0.0,
                  outline_samples=4, arrangement=True):
    """The whole census as data. `main` only formats what this returns."""
    from congestion_field import congestion_bins
    from net_queries import expand_net_patterns
    from placement import legality as leg

    try:
        from placement import options as opts
    except Exception:                                           # noqa: BLE001
        opts = None

    names = set(expand_net_patterns(pcb, list(nets)))
    name_by_id = {n.net_id: n.name for n in pcb.nets.values()}
    ids = [nid for nid, n in pcb.nets.items() if n.name in names]
    layers = list(getattr(pcb.board_info, 'copper_layers', []) or []) or ['F.Cu']
    bounds = getattr(pcb.board_info, 'board_bounds', None)

    # The corridor arithmetic a reader needs beside the ratios: one lane =
    # track + 2*clearance at the board's own floors (the 0.45mm number run
    # 23's forensics measured RN7's cage against).
    try:
        from list_nets import board_floor
        import routing_defaults as defaults
        clr, _s1 = board_floor(board_path, 'clearance', None, defaults.CLEARANCE)
        trk, _s2 = board_floor(board_path, 'track_width', None,
                               defaults.TRACK_WIDTH)
    except Exception:                                           # noqa: BLE001
        clr = trk = None

    # THE EFFECTIVE BIN, not the requested one. `congestion_bins` floors its
    # bin at 0.25mm, so a smaller --bin binned at 0.25 while this tool built
    # its window rectangles from the requested value: at --bin 0.1 the rows
    # read [25.9,21.3]-[26.0,21.4] for a window whose true left edge is
    # 64.75mm. Every coordinate in the report -- printed and JSON -- named a
    # place on the board where nothing was measured. Ask the census what it
    # actually used and build the rectangles from that.
    #
    # The lattice must therefore be built from the EFFECTIVE bin too, which
    # is why this is a two-step: ask for the bin, then enumerate.
    _b0, _t0, bin_mm = congestion_bins(pcb, ids, len(layers), bin_mm)
    lattice = lattice_windows(bounds, bin_mm) if bounds else []
    bins, _terminals, bin_mm = congestion_bins(
        pcb, ids, len(layers), bin_mm,
        include_bins=(lattice or None))

    doc = {'board': board_path,
           # The bin the census USED. `bin_requested` is what the caller
           # asked for; they differ when --bin is under the floor, and a
           # reader plotting `window` needs the one the rectangles came from.
           'bin_mm': bin_mm,
           'demand_nets': len(ids), 'layers': len(layers),
           'lane_mm': (round(trk + 2 * clr, 4)
                       if clr is not None and trk is not None else None),
           'threshold': threshold}

    bin_area_2d = bin_mm * bin_mm
    bin_area_total = bin_area_2d * len(layers)

    # --- rows: unchanged for every window that has demand ------------------
    rows = []
    for b, (free, owners) in bins.items():
        if not owners:
            continue                      # a cold candidate, reported below
        rows.append({
            'window': list(window_rect(b, bin_mm)),
            'demand_nets': len(owners),
            'free_area_mm2': round(free, 3),
            # `free` is floored at 5% of the bin area by congestion_bins, so
            # it is never 0 and the ratio is always finite. The old
            # `float('inf')` guard was unreachable AND would have written a
            # bare `Infinity` token into the JSON, which is not standard JSON
            # and which strict parsers refuse.
            'ratio': round(len(owners) / free, 4),
            'nets': sorted(name_by_id.get(n, str(n)) for n in owners)[:12],
        })
    rows.sort(key=lambda r: -r['ratio'])
    # Simultaneous routability needs >= 2 nets by definition: a single-net
    # sliver (a bin inside one part's own pad field) tops any ratio ranking
    # and means nothing here. Reported in the JSON, never in the ranking.
    hot = [r for r in rows if r['demand_nets'] >= 2
           and (threshold is None or r['ratio'] > threshold)]

    # Parts per window, for the top rows only (the fix target is a ref).
    for r in hot[:top]:
        r['refs'] = refs_in_rect(pcb, r['window'])

    doc['windows'] = rows[:max(top, 32)]
    doc['windows_demand'] = len(rows)

    if not bounds:
        # The one genuine refusal: no span, no board centre, no lattice.
        doc['outline'] = None
        doc['cold_windows'] = None
        doc['cold_regions'] = []
        doc['arrangement'] = None
        doc['reseat_target'] = None
        doc['skipped'] = ('no board_bounds: nothing to enumerate an outline '
                          'over')
        return doc, hot

    # --- the outline sweep --------------------------------------------------
    gate = leg.BoardOutlineGate(pcb.board_info, 0.0)
    rings = list(getattr(gate, 'rings', []) or [])
    doc['outline'] = {
        # `extract_board_contours` returns NO rings for a plain axis-aligned
        # rectangle ("use bounding box"), so most boards report bounding_box
        # and that is correct rather than a parse failure -- but a reader
        # comparing two boards needs to know which question was answered.
        'source': 'edge_cuts_contours' if rings else 'bounding_box',
        'rings': len(rings),
        'cutouts': len(list(getattr(gate, 'cutouts', []) or [])),
        'bounds': [round(v, 3) for v in bounds],
        'samples': outline_samples,
    }

    frac = {}
    for b in lattice:
        f = in_outline_fraction(window_rect(b, bin_mm), bounds, gate,
                                outline_samples, leg)
        if f > 0:
            frac[b] = f
    doc['windows_in_outline'] = len(frac)
    doc['windows_offboard'] = len(lattice) - len(frac)
    doc['windows_partial'] = sum(1 for f in frac.values() if f < 1.0 - 1e-9)

    # --- part coverage, per side -------------------------------------------
    #: A failure here CANNOT be swallowed into an empty part list. With the
    #: grader raising, esp_prog reported 81 cold windows instead of 29 and a
    #: 260mm2 "empty pocket" sitting on top of U1 and USB1 -- and said nothing,
    #: because the provenance line only prints when there IS an arrangement.
    #: "I could not measure the parts" and "the board has no parts" are the
    #: same distinction this whole tool exists to draw, one level up.
    graded_error = None
    try:
        graded = leg.graded_parts_from_file(pcb, board_path)
    except Exception as exc:                                    # noqa: BLE001
        graded = []
        graded_error = '%s: %s' % (type(exc).__name__, exc)
    real = [g for g in graded if not g.synthetic]
    board_area = (bounds[2] - bounds[0]) * (bounds[3] - bounds[1])
    containers = _containers(real, pcb.footprints, board_area, leg, opts)
    #: `placement_state` records an objection to courtyard-area metrics --
    #: they "break on the boards with no courtyards at all". They do not: a
    #: footprint that draws none falls back to its PAD bbox, which is real
    #: copper, and only a footprint with neither is the +/-0.5mm fiction. But
    #: a reader still needs to know which of the two they are looking at, so
    #: the provenance is counted rather than assumed.
    from_courtyard = 0
    try:
        from placement.parser import (courtyard_for_side,
                                      extract_courtyard_sides)
        _sides = extract_courtyard_sides(board_path)
        for gp in real:
            box = _sides.get(gp.ref)
            if box and courtyard_for_side(box, gp.side) is not None:
                from_courtyard += 1
    except Exception:                                           # noqa: BLE001
        from_courtyard = None
    doc['parts'] = {
        'graded': len(graded),
        'grading_error': graded_error,
        'weighed': len([g for g in real if g.ref not in containers]),
        'synthetic_excluded': len(graded) - len(real),
        'container_excluded': len(containers),
        'containers': sorted(containers),
        'from_courtyard': from_courtyard,
        'from_pads': (None if from_courtyard is None
                      else len(real) - from_courtyard),
        'container_guard': 'options.hosts_the_design' if opts is not None
                           else 'unavailable',
    }

    cover = courtyard_cover(real, containers, frac, bin_mm, leg)

    # --- cold classification ------------------------------------------------
    # Four buckets that PARTITION the in-outline windows, because the two
    # distinctions this tool exists to draw are exactly the ones that vanish
    # if a window is silently dropped: a window under a part is not a pocket,
    # and a part-free window full of another net's copper is not empty.
    touched = copper_touched_bins(pcb, bin_mm)
    cold_keys = []
    warm_unowned = 0
    under_parts = 0
    demand_in = 0
    for b, f in frac.items():
        free, owners = bins.get(b, (bin_area_total, frozenset()))
        if owners:
            demand_in += 1
            continue
        # Part-free is not empty: on a routed board a window with no part
        # still carries copper.
        #
        # The SWEPT test is the one that decides. The `free` term beside it is
        # a deliberate REDUNDANT cross-check, and saying so is the honest
        # framing: `congestion_bins` charges a segment to its midpoint bin and
        # a pad to its centre bin, both of which the swept stamp covers, so
        # swept is a strict superset. Measured over 8 boards x 3 bin sizes,
        # ZERO bins that `free` calls occupied are not also swept-touched --
        # which is why a mutation dropping the `free` arm survives the battery
        # and is an equivalent mutant rather than a coverage hole. It is kept
        # because it is one comparison and it guards the newer, more intricate
        # of the two; `t_the_swept_test_is_a_superset_of_the_free_area_rule`
        # pins the relation so the day it stops holding is a red row.
        if free < bin_area_total - 1e-9 or b in touched:
            warm_unowned += 1
            continue
        area_here = bin_area_2d * f
        c = cover.get(b, {})
        # max over sides, not sum: "a clear empty pocket" means empty on BOTH.
        if max(c.get('F', 0.0), c.get('B', 0.0)) > cold_cover * area_here + 1e-9:
            under_parts += 1
            continue
        cold_keys.append(b)
    doc['cold_windows'] = len(cold_keys)
    doc['warm_unowned_windows'] = warm_unowned
    doc['under_part_windows'] = under_parts
    #: The old census reported `len(bins)` as its window count, which mixed
    #: demand bins lying OFF the board in with the rest. This is the in-outline
    #: half, and it is the one the partition is over.
    doc['windows_demand_in_outline'] = demand_in
    doc['cold_cover'] = cold_cover

    regions = []
    for cells in flood_regions(cold_keys):
        area = round(sum(bin_area_2d * frac[c] for c in cells), 3)
        band = largest_band(cells)
        bx0, by0, bx1, by1 = band
        band_cells = [(x, y) for x in range(bx0, bx1 + 1)
                      for y in range(by0, by1 + 1)]
        band_area = round(sum(bin_area_2d * frac[c] for c in band_cells), 3)
        xs0 = min(c[0] for c in cells)
        ys0 = min(c[1] for c in cells)
        xs1 = max(c[0] for c in cells)
        ys1 = max(c[1] for c in cells)
        bbox = [round(xs0 * bin_mm, 3), round(ys0 * bin_mm, 3),
                round((xs1 + 1) * bin_mm, 3), round((ys1 + 1) * bin_mm, 3)]
        bb_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        band_rect = [round(bx0 * bin_mm, 3), round(by0 * bin_mm, 3),
                     round((bx1 + 1) * bin_mm, 3), round((by1 + 1) * bin_mm, 3)]
        near = sorted({gp.ref for gp in real
                       if gp.ref not in containers
                       and leg.rect_gap(bbox, gp.rect) <= bin_mm})
        regions.append({
            'area_mm2': area, 'windows': len(cells), 'bbox': bbox,
            'band_rect': band_rect,
            'band_mm': [round(band_rect[2] - band_rect[0], 3),
                        round(band_rect[3] - band_rect[1], 3)],
            'band_area_mm2': band_area,
            'fill': round(area / bb_area, 4) if bb_area > 0 else None,
            'edge_touch': any(frac[c] < 1.0 - 1e-9 for c in cells),
            'refs': near,
        })
    # Contiguous AREA, never window count -- that is the whole point of
    # ranking regions rather than windows. bbox breaks ties so the order is a
    # property of the finding, not of iteration.
    regions.sort(key=lambda r: (-r['area_mm2'], r['bbox']))
    for i, r in enumerate(regions):
        r['rank'] = i + 1
    #: `--no-cold` suppresses the COLD half only. It used to return before
    #: the outline sweep, so it silently took the arrangement census, the
    #: parts provenance and the reseat target with it -- while both the docs
    #: and the argparse help describe --no-arrangement as the separate switch
    #: for those.
    doc['cold_regions'] = regions[:max(top, 32)] if cold else []
    doc['cold_area_mm2'] = round(sum(r['area_mm2'] for r in regions), 3)
    in_area = sum(bin_area_2d * f for f in frac.values())
    doc['cold_area_frac'] = (round(doc['cold_area_mm2'] / in_area, 4)
                             if in_area > 0 else None)

    doc['arrangement'] = (arrangement_census(pcb, real, containers, bins,
                                             bin_mm, bounds, leg)
                          if arrangement else None)
    if doc['arrangement']:
        for q in doc['arrangement']['quadrants']:
            q['cold_windows'] = sum(
                1 for b in cold_keys
                if quadrant_of((b[0] + 0.5) * bin_mm, (b[1] + 0.5) * bin_mm,
                               bounds) == q['index'])

    if not cold:
        doc['cold_windows'] = None
        doc['warm_unowned_windows'] = None
        doc['under_part_windows'] = None
        doc['cold_area_mm2'] = None
        doc['cold_area_frac'] = None
        doc['skipped'] = 'cold census disabled'
    # #708: the lattice this board was laid out on, if it declares one. Read
    # off the same function the placer resolves its candidate offsets with, so
    # the census cannot report a pitch the engine does not use.
    try:
        from placement.board_grid import infer_board_grid
        _bg = infer_board_grid(pcb)
        doc['board_grid'] = {'step': _bg['step'], 'occupancy': _bg['occupancy'],
                             'n_parts': _bg['n_parts'],
                             'ties': list(_bg['ties']),
                             'reason': _bg['reason']}
    except Exception:                                           # noqa: BLE001
        doc['board_grid'] = None

    doc['reseat_target'] = _reseat_target(doc, bounds) if cold else None
    return doc, hot


def _reseat_target(doc, bounds=None):
    """The census's product: a place to move mass TO. A DESTINATION, not a scope.

    A printed string, not a weight and not a gate. #709 records why: a density
    objective term was tried, measured and rejected, because a bounded nudge
    cannot migrate a part out of a packed belt into empty space. That is
    reseat-scale work, so the census names a target and stops.

    IT IS NOT A `--reseat-region` ARGUMENT, and the first version of this
    printed one, which was wrong on every board rather than sometimes. A cold
    window cannot contain a pad centre BY CONSTRUCTION -- a pad's area is
    charged to its bin, so a window holding one is `warm_unowned`, never cold
    -- so `refs_in_rect(band_rect)` is empty for every band this ever names.
    Measured: 428 060 cold windows over 29 boards, zero containing a pad
    centre. The command was a scope that lifts nothing.

    The cold band's real use is as an intent block `zone`, which is the one
    thing in the stack that AIMS a re-seat at a rectangle. The scope to lift
    is a different rectangle -- the crowded one -- and the census names the
    parts bounding the pocket instead of pretending otherwise.
    """
    regions = doc.get('cold_regions') or []
    if not regions:
        return None
    top = regions[0]
    arr = doc.get('arrangement')
    move = None
    if arr and arr.get('sides'):
        s = arr['sides'][arr['headline_side']]
        dx, dy = s['offset_mm']
        # Mass sits at +offset, so the room is in the opposite direction.
        move = _compass(-dx, -dy)
    #: Clipped to the board. The band is lattice-aligned, so its outer edge
    #: overhangs the outline whenever a bound is not a multiple of the bin --
    #: on 6 of 11 measured boards -- and a zone rectangle that reaches off the
    #: board is not a zone anyone can seat a part in.
    zone = list(top['band_rect'])
    if bounds:
        zone = [max(zone[0], bounds[0]), max(zone[1], bounds[1]),
                min(zone[2], bounds[2]), min(zone[3], bounds[3])]
        zone = [round(v, 3) for v in zone]
    return {'region_bbox': top['bbox'], 'band_rect': top['band_rect'],
            'band_mm': top['band_mm'], 'area_mm2': top['area_mm2'],
            'windows': top['windows'], 'refs': top['refs'],
            'move_mass_toward': move,
            # The DESTINATION, clipped to the board. Named `zone` because that
            # is the intent key it belongs in; it is deliberately NOT called
            # `reseat_region`, which would invite the empty-scope mistake.
            'zone': zone,
            'zone_mm': [round(zone[2] - zone[0], 3),
                        round(zone[3] - zone[1], 3)],
            'contains_parts': False}


def census_scalars(doc):
    """THE derived scalars, for anyone who wants numbers rather than the doc.

    Public because there are two consumers -- this tool's own JSON_SUMMARY line
    and the predictor study's row -- and two copies of one definition is the
    defect `refs_in_rect` exists to avoid. `centroid_offset` in particular is
    not a matter of taste: taking only the X term records "dead centre" for a
    board whose mass sits 14% of the span off in Y (splitflap_driver reads
    X 0.0002 / Y 0.1395), and the study's own damage kinds -- `translate`,
    `wrong_side` -- displace mass in Y routinely. So the magnitude is the
    headline and both axes ship beside it.
    """
    arr = doc.get('arrangement') or {}
    side = (arr.get('sides') or {}).get(arr.get('headline_side')) or {}
    off = side.get('offset_frac_span') or [None, None]
    ctl = side.get('count_control_frac_span') or [None, None]
    top = (doc.get('cold_regions') or [{}])[0]
    mag = (round(math.hypot(off[0], off[1]), 4)
           if off[0] is not None and off[1] is not None else None)
    out = {
        'centroid_offset_frac': mag,
        'centroid_offset_frac_x': off[0],
        'board': doc['board'], 'bin_mm': doc['bin_mm'],
        'layers': doc['layers'], 'demand_nets': doc['demand_nets'],
        'lane_mm': doc['lane_mm'],
        'windows_demand': doc.get('windows_demand'),
        'windows_in_outline': doc.get('windows_in_outline'),
        'windows_offboard': doc.get('windows_offboard'),
        'windows_partial': doc.get('windows_partial'),
        'cold_windows': doc.get('cold_windows'),
        # Exported so a consumer can reconstruct the partition from the
        # summary line alone; `windows_demand` alone cannot, because it
        # counts off-board demand bins too.
        'windows_demand_in_outline': doc.get('windows_demand_in_outline'),
        'under_part_windows': doc.get('under_part_windows'),
        'warm_unowned_windows': doc.get('warm_unowned_windows'),
        'cold_regions': (len(doc.get('cold_regions') or [])
                         if doc.get('cold_windows') is not None else None),
        'cold_area_mm2': doc.get('cold_area_mm2'),
        'cold_area_frac': doc.get('cold_area_frac'),
        'cold_top_area_mm2': top.get('area_mm2'),
        'centroid_offset_frac_y': off[1],
        'centroid_count_control_frac': ctl[0],
        'outline_source': (doc.get('outline') or {}).get('source'),
        # #708. `board_grid_step` is None for a board that declares no lattice,
        # which is a real answer and not a missing one -- `board_grid_reason`
        # says which test it failed, so "no lattice" and "never measured" stay
        # distinguishable in the summary line alone.
        'board_grid_step': (doc.get('board_grid') or {}).get('step'),
        'board_grid_occupancy': (doc.get('board_grid') or {}).get('occupancy'),
        'board_grid_reason': (doc.get('board_grid') or {}).get('reason'),
    }
    return {k: v for k, v in out.items()
            if not (isinstance(v, float) and not math.isfinite(v))}


def format_report(doc, hot, top):
    """Every printed line, as a list. No line may start with '[' -- the
    ranked-row parser in tests/test_run23_pockets.py keys on exactly that."""
    lines = []
    lane = (f"one lane = track {doc['_trk']:g} + 2 x clearance {doc['_clr']:g}"
            f" = {doc['lane_mm']:g}mm" if doc['lane_mm'] is not None
            else "board floors unreadable")
    if doc.get('windows_in_outline') is not None:
        # `windows_demand` counts demand bins wherever they are, including one
        # under a part hanging off the board edge; the in-outline term is the
        # one the partition is over, and printing them side by side without
        # saying so mixes two populations.
        cw = doc.get('cold_windows')
        counts = (f"{doc.get('windows_demand_in_outline')} demand"
                  + (f" / {cw} cold" if cw is not None else "")
                  + f" / {doc['windows_in_outline']} in-outline window(s)"
                  + (f" ({doc['windows_demand']} demand bins in all, some "
                     f"off-board)"
                     if doc['windows_demand'] != doc.get(
                         'windows_demand_in_outline') else ""))
    else:
        counts = f"{doc['windows_demand']} window(s)"
    lines.append(f"Pocket census of {doc['board']}: {doc['demand_nets']} "
                 f"demand net(s), {counts} at {doc['bin_mm']:g}mm, "
                 f"{doc['layers']} layer(s); {lane}")
    lines.append("REPORT-ONLY: nothing gates on these numbers. The boundary "
                 "review dispositions each window below, in writing.")
    for r in hot[:top]:
        w = r['window']
        lines.append(f"  [{w[0]:g},{w[1]:g}]-[{w[2]:g},{w[3]:g}]  demand "
                     f"{r['demand_nets']:>3} net(s) / free "
                     f"{r['free_area_mm2']:g}mm2 = {r['ratio']:g}/mm2")
        lines.append(f"      nets: {', '.join(r['nets'])}"
                     + (" ..." if r['demand_nets'] > len(r['nets']) else ""))
        if r.get('refs'):
            lines.append(f"      refs: {', '.join(r['refs'][:14])}"
                         + (" ..." if len(r['refs']) > 14 else ""))

    if (doc.get('parts') or {}).get('grading_error'):
        # Loudest line in the report, and NOT conditional on the arrangement
        # block: without part geometry every window under a part reads cold,
        # and the ranked pockets are on top of the parts.
        lines.append("  !! PART GEOMETRY UNREADABLE (%s) -- every cold number "
                     "below is measured as if the board had no parts, so the "
                     "pockets it names may be underneath them. Do not act on "
                     "them." % doc['parts']['grading_error'])

    regions = doc.get('cold_regions') or []
    if doc.get('cold_windows') is not None:
        src = (doc.get('outline') or {}).get('source', '?')
        lines.append(f"COLD regions (in-outline, no demand net, no copper, no "
                     f"part courtyard): {len(regions)} region(s), "
                     f"{doc['cold_windows']} window(s); outline from {src}")
        lines.append(f"  of {doc['windows_in_outline']} in-outline window(s): "
                     f"{doc['windows_demand_in_outline']} carry demand, "
                     f"{doc['under_part_windows']} sit under a part, "
                     f"{doc['warm_unowned_windows']} are part-free but carry "
                     f"copper, {doc['cold_windows']} are cold")
        for r in regions[:top]:
            b, br = r['bbox'], r['band_rect']
            lines.append(f"  #{r['rank']:<3} {r['area_mm2']:>8.1f} mm2  "
                         f"{r['windows']:>4} window(s)  band "
                         f"{r['band_mm'][0]:g} x {r['band_mm'][1]:g}mm at "
                         f"[{br[0]:g},{br[1]:g}]-[{br[2]:g},{br[3]:g}]")
            lines.append(f"       bbox [{b[0]:g},{b[1]:g}]-[{b[2]:g},{b[3]:g}]"
                         f"  fill {r['fill']:.2f}"
                         + (f"  bounded by {', '.join(r['refs'][:8])}"
                            if r['refs'] else "")
                         + ("  edge-touching" if r['edge_touch'] else ""))

    arr = doc.get('arrangement')
    if arr:
        s = arr['sides'][arr['headline_side']]
        dx, dy = s['offset_mm']
        fx, fy = s['offset_frac_span']
        cx, cy = s['count_control_frac_span']
        where = ', '.join(
            f"{abs(v):.2f}mm {_compass(*ax)}"
            for v, ax in ((dx, (dx, 0)), (dy, (0, dy))) if abs(v) > 5e-3
        ) or 'ON the board centre'
        lines.append(
            f"arrangement: courtyard-AREA-weighted part centroid on side "
            f"{arr['headline_side']} is {where} of board centre = "
            f"{100 * (fx or 0):.1f}% / {100 * (fy or 0):.1f}% of span "
            f"(part COUNT would say {100 * (cx or 0):.1f}% / "
            f"{100 * (cy or 0):.1f}% -- a control, never the answer)")
        p = doc.get('parts') or {}
        lines.append(f"  parts weighed: {p.get('weighed', 0)} of "
                     f"{p.get('graded', 0)} graded "
                     f"({p.get('from_courtyard')} from a drawn courtyard, "
                     f"{p.get('from_pads')} from pad copper); "
                     f"{p.get('synthetic_excluded', 0)} synthetic excluded, "
                     f"{p.get('container_excluded', 0)} container excluded "
                     f"({p.get('container_guard')})")
        for q in arr['quadrants']:
            lines.append(f"  quadrant {q['name']} (region:q{q['index']})  "
                         f"{q['parts']:>3} part(s) / "
                         f"{q['part_area_mm2']:>8.1f}mm2 / "
                         f"{q['demand_nets']:>4} net-window(s) of demand "
                         f"({q['distinct_nets']} distinct) / "
                         f"{q['cold_windows']:>3} cold")

    rt = doc.get('reseat_target')
    if rt:
        r = rt['zone']
        lines.append(f"reseat target: largest landing site is the "
                     f"{rt['zone_mm'][0]:g} x {rt['zone_mm'][1]:g}mm band at "
                     f"[{r[0]:g},{r[1]:g}]-[{r[2]:g},{r[3]:g}]"
                     + (f"; the mass wants to move {rt['move_mass_toward']}"
                        if rt['move_mass_toward'] else ""))
        lines.append("  This is a DESTINATION, not a scope. A cold band holds "
                     "no part by construction, so `--reseat-region` over it "
                     "resolves to an empty scope on every board.")
        lines.append(f"  use:   declare it as an intent block zone -- the one "
                     f"thing in the stack that AIMS a re-seat at a rectangle: "
                     f'{{"name": "cold_{r[0]:g}_{r[1]:g}", "refs": [...], '
                     f'"zone": [{r[0]:g}, {r[1]:g}, {r[2]:g}, {r[3]:g}]}}')
        if rt.get('refs'):
            lines.append(f"  scope: the parts bounding this pocket are "
                         f"{', '.join(rt['refs'][:10])}"
                         + (' ...' if len(rt['refs']) > 10 else '')
                         + " -- name them, or a CROWDED rectangle, to "
                           "--reseat-region")
    return lines


def main():
    p = argparse.ArgumentParser(
        description="Windowed demand/free-area census of a board.")
    p.add_argument("board")
    p.add_argument("--nets", nargs='+', default=['*'], metavar='GLOB',
                   help="the DEMAND set (route.py glob syntax, '!' excludes). "
                        "Default: every net. Pass the set the route step will "
                        "carry so the census asks the same question")
    p.add_argument("--bin", type=float, default=2.0, metavar='MM',
                   help="window size (default 2.0mm -- about three 0.15/0.15 "
                        "lanes plus a via, the scale run 23's pocket failed "
                        "at; the router's own congestion cost uses 1.0). "
                        "FLOORED AT 0.25mm, which is the census's own floor; "
                        "a smaller value is reported as the floor it was "
                        "raised to, never silently accepted")
    p.add_argument("--top", type=int, default=8, metavar='N',
                   help="how many windows AND cold regions to print "
                        "(default 8)")
    p.add_argument("--threshold", type=float, default=None,
                   metavar='NETS_PER_MM2',
                   help="report only windows above this demand/free-area "
                        "ratio. Default: none -- print the top N whatever "
                        "their ratio, because an absolute threshold is "
                        "exactly what this tool is too young to own")
    p.add_argument("--no-cold", action='store_true',
                   help="suppress the COLD census (in-outline windows with no "
                        "demand, no copper and no part). It is ON by default: "
                        "#709 is that an empty region was structurally "
                        "unreportable, and a census nobody enables does not "
                        "fix that")
    p.add_argument("--cold-cover", type=float, default=0.0, metavar='FRAC',
                   help="largest share of a window a part courtyard may cover "
                        "and still count COLD (default 0.0 -- any courtyard "
                        "touch disqualifies). A false COLD becomes a bad "
                        "reseat target, which is what discredits the "
                        "instrument, so the default is the strict end")
    p.add_argument("--outline-samples", type=int, default=4, metavar='K',
                   help="K x K sub-samples used to measure how much of an "
                        "outline-CUT window is on the board (default 4). "
                        "Ignored on a board whose outline is its bounding "
                        "box, where the overlap is exact")
    p.add_argument("--no-arrangement", action='store_true',
                   help="suppress the arrangement census (courtyard-area-"
                        "weighted centroid offset, per-quadrant part mass "
                        "against per-quadrant demand)")
    p.add_argument("--json", default=None, metavar='PATH',
                   help="write the full census as JSON")
    args = p.parse_args()

    from kicad_parser import parse_kicad_pcb
    try:
        pcb = parse_kicad_pcb(args.board)
    except Exception as exc:                                    # noqa: BLE001
        print(f"cannot parse {args.board}: {exc}", file=sys.stderr)
        return 2

    if args.cold_cover < 0 or args.cold_cover > 1:
        p.error("--cold-cover is a fraction of a window in [0, 1]")
    if args.outline_samples < 1:
        p.error("--outline-samples is a per-axis sample count, at least 1")
    if args.outline_samples > 32:
        # The sweep is K*K per outline-cut window and unbounded above:
        # measured, interf_u_unrouted --bin 0.25 --outline-samples 16 takes
        # 3m50s against 26s at the default and produces IDENTICAL bucket
        # counts. It refines edge-window areas, nothing else.
        p.error("--outline-samples above 32 is quadratic for no measured "
                "gain; it refines edge-window AREAS only, and 16 already "
                "reproduces the default's bucket counts exactly")
    if not math.isfinite(args.bin):
        # `congestion_bins`' max(0.25, bin) passes inf straight through, and
        # the window rectangles then serialise as bare `Infinity`/`NaN` --
        # non-standard JSON that strict parsers refuse, which is the exact
        # thing the `ratio` comment and test_run23_pockets guard against.
        p.error("--bin must be a finite number of millimetres")
    if args.bin <= 0:
        p.error("--bin is a window size in mm; it must be positive "
                "(values under the 0.25mm census floor are raised to it and "
                "reported, but zero and negative are typos)")

    doc, hot = pocket_census(
        pcb, args.board, nets=args.nets, bin_mm=args.bin, top=args.top,
        threshold=args.threshold, cold=not args.no_cold,
        cold_cover=args.cold_cover, outline_samples=args.outline_samples,
        arrangement=not args.no_arrangement)

    if abs(doc['bin_mm'] - args.bin) > 1e-9:
        print(f"  --bin {args.bin:g}mm is below the census floor; binned at "
              f"{doc['bin_mm']:g}mm and reported at {doc['bin_mm']:g}mm",
              file=sys.stderr)

    doc['bin_requested_mm'] = args.bin
    # Only the formatter wants the two floors separately.
    lane = doc['lane_mm']
    doc['_clr'] = doc['_trk'] = None
    if lane is not None:
        try:
            from list_nets import board_floor
            import routing_defaults as defaults
            doc['_clr'], _ = board_floor(args.board, 'clearance', None,
                                         defaults.CLEARANCE)
            doc['_trk'], _ = board_floor(args.board, 'track_width', None,
                                         defaults.TRACK_WIDTH)
        except Exception:                                       # noqa: BLE001
            doc['lane_mm'] = None

    for line in format_report(doc, hot, args.top):
        print(line)

    doc.pop('_clr', None)
    doc.pop('_trk', None)
    print('JSON_SUMMARY: ' + json.dumps(census_scalars(doc), sort_keys=True))

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
        print(f"  JSON -> {args.json}")
    return 0


if __name__ == "__main__":
    import cli_banner
    cli_banner.install()   # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())
