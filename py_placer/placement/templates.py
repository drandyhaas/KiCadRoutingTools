"""Datasheet layout templates: a chip's support parts held where its datasheet draws them.

A switching regulator's datasheet comes with a recommended layout: each
support part's place and turn around the chip. A template
records that arrangement for one chip on one board, as each member's pose in
the chip's own frame -- the chip at the origin, unturned -- so the
arrangement moves and turns with the chip:

    {"anchor": "U2",
     "members": [{"ref": "C2", "at": [-1.82, 1.03], "rot": 90.0}, ...],
     "tolerance_mm": 0.3, "source": "<datasheet, section, figure>"}

An intent declares templates in `templates[]` (`floorplan` loads them).
`place_simple` seats each member at its template pose before the anneal,
carries the members rigidly with every move of the chip, and prices a
member's drift from its pose (`simple.Model`'s template term); the grade's
`template` rule measures the drift on a placed board.

The poses are FITTED to the board's own footprints, not copied from the
datasheet's drawing: a datasheet packs its parts tighter than a board's
courtyards allow. `fit` takes a sketch -- the members in the drawing's
order, nearest the chip first, each with the pads it puts on which pins, as
an intent `proximity[]` row names them -- and seats each member in turn as
close to those pins as the courtyards already seated allow (kept the board's
clearance apart, as the seat gate keeps them), at whichever quarter turn
brings its pads closest.
`py_tools/fit_template.py` is its command line.
"""
import math
import re
from typing import Dict, List, Tuple

import numpy as np

#: Keys of one `templates[]` row and of one member.
TEMPLATE_KEYS = {'anchor', 'members', 'tolerance_mm', 'source', 'note', 'why',
                 'context', 'fitted', 'kind', 'ground_return_mm'}
#: A row's `kind`, as the groups file names its template candidates
#: (`grouping.TEMPLATE_KINDS` but `none`). A `switching stage`'s member caps
#: have their ground return measured (`ground_returns`).
KINDS = ('switching stage', 'rf front end')
MEMBER_KEYS = {'ref', 'at', 'rot', 'note', 'pads'}
#: How far (mm, a member pad's distance from its template position, the worst
#: pad) a member may drift before the grade reports it: room for the anneal's
#: final legality step, which moves a part to its nearest legal pose.
TEMPLATE_TOLERANCE_MM = 0.3
#: How far beyond `gap_mm` (mm) the fit seats each member. The seat gate
#: refuses a courtyard gap under the clearance, and a member fitted at exactly
#: the clearance reads either side of it once its `at` is rounded and the
#: template turned onto the anchor's pose: bread_dcmotor's C1 was refused at
#: its template pose on 5 of 12 seeds and moved 4.75 mm.
FIT_MARGIN_MM = 0.001
#: How far (mm) a switching stage's member cap may return its ground to the
#: chip's (`ground_returns`) before the grade warns; a row's
#: `ground_return_mm` sets its own. Templates fitted with every cap's ground
#: beside the chip's or a like cap's read 0.2-1.9 mm on fusb302_pd_dev,
#: argus, bread_dcmotor, covg_patchclamp and bitaxe; an output cap given only
#: its VOUT partner seated past the inductor at 8.2-10.5 mm (fusb, argus,
#: bitaxe), and covg's input cap at 7.6 mm.
GROUND_RETURN_MM = 3.0
#: How far (mm, centre to centre) round a LOCKED anchor the fit takes the
#: board's other locked parts as obstacles and partners.
LOCKED_CONTEXT_MM = 30.0


class TemplateError(ValueError):
    """A `templates[]` row or a sketch that cannot be read."""


class FitError(TemplateError):
    """A sketch member the fit cannot seat. `partial` is the row of the
    members seated before it, so the arrangement so far can be drawn."""

    def __init__(self, msg, partial):
        super().__init__(msg)
        self.partial = partial


#: A sketch member's `side`: one side of the chip (its centre beyond that
#: courtyard edge, anywhere along it -- a part by a corner pin is still on
#: that pin's side), or two of different axes for a corner (`+x-y`: beyond
#: both edges).
SIDE_RE = re.compile(r'^([+-][xy])([+-][xy])?$')


def side_ok(side, mx, my, ar):
    """Whether a centre (mx, my) -- scalars or arrays -- lies on `side` of the
    anchor courtyard `ar` (x0, y0, x1, y1)."""
    m = SIDE_RE.match(side or '')
    if not m:
        raise TemplateError(f"side {side!r}: one of -x +x -y +y, or two of different "
                            f"axes for a corner (+x-y)")
    a, b = m.group(1), m.group(2)
    out = {'-x': ar[0] - mx, '+x': mx - ar[2], '-y': ar[1] - my, '+y': my - ar[3]}
    if b:
        if a[1] == b[1]:
            raise TemplateError(f"side {side!r}: a corner takes one x side and one y side")
        return (out[a] > 0) & (out[b] > 0)
    return out[a] > 0


def _num(v) -> bool:
    return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)


def validate(raw) -> List[Dict]:
    """The intent's `templates[]` rows, checked for what needs no board:
    shapes, a member named twice (in one template or across two: one part
    cannot sit at two places), a member that is its own or another
    template's anchor. Whether the refs exist is the board's question
    (`template_unresolved` in the grade)."""
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise TemplateError(f"templates: expected a list of rows, got {type(raw).__name__}")
    out, owner, anchors = [], {}, {}
    for i, t in enumerate(raw):
        where = f"templates[{i}]"
        if not isinstance(t, dict):
            raise TemplateError(f"{where}: expected an object with `anchor` and `members`")
        unknown = sorted(set(t) - TEMPLATE_KEYS)
        if unknown:
            raise TemplateError(f"{where}: unknown key(s) {', '.join(unknown)}")
        a = t.get('anchor')
        if not isinstance(a, str) or not a.strip():
            raise TemplateError(f"{where}.anchor: expected a reference, got {a!r}")
        if a in anchors:
            raise TemplateError(f"{where}.anchor: {a} already anchors {anchors[a]}")
        anchors[a] = where
        ms = t.get('members')
        if not isinstance(ms, list) or not ms:
            raise TemplateError(f"{where}.members: expected a non-empty list")
        members = []
        for j, m in enumerate(ms):
            w = f"{where}.members[{j}]"
            if not isinstance(m, dict):
                raise TemplateError(f"{w}: expected an object with `ref`, `at` and `rot`")
            unknown = sorted(set(m) - MEMBER_KEYS)
            if unknown:
                raise TemplateError(f"{w}: unknown key(s) {', '.join(unknown)}")
            r = m.get('ref')
            if not isinstance(r, str) or not r.strip():
                raise TemplateError(f"{w}.ref: expected a reference, got {r!r}")
            if r == a:
                raise TemplateError(f"{w}.ref: {r} is the template's own anchor")
            if r in owner:
                raise TemplateError(f"{w}.ref: {r} is already a member of {owner[r]}")
            owner[r] = where
            at = m.get('at')
            if not (isinstance(at, list) and len(at) == 2 and all(_num(v) for v in at)):
                raise TemplateError(f"{w}.at: expected [dx, dy] in mm, got {at!r}")
            rot = m.get('rot', 0.0)
            if not _num(rot) or abs(rot / 90.0 - round(rot / 90.0)) > 1e-6:
                raise TemplateError(f"{w}.rot: expected a quarter turn (0, 90, 180, 270), "
                                    f"got {rot!r}")
            members.append(dict(m, ref=r, at=[float(at[0]), float(at[1])],
                                rot=float(rot) % 360.0))
        tol = t.get('tolerance_mm', TEMPLATE_TOLERANCE_MM)
        if not _num(tol) or tol < 0:
            raise TemplateError(f"{where}.tolerance_mm: expected a non-negative number, "
                                f"got {tol!r}")
        kind = t.get('kind')
        if kind is not None and kind not in KINDS:
            raise TemplateError(f"{where}.kind: one of {', '.join(KINDS)}, got {kind!r}")
        gr = t.get('ground_return_mm')
        if gr is not None and (not _num(gr) or gr <= 0):
            raise TemplateError(f"{where}.ground_return_mm: expected a positive number, "
                                f"got {gr!r}")
        out.append(dict(t, anchor=a, members=members, tolerance_mm=float(tol)))
    for a, where in anchors.items():
        if a in owner:
            raise TemplateError(f"{where}.anchor: {a} is also a member of {owner[a]}; "
                                f"a template cannot carry another template's anchor")
    return out


def member_target(anchor_pose, member) -> Tuple[float, float, float]:
    """A member's pose for the anchor at `anchor_pose` (x, y, rot): its
    template pose carried from the anchor at the origin, unturned, as the
    placer carries a part (`simple.follow`)."""
    from .simple import follow
    return follow((0.0, 0.0, 0.0), tuple(anchor_pose),
                  (member['at'][0], member['at'][1], member['rot']))


def pad_deviation(part, pose, target) -> Tuple[float, float]:
    """(mean, worst) distance in mm between a part's connected pads at `pose`
    and at `target`: a turn away from the template moves the pads even where
    the origin stays."""
    a = part.pad_globals(*pose)
    b = part.pad_globals(*target)
    if not a:
        d = math.dist(pose[:2], target[:2])
        return d, d
    ds = [math.dist(p[:2], q[:2]) for p, q in zip(a, b)]
    return sum(ds) / len(ds), max(ds)


def measure(pcb, decl) -> Dict:
    """One template row measured on a parsed board: per member its worst
    pad's distance from where the template puts it for the anchor's pose on
    the board (`kicad_parser.local_to_global`, which `pad_globals` is
    bit-identical to), and whether it is on the anchor's side. `unresolved`
    names refs the board does not have."""
    from kicad_parser import local_to_global
    fps = pcb.footprints
    missing = [r for r in [decl['anchor']] + [m['ref'] for m in decl['members']]
               if r not in fps]
    if missing:
        return {'anchor': decl['anchor'], 'state': 'unresolved',
                'reason': f"not on the board: {', '.join(missing)}"}
    A = fps[decl['anchor']]
    apose = (A.x, A.y, (A.rotation or 0.0) % 360.0)
    rows = []
    for m in decl['members']:
        F = fps[m['ref']]
        tx, ty, trot = member_target(apose, m)
        pads = _conn_pads(F) or list(F.pads)
        ds = [math.dist((p.global_x, p.global_y),
                        local_to_global(tx, ty, trot, p.local_x, p.local_y)) for p in pads]
        worst = max(ds) if ds else math.dist((F.x, F.y), (tx, ty))
        mean = sum(ds) / len(ds) if ds else worst
        rows.append({'ref': m['ref'], 'worst_mm': round(worst, 4), 'mean_mm': round(mean, 4),
                     'side_differs': F.layer != A.layer})
    rows.sort(key=lambda r: -r['worst_mm'])
    return {'anchor': decl['anchor'], 'state': 'measured', 'members': rows,
            'tolerance_mm': decl['tolerance_mm']}


# -- a switching stage's ground return ----------------------------------------

def ground_return_rows(pcb, decl, rects_of) -> List[Dict]:
    """Each member capacitor's ground return in a template: from its pad on
    one of the anchor's grounds (`grouping.is_ground`, a net an anchor pad is
    on) to the nearest ground pad of the anchor or of a member transistor on
    that net (a controller's external low-side FET, where its switching
    current returns), the least sum of pad-edge gaps over a chain through
    other members' pads on it -- the copper a surface pour lays to join
    them, as a datasheet's layout shares the input caps' ground with the
    output caps' and the chip's PGND pins. `rects_of(ref)` gives a part's
    connected pads as [(net id, rect)]. A row per measured cap:
    `{ref, net, return_mm, via, to}` (`via`: the members the chain passes,
    in order; `to`: the part it ends on)."""
    import heapq
    from .grouping import is_ground
    from .legality import rect_gap
    from .roles import infer_role
    fps = pcb.footprints
    a = decl['anchor']
    refs = [a] + [m['ref'] for m in decl['members']]
    pads = {r: rects_of(r) for r in refs}
    name = lambda nid: getattr(pcb.nets.get(nid), 'name', '') or ''
    chip_gnd = {nid for nid, _ in pads[a] if is_ground(name(nid))}
    ends_at = {a} | {m['ref'] for m in decl['members']
                     if 'transistor' in infer_role(fps[m['ref']], m['ref'], None)['role']}
    rows = []
    for m in decl['members']:
        r = m['ref']
        own = pads[r]
        if len(own) != 2 or infer_role(fps[r], r, None)['role'] != 'capacitor':
            continue
        ends = [rect for nid, rect in own if nid in chip_gnd]
        if len(ends) != 1:
            continue
        nid = next(n for n, _ in own if n in chip_gnd)
        nodes = [(o, rect) for o in refs for n, rect in pads[o] if n == nid]
        src = next(k for k, (o, rect) in enumerate(nodes) if o == r and rect is ends[0])
        dist, prev, todo = {src: 0.0}, {}, [(0.0, src)]
        best = None
        while todo:
            d, k = heapq.heappop(todo)
            if d > dist.get(k, math.inf):
                continue
            if nodes[k][0] in ends_at:
                best = k
                break
            for j, (o, rect) in enumerate(nodes):
                if j == k or o == r:
                    continue
                nd = d + max(0.0, rect_gap(nodes[k][1], rect))
                if nd < dist.get(j, math.inf) - 1e-12:
                    dist[j], prev[j] = nd, k
                    heapq.heappush(todo, (nd, j))
        via, k = [], prev.get(best)
        while k is not None and k != src:
            if not via or via[-1] != nodes[k][0]:
                via.append(nodes[k][0])
            k = prev.get(k)
        rows.append({'ref': r, 'net': name(nid), 'return_mm': round(dist[best], 3),
                     'via': via[::-1], 'to': nodes[best][0]})
    return rows


def ground_returns(pcb, decl) -> List[Dict]:
    """`ground_return_rows` on the board as placed."""
    from .legality import pad_rect
    return ground_return_rows(pcb, decl, lambda r: [
        (p.net_id, pad_rect(p)) for p in _conn_pads(pcb.footprints[r])])


def ground_returns_fitted(pcb, state, row) -> List[Dict]:
    """`ground_return_rows` on a fitted row's own poses (the anchor at the
    origin, unturned), so the fit can say what the template will hold."""
    poses = {row['anchor']: (0.0, 0.0, 0.0)}
    poses.update({m['ref']: (m['at'][0], m['at'][1], m['rot']) for m in row['members']})

    def rects_of(r):
        fp, part = pcb.footprints[r], state.parts[r]
        part.ensure_rotation(poses[r][2])
        return list(zip((p.net_id for p in _conn_pads(fp)),
                        map(tuple, _pad_rects(part, fp, poses[r]))))
    return ground_return_rows(pcb, row, rects_of)


# -- fitting a sketch to the board's footprints -------------------------------

def _conn_pads(fp):
    """The footprint's connected pads, in `pad_globals` order."""
    return [p for p in fp.pads if p.net_id > 0]


def _legs(pcb, sub, pads):
    """[(subject pad index, [(partner ref, partner pad index)])]: each
    subject pad the sketch names, with the partner pads on its net that it
    names, as `place_simple` reads a `proximity[]` row's `pads`."""
    sconn = _conn_pads(pcb.footprints[sub])
    out = []
    want = {str(x) for x in pads.get(sub, ())}
    for i, q in enumerate(sconn):
        if want and str(q.pad_number) not in want:
            continue
        partners = []
        for other, nums in pads.items():
            if other == sub:
                continue
            on = {str(x) for x in nums}
            for j, o in enumerate(_conn_pads(pcb.footprints[other])):
                if o.net_id == q.net_id and (not on or str(o.pad_number) in on):
                    partners.append((other, j))
        if partners:
            out.append((i, partners))
    return out


def _pad_rects(part, fp, pose):
    """k x 4 array: the part's connected pads' copper rects at `pose`
    (`legality.pad_copper_box`), the box's offset from its pad turned and its
    half extents swapped by a quarter turn from the parsed pose."""
    from .legality import pad_copper_box
    from .simple import _turn
    pts = part.pad_globals(*pose)
    odd = abs((pose[2] - part.orig_rot) % 180.0 - 90.0) < 1e-6
    rows = []
    for (x, y, _n), p in zip(pts, _conn_pads(fp)):
        cx, cy, a, b = pad_copper_box(p)
        if cx != p.global_x or cy != p.global_y:
            ox, oy = _turn(cx - p.global_x, cy - p.global_y, pose[2] - part.orig_rot)
            x, y = x + ox, y + oy
        if odd:
            a, b = b, a
        rows.append((x - a, y - b, x + a, y + b))
    return np.array(rows, dtype=float).reshape(-1, 4)


def fit(pcb, state, sketch, *, step: float = 0.1, fine: float = 0.01,
        reach: float = 4.0) -> Dict:
    """A `templates[]` row fitted from `sketch`:

        {"anchor": "U2", "members": [{"ref": "C2", "pads": {"C2": [..],
          "U2": [..]}, "side"?: "-x", "rot"?: 90}, ...], "gap_mm"?: 0.0,
         "source"?: ..., "tolerance_mm"?: ...}

    The anchor sits at the origin, unturned. Each member in the sketch's
    order is seated where the sum over its legs of the pad-edge gap to the
    nearest partner pad (the anchor's pins, or a member seated before it) is
    least, its courtyard (`part.rect`) more than `gap_mm` (default: the board's
    clearance, the seat gate's courtyard rule; `FIT_MARGIN_MM` more) from what
    each part already seated presents on the faces they share (its courtyard
    on its own face, its drilled pads' boxes on the other, as the seat gate
    pairs them: a member on the far face of a through-hole chip sits clear of
    its leads, under its body if need be). It is seated trying
    each quarter turn (or the sketch's `rot`) and, with `side`, only centres
    on that side of the anchor (`side_ok`). A coarse grid
    (`step`) within `reach` mm of its partners, then a fine one round the
    best (a member's own `reach` overrides it); ties go to the pose nearer
    the anchor. A LOCKED anchor's locked neighbours are where the board has
    them (`fitted.context`): obstacles like the members seated, and partners
    a member may name (an antenna's feed). The row carries each member's
    fitted legs and its gap to the anchor (`fitted`), and `fitted.notes`
    names a member with one side that ended past a corner, so the result can
    be checked against the drawing. A member that cannot be seated raises
    `FitError`, saying whether its side or the parts seated before it (named)
    stood in the way, with the row so far."""
    from .legality import rect_gap
    from .simple import ROTS
    a = sketch.get('anchor')
    fps, parts = pcb.footprints, state.parts
    if a not in parts:
        raise TemplateError(f"anchor {a!r} is not on the board")
    # the seat gate's courtyard clearance (`QuenchState.candidate_valid`),
    # so a member at its template pose is legal beside the others
    gap = float(sketch.get('gap_mm', state.clearance))
    A = parts[a]
    a_side = getattr(A, 'side', None)
    for r in ROTS:
        A.ensure_rotation(r)
    poses = {a: (0.0, 0.0, 0.0)}
    seated = [A.rect(0.0, 0.0, 0.0)]
    # what each seated part presents on each face, for the face-aware test:
    # its courtyard on its own face, its drilled pads' boxes on the other
    occ = [(a, A, (0.0, 0.0, 0.0))]
    ar = seated[0]
    ac = ((ar[0] + ar[2]) / 2.0, (ar[1] + ar[3]) / 2.0)
    context = _locked_context(pcb, state, sketch, a, a_side)
    for o, rel in context.items():
        Q = parts[o]
        Q.ensure_rotation(rel[2])
        poses[o] = rel
        seated.append(Q.rect(*rel))
        occ.append((o, Q, rel))
    members, fitted, each, off, notes = [], {}, {}, {}, []
    for k, m in enumerate(sketch.get('members') or []):
        r = m.get('ref')
        where = f"sketch members[{k}] ({r})"
        if r not in parts or r == a:
            raise TemplateError(f"{where}: not a part of the board other than the anchor")
        if r in poses:
            raise TemplateError(f"{where}: named twice")
        P = parts[r]
        pads = m.get('pads') or {}
        if r not in pads:
            raise TemplateError(f"{where}: `pads` must name {r}'s own pads and its partners'")
        legs = _legs(pcb, r, pads)
        for _i, partners in legs:
            for o, _j in partners:
                if o not in poses:
                    raise TemplateError(f"{where}: partner {o} is not seated yet; list it "
                                        f"before {r}")
        if not legs:
            raise TemplateError(f"{where}: none of its named pads shares a net with a named "
                                f"partner pad")
        prects = {o: _pad_rects(parts[o], fps[o], poses[o]) for o in pads if o in poses}
        targets = [prects[o][j] for _i, ps in legs for o, j in ps]
        cx = float(np.mean([(t[0] + t[2]) / 2 for t in targets]))
        cy = float(np.mean([(t[1] + t[3]) / 2 for t in targets]))
        rots = [float(m['rot']) % 360.0] if 'rot' in m else list(ROTS)
        reach_m = float(m.get('reach', reach))
        side = m.get('side')
        if side:
            side_ok(side, 0.0, 0.0, ar)     # refuses a malformed side
        def blockers(rot):
            """[(member box at the origin, obstacle box)] over the faces the
            member shares with each seated part, as the seat gate pairs them
            (`legality.pair_min_gap`)."""
            from .legality import far_boxes, rect_on
            mc, mf = P.rects(0.0, 0.0, rot)
            out = []
            for _o, Q, qp in occ:
                Q.ensure_rotation(qp[2])
                qc, qf = Q.rects(*qp)
                for face in P.sides & Q.sides:
                    mb, qb = rect_on(face, P.side, mc, mf), rect_on(face, Q.side, qc, qf)
                    if mb is not None and qb is not None:
                        out += [(b1, b2) for b1 in far_boxes(mb) for b2 in far_boxes(qb)]
            return out

        def search(clear=True, sided=True):
            """(sum of leg gaps + tie-break, x, y, rot, legs) of the best
            origin, over the coarse grid and then a fine one round its best;
            None when no candidate passes. `clear` False drops the courtyard
            test and `sided` False the side, to say which one blocked."""
            best = None
            for rot in rots:
                P.ensure_rotation(rot)
                base = P.rect(0.0, 0.0, rot)
                prect0 = _pad_rects(P, fps[r], (0.0, 0.0, rot))
                pairs = blockers(rot)
                span = max(base[2] - base[0], base[3] - base[1]) / 2.0 + reach_m

                def score(X, Y):
                    """Leg gaps, feasibility and the tie-break, over arrays
                    of candidate origins."""
                    ok = np.ones(X.size, dtype=bool)
                    r0, r1 = base[0] + X, base[1] + Y
                    r2, r3 = base[2] + X, base[3] + Y
                    if clear:
                        for b, q in pairs:  # the seat gate's courtyard test: Euclidean gap
                            gx = np.maximum(0.0, np.maximum(q[0] - (b[2] + X), (b[0] + X) - q[2]))
                            gy = np.maximum(0.0, np.maximum(q[1] - (b[3] + Y), (b[1] + Y) - q[3]))
                            ok &= np.hypot(gx, gy) >= gap + FIT_MARGIN_MM
                    mx, my = (r0 + r2) / 2.0, (r1 + r3) / 2.0
                    if side and sided:
                        ok &= side_ok(side, mx, my, ar)
                    tot = np.zeros(X.size)
                    for i, ps in legs:
                        q = prect0[i]
                        g = None
                        for o, j in ps:
                            t = prects[o][j]
                            gx = np.maximum(0.0, np.maximum(t[0] - (q[2] + X), (q[0] + X) - t[2]))
                            gy = np.maximum(0.0, np.maximum(t[1] - (q[3] + Y), (q[1] + Y) - t[3]))
                            d = np.hypot(gx, gy)
                            g = d if g is None else np.minimum(g, d)
                        tot += g
                    tie = 1e-3 * np.hypot(mx - ac[0], my - ac[1])
                    return np.where(ok, tot + tie, np.inf), tot

                gx = np.arange(cx - span, cx + span + step / 2, step)
                gy = np.arange(cy - span, cy + span + step / 2, step)
                X, Y = np.meshgrid(gx, gy)
                # candidate ORIGINS: the grid places the rect's centre; shift
                # by its offset from the origin
                ox, oy = (base[0] + base[2]) / 2.0, (base[1] + base[3]) / 2.0
                X, Y = X.ravel() - ox, Y.ravel() - oy
                sc, _t = score(X, Y)
                i0 = int(np.argmin(sc))
                if not math.isfinite(sc[i0]):
                    continue
                fx = np.arange(X[i0] - step, X[i0] + step + fine / 2, fine)
                fy = np.arange(Y[i0] - step, Y[i0] + step + fine / 2, fine)
                FX, FY = np.meshgrid(fx, fy)
                FX, FY = FX.ravel(), FY.ravel()
                s2, t2 = score(FX, FY)
                i1 = int(np.argmin(s2))
                cand = (float(s2[i1]), float(FX[i1]), float(FY[i1]), rot, float(t2[i1]))
                if best is None or cand[0] < best[0] - 1e-9:
                    best = cand
            return best

        best = search()
        if best is None:
            why = f"{where}: no pose within {reach_m:g} mm of its partners"
            if side and search(sided=False) is not None:
                why += (f" on side {side}; it seats off that side -- change the side, or "
                        f"list it before the members that fill that side")
            else:
                from .legality import pair_min_gap
                _s, x, y, rot, _t = search(clear=False, sided=False)
                P.ensure_rotation(rot)
                wc, wf = P.rects(x, y, rot)
                taken = []
                for o, Q, qp in occ:
                    Q.ensure_rotation(qp[2])
                    qc, qf = Q.rects(*qp)
                    g = pair_min_gap(P.sides, P.side, wc, wf, Q.sides, Q.side, qc, qf)
                    if g is not None and g < gap:
                        taken.append(o)
                why += (f" clears the parts seated before it: the spot nearest its partners "
                        f"is taken by {', '.join(taken) or 'nothing (raise --reach)'}. List it "
                        f"before them, make a like part already seated its partner, or "
                        f"raise --reach")
            raise FitError(why, _row(sketch, a, members, fitted, gap, each, off, notes, context))
        _s, x, y, rot, tot = best
        x, y = round(x, 4), round(y, 4)
        poses[r] = (x, y, rot)
        P.ensure_rotation(rot)
        seated.append(P.rect(x, y, rot))
        occ.append((r, P, (x, y, rot)))
        mine = _pad_rects(P, fps[r], (x, y, rot))
        each[r] = [round(max(0.0, min(rect_gap(mine[i], prects[o][j]) for o, j in ps)), 3)
                   for i, ps in legs]
        q = seated[-1]
        off[r] = round(max(0.0, rect_gap(q, ar)), 3)
        if side and len(side) == 2:     # one side: say so when it ended beside another
            mx, my = (q[0] + q[2]) / 2.0, (q[1] + q[3]) / 2.0
            out = {'-x': ar[0] - mx, '+x': mx - ar[2], '-y': ar[1] - my, '+y': my - ar[3]}
            across = max((k for k in (('-x', '+x') if side[1] == 'y' else ('-y', '+y'))),
                         key=lambda k: out[k])
            if out[across] > out[side]:
                notes.append(f"{r} (side {side}) sits further past the chip's {across} edge "
                             f"than its {side} one: beside the {across} edge, at a corner")
        members.append(dict({'ref': r, 'at': [x, y], 'rot': rot, 'pads': pads},
                            **({'note': m['note']} if 'note' in m else {})))
        fitted[r] = round(tot, 3)
    row = _row(sketch, a, members, fitted, gap, each, off, notes, context)
    validate([row])
    return row


def _locked_context(pcb, state, sketch, anchor, a_side) -> Dict:
    """{ref: pose in the anchor's frame} of the locked parts round a LOCKED
    anchor, its members excepted: within LOCKED_CONTEXT_MM, containers left
    out (the fit pairs faces as the seat gate does). A movable anchor has
    none -- where it lands decides where they fall."""
    from .simple import follow
    parts, fps = state.parts, pcb.footprints
    if not getattr(parts[anchor], 'locked', False):
        return {}
    F = fps[anchor]
    apose = (F.x, F.y, (F.rotation or 0.0) % 360.0)
    names = {m.get('ref') for m in sketch.get('members') or ()}
    # a container (a pin frame round the board) holds parts, as the seat gate
    # lets it (`QuenchState.container_refs`)
    names |= set(getattr(state, 'container_refs', ()) or ())
    out = {}
    for o, Q in parts.items():
        if o == anchor or o in names or not getattr(Q, 'locked', False):
            continue
        G = fps[o]
        rel = follow(apose, (0.0, 0.0, 0.0), (G.x, G.y, (G.rotation or 0.0) % 360.0))
        if math.hypot(rel[0], rel[1]) <= LOCKED_CONTEXT_MM:
            out[o] = (round(rel[0], 4), round(rel[1], 4), rel[2] % 360.0)
    return out


def _row(sketch, anchor, members, fitted, gap, each, off, notes, context) -> Dict:
    """The `templates[]` row of the members fitted so far: `fitted` carries
    each member's legs summed (`legs_gap_mm`) and one by one (`legs_mm`, in
    the order of its named pads), its courtyard gap to the anchor's
    (`off_chip_mm`), the fit's `notes`, and a locked anchor's locked
    neighbours as the fit saw them (`context`)."""
    row = {'anchor': anchor, 'members': list(members),
           'tolerance_mm': float(sketch.get('tolerance_mm', TEMPLATE_TOLERANCE_MM)),
           'fitted': {'legs_gap_mm': dict(fitted), 'legs_mm': dict(each),
                      'off_chip_mm': dict(off), 'gap_mm': gap}}
    if notes:
        row['fitted']['notes'] = list(notes)
    if context:
        row['fitted']['context'] = {o: list(p) for o, p in context.items()}
    for k in ('kind', 'ground_return_mm', 'source', 'note', 'why', 'context'):
        if k in sketch:
            row[k] = sketch[k]
    return row


# -- what the agent reads and looks at -----------------------------------------

#: How near (mm) a pad's centre must be to the outermost pads' line to lie
#: on it: a pin row's centres agree to well under this, and an exposed pad's
#: pieces sit a pin's length or more inside.
PIN_LINE_MM = 0.15

#: More parts than this on a pin's net: a ground or a board-wide supply, shown
#: as a count rather than listed.
RAIL_PARTS = 20


def _exposed_pads(pads) -> set:
    """ids of an exposed pad's pieces: the pads of a number several pads
    share, and the largest pad when it is over twice the next, whose centres
    lie inside the box of the other pads' centres. A pin number repeated
    round the edge (GND pins) lies on that box, not inside it."""
    from collections import Counter
    n = Counter(str(p.pad_number) for p in pads)
    cand = {id(p) for p in pads if n[str(p.pad_number)] > 1}
    by_area = sorted(pads, key=lambda p: -(p.size_x * p.size_y))
    if len(by_area) > 1 and by_area[0].size_x * by_area[0].size_y > \
            2.0 * by_area[1].size_x * by_area[1].size_y:
        cand.add(id(by_area[0]))
    rest = [p for p in pads if id(p) not in cand]
    if len(rest) < 2:
        return set()
    x0, x1 = min(p.local_x for p in rest), max(p.local_x for p in rest)
    y0, y1 = min(p.local_y for p in rest), max(p.local_y for p in rest)
    return {id(p) for p in pads if id(p) in cand
            and x0 < p.local_x < x1 and y0 < p.local_y < y1}


def pin_table(pcb, ref, group=None) -> List[Dict]:
    """One row per pad of `ref`, for matching a datasheet's drawing to the
    board: its number and pin function, its position in the footprint's own
    frame (the frame a template is in) and which side of the footprint that
    is (`-x`, `+x`, `-y`, `+y`; `centre` for an exposed pad, `inner -x` and
    so on for an inner pad off the middle), its net, and
    the other parts on that net with their values and pads (`rail` with a
    count when more than RAIL_PARTS share it: a ground or a board-wide
    supply; with `group`, a set of refs, a rail's parts in that group are
    still listed -- a buck's output caps on the 3V3 rail). A pad's side is the outer row or column of pads it lies on,
    a corner pad's the longer of its two lines; an inner pad's is where the
    pieces of its number lie together (`_exposed_pads`)."""
    fp = pcb.footprints[ref]
    # a pad with neither a number nor a net is paste or thermal art
    pads = [p for p in fp.pads if str(p.pad_number).strip() or p.net_id]
    if not pads:
        return []
    exposed = _exposed_pads(pads)
    ring = [p for p in pads if id(p) not in exposed] or pads
    xs = [p.local_x for p in ring]
    ys = [p.local_y for p in ring]
    edge = {'-x': min(xs), '+x': max(xs), '-y': min(ys), '+y': max(ys)}
    # a single row (or column) of pads has no y (or x) lines to lie on
    live = [k for k in edge if max(xs if k[1] == 'x' else ys)
            - min(xs if k[1] == 'x' else ys) > PIN_LINE_MM]

    def lines(q):
        return [k for k in live
                if abs((q.local_x if k[1] == 'x' else q.local_y) - edge[k]) < PIN_LINE_MM]
    on = {id(q): [] if id(q) in exposed else lines(q) for q in pads}
    count = {k: sum(k in v for v in on.values()) for k in live}
    # an inner pad off the middle (a HotRod package's SW pad) says which way:
    # by the centre of all the pieces of its number
    mx, my = (edge['-x'] + edge['+x']) / 2.0, (edge['-y'] + edge['+y']) / 2.0
    hx = max(1e-6, (edge['+x'] - edge['-x']) / 2.0)
    hy = max(1e-6, (edge['+y'] - edge['-y']) / 2.0)

    def inner(q):
        sib = [o for o in pads if id(o) in exposed and str(o.pad_number) == str(q.pad_number)]
        nx = (sum(o.local_x for o in sib) / len(sib) - mx) / hx
        ny = (sum(o.local_y for o in sib) / len(sib) - my) / hy
        if max(abs(nx), abs(ny)) < 0.25:
            return 'centre'
        if abs(nx) >= abs(ny):
            return 'inner ' + ('+x' if nx > 0 else '-x')
        return 'inner ' + ('+y' if ny > 0 else '-y')
    rows = []
    for p in sorted(pads, key=lambda q: (len(str(q.pad_number)), str(q.pad_number))):
        ls = on[id(p)]
        side = (max(ls, key=lambda k: count[k]) if ls else
                inner(p) if id(p) in exposed else 'centre')
        others = []
        if p.net_id and p.net_id in pcb.nets:
            others = sorted({q.component_ref for q in pcb.nets[p.net_id].pads
                             if q.component_ref and q.component_ref != ref})
        row = {'pad': str(p.pad_number), 'function': p.pinfunction or '',
               'at': [round(p.local_x, 3), round(p.local_y, 3)], 'side': side,
               'net': p.net_name or ''}
        if len(others) > RAIL_PARTS:
            row['rail'] = len(others)
            others = [o for o in others if group and o in group]
        if others or 'rail' not in row:
            row['parts'] = [{'ref': o, 'value': getattr(pcb.footprints.get(o), 'value', '') or '',
                             'layer': getattr(pcb.footprints.get(o), 'layer', ''),
                             'pads': [str(q.pad_number) for q in pcb.footprints[o].pads
                                      if q.net_id == p.net_id]}
                            for o in others if o in pcb.footprints]
        rows.append(row)
    return rows


def _nearest(a, b):
    """The nearest points of rects a and b: where a leg's pad-edge gap is."""
    def axis(a0, a1, b0, b1):
        if a1 < b0:
            return a1, b0
        if b1 < a0:
            return a0, b1
        m = (max(a0, b0) + min(a1, b1)) / 2.0
        return m, m
    (ax, bx), (ay, by) = axis(a[0], a[2], b[0], b[2]), axis(a[1], a[3], b[1], b[3])
    return (ax, ay), (bx, by)


def render(pcb, state, row, path, scale: float = 60.0, boxed=()) -> None:
    """A PNG of a fitted template in the anchor's own frame (the chip at the
    origin, unturned, +y down as on the board): every courtyard, every pad
    with its number, each member's reference, and each leg drawn across its
    pad-edge gap to the nearest partner pad (as the fit measures it) --
    to set beside the datasheet's drawing. A part in `boxed` has no
    courtyard: its box is its pads', which is all the fit and the placer
    know of its body, and is labelled so. A locked anchor's locked
    neighbours (`fitted.context`) are drawn in grey where the board has
    them."""
    from PIL import Image, ImageDraw
    fps, parts = pcb.footprints, state.parts
    context = {o: tuple(p) for o, p in ((row.get('fitted') or {}).get('context') or {}).items()
               if o in parts}
    poses = {row['anchor']: (0.0, 0.0, 0.0)}
    poses.update(context)
    for m in row['members']:
        poses[m['ref']] = (m['at'][0], m['at'][1], m['rot'])
    rects, pads = {}, {}
    for r, pose in poses.items():
        P = parts[r]
        P.ensure_rotation(pose[2])
        rects[r] = P.rect(*pose)
        pads[r] = _pad_rects(P, fps[r], pose)
    allr = list(rects.values()) + [tuple(q) for v in pads.values() for q in v]
    x0 = min(q[0] for q in allr) - 1.0
    y0 = min(q[1] for q in allr) - 1.0
    x1 = max(q[2] for q in allr) + 1.0
    y1 = max(q[3] for q in allr) + 1.0
    W, H = int((x1 - x0) * scale), int((y1 - y0) * scale)
    img = Image.new('RGB', (W, H), (250, 250, 247))
    d = ImageDraw.Draw(img)

    def px(x, y):
        return ((x - x0) * scale, (y - y0) * scale)

    for r, q in rects.items():
        anchor = r == row['anchor']
        d.rectangle([px(q[0], q[1]), px(q[2], q[3])],
                    outline=(40, 40, 40) if anchor else (160, 160, 160) if r in context
                    else (90, 110, 160), width=3 if anchor else 2)
        if r in context:
            d.text(px(q[0] + 0.1, q[1] + 0.1), f"{r} (locked)", fill=(120, 120, 120))
    for r, P in pads.items():
        anchor = r == row['anchor']
        # largest first, so a big pad (a FET's drain) never hides the small ones
        drawn = sorted(zip(P, _conn_pads(fps[r])),
                       key=lambda e: -(e[0][2] - e[0][0]) * (e[0][3] - e[0][1]))
        for q, p in drawn:
            d.rectangle([px(q[0], q[1]), px(q[2], q[3])],
                        fill=(196, 160, 60) if anchor else (200, 200, 200) if r in context
                        else (170, 140, 70), outline=(120, 90, 30))
        for q, p in drawn:
            cxp, cyp = px((q[0] + q[2]) / 2, (q[1] + q[3]) / 2)
            d.text((cxp - 4, cyp - 6), str(p.pad_number),
                   fill=(20, 20, 20) if anchor else (90, 30, 30))
    for m in row['members']:
        legs = _legs(pcb, m['ref'], m.get('pads') or {}) if m.get('pads') else []
        for i, partners in legs:     # the nearest partner pad, as the fit measures
            a = pads[m['ref']][i]
            ends = [_nearest(a, pads[o][j]) for o, j in partners
                    if o in pads and j < len(pads[o])]
            if not ends:
                continue
            pa, pb = min(ends, key=lambda e: math.dist(*e))
            d.line([px(*pa), px(*pb)], fill=(220, 40, 40), width=3)
            for pt in (pa, pb):
                u, v = px(*pt)
                d.ellipse([u - 3, v - 3, u + 3, v + 3], fill=(220, 40, 40))
        q = rects[m['ref']]
        cxp, cyp = px((q[0] + q[2]) / 2, q[1])
        d.text((cxp - 8, cyp - 14), m['ref'] + (' (no courtyard)' if m['ref'] in boxed else ''),
               fill=(20, 40, 120))
    q = rects[row['anchor']]
    back = getattr(fps[row['anchor']], 'layer', 'F.Cu') == 'B.Cu'
    d.text(px(q[0], q[1] - 0.6), f"{row['anchor']} (origin, unturned; +x right, +y down)"
           + (' (no courtyard)' if row['anchor'] in boxed else '')
           + (' -- BACK face: its own frame, mirrored from the top view' if back else ''),
           fill=(20, 20, 20))
    img.save(path)
