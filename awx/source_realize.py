#!/usr/bin/env python3
"""Realize the plan's SOURCE moves with the production fanout engine, and
audit every tooth at EITHER end: ORIGINAL (the copper on the board
before) vs ASKED (the paper move the plan chose) vs ACHIEVED (the copper
the engine laid).

The engine takes one hint per ball -- the face to leave by -- and picks
the channel, the layer and the exit gap itself, so a hint accepted is
not a move achieved. Every realization therefore measures each tooth off
the WRITTEN board (the free end of the net's copper at that array, the
layer it ends on, its vias, its kind) and reports it against the ask,
per tooth and as an ORDER: the plan's crossing floor is a statement
about the order of exits along a face, so two teeth that both left by
the asked face but swapped gaps have changed what the braid will see. A
ball the engine refuses keeps its original copper (restored, said so)
rather than being stranded. A realized source board that is not
DRC-clean is rejected with its pairs printed; the caller keeps the
previous board.
"""
import math
import os
import re
import awx_settings
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import (add_tracks_and_vias_to_pcb,  # noqa: E402
                          remove_segments_from_content,
                          remove_vias_from_content)
import ship_vias  # noqa: E402  a via in a pad declares Type VII (#962)
from pcb_modification import remove_net_from_pcb_data  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402

from escape_moves import DIRS  # noqa: E402,F401  -- ONE source
import route_layers  # noqa: E402  the routing layers: a re-fan's runs may take any of them
GAP_TOL = 0.2      # mm: an achieved tooth this close to the asked exit is the same gap


def move_sig(m):
    """The identity of a menu move for the plan's feasibility ledger.

    THE LEGS ARE PART OF THE IDENTITY (2026-09-17 review). Without them
    this could not tell apart the very lanes `own_line` exists to create:
    a via-in-pad climb's three lanes (straight, and one either side) all
    reach the SAME exit point from the SAME site and differ only in the
    copper between -- so all three hashed to one signature. Measured on
    the bench's U1, 3216 of 11602 enumerated moves (28%) collapsed that
    way, always in groups of three.

    Legs are rounded to the same 2 dp as `exit_pt` and `site`, and are
    derived from grid positions, so the identity is stable across
    re-enumeration.
    """
    return (m.kind, m.direction, m.layer, round(m.exit_pt[0], 2),
            round(m.exit_pt[1], 2),
            (round(m.site[0], 2), round(m.site[1], 2)) if m.site else None,
            tuple((round(a[0], 2), round(a[1], 2),
                   round(b[0], 2), round(b[1], 2), L) for a, b, L in (m.legs or ())))


def move_class(m):
    """A menu move's CLASS: its signature without the site and legs, so every
    leg variant of one exit is one. A destination berth the fanout did not
    lay as asked is banned by class (fanout_from_plan.ban_moves, under
    PLAN_JUDGE=ends): its variants reach the same exit through the same
    neighbourhood, and the engine refuses them for the same reason."""
    return (m.kind, m.direction, m.layer, round(m.exit_pt[0], 2), round(m.exit_pt[1], 2))


# The engine lays at these (realize's own generate_bga_fanout call); the
# blocker census must use the SAME numbers or it names the wrong nets.
# Defaults from rules.py, resolved per board by the stage that installs
# (see rules.py, "USING IT"); the engine call below reads them at call time.
import rules as _rules            # noqa: E402  ONE source for every design rule
FAN_TRACK = _rules.active().fan_track
FAN_CLEAR = _rules.active().fan_clear


def _seg_point_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _seg_seg_dist(a, b, c, d, n=12):
    """Sampled segment-to-segment distance -- enough to decide whether two
    escapes contend for the same room (this picks WHICH nets to re-fan, not
    whether copper is legal; check_drc grades that)."""
    best = 1e9
    for i in range(n + 1):
        t = i / n
        px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        best = min(best, _seg_point_dist(px, py, c[0], c[1], d[0], d[1]))
        qx, qy = c[0] + t * (d[0] - c[0]), c[1] + t * (d[1] - c[1])
        best = min(best, _seg_point_dist(qx, qy, a[0], a[1], b[0], b[1]))
    return best


def moves_clash(a, b):
    """Do two menu Moves of DIFFERENT nets contend for the same room?

    Their legs where they share a layer, either one's via site against the
    other's legs on EVERY layer (a barrel pierces both), and the two sites
    against each other. `blockers_of` asks the same question of a move and
    the copper that stands on the board; this asks it of two moves NEITHER
    of which is on the board yet, which is what a GROUP move needs: the
    members are enumerated with each other's teeth stripped, so nothing in
    the obstacle map can see them and the engine meets the collision only
    when it lays them (K51: ten climbs asked through five column gaps, half
    of them degraded). Same numbers as `blockers_of`, so the two agree."""
    d_seg = FAN_TRACK + FAN_CLEAR
    d_via = te.VIA_SIZE / 2 + FAN_CLEAR + FAN_TRACK / 2
    d_vv = te.VIA_SIZE + FAN_CLEAR
    for (p, q, L) in (a.legs or ()):
        for (r, t, M) in (b.legs or ()):
            if L == M and _seg_seg_dist(p, q, r, t) < d_seg:
                return True
    for m, other in ((a, b), (b, a)):
        if m.site is None:
            continue
        for (p, q, _L) in (other.legs or ()):
            if _seg_point_dist(m.site[0], m.site[1], p[0], p[1], q[0], q[1]) < d_via:
                return True
    if a.site is not None and b.site is not None:
        if math.hypot(a.site[0] - b.site[0], a.site[1] - b.site[1]) < d_vv:
            return True
    return False


def blockers_of(pcb, move, nid, byname, pool):
    """The nets whose copper stands in the room `move` needs -- its legs on
    their own layers, and its via site on EVERY layer (a barrel pierces
    both). These are the escapes a one-net re-fan cannot touch, which is
    why the engine degrades the ask instead of laying it: `_follow_plan`
    can rip and re-lay a blocker, but only one that is IN THE SAME CALL
    (K41 SA11: asked a dogbone under U1, laid `level 3 lost
    ['face','gap','layer','kind'] = original`).

    Returns (movable, pinned): nets of `pool` that may be re-fanned with
    it, and blockers OUTSIDE the pool, which nothing in this chain can
    move (the K35 climb was walled by SA14, a net not in the run)."""
    id2nm = {v[0]: k for k, v in byname.items()}
    legs = [(tuple(a), tuple(b), L) for (a, b, L) in (move.legs or ())]
    site = tuple(move.site) if move.site else None
    d_seg = FAN_TRACK / 2 + FAN_CLEAR + FAN_TRACK / 2
    d_via = te.VIA_SIZE / 2 + FAN_CLEAR + FAN_TRACK / 2
    d_vv = te.VIA_SIZE / 2 + FAN_CLEAR + te.VIA_SIZE / 2
    hit = set()

    def take(other_id):
        if other_id and other_id != nid:
            hit.add(other_id)
    for sg in pcb.segments:
        if sg.net_id == nid or not sg.net_id:
            continue
        a, b = (sg.start_x, sg.start_y), (sg.end_x, sg.end_y)
        for (p, q, L) in legs:
            if sg.layer == L and _seg_seg_dist(p, q, a, b) < d_seg + sg.width / 2:
                take(sg.net_id)
        if site is not None and _seg_point_dist(site[0], site[1], a[0], a[1],
                                                b[0], b[1]) < d_via + sg.width / 2:
            take(sg.net_id)          # the barrel pierces every layer
    for v in pcb.vias:
        if v.net_id == nid or not v.net_id:
            continue
        for (p, q, _L) in legs:
            if _seg_point_dist(v.x, v.y, p[0], p[1], q[0], q[1]) < d_via + v.size / 2 - te.VIA_SIZE / 2:
                take(v.net_id)
        if site is not None and math.hypot(v.x - site[0], v.y - site[1]) < d_vv:
            take(v.net_id)
    names = {id2nm[i] for i in hit if i in id2nm}
    movable = sorted(n for n in names if n in pool)
    pinned = sorted(n for n in names if n not in pool)
    return movable, pinned


def full_move(m):
    """The plan's Move as the engine's FULL hint: face, exit point (its
    coordinate along the face is the gap), layer, kind, dog-bone site."""
    d = {'face': m.direction, 'exit': tuple(m.exit_pt), 'layer': m.layer,
         'kind': m.kind, 'site': (tuple(m.site) if m.site else None)}
    if awx_settings.get('PLAN_PAGES', '0') not in ('', '0') and m.legs:
        # the pages-first plan's berths are laid verbatim: the move's own
        # legs, laid by underpad.attempt before its search -- the engine's
        # "exact" is only the exact EXIT, and a stub audited exact once ran
        # four rows down the neighbouring gap, taking the lane two other
        # asks held (measured K41 122 -> 96 with the planner's own
        # conflict test made complete)
        d['legs'] = [(tuple(a), tuple(b), L) for (a, b, L) in m.legs]
    # a dog-bone whose via stands PAST its ball's diagonal cells (a stub along the ball's own line, or one that bends
    # into a gap first) carries that stub as the engine's walked `path` [ball, elbow, .., site]: the engine takes a
    # caller's site only from an adjacent cell or reached by such a path (underpad._dogbone_path_valid), and otherwise
    # put the via in a diagonal cell of its own and walked the run from there (K15 SDQ15, a via planned 1.6 mm into
    # DU1's empty band: laid at the diagonal, its B run wound 3 mm round the balls and shut SDQS1N out). A plain
    # dog-bone's stub is ONE 45-degree leg into its diagonal cell, and carries none
    if m.kind == 'dogbone' and m.site is not None and m.legs:
        stub = [(a, b) for (a, b, L) in m.legs if L != m.layer]
        if stub and math.hypot(m.site[0] - stub[-1][1][0], m.site[1] - stub[-1][1][1]) < 1e-6 and \
                (len(stub) > 1 or min(abs(stub[0][1][0] - stub[0][0][0]), abs(stub[0][1][1] - stub[0][0][1])) < 1e-6):
            d['path'] = [tuple(stub[0][0])] + [tuple(b) for (_a, b) in stub]
    return d


def snap_dir(dx, dy):
    h = math.hypot(dx, dy) or 1.0
    return min(DIRS, key=lambda k: (DIRS[k][0] - dx / h) * (DIRS[k][0] - dx / h)
               + (DIRS[k][1] - dy / h) * (DIRS[k][1] - dy / h))


def measure_tooth(pcb, nm, pad, byname, dest_ref=None, which=None):
    """What the board says about this net's escape at the array `pad`
    belongs to: the free end, the layer that end is on, the vias near the
    ball, and the kind/face that copper amounts to. `dest_ref` names the
    destination array when both ends are fanned out (te.endpoints then
    attributes the two free ends by walking the copper); the DESTINATION
    end is measured then, unless `which='src'` asks for the source end
    of that same attribution (replan.py measures both ends of a board
    fanned out at both). A net with no free end -- bare, or fully
    routed -- measures as None: the callers print '(bare)' and audit it
    as 'no copper' (replan.py re-fans a net it stripped to nothing)."""
    nid = byname[nm][0]
    try:
        ends = te.endpoints(pcb, [nm], byname, dest_ref=dest_ref)
    except AssertionError:
        return None
    tooth = (ends[nm][1] if dest_ref is not None and which != 'src'
             else ends[nm][0])
    last = next((s for s in pcb.segments if s.net_id == nid
                 and (abs(s.start_x - tooth[0]) + abs(s.start_y - tooth[1]) < 0.005
                      or abs(s.end_x - tooth[0]) + abs(s.end_y - tooth[1]) < 0.005)),
                None)
    layer = last.layer if last else '?'
    # the FACE is the direction the copper leaves by: the last segment's
    # outward direction. The bearing ball -> tooth misreads a long lateral
    # run that exits the south face 5 mm west of its ball as 'left'.
    if last:
        other = ((last.start_x, last.start_y)
                 if abs(last.end_x - tooth[0]) + abs(last.end_y - tooth[1]) < 0.005
                 else (last.end_x, last.end_y))
        seg_dir = snap_dir(tooth[0] - other[0], tooth[1] - other[1])
    else:
        seg_dir = snap_dir(tooth[0] - pad.global_x, tooth[1] - pad.global_y)
    # which boundary line of the ARRAY the tooth lies beyond decides the
    # face (a diagonal last step is ambiguous); the last segment breaks a
    # corner tie
    fp = pcb.footprints[pad.component_ref]
    g = em.grid_of(fp)
    x0, y0, x1, y1 = g.bbox
    hx, hy = g.pitch_x / 2, g.pitch_y / 2
    beyond = [f for f, ok in (('right', tooth[0] > x1 + hx * 0.5),
                              ('left', tooth[0] < x0 - hx * 0.5),
                              ('down', tooth[1] > y1 + hy * 0.5),
                              ('up', tooth[1] < y0 - hy * 0.5)) if ok]
    face = (beyond[0] if len(beyond) == 1
            else (seg_dir if seg_dir in beyond or not beyond else beyond[0]))
    vias = [v for v in pcb.vias if v.net_id == nid
            and math.hypot(v.x - pad.global_x, v.y - pad.global_y) < 6.0]
    pad_r = max(pad.size_x, pad.size_y) / 2
    if any(math.hypot(v.x - pad.global_x, v.y - pad.global_y) <= pad_r + 0.01
           for v in vias):
        kind = 'via_in_pad'
    elif vias:
        kind = 'dogbone'
    else:
        kind = 'surface'
    return {'tooth': (round(tooth[0], 3), round(tooth[1], 3)), 'layer': layer,
            'vias': len(vias), 'kind': kind,
            'direction': face,
            'bearing': snap_dir(tooth[0] - pad.global_x, tooth[1] - pad.global_y),
            'site': ((round(vias[0].x, 3), round(vias[0].y, 3)) if vias else None)}


def fmt(m):
    if m is None:
        return '(bare)'
    return (f"{m['kind']}/{m['direction']}/{m['layer'][0]} "
            f"exit=({m['tooth'][0]:.2f},{m['tooth'][1]:.2f}) v={m['vias']}")


def fmt_ask(m):
    return (f'{m.kind}/{m.direction}/{m.layer[0]} '
            f'exit=({m.exit_pt[0]:.2f},{m.exit_pt[1]:.2f}) v={m.vias}')


def order_agreement(asked, achieved, nets):
    """Per face: the order of the ASKED exits along the face vs the order
    of the ACHIEVED teeth (only teeth that left by the asked face count,
    the others are already a face miss). Returns {face: (kept, n, inversions)}."""
    out = {}
    for face in DIRS:
        ns = [n for n in nets if asked[n].direction == face
              and achieved.get(n) is not None and achieved[n]['direction'] == face]   # a bare ball has no tooth to order
        if len(ns) < 2:
            continue
        ax = 0 if face in ('up', 'down') else 1   # coordinate ALONG the face
        a_order = sorted(ns, key=lambda n: asked[n].exit_pt[ax])
        g_order = sorted(ns, key=lambda n: achieved[n]['tooth'][ax])
        kept = sum(1 for i, n in enumerate(a_order) if g_order[i] == n)
        rank = {n: i for i, n in enumerate(a_order)}
        seq = [rank[n] for n in g_order]
        inv = sum(1 for i in range(len(seq)) for j in range(i + 1, len(seq))
                  if seq[i] > seq[j])
        out[face] = (kept, len(ns), inv)
    return out


def audit(asked, achieved, original, laid, log, label):
    """Print the per-tooth table and the summary; return the audit dict."""
    names = list(asked)
    n_dir = n_lay = n_kind = n_gap = 0
    log('  ' + f'{"net":7s} {"ORIGINAL":42s} {"ASKED":52s} {"ACHIEVED":42s} verdict')
    audit_d = {}
    for nm in names:
        o, a, g = (original or {}).get(nm), asked[nm], achieved.get(nm)
        # the gap is a position ALONG the face (x on up/down faces, y on
        # left/right); the outward distance is the engine's exit margin
        ax = 0 if a.direction in ('up', 'down') else 1
        gap = abs(g['tooth'][ax] - a.exit_pt[ax]) if g else float('nan')
        outward = abs(g['tooth'][1 - ax] - a.exit_pt[1 - ax]) if g else float('nan')
        v = []
        if nm not in laid or g is None:
            v.append('REFUSED -> original kept' if o else 'REFUSED (no copper)')
        else:
            v.append('face ok' if g['direction'] == a.direction
                     else f'FACE {a.direction}->{g["direction"]}')
            v.append('layer ok' if g['layer'] == a.layer
                     else f'LAYER {a.layer[0]}->{g["layer"][0]}')
            v.append('kind ok' if g['kind'] == a.kind
                     else f'KIND {a.kind}->{g["kind"]}')
            v.append('gap ok' if gap <= GAP_TOL else f'GAP off {gap:.2f}mm along the face')
            if outward > GAP_TOL:
                v.append(f'exit {outward:.2f}mm further out than the model')
            n_dir += g['direction'] == a.direction
            n_lay += g['layer'] == a.layer
            n_kind += g['kind'] == a.kind
            n_gap += gap <= GAP_TOL
            if o and (g['tooth'] == o['tooth'] and g['layer'] == o['layer']
                      and g['vias'] == o['vias']):
                v.append('= original')
        exact = (nm in laid and g is not None and g['direction'] == a.direction
                 and g['layer'] == a.layer and g['kind'] == a.kind and gap <= GAP_TOL)
        # WHICH dimensions the engine could not give, for a caller that has
        # to decide whether asking again is worth a pass. Losing the gap is
        # a berth a hair along its own face; losing the LAYER or the KIND is
        # a structurally different berth, and the engine saying so once
        # means it will say so again.
        lost = [] if (nm not in laid or g is None) else \
            [k for k, okd in (('face', g['direction'] == a.direction),
                              ('layer', g['layer'] == a.layer),
                              ('kind', g['kind'] == a.kind),
                              ('gap', gap <= GAP_TOL)) if not okd]
        audit_d[nm] = {'original': o, 'asked': fmt_ask(a), 'achieved': g,
                       'gap_mm': round(gap, 3), 'outward_mm': round(outward, 3),
                       'verdict': ', '.join(v), 'exact': exact, 'lost': lost}
        log('  ' + f'{nm:7s} {fmt(o):42s} {fmt_ask(a):52s} {fmt(g):42s} {", ".join(v)}')
    n_ok = len(laid)
    same = sum(1 for nm in laid if audit_d[nm]['verdict'].endswith('= original'))
    log(f'  {label} audit: {len(names)} asked, {n_ok} laid -- face {n_dir}/{n_ok}, '
        f'layer {n_lay}/{n_ok}, kind {n_kind}/{n_ok}, same gap (<= {GAP_TOL} mm) '
        f'{n_gap}/{n_ok}' + (f'; {same} identical to the original' if original else ''))
    oa = order_agreement(asked, {n: achieved[n] for n in laid}, laid)
    log(f'  {label} ORDER along each face (asked vs achieved, face-ok teeth only): '
        + ', '.join(f'{f}: {k}/{n} ranks kept, {i} inversion(s)'
                    for f, (k, n, i) in oa.items()))
    return audit_d, {'asked': len(names), 'laid': n_ok, 'face': n_dir,
                     'layer': n_lay, 'kind': n_kind, 'gap': n_gap, 'order': oa}


def drc_pairs(board, nets=None, pcb_data=None):
    """The DRC violation lines of `board` at the fanout's floor. `nets`
    (short names): only checks involving one of them -- sound when every
    other piece of copper was clean before, i.e. when the caller changed
    only those nets on a clean board (2026-09-18: a probe's fanout board
    is such a board; the whole-board check was 0.47 s of a 6.6 s probe).
    In-process when scoped (the checker imported once), a subprocess
    otherwise, as ever."""
    if nets:
        import contextlib
        import io
        sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
        import check_drc as _cd
        if pcb_data is not None:
            # THE GATE SEES WHAT SHIP TIME DECLARES (#962, 2026-09-22): a via
            # the run put in a pad or a paste opening is stamped Type VII when
            # the board is written (ship_vias), so the in-memory check must
            # count it as protected too -- or a re-escape from the pad reads
            # as one `via-in-paste` on one pose of the gate and none on its
            # mirror (the turned frame's apertures), and the two route apart.
            try:
                import fab_notes as _fn
                _fn.apply_stamps_in_memory(_fn.via_protection_stamps(pcb_data.vias, [], pcb_data)[0])
            except Exception:                                    # noqa: BLE001
                pass
        pats = [f'*/{n}' for n in nets] + list(nets)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            try:
                _cd.run_drc(board, clearance=te.SPEC_CLEARANCE, clearance_margin=0.1, net_patterns=pats, max_print=10 ** 6,
                            pcb_data=pcb_data)
            except SystemExit:
                pass
        out = buf.getvalue()
        if 'NO DRC VIOLATIONS' not in out and 'DRC VIOLATION' not in out:
            raise RuntimeError(f'check_drc (scoped to {nets}) gave no verdict for {board}: '
                               + ((out.strip().splitlines() or ['(no output)'])[-1])[:200])
        if 'NO DRC VIOLATIONS' in out:
            return []
        return _violation_lines(out)
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        board, '--clearance', str(te.SPEC_CLEARANCE), '--clearance-margin', '0.1'],
                       capture_output=True, text=True, env=awx_settings.environ())
    out = r.stdout + r.stderr
    # A CHECKER THAT DID NOT REPORT IS NOT A CLEAN BOARD. This is the only
    # copper gate on the board `realize` writes (and replan's probe-clean
    # test), and `return []` on no output read a traceback -- bad path,
    # import error -- as "no violations". Verified: drc_pairs on a
    # nonexistent board returned [].
    if 'NO DRC VIOLATIONS' not in out and 'DRC VIOLATION' not in out:
        raise RuntimeError(
            f'check_drc gave no verdict for {board} (exit {r.returncode}): '
            + ((out.strip().splitlines() or ['(no output)'])[-1])[:200])
    if 'NO DRC VIOLATIONS' in out:
        return []
    return _violation_lines(out)


def _violation_lines(out):
    """The checker's violation lines: a pair line (`A <-> B`), never its
    progress text (main's "Checking copper-to-hole (track <-> NPTH drill)"
    carries the arrow), and a via-in-paste line, which names no pair."""
    return [ln.strip() for ln in out.splitlines()
            if (('<->' in ln and not ln.lstrip().startswith('Checking'))
                or ' in paste opening ' in ln)]


def _joint_spec():
    """the joint fanout's spec (FANOUT_JOINT: route_bus --joint-fanout), or None"""
    path = awx_settings.get('FANOUT_JOINT')
    if not path or not os.path.isfile(path):
        return None
    import json
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _joint_realize(pcb, sref, names, src_choice, src_pad, byname, jspec, log=print, hands=None):
    """realize()'s engine call under the joint fanout: the source array planned jointly (joint_escape.plan_array) --
    `names` (the stripped nets: those asked to move, and the blockers freed with them) preferring their asks, the
    array's other nets and plane balls with them, the bus never by the face away from the other array -- and the
    stripped nets laid as planned by the joint escape engine (a planned pair's gate kept), each pair leaving with the
    hand `hands` asks (its berths', fanout_from_plan.berth_hands). Returns the engine's (tracks, vias to add, vias to
    remove, failed nets)."""
    import joint_escape as _je
    a = next((x for x in jspec['arrays'] if x['ref'] == sref), {'others': [], 'drops': []})
    dref = next((x['ref'] for x in jspec['arrays'] if x['ref'] != sref), None)
    full = [byname[nm][1].name for nm in names]
    prefer = {_je.short_name(nm): {'tooth': tuple(m.exit_pt), 'direction': m.direction, 'layer': m.layer,
                                   'kind': m.kind} for nm, m in src_choice.items()}
    far = _je.far_face(pcb, sref, dref) if dref and dref in pcb.footprints else None
    import route_layers
    hints, rep = _je.plan_array(pcb, sref, full, a['others'], jspec['layers'], far=far, prefer=prefer,
                                drops=a['drops'], log=log, hands=hands, vias_only=route_layers.escape_vias('src'))
    at_names = {(round(src_pad[nm].global_x, 3), round(src_pad[nm].global_y, 3)) for nm in names}
    hints = {at: h for at, h in hints.items() if at in at_names}
    unplanned = sorted(nm for nm in names
                       if (round(src_pad[nm].global_x, 3), round(src_pad[nm].global_y, 3)) not in hints)
    log(f'  source realize (joint plan of {sref}): {rep["status"]}, {len(names) - len(unplanned)}/{len(names)} '
        f'stripped net(s) planned, pairs {rep.get("pairs_escaped")}/{rep.get("pairs")}, others '
        f'{rep["others_escaped"] + rep["others_strapped"]}/{rep["others_balls"]}, drops {rep["dropped"]}/'
        f'{rep["plane_balls"]}'
        + (f', pairs held to their berths\' hand {len(rep["hands_held"])}/'
           f'{len(rep["hands_held"]) + len(rep["hands_free"])} re-planned'
           + (f' (NO tooth pair of it: {rep["hands_free"]})' if rep['hands_free'] else '') if hands else '')
        + (f'; UNPLANNED {unplanned}' if unplanned else ''))
    spec = {'net_layers': {n: _je.bus_route_layers(pcb) for n in full}, 'priority': list(full)}
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[sref], pcb, net_filter=full, layers=list(pcb.board_info.copper_layers),
        track_width=FAN_TRACK, clearance=FAN_CLEAR, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
        exit_margin=0.5, escape_method='jointescape', plane_drop='off', escape_dir_hints=hints, bus=spec)
    tracks, vias_add, gated = _je.keep_pair_gates(pcb, sref, hints, tracks, vias_add)
    if gated:
        log(f'  source realize (joint plan of {sref}): left out, through a planned pair\'s gate: '
            + ', '.join(sorted(pcb.nets[i].name.split('/')[-1] for i in gated)))
    return tracks, vias_add, vias_rm, failed


def realize(board, src_choice, src_pad, byname, sref, out_path, log=print,
            guard_names=(), free=(), strict=False, clean_base=False, hands=None):
    """Strip the chosen nets' source copper, re-fan them in the asked
    faces, write `out_path`, audit every tooth. Returns a dict with the
    per-net audit, `ok` (laid) / `restored` (refused), and `rejected` (a
    DRC reason) when the written board must not be used. `hands`, under the
    joint fanout: the hand each pair's teeth leave with (_joint_realize).

    `free`: the JOINT SOURCE RE-FAN (2026-09-11). Nets stripped and
    re-laid in the SAME engine call as the chosen ones but given NO hint,
    so the engine may put them anywhere. They are the blockers
    (`blockers_of`) whose copper stands in the room the ask needs. A
    one-net re-fan cannot move them -- they are foreign copper to it --
    so the engine degrades the ask instead; inside one call
    `underpad._follow_plan` can rip and re-lay them around it. They are
    NOT audited against an ask (they have none) and are excluded from the
    drift guard, which is a guard on teeth nobody asked to move.
    `strict`: cap the engine's degrade ladder at level 2, so a ball with
    no berth on its asked face is left UNESCAPED (and so restored to its
    original copper, reported) rather than dumped somewhere the plan
    never asked for and audited as a near-miss."""
    pcb = parse_kicad_pcb(board)
    n2n = {i: n.name for i, n in pcb.nets.items()}
    free = [nm for nm in dict.fromkeys(free) if nm not in src_choice]
    names = list(src_choice) + free
    # the drift guard's BEFORE, read now, before any copper comes off (it
    # used to parse the board a second time for an untouched copy)
    others = [nm for nm in guard_names if nm not in src_choice and nm not in free]
    before = te.endpoints(pcb, others, byname) if others else {}
    original = {nm: measure_tooth(pcb, nm, src_pad[nm], byname) for nm in names}

    removed = {}
    for nm in names:
        removed[nm] = remove_net_from_pcb_data(pcb, byname[nm][0])
    import joint_escape as _je
    jspec = _joint_spec()
    # no placement step follows this chain, so every foreign pad -- a
    # decoupling cap under the array included -- is one a via must clear.
    # The JOINT fanout's bus step is followed by the cap placement step, which
    # moves the unlocked two-pad passives off the fanout: its realize plans and
    # lays round them as the comb and the other nets' fanout do
    # (joint_escape.movable_refs) -- marked immovable here alone, the realize's
    # plan and engine read RX10's pad under zynq U1 two ways. Once the cap step
    # has moved them (the whole route's later rounds, joint_escape.
    # passives_fixed), nothing moves them again: immovable here too.
    # The JOINT realize with more routing layers than two alone; on two the
    # asked teeth are laid as the chain lays them, the array's other balls
    # planned round them after (fanout_from_plan.joint_others): the joint plan
    # of the whole array, run for every re-fan, moved the asked teeth for every
    # ball's sake -- the zynq DDR's ends crossed 260 times to the chain's 202,
    # and its four realizes of round 1 took 400 s
    joint_ = jspec is not None and len(route_layers.layers()) > 2
    if not joint_ or _je.passives_fixed():
        pcb._fanout_all_foreign_immovable = True
    if joint_:
        # THE JOINT FANOUT (FANOUT_JOINT): the asked teeth from the source array's JOINT plan -- the stripped nets
        # preferring their asks, the array's other nets and plane balls planned with them so the teeth leave them
        # room, a pair's legs held together -- and the stripped nets laid exactly as planned (the other nets are laid
        # round the bus after the fanout, joint_others). The under-pad engine, given the asks alone, could not lay
        # some at all (zynq U1 on three layers: TXNRX's tooth, asked on B, In2 and F in turn, laid a gap off each
        # time, banned each time, until it had no option left)
        tracks, vias_add, vias_rm, failed = _joint_realize(pcb, sref, names, src_choice, src_pad, byname, jspec,
                                                           log, hands=hands)
    else:
        _je.reserve_ball_vias(pcb)      # the joint fanout's promise to the others, kept by a bus-only re-fan
        hints = {}
        for nm, m in src_choice.items():
            p = src_pad[nm]
            fm = full_move(m)
            if strict:
                fm['strict'] = True
            hints[(round(p.global_x, 3), round(p.global_y, 3))] = fm
        # the `free` nets get NO hint on purpose: they are stripped so the
        # engine has their room to give, not so it reproduces their escapes
        # the UNDER-PAD engine, not 'auto': the channel engine assigns a
        # channel per ball and runs it straight to the edge without treating
        # the unmoved nets' existing stubs as channel occupants (measured: a
        # re-fanned SDQ15 laid on top of SDQM0's stub, SBA1 on SA11's, 38
        # segment-segment DRC). The under-pad A* carries the exact registry of
        # every foreign track on the board, which a partial re-fan of an
        # already-fanned array needs.
        tracks, vias_add, vias_rm, failed = generate_bga_fanout(
            pcb.footprints[sref], pcb, net_filter=names, layers=route_layers.stacked(pcb.board_info.copper_layers),
            track_width=FAN_TRACK, clearance=FAN_CLEAR, via_size=te.VIA_SIZE,
            via_drill=te.VIA_DRILL,
            exit_margin=0.5, escape_method='underpad', plane_drop='off',
            escape_dir_hints=hints)
    got = {t['net_id'] for t in tracks}
    ok = [nm for nm in names if byname[nm][0] in got]
    restored = [nm for nm in names if nm not in ok]
    ok_ids = {byname[nm][0] for nm in ok}
    tracks = [t for t in tracks if t['net_id'] in ok_ids]
    vias_add = [v for v in vias_add if v.get('net_id') in ok_ids]

    content = open(board, encoding='utf-8').read()
    segs_rm = [s for nm in ok for s in removed[nm][0]]
    vias_rm0 = [v for nm in ok for v in removed[nm][1]]
    content, n_s = remove_segments_from_content(content, segs_rm, n2n)
    content, n_v = remove_vias_from_content(content, vias_rm0, n2n)
    if n_s != len(segs_rm) or n_v != len(vias_rm0):
        log(f'  source realize: WARNING strip matched {n_s}/{len(segs_rm)} '
            f'segments, {n_v}/{len(vias_rm0)} vias')
    stripped = out_path + '.stripped.tmp'
    with open(stripped, 'w', encoding='utf-8') as f:
        f.write(content)
    add_tracks_and_vias_to_pcb(stripped, out_path, tracks, vias_add, vias_rm,
                               net_id_to_name=n2n)
    os.remove(stripped)
    ship_vias.stamp(out_path, 'source realize', log)
    from copy_board import copy_siblings
    copy_siblings(board, out_path)      # (the project and the .kicad_dru's per-layer rules with it)

    pcb2 = parse_kicad_pcb(out_path)
    achieved = {nm: measure_tooth(pcb2, nm, src_pad[nm], byname) for nm in names}
    log(f'  source realize: {len(src_choice)} tooth/teeth asked to move, engine laid '
        f'{len([n for n in ok if n in src_choice])}, refused '
        f'{len([n for n in restored if n in src_choice])}'
        + (f' (restored: {", ".join(n for n in restored if n in src_choice)})'
           if any(n in src_choice for n in restored) else '')
        + (f'; {len(free)} blocker(s) re-fanned with them: {", ".join(free)}'
           if free else ''))
    if free:
        moved = [nm for nm in free
                 if achieved.get(nm) and original.get(nm)
                 and achieved[nm]['tooth'] != original[nm]['tooth']]
        log(f'  source realize: blockers -- {len(moved)} of {len(free)} took a '
            f'different tooth' + (f': {", ".join(moved)}' if moved else ''))
    audit_d, counts = audit(src_choice, {n: achieved[n] for n in src_choice},
                            {n: original[n] for n in src_choice},
                            [n for n in ok if n in src_choice], log, 'source')
    # the teeth NOT asked to move must not have moved (the engine re-fans
    # only the stripped nets, but say so from the board, not from trust)
    if others:
        after = te.endpoints(pcb2, others, byname)
        drift = [nm for nm in others if before[nm][0] != after[nm][0]]
        log(f'  source realize: {len(others) - len(drift)}/{len(others)} unmoved '
            f'teeth unchanged' + (f'; DRIFTED: {", ".join(drift)}' if drift else ''))
    pairs = drc_pairs(out_path, nets=list(src_choice) if clean_base else None)
    if jspec is not None and pairs:
        # (the joint fanout: a pad of a movable passive is the cap placement step's to move off this copper -- the
        # plan and the engine laid round it as not there, and a pair against it is no reason to refuse the board;
        # refused for it, the zynq U1 realize's whole board went for CTRL_OUT0's track across RX10.2)
        mov = _je.movable_refs(pcb, sref)
        cap = [ln for ln in pairs if any(r_ in mov for r_ in re.findall(r'\(([^.()\s]+)\.[^)\s]*\)', ln))]
        if cap:
            log(f'  source realize: {len(cap)} DRC pair(s) against a movable passive, left to the cap placement step')
            pairs = [ln for ln in pairs if ln not in cap]
    rejected = None
    if pairs:
        rejected = f'{len(pairs)} DRC pair(s) on the realized source board'
        log(f'  source realize: REJECTED -- {rejected}:')
        for ln in pairs[:12]:
            log('      ' + ln)
    return {'board': out_path, 'audit': audit_d, 'ok': ok, 'restored': restored,
            'achieved': achieved, 'original': original, 'rejected': rejected,
            'counts': counts, 'free': list(free), 'pairs': pairs, 'joint': jspec is not None}
