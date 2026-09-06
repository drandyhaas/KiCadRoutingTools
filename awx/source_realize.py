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
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import (add_tracks_and_vias_to_pcb,  # noqa: E402
                          remove_segments_from_content,
                          remove_vias_from_content)
from pcb_modification import remove_net_from_pcb_data  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402

DIRS = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}
LAYERS = ('F.Cu', 'B.Cu')
GAP_TOL = 0.2      # mm: an achieved tooth this close to the asked exit is the same gap


def move_sig(m):
    """The identity of a menu move for the plan's feasibility ledger."""
    return (m.kind, m.direction, m.layer, round(m.exit_pt[0], 2),
            round(m.exit_pt[1], 2),
            (round(m.site[0], 2), round(m.site[1], 2)) if m.site else None)


def full_move(m):
    """The plan's Move as the engine's FULL hint: face, exit point (its
    coordinate along the face is the gap), layer, kind, dog-bone site."""
    return {'face': m.direction, 'exit': tuple(m.exit_pt), 'layer': m.layer,
            'kind': m.kind, 'site': (tuple(m.site) if m.site else None)}


def snap_dir(dx, dy):
    h = math.hypot(dx, dy) or 1.0
    return min(DIRS, key=lambda k: (DIRS[k][0] - dx / h) ** 2
               + (DIRS[k][1] - dy / h) ** 2)


def measure_tooth(pcb, nm, pad, byname, dest_ref=None):
    """What the board says about this net's escape at the array `pad`
    belongs to: the free end, the layer that end is on, the vias near the
    ball, and the kind/face that copper amounts to. `dest_ref` names the
    destination array when both ends are fanned out (te.endpoints then
    attributes the two free ends by walking the copper)."""
    nid = byname[nm][0]
    ends = te.endpoints(pcb, [nm], byname, dest_ref=dest_ref)
    tooth = ends[nm][1] if dest_ref is not None else ends[nm][0]
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
              and achieved[n]['direction'] == face]
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
        audit_d[nm] = {'original': o, 'asked': fmt_ask(a), 'achieved': g,
                       'gap_mm': round(gap, 3), 'outward_mm': round(outward, 3),
                       'verdict': ', '.join(v), 'exact': exact}
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


def drc_pairs(board):
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        board, '--clearance', '0.1', '--clearance-margin', '0.1'],
                       capture_output=True, text=True)
    out = r.stdout + r.stderr
    if 'NO DRC VIOLATIONS' in out:
        return []
    return [ln.strip() for ln in out.splitlines() if '<->' in ln]


def realize(board, src_choice, src_pad, byname, sref, out_path, log=print,
            guard_names=()):
    """Strip the chosen nets' source copper, re-fan them in the asked
    faces, write `out_path`, audit every tooth. Returns a dict with the
    per-net audit, `ok` (laid) / `restored` (refused), and `rejected` (a
    DRC reason) when the written board must not be used."""
    pcb = parse_kicad_pcb(board)
    pcb0 = parse_kicad_pcb(board)      # untouched copy for the drift guard
    n2n = {i: n.name for i, n in pcb.nets.items()}
    names = list(src_choice)
    original = {nm: measure_tooth(pcb, nm, src_pad[nm], byname) for nm in names}

    removed = {}
    for nm in names:
        removed[nm] = remove_net_from_pcb_data(pcb, byname[nm][0])
    # no placement step follows this chain, so every foreign pad -- a
    # decoupling cap under the array included -- is one a via must clear
    pcb._fanout_all_foreign_immovable = True
    hints = {}
    for nm, m in src_choice.items():
        p = src_pad[nm]
        hints[(round(p.global_x, 3), round(p.global_y, 3))] = full_move(m)
    # the UNDER-PAD engine, not 'auto': the channel engine assigns a
    # channel per ball and runs it straight to the edge without treating
    # the unmoved nets' existing stubs as channel occupants (measured: a
    # re-fanned SDQ15 laid on top of SDQM0's stub, SBA1 on SA11's, 38
    # segment-segment DRC). The under-pad A* carries the exact registry of
    # every foreign track on the board, which a partial re-fan of an
    # already-fanned array needs.
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[sref], pcb, net_filter=names, layers=list(LAYERS),
        track_width=0.1, clearance=0.1, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
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
    pro = os.path.splitext(board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(out_path)[0] + '.kicad_pro')

    pcb2 = parse_kicad_pcb(out_path)
    achieved = {nm: measure_tooth(pcb2, nm, src_pad[nm], byname) for nm in names}
    log(f'  source realize: {len(names)} teeth asked to move, engine laid '
        f'{len(ok)}, refused {len(restored)}'
        + (f' (restored: {", ".join(restored)})' if restored else ''))
    audit_d, counts = audit(src_choice, achieved, original, ok, log, 'source')
    # the teeth NOT asked to move must not have moved (the engine re-fans
    # only the stripped nets, but say so from the board, not from trust)
    others = [nm for nm in guard_names if nm not in src_choice]
    if others:
        before = te.endpoints(pcb0, others, byname)
        after = te.endpoints(pcb2, others, byname)
        drift = [nm for nm in others if before[nm][0] != after[nm][0]]
        log(f'  source realize: {len(others) - len(drift)}/{len(others)} unmoved '
            f'teeth unchanged' + (f'; DRIFTED: {", ".join(drift)}' if drift else ''))
    pairs = drc_pairs(out_path)
    rejected = None
    if pairs:
        rejected = f'{len(pairs)} DRC pair(s) on the realized source board'
        log(f'  source realize: REJECTED -- {rejected}:')
        for ln in pairs[:12]:
            log('      ' + ln)
    return {'board': out_path, 'audit': audit_d, 'ok': ok, 'restored': restored,
            'achieved': achieved, 'original': original, 'rejected': rejected,
            'counts': counts}
