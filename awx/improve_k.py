#!/usr/bin/env python3
"""#622 intelligent iteration on a routed result (the wobble turned
into search). One chain result is a single draw from a chaotic
system; instead of accepting the draw, DIAGNOSE it and make informed
moves through the cheapest control channel -- the pages sidecar (a
braid-only rerun, no comb/berth redraw):

  1. braid the fanout board free -> baseline; dump its own plan.
  2. pin its own attempt-1 pages (kills inter-attempt drift).
  3. LEDGER DIAGNOSIS: per net, actual vias vs the model optimum on
     the same geometry. Every net paying more than the model is a
     WASTE net with a named mechanism (needless swimmer, wrong page,
     paged-but-weaving).
  4. targeted moves, one waste net at a time: flip its sidecar entry
     toward the model's ride (swimmer -> model page, paged -> other
     layer, paged -> swimmer). Re-braid; keep STRICTLY better only
     (open, drc, vias) lexicographic; iterate until a sweep is dry.
  5. RELAY CHANNELS for the top waste nets a pages flip did not fix
     (each trial = relay the fanout, RE-DUMP the plan from the
     relayed board, re-pin its own pages plus the accepted flips,
     braid -- the stale-frame protocol, measured at 60v when skipped):
       tooth: the tooth starts the net off the model's ride layer --
              relay it in place onto that layer (--keep-pos);
       berth: same decision at the DU1 end (--ref DU1);
       swap:  the net's corridor order inverts against a neighbour
              (launch_idx vs target_rank from the braid's own plan)
              -- exchange the two proven teeth (--swap);
       face:  a net still paying 2+ vias over its model floor is
              diving mid-ride -- the escape FACE the fanout chose
              (dest-anchored clustering, decided once) forces
              same-page crossings no in-place flip can undo. Offer
              the OTHER window faces at each end, coord = the
              ball->other-end line's crossing of that face (clamped;
              the engine negotiates the real slot), model's ride
              layer first. Pure geometry, net-name-free; each ask is
              screened by a swapped-stub single-net braid before
              paying the full one. IMPROVE_FACES=0 disables,
              IMPROVE_FACE_ASKS caps the budget (default 30).
     Accept-and-build: an accepted relay SWITCHES the fanout board
     every later trial builds on. Trials stay SERIAL by design.

All boards/sidecars under tmp/. usage: improve_k.py FO_BOARD [K]
"""
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)
from plan_global import opt_rides  # noqa: E402

import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument('fo')
_ap.add_argument('k', nargs='?', default='28')
_ap.add_argument('tag', nargs='?', default='')
_ap.add_argument('--num-improve', type=int, default=3,
                 help='evolution sweeps over the waste list (each '
                      'sweep re-diagnoses and tries targeted flips)')
_ap.add_argument('--src-ref', default='U1',
                 help='source array ref, for the anchors emit')
_a = _ap.parse_args()
FO = _a.fo
K = _a.k
NETS = subprocess.run([sys.executable, 'coherent_nets.py', K],
                      capture_output=True, text=True).stdout.strip()
TAG = os.path.join('tmp', 'imp' + K + _a.tag)
SIDECAR = os.path.splitext(FO)[0] + '.pages.json'
ENV = dict(os.environ, TWO_PAGE='1')


OPENS = {}     # score board path -> open net names of the last grade


# IMPROVE_FLOOR_JUDGE=1 (step-1 of the calibrated-ledger ladder):
# the strict screens judge a position candidate by whether the
# board's DP FLOOR drops (ledger_cal.Judge, calibrated on six boards
# 0902: slack 2-4 everywhere, true order reproduced), instead of by
# the net's own via count. Cache one Judge per best board.
_JUDGE = {'path': None, 'obj': None}


def _floor_judge():
    if os.environ.get('IMPROVE_FLOOR_JUDGE') != '1':
        return None
    if _JUDGE['path'] != best_board:
        _JUDGE['path'] = best_board
        try:
            from ledger_cal import Judge
            _JUDGE['obj'] = Judge(best_board, NETS.split(','))
        except Exception as e:
            print(f'  floor judge unavailable ({e})')
            _JUDGE['obj'] = None
    return _JUDGE['obj']


def braid(out, smooth=False):
    # trials skip #536 smoothing: it is 23s of a 52s braid (measured,
    # K35 cProfile) and the lexicographic key (open, drc, vias) is
    # smoothing-INVARIANT (verified: identical tuple, only segs
    # differ). The final winner is re-braided smoothed once.
    r = subprocess.run(
        [sys.executable, 'braid.py', '--board', FO, '--dest', 'DU1',
         '--nets', NETS, '--out', out]
        + ([] if smooth else ['--no-smooth']), env=ENV,
        capture_output=True, text=True)
    g = subprocess.run(
        [sys.executable, 'grade_k.py', out + '.kicad_pcb', NETS],
        capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', g)
    mo = re.search(r'open: ([A-Za-z0-9_,]+)', g)
    OPENS[out] = mo.group(1).split(',') if mo else []
    return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m \
        else (99, 99, 999)


def dump_plan(path):
    subprocess.run(
        [sys.executable, 'braid.py', '--board', FO, '--dest', 'DU1',
         '--nets', NETS, '--out', os.devnull, '--plan-json', path],
        env=ENV, capture_output=True, text=True)
    return json.load(open(path))


def swap_partner(n):
    """The corridor neighbour whose order inverts against n in the
    braid's own plan (launch_idx vs target_rank) -- the pair whose
    tooth exchange uncrosses at the source. Nearest inversion wins."""
    d = plan.get(n) or {}
    best = None
    for m, e in plan.items():
        if m == n or e.get('corridor') != d.get('corridor'):
            continue
        li, lj = d.get('launch_idx'), e.get('launch_idx')
        ri, rj = d.get('target_rank'), e.get('target_rank')
        if None in (li, lj, ri, rj):
            continue
        if (li < lj) != (ri < rj):
            gap = abs(li - lj)
            if best is None or gap < best[0]:
                best = (gap, m)
    return best[1] if best else None


def group_try(flip_net, flip_val, stranded, label, depth=2):
    """The GROUP move a single flip cannot make: keep a via-saving
    pages flip AND relay the nets it stranded (tooth to the other
    layer -- the crowded page refused their lanes, a different entry
    layer is the braid's relief). K28's SA8->swim reaches 46v but
    strands 2; this is the composed retry of exactly that near-miss.
    When the composed board itself saves vias but still strands
    (measured: SA8+SCKE1+SODT0 at 45v/1 open), recurse ONCE with the
    new opens appended -- the group of the group."""
    global FO, SIDECAR, best_score, best_board, plan, own, cur_pages
    global rides, divers, model
    tk = tried_key(('group', flip_net, flip_val, tuple(stranded),
                    depth))
    if tk in TRIED:
        return False                        # a hit is always a reject
    TRIED[tk] = True
    # pages-only composition FIRST: re-page each stranded net rather
    # than relaying its tooth -- free (one braid, no copper change),
    # and the only move available to a net whose ball can escape on
    # ONE layer only (SDQ7's under-pad B exit cannot obey an F tooth
    # ask; the relay heuristic refused it on every comb)
    trial0 = dict(cur_pages)
    trial0[flip_net] = flip_val
    for m in stranded:
        trial0[m] = None if trial0.get(m) is not None else \
            (plan.get(m) or {}).get('tooth_layer')
    put_pages(trial0)
    s0 = braid(f'{TAG}_{label}p')
    if s0 < best_score:
        print(f'  {label}p: open={s0[0]} drc={s0[1]} vias={s0[2]}'
              '   ACCEPT')
        best_score = s0
        best_board = f'{TAG}_{label}p.kicad_pcb'
        flips[flip_net] = flip_val
        for m in stranded:
            flips[m] = trial0[m]
        cur_pages = trial0
        return True
    print(f'  {label}p: open={s0[0]} drc={s0[1]} vias={s0[2]}')
    fo0, side0 = FO, SIDECAR
    for i, m in enumerate(stranded):
        d = plan.get(m) or {}
        oth = 'B.Cu' if d.get('tooth_layer') == 'F.Cu' else 'F.Cu'
        out = f'{TAG}_{label}g{i}.kicad_pcb'
        r = subprocess.run(
            [sys.executable, 'relay_net.py', FO, m, '--out', out,
             '--keep-pos', '--layer', oth],
            capture_output=True, text=True)
        if not os.path.exists(out) or 'MISSED' in r.stdout:
            print(f'  {label}: relay of {m} refused')
            FO, SIDECAR = fo0, side0
            return False
        FO = out
        SIDECAR = os.path.splitext(FO)[0] + '.pages.json'
    plan2 = dump_plan(f'{TAG}_{label}_plan.json')
    own2 = {k: d['page'] for k, d in plan2.items() if 'page' in d}
    trial = dict(own2)
    trial.update(flips)
    trial[flip_net] = flip_val
    put_pages(trial)
    s = braid(f'{TAG}_{label}_pin')
    if s < best_score:
        print(f'  {label}: open={s[0]} drc={s[1]} vias={s[2]}   ACCEPT')
        best_score = s
        best_board = f'{TAG}_{label}_pin.kicad_pcb'
        flips[flip_net] = flip_val
        plan, own, cur_pages = plan2, own2, trial
        rides, divers, model = opt_rides(plan)
        return True
    print(f'  {label}: open={s[0]} drc={s[1]} vias={s[2]}')
    newly = [m for m in OPENS.get(f'{TAG}_{label}_pin', [])
             if m not in stranded]
    if os.path.exists(SIDECAR):
        os.remove(SIDECAR)
    FO, SIDECAR = fo0, side0
    if depth > 1 and s[0] > 0 and s[1] <= best_score[1] \
            and s[2] < best_score[2] and 1 <= len(newly) <= 2:
        return group_try(flip_net, flip_val,
                         list(stranded) + newly,
                         label + 'x', depth - 1)
    return False


def relay_try(nets_arg, largs, label, screen=False, rescue=False,
              dedupe=None):
    """One relay trial, accept-and-build: relay the fanout, re-dump
    the plan FROM THE RELAYED BOARD, pin its own pages plus every
    accepted flip, braid, keep strictly better. On accept the relayed
    board becomes the base every later trial builds on. With
    screen=True (single net only) a cheap swapped-stub single-net
    braid filters clear losers before the full braid. With
    rescue=True a via-saving trial that STRANDS 1-2 nets gets the
    group move before dying: re-page each stranded net on the relayed
    board (free, one braid -- the same pages-first rescue group_try
    uses for flips)."""
    global FO, SIDECAR, best_score, best_board, plan, own, cur_pages
    global rides, divers, model
    relay_try.last_dup = False
    tk = tried_key(('relay', nets_arg, tuple(largs)))
    if tk in TRIED:
        return False                        # a hit is always a reject
    TRIED[tk] = True
    out = f'{TAG}_{label}.kicad_pcb'
    r = subprocess.run(
        [sys.executable, 'relay_net.py', FO, nets_arg, '--out', out]
        + largs, capture_output=True, text=True)
    if not os.path.exists(out) or 'MISSED' in r.stdout:
        print(f'  {label}: relay refused')
        return False
    # DELIVERED-geometry dedupe (audit #4): the engine may slide the
    # slot or fall back to another gap, so distinct asks can deliver
    # the SAME escape -- relay_net now reports the delivered face
    # geometrically; skip the screen/braid when this delivery was
    # already tried (distance match, never a rounded key)
    mdel = re.search(r'DELIVERED (\w+)/([\w.]+)@([-\d.]+)', r.stdout)
    if dedupe is not None and mdel:
        dside, dlay = mdel.group(1), mdel.group(2)
        dcoord = float(mdel.group(3))
        for s0_, l0_, c0_ in dedupe:
            if s0_ == dside and l0_ == dlay and abs(c0_ - dcoord) < 0.3:
                print(f'  {label}: duplicate delivery '
                      f'({dside}/{dlay}@{dcoord:.2f})')
                relay_try.last_dup = True
                return False
        dedupe.append((dside, dlay, dcoord))
    if screen and ',' not in nets_arg and screen_relay(
            nets_arg, out, strict=(screen == 'strict')):
        print(f'  {label}: screened out (single-net'
              + (', strict' if screen == 'strict' else '') + ')')
        return False
    return _trial_on_fo(out, label, rescue=rescue)


def _trial_on_fo(newfo, label, rescue=False):
    """The confirm tail every FO-changing trial shares: switch to the
    new fanout board, re-dump its plan, pin own pages + accepted
    flips, braid once, keep strictly better (with the optional
    stranded-nets rescue); restore FO/SIDECAR on reject."""
    global FO, SIDECAR, best_score, best_board, plan, own, cur_pages
    global rides, divers, model
    fo0, side0 = FO, SIDECAR
    FO = newfo
    SIDECAR = os.path.splitext(FO)[0] + '.pages.json'
    plan2 = dump_plan(f'{TAG}_{label}_plan.json')
    own2 = {m: d['page'] for m, d in plan2.items() if 'page' in d}
    trial = dict(own2)
    trial.update(flips)
    put_pages(trial)
    s = braid(f'{TAG}_{label}_pin')
    if s < best_score:
        print(f'  {label}: open={s[0]} drc={s[1]} vias={s[2]}   ACCEPT')
        best_score = s
        best_board = f'{TAG}_{label}_pin.kicad_pcb'
        plan, own, cur_pages = plan2, own2, trial
        rides, divers, model = opt_rides(plan)
        return True
    print(f'  {label}: open={s[0]} drc={s[1]} vias={s[2]}')
    if rescue and s[0] > 0 and s[1] <= best_score[1] \
            and s[2] < best_score[2]:
        gb = subprocess.run(
            [sys.executable, 'grade_k.py', best_board, NETS],
            capture_output=True, text=True).stdout
        mb = re.search(r'open: ([A-Za-z0-9_,]+)', gb)
        base_open = set(mb.group(1).split(',')) if mb else set()
        newly = [m for m in OPENS.get(f'{TAG}_{label}_pin', [])
                 if m not in base_open]
        # WIDE rescue for batch trials (rescue='wide'): the composed
        # wrap batches measured 64v-with-4-open at K35 -- 25+ vias on
        # the table beyond the <=2 cap. Cap 4 and ONE recursion round
        # (re-page the rescue's own newly-stranded on top), same
        # strictly-better keep throughout.
        cap = 4 if rescue == 'wide' else 2
        rnd_trial = trial
        rnd_flip = {}
        for rnd in range(2 if rescue == 'wide' else 1):
            if not (1 <= len(newly) <= cap):
                break
            if any(m in rnd_flip for m in newly):
                break                    # a re-paged net re-stranded
            trial2 = dict(rnd_trial)
            for m in newly:
                trial2[m] = None if trial2.get(m) is not None else \
                    (plan2.get(m) or {}).get('tooth_layer')
                rnd_flip[m] = trial2[m]
            put_pages(trial2)
            rtag = f'{TAG}_{label}_r{rnd if rnd else ""}'
            s2 = braid(rtag)
            if s2 < best_score:
                print(f'  {label}r{rnd if rnd else ""}: open={s2[0]} '
                      f'drc={s2[1]} vias={s2[2]}   ACCEPT')
                best_score = s2
                best_board = rtag + '.kicad_pcb'
                for m, v in rnd_flip.items():
                    flips[m] = v
                plan, own, cur_pages = plan2, own2, trial2
                rides, divers, model = opt_rides(plan)
                return True
            print(f'  {label}r{rnd if rnd else ""}: open={s2[0]} '
                  f'drc={s2[1]} vias={s2[2]}')
            # recursion condition: still via-saving, still stranding
            if not (s2[0] > 0 and s2[1] <= best_score[1]
                    and s2[2] < best_score[2]):
                break
            newly = [m for m in OPENS.get(rtag, [])
                     if m not in base_open]
            rnd_trial = trial2
    if os.path.exists(SIDECAR):
        os.remove(SIDECAR)
    FO, SIDECAR = fo0, side0
    return False


def actual_vias(board):
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(board)
    nm = {i: n.name.split('/')[-1] for i, n in p.nets.items()}
    out = {}
    for v in p.vias:
        n = nm.get(v.net_id)
        if n:
            out[n] = out.get(n, 0) + 1
    return out


def put_pages(pages):
    json.dump(pages, open(SIDECAR, 'w'), indent=1)


def _walk_strip(txt, token, match):
    out, i = [], 0
    pat = '(' + token
    while True:
        j = txt.find(pat, i)
        while j >= 0 and j + len(pat) < len(txt) \
                and txt[j + len(pat)] not in ' \n\t(':
            j = txt.find(pat, j + 1)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            ch = txt[k]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        if match(txt[j:k + 1]):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def _collect_blocks(txt, token, match):
    """The matching blocks themselves (same walk as _walk_strip)."""
    out, i = [], 0
    pat = '(' + token
    while True:
        j = txt.find(pat, i)
        while j >= 0 and j + len(pat) < len(txt) \
                and txt[j + len(pat)] not in ' \n\t(':
            j = txt.find(pat, j + 1)
        if j < 0:
            break
        k, depth = j, 0
        while True:
            ch = txt[k]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        if match(txt[j:k + 1]):
            out.append(txt[j:k + 1])
        i = k + 1
    return out


def _net_matcher(nid, netname):
    def net_match(block):
        m = re.search(r'\(net (\d+)\)', block)
        if m:
            return int(m.group(1)) == nid
        m = re.search(r'\(net "([^"]+)"\)', block)
        return bool(m and m.group(1) == netname)
    return net_match


def _swap_stub(board, relayed_fo, n, out):
    """Screen world for a POSITION relay: `board` with ALL of net n's
    copper replaced by the relayed fanout board's copper for n (its
    stubs at both arrays -- a fanout board has no lanes, so its net-n
    blocks ARE the new stubs)."""
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    from kicad_parser import parse_kicad_pcb
    fo_p = parse_kicad_pcb(relayed_fo)
    by = {net.name.split('/')[-1]: (i, net)
          for i, net in fo_p.nets.items()}
    nid, netname = by[n][0], by[n][1].name
    net_match = _net_matcher(nid, netname)
    fo_txt = open(relayed_fo, encoding='utf-8').read()
    blocks = (_collect_blocks(fo_txt, 'segment', net_match)
              + _collect_blocks(fo_txt, 'via', net_match))
    txt = open(board, encoding='utf-8').read()
    txt = _walk_strip(txt, 'segment', net_match)
    txt = _walk_strip(txt, 'via', net_match)
    pos = txt.rfind(')')
    txt = txt[:pos] + '\n'.join(blocks) + '\n' + txt[pos:]
    open(out, 'w', encoding='utf-8').write(txt)


def screen_relay(n, relayed_fo, strict=False):
    """Single-net screen for a position relay: swap n's copper for
    the relayed stub, braid n ALONE (free -- the braid picks the ride)
    against the frozen rest. Rejects only a routable-and-no-cheaper
    result; a frozen-world refusal pays the honest full braid.
    strict=True (the blind geometric face sweep -- measured: 2/3 of
    its screen survivors died at the 30-52s whole-board braid): the
    single-net gain must be >= 2 vias (a dive pair, the channel's own
    trigger unit) AND the lane must add no DRC to the frozen world."""
    try:
        scr = TAG + '_scr.kicad_pcb'
        _swap_stub(best_board, relayed_fo, n, scr)
        side = os.path.splitext(scr)[0] + '.pages.json'
        if os.path.exists(side):
            os.remove(side)
        r = subprocess.run(
            [sys.executable, 'braid.py', '--board', scr,
             '--dest', 'DU1', '--nets', n, '--out', TAG + '_scr1'],
            env=ENV, capture_output=True, text=True)
        b1 = TAG + '_scr1.kicad_pcb'
        if not os.path.exists(b1) or 'REFUSED' in r.stdout:
            return False
        J = _floor_judge() if strict else None
        if J is not None:
            nf = J.swap_floor(b1, n)
            if nf is None or nf >= J.floor_total:
                return True         # reject: the FLOOR does not drop
            print(f'  floor judge: {n} candidate floor '
                  f'{J.floor_total} -> {nf}')
        else:
            scr_v = actual_vias(b1).get(n, 99)
            cur_v = actual_vias(best_board).get(n, 0)
            if scr_v >= cur_v - (1 if strict else 0):
                return True
        if strict:
            g = subprocess.run(
                [sys.executable, 'grade_k.py', b1, NETS],
                capture_output=True, text=True).stdout
            m = re.search(r'drc=(\d+)', g)
            if m and int(m.group(1)) > best_score[1]:
                return True
        return False
    except Exception:
        return False


_GEOM = {}


def _geom(fo):
    """Escape geometry of a fanout board, cached per path: per array
    ref, the relay window bbox (pads + 0.25, relay_net's W), each
    net's ball position, and each net's CURRENT stub face (degree-1
    stub end in the rip window farthest from centre -- relay_net's
    own derivation)."""
    if fo in _GEOM:
        return _GEOM[fo]
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(fo)
    id2s = {i: net.name.split('/')[-1] for i, net in pcb.nets.items()}
    out = {}
    for ref in (_a.src_ref, 'DU1'):
        fp = pcb.footprints.get(ref)
        if fp is None:
            continue
        xs = [p.global_x for p in fp.pads]
        ys = [p.global_y for p in fp.pads]
        W = (min(xs) - 0.25, min(ys) - 0.25,
             max(xs) + 0.25, max(ys) + 0.25)
        RW = (min(xs) - 1.6, min(ys) - 1.6,
              max(xs) + 1.6, max(ys) + 1.6)
        balls = {}
        for p in fp.pads:
            nm = p.net_name.rsplit('/', 1)[-1] if p.net_name else ''
            if nm:
                balls[nm] = (p.global_x, p.global_y)
        deg, seglay = {}, {}
        for s in pcb.segments:
            nm = id2s.get(s.net_id)
            if nm not in balls:
                continue
            if not (RW[0] <= s.start_x <= RW[2]
                    and RW[1] <= s.start_y <= RW[3]):
                continue
            for pt in ((round(s.start_x, 3), round(s.start_y, 3)),
                       (round(s.end_x, 3), round(s.end_y, 3))):
                deg[(nm, pt)] = deg.get((nm, pt), 0) + 1
                seglay[(nm, pt)] = s.layer
        cx, cy = (W[0] + W[2]) / 2, (W[1] + W[3]) / 2
        stub_face = {}
        stub_end = {}
        ends = {}
        for (nm, pt), c in deg.items():
            if c != 1:
                continue
            d2 = (pt[0] - cx) ** 2 + (pt[1] - cy) ** 2
            if nm not in ends or d2 > ends[nm][0]:
                ends[nm] = (d2, pt)
        for nm, (_, pt) in ends.items():
            d = {'up': abs(pt[1] - W[1]), 'right': abs(pt[0] - W[2]),
                 'down': abs(pt[1] - W[3]), 'left': abs(pt[0] - W[0])}
            side = min(d, key=d.get)
            stub_face[nm] = side
            stub_end[nm] = (side, seglay[(nm, pt)],
                            pt[1] if side in ('left', 'right')
                            else pt[0])
        out[ref] = {'W': W, 'balls': balls, 'face': stub_face,
                    'end': stub_end}
    _GEOM[fo] = out
    return out


_NORMAL = {'up': (0.0, -1.0), 'down': (0.0, 1.0),
           'left': (-1.0, 0.0), 'right': (1.0, 0.0)}


def face_candidates(n, ref, fo):
    """General alternative escape faces for net n's end at `ref`: the
    three window faces it does NOT currently use, each with the coord
    where the straight line ball -> other-end crosses that face
    (clamped into the face span; the engine still negotiates the real
    slot), ordered by alignment with that line. Pure geometry -- no
    board knowledge."""
    g = _geom(fo)
    other_ref = 'DU1' if ref == _a.src_ref else _a.src_ref
    if ref not in g or other_ref not in g:
        return []
    ge, go = g[ref], g[other_ref]
    if n not in ge['balls'] or n not in go['balls']:
        return []
    bx, by = ge['balls'][n]
    ox, oy = go['balls'][n]
    dx, dy = ox - bx, oy - by
    W = ge['W']
    cur = ge['face'].get(n)
    cands = []
    for side, (nx, ny) in _NORMAL.items():
        if side == cur:
            continue
        # ray ball->other against this face's line
        if side in ('left', 'right'):
            edge = W[0] if side == 'left' else W[2]
            t = (edge - bx) / dx if abs(dx) > 1e-9 else -1
            raw = by + t * dy if t > 1e-9 else oy
            span = (W[1], W[3])
        else:
            edge = W[1] if side == 'up' else W[3]
            t = (edge - by) / dy if abs(dy) > 1e-9 else -1
            raw = bx + t * dx if t > 1e-9 else ox
            span = (W[0], W[2])
        coord = min(max(raw, span[0] + 0.3), span[1] - 0.3)
        norm = (dx * dx + dy * dy) ** 0.5 or 1.0
        align = (nx * dx + ny * dy) / norm
        cands.append((align, side, coord))
    cands.sort(reverse=True)
    return [(side, coord) for _, side, coord in cands]


FACE_BUDGET = int(os.environ.get('IMPROVE_FACE_ASKS', '30'))
_face_spent = 0

NEST_TOP = 3


def _nest_ask(m, tag, board, ask=None):
    """Translate a nest-model wrap move into a relay ask on `board`.
    With `ask` (the model's SHIFT-MINIMIZED position -- (end, side,
    coord) from analyze()['asks']) use it verbatim: the wrap exits
    only as far around as the orderings require (user-observed on the
    renders: the old back-third constant made a SE-corner ball walk
    its stub across the whole array). Fallback = back third."""
    end, wrap = tag.split('-')
    ref = _a.src_ref if end == 'src' else 'DU1'
    # NEST_MIN_ASKS=1: use the model's shift-minimized position.
    # GATED OFF as a default: the asks are measurably better copper
    # (3-4x cheaper wraps, the SE-ball-exits-north-west pathology
    # gone) but the changed accept ORDER regressed the K35 draw
    # 73 -> 78; re-earn per-rung before flipping on
    if ask is not None and os.environ.get('NEST_MIN_ASKS') == '1':
        _, side, coord = ask
    else:
        g = _geom(board).get(ref)
        if not g:
            return None
        W = g['W']
        w = W[2] - W[0]
        coord = W[0] + 0.25 * w if end == 'src' else W[2] - 0.25 * w
        side = 'up' if wrap == 'wrapN' else 'down'
    ride = rides.get(m) or 'F.Cu'
    return ((['--ref', 'DU1'] if ref == 'DU1' else [])
            + ['--side', side, '--coord', f'{coord:.2f}',
               '--layer', ride])


def nest_composed(nr, sweep):
    """The COMPOSED homotopy move (K41 measured the need: every SOLO
    wrap was rejected because it strands exactly the nets the OTHER
    recommended wraps would free -- the model's optimum is JOINT).
    Apply the descent's wrap set as ONE trial: relay each move onto
    the previous board (refusals skipped), then a single confirm
    braid through the shared accept/rescue tail. One braid for the
    whole batch; the per-move sweep remains the fallback."""
    moves = [(m, tag) for m, (tag, mm, solo) in
             sorted(nr['moves'].items(), key=lambda kv: -kv[1][2])
             if solo > 0][:6]
    if len(moves) < 2:
        return False
    tk = tried_key(('nestbatch', tuple(sorted(m for m, _ in moves))))
    if tk in TRIED:
        return False
    TRIED[tk] = True
    cur = FO
    applied = []
    for i, (m, tag) in enumerate(moves):
        largs = _nest_ask(m, tag, cur, ask=nr.get('asks', {}).get(m))
        if largs is None:
            continue
        out = f'{TAG}_s{sweep}nb{i}{m}.kicad_pcb'
        r = subprocess.run(
            [sys.executable, 'relay_net.py', cur, m, '--out', out]
            + largs, capture_output=True, text=True)
        if not os.path.exists(out) or 'MISSED' in r.stdout:
            continue                       # compose what applies
        cur = out
        applied.append(m)
    if len(applied) < 2:
        return False
    print(f'  nest composed: {len(applied)} wrap(s) '
          f'[{",".join(applied)}]')
    # NEST_WIDE_RESCUE=1: cap-4 + recursion batch rescue. GATED OFF:
    # mechanically sound (recovered 4->2 strands, 64v batch to 66v/2)
    # but the K35 draw regressed 73 -> 77/78 with it on -- the extra
    # early accepts steer the accept-and-build path into worse basins
    return _trial_on_fo(
        cur, f's{sweep}nestbatch',
        rescue='wide' if os.environ.get('NEST_WIDE_RESCUE') == '1'
        else True)


def nest_channel(sweep):
    """The HOMOTOPY channel: 'you can nest even more if the homotopy
    is adjusted at either the source or berth'. The nest model
    (plan_nest.analyze, cyclic boundary chords from the board's own
    escape crossings) recommends which nets to WRAP around which
    array; each recommendation, best solo crossing-reduction first,
    becomes a face-relay ask toward that array's BACK -- delivered
    by the same relay + screen + whole-board-confirm acceptance as
    every other channel. General: the back face and wrap side come
    from the model and the arrays' geometry, never from the bench."""
    if os.environ.get('IMPROVE_NEST', '1') != '1':
        return False
    try:
        from plan_nest import analyze
        nr = analyze(best_board, NETS.split(','), _a.src_ref, 'DU1',
                     minimize_shifts=(
                         os.environ.get('NEST_MIN_ASKS') == '1'))
    except Exception as e:
        print(f'  nest model unavailable ({e})')
        return False
    if not nr['moves']:
        return False
    cand = sorted(nr['moves'].items(),
                  key=lambda kv: (-kv[1][2], kv[1][1]))
    print(f'  nest model: {nr["as_is"]} -> {nr["minimum"]} '
          f'interleavings reachable; trying '
          f'{[m for m, _ in cand[:NEST_TOP]]}')
    # the JOINT batch FIRST -- the fa4..fa8 matrix (73/77/78/77/78)
    # says the ONLY 73v path runs THROUGH an early batch acceptance;
    # every reordering/widening experiment regressed the draw. The
    # committed default IS the record path; deviations live behind
    # NEST_WIDE_RESCUE / NEST_MIN_ASKS.
    if nest_composed(nr, sweep):
        return True
    got = False
    for m, (tag, mm, solo) in cand[:NEST_TOP]:
        if solo <= 0:
            continue
        end, _wrap = tag.split('-')
        ref = _a.src_ref if end == 'src' else 'DU1'
        base_largs = _nest_ask(m, tag, FO,
                               ask=nr.get('asks', {}).get(m))
        if base_largs is None:
            continue
        ride = rides.get(m)
        layers = ([ride, 'B.Cu' if ride == 'F.Cu' else 'F.Cu']
                  if ride else ['F.Cu', 'B.Cu'])
        cur_end = _geom(FO).get(ref, {}).get('end', {}).get(m)
        delivered = [cur_end] if cur_end else []
        side = base_largs[base_largs.index('--side') + 1]
        for L in layers:
            largs = list(base_largs)
            largs[largs.index('--layer') + 1] = L
            if relay_try(m, largs,
                         f's{sweep}nest{m}_{end}{side[0]}{L[0]}',
                         screen=True, rescue=True, dedupe=delivered):
                got = True
                break
        if got:
            break
    return got


def face_sweep(n, label, screen='strict'):
    """The POSITION channel: the fanout chose this net's escape face
    once (dest-anchored clustering) and no layer flip can fix a face
    that forces same-page crossings later. Offer the geometric
    alternatives at both ends, model's ride layer first; every ask
    goes through the same screen + whole-board-confirm acceptance."""
    global _face_spent
    if os.environ.get('IMPROVE_FACES', '1') != '1':
        return False
    ride = rides.get(n)
    layers_pref = ([ride, 'B.Cu' if ride == 'F.Cu' else 'F.Cu']
                   if ride else ['F.Cu', 'B.Cu'])
    for ref in (_a.src_ref, 'DU1'):
        # seed the delivery dedupe with the CURRENT stub end -- an
        # ask the engine walks back to today's geometry is a no-op
        cur_end = _geom(FO).get(ref, {}).get('end', {}).get(n)
        delivered = [cur_end] if cur_end else []
        for side, coord in face_candidates(n, ref, FO)[:2]:
            for L in layers_pref:
                largs = ((['--ref', 'DU1'] if ref == 'DU1' else [])
                         + ['--side', side, '--coord', f'{coord:.2f}',
                            '--layer', L])
                # a memo hit costs nothing -- don't spend budget on it
                # (audit finding #6: later sweeps re-offer identical
                # asks and exhausted the budget on instant rejects)
                if tried_key(('relay', n, tuple(largs))) in TRIED:
                    continue
                if _face_spent >= FACE_BUDGET:
                    return False
                _face_spent += 1
                ok = relay_try(n, largs,
                               f'{label}_{ref[0]}{side[0]}{L[0]}',
                               screen=screen, rescue=True,
                               dedupe=delivered)
                if getattr(relay_try, 'last_dup', False):
                    _face_spent -= 1    # a duplicate cost ~nothing
                if ok:
                    return True
    return False


def _strip_lane(board, fo_board, n, out):
    """Copy `board` with net n's LANE removed (everything of n not on
    the fanout board) -- the single-net screen's frozen world."""
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    from kicad_parser import parse_kicad_pcb
    fo_p = parse_kicad_pcb(fo_board)
    by = {net.name.split('/')[-1]: (i, net)
          for i, net in fo_p.nets.items()}
    nid, netname = by[n][0], by[n][1].name
    keep_sp = {(frozenset(((round(s.start_x, 3), round(s.start_y, 3)),
                           (round(s.end_x, 3), round(s.end_y, 3)))),
                s.layer)
               for s in fo_p.segments if s.net_id == nid}
    keep_via = {(round(v.x, 3), round(v.y, 3))
                for v in fo_p.vias if v.net_id == nid}

    def net_match(block):
        m = re.search(r'\(net (\d+)\)', block)
        if m:
            return int(m.group(1)) == nid
        m = re.search(r'\(net "([^"]+)"\)', block)
        return bool(m and m.group(1) == netname)

    def kill_seg(block):
        if not net_match(block):
            return False
        pts = re.findall(r'\((?:start|end) ([-\d.]+) ([-\d.]+)',
                         block)
        ml = re.search(r'\(layer "([^"]+)"\)', block)
        if len(pts) != 2 or not ml:
            return False
        key = (frozenset((round(float(x), 3), round(float(y), 3))
                         for x, y in pts), ml.group(1))
        return key not in keep_sp

    def kill_via(block):
        if not net_match(block):
            return False
        m = re.search(r'\(at ([-\d.]+) ([-\d.]+)\)', block)
        return bool(m) and (round(float(m.group(1)), 3),
                            round(float(m.group(2)), 3)) \
            not in keep_via

    txt = open(board, encoding='utf-8').read()
    txt = _walk_strip(txt, 'segment', kill_seg)
    txt = _walk_strip(txt, 'via', kill_via)
    open(out, 'w', encoding='utf-8').write(txt)


def screen_reject(n, v):
    """The user's 'don't redo the WHOLE route': strip n's lane off
    the current best board and braid n ALONE against the frozen rest
    under the flipped page (~8 s vs ~50 s). Rejects only a CLEAR
    loser (routable and no cheaper for the net itself); a frozen-
    world refusal or improvement pays the honest full braid, so
    accepts stay whole-board-confirmed."""
    try:
        scr = TAG + '_scr.kicad_pcb'
        _strip_lane(best_board, FO, n, scr)
        json.dump({n: v},
                  open(os.path.splitext(scr)[0] + '.pages.json', 'w'))
        r = subprocess.run(
            [sys.executable, 'braid.py', '--board', scr,
             '--dest', 'DU1', '--nets', n, '--out', TAG + '_scr1'],
            env=ENV, capture_output=True, text=True)
        b1 = TAG + '_scr1.kicad_pcb'
        if not os.path.exists(b1) or 'REFUSED' in r.stdout:
            return False
        scr_v = actual_vias(b1).get(n, 99)
        cur_v = actual_vias(best_board).get(n, 0)
        return scr_v >= cur_v
    except Exception:
        return False


# trial memo: the sweeps re-braid identical states (same FO, same
# pages -- e.g. a rejected flip retried every sweep at ~50 s each).
# A hit can only be a REJECT: best_score only falls, so a score that
# lost to an older best loses to the current one too.
TRIED = {}


def tried_key(extra=()):
    return (FO, tuple(sorted(flips.items())), tuple(extra))


# ---- step 1: free baseline (no sidecar)
if os.path.exists(SIDECAR):
    os.remove(SIDECAR)
best_score = braid(TAG + '_free')
best_board = TAG + '_free.kicad_pcb'
plan = dump_plan(TAG + '_plan.json')
own = {n: d['page'] for n, d in plan.items() if 'page' in d}
best_pages = None
print(f'free draw: open={best_score[0]} drc={best_score[1]} '
      f'vias={best_score[2]}')

# ---- step 2: pin own pages
put_pages(own)
s = braid(TAG + '_pin')
print(f'own pages pinned: open={s[0]} drc={s[1]} vias={s[2]}')
if s < best_score:
    best_score, best_board, best_pages = s, TAG + '_pin.kicad_pcb', dict(own)

# ---- steps 3-5: diagnose, targeted flips + relays, keep strictly
# better. `flips` records the ACCEPTED page intents by name so an
# accepted relay (which re-derives own pages from its new frame) can
# re-apply them on top.
rides, divers, model = opt_rides(plan)
cur_pages = dict(best_pages) if best_pages else dict(own)
flips = {}
RELAY_TOP = 4          # relay channels only for the worst few
for sweep in range(_a.num_improve):
    # OPENS channel first: an open net is the top waste (a refusal
    # measured B/B at K35 -- the B path walled at margin 6 -- has no
    # B-ward force move; the F-ward move is a RELAY). Tooth first,
    # then berth, to the opposite layer; the lexicographic keep means
    # closing an open always wins.
    g_ = subprocess.run(
        [sys.executable, 'grade_k.py', best_board, NETS],
        capture_output=True, text=True).stdout
    mo_ = re.search(r'open: ([A-Za-z0-9_,]+)', g_)
    for n in (mo_.group(1).split(',') if mo_ else [])[:3]:
        d = plan.get(n) or {}
        oth = 'F.Cu' if d.get('tooth_layer') == 'B.Cu' else 'B.Cu'
        ok = relay_try(n, ['--keep-pos', '--layer', oth],
                       f's{sweep}open_tooth{n}')
        if not ok:
            ok = relay_try(n, ['--ref', 'DU1', '--keep-pos',
                               '--layer', oth], f's{sweep}open_berth{n}')
        if not ok:
            # an open net whose layer relays failed may be FACE-walled
            # (K35's chronic refusal was B/B with no force move) --
            # unscreened: closing an open pays vias by design
            face_sweep(n, f's{sweep}openface{n}', screen=False)
    act = actual_vias(best_board)
    waste = sorted((n for n in plan
                    if act.get(n, 0) > model.get(n, 99)),
                   key=lambda n: model[n] - act.get(n, 0))
    print(f'sweep {sweep}: {len(waste)} waste nets '
          f'{[(n, act.get(n, 0), model[n]) for n in waste[:8]]}')
    improved = False
    fixed = set()
    near_miss = None       # (via_gain, flip_net, flip_val, stranded)
    for n in waste:
        variants = []
        if cur_pages.get(n) is None:
            if n not in divers:
                variants.append(rides[n])       # swimmer -> model page
        else:
            other = 'B.Cu' if cur_pages[n] == 'F.Cu' else 'F.Cu'
            if rides.get(n) == other:
                variants.append(other)          # paged -> model's layer
            variants.append(None)               # paged -> swimmer
        for v in variants:
            trial = dict(cur_pages)
            trial[n] = v
            tk = tried_key(('pages', n, v))
            if tk in TRIED:
                continue                    # a hit is always a reject
            tagv = v[0] if v else 'swim'
            if os.environ.get('IMPROVE_SCREEN', '1') == '1' \
                    and screen_reject(n, v):
                TRIED[tk] = 'screened'
                print(f'  {n} -> {tagv}: screened out (single-net)')
                continue
            put_pages(trial)
            s = braid(TAG + '_try')
            TRIED[tk] = s
            if s[0] > 0 and s[1] <= best_score[1] \
                    and s[2] < best_score[2]:
                gain = best_score[2] - s[2]
                if near_miss is None or gain > near_miss[0]:
                    near_miss = (gain, n, v,
                                 list(OPENS.get(TAG + '_try', [])))
            if s < best_score:
                print(f'  {n} -> {tagv}: open={s[0]} drc={s[1]} '
                      f'vias={s[2]}   ACCEPT')
                best_score = s
                cur_pages = trial
                flips[n] = v
                shutil.copy(TAG + '_try.kicad_pcb', TAG + '_best.kicad_pcb')
                # a chain board always has its .kicad_pro sibling;
                # a hand-supplied pro-less fanout board must not
                # crash the run mid-sweep -- but the missing DRC
                # floor is the #441 hazard, so say it loudly
                if os.path.exists(TAG + '_try.kicad_pro'):
                    shutil.copy(TAG + '_try.kicad_pro',
                                TAG + '_best.kicad_pro')
                else:
                    print('  WARNING: no .kicad_pro sibling on the '
                          'accepted board -- DRC floor not carried '
                          '(#441); grade at the routed clearance')
                best_board = TAG + '_best.kicad_pcb'
                improved = True
                fixed.add(n)
                break
            print(f'  {n} -> {tagv}: open={s[0]} drc={s[1]} '
                  f'vias={s[2]}')
    # the group move: the best via-saving flip that stranded nets.
    # STRANDED = the trial's opens MINUS the current best board's own
    # opens -- without the base subtraction any pre-existing open (the
    # normal state at high K) counts against the 1..3 gate and aims
    # the rescue at nets the flip never touched (audit finding #1:
    # K41 entered improve with 5 opens and this channel never fired)
    if near_miss is not None:
        gain, n0, v0, trial_opens = near_miss
        gb_ = subprocess.run(
            [sys.executable, 'grade_k.py', best_board, NETS],
            capture_output=True, text=True).stdout
        mb_ = re.search(r'open: ([A-Za-z0-9_,]+)', gb_)
        base_open = set(mb_.group(1).split(',')) if mb_ else set()
        stranded = [m for m in trial_opens if m not in base_open]
        if 1 <= len(stranded) <= 3:
            ok = group_try(n0, v0, stranded,
                           f's{sweep}grp{n0}_{"_".join(stranded)}')
            improved = improved or ok
    # relay channels: the worst waste nets the pages sweep left
    for n in [w for w in waste if w not in fixed][:RELAY_TOP]:
        d = plan.get(n) or {}
        ride = rides.get(n)
        ok = False
        if ride and d.get('tooth_layer') and d['tooth_layer'] != ride:
            ok = relay_try(n, ['--keep-pos', '--layer', ride],
                           f's{sweep}tooth{n}')
        if not ok and ride and d.get('dest_layer') \
                and d['dest_layer'] != ride:
            ok = relay_try(n, ['--ref', 'DU1', '--keep-pos',
                               '--layer', ride], f's{sweep}berth{n}')
        if not ok:
            p = swap_partner(n)
            if p:
                ok = relay_try(f'{n},{p}', ['--swap', '--layer', 'keep'],
                               f's{sweep}swap{n}_{p}')
        if not ok and act.get(n, 0) - model.get(n, 99) >= 2:
            # a net paying 2+ vias over its own model floor is diving
            # mid-ride: the escape FACE the fanout chose forces
            # same-page crossings no in-place layer flip can undo
            ok = face_sweep(n, f's{sweep}face{n}')
        improved = improved or ok
    # the homotopy channel LAST: it is model-aimed (joint), so give
    # the targeted per-net channels first shot at their own diagnosis
    improved = nest_channel(sweep) or improved
    if not improved:
        break

# trials braided --no-smooth; re-braid the WINNER once with #536
# smoothing (bit-deterministic braid -> the tuple must reproduce;
# kept unsmoothed with a warning if it ever does not)
if best_board.endswith('_free.kicad_pcb'):
    if os.path.exists(SIDECAR):
        os.remove(SIDECAR)
else:
    put_pages(cur_pages)
_s = braid(TAG + '_smooth', smooth=True)
if _s == best_score:
    best_board = TAG + '_smooth.kicad_pcb'
else:
    print(f'  final smooth re-braid diverged (open={_s[0]} '
          f'drc={_s[1]} vias={_s[2]} vs {best_score}); '
          'keeping the unsmoothed best')
if os.path.exists(SIDECAR):
    os.remove(SIDECAR)
print(f'\nBEST: {best_board} open={best_score[0]} drc={best_score[1]} '
      f'vias={best_score[2]}')

# ---- achieved teeth -> anchors. The accepted moves are knowledge the
# SOLVER never hears about; `plan_global.py solve --anchor <this>`
# pulls the next plan toward what the engine demonstrably delivered.
try:
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    from kicad_parser import parse_kicad_pcb
    fp = parse_kicad_pcb(FO).footprints[_a.src_ref]
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    anch = {}
    for n, d in plan.items():
        t = d.get('tooth')
        if not t:
            continue
        tx, ty = t
        dd = {'left': x0 - tx, 'right': tx - x1,
              'up': y0 - ty, 'down': ty - y1}
        side = max(dd, key=lambda k: dd[k])
        anch[n] = {'side': side,
                   'coord': round(ty if side in ('left', 'right')
                                  else tx, 3)}
    apath = TAG + '_anchors.json'
    json.dump(anch, open(apath, 'w'), indent=1, sort_keys=True)
    print(f'anchors for the next solve: {apath} ({len(anch)} teeth)')
except Exception as e:      # the emit must never cost the run
    print(f'anchors emit skipped ({e})')
