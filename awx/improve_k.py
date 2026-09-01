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
              -- exchange the two proven teeth (--swap).
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


def braid(out):
    r = subprocess.run(
        [sys.executable, 'braid.py', '--board', FO, '--dest', 'DU1',
         '--nets', NETS, '--out', out], env=ENV,
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
    own2 = {k: d.get('page') for k, d in plan2.items()}
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


def relay_try(nets_arg, largs, label):
    """One relay trial, accept-and-build: relay the fanout, re-dump
    the plan FROM THE RELAYED BOARD, pin its own pages plus every
    accepted flip, braid, keep strictly better. On accept the relayed
    board becomes the base every later trial builds on."""
    global FO, SIDECAR, best_score, best_board, plan, own, cur_pages
    global rides, divers, model
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
    fo0, side0 = FO, SIDECAR
    FO = out
    SIDECAR = os.path.splitext(FO)[0] + '.pages.json'
    plan2 = dump_plan(f'{TAG}_{label}_plan.json')
    own2 = {m: d.get('page') for m, d in plan2.items()}
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
own = {n: d.get('page') for n, d in plan.items()}
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
            relay_try(n, ['--ref', 'DU1', '--keep-pos',
                          '--layer', oth], f's{sweep}open_berth{n}')
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
                shutil.copy(TAG + '_try.kicad_pro', TAG + '_best.kicad_pro')
                best_board = TAG + '_best.kicad_pcb'
                improved = True
                fixed.add(n)
                break
            print(f'  {n} -> {tagv}: open={s[0]} drc={s[1]} '
                  f'vias={s[2]}')
    # the group move: the best via-saving flip that stranded nets
    if near_miss is not None and 1 <= len(near_miss[3]) <= 3:
        gain, n0, v0, stranded = near_miss
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
        improved = improved or ok
    if not improved:
        break

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
