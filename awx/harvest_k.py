#!/usr/bin/env python3
"""#622 SLACK HARVEST, the stronger one (item 1 of the 0902 brief).

The calibrated ledger splits every net's vias into a FLOOR (what its
crossings force) plus SLACK (realization waste). floor_evolve's
descent moves the floor and then harvests slack by re-braiding each
slack net alone, IN PLACE, with the stubs it already has. Measured
at K41 that harvest stalls at slack 14: seven nets each two vias over
their floor. Their anatomy (fev41b_final): a B dogbone stub at one
end feeding an F ride -- the dogbone via plus its return via are the
whole slack, and no in-place re-braid can remove them because the
stub's layer is not the braid's to change.

The sweep already GENERATES the fix (keep-position relays onto the
other layer, the alt faces) and realizes every one of them on the
frozen world -- then discards them, because its acceptance test is a
FLOOR drop and a stub relay leaves the floor flat by construction
(the floor's terminals are the PAD layers). This loop judges the same
candidates by what the brief's objective actually is: REALIZED vias.

First, COMPLETION: every K net the grade reports open is braided
alone against the frozen record (its stubs restored from the fanout
board) and kept when the open count drops -- measured before any
harvest existed: SA2, K41's chronic open (its composed closes spiked
10 drc inside the whole-board loops), routes in the single-net last
call at +5 vias, 0 drc, on the 79v record. The evolve loops never
tried it because a net with NO path has no row in the judge's table,
and every loop iterated that table.

Then per slack net (act > dp), a ladder on the frozen world, first
accept wins:
  0. in-place re-braid (floor_evolve's harvest);
  1. keep-position layer flips at each end -- the ride layer near
     that end first -- then both ends at once;
  2. the alt faces x layers at each end;
Then --pairs: two slack nets that cross each other, both stripped,
braided TOGETHER (a crossing the single-net braid must dive under
may be resolvable when both lanes move). Then the closing passes
drive_k already runs (collapse_dives, nudge_grazes to convergence).

Acceptance is lexicographic (open, drc) never worse and vias strictly
fewer; the judge's floor is REPORTED, not gated (a floor rise with
fewer real vias is a worse assignment better realized, and vias are
the objective). Every accept is a whole-board grade.

usage: harvest_k.py TAG BEST_BOARD FO_BOARD K [--passes 2] [--pairs]
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)
from ledger_cal import Judge  # noqa: E402
import surgical as sg         # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('--passes', type=int, default=2)
ap.add_argument('--pairs', action='store_true',
                help='after the single-net ladder, braid crossing '
                     'slack pairs together')
ap.add_argument('--no-faces', action='store_true',
                help='skip rung 2 (the 16 face asks per net)')
ap.add_argument('--src', default='U1')
ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
NETS = sg.k_nets(a.k)
nets = NETS.split(',')
T0 = time.time()


def say(msg):
    print(f'[{time.time() - T0:6.0f}s] {msg}', flush=True)


def ride_layer_near(J, nm, ref):
    """The layer the net's path rides ~2 mm in from the given end
    (past its stub) -- the layer a keep-pos flip at that end should
    ask for first."""
    P = J.paths[nm]
    L = P.cum[-1]
    t = min(2.0, L / 2) if ref == a.src else max(L - 2.0, L / 2)
    return P.layer_at(t)


def realize(best, fo_for_stubs, names, stem):
    """Strip `names` from best, restore their stubs from
    fo_for_stubs, braid them alone. Returns the board path or None."""
    scr = f'tmp/{a.tag}_scr.kicad_pcb'
    cur = best
    for i, nm in enumerate(names):
        nxt = scr if i == len(names) - 1 else f'tmp/{a.tag}_scr{i}.kicad_pcb'
        sg.swap_stub(cur, fo_for_stubs, nm, nxt)
        cur = nxt
    if not sg.braid_one(scr, ','.join(names), stem, dst=a.dst):
        return None
    return stem + '.kicad_pcb'


def accept_if_better(cand, label, best, g0, J):
    """Grade the realized candidate; return the new (best, g0, J) on
    an accept, else None. A candidate that saves vias but STRANDS a
    net gets one completion rescue (single-net close of each stranded
    net on its frozen copper) before the veto."""
    gc, opens = sg.grade_full(cand, NETS)
    if gc[0] > g0[0] and gc[1] <= g0[1] and gc[2] < g0[2]:
        _gb, base_open = sg.grade_full(best, NETS)
        cand, gc2, _o = sg.rescue_close(cand, fo, base_open, NETS,
                                        a.tag, a.dst)
        say(f'  {label}: strands {[m for m in opens if m not in base_open]}'
            f' -> rescue {gc} -> {gc2}')
        gc = gc2
    if (gc[0], gc[1]) <= (g0[0], g0[1]) and gc[2] < g0[2]:
        Jc = Judge(cand, nets, a.src, a.dst)
        keep = f'tmp/{a.tag}_acc_{label}.kicad_pcb'
        shutil.copy(cand, keep)
        say(f'  ACCEPT {label}: grade {g0} -> {gc}, floor '
            f'{J.floor_total} -> {Jc.floor_total}, changes '
            f'{Jc.act_total}')
        return keep, gc, Jc
    say(f'  {label}: {gc} (no)')
    return None


best, fo = a.best, a.fo
g0 = sg.grade(best, NETS)
J = Judge(best, nets, a.src, a.dst)
say(f'start {os.path.basename(best)}: grade {g0} floor {J.floor_total} '
    f'changes {J.act_total} slack {J.act_total - J.floor_total}')
menu = sg.Menu(fo, a.src, a.dst)

for pss in range(a.passes):
    # completion first: open K nets braided alone on the frozen world
    _g, opens = sg.grade_full(best, NETS)
    for m in opens:
        say(f'-- open {m}: single-net close')
        cand = realize(best, fo, [m], f'tmp/{a.tag}_b1')
        if cand is None:
            say(f'  {m}: braid refused')
            continue
        gc = sg.grade(cand, NETS)
        if gc[0] < g0[0] and gc[1] <= g0[1]:
            J = Judge(cand, nets, a.src, a.dst)
            keep = f'tmp/{a.tag}_acc_{m}_close.kicad_pcb'
            shutil.copy(cand, keep)
            say(f'  ACCEPT {m}_close: grade {g0} -> {gc}, floor '
                f'{J.floor_total}, changes {J.act_total}')
            best, g0 = keep, gc
        else:
            say(f'  {m}_close: {gc} (no)')
    slack = sorted(((J.paths[m].changes - J.per[m], m)
                    for m in J.paths), reverse=True)
    slack = [(d, m) for d, m in slack if d > 0]
    say(f'== pass {pss}: slack nets '
        + ', '.join(f'{m}+{d}' for d, m in slack))
    moved = False
    for d, m in slack:
        if J.paths[m].changes - J.per[m] <= 0:
            continue  # closed by an earlier accept this pass
        say(f'-- {m} (act {J.paths[m].changes} dp {J.per[m]})')
        # rung 0: in place
        cand = realize(best, fo, [m], f'tmp/{a.tag}_b1')
        r = cand and accept_if_better(cand, f'{m}_inplace', best, g0, J)
        if r:
            best, g0, J = r
            moved = True
            continue
        # rung 1: keep-pos flips, ride layer near the end first, then
        # both ends together
        ladder = []
        both = []
        for ref in (a.src, a.dst):
            if not menu.ball(ref, m):
                continue
            ride = ride_layer_near(J, m, ref)
            other = 'B.Cu' if ride == 'F.Cu' else 'F.Cu'
            ladder.append(menu.keep_pos(m, ref, ride))
            ladder.append(menu.keep_pos(m, ref, other))
            both.append((ref, ride))
        if len(both) == 2:
            ladder.append(('bothkp', both))
        if not a.no_faces:
            for ref in (a.src, a.dst):
                ladder.extend(menu.faces(m, ref))
        seen = set()
        for label, largs in ladder:
            relfo = f'tmp/{a.tag}_{m}_{label}_fo.kicad_pcb'
            if label == 'bothkp':
                # two relays in sequence, each on the other's output
                step = fo
                key = []
                for (ref, L) in largs:
                    lab2, la2 = menu.keep_pos(m, ref, L)
                    out2 = f'tmp/{a.tag}_{m}_{label}_{lab2}_fo.kicad_pcb'
                    k2 = sg.relay(step, m, la2, out2)
                    if k2 is None:
                        step = None
                        break
                    key.append(k2)
                    step = out2
                if step is None:
                    say(f'  {m} {label}: relay missed')
                    continue
                shutil.copy(step, relfo)
                k2 = tuple(key)
            else:
                k2 = sg.relay(fo, m, largs, relfo)
                if k2 is None:
                    say(f'  {m} {label}: relay missed')
                    continue
            if k2 in seen:
                continue
            seen.add(k2)
            rr = sg.realize_relay(best, fo, m, relfo, f'tmp/{a.tag}_b1',
                                  dst=a.dst)
            if rr is None:
                say(f'  {m} {label}: braid refused')
                continue
            cand = rr[0]
            if len(rr[1]) > 1:
                say(f'  {m} {label}: moved with blockers {rr[1][1:]}')
            r = accept_if_better(cand, f'{m}_{label}', best, g0, J)
            if r:
                best, g0, J = r
                fo = relfo
                menu = sg.Menu(fo, a.src, a.dst)
                moved = True
                break
    if not moved:
        say(f'  pass {pss}: nothing harvested, stopping')
        break

if a.pairs:
    say(f'== pairs: floor {J.floor_total} changes {J.act_total} '
        f'grade {g0}')
    slack = {m for m in J.paths if J.paths[m].changes - J.per[m] > 0}
    done = set()
    for m in sorted(slack):
        partners = sorted({o for (_t, o, _pl) in J.seqs[m]})
        for o in partners:
            pair = tuple(sorted((m, o)))
            if pair in done:
                continue
            done.add(pair)
            if J.paths[m].changes - J.per[m] <= 0:
                break
            say(f'-- pair {m}+{o}')
            cand = realize(best, fo, list(pair), f'tmp/{a.tag}_b2')
            if cand is None:
                say(f'  {m}+{o}: braid refused')
                continue
            r = accept_if_better(cand, f'{m}_{o}_pair', best, g0, J)
            if r:
                best, g0, J = r

# ---- closing passes (drive_k's tail): collapse dives, nudge grazes
out = f'tmp/{a.tag}_final.kicad_pcb'
r3 = subprocess.run(
    [sys.executable, 'collapse_dives.py', best, '--out', out],
    capture_output=True, text=True)
say('collapse: ' + (r3.stdout.strip().splitlines() or ['?'])[-1])
if not os.path.exists(out):
    shutil.copy(best, out)
for _i in range(3):
    out_n = f'tmp/{a.tag}_nudge.kicad_pcb'
    rn = subprocess.run(
        [sys.executable, 'nudge_grazes.py', out, '--out', out_n],
        capture_output=True, text=True)
    if rn.returncode != 0 or f'wrote {out_n}' not in rn.stdout:
        break
    os.replace(out_n, out)
    if '-> 0' in rn.stdout:
        break
pro = os.path.splitext(fo)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(out)[0] + '.kicad_pro')
gf = sg.grade(out, NETS)
Jf = Judge(out, nets, a.src, a.dst)
say(f'FINAL {out}: grade {gf} floor {Jf.floor_total} changes '
    f'{Jf.act_total} slack {Jf.act_total - Jf.floor_total} '
    f'(fo {fo})')
