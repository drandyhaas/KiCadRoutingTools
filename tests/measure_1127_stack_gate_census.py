#!/usr/bin/env python3
"""#1127: what the box stack gate refuses that check_assembly would accept,
and whether confirming it moves the placement A/B.

PRE-REGISTERED here, committed before the first run:

* boards: esp_prog, splitflap_driver, tigard, glasgow_revC, ulx3s,
  orangecrab_ext_pll, watchy, rp2350_fpga_eensy_prePlane,
  kit-dev-coldfire-xilinx_5213 (kicad_files/), plus KiCad 10's StickHub demo
  when the install is present (diagonal parts; read-only, never committed);
* engines: the seed (`test_placement_ab._run_seed`, the auto intent emitted
  from the board, `random.Random('0')`) and the quench
  (`test_placement_ab._run`, QUENCH_BASE, the same intent), each OFF vs ON,
  where ON is `legality.STACK_EXACT_CONFIRM` set around the engine call only
  (`engine_flags`); GND ignored in the guard columns;
* the census: every box hit the ON context confirms or not
  (`LegalityContext._stack_confirmed`), split by same / different net, by
  whether either part sits off the 90-degree lattice, and by whether a part is
  a 2-pad C* (a cap -- the #1105 population);
* the A/B, per engine, with `test_placement_ab._verdict`: seed rows on signal
  `intent_errors`, guards crossings, hpwl, unseated, body_blocking; quench rows
  on signal `crossings`, guards hpwl, body_blocking, body_advisory. The trial
  boards of an engine are the pre-registered boards whose ON census records at
  least one FLIP (a box hit the exact check clears) in that engine -- a
  mechanism criterion, fixed before any outcome is seen;
* GO, per engine: N >= 3 trial boards, improve on >= N-1, regress on none
  (CLAUDE.md's rule). The toggle ships default-on only if the engine whose
  flips the gate actually changes passes, and the other does not regress.

Soundness, reported alongside: the ON arms' `body_blocking` (check_assembly's
pad_intersection channel) never above the OFF arms', i.e. the confirmation
never admits a stack the exact channel would block.

Measured at 39affafb, and identically on an earlier build of the branch
apart from wall times (#1127 has the table): NO-GO for both engines. On 6 of
the 9 corpus boards neither engine reaches a box stack at all, and on 7 of 9
confirming flips none (orangecrab's seed reaches 16 and flips 0); rp2350 and
ulx3s flip and stay neutral; on StickHub (diagonal parts) the seed's
body_blocking goes 6 -> 0 while its crossings rise, and the quench adds an
edge_connector error. ON never raised body_blocking above OFF.

Not collected by run_all (no `test_` prefix). About an hour.

    python3 -X utf8 tests/measure_1127_stack_gate_census.py [--workdir DIR]
        [--json-out PATH] [--board NAME ...] [--engine seed|quench]

Exit 0 = GO, 1 = NO-GO, 2 = the measurement could not be taken, 3 = partial.
"""
import argparse
import json
import os
import sys
import tempfile
import time

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TESTS_DIR)
import test_placement_ab as AB                      # noqa: E402

BOARDS = ('esp_prog.kicad_pcb', 'splitflap_driver.kicad_pcb',
          'tigard.kicad_pcb', 'glasgow_revC.kicad_pcb', 'ulx3s.kicad_pcb',
          'orangecrab_ext_pll.kicad_pcb', 'watchy.kicad_pcb',
          'rp2350_fpga_eensy_prePlane.kicad_pcb',
          'kit-dev-coldfire-xilinx_5213.kicad_pcb')
# The demo through test_1094's resolver (KICAD_STICKHUB_DEMO, then the
# Windows, Linux and macOS install paths). It used to be one hard-coded
# Windows path, omitted without a word anywhere else.
from test_1094_rotated_courtyards import stickhub  # noqa: E402
ENGINES = {
    'seed': {'signal': 'intent_errors',
             'guard': ('crossings', 'hpwl', 'unseated', 'body_blocking')},
    'quench': {'signal': 'crossings',
               'guard': ('hpwl', 'body_blocking', 'body_advisory')},
}
ON = {'legality.STACK_EXACT_CONFIRM': True}
OFF = {'legality.STACK_EXACT_CONFIRM': False}


class _Census:
    """Wraps `LegalityContext._stack_confirmed` and tallies every call."""

    def __init__(self):
        from placement import legality
        self.legality = legality
        self.real = legality.LegalityContext._stack_confirmed
        self.rows = []

    def __enter__(self):
        real, rows = self.real, self.rows

        def spy(ctx, a, ai, pose_a, b, bi, pose_b):
            ok = real(ctx, a, ai, pose_a, b, bi, pose_b)
            pa, pb = ctx.parts.get(a), ctx.parts.get(b)
            na = pa.pads_local[ai][4] if pa and ai < len(pa.pads_local) else 0
            nb = pb.pads_local[bi][4] if pb and bi < len(pb.pads_local) else 0
            diag = any(abs(((p[2] % 90.0) + 45.0) % 90.0 - 45.0) > 1e-6
                       for p in (pose_a, pose_b))
            cap = any(r.startswith('C') and ctx.parts[r].n_pads == 2
                      for r in (a, b) if r in ctx.parts)
            rows.append({'confirmed': bool(ok),
                         'same_net': bool(na and na == nb),
                         'diagonal': diag, 'cap': cap})
            return ok
        self.legality.LegalityContext._stack_confirmed = spy
        return self

    def __exit__(self, *exc):
        self.legality.LegalityContext._stack_confirmed = self.real
        return False

    def summary(self):
        flips = [r for r in self.rows if not r['confirmed']]
        return {'calls': len(self.rows),
                'confirmed': len(self.rows) - len(flips),
                'flips': len(flips),
                'flips_same_net': sum(r['same_net'] for r in flips),
                'flips_diagonal': sum(r['diagonal'] for r in flips),
                'flips_cap': sum(r['cap'] for r in flips)}


def _boards(only):
    out = [os.path.join(AB.BOARDS, b) for b in BOARDS]
    sh = stickhub()
    if sh:
        out.append(sh)
    else:
        print("StickHub demo not found (KICAD_STICKHUB_DEMO or the install "
              "paths): its diagnostic rows are omitted", flush=True)
    if only:
        out = [b for b in out if os.path.basename(b) in only]
    return out


def measure(boards, engines, work):
    res = {}
    for board in boards:
        name = os.path.splitext(os.path.basename(board))[0]
        d = os.path.join(work, name)
        os.makedirs(d, exist_ok=True)
        intent = AB._intent_for(board, [], d)
        rec = res[name] = {}
        for eng in engines:
            t0 = time.time()
            out_off = os.path.join(d, f'{eng}_off.kicad_pcb')
            out_on = os.path.join(d, f'{eng}_on.kicad_pcb')
            if eng == 'seed':
                off = AB._run_seed(board, out_off, intent, {},
                                   ignore_nets=['GND'], engine_flags=OFF)
                with _Census() as cen:
                    on = AB._run_seed(board, out_on, intent, {},
                                      ignore_nets=['GND'], engine_flags=ON)
            else:
                kw = dict(AB.QUENCH_BASE, ignore_nets=['GND'])
                off = AB._run(board, out_off, intent, kw, engine_flags=OFF)
                with _Census() as cen:
                    on = AB._run(board, out_on, intent, kw, engine_flags=ON)
            mark, notes = AB._verdict(off, on, ENGINES[eng])
            rec[eng] = {'census': cen.summary(), 'mark': mark, 'notes': notes,
                        'off': {k: off.get(k) for k in (
                            'seconds', 'intent_errors', 'crossings', 'hpwl',
                            'unseated', 'body_blocking', 'body_advisory')},
                        'on': {k: on.get(k) for k in (
                            'seconds', 'intent_errors', 'crossings', 'hpwl',
                            'unseated', 'body_blocking', 'body_advisory')},
                        'wall': round(time.time() - t0, 1)}
            c = rec[eng]['census']
            print(f"{name} [{eng}] {mark.upper():<8} flips {c['flips']} of "
                  f"{c['calls']} box hit(s) (same-net {c['flips_same_net']}, "
                  f"diagonal {c['flips_diagonal']}, cap {c['flips_cap']}) | "
                  f"OFF {rec[eng]['off']} | ON {rec[eng]['on']}", flush=True)
            for n in notes:
                print(f"    {n}", flush=True)
    return res


def verdict(res, eng):
    trial = [b for b, r in res.items()
             if eng in r and r[eng]['census']['flips'] > 0]
    marks = [res[b][eng]['mark'] for b in trial]
    n = len(trial)
    imp, reg = marks.count('improve'), marks.count('regress')
    go = n >= 3 and reg == 0 and imp >= max(1, n - 1)
    return go, trial, imp, reg


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--workdir', default=None)
    ap.add_argument('--json-out', default=None)
    ap.add_argument('--board', action='append', default=None)
    ap.add_argument('--engine', action='append', default=None,
                    choices=sorted(ENGINES))
    a = ap.parse_args(argv)
    engines = a.engine or list(ENGINES)
    work = a.workdir or tempfile.mkdtemp(prefix='m1127_')
    try:
        res = measure(_boards(a.board), engines, work)
    except Exception as exc:                          # noqa: BLE001 - exit 2
        print(f"MEASUREMENT NOT TAKEN: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        return 2
    if a.json_out:
        with open(a.json_out, 'w', encoding='utf-8') as fh:
            json.dump(res, fh, indent=1, default=str)
    unsound = [(b, e) for b, r in res.items() for e, x in r.items()
               if (x['on']['body_blocking'] or 0) > (x['off']['body_blocking']
                                                      or 0)]
    print(f"\nsoundness: ON body_blocking above OFF on {unsound or 'no run'}")
    if a.board or a.engine:
        print("partial run: no GO verdict")
        return 3
    ok = True
    for eng in engines:
        go, trial, imp, reg = verdict(res, eng)
        print(f"{eng}: {'GO' if go else 'NO-GO'} -- {len(trial)} trial "
              f"board(s) with flips {trial}; improve {imp}, regress {reg}")
        ok = ok and go
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
