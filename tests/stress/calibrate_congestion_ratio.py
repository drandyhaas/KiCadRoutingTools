#!/usr/bin/env python3
"""What ratio, if any, separates a healthy placement repair from a legality-only one?

    python3 -X utf8 tests/stress/calibrate_congestion_ratio.py --out wk/calibration
    python3 -X utf8 tests/stress/calibrate_congestion_ratio.py --boards neo6502 --quick

P-close refuses a close-out when ``hpwl_gain < ratio * halo_gain``, where each
gain is ``(before - after) / before`` of a ``render_placement`` metric. The
``ratio`` shipped at **0.25, chosen and not measured**. This script measures it,
and is allowed to conclude the gate is wrong.

Why a ratio and not an absolute
-------------------------------
A blind run cannot see the undamaged control, so "how much of the DAMAGE did we
repair" is not computable at close-out time. What is computable is the
disparity between two gains against the board the run started from: if halo
closed most of its gap while hpwl closed almost none, the repair was local and
the arrangement was left alone. That is run 15's shape (halo 49.4%, hpwl 3.1%)
and the board it produced failed routing on 29 nets no parameter could reach.

Three populations, because a threshold needs both sides
-------------------------------------------------------
  undamaged -> itself      halo_gain ~ 0. The ratio must never be applied here;
                           this exercises the `halo_gain < 0.25` early-out.
  damaged -> repaired      the signal. What ratio do HEALTHY repairs reach?
  damaged -> legality-only  run 15's shape. Must be REFUSED, or the gate is
                           worthless.

"Healthy" is defined by LEGALITY OUTCOME (check_assembly blocking -> 0), never
by the metric under test. Defining it by hpwl would make this circular.

The knob trap
-------------
``halo`` is a PENALTY, so it depends on halo_base/halo_coef/halo_weight and the
clearance. ``render_placement._build_state`` uses halo_coef 0.25 /
crossing_penalty 10.0 / length_weight 1.0; ``pose_score.make_state`` (behind
``recovery.static_metrics``) uses 0.15 / 30.0 / 0.3, and discards halo anyway.
**A halo from anywhere but PlacementModel is not comparable to the gate's.** So
this script builds PlacementModel directly -- which is byte-identical to
``--json-out``'s metrics block by construction (render_placement.py: the payload
is literally ``dict(model.metrics)``) and costs ~0.2-0.8 s with no PNGs.

Any ratio measured here is therefore only valid for those weights. That is a
finding about the gate, and it belongs in the write-up.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

CORPUS = os.path.expanduser('~/Documents/kicad_stress_test/boards_unrouted_set1')

#: qualify.json (run 15, --draws 8, graded at 0.2) rated exactly these GOOD.
#: haxophone and upsy_desky came back WEAK -- the dose lands on fewer than 8 in
#: 10 draws -- so a calibration point from them would mostly measure the rig.
GOOD = ('neo6502', 'urchin', 'piantor')


def sh(cmd, timeout=1800):
    """(rc, stdout+stderr). Never raises on a tool's own failure."""
    env = dict(os.environ, KRT_NO_BANNER='1')
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           encoding='utf-8', errors='replace', timeout=timeout,
                           cwd=ROOT, env=env)
        return p.returncode, p.stdout
    except subprocess.TimeoutExpired:
        return 124, f'TIMEOUT after {timeout}s'


def metrics(board, clearance):
    """halo/hpwl/crossings exactly as the gate sees them.

    PlacementModel, not recovery.static_metrics -- see "the knob trap" above.
    """
    from kicad_parser import parse_kicad_pcb
    from render_placement import PlacementModel
    m = PlacementModel(parse_kicad_pcb(board), board, exact=True,
                       quench_kwargs={'clearance': clearance,
                                      'ignore_net_ids': set()}).metrics
    return {k: m.get(k) for k in ('halo', 'hpwl', 'crossings', 'total')}


def board_floor(board):
    """The board's own Default-netclass clearance. Never a round number."""
    pro = os.path.splitext(board)[0] + '.kicad_pro'
    try:
        d = json.load(open(pro, encoding='utf-8'))
        for c in d['net_settings']['classes']:
            if c.get('name') == 'Default':
                return float(c.get('clearance') or 0.2)
    except Exception:                                           # noqa: BLE001
        pass
    return 0.2


def assembly_blocking(board, clearance):
    """check_assembly's blocking count -- the LEGALITY definition of healthy."""
    with tempfile.TemporaryDirectory() as t:
        j = os.path.join(t, 'a.json')
        rc, _ = sh([sys.executable, '-X', 'utf8',
                    os.path.join(ROOT, 'py_tools', 'check_assembly.py'), board,
                    '--clearance', str(clearance), '--json', j], timeout=900)
        try:
            return int(json.load(open(j, encoding='utf-8')).get('blocking'))
        except Exception:                                       # noqa: BLE001
            return None


def gains(before, after):
    """(before-after)/before per key, the gate's own arithmetic."""
    out = {}
    for k in ('halo', 'hpwl', 'crossings'):
        b, n = before.get(k), after.get(k)
        ok = (isinstance(b, (int, float)) and isinstance(n, (int, float))
              and b > 0)
        out[k] = ((b - n) / b) if ok else None
    return out


def run_board(name, out_dir, quick=False):
    """Damage, repair two ways, and measure every step. Returns a row dict."""
    src = os.path.join(CORPUS, f'{name}.kicad_pcb')
    if not os.path.isfile(src):
        return {'board': name, 'error': f'no such board: {src}'}
    floor = board_floor(src)
    wk = os.path.join(out_dir, name)
    truth = os.path.join(out_dir, '_truth', name)
    for d in (wk, truth):
        os.makedirs(d, exist_ok=True)
    row = {'board': name, 'clearance': floor}

    t0 = time.time()
    rc, log = sh([sys.executable, '-X', 'utf8',
                  os.path.join(ROOT, 'tests/stress/stage_blind.py'),
                  src, wk, truth, '--kind', 'swap'], timeout=1800)
    dmg = os.path.join(wk, 'board.kicad_pcb')
    if rc != 0 or not os.path.isfile(dmg):
        return dict(row, error=f'stage_blind rc={rc}: {log.strip()[-300:]}')
    row['stage_seconds'] = round(time.time() - t0, 1)

    row['pristine'] = metrics(src, floor)
    row['damaged'] = metrics(dmg, floor)
    row['pristine_blocking'] = assembly_blocking(src, floor)
    row['damaged_blocking'] = assembly_blocking(dmg, floor)

    # POPULATION 1 -- undamaged against itself. The gate must not even reach
    # its ratio here (halo_gain ~ 0 trips the early-out).
    row['g_null'] = gains(row['pristine'], row['pristine'])

    # POPULATION 2 -- a REAL repair. The full ladder is what a placement half
    # actually runs; `legalize` is the stage that clears blocking pairs.
    full = os.path.join(wk, 'repaired.kicad_pcb')
    t0 = time.time()
    rc, log = sh([sys.executable, '-X', 'utf8',
                  os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'), dmg, full,
                  '--clearance', str(floor),
                  '--deadline', '300' if quick else '900'],
                 timeout=400 if quick else 1200)
    row['repair_rc'] = rc
    row['repair_seconds'] = round(time.time() - t0, 1)
    if os.path.isfile(full):
        row['repaired'] = metrics(full, floor)
        row['repaired_blocking'] = assembly_blocking(full, floor)
        row['g_repair'] = gains(row['damaged'], row['repaired'])
    else:
        row['repair_error'] = log.strip()[-300:]

    # POPULATION 3 -- legality only. `legalize` alone moves parts just enough to
    # clear violations and does nothing about the arrangement: run 15's shape,
    # reproduced deliberately rather than waited for.
    lego = os.path.join(wk, 'legality_only.kicad_pcb')
    t0 = time.time()
    rc, log = sh([sys.executable, '-X', 'utf8',
                  os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'), dmg, lego,
                  '--stages', 'classify,legalize', '--clearance', str(floor),
                  '--deadline', '300' if quick else '900'],
                 timeout=400 if quick else 1200)
    row['legality_rc'] = rc
    row['legality_seconds'] = round(time.time() - t0, 1)
    if os.path.isfile(lego):
        row['legality_only'] = metrics(lego, floor)
        row['legality_only_blocking'] = assembly_blocking(lego, floor)
        row['g_legality'] = gains(row['damaged'], row['legality_only'])
    else:
        row['legality_error'] = log.strip()[-300:]
    return row


def verdict_line(g, ratio):
    """What the gate would DO with these gains, in the gate's own logic."""
    if not g or g.get('halo') is None or g.get('hpwl') is None:
        return 'unmeasurable'
    if g['halo'] < 0.25:
        return 'pass (early-out: legality barely moved)'
    return ('pass' if g['hpwl'] >= ratio * g['halo']
            else f'REFUSE (hpwl {g["hpwl"]:.3f} < {ratio} x halo {g["halo"]:.3f})')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='wk/calibration')
    ap.add_argument('--boards', nargs='*', default=list(GOOD),
                    help=f'default: the qualify.json GOOD set {GOOD}')
    ap.add_argument('--ratio', type=float, default=0.25,
                    help='the ratio to REPORT against (default: the shipped 0.25)')
    ap.add_argument('--quick', action='store_true',
                    help='shorter deadlines; for a smoke run, not for a verdict')
    a = ap.parse_args()

    out = os.path.join(ROOT, a.out) if not os.path.isabs(a.out) else a.out
    os.makedirs(out, exist_ok=True)
    rows = []
    for name in a.boards:
        print(f'=== {name} ===', flush=True)
        r = run_board(name, out, quick=a.quick)
        rows.append(r)
        print(json.dumps(r, indent=1), flush=True)
        json.dump(rows, open(os.path.join(out, 'rows.json'), 'w'), indent=1)

    print('\n' + '=' * 78)
    print(f'{"board":10s} {"population":16s} {"halo_gain":>10s} {"hpwl_gain":>10s} '
          f'{"ratio":>8s}  {"blocking":>9s}  verdict @ {a.ratio}')
    for r in rows:
        if r.get('error'):
            print(f'{r["board"]:10s} ERROR {r["error"]}')
            continue
        for pop, gk, bk in (('undamaged->self', 'g_null', 'pristine_blocking'),
                            ('damaged->repair', 'g_repair', 'repaired_blocking'),
                            ('damaged->legality', 'g_legality',
                             'legality_only_blocking')):
            g = r.get(gk)
            if not g:
                print(f'{r["board"]:10s} {pop:16s} (not produced)')
                continue
            hg, pg = g.get('halo'), g.get('hpwl')
            rat = (pg / hg) if (hg and pg is not None and hg > 0) else None
            print(f'{r["board"]:10s} {pop:16s} '
                  f'{("%.4f" % hg) if hg is not None else "-":>10s} '
                  f'{("%.4f" % pg) if pg is not None else "-":>10s} '
                  f'{("%.4f" % rat) if rat is not None else "-":>8s}  '
                  f'{str(r.get(bk)):>9s}  {verdict_line(g, a.ratio)}')
    print('=' * 78)
    print(f'rows -> {os.path.join(out, "rows.json")}')
    print('\nREAD IT LIKE THIS: the gate is USEFUL only if damaged->repair passes'
          '\nand damaged->legality REFUSES, on every board. If both pass, the'
          '\nratio is too low. If both refuse, it is too high -- or hpwl does not'
          '\nseparate these populations at all, which is outcome 2 and means the'
          '\ngate should REPORT rather than refuse.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
