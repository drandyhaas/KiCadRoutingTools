#!/usr/bin/env python3
"""Measure how long KiCad's exact zone fill takes per board, next to the
board's deterministic fill signature (#831).

`kicad_exact_fill.refill_islands_ex` runs pcbnew's ZONE_FILLER in a
subprocess under `EXACT_FILL_TIMEOUT` (300 s). On expiry the callers
(`plane_fragility`, the GUI's `kicad_parser._live_fill`) fall back to the
drawn zone OUTLINES, so which geometry the router builds against depends on
whether pcbnew finished in 300 s on THIS machine. #831 asks whether a
predicate over `kicad_oracle._fill_cost_key` -- a signature that is a
function of the board alone -- separates "will finish" from "will not" with
margin. That question is answered by MEASUREMENT, which this tool produces:
one JSONL row per board with the signature, extra board features, the fill's
wall and child-CPU seconds and its `RefillStatus` reason.

Usage::

    python3 tests/stress/fill_timing_census.py --out fills.jsonl \\
        --timeout 1800 --workers 2 \\
        ~/Documents/kicad_stress_test/runs_set*/*/*planes*.kicad_pcb
    python3 tests/stress/fill_timing_census.py --out fills.jsonl \\
        --boards-from boards.txt
    python3 tests/stress/fill_timing_census.py --report fills.jsonl

Rows are APPENDED, and a board already present in `--out` is skipped, so a
run can be resumed. Boards run largest-file-first so the long tail starts
early and the pool stays busy. The corpus is read in place: the refill
stages each board (with its sibling `.kicad_pro`) into its own temp dir and
the parser is read-only, so nothing beside the output file is written.

`--report` prints the sorted table and the separation analysis (see
`analyse`): for every candidate predicate it reports the largest threshold
that keeps every timed-out board above it, the boards under 300 s that the
same threshold would wrongly refuse, and the margin between the two
populations. It does not pick one -- that is the reader's decision, and
#831's point is that it must be made from this table rather than guessed.
"""
import argparse
import glob
import json
import multiprocessing as mp
import os
import platform
import sys
import time

#: `resource` is POSIX-only. Imported at module scope it made this file
#: unimportable on Windows, which took `tests/test_831_fill_preflight_census.py`
#: -- a collected test that only calls the pure `analyse()` -- down with it at
#: IMPORT time, so the suite carried a standing failure on every Windows
#: machine (#882).
#:
#: Guarded rather than moved into `_child_cpu`, so the absence is a value this
#: module can report rather than an exception at the call site. Same shape as
#: `py_router/memory_debug.py`, which already does this for the same module.
try:
    import resource
    _HAS_RESOURCE = True
except ImportError:                                        # pragma: no cover
    resource = None
    _HAS_RESOURCE = False

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))

#: The production budget the analysis separates against.
PRODUCTION_TIMEOUT_S = 300


def _shoelace(poly):
    a = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def board_features(board_file):
    """The signature plus everything else cheap to read that might predict
    fill cost. `signature` is EXACTLY `kicad_oracle._fill_cost_key`, in both
    of its shapes (the 5-tuple, or ('path', realpath) on a parse failure)."""
    from kicad_oracle import _fill_cost_key
    from kicad_parser import parse_kicad_pcb
    t0 = time.monotonic()
    sig = _fill_cost_key(board_file)
    feats = {'signature': list(sig) if sig[0] == 'path'
             else [sig[0], sig[1], sig[2], sig[3], list(sig[4])],
             'signature_kind': sig[0]}
    if sig[0] == 'path':
        feats['parse_s'] = time.monotonic() - t0
        return feats
    pcb = parse_kicad_pcb(board_file)
    bb = pcb.board_info.board_bounds or (0, 0, 0, 0)
    zones = list(getattr(pcb, 'zones', ()) or ())
    zone_area = sum(_shoelace(z.polygon) for z in zones if len(z.polygon) >= 3)
    feats.update({
        'n_zones': len(zones),
        'n_pads': sum(len(f.pads) for f in pcb.footprints.values()),
        'n_footprints': len(pcb.footprints),
        'bbox': [round(v, 2) for v in bb],
        'bbox_area_mm2': round((bb[2] - bb[0]) * (bb[3] - bb[1]), 2),
        'zone_outline_area_mm2': round(zone_area, 2),
        'n_zone_layers': len({z.layer for z in zones}),
        'n_segments': len(pcb.segments),
        'n_vias': len(pcb.vias),
        'n_nets': len(pcb.nets),
        'n_copper_layers': len(pcb.board_info.copper_layers),
        'file_bytes': os.path.getsize(board_file),
        'parse_s': round(time.monotonic() - t0, 3),
    })
    return feats


def _child_cpu():
    """(cpu seconds, peak RSS) charged to reaped children, or (None, None).

    None rather than 0.0 where `resource` is absent. A zero here would be
    written into the recorded rows as `child_cpu_s: 0.0`, indistinguishable
    from a fill that genuinely cost nothing -- and those rows are compared
    against ones recorded on a POSIX machine. An absent measurement has to
    look absent.
    """
    if not _HAS_RESOURCE:                                  # pragma: no cover
        return None, None
    ru = resource.getrusage(resource.RUSAGE_CHILDREN)
    return ru.ru_utime + ru.ru_stime, ru.ru_maxrss


def measure_one(args):
    """One board: features, then a timed refill. Runs in a worker process,
    which serialises its refills, so the RUSAGE_CHILDREN delta is exactly the
    pcbnew subprocess of THIS fill (plus a one-off interpreter probe on the
    worker's first call, which `find_kicad_python` memoises)."""
    board_file, timeout = args
    from kicad_exact_fill import refill_islands_ex, find_kicad_python
    row = {'board': board_file}
    try:
        row.update(board_features(board_file))
    except Exception as e:                                 # noqa: BLE001
        row['feature_error'] = f'{type(e).__name__}: {e}'
    find_kicad_python()          # pay the probe OUTSIDE the timed window
    cpu0, _ = _child_cpu()
    t0 = time.monotonic()
    islands, st = refill_islands_ex(board_file, timeout=timeout)
    wall = time.monotonic() - t0
    cpu1, maxrss = _child_cpu()
    row.update({
        'status': st.reason,
        'detail': st.detail,
        'wall_s': round(wall, 2),
        'fill_elapsed_s': (round(st.elapsed_s, 2)
                           if st.elapsed_s is not None else None),
        # None all the way through where the platform cannot measure it, and
        # SAID so in the row rather than left for a reader to infer from a
        # null. `analyse()` reads neither field -- it separates on
        # `fill_elapsed_s` / `timeout_s` -- so a census recorded without them
        # still answers the question this module exists for.
        'child_cpu_s': (round(cpu1 - cpu0, 2)
                        if cpu0 is not None and cpu1 is not None else None),
        'child_maxrss_mb': (round(maxrss / (1024 * 1024), 1)
                            if maxrss is not None else None),
        'cpu_source': ('resource.RUSAGE_CHILDREN' if _HAS_RESOURCE else
                       f'not measured: no `resource` module on '
                       f'{platform.system()}'),
        'n_islands': (sum(len(v) for v in islands.values())
                      if islands else 0),
        'timeout_s': timeout,
    })
    return row


def _machine():
    cpu = ''
    try:
        import subprocess
        cpu = subprocess.run(['sysctl', '-n', 'machdep.cpu.brand_string'],
                             capture_output=True, text=True,
                             timeout=10).stdout.strip()
    except Exception:                                      # noqa: BLE001
        pass
    return {'platform': platform.platform(), 'cpu': cpu or platform.processor(),
            'ncpu': os.cpu_count()}


def load_rows(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def run_census(boards, out, timeout, workers):
    done = {r['board'] for r in load_rows(out) if 'status' in r}
    todo = [b for b in dict.fromkeys(boards) if b not in done]
    todo.sort(key=lambda b: -os.path.getsize(b))
    print(f"fill census: {len(todo)} board(s) to measure "
          f"({len(done)} already in {out}), timeout {timeout}s, "
          f"{workers} worker(s)")
    if not todo:
        return
    hdr = {'meta': True, 'machine': _machine(),
           'timeout_s': timeout, 'workers': workers,
           'started': time.strftime('%Y-%m-%d %H:%M:%S'),
           'n_boards': len(todo)}
    with open(out, 'a') as f:
        f.write(json.dumps(hdr) + '\n')
    t_start = time.monotonic()
    n = 0
    ctx = mp.get_context('spawn')
    with ctx.Pool(workers) as pool:
        for row in pool.imap_unordered(measure_one,
                                       [(b, timeout) for b in todo]):
            n += 1
            with open(out, 'a') as f:
                f.write(json.dumps(row) + '\n')
            print(f"[{n}/{len(todo)} {time.monotonic() - t_start:6.0f}s] "
                  f"{row['status']:14s} fill={row['fill_elapsed_s']} "
                  f"cpu={row['child_cpu_s']} sig="
                  f"{row.get('signature', row.get('feature_error'))} "
                  f"{os.path.relpath(row['board'])}", flush=True)


# ---------------------------------------------------------------- analysis
#: Candidate predicates over the row features. Every one is a MONOTONE
#: function of things the board alone determines; the first four are pure
#: functions of `_fill_cost_key` (its 5-tuple shape).
PREDICATES = {
    'n_zones': lambda r: r['n_zones'],
    'n_pads': lambda r: r['n_pads'],
    'bbox_area_mm2': lambda r: r['bbox_area_mm2'],
    'zones_x_bbox_area': lambda r: r['n_zones'] * r['bbox_area_mm2'],
    'zones_x_pads': lambda r: r['n_zones'] * r['n_pads'],
    'zones_x_bbox_area_x_pads':
        lambda r: r['n_zones'] * r['bbox_area_mm2'] * r['n_pads'],
    'zone_outline_area_mm2': lambda r: r['zone_outline_area_mm2'],
    'zone_area_x_items':
        lambda r: r['zone_outline_area_mm2']
        * (r['n_pads'] + r['n_segments'] + r['n_vias']),
    'zone_area_x_copper_items':
        lambda r: r['zone_outline_area_mm2'] * (r['n_segments'] + r['n_vias']),
    'n_segments+n_vias': lambda r: r['n_segments'] + r['n_vias'],
    'file_bytes': lambda r: r['file_bytes'],
}


def analyse(rows, budget=PRODUCTION_TIMEOUT_S, seconds_key='fill_elapsed_s'):
    """For each predicate: does a threshold separate over-budget boards from
    under-budget ones? Returns {name: {threshold, slow_min, fast_max,
    false_refusals, margin_ratio, ...}} where `threshold` is the LARGEST
    value that still refuses every slow board (the slow population's
    minimum), `false_refusals` counts fast boards at or above it, and
    `margin_ratio` = fast_max / slow_min (< 1 means clean separation; the
    smaller, the wider the gap). A `timeout` row counts as slow at its
    timeout, so a raised-timeout census still separates against `budget`."""
    rows = [r for r in rows if r.get('signature_kind') == 'fill'
            and r.get('status') in ('ok', 'timeout')]
    def secs(r):
        return r['timeout_s'] if r['status'] == 'timeout' else r[seconds_key]
    slow = [r for r in rows if secs(r) >= budget]
    fast = [r for r in rows if secs(r) < budget]
    out = {'n': len(rows), 'n_slow': len(slow), 'n_fast': len(fast),
           'budget_s': budget, 'predicates': {}}
    for name, fn in PREDICATES.items():
        fv = sorted(fn(r) for r in fast)
        sv = sorted(fn(r) for r in slow)
        rec = {'fast_max': fv[-1] if fv else None,
               'slow_min': sv[0] if sv else None}
        if sv:
            thr = sv[0]
            rec['threshold'] = thr
            rec['false_refusals'] = sum(1 for v in fv if v >= thr)
            rec['margin_ratio'] = (fv[-1] / thr) if (thr and fv) else None
            # the unsafe band: fast boards within 2x below the threshold
            rec['fast_within_2x'] = sum(1 for v in fv if thr / 2 <= v < thr)
            rec['separates'] = rec['false_refusals'] == 0
        # rank correlation with seconds, as a sanity check on "predicts"
        rec['spearman'] = _spearman([fn(r) for r in rows], [secs(r) for r in rows])
        out['predicates'][name] = rec
    return out


def _spearman(x, y):
    n = len(x)
    if n < 3:
        return None
    def ranks(v):
        order = sorted(range(n), key=lambda i: v[i])
        rk = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1
            for k in range(i, j + 1):
                rk[order[k]] = avg
            i = j + 1
        return rk
    rx, ry = ranks(x), ranks(y)
    mx, my = sum(rx) / n, sum(ry) / n
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    den = (sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry)) ** 0.5
    return round(num / den, 3) if den else None


def report(path, budget=PRODUCTION_TIMEOUT_S, top=25):
    rows = load_rows(path)
    meta = [r for r in rows if r.get('meta')]
    rows = [r for r in rows if not r.get('meta')]
    if meta:
        print(f"machine: {meta[0]['machine']}")
    by_status = {}
    for r in rows:
        by_status[r.get('status')] = by_status.get(r.get('status'), 0) + 1
    print(f"rows: {len(rows)}  by status: {by_status}")
    timed = [r for r in rows if r.get('status') in ('ok', 'timeout')
             and r.get('signature_kind') == 'fill']
    timed.sort(key=lambda r: -(r['timeout_s'] if r['status'] == 'timeout'
                               else r['fill_elapsed_s']))
    print(f"\nslowest {top} (fill_s cpu_s status  zones pads fps bbox_mm2 "
          f"zone_mm2 segs vias  board):")
    for r in timed[:top]:
        s = r['timeout_s'] if r['status'] == 'timeout' else r['fill_elapsed_s']
        # `{None:7.1f}` raises, and a row recorded on a platform without
        # `resource` carries None here (#882). Printed as `-` so the column
        # stays readable and an unmeasured cost cannot be read as a small one.
        _cpu = r.get('child_cpu_s')
        _cpu = f"{_cpu:7.1f}" if isinstance(_cpu, (int, float)) else f"{'-':>7}"
        print(f"  {s:7.1f} {_cpu} {r['status']:8s} "
              f"{r['n_zones']:4d} {r['n_pads']:5d} {r['n_footprints']:4d} "
              f"{r['bbox_area_mm2']:9.0f} {r['zone_outline_area_mm2']:9.0f} "
              f"{r['n_segments']:6d} {r['n_vias']:5d}  "
              f"{os.path.relpath(r['board'])}")
    a = analyse(rows, budget=budget)
    print(f"\nseparation against {budget}s: n={a['n']} slow={a['n_slow']} "
          f"fast={a['n_fast']}")
    for name, rec in a['predicates'].items():
        if rec.get('slow_min') is None:
            print(f"  {name:28s} spearman={rec['spearman']} "
                  f"fast_max={rec['fast_max']}  (no slow boards)")
            continue
        print(f"  {name:28s} spearman={rec['spearman']} "
              f"thr={rec['threshold']:.6g} fast_max={rec['fast_max']:.6g} "
              f"margin={rec['margin_ratio']:.3g} "
              f"false_refusals={rec['false_refusals']} "
              f"fast_within_2x={rec['fast_within_2x']} "
              f"{'SEPARATES' if rec['separates'] else 'overlaps'}")
    return a


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('boards', nargs='*', help='board files or globs')
    ap.add_argument('--boards-from', help='file with one board path per line')
    ap.add_argument('--out', help='JSONL rows (appended)')
    ap.add_argument('--timeout', type=int, default=1800,
                    help='refill timeout in seconds (default 1800; the '
                         f'production budget is {PRODUCTION_TIMEOUT_S})')
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--report', metavar='JSONL',
                    help='print the table + separation analysis for a census')
    ap.add_argument('--budget', type=int, default=PRODUCTION_TIMEOUT_S,
                    help='seconds the analysis separates against')
    args = ap.parse_args(argv)
    if args.report:
        report(args.report, budget=args.budget)
        return 0
    boards = []
    for b in args.boards:
        hits = glob.glob(os.path.expanduser(b))
        boards.extend(hits if hits else [b])
    if args.boards_from:
        with open(args.boards_from) as f:
            boards.extend(l.strip() for l in f if l.strip())
    boards = [os.path.abspath(os.path.expanduser(b)) for b in boards]
    missing = [b for b in boards if not os.path.isfile(b)]
    if missing:
        print(f"ERROR: {len(missing)} board(s) not found, e.g. {missing[0]}")
        return 2
    if not boards or not args.out:
        ap.error('need boards and --out (or --report)')
    run_census(boards, args.out, args.timeout, args.workers)
    return 0


if __name__ == '__main__':
    sys.exit(main())
