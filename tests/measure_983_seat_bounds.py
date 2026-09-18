#!/usr/bin/env python3
"""#983 and its two siblings: does an edge seat write the pose its own grade accepts?

    # one arm per engine tree (each a clean checkout):
    python3 -B -X utf8 tests/measure_983_seat_bounds.py --repo <base-tree> --out base.json
    python3 -B -X utf8 tests/measure_983_seat_bounds.py --repo <head-tree> --out head.json
    python3 -B -X utf8 tests/measure_983_seat_bounds.py --diff base.json head.json
    # --quick runs a thinned lattice (for iterating, never for a MEASURED row)

NOT named `test_*`, so `run_all.py` never collects it: it needs a second
checkout and a few minutes per arm.

Three mechanisms, each a lattice of seats graded on the written board:

  A  (#983) along-edge window. A rung clamped to a declared window END is
     written `round(x, 3)`, up to 0.5 um outside the window, and
     `_grade_along_edge` refuses anything past EPS (1 nm).
       L-A1  the issue's own lattice: 2880 positions of a locked blocker R9,
             `_seat_edge(..., target=(2.18, -0.24))`. R9 is test_975's
             band_later R9 (courtyard +-1 x +-0.6, pads +-0.5); the issue did
             not print it, and this geometry reproduces its 70/2880 exactly.
       L-A2  the same board through stage 1 of `seed_from_intent`, blocker y
             thinned to 0.1 mm.
       L-A1r the issue's lattice, blocker x 0.5-1.75, through
             `repair_placement` -- the PRODUCTION caller, which hands the seat
             a live grader (L-A1 calls `_seat_edge` bare, as the issue did).
       L-A3  a `center_on_edge` window: tolerance 0.5 over a blocker
             lattice, and tolerance 0 (a window the 1 um grid cannot always
             meet -- disclosed, not fixed).
  B  overhang band. The seat accepts a band reading within +-0.02 mm
     (`_edge_correct`, `_body_band_correct`, `edge_seat_ok`); the grade
     within EPS. A first seat can grade outside its band by up to the
     seat's 0.02 mm plus half a micron of rounding (0.019 mm measured).
       L-B1  a drawn body 0.400-0.706 mm past its courtyard, four bands.
       L-B2  no declared max, `min` >= 0.6: the target IS the minimum, and a
             body edge off the 1 um grid rounds under it.
       L-B3  a courtyard-only receptacle on a {0, 0} band, courtyard edge off
             the grid (a band_max reading of up to 0.5 um).
       L-B4  a gate margin under 0.02 mm: the walk calls its first guess
             converged at target + margin.
  C  stage 1 converts the declared window and the part's extents at its
     INPUT rotation, then applies a DECLARED one.
       L-C   splitflap_driver J5 on the north edge, seeded alone
             (`seed_refs`), declared at 0/90/180/270 with a centre claim and
             with a band.

INPUTS, the WRITER and the GRADER come from THIS file's repo: every board's
sha256 is recorded per row, the repo's commit as `here_sha` (a dirty tree is
refused), and `--diff` refuses two arms graded by different commits. Only the
ENGINE comes from `--repo`: a `--worker` subprocess runs there, with that
tree's `py_*` first on sys.path and PYTHONHASHSEED=0, and hands back poses and
notes. The worker also counts, per case, how often each correction the fix
adds CHANGED its input (`_window_nudge`, `_band_settle`,
`_stage1_geometry_rot`), binding each call's arguments BY NAME so a signature
change cannot silently count the wrong one; on a tree without them the count
is null. The base arm must be the fix's parent: #986's own floor preference
moves hundreds of these seats relative to upstream main.

THE RULE, written before any fix existed and tightened by the pre-push
review (never loosened):

  every row: head `unseated` <= base; head grade errors (edge-connector and
    `legality`) a sub-multiset of base's (a band error traded for a setback
    error fails); head pad-edge floor shortfall no worse on EITHER count
    (pads short, worst shortfall); no courtyard overlap on the written board
    where base had none. An overlap base already had may deepen -- the
    user's choice for #983, see seeder `_no_worse` -- and every such row is
    counted and its growth printed.
  a row whose written pose differs from base must have fired a correction
    (on SOME rung of that seat: the count is per seat, not per kept rung).
  a row that raised on either arm fails the run; a row seated only on head
    is printed, and must have fired too.
  SIGNAL: the A, B and C error counts fall; rows that stay dirty are named.
  Rows where nothing changed are counted as NULL rows, never dropped.

MEASURED: filled in from the runs, never predicted.
"""
import argparse
import contextlib
import hashlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MARK = 'M983: '
SPLIT = os.path.join(HERE, 'kicad_files', 'splitflap_driver.kicad_pcb')
CORRECTIONS = ('_window_nudge', '_band_settle', '_stage1_geometry_rot')


def _here_paths():
    for p in (HERE, os.path.join(HERE, 'py_router'), os.path.join(HERE, 'py_placer'),
              os.path.join(HERE, 'py_tools')):
        if p not in sys.path:
            sys.path.insert(0, p)


def sha(path):
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def git(root, *args):
    return subprocess.run(['git', '-C', root, *args], capture_output=True, text=True,
                          check=True).stdout.strip()


# ---------------------------------------------------------------- inputs

ISSUE_J1 = ('  (footprint "t" (layer "F.Cu") (at 14.15 9.0 270)\n'
            '    (property "Reference" "J1")\n'
            '    (pad "1" smd oval (at 2.75 -1.41) (size 0.69 1.03) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.72 -0.09) (size .5 .5) (layers "F.Cu")))\n')


def _r9(bx, by):
    return (f'  (footprint "r" (locked yes) (layer "F.Cu") (at {bx} {by} 0)\n'
            '    (property "Reference" "R9")\n'
            '    (fp_rect (start -1 -0.6) (end 1 0.6) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu")))\n')


def _board(size, *footprints):
    w, h = size
    return ('(kicad_pcb (version 20241229) (generator "m983")\n'
            f'  (gr_rect (start 0 0) (end {w} {h}) (layer "Edge.Cuts"))\n'
            + ''.join(footprints) + ')\n')


def _j1(graphics, pads, at='10 10 0'):
    return (f'  (footprint "t" (layer "F.Cu") (at {at})\n'
            '    (property "Reference" "J1")\n'
            + ''.join(f'    {g}\n' for g in graphics)
            + ''.join(f'    {p}\n' for p in pads) + '  )\n')


PADS3 = ('(pad "1" smd rect (at -1.905 -0.5) (size .5 .5) (layers "F.Cu"))',
         '(pad "2" smd rect (at -1.905 0.5) (size .5 .5) (layers "F.Cu"))',
         '(pad "3" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))')


def cases(quick=False):
    """[(case, board_text_or_path)] -- every input, built here."""
    out = []
    along = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': 1.5},
             'along_edge_band': {'from': 0.89, 'to': 0.92}}
    xs = [round(0.5 + 0.25 * i, 2) for i in range(24)]
    ys = [round(12.0 + 0.05 * j, 2) for j in range(120)]
    if quick:
        xs = xs[:4]
    for bx in xs:
        for by in ys:
            out.append(({'id': f'A1/{bx}/{by}', 'lattice': 'A1', 'ladder': 'seat',
                         'entry': along, 'target': [2.18, -0.24], 'size': [28.3, 18.0]},
                        _board((28.3, 18.0), ISSUE_J1, _r9(bx, by))))
    for bx in xs[::2] if not quick else xs[:2]:
        for by in ys[::2]:
            out.append(({'id': f'A2/{bx}/{by}', 'lattice': 'A2', 'ladder': 'stage1',
                         'entry': along, 'size': [28.3, 18.0]},
                        _board((28.3, 18.0), ISSUE_J1, _r9(bx, by))))
    for bx in xs[:6] if not quick else xs[:2]:
        for by in ys if not quick else ys[::4]:
            out.append(({'id': f'A1r/{bx}/{by}', 'lattice': 'A1r', 'ladder': 'repair',
                         'entry': along, 'size': [28.3, 18.0]},
                        _board((28.3, 18.0), ISSUE_J1, _r9(bx, by))))
    centre = dict(along)
    del centre['along_edge_band']
    for tol, bxs, byss in ((0.5, (0.5, 0.75, 1.0, 1.25), [round(6.0 + 0.05 * j, 2) for j in range(121)]),
                           (0.0, (0.5, 1.0), [round(6.0 + 0.25 * j, 2) for j in range(25)])):
        for bx in bxs[:2] if quick else bxs:
            for by in byss[::4] if quick else byss:
                e = dict(centre, center_on_edge={'tolerance_mm': tol})
                for ladder in ('seat', 'stage1'):
                    out.append(({'id': f'A3/{tol}/{ladder}/{bx}/{by}', 'lattice': 'A3',
                                 'ladder': ladder, 'entry': e, 'target': [2.18, -0.24],
                                 'size': [28.3, 18.0]},
                                _board((28.3, 18.0), ISSUE_J1, _r9(bx, by))))
    body = '(fp_rect (start -3 -1) (end 1 1) (layer "F.Fab"))'
    for band in ({'min': 0.3, 'max': 0.5}, {'min': 0.25, 'max': 0.35},
                 {'min': 0.0, 'max': 0.02}, {'min': 0.5}):
        for k in range(0, 61, 4 if quick else 1):
            cl = round(-2.60 + 0.0051 * k, 4)
            crt = f'(fp_rect (start {cl} -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
            e = {'ref': 'J1', 'edge': 'west', 'overhang_mm': dict(band)}
            for ladder in ('seat', 'stage1'):
                out.append(({'id': f'B1/{json.dumps(band, sort_keys=True)}/{cl}/{ladder}',
                             'lattice': 'B1', 'ladder': ladder, 'entry': e, 'target': [0.0, 10.0],
                             'size': [20, 20]},
                            _board((20, 20), _j1((body, crt), PADS3))))
    for lo in (0.6, 0.75, 1.0):
        for k in range(0, 11, 5 if quick else 1):
            bl = round(-4.999 - 0.0001 * k, 4)
            g = f'(fp_rect (start {bl} -1) (end 1 1) (layer "F.Fab"))'
            e = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': lo}}
            for ladder in ('seat', 'stage1'):
                out.append(({'id': f'B2/{lo}/{bl}/{ladder}', 'lattice': 'B2', 'ladder': ladder,
                             'entry': e, 'target': [0.0, 10.0], 'size': [20, 20]},
                            _board((20, 20), _j1((g,), PADS3))))
    rpads = ('(pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))',
             '(pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))')
    for k in range(0, 10, 3 if quick else 1):
        yb = round(-1.0 - 0.0001 * k, 4)
        crt = f'(fp_rect (start -1.5 {yb}) (end 1.5 1) (layer "F.CrtYd"))'
        e = {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 0.0},
             'class': 'edge_receptacle'}
        for ladder in ('seat', 'stage1'):
            out.append(({'id': f'B3/{yb}/{ladder}', 'lattice': 'B3', 'ladder': ladder,
                         'entry': e, 'target': [10.0, 0.0], 'size': [20, 20]},
                        _board((20, 20), _j1((crt,), rpads))))
    for hi in (0.005, 0.01, 0.015):
        crt = '(fp_rect (start -2.3 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        e = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.0, 'max': hi}}
        for ladder in ('seat', 'stage1'):
            out.append(({'id': f'B4/{hi}/{ladder}', 'lattice': 'B4', 'ladder': ladder,
                         'entry': e, 'target': [0.0, 10.0], 'size': [20, 20],
                         'clearance': 0.01, 'edge_clearance': 0.01},
                        _board((20, 20), _j1((crt,), PADS3))))
    for claim in ({'center_on_edge': {'tolerance_mm': 1.0}},
                  {'along_edge_band': {'from': 0.2, 'to': 0.3}}):
        for rot in (0, 90, 180, 270):
            e = dict({'ref': 'J5', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 0.8}},
                     **claim)
            out.append(({'id': f'C/{sorted(claim)[0]}/{rot}', 'lattice': 'C', 'ladder': 'stage1',
                         'entry': e, 'blocks': [{'name': 'j5', 'refs': ['J5'], 'rotation': rot}],
                         'seed_refs': ['J5']}, SPLIT))
    return out


def intent_of(case):
    from placement import floorplan
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
           'edge_connectors': [dict(case['entry'])]}
    if case.get('blocks'):
        doc['blocks'] = [dict(b) for b in case['blocks']]
    return doc


# ---------------------------------------------------------------- worker

def worker():
    """Runs in the ENGINE tree. Reads cases on stdin, prints one MARK line each."""
    repo = os.getcwd()
    for p in ('py_tools', 'py_placer', 'py_router'):
        sys.path.insert(0, os.path.join(repo, p))
    import random
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan, seeder
    import pose_score
    import inspect
    fires = {}

    def wrap(name, changed):
        f = getattr(seeder, name, None)
        if f is None:
            return
        sig = inspect.signature(f)

        def counted(*a, **k):
            r = f(*a, **k)
            if changed(sig.bind(*a, **k).arguments, r):
                fires[name] = fires.get(name, 0) + 1
            return r
        setattr(seeder, name, counted)
    wrap('_window_nudge', lambda b, r: tuple(r) != (b['x'], b['y']))
    wrap('_band_settle', lambda b, r: tuple(r[:2]) != (b['x'], b['y']))
    wrap('_stage1_geometry_rot', lambda b, r: r != b['part'].rot)
    present = {n: hasattr(seeder, n) for n in CORRECTIONS}
    for line in sys.stdin:
        case = json.loads(line)
        fires.clear()
        row = {'id': case['id']}
        clr, edge = case.get('clearance', .25), case.get('edge_clearance', .55)
        sink = io.StringIO()
        try:
            with contextlib.redirect_stdout(sink):
                pcb = parse_kicad_pcb(case['path'])
                ref = case['entry']['ref']
                if case['ladder'] == 'seat':
                    st = pose_score.make_state(pcb, case['path'], clearance=clr,
                                               board_edge_clearance=edge)
                    notes = []
                    ok = seeder._seat_edge(st, ref, dict(case['entry']), set(), notes,
                                           target=tuple(case['target']))
                    p = st.parts[ref]
                    row.update(ok=bool(ok), poses={ref: [p.x, p.y, p.rot]} if ok else {},
                               notes=notes)
                elif case['ladder'] == 'repair':
                    intent = floorplan.intent_from_dict(json.loads(case['intent']))
                    res = seeder.repair_placement(pcb, case['path'], intent, clearance=clr,
                                                  board_edge_clearance=edge)
                    moved = {q['reference']: [q['new_x'], q['new_y'], q['new_rotation']]
                             for q in res['moves']}
                    row.update(ok=ref in moved and ref not in (res.get('unseated') or []),
                               poses={ref: moved[ref]} if ref in moved else {},
                               notes=list(res.get('notes') or []))
                else:
                    intent = floorplan.intent_from_dict(json.loads(case['intent']))
                    kw = {}
                    if case.get('seed_refs'):
                        kw['seed_refs'] = set(case['seed_refs'])
                    res = seeder.seed_from_intent(pcb, case['path'], intent, random.Random(0),
                                                  clearance=clr, board_edge_clearance=edge, **kw)
                    placed = {q['reference']: [q['new_x'], q['new_y'], q['new_rotation']]
                              for q in res['placements']}
                    row.update(ok=ref in placed and ref not in (res.get('unseated') or []),
                               poses={r: v for r, v in placed.items()
                                      if r == ref or r in (case.get('seed_refs') or ())},
                               notes=list(res.get('notes') or []))
        except Exception as exc:                               # noqa: BLE001
            row.update(ok=False, poses={}, notes=[], error=f'{type(exc).__name__}: {exc}')
        row['fires'] = {n: (fires.get(n, 0) if present[n] else None) for n in CORRECTIONS}
        print(MARK + json.dumps(row, sort_keys=True), flush=True)


# ---------------------------------------------------------------- grading (HERE)

def classify(violations, ref):
    """Edge-connector errors on `ref`, by the `measured` key the rule writes,
    plus any board-level `legality` error (a budget the seat may have broken)."""
    kinds = []
    for v in violations:
        if v.rule == 'legality':
            kinds.append('legality')
            continue
        if v.ref != ref or v.rule != 'edge_connector':
            continue
        m = v.measured or {}
        if 'along_edge_fraction' in m or 'along_edge_offset_mm' in m:
            kinds.append('along_edge')
        elif 'overhang_mm' in m:
            kinds.append('band')
        elif 'edge_clearance_mm' in m:
            kinds.append('setback')
        elif 'edge' in m:
            kinds.append('nearest_edge')
        elif 'pad_copper_outside_mm' in m:
            kinds.append('pad_copper_outside')
        else:
            kinds.append('other')
    return sorted(kinds)


def grade_row(case, row, tmp):
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan, legality
    from placement.writer import write_placed_output
    ref = case['entry']['ref']
    clr, edge = case.get('clearance', .25), case.get('edge_clearance', .55)
    out = os.path.join(tmp, 'graded.kicad_pcb')
    moves = [{'reference': r, 'new_x': round(v[0], 3), 'new_y': round(v[1], 3),
              'new_rotation': v[2]} for r, v in row['poses'].items()]
    with contextlib.redirect_stdout(io.StringIO()):
        write_placed_output(case['path'], out, moves)
    pcb = parse_kicad_pcb(out)
    graded = floorplan.grade(floorplan.intent_from_dict(json.loads(case['intent'])), pcb, out,
                             clearance=clr, board_edge_clearance=edge)
    floor = legality.grade_pad_edge_clearance(pcb, edge, out)
    short = [f['shortfall_mm'] for f in floor['findings']
             if f['pad_ref'].split('.')[0] == ref]
    fp = pcb.footprints[ref]
    return {'errors': classify(graded.errors, ref),
            'written': [round(fp.x, 4), round(fp.y, 4), round((fp.rotation or 0.0) % 360.0, 6)],
            'floor_n': len(short), 'floor_max': round(max(short, default=0.0), 6),
            'overlap': round(float(graded.legality.get('overlap_area') or 0.0), 6)}


def collect(repo, out_path, quick):
    _here_paths()
    repo = os.path.abspath(repo)
    for tree in (repo, HERE):
        if git(tree, 'status', '--porcelain', '--untracked-files=no'):
            sys.exit(f'REFUSED: {tree} has uncommitted changes')
    tmp = tempfile.mkdtemp(prefix='m983_')
    try:
        todo, input_sha = [], {}
        for i, (case, board) in enumerate(cases(quick)):
            if board == SPLIT:
                path = os.path.join(tmp, 'splitflap.kicad_pcb')
                if not os.path.exists(path):
                    shutil.copyfile(SPLIT, path)
            else:
                path = os.path.join(tmp, f'c{i}.kicad_pcb')
                with open(path, 'w', encoding='utf-8') as stream:
                    stream.write(board)
            case = dict(case, path=path, intent=json.dumps(intent_of(case), sort_keys=True))
            input_sha[case['id']] = sha(path)
            todo.append(case)
        env = dict(os.environ, PYTHONHASHSEED='0', PYTHONIOENCODING='utf-8', KRT_NO_BANNER='1')
        t0 = time.perf_counter()
        proc = subprocess.run([sys.executable, '-B', '-X', 'utf8', os.path.abspath(__file__),
                               '--worker'], input=''.join(json.dumps(c) + '\n' for c in todo),
                              capture_output=True, text=True, cwd=repo, env=env)
        seconds = round(time.perf_counter() - t0, 1)
        rows = {}
        for line in proc.stdout.splitlines():
            if line.startswith(MARK):
                r = json.loads(line[len(MARK):])
                rows[r['id']] = r
        if len(rows) != len(todo):
            sys.exit(f'worker returned {len(rows)} of {len(todo)} rows (rc {proc.returncode})\n'
                     + proc.stderr[-3000:])
        for case in todo:
            r = rows[case['id']]
            if r['ok'] and r['poses']:
                r.update(grade_row(case, r, tmp))
            r.update(lattice=case['lattice'], input_sha256=input_sha[case['id']])
        doc = {'repo': repo, 'engine_sha': git(repo, 'rev-parse', 'HEAD'),
               'here_sha': git(HERE, 'rev-parse', 'HEAD'), 'quick': quick,
               'python': sys.version.split()[0], 'worker_seconds': seconds,
               'rows': [rows[c['id']] for c in todo]}
        with open(out_path, 'w', encoding='utf-8') as stream:
            json.dump(doc, stream, indent=1, sort_keys=True)
        summarize(doc)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _dirty(row, kinds=None):
    errs = row.get('errors') or []
    return [e for e in errs if kinds is None or e in kinds]


def summarize(doc):
    print(f'engine {doc["engine_sha"][:10]}  inputs {doc["here_sha"][:10]}  '
          f'worker {doc["worker_seconds"]}s  quick={doc["quick"]}')
    lattices = sorted({r['lattice'] for r in doc['rows']})
    print(f'{"lattice":8s} {"rows":>5s} {"seated":>7s} {"along":>6s} {"band":>5s} '
          f'{"setback":>8s} {"other":>6s} {"floor":>6s}  fired')
    for lat in lattices:
        rs = [r for r in doc['rows'] if r['lattice'] == lat]
        seated = [r for r in rs if r['ok']]
        n = lambda kinds: sum(1 for r in seated if _dirty(r, kinds))
        fired = {c: sum((r['fires'] or {}).get(c) or 0 for r in rs) for c in CORRECTIONS
                 if any((r['fires'] or {}).get(c) is not None for r in rs)}
        print(f'{lat:8s} {len(rs):5d} {len(seated):7d} {n({"along_edge"}):6d} {n({"band"}):5d} '
              f'{n({"setback"}):8d} {n({"nearest_edge", "pad_copper_outside", "other"}):6d} '
              f'{sum(1 for r in seated if r.get("floor_n")):6d}  {fired or "-"}')
    errors = [r for r in doc['rows'] if r.get('error')]
    if errors:
        print(f'{len(errors)} rows raised, first: {errors[0]["id"]}: {errors[0]["error"]}')


# ---------------------------------------------------------------- diff

def diff(a_path, b_path):
    docs = []
    for p in (a_path, b_path):
        with open(p, encoding='utf-8') as stream:
            docs.append(json.load(stream))
    base, head = docs
    if base.get('quick') != head.get('quick'):
        print('REFUSED: one arm is --quick and the other is not')
        return 2
    if base['here_sha'] != head['here_sha']:
        print(f'REFUSED: the arms were graded by different commits '
              f'({base["here_sha"][:10]} vs {head["here_sha"][:10]}); re-run both '
              f'from one tree')
        return 2
    for d in docs:
        summarize(d)
        print()
    rb = {r['id']: r for r in base['rows']}
    rh = {r['id']: r for r in head['rows']}
    violations, changed, null, fixed, still = [], 0, 0, [], []
    grew, newly = [], []
    fired = lambda r: any((r['fires'] or {}).get(c) for c in CORRECTIONS)
    for rid in sorted(set(rb) | set(rh)):
        b, h = rb.get(rid), rh.get(rid)
        if b is None or h is None:
            violations.append((rid, 'missing on one side'))
            continue
        if b['input_sha256'] != h['input_sha256']:
            violations.append((rid, 'inputs differ'))
        for side, r in (('base', b), ('head', h)):
            if r.get('error'):
                violations.append((rid, f'raised on {side}: {r["error"]}'))
        if b['ok'] and not h['ok']:
            violations.append((rid, 'unseated on head'))
        if h['ok'] and not b['ok']:
            newly.append((rid, h.get('errors')))
            if not fired(h):
                violations.append((rid, 'seated on head only, with no correction fired'))
        eb, eh = b.get('errors') or [], h.get('errors') or []
        if h['ok'] and b['ok']:
            # A multiset SUBSET, not a count: a correction that trades a band
            # error for a setback error keeps the count and is still a trade.
            if any(eh.count(k) > eb.count(k) for k in set(eh)):
                violations.append((rid, f'grade errors {eb} -> {eh}'))
            # Each floor count on its own: fewer pads short with a deeper
            # worst is still deeper.
            if (h.get('floor_n', 0) > b.get('floor_n', 0)
                    or h.get('floor_max', 0) > b.get('floor_max', 0) + 1e-9):
                violations.append((rid, f'floor {b.get("floor_n")}/{b.get("floor_max")} -> '
                                        f'{h.get("floor_n")}/{h.get("floor_max")}'))
            ob, oh = b.get('overlap', 0.0), h.get('overlap', 0.0)
            if ob <= 1e-6 < oh:
                violations.append((rid, f'new courtyard overlap 0 -> {oh}'))
            elif oh > ob + 1e-6:
                grew.append((rid, ob, oh))
            if b.get('written') != h.get('written'):
                changed += 1
                if not fired(h):
                    violations.append((rid, f'pose changed with no correction fired: '
                                            f'{b.get("written")} -> {h.get("written")}'))
            else:
                null += 1
            if eb and not eh:
                fixed.append(rid)
            elif eh:
                still.append((rid, eh))
    print(f'rows {len(rh)}  pose changed {changed}  NULL (unchanged) {null}  '
          f'fixed {len(fixed)}  still dirty on head {len(still)}')
    if grew:
        worst = max(oh - ob for _r, ob, oh in grew)
        print(f'existing courtyard overlap deepened on {len(grew)} rows, by at most '
              f'{worst:.4f} mm2 (base overlap {min(g[1] for g in grew):.4f}-'
              f'{max(g[1] for g in grew):.4f} mm2)')
    for rid, errs in newly[:10]:
        print(f'  seated on head only: {rid}: {errs}')
    for rid, eh in still[:20]:
        print(f'  still dirty: {rid}: {eh}')
    if violations:
        print(f'VIOLATIONS: {len(violations)}')
        for v in violations[:40]:
            print(f'  {v[0]}: {v[1]}')
        print('VERDICT: FAIL')
        return 1
    print('VERDICT: PASS (no unseat, no grade or floor regression, every change fired)')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--repo')
    ap.add_argument('--out')
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--diff', nargs=2, metavar=('BASE', 'HEAD'))
    ap.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.worker:
        return worker()
    if args.diff:
        _here_paths()
        return diff(*args.diff)
    if not (args.repo and args.out):
        ap.error('--repo and --out, or --diff BASE HEAD')
    return collect(args.repo, args.out, args.quick)


if __name__ == '__main__':
    sys.exit(main())
