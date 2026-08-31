#!/usr/bin/env python3
"""#797: what the SEEDER does with a declared exclusive zone, on real boards.

Run it once before the change and once after, into two files, and diff them.
A run that lives only in a terminal is not evidence, so the numbers are written
to JSON with the engine SHA that produced them.

    python3 -X utf8 tests/measure_797_seed_exclusive.py --out before.json
    ... apply the change ...
    python3 -X utf8 tests/measure_797_seed_exclusive.py --out after.json
    python3 -X utf8 tests/measure_797_seed_exclusive.py --diff before.json after.json

NOT named `test_*.py`: it drives `place_seed` over four corpus boards and takes
minutes, so `tests/run_all.py` must not collect it.

THE FIXTURE, and its honest description. `floorplan.emit_intent` reads a
starter intent OFF the board -- one block per derived group, each zone the
members' bounding box padded 1mm, clamped to the envelope and KEPT ONLY WHERE
DISJOINT from every other kept zone (floorplan.py:1890-1901). This script then
sets `exclusive: true` on every block that has a zone, BY RULE and never on a
hand-picked list: a fixture that named its own zones would be fitted to the
arrangement it is measuring, which is the defect `test_placement_ab.py`'s
`_intent_for` docstring records for keep-outs.

So these are SYNTHETIC exclusive declarations derived from the board, not a
human's RF keep-clear. They are a reproducible stress fixture and the PR must
say so. What makes them a fair one is the disjointness: the strangers inside a
zone are parts of some other group, not artifacts of two zones overlapping.

BOTH NUMBERS OR NEITHER. The conjunct can only ever REMOVE poses, so a drop in
`zone_exclusive` errors bought by stranding parts is not an improvement, it is
a trade. Every row therefore carries `unseated` beside `zone_exclusive`, and a
board whose seeder never intruded is reported as a NULL rather than dropped.
"""
import argparse
import json
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
        sys.path.insert(0, os.path.join(_p, 'py_router'))
        sys.path.insert(0, os.path.join(_p, 'py_tools'))
        sys.path.insert(0, os.path.join(_p, 'py_placer'))

#: The boards `emit_intent` produces at least one ZONED block for. Checked, not
#: assumed: splitflap_driver and tigard produce zero blocks and therefore
#: cannot carry this measurement at all -- stated here because they are the two
#: boards the placement tests most often reach for.
BOARDS = (
    'kit-dev-coldfire-xilinx_5213',
    'ulx3s',
    'orangecrab_ext_pll',
    'rp2350_fpga_eensy_prePlane',
)

SOURCES = ('kicad', 'sheet')
CLEARANCE = '0.2'
EDGE = '0.55'


def engine_sha():
    try:
        r = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=REPO,
                           capture_output=True, text=True, timeout=30)
        sha = r.stdout.strip()
        d = subprocess.run(['git', 'status', '--porcelain'], cwd=REPO,
                           capture_output=True, text=True, timeout=30)
        return sha + ('+dirty' if d.stdout.strip() else '')
    except Exception as exc:                                # noqa: BLE001
        return f'unknown ({exc})'


def write_intent(board_path, out_path):
    """The emitted intent with `exclusive` set on every ZONED block."""
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    doc = floorplan.emit_intent(parse_kicad_pcb(board_path), board_path,
                                group_sources=SOURCES)
    zoned = 0
    for b in doc['blocks']:
        if b.get('zone'):
            b['exclusive'] = True
            zoned += 1
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=2)
    return zoned, len(doc['blocks'])


def parse_summary(stdout):
    for line in stdout.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            try:
                return json.loads(line.split(':', 1)[1])
            except ValueError:
                return None
    return None


def grade_counts(out_board, intent_path):
    """`rule_zone_exclusive` errors on the WRITTEN board, from check_floorplan.

    Counted from the ERRORS, never from `violations_by_rule`, which counts
    warnings too: `zone_exclusive` defaults to ERROR today, so the two agree --
    but the number that goes in a PR must be the one that cannot silently
    change meaning when someone sets a severity.
    """
    jpath = out_board + '.grade.json'
    argv = [sys.executable, '-X', 'utf8',
            os.path.join('py_tools', 'check_floorplan.py'), out_board,
            '--intent', intent_path, '--group-by', ','.join(SOURCES),
            '--clearance', CLEARANCE, '--board-edge-clearance', EDGE,
            '--json', jpath, '--exit-zero', '-q']
    r = subprocess.run(argv, cwd=REPO, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=1800,
                       env=dict(os.environ, PYTHONHASHSEED='0',
                                PYTHONIOENCODING='utf-8'))
    if not os.path.exists(jpath):
        return {'error': f'check_floorplan wrote no json (rc={r.returncode})',
                'stderr': r.stderr[-400:]}
    with open(jpath, encoding='utf-8') as f:
        doc = json.load(f)
    by_rule = {}
    for v in doc.get('violations') or []:
        if v.get('severity') == 'error':
            by_rule[v['rule']] = by_rule.get(v['rule'], 0) + 1
    return {'zone_exclusive': by_rule.get('zone_exclusive', 0),
            'errors_by_rule': by_rule,
            'errors_total': sum(by_rule.values())}


def one_arm(board, wd, polish):
    from kicad_parser import parse_kicad_pcb
    bpath = os.path.join(REPO, 'kicad_files', f'{board}.kicad_pcb')
    tag = 'polish' if polish else 'seed'
    ipath = os.path.join(wd, f'{board}.intent.json')
    opath = os.path.join(wd, f'{board}.{tag}.kicad_pcb')
    zoned, blocks = write_intent(bpath, ipath)

    argv = [sys.executable, '-X', 'utf8',
            os.path.join('py_placer', 'place_seed.py'), bpath, opath,
            '--intent', ipath, '--force', '--seed', '0',
            '--group-by', ','.join(SOURCES),
            '--clearance', CLEARANCE, '--board-edge-clearance', EDGE]
    if not polish:
        argv.append('--no-polish')
    t0 = time.time()
    r = subprocess.run(argv, cwd=REPO, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=3600,
                       env=dict(os.environ, PYTHONHASHSEED='0',
                                PYTHONIOENCODING='utf-8'))
    secs = round(time.time() - t0, 1)
    s = parse_summary(r.stdout) or {}
    row = {'board': board, 'arm': tag, 'rc': r.returncode, 'seconds': secs,
           'zoned_blocks': zoned, 'blocks': blocks,
           'unseated': s.get('unseated'),
           'unseated_refs': sorted(s.get('unseated_refs') or ()),
           'verdicts': {}, 'summary_grade_errors': s.get('grade_errors')}
    for _ref, v in (s.get('no_pose_verdict') or {}).items():
        row['verdicts'][v] = row['verdicts'].get(v, 0) + 1
    if os.path.exists(opath):
        row.update(grade_counts(opath, ipath))
    else:
        row['error'] = f'place_seed wrote no board (rc={r.returncode})'
        row['stderr'] = r.stderr[-400:]
    return row


def run(out_path, boards, arms):
    import tempfile
    rows = []
    with tempfile.TemporaryDirectory() as wd:
        for b in boards:
            bp = os.path.join(REPO, 'kicad_files', f'{b}.kicad_pcb')
            if not os.path.exists(bp):
                print(f"  SKIP {b}: no such board")
                continue
            for polish in arms:
                row = one_arm(b, wd, polish)
                rows.append(row)
                print(f"  {row['board']:<32} {row['arm']:<7} "
                      f"zone_exclusive={row.get('zone_exclusive')!s:>4}  "
                      f"unseated={row.get('unseated')!s:>4}  "
                      f"rc={row['rc']}  {row['seconds']}s")
    doc = {'engine_sha': engine_sha(), 'sources': list(SOURCES),
           'clearance': float(CLEARANCE), 'board_edge_clearance': float(EDGE),
           'rows': rows}
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=2, sort_keys=True)
    print(f"\nwrote {out_path} ({len(rows)} row(s), engine {doc['engine_sha']})")
    return 0


def diff(a_path, b_path):
    with open(a_path, encoding='utf-8') as f:
        a = json.load(f)
    with open(b_path, encoding='utf-8') as f:
        b = json.load(f)
    ka = {(r['board'], r['arm']): r for r in a['rows']}
    kb = {(r['board'], r['arm']): r for r in b['rows']}
    print(f"BEFORE engine {a['engine_sha']}\nAFTER  engine {b['engine_sha']}\n")
    print(f"{'board':<32} {'arm':<7} {'zone_exclusive':>16} "
          f"{'unseated':>18}  note")
    nulls = 0
    for k in sorted(set(ka) | set(kb)):
        ra, rb = ka.get(k), kb.get(k)
        if not ra or not rb:
            print(f"{k[0]:<32} {k[1]:<7} {'ROW ONLY ON ONE SIDE':>16}")
            continue
        za, zb = ra.get('zone_exclusive'), rb.get('zone_exclusive')
        ua, ub = ra.get('unseated'), rb.get('unseated')
        note = ''
        if za == 0 and zb == 0:
            note = 'NULL: the seeder was already clean on this board'
            nulls += 1
        elif zb is not None and za is not None and zb > za:
            note = 'REGRESSION: more intrusions after (the conjunct can only remove poses)'
        print(f"{k[0]:<32} {k[1]:<7} {str(za) + ' -> ' + str(zb):>16} "
              f"{str(ua) + ' -> ' + str(ub):>18}  {note}")
    print(f"\n{nulls} null row(s), reported rather than dropped.")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--out', help='write the measurement here')
    p.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'))
    p.add_argument('--board', action='append',
                   help='measure only this board (repeatable)')
    p.add_argument('--no-polish-only', action='store_true',
                   help='skip the polish-on arm')
    a = p.parse_args(argv)
    if a.diff:
        return diff(*a.diff)
    if not a.out:
        p.error('one of --out or --diff is required')
    arms = (False,) if a.no_polish_only else (False, True)
    return run(a.out, a.board or BOARDS, arms)


if __name__ == '__main__':
    sys.exit(main())
