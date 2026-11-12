#!/usr/bin/env python3
"""#1798: a route.py step's `skew_moved_by_cleanup` row is the shipped board's.

A step that routes something reports a skew-graded group twice over: the
post-route cleanup's comparison (#1759) and the end-of-run passes' (#1798).
The first one's after-value is the board right after the in-run cleanup, and
the end-of-run passes can change the group again.

The chain: an underpad QFN fanout, route_diff matches and records the pair
CSI_CLK, then `route.py --nets '*'` routes the two CSI_D0 nets single-ended.
Its post-route cleanup's stub-gap snap lays a 32 um segment on CSI_CLK_N
from a track end just outside a via's rim to a point inside it, which KiCad
then measures into the via, and names the pair at the skew that gives; the
end-of-run collapse then removes that segment. The row kept the first
number, which no board had.

Rows:
  1. the chain, CLI: the post-route line names the pair, the end-of-run
     passes change it again, and the pair's row in --json-out carries the
     skew check_drc measures on the written board, with a line giving that
     number -- or, when the pair ships within its rule, no row and one
     closing line;
  2. ...and batch_route in memory (the GUI's leg) returns the same rows;
  3. unit, on snapshots: the rows are one comparison, from the snapshot
     before the post-route cleanup to the shipped board. A group named in-run
     and shipped at another skew is one row at the shipped skew; shipped no
     further out than it came in is no row and a closing line; shipped as
     named is the row, unsaid twice; a group only the end-of-run passes moved
     is a row and a WARNING; a group out of range at the input that ships no
     further out is no row; a group the shipped board cannot measure keeps
     its in-run row.

    python3 tests/test_1798_shipped_skew_row.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb')
PAIR = {'CSI_CLK_P', 'CSI_CLK_N'}
RULE = 'r'


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def py(*argv):
    return run_check([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                     accept=True)


def pair_rows(rows):
    return [x for x in rows or [] if set(x['nets']) == PAIR]


def judge(tag, rows, log, shipped, said):
    """The pair's rows and lines against `shipped`, its row in
    check_drc.skew_group_snapshot of the board the step wrote. `said` is the
    skew the leg's post-route line gave, None when it printed none."""
    if shipped['excess_mm'] > 1e-4:
        check(f"{tag}: the pair's one row carries the shipped skew",
              len(rows) == 1
              and abs(rows[0]['skew_after_mm'] - shipped['skew_mm']) < 1e-4,
              f"shipped {shipped['skew_mm']:.4f} mm, rows "
              f"{[(x['skew_before_mm'], x['skew_after_mm']) for x in rows]}")
        check(f"{tag}: a WARNING gives the shipped skew", re.search(
            r"WARNING[^\n]*\(#1798\)[^\n]* moved CSI_CLK_P / CSI_CLK_N out of "
            r"its skew rule[^\n]*-> %.4f mm" % shipped['skew_mm'], log) is not None)
    else:
        check(f"{tag}: a pair that ships within its rule has no row", rows == [],
              str(rows))
        if said is not None:
            check(f"{tag}: one closing line says it ships within", len(re.findall(
                r"CSI_CLK_P / CSI_CLK_N ships within its skew rule[^\n]*skew "
                r"%.4f mm after the end-of-run cleanup \(#1798\), not the %.4f mm"
                % (shipped['skew_mm'], said), log)) == 1)


def chain_rows(tmp):
    from kicad_parser import parse_kicad_pcb
    from check_drc import skew_group_snapshot
    a, f, b, c = (os.path.join(tmp, f'{t}.kicad_pcb') for t in 'afbc')
    py(tool('copy_board.py'), BOARD, a)
    py(tool('qfn_fanout.py'), evidence(a, 'board copy'), '-o', f,
       '--component', 'U1', '--nets', 'CSI_*', '--escape-method', 'underpad',
       '--layer', 'In1.Cu', '--width', '0.127', '--via-size', '0.5',
       '--via-drill', '0.2', '--allow-via-in-pad')
    py(tool('route_diff.py'), evidence(f, 'fanout board'), b,
       '--nets', 'CSI_CLK*', '--layers', 'F.Cu', 'In1.Cu',
       '--track-width', '0.127', '--diff-pair-gap', '0.2', '--via-size', '0.6',
       '--diff-pair-intra-match')

    print("1. the step that routes the other nets (CLI)")
    js = os.path.join(tmp, 'c.json')
    r = py(tool('route.py'), evidence(b, 'route_diff board'), c, '--nets', '*',
           '--json-out', js)
    log = r.stdout + r.stderr
    m = re.search(r"WARNING[^\n]*the post-route cleanup moved CSI_CLK_P / "
                  r"CSI_CLK_N out of its skew rule[^\n]*skew ([0-9.]+) -> "
                  r"([0-9.]+) mm", log)
    if m is None:
        print("  [BROKEN TEST] the post-route cleanup named no pair: nothing "
              "here has an in-run row to compare with the shipped board\n"
              + log[-800:])
        return False
    said = float(m.group(2))
    pc = parse_kicad_pcb(evidence(c, 'routed board'))
    shipped = [v for v in skew_group_snapshot(pc, c).values()
               if v and set(v['nets']) == PAIR]
    if len(shipped) != 1:
        print("  [BROKEN TEST] the written board does not grade the pair: "
              + str(shipped))
        return False
    shipped = shipped[0]
    if abs(shipped['skew_mm'] - said) < 1e-4:
        print("  [BROKEN TEST] the pair ships at the skew the post-route line "
              f"gave ({said:.4f} mm): the end-of-run passes did not touch it")
        return False
    print(f"  post-route line: {float(m.group(1)):.4f} -> {said:.4f} mm; the "
          f"written board: {shipped['skew_mm']:.4f} mm")
    with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
        cli = pair_rows(json.load(fh).get('skew_moved_by_cleanup'))
    judge('--json-out', cli, log, shipped, said)

    print("2. ...and batch_route in memory")
    from route import batch_route
    pcb = parse_kicad_pcb(b)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        _ok, _fail, _t, rd = batch_route(
            b, '', [n.name for n in pcb.nets.values() if n.name],
            pcb_data=pcb, return_results=True)
    gui = pair_rows(rd.get('skew_moved_by_cleanup'))
    m = re.search(r"the post-route cleanup moved CSI_CLK_P / CSI_CLK_N out of its "
                  r"skew rule[^\n]*-> ([0-9.]+) mm", buf.getvalue())
    judge('results_data', gui, buf.getvalue(), shipped,
          float(m.group(1)) if m else None)
    return True


def snap(skew, ids=(1, 2), mx=0.1):
    return {(RULE, ids): None if skew is None else {
        'nets': ['A_P', 'A_N'], 'rule': RULE, 'skew_mm': skew, 'min_mm': None,
        'max_mm': mx, 'excess_mm': round(max(skew - mx, 0.0), 4)}}


def unit_rows():
    print("3. unit: the rows, from snapshots")
    try:
        from route import _shipped_skew_rows1798, _skew_by_name
    except ImportError as e:
        check("route.py compares the in-run rows with the shipped board", False,
              str(e))
        return
    named = [{'nets': ['A_P', 'A_N'], 'rule': RULE, 'skew_before_mm': 0.0878,
              'skew_after_mm': 0.39, 'min_mm': None, 'max_mm': 0.1,
              'was_within': True}]

    def run(in_run, at_input, shipped):
        # The shipped board is a re-parse: its net ids are (7, 8), and
        # _late_skew_snapshot1798 keys it by name.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rows = _shipped_skew_rows1798(in_run, at_input,
                                          _skew_by_name(shipped))
        return ([(x['skew_before_mm'], x['skew_after_mm']) for x in rows],
                buf.getvalue())

    rows, out = run(named, snap(0.0878), snap(0.3378, (7, 8)))
    check("named in-run, shipped at another skew: one row at the shipped skew",
          rows == [(0.0878, 0.3378)], str(rows))
    check("...and one WARNING gives it", out.count('WARNING') == 1
          and 'the post-route and end-of-run cleanups (#1798) moved A_P / A_N'
          in out and '0.0878 -> 0.3378 mm' in out, out.strip())
    rows, out = run(named, snap(0.0878), snap(0.0878, (7, 8)))
    check("named in-run, shipped as it came in: no row", rows == [], str(rows))
    check("...and one closing line", ' moved ' not in out and out.count('\n') == 1
          and "A_P / A_N ships within its skew rule 'r' (max 0.1 mm): skew "
              "0.0878 mm after the end-of-run cleanup (#1798), not the 0.3900 mm"
          in out, out.strip())
    rows, out = run(named, snap(0.0878), snap(0.39, (7, 8)))
    check("named in-run, shipped as named: the row, and nothing said twice",
          rows == [(0.0878, 0.39)] and out == '', f"{rows} {out.strip()}")
    rows, out = run([], snap(0.05), snap(0.3, (7, 8)))
    check("moved by the end-of-run passes only: its row and its WARNING",
          rows == [(0.05, 0.3)] and out.count('WARNING') == 1
          and 'cleanups (#1798) moved A_P / A_N' in out,
          f"{rows} {out.strip()}")
    rows, out = run([], snap(0.3), snap(0.25, (7, 8)))
    check("out of range at the input and shipped no further out: no row",
          rows == [] and out == '', f"{rows} {out.strip()}")
    rows, out = run(named, snap(0.0878), snap(None, (7, 8)))
    check("not measurable on the shipped board: the in-run row stands",
          rows == [(0.0878, 0.39)] and out == '', f"{rows} {out.strip()}")


def main():
    tmp = tempfile.mkdtemp(prefix='t1798s_')
    try:
        if not chain_rows(tmp):
            return 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    unit_rows()
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
