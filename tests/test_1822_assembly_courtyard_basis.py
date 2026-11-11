"""#1822: check_assembly names the basis of a courtyard pair whose member
draws no courtyard.

The courtyard channel grades a part that draws no courtyard on a stand-in
(its .Fab body, silk, or its pad box: `GradedPart.source`), and KiCad reports
no courtyards_overlap for such a pair. The pair line used to print the bare
channel name, `J3 <-> U2  courtyard ... COURTYARD-BLOCKING`, and --json
carried no source, so the pair read as a KiCad courtyard finding.

On an in-repo board where J3 draws no courtyard (pad box) and U1 is graded on
its .Fab body:
  1. the J3 <-> U2 pair line and COURTYARD BLOCKING row name J3 and
     `pad_bbox`; the Q2 <-> U1 ones name U1 and `fab`;
  2. every --json courtyard pair carries `a_source` / `b_source`, equal to
     `legality.graded_parts_from_file`'s source for those parts;
  3. control: D2 <-> Q2 (two drawn courtyards) prints as before, and the
     counts and the verdict do not move.

    python3 tests/test_1822_assembly_courtyard_basis.py
"""
import contextlib
import io
import json
import os
import re
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

BOARD = os.path.join(REPO, 'kicad_files', 'watchy.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}"
          f"{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def pair_line(out, a, b):
    hits = [ln for ln in out.splitlines()
            if re.match(rf'^    {re.escape(a)} <-> {re.escape(b)}  courtyard', ln)]
    return hits[0] if hits else ''


def blocking_row(out, a, b):
    sec = out.split('COURTYARD BLOCKING (', 1)
    if len(sec) < 2:
        return ''
    hits = [ln for ln in sec[1].splitlines()
            if ln.startswith(f'    {a} <-> {b}  ')]
    return hits[0] if hits else ''


def main():
    run_utils.evidence(BOARD, 'the board')
    with tempfile.TemporaryDirectory() as td:
        js = os.path.join(td, 'a.json')
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             os.path.join(REPO, 'py_tools', 'check_assembly.py'),
                             BOARD, '--json', js], accept=True)
        out = r.stdout
        run_utils.evidence(js, 'the --json report')
        with open(js, encoding='utf-8') as f:
            doc = json.load(f)

    print("1. the pair line and the blocking row name the stand-in")
    ln = pair_line(out, 'J3', 'U2')
    check("J3 <-> U2 line names J3: pad_bbox",
          'J3: pad_bbox' in ln and 'no drawn courtyard' in ln, repr(ln))
    ln = pair_line(out, 'Q2', 'U1')
    check("Q2 <-> U1 line names U1: fab",
          'U1: fab' in ln and 'Q2:' not in ln, repr(ln))
    ln = blocking_row(out, 'J3', 'U2')
    check("COURTYARD BLOCKING row J3 <-> U2 names J3: pad_bbox",
          'J3: pad_bbox' in ln, repr(ln))
    ln = blocking_row(out, 'J1', 'SW1')
    check("COURTYARD BLOCKING row J1 <-> SW1 names both members",
          'J1: pad_bbox' in ln and 'SW1: fab' in ln, repr(ln))

    print("2. --json carries each member's source")
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
        src = {p.ref: p.source
               for p in legality.graded_parts_from_file(pcb, BOARD)}
    pairs = doc['courtyard_pairs']
    check("courtyard pairs present", len(pairs) >= 4, str(len(pairs)))
    bad = [(q['a'], q['b']) for q in pairs
           if q.get('a_source') != src.get(q['a'])
           or q.get('b_source') != src.get(q['b'])]
    check("every courtyard pair's a_source/b_source = GradedPart.source",
          not bad, str(bad))
    blk = {(q['a'], q['b']): q for q in doc['courtyard_blocking_pairs']}
    q = blk.get(('J3', 'U2')) or {}
    check("courtyard_blocking_pairs carry the sources too",
          q.get('a_source') == 'pad_bbox' and q.get('b_source') == 'courtyard',
          str(q))

    print("3. control: two drawn courtyards, counts and verdict")
    ln = pair_line(out, 'D2', 'Q2')
    check("D2 <-> Q2 line unchanged",
          ln.strip() == 'D2 <-> Q2  courtyard  1.092mm2 side F  '
                        'COURTYARD-BLOCKING', repr(ln))
    ln = blocking_row(out, 'D2', 'Q2')
    check("D2 <-> Q2 blocking row unchanged",
          ln.strip() == 'D2 <-> Q2  1.092mm2  depth 0.84mm  side F', repr(ln))
    check("counts unchanged",
          (doc['blocking'], doc['advisory'], doc['courtyard_blocking'])
          == (0, 6, 4),
          str((doc['blocking'], doc['advisory'], doc['courtyard_blocking'])))
    check("verdict buildable", 'VERDICT: buildable' in out)

    if failures:
        print(f"FAIL: {len(failures)} check(s): {failures}")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
