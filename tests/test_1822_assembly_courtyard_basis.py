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
     `legality.graded_parts_from_file`'s source for a member on its own
     side, and `drilled_pads` for a through-hole member on its far side;
  3. control: D2 <-> Q2 (two drawn courtyards) prints as before, and the
     counts and the verdict do not move;
  4. a pair on a through-hole member's FAR side names what was graded
     there, the boxes of its drilled pads, whatever its own side draws:
     watchy's REF** <-> SW1 (SW1 is on F and draws no courtyard: no longer
     `SW1: fab`), rp2350_fpga_eensy_prePlane's J1 <-> J3 (J1 draws F.CrtYd
     only: no longer the bare `courtyard`) and ulx3s's D51 <-> GPDI1.

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
#: what a through-hole member is graded on on its far side, as the line and
#: --json spell it (`legality.FAR_SOURCE_PADS`)
FAR = 'drilled_pads'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}"
          f"{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def pair_line(out, a, b, side=''):
    hits = [ln for ln in out.splitlines()
            if re.match(rf'^    {re.escape(a)} <-> {re.escape(b)}  courtyard', ln)
            and (not side or f' side {side}  ' in ln)]
    return hits[0] if hits else ''


def assembly(board, td):
    """(stdout, --json document) of check_assembly on a tracked board."""
    path = run_utils.evidence(os.path.join(REPO, 'kicad_files', board + '.kicad_pcb'),
                              'the board')
    js = os.path.join(td, board + '.json')
    r = run_utils.check([sys.executable, '-X', 'utf8',
                         os.path.join(REPO, 'py_tools', 'check_assembly.py'),
                         path, '--json', js], accept=True)
    with open(run_utils.evidence(js, 'the --json report'), encoding='utf-8') as f:
        return r.stdout, json.load(f)


def far_side(td, out, doc, sw1):
    """4. the far side of a through-hole member."""
    from placement import legality
    far = FAR
    ln = pair_line(out, 'REF**', 'SW1', 'B')
    check("watchy REF** <-> SW1 (side B) names SW1's drilled pads, not its "
          ".Fab body", f'SW1: {far} on its far side' in ln and 'SW1: fab' not in ln
          and 'REF**: fab' in ln and ln.count('no drawn courtyard on side B)') == 1,
          repr(ln))
    q = next((q for q in doc['courtyard_pairs']
              if (q['a'], q['b'], q['side']) == ('REF**', 'SW1', 'B')), {})
    check("its --json pair: REF** fab (own side), SW1 drilled_pads",
          (q.get('a_source'), q.get('b_source')) == ('fab', far), str(q))
    on = getattr(legality, 'courtyard_source_on', None)
    check("the source is by side: SW1 is fab on F, drilled_pads on B, and "
          "courtyard on B only for a footprint that draws that face",
          on is not None and getattr(legality, 'FAR_SOURCE_PADS', None) == far
          and (on(sw1, 'F'), on(sw1, 'B'), on(sw1, 'B', far_drawn=True))
          == ('fab', far, 'courtyard'),
          'legality.courtyard_source_on' if on is None else '')

    out, doc = assembly('rp2350_fpga_eensy_prePlane', td)
    ln = pair_line(out, 'J1', 'J3', 'B')
    check("rp2350 J1 <-> J3 (side B) is not the bare `courtyard`: J1 draws "
          "F.CrtYd only", ln.strip().startswith(
              f'J1 <-> J3  courtyard(J1: {far} on its far side; '
              f'no drawn courtyard on side B)  ') and 'J3:' not in ln, repr(ln))
    q = next((q for q in doc['courtyard_pairs']
              if (q['a'], q['b'], q['side']) == ('J1', 'J3', 'B')), {})
    check("its --json pair: J1 drilled_pads, J3 courtyard (own side)",
          (q.get('a_source'), q.get('b_source')) == (far, 'courtyard'), str(q))

    out, doc = assembly('ulx3s', td)
    ln = pair_line(out, 'D51', 'GPDI1', 'B')
    check("ulx3s D51 <-> GPDI1 (side B) names GPDI1's drilled pads, not fab",
          f'GPDI1: {far} on its far side' in ln and 'GPDI1: fab' not in ln
          and 'D51:' not in ln, repr(ln))
    q = next((q for q in doc['courtyard_pairs']
              if (q['a'], q['b'], q['side']) == ('D51', 'GPDI1', 'B')), {})
    check("its --json pair: D51 courtyard (own side), GPDI1 drilled_pads",
          (q.get('a_source'), q.get('b_source')) == ('courtyard', far), str(q))


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
        parts = {p.ref: p for p in legality.graded_parts_from_file(pcb, BOARD)}
    pairs = doc['courtyard_pairs']
    check("courtyard pairs present", len(pairs) >= 4, str(len(pairs)))

    def want(ref, side):
        # no watchy footprint draws a courtyard on both faces
        p = parts[ref]
        return p.source if p.side == side else FAR
    bad = [(q['a'], q['b'], q['side']) for q in pairs
           if q.get('a_source') != want(q['a'], q['side'])
           or q.get('b_source') != want(q['b'], q['side'])]
    check("every courtyard pair's a_source/b_source = GradedPart.source on "
          "the member's own side, drilled_pads on its far side", not bad, str(bad))
    n_far = sum(parts[q[k]].side != q['side'] for q in pairs for k in ('a', 'b'))
    check("and watchy has far-side members to say it of", n_far >= 1, str(n_far))
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

    print("4. a pair on a through-hole member's far side")
    with tempfile.TemporaryDirectory() as td:
        far_side(td, out, doc, parts['SW1'])

    if failures:
        print(f"FAIL: {len(failures)} check(s): {failures}")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
