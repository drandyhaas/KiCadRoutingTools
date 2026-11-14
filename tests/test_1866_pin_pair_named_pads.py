#!/usr/bin/env python3
"""#1866: a `pin_pair` length group can name its two pads, and names the pair
it measured.

`board_score --length-groups` in `"mode": "pin_pair"` measures each net
between its first two pads in the order the board file lists its footprints,
and said nowhere which pads those were. On rp2350_fpga_eensy_prePlane the two
USB-C data nets have three pads each (U3 and the connector's two contacts):
the group read spread 0.1886 mm between U3 and J1.A6/A7, and on the same
board with only the U3 footprint block moved to the end of the file it read
0.3886 mm between the connector's own two contacts, and failed.

Now a group may carry `"pads": {net: ["REF.PAD", "REF.PAD"]}`, which is
measured whatever the file order, and every `pin_pair` group names the pair
each net was measured between in `measured_pads`. A net the key does not
name is measured between the pair `pin_pair_pads` chooses from its COPPER
pads whatever the file order: its two pads, or on a net of three or more the
two farthest apart along the copper (by straight line while the net is not
joined), ties to the labels first in natural order -- and the group's
`ambiguous_pair` names each net whose pair was chosen from three or more.

    python3 tests/test_1866_pin_pair_named_pads.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))
import run_utils  # noqa: E402
from synth import RoutingBoard  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import net_queries as nq  # noqa: E402
import board_score  # noqa: E402

BOARD = os.path.join(REPO, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
DP, DN = 'Net-(J1-D+-PadA6)', 'Net-(J1-D--PadA7)'
LANE = {DP: ['U3.6', 'J1.A6'], DN: ['U3.4', 'J1.A7']}
# the same lane as the default pair names it: the two pads farthest apart
# along the copper (U3 -> J1.A6 2.537 mm, over J1.A6 -> J1.B6 1.637 and
# U3.6 -> J1.B6 0.900), in label order
FARTHEST = {DP: ['J1.A6', 'U3.6'], DN: ['J1.A7', 'U3.4']}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _block_end(text, start):
    depth, j = 0, start
    while True:
        depth += (text[j] == '(') - (text[j] == ')')
        j += 1
        if depth == 0:
            return j


def moved_copy(dst):
    """The board with the U3 footprint block moved to the end of the
    footprints: the same copper, nets and pads in another file order."""
    with open(BOARD, encoding='utf-8', newline='') as fh:
        t = fh.read()
    s = t.rindex('\n\t(footprint ', 0, t.index('(property "Reference" "U3"')) + 1
    s0 = t.index('(', s)
    j = _block_end(t, s0)
    block, rest = t[s0:j], t[:s] + t[j:]
    k = _block_end(rest, rest.index('(', rest.rindex('\n\t(footprint ')))
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(rest[:k] + '\n\t' + block + rest[k:])
    return run_utils.evidence(dst) or dst


def hole_first(dst):
    """Net SIG's first pad in file order is an unplated hole tied to the net,
    which carries no copper; then pads A and B joined by a track. SIG2 is two
    pads and a track."""
    rb = RoutingBoard(w=40, h=20)
    rb.items.append(
        '  (footprint "synth:H" (layer "F.Cu") (uuid "%s") (at 5 5)\n'
        '    (property "Reference" "H1" (at 0 -2 0) (layer "F.SilkS") (uuid "%s")\n'
        '      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        '    (pad "" np_thru_hole circle (at 0 0) (size 1.2 1.2) (drill 1.2) '
        '(layers "*.Cu" "*.Mask") (net %d "SIG") (uuid "%s"))\n  )'
        % (rb._uid(), rb._uid(), rb.net('SIG'), rb._uid()))
    rb.pad('A', 10, 10, 'SIG')
    rb.pad('B', 30, 10, 'SIG')
    rb.seg(10, 10, 30, 10, 'SIG', w=0.2)
    rb.pad('C', 10, 15, 'SIG2')
    rb.pad('D', 30, 15, 'SIG2')
    rb.seg(10, 15, 30, 15, 'SIG2', w=0.2)
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    return run_utils.evidence(dst) or dst


def star(rb, order=('Z', 'X', 'Y')):
    """Net STAR: pads X (10, 30), Y (30, 30) and Z (20, 40), each on a track
    to a joint at (20, 30), so every pair is 20 mm apart along the copper (a
    three-way tie), added to the file in `order`. Net LOOSE: the same three
    pads 20 mm lower with no copper, X-Y 20 mm apart in a straight line and
    either to Z 14.14 mm."""
    at = {'X': (10, 30), 'Y': (30, 30), 'Z': (20, 40)}
    for ref in order:
        x, y = at[ref]
        rb.pad(ref, x, y, 'STAR')
        rb.seg(x, y, 20, 30, 'STAR', w=0.2)
    for ref in order:
        x, y = at[ref]
        rb.pad(ref + 'L', x, y + 20, 'LOOSE')


def star_board(dst, order):
    """A board of nets STAR and LOOSE alone, their pads in `order`."""
    rb = RoutingBoard(w=40, h=70)
    star(rb, order)
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    return run_utils.evidence(dst) or dst


def group(board, td, spec):
    """One length group `G` through board_score.score_length -> (result, G)."""
    path = os.path.join(td, 'groups.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump({'G': spec}, fh)
    with contextlib.redirect_stdout(io.StringIO()):
        res = board_score.score_length(board, path)
    return res, (res.get('groups') or {}).get('G') or {}


def main():
    td = tempfile.mkdtemp(prefix='krt1866_')
    try:
        moved = moved_copy(os.path.join(td, 'moved.kicad_pcb'))
        base = {'nets': [DP, DN], 'tolerance_mm': 0.25, 'mode': 'pin_pair'}

        print('1. a named pair is measured whatever the file order')
        seen = {}
        for tag, b in (('as shipped', BOARD), ('U3 moved to the end', moved)):
            res, g = group(b, td, dict(base, pads=LANE))
            seen[tag] = g.get('spread_mm')
            check(f'{tag}: the group passes on the lane from U3 to the connector '
                  f'(spread 0.1886 mm)', res.get('count') == 0 and g.get('pass') is True
                  and g.get('spread_mm') == 0.1886, json.dumps(g)[:300])
            check(f'{tag}: it names the pads it measured, and reads the key',
                  g.get('measured_pads') == LANE and 'unknown_keys' not in g,
                  json.dumps(g)[:300])
        check('the two file orders read one spread', len(set(seen.values())) == 1,
              str(seen))

        print('2. an unnamed net: its two copper pads farthest apart along the '
              'copper, whatever the file order, flagged')
        for tag, b in (('as shipped', BOARD), ('U3 moved to the end', moved)):
            res, g = group(b, td, base)
            check(f'{tag}: U3 to the first contact, spread 0.1886 mm, pass',
                  g.get('measured_pads') == FARTHEST and g.get('spread_mm') == 0.1886
                  and g.get('pass') is True, json.dumps(g)[:300])
            check(f'{tag}: each 3-pad net is in ambiguous_pair, with its pair',
                  g.get('ambiguous_pair') == FARTHEST, json.dumps(g)[:400])
        res, g = group(moved, td, dict(base, pads={DP: LANE[DP]}))
        check('a key naming one net of two: that net as named and not flagged, '
              'the other chosen and flagged',
              g.get('measured_pads') == {DP: LANE[DP], DN: FARTHEST[DN]}
              and g.get('ambiguous_pair') == {DN: FARTHEST[DN]},
              json.dumps(g)[:400])

        print('3. net_queries: the named pair is the measured pair')
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(moved)
        nid = {n.name: i for i, n in pcb.nets.items()}[DP]
        by = {nq.pad_label(p): p for p in pcb.pads_by_net[nid]}
        want = nq.pin_pair_path_length(pcb, nid, by['U3.6'], by['J1.B6'])
        got = nq.net_match_length(pcb, nid, 'pin_pair', pads=('U3.6', 'J1.B6'))
        check('net_match_length(pads=two labels) is the walk between those pads',
              want is not None and got == want, f'{got} vs {want}')
        check('... and takes the Pad objects as well',
              nq.net_match_length(pcb, nid, 'pin_pair',
                                  pads=(by['U3.6'], by['J1.B6'])) == want)
        how = {}
        check('pin_pair_pads without a pair: the two farthest along the copper, '
              'not the first two in file order (J1.A6, J1.B6 there)',
              [nq.pad_label(p) for p in nq.pin_pair_pads(pcb, nid, detail=how)]
              == FARTHEST[DP] and how == {'how': 'farthest_along_copper',
                                          'ambiguous': True}, str(how))
        for order in (('Z', 'X', 'Y'), ('Y', 'Z', 'X')):
            with contextlib.redirect_stdout(io.StringIO()):
                sp = parse_kicad_pcb(star_board(
                    os.path.join(td, 'star_%s.kicad_pcb' % ''.join(order)), order))
            sids = {n.name: i for i, n in sp.nets.items()}
            how = {}
            got = [nq.pad_label(p) for p in nq.pin_pair_pads(sp, sids['STAR'],
                                                              detail=how)]
            check(f'a three-way tie, file order {"".join(order)}: the labels that '
                  f'come first (X.1, Y.1)', got == ['X.1', 'Y.1']
                  and how.get('how') == 'farthest_along_copper', f'{got} {how}')
            how = {}
            got = [nq.pad_label(p) for p in nq.pin_pair_pads(sp, sids['LOOSE'],
                                                              detail=how)]
            check(f'no copper, file order {"".join(order)}: the farthest in a '
                  f'straight line (XL.1, YL.1), which reads None',
                  got == ['XL.1', 'YL.1']
                  and how == {'how': 'farthest_straight_line', 'ambiguous': True}
                  and nq.net_match_length(sp, sids['LOOSE'], 'pin_pair') is None,
                  f'{got} {how}')
        by['J1.B6'].die_length = 1.0
        check('the pad-to-die term follows the named pair',
              nq.net_match_pad_to_die(pcb, nid, 'pin_pair', pads=('U3.6', 'J1.B6')) == 1.0
              and nq.net_match_pad_to_die(pcb, nid, 'pin_pair', pads=('U3.6', 'J1.A6')) == 0.0
              and nq.net_match_length(pcb, nid, 'pin_pair', pads=('U3.6', 'J1.B6')) == want + 1.0)
        by['J1.B6'].die_length = 0.0
        for label, pads, why in (
                ('a pad of another net', ('U3.6', 'U3.4'), 'U3.4 names 0 of its copper pads'),
                ('one pad twice', ('U3.6', 'U3.6'), 'U3.6 is named twice'),
                ('three pads', ('U3.6', 'J1.A6', 'J1.B6'), '3 pad(s) given, a pin pair is two')):
            try:
                nq.pin_pair_pads(pcb, nid, pads)
                said = 'no error'
            except ValueError as exc:
                said = str(exc)
            check(f'{label} is refused, saying so', why in said and DP in said, said)

        print('4. a "pads" key that cannot be read fails the group, with the reason')
        for label, spec, why in (
                ('a pad that is not on the net', dict(base, pads={DP: ['U3.6', 'U3.4']}),
                 f'"pads": net {DP}: U3.4 names 0 of its copper pads (J1.A6, J1.B6, U3.6)'),
                ('a net that is not in the group', dict(base, pads={'GND': ['U3.6', 'J1.A6']}),
                 '"pads" names \'GND\', which is not a net of this group'),
                ('a pair that is no list of two labels', dict(base, pads={DP: 'U3.6'}),
                 '"pads" must be {net: ["REF.PAD", "REF.PAD"]}'),
                ('a total-copper group', dict(base, mode='total', pads=LANE),
                 'this group measures total copper')):
            res, g = group(BOARD, td, spec)
            check(f'{label}: the group fails and says why',
                  res.get('count') == 1 and g.get('pass') is False
                  and why in (g.get('skipped') or ''), json.dumps(g)[:300])

        for label, spec in (
                ('a listed chain', {'chains': [[DP], [DN]]}),
                ('"chains": "auto"', {'chains': 'auto'})):
            res, g = group(BOARD, td, dict(spec, tolerance_mm=0.25, mode='pin_pair',
                                           pads={'NO_SUCH_NET': ['U3.6', 'J1.A6']}))
            check(f'a net that is in no chain, {label}: the group fails and says why',
                  res.get('count') == 1 and g.get('pass') is False
                  and g.get('skipped') == '"pads" names \'NO_SUCH_NET\', which is '
                                          'not a net of this group',
                  json.dumps(g)[:300])

        print('4b. the default pair on a net whose first pad has no copper')
        holes = hole_first(os.path.join(td, 'hole.kicad_pcb'))
        with contextlib.redirect_stdout(io.StringIO()):
            hp = parse_kicad_pcb(holes)
        sig = {n.name: i for i, n in hp.nets.items()}['SIG']
        try:
            said = repr(nq.net_match_length(hp, sig, 'pin_pair'))
        except Exception as exc:                                # noqa: BLE001
            said = f'{type(exc).__name__}: {exc}'
        check('net_match_length measures the two copper pads, past the hole '
              '(20.0 mm; it read None from the hole)', said == '20.0', said)
        res, g = group(holes, td, {'nets': ['SIG', 'SIG2'], 'mode': 'pin_pair',
                                   'tolerance_mm': 1.0})
        check('the group is measured between the copper pads, and a net of two '
              'copper pads is not flagged ambiguous',
              g.get('pass') is True and g.get('spread_mm') == 0.0
              and g.get('measured_pads') == {'SIG': ['A.1', 'B.1'], 'SIG2': ['C.1', 'D.1']}
              and 'ambiguous_pair' not in g, json.dumps(g)[:300])
        res, g = group(holes, td, {'nets': ['SIG', 'SIG2'], 'mode': 'pin_pair',
                                   'tolerance_mm': 1.0, 'pads': {'SIG': ['A.1', 'B.1']}})
        check('named, the same net is measured between its two copper pads',
              g.get('pass') is True and g.get('spread_mm') == 0.0
              and g.get('measured_pads', {}).get('SIG') == ['A.1', 'B.1'],
              json.dumps(g)[:300])
        res, g = group(holes, td, {'nets': ['SIG', 'SIG2'], 'mode': 'pin_pair',
                                   'tolerance_mm': 1.0, 'pads': {'SIG': ['H1.', 'B.1']}})
        check('naming the hole is refused: it is not a copper pad of the net',
              g.get('pass') is False and 'H1. names 0 of its copper pads (A.1, B.1)'
              in (g.get('skipped') or ''), json.dumps(g)[:300])

        print('5. chains read the key too, and the controls are unchanged')
        res, g = group(moved, td, {'chains': [[DP], [DN]], 'tolerance_mm': 0.25,
                                   'mode': 'pin_pair', 'pads': LANE})
        rows = g.get('chains') or {}
        check('a chains group measures the named pairs and names them',
              g.get('pass') is True and g.get('spread_mm') == 0.1886
              and (rows.get(DP) or {}).get('measured_pads') == {DP: LANE[DP]}
              and (rows.get(DN) or {}).get('measured_pads') == {DN: LANE[DN]}
              and 'ambiguous_pair' not in g, json.dumps(g)[:400])
        res, g = group(moved, td, {'chains': [[DP], [DN]], 'tolerance_mm': 0.25,
                                   'mode': 'pin_pair'})
        rows = g.get('chains') or {}
        check('a chains group without the key: the farthest pairs, flagged',
              g.get('pass') is True and g.get('spread_mm') == 0.1886
              and (rows.get(DP) or {}).get('measured_pads') == {DP: FARTHEST[DP]}
              and g.get('ambiguous_pair') == FARTHEST, json.dumps(g)[:400])
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             os.path.join(REPO, 'py_tools', 'board_score.py'),
                             '--help'], accept=True)
        helptext = ' '.join(r.stdout.split())
        check('board_score --help says how an unnamed pair is chosen and flagged',
              'farthest apart along the copper' in helptext
              and 'ambiguous_pair' in helptext, helptext[:200])
        for tag, b in (('as shipped', BOARD), ('U3 moved', moved)):
            res, g = group(b, td, {'nets': [DP, DN], 'tolerance_mm': 0.25})
            check(f'{tag}: a total-copper group reads 0.2114 mm and names no pads',
                  g.get('spread_mm') == 0.2114 and g.get('pass') is True
                  and 'measured_pads' not in g, json.dumps(g)[:300])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
