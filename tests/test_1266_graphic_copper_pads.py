#!/usr/bin/env python3
"""#1266: a pad on ANOTHER part's footprint graphic copper is seen by
check_drc, placement legality and check_assembly -- the filled interior too.

esp_prog's U2 (SOT-89) draws its tab as a filled `fp_poly` on F.Cu that runs
4 mm past its pads. `place_pose set C3 135.6 96.69 --rot 180` on the stripped
board put C3.1 (+3.3V) across the tab's east edge and C3.2 (GND) wholly inside
it, and place_pose called that `legal`, check_assembly `buildable`, and
check_drc reported only C3.1. A second check_drc leak: a foreign pad that
covered a VERTEX of the tab granted its own net to the tab, which then waived
its own short (C3 at 135.85 95.79 graded clean).

Checks:
  1. The census (`check_drc.footprint_graphic_pad_census`): C3.1 touching,
     C3.2 touching and wholly inside; the human board has no row, and U2's
     own pads on its own tab never make one (the own-pad lift).
  2. check_drc: both C3 pads on the repro board, C3.1 on the vertex board,
     the human board still clean with its 5 accepted own-copper rows; a
     foreign pad no longer grants the tab its net.
  3. The search's gate (`LegalityContext.pair_shortfall` / `pads_ok`): C3 on
     the tab is refused as a stack, C3 clear of it is admitted, and a pair
     that draws no graphic copper returns the pad channel's own object. An
     on-board seed licenses its own contact; an OFF-board seed (a pile) does
     not, even where the pile already overlapped the tab.
  4. place_pose refuses the repro FOR this reason (exit 4) and accepts a clear
     pose (the negative control).
  5. check_assembly: NOT BUILDABLE (exit 4) against the stripped board (C3
     moved); `--baseline` the repro board itself (nothing moved) and no
     baseline at all both stay buildable -- the courtyard gate's currency.
  6. An owner WITH a courtyard: U2 given an F.CrtYd covering only its pads is
     refused the same way -- the copper reaches past the courtyard.
  7. A net tie's own net on the tie's copper is a contact but not a stack
     (Kelvin ties sit on shunt pads by design); another net on it is.

    python3 tests/test_1266_graphic_copper_pads.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board                                   # noqa: E402
from kicad_parser import parse_kicad_pcb                            # noqa: E402
from check_drc import (run_drc, footprint_graphic_pad_census,       # noqa: E402
                       graphic_effective_nets)
from placement import legality                                      # noqa: E402
from run_utils import check as run_check, evidence                  # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
STRIP = os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py')
POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')
ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')
ON_TAB = ('135.6', '96.69')          # the issue's pose: one pad across, one inside
ON_CORNER = ('135.85', '95.79')      # pad 1 over the tab's NE vertex
CLEAR = ('132.3', '100.22')          # 0.25 mm west of C3's own pose, clear
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def summary(r):
    m = re.search(r'JSON_SUMMARY: (\{.*\})', r.stdout or '')
    return json.loads(m.group(1)) if m else {}


def place(src, out, xy, force=False, **kw):
    argv = [sys.executable, '-X', 'utf8', POSE, src, out,
            'set', 'C3', xy[0], xy[1], '--rot', '180']
    return run_check(argv + (['--force'] if force else []), **kw)


def rows_for(board, ref='C3'):
    return {r['pad_ref']: r for r in footprint_graphic_pad_census(
        quiet(parse_kicad_pcb, board), 0.25)['rows'] if r['ref'] == ref}


def drc(board):
    """(violations, accepted rows) from check_drc at 0.1 mm."""
    res = quiet(run_drc, board, clearance=0.1, quiet=True, print_summary=False)
    return ([v for v in res if not v.get('accepted')],
            [v for v in res if v.get('accepted')])


def u2_pad_hits(violations):
    return {v.get('pad_ref'): v for v in violations
            if v.get('type') == 'pad-segment' and 'U2' in (v.get('item2') or '')}


def net_tie_board():
    """NT1: a declared tie whose filled bar bridges its pads A (0,0) and
    B (2,0); R1.1 (net A) and R2.1 (net C) are other parts' pads on the bar."""
    from kicad_parser import Segment, Pad, Net, PCBData, Footprint

    def pad(ref, num, net, name, x, y):
        return Pad(pad_number=num, net_id=net, net_name=name, global_x=x,
                   global_y=y, local_x=x, local_y=y, size_x=0.4, size_y=0.4,
                   shape='rect', layers=['F.Cu'], drill=0, pad_type='smd',
                   component_ref=ref)
    a, b = pad('NT1', '1', 1, 'A', 0.0, 0.0), pad('NT1', '2', 2, 'B', 2.0, 0.0)
    r1, r2 = pad('R1', '1', 1, 'A', 1.0, 0.0), pad('R2', '1', 3, 'C', 1.4, 0.0)
    ring = ((0.0, -0.15), (2.0, -0.15), (2.0, 0.15), (0.0, 0.15))
    segs = [Segment(start_x=p[0], start_y=p[1], end_x=q[0], end_y=q[1],
                    width=0.1, layer='F.Cu', net_id=0, graphic=True,
                    owner_ref='NT1', graphic_kind='poly', graphic_filled=True,
                    graphic_ring=ring, drawn_width=0.1)
            for p, q in zip(ring, ring[1:] + ring[:1])]
    fps = {'NT1': Footprint(reference='NT1', footprint_name='L:NT', x=0, y=0,
                            rotation=0, layer='F.Cu', pads=[a, b],
                            net_tie_groups=[['1', '2']]),
           'R1': Footprint(reference='R1', footprint_name='L:R', x=1.0, y=0,
                           rotation=0, layer='F.Cu', pads=[r1]),
           'R2': Footprint(reference='R2', footprint_name='L:R', x=1.4, y=0,
                           rotation=0, layer='F.Cu', pads=[r2])}
    return PCBData(board_info=None, nets={1: Net(1, 'A'), 2: Net(2, 'B'),
                                          3: Net(3, 'C')},
                   footprints=fps, vias=[], segments=segs,
                   pads_by_net={1: [a, r1], 2: [b], 3: [r2]})


def add_u2_courtyard(src, dst):
    """U2 with an F.CrtYd rect over its PADS only (local y 1.0..2.3), so the
    tab reaches 2.4 mm past it."""
    with open(src, encoding='utf-8', newline='') as fh:
        txt = fh.read()
    i = txt.index('(property "Reference" "U2"')
    j = txt.rindex('(footprint ', 0, i)
    at = txt.index('(at ', j)
    end = txt.index('\n', at)
    rect = ('\n\t\t(fp_rect (start -2.3 1.0) (end 2.3 2.3) '
            '(stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))')
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(txt[:end] + rect + txt[end:])


def main():
    work = tempfile.mkdtemp(prefix='krt1266_')
    try:
        esp = os.path.join(work, 'esp.kicad_pcb')
        quiet(copy_board, ESP, esp)
        bare = os.path.join(work, 'esp_u.kicad_pcb')
        run_check([sys.executable, '-X', 'utf8', STRIP, esp, bare], accept=True)
        evidence(bare, 'stripped board')
        on_tab = os.path.join(work, 'on_tab.kicad_pcb')
        on_corner = os.path.join(work, 'on_corner.kicad_pcb')
        clear = os.path.join(work, 'clear.kicad_pcb')
        place(bare, on_tab, ON_TAB, force=True, accept=True)
        place(bare, on_corner, ON_CORNER, force=True, accept=True)
        evidence(on_tab, 'C3 on the tab')
        evidence(on_corner, 'C3 on the corner')

        print("1. the census")
        r = rows_for(on_tab)
        check("C3.1 touches U2's copper, across its edge (not inside)",
              'C3.1' in r and r['C3.1']['contact'] and r['C3.1']['owner_ref'] == 'U2'
              and not r['C3.1']['inside_filled'], str(r.get('C3.1')))
        check("C3.2 touches it and lies WHOLLY inside the filled tab",
              'C3.2' in r and r['C3.2']['contact'] and r['C3.2']['inside_filled'],
              str(r.get('C3.2')))
        check("... both on a net that is not the tab's own",
              all(not x['same_net'] and x['shortfall_mm'] > 0 for x in r.values()))
        human = footprint_graphic_pad_census(quiet(parse_kicad_pcb, ESP), 0.25)
        check("the human board has no row -- U2's own pads on its own tab "
              "never make one", human['rows'] == [], str(human['rows'][:2]))

        print("2. check_drc")
        viol, _acc = drc(on_tab)
        hits = u2_pad_hits(viol)
        check("C3.2 wholly inside the tab is reported (it touches no stroke)",
              'C3.2' in hits and hits['C3.2'].get('inside_filled'), str(list(hits)))
        check("C3.1 across the edge is still reported", 'C3.1' in hits)
        viol, _acc = drc(on_corner)
        hits = u2_pad_hits(viol)
        check("C3.1 over the tab's VERTEX is reported (it used to grant the "
              "tab its net)", 'C3.1' in hits, str(list(hits)))
        pcb = quiet(parse_kicad_pcb, on_corner)
        tab = [s for s in pcb.segments if getattr(s, 'graphic', False)
               and s.owner_ref == 'U2']
        eff = graphic_effective_nets(pcb)
        n33 = {n.name: i for i, n in pcb.nets.items()}['/+3.3V']
        check("the foreign pad grants the tab no net",
              all(n33 not in eff[id(s)] for s in tab))
        viol, acc = drc(esp)
        own = [a for a in acc if a.get('type') == 'footprint-own-copper']
        check("the human board stays clean", not viol, str(viol[:2]))
        check("... with its 5 own-copper rows still accepted (#995)",
              len(own) == 5, f"{len(own)} rows")

        print("3. the search's gate")
        pcb = quiet(parse_kicad_pcb, bare)
        parts = legality.build_part_pads(pcb.footprints, 0.25)
        graphics = legality.build_part_graphics(pcb, refs=set(parts))
        check("U2 is the one part drawing graphic copper", set(graphics) == {'U2'},
              str(sorted(graphics)))
        poses = {r: (fp.x, fp.y, fp.rotation or 0.0)
                 for r, fp in pcb.footprints.items()}
        ctx = legality.LegalityContext(parts, None, 0.25, poses.get, poses.get,
                                       graphics=graphics)
        tab_pose = (float(ON_TAB[0]), float(ON_TAB[1]), 180.0)
        sf = ctx.pair_shortfall('C3', 'U2', pose_a=tab_pose)
        check("C3 on the tab: a stack and a short against U2",
              sf.stack and sf.pad_overlap and sf.pad > 0, str(sf))
        why = {}
        check("pads_ok refuses it",
              not ctx.pads_ok('C3', *tab_pose, ['U2'], why=why), str(why))
        sf_rev = ctx.pair_shortfall('U2', 'C3', pose_b=tab_pose)
        check("... measured the same from U2's side", sf_rev.stack)
        clear_pose = (float(CLEAR[0]), float(CLEAR[1]), 180.0)
        check("C3 clear of the tab is admitted",
              ctx.pads_ok('C3', *clear_pose, ['U2']))
        plain = ctx.pair_shortfall('C3', 'R1')
        check("a pair drawing no graphic copper is the pad channel's own result",
              plain == ctx._pads_pair_shortfall('C3', 'R1'))
        # The licence: seeds AT the contact, on the board, then off it.
        gate = legality.BoardOutlineGate(pcb.board_info, 0.0)
        off = gate.bounds[2] + 20.0 - poses['U2'][0]
        for label, dx, expect in (("an ON-board seed already on the tab licenses "
                                   "that contact", 0.0, True),
                                  ("an OFF-board seed on the tab (a pile) "
                                   "licenses nothing", off, False)):
            seeds = dict(poses)
            seeds['C3'] = (tab_pose[0] + dx, tab_pose[1], 180.0)
            seeds['U2'] = (poses['U2'][0] + dx,) + tuple(poses['U2'][1:])
            lic = legality.LegalityContext(parts, gate, 0.25, poses.get,
                                           seeds.get, graphics=graphics)
            check(label, lic.pads_ok('C3', *tab_pose, ['U2']) is expect)

        print("4. place_pose")
        out = os.path.join(work, 'refused.kicad_pcb')
        r4 = place(bare, out, ON_TAB, refuse='graphic_pad_stack_pairs new: C3/U2',
                   code=4)
        check("refused, nothing written", not os.path.exists(out))
        s = summary(r4)
        check("the summary names both pads",
              {x['pad_ref'] for x in s.get('graphic_pad_rows_after') or ()}
              == {'C3.1', 'C3.2'}, str(s.get('graphic_pad_rows_after')))
        r4c = place(bare, clear, CLEAR, accept=True)
        check("a clear pose is accepted and legal", summary(r4c).get('legal') is True,
              str(summary(r4c).get('refused')))

        print("5. check_assembly")
        r5 = run_check([sys.executable, '-X', 'utf8', ASSEMBLY, on_tab,
                        '--baseline', bare], refuse='NOT BUILDABLE', code=4)
        check("names C3.2 wholly inside U2's copper",
              re.search(r"C3\.2 \(GND\) on U2's copper.*wholly inside", r5.stdout)
              is not None)
        r5b = run_check([sys.executable, '-X', 'utf8', ASSEMBLY, on_tab,
                         '--baseline', on_tab], accept=True)
        check("--baseline the board itself: the pair is inherited, buildable",
              "baseline's own (no member moved)" in r5b.stdout
              and 'VERDICT: buildable' in r5b.stdout)
        r5c = run_check([sys.executable, '-X', 'utf8', ASSEMBLY, on_tab],
                        accept=True)
        check("no --baseline: reported, not gated (the courtyard currency)",
              'REPORT-ONLY' in r5c.stdout and 'VERDICT: buildable' in r5c.stdout)
        run_check([sys.executable, '-X', 'utf8', ASSEMBLY, clear,
                   '--baseline', bare], accept=True)
        check("the clear pose is buildable (negative control)", True)

        print("6. an owner WITH a courtyard")
        cy = os.path.join(work, 'esp_cy.kicad_pcb')
        add_u2_courtyard(bare, cy)
        if os.path.exists(os.path.splitext(bare)[0] + '.kicad_pro'):
            shutil.copy(os.path.splitext(bare)[0] + '.kicad_pro',
                        os.path.splitext(cy)[0] + '.kicad_pro')
        lb = quiet(legality.part_local_bounds, quiet(parse_kicad_pcb, cy), cy)['U2']
        check("U2 now grades on a COURTYARD, which does not reach the tab",
              lb.source == 'courtyard', lb.source)
        out = os.path.join(work, 'cy_refused.kicad_pcb')
        place(cy, out, ON_TAB, refuse='graphic_pad_stack_pairs new: C3/U2', code=4)
        check("place_pose still refuses C3 on the tab past U2's courtyard",
              not os.path.exists(out))

        print("7. a net tie's own net")
        rows = {r['pad_ref']: r for r in footprint_graphic_pad_census(
            net_tie_board(), 0.2, routing_layers=['F.Cu', 'B.Cu'])['rows']}
        check("R1.1 (net A) on NT1's A-B bridge: a contact, not a stack",
              rows.get('R1.1', {}).get('contact') is True
              and rows['R1.1']['stack'] is False and rows['R1.1']['net_tie'],
              str(rows.get('R1.1')))
        check("R2.1 (net C) on the same bridge: a stack",
              rows.get('R2.1', {}).get('stack') is True, str(rows.get('R2.1')))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print()
    if failures:
        print(f"FAIL: {len(failures)} check(s): {failures}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
