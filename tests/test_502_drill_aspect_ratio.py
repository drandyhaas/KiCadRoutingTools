#!/usr/bin/env python3
"""#502: a through via's drill is held to the board thickness's aspect ratio.

Aspect ratio (board thickness / finished hole) is a hard fab gate, and nothing
modelled it: a default run (`--fab-tier auto`, `--escalation fab`) escalated
to the advanced 0.25/0.15 via on a board of any thickness, which is 13.3:1 on
a 2.0 mm board, and check_drc graded that drill at the same 0.15 floor.

Each tier is held to its own floor drill at the 1.6 mm board a fab quotes its
minimum drill for: standard 8:1 (1.6 / 0.20), advanced 10.67:1 (1.6 / 0.15).
So:

  1. check_drc grades a through via's drill at thickness / ratio on a thicker
     board, and names the aspect ratio; a 1.6 mm board and a blind via are
     graded as before.
  2. the rungs a run descends through carry that drill (the diameter grown to
     keep the rung's ring), at both tiers; on a board up to 1.6 mm thick they
     are the floor table's, unchanged.
  3. the thickness is the stackup's entries summed (masks are not entries),
     else `(general (thickness ...))`, else unknown -- and unknown raises
     nothing.
  4. an explicit drill below the floor is pinned up, as one below the
     physical fab floor is.
  5. the run's `Design rules [...]` line and JSON_SUMMARY say so in aspect
     terms; `--strict-sizes` does not count it, since it narrows nothing.
  6. a smaller drill a `--fab-overrides` file declares raises nothing on a
     1.6 mm board either.

    python3 tests/test_502_drill_aspect_ratio.py
"""
import io
import os
import sys
import tempfile
from contextlib import redirect_stdout
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import fab_tiers  # noqa: E402
from run_utils import evidence  # noqa: E402

#: A 4-layer stackup summing to 2.0 mm: the first prepreg is two sheets
#: (`addsublayer`), and the masks are not entries the sum reads.
STACKUP_2MM = """
		(stackup
			(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
			(layer "F.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 1" (type "prepreg") (thickness 0.2) (material "a")
				(epsilon_r 4.2) (loss_tangent 0.02)
				addsublayer (thickness 0.2) (material "b") (epsilon_r 4.4) (loss_tangent 0.02))
			(layer "In1.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 2" (type "core") (thickness 1.03) (material "FR4")
				(epsilon_r 4.5) (loss_tangent 0.02))
			(layer "In2.Cu" (type "copper") (thickness 0.035))
			(layer "dielectric 3" (type "prepreg") (thickness 0.43) (material "FR4")
				(epsilon_r 4.5) (loss_tangent 0.02))
			(layer "B.Cu" (type "copper") (thickness 0.035))
			(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
		)"""


def board(general, stackup='', vias=('(via (at 10 10) (size 0.25) (drill 0.15) '
                                      '(layers "F.Cu" "B.Cu") (net 1))',)):
    gen = f'\n\t(general\n\t\t(thickness {general})\n\t)' if general else ''
    return f"""(kicad_pcb
	(version 20241229)
	(generator "pcbnew"){gen}
	(layers
		(0 "F.Cu" signal)
		(4 "In1.Cu" signal)
		(6 "In2.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(setup{stackup}
	)
	(net 0 "")
	(net 1 "A")
	(gr_rect (start 0 0) (end 20 20) (stroke (width 0.05) (type default))
		(fill no) (layer "Edge.Cuts"))
	{chr(10).join(vias)}
)
"""


def write(tmp, name, text):
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return evidence(path, name)


def install(path, tier='auto', overrides=None):
    """What a routing CLI does with its board before routing."""
    fab_tiers.set_default_fab_tier(tier, overrides or {})
    with redirect_stdout(io.StringIO()):
        fab_tiers.set_policy_from_args(SimpleNamespace(escalation='fab'), path)


def vias(rungs):
    return [(round(r['via_diameter'], 4), round(r['via_drill'], 4)) for r in rungs]


def drill_rows(path):
    from check_drc import run_drc
    with redirect_stdout(io.StringIO()):
        viols = run_drc(path, clearance=0.1, quiet=True, print_summary=False)
    return [v for v in viols if v['type'] == 'via-drill-size']


def main():
    tmp = tempfile.mkdtemp(prefix='t502_')
    thick = write(tmp, 'thick.kicad_pcb', board(1.6, STACKUP_2MM))
    thin = write(tmp, 'thin.kicad_pcb', board(1.6))
    general = write(tmp, 'general.kicad_pcb', board(2.4))
    blind = write(tmp, 'blind.kicad_pcb', board(1.6, STACKUP_2MM, vias=(
        '(via blind (at 10 10) (size 0.25) (drill 0.15) (layers "F.Cu" "In1.Cu") (net 1))',)))
    bare = write(tmp, 'bare.kicad_pcb', board(None))
    try:
        print('1. check_drc grades a through drill at the aspect floor on a thick board')
        rows = drill_rows(thick)
        assert len(rows) == 1, f'the 0.15 drill on a 2.0 mm board (13.3:1) graded clean: {rows}'
        r = rows[0]
        assert abs(r['min_drill'] - 0.188) < 1e-9, r
        assert (r.get('aspect_ratio'), r.get('max_aspect_ratio'),
                r.get('board_thickness_mm')) == (13.33, 10.67, 2.0), r
        assert drill_rows(thin) == [], 'a 1.6 mm board grades the 0.15 drill clean, as before'
        assert drill_rows(blind) == [], 'a blind via is not drilled through the board'
        print('   PASS')

        print('2. the rungs carry the raised drill at both tiers; a 1.6 mm board is unchanged')
        table = {}
        for tier in ('auto', 'standard', 'advanced'):
            fab_tiers.set_board_thickness(None)
            table[tier] = vias(fab_tiers.fab_floor_ladder(4, tier, {}))
        install(thin)
        assert vias(fab_tiers.escalation_rungs(4)) == table['auto'] == \
            [(0.45, 0.2), (0.3, 0.15), (0.25, 0.15)], table
        assert fab_tiers.physical_fab_floor(4)['via_drill'] == 0.15
        install(thick)
        assert vias(fab_tiers.escalation_rungs(4)) == \
            [(0.5, 0.25), (0.338, 0.188), (0.288, 0.188)], vias(fab_tiers.escalation_rungs(4))
        assert vias([fab_tiers.fab_floor_min(4)]) == [(0.288, 0.188)]
        assert vias([fab_tiers.physical_fab_floor(2)]) == [(0.288, 0.188)]
        install(thick, 'standard')
        assert vias(fab_tiers.escalation_rungs(4)) == [(0.5, 0.25)], 'standard is 8:1'
        install(thick, 'advanced')
        assert vias(fab_tiers.escalation_rungs(4)) == [(0.288, 0.188)], 'advanced is 10.67:1'
        for t, want in ((1.58, 0.15), (1.6, 0.15), (2.0, 0.188), (3.2, 0.3)):
            fab_tiers.set_board_thickness(t)
            got = fab_tiers.fab_floor_min(4, 'auto', {})['via_drill']
            assert abs(got - want) < 1e-9, (t, got, want)
        print('   PASS')

        print('3. the thickness: stackup sum, else (general (thickness)), else unknown')
        from kicad_parser import StackupLayer, board_thickness_mm, parse_kicad_pcb
        assert parse_kicad_pcb(thick).board_info.board_thickness == 2.0
        assert parse_kicad_pcb(thin).board_info.board_thickness == 1.6
        assert parse_kicad_pcb(general).board_info.board_thickness == 2.4
        assert parse_kicad_pcb(bare).board_info.board_thickness is None
        assert board_thickness_mm([StackupLayer('F.Cu', 'copper', 0.035),
                                   StackupLayer('d', 'core', 1.51),
                                   StackupLayer('B.Cu', 'copper', 0.035)], 1.6) == 1.58
        assert board_thickness_mm([], None) is None
        assert fab_tiers.board_thickness_of_file(thick) == 2.0
        install(bare)
        assert fab_tiers.get_board_thickness() is None
        assert vias(fab_tiers.escalation_rungs(4)) == table['auto'], 'unknown raises nothing'
        print('   PASS')

        print('4. an explicit drill below the aspect floor is pinned up')
        install(thick)
        with redirect_stdout(io.StringIO()) as out:
            pinned = fab_tiers.enforce_fab_floors(4, via_size=0.25, via_drill=0.15)
        assert pinned == {'via_drill': 0.188, 'via_size': 0.288}, (pinned, out.getvalue())
        assert '--via-drill 0.15 is below the physical fab floor 0.188' in out.getvalue()
        assert fab_tiers.fab_floor_for_param('via_drill', 4, 'auto', {}, thickness=1.6) == 0.15
        install(thin)
        with redirect_stdout(io.StringIO()):
            assert fab_tiers.enforce_fab_floors(4, via_size=0.25, via_drill=0.15) == {}
        print('   PASS')

        print('5. the Design rules line and JSON_SUMMARY say so; --strict-sizes does not count it')
        install(thick)
        line = fab_tiers.escalation_report_line()
        assert ('0.188 mm at 10.67:1 drill aspect ratio' in line and '2 mm thick board' in line
                and '0.25 mm at 8:1' in line), line
        a = fab_tiers.escalation_summary()['aspect_floor']
        assert a == {'board_thickness_mm': 2.0, 'rungs': [
            {'max_aspect_ratio': 8.0, 'via_drill': 0.25, 'table_via_drill': 0.2},
            {'max_aspect_ratio': 10.67, 'via_drill': 0.188, 'table_via_drill': 0.15}]}, a
        assert fab_tiers.narrowed_summary() is not None
        with redirect_stdout(io.StringIO()):
            assert fab_tiers.finish_design_rules(SimpleNamespace(strict_sizes=True)) == 0
        install(thin)
        assert fab_tiers.escalation_report_line() == ''
        assert 'aspect_floor' not in fab_tiers.escalation_summary()
        assert fab_tiers.narrowed_summary() is None
        print('   PASS')

        print('6. a smaller override drill raises nothing on a 1.6 mm board')
        install(thin, overrides={'via_drill': 0.1, 'via_diameter': 0.2})
        assert vias(fab_tiers.escalation_rungs(4)) == [(0.2, 0.1)]
        install(thick, overrides={'via_drill': 0.1, 'via_diameter': 0.2})
        assert vias(fab_tiers.escalation_rungs(4)) == [(0.225, 0.125)], \
            vias(fab_tiers.escalation_rungs(4))
        print('   PASS')
    finally:
        fab_tiers.set_default_fab_tier('auto', {})
        fab_tiers.set_escalation_policy('fab', {})
        fab_tiers.set_board_thickness(None)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
