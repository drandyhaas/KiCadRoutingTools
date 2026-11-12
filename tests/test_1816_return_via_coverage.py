#!/usr/bin/env python3
"""#1816: ground return-via coverage is measured, by ONE test.

The coverage test used to live only inside add_gnd_vias_to_existing_board,
which prints its counts and runs only while adding vias; the review skill
carried a second copy that already disagreed with it. `return_vias` is now
the one test: the step skips a covered via with it, `check_return_vias.py`
reports what it leaves uncovered, and board_score publishes
`advisory.return_vias` over the --impedance-nets nets, outside `blocking`.

Pinned here:
  1. a board with no ground return copper reports every signal via uncovered,
     and a ground via 1 mm away covers its signal via;
  2. a via of ANOTHER ground domain covers nothing (#489);
  3. a power-rail via is not a signal via: the check leaves it out and the
     step places no return via for it;
  4. the CLI writes the counts, and board_score's advisory count stays out of
     `blocking`;
  5. nothing left out reads as covered: a `--nets` pattern naming no net of
     the board is named and exits 2 instead of printing `0 of 0` alone, and a
     net left out as power is named with the reason, by the check and by
     board_score when `--impedance-nets` names it.
"""
import json
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, 'tests')
for _p in (TESTS, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools'),
           os.path.join(ROOT, 'rust_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import Zone, parse_kicad_pcb  # noqa: E402
from synth import make_net, make_pad, make_via  # noqa: E402
from return_vias import ReturnViaModel, return_via_coverage  # noqa: E402
import test_489_ground_domains as g489  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
POWER = 9


def t_no_return_copper_and_one_via_covers():
    pcb = parse_kicad_pcb(evidence(BOARD, 'fixture board'))
    rep = return_via_coverage(pcb, 2.25)
    assert rep['signal_vias'] == 383 and rep['uncovered'] == 383, \
        {k: rep[k] for k in ('signal_vias', 'uncovered', 'power_vias')}
    model = ReturnViaModel(pcb)
    target = model.signal_vias()[0]
    pcb.vias.append(make_via(target.x + 1.0, target.y, net_id=model.gnd_net_id,
                             size=0.45, drill=0.25, layers=['F.Cu', 'B.Cu']))
    rep2 = return_via_coverage(pcb, 2.25)
    assert rep2['uncovered'] < 383, rep2['uncovered']
    assert not any(math.isclose(u['x'], round(target.x, 4))
                   and math.isclose(u['y'], round(target.y, 4))
                   for u in rep2['uncovered_vias']), 'the covered via is still listed'
    print(f"  PASS: 383 of 383 uncovered; a GND via 1 mm away covers its via "
          f"({rep2['uncovered']} left)")


def _two_domain_board():
    b = g489._board()
    b.vias = [make_via(4.0, 5.0, net_id=g489.DSIG, size=0.45, drill=0.25,
                       layers=['F.Cu', 'B.Cu'])]
    return b


def t_other_domain_covers_nothing():
    b = _two_domain_board()
    b.vias.append(make_via(4.0, 6.0, net_id=g489.AGND, size=0.45, drill=0.25,
                           layers=['F.Cu', 'B.Cu']))
    rep = return_via_coverage(b, 2.0)
    assert rep['signal_vias'] == 1 and rep['uncovered'] == 1, rep
    b.vias.append(make_via(4.0, 6.0, net_id=g489.GND, size=0.45, drill=0.25,
                           layers=['F.Cu', 'B.Cu']))
    rep = return_via_coverage(b, 2.0)
    assert rep['uncovered'] == 0, rep
    print("  PASS: an AGND via does not cover a GND-domain signal via; a GND via does")


def _with_power_via(b):
    """A +3V3 rail with a via far from any ground copper."""
    b.nets[POWER] = make_net(POWER, '+3V3')
    p = make_pad(net_id=POWER, x=15, y=8, ref='U1', num='3')
    b.pads_by_net[POWER] = [p]
    b.vias.append(make_via(15.0, 5.0, net_id=POWER, size=0.45, drill=0.25,
                           layers=['F.Cu', 'B.Cu']))
    return b


def t_power_via_is_not_a_signal_via():
    b = _with_power_via(_two_domain_board())
    rep = return_via_coverage(b, 2.0)
    assert rep['signal_vias'] == 1 and rep['power_vias'] == 1, rep
    assert all(u['net'] != '+3V3' for u in rep['uncovered_vias']), rep
    assert rep.get('power_nets') == {'+3V3': {
        'vias': 1, 'why': 'its name reads as a supply rail'}}, rep.get('power_nets')
    # ... and only over the nets asked
    assert return_via_coverage(b, 2.0, net_ids={g489.DSIG}).get('power_nets') == {}

    # The step: with the power via as the only via, it places nothing.
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map
    from add_gnd_vias import add_gnd_vias_to_existing_board
    e2e = _with_power_via(g489._board())
    e2e.zones = [Zone(net_id=nid, net_name=name, layer=lay,
                      polygon=[(-5, -5), (35, -5), (35, 15), (-5, 15)])
                 for nid, name in ((g489.GND, 'GND'), (g489.AGND, 'AGND'))
                 for lay in ('F.Cu', 'B.Cu')]
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.1,
                          track_width=0.15, clearance=0.15, via_size=0.45,
                          via_drill=0.25, hole_to_hole_clearance=0.25)
    placed = add_gnd_vias_to_existing_board(
        e2e, '', 3.0, cfg, build_base_obstacle_map(e2e, cfg, []),
        GridCoord(cfg.grid_step))
    assert placed == [], f"a return via was placed for a power-rail via: {placed}"
    print("  PASS: a power-rail via is left out of the check, and the step places none for it")


def t_cli_and_board_score_advisory():
    with tempfile.TemporaryDirectory() as td:
        js = os.path.join(td, 'rv.json')
        check([sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'py_tools', 'check_return_vias.py'), BOARD,
               '--distance', '2.25', '--json', js], accept=True)
        with open(evidence(js, 'check_return_vias JSON'), encoding='utf-8') as f:
            rep = json.load(f)
        assert rep['signal_vias'] == 383 and rep['uncovered'] == 383, rep['uncovered']

        sj = os.path.join(td, 'score.json')
        nets = 'Net-(U1B-CA*'
        check([sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'py_tools', 'board_score.py'), BOARD,
               '--impedance-nets', nets, '--json', sj, '--quiet'], code=4)
        with open(evidence(sj, 'board_score JSON'), encoding='utf-8') as f:
            s = json.load(f)
    pcb = parse_kicad_pcb(BOARD)
    from net_queries import matches_net_filter
    want = {nid for nid, n in pcb.nets.items() if matches_net_filter(n.name, [nets])}
    expect = return_via_coverage(pcb, 2.0, net_ids=want)['uncovered']
    assert expect, 'the named nets have no signal vias: the fixture tests nothing'
    assert s['advisory'].get('return_vias') == expect, (s['advisory'], expect)
    assert 'return_vias' not in s['blocking_by'], s['blocking_by']
    assert s['blocking'] == sum(v for v in s['blocking_by'].values() if v), \
        (s['blocking'], s['blocking_by'])
    print(f"  PASS: CLI 383/383; board_score advisory.return_vias {expect}, "
          f"blocking {s['blocking']} without it")


#: A signal net of the fixture, with 3 vias, and the name it is given below:
#: one that reads as a supply rail by name alone (it holds "VIN").
SIGNAL, AS_RAIL = 'Net-(U2A-DATA_12)', '/VIN_AUX'


def t_nothing_left_out_reads_as_covered():
    tool = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_tools', 'check_return_vias.py')]
    score = [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_tools', 'board_score.py')]
    with tempfile.TemporaryDirectory() as td:
        # a pattern naming no net of the board
        js = os.path.join(td, 'none.json')
        check(tool + [BOARD, '--nets', 'NO_SUCH_NET', '--json', js],
              refuse="--nets pattern(s) matching no net of the board: 'NO_SUCH_NET'", code=2)
        with open(evidence(js, 'check_return_vias JSON'), encoding='utf-8') as f:
            assert json.load(f)['patterns_matching_no_net'] == ['NO_SUCH_NET']
        two = f'{SIGNAL}, Net-(U2A-DATA_22)'
        r = check(tool + [BOARD, '--nets', two],
                  refuse=f"({two!r} was read as one pattern holding whitespace",
                  code=2)
        # --nets does not split at commas, so the hint must not offer them
        assert 'commas' not in r.stdout, r.stdout[:400]
        r = check(tool + [BOARD, '--nets', SIGNAL, 'NO_SUCH_NET'],
                  refuse="--nets pattern(s) matching no net of the board: 'NO_SUCH_NET'", code=2)
        assert 'Return vias: 3 of 3 signal via(s)' in r.stdout, r.stdout[:400]
        r = check(tool + [BOARD, '--nets', SIGNAL, '--json', js], accept=True)
        assert 'matching no net' not in r.stdout and 'Left out as power' not in r.stdout, \
            r.stdout[:400]
        with open(evidence(js, 'check_return_vias JSON'), encoding='utf-8') as f:
            rep = json.load(f)
        assert rep['patterns_matching_no_net'] == [] and rep['power_nets'] == {} \
            and rep['signal_vias'] == 3, rep

        # the same board with that one net renamed: the name alone reads as a rail
        with open(evidence(BOARD, 'fixture board'), encoding='utf-8', newline='') as f:
            text = f.read()
        assert text.count(f'"{SIGNAL}"') == 3 and f'"{AS_RAIL}"' not in text, \
            'the fixture changed'
        b = os.path.join(td, 'b.kicad_pcb')
        with open(b, 'w', encoding='utf-8', newline='') as f:
            f.write(text.replace(f'"{SIGNAL}"', f'"{AS_RAIL}"'))
        why = 'its name reads as a supply rail'
        r = check(tool + [evidence(b, 'renamed board'), '--nets', AS_RAIL,
                          '--json', js], accept=True)
        assert 'Return vias: 0 of 0 signal via(s)' in r.stdout \
            and '(3 power-rail via(s) left out)' in r.stdout, r.stdout[:400]
        assert f'  {AS_RAIL}: 3 via(s) -- {why}' in r.stdout, r.stdout[:600]
        with open(evidence(js, 'check_return_vias JSON'), encoding='utf-8') as f:
            rep = json.load(f)
        assert rep['power_nets'] == {AS_RAIL: {'vias': 3, 'why': why}} \
            and rep['power_vias'] == 3 and rep['signal_vias'] == 0, rep

        # board_score: the named net's 0 is not printed as nothing to see
        sj = os.path.join(td, 'score.json')
        r = check(score + [b, '--impedance-nets', AS_RAIL, '--json', sj, '--quiet'],
                  refuse=f"3 via(s) on --impedance-nets net(s) were left out as "
                         f"power, not examined: {AS_RAIL} ({why})", code=4)
        with open(evidence(sj, 'board_score JSON'), encoding='utf-8') as f:
            s = json.load(f)
        assert s['advisory'].get('return_vias') == 0, s['advisory']
        assert s['components']['return_vias']['power_nets'] == {
            AS_RAIL: {'vias': 3, 'why': why}}, s['components']['return_vias']
        # ... and the count is what it was: 380 signal vias, all uncovered
        full = return_via_coverage(parse_kicad_pcb(b), 2.0)
        assert (full['signal_vias'], full['uncovered'], full['power_vias']) == \
            (380, 380, 3), full
    print("  PASS: a pattern naming no net exits 2 and is named; a net left out "
          "as power is named with its reason, by the check and by board_score")


if __name__ == '__main__':
    t_no_return_copper_and_one_via_covers()
    t_other_domain_covers_nothing()
    t_power_via_is_not_a_signal_via()
    t_cli_and_board_score_advisory()
    t_nothing_left_out_reads_as_covered()
