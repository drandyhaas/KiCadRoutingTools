"""#1146: a soft pairwise keep-away between two net groups.

`--keep-away AGG:VICTIM:GAP` prices, while a net of one side routes, every
grid cell where its track would sit closer than GAP (edge to edge, same
layer) to copper of the other side -- except within `--keep-away-free` of the
routed net's own pads. Nets of one side route against each other at the
normal clearance. The run reports per net the track length left inside a
band (JSON_SUMMARY keep_away).

Rows:
  - rule parsing (a GAP above 10 mm, a name split by a space), and route.py
    refusing a malformed rule, or a free radius / cost outside the GUI's
    range, for that reason;
  - the band rows a net is priced with: opposite copper only, the band's
    edge, the free radius, and nothing at cost 0 or for a net in no rule;
  - the resolution log: one count line per rule however many names a side
    lists exactly (a space written '?' makes each a wildcard), a pattern that
    matched nothing warned about once however many rules list it, and
    nothing again for another grid on the board or a quiet re-read;
  - net-class sides (`class=NAME`, `!class=NAME`) resolved from the project,
    alone and mixed with net patterns, whose name terms read as --nets reads
    them (an active-low `!NAME` net) and which never pull in unconnected-*;
  - the rescue paths' stamp: the same rows, nothing without a rule, and a
    rung's window prices only the band that reaches it, caching nothing;
  - the band follows the copper: gone with a ripped track (a pad's stays),
    back on restore, moved by an in-place edit that bumps no epoch, and by a
    pad that moves while the copper does not;
  - the report on two parallel tracks of known spacing: in-band length, the
    free radius, a narrower GAP and another layer; and its closest spacing is
    the closest copper, not the deepest band;
  - route_diff takes the same rules (lvds_converter_dualclk's clock pair kept
    off its data pair);
  - end to end on splitflap_driver: routing with the cost leaves less length
    inside a 2 mm band than routing without it, every net still routed, both
    boards graded by the same report, which --json-out carries as measured on
    the written board; and re-running on the routed board at cost 0, with
    nothing left to route, still reports it.

    python3 tests/test_1146_keep_away.py
"""

import contextlib
import io
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import check, evidence  # noqa: E402

import keep_away as ka  # noqa: E402
from kicad_parser import (BoardInfo, Net, Pad, PCBData, Segment,  # noqa: E402
                          parse_kicad_pcb)
from routing_config import GridRouteConfig  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _config(rules, free=1.0, cost=0.5):
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.2,
                           keep_away=ka.normalize_keep_away_specs(rules),
                           keep_away_free=free, keep_away_cost=cost)


def _board(v_layer='F.Cu'):
    """A: a track (0,0)-(10,0) with a round pad at its start; A2: a pad far
    away; V: a track (3,0.6)-(13,0.6). Widths 0.2, so A and V run 0.4 mm
    apart edge to edge from x=3 to x=10."""
    nets = {1: Net(1, 'A'), 2: Net(2, 'V'), 3: Net(3, 'A2'), 4: Net(4, 'X')}
    pad_a = Pad('R1', '1', 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 'circle', ['F.Cu'],
                1, 'A', pad_type='smd')
    pad_a2 = Pad('R2', '1', 0.0, -5.0, 0.0, 0.0, 0.5, 0.5, 'circle', ['F.Cu'],
                 3, 'A2', pad_type='smd')
    segs = [Segment(0.0, 0.0, 10.0, 0.0, 0.2, 'F.Cu', 1),
            Segment(3.0, 0.6, 13.0, 0.6, 0.2, v_layer, 2)]
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    return PCBData(board_info=bi, nets=nets, footprints={}, vias=[],
                   segments=segs, pads_by_net={1: [pad_a], 3: [pad_a2]})


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def test_parse():
    rules = ka.parse_keep_away_rules(['CLK*,/I2C_*:/AUDIO_*:0.5',
                                      '/RELAY_*:/AUDIO_*:.3; X:Y:1'])
    assert [r.spec for r in rules] == ['CLK*,/I2C_*:/AUDIO_*:0.5',
                                       '/RELAY_*:/AUDIO_*:0.3', 'X:Y:1'], rules
    assert rules[0].aggressor == ('CLK*', '/I2C_*') and rules[0].gap == 0.5
    for bad, why in (('A:B', 'expected AGGRESSOR:VICTIM:GAP'),
                     ('A:B:C:0.5', 'expected AGGRESSOR:VICTIM:GAP'),
                     ('A:B:wide', 'is not a number'),
                     ('A:B:0', 'GAP must be > 0'),
                     ('A:B:50', 'GAP must be > 0 and <= 10 mm'),
                     ('class=High Speed:B:0.5', "write a space inside a net or "
                                                "class name as '?'"),
                     (':B:0.5', 'both net groups')):
        try:
            ka.parse_keep_away_rules([bad])
        except ValueError as e:
            assert why in str(e), (bad, str(e))
        else:
            raise AssertionError(f"{bad!r} was accepted")
    assert ka.parse_keep_away_rules(None) == ()
    assert ka.normalize_keep_away_specs(['A:B:0.123456789']) == ('A:B:0.123456789',)


def test_route_refuses_a_bad_rule():
    with tempfile.TemporaryDirectory() as td:
        check([sys.executable, '-X', 'utf8', 'py_router/route.py', BOARD,
               os.path.join(td, 'o.kicad_pcb'), '--keep-away', 'A:B'],
              refuse="keep-away rule 'A:B': expected AGGRESSOR:VICTIM:GAP",
              code=2)
        # The GUI's spin control stops at 5; a plan must not carry more.
        check([sys.executable, '-X', 'utf8', 'py_router/route.py', BOARD,
               os.path.join(td, 'o.kicad_pcb'), '--keep-away', 'A:B:1',
               '--keep-away-cost', '9'],
              refuse='--keep-away-cost 9 is outside 0..5', code=2)


def _cells(rows):
    return {(int(r[0]), int(r[1]), int(r[2])) for r in rows}


def test_band_rows():
    pcb = _board()
    cfg = _config(['A,A2:V:0.5'], free=1.0, cost=0.5)
    rows = _quiet(ka.keep_away_rows, cfg, pcb, 1)
    assert rows is not None and len(rows), "A gets no band"
    assert set(rows[:, 3].tolist()) == {cfg.cell_cost(0.5)}, "cost per cell"
    cells = _cells(rows)
    assert {c[0] for c in cells} == {0}, "band left V's layer"
    # A's centreline 0.6 below V's: inside 0.1 + 0.5 + 0.1 of it; 0.8 is not.
    assert (0, 80, 0) in cells and (0, 80, -2) not in cells
    # The free radius (pad radius 0.25 + 1.0) lifts the band at A's pad only.
    assert (0, 40, 0) in cells
    rows4 = _quiet(ka.keep_away_rows, _config(['A,A2:V:0.5'], free=4.0), pcb, 1)
    assert (0, 40, 0) not in _cells(rows4) and (0, 80, 0) in _cells(rows4)
    # V is priced against both aggressors' copper, A's track and A2's pad.
    v = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 2))
    assert (0, 10, -2) in v and (0, 0, -55) in v
    # A2 is on A's side: A's copper is not a band for it, V's is.
    a2 = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 3))
    assert (0, 10, -3) not in a2 and (0, 80, 0) in a2
    assert _quiet(ka.keep_away_rows, cfg, pcb, 4) is None, "net in no rule"
    assert _quiet(ka.keep_away_rows, _config(['A:V:0.5'], cost=0), pcb, 1) is None


def _log(fn, *a, **k):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fn(*a, **k)
    return buf.getvalue().splitlines()


def test_resolution_log():
    from net_queries import expand_net_patterns
    rules = ['A,A?:V,N?PE:0.5', 'X:V,N?PE:0.3']
    pcb = _board()
    assert _log(ka._state, _config(rules), pcb) == [
        "Keep-away rule 1 'A,A?:V,N?PE:0.5': 2 net(s) vs 1 net(s)",
        "Keep-away rule 2 'X:V,N?PE:0.3': 1 net(s) vs 1 net(s)",
        "WARNING: keep-away: Pattern 'N?PE' matched no nets"]
    cfg = _config(rules)
    cfg.grid_step = 0.05
    assert _log(ka._state, cfg, pcb) == []
    assert _log(ka.disclose_keep_away, _board(), _config(rules), quiet=True) == []
    long_side = ','.join(['A?'] * 30)
    line = _log(ka._state, _config([f'{long_side}:V:1']), _board())[0]
    assert line == f"Keep-away rule 1 '{long_side[:45]}...:1': 1 net(s) vs 1 net(s)", line
    # --nets keeps its counts; only a caller asking for quiet loses them.
    assert _log(expand_net_patterns, pcb, ['A?']) == ["Pattern 'A?' matched 1 nets"]
    assert _log(expand_net_patterns, pcb, ['A?', 'N?PE'], quiet=True) == [
        "Warning: Pattern 'N?PE' matched no nets"]


def test_class_terms():
    """A side may name net CLASSES (`class=NAME`, a glob; `!class=NAME` takes
    one out), resolved from the board's project the way the router and
    check_drc resolve them -- an assignment, a netclass pattern, and Default
    for a net no class claims."""
    with tempfile.TemporaryDirectory() as td:
        pcb = _board()
        pcb.source_path = os.path.join(td, 'b.kicad_pcb')
        with open(os.path.join(td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({'net_settings': {
                'classes': [{'name': n, 'clearance': 0.2}
                            for n in ('Default', 'Audio', 'Digital')],
                'netclass_assignments': {'A': 'Digital'},
                'netclass_patterns': [{'netclass': 'Audio', 'pattern': 'V*'}]}}, fh)

        def sides(rule):
            st = _quiet(ka._state, _config([rule]), pcb)
            a, v, _g = st.groups[0]
            name = {i: pcb.nets[i].name for i in pcb.nets}
            return sorted(name[i] for i in a), sorted(name[i] for i in v), st
        assert sides('class=Digital:class=Audio:0.5')[:2] == (['A'], ['V'])
        assert sides('class=Default:class=Aud*:0.5')[:2] == (['A2', 'X'], ['V'])
        assert sides('!class=Audio:class=Audio:0.5')[:2] == (['A', 'A2', 'X'], ['V'])
        assert sides('class=Digital,A2:V:0.5')[:2] == (['A', 'A2'], ['V'])
        assert sides('A*,!class=Digital:V:0.5')[:2] == (['A2'], ['V'])
        _a, _v, st = sides('class=Nope:V:0.5')
        assert st.unmatched and "class 'Nope' has no nets" in st.notes[0], st.notes
        # Name terms read as --nets reads them: '!RESET' naming a real
        # active-low net is that net (#177), here as in a plain rule; and a
        # class, like '*', never brings in an unconnected-* net.
        pcb.nets[5] = Net(5, '!RESET')
        pcb.nets[6] = Net(6, 'unconnected-(U1-Pad3)')
        pcb._keep_away_state = {}       # new nets: what a new run starts with
        assert sides('!RESET:V:0.5')[0] == ['!RESET']
        assert sides('class=Digital,!RESET:V:0.5')[0] == ['!RESET', 'A']
        assert sides('class=Default:V:0.5')[0] == ['!RESET', 'A2', 'X']
        assert sides('!class=Audio:class=Audio:0.5')[0] == ['!RESET', 'A', 'A2', 'X']
        assert sides('class=Default,!X:V:0.5')[0] == ['!RESET', 'A2']
        # The band is the one a pattern rule naming the same nets builds.
        assert (_cells(_quiet(ka.keep_away_rows, _config(['class=Digital:class=Audio:0.5']),
                              pcb, 1))
                == _cells(_quiet(ka.keep_away_rows, _config(['A:V:0.5']), pcb, 1)))
    try:
        ka.parse_keep_away_rules(['class=:V:0.5'])
    except ValueError as e:
        assert 'needs a net-class name' in str(e)
    else:
        raise AssertionError("an empty class= was accepted")


def test_stamp_on_rescue_maps():
    """net_rescue's gap rescue and terminal escalation route on clones no
    proximity builder prepared; they stamp the band themselves, and stamp
    nothing at all without a rule."""
    class Map:
        def __init__(self):
            self.calls = []

        def set_layer_proximity_batch(self, rows):
            self.calls.append(rows)
    pcb = _board()
    m = Map()
    assert not ka.stamp_keep_away(m, GridRouteConfig(layers=['F.Cu', 'B.Cu']),
                                  pcb, 1) and m.calls == []
    cfg = _config(['A:V:0.5'])
    assert ka.stamp_keep_away(m, cfg, pcb, 1) and len(m.calls) == 1
    full = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    assert _cells(m.calls[0]) == full
    # A rung routing inside a window prices only the band that reaches it:
    # the same cells there, none of V's far pad, and nothing cached (a
    # rung's grid is fine enough that the whole board's band costs minutes).
    pcb.pads_by_net[2] = [Pad('R3', '1', 20.0, 0.0, 0.0, 0.0, 0.5, 0.5, 'circle',
                              ['F.Cu'], 2, 'V', pad_type='smd')]
    full = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    st = ka._state(cfg, pcb)
    cached = (len(st._fields), len(st._composites))
    win = (6.0, -1.0, 9.0, 1.0)
    assert ka.stamp_keep_away(m, cfg, pcb, 1, window=win)
    got = _cells(m.calls[-1])

    def inside(c):
        return 60 <= c[1] <= 90 and -10 <= c[2] <= 10
    assert {c for c in got if inside(c)} == {c for c in full if inside(c)}
    assert (0, 200, 0) in full and (0, 200, 0) not in got, "far copper rasterised"
    assert (len(st._fields), len(st._composites)) == cached, "window cached"


def test_band_follows_the_copper():
    """The band is rebuilt from the copper as it is at each prepare: a ripped
    victim takes its band with it and a restored one brings the same band
    back, and an in-place edit that bumps no copper epoch and changes no list
    length (a coordinate nudged, a list replaced) is seen too."""
    from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
    pcb = _board()
    # V needs pads, or the restore's dead-end sweep drops its track.
    pcb.pads_by_net[2] = [Pad('R3', str(i), x, 0.6, 0.0, 0.0, 0.5, 0.5, 'circle',
                              ['F.Cu'], 2, 'V', pad_type='smd')
                          for i, x in ((1, 3.0), (2, 13.0))]
    cfg = _config(['A:V:0.5'])
    before = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    # (8, 0) is in the band of V's track; (3, 0) in that of V's pad.
    assert (0, 80, 0) in before and (0, 30, 0) in before
    v_seg = pcb.segments[1]
    _quiet(remove_route_from_pcb_data, pcb, {'new_segments': [v_seg], 'new_vias': []})
    ripped = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    assert (0, 80, 0) not in ripped, "a ripped track is still priced"
    assert (0, 30, 0) in ripped, "the pads were ripped with the track"
    _quiet(add_route_to_pcb_data, pcb, {'new_segments': [v_seg], 'new_vias': []})
    assert _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1)) == before, "restore"
    epoch = getattr(pcb, '_copper_epoch', 0)
    v_seg.start_y = v_seg.end_y = 1.6              # nudged 1 mm, in place
    assert getattr(pcb, '_copper_epoch', 0) == epoch
    moved = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    assert (0, 80, 0) not in moved and (0, 80, 10) in moved, "stale band"
    pcb.segments = [pcb.segments[0]]               # the GUI's sync replaces lists
    assert (0, 80, 10) not in _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    # A pad that moves while no copper does (a polarity swap re-nets pads).
    far = Pad('R4', '1', 20.0, 0.0, 0.0, 0.0, 0.5, 0.5, 'circle', ['F.Cu'], 2, 'V',
              pad_type='smd')
    pcb.pads_by_net[2].append(far)
    assert (0, 200, 0) in _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    far.global_y = 5.0
    moved = _cells(_quiet(ka.keep_away_rows, cfg, pcb, 1))
    assert (0, 200, 0) not in moved and (0, 200, 50) in moved, "stale pad band"


def test_report():
    rep = _quiet(ka.keep_away_report, _board(), _config(['A:V:0.5'], free=1.0))
    a, v = rep['nets']['A'], rep['nets']['V']
    # In band while within 0.5 of the other track: x in (2.64, 10.36) -- the
    # sqrt(0.49 - 0.36) reach past each track's end.
    assert abs(a['in_band_mm'] - 7.36) < 0.1, a
    assert abs(v['in_band_mm'] - 7.36) < 0.1, v
    assert abs(a['min_spacing_mm'] - 0.4) < 1e-3 and a['closest_net'] == 'V', a
    assert rep['nets_checked'] == 2 and rep['nets_in_band'] == 2, rep
    # A free radius reaching x=5.25 takes A's stretch before it off the books.
    a5 = _quiet(ka.keep_away_report, _board(), _config(['A:V:0.5'], free=5.0))
    assert abs(a5['nets']['A']['in_band_mm'] - 4.75) < 0.1, a5['nets']['A']
    assert _quiet(ka.keep_away_report, _board(),
                  _config(['A:V:0.3']))['nets_in_band'] == 0, "0.4 apart is outside 0.3"
    assert _quiet(ka.keep_away_report, _board('B.Cu'),
                  _config(['A:V:0.5']))['nets_in_band'] == 0, "other layer"
    json.dumps(rep)
    # The closest spacing is the closest copper: A runs 0.8 mm from X under a
    # 2 mm rule (deep in that band) and 0.3 mm from V under a 0.35 mm rule.
    pcb = _board()
    pcb.segments = [Segment(0.0, 0.0, 10.0, 0.0, 0.2, 'F.Cu', 1),
                    Segment(0.0, 1.0, 10.0, 1.0, 0.2, 'F.Cu', 4),
                    Segment(0.0, -0.5, 10.0, -0.5, 0.2, 'F.Cu', 2)]
    pcb.pads_by_net = {}
    a = _quiet(ka.keep_away_report, pcb,
               _config(['A:X:2', 'A:V:0.35'], free=0))['nets']['A']
    assert a['closest_net'] == 'V' and abs(a['min_spacing_mm'] - 0.3) < 1e-3, a
    assert a['gap_mm'] == 0.35, a


def _summary(log):
    return json.loads(re.findall(r'^JSON_SUMMARY: (.*)$', log, re.M)[-1])


def test_cost_steers_the_route():
    rule = '/MOTOR_*,/OUT_*:/SENSOR_*:2'
    cfg = _config([rule], free=1.5)
    graded = {}
    env = dict(os.environ, MSYS2_ARG_CONV_EXCL='*')
    with tempfile.TemporaryDirectory() as td:
        for arm, extra in (('off', []),
                           ('on', ['--keep-away', rule, '--keep-away-cost', '1'])):
            out = os.path.join(td, f'{arm}.kicad_pcb')
            jout = os.path.join(td, f'{arm}.json')
            r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py',
                                evidence(BOARD), out, '--nets', '/*',
                                '--json-out', jout] + extra,
                               cwd=ROOT, capture_output=True, text=True,
                               encoding='utf-8', errors='replace', env=env,
                               timeout=900)
            assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
            s = _summary(r.stdout)
            assert s['failed'] == 0 and s['successful'] == 71, (arm, s['failed'])
            graded[arm] = _quiet(ka.keep_away_report,
                                 parse_kicad_pcb(evidence(out)), cfg)
            with open(evidence(jout), encoding='utf-8') as fh:
                doc = json.load(fh)
            if arm == 'on':
                assert abs(s['keep_away']['in_band_mm']
                           - graded[arm]['in_band_mm']) < 0.5, s['keep_away']
                # --json-out carries the reading of the board this run wrote.
                assert doc['keep_away']['measured_on'] == 'written board', doc
                assert abs(doc['keep_away']['in_band_mm']
                           - graded[arm]['in_band_mm']) < 0.01, doc['keep_away']
            else:
                assert 'keep_away' not in s and 'keep_away' not in doc
        # Grading the routed board at cost 0: nothing is left to route, and
        # the report is still made, on the board as it stands.
        jout = os.path.join(td, 'grade.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py',
                            evidence(out), os.path.join(td, 'grade.kicad_pcb'),
                            '--nets', '/*', '--json-out', jout,
                            '--keep-away', rule, '--keep-away-cost', '0'],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', env=env, timeout=900)
        assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
        assert 'All nets are already' in r.stdout, r.stdout[-2000:]
        with open(evidence(jout), encoding='utf-8') as fh:
            doc = json.load(fh)
        assert abs(doc['keep_away']['in_band_mm']
                   - graded['on']['in_band_mm']) < 0.01, doc.get('keep_away')
    off, on = graded['off']['in_band_mm'], graded['on']['in_band_mm']
    print(f"    in band: {off} mm without the cost, {on} mm with it")
    assert off > 5 and on < off / 2, (off, on)


def test_diff_pairs_keep_away():
    """route_diff takes the same rules: on lvds_converter_dualclk the clock
    pair runs 3.3 mm inside a 1 mm band of the data pair without them and
    clear of it with them, both pairs still routed."""
    board = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
    rule = '/CLK+,/CLK-:/DATA+,/DATA-:1'
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'],
                          keep_away=ka.normalize_keep_away_specs([rule]),
                          keep_away_free=1.5)
    in_band = {}
    env = dict(os.environ, MSYS2_ARG_CONV_EXCL='*')
    with tempfile.TemporaryDirectory() as td:
        for arm, extra in (('off', []),
                           ('on', ['--keep-away', rule, '--keep-away-cost', '1'])):
            out = os.path.join(td, f'{arm}.kicad_pcb')
            r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route_diff.py',
                                evidence(board), out, '--nets', '/CLK+', '/CLK-',
                                '/DATA+', '/DATA-'] + extra,
                               cwd=ROOT, capture_output=True, text=True,
                               encoding='utf-8', errors='replace', env=env, timeout=900)
            assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
            s = _summary(r.stdout)
            assert s['successful'] == 2 and s['failed'] == 0, (arm, s['failed'])
            assert ('keep_away' in s) == (arm == 'on'), arm
            in_band[arm] = _quiet(ka.keep_away_report,
                                  parse_kicad_pcb(evidence(out)), cfg)['in_band_mm']
    assert in_band['off'] > 1 and in_band['on'] < in_band['off'] / 2, in_band


TESTS = [test_parse, test_route_refuses_a_bad_rule, test_band_rows,
         test_resolution_log, test_class_terms, test_stamp_on_rescue_maps, test_band_follows_the_copper,
         test_report, test_diff_pairs_keep_away, test_cost_steers_the_route]


if __name__ == '__main__':
    fails = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
