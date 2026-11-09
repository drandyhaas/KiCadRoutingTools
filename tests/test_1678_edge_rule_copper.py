#!/usr/bin/env python3
"""#1678: a `.kicad_dru` `edge_clearance` rule holds the tracks and vias a
routing step lays, and check_drc grades them, as KiCad reads the rule.

The rule bound pads (#1615) and zone fills (#1643) and nothing else: a run laid
copper inside it and check_drc said nothing. KiCad's own reading was measured
on kicad-cli 10.0.6 with one rule at a time over a 4-layer board carrying tracks
on three layers and vias at a 0.3 mm gap (`CASES`):

  - `(layer ...)` excludes nothing: `(layer inner)` and `(layer B.Cu)` flag an
    F.Cu track, an F.Cu pad and a through via alike. Only the zone gap reads it.
  - a condition is read for the item: Type, NetName, NetClass, `A.Layer` (a
    track's own layer, and no via), `A.existsOnLayer` (a through via is on all).
  - `A.Type != 'Via'` binds a via too: the other item is the Edge.Cuts shape,
    which satisfies it, and KiCad tries the condition both ways.
  - a rule replaces the Board Setup minimum, a lower one too.

  1. `PadEdgeRules.for_copper` answers each row; a condition it cannot read is
     a range.
  2. `EdgeFloors`: the strictest floor for scalar readers, the lowest per kind
     for the shared band, the fab floor under a relaxing rule.
  3. the install: the scalar moves, the declared value stays for the writeback,
     and a rule that moves nothing leaves the config untouched.
  4. the map's band follows the kind's floor, and a net above it gets the rest
     only while it routes (`net_keepouts.stamp`), which removes what it added.
  5. check_drc flags each row where KiCad does; with kicad-cli present the two
     are compared row by row.
  6. a routed board keeps the rule, and the end-of-step line and JSON name it.

    python3 tests/test_1678_edge_rule_copper.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)

from run_utils import evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from design_rules import DesignRules, EdgeFloors, PadEdgeRules, edge_rule_grade  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-u', '-X', 'utf8']
SMALL_BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
GAP = 0.3

#: (item key, kind, layer, net id, x so the copper's gap is GAP)
ITEMS = [('track F A', 'track', 'F.Cu', 1), ('track In1 A', 'track', 'In1.Cu', 1),
         ('track In2 B', 'track', 'In2.Cu', 2), ('track B B', 'track', 'B.Cu', 2),
         ('via A', 'via', None, 1), ('via B', 'via', None, 2)]
TRACKS = {'track F A', 'track In1 A', 'track In2 B', 'track B B'}
ALL = TRACKS | {'via A', 'via B'}

R = '(rule e {} (constraint edge_clearance (min 0.4mm)))'
#: (row, rule file, Board Setup edge minimum, what KiCad flags at a 0.3 mm gap)
CASES = [
    ('no rule', '', 0.0, set()),
    ('plain', R.format(''), 0.0, ALL),
    ('layer inner', R.format('(layer inner)'), 0.0, ALL),
    ('layer B.Cu', R.format('(layer B.Cu)'), 0.0, ALL),
    ('Type Track', R.format('''(condition "A.Type == 'Track'")'''), 0.0, TRACKS),
    ('Type Via', R.format('''(condition "A.Type == 'Via'")'''), 0.0, {'via A', 'via B'}),
    ('not Via', R.format('''(condition "A.Type != 'Via'")'''), 0.0, ALL),
    ('NetName A', R.format('''(condition "A.NetName == 'A'")'''), 0.0,
     {'track F A', 'track In1 A', 'via A'}),
    ('NetClass HS', R.format('''(condition "A.NetClass == 'HS'")'''), 0.0,
     {'track In2 B', 'track B B', 'via B'}),
    ('A.Layer In1', R.format('''(condition "A.Layer == 'In1.Cu'")'''), 0.0, {'track In1 A'}),
    ('existsOnLayer In1', R.format('''(condition "A.existsOnLayer('In1.Cu')")'''), 0.0,
     {'track In1 A', 'via A', 'via B'}),
    ('relax plain', '(rule e (constraint edge_clearance (min 0.2mm)))', 0.5, set()),
    ('relax inner', '(rule e (layer inner) (constraint edge_clearance (min 0.2mm)))', 0.5, set()),
    ('relax via', '''(rule e (condition "A.Type == 'Via'")
       (constraint edge_clearance (min 0.2mm)))''', 0.5, TRACKS),
    ('last wins', '(rule a (constraint edge_clearance (min 0.4mm)))\n'
                  '(rule b (layer inner) (constraint edge_clearance (min 0.2mm)))', 0.0, set()),
]


def write_board(path, dru, edge_min):
    """Each item alone, 3 mm apart along x = GAP, its copper GAP from the edge."""
    items, y, ys = [], 2, {}
    for n, (key, kind, layer, net) in enumerate(ITEMS):
        uuid = f'00000000-0000-0000-0000-{n + 1:012d}'
        if kind == 'track':
            items.append(f'(segment (start {GAP + 0.1} {y}) (end {GAP + 0.1} {y + 1}) (width 0.2) '
                         f'(layer "{layer}") (net {net}) (uuid "{uuid}"))')
        else:
            items.append(f'(via (at {GAP + 0.25} {y}) (size 0.5) (drill 0.3) '
                         f'(layers "F.Cu" "B.Cu") (net {net}) (uuid "{uuid}"))')
        ys[y] = key
        y += 3
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20240108) (generator "t") (generator_version "8.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal)
          (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "A") (net 2 "B")
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.05) (type default)) (fill none)
           (layer "Edge.Cuts"))
  {" ".join(items)}
)
''')
    base = os.path.splitext(path)[0]
    dflt = {'bus_width': 12, 'clearance': 0.2, 'diff_pair_gap': 0.25, 'diff_pair_via_gap': 0.25,
            'diff_pair_width': 0.2, 'line_style': 0, 'microvia_diameter': 0.3,
            'microvia_drill': 0.2, 'name': 'Default', 'pcb_color': 'rgba(0, 0, 0, 0.000)',
            'priority': 2147483647, 'schematic_color': 'rgba(0, 0, 0, 0.000)',
            'track_width': 0.2, 'via_diameter': 0.6, 'via_drill': 0.3, 'wire_width': 6}
    proj = {'board': {'design_settings': {
                'rules': {'min_copper_edge_clearance': edge_min, 'min_clearance': 0.0,
                          'min_track_width': 0.0, 'min_via_diameter': 0.0,
                          'min_via_annular_width': 0.0, 'min_through_hole_diameter': 0.0,
                          'min_hole_to_hole': 0.0, 'min_hole_clearance': 0.0,
                          'min_connection': 0.0},
                'rule_severities': {'track_dangling': 'ignore', 'via_dangling': 'ignore',
                                    'unconnected_items': 'ignore'}}},
            'meta': {'filename': os.path.basename(base) + '.kicad_pro', 'version': 1},
            'net_settings': {'classes': [dflt, dict(dflt, name='HS', priority=0)],
                             'meta': {'version': 3},
                             'netclass_patterns': [{'pattern': 'B', 'netclass': 'HS'}]}}
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump(proj, f)
    if dru:
        with open(base + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write('(version 1)\n' + dru + '\n')
    return ys


def expect_floor(flagged, key, row):
    assert (row[0] > GAP) == (key in flagged), (key, row, flagged)


def main():
    evidence(SMALL_BOARD, 'small routed-board fixture')
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli()
    with tempfile.TemporaryDirectory(prefix='krt1678_') as td:
        print('1. for_copper reads each row as KiCad does')
        for name, dru, edge_min, flagged in CASES:
            d = os.path.join(td, name.replace(' ', '_').replace('.', '_'))
            os.makedirs(d)
            path = os.path.join(d, 'b.kicad_pcb')
            write_board(path, dru, edge_min)
            pcb = parse_kicad_pcb(path)
            edge = PadEdgeRules(pcb, path)
            base = edge_min or 0.0
            for key, kind, layer, net in ITEMS:
                expect_floor(flagged, key, edge.for_copper(net, kind, base, layer) + (name,))
        print('   PASS: every row')

        print('   a condition it cannot read is a range for copper, not a part')
        d = os.path.join(td, 'court')
        os.makedirs(d)
        path = os.path.join(d, 'b.kicad_pcb')
        write_board(path, '''(rule c (condition "A.insideCourtyard('J1')")
          (constraint edge_clearance (min 0mm)))''', 0.4)
        pcb = parse_kicad_pcb(path)
        assert PadEdgeRules(pcb, path).for_copper(1, 'track', 0.4, 'F.Cu') == (0.0, 0.4, ''), \
            'a track may lie in the courtyard, and only then does the rule bind it'
        print('   PASS')

        print('2-3. EdgeFloors and the install')
        from routing_config import GridRouteConfig
        from design_rules import install_design_rules

        def installed(name, dru, edge_min, scalar):
            d = os.path.join(td, 'i_' + name)
            os.makedirs(d)
            path = os.path.join(d, 'b.kicad_pcb')
            write_board(path, dru, edge_min)
            pcb = parse_kicad_pcb(path)
            cfg = GridRouteConfig(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                                  board_edge_clearance=scalar)
            install_design_rules(cfg, path, pcb, net_keepouts=True)
            return cfg, pcb
        cfg, _ = installed('plain', R.format(''), 0.0, 0.2)
        assert cfg.board_edge_clearance == 0.4 and cfg.board_edge_declared == 0.2
        assert cfg.edge_floors.track_band == cfg.edge_floors.via_band == 0.4
        assert not cfg.rules.net_keepouts, 'a uniform rule needs no per-net stamp'
        honoured = dict(cfg.rules.honoured_rules())
        assert 'held to 0.4 mm' in honoured['e'], honoured
        cfg, _ = installed('relax', '(rule e (constraint edge_clearance (min 0.05mm)))', 0.5, 0.5)
        assert cfg.board_edge_clearance == 0.2, 'only the fab floor pins a relaxing rule up'
        cfg, _ = installed('type', R.format('''(condition "A.Type == 'Track'")'''), 0.0, 0.2)
        assert (cfg.edge_floors.track_band, cfg.edge_floors.via_band) == (0.4, 0.2)
        assert cfg.board_edge_clearance == 0.4, 'the strictest, for one-number readers'
        cfg, _ = installed('none', '''(rule e (condition "A.Type == 'Pad'")
           (constraint edge_clearance (min 0.4mm)))''', 0.0, 0.2)
        assert cfg.edge_floors is None and cfg.board_edge_clearance == 0.2
        assert cfg.board_edge_declared is None, 'a rule that binds no copper moves nothing'
        print('   PASS')

        print('4. the band, and a net above it')
        from net_keepouts import attach, stamp
        from obstacle_map import add_board_edge_obstacles
        import numpy as np

        class Rec:
            """Every cell a stamp adds and removes, by map call."""
            def __init__(self):
                self.track, self.via = [], []

            def add_static_blocked_cells_batch(self, a):
                self.track.append(np.array(a))

            def add_static_blocked_vias_batch(self, a):
                self.via.append(np.array(a))

            add_blocked_cells_batch = add_static_blocked_cells_batch
            add_blocked_vias_batch = add_static_blocked_vias_batch

            def remove_blocked_cells_batch(self, a):
                self.track = [x for x in self.track if not np.array_equal(x, a)]

            def remove_blocked_vias_batch(self, a):
                self.via = [x for x in self.via if not np.array_equal(x, a)]

            def reach(self):
                """How far in from x=0, in mm at 0.05 mm per cell, tracks reach."""
                c = np.concatenate(self.track) if self.track else np.empty((0, 3))
                c = c[(c[:, 1] > 100) & (c[:, 1] < 700) & (c[:, 0] < 400)]
                return (c[:, 0].max() + 1) * 0.05 if len(c) else 0.0

        cfg, pcb = installed('net', R.format('''(condition "A.NetName == 'A'")'''), 0.0, 0.2)
        cfg.grid_step = 0.05
        assert cfg.edge_floors.track_band == 0.2 and cfg.board_edge_clearance == 0.4
        assert [sorted(k['nets']) for k in cfg.rules.net_keepouts] == [['A']]
        rec = Rec()
        add_board_edge_obstacles(rec, pcb, cfg)
        band = rec.reach()
        assert band < cfg.edge_floors.track_band + cfg.track_width / 2 + 0.1, band
        attach(pcb, cfg.rules.net_keepouts)
        netmap = Rec()
        with stamp(netmap, pcb, (1,), cfg):
            assert netmap.reach() >= 0.4 + cfg.track_width / 2 - 0.05, netmap.reach()
            assert netmap.via, 'its vias are held too'
        assert not netmap.track and not netmap.via, 'stamp() removes exactly what it added'
        other = Rec()
        with stamp(other, pcb, (2,), cfg):
            assert not other.track, 'a net the rule does not bind adds nothing'
        attach(pcb, None)
        print('   PASS')

        print('5. check_drc flags each row where KiCad does')
        for name, dru, edge_min, flagged in CASES:
            d = os.path.join(td, 'g_' + name.replace(' ', '_').replace('.', '_'))
            os.makedirs(d)
            path = os.path.join(d, 'b.kicad_pcb')
            ys = write_board(path, dru, edge_min)
            out = os.path.join(d, 'drc.json')
            subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), path, '--json',
                                 out, '--quiet'], capture_output=True, text=True, timeout=120,
                           check=False)
            got = set()
            for it in json.load(open(out, encoding='utf-8'))['items']:
                if it['type'] in ('segment-board-edge', 'via-board-edge') \
                        and it.get('accepted') is None:
                    y = (it.get('seg_loc') or (0, it.get('via_loc', (0, 0))[1]))[1]
                    got.add(ys[int(y)])
            # check_drc holds Board Setup's minimum no lower than the fab floor
            want = flagged if edge_min else flagged
            assert got == want, (name, sorted(got), sorted(want))
            if cli:
                k_out = os.path.join(d, 'kicad.json')
                subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-error',
                                '-o', k_out, path], capture_output=True, text=True, timeout=120,
                               check=False)
                k_got = set()
                for v in json.load(open(k_out, encoding='utf-8'))['violations']:
                    if v['type'] == 'copper_edge_clearance':
                        for i in v['items']:
                            if 'Edge.Cuts' not in i['description']:
                                k_got.add(ys[int(i['pos']['y'])])
                assert k_got == want, ('kicad-cli', name, sorted(k_got), sorted(want))
        print(f"   PASS{' and kicad-cli agrees on every row' if cli else ' (no kicad-cli here)'}")

        print('6. a routed board keeps the rule')
        d = os.path.join(td, 'route')
        os.makedirs(d)
        src = os.path.join(d, 'in.kicad_pcb')
        subprocess.run(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), SMALL_BOARD, src],
                       capture_output=True, text=True, timeout=60, check=True)
        with open(os.path.join(d, 'in.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write('(version 1)\n(rule "edge_all" (constraint edge_clearance (min 1.0mm)))\n')
        out = os.path.join(d, 'out.kicad_pcb')
        r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'route.py'), src, out,
                                 '--nets', '*'], capture_output=True, text=True, timeout=500,
                           check=False)
        assert os.path.isfile(out), r.stdout[-1500:] + r.stderr[-500:]
        assert "rule 'edge_all': edge_clearance min 1 mm" in r.stdout, r.stdout[:1500]
        assert "rule 'edge_all' (edge_clearance min 1 mm): this step's" in r.stdout, r.stdout[-1500:]
        js = json.loads(next(ln for ln in reversed(r.stdout.splitlines())
                             if ln.startswith('JSON_SUMMARY:'))[len('JSON_SUMMARY:'):])
        rows = js['design_rules']['edge_rules']
        assert [x['rule'] for x in rows] == ['edge_all'], rows
        board = parse_kicad_pcb(out)
        graded = edge_rule_grade(board, board.segments, board.vias, src)[0]
        assert graded['tracks'] > 0, graded
        project = json.load(open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8'))
        written = project['board']['design_settings']['rules']['min_copper_edge_clearance']
        assert written < 1.0, f'the rule is the rule file\'s, not the board minimum: {written}'
        d2 = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out, '--json',
                                  os.path.join(d, 'drc.json'), '--quiet'],
                            capture_output=True, text=True, timeout=120, check=False)
        assert d2.returncode in (0, 1), d2.stderr[-500:]
        real = [i for i in json.load(open(os.path.join(d, 'drc.json'), encoding='utf-8'))['items']
                if i['type'] in ('segment-board-edge', 'via-board-edge')
                and i.get('accepted') is None]
        assert not real, f'copper inside the rule off any pad: {real[:3]}'
        print(f"   PASS: {graded['tracks']} tracks, {graded['vias']} vias bound, "
              f"closest {graded['closest']} mm; those inside are on pads inside the rule")
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
