#!/usr/bin/env python3
"""Constraint-agreement harness (#530 section 0): does KiCad's DRC enforce the
value design_rules.DesignRules resolves?

For each fixture row a tiny synthetic board is written with a probe pair
placed at exactly KRT_resolved - EPS and again at KRT_resolved + EPS, then
`kicad-cli pcb drc --format json --severity-all` grades both. AGREEMENT is
"the -EPS probe is flagged with the expected violation type and the +EPS
probe is not". Any other outcome brackets KiCad's real threshold, which a
short bisection then reports so the row's failure names the number KiCad
actually enforces.

The rows are the tier boundaries of pcbnew/drc/drc_engine.cpp::EvalRules,
one per boundary the resolver encodes (tests/test_design_rules.py uses the
same names). This file IS the specification for that order: a boundary the
resolver gets wrong fails here against the real engine, not against a
recollection.

    python3 tests/oracle/constraint_agreement.py            # every row
    python3 tests/oracle/constraint_agreement.py --row pad_override_below_class
    python3 tests/oracle/constraint_agreement.py --keep     # keep the boards

Skips with a note (exit 0) where kicad-cli is not installed, mirroring
kicad_oracle.py; tests/test_530_constraint_agreement.py wraps it for
run_all.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from design_rules import DesignRules, RuleItem  # noqa: E402

EPS = 0.01  # mm; comfortably above KiCad's 1 nm resolution, below any fixture step
W = 0.2     # probe track width (mm)

# violation types kicad-cli reports per constraint kind
VIOLATION_TYPES = {
    'clearance': {'clearance'},
    'track_width': {'track_width'},
    'via_diameter': {'via_diameter'},
    'hole_size': {'drill_out_of_range', 'hole_size'},
    'hole_to_hole': {'hole_near_hole', 'hole_to_hole'},
    'hole_clearance': {'hole_clearance'},
    'edge_clearance': {'copper_edge_clearance'},
}


# --------------------------------------------------------------------------
# Board writer
# --------------------------------------------------------------------------

DEFAULT_CLASS = {
    "bus_width": 12, "clearance": 0.2, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
    "diff_pair_width": 0.2, "line_style": 0, "microvia_diameter": 0.3, "microvia_drill": 0.2,
    "name": "Default", "pcb_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647,
    "schematic_color": "rgba(0, 0, 0, 0.000)", "track_width": 0.2, "via_diameter": 0.6,
    "via_drill": 0.3, "wire_width": 6,
}

NETS = {1: 'A', 2: 'B', 3: 'C'}


def _fp(ref, x, y, net_id, net_name, pad_clearance=None, size=1.0):
    clr = f" (clearance {pad_clearance})" if pad_clearance is not None else ""
    return f'''
  (footprint "Probe:Pad" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS") (hide yes)
      (effects (font (size 1 1) (thickness 0.15))))
    (property "Value" "P" (at 0 1.5 0) (layer "F.Fab") (hide yes)
      (effects (font (size 1 1) (thickness 0.15))))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size {size} {size}) (layers "F.Cu") (net {net_id} "{net_name}"){clr})
  )'''


def write_board(path, *, segments=(), vias=(), footprints=(), rules=None, classes=None,
                patterns=(), assignments=None, dru=None, edge=(0, 0, 40, 40)):
    """segments: (x1,y1,x2,y2,width,layer,net); vias: (x,y,size,drill,net);
    footprints: dicts for _fp. rules: rules.min_* dict. classes: extra class
    dicts (Default is always complete). patterns: [(glob, class)]."""
    nets = ''.join(f'\n  (net {i} "{n}")' for i, n in NETS.items())
    segs = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width {w}) (layer "{l}") (net {n}))'
                   for a, b, c, d, w, l, n in segments)
    vs = ''.join(f'\n  (via (at {x} {y}) (size {s}) (drill {d}) (layers "F.Cu" "B.Cu") (net {n}))'
                 for x, y, s, d, n in vias)
    fps = ''.join(_fp(**f) for f in footprints)
    x0, y0, x1, y1 = edge
    text = f'''(kicad_pcb (version 20240108) (generator "krt_agreement") (generator_version "8.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 ""){nets}
  (gr_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
{segs}{vs}{fps}
)
'''
    open(path, 'w', encoding='utf-8').write(text)
    base = os.path.splitext(path)[0]
    cls = [dict(DEFAULT_CLASS)] + [dict(c) for c in (classes or [])]
    proj = {
        "board": {"design_settings": {
            "rules": {"min_clearance": 0.0, "min_track_width": 0.0, "min_via_diameter": 0.0,
                      "min_via_annular_width": 0.0, "min_through_hole_diameter": 0.0,
                      "min_hole_to_hole": 0.0, "min_hole_clearance": 0.0,
                      "min_copper_edge_clearance": 0.0, "min_connection": 0.0,
                      "min_microvia_diameter": 0.0, "min_microvia_drill": 0.0,
                      **(rules or {})},
            "rule_severities": {"track_dangling": "ignore", "unconnected_items": "ignore",
                                "lib_footprint_issues": "ignore", "lib_footprint_mismatch": "ignore",
                                "footprint_type_mismatch": "ignore", "missing_courtyard": "ignore",
                                "silk_over_copper": "ignore", "solder_mask_bridge": "ignore",
                                "footprint_filters_mismatch": "ignore"}}},
        "meta": {"filename": os.path.basename(base) + ".kicad_pro", "version": 1},
        "net_settings": {"classes": cls, "meta": {"version": 3},
                         "netclass_patterns": [{"pattern": p, "netclass": c} for p, c in patterns],
                         **({"netclass_assignments": assignments} if assignments else {})},
    }
    json.dump(proj, open(base + ".kicad_pro", "w"), indent=2)
    dru_path = base + ".kicad_dru"
    if dru:
        open(dru_path, 'w').write("(version 1)\n" + dru + "\n")
    elif os.path.exists(dru_path):
        os.remove(dru_path)


def pcb_data_for(path):
    return SimpleNamespace(nets={i: SimpleNamespace(name=n) for i, n in NETS.items()},
                           board_info=SimpleNamespace(copper_layers=['F.Cu', 'B.Cu']),
                           groups={}, source_path=path)


# --------------------------------------------------------------------------
# kicad-cli
# --------------------------------------------------------------------------

def find_kicad_cli():
    try:
        from kicad_oracle import find_kicad_cli as _f
        return _f()
    except Exception:                                          # noqa: BLE001
        return shutil.which('kicad-cli')


def kicad_violations(cli, board):
    out = board + '.drc.json'
    r = subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '--output', out, board], capture_output=True, text=True, timeout=120)
    if not os.path.exists(out):
        raise RuntimeError(f"kicad-cli produced no report (rc={r.returncode}): {r.stderr[:300]}")
    data = json.load(open(out))
    return [(v.get('type'), v.get('description', '')) for v in data.get('violations', [])]


# --------------------------------------------------------------------------
# Rows
# --------------------------------------------------------------------------

def _pair_tracks(gap, layer='F.Cu', nets=(1, 2)):
    """Two parallel 10 mm tracks with copper-edge gap `gap`."""
    y2 = 10 + W + gap
    return [(5, 10, 15, 10, W, layer, nets[0]), (5, y2, 15, y2, W, layer, nets[1])]


def _pad_and_track(gap, pad_clearance, pad_net=1, track_net=2):
    """A 1 mm SMD pad at (10,10) with a track running below it at edge gap `gap`."""
    y = 10 + 0.5 + gap + W / 2
    return ([{'ref': 'U1', 'x': 10, 'y': 10, 'net_id': pad_net, 'net_name': NETS[pad_net],
              'pad_clearance': pad_clearance}],
            [(5, y, 15, y, W, 'F.Cu', track_net)])


class Row:
    def __init__(self, name, kind, krt, geometry, board_kw, note=''):
        self.name, self.kind, self.krt_fn, self.geometry, self.board_kw, self.note = \
            name, kind, krt, geometry, board_kw, note


def _rows():
    rows = []

    def add(name, kind, board_kw, krt, geometry, note=''):
        rows.append(Row(name, kind, krt, geometry, board_kw, note))

    # -- clearance rows -------------------------------------------------
    def cl(a=1, b=2, layer='F.Cu'):
        return lambda dr: dr.resolve('clearance', dr.item_for_net(a, 'track', layer),
                                     dr.item_for_net(b, 'track', layer), layer).min

    add('netclass_alone', 'clearance',
        dict(classes=[], rules={}),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'Default class clearance 0.2, nothing else')
    add('netclass_pairwise_max', 'clearance',
        dict(classes=[{'name': 'HV', 'clearance': 0.5, 'priority': 0}], patterns=[('A', 'HV')]),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'net A in HV(0.5) vs net B in Default(0.2) -> max')
    add('rule_tightens_class', 'clearance',
        dict(dru='(rule "t" (constraint clearance (min 0.3mm)))'),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'unconditioned rule 0.3 over class 0.2')
    add('rule_relaxes_below_class', 'clearance',
        dict(dru='(rule "r" (constraint clearance (min 0.12mm)))', rules={'min_clearance': 0.05}),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'rule 0.12 below class 0.2, board min 0.05 -> the rule wins (last match)')
    add('board_min_clearance_floors_class', 'clearance',
        dict(classes=[{'name': 'T', 'clearance': 0.1, 'priority': 0}], patterns=[('A', 'T'), ('B', 'T')],
             rules={'min_clearance': 0.25}),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'class T 0.1 (both nets) but board min_clearance 0.25 -> the board minimum floors a CLASS')
    add('board_min_clearance_vs_rule', 'clearance',
        dict(dru='(rule "r" (constraint clearance (min 0.12mm)))', rules={'min_clearance': 0.25}),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'rule 0.12 with board min_clearance 0.25: measured on KiCad 10.0.0 the RULE wins '
        '(the post-loop floor applies to net-class values and pad overrides only)')
    add('netclass_scoped_rule', 'clearance',
        dict(classes=[{'name': 'HV', 'clearance': 0.2, 'priority': 0}], patterns=[('A', 'HV')],
             dru='(rule "hv" (constraint clearance (min 0.4mm)) (condition "A.NetClass == \'HV\' || B.NetClass == \'HV\'"))'),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'rule binds the HV pair at 0.4')
    add('netclass_scoped_rule_nonmember', 'clearance',
        dict(classes=[{'name': 'HV', 'clearance': 0.2, 'priority': 0}], patterns=[('A', 'HV')],
             dru='(rule "hv" (constraint clearance (min 0.4mm)) (condition "A.NetClass == \'HV\' || B.NetClass == \'HV\'"))'),
        cl(2, 3), lambda v: dict(segments=_pair_tracks(v, nets=(2, 3))),
        'the B-C pair is not HV: class 0.2')
    add('layer_scoped_rule_on_layer', 'clearance',
        dict(dru='(rule "b" (layer "B.Cu") (constraint clearance (min 0.3mm)))'),
        cl(layer='B.Cu'), lambda v: dict(segments=_pair_tracks(v, layer='B.Cu')),
        '(layer B.Cu) rule binds on B.Cu')
    add('layer_scoped_rule_off_layer', 'clearance',
        dict(dru='(rule "b" (layer "B.Cu") (constraint clearance (min 0.3mm)))'),
        cl(layer='F.Cu'), lambda v: dict(segments=_pair_tracks(v, layer='F.Cu')),
        '(layer B.Cu) rule does not bind on F.Cu')
    add('netname_condition', 'clearance',
        dict(dru='(rule "n" (constraint clearance (min 0.35mm)) (condition "A.NetName == \'A\' || B.NetName == \'A\'"))'),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'NetName condition')
    add('multi_class_priority', 'clearance',
        dict(classes=[{'name': 'P1', 'clearance': 0.4, 'priority': 0},
                      {'name': 'P2', 'clearance': 0.6, 'priority': 1}],
             patterns=[('A', 'P1'), ('A*', 'P2')]),
        cl(), lambda v: dict(segments=_pair_tracks(v)),
        'net A matches P1 (priority 0) and P2 (priority 1): the aggregate takes P1')

    # -- pad override rows (KiCad returns before rules) ---------------------
    def cl_pad(dr):
        return dr.resolve('clearance',
                          dr.item_for_net(2, 'track', 'F.Cu'),
                          RuleItem(type='pad', net_id=1, net_name='A', layers=frozenset({'F.Cu'}),
                                   netclasses=dr.memberships.get(1, frozenset()),
                                   effective_class=dr.effective_class(1),
                                   footprint_ref='U1', pad_type='smd', clearance_override=dr._pad_clr),
                          'F.Cu').min

    def pad_geom(pad_clr):
        def g(v):
            fps, segs = _pad_and_track(v, pad_clr)
            return dict(footprints=fps, segments=segs)
        return g

    add('pad_override_below_class', 'clearance',
        dict(classes=[], rules={'min_clearance': 0.05}, _pad_clr=0.1),
        cl_pad, pad_geom(0.1),
        'pad (clearance 0.1) under a 0.2 class, board min 0.05 -> 0.1 (override replaces)')
    add('pad_override_below_board_min', 'clearance',
        dict(classes=[], rules={'min_clearance': 0.15}, _pad_clr=0.1),
        cl_pad, pad_geom(0.1),
        'pad override 0.1 floored at board min_clearance 0.15')
    add('pad_override_beats_rule', 'clearance',
        dict(dru='(rule "t" (constraint clearance (min 0.4mm)))', rules={'min_clearance': 0.05},
             _pad_clr=0.1),
        cl_pad, pad_geom(0.1),
        'pad override 0.1 wins over an unconditioned 0.4 rule (early return)')

    # -- size rows ---------------------------------------------------------
    def tw(dr):
        return dr.resolve('track_width', dr.item_for_net(1, 'track', 'F.Cu'), None, 'F.Cu').min

    add('board_min_track_width', 'track_width',
        dict(rules={'min_track_width': 0.2}),
        tw, lambda v: dict(segments=[(5, 10, 15, 10, v, 'F.Cu', 1)]),
        'board min_track_width alone')
    add('rule_lowers_track_min_below_board', 'track_width',
        dict(rules={'min_track_width': 0.2}, dru='(rule "n" (constraint track_width (min 0.1mm)))'),
        tw, lambda v: dict(segments=[(5, 10, 15, 10, v, 'F.Cu', 1)]),
        'a rule may take track_width BELOW the board minimum (no post-loop floor)')
    add('rule_raises_track_min', 'track_width',
        dict(rules={'min_track_width': 0.2}, dru='(rule "w" (constraint track_width (min 0.5mm)))'),
        tw, lambda v: dict(segments=[(5, 10, 15, 10, v, 'F.Cu', 1)]),
        'rule 0.5 over board 0.2')
    add('netclass_width_is_not_a_minimum', 'track_width',
        dict(rules={'min_track_width': 0.1}, classes=[{'name': 'W', 'track_width': 0.5, 'clearance': 0.2, 'priority': 0}],
             patterns=[('A', 'W')]),
        tw, lambda v: dict(segments=[(5, 10, 15, 10, v, 'F.Cu', 1)]),
        'class W track_width 0.5 is a draw default; DRC floor is the board 0.1')

    def vd(dr):
        return dr.resolve('via_diameter', dr.item_for_net(1, 'via', 'F.Cu'), None, 'F.Cu').min

    add('board_min_via_diameter', 'via_diameter',
        dict(rules={'min_via_diameter': 0.5, 'min_through_hole_diameter': 0.2}),
        vd, lambda v: dict(vias=[(10, 10, v, 0.2, 1)]),
        'board min_via_diameter alone')
    add('rule_lowers_via_min_below_board', 'via_diameter',
        dict(rules={'min_via_diameter': 0.5, 'min_through_hole_diameter': 0.15},
             dru='(rule "v" (constraint via_diameter (min 0.3mm)) (condition "A.Type == \'Via\'"))'),
        vd, lambda v: dict(vias=[(10, 10, v, 0.15, 1)]),
        'a Type-scoped via rule below the board minimum wins')
    return rows


# --------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------

def _flagged(viols, kind):
    types = VIOLATION_TYPES[kind]
    return [v for v in viols if v[0] in types]


def run_row(row, cli, workdir, keep=False):
    d = os.path.join(workdir, row.name)
    os.makedirs(d, exist_ok=True)
    board = os.path.join(d, 'probe.kicad_pcb')
    kw = dict(row.board_kw)
    pad_clr = kw.pop('_pad_clr', None)
    # 1. the resolver's answer on a board with NO probe copper (rules only)
    write_board(board, **kw)
    dr = DesignRules.from_project(pcb_data_for(board), board)
    dr._pad_clr = pad_clr
    krt = row.krt_fn(dr)
    result = {'row': row.name, 'kind': row.kind, 'krt': krt, 'note': row.note}
    if krt is None:
        result['verdict'] = 'NO-VALUE'
        return result
    # 2. probe at -EPS and +EPS
    verdicts = {}
    for sign, label in ((-1, 'minus'), (+1, 'plus')):
        v = round(krt + sign * EPS, 4)
        write_board(board, **kw, **row.geometry(v))
        viols = kicad_violations(cli, board)
        verdicts[label] = bool(_flagged(viols, row.kind))
        result[f'{label}_types'] = sorted({t for t, _ in viols})
    result['minus_flagged'] = verdicts['minus']
    result['plus_flagged'] = verdicts['plus']
    if verdicts['minus'] and not verdicts['plus']:
        result['verdict'] = 'AGREE'
    else:
        # bisect KiCad's real threshold in [krt/4, krt*3] to name the number
        lo, hi = max(0.02, krt * 0.25), krt * 3.0
        for _ in range(12):
            mid = round((lo + hi) / 2, 4)
            write_board(board, **kw, **row.geometry(mid))
            if _flagged(kicad_violations(cli, board), row.kind):
                lo = mid
            else:
                hi = mid
        result['kicad'] = round((lo + hi) / 2, 3)
        result['verdict'] = 'DISAGREE'
    if not keep:
        shutil.rmtree(d, ignore_errors=True)
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', action='append', help='run only this row (repeatable)')
    ap.add_argument('--keep', action='store_true', help='keep the probe boards')
    ap.add_argument('--workdir', default=None)
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args(argv)
    cli = find_kicad_cli()
    if not cli:
        print("SKIP: kicad-cli not found (set KICAD_CLI); the agreement harness needs KiCad")
        return 0
    workdir = args.workdir or tempfile.mkdtemp(prefix='krt_agree_')
    rows = _rows()
    if args.row:
        rows = [r for r in rows if r.name in set(args.row)]
    results = []
    bad = 0
    print(f"kicad-cli: {cli}\n{'row':38} {'kind':13} {'KRT':>7} {'-eps':>5} {'+eps':>5}  verdict")
    for row in rows:
        try:
            res = run_row(row, cli, workdir, keep=args.keep)
        except Exception as e:                                 # noqa: BLE001
            res = {'row': row.name, 'kind': row.kind, 'verdict': f'ERROR {e}'}
        results.append(res)
        v = res.get('verdict')
        if v != 'AGREE':
            bad += 1
        extra = f"  KiCad enforces ~{res['kicad']}" if 'kicad' in res else ''
        print(f"{res['row']:38} {res['kind']:13} {str(res.get('krt')):>7} "
              f"{str(res.get('minus_flagged', '-')):>5} {str(res.get('plus_flagged', '-')):>5}  "
              f"{v}{extra}")
        if v not in ('AGREE',):
            print(f"      {row.note}")
    if args.json_out:
        json.dump(results, open(args.json_out, 'w'), indent=2)
    print(f"\n{len(rows) - bad}/{len(rows)} rows agree with KiCad's DRC engine"
          + (f"  (boards kept in {workdir})" if args.keep else ''))
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
