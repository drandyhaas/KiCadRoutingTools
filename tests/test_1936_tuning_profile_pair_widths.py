"""#1936 part 2: a differential pair is coupled at its tuning profile's width
and gap on each layer.

A KiCad 10 differential tuning profile (`type` 1) stores, per signal layer,
the member width and the P/N gap of a pair at its target impedance
(`layer_entries[].width` / `.diff_pair_gap`, nanometres). route_diff drew such
a pair at its net class's one `diff_pair_width` / `diff_pair_gap` on every
layer.

The board is cap_chain's two pairs (DPA_*, DPB_*; SMD pads on F.Cu, the
connector's through-hole pads on every layer) in class DP (0.25 mm pair width
and gap, 0.2 mm clearance) that names the differential profile D90.

  1. `profile_pair_geometry` gives each pair its (width, gap) per layer, a
     floor pins a narrow width up and names it, and a single-ended profile or
     a pair with no profile gives nothing;
  2. route_diff.py on F.Cu alone, no --track-width / --diff-pair-gap: every
     leg is the profile's F.Cu width at the profile's F.Cu pitch, connectors
     included, and the run says which profile;
  3. F.Cu priced up so the pairs run on B.Cu: each layer's coupled run is at
     its own width and pitch, and the run says the connectors keep the class
     geometry (the pads are on layers the profile sizes differently);
  4. a profile gap under the clearance is built at the clearance, and named;
  5. an explicit --track-width, or an explicit --diff-pair-gap, wins;
  6. one pair in a class with no profile: the engine carries one geometry per
     layer per call, so nothing is applied and the run says why;
  7. no profile: nothing printed, the class geometry, as before.
Every routed board is DRC clean at its clearance with every pair net joined.

A layer's width and gap are, in order: a stored impedance declaration's, a
`.kicad_dru` rule's, the profile's, the class's:
  8. --impedance outranks the profile, and so does a stored impedance
     declaration;
  9. a `.kicad_dru` rule that gives one `track_width` for every layer sets
     the width (the profile still gives the gap), and a `diff_pair_gap` rule
     sets the gap; the run names each;
 10. the profile's width is clamped into Board Setup's minimum track and its
     gap into a rule's minimum, each named.

The two engine changes the per-layer gap needs:
 11. a profile's pair narrowed to fit (#1742) is routed, with the narrowed
     copper at the profile's gap; `narrowed_pair_config` used to drop the gap
     of every layer it was not handed, which the config row in 12 pins;
 12. the P/N via offset reads each layer's own gap: at the offset
     `_pair_via_offset` gives, every layer's tracks at their own half-pitch
     clear the partner's via, for a pair narrowed by `narrowed_pair_config`
     and for a profile's gaps under the class gap. With the default
     --routing-clearance-margin 1.0 and a gap no narrower than the P/N
     clearance the via-to-via distance already covers it, so the offset is
     what it was.
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
from test_diff_dru_layer_widths import _coupled_pitches  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'cap_chain.kicad_pcb')
NETS = ['DPA_*', 'DPB_*']
LINE = 'Tuning-profile pair geometry:'
CLR = 0.2
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def entry(layer, width, gap):
    return {"signal_layer": layer, "top_reference_layer": "",
            "bottom_reference_layer": "", "delay": 6000000,
            "diff_pair_gap": int(round(gap * 1e6)), "width": int(round(width * 1e6))}


def project(entries, kind=1, pattern='DP*'):
    cls = {"clearance": CLR, "track_width": 0.25, "diff_pair_gap": 0.25,
           "diff_pair_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3}
    return {
        "net_settings": {
            "classes": [dict(cls, name="Default"),
                        dict(cls, name="DP", priority=0, tuning_profile="D90")],
            "netclass_patterns": [{"netclass": "DP", "pattern": pattern}]},
        "tuning_profiles": {"meta": {"version": 0}, "tuning_profiles_impedance_geometric": [{
            "enable_time_domain_tuning": False, "profile_name": "D90",
            "target_impedance": 90.0, "type": kind, "via_overrides": [],
            "via_prop_delay": 0, "layer_entries": entries}]}}


def stage(td, tag, proj, dru=None, src=None):
    sub = os.path.join(td, tag)
    os.makedirs(sub)
    board = os.path.join(sub, 'cc.kicad_pcb')
    shutil.copy(run_utils.evidence(src or BOARD), board)
    if dru:
        with open(os.path.join(sub, 'cc.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    for stem in ('cc', 'o'):
        with open(os.path.join(sub, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump(proj, fh)
    return run_utils.evidence(board), os.path.join(sub, 'o.kicad_pcb')


def graded(out):
    """(parsed board, clearance violations past the writer's rounding, open
    pair nets) of a routed board."""
    from kicad_parser import parse_kicad_pcb
    from check_drc import run_drc
    from check_connected import run_connectivity_check
    from kicad_writer import EDGE_ROUNDING_MM
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(run_utils.evidence(out))
        drc = run_drc(out, clearance=CLR, quiet=True, print_summary=False,
                      check_sizes=False, clearance_margin=0.0)
        conn = run_connectivity_check(out, net_patterns=NETS, quiet=True)
    assert any(s.net_id for s in pcb.segments), 'BROKEN TEST: nothing was routed'
    return pcb, [v for v in drc if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM], conn


def cli(td, tag, proj, extra=(), dru=None, src=None):
    board, out = stage(td, tag, proj, dru, src)
    r = subprocess.run(PY + [os.path.join('py_router', 'route_diff.py'), board, out,
                             '--nets'] + NETS + list(extra),
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    assert r.returncode == 0 and 'Traceback' not in r.stderr, (
        f"BROKEN TEST: route_diff.py did not run ({tag})\n{(r.stdout + r.stderr)[-1200:]}")
    return (r.stdout,) + graded(out)


def widths(pcb, layer=None):
    return sorted({round(s.width, 4) for s in pcb.segments
                   if s.net_id and (layer is None or s.layer == layer)})


def clean(tag, drc, conn):
    ok(not drc and not conn, f"{tag}: DRC clean at {CLR} and every pair net joined "
                             f"({len(drc)} violation(s), {len(conn)} open)")


def reader(td):
    print('1. the reader')
    from kicad_parser import parse_kicad_pcb
    from tuning_profiles import profile_pair_geometry
    board, _out = stage(td, 'rd', project([entry('F.Cu', 0.15, 0.3), entry('B.Cu', 0.1, 0),
                                           entry('In1.Cu', 0, 0.2)], pattern='DPA_*'))
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    ids = {n.name: i for i, n in pcb.nets.items() if i}
    pairs = [(ids['DPA_P'], ids['DPA_N']), (ids['DPB_P'], ids['DPB_N'])]
    got, raised = profile_pair_geometry(pcb, board, pairs)
    ok(got == {ids['DPA_P']: ('D90', {'F.Cu': (0.15, 0.3), 'B.Cu': (0.1, None)})}
       and not raised,
       f"the profiled pair gets (width, gap) per layer, no gap where 0 is stored, "
       f"nothing for a layer with no width, and the other pair nothing: {got}")
    got, raised = profile_pair_geometry(pcb, board, pairs, layers=['B.Cu'], floor=0.127)
    ok(got == {ids['DPA_P']: ('D90', {'B.Cu': (0.127, None)})}
       and raised == [('D90', 'B.Cu', 'width', 0.1, 0.127)],
       f"a floor pins the narrow width up and names it, on the layers asked: {got}, {raised}")
    board, _out = stage(td, 'rd_se', project([entry('F.Cu', 0.15, 0.3)], kind=0))
    got, _r = profile_pair_geometry(pcb, board, pairs)
    ok(got == {}, f"a single-ended profile sizes no pair: {got}")


def routed(td):
    two = [entry('F.Cu', 0.15, 0.3), entry('B.Cu', 0.2, 0.35)]

    print('2. F.Cu alone')
    log, pcb, drc, conn = cli(td, 'f', project(two), ['--layers', 'F.Cu'])
    ok(LINE in log and 'D90: 2' in log and 'F.Cu=0.15mm/gap 0.3mm' in log,
       'the run says which profile gave the geometry')
    ok(widths(pcb) == [0.15], f"every leg is the profile's F.Cu width: {widths(pcb)}")
    pitches = _coupled_pitches(pcb, 0.15)
    ok(pitches and min(pitches) > 0.45 - 1e-3 and pitches.count(0.45) > len(pitches) / 2,
       f"coupled at the profile's F.Cu pitch 0.15 + 0.3: {sorted(set(pitches))}")
    clean('F.Cu alone', drc, conn)

    print('3. each layer its own')
    log, pcb, drc, conn = cli(td, 'fb', project(two),
                              ['--layers', 'F.Cu', 'B.Cu', '--layer-costs', '20', '1'])
    ok('F.Cu=0.15mm/gap 0.3mm, B.Cu=0.2mm/gap 0.35mm' in log, 'the run names both layers')
    ok(widths(pcb, 'B.Cu') == [0.2], f"B.Cu legs at the B.Cu width: {widths(pcb, 'B.Cu')}")
    pb, pf = _coupled_pitches(pcb, 0.2), _coupled_pitches(pcb, 0.15)
    # (the short legs either side of a via sit off the pitch)
    # (...and main's slanted via legs sit 1 um under it, so within 2 um)
    ok(pb and min(pb) > 0.5
       and sum(abs(p - 0.55) <= 0.002 for p in pb) > len(pb) / 2,
       f"the B.Cu run at its pitch 0.2 + 0.35: {sorted(pb)}")
    ok(pf and 0.45 in pf, f"the F.Cu run at its pitch 0.15 + 0.3: {sorted(set(pf))}")
    ok('connectors from the pads to the coupled run are drawn at the net-class' in log
       and 0.25 in widths(pcb, 'F.Cu'),
       f"the connectors keep the class width, and the run says so: {widths(pcb, 'F.Cu')}")
    clean('each layer', drc, conn)

    print('4. a gap under the clearance')
    log, pcb, drc, conn = cli(td, 'gap', project([entry('F.Cu', 0.15, 0.1)]),
                              ['--layers', 'F.Cu'])
    ok('F.Cu=0.15mm/gap 0.2mm' in log
       and "profile 'D90' F.Cu: gap 0.1 mm is below what KiCad grades P against N at, "
           "built at 0.2 mm" in log, 'the gap is raised to the clearance and named')
    pitches = _coupled_pitches(pcb, 0.15)
    ok(pitches and min(pitches) > 0.35 - 1e-3, f"built at 0.15 + 0.2: {sorted(set(pitches))}")
    clean('raised gap', drc, conn)

    print('5. explicit flags win')
    log, pcb, drc, conn = cli(td, 'tw', project(two), ['--layers', 'F.Cu',
                                                        '--track-width', '0.22'])
    ok(LINE not in log and widths(pcb) == [0.22], f"--track-width: {widths(pcb)}")
    clean('--track-width', drc, conn)
    log, pcb, drc, conn = cli(td, 'dg', project(two), ['--layers', 'F.Cu',
                                                        '--diff-pair-gap', '0.4'])
    ok(LINE not in log and widths(pcb) == [0.25], f"--diff-pair-gap: {widths(pcb)}")
    clean('--diff-pair-gap', drc, conn)

    print('6. pairs that disagree')
    log, pcb, drc, conn = cli(td, 'split', project(two, pattern='DPA_*'),
                              ['--layers', 'F.Cu'])
    ok(LINE not in log and 'tuning-profile pair geometry NOT applied' in log
       and 'DPA_P: D90' in log and 'DPB_P: no differential profile' in log,
       'nothing is applied and the run names the pairs')
    ok(widths(pcb) == [0.25], f"both pairs at the class width: {widths(pcb)}")
    clean('split', drc, conn)

    print('7. no profile')
    proj = project(two)
    del proj['net_settings']['classes'][1]['tuning_profile']
    log, pcb, drc, conn = cli(td, 'none', proj, ['--layers', 'F.Cu'])
    ok('uning-profile' not in log and widths(pcb) == [0.25],
       f"the class geometry and no line: {widths(pcb)}")
    clean('no profile', drc, conn)


RULE = ('(version 1)\n(rule "W"\n  (constraint %s)\n'
        '  (condition "A.hasNetclass(\'DP\')"))\n')


def with_stackup(td):
    """cap_chain with a 2-layer stackup, which a stored impedance needs."""
    import test_1747_narrowed_pair_gap as t47
    with open(run_utils.evidence(BOARD), encoding='utf-8') as fh:
        text = fh.read()
    i = text.index('\n', text.index('(setup')) + 1
    path = os.path.join(td, 'stackup.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text[:i] + t47.STACKUP + text[i:])
    return run_utils.evidence(path)


def precedence(td):
    two = [entry('F.Cu', 0.15, 0.3), entry('B.Cu', 0.2, 0.35)]

    print('8. --impedance and a stored declaration')
    log, pcb, drc, conn = cli(td, 'imp', project(two), ['--layers', 'F.Cu',
                                                         '--impedance', '90'])
    ok(LINE not in log and 0.15 not in widths(pcb),
       f"--impedance outranks the profile: {widths(pcb)}")
    clean('--impedance', drc, conn)
    proj = project(two)
    proj['kicad_routing_tools'] = {'net_impedance': {
        n: {'ohms': 90.0, 'differential': True, 'pair_gap': 0.25}
        for n in ('DPA_P', 'DPA_N', 'DPB_P', 'DPB_N')}}
    log, pcb, drc, conn = cli(td, 'sto', proj, ['--layers', 'F.Cu'],
                              src=with_stackup(td))
    ok(LINE not in log and 'Reapplying stored 90 ohm differential' in log
       and 0.15 not in widths(pcb),
       f"a stored impedance declaration outranks it: {widths(pcb)}")
    clean('stored', drc, conn)

    print('9. a .kicad_dru rule')
    log, pcb, drc, conn = cli(td, 'rw', project(two), ['--layers', 'F.Cu'],
                              dru=RULE % 'track_width (opt 0.2mm)')
    pitches = _coupled_pitches(pcb, 0.2)
    ok(widths(pcb) == [0.2] and "F.Cu: .kicad_dru rule \"W\" sets the width" in log
       and 'F.Cu=0.2mm/gap 0.3mm' in log,
       f"a rule with one width for every layer sets the width: {widths(pcb)}")
    ok(pitches and min(pitches) > 0.5 - 1e-3 and pitches.count(0.5) > len(pitches) / 2,
       f"and the profile still gives the gap, 0.2 + 0.3: {sorted(set(pitches))}")
    clean('width rule', drc, conn)
    log, pcb, drc, conn = cli(td, 'rg', project(two), ['--layers', 'F.Cu'],
                              dru=RULE % 'diff_pair_gap (opt 0.4mm)')
    pitches = _coupled_pitches(pcb, 0.15)
    ok(widths(pcb) == [0.15] and "F.Cu: .kicad_dru rule \"W\" sets the gap" in log
       and pitches and min(pitches) > 0.55 - 1e-3
       and pitches.count(0.55) > len(pitches) / 2,
       f"a diff_pair_gap rule sets the gap, 0.15 + 0.4: {sorted(set(pitches))}")
    clean('gap rule', drc, conn)

    print('10. the clamp')
    proj = project(two)
    proj['board'] = {'design_settings': {'rules': {'min_track_width': 0.2}}}
    log, pcb, drc, conn = cli(td, 'cm', proj, ['--layers', 'F.Cu'])
    ok(widths(pcb) == [0.2] and "F.Cu: width 0.15 mm is below" in log
       and 'drawn at 0.2 mm' in log,
       f"a profile width under Board Setup's minimum track is drawn at the "
       f"minimum: {widths(pcb)}")
    clean('minimum track', drc, conn)
    log, pcb, drc, conn = cli(td, 'cg', project(two), ['--layers', 'F.Cu'],
                              dru=RULE % 'diff_pair_gap (min 0.35mm)')
    pitches = _coupled_pitches(pcb, 0.15)
    ok("F.Cu: gap 0.3 mm is below" in log and pitches
       and min(pitches) > 0.5 - 1e-3 and pitches.count(0.5) > len(pitches) / 2,
       f"a profile gap under a rule's minimum is built at the minimum, "
       f"0.15 + 0.35: {sorted(set(pitches))}")
    clean('minimum gap', drc, conn)


def narrowed(td):
    print("11. a profile's pair narrowed to fit")
    import math
    import test_1747_narrowed_pair_gap as t47
    sub = os.path.join(td, 'gate')
    os.makedirs(sub)
    src = t47._board(sub)
    proj = project([entry('F.Cu', 0.4, 0.17), entry('B.Cu', 0.4, 0.17)])
    for c in proj['net_settings']['classes']:
        c['clearance'] = t47.CLR
    for stem in ('cc', 'o'):
        with open(os.path.join(sub, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump(proj, fh)
    out = os.path.join(sub, 'o.kicad_pcb')
    r = subprocess.run(PY + [os.path.join('py_router', 'route_diff.py'), src, out,
                             '--nets', 'DPA_*', '--via-size', '0.6', '--via-drill', '0.3',
                             '--layers', 'F.Cu', 'B.Cu'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY: ')), None)
    assert line, f"BROKEN TEST: route_diff.py did not run\n{(r.stdout + r.stderr)[-1200:]}"
    doc = json.loads(line.split(': ', 1)[1])
    rep = next((p for p in doc['pair_reports'] if p['pair'] == 'DPA'), {})
    nar = rep.get('narrowed_to_mm')
    assert doc['routed_diff_pairs'] == ['DPA'] and nar and nar < 0.4 - 1e-6, (
        f"BROKEN TEST: the pair did not route narrowed: {rep}")
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(run_utils.evidence(out))
    names = {n.net_id: n.name for n in pcb.nets.values()}
    segs = {'DPA_P': [], 'DPA_N': []}
    for g in pcb.segments:
        if names.get(g.net_id) in segs:
            segs[names[g.net_id]].append(g)
    gaps = []
    for a in segs['DPA_P']:
        if abs(a.width - nar) < 1e-4 and \
                math.hypot(a.end_x - a.start_x, a.end_y - a.start_y) > 0.3:
            eg = t47._edge_gap(a, [b for b in segs['DPA_N'] if b.layer == a.layer])
            if eg is not None and eg < 0.3:
                gaps.append(round(eg, 3))
    ok(gaps and all(abs(g - 0.17) < 0.005 for g in gaps),
       f"narrowed to {nar:g}, the narrowed copper sits at the profile's gap "
       f"0.17, not the class's 0.25: {sorted(gaps)}")


def via_offset(td):
    print('12. the P/N via offset')
    from routing_config import GridRouteConfig
    from diff_pair_loop import narrowed_pair_config
    from diff_pair_routing import _pair_via_offset, layer_pair_spacing

    def base(margin, **kw):
        return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.25, clearance=0.15,
                               via_size=0.6, via_drill=0.3, grid_step=0.05,
                               diff_pair_gap=0.25, routing_clearance_margin=margin, **kw)

    def short(cfg):
        """The furthest any layer's track, at its own half-pitch, sits inside
        the partner via's clearance when the vias are at the offset."""
        off = _pair_via_offset(cfg, (cfg.track_width + cfg.diff_pair_gap) / 2)
        sp = layer_pair_spacing(cfg)
        return max((cfg.clearance + cfg.get_track_width(l) / 2 + cfg.via_size / 2)
                   * cfg.routing_clearance_margin - (off + sp(l)) for l in cfg.layers)

    wide = base(1.3, layer_widths={'F.Cu': 0.41, 'B.Cu': 0.3})
    nc = narrowed_pair_config(wide, 0.2, {'F.Cu': 0.16})
    ok(nc.pair_layer_gaps == {'F.Cu': 0.16} and short(nc) <= 1e-9,
       f"a pair narrowed to 0.2 at gap 0.16: its tracks clear the partner's "
       f"via at the offset (short by {max(short(nc), 0):.4f})")
    prof = base(1.3, layer_widths={'F.Cu': 0.15, 'B.Cu': 0.2},
                pair_layer_gaps={'F.Cu': 0.16, 'B.Cu': 0.2})
    ok(short(prof) <= 1e-9,
       f"a profile's gaps under the class gap: the same "
       f"(short by {max(short(prof), 0):.4f})")
    kept = narrowed_pair_config(prof, 0.15, {'B.Cu': 0.18})
    ok(kept.pair_layer_gaps == {'F.Cu': 0.16, 'B.Cu': 0.18},
       f"a layer a narrowing does not re-solve keeps its gap: {kept.pair_layer_gaps}")
    # inert at the default margin while a gap is no narrower than the clearance
    plain = narrowed_pair_config(base(1.0, layer_widths={'F.Cu': 0.41, 'B.Cu': 0.3}),
                                 0.2, {'F.Cu': 0.16})
    sp0 = (plain.track_width + plain.diff_pair_gap) / 2
    from dataclasses import replace
    ok(abs(_pair_via_offset(plain, sp0)
           - _pair_via_offset(replace(plain, pair_layer_gaps={}), sp0)) < 1e-9
       and short(plain) <= 1e-9,
       f"at margin 1.0 the via-to-via distance already covers it: the offset "
       f"is {_pair_via_offset(plain, sp0):g} with the layer's gap or without")


def main():
    td = tempfile.mkdtemp(prefix='krt1936p_')
    try:
        reader(td)
        routed(td)
        precedence(td)
        narrowed(td)
        via_offset(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
