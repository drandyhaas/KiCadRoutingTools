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


def stage(td, tag, proj):
    sub = os.path.join(td, tag)
    os.makedirs(sub)
    board = os.path.join(sub, 'cc.kicad_pcb')
    shutil.copy(run_utils.evidence(BOARD), board)
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


def cli(td, tag, proj, extra=()):
    board, out = stage(td, tag, proj)
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
       and raised == [('D90', 'B.Cu', 0.1, 0.127)],
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
    ok(pb and min(pb) > 0.5 and pb.count(0.55) > len(pb) / 2,
       f"the B.Cu run at its pitch 0.2 + 0.35: {sorted(set(pb))}")
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


def main():
    td = tempfile.mkdtemp(prefix='krt1936p_')
    try:
        reader(td)
        routed(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
