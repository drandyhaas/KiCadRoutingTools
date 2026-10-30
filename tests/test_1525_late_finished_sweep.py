#!/usr/bin/env python3
"""A net the in-run cleanup kept as unfinished, and a later pass finished,
ships without its dead ends (#1525).

run_post_route_cleanup protects an UNFINISHED net from the dead-end sweep
(#473): its spurs are the landing sites a later pass welds to. The graze
prune before it is not scoped that way and can cut such a net's link in two;
when the plane finalize or the reconciliation then finishes the net, nothing
swept the two halves, and KiCad graded them track_dangling. The end of the
run now sweeps the dead ends of every protected net that is COMPLETE on the
board it ships (route._late_finished_dead_end_sweep1525), on both fronts.

Fixture: GND poured on In1.Cu only; pad U1 reaches the pour through a via,
which also carries a B.Cu stub whose far end joins nothing. Pad U2 reaches
the pour through its own via -- or, in the unfinished control, has no copper
at all.

    python3 tests/test_1525_late_finished_sweep.py
"""
import os
import sys
import tempfile

RUN_ALL_TIMEOUT = 120
#: seconds; runs no chain and starts no child process.
RUN_ALL_FAST_OK = True

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

from kicad_parser import parse_kicad_pcb                          # noqa: E402
from improvement_gate import copper_signature                     # noqa: E402
import pcb_modification as pm                                     # noqa: E402
import route                                                      # noqa: E402

FAILURES = []
STUB = (15.0, 20.0, 20.0, 20.0)


def check(name, ok, detail=''):
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{('  -- ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(name)


def pad(ref, x):
    return f'''
  (footprint "P:Pad" (layer "F.Cu") (at {x} 20)
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
    (property "Value" "P" (at 0 1.5 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))'''


def write_board(td, name, finished):
    """The fixture, written next to a project file; returns its path."""
    u2_copper = '''
  (segment (start 30 20) (end 25 20) (width 0.2) (layer "F.Cu") (net 1))
  (via (at 25 20) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))''' \
        if finished else ''
    text = f'''(kicad_pcb (version 20240108) (generator "test") (generator_version "8.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND"){pad("U1", 10)}{pad("U2", 30)}
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (segment (start 10 20) (end 15 20) (width 0.2) (layer "F.Cu") (net 1))
  (via (at 15 20) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (segment (start 15 20) (end 20 20) (width 0.2) (layer "B.Cu") (net 1)){u2_copper}
  (zone (net 1) (net_name "GND") (layer "In1.Cu") (hatch edge 0.5)
    (connect_pads (clearance 0.2)) (min_thickness 0.2) (filled_areas_thickness no)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 2 2) (xy 38 2) (xy 38 38) (xy 2 38))))
)
'''
    path = os.path.join(td, f'{name}.kicad_pcb')
    with open(path, 'w') as f:
        f.write(text)
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w') as f:
        f.write('{}')
    return path


def is_stub(s):
    return s.layer == 'B.Cu' and (round(s.start_x, 3), round(s.start_y, 3),
                                  round(s.end_x, 3), round(s.end_y, 3)) == STUB


def stub_in(segments):
    return any(is_stub(s) for s in segments)


def signature(pcb, segments, vias):
    return copper_signature(segments, vias,
                            lambda n: pcb.nets[n].name if n in pcb.nets else None)


def cli(td, name, finished, protected, keep_input_copper=False,
        input_is_board=False):
    """The CLI front: the pass edits the written file. Returns (stub still in
    the file, nets reported finished)."""
    path = write_board(td, name, finished)
    sig = None
    if input_is_board:
        b = parse_kicad_pcb(path)
        sig = signature(b, b.segments, b.vias)
    done = route._late_finished_dead_end_sweep1525(
        None, path, False, None, None, sig, protected, keep_input_copper)
    return stub_in(parse_kicad_pcb(path).segments), done


def gui(td, name, stub_is_input):
    """The GUI front: the pass edits the change-set and pcb_data. The stub is
    this run's copper (a result) or input copper. Returns (stub in the write
    model, stub in segments_to_remove, stub in a result, stub in pcb_data)."""
    pcb = parse_kicad_pcb(write_board(td, name, True))
    stub = next(s for s in pcb.segments if is_stub(s))
    orig_segs = [s for s in pcb.segments if stub_is_input or s is not stub]
    orig_vias = list(pcb.vias)
    rd = {'results': [] if stub_is_input else
          [{'net_name': 'GND', 'new_segments': [stub], 'new_vias': []}]}

    def write_model(rd_):
        drop = {id(s) for s in rd_.get('segments_to_remove') or []}
        segs = [s for s in orig_segs if id(s) not in drop]
        for r in rd_.get('results') or []:
            segs += list(r.get('new_segments') or [])
        return {1: segs}, {1: list(orig_vias)}

    route._late_finished_dead_end_sweep1525(
        pcb, None, True, rd, write_model,
        signature(pcb, orig_segs, orig_vias), {'GND'}, False)
    return (stub_in(write_model(rd)[0][1]),
            any(s is stub for s in rd.get('segments_to_remove') or []),
            any(s is stub for r in rd['results'] for s in r['new_segments']),
            any(s is stub for s in pcb.segments))


def main():
    td = tempfile.mkdtemp(prefix='t1525late_')
    print("#1525 precondition: the in-run sweep keeps a protected net's stub")
    pcb = parse_kicad_pcb(write_board(td, 'pre', True))
    pm.sweep_dead_ends([], pcb, {1}, protect_net_ids={1})
    check("sweep_dead_ends with protect_net_ids keeps the B.Cu stub (#473)",
          stub_in(pcb.segments))
    pcb = parse_kicad_pcb(write_board(td, 'pre2', True))
    pm.sweep_dead_ends([], pcb, {1})
    check("control: unprotected, the same sweep removes it",
          not stub_in(pcb.segments))

    fn = getattr(route, '_late_finished_dead_end_sweep1525', None)
    check("route.py has an end-of-run dead-end sweep for finished protected "
          "nets", fn is not None)
    if fn is None:
        print(f"\nFAIL: {len(FAILURES)} check(s): {FAILURES}")
        return 1

    print("#1525 CLI front (the written file)")
    kept, done = cli(td, 'cli_fin', True, {'GND'})
    check("protected net, complete at the end: the stub leaves the file",
          not kept, f"kept={kept}")
    check("... and the net is reported finished", done == {'GND'}, f"{done}")
    kept, done = cli(td, 'cli_open', False, {'GND'})
    check("negative control: protected net still unfinished -> stub kept",
          kept, f"kept={kept}")
    check("... and the net is not reported finished", done == set(), f"{done}")
    kept, _done = cli(td, 'cli_unprot', True, set())
    check("scope: a net that was not protected is not this pass's business",
          kept, f"kept={kept}")
    kept, _done = cli(td, 'cli_keep', True, {'GND'}, keep_input_copper=True,
                      input_is_board=True)
    check("--keep-input-copper: an INPUT stub stays", kept, f"kept={kept}")
    kept, _done = cli(td, 'cli_input', True, {'GND'}, input_is_board=True)
    check("without it, an input stub of a finished net goes too",
          not kept, f"kept={kept}")

    print("#1525 GUI front (the change-set)")
    in_model, in_remove, in_result, in_pcb = gui(td, 'gui_run', False)
    check("this run's stub: dropped from its result, not a removal",
          not in_model and not in_remove and not in_result and not in_pcb,
          f"model={in_model} remove={in_remove} result={in_result} pcb={in_pcb}")
    in_model, in_remove, in_result, in_pcb = gui(td, 'gui_input', True)
    check("an input stub: rides segments_to_remove, leaves pcb_data",
          not in_model and in_remove and not in_pcb,
          f"model={in_model} remove={in_remove} pcb={in_pcb}")

    if FAILURES:
        print(f"\nFAIL: {len(FAILURES)} check(s): {FAILURES}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
