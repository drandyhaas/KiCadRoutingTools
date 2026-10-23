#!/usr/bin/env python3
"""py_tools/route_review.py: what to move after a route.

On tracked boards (a few seconds, no corpus):
  1. the review runs end to end on a routed board (routed_output, a partly
     routed BGA board) and writes every key the skills cite; a net with no
     copper names all its parts as loose;
  2. a connected net with its copper deleted turns OPEN, named with the part
     holding the loose pad -- the control that the open list reads the copper;
  3. on esp_prog's human placement no two-pad part is turned against its
     wiring, and one with its pads swapped is, with a `swap_pose` that puts
     each pad where the other was (#1322);
  4. a congested area names the nets through it.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for d in ('py_tools', 'py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, d))
from kicad_parser import parse_kicad_pcb  # noqa: E402
import route_review  # noqa: E402

ROUTED = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
PLACED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
results = []


def ok(name, cond, detail=''):
    results.append(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f" -- {detail}" if not cond and detail else ''))


def test_runs_and_writes_its_keys():
    r = route_review.review(ROUTED)
    for k in ('open_nets', 'most_vias', 'longest_detours', 'congested_areas',
              'turned_parts', 'counts', 'layers_with_signal', 'pitch_mm'):
        ok(f"key {k}", k in r)
    bare = [o for o in r['open_nets'] if not o['copper']]
    ok("a net with no copper names all its parts as loose",
       bool(bare) and all(o['disconnected_pad_parts'] == o['parts'] for o in bare), str(bare[:2]))
    ok("costly nets carry a detour over their tree",
       all(n['detour'] >= 1.0 for n in r['longest_detours']), str(r['longest_detours'][:2]))
    return r


def test_a_cut_track_is_an_open_net():
    r0 = route_review.review(ROUTED)
    shut = {o['net'] for o in r0['open_nets']}
    pcb = parse_kicad_pcb(ROUTED)
    rails = route_review.BoardView(pcb).rails
    seg = next(s for s in pcb.segments
               if s.net_id and s.net_id not in rails and not getattr(s, 'graphic', False)
               and pcb.nets[s.net_id].name not in shut
               and len({p.component_ref for p in pcb.pads_by_net.get(s.net_id, ())}) == 2)
    net = pcb.nets[seg.net_id].name
    pcb.segments = [s for s in pcb.segments if s.net_id != seg.net_id]
    pcb.vias = [v for v in pcb.vias if v.net_id != seg.net_id]
    r = route_review.review(ROUTED, pcb=pcb)
    hit = [o for o in r['open_nets'] if o['net'] == net]
    ok(f"{net}, connected, is open with its copper removed", bool(hit), str(r['counts']))
    ok("its loose pads' parts are named", bool(hit) and bool(hit[0]['disconnected_pad_parts']),
       str(hit))


def test_a_swapped_part_is_turned():
    r0 = route_review.review(PLACED)
    ok("the human placement has no turned two-pad part", r0['turned_parts'] == [],
       str(r0['turned_parts'][:3]))
    pcb = parse_kicad_pcb(PLACED)
    rails = route_review.BoardView(pcb).rails
    # a two-pad part whose two signal pads sit far enough apart to matter
    for ref, fp in sorted(pcb.footprints.items()):
        pads = [p for p in fp.pads if p.net_id and p.net_id not in rails]
        if len(fp.pads) == 2 and len(pads) == 2 and pads[0].net_id != pads[1].net_id:
            break
    else:
        ok("a two-pad signal part exists", False)
        return
    a, b = pads
    a.global_x, b.global_x = b.global_x, a.global_x
    a.global_y, b.global_y = b.global_y, a.global_y
    r = route_review.review(PLACED, pcb=pcb)
    ok(f"{ref} with its pads swapped is turned",
       any(t['ref'] == ref for t in r['turned_parts']), str(r['turned_parts'][:3]))
    # #1322: the printed pose makes the swap the finding priced -- each pad
    # lands where the other was. `rotate` would turn about the origin.
    row = [t for t in r['turned_parts'] if t['ref'] == ref]
    if not row:
        return
    import contextlib
    import io
    import tempfile
    from placement.writer import write_placed_output
    x, y, rot = row[0]['swap_pose']
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'swapped.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            write_placed_output(PLACED, out, [{'reference': ref, 'new_x': x,
                                               'new_y': y, 'new_rotation': rot}])
        moved = parse_kicad_pcb(out).footprints[ref].pads
    # `a` and `b` hold each other's positions in memory, so each written pad
    # must sit at the position the in-memory twin now names.
    got = {p.pad_number: (p.global_x, p.global_y) for p in moved}
    want = {a.pad_number: (a.global_x, a.global_y),
            b.pad_number: (b.global_x, b.global_y)}
    ok(f"{ref}'s swap_pose puts each pad where the other was",
       all(abs(got[n][0] - want[n][0]) < 1e-3 and abs(got[n][1] - want[n][1]) < 1e-3
           for n in want), f"{got} vs {want}")


def test_a_congested_area_names_its_parts():
    r = route_review.review(ROUTED, hot=0.2)
    ok("a low threshold finds an area", bool(r['congested_areas']), str(r['counts']))
    if r['congested_areas']:
        a = r['congested_areas'][0]
        ok("the area names nets through it", bool(a['nets']), str(a))
        ok("the area's box is ordered", a['box'][0] < a['box'][2] and a['box'][1] < a['box'][3])


def main():
    print("route_review: what to move after a route")
    for fn in (test_runs_and_writes_its_keys, test_a_cut_track_is_an_open_net,
               test_a_swapped_part_is_turned, test_a_congested_area_names_its_parts):
        print(f"--- {fn.__name__}")
        fn()
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
