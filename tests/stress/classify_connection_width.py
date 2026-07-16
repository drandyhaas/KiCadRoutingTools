#!/usr/bin/env python3
"""Classify kicad-cli connection_width items: float-dust artifact vs real neck.

KiCad's connection_width checker measures the copper web through its own
float-borderline polygon arithmetic; a cap landing EXACTLY tangent to
adjacent same-net copper (routine grid arithmetic) can read as a micro-void
with a sub-minimum web on copper that is provably >= the floor everywhere
(#406; measured live: 0.079 mm reported on >= 0.2 mm copper). This tool
separates the two populations with the same polygon-erosion proof used to
root-cause that emission:

  For each flagged position, build the net's local copper union (segments as
  round-capped buffers + pads, within RADIUS mm), take the connected
  component nearest the flag, erode by floor/2 * (1-EPS), and test whether it
  SURVIVES connected. A real neck (< floor web) erodes through and splits or
  vanishes; an artifact (copper >= floor everywhere) survives as one piece.

Limitations: zones are not modeled (stripped-corpus outputs carry none); a
flag on zone copper classifies as NO-COPPER-FOUND rather than guessing.

  python3 tests/stress/classify_connection_width.py <wave_dir> [--boards-dir <unrouted_dir>]
  python3 tests/stress/classify_connection_width.py --board <final.kicad_pcb> [--baseline <unrouted>]
  python3 tests/stress/classify_connection_width.py --self-test
"""
import argparse
import json
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tests" / "stress"))

RADIUS = 5.0   # mm of local copper context around the flagged position
EPS = 1e-3     # relative erosion shortfall (stay strictly inside the floor)


def _pad_shape(pad):
    from shapely.geometry import Point, box
    if pad.drill and pad.pad_type == "np_thru_hole":
        return None
    w, h = (pad.size_x or 0) / 2.0, (pad.size_y or 0) / 2.0
    if w <= 0 or h <= 0:
        return None
    if pad.shape == "circle":
        return Point(pad.global_x, pad.global_y).buffer(w, quad_segs=32)
    shp = box(pad.global_x - w, pad.global_y - h, pad.global_x + w, pad.global_y + h)
    if pad.shape in ("oval", "roundrect"):
        r = min(w, h) * (1.0 if pad.shape == "oval" else 0.25)
        shp = shp.buffer(-r).buffer(r, quad_segs=16)
    rot = getattr(pad, "rect_rotation", 0) or 0
    if rot:
        import shapely.affinity as aff
        shp = aff.rotate(shp, rot, origin=(pad.global_x, pad.global_y))
    return shp


def classify_board(final, floor_mm, items):
    """items: [{'nets': set, 'pos': (x,y), ...}] -> same dicts + 'verdict'."""
    from shapely.geometry import LineString, Point
    from shapely.ops import unary_union
    from kicad_parser import parse_kicad_pcb

    pcb = parse_kicad_pcb(str(final))
    name_to_id = {n.name: nid for nid, n in pcb.nets.items()}
    out = []
    for it in items:
        x, y = it["pos"]
        net_ids = {name_to_id[n] for n in it["nets"] if n in name_to_id}
        shapes = []
        for s in pcb.segments:
            if net_ids and s.net_id not in net_ids:
                continue
            if not s.layer.endswith(".Cu"):
                continue
            seg = LineString([(s.start_x, s.start_y), (s.end_x, s.end_y)])
            if seg.distance(Point(x, y)) < RADIUS:
                shapes.append(seg.buffer(s.width / 2.0, quad_segs=32))
        for fp in pcb.footprints.values():
            for pad in fp.pads:
                if net_ids and pad.net_id not in net_ids:
                    continue
                if abs(pad.global_x - x) < RADIUS and abs(pad.global_y - y) < RADIUS:
                    shp = _pad_shape(pad)
                    if shp is not None:
                        shapes.append(shp)
        if not shapes:
            out.append({**it, "verdict": "NO-COPPER-FOUND"})
            continue
        union = unary_union(shapes)
        comps = list(union.geoms) if union.geom_type == "MultiPolygon" else [union]
        comp = min(comps, key=lambda c: c.distance(Point(x, y)))
        eroded = comp.buffer(-(floor_mm / 2.0) * (1.0 - EPS))
        if eroded.is_empty:
            verdict = "REAL-NECK (erodes away)"
        elif eroded.geom_type == "MultiPolygon" and len(list(eroded.geoms)) > 1:
            verdict = "REAL-NECK (splits)"
        else:
            verdict = "ARTIFACT (copper >= floor, survives erosion)"
        out.append({**it, "verdict": verdict})
    return out


_TEST_BOARD = """(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(general
		(thickness 1.6)
		(legacy_teardrops no)
	)
	(paper "A4")
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(setup
		(pad_to_mask_clearance 0)
	)
	(net 0 "")
	(net 1 "N1")
{segments})
"""

_SEG = """	(segment
		(start {x1} {y1})
		(end {x2} {y2})
		(width {w})
		(layer "F.Cu")
		(net "N1")
		(uuid "00000000-0000-0000-0000-0000000000{i:02d}")
	)
"""


def self_test():
    """Validate BOTH branches on known geometry before trusting any verdict:
    a 0.05 mm bridge under a 0.2 floor MUST classify REAL-NECK; the live-jog
    copper (three 0.2 mm segments, provably >= 0.2 everywhere) MUST classify
    ARTIFACT."""
    import tempfile
    ok = True
    with tempfile.TemporaryDirectory(prefix="cwclass_") as d:
        cases = [
            ("neck", [((10, 10), (18, 10), 1), ((18, 10), (22, 10), 0.05),
                      ((22, 10), (30, 10), 1)],
             (20.0, 10.0), "REAL-NECK"),
            ("jog", [((31.5, 59.4), (27.7, 59.4), 0.2),
                     ((27.7, 59.4), (27.7, 59.2), 0.2),
                     ((27.7, 59.2), (27.9, 59.0), 0.2)],
             (27.7, 59.3), "ARTIFACT"),
        ]
        for name, segs, pos, want in cases:
            body = "".join(_SEG.format(x1=a[0], y1=a[1], x2=b[0], y2=b[1],
                                       w=w, i=i)
                           for i, (a, b, w) in enumerate(segs))
            p = os.path.join(d, name + ".kicad_pcb")
            with open(p, "w") as f:
                f.write(_TEST_BOARD.format(segments=body))
            v = classify_board(p, 0.2, [{"nets": {"N1"}, "pos": pos}])[0]["verdict"]
            good = v.startswith(want)
            ok &= good
            print(f"  {'PASS' if good else 'FAIL'}  {name}: {v} (want {want}*)")
    print("self-test", "OK" if ok else "FAILED")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wave", nargs="?", help="wave dir with summary.json")
    ap.add_argument("--board", help="single final .kicad_pcb")
    ap.add_argument("--baseline")
    ap.add_argument("--boards-dir", help="unrouted inputs for baselines")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    from kicad_drc_compare import compare_board_data

    jobs = []
    if args.board:
        jobs.append((Path(args.board), args.baseline))
    elif args.wave:
        wave = Path(args.wave)
        for r in json.loads((wave / "summary.json").read_text()):
            if r.get("kicad_connection_width"):
                final = wave / r["board"] / (r["final"] or "final.kicad_pcb")
                base = (Path(args.boards_dir) / f"{r['board']}.kicad_pcb"
                        if args.boards_dir else None)
                jobs.append((final, str(base) if base and base.exists() else None))
    else:
        ap.error("need a wave dir, --board, or --self-test")

    if not jobs:
        print("no boards with connection_width items")
        return 0
    total = {"artifact": 0, "real": 0, "other": 0}
    for final, base in jobs:
        data = compare_board_data(str(final), baseline=base)
        items = data.get("connection_width_items") or []
        floor_mm = data.get("connection_width_min")
        if not items or not floor_mm:
            print(f"{final}: no items on regrade")
            continue
        for v in classify_board(final, floor_mm, items):
            kind = ("artifact" if v["verdict"].startswith("ARTIFACT")
                    else "real" if v["verdict"].startswith("REAL") else "other")
            total[kind] += 1
            print(f"{Path(final).parent.name}: {v['verdict']:44s} "
                  f"{sorted(v['nets'])} @ {v['pos']}  {v.get('desc', '')[:50]}")
    print(f"\nclassified: {total['artifact']} artifact(s), {total['real']} "
          f"real neck(s), {total['other']} unresolved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
