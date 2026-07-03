#!/usr/bin/env python3
"""Validate one candidate .kicad_pcb for the stress corpus.

Prints a one-line JSON verdict: loads in kicad_parser, has real routable content,
loads in pcbnew (so prep will work), plus a difficulty tier guess.

Usage: python3 validate_candidate.py <file.kicad_pcb>
PASS criteria: parser loads; 2<=copper_layers<=8; footprints>=10; routable_nets>=20.
"""
import json, sys, subprocess, os, re
REPO = "/Users/andy/Documents/KiCadRoutingTools"
sys.path.insert(0, REPO)
KPY = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"


def classify(layers, fps, routable, big_bga):
    if big_bga or layers >= 6 or fps >= 120 or routable >= 250:
        return "hard"
    if layers >= 4 or fps >= 45 or routable >= 80:
        return "medium"
    return "easy"


def _strip_zones(text):
    """Remove every balanced (zone ...) block in a single forward pass. Zone
    COPPER FILLS dominate big boards (a 62MB 8-layer board is ~509k (xy) fill
    points) and none of the validation metrics -- footprints, outline, nets, pads
    -- need them, so dropping zones makes parse_kicad_pcb finish in seconds instead
    of minutes (or, unbounded, hang forever -- there was no timeout on the parse)."""
    out, i, n = [], 0, len(text)
    while True:
        j = text.find('(zone', i)
        if j < 0:
            out.append(text[i:]); break
        out.append(text[i:j]); depth, k = 0, j
        while k < n:
            c = text[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    k += 1; break
            k += 1
        i = k
    return ''.join(out)


def _parse_no_zones(f):
    """parse_kicad_pcb on the board with zones stripped (safe: metrics ignore zones)."""
    import tempfile
    from kicad_parser import parse_kicad_pcb
    try:
        stripped = _strip_zones(open(f, errors='replace').read())
        tf = tempfile.NamedTemporaryFile(suffix='.kicad_pcb', delete=False, mode='w')
        tf.write(stripped); tf.close()
        try:
            return parse_kicad_pcb(tf.name)
        finally:
            os.unlink(tf.name)
    except Exception:
        return parse_kicad_pcb(f)  # fall back to a full parse


def main():
    f = sys.argv[1]
    v = {"file": os.path.basename(f), "pass": False}
    try:
        pcb = _parse_no_zones(f)
        layers = len(pcb.board_info.copper_layers)
        fps = len(pcb.footprints)
        routable = sum(1 for n in pcb.nets.values() if len(n.pads) >= 2 and n.net_id != 0)
        # biggest footprint pad count -> BGA/dense hint
        big = max((len(fp.pads) for fp in pcb.footprints.values()), default=0)
        # Board outline + off-board footprints: a route-ready board needs an
        # Edge.Cuts outline (board_bounds) with its components INSIDE it. Footprints
        # placed outside the outline are unroutable (their pads can't be reached) and
        # manufacture phantom DRC -- a curation defect (framework_dock/apple1/pico_disp).
        bb = pcb.board_info.board_bounds
        off = None
        if bb:
            x0, y0, x1, y1 = bb
            off = sum(1 for fp in pcb.footprints.values()
                      if fp.x < x0 or fp.x > x1 or fp.y < y0 or fp.y > y1)
        v.update(copper_layers=layers, footprints=fps, routable_nets=routable,
                 max_pads=big, has_outline=bool(bb), off_board_fps=off)
    except Exception as e:
        v["error"] = f"parser: {type(e).__name__}: {e}"[:160]
        print(json.dumps(v)); return
    # KiCad version token
    try:
        head = open(f, "r", errors="replace").read(400)
        m = re.search(r"\(version\s+(\d+)\)", head)
        v["kicad_version"] = int(m.group(1)) if m else None
    except Exception:
        v["kicad_version"] = None
    # pcbnew load (prep will use this)
    try:
        r = subprocess.run([KPY, "-c",
            f"import pcbnew,sys; b=pcbnew.LoadBoard(sys.argv[1]); print(len(b.GetFootprints()))", f],
            capture_output=True, text=True, timeout=90)
        v["pcbnew_load"] = (r.returncode == 0 and r.stdout.strip().isdigit())
        if not v["pcbnew_load"]:
            v["pcbnew_err"] = (r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "rc=%d" % r.returncode)[:120]
    except Exception as e:
        v["pcbnew_load"] = False
        v["pcbnew_err"] = str(e)[:120]
    v["tier"] = classify(v["copper_layers"], v["footprints"], v["routable_nets"], v["max_pads"] >= 200)
    # A board is route-ready only if it has an outline AND nearly all footprints sit
    # inside it. Tolerate a couple of edge connectors/mounting parts (<= max(2, 10%)).
    off = v.get("off_board_fps")
    off_ok = v["has_outline"] and off is not None and off <= max(2, 0.1 * v["footprints"])
    if not off_ok:
        v["reject_reason"] = ("no board outline" if not v["has_outline"]
                              else f"{off} footprints off-board (curation defect)")
    v["pass"] = (2 <= v["copper_layers"] <= 8 and v["footprints"] >= 10
                 and v["routable_nets"] >= 20 and v["pcbnew_load"] and off_ok)
    print(json.dumps(v))


if __name__ == "__main__":
    main()
