#!/usr/bin/env python3
"""Cross-check check_drc.py against KiCad's own DRC engine (issue #316).

check_drc.py is the grading authority for the whole stress methodology, so its
accuracy is load-bearing. Per board this runs `kicad-cli pcb drc --format json
--severity-error --refill-zones` (KiCad grades at the board's OWN .kicad_pro
rules -- the clearance ledger writes the routed floors there, so the two
engines see the same constraints), filters both sides to the COPPER-CLEARANCE
classes they both check, then matches items by net pair + location and
classifies every discrepancy:

  * MATCHED        -- same net pair within 0.5 mm: agreement.
  * KICAD-ONLY     -- check_drc false negative (the dangerous direction;
                      e.g. the butterstick shorting_items under-classification).
  * CHECKDRC-ONLY  -- check_drc phantom (over-report) OR a class KiCad cannot
                      see (KiCad 10 net-unifies touching copper, so it is
                      structurally blind to overlaps it merged into one net).

Known-benign asymmetries filtered out:
  * KiCad classes we deliberately do not grade (annular width, courtyard,
    silkscreen, starved thermals, text/edge, footprint checks, ...).
  * KiCad "unconnected_items" (that is check_connected's job, not check_drc's).
  * check_drc warnings (same-net copper classes + sub-coincidence gaps) --
    KiCad permits same-net overlap outright.

Usage:
  python3 tests/stress/kicad_drc_compare.py <board.kicad_pcb> [...]
  python3 tests/stress/kicad_drc_compare.py --wave <wave_dir>   # summary.json
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

KICAD_CLI = os.environ.get(
    "KICAD_CLI", "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli")

# kicad violation types that correspond to copper-clearance/short classes
# check_drc grades. Everything else (annular_width, courtyard, silk, thermal,
# malformed, zone fill, footprint/lib checks, text...) is out of scope.
KICAD_COPPER_TYPES = {
    "clearance", "shorting_items", "tracks_crossing",
    "hole_clearance", "hole_near_hole", "drilled_holes_too_close",
    "drilled_holes_colliding", "via_hole_larger_than_pad",
    "copper_edge_clearance",
}

MATCH_RADIUS_MM = 0.5


def run_kicad_drc(board: str):
    """Run kicad-cli DRC, return (violations, error) with items filtered to
    copper classes. Each -> {type, nets:frozenset, pos:(x,y), desc}."""
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        out_json = f.name
    try:
        r = subprocess.run(
            [KICAD_CLI, "pcb", "drc", "--format", "json", "--severity-error",
             "--refill-zones", "--output", out_json, board],
            capture_output=True, text=True, timeout=1800)
        if not os.path.exists(out_json) or os.path.getsize(out_json) == 0:
            return None, f"kicad-cli produced no report (rc={r.returncode}: {r.stderr.strip()[:200]})"
        data = json.load(open(out_json))
    except Exception as e:  # noqa: BLE001 -- per-board tool robustness
        return None, f"kicad-cli failed: {e}"
    finally:
        try:
            os.unlink(out_json)
        except OSError:
            pass
    out = []
    for v in data.get("violations", []):
        vtype = v.get("type", "")
        if vtype not in KICAD_COPPER_TYPES:
            continue
        nets = set()
        pos = None
        for item in v.get("items", []):
            d = item.get("description", "")
            # descriptions look like: "Track [/NET] on F.Cu, length ..." or
            # "Via [/NET] on F.Cu - B.Cu" or "Pad 3 [/NET] of U1 on F.Cu"
            if "[" in d and "]" in d:
                nets.add(d[d.index("[") + 1:d.index("]")])
            p = item.get("pos")
            if pos is None and p:
                pos = (float(p.get("x", 0.0)), float(p.get("y", 0.0)))
        out.append({"type": vtype, "nets": frozenset(nets),
                    "pos": pos or (0.0, 0.0),
                    "desc": v.get("description", "")})
    return out, None


def run_check_drc(board: str, clearance: float = None, netclasses: bool = False):
    """Run check_drc.run_drc, return counted violations (post warning-split)
    as {type, nets:frozenset, pos:(x,y)}.

    Parity with the kicad-cli side (#326/#338): pad/footprint local-clearance
    overrides are graded unconditionally (they live in the .kicad_pcb, which
    staging never touches); the board's min_copper_edge_clearance is always
    honored (staging leaves it alone); per-NETCLASS clearances only when
    `netclasses` is set -- the staged copy clamps every class down to the
    routed clearance, so grading the original class values here would be
    stricter than what kicad-cli sees."""
    from check_drc import run_drc
    kw = {"clearance": clearance} if clearance else {}
    try:
        from fix_kicad_drc_settings import read_project_edge_clearance
        edge = read_project_edge_clearance(board)
        if edge:
            kw["board_edge_clearance"] = edge
    except Exception:
        pass
    if netclasses:
        try:
            from list_nets import read_design_rules, net_clearance_map
            from kicad_parser import extract_nets
            rules = read_design_rules(board)
            if rules.get("classes"):
                with open(board, encoding="utf-8", errors="replace") as f:
                    net_objs, _ = extract_nets(f.read())
                ncl = net_clearance_map(board, [n.name for n in net_objs.values()],
                                        rules=rules)
                if ncl:
                    kw["net_clearances"] = ncl
        except Exception:
            pass
    # #383: NO redirect_stdout -- swapping the process-global sys.stdout inside
    # a worker thread stole other concurrent boards' prints (result lines) into
    # this buffer. print_summary=False makes run_drc emit nothing in the success
    # path, so no capture is needed.
    violations = run_drc(board, quiet=True, print_summary=False, **kw)
    out = []
    for v in violations or []:
        nets = frozenset(str(v.get(k)) for k in ("net1", "net2") if v.get(k))
        pos = None
        for k in ("loc1", "via_loc", "pad_loc", "seg_loc", "cross_point", "loc2"):
            p = v.get(k)
            if p:
                pos = (float(p[0]), float(p[1]))
                break
        item = {"type": v["type"], "nets": nets, "pos": pos or (0.0, 0.0)}
        # Board-edge reconciliation (edge family) needs the WHOLE segment, not
        # just its start: check_drc anchors segment-board-edge at the segment
        # start while kicad anchors copper_edge_clearance at the edge-closest
        # point ON the segment, so point-to-segment distance collapses the
        # anchor mismatch. seg_loc is a 4-tuple (sx, sy, ex, ey).
        seg = v.get("seg_loc")
        if seg and len(seg) >= 4:
            item["seg"] = (float(seg[0]), float(seg[1]), float(seg[2]), float(seg[3]))
        out.append(item)
    return out


def match(kicad_items, cd_items):
    """Greedy match by shared net + proximity; returns (matched, kicad_only, cd_only)."""
    used = set()
    matched = []
    kicad_only = []
    for kv in kicad_items:
        best = None
        for i, cv in enumerate(cd_items):
            if i in used:
                continue
            if kv["nets"] and cv["nets"] and not (kv["nets"] & cv["nets"]):
                continue
            d = math.hypot(kv["pos"][0] - cv["pos"][0], kv["pos"][1] - cv["pos"][1])
            # The two engines anchor a violation at different reference points
            # (check_drc: pad copper centre; kicad: item anchor / track vertex),
            # which for big module pads differ by mm. When BOTH nets agree the
            # identity is unambiguous -- allow a generous radius; the tight
            # radius only arbitrates single-net/partial matches.
            radius = 3.0 if (kv["nets"] and kv["nets"] == cv["nets"]) else MATCH_RADIUS_MM
            if d <= radius and (best is None or d < best[0]):
                best = (d, i)
        if best is not None:
            used.add(best[1])
            matched.append((kv, cd_items[best[1]]))
        else:
            kicad_only.append(kv)
    cd_only = [cv for i, cv in enumerate(cd_items) if i not in used]
    return matched, kicad_only, cd_only


# --- Board-edge anchor reconciliation (edge family only) ---------------------
# The two engines report the SAME copper-to-edge violation at DIFFERENT anchors:
# kicad's `copper_edge_clearance` anchors at the edge-closest point (which lies
# ON the offending copper), while check_drc's `segment-board-edge` anchors at the
# segment START (mm away on a long track). The generic point-to-point matcher
# fails, so a shared edge violation double-counts as kicad_only AND checkdrc_only
# (core1106_cam: 51 shared -> 51 + 51). This narrow reconciliation matches the
# edge family by (a) shared net + proximity to the WHOLE segment, then (b)
# net-set agreement within the family (mirroring the net-PAIR agreement metric
# already used below). Genuine one-sided divergence -- a net only ONE engine
# flags for edge -- has no counterpart on the other side under either pass, so it
# is left in the remaining kicad_only / checkdrc_only and stays reported.
EDGE_KICAD_TYPES = {"copper_edge_clearance"}
EDGE_CD_TYPES = {"segment-board-edge", "via-board-edge", "pad-board-edge"}
EDGE_MATCH_RADIUS_MM = 2.0   # generous: kicad's edge point sits on the segment,


def _point_to_segment(pt, seg):
    """Distance from point pt=(x,y) to segment seg=(sx,sy,ex,ey)."""
    px, py = pt
    sx, sy, ex, ey = seg
    dx, dy = ex - sx, ey - sy
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - sx, py - sy)
    t = ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (sx + t * dx), py - (sy + t * dy))


def _edge_net_ok(kv, cv):
    """Same-net gate for the edge family: a copper-to-edge item carries one net
    (the copper's); a blank/no-net edge item may pair with anything."""
    if kv["nets"] and cv["nets"]:
        return bool(kv["nets"] & cv["nets"])
    return True


def _edge_dist(kv, cv):
    """kicad edge point vs check_drc edge item -- point-to-segment when the cd
    item is a track (its whole segment is known), else point-to-point."""
    if cv.get("seg"):
        return _point_to_segment(kv["pos"], cv["seg"])
    return math.hypot(kv["pos"][0] - cv["pos"][0], kv["pos"][1] - cv["pos"][1])


def reconcile_edge_family(kicad_only, cd_only):
    """Collapse the edge-anchor double-count. Returns
    (n_reconciled, remaining_kicad_only, remaining_cd_only)."""
    k_idx = [i for i, v in enumerate(kicad_only) if v["type"] in EDGE_KICAD_TYPES]
    c_idx = [i for i, v in enumerate(cd_only) if v["type"] in EDGE_CD_TYPES]
    if not k_idx or not c_idx:
        return 0, kicad_only, cd_only
    k_matched, c_matched = set(), set()
    # Pass 1: shared-net proximity to the whole segment (anchor-collapsing).
    for ki in k_idx:
        kv = kicad_only[ki]
        best = None
        for ci in c_idx:
            if ci in c_matched:
                continue
            cv = cd_only[ci]
            if not _edge_net_ok(kv, cv):
                continue
            d = _edge_dist(kv, cv)
            if d <= EDGE_MATCH_RADIUS_MM and (best is None or d < best[0]):
                best = (d, ci)
        if best is not None:
            c_matched.add(best[1])
            k_matched.add(ki)
    # Pass 2: net-set agreement (position-independent) for the residue -- a net
    # BOTH engines flag for edge is the same condition however each anchored it.
    # Min-count pairing (one cd item consumes one kicad item of the same net set)
    # preserves genuine per-net over-flagging as one-sided.
    from collections import defaultdict
    k_by_net = defaultdict(list)
    for ki in k_idx:
        if ki not in k_matched:
            k_by_net[frozenset(kicad_only[ki]["nets"])].append(ki)
    for ci in c_idx:
        if ci in c_matched:
            continue
        pool = k_by_net.get(frozenset(cd_only[ci]["nets"]))
        if pool:
            k_matched.add(pool.pop())
            c_matched.add(ci)
    rem_k = [v for i, v in enumerate(kicad_only) if i not in k_matched]
    rem_c = [v for i, v in enumerate(cd_only) if i not in c_matched]
    return len(k_matched), rem_k, rem_c


# --- #408: accept-by-design edge-clearance subtraction -----------------------
# The dominant remaining corpus DRC class, copper_edge_clearance, mixes real
# defects with accepted-by-design edge routing: a pad/connector whose own copper
# sits inside the board's edge-clearance band (card-edge connectors, edge-mounted
# USB-C, mounting pads) can only be reached by running copper INTO the band. The
# router EMITS those nets as `intentional_edge_band_nets` in its JSON_SUMMARY
# (#408). Grading is NET-SCOPED: if a net was routed into the edge band to reach
# an in-band pad, ALL of its edge-clearance items are accepted-by-design and
# ignored on BOTH engines. Net-scoping keeps the two engines symmetric for free
# (they anchor the same edge violation at different points, so a positional test
# would drop one side and orphan its twin into kicad_only) -- and it is enough:
# a signal net only reaches the edge at its connector, so accepting its whole
# edge class is the right call.


def parse_intentional_edge_nets(log_text: str):
    """Collect `intentional_edge_band_nets` (#408) -- the net names the router
    routed into the edge band -- from every JSON_SUMMARY line in a routing/replay
    log (a board's chain runs several steps, each of which may emit the key:
    single-ended route, plane taps, plane reconnect). Returns the sorted union of
    net names; empty when no step carried the key (the common edge-honoring
    board). Tolerates the older rich {net, ...} dict schema too."""
    nets = set()
    for m in re.finditer(r"JSON_SUMMARY:\s*(\{.*\})", log_text):
        try:
            d = json.loads(m.group(1))
        except Exception:  # noqa: BLE001 -- a non-JSON tail must not abort parsing
            continue
        for e in d.get("intentional_edge_band_nets") or []:
            n = e if isinstance(e, str) else (e.get("net") if isinstance(e, dict) else None)
            if n:
                nets.add(n)
    return sorted(nets)


def _drop_intentional_edge(items, edge_types, intentional_nets):
    """Drop accepted-by-design edge items (#408): every edge-family violation
    whose net is in `intentional_nets` (a set of net names). Net-scoped -- once a
    net is known to reach an in-band pad, its whole edge class is accepted.
    Returns (kept_items, n_dropped)."""
    if not intentional_nets:
        return items, 0
    kept, dropped = [], 0
    for v in items:
        if v["type"] in edge_types and (v["nets"] & intentional_nets):
            dropped += 1
        else:
            kept.append(v)
    return kept, dropped


def _staged_copy(board: str, clearance: float):
    clearance = float(clearance)
    """Copy board + sibling .kicad_pro into a temp dir with the DEFAULT
    net-class clearance forced to `clearance`, so KiCad grades at the SAME
    constraint check_drc uses (the routed clearance) instead of the board's
    original design netclass -- grading stricter manufactures phantom
    clearance items and drowns the real signal.

    PR392: only the DEFAULT class (the routing side, routed at the run clearance)
    is equalized. NON-Default classes keep their ORIGINAL clearance because the
    router now RESPECTS them (pairwise max(classA, classB)); KiCad then grades each
    cross-class pair at its true max(A,B) from the real classes. Clamping non-Default
    classes down (the old behaviour) would grade that copper LOOSER than it was
    routed and hide genuine class-clearance shortfalls."""
    import shutil
    d = tempfile.mkdtemp(prefix="kdrc_")
    b2 = os.path.join(d, os.path.basename(board))
    shutil.copy(board, b2)
    pro = os.path.splitext(board)[0] + ".kicad_pro"
    pro2 = os.path.splitext(b2)[0] + ".kicad_pro"
    try:
        cfg = json.load(open(pro)) if os.path.exists(pro) else {}
    except Exception:
        cfg = {}
    def _f(x, dflt):
        try:
            return float(x)
        except (TypeError, ValueError):
            return dflt
    for c in cfg.setdefault("net_settings", {}).setdefault("classes", []) or []:
        # Only the Default class is equalized to the routed clearance; non-Default
        # classes keep their original clearance (routing respects them -- PR392).
        if c.get("name", "Default") == "Default" and "clearance" in c:
            c["clearance"] = min(_f(c.get("clearance"), clearance), clearance)
    rules = cfg.setdefault("board", {}).setdefault("design_settings", {}).setdefault("rules", {})
    rules["min_clearance"] = min(_f(rules.get("min_clearance"), clearance) or clearance, clearance)
    # Hole floor (#327): equalize min_hole_clearance to the routed copper
    # clearance too -- our stack guarantees the NPTH fab floor (0.2, #308) and
    # copper clearance, not a board's stricter DESIGN hole rule; grading holes
    # stricter than the routing was told manufactures config-mismatch items.
    # (Footprint-LEVEL clearance overrides remain and are a real gap: #326.)
    mhc = _f(rules.get("min_hole_clearance"), None)
    if mhc and mhc > clearance:
        rules["min_hole_clearance"] = clearance
    # Edge (#338/#295 class): when the project records NO copper-to-edge rule,
    # kicad-cli falls back to its built-in 0.5mm default while check_drc falls
    # back to the routed clearance -- a pure config artifact (openstint: chain
    # lost the project, kicad's default manufactured 11 phantom items). Pin
    # the absent key to the routed clearance so both engines grade alike. A
    # RECORDED rule is left untouched (it is a real design constraint).
    if "min_copper_edge_clearance" not in rules:
        rules["min_copper_edge_clearance"] = clearance
    # Silk checks are OUT OF SCOPE (results are filtered to copper classes)
    # but DRC_TEST_PROVIDER_SILK_CLEARANCE dominates wall time on art-heavy
    # boards -- lily58's keyboard legends made kicad-cli spin >10 MINUTES at
    # 100% CPU in that one provider (#333, confirmed by sampling). Severity
    # "ignore" makes the DRC engine skip the provider entirely.
    sev = cfg["board"]["design_settings"].setdefault("rule_severities", {})
    for k in ("silk_over_copper", "silk_overlap", "silk_edge_clearance"):
        sev[k] = "ignore"
    json.dump(cfg, open(pro2, "w"))
    return d, b2


def _pro_clearance(board: str):
    """Read the board's routed copper clearance from its sibling .kicad_pro
    Default net class -- the same Stage-C auto-grade check_drc.py's CLI does
    (#226/#227). Grading at a default (0.1) instead of the board's own floor
    manufactures phantom sub-clearance grazes on legitimately tight copper
    (#359). Returns None if no project / no clearance is recorded."""
    try:
        from fix_kicad_drc_settings import find_project, project_copper_clearance
        pro = find_project(board)
        if os.path.isfile(pro):
            with open(pro) as f:
                return project_copper_clearance(json.load(f))
    except Exception:  # noqa: BLE001 -- best-effort, fall back to check_drc default
        pass
    return None


def kicad_items_for(board: str, clearance: float = None):
    """kicad-cli copper items for a board, staged the same way compare_board
    stages (netclasses clamped to the routed clearance when known). Returns
    (items, err)."""
    if clearance:
        _tmpd, board_k = _staged_copy(board, clearance)
    else:
        _tmpd, board_k = None, board
    try:
        return run_kicad_drc(board_k)
    finally:
        if _tmpd:
            import shutil
            shutil.rmtree(_tmpd, ignore_errors=True)


def _subtract_baseline(items, base_items):
    """Two-pass symmetric baseline subtraction shared by both engines' sides
    (#326/#338/#405): drop from `items` every pre-existing (unrouted-input)
    condition present in `base_items`, so what remains is ROUTER-attributable.
    Pass 1 = positional match (identical template copper => distance 0); pass 2
    = positional-drift-tolerant (type, nets) twin (kicad re-anchors the same
    design condition at a different instance position run-to-run; openstint's
    pre-routed /A- input copper matched 6 of 8 positionally). Returns
    (remaining_items, n_preexisting_subtracted)."""
    if not base_items:
        return items, 0
    matched_pre, base_rest, rest = match(base_items, items)
    pre = len(matched_pre)
    pool = {}
    for bv in base_rest:
        pool[(bv["type"], bv["nets"])] = pool.get((bv["type"], bv["nets"]), 0) + 1
    keep = []
    for v in rest:
        key = (v["type"], v["nets"])
        if pool.get(key):
            pool[key] -= 1
            pre += 1
        else:
            keep.append(v)
    return keep, pre


def compare_board_data(board: str, label: str = None, clearance: float = None,
                       baseline: str = None, intentional_edge_band_nets=None):
    """SHARED per-board grading core used by BOTH the CLI (`compare_board`) and
    ab_replay_grade's `_kicad_grade` -- neither reimplements subtraction or
    matching, so the two tools produce identical numbers for the same board by
    construction.

    Runs kicad-cli + check_drc, symmetrically baseline-subtracts both sides
    (#405), matches the two, then reconciles the board-edge anchor mismatch
    (Part B). `baseline` = the UNROUTED input board: its items are pre-existing
    design conditions (e.g. edge-connector pads inside the board's own
    copper_edge rule) that no routing can fix -- subtracted from BOTH engines so
    the counts reflect router-introduced copper.

    `intentional_edge_band_nets` (#408) = the router's JSON_SUMMARY list of net
    names it deliberately routed into the edge band to reach an in-band pad. Every
    edge-clearance violation on those nets is accepted-by-design (routing to an
    edge connector must ride the band) and dropped from BOTH engines (net-scoped),
    counted as kicad_intentional_edge / checkdrc_intentional_edge; kicad /
    check_drc then report the REAL edge defects. Unlike `baseline`, these are
    router-ADDED but accepted, so baseline subtraction alone never catches them.

    Returns a dict (numeric grade + item lists + verdict for the printer), or
    None if kicad-cli could not grade the board (caller degrades to Nones)."""
    label = label or os.path.basename(board)
    if clearance is None:
        clearance = _pro_clearance(board)
    kicad, err = kicad_items_for(board, clearance)
    if err:
        return {"board": label, "skip": err}
    pre = 0
    if baseline and os.path.exists(baseline):
        base_items, berr = kicad_items_for(baseline, clearance)
        if not berr and base_items:
            kicad, pre = _subtract_baseline(kicad, base_items)
    cd = run_check_drc(board, clearance, netclasses=not bool(clearance))
    # Symmetric baseline subtraction (#405): drop the input's own check_drc
    # items too (e.g. vfo_ctrl's 23 FG chassis-ground segments already <edge on
    # the bare board), mirroring the kicad side, so both are graded on
    # router-attributable copper.
    cd_pre = 0
    if baseline and os.path.exists(baseline):
        try:
            cd_base = run_check_drc(baseline, clearance, netclasses=not bool(clearance))
        except Exception:  # noqa: BLE001 -- best-effort, tolerate a bad input
            cd_base = None
        cd, cd_pre = _subtract_baseline(cd, cd_base or [])
    # #408: accept-by-design edge items -- every copper_edge_clearance (kicad) /
    # *-board-edge (check_drc) violation on a net the router routed into the edge
    # band to reach an in-band pad. Net-scoped drop from BOTH engines BEFORE
    # matching: since both sides lose the same nets' edge items, match/reconcile
    # never see them and neither side is orphaned into *_only (a positional test
    # would drop one engine's anchor and inflate the other's divergence). What
    # remains is REAL edge defects on nets that had no business at the edge.
    kicad_intentional = checkdrc_intentional = 0
    inets = frozenset(n for n in (intentional_edge_band_nets or []) if n)
    if inets:
        kicad, kicad_intentional = _drop_intentional_edge(kicad, EDGE_KICAD_TYPES, inets)
        cd, checkdrc_intentional = _drop_intentional_edge(cd, EDGE_CD_TYPES, inets)
    matched, kicad_only, cd_only = match(kicad, cd)
    # Part B: collapse the board-edge anchor double-count (same edge violation
    # anchored differently by each engine). One-sided edge divergence survives.
    edge_reconciled, kicad_only, cd_only = reconcile_edge_family(kicad_only, cd_only)
    n_matched = len(matched) + edge_reconciled
    # NET-PAIR agreement: the engines report at different granularities
    # (check_drc: one violation per offending segment; kicad: grouped item
    # pairs), so the honest consistency metric is the SET of net pairs each
    # engine implicates. Instance-level diffs then refine within agreed pairs.
    kpairs = {v["nets"] for v in kicad if len(v["nets"]) == 2}
    cpairs = {v["nets"] for v in cd if len(v["nets"]) == 2}
    pk_only, pc_only = kpairs - cpairs, cpairs - kpairs
    verdict = ("CONSISTENT" if not kicad_only and not cd_only else
               "PAIR-CONSISTENT" if not pk_only and not pc_only else "DIVERGED")
    # kicad / check_drc are the REAL router-attributable counts: pre-existing
    # (baseline) AND accepted-by-design edge items (#408) already removed. The
    # subtracted edge counts are reported separately for transparency.
    return {"board": label, "kicad": len(kicad), "kicad_preexisting": pre,
            "check_drc": len(cd), "checkdrc_preexisting": cd_pre,
            "kicad_intentional_edge": kicad_intentional,
            "checkdrc_intentional_edge": checkdrc_intentional,
            "matched": n_matched, "kicad_only": len(kicad_only),
            "checkdrc_only": len(cd_only),
            "pairs_both": len(kpairs & cpairs),
            "pairs_kicad_only": len(pk_only), "pairs_checkdrc_only": len(pc_only),
            "verdict": verdict,
            "kicad_only_items": kicad_only, "checkdrc_only_items": cd_only}


# Summary keys the CLI callers persist (item lists + verdict are print-only).
_SUMMARY_KEYS = ("board", "kicad", "kicad_preexisting", "check_drc",
                 "kicad_intentional_edge", "checkdrc_intentional_edge",
                 "matched", "kicad_only", "checkdrc_only",
                 "pairs_kicad_only", "pairs_checkdrc_only")


def compare_board(board: str, label: str = None, clearance: float = None,
                  baseline: str = None, intentional_edge_band_nets=None):
    """Thin CLI wrapper over compare_board_data: print the per-board line +
    divergence detail, return the summary-key subset (schema unchanged)."""
    label = label or os.path.basename(board)
    data = compare_board_data(board, label=label, clearance=clearance,
                              baseline=baseline,
                              intentional_edge_band_nets=intentional_edge_band_nets)
    if data is None or "skip" in (data or {}):
        print(f"{label}: SKIP ({data.get('skip') if data else 'no result'})")
        return None
    pre_note = f" (+{data['kicad_preexisting']} pre-existing on input, subtracted)" if data["kicad_preexisting"] else ""
    cd_pre_note = f" (+{data['checkdrc_preexisting']} pre-existing, subtracted)" if data["checkdrc_preexisting"] else ""
    # #408: accepted-by-design edge items removed from both engines.
    ie = data.get("kicad_intentional_edge", 0) or data.get("checkdrc_intentional_edge", 0)
    ie_note = (f" [#408: -{data.get('kicad_intentional_edge', 0)} kicad / "
               f"-{data.get('checkdrc_intentional_edge', 0)} check_drc intentional edge]") if ie else ""
    print(f"{label}: kicad={data['kicad']}{pre_note} check_drc={data['check_drc']}{cd_pre_note} "
          f"matched={data['matched']} kicad_only={data['kicad_only']} checkdrc_only={data['checkdrc_only']}{ie_note} | "
          f"net-pairs: both={data['pairs_both']} kicad_only={data['pairs_kicad_only']} "
          f"checkdrc_only={data['pairs_checkdrc_only']}  {data['verdict']}")
    for kv in data["kicad_only_items"]:
        print(f"    KICAD-ONLY  {kv['type']:16s} {sorted(kv['nets'])} @ {kv['pos']}  {kv.get('desc','')[:60]}")
    for cv in data["checkdrc_only_items"]:
        print(f"    CHECKDRC-ONLY {cv['type']:16s} {sorted(cv['nets'])} @ {cv['pos']}")
    return {k: data[k] for k in _SUMMARY_KEYS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("boards", nargs="*")
    ap.add_argument("--wave", action="append", default=[],
                    help="wave dir with summary.json; compares every final board")
    ap.add_argument("--baseline", default=None,
                    help="UNROUTED input board: its kicad items (pre-existing "
                         "design conditions, e.g. edge-connector pads inside "
                         "the board's own edge rule) are subtracted so "
                         "kicad_only reflects router-introduced items "
                         "(single-board mode only)")
    ap.add_argument("--intentional-edge-log", default=None,
                    help="routing/replay log to read the router's "
                         "intentional_edge_band_nets (#408) from; its "
                         "accepted-by-design edge-clearance items are subtracted "
                         "(single-board mode; --wave auto-reads each board's "
                         "_replay.log)")
    args = ap.parse_args()
    boards = list(args.boards)
    # #408: single-board intentional list from an explicit log (if any).
    cli_intentional = None
    if args.intentional_edge_log and os.path.exists(args.intentional_edge_log):
        with open(args.intentional_edge_log, errors="replace") as f:
            cli_intentional = parse_intentional_edge_nets(f.read())
    jobs = [(b, None, cli_intentional) for b in boards]
    for wave in args.wave:
        for e in json.load(open(os.path.join(wave, "summary.json"))):
            if e.get("final"):
                p = os.path.join(wave, e["board"], os.path.basename(e["final"]))
                if os.path.exists(p):
                    clr = e.get("clearance")
                    # #408: the board's chain log carries its intentional list.
                    log = os.path.join(wave, e["board"], "_replay.log")
                    intentional = None
                    if os.path.exists(log):
                        with open(log, errors="replace") as f:
                            intentional = parse_intentional_edge_nets(f.read())
                    jobs.append((p, float(clr) if clr else None, intentional))
    rows = [r for b, clr, intentional in jobs
            if (r := compare_board(b, label=None, clearance=clr,
                                   baseline=args.baseline,
                                   intentional_edge_band_nets=intentional))]
    div = [r for r in rows if r["kicad_only"] or r["checkdrc_only"]]
    print(f"\n{len(rows)} boards compared: {len(rows) - len(div)} consistent, "
          f"{len(div)} diverged "
          f"(kicad_only total {sum(r['kicad_only'] for r in rows)}, "
          f"checkdrc_only total {sum(r['checkdrc_only'] for r in rows)})")
    return 1 if div else 0


if __name__ == "__main__":
    sys.exit(main())
