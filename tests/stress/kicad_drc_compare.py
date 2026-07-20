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

Graded SEPARATELY (#406, never matched against check_drc): KiCad's
`connection_width` min-copper-web class. The checker is OFF by default
(min_connection 0, warning severity), so _staged_copy stages the constraint
(author-set min_connection kept, else the project's min_track_width) and
lifts the severity; the count is reported as kicad_connection_width (None =
not graded -- no recorded floor). check_drc has no counterpart by design:
the flagged artifact lives in KiCad's own float-borderline web measurement.

Usage:
  python3 tests/stress/kicad_drc_compare.py <board.kicad_pcb> [...]
  python3 tests/stress/kicad_drc_compare.py --wave <wave_dir>   # summary.json
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

# Env var wins; else PATH (Windows/Linux installs put kicad-cli there, the
# kicad_oracle.py idiom); else the macOS app-bundle default.
KICAD_CLI = (os.environ.get("KICAD_CLI") or shutil.which("kicad-cli")
             or "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli")

# kicad violation types that correspond to copper-clearance/short classes
# check_drc grades. Everything else (annular_width, courtyard, silk, thermal,
# malformed, zone fill, footprint/lib checks, text...) is out of scope.
KICAD_COPPER_TYPES = {
    "clearance", "shorting_items", "tracks_crossing",
    "hole_clearance", "hole_near_hole", "drilled_holes_too_close",
    "drilled_holes_colliding", "via_hole_larger_than_pad",
    "copper_edge_clearance",
}

# Min-copper-web class (#406): graded SEPARATELY from the copper-clearance
# classes -- check_drc has no counterpart (the artifact lives in KiCad's own
# float-borderline web measurement, which a from-scratch geometric check
# cannot reproduce), so these items never enter the match and can never
# pollute kicad_only (the check_drc-false-negative alarm channel).
KICAD_WEB_TYPES = {"connection_width"}

MATCH_RADIUS_MM = 0.5


def run_kicad_drc(board: str):
    """Run kicad-cli DRC, return (violations, error) with items filtered to
    copper + web classes. Each -> {type, nets:frozenset, pos:(x,y), desc,
    kinds} (kinds = the offending item kinds, e.g. ('track', 'zone'))."""
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
        if vtype not in KICAD_COPPER_TYPES and vtype not in KICAD_WEB_TYPES:
            continue
        nets = set()
        pos = None
        copper_pos = None
        kinds = set()
        for item in v.get("items", []):
            d = item.get("description", "")
            # descriptions look like: "Track [/NET] on F.Cu, length ..." or
            # "Via [/NET] on F.Cu - B.Cu" or "Pad 3 [/NET] of U1 on F.Cu"
            if "[" in d and "]" in d:
                nets.add(d[d.index("[") + 1:d.index("]")])
            if d:
                kinds.add(d.split()[0].lower())
            p = item.get("pos")
            if pos is None and p:
                pos = (float(p.get("x", 0.0)), float(p.get("y", 0.0)))
            # For edge violations the FIRST item is often the Edge.Cuts /
            # NPTH-pad object, whose anchor can sit a far corner away from
            # the offending copper (quickfeather: a 58mm gr_line anchored
            # 24mm from the flagged track) -- proximity pairing against
            # check_drc's copper-anchored items then never matches. Prefer
            # the position of the COPPER item (the one carrying a [net]).
            if copper_pos is None and p and "[" in d and "]" in d:
                copper_pos = (float(p.get("x", 0.0)), float(p.get("y", 0.0)))
        if vtype in EDGE_KICAD_TYPES and copper_pos is not None:
            pos = copper_pos
        out.append({"type": vtype, "nets": frozenset(nets),
                    "pos": pos or (0.0, 0.0),
                    "desc": v.get("description", ""),
                    "kinds": tuple(sorted(kinds))})
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
    # _staged_copy forces copper_edge_clearance severity to "error" on the
    # kicad-cli side (#441: authored-"ignore" boards must not hide router copper
    # run to the milled edge). Mirror that here or every edge item on an
    # ignore-authored board (sofle_pico) grades one-sided as kicad_only -- and
    # check_drc's accepted-edge publications (pad-covered, quantization-margin,
    # #448 npth-slot) never happen, so the reconciler can't subtract them.
    kw["respect_edge_severity"] = False
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
        # check_drc publishes 'accepted' edge items (a track covered by an
        # edge-exempt pad -- no new edge copper): not a check_drc failure, but the
        # matching kicad copper_edge_clearance finding must be dropped, not alarmed
        # on as a false negative. Carry the flag through for the reconciler.
        if v.get("accepted"):
            item["accepted"] = v["accepted"]
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


def _drop_kicad_covered_by_accepted(kicad, cd_accepted):
    """Drop kicad copper_edge_clearance items that match an ACCEPTED check_drc edge
    item -- a track whose edge copper is covered by an edge-exempt pad (check_drc's
    authority; ottercast R7.2). Respecting check_drc's accept here keeps the item out
    of kicad_only (the false-negative alarm) instead of double-reporting a
    pad-placement condition no routing can fix. Returns (remaining_kicad, n_dropped)."""
    acc = [c for c in cd_accepted if c.get("type") in EDGE_CD_TYPES]
    if not acc:
        return kicad, 0
    c_used, keep, dropped = set(), [], 0
    for kv in kicad:
        if kv.get("type") not in EDGE_KICAD_TYPES:
            keep.append(kv)
            continue
        best = None
        for i, cv in enumerate(acc):
            if i in c_used or not _edge_net_ok(kv, cv):
                continue
            d = _edge_dist(kv, cv)
            if d <= EDGE_MATCH_RADIUS_MM and (best is None or d < best[0]):
                best = (d, i)
        if best is not None:
            c_used.add(best[1])
            dropped += 1  # covered by an edge-exempt pad -> accepted-by-design
        else:
            keep.append(kv)
    return keep, dropped


# --- #408: accept-by-design edge-clearance subtraction -----------------------
def _net_tie_accepted_pairs(board: str):
    """[(frozenset({netA, netB}), (x, y))] -- net pairs a footprint's
    net_tie_pad_groups deliberately shorts, with the footprint position.

    KiCad reports the Kelvin-shunt escape contact as shorting_items/clearance
    even on the HUMAN-routed originals (cynthion ships 4 such items, one per
    shunt) -- an enclosed sense tab cannot be exited without touching its
    partner pad, and the designer accepts it. Grading treats tie-pair items AT
    the tie footprint as accepted-by-design (the #408 pattern); the positional
    gate keeps genuine track-track shorts between the same nets elsewhere on
    the board flagged."""
    try:
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(board)
    except Exception:
        return []
    pairs = []
    for fp in pcb.footprints.values():
        if not getattr(fp, 'net_tie_groups', None):
            continue
        by_num = {}
        for p in fp.pads:
            by_num.setdefault(p.pad_number, []).append(p)
        for group in fp.net_tie_groups:
            members = [p for num in group for p in by_num.get(num, [])]
            names = {p.net_name for p in members if p.net_id != 0 and p.net_name}
            if len(names) >= 2:
                for a in names:
                    for b in names:
                        if a < b:
                            pairs.append((frozenset((a, b)), (fp.x, fp.y)))
    return pairs


_NET_TIE_ACCEPT_RADIUS_MM = 3.0


def _drop_net_tie_accepted(items, tie_pairs):
    """Drop accepted net-tie contact items: type shorting_items / clearance /
    pad-segment whose net pair is a tie pair and whose anchor lies within
    _NET_TIE_ACCEPT_RADIUS_MM of the tie footprint. Returns (kept, dropped)."""
    if not tie_pairs:
        return items, 0
    kept, dropped = [], 0
    for v in items:
        hit = False
        if len(v.get("nets") or ()) == 2:
            for pair, (fx, fy) in tie_pairs:
                if v["nets"] == pair and \
                        math.hypot(v["pos"][0] - fx, v["pos"][1] - fy) <= _NET_TIE_ACCEPT_RADIUS_MM:
                    hit = True
                    break
        if hit:
            dropped += 1
        else:
            kept.append(v)
    return kept, dropped


def _web_min_connection(cfg: dict):
    """The min-copper-web width (mm) a board should be graded at (#406), from
    its .kicad_pro: an author-set `min_connection` is a real design rule and
    wins; otherwise the project's `min_track_width` (the post-route ledger
    floors it at the smallest object on the board, so the graded condition is
    "a copper web narrower than the narrowest intentional track"). None when
    neither is recorded -- connection_width is then NOT graded (KiCad's
    default min_connection is 0 = checker off), and the caller reports None
    rather than a fake clean 0."""
    try:
        rules = cfg.get("board", {}).get("design_settings", {}).get("rules", {})
        for key in ("min_connection", "min_track_width"):
            try:
                v = float(rules.get(key))
            except (TypeError, ValueError):
                continue
            if v > 0:
                return v
    except AttributeError:
        pass
    return None


def _staged_copy(board: str, clearance: float):
    """Copy board + sibling .kicad_pro into a temp dir, staged so KiCad grades at
    the routed floor. `clearance` here is the caller's RESOLVED grading clearance,
    which compare_board_data now sets to the board's OWN recorded floor (#439), so
    the Default-class equalization below is a no-op on routed output (the recorded
    Default class already IS the floor) -- it only bites a stock/project-less board.

    NON-Default classes are left untouched because the #439 writeback already
    CLAMPED them to the routed floor in the .kicad_pro (clamp_nondefault_netclasses,
    default True), so KiCad reads back the routed values directly and grades each
    cross-class pair at its true max(A,B). (Pre-#439 this relied on PR392's
    "router respects non-Default classes"; post-#439 the writeback does the clamping.)

    This function still does the useful, board-derived work: skip the slow silk DRC
    provider, stage the #406 min-connection web check, and pin an ABSENT edge rule to
    the fab floor. The clearance forcings are legacy and mostly inert now that the
    caller passes the recorded floor."""
    import shutil
    clearance = float(clearance)
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
    # Edge (#338/#295/#441 class): copper-to-edge is graded at max(recorded rule,
    # clearance, 0.20 fab copper-to-edge floor). Two config artifacts this fixes:
    #  * ABSENT key -- kicad-cli falls back to its built-in 0.5mm default while
    #    check_drc falls back to the routed clearance (openstint: a lost project
    #    manufactured 11 phantom items). Pin it so both engines grade alike.
    #  * RECORDED sub-fab key -- 80/184 corpus boards declare < 0.20 (32 at 0.0);
    #    grading at that tiny value masks copper run to the milled edge (#441). It
    #    is NOT a legitimate design constraint below what the fab can make, so it is
    #    pinned UP to the fab floor -- matching the routers, which now enforce
    #    max(cli, board rule, 0.20) to the edge (effective_board_edge_clearance).
    # A recorded rule ABOVE the fab floor (e.g. 0.5) is preserved as a real design
    # constraint via the max().
    from fix_kicad_drc_settings import fab_edge_floor
    _fab_edge = fab_edge_floor(board)
    _rec_edge = _f(rules.get("min_copper_edge_clearance"), None)
    rules["min_copper_edge_clearance"] = max(
        _rec_edge if _rec_edge is not None else clearance, _fab_edge)
    # Silk checks are OUT OF SCOPE (results are filtered to copper classes)
    # but DRC_TEST_PROVIDER_SILK_CLEARANCE dominates wall time on art-heavy
    # boards -- lily58's keyboard legends made kicad-cli spin >10 MINUTES at
    # 100% CPU in that one provider (#333, confirmed by sampling). Severity
    # "ignore" makes the DRC engine skip the provider entirely.
    sev = cfg["board"]["design_settings"].setdefault("rule_severities", {})
    for k in ("silk_over_copper", "silk_overlap", "silk_edge_clearance"):
        sev[k] = "ignore"
    # #441: force copper_edge_clearance ON. A board may author it "ignore" (many
    # corpus boards do -- lpddr4_testbed's unrouted input ignores it), which makes
    # kicad-cli skip the edge provider entirely -> the wave undercounts edge defects
    # AND, worse, grades the UNROUTED baseline (ignore) differently from the routed
    # final (error), so baseline subtraction spuriously attributes the whole delta
    # to the router. Staging both sides at "error" restores symmetry and surfaces
    # real router copper run to the milled edge (graded at the pinned floor above).
    sev["copper_edge_clearance"] = "error"
    # Min copper web (#406): KiCad's connection_width checker is OFF by default
    # (min_connection 0) and warning-severity, so routed-copper micro-webs were
    # invisible to every automated consumer in the repo -- the corpus gave NO
    # signal on the class. Same config-artifact family as the #338 edge rule:
    # stage the constraint (author-set min_connection kept, else the project's
    # min_track_width; see _web_min_connection) and lift the severity to error
    # so the --severity-error run reports it. Items are graded SEPARATELY from
    # the copper-clearance classes (KICAD_WEB_TYPES) -- check_drc has no
    # counterpart to match against.
    web_min = _web_min_connection(cfg)
    if web_min is not None:
        rules["min_connection"] = web_min
        sev["connection_width"] = "error"
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
                       baseline: str = None):
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

    Returns a dict (numeric grade + item lists + verdict for the printer), or
    None if kicad-cli could not grade the board (caller degrades to Nones)."""
    label = label or os.path.basename(board)
    # #439: grade at the board's OWN recorded routed floor (the Default net-class
    # clearance the clearance ledger + writeback record in the sibling .kicad_pro --
    # always <= the manifest --clearance because the writeback only-lowers), NOT the
    # passed manifest clearance. A board may legitimately fine-tap SOME connections
    # below the nominal --clearance; grading at the manifest ceiling manufactures
    # phantom sub-clearance items on that copper (neo6502: 499 phantom at 0.127 vs 0
    # at the recorded 0.1; olimex_lora: 125 vs 36). The passed value is only a
    # FALLBACK for a final shipped without a recorded floor (project-less board,
    # #217; stock/minimal project, the tigard class). Reassigning it here threads the
    # recorded floor to EVERY kicad_items_for / run_check_drc / web_min call below --
    # finals AND baseline -- so the two engines agree and baseline subtraction (#405)
    # lines up at the same floor.
    recorded = _pro_clearance(board)
    if recorded is not None:
        clearance = recorded
    kicad, err = kicad_items_for(board, clearance)
    if err:
        return {"board": label, "skip": err}
    # Min copper web (#406): split the web class out BEFORE baseline
    # subtraction and matching -- the positional subtraction pass is
    # type-blind, so pooling would let a baseline clearance item consume a
    # final web item (and vice versa); and check_drc has no web counterpart,
    # so matching would orphan every item into kicad_only (the false-negative
    # alarm channel). web_min mirrors the _staged_copy condition: None =
    # connection_width was NOT graded (no staged run / no recorded floor),
    # reported as None rather than a fake clean 0.
    web_min = None
    if clearance:
        pro = os.path.splitext(board)[0] + ".kicad_pro"
        try:
            web_min = _web_min_connection(
                json.load(open(pro)) if os.path.exists(pro) else {})
        except Exception:  # noqa: BLE001 -- unreadable project = not graded
            web_min = None
    web_items = [v for v in kicad if v["type"] in KICAD_WEB_TYPES]
    kicad = [v for v in kicad if v["type"] not in KICAD_WEB_TYPES]
    pre = web_pre = 0
    if baseline and os.path.exists(baseline):
        base_items, berr = kicad_items_for(baseline, clearance)
        if not berr and base_items:
            base_web = [v for v in base_items if v["type"] in KICAD_WEB_TYPES]
            base_items = [v for v in base_items if v["type"] not in KICAD_WEB_TYPES]
            kicad, pre = _subtract_baseline(kicad, base_items)
            web_items, web_pre = _subtract_baseline(web_items, base_web)
    cd = run_check_drc(board, clearance, netclasses=not bool(clearance))
    # check_drc 'accepted' edge items (track covered by an edge-exempt pad): not
    # check_drc failures -- split them out of the counted list, and use them to drop
    # the matching kicad copper_edge_clearance finding below (respect check_drc's
    # authority instead of alarming it as a false negative).
    cd_accepted = [c for c in cd if c.get("accepted")]
    cd = [c for c in cd if not c.get("accepted")]
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
    kicad_intentional = checkdrc_intentional = 0
    # Static footprint-geometry conditions (#450 kbic65): a hole_clearance
    # item whose participants are ALL pads is placement/footprint design
    # (overlapping alternative-layout switch footprints: a PTH pad's copper
    # over the neighbor variant's NPTH mounting hole) -- pad copper and pad
    # holes are footprint geometry. Baseline subtraction normally removes
    # them, EXCEPT when (remove_unused_layers) hid the pad's inner-layer
    # copper on the UNROUTED input: connecting the pad materializes the
    # copper, so the item exists only on the routed board while remaining
    # 100% footprint geometry. Drop them as pre-existing design conditions.
    # Scoped to hole_clearance: all-pad clearance/shorting items stay visible
    # (the cap optimizer MOVES passives, so those can be chain-attributable).
    _pad_kinds = {"pth", "npth", "pad"}
    _static_pad = [v for v in kicad
                   if v["type"] == "hole_clearance"
                   and v.get("kinds") and set(v["kinds"]) <= _pad_kinds]
    if _static_pad:
        kicad = [v for v in kicad if v not in _static_pad]
        pre += len(_static_pad)
        print(f"    ({len(_static_pad)} all-pad footprint-geometry item(s) "
              f"dropped as pre-existing design conditions)")
    # Accept-by-design: drop kicad edge findings covered by an edge-exempt pad
    # (check_drc published them as 'accepted'), symmetric with check_drc having
    # already excluded them from its count.
    if cd_accepted:
        kicad, _k_acc = _drop_kicad_covered_by_accepted(kicad, cd_accepted)
        kicad_intentional += _k_acc
        checkdrc_intentional += len(cd_accepted)
    # Net-tie accepted contact (Kelvin shunts): tie-pair shorting/clearance
    # items AT the tie footprint are accepted-by-design on human-routed
    # originals too -- drop from BOTH engines before matching (board-derived,
    # no router emission needed).
    tie_pairs = _net_tie_accepted_pairs(board)
    if tie_pairs:
        kicad, k_tie = _drop_net_tie_accepted(kicad, tie_pairs)
        cd, c_tie = _drop_net_tie_accepted(cd, tie_pairs)
        kicad_intentional += k_tie
        checkdrc_intentional += c_tie
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
            # #406 min copper web: None = not graded (no recorded floor).
            "kicad_connection_width": (len(web_items) if web_min is not None
                                       else None),
            "connection_width_min": web_min,
            "connection_width_preexisting": web_pre,
            "connection_width_items": web_items,
            "kicad_only_items": kicad_only, "checkdrc_only_items": cd_only}


# Summary keys the CLI callers persist (item lists + verdict are print-only).
_SUMMARY_KEYS = ("board", "kicad", "kicad_preexisting", "check_drc",
                 "kicad_intentional_edge", "checkdrc_intentional_edge",
                 "matched", "kicad_only", "checkdrc_only",
                 "pairs_kicad_only", "pairs_checkdrc_only",
                 "kicad_connection_width", "connection_width_min")


def compare_board(board: str, label: str = None, clearance: float = None,
                  baseline: str = None):
    """Thin CLI wrapper over compare_board_data: print the per-board line +
    divergence detail, return the summary-key subset (schema unchanged)."""
    label = label or os.path.basename(board)
    data = compare_board_data(board, label=label, clearance=clearance,
                              baseline=baseline)
    if data is None or "skip" in (data or {}):
        print(f"{label}: SKIP ({data.get('skip') if data else 'no result'})")
        return None
    pre_note = f" (+{data['kicad_preexisting']} pre-existing on input, subtracted)" if data["kicad_preexisting"] else ""
    cd_pre_note = f" (+{data['checkdrc_preexisting']} pre-existing, subtracted)" if data["checkdrc_preexisting"] else ""
    # #408: accepted-by-design edge items removed from both engines.
    ie = data.get("kicad_intentional_edge", 0) or data.get("checkdrc_intentional_edge", 0)
    ie_note = (f" [#408: -{data.get('kicad_intentional_edge', 0)} kicad / "
               f"-{data.get('checkdrc_intentional_edge', 0)} check_drc intentional edge]") if ie else ""
    # #406 min copper web: separate class, not matched against check_drc.
    cw = data.get("kicad_connection_width")
    cw_min = data.get("connection_width_min")
    cw_pre = data.get("connection_width_preexisting", 0)
    cw_pre_note = f" (+{cw_pre} pre-existing, subtracted)" if cw_pre else ""
    cw_note = (f" connwidth={cw}@<{cw_min}mm{cw_pre_note}" if cw is not None
               else " connwidth=not-graded")
    print(f"{label}: kicad={data['kicad']}{pre_note} check_drc={data['check_drc']}{cd_pre_note} "
          f"matched={data['matched']} kicad_only={data['kicad_only']} checkdrc_only={data['checkdrc_only']}{ie_note} | "
          f"net-pairs: both={data['pairs_both']} kicad_only={data['pairs_kicad_only']} "
          f"checkdrc_only={data['pairs_checkdrc_only']} |{cw_note}  {data['verdict']}")
    for kv in data["kicad_only_items"]:
        print(f"    KICAD-ONLY  {kv['type']:16s} {sorted(kv['nets'])} @ {kv['pos']}  {kv.get('desc','')[:60]}")
    for cv in data["checkdrc_only_items"]:
        print(f"    CHECKDRC-ONLY {cv['type']:16s} {sorted(cv['nets'])} @ {cv['pos']}")
    for wv in data.get("connection_width_items", []):
        print(f"    CONNWIDTH   {'/'.join(wv.get('kinds', ())) or '?':16s} "
              f"{sorted(wv['nets'])} @ {wv['pos']}  {wv.get('desc', '')[:60]}")
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
    args = ap.parse_args()
    boards = list(args.boards)
    jobs = [(b, None) for b in boards]
    for wave in args.wave:
        for e in json.load(open(os.path.join(wave, "summary.json"))):
            if e.get("final"):
                p = os.path.join(wave, e["board"], os.path.basename(e["final"]))
                if os.path.exists(p):
                    clr = e.get("clearance")
                    jobs.append((p, float(clr) if clr else None))
    rows = [r for b, clr in jobs
            if (r := compare_board(b, label=None, clearance=clr,
                                   baseline=args.baseline))]
    div = [r for r in rows if r["kicad_only"] or r["checkdrc_only"]]
    print(f"\n{len(rows)} boards compared: {len(rows) - len(div)} consistent, "
          f"{len(div)} diverged "
          f"(kicad_only total {sum(r['kicad_only'] for r in rows)}, "
          f"checkdrc_only total {sum(r['checkdrc_only'] for r in rows)})")
    # #406 min copper web -- graded on boards with a recorded floor only.
    cw_rows = [r for r in rows if r.get("kicad_connection_width") is not None]
    print(f"connection_width: {sum(r['kicad_connection_width'] for r in cw_rows)} "
          f"item(s) on {len(cw_rows)} graded board(s) "
          f"({len(rows) - len(cw_rows)} not graded: no recorded min floor)")
    return 1 if div else 0


if __name__ == "__main__":
    sys.exit(main())
