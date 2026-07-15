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
        out.append({"type": v["type"], "nets": nets, "pos": pos or (0.0, 0.0)})
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


def _staged_copy(board: str, clearance: float):
    clearance = float(clearance)
    """Copy board + sibling .kicad_pro into a temp dir with the Default
    net-class clearance forced to `clearance`, so KiCad grades at the SAME
    constraint check_drc uses (the routed clearance) instead of the board's
    original design netclass -- grading stricter manufactures phantom
    clearance items and drowns the real signal."""
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
        if "clearance" in c:
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


def compare_board(board: str, label: str = None, clearance: float = None,
                  baseline: str = None):
    """`baseline` = the UNROUTED input board (#326/#338 accounting): its kicad
    items -- pre-existing design conditions like edge-connector pads inside
    the board's own copper_edge rule -- are matched against the final's and
    reported separately, so `kicad_only` reflects ROUTER-introduced items."""
    label = label or os.path.basename(board)
    if clearance is None:
        clearance = _pro_clearance(board)
    kicad, err = kicad_items_for(board, clearance)
    if err:
        print(f"{label}: SKIP ({err})")
        return None
    pre = 0
    if baseline and os.path.exists(baseline):
        base_items, berr = kicad_items_for(baseline, clearance)
        if not berr and base_items:
            matched_pre, base_rest, kicad = match(base_items, kicad)
            pre = len(matched_pre)
            # Second pass, positional-drift tolerant: kicad anchors the same
            # design condition at different instance positions run-to-run
            # (openstint's pre-routed /A- input copper matched 6 of 8). An
            # unmatched final item whose (type, nets) still has an unmatched
            # BASELINE twin is the same pre-existing condition.
            pool = {}
            for bv in base_rest:
                pool[(bv["type"], bv["nets"])] = pool.get((bv["type"], bv["nets"]), 0) + 1
            still = []
            for kv in kicad:
                key = (kv["type"], kv["nets"])
                if pool.get(key):
                    pool[key] -= 1
                    pre += 1
                else:
                    still.append(kv)
            kicad = still
    cd = run_check_drc(board, clearance, netclasses=not bool(clearance))
    # Symmetric baseline subtraction (#405): the kicad side above drops
    # pre-existing (unrouted-input) conditions, but check_drc's items were left
    # un-subtracted -- so a pre-existing condition check_drc grades but kicad
    # does not (e.g. vfo_ctrl's 23 FG chassis-ground segments already <edge on
    # the bare board) surfaced as phantom checkdrc_only. Subtract the input's
    # own check_drc items too, mirroring the kicad two-pass (positional match +
    # type/net twin), so both engines are graded on router-attributable copper.
    cd_pre = 0
    if baseline and os.path.exists(baseline):
        try:
            cd_base = run_check_drc(baseline, clearance, netclasses=not bool(clearance))
        except Exception:  # noqa: BLE001 -- best-effort, tolerate a bad input
            cd_base = None
        if cd_base:
            m_pre, cd_base_rest, cd = match(cd_base, cd)   # positional (identical template copper => dist 0)
            cd_pre = len(m_pre)
            cpool = {}
            for bv in cd_base_rest:
                cpool[(bv["type"], bv["nets"])] = cpool.get((bv["type"], bv["nets"]), 0) + 1
            keep = []
            for cv in cd:
                key = (cv["type"], cv["nets"])
                if cpool.get(key):
                    cpool[key] -= 1
                    cd_pre += 1
                else:
                    keep.append(cv)
            cd = keep
    matched, kicad_only, cd_only = match(kicad, cd)
    # NET-PAIR agreement: the engines report at different granularities
    # (check_drc: one violation per offending segment; kicad: grouped item
    # pairs), so the honest consistency metric is the SET of net pairs each
    # engine implicates. Instance-level diffs then refine within agreed pairs.
    kpairs = {v["nets"] for v in kicad if len(v["nets"]) == 2}
    cpairs = {v["nets"] for v in cd if len(v["nets"]) == 2}
    pk_only, pc_only = kpairs - cpairs, cpairs - kpairs
    verdict = ("CONSISTENT" if not kicad_only and not cd_only else
               "PAIR-CONSISTENT" if not pk_only and not pc_only else "DIVERGED")
    pre_note = f" (+{pre} pre-existing on input, subtracted)" if pre else ""
    cd_pre_note = f" (+{cd_pre} pre-existing, subtracted)" if cd_pre else ""
    print(f"{label}: kicad={len(kicad)}{pre_note} check_drc={len(cd)}{cd_pre_note} matched={len(matched)} "
          f"kicad_only={len(kicad_only)} checkdrc_only={len(cd_only)} | "
          f"net-pairs: both={len(kpairs & cpairs)} kicad_only={len(pk_only)} "
          f"checkdrc_only={len(pc_only)}  {verdict}")
    for kv in kicad_only:
        print(f"    KICAD-ONLY  {kv['type']:16s} {sorted(kv['nets'])} @ {kv['pos']}  {kv['desc'][:60]}")
    for cv in cd_only:
        print(f"    CHECKDRC-ONLY {cv['type']:16s} {sorted(cv['nets'])} @ {cv['pos']}")
    return {"board": label, "kicad": len(kicad), "kicad_preexisting": pre,
            "check_drc": len(cd),
            "matched": len(matched), "kicad_only": len(kicad_only),
            "checkdrc_only": len(cd_only),
            "pairs_kicad_only": len(pk_only), "pairs_checkdrc_only": len(pc_only)}


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
    return 1 if div else 0


if __name__ == "__main__":
    sys.exit(main())
