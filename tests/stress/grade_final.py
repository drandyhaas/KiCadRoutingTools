#!/usr/bin/env python3
"""Harness backstop: independently grade a stress worker's FINAL board.

Run by run_board.sh after the agent exits. Resolves the actual final output board,
grades DRC at the routed clearance + connectivity, writes authoritative_grade.json,
and flags a MISGRADE when it disagrees with the agent's self-reported
drc.final_violations. This catches a stale / pre-mutation self-grade slipping
through -- e.g. artix_dc_scm reported 0 DRC (graded step5d_reconnect) while the
final board it shipped (step5e_fix, a `--rip-existing-nets '*'` retry) was 519.

Usage: grade_final.py <rundir> <result_json>
"""
import json, os, re, subprocess, sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HERE = os.path.dirname(os.path.abspath(__file__))  # tests/stress -- the shared graders


def redo_final_and_clearance(txt):
    final, clrs = None, []
    for l in txt.splitlines():
        l = l.strip()
        if not l or l.startswith('#') or 'check_' in l:
            continue
        toks = [t.strip("'\"") for t in l.split() if t.strip("'\"").endswith('.kicad_pcb')]
        if toks:
            final = os.path.basename(toks[-1])
        clrs += [float(x) for x in re.findall(r'--clearance\s+([0-9.]+)', l)]
    return final, (min(clrs) if clrs else 0.1)


# Flag -> fix_project_for_output kwarg. The SMALLEST value any step used is the
# floor the whole board must grade at (a step routing at 0.09 leaves 0.09 copper
# everywhere it ran), so the shipped .kicad_pro must record the chain minimum.
_FLOOR_FLAGS = {
    'clearance': 'clearance',
    'hole-to-hole-clearance': 'hole_to_hole', 'hole-to-hole': 'hole_to_hole',
    'board-edge-clearance': 'edge_clearance', 'edge-clearance': 'edge_clearance',
    'track-width': 'track_width', 'via-size': 'via_diameter', 'via-drill': 'via_drill',
}


def redo_floors(txt):
    """Smallest value each DRC-floor flag took across the recorded chain, so the
    shipped board's .kicad_pro can be graded at the tightest clearance actually
    routed (not a step's looser nominal). A bare `cp <step> <final>` strands the
    sibling .kicad_pro, so the final can otherwise carry a step's higher default
    and manufacture phantom violations in KiCad (#403/#326)."""
    floors = {}
    for l in txt.splitlines():
        l = l.strip()
        if not l or l.startswith('#') or 'check_' in l:
            continue
        for flag, kw in _FLOOR_FLAGS.items():
            for m in re.findall(r'--%s\s+([0-9.]+)' % re.escape(flag), l):
                v = float(m)
                if v > 0 and (kw not in floors or v < floors[kw]):
                    floors[kw] = v
    return floors


def newest(rundir, pred):
    cands = [f for f in os.listdir(rundir) if f.endswith('.kicad_pcb') and pred(f)]
    return max(cands, key=lambda f: os.path.getmtime(os.path.join(rundir, f))) if cands else None


def main():
    rundir, result = sys.argv[1], sys.argv[2]
    man = os.path.join(rundir, "redo_commands.sh")
    redo_final, clr = redo_final_and_clearance(open(man).read()) if os.path.exists(man) else (None, 0.1)

    # Resolve the final board: results-JSON field -> newest final*.kicad_pcb ->
    # redo's last output -> newest .kicad_pcb in the dir.
    try:
        d = json.load(open(result))
    except Exception:
        d = {}
    fb = None
    for k in ("final_board", "output", "final"):
        if d.get(k):
            fb = os.path.basename(d[k]); break
    if not fb or not os.path.exists(os.path.join(rundir, fb)):
        fb = newest(rundir, lambda f: f.startswith('final')) or redo_final or newest(rundir, lambda f: True)

    grade = {"final_board": fb, "clearance": clr}
    fp = os.path.join(rundir, fb) if fb else None
    if not fp or not os.path.exists(fp):
        grade["error"] = "no final board found"
        json.dump(grade, open(os.path.join(rundir, "authoritative_grade.json"), "w"), indent=1)
        print("[grade_final] no final board found"); return

    # Carry the chain's tightest DRC floors onto the shipped board's .kicad_pro.
    # Each routing step records its floors in the OUTPUT's sibling .kicad_pro and
    # the next step inherits them (fix_project_for_output copies the input project
    # + min-merges), but a bare `cp <step> <final>` strands that file -- so the
    # delivered final can carry a later step's looser clearance and a KiCad DRC
    # reads phantom violations on legitimately fine copper. grade_final grades at
    # -c chain-min regardless, but a user opening the board would still see the
    # phantoms; write the floors so the shipped .kicad_pro matches the grade.
    # fix_project_for_output min-merges and clamps to the board's own minima, so
    # this never loosens a correct project or tightens below the real copper.
    try:
        sys.path.insert(0, REPO)
        from fix_kicad_drc_settings import fix_project_for_output
        floors = redo_floors(open(man).read()) if os.path.exists(man) else {}
        floors.setdefault('clearance', clr)
        fix_project_for_output(fp, input_pcb=fp, **floors)
    except Exception as e:
        grade["floor_writeback_err"] = str(e)[:120]

    # Grade at the board's REAL design floor, not the raw route-step --clearance.
    # `--clearance` is a #439 CEILING, so the clearance KiCad actually enforces is
    # min(--clearance, netclass) -- exactly what fix_project_for_output just wrote
    # into the sibling .kicad_pro. Grading at the bare chain-min ceiling (e.g. 0.15
    # when the board's netclass is 0.10) manufactures phantom sub-ceiling grazes on
    # copper that is legal at the real rule: KiCad grades at the netclass and shows
    # 0 (nes_rom_vomitter 156->0, molekula_enc 133->0, pcie_test_edge 86->0).
    grade_clr = clr
    try:
        from list_nets import board_default_netclass_clearance
        ncl = board_default_netclass_clearance(fp)
        if ncl and ncl > 0:
            grade_clr = min(clr, ncl)
    except Exception as e:
        grade["netclass_read_err"] = str(e)[:100]
    grade["clearance"] = grade_clr

    # Grade through the SHARED no-LLM core so this LLM path and the no-LLM path
    # (ab_replay_grade / kicad_drc_compare) grade IDENTICALLY by construction --
    # neither reimplements clearance resolution, baseline subtraction, or matching.
    # compare_board_data grades at the recorded .kicad_pro floor (#439), then
    # SYMMETRICALLY SUBTRACTS the pre-existing conditions of the UNROUTED INPUT
    # (#405) and the accept-by-design cases (net-ties, edge-exempt pads). So
    # grade["drc"] is ROUTER-ATTRIBUTABLE, not raw: a board whose only DRC is
    # pre-existing input copper (g474: 12 chassis-ground edge segments already
    # <edge on the bare board) grades 0, matching KiCad -- grade_final previously
    # reported all 12 as ours. baseline = the pristine unrouted input (first
    # .kicad_pcb token of the recorded chain), resolved by ab_replay's helper.
    sys.path.insert(0, HERE)
    base = None
    try:
        from ab_replay_grade import manifest_baseline
        if os.path.exists(man):
            base = manifest_baseline(open(man).read(), rundir, os.path.dirname(rundir),
                                     os.path.dirname(os.path.dirname(os.path.abspath(rundir))))
            if not base or not os.path.isabs(base) or not os.path.exists(base):
                base = None
    except Exception:
        base = None
    grade["baseline"] = os.path.basename(base) if base else None

    try:
        from kicad_drc_compare import compare_board_data, run_check_drc, _subtract_baseline
        data = compare_board_data(fp, clearance=grade_clr, baseline=base)
        if not data or data.get("skip"):
            raise RuntimeError((data or {}).get("skip", "compare_board_data unavailable"))
        grade["drc"] = data["check_drc"]                    # router-attributable (baseline-subtracted)
        grade["drc_kicad"] = data["kicad"]                  # kicad-cli cross-check side
        grade["drc_preexisting"] = data["checkdrc_preexisting"]
        grade["drc_verdict"] = data["verdict"]
        # by_type + raw-final of the ROUTER-ATTRIBUTABLE items, via the same helpers
        cd = run_check_drc(fp, grade_clr)
        grade["drc_final_raw"] = len(cd)
        if base:
            cd, _ = _subtract_baseline(cd, run_check_drc(base, grade_clr))
        from collections import Counter
        grade["by_type"] = dict(Counter(v.get("type", "?") for v in cd))
    except Exception as e:
        # Fallback (no kicad-cli / shared core unavailable): check_drc only at the
        # netclass floor, NO baseline subtraction -- may over-count pre-existing.
        grade["grade_core_err"] = str(e)[:150]
        try:
            o = subprocess.run(["python3", "-X", "utf8", f"{REPO}/check_drc.py", fp, "-c", str(grade_clr)],
                               capture_output=True, text=True, timeout=1200).stdout
            m = re.search(r"FOUND (\d+) DRC", o)
            grade["drc"] = int(m.group(1)) if m else (0 if "NO DRC" in o else None)
            grade["drc_final_raw"] = grade["drc"]
            grade["by_type"] = dict((k.strip(), int(v)) for k, v in
                                    re.findall(r"^([A-Z][A-Z -]+?) violations \((\d+)\):", o, re.M))
        except Exception as e2:
            grade["drc"] = None; grade["drc_err"] = str(e2)[:100]
    try:
        o = subprocess.run(["python3", "-X", "utf8", f"{REPO}/check_connected.py", fp],
                           capture_output=True, text=True, timeout=1200).stdout
        grade["fully_connected"] = "ALL NETS FULLY CONNECTED" in o
    except Exception as e:
        grade["fully_connected"] = None; grade["conn_err"] = str(e)[:100]

    # Misgrade compares LIKE FOR LIKE: the agent's final_violations is its RAW final
    # DRC count, so compare it to our raw final (drc_final_raw), not the
    # baseline-subtracted router-attributable grade["drc"].
    agent = (d.get("drc") or {}).get("final_violations")
    grade["agent_drc"] = agent
    raw = grade.get("drc_final_raw", grade.get("drc"))
    grade["misgrade"] = bool(isinstance(agent, int) and isinstance(raw, int)
                             and abs(raw - agent) >= 10)
    json.dump(grade, open(os.path.join(rundir, "authoritative_grade.json"), "w"), indent=1)
    tag = f"  *** MISGRADE: agent={agent} raw_final={raw} ***" if grade["misgrade"] else ""
    pre = grade.get("drc_preexisting")
    pre_note = f" (+{pre} pre-existing subtracted)" if pre else ""
    print(f"[grade_final] {fb}: DRC={grade.get('drc')}{pre_note} conn={grade.get('fully_connected')} "
          f"(agent reported {agent}){tag}")


if __name__ == "__main__":
    main()
