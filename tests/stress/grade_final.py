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

    try:
        o = subprocess.run(["python3", "-X", "utf8", f"{REPO}/check_drc.py", fp, "-c", str(clr)],
                           capture_output=True, text=True, timeout=1200).stdout
        m = re.search(r"FOUND (\d+) DRC", o)
        grade["drc"] = int(m.group(1)) if m else (0 if "NO DRC" in o else None)
        grade["by_type"] = dict((k.strip(), int(v)) for k, v in
                                re.findall(r"^([A-Z][A-Z -]+?) violations \((\d+)\):", o, re.M))
    except Exception as e:
        grade["drc"] = None; grade["drc_err"] = str(e)[:100]
    try:
        o = subprocess.run(["python3", "-X", "utf8", f"{REPO}/check_connected.py", fp],
                           capture_output=True, text=True, timeout=1200).stdout
        grade["fully_connected"] = "ALL NETS FULLY CONNECTED" in o
    except Exception as e:
        grade["fully_connected"] = None; grade["conn_err"] = str(e)[:100]

    agent = (d.get("drc") or {}).get("final_violations")
    grade["agent_drc"] = agent
    grade["misgrade"] = bool(isinstance(agent, int) and isinstance(grade.get("drc"), int)
                             and abs(grade["drc"] - agent) >= 10)
    json.dump(grade, open(os.path.join(rundir, "authoritative_grade.json"), "w"), indent=1)
    tag = f"  *** MISGRADE: agent={agent} real={grade.get('drc')} ***" if grade["misgrade"] else ""
    print(f"[grade_final] {fb}: DRC={grade.get('drc')} conn={grade.get('fully_connected')} "
          f"(agent reported {agent}){tag}")


if __name__ == "__main__":
    main()
