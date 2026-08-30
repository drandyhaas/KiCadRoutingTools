#!/usr/bin/env python3
"""RELEASE GATE: obstacle ref-count integrity on a REAL board (#803).

Runs a real routing chain with KICAD_OBSTACLE_AUDIT=1 KICAD_OBSTACLE_LEDGER=1
and asserts the ref-count invariants with the REAL counts the audit prints.
This is a mandatory pre-release step -- see docs/release-pipeline.md.

WHY A REAL BOARD AND NOT A UNIT TEST. The working obstacle map is
reference-counted in Rust, and the failure that motivated this gate is a
BOOKKEEPING one that only appears when a full chain drives the map through
thousands of prepare/route/restore/rip cycles. A mocked map reproduces
whatever the author imagined; it cannot reproduce an entry being replaced by
one of nine call sites at a moment nobody modelled.

WHY glasgow_revC. It is the board that actually reproduces. Measured
2026-08-29: the fast in-repo board (splitflap_driver, 8 s) reports every
invariant BALANCED with the fix DISABLED, so a gate pointed at it would be
VACUOUS -- it would pass no matter what. glasgow_revC with the in-loop
stub-debris trim ON reports an unbalanced cache object and 30510/23699/22244
cells unaccounted for by any net cache, and 7 DRC violations; with the trim
OFF it reports none and 0. Cost is the reason it is a RELEASE gate and not a
run_all test: the chain is ~6 minutes.

SELF-CONTAINED. The chain below is run from `kicad_files/glasgow_revC.kicad_pcb`
and its `.kicad_pro` sibling, both in this repo -- no stress corpus required, so
this gate runs on a fresh clone. The board file is byte-identical to the
corpus's `boards_unrouted_set1/glasgow_revC.kicad_pcb` (verified with cmp), and
the steps are the recorded `runs_set1/glasgow_revC` manifest verbatim. The
`.kicad_pro` is REQUIRED, not decoration: it carries min_hole_to_hole 0.25, the
rule the violating via pair breaks, and routing without it resolves a looser
floor from the stock netclass (#441). `--manifest` still replays a recorded run
instead, for comparing against a corpus board.

THE INVARIANTS, and which are enforced:

  A. hard ref-counted maps return to base   ENFORCED  (blocked_cells,
     blocked_vias, source_target, free_vias, blocked_vias_small)
  B. no unbalanced cache OBJECT             ENFORCED
  C. no cells unaccounted for by any cache  ENFORCED
  D. raw add/remove op deltas cancel        RECORDED, NOT ENFORCED

D is a REAL, SEPARATE, PRE-EXISTING leak, recorded here rather than dropped so
it stays a change detector instead of folklore. It is NOT caused by the trim --
measured on glasgow_revC it is present with the trim OFF and LARGER there:

    trim ON   cells +11326 vias +3942   add_segments_list @
              phase3_routing.py:try_phase3_ripup:1117
    trim OFF  cells +19529 vias +5984   (same site) plus two further sites
              at run_phase3_tap_routing:680 (+35856/+12187, +4695/+1740)

Enforcing D today would fail every release for a bug this gate did not find and
does not describe. When it is fixed, move it to ENFORCED and delete this note.
"""
import argparse
import os
import re
import subprocess
import sys

HARD_MAPS = ("blocked_cells", "blocked_vias", "source_target", "free_vias",
             "blocked_vias_small")

ROUTE_TAIL = (
    " --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.2 --track-width 0.2"
    " --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25"
    " --board-edge-clearance 0.5"
    " --power-nets GND +3V3 +5V +1V2 /xVBUS /VCCPLL0 /VCCPLL1 /GNDPLL0 /GNDPLL1"
    " Net-(U12-Vbus) Net-(U21-Vbus) /IO_Banks/VIOA /IO_Banks/VIOB"
    " --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.4 0.4 0.4 0.4"
)


def parse_audit(text):
    """Pull the REAL numbers out of an audit/ledger log."""
    return {
        "audit_blocks": len(re.findall(r"\[OBSTACLE AUDIT", text)),
        "ledger_blocks": len(re.findall(r"\[OBSTACLE LEDGER", text)),
        # B: "UNBALANCED serial 1583 (net ?): adds-removes=1, expected 0"
        "unbalanced": re.findall(
            r"UNBALANCED serial (\d+) \(net ([^)]*)\): adds-removes=(-?\d+)", text),
        # C: "    blocked_cells: +30510 cells unaccounted for by any net cache"
        "unaccounted": [(n, int(d)) for n, d in re.findall(
            r"^\s+(\w+): ([+-]?\d+) cells unaccounted for by any net cache",
            text, re.M)],
        # A: the explicit LEAK/DESYNC header the audit prints instead of BALANCED
        "leak_headers": len(re.findall(r"LEAK/DESYNC in ref-counted", text)),
        "balanced": len(re.findall(r"BALANCED: ref-counted maps", text)),
        # D: recorded only
        "raw_ops": re.findall(
            r"^\s+cells ([+-]?\d+) vias ([+-]?\d+)\s+(\S+ @ \S+)", text, re.M),
        "repairs": sum(int(m) for m in
                       re.findall(r"\[RESIDENCY\] map repairs performed: (\d+)", text)),
    }


def report(a):
    print(f"  audit blocks: {a['audit_blocks']}   ledger blocks: {a['ledger_blocks']}")
    print(f"  BALANCED verdicts: {a['balanced']}   LEAK/DESYNC headers: {a['leak_headers']}")
    print(f"  residency map repairs performed: {a['repairs']}")
    if a["unbalanced"]:
        print(f"  UNBALANCED cache objects: {len(a['unbalanced'])}")
        for serial, net, delta in a["unbalanced"][:8]:
            print(f"    serial {serial} (net {net}): adds-removes={delta}")
    if a["unaccounted"]:
        print(f"  UNACCOUNTED cells: {len(a['unaccounted'])} entries")
        for name, delta in a["unaccounted"][:8]:
            print(f"    {name}: {delta:+d}")
    if a["raw_ops"]:
        print(f"  [recorded, invariant D not enforced] unbalanced raw-op sites:"
              f" {len(a['raw_ops'])}")
        for cells, vias, site in a["raw_ops"][:4]:
            print(f"    cells {cells} vias {vias}  {site}")


def verdict(a):
    """Return (ok, failures). Enforces A, B, C only -- see the module docstring."""
    f = []
    if a["audit_blocks"] == 0 or a["ledger_blocks"] == 0:
        # The run produced no audit at all: the gate tested NOTHING. This is a
        # BROKEN GATE, not a pass -- the repo has shipped that mistake before.
        f.append("BROKEN GATE: no audit/ledger output -- the run did not "
                 "produce the instrumentation this gate reads")
        return False, f
    if a["leak_headers"]:
        f.append(f"A: {a['leak_headers']} LEAK/DESYNC header(s) -- a hard "
                 f"ref-counted map did not return to base")
    if a["unbalanced"]:
        f.append(f"B: {len(a['unbalanced'])} unbalanced cache object(s): "
                 + ", ".join(f"serial {s} adds-removes={d}"
                             for s, _n, d in a["unbalanced"][:5]))
    hard = [(n, d) for n, d in a["unaccounted"] if n in HARD_MAPS]
    if hard:
        f.append("C: cells unaccounted for by any net cache: "
                 + ", ".join(f"{n} {d:+d}" for n, d in hard))
    # free_vias is called out separately because it is the quietest of the hard
    # maps and the easiest to wave through as "only +1". It is not cosmetic: a
    # free-via registration marks a cell as a legal via site for the net being
    # routed, and #189's _register_unblock_via ALSO drops a source/target
    # override on every layer plus an 11x11 allowed-cell block -- all
    # permissive, none blocking. A residual here means such a registration
    # outlived its route, so the map is still telling some later search that a
    # cell is open. ANY non-zero free_vias fails this gate.
    _fv = [d for n, d in a["unaccounted"] if n == "free_vias" and d]
    if _fv:
        f.append(f"C(free_vias): {max(_fv):+d} free-via cell(s) left registered "
                 f"at end of run -- a permissive override that outlived its "
                 f"route. Must be ZERO.")
    return (not f), f


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest",
                    help="replay a recorded run's redo_commands.sh instead of "
                         "the self-contained in-repo chain")
    ap.add_argument("--workdir", default=None, help="where to replay (must be writable)")
    ap.add_argument("--log", help="grade an EXISTING audit log instead of running")
    ap.add_argument("--self-test", action="store_true",
                    help="check the parser/verdict logic against known-good and "
                         "known-bad text, in milliseconds")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.log:
        text = open(args.log, errors="ignore").read()
    else:
        if args.manifest:
            text = replay(args.manifest, args.workdir)
        else:
            text = run_chain(args.workdir)      # self-contained, no corpus
        if text is None:
            return 2

    a = parse_audit(text)
    print("\n[OBSTACLE BALANCE GATE] real counts from the run:")
    report(a)
    ok, failures = verdict(a)
    print()
    if ok:
        print("PASS: invariants A (maps return to base), B (no unbalanced cache "
              "object) and C (no unaccounted cells) all hold.")
        return 0
    print("FAIL:")
    for x in failures:
        print(f"  - {x}")
    return 1


# The recorded runs_set1/glasgow_revC chain, verbatim. `{D}` is the workdir and
# `{SRC}` the staged board. Steps 7 and 8 are DELIBERATELY identical: the
# recorded run routed to the same output path twice, which re-reads the sibling
# .kicad_pro DRC floor and changes the routing. Reproducing the defect means
# reproducing that, so do not "clean up" the duplicate.
CHAIN = [
    "route_planes.py {SRC} {D}/step1_planes.kicad_pcb --nets GND +3V3"
    " --plane-layers In1.Cu In2.Cu --clearance 0.2 --hole-to-hole-clearance 0.25"
    " --board-edge-clearance 0.5 --via-size 0.5 --via-drill 0.3",

    "bga_fanout.py {D}/step1_planes.kicad_pcb --output {D}/step2_bga_fanout.kicad_pcb"
    " --component U30 --layers F.Cu In1.Cu In2.Cu B.Cu --nets * !GND !+3V3"
    " --diff-pairs /IO_Banks/Z* --diff-pair-gap 0.09 --clearance 0.09"
    " --track-width 0.11 --via-size 0.22 --via-drill 0.12 --grid-step 0.1",

    "place_fanout_clearance.py {D}/step2_bga_fanout.kicad_pcb"
    " {D}/step2b_placeclear.kicad_pcb --clearance 0.09",

    "qfn_fanout.py {D}/step2b_placeclear.kicad_pcb --output {D}/step3_qfn_fanout.kicad_pcb"
    " --component U1 --nets * !GND !+3V3 --clearance 0.09 --width 0.10 --grid-step 0.05",

    "place_fanout_clearance.py {D}/step3_qfn_fanout.kicad_pcb"
    " {D}/step3b_placeclear.kicad_pcb --clearance 0.09",

    "route_diff.py {D}/step3b_placeclear.kicad_pcb {D}/step4_diffpairs.kicad_pcb"
    " /IO_Banks/Z* /USB_* Net-(U12-Vin*) Net-(U21-Vin*)"
    " --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1"
    " --clearance 0.1 --no-gnd-vias",

    "route.py {D}/step4_diffpairs.kicad_pcb {D}/step5_route.kicad_pcb *" + ROUTE_TAIL,
    "route.py {D}/step4_diffpairs.kicad_pcb {D}/step5_route.kicad_pcb *" + ROUTE_TAIL,
    "route.py {D}/step5_route.kicad_pcb {D}/step5b_retry.kicad_pcb *" + ROUTE_TAIL,
]


def run_chain(workdir):
    """Run the self-contained glasgow chain; return the combined log text."""
    import shutil
    import tempfile
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    board = os.path.join(repo, "kicad_files", "glasgow_revC.kicad_pcb")
    proj = os.path.join(repo, "kicad_files", "glasgow_revC.kicad_pro")
    for f in (board, proj):
        if not os.path.isfile(f) or os.path.getsize(f) == 0:
            print(f"ABORT: missing or empty {f} -- this gate needs the in-repo "
                  f"board AND its .kicad_pro (#441)")
            return None
    d = workdir or tempfile.mkdtemp(prefix="obsbal_")
    os.makedirs(d, exist_ok=True)
    src = os.path.join(d, "glasgow_revC.kicad_pcb")
    shutil.copy2(board, src)
    shutil.copy2(proj, os.path.join(d, "glasgow_revC.kicad_pro"))
    env = dict(os.environ,
               KICAD_OBSTACLE_AUDIT="1", KICAD_OBSTACLE_LEDGER="1",
               KICAD_RESIDENCY_STATS="1")
    out = []
    print(f"running the in-repo glasgow chain in {d}  (~6 min, {len(CHAIN)} steps)")
    for i, tmpl in enumerate(CHAIN, 1):
        argv = tmpl.format(D=d, SRC=src).split()
        argv = [sys.executable, "-u", "-X", "utf8",
                os.path.join(repo, "py_router", argv[0])] + argv[1:]
        print(f"  [{i}/{len(CHAIN)}] {os.path.basename(argv[4])}", flush=True)
        p = subprocess.run(argv, cwd=d, env=env, capture_output=True, text=True)
        out.append(p.stdout + p.stderr)
        if p.returncode != 0:
            # Report the REASON: a chain that dies before routing tests nothing.
            tail = (p.stdout + p.stderr).strip().splitlines()[-6:]
            print(f"ABORT: step {i} ({os.path.basename(argv[4])}) exited "
                  f"{p.returncode}:")
            for t in tail:
                print("    " + t)
            open(os.path.join(d, "audit.log"), "w").write("".join(out))
            return None
    text = "".join(out)
    log = os.path.join(d, "audit.log")
    open(log, "w").write(text)
    print(f"  log: {log}")
    return text


def replay(manifest, workdir):
    import shutil
    import tempfile
    src = os.path.dirname(os.path.abspath(manifest))
    d = workdir or tempfile.mkdtemp(prefix="obsbal_")
    os.makedirs(d, exist_ok=True)
    # Rewrite the manifest's absolute paths into the workdir. The recorded
    # manifest writes into its OWN run directory; replaying it unrewritten
    # OVERWRITES the recorded corpus board (measured -- it cost 7 intermediate
    # boards of a graded run).
    text = open(manifest, errors="ignore").read()
    rewritten = text.replace(src.rstrip("/") + "/", d.rstrip("/") + "/")
    if src.rstrip("/") + "/" in rewritten:
        print(f"ABORT: could not rewrite every path out of {src} -- refusing to "
              f"run, it would overwrite the recorded run")
        return None
    script = os.path.join(d, "redo_commands.sh")
    open(script, "w").write(rewritten)
    env = dict(os.environ,
               KICAD_OBSTACLE_AUDIT="1", KICAD_OBSTACLE_LEDGER="1",
               KICAD_RESIDENCY_STATS="1")
    print(f"replaying {manifest}\n  -> {d}  (~6 min)")
    p = subprocess.run(["bash", script], cwd=d, env=env,
                       capture_output=True, text=True)
    log = os.path.join(d, "audit.log")
    open(log, "w").write(p.stdout + p.stderr)
    print(f"  log: {log}")
    return p.stdout + p.stderr


def self_test():
    """The gate's own logic, checked BOTH directions in milliseconds."""
    good = """
============================================================
[OBSTACLE AUDIT] working - sum(caches) vs base (226 net caches removed)
  BALANCED: ref-counted maps (blocked_cells/blocked_vias/source_target/free_vias) return exactly to base.
[OBSTACLE LEDGER] 1836 events, 965 cache objects touched the working map
  cache-object ledger BALANCED (leak, if any, is in raw ops below)
    cells +19529 vias +5984  add_segments_list @ phase3_routing.py:try_phase3_ripup:1117
[RESIDENCY] map repairs performed: 3
"""
    bad = """
[OBSTACLE AUDIT] working - sum(caches) vs base (226 net caches removed)
  LEAK/DESYNC in ref-counted obstacle maps:
    blocked_cells: +30510 cells unaccounted for by any net cache
    blocked_vias: +23699 cells unaccounted for by any net cache
[OBSTACLE LEDGER] 1888 events, 993 cache objects touched the working map
  UNBALANCED serial 1583 (net ?): adds-removes=1, expected 0 (resident=False)
"""
    ga, ba = parse_audit(good), parse_audit(bad)
    ok_g, f_g = verdict(ga)
    ok_b, f_b = verdict(ba)
    fails = []
    if not ok_g:
        fails.append(f"known-GOOD text must PASS, got: {f_g}")
    if ok_b:
        fails.append("known-BAD text must FAIL, but it passed")
    if len(ba["unbalanced"]) != 1:
        fails.append(f"parser missed the unbalanced object: {ba['unbalanced']}")
    if sorted(ba["unaccounted"]) != [("blocked_cells", 30510), ("blocked_vias", 23699)]:
        fails.append(f"parser misread the real counts: {ba['unaccounted']}")
    if ga["repairs"] != 3:
        fails.append(f"parser misread repairs: {ga['repairs']}")
    # D is RECORDED, never enforced: good text has a raw-op site and still passes.
    if not ga["raw_ops"]:
        fails.append("parser missed the recorded raw-op site")
    # An empty log is a BROKEN GATE, not a pass.
    ok_e, f_e = verdict(parse_audit(""))
    if ok_e or "BROKEN GATE" not in f_e[0]:
        fails.append("empty log must report BROKEN GATE, not pass")
    if fails:
        print("SELF-TEST FAILED:")
        for x in fails:
            print("  -", x)
        return 1
    print("self-test OK: good passes, bad fails, empty log = BROKEN GATE, "
          "real counts parsed, invariant D recorded but not enforced")
    return 0


if __name__ == "__main__":
    sys.exit(main())
