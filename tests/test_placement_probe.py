#!/usr/bin/env python3
"""Opt-in: does a placement change actually ROUTE better?

`test_placement_ab.py` grades placement with placement metrics, independently
re-derived. That catches a term that games its own model, but every one of
those metrics is still a PROXY -- and the repo's own history says proxies
mislead here: one accepted placement improved `crossings` 85->60 and `hpwl`
602->596 while breaking a decap requirement, and run 6 routed MORE copper than
run 8 while scoring 29 plane-void crossings against run 8's 43.

So this tier routes. It is slower, which is why it is opt-in rather than part
of the A/B gate.

The scoping is the design decision worth stating: nets are chosen **causally,
not by which parts moved**. Routing "the nets touching moved parts" measures
whatever the optimizer happened to touch, which is circular -- a term that
moves nothing scores a perfect null. Instead it routes the union of

    (nets `net_affinity` flagged)  U  (the declared corridor nets)

which is the set the CHANGE CLAIMS to help, fixed before either board is
routed, and identical on both. If those nets do not route better, the signal is
not predictive and the change should not ship, whatever the proxies said.

Usage:
    python3 -X utf8 tests/test_placement_probe.py --off A.kicad_pcb \\
        --on B.kicad_pcb [--intent fp.json] [--extra-nets 'SDRAM_A*']
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def causal_nets(board, intent_path=None, extra=None):
    """The nets the change claims to help, as glob-free exact names.

    Derived from the OFF board only, so the ON board cannot influence which
    nets it is judged on.
    """
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    names = set(extra or ())
    if not intent_path:
        return sorted(names)
    pcb = parse_kicad_pcb(board)
    intent = floorplan.load_intent(intent_path)
    result = floorplan.grade(intent, pcb, board, with_health=True)
    health = result.health or {}
    for row in (health.get('net_affinity') or []):
        if row.get('net'):
            names.add(row['net'])
    # Corridor nets: the buses a corridor term claims to unclutter.
    from net_queries import matches_net_filter
    for spec in ((intent.health or {}).get('bus_corridors') or []):
        pats = list(spec.get('nets') or ())
        if not pats:
            continue
        for nid, n in pcb.nets.items():
            if nid > 0 and n.name and matches_net_filter(n.name, pats):
                names.add(n.name)
    return sorted(names)


def probe(board, nets, workdir, route_args=None):
    """Route `nets` on `board` and return the connectivity/DRC verdict."""
    from converge import scoped_route
    out = os.path.join(workdir, os.path.basename(board).replace(
        '.kicad_pcb', '_probe.kicad_pcb'))
    return scoped_route(board, nets, out=out,
                        extra_args=list(route_args or ()))


def main(argv=None):
    # This is an OPT-IN two-board A/B driver, not a self-running test: --off
    # and --on are required, so a bare `test_placement_probe.py` (which is how
    # tests/run_all.py invokes every test_*.py) can only ever exit 2 on
    # argparse. Skip cleanly instead, the same way the KiCad-python gates skip
    # when their interpreter is absent -- a red that no change can fix teaches
    # the reader to ignore the failure list.
    if not (sys.argv[1:] if argv is None else list(argv)):
        print("SKIP: opt-in A/B probe -- needs --off BOARD --on BOARD "
              "(it ROUTES both boards; see CLAUDE.md)")
        return 0
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--off', required=True, help='the baseline board')
    p.add_argument('--on', required=True, help='the changed board')
    p.add_argument('--intent', default=None,
                   help='floorplan intent; without it, only --extra-nets '
                        'scope the probe')
    p.add_argument('--extra-nets', nargs='*', default=(),
                   help='additional exact net names to include')
    # ONE string, shlex-split -- not nargs='*'. argparse refuses dash-prefixed
    # values for a list option, so `--route-args --clearance 0.2` exits 2. Same
    # trap and same fix as compare_seeds.py's forwarded args.
    p.add_argument('--route-args', default='',
                   help="forwarded to the router, IDENTICALLY on both boards, "
                        "as ONE quoted string: --route-args '--clearance 0.2'")
    p.add_argument('--workdir', default=None)
    args = p.parse_args(argv)

    import tempfile
    workdir = args.workdir or tempfile.mkdtemp(prefix='placement_probe_')
    os.makedirs(workdir, exist_ok=True)

    import shlex
    route_args = shlex.split(args.route_args) if args.route_args else []
    nets = causal_nets(args.off, args.intent, args.extra_nets)
    if not nets:
        print("no causal nets: nothing this change claims to help is "
              "identifiable. Declare health.bus_corridors, or pass "
              "--extra-nets.", file=sys.stderr)
        return 2
    print(f"probing {len(nets)} net(s) chosen from the OFF board: "
          f"{', '.join(nets[:8])}{' ...' if len(nets) > 8 else ''}\n")

    try:
        off = probe(args.off, nets, workdir, route_args)
        on = probe(args.on, nets, workdir, route_args)
    except ImportError as exc:
        print(f"probe needs converge.scoped_route: {exc}", file=sys.stderr)
        return 2

    # scoped_route returns {'summary': {...}, 'argv': ..., 'stdout_tail': ...}.
    # The counts are INSIDE 'summary'; reading the top level finds nothing and
    # reports INCONCLUSIVE on a probe that ran perfectly well.
    sa = (off or {}).get('summary') or {}
    sb = (on or {}).get('summary') or {}
    if not sa or not sb:
        print(f"OFF {json.dumps(off, sort_keys=True)[:400]}")
        print(f"ON  {json.dumps(on, sort_keys=True)[:400]}")
        print("\nINCONCLUSIVE: the router returned no summary to compare")
        return 2

    def _unconnected(s):
        return int(s.get('pad_pairs_total', 0)) - int(
            s.get('pad_pairs_connected', 0))

    # A ladder, most-decisive first, matching how the repo ranks a board:
    # connectivity before cost. Vias are the tie-break because on a 2-layer
    # board every via is a plane cut -- the damage the corridor work is about.
    LADDER = (
        ('failed nets', lambda s: int(s.get('failed', 0) or 0)),
        ('open nets', lambda s: len(s.get('open_single') or ())),
        ('unconnected pad pairs', _unconnected),
        ('vias', lambda s: int(s.get('total_vias', 0) or 0)),
    )
    print(f"{'metric':<24} {'OFF':>8} {'ON':>8}")
    verdict = None
    for label, fn in LADDER:
        a, b = fn(sa), fn(sb)
        mark = '' if a == b else ('  BETTER' if b < a else '  WORSE')
        print(f"{label:<24} {a:>8} {b:>8}{mark}")
        if verdict is None and a != b:
            verdict = (label, a, b, b < a)

    if verdict is None:
        print("\nNO CHANGE on any rung -- the placement proxies moved and the "
              "routing did not. That is a reason to doubt the term, not a "
              "pass.")
        return 0
    label, a, b, better = verdict
    print(f"\n{'BETTER' if better else 'WORSE'} on the first rung that moved "
          f"({label} {a} -> {b})")
    # Materiality. The bottom rung moving by a couple of percent is not a
    # result, and calling it one is how a rejected term gets rehabilitated by a
    # rounding difference. Measured: the corridor A/B pair probes 51 -> 50 vias
    # while the placement grade says its own signal got WORSE.
    if label == LADDER[-1][0] and a and abs(b - a) / a < 0.05:
        print(f"...but that is a {100.0 * abs(b - a) / a:.1f}% move on the "
              f"LAST rung with connectivity identical. Not a result. Judge "
              f"this change on the placement grade, and if that disagrees, "
              f"believe the disagreement.")
    if not better:
        print("do not ship this change")
    return 0 if better else 1


if __name__ == '__main__':
    sys.exit(main())
