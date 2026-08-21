#!/usr/bin/env python3
"""K diverse placement candidates from one placed board, ranked and rendered.

Usage:
  python place_portfolio.py board.kicad_pcb --out-dir DIR [options]

The quench (place_optimize.py) is deterministic by design, so re-running it
never produces a different placement. This tool generates real OPTIONS: legal
seeded perturbations of the input placement (jitter / rotation variants /
block-interior swaps), each quenched with the ordinary engine, scored without
routing, pruned to a diverse slate, probe-routed at the top, and presented as
per-candidate renders plus a machine-readable portfolio.json.

Candidate 0 is always the plain quench of the input -- "keep what I have" is a
first-class outcome and the identity anchor (its numbers equal a place_optimize
run with the same knobs). Same --seed + same input => byte-identical portfolio;
--only N regenerates exactly candidate N, which is what a ledger replays.

Exit codes: 0 at least one viable candidate; 2 bad arguments; 3 the board is
not in a state placement may touch (unplaced, or already carries copper);
4 the run completed but zero candidates passed the hard gates.

See docs/placement-optimization.md (Portfolio section) for the background.
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import os
import subprocess
import sys

# REPO root (this script lives in py_placer/): subprocesses run with cwd=ROOT.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _parse_strategies(text):
    from placement.portfolio import DEFAULT_STRATEGIES
    out = tuple(s.strip() for s in text.split(',') if s.strip())
    bad = [s for s in out if s not in DEFAULT_STRATEGIES]
    if bad or not out:
        raise argparse.ArgumentTypeError(
            f"unknown strategy {', '.join(bad) or '(none)'}; "
            f"choose from {', '.join(DEFAULT_STRATEGIES)}")
    return out


def build_parser():
    import routing_defaults as defaults
    from placement.cli_gates import add_board_state_args, add_tidiness_args

    p = argparse.ArgumentParser(
        description="Placement portfolio: K diverse candidates from one seed.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python place_portfolio.py board.kicad_pcb --out-dir pf
  python place_portfolio.py board.kicad_pcb --out-dir pf --seed 7 \\
      --intent floorplan.json --ignore-nets GND VCC --route-top 0
""")
    p.add_argument("input_file", help="Input KiCad PCB file (placed, unrouted)")
    p.add_argument("--out-dir", required=True,
                   help="Directory for candidate boards, renders and "
                        "portfolio.json")
    p.add_argument("--seed", type=int, default=0,
                   help="Perturbation seed -- the ONLY source of randomness. "
                        "Same seed + same input reproduces the portfolio "
                        "byte for byte (default: 0)")
    p.add_argument("--candidates", type=int, default=12,
                   help="How many candidates to generate, baseline included "
                        "(default: 12)")
    p.add_argument("--keep", type=int, default=5,
                   help="How many diverse candidates to present, on top of "
                        "the always-present baseline (default: 5)")
    p.add_argument("--strategy", type=_parse_strategies,
                   default=('jitter', 'poses', 'swap'), metavar="LIST",
                   help="Comma list of perturbation strategies, allocated "
                        "round-robin (default: jitter,poses,swap)")
    p.add_argument("--radius", type=float, default=4.0,
                   help="Jitter amplitude in mm (default: 4.0). With the "
                        "quench's own --max-displacement this bounds total "
                        "exploration; 10mm-class values destroy corridors")
    p.add_argument("--diversity-mm", type=float, default=2.0,
                   help="Minimum pairwise pose distance among kept candidates "
                        "(default: 2.0). Below it a candidate is a near-clone "
                        "and is skipped in favour of the next-ranked one")
    p.add_argument("--rank-crossing-band", type=float, default=0.0,
                   metavar="FRAC",
                   help="Treat crossing counts within FRAC of the baseline as "
                        "TIED, so the advisory health signal (bus-corridor "
                        "crossings, corridor intrusions, displaced blocks) "
                        "decides inside the band. 0 = off (default, legacy "
                        "order). Crossings is near-injective over a real "
                        "slate, so without a band the first slot decides and "
                        "health never speaks. Try 0.02")
    p.add_argument("--corridor-weight", type=float, default=0.0,
                   metavar="W",
                   help="EXPERIMENTAL and NOT ADOPTED -- read this before "
                        "using it. Prices the LENGTH each foreign airwire cuts "
                        "through a corridor declared in the intent's "
                        "health.bus_corridors, at W per mm. 0 = off (default), "
                        "and at 0 no corridor is built at all. The A/B "
                        "(tests/test_placement_ab.py) measured it on three "
                        "boards: it does move parts, but the re-derived "
                        "bus_foreign_crossings improved on only ONE of the "
                        "three and hpwl got worse on ulx3s. The optimizer "
                        "minimises against corridors frozen at construction "
                        "while the grader re-derives them from the final poses "
                        "-- and since a corridor is defined by the bus's own "
                        "pads, moving parts moves the corridor, so the gain "
                        "does not transfer. Read the cut as a DIAGNOSTIC "
                        "instead: 'check_floorplan --intent --health' reports "
                        "cut_mm and cover per corridor")
    p.add_argument("--intent", default=None, metavar="JSON",
                   help="Floorplan intent: hard gate (error-free grade "
                        "required) plus the health signals in the rank key")
    p.add_argument("--group-by", default="auto",
                   help="Block sources for swap moves and intent grading "
                        "(comma list of kicad/sheet/netprefix/decap; 'auto' = "
                        "kicad,sheet; default: auto). NOT passed to the "
                        "quench: candidate 0 stays identical to a default "
                        "place_optimize run")
    p.add_argument("--lock", nargs="+", default=None, metavar="REF",
                   help="Reference patterns the portfolio must not perturb "
                        "(board (locked yes) refs are always honored)")
    p.add_argument("--ignore-nets", nargs="+", default=None, metavar="NET",
                   help="Net patterns excluded from airwire scoring and probe "
                        "routes (plane-routed rails)")
    # Quench knobs, place_optimize spellings. Defaults follow the placement
    # GUIDANCE (pose_score.make_state) rather than place_optimize's library
    # defaults, so ranking and quenching optimize the same objective.
    p.add_argument("--max-displacement", type=float, default=3.0,
                   help="Quench displacement cap in mm (default: 3.0, the "
                        "measured sweet spot; a repo-measured value outranks "
                        "this)")
    p.add_argument("--step", type=float, default=1.0,
                   help="Quench candidate grid step in mm (default: 1.0)")
    p.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                   help=f"Routing grid snap in mm (default: {defaults.GRID_STEP})")
    p.add_argument("--clearance", type=float, default=None,
                   help=f"Min courtyard gap in mm (default: {defaults.CLEARANCE})")
    p.add_argument("--board-edge-clearance", type=float, default=None,
                   help="Hard clearance from board edge in mm (default: 0.55)")
    p.add_argument("--crossing-penalty", type=float, default=30.0,
                   help="Cost per airwire crossing (default: 30, guidance)")
    p.add_argument("--length-weight", type=float, default=0.3,
                   help="Weight on airwire length (default: 0.3, guidance)")
    p.add_argument("--halo-base", type=float, default=0.5)
    p.add_argument("--halo-coef", type=float, default=0.15,
                   help="Extra halo per sqrt(pin count) (default: 0.15, "
                        "guidance)")
    p.add_argument("--halo-weight", type=float, default=2.0)
    p.add_argument("--edge-halo", type=float, default=2.0)
    p.add_argument("--edge-weight", type=float, default=2.0)
    p.add_argument("--no-rotate", action="store_true",
                   help="Disable quench rotation moves")
    p.add_argument("--no-swap", action="store_true",
                   help="Disable quench same-footprint swap moves")
    p.add_argument("--max-passes", type=int, default=10)
    # Probe tier
    p.add_argument("--route-top", type=int, default=2,
                   help="Probe-route the baseline plus the top N kept "
                        "candidates and rank the probed subset by measured "
                        "failures/iterations (default: 2; 0 = static ranking "
                        "only)")
    p.add_argument("--route-args", nargs="+", default=None,
                   help="Extra args for the probe routes (e.g. --clearance)")
    p.add_argument("--plane-score", nargs="+", default=None,
                   metavar="NET[:LAYER]",
                   help="pour these nets on a scratch copy of every viable "
                        "candidate (KiCad ZONE_FILLER refill, the #424 "
                        "exact-fill truth) and fold plane fragility into "
                        "the ranking: island count before hpwl, neck sum "
                        "after it. A bare NET pours the last copper layer. "
                        "Needs KiCad python; see plane_score.py")
    p.add_argument("--plane-score-budget", type=float, default=300.0,
                   help="total seconds for --plane-score across all "
                        "candidates; on overrun the plane terms are "
                        "annotated but EXCLUDED from the ranking "
                        "(default: 300)")
    p.add_argument("--full-probe", action="store_true",
                   help="After the shared-window probe, route the WHOLE "
                        "board on the baseline + the window winner; that "
                        "verdict outranks the window one (a candidate can "
                        "win its window while losing the board). Costs two "
                        "extra full routes.")
    p.add_argument("--route-timeout", type=float, default=900,
                   help="Seconds per probe route (default: 900)")
    # Presentation & provenance
    p.add_argument("--no-render", action="store_true",
                   help="Skip the per-candidate PNGs")
    p.add_argument("--montage", default=None, metavar="PNG",
                   help="Also paste the kept candidates into one labeled grid")
    p.add_argument("--ledger", default=None, metavar="PATH",
                   help="Record each kept candidate in a board_store ledger "
                        "with an exact --only replay command")
    p.add_argument("--only", type=int, default=None, metavar="N",
                   help="Regenerate exactly candidate N (>=1) and exit -- the "
                        "replay primitive. Skips scoring, renders and probes")
    add_board_state_args(p)
    add_tidiness_args(p)
    return p


def _quench_kw(args, intent=None):
    corridor_specs = None
    if args.corridor_weight > 0.0 and intent is not None:
        # The SAME `health.bus_corridors` the grader reads, so the optimizer
        # cannot be priced against lanes the check then re-derives differently.
        corridor_specs = list(
            (getattr(intent, 'health', None) or {}).get('bus_corridors') or ())
        if not corridor_specs:
            print("--corridor-weight given but the intent declares no "
                  "health.bus_corridors: the term is inert", file=sys.stderr)
    return dict(
        max_displacement=args.max_displacement, step=args.step,
        grid_step=args.grid_step, clearance=args.clearance,
        board_edge_clearance=args.board_edge_clearance,
        crossing_penalty=args.crossing_penalty,
        length_weight=args.length_weight, halo_base=args.halo_base,
        halo_coef=args.halo_coef, halo_weight=args.halo_weight,
        edge_halo=args.edge_halo, edge_weight=args.edge_weight,
        allow_rotations=not args.no_rotate, allow_swaps=not args.no_swap,
        max_passes=args.max_passes, ignore_nets=args.ignore_nets,
        lock_refs=args.lock, align_weight=args.align_weight,
        align_radius=args.align_radius, align_span=args.align_span,
        orient_weight=args.orient_weight,
        corridor_weight=args.corridor_weight, corridor_specs=corridor_specs)


def _replay_argv(args, index):
    """The exact command that regenerates candidate `index`: this run's own
    argv with the recording/presentation flags stripped and --only appended.
    Stored in the ledger so `converge.py replay` is a one-liner."""
    argv = list(sys.argv[1:])
    for flag in ('--ledger', '--montage', '--only'):
        while flag in argv:
            i = argv.index(flag)
            del argv[i:i + 2]
    if '--no-render' not in argv:
        argv.append('--no-render')
    return ([sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_placer', 'place_portfolio.py')]
            + argv + ['--only', str(index)])


def _affected_nets(pcb_data, cands, ignore_ids):
    """Union of net names incident to any probed candidate's moved refs.
    One shared set, so every probe routes the SAME nets and the verdicts
    compare like with like."""
    refs = set()
    for c in cands:
        refs.update(c.moved_refs)
    names = set()
    for ref in sorted(refs):
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        for pad in fp.pads:
            if pad.net_id > 0 and pad.net_id not in ignore_ids:
                names.add(pad.net_name)
    return sorted(n for n in names if n)


def _probe(board, nets, args):
    """One probe route; returns the JSON-safe verdict row."""
    from converge import scoped_route, route_verdict
    try:
        res = scoped_route(board, nets, extra_args=args.route_args or [],
                           timeout=args.route_timeout)
    except subprocess.TimeoutExpired:
        return {'failures': None, 'note': f'timeout after {args.route_timeout}s',
                'iterations': None, 'nets': len(nets)}
    n, note = route_verdict(res['summary'])
    return {'failures': n, 'note': note,
            'iterations': res['summary'].get('total_iterations'),
            'vias': res['summary'].get('total_vias'), 'nets': len(nets),
            'returncode': res['returncode']}


def _render(board, before, out_png, ignore_nets):
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_tools', 'render_placement.py'), board,
            '--before', before, '-o', out_png]
    if ignore_nets:
        argv += ['--ignore-nets'] + list(ignore_nets)
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    if r.returncode != 0:
        print(f"WARNING: render failed for {os.path.basename(board)} "
              f"(rc={r.returncode}): {r.stdout[-200:]}", file=sys.stderr)
        return False
    return True


def _montage(path, rows, out_dir):
    """Paste the kept candidates' PNGs into one labeled grid. Presentation
    only: a failure here never fails the run."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print("WARNING: --montage needs PIL; skipped", file=sys.stderr)
        return False
    imgs = []
    for label, png in rows:
        if png and os.path.isfile(png):
            imgs.append((label, Image.open(png)))
    if not imgs:
        return False
    cols = min(3, len(imgs))
    cell_w = max(im.width for _, im in imgs)
    cell_h = max(im.height for _, im in imgs) + 24
    rows_n = (len(imgs) + cols - 1) // cols
    sheet = Image.new('RGB', (cols * cell_w, rows_n * cell_h), 'white')
    draw = ImageDraw.Draw(sheet)
    for k, (label, im) in enumerate(imgs):
        cx, cy = (k % cols) * cell_w, (k // cols) * cell_h
        sheet.paste(im, (cx, cy + 24))
        draw.text((cx + 6, cy + 4), label, fill='black')
    sheet.save(path)
    print(f"Wrote {path}")
    return True


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Run-7 S1 / run-13 F6: unset floors come from the BOARD, not a constant.
    # A fixed 0.25 on a board declaring 0.2 measured 34% more shortfall and
    # double the oob count -- and these tools VETO candidate moves on it, so a
    # wrong floor steers the search, it does not merely mis-report.
    from list_nets import board_floor_knobs
    args.clearance, args.board_edge_clearance, _knobs = board_floor_knobs(
        args.input_file, args.clearance, args.board_edge_clearance)
    print(f"legality at clearance {args.clearance} "
          f"({_knobs['clearance']['source']}), edge {args.board_edge_clearance} "
          f"({_knobs['board_edge_clearance']['source']})")
    # The probe/render subprocesses run with cwd=ROOT (the repo), so a path
    # relative to the CALLER's cwd silently breaks every one of them
    # (measured: 3/3 probe routes rc=1 "no summary", 6/6 renders rc=1, on a
    # portfolio invoked from a board repo). Absolutize at the door.
    args.input_file = os.path.abspath(args.input_file)
    args.out_dir = os.path.abspath(args.out_dir)
    if args.montage:
        args.montage = os.path.abspath(args.montage)
    if args.ledger:
        args.ledger = os.path.abspath(args.ledger)
    if args.intent:
        args.intent = os.path.abspath(args.intent)
    if args.only is not None and args.only < 1:
        parser.error("--only takes a candidate index >= 1 (0 is the baseline "
                     "quench, which needs no stream to reproduce)")
    if args.only is None:
        try:
            from redo_record import record_invocation
            record_invocation()
        except Exception:
            pass

    from kicad_parser import parse_kicad_pcb
    from placement import portfolio
    from placement.groups import GroupError, derive_groups, parse_sources
    from placement.placement_state import gate_or_exit

    print(f"Loading {args.input_file}...")
    pcb = parse_kicad_pcb(args.input_file)
    gate_or_exit(pcb, args.input_file, 'place_portfolio.py',
                 allow_unplaced=args.allow_unplaced,
                 allow_routed=args.allow_routed)

    try:
        sources = parse_sources(args.group_by)
    except GroupError as exc:
        parser.error(str(exc))

    intent = None
    if args.intent:
        from placement import floorplan
        try:
            intent = floorplan.load_intent(args.intent)
        except (OSError, ValueError) as exc:
            print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
            return 2

    # Swap blocks: the intent's declared blocks when given (zones are the
    # designer's grouping), else the structural sources. Never handed to the
    # quench itself -- candidate 0 must equal a default place_optimize run.
    if intent is not None:
        from placement import floorplan
        swap_blocks, _problems = floorplan.resolve_blocks(intent, pcb, sources)
    else:
        swap_blocks = derive_groups(pcb, sources) if sources else {}

    result = portfolio.generate(
        args.input_file, args.out_dir, seed=args.seed,
        n_candidates=args.candidates, strategies=args.strategy,
        radius=args.radius, lock_globs=args.lock,
        ignore_nets=args.ignore_nets, swap_blocks=swap_blocks,
        quench_kw=_quench_kw(args, intent), only=args.only)
    baseline = result['baseline']
    cands = result['candidates']
    free = result['free']

    if args.only is not None:
        if not cands:
            # --only is the REPLAY path and can land here with an empty list
            # when the candidate it names was never generated. Report it as
            # such rather than raising an IndexError.
            print('JSON_SUMMARY: ' + json.dumps(
                {'only': args.only, 'candidates': 0, 'complete': False,
                 'status': 'empty',
                 'reason': 'the candidate was never generated'},
                sort_keys=True))
            return 4
        c = cands[0]
        print("JSON_SUMMARY: " + json.dumps(
            {'only': args.only, 'strategy': c.strategy, 'board': c.board,
             'seed_board': c.seed_board, 'perturbed': len(c.poses),
             'out_dir': args.out_dir}, sort_keys=True))
        return 0

    # ----- score ------------------------------------------------------------
    from placement.floorplan import UntrustworthyOutline
    baseline_overlap = baseline.metrics.get('overlap_area', 0.0)
    baseline_oob = baseline.metrics.get('oob_count', 0)
    baseline_pad_pairs = baseline.metrics.get('pad_conflict_pairs', 0) or 0
    baseline_hole = baseline.metrics.get('hole_shortfall', 0.0) or 0.0
    try:
        for c in cands:
            portfolio.score_candidate(
                c, free=free, baseline_overlap=baseline_overlap,
                baseline_oob=baseline_oob,
                baseline_pad_pairs=baseline_pad_pairs,
                baseline_hole_shortfall=baseline_hole,
                clearance=args.clearance,
                board_edge_clearance=args.board_edge_clearance,
                grid_step=args.grid_step, ignore_nets=args.ignore_nets,
                intent=intent, group_sources=sources)
        portfolio.score_candidate(
            baseline, free=free, baseline_overlap=baseline_overlap,
            baseline_oob=baseline_oob,
            baseline_pad_pairs=baseline_pad_pairs,
            baseline_hole_shortfall=baseline_hole,
            clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            grid_step=args.grid_step, ignore_nets=args.ignore_nets,
            intent=intent, group_sources=sources)
    except UntrustworthyOutline as exc:
        print(f"board outline cannot be trusted for grading: {exc}",
              file=sys.stderr)
        return 3

    # ----- plane-fragility scoring (--plane-score, #118 follow-up) ----------
    # Pour the declared plane nets on a scratch copy of every board (KiCad
    # refill, the #424 exact-fill truth) and fold the result into the rank:
    # island count before hpwl, neck sum after it (see portfolio.rank_key).
    # All-or-nothing: partial coverage would rank scored candidates against
    # unscored ones, so on budget overrun / no KiCad python the flat rank
    # keys are stripped and only the annotation (metrics['plane']) survives.
    if args.plane_score:
        import time as _time
        from plane_score import parse_plane_specs, plane_fragility_score
        specs = parse_plane_specs(
            args.plane_score,
            list(pcb.board_info.copper_layers or ['F.Cu', 'B.Cu']))
        contenders = [baseline] + [c for c in cands if c.board]
        t0 = _time.time()
        incomplete = False
        for c in contenders:
            spent = _time.time() - t0
            if spent > args.plane_score_budget:
                print(f"[plane] budget {args.plane_score_budget:.0f}s "
                      f"exhausted before cand {c.index} -- plane terms "
                      f"EXCLUDED from ranking (partial coverage ranks "
                      f"unfairly); annotations kept")
                incomplete = True
                break
            sc = plane_fragility_score(c.board, specs,
                                       grid_step=args.grid_step)
            if sc is None:
                print("[plane] refill unavailable (KiCad python not found, "
                      "or no named net) -- plane scoring skipped")
                incomplete = True
                break
            c.metrics['plane'] = sc
            c.metrics['plane_islands'] = sc['islands']
            c.metrics['plane_neck'] = sc['neck_sum']
            who = 'baseline' if c.index == 0 else f'cand {c.index}'
            print(f"[plane] {who}: islands={sc['islands']} "
                  f"neck={sc['neck_sum']}mm-eq")
        if incomplete:
            for c in contenders:
                c.metrics.pop('plane_islands', None)
                c.metrics.pop('plane_neck', None)

    # Rule 1 of the acceptance conjunction, K-way (run-7 S6): annotate every
    # candidate whose crossings/hpwl are WORSE than the baseline row.
    # Annotate, not gate: violators stay ranked and probed, they are only
    # barred from the silent JSON_SUMMARY['best'] pick below.
    rule1_violators = set()
    for c in cands:
        if not c.board:
            continue
        r1 = portfolio.rule1_check(c, baseline)
        c.gates['rule1'] = r1
        if r1:
            rule1_violators.add(c.index)

    band_q = portfolio.band_width(cands, args.rank_crossing_band,
                                  baseline.metrics.get('crossings'))
    if band_q:
        print(f"Ranking: crossings banded at {band_q} "
              f"(--rank-crossing-band {args.rank_crossing_band}); the advisory "
              f"health signal decides inside a band")
    ranking_static = portfolio.rank_static(cands, band_q)
    kept, backfilled = portfolio.select_diverse(
        cands, ranking_static, args.keep, args.diversity_mm, free, baseline)
    by_index = {c.index: c for c in cands}
    viable = [c.index for c in cands if c.viable]

    for c in cands:
        state = ('viable' if c.viable else
                 'barren' if not c.board else 'gated out')
        print(f"  cand {c.index:2d} [{c.strategy:8s}] {state}: "
              f"crossings={c.metrics.get('crossings')} "
              f"hpwl={c.metrics.get('hpwl')} "
              f"inv={c.metrics.get('inversions')} "
              f"disp={c.displacement_rms:.2f}mm"
              + (f"  RULE1: {'; '.join(c.gates['rule1'])}"
                 if c.gates.get('rule1') else '')
              + (f"  ({'; '.join(c.gates.get('reasons', []))})"
                 if not c.viable and c.gates.get('reasons') else ''))
    if backfilled:
        print(f"NOTE: only {len(kept) - len(backfilled)} candidate(s) sit "
              f">= {args.diversity_mm}mm apart; {len(backfilled)} backfilled "
              f"by rank ({', '.join(map(str, backfilled))}) -- the slate is "
              f"less diverse than --keep asked for")

    # ----- probe tier -------------------------------------------------------
    ranking_routed = []
    window_kind = 'window'
    if args.route_top > 0 and kept:
        from place_route_loop import _ratsnest_screen
        ids = portfolio.ignore_net_ids(pcb, args.ignore_nets)
        probe_cands = [by_index[i] for i in kept[:args.route_top]]
        nets = _affected_nets(pcb, [baseline] + probe_cands, ids)
        routable = sum(1 for nid, net in pcb.nets.items()
                       if nid > 0 and nid not in ids and len(net.pads) >= 2)
        if routable and len(nets) > 0.5 * routable:
            nets = ['*'] + [f'!{p}' for p in (args.ignore_nets or [])]
            window_kind = 'full'
            print(f"[probe] affected set exceeds half the board -- "
                  f"full probe routes")
        elif not nets:
            print("[probe] no affected nets; skipping the probe tier")
        if nets:
            print(f"[probe] routing {len(nets)} net pattern(s) on baseline + "
                  f"top {len(probe_cands)}")
            baseline.route = _probe(baseline.board, nets, args)
            baseline.route['probe_kind'] = window_kind
            base_rat = {'crossings': baseline.metrics.get('crossings'),
                        'hpwl': baseline.metrics.get('hpwl')}
            for c in probe_cands:
                skip, note = _ratsnest_screen(
                    base_rat, {'crossings': c.metrics.get('crossings'),
                               'hpwl': c.metrics.get('hpwl')}, 25.0)
                if skip:
                    c.route = {'failures': None, 'iterations': None,
                               'note': f'route skipped: {note}'}
                    print(f"[probe] cand {c.index}: skipped ({note})")
                    continue
                c.route = _probe(c.board, nets, args)
                c.route['probe_kind'] = window_kind
                print(f"[probe] cand {c.index}: failures="
                      f"{c.route['failures']} ({c.route['note']})")
            probed = [c for c in [baseline] + probe_cands
                      if c.route and c.route.get('failures') is not None]
            static_pos = {idx: k for k, idx in
                          enumerate([baseline.index] + ranking_static)}
            probed.sort(key=lambda c: (c.route['failures'],
                                       c.route.get('iterations') or 0,
                                       static_pos.get(c.index, 1 << 30)))
            ranking_routed = [c.index for c in probed]

    # ----- full-board probe (--full-probe, run-7 S2/S7) ---------------------
    # The window probe compares like with like on the AFFECTED set, but a
    # candidate can win its window while losing the board (the run-7 adoption
    # question was answered by exactly this escalation, hand-rolled). Route
    # the whole board on the baseline + the window winner; this verdict
    # OUTRANKS the window one.
    ranking_full = []
    if args.full_probe and ranking_routed and window_kind != 'full':
        full_nets = ['*'] + [f'!{p}' for p in (args.ignore_nets or [])]
        top_cand = next((by_index[i] for i in ranking_routed if i != 0), None)
        contenders = [baseline] + ([top_cand] if top_cand is not None else [])
        print(f"[probe] FULL-BOARD probe (--full-probe): baseline"
              + (f" + cand {top_cand.index}" if top_cand is not None else ""))
        for c in contenders:
            c.route_full = _probe(c.board, full_nets, args)
            c.route_full['probe_kind'] = 'full'
            who = 'baseline' if c.index == 0 else f'cand {c.index}'
            print(f"[probe/full] {who}: failures={c.route_full['failures']} "
                  f"({c.route_full['note']})")
        probed_f = [c for c in contenders
                    if c.route_full and c.route_full.get('failures') is not None]
        probed_f.sort(key=lambda c: (c.route_full['failures'],
                                     c.route_full.get('iterations') or 0,
                                     c.index))
        ranking_full = [c.index for c in probed_f]
    elif args.full_probe and window_kind == 'full':
        # The window probe already routed the whole board; its ranking IS the
        # full verdict.
        ranking_full = list(ranking_routed)

    # ----- renders ----------------------------------------------------------
    renders = {}
    if not args.no_render:
        png0 = os.path.join(args.out_dir, 'baseline.png')
        if _render(baseline.board, args.input_file, png0, args.ignore_nets):
            renders[0] = png0
        for i in kept:
            c = by_index[i]
            png = os.path.join(args.out_dir, f'cand_{i:02d}.png')
            if _render(c.board, args.input_file, png, args.ignore_nets):
                renders[i] = png
    if args.montage:
        rows = [(f"#0 baseline  x={baseline.metrics.get('crossings')} "
                 f"hpwl={baseline.metrics.get('hpwl')}", renders.get(0))]
        for i in kept:
            c = by_index[i]
            rows.append((f"#{i} {c.strategy}  x={c.metrics.get('crossings')} "
                         f"hpwl={c.metrics.get('hpwl')}", renders.get(i)))
        _montage(args.montage, rows, args.out_dir)

    # ----- ledger -----------------------------------------------------------
    if args.ledger:
        from board_store import BoardStore, Ledger
        store = BoardStore(os.path.join(
            os.path.dirname(os.path.abspath(args.ledger)), 'boards'))
        lg = Ledger(args.ledger)
        base_sha = store.put(baseline.board)
        lg.append({'iteration': len(lg.entries()), 'kind': 'completion',
                   'lever': 'portfolio-baseline', 'result_sha': base_sha,
                   'parent_sha': None, 'lever_argv': None,
                   'score': {'crossings': baseline.metrics.get('crossings'),
                             'hpwl': baseline.metrics.get('hpwl')},
                   'accepted': False})
        for i in kept:
            c = by_index[i]
            sha = store.put(c.board)
            lg.append({'iteration': len(lg.entries()), 'kind': 'completion',
                       'lever': f'portfolio-{c.strategy}', 'result_sha': sha,
                       'parent_sha': base_sha,
                       'lever_argv': _replay_argv(args, i),
                       'score': {'crossings': c.metrics.get('crossings'),
                                 'inversions': c.metrics.get('inversions'),
                                 'hpwl': c.metrics.get('hpwl'),
                                 'health_penalty':
                                     c.metrics.get('health_penalty'),
                                 'route': c.route},
                       'accepted': False})
        print(f"Recorded baseline + {len(kept)} candidate(s) in {args.ledger} "
              f"(adoption = your accepted:true entry, or converge.py record)")

    # ----- portfolio.json + summary -----------------------------------------
    doc = {'input': args.input_file, 'seed': args.seed,
           'argv': sys.argv[1:], 'free': free,
           'diversity_mm': args.diversity_mm,
           'rank_crossing_band': args.rank_crossing_band,
           'rank_band_q': band_q,
           'corridor_weight': args.corridor_weight,
           'baseline': baseline.to_dict(),
           'candidates': [c.to_dict() for c in cands],
           'ranking_static': ranking_static,
           'ranking_routed': ranking_routed,
           'ranking_full': ranking_full,
           'rule1_violators': sorted(rule1_violators),
           'kept': kept, 'backfilled': backfilled,
           'renders': {str(k): v for k, v in renders.items()}}
    doc_path = os.path.join(args.out_dir, 'portfolio.json')
    with open(doc_path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1, sort_keys=True)
    print(f"Wrote {doc_path}")

    # Headline pick: probe ranking (full-board when --full-probe ran) over
    # static, first index that passes rule 1. Every ranked candidate
    # violating rule 1 -> fall back to the BASELINE, never to a violator
    # (run-7 S6: a worse-on-both-proxies candidate topped the slate).
    ranking_primary = ranking_full or ranking_routed
    best = portfolio.select_best(ranking_primary, ranking_static,
                                 rule1_violators)
    if best is None and (ranking_primary or ranking_static):
        best = 0
        print("NOTE: every ranked candidate violates rule 1 (metrics worse "
              "than the baseline) -- best falls back to the baseline; read "
              "portfolio.json before adopting a violator deliberately")
    best_c = by_index.get(best, baseline) if best is not None else None
    print("JSON_SUMMARY: " + json.dumps({
        'candidates': len(cands) + 1, 'viable': len(viable),
        'kept': len(kept), 'best': best,
        'probe_kind': ('full' if ranking_full
                       else (window_kind if ranking_routed else None)),
        'rule1_excluded': sorted(rule1_violators),
        'best_crossings': (best_c.metrics.get('crossings')
                           if best_c else None),
        'baseline_crossings': baseline.metrics.get('crossings'),
        'best_hpwl': best_c.metrics.get('hpwl') if best_c else None,
        'baseline_hpwl': baseline.metrics.get('hpwl'),
        'out_dir': args.out_dir}, sort_keys=True))
    return 0 if viable else 4


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI
    # writes carries its name. Nothing called declare_lever outside
    # tests, so the unaided instrument had no armed state at all:
    # unarmed it is silent, and armed by hand it refused the engine.
    from placement.provenance import declare_lever
    with declare_lever('place_portfolio.py', sys.argv):
        sys.exit(main())
