#!/usr/bin/env python3
"""Place a board by function, simply: groups, one objective, annealed.

    python3 -X utf8 py_placer/place_simple.py board.kicad_pcb placed.kicad_pcb \\
        --groups groups.json --intent intent.json [--seed 0] [--json report.json]
    python3 -X utf8 py_placer/place_simple.py board.kicad_pcb 's{seed}.kicad_pcb' \\
        --groups groups.json --intent intent.json [--jobs 4] [--json 's{seed}.json']

An OUT carrying {seed}, with no seed named, places the default seeds: 6
started scattered, then 6 with each group gathered (`--scattered`,
`--clustered`, and the opt-in `--anchored`), one process each, and names the
`--judge` (6) legal seeds for a judging route, printed and written to
judge.json beside the boards: those `check_floorplan --intent` grades
without an error and `check_assembly --baseline` grades buildable first,
then the fewest such defects, each lot by lowest cost.
`--seeds LIST` places the seeds it lists instead, and names the judged ones
the same way.

1. GROUPS: the groups file (`check_groups.py`), a crystal and its load caps
   folded into their IC's group. A group is a set of parts kept together:
   the objective's group term keeps it in one piece, whatever shape the
   free space gives it. A group's `near`,
   `apart` and `edge` hints are soft terms of the objective; its `side`, and
   every face the intent declares, is applied first (the writer's flip).
2. The parts the file locks stay put (connectors on their edges, mounting
   holes: whatever the enclosure or a mating part decides), and so does
   every part the intent seats at a declared pose (`fixed_poses[]`, from
   the design brief or mechanical.json), written there first; every other
   part starts at a random place on the board, or, for a seed named in
   `--clustered-seeds`, with each group's members gathered round one random
   point. An unlocked edge part (a
   connector, receptacle, switch or test pad, by its declared `class`, else
   its inferred one) the intent declares on an edge
   (`edge_connectors[]` with an `edge`) is held to that edge
   (`placement/edge_seat.py`): its mating face out, flush with the low
   end of its overhang band, and the anneal only slides it along the edge
   (and turns it, when it has no mating face). Parts declared on one edge
   that cannot all stand there together keep the most that fit, the
   earlier declared on a tie; the rest are placed as any other part, and
   the edge is reported once (`oversubscribed`): the intent's to fix, not a
   seed's. A part seated on one edge near a corner walls the other edge's
   line where it stands wherever it seats. The intent's keep-outs and
   zones bind every part through the anneal's overlap term.
3. ANNEAL: one cost (`placement/simple.py`) -- signal and rail trees,
   crossings, claims and each claim's `max_mm` limit (a crystal at its
   pins), decaps spread over their own group's supply pins,
   each group's gap tree, the halo in front of fine-pitch parts' pads,
   airwires over foreign pads, routing congestion, and the intent's decap
   limits where it grades them at error -- lowered by simulated
   annealing (`placement/simple_anneal.py`), MOVES_PER_PART moves per
   movable part from `--seed`, so the same board, inputs and seed give the
   same placement. A run that leaves a part with no legal pose (a big part
   wedged between fixed ones on a full board) is run again from a fresh
   start, up to RESTARTS times, the first legal placement kept (a part too
   big for the usable area at either quarter turn does not count against
   it): a run that got stuck is a worse arrangement than a fresh one that
   did not, so the fresh start is worth its time. The last run, if it
   never comes clean, is mended where it stands instead
   (`simple_anneal.Annealer.repair`: its overlaps shoved apart), so no part
   is left illegal that a shove frees; when no run is legal, the one that
   left the fewest parts illegal is kept, the earlier on a tie. On
   ch32v003_usb (22 movable parts) the whole run takes about 2.5 s.

Exit codes: 0 placed, 2 unreadable input, a part in two groups, or a face the
board cannot take (two declarations disagree, or a KiCad-locked part is
declared on the other face or at another pose), 4 placed with parts left
illegal, or with a declared pose that is not legal (`refused`) or an edge
that cannot seat all its declared parts together (`oversubscribed`) -- each
the same in every seed, so it is not counted against the seed and the judge
still names seeds.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

#: The anneal's default length per movable part, set by the time budget:
#: ch32v003_usb (22 movable parts) anneals in about 2 s. More moves still pay
#: (`--moves`); fewer leave the run cooling too fast.
MOVES_PER_PART = 500
#: Further anneals, each from a fresh start, when one leaves parts illegal.
RESTARTS = 2
#: The default seeds (an OUT carrying {seed}, no seed named): how many are
#: placed with each start, and how many of the legal ones of lowest
#: `final.cost` are named for a judging route. Over 11 boards, 6 scattered + 6
#: clustered with the 6 cheapest judged shipped fewer open nets and vias than
#: 24 scattered with 12 judged, in half the time; judging all 12 shipped worse
#: (the judge errs more with more to choose from).
SCATTERED_SEEDS = 6
CLUSTERED_SEEDS = 6
JUDGED_SEEDS = 6
#: Anchored seeds (`--anchored`) are opt-in, numbered after the clustered ones.
#: Over 10 boards, 6 scattered + 6 anchored routed better than the default
#: seeds on two (covg blocking 3.1 -> 0.1, bitaxe) and worse on three (tigard,
#: argus, librevna), so the default has none.
ANCHORED_SEEDS = 0

import argparse
import json
import os
import shutil
import sys
import tempfile
import time

from kicad_parser import parse_kicad_pcb  # noqa: E402


def face_targets(intent, blocks, doc, pcb):
    """`(sides, conflicts)`: the face each part must be on -- every face the
    intent declares (`floorplan.declared_side_claims`) and each group's
    `side` -- and the parts two declarations put on different faces."""
    from placement import floorplan
    claims = {r: list(rows) for r, rows in
              floorplan.declared_side_claims(intent, blocks).items()
              if r in pcb.footprints}
    for g in doc['groups']:
        if g.get('side') in ('F', 'B'):
            for r in g['refs']:
                if r in pcb.footprints:
                    claims.setdefault(r, []).append((g['side'], f"group {g['name']}"))
    return floorplan.declared_sides(claims)


def hints(doc, state, movable, args):
    """The groups file's `near`, `apart` and `edge` hints as cost-model
    terms, each over every member the board has (fixed members included);
    a hint whose members are all fixed is a constant and is left out."""
    members = {g['name']: [r for r in g['refs'] if r in state.parts] for g in doc['groups']}
    mov = set(movable)
    out, seen = [], set()
    for g in doc['groups']:
        a = members[g['name']]
        if not a:
            continue
        if g.get('edge') and any(r in mov for r in a):
            out.append({'key': f"edge:{g['name']}:{g['edge']}", 'kind': 'edge', 'a': a,
                        'b': None, 'edge': g['edge'], 'weight': args.edge_weight})
        for kind, w in (('near', args.near_weight), ('apart', args.apart_weight)):
            for other in g.get(kind) or ():
                b = members.get(other) or []
                pair = (kind,) + tuple(sorted((g['name'], other)))
                if not b or pair in seen or not any(r in mov for r in a + b):
                    continue
                seen.add(pair)
                out.append({'key': f"{kind}:{pair[1]}:{pair[2]}", 'kind': kind, 'a': a,
                            'b': b, 'edge': None, 'weight': w})
    return out


def build(pcb, board, doc, base, args, intent=None, blocks=None, fixed=(),
          estimate_skip=()):
    """`(state, model, movable, notes)` for the board, its groups and its
    base intent (the JSON for its claims; the loaded `intent`, with its
    resolved `blocks`, for its keep-outs and zones)."""
    from list_nets import board_floor_knobs
    from placement import floorplan as fp
    from placement.edge_seat import seat_state
    from placement.grouping import (BoardView, claim_rows, decap_conflicts,
                                    fold_followers, group_of, is_ground)
    from placement.simple import Model
    notes = []
    clearance, edge, _ = board_floor_knobs(board)
    # a part that draws no courtyard is seated on an estimated one (#1300),
    # except the parts named in estimate_skip (place_simple's fallback)
    state = seat_state(pcb, board, intent, blocks, clearance, edge, fixed,
                       estimate_courtyards=True, estimate_skip=estimate_skip)
    movable = sorted(r for r, p in state.parts.items() if not p.locked)
    # claims: the intent's own, and each crystal at its IC's oscillator pins
    rows, xnotes = claim_rows(pcb, base)
    notes += xnotes
    # datasheet templates (`placement.templates`): each member joins its
    # chip's group, as a claimed part joins its partner's
    from placement.templates import validate as validate_templates
    tmpls = (list(intent.templates) if intent is not None
             else validate_templates(base.get('templates')))
    tmembers = {m['ref'] for t in tmpls for m in t['members']}
    for t in tmpls:
        notes.append(f"template {t['anchor']}: {', '.join(m['ref'] for m in t['members'])} "
                     f"seated as drawn and carried with it")
    doc, folded = fold_followers(doc, rows + [{'ref': m['ref'], 'near': t['anchor']}
                                              for t in tmpls for m in t['members']])
    if folded:
        notes.append("in their IC's group: " + ', '.join(
            f"{r} ({a} -> {b})" for r, (a, b) in sorted(folded.items())))
    groups = []
    for g in doc['groups']:
        members = [r for r in g['refs'] if r in state.parts]
        if len(members) >= 2 and any(r in movable for r in members):
            groups.append((g['name'], members))
    claims, limits = [], []
    for c in rows:
        s, p = str(c['ref']), str(c['near'])
        if s not in pcb.footprints or p not in pcb.footprints:
            continue
        pads = c.get('pads') or {}
        sp = [q for q in pcb.footprints[s].pads
              if q.net_id and (not pads.get(s) or str(q.pad_number) in map(str, pads[s]))]
        legs = []
        for q in sp:
            pp = [str(o.pad_number) for o in pcb.footprints[p].pads if o.net_id == q.net_id
                  and (not pads.get(p) or str(o.pad_number) in map(str, pads[p]))]
            if pp:
                legs.append((str(q.pad_number), pp))
        if legs:
            claims.append((s, p, legs))
            m = c.get('max_mm')
            limits.append(float(m) if isinstance(m, (int, float)) else None)
    # decaps: each cap from a supply rail to ground, matched to that rail's
    # supply pins (`simple.Model._sites`) by the rule check_groups reports by
    # (`BoardView.decaps`) -- not a claimed part, and not a cap whose IC is in
    # another group (an RC cap beside its button, not a decap)
    view = BoardView(pcb)
    exempt = {str(c['ref']) for c in rows} | tmembers | {
        c['cap'] for c in decap_conflicts(doc, view)}
    supply = view.supply_pins()
    # a decap serves its own group's IC when the group has one on its rail:
    # an LDO's output cap belongs at the LDO, not at the MCU's VDD pin
    gof = group_of(doc)
    decaps, decap_pins, foreign = [], {}, set()
    for r, nid in sorted(view.decaps().items()):
        if r in state.parts and r not in exempt:
            decaps.append((r, nid))
            serves = view.decap_pins(r)
            own = [q for q in serves if gof.get(q[0]) == gof.get(r)]
            if own:
                decap_pins[r] = own
            else:
                foreign.add(r)
                if serves != supply[nid]:
                    decap_pins[r] = serves     # a stacked cap: its owners' pins only
    for r, nid in decaps:
        pins = decap_pins.get(r) or supply[nid]
        notes.append(f"decap {r} on "
                     + '|'.join(pcb.nets[n].name for n in view.decap_rails()[r]) + " serves "
                     + ', '.join(sorted({ic for ic, _n in pins}))
                     + (" (no IC of its group is on its rail)" if r in foreign else ""))
    # congestion counts every net that is routed as tracks: all but ground,
    # which is poured as a plane
    cnets = sorted(n for n in state.net_refs if n and not is_ground(pcb.nets[n].name))
    model = Model(state, pcb, rails=view.rails, claims=claims,
                  decaps=decaps, supply=supply, decap_pins=decap_pins,
                  groups=groups, congestion_weight=args.congestion_weight,
                  track_pitch=args.track_pitch, congestion_nets=cnets,
                  rail_weight=args.rail_weight,
                  ground_nets={n for n in view.rails if is_ground(pcb.nets[n].name)},
                  ground_weight=args.ground_weight, pad_weight=args.pad_weight,
                  crossing_weight=args.crossing_weight, claim_weight=args.claim_weight,
                  decap_weight=args.decap_weight, group_weight=args.group_weight,
                  hints=hints(doc, state, movable, args),
                  halo_coef=args.halo_coef, halo_weight=args.halo_weight,
                  claim_limits=limits, claim_limit_weight=args.claim_limit_weight,
                  templates=tmpls, template_weight=args.template_weight)
    # the unlocked parts the intent declares on an edge, seated there; an
    # edge that cannot hold all its declared parts together seats the most
    # that fit and records itself (`edge_oversubscribed`)
    model.edge_seats, model.edge_unseated, model.edge_oversubscribed = {}, {}, []
    if intent is not None:
        from placement import edge_seat
        model.edge_seats, snotes, model.edge_unseated = edge_seat.seats(
            state, pcb, board, intent, set(movable), crowded=model.edge_oversubscribed)
        notes += snotes
    # the intent's decap limits, where it grades them at error: priced as the
    # grade measures them (the tether gate's terms), per mm past each limit
    if intent is not None and args.decap_limit_weight > 0:
        from placement.quench import TetherGateView
        spec = {k: v for k, v in fp.tether_gate_spec(intent).items()
                if k in ('decap_distance', 'decap_pin_distance')}
        if spec:
            model.set_limits(TetherGateView(pcb, board, state.parts, spec), args.decap_limit_weight)
            notes.append(f"decap limits priced: {len(model._lterms)} term(s) ("
                         + ', '.join(f"{k} {v['limit']:g} mm" for k, v in sorted(spec.items())) + ")")
    return state, model, movable, notes


def _line(t):
    groups = ', '.join(f"{n} {v:.1f}" for n, v in t['per_group'].items() if v > 0.05)
    hints = ', '.join(f"{n} {v:.1f}" for n, v in t['per_hint'].items() if v > 0.05)
    return (f"cost {t['cost']:.1f} (wire {t['wire']:.1f} mm, {t['crossings']} crossings, "
            f"claims {t['claims']:.1f}, "
            + (f"{t['claim_broken']} claim limit(s) broken, " if t.get('claim_broken') else "")
            + f"decaps {t['decaps']:.1f}, "
            + (f"template {t['template']:.1f} mm, " if t.get('template') else "")
            + (f"{t['decap_broken']} decap limit(s) broken, " if t.get('decap_broken') else "")
            + f"halo {t['halo']:.2f}, "
            f"pad hits {t['pad_hits']:.0f}, congestion {t['congestion']:.2f}, "
            f"groups {t['groups']:.1f}" + (f": {groups}" if groups else "")
            + f", hints {t['hints']:.1f}" + (f": {hints}" if hints else "") + ")")


def _floorplan(board, intent):
    """The placed board's floorplan as the judge and `board_score --intent`
    grade it (`board_score.score_floorplan`: check_floorplan --intent, its
    ERROR findings): `{'errors': N, 'rules': [...]}`, or `{'errors': None,
    'reason': ...}` when check_floorplan could not grade the board. A route
    cannot clear such an error (a decap past its pin limit), so a seed that
    carries one cannot be DONE as placed (#1296)."""
    from board_score import score_floorplan
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with tempfile.TemporaryDirectory(prefix='place_simple_fp_') as tmp:
        fp = score_floorplan(root, board, intent, tmp)
    if not fp.get('ran'):
        return {'errors': None, 'reason': fp.get('reason')}
    return {'errors': fp['count'], 'rules': sorted({str(r) for r in fp.get('violations') or ()})}


def _assembly(board, baseline, intent):
    """The placed board's assembly verdict as the judge and `board_score
    --baseline` grade it (`board_score.score_assembly`: check_assembly
    against the board this seed was placed from, so a courtyard pair a moved
    part entered gates): `{'blocking': N, 'buildable': bool, 'fired': [...]}`
    -- `blocking` is board_score's assembly count, 0 when buildable -- or
    `{'blocking': None, 'reason': ...}` when check_assembly could not grade
    the board. No route can make a NOT BUILDABLE placement buildable, so a
    seed that is one cannot be DONE as placed."""
    from board_score import score_assembly
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with tempfile.TemporaryDirectory(prefix='place_simple_asm_') as tmp:
        asm = score_assembly(root, board, intent, tmp, baseline=baseline)
    if not asm.get('ran'):
        return {'blocking': None, 'reason': asm.get('reason')}
    fired = sorted(k for k, v in (asm.get('conjuncts') or {}).items()
                   if isinstance(v, int) and not isinstance(v, bool) and v)
    return {'blocking': asm['count'], 'buildable': asm['buildable'], 'fired': fired}


def _asm_text(asm):
    """'NOT BUILDABLE (courtyard_blocking)', '' when buildable or absent."""
    if not asm:
        return ''
    if asm.get('blocking') is None:
        return f"assembly not graded ({asm.get('reason')})"
    if asm.get('buildable'):
        return ''
    return f"NOT BUILDABLE ({', '.join(asm.get('fired') or ()) or 'check_assembly'})"


def _fp_text(fp):
    """'3 floorplan error(s) (decap_pin_distance)', '' when clean or absent."""
    e = (fp or {}).get('errors')
    if e is None and fp:
        return f"floorplan not graded ({fp.get('reason')})"
    return f"{e} floorplan error(s) ({', '.join(fp.get('rules') or ())})" if e else ''


def _seed_list(spec):
    """'0-3' or '0,1,2,3' (or a mix: '0-2,7') -> [0, 1, 2, 3]."""
    out = []
    for part in str(spec).split(','):
        part = part.strip()
        if '-' in part:
            lo, hi = part.split('-', 1)
            out += list(range(int(lo), int(hi) + 1))
        elif part:
            out.append(int(part))
    return out


def _many_seeds(ap, a):
    """`--seeds` (or the default seeds): each seed a `--seed` run of this
    script in its own process, `--jobs` at a time, every other option passed
    through as parsed; then the seeds to judge (`_name_judged`). Exit 2 if any
    seed failed outright, else 4 if any left parts illegal, else 0."""
    import subprocess
    try:
        seeds = _seed_list(a.seeds)
    except ValueError:
        print(f"place_simple: --seeds {a.seeds!r} is not a list like 0-3 or 0,1,2,3",
              file=sys.stderr)
        return 2
    if not seeds or len(set(seeds)) != len(seeds):
        print(f"place_simple: --seeds {a.seeds!r} names no seed or one twice", file=sys.stderr)
        return 2
    many = len(seeds) > 1
    if many and ('{seed}' not in a.out or (a.json and '{seed}' not in a.json)):
        print("place_simple: with several --seeds, OUT (and --json) must carry {seed}",
              file=sys.stderr)
        return 2
    skip = {'help', 'board', 'out', 'seed', 'seeds', 'jobs', 'json',
            'scattered', 'clustered', 'anchored', 'judge'}
    passed = []
    for act in ap._actions:
        if act.dest in skip or not act.option_strings:
            continue
        val = getattr(a, act.dest)
        if val is not None:
            passed += [act.option_strings[0], str(val)]
    jobs = max(1, a.jobs or min(len(seeds), 4))
    runs = []
    for s in seeds:
        out = a.out.format(seed=s)
        js = a.json.format(seed=s) if a.json else None
        for p in (out, js):
            if p and os.path.dirname(p):
                os.makedirs(os.path.dirname(p), exist_ok=True)
        cmd = ([sys.executable, '-X', 'utf8', os.path.abspath(__file__), a.board, out]
               + passed + ['--seed', str(s)] + (['--json', js] if js else []))
        runs.append((s, out, cmd))
    print(f"place_simple: seeds {', '.join(map(str, seeds))} ({jobs} at a time)")
    live, done, summaries = [], {}, {}
    pending = list(runs)
    while pending or live:
        while pending and len(live) < jobs:
            s, out, cmd = pending.pop(0)
            log = open(os.path.splitext(out)[0] + '.log', 'w', encoding='utf-8')
            live.append((s, out, log, subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)))
        fin = [x for x in live if x[3].poll() is not None]
        if not fin:
            time.sleep(0.2)
            continue
        s, out, log, proc = fin[0]
        live.remove(fin[0])
        rc = proc.returncode
        log.close()
        done[s] = rc
        summary = None
        with open(os.path.splitext(out)[0] + '.log', encoding='utf-8') as f:
            for line in f:
                if line.startswith('JSON_SUMMARY: '):
                    summary = json.loads(line[len('JSON_SUMMARY: '):])
        if summary:
            summaries[s] = summary
            fin = summary['final']
            ill = summary.get('illegal') or []
            start = summary.get('start')
            fpt = ', '.join(t for t in (_fp_text(summary.get('floorplan')),
                                        _asm_text(summary.get('assembly'))) if t)
            print(f"  seed {s}{f' ({start})' if start in ('clustered', 'anchored') else ''}: "
                  f"cost {fin['cost']:.1f}, {summary['moves']} moves in "
                  f"{summary['seconds']['anneal']} s"
                  + (f", no legal pose for {', '.join(ill)}" if ill else '')
                  + (f", {fpt}" if fpt else '') + f" -> {out}")
        else:
            print(f"  seed {s}: exit {rc}, no summary (see {os.path.splitext(out)[0]}.log)")
    _name_judged(a, seeds, summaries)
    rcs = set(done.values())
    return 2 if rcs - {0, 4} else (4 if 4 in rcs else 0)


def judge_order(reports):
    """`{seed: report}` (each seed's summary or `--json` report) -> its legal
    seeds in judging order: the placement's own defects first -- floorplan
    errors plus check_assembly's blocking count, summed as board_score sums
    them -- then `final.cost` (#1296).

    A seed check_floorplan grades with an ERROR cannot be DONE as placed, and
    the limit it breaks often lowers its cost (it stopped paying to hold the
    cap in), so by cost alone it was named first: gp14 board 05 judged
    [8, 7, 3, 4, 9, 5] with s8 three decaps past their pin limit, and the
    judge kept s8. Such a seed is ranked below every seed that grades clean,
    not dropped: when none grades clean, the fewest errors are still judged
    rather than nothing. A seed whose floorplan was not graded ranks after
    every graded one. A seed that left a part illegal is never judged.

    A NOT BUILDABLE seed is the same kind of defect: no route makes it
    buildable, and the anneal reports `illegal []` while check_assembly gates
    a courtyard pair it does not model (the mixer's seeds, every one NOT
    BUILDABLE on its potentiometers' bodies). Its assembly count joins the
    floorplan errors. A report written before the assembly grade existed
    counts 0; one check_assembly could not grade ranks with the ungraded."""
    def rank(s):
        e = (reports[s].get('floorplan') or {}).get('errors')
        asm = reports[s].get('assembly')
        b = 0 if asm is None else asm.get('blocking')
        return (e is None or b is None, (e or 0) + (b or 0),
                reports[s]['final']['cost'], s)
    return sorted((s for s, r in reports.items() if not r.get('illegal')), key=rank)


def _name_judged(a, seeds, summaries):
    """A run of several seeds' last step: the `--judge` legal seeds to judge
    (`judge_order`), printed and written to `judge.json` beside the seeds'
    boards (`judge` in judging order; `scattered`, `clustered`, `anchored` and
    `illegal` list the seeds by start and verdict, `refused` and
    `oversubscribed` the intent's own faults, the same in every seed and
    each WARNED once, `floorplan` each seed's
    floorplan errors, `assembly` its check_assembly blocking count, and
    `baseline` the board the seeds were placed from, which judge_seeds hands
    quick_score so the fast route's score grades assembly the same way)."""
    judge = judge_order({s: summaries[s] for s in seeds if s in summaries})[:a.judge]
    doc = {'judge': judge,
           'scattered': [s for s in seeds if summaries.get(s, {}).get('start')
                         not in ('clustered', 'anchored')],
           'clustered': [s for s in seeds if summaries.get(s, {}).get('start') == 'clustered'],
           'anchored': [s for s in seeds if summaries.get(s, {}).get('start') == 'anchored'],
           'illegal': [s for s in seeds if summaries.get(s, {}).get('illegal')],
           # the declared poses refused, the same in every seed: the
           # intent's to fix, not a seed's
           'refused': next((summaries[s]['refused'] for s in seeds
                            if summaries.get(s, {}).get('refused')), {}),
           # and the edges that cannot seat all their declared parts
           # together, likewise
           'oversubscribed': next((summaries[s]['oversubscribed'] for s in seeds
                                   if summaries.get(s, {}).get('oversubscribed')), []),
           # each placed seed's check_floorplan --intent errors (None: not
           # graded), keyed by seed as JSON keys must be
           'floorplan': {str(s): (summaries[s].get('floorplan') or {}).get('errors')
                         for s in seeds if s in summaries},
           # each placed seed's check_assembly blocking count (0 buildable,
           # None not graded) against the board it was placed from
           'assembly': {str(s): (summaries[s].get('assembly') or {}).get('blocking')
                        for s in seeds if s in summaries},
           'baseline': (os.path.abspath(a.board) if getattr(a, 'board', None)
                        else None)}
    path = os.path.join(os.path.dirname(a.out.format(seed=seeds[0])), 'judge.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1)
    print(f"place_simple: judge seeds {', '.join(map(str, judge)) or 'none'} "
          f"(the {a.judge} legal seeds of lowest cost, buildable and floorplan-clean "
          f"first) -> {path}")
    for s in judge:
        asm = summaries[s].get('assembly') or {}
        if asm.get('blocking'):
            print(f"  WARNING: seed {s} is judged {_asm_text(asm)}: it cannot be DONE as "
                  f"placed (fewer than {a.judge} legal seeds are buildable and clean)")
        elif asm and asm.get('blocking') is None:
            print(f"  WARNING: seed {s} is judged with its {_asm_text(asm)}: grade it with "
                  f"check_assembly --baseline before keeping it")
        fp = summaries[s].get('floorplan') or {}
        if fp.get('errors'):
            print(f"  WARNING: seed {s} is judged with {_fp_text(fp)}: it cannot be DONE "
                  f"as placed (fewer than {a.judge} legal seeds grade clean)")
        elif fp and fp.get('errors') is None:
            print(f"  WARNING: seed {s} is judged with its {_fp_text(fp)}: grade it with "
                  f"check_floorplan --intent before keeping it")
    for r, why in sorted(doc['refused'].items()):
        print(f"  WARNING: {r}'s declared pose is not legal, in every seed: "
              f"{'; '.join(why)}")
    from placement.edge_seat import crowding_text
    for rec in doc['oversubscribed']:
        print(f"  WARNING: {crowding_text(rec)}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board', help='the board; the parts it locks stay put')
    ap.add_argument('out', help='the placed board to write')
    ap.add_argument('--groups', required=True, help='the groups file (check_groups.py)')
    ap.add_argument('--intent', required=True,
                    help='the emitted intent (check_floorplan --emit-intent): its '
                         'proximity claims, keep-outs, zones, declared faces and decap '
                         'limits')
    ap.add_argument('--crossing-weight', type=float, default=2.0,
                    help='mm of wire one airwire crossing costs (default 2)')
    ap.add_argument('--claim-weight', type=float, default=1.0,
                    help='per mm of a claimed pad to its partner pad (default 1)')
    ap.add_argument('--claim-limit-weight', type=float, default=10.0,
                    help="per mm a claimed pad is past its claim's max_mm, pad edge to "
                         "its partner pad as the grade measures it (a crystal leg past "
                         "8.1 mm of its IC pin, a load cap past 2.2 mm of its crystal); "
                         "a move of the partner carries the parts holding their limits "
                         "at it (default 10; 0 leaves the limits unpriced)")
    ap.add_argument('--template-weight', type=float, default=100.0,
                    help="per mm a datasheet-template member's pads are, on average, from "
                         "where the intent's `templates[]` puts them for its chip's pose "
                         "(default 100); the members are seated there before the anneal and "
                         "carried with every move of the chip")
    ap.add_argument('--decap-weight', type=float, default=2.0,
                    help='per mm of a decap to the supply-pin site it is matched to '
                         '(default 2)')
    ap.add_argument('--decap-limit-weight', type=float, default=10.0,
                    help='per mm past an intent decap limit it grades at error '
                         '(decaps.max_distance_mm, max_pin_distance_mm), measured as the '
                         'grade measures it (default 10; 0 leaves the limits unpriced)')
    ap.add_argument('--group-weight', type=float, default=5.0,
                    help='per mm of gap in a group\'s tree over its members\' '
                         'courtyards (default 5)')
    ap.add_argument('--near-weight', type=float, default=2.0,
                    help='per mm of gap between two groups the groups file puts '
                         '`near` each other (default 2)')
    ap.add_argument('--apart-weight', type=float, default=5.0,
                    help='per mm two groups kept `apart` come within 3 mm of each '
                         'other (default 5)')
    ap.add_argument('--edge-weight', type=float, default=2.0,
                    help="per mm a group with an `edge` is from that board edge "
                         "(default 2)")
    ap.add_argument('--halo-coef', type=float, default=0.25,
                    help='mm of halo per net leaving by a side (default 0.25)')
    ap.add_argument('--halo-weight', type=float, default=10.0,
                    help='cost per mm^2 of a halo shortfall (default 10)')
    ap.add_argument('--rail-weight', type=float, default=0.3,
                    help='per mm of a supply rail\'s tree: less than a signal\'s 1, '
                         'being wide or poured, but not nothing (default 0.3)')
    ap.add_argument('--ground-weight', type=float, default=0.02,
                    help='per mm of ground\'s tree: a plane, each pad only drops a '
                         'via, so almost nothing (default 0.02)')
    ap.add_argument('--pad-weight', type=float, default=2.0,
                    help='per crossing of a tree edge over a foreign pad, a through-hole '
                         'pad counting twice (default 2)')
    ap.add_argument('--congestion-weight', type=float, default=5.0,
                    help='per mm^2 of routing overflow squared, summed over a 0.5 mm '
                         'grid (default 5)')
    ap.add_argument('--track-pitch', type=float, default=0.33,
                    help='mm of board one track takes, width plus clearance, for the '
                         'congestion supply (default 0.33)')
    ap.add_argument('--moves', type=int, default=0,
                    help='annealing moves (default 0: MOVES_PER_PART per movable '
                         'part)')
    ap.add_argument('--seed', type=int,
                    help='the random stream: the start and the moves (default 0); with '
                         'an OUT carrying {seed}, that one seed placed there')
    ap.add_argument('--seeds', metavar='LIST',
                    help='place these seeds, e.g. 0-3 or 0,1,2,3, instead of the default '
                         'ones: OUT and --json carry {seed}, each seed is its own process '
                         '(the same placement a --seed run gives) and its log goes beside '
                         'its board as .log')
    ap.add_argument('--clustered-seeds', metavar='LIST',
                    help='the seeds, listed as --seeds lists them, that start with each '
                         "group's movable members gathered round one random point of the "
                         'board instead of every part scattered on its own (default: none)')
    ap.add_argument('--anchored-seeds', metavar='LIST',
                    help="the seeds, listed as --seeds lists them, that start with each "
                         "group gathered round the point its wiring prefers (a quick "
                         "least-squares solve toward the fixed parts and groups it is "
                         "wired to, re-weighted and jittered per seed)")
    ap.add_argument('--scattered', type=int,
                    help=f'with an OUT carrying {{seed}} and no seed named, the seeds '
                         f'started scattered, numbered from 0 (default {SCATTERED_SEEDS})')
    ap.add_argument('--clustered', type=int,
                    help=f'likewise, the seeds started clustered, after the scattered ones '
                         f'(default {CLUSTERED_SEEDS})')
    ap.add_argument('--anchored', type=int,
                    help=f'likewise, the seeds started anchored, after the clustered ones '
                         f'(default {ANCHORED_SEEDS}: opt-in)')
    ap.add_argument('--judge', type=int, default=JUDGED_SEEDS,
                    help=f'in a run of several seeds, how many legal seeds to name for a '
                         f'judging route, printed and written to judge.json beside the '
                         f'boards: those check_floorplan --intent grades without an error '
                         f'and check_assembly grades buildable first, then by lowest cost '
                         f'(default {JUDGED_SEEDS})')
    ap.add_argument('--jobs', type=int, default=0,
                    help='with several seeds, how many run at once (default: one per '
                         'seed, at most 4)')
    ap.add_argument('--json', help='write the report here')
    a = ap.parse_args(argv)
    starts = [f for f, v in (('--scattered', a.scattered), ('--clustered', a.clustered),
                             ('--anchored', a.anchored)) if v is not None]
    if a.seed is None and not a.seeds and '{seed}' in a.out:
        if a.clustered_seeds or a.anchored_seeds:
            print("place_simple: the default seeds number their own starts; name the seeds "
                  "with --seeds to use --clustered-seeds or --anchored-seeds", file=sys.stderr)
            return 2
        sc = SCATTERED_SEEDS if a.scattered is None else a.scattered
        cl = CLUSTERED_SEEDS if a.clustered is None else a.clustered
        an = ANCHORED_SEEDS if a.anchored is None else a.anchored
        n = sc + cl + an
        if min(sc, cl, an) < 0 or n < 1:
            print("place_simple: --scattered, --clustered and --anchored must be 0 or more, "
                  "with at least one seed", file=sys.stderr)
            return 2
        a.seeds = f'0-{n - 1}'
        a.clustered_seeds = f'{sc}-{sc + cl - 1}' if cl else None
        a.anchored_seeds = f'{sc + cl}-{n - 1}' if an else None
    elif starts:
        print(f"place_simple: {', '.join(starts)} set the default seeds of an OUT carrying "
              "{seed}; with --seed or --seeds, list the starts with --clustered-seeds and "
              "--anchored-seeds", file=sys.stderr)
        return 2
    if a.judge < 0:
        print("place_simple: --judge must be 0 or more", file=sys.stderr)
        return 2
    if not a.seeds:
        if a.seed is None:
            a.seed = 0
        a.out = a.out.replace('{seed}', str(a.seed))
        if a.json:
            a.json = a.json.replace('{seed}', str(a.seed))
    try:
        clustered = a.seed in _seed_list(a.clustered_seeds or '')
        anchored = a.seed in _seed_list(a.anchored_seeds or '')
    except ValueError:
        print(f"place_simple: --clustered-seeds {a.clustered_seeds!r} or --anchored-seeds "
              f"{a.anchored_seeds!r} is not a list like 0-3 or 0,1,2,3", file=sys.stderr)
        return 2
    if a.seeds:
        return _many_seeds(ap, a)
    t0 = time.time()
    from placement import floorplan
    from placement.groups import parse_sources
    from placement.grouping import GroupsError, load_groups
    from placement.parser import extract_locked_refs
    from placement.portfolio import copy_siblings
    from placement.writer import SideFlipUnsupported, write_placed_output
    try:
        doc = load_groups(a.groups)
        base = json.load(open(a.intent, encoding='utf-8'))
        intent = floorplan.load_intent(a.intent)
        pcb = parse_kicad_pcb(a.board)
    except (GroupsError, OSError, ValueError) as ex:
        print(f"place_simple: {ex}", file=sys.stderr)
        return 2
    first, twice = {}, {}
    for g in doc['groups']:
        for r in g['refs']:
            if first.setdefault(r, g['name']) != g['name']:
                twice.setdefault(r, [first[r]]).append(g['name'])
    if twice:
        print("place_simple: in two groups: " + '; '.join(
            f"{r} ({', '.join(gs)})" for r, gs in sorted(twice.items()))
            + ". check_groups names every problem in the file. Nothing was written.",
            file=sys.stderr)
        return 2
    sources = parse_sources('auto')
    blocks, _problems = floorplan.resolve_blocks(intent, pcb, sources)
    # FACES first: the parts a declaration puts on the other face are
    # flipped at their pose (the writer's #714 flip) before anything is placed
    sides, conflicts = face_targets(intent, blocks, doc, pcb)
    if conflicts:
        print("place_simple: contradictory face declarations: " + '; '.join(
            f"{r}: " + ', '.join(f"{s} from {src}" for s, src in rows)
            for r, rows in sorted(conflicts.items())) + ". Nothing was written.",
            file=sys.stderr)
        return 2
    # ... and every part the intent seats at a declared pose goes there and
    # stays (`fixed_poses[]`: the mechanical facts the brief compiles)
    fixed, flips = floorplan.declared_moves(intent, sides, pcb)
    locked = sorted({f['reference'] for f in flips} & extract_locked_refs(a.board))
    if locked:
        print(f"place_simple: {', '.join(locked)} locked in the board on a face or at a "
              f"pose its declaration does not name; a lock is a decision. Nothing was "
              f"written.", file=sys.stderr)
        return 2
    src, tmp = a.board, None
    if flips:
        tmp = tempfile.mkdtemp(prefix='place_simple_')
        src = os.path.join(tmp, os.path.basename(a.board))
        try:
            write_placed_output(a.board, src, flips)
        except (ValueError, SideFlipUnsupported) as ex:
            shutil.rmtree(tmp, ignore_errors=True)
            print(f"place_simple: cannot flip: {ex}", file=sys.stderr)
            return 2
        copy_siblings(a.board, src)
        pcb = parse_kicad_pcb(src)
        blocks, _problems = floorplan.resolve_blocks(intent, pcb, sources)
        print("  NOTE: at their declared pose or face: " + ', '.join(
            f"{f['reference']}" + (f" -> {f['new_side']}" if 'new_side' in f else
                                   f" at ({f['new_x']:g}, {f['new_y']:g}) {f['new_rotation']:g}")
            for f in flips))
    from placement import simple_anneal
    from placement.simple import legal
    state, model, movable, notes = build(pcb, src, doc, base, a, intent, blocks, set(fixed))
    for n in notes:
        print(f"  NOTE: {n}")
    for r, why in sorted(model.edge_unseated.items()):
        print(f"  WARNING: {r} is declared on an edge but has no seat there ({why}); "
              f"placed as any other part")
    from placement.edge_seat import crowding_text
    for rec in model.edge_oversubscribed:
        print(f"  WARNING: {crowding_text(rec)}")
    print(f"place_simple: {len(movable)} movable part(s), fixed: "
          f"{', '.join(sorted(r for r in state.parts if r not in movable)) or 'none'}")
    for name, members in model.groups:
        print(f"  group {name}: {', '.join(members)}")
    # a declared pose is a mechanical fact, checked and never nudged (#1054),
    # against every part that stays where it is
    from placement.seeder import _fixed_pose_check
    waived = frozenset(frozenset(p) for p in intent.courtyard_waiver_pairs())
    stay = {r: (p.x, p.y, p.rot) for r, p in state.parts.items() if r not in movable}
    refused = {}
    for r in sorted(set(fixed) & set(state.parts)):
        how, reasons, _c = _fixed_pose_check(
            state, r, stay[r], {o: q for o, q in stay.items() if o != r}, waived=waived)
        if how is None:
            refused[r] = reasons
            print(f"  WARNING: {r}'s declared pose is not legal: {'; '.join(reasons)}")
    # a part too big for the usable box at either quarter turn ends outside
    # it whatever the anneal does, so a fresh start cannot help it
    ux0, uy0, ux1, uy1 = state.usable
    too_big = set()
    for r in movable:
        x0, y0, x1, y1 = state.parts[r].rect()
        w, h = x1 - x0, y1 - y0
        if not ((w <= ux1 - ux0 and h <= uy1 - uy0) or (h <= ux1 - ux0 and w <= uy1 - uy0)):
            too_big.add(r)
            print(f"  WARNING: {r} ({w:.2f} x {h:.2f} mm) does not fit in the usable area "
                  f"({ux1 - ux0:.2f} x {uy1 - uy0:.2f} mm) at either quarter turn")
    t1 = time.time()
    moves = a.moves or MOVES_PER_PART * len(movable)

    def ok(r, st=None, mdl=None):
        st, mdl = st or state, mdl or model
        p = st.parts[r]
        if r in mdl.edge_seats:         # a declared pose's check, on its edge
            from placement.edge_seat import legal_on_edge
            fits = legal_on_edge(st, r, (p.x, p.y, p.rot))
        else:
            fits = legal(st, r, p.x, p.y, p.rot)
        return fits and st.intent_clear(r, p.grade_rects(p.x, p.y, p.rot))

    def runs(st, mdl):
        """The restarts on one state: `(kept, k)`, the run that left the
        fewest parts illegal -- (illegal refs, poses, res), the earlier on a
        tie -- and the last run's index."""
        kept, k = None, -1
        for k in range(RESTARTS + 1):
            # a restart's own stream, distinct from every other seed's first run
            seed = a.seed if k == 0 else (a.seed + 1) * 1000003 + k
            simple_anneal.scatter(mdl, movable, seed=seed, clustered=clustered,
                                  anchored=anchored)
            res = simple_anneal.anneal(mdl, movable, moves=moves, seed=seed,
                                       log=lambda s: print(f"  {s}"),
                                       repair=k == RESTARTS, seats=mdl.edge_seats)
            illegal = [r for r in movable if not ok(r, st, mdl)]
            if kept is None or len(illegal) < len(kept[0]):
                kept = (illegal, {r: (st.parts[r].x, st.parts[r].y, st.parts[r].rot)
                                  for r in movable}, res)
            if set(illegal) <= too_big:
                break
            print(f"  NOTE: run {k + 1} left no legal pose for {', '.join(illegal)}"
                  + ("; annealing again from a fresh start" if k < RESTARTS else ""))
        return kept, k
    fallback = []           # parts seated on what the board draws instead (#1300)
    if not movable:
        print("place_simple: nothing to place; the board is written as it is")
        k, kept = -1, ([], {}, {'moves': 0, 'reseated': [], 'stats': {}})
    else:
        kept, k = runs(state, model)
        # #1300: an ESTIMATED courtyard is ours, not the board's. When the
        # runs leave a part no legal pose on the estimates, the part itself
        # goes back to the basis KiCad judges (what the board draws, its pad
        # box), then every part does -- the model before the estimates --
        # and the run that leaves fewer parts illegal is kept, the estimates'
        # on a tie: an estimate that costs a legal pose is not worth an
        # illegal one (esp_prog U2 beside USB1, 7 of 12 seeds).
        est = set(state.courtyard_estimates)
        for skip in sorted({frozenset((set(kept[0]) - too_big) & est), frozenset(est)},
                           key=len):
            if not skip or not set(kept[0]) - too_big:
                continue
            print(f"  NOTE: {', '.join(sorted(set(kept[0]) - too_big))} left without a "
                  f"legal pose on estimated courtyards; seating "
                  f"{', '.join(sorted(skip)) if skip != est else 'every part'} on what "
                  f"the board draws (#1300)")
            st2, mdl2, _m2, _n2 = build(pcb, src, doc, base, a, intent, blocks,
                                        set(fixed), estimate_skip=skip)
            kept2, k2 = runs(st2, mdl2)
            if len(kept2[0]) < len(kept[0]):
                state, model, kept, k, fallback = st2, mdl2, kept2, k2, sorted(skip)
    _ill, poses, res = kept
    for r, pose in poses.items():
        state.parts[r].ensure_rotation(pose[2])
        state.apply_move(r, *pose)
    model.reprice_all()
    final = model.total()
    t2 = time.time()
    # each template member's drift from its template pose, worst pad
    from placement.templates import pad_deviation
    drift = []
    for i, (anc, m) in enumerate(model.tmpl):
        P = state.parts[m['ref']]
        _mean, worst = pad_deviation(P, (P.x, P.y, P.rot), model.template_target(i))
        drift.append({'anchor': anc, 'ref': m['ref'], 'worst_mm': round(worst, 3)})
    # a refused declared pose is the intent's, the same in every seed: it is
    # reported apart (`refused`), not counted against the seed (#1297)
    illegal = [r for r in movable if not ok(r)]
    print(f"place_simple: annealed {res['moves']} moves in {t2 - t1:.1f} s; {_line(final)}")
    if res.get('repaired'):
        print("  NOTE: the last run never came clean; its overlaps were shoved apart where it stood")
    if res['reseated']:
        print(f"  NOTE: moved to the nearest legal pose after the anneal: {', '.join(res['reseated'])}")
    if illegal:
        print(f"  WARNING: no legal pose found for: {', '.join(illegal)}")
    if drift:
        print("  template drift (worst pad, mm): " + ', '.join(
            f"{d['ref']} {d['worst_mm']:.2f}" for d in drift))
    # an `apart` hint whose groups still touch prices the least gap over all
    # their member pairs, 0 at any pose where they touch: a flat
    # weight x APART_MM that steered nothing (#1330)
    from placement.simple import APART_MM
    saturated = [h['key'] for h in model.hints
                 if h['kind'] == 'apart' and model._hint_full(h)[0] <= 0.0]
    for key in saturated:
        print(f"  WARNING: {key}: the two groups touch, so the hint sits at its "
              f"saturated value (weight x {APART_MM:g} mm wherever they still "
              f"touch) and steered nothing; separate them with a zone or keep-out, "
              f"or drop the hint")
    placements = [{'reference': r, 'new_x': round(state.parts[r].x, 4),
                   'new_y': round(state.parts[r].y, 4),
                   'new_rotation': state.parts[r].rot % 360.0}
                  for r in movable]
    write_placed_output(src, a.out, placements)
    copy_siblings(a.board, a.out)
    if tmp:
        shutil.rmtree(tmp, ignore_errors=True)
    # the placed board's floorplan, graded as the judge grades it (#1296):
    # report only, the exit code and the placement are untouched
    fp = _floorplan(a.out, a.intent)
    print(f"  floorplan: {_fp_text(fp) or 'no error'} (check_floorplan --intent)")
    # and its assembly, against the board it was placed from: report only
    asm = _assembly(a.out, a.board, a.intent)
    print(f"  assembly: {_asm_text(asm) or 'buildable'} (check_assembly --baseline)")
    final.pop('per_group', None)
    final.pop('per_hint', None)
    report = {'final': final, 'moves': res['moves'], 'seed': a.seed,
              'start': ('anchored' if anchored else 'clustered' if clustered
                        else 'scattered'), 'runs': k + 1,
              'declared': sorted(f['reference'] for f in flips),
              'reseated': res['reseated'], 'repaired': bool(res.get('repaired')),
              'illegal': illegal, 'refused': refused, 'estimate_fallback': fallback,
              'oversubscribed': model.edge_oversubscribed, 'floorplan': fp, 'assembly': asm,
              'unseated': model.edge_unseated, 'notes': notes,
              'groups': model.groups, 'stats': res['stats'], 'templates': drift,
              'hints_saturated': saturated,
              'seconds': {'setup': round(t1 - t0, 1), 'anneal': round(t2 - t1, 1)}}
    if a.json:
        json.dump(report, open(a.json, 'w', encoding='utf-8'), indent=1)
    print('JSON_SUMMARY: ' + json.dumps({k: report[k] for k in
                                          ('final', 'moves', 'seed', 'start', 'reseated', 'illegal',
                                           'refused', 'oversubscribed', 'floorplan', 'assembly',
                                           'hints_saturated', 'estimate_fallback', 'seconds')}))
    return 4 if illegal or refused or model.edge_oversubscribed else 0


if __name__ == '__main__':
    sys.exit(main())
