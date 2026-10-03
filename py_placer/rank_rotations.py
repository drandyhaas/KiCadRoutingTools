#!/usr/bin/env python3
"""Rank one part's rotations by what place_seed and its polish produce at each.

On a pile a part keeps its input rotation -- a generator default, not a
decision -- and nothing else ranks a large IC's rotation: `--rotate-by-facing`
scores edge facing only, and `converge.py poses --ref` moves ONE part with its
neighbours frozen, so once the seed has packed an IC's decaps against its
supply pins every other rotation of it is vetoed (#1113: StickHub's U1, every
candidate dropped). The rotation has to be judged at SEED level: seat the
whole board once per candidate angle and compare what comes out. Run 39 did
that by hand -- U1 at 270 instead of the pile's 0 took seed crossings 222 to
182 and the first full route's blocking 31-37 to 16-19.

Per candidate rotation (the input angle, +90, +180, +270; the 45-degree set
too with --diagonal-rotations): the input intent plus one block declaring the
part's `rotation`, then place_seed for every --seeds value, exactly as
compare_seeds runs it (same subprocess, same polish), and the written board
read back to confirm the part really sits at that angle. A rotation where the
part went unseated, or was written at another angle, is a hard fail and ranks
last. Next come the angles where a seed's re-seat could not put the part back
(`reseat_declined`, #1117): the polish walked it out because that paid, so
they rank after every angle whose seeds held it, but they still rank. The rest
rank by (unseated, probe failures when probed, crossings, hpwl,
grade errors) over the seeds' medians; a tie goes to the earlier angle in the
ladder, the input angle when it is ranked. Any other seed that fails its
intent gate is NOT a tier (on a pile most do, for repairable reasons), but
every angle reports how many of its seeds did, and the winner line says so. --probe
routes the top --probe-top rotations full-board (converge.probe_route), and a
probe verdict outranks crossings, which is only a proxy (run 7).

A CONTROL arm seeds the same seeds with the intent as given -- no rotation
declared -- so the baseline the winner is compared with is the seed you would
have got, not the input angle forced (the seeder may turn the part itself).
It is reported, never ranked.

    python rank_rotations.py pile.kicad_pcb --intent fp.json --out-dir rot \\
        --probe --write-intent fp_rot.json

With no --ref it ranks the unlocked, undeclared, non-connector part with the
most connected pads (at least --min-pads). Exit 0 with a winner; 2 for usage
errors; 3 when place_seed cannot seed the board (it looks placed: pass
--seed-args='--force'); 4 when nothing is rankable (the part is locked, its
rotation is already declared, no part is eligible, the zone plan is refused,
or every rotation hard-failed).
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import fnmatch
import json
import os
import statistics
import sys

from compare_seeds import (_split_forwarded, probe_full, probe_nets,
                           run_place_seed)

#: Flags --seed-args may not carry: the ranker sets them itself, or they
#: would make an arm something other than a polished seed from this pile.
FORBIDDEN_SEED_ARGS = ('--seed', '--intent', '--no-polish', '--repair',
                       '--repair-decaps', '--reseat', '--reseat-region',
                       '--reseat-min-gain', '--dry-run', '--group-by')

def place_seed_options():
    """place_seed's own option strings, read off its `--help` (its parser
    is built inside `main()`, so it cannot be imported)."""
    import re
    import subprocess
    from compare_seeds import ROOT
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_placer', 'place_seed.py'),
                        '--help'], capture_output=True, text=True,
                       encoding='utf-8', errors='replace', cwd=ROOT)
    return sorted(set(re.findall(r'(?<![\w-])(--[a-z][a-z0-9-]*)',
                                 r.stdout)))


def resolve_option(token, options):
    """The option argparse would read `token` as: exact, else a unique
    prefix (argparse's abbreviation rule). None when it names none;
    `'<ambiguous>'` when it abbreviates several."""
    tok = token.split('=', 1)[0]
    if not tok.startswith('--'):
        return None
    if tok in options:
        return tok
    hits = [o for o in options if o.startswith(tok)]
    if len(hits) == 1:
        return hits[0]
    return '<ambiguous>' if hits else None


#: place_seed exit codes the ranker acts on.
PLACE_SEED_OK = (0, 4)       # 4: written, but the seed fails its intent gate
PLACE_SEED_PLACED = 3        # the board looks placed (UNPLACED_EXIT)
PLACE_SEED_PLAN_REFUSED = 5  # the zone plan was refused before any write


def build_parser():
    p = argparse.ArgumentParser(
        description="Rank one part's rotations by what place_seed and its "
                    "polish produce at each.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python rank_rotations.py pile.kicad_pcb --intent fp.json --out-dir rot
  python rank_rotations.py pile.kicad_pcb --intent fp.json --ref U1 \\
      --seeds 0 1 2 --out-dir rot --probe --write-intent fp_rot.json
""")
    p.add_argument("input_file", help="Unplaced board (outline + parts pile)")
    p.add_argument("--intent", required=True,
                   help="Floorplan intent; each arm adds one rotation block "
                        "to a copy, the file itself is never modified")
    p.add_argument("--ref", default=None,
                   help="The part to rank. Default: the eligible part with "
                        "the most connected pads")
    p.add_argument("--min-pads", type=int, default=16,
                   help="Without --ref, only parts with at least this many "
                        "connected pads are eligible (default 16)")
    p.add_argument("--rotations", type=float, nargs="+", default=None,
                   help="Absolute angles to rank instead of the input angle "
                        "and its quarter turns")
    p.add_argument("--diagonal-rotations", action="store_true",
                   help="Also rank the input angle +45/135/225/315, and pass "
                        "--diagonal-rotations to every place_seed arm")
    p.add_argument("--seeds", type=int, nargs="+", default=[0],
                   help="Seed values; every rotation uses the same ones "
                        "(default: 0)")
    p.add_argument("--out-dir", required=True,
                   help="Directory for the arm intents, boards and "
                        "rotations.json")
    p.add_argument("--ignore-nets", nargs="+", default=None,
                   help="Net patterns forwarded to place_seed's polish and "
                        "excluded from the probe")
    p.add_argument("--group-by", default="auto",
                   help="Block sources, forwarded to place_seed (default: "
                        "auto)")
    p.add_argument("--seed-args", nargs="+", default=None,
                   help="Extra args for every place_seed call, in ONE quoted "
                        "= value: --seed-args='--rotate-by-facing --force'")
    p.add_argument("--probe", action="store_true",
                   help="Route the top --probe-top rotations full-board; a "
                        "probe verdict outranks crossings")
    p.add_argument("--probe-top", type=int, default=2,
                   help="How many rotations --probe routes (default 2)")
    p.add_argument("--route-args", nargs="+", default=None,
                   help="Extra args for every probe route, same quoting rule "
                        "as --seed-args")
    p.add_argument("--json-out", default=None,
                   help="Where to write the ranking (default: "
                        "<out-dir>/rotations.json)")
    p.add_argument("--write-best", default=None, metavar="BOARD",
                   help="Copy the winning rotation's best seed board here "
                        "(with its siblings)")
    p.add_argument("--write-intent", default=None, metavar="JSON",
                   help="Write the input intent plus the winning rotation "
                        "block here")
    return p


def same_angle(a, b, tol=1e-3):
    """Equal as rotations (modulo 360)."""
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d) <= tol


def _norm(a):
    a = float(a) % 360.0
    return 0.0 if abs(a - 360.0) < 1e-9 else a


def candidate_rotations(input_rot, explicit=None, diagonal=False):
    """The ladder: the input angle first, then its quarter turns, then (with
    `diagonal`) its 45-degree turns. `explicit` replaces it, in its order."""
    if explicit:
        return [_norm(r) for r in explicit]
    steps = [0.0, 90.0, 180.0, 270.0] + ([45.0, 135.0, 225.0, 315.0]
                                         if diagonal else [])
    return [_norm(input_rot + d) for d in steps]


def rotation_block(ref, rot, taken=()):
    """One intent block declaring `ref`'s rotation, named apart from the
    intent's own blocks."""
    from placement.utility import literal_ref_glob
    name = f'rotation:{ref}'
    k = 2
    while name in taken:
        name, k = f'rotation:{ref}~{k}', k + 1
    return {'name': name, 'refs': [literal_ref_glob(ref)], 'rotation': rot,
            'context': {'source': 'rank_rotations'}}


def with_rotation(raw, ref, rot):
    """A copy of the raw intent document plus `ref`'s rotation block."""
    doc = json.loads(json.dumps(raw))
    blocks = list(doc.get('blocks') or [])
    blocks.append(rotation_block(ref, rot,
                                 {str(b.get('name')) for b in blocks}))
    doc['blocks'] = blocks
    # #893: `rotation` on a block needs reader 5.
    doc['min_reader'] = max(int(doc.get('min_reader') or 0), 5)
    return doc


def eligible_refs(pcb, intent, blocks, pcb_file, *, min_pads=16,
                  seed_args=()):
    """`([(ref, connected pads)], {ref: reason})`, ordered most pads first.

    Always excluded (an explicit --ref among them is a refusal): a part the
    seed never turns -- (locked yes) in the file, `must_lock`, an edge claim,
    a fixed pose, a declared rotation, outside a partially-unplaced board's
    pile without --force -- and a part with no connected pad. Excluded from
    the DEFAULT choice only: a classified part (connectors, mechanical), and
    one with fewer than `min_pads` connected pads."""
    from placement import floorplan
    from placement.part_class import classify_part
    from placement.placement_state import assess_placement
    refs_all = sorted(pcb.footprints)
    must = {r for p in intent.must_lock for r in fnmatch.filter(refs_all, p)}
    edge = {str(c['ref']) for c in intent.edge_claims()}
    fixed = {str(f['ref']) for f in intent.fixed_poses}
    declared = floorplan.rotations_for_ref(intent, blocks)
    outside = set()
    st = assess_placement(pcb, pcb_file)
    if (st.partially_unplaced and not st.unplaced
            and '--force' not in seed_args):
        outside = set(refs_all) - set(st.stacked_suspect_refs)
    hard: dict = {}
    soft: dict = {}
    pads_of = {}
    for ref in refs_all:
        fp = pcb.footprints[ref]
        pads_of[ref] = sum(1 for p in fp.pads if p.net_id > 0)
        if getattr(fp, 'locked', False):
            hard[ref] = 'locked'
        elif ref in must:
            hard[ref] = 'must_lock'
        elif ref in edge:
            hard[ref] = 'edge_claim'
        elif ref in fixed:
            hard[ref] = 'fixed_pose'
        elif ref in declared:
            hard[ref] = 'declared_rotation'
        elif ref in outside:
            hard[ref] = 'not_in_seed_scope'
        elif not pads_of[ref]:
            hard[ref] = 'padless'
        elif classify_part(fp, ref).name is not None:
            soft[ref] = f'class:{classify_part(fp, ref).name}'
        elif pads_of[ref] < min_pads:
            soft[ref] = f'pads<{min_pads}'
    cands = sorted(((r, pads_of[r]) for r in refs_all
                    if r not in hard and r not in soft),
                   key=lambda rp: (-rp[1], rp[0]))
    return cands, {'hard': hard, 'soft': soft}


def _median(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def aggregate(rot, rows, ladder_index):
    """One rotation's seeds folded into the record `rotation_key` ranks."""
    hard = next((r['hard_fail'] for r in rows if r.get('hard_fail')), None)
    fails = [(r.get('probe') or {}).get('failures') for r in rows]
    probed = any(f is not None for f in fails)

    def _spread(key):
        vals = {r['seed']: r.get(key) for r in rows}
        got = [v for v in vals.values() if v is not None]
        return {'median': _median(got), 'min': min(got) if got else None,
                'max': max(got) if got else None, 'by_seed': vals}
    return {'rotation': rot, 'ladder_index': ladder_index, 'hard_fail': hard,
            'gated_seeds': sum(1 for r in rows if r.get('gated')),
            'declined_seeds': sum(1 for r in rows
                                  if r.get('reseat_declined_ref')),
            'unseated_max': max((r.get('unseated') or 0) for r in rows),
            'probed': probed,
            'probe_failures': _median(fails) if probed else None,
            'crossings': _spread('crossings'), 'hpwl': _spread('hpwl'),
            'grade_errors': _spread('grade_errors')}


def rotation_key(agg):
    """Lower is better: hard fails last, then fewer seeds whose re-seat
    declined the part at this angle (#1117), then fewer unseated parts, then a
    probe verdict (a probed rotation before an unprobed one, fewer failures
    first), then crossings, hpwl and grade errors (medians over the seeds),
    then the ladder order, which puts the input rotation first on a tie."""
    big = float('inf')

    def _m(key):
        v = agg[key]['median'] if isinstance(agg[key], dict) else agg[key]
        return big if v is None else v
    return (agg['hard_fail'] is not None,
            agg.get('declined_seeds', 0),
            agg['unseated_max'],
            0 if agg['probed'] else 1,
            agg['probe_failures'] if agg['probed'] else 0,
            _m('crossings'),
            _m('hpwl'),
            _m('grade_errors'),
            agg['ladder_index'])


def classify_row(row, ref, written_rot):
    """Fill `ref_seated`, `rotation_applied` and `hard_fail` on a seed row."""
    row['ref_seated'] = (ref not in (row.get('unseated_refs') or ())
                         and ref not in (row.get('rotation_unseated') or {}))
    row['written_rotation'] = written_rot
    row['rotation_applied'] = (written_rot is not None
                               and same_angle(written_rot, row['rotation']))
    # #1117: the angle held, but the seed's post-polish re-seat could not put
    # the part back (into its zone, out of a keep-out or another block's
    # exclusive zone) at it. The polish walked it out BECAUSE that lowered its
    # cost, so ranked on crossings and hpwl alone this arm would beat an
    # angle whose seed satisfies its intent. A TIER, not a hard fail: a zone
    # too full to take the part at ANY angle says nothing about the rotation,
    # and as a hard fail it refused to rank anything at all.
    row['reseat_declined_ref'] = ref in (row.get('reseat_declined') or {})
    if row['place_seed_rc'] not in PLACE_SEED_OK:
        row['hard_fail'] = 'place_seed_failed'
    elif not row['ref_seated']:
        row['hard_fail'] = 'ref_unseated'
    elif not row['rotation_applied']:
        row['hard_fail'] = 'rotation_not_applied'
    elif row.get('crossings') is None:
        row['hard_fail'] = 'no_metrics'
    else:
        row['hard_fail'] = None
    return row


def _best_seed_row(rows):
    ok = [r for r in rows if not r.get('hard_fail')]
    if not ok:
        return None

    def _k(r):
        f = (r.get('probe') or {}).get('failures')
        return (f is None, f if f is not None else 0, r.get('crossings'),
                r.get('hpwl'), r['seed'])
    return min(ok, key=_k)


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.input_file = os.path.abspath(args.input_file)
    args.intent = os.path.abspath(args.intent)
    args.out_dir = os.path.abspath(args.out_dir)
    args.seed_args = _split_forwarded(args.seed_args)
    args.route_args = _split_forwarded(args.route_args)
    _opts = place_seed_options()
    _resolved = {a: resolve_option(a, _opts) for a in args.seed_args
                 if a.startswith('--')}
    bad = [f"{a} (= {r})" if r != a else a for a, r in _resolved.items()
           if r in FORBIDDEN_SEED_ARGS or r == '<ambiguous>']
    if bad:
        parser.error(f"--seed-args may not carry {', '.join(bad)}: the "
                     f"ranker sets --seed, --intent and --group-by itself, "
                     f"and every arm is one polished seed (no --no-polish, "
                     f"--repair*, --reseat* or --dry-run); an ambiguous "
                     f"abbreviation is refused too")
    args.seed_force = any(r == '--force' for r in _resolved.values())
    if len(set(args.seeds)) != len(args.seeds):
        parser.error("--seeds has duplicates")
    if args.rotations and len({_norm(r) for r in args.rotations}) != len(
            args.rotations):
        parser.error("--rotations has duplicates (modulo 360)")
    if args.probe_top < 1:
        parser.error("--probe-top must be at least 1")
    # Output paths are checked BEFORE an hour of seeding, not after.
    if args.write_best and not args.write_best.endswith('.kicad_pcb'):
        parser.error("--write-best must name a .kicad_pcb file")
    for _flag, _out in (('--write-best', args.write_best),
                        ('--write-intent', args.write_intent),
                        ('--json-out', args.json_out)):
        _d = os.path.dirname(os.path.abspath(_out)) if _out else None
        if _d and not os.path.isdir(_d):
            parser.error(f"{_flag}: no such directory {_d}")

    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.groups import GroupError, parse_sources
    from placement.provenance import file_pose_digest
    try:
        sources = parse_sources(args.group_by)
    except GroupError as exc:
        parser.error(str(exc))
    try:
        with open(args.intent, encoding='utf-8') as fh:
            raw = json.load(fh)
        intent = floorplan.intent_from_dict(raw, args.intent)
    except (OSError, ValueError, floorplan.IntentError) as exc:
        print(f"rank_rotations: cannot load intent {args.intent}: {exc}",
              file=sys.stderr)
        return 2
    pcb = parse_kicad_pcb(args.input_file)
    blocks, _probs = floorplan.resolve_blocks(intent, pcb, sources)
    cands, excluded = eligible_refs(pcb, intent, blocks, args.input_file,
                                    min_pads=args.min_pads,
                                    seed_args=(['--force'] if args.seed_force
                                               else []))
    os.makedirs(args.out_dir, exist_ok=True)
    try:
        from redo_record import record_invocation
        record_invocation()
    except Exception:
        pass

    doc = {'tool': 'rank_rotations', 'input': args.input_file,
           'intent': args.intent, 'ref': None, 'ref_selected_by': None,
           'eligibility': {'min_pads': args.min_pads,
                           'candidates': [list(c) for c in cands[:5]],
                           'excluded': excluded['hard']},
           'seeds': args.seeds, 'seed_args': args.seed_args,
           'ignore_nets': args.ignore_nets or [],
           'route_args': args.route_args,
           'probe': {'enabled': args.probe, 'top': args.probe_top,
                     'nets': probe_nets(args.ignore_nets)},
           'rows': [], 'rotations': [], 'ranking': [], 'best_rotation': None,
           'control': None,
           'best': None, 'separated': None, 'probe_overrode_crossings': None,
           'written': {}, 'refused': None, 'exit_code': None}

    def _finish(code, refused=None, message=None):
        doc['refused'], doc['exit_code'] = refused, code
        if message:
            print(f"rank_rotations: {message}", file=sys.stderr)
        path = args.json_out or os.path.join(args.out_dir, 'rotations.json')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
        print(f"Wrote {path}")
        best = doc['best'] or {}
        inp = next((a for a in doc['rotations']
                    if doc.get('input_rotation') is not None
                    and same_angle(a['rotation'], doc['input_rotation'])),
                   None)
        print("JSON_SUMMARY: " + json.dumps({
            'ref': doc['ref'], 'input_rotation': doc.get('input_rotation'),
            'best_rotation': doc['best_rotation'],
            'ranking': doc['ranking'],
            'best_crossings': best.get('crossings'),
            'input_crossings': (inp or {}).get('crossings', {}).get('median'),
            'control_crossings': (doc.get('control') or {}).get('crossings'),
            'best_gated_seeds': best.get('gated_seeds'),
            'best_grade_errors': best.get('grade_errors'),
            'separated': doc['separated'],
            'probed': sum(1 for r in doc['rows'] if r.get('probe')),
            'hard_failed': sorted({a['rotation'] for a in doc['rotations']
                                   if a['hard_fail']}),
            'refused': refused, 'out_dir': args.out_dir,
            'exit_code': code}, sort_keys=True))
        return code

    if args.ref is not None:
        if args.ref not in pcb.footprints:
            parser.error(f"{args.ref} names nothing on this board")
        why = excluded['hard'].get(args.ref)
        if why is not None:
            text = {
                'locked': "is (locked yes) in the file -- the seeder never "
                          "turns a locked part",
                'must_lock': "is in the intent's must_lock -- the seeder "
                             "never turns it",
                'edge_claim': "is an edge connector claim -- its rotation "
                              "comes from its edge",
                'fixed_pose': "has a fixed pose -- its rotation is declared",
                'declared_rotation': "already has a declared rotation in the "
                                     "intent",
                'not_in_seed_scope': "is outside the pile place_seed seeds "
                                     "on this partially-unplaced board (pass "
                                     "--seed-args='--force' to re-seed every "
                                     "unlocked part)",
                'padless': "has no connected pad -- its rotation changes no "
                           "metric"}[why]
            doc['ref'] = args.ref
            return _finish(4, why, f"{args.ref} {text}")
        ref, sel = args.ref, 'argument'
    else:
        if not cands:
            return _finish(4, 'no_eligible_ref',
                           f"no part is eligible to rank (unlocked, "
                           f"undeclared, not a connector, at least "
                           f"{args.min_pads} connected pads)")
        ref, sel = cands[0][0], 'default'
    doc['ref'], doc['ref_selected_by'] = ref, sel
    input_rot = _norm(pcb.footprints[ref].rotation or 0.0)
    rots = candidate_rotations(input_rot, args.rotations,
                               args.diagonal_rotations)
    doc['input_rotation'], doc['candidates'] = input_rot, rots
    print(f"rank_rotations: {ref} ({dict(cands).get(ref, '?')} connected "
          f"pad(s), selected by {sel}), input rotation {input_rot:g}; "
          f"ranking {', '.join(f'{r:g}' for r in rots)} over seed(s) "
          f"{', '.join(map(str, args.seeds))}")

    # Every arm's intent is written and LOADED before any seed runs, so a
    # malformed arm refuses up front instead of after an hour of seeding.
    arm_intent = {}
    for rot in rots:
        path = os.path.join(args.out_dir, f'rot_{rot:g}.intent.json')
        arm = with_rotation(raw, ref, rot)
        try:
            floorplan.intent_from_dict(arm, path)
        except floorplan.IntentError as exc:
            return _finish(2, 'arm_intent',
                           f"the intent with {ref} at {rot:g} does not load: "
                           f"{exc}")
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(arm, fh, indent=1, sort_keys=True)
            fh.write('\n')
        arm_intent[rot] = path

    seed_args = list(args.seed_args) + ['--group-by', args.group_by]
    if args.diagonal_rotations and '--diagonal-rotations' not in seed_args:
        seed_args.append('--diagonal-rotations')

    # The CONTROL: the same seeds with the intent as given, no rotation
    # declared -- the seed the caller would have got. Never ranked.
    control_rows = []
    for seed in args.seeds:
        out = os.path.join(args.out_dir, f'control_seed_{seed}.kicad_pcb')
        print(f"[{ref} undeclared (control), seed {seed}] place_seed -> "
              f"{os.path.basename(out)}")
        r, s = run_place_seed(args.input_file, args.intent, seed, out,
                              ignore_nets=args.ignore_nets,
                              seed_args=seed_args)
        if r.returncode == PLACE_SEED_PLAN_REFUSED:
            return _finish(4, 'plan_check',
                           "the zone plan was refused before any seed was "
                           "written (place_seed exit 5): "
                           + (r.stderr or r.stdout)[-300:].strip())
        if r.returncode == PLACE_SEED_PLACED:
            return _finish(3, 'board_placed',
                           "place_seed will not seed this board (exit 3: it "
                           "looks placed) -- pass --seed-args='--force' to "
                           "re-seed every unlocked part")
        fp = (parse_kicad_pcb(out).footprints.get(ref)
              if r.returncode in PLACE_SEED_OK and os.path.isfile(out)
              else None)
        control_rows.append({
            'seed': seed, 'board': out, 'place_seed_rc': r.returncode,
            'gated': r.returncode == 4, 'crossings': s.get('crossings'),
            'hpwl': s.get('hpwl'), 'unseated': s.get('unseated'),
            'grade_errors': s.get('grade_errors'),
            'written_rotation': (_norm(fp.rotation or 0.0)
                                 if fp is not None else None),
            'pose_digest': file_pose_digest(out) if fp is not None else None})
        print(f"    crossings {s.get('crossings')}  hpwl {s.get('hpwl')}  "
              f"{ref} written at {control_rows[-1]['written_rotation']}")
    doc['control'] = {
        'rows': control_rows,
        'crossings': _median([c['crossings'] for c in control_rows]),
        'hpwl': _median([c['hpwl'] for c in control_rows]),
        'rotations': sorted({c['written_rotation'] for c in control_rows
                             if c['written_rotation'] is not None})}

    by_rot = {rot: [] for rot in rots}
    for rot in rots:
        for seed in args.seeds:
            out = os.path.join(args.out_dir,
                               f'rot_{rot:g}_seed_{seed}.kicad_pcb')
            print(f"[{ref} @ {rot:g}, seed {seed}] place_seed -> "
                  f"{os.path.basename(out)}")
            r, s = run_place_seed(args.input_file, arm_intent[rot], seed,
                                  out, ignore_nets=args.ignore_nets,
                                  seed_args=seed_args)
            row = {'rotation': rot, 'delta': _norm(rot - input_rot),
                   'seed': seed, 'board': out, 'intent': arm_intent[rot],
                   'place_seed_rc': r.returncode,
                   'gated': r.returncode == 4,
                   'grade_errors': s.get('grade_errors'),
                   'unseated': s.get('unseated'),
                   'unseated_refs': s.get('unseated_refs') or [],
                   'rotation_unseated': s.get('rotation_unseated') or {},
                   # #1117: the parts the seed's post-polish re-seat could not
                   # put back at their declared angle. `classify_row` flags the
                   # ranked ref being one of them, and `rotation_key` ranks such
                   # an angle as a TIER -- after every angle whose seeds held
                   # the part, never eliminated.
                   'reseat_declined': s.get('reseat_declined') or {},
                   'pad_conflicts_seeded': s.get('pad_conflicts_seeded'),
                   'decap_claimed': (s.get('decap_stage') or {}).get(
                       'claimed'),
                   'decap_claimed_late': ((s.get('decap_stage') or {}).get(
                       'late') or {}).get('claimed'),
                   'crossings': s.get('crossings'), 'hpwl': s.get('hpwl'),
                   'probe': None, 'pose_digest': None}
            written = None
            if r.returncode in PLACE_SEED_OK and os.path.isfile(out):
                fp = parse_kicad_pcb(out).footprints.get(ref)
                written = (fp.rotation or 0.0) if fp is not None else None
                row['pose_digest'] = file_pose_digest(out)
            else:
                row['note'] = ('place_seed failed: '
                               + (r.stderr or r.stdout)[-300:].strip())
            classify_row(row, ref, written)
            print(f"    crossings {row['crossings']}  hpwl {row['hpwl']}  "
                  f"unseated {row['unseated']}  grade errors "
                  f"{row['grade_errors']}"
                  + (f"  HARD FAIL: {row['hard_fail']}"
                     if row['hard_fail'] else ''))
            by_rot[rot].append(row)
            doc['rows'].append(row)

    aggs = [aggregate(rot, by_rot[rot], i) for i, rot in enumerate(rots)]
    if args.probe:
        pre = sorted((a for a in aggs if a['hard_fail'] is None),
                     key=rotation_key)
        for a in pre[:args.probe_top]:
            for row in by_rot[a['rotation']]:
                print(f"[{ref} @ {a['rotation']:g}, seed {row['seed']}] "
                      f"probing full board")
                row['probe'] = probe_full(row['board'],
                                          probe_nets(args.ignore_nets),
                                          args.route_args)
                pr = row['probe']
                print(f"    probe: {pr.get('failures')} failure(s) "
                      f"({pr.get('status')}: {pr.get('note')})")
        aggs = [aggregate(rot, by_rot[rot], i) for i, rot in enumerate(rots)]
    ranked = sorted(aggs, key=rotation_key)
    for i, a in enumerate(ranked):
        a['rank'] = i + 1
        a['key'] = [str(k) for k in rotation_key(a)]
    doc['rotations'] = ranked
    doc['ranking'] = [a['rotation'] for a in ranked]

    print(f"\n{'rotation':>8}  {'rank':>4}  {'crossings':>9}  {'hpwl':>9}  "
          f"{'unseated':>8}  {'errors':>6}  {'probe':>5}  hard fail")
    for a in ranked:
        print(f"{a['rotation']:>8g}  {a['rank']:>4}  "
              f"{str(a['crossings']['median']):>9}  "
              f"{str(a['hpwl']['median']):>9}  {a['unseated_max']:>8}  "
              f"{str(a['grade_errors']['median']):>6}  "
              f"{str(a['probe_failures'] if a['probed'] else '-'):>5}  "
              f"{a['hard_fail'] or ''}")

    winner = ranked[0] if ranked and ranked[0]['hard_fail'] is None else None
    if winner is None:
        return _finish(4, 'all_hard_failed',
                       f"no rotation of {ref} could be ranked -- "
                       + '; '.join(f"{a['rotation']:g}: {a['hard_fail']}"
                                   for a in ranked))
    ok = [a for a in ranked if a['hard_fail'] is None]
    if len(ok) > 1 and len(args.seeds) > 1:
        w, r2 = ok[0]['crossings'], ok[1]['crossings']
        doc['separated'] = (None if w['max'] is None or r2['min'] is None
                            else w['max'] < r2['min'])
        by_x = min(ok, key=lambda a: (a['crossings']['median']
                                      if a['crossings']['median'] is not None
                                      else float('inf'), a['ladder_index']))
        doc['probe_overrode_crossings'] = (winner['probed']
                                           and by_x is not winner)
    elif len(ok) > 1:
        by_x = min(ok, key=lambda a: (a['crossings']['median']
                                      if a['crossings']['median'] is not None
                                      else float('inf'), a['ladder_index']))
        doc['probe_overrode_crossings'] = (winner['probed']
                                           and by_x is not winner)
    best_row = _best_seed_row(by_rot[winner['rotation']])
    doc['best_rotation'] = winner['rotation']
    doc['best'] = {'rotation': winner['rotation'], 'seed': best_row['seed'],
                   'board': best_row['board'],
                   'crossings': winner['crossings']['median'],
                   'hpwl': winner['hpwl']['median']}
    doc['best']['gated_seeds'] = winner['gated_seeds']
    doc['best']['grade_errors'] = winner['grade_errors']['median']
    ctl = doc['control']
    print(f"\nbest rotation for {ref}: {winner['rotation']:g} -- crossings "
          f"{ctl['crossings']} -> {winner['crossings']['median']}, hpwl "
          f"{ctl['hpwl']} -> {winner['hpwl']['median']} against the "
          f"undeclared seed ({ref} written at "
          + (', '.join(f"{r:g}" for r in ctl['rotations']) or '?') + ")"
          + (f", probe failures {winner['probe_failures']}"
             if winner['probed'] else ''))
    if winner['gated_seeds']:
        print(f"  its seed fails its intent gate on {winner['gated_seeds']} "
              f"of {len(args.seeds)} seed(s) (grade errors "
              f"{winner['grade_errors']['median']}): repair it "
              f"(place_seed --repair) or weigh the next angle")
    if len(args.seeds) == 1:
        print("  one seed: the margin has no spread to compare against; add "
              "--seeds 0 1 2 to see one")
    if args.write_best:
        from copy_board import copy_board
        copy_board(best_row['board'], os.path.abspath(args.write_best))
        doc['written']['best_board'] = os.path.abspath(args.write_best)
        print(f"Wrote {args.write_best} (rotation {winner['rotation']:g}, "
              f"seed {best_row['seed']})")
    if args.write_intent:
        with open(args.write_intent, 'w', encoding='utf-8') as fh:
            json.dump(with_rotation(raw, ref, winner['rotation']), fh,
                      indent=1, sort_keys=True)
            fh.write('\n')
        doc['written']['intent'] = os.path.abspath(args.write_intent)
        print(f"Wrote {args.write_intent} ({ref} declared at "
              f"{winner['rotation']:g})")
    return _finish(0)


if __name__ == "__main__":
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())
