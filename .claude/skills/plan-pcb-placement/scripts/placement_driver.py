#!/usr/bin/env python3
"""The placement workflow, one stage at a time.

A skill document is read all at once, which is how a 4,900-line one produced
executors that skimmed the gate and improvised the ladder. This script is the
tape head instead: you ask for one stage, it prints that stage's instructions
and nothing else, and it REFUSES to print a later stage until you hand it the
evidence the earlier one was supposed to produce.

The refusal is the point. A gate written in prose is a sentence someone skims;
a gate that withholds the next instructions cannot be skimmed past.

    python3 -X utf8 <this> --stage P0 --board b.kicad_pcb
    python3 -X utf8 <this> --stage P3 --board b.kicad_pcb --drc-json wk/drc0.json
    python3 -X utf8 <this> --list
    python3 -X utf8 <this> --dump-all          # every stage, all branches
    python3 -X utf8 <this> --self-test

Output carries exactly three tags:

    <stage_instructions>  act on these yourself
    <subagent_prompt>     copy VERBATIM into a subagent; do NOT follow it
    <error>               you skipped evidence; go produce it

Exit: 0 emitted, 2 usage, 4 a guard refused.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'driver'}

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SKILL_DIR = os.path.dirname(HERE)
#: The reference pages live with the combined skill; there is no
#: plan-pcb-routing/references directory, so the old path resolved to
#: nothing and every stage that cited a reference cited a missing file.
REFS = os.path.join(os.path.dirname(SKILL_DIR),
                    'plan-pcb-placement-and-routing', 'references')


# --------------------------------------------------------------------------
# evidence loading -- a guard reads the real artifact, never a pasted claim
# --------------------------------------------------------------------------

def _load(path, what):
    """(data, error_text). A guard that cannot read its evidence refuses."""
    if not path:
        return None, (f'{what} not provided. Produce it first, then re-run '
                      f'this stage with the flag that names it.')
    if not os.path.isfile(path):
        return None, f'{what}: no such file: {path}'
    try:
        with open(path, encoding='utf-8') as fh:
            return json.load(fh), None
    except Exception as exc:                                # noqa: BLE001
        return None, f'{what}: unreadable ({type(exc).__name__}: {exc})'


def err(text):
    return f'<error>\n{text}\n</error>'


# --------------------------------------------------------------------------
# the stages
# --------------------------------------------------------------------------

def p_brief(a):
    """Read what was DECLARED before measuring what is there (#711)."""
    return f'''<stage_instructions stage="P-brief" name="what the board is FOR" of="{len(STAGES)}">
Every other stage here MEASURES the board. This one asks what the board is
supposed to be, because the two most consequential placement facts -- which
edge a connector belongs on, and where along it -- are not in the board file
and never will be.

Without a declaration the toolchain infers them. `check_floorplan --emit-intent`
reads each connector's edge off its CURRENT POSE (`_nearest_edge`), so an
intent emitted from a damaged board records the damage as the requirement. That
is not a bug in the emitter; it is the only thing it can do with no spec.

1. READ what is already declared, and what is not:

     python3 -X utf8 py_tools/board_brief.py {a.board} --json wk/brief0.json

   The `design_brief` section is the only DECLARED one in that document.
   Everything else -- including `mechanical` -- is inference. If it says
   NONE DECLARED, that sentence is the finding.

2. If a `<board>.design-brief.json` sibling exists, `check_floorplan.py` and
   `board_brief.py` discover it without a flag, and `--emit-intent` COMPILES
   it into the intent every later step already reads through `--intent`. You
   do not hand it to the placement CLIs directly; they have no `--brief`.

3. If none exists, ASK -- and ask only what cannot be inferred. In priority
   order, because these are the questions whose answers change a placement:

     a. For each connector: which EDGE, and is it centred on that edge?
        (Every connector-shaped defect a human has flagged on this toolchain's
        output was one of these two facts being wrong.)
     b. What is this board physically, and how is it held or mounted?
        (`product.form_factor`, `primary_axis` -- without a datum, "off
        centre" has no meaning.)
     c. Are there zones nothing may enter? (`keepouts` -- an enclosure rib, a
        battery, an antenna clearance. These are GRADED and the seat search
        honours them, and nothing but a human can state one.)
     d. Which parts must sit within N mm of WHICH, and why? (`proximity` --
        a crystal to its load pins, bulk caps to the regulator they feed, a
        transistor pair that must stay matched. Name the pair, the limit in
        mm, and the requirement it came from.) This is the one constraint
        the NETLIST implies and no instrument here can read: a 3mm crystal
        loop and a 30mm one have identical connectivity. The decap rules
        cannot stand in for it -- they elect their own partner and need >= 4
        copper pads, so a 3-pad regulator can never be one at any radius.

   Write the answers to `<board>.design-brief.json`. See docs/design-brief.md;
   the minimum is three fields and one row per connector, and "unknown" is a
   legal answer that is reported rather than guessed.

4. If nobody can answer, say so on the record and continue:
   `--waive brief:<why>`. A waived stage is a stated gap, not a silent one.

Next: python3 -X utf8 {sys.argv[0]} --stage P0 --board {a.board}
</stage_instructions>'''


def _p0_reading(a):
    """What the two instruments SAY, once they have been produced (#937).

    P0 asks the reader to "say which row you are in" over a five-row table
    whose first and fourth rows are arithmetic on two numbers this stage's own
    flags already name -- and P0 never opened either file. Judgement spent on
    arithmetic is judgement not spent on the board, so the driver does the
    arithmetic and the reader disposes.

    WHAT IT DOES NOT DO IS PICK THE ROW. Two of the five -- `unplaced` and
    `board carries copper` -- are not derivable from these documents at all
    (P1's own text says an exit code does not test placedness, and neither
    file carries a track or via count), so a driver that announced a row would
    be announcing one it cannot see. It reports what each instrument reads and
    NAMES what it cannot.

    Returns '' when neither document was supplied -- the first-entry case,
    where there is nothing to read yet.
    """
    if not a.drc_json and not a.assembly_json:
        return ''
    drc = _load(a.drc_json, 'drc')[0] if a.drc_json else None
    asm = _load(a.assembly_json, 'assembly')[0] if a.assembly_json else None
    rows, v, b = [], None, None
    if a.drc_json:
        v = _dig(drc, 'violations')
        if v is None:
            v = _dig(drc, 'total_violations')
        if isinstance(v, list):
            v = len(v)
        rows.append('  check_drc       : ' + (
            f'{v} violation(s) on the copper-free board'
            if isinstance(v, int) else
            f'NO violation count in {a.drc_json} -- that file answers nothing'))
    if a.assembly_json:
        # THE VERDICT, not `blocking`. Since #918 `blocking` is one of five
        # not_buildable conjuncts, so a board unbuildable through a
        # coincident-origin stack or a containment reads `blocking` 0 -- and
        # one tracked board does exactly that.
        b = _dig(asm, 'buildable')
        # "[blocking 0, 1 of its 5 conjuncts]" PARSED BACKWARDS: beside a count
        # it reads as "one of the five fired". Worse on a healthy board, where
        # check_assembly writes the verdict as the literal string "buildable
        # (blocking 0)" -- the line became "buildable (blocking 0)  [blocking
        # 0, 1 of its 5 conjuncts]", whose plain reading is "buildable BECAUSE
        # blocking is 0", the exact inference #918 exists to kill.
        rows.append('  check_assembly  : '
                    + str(_dig(asm, 'verdict') or 'no verdict recorded')
                    + (f'   (blocking {_dig(asm, "blocking")} is only ONE of '
                       f'the five conjuncts this verdict is made of -- act on '
                       f'the verdict, never on the count)'
                       if _dig(asm, 'blocking') is not None else ''))
        oob = _dig(asm, 'oob_pad_copper_count')
        if isinstance(oob, int) and oob > 0:
            # NAME THE PARTS: a count is not something a reader can act on,
            # which is the argument the off-outline refusal itself makes, and
            # the refs sit one key away in the same document.
            _refs = [r[0] if isinstance(r, (list, tuple)) else r
                     for r in (_dig(asm, 'oob_pad_copper_refs') or [])]
            rows.append(f'  ...and PAD COPPER OFF THE OUTLINE on {oob} '
                        f'part(s): '
                        + (', '.join(str(r) for r in _refs[:8]) or
                           'see oob_pad_copper_refs'))
            # ...and give it a row. The five-row table below has none for this
            # finding, so a board with drc 0 + buildable true + copper off the
            # outline landed on "both clean -> hand it to routing and stop",
            # carrying the defect that produces 100% of unrouted nets.
            rows.append('                    This OUTRANKS every row below: '
                        'those nets cannot be routed at all. Re-seat them '
                        '(P2 -> P3) before you classify. If they are '
                        'castellations or a declared card edge, the crossing '
                        'is by design -- say so and carry on.')
    clash = ''
    if isinstance(v, int) and isinstance(b, bool) and (v == 0) != b:
        clash = (f'\nTHE TWO DISAGREE: check_drc reads {v} violation(s) and '
                 f'check_assembly reads '
                 f'{"buildable" if b else "NOT BUILDABLE"}. Say which you are '
                 f'acting on, and why, before you move a part.\n')
    elif a.assembly_json and b is None:
        clash = ('\nThat assembly document carries no `buildable` field, so '
                 'the two could not be compared. Treat the VERDICT string '
                 'above as authoritative.\n')
    # "dispose of any disagreement above" used to print even when nothing
    # disagreed -- and a reader hunting for the disagreement they were told to
    # dispose of finds `NOT BUILDABLE` beside `blocking 0` and "resolves" it by
    # trusting the count. Only ask when there is something to ask about.
    return ('\nWHAT THE INSTRUMENTS SAY about the files you named -- the row '
            'is still yours to pick:\n\n' + '\n'.join(rows) + '\n' + clash +
            '\nNeither document says whether this board is UNPLACED or '
            'whether it CARRIES COPPER. Both are in one more command:\n'
            '  python3 -X utf8 py_tools/board_brief.py ' + str(a.board) +
            ' --json wk/brief0.json\n'
            'Read `unplaced` / `partially_unplaced` for the first, and '
            '`has_copper` / `segments` / `vias` for the second.\n\n'
            + ('Name your row, and dispose of the disagreement above.\n'
               if clash else 'Name your row.\n'))


def p0(a):
    """Decide whether to touch the placement at all."""
    return f'''<stage_instructions stage="P0" name="gate" of="{len(STAGES)}">
MEASURE this board's placement, then decide. Do not decide first.

The measurement is two commands and it is never optional. Skipping it is how a
board with two parts stacked on each other reaches routing -- every other
instrument in the chain looks at copper, and there is no copper yet.

Run BOTH, on the board with its copper removed. Neither alone is enough: the
first cannot see two parts stacked on the same net, the second is the channel
that can. Copper still on? `copy_board.py` it, `route.py <copy> <copy> --nets '*'
--undo` (keeps the .kicad_pro floor; leaves zone pours -- --ignore-nets those),
measure THAT. Never stress/strip_routing.py -- non-negotiable 2 forbids it.

  python3 -X utf8 py_router/check_drc.py {a.board} --clearance <the board's own floor> --clearance-margin 0 --json wk/drc0.json
  echo "EXIT=$?"
  python3 -X utf8 py_tools/check_assembly.py {a.board} --json wk/assembly0.json
  echo "EXIT=$?"

check_assembly now READS THE BOARD when --clearance is omitted (its Default
net-class clearance, else routing_defaults), and prints the value with its
source. Passing the board's own floor explicitly is therefore redundant, and
passing anything ELSE is now an override it will warn about. It used to default
to a flat 0.25 regardless of the board, which graded stricter than the thing it
was grading -- measured, one board: pad_conflicts 96 at the default vs 39 at
its own floor.

Pass --clearance only when you deliberately want a floor OTHER than the
board's, and never a round number you chose because it looked plausible. Check
the printed `[board netclass]` / `[fixed default]` tag: `fixed default` means
the board declared nothing and the number is a fallback, not agreement.

Every violation a COPPER-FREE board returns is a placement defect that no
router can remove.

{_p0_reading(a)}
Then classify by what you MEASURED, and say which row you are in:

  both clean                  -> the placement is fit. Do not run a pass over
                                 it: an optimizer on a placement that already
                                 passes makes it worse (measured -- the default
                                 weights caused two new routing failures). Hand
                                 the board to /plan-pcb-routing and stop here.
                                 This is a verdict you reached, not a default
                                 you assumed.
  board carries copper        -> STOP. Placement moves footprints, not tracks;
                                 the tools exit 3. Re-run from the unrouted board.
  unplaced                    -> P1
  violations, or a mechanically-fixed part where mechanics forbid  -> P2
  rough/imported, all legal   -> P5 (a slate), or P4 for local violations only

Next: python3 -X utf8 {sys.argv[0]} --stage <P2|P5> --board {a.board} \\
          --drc-json wk/drc0.json --assembly-json wk/assembly0.json

Next, for P1 only: it seeds FROM A ZONE PLAN and refuses without one.
  python3 -X utf8 {sys.argv[0]} --stage P1 --board {a.board} --zone-plan <the plan>

Next, for P4 only: it grades DELTAS, so it also needs the pair. It refuses
without both -- an absolute threshold is what made two of its gates unusable.
  python3 -X utf8 {sys.argv[0]} --stage P4 --board {a.board} \\
      --drc-json wk/drc0.json --before <the board this one came from> \\
      --render-json <that pair's render>
</stage_instructions>'''


def p1(a):
    ok, plan = _guard_zone_plan(a)
    if not ok:
        return err(plan)
    return f'''<stage_instructions stage="P1" name="unplaced" of="{len(STAGES)}">
The board has no placement to repair. Do not test this with an exit code -- one
placement tool exits 0 and gives advice on a board with every part at its
generator default. Test positively:

  python3 -X utf8 -c "
from kicad_parser import parse_kicad_pcb
p = parse_kicad_pcb('{a.board}')
print('no outline:', p.board_info.board_bounds is None)
print('stacked at defaults:', len({{(round(f.x,3), round(f.y,3))
      for f in p.footprints.values()}}) < len(p.footprints) / 2)"

ZONE PLAN {a.zone_plan}: all {plan['blocks']} footprint(s) accounted for.
{plan['zoned']} zoned block(s) cover the other {plan['movable']} movable footprint(s); {plan['locked']} claimed
by must_lock or pinned in the file ({plan['pinned']} pinned, the only kind the seeder
cannot move); {plan['edge']} declared edge connector(s), {plan['seeded_edge']} of them the seeder's to
choose; {plan['padless']} pad-less, the seeder never moves them ({plan['padless_locked']} locked, {plan['padless_disposed']} dispositioned).

THE SEEDER PLACES THE RESIDUE. It is a greedy first-fit that packs declared
zones and drops everything else at its connectivity centroid, at the first
rotation that fits -- so it is good at the many small parts and has no
representation at all for a decision. Decide these YOURSELF, above, and lock
them; what is left is what the seeder is for.

Walk the ladder in order and say which rung applies:

1. The repo has its own seeder -> run it, then treat the output as a rough
   placement (P4/P5).
2. Otherwise: DECIDE, then seed the rest.
   a. Place and lock every part whose pose is a decision -- the connectors
      (which edge, where along it, which way the mating face points), the
      mechanically-fixed parts, anything a spec pins. P2 is the stage that
      enumerates them; `place_pose set/rotate/lock` is the verb, and it
      refuses a pose that makes the board's placement legality worse.
   b. Seed the rest FROM THE PLAN, several seeds, and rank only the ones that
      pass their own gate (exit 4 names the rule; none passing is the PLAN's
      problem, not the seeder's -- run 26 had none pass and hand-placed
      everything instead):
       python3 -X utf8 py_placer/compare_seeds.py {a.board} --intent {a.zone_plan} \\
           --seeds 0 1 2 --out-dir wk/seedcmp
   c. Then FACE the rows. The seeder keeps the first rotation that fits, so
      read `edge_facing` per part off the best seed and turn every part whose
      connected pads face the outline with nothing beyond (run 26's regulator:
      3 of 3 pins 0.40 mm from the edge, and a review with no number wrote
      PASS):
       python3 -X utf8 py_placer/placement_score.py wk/seedcmp/seed_<best>.kicad_pcb \\
           --intent {a.zone_plan} --json wk/terms_seed.json
       python3 -X utf8 py_placer/place_pose.py wk/seedcmp/seed_<best>.kicad_pcb \\
           seed.kicad_pcb face <REF> <FACE> <PARTNER>
3. No seed passes, on a rule the plan itself sets -> fix the plan and say so.
   This toolchain does not invent a placement, and inventing mechanical
   geometry is what every rule here forbids.

Next: P4 legalizes the seed, P6 declares the intent first. Both FOLLOW a move,
so both refuse without the render of the seed against the board it came from:
  python3 -X utf8 {sys.argv[0]} --stage <P4|P6> --board seed.kicad_pcb \\
      --before {a.board} --render-json <the seed's render>
</stage_instructions>'''


def p2(a):
    ok, why = _guard_damage(a)
    if not ok:
        return err(why)
    return f'''<stage_instructions stage="P2" name="mechanical facts" of="{len(STAGES)}">
Before any search runs, separate the parts whose position is NOT a netlist
question. Each is placed by a determinant you can name, and an optimizer that
moves them is destroying information.

  class                          determined by            detect on any board
  mounting holes / NPTH          the enclosure pattern    pad_type np_thru_hole,
                                                          or 0 connected pins
  edge connectors, castellations the mating standard      courtyard intersects
                                                          (castellated: centred
                                                          on) the outline
  enclosure-referenced parts     an aperture in the spec  SPEC ONLY -- not
  (USB, barrel, RF, buttons)                              board-derivable
  fiducials, test points         a fab or fixture rule    usually spec

Run the lock advisor and act on it:

  python3 -X utf8 py_placer/place_optimize.py {a.board} --suggest-locks \\
      --suggest-locks-json wk/locks.json

The gate for leaving this stage is `unlocked_high == 0`, or every remaining
finding dispositioned IN WRITING with the reason.

On a DAMAGED board that gate is usually unreachable by locking, and reaching
for it is the trap: a displaced part can carry a high-confidence finding, and
locking it records the damaged pose as a decision every later stage then treats
as ground truth. Disposition instead, per ref, against a measurement. The
advisor demotes a part lying WHOLLY off the board for this reason; a part
hanging PARTLY off an edge it cannot distinguish from a real edge-mounted one,
so that one is yours to judge -- compare its excursion with the other
edge-mounted parts on this board.

A part the FILE marks (locked yes) is never yours to move, whatever any intent
says. Place a part, verify it, then lock it -- never inherit a lock you have
not checked.

Next: python3 -X utf8 {sys.argv[0]} --stage P3 --board {a.board} \\
          --drc-json {a.drc_json or 'wk/drc0.json'} --locks-json wk/locks.json
</stage_instructions>'''


def p3(a):
    ok, why = _guard_damage(a)
    if not ok:
        return err(why)
    locks, lerr = _load(a.locks_json, 'The lock advisor output (--locks-json)')
    if lerr:
        # `--suggest-locks-json`, which is what P2 prints thirty lines up. The
        # `--json` this used to say is not a place_optimize flag at all, so the
        # command a STUCK reader is handed here died at argparse -- and nothing
        # saw it, because --dump-all satisfies every guard and never renders a
        # refusal (#923).
        return err(lerr + '\n\nP2 produces it:\n  python3 -X utf8 '
                          f'py_placer/place_optimize.py {a.board} --suggest-locks '
                          '--suggest-locks-json wk/locks.json')
    high = _dig(locks, 'unlocked_high')
    if isinstance(high, int) and high > 0:
        # --waive was used ONLY for its truthiness: the strings were never
        # parsed, the count was never compared to `unlocked_high` (so one
        # `--waive X:y` unblocked a report of 300), and they were never written
        # anywhere. P3 hard-refuses without them, so a resumed run could not
        # re-enter this stage -- run 9's six waivers survive only because the
        # teammate happened to re-type them into a handover message.
        parsed, malformed = {}, []
        for w in (a.waive or []):
            ref, sep, reason = str(w).partition(':')
            if not sep or not ref.strip() or not reason.strip():
                malformed.append(w)
            else:
                parsed[ref.strip()] = reason.strip()
        if malformed:
            return err(
                f'--waive takes REF:reason, and these do not parse: '
                f'{", ".join(repr(m) for m in malformed)}.\n\nThe reason is the '
                f'point -- a waiver without one is a flag that makes the gate '
                f'go away, which is what this stage exists to prevent.')
        if len(parsed) < high:
            return err(
                f'The lock advisor reports unlocked_high = {high}. Those are '
                f'parts whose position looks load-bearing and which nothing is '
                f'holding, so a search is free to move them.\n\nYou waived '
                f'{len(parsed)} of {high}'
                f'{" (" + ", ".join(sorted(parsed)) + ")" if parsed else ""}. '
                f'Lock them, or re-run with --waive REF:reason for EACH one. '
                f'The count used to go unchecked, so a single waiver cleared '
                f'any number of findings.')
        _wp = os.path.join(os.path.dirname(os.path.abspath(a.locks_json)),
                           'waivers.json')
        try:
            with open(_wp, 'w', encoding='utf-8') as _wf:
                json.dump({'board': os.path.abspath(a.board),
                           'unlocked_high': high, 'waivers': parsed},
                          _wf, indent=1, sort_keys=True)
        except OSError:
            _wp = None
    return f'''<stage_instructions stage="P3" name="reconstruct" of="{len(STAGES)}">
The board is placed WRONG, not merely rough, so the quench is the wrong tool:
it is a local search on a continuous lattice, and what you have is a structural
error. Escalate, do not compose. Each rung has an applicability test; run the
test, and when it fails say so and fall through.

R1  Which parts did P2 establish? Their positions are arithmetic now.
R2  Does the BOARD determine a position? A family whose pattern is
    over-determined by its surviving members predicts the rest. The fit
    PROPOSES; the copper-free measurement DECIDES. Apply a proposal only when
      - the fit is over-determined (>= 2 survivors for a translation,
        >= 3 with rotation or scale), AND
      - the residual collapses to a grid step within a TOLERANCE, not to
        equality, AND
      - applying it improves the copper-free violation count AND does not
        increase the off-board amount.
    Both conjuncts, always: the count alone is gameable by evacuation -- a
    conflict removed by pushing a part off the board reads as an improvement.
    If it does not improve, REVERT: the determinant was not on the board.
R3  Apply with the repair tools, never the from-scratch seeder:
      python3 -X utf8 py_placer/place_seed.py {a.board} r.kicad_pcb --intent fp.json --repair
      python3 -X utf8 py_placer/place_reconstruct.py {a.board} r.kicad_pcb [--intent fp.json]
    Both take --dry-run, --intent, --clearance and --grid-step.

    NO STEP HAS A WALL-CLOCK BUDGET: `--deadline` was removed everywhere (no
    result may depend on timing), so passing it is an argparse error. A harness
    timeout SIGTERMs the tool -- shutdown never runs, exit 143, no partial board
    and no summary. 143 and 124 are the SHELL's codes, not a tool's. Bound long
    steps by SCOPE and run them detached.
R3b A part whose pad CENTRES are off the outline is not repairable by a
    minimal-move sweep, whatever cap you give it: every repair search starts
    from the part's current pose, and that pose carries no information once
    the part is tens of millimetres out. LIFT it instead --
      python3 -X utf8 py_placer/place_seed.py {a.board} r.kicad_pcb --intent fp.json \\
          --reseat --clearance <floor>
    (bare --reseat = auto scope = exactly those parts; name refs/globs to
    scope it yourself). Or as a ladder rung: place_reconstruct --stages
    classify,fit,vector,assign,exchange,reseat,legalize.
    THE NUMBER TO READ IS `witnesses_after`, NOT `repaired`/`reseated`: the
    first is what predicts routability, the second counts effort. Key it off
    render_placement's checklist.a_off_outline.pad_copper channel -- but if you
    are heading for the routing loop, the number that REFUSES there is
    check_assembly's `oob_pad_count`, which is a different census of the same
    idea (part pad AABB vs an outline inflated by the grading clearance, so it
    moves with --clearance). Drive to zero on both (#788).
    Measured, same board: --repair 4m55s and attempted NONE of the 11
    off-board parts; --reseat 13s, 11 of 11, off-outline count to 0.
    Do NOT expect recovery to improve -- it seats by net centroid.
R4  Test for a rigid displacement. Offsets of +v and -v are AGREEMENT, not
    disagreement -- an exchange displaces its two groups by opposite vectors,
    and that pair is the swap's signature. Offsets disagreeing in MAGNITUDE
    mean there is no single rigid displacement: stop, go to P5.
R5  Anchor the large parts, then size the gap between two anchors from the
    SMALL PARTS that live in the corridor, not from the routing that crosses
    it (measured: small-part extent correlates with the gap a human left on
    8 of 8 boards; the routing cut does not correlate at all).

After ANY anchor move, re-run the escape ledger for every neighbouring
fine-pitch part -- a widened corridor can eat another part's escape face.

LOOK AT WHAT YOU MOVED. This stage moves parts, and the next one will not open
without a render of the result -- so produce it here, and READ it. --pair diffs
the findings BY NAME across the two boards, which is what says whether the move
fixed anything or just swapped one conflict for another (a level count hides a
swap):

  python3 -X utf8 py_tools/render_placement.py r.kicad_pcb --before {a.board} --pair \\
      --clearance <the board's own floor> --ignore-nets <the poured nets> \\
      --expect-moved <the count this stage reported> \\
      --review-sheet wk/sheet_p3.png \\
      --json-out wk/render_p3.json -o wk/render_p3.png

It prints WHAT THIS PANEL SHOWS (every finding in words), THE WORST N (one crop
command each) and DECLUTTER (the flags that clear the noise). Run one of the
crops it hands you; that is the whole point of it handing them to you.

Next: python3 -X utf8 {sys.argv[0]} --stage P4 --board r.kicad_pcb \\
          --before {a.board} --drc-json <a fresh copper-free DRC of r> \\
          --render-json wk/render_p3.json
</stage_instructions>'''


def p4(a):
    if not a.before:
        return err('P4 compares a result against what it came from, so it '
                   'needs --before <the board this one was derived from>. '
                   'Without it none of the gates below can be a DELTA, and an '
                   'absolute threshold is what made two of them unusable.')
    if not os.path.isfile(a.before):
        return err(f'--before: no such file: {a.before}')
    _ok, _why = _guard_render(a)
    if not _ok:
        return err(_why)
    return f'''<stage_instructions stage="P4" name="fix loop" of="{len(STAGES)}">
One lap = measure, ONE targeted change, verify. Cap: 5 laps. Anything still
broken at the cap is NAMED with its measurement, not carried silently.

MEASURE (all four, every lap, on the copper-free board):

  python3 -X utf8 py_router/check_drc.py {a.board} --clearance <floor> --clearance-margin 0 --baseline <the board this RUN started from>
  python3 -X utf8 py_tools/check_assembly.py {a.board} --baseline <the board this RUN started from>
  python3 -X utf8 py_tools/check_channels.py {a.board} --baseline <the board this RUN started from> --gate
  python3 -X utf8 check_rigid_consistency.py {a.before} {a.board}

check_assembly and check_channels now READ THE BOARD's own clearance (and
check_channels its track width too) and print each value with its source, so
do NOT pass <floor> to them -- omitting it is what gets the board's floor.
They used to default to a flat 0.25 / 0.3 regardless: on a 0.2 board that
track width invented a "U2 N short 1 lane" deficit that does not exist
(supply 14, demand 12), and it was handed forward as floorplan-shaped residue.
check_drc still wants it spelled out; its --baseline grades a graphic-copper graze
a lap's MOVE created (without it, accepted `unverified`, so a lap grades clean).

READ THE PRINTED SOURCE. `[board netclass]` / `[board constraint]` means the
board answered; `[fixed default]` means it declared nothing and the number is
this tool's fallback -- compare such a run only against boards graded the
same way.

The last three are DELTAS against the board you started from, deliberately.
Each has an absolute form that was measured and refused: a starved escape face,
a courtyard overlap and a contact pair are all routinely properties of the
DESIGN, so the absolute count fires on healthy and human-placed boards alike.
The delta cancels the design term because both boards carry it.

What each one refuses to let pass:
  check_assembly     `buildable: false` -- ANY of its five not_buildable
                     conjuncts: a blocking pad pair (two parts on the same
                     copper), copper on a LOCKED part, a coincident-origin
                     stack, a containment, or a moved-vs-baseline courtyard
                     gate. Read the VERDICT, NOT `blocking == 0`: that scalar
                     is one of the five, so a board unbuildable through a
                     stack or a containment reads 0 and is not buildable
                     (#918)
  check_channels     a face that now carries demand and has LOST its escape
                     relative to the baseline -- all of it, or a large share
                     of it (--min-supply-drop, default 0.20). #847: the
                     zero-crossing form alone missed a face going 43 -> 28
                     lanes against a demand of 12
  check_rigid_...    a new contact between two parts moved by DIFFERENT
                     vectors -- impossible in a rigid restore, so the result is
                     a search that happened to fit, not the damage undone

FIX: one change, aimed at a named finding. The eaten_by refs in the channel
ledger are the move targets; the pair members in the assembly report are the
candidates. Do not batch fixes -- a lap that changes three things cannot tell
you which one worked.

VERIFY: re-run all four. A lap is ACCEPTED only if the finding it aimed at is
gone and nothing above it got worse.

Then record it, before starting the next lap:
  python3 -X utf8 py_placer/converge.py record --ledger wk/ledger.jsonl \\
      --board {a.board} --kind placement --lever "<what you changed and why>" \\
      --argv <the real command that produced this board, as BARE TOKENS>

--argv takes the rest of the line, unquoted, and it must REPLAY: converge
refuses (exit 2) any first token that is not a real file or on PATH, and a
placeholder like "<all but R12>" inside a quoted string makes the whole thing
one unrunnable token. Expand the arguments you actually used.

A lap that did NOT clear its finding is recorded too, with --rejected, and then
stepped back. Keeping it is what makes "two flat laps" detectable -- a loop that
discards its failures cannot tell a plateau from a fresh start -- and the step
back is a checkout of the parent board, siblings included, not a reconstruction:

  python3 -X utf8 py_placer/converge.py step-back --ledger wk/ledger.jsonl \\
      --out <where to put the restored board>

Two flat laps in a row means the residue is floorplan-shaped, not repairable
here: stop and go to P5.

EVERY LAP THAT MOVES A PART OWES A RENDER, and P6/P-close will not open without
one of the board they are handed. Re-render after each accepted lap, against the
board that lap came from:

  python3 -X utf8 py_tools/render_placement.py <this lap> --before <the lap before it> \\
      --pair --clearance <floor> --ignore-nets <poured nets> \\
      --expect-moved <the COUNT of parts this lap moved> \\
      --review-sheet wk/sheet_lapN.png \\
      --json-out wk/render_lapN.json -o wk/render_lapN.png

Read WHAT THE MOVE DID: `N fixed, M NEW` is the lap's verdict. A lap that
introduces findings it did not resolve is a lap to revert, and the count alone
will not tell you -- 46 -> 46 can be nine fixed and nine new somewhere else.

Record it: converge.py record ... --render-json wk/render_lapN.json

Next: P5 for a slate, P6 to declare and grade; then P-close. P6 follows a move,
so it refuses without the render this lap already produced:
  python3 -X utf8 {sys.argv[0]} --stage <P5|P6> --board {a.board} \\
      --before <the board this lap started from> \\
      --render-json wk/render_lapN.json
</stage_instructions>'''


def p5(a):
    return f'''<stage_instructions stage="P5" name="options" of="{len(STAGES)}">
Use this when the question is "which arrangement", not "is this one legal".

  python3 -X utf8 py_placer/place_portfolio.py {a.board} --out-dir wk/slate \\
      --candidates <K> --keep <N> [--full-probe] \\
      --intent <the graded floorplan intent> \
      --lock <the refs the lock advisor printed, plus the board's own (locked yes)>

PASS --intent AND --lock, or rule 1 below grades nothing. The locks are NOT a
P2 artifact -- P2 moves parts and writes no lock list. They come from the lock
advisor (Step 0b: `place_optimize.py <board> --suggest-locks`) and from the
refs the board already stamps
`(locked yes)`, which place_portfolio honours whether or not you name them.
place_portfolio learns the declared intent from --intent and the mechanical
locks from --lock; without them its HARD gate has no constraint to be hard
about, and a step that optimises against no constraint is the failure this
whole procedure exists to stop. Both ARE read when given (`args.lock` reaches
the seeder and the quench); what nothing does is REQUIRE them -- only
--out-dir is required -- so a run with neither still produces a slate, prints
JSON_SUMMARY and exits 0. The refusal is yours to make.

Rank rules, in this order:
  1. HARD gates first: legality and the declared intent. A candidate that fails
     either is not in the running, however good it looks.
  2. Prefer a ranking that ROUTED something over one that only measured the
     placement. A placement metric cannot see the thing you are choosing for.
  3. hpwl and crossings should ANNOTATE the slate rather than rank it -- they
     correlate positively with distance-to-truth on damaged boards, which is
     the measured dependent variable, not routed blocking (#703 measured
     crossings against routed blocking too: it fails its sign rule 5/1 on the
     full sample and passes 6/0 once optimizer-made placements are excluded,
     so neither arm is the answer; docs/placement-predictors.md).
     BUT READ WHAT rank_key ACTUALLY DOES: py_placer/placement/portfolio.py's
     rank_key orders on crossings FIRST, and its own docstring calls that an
     unresolved, disclosed contradiction with this rule. #789 withdrew the
     crossings BAR, not the crossings ORDER. So do not take the printed order
     as agreeing with rule 3 -- read portfolio.json and decide deliberately.

Adopt one deliberately, say why in writing, and re-run P4 on the adopted board.
Adoption is a decision, not a step -- it is not replayable, so it belongs in
the record.

Next: render the adopted board against this one, then re-run the fix loop on it:
  python3 -X utf8 {sys.argv[0]} --stage P4 --board <adopted> \\
      --before {a.board} --render-json <the adopted board's render>
</stage_instructions>'''


def p6(a):
    # Mandate 1: you cannot declare zones for a board you have not looked at.
    # An intent authored off coordinates alone is a guess with a schema.
    _ok, _why = _guard_render(a)
    if not _ok:
        return err(_why)
    return f'''<stage_instructions stage="P6" name="declare the intent" of="{len(STAGES)}">
An intent turns "it looks right" into something gradable.

  python3 -X utf8 py_tools/check_floorplan.py {a.board} --emit-intent wk/intent.json
  # then EDIT it down: the emit describes the board as it is, including its
  # damage. Keep what the SPEC requires; delete what is merely observed.

  python3 -X utf8 py_tools/check_floorplan.py {a.board} --intent wk/intent.json --health

Two traps, both measured:
  - An emitted intent records the board's own geometry as a requirement. On a
    damaged board that bakes the damage in as the target. Entries derived from
    observation are marked suspect for exactly this reason; treat them as
    hypotheses, never as the spec.
  - must_lock is a REQUIREMENT ("these refs must end up locked"), not a
    licence. The file's own (locked yes) stamps are the authority on what may
    move, and they are recorded separately, under context.

A zone that cannot contain a part's courtyard at any rotation is graded on the
part's anchor point instead, and the tool says so.

Next: python3 -X utf8 {sys.argv[0]} --stage P4 --board {a.board} \\
          --before <the board before any change> \\
          --render-json <this board's render, against that one>
</stage_instructions>'''


def p_close(a):
    if not a.before:
        return err('The close-out compares against the board this run started '
                   'from: pass --before <original>.')
    if not os.path.isfile(a.before):
        return err(f'--before: no such file: {a.before}')
    # The last chance to have looked at what is being handed on. Run 9's
    # placement half closed out having read no image at all, and the omission
    # was invisible afterwards because nothing recorded reads.
    _ok, _why = _guard_render(a)
    if not _ok:
        return err(_why)
    # The intent, or a stated reason there is none. Same shape as P3's --waive:
    # a named exemption is a decision on the record; an absent one is a gap
    # nobody can see afterwards.
    _iw = [w for w in (a.waive or []) if w.split(':', 1)[0].strip() == 'intent']
    if not a.intent_json and not _iw:
        return err(
            'The close-out has no graded floorplan intent (--intent-json).\n\n'
            'An intent is what turns "it looks right" into something gradable, '
            'and P6 -- the rung that declares one -- is reachable only as a '
            'side branch off P1/P4, so a run can arrive here having declared '
            'nothing. Not hypothetical: one run shipped an intent with '
            '`rules_run: 0`, which constrained two laps with nothing, then a '
            'second covering 0 of 266 parts. Nothing objected either time.\n\n'
            '  python3 -X utf8 py_tools/check_floorplan.py <board> '
            '--emit-intent wk/intent.json\n'
            '  # then EDIT IT DOWN -- the emit describes the board AS IT IS, '
            'damage included\n'
            '  python3 -X utf8 py_tools/check_floorplan.py <board> '
            '--intent wk/intent.json --require-rules 1 '
            '--json wk/intent_result.json\n\n'
            'If this board genuinely has no spec to declare, put that on the '
            'record instead: --waive intent:<why>.')
    if _iw and not a.intent_json and not (
            _iw[0].split(':', 1)[1].strip() if ':' in _iw[0] else ''):
        return err('--waive intent: needs a REASON after the colon. "No '
                   'intent" with no cause is the gap, not the fix.')
    # --intent-json was tested for TRUTHINESS only: any string satisfied it, so
    # the gate that exists to stop a vacuous intent could itself be satisfied
    # vacuously -- by a path to nothing. Open it, and require that rules
    # actually ran, which is what the error text above already tells the reader
    # to produce (`--require-rules 1`).
    _cov_read = ('no floorplan intent on the record -- waived, and the gap is\n   stated rather than hidden.')
    if a.intent_json:
        _idoc, _ierr = _load(a.intent_json, 'The floorplan intent (--intent-json)')
        if _ierr:
            return err(_ierr + '\n\nThis gate opens the file now; it used to '
                               'accept the ARGUMENT and never the document, so '
                               'a path to nothing satisfied it.')
        _regrade = (f'  python3 -X utf8 py_tools/check_floorplan.py '
                    f'{a.board} --intent <the intent> --require-rules 1 '
                    f'--require-brief-coverage --json wk/intent_result.json')
        # ARM 0 -- THE SHAPE. `check_floorplan --json` writes `rules_run` as a
        # LIST of rule names; the JSON_SUMMARY line writes it as a COUNT. Both
        # are accepted. Anything else, ABSENT INCLUDED, is refused -- because
        # this gate tested `isinstance(int)` against the very document its own
        # help text names, which writes a list, so IT NEVER FIRED. Six rules
        # ran, the pass was clean, and every declared clause was ungraded.
        _ran = _idoc.get('rules_run')
        if isinstance(_ran, bool) or not isinstance(_ran, (int, list, tuple)):
            return err(
                f'That document\'s `rules_run` is {type(_ran).__name__}, which '
                f'this gate cannot read as "how much was graded".\n\n'
                f'`check_floorplan --intent I --json PATH` writes it as a LIST '
                f'of rule names; the JSON_SUMMARY line writes it as a COUNT. '
                f'Both are accepted here. Anything else -- including ABSENT -- '
                f'is not a graded intent, and that is not hypothetical: this '
                f'gate used to test `isinstance(int)` against a document that '
                f'writes a list, so it never fired once.\n' + _regrade)
        _n = len(_ran) if isinstance(_ran, (list, tuple)) else _ran
        # ARM 1 -- NOTHING GRADED. The long-standing refusal, now firing on
        # `rules_run: []` too, which is the shape production can write.
        if _n <= 0:
            return err(
                f'That intent graded {_n} rules, so it constrained nothing.\n\n'
                f'One run shipped `rules_run: 0` and a second covered 0 of 266 '
                f'parts; nothing objected either time, because this gate only '
                f'checked that a path was PASSED. Edit the intent down to the '
                f'clauses this board must satisfy, then:\n' + _regrade)
        # ARM 2 -- NO COVERAGE BLOCK. Refused rather than assumed: "assume
        # covered" is the inert gate again, one key over.
        _cov = _idoc.get('brief_coverage')
        if _cov is not None and not isinstance(_cov, dict):
            return err(
                f'That document\'s `brief_coverage` is '
                f'{type(_cov).__name__}, not an object, so this gate cannot '
                f'read it. A malformed block must REFUSE and not crash: a '
                f'traceback is neither the pass nor the refusal this stage '
                f'promises, and a non-zero exit is not evidence unless it '
                f'names its reason.\n' + _regrade)
        if isinstance(_cov, dict) and _cov.get('schema') not in (None, 1):
            return err(
                f'That `brief_coverage` block declares schema '
                f'{_cov.get("schema")!r} and this gate reads schema 1. A '
                f'schema this stage has never seen is not something to grade '
                f'optimistically -- reading a newer block with older rules is '
                f'the inert gate again, one key over.\n' + _regrade)
        if isinstance(_cov, dict) and not isinstance(
                _cov.get('clauses', []), list):
            return err(
                f'That `brief_coverage.clauses` is '
                f'{type(_cov.get("clauses")).__name__}, not a list of '
                f'clauses.\n' + _regrade)
        if _cov is None:
            return err(
                f'That intent result carries no `brief_coverage`, so nothing '
                f'here can say whether the design brief\'s clauses were GRADED '
                f'or merely present.\n\n'
                f'`rules_run` counts RULES. One run graded six of them, passed, '
                f'and measured not one clause its brief declared -- the count '
                f'was satisfied by rules nobody had declared anything for. '
                f'Re-grade with a build that writes the block:\n' + _regrade
                + f'\n\nA board with no design brief at all still closes -- the '
                  f'block is written EMPTY for it, and this stage prints the '
                  f'absence. What it cannot do is read a document that '
                  f'predates the block. If there is no spec to declare at all, '
                  f'the honest route is the intent waiver: drop --intent-json '
                  f'and pass --waive intent:<why there is no spec>.')
        # ARM 3 -- CLAUSE COVERAGE. `not_claimed` and `carried` never reach
        # here: an author writing "unknown" is declaring honestly, and
        # punishing that is how a channel teaches people to stop declaring.
        #
        # An EMPTY clause list is a board that declares nothing, and it closes
        # -- #711 made a brief-less board cost nothing and this must not
        # reverse that. The absence is printed in the body instead, because a
        # gate that passes silently is a gate that vanished from the record.
        # That is why the block is written even when it is empty: absent and
        # empty would otherwise be the same document.
        _rows = _cov.get('clauses') or []
        _junk = [r for r in _rows if not isinstance(r, dict)]
        if _junk:
            return err(
                f'A `brief_coverage.clauses` entry is '
                f'{type(_junk[0]).__name__}, not a clause object.\n'
                + _regrade)
        _known = {str(r.get('id')) for r in _rows}
        _waived, _phantom, _noreason = _clause_waivers(a, _known)
        if _noreason:
            return err(
                f'--waive brief-clause:{_noreason[0]}: needs a REASON after '
                f'the colon -- why THIS board cannot answer that clause. The '
                f'reason IS the finding; without one the waiver records only '
                f'that somebody wanted the gate to stop.')
        if _phantom:
            return err(
                f'--waive brief-clause names {_phantom[0]!r}, which this '
                f'document does not carry. A waiver that matches nothing is a '
                f'gate that looks satisfied and is not. The clause ids here '
                f'are:\n' + '\n'.join(f'  {i}' for i in sorted(_known)))
        # A WHITELIST of the states that may pass, not a blacklist of the two
        # that may not. The producer's good set is closed and tiny, and a
        # blacklist fails OPEN on a sixth state: measured, a row spelled
        # `ungraded` or `UNCOVERED` or carrying no `state` at all sailed
        # through while the clause was genuinely unmeasured. Note the
        # asymmetry that made this easy to miss -- the producer raises on a
        # novel state (`counts[state] += 1`) while the consumer stayed silent.
        _PASSES = ('graded', 'not_claimed', 'carried')
        _open = [r for r in _rows
                 if str(r.get('id')) not in _waived
                 and (r.get('state') not in _PASSES or r.get('drifted'))]
        if _open:
            def _bucket(name, pred):
                got = [r for r in _open if pred(r)]
                if not got:
                    return ''
                return f'\n  {name}\n' + '\n'.join(
                    f"    - {r.get('id')}: {r.get('why') or 'no reason given'}"
                    for r in got)
            return err(
                f'That intent graded {_n} rule(s), but {len(_open)} of the '
                f'design brief\'s {len(_rows)} declared clause(s) reached no '
                f'verdict:\n'
                + _bucket('UNCOVERED -- no rule looked at these',
                          lambda r: r.get('state') == 'uncovered')
                + _bucket('ABSTAINED -- a rule ran and declined on this clause',
                          lambda r: r.get('state') == 'abstained')
                + _bucket('DRIFTED -- graded, but not against what the brief '
                          'declares',
                          lambda r: (r.get('drifted')
                                     and r.get('state') == 'graded'))
                # The catch-all, so a row can never land in `_open` and in no
                # bucket: that produced a refusal with a BLANK list, which is
                # unactionable and reads like a bug in the gate rather than a
                # finding about the board.
                + _bucket('UNRECOGNISED -- this gate does not know this state',
                          lambda r: (r.get('state') not in
                                     ('uncovered', 'abstained')
                                     and not (r.get('drifted')
                                              and r.get('state') == 'graded')))
                + f'\n\n`rules_run` counts RULES, not CLAUSES, and a count of '
                  f'six is what let one run close clean with a whole list like '
                  f'this ungraded. Fold the brief into the document that is '
                  f'actually graded, then re-grade:\n'
                  f'  python3 -X utf8 py_tools/check_floorplan.py {a.board} '
                  f'--emit-intent wk/intent.json\n'
                  f'  # edit it down: keep what the SPEC requires, delete what '
                  f'is merely observed\n' + _regrade
                + f'\n\nA clause this board genuinely cannot answer is waived '
                  f'BY NAME, with a reason:\n'
                  f'  --waive brief-clause:<id>:<why this board cannot answer '
                  f'it>')
        _cov_read = (
            f"{_cov.get('graded', 0)} of {len(_rows)} declared brief "
            f"clause(s) graded"
            + (f", {len(_waived)} waived by name" if _waived else '')
            + '.') if _rows else (
            # Read off `brief`, which `clause_coverage` fills with the brief's
            # own basename precisely so a consumer can tell these apart.
            # Inferring absence from an EMPTY CLAUSE LIST made this stage
            # report "no design brief beside this board" for a board whose
            # brief was named in the very same block -- it declared only
            # free-text unknowns, so it compiled to no clause.
            f"the design brief {_cov.get('brief')} declares no gradable "
            f"clause -- everything in it is carried, declared unknown, or "
            f"free text, so there was nothing for a rule to reach."
            if _cov.get('brief') else
            'no design brief beside this board, so 0 clauses were declared '
            'and none could be graded. Every `edge` in that intent is an '
            'INFERENCE from a part pose, not a declaration.')
    # The routability read. It REFUSES only when the evidence is missing, never
    # on the numbers themselves -- see _guard_congestion and
    # docs/placement-calibration.md for why the threshold that used to live
    # here was withdrawn.
    _cok, _cwhy = _guard_congestion(a)
    if not _cok:
        return err(_cwhy)
    # The two off-board gate waivers this stage honours, ECHOED and PERSISTED
    # (#1031 review) the way P3 records its lock waivers: a waiver that
    # clears a gate and leaves no trace is a flag that made a finding vanish.
    # Written into the waivers.json beside the render, merged under
    # `closeout` so P3's own keys survive.
    _gw = {n: _waiver_for(a, n) for n in ('keepout-band', 'off-outline')}
    _gw = {n: r for n, r in _gw.items() if r}
    _gw_file = None
    if _gw:
        _gw_file = os.path.join(
            os.path.dirname(os.path.abspath(a.render_json)), 'waivers.json')
        try:
            _prev = {}
            if os.path.isfile(_gw_file):
                with open(_gw_file, encoding='utf-8') as _gf:
                    _prev = json.load(_gf)
            if not isinstance(_prev, dict):
                _prev = {}
            _prev['closeout'] = {'board': os.path.abspath(a.board),
                                 'waivers': _gw}
            with open(_gw_file, 'w', encoding='utf-8') as _gf:
                json.dump(_prev, _gf, indent=1, sort_keys=True)
        except (OSError, ValueError):
            _gw_file = None
    _gw_read = ('; '.join(f'--waive {n}: {r}' for n, r in sorted(_gw.items()))
                + (f'  (recorded in {_gw_file})' if _gw_file
                   else '  (NOT recorded: waivers.json could not be written)')
                ) if _gw else 'none'
    return f'''<stage_instructions stage="P-close" name="close out" of="{len(STAGES)}">
Prove the placement, then hand it on.

  DECLARED SPEC: {_cov_read}
  GATE WAIVERS: {_gw_read}

{_cwhy}

1. RE-MEASURE, exactly as P4 does, and put the numbers in the report. A verdict
   with no numbers beside it is an opinion.
2. Dispatch an INDEPENDENT verifier. Its prompt is a reference file, quoted
   whole -- do not paraphrase it, and do not read it as your own instructions:

<subagent_prompt agent="verifier" description="verify the placement">
Read {os.path.join(REFS, 'verifier-prompts.md')} and apply its placement
lenses to this board. Your inputs are:
    result   {a.board}
    before   {a.before}
    ledger   wk/ledger.jsonl
    intent   {a.intent_json or '(none -- waived on the record; see --waive)'}
Lens 1 (`intent`) needs that last file and was dispatched without it, so it had
no inputs and could not fail. Its test is CLAUSE COVERAGE, not the rule count:
read `brief_coverage` in that document and FAIL if any declared clause is
`uncovered` or `abstained`, or if one is `drifted`. `rules_run: 6` with every
brief clause ungraded is a vacuous pass, and it is the shape that passed here.
`rules_run == 0` still fails, as the floor beneath that.
Re-derive every number yourself from the boards; do not trust the report.
Answer with a line beginning VERDICT= and nothing above it.
</subagent_prompt>

3. The report states, for each item: what was damaged, what changed, the
   measurement that says so, and what remains with WHY it is unfixable here.
4. Placement invalidates every downstream routed board. Say so, and hand the
   board to /plan-pcb-routing -- or to /plan-pcb-placement-and-routing if this
   run also routes.
5. Render the run. Every other artifact is a snapshot; this is the only one
   that shows WHICH lap moved what, and it comes straight off the ledger:

       python3 -X utf8 py_tools/make_film.py --from-ledger wk/ledger.jsonl -o wk/place.mp4

   Rejected laps are in it too, badged -- seeing where the search went is the
   point. If it comes out one or two beats long, the laps were not recorded and
   the ledger is the thing to fix, not the film.

STOP conditions, name the one that applies:
  - every gate clean;
  - the residue named, measured, and shown unfixable at this stage;
  - two consecutive laps with no change (floorplan-shaped: needs a different
    arrangement, not another repair);
  - the 5-lap cap.
</stage_instructions>'''


# --------------------------------------------------------------------------
# guards
# --------------------------------------------------------------------------

def _dig(doc, key):
    """First occurrence of `key` anywhere in a nested JSON document."""
    if isinstance(doc, dict):
        if key in doc:
            return doc[key]
        for v in doc.values():
            got = _dig(v, key)
            if got is not None:
                return got
    elif isinstance(doc, list):
        for v in doc:
            got = _dig(v, key)
            if got is not None:
                return got
    return None


def _engine():
    """py_placer / py_router on sys.path, once, for the one guard that reads
    the board and the intent with the SAME code the seeder runs -- never a
    re-typed fnmatch -- so a plan this guard passes is the plan the seeder
    reads, block for block."""
    root = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..'))
    for sub in ('py_placer', 'py_router'):
        p = os.path.join(root, sub)
        if p not in sys.path:
            sys.path.insert(0, p)


def _guard_zone_plan(a):
    """P1 seeds FROM A PLAN. Refuse to seed from nothing (run 26).

    Run 26's placement half seeded from an intent carrying ONE zone, and
    then hand-placed most of its parts one pose at a time, because nothing
    had decided where anything went before the first seed. The arrangement
    is a decision, and this guard makes it a document: every movable part
    in a block with a `zone` rectangle and a `note`, or the stage does not
    print. MOVABLE is what the seeder moves -- any part with a pad
    (place_seed sends a pad-bearing pile part to the board centre whether
    or not a pad is connected, so a fiducial or a mounting hole must be
    zoned or must_lock too) minus `must_lock` patterns (fnmatch, as the
    seeder resolves them) minus the intent's `edge_claims()` (exact refs,
    as the grader looks them up; a `connector_affinity` entry claims no
    edge and IS seeded at its centroid, so it needs a zone). The COVERAGE
    DENOMINATOR is every footprint block (#959): a PAD-LESS block is not
    the seeder's to move, so it is answered separately -- placed and locked
    by hand, or dispositioned in `dispositions.refs` -- rather than left
    out of the count. Returns (True, {counts}) or (False, why)."""
    import fnmatch
    plan, perr = _load(a.zone_plan, 'The zone plan (--zone-plan)')
    if perr:
        return False, (
            perr + '\n\nA zone plan is a floorplan intent whose `blocks[].zone` '
            'rectangles cover every movable part -- the arrangement, decided '
            'BEFORE the first seed rather than one hand-placed pose at a '
            'time after it. Start from what the board and the brief already '
            'say, then author the zones by hand (a rectangle in board mm '
            'and a `note` saying why, per block):\n'
            f'  python3 -X utf8 py_tools/check_floorplan.py {a.board} '
            '--emit-intent wk/zone_plan.json')
    _engine()
    from placement import floorplan as _fp
    try:
        intent = _fp.intent_from_dict(plan, a.zone_plan)
    except Exception as exc:                                # noqa: BLE001
        return False, (
            f'The zone plan does not read as a floorplan intent '
            f'({type(exc).__name__}: {exc}). The seeder would refuse it the '
            'same way; fix the file, not the flag.')
    zoned = [z for z in intent.blocks if z.rect is not None]
    if not zoned:
        return False, (
            'The zone plan declares no `blocks[].zone` at all, so it decides '
            'nothing about where anything goes: the seeder would seat every '
            'part at its connectivity centroid, which is how run 26 ended '
            'up hand-placing most of its parts after the seed. Give every '
            'movable part a block with a zone rectangle and a note.')
    noteless = sorted(z.name for z in zoned if not (z.note or '').strip())
    if noteless:
        return False, (
            f'{len(noteless)} zoned block(s) carry no `note`: '
            f'{", ".join(noteless)}. A zone is a decision, and a rectangle '
            'with no reason beside it is one nobody can review or revisit; '
            'say in the note why THESE parts go THERE.')
    from kicad_parser import parse_kicad_pcb
    try:
        pcb = parse_kicad_pcb(a.board)
    except Exception as exc:                                # noqa: BLE001
        return False, (
            f'The board cannot be read ({type(exc).__name__}: {exc}), so '
            'nothing can say which parts the plan covers.')
    if not pcb.footprints:
        return False, (
            f'{a.board} parses but carries no footprint, so there is nothing '
            'for a zone plan to cover. This is not the unplaced board; point '
            '--board at it.')
    members, _problems = _fp.resolve_blocks(intent, pcb, ('kicad', 'sheet'))
    covered = set()
    for z in zoned:
        covered.update(members.get(z.name, ()))
    # #959 (#999): the denominator is every footprint BLOCK (#726 keys:
    # `#<uuid>` for a reference-less block, `Ref*~2` for a second block
    # sharing a reference). It was `if fp_.pads`, which is what the seeder
    # moves -- and so exactly what hid run 29's three pad-less logos: "6 zoned
    # blocks cover all 13 movable" on a 21-block board, while one logo sat at
    # the pile origin printing silk across CON2's apertures for 12 laps.
    # A pad-less block is not the seeder's (it is not in its state at all),
    # so it is answered separately below: placed and locked by hand, or
    # dispositioned in writing. Never excluded.
    padless = {ref for ref, fp_ in pcb.footprints.items() if not fp_.pads}
    movable = {ref for ref, fp_ in pcb.footprints.items() if fp_.pads}
    # TWO DIFFERENT LOCKS, and they answer two different questions.
    #
    # `file_locked` is `(locked yes)` in the board, which is what actually
    # pins a pose: the seeder puts such a ref in `placed` before stage 1 runs
    # and every stage skips it. `must_lock` is a claim about the FILE -- "this
    # ought to be locked" -- that the seeder honours by STAMPING the lock into
    # its OUTPUT, after it has seated the part. So must_lock answers "is this
    # the seed's to arrange" (stage 1.5 seats it at its current pose where it
    # can) and does NOT answer "is its pose already decided".
    #
    # Measured, and this is the review finding that corrected this guard: a
    # declared edge connector with `must_lock` and no file lock is seated by
    # stage 1 at the band midpoint at its incoming angle, byte for byte as if
    # nothing had been declared at all. Accepting must_lock here would have
    # let the plan through on the very route SKILL.md recommends.
    file_locked = {ref for ref in movable
                   if getattr(pcb.footprints[ref], 'locked', False)}
    locked = file_locked | {ref for ref in movable
                            if any(fnmatch.fnmatch(ref, pat)
                                   for pat in intent.must_lock)}
    claimed = {str(c.get('ref')) for c in intent.edge_claims()}
    edge = {ref for ref in movable if ref in claimed}
    left = sorted(movable - locked - edge - covered)
    if left:
        return False, (
            f'{len(left)} movable footprint(s) sit in no zoned block: '
            f'{", ".join(left)}. A part the plan does not place is a part '
            'the seeder puts at its connectivity centroid, at the first '
            'rotation that fits -- the pose nobody decided. Add each to a '
            'block with a zone, or declare it must_lock / an edge connector '
            'if that is what it is.')
    # THE SEEDER PLACES THE RESIDUE, NOT THE DECISIONS (run 27).
    #
    # A declared edge connector states its EDGE and nothing else that matters:
    # where along the edge it sits and which way its mating face points are
    # both decisions, and the seeder has neither. It takes the declared band's
    # midpoint (or an even distribution when no band is declared) and the
    # part's incoming angle, which on a pile is a generator default. Measured
    # on a 21-part 2-layer board: both free connectors came out at rotation 0
    # with their mating faces pointing nowhere in particular, and one of them
    # took the band midpoint straight through a fixed socket's ground tab.
    #
    # So a declared edge connector is placed BY THE AUTHOR and locked, and the
    # seed fills in around it. `--waive seed-connectors:<reason>` is the way
    # to hand one to the seeder deliberately -- the escape exists because a
    # board whose connectors genuinely have a free run along their edge is a
    # real case, and it should be a decision on the record rather than a
    # default nobody chose.
    # `file_locked`, NOT `locked`: only the file lock keeps stage 1 off it.
    free_edge = sorted(edge - file_locked)
    _sw = _waiver_for(a, 'seed-connectors')
    if free_edge and _sw == '':
        return False, (
            '--waive seed-connectors: needs a REASON after the colon. Handing '
            f'{", ".join(free_edge)} to the seeder is a decision about who '
            'chooses those poses, and a waiver with no reason is a flag that '
            'makes the refusal go away rather than an answer to it.')
    if free_edge and _sw is None:
        return False, (
            f'{len(free_edge)} declared edge connector(s) carry no '
            f'`(locked yes)` in the board: {", ".join(free_edge)}. An edge is '
            'the only thing their declaration states; where along it they sit '
            'and which way the mating face points are decisions the seeder '
            'does not have, so it takes the band midpoint at the incoming '
            'angle. Place each one yourself and lock it, then seed the rest:\n'
            f'  python3 -X utf8 py_placer/place_pose.py {a.board} {a.board} '
            'set <REF> <X> <Y> --rot <DEG>\n'
            f'  python3 -X utf8 py_placer/place_pose.py {a.board} {a.board} '
            'lock <REF>\n'
            'An intent `must_lock` does NOT do this: it is a claim about the '
            'file that the seeder stamps into its OUTPUT after seating the '
            'part, so a must_lock connector is seated at the band midpoint '
            'exactly as an undeclared one is (measured). The FILE lock is '
            'what stage 1 skips.\n'
            '(P2 is the stage for deciding them; `check_floorplan --intent '
            '<plan> --json` grades the result against the declared edge and '
            'band.)\n\n'
            'Or hand them to the seeder on the record: '
            '--waive seed-connectors:<why the seeder may choose these poses>')
    # #959 (#999): pad-less blocks, AFTER every pre-existing refusal so each
    # keeps its precedence (a plan with a free connector is told that first).
    ok_, why_ = _padless_owed(a, intent, pcb, padless, covered)
    if not ok_:
        return False, why_
    # #959 (#1001): the declared channels, reconciled -- contradictions,
    # unlocked mechanical refs, and drift from the brief.
    _bf, _bp, _brep, _berr = _p1_brief(a, pcb)
    ok_, why_ = _mechanical_owed(a, intent, plan, pcb, _bf, _bp, _brep,
                                 _berr)
    if not ok_:
        return False, why_
    # #959 (#998): the plan checked against itself and the board, before
    # the first pose write.
    ok_, why_ = _plan_owed(a, intent, pcb, _fp)
    if not ok_:
        return False, why_
    # LAST, so every refusal above keeps its wording and its precedence: a plan
    # that leaves parts unzoned is told that first, not that a rule is dark.
    ok_, why_ = _roster_owed(a, intent, pcb, _fp)
    if not ok_:
        return False, why_
    padless_locked = {r for r in padless
                      if getattr(pcb.footprints[r], 'locked', False)}
    return True, {'blocks': len(pcb.footprints),
                  'zoned': len(zoned), 'movable': len(movable - locked - edge),
                  'locked': len(locked), 'pinned': len(file_locked),
                  'edge': len(edge), 'seeded_edge': len(free_edge),
                  'padless': len(padless),
                  'padless_locked': len(padless_locked),
                  'padless_disposed': len(padless - padless_locked)}


def _padless_owed(a, intent, pcb, padless, covered):
    """#959 (#999): every PAD-LESS block answered for. `(True, None)` or
    `(False, why)` -- the guard shape, so `--dump-refusals` gates the texts.

    The seeder never places a block with no pads -- it is not in the
    placement state at all -- so on a pile it stays where the input left it.
    The answers that work today are the author's: place it (`place_pose set`
    has no pad requirement) and lock it, or write why it may stay where it
    is. `must_lock` is NOT an answer: it stamps a lock the seeder writes after
    seating, and the seeder never seats this block.

    A ZONE is an answer for none of them, and a lock does not make it one
    unless the block draws a courtyard: `rule_zone_containment` grades only
    parts the placement state carries, and a block with no pads and no
    courtyard is not one -- measured, a locked courtyard-less logo 13 mm
    outside its zone graded PASS. 0 of the 30 pad-less blocks on the 22
    corpus boards draw a courtyard.

    `dispositions.refs` keys are EXACT block keys, never globs: `Ref*` as a
    pattern also matches `Ref*~2`, and a disposition that silently covered a
    second block would excuse a part nobody looked at.
    """
    board = a.board
    ref_disp = (intent.dispositions or {}).get('refs', {})
    unknown = sorted(k for k in ref_disp if k not in pcb.footprints)
    if unknown:
        return False, (
            f'dispositions.refs names {len(unknown)} block(s) this board does '
            f'not have: {", ".join(unknown)}. Keys are EXACT block keys as the '
            'board parses them -- `#<uuid>` for a reference-less block, '
            '`Ref*~2` for a second block sharing a reference -- never globs.')
    padded = sorted(k for k in ref_disp if k not in padless)
    if padded:
        return False, (
            f'dispositions.refs answers PAD-LESS blocks only, and '
            f'{", ".join(padded)} carr{"ies" if len(padded) == 1 else "y"} '
            'pads. A block with pads is the seeder\'s to place: zone it, '
            'must_lock it, or declare it an edge connector.')
    file_locked = {r for r in padless
                   if getattr(pcb.footprints[r], 'locked', False)}
    stale = sorted(k for k in ref_disp if k in file_locked)
    if stale:
        return False, (
            f'dispositions.refs answers {", ".join(stale)}, which '
            f'{"is" if len(stale) == 1 else "are"} already locked in the '
            'board: the lock is the answer, and a second one reads as though '
            'the block were still undecided. Remove the disposition.')
    # Which zoned block, by which pattern, reaches each pad-less block --
    # matched as `resolve_blocks` matches (`fnmatch`), so this reads the
    # members the grader will. Two questions, two populations:
    #   * a block that draws a courtyard IS graded wherever a zoned block's
    #     pattern reaches it, glob or not, so any match makes it zoned;
    #   * a block that draws none is graded by nothing, so the zone is only a
    #     claim when the author NAMED it (the key, its escaped form, or a
    #     pattern with no wildcard). A class glob sweeping one in -- `R*`
    #     catching a `REF**` logo, `D*` catching a `D&M` mark -- asks
    #     nothing of it (round-2 verification: a normal glob plan on a
    #     shipping board was refused with advice place_pose then refused).
    import fnmatch as _fnm
    import glob as _glob
    _nc = os.path.normcase

    def _explicit(pat, key):
        # The key itself (glasgow's `REF**` as the board spells it), its
        # escaped form, or a wildcard-free pattern -- compared the way
        # `fnmatch` compares, so on Windows `ref[*][*]` names `REF**` for
        # this check exactly as it does for `resolve_blocks`.
        return (_nc(pat) == _nc(key) or _nc(pat) == _nc(_glob.escape(key))
                or not any(ch in pat for ch in '*?['))
    cover = {}
    for _z in intent.blocks:
        if _z.rect is None:
            continue
        for _pat in _z.refs:
            for _r in padless:
                if _fnm.fnmatch(_r, _pat):
                    cover.setdefault(_r, []).append(
                        (_z.name, _pat, _explicit(_pat, _r)))
    answered = file_locked | set(ref_disp)
    zoned = padless & covered
    named = {r for r, hits in cover.items() if any(e for _b, _p, e in hits)}

    def _where(r, explicit=False):
        hits = [(b, p) for b, p, e in cover.get(r, ()) if e or not explicit]
        return (', '.join(f"{r} (block {b!r}, refs {p!r})" for b, p in hits)
                or r)
    courted = set()
    if zoned:
        try:
            from placement.body import board_bodies, SOURCE_COURTYARD
            _b = board_bodies(pcb, board)
            courted = {r for r in zoned
                       if getattr(_b.get(r), 'source', '') == SOURCE_COURTYARD}
        except Exception:                                   # noqa: BLE001
            courted = set()
    ungradable = sorted(named - courted)
    if ungradable:
        done = [r for r in ungradable if r in answered]
        return False, (
            f'{len(ungradable)} pad-less block(s) are named in a zoned '
            f'block and draw no courtyard: '
            f'{"; ".join(_where(r, True) for r in ungradable)}. No rule can grade '
            'where such a block is -- `zone_containment` grades only parts '
            'the placement state carries, and a block with no pads and no '
            'courtyard is not one -- so the zone is a claim nothing checks. '
            'Remove it from the block\'s `refs`'
            + (f' ({", ".join(done)} '
               f'{"is" if len(done) == 1 else "are"} already answered by a '
               'lock or a disposition, so that is all that is left to do)'
               if done else '')
            + '. An unanswered one is then answered like any pad-less '
              'block: place it yourself and lock it, or write why it may '
              'stay where it is (`dispositions.refs`).')
    unlocked_zoned = sorted(courted - file_locked)
    if unlocked_zoned:
        return False, (
            f'{len(unlocked_zoned)} pad-less block(s) are named in a zoned '
            f'block: {"; ".join(_where(r) for r in unlocked_zoned)}. The '
            'seeder never places a block with no pads, so the zone does not '
            'move it -- but it draws a courtyard, so the zone DOES grade it '
            'once it is placed (a disposition does not place it). Place it '
            'yourself and lock it:\n'
            f"  python3 -X utf8 py_placer/place_pose.py {board} {board} "
            "set '<KEY>' <X> <Y> --rot <DEG> lock '<KEY>'")
    open_ = sorted(padless - file_locked - set(ref_disp))
    if open_:
        return False, (
            f'{len(open_)} pad-less block(s) are answered for by nothing: '
            f'{", ".join(open_)}. A logo or a graphic has no pads, so the '
            'seeder never moves it: on a pile it stays at the pile origin, '
            'which is where run 29 left one printing silk across CON2\'s '
            'apertures for 12 laps (10 pairs, max 0.986 mm). Place each one '
            'yourself and lock it:\n'
            f"  python3 -X utf8 py_placer/place_pose.py {board} {board} "
            "set '<KEY>' <X> <Y> --rot <DEG> lock '<KEY>'\n"
            'or write why it may stay where it is, in the zone plan: '
            '"dispositions": {"refs": {"<KEY>": "<why>"}}. `must_lock` does '
            'not place a block the seeder never seats.')
    return True, None


def _clause_waivers(a, known):
    """`(waived, phantom, noreason)` for every `--waive brief-clause:<id>:<why>`.

    Shared by P-close and P1 (#959) so a clause is waived the same way at
    both gates.

    The id is resolved AGAINST THE DOCUMENT rather than by splitting on
    colons: a clause id contains them (`proximity[0:Y1~U1].max_mm`) and so
    may a reason, so any positional split cuts one of the two in half. The
    document is the authority on what its own ids are.
    """
    waived, phantom, noreason = set(), [], []
    for w in (a.waive or []):
        if not w.startswith('brief-clause:'):
            continue
        rest = w[len('brief-clause:'):]
        # LONGEST first, and the `+ ':'` matters: without it
        # `keepouts[batt]xyz:reason` would silently waive `keepouts[batt]`,
        # and without longest-first a waiver for `keepouts[usb-shell]` could
        # resolve to `keepouts[usb]` and leave the real clause open while
        # reporting it waived. That pair is REAL -- keep-out names are the
        # author's, so one being a prefix of another is ordinary.
        # (`interfaces[J1].edge_band` stood here and is not a clause id any
        # producer emits: the interface ids are `.edge`, `.along_edge`,
        # `.user_facing`, `.overhang_mm`.)
        hit = next((i for i in sorted(known, key=len, reverse=True)
                    if rest == i or rest.startswith(i + ':')), None)
        if hit is None:
            # The WHOLE residue, not `split(':', 1)[0]` -- that is the very
            # bug this resolution fixed, surviving one line over in the
            # message: a waiver for `proximity[0:Y1~U2].max_mm` was reported
            # as naming `proximity[0`.
            phantom.append(rest)
        elif not rest[len(hit) + 1:].strip():
            noreason.append(hit)
        else:
            waived.add(hit)
    return waived, phantom, noreason


def _p1_brief(a, pcb):
    """`(fragment, path, report, error)` for the design brief beside the
    board, compiled the way check_floorplan compiles it; `(None, '', None,
    None)` when there is none. An unreadable brief is returned as `error`,
    never swallowed: P-brief is an instruction stage with no guard, and a plan
    checked against no brief passes every declaration it drops (Phase-1
    verifier: `"bogus_key": 1` in fixture 711's brief let a plan with no
    keep-outs through P1 while check_floorplan exited 2 on the same file)."""
    from placement import design_brief as _db
    bp = _db.discover_brief(a.board)
    if not bp:
        return None, '', None, None
    try:
        frag, rep = _db.compile_with_consequences(
            _db.load_brief(bp), pcb, a.board)
    except Exception as exc:                                # noqa: BLE001
        return None, bp, None, f'{type(exc).__name__}: {exc}'
    return frag, bp, rep, None


def _p1_mechanical(a):
    """`(mechanical, path, error_or_None)` -- the same discovery and refusal
    `check_floorplan` uses (`cli_gates.load_mechanical_or_exit`)."""
    import io
    import contextlib
    from placement.cli_gates import load_mechanical_or_exit
    buf = io.StringIO()
    with contextlib.redirect_stderr(buf):
        mech, path, rc = load_mechanical_or_exit(a, a.board)
    return mech, path, (buf.getvalue().strip() or 'unreadable') if rc else None


def _mechanical_owed(a, intent, plan, pcb, brief_fragment, brief_path,
                     brief_report=None, brief_err=None):
    """#959 (#1001): P1's questions about the DECLARED channels.

    1. A disagreement between two declared / recorded values (the brief and
       `mechanical.json`, a mechanical pose and the outline) is a
       CONTRADICTION, and the plan must acknowledge it in writing. The
       acknowledgement ACCEPTS the winner the row names; it cannot flip it,
       because a plan is not a declaration -- to make the other value hold,
       correct the source that is wrong. Run 29's brief put USB1 east while
       its mechanical declaration put it west, and nothing compared them.
    2. Every mechanical ref the grade anchors must be FILE-locked, and every
       mechanical ref must sit AT its declared pose. The grade compiles the
       anchors from the file itself, whatever the plan says; this checks the
       board they will be graded on. Measured before this: run 29 moved and
       locked `Ref*` 25.9 mm off its declared pose and P1 passed, because it
       checked the lock and not where the lock was. `--waive
       seed-connectors` does not reach these: a mechanical fact is not the
       seeder's to choose.
    3. The plan must carry every clause the brief declares, as the brief
       declares it -- the clause coverage P-close grades, asked of the plan.
       Before this only P-close refused, so `place_seed` seeded whatever the
       zone plan said; and a plan that DROPPED a clause (no proximity row, no
       edge entry, no keep-out) passed P1 outright.
    Returns `(True, None)` or `(False, why)` -- the guard shape.
    """
    from placement import reconcile as _rc
    from placement import design_brief as _db
    from placement import floorplan as _fp
    if brief_err:
        return False, (
            f'The design brief {brief_path} cannot be read ({brief_err}). '
            'check_floorplan refuses it at exit 2, and a plan checked '
            'against no brief passes every declaration it drops. Fix the '
            'brief (P-brief shows its shape).')
    mech, mech_path, mech_err = _p1_mechanical(a)
    if mech_err:
        return False, (
            f'The mechanical declaration {mech_path} cannot be used '
            f'({mech_err}). A file by that name that reads as "no '
            'mechanical facts" would be the silent absence #959 is about. '
            'Fix or restore it. Outside an unaided regime, --no-mechanical '
            'says it is not an input; inside one it was recorded at staging '
            'and cannot be switched off.')
    try:
        from list_nets import board_floor_knobs
        _kn = board_floor_knobs(a.board)[2]
    except Exception:                                       # noqa: BLE001
        _kn = None
    rows = _rc.reconcile(pcb, a.board, brief_fragment=brief_fragment,
                         brief_source=brief_path or None, mechanical=mech,
                         intent_doc=plan, intent_source=a.zone_plan,
                         floors_used=_kn)
    answered = (intent.dispositions or {}).get('contradictions', {})
    contra = _rc.contradictions(rows)
    ids = {r['id'] for r in contra}
    open_ = [r for r in contra if r['id'] not in answered]
    stale = sorted(k for k in answered if k not in ids)
    if open_ or stale:
        return False, (
            (f'{len(open_)} contradiction(s) between DECLARED sources, and '
             'the plan does not acknowledge them' if open_ else
             'The zone plan answers contradictions this board does not have '
             '-- a stale answer reads as though something were decided when '
             'nothing is')
            + ':\n'
            + ''.join(
                f"  - {r['id']}: " + '; '.join(
                    f"{ch} {v['value']!r} [{v['authority']}, {v['source']}]"
                    for ch, v in r['values'].items()
                    if v['value'] is not None)
                + f" -> {r['winner']} wins -- {r['why']}\n" for r in open_)
            + ''.join(
                f"  - dispositions.contradictions.{k} answers no "
                'contradiction this plan has; remove it\n' for k in stale)
            + '\nEach names both values, where they came from and which one '
            'wins: the stronger source. Acknowledging a row ACCEPTS that '
            'winner -- a plan is not a declaration and cannot overrule one. '
            'If the winner is wrong, correct its SOURCE (the brief, '
            'mechanical.json or the board) instead. Acknowledge IN THE ZONE '
            'PLAN:\n'
            '  "dispositions": {"contradictions": {"<id>": "<why the '
            'winning value holds>"}}')
    # A HYPOTHESIS that loses to a RECORDED fact: under an unaided regime the
    # brief is the run's own reading, so run 29's USB1 -- brief east,
    # mechanical.json west -- is drift the mechanical value wins, not a
    # contradiction. Nothing refused it: P1 then demanded USB1 locked WEST
    # while the brief-clause check demanded the plan carry EAST, and the
    # grade failed on a locked part the seeder never moves (pre-push
    # review). The recorded fact cannot be overruled, so the answer is to
    # correct the losing source; there is no disposition for it.
    beaten = []
    for r in rows:
        if r.get('kind') != 'drift':
            continue
        wv = (r.get('values') or {}).get(r.get('winner')) or {}
        if wv.get('authority') != 'recorded_fact':
            continue
        # The brief and the plan only: a POSE the run moved is answered by
        # the lock-at-pose check below, with the place_pose commands.
        losers = [(ch, v) for ch, v in (r.get('values') or {}).items()
                  if ch in ('brief', 'intent') and ch != r.get('winner')
                  and v.get('value') is not None
                  and v.get('authority') == 'hypothesis'
                  and v.get('value') != wv.get('value')]
        if losers:
            beaten.append((r, wv, losers))
    if beaten:
        return False, (
            f'{len(beaten)} value(s) this run wrote disagree with a RECORDED '
            'fact that outranks them:\n'
            + ''.join(
                f"  - {r['id']}: {r['winner']} {wv['value']!r} "
                f"[recorded_fact, {wv.get('source')}] vs "
                + '; '.join(f"{ch} {v['value']!r} [hypothesis, "
                            f"{v.get('source')}]" for ch, v in losers)
                + '\n' for r, wv, losers in beaten)
            + '\nThe recorded value holds -- it existed before this run, '
            'and a value the run wrote (its own brief, the zone plan) '
            'cannot overrule it. Correct the losing source so the two '
            'agree: the brief this run wrote, and the plan\'s entry, then '
            're-check. There is no disposition for this -- a plan that '
            'carried the losing value would be graded against a fact '
            'nothing in this run may change.')
    lost = set(_rc.lost_mechanical_refs(rows))
    anchored = sorted(
        ref for ref, _p in (mech or {}).get('poses', {}).items()
        if ref in pcb.footprints and pcb.footprints[ref].pads
        and ref not in lost)
    drifted = {v.ref: v for v in (_fp.mechanical_drift(
        intent, pcb, mech, skip=sorted(lost)) if mech else ())}
    unlocked = [r for r in anchored
                if not getattr(pcb.footprints[r], 'locked', False)]
    owed_m = sorted(set(unlocked) | set(drifted))
    if owed_m:
        # A board carrying copper refuses every pose write without
        # `--allow-routed` (orangecrab: 742 segments), so the printed
        # remedy says so rather than failing at exit 3.
        routed = ' --allow-routed' if (pcb.segments or pcb.vias) else ''
        return False, (
            f'{len(owed_m)} mechanical ref(s) in {mech_path} are not held '
            'at their declared pose: '
            + '; '.join(
                (drifted[r].message.split(' -- ')[0]
                 if r in drifted else f"{r} is not locked")
                for r in owed_m)
            + '. A declared pose is a recorded fact: for a part with pads '
            'the grade compiles an anchor at exactly that pose from the file '
            'itself, whatever the plan says, and any part that drifted or '
            'turned is an ERROR. Only a FILE lock keeps the seeder off a '
            'part, and '
            'measured, it cannot seat one at an exact pose. Put each where '
            'the declaration says and lock it there:\n'
            # A part locked where it should not be takes TWO calls: one
            # call may not both unlock and lock a ref (place_pose refuses
            # that as ambiguous), and moving a locked part needs `unlock`
            # in the same call as the move.
            + ''.join(
                f"  python3 -X utf8 py_placer/place_pose.py {a.board} "
                f"{a.board}{routed} "
                + (f"unlock '{r}' " if getattr(pcb.footprints[r], 'locked',
                                               False) else '')
                + f"set '{r}' {mech['poses'][r]['x']} "
                f"{mech['poses'][r]['y']}"
                + (f" --rot {mech['poses'][r]['rot']}"
                   if mech['poses'][r]['rot'] is not None else '')
                + (f"\n  python3 -X utf8 py_placer/place_pose.py {a.board} "
                   f"{a.board}{routed} lock '{r}'\n"
                   if getattr(pcb.footprints[r], 'locked', False)
                   else f" lock '{r}'\n")
                for r in owed_m)
            + 'If the declared pose is the wrong one, correct '
            f'{os.path.basename(mech_path or "mechanical.json")} -- a plan '
            'cannot overrule it. (`--waive seed-connectors` does not reach '
            'these: a mechanical fact is not the seeder\'s to choose.)')
    if brief_fragment:
        lines_by_id = {}
        for cid, line in _db.drift_pairs(plan, brief_fragment):
            if cid:
                lines_by_id.setdefault(cid, line)
        cov = _db.clause_coverage(
            brief_report or {}, plan,
            rules_run=[n for n, _f in _fp.RULES if _fp._wants(intent, n)],
            drifted_ids=sorted(lines_by_id))
        why_by_id = {c['id']: (lines_by_id.get(c['id']) or c['why'])
                     for c in cov['clauses']
                     if c['state'] == 'uncovered' or c['drifted']}
        waived, phantom, noreason = _clause_waivers(
            a, {c['id'] for c in cov['clauses']} | set(lines_by_id))
        open_d = sorted(i for i in why_by_id if i not in waived)
        if noreason:
            return False, (f'--waive brief-clause:{noreason[0]}: needs a REASON '
                    'after the colon -- why this plan may drift from what '
                    'the brief declares.')
        if open_d:
            return False, (
                f'The zone plan drops or contradicts {len(open_d)} clause(s) '
                'of the design brief:\n'
                + ''.join(f'  - {i}: {why_by_id[i]}\n' for i in open_d)
                + '\nThe brief is the declaration; a plan that leaves a '
                'clause out, or says something else, is a guess, and the '
                'seeder would place against the guess. Fold the brief back in '
                '(re-emit with `check_floorplan --emit-intent`, then edit), '
                'or waive a clause BY NAME with the reason this board cannot '
                'hold it:\n'
                '  --waive brief-clause:<id>:<why>')
    return True, None


def _plan_owed(a, intent, pcb, _fp):
    """#959 (#998): the zone plan checked against itself and the board BEFORE
    the first pose write. `(True, None)` or `(False, why)`.

    Run 29 found its zone plan's ERRORs at lap 5: the only caller of the
    self-consistency check was the grade, which runs after the seed is
    written. `floorplan.plan_check` refuses only what no arrangement can
    satisfy, so every finding here is a plan to fix, not a placement to
    try. An outline that cannot be graded is left to the roster check,
    which says so in its own words.
    """
    try:
        from list_nets import board_floor_knobs
        clr, edge_clr, _k = board_floor_knobs(a.board, None, None)
        # The SAME group sources the guard resolved the plan's blocks with
        # above: without them a sheet-group block resolves to nothing and
        # reads as `block_unresolved` -- a false refusal of every emitted
        # plan on a hierarchical board (caught by the corpus control).
        found, _meas = _fp.plan_check(intent, pcb, a.board, clearance=clr,
                                      board_edge_clearance=edge_clr,
                                      group_sources=('kicad', 'sheet'))
    except _fp.UntrustworthyOutline:
        return True, None
    errs = [v for v in found if v.severity == _fp.ERROR]
    if not errs:
        return True, None
    return False, (
        f'{len(errs)} finding(s) in the zone plan that no arrangement can '
        'satisfy -- checked against the plan itself and the board, before '
        'any pose exists (run 29 found its plan errors at lap 5):\n'
        + ''.join(f'  - {v.rule}: {v.message}\n' for v in errs)
        + '\nFix the plan, then re-check it without seeding:\n'
        f'  python3 -X utf8 py_tools/check_floorplan.py {a.board} '
        f'--intent {a.zone_plan} --plan-only')


def _roster_owed(a, intent, pcb, _fp):
    """#959 (#997): the rules this plan leaves DARK, answered in writing.

    Run 29 graded 22 times with 6 of 14 rules never running, and every run
    printed each one's reason -- "the intent declares no ..." -- to a reader who
    had no reason to act on it. A plan is where that becomes a decision: a
    rule that applies to this board, fails the grade when it fires, and that
    the plan neither arms nor excuses is a question the plan never asked.

    Refused only when a board fact says the rule applies and it is gating
    (`floorplan._roster`): policy rules (`proximity`, `zone_exclusive`) and
    advisory ones are reported, never refused, because measured before this
    was built they were dark on 22 of 22 corpus boards and a refusal every
    plan answers the same way carries no signal. Returns `(True, None)` or
    `(False, why)` -- the guard shape, so `--dump-refusals` gates the text.
    """
    brief_fragment = _p1_brief(a, pcb)[0]
    try:
        from list_nets import board_floor_knobs
        clr, edge_clr, _k = board_floor_knobs(a.board, None, None)
        rows = _fp.rule_roster(intent, pcb, a.board, clearance=clr,
                               board_edge_clearance=edge_clr,
                               brief_fragment=brief_fragment)
    except _fp.UntrustworthyOutline as exc:
        return False, (f'The board outline cannot be graded ({exc}), so nothing can '
                'say which rules this plan leaves dark. Fix the outline '
                'before planning against it.')
    owed = _fp.roster_refusal_lines(rows)
    stale = [s_ for s_ in _fp.stale_dispositions(intent, rows)
             if s_.startswith('dispositions.withheld.')]
    if not owed and not stale:
        return True, None
    return False, (
        (f'{len(owed)} rule(s) this plan leaves dark apply to this board and '
         'would fail the grade when they fire, and nothing answers for them. '
         'A rule nobody armed and nobody excused is a question the plan '
         'never asked, and `check_floorplan` will print its skip reason on '
         'every lap without anyone acting on it (run 29: 6 of 14 rules, 22 '
         'invocations):\n' if owed else
         'This plan answers something that is not asked: a disposition for '
         'a key nothing withholds reads as though a budget were excused '
         'that the grade in fact enforces:\n')
        + ''.join(f'  - {line}\n' for line in owed)
        + ''.join(f'  - STALE {s_}; remove it\n' for s_ in stale)
        + '\nAnswer each IN THE ZONE PLAN -- arm the rule with its key, or '
        'write why it does not apply to this design:\n'
        '  "dispositions": {"rules": {"<rule>": "<why>"}, '
        '"withheld": {"<key>": "<why>"}}\n'
        'Arming a rule with an invented number is the wrong answer: a '
        'limit read off the board you are about to move grades clean by '
        'construction. Declare it from a requirement, or say there is '
        'none.')


def _guard_damage(a):
    """P2/P3 exist to repair damage; refuse to run them blind, or on a clean board."""
    drc, derr = _load(a.drc_json, 'The copper-free DRC result (--drc-json)')
    if derr:
        return False, (derr + '\n\nP0 produces it:\n  python3 -X utf8 '
                              f'py_router/check_drc.py {a.board} --clearance <floor> '
                              '--clearance-margin 0 --json wk/drc0.json')
    count = _dig(drc, 'violations')
    if count is None:
        count = _dig(drc, 'total_violations')
    if isinstance(count, list):
        count = len(count)
    if count is None:
        return False, (
            'That JSON carries no violation count, so it does not say whether '
            'this board has placement damage. A file that parses is not '
            'evidence -- an empty or unrelated JSON passes a file-exists check '
            'and answers nothing.\n\nProduce the real measurement:\n'
            f'  python3 -X utf8 py_router/check_drc.py {a.board} --clearance <floor> '
            '--clearance-margin 0 --json wk/drc0.json')
    if isinstance(count, int) and count == 0:
        asm, _ = _load(a.assembly_json, 'assembly')
        # THE VERDICT, not `blocking` (#937). `blocking` is ONE of
        # check_assembly's five not_buildable conjuncts since #918, so a board
        # unbuildable through a coincident-origin stack, a containment, copper
        # on a locked part or a moved-vs-baseline courtyard gate reads
        # `blocking` 0 -- and then this guard told the reader there was "no
        # damage for this stage to repair" about a board its own instrument
        # had just graded NOT BUILDABLE. One tracked board is exactly that
        # case. `buildable` is the field that answers the question this guard
        # is asking; `blocking` is the fallback for a document old enough not
        # to carry it.
        buildable = _dig(asm, 'buildable') if asm else None
        blocking = _dig(asm, 'blocking') if asm else None
        undamaged = (not blocking) if buildable is None else bool(buildable)
        if undamaged:
            return False, (
                'The copper-free board reports 0 violations and '
                + ('an assembly verdict of buildable'
                   if buildable is not None else 'no blocking assembly pair')
                + '. There is no damage for this stage to repair, '
                'and running a placement search on a legal board makes it '
                'worse (measured).\n\nIf you want a different ARRANGEMENT '
                'rather than a repair, that is --stage P5. If you were '
                'expecting damage, your clearance floor is probably wrong: '
                'read it off the board, not from a round number.')
    return True, ''


def _guard_render(a):
    """A stage that FOLLOWS a move refuses until the move was looked at.

    Run 9 produced ZERO render_placement reads across an entire placement
    campaign and nothing noticed -- not the driver, not the ledger, not the
    operator. The skill mandates a read in eight cases; this file enforced none
    of them, so the mandate was carried entirely by prose that an executor
    skims. That is the same failure mode this whole driver exists to fix: "a
    gate written in prose is a sentence someone skims; a gate that withholds
    the next instructions cannot be skimmed past."

    What it checks, and why each one:
      * the document loads -- a claim is not evidence;
      * `instrument.board` is THIS board -- render_placement records the board
        it rendered, and without this check an earlier lap's render satisfies a
        later stage (the same board-binding hole loop_driver.l2 has);
      * a `checklist` block exists -- a bare `--json` render answers none of
        mandate 8's four questions;
      * `checklist.d_moved.match` is not False -- if the caller declared
        --expect-moved, the render must agree with it.
    """
    doc, derr = _load(a.render_json, 'The placement render (--render-json)')
    if derr:
        return False, (
            derr + '\n\nEvery stage that FOLLOWS a move needs one. Produce it '
                   'against the board this one came from:\n'
                   f'  python3 -X utf8 py_tools/render_placement.py {a.board} \\\n'
                   f'      --before <the board it was derived from> \\\n'
                   '      --clearance <the board\'s own floor> '
                   '--ignore-nets <the poured nets> \\\n'
                   '      --expect-moved <how many the stage said it moved> \\\n'
                   '      --review-sheet wk/sheet.png \\\n'
                   '      --json-out wk/render.json -o wk/render.png '
                   '--quiet\n\n'
                   'Then READ it -- the JSON is the re-measurement channel, the '
                   'picture is what catches what no metric models.')
    inst = doc.get('instrument') or {}
    shown = inst.get('board')
    if not shown:
        return False, (
            'That render carries no `instrument.board`, so it cannot say WHICH '
            'board it looked at. A render that parses is not evidence that this '
            'board was looked at. Re-render with --json-out (not bare --json).')
    if os.path.normcase(os.path.abspath(shown)) != \
            os.path.normcase(os.path.abspath(a.board)):
        return False, (
            f'That render is of a DIFFERENT board:\n'
            f'      rendered: {shown}\n'
            f'      staging : {os.path.abspath(a.board)}\n\n'
            f'An earlier lap\'s render does not prove this one was looked at.')
    chk = doc.get('checklist')
    if not isinstance(chk, dict):
        return False, (
            'That render has no `checklist` block, so it answers none of the '
            'four questions the read exists to ask: is any part off the '
            'outline, is any part on any other part, is any part on a hole or a '
            'locked part, and did more parts move than the step claimed.\n\n'
            'Re-render with --json-out.')
    # PAD COPPER OFF THE OUTLINE -- the top-priority placement defect, and
    # until now checked at ONE of the three doors.
    #
    # `loop_driver.l2` refuses a board on it (measured, its own comment:
    # "`oob_pad_count` alone refuses 24, of which 12 are refusals `blocking`
    # misses"), and nothing in this file did -- so the same board closed out
    # clean through the placement door and was refused through the loop.
    # CLAUDE.md: "A part whose pad copper lies outside the outline is the
    # top-priority placement defect... Measured, run 10: 11 such parts produced
    # ALL 13 unrouted nets and most of the 37 broken ones."
    #
    # THE CHANNEL MATTERS, and it is not the one the loop uses. l2 reads
    # `oob_pad_count` out of check_assembly's report, which is a part-level pad
    # AABB inflated by the grading clearance -- its own `oob_pad_basis` string
    # says so and points here instead. Measured over every tracked board: that
    # count is non-zero on three, and on two of them -- both human-designed
    # reference boards carrying edge-mounted switches, at 0.03mm and 0.17mm --
    # this per-PAD measure reports an EMPTY list. The AABB fires on the
    # bounding box of an edge part; the pads are on the board. Gating on that
    # count would refuse two human boards on a measurement artifact, which is
    # why CLAUDE.md names this key and adds "a whole-board pass/fail verdict is
    # the wrong channel for it". (The boards are named in the PR that added
    # this, not here: a skill that names corpus boards is what
    # tests/test_run8_skills_generic.py refuses, because guidance written
    # around one board stops being guidance.)
    #
    # So this gate binds on the per-pad list and NAMES THE PARTS, because a
    # count is not something you can act on and the refusal exists to be acted
    # on.
    # THE BY-DESIGN ESCAPE (#937). This census is per-pad against the real
    # outline with NO exemption for castellations, card edges or a declared
    # `edge_connectors` band -- and this skill says elsewhere that a
    # castellated pad is centred ON the outline and such parts are MEANT to
    # cross it. Without an escape the refusal ordered a reader to drag a
    # mating connector inboard, which breaks the thing the board exists to
    # mate with, and no flag could clear it.
    #
    # A REASON IS REQUIRED, like every other waiver here: `--waive
    # off-outline:` with nothing after it is refused rather than honoured,
    # because a waiver with no reason is a flag that makes a gate disappear.
    _oobw = _waiver_for(a, 'off-outline')
    if _oobw == '':
        return False, ('--waive off-outline needs a REASON after the colon: '
                       'which refs are by design, and to what mating '
                       'standard. A waiver with no reason is a flag that '
                       'makes the gate disappear.')
    _oob = (chk.get('a_off_outline') or {}).get('pad_copper')
    if _oobw:
        _oob = None
    if isinstance(_oob, list) and _oob:
        _refs = []
        for _it in _oob:
            _r = _it.get('reference') if isinstance(_it, dict) else _it
            if _r and str(_r) not in _refs:
                _refs.append(str(_r))
        return False, (
            f'{len(_oob)} part(s) carry PAD COPPER outside the board outline: '
            f'{", ".join(_refs) or "see checklist.a_off_outline.pad_copper"}.'
            f'\n\nThose nets cannot be routed at all, so this converts '
            f'one-for-one into `unrouted` and `broken` -- measured, run 10: 11 '
            f'such parts produced ALL 13 unrouted nets and most of the 37 '
            f'broken ones. It is the top-priority placement defect, ahead of '
            f'every clearance graze.\n\nUNLESS THE CROSSING IS BY DESIGN. A '
            f'castellated row, a card edge and a declared `edge_connectors` '
            f'part are all MEANT to cross the boundary -- this census has no '
            f'exemption for them, so it names them too. If that is what these '
            f'are, declare them in the intent\'s `edge_connectors` and re-run '
            f'this stage with --waive off-outline:<the refs and the mating '
            f'standard>. Do not move a connector inboard to satisfy a gate; '
            f'that breaks the thing the board exists to mate with.\n\n'
            f'Otherwise RE-SEAT them -- by net centroid, not back to an old '
            f'pose (the objective is a board that routes, not a board '
            f'restored):\n  python3 -X utf8 py_placer/place_seed.py <board> '
            f'<out> --intent <intent> --reseat --clearance <the floor>\n'
            f'The outline itself is never yours to change.\n\n'
            f'This is the per-PAD measure, not check_assembly\'s '
            f'`oob_pad_count`, which is a part-level AABB inflated by the '
            f'clearance and reads non-zero on human boards whose pads are '
            f'fine.')
    # #1031: pads in a board RULE-AREA keep-out band -- the same defect by
    # another route. The router lands no track there, so the net fails
    # "boxed in by static obstacles" even on an empty board (run 32: 7 of
    # 18 open joins). render_placement emits the key on every render, so an
    # absent key is an older render, not a clean one, and is not refused
    # here (the pad_copper arm above has the same reading).
    _kow = _waiver_for(a, 'keepout-band')
    if _kow == '':
        return False, ('--waive keepout-band needs a REASON after the colon: '
                       'which pads sit in the band by design and how their '
                       'nets are reached. A waiver with no reason is a flag '
                       'that makes the gate disappear.')
    # A census that could not be BUILT leaves `keepout_copper` at [] and
    # names the failure in `keepout_copper_unmeasured` -- unmeasured is not
    # clean, so that error row refuses exactly like a finding.
    _kerr = [u for u in ((chk.get('a_off_outline') or {})
                         .get('keepout_copper_unmeasured') or ())
             if isinstance(u, (list, tuple)) and len(u) > 2
             and u[1] == 'error']
    if _kerr and not _kow:
        return False, (
            f'The render could not build its rule-area keep-out census: '
            f'{_kerr[0][2]}.\n\nIts keepout_copper list is therefore EMPTY '
            f'BY FAILURE, not by measurement, and an empty list here would '
            f'read as "no pad in any keep-out band". Re-render; if the '
            f'census still fails, check the board with\n'
            f'  python3 -X utf8 py_tools/check_assembly.py <board> '
            f'--clearance <the floor>\n'
            f'which prints the same channel from the file\'s own poses.')
    _ko = (chk.get('a_off_outline') or {}).get('keepout_copper')
    if _kow:
        _ko = None
    # NEW keep-out copper only, when the render graded the --before board
    # too (`keepout_copper_before`, the #962 --baseline idea): a part the
    # input already seated in the band -- a human reference board -- at no
    # greater depth is inherited, not this placement's doing. Without that
    # key the judgement is ABSOLUTE.
    _kob = (chk.get('a_off_outline') or {}).get('keepout_copper_before')
    _kbasis = 'absolute (the render carries no --before census)'
    if isinstance(_ko, list) and isinstance(_kob, list):
        _was = {}
        for _it in _kob:
            if isinstance(_it, (list, tuple)) and len(_it) > 1:
                _was[str(_it[0])] = float(_it[1])
        _ko = [_it for _it in _ko
               if not (isinstance(_it, (list, tuple)) and len(_it) > 1
                       and str(_it[0]) in _was
                       and float(_it[1]) <= _was[str(_it[0])] + 1e-6)]
        _kbasis = ('NEW against the --before board (parts it already seated '
                   'there, no deeper, are inherited)')
    if isinstance(_ko, list) and _ko:
        _krefs = []
        for _it in _ko:
            _r = (_it[0] if isinstance(_it, (list, tuple)) and _it
                  else _it.get('reference') if isinstance(_it, dict) else _it)
            if _r and str(_r) not in _krefs:
                _krefs.append(str(_r))
        return False, (
            f'{len(_ko)} part(s) seat pads inside a rule-area KEEP-OUT band, '
            f'where no track can land: '
            f'{", ".join(_krefs) or "see checklist.a_off_outline.keepout_copper"}.'
            f'\nJudged: {_kbasis}.'
            f'\n\nThe per-pad detail is in '
            f'checklist.a_off_outline.keepout_copper_pads. Those nets fail '
            f'"boxed in by static obstacles" even routed first on an empty '
            f'board, so this converts into `unrouted` exactly like pad copper '
            f'off the outline.\n\nMove each named part inward until its pads '
            f'clear the band -- place_pose grades the keep-out and refuses a '
            f'pose that deepens it:\n  python3 -X utf8 py_placer/place_pose.py '
            f'<board> <out> set <REF> <x> <y> --snap --clearance <the floor>\n'
            f'The rule area is the board\'s own declaration; it is never '
            f'yours to delete. If a pad sits there by design and its net is '
            f'served some other way, re-run this stage with --waive '
            f'keepout-band:<the pads and how they are reached>.')
    d = chk.get('d_moved') or {}
    if d.get('match') is False:
        return False, (
            f'The render disagrees with the move count you declared: it '
            f'measured {d.get("moved")} moved part(s), you passed '
            f'--expect-moved {d.get("expected")}.\n\nOne of the two is wrong, '
            f'and "more parts moved than the step claimed" is exactly what '
            f'mandate 8(d) exists to catch. Resolve it before continuing.')
    # #895's fifth check. `render_placement` records the sheet it composed in
    # `review_sheet`, so "was a review sheet built at all" becomes a gate
    # rather than a paragraph -- and the seven boundary criteria are answered
    # FROM that sheet. It still cannot check that anybody LOOKED; what it can
    # check is that the thing to look at exists, which is where the previous
    # mandate stopped. A `None` means the run asked for a sheet and the tool
    # could not write one; an absent key means it was never asked for.
    _sheet = doc.get('review_sheet') if 'review_sheet' in doc else ''
    if _sheet is None or (_sheet == '' and 'review_sheet' in doc):
        return False, (
            'That render was asked for a review sheet and none was written, '
            'so there is nothing to answer the seven boundary criteria from.'
            '\n\nRe-render with --review-sheet <PATH> --json-out <PATH>.json '
            '--quiet -- the last two flags are what keep the read BLIND, and '
            'the sheet is what the criteria are measured off.')
    if _sheet and not os.path.isfile(_sheet):
        return False, (
            f'That render names a review sheet that is not there:\n'
            f'      {_sheet}\n\n'
            f'A document that names a sheet nobody can open is the same '
            f'evidence as no sheet at all.')
    return True, ''


def _metrics_of(path, what):
    """(metrics dict, error) from a render_placement --json-out document."""
    doc, derr = _load(path, what)
    if derr:
        return None, derr
    m = doc.get('metrics')
    if not isinstance(m, dict):
        return None, (f'{what} carries no `metrics` block, so it cannot say '
                      f'what the arrangement costs. That is a bare --json '
                      f'render; re-render with --json-out.')
    return m, ''


def _waiver_for(a, name):
    """The reason given for `--waive <name>:<reason>`.

    None = not waived, '' = waived with no reason (which every caller refuses),
    otherwise the reason. Three hand-rolled copies of this split predate it;
    this one exists because the two routability waivers both need it and a
    fourth copy is where the strip() gets forgotten.
    """
    for w in (getattr(a, 'waive', None) or ()):
        if w.split(':', 1)[0].strip() == name:
            return w.split(':', 1)[1].strip() if ':' in w else ''
    return None


def _moved_count(path):
    """How many parts the close-out render says moved, or None.

    `moved_refs` is already in every `--json-out` render made with `--before`
    (render_placement.py serialises `moved_parts()` straight into it), so the
    disturbance read costs no new geometry and no second parse. Never refuses:
    a render made without `--before` legitimately has none.
    """
    doc, derr = _load(path, 'render')
    if derr or not isinstance(doc, dict):
        return None
    refs = doc.get('moved_refs')
    if isinstance(refs, list):
        return len(refs)
    n = (doc.get('checklist') or {}).get('d_moved', {}).get('moved')
    return n if isinstance(n, int) else None


def _implicated_refs(paths):
    """Refs any violation NAMED on the board the run started from.

    Reads check_assembly.py and check_drc.py JSON. This is NOT "the parts the
    damage displaced" and must never be used as one: a `swap` moves parts that
    land perfectly legally, and moving those back is the repair, not collateral.
    It is only "who was complained about", which is what makes a large gap
    against `moved` a question worth asking rather than an accusation.
    """
    refs = set()
    for p in paths or ():
        doc, derr = _load(p, 'damage report')
        if derr or not isinstance(doc, dict):
            continue
        for key in ('blocking_pairs', 'advisory_pairs', 'locked_contact_pairs'):
            for pair in doc.get(key) or ():
                if not isinstance(pair, dict):
                    continue
                for k in ('a', 'b', 'locked_ref'):
                    if pair.get(k):
                        refs.add(str(pair[k]))
        for item in doc.get('items') or ():
            if not isinstance(item, dict):
                continue
            for k in ('pad_ref', 'pad_ref2'):
                v = str(item.get(k) or '')
                # "C1.1" -> C1, "RM2.1.2" -> RM2. KiCad references do not
                # contain dots; pad numbers do.
                if v:
                    refs.add(v.split('.', 1)[0])
            # #962: check_drc names footprint GRAPHIC copper by its owner
            # (graphic-off-board / graphic-board-edge). Only from counted
            # rows: an ACCEPTED immutable-graphic row is inherited art (a
            # library antenna), and reading it would implicate that part on
            # every run.
            # Graphic rows only: a via-in-paste row also carries an owner_ref
            # (the opening's part), but it complains about a VIA, not a pose.
            if (item.get('owner_ref') and not item.get('accepted')
                    and item.get('type') in ('graphic-off-board', 'graphic-board-edge')):
                refs.add(str(item['owner_ref']))
        # check_assembly's graphic-copper channel ([[ref, mm], ...])
        for ref_mm in doc.get('oob_graphic_copper_refs') or ():
            if isinstance(ref_mm, (list, tuple)) and ref_mm:
                refs.add(str(ref_mm[0]))
    return refs


def _guard_congestion(a):
    """Put the routability numbers in front of the close-out. Do NOT refuse on them.

    Returns (ok, text). `ok` is False ONLY when the evidence is missing --
    the numbers themselves never refuse. The text is a REPORT that p_close
    prints inside its instructions.

    Why the report exists. Every other gate here is a LEGALITY gate --
    check_drc, check_assembly, check_channels, check_rigid_consistency -- so a
    run that cleared every blocking pad pair satisfied its close-out and
    stopped, whatever it had done to the arrangement. Measured on neo6502
    (run 15), against the undamaged control the run could not see: halo closed
    64.9% of the damage, crossings 8.0%. It fixed the symptoms and handed on a
    board harder to route than it found it; routing then failed on 29 nets, and
    the classifier said `parameter` -- correctly, by its own tests, which are
    all per-net and cannot see global capacity.

    WHY IT IS NOT A REFUSAL, and this is measured, not a preference
    (docs/placement-calibration.md, three boards, three populations each):

      * At the shipped ratio of 0.25 the gate refused neo6502's FULL repair --
        the one that took blocking 18 -> 0. It refused every outcome the corpus
        could produce, correct ones included.
      * The premise inverts. The gate assumed damage RAISES hpwl so a repair
        should lower it; on piantor the `swap` damage LOWERED hpwl (2263.6 ->
        1965.1), because swapping parts on a regular matrix shortens nets. There
        a PERFECT repair -- restoring the pristine board exactly -- scores
        hpwl_gain -0.152 and would have been REFUSED. A gate that refuses the
        correct answer must not refuse.
      * Only ONE of three boards yielded a calibration pair at all (`swap` gave
        18 blocking pairs on neo6502 and 1 on the other two), so any threshold
        would have been fitted to n=1 -- the same kind of number as the one it
        replaced.
      * Even the best-fitting ratio would not have done the job: neo6502's two
        populations separate at 0.093 vs 0.0095, and run 15's own arm scored
        0.064 -- above any splitting value, so the calibrated gate would have
        PASSED the board it was built to catch.

    What survives is the reading, which needs no threshold: if legality closed
    most of its gap while hpwl closed almost none, that is worth seeing at the
    moment a lever could still be pulled. The executor decides.

    hpwl and not crossings, still: non-negotiable 4, r(crossings) = +0.780
    against distance-to-truth -- not against routed blocking, which nothing has
    correlated it against (docs/placement-predictors.md). crossings is printed and never tested.
    """
    # THE WAIVER IS PARSED HERE AND SPENT AT THE BOTTOM. It used to return
    # (True, '') on the spot, ABOVE the evidence loads -- so a waived gate
    # printed nothing at all: no numbers, no reason, no trace it had fired.
    # Measured, run 16 on neo6502: the gate DID fire (halo +54.0%, hpwl +7.8%),
    # somebody dispositioned it, and the close-out body came out with an empty
    # routability block. Reading only the artifacts afterwards, the run looked
    # like a placement that had never been asked the question. A gate that
    # demands a disposition and then discards it is worse than no gate: it
    # converts an answer into silence and charges for the ceremony.
    #
    # So the waiver now suppresses the REFUSAL and nothing else. The numbers
    # still print, and the reason prints beside them -- the same shape
    # loop_driver's --accept-congestion has always had ("Reason on the record").
    _waiver = _waiver_for(a, 'congestion')
    if _waiver == '':
        return False, ('--waive congestion: needs a REASON after the '
                       'colon. An unexplained waiver is the gap, not the '
                       'fix.')
    _coll = _waiver_for(a, 'collateral')
    if _coll == '':
        return False, ('--waive collateral: needs a REASON after the colon. '
                       'An unexplained waiver is the gap, not the fix.')
    if not a.congestion_before:
        # Waived with no comparison rendered: there is nothing to measure, but
        # the decision still goes on the record rather than vanishing.
        if _waiver:
            return True, ('  ROUTABILITY: not measured -- no --congestion-before '
                          'render, waived.\n'
                          f'  Reason on the record: {_waiver}')
        return False, (
            'The close-out has no congestion comparison (--congestion-before).\n\n'
            'Every other gate here is a LEGALITY gate, so without this one a '
            'placement that cleared its blocking pairs and left the board just '
            'as tangled as it found it closes out clean -- which is exactly '
            'what happened on neo6502: 64.9% of the halo damage repaired, 8.0% '
            'of the crossings damage, and the routing half then failed on 29 '
            'nets that no router parameter could have saved.\n\n'
            'Render the board this run STARTED from, then pass it:\n'
            f'  python3 -X utf8 py_tools/render_placement.py {a.before} \\\n'
            '      --clearance <the board\'s own floor> '
            '--ignore-nets <the poured nets> \\\n'
            '      --json-out wk/congestion_before.json '
            '-o wk/congestion_before.png --quiet\n'
            f'  ... --stage P-close --congestion-before wk/congestion_before.json\n\n'
            'If this board genuinely has no congestion to compare (a handful of '
            'parts, no buses), put that on the record: '
            '--waive congestion:<why>.')
    before, berr = _metrics_of(a.congestion_before, 'The --congestion-before render')
    if berr:
        return False, berr
    after, aerr = _metrics_of(a.render_json, 'The close-out render (--render-json)')
    if aerr:
        return False, aerr

    def _gain(key):
        b, n = before.get(key), after.get(key)
        if not isinstance(b, (int, float)) or not isinstance(n, (int, float)):
            return None
        if b <= 0:
            return None
        return (b - n) / float(b)

    # GATE ON HPWL, NOT ON CROSSINGS. Non-negotiable 4 of this skill: "REPORT
    # crossings and aggregate courtyard overlap; never gate on them -- both
    # correlate POSITIVELY with distance-to-truth." Measured across 29
    # candidates on one board, r(crossings) = +0.780 against
    # distance-to-the-correct-placement -- NOT against routed blocking; see
    # docs/placement-predictors.md -- and one candidate reached 233 crossings
    # -- better than the human original's 276 -- while sitting 18.7 mm out of
    # position. A gate on crossings pressures the search toward LOWER crossings,
    # which by that correlation is pressure toward a WORSE placement. hpwl is
    # the wirelength metric whose minimum is at the truth, so it is the one that
    # can carry a gate. crossings is printed below and never tested.
    halo_gain, hpwl_gain = _gain('halo'), _gain('hpwl')
    if halo_gain is None or hpwl_gain is None:
        return True, ('  routability: NOT COMPARABLE -- the two renders do not '
                      'both carry numeric `halo` and `hpwl`. Re-render both '
                      'with --json-out at the same --clearance if you want '
                      'this read.'
                      + (f'\n  Reason on the record: {_waiver}'
                         if _waiver else ''))
    _cx = _gain('crossings')
    lines = [
        '  ROUTABILITY, measured against the board this run started from:',
        f'    halo       {before.get("halo"):9.1f} -> {after.get("halo"):9.1f}'
        f'   ({halo_gain * 100:+.1f}% of its gap closed)',
        f'    hpwl       {before.get("hpwl"):9.1f} -> {after.get("hpwl"):9.1f}'
        f'   ({hpwl_gain * 100:+.1f}% of its gap closed)',
        f'    crossings  {before.get("crossings"):9.0f} -> '
        f'{after.get("crossings"):9.0f}'
        + (f'   ({_cx * 100:+.1f}% of its gap closed)' if _cx is not None
           else ''),
    ]
    # DISTURBANCE. How much of the board did the repair touch, against how much
    # anything complained about? Run 16 moved 76 parts on a board where 24 refs
    # were named by any violation, 7 of those never moved, and NOTHING reported
    # it: the number was sitting in the mandated close-out render as
    # `moved_refs`, and the only guard that read it asked `d_moved.match` --
    # whether the count agreed with what the caller DECLARED. It did (76 == 76),
    # so the run read clean.
    #
    # This is a QUESTION, not an accusation, and the asymmetry is the point: a
    # `swap` displaces parts that land perfectly legally, so moving them back is
    # the repair and would show here as "disturbance" too. Nothing blind can
    # tell those apart -- which is exactly why the resolution is a sentence from
    # the executor rather than a verdict from the driver.
    _moved = _moved_count(a.render_json)
    _named = _implicated_refs(getattr(a, 'damage_json', None))
    _disturbs = False
    # WHICH BOARD the render measured against decides what this number MEANS.
    # P4 mandates each lap's render be made against THE PREVIOUS LAP, so a
    # close-out render can legitimately carry one lap's disturbance rather than
    # the run's. Run 16's happened to be run-scoped (76 == the whole run), which
    # is luck, not a guarantee. Label it rather than refuse: a wrong label on a
    # real number is the failure this whole gate exists to stop, and a refusal
    # here would fire on every correctly-rendered multi-lap run.
    _rb = ((_load(a.render_json, 'render')[0] or {}).get('instrument')
           or {}).get('before')
    _scoped = (os.path.normcase(os.path.abspath(_rb))
               == os.path.normcase(os.path.abspath(a.before))
               if (_rb and a.before) else False)
    if _moved is not None and not _scoped:
        lines.append('    disturbance  NOT RUN-SCOPED -- that render was made '
                     'against ' + (os.path.basename(_rb) if _rb
                                   else 'no --before board')
                     + ', not the board this run started from, so the count '
                       'below is one lap and the run total is larger.')
    if _moved is not None and _named:
        lines.append(f'    disturbance {_moved:9d} part(s) moved; '
                     f'{len(_named)} ref(s) named by a violation on the board '
                     f'this run started from')
        # 2x is a ROUND NUMBER, not a calibration: n=1, and this file already
        # withdrew one threshold fitted to a single board. It is set where a
        # one-line answer is cheap and silence is not, and the refusal says so.
        _disturbs = _moved > 2 * len(_named)
    elif _moved is not None:
        lines.append(f'    disturbance {_moved:9d} part(s) moved '
                     f'(pass --damage-json to compare against what was '
                     f'actually complained about)')
    def _tail():
        """Spend the disturbance read once the congestion read is settled."""
        if _disturbs and not _coll:
            return False, chr(10).join(lines + [
                '',
                'DISTURBANCE: this repair moved far more of the board than '
                'anything complained about.',
                'That is not automatically wrong -- damage that lands legally '
                'still has to be moved back,',
                'and nothing here can see which is which. But it is the shape '
                'of a search that wandered,',
                'and it must be answered rather than left for the reader: name '
                'the levers, or say why',
                'the board needed rearranging that widely.',
                '    --waive collateral:<the reason>',
            ])
        if _coll:
            lines.append('  DISTURBANCE on the record (--waive collateral): '
                         f'{_coll}')
        return True, chr(10).join(lines)

    if halo_gain >= 0.25 and hpwl_gain < 0.25 * halo_gain and _waiver:
        # Waived, WITH the read still on screen. This is the branch run 16
        # took, and the branch that used to print nothing whatsoever.
        lines += ['', '  DISPOSITION on the record (--waive congestion): '
                  f'{_waiver}']
        return _tail()
    if halo_gain >= 0.25 and hpwl_gain < 0.25 * halo_gain:
        # BINDING -- but on a DISPOSITION, not on the numbers. The driver does
        # not judge whether this placement is good; it judges whether anybody
        # ANSWERED. That distinction is the whole lesson of the calibration: a
        # gate that scored the board refused a PERFECT repair on piantor,
        # because hpwl inverts under `swap` there. A gate that asks for a
        # written decision cannot make that mistake -- the perfect repair
        # records why the read looks poor and proceeds.
        #
        # Merely PRINTING this was not enough, and that was a real regression
        # while it lasted: every other gate here is a legality gate, so a run in
        # exactly run 15's shape closed out clean with the evidence on screen
        # and nothing asked of it. Advisory is what the close-out already had
        # too much of.
        lines += [
            '',
            'DISPOSITION REQUIRED. Legality closed a large share of its gap and '
            'hpwl closed almost',
            'none. That is the shape of a repair that fixed the violations and '
            'left the arrangement',
            'as tangled as it found it -- on neo6502 that board routed to 29 '
            'unrouted nets, which',
            'the classifier then read as `parameter`, because every per-net test '
            'can pass on a board',
            'no router can finish.',
            '',
            'This is NOT a claim that your placement is wrong. The numbers are '
            'not judged -- they',
            'cannot be, and docs/placement-calibration.md says why: on one '
            'corpus board a PERFECT repair',
            'scores worse than the damage it repaired, because `swap` shortens '
            'nets on a regular',
            'matrix. What is required is that somebody decided.',
            '',
            'Either PULL A LEVER -- P4 with the corridor/affinity terms, or P5 '
            'for a different',
            'arrangement -- and close out on the result, or record why none '
            'applies:',
            '    --waive congestion:<the measurement, or the reason>',
            '',
            'Good reasons exist and are believed: the board is dense against its '
            'outline and every',
            'lever is spent; the parts that would move are locked; hpwl is known '
            'to invert on this',
            'damage kind. Say which.',
        ]
        return False, chr(10).join(lines)
    # A waiver on a read that did not trip is not an error, but it should not
    # vanish either -- somebody expected this to bind and it did not.
    if _waiver:
        lines += ['', '  DISPOSITION on the record (--waive congestion, which '
                  f'this read did not require): {_waiver}']
    return _tail()


#: Every stage's POPULATED body, held where it was measured (#937).
#:
#: Not one shared number: the 80-line assertion in `_self_test` measures
#: whatever the CHEAP fixture returns, so a stage that refuses there is
#: measured as its 3-line refusal and its instructions were never seen at all.
#: P4's 98 passed that way. These are the real figures, and they are ceilings
#: rather than targets -- their job is to stop silent growth, and raising one
#: is a deliberate edit here with a reason beside it.
#:
#: P4 is over the 80-line norm and pinned at what it is: the structural
#: finding is that its body holds FIVE VERBS (measure, act, prove, record,
#: loop), so the remedy is a split, not a trim.
#:
#: P0 70 -> 75 (#941 row 11), the deliberate edit this comment asks for. P0
#: mandated measuring "on the board with its copper removed" and named no lever
#: for removing it -- and the only full copper stripper in the tree,
#: tests/stress/strip_routing.py, is one non-negotiable 2 forbids BY NAME on a
#: user's board. So the stage ordered something it gave no way to do, and the
#: nearest tool was the forbidden one. Three lines name `route.py --undo`, its
#: two limits (refuses unscoped; leaves zone pours) and the prohibition. A
#: mandate with no lever is worse than three lines of body.
#:
#: P1 45 -> 60 (run 27). The ladder's rung 2 became "decide, THEN seed the
#: rest": a paragraph saying what the seeder is (a greedy first-fit with no
#: representation for a decision) and a sub-rung for placing and locking the
#: connectors and the mechanically-fixed parts before it runs. Measured on a
#: 21-part 2-layer board: left to the seeder, both free connectors came out at
#: rotation 0 and one of them took its declared band's midpoint through a
#: fixed socket's ground tab, on every one of ten seeds.
#:
#: P1 35 -> 45 and P0 75 -> 78 (run 26). P1 seeds FROM A ZONE PLAN now: its
#: body gained the plan's coverage line, a ranked-seeds command in place of
#: the single seed, and a facing step -- the three things run 26's placement
#: half did not have and hand-placed most of its parts without. P0 gained the
#: three-line handoff that carries the plan into P1, since P1 refuses without it.
_BODY_CEILING = {'P-brief': 60, 'P0': 78, 'P1': 60, 'P2': 45, 'P3': 80,
                 'P4': 100, 'P5': 45, 'P6': 30, 'P-close': 60}

STAGES = {
    'P-brief': p_brief,
    'P0': p0, 'P1': p1, 'P2': p2, 'P3': p3, 'P4': p4, 'P5': p5, 'P6': p6,
    'P-close': p_close,
}
TITLES = {
    'P-brief': 'what the board is FOR -- the declared design brief',
    'P0': 'gate -- should the placement be touched at all',
    'P1': 'unplaced ladder',
    'P2': 'the parts whose position is a mechanical fact',
    'P3': 'reconstruct a damaged placement',
    'P4': 'the fix loop (measure, one change, verify)',
    'P5': 'a slate of arrangements',
    'P6': 'declare the floorplan intent',
    'P-close': 'close out and hand on',
}


def _args(argv=None):
    ap = argparse.ArgumentParser(add_help=True, description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stage', choices=sorted(STAGES))
    ap.add_argument('--board', default='board.kicad_pcb')
    ap.add_argument('--before', default=None,
                    help='the board this one was derived from -- the IMMEDIATE '
                         'predecessor, which is what the render delta wants. '
                         'NOT the same thing as check_assembly --baseline, '
                         'which must be the board the RUN started from: its '
                         'courtyard conjunct gates only pairs whose members '
                         'MOVED relative to the baseline, so a per-lap baseline '
                         'hides every pair moved on an earlier lap and can read '
                         'buildable where the origin reads NOT BUILDABLE')
    ap.add_argument('--drc-json', default=None)
    ap.add_argument('--assembly-json', default=None)
    ap.add_argument('--locks-json', default=None)
    ap.add_argument('--render-json', default=None, metavar='PATH',
                    help='render_placement --json-out document for THIS board. '
                         'Required by every stage that follows a move (P4, P6, '
                         'P-close): the read mandates were prose-only, and a '
                         'whole campaign once ran with zero reads and nothing '
                         'noticed.')
    ap.add_argument('--intent-json', default=None, metavar='PATH',
                    help='the GRADED floorplan intent -- check_floorplan '
                         '--intent I --json PATH. P-close requires it, or '
                         '--waive intent:<why there is no spec to declare>. '
                         'P6 is the rung that produces one and is reachable '
                         'only as a side branch off P1/P4, so a run could walk '
                         'P0-P4-P-close and declare nothing: two laps were '
                         'once graded against an intent running ZERO rules, '
                         'and every tool consuming it said nothing.')
    ap.add_argument('--congestion-before', default=None, metavar='PATH',
                    help='render_placement --json-out of the --before board. '
                         'P-close compares its crossings/halo against the '
                         'close-out render, because a placement can clear every '
                         'legality gate while leaving the arrangement as '
                         'congested as it found it.')
    ap.add_argument('--damage-json', action='append', default=None,
                    metavar='PATH',
                    help='check_assembly.py / check_drc.py JSON of the board '
                         'this run STARTED from; repeatable. P-close reads the '
                         'refs they NAME and reports them beside the count of '
                         'parts that moved, because a repair that moved far '
                         'more of the board than anything complained about is '
                         'a question nothing else in the close-out asks. '
                         'Omitted, the move count still prints and nothing '
                         'refuses.')
    ap.add_argument('--zone-plan', default=None, metavar='PATH',
                    help='the floorplan intent P1 seeds FROM: a block with a '
                         '`zone` rectangle and a `note` for every movable '
                         'block (must_lock and declared edge connectors '
                         'excepted; a pad-less block is placed and locked by '
                         'hand, or named in `dispositions.refs`). P1 refuses '
                         'without one and names the blocks a plan leaves '
                         'out: run 26 seeded from a single zone and then '
                         'hand-placed most of its parts, one pose at a time, '
                         'with no plan anywhere.')
    # --congestion-ratio is GONE. It set a threshold P-close refused on, and the
    # calibration withdrew that refusal (docs/placement-calibration.md): the
    # premise inverts on 1 of 3 corpus boards, where a perfect repair scores a
    # negative hpwl gain and the gate refused the correct answer. A flag that
    # no longer
    # reaches a decision is the same lie as render_placement's --metrics was --
    # it reads as a knob somebody thought about. The routability numbers are
    # REPORTED at P-close now, and 0.25 is baked nowhere.
    ap.add_argument('--waive', action='append', default=[], metavar='REF:reason')
    ap.add_argument('--mechanical', default=None, metavar='PATH',
                    help='mechanical.json for P1 to reconcile (#959). '
                         'Discovered in the board\'s directory when omitted, '
                         'as stage_unaided writes it')
    ap.add_argument('--no-mechanical', action='store_true',
                    help='do not read mechanical.json even if one sits '
                         'beside the board (the OFF arm)')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--dump-all', action='store_true')
    ap.add_argument('--dump-refusals', action='store_true',
                    help='every REFUSAL this driver can print, guards '
                         'unsatisfied. --dump-all shows the instructions; this '
                         'shows the other branch, which is where a stuck '
                         'reader gets their next command (#923).')
    ap.add_argument('--self-test', action='store_true')
    return ap.parse_args(argv)


def main(argv=None):
    a = _args(argv)
    if a.list:
        # FROM THE REGISTRY, never a second hand-written tuple. The
        # tuple that used to live here omitted P-brief, so one
        # procedure had four stage counts -- STAGES 9, --list 8,
        # P-brief's own tag of="8" and the other eight of="7" -- and
        # the stage nobody could find is the only one that records a
        # design fact (#711). The driver's own refusals send a stuck
        # reader here to find the stages, so this list IS the index.
        for key in STAGES:
            print(f'  {key:8s} {TITLES[key]}')
        return 0
    if a.dump_all:
        return _dump_all()
    if a.dump_refusals:
        return _dump_refusals()
    if a.self_test:
        return _self_test()
    if not a.stage:
        print('placement_driver: --stage is required (see --list)',
              file=sys.stderr)
        return 2
    out = STAGES[a.stage](a)
    print(out)
    return 4 if out.startswith('<error>') else 0


def _fake_render(board, halo=100.0, crossings=100.0, hpwl=1000.0, moved=3):
    """The minimum render document `_guard_render` accepts, for the fixtures.

    Deliberately the REAL shape render_placement emits (`instrument.board` +
    a `checklist` with mandate 8's keys), not a stub that happens to pass:
    a fixture that satisfies a guard by a shape the real tool never produces
    would let the guard drift away from its instrument unnoticed.

    `metrics` is here for the same reason -- `_guard_congestion` reads
    halo/crossings/hpwl out of exactly this block, so a fixture without one
    would let that guard drift too. `moved_refs` likewise, for the disturbance
    read: it is the real serialised shape (a list of {reference, dist}), and a
    fixture carrying a bare count would let that read drift away from the
    render document it is written against.
    """
    return {
        'metrics': {'halo': halo, 'crossings': crossings, 'hpwl': hpwl},
        'instrument': {'board': os.path.abspath(board)},
        'moved': moved,
        'moved_refs': [{'reference': f'R{i + 1}', 'dist': 1.0}
                       for i in range(moved)],
        'checklist': {
            'a_off_outline': {'pad_copper': [], 'courtyard': []},
            'b_pad_clearance_pairs': [], 'b_body_overlap_pairs': [],
            'c_hole_conflicts': [], 'c_locked_refs': [],
            'd_moved': {'moved': moved, 'expected': None, 'match': None},
        },
    }


def _tiny_board(path, refs, unconnected=(), locked=(), padless=(),
                courtyard=()):
    """A board `parse_kicad_pcb` reads: an outline and one part per ref, each
    with one pad -- connected, except for the refs in `unconnected` (a
    mounting hole, a fiducial), which the seeder moves all the same. Refs in
    `locked` carry `(locked yes)`, which is what `place_pose lock` writes and
    what the zone-plan guard reads as "the author decided this one". Refs in
    `padless` carry NO pad at all -- a logo or a graphic, which the seeder
    never moves (#959); those in `courtyard` also draw an F.CrtYd rectangle,
    which is what lets `zone_containment` grade a pad-less block at all. On
    disk, because the guard reads the file the flag names."""
    def _pad(r):
        if r in padless:
            return ('    (fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd") '
                    '(width 0.05))\n' if r in courtyard else '')
        return (f'    (pad "1" smd rect (at 0 0) (size 0.6 0.8) (layers "F.Cu") '
                f'(net {0 if r in unconnected else 1} '
                f'"{"" if r in unconnected else "/A"}") (uuid "p1-{r}"))\n')
    fps = ''.join(
        f'  (footprint "test:FP" (layer "F.Cu") (uuid "fp-{r}") (at {2 + 3 * i} 2)'
        f'{" (locked yes)" if r in locked else ""}\n'
        f'    (property "Reference" "{r}" (at 0 0))\n'
        + _pad(r) +
        '  )\n' for i, r in enumerate(refs))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "test")\n'
                 '  (net 0 "")\n  (net 1 "/A")\n'
                 '  (gr_rect (start 0 0) (end 20 10) (layer "Edge.Cuts") '
                 '(uuid "e1"))\n' + fps + ')\n')
    return path


def _no_outline_board(path):
    """`_tiny_board` with its Edge.Cuts rectangle removed: parts the guard can
    resolve, and no outline anything can be graded against (#959)."""
    _tiny_board(path, ('U1', 'U2'))
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    text = '\n'.join(line for line in text.split('\n')
                     if 'Edge.Cuts' not in line)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _zone_plan_doc(blocks, **extra):
    """A floorplan intent carrying `blocks` and nothing else the guard
    does not ask for -- plus the two written dispositions the tiny board's
    roster owes (#959): it declares no envelope and no legality budget, both
    apply to any board with an outline, and both are gating. A scenario that
    means to exercise the roster refusal passes `dispositions={}`."""
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
           'blocks': blocks,
           'dispositions': {'rules': {
               'envelope': 'the fixture board is its own envelope',
               'legality': 'the fixture grades placement, not legality'}}}
    # A scenario that ARMS one of the two drops its default disposition --
    # a disposition for an armed rule is refused at load, so keeping it
    # would fail the scenario before it reached the check it is about.
    for key, rule in (('envelope', 'envelope'),
                      ('legality_budget', 'legality')):
        if key in extra and 'dispositions' not in extra:
            doc['dispositions']['rules'].pop(rule, None)
    doc.update(extra)
    if not doc['dispositions'] or doc['dispositions'] == {'rules': {}}:
        del doc['dispositions']
    return doc


def _next_line_fixture(tmp):
    """Fabricated evidence for every guard, as flag -> path.

    The same shape `_dump_all` builds, exposed so the self-test can re-point a
    printed `Next:` command's flags at real files and find out whether the flag
    SET the line names is enough to reach the stage it names.
    """
    import json as _json

    def wrote(name, doc):
        p = os.path.join(tmp, name)
        with open(p, 'w', encoding='utf-8') as fh:
            _json.dump(doc, fh)
        return p

    # The board is a REAL two-part board, not an empty file: P1's zone-plan
    # guard parses it and resolves the plan's blocks against it, and an
    # empty file would make every populated dump of P1 a refusal.
    board = _tiny_board(os.path.join(tmp, 'b.kicad_pcb'), ('U1', 'U2'))
    before = os.path.join(tmp, 'a.kicad_pcb')
    open(before, 'w', encoding='utf-8').close()
    return {
        'board': board,
        'flags': {
            '--board': board,
            '--before': before,
            '--zone-plan': wrote('zp.json', _zone_plan_doc(
                [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                  'note': 'the fixture plan: both parts, one zone'}])),
            '--drc-json': wrote('d.json', {'violations': 3}),
            '--locks-json': wrote('l.json', {'findings': [],
                                             'lock_patterns': []}),
            '--assembly-json': wrote('as.json', {'blocking': 1}),
            '--render-json': wrote('r.json', _fake_render(
                board, halo=50.0, crossings=60.0, hpwl=800.0)),
            '--congestion-before': wrote('cb.json', _fake_render(
                before, halo=100.0, crossings=100.0, hpwl=1000.0)),
            '--intent-json': wrote('i.json', {
                'rules_run': ['envelope'], 'parts_covered': 7,
                'violations': [],
                'brief_coverage': {'brief': None, 'clauses': [], 'graded': 0,
                                   'uncovered': 0, 'abstained': 0,
                                   'complete': True}}),
        },
    }


def _fixture_argv(fix):
    """`_next_line_fixture`'s flag map as an argv list."""
    argv = []
    for flag, path in fix['flags'].items():
        argv += [flag, path]
    return argv


def _render_for_next(key, fix):
    """One stage's body under complete evidence, for reading its Next: lines."""
    return STAGES[key](
        _args(_fixture_argv(fix) + ['--waive', 'X:checked']))


def _next_commands(body):
    r"""[(stages, flags)] for every `Next:` handoff in a rendered stage body.

    `stages` is a list because a handoff may offer a choice (`<P4|P6>`); every
    branch of it has to reach its stage, not just the first.

    THREE SHAPES HAVE SILENTLY DEFEATED THIS PARSER, each while the remaining
    labels still printed PASS, so each is answered explicitly below:

    * stopping the block at the first line missed a handoff whose command sat
      two prose lines down;
    * keying the label on `Next:` missed one reworded to `Next, for P4 only:`;
    * ending the block at a blank line missed a label whose command was a
      paragraph away.

    A block therefore runs from one `Next` label to the NEXT one (or the
    closing tag), the label match is `^\s*Next\b` so indentation cannot hide
    it, and every `--stage` in the block is a handoff rather than only the
    first.

    Flags are scoped to the COMMAND that carries the `--stage`, never to the
    whole paragraph: taking them paragraph-wide lets a handoff pass by
    mentioning a flag in prose while the printed command omits it. Only the
    flag NAMES are used -- the printed values are placeholders a reader fills
    in, and whether `<adopted>` exists is not what is being tested.
    """
    import re as _re
    lines = body.splitlines()
    heads = [i for i, ln in enumerate(lines) if _re.match(r'\s*Next\b', ln)]
    out = []
    for n, i in enumerate(heads):
        end = heads[n + 1] if n + 1 < len(heads) else len(lines)
        block = []
        for ln in lines[i:end]:
            if ln.startswith('</'):
                break
            block.append(ln)
        # Split the block into the commands it prints. A COMMAND is a line
        # invoking python3 plus exactly its backslash continuations -- nothing
        # else. Ending a command only at the next `python3` folded any prose
        # AFTER it into the command, so a handoff could pass by mentioning a
        # flag in a following sentence while its printed command omitted it:
        # the same defeat as prose-before, from the other side, and the
        # docstring above claimed immunity to both. Prose is still collected
        # into its own segment, so a handoff written as bare prose is read.
        segs, cur, cont = [], [], False
        for ln in block:
            starts = 'python3' in ln
            if cont:                       # a continuation of the command in cur
                cur.append(ln)
                cont = ln.rstrip().endswith('\\')
                continue
            if cur and (starts or any('python3' in c for c in cur)):
                # A new invocation, or prose AFTER a command that has ended.
                segs.append(cur)
                cur = []
            cur.append(ln)
            cont = starts and ln.rstrip().endswith('\\')
        segs.append(cur)
        for seg in segs:
            text = ' '.join(s.rstrip('\\').strip() for s in seg)
            flags = [f for f in _re.findall(r'(?<![\w-])(--[a-z][a-z-]+)', text)
                     if f != '--stage']
            for m in _re.finditer(r'--stage[=\s]+(\S+)', text):
                raw = m.group(1).strip('`\'"')
                out.append(([s for s in raw.strip('<>').split('|')]
                            if raw.startswith('<') else [raw], flags))
    return out


def _dump_all():
    """Every stage's REAL body, guards satisfied.

    This used to pass filenames that do not exist, so P2, P3 and P4 dumped
    their REFUSALS -- three of the nine, including the two that carry the
    most commands. Anything auditing the driver through --dump-all (a flag
    checker, a reviewer, a person) was reading error text and seeing no
    commands to be wrong. Guard evidence is cheap to fabricate HERE, where the
    point is to show the instructions rather than to act on them.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        # ONE fabrication, shared with the Next: arm of --self-test.
        # It was copied there and the copy immediately drifted (a
        # `--locks-json` missing `lock_patterns`), and nothing asserted
        # the two agreed -- so a drift affecting one source stage would
        # have landed as a quietly smaller count.
        loose = _args(_fixture_argv(_next_line_fixture(tmp))
                      + ['--waive', 'X:checked'])
        refused = []
        for key in sorted(STAGES):
            body = STAGES[key](loose)
            print(f'===== {key} =====')
            print(body)
            if body.startswith('<error>'):
                refused.append(key)
    if refused:
        # Loud, because a silently-refusing dump is what hid this for a while.
        print(f'\n!! {len(refused)} stage(s) dumped a REFUSAL, not their '
              f'instructions: {", ".join(refused)}')
        return 1
    return 0


# --------------------------------------------------------------------------
# the REFUSALS (#923) -- the other half of --dump-all
# --------------------------------------------------------------------------
#: The shortest literal worth checking. MEASURED rather than chosen: at 40
#: characters six placement sites and three loop sites carried nothing long
#: enough to check and counted as rendered without being looked at -- one of
#: them `_load`'s "unreadable" branch, whose longest literal is
#: `': unreadable ('`. At 12 every site that carries a literal at all becomes
#: checkable, and what is left is only the pass-throughs (`err(why)`), whose
#: text belongs to the guard that composed it and is checked there.
_CHUNK = 12


def _refusal_sites(path=None):
    """Every place this file can refuse, and the TEXT each one prints.

    Two shapes: an `err(...)` call, and a `return <False|None>, '<text>'` that
    a caller wraps in `err()`.

    NEITHER IS FILTERED BY FUNCTION NAME. The first version asked whether the
    enclosing function was called `_guard_*` / `_load` / `_metrics_of` -- and
    in the sibling driver `_count`, nested inside a stage, composes three
    refusals that matched none of those, so its texts were not sites at all
    while the dump reported 100% coverage. A hand-written prefix is the
    hand-written list this whole mechanism exists to stop trusting.

    Coverage is measured on the TEXT, not on the line: one `err(...)` can carry
    four arms (a `_bucket(...)` per clause state, a ternary's two halves), and a
    line-granular check calls the whole call rendered when one arm ran. Each
    site therefore carries every string literal it can print of at least
    `_CHUNK` characters, and it counts as rendered only when the dump contains
    all of them.

    Returns {(line, col): (function, kind, [chunks])}.
    """
    import ast
    path = path or os.path.abspath(__file__)
    with open(path, encoding='utf-8') as fh:
        tree = ast.parse(fh.read())

    def chunks(node):
        return [sub.value for sub in ast.walk(node)
                if isinstance(sub, ast.Constant) and isinstance(sub.value, str)
                and len(sub.value.strip()) >= _CHUNK]

    owner = {}
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef):
            for sub in ast.walk(fn):
                owner[id(sub)] = fn.name

    sites = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'err'):
            # An `err(why)` carries no literal of its own: its text was
            # composed by a guard, which is a site there. Registering it here
            # with zero chunks made it "fully rendered" without anything being
            # looked at -- eleven of them across the two drivers -- so it is
            # counted as a pass-through instead.
            got = chunks(node)
            if got:
                sites[(node.lineno, node.col_offset)] = (
                    owner.get(id(node), '<module>'), 'err', got)
        elif (isinstance(node, ast.Return)
                and isinstance(node.value, ast.Tuple)
                and len(node.value.elts) == 2):
            head, text = node.value.elts
            # A refusal is a falsy first element with TEXT beside it.
            # `return True, ''` and `return json.load(fh), None` are the
            # SUCCESS shapes of the same helpers.
            falsy = (isinstance(head, ast.Constant)
                     and head.value in (False, None))
            got = chunks(text)
            if falsy and got:
                sites[(node.lineno, node.col_offset)] = (
                    owner.get(id(node), '<module>'), 'guard', got)
    return sites



def _passthrough_count(path=None):
    """Refusal sites that carry NO literal of their own: `err(why)`.

    Reported beside the coverage number so it is read for what it is. Their
    text was composed by a guard, which is a site of its own and is checked
    there; counting them as covered without saying so is how "N of N" starts
    meaning less than it looks.
    """
    import ast
    path = path or os.path.abspath(__file__)
    with open(path, encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    n = 0
    for node in ast.walk(tree):
        target = None
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'err'):
            target = node
        elif (isinstance(node, ast.Return) and isinstance(node.value, ast.Tuple)
                and len(node.value.elts) == 2):
            head = node.value.elts[0]
            if isinstance(head, ast.Constant) and head.value in (False, None):
                target = node.value.elts[1]
        if target is None:
            continue
        if not any(isinstance(s, ast.Constant) and isinstance(s.value, str)
                   and len(s.value.strip()) >= _CHUNK
                   for s in ast.walk(target)):
            n += 1
    return n

def _refusal_scenarios(tmp):
    """Evidence-STARVED namespaces: one per guard branch, each labelled.

    The mirror of `_dump_all`'s single satisfied namespace. No row is
    load-bearing on its own -- `_dump_refusals` measures which branches the set
    actually reached and names the ones it did not -- so this is a starting
    point for the trace, not the definition of what gets covered.
    """
    def wrote(name, doc):
        p = os.path.join(tmp, name)
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        return p

    board = os.path.join(tmp, 'b.kicad_pcb')
    before = os.path.join(tmp, 'a.kicad_pcb')
    other = os.path.join(tmp, 'other.kicad_pcb')
    for p in (board, before, other):
        open(p, 'w', encoding='utf-8').close()
    missing = os.path.join(tmp, 'nope.json')
    unreadable = os.path.join(tmp, 'truncated.json')
    with open(unreadable, 'w', encoding='utf-8') as fh:
        fh.write('{"violations": 12')          # a real half-written artifact

    def render(**kw):
        """A `_fake_render` with one field bent, written out."""
        name = kw.pop('name')
        doc = _fake_render(kw.pop('of', board), **{
            k: v for k, v in kw.items()
            if k in ('halo', 'crossings', 'hpwl', 'moved')})
        for k, v in kw.items():
            if k in ('halo', 'crossings', 'hpwl', 'moved'):
                continue
            if v is _DROP:
                doc.pop(k, None)
            else:
                doc[k] = v
        return wrote(name, doc)

    base = ['--board', board]
    with_before = base + ['--before', before]
    damaged = ['--drc-json', wrote('dmg.json', {'violations': 12})]
    good_render = ['--render-json', render(name='r_ok.json')]
    graded_intent = ['--intent-json', wrote('i_ok.json', {
        'rules_run': ['envelope'], 'violations': [],
        'brief_coverage': {'schema': 1, 'brief': None, 'clauses': [],
                           'graded': 0, 'uncovered': 0, 'abstained': 0,
                           'complete': True}})]

    tiny = _tiny_board(os.path.join(tmp, 'tiny.kicad_pcb'), ('U1', 'U2'))
    logo_board = _tiny_board(os.path.join(tmp, 'logo.kicad_pcb'),
                             ('U1', 'U2', 'LOGO1'), padless=('LOGO1',))
    courted_board = _tiny_board(os.path.join(tmp, 'logo_cy.kicad_pcb'),
                                ('U1', 'U2', 'LOGO1'), padless=('LOGO1',),
                                courtyard=('LOGO1',))
    locked_logo_board = _tiny_board(
        os.path.join(tmp, 'logo_lk.kicad_pcb'), ('U1', 'U2', 'LOGO1'),
        padless=('LOGO1',), locked=('LOGO1',))

    def mech_board(name, brief_edge=None, mech=None, brief_raw=None):
        """A tiny board in its OWN directory, optionally with a sibling
        design brief declaring U2's edge (or `brief_raw` verbatim) and a
        `mechanical.json` (#959)."""
        d = os.path.join(tmp, 'mech_' + name)
        os.makedirs(d, exist_ok=True)
        b = _tiny_board(os.path.join(d, 'board.kicad_pcb'), ('U1', 'U2'))
        if brief_raw is not None:
            with open(os.path.join(d, 'board.design-brief.json'), 'w',
                      encoding='utf-8') as fh:
                json.dump(brief_raw, fh)
        if brief_edge:
            with open(os.path.join(d, 'board.design-brief.json'), 'w',
                      encoding='utf-8') as fh:
                json.dump({'schema': 1, 'kind': 'design-brief',
                           'units': 'mm', 'board': 'board.kicad_pcb',
                           'interfaces': [{'ref': 'U2', 'edge': brief_edge,
                                           'user_facing': True}]}, fh)
        if mech is not None:
            with open(os.path.join(d, 'mechanical.json'), 'w',
                      encoding='utf-8') as fh:
                json.dump(mech, fh)
        return b
    zp_ok = wrote('zp_ok.json', _zone_plan_doc(
        [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
          'note': 'both parts, one zone'}]))

    def clause_intent(name, **row):
        r = {'id': 'proximity[0:Y1~U1].max_mm', 'state': 'uncovered'}
        r.update(row)
        return ['--intent-json', wrote(name, {
            'rules_run': ['proximity'], 'violations': [],
            'brief_coverage': {'schema': 1, 'clauses': [r]}})]

    def coverage(name, cov):
        return ['--intent-json', wrote(name, {'rules_run': ['envelope'],
                                              'brief_coverage': cov})]

    return [
        # nothing at all -- every "not provided" text, and the --before gates
        ('no evidence at all', base),
        ('--before names nothing', base + ['--before', os.path.join(
            tmp, 'nope.kicad_pcb')]),
        # _guard_damage
        ('a DRC json that measures nothing', base
         + ['--drc-json', wrote('bare.json', {'schema': 1})]),
        ('a JSON file that does not parse', base + ['--drc-json', unreadable]),
        # BOTH ARMS of the no-damage refusal (#937). The first has no
        # assembly document at all, so the guard falls back to `blocking`;
        # the second supplies a verdict, which is the field it now reads --
        # `blocking` is 1 of check_assembly's 5 not_buildable conjuncts since
        # #918, so a board can be NOT BUILDABLE at blocking 0 and this guard
        # used to call that "no damage to repair".
        ('a board with no damage to repair', base
         + ['--drc-json', wrote('clean.json', {'violations': 0})]),
        ('a board the assembly verdict calls buildable', base
         + ['--drc-json', wrote('clean2.json', {'violations': 0}),
            '--assembly-json', wrote('asm_ok.json',
                                     {'buildable': True, 'blocking': 0,
                                      'verdict': 'buildable (blocking 0)'})]),
        # _guard_zone_plan (run 26): P1 seeds FROM a plan, one row per way
        # the plan fails to be one. The base board is an EMPTY file, which
        # is the no-footprint arm; the missing-plan arm is `no evidence at
        # all` above.
        ('a zone plan that is not an intent', base
         + ['--zone-plan', wrote('zp_bad.json', {'schema': 1, 'blocks': 'x'})]),
        ('a zone plan whose blocks carry no zone', base
         + ['--zone-plan', wrote('zp_nozone.json', _zone_plan_doc(
             [{'name': 'a', 'refs': ['U*']}]))]),
        ('a zoned block with no note', base
         + ['--zone-plan', wrote('zp_nonote.json', _zone_plan_doc(
             [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 10, 10]}]))]),
        # The two rows below name a board OTHER than `base`'s, and every
        # stage runs on every row: P2's damage refusal embeds the board
        # path in a check_drc command, so a new path is a new command span
        # on a tool test_431 cannot value-check (it pins that count).
        # `damaged` satisfies that guard, so P2 renders its body instead.
        ('a zone plan over a board that cannot be read',
         ['--board', os.path.join(tmp, 'nope.kicad_pcb'), '--zone-plan', zp_ok]
         + damaged),
        ('a zone plan over a board with no footprint', base
         + ['--zone-plan', zp_ok]),
        ('a zone plan leaving parts uncovered',
         ['--board', tiny, '--zone-plan', wrote('zp_half.json', _zone_plan_doc(
             [{'name': 'a', 'refs': ['U1'], 'zone': [0, 0, 5, 5],
               'note': 'U1 only'}]))]
         + damaged),
        # run 27: the seeder does not choose a declared connector's pose, and
        # the hand-over that lets it needs a reason. Both arms, because the
        # no-reason one is a different text.
        ('a declared connector left for the seeder',
         ['--board', tiny, '--zone-plan', wrote('zp_ec.json', _zone_plan_doc(
             [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
               'note': 'the ICs'}],
             edge_connectors=[{'ref': 'U2', 'edge': 'west'}]))]
         + damaged),
        ('a seed-connectors waiver with no reason',
         ['--board', tiny, '--zone-plan', wrote(
             'zp_ec2.json', _zone_plan_doc(
                 [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
                   'note': 'the ICs'}],
                 edge_connectors=[{'ref': 'U2', 'edge': 'west'}])),
            '--waive', 'seed-connectors:']
         + damaged),
        # #959 (#999): the PAD-LESS block arms, one row per text. The board
        # adds a logo with no pad to the two ICs.
        ('a pad-less block nothing answers for',
         ['--board', logo_board, '--zone-plan', zp_ok] + damaged),
        ('a pad-less block inside a zoned block',
         ['--board', logo_board, '--zone-plan', wrote(
             'zp_logo_zoned.json', _zone_plan_doc(
                 [{'name': 'all', 'refs': ['U*', 'LOGO1'],
                   'zone': [0, 0, 10, 10], 'note': 'ICs and the logo'}]))]
         + damaged),
        # The same zone around a logo already LOCKED: the advice changes to
        # "remove it from the block", since the lock is already there.
        ('a locked pad-less block named in a zoned block',
         ['--board', locked_logo_board, '--zone-plan', wrote(
             'zp_logo_zoned_locked.json', _zone_plan_doc(
                 [{'name': 'all', 'refs': ['U*', 'LOGO1'],
                   'zone': [0, 0, 10, 10], 'note': 'ICs and the logo'}]))]
         + damaged),
        ('a pad-less disposition naming a block the board lacks',
         ['--board', logo_board, '--zone-plan', wrote(
             'zp_logo_unknown.json', _zone_plan_doc(
                 [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                   'note': 'both ICs'}],
                 dispositions={'refs': {'LOGO9': 'no such block'}}))]
         + damaged),
        ('a pad-less block with a courtyard, zoned but not locked',
         ['--board', courted_board, '--zone-plan', wrote(
             'zp_logo_courted.json', _zone_plan_doc(
                 [{'name': 'all', 'refs': ['U*', 'LOGO1'],
                   'zone': [0, 0, 10, 10], 'note': 'ICs and the logo'}]))]
         + damaged),
        ('a pad-less disposition for a block already locked',
         ['--board', locked_logo_board, '--zone-plan', wrote(
             'zp_logo_locked.json', _zone_plan_doc(
                 [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                   'note': 'both ICs'}],
                 dispositions={'refs': {'LOGO1': 'already answered'}}))]
         + damaged),
        ('a pad-less disposition naming a block with pads',
         ['--board', logo_board, '--zone-plan', wrote(
             'zp_logo_padded.json', _zone_plan_doc(
                 [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                   'note': 'both ICs'}],
                 dispositions={'refs': {'U1': 'wrong kind of block'}}))]
         + damaged),
        # #959 (#1001): the declared channels. Each board lives in its OWN
        # directory, because `mechanical.json` is discovered per directory and
        # one in `tmp` would change every other row's board.
        ('a brief and mechanical.json that contradict each other',
         ['--board', mech_board('contra', brief_edge='east',
                                mech={'interfaces': [{'ref': 'U2',
                                                      'edge': 'west'}]}),
          '--zone-plan', zp_ok] + damaged),
        ('a design brief that does not compile',
         ['--board', mech_board('badbrief', brief_raw={
             'schema': 1, 'kind': 'design-brief', 'units': 'mm',
             'board': 'board.kicad_pcb', 'bogus_key': 1}),
          '--zone-plan', zp_ok] + damaged),
        ('a contradiction disposition that answers nothing',
         ['--board', mech_board('stale'),
          '--zone-plan', wrote('zp_stale_contra.json', _zone_plan_doc(
              [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                'note': 'both parts, one zone'}],
              dispositions={'contradictions': {'U9:edge': 'no such row'},
                            'rules': {'envelope': 'fixture',
                                      'legality': 'fixture'}}))]
         + damaged),
        ('a plan edge a recorded mechanical edge outranks',
         ['--board', mech_board('beaten', mech={
             'interfaces': [{'ref': 'U2', 'edge': 'west'}]}),
          '--zone-plan', wrote('zp_beaten.json', _zone_plan_doc(
              [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                'note': 'both parts, one zone'}],
              edge_connectors=[{'ref': 'U2', 'edge': 'east'}])),
          '--waive', 'seed-connectors:the fixture hands U2 over']
         + damaged),
        ('a mechanical ref the board does not lock',
         ['--board', mech_board('unlocked', mech={
             'fixed': [{'ref': 'U1', 'x': 2.0, 'y': 2.0, 'rot': 0,
                        'reason': 'the mounting datum'}]}),
          '--zone-plan', zp_ok] + damaged),
        ('a mechanical.json that does not read',
         ['--board', mech_board('garbled', mech={'nonsense': 1}),
          '--zone-plan', zp_ok] + damaged),
        ('a zone plan that drifts from the brief',
         ['--board', mech_board('drift', brief_edge='east'),
          '--zone-plan', wrote('zp_drift.json', _zone_plan_doc(
              [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                'note': 'both parts, one zone'}],
              edge_connectors=[{'ref': 'U2', 'edge': 'west'}])),
          '--waive', 'seed-connectors:the fixture hands U2 over']
         + damaged),
        ('a brief-clause waiver at P1 with no reason',
         ['--board', mech_board('drift2', brief_edge='east'),
          '--zone-plan', wrote('zp_drift2.json', _zone_plan_doc(
              [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                'note': 'both parts, one zone'}],
              edge_connectors=[{'ref': 'U2', 'edge': 'west'}])),
          '--waive', 'seed-connectors:the fixture hands U2 over',
          '--waive', 'brief-clause:interfaces[U2].edge:']
         + damaged),
        # #959 (#998): a plan no arrangement can satisfy -- two 0.6 x 0.8 mm
        # parts need 0.96 mm2 and their zone (tolerance 0) holds 0.81, so
        # they overlap by at least 0.15 mm2, over a declared budget of 0.
        ('a zone plan whose zone cannot hold its members',
         ['--board', tiny, '--zone-plan', wrote('zp_overfull.json',
                                                 _zone_plan_doc(
             [{'name': 'all', 'refs': ['U*'], 'zone': [1.5, 1.5, 2.4, 2.4],
               'tolerance_mm': 0, 'note': 'both parts, one tiny zone'}],
             legality_budget={'overlap_area': 0}))]
         + damaged),
        # #959 (#997): the roster, LAST in P1. One row carries both arms -- a
        # gating rule nothing answers for (the tiny board declares no
        # envelope and no legality budget) and a disposition that answers a
        # withheld key nothing withheld -- so every literal renders.
        ('a zone plan that leaves gating rules dark',
         ['--board', tiny, '--zone-plan', wrote('zp_dark.json', _zone_plan_doc(
             [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
               'note': 'both parts, one zone'}],
             dispositions={'withheld': {'overlap_area': 'stale on purpose'}}))]
         + damaged),
        # ...and a plan whose ONLY debt is the stale one, so the header that
        # says so renders too.
        ('a zone plan whose only debt is a stale disposition',
         ['--board', tiny, '--zone-plan', wrote(
             'zp_stale_only.json', _zone_plan_doc(
                 [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                   'note': 'both parts, one zone'}],
                 dispositions={
                     'rules': {'envelope': 'the fixture is its own envelope',
                               'legality': 'the fixture grades placement'},
                     'withheld': {'overlap_area': 'stale on purpose'}}))]
         + damaged),
        ('a zone plan over a board with no outline',
         ['--board', _no_outline_board(os.path.join(tmp, 'noedge.kicad_pcb')),
          '--zone-plan', zp_ok] + damaged),
        # P3's lock advice
        ('no lock advice', base + damaged),
        ('unlocked_high with nothing waived', base + damaged
         + ['--locks-json', wrote('locks.json', {'unlocked_high': 3})]),
        ('a waiver that does not parse', base + damaged
         + ['--locks-json', wrote('locks2.json', {'unlocked_high': 3}),
            '--waive', 'R1']),
        # _guard_render -- one row per way a render fails to be evidence
        ('a render that is not there', with_before + ['--render-json', missing]),
        ('a render with no instrument.board', with_before
         + ['--render-json', render(name='r_noinst.json', instrument={})]),
        ('a render of a different board', with_before
         + ['--render-json', render(name='r_other.json', of=other)]),
        ('a render with no checklist', with_before
         + ['--render-json', render(name='r_nochk.json', checklist=_DROP)]),
        ('a render naming parts with pad copper off the outline', with_before
         + ['--render-json', render(name='r_oob.json', checklist={
             'a_off_outline': {'pad_copper': [{'reference': 'U7'},
                                              {'reference': 'J2'}],
                               'courtyard': []},
             'd_moved': {'moved': 3, 'expected': None, 'match': None}})]),
        # ...and the by-design escape WITHOUT its reason (#937). The waiver
        # exists because a castellated row or a card edge is meant to cross
        # the outline and this census has no exemption for one; a waiver with
        # no reason is refused rather than honoured.
        ('an off-outline waiver with no reason', with_before
         + ['--waive', 'off-outline:',
            '--render-json', render(name='r_oobw.json', checklist={
                'a_off_outline': {'pad_copper': [{'reference': 'J9'}],
                                  'courtyard': []},
                'd_moved': {'moved': 1, 'expected': None, 'match': None}})]),
        # ...and the same list carrying no `reference`, which is the arm that
        # falls back to naming the key. Without this row that fallback is a
        # branch nothing renders -- exactly what --dump-refusals exists to say.
        ('a render with off-outline pad copper it cannot attribute', with_before
         + ['--render-json', render(name='r_oob_anon.json', checklist={
             'a_off_outline': {'pad_copper': [{'amount_mm': 1.2}],
                               'courtyard': []},
             'd_moved': {'moved': 3, 'expected': None, 'match': None}})]),
        # #1031: pads in a rule-area keep-out band, named, unattributable,
        # and the keepout-band waiver without its reason
        ('a render naming parts with pads in a keep-out band', with_before
         + ['--render-json', render(name='r_ko.json', checklist={
             'a_off_outline': {'pad_copper': [], 'courtyard': [],
                               'keepout_copper': [['R12', 0.284],
                                                  ['U14', 0.187]]},
             'd_moved': {'moved': 2, 'expected': None, 'match': None}})]),
        ('a render with NEW keep-out band pads against its --before board',
         with_before + ['--render-json', render(name='r_ko_new.json', checklist={
             'a_off_outline': {'pad_copper': [], 'courtyard': [],
                               'keepout_copper': [['C6', 0.1028],
                                                  ['U14', 0.187]],
                               'keepout_copper_before': [['C6', 0.1028]]},
             'd_moved': {'moved': 1, 'expected': None, 'match': None}})]),
        ('a render with keep-out band pads it cannot attribute', with_before
         + ['--render-json', render(name='r_ko_anon.json', checklist={
             'a_off_outline': {'pad_copper': [], 'courtyard': [],
                               'keepout_copper': [{'amount_mm': 0.3}]},
             'd_moved': {'moved': 2, 'expected': None, 'match': None}})]),
        ('a render whose keep-out census could not be built', with_before
         + ['--render-json', render(name='r_ko_err.json', checklist={
             'a_off_outline': {'pad_copper': [], 'courtyard': [],
                               'keepout_copper': [],
                               'keepout_copper_unmeasured': [
                                   ['*', 'error', 'ValueError: fixture']]},
             'd_moved': {'moved': 1, 'expected': None, 'match': None}})]),
        ('a keepout-band waiver with no reason', with_before
         + ['--waive', 'keepout-band:',
            '--render-json', render(name='r_kow.json', checklist={
                'a_off_outline': {'pad_copper': [], 'courtyard': [],
                                  'keepout_copper': [['R12', 0.284]]},
                'd_moved': {'moved': 1, 'expected': None, 'match': None}})]),
        ('a render that disagrees on the move count', with_before
         + ['--render-json', render(name='r_moved.json', checklist={
             'd_moved': {'moved': 9, 'expected': 3, 'match': False}})]),
        ('a render asked for a sheet that was never written', with_before
         + ['--render-json', render(name='r_sheet.json', review_sheet=None)]),
        ('a render naming a sheet that is not there', with_before
         + ['--render-json', render(name='r_sheet2.json',
                                    review_sheet=os.path.join(tmp, 'no.md'))]),
        # P-close's intent ladder
        ('a close-out with no intent', with_before + good_render),
        ('an intent waived with no reason', with_before + good_render
         + ['--waive', 'intent:']),
        ('an intent path that opens nothing', with_before + good_render
         + ['--intent-json', missing]),
        ('rules_run of a shape this gate cannot read', with_before + good_render
         + ['--intent-json', wrote('i_shape.json', {'rules_run': 'six'})]),
        ('an intent that graded nothing', with_before + good_render
         + ['--intent-json', wrote('i_zero.json', {'rules_run': []})]),
        ('a brief_coverage that is not an object', with_before + good_render
         + coverage('i_cov.json', [])),
        ('a brief_coverage of an unknown schema', with_before + good_render
         + coverage('i_schema.json', {'schema': 9})),
        ('brief_coverage.clauses that is not a list', with_before + good_render
         + coverage('i_clauses.json', {'schema': 1, 'clauses': 3})),
        ('an intent with no coverage block at all', with_before + good_render
         + ['--intent-json', wrote('i_nocov.json', {'rules_run': ['envelope']})]),
        ('a clauses entry that is not a clause', with_before + good_render
         + coverage('i_junk.json', {'schema': 1, 'clauses': ['x']})),
        ('a clause waiver with no reason', with_before + good_render
         + clause_intent('i_wr.json')
         + ['--waive', 'brief-clause:proximity[0:Y1~U1].max_mm:']),
        ('a clause waiver naming nothing', with_before + good_render
         + clause_intent('i_ph.json')
         + ['--waive', 'brief-clause:nosuch.clause:because']),
        ('a clause nothing graded', with_before + good_render
         + clause_intent('i_open.json')),
        # ...and the other three buckets of the same refusal. A line-granular
        # coverage check called this site rendered once ANY of them ran, and
        # three of the four texts a reader can be handed here had never been
        # printed -- including the catch-all whose own comment says it exists so
        # the list can never come out blank.
        ('a clause the author declared unknown', with_before + good_render
         + clause_intent('i_abst.json', state='abstained')),
        ('a clause graded against the wrong number', with_before + good_render
         + clause_intent('i_drift.json', state='graded', drifted=True)),
        ('a clause state this gate has never seen', with_before + good_render
         + clause_intent('i_unk.json', state='ungraded')),
        # _guard_congestion
        ('no before-render to compare against', with_before + good_render
         + graded_intent),
        ('congestion waived with no reason', with_before + good_render
         + graded_intent + ['--waive', 'congestion:']),
        ('collateral waived with no reason', with_before + good_render
         + graded_intent + ['--waive', 'collateral:']),
        ('a before-render carrying no metrics', with_before + good_render
         + graded_intent + ['--congestion-before',
                            render(name='cb_bare.json', of=before,
                                   metrics=_DROP)]),
        ('a before-render that is not there', with_before + good_render
         + graded_intent + ['--congestion-before', missing]),
        ('a close-out render carrying no metrics', with_before + graded_intent
         + ['--render-json', render(name='r_nometrics.json', metrics=_DROP,
                                    checklist={
                                        'a_off_outline': {'pad_copper': [],
                                                          'courtyard': []},
                                        'd_moved': {'moved': 3,
                                                    'expected': None,
                                                    'match': None}}),
            '--congestion-before', render(name='cb_ok.json', of=before)]),
        # legality closed most of its gap, wirelength closed almost none
        ('a repair the numbers cannot settle', with_before + graded_intent
         + ['--render-json', render(name='r_disp.json', halo=50.0,
                                    crossings=60.0, hpwl=999.0),
            '--congestion-before', render(name='cb_disp.json', of=before,
                                          halo=100.0, crossings=100.0,
                                          hpwl=1000.0)]),
        # far more of the board moved than anything complained about
        ('a repair that moved far more than was complained about',
         with_before + graded_intent
         + ['--render-json', render(name='r_dist.json', halo=10.0,
                                    crossings=10.0, hpwl=100.0, moved=10),
            '--congestion-before', render(name='cb_dist.json', of=before,
                                          halo=100.0, crossings=100.0,
                                          hpwl=1000.0),
            '--damage-json', wrote('damage.json', {
                'blocking_pairs': [{'a': 'R1', 'b': 'R2'}]})]),
    ]


#: Sentinel for "delete this key from the fixture", so a scenario can build a
#: render that is missing a block rather than one carrying an empty one -- the
#: two are different documents and two different guards.
_DROP = object()


def _dump_refusals():
    """Every refusal this driver can print, with its guards UNSATISFIED.

    `--dump-all` fabricates PASSING evidence for every guard, deliberately: its
    job is to show the instructions. The cost is that no refusal is ever
    rendered, so the commands inside them -- the ones a STUCK reader runs next
    -- were the least-checked strings in the file. One spelled a flag
    `place_optimize.py` does not have and printed a command that exits 2, thirty
    lines below the branch that spells it correctly, and the gate that exists to
    catch exactly that (`tests/test_431_skill_commands.py`) reads this driver
    through `--dump-all` and so could not see it (#923).

    Coverage is MEASURED, not asserted: every literal a refusal can print is
    looked for IN THE DUMP, and any that never appears is named here and makes
    this exit 1. A refusal added without a scenario is a failure, not a gap --
    and so is one arm of a refusal that has four.
    """
    import tempfile
    sites = _refusal_sites()
    seen, crashed, produced = {}, [], []

    with tempfile.TemporaryDirectory() as tmp:
        scenarios = _refusal_scenarios(tmp)
        for label, argv in scenarios:
            a = _args(argv)
            for key in sorted(STAGES):
                try:
                    out = STAGES[key](a)
                except Exception as exc:                    # noqa: BLE001
                    crashed.append((key, label,
                                    f'{type(exc).__name__}: {exc}'))
                    continue
                # EVERY body feeds the coverage check, and only the
                # refusals are printed. `_delegation` returns
                # `(False, '--no-delegate was passed...')`, which is a REPORT
                # inside a stage body rather than a refusal -- mechanically
                # indistinguishable from a guard's `(False, text)` without
                # tracing where the text flows, so the honest question is "is
                # this text ever produced", not "is it produced inside
                # `<error>`".
                produced.append(out)
                if out.startswith('<error>') and out not in seen:
                    seen[out] = (key, label)

    rendered = []
    for out, (key, label) in seen.items():
        print(f'===== {key} refuses: {label} =====')
        print(out)
        rendered.append(out)
    dump = '\n'.join(produced)

    missed = []
    for (line, _col), (fn, kind, chunks) in sorted(sites.items()):
        gone = [c for c in chunks if c not in dump]
        if gone:
            missed.append((line, fn, kind, len(chunks), gone))
    total_chunks = sum(len(v[2]) for v in sites.values())
    print(f'\n{len(seen)} distinct refusal(s) from {len(scenarios)} '
          f'scenario(s); {len(sites) - len(missed)} of {len(sites)} refusal '
          f'text(s) fully rendered, over {total_chunks} literal chunk(s); '
          f'{_passthrough_count()} pass-through(s) print a text composed '
          f'elsewhere and are checked there.')
    for line, fn, kind, total, gone in missed:
        print(f'!! line {line} ({fn}, {kind}): {len(gone)} of {total} chunk(s) '
              f'no scenario renders')
        for chunk in gone[:2]:
            print(f'     {chunk.strip()[:100]!r}')
    for key, label, why in crashed:
        print(f'!! {key} raised instead of refusing on {label!r}: {why}')
    if missed or crashed:
        print('\nAdd a row to _refusal_scenarios, or delete the dead branch: '
              'a refusal nothing renders is a command nothing checks. (A tool '
              'a stage shells out to being absent looks the same from here -- '
              'check that first if several unrelated texts went missing.)')
        return 1
    return 0


def _self_test():
    """Every stage emits; every guard refuses without its evidence."""
    import contextlib
    import io
    import re
    import tempfile
    bad = []

    def want(cond, label):
        print(f'  {"PASS" if cond else "FAIL"}  {label}')
        if not cond:
            bad.append(label)

    for key in sorted(STAGES):
        out = STAGES[key](_args(['--board', 'b.kicad_pcb', '--before',
                                 'a.kicad_pcb']))
        want(out.startswith(('<stage_instructions', '<error>')),
             f'{key} emits a tagged block')
        # Every stage either hands the reader onward or is the terminal one.
        # A stage that ends without saying where to go is where an executor
        # starts improvising, which is the failure this driver exists to stop.
        want(key == 'P-close' or 'Next:' in out or out.startswith('<error>'),
             f'{key} says what comes next')
        # Instructions must fit in a reading, not a scroll. SAY WHICH ARM THIS
        # MEASURED: with only --board and --before, five of the nine stages
        # refuse, so this line was reporting a 3-line refusal as "under 80
        # lines" for the stages whose bodies are the longest in the file.
        _arm = 'refusal' if out.startswith('<error>') else 'body'
        want(len(out.splitlines()) <= 80,
             f'{key} stays under 80 lines ({_arm}, {len(out.splitlines())})')
    # --list is the index the refusals send a stuck reader to, so it is read
    # back from the PRINTER rather than re-derived from STAGES -- re-deriving
    # would pass on a --list that prints nothing at all.
    _buf = io.StringIO()
    with contextlib.redirect_stdout(_buf):
        main(['--list'])
    _listed = {ln.split()[0] for ln in _buf.getvalue().splitlines() if ln.strip()}
    want(_listed == set(STAGES),
         f'--list names every stage ({sorted(set(STAGES) - _listed)} missing, '
         f'{sorted(_listed - set(STAGES))} invented)')

    # EVERY `Next:` LINE REACHES ITS STAGE.
    #
    # SIX of them named a stage without the flags that stage hard-requires --
    # P0->P4, P1->P4, P1->P6, P4->P6, P5->P4, P6->P4; P4 and P6 both refuse
    # without --render-json, and P4 without --before too -- so a reader
    # following the handoff exactly as printed got exit 4 and a refusal instead
    # of the next step. P3's Next: line carried its flags all along, which is
    # what made this an oversight rather than a policy.
    #
    # This checks the FLAG SET, not the placeholder paths: each flag on the
    # printed command is re-pointed at fabricated evidence and the named stage
    # is called with exactly that. A stage that then refuses is refusing for a
    # flag the Next: line did not name.
    with tempfile.TemporaryDirectory() as _tmp:
        _fix = _next_line_fixture(_tmp)
        _bodies = {k: _render_for_next(k, _fix) for k in sorted(STAGES)}

        # The `of=` count is the model's own sense of how far along it is, and
        # it is TEXT -- so it is derived from the registry and checked against
        # it here. Eight stages used to say of="7" and P-brief of="8", over a
        # registry of nine.
        #
        # MEASURED ON THE BODY. The first version of this arm ran on the cheap
        # `--board/--before` render, where five of the nine stages refuse --
        # and a refusal carries no `of=` tag, so `_m is None` passed it
        # unconditionally for exactly those five. Hardcoding P4 back to of="7"
        # printed PASS while `--dump-all` showed of="7" to a reader. The arm
        # right above it had already been corrected for the same mistake.
        for _k, _body in _bodies.items():
            _m = re.search(r'\bof="(\d+)">', _body)
            want(_m is not None and int(_m.group(1)) == len(STAGES),
                 f'{_k} counts the stages the registry has '
                 f'({_m.group(1) if _m else "no of= tag in its body"})')

        # THE BODY LENGTHS, out loud -- AND HELD (#937). The cap above
        # measures whichever arm the CHEAP fixture produces, so for a stage
        # that refuses there it has never seen the instructions at all: P4's
        # 98 lines passed that assertion as a 3-line refusal, and P3 at 79 sits
        # in the same blind spot one line under the line.
        #
        # Reporting alone was the previous answer and it is not enough: a
        # number nobody can exceed is a number nobody has to argue with, and
        # over this PR P0 grew by 10 with nothing to notice. So every stage is
        # now held at a MEASURED ceiling rather than at one shared number.
        #
        # A ceiling, not a target: it exists to stop silent growth, and moving
        # one is a deliberate edit here with a reason. P4 is over the 80-line
        # norm and is pinned at what it is, because trimming it is an
        # editorial job -- and the structural finding is that its 98 lines are
        # FIVE VERBS (measure, act, prove, record, loop), so the fix is a split
        # rather than a trim.
        _over = {k: len(v.splitlines()) for k, v in _bodies.items()}
        print('  NOTE  stage body lines: '
              + ', '.join(f'{k} {v}' for k, v in _over.items())
              + f" -- over the 80-line norm: "
              + (', '.join(f'{k} ({v})' for k, v in _over.items() if v > 80)
                 or 'none'))
        for _k, _n in sorted(_over.items()):
            _ceil = _BODY_CEILING.get(_k)
            want(_ceil is not None,
                 f'{_k} declares a body ceiling (a new stage must)')
            if _ceil is not None:
                want(_n <= _ceil,
                     f'{_k} body is {_n} line(s), ceiling {_ceil}')

        _checked = 0
        for key, _body in _bodies.items():
            for _cmd in _next_commands(_body):
                _target, _flags = _cmd
                for _t in _target:
                    _argv = ['--stage', _t]
                    for _f in _flags:
                        _argv += [_f, _fix['flags'].get(_f, _fix['board'])]
                    _out = STAGES[_t](_args(_argv)) if _t in STAGES else '<error>'
                    _checked += 1
                    want(not _out.startswith('<error>'),
                         f"{key}'s Next: reaches {_t} "
                         f"({' '.join(_flags) or 'no flags'})"
                         + ('' if not _out.startswith('<error>') else
                            ' -- ' + ' '.join(_out.splitlines()[1:2])))
        # ...AND THE FILE IT NAMES IS ONE SOME STAGE WROTE.
        #
        # Reaching the stage is not enough. Two handoffs named `wk/render0.json`
        # and `wk/render_seed.json`, which no stage body produces, so a reader
        # following them literally still got exit 4 -- and the arm above cannot
        # see it, because it re-points every flag at fabricated evidence and so
        # tests the flag SET rather than the recipe. P3 and P4 render their own
        # (`wk/render_p3.json`, `wk/render_lapN.json`) and hand those on, which
        # is the shape that works. Anything not produced here is written as a
        # `<placeholder>` instead, the way P5 and P6 do it.
        # PRODUCED = mentioned outside the Next: blocks, i.e. the body told the
        # reader how to get it. Deliberately not a list of output flags: an
        # exempted name is where a guard fails, and no plausible list would
        # have carried `--suggest-locks-json`, which is how wk/locks.json is
        # written.
        _wanted, _written = set(), set()
        for _body in _bodies.values():
            _blocks = re.findall(r'(?m)^\s*Next\b.*?(?=^\s*Next\b|\Z)',
                                 _body, re.S)
            for _b in _blocks:
                _wanted |= set(re.findall(r'--[\w-]+[=\s]+(wk/[\w./-]+)', _b))
            _rest = _body
            for _b in _blocks:
                _rest = _rest.replace(_b, '')
            _written |= set(re.findall(r'(wk/[\w./-]+)', _rest))
        _orphan = sorted(_wanted - _written)
        want(not _orphan,
             f'every wk/ file a Next: line names is written by some stage '
             f'({", ".join(_orphan) or "none orphaned"})')

        # Vacuity: a parser that stops finding Next: commands would pass every
        # arm above by checking nothing.
        # Pinned near the measured 13, not at a token value: this arm has
        # twice stopped seeing a handoff while printing PASS for the others
        # (a block parser that stopped at the first line, and a label reworded
        # from `Next:` to `Next,`). A floor is what turns that into a failure.
        want(_checked >= 13, f'{_checked} Next: handoff(s) checked')

    # Guards refuse without evidence.
    want(STAGES['P3'](_args(['--board', 'b'])).startswith('<error>'),
         'P3 refuses without the copper-free DRC result')
    want(STAGES['P4'](_args(['--board', 'b'])).startswith('<error>'),
         'P4 refuses without --before (its gates are deltas)')

    with tempfile.TemporaryDirectory() as tmp:
        clean = os.path.join(tmp, 'clean.json')
        json.dump({'violations': 0}, open(clean, 'w', encoding='utf-8'))
        out = STAGES['P3'](_args(['--board', 'b', '--drc-json', clean]))
        want(out.startswith('<error>') and 'no damage' in out,
             'P3 refuses on a board with nothing to repair')

        dmg = os.path.join(tmp, 'dmg.json')
        json.dump({'violations': 12}, open(dmg, 'w', encoding='utf-8'))
        locks = os.path.join(tmp, 'locks.json')
        json.dump({'JSON_SUMMARY': {'unlocked_high': 3}},
                  open(locks, 'w', encoding='utf-8'))
        out = STAGES['P3'](_args(['--board', 'b', '--drc-json', dmg,
                                  '--locks-json', locks]))
        want(out.startswith('<error>') and 'unlocked_high' in out,
             'P3 refuses while load-bearing parts are unlocked')
        # This used to pass ONE waiver against unlocked_high = 3 and assert the
        # stage proceeded -- so the test's own label ("each finding") described
        # a rule the code did not implement and the test did not check.
        out = STAGES['P3'](_args(['--board', 'b', '--drc-json', dmg,
                                  '--locks-json', locks, '--waive', 'U1:checked']))
        want(out.startswith('<error>') and '1 of 3' in out,
             'P3 refuses when the waivers do not cover every finding')
        out = STAGES['P3'](_args(['--board', 'b', '--drc-json', dmg,
                                  '--locks-json', locks,
                                  '--waive', 'U1:checked', '--waive', 'U2:edge',
                                  '--waive', 'U3:standoff']))
        want(out.startswith('<stage_instructions'),
             'P3 proceeds once each finding is waived in writing')
        out = STAGES['P3'](_args(['--board', 'b', '--drc-json', dmg,
                                  '--locks-json', locks, '--waive', 'U1',
                                  '--waive', 'U2:e', '--waive', 'U3:s']))
        want(out.startswith('<error>') and 'do not parse' in out,
             'P3 refuses a waiver with no reason (REF, not REF:reason)')

    # P1 seeds FROM A PLAN (run 26). The guard names what the plan leaves out,
    # and only that: a must_lock part and a declared edge connector are not
    # the seed's to arrange and must not be demanded.
    with tempfile.TemporaryDirectory() as tmp1:
        _tb = _tiny_board(os.path.join(tmp1, 'tiny.kicad_pcb'),
                          ('U1', 'U2', 'H1', 'J1'))

        def _zp(name, blocks, **extra):
            p = os.path.join(tmp1, name)
            json.dump(_zone_plan_doc(blocks, **extra),
                      open(p, 'w', encoding='utf-8'))
            return p

        out = STAGES['P1'](_args(['--board', _tb]))
        want(out.startswith('<error>') and '--emit-intent' in out,
             'P1 refuses without a zone plan, and says how to start one')
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _zp(
            'half.json', [{'name': 'a', 'refs': ['U1'], 'zone': [0, 0, 5, 5],
                           'note': 'U1 only'}])]))
        want(out.startswith('<error>') and '3 movable footprint(s)' in out
             and 'H1, J1, U2' in out,
             'P1 names every movable footprint the plan leaves out')
        # A declared edge connector is exempt from ZONING only when it CLAIMS
        # an edge (`edge_claims()`, what the seeder's edge stage seats); a
        # connector_affinity entry is seeded at its centroid like any part.
        # It is exempt from being DECIDED never: run 27.
        _plan_ec = _zp(
            'lock.json', [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
                           'note': 'the two ICs'}],
            must_lock=['H*'], edge_connectors=[{'ref': 'J1', 'edge': 'west'}])
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _plan_ec]))
        want(out.startswith('<error>')
             and 'carry no `(locked yes)` in the board' in out
             and 'J1' in out and 'place_pose.py' in out,
             'P1 refuses to let the seeder choose a declared connector\'s pose')
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _plan_ec,
                                  '--waive', 'seed-connectors:']))
        want(out.startswith('<error>') and 'needs a REASON' in out,
             '...and the waiver that hands it over needs a reason')
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _plan_ec,
                                  '--waive',
                                  'seed-connectors:the west edge is clear']))
        want(out.startswith('<stage_instructions')
             and '1 zoned block(s) cover the other 2 movable' in out
             and '1 of them the seeder' in out
             and 'all 4 footprint(s) accounted for' in out,
             'P1 proceeds once the hand-over is on the record, and says how '
             'many connectors the seeder is choosing')
        # `must_lock` DOES NOT SATISFY THIS, and the review is why the arm
        # says so: the seeder stamps must_lock into its OUTPUT after seating
        # the part, so a must_lock connector with no file lock is seated at
        # the band midpoint at its incoming angle, byte for byte as if nothing
        # had been declared (measured on the seeder directly). Accepting it
        # would have let the plan through on the route SKILL.md recommends.
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _zp(
            'locked_ec.json',
            [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
              'note': 'the two ICs'}],
            must_lock=['H*', 'J1'],
            edge_connectors=[{'ref': 'J1', 'edge': 'west'}])]))
        want(out.startswith('<error>') and 'must_lock` does NOT do this' in out,
             'P1 refuses a must_lock connector and says why that is not a pin')
        # The FILE lock is what the refusal tells the reader to write
        # (`place_pose lock`) and what an author who placed it by hand has.
        # The intent need not mention it at all.
        _tb3 = _tiny_board(os.path.join(tmp1, 'lockfile.kicad_pcb'),
                           ('U1', 'U2', 'H1', 'J1'), locked=('H1', 'J1'))
        out = STAGES['P1'](_args(['--board', _tb3, '--zone-plan', _zp(
            'filelock_ec.json',
            [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
              'note': 'the two ICs'}],
            edge_connectors=[{'ref': 'J1', 'edge': 'west'}])]))
        want(out.startswith('<stage_instructions')
             and '2 pinned, the only kind' in out,
             'a connector locked IN THE FILE needs no intent clause and no '
             'waiver, and the census counts the pins apart')
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _zp(
            'aff.json', [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
                          'note': 'the two ICs'}],
            must_lock=['H*'],
            edge_connectors=[{'ref': 'J1', 'class': 'connector_affinity'}])]))
        want(out.startswith('<error>') and '1 movable footprint(s)' in out
             and ': J1.' in out,
             'P1 does not exempt a connector_affinity entry -- it claims no '
             'edge and the seeder seats it at its centroid')
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _zp(
            'nozone.json', [{'name': 'a', 'refs': ['U*']}])]))
        want(out.startswith('<error>') and 'no `blocks[].zone`' in out,
             'P1 refuses a plan whose blocks carry no zone')
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _zp(
            'nonote.json', [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5]}],
            must_lock=['H*'], edge_connectors=[{'ref': 'J1', 'edge': 'west'}])]))
        want(out.startswith('<error>') and 'carry no `note`' in out
             and ': a.' in out,
             'P1 refuses a zoned block that states no reason')
        # A part with a pad and NO connected pin is still the seeder's to
        # move (place_seed sends every pad-bearing pile part to the centre),
        # so a fiducial or a mounting hole is demanded like any other part
        # until the plan locks it -- P2's table calls it a mechanical fact,
        # and a fact goes in `must_lock`, not in a zone.
        _tb2 = _tiny_board(os.path.join(tmp1, 'fid.kicad_pcb'),
                           ('U1', 'U2', 'FID1'), unconnected=('FID1',))
        out = STAGES['P1'](_args(['--board', _tb2, '--zone-plan', _zp(
            'fid.json', [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
                          'note': 'the two ICs'}])]))
        want(out.startswith('<error>') and '1 movable footprint(s)' in out
             and ': FID1.' in out,
             'P1 demands a part with no connected pin too -- the seeder moves it')
        out = STAGES['P1'](_args(['--board', _tb2, '--zone-plan', _zp(
            'fid_lock.json', [{'name': 'a', 'refs': ['U*'], 'zone': [0, 0, 5, 5],
                               'note': 'the two ICs'}], must_lock=['FID*'])]))
        want(out.startswith('<stage_instructions'),
             '...and is satisfied once must_lock names it')
        # Membership comes from resolve_blocks, the seeder's own rule: a
        # block naming a GROUP this board does not have covers nothing, so
        # every part is named -- a re-typed fnmatch has no group to miss.
        out = STAGES['P1'](_args(['--board', _tb, '--zone-plan', _zp(
            'group.json', [{'name': 'a', 'group': 'nosuch',
                            'zone': [0, 0, 5, 5], 'note': 'by group'}],
            must_lock=['H*'], edge_connectors=[{'ref': 'J1', 'edge': 'west'}])]))
        want(out.startswith('<error>') and '2 movable footprint(s)' in out
             and 'U1, U2' in out,
             'P1 resolves a group block the way the seeder does, and an '
             'unknown group covers nobody')

    # P-close's congestion gate. The numbers are neo6502's own (run 15): a
    # placement that closed 49% of its halo gap and 2% of its crossings gap
    # closed out CLEAN, and the routing half then failed on 29 nets no router
    # parameter could reach. Each case below is a way that could go wrong.
    with tempfile.TemporaryDirectory() as tmp3:
        _pb = os.path.join(tmp3, 'b.kicad_pcb')
        _pa = os.path.join(tmp3, 'a.kicad_pcb')
        for _p in (_pb, _pa):
            open(_p, 'w', encoding='utf-8').close()

        def _wr(name, doc):
            p = os.path.join(tmp3, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        _int = _wr('i.json', {'rules_run': ['envelope'], 'violations': [],
                              'brief_coverage': {'brief': None, 'clauses': [], 'graded': 0,
                   'uncovered': 0, 'abstained': 0, 'complete': True}})

        def _close(after, before=None, extra=()):
            argv = ['--board', _pb, '--before', _pa,
                    '--render-json', _wr('ra.json', after),
                    '--intent-json', _int]
            if before is not None:
                argv += ['--congestion-before', _wr('rb.json', before)]
            return STAGES['P-close'](_args(argv + list(extra)))

        _dmg = _fake_render(_pa, halo=869.1, crossings=1871.0, hpwl=4193.6)
        _r15 = _fake_render(_pb, halo=440.1, crossings=1827.0, hpwl=4062.2)

        # Run 15's arm is REPORTED and NOT refused. The threshold that used to
        # refuse here was withdrawn on measurement: at 0.25 it also refused
        # neo6502's successful repair (blocking 18 -> 0), and on piantor -- where
        # `swap` LOWERS hpwl, because swapping parts on a regular matrix
        # shortens nets -- it would have refused a PERFECT repair, one that
        # restored the pristine board exactly.
        # See docs/placement-calibration.md.
        out = _close(_r15, _dmg)
        # It binds on a DISPOSITION, not on the numbers: run 15's shape must not
        # close out unacknowledged, but the driver never says the placement is
        # wrong -- it cannot, since a perfect repair scores like this on piantor.
        want(out.startswith('<error>') and 'DISPOSITION REQUIRED' in out,
             'P-close refuses run 15\'s shape until somebody dispositions it')
        want('ROUTABILITY, measured against' in out and 'hpwl' in out,
             'the refusal SHOWS the read it is asking about')
        want('not judged' in out and 'PERFECT repair' in out,
             'the refusal says the numbers are not a verdict on the placement')
        want(not _close(_r15, _dmg,
                        ('--waive', 'congestion:every lever spent')
                        ).startswith('<error>'),
             'a written disposition closes it out')
        # crossings is printed and never tested -- non-negotiable 4,
        # r(crossings) = +0.780 against distance-to-truth, not against routed
        # blocking (docs/placement-predictors.md).
        want('crossings' in out, 'P-close prints crossings alongside')
        # A proportionate repair gets the numbers and none of the warning.
        out = _close(_fake_render(_pb, halo=400.0, crossings=1200.0, hpwl=700.0),
                     _fake_render(_pa, halo=800.0, crossings=1800.0, hpwl=1000.0))
        want('ROUTABILITY, measured against' in out
             and 'legality closed a large share' not in out,
             'P-close reports without the warning when hpwl moved too')
        out = _close(_r15)
        want(out.startswith('<error>') and '--congestion-before' in out,
             'P-close refuses with no congestion comparison at all')
        # Proportionate repair: both gaps close together. Must NOT trip.
        out = _close(_fake_render(_pb, halo=400.0, crossings=1200.0,
                                  hpwl=700.0),
                     _fake_render(_pa, halo=800.0, crossings=1800.0,
                                  hpwl=1000.0))
        want(out.startswith('<stage_instructions'),
             'P-close passes a repair that moved congestion too')
        # Barely-damaged board: a small legality gain has nothing to be
        # disproportionate to, so the ratio must not be applied at all.
        out = _close(_fake_render(_pb, halo=95.0, crossings=100.0),
                     _fake_render(_pa, halo=100.0, crossings=100.0))
        want(out.startswith('<stage_instructions'),
             'P-close does not apply the ratio when legality barely moved')
        out = _close(_r15, _dmg, ('--waive', 'congestion:U2 locked, measured'))
        want(out.startswith('<stage_instructions'),
             'P-close accepts a congestion waiver WITH a reason')
        out = _close(_r15, _dmg, ('--waive', 'congestion:'))
        want(out.startswith('<error>') and 'needs a REASON' in out,
             'P-close refuses a congestion waiver with no reason')
        # A WAIVED gate must still SHOW its read. Run 16 waived this one and the
        # close-out came out with an empty routability block -- no numbers, no
        # reason, no trace the gate had fired. The disposition existed and the
        # tool discarded it, which is how the arrangement question vanished from
        # a run that had been asked it directly.
        out = _close(_r15, _dmg, ('--waive', 'congestion:U2 locked, measured'))
        want('ROUTABILITY, measured against' in out and 'hpwl' in out,
             'a WAIVED congestion read still prints its numbers')
        want('U2 locked, measured' in out,
             '...and the waiver reason is echoed onto the record, not dropped')

        # DISTURBANCE. Run 16's real numbers: 76 parts moved on a board where
        # 24 refs were named by any violation. It closed out clean.
        _dmgj = _wr('dmg.json', {
            'blocking_pairs': [{'a': f'C{i}', 'b': 'Q1'} for i in range(12)]})
        _moved76 = _fake_render(_pb, halo=440.1, crossings=1827.0, hpwl=4062.2,
                                moved=76)
        _prop = _fake_render(_pb, halo=440.1, crossings=1827.0, hpwl=4062.2,
                             moved=9)
        out = _close(_moved76, _dmg, ('--damage-json', _dmgj,
                                      '--waive', 'congestion:spent'))
        want(out.startswith('<error>') and 'DISTURBANCE' in out,
             'P-close refuses a repair that moved far more than was complained about')
        want('76 part(s) moved' in out and '13 ref(s) named' in out,
             '...and the refusal SHOWS both counts')
        want('not automatically wrong' in out,
             '...and disclaims being a verdict: legal damage must move back too')
        out = _close(_moved76, _dmg, ('--damage-json', _dmgj,
                                      '--waive', 'congestion:spent',
                                      '--waive', 'collateral:reseated the swap lattice'))
        want(out.startswith('<stage_instructions')
             and 'reseated the swap lattice' in out,
             'a written disturbance disposition closes it out, and is echoed')
        out = _close(_moved76, _dmg, ('--damage-json', _dmgj,
                                      '--waive', 'congestion:spent',
                                      '--waive', 'collateral:'))
        want(out.startswith('<error>') and 'needs a REASON' in out,
             'P-close refuses a disturbance waiver with no reason')
        out = _close(_prop, _dmg, ('--damage-json', _dmgj,
                                   '--waive', 'congestion:spent'))
        want(out.startswith('<stage_instructions'),
             'P-close passes a repair proportionate to what was complained about')
        # No --damage-json: the count still prints, and NOTHING refuses. A read
        # that degrades into a refusal when its optional evidence is absent is a
        # gate nobody can run.
        out = _close(_moved76, _dmg, ('--waive', 'congestion:spent'))
        want(out.startswith('<stage_instructions') and '76 part(s) moved' in out,
             'without --damage-json the move count prints and does not refuse')
        # The intent gate used to accept any string as a path.
        out = STAGES['P-close'](_args(
            ['--board', _pb, '--before', _pa,
             '--render-json', _wr('ra2.json', _r15),
             '--intent-json', os.path.join(tmp3, 'nope.json'),
             '--congestion-before', _wr('rb2.json', _dmg)]))
        want(out.startswith('<error>') and 'intent' in out.lower(),
             'P-close opens --intent-json instead of trusting the argument')
        out = STAGES['P-close'](_args(
            ['--board', _pb, '--before', _pa,
             '--render-json', _wr('ra3.json', _r15),
             '--intent-json', _wr('i0.json', {'rules_run': 0}),
             '--congestion-before', _wr('rb3.json', _dmg)]))
        want(out.startswith('<error>') and 'constrained nothing' in out,
             'P-close refuses an intent that graded 0 rules')

        # #902. THE SHAPE PRODUCTION ACTUALLY WRITES. The case above feeds an
        # INT, which `check_floorplan --json` never produces -- it writes a
        # LIST -- so the old `isinstance(_ran, int)` gate passed this test and
        # never fired once in production.
        out = STAGES['P-close'](_args(
            ['--board', _pb, '--before', _pa,
             '--render-json', _wr('ra4.json', _r15),
             '--intent-json', _wr('i1.json', {'rules_run': []}),
             '--congestion-before', _wr('rb4.json', _dmg)]))
        want(out.startswith('<error>') and 'constrained nothing' in out,
             'P-close refuses rules_run: [] -- the shape the JSON file writes')
        for _shape in ({'rules_run': 'envelope'}, {}):
            out = STAGES['P-close'](_args(
                ['--board', _pb, '--before', _pa,
                 '--render-json', _wr('ra5.json', _r15),
                 '--intent-json', _wr('i2.json', _shape),
                 '--congestion-before', _wr('rb5.json', _dmg)]))
            want(out.startswith('<error>') and 'cannot read' in out,
                 f'P-close refuses a rules_run this gate cannot read '
                 f'({_shape or "absent"})')

        _six = ['envelope', 'zone_containment', 'zone_side', 'assembly_side',
                'keepout', 'legality']
        out = STAGES['P-close'](_args(
            ['--board', _pb, '--before', _pa,
             '--render-json', _wr('ra6.json', _r15),
             '--intent-json', _wr('i3.json', {'rules_run': _six}),
             '--congestion-before', _wr('rb6.json', _dmg)]))
        want(out.startswith('<error>') and 'brief_coverage' in out
             and 'require-brief-coverage' in out,
             'P-close refuses a graded intent that carries no coverage block')

        def _covered(rows, graded=0, brief='b.json'):
            # `brief` is what the block says was FOUND, and it is a separate
            # fact from whether any clause compiled: a brief declaring only
            # free-text unknowns names itself here and carries no clause.
            return {'rules_run': _six,
                    'brief_coverage': {'brief': brief, 'clauses': rows,
                                       'graded': graded}}

        _open_rows = [
            {'id': 'proximity[0:Y1~U1].max_mm', 'state': 'uncovered',
             'drifted': False, 'why': 'the intent carries no proximity claim'},
            {'id': 'keepouts[batt]', 'state': 'uncovered', 'drifted': False,
             'why': 'the intent carries no keepout named batt'},
            {'id': 'interfaces[J1].along_edge', 'state': 'abstained',
             'drifted': False, 'why': 'this board has no usable bounds'},
        ]

        def _close_cov(rows, extra=(), graded=0, brief='b.json'):
            # The `_r15` / `_dmg` pair is run 15's shape, which this stage
            # REPORTS and refuses until somebody dispositions it -- so every
            # case that expects the stage to PROCEED carries that disposition
            # and is testing the coverage arm alone.
            return STAGES['P-close'](_args(
                ['--board', _pb, '--before', _pa,
                 '--render-json', _wr('rc.json', _r15),
                 '--intent-json', _wr('ic.json',
                                      _covered(rows, graded, brief)),
                 '--congestion-before', _wr('rd.json', _dmg),
                 '--waive', 'congestion:every lever spent'] + list(extra)))

        out = _close_cov(_open_rows)
        want(out.startswith('<error>')
             and all(r['id'] in out for r in _open_rows)
             and 'UNCOVERED' in out and 'ABSTAINED' in out
             and 'constrained nothing' not in out,
             'P-close names every uncovered and abstained clause, on the '
             'coverage arm rather than the rule count')

        _waivers = []
        for r in _open_rows:
            _waivers += ['--waive', f"brief-clause:{r['id']}:the board cannot "
                                    f"answer this"]
        out = _close_cov(_open_rows, extra=_waivers)
        want(not out.startswith('<error>') and 'waived by name' in out,
             'a per-clause waiver with a reason closes the gate')

        out = _close_cov(_open_rows,
                         extra=['--waive', 'brief-clause:proximity[0:Y1~U1]'
                                           '.max_mm:'])
        want(out.startswith('<error>') and 'needs a REASON' in out,
             'a clause waiver with no reason is refused')

        out = _close_cov(_open_rows,
                         extra=['--waive', 'brief-clause:nope[0]:x'])
        want(out.startswith('<error>') and 'does not carry' in out,
             'a waiver naming a clause this document lacks is refused')

        # The control that the gate is CLEARABLE at all: an author who wrote
        # "unknown", and a key this toolchain carries by design, must not
        # block -- punishing an honest unknown teaches people to stop
        # declaring.
        out = _close_cov([
            {'id': 'proximity[0:Y1~U1].max_mm', 'state': 'not_claimed',
             'drifted': False, 'why': 'the brief declares this "unknown"'},
            {'id': 'interfaces[J1].mount_mode', 'state': 'carried',
             'drifted': False, 'why': 'carried into context'}], graded=0)
        want(not out.startswith('<error>') and 'declared brief clause' in out,
             'unknown and carried clauses never block, and the read is printed')

        # A brief-less board closes, and SAYS SO: #711 made declaring nothing
        # cost nothing, and this must not reverse that.
        out = _close_cov([], brief=None)
        want(not out.startswith('<error>') and 'no design brief' in out,
             'a board with no brief closes with the absence on the record')
        # ...and a brief that WAS found but declares no gradable clause must
        # not be reported as absent. Inferring absence from an empty clause
        # list said "no design brief beside this board" about a board whose
        # brief was named in the same block.
        out = _close_cov([], brief='minimal.design-brief.json')
        want(not out.startswith('<error>')
             and 'minimal.design-brief.json' in out
             and 'no design brief beside' not in out,
             'a brief that declares no gradable clause is named, not called '
             'absent')

        # The four mutants that SURVIVED the first battery. Each names the
        # code it pins, because a case whose subject is not obvious is a case
        # somebody later deletes as redundant.

        # `True` is an `int` in Python, so without the explicit bool check a
        # `rules_run: true` reads as a count of ONE and passes every arm.
        for _b in (True, False):
            out = STAGES['P-close'](_args(
                ['--board', _pb, '--before', _pa,
                 '--render-json', _wr('rb1.json', _r15),
                 '--intent-json', _wr('ib1.json', {'rules_run': _b}),
                 '--congestion-before', _wr('rb2b.json', _dmg)]))
            want(out.startswith('<error>') and 'cannot read' in out,
                 f'P-close refuses rules_run: {_b} -- a bool is an int in '
                 f'Python and would read as a count')

        # LONGEST-MATCH resolution. The pair has to be COLON-AMBIGUOUS or the
        # two orders agree: with `keepouts[a]` and `keepouts[a]:b`, a waiver
        # for the longer one begins with the shorter one PLUS a colon, so
        # shortest-first waives the WRONG clause and reads `b:...` as the
        # reason. A first draft used `edge` / `edge_band`, where no colon
        # separates them, both orders resolved correctly, and the mutation
        # survived -- a case that cannot tell the two apart is not a case.
        _pair = [
            {'id': 'keepouts[a]', 'state': 'uncovered', 'drifted': False,
             'why': 'the short one'},
            {'id': 'keepouts[a]:b', 'state': 'uncovered', 'drifted': False,
             'why': 'the long one'},
        ]
        out = _close_cov(_pair, extra=[
            '--waive', 'brief-clause:keepouts[a]:b:cannot answer'])
        want(out.startswith('<error>') and 'the short one' in out
             and 'the long one' not in out,
             'a waiver resolves to the LONGEST matching id, leaving the '
             'shorter clause open')

        # A typo'd SUFFIX must not silently waive the real clause: the `+ ':'`
        # is what stops `keepouts[batt]xyz:reason` matching `keepouts[batt]`.
        out = _close_cov(
            [{'id': 'keepouts[batt]', 'state': 'uncovered', 'drifted': False,
              'why': 'x'}],
            extra=['--waive', 'brief-clause:keepouts[batt]xyz:typo'])
        want(out.startswith('<error>') and 'does not carry' in out
             and 'keepouts[batt]xyz' in out,
             'an id with a typo\'d suffix does not waive the real clause')

        # ...and the refusal names the WHOLE id the caller typed. This one
        # CONTAINS a colon, which is what makes it discriminate: the old
        # `split(':', 1)[0]` reported `proximity[0` and sent a reader looking
        # for a clause by that name. A phantom id with no colon in it cannot
        # tell the two spellings apart.
        out = _close_cov(
            [{'id': 'keepouts[batt]', 'state': 'uncovered', 'drifted': False,
              'why': 'x'}],
            extra=['--waive', 'brief-clause:proximity[0:Y1~U2].max_mm:typo'])
        want(out.startswith('<error>')
             and 'proximity[0:Y1~U2].max_mm' in out,
             'the phantom-id refusal names the whole id, not the fragment '
             'before its first colon')

        # THE DRIFTED ARM. Every other fixture here carries `drifted: False`,
        # so deleting the drift test left the self-test green while the
        # verifier prompt requires it.
        out = _close_cov([{'id': 'interfaces[USB1].edge', 'state': 'graded',
                           'drifted': True, 'why': ''}], graded=1)
        want(out.startswith('<error>') and 'DRIFTED' in out
             and 'interfaces[USB1].edge' in out,
             'a GRADED clause that drifted still refuses -- measured, against '
             'the wrong requirement')

        # A state this gate has never seen must FAIL CLOSED. The producer
        # raises on a novel state; the consumer used to pass silently.
        for _st in ('ungraded', 'UNCOVERED', None):
            out = _close_cov([{'id': 'keepouts[k]', 'state': _st,
                               'drifted': False, 'why': 'novel'}])
            want(out.startswith('<error>') and 'UNRECOGNISED' in out,
                 f'an unrecognised clause state ({_st!r}) refuses and is '
                 f'named, rather than passing as if it were graded')

        # #895's fifth render check. It is BACKWARD-COMPATIBLE by design -- a
        # document with no `review_sheet` key was produced by a run that never
        # asked for one -- so the two shapes that MEAN something are fed here
        # by hand.
        #
        # Until #963 that was the only way it could be reached AT ALL, and the
        # sentence that stood here said so: none of this driver's own templates
        # passed `--review-sheet`, so no render they produce ever carried the
        # key, so the absent-key arm swallowed every one of them. The guard was
        # live code that could not fire on its own population. The templates
        # pass the flag now, and
        # `tests/test_963_render_templates.py::
        # test_the_guard_can_fire_on_the_drivers_own_render` holds them to it.
        _sheet_file = _wr('sheet_exists.json', {'x': 1})
        for _val, _want in ((None, 'none was written'),
                            (os.path.join(tmp3, 'no_such_sheet.png'),
                             'not there')):
            _rj = dict(_r15)
            _rj['review_sheet'] = _val
            out = STAGES['P-close'](_args(
                ['--board', _pb, '--before', _pa,
                 '--render-json', _wr('rs1.json', _rj),
                 '--intent-json', _wr('is1.json', _covered([])),
                 '--congestion-before', _wr('rs2.json', _dmg),
                 '--waive', 'congestion:spent']))
            want(out.startswith('<error>') and _want in out,
                 f'a render whose review sheet is {_val!r} is refused')
        # PAD COPPER OFF THE OUTLINE -- the top-priority placement defect, and
        # the one this door did not check at all until #936. Held here rather
        # than only by --dump-refusals: blinding the read left the driver's own
        # self-test green, which a battery row measured as a SURVIVOR.
        _rj = dict(_r15)
        _rj['review_sheet'] = _sheet_file
        _rj['checklist'] = dict(_rj.get('checklist') or {})
        _rj['checklist']['a_off_outline'] = {
            'pad_copper': [{'reference': 'U7'}, {'reference': 'J2'}],
            'courtyard': []}
        out = STAGES['P-close'](_args(
            ['--board', _pb, '--before', _pa,
             '--render-json', _wr('rs_oob.json', _rj),
             '--intent-json', _wr('is_oob.json', _covered([], brief=None)),
             '--congestion-before', _wr('rs_oob2.json', _dmg),
             '--waive', 'congestion:spent']))
        want(out.startswith('<error>') and 'PAD COPPER outside' in out
             and 'U7' in out,
             'a render naming parts with pad copper off the outline is refused,'
             ' and the refusal names them')

        _rj = dict(_r15)
        _rj['review_sheet'] = _sheet_file
        out = STAGES['P-close'](_args(
            ['--board', _pb, '--before', _pa,
             '--render-json', _wr('rs3.json', _rj),
             '--intent-json', _wr('is3.json', _covered([], brief=None)),
             '--congestion-before', _wr('rs4.json', _dmg),
             '--waive', 'congestion:spent']))
        want(not out.startswith('<error>'),
             'a render naming a sheet that EXISTS passes the fifth check')

        # A malformed block REFUSES; it does not traceback. "Refused, never
        # assumed" is arm 2's contract, and a crash is neither.
        for _shape in ([], 'x', 0, {'clauses': 'x'}, {'clauses': ['x']},
                       {'schema': 2, 'clauses': []}):
            out = STAGES['P-close'](_args(
                ['--board', _pb, '--before', _pa,
                 '--render-json', _wr('rm1.json', _r15),
                 '--intent-json', _wr('im1.json',
                                      {'rules_run': _six,
                                       'brief_coverage': _shape}),
                 '--congestion-before', _wr('rm2.json', _dmg)]))
            want(out.startswith('<error>'),
                 f'a malformed brief_coverage ({_shape!r}) is refused, not a '
                 f'traceback')

    # Banned shapes: hedging, and a subagent prompt the model might obey itself.
    # The evidence must be REAL files: this used to pass bare names ('b', 'a',
    # 'd'), so every stage with an existence check dumped its refusal instead of
    # its body and the banned-shape scan ran over error text. It went unnoticed
    # because the only assertion that depended on a body -- the subagent-prompt
    # one -- happened to be satisfied by the one stage that had no such check.
    with tempfile.TemporaryDirectory() as tmp2:
        # The SAME fabrication --dump-all uses. The congestion pair it
        # builds must PASS the gate rather than dodge it: it carries a
        # real `metrics` block on both sides (halo closed 50% of its
        # gap, crossings 40% -- proportionate), so a change to the
        # gate's arithmetic is noticed here.
        _ev = _args(_fixture_argv(_next_line_fixture(tmp2))
                    + ['--waive', 'X:y'])
        bodies = {k: STAGES[k](_ev) for k in sorted(STAGES)}
        everything = '\n'.join(bodies.values())
    _refused = [k for k, v in bodies.items() if v.startswith('<error>')]
    want(not _refused,
         f'every stage emits its body under complete evidence '
         f'(refused: {", ".join(_refused) or "none"})')
    for phrase in ('you may want to', 'if you are not sure', 'consider running'):
        want(phrase not in everything.lower(), f'no hedging: {phrase!r}')
    want(everything.count('<subagent_prompt') >= 1,
         'at least one stage dispatches an independent verifier')

    print('OK' if not bad else f'FAIL: {len(bad)}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
