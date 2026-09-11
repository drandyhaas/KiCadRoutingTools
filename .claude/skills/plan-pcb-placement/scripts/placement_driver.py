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


def p0(a):
    """Decide whether to touch the placement at all."""
    return f'''<stage_instructions stage="P0" name="gate" of="{len(STAGES)}">
MEASURE this board's placement, then decide. Do not decide first.

The measurement is two commands and it is never optional. Skipping it is how a
board with two parts stacked on each other reaches routing -- every other
instrument in the chain looks at copper, and there is no copper yet.

Run BOTH, on the board with its copper removed. Neither alone is enough: the
first cannot see two parts stacked on the same net, the second is the channel
that can.

  python3 -X utf8 py_router/check_drc.py {a.board} --clearance <the board's own floor> --json wk/drc0.json
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

Next: python3 -X utf8 {sys.argv[0]} --stage <P1|P2|P5> --board {a.board} \\
          --drc-json wk/drc0.json --assembly-json wk/assembly0.json

Next, for P4 only: it grades DELTAS, so it also needs the pair. It refuses
without both -- an absolute threshold is what made two of its gates unusable.
  python3 -X utf8 {sys.argv[0]} --stage P4 --board {a.board} \\
      --drc-json wk/drc0.json --before <the board this one came from> \\
      --render-json <that pair's render>
</stage_instructions>'''


def p1(a):
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

Walk the ladder in order and say which rung applies:

1. The repo has its own seeder -> run it, then treat the output as a rough
   placement (P4/P5).
2. No seeder, but an intent exists or the spec states placement facts ->
   author the intent (P6), then seed from it:
       python3 -X utf8 py_placer/place_seed.py {a.board} seed.kicad_pcb --intent fp.json
   The seeder grades its own output against the same intent; exit 4 means the
   seed does not satisfy the intent it was built from, and says which rule broke.
3. Neither -> say so and STOP. This toolchain does not invent a placement, and
   inventing mechanical geometry is what every rule here forbids.

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

  python3 -X utf8 py_router/check_drc.py {a.board} --clearance <floor> --clearance-margin 0
  python3 -X utf8 py_tools/check_assembly.py {a.board} --baseline {a.before}
  python3 -X utf8 py_tools/check_channels.py {a.board} --baseline {a.before} --gate
  python3 -X utf8 check_rigid_consistency.py {a.before} {a.board}

check_assembly and check_channels now READ THE BOARD's own clearance (and
check_channels its track width too) and print each value with its source, so
do NOT pass <floor> to them -- omitting it is what gets the board's floor.
They used to default to a flat 0.25 / 0.3 regardless: on a 0.2 board that
track width invented a "U2 N short 1 lane" deficit that does not exist
(supply 14, demand 12), and it was handed forward as floorplan-shaped residue.
check_drc still wants it spelled out.

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
      --expect-moved <what the lever said it moved> \\
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
      --intent <the graded floorplan intent> --lock <the P2 locks>

PASS --intent AND --lock, or rule 1 below grades nothing. place_portfolio
learns the declared intent from --intent and the mechanical locks from --lock;
without them its HARD gate has no constraint to be hard about, and a step that
optimises against no constraint is the failure this whole procedure exists to
stop.

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
        # The id is resolved AGAINST THE DOCUMENT rather than by splitting on
        # colons: a clause id contains them (`proximity[0:Y1~U1].max_mm`) and
        # so may a reason, so any positional split cuts one of the two in half.
        # The document is the authority on what its own ids are.
        _waived, _phantom, _noreason = set(), [], []
        for _w in (a.waive or []):
            if not _w.startswith('brief-clause:'):
                continue
            _rest = _w[len('brief-clause:'):]
            # LONGEST first, and the `+ ':'` matters: without it
            # `keepouts[batt]xyz:reason` would silently waive `keepouts[batt]`,
            # and without longest-first a waiver for `keepouts[usb-shell]`
            # could resolve to `keepouts[usb]` and leave the real clause open
            # while reporting it waived. That pair is REAL -- keep-out names
            # are the author's, so one being a prefix of another is ordinary.
            # (`interfaces[J1].edge_band` stood here and is not a clause id any
            # producer emits: the interface ids are `.edge`, `.along_edge`,
            # `.user_facing`, `.overhang_mm`.)
            _hit = next((i for i in sorted(_known, key=len, reverse=True)
                         if _rest == i or _rest.startswith(i + ':')), None)
            if _hit is None:
                # The WHOLE residue, not `split(':', 1)[0]` -- that is the
                # very bug this resolution fixed, surviving one line over in
                # the message: a waiver for `proximity[0:Y1~U2].max_mm` was
                # reported as naming `proximity[0`.
                _phantom.append(_rest)
            elif not _rest[len(_hit) + 1:].strip():
                _noreason.append(_hit)
            else:
                _waived.add(_hit)
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
    return f'''<stage_instructions stage="P-close" name="close out" of="{len(STAGES)}">
Prove the placement, then hand it on.

  DECLARED SPEC: {_cov_read}

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


def _guard_damage(a):
    """P2/P3 exist to repair damage; refuse to run them blind, or on a clean board."""
    drc, derr = _load(a.drc_json, 'The copper-free DRC result (--drc-json)')
    if derr:
        return False, (derr + '\n\nP0 produces it:\n  python3 -X utf8 '
                              f'py_router/check_drc.py {a.board} --clearance <floor> '
                              '--json wk/drc0.json')
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
            '--json wk/drc0.json')
    if isinstance(count, int) and count == 0:
        asm, _ = _load(a.assembly_json, 'assembly')
        blocking = _dig(asm, 'blocking') if asm else None
        if not blocking:
            return False, (
                'The copper-free board reports 0 violations and no blocking '
                'assembly pair. There is no damage for this stage to repair, '
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
                   '      --json-out wk/render.json -o wk/render.png\n\n'
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
    _oob = (chk.get('a_off_outline') or {}).get('pad_copper')
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
            f'every clearance graze.\n\nMove those parts back inside the '
            f'outline and re-render. The outline is not yours to change.\n\n'
            f'This is the per-PAD measure, not check_assembly\'s '
            f'`oob_pad_count`, which is a part-level AABB inflated by the '
            f'clearance and reads non-zero on human boards whose pads are '
            f'fine.')
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
            '      --json-out wk/congestion_before.json -o wk/congestion_before.png\n'
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
                    help='the board this one was derived from (the delta gates '
                         'need it)')
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
    # --congestion-ratio is GONE. It set a threshold P-close refused on, and the
    # calibration withdrew that refusal (docs/placement-calibration.md): the
    # premise inverts on 1 of 3 corpus boards, where a perfect repair scores a
    # negative hpwl gain and the gate refused the correct answer. A flag that
    # no longer
    # reaches a decision is the same lie as render_placement's --metrics was --
    # it reads as a knob somebody thought about. The routability numbers are
    # REPORTED at P-close now, and 0.25 is baked nowhere.
    ap.add_argument('--waive', action='append', default=[], metavar='REF:reason')
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

    board = os.path.join(tmp, 'b.kicad_pcb')
    before = os.path.join(tmp, 'a.kicad_pcb')
    for p in (board, before):
        open(p, 'w', encoding='utf-8').close()
    return {
        'board': board,
        'flags': {
            '--board': board,
            '--before': before,
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
        ('a board with no damage to repair', base
         + ['--drc-json', wrote('clean.json', {'violations': 0})]),
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
        # ...and the same list carrying no `reference`, which is the arm that
        # falls back to naming the key. Without this row that fallback is a
        # branch nothing renders -- exactly what --dump-refusals exists to say.
        ('a render with off-outline pad copper it cannot attribute', with_before
         + ['--render-json', render(name='r_oob_anon.json', checklist={
             'a_off_outline': {'pad_copper': [{'amount_mm': 1.2}],
                               'courtyard': []},
             'd_moved': {'moved': 3, 'expected': None, 'match': None}})]),
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

        # THE BODY LENGTHS, out loud. The cap above measures whichever arm the
        # cheap fixture produces, so for a stage that refuses there it has
        # never seen the instructions at all. These are the real numbers; P4 is
        # over the 80-line norm today and trimming it is an editorial job, not
        # a fact fix, so this reports rather than refuses. Reported > silent:
        # a number nobody prints is a number nobody argues with.
        _over = {k: len(v.splitlines()) for k, v in _bodies.items()}
        print('  NOTE  stage body lines: '
              + ', '.join(f'{k} {v}' for k, v in _over.items())
              + f" -- over the 80-line norm: "
              + (', '.join(f'{k} ({v})' for k, v in _over.items() if v > 80)
                 or 'none'))

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
        # asked for one -- so it can only be seen by feeding the two shapes
        # that mean something, or it is an arm nothing exercises.
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
