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

def p0(a):
    """Decide whether to touch the placement at all."""
    return f'''<stage_instructions stage="P0" name="gate" of="7">
MEASURE this board's placement, then decide. Do not decide first.

The measurement is two commands and it is never optional. Skipping it is how a
board with two parts stacked on each other reaches routing -- every other
instrument in the chain looks at copper, and there is no copper yet.

Run BOTH, on the board with its copper removed. Neither alone is enough: the
first cannot see two parts stacked on the same net, the second is the channel
that can.

  python3 -X utf8 py_router/check_drc.py {a.board} --clearance <the board's own floor>
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

Next: python3 -X utf8 {sys.argv[0]} --stage <P1|P2|P4|P5> --board {a.board} \\
          --drc-json wk/drc0.json --assembly-json wk/assembly0.json
</stage_instructions>'''


def p1(a):
    return f'''<stage_instructions stage="P1" name="unplaced" of="7">
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

Next: --stage P4 (legalize the seed) or --stage P6 (declare the intent first).
</stage_instructions>'''


def p2(a):
    ok, why = _guard_damage(a)
    if not ok:
        return err(why)
    return f'''<stage_instructions stage="P2" name="mechanical facts" of="7">
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
        return err(lerr + '\n\nP2 produces it:\n  python3 -X utf8 '
                          f'py_placer/place_optimize.py {a.board} --suggest-locks '
                          '--json wk/locks.json')
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
    return f'''<stage_instructions stage="P3" name="reconstruct" of="7">
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
    Both take --dry-run and report what they WOULD do, and both take
    NO STEP HAS A WALL-CLOCK BUDGET -- `--deadline` was removed everywhere (no result may depend on timing), so passing it is an argparse error. A harness timeout SIGTERMs the tool, its shutdown never runs, and you get exit 143 with no partial board and no summary. 143 and 124 are the SHELL's codes, not a tool's. Run long steps DETACHED, and bound them by scope rather than by a clock.
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
    render_placement's checklist.a_off_outline.pad_copper channel.
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
          --before {a.board} --drc-json wk/drc1.json \\
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
    return f'''<stage_instructions stage="P4" name="fix loop" of="7">
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
  check_assembly     a blocking pair (two parts on the same copper -- not
                     buildable whatever the nets), or copper on a LOCKED part
  check_channels     a face that now carries demand with no lane left, and did
                     not before
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

Next: --stage P5 (a slate) or --stage P6 (declare and grade), then P-close.
</stage_instructions>'''


def p5(a):
    return f'''<stage_instructions stage="P5" name="options" of="7">
Use this when the question is "which arrangement", not "is this one legal".

  python3 -X utf8 py_placer/place_portfolio.py {a.board} --out-dir wk/slate \\
      --candidates <K> --keep <N> [--full-probe]

Rank rules, in this order:
  1. HARD gates first: legality and the declared intent. A candidate that fails
     either is not in the running, however good it looks.
  2. Prefer a ranking that ROUTED something over one that only measured the
     placement. A placement metric cannot see the thing you are choosing for.
  3. hpwl and crossings ANNOTATE the slate; they do not rank it. Both correlate
     positively with distance-to-truth on damaged boards -- that is the
     measured dependent variable, not routed blocking; nothing here has
     correlated either with blocking (docs/placement-predictors.md).

Adopt one deliberately, say why in writing, and re-run P4 on the adopted board.
Adoption is a decision, not a step -- it is not replayable, so it belongs in
the record.

Next: --stage P4 --board <adopted> --before {a.board}
</stage_instructions>'''


def p6(a):
    # Mandate 1: you cannot declare zones for a board you have not looked at.
    # An intent authored off coordinates alone is a guess with a schema.
    _ok, _why = _guard_render(a)
    if not _ok:
        return err(_why)
    return f'''<stage_instructions stage="P6" name="declare the intent" of="7">
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

Next: --stage P4 --board {a.board} --before <the board before any change>
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
    if a.intent_json:
        _idoc, _ierr = _load(a.intent_json, 'The floorplan intent (--intent-json)')
        if _ierr:
            return err(_ierr + '\n\nThis gate opens the file now; it used to '
                               'accept the ARGUMENT and never the document, so '
                               'a path to nothing satisfied it.')
        _ran = _idoc.get('rules_run')
        if isinstance(_ran, int) and _ran <= 0:
            return err(
                f'That intent graded {_ran} rules, so it constrained nothing.\n\n'
                f'One run shipped `rules_run: 0` and a second covered 0 of 266 '
                f'parts; nothing objected either time, because this gate only '
                f'checked that a path was PASSED. Edit the intent down to the '
                f'clauses this board must satisfy, then:\n'
                f'  python3 -X utf8 py_tools/check_floorplan.py {a.board} '
                f'--intent {a.intent_json} --require-rules 1 '
                f'--json wk/intent_result.json')
    # The routability read. It REFUSES only when the evidence is missing, never
    # on the numbers themselves -- see _guard_congestion and
    # wk/calibration/RESULT.md for why the threshold that used to live here was
    # withdrawn.
    _cok, _cwhy = _guard_congestion(a)
    if not _cok:
        return err(_cwhy)
    return f'''<stage_instructions stage="P-close" name="close out" of="7">
Prove the placement, then hand it on.

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
no inputs and could not fail. It MUST fail on `rules_run == 0`: a grade that ran
no rules is a vacuous pass, not a clean board.
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
    d = chk.get('d_moved') or {}
    if d.get('match') is False:
        return False, (
            f'The render disagrees with the move count you declared: it '
            f'measured {d.get("moved")} moved part(s), you passed '
            f'--expect-moved {d.get("expected")}.\n\nOne of the two is wrong, '
            f'and "more parts moved than the step claimed" is exactly what '
            f'mandate 8(d) exists to catch. Resolve it before continuing.')
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
    (wk/calibration/RESULT.md, three boards, three populations each):

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
            'cannot be, and wk/calibration/RESULT.md says why: on one corpus '
            'board a PERFECT repair',
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
    'P0': p0, 'P1': p1, 'P2': p2, 'P3': p3, 'P4': p4, 'P5': p5, 'P6': p6,
    'P-close': p_close,
}
TITLES = {
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
    # calibration withdrew that refusal (wk/calibration/RESULT.md): the premise
    # inverts on 1 of 3 corpus boards, where a perfect repair scores a negative
    # hpwl gain and the gate refused the correct answer. A flag that no longer
    # reaches a decision is the same lie as render_placement's --metrics was --
    # it reads as a knob somebody thought about. The routability numbers are
    # REPORTED at P-close now, and 0.25 is baked nowhere.
    ap.add_argument('--waive', action='append', default=[], metavar='REF:reason')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--dump-all', action='store_true')
    ap.add_argument('--self-test', action='store_true')
    return ap.parse_args(argv)


def main(argv=None):
    a = _args(argv)
    if a.list:
        for key in ('P0', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P-close'):
            print(f'  {key:8s} {TITLES[key]}')
        return 0
    if a.dump_all:
        return _dump_all()
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


def _dump_all():
    """Every stage's REAL body, guards satisfied.

    This used to pass filenames that do not exist, so P2, P3 and P4 dumped
    their REFUSALS -- three of eight stages, including the two that carry the
    most commands. Anything auditing the driver through --dump-all (a flag
    checker, a reviewer, a person) was reading error text and seeing no
    commands to be wrong. Guard evidence is cheap to fabricate HERE, where the
    point is to show the instructions rather than to act on them.
    """
    import json as _json
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        def wrote(name, doc):
            p = os.path.join(tmp, name)
            with open(p, 'w', encoding='utf-8') as fh:
                _json.dump(doc, fh)
            return p

        board = os.path.join(tmp, 'b.kicad_pcb')
        before = os.path.join(tmp, 'a.kicad_pcb')
        for p in (board, before):
            open(p, 'w', encoding='utf-8').close()
        loose = _args([
            '--board', board, '--before', before,
            '--drc-json', wrote('d.json', {'violations': 3}),
            '--locks-json', wrote('l.json', {'findings': [],
                                             'lock_patterns': []}),
            '--assembly-json', wrote('as.json', {'blocking': 1}),
            '--render-json', wrote('r.json', _fake_render(
                board, halo=50.0, crossings=60.0, hpwl=800.0)),
            # P-close's congestion gate needs a before/after pair, and the
            # pair must PASS: halo closed 50% of its gap and crossings 40%, so
            # the repair was proportionate. Fabricated here for the same reason
            # every other guard's evidence is -- the point of --dump-all is to
            # show the instructions, not to act on them.
            '--congestion-before', wrote('cb.json', _fake_render(
                before, halo=100.0, crossings=100.0, hpwl=1000.0)),
            '--intent-json', wrote('i.json', {'rules_run': ['envelope'],
                                              'parts_covered': 7,
                                              'violations': []}),
            '--waive', 'X:checked'])
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


def _self_test():
    """Every stage emits; every guard refuses without its evidence."""
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
        # Instructions must fit in a reading, not a scroll.
        want(len(out.splitlines()) <= 80, f'{key} stays under 80 lines')

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

        _int = _wr('i.json', {'rules_run': ['envelope'], 'violations': []})

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
        # restored the pristine board exactly. See wk/calibration/RESULT.md.
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

    # Banned shapes: hedging, and a subagent prompt the model might obey itself.
    # The evidence must be REAL files: this used to pass bare names ('b', 'a',
    # 'd'), so every stage with an existence check dumped its refusal instead of
    # its body and the banned-shape scan ran over error text. It went unnoticed
    # because the only assertion that depended on a body -- the subagent-prompt
    # one -- happened to be satisfied by the one stage that had no such check.
    with tempfile.TemporaryDirectory() as tmp2:
        _b = os.path.join(tmp2, 'b.kicad_pcb')
        _a = os.path.join(tmp2, 'a.kicad_pcb')
        for _p in (_b, _a):
            open(_p, 'w', encoding='utf-8').close()

        def _w(name, doc):
            p = os.path.join(tmp2, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        _ev = _args(['--board', _b, '--before', _a,
                     '--drc-json', _w('d.json', {'violations': 3}),
                     '--locks-json', _w('l.json', {'findings': []}),
                     '--assembly-json', _w('as.json', {'blocking': 1}),
                     '--render-json', _w('r.json', _fake_render(
                         _b, halo=50.0, crossings=60.0, hpwl=800.0)),
                     # A run that closed half its legality gap AND 40% of its
                     # crossings gap -- proportionate, so `_guard_congestion`
                     # lets it through. The fixture must pass the gate, not
                     # dodge it: it carries a real `metrics` block on both
                     # sides, so if the gate's arithmetic changes this notices.
                     '--congestion-before', _w('cb.json', _fake_render(
                         _a, halo=100.0, crossings=100.0, hpwl=1000.0)),
                     '--intent-json', _w('i.json', {'rules_run': ['envelope'],
                                                    'parts_covered': 7,
                                                    'violations': []}),
                     '--waive', 'X:y'])
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
