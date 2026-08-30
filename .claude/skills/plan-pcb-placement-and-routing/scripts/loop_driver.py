#!/usr/bin/env python3
"""The loop BETWEEN placement and routing, one stage at a time.

The two halves each have a driver, and each keeps its own rules in front of the
executor one stage at a time. Their accept rules genuinely conflict -- placement
accepts a lap when the named finding it aimed at is gone, routing accepts an
iteration when `blocking` strictly decreased -- so an executor holding both
skills at once has two contradictory definitions of "better" live in the same
context. That is the confusion this driver removes: it never emits both.

What it adds beyond sequencing is the part nothing else owns:

  * the hand-off REFUSES without a placement close-out, so routing cannot start
    on a board whose placement was never proved;
  * a retry REFUSES without a stated classification, because the three failure
    shapes re-enter at three different points and cost is asymmetric -- a wrong
    parameter guess wastes iterations, a wrong placement guess wastes the whole
    routed board;
  * a placement-shaped re-entry MARKS every routed board stale, by name, so the
    next pass cannot quietly reuse one.

State crosses the boundary on DISK (the converge ledger), never in a head.
Delegation is a choice about context volume, not about correctness -- see
--delegate.

    python3 -X utf8 <this> --stage L1 --board b.kicad_pcb
    python3 -X utf8 <this> --list
    python3 -X utf8 <this> --dump-all
    python3 -X utf8 <this> --self-test

Tags: <stage_instructions> act on these; <subagent_prompt> copy verbatim into a
teammate; <error> you skipped evidence.

Exit: 0 emitted, 2 usage, 4 a guard refused.
"""
import argparse
import hashlib
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(HERE))))
# #522/py_placer layout: the engine modules (board_store, converge, ...) live
# in py_router/py_tools/py_placer, not at the repo root, so inserting only
# ROOT left `from board_store import sha256_file` failing when run as a
# script.
sys.path.insert(0, ROOT)
for _pkg in ('py_router', 'py_tools', 'py_placer'):
    _d = os.path.join(ROOT, _pkg)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)

SHAPES = ('parameter', 'placement', 'floorplan')


def err(text):
    return f'<error>\n{text}\n</error>'


def _load(path, what):
    if not path:
        return None, f'{what} not provided.'
    if not os.path.isfile(path):
        return None, f'{what}: no such file: {path}'
    try:
        with open(path, encoding='utf-8') as fh:
            return json.load(fh), None
    except Exception as exc:                                # noqa: BLE001
        return None, f'{what}: unreadable ({type(exc).__name__}: {exc})'


def _recorded(board, ledger):
    """Is THIS board in the ledger, by content?

    Not "did the executor say it recorded" -- the same sha256 the board store
    keys on. A run of the combined skill recorded two placement laps and zero
    routing ones, and nothing noticed: the film came out two frames long, the
    staleness list printed "none recorded" at the one moment it mattered, and
    step-back had nothing to step back to. Recording is the spine of all three,
    so the stage that depends on them checks it rather than trusting it.

    None when the question cannot be answered (no ledger yet, unreadable
    board): the caller must not turn "I could not check" into a refusal.
    """
    if not os.path.isfile(board) or not os.path.isfile(ledger):
        return None
    try:
        sys.path.insert(0, ROOT)
        from board_store import sha256_file
        sha = sha256_file(board)
    except Exception:                                       # noqa: BLE001
        return None
    return any(r.get('result_sha') == sha for r in _ledger_rows(ledger))


def _guard_route_render(a):
    """(ok, why) -- L3's --focus render, checked the way L2 checks its document.

    L3 used to test `if not a.render_json:` and nothing else, so ANY path that
    existed satisfied the stage that classifies the failure -- the decision the
    whole loop turns on. L2 and L5 both load, shape-test and board-bind their
    documents; this one took a filename.

    Ported from placement_driver._guard_render, with two checks that only make
    sense on a ROUTED board:

      * `instrument.summary_json` -- render_placement silently falls back to
        clustering LEGALITY findings when it gets no --summary-json, so a focus
        render without one is a picture of a different question. On a routed
        board that is the difference between "where did routing fail" and
        "where is the placement untidy".
      * `moved_refs` -- the freeze was pure prose and had no mechanism at all.
        A render made with --before names every footprint that moved, so if the
        routing half moved a part despite being told not to, this is where it
        surfaces. Nothing else in the chain looks.
    """
    doc, derr = _load(getattr(a, 'render_json', None),
                      'The focus render (--render-json)')
    if derr:
        return False, derr
    inst = doc.get('instrument') or {}
    shown = inst.get('board')
    if not shown:
        return False, (
            'That render carries no `instrument.board`, so it cannot say WHICH '
            'board it looked at. Re-render with --json-out, not bare --json.')
    if os.path.normcase(os.path.abspath(shown)) != \
            os.path.normcase(os.path.abspath(a.board)):
        return False, (
            f'That render is of a DIFFERENT board:\n'
            f'      rendered: {shown}\n'
            f'      classifying: {os.path.abspath(a.board)}\n\n'
            f'An earlier lap\'s render does not prove this one was looked at, '
            f'and classifying from it classifies the wrong failure.')
    if not isinstance(doc.get('checklist'), dict):
        return False, (
            'That render has no `checklist` block, so it answers none of the '
            'questions the read exists to ask. Re-render with --json-out.')
    if not inst.get('summary_json'):
        return False, (
            'That render was made without --summary-json, so its --focus '
            'panels cluster the LEGALITY findings, not the routing failures. '
            'render_placement falls back silently, which is why this has to be '
            'checked rather than assumed: the picture looks right and answers '
            'the wrong question.\n\nRe-render with --summary-json pointed at '
            'the route log carrying JSON_SUMMARY. If the routing half did not '
            'keep one, that is the finding -- say so rather than classifying '
            'from the legality clusters.')
    moved = doc.get('moved_refs')
    # render_placement serialises moved_refs as {'reference', 'dist'} DICTS,
    # and has since it was added. `map(str, ...)` on those printed
    # "{'reference': 'C7', 'dist': 2.0}, ..." into the one refusal whose whole
    # job is to NAME the parts that moved -- so the message that catches a
    # broken freeze was unreadable exactly when it fired.
    _names = sorted(m.get('reference', m) if isinstance(m, dict) else m
                    for m in (moved or []))
    if inst.get('before') and moved:
        return False, (
            f'THE FREEZE DID NOT HOLD. That render was made against a --before '
            f'board and reports {len(moved)} footprint(s) moved: '
            f'{", ".join(map(str, _names[:8]))}'
            f'{" ..." if len(moved) > 8 else ""}.\n\nThe routing half is told '
            f'the placement is frozen and that if it concludes a part must '
            f'move it should stop and say so. A routed board whose parts moved '
            f'is not a routing result: every placement measurement taken '
            f'before it is now stale, and the classification below would be '
            f'about a board nobody graded.\n\nGo back to the placement half '
            f'with what moved, or revert those parts and re-route.')
    return True, None


def _score_board_mismatch(board, payload):
    """The score's `board_sha` when it is NOT this board's, else None.

    Shared by L3 and L5 so the two cannot drift: L5 had this binding from the
    start and L3 did not, which is exactly how a classification got made about
    one board from another board's score. Returns None when the question is
    unanswerable (no sha, no board, board_store unimportable) -- an absent
    answer must not read as a mismatch.
    """
    if not board or not os.path.isfile(board) or not isinstance(payload, dict):
        return None
    psha = payload.get('board_sha')
    if not psha:
        return None
    try:
        sys.path.insert(0, ROOT)
        from board_store import sha256_file
        return None if sha256_file(board) == psha else psha
    except Exception:                                           # noqa: BLE001
        return None


# The hpwl gain below which the congestion READ is worth pointing at. It decides
# whether to print a warning beside the numbers -- it does NOT decide anything.
# It used to gate, and the calibration withdrew that
# (docs/placement-calibration.md):
# the premise "damage raises hpwl, so a repair lowers it" INVERTS on 1 of 3
# corpus boards, where a perfect repair scores a negative gain and the gate
# refused the correct answer.
#
# Being only a reporting cut is why it can stay an uncalibrated round number. It
# is also why placement_driver's matching flag was removed rather than tuned:
# that one gated, this one annotates, and a threshold nobody acts on does not
# need to be right -- it needs to be roughly where the interesting cases are.
_CONGESTION_RATIO = 0.25


def _guard_congestion(a):
    """`parameter` is the DEFAULT ANSWER of a classifier that cannot see
    congestion, so it has to be the one that carries evidence.

    Both placement tests L3 names are per-NET. `check_reachability` asks whether
    ONE pad can reach ITS net; `check_channels` asks whether ONE face has lane
    supply. Both can pass on every net of a board that no router can finish,
    because simultaneous routability is a GLOBAL property and neither test looks
    at more than one net at a time.

    Measured on neo6502 (run 15): 0 starved faces, every probed pad PASSABLE on
    the copper-free board -- and 1827 airwire crossings against the undamaged
    original's 1324, +38%. The verdict was `parameter`, the loop spent its
    authorised iteration on rip-up depth, and `blocking` did not move, because
    the arrangement was the problem and no parameter addresses an arrangement.

    So: a `parameter` re-entry must show that congestion is NOT the story. That
    is one number from a render both halves already produce.
    """
    if not a.congestion_json:
        return False, (
            'A `parameter` verdict needs a congestion comparison, and there is '
            'none (--congestion-json).\n\n'
            'Every placement test L3 names is per-NET -- check_reachability asks '
            'if ONE pad reaches ITS net, check_channels if ONE face has supply. '
            'A board can pass both on every net and still be unroutable, because '
            'routing them SIMULTANEOUSLY is a global property that neither test '
            'can see. `parameter` is what a blind classifier returns by default, '
            'so it is the verdict that has to prove itself.\n\n'
            'Measured (neo6502, run 15): 0 starved faces, every probed pad '
            'PASSABLE -- and crossings 1827 against the original board\'s 1324. '
            'The `parameter` iteration that followed moved `blocking` by nothing.\n\n'
            'Render the COPPER-FREE board the route ran on, and the board '
            'placement started from:\n'
            '  python3 -X utf8 py_tools/render_placement.py <the placed board> \\\n'
            '      --clearance <floor> --ignore-nets <poured nets> \\\n'
            '      --json-out wk/cong_now.json -o wk/cong_now.png\n'
            '  python3 -X utf8 py_tools/render_placement.py <the pre-placement board> \\\n'
            '      --clearance <floor> --ignore-nets <poured nets> \\\n'
            '      --json-out wk/cong_base.json -o wk/cong_base.png\n'
            '  ... --stage L4 --shape parameter \\\n'
            '      --congestion-json wk/cong_now.json '
            '--congestion-baseline wk/cong_base.json\n\n'
            'Render the COPPER-FREE board deliberately: on a routed board a '
            'CAGED verdict means the ROUTER\'S OWN COPPER boxed the pad in -- '
            'rip-up depth -- not geometry. Run 15 had three genuine CAGED pads '
            'on the routed board and ALL THREE were PASSABLE once the copper '
            'was gone.')
    now, nerr = _load(a.congestion_json, 'The congestion render (--congestion-json)')
    if nerr:
        return False, nerr
    m_now = (now or {}).get('metrics')
    if not isinstance(m_now, dict) or not isinstance(
            m_now.get('hpwl'), (int, float)):
        return False, (
            'That render carries no numeric `metrics.hpwl`, so it cannot '
            'answer the routability question. Re-render with --json-out (not a '
            'bare --json).')
    if not a.congestion_baseline:
        return False, (
            'An hpwl figure on its own is not evidence -- it is neither good '
            'nor bad without the board it came from. Pass '
            '--congestion-baseline <render of the pre-placement board>.')
    base, berr = _load(a.congestion_baseline, 'The congestion baseline')
    if berr:
        return False, berr
    m_base = (base or {}).get('metrics')
    if not isinstance(m_base, dict) or not isinstance(
            m_base.get('hpwl'), (int, float)):
        return False, ('The congestion baseline carries no numeric '
                       '`metrics.hpwl`. Re-render it with --json-out.')
    # hpwl, NOT crossings. The placement skill's non-negotiable 4 is explicit:
    # report crossings, never gate on it, because it correlates POSITIVELY with
    # distance-to-truth -- DISTANCE, never routed blocking, which nothing has
    # correlated it against (docs/placement-predictors.md)
    # (r = +0.780 over 29 candidates; one candidate beat the
    # human original's crossings while sitting 18.7 mm out of position). A gate
    # on crossings is pressure toward a worse board. hpwl's minimum is at the
    # truth, so it is the one that can carry a gate.
    #
    # NOT universally, and the exception is why the threshold below was
    # withdrawn: on piantor the DAMAGED board scores LOWER hpwl than the truth
    # (1965.07 against 2263.61), because `swap` shortens nets on a regular
    # matrix. hpwl's direction has to be checked per board, not assumed.
    # See docs/placement-calibration.md.
    b, n = float(m_base['hpwl']), float(m_now['hpwl'])
    if b <= 0:
        return True, None
    gain = (b - n) / b
    # REPORT, do not refuse. The threshold that used to live here was withdrawn
    # on measurement (docs/placement-calibration.md): the same premise --
    # damage raises hpwl, so a repair lowers it -- INVERTS on 1 of 3 corpus
    # boards, where a perfect repair scores a negative gain. A gate that
    # refuses the
    # correct answer must not refuse. What survives is the requirement above:
    # you must LOOK at global congestion before calling a failure
    # `parameter`-shaped, because every per-net test can pass on a board no
    # router can finish. That part needs no threshold.
    if gain >= _CONGESTION_RATIO:
        return True, None
    _cx_b = m_base.get('crossings')
    _cx_n = m_now.get('crossings')
    _cx = (f'  crossings  {float(_cx_b):.0f} -> {float(_cx_n):.0f}'
           f'   [REPORTED, never gated -- non-negotiable 4]\n'
           if isinstance(_cx_b, (int, float))
           and isinstance(_cx_n, (int, float)) else '')
    if a.accept_congestion and str(a.accept_congestion).strip():
        return True, (
            f'  CONGESTION READ (accepted): hpwl {b:.1f} -> {n:.1f} '
            f'({gain * 100:+.1f}% of the gap closed).\n'
            f'  Reason on the record: {str(a.accept_congestion).strip()}\n')
    # BINDING on a DISPOSITION, not on the numbers -- the same shape as
    # P-close's. Printing this and proceeding would have changed nothing about
    # run 15 except that the evidence would have been on screen while the
    # routing iteration was spent anyway. The driver does not claim the
    # placement is wrong; it claims nobody has said which it is.
    return False, (
        f'The congestion read says this may be PLACEMENT-shaped, and nothing '
        f'has said otherwise.\n\n'
        f'    hpwl       {b:.1f} -> {n:.1f}   ({gain * 100:+.1f}% of the gap '
        f'closed)\n' + _cx.replace('  crossings', '    crossings') +
        f'\n'
        f'    The placement left {(1 - gain) * 100:.1f}% of the wirelength it '
        f'started with.\n'
        f'    Every per-net test can still pass here -- that is the blind spot: '
        f'on neo6502\n'
        f'    the classifier saw 0 starved faces and every pad PASSABLE, '
        f'returned `parameter`,\n'
        f'    and the iteration it bought moved `blocking` by nothing. If a '
        f'placement lever is\n'
        f'    left, it is cheaper than a routing iteration.\n\n'
        f'The numbers are NOT a verdict on the placement and cannot be: on one '
        f'corpus board a\n'
        f'PERFECT repair scores worse than the damage it repaired, because '
        f'`swap` shortens nets\n'
        f'on a regular matrix (docs/placement-calibration.md). What is missing '
        f'is a DECISION.\n\n'
        f'Either re-enter at PLACEMENT -- --shape placement, which is the '
        f'cheaper direction --\n'
        f'or say why parameter is right anyway:\n'
        f'    --accept-congestion "<the measurement, or the reason>"\n')


def _ledger_rows(path):
    if not path or not os.path.isfile(path):
        return []
    out = []
    for line in open(path, encoding='utf-8'):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out


# --------------------------------------------------------------------------
# stages
# --------------------------------------------------------------------------

#: There used to be two size thresholds here, DELEGATE_ABOVE_PARTS = 200 and
#: DELEGATE_ABOVE_NETS = 300, deciding when a half ran in a teammate instead of
#: in the orchestrator's own context. They are gone, and `_delegation` explains
#: why: the thing they were proxying for (context volume) is not the thing that
#: actually goes wrong. What goes wrong is that an inline inner loop absorbs the
#: outer loop's stage, and that happens at any size.
#:
#: The counts are still read and still printed. They are context in the
#: emission, not a decision, so there is no number left to tune and no split
#: decision left to reconcile between the two halves.


def _work(a):
    """The run's work dir -- where the PARENT names the handback artifacts.

    A contract where the teammate chooses the paths is not checkable: the parent
    has nothing to compare against, so every downstream gate degrades to
    trusting a message. Naming them here inverts that. The teammate fills paths
    the parent already knows, verifying a handback becomes os.path.isfile() plus
    the bindings that already exist, and a teammate that died mid-chain produces
    "these files do not exist" instead of a plausible paragraph.
    """
    # As GIVEN, not absolutised: every other path this driver emits is echoed
    # in the form the caller used, and a Windows abspath spliced onto a
    # forward-slash suffix produces a mixed-separator path nobody can paste.
    if getattr(a, 'ledger', None):
        return os.path.dirname(a.ledger).replace('\\', '/') or '.'
    return 'wk'


def _cycle_index(ledger, starting=False):
    """Which CYCLE of the loop this is, 1-based, read off the ledger.

    A cycle ends when the loop re-enters placement after routing -- which is a
    designed path: L5's CONTINUE branch and L4's placement/floorplan re-entries
    all say `--stage L1`. The driver had no concept of one, and named the same
    output paths every time. Measured, on a real second cycle L2 printed:

        write the result to <workdir>/<board>/frozen.kicad_pcb
        Take the list from <workdir>/<board>/freeze_refs.json

    Both are CYCLE ONE's files. That `frozen.kicad_pcb` is cycle 1's
    `--authored-from` baseline and is referenced by the ledger by content hash,
    so following the instruction literally destroys the provenance every later
    gate checks against -- and check_complete's fab-floor check, which is the
    only route to UNSOUND, compares against exactly that file.

    Two refinements the real ledger forced, both about what is NOT a new lap:

      * routing before any placement does not start cycle 2 on the first
        placement lap (a routing-first run kept the bare names, as documented);
      * a placement row whose board content was ALREADY recorded produced no
        new board, so it is a disposition, not a lap. Measured: iteration 33 of
        the run-17 ledger is `kind: placement`, lever "cycle2 VERIFIER
        DISPOSITION (no pose change; same board as it.30)", and it re-records
        it.30's sha -- it read as a third cycle on a ledger whose own levers
        say cycle 2 throughout.

    THE STAGE MATTERS, and this is `starting` -- the third thing measurement
    forced. L1 is asked BEFORE its placement lap exists: at the top of cycle 2
    the ledger is [placement..., routing...] and no cycle-2 placement row has
    been written yet, so counting only placement phases returns 1 and L1 names
    cycle ONE's placed board -- the exact defect this function is for, still
    live on the one stage that opens a cycle. `starting=True` (L1: about to
    OPEN a placement phase) therefore counts the phase it is about to create
    when the ledger currently ends in routing. `starting=False` (L2 and after:
    about to route the placement that has just been recorded) reports the phase
    the current board belongs to.

    It does NOT err high: a cycle-2 placement lap that reproduces cycle 1's
    board byte for byte reads as a disposition and holds the count at 1. That
    is why the collision note exists beside it -- content-addressed, and it
    fires whatever this returns.
    """
    phases, phase, seen = 0, None, set()
    for r in _ledger_rows(ledger):
        kind, sha = r.get('kind') or '', r.get('result_sha')
        if kind in ('completion', 'routing'):
            if phase == 'placement':
                phase = 'routing'
        elif kind == 'placement':
            if phase != 'placement' and sha not in seen:
                phases, phase = phases + 1, 'placement'
        if sha:
            seen.add(sha)
    if starting and phase == 'routing':
        phases += 1
    return max(1, phases)


def _cyc_name(name, n):
    """'frozen.kicad_pcb' -> 'frozen_c2.kicad_pcb' on cycle 2, unchanged on 1.

    Cycle 1 keeps the bare names on purpose: every chain in the docs, every
    recorded manifest and every existing run dir uses them, and renaming them
    would strand all of it to fix a bug that only exists from cycle 2 on.
    """
    if n <= 1:
        return name
    stem, ext = os.path.splitext(name)
    return f'{stem}_c{n}{ext}'


#: The handback artifacts the PARENT names, per stage. Values are the cycle-1
#: names; `_paths` applies the cycle suffix.
_ARTIFACTS = ('placed.kicad_pcb', 'assembly_close.json',
              'place_close_render.json', 'freeze_refs.json',
              'frozen.kicad_pcb', 'routed.kicad_pcb', 'score.json',
              'route.log', 'routing_close.json', 'handoff.json', 'handoff.png')


def _paths(a, starting=False):
    """(cycle, {name: path}) -- the artifact names for THIS cycle.

    `starting` is L1's: it opens a placement phase, so it must count the phase
    it is about to create. See _cycle_index.
    """
    work = _work(a)
    n = _cycle_index(getattr(a, 'ledger', None), starting=starting)
    # ...AND NEVER BELOW WHAT IS ALREADY ON DISK. L1 counts the phase it opens
    # and L2 counts the phase the recorded board sits in, so they disagree in
    # one measurable case: L1 says cycle 2, the placement half then finds
    # nothing to change and records a DISPOSITION (a row re-recording a board
    # already in the ledger -- run 17's it.33 shape), and the phase count drops
    # back to 1. L2 would then name cycle 1's frozen.kicad_pcb and take its
    # freeze list from cycle 1's freeze_refs.json, which is the very defect the
    # suffix exists to prevent, reached by another road.
    #
    # The artifacts L1 already told the half to write are the evidence, and
    # they are on disk. A cycle number never goes backwards.
    try:
        for f in (os.listdir(work) if os.path.isdir(work) else ()):
            stem, ext = os.path.splitext(f)
            m = re.match(r'^(.*)_c(\d+)$', stem)
            if m and f'{m.group(1)}{ext}' in _ARTIFACTS:
                n = max(n, int(m.group(2)))
    except Exception:                                       # noqa: BLE001
        pass
    return n, {k: f'{work}/{_cyc_name(k, n)}' for k in _ARTIFACTS}


def _ledger_collision(a, paths):
    """A WARNING block when a path we are about to name as an OUTPUT already
    holds a board THE LEDGER REFERENCES -- else ''.

    The belt beside the cycle suffix. A suffix fixes the case the driver can
    see coming; this names any output path that would land on a file the
    ledger's history depends on -- a step-back target, a film frame, an
    --authored-from baseline. Content-addressed, so it cannot be fooled by a
    path that merely looks familiar, and silent when the question is
    unanswerable (no ledger, unreadable file).

    A WARNING and not a refusal, deliberately. These stages are instruction
    printers -- the doctrine is "ask its driver for one stage at a time" -- so
    RE-INVOKING one after doing what it said is normal, and as a refusal this
    made every resumed or interrupted session exit 4 on an artifact it had
    correctly produced. Worse, its remedy ("move the existing file") is exactly
    what destroys the --authored-from provenance it exists to protect. The
    information is what was missing; the veto was never the point.
    """
    shas = {r.get('result_sha') for r in _ledger_rows(getattr(a, 'ledger', None))
            if r.get('result_sha')}
    if not shas:
        return ''
    try:
        sys.path.insert(0, ROOT)
        from board_store import sha256_file
    except Exception:                                       # noqa: BLE001
        return ''
    hits = []
    for p in paths:
        if not os.path.isfile(p):
            continue
        try:
            s = sha256_file(p)
        except Exception:                                   # noqa: BLE001
            continue
        if s in shas:
            hits.append((p, s))
    if not hits:
        return ''
    named = '\n'.join(f'    {p}  = ledger board {s[:12]}...' for p, s in hits)
    return (
        f'\nALREADY IN THE LEDGER -- do not overwrite these:\n{named}\n'
        f'  Re-running this stage? Then they are the artifacts you already '
        f'produced; leave\n'
        f'  them and skip ahead. Starting a NEW cycle? Then record the laps '
        f'first so the\n'
        f'  cycle count advances and this stage names fresh files -- do not '
        f'move or rename\n'
        f'  these, because check_complete --authored-from grades against them '
        f'and without\n'
        f'  that baseline UNSOUND is unreachable.\n')


def _log_invocation(a, stage, out, code):
    """Append this invocation to a log BESIDE the ledger. stdout is untouched.

    The `stages.log` that made the measured run auditable exists only because
    the CALLER teed it. Nothing the driver did was recorded by the driver, so an
    unlogged invocation could only ever be inferred -- never detected -- and a
    stage that ran, refused and was quietly worked around left no trace at all.
    The loop's own history is what every gate downstream reads.

    Never allowed to break a stage: a driver that refuses to run because it
    could not journal is worse than one that is occasionally unjournalled.
    """
    try:
        d = os.path.dirname(getattr(a, 'ledger', '') or '') or '.'
        if not os.path.isdir(d):
            # SAY SO. An unlogged invocation that is also unannounced is the
            # exact hole this function exists to close -- it could then only be
            # inferred, never detected. stderr, so stdout stays byte-identical.
            print(f'loop_driver NOTE: no log written -- {d} does not exist, so '
                  f'this invocation is UNRECORDED. Create the ledger directory '
                  f'(or pass --ledger inside it) and the driver journals '
                  f'itself.', file=sys.stderr)
            return None
        p = os.path.join(d, 'loop_driver.log')
        row = {'t': round(time.time(), 3),
               'iso': time.strftime('%Y-%m-%dT%H:%M:%S'),
               'stage': stage, 'exit': code, 'refused': bool(code == 4),
               'board': getattr(a, 'board', None),
               'ledger': getattr(a, 'ledger', None),
               'cwd': os.getcwd(), 'argv': list(sys.argv),
               # The emitted text by content: two consecutive refusals coming
               # out byte-identical is itself a finding (it was one), and a
               # reader cannot see that from a log that keeps only the stage.
               'out_sha': hashlib.sha256(
                   (out or '').encode('utf-8')).hexdigest()[:16],
               'out_lines': len((out or '').splitlines())}
        with open(p, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps(row, sort_keys=True) + '\n')
        return p
    except Exception:                                       # noqa: BLE001
        return None


#: Never open anything but the board you were given. A delegated half is a fresh
#: agent with Read and Glob, and on the perturbed corpus the control board and
#: the pose record sit one directory above the subject. The fence has always
#: been behavioural; before delegation it relied on ONE agent declining to look,
#: and now it relies on three. Naming the carriers is what closes the path.
FENCE_CLAUSE = '''
The ONLY board you may open is the one named above. If you come across a
control board, a `_truth/` directory, a `*.perturb.json` pose record, a `.bak`
or any other board file in or near the work dir, do NOT open it: say that you
found it and carry on without it. Reading one silently invalidates the whole
run, and nothing downstream can detect that it happened.
Use the repo's engine tools for every board mutation. If you write ANY script
that computes or writes poses or copper, disclose it in your next message and
name it in every ledger lap it feeds -- a disclosed hand-assist is a finding;
an undisclosed one silently invalidates the run.'''


def _board_size(board):
    """(parts, nets) with pads/names, or (None, None) if it cannot be read.

    None matters: "I could not count" must not silently become "small enough
    to run inline", so the caller keeps whatever the flags said instead of
    inventing a decision out of a failed read.
    """
    if not board or not os.path.isfile(board):
        return None, None
    try:
        sys.path.insert(0, ROOT)
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(board)
        return (sum(1 for f in (pcb.footprints or {}).values() if f.pads),
                sum(1 for n in (pcb.nets or {}).values() if n.name))
    except Exception:                                       # noqa: BLE001
        return None, None


def _part_count(board):
    return _board_size(board)[0]


def _delegation(a, half='placement'):
    """(delegate?, why) -- BOTH halves go to a teammate, and the size is context.

    Delegation used to be decided by board size. Run 14 is why it is not any
    more. That board measured 191 pad-bearing parts against a 200 threshold and
    150 nets against 300, so both halves ran inline -- and the routing half,
    holding far more context than the orchestrator, classified its own failure
    in its V2 stage and acted on it. The classification never travelled up, L3
    and L4 never fired, and a full re-run of the routing chain happened that the
    outer loop never authorised and never saw.

    That is not a threshold being set wrong. **An inline inner loop can silently
    do the outer loop's job**, because it always knows more than the parent
    does, and no number distinguishes the boards where it will from the boards
    where it will not. A teammate cannot: the only thing that crosses the
    boundary is a document the parent has to read.

    So the default is delegate, both halves, every size. Uniformity is also what
    keeps the delegated path exercised -- until now it was covered by nothing
    but a text-emission assertion, so a handback could not have been wrong in
    any way a test would notice.

    The size is still MEASURED and still printed, because "191 parts <= 200" is
    the line that told run 14 what it had chosen. It is context now, not a
    decision, and an unreadable board therefore changes nothing.

    `--no-delegate` is the escape hatch and is load-bearing: the self-test, the
    parity gates and any headless CI have to run in one process.
    """
    parts, nets = _board_size(a.board)
    if half == 'routing':
        val, unit, other, other_unit = nets, 'nets', parts, 'parts'
    else:
        val, unit, other, other_unit = parts, 'parts', nets, 'nets'
    if val is None:
        size = (f'{a.board} could not be read to size it, which changes '
                f'nothing -- the decision no longer depends on the size')
    else:
        size = f'{val} {unit}, {other} {other_unit} for context'
    if getattr(a, 'no_delegate', False):
        return False, f'--no-delegate was passed, so this half runs here ({size})'
    return True, (f'this half goes to a TEAMMATE. Always, not because of a '
                  f'threshold ({size}). --no-delegate runs it here instead')


def l1(a):
    """Place. Delegated by default; --no-delegate is the escape hatch."""
    delegate, why = _delegation(a)
    work = _work(a)
    cyc, P = _paths(a, starting=True)
    _placed, _asm = P['placed.kicad_pcb'], P['assembly_close.json']
    _rend, _refs = P['place_close_render.json'], P['freeze_refs.json']
    _clash = _ledger_collision(a, [_placed, _asm, _rend, _refs])
    _cycnote = ('' if cyc == 1 else
                f'\nCYCLE {cyc} (the ledger records {cyc - 1} completed '
                f'placement->routing pass(es)). The handback names below carry '
                f'a _c{cyc} suffix ON PURPOSE: cycle 1\'s files are the '
                f'--authored-from baseline and the boards step-back and the '
                f'film read by content hash. Do not write over them.\n')
    if delegate:
        return f'''<stage_instructions stage="L1" name="place (delegated)" of="5">
DELEGATING: {why}.{_cycnote}{_clash}

Delegate the placement half to a TEAMMATE spawned with an agent type that HAS
the Agent tool -- `claude` or `general-purpose`, never `Explore` or `Plan`,
whose definitions exclude it. The placement skill dispatches its own
verification subagent at its close-out, and a half that cannot spawn cannot
verify itself. Give it the prompt below verbatim.

<subagent_prompt agent="claude" description="place {os.path.basename(a.board)}">
Drive the placement half of this board to its close-out, and do not route.

  board:  {a.board}
  ledger: {a.ledger}

Use /plan-pcb-placement. Ask its driver for one stage at a time:
  python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/placement_driver.py \\
      --stage P0 --board {a.board}
and follow the stage it prints, including its refusals -- an <error> means a
gate is holding, so produce what it asks for rather than working around it.

Record every accepted lap with converge.py --kind placement into the ledger
above. The next stage checks the board against that ledger BY CONTENT HASH, so
a lap you did not record is a lap that did not happen.

{_clash}WRITE YOUR CLOSE-OUT TO THESE EXACT PATHS. They are named here rather than
chosen by you, because the next gate opens the files; it does not read your
message.

  placed board : {_placed}
  close-out    : {_asm}
                 python3 -X utf8 py_tools/check_assembly.py {_placed} \\
                     --clearance <the board's own floor> \\
                     --json {_asm}
  render       : {_rend}
                 the render_placement.py --json-out you actually READ
  freeze refs  : {_refs}
                 a JSON list of the refs whose pose is a DECISION -- moved
                 deliberately, or mechanically pinned. WRITE it: a pose diff
                 cannot tell a deliberate re-seat from a sweep, and it froze 76
                 moved parts where 24 were ever named. Not `must_lock`, which
                 `place_seed --repair` LIFTS.

`assembly_close.json` must be check_assembly.py's output and nothing else.
board_score.py publishes a field called `blocking` too and means something
entirely different by it; handing that over silently disables three of the four
checks the gate runs.
{FENCE_CLAUSE}

Return, and return ONLY:
  1. confirmation that each of the four paths above exists, or WHICH does not;
  2. what remains unfixed, each with the measurement that says it is unfixable
     at this stage;
  3. the refs you locked and why.
Do not summarise the process, and do not retype the numbers -- the gate
re-reads them from the files.
</subagent_prompt>

When it returns, continue here with --stage L2 on the paths named above.

Next: python3 -X utf8 {sys.argv[0]} --stage L2 \\
          --board {_placed} \\
          --ledger {a.ledger} \\
          --placement-report {_asm}
</stage_instructions>'''
    return f'''<stage_instructions stage="L1" name="place" of="5">
INLINE: {why}.{_cycnote}{_clash}

Place this board yourself, driven. Do not read the placement skill end to end:
ask its driver for one stage at a time, so only one loop's rules are ever in
front of you.

  python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/placement_driver.py \\
      --stage P0 --board {a.board}

Follow it to P-close, including its refusals. Record every accepted lap into
{a.ledger} with converge.py.

--delegate forces a teammate for this half whatever the size, and
--no-delegate forces it inline. That is a context decision, not a correctness
one: the guards below are identical either way.

Next: python3 -X utf8 {sys.argv[0]} --stage L2 --board <placed board> \\
          --ledger {a.ledger} --placement-report <its close-out json>
</stage_instructions>'''


def l2(a):
    """Freeze what placement decided, then route."""
    work = _work(a)
    # Same separator as the paths beside it. A command block that mixes
    # C:\a\b with C:/a/b in one invocation is one nobody can paste.
    _res = getattr(a, 'accept_residue', None)
    if _res is not None:
        _bad = [n for n in _res if n not in L2_CHECKS]
        if not _res or _bad:
            return err(
                f'--accept-residue now names WHICH check you are accepting. '
                f'Valid: {" ".join(L2_CHECKS)}.'
                + (f' Unknown: {" ".join(_bad)}.' if _bad else
                   ' A bare --accept-residue waived all of them at once, which '
                   'is how a spurious `blocking` refusal also waived the '
                   '`oob_pad_count` gate (run 10, 21 parts with pad copper off '
                   'the board).'))
    rep, e = _load(a.placement_report, 'The placement close-out (--placement-report)')
    if e:
        return err(
            e + '\n\nRouting may not start on a board whose placement was '
                'never proved. L1 ends with a close-out carrying the four '
                'measurements; produce it, then come back.\n\nIf the board '
                'arrived already placed by someone else, run the placement '
                'gate alone and use ITS output:\n'
                '  python3 -X utf8 py_router/check_drc.py <board> --clearance <floor> '
                '--json wk/drc0.json\n'
                '  python3 -X utf8 py_tools/check_assembly.py <board> --json '
                'wk/assembly0.json')
    # Bind the close-out to the board it is unlocking. check_assembly writes the
    # graded board's path into the payload and this gate never compared it, so
    # an EARLIER lap's close-out (or another board's entirely) unlocked routing
    # on a board nobody graded. Cheap, and it closes a hole that is invisible
    # afterwards because the ledger records only that the stage passed.
    _graded = rep.get('board')
    if _graded and a.board:
        if os.path.normcase(os.path.abspath(_graded)) != \
                os.path.normcase(os.path.abspath(a.board)):
            return err(
                f'The placement close-out grades a DIFFERENT board than the one '
                f'being handed to routing:\n\n  close-out graded : {_graded}\n'
                f'  handing to route: {os.path.abspath(a.board)}\n\n'
                f'A stale close-out is not evidence about this board. Re-grade '
                f'the board you are actually routing:\n'
                f'  python3 -X utf8 py_tools/check_assembly.py {a.board} --json '
                f'wk/assembly_close.json')

    # SHAPE, BEFORE CONTENT. `blocking` exists in BOTH check_assembly's report
    # and board_score's, with completely different meanings -- a
    # pad-intersection-PAIR count vs a six-component TOTAL over unrouted +
    # broken + drc + undersized + floorplan + assembly -- while
    # buildable/verdict/locked_contacts/oob_pad_count exist only in
    # check_assembly's. Measured on run 10: handing board_score's JSON here
    # silently disabled three of the four checks below (their keys are simply
    # absent, and `None is False` is False) and fired the fourth with
    # `blocking = 57` -- 100% unrouted, 0% assembly -- under a message blaming
    # "a blocking assembly pair". Nothing about that JSON looks malformed,
    # which is why a key-presence check is the only thing that catches it.
    #
    # NOT waivable by --accept-residue: a residue is a measured defect somebody
    # accepted, and this is the wrong instrument's output. There is nothing to
    # accept.
    _missing = [k for k in L2_CHECKS if k not in rep]
    if _missing:
        _hint = ''
        if rep.get('kind') == 'board-score' or 'blocking_by' in rep:
            _hint = (
                "\n\nThis looks like `board_score.py`'s JSON, not "
                "`check_assembly.py`'s. They share the field name `blocking` "
                "and mean different things by it: board_score's is a total "
                "over unrouted + broken + drc + undersized + floorplan + "
                "assembly, check_assembly's is a count of pad-intersection "
                "PAIRS. board_score grades the ROUTE; this gate grades the "
                "PLACEMENT.")
        return err(
            f'The placement close-out is not shaped like a `check_assembly.py` '
            f'report: it is missing {", ".join(_missing)}.\n\nThis gate reads '
            f'exactly four measurements -- {", ".join(L2_CHECKS)} -- and a '
            f'document missing any of them cannot answer the questions the '
            f'gate asks. A missing key is not a passing one; the checks that '
            f'read it would simply not run.{_hint}\n\nProduce the right '
            f'document:\n  python3 -X utf8 py_tools/check_assembly.py '
            f'{a.board or "<board>"} --json wk/assembly_close.json')

    def _count(key):
        """(value, refusal). Anything that is not a finite, non-negative real
        is UNMEASURED, never clean.

        The guards were `isinstance(x, int) and x > 0`, which passes a
        string "7" (fails the isinstance, so the refusal never fires), a float
        3.0 (not an int), a NaN (`nan > 0` is False, and json round-trips a
        bare NaN) and a negative. Every one of those reads as a clean board.
        """
        v = rep.get(key)
        if v is None:
            return None, None
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return None, (
                f'The close-out reports `{key}` as {type(v).__name__} {v!r}, '
                f'not a number. A count that is not a number has not been '
                f'measured, and an unmeasured count is not a passing one -- '
                f'the old guard here accepted exactly this shape as clean. '
                f'Re-produce the close-out with check_assembly --json.')
        if v != v or v in (float('inf'), float('-inf')):
            return None, (
                f'The close-out reports `{key}` = {v!r}. NaN and infinity both '
                f'compare False against `> 0`, so they used to read as a clean '
                f'board. Re-produce the close-out.')
        if v < 0:
            return None, (
                f'The close-out reports `{key}` = {v}, which is negative and '
                f'therefore not a count. Re-produce the close-out.')
        return v, None

    # check_assembly always writes `buildable` and `locked_contacts`; this gate
    # read NEITHER, so a board it calls NOT BUILDABLE passed. SKILL.md's
    # non-negotiable 3 names all three -- a blocking pair, a locked-part
    # contact, a rigid inconsistency -- and the driver enforced two.
    _buildable = rep.get('buildable')
    _verdict = rep.get('verdict')
    if (_buildable is False or (isinstance(_verdict, str)
                                and 'NOT BUILDABLE' in _verdict.upper())) \
            and not _accept(a, 'buildable'):
        return err(
            f'The placement close-out says the board is NOT BUILDABLE '
            f'(buildable={_buildable!r}, verdict={_verdict!r}).\n\nThis gate '
            f'used to read only `blocking` and `oob_pad_count`, so a board its '
            f'own instrument calls unbuildable walked through. Go back to the '
            f'placement half, or re-run with --accept-residue buildable if this is a '
            f'measured, named, accepted residue.')
    _lc, _lcerr = _count('locked_contacts')
    if _lcerr:
        return err(_lcerr)
    if _lc and _lc > 0 and not _accept(a, 'locked_contacts'):
        return err(
            f'The placement close-out reports locked_contacts = {_lc}: copper '
            f'or a part is in contact with a KiCad-LOCKED part.\n\nA locked '
            f'pose is a decision somebody made -- an enclosure standoff, a '
            f'panel cut-out -- so a placement search may not settle this by '
            f'moving the other part somewhere it likes better, and routing '
            f'cannot settle it at all. Fix it in the placement half.')

    blocking, _berr = _count('blocking')
    if _berr:
        return err(_berr)
    if blocking is None:
        return err(
            'The placement close-out carries no `blocking` count, so it does '
            'not say whether the placement is assembly-clean -- and a missing '
            'measurement is not a passing one. Produce the close-out from the '
            'instrument that measures it:\n'
            '  python3 -X utf8 py_tools/check_assembly.py <board> --json '
            'wk/assembly_close.json\n\nAn EMPTY or unrelated JSON satisfies a '
            'file-exists check and tells you nothing; that is the failure this '
            'refusal exists to stop.')
    if blocking > 0 and not _accept(a, 'blocking'):
        return err(
            f'The placement close-out reports blocking = {blocking}. A board '
            f'that reaches routing with a blocking assembly pair will fail '
            f'routing for a reason routing cannot fix, and the retries spent '
            f'there are wasted.\n\nGo back to the placement half, or re-run '
            f'this stage with --accept-residue blocking if that residue is measured '
            f'unfixable and NAMED in the close-out -- which is a decision you '
            f'are recording, not a flag that makes it go away.')
    # blocking == 0 is not the same as routable. A part whose pads lie off the
    # board carries nets no router can reach, and it produces NO blocking pair
    # -- there is nothing for it to collide with out there. Measured on a
    # damaged board that passed this gate at blocking 0 with three parts
    # sitting wholly off the outline; every net on them would have failed, and
    # the loop would have spent a routing pass to discover it.
    oob, _ooberr = _count('oob_pad_count')
    if _ooberr:
        return err(_ooberr)
    if oob and oob > 0 and not _accept(a, 'oob_pad_count'):
        return err(
            f'The placement close-out reports blocking = 0, but '
            f'oob_pad_count = {oob}: {oob} part(s) carry pad copper OFF the '
            f'board. Those parts are assembly-clean precisely because nothing '
            f'is out there to collide with, and their nets cannot be routed at '
            f'all.\n\nThis is placement-shaped damage, and it is cheaper to '
            f'fix now than to discover it as a routing failure and re-enter. '
            f'Go back to the placement half.\n\nIf the overhang is BY DESIGN '
            f'-- a card edge, a switch actuator, a castellated module -- '
            f'declare it in the floorplan intent (edge_connectors), which '
            f'exempts it and makes the exemption reviewable, and then re-run '
            f'with --accept-residue oob_pad_count.')
    # The PLACED board must be in the ledger, the same way L3 checks the routed
    # one. It never was, so a delegated placement half could do the whole thing
    # and record nothing, and the first stage to notice would be L4's staleness
    # list printing "none recorded" at the one moment it mattered.
    if _recorded(a.board, a.ledger) is False:
        return err(
            f'{a.board} is not in {a.ledger}: no lap records a board with this '
            f'content hash.\n\nThe placement half was asked to record every '
            f'accepted lap with converge.py --kind placement, and the board it '
            f'handed over is not one of them. Either it recorded nothing, or '
            f'the board changed after the last lap was recorded -- and both '
            f'are the same problem for everything downstream, because the '
            f'ledger is what step-back, the film and the staleness list all '
            f'read.\n\nRecord it:\n  python3 -X utf8 py_placer/converge.py record '
            f'--ledger {a.ledger} \\\n      --board {a.board} --kind placement '
            f'--argv <the command that produced it>')
    # Deliberately NOT refusing on an empty or absent ledger. `_recorded`
    # returns None there, and this gate's own refusal text documents the case:
    # a board placed by someone else, handed straight to routing, has no
    # placement lap to record and never will. "I could not check" must not
    # become "you failed", or the legitimate path stops working.
    delegate, why = _delegation(a, half='routing')
    cyc, P = _paths(a)
    _frozen, _routed = P['frozen.kicad_pcb'], P['routed.kicad_pcb']
    _refs, _score, _log = P['freeze_refs.json'], P['score.json'], P['route.log']
    _close, _hoj, _hop = (P['routing_close.json'], P['handoff.json'],
                          P['handoff.png'])
    # The freeze SOURCE is the board handed to this stage, not a name derived
    # from the work dir: on a second cycle the placed board is placed_c2, and
    # `copy_board placed.kicad_pcb frozen.kicad_pcb` would freeze cycle 1's
    # board into cycle 1's baseline -- twice wrong in one line.
    _clash = _ledger_collision(a, [_frozen, _routed, _close, _score])
    # The --authored-from baseline is CYCLE 1's frozen board on EVERY cycle,
    # exactly as the cycle note below promises. The emitted command used to
    # pass the current cycle's frozen_cN instead (run-17 audit, D5) -- and the
    # writeback only ever loosens, so a later cycle's frozen board may already
    # carry ratcheted floors: grading against it silently weakens the very
    # check the note says it feeds.
    _authored = f'{work}/frozen.kicad_pcb'
    _cycnote = ('' if cyc == 1 else
                f'\nCYCLE {cyc}. The output names below carry a _c{cyc} suffix: '
                f'cycle 1\'s frozen.kicad_pcb is this chain\'s --authored-from '
                f'baseline and is in the ledger by content hash. Overwriting it '
                f'is not a tidiness question -- check_complete grades this run '
                f'against it, and without it UNSOUND is unreachable.\n')
    freeze = f'''FREEZE first, INTO A NEW FILE. Lock the refs whose poses are decisions and
write the result to {_frozen} (with its .kicad_pro -- use
copy_board.py). Take the list from {_refs}, which the placement
half wrote; do not re-derive it by diffing poses.

  python3 -X utf8 py_router/copy_board.py {a.board} {_frozen}
  ... stamp (locked yes) on the refs that file names ...
  python3 -X utf8 py_placer/converge.py record --ledger {a.ledger} \\
      --board {_frozen} --kind placement \\
      --lever "L2 freeze: <n> refs the placement half named as decisions"

A later step that moves a decided pose silently undoes the placement work, and
nothing downstream will report it -- that is why the freeze exists.

DO NOT FREEZE IN PLACE. Writing locks back into placed.kicad_pcb changes its
content hash, and the placement half may still be running: measured, one did,
saw the hash move under it, read that as corruption, reverted the file, and had
to restore it from the content-addressed store. The placed board is that half's
artifact and its ledger binding; leave it alone and hand on the new file.'''
    if delegate:
        return f'''<stage_instructions stage="L2" name="freeze, then route (delegated)" of="5">
DELEGATING: {why}.{_cycnote}{_clash}

{freeze}

Freeze BEFORE you hand it over -- the locks are a decision from the placement
half, and a teammate that receives an unfrozen board cannot know which poses
were deliberate.

Then delegate the routing half to a TEAMMATE, for the same reason L1 does: use
an agent type that HAS the Agent tool (`claude` or `general-purpose`, never
`Explore` or `Plan`), because the routing skill fans out three verification
subagents at close-out and a half that cannot spawn cannot verify itself. This
half also produces the most output of anything in the loop -- a route log on a
board this size runs to thousands of lines -- so it is the one most worth
keeping out of this context.

<subagent_prompt agent="claude" description="route {os.path.basename(a.board)}">
Route this board to its close-out. The placement is FROZEN: do not move a
footprint, and if you conclude one must move, stop and say so rather than
moving it.

  board:  {_frozen}
  ledger: {a.ledger}

Route by FOLLOWING THE ROUTING SKILL, so the routing loop's rules are the
only ones in front of you:

  Follow /plan-pcb-routing on {_frozen}, one stage at a time, from its
  "How to run this skill" section onward.

Its Step 0 placement gate will pass: the placement half just did that work,
and the close-out is the evidence -- so it drops straight through to the
routing stages. Follow its refusals too -- when a gate holds, produce what it
asks for rather than working around it.

TAKE THE HAND-OFF PICTURE before your first route. This is the last moment the
board is copper-free, so it is the only render that shows the placement ALONE --
afterwards every panel is placement plus whatever the router did:

  python3 -X utf8 py_tools/render_placement.py {_frozen} \\
      --clearance <the board's own floor> --ignore-nets <the poured nets> \\
      --json-out {_hoj} -o {_hop}

Anything its WHAT THIS PANEL SHOWS block names as off the outline, stacked or
hole-conflicting will still be there after the route, and no router setting
removes it. Keep that board: it is the baseline `check_channels --baseline`
needs, and you return its path below.

No step has a wall-clock budget -- `--deadline` was removed everywhere (#621:
no result may depend on timing, so the same board with the same arguments has
to produce the same copper on a slow machine and a fast one), and passing it is
an argparse error. A harness timeout SIGTERMs the tool, its own shutdown never
runs, and you get exit 143 with no partial board and no summary. 143 and 124
are the SHELL's codes, not a tool's. Run long steps DETACHED, and bound them by
SCOPE -- fewer nets in --nets, a tighter --max-ripup, a named set of refs
rather than a board sweep, --evict-depth 0 (already the default; it
censuses and moves nothing, where 1 and 2 also trade blockers out) -- rather than by a clock.

Record EVERY accepted iteration into the ledger above with converge.py, and
every rejected one with --rejected before stepping back. The stage after this
one refuses a board that is not in the ledger, by content hash.

WHEN YOU CLASSIFY A FAILURE, THE CLASSIFICATION DECIDES WHO FIXES IT:
  - `parameter`  -- yours. Re-enter your own failing R stage, record the lap,
                    carry on.
  - `placement`  -- NOT yours. Stop and return it. Fixing it means moving a
                    part, which you are forbidden to do anyway.
  - `floorplan`  -- NOT yours. Stop and return it. Fixing it means a different
                    arrangement, which is the other half's job.
A `placement` or `floorplan` verdict ends your turn. Do not spend router
parameters on it: that is the most expensive mistake available here, and a half
that fixes it locally is a half doing the outer loop's job invisibly.

{_clash}WRITE YOUR CLOSE-OUT TO THESE EXACT PATHS. They are named here rather than
chosen by you, because every gate after this opens the files; none of them
reads your message.

  routed board : {_routed}
  score        : {_score}          board_score.py --json
  route log    : {_log}           the one carrying JSON_SUMMARY
  close-out    : {_close}
                 python3 -X utf8 check_complete.py {_routed} \\
                     --authored-from {_authored} \\
                     --json {_close}

`--authored-from` is given to you above because only this loop knows it, and it
is not bookkeeping: without it check_complete's fab-floor check cannot run at
all, so UNSOUND becomes unreachable and a board that ships copper below its own
declared floor reads clean. The writeback only ever loosens, so the board you
were handed is the last thing left to compare against.
{FENCE_CLAUSE}

Return, and return ONLY:
  1. confirmation that each of the four paths above exists, or WHICH does not;
  2. the three routed-board VERDICT= lines, verbatim, one per line -- the
     ledger's own --final refuses without connectivity, drc and spec;
  3. SHAPE=<parameter|placement|floorplan>, or `none` if nothing failed;
  4. the failing nets BY NAME, not counted;
  5. anything left UNGRADED, named as unexamined rather than clean.

Do not summarise the process, and do not retype the numbers.

IF A STEP RUNS LONG, HAND BACK -- do not wait on it. You cannot: a teammate has
no way to sit on a backgrounded process and be woken when it exits, and both
routing halves in run 20 returned "still working, I'll wait" for exactly that
reason. Launch it DETACHED -- with a scope it can finish, since no tool stops
itself on a clock -- then return three things and stop:
  LOG=<where it is writing>   MARKER=<the file whose appearance means done>
  NEXT=<what consumes it>
This loop arms the wait and resumes you. That is a correct hand-back, not a
failure -- report it as one and it will be read as one. The marker must be
written by the STEP, never by you: a "done" file you write before returning says
the work finished when it has not started. The cheapest marker is the step's
own exit -- redirect it into a log and, as the LAST action of the same command
line, write its exit code into <label>.done. Wait on THAT file, never on a log
line (run 23 lost 25 minutes to a completion line that goes to stdout while the
waiter grepped the log).

READ ROUTE RESULTS FROM THE `JSON_SUMMARY_MIN:` LINE -- one per run,
authoritative-last, the MERGED tally in <1KB (#686). The big JSON_SUMMARY lines
are several kB each, several per log, with scope semantics the log itself warns
about; they are forensics, not your read.
</subagent_prompt>

When it returns, continue here with the paths it named. Do not retype its
numbers -- the gates re-read them from disk.

Two rules that are only true HERE, where the halves meet:
  - copper is not evidence about placement. A route that completed does not
    ratify the placement it ran on, and one that failed does not condemn it.
  - every routed board produced from THIS placement is valid only while this
    placement stands.

Next, on success: --stage L5. On a failure: --stage L3 --score <SCORE_JSON>
         --render-json <a --focus render; L3 will not open without one>
</stage_instructions>'''
    return f'''<stage_instructions stage="L2" name="freeze, then route" of="5">
INLINE: {why}.{_cycnote}{_clash}

{freeze}

Then route by FOLLOWING THE ROUTING SKILL, so the routing loop's rules are the
only ones in front of you:

  Follow /plan-pcb-routing on {_frozen}, one stage at a time, from its
  "How to run this skill" section onward.

Its Step 0 placement gate will pass: you just did that work, and the close-out
is the evidence -- so it drops straight through to the routing stages.

No step has a wall-clock budget -- `--deadline` was removed everywhere (#621:
no result may depend on timing, so the same board with the same arguments has
to produce the same copper on a slow machine and a fast one), and passing it is
an argparse error. A harness timeout SIGTERMs the tool, its own shutdown never
runs, and you get exit 143 with no partial board and no summary. 143 and 124
are the SHELL's codes, not a tool's. Run long steps DETACHED, and bound them by
SCOPE -- fewer nets in --nets, a tighter --max-ripup, a named set of refs
rather than a board sweep, --evict-depth 0 (already the default; it
censuses and moves nothing, where 1 and 2 also trade blockers out) -- rather than by a clock.

Two rules that are only true HERE, where the halves meet:
  - copper is not evidence about placement. A route that completed does not
    ratify the placement it ran on, and one that failed does not condemn it.
  - every routed board produced from THIS placement is valid only while this
    placement stands.

TAKE THE HAND-OFF PICTURE before the first route. This is the last moment the
board is copper-free, so it is the only render that shows the placement ALONE --
afterwards every panel is placement plus whatever the router did, and the two
become hard to separate by eye:

  python3 -X utf8 py_tools/render_placement.py {_frozen} \\
      --clearance <the board's own floor> --ignore-nets <the poured nets> \\
      --json-out {_hoj} -o {_hop}

Its WHAT THIS PANEL SHOWS block is what routing is being given. Anything it
names as off the outline, stacked, or hole-conflicting will still be there after
the route, and no router setting removes it -- so if that list is not empty,
read this stage's refusals again before spending a routing pass on it.

Next, on success: --stage L5. On a failure: --stage L3 --score <score json>
         --render-json <a --focus render; L3 will not open without one>
</stage_instructions>'''


def l3(a):
    """Classify. The whole loop turns on this."""
    score, e = _load(a.score, 'The routing score (--score)')
    if e:
        return err(e + '\n\nA retry without a classification is a guess, and '
                       'the three shapes re-enter at three different points. '
                       'Produce the score first:\n  python3 -X utf8 '
                       '.claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py '
                       '<board> --json wk/score.json')
    blocking = score.get('blocking')
    if blocking is None:
        return err(
            'The score carries no `blocking` count. `blocking == null` is not '
            'zero -- it means something was not graded, and a component '
            'nothing examined is reported UNEXAMINED, never clean. Re-score '
            'the board and read why the count is missing before classifying '
            'anything:\n  python3 -X utf8 '
            '.claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py <board> '
            '--json wk/score.json')
    # Bind the score to the board, exactly as L5 does. L3 named `--board` in
    # every command it emitted and never checked that `--score` described the
    # same file, so a classification could be made about one board from another
    # board's numbers -- and it was: run 15 passed
    # `--board routed.kicad_pcb --score r_R5k_i2_score.json`, two different
    # boards, and this stage accepted it without a word. board_score writes
    # `board_sha` precisely so this cannot happen.
    _sha_bad = _score_board_mismatch(a.board, score)
    if _sha_bad:
        return err(
            f'The score was taken on a DIFFERENT board than --board names.\n\n'
            f'  --board        : {os.path.abspath(a.board)}\n'
            f'  score board_sha: {str(_sha_bad)[:16]}...\n\n'
            f'Classifying board A from board B\'s failures is a guess wearing a '
            f'measurement\'s clothes. Re-score the board you are classifying, '
            f'then come back.')
    if _recorded(a.board, a.ledger) is False:
        return err(
            f'This routed board is not in the ledger.\n\n  board:  {a.board}\n'
            f'  ledger: {a.ledger}\n\nRecord it before classifying. Everything '
            f'that happens after this point reads the ledger and not the '
            f'board: L4 names the routed boards a placement change makes '
            f'stale, L5 decides whether the loop is over from the score '
            f'history, step-back restores from the board store, and the film '
            f'is the ledger read out loud. A run that skipped this recorded '
            f'two placement laps and no routing ones -- the staleness list '
            f'then printed "none recorded" at the one moment it mattered.\n\n'
            f'  python3 -X utf8 py_placer/converge.py record --ledger {a.ledger} \\\n'
            f'      --board {a.board} --kind completion \\\n'
            f'      --lever "<what you changed and why>" \\\n'
            f'      --score-file {a.score} \\\n'
            f'      --argv <the real command, as bare tokens>')
    if blocking == 0:
        return f'''<stage_instructions stage="L3" name="classify" note="nothing to classify">
blocking == 0. There is no failure to classify -- and that is not the same as
being finished.

`blocking` is only the first key of a lexicographic score; `quality`
(vias, copper_mm, segments) orders the boards that already reached 0. So the
question now is not "is it done" but "can either half still improve it", and
L5 answers that from the ledger rather than from an impression.

Confirm nothing is merely UNGRADED first -- a component nothing examined is
reported unexamined, never clean.

(If L5 answers CONTINUE because routing quality is still improving, the lever
lives in the routing half's own convergence loop -- this stage classifies
FAILURES and has nothing to offer a board at blocking == 0, so do not bounce
between here and L5 looking for one.)

Next: --stage L5 --board {a.board} --score {a.score} --ledger {a.ledger}
</stage_instructions>'''
    if not a.render_json:
        return err(
            f'blocking = {blocking}, so there IS a failure to classify -- and '
            f'the classification is the decision this whole loop turns on. Do '
            f'it with the picture, not from the score.\n\n'
            f'"Do the failures share one pocket, or scatter?" is the question '
            f'that separates placement-shaped from parameter-shaped, and it is '
            f'ONE look. A run that skipped it spent eleven iterations on '
            f'clearance grading while five nets sat with no copper at all; the '
            f'panels would have shown that in seconds.\n\n'
            f'  python3 -X utf8 py_tools/render_placement.py {a.board} \\\n'
            f'      --summary-json <the route log JSON_SUMMARY> --focus \\\n'
            f'      --clearance <the board\'s own floor> '
            f'--ignore-nets <the poured nets> \\\n'
            f'      --json-out wk/focus.json -o wk/focus/\n\n'
            f'  (no route summary to hand it? --focus now clusters the '
            f'LEGALITY findings instead, which is the only form of the '
            f'question a copper-free board can answer.)\n\n'
            f'READ the panels, then re-run this stage with '
            f'--render-json wk/focus.json.\n\n'
            f'The render prints WHAT THIS PANEL SHOWS and THE WORST N with a '
            f'crop command each; one pocket means a local fix, scattered means '
            f'systemic, and that is the classification below.')
    # A path that exists is not a render of THIS board, made from the route
    # log, on a board whose parts stayed put. L2 and L5 both shape-test and
    # bind their documents; the stage that makes the classification took a
    # filename and asked nothing else of it.
    _rok, _rwhy = _guard_route_render(a)
    if not _rok:
        return err(_rwhy)
    return f'''<stage_instructions stage="L3" name="classify the failure" of="5">
blocking = {blocking}. Name the SHAPE before choosing anything.

You have the focus panels ({a.render_json}). Say in one line what they showed --
one pocket or scattered -- and let that lead the evidence below, rather than
being reverse-engineered to agree with it.

All three shapes look identical from inside the router -- nets that did not
route. What separates them is a measurement, and the cost of guessing is
asymmetric.

  parameter-shaped   grid, rip-up depth, layer costs, width. The router had a
                     path and did not take it.
                     Evidence: lane supply at the escape faces, congestion near
                     the pre-placement baseline, and a diagnosis naming a knob.

  placement-shaped   a starved escape face, a part its net cannot reach, OR an
                     arrangement too tangled to route as a whole. No router
                     setting adds a lane or removes a crossing.
                     Evidence: check_channels shows a face carrying demand with
                     zero supply; check_reachability says CAGED on the
                     COPPER-FREE board; or crossings sit far above the board
                     placement started from.

  floorplan-shaped   a clause no arrangement at this placement satisfies.
                     Evidence: the constraint survives every parameter and
                     every local move.

Measure it, do not infer it from how the failure feels:

  python3 -X utf8 py_tools/check_channels.py {a.board} --baseline <the pre-route board> \\
      --clearance <the board's own floor> --track-width <the board's own floor>
  python3 -X utf8 py_tools/check_reachability.py <the COPPER-FREE board> --pad <REF.PAD> --json
  python3 -X utf8 py_tools/render_placement.py <the COPPER-FREE board> --json-out wk/cong_now.json
  python3 -X utf8 py_tools/render_placement.py <the pre-placement board> --json-out wk/cong_base.json

Three things that have each cost a run:

  * PASS check_channels the board's OWN track width. At its 0.3 default it
    reported a face "short 1 lane" that at the board's real 0.2 had supply 14
    for demand 12 -- an invented deficit, handed on as floorplan-shaped residue.

  * Run check_reachability on the COPPER-FREE board. On a routed board its
    geometry includes the ROUTER'S OWN copper, so CAGED there means rip-up
    depth, not placement. Measured: three genuine CAGED pads, all three
    PASSABLE once the copper was removed. Read the EXIT CODE, not the JSON
    `verdict` alone -- rc=2 is NO-TARGET, "the question was not answerable",
    and it is not a CAGED.

  * Both those tests are PER-NET, and simultaneous routability is not. A board
    can pass every one of them and still be unroutable. That is why `parameter`
    -- the answer a blind classifier gives by default -- is the verdict L4 makes
    you evidence: --congestion-json plus --congestion-baseline.

check_reachability answers about ONE pad per run -- give it a pad on a failing
net (or --net <id> --at <x,y>). A single genuine CAGED (exit 1) on the
copper-free board is enough to make the shape placement.

Then re-run this stage with the shape you measured:
  --stage L4 --shape <parameter|placement|floorplan> --board {a.board} \\
      --score {a.score} --ledger {a.ledger} \\
      [--congestion-json wk/cong_now.json --congestion-baseline wk/cong_base.json]
</stage_instructions>'''


def l4(a):
    """Re-enter, at the point the classification names."""
    if a.shape not in SHAPES:
        return err(
            'A re-entry needs a measured shape: --shape parameter | placement '
            '| floorplan.\n\nThis is the guard that matters most in this '
            'driver. Retrying a placement-shaped failure with router '
            'parameters spends iterations on a board no parameter can fix; '
            'sending a parameter-shaped failure back to placement throws away '
            'a routed board for nothing. L3 tells you how to measure it.')
    if a.shape == 'parameter':
        # Refuses only when the congestion evidence is MISSING. The numbers
        # themselves report and never refuse -- see _guard_congestion.
        _cok, _cwhy = _guard_congestion(a)
        if not _cok:
            return err(_cwhy)
        _cread = f'\n{_cwhy}\n' if _cwhy else ''
        return f'''<stage_instructions stage="L4" name="re-enter: parameter" of="5">
{_cread}
Re-enter the FAILING ROUTING STEP with the parameter changed. Nothing before it
is invalidated, and the routed board stands.

  Re-run THAT step from /plan-pcb-routing on {a.board} -- the failing step
  only, not the chain from the top.

Change ONE parameter. An iteration that changes three cannot tell you which one
worked, and the ledger entry it writes is unusable as evidence for the next.

Write to a FRESH output path -- board_R5_i2, not board_R5 again. The routing
steps write a sibling .kicad_pro carrying the DRC floor, and re-running to the
same path reads that floor back in, so the second run is not the first run with
one parameter changed: it starts from a different floor. The comparison the
whole iteration exists to make is then between two things that differ in two
ways. It also keeps the ledger's recorded sha pointing at a file that still
holds what was recorded.

Record it, then go back to L3 with the new score. If two parameter iterations
in a row do not move `blocking`, the shape was probably not parameter --
re-measure rather than trying a third.

Next: --stage L3 --board {a.board} --score <new score>
</stage_instructions>'''
    if a.shape == 'floorplan':
        return f'''<stage_instructions stage="L4" name="re-enter: floorplan" of="5">
No arrangement at this placement satisfies the clause, so neither a router
parameter nor a local repair will reach it. Go for a different ARRANGEMENT:

  python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/placement_driver.py \\
      --stage P5 --board <the pre-route placed board>

State the clause as intent first, so the slate can be graded against it rather
than eyeballed.

Every routed board from the old placement is stale. Say so in the ledger before
you start, and start the chain again at L1.

Next: --stage L1 --board <the adopted arrangement> --ledger {a.ledger}
</stage_instructions>'''
    # placement-shaped: the expensive one
    rows = _ledger_rows(a.ledger)
    routed = [r for r in rows if (r.get('kind') or '') in ('completion', 'routing')]
    if routed:
        stale = '\n'.join(f'    - iteration {r.get("iteration")}: '
                          f'{(r.get("result_sha") or "")[:12]}'
                          for r in routed[-8:])
    else:
        # An empty list here is not reassurance. Reaching L4 means a route ran
        # and failed, so a ledger with no routed row means the routing was
        # never recorded -- and this stage's whole job is naming the boards
        # that must not be reused. Say that, rather than printing a blank that
        # reads as "nothing to invalidate".
        stale = (
            '    NONE RECORDED -- and that is a problem, not a clean bill.\n'
            '    You are here because a route failed, so a routed board\n'
            '    exists somewhere. It is not in this ledger, so nothing can\n'
            '    tell the next pass which files are now stale. Find every\n'
            '    board produced from this placement and treat ALL of them as\n'
            '    stale, then record the routing result before the next\n'
            '    iteration so this list works next time:\n'
            '      python3 -X utf8 py_placer/converge.py record --ledger '
            f'{a.ledger} \\\n'
            '          --board <the routed board> --kind completion \\\n'
            '          --score-file <the score json> --argv <the command>')
    return f'''<stage_instructions stage="L4" name="re-enter: placement" of="5">
This is the expensive one, and the cost is the point: no router setting adds a
lane, so every routed board produced from this placement is now stale.

Routed boards this ledger recorded, which must not be reused:
{stale}

1. Record the classification in the ledger BEFORE moving, with the measurement
   that produced it -- otherwise the next pass cannot tell why the placement
   changed and will re-derive it wrongly.
2. Re-enter the placement half at the rung the damage calls for:
       P2  a part whose position is a mechanical fact is wrong
       P3  the structure is wrong (a swap, a drag, a bad import)
       P4  local violations only
   python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/placement_driver.py \\
       --stage <P2|P3|P4> --board <the placed board> --before <its input>
3. Then restart the chain at L1. Do not resume routing from a stale board, and
   do not carry a routed net list across: the nets are the same, the corridors
   are not.

Before you restart, SEE what the re-entry changed. Otherwise the next lap begins
on a board nobody looked at, and "the loop turned" becomes the only record that
anything happened:

  python3 -X utf8 py_tools/render_placement.py <the re-placed board> \\
      --before <the board it came from> --pair \\
      --clearance <the board's own floor> --ignore-nets <the poured nets> \\
      --json-out wk/reenter.json -o wk/reenter.png

Read WHAT THE MOVE DID. A re-entry that resolves nothing and introduces findings
is a lap to revert, and `N fixed, M NEW` says which -- a level count does not.
Attach it: converge.py record ... --render-json wk/reenter.json

Next: --stage L1 --board <the re-placed board> --ledger {a.ledger}
</stage_instructions>'''


def _verdict(a):
    """Ask converge for the stop verdict. Returns (name, doc, exit) or None."""
    if not a.score or not os.path.isfile(a.score):
        return None
    # L5 interpolated `a.board` into every command it emits and NEVER checked
    # it. A typo, or a stale path a later lap overwrote, produced a
    # `DONE-EXHAUSTED` close-out naming a board nobody graded -- and the
    # close-out is the run's terminal artifact. board_score already writes a
    # `board_sha` with the same digest board_store uses, so binding them is
    # free.
    if a.board and not os.path.isfile(a.board):
        return ('NO-BOARD', {'board': a.board}, 3)
    try:
        with open(a.score, encoding='utf-8') as _sf:
            _payload = json.load(_sf)
    except Exception:                                           # noqa: BLE001
        _payload = None
    if isinstance(_payload, dict) and a.board and os.path.isfile(a.board):
        _psha = _payload.get('board_sha')
        if _psha:
            try:
                sys.path.insert(0, ROOT)
                from board_store import sha256_file
                if sha256_file(a.board) != _psha:
                    return ('SCORE-MISMATCH',
                            {'board': a.board, 'payload_sha': _psha}, 3)
            except Exception:                                   # noqa: BLE001
                pass
    import subprocess
    # ABSPATH BOTH FILES: this subprocess runs with cwd=ROOT while every
    # isfile() check above resolved against the CALLER's cwd -- so from any
    # other directory, L5 could validate one file and converge could read
    # another (a missing ledger reads as empty history, which is a confident
    # CONTINUE derived from nothing). Same fix shape as check_complete's
    # relative-path repair; run-17 audit, D4.
    p = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_placer', 'converge.py'),
         'verdict', '--ledger', os.path.abspath(a.ledger),
         '--score', os.path.abspath(a.score),
         '--budget', str(a.budget), '--flat', str(a.flat)],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT)
    try:
        doc = json.loads(p.stdout)
    except Exception:                                       # noqa: BLE001
        return None
    return doc.get('verdict'), doc, p.returncode


def final_record_command(ledger, board, score, name):
    """The run-closing record, EXACTLY as L5 prints it.

    A separate function so a test can EXECUTE the printed command
    (tests/test_converge.py substitutes the placeholders and runs it). The D1
    finding: this stage printed a command its own converge refused as written
    -- no --lens slots at all, and cmd_record's FAIL-lens gate then took only
    the numeric stop vocabulary, so the STUCK/BUDGET paths (where a FAIL lens
    is the normal case) were refused on the interpolated verdict name.
    """
    return (
        f'python3 -X utf8 py_placer/converge.py record --ledger {ledger} --board {board} \\\n'
        f'      --kind completion --final --stop-condition "{name}" \\\n'
        f"      --lens '<the connectivity VERDICT= line, verbatim>' \\\n"
        f"      --lens '<the drc VERDICT= line, verbatim>' \\\n"
        f"      --lens '<the spec VERDICT= line, verbatim>' \\\n"
        f'      --score-file {score} --argv <the command that produced this board>')


def l5(a):
    """Close out -- but only if the loop is actually finished.

    This stage used to print a checklist and end, which made it the place a run
    stopped rather than the place a run was MEASURED to be over. Reaching
    `blocking == 0` is the floor: the score is lexicographic and `quality`
    orders the boards that already got there, so a solved board whose quality
    is still improving has to go round again. And a run that stopped because it
    was finished must not look like one that stopped because it was stuck.
    """
    got = _verdict(a)
    if got and got[0] == 'NO-BOARD':
        return err(
            f'L5 was asked to close out a board that does not exist:\n\n'
            f'  {os.path.abspath(got[1]["board"])}\n\n'
            f'This stage names the board in every command it emits and used to '
            f'validate none of them, so a typo or a path a later lap overwrote '
            f'produced a DONE close-out for a file nobody graded -- in the '
            f'run\'s terminal artifact. Point --board at the board you are '
            f'closing out.')
    if got and got[0] == 'SCORE-MISMATCH':
        return err(
            f'The score was taken on a DIFFERENT board than --board names.\n\n'
            f'  --board       : {os.path.abspath(got[1]["board"])}\n'
            f'  score board_sha: {str(got[1]["payload_sha"])[:16]}...\n\n'
            f'board_score embeds the sha of the board it graded precisely so '
            f'this cannot go unnoticed. Re-score the board you are closing '
            f'out, then come back.')
    if got is None:
        return err(
            'L5 decides whether the loop is OVER, and that is a measurement, '
            'not a judgement call. Score the board and come back:\n'
            '  python3 -X utf8 '
            '.claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py '
            f'{a.board} --json wk/score_final.json\n'
            f'  python3 -X utf8 {sys.argv[0]} --stage L5 --board {a.board} \\\n'
            f'      --ledger {a.ledger} --score wk/score_final.json\n\n'
            'Every stop RULE in this toolchain was already written down and '
            'none of them had a mechanism, so "done" and "stuck" came out '
            'looking identical. This is that mechanism.')
    # AN ABSENT LEDGER POISONS THE VERDICT ITSELF, so this refuses before the
    # branch rather than inside the terminal one. `_verdict` reads the ledger
    # to tell "finished" from "stuck"; with no file it sees no history and
    # answers CONTINUE -- an ungated branch. So a run that simply never created
    # the ledger got a confident "go round again" derived from nothing, and
    # every gate that checks recording was skipped on the way. That is the
    # run-13 shape at its purest: not a lap recorded wrong, a spine that was
    # never there.
    #
    # L2 and L3 deliberately still pass in this case -- `_recorded` answers
    # None and their comments explain why (a board placed elsewhere has no
    # placement lap and never will). By L5 that reasoning is spent: the routing
    # half was asked in writing to record every accepted iteration.
    if not os.path.isfile(a.ledger or ''):
        return err(
            f'There is no ledger at {a.ledger or "<none given>"}, and L5 '
            f'decides whether the loop is over BY READING IT.\n\nWith no file '
            f'there is no history, so the measurement below would not be a '
            f'measurement -- it would be this stage reporting that a run which '
            f'recorded nothing has nothing left to improve.\n\nL2 and L3 let a '
            f'missing ledger pass on purpose: a board placed elsewhere has no '
            f'placement lap to record. That does not apply here -- the routing '
            f'half was asked to record every accepted iteration, and every '
            f're-entry, the film and step-back all read this file.\n\n'
            f'  python3 -X utf8 py_placer/converge.py record --ledger {a.ledger} \\\n'
            f'      --board {a.board} --kind completion \\\n'
            f'      --argv <the command that produced it>')
    name, doc, _code = got
    why = doc.get('reason', '')

    # A score that EXISTS but cannot be read is not a stop verdict -- it is a
    # missing measurement. This used to fall through to the terminal branch,
    # so an unparseable score file printed the full ship ceremony (including
    # `--final --stop-condition "NO-SCORE"`) instead of "re-score". Run-17
    # audit, D9.
    if name == 'NO-SCORE':
        return err(
            f'The score at {a.score} could not be read ({why or "unparseable"}), '
            f'and L5 decides whether the loop is over FROM the score. An '
            f'unreadable measurement is not a stop condition. Re-score the '
            f'board, then come back:\n'
            f'  python3 -X utf8 '
            f'.claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py '
            f'{a.board} --json wk/score_final.json\n'
            f'  python3 -X utf8 {sys.argv[0]} --stage L5 --board {a.board} \\\n'
            f'      --ledger {a.ledger} --score wk/score_final.json')

    if name == 'CONTINUE':
        # THE CROSS-CHECK RUNS HERE TOO. It is still not a REQUIREMENT on this
        # branch -- no close-out is demanded, and a run with neither a final
        # row nor a close-out passes untouched -- but if either exists, the
        # contradiction is checkable and this is the stage documented to catch
        # it. The measured run wrote its false `--final` row and then kept
        # going, so every visit to L5 took this branch and the gate sat idle.
        _x = _cross_check(a, name, _peek_close(a))
        if _x:
            return _x
        halves = ', '.join(doc.get('improving') or ['a half'])
        return f'''<stage_instructions stage="L5" name="not done yet" of="5">
The loop is NOT over: {halves} is still improving.

{why}

Go round again. A board that merely routes is the floor -- keep pulling levers
until neither half can improve either key.

  placement still improving -> --stage L1 --board {a.board} --ledger {a.ledger}
  routing still improving   -> the lever is pulled INSIDE the routing half
                               (its own convergence stages -- quality passes,
                               via count, copper length), not by this driver:
                               re-enter it on the routed board, record the
                               lap, then --stage L3 --board {a.board} \\
                                   --score <the new score> --ledger {a.ledger}

Record the lap you are about to run, accepted or rejected. A rejected lap is
data: it is what makes the plateau detectable, and without it this stage cannot
tell a finished run from a stalled one.
</stage_instructions>'''

    # The terminal branches are where the run SHIPS, so this is where the
    # routing half has to have closed out. Not CONTINUE: that is the go-round-
    # again branch, and gating it would put the most expensive instrument in
    # the repo into the loop that runs most often, to say nothing the score has
    # not already said.
    _refusal = _close_out(a, name)
    if _refusal:
        return _refusal

    verdicts = {
        'DONE-EXHAUSTED': 'the board is done, and measured to be done',
        'STUCK': 'stopping is legitimate; calling this finished is not',
        'BUDGET': 'the budget ended this run, not the board',
    }
    return f'''<stage_instructions stage="L5" name="close out: {name}" of="5">
{verdicts.get(name, name)}.

{why}

Confirm with the instruments, and put the numbers in the report beside the
names of the instruments that produced them:

  python3 -X utf8 check_complete.py {a.board} --clearance <floor> \\
      --authored-from <the board this chain STARTED from>
  python3 -X utf8 py_router/check_drc.py {a.board} --clearance <floor> --clearance-margin 0.1
  python3 -X utf8 py_router/check_connected.py {a.board}
  python3 -X utf8 py_tools/check_assembly.py {a.board}

check_complete is the one that fails CLOSED: board_score exits 0 with four of
nine components ungraded, and it has no component at all for orphan stubs,
weird copper or pad overlaps. --authored-from is what lets it see whether this
board grades itself against floors it rewrote -- the writeback only ever
loosens, so without the original project there is nothing left to compare to.

Connectivity is orthogonal to DRC: a DRC-clean board can be entirely
disconnected, because isolated copper has no clearance conflicts.

Close the ledger with the stop condition NAMED and the three routed-board
lenses ATTACHED -- converge refuses --final without them, deliberately. The
VERDICT= lines come from the routed-board lens verifiers
(references/verifier-prompts.md; the routing half dispatches them at its
close-out) -- paste each line verbatim, FAIL included. A FAIL is compatible
with STUCK and BUDGET; it is only DONE-EXHAUSTED that no failing lens may
sit beside.

  {final_record_command(a.ledger, a.board, a.score, name)}

Then render the run. It is the only artifact that shows HOW the board got here,
and because both halves recorded into one ledger it is ONE film, not two:

  python3 -X utf8 py_tools/make_film.py --from-ledger {a.ledger} -o wk/run.mp4

And take the ONE still that shows whether the run made progress: the board this
run started from against the board it is ending with, at identical instrument
settings, diffed BY NAME.

  python3 -X utf8 py_tools/render_placement.py {a.board} \\
      --before <the board this RUN started from> --pair \\
      --clearance <the board's own floor> --ignore-nets <the poured nets> \\
      --json-out wk/run_pair.json -o wk/run_pair.png

Its WHAT THE MOVE DID block is the run's verdict in one place -- `N fixed, M
NEW` per finding class, plus crossings/hpwl/overlap before and after. Put those
numbers in the report. Two reasons they are not optional:

  * a level count hides a swap. "46 body stacks then 46 body stacks" can be
    nine resolved and nine created somewhere else, and only the by-name diff
    shows it -- the same reason the ledger records failing nets by name.
  * the discrete findings and the aggregate can disagree. On one run every
    finding class improved while `overlap_area` got WORSE (237.50 -> 239.02);
    a single scalar picks one and hides the other, and a report built on the
    flattering one is the failure this stage exists to prevent.

Report, per half, the number and the instrument beside it; say how many times
the loop turned and why each turn happened; and name anything UNEXAMINED rather
than reporting it clean. A chain that re-entered placement twice is not a
failure -- an unexplained one is.

<subagent_prompt agent="claude" description="verify the finished board">
Verify this board end to end, independently.

  board:  {a.board}
  ledger: {a.ledger}

Read .claude/skills/plan-pcb-placement-and-routing/references/verifier-prompts.md and apply
its routed-board lenses, then check the PLACEMENT half too: the copper-free
gate cannot be re-run on a routed board, so verify it from the ledger's
recorded placement close-out and confirm the poses still match the board.

Re-derive every number yourself. Do not trust the report.
Answer with a line beginning VERDICT= and nothing above it.
</subagent_prompt>
</stage_instructions>'''


#: The four measurements L2 reads out of the placement close-out. It is also
#: the SHAPE TEST -- these four exist in `check_assembly.py`'s JSON and in no
#: other report this chain produces -- and the vocabulary of --accept-residue.
L2_CHECKS = ('buildable', 'verdict', 'locked_contacts', 'blocking',
             'oob_pad_count')


def _accept(a, check: str) -> bool:
    """Is THIS check's residue accepted?

    One blanket flag used to waive all four at once, so clearing a SPURIOUS
    `blocking` refusal -- one raised by the wrong document being handed in --
    silently also waived the `oob_pad_count` gate, which on that board was the
    only load-bearing check of the four (21 parts with pad copper off the
    board). A bare --accept-residue is refused upstream; only named checks
    waive anything."""
    names = getattr(a, 'accept_residue', None)
    return bool(names) and check in names


#: The routing close-out, produced by `check_complete.py --json`.
#: `verdict` alone is not a shape test -- check_assembly's report carries a
#: `verdict` too -- so the value vocabulary is tested as well. That option was
#: not available to L2 (its `blocking` collided with no vocabulary at all),
#: which is why that gate has to lean on key presence.
CLOSE_KEYS = ('board', 'score', 'components', 'fab_floors', 'verdict',
              'reason', 'ungraded')
CLOSE_VERDICTS = ('DONE', 'INCOMPLETE', 'UNSOUND')

#: The vocabulary of --accept-unclosed. Deliberately NOT --accept-residue's:
#: `_accept` reads one namespace attribute, so sharing it would mean an
#: `--accept-residue blocking` passed for the PLACEMENT gate silently waived a
#: ROUTING check -- the run-10 compounding hazard rebuilt across gates instead
#: of within one. `shape` and `binding` are absent on purpose: a malformed or
#: mis-bound document is the wrong document, and there is nothing to accept.
CLOSE_CHECKS = ('instruments', 'fab_floors', 'ungraded', 'agreement')


def _accept_close(a, check: str) -> bool:
    """Is THIS close-out check accepted? (L5's routing gate.)"""
    names = getattr(a, 'accept_unclosed', None)
    return bool(names) and check in names


def _peek_close(a):
    """The --routing-close document if one was given and parses, else None.

    Deliberately silent about a missing or malformed one: `_close_out` owns
    those refusals, and the CONTINUE branch must not acquire them (gating the
    go-round-again branch on the most expensive instrument in the repo is the
    one thing that branch is documented NOT to do).
    """
    doc, e = _load(getattr(a, 'routing_close', None), 'x')
    return None if e else doc


def _cross_check(a, name, doc):
    """Refusal when two independent instruments disagree -- or None.

    L5 is documented as the stage that refuses a contradiction, and it only
    ever COMPARED on one path: `name == 'DONE-EXHAUSTED' and close-out verdict
    != DONE`. Measured: in the run that wrote a `--final` row carrying
    VERDICT=PASS:lens=connectivity on 32 unrouted nets and 47 broken joins,
    converge never reached DONE-EXHAUSTED -- so the false lens was never tested
    by the gate built to test it, and it walked L3, L4 and L5.

    So the comparison now runs whenever there is anything to compare: a
    close-out, a `--final` ledger row, or both. Three pairs, each between
    instruments that do not share a code path:

      * a final row's PASS lens against the score IN THAT SAME ROW;
      * a final row claiming every lens passed against a close-out that says
        INCOMPLETE or UNSOUND;
      * converge's DONE-EXHAUSTED against a close-out that is not DONE.
    """
    if _accept_close(a, 'agreement'):
        return None
    rows = _ledger_rows(getattr(a, 'ledger', None))
    # SUPERSESSION IS PER LENS. The ledger is append-only, so a run that wrote
    # a wrong close-out and then wrote the correction has both on file, and
    # checking every final row makes the CORRECTION unable to clear the gate,
    # permanently: on the real ledger iteration 21 carries the false PASS and
    # iteration 22 is its signed retraction ("the earlier PASS answered a
    # mis-scoped prompt"), so that run could never reach L5 on any branch.
    #
    # But taking only the LAST final row is too coarse the other way: any later
    # final row buries the false claim, and a one-lens `--kind systemic --final`
    # row is trivially writable (the connectivity/drc/spec requirement only
    # binds `--kind completion`). Measured: a later row carrying nothing but
    # `VERDICT=PASS:lens=spec` cleared a live false PASS on connectivity.
    #
    # So: for each LENS NAME, the live claim is the most recent final row that
    # says anything about THAT lens, judged against that row's own score. A
    # retraction supersedes; an unrelated row does not.
    finals = [r for r in rows if r.get('final')]
    live = {}
    for r in finals:
        for raw in (r.get('lenses') or []):
            m = re.match(r'^VERDICT=(?:PASS|FAIL):lens=([A-Za-z0-9_-]+)',
                         str(raw).strip())
            if m:
                live[m.group(1).lower()] = (r, str(raw))
    pairs = []
    try:
        sys.path.insert(0, ROOT)
        from converge import lens_contradictions
    except Exception:                                       # noqa: BLE001
        lens_contradictions = None
    for _name, (r, raw) in (sorted(live.items()) if lens_contradictions else ()):
        for lens, comp, n in lens_contradictions([raw], r.get('score')):
            pairs.append((
                f'ledger iteration {r.get("iteration")} (--final)',
                f'VERDICT=PASS:lens={lens}',
                f'its OWN score in the same row: {comp} = {n:g}'))

    _cv = (doc or {}).get('verdict') if isinstance(doc, dict) else None
    # ...and the LIVE lens set -- one verdict per lens name, latest wins --
    # against a close-out that says the board is not done. Reading every final
    # row here would flag a superseded claim; reading only the last row would
    # miss a lens that row never mentioned.
    if _cv in ('INCOMPLETE', 'UNSOUND') and live:
        _fails = [n for n, (_r, raw) in live.items()
                  if raw.strip().startswith('VERDICT=FAIL')]
        if not _fails:
            _its = sorted({str(r.get('iteration')) for r, _raw in live.values()})
            pairs.append((
                f'ledger --final row(s) {", ".join(_its)}',
                f'{len(live)} live lens verdict(s), all PASS: '
                f'{", ".join(sorted(live))}',
                f'check_complete: {_cv} -- {doc.get("reason", "")}'))
    if name == 'DONE-EXHAUSTED' and isinstance(doc, dict) and _cv != 'DONE':
        pairs.append(('converge verdict',
                      'DONE-EXHAUSTED (blocking == 0, and a plateau)',
                      f'check_complete: {_cv} -- {doc.get("reason", "")}'))
    if not pairs:
        return None
    body = '\n\n'.join(f'  {who}\n    claims : {claim}\n    against: {other}'
                       for who, claim, other in pairs)
    return err(
        f'TWO INSTRUMENTS DISAGREE, and this is the one place that must not be '
        f'waved through.\n\n{body}\n\nEvery pair above is between instruments '
        f'that do not share a code path, which is the whole point: converge '
        f'reaches its verdict from `blocking == 0` plus a plateau, and '
        f'`blocking == 0` is exactly the number check_complete exists to '
        f'distrust -- it has no component for orphan stubs, weird copper or '
        f'cross-footprint pad overlaps. A lens verdict is a third, independent '
        f'reading, and a PASS beside a count that refutes it is not evidence '
        f'about the board, it is evidence about the verifier.\n\nThis gate used '
        f'to compare ONLY on the DONE path, so in the run that shipped a false '
        f'`PASS:lens=connectivity` on 32 unrouted nets it never ran at all.\n\n'
        f'Fix what the close-out names and re-score, re-dispatch the lens that '
        f'disagrees, or --accept-unclosed agreement and say in the report which '
        f'instrument you are overriding and why.')


def _close_out(a, name):
    """Refusal string, or None when the close-out clears L5.

    The asymmetry this exists to remove: L2 refuses to START routing without a
    placement close-out, while nothing ever refused to FINISH. A run reached
    the terminal artifact having never invoked the routing half's own V1-V5 at
    all, and shipped a board carrying a power-rail-to-signal short.

    The gate is NOT "produce a document" -- a well-shaped empty one would
    satisfy that. It is that TWO INDEPENDENT INSTRUMENTS MUST NOT CONTRADICT
    EACH OTHER. converge reaches DONE-EXHAUSTED from `blocking == 0` plus a
    plateau, and `blocking == 0` is exactly the number check_complete exists to
    distrust: it has no component at all for orphan stubs, weird copper or
    cross-footprint pad overlaps, so a short can live in the gap between them.
    The contradiction is the refusal.
    """
    _res = getattr(a, 'accept_unclosed', None)
    if _res is not None:
        _bad = [n for n in _res if n not in CLOSE_CHECKS]
        if not _res or _bad:
            return err(
                f'--accept-unclosed names WHICH close-out check you are '
                f'accepting. Valid: {" ".join(CLOSE_CHECKS)}.'
                + (f' Unknown: {" ".join(_bad)}.' if _bad else
                   ' A bare flag would waive all of them at once, which is how '
                   'a spurious refusal also waives the one check that was '
                   'load-bearing.'))

    # THE SUCCESS PATH SKIPS L3. On a clean route the loop goes L2 -> L5
    # directly, so `_recorded` -- the only check that the routing half actually
    # wrote laps -- never ran on the board being shipped. That is the run-13
    # failure (two placement laps, zero routing ones, nothing noticed)
    # reappearing on the path where nothing looks wrong.
    #
    # AND AN ABSENT LEDGER REFUSES *HERE*, unlike at L2 and L3. `_recorded`
    # returns None for "no ledger file at all", and those two stages must not
    # turn that into a refusal -- a board somebody else placed, handed straight
    # to routing, has no placement lap to record and never will. By the ship
    # gate that reasoning is spent: the routing half was told in writing to
    # record every accepted iteration, so a ledger that does not exist is not
    # an unanswerable question, it is the answer. Without this, the run-13
    # shape survives in its purest form -- never create the ledger, and every
    # gate that checks it degrades to silence.
    if not os.path.isfile(a.ledger or ''):
        return err(
            f'There is no ledger at {a.ledger}. This is the board about to '
            f'ship, and nothing recorded how it got here.\n\nL2 and L3 let a '
            f'missing ledger pass, deliberately: a board placed elsewhere has '
            f'no placement lap to record. That does not apply now -- the '
            f'routing half was asked to record every accepted iteration, so an '
            f'absent ledger means either none were recorded or they went '
            f'somewhere else. The film, step-back and every re-entry read this '
            f'file; without it the run cannot be reproduced or reverted.\n\n'
            f'  python3 -X utf8 py_placer/converge.py record --ledger {a.ledger} \\\n'
            f'      --board {a.board} --kind completion \\\n'
            f'      --argv <the command that produced it>')
    if _recorded(a.board, a.ledger) is False:
        return err(
            f'{a.board} is not in {a.ledger}: no lap records a board with this '
            f'content hash.\n\nThis is the board about to ship. The ledger is '
            f'what the film, step-back and every re-entry read, and a board '
            f'that never entered it cannot be reproduced or reverted to. A '
            f'clean route is exactly the case where this goes unnoticed, '
            f'because nothing else complains.\n\n  python3 -X utf8 py_placer/converge.py '
            f'record --ledger {a.ledger} \\\n      --board {a.board} --kind '
            f'completion --argv <the command that produced it>')

    doc, e = _load(getattr(a, 'routing_close', None),
                   'The routing close-out (--routing-close)')
    if e:
        return err(
            e + f'\n\nL5 is where the run ships, so it is where the routing '
                f'half has to have closed out. Produce it:\n\n'
                f'  python3 -X utf8 check_complete.py {a.board} \\\n'
                f'      --authored-from <the board this chain STARTED from> \\\n'
                f'      --json wk/routing_close.json\n\n'
                f'--authored-from is not optional bookkeeping: without it the '
                f'floor check cannot run at all, and UNSOUND becomes '
                f'unreachable. The chain rewrites the floors in place, so the '
                f'original project is the only thing left to compare against.')

    _missing = [k for k in CLOSE_KEYS if k not in doc]
    if _missing or doc.get('verdict') not in CLOSE_VERDICTS:
        _hint = ''
        if doc.get('kind') == 'board-score' or 'blocking_by' in doc:
            _hint = ("\n\nThis looks like `board_score.py`'s JSON. That is the "
                     "document check_complete WRAPS and distrusts -- it exits 0 "
                     "with four of nine components ungraded and has no "
                     "component for orphan stubs, weird copper or pad overlaps.")
        elif 'ledger_rows' in doc:
            _hint = ("\n\nThis looks like `converge.py verdict`'s JSON. That is "
                     "the OTHER half of this gate: it never opens the board, so "
                     "it cannot see the things this check is for.")
        return err(
            f'The routing close-out is not shaped like a `check_complete.py` '
            f'report'
            + (f': it is missing {", ".join(_missing)}.' if _missing else
               f': `verdict` is {doc.get("verdict")!r}, not one of '
               f'{" / ".join(CLOSE_VERDICTS)}.')
            + f' A missing key is not a passing one.{_hint}\n\nProduce the '
              f'right document:\n  python3 -X utf8 check_complete.py {a.board} '
              f'--authored-from <original> --json wk/routing_close.json')

    # Bind by CONTENT. A path comparison accepts a close-out for a board that
    # has since been rewritten, and the close-out is the terminal artifact.
    _dsha = doc.get('board_sha')
    if _dsha and a.board and os.path.isfile(a.board):
        try:
            sys.path.insert(0, ROOT)
            from board_store import sha256_file
            if sha256_file(a.board) != _dsha:
                return err(
                    f'The close-out grades a DIFFERENT board than the one being '
                    f'closed out:\n\n  --board          : '
                    f'{os.path.abspath(a.board)}\n'
                    f'  close-out board_sha: {str(_dsha)[:16]}...\n\n'
                    f'Re-run check_complete on the board you are shipping.')
        except Exception:                                       # noqa: BLE001
            pass
    elif doc.get('board') and a.board:
        if os.path.normcase(os.path.abspath(doc['board'])) != \
                os.path.normcase(os.path.abspath(a.board)):
            return err(
                f'The close-out names a different board:\n\n'
                f'  close-out : {doc["board"]}\n'
                f'  --board   : {os.path.abspath(a.board)}')

    # `--skip-slow` empties `components`, leaving a document that passes every
    # shape check while carrying LESS than the score already in hand -- the
    # added instruments are the only reason to prefer it.
    if doc.get('components') == {} and not _accept_close(a, 'instruments'):
        return err(
            'The close-out ran with --skip-slow: `components` is empty, so '
            'orphan stubs, weird copper and cross-footprint pad overlaps were '
            'NOT examined. Those are the instruments board_score has no '
            'component for, and they are the reason this document gates '
            'anything.\n\nRe-run without --skip-slow, or '
            '--accept-unclosed instruments to ship with them unexamined.')

    _ff = doc.get('fab_floors') or {}
    if _ff.get('ran') is False and not _accept_close(a, 'fab_floors'):
        return err(
            f'The close-out could not check the fab floors: '
            f'{_ff.get("reason", "no reason given")}.\n\nWithout it UNSOUND is '
            f'unreachable by construction, so a DONE from this document cannot '
            f'distinguish "the copper is right" from "the rule moved". Pass '
            f'--authored-from <the board this chain STARTED from>, or '
            f'--accept-unclosed fab_floors.')

    _ung = doc.get('ungraded') or []
    if _ung and not _accept_close(a, 'ungraded'):
        return err(
            f'{len(_ung)} component(s) were never examined: '
            f'{", ".join(_ung)}.\n\nNothing was asked to grade them, so they '
            f'are UNKNOWN rather than clean, and they contributed nothing to '
            f'`blocking`. Pass the spec flags check_complete needs, or '
            f'--accept-unclosed ungraded to ship with them unexamined.')

    # THE AGREEMENT CHECK. Only fires where instruments disagree:
    # STUCK/BUDGET alongside INCOMPLETE is a CONSISTENT pair and passes. It
    # lives in _cross_check now because it also has to run on the CONTINUE
    # branch -- see there for the run where it never ran at all.
    return _cross_check(a, name, doc)


STAGES = {'L1': l1, 'L2': l2, 'L3': l3, 'L4': l4, 'L5': l5}
TITLES = {'L1': 'place (inline or delegated)',
          'L2': 'freeze what placement decided, then route',
          'L3': 'classify the failure -- the whole loop turns on this',
          'L4': 're-enter at the point the classification names',
          'L5': 'close out, both halves, measured'}


def _args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stage', choices=sorted(STAGES))
    ap.add_argument('--board', default='board.kicad_pcb')
    ap.add_argument('--ledger', default='wk/ledger.jsonl')
    ap.add_argument('--placement-report', default=None)
    ap.add_argument('--score', default=None)
    ap.add_argument('--congestion-json', default=None, metavar='PATH',
                    help='render_placement --json-out of the COPPER-FREE board '
                         'the route ran on, plus (--congestion-baseline) the '
                         'board placement started from. L4 refuses '
                         '--shape parameter without it: every placement test '
                         'the classifier runs is per-NET, so global capacity '
                         'is invisible to all of them.')
    ap.add_argument('--congestion-baseline', default=None, metavar='PATH',
                    help='render_placement --json-out of the pre-placement '
                         'board, to compare --congestion-json against.')
    ap.add_argument('--accept-congestion', default=None, metavar='REASON',
                    help='Proceed with --shape parameter even though the '
                         'congestion read says the failure may be '
                         'placement-shaped. Needs a REASON: the numbers are not '
                         'judged (they cannot be -- see '
                         'docs/placement-calibration.md), so what is being '
                         'recorded is that somebody decided, not that the '
                         'board passed. '
                         'Deliberately NOT part of --accept-residue: that flag '
                         'speaks the L2 placement gate\'s vocabulary, and a '
                         'waiver that covers two gates at once waives the one '
                         'that was working.')
    ap.add_argument('--render-json', default=None, metavar='PATH',
                    help='render_placement --json-out document. L3 requires '
                         'one when there is a failure to classify: "one pocket '
                         'or scattered" is the question that separates '
                         'placement-shaped from parameter-shaped, and it is a '
                         'look, not an inference from the score.')
    ap.add_argument('--shape', default=None, choices=SHAPES)
    ap.add_argument('--budget', type=int, default=100,
                    help='ledger entries this run may write before L5 calls '
                         'it (default 100, the figure convergence.md states)')
    ap.add_argument('--flat', type=int, default=5,
                    help='RECORDED laps -- accepted OR rejected -- a half '
                         'may go without improving before it counts as '
                         'blocked (default 5, ditto). A rejection is the '
                         'evidence that a half tried and did not improve; '
                         'routing accepts only on strict improvement, so '
                         'an exhausted half has no accepted lap to give.')
    ap.add_argument('--no-delegate', action='store_true',
                    help='run both halves in THIS context instead of handing '
                         'them to teammates. The escape hatch: the self-test, '
                         'the parity gates and headless CI need one process. '
                         'Everything else should delegate')
    ap.add_argument('--delegate', action='store_true',
                    help='explicit form of the default (both halves go to a '
                         'teammate). Accepted so existing invocations keep '
                         'working; it changes nothing on its own')
    ap.add_argument('--accept-residue', nargs='*', action='extend',
                    metavar='CHECK', default=None,
                    help='proceed to routing with a NAMED, measured-unfixable '
                         'placement residue -- naming WHICH check you are '
                         'accepting: ' + ' '.join(L2_CHECKS) + '. One blanket '
                         'flag used to waive all of them at once, so clearing '
                         'a spurious `blocking` refusal silently also waived '
                         'the `oob_pad_count` gate that was the load-bearing '
                         'one (run 10). A bare --accept-residue is refused.')
    ap.add_argument('--routing-close', default=None, metavar='PATH',
                    help='the ROUTING close-out, from `check_complete.py '
                         '--json`. L5 refuses without it: L2 refuses to START '
                         'routing without a placement close-out and nothing '
                         'ever refused to FINISH, so a run reached the '
                         'terminal artifact having never entered the routing '
                         "half's own V1-V5 loop at all.")
    ap.add_argument('--accept-unclosed', nargs='*', action='extend',
                    metavar='CHECK', default=None,
                    help='ship with a NAMED close-out check unsatisfied: '
                         + ' '.join(CLOSE_CHECKS) + '. Deliberately separate '
                         'from --accept-residue, which is the placement gate: '
                         'one shared flag would let a waiver granted for '
                         'placement silently waive a routing check. A bare '
                         '--accept-unclosed is refused.')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--dump-all', action='store_true')
    ap.add_argument('--self-test', action='store_true')
    return ap.parse_args(argv)


def main(argv=None):
    a = _args(argv)
    if a.list:
        for k in ('L1', 'L2', 'L3', 'L4', 'L5'):
            print(f'  {k}  {TITLES[k]}')
        return 0
    if a.self_test:
        return _self_test()
    if a.dump_all:
        # The guards must be SATISFIED here. Dumping with files that do not
        # exist prints three refusals and no instructions, which is the
        # opposite of what a dump is for.
        import tempfile
        refused = []
        with tempfile.TemporaryDirectory() as tmp:
            def wrote(name, doc):
                p = os.path.join(tmp, name)
                with open(p, 'w', encoding='utf-8') as fh:
                    json.dump(doc, fh)
                return p
            # A REAL board file, and a close-out carrying every key the L2 gate
            # reads. Both were bare/partial before: the board did not exist (L5
            # now validates it) and the close-out omitted `buildable` /
            # `locked_contacts` / `board` (L2 now reads them). A dump whose
            # fixture cannot satisfy the guards prints refusals, which is the
            # opposite of what a dump is for.
            _bd = os.path.join(tmp, 'b.kicad_pcb')
            open(_bd, 'w', encoding='utf-8').close()
            # ...and a LEDGER carrying that board's sha. L5 now refuses without
            # one, because its verdict is computed from the ledger and an empty
            # history reads as "still improving".
            sys.path.insert(0, ROOT)
            from board_store import sha256_file as _shad
            _ld = os.path.join(tmp, 'ledger.jsonl')
            with open(_ld, 'w', encoding='utf-8') as _fh:
                _fh.write(json.dumps({'iteration': 0, 'kind': 'completion',
                                      'accepted': True,
                                      'result_sha': _shad(_bd)}) + '\n')
            loose = _args(['--board', _bd, '--ledger', _ld,
                           '--score', wrote('s.json', {'blocking': 2}),
                           '--placement-report', wrote(
                               'p.json', {'blocking': 0, 'oob_pad_count': 0,
                                          'buildable': True,
                                          'verdict': 'buildable (blocking 0)',
                                          'locked_contacts': 0,
                                          'board': _bd}),
                           # `summary_json` is load-bearing, not decoration:
                           # _guard_route_render refuses without it, because a
                           # --focus render made with no route summary clusters
                           # the LEGALITY findings instead of the routing
                           # failures. This fixture predated that guard (df9e81d)
                           # and so dumped L3's refusal instead of its
                           # instructions.
                           '--render-json', wrote('rj.json', {
                               'instrument': {'board': _bd,
                                              'summary_json': 'wk/summary.json'},
                               'checklist': {'d_moved': {'match': None}}}),
                           '--shape', 'placement'])
            for k in sorted(STAGES):
                print(f'===== {k} =====')
                body = STAGES[k](loose)
                print(body)
                if body.startswith('<error>'):
                    refused.append(k)
            # Both halves can delegate, and the teammate prompts are where the
            # commands a delegated run will actually execute live -- dumping
            # only the inline forms leaves them unscanned.
            loose.delegate = True
            for k in ('L1', 'L2'):
                print(f'===== {k} (delegated) =====')
                body = STAGES[k](loose)
                print(body)
                if body.startswith('<error>'):
                    refused.append(f'{k}/delegated')
            loose.delegate = False

            # L5 has FOUR outcomes and the dump above shows one of them. The
            # other three carry the commands that close a run out -- the
            # --final record and the film -- so dumping only CONTINUE leaves
            # them unscanned by anything that reads this output. Drive the
            # real verdict with real ledgers rather than faking the branch.
            # `result_sha` is what _close_out's _recorded() matches the board
            # against, and it fires BEFORE the close-out is even loaded. Rows
            # without it are rows no real ledger produces -- the self-test
            # already says exactly that and threads the sha; this fixture was
            # not updated when the guard landed (a25ccf7), so all three
            # terminal L5 branches dumped refusals.
            sys.path.insert(0, ROOT)
            from board_store import sha256_file as _sha_of
            _bdsha = _sha_of(_bd)
            flat = ([{'kind': 'placement', 'accepted': True,
                      'result_sha': _bdsha,
                      'score': {'blocking': 0, 'quality': {}}}] * 6
                    + [{'kind': 'completion', 'accepted': True,
                        'result_sha': _bdsha,
                        'score': {'blocking': 0, 'quality': {}}}] * 6)
            stuck = [dict(r, score={'blocking': 4, 'quality': {}})
                     for r in flat]
            def close_doc(verdict):
                return {'schema': 1, 'kind': 'board-complete', 'board': _bd,
                        'score': {'blocking': 0},
                        'components': {'orphan_stubs': {'ran': True}},
                        'fab_floors': {'ran': True, 'relaxed': []},
                        'verdict': verdict, 'reason': 'fixture',
                        'ungraded': []}

            # Each terminal branch needs its own close-out, and they are NOT
            # all DONE: pairing DONE-EXHAUSTED with DONE and the other two with
            # INCOMPLETE is what makes this dump show the agreement rule is
            # ONE-DIRECTIONAL. A fixture set that is DONE throughout would
            # print identically whether or not the rule existed.
            for label, rows, sc, budget, cv in (
                    ('DONE-EXHAUSTED', flat, {'blocking': 0}, 100, 'DONE'),
                    ('STUCK', stuck, {'blocking': 4}, 100, 'INCOMPLETE'),
                    ('BUDGET', stuck, {'blocking': 4}, 1, 'INCOMPLETE')):
                lp = os.path.join(tmp, f'l_{label}.jsonl')
                with open(lp, 'w', encoding='utf-8') as fh:
                    for i, r in enumerate(rows):
                        fh.write(json.dumps(dict(r, iteration=i)) + '\n')
                v = _args(['--board', _bd, '--ledger', lp,
                           '--budget', str(budget),
                           '--routing-close', wrote(f'c_{label}.json',
                                                    close_doc(cv)),
                           '--score', wrote(f's_{label}.json', sc)])
                print(f'===== L5 ({label}) =====')
                body = STAGES['L5'](v)
                print(body)
                if body.startswith('<error>'):
                    refused.append(f'L5/{label}')
        if refused:
            print(f'\n!! {len(refused)} stage(s) dumped a REFUSAL, not their '
                  f'instructions: {", ".join(refused)}')
            return 1
        return 0
    if not a.stage:
        print('loop_driver: --stage is required (see --list)', file=sys.stderr)
        return 2
    out = STAGES[a.stage](a)
    code = 4 if out.startswith('<error>') else 0
    # BEFORE the print, so a stage that dies printing still leaves its trace,
    # and on stdout NOTHING changes -- existing callers tee exactly what they
    # teed before.
    _log_invocation(a, a.stage, out, code)
    print(out)
    return code


def _self_test():
    import tempfile
    bad = []

    def want(cond, label):
        print(f'  {"PASS" if cond else "FAIL"}  {label}')
        if not cond:
            bad.append(label)

    base = ['--board', 'b.kicad_pcb']
    for key in sorted(STAGES):
        out = STAGES[key](_args(base + ['--score', 'x.json',
                                        '--placement-report', 'p.json',
                                        '--shape', 'placement']))
        want(out.startswith(('<stage_instructions', '<error>')),
             f'{key} emits a tagged block')
        # 74, not 70: run-19 A2 grew FENCE_CLAUSE by four lines (the
        # hand-script disclosure duty). L1 sits exactly AT the cap, as it
        # did at 70 -- any further growth is a deliberate decision, here.
        want(len(out.splitlines()) <= 74, f'{key} stays under 74 lines')

    want(STAGES['L2'](_args(base)).startswith('<error>'),
         'routing refuses to start without a placement close-out')
    want(STAGES['L3'](_args(base)).startswith('<error>'),
         'classification refuses without a score')
    want(STAGES['L4'](_args(base)).startswith('<error>'),
         'a re-entry refuses without a measured shape')

    with tempfile.TemporaryDirectory() as tmp:
        # Every L2 fixture now carries the FOUR keys the gate reads, because a
        # document missing any of them is refused on SHAPE before any of its
        # content is looked at (see L2_CHECKS). That refusal is the change
        # detector: run 10 fed this gate `board_score`'s JSON, which shares the
        # field name `blocking` and means a six-component total by it, and
        # three of the four checks silently did not run.
        _asm = {'buildable': True, 'verdict': 'buildable (blocking 0)',
                'locked_contacts': 0, 'oob_pad_count': 0}
        p = os.path.join(tmp, 'p.json')
        json.dump({**_asm, 'blocking': 3}, open(p, 'w', encoding='utf-8'))
        out = STAGES['L2'](_args(base + ['--placement-report', p]))
        want(out.startswith('<error>') and 'blocking = 3' in out,
             'routing refuses a placement that still has blocking pairs')
        out = STAGES['L2'](_args(base + ['--placement-report', p,
                                         '--accept-residue', 'blocking']))
        want(out.startswith('<stage_instructions'),
             '...and proceeds when the residue is explicitly accepted')

        # blocking == 0 with parts off the board: assembly-clean because
        # nothing is out there to collide with, and unroutable for the same
        # reason. Caught by running the real loop on a damaged board.
        oob = os.path.join(tmp, 'oob.json')
        json.dump({**_asm, 'blocking': 0, 'oob_pad_count': 4},
                  open(oob, 'w', encoding='utf-8'))
        out = STAGES['L2'](_args(base + ['--placement-report', oob]))
        want(out.startswith('<error>') and 'oob_pad_count = 4' in out,
             'routing refuses a blocking-0 placement with pads off the board')
        want('edge_connectors' in out,
             '...and names how a BY-DESIGN overhang is declared instead')
        out = STAGES['L2'](_args(base + ['--placement-report', oob,
                                         '--accept-residue', 'oob_pad_count']))
        want(out.startswith('<stage_instructions'),
             '...and still proceeds when that is explicitly accepted')

        # THE COMPOUNDING HAZARD the per-check flag exists to remove: accepting
        # one check must not waive another. This exact pair is what happened --
        # a `blocking` refusal was cleared with a blanket flag, and the
        # oob_pad_count gate went with it.
        out = STAGES['L2'](_args(base + ['--placement-report', oob,
                                         '--accept-residue', 'blocking']))
        want(out.startswith('<error>') and 'oob_pad_count = 4' in out,
             'accepting `blocking` does NOT waive the oob_pad_count gate')
        out = STAGES['L2'](_args(base + ['--placement-report', oob,
                                         '--accept-residue']))
        want(out.startswith('<error>') and 'names WHICH check' in out,
             'a BARE --accept-residue is refused, not a blanket waiver')
        out = STAGES['L2'](_args(base + ['--placement-report', oob,
                                         '--accept-residue', 'nonsense']))
        want(out.startswith('<error>') and 'Unknown: nonsense' in out,
             'an unknown check name is refused by name')

        # SHAPE: board_score's JSON is not check_assembly's, and sharing the
        # field name `blocking` is precisely why this must be a key-presence
        # test rather than a value test.
        bs = os.path.join(tmp, 'bs.json')
        json.dump({'kind': 'board-score', 'blocking': 57,
                   'blocking_by': {'unrouted': 57}, 'advisory': {}},
                  open(bs, 'w', encoding='utf-8'))
        out = STAGES['L2'](_args(base + ['--placement-report', bs]))
        want(out.startswith('<error>') and 'board_score' in out,
             'L2 refuses board_score\'s JSON BY SHAPE and names it')
        want('blocking = 57' not in out,
             '...without firing the blocking check on the wrong number')
        out = STAGES['L2'](_args(base + ['--placement-report', bs,
                                         '--accept-residue', 'blocking']))
        want(out.startswith('<error>') and 'board_score' in out,
             '...and the shape refusal is NOT waivable by --accept-residue')

        # A report that answers neither question is not a close-out.
        empty = os.path.join(tmp, 'empty.json')
        json.dump({}, open(empty, 'w', encoding='utf-8'))
        want(STAGES['L2'](_args(base + ['--placement-report', empty]))
             .startswith('<error>'),
             'an empty JSON does not satisfy the hand-off guard')
        want(STAGES['L3'](_args(base + ['--score', empty]))
             .startswith('<error>'),
             'an empty JSON does not satisfy the classification guard')

        s = os.path.join(tmp, 's.json')
        json.dump({'blocking': 0}, open(s, 'w', encoding='utf-8'))
        out = STAGES['L3'](_args(base + ['--score', s]))
        want('nothing to classify' in out,
             'a clean score has no failure to classify')

    with tempfile.TemporaryDirectory() as _ct:
        def _cong(name, hpwl, crossings=500.0):
            p = os.path.join(_ct, name)
            json.dump({'metrics': {'hpwl': hpwl, 'crossings': crossings,
                                   'halo': 10.0}},
                      open(p, 'w', encoding='utf-8'))
            return p
        # A placement that closed most of its HPWL gap: `parameter` is then a
        # believable verdict and L4 emits its body. hpwl and not crossings --
        # the placement skill's non-negotiable 4, r(crossings) = +0.780 against
        # distance-to-truth -- not against routed blocking
        # (docs/placement-predictors.md).
        _cbase, _cgood = _cong('cb.json', 1000.0), _cong('cg.json', 600.0)
        _cbad = _cong('cn.json', 980.0)          # neo6502's shape: 2% closed
        _cong_ok = ['--congestion-json', _cgood, '--congestion-baseline', _cbase]

        # 'stale' matched BOTH branches of the placement text (the by-name
        # list and the NONE-RECORDED warning), so the needle asserted nothing
        # about which fired. The base fixture has no ledger, so the honest
        # output is the warning -- pin THAT string.
        for shape, needle in (('parameter', 'the routed board stands'),
                              ('floorplan', 'different ARRANGEMENT'),
                              ('placement', 'NONE RECORDED')):
            extra = _cong_ok if shape == 'parameter' else []
            out = STAGES['L4'](_args(base + ['--shape', shape] + extra))
            want(needle in out, f'{shape}-shaped re-entry says what it costs')
        # ...and with a routed board actually recorded, the staleness list is
        # BY NAME, not the warning.
        _sl = os.path.join(_ct, 'stale.jsonl')
        with open(_sl, 'w', encoding='utf-8') as fh:
            fh.write(json.dumps({'iteration': 4, 'kind': 'completion',
                                 'accepted': True,
                                 'result_sha': 'a' * 64}) + '\n')
        out = STAGES['L4'](_args(base + ['--shape', 'placement',
                                         '--ledger', _sl]))
        want('- iteration 4: aaaaaaaaaaaa' in out
             and 'NONE RECORDED' not in out,
             'a recorded routed board is named stale by iteration and sha')

        # The guard itself. `parameter` is the answer a classifier gives when
        # it cannot see congestion, so it is the one that must carry evidence.
        out = STAGES['L4'](_args(base + ['--shape', 'parameter']))
        want(out.startswith('<error>') and '--congestion-json' in out,
             'a parameter re-entry refuses without a congestion comparison')
        # It REPORTS the congestion read and does NOT refuse on it. The
        # threshold was withdrawn on measurement: the premise (damage raises
        # hpwl) inverts on 1 of 3 corpus boards, where a perfect repair scores
        # a negative gain -- docs/placement-calibration.md.
        # A poor read binds on a DISPOSITION, not on the numbers: `parameter`
        # must not be spent unacknowledged, but the driver never claims the
        # placement is wrong -- it cannot, since a perfect repair scores like
        # this on piantor (docs/placement-calibration.md).
        out = STAGES['L4'](_args(base + ['--shape', 'parameter',
                                         '--congestion-json', _cbad,
                                         '--congestion-baseline', _cbase]))
        want(out.startswith('<error>') and 'may be PLACEMENT-shaped' in out,
             'a parameter re-entry refuses a poor read until it is dispositioned')
        want('NOT a verdict' in out and '--accept-congestion' in out,
             'the refusal says the numbers are not a verdict, and names the out')
        out = STAGES['L4'](_args(base + ['--shape', 'parameter',
                                         '--congestion-json', _cbad,
                                         '--congestion-baseline', _cbase,
                                         '--accept-congestion', 'edge parts locked']))
        want(not out.startswith('<error>') and 'accepted' in out,
             'a written disposition lets the parameter re-entry proceed')
        out = STAGES['L4'](_args(base + ['--shape', 'parameter',
                                         '--congestion-json', _cgood,
                                         '--congestion-baseline', _cbase]))
        want('CONGESTION READ' not in out,
             'a healthy congestion gain adds no warning')
        out = STAGES['L4'](_args(base + ['--shape', 'parameter',
                                         '--congestion-json', _cgood]))
        want(out.startswith('<error>') and '--congestion-baseline' in out,
             'a crossings count with no baseline is not evidence')
        # placement/floorplan must NOT be gated on it -- they are the verdicts
        # that cost a routed board, and making them harder to reach than
        # `parameter` would invert the whole point.
        for shape in ('placement', 'floorplan'):
            out = STAGES['L4'](_args(base + ['--shape', shape]))
            want(not out.startswith('<error>'),
                 f'{shape}-shaped re-entry is NOT gated on congestion evidence')

    inline = STAGES['L1'](_args(base + ['--no-delegate']))
    deleg = STAGES['L1'](_args(base + ['--delegate']))
    want('<subagent_prompt' in deleg and '<subagent_prompt' not in inline,
         'only the delegated form dispatches, and --no-delegate suppresses it')
    want('TEAMMATE' in deleg and 'Agent tool' in deleg,
         'delegation names the agent-type constraint, not a folk rule')
    # The old text said "a subagent cannot spawn one", which is false in this
    # harness: `claude` and `general-purpose` carry the Agent tool and can. The
    # real constraint is the agent TYPE -- `Explore` and `Plan` are defined
    # without it -- so a half spawned as one of those cannot verify itself.
    want('cannot spawn one' not in deleg,
         'the retired claim that a subagent cannot spawn is gone')

    # DELEGATION IS THE DEFAULT, at every size. Run 14 measured 191 parts and
    # 150 nets, ran both halves inline under the old thresholds, and the
    # routing half then classified its own failure and acted on it without the
    # outer loop's L3/L4 ever firing. The size is still printed, as context.
    import tempfile as _tf
    with _tf.TemporaryDirectory() as _t:
        rep = os.path.join(_t, 'p.json')
        # All four L2_CHECKS keys: this fixture has to get PAST the shape gate
        # to reach the delegation decision it is actually testing.
        json.dump({'blocking': 0, 'oob_pad_count': 0, 'locked_contacts': 0,
                   'buildable': True, 'verdict': 'buildable (blocking 0)'},
                  open(rep, 'w', encoding='utf-8'))
        # ANY board in the corpus: this asserts the RULE, not a board, and
        # naming one would pin a skill to a specific project.
        import glob as _g
        small = next(iter(sorted(
            _g.glob(os.path.join(ROOT, 'kicad_files', '*.kicad_pcb')))), None)
        if small and os.path.isfile(small):
            n = _part_count(small)
            want(isinstance(n, int) and n > 0,
                 'the board can be sized at all')
            auto = ['--board', small, '--placement-report', rep]
            for k in ('L1', 'L2'):
                out = STAGES[k](_args(auto))
                want('DELEGATING:' in out and '<subagent_prompt' in out,
                     f'{k} delegates by default, whatever the board size')
                want(str(n) in out,
                     f'{k} still names the measured size, as context')
                want('not because of a threshold' in out,
                     f'{k} says the size did not decide it')
                out = STAGES[k](_args(auto + ['--no-delegate']))
                want('INLINE:' in out and '<subagent_prompt' not in out,
                     f'{k}: --no-delegate is the escape hatch and still works')
            # A delegated placement half could do the whole thing and record
            # nothing: L2 checks the PLACED board by content hash, the way L3
            # already checked the routed one.
            _lg = os.path.join(_t, 'wrong.jsonl')
            with open(_lg, 'w', encoding='utf-8') as _fh:
                _fh.write(json.dumps({'iteration': 0, 'kind': 'placement',
                                      'accepted': True,
                                      'result_sha': '0' * 64}) + '\n')
            # Anchor on the REFUSAL's wording, not on "is not in": the
            # delegated prompt itself contains "refuses a board that is not in
            # the ledger", so the loose phrase matches the happy path too.
            _norec = 'no lap records a board with this content hash'
            want(_norec in STAGES['L2'](_args(auto + ['--ledger', _lg])),
                 'L2 refuses a placed board that no lap recorded')
            # ...and "no ledger at all" is NOT that: a board placed by someone
            # else, handed straight to routing, has no lap to record.
            want(_norec not in STAGES['L2'](_args(
                     auto + ['--ledger', os.path.join(_t, 'none.jsonl')])),
                 'L2 does not turn "could not check" into "you failed"')
        # An unreadable board no longer changes the decision -- it only changes
        # what can be SAID about it.
        out = STAGES['L1'](_args(['--board', os.path.join(_t, 'nope.kicad_pcb'),
                                  '--placement-report', rep]))
        want('could not be read' in out and '<subagent_prompt' in out,
             'an unreadable board still delegates, and says it could not size it')

    # Assemble the emitted text from stages whose guards are SATISFIED --
    # otherwise this scans refusals and reports the instructions are missing
    # content the refusals were never going to carry.
    with tempfile.TemporaryDirectory() as tmp:
        # A REAL board: L5 now validates the path it interpolates into its
        # close-out commands, and the close-out is the run's terminal artifact.
        _bf = os.path.join(tmp, 'b.kicad_pcb')
        open(_bf, 'w', encoding='utf-8').close()
        ok_rep = os.path.join(tmp, 'p.json')
        # The close-out must also carry the keys the L2 gate now reads --
        # `buildable`, `locked_contacts` and the board it graded -- because a
        # payload that omits them is exactly what used to walk an unbuildable
        # board into routing.
        json.dump({'blocking': 0, 'oob_pad_count': 0, 'buildable': True,
                   'verdict': 'buildable (blocking 0)', 'locked_contacts': 0,
                   'board': _bf}, open(ok_rep, 'w', encoding='utf-8'))
        ok_score = os.path.join(tmp, 's.json')
        json.dump({'blocking': 4}, open(ok_score, 'w', encoding='utf-8'))
        ok_rj = os.path.join(tmp, 'rj.json')
        # L3 now shape-tests and binds this the way L2 and L5 bind theirs, so
        # the fixture has to be a render a real run produces: made from the
        # route log (`summary_json`, or --focus clusters legality instead) and
        # reporting that nothing moved (`moved_refs`, which is the only place
        # a broken freeze surfaces).
        json.dump({'instrument': {'board': _bf,
                                  'summary_json': os.path.join(tmp, 'r.log'),
                                  'before': _bf},
                   'moved_refs': [],
                   'checklist': {'d_moved': {'match': None}}},
                  open(ok_rj, 'w', encoding='utf-8'))
        # The three ways a focus render can exist and still be worthless. L3
        # used to accept any path that resolved.
        def _rj(name, **over):
            d = {'instrument': {'board': _bf,
                                'summary_json': os.path.join(tmp, 'r.log'),
                                'before': _bf},
                 'moved_refs': [], 'checklist': {'d_moved': {'match': None}}}
            d['instrument'].update(over.pop('instrument', {}))
            d.update(over)
            p = os.path.join(tmp, name)
            json.dump(d, open(p, 'w', encoding='utf-8'))
            return p

        def _l3(rj):
            return STAGES['L3'](_args(['--board', _bf, '--score', ok_score,
                                       '--render-json', rj]))

        want('DIFFERENT board' in _l3(
                 _rj('rj_other.json',
                     instrument={'board': os.path.join(tmp, 'other.kicad_pcb')})),
             'L3 refuses a focus render of another board')
        want('without --summary-json' in _l3(
                 _rj('rj_nosum.json', instrument={'summary_json': None})),
             'L3 refuses a focus render that clustered legality, not failures')
        want('FREEZE DID NOT HOLD' in _l3(
                 _rj('rj_moved.json', moved_refs=['R1', 'C7'])),
             'L3 refuses a routed board whose parts moved, and names them')

        # The LEDGER is evidence too, now that L5 refuses without one -- it is
        # the file L5's verdict is computed from. This fixture asserts every
        # stage emits "once its evidence exists", so the ledger has to be part
        # of what exists; without it the assertion silently tested the default
        # path `wk/ledger.jsonl`, which is not in the repo.
        ok_led = os.path.join(tmp, 'ledger.jsonl')
        sys.path.insert(0, ROOT)
        from board_store import sha256_file as _sha
        with open(ok_led, 'w', encoding='utf-8') as _fh:
            _fh.write(json.dumps({'iteration': 0, 'kind': 'completion',
                                  'accepted': True,
                                  'result_sha': _sha(_bf)}) + '\n')
        full = ['--board', _bf, '--ledger', ok_led,
                '--score', ok_score, '--placement-report', ok_rep,
                '--render-json', ok_rj, '--shape', 'placement']
        everything = '\n'.join(STAGES[k](_args(full)) for k in sorted(STAGES))
        for k in sorted(STAGES):
            want(not STAGES[k](_args(full)).startswith('<error>'),
                 f'{k} emits instructions once its evidence exists')
    for phrase in ('you may want to', 'if you are not sure', 'usually skip'):
        want(phrase not in everything.lower(), f'no hedging: {phrase!r}')
    want('copper is not evidence about placement' in everything,
         'the rule that only exists here is stated')

    # L5 is a MEASUREMENT now, not a checklist. Its whole point is that a run
    # which stopped because it was finished and one that stopped because it was
    # stuck no longer produce the same artifact.
    with tempfile.TemporaryDirectory() as tmp:
        # The board must EXIST for these. It used to be the bare name
        # 'b.kicad_pcb', which is exactly the hole L5 now closes: this stage
        # names the board in every command it emits and validated none of them,
        # so a DONE close-out could be produced for a file nobody graded. The
        # fixtures were relying on that.
        _b5 = os.path.join(tmp, 'b.kicad_pcb')
        open(_b5, 'w', encoding='utf-8').close()
        base = ['--board', _b5]

        want(STAGES['L5'](_args(base)).startswith('<error>'),
             'close-out refuses without a score to decide on')
        want('does not exist' in STAGES['L5'](_args(
                 ['--board', os.path.join(tmp, 'ghost.kicad_pcb'),
                  '--ledger', os.path.join(tmp, 'x.jsonl'),
                  '--score', (lambda p: (json.dump({'blocking': 0},
                                                   open(p, 'w', encoding='utf-8')),
                                         p)[1])(os.path.join(tmp, 'g.json'))])),
             'close-out refuses a board that does not exist')

        def ledger_of(rows, name):
            p = os.path.join(tmp, name)
            with open(p, 'w', encoding='utf-8') as fh:
                for i, r in enumerate(rows):
                    fh.write(json.dumps(dict(r, iteration=i)) + '\n')
            return p

        def scored(doc, name):
            p = os.path.join(tmp, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        def closed(name, **over):
            """A check_complete.py close-out, DONE unless told otherwise."""
            d = {'schema': 1, 'kind': 'board-complete', 'board': _b5,
                 'score': {'blocking': 0}, 'components': {'orphan_stubs': {}},
                 'fab_floors': {'ran': True, 'relaxed': []},
                 'verdict': 'DONE', 'reason': 'clean', 'ungraded': []}
            d.update(over)
            return scored(d, name)

        # The rows carry the fixture board's REAL content hash, because the
        # close-out now checks that the board it is about to ship is in the
        # ledger -- on the success path L2 goes straight to L5, so this was the
        # one route where nothing verified that the routing half recorded
        # anything. A fixture without result_sha would be a fixture no real
        # ledger produces.
        try:
            sys.path.insert(0, ROOT)
            from board_store import sha256_file as _sha256_file
            _b5sha = _sha256_file(_b5)
        except Exception:                                   # noqa: BLE001
            _b5sha = None
        flat = ([{'kind': 'placement', 'accepted': True, 'result_sha': _b5sha,
                  'score': {'blocking': 0, 'quality': {}}}] * 6
                + [{'kind': 'completion', 'accepted': True,
                    'result_sha': _b5sha,
                    'score': {'blocking': 0, 'quality': {}}}] * 6)
        out = STAGES['L5'](_args(base + [
            '--ledger', ledger_of([], 'fresh.jsonl'),
            '--score', scored({'blocking': 0}, 'zero.json')]))
        want('not done yet' in out,
             'blocking == 0 with the halves still moving does NOT close out')
        # CONTINUE must NOT be gated: it is the go-round-again branch, and
        # gating it would run the most expensive instrument every lap.
        want('routing close-out' not in out,
             'the CONTINUE branch is not gated on a close-out')

        _fl = ledger_of(flat, 'flat.jsonl')
        _z2 = scored({'blocking': 0}, 'z2.json')
        done_args = base + ['--ledger', _fl, '--score', _z2]

        # THE SUCCESS PATH IS THE ONE THAT SKIPPED THIS. A clean route goes
        # L2 -> L5 with no L3 in between, so until now nothing ever checked
        # that the routing half recorded the board it is shipping.
        if _b5sha:
            _wrong = ledger_of([dict(r, result_sha='0' * 64) for r in flat],
                               'wrongsha.jsonl')
            want('is not in' in STAGES['L5'](_args(
                     base + ['--ledger', _wrong, '--score', _z2,
                             '--routing-close', closed('rc_unrec.json')])),
                 'the close-out refuses to ship a board no lap recorded')
        # An ABSENT ledger is a refusal HERE and a pass at L2/L3. `_recorded`
        # answers None for both, so a run that simply never created the file
        # walked every gate that checks it -- the run-13 shape in its purest
        # form. Both directions are asserted, because a gate that refused a
        # missing ledger everywhere would break the board-placed-elsewhere path
        # that L2's None policy exists to keep working.
        _gone = os.path.join(tmp, 'no-such-ledger.jsonl')
        want('no ledger at' in STAGES['L5'](_args(
                 base + ['--ledger', _gone, '--score', _z2,
                         '--routing-close', closed('rc_noled.json')])),
             'L5 refuses when the ledger does not exist at all')
        # ...on the CONTINUE branch too, which is the one it reaches: an empty
        # history reads as "still improving", and that branch is ungated.
        want('no ledger at' in STAGES['L5'](_args(
                 base + ['--ledger', _gone, '--score', _z2])),
             '...including on the ungated CONTINUE branch')
        _pr5 = scored({'board': _b5, 'buildable': True,
                       'verdict': 'buildable (blocking 0)',
                       'locked_contacts': 0, 'blocking': 0,
                       'oob_pad_count': 0}, 'pr_noled.json')
        want(not STAGES['L2'](_args(
                 base + ['--ledger', _gone,
                         '--placement-report', _pr5])).startswith('<error>'),
             '...while L2 still passes with no ledger (placed elsewhere)')
        out = STAGES['L5'](_args(done_args))
        want(out.startswith('<error>') and 'routing close-out' in out,
             'close-out refuses to SHIP without a routing close-out')
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', closed('c_done.json')]))
        want('DONE-EXHAUSTED' in out and 'make_film' in out,
             'a plateaued solved board closes out, with the film')

        # THE AGREEMENT CHECK, both directions. Asserting only the refusal
        # would pass for a gate that refused everything.
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', closed('c_unsound.json', verdict='UNSOUND',
                                      reason='floors moved')]))
        want(out.startswith('<error>') and 'TWO INSTRUMENTS DISAGREE' in out,
             'DONE-EXHAUSTED + UNSOUND is refused')
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', closed('c_inc.json', verdict='INCOMPLETE',
                                      reason='orphan stubs')]))
        want(out.startswith('<error>') and 'TWO INSTRUMENTS DISAGREE' in out,
             'DONE-EXHAUSTED + INCOMPLETE is refused')
        want('DONE-EXHAUSTED' in STAGES['L5'](_args(
                 done_args + ['--routing-close', closed('c_ok2.json'),
                              '--accept-unclosed', 'agreement'])),
             '...and --accept-unclosed agreement ships it, named')

        _stuck = ledger_of(
            [dict(r, score={'blocking': 4, 'quality': {}}) for r in flat],
            'stuck.jsonl')
        _f4 = scored({'blocking': 4}, 'four.json')
        out = STAGES['L5'](_args(base + [
            '--ledger', _stuck, '--score', _f4,
            '--routing-close', closed('c_inc2.json', verdict='INCOMPLETE',
                                      reason='blocking = 4')]))
        want('STUCK' in out and 'calling this finished is not' in out,
             'a plateaued UNSOLVED board says stuck, not done')
        want(not out.startswith('<error>'),
             'STUCK + INCOMPLETE is a CONSISTENT pair and is NOT refused')

        # The document must be the right one, and about the right board.
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', scored(
                {'kind': 'board-score', 'blocking': 0, 'blocking_by': {},
                 'quality': {}}, 'c_bs.json')]))
        want(out.startswith('<error>') and 'board_score' in out,
             'a board_score JSON is refused BY SHAPE and named')
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', closed('c_skip.json', components={})]))
        want(out.startswith('<error>') and '--skip-slow' in out,
             'a --skip-slow document is refused, and --skip-slow is named')
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', closed('c_noff.json',
                                      fab_floors={'ran': False,
                                                  'reason': 'no --authored-from'})]))
        want(out.startswith('<error>') and 'UNSOUND is unreachable' in out,
             'a close-out that could not check the floors is refused')
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', closed('c_ung.json',
                                      ungraded=['impedance', 'length'])]))
        want(out.startswith('<error>') and 'never examined' in out,
             'unexamined components are refused, and named')
        # ...and each waiver waives ONLY its own check.
        out = STAGES['L5'](_args(done_args + [
            '--routing-close', closed('c_two.json', components={},
                                      ungraded=['impedance']),
            '--accept-unclosed', 'ungraded']))
        want(out.startswith('<error>') and '--skip-slow' in out,
             'accepting `ungraded` does NOT waive `instruments`')
        want(STAGES['L5'](_args(done_args + [
                 '--routing-close', closed('c_bare.json'),
                 '--accept-unclosed'])).startswith('<error>'),
             'a bare --accept-unclosed is refused')
        want('Unknown: nonsense' in STAGES['L5'](_args(done_args + [
                 '--routing-close', closed('c_unk.json'),
                 '--accept-unclosed', 'nonsense'])),
             'an unknown --accept-unclosed name is refused BY NAME')

        # The staleness list, the film and step-back all read the ledger, so
        # the stage that feeds them checks the board is in it.
        board = os.path.join(tmp, 'b.kicad_pcb')
        open(board, 'w', encoding='utf-8').write('(kicad_pcb)')
        out = STAGES['L3'](_args(
            ['--board', board, '--ledger', ledger_of(flat, 'l3.jsonl'),
             '--score', scored({'blocking': 3}, 'b3.json')]))
        want(out.startswith('<error>') and 'not in the ledger' in out,
             'classification refuses a routed board nobody recorded')

    # ------------------------------------------------------------------ D3
    # THE CROSS-CHECK, on the branch it never ran on. L5 is documented as the
    # stage that refuses when two independent instruments disagree, and it only
    # compared on the DONE path. The measured run wrote a --final row carrying
    # VERDICT=PASS:lens=connectivity on a score of 32 unrouted / 47 broken and
    # then kept going, so every visit to L5 took CONTINUE and the gate sat idle.
    with tempfile.TemporaryDirectory() as tmp:
        _b = os.path.join(tmp, 'b.kicad_pcb')
        open(_b, 'w', encoding='utf-8').write('(kicad_pcb)')
        sys.path.insert(0, ROOT)
        from board_store import sha256_file as _shaf
        _bs = _shaf(_b)
        _run17 = {'blocking': 79, 'quality': {},
                  'blocking_by': {'unrouted': 32, 'broken': 47, 'drc': 0,
                                  'undersized': 0}}

        def _led(rows, name):
            p = os.path.join(tmp, name)
            with open(p, 'w', encoding='utf-8') as fh:
                for i, r in enumerate(rows):
                    fh.write(json.dumps(dict(r, iteration=i,
                                             result_sha=_bs)) + '\n')
            return p

        def _js(doc, name):
            p = os.path.join(tmp, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        _false = {'kind': 'completion', 'accepted': True, 'final': True,
                  'score': _run17,
                  'lenses': ['VERDICT=PASS:lens=connectivity',
                             'VERDICT=PASS:lens=drc',
                             'VERDICT=PASS:lens=spec']}
        _honest = dict(_false, lenses=[
            'VERDICT=FAIL:lens=connectivity;finding=32 nets carry no copper;'
            'evidence=score.json#/blocking_by',
            'VERDICT=PASS:lens=drc', 'VERDICT=PASS:lens=spec'])
        _sc = _js({'blocking': 79}, 'sc.json')
        _l5 = ['--board', _b, '--score', _sc]

        out = STAGES['L5'](_args(_l5 + ['--ledger', _led([_false], 'f.jsonl')]))
        want(out.startswith('<error>') and 'TWO INSTRUMENTS DISAGREE' in out,
             'L5 catches a false lens on the CONTINUE branch, not only on DONE')
        want('unrouted = 32' in out and 'lens=connectivity' in out,
             '...and names the lens and the number that refutes it')
        want(not STAGES['L5'](_args(
                 _l5 + ['--ledger', _led([_honest], 'h.jsonl')]
             )).startswith('<error>'),
             'a FAIL lens beside the same numbers is HONEST and is not refused')
        want(not STAGES['L5'](_args(
                 _l5 + ['--ledger', _led([dict(_false, final=False)],
                                         'nf.jsonl')]
             )).startswith('<error>'),
             'a non-final row is not a close-out claim and is not cross-checked')
        want('TWO INSTRUMENTS DISAGREE' not in STAGES['L5'](_args(
                 _l5 + ['--ledger', _led([_false], 'w.jsonl'),
                        '--accept-unclosed', 'agreement'])),
             '...and --accept-unclosed agreement still waives it, named')
        # A final row claiming every lens passed, against a close-out that says
        # the board is INCOMPLETE. Neither instrument knows about the other.
        _cd = _js({'schema': 1, 'kind': 'board-complete', 'board': _b,
                   'score': {'blocking': 0},
                   'components': {'orphan_stubs': {}},
                   'fab_floors': {'ran': True}, 'verdict': 'INCOMPLETE',
                   'reason': 'orphan stubs', 'ungraded': []}, 'cd.json')
        _clean = dict(_false, score={'blocking': 0, 'quality': {},
                                     'blocking_by': {'unrouted': 0,
                                                     'broken': 0}})
        out = STAGES['L5'](_args(_l5 + ['--ledger', _led([_clean], 'c.jsonl'),
                                        '--routing-close', _cd]))
        want(out.startswith('<error>') and 'all PASS' in out,
             'an all-PASS final row against an INCOMPLETE close-out is refused')
        # A LATER final row SUPERSEDES an earlier one. The ledger is
        # append-only, so a run that wrote a wrong close-out and then wrote the
        # correction has both on file: checking every final row makes the
        # correction unable to clear the gate, forever. Measured on the real
        # ledger -- iteration 21 is the false PASS and iteration 22 its signed
        # retraction -- that run could never have reached L5 again.
        want(not STAGES['L5'](_args(
                 _l5 + ['--ledger', _led([_false, _honest], 'ret.jsonl')]
             )).startswith('<error>'),
             'a later final row RETRACTS an earlier false one')
        # ...but ONLY on the lens it speaks to. Taking simply "the last final
        # row" let ANY later row bury the claim, and a one-lens
        # `--kind systemic --final` row is trivially writable -- the
        # connectivity/drc/spec requirement only binds --kind completion.
        # Measured: a later row carrying nothing but PASS:lens=spec cleared a
        # live false PASS on connectivity.
        want(STAGES['L5'](_args(_l5 + ['--ledger', _led(
                 [_false, dict(_false, kind='systemic',
                               lenses=['VERDICT=PASS:lens=spec'])],
                 'bury.jsonl')])).startswith('<error>'),
             'an unrelated later final row does NOT bury a false lens')
        want(STAGES['L5'](_args(
                 _l5 + ['--ledger', _led([_honest, _false], 'reg.jsonl')]
             )).startswith('<error>'),
             '...and a false one written LAST is still caught')
        # BOTH lenses in the table are live. Only `connectivity` was exercised,
        # so deleting the drc row would have gone unnoticed.
        _drc = dict(_false, lenses=['VERDICT=PASS:lens=drc'],
                    score={'blocking': 4, 'quality': {},
                           'blocking_by': {'unrouted': 0, 'broken': 0,
                                           'drc': 4, 'undersized': 0}})
        want('lens=drc' in STAGES['L5'](_args(
                 _l5 + ['--ledger', _led([_drc], 'drc.jsonl')])),
             'the drc lens is checked too, not only connectivity')
        # ...and a lens name is CASE-FOLDED before the table lookup: the
        # grammar accepts [A-Za-z0-9_-]+, so `lens=Connectivity` passed the
        # format check, missed the lower-case table and was never compared.
        want(STAGES['L5'](_args(_l5 + ['--ledger', _led([dict(
                 _false, lenses=['VERDICT=PASS:lens=Connectivity'])],
                 'case.jsonl')])).startswith('<error>'),
             'a capitalised lens name does not bypass the comparison')

    # ------------------------------------------------------------------ D4
    # A SECOND CYCLE IS A DESIGNED PATH -- L5's CONTINUE branch and both
    # expensive L4 re-entries all say `--stage L1` -- and the driver had no
    # concept of one. Measured, on a real second cycle, L2 printed cycle ONE's
    # frozen.kicad_pcb (this chain's --authored-from baseline, in the ledger by
    # content hash) and cycle ONE's freeze_refs.json.
    with tempfile.TemporaryDirectory() as tmp:
        _b = os.path.join(tmp, 'b.kicad_pcb')
        open(_b, 'w', encoding='utf-8').write('(kicad_pcb)')
        sys.path.insert(0, ROOT)
        from board_store import sha256_file as _shaf2
        _rep = os.path.join(tmp, 'p.json')
        json.dump({'blocking': 0, 'oob_pad_count': 0, 'locked_contacts': 0,
                   'buildable': True, 'verdict': 'buildable (blocking 0)',
                   'board': _b}, open(_rep, 'w', encoding='utf-8'))

        def _led2(rows, name):
            p = os.path.join(tmp, name)
            with open(p, 'w', encoding='utf-8') as fh:
                for i, r in enumerate(rows):
                    fh.write(json.dumps(dict(r, iteration=i)) + '\n')
            return p

        _one = [{'kind': 'placement', 'accepted': True,
                 'result_sha': _shaf2(_b)}]
        _two = _one + [{'kind': 'completion', 'accepted': True,
                        'result_sha': 'c' * 64},
                       {'kind': 'placement', 'accepted': True,
                        'result_sha': 'd' * 64}]
        want(_cycle_index(_led2(_one, 'c1.jsonl')) == 1
             and _cycle_index(_led2(_two, 'c2.jsonl')) == 2,
             'the cycle index is read off the ledger, not assumed')
        # L1 OPENS a cycle, so it is asked BEFORE its own lap exists. At the
        # top of cycle 2 the ledger is [placement..., routing...] and counting
        # only recorded placement phases returns 1 -- so L1, the one stage that
        # starts a cycle, still named cycle ONE's placed board. Measured on the
        # fix itself: a 78-line stage whose teammate prompt read
        # `placed board : wk/placed.kicad_pcb`.
        _top2 = _led2([{'kind': 'placement', 'accepted': True,
                        'result_sha': _shaf2(_b)},
                       {'kind': 'completion', 'accepted': True,
                        'result_sha': 'c' * 64}], 'top2.jsonl')
        want(_cycle_index(_top2, starting=True) == 2
             and _cycle_index(_top2) == 1,
             'L1 counts the cycle it is about to OPEN; L2 the one it is in')
        _l1top = STAGES['L1'](_args(['--board', _b, '--placement-report', _rep,
                                     '--ledger', _top2]))
        want('placed_c2.kicad_pcb' in _l1top and '/placed.kicad_pcb' not in _l1top,
             'L1 at the top of cycle 2 names cycle 2\'s board, not cycle 1\'s')
        # A placement row that re-records a board ALREADY in the ledger changed
        # nothing, so it is a disposition and not a lap. Measured: run 17's
        # iteration 33 ("VERIFIER DISPOSITION (no pose change; same board as
        # it.30)") read as a third cycle on a ledger whose levers say cycle 2.
        want(_cycle_index(_led2(_two + [
                 {'kind': 'completion', 'accepted': True,
                  'result_sha': 'e' * 64},
                 {'kind': 'placement', 'accepted': True,
                  'result_sha': 'd' * 64}], 'disp.jsonl')) == 2,
             'a disposition row that re-records a known board is not a cycle')
        # ...and routing BEFORE any placement does not put the first placement
        # lap into cycle 2, which would break "cycle 1 keeps the bare names".
        want(_cycle_index(_led2([{'kind': 'completion', 'accepted': True,
                                  'result_sha': 'a' * 64},
                                 {'kind': 'placement', 'accepted': True,
                                  'result_sha': 'b' * 64}], 'rf.jsonl')) == 1,
             'a routing-first ledger still starts at cycle 1')

        _a1 = ['--board', _b, '--placement-report', _rep,
               '--ledger', _led2(_one, 'l1.jsonl')]
        _a2 = ['--board', _b, '--placement-report', _rep,
               '--ledger', _led2(_two, 'l2.jsonl')]
        for k in ('L1', 'L2'):
            one, two = STAGES[k](_args(_a1)), STAGES[k](_args(_a2))
            want('_c2' not in one and 'CYCLE 2' not in one,
                 f'{k} cycle 1 keeps the bare artifact names')
            want('_c2.kicad_pcb' in two and 'CYCLE 2' in two,
                 f'{k} cycle 2 names its OWN artifacts, and says which cycle')
        two = STAGES['L2'](_args(_a2))
        # The one legitimate mention of cycle 1's bare name is the
        # --authored-from READ (D5: floors are graded against the board whose
        # floors are still authored). Every other mention is a write target
        # and stays forbidden.
        _two_writes = '\n'.join(l for l in two.splitlines()
                                if '--authored-from' not in l)
        want('/frozen.kicad_pcb' not in _two_writes
             and '/freeze_refs.json' not in _two_writes,
             'a second cycle is never told to WRITE cycle 1\'s frozen board')
        want('--authored-from' in two and '/frozen.kicad_pcb' in two,
             '...while its close-out still READS cycle 1\'s frozen baseline')
        want(f'copy_board.py {_b} ' in two,
             'the freeze copies the board it was HANDED, not a derived name')
        # The belt: whatever the naming, SAY SO when a named output path
        # already holds a board the ledger references. A note, not a refusal --
        # these stages are instruction printers, so re-invoking one after doing
        # what it said is normal, and as a refusal this exited 4 on every
        # resumed session, recommending the one action (move the file) that
        # destroys the --authored-from baseline it exists to protect.
        _work2 = os.path.dirname(_a2[-1]).replace('\\', '/')
        _fz = os.path.join(_work2, 'frozen_c2.kicad_pcb')
        open(_fz, 'w', encoding='utf-8').write('(kicad_pcb) baseline')
        _coll = _led2(_two + [{'kind': 'completion', 'accepted': True,
                               'result_sha': _shaf2(_fz)}], 'coll.jsonl')
        _ca = ['--board', _b, '--placement-report', _rep, '--ledger', _coll]
        out = STAGES['L2'](_args(_ca))
        want(not out.startswith('<error>') and 'ALREADY IN THE LEDGER' in out
             and 'frozen_c2' in out,
             'a named output already in the ledger is NAMED, not refused')
        want('do not move or rename' in out,
             '...and the note does not recommend destroying the baseline')
        # ...and the TEAMMATE sees it. The agent that writes these files is the
        # one that has to be warned; a note living only in the orchestrator's
        # copy, above a 50-line subagent prompt, is a note the writer never
        # reads.
        # Between the OPENING and CLOSING tags. Splitting on the opening tag
        # alone keeps everything after the prompt too, so a note that drifted
        # below </subagent_prompt> would still pass -- and on a stage with no
        # prompt at all the assertion would be vacuous.
        def _prompt(text):
            if '<subagent_prompt' not in text:
                return ''
            return text.split('<subagent_prompt', 1)[1].split(
                '</subagent_prompt>', 1)[0]

        want('ALREADY IN THE LEDGER' in _prompt(out),
             '...and the warning rides INSIDE L2\'s subagent prompt')
        want('ALREADY IN THE LEDGER' not in STAGES['L2'](_args(
                 ['--board', _b, '--placement-report', _rep,
                  '--ledger', _led2(_two, 'clean.jsonl')])),
             '...and stays silent when no named path collides')
        # L1 checks its own artifacts too, not only L2's.
        _pl = os.path.join(_work2, 'placed_c2.kicad_pcb')
        open(_pl, 'w', encoding='utf-8').write('(kicad_pcb) placed')
        _l1c = STAGES['L1'](_args(
            ['--board', _b, '--placement-report', _rep,
             '--ledger', _led2(_two + [
                 {'kind': 'placement', 'accepted': True,
                  'result_sha': _shaf2(_pl)}], 'coll1.jsonl')]))
        want('ALREADY IN THE LEDGER' in _l1c,
             'L1 names its own colliding artifacts too, not only L2\'s')
        # L1's PROMPT too: the claim was "both prompts" and only L2's was
        # checked, so deleting L1's stayed green.
        want('ALREADY IN THE LEDGER' in _prompt(_l1c),
             '...inside L1\'s subagent prompt, which writes the placed board')

    # ------------------------------------------------------------------ D5
    # `stages.log` existed only because the CALLER teed it, so an unlogged
    # invocation could only ever be inferred, never detected.
    with tempfile.TemporaryDirectory() as tmp:
        import contextlib
        import io as _io
        _b = os.path.join(tmp, 'b.kicad_pcb')
        open(_b, 'w', encoding='utf-8').write('(kicad_pcb)')
        _lg = os.path.join(tmp, 'ledger.jsonl')
        open(_lg, 'w', encoding='utf-8').close()
        _argv = ['--stage', 'L1', '--board', _b, '--ledger', _lg]
        _cap = _io.StringIO()
        with contextlib.redirect_stdout(_cap):
            rc = main(_argv)
        _logp = os.path.join(tmp, 'loop_driver.log')
        want(rc == 0 and os.path.isfile(_logp),
             'the driver writes its own log beside the ledger')
        rows = [json.loads(x) for x in open(_logp, encoding='utf-8') if x.strip()]
        want(len(rows) == 1 and rows[0]['stage'] == 'L1'
             and rows[0]['exit'] == 0 and rows[0]['refused'] is False,
             '...recording the stage and how it ended')
        want(isinstance(rows[0].get('argv'), list) and rows[0].get('out_sha'),
             '...with the argv and a digest of what it emitted')
        want(_cap.getvalue().strip() == STAGES['L1'](_args(_argv)).strip(),
             'stdout is UNCHANGED, so an existing caller sees exactly what it saw')
        with contextlib.redirect_stdout(_io.StringIO()):
            main(['--stage', 'L2', '--board', _b, '--ledger', _lg])
        rows = [json.loads(x) for x in open(_logp, encoding='utf-8') if x.strip()]
        want(len(rows) == 2 and rows[1]['stage'] == 'L2'
             and rows[1]['exit'] == 4 and rows[1]['refused'] is True,
             'a REFUSAL is logged too -- that is the invocation worth detecting')
        # The log is written BEFORE stdout, so a stage that dies printing still
        # leaves its trace -- which is the invocation most worth having.
        _dead = os.path.join(tmp, 'dead')
        os.makedirs(_dead)
        _dl = os.path.join(_dead, 'ledger.jsonl')
        open(_dl, 'w', encoding='utf-8').close()

        class _Boom(_io.StringIO):
            def write(self, _s):
                raise RuntimeError('stdout is gone')

        try:
            with contextlib.redirect_stdout(_Boom()):
                main(['--stage', 'L1', '--board', _b, '--ledger', _dl])
        except RuntimeError:
            pass
        want(os.path.isfile(os.path.join(_dead, 'loop_driver.log')),
             'the log is written BEFORE stdout, so a dying stage still logs')
        # A log that cannot be written must never break a stage -- but it must
        # not be SILENT either: an unlogged invocation that says nothing can
        # only be inferred, which is the hole this closes.
        _nodir = _args(['--stage', 'L1', '--board', _b,
                        '--ledger', os.path.join(tmp, 'no', 'such', 'l.jsonl')])
        _errbuf = _io.StringIO()
        with contextlib.redirect_stderr(_errbuf):
            _r = _log_invocation(_nodir, 'L1', 'x', 0)
        want(_r is None and not STAGES['L1'](_nodir).startswith('<error>'),
             'an unwritable log is never a refusal')
        want('UNRECORDED' in _errbuf.getvalue(),
             '...and says on stderr that the invocation went unrecorded')

    # ------------------------------------------- REGRESSION PINS (do not weaken)
    # Three refusals that each caught a REAL defect in the measured run. They
    # are pinned here by their measurement so a later change cannot quietly
    # soften them -- the failure mode being guarded against is a gate that is
    # relaxed to make a run proceed and never tightened again.
    with tempfile.TemporaryDirectory() as tmp:
        _b = os.path.join(tmp, 'b.kicad_pcb')
        open(_b, 'w', encoding='utf-8').write('(kicad_pcb)')
        # BOTH halves plateaued, so L5 reaches a TERMINAL branch -- the
        # not-in-ledger check lives on the ship path, and a CONTINUE fixture
        # would pin nothing.
        _wrong = os.path.join(tmp, 'wrong.jsonl')
        with open(_wrong, 'w', encoding='utf-8') as fh:
            for i, kind in enumerate(['placement'] * 6 + ['completion'] * 6):
                fh.write(json.dumps({
                    'iteration': i, 'kind': kind, 'accepted': True,
                    'result_sha': '0' * 64,
                    'score': {'blocking': 0, 'quality': {}}}) + '\n')

        def _w(doc, name):
            p = os.path.join(tmp, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        _asm = _w({'blocking': 0, 'oob_pad_count': 0, 'locked_contacts': 0,
                   'buildable': True, 'verdict': 'buildable (blocking 0)',
                   'board': _b}, 'asm.json')
        _s3 = _w({'blocking': 3}, 's3.json')
        _rj = _w({'instrument': {'board': _b, 'summary_json': 'r.log'},
                  'moved_refs': [], 'checklist': {}}, 'rj.json')
        _cc = _w({'schema': 1, 'kind': 'board-complete', 'board': _b,
                  'score': {'blocking': 0}, 'components': {'orphan_stubs': {}},
                  'fab_floors': {'ran': True}, 'verdict': 'DONE',
                  'reason': 'clean', 'ungraded': []}, 'cc.json')
        _need = 'no lap records a board with this content hash'
        for k, extra in (('L2', ['--placement-report', _asm]),
                         ('L3', ['--score', _s3, '--render-json', _rj]),
                         ('L5', ['--score', _w({'blocking': 0}, 's0.json'),
                                 '--routing-close', _cc])):
            out = STAGES[k](_args(['--board', _b, '--ledger', _wrong] + extra))
            want(out.startswith('<error>') and (
                _need in out or 'not in the ledger' in out),
                f'PIN: {k} still refuses a board no lap recorded (caught 3x)')
        # PIN: `parameter` is the verdict a blind classifier returns by default,
        # so it is the one that must carry a measurement. Run 15 spent its
        # authorised iteration on rip-up depth for an arrangement problem.
        out = STAGES['L4'](_args(['--board', _b, '--shape', 'parameter']))
        want(out.startswith('<error>') and '--congestion-json' in out,
             'PIN: a `parameter` re-entry still needs a congestion measurement')
        want(STAGES['L4'](_args(['--board', _b, '--shape', 'parameter',
                                 '--accept-residue', 'blocking'])
                          ).startswith('<error>'),
             '...and the L2 placement vocabulary does not waive it')
        # PIN: the placement close-out must be check_assembly's document.
        # board_score publishes `blocking` too and means a six-component total.
        out = STAGES['L2'](_args(['--board', _b, '--placement-report', _w(
            {'kind': 'board-score', 'blocking': 57,
             'blocking_by': {'unrouted': 57}}, 'bs.json')]))
        want(out.startswith('<error>') and 'board_score' in out
             and 'blocking = 57' not in out,
             'PIN: L2 still refuses board_score JSON BY SHAPE, without firing '
             'the blocking check on the wrong number')

        # ------------------------------------------- run-17 audit fixes (D-series)
        # D1: the run-closing record carries the three lens slots. The full
        # runs-as-printed pin (substitute placeholders, EXECUTE the command)
        # lives in tests/test_converge.py; this is the cheap structural half.
        _frc = final_record_command('l.jsonl', 'b.kicad_pcb', 's.json', 'STUCK')
        want(_frc.count('--lens') == 3 and 'connectivity' in _frc
             and 'drc' in _frc and 'spec' in _frc,
             'D1: the printed --final command carries three --lens slots')

        # D4: the verdict subprocess must read the SAME ledger the caller
        # named, from any cwd. converge treats a MISSING ledger exactly like
        # an empty one (CONTINUE from zero rows, measured), so the fixture
        # ledger carries one real row -- pre-fix, cwd=ROOT made the relative
        # name resolve to a nonexistent file and ledger_rows read 0.
        import subprocess as _sp
        _d4 = os.path.join(tmp, 'd4')
        os.makedirs(_d4)
        _dled = os.path.join(_d4, 'led.jsonl')
        _rec = _sp.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_placer', 'converge.py'), 'record',
                        '--ledger', _dled, '--board', _b,
                        '--kind', 'completion', '--lever', 'd4 fixture row'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
        json.dump({'blocking': 0},
                  open(os.path.join(_d4, 'sc.json'), 'w', encoding='utf-8'))
        _prev = os.getcwd()
        try:
            os.chdir(_d4)
            _got = _verdict(_args(['--board', _b, '--ledger', 'led.jsonl',
                                   '--score', 'sc.json']))
        finally:
            os.chdir(_prev)
        want(_rec.returncode == 0 and _got is not None
             and isinstance(_got[1], dict)
             and (_got[1].get('ledger_rows') or 0) >= 1,
             'D4: _verdict reads the caller-relative ledger from any cwd')

        # D5: on cycle >= 2, the emitted close-out grades against CYCLE 1's
        # frozen board -- the only one whose floors are still authored -- while
        # the freeze itself writes the _cN name.
        _c2 = os.path.join(tmp, 'c2')
        os.makedirs(_c2)
        open(os.path.join(_c2, 'frozen_c2.kicad_pcb'), 'w',
             encoding='utf-8').close()
        _c2fwd = _c2.replace('\\', '/')
        out = STAGES['L2'](_args(['--board', _b, '--placement-report', _asm,
                                  '--ledger', _c2fwd + '/led.jsonl']))
        want(f'--authored-from {_c2fwd}/frozen.kicad_pcb' in out
             and 'frozen_c2.kicad_pcb' in out,
             'D5: cycle-2 close-out grades against cycle 1\'s frozen baseline')

        # D9: a score that exists but cannot be read is a re-score refusal,
        # never the ship ceremony.
        _badscore = os.path.join(tmp, 'bad.json')
        open(_badscore, 'w', encoding='utf-8').write('not json')
        out = STAGES['L5'](_args(['--board', _b, '--ledger', _wrong,
                                  '--score', _badscore]))
        want(out.startswith('<error>') and 'close out:' not in out
             and 'board_score.py' in out and '--stop-condition' not in out,
             'D9: an unreadable score is a re-score refusal, not a ceremony')

        # run-19 A1: a REPEATED --accept-* flag must ACCUMULATE. Plain
        # nargs='*' made the second occurrence REPLACE the first, so
        # `--accept-unclosed agreement --accept-unclosed ungraded` silently
        # dropped `agreement` and the cross-check re-fired at L5. A bare flag
        # still yields [] (the refusals keep firing); omitted still yields
        # None (every `is not None` guard holds).
        _au = _args(base + ['--accept-unclosed', 'agreement',
                            '--accept-unclosed', 'ungraded'])
        want(_au.accept_unclosed == ['agreement', 'ungraded'],
             'A1: repeated --accept-unclosed flags accumulate, never replace')
        _ar = _args(base + ['--accept-residue', 'blocking',
                            '--accept-residue', 'oob_pad_count'])
        want(_ar.accept_residue == ['blocking', 'oob_pad_count'],
             'A1: repeated --accept-residue flags accumulate, never replace')

        # --------------------- run-17 audit: the gates that had no pinning test
        # Each of these held in the audit's hand-checks -- and deleting any of
        # them stayed green. That is the regression shape these pins close.
        _other = os.path.join(tmp, 'other.kicad_pcb')

        # L2 close-out <-> board binding: a stale or foreign close-out must
        # not unlock routing on a board nobody graded.
        out = STAGES['L2'](_args(['--board', _b, '--placement-report', _w(
            {'blocking': 0, 'oob_pad_count': 0, 'locked_contacts': 0,
             'buildable': True, 'verdict': 'ok', 'board': _other},
            'asm_foreign.json')]))
        want(out.startswith('<error>') and 'DIFFERENT board' in out,
             'PIN: L2 refuses a close-out that graded a different board')

        # L2 malformed counts: a count that is not a number was never measured.
        for _v, _needle in (('3', 'not a number'), (True, 'not a number'),
                            (float('nan'), 'NaN and infinity'),
                            (-1, 'negative')):
            out = STAGES['L2'](_args(['--board', _b, '--placement-report',
                                      _w({'blocking': _v, 'oob_pad_count': 0,
                                          'locked_contacts': 0,
                                          'buildable': True, 'verdict': 'ok'},
                                         'asm_malformed.json')]))
            want(out.startswith('<error>') and _needle in out,
                 f'PIN: a malformed blocking count ({_v!r}) is refused')

        # L2 buildable / locked_contacts: SKILL.md non-negotiable 3.
        out = STAGES['L2'](_args(['--board', _b, '--placement-report', _w(
            {'blocking': 0, 'oob_pad_count': 0, 'locked_contacts': 0,
             'buildable': False, 'verdict': 'NOT BUILDABLE (2 pairs)'},
            'asm_nb.json')]))
        want(out.startswith('<error>') and 'NOT BUILDABLE' in out,
             'PIN: L2 refuses a close-out that says NOT BUILDABLE')
        out = STAGES['L2'](_args(['--board', _b, '--placement-report', _w(
            {'blocking': 0, 'oob_pad_count': 0, 'locked_contacts': 2,
             'buildable': True, 'verdict': 'ok'}, 'asm_lc.json')]))
        want(out.startswith('<error>') and 'locked_contacts = 2' in out,
             'PIN: L2 refuses a contact with a KiCad-LOCKED part')

        # L3 score <-> board binding, and the render the classification needs.
        out = STAGES['L3'](_args(['--board', _b, '--score', _w(
            {'blocking': 3, 'board_sha': 'f' * 64}, 's_foreign.json')]))
        want(out.startswith('<error>') and 'DIFFERENT board' in out,
             'PIN: L3 refuses a score taken on a different board')
        out = STAGES['L3'](_args(['--board', _b, '--score', _w(
            {'blocking': 3}, 's_norender.json')]))
        want(out.startswith('<error>') and '--render-json' in out,
             'PIN: L3 with blocking > 0 refuses to classify without the picture')

        # L5 score <-> board binding.
        out = STAGES['L5'](_args(['--board', _b, '--ledger', _wrong,
                                  '--score', _w({'blocking': 0,
                                                 'board_sha': 'f' * 64},
                                                's5_foreign.json')]))
        want(out.startswith('<error>') and 'DIFFERENT board' in out,
             'PIN: L5 refuses a score taken on a different board')

        # L5 close-out <-> board binding, on a REAL terminal verdict: ledger
        # rows carry _b's true sha so the recorded-board gate passes and the
        # binding check is the one that fires.
        sys.path.insert(0, ROOT)
        from board_store import sha256_file as _sha
        _okled = os.path.join(tmp, 'ok.jsonl')
        with open(_okled, 'w', encoding='utf-8') as fh:
            for i, kind in enumerate(['placement'] * 6 + ['completion'] * 6):
                fh.write(json.dumps({
                    'iteration': i, 'kind': kind, 'accepted': True,
                    'result_sha': _sha(_b),
                    'score': {'blocking': 0, 'quality': {}}}) + '\n')
        out = STAGES['L5'](_args(['--board', _b, '--ledger', _okled,
                                  '--score', _w({'blocking': 0}, 's5.json'),
                                  '--routing-close', _w(
            {'schema': 1, 'kind': 'board-complete', 'board': _other,
             'score': {'blocking': 0}, 'components': {'orphan_stubs': {}},
             'fab_floors': {'ran': True}, 'verdict': 'DONE',
             'reason': 'clean', 'ungraded': []}, 'cc_foreign.json')]))
        want(out.startswith('<error>') and 'different board' in out,
             'PIN: L5 refuses a close-out that names a different board')

    print('OK' if not bad else f'FAIL: {len(bad)}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
