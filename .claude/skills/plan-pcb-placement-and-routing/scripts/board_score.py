#!/usr/bin/env python3
"""One authoritative score for a board, so a loop can tell better from worse.

Why this exists
---------------
`place_route_loop.better()` compares `failures` and `iterations`, and BOTH come
out of route.py's own ``JSON_SUMMARY``. It never runs a checker. CLAUDE.md warns
about exactly this -- *"Routers can report false success. A router's own 'routed'
tally may come from a local/heuristic proxy while pads stay disconnected"* -- so a
round can be reported ACCEPTED while pads sit disconnected and DRC is dirty. That
is how a board went out at 39/44 nets with 762 DRC errors and 141/141 vias below
its own spec.

This script is the second opinion. It answers one question -- *is this board
better than that board?* -- with a number that cannot be produced by the thing
being graded.

It reimplements NO checking. Every component shells out to the real CLI, which is
deliberate and not laziness: `check_drc.main()` resolves the grading clearance
from the sibling ``.kicad_pro``, installs the per-layer ``.kicad_dru`` rules,
builds the per-netclass clearance map and derives the edge / hole-to-hole floors
(check_drc.py:2968-3045). Reimplementing that resolution here would drift, and a
grader that ignores the dru "will manufacture phantom flags on relaxed layers and
miss real ones on tightened layers" (CLAUDE.md). Subprocessing keeps the score
BY CONSTRUCTION equal to what `check_drc.py` itself reports.

The score
---------
Lexicographic, never a weighted sum -- a weighted sum lets a router buy off a
disconnected net with a lower via count::

    score = (blocking, quality)
    blocking = (unrouted + broken + drc + undersized + floorplan
               + assembly + impedance + length + net_widths)
    quality  = (vias, copper_mm, segments)      # only compared once blocking == 0

`blocking` must reach 0 before a board is deliverable. `quality` orders the
boards that already got there.

Vacuity
-------
"0 violations" and "0 rules ran" are different answers, and a loop that cannot
tell them apart will happily converge on a board nothing graded. Every component
reports `ran: true|false` plus a `reason` when it did not, and `blocking` is
`None` -- not 0 -- when a component that was asked for could not run.

Exit codes (deliberately the same dialect as check_floorplan.py)
    0  graded, blocking == 0
    1  crash
    2  bad arguments
    3  board state (missing file, unparseable board)
    4  graded, blocking > 0
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

# Violation types that mean "this copper is below the fab/spec floor" rather than
# "this copper is too close to that copper". check_drc emits both from one run;
# the split matters because they take different levers -- a size violation is a
# re-route at a different width/via, a clearance violation is a routing conflict.
# Source: check_drc.py's `by_type` grouping (:2801) over the 'type' key.
SIZE_TYPES = frozenset({'track-width', 'via-size', 'via-drill-size'})

# Seg-seg pairs whose violation exists ONLY because a .kicad_dru track
# rule raised the pair clearance (check_drc tags them with a distinct type and
# the binding rule's name). They are the structural, registered-floor-governed
# population -- a repo's check_dru gate is their arbiter -- so board_score
# reports them as ADVISORY, outside `blocking`. Run 6 measured why: 610 such
# pairs drowned a blocking of ~17 physical defects into an unusable 627.
RULE_PAIR_TYPES = frozenset({'segment-segment-track-rule'})

_DRC_TOTAL = re.compile(r'^FOUND (\d+) DRC VIOLATIONS', re.M)
# The per-type header carries an OPTIONAL suffix between the count and the
# colon -- `PAD-PAD violations (40) -- 32 in CONTACT:` (check_drc.py's `_ct`,
# added in 8d084b4). This regex pinned `):` and so matched nothing on any board
# with a contact-grade violation, while `_DRC_TOTAL` kept matching happily: the
# guard below saw a summary, `by_type` came back empty, and the drc component
# reported a confident `count: 0` on a board with 40 pad-pad shorts. Anything
# up to the newline is allowed between the `)` and the `:` for that reason.
_DRC_TYPE = re.compile(r'^([A-Z0-9-]+) violations \((\d+)\)[^\n]*:', re.M)
_CONN_TOTAL = re.compile(r'^FOUND (\d+) ISSUES', re.M)
_CONN_UNROUTED = re.compile(r'^\s+Unrouted nets \((\d+)\):', re.M)
_CONN_BROKEN = re.compile(r'^\s+Connectivity issues \((\d+)\):', re.M)
_CONN_COMPONENTS = re.compile(r'^\s+Disconnected components: (\d+)', re.M)


def krt_dir() -> str:
    """The KiCadRoutingTools clone whose checkers we grade with.

    $PCB_KICADROUTINGTOOLS wins so the skill works against an outside repo; the
    walk up from this file covers the in-repo case. Raise rather than fall back:
    scoring with a different clone's checkers would describe the wrong engine.
    """
    env = os.environ.get('PCB_KICADROUTINGTOOLS', '').strip().strip('"')
    if env:
        if not _is_clone(env):
            raise SystemExit(f"PCB_KICADROUTINGTOOLS={env!r} has no check_drc.py "
                             f"-- not a KiCadRoutingTools clone")
        return env
    # <krt>/.claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py -> four up
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(here))))
    if _is_clone(root):
        return root
    raise SystemExit(
        "Cannot locate KiCadRoutingTools. Set PCB_KICADROUTINGTOOLS to the clone, "
        "or run this script from inside one.")


_TOOL_DIRS = ('', 'py_router', 'py_tools', 'py_placer')


def _is_clone(root: str) -> bool:
    """A KiCadRoutingTools clone in EITHER layout (flat, or #522 py_router/)."""
    return any(os.path.isfile(os.path.join(root, d, 'check_drc.py'))
               for d in _TOOL_DIRS)


def _tool_path(root: str, tool: str) -> str:
    """Absolute path to `tool`, wherever the #522/placement-split layout put it.

    Falls back to the flat join so the error message a missing tool produces
    still names the place the caller expected it.
    """
    for d in _TOOL_DIRS:
        p = os.path.join(root, d, tool)
        if os.path.isfile(p):
            return p
    return os.path.join(root, tool)


def run_tool(root: str, tool: str, *args) -> tuple:
    """(returncode, combined output). -X utf8 for the Ω/µ the tools print.

    KRT_NO_BANNER: the child instruments self-echo CMD/EXIT (run-4 B1) --
    inside a COMPOSED run those lines would land mid-document and read as
    the outer gate's exit (measured: a blocking-153 score log carrying an
    inner checker's EXIT=0). The outer invocation is the evidence unit."""
    cmd = [sys.executable, '-X', 'utf8', _tool_path(root, tool)] + [str(a) for a in args]
    env = dict(os.environ, KRT_NO_BANNER='1')
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       encoding='utf-8', errors='replace', env=env)
    return p.returncode, p.stdout


def skipped(reason: str) -> dict:
    """A component that did not run. `count` is None, never 0 (see Vacuity)."""
    return {'ran': False, 'reason': reason, 'count': None}


def _unescape_net_name(s: str) -> str:
    """Raw s-expression string body -> the net name every other field uses.

    Delegates to the parser so the two cannot drift; the inline fallback is for
    the case where this script is run against an outside clone whose root has
    not been put on sys.path yet. Both do the same thing KiCad does: `\\\\` is one
    backslash and `\\"` is one quote.
    """
    try:
        from kicad_parser import _unescape_kicad_string
        return _unescape_kicad_string(s)
    except Exception:                                           # noqa: BLE001
        if '\\' not in s:
            return s
        return s.replace('\\\\', '\\').replace('\\"', '"')


# WHAT `poured_nets` MEANS, carried in the payload beside the value itself.
# It is "this net has at least one zone on the board", which is exactly the
# fact that picks the REPAIR HANDLER for a broken net -- and it is NOT a list
# of plane nets. Measured on neo6502: the 61 zone-backed nets covered 332 of
# 545 pads (72%), including all of /A0../A15, because that board pours many
# signal nets; a consumer that read it as "the planes, safe to ignore" removed
# most of the board from its own analysis. Emitting the sentence next to the
# list is the point -- a comment in this file is not visible to the consumer
# reading the JSON, and that is who got this wrong.
POURED_NETS_MEANING = (
    'nets with at least one zone on the board, i.e. a broken one of these is '
    "route_disconnected_planes' job rather than route.py's. This is NOT a list "
    'of plane/power nets and is NOT a safe --ignore-nets population: a board '
    'that pours signal nets puts them in here too (measured: 332 of 545 pads).')


def score_connectivity(root: str, board: str) -> dict:
    """Unrouted and broken nets, from check_connected.py.

    This is the authoritative, zone/fill-aware answer -- it reconciles
    pour-backed nets against KiCad's exact fill in both directions, which the
    router's own tally does not do.
    """
    rc, out = run_tool(root, 'check_connected.py', board)
    if 'ALL NETS FULLY CONNECTED' in out:
        return {'ran': True, 'count': 0, 'unrouted': 0, 'broken': 0, 'nets': []}
    m = _CONN_TOTAL.search(out)
    if not m:
        return skipped(f'check_connected.py produced no summary (rc={rc})')
    unrouted = int(u.group(1)) if (u := _CONN_UNROUTED.search(out)) else 0
    broken_nets = int(b.group(1)) if (b := _CONN_BROKEN.search(out)) else 0
    # `broken` counts SEPARATIONS, not nets. Counting nets makes a net split into
    # 10 pieces score the same as one split into 2, so the score ranked a board
    # that had just taken GND from 10 components to 6 (23 stranded pads to 5) as
    # WORSE, and the run had to be delivered against its own gate. Each net needs
    # (components - 1) more joins to be whole, so that is the honest unit: it is
    # 0 exactly when the net is connected, and it falls monotonically as repairs
    # land. `broken_nets` is kept for the report.
    comps = [int(c) for c in _CONN_COMPONENTS.findall(out)]
    broken = sum(max(0, c - 1) for c in comps) if comps else broken_nets
    # Net names, so the ledger can say WHICH nets failed and a later round can
    # tell "the same nets every time" (parameters) from "different nets"
    # (congestion) -- the Step 9 classification needs this distinction.
    unrouted_names = re.findall(r'^\s{4}(\S+) \(\d+ pads\)', out, re.M)
    broken_names = re.findall(r'^\s{2}(\S+) \(net \d+\):', out, re.M)
    nets = unrouted_names + broken_names

    # PER-NET DETAIL FOR `broken`, because a COUNT IS NOT A WORK LIST. `unrouted`
    # is actionable from its names alone -- the net has no copper, route it. A
    # `broken` net needs three more things before anyone can act: WHICH net, how
    # many pieces (a 5-way split and a 2-way split are different jobs), and WHERE
    # the stranded pads are. All three are already in check_connected's output and
    # were being parsed and dropped, so `blocking_by.broken: 14` was a number with
    # nothing behind it. Measured consequence: a run drove `unrouted` to 0 using
    # the per-net width detail as its model, and left `broken` untouched at 14
    # across two iterations because it had nothing equivalent to work from.
    #
    # The pad REF matters as much as the count. A break whose stranded pad sits on
    # a do-not-fit part is not a functional defect and must not be chased forever;
    # a break on a plane net wants route_disconnected_planes, not route.py. The
    # ref is what lets the caller tell those apart.
    detail, cur = {}, None
    for line in out.splitlines():
        if (mm := re.match(r'^\s{2}(\S+) \(net \d+\):\s*$', line)):
            cur = mm.group(1)
            detail[cur] = {'components': None, 'joins_needed': None,
                           'stranded_pads': []}
        elif cur and (mc := re.match(r'^\s+Disconnected components:\s*(\d+)', line)):
            detail[cur]['components'] = int(mc.group(1))
            detail[cur]['joins_needed'] = max(0, int(mc.group(1)) - 1)
        elif cur and (mp := re.match(
                r'^\s+\(([-\d.]+),\s*([-\d.]+)\) on (\S+)(?:\s+\[(\S+)\])?', line)):
            detail[cur]['stranded_pads'].append(
                {'x': float(mp.group(1)), 'y': float(mp.group(2)),
                 'layer': mp.group(3), 'ref': mp.group(4)})

    # NAME THE TOOL, not just the defect. Which step fixes a break is decided by
    # ONE fact the board already carries: is the net POURED? A stranded pad on a
    # plane net cannot be reached by route.py at all -- it needs a tap via, which
    # is route_disconnected_planes' job -- and a run that reaches for route.py on
    # everything watches the count sit still. Measured: `broken` held at 14 across
    # two iterations of route.py calls, then fell to 11 in ONE
    # route_disconnected_planes call once the plane nets were separated out.
    #
    # Poured-ness is read off the board's own zones, so this is a fact and not a
    # guess. Everything else is `route`; the DNF case stays a human call, which is
    # what the stranded pad's `ref` is in the list for.
    poured = set()
    try:
        with open(board, encoding='utf-8', errors='replace') as f:
            txt = f.read()
        # A zone names its net as EITHER `(net_name "GND")` or `(net "GND")`
        # depending on the writer -- KiCad 10 emits the second, and matching only
        # the first classified a poured GND as `route` and sent the caller to a
        # tool that cannot tap a pour. `(net 4)` is the numeric form and is
        # deliberately not matched here: it identifies nothing without the net
        # table, and a wrong name is worse than a missing one.
        # `(?!_)` keeps `(zone_connect 2)` -- a per-pad property that appears
        # hundreds of times inside footprints -- out of the scan.
        #
        # UNESCAPE, because this is the one net-name field in the whole payload
        # read out of the RAW FILE TEXT rather than through the parser, and the
        # file stores `/GPIO10\OE3#` as `/GPIO10\\OE3#`. Every other field here
        # comes via check_connected/kicad_parser, which unescape. Measured on
        # neo6502: `poured_nets` published `/GPIO10\\OE3#` while `unrouted.nets`
        # in the SAME json.dump published `/GPIO10\OE3#` -- one file, one writer,
        # two spellings of one net. A consumer built `--ignore-nets` from it,
        # 10 of 61 names matched nothing, and the render came back hpwl +45.6%
        # / crossings +129.5% on an identical board with nothing flagging it.
        # The capture stays `[^"]+` -- deliberately the SAME body pattern
        # kicad_parser uses for the net table (:2234), escaped-quote blind spot
        # included. Widening it here alone would make this field disagree with
        # the parser on a net whose name contains `\"`, which is the exact class
        # of defect the net-name audit exists to catch; the two must be wrong
        # together or right together, and the parser is the one that decides.
        for zb in re.finditer(r'\(zone(?!_)', txt):
            seg = txt[zb.start():zb.start() + 400]
            if (zn := re.search(r'\(net(?:_name)? "([^"]+)"\)', seg)):
                poured.add(_unescape_net_name(zn.group(1)))
    except OSError:
        pass
    for name, v in detail.items():
        v['handler'] = ('route_disconnected_planes' if name in poured
                        else 'route')

    return {'ran': True, 'count': int(m.group(1)), 'unrouted': unrouted,
            'broken': broken, 'broken_nets': broken_nets,
            'components_per_broken_net': comps, 'nets': sorted(set(nets)),
            'unrouted_net_names': sorted(set(unrouted_names)),
            'poured_nets': sorted(poured),
            'poured_nets_meaning': POURED_NETS_MEANING,
            'broken_detail': detail}


def unrouted_shape(board: str, unrouted_names) -> dict:
    """Which unrouted nets are PLACEMENT-blocked, and which are merely open.

    `unrouted` is one bucket holding two different failure shapes, and the
    router's own filter already separates them for free. A net with fewer than
    2 ON-BOARD pads is refused outright by `net_queries.filter_routable_nets`
    with a boxed warning -- no router setting will ever route it, so retrying it
    in the router is exactly the mistake the placement+routing skill's
    non-negotiable 2 exists to prevent. Measured on run 10: 13 of 57 `unrouted`
    nets were this shape, and the loop spent a routing pass plus a plane-repair
    pass discovering it.

    This does NOT change `count`, and that is the property the change must not
    break. The 13 are real, current damage -- they are just damage a router
    cannot take a lever to. Fixing it in `check_connected.py` instead would
    move `blocking` in every ledger already recorded, and would be wrong on the
    merits: an off-board pad IS a defect, so 57 is the honest total. What was
    missing is the SHAPE, not the number.
    """
    from kicad_parser import parse_kicad_pcb
    from check_drc import make_off_board_test
    from net_queries import routable_pad_count
    try:
        pcb = parse_kicad_pcb(board)
        off = make_off_board_test(pcb.board_info)
    except Exception as exc:                                   # noqa: BLE001
        return {'error': f'{type(exc).__name__}: {exc}'}
    by_name = {n.name: nid for nid, n in pcb.nets.items() if n.name}
    blocked, open_nets, refs = {}, [], set()
    for name in unrouted_names or ():
        nid = by_name.get(name)
        pads = getattr(pcb, 'pads_by_net', {}).get(nid, ()) if nid else ()
        if nid is not None and routable_pad_count(pcb, nid, off) < 2 <= len(pads):
            bad = sorted({p.component_ref for p in pads
                          if off(p.global_x, p.global_y)})
            blocked[name] = bad
            refs.update(bad)
        else:
            open_nets.append(name)
    return {'placement_blocked': blocked,
            'placement_blocked_refs': sorted(refs),
            'open': sorted(open_nets)}


#: check_assembly's five `not_buildable` conjuncts, by the JSON key each one
#: publishes (check_assembly.py:508-510). `blocking` -- pad INTERSECTIONS -- is
#: the first of them and is the only one this component used to read (#918).
#:
#: THEY ARE NOT DISJOINT AND THEY ARE NOT ONE CURRENCY, which is why `count`
#: below does not add them up:
#:
#:   * `locked_contacts` is a strict SUBSET of `blocking`. `locked_ref` is set
#:     at exactly one site, inside the pad-intersection channel, so every
#:     locked-contact pair is already a blocking pair --
#:     `tests/test_run8_locked_contact.py` asserts in so many words that it is
#:     "a second channel, not a re-count".
#:   * `coincident_origins` counts GROUPS, not pairs: an N-part stack is one
#:     group and N(N-1)/2 potential pairs.
#:   * `containment_blocking` counts `fab`-kind pairs, a different geometry
#:     channel from `blocking`'s pad intersections -- but routinely the SAME
#:     ref pair. Measured on a perturbed corpus board, 3 of 11 containments
#:     named a pair already in `blocking`; on the fixture
#:     `tests/test_918_assembly_verdict.py` builds, one stacked-capacitor
#:     defect appears as a group AND as a containment.
#:
#: `courtyard_blocking_gating` is null here BY CONSTRUCTION: it is the
#: moved-vs-baseline subset of the courtyard census, and board_score passes no
#: --baseline, so check_assembly publishes null rather than 0. Reported as
#: unmeasured, never counted as clean.
ASSEMBLY_CONJUNCTS = ('blocking', 'locked_contacts', 'coincident_origins',
                      'containment_blocking', 'courtyard_blocking_gating')

#: The conjuncts that can ACTUALLY flip the verdict while `blocking` is 0, in
#: this scorer's invocation. Two, not four:
#:   * `locked_contacts` cannot -- it is a subset of `blocking` (above), so a
#:     locked contact implies `blocking >= 1` and the board never had 0;
#:   * `courtyard_blocking_gating` cannot -- it is `[]` unless `--baseline` was
#:     passed, and board_score never passes one.
#: Written down because the issue, and this file's first draft, claimed all
#: four -- and a motivating case that cannot occur is not a motivating case.
ASSEMBLY_LIVE_CONJUNCTS = ('coincident_origins', 'containment_blocking')


def assembly_component(doc: dict, rc: int) -> dict:
    """check_assembly's VERDICT, not one of its five conjuncts (#918).

    `not_buildable` is `blocking or locked_contact or stack_groups or
    containment_blocking or courtyard_gating`. This component read `blocking`
    ALONE, so a board unbuildable through a coincident-origin stack or a
    containment contributed 0 to `blocking` -- the headline the whole loop
    ranks and stops on. check_assembly publishes `buildable` and `verdict` for
    exactly this reader, and the comment above them names this defect verbatim,
    so the verdict is READ here and the disjunction is never re-derived.

    `count` IS `blocking`, floored at 1 when the verdict says NOT BUILDABLE.
    It is deliberately NOT the sum of the conjuncts, and that was the first
    draft's bug: `locked_contacts` is a subset of `blocking` and
    `containment_blocking` routinely names a ref pair already in it, so adding
    them counts one defect twice (measured on a perturbed corpus board: the
    sum reported 44 where there were 38 distinct defective ref pairs and 30
    pad intersections; and 2 on the stacked-capacitor fixture in
    `tests/test_918_assembly_verdict.py`, where one defect appears as a group
    AND as a containment). And they are not one currency -- pairs, a subset of
    those pairs, and GROUPS -- so their total is a number with no unit.

    The floor of 1 is therefore the whole mechanism, not a safety net: it says
    "this board is not buildable" without inventing a magnitude. All five
    conjuncts are published in `conjuncts` for a reader who wants to know
    WHICH fired, and `count_basis` names how `count` was reached, so the
    number is falsifiable from its own payload.

    Two refusals rather than a quiet answer:

      * no `buildable` key -- an older check_assembly is a DIFFERENT
        instrument, and re-deriving the conjunction from whatever keys it did
        publish is the exact thing this change removes;
      * the exit code and the verdict DISAGREE (rc 4 with buildable true, rc 0
        with buildable false) -- the instrument contradicting itself, which is
        never a number to report. This is the self-check
        tests/test_board_score_floorplan_severity.py ends on, from the other
        side: there the scorer had to agree with the grader, here the grader
        has to agree with itself before the scorer will read it.

    Pure, so both arms are unit-testable without a board.
    """
    buildable = doc.get('buildable')
    if not isinstance(buildable, bool):
        return skipped(
            "check_assembly published no `buildable` key: `blocking` alone is "
            "1 of its 5 not_buildable conjuncts (check_assembly.py:508-510), "
            "and this component will not re-derive the other four")
    if (rc == 4) != (not buildable):
        return skipped(
            f"check_assembly contradicts itself: exit {rc} with "
            f"buildable={buildable!r} (it exits 4 exactly when the verdict is "
            f"NOT BUILDABLE). Reporting either number would be reporting an "
            f"instrument that disagrees with itself")
    conjuncts = {k: doc.get(k) for k in ASSEMBLY_CONJUNCTS}
    measured = {k: v for k, v in conjuncts.items()
                if isinstance(v, int) and not isinstance(v, bool)}
    unmeasured = sorted(k for k in conjuncts if k not in measured)
    blocking = int(doc.get('blocking') or 0)
    fired = sorted(k for k, v in measured.items() if v and k != 'blocking')
    if buildable:
        count, basis = blocking, 'blocking (buildable)'
    elif blocking:
        count = blocking
        basis = (f'blocking ({blocking}); NOT BUILDABLE, and the conjuncts are '
                 f'not summed -- they overlap and are not one currency'
                 + (f' (also fired: {", ".join(fired)})' if fired else ''))
    else:
        # THE case this component exists for: NOT BUILDABLE at blocking 0.
        count = 1
        basis = ('1: NOT BUILDABLE with blocking 0, so the verdict rests '
                 'entirely on '
                 + (', '.join(fired) if fired else
                    'a conjunct this scorer cannot see')
                 + '. One, not a sum: the conjuncts overlap (locked_contacts '
                   'is a subset of blocking; a containment routinely names a '
                   'pair already in it) and are not one currency (pairs vs '
                   'GROUPS), so their total has no unit')
    # WHICH conjuncts can actually reach this branch, so a reader is not sent
    # looking for a case that cannot happen.
    live = [k for k in ASSEMBLY_LIVE_CONJUNCTS if measured.get(k)]
    # `courtyard_gating_basis` is the producer's own word for whether conjunct
    # 5 was armed. READ it rather than asserting it: this function is public
    # and pure, so it can legitimately be handed a document produced WITH
    # --baseline, and a payload whose thesis is "not measured must never read
    # as measured" must not hardcode an armedness it never measured.
    _cg_basis = doc.get('courtyard_gating_basis')
    return {'ran': True, 'count': count, 'count_basis': basis,
            'buildable': buildable, 'verdict': doc.get('verdict'),
            'conjuncts': conjuncts,
            'conjuncts_unmeasured': unmeasured,
            'conjuncts_fired': fired,
            'live_conjuncts_fired': live,
            'courtyard_gating_armed':
                isinstance(conjuncts['courtyard_blocking_gating'], int),
            'courtyard_gating_basis': _cg_basis,
            'courtyard_gating_reason': (
                None if isinstance(conjuncts['courtyard_blocking_gating'], int)
                else 'no --baseline was passed, so check_assembly\'s fifth '
                     'conjunct (moved-vs-baseline courtyard interpenetration) '
                     'is unarmed and publishes null. board_score never passes '
                     'one, so it is unarmed on every board this scorer grades'),
            'advisory_pairs': int(doc.get('advisory') or 0),
            'waived_pairs': int(doc.get('waived') or 0),
            'pairs': doc.get('blocking_pairs') or [],
            'locked_contact_pairs': doc.get('locked_contact_pairs') or [],
            'coincident_origin_groups': doc.get('coincident_origin_groups') or [],
            'containments': doc.get('containments') or []}


def score_assembly(root: str, board: str, intent: str, tmp: str,
                   clearance=None) -> dict:
    """Blocking BODY pairs (run-6): two footprints' pad copper in the same
    space -- physically unbuildable, invisible to every copper checker (the
    shipped C14-on-R14 stack). Runs check_assembly.py, which needs NO
    intent to be meaningful (--intent only adds authored waivers), so this
    component grades on every board that the tool can read -- the floorplan
    path can be vacuous by self-blessed budget; this one cannot.

    It is no longer unconditional, and the exception is deliberate: an older
    check_assembly that publishes no `buildable`, or one whose exit code and
    verdict disagree, is REFUSED by `assembly_component` and lands in
    `ungraded`. A different instrument reporting a number this scorer would
    have to re-derive is not a measurement (#918).

    Runs the tool; `assembly_component` reads its document (#918)."""
    out = os.path.join(tmp, 'assembly.json')
    args = [board, '--json', out]
    if intent:
        args += ['--intent', intent]
    # Forward --clearance, exactly as score_drc does one function below. Without
    # it check_assembly falls back to routing_defaults.CLEARANCE (a flat 0.25)
    # and does NOT read the board, so every assembly component this scorer has
    # ever published was graded at 0.25 regardless of the board's own floor --
    # stricter than the thing being graded on any board below 0.25 (measured
    # elsewhere: pad_conflicts 96 at the default vs 39 at the board's floor).
    # `blocking` itself is largely clearance-insensitive, which is why this went
    # unnoticed; `advisory_pairs` and `waived_pairs` are not.
    if clearance is not None:
        args += ['--clearance', str(clearance)]
    rc, text = run_tool(root, 'check_assembly.py', *args)
    if rc not in (0, 4) or not os.path.exists(out):
        return skipped(f'check_assembly rc {rc}: {text.strip()[-200:]}')
    try:
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    except Exception as exc:
        return skipped(f'check_assembly json unreadable: {exc}')
    return assembly_component(doc, rc)


def score_drc(root: str, board: str, clearance=None, sizes=None) -> tuple:
    """(drc, undersized, rule_pairs) -- physical clearance violations,
    sub-floor copper, and .kicad_dru track-rule-governed pairs (advisory).

    Both come from ONE check_drc run, split on the violation type. Omitting
    --clearance is the norm and not an oversight: check_drc then reads the
    sibling .kicad_pro Default class, which is the floor the board was actually
    routed to. Passing a guessed round number manufactures phantom violations
    on legitimately tight copper (CLAUDE.md).

    `sizes` is the opposite case, and it is the one that shipped a broken board.
    check_drc defaults its size floors to the FAB minimum derived from the layer
    count -- correct when the fab is the only constraint, and blind when the
    board's own spec is TIGHTER. Measured on a 2-layer board whose spec asked
    for 0.6 mm vias with a 0.15 mm annular ring: every one of its 141 vias
    violated that and nothing caught it, because 0.25 mm clears the 2-layer fab
    floor. Pass the spec's numbers whenever the spec has any.
    """
    args = [board]
    if clearance is not None:
        args += ['-c', str(clearance)]
    for flag, val in (sizes or {}).items():
        if val is not None:
            args += [flag, str(val)]
    # --max-print 0 prints every violation of every type, so the per-type header
    # counts are complete rather than capped at the default 20.
    args += ['--max-print', '0']
    rc, out = run_tool(root, 'check_drc.py', *args)
    if 'NO DRC VIOLATIONS FOUND' in out:
        return ({'ran': True, 'count': 0, 'by_type': {}, 'graded_at': _graded_at(out)},
                {'ran': True, 'count': 0, 'by_type': {}},
                {'ran': True, 'count': 0, 'by_type': {}})
    if not _DRC_TOTAL.search(out):
        r = skipped(f'check_drc.py produced no summary (rc={rc})')
        return r, dict(r), dict(r)
    by_type = {t.lower(): int(n) for t, n in _DRC_TYPE.findall(out)}
    # FAIL CLOSED on a parse that disagrees with itself. `_DRC_TOTAL` and
    # `_DRC_TYPE` read the SAME output, so a positive total with no per-type
    # lines means this parser no longer understands check_drc's format -- not
    # that the board is clean. Reporting 0 there is the worst available answer:
    # `blocking` is the routing half's accept rule and this report's headline,
    # so a silently-zeroed drc component lets a run "improve" while shorts pile
    # up unseen. That is exactly how a 40-violation board scored drc 0.
    _total = int(_DRC_TOTAL.search(out).group(1))
    if _total > 0 and not by_type:
        # ran=True with count=None routes this to UNKNOWN (blocking None,
        # exit 4) -- the same idiom score_impedance uses, and for the same
        # reason. `skipped()` would be WRONG here: it sets ran=False, which
        # `unknown` filters out, so blocking would sum without this component
        # and hand back the very number the drift produced (264 on the board
        # that motivated this). A parser that cannot read its input must make
        # the scalar unusable, not merely annotate it.
        r = {'ran': True, 'count': None, 'by_type': {},
             'reason': f'check_drc.py reported {_total} violations but this '
                       f'parser matched no per-type header -- format drift, '
                       f'NOT a clean board (rc={rc})'}
        return r, dict(r), dict(r)
    size = {t: n for t, n in by_type.items() if t in SIZE_TYPES}
    rule = {t: n for t, n in by_type.items() if t in RULE_PAIR_TYPES}
    clear = {t: n for t, n in by_type.items()
             if t not in SIZE_TYPES and t not in RULE_PAIR_TYPES}
    return ({'ran': True, 'count': sum(clear.values()), 'by_type': clear,
             'graded_at': _graded_at(out)},
            {'ran': True, 'count': sum(size.values()), 'by_type': size},
            {'ran': True, 'count': sum(rule.values()), 'by_type': rule})


def _graded_at(out: str):
    """The clearance check_drc actually graded at -- quote it, never assume it."""
    m = re.search(r'Grading at clearance ([\d.]+) mm', out)
    return float(m.group(1)) if m else None


def score_floorplan(root: str, board: str, intent: str, tmp: str) -> dict:
    """check_floorplan --intent. Exit 4 = graded and violated.

    Carries rules_run/rules_skipped through, because an intent that resolves to
    zero rules grades clean by vacuity -- and a typo'd block is exactly the
    failure the grader exists to catch.

    `count` is the ERROR-severity violations only, because it is summed into
    `blocking` and `blocking` is what `converge` requires to reach 0. Counting
    every violation made this instrument CONTRADICT the one it reads: measured
    on a corpus board with one warn-demoted `block_unresolved`, check_floorplan
    reported `errors 0, warnings 1, pass true` and exited 0, while this function
    returned count 1 and drove `blocking` to 84 and exit 4. A warn was therefore
    a permanent blocker with no way to clear it -- the intent schema has no
    `off` severity, and `--exit-zero` would not have helped either -- it
    suppresses check_floorplan's EXIT CODE and disables no rule, while this
    function reads the JSON violation list rather than the exit code -- so
    `severity: warn` did not mean what floorplan.py:57 says it means ("reported
    and does not fail the run").

    Warnings stay REPORTED, in their own keys, so demoting a rule still surfaces
    it. A missing `severity` counts as an error: this must never silently
    un-block something on an older JSON.
    """
    if not intent:
        return skipped('no --intent given; the floorplan is ungraded')
    if not os.path.isfile(intent):
        return skipped(f'intent file not found: {intent}')
    out_json = os.path.join(tmp, 'floorplan.json')
    rc, out = run_tool(root, 'check_floorplan.py', board, '--intent', intent,
                       '--json', out_json, '-q')
    if rc in (2, 3) or not os.path.isfile(out_json):
        return skipped(f'check_floorplan.py exited {rc}: {out.strip()[-200:]}')
    with open(out_json, encoding='utf-8') as f:
        d = json.load(f)
    viols = d.get('violations') or []
    errors = [v for v in viols if (v.get('severity') or 'error') == 'error']
    warns = [v for v in viols if (v.get('severity') or 'error') != 'error']
    return {'ran': True, 'count': len(errors),
            'rules_run': len(d.get('rules_run') or []),
            'rules_skipped': list((d.get('rules_skipped') or {}).keys()),
            'violations': [v.get('rule') for v in errors],
            # Reported, never summed into `blocking`. Printed even when zero,
            # so "no warnings" and "warnings not looked at" cannot look alike.
            'warnings': len(warns),
            'warning_rules': sorted({str(v.get('rule')) for v in warns})}


def score_impedance(root: str, board: str, nets, tmp: str) -> dict:
    """check_impedance -- reference-plane continuity and declared-gap audit."""
    if not nets:
        return skipped('no --impedance-nets given; impedance is ungraded')
    out_json = os.path.join(tmp, 'impedance.json')
    rc, out = run_tool(root, 'check_impedance.py', board, '--nets', *nets,
                       '--json', out_json)
    if not os.path.isfile(out_json):
        return skipped(f'check_impedance.py exited {rc}: {out.strip()[-200:]}')
    with open(out_json, encoding='utf-8') as f:
        d = json.load(f)
    tot = d.get('totals') or {}
    if not tot.get('nets_analyzed'):
        # Asked for, but nothing matched -- that is a FINDING (the globs are
        # wrong, or the nets are unrouted), not a pass. ran=True with count=None
        # routes this to UNKNOWN (blocking None, exit 4), never to `ungraded`:
        # the comma-joined form of --impedance-nets silently zero-matched for
        # four whole runs on one board while blocking summed without the
        # component and the exit code read 0 (test-board run 5, journal [11]).
        return {'ran': True, 'count': None,
                'reason': 'no routed nets matched --impedance-nets'}
    return {'ran': True, 'count': int(tot.get('nets_with_crossing') or 0),
            'nets_analyzed': tot.get('nets_analyzed'),
            'crossings': tot.get('crossings'),
            'segments_over_void': tot.get('segments_over_void')}


def score_length(board: str, groups_file: str) -> dict:
    """Length-match spread per declared group, via net_queries (no new geometry).

    `--length-groups` is a JSON file::

        {"BYTE0": {"nets": ["/DQ0", "/DQ1"], "tolerance_mm": 0.1},
         "USB":   {"nets": ["/USB_DP", "/USB_DM"], "tolerance_mm": 0.05,
                   "mode": "pin_pair"}}

    `mode: "pin_pair"` measures the driver->receiver PATH instead of total net
    copper. Use it for any multipoint or stubbed net: total copper sums every
    branch and matches no real signal path.
    """
    if not groups_file:
        return skipped('no --length-groups given; length matching is ungraded')
    if not os.path.isfile(groups_file):
        return skipped(f'length-groups file not found: {groups_file}')
    from kicad_parser import parse_kicad_pcb
    from net_queries import net_copper_lengths, pin_pair_path_length

    with open(groups_file, encoding='utf-8') as f:
        groups = json.load(f)
    pcb = parse_kicad_pcb(board)
    by_name = {n.name: nid for nid, n in pcb.nets.items()}
    failures, unmeasured, detail = 0, 0, {}
    for gname, spec in groups.items():
        names = [n for n in spec.get('nets', [])]
        tol = float(spec.get('tolerance_mm', 0.1))
        ids = [by_name[n] for n in names if n in by_name]
        missing = [n for n in names if n not in by_name]
        if len(ids) < 2:
            detail[gname] = {'skipped': f'fewer than 2 of its nets exist on the '
                                        f'board (missing: {missing})'}
            failures += 1          # a group naming nets that do not exist is a
            continue               # finding about the spec, not a pass
        if spec.get('mode') == 'pin_pair':
            lengths = {}
            for n in names:
                if n not in by_name:
                    continue
                pads = pcb.pads_by_net.get(by_name[n]) or []
                L = (pin_pair_path_length(pcb, by_name[n], pads[0], pads[1])
                     if len(pads) >= 2 else None)
                if L is not None:
                    lengths[n] = L
            if len(lengths) < 2:
                # NOT a failure. "Not routed yet" is what `unrouted`/`broken`
                # already measure, and counting it here too made every 0-copper
                # board score length=1 per group -- a permanent phantom blocker
                # that no routing could ever clear. Distinct from the
                # missing-nets case above, which IS a finding about the spec.
                detail[gname] = {'skipped': 'no track path between the pad pair '
                                            '(plane-only or broken)'}
                unmeasured += 1
                continue
        else:
            got = net_copper_lengths(pcb, ids)
            lengths = {n: got[by_name[n]] for n in names if n in by_name}
        spread = max(lengths.values()) - min(lengths.values())
        ok = spread <= tol
        failures += 0 if ok else 1
        detail[gname] = {'spread_mm': round(spread, 4), 'tolerance_mm': tol,
                         'pass': ok, 'missing_nets': missing,
                         'worst': max(lengths, key=lengths.get)}
    if unmeasured and unmeasured == len(detail):
        # Nothing could be measured at all -- report UNGRADED rather than a
        # clean 0, so the gate cannot be passed by a board nothing examined.
        return dict(skipped('no group had a measurable track path yet'),
                    groups=detail)
    return {'ran': True, 'count': failures, 'groups': detail,
            'unmeasured_groups': unmeasured}



def score_net_widths(board: str, spec_file: str) -> dict:
    """Per-net REQUIRED widths -- the gap `undersized` structurally cannot see.

    `undersized` comes from check_drc's size floors, which are BOARD-WIDE
    minima: they answer "is any copper thinner than X?". A spec that demands a
    PARTICULAR net be at least a given width is a different question, and the
    difference is not academic -- it is the one that shipped. Measured:
    `undersized` read 0 while 47 segments breached a >=0.4 mm rail requirement,
    and a pair spec'd at 0.8 mm that came out at 0.16 mm would not be caught
    either, because 0.16 mm clears every board-wide floor.

    `spec_file` is a JSON file of ``{"<net glob>": <min mm>}``, e.g.::

        {"USB_D*": 0.8, "VCC3V3": 0.4, "VBUS": 0.4, "XIN": 0.15}

    First matching glob wins, so list the specific patterns before the broad
    ones. Nets with no copper are not counted here -- that is `unrouted`'s job.
    """
    if not spec_file:
        return skipped('no --net-min-widths given; per-net widths are ungraded')
    if not os.path.isfile(spec_file):
        return skipped(f'net-min-widths file not found: {spec_file}')
    import fnmatch
    from collections import defaultdict
    from kicad_parser import parse_kicad_pcb

    with open(spec_file, encoding='utf-8') as fh:
        want = json.load(fh)
    # Non-numeric values are annotations (a "_comment" key), not net patterns;
    # letting them through pollutes patterns_matching_no_routed_net -- the
    # exact field a reader scans for typo'd globs. Same idiom as check_dru's
    # floors filter.
    want = {k: v for k, v in want.items() if isinstance(v, (int, float))}
    pcb = parse_kicad_pcb(board)
    by_id = {n.net_id: n.name for n in pcb.nets.values()}
    seen = defaultdict(list)
    for seg in pcb.segments:
        name = by_id.get(seg.net_id)
        if name:
            seen[name].append(seg.width)

    failures, detail = 0, {}
    for name, widths in sorted(seen.items()):
        req = next((mm for pat, mm in want.items() if fnmatch.fnmatch(name, pat)),
                   None)
        if req is None:
            continue
        under = [w for w in widths if w < float(req) - 1e-9]
        if under:
            failures += 1
            detail[name] = {'required_mm': float(req),
                            'narrowest_mm': round(min(widths), 4),
                            'segments_under': len(under),
                            'segments_total': len(widths)}
    unmatched = [p for p in want
                 if not any(fnmatch.fnmatch(n, p) for n in seen)]
    return {'ran': True, 'count': failures, 'nets': detail,
            'patterns_matching_no_routed_net': unmatched}

def _floors(board: str, sizes: dict) -> dict:
    """The size floors this score was graded against, and where each came from.

    Only `components.drc.graded_at` survived before -- and that is scraped from
    check_drc's stdout, not computed -- so `min_track_width`,
    `min_via_diameter`, `min_via_drill` and `min_via_annular_width` appeared in
    no score payload at any row. Two consequences, both measured on run 9:

      * an A/B across runs is incomparable on four of five floors, and nothing
        says so; and
      * the DRC writeback lowers a project's own floors to whatever was routed
        (run 9: track 0.2->0.1, via 0.5->0.25, annular 0.1->0.05, clearance
        0.2->0.09, and only ONE of the four steps that did it printed a
        warning), so a later score grades against a rule the run itself moved
        and reads clean. `check_complete --authored-from` catches that, but the
        score should carry the evidence rather than requiring a second tool.

    `requested` is what the caller passed; `board` is what the project declares
    now. They differ exactly when a spec floor is tighter than the board's own.
    """
    out = {'requested': {k.lstrip('-').replace('-', '_'): v
                         for k, v in (sizes or {}).items() if v is not None}}
    try:
        import list_nets
        dr = list_nets.read_design_rules(board)
        con = (dr or {}).get('constraints') or {}
        out['board'] = {k: con.get(k) for k in (
            'min_clearance', 'min_track_width', 'min_via_diameter',
            'min_via_annular_width', 'min_hole_to_hole',
            'min_through_hole_diameter', 'min_copper_edge_clearance')}
        cls = ((dr or {}).get('classes') or {}).get('Default') or {}
        out['default_netclass'] = {k: cls.get(k) for k in
                                   ('clearance', 'track_width',
                                    'via_diameter', 'via_drill')}
        # run-12 Tier 1.3. `board` and `default_netclass` above go all-None on a
        # board that declares nothing, which reads identically to "the accessor
        # failed" and identically across two different boards. Name the state:
        # every floor the components were graded at is then a fallback each
        # checker chose for itself, not this board's own. Measured on a board
        # shipping no .kicad_pro at all -- a whole baseline was graded that way
        # and nothing in the transcript said so.
        out['declares_no_floor'] = not ((dr or {}).get('classes')
                                        or (dr or {}).get('constraints'))
        out['source'] = (dr or {}).get('source')
    except Exception as exc:                                    # noqa: BLE001
        out['error'] = f'{type(exc).__name__}: {exc}'
    return out


def declared_net_names(score: dict) -> dict:
    """{field path: [net names]} for every score field that names a REAL net.

    Registered explicitly rather than discovered by walking the payload,
    because the payload also carries fields whose whole PURPOSE is to name
    things that do not exist -- `length.groups[].missing_nets` and
    `net_widths.patterns_matching_no_routed_net` are findings about the spec,
    and auditing them would report every correct run as inconsistent. A new
    net-name field must be added here; that is the intended cost.
    """
    c = score.get('components') or {}
    unrouted = c.get('unrouted') or {}
    broken = c.get('broken') or {}
    widths = c.get('net_widths') or {}
    length = c.get('length') or {}
    fields = {
        'connectivity_nets': list(score.get('connectivity_nets') or ()),
        'components.unrouted.nets': list(unrouted.get('nets') or ()),
        'components.unrouted.open': list(unrouted.get('open') or ()),
        'components.unrouted.placement_blocked':
            list((unrouted.get('placement_blocked') or {}).keys()),
        'components.broken.poured_nets': list(broken.get('poured_nets') or ()),
        'components.broken.nets': list((broken.get('nets') or {}).keys()),
        'components.net_widths.nets': list((widths.get('nets') or {}).keys()),
        'components.length.groups[].worst':
            [g['worst'] for g in (length.get('groups') or {}).values()
             if isinstance(g, dict) and g.get('worst')],
    }
    return {k: v for k, v in fields.items() if v}


def audit_net_names(board: str, score: dict) -> dict:
    """EVERY net name this score publishes must exist on the board it graded.

    The defect this exists to catch cannot be caught any other way: `poured_nets`
    was published double-escaped (`/GPIO10\\\\OE3#` where the board and every
    sibling field said `/GPIO10\\OE3#`) and nothing noticed for the life of the
    field, because a downstream tool CANNOT TELL A MANGLED NAME FROM A NET THAT
    DOES NOT EXIST -- both simply match nothing. Measured cost of the one that
    shipped: a consumer built `--ignore-nets` from the mangled list, 10 of 61
    names silently matched nothing, and the resulting render reported hpwl
    +45.6% and crossings +129.5% on a board that had not changed.

    So this is a self-check on the INSTRUMENT, not a grade of the board. It
    deliberately does NOT touch `blocking` or the exit code: every mismatch it
    can find is a bug in this script, and moving the scalar would rewrite the
    meaning of every ledger row already recorded to chase one.
    """
    try:
        # Self-sufficient import: main() bootstraps sys.path for the #522
        # layout, but this function is also called IN-PROCESS (the worklist
        # tests import it directly), where that bootstrap never ran -- the
        # audit then reported ran:False on a perfectly parseable board.
        for _d in _TOOL_DIRS:
            _p = os.path.join(krt_dir(), _d) if _d else krt_dir()
            if os.path.isdir(_p) and _p not in sys.path:
                sys.path.insert(0, _p)
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(board)
    except Exception as exc:                                    # noqa: BLE001
        return {'ran': False,
                'reason': f'board unparseable here: {type(exc).__name__}: {exc}'}
    known = {n.name for n in (pcb.nets or {}).values() if n.name}
    fields = declared_net_names(score)
    if not known:
        # No net table = nothing to check against. Saying so beats a confident
        # "0 unknown", which is the same vacuity trap `ran: false` exists for
        # everywhere else in this file.
        return {'ran': False, 'reason': 'board declares no net table',
                'names_published': sum(len(v) for v in fields.values())}
    unknown = {}
    checked = 0
    for field, names in fields.items():
        for name in names:
            checked += 1
            if name not in known:
                unknown.setdefault(field, []).append(name)
    return {'ran': True, 'checked': checked, 'board_nets': len(known),
            'fields_checked': sorted(fields),
            'unknown': {k: sorted(v) for k, v in unknown.items()},
            'unknown_count': sum(len(v) for v in unknown.values())}


def quality(board: str) -> dict:
    """Tie-breakers, compared ONLY once blocking == 0. Never a blocker itself:
    a board is not worse for having more copper if the alternative is a
    disconnected net."""
    try:
        from kicad_parser import parse_kicad_pcb
        import math
        pcb = parse_kicad_pcb(board)
        # ROUTED copper only (#908). A footprint's own drawn copper -- a SOT89
        # tab, a PCB antenna -- parses as `graphic=True` Segments, and it is
        # identical in every candidate placement of the same board: counting
        # it adds a constant to `copper_mm` and, worse, makes a copper-FREE
        # board look like it has a quality key that can rank. A board carrying
        # a meander antenna can reach dozens of such segments with not one
        # routed track on it.
        segs = [s for s in pcb.segments if not getattr(s, 'graphic', False)]
        mm = sum(math.dist((s.start_x, s.start_y), (s.end_x, s.end_y))
                 for s in segs)
        return {'vias': len(pcb.vias), 'copper_mm': round(mm, 2),
                'segments': len(segs)}
    except Exception as e:
        return {'error': str(e)}


def score_placement(root: str, board: str, tmp: str, intent: str = '',
                    parent: dict = None) -> dict:
    """Placement quality for a copper-free lap (#894). REPORT-ONLY.

    NOT in `parts`, so not in `blocking`, not in `blocking_by`, not in
    `ungraded`, not in `unknown`, and it cannot move the exit code. Three
    reasons, and the first is the one to read: its terms are millimetres and
    counts in four different currencies, and `blocking` is a sum. Adding
    metres of pair length to a violation count produces a number with no unit.
    The second is that `parts` is AST-scraped by
    `tests/test_904_lens_components_cover_blocking.py`, which would then
    demand a verifier lens for a component no lens grades. The third is that
    keeping it out leaves every existing ledger's `blocking` untouched.

    Run as a SUBPROCESS, like every other component here (see this module's
    header): `placement_score` builds a quench state that PRINTS to stdout,
    and this script's stdout carries `SCORE_JSON=` which every consumer parses
    whole. The tool takes `--json`, so nothing of its own reaches this stream.
    """
    out = os.path.join(tmp, 'placement.json')
    args = [board, '--json', out]
    if intent:
        args += ['--intent', intent]
    rc, text = run_tool(root, 'placement_score.py', *args)
    if rc != 0 or not os.path.exists(out):
        return skipped(f'placement_score rc {rc}: {text.strip()[-200:]}')
    try:
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    except Exception as exc:                                 # noqa: BLE001
        return skipped(f'placement_score json unreadable: {exc}')
    if parent is not None:
        try:
            sys.path.insert(0, os.path.join(root, 'py_placer'))
            import placement_score as _ps
            verdict, detail = _ps.compare_terms(
                (parent.get('placement') or {}).get('terms'), doc['terms'])
            doc['vs_parent'] = {
                'verdict': verdict, 'terms': detail,
                'board_sha': parent.get('board_sha'),
                'label': parent.get('label')}
        except Exception as exc:                             # noqa: BLE001
            doc['vs_parent'] = {'error': f'{type(exc).__name__}: {exc}'}
    return doc


def build_parser():
    p = argparse.ArgumentParser(
        description='One authoritative (blocking, quality) score for a board',
        epilog='Exit 0 = blocking is 0; 4 = graded with blockers; '
               '3 = board state; 2 = bad arguments; 1 = crash.')
    p.add_argument('board', help='the .kicad_pcb to score')
    p.add_argument('--intent', help='floorplan intent JSON (check_floorplan '
                                    '--intent). Omitted = floorplan ungraded')
    p.add_argument('--clearance', type=float,
                   help='grade DRC at this clearance. OMIT IT unless you know '
                        'better than the board: check_drc then reads the '
                        'sibling .kicad_pro, which is the floor the board was '
                        'actually routed to')
    g = p.add_argument_group(
        'spec size floors',
        "check_drc defaults these to the FAB minimum for the layer count. Pass "
        "the board's own spec whenever it is TIGHTER than the fab -- that gap is "
        "how 141 spec-violating vias once graded clean.")
    g.add_argument('--min-track-width', type=float, metavar='MM')
    g.add_argument('--min-via-diameter', type=float, metavar='MM')
    g.add_argument('--min-via-drill', type=float, metavar='MM')
    g.add_argument('--size-margin', type=float, metavar='MM',
                   help='absolute tolerance on the size checks (default: exact floor)')
    p.add_argument('--impedance-nets', nargs='+', metavar='GLOB',
                   help='route.py --nets glob syntax, SPACE separated; commas '
                        'inside a token are split too (a comma-joined list used '
                        'to become one impossible glob that silently matched '
                        'nothing). Enables the impedance component')
    p.add_argument('--net-min-widths', metavar='JSON',
                   help='JSON FILE of {"<net glob>": <min mm>} -- per-net width '
                        'requirements. `undersized` only sees BOARD-WIDE floors, '
                        'so a spec demanding a particular net be wider (a 0.8mm '
                        'USB pair, 0.4mm rails) is invisible to it. First '
                        'matching glob wins')
    p.add_argument('--length-groups', metavar='JSON',
                   help='{"group": {"nets": [...], "tolerance_mm": 0.1, '
                        '"mode": "pin_pair"}} -- enables the length component')
    p.add_argument('--placement-terms', action='store_true',
                   help='grade the PLACEMENT terms (#894): worst diff-pair '
                        'span, crossed pin orders, cluster distance, plane-cut '
                        'proxy, pad-area balance. REPORT-ONLY -- never in '
                        '`blocking`, never in the exit code. On a copper-free '
                        'board `quality` is (0, 0.0, 0) for every placement, '
                        'so this is the only thing that can rank two of them. '
                        'Opt-in because it builds a quench state')
    p.add_argument('--parent-score', metavar='PATH',
                   help="the parent lap's board_score JSON. Each placement "
                        "term is then reported raw AND as a delta against it. "
                        "Ignored without --placement-terms")
    p.add_argument('--json', metavar='PATH', help='write the full score here')
    p.add_argument('--label', default='', help='free text carried into the JSON '
                                               '(the ledger uses it for the lever)')
    p.add_argument('--quiet', '-q', action='store_true',
                   help='print only SCORE_JSON= and the one-line summary')
    return p


def main():
    args = build_parser().parse_args()
    if not os.path.isfile(args.board):
        print(f"board not found: {args.board}", file=sys.stderr)
        return 3
    root = krt_dir()
    # In-process imports (kicad_parser, net_queries) come from the engine dir;
    # #522 + the placement split spread them over py_router/ and py_tools/, so
    # add every layout dir. The flat entry keeps an older clone working.
    for _d in _TOOL_DIRS:
        _p = os.path.join(root, _d) if _d else root
        if os.path.isdir(_p) and _p not in sys.path:
            sys.path.insert(0, _p)
    sizes = {'--min-track-width': args.min_track_width,
             '--min-via-diameter': args.min_via_diameter,
             '--min-via-drill': args.min_via_drill,
             '--size-margin': args.size_margin}
    # A real temp dir, NOT a dotfile beside the board: these are intermediate
    # JSONs nobody reads twice, and scoring a board must leave nothing behind in
    # the user's project. `--json` is the copy you keep.
    with tempfile.TemporaryDirectory(prefix='board_score_') as tmp:
        conn = score_connectivity(root, args.board)
        drc, undersized, rule_pairs = score_drc(root, args.board, args.clearance, sizes)
        floorplan = score_floorplan(root, args.board, args.intent, tmp)
        assembly = score_assembly(root, args.board, args.intent, tmp,
                                  args.clearance)
        _imp_nets = ([g for tok in args.impedance_nets for g in tok.split(',') if g]
                     if args.impedance_nets else args.impedance_nets)
        imped = score_impedance(root, args.board, _imp_nets, tmp)
        length = score_length(args.board, args.length_groups)
        net_widths = score_net_widths(args.board, args.net_min_widths)
        placement = None
        if args.placement_terms:
            _parent = None
            if args.parent_score:
                try:
                    with open(args.parent_score, encoding='utf-8') as _f:
                        _parent = json.load(_f)
                except Exception as _exc:                    # noqa: BLE001
                    print(f'--parent-score unreadable, so no delta is '
                          f'reported: {_exc}', file=sys.stderr)
            placement = score_placement(root, args.board, tmp, args.intent,
                                        _parent)

    # Both connectivity components carry their work list, not just their count --
    # see score_connectivity. `unrouted` needs names; `broken` needs names, piece
    # counts and the stranded pads, or 9.1a's lever 2 has nothing to act on.
    # ...and `unrouted` carries its SHAPE. `count` is deliberately untouched, so
    # `blocking` below is unchanged by construction.
    shape = unrouted_shape(args.board, conn.get('unrouted_net_names', []))
    parts = {'unrouted': {'ran': conn['ran'], 'count': conn.get('unrouted'),
                          'nets': conn.get('unrouted_net_names', []),
                          **shape},
             'broken': {'ran': conn['ran'], 'count': conn.get('broken'),
                        'poured_nets': conn.get('poured_nets', []),
                        'poured_nets_meaning': POURED_NETS_MEANING,
                        'nets': conn.get('broken_detail', {})},
             'drc': drc, 'undersized': undersized, 'floorplan': floorplan,
             'assembly': assembly,
             'impedance': imped, 'length': length, 'net_widths': net_widths}

    # Track-rule-governed pairs are ADVISORY: their gate is the repo's own
    # registered-floor checker (check_dru), not this scalar. They live beside
    # `parts`, never in it -- the blocking sum below iterates parts, and 610
    # floor-governed pairs must not drown ~17 physical defects (run 6).
    advisory = {'drc_rule_pairs': rule_pairs}

    # A component that was ASKED for and could not run leaves blocking unknown.
    # Reporting 0 there would let the loop stop on a board nothing graded.
    counts = [v.get('count') for v in parts.values()]
    unknown = [k for k, v in parts.items()
               if v.get('count') is None and v.get('ran') is not False]
    blocking = None if unknown else sum(c for c in counts if c)

    # board_sha binds this payload to the exact file it graded (run-3 B4:
    # three ledger entries shipped embedding a PRIOR board's quality because
    # nothing tied a score to its board). Same sha256-of-bytes as
    # board_store.put, so converge record can compare them.
    import hashlib
    _h = hashlib.sha256()
    with open(args.board, 'rb') as _f:
        for chunk in iter(lambda: _f.read(1 << 20), b''):
            _h.update(chunk)

    score = {'schema': 1, 'kind': 'board-score', 'board': os.path.abspath(args.board),
             'board_sha': _h.hexdigest(),
             'label': args.label, 'blocking': blocking,
             'blocking_by': {k: v.get('count') for k, v in parts.items()},
             'advisory': {k: v.get('count') for k, v in advisory.items()},
             'ungraded': sorted(k for k, v in parts.items() if v.get('ran') is False),
             'unknown': sorted(unknown), 'quality': quality(args.board),
             # BESIDE `quality`, never inside `parts` -- see score_placement.
             # Absent entirely without the flag, so a payload that carries the
             # key is one that asked for it.
             **({'placement': placement} if placement is not None else {}),
             'components': {**parts, **advisory},
             'floors': _floors(args.board, sizes),
             'connectivity_nets': conn.get('nets', [])}
    # Self-check the payload against the board it just graded (see
    # audit_net_names). Computed AFTER `score` is assembled so it audits what is
    # actually published, not what this function believes it published.
    score['net_name_audit'] = audit_net_names(args.board, score)

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(score, f, indent=2, sort_keys=True)

    print('SCORE_JSON=' + json.dumps(score, sort_keys=True, separators=(',', ':')))
    bits = ' '.join(f'{k}={v}' for k, v in score['blocking_by'].items()
                    if v is not None)
    q = score['quality']
    print(f"BLOCKING={blocking}  ({bits})  "
          f"vias={q.get('vias')} copper_mm={q.get('copper_mm')}")
    _adv_bits = ' '.join(f'{k}={v}' for k, v in score['advisory'].items() if v)
    if _adv_bits:
        print(f"ADVISORY (floor-governed, not blocking): {_adv_bits}")
    # PLACEMENT terms, and the note when they are missing on a board where
    # nothing else can rank a lap.
    if placement is not None:
        _pt = placement.get('terms') or {}
        _bits = ' '.join(
            f"{k}={_pt[k]['value']}" for k in (placement.get('term_order') or [])
            if _pt.get(k, {}).get('value') is not None)
        _ungraded = [k for k in (placement.get('term_order') or [])
                     if _pt.get(k, {}).get('ran') is False]
        # A plain variable, not a multi-line expression inside the f-string:
        # that spelling is PEP 701 and a SyntaxError before Python 3.12, and
        # README.md says 3.9+. It would have broken the whole scorer at import
        # on a supported interpreter.
        _summary = _bits or 'nothing measured'
        print(f"PLACEMENT (report-only, not blocking): {_summary}")
        if _ungraded:
            print(f"  placement terms UNGRADED (not scored, not passed): "
                  f"{', '.join(_ungraded)}")
        _vs = placement.get('vs_parent') or {}
        if _vs.get('terms'):
            sys.path.insert(0, os.path.join(root, 'py_placer'))
            try:
                import placement_score as _ps
                print(f"  vs parent: {_vs.get('verdict')} -- "
                      f"{_ps.format_delta(_vs['terms'])}")
            except Exception:                                # noqa: BLE001
                pass
    elif (score['quality'] or {}).get('segments') == 0:
        print("DEGENERATE QUALITY KEY: this board carries 0 copper segments, "
              "so `quality` is (0, 0.0, 0) on EVERY such board and cannot "
              "rank two placements. Re-score with --placement-terms to get a "
              "key the placement half can compare.")
    # WHICH LEVER. `unrouted` looks the same whether the router had a path and
    # missed it (parameter-shaped) or the net has fewer than 2 pads ON the
    # board (placement-shaped, and no router setting will ever fix it). The
    # router's own filter already knows; nothing surfaced it, so run 10 spent a
    # routing pass and a plane-repair pass finding out.
    _pb = (score['components']['unrouted'].get('placement_blocked') or {})
    if _pb:
        print(f"PLACEMENT-BLOCKED: {len(_pb)} of "
              f"{score['blocking_by']['unrouted']} unrouted net(s) have <2 "
              f"ON-BOARD pads and CANNOT be routed at this placement -- "
              f"bring these parts back on the board first: "
              f"{' '.join(score['components']['unrouted']['placement_blocked_refs'])}")
    # WHICH FLOOR. A board that declares no net class and no board constraint
    # was graded entirely against each checker's own fallback, and every
    # `floors` value is None -- indistinguishable in a table from a board whose
    # rules simply were not read. Say it once, here, rather than leaving the
    # reader to notice a column of nulls (run-12 Tier 1.3; corpus boards that
    # ship no .kicad_pro are common). Report-only: `blocking` and the exit code
    # are untouched.
    if score['floors'].get('declares_no_floor'):
        print("NO DECLARED FLOOR: this board carries no net class and no board "
              "constraint (no sibling .kicad_pro, no (net_class) block), so "
              "every component above was graded at its checker's FALLBACK, not "
              "at this board's own rules. Compare it only against boards graded "
              "the same way, and pass --clearance <the routed value> when the "
              "chain records one.")
    # THE INSTRUMENT CHECKING ITSELF. Loud on stdout AND stderr: a name this
    # score publishes that the board does not have is unusable downstream and
    # fails SILENTLY there (it matches nothing, exactly like a net that is
    # genuinely absent). Report-only by construction -- see audit_net_names.
    _audit = score['net_name_audit']
    if _audit.get('unknown_count'):
        _lines = '; '.join(
            f"{f}: {', '.join(repr(n) for n in ns[:5])}"
            f"{' ...' if len(ns) > 5 else ''}"
            for f, ns in sorted(_audit['unknown'].items()))
        _msg = (f"NET-NAME MISMATCH: {_audit['unknown_count']} of "
                f"{_audit['checked']} net name(s) this score publishes do NOT "
                f"exist on the board it graded -- {_lines}. These will match "
                f"NOTHING downstream and will not announce it. This is a bug in "
                f"board_score, not a finding about the board.")
        print(_msg)
        print(_msg, file=sys.stderr)
    if score['ungraded']:
        # Loud, because this is the difference between "clean" and "unexamined".
        print(f"UNGRADED (not scored, not passed): {', '.join(score['ungraded'])}")
    if unknown:
        print(f"UNKNOWN (asked for, could not run): {', '.join(unknown)}")
        return 4
    return 0 if blocking == 0 else 4


if __name__ == '__main__':
    # CMD/EXIT self-echo (run-5 c1). Guarded: this script lives four levels
    # under the repo root, and the banner must never be the reason a grade
    # fails. Children already run with KRT_NO_BANNER (cc42e33), so a
    # composed run prints exactly ONE banner pair -- this one.
    try:
        import os as _os
        import sys as _sys
        _root = _os.path.dirname(_os.path.dirname(_os.path.dirname(
            _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))))
        # cli_banner moved into py_router/ with the placement split; keep the
        # flat root too so an older clone still banners.
        for _d in ('py_router', ''):
            _p = _os.path.join(_root, _d) if _d else _root
            if _os.path.isdir(_p) and _p not in _sys.path:
                _sys.path.insert(0, _p)
        import cli_banner
        cli_banner.install()
    except Exception:
        pass
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception as exc:                                   # noqa: BLE001
        print(f"board_score crashed: {exc}", file=sys.stderr)
        sys.exit(1)
