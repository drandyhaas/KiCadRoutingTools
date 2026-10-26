#!/usr/bin/env python3
"""One authoritative score for a board, so a loop can tell better from worse.

Why this exists
---------------
A router's own ``JSON_SUMMARY`` is not a verdict. CLAUDE.md: *"Routers can report
false success. A router's own 'routed' tally may come from a local/heuristic
proxy while pads stay disconnected"* -- so a board can read as routed while pads
sit disconnected and DRC is dirty.

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
               + assembly + impedance + length + net_widths + diff_pairs)
    quality  = (vias, copper_mm, segments)      # only compared once blocking == 0

`blocking` must reach 0 before a board is deliverable. `quality` orders the
boards that already got there. Below that, two boards of equal `blocking`
are ordered by `joins_needed` -- the joins their open nets still need (an
unrouted net of N pads needs N - 1, a broken one components - 1) -- never by
`quality`: a board missing a 17-pad rail has fewer vias BECAUSE the rail is
missing, and one net counts 1 in `blocking` whatever its size (#1340).

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

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'routing', 'combined'], 'kind': 'instrument'}

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
SIZE_TYPES = frozenset({'track-width', 'via-size', 'via-drill-size',
                        'via-annular'})

# Seg-seg pairs whose violation exists ONLY because a .kicad_dru track
# rule raised the pair clearance (check_drc tags them with a distinct type and
# the binding rule's name). They are the structural, registered-floor-governed
# population -- a repo's check_dru gate is their arbiter -- so board_score
# reports them as ADVISORY, outside `blocking`. Run 6 measured why: 610 such
# pairs drowned a blocking of ~17 physical defects into an unusable 627.
RULE_PAIR_TYPES = frozenset({'segment-segment-track-rule'})

# #962: a via in a solder-paste opening that is not filled+capped. ADVISORY,
# beside `drc_rule_pairs`, never in `blocking`. On a KiCad 10-format board
# the tool stamps IPC-4761 Type VII onto every via it puts under solder at ship
# time, so what fires on a board this chain routed is almost always a via the
# input already had -- 136 on one corpus board alone (a pre-KiCad-10 file
# cannot carry the tokens; check_drc accepts those as undeclarable). No
# placement or routing lap can change a fab spec it did not write, so
# counting them in `blocking` would make 0 unreachable there.
# Pass --baseline <input board> and check_drc accepts those as
# `inherited-via-in-paste`; what is left is then a site the run created, and
# counts in `blocking` (#1171).
VIA_PASTE_TYPES = frozenset({'via-in-paste'})
# check_drc's `keepout` (a track or via in a keep-out rule area that forbids
# it) is deliberately in none of these sets: it is routed copper where the
# board says none may go, so it stays in `drc` and counts in `blocking`
# (copper the --baseline input already had there is accepted, not listed).

# #1303: a .kicad_dru skew or diff_pair_uncoupled rule a pair's copper breaks
# (check_drc's pair-rule pass, KiCad's skew_out_of_range and
# diff_pair_uncoupled_length_too_long). Split out of `drc` into the
# `diff_pairs` component, which carries each pair's measurement beside them.
PAIR_RULE_TYPES = frozenset({'skew-out-of-range', 'diff-pair-uncoupled'})

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
    # <krt>/py_tools/board_score.py -> one up
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
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


#: Wall seconds per sub-checker this run, published as SCORE_JSON
#: `tool_seconds` (#1202 comment: a 61-minute score could not be attributed
#: after the fact, because nothing recorded which child took the time).
TOOL_SECONDS: dict = {}


def run_tool(root: str, tool: str, *args) -> tuple:
    """(returncode, combined output). -X utf8 for the Ω/µ the tools print.

    KRT_NO_BANNER: the child instruments self-echo CMD/EXIT (run-4 B1) --
    inside a COMPOSED run those lines would land mid-document and read as
    the outer gate's exit (measured: a blocking-153 score log carrying an
    inner checker's EXIT=0). The outer invocation is the evidence unit."""
    cmd = [sys.executable, '-X', 'utf8', _tool_path(root, tool)] + [str(a) for a in args]
    env = dict(os.environ, KRT_NO_BANNER='1')
    import time as _time
    t0 = _time.monotonic()
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       encoding='utf-8', errors='replace', env=env)
    TOOL_SECONDS[tool] = TOOL_SECONDS.get(tool, 0.0) + (_time.monotonic() - t0)
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
# It is "this net has at least one zone on the board", which decides HOW a
# broken net gets fixed (route.py's in-run plane finalize, which only runs on
# nets inside that step's --nets) -- and it is NOT a list of plane nets.
# Measured on neo6502: the 61 zone-backed nets covered 332 of 545 pads (72%),
# including all of /A0../A15, because that board pours many signal nets; a
# consumer that read it as "the planes, safe to ignore" removed most of the
# board from its own analysis. Emitting the sentence next to the list is the
# point -- a comment in this file is not visible to the consumer reading the
# JSON, and that is who got this wrong.
POURED_NETS_MEANING = (
    'nets with at least one zone on the board. A broken one of these is fixed '
    "by route.py with the net inside its --nets: the step's in-run plane "
    "finalize (#562) taps and joins it at that step's own track and via "
    "sizes, checked against KiCad's exact fill, and runs even when the step "
    'has nothing else to route (#1112). This is NOT a list of plane/power '
    'nets and is NOT a safe --ignore-nets population: a board that pours '
    'signal nets puts them in here too (measured: 332 of 545 pads).')


def poured_net_names(board: str) -> set:
    """Nets with at least one zone on the board, read off the file text
    (POURED_NETS_MEANING). Its own function so a fully connected board
    reports them too (#1202: `poured_nets` read [] next to its own
    meaning on a board carrying a GND zone)."""
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
    return poured


def score_connectivity(root: str, board: str) -> dict:
    """Unrouted and broken nets, from check_connected.py.

    This is the authoritative, zone/fill-aware answer -- it reconciles
    pour-backed nets against KiCad's exact fill in both directions, which the
    router's own tally does not do.
    """
    rc, out = run_tool(root, 'check_connected.py', board)
    if 'ALL NETS FULLY CONNECTED' in out:
        return {'ran': True, 'count': 0, 'unrouted': 0, 'broken': 0, 'nets': [],
                'unrouted_pads': {}, 'joins_needed': 0,
                'poured_nets': sorted(poured_net_names(board)),
                'poured_nets_meaning': POURED_NETS_MEANING}
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
    unrouted_pads = {n: int(c) for n, c in
                     re.findall(r'^\s{4}(\S+) \((\d+) pads\)', out, re.M)}
    unrouted_names = list(unrouted_pads)
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
    # a do-not-fit part is not a functional defect and must not be chased forever,
    # and the ref is what lets the caller tell it apart.
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

    # NAME THE TOOL, not just the defect. Every break is route.py's (#1112). A
    # stranded pad on a POURED net is reached by route.py's in-run plane finalize
    # (#562: the repair engine's taps and joins plus the KiCad exact-fill
    # reconnect, at the route step's own track/via sizes), provided the net is
    # inside that step's --nets. This used to say `repair_planes`, on a
    # measurement taken three days BEFORE the finalize landed (`broken` held at
    # 14 under route.py, fell to 11 in one repair call), and a run that obeyed it
    # ended its chain on a standalone repair at the board's 0.5 mm class via
    # around copper routed with 0.3 mm vias: the repair cannot know the chain's
    # sizes, because the .kicad_pro writeback never lowers the class's via/track
    # draw defaults (#842). The finalize also runs when route.py finds nothing
    # else to route (#1112), so a break only KiCad's exact fill sees is reached
    # too. The DNF case stays a human call, which is what the stranded pad's
    # `ref` is in the list for.
    poured = poured_net_names(board)
    for name, v in detail.items():
        v['handler'] = 'route'

    # THE JOINS THE BOARD STILL NEEDS (#1340). `unrouted` counts NETS, so a
    # 17-pad rail with no copper weighs 1 -- the same as one stranded pad --
    # and a tie on `blocking` fell to vias and copper, where the missing rail
    # reads as quality: StickHub's rail-less board (117 vias) outranked the
    # one with a single open pad (132). An unrouted net of N pads needs N - 1
    # joins, a broken one (components - 1), the unit `broken` already counts;
    # their sum breaks a `blocking` tie before vias or copper do. check_
    # connected prints each unrouted net's pad count, which was parsed and
    # dropped.
    joins = sum(max(0, n - 1) for n in unrouted_pads.values()) + broken
    return {'ran': True, 'count': int(m.group(1)), 'unrouted': unrouted,
            'broken': broken, 'broken_nets': broken_nets,
            'components_per_broken_net': comps, 'nets': sorted(set(nets)),
            'unrouted_net_names': sorted(set(unrouted_names)),
            'unrouted_pads': dict(sorted(unrouted_pads.items())),
            'joins_needed': joins,
            'poured_nets': sorted(poured),
            'poured_nets_meaning': POURED_NETS_MEANING,
            'broken_detail': detail}


def unrouted_shape(board: str, unrouted_names, pcb=None) -> dict:
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
        if pcb is None:
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


#: check_assembly's twelve `not_buildable` conjuncts, by the JSON key each one
#: publishes (check_assembly's `not_buildable = ...` line). `blocking` -- pad INTERSECTIONS -- is
#: the first of them and is the only one this component used to read (#918).
#:
#: THEY ARE NOT DISJOINT AND THEY ARE NOT ONE CURRENCY, which is why `count`
#: below does not add them up:
#:
#:   * `locked_contacts` is a strict SUBSET of `blocking`. `locked_ref` is set
#:     at exactly one site, inside the pad-intersection channel, so every
#:     locked-contact pair is already a blocking pair --
#:     `tests/test_run8_locked_contact.py` (since retired) asserted in so many
#:     words that it is "a second channel, not a re-count".
#:   * `coincident_origins` counts GROUPS, not pairs: an N-part stack is one
#:     group and N(N-1)/2 potential pairs.
#:   * `containment_blocking` counts `fab`-kind pairs, a different geometry
#:     channel from `blocking`'s pad intersections -- but routinely the SAME
#:     ref pair. Measured on a perturbed corpus board, 3 of 11 containments
#:     named a pair already in `blocking`; on the fixture
#:     `tests/test_918_assembly_verdict.py` builds, one stacked-capacitor
#:     defect appears as a group AND as a containment.
#:
#: `courtyard_blocking_gating` is the moved-vs-baseline subset of the
#: courtyard census: armed only when board_score is handed `--baseline`, which
#: it forwards to check_assembly (#1183). Unarmed, check_assembly publishes
#: null rather than 0, and the scorer reports it unmeasured -- in
#: `conjuncts_unmeasured` and in the top-level `ungraded` as
#: 'assembly.courtyard_gating' -- never counted as clean.
#:
#: `testpoint_covered_gating` (#1250) is armed by the same `--baseline` on
#: the same currency (a test point a moved member left under another part's
#: drawn body), and is null when unarmed for the same reason.
#:
#: `body_edge_band_gating` (#1255) is armed by a DECLARATION instead -- the
#: intent's `assembly.body_edge_mm` -- and is 0, not null, when nothing is
#: declared: the census still ran, and an undeclared band gates nothing by
#: policy, which is a measured answer rather than a missing one.
#:
#: `graphic_pad_gating` (#1266) is the courtyard currency again -- a pad a
#: moved member put on another part's footprint graphic copper -- and is
#: null when unarmed.
#:
#: `own_pad_shorts` (#1306) is two of ONE part's pads touching on two real
#: nets. Always measured; `--baseline` only exempts the pairs its footprint
#: already had.
ASSEMBLY_CONJUNCTS = ('blocking', 'locked_contacts', 'coincident_origins',
                      'containment_blocking', 'courtyard_blocking_gating',
                      'oob_pad_copper_gating_count', 'mating_keepout_count',
                      'pin_in_courtyard', 'testpoint_covered_gating',
                      'body_edge_band_gating', 'graphic_pad_gating',
                      'own_pad_shorts')

#: The conjuncts that can ACTUALLY flip the verdict while `blocking` is 0, in
#: this scorer's invocation. Not every conjunct can:
#:   * `locked_contacts` cannot -- it is a subset of `blocking` (above), so a
#:     locked contact implies `blocking >= 1` and the board never had 0;
#:   * `courtyard_blocking_gating` can since #1183, when `--baseline` arms it:
#:     a part moved onto another's courtyard gates at `blocking` 0.
#: Written down because the issue, and this file's first draft, claimed all
#: four -- and a motivating case that cannot occur is not a motivating case.
#: #1096 added `oob_pad_copper_gating_count`: pad copper past the outline
#: (true pad outlines, castellated pads left out) gates the
#: verdict, and it does not need `blocking` to fire. It overlaps `unrouted` /
#: `broken` (such a part's nets cannot be routed) and adds at most 1 through
#: the NOT-BUILDABLE floor, deliberately: CLAUDE.md ranks it first.
#: fa10 P1 (#1212) added `pin_in_courtyard`: a pin frame's drilled pin inside
#: another part's courtyard, absolute, so it too fires at `blocking` 0.
#: #1250 added `testpoint_covered_gating`: a test point a moved member left
#: under another part's drawn body, armed by `--baseline` like the courtyard
#: gate -- its pads are not another part's pads, so `blocking` stays 0.
#: #1255 added `body_edge_band_gating`: a body inside the body-to-edge band
#: the intent declares, a fact about one part, so it too fires at
#: `blocking` 0.
#: #1266 added `graphic_pad_gating`: a pad on another part's footprint
#: GRAPHIC copper is no pad intersection, so it fires at `blocking` 0.
#: #1306 added `own_pad_shorts`: two pads of ONE part are no pair of parts,
#: so it fires at `blocking` 0 too.
ASSEMBLY_LIVE_CONJUNCTS = ('coincident_origins', 'containment_blocking',
                           'oob_pad_copper_gating_count', 'mating_keepout_count',
                           'pin_in_courtyard', 'courtyard_blocking_gating',
                           'testpoint_covered_gating', 'body_edge_band_gating',
                           'graphic_pad_gating', 'own_pad_shorts')


def assembly_component(doc: dict, rc: int) -> dict:
    """check_assembly's VERDICT, not one of its ten conjuncts (#918).

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
            f"check_assembly published no `buildable` key: `blocking` alone is "
            f"1 of its {len(ASSEMBLY_CONJUNCTS)} not_buildable conjuncts "
            f"(check_assembly's `not_buildable` line), and this component "
            f"will not re-derive the other {len(ASSEMBLY_CONJUNCTS) - 1}")
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
    # The per-pair lists, each with its full COUNT. `--json` carries every
    # entry (`<list>_truncated` 0); the stdout SCORE_JSON caps them
    # (`stdout_payload`, #1194), so the count is what a stdout reader keeps.
    lists = {'pairs': doc.get('blocking_pairs') or [],
             'locked_contact_pairs': doc.get('locked_contact_pairs') or [],
             'coincident_origin_groups':
                 doc.get('coincident_origin_groups') or [],
             'containments': doc.get('containments') or []}
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
                     'is unarmed and publishes null, and so is its ninth '
                     '(a test point a move left under a body, #1250). Pass '
                     '--baseline <the board this one was derived from> to arm '
                     'it (#1183)'),
            'advisory_pairs': int(doc.get('advisory') or 0),
            'waived_pairs': int(doc.get('waived') or 0),
            **lists,
            **{k + '_count': len(v) for k, v in lists.items()},
            **{k + '_truncated': 0 for k in lists}}


#: How many entries of each assembly per-pair list the stdout SCORE_JSON
#: carries (#1194). A pile's `pairs` alone made that one line 590 KB (2302
#: pad-intersection pairs on One-Air-Max, each with its overlap rect and
#: shorts), and it landed in every log and subprocess capture reading stdout.
#: Every in-repo consumer of the full lists reads `--json`.
STDOUT_LIST_CAP = 20
ASSEMBLY_LISTS = ('pairs', 'locked_contact_pairs', 'coincident_origin_groups',
                  'containments')


def stdout_payload(score: dict, cap: int = STDOUT_LIST_CAP) -> dict:
    """The SCORE_JSON copy of `score`: each assembly list capped at `cap`
    entries, `<list>_count` the full length and `<list>_truncated` how many
    were left out (#1194). `score` itself -- what `--json` writes -- is not
    touched. Pure."""
    asm = (score.get('components') or {}).get('assembly')
    if not isinstance(asm, dict):
        return score
    long_ = [k for k in ASSEMBLY_LISTS
             if isinstance(asm.get(k), list) and len(asm[k]) > cap]
    if not long_:
        return score
    asm = dict(asm)
    for k in long_:
        full = asm[k]
        asm[k] = full[:cap]
        asm[k + '_count'] = len(full)
        asm[k + '_truncated'] = len(full) - cap
    return {**score, 'components': {**score['components'], 'assembly': asm}}


def ungraded_entries(parts: dict) -> list:
    """The top-level `ungraded` list: every component that did not run,
    and 'assembly.courtyard_gating' whenever check_assembly's courtyard gate
    was not armed (#964 item 2 / #1183) -- an UNARMED conjunct is
    unexamined, and only a top-level entry is something a loop compares.
    Listed whether or not assembly itself ran: a lap whose assembly FAILED
    must still read as grading fewer things than an unarmed one, or
    converge takes its lower `blocking` for a gain (phase-6 verifier: 10 ->
    9 accepted, because the dotted entry vanished with the component)."""
    return sorted(
        [k for k, v in parts.items() if v.get('ran') is False]
        + (['assembly.courtyard_gating']
           if not (parts.get('assembly') or {}).get('courtyard_gating_armed')
           else [])
        # #1390: a .kicad_dru size rule check_drc could not model is a
        # declaration nothing grades.
        + (['undersized.dru_rules']
           if (parts.get('undersized') or {}).get('ungraded_rules') else []))


def baseline_problem(path: str):
    """Why `path` cannot be a --baseline, or None: it must be a KiCad board
    with at least one footprint (a cheap text test; check_assembly parses it
    and refuses the same way)."""
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            text = fh.read()
    except OSError as exc:
        return f'unreadable ({exc})'
    if '(kicad_pcb' not in text[:4096]:
        return 'not a KiCad board'
    if '(footprint ' not in text:
        return 'a board with no footprints'
    return None


def score_assembly(root: str, board: str, intent: str, tmp: str,
                   clearance=None, baseline=None) -> dict:
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
    # #1183: the board this one was derived from ARMS check_assembly's fifth
    # conjunct -- a courtyard pair a MOVED part entered gates. board_score had
    # the flag since #962 and handed it to check_drc only, so One-Air-Max's
    # seed graded `buildable` here and NOT BUILDABLE (22 of 22 pairs gating)
    # under check_assembly --baseline.
    if baseline:
        args += ['--baseline', baseline]
    rc, text = run_tool(root, 'check_assembly.py', *args)
    if rc not in (0, 4) or not os.path.exists(out):
        return skipped(f'check_assembly rc {rc}: {text.strip()[-200:]}')
    try:
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    except Exception as exc:
        return skipped(f'check_assembly json unreadable: {exc}')
    return assembly_component(doc, rc)


def score_drc(root: str, board: str, clearance=None, sizes=None,
              baseline=None) -> tuple:
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
    if baseline:
        args += ['--baseline', baseline]
    rc, out = run_tool(root, 'check_drc.py', *args)
    # #1390: .kicad_dru size rules check_drc could not model. The copper they
    # bind is ungraded, whatever `undersized` says about the rest.
    _unsz = sorted(set(re.findall(r"NOT GRADED \(size\): rule '([^']*)'", out)))
    _size_extra = {'ungraded_rules': _unsz} if _unsz else {}
    if 'NO DRC VIOLATIONS FOUND' in out:
        return ({'ran': True, 'count': 0, 'by_type': {}, 'graded_at': _graded_at(out),
                 'graphic_grazes_unverified': _unverified_grazes(out),
                 'via_in_paste': {'ran': True, 'count': 0, 'by_type': {},
                                  'undeclarable': _undeclarable_vias(out)}},
                {'ran': True, 'count': 0, 'by_type': {}, **_size_extra},
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
    vip = {t: n for t, n in by_type.items() if t in VIA_PASTE_TYPES}
    pairs = {t: n for t, n in by_type.items() if t in PAIR_RULE_TYPES}
    clear = {t: n for t, n in by_type.items()
             if t not in SIZE_TYPES and t not in RULE_PAIR_TYPES
             and t not in VIA_PASTE_TYPES and t not in PAIR_RULE_TYPES}
    return ({'ran': True, 'count': sum(clear.values()), 'by_type': clear,
             'graded_at': _graded_at(out),
             'graphic_grazes_unverified': _unverified_grazes(out),
             'pair_rules': pairs,
             'via_in_paste': {'ran': True, 'count': sum(vip.values()), 'by_type': vip,
                              'undeclarable': _undeclarable_vias(out)}},
            {'ran': True, 'count': sum(size.values()), 'by_type': size,
             **_size_extra},
            {'ran': True, 'count': sum(rule.values()), 'by_type': rule})


def _undeclarable_vias(out: str) -> int:
    """Vias in a paste opening on a pre-KiCad-10 file, which cannot carry
    per-via capping/filling at all (#962). check_drc ACCEPTS them (the
    requirement belongs on the fab drawing) and counts them on its console
    line; this reads that count so `count` is not mistaken for all of them."""
    m = re.search(r'(\d+) undeclarable in this file format', out)
    return int(m.group(1)) if m else 0


def _unverified_grazes(out: str) -> int:
    """Footprint graphic copper grazing the edge that check_drc ACCEPTED
    without knowing whether a part move made the graze (#962).

    Without --baseline, check_drc cannot tell a library graze (an antenna's
    own art) from one a placement lap created; both are accepted and count toward
    nothing. The number is disclosed so a reader knows `count`
    does not cover them. Copper PAST the outline is `graphic-off-board` and is
    counted either way.
    """
    m = re.search(r'ACCEPTED as immutable-graphic: \d+ row\(s\) \(([^)]*)\)', out)
    if not m:
        return 0
    u = re.search(r'unverified (\d+)', m.group(1))
    return int(u.group(1)) if u else 0


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


def declared_pair_classes(board: str, names: dict, pairs, member=None) -> dict:
    """{class: ['P / N', ...]} -- the net classes the project declares for
    differential pairs (#1388): a class other than Default whose
    ``diff_pair_gap`` or ``diff_pair_width`` differs from Default's (KiCad
    writes both keys into every class, so a value merely present declares
    nothing), restricted to the pairs (KiCad's MatchDpSuffix couples) with a
    member net in it. {} when the project declares none."""
    try:
        from list_nets import read_design_rules, net_class_memberships
        dr = read_design_rules(board)
        classes = dr.get('classes') or {}
        base = classes.get('Default') or {}
        decl = {c for c, v in classes.items() if c != 'Default'
                and any(k in v and v[k] != base.get(k)
                        for k in ('diff_pair_gap', 'diff_pair_width'))}
        if not decl:
            return {}
        if member is None:
            member = net_class_memberships(board, names, dr)
    except Exception:                                        # noqa: BLE001
        return {}
    out = {}
    for p, n in pairs:
        for c in (member.get(p, set()) | member.get(n, set())) & decl:
            out.setdefault(c, []).append(f'{names[p]} / {names[n]}')
    return {c: sorted(v) for c, v in out.items()}


def score_diff_pairs(board: str, drc: dict, pcb=None) -> dict:
    """Every differential pair on the board, graded on its copper (#1303).

    The pairs are KiCad's (`kicad_dru.coupled_net_pairs`: a MatchDpSuffix
    complement that exists). Each one whose members' pads are all joined by
    track copper is MEASURED as KiCad's DRC measures it
    (`net_queries.diff_pair_copper`): coupled and uncoupled length under the
    pair's binding diff_pair_gap window (`design_rules.DesignRules`), track
    lengths, the layers each member uses, and the P/N skew of
    `net_match_length(mode 'total')`. On the board that motivated this
    (imx6ulz, gp14 H10) route_diff reported DQS0 'coupled' while P ran on
    In2.Cu, N on B.Cu -- 0 mm coupled, 4.5 mm skew -- and no grader said so.

    GRADED only against what the board DECLARES: its `.kicad_dru` skew and
    diff_pair_uncoupled rules, which check_drc's pair-rule pass grades and
    this component counts (`count`; the rows leave `drc`, so nothing is
    counted twice). A board that declares neither is measured, not graded:
    `ran` False with the measurements attached, listed in UNGRADED -- KiCad
    itself grades no pair without a rule, and an invented threshold would
    fail every pair a design never asked to couple (a `+`/`-` named analog
    input is a pair to MatchDpSuffix). `no_coupled_copper` names the
    measured pairs with no coupled run at all, report-only."""
    rows = (drc or {}).get('pair_rules') or {}
    try:
        from kicad_parser import parse_kicad_pcb
        from kicad_dru import coupled_net_pairs, read_board_length_rules
        from net_queries import diff_pair_copper, net_pads_joined, CopperIndex
        from design_rules import DesignRules
        if pcb is None:
            pcb = parse_kicad_pcb(board)
        rules, _notes = read_board_length_rules(
            board, kinds=('skew', 'diff_pair_uncoupled'))
        names = {nid: n.name for nid, n in pcb.nets.items() if nid and n.name}
        pairs = coupled_net_pairs(names)
        measured, unjoined = {}, []
        index = dr = None
        for p, n in sorted(pairs, key=lambda pn: names[pn[0]]):
            if len(pcb.pads_by_net.get(p) or ()) < 2 or len(pcb.pads_by_net.get(n) or ()) < 2:
                continue
            if not (net_pads_joined(pcb, p) and net_pads_joined(pcb, n)):
                unjoined.append({'p_net': names[p], 'n_net': names[n]})
                continue
            if index is None:
                index, dr = CopperIndex(pcb), DesignRules.from_project(pcb, board)
            gap = dr.resolve('diff_pair_gap', dr.item_for_net(p, 'track'))
            gmin = gap.kicad_min if gap.kicad_min is not None else gap.min
            rec = diff_pair_copper(pcb, p, n, gmin, gap.max, index=index)
            rec.update(n_net=names[n], gap_window_mm=[gmin, gap.max])
            measured[names[p]] = rec
    except Exception as exc:                                 # noqa: BLE001
        return {'ran': True, 'count': None,
                'reason': f'diff-pair measurement failed: {type(exc).__name__}: {exc}'}
    # `pairs` is keyed by the P net, each record naming its `n_net`.
    detail = {'pairs': measured, 'unjoined_pairs': unjoined,
              'no_coupled_copper': sorted(k for k, r in measured.items()
                                          if r['coupled_mm'] <= 0),
              'by_type': rows}
    # #1388: a rule whose condition matches no net grades nothing, and
    # "N rules, 0 violations" read as a graded, clean component. Only the
    # rules that bind something count as declaring what the pairs must meet.
    try:
        from kicad_dru import unbound_length_rules
        from list_nets import net_class_memberships
        member = net_class_memberships(board, names)
        unbound = [r for kind in ('skew', 'diff_pair_uncoupled')
                   for r in unbound_length_rules(
                       [x for x in rules if x.kind == kind], names, member)]
    except Exception:                                        # noqa: BLE001
        member, unbound = {}, []
    if unbound:
        detail['unbound_rules'] = [r.name for r in unbound]
    live = [r for r in rules if all(r is not u for u in unbound)]
    if not live:
        why = ('no .kicad_dru skew / diff_pair_uncoupled rule declares what '
               'the pairs must meet' if not rules else
               f"the .kicad_dru rule(s) {', '.join(repr(r.name) for r in unbound)} "
               f"bind no net on this board, so nothing declares what the pairs "
               f"must meet")
        # A declared pair class is a declaration the board makes: it is named
        # (`declared`) so the completion check can refuse to call the board
        # done on pairs nothing grades.
        declared = declared_pair_classes(board, names, pairs, member)
        if declared:
            detail['declared'] = declared
            why += ('; the project DECLARES diff-pair class(es) '
                    + ', '.join(f"{c} ({len(v)} pair(s))"
                                for c, v in sorted(declared.items())))
        return dict(skipped(why + '; measured, not graded'), **detail)
    if (drc or {}).get('count') is None:
        # check_drc grades the rules; without its run the rows are unknown,
        # not zero.
        return {'ran': True, 'count': None, 'rules': len(rules), **detail,
                'reason': 'check_drc did not grade the board'}
    return {'ran': True, 'count': sum(rows.values()), 'rules': len(live), **detail}


def declared_impedance_nets(board: str) -> list:
    """Net names a routing step DECLARED an impedance for (#1389): the
    sibling project's ``net_impedance`` records (``route.py --impedance``),
    minus the ones recorded ``applied: false`` -- no width was solved for those,
    so their copper carries no impedance promise (check_impedance skips them
    the same way). [] when there are none or the records cannot be read."""
    try:
        from protected_nets import read_impedance_specs, pro_path_for_board
        specs = read_impedance_specs(pro_path_for_board(board))
    except Exception:                                        # noqa: BLE001
        return []
    return sorted(nm for nm, sp in specs.items() if sp.get('applied') is not False)


def _glob_literal(name: str) -> str:
    """``name`` as a check_impedance ``--nets`` pattern that matches only
    itself: a net named ``/DQ[0]`` would otherwise be a character class, and
    an active-low ``!RESET`` an exclusion."""
    lit = re.sub(r'([*?\[])', r'[\1]', name)
    return '\\' + lit if lit.startswith('!') else lit


def score_impedance(root: str, board: str, nets, tmp: str) -> dict:
    """check_impedance -- reference-plane continuity and declared-gap audit.

    Graded: the nets named by ``--impedance-nets``. `count` is the nets that
    fail: a reference-plane crossing, or a coplanar gap the declaration
    promised and the copper does not have (the width was solved ASSUMING it).

    Measured, not counted (#1389): every other net the board's project records
    an impedance declaration for (``route.py --impedance``), reported as
    `declared_failing` -- a declaration nothing named is disclosed rather than
    made a blocker.

    The graded nets are exactly the ones check_impedance would select for
    ``--nets <--impedance-nets>``: the selection is check_impedance's own
    matcher (`net_queries.matches_net_filter`), so a sheet-path net
    (``USB_D*`` names ``/USB_DP``) and ``!`` exclusions select the same nets
    here as there."""
    from net_queries import matches_net_filter, split_net_patterns
    declared = declared_impedance_nets(board)
    asked = list(nets or [])
    if not asked and not declared:
        return skipped('no --impedance-nets given; impedance is ungraded')
    # One check_impedance run measures the union of the asked selection and
    # the declarations. An exclusion in the asked list cannot be OR-ed with
    # the declared literals inside one filter (it would exclude across both,
    # and an exclusion-only list would stop meaning "every other net"), so
    # then every routed net is measured and the split is made below.
    if not declared:
        sel = asked
    elif split_net_patterns(asked)[1]:
        sel = []
    else:
        sel = asked + [_glob_literal(n) for n in declared]
    out_json = os.path.join(tmp, 'impedance.json')
    rc, out = run_tool(root, 'check_impedance.py', board,
                       *(['--nets', *sel] if sel else []), '--json', out_json)
    if not os.path.isfile(out_json):
        return skipped(f'check_impedance.py exited {rc}: {out.strip()[-200:]}')
    with open(out_json, encoding='utf-8') as f:
        d = json.load(f)
    rows = d.get('nets') or []

    declared_set = set(declared)

    def named(n):
        return bool(asked) and matches_net_filter(n, asked)
    failing = sorted(r['net_name'] for r in rows
                     if r.get('crossings')
                     or (r.get('length_no_ground_mm', 0.0)
                         + r.get('length_gap_wrong_mm', 0.0)) > 0.01)
    extra = {}
    if declared:
        extra = {'declared_nets': declared,
                 'declared_failing': [n for n in failing
                                      if n in declared_set and not named(n)]}
    if not asked:
        return dict(skipped('no --impedance-nets given; impedance is ungraded '
                            '(the project\'s impedance declarations are measured '
                            'in declared_failing, not graded)'), **extra)
    graded = [r for r in rows if named(r['net_name'])]
    if not graded:
        # Asked for, but nothing matched -- that is a FINDING (the globs are
        # wrong, or the nets are unrouted), not a pass. ran=True with count=None
        # routes this to UNKNOWN (blocking None, exit 4), never to `ungraded`:
        # the comma-joined form of --impedance-nets silently zero-matched for
        # four whole runs on one board while blocking summed without the
        # component and the exit code read 0 (test-board run 5, journal [11]).
        return {'ran': True, 'count': None,
                'reason': 'no routed nets matched --impedance-nets', **extra}
    broken = sorted(r['net_name'] for r in graded
                    if (r.get('length_no_ground_mm', 0.0)
                        + r.get('length_gap_wrong_mm', 0.0)) > 0.01)
    crossing = sorted(r['net_name'] for r in graded if r.get('crossings'))
    return {'ran': True, 'count': len(set(broken) | set(crossing)),
            'nets_with_crossing': len(crossing),
            'nets_broken_promise': broken,
            'nets_analyzed': len(graded),
            'crossings': sum(int(r.get('crossings') or 0) for r in graded),
            'segments_over_void': sum(int(r.get('segments_over_void') or 0)
                                      for r in graded),
            **extra}


def score_length(board: str, groups_file: str) -> dict:
    """Length-match spread per declared group, via net_queries (no new geometry).

    `--length-groups` is a JSON file::

        {"BYTE0": {"nets": ["/DQ0", "/DQ1"], "tolerance_mm": 0.1},
         "USB":   {"nets": ["/USB_DP", "/USB_DM"], "tolerance_mm": 0.05,
                   "mode": "pin_pair"}}

    `mode: "pin_pair"` measures the driver->receiver PATH instead of total net
    copper. Use it for any multipoint or stubbed net: total copper sums every
    branch and matches no real signal path.

    BOTH modes measure a member only once its pads are joined by track copper
    (`net_queries.net_match_length`), and a member that is not makes its whole
    group UNMEASURED (#1246). Total copper used to grade whatever the net had:
    no copper read 0.0 mm, so two unrouted nets passed at spread 0.0, and
    fanout stubs read their stub length, so a stub-only group FAILED and added
    to `blocking` on top of `broken`. A partial group is unmeasured too, rather
    than graded on the members that happen to be routed: a spread over a subset
    is not the group's spread, and a pass on it would be a pass nobody earned.

    Each length includes the member's PAD-TO-DIE term (`(die_length ...)`,
    #1248), as KiCad's length/skew DRC does: every pad of the net in total
    mode, the measured pad pair in pin_pair mode. A group whose lengths carry
    one names it in `pad_to_die_mm`.
    """
    if not groups_file:
        return skipped('no --length-groups given; length matching is ungraded')
    if not os.path.isfile(groups_file):
        return skipped(f'length-groups file not found: {groups_file}')
    from kicad_parser import parse_kicad_pcb
    from net_queries import net_match_length, net_match_pad_to_die

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
        mode = 'pin_pair' if spec.get('mode') == 'pin_pair' else 'total'
        lengths, unjoined = {}, []
        for n in names:
            if n not in by_name:
                continue
            L = net_match_length(pcb, by_name[n], mode=mode)
            if L is None:
                unjoined.append(n)
            else:
                lengths[n] = L
        if unjoined:
            # NOT a failure. "Not routed yet" is what `unrouted`/`broken`
            # already measure, and counting it here too made every 0-copper
            # board score length=1 per group -- a permanent phantom blocker
            # that no routing could ever clear. Distinct from the
            # missing-nets case above, which IS a finding about the spec.
            detail[gname] = {'skipped': f'{len(unjoined)} of {len(ids)} member(s) '
                                        f'have pads not joined by track copper '
                                        f'(unrouted, stub-only, broken or '
                                        f'plane-only)',
                             'unjoined_nets': unjoined, 'missing_nets': missing}
            unmeasured += 1
            continue
        spread = max(lengths.values()) - min(lengths.values())
        ok = spread <= tol
        failures += 0 if ok else 1
        detail[gname] = {'spread_mm': round(spread, 4), 'tolerance_mm': tol,
                         'pass': ok, 'missing_nets': missing,
                         'worst': max(lengths, key=lengths.get)}
        # #1248: the lengths include each member's pad-to-die term, as KiCad's
        # DRC does. Disclosed only where a pad declares one, so a grade moved
        # by a term no copper shows says which term moved it.
        die = {n: net_match_pad_to_die(pcb, by_name[n], mode) for n in lengths}
        if any(die.values()):
            detail[gname]['pad_to_die_mm'] = {n: round(v, 4)
                                              for n, v in die.items() if v}
    if unmeasured and unmeasured == len(detail):
        # Nothing could be measured at all -- report UNGRADED rather than a
        # clean 0, so the gate cannot be passed by a board nothing examined.
        return dict(skipped('no group had every member joined by track '
                            'copper yet'),
                    groups=detail)
    return {'ran': True, 'count': failures, 'groups': detail,
            'unmeasured_groups': unmeasured}



def _pad_neck_zone_mm() -> float:
    """Length of the router's pad-entry zone: the stretch of a power route at
    each pad end that may run at the pad's own width (#1318)."""
    try:
        from routing_config import GridRouteConfig
        return float(GridRouteConfig.__dataclass_fields__['neckdown_length'].default)
    except Exception:                                           # noqa: BLE001
        return 2.5


def score_net_widths(board: str, spec_file: str) -> dict:
    """Per-net REQUIRED widths -- the gap `undersized` structurally cannot see.

    `undersized` comes from check_drc's size floors, which are BOARD-WIDE
    minima: they answer "is any copper thinner than X?". A spec that demands a
    PARTICULAR net be at least a given width is a different question, and the
    difference is not academic -- it is the one that shipped. Measured:
    `undersized` read 0 while 47 segments breached a >=0.4 mm rail requirement,
    and a pair spec'd at 0.8 mm that came out at 0.16 mm would not be caught
    either, because 0.16 mm clears every board-wide floor.

    The requirements are `spec_file`, a JSON file of
    ``{"<net glob>": <min mm>}``, e.g.::

        {"USB_D*": 0.8, "VCC3V3": 0.4, "VBUS": 0.4, "XIN": 0.15}

    First matching glob wins, so list the specific patterns before the broad
    ones. They are graded: a net under its requirement counts.

    The widths the routing steps were ASKED for (route.py's --power-nets /
    --power-nets-widths, recorded in the board's `kicad_routing_tools`
    records, #1379) are MEASURED the same way for every net no file pattern
    covers and reported as `requested_under`, without counting: the router
    narrows a power route where its width does not fit and discloses it
    (neck-down), so a request is what it aimed for, not what the board owes.
    To hold a net to a width, put it in `spec_file`.

    A segment under its requirement is still excused where it is a PAD NECK
    (#1318): within the router's pad-entry zone of a pad of its own net that is
    narrower than the requirement, at no less than that pad's short side --
    the width the router lays a pad entry at where the full width cannot sit
    on the pad. Narrower than the pad, or beyond the zone, it counts: only the
    LENGTH of a segment inside the zone is excused, so a long run that merely
    starts at the pad counts for everything past it. What was excused is
    disclosed per net (`pad_neck_allowed_mm`).

    Nets with no copper are not counted here -- that is `unrouted`'s job.
    """
    import fnmatch
    import math
    from collections import defaultdict
    from kicad_parser import parse_kicad_pcb

    want = {}
    if spec_file:
        if not os.path.isfile(spec_file):
            return skipped(f'net-min-widths file not found: {spec_file}')
        with open(spec_file, encoding='utf-8') as fh:
            want = json.load(fh)
        # Non-numeric values are annotations (a "_comment" key), not net
        # patterns; letting them through pollutes patterns_matching_no_routed_net
        # -- the exact field a reader scans for typo'd globs. Same idiom as
        # check_dru's floors filter.
        want = {k: v for k, v in want.items() if isinstance(v, (int, float))}
    recorded = {}
    try:
        from protected_nets import read_power_widths, pro_path_for_board
        recorded = read_power_widths(pro_path_for_board(board))
    except Exception:                                           # noqa: BLE001
        recorded = {}
    if not want and not recorded:
        return skipped('no --net-min-widths given; per-net widths are ungraded')
    pcb = parse_kicad_pcb(board)
    by_id = {n.net_id: n.name for n in pcb.nets.values()}
    seen = defaultdict(list)
    for seg in pcb.segments:
        name = by_id.get(seg.net_id)
        if name:
            seen[name].append((seg.width, math.hypot(seg.end_x - seg.start_x,
                                                     seg.end_y - seg.start_y),
                               seg))

    def required(name):
        """(mm, graded): a file pattern's requirement is graded, a recorded
        request only measured."""
        for pat, mm in want.items():
            if fnmatch.fnmatchcase(name, pat):
                return mm, True
        mm = recorded.get(name)
        return (mm, False) if mm is not None else (None, False)

    zone = _pad_neck_zone_mm()
    pads_of = defaultdict(list)
    for n_id, name in by_id.items():
        for pad in pcb.pads_by_net.get(n_id, ()) or ():
            pads_of[name].append((pad.global_x, pad.global_y,
                                  min(pad.size_x, pad.size_y),
                                  max(pad.size_x, pad.size_y)))

    def neck_length(name, seg, req):
        """The LENGTH of `seg` that lies in a pad-entry zone of a qualifying
        own-net pad -- only that much is a neck; the rest of the segment is
        not excused by touching the zone at one end."""
        spans = [_segment_span_in_disc(seg, px, py, zone + long_ / 2.0)
                 for px, py, short, long_ in pads_of.get(name, ())
                 if short < req - 1e-9 and seg.width >= short - 1e-3]
        return _union_length(s for s in spans if s)

    failures, detail, allowed, requested_under = 0, {}, {}, {}
    for name, widths in sorted(seen.items()):
        req, graded = required(name)
        if req is None:
            continue
        under, excused = [], 0.0
        for w, L, seg in widths:
            if w < float(req) - 1e-9:
                neck = min(L, neck_length(name, seg, float(req)))
                excused += neck
                if L - neck > 1e-6:
                    under.append((w, L - neck))
        if excused:
            allowed[name] = round(excused, 2)
        if under:
            # #1033: LENGTH, not only a segment count. A count cannot tell
            # a 0.2 mm pad neck from a 59 mm pad-to-pad run at the signal
            # width -- run 32's +3V3 read 561/1143 segments, which hid that
            # a third of its copper LENGTH was at 0.127.
            _tot = sum(L for _w, L, _s in widths)
            _und = sum(L for _w, L in under)
            (detail if graded else requested_under)[name] = {
                            'required_mm': float(req),
                            'narrowest_mm': round(min(w for w, _L in under), 4),
                            'segments_under': len(under),
                            'segments_total': len(widths),
                            'length_under_mm': round(_und, 2),
                            'length_total_mm': round(_tot, 2),
                            'length_under_share': (round(_und / _tot, 4)
                                                   if _tot > 0 else 0.0)}
            if graded:
                failures += 1
    unmatched = [p for p in want
                 if not any(fnmatch.fnmatchcase(n, p) for n in seen)]
    extra = {'requested_under': requested_under,
             'requirements_from': {'file': len(want), 'recorded': len(recorded)},
             'pad_neck_allowed_mm': allowed}
    if not want:
        return dict(skipped('no --net-min-widths given; per-net widths are '
                            'ungraded (the requested power widths are measured '
                            'in requested_under, not graded)'), **extra)
    return {'ran': True, 'count': failures, 'nets': detail,
            'patterns_matching_no_routed_net': unmatched, **extra}


def _segment_span_in_disc(seg, cx, cy, r):
    """(start, end) distances along `seg` of its part inside the disc of
    radius `r` around (cx, cy), or None when no part of it is."""
    import math
    dx, dy = seg.end_x - seg.start_x, seg.end_y - seg.start_y
    fx, fy = seg.start_x - cx, seg.start_y - cy
    a = dx * dx + dy * dy
    c = fx * fx + fy * fy - r * r
    if a <= 0:
        return (0.0, 0.0) if c <= 0 else None
    b = 2.0 * (fx * dx + fy * dy)
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return None
    root = math.sqrt(disc)
    f0 = max(0.0, (-b - root) / (2.0 * a))
    f1 = min(1.0, (-b + root) / (2.0 * a))
    if f1 <= f0:
        return None
    L = math.sqrt(a)
    return (f0 * L, f1 * L)


def _union_length(spans) -> float:
    """Total length covered by (start, end) intervals, overlaps counted once."""
    total, end = 0.0, None
    for a, b in sorted(spans):
        if end is None or a > end:
            total += b - a
            end = b
        elif b > end:
            total += b - end
            end = b
    return total


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
        'components.length.groups[].unjoined_nets':
            [n for g in (length.get('groups') or {}).values()
             if isinstance(g, dict) for n in (g.get('unjoined_nets') or ())],
        'components.length.groups[].pad_to_die_mm':
            [n for g in (length.get('groups') or {}).values()
             if isinstance(g, dict) for n in (g.get('pad_to_die_mm') or {})],
        # #1303: diff pairs, keyed by the P net with the N net inside.
        'components.diff_pairs.pairs':
            [n for p, r in ((c.get('diff_pairs') or {}).get('pairs') or {}).items()
             for n in (p, r.get('n_net')) if n],
        'components.diff_pairs.unjoined_pairs':
            [n for r in ((c.get('diff_pairs') or {}).get('unjoined_pairs') or ())
             for n in (r.get('p_net'), r.get('n_net')) if n],
    }
    return {k: v for k, v in fields.items() if v}


def audit_net_names(board: str, score: dict, pcb=None) -> dict:
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
        if pcb is None:
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


def quality(board: str, pcb=None) -> dict:
    """Tie-breakers, compared ONLY once blocking == 0. Never a blocker itself:
    a board is not worse for having more copper if the alternative is a
    disconnected net."""
    try:
        from kicad_parser import parse_kicad_pcb
        import math
        if pcb is None:
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


def build_parser():
    p = argparse.ArgumentParser(
        description='One authoritative (blocking, quality) score for a board',
        epilog='Exit 0 = blocking is 0; 4 = graded with blockers; '
               '3 = board state; 2 = bad arguments; 1 = crash.')
    p.add_argument('board', help='the .kicad_pcb to score')
    p.add_argument('--intent', help='floorplan intent JSON (check_floorplan '
                                    '--intent). Omitted = floorplan ungraded')
    p.add_argument('--baseline', metavar='BOARD',
                   help='the board this one was derived from (the unrouted '
                        'input). check_drc then accepts the vias it already had '
                        'in a paste opening as inherited, and grades a graze of '
                        'footprint graphic copper that a part MOVE created (#962). '
                        'A via-in-paste left after that is the run\'s own and '
                        'counts in BLOCKING (#1171). check_assembly then GATES '
                        'a courtyard pair a moved part entered (#1183). '
                        'Omitted, that gate is listed in `ungraded`')
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
    p.add_argument('--impedance-nets', nargs='+', action='extend', metavar='GLOB',
                   help='route.py --nets glob syntax, SPACE separated; commas '
                        'inside a token are split too (a comma-joined list used '
                        'to become one impossible glob that silently matched '
                        'nothing). Enables the impedance component')
    p.add_argument('--net-min-widths', metavar='JSON',
                   help='JSON FILE of {"<net glob>": <min mm>} -- per-net width '
                        'requirements. `undersized` only sees BOARD-WIDE floors, '
                        'so a spec demanding a particular net be wider (a 0.8mm '
                        'USB pair, 0.4mm rails) is invisible to it. First '
                        'matching glob wins. The widths route.py was ASKED for '
                        '(--power-nets-widths) are read from the board\'s '
                        'records with no flag and MEASURED for the nets no '
                        'pattern covers (requested_under), not graded')
    p.add_argument('--length-groups', metavar='JSON',
                   help='JSON FILE of {"group": {"nets": [...], '
                        '"tolerance_mm": 0.1, "mode": "pin_pair"}} -- enables '
                        'the length component. A PATH, like --net-min-widths: '
                        'anything that is not an existing file leaves length '
                        'matching UNGRADED (ran: false) and the board scoring '
                        'exit 0, which reads as a pass')
    p.add_argument('--json', metavar='PATH',
                   help='write the full score here. The stdout SCORE_JSON= '
                        'line caps each components.assembly list (pairs, '
                        'containments, locked_contact_pairs, '
                        'coincident_origin_groups) at %d entries and says so '
                        'in <list>_count / <list>_truncated; this file carries '
                        'every entry' % STDOUT_LIST_CAP)
    p.add_argument('--label', default='', help='free text carried into the JSON '
                                               '(the ledger uses it for the lever)')
    p.add_argument('--quiet', '-q', action='store_true',
                   help='accepted for callers that pass it; changes nothing. '
                        'stdout is already SCORE_JSON=, the BLOCKING= summary '
                        'and the disclosure lines (ADVISORY, UNGRADED, ...), '
                        'which stay because they are the difference between '
                        'clean and unexamined')
    return p


def main():
    args = build_parser().parse_args()
    if not os.path.isfile(args.board):
        print(f"board not found: {args.board}", file=sys.stderr)
        return 3
    if args.baseline is not None and not os.path.isfile(args.baseline):
        # Unreadable, the gate it arms would sit unarmed with the caller
        # believing it graded (#1183).
        print(f"baseline not found: {args.baseline}", file=sys.stderr)
        return 2
    _bad = baseline_problem(args.baseline) if args.baseline else None
    if _bad:
        # A file that is not a board with footprints arms the gate against
        # EVERY part: none is found in it, so each reads as moved (phase-6
        # verifier: pristine ulx3s, 8 of 8 pairs gating against an empty
        # file, 0 of 8 against itself).
        print(f"baseline not usable: {args.baseline}: {_bad}",
              file=sys.stderr)
        return 2
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
        drc, undersized, rule_pairs = score_drc(root, args.board, args.clearance, sizes,
                                                baseline=args.baseline)
        floorplan = score_floorplan(root, args.board, args.intent, tmp)
        assembly = score_assembly(root, args.board, args.intent, tmp,
                                  args.clearance, baseline=args.baseline)
        _imp_nets = ([g for tok in args.impedance_nets for g in tok.split(',') if g]
                     if args.impedance_nets else args.impedance_nets)
        imped = score_impedance(root, args.board, _imp_nets, tmp)
        length = score_length(args.board, args.length_groups)
        net_widths = score_net_widths(args.board, args.net_min_widths)

    # Both connectivity components carry their work list, not just their count --
    # see score_connectivity. `unrouted` needs names; `broken` needs names, piece
    # counts and the stranded pads, or 9.1a's lever 2 has nothing to act on.
    # ...and `unrouted` carries its SHAPE. `count` is deliberately untouched, so
    # `blocking` below is unchanged by construction.
    # ONE in-process parse for the three readers below (#1194): each parsed
    # the board itself, so a parser warning (the two-line NET-TAGGED copper
    # graphic notice) printed three times. Read-only for all three. A board
    # that will not parse leaves each to parse and report for itself.
    try:
        from kicad_parser import parse_kicad_pcb
        _pcb = parse_kicad_pcb(args.board)
    except Exception:                                        # noqa: BLE001
        _pcb = None
    shape = unrouted_shape(args.board, conn.get('unrouted_net_names', []),
                           pcb=_pcb)
    diff_pairs = score_diff_pairs(args.board, drc, pcb=_pcb)
    parts = {'unrouted': {'ran': conn['ran'], 'count': conn.get('unrouted'),
                          'nets': conn.get('unrouted_net_names', []),
                          'pads': conn.get('unrouted_pads', {}),
                          **shape},
             'broken': {'ran': conn['ran'], 'count': conn.get('broken'),
                        'poured_nets': conn.get('poured_nets', []),
                        'poured_nets_meaning': POURED_NETS_MEANING,
                        'nets': conn.get('broken_detail', {})},
             'drc': drc, 'undersized': undersized, 'floorplan': floorplan,
             'assembly': assembly,
             'impedance': imped, 'length': length, 'net_widths': net_widths,
             'diff_pairs': diff_pairs}

    # Track-rule-governed pairs are ADVISORY: their gate is the repo's own
    # registered-floor checker (check_dru), not this scalar. They live beside
    # `parts`, never in it -- the blocking sum below iterates parts, and 610
    # floor-governed pairs must not drown ~17 physical defects (run 6).
    advisory = {'drc_rule_pairs': rule_pairs,
                'drc_via_in_paste': drc.get('via_in_paste')
                or {'ran': drc.get('ran'), 'count': None, 'by_type': {}}}
    # #1171: with --baseline, check_drc has already accepted every via the
    # input had under solder (`inherited-via-in-paste`), so what is left is a
    # site THIS run made -- a via it laid or re-laid there, or a part it moved
    # onto one -- and the advisory rationale above ("a via the input already
    # had") no longer holds. It blocks like any other defect.
    if args.baseline:
        parts['drc_via_in_paste'] = advisory.pop('drc_via_in_paste')

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
             # the joins the open nets still need: a `blocking` tie's
             # breaker, ahead of `quality` (#1340); None when unmeasured
             'joins_needed': conn.get('joins_needed') if conn.get('ran') else None,
             'blocking_by': {k: v.get('count') for k, v in parts.items()},
             'advisory': {k: v.get('count') for k, v in advisory.items()},
             'ungraded': ungraded_entries(parts),
             'unknown': sorted(unknown),
             'quality': quality(args.board, pcb=_pcb),
             'components': {**parts, **advisory},
             'floors': _floors(args.board, sizes),
             'connectivity_nets': conn.get('nets', []),
             'tool_seconds': {k: round(v, 2)
                              for k, v in sorted(TOOL_SECONDS.items())}}
    # Self-check the payload against the board it just graded (see
    # audit_net_names). Computed AFTER `score` is assembled so it audits what is
    # actually published, not what this function believes it published.
    score['net_name_audit'] = audit_net_names(args.board, score, pcb=_pcb)

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(score, f, indent=2, sort_keys=True)

    # The assembly lists capped (#1194); `--json` above has every entry.
    print('SCORE_JSON=' + json.dumps(stdout_payload(score), sort_keys=True,
                                     separators=(',', ':')))
    bits = ' '.join(f'{k}={v}' for k, v in score['blocking_by'].items()
                    if v is not None)
    q = score['quality']
    print(f"BLOCKING={blocking}  ({bits})  "
          f"vias={q.get('vias')} copper_mm={q.get('copper_mm')}  "
          f"joins_needed={score['joins_needed']}")
    _adv_bits = ' '.join(f'{k}={v}' for k, v in score['advisory'].items() if v)
    if _adv_bits:
        print(f"ADVISORY (floor-governed, not blocking): {_adv_bits}")
    # #1303: what each pair's copper delivers, whether or not a rule grades it.
    _dp = score['components'].get('diff_pairs') or {}
    if _dp.get('pairs'):
        _worst = sorted(_dp['pairs'].items(),
                        key=lambda kv: -(kv[1].get('skew_mm') or 0))[:3]
        print(f"DIFF PAIRS: {len(_dp['pairs'])} measured"
              + ('' if _dp.get('ran') else
                 ' (not graded: no .kicad_dru skew / diff_pair_uncoupled rule'
                 + (' binds a net' if _dp.get('unbound_rules') else '') + ')')
              + "; largest skew: " + ', '.join(
                  f"{k} {v['skew_mm']:.3f}mm "
                  f"(coupled {v['coupled_mm']:.2f} of "
                  f"{max(v['length_p_mm'], v['length_n_mm']):.2f}mm)"
                  for k, v in _worst))
        if _dp.get('declared'):
            print(f"  DECLARED, NOT GRADED: the project declares diff-pair "
                  f"class(es) {', '.join(sorted(_dp['declared']))} and no rule "
                  f"grades their pairs -- add a .kicad_dru skew / "
                  f"diff_pair_uncoupled rule")
        if _dp.get('no_coupled_copper'):
            print(f"  NO COUPLED COPPER (P and N never run side by side): "
                  f"{', '.join(_dp['no_coupled_copper'])}")
    # Measured, not counted (#1379, #1389): what the routing steps were asked
    # for, and the impedance declarations no --impedance-nets named.
    _ru = (score['components'].get('net_widths') or {}).get('requested_under') or {}
    if _ru:
        print(f"REQUESTED POWER WIDTHS (measured, not graded): {len(_ru)} net(s) "
              f"ship copper under the width route.py was asked for: " + ', '.join(
                  f"{n} {r['length_under_mm']:.1f} of {r['length_total_mm']:.1f} mm "
                  f"under {r['required_mm']:g}" for n, r in sorted(_ru.items())[:6])
              + (', ...' if len(_ru) > 6 else '')
              + " -- name a net in --net-min-widths to grade it")
    _df = (score['components'].get('impedance') or {}).get('declared_failing') or []
    if _df:
        print(f"DECLARED IMPEDANCE (measured, not graded): {len(_df)} net(s) the "
              f"project records an impedance for fail reference-plane continuity "
              f"or their coplanar gap: {', '.join(_df[:8])}"
              + (', ...' if len(_df) > 8 else '')
              + " -- name them in --impedance-nets to grade them")
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
