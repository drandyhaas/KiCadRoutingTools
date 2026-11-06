#!/usr/bin/env python3
"""Prove a routing plan before it runs. Read-only, no model in the loop.

    python3 -X utf8 route_plan_check.py <board>_plan.sh
    python3 -X utf8 route_plan_check.py <run dir> --board board.kicad_pcb
    python3 -X utf8 route_plan_check.py wk/<run>/judge_chain.txt --board board.kicad_pcb
    python3 -X utf8 route_plan_check.py <plan> --list

Exit: 0 checked and clean, 1 crash, 2 usage, 3 the plan could not be read,
4 a rule failed. Deliberately the fail-closed dialect `board_score.py` uses --
1 is a CRASH, never a finding, so a traceback can never read as a verdict.

TWO FORMATS, ONE SET OF RULES (#1592)
-------------------------------------
A `<board>_plan.sh` names its boards by path and begins with `cd <repo>`. A
JUDGING CHAIN -- the file `py_placer/judge_seeds.py` runs, which
plan-pcb-placement 3.7 has the agent write -- names them by placeholder
(`{in}` the board a step reads, `{out}` the one it writes) and is run from
the repo root with no shell. A file whose commands name a judge_seeds
placeholder is read as a chain, exactly as judge_seeds reads it
(`read_chain`), and every rule applies with `{in}`/`{out}` as its boards --
a step with `{out}` is one that writes a board -- except R01: judge_seeds
asks a chain for no `cd`. A chain judge_seeds would refuse (no `{in}`, no
`{out}`, an unknown placeholder, a shell builtin such as that `cd`) is exit
3, for judge_seeds' own reason.

WHY A CHECKER AND NOT A DRIVER
-------------------------------
The placement half stages its work behind a driver because the AI is DECIDING
there, and a refusal is the mechanism that keeps a decision honest. Routing's
failure mode is different: the CHAIN is wrong, not a judgement. A chain is a
recorded artifact, so it can be PROVED instead of supervised -- which is also
the only thing that works at the standalone door, where `/plan-pcb-routing`
runs under `ai_backend.ANALYSIS_CONSTRAINT` ("analysis and planning only: do
not execute any routing commands and do not modify any files") and a stage
driver could not run at all.

WHAT IT READS, AND WHY NOT THE CONVERTED PLAN
----------------------------------------------
`make_plan.plan_commands` -> `redo_stress_test.parse_manifest`: every recorded
command, in file order, nothing pruned. NOT `plan_steps_from_manifest`, which
is the GUI conversion and drops exactly what these rules are about -- measured
on a skill-compliant plan, the converted steps see 0 of the three verification
commands, 0 of the `cd`/`cp` lines, and cannot tell `route_planes` from
`repair_planes` (both become one action). Its pruner can also nominate a
RETRY as the chain end, which would make the "ends on route.py" rule grade a
chain the file does not describe.

Two rules need the file TEXT rather than the commands, because they are about
what appears as a COMMENT, and `parse_manifest` drops comments: a compliant
plan and a plan with no verification at all are identical to it.

WHAT IT DOES NOT CHECK, AND WHO DOES
--------------------------------------
Nine rules in the skill need a routed board or a completed run. They are
listed by `--list` with the tool that owns each, and printed at the end of
every run. An unchecked rule that says so is honest; one that is silently
absent is the failure this whole issue is about.
"""
import argparse
import fnmatch
import os
import re
import sys

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(HERE))))
for _d in (ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'py_placer')):
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.insert(0, _d)

import routing_defaults  # noqa: E402  (needs the sys.path bootstrap above)

CLEAN, CRASH, USAGE, UNREADABLE, REFUSED = 0, 1, 2, 3, 4

#: The rules this checker cannot decide, each with the tool that can. Printed
#: every run. A plan is not "proved" by this exiting 0 -- it is proved as far
#: as a recorded chain can be, and these are the rest.
DELEGATED = (
    ('the final board has no unrouted net',
     'py_router/check_connected.py'),
    ('no DRC violation at the clearance the board was routed to',
     'py_router/check_drc.py'),
    ('no orphan stub, dead-end antenna or stacked duplicate',
     'py_tools/check_orphan_stubs.py, py_router/check_weird.py'),
    ('no redundant copper loop', 'py_tools/check_cycles.py'),
    ('fanout escaped every ball it was asked to (failed == 0)',
     "the fanout step's own JSON_SUMMARY.failed"),
    ('diff pairs did not silently peel to single-ended',
     'JSON_SUMMARY.failed_diff_pairs / single_ended_diff_pairs / '
     'partial_diff_pairs'),
    ('emitted width meets each width-specced net minimum',
     'board_score.py --net-min-widths'),
    ('pours kept their promises to carved-off balls',
     'JSON_SUMMARY.pour_served'),
    ('the board is DONE, by an aggregate that fails closed',
     'check_complete.py'),
)


# --------------------------------------------------------------- argv helpers

def tool_of(argv):
    """The basename of the tool a recorded command runs, or None.

    The FIRST `.py` token, not the last: `python3 -X utf8 route.py in out`
    has one, but a command carrying a `.py` path as an ARGUMENT would
    otherwise be misread as running it.
    """
    for tok in argv:
        if tok.endswith('.py'):
            return os.path.basename(tok)
    return None


def values(argv, flag):
    """Every token after `flag` up to the next `--option`. [] when absent."""
    out = []
    if flag not in argv:
        return out
    i = argv.index(flag) + 1
    while i < len(argv) and not argv[i].startswith('--'):
        out.append(argv[i])
        i += 1
    return out


def scalar(argv, flag):
    """`float(flag's single value)`, or None when absent or not a number."""
    got = values(argv, flag)
    try:
        return float(got[0])
    except (IndexError, ValueError):
        return None


def is_kept_join(argv):
    """`check_join.py ... --keep-staged <out>`: the one `check_*` tool that
    writes a board, the hand join a chain continues from (#1397, #1646)."""
    return tool_of(argv) == 'check_join.py' and '--keep-staged' in argv


def is_check(argv):
    """A read-only grader, by the repo's own convention (`check_*`) -- not
    a kept join, which writes a board."""
    t = tool_of(argv)
    return bool(t and t.startswith('check_')) and not is_kept_join(argv)


def boards_in(argv, chain=False):
    """The board tokens: `.kicad_pcb` paths, or in a judging chain the
    tokens naming `{in}` / `{out}` (#1592)."""
    if chain:
        from judge_seeds import placeholders
        return [a for a in argv if placeholders(a) & {'in', 'out'}]
    return [a for a in argv if a.endswith('.kicad_pcb')]


# --------------------------------------------------------------- the context

class Plan:
    """Everything a rule may read: the raw text, and the recorded commands."""

    def __init__(self, path, text, cmds, board=None, chain=False,
                 board_path=None):
        self.path = path
        self.text = text
        self.lines = text.splitlines()
        self.cmds = cmds                      # [(cwd, argv)], file order
        self.board = board                    # PCBData, or None
        self.board_path = board_path          # its file (the project beside it)
        self.chain = chain                    # a judge_seeds chain ({in}/{out})
        self.notes = {}                       # {rule id: [what it could not decide]}
        self._noting = None

    def note(self, text):
        """What a rule could not decide and therefore did not refuse -- printed
        under the rule as NOTE, never a failure (#1709)."""
        self.notes.setdefault(self._noting, []).append(text)

    @property
    def argvs(self):
        """Each command's argv WITHOUT its leading `NAME=value` env prefix,
        which parse_manifest puts there for an `export`ed or recorded knob
        (#1302, #1337), so `argv[0]` is the command run."""
        from redo_stress_test import split_env_prefix
        return [split_env_prefix(argv)[1] for _cwd, argv in self.cmds]

    def by_tool(self, *names):
        return [a for a in self.argvs if tool_of(a) in names]

    @property
    def mutating(self):
        """Commands that write a board: not a check, and naming a board --
        in a chain, naming `{out}`, since judge_seeds reads a step without
        one as writing no board."""
        if self.chain:
            from judge_seeds import placeholders
            return [a for a in self.argvs if not is_check(a) and tool_of(a)
                    and any('out' in placeholders(t) for t in a)]
        return [a for a in self.argvs if not is_check(a) and boards_in(a)
                and tool_of(a)]


# ----------------------------------------------------------------- the rules
# Each returns a list of failure strings; empty means the rule held. The
# citation is the line in plan-pcb-routing/SKILL.md the rule comes from, so a
# refusal can be argued with at its source.

def r_starts_with_cd(p):
    """SKILL.md :2554 -- begin with an explicit `cd <repo>` line, not just a
    `# cwd=` comment, so a bare `bash <board>_plan.sh` works at all."""
    for ln in p.lines:
        s = ln.strip()
        if not s or s.startswith('#') or s in ('set -e',):
            continue
        if s.split()[0] == 'cd':
            return []
        return [f'the first executable line is {s[:70]!r}, not a `cd <repo>`. '
                f'Relative py_router/ paths fail from any other directory.']
    return ['the plan has no executable line at all']


def r_no_pipes(p):
    """SKILL.md :2640 -- no pipes. `manifest_to_plan` tokenises pipe segments
    into NET GLOBS, which is measured, not theoretical: a `2>&1 | tee x.log`
    tail converts to `nets: ['*', '2>&1', '|', 'tee', 'x.log']`.

    A pipe is a `|` the shell reads, outside quotes (#1495): a quoted split
    plane group, `--nets GND '+3V3|+5V'`, is one argument and no pipe."""
    import shlex
    bad = []
    for n, ln in enumerate(p.lines, 1):
        s = ln.strip()
        if not s or s.startswith('#') or '|' not in s:
            continue
        try:
            lx = shlex.shlex(s.rstrip('\\'), posix=True, punctuation_chars='|&;')
            lx.whitespace_split = True
            piped = any(set(t) <= {'|', '&'} and '|' in t for t in lx)
        except ValueError:
            piped = True            # unbalanced quotes: the shell's reading is unknown
        if piped:
            bad.append(f'line {n} carries a pipe: {s[:70]!r}')
    return bad


def r_verification_is_commented(p):
    """SKILL.md :2557 -- verification goes in as COMMENTS, never executable
    lines. The file's exit code is read as the chain verdict, so a trailing
    read-only check that exits non-zero makes a good board report failed
    (measured: half of one 15-board wave reported rc != 0 from exactly this).

    Needs the TEXT: `parse_manifest` drops comments, so to it a compliant plan
    and a plan with no verification are the same file.
    """
    bad = []
    lines = p.lines
    n = 0
    while n < len(lines):
        first, s = n + 1, lines[n].strip()
        while not s.startswith('#') and s.endswith('\\') and n + 1 < len(lines):
            n += 1
            s = s[:-1] + ' ' + lines[n].strip()
        n += 1
        if not s or s.startswith('#'):
            continue
        argv = s.split()
        if is_check(argv):
            bad.append(f'line {first} runs {tool_of(argv)} as an EXECUTABLE line; its exit '
                       f'code becomes the chain verdict. Comment it out.')
    return bad


def r_ends_on_route(p):
    """SKILL.md "End every chain on route.py" -- the last board-mutating
    command is `route.py`. Only route.py finalizes planes, so a chain ending
    on a bare re-pour or a diff step writes a final board no weld/oracle pass
    ever verified. That is a PLAN ERROR, not a tuning choice.

    The one exception: hand joins (`check_join.py --keep-staged`) FOLLOW the
    final route.py, since a route step after a join may rip its net (#1360,
    #1646). They are the only commands that may come after it."""
    muts = list(p.mutating)
    if not muts:
        return ['the plan has no board-mutating command at all']
    while muts and is_kept_join(muts[-1]):
        muts.pop()
    if not muts:
        return ['the plan writes no board but by hand joins: no route.py '
                'finalizes it']
    last = tool_of(muts[-1])
    if last != 'route.py':
        after = (' (the hand joins after it do not change that)'
                 if len(muts) < len(p.mutating) else '')
        return [f'the last board-mutating command is {last}, not route.py{after}. '
                f'Only route.py finalizes planes, so this ships a board whose '
                f'welds and taps nothing verified. Only a hand join '
                f'(check_join.py --keep-staged) may follow the final route.py.']
    return []


def r_checkers_are_present(p):
    """SKILL.md :1958 -- "Always run DRC, connectivity, and orphan stub
    checks". Read together with :2557 and :2562 this means the three appear
    AS COMMENTS after the final route.py, NOT that the plan ends with them:
    ending on them would break the exit-code rule and the route.py rule at
    once. Checked on the text, for the same reason."""
    want = {'check_drc.py': 'DRC', 'check_connected.py': 'connectivity',
            'check_orphan_stubs.py': 'orphan stubs'}
    missing = [f'{name} ({tool})' for tool, name in want.items()
               if tool not in p.text]
    if missing:
        return [f'the plan never names {", ".join(sorted(missing))}. The '
                f'three verification commands belong in the file as '
                f'COMMENTS after the final route.py.']
    return []


def r_diff_gap_not_below_clearance(p):
    """SKILL.md :571 -- never set `--diff-pair-gap` below the same command's
    `--clearance`. KiCad grades the pair's own coupling as a plain clearance
    violation, so route_diff floors the gap up to clearance anyway."""
    bad = []
    for argv in p.by_tool('route_diff.py'):
        gap, clr = scalar(argv, '--diff-pair-gap'), scalar(argv, '--clearance')
        if gap is not None and clr is not None and gap < clr - 1e-9:
            bad.append(f'route_diff --diff-pair-gap {gap} is below the same '
                       f'command\'s --clearance {clr}')
    return bad


def _keeps_a_relief_pour(p, argv):
    """True when this route_planes command's `--thermal-relief` REPLACES a
    designer's relief pour (SKILL.md Step 3, "A pour the board already has",
    #1329): the board (needs --board) already carries a thermal-connected
    pour on a net the command pours. Without the board the claim cannot be
    checked, so it is not credited (#1406)."""
    if tool_of(argv) != 'route_planes.py' or p.board is None:
        return False
    pats = values(argv, '--nets')
    return any(getattr(z, 'pad_connection', None) == 'thermal'
               and any(fnmatch.fnmatchcase(z.net_name, pat) for pat in pats)
               for z in getattr(p.board, 'zones', []) or [])


def r_no_teardrops_or_thermal(p):
    """SKILL.md :1044 -- do not recommend `--add-teardrops` (7% of human
    boards use them) and do not set `--thermal-relief`, except to re-pour a
    designer's thermal-relief pour and keep its relief (Step 3, #1329)."""
    bad = []
    for argv in p.argvs:
        for flag in ('--add-teardrops', '--thermal-relief'):
            if flag not in argv:
                continue
            if flag == '--thermal-relief' and _keeps_a_relief_pour(p, argv):
                continue
            bad.append(f'{tool_of(argv)} passes {flag}, which the skill '
                       f'says to leave alone'
                       + (' unless the pour it replaces is already a '
                          'thermal-relief pour (pass --board to show it)'
                          if flag == '--thermal-relief' else ''))
    return bad


def r_no_smoothing_off(p):
    """SKILL.md :1377 -- smoothing is ON by default and the corpus refuted
    turning it off (#536, then 68db5e3d reverting 97e4443)."""
    return [f'{tool_of(a)} passes --no-smoothing; the corpus refuted that'
            for a in p.argvs if '--no-smoothing' in a]


def r_no_max_iterations(p):
    """SKILL.md :1971 -- do not cap iterations on the route step."""
    return [f'{tool_of(a)} passes --max-iterations, which the skill says not '
            f'to set' for a in p.argvs if '--max-iterations' in a]


def r_max_ripup_within_bounds(p):
    """SKILL.md :2643 with :1971 -- 5 on dense boards, else the default 3,
    and "escalate above 5 only as a last resort on a specific failing net".

    Only the UPPER BOUND is decidable from a plan: the dense antecedent (a
    fine-pitch BGA, or >150 nets) is a fact about the board, and the negative
    arm is the flag's ABSENCE rather than an explicit 3 -- so neither can be
    graded here without manufacturing a false refusal.
    """
    bad = []
    for argv in p.argvs:
        v = scalar(argv, '--max-ripup')
        if v is not None and v > 5:
            bad.append(f'{tool_of(argv)} passes --max-ripup {v:g}; above 5 is '
                       f'a last resort for one failing net, never an opening '
                       f'move')
    return bad


def r_cp_carries_the_project(p):
    """SKILL.md :1462 -- never `cp` a board without its `.kicad_pro`. The
    sibling carries the DRC floor the chain routed to; stranding it makes the
    next step resolve a looser floor and stamp it over tighter copper.

    A PAIRING rule, not a prohibition: `cp a.kicad_pro b.kicad_pro` beside it
    is explicitly allowed, and `py_router/copy_board.py` is the recommended
    form.
    """
    bad = []
    copied_pro = set()
    for argv in p.argvs:
        if argv and argv[0] == 'cp' and len(argv) >= 3 \
                and argv[-1].endswith('.kicad_pro'):
            copied_pro.add(os.path.basename(argv[-1])[:-len('.kicad_pro')])
    for argv in p.argvs:
        if not argv or argv[0] != 'cp':
            continue
        pcbs = boards_in(argv, p.chain)
        if not pcbs:
            continue
        stem = os.path.basename(pcbs[-1])[:-len('.kicad_pcb')]
        if stem not in copied_pro:
            bad.append(f'`cp ... {os.path.basename(pcbs[-1])}` strands the '
                       f'sibling .kicad_pro (the DRC floor). Use '
                       f'py_router/copy_board.py, or copy the .kicad_pro too.')
    return bad


def r_one_cap_pass_after_fanout(p):
    """SKILL.md Step 1c -- a plan that fans out a BGA runs exactly one
    `place_fanout_clearance.py`, and it runs after every fanout step.

    The pass moves only 2-pad parts near a BGA's ball field
    (`fanout_clearance` builds its movers from `find_components_by_type(pcb,
    'BGA')`, `is_near_bga_cap`), so it is asked for by a `bga_fanout.py` step,
    or by a `qfn_fanout.py` step on a board that has a BGA (needs `--board`;
    without one that case is NOTED). On a board with no BGA it has nothing
    to move (#1710). A cap pass the plan runs anyway is still held to once,
    after every fanout: the pass is board-global and reads every via."""
    order = [tool_of(a) for a in p.argvs]
    caps = [i for i, t in enumerate(order) if t == 'place_fanout_clearance.py']
    fans = [i for i, t in enumerate(order)
            if t in ('bga_fanout.py', 'qfn_fanout.py')]
    if not fans:
        return []
    if len(caps) > 1:
        return [f'{len(caps)} place_fanout_clearance.py steps; the cap pass '
                f'runs ONCE, after all fanouts']
    if not caps:
        if 'bga_fanout.py' in order:
            return ['the plan fans out a BGA (bga_fanout.py) but never runs '
                    'place_fanout_clearance.py']
        if p.board is None:
            p.note('the plan fans out with qfn_fanout.py only and runs no '
                   'place_fanout_clearance.py, which moves only parts near a '
                   'BGA: pass --board to check whether this board has one')
            return []
        from kicad_parser import find_components_by_type
        bgas = sorted(fp.reference for fp in
                      find_components_by_type(p.board, 'BGA'))
        if bgas:
            return [f'the plan fans out but never runs '
                    f'place_fanout_clearance.py, and the board has a BGA '
                    f'({", ".join(bgas[:4])}{" ..." if len(bgas) > 4 else ""}) '
                    f'whose nearby parts it moves off fanout vias']
        return []
    if caps[0] < max(fans):
        return ['place_fanout_clearance.py runs before a later fanout step; '
                'it must come after all of them']
    return []


def r_first_pour_has_no_via_tail(p):
    """SKILL.md :976 -- the Step-1 `route_planes` call carries no
    `--add-gnd-vias` and no `--stitch-vias`: the bare pour comes first and the
    via tail is a later, high-speed-tier decision."""
    pours = p.by_tool('route_planes.py')
    if not pours:
        return []
    bad = []
    for flag in ('--add-gnd-vias', '--stitch-vias'):
        if flag in pours[0]:
            bad.append(f'the first route_planes step passes {flag}; the first '
                       f'pour is bare (#562)')
    return bad


def _globs(argv, flag):
    return set(values(argv, flag))


def _claims(argv, net):
    """Does this route step's `--nets` take `net`? The router's own order:
    the include patterns add, a `!` pattern removes (fnmatch either way)."""
    pats = values(argv, '--nets')
    if any(n.startswith('!') and fnmatch.fnmatchcase(net, n[1:])
           for n in pats):
        return False
    return any(not n.startswith('!') and fnmatch.fnmatchcase(net, n)
               for n in pats)


def r_net_coverage_reconciles(p):
    """SKILL.md Step 5b, the `assert`s of its step 2 -- which ship in a
    fenced block NO TEST RUNS, and whose violation is how `GNDA` ended a run
    at 0/23 pads connected while the run reported success.

      * a net a route step excludes (`!X`) is claimed by another stage:
        Step 2b's impedance pass, or a LATER route.py step whose `--nets`
        takes it (#1346). Step 8's pour-last flow is the case: the signal
        step runs `'*' '!GND'`, the pour comes after it, and the closing
        `'*'` step claims GND, whose in-run finalize welds the late pour.
        An EARLIER claim does not count: the last route step to see a net
        is the one whose finalize answers for it;
      * every Step-2b impedance net is excluded by a route step;
      * every poured net MUST appear in a route step's `--power-nets`.

    Compared as the LITERAL PATTERNS the plan passes, which is exact only for
    literal net names: `!+3V3*` against `+3V3` is a glob question and needs
    the board's net list. A pattern mismatch is therefore reported as one, in
    the words the skill uses, rather than silently resolved.
    """
    routes = [a for a in p.by_tool('route.py') if '--impedance' not in a]
    imped = [a for a in p.by_tool('route.py') if '--impedance' in a]
    pours = p.by_tool('route_planes.py')
    if not routes:
        return []
    exclusions, power = set(), set()
    for argv in routes:
        exclusions |= {n[1:] for n in _globs(argv, '--nets')
                       if n.startswith('!')}
        power |= _globs(argv, '--power-nets')
    impedance_se = set()
    for argv in imped:
        impedance_se |= {n for n in _globs(argv, '--nets')
                         if not n.startswith('!')}
    plane_nets = set()
    from plane_groups import plane_net_names
    for argv in pours:
        # #1495: a split layer's quoted '|' group pours each of its nets
        plane_nets |= set(plane_net_names(values(argv, '--nets')))

    bad = []
    orphans = set()
    for i, argv in enumerate(routes):
        for net in {n[1:] for n in _globs(argv, '--nets')
                    if n.startswith('!')} - impedance_se:
            if not any(_claims(later, net) for later in routes[i + 1:]):
                orphans.add(net)
    if orphans:
        bad.append(f'Net-coverage gap: {sorted(orphans)} handled by no stage '
                   f'-- a route step excludes it, no LATER route.py step '
                   f'claims it, and Step 2b routes {sorted(impedance_se)}')
    unexcluded = impedance_se - exclusions
    if unexcluded:
        bad.append(f'Step 2b routes {sorted(unexcluded)} but no route step '
                   f'excludes it -- the route steps exclude '
                   f'{sorted(exclusions)}')
    unsized = plane_nets - power
    if unsized:
        bad.append(f'Poured but no route-step width: {sorted(unsized)} -- '
                   f'that is where the finalize\'s taps and welds get their '
                   f'width')
    return bad


def r_gnd_via_distance(p):
    """Step 3 GND return vias -- `--gnd-via-distance` >= 3x (via + clearance).

    The size and the clearance are resolved from the PLAN, not from the one
    argv that carries the distance. This rule used to `continue` unless all
    three flags appeared together, and the step that sets the distance is a
    `route_planes.py` GND-via pass, which has no reason to restate a via size
    the earlier steps already fixed -- so the rule could not fire on the
    skill's own Step 3 command, which is precisely the command #941 row 2
    reports as recommending a distance below the floor.

    Resolution order: this argv, then the other steps RUN BY THE SAME TOOL,
    then what the step itself falls back to when it is given neither --
    route_planes' main reads the board's Default net class (`via_diameter`,
    `clearance`) and only then `routing_defaults` (#1709) -- so the `--board`'s
    Default class, then `routing_defaults`. This step's `--clearance-ceiling`
    then caps the clearance, as route_planes caps it (`min(base, ceiling)`).
    The reason names which it used, so a refusal resting on a default is not
    mistaken for one resting on the plan. Without `--board` a floor resting on
    `routing_defaults` is not what the step routes at (it reads the board), so
    it is NOTED, never refused. Not modelled: the fab-floor pin-up of a class
    below the physical floor, and a Default class an earlier step lowered in
    the project it writes back (the same-tool read covers a plan that says
    so).

    Scoped to the same tool, and taking the SMALLEST value there, because the
    vias whose spacing this grades are the ones THIS tool places. An earlier
    draft took `max()` across every command in the plan, on the theory that the
    widest via implies the floor that must hold for the whole board. That is
    wrong, and refuses correct plans: a coarse PGA escape via
    (`bga_fanout --via-size 0.8 --clearance 0.1`) beside a fine signal route
    (`route.py --via-size 0.25 --clearance 0.0889`) yields a resolved floor of
    2.70, while the GND-via pass that places the vias resolves 0.5/0.0889 for a
    real floor of 1.77 -- so a correct 2.0 is refused by a via the pass never
    places. A fanout escape via is not the via the GND pass places, and the
    skill itself has them at different sizes.
    """
    def _resolved(read, argv, board_key, fallback):
        # the OTHER steps: this one's own --clearance-ceiling caps what is
        # resolved here, and must not be read back as the base it caps
        tool = tool_of(argv)
        seen = [v for a in p.by_tool(tool) if a != argv
                for v in (read(a),) if v is not None]
        if seen:
            return min(seen), f'{tool} elsewhere in the plan'
        if p.board_path is not None:
            from list_nets import board_default_netclass_param
            got = board_default_netclass_param(p.board_path, board_key)
            if got is not None:
                return got, 'board Default class'
        return fallback, 'routing_defaults'

    def _via(argv):
        return scalar(argv, '--via-size')

    def _clearance(argv):
        # A chain's routing steps spell the floor `--clearance-ceiling` (Step
        # 4), so a plan that says only that must not read as saying nothing.
        given = [v for v in (scalar(argv, '--clearance'),
                             scalar(argv, '--clearance-ceiling'))
                 if v is not None]
        return min(given) if given else None

    bad = []
    for argv in p.argvs:
        d = scalar(argv, '--gnd-via-distance')
        if d is None:
            continue
        tool = tool_of(argv)
        vs, vs_src = _via(argv), 'this step'
        if vs is None:
            vs, vs_src = _resolved(_via, argv, 'via_diameter',
                                   routing_defaults.VIA_SIZE)
        clr, clr_src = scalar(argv, '--clearance'), 'this step'
        if clr is None:
            clr, clr_src = _resolved(_clearance, argv, 'clearance',
                                     routing_defaults.CLEARANCE)
        ceiling = scalar(argv, '--clearance-ceiling')
        if ceiling is not None and ceiling < clr:
            clr, clr_src = ceiling, 'this step'
        floor = 3.0 * (vs + clr)
        sizes = (f'3x(via {vs:g} [{vs_src}] + clearance {clr:g} '
                 f'[{clr_src}]) = {floor:.3f}')
        if p.board_path is None and 'routing_defaults' in (vs_src, clr_src):
            p.note(f'{tool} --gnd-via-distance {d:g} '
                   f'{"is below" if d < floor - 1e-9 else "clears"} {sizes}, '
                   f'but that rests on routing_defaults, and the step routes '
                   f"at the board's Default net class: pass --board to check "
                   f'it. Not refused on a default.')
        elif d < floor - 1e-9:
            bad.append(f'{tool_of(argv)} --gnd-via-distance {d:g} is below '
                       f'{sizes}')
    return bad


def r_impedance_needs_a_stackup(p):
    """SKILL.md :905 / :2600 -- with no real stackup, STATE THE NUMBERS and
    run no `--impedance` and no `--time-matching` pass. Do NOT author a
    stackup to make the numbers appear.

    Needs the board, so it is skipped (and said so) without --board.
    """
    if p.board is None:
        return []
    stack = getattr(p.board.board_info, 'stackup', None) or []
    if [s for s in stack if getattr(s, 'thickness', None)]:
        return []
    bad = []
    for argv in p.argvs:
        for flag in ('--impedance', '--time-matching'):
            if flag in argv:
                bad.append(f'{tool_of(argv)} passes {flag}, but the board '
                           f'declares no stackup thickness -- the numbers '
                           f'would be computed from a default that is not '
                           f'this board')
    return bad


def r_fanout_layers_exclude_planes(p):
    """Step 10 rule 3 -- fanout escape copper must stay off an INNER layer the
    plan pours a solid plane on, or the escape routes into the pour.

    The lever is `--layer-costs` (one value per `--layers` entry, negative =
    forbidden, #288), NOT a shorter `--layers`. Both forbid identically -- a
    negative entry is filtered out by the same `keep` list that a missing layer
    never joins -- so the preference is about derivation, not effect: the cost
    vector is what `route.py` takes too, a positive weight can price a layer
    rather than delete it, and `--layers` stays a statement of the stack. What
    matters HERE is that a poured layer carrying a negative cost is COMPLIANT,
    so this rule refuses only a poured layer the plan neither prices nor omits.

    Two things this rule must not demand, both measured against the engine:

      * `--layers[0]` cannot be forbidden -- `bga_fanout` raises "The top escape
        layer (...) cannot be forbidden - edge escapes are placed on it". A rule
        that refused it would demand something no plan can satisfy.
      * An OUTER-layer pour under a fanned part is sometimes the prescribed fix
        (Step 1: when an inner pour cannot thread the ball lattice even at the
        fab floor, the outer pour connects those pads by direct contact), and
        the route step's in-run finalize re-pours it. Refusing that would refuse
        the skill's own remedy.

    Hence: inner layers only, and only when unpriced.
    """
    plane_layers = set()
    for argv in p.by_tool('route_planes.py', 'repair_planes.py'):
        plane_layers |= set(values(argv, '--plane-layers'))
    if not plane_layers:
        return []
    bad = []
    for argv in p.by_tool('bga_fanout.py', 'qfn_fanout.py'):
        layers = values(argv, '--layers')
        if not layers:
            continue
        costs = values(argv, '--layer-costs')
        forbidden = set()
        if len(costs) == len(layers):
            for name, cost in zip(layers, costs):
                try:
                    if float(cost) < 0:
                        forbidden.add(name)
                except ValueError:
                    pass
        # layers[0] is the top escape layer: the engine refuses to forbid it,
        # so it is never this rule's to refuse either.
        clash = sorted((set(layers[1:]) & plane_layers) - forbidden)
        if clash:
            bad.append(f'{tool_of(argv)} --layers includes {clash}, which the '
                       f'plan pours a solid plane on, and --layer-costs does '
                       f'not forbid them (a negative cost per layer, #288)')
    return bad


def r_every_board_read_is_written_first(p):
    """Step 9 -- every board a step reads exists when the step runs (#1708).

    A board a step reads is a SEED (read before any step writes a board: the
    plan's input, or the source of a `copy_board.py` / `cp`) or the output of
    an EARLIER step. The step's own output path does not count: `route.py
    s1b.kicad_pcb s1b.kicad_pcb` with nothing writing `s1b` first has no board
    to read, and its tool refuses it ("input board ... does not exist"). Nor
    does a later step's. Run as written, such a plan cannot reach its final
    step; and when a later step reads a board that does exist, the file can
    still exit 0 with a final board the skipped step never touched.

    Inputs and outputs are read the way the replayer reads them
    (`redo_stress_test.board_tokens`: the last board token is the output),
    keyed by the path each resolves to from its command's cwd, so two
    folders' `s1.kicad_pcb` are two boards. A `mv` takes its source away.

    A judging chain cannot have the hole: judge_seeds binds each step's
    `{in}` to the board the step before it wrote, so the rule holds there by
    construction."""
    if p.chain:
        return []
    from redo_stress_test import (board_tokens, boards_read, is_check_cmd,
                                  split_env_prefix)
    here = os.path.dirname(os.path.abspath(p.path))

    def where(cwd, tok):
        return os.path.normpath(os.path.join(cwd or here,
                                             os.path.expanduser(tok)))

    written, seeds, bad = set(), set(), []
    for n, (cwd, argv) in enumerate(p.cmds, 1):
        argv = split_env_prefix(argv)[1]
        if is_check_cmd(argv):
            continue
        ins, out = board_tokens(argv)
        out = where(cwd, out) if out is not None else None
        named = {where(cwd, t): t for t in ins}
        for at in boards_read(list(named), out):
            if at in written or at in seeds:
                continue
            if not written:             # nothing written yet: a seed
                seeds.add(at)
                continue
            bad.append(f'{tool_of(argv) or argv[0]} (command {n} of '
                       f'{len(p.cmds)}) reads {named[at]}, which no earlier '
                       f'step writes'
                       + (' -- its own output path counts only once an '
                          'earlier step has written it' if at == out else '')
                       + '. Read the board the step before it wrote.')
        if argv[:1] == ['mv']:
            written -= set(named)
            seeds -= set(named)
        if out is not None:
            written.add(out)
    return bad


#: (id, what it checks, SKILL.md citation, fn). Order is the order printed.
RULES = (
    ('R01', 'the plan begins with an explicit cd', 'Step 9', r_starts_with_cd),
    ('R02', 'no pipes', 'Step 10 rule 5', r_no_pipes),
    ('R03', 'verification is commented, not executable', 'Step 9',
     r_verification_is_commented),
    ('R04', 'the chain ends on route.py (hand joins after it)', 'End every chain on route.py',
     r_ends_on_route),
    ('R05', 'the three verification commands are present', 'Important Notes 4',
     r_checkers_are_present),
    ('R06', 'diff-pair gap is not below clearance', 'Step 4 diff pairs',
     r_diff_gap_not_below_clearance),
    ('R07', 'no teardrops, no thermal relief', 'Important Notes',
     r_no_teardrops_or_thermal),
    ('R08', 'smoothing is left on', 'Octolinear smoothing is ON', r_no_smoothing_off),
    ('R09', 'iterations are not capped', 'Important Notes 15', r_no_max_iterations),
    ('R10', 'max-ripup stays within bounds', 'Step 10 rule 6',
     r_max_ripup_within_bounds),
    ('R11', 'a cp carries the .kicad_pro', 'Never cp a board without its .kicad_pro', r_cp_carries_the_project),
    # NOT 'after every fanout': that was the SKILL sentence this rule exists
    # to refuse, copied into the rule's own name (#941 row 3). The body has
    # always enforced "once, after the last".
    ('R12', 'one cap pass, after the last fanout', 'Step 1c',
     r_one_cap_pass_after_fanout),
    ('R13', 'the first pour is bare', 'Step 1 bare pour', r_first_pour_has_no_via_tail),
    ('R14', 'net coverage reconciles (Step 5b)', 'Step 5b',
     r_net_coverage_reconciles),
    ('R15', 'gnd-via distance clears 3x(via+clearance)',
     'Step 3 GND return vias',
     r_gnd_via_distance),
    ('R16', 'no impedance pass without a stackup [needs --board]',
     'Step 10 rule 1',
     r_impedance_needs_a_stackup),
    # NOT '[needs --board]': this rule reads only the plan's own argv, never
    # `p.board`, and `check()` SKIPS every rule whose text carries the marker.
    # The skill's own command line does pass --board, so the rule ran there;
    # what it did NOT run on is the bare `route_plan_check.py <plan>` form --
    # this tool's own first usage line -- and that is the form test_937's
    # harness uses, so the GATE never exercised R17 at all. Gating a rule on a
    # flag it has no use for buys nothing and costs exactly that.
    ('R17', 'fanout escapes stay off poured inner layers',
     'Step 10 rule 3',
     r_fanout_layers_exclude_planes),
    ('R18', 'every board a step reads is written before it', 'Step 9',
     r_every_board_read_is_written_first),
)


#: Rules a judging chain is not asked, and why (#1592).
NOT_ASKED_OF_A_CHAIN = {
    'R01': 'not asked of a judging chain: judge_seeds runs every step from '
           'the repo root',
}


def read_chain_commands(manifest, text):
    """[(None, argv)] for a judging chain, read as judge_seeds reads it, or
    None when the file is not one (no command line names a judge_seeds
    placeholder). A chain judge_seeds would refuse raises ValueError with
    its reason, which main() reports as exit 3."""
    import shlex
    from judge_seeds import chain_problems, is_chain, read_chain
    if not is_chain([ln for ln in text.splitlines()
                     if ln.strip() and not ln.lstrip().startswith('#')]):
        return None
    steps = read_chain(manifest)
    bad = chain_problems(steps)
    if bad:
        raise ValueError('the chain cannot judge a seed (judge_seeds would '
                         'refuse it): ' + '; '.join(bad))
    return [(None, shlex.split(s)) for s in steps]


def check(path, board_path=None):
    """(plan, failures, skipped) -- failures is [(id, what, cite, [reasons])],
    skipped [(id, what, cite, why)]."""
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    from make_plan import plan_commands, resolve_manifest
    manifest = resolve_manifest(path)
    with open(manifest, encoding='utf-8', errors='replace') as fh:
        text = fh.read()
    cmds = read_chain_commands(manifest, text)
    chain = cmds is not None
    if not chain:
        cmds = plan_commands(path)
    board = None
    if board_path:
        from kicad_parser import parse_kicad_pcb
        board = parse_kicad_pcb(board_path)
    plan = Plan(manifest, text, cmds, board, chain=chain,
                board_path=board_path or None)

    failures, skipped = [], []
    for rid, what, cite, fn in RULES:
        if board is None and 'needs --board' in what:
            skipped.append((rid, what, cite, 'pass --board to check it'))
            continue
        if chain and rid in NOT_ASKED_OF_A_CHAIN:
            skipped.append((rid, what, cite, NOT_ASKED_OF_A_CHAIN[rid]))
            continue
        plan._noting = rid
        reasons = fn(plan)
        if reasons:
            failures.append((rid, what, cite, reasons))
    return plan, failures, skipped


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('plan', nargs='?',
                    help='the <board>_plan.sh / redo_commands.sh, or the run '
                         'directory holding one')
    ap.add_argument('--board', default=None,
                    help='the board the plan is about; enables the rules '
                         'whose antecedent is a fact about the board')
    ap.add_argument('--list', action='store_true',
                    help='every rule and every DELEGATED check, then exit')
    a = ap.parse_args(argv)

    if a.list:
        print('rules checked here:')
        for rid, what, cite, _fn in RULES:
            print(f'  {rid}  {what:<52} SKILL.md, {cite}')
        print('\nNOT checked here -- a plan cannot answer these; the tool '
              'that can:')
        for what, who in DELEGATED:
            print(f'       {what:<52} {who}')
        return CLEAN
    if not a.plan:
        ap.print_help()
        return USAGE

    try:
        plan, failures, skipped = check(a.plan, a.board)
    except FileNotFoundError as exc:
        print(f'route_plan_check: {exc}', file=sys.stderr)
        return UNREADABLE
    except ValueError as exc:               # an unterminated quote, say
        print(f'route_plan_check: {plan_err(exc)}', file=sys.stderr)
        return UNREADABLE

    try:
        shown = os.path.relpath(plan.path)
    except ValueError:      # another Windows drive: no relative path exists
        shown = plan.path
    print(f'{shown}: '
          + ('a judging chain ({in}/{out}, the judge_seeds format): '
             if plan.chain else '')
          + f'{len(plan.cmds)} {"" if plan.chain else "recorded "}'
          f'command(s), {len(plan.mutating)} of them board-mutating')
    for rid, what, cite, _fn in RULES:
        hit = next((f for f in failures if f[0] == rid), None)
        skip = next((s for s in skipped if s[0] == rid), None)
        notes = plan.notes.get(rid, [])
        if skip:
            print(f'  SKIP  {rid}  {what}  -- {skip[3]}')
        elif hit:
            print(f'  FAIL  {rid}  {what}   [SKILL.md, {cite}]')
            for reason in hit[3]:
                print(f'          {reason}')
        elif notes:
            print(f'  NOTE  {rid}  {what}')
        else:
            print(f'  ok    {rid}  {what}')
        for text in notes if not skip else ():
            print(f'          note: {text}')

    print('\nNOT checked here -- a recorded plan cannot answer these:')
    for what, who in DELEGATED:
        print(f'       {what}  ->  {who}')

    if failures:
        print(f'\nREFUSED: {len(failures)} rule(s) failed. Fix the plan; do '
              f'not run it.')
        return REFUSED
    print(f'\nOK: {len(RULES) - len(skipped)} rule(s) checked, '
          f'{len(skipped)} skipped, {len(DELEGATED)} delegated.')
    return CLEAN


def plan_err(exc):
    return f'the plan could not be read: {exc}'


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(CRASH)
