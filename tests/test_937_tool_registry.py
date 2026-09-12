#!/usr/bin/env python3
"""#937: every runnable tool is in the catalogue, and the catalogue is gated.

This is the anti-deletion guarantee. A restructure cannot quietly drop a tool,
because the completeness case below fails when a runnable tool has no
`KRT_TOOL` declaration -- and it fails on the tool that was dropped, by name.

Before this gate the repo had three catalogues and no completeness check in
the direction that matters. `krt_capabilities.KNOWN_MODULES` is a
hand-maintained tuple of 29 (one member, `route_summary.py`, is not a CLI at
all); `FLAG_SCRIPTS` covers 8; `test_431.discovered_tools()` finds the 49 the
skills happen to invoke. Every one of them asks "does this named thing
exist?"; none asks "is every thing that exists named?".

THE FLOORS ARE PER DOOR, AND THAT IS THE WHOLE POINT. A registry that is
complete overall but omits every routing tool from the routing view passes any
global count, and the reader at the routing door is exactly as stranded as
before. `tests/mutate_937.py` proves the floors bind by setting every row's
scope to `['placement']`: the routing and combined floors must redden while
the total does not move.

Floors, not pins, and deliberately well under today's numbers: their job is to
catch the PREDICATE going dark. A sweep that suddenly finds nothing would
otherwise report "all 0 tools declared" as a pass, which is this repo's
recorded vacuity failure and the reason `test_803` carries `MIN_RESOLVED` and
`test_718` carries `_BATTERY_COUNT`. The exact-membership job belongs to the
completeness case, which needs no number.
"""
import ast
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import krt_registry as R                                     # noqa: E402

#: The probe launches one subprocess per tracked non-test .py (240 of them,
#: ~11 s at 8 workers) and the registry is built once and shared. 300 gives
#: the same headroom test_798 takes for a far smaller sweep.
RUN_ALL_TIMEOUT = 300

#: Measured at the commit that added this gate: 71 tools, 31 placement,
#: 34 routing, 31 combined. These are FLOORS well under those, not targets --
#: see the module docstring. Raising one is a deliberate decision, here.
_MIN_TOOLS = 60
_MIN_PER_DOOR = {'placement': 25, 'routing': 28, 'combined': 25}

#: Built once: every case below reads it, and rebuilding per case would pay
#: the 11 s probe five times over.
ROWS = R.registry(ROOT)


def t_every_runnable_tool_is_declared():
    """The completeness direction, and the reason this file exists."""
    undeclared = sorted(r['path'] for r in ROWS if not r['declared'])
    assert not undeclared, (
        f'{len(undeclared)} runnable tool(s) carry no KRT_TOOL declaration, '
        f'so no door view can show them and a restructure could drop them '
        f'unnoticed: {undeclared}')
    print(f'  PASS: {len(ROWS)} runnable tool(s), all declared')


def t_no_declaration_invents_its_own_vocabulary():
    """A typo'd door is a tool that silently appears in NO view, which is the
    failure this registry exists to prevent -- so it is a hard error, not a
    warning."""
    bad_kind = sorted(r['path'] for r in ROWS if r['kind'] not in R.KINDS)
    assert not bad_kind, f'kind not one of {R.KINDS}: {bad_kind}'
    bad_scope = sorted(
        r['path'] for r in ROWS
        if r['scope'] is None or any(s not in R.DOORS for s in r['scope']))
    assert not bad_scope, f'scope not a list of {R.DOORS}: {bad_scope}'
    print(f'  PASS: every kind in {R.KINDS}, every scope within {R.DOORS}')


def t_no_declaration_outlives_the_tool_it_describes():
    """The REVERSE direction. A file that stops answering `--help` -- a CLI
    demoted to a library, say -- keeps its declaration and would go on being
    counted in a door's floor while no reader can run it. Nothing else in the
    repo would notice."""
    runnable = {r['path'] for r in ROWS}
    stale = []
    for rel in R.tracked_python(ROOT):
        if rel in runnable:
            continue
        if R.declaration(ROOT, rel) is not None:
            stale.append(rel)
    assert not stale, (
        f'{len(stale)} file(s) declare KRT_TOOL but no longer answer --help '
        f'with a usage line, so they are counted by the registry and runnable '
        f'by nobody: {sorted(stale)}')
    print(f'  PASS: no declaration outlives its tool')


def t_each_door_view_is_populated():
    """The anti-vacuity floor, PER DOOR. See the module docstring."""
    assert len(ROWS) >= _MIN_TOOLS, (
        f'only {len(ROWS)} runnable tools found (floor {_MIN_TOOLS}) -- the '
        f'--help probe has probably stopped working, and a probe that finds '
        f'nothing reports every door as complete')
    counts = {}
    for door in R.DOORS:
        counts[door] = len(R.door_view(ROWS, door))
        assert counts[door] >= _MIN_PER_DOOR[door], (
            f'the {door} door shows {counts[door]} tool(s), floor '
            f'{_MIN_PER_DOOR[door]}. A registry complete overall but empty at '
            f'one door leaves that reader exactly as stranded as before.')
    scopeless = sum(1 for r in ROWS if not r['scope'])
    print(f'  PASS: {len(ROWS)} tools; ' +
          ', '.join(f'{d} {counts[d]}' for d in R.DOORS) +
          f'; {scopeless} declared into no door')


def t_every_self_recorder_is_declared_an_actor():
    """The cross-check, and it is a CROSS-CHECK: `record_invocation` is the
    precise half of the derivation (17 call sites, 17 actors) but it has
    misses, so a disagreement is reported by name rather than used to
    overwrite the declaration."""
    wrong = sorted(r['path'] for r in ROWS if r['self_records']
                   and r['kind'] not in ('actor', 'conditional'))
    assert not wrong, (
        f'{len(wrong)} tool(s) call record_invocation -- they write a board '
        f'and self-record it -- but are declared neither actor nor '
        f'conditional: {wrong}')
    n = sum(1 for r in ROWS if r['self_records'])
    print(f'  PASS: {n} self-recorder(s), all declared actor or conditional')


def t_every_tool_carries_its_own_purpose_text():
    """`purpose` is DERIVED from the docstring rather than copied into the
    registry, which is only sound while every tool has one."""
    mute = sorted(r['path'] for r in ROWS if not r['purpose'])
    assert not mute, (
        f'{len(mute)} tool(s) have no module docstring, so the registry has '
        f'no purpose text to derive and would need a second copy that can '
        f'drift: {mute}')
    shortest = min(ROWS, key=lambda r: len(r['purpose']))
    print(f'  PASS: all {len(ROWS)} carry purpose text; shortest is '
          f'{len(shortest["purpose"])} chars ({os.path.basename(shortest["path"])})')


def t_scope_is_still_not_derivable():
    """Re-measure the claim the `scope` field rests on, every run.

    `scope` is declared because the two candidate derivations disagree and
    neither is complete. That is a fact about the tree, not a law, and a
    declared field nobody can retire is how a registry becomes make-work -- so
    if the directory rule and the usage rule ever BOTH decide for every tool
    AND agree, this fails and says to delete the field.

    The two rules measure different questions, which is why they differ: a
    directory is a PROVENANCE fact (the #522 reorg moved these files) and a
    skill naming a tool is a USAGE fact.
    """
    skill_door = {'plan-pcb-placement': 'placement',
                  'plan-pcb-placement-and-routing': 'combined'}
    texts = {}
    r = subprocess.run(['git', 'ls-files', '.claude/skills'],
                       cwd=ROOT, capture_output=True, text=True)
    for rel in r.stdout.split():
        if not rel.endswith('.md'):
            continue
        with open(os.path.join(ROOT, rel), encoding='utf-8',
                  errors='replace') as fh:
            texts[rel] = fh.read()

    undecided_dir = undecided_use = disagree = 0
    for row in ROWS:
        top = row['path'].split('/')[0]
        by_dir = {'py_placer': {'placement'},
                  'py_router': {'routing'}}.get(top)
        by_use = set()
        pat = re.compile(r'(?<![A-Za-z0-9_])'
                         + re.escape(os.path.basename(row['path'])))
        for rel, text in texts.items():
            if pat.search(text):
                by_use.add(skill_door.get(rel.split('/')[2], 'routing'))
        if by_dir is None:
            undecided_dir += 1
        if not by_use:
            undecided_use += 1
        if by_dir and by_use and by_dir != by_use:
            disagree += 1

    assert undecided_dir or undecided_use or disagree, (
        'the directory rule and the usage rule now decide for every tool and '
        'agree everywhere. `scope` has become derivable -- retire the '
        'declared field rather than going on maintaining it.')
    print(f'  PASS: scope stays declared -- directory rule undecided for '
          f'{undecided_dir}, usage rule empty for {undecided_use}, '
          f'{disagree} disagreement(s)')


def t_the_predicate_covers_everything_test_431_discovers():
    """No coverage is lost by adopting this catalogue: it must be a superset
    of what the skills-scanning gate already finds."""
    sys.path.insert(0, os.path.join(ROOT, 'tests'))
    import test_431_skill_commands as t431              # noqa: E402
    discovered = {os.path.basename(t) for t in t431.TOOLS}
    ours = {os.path.basename(r['path']) for r in ROWS}
    missing = sorted(discovered - ours)
    assert not missing, (
        f'test_431 invokes {len(missing)} tool(s) this registry does not '
        f'list, so adopting it would LOSE coverage: {missing}')
    print(f'  PASS: all {len(discovered)} tools test_431 discovers are in the '
          f'registry ({len(ours) - len(discovered)} more besides)')


TESTS = (t_every_runnable_tool_is_declared,
         t_no_declaration_invents_its_own_vocabulary,
         t_no_declaration_outlives_the_tool_it_describes,
         t_each_door_view_is_populated,
         t_every_self_recorder_is_declared_an_actor,
         t_every_tool_carries_its_own_purpose_text,
         t_scope_is_still_not_derivable,
         t_the_predicate_covers_everything_test_431_discovers)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(f'--- {fn.__name__}')
        fn()
    print('\nALL PASS')
