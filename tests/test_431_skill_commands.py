"""Every flag the skill and docs tell Claude to pass must actually exist (#431).

Most of a skill is prose and untestable. This is the part that ROTS: a doc
telling Claude to pass a flag that was renamed or never existed produces a
confident, wrong command, and nothing catches it until a user runs it.

`tests/run_doc_examples.py` reads ```python blocks from `docs/*.md` only -- not
`.claude/skills/`, and not bash blocks -- so it cannot cover this. Precedent for
the doc-vs-code gate: `run_doc_examples.gridrouteconfig_undocumented_fields` and
`tests/gui_parity/test_cli_postpass_coverage.py`.

Explicitly NOT testable, and worth saying rather than pretending: whether Claude
*decides correctly* what to do with a given placement. The mitigations are
design, not assertion -- a MANDATORY copper-free measurement whose two outcomes
are both legitimate, the decision table it feeds, the driver that refuses to
emit a repair stage without that measurement, and the board-state gates that
refuse the worst case outright.

Note the design change (run 8): this file used to assert the skill said
placement was "normally SKIPPED". That was the wrong invariant. A default of
SKIP is satisfied most cheaply by skipping, and the thing being skipped is the
check that catches stacked parts.
"""

import importlib.util
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Files that instruct Claude or a human to run these tools.
# The skill was split into three (run-8 S2): placement, routing, and the thin
# combined one that sequences them. A flag can now live in any of them, and the
# whole point of this gate is that NO skill file drifts from the real parsers --
# so every one is a source, and the assertions below say which file must carry
# which rule.
def source_text(rel):
    """What the executor actually reads for this source.

    For a DRIVER that is its --dump-all output, not its Python source. The
    source splits one emitted command across several string literals, so a
    source scan sees `--json` at the end of a line whose value is on the next
    one and reports a defect that does not exist -- while missing that the
    emitted text is what the executor runs. Ask the driver.
    """
    path = os.path.join(ROOT, rel)
    if not os.path.isfile(path):
        return ''
    if rel.endswith('_driver.py'):
        env = dict(os.environ, COLUMNS='200', KRT_NO_BANNER='1')
        p = subprocess.run([sys.executable, '-X', 'utf8', path, '--dump-all'],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT, timeout=300, env=env)
        out = (p.stdout or '') + (p.stderr or '')
        assert '=====' in out, f'{rel} --dump-all emitted nothing:\n{out[:400]}'
        assert p.returncode == 0, \
            f'{rel} --dump-all exited {p.returncode}; a stage refused:\n{out[-600:]}'
        return out
    return open(path, encoding='utf-8', errors='replace').read()


def _all_skill_text():
    """Every skill file as one string: a rule may live in any of the three."""
    out = []
    for rel in SOURCES:
        path = os.path.join(ROOT, rel)
        if os.path.isfile(path):
            out.append(open(path, encoding='utf-8').read())
    return '\n'.join(out)


SOURCES = [
    '.claude/skills/plan-pcb-routing/SKILL.md',
    '.claude/skills/plan-pcb-placement/SKILL.md',
    '.claude/skills/plan-pcb-placement-and-routing/SKILL.md',
    # The skill's reference pages carry command blocks too (#549). Without them
    # every block moved out of SKILL.md becomes flag-unchecked, which is the
    # quiet way this gate stops gating.
    '.claude/skills/plan-pcb-placement-and-routing/references/evidence-map.md',
    '.claude/skills/plan-pcb-placement-and-routing/references/verifier-prompts.md',
    '.claude/skills/plan-pcb-placement-and-routing/references/convergence.md',
    # ...and so does the DRIVER that now emits the workflow. This is the same
    # hole one level down, and it opened exactly as the comment above predicts:
    # the stage bodies moved out of SKILL.md into scripts/*.py, the gate kept
    # scanning only .md, and it went on reporting "all flag citations real"
    # while the drivers emitted SEVEN commands that die at argparse. A command
    # the executor is told to run is a command this gate must check, whatever
    # file it is spelled in.
    '.claude/skills/plan-pcb-placement/scripts/placement_driver.py',
    '.claude/skills/plan-pcb-placement-and-routing/scripts/loop_driver.py',
    'docs/floorplan-intent.md',
    'docs/placement-optimization.md',
    'docs/claude-skills.md',
    'py_placer/placement/README.md',
    'README.md',
]

# Tools are DISCOVERED from what the sources actually invoke, never listed by
# hand. The hand-written list had six entries while the skills invoked twenty:
# route.py, route_diff.py, route_planes.py, check_drc.py, check_reachability.py
# and the rest were cited constantly and checked nowhere. A list someone has to
# remember to extend is a list that silently stops covering things.
# Only an INVOKED tool: something a `python3 ...` line runs. Matching bare
# `foo.py` anywhere in the text sweeps in every library module the prose names
# (routing_defaults.py, obstacle_map.py, ...), which have no CLI at all -- and a
# gate that reports 40 unparseable libraries is a gate nobody reads.
_TOOL_RE = re.compile(
    r'\bpython[0-9]*(?:\s+-[A-Za-z]\S*(?:\s+\S+)?)*\s+'
    r'((?:[\w./-]+/)?[a-z][a-z0-9_]*\.py)')


def discovered_tools():
    """Every repo tool the skills tell the executor to run."""
    found = set()
    for rel in SOURCES:
        path = os.path.join(ROOT, rel)
        if not os.path.isfile(path):
            continue
        text = open(path, encoding='utf-8', errors='replace').read()
        for m in _TOOL_RE.finditer(text):
            name = m.group(1).replace('\\', '/')
            # A test is not a tool: it has no flag contract for the executor,
            # and resolving its "parser" means EXECUTING it inside this gate.
            if name.startswith(('tests/', 'wk/')):
                continue
            if os.path.isfile(os.path.join(ROOT, name)):
                found.add(name)
    return tuple(sorted(found))


TOOLS = discovered_tools()

# Flags that belong to a DIFFERENT tool on the same command line (a pipe, a
# --route-args payload). --route-args carries route.py's flags verbatim.
_ROUTE_ARGS_RE = re.compile(r"--route-args\s+(['\"])(.*?)\1", re.S)


def _flags_from_help(tool):
    """Option strings straight out of `tool --help`.

    Needed because a good half of these tools build their parser inside the
    `if __name__ == '__main__'` block, where importing the module cannot reach
    it. --help is the same contract the executor sees, which makes it the right
    authority anyway. Wide COLUMNS so argparse does not wrap a long flag onto
    two lines and hide it from the regex.
    """
    def ask(*argv):
        env = dict(os.environ, COLUMNS='200', KRT_NO_BANNER='1')
        p = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, tool), *argv, '--help'],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT, timeout=180, env=env)
        return (p.stdout or '') + (p.stderr or ''), p.returncode

    text, rc = ask()
    if '--help' not in text:
        raise RuntimeError(f'--help produced no option list (exit {rc})')
    flags = set(re.findall(r'(--[a-z][a-z0-9-]+)', text))
    # A SUBCOMMAND tool keeps its real flags on the subparsers, and top-level
    # --help never lists them. Reading only the top level reports every
    # subcommand flag as nonexistent, which is a wall of false failures that
    # buries the true ones. argparse prints the choices as {a,b,c}.
    for m in re.finditer(r'\{([a-z0-9_][a-z0-9_,-]*)\}', text):
        for sub in m.group(1).split(','):
            sub_text, _ = ask(sub)
            if '--help' in sub_text:
                flags |= set(re.findall(r'(--[a-z][a-z0-9-]+)', sub_text))
    return flags


def _parser_for(tool):
    """Build the tool's real argparse parser and return its option strings."""
    path = os.path.join(ROOT, tool)
    # basename, not the whole entry: a TOOLS entry may be a PATH (skill-local
    # helpers live under .claude/skills/...), and a module name carrying path
    # separators is legal but confusing in tracebacks.
    spec = importlib.util.spec_from_file_location(
        os.path.basename(tool)[:-3] + '_probe', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    if hasattr(mod, 'build_parser'):
        p = mod.build_parser()
    else:
        # place_optimize / place_route_loop build the parser inside main(); run
        # it with --help intercepted so we get the parser without executing.
        import argparse
        got = {}
        real_parse = argparse.ArgumentParser.parse_args

        def capture(self, *a, **kw):
            got['p'] = self
            raise SystemExit(0)
        argparse.ArgumentParser.parse_args = capture
        try:
            mod.main()
        except SystemExit:
            pass
        finally:
            argparse.ArgumentParser.parse_args = real_parse
        p = got.get('p')
        assert p is not None, f"could not capture {tool}'s parser"
    return _option_strings(p)


def _option_strings(parser):
    """Every option a parser accepts, subcommands included.

    A subparsers action carries no option_strings of its own, so reading only
    the top level of a verb-style tool reports every one of its real flags as
    nonexistent -- dozens of false failures with the true ones buried in them.
    """
    import argparse
    out = {s for a in parser._actions for s in a.option_strings}
    for a in parser._actions:
        if isinstance(a, argparse._SubParsersAction):
            for sub in a.choices.values():
                out |= _option_strings(sub)
    return out


def _strip_flag_payloads(block):
    """Drop quoted spans that are a flag's VALUE; keep ordinary quoted args.

    `--lever 'rip lever: --rip-existing-nets X + --grid-step 0.025'` describes
    a step in prose, and its contents are not this tool's flags. But
    `--nets "*" "!GND"` is an ordinary quoted argument, and dropping it makes
    --nets look like a flag given no value. So strip a quoted span only when it
    carries flags of its own, and keep it when it invokes something: --argv
    "python3 route.py --nets ..." really is a command worth checking.
    """
    def repl(m):
        inner = m.group(2)
        if '.py' in inner:
            return m.group(0)
        return ' ' if '--' in inner else m.group(0)
    return re.sub(r"""(['"])(.*?)\1""", repl, block, flags=re.S)


def _cited_flags(block, tool):
    """Every flag in one whole shell command that invokes `tool`.

    Scans the ENTIRE block, continuation lines included. Filtering to lines
    containing the tool name (the obvious first cut) reads only the first line
    of a backslash-continued command and silently checks almost nothing -- this
    gate found 5 flags that way instead of 20.

    A flag belongs to the LAST tool named before it, which is how a shell reads
    it. Handing every flag on the line to every tool on the line is what made
    `converge.py record --kind route --argv "route.py --nets ..."` report
    --kind and --argv as route.py flags: real-looking failures against a tool
    that never saw them, mixed in with the true ones.
    """
    # strip --route-args payloads: those are route.py's flags, not this tool's
    block = _ROUTE_ARGS_RE.sub(' ', block)
    # A quoted VALUE is prose, not a command line: `--lever 'rip lever:
    # --rip-existing-nets X + --grid-step 0.025'` describes what a step did.
    # Keep quoted spans that invoke something, because those (--argv
    # "python3 route.py --nets ...") really are commands worth checking.
    block = _strip_flag_payloads(block)
    # In prose, a command lives inside backticks and the sentence around it
    # talks about OTHER tools by name ("`converge.py where ...` -- pass
    # `--summary-json` on the render"). Scanning the whole line hands the
    # render's flags to converge. One span, one command.
    spans = re.findall(r'`([^`]+)`', block) or [block]

    out = set()
    for span in spans:
        current = None
        for tok in re.split(r'\s+', span):
            bare = tok.strip('\'"`(),;').replace('\\', '/')
            if bare.endswith('.py'):
                current = next((t for t in TOOLS
                                if bare == t or bare.endswith('/' + t)
                                or os.path.basename(t) == os.path.basename(bare)),
                               current)
                continue
            m = re.match(r'(--[a-z][a-z0-9-]+)', bare)
            if m and current == tool:
                out.add(m.group(1))
    return out


def _continued_blocks(text, tool):
    """Whole shell commands (handling trailing backslashes) that run `tool`."""
    blocks, cur = [], None
    for line in text.splitlines():
        if cur is not None:
            cur.append(line)
            if not line.rstrip().endswith('\\'):
                blocks.append('\n'.join(cur))
                cur = None
            continue
        if tool in line and not line.lstrip().startswith('#'):
            cur = [line]
            if not line.rstrip().endswith('\\'):
                blocks.append('\n'.join(cur))
                cur = None
    return blocks


DRIVERS = tuple(s for s in SOURCES if s.endswith('_driver.py'))


def _tool_spans(block, tool):
    """Token runs that belong to `tool`: from its name to the next tool named."""
    block = _ROUTE_ARGS_RE.sub(' ', block)
    block = _strip_flag_payloads(block)
    spans, cur = [], None
    for tok in re.split(r'\s+', block):
        bare = tok.strip('\'"`(),;').replace('\\', '/')
        if bare.endswith('.py'):
            hit = next((t for t in TOOLS
                        if bare == t or bare.endswith('/' + t)
                        or os.path.basename(t) == os.path.basename(bare)), None)
            if cur is not None:
                spans.append(cur)
                cur = None
            if hit == tool:
                cur = []
            continue
        if cur is not None and bare:
            cur.append(bare)
    if cur is not None:
        spans.append(cur)
    return spans


def _parser_obj(tool):
    """The parser itself, for metadata a flag-name set cannot carry."""
    path = os.path.join(ROOT, tool)
    spec = importlib.util.spec_from_file_location(
        os.path.basename(tool)[:-3] + '_meta', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    if hasattr(mod, 'build_parser'):
        return mod.build_parser()
    import argparse
    got = {}
    real = argparse.ArgumentParser.parse_args

    def capture(self, *a, **kw):
        got['p'] = self
        raise SystemExit(0)
    argparse.ArgumentParser.parse_args = capture
    try:
        mod.main()
    except SystemExit:
        pass
    finally:
        argparse.ArgumentParser.parse_args = real
    assert got.get('p') is not None, f'no parser for {tool}'
    return got['p']


def test_driver_commands_supply_required_options_and_values():
    """A flag that EXISTS can still make the command die at argparse.

    The flag-name check above cannot see two failures that stop the executor
    just as hard, and both shipped:

      * a REQUIRED option left out entirely -- `route_planes.py` without
        `--plane-layers` exits 2 before it opens the board;
      * a flag that takes a value, given none -- `board_score.py --json` at the
        end of a line is `expected one argument`.

    Scoped to the drivers, because a driver emission is a command the executor
    is told to run verbatim. Prose may legitimately show a fragment.
    """
    problems = []
    checked = 0
    for src in DRIVERS:
        text = source_text(src)
        for tool in TOOLS:
            spans = [s for b in _continued_blocks(text, tool)
                     for s in _tool_spans(b, tool)]
            if not spans:
                continue
            try:
                parser = _parser_obj(tool)
            except Exception:
                continue          # covered by the flag test's own <parser> row
            import argparse as _ap
            # store_true/store_false/count/help consume nothing; every other
            # action stores a value and argparse errors without one -- EXCEPT
            # when its nargs makes zero values legal. `nargs='*'` and `'?'`
            # both accept none, so `--reseat --clearance 0.2` parses to
            # `{'reseat': [], 'clearance': 0.2}` and is the documented calling
            # convention ("bare --reseat = auto scope"). Without this the check
            # reported a correct, working command as dying at argparse, and the
            # false positive outlived several attempts to fix the driver that
            # was never wrong. `nargs='+'` and a fixed count still need values.
            def _optional_value(a):
                return getattr(a, 'nargs', None) in ('*', '?', 0)
            takes_value = {s: (not isinstance(a, (_ap._StoreTrueAction,
                                                  _ap._StoreFalseAction,
                                                  _ap._StoreConstAction,
                                                  _ap._CountAction,
                                                  _ap._HelpAction))
                               and not _optional_value(a))
                           for a in parser._actions for s in a.option_strings}
            required = [tuple(a.option_strings) for a in parser._actions
                        if getattr(a, 'required', False) and a.option_strings]
            for span in spans:
                checked += 1
                for opts in required:
                    if not any(o in span for o in opts):
                        problems.append(
                            (src, tool, f'required {"/".join(opts)} not passed'))
                for i, tok in enumerate(span):
                    if not takes_value.get(tok):
                        continue
                    nxt = span[i + 1] if i + 1 < len(span) else None
                    if nxt is None or nxt.startswith('--'):
                        problems.append(
                            (src, tool, f'{tok} takes a value, none given'))
    assert not problems, (
        'driver commands that die at argparse:\n'
        + '\n'.join(f'  {s}:  {t}  {w}' for s, t, w in sorted(set(problems))))
    assert checked >= 10, f'only {checked} driver command(s) scanned'
    print(f'  PASS: {checked} driver command spans, all runnable')


def test_every_documented_flag_exists():
    problems = []
    checked = 0
    for tool in TOOLS:
        # Collect the citations FIRST, and only then go looking for a parser.
        # A tool invoked without flags has no contract to check, and resolving
        # a parser for it is how a useful failure list fills up with noise
        # about modules nobody passed a flag to.
        cites = []
        for src in SOURCES:
            text = source_text(src)
            for block in _continued_blocks(text, tool):
                for flag in _cited_flags(block, tool):
                    cites.append((src, flag))
        if not cites:
            continue
        try:
            valid = _parser_for(tool)
        except Exception:
            # The import path cannot reach a parser built under
            # `if __name__ == '__main__'`. Ask the tool the way the executor
            # does before calling it unparseable.
            try:
                valid = _flags_from_help(tool)
            except Exception as e:
                problems.append((tool, '<parser>', f"{type(e).__name__}: {e}"))
                continue
        for src, flag in cites:
            checked += 1
            if flag not in valid:
                problems.append((tool, src, flag))
    assert not problems, "documented flags that do not exist:\n" + "\n".join(
        f"  {t}  in {s}:  {f}" for t, s, f in problems)
    # A gate that checks nothing passes for the wrong reason. The docs cite well
    # over a dozen flags across these tools; if this trips, the block/flag
    # scanner stopped matching rather than the docs becoming clean.
    assert checked >= 15, f"only {checked} flag citations found -- scanner broken?"
    print(f"  PASS: {checked} flag citations, all real")


def test_the_placement_tools_are_actually_mentioned():
    """Guards the reverse failure: the gate passing because the skill stopped
    mentioning placement at all."""
    skill = _all_skill_text()
    for token in ('place_optimize.py', 'render_placement.py', '--suggest-locks',
                  'Step 0'):
        assert token in skill, f"{token} missing from the skill"


def test_exit_code_contract_is_documented():
    """The skill tells Claude to branch on exit 3. If the constant moves and the
    docs do not, the instruction silently becomes wrong."""
    from placement.placement_state import UNPLACED_EXIT
    assert UNPLACED_EXIT == 3
    skill = _all_skill_text()
    assert 'exit 3' in skill or 'exits 3' in skill, \
        "the skill must state the exit-3 contract it tells Claude to rely on"


def test_skill_decides_placement_by_measurement_not_by_default():
    """The single most important thing for a model to get right here.

    This used to assert the skill said placement was "normally SKIPPED", and
    that was the wrong invariant to pin. A default of SKIP is what lets an
    executor route a board whose parts are stacked on each other: the cheapest
    way to satisfy "usually skip" is to skip, and the check that would have
    caught the damage is the thing being skipped.

    The rule is measure-then-decide. The measurement is two commands on the
    copper-free board, it is never optional, and BOTH outcomes are legitimate:
    clean means route (a verdict, not an assumption), dirty means fix. The
    reason not to optimise a clean placement survives -- as a consequence of
    the measurement rather than as a reason to skip it.
    """
    skill = _all_skill_text()
    # The gate is a measurement, and it is mandatory.
    assert 'measure first, then decide' in skill.lower()
    assert 'never optional' in skill.lower() or 'NEVER skip the assessment' in skill
    # Both instruments, because neither alone sees a same-net stack.
    assert 'check_drc' in skill and 'check_assembly' in skill
    assert 'copper-free' in skill.lower() or 'COPPER-FREE' in skill
    # The measured reason a CLEAN placement is left alone must survive.
    assert 'WORSE by a polish pass' in skill or 'makes it worse' in skill
    assert 'decision table' in skill.lower()
    # and that the render is not mistaken for the verdict (#431 limit 3)
    assert 'triage, not a verdict' in skill


def test_routing_only_stays_the_default_path():
    """#549. Placement must stay reachable only through a board-state branch or
    a post-failure branch, never on the path of "here is a board, route it".

    The structural guarantee is that placement cannot enter a plan at all. It is
    load-bearing rather than tidy: ai_plan DROPS an unknown action with a
    one-line note and RUNS THE REMAINING STEPS ANYWAY, so a `{"action":"place"}`
    step would silently route an unplaced board.
    """
    sys.path.insert(0, os.path.join(ROOT, 'kicad_routing_plugin'))
    sys.path.insert(0, os.path.join(os.path.join(ROOT, 'kicad_routing_plugin'), 'py_placer'))  # placement split
    sys.path.insert(0, os.path.join(os.path.join(ROOT, 'kicad_routing_plugin'), 'py_router'))  # placement split
    sys.path.insert(0, os.path.join(os.path.join(ROOT, 'kicad_routing_plugin'), 'py_tools'))  # placement split
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        '_ai_plan_probe', os.path.join(ROOT, 'kicad_routing_plugin', 'ai_plan.py'))
    src = open(spec.origin, encoding='utf-8').read()
    m = re.search(r'KNOWN_ACTIONS\s*=\s*\(([^)]*)\)', src, re.S)
    assert m, "KNOWN_ACTIONS not found in ai_plan.py"
    actions = re.findall(r"['\"]([a-z_]+)['\"]", m.group(1))
    assert actions, actions
    for bad in ('place', 'placement', 'place_optimize', 'place_route_loop',
                'quench', 'floorplan'):
        assert bad not in actions, (
            f"{bad!r} became a plan action. ai_plan drops unknown actions and "
            f"runs the rest, so a placement step in a plan silently routes an "
            f"un-placed board")

    # And the skill's own plan TEMPLATE must not grow one either.
    skill = _all_skill_text()
    fences = re.findall(r'```[^\n]*\n(.*?)```', skill, re.S)
    template = max((f for f in fences if '"action"' in f or 'Step-by-Step' in f),
                   key=len, default='')
    assert template, "the example plan template was not found"
    for tool in ('place_optimize.py', 'place_route_loop.py'):
        assert tool not in template, \
            f"{tool} appeared in the plan template; placement is CLI-only"
    print(f"  PASS: {len(actions)} plan actions, none placement; "
          f"template clean")


def test_skill_states_the_board_outline_is_not_editable():
    """#549. True today only by construction -- no writer emits an Edge.Cuts
    primitive -- and stated nowhere, so nothing stops a future change or a
    confident model from resizing a board to make parts fit."""
    skill = _all_skill_text()
    low = skill.lower()
    # AND, not OR. Written as `or` first, this passed with either phrase
    # deleted -- both were present, so neither was actually pinned.
    assert 'outline is not yours to change' in low, \
        "the skill must state that the board outline is the user's, not ours"
    assert 'never resize a board' in low, \
        "the skill must carry the imperative, not only the heading"
    # and must name the three tools that DO rewrite Edge.Cuts, as things not to run
    for tool in ('fix_outline_gaps.py', 'strip_routing.py', 'prep_set2.py'):
        assert tool in skill, f"{tool} rewrites Edge.Cuts and is not warned about"
    assert 'oob_area' in skill, \
        "the cutout-blind metric must be called out where oob is discussed"
    print("  PASS: outline rule present, all 3 rewriting tools named")


def test_verdict_lines_do_not_collide_with_the_gui_result_contract():
    """ai_backend.extract_result_line takes the LAST `RESULT=` line and ai_gui
    parses it as the plan JSON. A verifier verdict spelled `RESULT=` would be
    read as a malformed plan."""
    src = open(os.path.join(ROOT, 'kicad_routing_plugin', 'ai_backend.py'),
               encoding='utf-8').read()
    assert 'RESULT=' in src, "the host contract moved; re-check this gate"
    # The VERDICT= verifier contract lives in the COMBINED skill: when the
    # skills merged, the verifier stages moved out of plan-pcb-routing (which
    # is a pure routing planner again) into plan-pcb-placement-and-routing.
    # Checking SOURCES[0] here asserted the contract against a file that no
    # longer owns it.
    for rel in ('.claude/skills/plan-pcb-placement-and-routing/SKILL.md',
                '.claude/skills/plan-pcb-placement-and-routing/references/verifier-prompts.md'):
        path = os.path.join(ROOT, rel)
        if not os.path.isfile(path):
            continue
        text = open(path, encoding='utf-8').read()
        for line in text.splitlines():
            st = line.strip().strip('`')
            if st.startswith('RESULT=') and ('PASS' in st or 'FAIL' in st
                                             or 'lens=' in st):
                raise AssertionError(
                    f"{rel}: verifier verdict spelled RESULT=, which the GUI "
                    f"parses as the plan JSON: {st[:60]}")
        assert 'VERDICT=' in text, f"{rel}: no VERDICT= contract found"
    print("  PASS: verdicts use VERDICT=; RESULT= left to the host")


def test_the_score_is_the_gate_and_the_router_is_not_the_judge():
    """The board that prompted this shipped at 39/44 nets and 762 DRC errors
    with every tool reporting success, because the only thing being consulted
    was the router's own tally. Two claims have to stay in the skill or that
    recurs: the score exists and is the gate, and place_route_loop's ACCEPTED
    is not a verdict."""
    skill = _all_skill_text()
    low = skill.lower()

    assert 'board_score.py' in skill, \
        "the skill must name the score helper -- it is the only number not " \
        "produced by the thing being graded"
    # The router's self-report must be explicitly demoted. Without this the
    # skill reads ACCEPTED as 'this round is good', which is how a disconnected
    # board survives an 'improving' loop.
    assert 'not a quality verdict' in low, \
        "the skill must state that place_route_loop's ACCEPTED is not a verdict"
    assert 'better()' in skill and 'place_route_loop.py:358' in skill, \
        "cite where the router-self-report comparison actually lives"
    # The loop has to be bounded, or 'keep going until fixed' is unbounded.
    assert '100 iterations per board' in low or '100 per board' in low, \
        "the convergence budget must be stated in the skill"
    # ...but bounded is not the only failure mode. A run stopped at 11 of 20 and
    # called it "budget exhausted" while its own ledger said the levers were not
    # exhausted, so the skill must also say what is NOT a stop condition.
    assert 'not a stop condition' in low or 'not stop conditions' in low, \
        "the skill must name the non-reasons for stopping (wall-clock, fatigue, " \
        "'the score stopped moving') -- bounding the loop from above is useless " \
        "if it can be abandoned from below"
    # The lever must be chosen by connectivity, not by whichever number is
    # biggest: `drc` can be ~90% grading artifact on a multi-class board, and a
    # run that let it pick spent eleven iterations on clearances while five nets
    # carried no copper.
    assert 'connectivity first' in low, \
        "the skill must rank unrouted/broken above drc when choosing the lever"
    # Re-entering at the failing step is what makes a 100-iteration budget cheap.
    assert 'rip-existing-nets' in skill, \
        "ripping blocking nets must be documented as a sanctioned lever"
    # Vacuity: ungraded must never read as clean.
    assert 'ungraded' in low, "the skill must distinguish ungraded from passed"

    conv = os.path.join(ROOT, '.claude/skills/plan-pcb-placement-and-routing/references/convergence.md')
    assert os.path.isfile(conv), "references/convergence.md is missing"
    ctext = open(conv, encoding='utf-8').read()
    # parent_sha / "stop condition" are the REAL schema (board_store.Ledger +
    # converge.py record); the old parent_board/stopped_by names never existed
    # in any writer and the pin rotted silently.
    for key in ('ledger.jsonl', 'parent_sha', 'stop condition', 'blocking'):
        assert key in ctext, f"convergence.md does not document `{key}`"
    # The ledger has to be the one the TOOLS read. `board_store.Ledger` is
    # append-only JSONL and `converge.py record` is its only writer, so a
    # hand-written single JSON document leaves step-back, replay, status and
    # make_film --from-ledger all unreachable -- which is what the skill used
    # to prescribe.
    for verb in ('record', 'status'):
        assert f'converge.py {verb}' in ctext or f'converge.py {verb}' in skill, \
            f"neither the skill nor convergence.md names `converge.py {verb}`"
    # The ledger is bare JSONL with NO wrapper object (convergence.md says so
    # itself), so there is no "limit" field to pin -- the budget lives in
    # prose and in stop condition 2's "100 ledger entries actually written".
    assert '100 per board' in ctext, \
        "convergence.md must state the 100-per-board budget"
    print("  PASS: score is the gate, router self-report demoted, loop bounded")


def test_routed_board_lenses_exist_and_reenter_the_loop():
    """A verifier fan-out that only reports is not a gate. The three
    routed-board lenses must exist, and a FAIL must be documented as
    re-entering the loop rather than becoming a caveat on a shipped board."""
    rel = '.claude/skills/plan-pcb-placement-and-routing/references/verifier-prompts.md'
    text = open(os.path.join(ROOT, rel), encoding='utf-8').read()
    for lens in ('connectivity', 'drc', 'spec'):
        assert f'`{lens}`' in text, f"routed-board lens `{lens}` is missing"
    assert 're-enters the loop' in text.lower(), \
        "a FAIL must be documented as re-entering the loop, not as a footnote"
    assert 'do not re-word a fail into a caveat' in text.lower(), \
        "the caveat-laundering failure mode must be refused by name"
    print("  PASS: 3 routed lenses present, FAIL re-enters the loop")


TESTS = [
    test_every_documented_flag_exists,
    test_driver_commands_supply_required_options_and_values,
    test_the_score_is_the_gate_and_the_router_is_not_the_judge,
    test_routed_board_lenses_exist_and_reenter_the_loop,
    test_the_placement_tools_are_actually_mentioned,
    test_exit_code_contract_is_documented,
    test_routing_only_stays_the_default_path,
    test_skill_states_the_board_outline_is_not_editable,
    test_verdict_lines_do_not_collide_with_the_gui_result_contract,
    test_skill_decides_placement_by_measurement_not_by_default,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
