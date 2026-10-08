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
design, not assertion -- the free-agent skill's independent verifier and the
board-state gates that refuse the worst case outright. (The staged placement
skill's measure-then-decide assertions left with that skill.)

THE THREE BLIND SPOTS (#923), named here so the next person who finds a class
this gate misses knows which of the three they are looking at. A fact-checking
pass found ~15 wrong claims in the skills and this file passed on every one:

  1. IT CHECKED THAT A FLAG EXISTS, NEVER WHAT IT MEANS. A moved default, an
     inverted meaning and a changed exit contract all read as fine.
     Closed for two of those:
     `test_the_defaults_the_skills_quote_are_the_real_defaults` resolves a
     quoted default against the real parser (`--heuristic-weight` was taught
     as 1.9 after #586 made it 2.3), and `test_exit_code_contract_is_documented`
     no longer accepts the literal string `exits 3` as evidence -- it reads
     whether the annotated flag can reach `gate_or_exit` at all. What is still
     open: a flag whose MEANING moved without its default or exit code moving.

  2. IT NEVER SAW A REFUSAL. `source_text` read a driver through `--dump-all`,
     which fabricated PASSING evidence for every guard, so the commands inside
     `err(...)` were unscanned. Closed by `--dump-refusals` at the time; the
     staged drivers (placement_driver.py, loop_driver.py) have since been
     RETIRED along with that machinery, and no source here emits commands
     from code any more -- every source is read as the text it is.

  3. IT COULD NOT SEE A CLAIM ABOUT A TOOL'S OUTPUT. Every "read the `X` field"
     instruction was invisible -- the class containing `hot[].ratio`, a key no
     instrument emits. Closed in a sibling file,
     `tests/test_923_output_key_claims.py`, which runs the six instruments the
     skills quote and resolves their cited keys against the real documents.
     What is still open, and that file says so: a key claim in prose that names
     no instrument, and a claim about a tool it does not run.

A FOURTH, found by #1115: IT CHECKED THAT A FLAG EXISTS, NEVER THAT IT IS GIVEN
ITS VALUE. `board_brief.py <board> --json` passed, because `--json` exists; it
takes a PATH, so the command exited 2 as written. And the tool was never
enrolled at all, because discovery read only `python3 ... x.py` lines and the
skill ran it bare in a backtick span. Closed by
`test_every_value_flag_is_given_a_value` (arity read from `--help`) and
`_BARE_SPAN_RE`. What is still open:
  * a POINTER -- `` `route.py --nets` `` in a sentence, a tool-index row --
    names a flag without running anything and is deliberately not read, so a
    pointer that drops a value is invisible; and a tool cited ONLY as
    pointers is never enrolled at all, so not even its flag names are
    checked (board_context, check_channels, check_reachability and others
    -- their flags all existed when this was written);
  * a value of the WRONG kind (a path where a number belongs) reads as given;
  * short options (`-c`, `-o`) are not read, and shell shapes the token
    split does not model -- an operator glued to its neighbour (`--json;`,
    `|tee`, `2>/dev/null`) -- read as values. None occurs in a source today.
"""

import functools
import importlib.util
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Declared rather than left to the 600 s global: ~20 tools are asked for
#: their parser, and a gate killed by the runner's default budget reports the
#: same "no result" as a broken one. #1115's arity scan asks every discovered
#: tool (and subcommand) for `--help`; reading an option's CHOICES as
#: subcommands had made that ~150 s, and since `_subcommand_names` the whole
#: file runs in about 30 s.
RUN_ALL_TIMEOUT = 900


#: CACHED because the scans below ask for every source once per TOOL. Sound
#: because nothing in this file writes to a source between reads.
@functools.lru_cache(maxsize=None)
def source_text(rel):
    """What the executor actually reads for this source: its text.

    A MISSING source reads as empty, and `test_every_source_exists` is what
    keeps that from being a quiet way to stop gating a file.
    """
    path = os.path.join(ROOT, rel)
    if not os.path.isfile(path):
        return ''
    return open(path, encoding='utf-8', errors='replace').read()


def _all_skill_text():
    """Every skill file as one string: a rule may live in any of the three."""
    out = []
    for rel in SOURCES:
        path = os.path.join(ROOT, rel)
        if os.path.isfile(path):
            out.append(open(path, encoding='utf-8').read())
    return '\n'.join(out)


# Files that instruct Claude or a human to run these tools. A flag can live in
# any of them, and the whole point of this gate is that NO skill file drifts
# from the real parsers -- so every one is a source.
SOURCES = [
    '.claude/skills/plan-pcb-routing/SKILL.md',
    # The free-agent skill replaced the staged placement and combined skills
    # (and their drivers). It prescribes no procedure, but every command it
    # does spell -- the milestone record, the film, the verifier's checks --
    # is one the executor runs verbatim. Its reference page carries most of
    # them (#549: a block moved out of SKILL.md must not go flag-unchecked).
    '.claude/skills/pcb-free-agent/SKILL.md',
    '.claude/skills/pcb-free-agent/references/verifier.md',
    'docs/floorplan-intent.md',
    'docs/placement-optimization.md',
    'docs/claude-skills.md',
    'py_placer/placement/README.md',
    'README.md',
    # The stress RUNBOOK is prose a run is told to follow, with live command
    # blocks in it (`run_watch.py`, `tee_cmd.py`, the audits), and it was in NO
    # text-invariant gate's source list: not this one, not test_doc_flag_liveness,
    # not test_803. Doctrine that lands only there ships unpinned, which is how
    # the watcher protocol went two runs without one.
    'tests/stress/RUNBOOK.md',
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


#: #1115: a tool run BARE inside one backtick span -- `` `board_brief.py <board>
#: --json` `` -- is an invocation too, and the `python3` prefix is the only
#: thing it lacks. `_TOOL_RE` alone never enrolled board_brief.py, so the one
#: command the free-agent skill gives for choosing its mode (exit 2 as cited)
#: was checked by nothing. Narrow on purpose: the token after the tool must be
#: a `<placeholder>` or a positional, both INSIDE the span, so a module named
#: in prose (`` `routing_defaults.py` holds ``) stays out.
_BARE_SPAN_RE = re.compile(
    r'`((?:[\w./-]+/)?[a-z][a-z0-9_]*\.py)[ \t]+(?:<[^>`\n]+>|[^\s`<-][^\s`]*)')

#: Where a bare basename is looked up, in this order.
_TOOL_DIRS = ('', 'py_router', 'py_tools', 'py_placer')


def _tools_in(text):
    """The repo tools `text` invokes: `python3 ... x.py` lines, and backtick
    spans that run a tool bare (`_BARE_SPAN_RE`)."""
    found = set()
    names = [m.group(1) for m in _TOOL_RE.finditer(text)]
    bare = [m.group(1) for m in _BARE_SPAN_RE.finditer(text)]
    for name in names + bare:
        name = name.replace('\\', '/')
        # A test is not a tool: it has no flag contract for the executor,
        # and resolving its "parser" means EXECUTING it inside this gate.
        if name.startswith(('tests/', 'wk/')):
            continue
        if os.path.isfile(os.path.join(ROOT, name)):
            found.add(name)
        elif '/' not in name:
            hit = next((os.path.join(d, name).replace('\\', '/')
                        for d in _TOOL_DIRS
                        if os.path.isfile(os.path.join(ROOT, d, name))), None)
            if hit:
                found.add(hit)
    return found


def discovered_tools():
    """Every repo tool the skills tell the executor to run."""
    found = set()
    for rel in SOURCES:
        path = os.path.join(ROOT, rel)
        if not os.path.isfile(path):
            continue
        found |= _tools_in(source_text(rel))
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
    # buries the true ones. argparse prints the choices as {a,b,c} -- for
    # an option's choices too, so only the positional section's group is a
    # subcommand list (`_subcommand_names`, #1115).
    for sub in _subcommand_names(text):
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
    spans = re.findall(r'`([^`]+)`', block)
    # ...and what is NOT in backticks, when a command lives there. A refusal
    # says things like "The close-out reports `blocking` as str" and then
    # prints its recipe unquoted: scanning only the backticked spans dropped
    # the recipe entirely, which a battery row proved by shipping
    # `check_assembly.py <board> --totally-bogus x` past this gate. Keeping the
    # two apart is what stops a sentence's `--flag` being read as the
    # neighbouring tool's.
    outside = re.sub(r'`[^`]+`', ' ', block)
    if '.py' in outside:
        spans.append(outside)
    if not spans:
        spans = [block]

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
    """Whole shell commands (handling trailing backslashes) that run `tool`.

    A tool is matched by its PATH or by its bare basename, because the drivers
    spell it both ways -- `check_assembly.py` appears ten times in loop_driver
    and only six of them carry `py_tools/`. Matching the path alone left a
    command spelled the other way unscanned: a battery row put
    `check_drc.py <board> --totally-bogus-flag x` inside a refusal and this
    gate reported ALL PASS. The span walk below already resolves a bare name
    to its TOOLS entry, so only the block selection was missing it.
    """
    base = os.path.basename(tool)
    blocks, cur = [], None
    for line in text.splitlines():
        if cur is not None:
            cur.append(line)
            if not line.rstrip().endswith('\\'):
                blocks.append('\n'.join(cur))
                cur = None
            continue
        if (tool in line or base in line) and not line.lstrip().startswith('#'):
            cur = [line]
            if not line.rstrip().endswith('\\'):
                blocks.append('\n'.join(cur))
                cur = None
    return blocks


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


# ---------------------------------------------------------------------------
# ARITY (#1115): a flag that takes a value must be given one
# ---------------------------------------------------------------------------
# The existence check reads a set of flag NAMES, so `board_brief.py <board>
# --json` passed it: `--json` exists. It takes a PATH, so the command the
# free-agent skill gave for choosing its mode exited 2 as written. Arity is
# read from `--help` only, never from `_parser_obj`: the py_tools/py_placer
# CLIs `import _path`, which resolves only when an earlier tool happened to
# extend `sys.path`, so a parser object is available in an ORDER-dependent
# way and a gate built on it would change its answer with the tool list.

#: A token that ends a shell command, or starts a comment.
_SHELL_STOP = frozenset(('|', '||', '&&', ';', '>', '>>', '2>&1', '2>', '&'))
#: What a quoted payload becomes: ONE value token, never blank (`--nets "*"`
#: is a flag given its value; blanking the quotes made it look valueless).
_QUOTED = 'QUOTED'


def _min_values(head):
    """The fewest values an option takes, from its --help head.

    Python 3.13 prints the metavar ONCE, after the last alias
    (`-n, --nets NETS [NETS ...]`); older ones repeat it per alias. So the
    metavar is read off whichever alias carries one. Optional groups (`[...]`,
    nested) and `...` count nothing; what is left counts one each:
    `NETS [NETS ...]` -> 1, `X0 Y0 X1 Y1` -> 4, `[REF ...]` -> 0.
    """
    tails = []
    for alt in head.split(', '):
        parts = alt.strip().split(None, 1)
        if len(parts) == 2 and parts[0].startswith('-'):
            tails.append(parts[1])
    tail = tails[-1] if tails else ''
    prev = None
    while prev != tail:
        prev, tail = tail, re.sub(r'\[[^\[\]]*\]', ' ', tail)
    return len([t for t in tail.replace('...', ' ').split() if t])


@functools.lru_cache(maxsize=None)
def _sub_help_text(tool, sub):
    env = dict(os.environ, COLUMNS='200', KRT_NO_BANNER='1')
    p = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, tool), sub, '--help'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=180, env=env)
    return (p.stdout or '') + (p.stderr or '')


def _subcommand_names(text):
    """The subcommands a `--help` lists: the brace group that STARTS a line of
    the positional section (`  {record,verdict,...}`). A brace group anywhere
    else is an option's CHOICES (`--escalation {fab,board,off}`); asking
    `tool <choice> --help` re-prints the top-level help, and reading every
    such group was 130 of 182 subprocesses for no information (#1115's
    verifier)."""
    return [s for m in re.finditer(r'^\s+\{([a-z0-9_][a-z0-9_,-]*)\}', text,
                                   re.M)
            for s in m.group(1).split(',')]


#: A usage-synopsis group, `[--near X Y]` / `[--relative]`: a flag and the
#: metavars it takes. place_pose's verbs are parsed by hand and documented
#: ONLY in such an epilog, so the option lines alone never saw `--rot`.
#: Flat groups only: a nested repetition (`[--x A [A ...]]`) is not read,
#: and such a flag would count as a switch. No verb flag has that shape.
_USAGE_GROUP = re.compile(r'\[(--[a-z][a-z0-9-]+)((?:\s+[A-Z][A-Z0-9_]*)*)\]')


@functools.lru_cache(maxsize=None)
def _arity(tool):
    """{--flag: fewest values it takes} over `tool --help` and each
    subcommand's help. A flag two subcommands define differently keeps the
    SMALLER count: this gate reports only what is short under every reading.

    Option lines are the authority; a flag they do not list is read off the
    usage synopses (`_USAGE_GROUP`), which is where a hand-parsed verb's
    options live. A `--help` that prints no option list REFUSES: an empty
    table would read every flag of the tool as a switch, silently."""
    text, rc = _help_text(tool)
    if '--help' not in text:
        raise RuntimeError(f'{tool} --help produced no option list '
                           f'(exit {rc}); its arity cannot be read')
    texts = [text]
    texts.extend(_sub_help_text(tool, s) for s in _subcommand_names(text))
    out = {}
    for t in texts:
        for line in t.splitlines():
            if not re.match(r'\s{1,6}-', line):
                continue
            head = line.strip().split('  ', 1)[0]
            n = _min_values(head)
            for flag in re.findall(r'(--[a-z][a-z0-9-]+)', head):
                out[flag] = min(n, out.get(flag, n))
    for t in texts:
        for m in _USAGE_GROUP.finditer(t):
            if m.group(1) not in out:
                out[m.group(1)] = len(m.group(2).split())
    return out


def _value_blocks(text, tool):
    """(line number, block) for every block that names `tool`.

    `_continued_blocks`' join, plus one more: a backtick span that wraps onto
    the next line (`` `qfn_fanout.py --escape-method `` / `` underpad` ``) is
    joined until its backticks balance -- otherwise the flag reads as given no
    value at the line break. Kept apart from `_continued_blocks` so the
    existence gate's measured floors do not move.
    """
    base = os.path.basename(tool)
    lines = text.splitlines()
    out, i = [], 0
    while i < len(lines):
        line = lines[i]
        if (tool in line or base in line) and not line.lstrip().startswith('#'):
            start, cur = i, [line]
            while ((cur[-1].rstrip().endswith('\\')
                    or '\n'.join(cur).count('`') % 2)
                   and i + 1 < len(lines) and len(cur) < 6):
                i += 1
                cur.append(lines[i])
            out.append((start + 1, '\n'.join(cur)))
        i += 1
    return out


def _resolve(bare):
    return next((t for t in TOOLS
                 if bare == t or bare.endswith('/' + t)
                 or os.path.basename(t) == os.path.basename(bare)), None)


def _is_value(tok):
    """A token that can be an option's value: not the end of the command, not
    another option (a negative number is a value: `--rot -90`), not a shell
    operator, not a comment."""
    if not tok or tok in _SHELL_STOP or tok.startswith('#'):
        return False
    if tok.startswith('-') and not re.match(r'-[0-9.]', tok):
        return False
    return True


def _missing_values(block, tool, arity):
    """[(flag, takes, got)] for every value flag `block` gives too few values,
    inside an INVOCATION of `tool`.

    An invocation is a span where a `python...` token precedes the tool, or
    the token after it is a positional or a `<placeholder>`. A POINTER --
    `` `route.py --nets` `` in prose, a tool-index row -- names a flag without
    running anything, and a gate that failed it would be deleted. Also returns
    how many value flags it checked, so a floor can say it looked.
    """
    block = _ROUTE_ARGS_RE.sub(' --route-args %s ' % _QUOTED, block)
    spans = re.findall(r'`([^`]+)`', block)
    # A block always names the tool, so when no span does, the text outside
    # the spans carries the `.py` and is appended here: no third case.
    outside = re.sub(r'`[^`]+`', ' ', block)
    if '.py' in outside:
        spans.append(outside)

    def quoted(m):
        return m.group(0) if '.py' in m.group(2) else ' %s ' % _QUOTED
    misses, checked = [], 0
    for span in spans:
        # Per SPAN, not per block: a prose apostrophe outside the backticks
        # ("the skill's `x.py <b> --json` isn't") would otherwise pair with
        # one past the span and swallow the command whole -- a silent miss.
        span = re.sub(r"""(['"])(.*?)\1""", quoted, span, flags=re.S)
        toks = [t.strip('\'"`(),;').replace('\\', '/')
                for t in re.split(r'\s+', span)]
        toks = [t for t in toks if t and t != '/']
        current = None
        for i, tok in enumerate(toks):
            if tok.endswith('.py'):
                hit = _resolve(tok)
                current = None
                if hit == tool:
                    nxt = toks[i + 1] if i + 1 < len(toks) else ''
                    ran = any(re.match(r'python[0-9.]*(\.exe)?$', t)
                              for t in toks[:i])
                    if ran or (nxt and _is_value(nxt)):
                        current = hit
                continue
            if current != tool:
                continue
            m = re.match(r'(--[a-z][a-z0-9-]+)(=?)', tok)
            if not m or m.group(2):
                continue
            need = arity.get(m.group(1), 0)
            if need <= 0:
                continue
            checked += 1
            got = 0
            while got < need and i + 1 + got < len(toks) and _is_value(
                    toks[i + 1 + got]):
                got += 1
            if got < need:
                misses.append((m.group(1), need, got))
    return misses, checked


@functools.lru_cache(maxsize=None)
def _parser_obj(tool):
    """The parser itself, for metadata a flag-name set cannot carry.

    CACHED: building one runs the module (and, for the tools that build their
    parser inside `main()`, calls it), and three tests now ask for the same
    parsers.
    """
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


#: The LAST number in a Default cell, so the rows that spell the real rule --
#: "board's Default class, else 0.25" -- stay checked instead of dropping out
#: of the scan for being honest about where the value comes from.
_DEFAULT_CELL = re.compile(r'([0-9][0-9.]*[0-9]|[0-9])(?![0-9.])')
_DEFAULT_PROSE = re.compile(r'default[:\s]+`?([0-9][0-9.]*)`?')
_FLAG_IN = re.compile(r'(--[a-z][a-z0-9-]+)')


@functools.lru_cache(maxsize=None)
def _help_blocks(tool):
    """{flag: the help paragraph it heads}, straight out of `tool --help`.

    THE PARSER IS NOT ALWAYS REACHABLE. `route.py` builds its parser under
    `if __name__ == '__main__'` with no `main()` to intercept, so
    `_parser_obj` raises for it -- and the defaults check below silently
    skipped every flag it owns, `--heuristic-weight` included. Measured: the
    battery row that moves `HEURISTIC_WEIGHT` SURVIVED, which is #923's own
    acceptance criterion failing quietly. argparse prints `(default: X)`
    whenever the help string asks for it, and that is the same authority the
    parser would have been.
    """
    text, _rc = _help_text(tool)
    blocks, current = {}, []
    for line in text.splitlines():
        if re.match(r'\s{1,4}-', line):
            # The option NAMES, which argparse separates from the help text by
            # two spaces -- and the line's own indent is two spaces, so the
            # split has to happen after stripping it or every block comes out
            # empty (measured: 0 blocks over a 26 KB --help).
            head = line.strip().split('  ', 1)[0]
            current = re.findall(r'(--[a-z][a-z0-9-]+)', head)
            for f in current:
                blocks.setdefault(f, [])
        for f in current:
            blocks[f].append(line)
    return {f: '\n'.join(v) for f, v in blocks.items()}


@functools.lru_cache(maxsize=None)
def _help_text(tool):
    env = dict(os.environ, COLUMNS='200', KRT_NO_BANNER='1')
    p = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, tool), '--help'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=180, env=env)
    return (p.stdout or '') + (p.stderr or ''), p.returncode


def _all_actions(parser):
    """Every action, SUBCOMMANDS INCLUDED.

    A verb-style tool keeps its real flags on the subparsers -- `converge.py
    verdict --flat N` is not on the top-level parser at all -- and reading only
    the top level attributes such a flag to whichever other tool happens to
    define the same name. `_option_strings` already recurses for the flag-name
    check; this is the same walk, keeping the actions.
    """
    import argparse
    out = list(parser._actions)
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            for sub in action.choices.values():
                out.extend(_all_actions(sub))
    return out


def _owning_tools(flag, text, before):
    """(tool, default, help text) for every tool that defines `flag`.

    A flag name alone does not say whose it is -- `--grid-step` belongs to five
    tools -- so the file's own nearest preceding `python3 ... tool.py` orders
    the list, and everything that defines it stays in it. Returning them ALL
    matters: where several own it and they disagree, that is worth saying
    instead of picking one.

    TWO SOURCES, because one is not enough. The parser is the better authority
    and is unreachable for the tools that build it under
    `if __name__ == '__main__'` -- `route.py` among them -- so a flag nobody's
    parser could be built for is read from `--help`, where argparse prints the
    same default. Without that, every default `route.py` documents went
    unchecked and the battery row that moves `HEURISTIC_WEIGHT` survived.
    """
    owners = []
    for tool in TOOLS:
        try:
            parser = _parser_obj(tool)
        except Exception:                                    # noqa: BLE001
            continue
        for action in _all_actions(parser):
            if flag in action.option_strings:
                owners.append((tool, action.default, action.help or ''))
                break
    if not owners:
        for tool in TOOLS:
            try:
                _parser_obj(tool)
                continue                     # its parser answered above
            except Exception:                                # noqa: BLE001
                pass
            block = _help_blocks(tool).get(flag)
            if not block:
                continue
            m = re.search(r'\(default:\s*([^)]*)\)', block)
            owners.append((tool, (m.group(1).strip() if m else None), block))
    if not owners:
        return []
    seen = [m.group(1) for m in _TOOL_RE.finditer(text[:before])]
    for name in reversed(seen):
        hit = [row for row in owners
               if row[0] == name or row[0].endswith('/' + name)
               or os.path.basename(row[0]) == os.path.basename(name)]
        if hit:
            # Nearest first, everything else behind it: the order is for the
            # failure message, and the check itself reads them all.
            return hit + [row for row in owners if row not in hit]
    return owners


def _quoted_defaults(rel, text):
    """(flag, quoted value, where) for every DEFAULT the text states.

    Two forms, both structural -- no prose parsing:
      * a markdown table with a `Default` column, which is how the routing
        skill's parameter tables are written;
      * `default: N` / `default N` within 120 characters after a flag.
    """
    out = []
    header, default_col, off = None, None, 0
    for lineno, line in enumerate(text.splitlines(), 1):
        off += len(line) + 1
        if line.startswith('|'):
            cells = [c.strip() for c in line.strip().strip('|').split('|')]
            if header is None:
                header = cells
                default_col = next((i for i, c in enumerate(cells)
                                    if c.lower().strip('* ') == 'default'), None)
                continue
            if set(''.join(cells)) <= set('-: '):
                continue                          # the |---|---| rule line
            if default_col is not None and len(cells) > default_col:
                flag = _FLAG_IN.search(cells[0])
                nums = _DEFAULT_CELL.findall(cells[default_col])
                if flag and nums:
                    out.append((flag.group(1), nums[-1],
                                f'{rel}:{lineno}', off))
            continue
        header, default_col = None, None
    for m in _DEFAULT_PROSE.finditer(text):
        window = text[max(0, m.start() - 120):m.start()]
        flag, at = None, None
        for f in _FLAG_IN.finditer(window):
            # A trailing hyphen is a WRAPPED flag (`--min-supply-` at a line
            # end), and a trailing dot is the sentence's, not the number's.
            flag, at = f.group(1).rstrip('-'), f.end()
        # ...and the flag has to be the thing the default belongs to. The span
        # between them may close the flag's own code span (`--flat N` counts
        # RECORDED laps (default 5)) but not run through a SECOND backticked
        # name or a sentence break: `--ignore-nets` sitting two clauses before
        # "`health.max_fanout` is the backstop, default 20" is not a claim
        # about --ignore-nets, and reading it as one is how a gate starts
        # reporting defects that are not there.
        between = window[at:] if at is not None else ''
        if between.count('`') > 1 or ';' in between:
            flag = None
        if flag:
            lineno = text.count('\n', 0, m.start()) + 1
            out.append((flag, m.group(1).rstrip('.'),
                        f'{rel}:{lineno}', m.start()))
    return out


def test_the_defaults_the_skills_quote_are_the_real_defaults():
    """A flag that EXISTS can still be documented with a default that moved.

    This gate has always checked that a cited flag is real and never what it
    MEANS, so `--heuristic-weight` was taught as 1.9 for as long as it took
    nobody to notice `routing_defaults.HEURISTIC_WEIGHT` had become 2.3 (#586).
    A number a reader reasons from is a claim about the code exactly as a flag
    name is, and it is resolved the same way: against the real parser.

    Sibling of `tests/test_doc_constants.py`, which holds the constants a doc
    quotes BY NAME. This holds the ones it quotes as a flag's default -- the
    two live in different files because a default is the parser's, and this is
    where the parsers are already built.
    """
    problems, checked, unresolved = [], 0, []
    for rel in SOURCES:
        if not rel.endswith('.md'):
            continue
        path = os.path.join(ROOT, rel)
        if not os.path.isfile(path):
            continue
        text = open(path, encoding='utf-8', errors='replace').read()
        for flag, quoted, where, off in _quoted_defaults(rel, text):
            owners = _owning_tools(flag, text, off)
            if not owners:
                unresolved.append((where, flag))
                continue
            checked += 1
            reals = {}
            for tool, default, _help in owners:
                reals.setdefault(repr(default), []).append(tool)
            try:
                want = float(quoted)
            except ValueError:
                continue

            def _number(value):
                if isinstance(value, bool) or value is None:
                    return None
                if isinstance(value, (int, float)):
                    return float(value)
                try:
                    return float(str(value).strip())
                except ValueError:
                    return None

            # ANY owner agreeing is enough. A flag name does not say whose it
            # is -- `--flat` is a plateau count on `converge.py` and a boolean
            # on `render_placement.py`, and the nearest preceding invocation
            # picked the wrong one -- so accusing the doc on one tool's default
            # reports a defect that is not there. What this gate is for is a
            # number that matches NOTHING, which is what a moved default is.
            ok = any(_number(d) == want for _t, d, _h in owners)
            if not ok:
                # A `default=None` flag whose help says "the board's own
                # constraint, else X" is not undocumented -- X is the real
                # fallback and the help string is BUILT from the constant, so
                # it is the same authority as the default would be. Matched
                # with digit boundaries, or 0.2 would be satisfied by 0.25.
                pat = re.compile(r'(?<![0-9.])' + re.escape(quoted)
                                 + r'(?![0-9])')
                ok = any(_number(d) is None and pat.search(h or '')
                         for _t, d, h in owners)
            if not ok:
                problems.append(
                    (where, flag, quoted,
                     ', '.join(f'{v} ({"/".join(t)})'
                               for v, t in sorted(reals.items()))))
    assert not problems, (
        'documented defaults that are not the real ones:\n'
        + '\n'.join(f'  {w}: {f} documented as {q}, parser says {r}'
                    for w, f, q, r in sorted(problems)))
    # The two parameter tables in the routing skill alone supply five rows, so
    # a scan finding fewer than that has stopped matching rather than the
    # skills having stopped quoting defaults.
    assert checked >= 8, f'only {checked} documented default(s) resolved'
    print(f'  PASS: {checked} documented defaults match their parser'
          + (f' ({len(unresolved)} flag(s) no discovered tool defines)'
             if unresolved else ''))


def test_every_documented_flag_exists():
    problems = []
    checked = 0
    uncited = []
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
            # DISCLOSED, not silently skipped (#937). A tool the skills invoke
            # and pass no flag to is legitimate -- but it is also exactly what
            # a tool going dark looks like, and the 900-odd aggregate below
            # absorbs it either way. Naming the population turns an invisible
            # zero into a number somebody can argue with, which is the shape
            # #939 used for the 13 value-unchecked spans.
            uncited.append(tool)
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
    # History: 862 measured while the two staged drivers were sources (their
    # instruction and refusal dumps supplied ~500 of them). The drivers and
    # the staged skills were RETIRED for pcb-free-agent, whose skill spells
    # few commands by design. Measured after: 352 -- the floor sits close
    # under it so that losing one whole source still trips it.
    assert checked >= 320, f"only {checked} flag citations found -- scanner broken?"
    # ...and the population this aggregate CANNOT see. A per-tool floor was
    # considered and rejected: test_doc_flag_liveness gives the reason in its
    # own words -- "a gate that cries wolf gets deleted" -- and most of these
    # are tools the skills legitimately invoke bare. A CEILING is the right
    # shape: legitimate to have, illegitimate to grow.
    assert len(uncited) <= 10, (
        f"{len(uncited)} of the {len(TOOLS)} discovered tools are invoked "
        f"with no flag at all, so this gate checks nothing about them: "
        f"{sorted(uncited)}")
    print(f"  PASS: {checked} flag citations, all real "
          f"({len(uncited)} tool(s) invoked with no flag: "
          f"{', '.join(sorted(os.path.basename(t) for t in uncited))})")


def test_the_placement_tools_are_actually_mentioned():
    """Guards the reverse failure: the gate passing because the skill stopped
    mentioning placement at all."""
    skill = _all_skill_text()
    for token in ('place_optimize.py', 'render_placement.py', '--suggest-locks',
                  'Step 0'):
        assert token in skill, f"{token} missing from the skill"


def _returns_before_gate(tool, dest):
    """Does `tool`'s main() return inside `if args.<dest>:` before it gates?

    The board-state gate (`gate_or_exit`) is what makes exit 3 possible, so a
    branch that returns above it cannot produce exit 3 whatever the doc says.
    Read statically, because the alternative is running the tool on an
    unplaced board and there is no unplaced board in the fixtures.
    """
    import ast
    src = open(os.path.join(ROOT, tool), encoding='utf-8',
               errors='replace').read()
    tree = ast.parse(src)
    for fn in ast.walk(tree):
        if not isinstance(fn, ast.FunctionDef) or fn.name != 'main':
            continue
        gate = None
        for node in ast.walk(fn):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == 'gate_or_exit'):
                gate = node.lineno if gate is None else min(gate, node.lineno)
        if gate is None:
            return False                 # no gate at all: nothing to precede
        for node in ast.walk(fn):
            if not isinstance(node, ast.If) or node.lineno >= gate:
                continue
            names = {n.attr for n in ast.walk(node.test)
                     if isinstance(n, ast.Attribute)}
            if dest not in names:
                continue
            if any(isinstance(x, ast.Return) for x in ast.walk(node)):
                return True
    return False


def test_exit_code_contract_is_documented():
    """The skill tells Claude to branch on exit 3, and every "exits 3" beside a
    command is a claim about THAT command.

    The substring check this used to be was satisfied by a false claim: the
    placement skill annotated `place_optimize.py <board> --suggest-locks` with
    "exits 3 if not [placed]", and `--suggest-locks` returns 0 from its own
    branch above `gate_or_exit` -- measured exit 0 on a copper-carrying board,
    and the same file says so 350 lines further down. That is blind spot 1 of
    #923 in one line: the flag exists, the exit contract it is documented with
    does not.
    """
    from placement.placement_state import UNPLACED_EXIT
    assert UNPLACED_EXIT == 3
    skill = _all_skill_text()
    assert 'exit 3' in skill or 'exits 3' in skill, \
        "the skill must state the exit-3 contract it tells Claude to rely on"

    # The positive control, and it is the real code rather than a fixture: the
    # analyser must SEE place_optimize's --suggest-locks branch returning above
    # the gate, and must not see one where there is none. Without this pair a
    # broken analyser reports every claim as fine.
    assert _returns_before_gate('py_placer/place_optimize.py',
                                'suggest_locks'), \
        'the analyser no longer sees the --suggest-locks early return; either ' \
        'place_optimize changed or this check stopped working'
    assert not _returns_before_gate('py_placer/place_optimize.py',
                                    'allow_unplaced'), \
        'the analyser reports a branch that does not return above the gate'

    problems, checked = [], 0
    for rel in SOURCES:
        if not rel.endswith('.md'):
            continue
        path = os.path.join(ROOT, rel)
        if not os.path.isfile(path):
            continue
        lines = open(path, encoding='utf-8', errors='replace').read().splitlines()
        for i, line in enumerate(lines):
            if 'exits 3' not in line and 'exit 3' not in line:
                continue
            if not line.lstrip().startswith('#'):
                continue            # prose, not an annotation on a command
            block = '\n'.join(lines[i + 1:i + 4])
            for tool in TOOLS:
                for b in _continued_blocks(block, tool):
                    for flag in _cited_flags(b, tool):
                        dest = flag.lstrip('-').replace('-', '_')
                        checked += 1
                        if _returns_before_gate(tool, dest):
                            problems.append((f'{rel}:{i + 1}', tool, flag))
    assert not problems, (
        'commands annotated "exits 3" whose flag returns before the board-state '
        'gate:\n' + '\n'.join(f'  {w}: {t} {f} answers and returns 0'
                              for w, t, f in sorted(set(problems))))
    # POSITIVE CONTROL on the SCANNER, because `checked == 0` today and a
    # scanner that has stopped matching reports exactly the same zero. The two
    # `_returns_before_gate` assertions above hold the ANALYSER down; nothing
    # held down the thing that feeds it. Synthetic text in the shape the scan
    # looks for -- a `#` comment carrying "exits 3", a command within the next
    # three lines -- must produce exactly one row, and the same text with the
    # annotation as prose rather than a comment must produce none.
    def _scan(sample):
        n = 0
        ls = sample.splitlines()
        for i, line in enumerate(ls):
            if 'exits 3' not in line and 'exit 3' not in line:
                continue
            if not line.lstrip().startswith('#'):
                continue
            blk = '\n'.join(ls[i + 1:i + 4])
            for tool in TOOLS:
                for b in _continued_blocks(blk, tool):
                    n += len(_cited_flags(b, tool))
        return n

    _hit = ('# --suggest-locks exits 3 when the board is unplaced\n'
            'python3 -X utf8 py_placer/place_optimize.py b.kicad_pcb '
            '--suggest-locks\n')
    _miss = ('The tool exits 3 when the board is unplaced.\n'
             'python3 -X utf8 py_placer/place_optimize.py b.kicad_pcb '
             '--suggest-locks\n')
    assert _scan(_hit) == 1, (
        'the exit-code scanner no longer finds an annotated command -- so its '
        f'{checked} is a claim about the scanner, not about the skills')
    assert _scan(_miss) == 0, (
        'the exit-code scanner reads PROSE as an annotation; an "exits 3" in a '
        'sentence is not a claim attached to a command')
    print(f'  PASS: exit-3 contract; {checked} annotated flag(s) reach the gate'
          if checked else
          '  PASS: exit-3 contract; 0 commands in the skills are annotated with '
          'an exit code today -- the analyser controls and the scanner control '
          'are what is live')


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


def test_verdict_lines_do_not_collide_with_the_gui_result_contract():
    """ai_backend.extract_result_line takes the LAST `RESULT=` line and ai_gui
    parses it as the plan JSON. A verifier verdict spelled `RESULT=` would be
    read as a malformed plan."""
    src = open(os.path.join(ROOT, 'kicad_routing_plugin', 'ai_backend.py'),
               encoding='utf-8').read()
    assert 'RESULT=' in src, "the host contract moved; re-check this gate"
    # The VERDICT= verifier contract lives in the free-agent skill's verifier
    # brief (it replaced the staged skills' verifier prompts). A MISSING file
    # fails rather than being skipped: skipping is how this check went on
    # passing against two retired files that no longer existed.
    for rel in ('.claude/skills/pcb-free-agent/references/verifier.md',):
        path = os.path.join(ROOT, rel)
        assert os.path.isfile(path), f'{rel} is missing; re-aim this gate'
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


def test_every_source_exists():
    """`source_text` reads a missing source as empty, so a retired file left in
    SOURCES stops being gated without a sound -- every floor here is then a
    claim about fewer files than it names. Measured: two skills and their
    drivers were retired and this gate kept listing all seven files."""
    missing = [rel for rel in SOURCES
               if not os.path.isfile(os.path.join(ROOT, rel))]
    assert not missing, f'SOURCES names files that do not exist: {missing}'


def test_the_value_scan_catches_a_missing_value():
    """#1115's control, on text this test hands the scanner: the cited form
    that exited 2 must be caught, and every legitimate shape that looks like
    it must not. Uses board_brief's REAL arity (`--json PATH`), so a parser
    change that made `--json` a switch would fail here, not pass silently."""
    tool = 'py_tools/board_brief.py'
    arity = _arity(tool)
    assert arity.get('--json') == 1, f"board_brief --json arity {arity.get('--json')}"

    def scan(text):
        hits = []
        for _ln, block in _value_blocks(text, tool):
            hits.extend(_missing_values(block, tool, arity)[0])
        return hits

    must_hit = [
        # the #1115 line as it shipped
        "With no mode, `board_brief.py <board> --json` (`pile`) is the test.",
        "`python3 -X utf8 py_tools/board_brief.py b.kicad_pcb --json | tee x`",
        "`py_tools/board_brief.py <board> --json --fit 1`",
        "```bash\npython3 -X utf8 py_tools/board_brief.py b.kicad_pcb \\\n"
        "    --json\n```",
        # an apostrophe on each side must not pair across the span
        "the skill's `board_brief.py <board> --json` isn't runnable",
        # a span that wraps: the flag is at the end of the SPAN, not the line
        "run `board_brief.py <board>\n--json` here",
        # a comment ends the command
        "```bash\npython3 py_tools/board_brief.py b.kicad_pcb --json  # brief\n```",
        # a `python3` prefix makes it an invocation even with no positional
        "`python3 py_tools/board_brief.py --json`",
        # an UNQUOTED command beside a span on the same line
        "read `pile`, then python3 py_tools/board_brief.py b.kicad_pcb --json",
    ]
    must_pass = [
        "`python3 -X utf8 py_tools/board_brief.py <board> --json brief.json`",
        "`board_brief.py <board> --json=brief.json`",
        "| read the board | `py_tools/board_brief.py --json`, more |",
        "the skill's `board_brief.py <board> --json <out>` isn't a pointer",
        "`board_brief.py <board> --json \"wk/a b/brief.json\"`",
        "`board_brief.py <board>\n--json <out>` wraps onto the next line",
        "`board_brief.py <board> --fit -0.5`",
    ]
    assert arity.get('--fit') == 1, f"board_brief --fit arity {arity.get('--fit')}"
    for text in must_hit:
        assert scan(text), f"MISSED: {text!r}"
    for text in must_pass:
        assert not scan(text), f"FALSE HIT {scan(text)}: {text!r}"
    # place_pose's verbs are parsed by hand and documented only in its usage
    # synopses: their options must be READ, and a negative value is a value.
    pose = 'py_placer/place_pose.py'
    assert pose in TOOLS, f'{pose} is no longer discovered; pick another tool'
    a = _arity(pose)
    assert (a.get('--rot'), a.get('--near'), a.get('--relative')) == (1, 2, 0), (
        'place_pose verb arity', a.get('--rot'), a.get('--near'))

    def pose_scan(cmd):
        return [m for _l, blk in _value_blocks(cmd, pose)
                for m in _missing_values(blk, pose, a)[0]]
    head = "`python3 py_placer/place_pose.py in.kicad_pcb out.kicad_pcb set U2 "
    assert not pose_scan(head + "1 2 --rot -90`"), 'a negative number read as an option'
    assert pose_scan(head + "--rot`"), 'a verb option given no value was missed'
    assert pose_scan(head + "--near 130`"), 'a 2-value option given 1 was missed'
    # A subcommand's own options come from its own --help.
    conv = 'py_placer/converge.py'
    assert conv in TOOLS and _arity(conv).get('--flat') == 1, (
        'converge verdict --flat is read only from the subcommand help')
    # A --help that prints no option list refuses rather than reading as
    # "every flag is a switch".
    try:
        _arity('tests/no_such_tool_1115.py')
    except RuntimeError as exc:
        assert 'no option list' in str(exc), exc
    else:
        raise AssertionError('an unreadable --help read as an empty arity table')
    # Discovery: the bare span enrols the tool, a module named in prose
    # does not.
    assert 'py_tools/board_brief.py' in _tools_in("`board_brief.py <board>`")
    assert not _tools_in("`routing_defaults.py` holds the defaults")
    print(f"  PASS: {len(must_hit)} missing values caught, {len(must_pass)} "
          f"legitimate shapes passed")


def test_every_value_flag_is_given_a_value():
    """#1115: in every source, a flag that takes a value is given one, inside
    every invocation of every discovered tool. A POINTER (`` `route.py
    --nets` `` in prose, a tool-index row) runs nothing and is not read."""
    from concurrent.futures import ThreadPoolExecutor
    blocks = {tool: [(src, ln, b) for src in SOURCES
                     for ln, b in _value_blocks(source_text(src), tool)]
              for tool in TOOLS}
    # Every `--help` is a subprocess and they are independent: asked one at a
    # time they cost ~4 minutes, which this file's budget cannot absorb.
    with ThreadPoolExecutor(max_workers=8) as ex:
        arities = dict(zip(TOOLS, ex.map(
            lambda t: _arity(t) if blocks[t] else {}, TOOLS)))
    problems, checked = [], 0
    for tool in TOOLS:
        for src, ln, block in blocks[tool]:
            misses, n = _missing_values(block, tool, arities[tool])
            checked += n
            problems.extend((src, ln, tool, f, need, got)
                            for f, need, got in misses)
    assert not problems, (
        "flags cited WITHOUT the value they take -- the command fails as "
        "written:\n" + "\n".join(
            f"  {s}:{ln}  {os.path.basename(t)} {f} takes {n}, given {g}"
            for s, ln, t, f, n, g in problems))
    # A scan that stopped matching would pass for the wrong reason. Measured
    # at introduction (#1115): 274. The floor sits close under it, so losing
    # one whole source's commands still trips it.
    assert checked >= 240, f"only {checked} value flags checked -- scanner broken?"
    print(f"  PASS: {checked} value-taking flag citations, every one given "
          f"its value")


TESTS = [
    test_every_source_exists,
    test_the_value_scan_catches_a_missing_value,
    test_every_value_flag_is_given_a_value,
    test_every_documented_flag_exists,
    test_the_defaults_the_skills_quote_are_the_real_defaults,
    test_the_placement_tools_are_actually_mentioned,
    test_exit_code_contract_is_documented,
    test_routing_only_stays_the_default_path,
    test_verdict_lines_do_not_collide_with_the_gui_result_contract,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print("ALL PASS")
