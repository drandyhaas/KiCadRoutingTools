#!/usr/bin/env python3
"""Every `--flag` the routing docs tell you to pass must actually exist.

Run 22 lost a routing lap to `--track-width-floor`. The flag was DELETED in
53a5a16e (with `--protect-nets` and `--net-layers`), but the routing skill
still told the reader to pass it in nine places, `docs/api-routing-config.md`
still documented the config field as live, and `krt_capabilities.py`'s own
usage example still used it. The run followed the documentation, `route.py`
answered `error: unrecognized arguments`, and a lap was spent finding out why.

Nothing caught it, and the near-miss is instructive: `test_krt_capabilities.py`
asserts `--track-width-floor` is absent on `route_planes.py` and
`route_diff.py` -- but never on `route.py`, the one CLI it had actually been
removed from. The removal passed CI because the only assertions about the flag
were about the two tools that never had it.

How this differs from `tests/test_431_skill_commands.py`, which is ALSO a
"every flag the skill tells you to pass must exist" gate over the same skill
files -- keep both, and do not delete either as a duplicate:

  * #431 reads COMMAND LINES and resolves each flag against the NAMED tool's
    real parser (`--help`, or the built argparse object). Stronger, per-tool,
    and it is the right check for an emitted invocation.
  * this gate reads PROSE -- a `--flag` in backticks in a sentence -- and
    resolves it as a UNION over every tool, because prose names a flag without
    naming a tool.

That is exactly the gap 53a5a16e fell through: the stale `--track-width-floor`
instruction was a sentence ("plus `--track-width-floor` for a width clause"),
never a command line, so #431 could not see it and was not at fault.

The gate is UNION-shaped on purpose: a flag is live if ANY routing CLI defines
it. A per-tool gate would drown in false positives, because the prose
legitimately discusses one tool's flag while describing another's step, and a
gate that cries wolf gets deleted.

Run: python3 -X utf8 tests/test_doc_flag_liveness.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.environ.setdefault('KRT_NO_BANNER', '1')

import krt_capabilities as K                                   # noqa: E402

#: The docs this gate holds to the engine's actual surface.
DOCS = (
    os.path.join('.claude', 'skills', 'plan-pcb-routing', 'SKILL.md'),
    os.path.join('.claude', 'skills', 'plan-pcb-placement-and-routing',
                 'SKILL.md'),
    os.path.join('docs', 'api-routing-config.md'),
)

#: What counts as LIVE: any non-test source file that registers the flag with
#: argparse. Deliberately a text scan over the whole engine rather than a
#: per-CLI parser walk -- `krt_capabilities.script_flags` follows a registrar
#: only when it sits BESIDE the script (the rule that stops it handing
#: route_planes the whole of route.py's vocabulary), so flags registered from
#: a sub-package (`py_placer/placement/cli_gates.py` supplies --suggest-locks
#: and --allow-routed) read as dead and the gate cries wolf. A gate that cries
#: wolf gets deleted, which would be worse than no gate.
SKIP_DIRS = ('.git', 'wk', 'kicad_files', 'docs', 'node_modules',
             '__pycache__', 'rust_router')

#: A flag literal quoted inside an `add_argument(...)` CALL -- not any
#: `--flag`-shaped string anywhere in the source.
#:
#: The first version of this gate scanned all non-test source text for
#: `--flag` literals, on the reasoning that a removed flag disappears from the
#: source entirely. It does not. A REMOVED flag survives in prose: docstrings,
#: usage examples, comments and legacy tables. Measured on the commit that
#: introduced this file, the broad scan reported:
#:
#:     --track-width-floor -> LIVE      (krt_capabilities.py's usage example)
#:     --protect-nets      -> LIVE      (a tuple in tests/stress/manifest_to_plan.py)
#:     --net-layers        -> dead
#:
#: -- so the gate was blind to two of the three flags it was written to catch,
#: including the one that cost run 22 a lap. It passed only because that commit
#: had already removed the doc mentions by hand; re-adding one tomorrow would
#: have stayed green. A gate that cries wolf gets deleted, but a gate that
#: never cries at all is worse: it is deleted AND it was never doing anything.
#:
#: The two objections the broad scan was reaching for both survive here:
#:   * flags registered from a sibling sub-package (`--suggest-locks` via
#:     `py_placer/placement/cli_gates.py`) stay live, because the scan is a
#:     UNION over every non-test source file, not a per-CLI parser walk;
#:   * generated boolean pairs stay live via the `--no-` derivation below --
#:     `argparse.BooleanOptionalAction` registers `--refs` and supplies
#:     `--no-refs` (`py_router/route_render.py`), which appears in no
#:     add_argument call anywhere. (The broad scan's comment cited
#:     `--no-ratsnest` here; there is no such flag -- render_placement.py
#:     registers `--ratsnest-nets` and `--ratsnest-all` and nothing else.)
_ADD_ARG_CALL = re.compile(r"add_argument\s*\(")
_FLAG_LIT = re.compile(r"""['"](--[a-z][a-z0-9-]{2,})['"]""")


def _call_args(text, open_paren):
    """The source slice between `add_argument(` and its matching `)`.

    Quote-aware, because help strings routinely contain unbalanced parens
    ("(default: 5)"). Returns '' if the call does not close within a
    generous window -- a malformed slice must yield no flags rather than
    swallow the rest of the file.
    """
    depth, k, n = 0, open_paren, min(len(text), open_paren + 4000)
    while k < n:
        c = text[k]
        if c in '\'"':
            q = c
            trip = text.startswith(q * 3, k)
            k += 3 if trip else 1
            while k < n:
                if text[k] == '\\':
                    k += 2
                    continue
                if trip and text.startswith(q * 3, k):
                    k += 3
                    break
                if not trip and text[k] == q:
                    k += 1
                    break
                if not trip and text[k] == '\n':
                    break
                k += 1
            continue
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return text[open_paren:k]
        k += 1
    return ''


def live_flags():
    """Every long flag any non-test source registers with argparse."""
    out = set()
    for base, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs
                   if d not in SKIP_DIRS and not d.startswith('.')
                   or d in ('.claude',)]
        for name in files:
            # Skip TEST files, not the tests/ tree: tests/stress carries real
            # tools the docs legitimately tell you to run (run_watch.py,
            # fence_audit.py, tee_cmd.py).
            if not name.endswith('.py') or name.startswith('test_'):
                continue
            try:
                text = open(os.path.join(base, name), encoding='utf-8',
                            errors='replace').read()
            except OSError:
                continue
            for m in _ADD_ARG_CALL.finditer(text):
                out |= set(_FLAG_LIT.findall(
                    _call_args(text, m.end() - 1)))
    # Paired boolean flags: several tools register `--x` and get `--no-x`
    # from a helper, so `--no-ratsnest` is real on render_placement.py while
    # appearing in no add_argument call anywhere. A text scan cannot see the
    # generated half, so derive it.
    out |= {'--no-' + f[2:] for f in list(out)}
    return out


#: Flags that belong to something other than this repo's CLIs. Seeded by
#: running the gate once and reading what it found; keep it short, and add to
#: it only for a genuinely foreign tool.
EXTERNAL = {
    # git / shell / kicad-cli / pytest / gh, quoted in worked examples
    '--oneline', '--json', '--format', '--output', '--help', '--version',
    '--no-verify', '--hard', '--force', '--from-source', '--stat',
    '--exclude-all', '--define-var', '--drc', '--severity-all', '--units',
    '--schematic-parity', '--all', '--quiet', '--verbose', '--dry-run',
    '--name-only', '--porcelain', '--short', '--set-upstream', '--amend',
    '--no-pager', '--follow', '--patch', '--word-diff', '--color',
    '--recurse-submodules', '--depth', '--branch', '--tags',
    # Prose placeholders, not flags: "`--flag`" in a worked example, and
    # "`--stitch-`" as the prefix of a family.
    '--flag', '--stitch-',
}

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


#: A mention sitting in one of these sentences is the doc DOING ITS JOB --
#: telling the reader a flag is gone. Counting those as errors would punish
#: the very correction this gate exists to produce, and would leave the gate
#: permanently unfixable: every honest "--x was REMOVED" note would fail it.
_ABSENT = re.compile(
    r'REMOVED|does not exist|do NOT exist|there is no|no such flag|'
    r'was removed|were removed|deleted in|not a flag|do not emit|'
    r'no longer exists|has been removed',
    re.IGNORECASE)


def documented_flags(rel):
    """Long flags a doc tells the reader to PASS, as `--flag` in backticks.

    Bare prose mentions are not matched: the point is the instruction, not
    every incidental word. And a mention whose sentence says the flag is
    ABSENT is not an instruction either -- see `_ABSENT`.
    """
    lines = open(os.path.join(ROOT, rel), encoding='utf-8').read().splitlines()
    found = {}
    for n, line in enumerate(lines):
        # The sentence, generously: this line plus its neighbours, because a
        # "REMOVED" verdict often lands a line away from the flag it names.
        if _ABSENT.search(' '.join(lines[max(0, n - 2):n + 3])):
            continue
        for m in re.finditer(r'`([^`\n]*?)`', line):
            for f in re.findall(r'(?<![\w-])(--[a-z][a-z0-9-]{2,})',
                                m.group(1)):
                found[f] = found.get(f, 0) + 1
    return found



#: Scripts a doc names but the repo does not ship. Same failure as a dead flag
#: -- the reader runs it, gets "No such file or directory", and spends a lap
#: finding out the tool was renamed. `route_disconnected_planes.py` became
#: `repair_planes.py`, and both skills still named the old one.
#:
#: Same ABSENT-sentence exemption as the flag half: a doc that says "x.py was
#: renamed to y.py" is doing its job.
_SCRIPT_RE = re.compile(r'(?<![\w-])([a-z][a-z0-9_]{2,40}\.py)(?![\w])')

#: Names that are examples or foreign tools, not repo scripts.
EXTERNAL_SCRIPTS = {
    # "If the repo has a `check_spec.py` (or equivalent), run it" -- a
    # conditional about a file the reader may have, not an instruction to run
    # one this repo ships. The only entry that is load-bearing today; keep the
    # set this short, and add to it only for a genuinely foreign tool.
    'check_spec.py',
}


def repo_scripts():
    """Every .py filename the repo ships."""
    out = set()
    for base, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS
                   and (not d.startswith('.') or d == '.claude')]
        for name in files:
            if name.endswith('.py'):
                out.add(name)
    return out


def documented_scripts(rel):
    """Script names a doc tells the reader to RUN, as `x.py` in backticks or
    on a command line, minus any sitting in an "it is gone" sentence."""
    lines = open(os.path.join(ROOT, rel), encoding='utf-8').read().splitlines()
    found = {}
    for n, line in enumerate(lines):
        if _ABSENT.search(' '.join(lines[max(0, n - 2):n + 3])):
            continue
        cands = [m.group(0) for m in re.finditer(r'`([^`\n]*?)`', line)]
        if 'python3' in line or re.search(r'\b(py_router|py_placer|py_tools|'
                                          r'tests)/', line):
            cands.append(line)
        for c in cands:
            for s in _SCRIPT_RE.findall(c):
                found[s] = found.get(s, 0) + 1
    return found


def main():
    live = live_flags()
    scripts = repo_scripts()
    check('the capability scan found a plausible flag surface',
          len(live) > 100, f'only {len(live)} flags found by the add_argument scan')
    check('the script scan found a plausible file surface',
          len(scripts) > 100, f'only {len(scripts)} .py files found')

    for rel in DOCS:
        if not os.path.exists(os.path.join(ROOT, rel)):
            continue
        print(f'{rel}')
        documented = documented_flags(rel)
        dead = sorted(f for f in documented
                      if f not in live and f not in EXTERNAL)
        check(f'{os.path.basename(rel)} names no flag the engine dropped',
              not dead,
              'these are documented but exist on NO tool: '
              + ', '.join(f'{f} (x{documented[f]})' for f in dead))

        documented_s = documented_scripts(rel)
        dead_s = sorted(s for s in documented_s
                        if s not in scripts and s not in EXTERNAL_SCRIPTS)
        check(f'{os.path.basename(rel)} names no script the repo dropped',
              not dead_s,
              'these are documented but the repo ships no such file: '
              + ', '.join(f'{s} (x{documented_s[s]})' for s in dead_s))


    print('the near-miss that let 53a5a16e through')
    caps = K.capabilities()
    # test_krt_capabilities asserted this flag's absence on the two tools that
    # never had it, and not on the one it was removed from.
    for token in ('route.py:--track-width-floor',
                  'route.py:--net-layers',
                  'route.py:--protect-nets'):
        check(f'{token} is reported missing',
              bool(K.missing(caps, [token])),
              'either the flag came back, or the capability scan is lying')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
