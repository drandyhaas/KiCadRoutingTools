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

#: ANY quoted long-flag literal in non-test source, not only `add_argument`
#: calls. Deliberately over-broad, because the two error directions are not
#: symmetric here:
#:
#:   a false POSITIVE (calling a live flag dead) fails the build on correct
#:   documentation, and a gate that cries wolf gets deleted -- taking the
#:   real finding with it.
#:   a false NEGATIVE (missing a dead flag) just leaves one stale doc line.
#:
#: And the over-approximation costs almost nothing for the class this gate
#: exists to catch: a flag that was genuinely REMOVED disappears from the
#: source entirely -- help text, error strings and all -- which is exactly
#: what happened to --track-width-floor in 53a5a16e.
#:
#: Narrower rules were tried and each produced false positives: flags
#: registered from a sibling sub-package (--suggest-locks via
#: placement/cli_gates.py) and generated boolean pairs (--no-ratsnest, whose
#: positive half never appears in an add_argument call either).
_ADD_ARG = re.compile(r"(?<![\w-])(--[a-z][a-z0-9-]{2,})")


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
            out |= set(_ADD_ARG.findall(text))
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


def main():
    live = live_flags()
    check('the capability scan found a plausible flag surface',
          len(live) > 100, f'only {len(live)} flags found by the add_argument scan')

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
