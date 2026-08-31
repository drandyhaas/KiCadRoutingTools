#!/usr/bin/env python3
"""#798: the published capability document must match the real parsers.

`krt_capabilities.script_flags` reads flags from SOURCE rather than by
importing, because a consumer asking "can this clone do X" must not be able to
trigger a side effect by asking. That makes it a static approximation of a
runtime fact, and the approximation was wrong in one direction on every
published script: measured at the parent commit, all eight `FLAG_SCRIPTS`
under-reported, by 1 to 18 flags each.

The gate here is the RELATION, enumerated over `FLAG_SCRIPTS` rather than
written out as a list of rows. A hand-written row list only ever covers the
flags someone thought of, and the flags nobody thought of are the ones that
break -- `--intent` reading as unsupported on all four placement CLIs is
exactly that failure, and `tests/test_krt_capabilities.py` passed throughout
because it had no row for it.

Two directions, and they are not symmetric:

  * A FALSE POSITIVE -- claiming a flag the CLI rejects -- kills a consumer's
    chain mid-run on a capability check that passed. `script_flags`' own
    docstring calls avoiding this "the whole correctness of this function".
    Asserted for every script, always.
  * A FALSE NEGATIVE -- missing a flag the CLI accepts -- makes a consumer
    refuse to run against an engine that was fine. Asserted as an exact match
    for the published scripts, because after #798 they DO match exactly, and a
    subset assertion alone would let the old under-reporting creep back.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import krt_capabilities as k                                # noqa: E402

RUN_ALL_TIMEOUT = 300

#: A long option as argparse RENDERS it: at the start of an option line, on its
#: own or after a short alias. Scraping every `--token` out of the help TEXT
#: instead picks up prose ("Use --group-by to choose...") and words broken
#: across a wrap, which manufactures phantom flags like `--net-` and makes the
#: comparison meaningless in the direction that matters.
_OPT = re.compile(r'^\s{0,10}(?:-\w,\s*)?(--[A-Za-z0-9][A-Za-z0-9-]*)', re.M)


def _real_flags(script):
    """What the CLI's own argparse accepts, from `--help`. The ground truth.

    `--help` is argparse's, not the script's, so it is excluded: no source
    scan can or should find it.
    """
    path = k._tool_path(k.ROOT, script)
    r = subprocess.run([sys.executable, '-X', 'utf8', path, '--help'],
                       capture_output=True, text=True, timeout=180)
    return set(_OPT.findall(r.stdout + r.stderr)) - {'--help'}


def t_no_script_claims_a_flag_its_parser_rejects():
    """The dangerous direction, over every published script."""
    for script in k.FLAG_SCRIPTS:
        real = _real_flags(script)
        assert real, f'{script} --help produced no options at all'
        got = set(k.script_flags(k._tool_path(k.ROOT, script)))
        assert got <= real, (
            f'{script} claims flags argparse rejects: {sorted(got - real)}')
    print(f"  PASS: {len(k.FLAG_SCRIPTS)} scripts, no false positives")


def t_every_published_script_reports_every_flag_it_accepts():
    """The under-reporting #798 is about, as an EXACT match.

    Exact rather than "close enough": at the parent commit these differed by
    1 to 18 flags per script, and a tolerance is how that comes back.
    """
    for script in k.FLAG_SCRIPTS:
        real = _real_flags(script)
        got = set(k.script_flags(k._tool_path(k.ROOT, script)))
        assert got == real, (
            f'{script}: missing {sorted(real - got)}, '
            f'extra {sorted(got - real)}')
    print(f"  PASS: {len(k.FLAG_SCRIPTS)} scripts match their parsers exactly")


def t_a_registrar_is_resolved_per_FUNCTION_not_per_module():
    """The false-positive trap the module-level union walked into.

    `py_placer/placement/cli_gates.py` holds four registrars.
    `place_optimize.py` calls all four; `place_portfolio.py` calls three and
    NOT `add_lock_advisor_args`. Unioning the module credits portfolio with
    `--suggest-locks`, which its parser rejects with exit 2 -- so a fix that
    only teaches the resolver about dotted paths passes the first assertion
    here and fails the second.
    """
    caps = k.capabilities()
    assert k.missing(caps, ['place_optimize.py:--intent']) == [], \
        'a flag from a sub-package registrar must be found'
    assert k.missing(caps, ['place_optimize.py:--suggest-locks']) == [], \
        'place_optimize.py calls add_lock_advisor_args'
    # ...and the same registrar module must NOT leak into a script that
    # imports it but never calls that particular registrar.
    port = set(k.script_flags(k._tool_path(k.ROOT, 'place_portfolio.py')))
    assert '--intent' in port, 'place_portfolio.py does call add_intent_arg'
    assert '--align-weight' in port, 'and add_tidiness_args'
    assert '--suggest-locks' not in port, (
        'place_portfolio.py imports cli_gates but never calls '
        'add_lock_advisor_args; argparse rejects --suggest-locks')
    print("  PASS: per-function resolution, both directions")


def t_a_registrar_that_also_owns_a_cli_is_still_followed():
    """The second mechanism, which #798 does not name.

    `fix_kicad_drc_settings.py` registers three flags into route.py's parser
    via `add_drc_fix_args(parser)` -- and has a CLI of its own, so the
    module-level `_builds_own_parser` veto skipped it and route.py
    under-reported every one of them.
    """
    got = set(k.script_flags(k._tool_path(k.ROOT, 'route.py')))
    for flag in ('--keep-thermal', '--enable-used-layers',
                 '--no-fix-drc-settings'):
        assert flag in got, f'route.py accepts {flag} and must report it'
    print("  PASS: a registrar that also owns a CLI is followed")


def t_a_short_alias_does_not_hide_the_long_option():
    """The third mechanism, in the base regex.

    48 call sites spell `add_argument('-q', '--quiet', ...)`. Requiring the
    long form FIRST reported every one as unsupported; `check_floorplan.py`'s
    `--quiet` was the last flag still missing after the registrar work.
    """
    got = set(k.script_flags(k._tool_path(k.ROOT, 'check_floorplan.py')))
    assert '--quiet' in got, 'a long option declared after a short alias'
    print("  PASS: --quiet found behind its -q alias")


def t_a_script_still_does_NOT_inherit_another_CLIs_flags():
    """The invariant the per-function pass must not break.

    Structural, not incidental: a function that builds its own
    `ArgumentParser` is not a registrar, and `route.py` exposes no
    parser-taking registrar at all -- so `route_planes.py`, which imports it,
    cannot inherit its vocabulary however the resolution changes.
    """
    caps = k.capabilities()
    for token in ('route_planes.py:--net-clearances',
                  'route_planes.py:--track-width-floor',
                  'repair_planes.py:--net-clearances',
                  'route_diff.py:--track-width-floor'):
        assert k.missing(caps, [token]), \
            f'{token} does not exist on that CLI and must be reported missing'
    print("  PASS: registrars are followed, other CLIs are not")


def t_a_computed_flag_name_is_a_known_limit_not_a_silent_gap():
    """`render_placement.py` builds 18 toggles as `f'--{name}'`
    (py_tools/render_placement.py:1571-1573), so no source scan can see them.

    That is why it is NOT in FLAG_SCRIPTS, and this pins the reason: if
    someone adds it, the exact-match test above would fail with a list of 18
    flags and no explanation. Here the explanation is the test.
    """
    assert 'render_placement.py' not in k.FLAG_SCRIPTS, (
        'render_placement.py registers flags with a computed name; a source '
        'scan cannot report them, so it must not be published as a flag set')
    got = set(k.script_flags(k._tool_path(k.ROOT, 'render_placement.py')))
    real = _real_flags('render_placement.py')
    assert got <= real, f'even unpublished, no false positives: {got - real}'
    assert real - got, (
        'render_placement.py started declaring every flag literally; the '
        'reason it is excluded from FLAG_SCRIPTS no longer holds')
    print(f"  PASS: {len(real - got)} computed flag names, excluded on purpose")


TESTS = (t_no_script_claims_a_flag_its_parser_rejects,
         t_every_published_script_reports_every_flag_it_accepts,
         t_a_registrar_is_resolved_per_FUNCTION_not_per_module,
         t_a_registrar_that_also_owns_a_cli_is_still_followed,
         t_a_short_alias_does_not_hide_the_long_option,
         t_a_script_still_does_NOT_inherit_another_CLIs_flags,
         t_a_computed_flag_name_is_a_known_limit_not_a_silent_gap)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(fn.__name__)
        fn()
    print('\nALL PASS')
