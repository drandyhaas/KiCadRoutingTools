#!/usr/bin/env python3
"""A consumer must be able to prove this clone is the engine it pinned.

A board repository points an environment variable at a KiCadRoutingTools clone
and routes with it. The strongest check available to that consumer has been
"does route.py exist as a file" -- which passes for a clone that is on the wrong
branch, years old, or missing the module the chain depends on. The chain then
runs, prints green, and describes an engine the repo does not pin.

That is the failure a no-fallbacks rule exists to prevent, and nothing detects
it, because every other signal in the run is produced BY the thing whose
identity is in doubt.

`--capabilities` publishes the inventory; `--require` asserts against it and
exits non-zero naming the gap.
"""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from krt_capabilities import (capabilities, missing, script_flags,  # noqa: E402
                              _tool_path)


def _run(args, cwd=ROOT):
    return subprocess.run([sys.executable, '-X', 'utf8'] + args,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=cwd)


def test_the_inventory_is_json_and_complete():
    caps = capabilities()
    assert caps['schema'] == 1
    assert caps['is_git_clone'] is True, "this repo is a git clone"
    absent = [m for m, ok in caps['modules'].items() if not ok]
    assert not absent, f"modules missing from this clone: {absent}"
    print(f"  PASS: {len(caps['modules'])} modules present, inventory is JSON")


def test_flags_are_read_without_importing():
    """Asking 'can you do X' must not be able to trigger a side effect."""
    flags = script_flags(_tool_path(ROOT, 'route.py'))
    for f in ('--track-width', '--json-out', '--nets', '--rip-existing-nets'):
        assert f in flags, f"{f} not discovered in route.py"
    assert script_flags(os.path.join(ROOT, 'no_such_file.py')) == []
    print("  PASS: flags read from source, missing file is empty not fatal")


def test_require_passes_on_what_this_clone_has():
    r = _run([os.path.join(ROOT, 'krt_capabilities.py'), '--require',
              'route.py:--track-width', 'route.py:--json-out',
              'repair_planes.py:--plane-layers',
              'place_route_loop.py:--accept-cmd', 'check_floorplan.py'])
    assert r.returncode == 0, r.stderr
    print("  PASS: --require exits 0 when every token is satisfied")


def test_require_fails_loudly_and_names_every_gap():
    """The message must name what is missing -- a consumer chasing a routing
    symptom will never guess 'wrong branch' on its own."""
    r = _run([os.path.join(ROOT, 'krt_capabilities.py'), '--require',
              'route.py:--flag-that-does-not-exist', 'no_such_module.py'])
    assert r.returncode == 3, f"expected exit 3, got {r.returncode}"
    assert '--flag-that-does-not-exist' in r.stderr
    assert 'no_such_module.py' in r.stderr
    assert 'flag not supported' in r.stderr and 'module not present' in r.stderr, \
        "the two failure kinds must be distinguishable"
    print("  PASS: --require exits 3 and names both kinds of gap")


def test_missing_is_pure_and_testable():
    caps = {'modules': {'a.py': True, 'b.py': False}, 'flags': {'a.py': ['--x']}}
    assert missing(caps, ['a.py']) == []
    assert missing(caps, ['a.py:--x']) == []
    assert missing(caps, ['b.py']) == ['b.py (module not present)']
    assert missing(caps, ['a.py:--y']) == ['a.py --y (flag not supported)']
    print("  PASS: the gap computation is a pure function")


def test_route_py_answers_without_a_board():
    """`route.py --capabilities` must work with no positional argument: the
    question is asked before there is any trust in the clone, so requiring a
    board to ask it defeats the purpose."""
    r = _run([_tool_path(ROOT, 'route.py'), '--capabilities'])
    assert r.returncode == 0, r.stderr[-800:]
    caps = json.loads(r.stdout)
    assert caps['modules']['route.py'] is True
    assert '--json-out' in caps['flags']['route.py']
    print("  PASS: route.py --capabilities needs no input_file")



def test_a_flag_from_a_SHARED_registrar_is_not_reported_missing():
    """The dangerous direction is a false NEGATIVE: a consumer that trusts
    --require refuses to run against an engine that was fine all along.

    Two shapes produced one. `--fab-overrides` is registered by
    `fab_tiers.add_fab_args`, so a text scan of route.py's own source never saw
    it. And `qfn_fanout.py` is a thin shim over `qfn_fanout/__init__.py`, where
    its flags actually live -- and it was not in FLAG_SCRIPTS at all, so "never
    scanned" came back worded as "flag not supported"."""
    import krt_capabilities as k
    caps = k.capabilities()
    assert k.missing(caps, ['route.py:--fab-overrides']) == [],         "a flag from a shared argparse registrar must be found"
    assert k.missing(caps, ['qfn_fanout.py:--width']) == [],         "a package shim's flags live in <pkg>/__init__.py"
    # and a script outside the repo root, named by path
    assert k.missing(caps, [
        '.claude/skills/plan-pcb-routing/scripts/board_score.py:--net-min-widths'
    ]) == [], "a required script may be named by path"
    # a genuinely absent flag must still be reported
    assert k.missing(caps, ['route.py:--not-a-real-flag']),         "a real gap must still be reported"
    print("  PASS: shared registrars, package shims and paths all resolve")

def test_a_script_does_NOT_inherit_another_CLIs_flags():
    """The fix for false negatives manufactured false POSITIVES, which are worse.

    `script_flags` follows local imports one level to pick up shared registrars.
    Indiscriminately, that also follows `route_planes.py -> route.py` (and
    `-> check_drc.py`, `-> list_nets.py`), handing the plane scripts route.py's
    entire 97-flag vocabulary. Measured on a live run: `--require
    route_planes.py:--net-clearances` answered OK, the chain was written to use
    it, and argparse killed the step with exit 2 mid-run. A capability gate that
    passes for a flag the CLI rejects is the exact failure the module exists to
    prevent, inverted.

    The discriminator is structural: a REGISTRAR adds arguments to a parser
    somebody else owns; a CLI constructs its own ArgumentParser."""
    import krt_capabilities as k
    caps = k.capabilities()

    # These four are argparse errors on the real CLI -- verified by --help.
    for token in ('route_planes.py:--net-clearances',
                  'route_planes.py:--track-width-floor',
                  'repair_planes.py:--net-clearances',
                  'route_diff.py:--track-width-floor'):
        assert k.missing(caps, [token]), \
            f'{token} does not exist on that CLI and must be reported missing'

    # ...while every genuinely-shared flag still resolves. --fab-overrides comes
    # from the fab_tiers registrar (0 ArgumentParser, 2 add_argument), which is
    # exactly the hop that must survive.
    for token in ('route_planes.py:--fab-overrides',
                  'repair_planes.py:--plane-layers',
                  'repair_planes.py:--fab-overrides',
                  'route.py:--net-clearances',
                  'route_diff.py:--net-clearances',
                  'qfn_fanout.py:--escape-method'):
        assert k.missing(caps, [token]) == [], \
            f'{token} DOES exist on that CLI and must not be reported missing'
    print("  PASS: registrars are followed, other CLIs are not")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
