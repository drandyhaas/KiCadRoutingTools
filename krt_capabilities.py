#!/usr/bin/env python3
"""What this clone can actually do, as JSON.

A board repository pins a KiCadRoutingTools clone through an environment
variable and then routes with it. Today the strongest check available to that
consumer is "does route.py exist as a file" -- which passes for a clone that is
years old, on the wrong branch, or missing the module the chain depends on. The
run then completes, prints green, and describes an engine the repo does not
pin. That is the one failure a no-fallbacks rule exists to prevent, and nothing
detects it.

So: publish the capability set and let the consumer assert against it.

    python3 krt_capabilities.py                 # everything, as JSON
    python3 krt_capabilities.py --require route.py:--track-width-floor board_score
    python3 route.py --capabilities             # same JSON, from the router

`--require` takes `module` or `module:--flag` tokens and exits non-zero listing
everything missing, so a consumer's check is one line and its failure message
names the gap instead of the symptom.
"""
import argparse
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))

# The #522/placement-split layout spread the tools over py_router/, py_tools/
# and py_placer/. A consumer keeps naming them bare ('route.py'), so every
# lookup resolves through _tool_path, which accepts EITHER layout -- flat for
# an older clone, packaged for this one.
_TOOL_DIRS = ('', 'py_router', 'py_tools', 'py_placer')


def _tool_path(root, mod):
    """Absolute path to `mod`, wherever the #522/placement-split layout put it.

    Falls back to the flat join so the error message a missing module produces
    still names the place the caller expected it.
    """
    for d in _TOOL_DIRS:
        p = os.path.join(root, d, mod)
        if os.path.isfile(p):
            return p
    return os.path.join(root, mod)


# The modules a consumer is likely to depend on by name. Presence is reported
# for every one; absence is only an ERROR when --require asks for it.
# `route_disconnected_planes.py` was RENAMED to `repair_planes.py`; a consumer
# pinning the old name still gets an honest "module not present" from
# missing(), which looks up un-inventoried names itself.
KNOWN_MODULES = (
    'route.py', 'route_diff.py', 'route_planes.py', 'repair_planes.py',
    'place_optimize.py', 'place_route_loop.py', 'place_fanout_clearance.py',
    'bga_fanout.py', 'qfn_fanout.py',
    'check_drc.py', 'check_connected.py', 'check_floorplan.py',
    'check_impedance.py', 'check_orphan_stubs.py', 'check_pads.py',
    'kicad_unconnected.py', 'net_forensics.py', 'copy_board.py',
    'make_movie.py', 'render_placement.py', 'list_nets.py', 'route_summary.py',
)

# Scripts whose flag set a consumer may want to pin.
FLAG_SCRIPTS = ('route.py', 'route_diff.py', 'route_planes.py',
                'repair_planes.py', 'place_route_loop.py',
                'place_optimize.py', 'check_drc.py', 'check_floorplan.py')

_FLAG_RE = re.compile(r'add_argument\(\s*["\'](--[A-Za-z0-9][A-Za-z0-9-]*)["\']')
# local `import x` / `from x import ...` -- the shared-registrar hop in script_flags
_IMPORT_RE = re.compile(r'^\s*(?:from|import)\s+([a-z_][a-z0-9_]*)', re.M)


_PARSER_RE = re.compile(r'\bArgumentParser\s*\(')


def _builds_own_parser(path):
    """True if this module is a CLI in its own right, not a flag registrar.

    Its flags belong to ITS parser, so unioning them into an importer's flag set
    is a false positive -- see script_flags.
    """
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            return bool(_PARSER_RE.search(f.read()))
    except OSError:
        return True          # unreadable: assume the risky direction


def script_flags(path, _depth=1):
    """Long flags a script's argparse defines, including SHARED registrars.

    Read from the source rather than by importing and building the parser:
    importing runs module-level code, and a consumer asking "can this clone do
    X" must not be able to trigger a side effect by asking.

    Reading only the script's OWN text is not enough, and the failure is the
    dangerous direction -- a false NEGATIVE. Several flags are added by a shared
    helper in another module (`fab_tiers.add_fab_args` registers
    `--fab-tier`/`--fab-overrides` for route.py, route_diff.py, the fanouts and
    the plane scripts), so a text scan of route.py reports `--fab-overrides` as
    unsupported on a clone that supports it perfectly well -- and a consumer that
    trusts `--require` then refuses to run against a good engine. So follow the
    script's LOCAL imports one level and union their flags too.

    One level, and only modules resolving to a .py beside the script: enough for
    a registrar helper, and it cannot wander into the whole dependency graph.

    FOLLOW REGISTRARS, NEVER OTHER CLIs. That distinction is the whole
    correctness of this function, and getting it wrong inverts the tool's
    failure mode into the dangerous direction. `route_planes.py` imports
    `route.py` (and `check_drc.py`, and `list_nets.py`), so an indiscriminate
    hop hands route_planes route.py's entire 97-flag vocabulary -- and
    `--require route_planes.py:--net-clearances` then answers OK for a flag
    argparse rejects with exit 2. A consumer's chain dies mid-run on a
    capability check that passed.

    The discriminator is structural, not a name list: a REGISTRAR adds arguments
    to a parser somebody else owns (`fab_tiers.py`: 2 add_argument, 0
    ArgumentParser); a CLI builds its own (`route.py`: 97 add_argument, 1
    ArgumentParser). Only a module that never constructs an ArgumentParser can
    be contributing its flags to this script's parser.
    """
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            src = f.read()
    except OSError:
        return []
    flags = set(_FLAG_RE.findall(src))
    if _depth > 0:
        root = os.path.dirname(os.path.abspath(path)) or '.'
        me = os.path.abspath(path)
        for mod in sorted(set(_IMPORT_RE.findall(src))):
            # A module OR a package: `qfn_fanout.py` is a thin shim over
            # `qfn_fanout/__init__.py`, which is where its 40-odd flags live.
            # Checking only `<mod>.py` resolves back to the shim itself and
            # finds nothing.
            # `qfn_fanout.py` is a shim over `qfn_fanout/__init__.py`, which is
            # where its own parser and 40-odd flags live. That hop is the script
            # reaching its OWN implementation, not borrowing a second CLI's, so
            # it is allowed through the parser test.
            own_package = (mod == os.path.splitext(os.path.basename(path))[0])
            for sib in (os.path.join(root, mod + '.py'),
                        os.path.join(root, mod, '__init__.py')):
                if (os.path.isfile(sib) and os.path.abspath(sib) != me
                        and (own_package or not _builds_own_parser(sib))):
                    flags.update(script_flags(sib, _depth - 1))
    return sorted(flags)


def capabilities(root=ROOT):
    mods = {m: os.path.isfile(_tool_path(root, m)) for m in KNOWN_MODULES}
    flags = {s: script_flags(_tool_path(root, s))
             for s in FLAG_SCRIPTS if mods.get(s)}
    out = {
        'schema': 1,
        'root': root,
        'is_git_clone': os.path.exists(os.path.join(root, '.git')),
        'modules': mods,
        'flags': flags,
    }
    try:                                    # best-effort, never fatal
        _eng = os.path.join(root, 'py_router')
        if os.path.isdir(_eng) and _eng not in sys.path:
            sys.path.insert(0, _eng)        # #522 layout: the engine dir
        import routing_defaults as _d
        out['version'] = getattr(_d, 'VERSION', None)
    except Exception:
        out['version'] = None
    return out


def missing(caps, required):
    """Which `module` / `module:--flag` tokens this clone cannot satisfy.

    Scans whatever it is ASKED about rather than only what the inventory
    pre-scanned. `FLAG_SCRIPTS` is the inventory's list, and answering a
    question about a script outside it with "flag not supported" conflates
    *not scanned* with *not there* -- a false negative, which is the dangerous
    direction: a consumer that trusts `--require` then refuses to run against
    an engine that was fine. Measured: `qfn_fanout.py --width` reported
    unsupported on a clone where it has existed all along.

    A token may name a path (`.claude/skills/.../board_score.py:--flag`), so a
    script that does not sit at the repo root can be required too.
    """
    root = caps.get('root', ROOT)
    gaps = []
    for token in required:
        mod, _, flag = token.partition(':')
        path = _tool_path(root, mod)
        present = caps['modules'].get(mod)
        if present is None:                      # not in the inventory: look
            present = os.path.isfile(path)
        if not present:
            gaps.append(f"{mod} (module not present)")
            continue
        if flag:
            known = caps['flags'].get(mod)
            if known is None:                    # not pre-scanned: scan it now
                known = script_flags(path)
            if flag not in known:
                gaps.append(f"{mod} {flag} (flag not supported)")
    return gaps


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--require', nargs='+', metavar='TOKEN', default=None,
                    help="Assert these are available: `module` or `module:--flag`. "
                         "Exits 3 listing everything missing.")
    ap.add_argument('--quiet', '-q', action='store_true',
                    help='With --require, print nothing on success.')
    a = ap.parse_args(argv)

    caps = capabilities()
    if not a.require:
        json.dump(caps, sys.stdout, indent=1, sort_keys=True)
        sys.stdout.write('\n')
        return 0

    gaps = missing(caps, a.require)
    if gaps:
        print(f"KiCadRoutingTools clone at {caps['root']} cannot satisfy "
              f"{len(gaps)} requirement(s):", file=sys.stderr)
        for g in gaps:
            print(f"  - {g}", file=sys.stderr)
        print("This is not the engine you pinned. Check the branch and the "
              "environment variable rather than the routing result.",
              file=sys.stderr)
        return 3
    if not a.quiet:
        print(f"OK: {len(a.require)} requirement(s) satisfied by {caps['root']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
