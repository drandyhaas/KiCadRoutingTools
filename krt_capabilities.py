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
    python3 krt_capabilities.py --require route.py:--rip-existing-nets check_drc.py
    python3 route.py --capabilities             # same JSON, from the router

`--require` takes `module` or `module:--flag` tokens and exits non-zero listing
everything missing, so a consumer's check is one line and its failure message
names the gap instead of the symptom.
"""
import argparse
import ast
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
    # The two pre-route placement instruments. `check_channels.py` is the
    # per-face lane ledger the placement skill tells an operator to run before
    # blaming the router; `check_capacity.py` is the only tool that answers
    # "would more copper layers help" (#700) and was in no capability list and
    # no .md file anywhere -- an instrument nobody can discover produces no
    # findings.
    'check_channels.py', 'check_capacity.py',
)

# Scripts whose flag set a consumer may want to pin.
FLAG_SCRIPTS = ('route.py', 'route_diff.py', 'route_planes.py',
                'repair_planes.py', 'place_route_loop.py',
                'place_optimize.py', 'check_drc.py', 'check_floorplan.py')

# The long option, whether or not a SHORT one is declared before it. 48 call
# sites in this repo spell `add_argument('-q', '--quiet', ...)`, and requiring
# the long form to come first reported every one of them as unsupported --
# `check_floorplan.py:--quiet` was the last flag still missing once #798's
# registrar resolution landed. A third under-reporting mechanism, in the
# cheapest possible place.
_FLAG_RE = re.compile(
    r'add_argument\(\s*(?:["\']-[A-Za-z0-9]["\']\s*,\s*)?'
    r'["\'](--[A-Za-z0-9][A-Za-z0-9-]*)["\']')
# local `import x` / `from x import ...` -- the shared-registrar hop in script_flags
_IMPORT_RE = re.compile(r'^\s*(?:from|import)\s+([a-z_][a-z0-9_]*)', re.M)
# The same, DOTTED, for the per-function hop (#798). Kept separate rather than
# widening the one above: the module-level hop's `<mod>.py`-beside-the-script
# rule is what keeps it from wandering, and `placement.cli_gates` resolving to
# the 0-byte `py_placer/placement/__init__.py` is precisely the bug -- the hop
# was TAKEN, into an empty file, rather than skipped.
_DOTTED_IMPORT_RE = re.compile(
    r'^\s*(?:from|import)\s+([A-Za-z_][A-Za-z0-9_.]*)', re.M)


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


def _module_candidates(root, dotted):
    """Every file a dotted import could resolve to, across the #522 layout.

    `_TOOL_DIRS` already encodes that a tool may live in py_router/, py_tools/
    or py_placer/, and the registrar hop needs the same map because the scripts
    reach across it at runtime: `py_tools/_path.py` puts ../py_router and
    ../py_placer on sys.path, which is how `py_tools/check_floorplan.py`
    imports `placement.cli_gates` out of py_placer/. A hop rooted at the
    SCRIPT's own directory could never resolve that, whatever the regex did.
    """
    parts = dotted.split('.')
    out = []
    for d in _TOOL_DIRS:
        base = os.path.join(root, d, *parts)
        out.append(base + '.py')
        out.append(os.path.join(base, '__init__.py'))
    return out


def _registrar_functions(path):
    """``{function_name: {flag, ...}}`` for each REGISTRAR function in `path`.

    A registrar function adds arguments to a parser it was HANDED -- directly,
    or through a group derived from it -- and never constructs an
    `ArgumentParser` of its own. That is `_builds_own_parser`'s structural
    discriminator moved down one level, and the move is the whole fix (#798),
    because BOTH of the ways the module-level rule got the answer wrong are
    module-shaped:

      * `placement/cli_gates.py` holds FOUR registrars and each placement CLI
        calls a different subset. Unioning the module credits
        `place_portfolio.py` with `--suggest-locks`, which argparse rejects --
        a FALSE POSITIVE, the direction this module exists to prevent.
      * `fix_kicad_drc_settings.py` registers `--keep-thermal`,
        `--enable-used-layers` and `--no-fix-drc-settings` into route.py's
        parser via `add_drc_fix_args(parser)`, and owns a CLI as well. The
        module-level veto therefore skipped it entirely, so route.py
        under-reported three flags it accepts.

    Group-derived locals are followed (`g = parser.add_argument_group(...)`,
    then `g.add_argument(...)`) because that is exactly how `add_drc_fix_args`
    is written; a resolver that missed it would fix the placement half only.
    """
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            tree = ast.parse(f.read())
    except (OSError, SyntaxError, ValueError):
        return {}
    out = {}
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        holders = {a.arg for a in node.args.args}
        holders |= {a.arg for a in node.args.kwonlyargs}
        if not holders:
            continue
        if any(isinstance(c, ast.Call)
               and (getattr(c.func, 'id', '') == 'ArgumentParser'
                    or getattr(c.func, 'attr', '') == 'ArgumentParser')
               for c in ast.walk(node)):
            continue                  # a CLI's own builder, not a registrar
        for _ in range(3):            # `g = p.add_argument_group()` chains
            grew = False
            for c in ast.walk(node):
                if not (isinstance(c, ast.Assign)
                        and isinstance(c.value, ast.Call)):
                    continue
                fn = c.value.func
                if getattr(fn, 'attr', '') not in (
                        'add_argument_group', 'add_mutually_exclusive_group'):
                    continue
                if getattr(getattr(fn, 'value', None), 'id', None) not in holders:
                    continue
                for tgt in c.targets:
                    if isinstance(tgt, ast.Name) and tgt.id not in holders:
                        holders.add(tgt.id)
                        grew = True
            if not grew:
                break
        flags = set()
        for c in ast.walk(node):
            if not (isinstance(c, ast.Call)
                    and getattr(c.func, 'attr', '') == 'add_argument'):
                continue
            if getattr(getattr(c.func, 'value', None), 'id', None) not in holders:
                continue
            for a in c.args:
                if (isinstance(a, ast.Constant) and isinstance(a.value, str)
                        and a.value.startswith('--')):
                    flags.add(a.value)
        if flags:
            out[node.name] = flags
    return out


def _called_names(src):
    """Every function name this source CALLS, bare or attribute-qualified.

    Both spellings are in use and both must resolve: route.py does
    `from fab_tiers import add_fab_tier_args` then `add_fab_tier_args(parser)`,
    while a module imported whole is called as `fab_tiers.add_fab_tier_args(p)`.
    """
    try:
        tree = ast.parse(src)
    except (SyntaxError, ValueError):
        return set()
    out = set()
    for c in ast.walk(tree):
        if isinstance(c, ast.Call):
            name = getattr(c.func, 'id', None) or getattr(c.func, 'attr', None)
            if name:
                out.add(name)
    return out


def _registrar_flags(path, root, src):
    """Flags from the registrar FUNCTIONS this script actually calls (#798).

    Strictly ADDITIVE over the module-level hop in `script_flags`, and that is
    what makes it safe to land: the module-level pass produces zero false
    positives today (measured over all eight FLAG_SCRIPTS against their real
    parsers), so unioning a second pass can only be wrong if the second pass
    itself over-reports. It cannot over-report by following another CLI,
    because a function that builds its own parser is not a registrar --
    checked structurally, and `route.py` exposes no parser-taking registrar at
    all, so `route_planes.py` cannot inherit its 97-flag vocabulary however
    this resolution changes.
    """
    called = _called_names(src)
    if not called:
        return set()
    me = os.path.abspath(path)
    flags = set()
    for dotted in sorted(set(_DOTTED_IMPORT_RE.findall(src))):
        for cand in _module_candidates(root, dotted):
            if not os.path.isfile(cand) or os.path.abspath(cand) == me:
                continue
            for name, fl in _registrar_functions(cand).items():
                if name in called:
                    flags |= fl
    return flags


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

    #798: that module-level rule still UNDER-reports, on all eight
    FLAG_SCRIPTS, by 1 to 18 flags each -- two mechanisms, both module-shaped,
    and `_registrar_flags` is the per-FUNCTION pass that answers them. It is
    unioned in rather than replacing anything, because the module-level pass
    is measured to produce zero false positives today and a strictly additive
    second pass can only be wrong if it over-reports on its own.
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
        # #798, the per-FUNCTION pass. Additive, and rooted at the REPO rather
        # than beside the script, because the layout the scripts import across
        # is the repo's, not the directory's.
        flags |= _registrar_flags(path, ROOT, src)
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
