#!/usr/bin/env python3
"""Would a tool go ahead with this recorded command line, or refuse it before
routing -- at its OWN argparse, or in a check its main makes after parsing?

A recorded manifest keeps every command the agent ran, including the ones the
tool REJECTED at argument parsing -- hexberry_fpga's `qfn_fanout.py ...
--dry-run` (qfn_fanout has no --dry-run: argparse error, exit 2, no board).
The manifest records no exit status, and the pruner keeps a command by the
boards it names, so a rejected command that names an output a later step
reads is KEPT, and manifest_to_plan used to convert it into a plan step that
runs, and succeeds, in the GUI -- where the CLI ran nothing at all.

`rejection(argv)` answers with the tool's real parser, not a copy of its flag
list: the parser is captured in a CHILD process (it is built inside each
tool's `main()` or `__main__` block, whose side effects -- the redo-manifest
recorder, cli_banner's exit echo -- must not run in
the caller), at its first `ArgumentParser.parse_args` call, which is then
handed each command line in turn. One child per tool, kept for the life of
the calling process, so converting the whole corpus costs one parser build
per tool rather than one per command.

A main can also refuse AFTER parsing (route_planes' nets/layers count,
route.py's --force-reroute with no scope, a missing --fab-overrides file).
The child judges those too, from the parsed arguments: see POST_PARSE and
REFUSAL_SITES below, and why that is a hand-kept inventory rather than a run
of the main.

`parsed_args(argv)` hands back the parsed namespace itself, through the
same child: lineage_scope.py reads what each step SCOPES that way.

The parser is TODAY'S. A command recorded under an older commit that used a
flag removed since then reads as rejected here; that is what a replay of it
would do today too, and `rejection` says which flag in its reason. (Over the
corpus no KEPT command is refused either way: the guard is for the shape.)
"""
import atexit
import json
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: The tools manifest_to_plan converts, by basename -> path under the repo.
TOOL_PATHS = {
    'route.py': 'py_router/route.py',
    'route_diff.py': 'py_router/route_diff.py',
    'route_planes.py': 'py_router/route_planes.py',
    'repair_planes.py': 'py_router/repair_planes.py',
    'route_disconnected_planes.py': 'py_router/repair_planes.py',
    'bga_fanout.py': 'py_router/bga_fanout.py',
    'qfn_fanout.py': 'py_router/qfn_fanout.py',
    'place_fanout_clearance.py': 'py_placer/place_fanout_clearance.py',
}

_CHILD = r'''
import argparse, contextlib, io, json, os, runpy, sys
tool, tool_name, here = sys.argv[1], sys.argv[2], sys.argv[3]
real = argparse.ArgumentParser.parse_args
class _Got(BaseException):
    pass
got = {}
def capture(self, args=None, namespace=None):
    got['p'] = self
    raise _Got()
argparse.ArgumentParser.parse_args = capture
sys.argv = [tool]
sys.path.insert(0, os.path.dirname(os.path.abspath(tool)))
# Only the parser is wanted, so a missing Rust router (a checkout or a test
# copy without a built grid_router) must not stop the tool before it builds
# one: its presence check is skipped and, when the module really is absent, a
# stand-in whose classes refuse to be made satisfies the module-level imports.
# Nothing here routes.
try:
    import startup_checks
    startup_checks.check_rust_library = lambda: 'parser-probe'
except Exception:
    pass
sys.path.insert(1, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(tool))), 'rust_router'))
try:
    import grid_router  # noqa: F401  (the real one, where it is built)
except ImportError:
    import types
    class _Absent:
        def __init__(self, *a, **k):
            raise RuntimeError('grid_router stand-in: a parser probe never routes')
    def _absent_attr(name):
        if name.startswith('__'):
            raise AttributeError(name)
        return _Absent
    _gr = types.ModuleType('grid_router')
    _gr.__getattr__ = _absent_attr
    _gr.__version__ = 'parser-probe'
    sys.modules['grid_router'] = _gr
out = sys.stdout
try:
    with contextlib.redirect_stdout(io.StringIO()):
        runpy.run_path(tool, run_name='__main__')
except _Got:
    pass
except BaseException as e:
    pass
argparse.ArgumentParser.parse_args = real
p = got.get('p')
out.write('##READY %s\n' % ('ok' if p is not None else 'none'))
out.flush()
if p is None:
    os._exit(0)
sys.path.insert(0, here)
import cli_argv_check
for line in sys.stdin:
    req = json.loads(line)
    args, cwd = req['args'], req.get('cwd')
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
            ns = p.parse_args(args)
            why = cli_argv_check.post_parse_rejection(tool_name, ns, cwd)
        reply = {'ok': True} if not why else {'ok': False, 'code': 2,
                                              'why': why, 'post': True}
        if reply['ok'] and req.get('ns'):
            # parsed_args(): the namespace itself, every JSON-able value.
            ns_out = {}
            for k, v in vars(ns).items():
                try:
                    json.dumps(v)
                except (TypeError, ValueError):
                    continue
                ns_out[k] = v
            reply['ns'] = ns_out
    except SystemExit as e:
        msg = [l for l in err.getvalue().splitlines() if l.strip()]
        reply = {'ok': False, 'code': e.code,
                 'why': msg[-1] if msg else 'exited %r while parsing' % (e.code,)}
    except Exception as e:
        reply = {'ok': False, 'code': None,
                 'why': '%s: %s' % (type(e).__name__, e)}
    out.write('##R ' + json.dumps(reply) + '\n')
    out.flush()
os._exit(0)
'''

# ---------------------------------------------------------------- post-parse
# A main can refuse AFTER parse_args too: `parser.error(...)`, `sys.exit(2)`,
# a non-zero return. Running the real main up to the board in the child would
# let the tool decide for itself, but it is not clean here: route.py reads the
# input board's project and copper count, and then parses the board, BEFORE
# most of its argument checks (--keep-away, --force-reroute, --group), so a
# stop at the first board read never reaches them, and going further means
# parsing every recorded board at conversion time. So the checks that are a
# function of the arguments -- and of a file the command NAMES -- are made here,
# in each main's own order, through the tool's own helper wherever there is
# one. REFUSAL_SITES is the inventory of every post-parse refusal in those
# mains; tests/gui_parity/test_manifest_plan_parity.py holds it complete by
# AST, so a new refusal fails the gate until it is classified here.

def _at(cwd, path):
    """`path` as the command opened it (relative to its recorded cwd)."""
    return path if os.path.isabs(path) or not cwd else os.path.join(cwd, path)


def _can_see(cwd):
    """A named file can be judged only where the command ran: if its folder is
    not here (another machine's manifest), a missing file says nothing."""
    return not cwd or os.path.isdir(cwd)


def _output_twice(ns, cwd):
    out, pos = getattr(ns, 'output', None), getattr(ns, 'output_file', None)
    if out is not None and pos is not None and pos != out:
        return "a positional output and --output were both given, and differ"


def _fab_overrides_missing(ns, cwd):
    path = getattr(ns, 'fab_overrides', None)
    if path and _can_see(cwd) and not os.path.isfile(_at(cwd, path)):
        return f"--fab-overrides file not found: {path}"


def _net_clearances_unreadable(ns, cwd):
    path = getattr(ns, 'net_clearances', None)
    if not path or not _can_see(cwd):
        return None
    from list_nets import load_net_clearances
    try:
        load_net_clearances(_at(cwd, path))
    except (OSError, ValueError) as e:
        return f"--net-clearances: {e}"


def _keep_away(ns, cwd):
    from keep_away import keep_away_knob_error, parse_keep_away_rules
    if getattr(ns, 'keep_away', None):
        try:
            parse_keep_away_rules(ns.keep_away)
        except ValueError as e:
            return f"--keep-away: {e}"
    return keep_away_knob_error(ns.keep_away_free, ns.keep_away_cost)


def _route_group_empty(ns, cwd):
    if ns.group is not None and not ns.group.strip():
        return "--group was given an empty name"


def _route_force_reroute_unscoped(ns, cwd):
    patterns = [p.strip() for p in list(ns.net_patterns or []) + list(ns.nets or [])
                if p.strip()]
    if ns.force_reroute and not patterns and not list(ns.component or []):
        return "--force-reroute requires an explicit net scope"


def _diff_force_reroute_unscoped(ns, cwd):
    patterns = [p.strip() for p in list(ns.net_patterns or []) + list(ns.nets or [])
                if p.strip()]
    if ns.force_reroute and not patterns:
        return "--force-reroute requires an explicit net scope"


def _route_group_sources(ns, cwd):
    if not ns.group:
        return None
    try:
        __import__('_placer_path')
    except Exception:
        pass
    from placement.groups import GroupError, parse_sources
    try:
        parse_sources(ns.group_by)
    except GroupError as e:
        return f"--group-by: {e}"


def _planes_count_mismatch(ns, cwd):
    if len(ns.nets) != len(ns.plane_layers):
        return (f"number of net arguments ({len(ns.nets)}) must match number "
                f"of plane layers ({len(ns.plane_layers)})")


def _power_nets_count_mismatch(ns, cwd):
    pn = list(getattr(ns, 'power_nets', None) or [])
    pw = list(getattr(ns, 'power_nets_widths', None) or [])
    if len(pn) != len(pw):
        return (f"--power-nets ({len(pn)}) and --power-nets-widths ({len(pw)}) "
                f"must have the same length")


def _route_power_nets_count_mismatch(ns, cwd):
    # route.py: widths may be omitted (the engine then sizes the power nets
    # itself); given, they must pair one to one with the nets (#1410).
    pn = list(getattr(ns, 'power_nets', None) or [])
    pw = list(getattr(ns, 'power_nets_widths', None) or [])
    if pn and pw and len(pn) != len(pw):
        return (f"--power-nets ({len(pn)}) and --power-nets-widths ({len(pw)}) "
                f"must have the same length")


def _repair_count_mismatch(ns, cwd):
    if ns.nets and ns.plane_layers and len(ns.nets) != len(ns.plane_layers):
        return "when both --nets and --plane-layers are given, counts must match"


def _cap_clearance_not_finite(ns, cwd):
    import math
    if ns.clearance is not None and not math.isfinite(ns.clearance):
        return f"--clearance must be a finite number, got {ns.clearance!r}"


def _cap_intent_unreadable(ns, cwd):
    if not getattr(ns, 'intent', None) or not _can_see(cwd):
        return None
    from placement import floorplan
    here = os.getcwd()
    try:
        if cwd:
            os.chdir(cwd)
        try:
            intent = floorplan.load_intent(ns.intent)
        except (OSError, ValueError) as e:
            return f"cannot load intent {ns.intent}: {e}"
        try:
            floorplan.tether_gate_spec(intent)
        except (TypeError, ValueError) as e:
            return f"intent {ns.intent}: its decap limits do not read as numbers: {e}"
    finally:
        os.chdir(here)


#: tool -> its argument-judged post-parse checks, in its main's order.
POST_PARSE = {
    'route.py': (_fab_overrides_missing, _output_twice, _route_group_empty,
                 _keep_away, _route_force_reroute_unscoped, _route_group_sources,
                 _route_power_nets_count_mismatch, _net_clearances_unreadable),
    'route_diff.py': (_keep_away, _fab_overrides_missing, _output_twice,
                      _diff_force_reroute_unscoped, _net_clearances_unreadable),
    'route_planes.py': (_fab_overrides_missing, _output_twice,
                        _planes_count_mismatch, _net_clearances_unreadable),
    'repair_planes.py': (_fab_overrides_missing, _output_twice,
                         _repair_count_mismatch),
    'route_disconnected_planes.py': (_fab_overrides_missing, _output_twice,
                                     _repair_count_mismatch),
    'bga_fanout.py': (_fab_overrides_missing,),
    'qfn_fanout.py': (_power_nets_count_mismatch, _fab_overrides_missing),
    'place_fanout_clearance.py': (_cap_clearance_not_finite,
                                  _cap_intent_unreadable),
}

#: Every post-parse refusal site in each tool's main, by fingerprint -- a
#: parser.error's message head, else the source of the `if` that guards the
#: exit/return, else `call:<helper>` for a helper that exits -- with how this
#: module treats it. `argv`/`file`: judged here by the named POST_PARSE check.
#: `board`: depends on the board's (or its folder's) contents, so it cannot be
#: judged from the command -- out of scope. `never a step`: the converter
#: refuses the whole command anyway. `after the run`: the tool ran.
REFUSAL_SITES = {
    'route.py': {
        'call:fab_tier_from_args': ('file', '_fab_overrides_missing'),
        'specify the output path once: positional': ('argv', '_output_twice'),
        'except GroupError from parse_sources':
            ('never a step', 'under --list-groups, which is refused'),
        'if args.group is not None and (not args.group.strip())':
            ('argv', '_route_group_empty'),
        '--keep-away: ': ('argv', '_keep_away'),
        'call:read_brief_keep_away': (
            'board', "reads the design brief BESIDE the input board -- not a "
                     "file the command names, and its record-time contents "
                     "are not in the manifest"),
        'except ValueError from read_brief_keep_away': (
            'board', "the design brief's keep_away (as above)"),
        'if _kae': ('argv', '_keep_away'),
        '--force-reroute requires an explicit net':
            ('argv', '_route_force_reroute_unscoped'),
        '--power-nets (': ('argv', '_route_power_nets_count_mismatch'),
        '--component matched no footprint on this':
            ('board', "the board's footprint references"),
        # parse_sources is the argument half; block_refs, in the same try,
        # needs the board's parts (out of scope).
        'except (GroupError, GroupRoutingError) from parse_sources':
            ('argv', '_route_group_sources'),
        'if not net_names': ('board', "which nets the patterns match"),
        'if not resolve_net_ids(pcb_data, net_names)':
            ('board', "the board's net names"),
        'if _scope_defaulted': ('never a step', 'under --undo, which is refused'),
        'call:finish_design_rules':
            ('after the run', '--strict-sizes: exit 3 after routing'),
        'if _rc857': ('after the run', '--strict-sizes: exit 3 after routing'),
        'call:load_net_clearances': ('file', '_net_clearances_unreadable'),
    },
    'route_diff.py': {
        '--force-reroute requires an explicit net':
            ('argv', '_diff_force_reroute_unscoped'),
        '--keep-away: ': ('argv', '_keep_away'),
        'call:read_brief_keep_away': (
            'board', "reads the design brief beside the input board"),
        'except ValueError from read_brief_keep_away': (
            'board', "the design brief's keep_away"),
        'if _kae': ('argv', '_keep_away'),
        'call:fab_tier_from_args': ('file', '_fab_overrides_missing'),
        'specify the output path once: positional': ('argv', '_output_twice'),
        'if not net_names': ('board', "which nets the patterns match"),
        'call:load_net_clearances': ('file', '_net_clearances_unreadable'),
        'if _NO_PAIRS_MATCHED': ('board', "set by the engine: no pair matched"),
        'call:finish_design_rules': ('after the run', '--strict-sizes: exit 3 after routing'),
        'if _rc857': ('after the run', '--strict-sizes: exit 3 after routing'),
    },
    'route_planes.py': {
        'call:fab_tier_from_args': ('file', '_fab_overrides_missing'),
        'both a positional output and --output we': ('argv', '_output_twice'),
        'number of net arguments (': ('argv', '_planes_count_mismatch'),
        'call:load_net_clearances': ('file', '_net_clearances_unreadable'),
        'call:finish_design_rules': ('after the run', '--strict-sizes: exit 3 after routing'),
    },
    'repair_planes.py': {
        'call:fab_tier_from_args': ('file', '_fab_overrides_missing'),
        'both a positional output and --output we': ('argv', '_output_twice'),
        'if len(args.nets) != len(args.plane_layers)':
            ('argv', '_repair_count_mismatch'),
        'if not zone_pairs': ('board', "zones auto-detected from the board"),
        'if LAST_RIPPED_STILL_OPEN':
            ('after the run', 'rip casualties ship open: exit 4'),
        'call:finish_design_rules': ('after the run', '--strict-sizes: exit 3 after routing'),
    },
    'bga_fanout.py': {
        'call:fab_tier_from_args': ('file', '_fab_overrides_missing'),
        'if bga_components': ('board', "auto-detects the BGA on the board"),
        'if args.component not in pcb_data.footprints':
            ('board', "the board's footprint references"),
        'if _stagger_why and (not env_knobs.ALLOW_STAGGERED_BGA)':
            ('board', "the part's pad lattice (#500)"),
        'call:finish_design_rules': ('after the run', '--strict-sizes: exit 3 after routing'),
    },
    'qfn_fanout.py': {
        '--power-nets (': ('argv', '_power_nets_count_mismatch'),
        'call:fab_tier_from_args': ('file', '_fab_overrides_missing'),
        'if qfn_components': ('board', "auto-detects the QFN on the board"),
        'if args.component not in pcb_data.footprints':
            ('board', "the board's footprint references"),
        'if _single': ('board', "the part's pads (#1195: not a QFN/QFP)"),
        'call:finish_design_rules': ('after the run', '--strict-sizes: exit 3 after routing'),
    },
    'place_fanout_clearance.py': {
        '--clearance must be a finite number, got':
            ('argv', '_cap_clearance_not_finite'),
        'call:load_intent_or_exit': ('file', '_cap_intent_unreadable'),
        'if _rc': ('file', '_cap_intent_unreadable'),
        'call:tether_gate_spec': ('file', '_cap_intent_unreadable'),
        'except (TypeError, ValueError) from _fp1067.tether_gate_spec':
            ('file', '_cap_intent_unreadable'),
        'call:declared_cap_rotations': (
            'board', "an intent's declared rotations, resolved against the "
                     "board's parts (#1122)"),
        'except _fp1122.IntentError from declared_cap_rotations': (
            'board', "the same (#1122)"),
    },
}


def post_parse_rejection(tool, ns, cwd=None):
    """Why `tool`'s main would refuse these PARSED arguments before routing,
    or None -- the argument- and named-file checks of REFUSAL_SITES."""
    for check in POST_PARSE.get(tool, ()):
        try:
            why = check(ns, cwd)
        except Exception:
            why = None          # a check that cannot run cannot refuse
        if why:
            return why
    return None


_children = {}


def _child(tool):
    """The live checker for one tool, started on first use; None when its
    parser could not be captured (the caller then falls back)."""
    rel = TOOL_PATHS[tool]
    if tool in _children:
        return _children[tool]
    env = dict(os.environ)
    env.pop('REDO_MANIFEST', None)    # the recorder must not log the probe
    proc = subprocess.Popen(
        [sys.executable, '-c', _CHILD, os.path.join(REPO, rel), tool,
         os.path.dirname(os.path.abspath(__file__))],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL, text=True, cwd=REPO, env=env)
    ready = None
    for line in proc.stdout:
        if line.startswith('##READY'):
            ready = line.split()[1]
            break
    if ready != 'ok':
        proc.kill()
        proc = None
    _children[tool] = proc
    return proc


def _close():
    for proc in _children.values():
        if proc is not None:
            try:
                proc.stdin.close()
                proc.wait(timeout=5)
            except Exception:
                proc.kill()


atexit.register(_close)


def tool_args(argv):
    """(tool basename, the arguments after it) for a recorded command whose
    tool this module can check, else None."""
    for k, a in enumerate(argv):
        base = os.path.basename(a)
        if base in TOOL_PATHS:
            return base, list(argv[k + 1:])
    return None


def _flag_list_rejection(tool, args):
    """The fallback when the tool's parser cannot be built here (a missing
    dependency in this interpreter): its long flags as krt_capabilities reads
    them from the source -- held exact against the real parsers by
    tests/test_798_registrar_flags.py -- with
    argparse's own rule that an
    unambiguous prefix of a long option is that option. It sees an unknown
    flag (the --dry-run class), not a bad value or a missing argument."""
    try:
        sys.path.insert(0, REPO)
        import krt_capabilities as caps
        flags = set(caps.script_flags(caps._tool_path(caps.ROOT, tool)))
    except Exception:
        return None
    if len(flags) < 5:        # read nothing: nothing to refuse on
        return None
    for a in args:
        if not a.startswith('--') or a == '--':
            continue
        name = a.split('=', 1)[0]
        if name in flags or sum(f.startswith(name) for f in flags) == 1:
            continue
        return (f"{tool} rejects its arguments: unrecognized argument {name} "
                f"(its parser could not be built here; checked against the "
                f"tool's flag list)")
    return None


def rejection(argv, cwd=None):
    """Why the recorded command's tool would refuse it before routing -- its
    argparse's own message, or a post-parse check of REFUSAL_SITES -- or None
    when it would go ahead, or when the tool is not one this module checks.
    `cwd` is the folder the command ran in, for a file it names. Where the
    parser cannot be built, the tool's flag list stands in
    (`_flag_list_rejection`), and the post-parse checks are not made."""
    found = tool_args(argv)
    if found is None:
        return None
    tool, args = found
    reply = _ask(tool, args, cwd)
    if reply is _NO_PARSER:
        return _flag_list_rejection(tool, args)
    if reply is None or reply['ok']:
        return None
    if reply.get('code') == 0:
        return f"{tool} prints and exits at parse time ({reply['why']})"
    if reply.get('post'):
        return f"{tool} refuses its arguments after parsing: {reply['why']}"
    return f"{tool} rejects its arguments: {reply['why']}"


_NO_PARSER = object()


def _ask(tool, args, cwd, want_ns=False):
    """The child's reply to one command line, `_NO_PARSER` when the tool's
    parser could not be built here, or None when the child said nothing."""
    proc = _child(tool)
    if proc is None:
        return _NO_PARSER
    proc.stdin.write(json.dumps({'args': args, 'cwd': cwd, 'ns': want_ns}) + '\n')
    proc.stdin.flush()
    for line in proc.stdout:
        if line.startswith('##R '):
            return json.loads(line[4:])
    return None


def parsed_args(argv, cwd=None):
    """What the recorded command's tool PARSED it into, by its real parser.

    Returns None when the tool is not one this module checks or its parser
    cannot be built here; otherwise a dict with `tool`, `ok` and, when the
    tool would go ahead, `ns` = {dest: value} (every JSON-able value of the
    argparse namespace), else `why` it refuses. This is how a reader learns
    what a step SCOPES (its positional net patterns, --nets, --component ...)
    without copying a parser: a hand-rolled split cannot tell a positional
    net from the value of a flag it does not know takes one."""
    found = tool_args(argv)
    if found is None:
        return None
    tool, args = found
    reply = _ask(tool, args, cwd, want_ns=True)
    if reply is _NO_PARSER or reply is None:
        return None
    out = {'tool': tool, 'ok': bool(reply.get('ok'))}
    if out['ok']:
        out['ns'] = reply.get('ns') or {}
    else:
        out['why'] = reply.get('why')
    return out


if __name__ == '__main__':
    print(rejection(sys.argv[1:], cwd=os.getcwd()) or 'accepted')
