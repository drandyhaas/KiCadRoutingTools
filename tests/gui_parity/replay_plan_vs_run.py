#!/usr/bin/env python3
"""Replay a stress run's GUI plan through the REAL plugin and diff it against
the CLI corpus board.

Every stress board leaves a GUI-loadable plan next to its chain
(`run_board.sh` -> `manifest_to_plan.py` -> `<board>_plan.json`). This driver
does, with no buttons and no LLM, exactly what a user does with it:

    open the UNROUTED input board -> AI tab -> Load... -> Run All Selected

and then grades the result against the CLI chain the corpus run recorded.

WHAT MAKES THIS DIFFERENT FROM test_gui_engine_parity.py: that harness
hand-mirrors the plan->params mapping onto shimmed dialog objects, so it proves
the ENGINE half only (same batch_route kwargs -> same board) and is blind to
bugs in `manifest_to_plan` (converter) or `ai_plan.apply_step_params`
(apply) -- the two layers that silently diverged in the set11 rp2350 replay
(#361). Here the REAL `swig_gui.RoutingDialog` is constructed headless (parent
None, no Show()) with its REAL tabs and controls, and the REAL
`ai_plan.PlanExecutor` drives it inside a wx MainLoop -- the same
parse_plan_result -> reset_params_to_defaults -> apply_step_params ->
apply_step_selection -> tab._on_*() path the buttons run. Nothing is mirrored,
so a converter/apply bug shows up as a board difference.

THE TWO LEGS RUN ON DIFFERENT PYTHONS, ON PURPOSE. The GUI leg needs pcbnew so
it runs under KiCad's bundled python (3.9 here); the CLI leg runs the manifest's
own `python3` (3.14 here). Routing is supposed to be interpreter-independent, so
this doubles as a check on that -- and it caught a case where it was not: Python
3.12 switched sum() to compensated summation, bga_fanout's cluster means used
it, and eth_tap's U13 fanout emitted 109 segments on 3.14 vs 110 on 3.9 from
byte-identical inputs (fixed in grid.py with math.fsum). If copper diverges,
check the interpreters before blaming the GUI.

THE REFERENCE IS RE-RUN AT HEAD, NOT READ OFF DISK. The recorded corpus boards
were routed by whatever commit the run happened on; diffing a GUI replay at HEAD
against them measures the ENGINE'S drift since that date, not the GUI/CLI gap,
and the drift dwarfs it. Measured on nano_eeprom_prog: the recorded (2026-07-09)
board has 419 segments, the SAME manifest replayed at HEAD has 570 -- so the
recorded board reports a 466-segment "divergence" that is entirely the router
having moved on. So by default this replays the manifest's pruned chain into a
fresh dir with `redo_stress_test.py` (same pruning, same commands, current code)
and compares against THAT; the recorded board is still graded alongside, as
context. `--cli-ref recorded` opts out and is only honest when HEAD is the
commit the run was recorded at.

Comparison is at three levels, finals AND per step:
  1. GRADE   -- router-attributable DRC (baseline-subtracted, at the chain's
                recorded floor) + connectivity, via the SHARED grading core
                (kicad_drc_compare.compare_board_data) the corpus itself uses,
                so the numbers are comparable by construction. Both sides are
                graded fresh at HEAD (the grader has tightened over time too;
                comparing a fresh grade to a stale one manufactures deltas).
  2. COPPER  -- segments and vias as canonical UUID-independent MULTISETS
                (net name, layer, unordered endpoints, width / net, x, y, size,
                drill, span). EXACT to the nanometre by default: .kicad_pcb
                outputs carry per-run random UUIDs so byte-diffing is
                meaningless, but the geometry itself is reproducible and the two
                fronts are expected to agree exactly. Anything left over is also
                near-matched within --tol, to separate a sub-micron
                representational difference from genuinely different routing.
  3. STATE   -- zones (net/layer/priority/outline) and footprint poses
                (optimize_caps moves parts; the #362 position-sync class).

Per-step localization: the plan's steps are paired 1:1 with the CLI chain's
intermediate boards (re-deriving `manifest_to_plan`'s own pruning in-process to
recover each step's output file), the GUI board is snapshotted after every step,
and the FIRST diverging step is reported -- which is the number you actually
want when a full-chain final differs.

Usage:
    python3 tests/gui_parity/replay_plan_vs_run.py <rundir> [options]
    python3 tests/gui_parity/replay_plan_vs_run.py --set <runs_setN> [options]

    <rundir>            a stress run dir, e.g. ~/Documents/kicad_stress_test/
                        runs_set10/eth_tap
    --set DIR           batch: every board dir under DIR, one subprocess each,
                        then a summary table
    --boards a,b,c      with --set: only these board dirs
    --workdir DIR       where the replay runs (default: tests/gui_parity/work/
                        replay_<board>); reused unless --fresh
    --cli-ref S         head (default) = replay the manifest at HEAD into
                        <workdir>/cli_head and compare against that; recorded =
                        compare against the boards already in the run dir (only
                        valid when HEAD is the commit that produced them)
    --plan-source S     saved (default) = the recorded <board>_plan.json, what
                        the GUI Load... button gets; regen = rebuild it from
                        redo_commands.sh at HEAD. A saved plan can be STALE if
                        the manifest was re-recorded later -- that is detected
                        and reported either way, and makes the comparison
                        apples-to-oranges (the GUI then runs a different step
                        list than the CLI reference); use regen in that case.
    --raw-plan          skip parse_plan_result's plan augmentation (the #479
                        late-pinch repair_planes guard the GUI appends), for a
                        strictly like-for-like step count vs the CLI chain
    --max-steps N       run only the first N plan steps (bisecting)
    --gui-board PATH    skip the replay; compare this already-replayed board
    --tol MM            near-match radius for classifying unmatched copper
                        (default 0.001 = 1 um). Never affects the verdict.
    --dp N              coordinate rounding decimals for the canonical multisets
                        (default 6 = 1 nm, the board's own resolution, i.e. an
                        exact comparison). Both fronts now emit copper through
                        kicad_parser.mm_to_iu, so GUI and CLI geometry is
                        bit-identical and no tolerance is needed; loosen it only
                        to characterise a divergence you are already chasing.
    --timeout SEC       hard cap on the replay (default 7200)
    --json OUT          write the full result dict as JSON
    --verbose           let engine output through instead of tee-ing it to
                        <workdir>/replay.log
    --fresh             wipe the workdir first

Exit codes: 0 = same routing (IDENTICAL, or EQUIVALENT = every difference pairs
within --tol), 1 = grade parity but copper genuinely differs, 2 = grade differs,
3 = replay failed / could not run.

Needs KiCad's python (pcbnew + wx); re-execs into it automatically. Grading
needs kicad-cli for the shared core and degrades to check_drc-only without it.
"""

# ---------------------------------------------------------------------------
# macOS: if this HANGS at ~0% CPU, it is NOT wx, machine load, or a deadlock.
#
# After any wx process here is killed (a pkill, a timeout, a crash), macOS
# decides the app "quit unexpectedly", and the NEXT headless launch stops inside
# NSApplication bootstrap showing the restore-windows alert you cannot see:
#     -[NSPersistentUIRestorer promptToIgnorePersistentState]
#         -> -[NSAlert runModal]
# Headless, nobody can click it, so it waits forever: process state SN accruing
# ~0.3s of CPU over many minutes, which reads exactly like a hang. This cost a
# full session of ".gui-parity-checked" markers recording "wx blocked, gate NOT
# RUN" -- the gates were fine the whole time.
#
#   diagnose:  sample <pid> 3 -mayDie | grep -E "NSAlert|PersistentUI"
#   fix:       defaults write -g ApplePersistenceIgnoreState -bool YES
#
# A sandboxed HOME does NOT help -- cfprefsd serves that pref per-user
# regardless of HOME. With the default set, test_gui_engine_parity.py runs ~90s.
# ---------------------------------------------------------------------------
import os
import sys

# wx debug builds assert on about_tab's wxEXPAND|wxALIGN_* sizer flags, which is
# harmless in the real plugin but fatal when constructing the dialog headless.
# Must be set before wx is imported (and before the re-exec, so the child sees it).
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

import argparse
import glob
import json
import re
import shutil
import subprocess
import time
from collections import Counter, defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STRESS_TESTS = os.path.join(REPO, 'tests', 'stress')
for _p in (REPO, STRESS_TESTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]


def _reexec_into_kicad():
    """Re-exec into a python that has pcbnew (the plugin needs it and wx)."""
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                # os.execv re-splits argv on spaces on Windows, and the
                # interpreter lives under "Program Files".
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("ERROR: no python with pcbnew+wx found (KiCad's bundled python).",
          file=sys.stderr)
    sys.exit(3)


# --------------------------------------------------------------- run resolution

def resolve_run(rundir):
    """Everything the replay + comparison needs, read off a recorded run dir.

    Reuses grade_final / ab_replay_grade helpers rather than reparsing the
    manifest a third way, so the input board, the final board and the grading
    clearance are resolved EXACTLY as the corpus graded them.
    """
    rundir = os.path.abspath(rundir)
    info = {'rundir': rundir, 'board': os.path.basename(rundir)}
    manifest = os.path.join(rundir, 'redo_commands.sh')
    if not os.path.exists(manifest):
        raise RuntimeError(f"no redo_commands.sh in {rundir}")
    info['manifest'] = manifest
    txt = open(manifest, encoding='utf-8', errors='replace').read()

    from grade_final import redo_final_and_clearance
    from ab_replay_grade import manifest_baseline
    cli_final_name, clearance = redo_final_and_clearance(txt)
    info['chain_clearance'] = clearance

    # The unrouted input board = the chain's baseline (also the #405 subtraction
    # baseline), resolved against the run dir and its sibling board dirs.
    base = manifest_baseline(txt, rundir, os.path.dirname(rundir),
                             os.path.dirname(os.path.dirname(rundir)))
    if base and not os.path.isabs(base):
        base = os.path.join(rundir, base)
    if not base or not os.path.exists(base):
        raise RuntimeError("could not resolve the unrouted input board from the manifest")
    info['input_board'] = base

    # The CLI final actually on disk. A re-recorded manifest can name a step that
    # was never run (eth_tap's step17_final_verify), so fall back the way
    # grade_final does: recorded grade -> newest final* -> newest board.
    cand = []
    if cli_final_name:
        cand.append(os.path.join(rundir, os.path.basename(cli_final_name)))
    grade_path = os.path.join(rundir, 'authoritative_grade.json')
    recorded = {}
    if os.path.exists(grade_path):
        try:
            recorded = json.load(open(grade_path))
        except Exception:
            recorded = {}
    if recorded.get('final_board'):
        cand.append(os.path.join(rundir, recorded['final_board']))
    boards = [f for f in os.listdir(rundir) if f.endswith('.kicad_pcb')]
    if boards:
        cand.append(os.path.join(rundir, max(
            boards, key=lambda f: os.path.getmtime(os.path.join(rundir, f)))))
    info['cli_final'] = next((c for c in cand if os.path.exists(c)), None)
    if not info['cli_final']:
        raise RuntimeError(f"no CLI final board found in {rundir}")
    info['recorded_grade'] = recorded
    return info


def _regen_steps(manifest):
    """manifest_to_plan's conversion in-process, KEEPING each step's `_files`
    (its CLI input/output boards) so plan steps can be paired with the chain's
    intermediate boards.

    This used to re-implement the converter's loop and had to be kept in step
    with it by hand -- exactly the drift this harness exists to catch. It now
    calls the converter's own function, so a change there cannot leave the
    harness comparing against a stale conversion."""
    from manifest_to_plan import plan_steps_from_manifest
    return plan_steps_from_manifest(manifest, keep_files=True)[0]


def load_plan(info, source='saved', augment=True):
    """Return (steps, cli_names, notes).

    `steps` are what the GUI would hold after Load...: the plan JSON put through
    the REAL parse_plan_result (which validates, inserts optimize_caps and
    appends the #479 late-pinch repair_planes guard) unless augment=False.
    `cli_names[i]` is the BASENAME of the CLI chain board plan step i
    corresponds to (or None when the step has no CLI counterpart). Names, not
    paths: the caller resolves them against whichever reference dir is in use,
    and a board can exist only in the HEAD replay (a re-recorded manifest can
    name a step the original run never wrote).
    """
    notes = []
    regen = _regen_steps(info['manifest'])
    regen_actions = [s['action'] for s in regen]

    saved_path = os.path.join(info['rundir'], f"{info['board']}_plan.json")
    if not os.path.exists(saved_path):
        cands = sorted(glob.glob(os.path.join(info['rundir'], '*_plan.json')))
        saved_path = cands[0] if cands else None
    saved = None
    if saved_path and os.path.exists(saved_path):
        try:
            saved = json.load(open(saved_path)).get('steps')
        except Exception as e:
            notes.append(f"saved plan unreadable ({e}); using regenerated plan")
            saved = None

    if source == 'saved' and saved:
        raw, info['plan_path'] = saved, saved_path
        saved_actions = [s['action'] for s in saved]
        if saved_actions != regen_actions:
            notes.append(
                f"saved plan is STALE vs redo_commands.sh "
                f"({len(saved_actions)} steps vs {len(regen_actions)}) -- the "
                f"manifest was re-recorded after the plan was written")
    else:
        raw, info['plan_path'] = [dict(s) for s in regen], '<regenerated>'
        if source == 'saved':
            notes.append("no saved plan JSON found; regenerated from the manifest")

    # Pair each raw step with its CLI output board: align on the action sequence
    # (they are emitted in chain order), matching the longest common prefix so a
    # stale plan still localizes everything up to where it diverged.
    raw_actions = [s['action'] for s in raw]
    n_align = 0
    while (n_align < len(raw_actions) and n_align < len(regen_actions)
           and raw_actions[n_align] == regen_actions[n_align]):
        n_align += 1
    if n_align < len(raw_actions):
        notes.append(f"plan/manifest step alignment stops at step {n_align + 1}"
                     f" -- per-step localization covers steps 1-{n_align}")
    cli_names = []
    for i in range(len(raw)):
        name = None
        if i < n_align:
            files = regen[i].get('_files') or []
            if files:
                name = os.path.basename(files[-1])
        cli_names.append(name)

    for s in raw:
        s.pop('_files', None)

    if not augment:
        return raw, cli_names, notes

    from kicad_routing_plugin.ai_plan import parse_plan_result
    steps, errors = parse_plan_result(json.dumps({'steps': raw}))
    if steps is None:
        raise RuntimeError(f"plan rejected by parse_plan_result: {errors}")
    notes += [f"parse_plan_result: {e}" for e in errors]
    if len(steps) != len(raw):
        notes.append(f"parse_plan_result augmented the plan: {len(raw)} -> "
                     f"{len(steps)} steps (GUI-side guards; the CLI chain has no "
                     f"counterpart for the added step(s))")
        cli_names = cli_names + [None] * (len(steps) - len(cli_names))
    return steps, cli_names[:len(steps)], notes


# ----------------------------------------------------------- CLI reference leg

def build_cli_reference(info, workdir, timeout=10800, verbose=False):
    """Replay the recorded manifest at HEAD into <workdir>/cli_head.

    Uses redo_stress_test.py -- the canonical no-LLM replayer -- so the commands,
    the pruning (compute_prune_keep) and therefore the intermediate board names
    are IDENTICAL to what the plan's steps were derived from, which is what makes
    the per-step pairing valid. --workdir runs every command inside the fresh dir
    and --remap rewrites the recorded absolute run-dir paths into it, so the
    original run dir is never touched.
    """
    dest = os.path.join(workdir, 'cli_head')
    os.makedirs(dest, exist_ok=True)
    log_path = os.path.join(workdir, 'cli_head.log')

    # The CLI leg runs AS DEPLOYED -- whatever `python3` the recorded manifest
    # lines resolve to -- while the GUI leg runs under KiCad's bundled python.
    # That is deliberate: routing must be interpreter-INDEPENDENT, so leaving
    # the two legs on their real interpreters makes this driver a live check on
    # that. It was not always true. Python 3.12 changed sum() to compensated
    # summation, and bga_fanout's cluster means used it, so eth_tap's U13 fanout
    # emitted 109 segments on 3.14 and 110 on 3.9 from byte-identical inputs
    # (fixed: grid.py now uses math.fsum). If a cross-version difference ever
    # returns it will surface HERE, as a copper divergence -- check the
    # interpreters before concluding the GUI is at fault.
    cmd = ['python3', '-X', 'utf8',
           os.path.join(STRESS_TESTS, 'redo_stress_test.py'), info['manifest'],
           '--workdir', dest,
           '--remap', f"{info['rundir']}:{dest}",
           '--skip-checks', '--skip-validate']
    t0 = time.time()
    with open(log_path, 'w') as log:
        r = subprocess.run(cmd, stdout=(None if verbose else log),
                           stderr=subprocess.STDOUT, timeout=timeout)
    out = {'dir': dest, 'returncode': r.returncode,
           'seconds': round(time.time() - t0, 1), 'log': log_path}
    boards = [f for f in os.listdir(dest) if f.endswith('.kicad_pcb')]
    if not boards:
        out['final'] = None
        return out
    # Same resolution order as the recorded side: the manifest's last output,
    # else the newest board the replay produced.
    prefer = []
    from grade_final import redo_final_and_clearance
    name, _ = redo_final_and_clearance(open(info['manifest'], encoding='utf-8',
                                            errors='replace').read())
    if name:
        prefer.append(os.path.basename(name))
    prefer.append(max(boards, key=lambda f: os.path.getmtime(os.path.join(dest, f))))
    out['final'] = next((os.path.join(dest, p) for p in prefer
                         if os.path.exists(os.path.join(dest, p))), None)
    return out


# ------------------------------------------------------------------ the replay

def _copy_board_with_siblings(src, dst):
    """Copy a board AND every sibling (.kicad_pro carries the DRC floor; a bare
    cp strands it and the next step resolves a looser floor -- #441)."""
    shutil.copy(src, dst)
    src_stem = os.path.splitext(src)[0]
    dst_stem = os.path.splitext(dst)[0]
    for f in glob.glob(src_stem + '.*'):
        ext = os.path.splitext(f)[1]
        if ext == '.kicad_pcb':
            continue
        try:
            shutil.copy(f, dst_stem + ext)
        except OSError:
            pass


class _Tee:
    """Send engine stdout (including the Rust router's fd-1 writes) to a log
    file while keeping a handle on the real stdout for progress lines."""

    def __init__(self, path, enabled):
        self.path, self.enabled = path, enabled
        self._saved_fd = None
        self._log_fd = None

    def __enter__(self):
        if not self.enabled:
            return self
        sys.stdout.flush()
        self._saved_fd = os.dup(1)
        self._log_fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        os.dup2(self._log_fd, 1)
        return self

    def progress(self, text):
        line = (text + "\n").encode()
        if self._saved_fd is not None:
            os.write(self._saved_fd, line)
        else:
            sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def __exit__(self, *exc):
        if self._saved_fd is None:
            return False
        sys.stdout.flush()
        os.dup2(self._saved_fd, 1)
        os.close(self._saved_fd)
        os.close(self._log_fd)
        self._saved_fd = None
        return False


def replay(info, steps, workdir, timeout=7200, verbose=False, snapshots=True):
    """Run the plan through the REAL plugin dialog + PlanExecutor, headless.

    The driving itself (headless dialog, PlanExecutor in a wx MainLoop, popup
    stubs, per-step snapshots, deterministic wx teardown) lives in the
    repo-root `headless_plan` module, which is also what the user-facing
    `run_plan.py` runs -- so this harness exercises the shipped driver rather
    than a private copy of it.

    Returns a dict with the produced board, per-step status and timings.
    """
    from headless_plan import run_plan

    live = os.path.join(workdir, 'gui_replay.kicad_pcb')
    _copy_board_with_siblings(info['input_board'], live)

    state = {'started': {}}
    log_path = os.path.join(workdir, 'replay.log')
    tee = _Tee(log_path, enabled=not verbose)

    from kicad_routing_plugin.ai_plan import step_label

    def on_status(index, status):
        if status == 'running':
            state['started'][index] = time.time()
            tee.progress(f"  step {index + 1}/{len(steps)}  "
                         f"{step_label(index + 1, steps[index])} ...")
            return
        elapsed = time.time() - state['started'].get(index, time.time())
        tee.progress(f"  step {index + 1}/{len(steps)}  {status}  ({elapsed:.1f}s)")

    try:
        shown = os.path.relpath(log_path, REPO)
    except ValueError:      # a work dir on another Windows drive than the repo
        shown = log_path
    with tee:
        tee.progress(f"  replaying {len(steps)} step(s) "
                     f"(engine output -> {shown})")
        result = run_plan(live, steps,
                          snapshot_dir=(workdir if snapshots else None),
                          snapshot_prefix='gui_step',
                          snapshot_format='{prefix}{n:02d}.kicad_pcb',
                          timeout=timeout, on_status=on_status, quiet=True)

    result['live_board'] = live      # the name the rest of this harness uses

    with open(os.path.join(workdir, 'replay_plan_log.txt'), 'w') as f:
        f.write("\n".join(result['log']))
    return result


# -------------------------------------------------------------------- grading

def grade_board(board, clearance, baseline):
    """Router-attributable DRC + connectivity through the corpus's OWN shared
    grading core, so these numbers are comparable to authoritative_grade.json
    by construction (same clearance resolution, same #405 baseline subtraction).
    """
    out = {'board': os.path.basename(board), 'clearance': clearance}
    try:
        from kicad_drc_compare import compare_board_data
        data = compare_board_data(board, clearance=clearance, baseline=baseline)
        if not data or data.get('skip'):
            raise RuntimeError((data or {}).get('skip', 'compare_board_data unavailable'))
        out['drc'] = data['check_drc']
        out['drc_kicad'] = data['kicad']
        out['drc_preexisting'] = data['checkdrc_preexisting']
        out['drc_kicad_only'] = data['kicad_only']
        out['drc_checkdrc_only'] = data['checkdrc_only']
    except Exception as e:
        out['grade_core_err'] = str(e)[:150]
        try:
            o = subprocess.run([sys.executable, '-X', 'utf8',
                                os.path.join(REPO, 'py_router', 'check_drc.py'), board,
                                '-c', str(clearance)],
                               capture_output=True, text=True, timeout=1800).stdout
            m = re.search(r'FOUND (\d+) DRC', o)
            out['drc'] = int(m.group(1)) if m else (0 if 'NO DRC' in o else None)
        except Exception as e2:
            out['drc'] = None
            out['drc_err'] = str(e2)[:100]
    try:
        o = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(REPO, 'py_router', 'check_connected.py'), board],
                           capture_output=True, text=True, timeout=1800).stdout
        out['fully_connected'] = 'ALL NETS FULLY CONNECTED' in o
        m = re.search(r'(\d+)\s+net\(?s?\)?\s+.*not fully connected', o, re.I)
        if m:
            out['unconnected_nets'] = int(m.group(1))
    except Exception as e:
        out['fully_connected'] = None
        out['conn_err'] = str(e)[:100]
    return out


# ------------------------------------------------------------- copper compare

def canon(path, dp=6):
    """UUID-independent canonical multisets of a board's routed state.

    Segments key on (net NAME, layer, unordered endpoints, width) and vias on
    (net, x, y, size, drill, span) -- never on UUID, which is random per run.
    Counters (not sets) so a duplicated segment is a difference, not a no-op.
    `dp` defaults to 6 (1 nm = the board's own resolution), i.e. an EXACT
    comparison -- see --dp in the module docstring.
    """
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    name = {n.net_id: n.name for n in pcb.nets.values()}

    def net(nid):
        return name.get(nid, f'#{nid}')

    segs = Counter()
    for s in pcb.segments:
        a = (round(s.start_x, dp), round(s.start_y, dp))
        b = (round(s.end_x, dp), round(s.end_y, dp))
        segs[(net(s.net_id), s.layer, tuple(sorted((a, b))), round(s.width, dp))] += 1
    vias = Counter()
    for v in pcb.vias:
        vias[(net(v.net_id), round(v.x, dp), round(v.y, dp), round(v.size, dp),
              round(v.drill, dp), tuple(v.layers or ()))] += 1
    zones = Counter()
    for z in (pcb.zones or []):
        poly = tuple(sorted((round(x, dp), round(y, dp)) for x, y in (z.polygon or [])))
        zones[(z.net_name, z.layer, z.priority, poly)] += 1
    fps = Counter()
    for ref, fp in (pcb.footprints or {}).items():
        fps[(ref, round(fp.x, dp), round(fp.y, dp), round(fp.rotation or 0.0, 3))] += 1
    return {'segments': segs, 'vias': vias, 'zones': zones, 'footprints': fps}


def _near_match(gui_only, cli_only, tol, kind):
    """Pair leftover items across sides within `tol` so sub-micron quantization
    is separated from genuinely different geometry. Greedy, bucketed by the
    non-positional part of the key; positional buckets are capped so a wholly
    different route can't make this quadratic."""
    if not tol or not gui_only or not cli_only:
        return 0
    def bucket(item):
        if kind == 'segments':
            netn, layer, _pts, width = item
            return (netn, layer, width)
        if kind == 'vias':
            netn, _x, _y, size, drill, span = item
            return (netn, size, drill, span)
        return None

    def points(item):
        if kind == 'segments':
            return item[2]
        return ((item[1], item[2]),)

    pool = defaultdict(list)
    for it in cli_only:
        pool[bucket(it)].append(it)
    paired = 0
    for it in gui_only:
        cands = pool.get(bucket(it))
        if not cands or len(cands) > 4000:
            continue
        gp = points(it)
        for j, other in enumerate(cands):
            op = points(other)
            if len(gp) != len(op):
                continue
            if all(abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol
                   for a, b in zip(gp, op)):
                cands.pop(j)
                paired += 1
                break
    return paired


def compare_copper(gui_path, cli_path, dp=6, tol=0.001):
    g, c = canon(gui_path, dp), canon(cli_path, dp)
    out = {}
    for kind in ('segments', 'vias', 'zones', 'footprints'):
        gk, ck = g[kind], c[kind]
        gui_only = gk - ck
        cli_only = ck - gk
        common = sum((gk & ck).values())
        entry = {'gui': sum(gk.values()), 'cli': sum(ck.values()),
                 'common': common,
                 'gui_only': sum(gui_only.values()),
                 'cli_only': sum(cli_only.values())}
        entry['identical'] = entry['gui_only'] == 0 and entry['cli_only'] == 0
        entry['equivalent'] = entry['identical']
        if kind in ('segments', 'vias') and not entry['identical']:
            # Since #493 both fronts emit copper through kicad_parser.mm_to_iu,
            # so identical routing lands on identical integer nanometres and the
            # exact multisets above should already agree. Pairing whatever is
            # left within --tol still separates a sub-micron representational
            # difference (were one to reappear) from genuinely different routing.
            entry['near_matched'] = _near_match(list(gui_only.elements()),
                                                list(cli_only.elements()), tol, kind)
            entry['unmatched_gui'] = entry['gui_only'] - entry['near_matched']
            entry['unmatched_cli'] = entry['cli_only'] - entry['near_matched']
            entry['equivalent'] = (entry['unmatched_gui'] == 0
                                   and entry['unmatched_cli'] == 0)
            per_net = Counter()
            for it in gui_only.elements():
                per_net[it[0]] += 1
            for it in cli_only.elements():
                per_net[it[0]] += 1
            entry['top_nets'] = per_net.most_common(8)
        if kind == 'footprints' and not entry['identical']:
            moved = {it[0] for it in gui_only.elements()} & \
                    {it[0] for it in cli_only.elements()}
            entry['moved_refs'] = sorted(moved)[:20]
            entry['moved_count'] = len(moved)
        out[kind] = entry
    out['identical'] = all(out[k]['identical'] for k in ('segments', 'vias'))
    out['equivalent'] = all(out[k]['equivalent'] for k in ('segments', 'vias'))
    return out


def compare_steps(workdir, steps, cli_boards, dp, tol, limit_kinds=('segments', 'vias')):
    """Per-step copper diff -> the FIRST diverging step."""
    rows = []
    for i, step in enumerate(steps):
        snap = os.path.join(workdir, f'gui_step{i + 1:02d}.kicad_pcb')
        cli = cli_boards[i] if i < len(cli_boards) else None
        if not os.path.exists(snap) or not cli:
            rows.append({'index': i, 'action': step['action'],
                         'cli_board': os.path.basename(cli) if cli else None,
                         'compared': False})
            continue
        cmp = compare_copper(snap, cli, dp=dp, tol=tol)
        rows.append({'index': i, 'action': step['action'],
                     'cli_board': os.path.basename(cli), 'compared': True,
                     'identical': all(cmp[k]['identical'] for k in limit_kinds),
                     'equivalent': all(cmp[k]['equivalent'] for k in limit_kinds),
                     'segments': cmp['segments'], 'vias': cmp['vias']})
    # "First divergence" means the first step producing GENUINELY different
    # copper -- a step that only differs by the 1 um apply rounding is not it.
    first = next((r for r in rows if r.get('compared') and not r['equivalent']), None)
    return rows, first


# --------------------------------------------------------------------- report

def _fmt_counts(entry):
    tag = ('IDENTICAL' if entry['identical']
           else 'EQUIVALENT' if entry.get('equivalent') else 'DIFFERS')
    s = (f"GUI {entry['gui']:>6}  CLI {entry['cli']:>6}  common {entry['common']:>6}  "
         f"GUI-only {entry['gui_only']:>5}  CLI-only {entry['cli_only']:>5}  {tag}")
    if entry.get('near_matched'):
        s += (f"  ({entry['near_matched']} pair within tol; "
              f"{entry.get('unmatched_gui', 0)}/{entry.get('unmatched_cli', 0)} "
              f"genuinely unmatched)")
    return s


def report(res, out=sys.stdout):
    p = lambda *a: print(*a, file=out)
    info = res['info']
    p("=" * 78)
    p(f"GUI plan replay vs CLI corpus run -- {info['board']}")
    p("=" * 78)
    p(f"run dir      : {info['rundir']}")
    p(f"input board  : {info['input_board']}")
    p(f"plan         : {info.get('plan_path')}  ({res['n_steps']} steps)")
    ref = res.get('cli_ref')
    if ref:
        p(f"CLI reference: manifest REPLAYED AT HEAD -> "
          f"{os.path.basename(res.get('cli_final', '?'))}  ({ref['seconds']}s)")
    else:
        p(f"CLI reference: recorded board {os.path.basename(info['cli_final'])} "
          f"(routed at the run's own commit -- engine drift shows up as divergence)")
    p(f"grade floor  : {info['chain_clearance']} mm  "
      f"(baseline-subtracted vs the unrouted input)")
    for n in res.get('plan_notes', []):
        p(f"  ! {n}")

    rep = res.get('replay')
    if rep:
        p("")
        p("-- replay " + "-" * 67)
        done = sum(1 for s in rep['steps'] if s['status'] == 'done')
        p(f"  {done}/{res['n_steps']} steps completed in {rep['seconds']}s")
        for s in rep['steps']:
            if s['status'] != 'done':
                p(f"  step {s['index'] + 1}: {s['status']}  ({s['label']})")
        if rep.get('aborted'):
            p(f"  ABORTED: {rep['aborted']}")
        if rep.get('timed_out'):
            p("  TIMED OUT")

    if res.get('grade'):
        g, c = res['grade']['gui'], res['grade']['cli']
        rec = res['grade'].get('recorded') or {}
        p("")
        p("-- grade " + "-" * 68)
        p(f"  {'':<22}{'GUI replay':>14}{'CLI @HEAD':>14}{'recorded':>14}")
        for label, key in (('router DRC', 'drc'),
                           ('fully connected', 'fully_connected'),
                           ('kicad-cli DRC', 'drc_kicad')):
            if key == 'drc_kicad' and g.get(key) is None and c.get(key) is None:
                continue
            p(f"  {label:<22}{str(g.get(key)):>14}{str(c.get(key)):>14}"
              f"{str(rec.get(key)):>14}")
        for side, d in (('GUI', g), ('CLI', c)):
            if d.get('grade_core_err'):
                p(f"  ! {side} grading fell back to check_drc: {d['grade_core_err']}")

    if res.get('drift'):
        d = res['drift']
        if d['identical']:
            p("")
            p("  engine drift since the run was recorded: none "
              "(recorded board == HEAD replay)")
        else:
            p("")
            p(f"  engine drift since the run was recorded (HEAD replay vs recorded "
              f"board, NOT a GUI/CLI gap):")
            p(f"    segments {d['segments']['cli']} -> {d['segments']['gui']}, "
              f"vias {d['vias']['cli']} -> {d['vias']['gui']}, "
              f"common {d['segments']['common']}")

    if res.get('copper'):
        p("")
        p("-- final board, canonical copper (UUID-independent) " + "-" * 25)
        for kind in ('segments', 'vias', 'zones', 'footprints'):
            p(f"  {kind:<11}{_fmt_counts(res['copper'][kind])}")
        for kind in ('segments', 'vias'):
            top = res['copper'][kind].get('top_nets')
            if top:
                p(f"  {kind} differing by net: " +
                  ", ".join(f"{n}={k}" for n, k in top))
        fp = res['copper']['footprints']
        if fp.get('moved_count'):
            p(f"  footprints at different poses: {fp['moved_count']} "
              f"({', '.join(fp['moved_refs'])})")

    if res.get('per_step'):
        p("")
        p("-- per-step localization " + "-" * 52)
        for r in res['per_step']:
            if not r['compared']:
                why = 'no CLI counterpart' if not r['cli_board'] else 'no snapshot'
                p(f"  step {r['index'] + 1:>2} {r['action']:<14} -- skipped ({why})")
                continue
            tag = ('match' if r['identical']
                   else 'match (within tol)' if r['equivalent'] else 'DIVERGES')
            p(f"  step {r['index'] + 1:>2} {r['action']:<14} vs {r['cli_board']:<32} "
              f"segs {r['segments']['common']}/{r['segments']['cli']} "
              f"vias {r['vias']['common']}/{r['vias']['cli']}  {tag}")
        first = res.get('first_divergence')
        if first:
            p(f"  >> FIRST GENUINE DIVERGENCE: step {first['index'] + 1} "
              f"({first['action']}) vs {first['cli_board']}: segments "
              f"{first['segments'].get('unmatched_gui')} GUI-only / "
              f"{first['segments'].get('unmatched_cli')} CLI-only beyond tol")
        elif any(r['compared'] for r in res['per_step']):
            p("  >> no per-step divergence")

    p("")
    p(f"VERDICT: {res['verdict']}")
    if res.get('workdir'):
        p(f"workdir: {res['workdir']}")


# ----------------------------------------------------------------------- main

def run_one(rundir, args):
    res = {'rundir': rundir}
    info = resolve_run(rundir)
    res['info'] = info

    steps, cli_names, notes = load_plan(
        info, source=args.plan_source, augment=not args.raw_plan)
    if args.max_steps:
        steps = steps[:args.max_steps]
        cli_names = cli_names[:args.max_steps]
        notes.append(f"--max-steps {args.max_steps}: running a prefix of the plan")
    res['n_steps'] = len(steps)
    res['plan_notes'] = notes
    res['plan'] = steps

    workdir = args.workdir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'work', f"replay_{info['board']}")
    if args.fresh and os.path.isdir(workdir):
        shutil.rmtree(workdir)
    os.makedirs(workdir, exist_ok=True)
    res['workdir'] = workdir

    # The CLI reference. Re-running the chain at HEAD is the default because a
    # recorded board carries its run-date engine; see the module docstring.
    cli_final = info['cli_final']
    ref_dir = info['rundir']
    if args.cli_ref == 'head':
        ref = build_cli_reference(info, workdir, timeout=args.timeout,
                                  verbose=args.verbose)
        res['cli_ref'] = ref
        if not ref.get('final'):
            res['verdict'] = (f"CLI REFERENCE REPLAY FAILED (rc={ref['returncode']}, "
                              f"see {ref['log']})")
            res['exit'] = 3
            return res
        cli_final = ref['final']
        ref_dir = ref['dir']
        if ref['returncode'] != 0:
            notes.append(f"CLI reference replay returned rc={ref['returncode']} "
                         f"(partial chain; see {os.path.basename(ref['log'])})")
    # Resolve each step's CLI board against the reference dir actually in use.
    cli_boards = [(os.path.join(ref_dir, n)
                   if n and os.path.exists(os.path.join(ref_dir, n)) else None)
                  for n in cli_names]
    # A truncated GUI run must be compared against the CLI board of its LAST
    # executed step, not the chain's final board.
    if args.max_steps:
        tail = next((b for b in reversed(cli_boards) if b), None)
        if tail:
            cli_final = tail
            notes.append(f"--max-steps: comparing against {os.path.basename(tail)} "
                         f"(the CLI board of the last executed step)")
    res['cli_final'] = cli_final

    if args.gui_board:
        gui_final = os.path.abspath(args.gui_board)
        res['replay'] = None
    else:
        rep = replay(info, steps, workdir, timeout=args.timeout,
                     verbose=args.verbose)
        res['replay'] = rep
        gui_final = rep['live_board']
        if rep['aborted'] or rep['timed_out'] or rep['completed'] != len(steps):
            res['verdict'] = (f"REPLAY INCOMPLETE ({rep['completed']}/{len(steps)} "
                              f"steps; {rep['aborted'] or 'timed out'})")
            res['exit'] = 3
            return res
    res['gui_final'] = gui_final

    baseline = info['input_board']
    clr = info['chain_clearance']
    res['grade'] = {
        'gui': grade_board(gui_final, clr, baseline),
        'cli': grade_board(cli_final, clr, baseline),
    }
    if args.cli_ref == 'head':
        # Grade the recorded board too: the gap between it and the HEAD replay is
        # the engine's own drift, which is worth seeing but is NOT a GUI/CLI gap.
        res['grade']['recorded'] = grade_board(info['cli_final'], clr, baseline)
        res['drift'] = compare_copper(cli_final, info['cli_final'],
                                      dp=args.dp, tol=args.tol)
    res['copper'] = compare_copper(gui_final, cli_final, dp=args.dp, tol=args.tol)
    if not args.gui_board:
        rows, first = compare_steps(workdir, steps, cli_boards, args.dp, args.tol)
        res['per_step'] = rows
        res['first_divergence'] = first

    g, c = res['grade']['gui'], res['grade']['cli']
    grade_parity = (g.get('drc') == c.get('drc')
                    and g.get('fully_connected') == c.get('fully_connected'))
    cu = res['copper']
    if grade_parity and cu['identical']:
        res['verdict'] = "IDENTICAL -- same grade AND byte-for-byte same copper geometry"
        res['exit'] = 0
    elif grade_parity and cu['equivalent']:
        res['verdict'] = (
            f"EQUIVALENT -- same grade, and every differing item pairs within "
            f"{args.tol} mm (a sub-micron representational difference, not "
            f"different routing)")
        res['exit'] = 0
    elif grade_parity:
        res['verdict'] = (
            f"GRADE PARITY, COPPER DIFFERS "
            f"(segments {cu['segments'].get('unmatched_gui', cu['segments']['gui_only'])} "
            f"GUI-only / {cu['segments'].get('unmatched_cli', cu['segments']['cli_only'])} "
            f"CLI-only beyond tol; vias "
            f"{cu['vias'].get('unmatched_gui', cu['vias']['gui_only'])}/"
            f"{cu['vias'].get('unmatched_cli', cu['vias']['cli_only'])})")
        res['exit'] = 1
    else:
        res['verdict'] = (f"GRADE DIFFERS -- GUI drc={g.get('drc')} "
                          f"conn={g.get('fully_connected')} vs CLI drc={c.get('drc')} "
                          f"conn={c.get('fully_connected')}")
        res['exit'] = 2
    return res


def _json_safe(o):
    if isinstance(o, dict):
        return {str(k): _json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_safe(v) for v in o]
    if isinstance(o, (str, int, float, bool)) or o is None:
        return o
    return str(o)


def run_set(args):
    setdir = os.path.abspath(args.set)
    names = [d for d in sorted(os.listdir(setdir))
             if os.path.exists(os.path.join(setdir, d, 'redo_commands.sh'))]
    if args.boards:
        want = {b.strip() for b in args.boards.split(',')}
        names = [n for n in names if n in want]
    if not names:
        print(f"no board run dirs under {setdir}", file=sys.stderr)
        return 3
    print(f"replaying {len(names)} board(s) from {setdir}\n")
    rows = []
    passthrough = []
    for flag in ('--plan-source', '--cli-ref', '--tol', '--dp', '--timeout'):
        val = getattr(args, flag.lstrip('-').replace('-', '_'))
        if val is not None:
            passthrough += [flag, str(val)]
    for flag in ('--raw-plan', '--fresh', '--verbose'):
        if getattr(args, flag.lstrip('-').replace('-', '_')):
            passthrough.append(flag)
    for i, name in enumerate(names, 1):
        rundir = os.path.join(setdir, name)
        out_json = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'work',
            f'replay_{name}', 'result.json')
        print(f"[{i}/{len(names)}] {name} ...", flush=True)
        # One subprocess per board: a fresh wx.App + pcbnew board per replay, so
        # no dialog/board state leaks between boards.
        r = subprocess.run([sys.executable, os.path.abspath(__file__), rundir,
                            '--json', out_json] + passthrough)
        row = {'board': name, 'exit': r.returncode}
        try:
            d = json.load(open(out_json))
            row['verdict'] = d.get('verdict')
            row['gui_drc'] = (d.get('grade') or {}).get('gui', {}).get('drc')
            row['cli_drc'] = (d.get('grade') or {}).get('cli', {}).get('drc')
            seg = (d.get('copper') or {}).get('segments', {})
            # Beyond-tol counts are the meaningful ones; fall back to the raw
            # diff when the sides matched exactly (no near-match pass ran).
            row['seg_gui_only'] = seg.get('unmatched_gui', seg.get('gui_only'))
            row['seg_cli_only'] = seg.get('unmatched_cli', seg.get('cli_only'))
            fd = d.get('first_divergence')
            row['first_div'] = f"step {fd['index'] + 1} {fd['action']}" if fd else None
        except Exception:
            row['verdict'] = 'no result'
        rows.append(row)
    print("\n" + "=" * 108)
    print(f"{'board':<26}{'GUI drc':>8}{'CLI drc':>8}{'seg beyond tol':>16}"
          f"{'first divergence':>24}  result")
    print("-" * 108)
    for r in rows:
        seg = (f"+{r.get('seg_gui_only')}/-{r.get('seg_cli_only')}"
               if r.get('seg_gui_only') is not None else '-')
        short = (r.get('verdict') or '').split(' --')[0].split(' (')[0]
        print(f"{r['board']:<26}{str(r.get('gui_drc')):>8}{str(r.get('cli_drc')):>8}"
              f"{seg:>16}{str(r.get('first_div')):>24}  {short}")
    ident = sum(1 for r in rows if r['exit'] == 0)
    print("-" * 100)
    print(f"{ident}/{len(rows)} identical, "
          f"{sum(1 for r in rows if r['exit'] == 1)} grade-parity-copper-differs, "
          f"{sum(1 for r in rows if r['exit'] == 2)} grade differs, "
          f"{sum(1 for r in rows if r['exit'] == 3)} failed")
    return 0 if ident == len(rows) else 1


def main():
    ap = argparse.ArgumentParser(add_help=True, description=__doc__.split('\n')[0])
    ap.add_argument('rundir', nargs='?', help='a stress run dir')
    ap.add_argument('--set', help='batch-replay every board dir under this set dir')
    ap.add_argument('--boards', help='with --set: comma-separated board names')
    ap.add_argument('--workdir')
    ap.add_argument('--cli-ref', choices=('head', 'recorded'), default='head')
    ap.add_argument('--plan-source', choices=('saved', 'regen'), default='saved')
    ap.add_argument('--raw-plan', action='store_true')
    ap.add_argument('--max-steps', type=int)
    ap.add_argument('--gui-board')
    ap.add_argument('--tol', type=float, default=0.001)
    ap.add_argument('--dp', type=int, default=6)
    ap.add_argument('--timeout', type=float, default=7200)
    ap.add_argument('--json')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--fresh', action='store_true')
    args = ap.parse_args()

    if args.set:
        return run_set(args)
    if not args.rundir:
        ap.error('give a run dir, or --set <runs_setN>')

    try:
        res = run_one(args.rundir, args)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 3
    report(res)
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w') as f:
            json.dump(_json_safe({k: v for k, v in res.items() if k != 'plan'}),
                      f, indent=1)
    return res.get('exit', 3)


if __name__ == '__main__':
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
