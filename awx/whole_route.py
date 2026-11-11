#!/usr/bin/env python3
"""whole_route.py K OUTDIR [ROUNDS] [--feedback-rounds F] -- the whole route on rung K, end to end, in Python.
   whole_route.py --loop SOLVE.json OUTDIR [ROUNDS] -- the loop alone on a solve (the bench from BENCH / NETS / DEST).

The same chain as whole_chain.sh and whole_loop.sh, and the same stages run the same way: each stage is its own
process, given the environment and the arguments the shell scripts give it, in the same order, on the same branches.
What changes is only the driver, which no longer needs zsh.

The chain (whole_chain.sh): the fanout choosing every net's tooth and berth with the whole route's ends model
(fanout_from_plan, PLAN_JUDGE=ends), the solve (whole_solve), the loop (below) and the route all at once (route_lanes),
graded (check_connected, check_drc). A round that leaves nets open is followed by round 1 AGAIN under the next seed
(cp_seed: every solve of the round searches under the step's seed plus the round less one) -- from the same board, the
cap step again, nothing carried from the round before: another draw, not a repair of this one -- up to ROUNDS rounds
(default 4), the first laid clean ending them. With --feedback-rounds F (default 0), up to F of them after each draw are
FEEDBACK ROUNDS under its seed instead: the ends the round before's audits found crowded, its lanes over two vias and
its open nets (whole_feedback) go back to the fanout (FEEDBACK=), which chooses again INCREMENTALLY from that round's
board. A round whose solve has NO PLAN is followed by a feedback round whatever F (the lanes its solve found no room
for, named), within the ROUNDS. BASE (default fb_t2q_pairs.kicad_pcb) is the bench, DEST (default DU1) its destination
part, both from the environment as the shell script reads them.

The loop (whole_loop.sh), every step fed by a MEASUREMENT of the one before: geometry (whole_geo), polish
(whole_polish), audit (whole_audit through whole_gate); side flips the polish could not avoid go to the geometry again
on the same solve, or with cuts and findings to the solve again; a smooth plan that passes has its pairs laid first
(whole_snap --pairs), the singles fitted round them (whole_polish; SNAP_KEEP where a single is short) and snapped
(whole_snap), audited, gated and linted. A loop that is NOT CONVERGING stops (exit 3), and so does one whose findings
stand at the ENDS (exit 4, the fanout's to change). Each stage goes through stage_cache.py, on by default here as in
whole_loop.sh (STAGE_CACHE=0 runs every stage).

The chain never ends with nothing while a plan for some of the lanes can be laid: a round's solve keeps a plan it cannot
prove (laid all the same), a loop that does not pass is laid from the plan it held (held_plan; a lane it cannot lay
stays open), and a board the fanout's audit refuses, or a round that lays NOTHING or leaves nets OPEN, names its lanes
(whole_feedback: the split pairs and the far face's teeth, the open nets, else the lanes the ends model names) and the
next seed's round follows. The loop's last re-solve keeps a plan it cannot prove. When no round laid anything, the LAST
RESORT is a partial plan on the last ends laid, the named lanes (then more) left out and open. The run's result is its best --
the fewest open nets, then the fewest vias -- in OUTDIR/best.kicad_pcb (and its own rN/ or partialN/seq.kicad_pcb).
The last line is that result's grade:
  WHOLE K=.. round=.. lanes=../.. vias=.. copper=..mm connected=0|1 drc=N secs=.. open=..
Exits 0 when it is connected and DRC-clean, 1 when nets are left open or a violation stands, 2 when nothing could be
laid at all (no fanout board, or no plan even for half the lanes).
--loop exits 0 with OUTDIR/plan.json the snapped plan that passed, 1 when no round got there, 3 when it stopped not
converging, 4 when it stopped at crowded ends.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import atexit
import collections
import contextlib
import glob
import io
import json
import math
import os
import re
import runpy
import shutil
import subprocess
import sys
import tempfile
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import awx_settings  # noqa: E402
import cp_seed  # noqa: E402
PYR = os.path.normpath(os.path.join(HERE, '..', 'py_router'))
PY = sys.executable
PATIENCE = 2                               # the loop's rounds in a row without a new best
HELD_DIVES = 2                             # a stopping loop's rounds more for its held plan's pairs (held_dives)
INHERIT = object()                         # run(to=INHERIT): the command's output where the driver's own goes


# =============================================================================== running a stage
class Log:
    """where the driver's own lines go: the console for the chain, the round's loop.log for the loop"""

    def __init__(self, f=None):
        self.f = f

    def __call__(self, s=''):
        if self.f is None:
            print(s, flush=True)
        else:
            self.f.write(s + '\n')
            self.f.flush()


FANOUT_RAISES = 3   # a feedback round's fanout laying the round before's ends again: its findings raised, the fanout again
INPROC = False      # --inproc: every stage in this process, as a routing call inside KiCad's own process will run them


@contextlib.contextmanager
def _fds(out_f, err_f):
    """file descriptors 1 and 2 on these open files for the block -- what a child's `> log 2>&1` gives it, the
    compiled code's own writes (the router, the solvers) included"""
    sys.stdout.flush(); sys.stderr.flush()
    saved = os.dup(1), os.dup(2)
    try:
        os.dup2(out_f.fileno(), 1)
        os.dup2(err_f.fileno(), 2)
        yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        os.dup2(saved[0], 1); os.dup2(saved[1], 2)
        os.close(saved[0]); os.close(saved[1])


def _stage_exit():
    """what a stage's process does on its way out, done at the end of a stage run in this one: the taut memo's last
    dirty shards saved (detect_buses registers it with atexit). Its REPORT is not printed, here or at exit: it counts
    what this process holds, every stage's shards so far, where a stage's own process holds only its own -- printed
    into a stage's output it became the last line of a gate's or a lint's, whose verdict is read off that line"""
    db = sys.modules.get('detect_buses')
    if db is not None:
        db._memo_save(force=True)
        atexit.unregister(db._memo_report)
    # ...and what a stage's process held and the next stage's never had, emptied: the cut phase's winding cuts judge
    # prices at, the buses whose joint ends could not decide, the checks' memo (keyed with no seed, areas or finished
    # buses) -- left, a round's ends were judged on an earlier round's cuts on another bench, and run in one process
    # the chain chose otherwise than run as processes. (An array's last plan stays: the warm start checks it before it
    # answers anything)
    we = sys.modules.get('whole_ends')
    if we is not None:
        we.WIND_SEEN.clear()
        we.JOINT_UNDECIDED.clear()
        we._CHECK_MEMO.clear()


def _inproc(argv, env, out_f, err_f):
    """`python3 ARGV...` run in this process as its own would run it: ENV the settings every awx module reads
    (awx_settings.given: the whole of them, as an environment is the whole of a child's -- os.environ is neither read
    by them nor written), the arguments, awx/ its directory, fds 1 and 2 on OUT_F / ERR_F (None: where the driver's
    go); the exit code"""
    # (sys.path is not restored: a module imported by an earlier stage put its paths there once, and a later stage's
    # lazy imports -- py_router's design_rules, say -- find their modules through them)
    saved = list(sys.argv), os.getcwd()
    os.chdir(HERE)
    sys.argv = list(argv)
    sys.path.insert(0, os.path.dirname(os.path.abspath(argv[0])))
    rc = 0
    try:
        with awx_settings.given(env), (_fds(out_f, err_f) if out_f is not None else contextlib.nullcontext()):
            try:
                runpy.run_path(argv[0], run_name='__main__')
            except SystemExit as e:
                if isinstance(e.code, str):         # sys.exit('why'): the interpreter prints it and exits 1
                    print(e.code, file=sys.stderr)
                rc = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
            except Exception:
                # (a stage's crash is its exit 1, as its own process's would be -- an interrupt is not: it stops the
                # driver, as Ctrl-C stops a chain of processes)
                traceback.print_exc()
                rc = 1
            finally:
                try:
                    _stage_exit()
                finally:
                    sys.stdout.flush(); sys.stderr.flush()
    finally:
        sys.argv = saved[0]
        os.chdir(saved[1])
    return rc


def run(argv, env, log_path=None, err=None, to=None, own_process=False):
    """`python3 ARGV...` in awx/ as the shell runs it: stdout and stderr to LOG_PATH (`> log 2>&1`), or both to the
    open file TO (a command inside the loop left unredirected: its lines land in the loop's log), or stdout captured
    and stderr to ERR (the loop's own log, as `$(...)` inside it leaves it); (exit code, stdout). In a process of its
    own, or with INPROC in this one (OWN_PROCESS: in its own whatever INPROC says)."""
    if INPROC and not own_process:
        if log_path is not None:
            with open(log_path, 'w') as f:
                return _inproc(argv, env, f, f), ''
        if to is INHERIT:
            return _inproc(argv, env, None, None), ''
        if to is not None:
            return _inproc(argv, env, to, to), ''
        with tempfile.TemporaryFile('w+') as cap, open(os.devnull, 'w') as dn:
            rc = _inproc(argv, env, cap, err or dn)
            cap.seek(0)
            return rc, cap.read()
    if log_path is not None:
        with open(log_path, 'w') as f:
            return subprocess.run([PY] + argv, cwd=HERE, env=env, stdout=f, stderr=subprocess.STDOUT).returncode, ''
    if to is INHERIT:
        return subprocess.run([PY] + argv, cwd=HERE, env=env).returncode, ''
    if to is not None:
        return subprocess.run([PY] + argv, cwd=HERE, env=env, stdout=to, stderr=to).returncode, ''
    p = subprocess.run([PY] + argv, cwd=HERE, env=env, stdout=subprocess.PIPE, stderr=err or subprocess.DEVNULL,
                       text=True)
    return p.returncode, p.stdout


def lines_of(path):
    try:
        with open(path) as f:
            return f.read().splitlines()
    except OSError:
        return []


def grep(path, pattern):
    """the lines of PATH matching the extended regex PATTERN"""
    r = re.compile(pattern)
    return [ln for ln in lines_of(path) if r.search(ln)]


def source_board_named(fo_log):
    """the source board a fanout's log names last -- its destination pass's '(source board: NAME, ...' line, on either
    path (fanout_from_plan.fanout_destination, joint_destination) -- or None"""
    sb = [m for ln in lines_of(fo_log) for m in re.findall(r'source board: ([^,]+)', ln)]
    return sb[-1] if sb else None


def tail(path, n):
    return lines_of(path)[-n:]


def prefixed(text, prefix):
    """`echo "$text" | sed 's/^/PREFIX/'`: every line prefixed, an empty text one prefixed empty line"""
    return '\n'.join(prefix + ln for ln in (text.split('\n') if text else ['']))


def stripped(s):
    """`$(...)`: the output less its trailing newlines"""
    return s.rstrip('\n')


# =============================================================================== the loop (whole_loop.sh)
def nflips(files):
    """the side flips in a comma list of polish outputs, every file's"""
    return len({tuple(x) for f in files.split(',') if f for x in json.load(open(f)).get('flips', [])})


def findings(gate_line):
    """the findings in a gate line (whole_gate's summary): dive, static, shape, swim, pitch in the plan, band outside
    (0/1); a gate line without its counts (an audit that did not run to its end) counts as many"""
    s = gate_line
    try:
        n = sum(int(re.search(k + r' (\d+)', s).group(1)) for k in ('dive', 'static', 'shape', 'swim'))
        n += int(re.search(r'pitch (\d+) in the plan', s).group(1))
        n += int(re.search(r'band broken (\d+)', s).group(1)) if 'band broken' in s else 0
        n += int(re.search(r'(\d+) lane\(s\) missing', s).group(1)) if 'missing' in s else 0     # a lane a snap could not lay
        return n + (1 if float(re.search(r'band ([\d.]+) mm', s).group(1)) > 0 else 0)
    except AttributeError:
        return 999


def loop_rounds():
    """the loop's rounds at most: 6, or LOOP_ROUNDS"""
    return int(awx_settings.get('LOOP_ROUNDS') or '6')


def lay_tries():
    """the boards a loop that did not pass may lay (lay_rounds): LAY_TRY where it is set -- N in all, 1 the held plan's
    alone -- else 0, no limit: the held plan's, then every other round's until one lays with no net open"""
    return int(awx_settings.get('LAY_TRY') or '0')


class NotConverging(Exception):
    pass


# a lane the snap could not lay is held under an island on one layer when its search stuck within this much of it, mm
SNAP_ISLAND_REACH = 2.0
# a lane the audit found against an array's own pad is held off that layer this far round the place at least, mm
ARRAY_CUT_R = 0.5


def _layer_index(tag):
    """a routing layer's index from its tag in a finding: F or B (the audits' letter), else the layer's name
    (route_layers: F.Cu 0, B.Cu 1, the inner ones after)"""
    if tag in ('F', 'F.Cu'):
        return 0
    if tag in ('B', 'B.Cu'):
        return 1
    import route_layers
    return route_layers.index(tag)


def _seg_pt(p, a, b):
    """point p's distance from the segment a-b"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def layer_cut(lane, island, box, layer):
    """the loop's LAYER cut: `lane` held off the island's layers across `island` (its box, whole_geo's island_boxes;
    `blocked` the island's layers, `layer` -- on two -- the one the lane is held on) -- none where the island stands on
    every routing layer, which no change answers (on two: where it stands on both)"""
    import route_layers
    if not box or layer not in box[4] or len(box[4]) >= len(route_layers.layers()):
        return None
    c = {'lane': lane, 'island': island, 'layer': 1 - layer, 'box': list(box[:4])}
    if len(route_layers.layers()) > 2:
        c['blocked'] = sorted(box[4])
    return c


_TRACKS = {}       # (a bench file, its mtime and size) -> (its tracks by layer index, its keep-out areas' boxes, shapes)


def _tracks_of(bench, areas=None):
    """(the bench's tracks as (x0, y0, x1, y1, half width, net name) per routing-layer index (route_layers), the
    bboxes of its keep-out areas as plan_audit.keepout_areas lists them, and their shapes (polygon, holes) as point
    lists), read once -- in the PLAN's frame, as the audit's points and the plan's lanes are: on a round of the other
    chirality (its sidecar's chi), mirrored about the board's axis as repair_ends mirrors its points back. Read as the
    board has them, a mirrored round's layer cuts met no other net's track at their place and held no other lane off
    it: an earlier bus's In2 run took a different lane of the bus each round. `areas`: the round's areas file
    (BUS_AREAS), its keep-outs listed after the board's as the audit lists them"""
    st_ = os.stat(bench)
    # (keyed by the bench -- and, negotiated, by the other buses' lanes as published now, JOINT_DIR/lanes: read once,
    # a later round's cuts were made against those lanes as round 1 had them. The round's areas are added per call:
    # in the key, the keep-out rows (with them) and the copper rows (without) re-parsed the board in turn)
    jf_ = None
    if awx_settings.get('JOINT_FOREIGN') and awx_settings.get('JOINT_DIR'):
        jd_ = os.path.join(awx_settings.get('JOINT_DIR'), 'lanes')
        jf_ = tuple(sorted((f_, os.stat(os.path.join(jd_, f_)).st_mtime_ns) for f_ in os.listdir(jd_)
                           if f_.endswith('.json'))) if os.path.isdir(jd_) else ()
    key = (os.path.abspath(bench), st_.st_mtime_ns, st_.st_size, awx_settings.get('JOINT_FOREIGN') or '', jf_)
    if key not in _TRACKS:
        import route_layers
        sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(bench)
        if awx_settings.get('JOINT_FOREIGN'):
            import joint_lanes
            joint_lanes.foreign(pcb)        # (the other buses' lanes, copper of their nets: joint_lanes)
        lays = list(route_layers.layers())
        by = collections.defaultdict(list)
        from coherent_nets import namer
        sn = namer(pcb)
        side_ = os.path.splitext(bench)[0] + '.plan.json'
        my = lambda y_: y_
        ml = lambda L_: L_
        if os.path.isfile(side_) and int(json.load(open(side_)).get('chi', 1)) < 0:
            # (turned over: y about the axis, and F.Cu and B.Cu swapped -- braid.setup's turn-over, flip_frame)
            from bga_fanout.flip_frame import mirror_axis, other_layer
            cy2_ = 2 * mirror_axis(pcb)
            my = lambda y_: cy2_ - y_
            ml = other_layer
        for s_ in pcb.segments:
            if s_.layer in lays and not getattr(s_, 'graphic', False):
                nm = sn(pcb.nets[s_.net_id].name) if s_.net_id in pcb.nets else ''
                by[_layer_index(ml(s_.layer))].append((s_.start_x, my(s_.start_y), s_.end_x, my(s_.end_y),
                                                         s_.width / 2, nm))
        import plan_audit as _pa
        ka_ = _pa.keepout_areas(pcb, lays)
        rnd = lambda P: [[round(float(x_), 4), round(float(my(y_)), 4)] for x_, y_ in P]
        box_ = lambda b_: [b_[0], min(my(b_[1]), my(b_[3])), b_[2], max(my(b_[1]), my(b_[3]))]
        _TRACKS.clear()
        _TRACKS[key] = (dict(by), [box_(k_[3]) for k_ in ka_], [(rnd(k_[0]), [rnd(h_) for h_ in k_[1]]) for k_ in ka_])
    by_, kb_, ks_ = _TRACKS[key]
    # (...and the agent's areas after the board's keep-outs, in the plan's frame already -- the audit's K numbers)
    import plan_audit as _pa
    with awx_settings.given({**awx_settings.environ(), 'BUS_AREAS': areas or ''}):
        ar_ = _pa.bus_areas()
    return (by_, kb_ + [list(a_[3]) for a_ in ar_],
            ks_ + [([[round(float(x_), 4), round(float(y_), 4)] for x_, y_ in a_[0]], []) for a_ in ar_])


_VIA_ENDS = {}      # (a bench file, its mtime and size) -> its single lanes' via ends {(lane, 0 tooth / 1 berth)}


def _via_ends(bench):
    """the single lanes' ends on `bench` that are VIA ENDS -- its plan sidecar's end standing on a via of its net, or
    running to one on its layer (relayer.run_to_via), as whole_solve reads them: a via end's run is moved onto the
    layer the solve chooses (relay_ends), so its layer is no more the fanout's than the rest of its lane is"""
    sc_ = os.path.splitext(bench)[0] + '.plan.json'
    if not os.path.isfile(sc_):
        return set()
    st_ = os.stat(bench)
    key = (os.path.abspath(bench), st_.st_mtime_ns, st_.st_size)
    if key not in _VIA_ENDS:
        if PYR not in sys.path:
            sys.path.insert(0, PYR)
        from kicad_parser import parse_kicad_pcb
        from coherent_nets import namer
        import relayer as _relayer
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(bench)
        sn = namer(pcb)
        ids = {sn(nt.name): i for i, nt in pcb.nets.items() if nt.name}
        sc = json.load(open(sc_))
        lay = (sc.get('tooth_layer') or {}, sc.get('dest_layer') or {})
        out = set()
        for n, e_ in (sc.get('ends') or {}).items():
            nid = ids.get(n)
            if nid is None:
                continue
            for k_ in (0, 1):
                pt_, L_ = e_[k_], lay[k_].get(n)
                if any(v.net_id == nid and math.hypot(v.x - pt_[0], v.y - pt_[1]) < v.size / 2 for v in pcb.vias) \
                        or (L_ and _relayer.run_to_via(pcb, nid, pt_, L_)):
                    out.add((n, k_))
        _VIA_ENDS.clear()
        _VIA_ENDS[key] = out
    return _VIA_ENDS[key]


def audit_layer_cuts(audit, geo, bench=None, areas=None):
    """the LAYER cuts an audit's STATIC findings give: a lane found against a pad of an island (a geometry's 'islands',
    'REF.PAD' -> island) on one layer, the layer it was found on -- and, on more routing layers than two, a lane found
    against an ARRAY's own pad (no island), off the layer it was found on round that place (ARRAY_CUT_R): it was pushed
    into the pad by the lanes beside it on its layer -- a ring's lanes crowding the channel beside the destination
    (synth wind_rot16_g3_vias: nine lanes up a 2 mm channel on one layer, the innermost into the destination's corner
    ball every round) -- and on another layer it has room. A drilled ball stands on every layer, so this is no island's
    cut, which no change answers. The same, on more than two, for a lane found against another net's TRACK (copper: a
    bus an earlier step routed, on its layer alone -- sdi_mipi_conv's SDI_D0 from its surface tooth round U11's corner
    into the DDR3 bus's F.Cu fan, a finding at the teeth the loop had stopped on, ENDS CROWDED, both rounds): on another
    layer the lane passes under it, its tooth's change right at the tooth (whole_solve's TVIA). With the `bench` the
    lane is held off EVERY routing layer another net's tracks cross that place on, as an island's cut holds it off the
    island's layers -- that bus routed on four of the eight there, one layer a round found the next one blocked -- and
    none where they cross it on every layer, which no change answers. And a lane found in a KEEP-OUT rule area whose
    tracks are not allowed on some layers (plan_audit.keepout_areas -- the router stamps it, the plan did not see it:
    sdi_mipi_conv's wound SDI lanes round U15 laid on In2.Cu through the area east of it that keeps In1 and In2 clear,
    and the snap failed them there every round) is held off its layers where its laid path meets the area itself (its
    polygon: own_spans), not the area's box -- the area east of U15 is an L whose box reaches U15's berths. And on more
    than two, a PITCH finding at one lane's end (the geometry's `lanes`): the lane passing that end held off the
    layer there"""
    import route_layers
    nl = len(route_layers.layers())
    out, isl, bxs = [], geo.get('islands') or {}, geo.get('island_boxes') or {}
    ends_ = {n: (v['xy'][0], v['xy'][-1]) for n, v in (geo.get('lanes') or {}).items() if v.get('xy')}
    for ln in lines_of(audit):
        p = re.match(r'PITCH (\S+)\s+(\S+)\s+([FB]|In\d+\.Cu) min [-\d.]+\s+over [\d.]+ mm\s+at \(([-\d.]+),\s*([-\d.]+)\)',
                     ln)
        if p and nl > 2:
            # (two lanes short of their pitch on one layer at ONE lane's end -- its tooth or berth within the cut's box
            # of the place, the other lane passing it: the end is the fanout's and stands where it is, so the lane
            # passing is held off that layer there, as one found against another net's track. The history alone moves
            # it nowhere: a group bus's lane squeezed past a berth on its layer every round, its change two millimetres
            # on, tied to its crossings, and the loop stopped on the same finding with the berth's lane open. Both
            # lanes ending there are the fanout's to part; neither, the solve's own crossing to price)
            import rules as _rules
            R_ = _rules.active()
            x, y = float(p.group(4)), float(p.group(5))
            r = max(ARRAY_CUT_R, 3 * (R_.track / 2 + R_.clearance + R_.grid / 2))
            at_ = [any(math.hypot(e_[-2] - x, e_[-1] - y) <= r for e_ in ends_.get(n_, ()))
                   for n_ in (p.group(1), p.group(2))]
            if at_[0] != at_[1] and all(n_ in ends_ for n_ in (p.group(1), p.group(2))):
                lane, pad = (p.group(2), p.group(1)) if at_[0] else (p.group(1), p.group(2))
                c = {'lane': lane, 'island': pad, 'layer': 0, 'box': [x - r, y - r, x + r, y + r],
                     'blocked': [_layer_index(p.group(3))], 'firm': True}
                if c not in out:
                    out.append(c)
            continue
        k = re.match(r'STATIC (\S+)\s+([FB]|In\d+\.Cu) ([-+]?[\d.]+)/([\d.]+) keepout K(\d+) (\S+) at '
                     r'\(([-\d.]+),\s*([-\d.]+)\)', ln)
        if k:
            # (a KEEP-OUT area's layers: the lane held off them across its box -- the bench's, else round the place)
            lane, x, y, r = k.group(1), float(k.group(7)), float(k.group(8)), max(ARRAY_CUT_R, 3 * float(k.group(4)))
            blk = {_layer_index(k.group(2))} | {_layer_index(t_) for t_ in k.group(6).split('+')}
            kb = _tracks_of(bench, areas)[1] if bench else []
            ki = int(k.group(5))
            box = list(kb[ki]) if ki < len(kb) else [x - r, y - r, x + r, y + r]
            c = {'lane': lane, 'island': 'keepout K' + k.group(5), 'layer': 0, 'box': box, 'blocked': sorted(blk)} \
                if nl > 2 and len(blk) < nl else None
            if c and ki < len(kb):
                c['poly'], c['holes'] = _tracks_of(bench, areas)[2][ki]
            if c and c not in out:
                out.append(c)
            continue
        m = re.match(r'STATIC (\S+)\s+([FB]|In\d+\.Cu) ([-+]?[\d.]+)/([\d.]+) (pad|hole|copper) (\S+) ', ln)
        if not m:
            continue
        lane, L, bar, kind, pad = m.group(1), _layer_index(m.group(2)), float(m.group(4)), m.group(5), m.group(6)
        if pad in isl:
            c = layer_cut(lane, isl[pad], bxs.get(isl[pad]), L)
        elif nl > 2 and (at := re.search(r' at \(([-\d.]+),\s*([-\d.]+)\)', ln)):
            x, y, r = float(at.group(1)), float(at.group(2)), max(ARRAY_CUT_R, 3 * bar)
            blk = {L}
            if kind == 'copper' and bench:
                # (every layer another net's track crosses the place on: its distance from the place within the box)
                for Lk, segs in _tracks_of(bench)[0].items():
                    if any(nm_ != lane and _seg_pt((x, y), (x0_, y0_), (x1_, y1_)) < r + hw_
                           for x0_, y0_, x1_, y1_, hw_, nm_ in segs):
                        blk.add(Lk)
            c = {'lane': lane, 'island': pad, 'layer': 0, 'box': [x - r, y - r, x + r, y + r], 'blocked': sorted(blk)} \
                if len(blk) < nl else None
            if c and kind == 'copper':
                c['firm'] = True       # (a track the lane crosses is no island it might be sent round: whole_solve)
        else:
            c = None
        if c and c not in out:
            out.append(c)
    return out


def place_cuts(cuts, plan, bench, own=()):
    """(more routing layers than two) a cut against a TRACK an earlier step laid (audit_layer_cuts' copper cut, of a net
    not among the bus's own, `own`) held for every lane through its place: each other lane whose laid path (`plan`'s)
    passes the cut's box within its bar of that copper on a layer the cut holds -- the audit's bar, track/2 + clearance
    + half a grid step -- is held off those of them there too. The lane found, held off alone, sent the next round's
    plan through the same crossing on another lane (sdi_mipi_conv's SDI lanes across the DDR3 bus's B.Cu and In2.Cu fan
    below U11: the same four lanes through it every round, a different one on B each round, until the loop stopped on
    ENDS CROWDED after six). The bus's own copper -- its stubs, where its lanes end, which the plan orders itself -- is
    no such place, and a lane that ENDS in the place is not held, its end being the fanout's to move: the zynq's ETH
    stubs at U1's teeth would hold ETH_TXCTL off its own stub's layer, and the lanes beside it. Unless that end is a
    VIA END (_via_ends): its run's layer is the solve's to choose, and the place blocks it as it blocked the lane found
    -- two lanes' teeth stacked on two layers at one exit, another bus's track crossing in front of it on one of them:
    the lane found was held off that layer, the other put on it the next round, a layer cut a round apiece, and the
    re-solve holding every crosser could keep neither off it"""
    import route_layers
    import rules as _rules
    if len(route_layers.layers()) <= 2 or not bench:
        return []
    R = _rules.active()
    bar = R.track / 2 + R.clearance + R.grid / 2
    tracks = _tracks_of(bench)[0]
    lanes = plan.get('lanes') or {}
    vends = _via_ends(bench)
    out = []
    for c in cuts:
        if not c.get('firm') or c['island'] in own:
            continue
        x0, y0, x1, y1 = c['box'][:4]
        inb = lambda p, g: x0 - g <= p[0] <= x1 + g and y0 - g <= p[1] <= y1 + g
        # (the copper on each held layer whose box comes within its bar of the cut's)
        near = {L: [s_ for s_ in tracks.get(L, []) if s_[5] not in own
                    and min(s_[0], s_[2]) <= x1 + bar + s_[4] and max(s_[0], s_[2]) >= x0 - bar - s_[4]
                    and min(s_[1], s_[3]) <= y1 + bar + s_[4] and max(s_[1], s_[3]) >= y0 - bar - s_[4]]
                for L in c['blocked']}
        for n, v in lanes.items():
            if n == c['lane'] or not v.get('xy'):
                continue
            if any(inb(p_, bar) and (n, k_) not in vends for k_, p_ in ((0, v['xy'][0]), (1, v['xy'][-1]))):
                continue        # (a lane ENDING there: its end is the fanout's to move, not a layer's to leave)
            pts = [p for p in _dense([list(q) for q in v['xy']], R.grid / 2) if inb(p, bar)]
            blk = sorted(L for L, ss in near.items()
                         if any(nm != n and _seg_pt(p, (a, b), (e, f)) < hw + bar
                                for p in pts for a, b, e, f, hw, nm in ss))
            k = dict(c, lane=n, blocked=blk)
            if pts and blk and k not in cuts and k not in out:
                out.append(k)
        # ...and wherever else a lane's laid path meets THAT net's copper on a held layer: a track is a wall along its
        # whole run, not at the place one lane was found. An earlier bus's lane from the array's face away on In2 took
        # one lane in round 1, held off In2 at the place; another, laid over the same run 2.7 mm north on In1, was put
        # on In2 in round 2 and crossed it there
        r_ = max(ARRAY_CUT_R, 3 * bar)
        run_ = {L: [s_ for s_ in tracks.get(L, []) if s_[5] == c['island']] for L in c['blocked']}
        run_ = {L: ss for L, ss in run_.items() if ss}
        if not run_:
            continue
        rb_ = [min(min(s_[0], s_[2]) - s_[4] for ss in run_.values() for s_ in ss) - bar,
               min(min(s_[1], s_[3]) - s_[4] for ss in run_.values() for s_ in ss) - bar,
               max(max(s_[0], s_[2]) + s_[4] for ss in run_.values() for s_ in ss) + bar,
               max(max(s_[1], s_[3]) + s_[4] for ss in run_.values() for s_ in ss) + bar]
        for n, v in lanes.items():
            if n == c['lane'] or not v.get('xy'):
                continue
            hits_, cl_ = [], []
            for p in _dense([list(q) for q in v['xy']], R.grid / 2):
                if not (rb_[0] <= p[-2] <= rb_[2] and rb_[1] <= p[-1] <= rb_[3]):
                    continue
                ls_ = [L for L, ss in run_.items()
                       if any(_seg_pt(p, (a, b), (e, f)) < hw + bar for a, b, e, f, hw, _nm in ss)]
                if not ls_:
                    continue
                if cl_ and math.hypot(p[-2] - cl_[-1][0][-2], p[-1] - cl_[-1][0][-1]) > 2 * r_:
                    hits_.append(cl_)
                    cl_ = []
                cl_.append((p, ls_))
            if cl_:
                hits_.append(cl_)
            for cl_ in hits_:
                xs_, ys_ = [p[-2] for p, _l in cl_], [p[-1] for p, _l in cl_]
                bx_ = [min(xs_) - r_, min(ys_) - r_, max(xs_) + r_, max(ys_) + r_]
                if any(bx_[0] <= p_[-2] <= bx_[2] and bx_[1] <= p_[-1] <= bx_[3] and (n, k_) not in vends
                       for k_, p_ in ((0, v['xy'][0]), (1, v['xy'][-1]))):
                    continue        # (a lane ENDING there: its end is the fanout's to move)
                k = dict(c, lane=n, blocked=sorted({L for _p, ls_ in cl_ for L in ls_}), box=bx_)
                if k not in cuts and k not in out:
                    out.append(k)
    return out


def _dense(pts, step):
    """a polyline's points `pts` ((u,) x, y rows) with points added every `step` mm of board between them, the leading
    values interpolated too"""
    out = []
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[-2] - a[-2], b[-1] - a[-1])
        m = max(1, int(math.ceil(d / step)))
        out += [[a[j] + (b[j] - a[j]) * i / m for j in range(len(a))] for i in range(m)]
    return out + [list(pts[-1])] if pts else out


def own_spans(cf, geo, plan=None):
    """(more routing layers than two) the cuts of a cut file `cf` held to the stretch of each lane's OWN route that met
    what cut it, read off the geometry's map of its laid lanes (`geo`'s uxy: each lane's columns as route u, x, y). A
    cut is written in route u, and a place in u is a line across the frame: where lanes run steeply across the trunk --
    the zynq's LVDS bus from U1's east face to U5's north ring, the trunk from centre to centre -- a few mm of the lane
    advance it a fraction of that, and a cut over the island's whole box or a via's whole room in u lands on stretches of
    the lane millimetres from the place (a layer cut across five lanes' own teeth, held off their layer from before they
    start; RX_D0's via cut 7 mm from the tooth via window it closed: the solve proved no plan). A VIA cut is held to the
    lane's route within its room of the site (the lane's place at the cut's u), inside the cut it was; a LAYER cut
    carries the `span` of the lane's route where its laid path (`plan`'s, else the geometry's) meets the island's box
    grown by a lane's clearance -- the solve reads it in place of the box's projection. A cut whose lane the map does not
    reach stands as it was"""
    import route_layers
    tab = geo.get('uxy') or {}
    if len(route_layers.layers()) <= 2 or not tab:
        return cf
    import rules as _rules
    R = _rules.active()
    reach = R.track / 2 + R.clearance + R.grid

    def at_u(rows, u):
        for a, b in zip(rows, rows[1:]):
            if a[0] <= u <= b[0]:
                t = (u - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                return a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t
        return None
    dense = {}

    def rows_of(n):
        if n not in dense:
            dense[n] = _dense(tab[n], R.grid / 2) if n in tab else []
        return dense[n]
    out = dict(cf)
    vc = []
    for c in cf.get('vcuts', []):
        rows = rows_of(c['lane'])
        site = at_u(rows, c['u']) if rows else None
        if site is None:
            vc.append(c)
            continue
        i0 = min(range(len(rows)), key=lambda i: abs(rows[i][0] - c['u']))
        lo = hi = i0                    # the run of the lane's route within the via's room of the site, round it
        near = lambda i: math.hypot(rows[i][1] - site[0], rows[i][2] - site[1]) <= c['w']
        while lo > 0 and near(lo - 1):
            lo -= 1
        while hi < len(rows) - 1 and near(hi + 1):
            hi += 1
        a, b = max(rows[lo][0], c['u'] - c['w']), min(rows[hi][0], c['u'] + c['w'])
        vc.append(dict(c, u=round((a + b) / 2, 4), w=round((b - a) / 2, 4)))
    if 'vcuts' in cf:
        out['vcuts'] = vc
    lanes = (plan or geo).get('lanes') or {}
    lc = []
    for c in cf.get('lcuts', []):
        rows, path = rows_of(c['lane']), (lanes.get(c['lane']) or {}).get('xy')
        if not rows or not path or 'box' not in c:
            lc.append(c)
            continue
        x0, y0, x1, y1 = c['box'][:4]
        us = []
        pts_ = _dense([list(q) for q in path], R.grid / 2)
        if c.get('poly'):
            # (a keep-out area: where the path meets its polygon, within a lane's clearance -- not its box)
            import numpy as _np
            import plan_audit as _pa
            dk_ = _pa.keepout_dist(_np.array([p_[-2:] for p_ in pts_], float), _np.array(c['poly'], float),
                                   [_np.array(h_, float) for h_ in c.get('holes') or []]) if pts_ else []
            pts_ = [p_ for p_, d_ in zip(pts_, dk_) if d_ <= reach]
        for p in pts_:
            if x0 - reach <= p[0] <= x1 + reach and y0 - reach <= p[1] <= y1 + reach:
                # (the laid path's place on the lane's route: its nearest column, the polish having moved it little)
                d, u = min((math.hypot(r_[1] - p[0], r_[2] - p[1]), r_[0]) for r_ in rows)
                if d <= 2 * reach:
                    us.append(u)
        lc.append(dict(c, span=[round(min(us), 4), round(max(us), 4)]) if us else c)
    if 'lcuts' in cf:
        out['lcuts'] = lc
    return out


def snap_layer_cuts(snap_log, geo):
    """the LAYER cuts a snap's failures give: a lane it could not lay, its search stuck within SNAP_ISLAND_REACH of an
    island on one layer, the layer it was on there -- the nearest such island"""
    out, bxs = [], geo.get('island_boxes') or {}
    for ln in lines_of(snap_log):
        m = re.match(r'\s*(\S+)\s+FAILED:.* at \(([-\d.]+), ([-\d.]+)\) on ((?:F|B|In\d+)\.Cu)', ln)
        if not m:
            continue
        x, y, L = float(m.group(2)), float(m.group(3)), _layer_index(m.group(4))
        d = lambda b: math.hypot(max(b[0] - x, 0.0, x - b[2]), max(b[1] - y, 0.0, y - b[3]))
        near = sorted((d(b), k) for k, b in bxs.items() if list(b[4]) == [L] and d(b) <= SNAP_ISLAND_REACH)
        if near:
            c = layer_cut(m.group(1), near[0][1], bxs[near[0][1]], L)
            if c and c not in out:
                out.append(c)
    return out


REPAIR_WIDEN = 3             # (LOOP_HOLD) the round's repair freeing the lanes crossing a lane whose cut it broke, so often


def round_lanes(cuts, audit):
    """the lanes a loop round names: every lane of its cut file `cuts` (island, via, layer and crossing cuts) and of
    its audit's findings (`audit`: a row's lanes after its kind -- STATIC, DIVE, PITCH, SHAPE and the rest, and a
    LINT row's)"""
    out = set()
    try:
        c = json.load(open(cuts)) if cuts else {}
        for k_ in ('cuts', 'vcuts', 'lcuts', 'ccuts'):
            for x in c.get(k_) or ():
                for f_ in ('lane', 'a', 'b'):
                    if isinstance(x, dict) and x.get(f_):
                        out.add(x[f_])
    except (OSError, ValueError):
        pass
    for ln in lines_of(audit) if os.path.isfile(audit) else ():
        m_ = re.match(r'^(STATIC|DIVE|PITCH|SHAPE|SWIM|STUB|FOLD|NECK)\s+(\S+)(?:\s+(\S+))?', ln)
        if m_:
            out.add(m_.group(2))
            if m_.group(1) == 'PITCH' and m_.group(3):
                out.add(m_.group(3))
        m_ = re.match(r'^LINT \S+ (\S+)', ln)        # (a lint finding: its kind, then its lane)
        if m_:
            out.add(m_.group(1))
    return {x for x in out if not re.match(r'^[-+]?[\d.]+$', x) and not x.startswith('(') and not x.endswith('(s)')}


def loop(solve, out, rounds, env, log):
    """whole_loop.sh SOLVE OUTDIR ROUNDS, its lines to LOG: 0 with OUTDIR/plan.json the snapped plan that passed, 1 when
    no round got there, 3 when it stopped not converging, 4 when it stopped at crowded ends"""
    env = dict(env)
    # the braid's plan environment the planning reads (a pages-first sidecar's paging, its pairs)
    env.update(BRAID_PAIRS='1', BRAID_EXACT_PAGES='0', PLAN_PAGES_SIDERS='2')
    # every expensive stage through stage_cache.py -- on here, a harness's cache (STAGE_CACHE=0 runs them all)
    env['STAGE_CACHE'] = env.get('STAGE_CACHE') or '1'
    env['TAUT_MEMO'] = env.get('TAUT_MEMO') or '1'
    # (the loop's directory its own: an earlier run's into it left plans, audits and the best round's record that this
    # one read as its own -- the held plan, the feedback's hot files, the firm cuts known broken)
    # (...never with what this loop is to read still in it -- its solve, a seed's cuts, flips or history: a loop asked
    # to run again in the directory its own inputs stand in is refused, not run on what the clearing left)
    solve = os.path.abspath(solve)
    inside_ = [f_ for f_ in [solve] + [x_ for k_ in ('SEED_CUTS', 'SEED_FLIPS', 'SEED_HIST')
                                       for x_ in (env.get(k_) or '').split(',') if x_]
               if os.path.abspath(f_).startswith(os.path.abspath(out) + os.sep)]
    if inside_:
        log(f"=== the loop's directory {out} holds its own input ({os.path.basename(inside_[0])}): give it another")
        return 1
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out, exist_ok=True)
    O = lambda name: os.path.join(out, name)
    st = dict(flips=env.get('SEED_FLIPS', ''), cuts=env.get('SEED_CUTS', ''), hist=env.get('SEED_HIST', ''),
              best=-1, best_i=0, stall=0, solve=solve)
    # (more routing layers than two, a relayed bench: each solve's plan on its own relayed bench -- the bench each
    # round's plans stand on, for the lay: plan_bench)
    benches = {}
    seen = {}

    def bench_is(key):
        benches[key] = env['BENCH']
        json.dump(benches, open(O('benches.json'), 'w'), indent=1)

    def keep_state():
        # (each round's solve and flips, and every cut and history the loop holds: a loop run again on a repaired
        # bench starts at this one's best round with them -- loop_seeds)
        json.dump({'rounds': seen, 'cuts': st['cuts'], 'hist': st['hist']}, open(O('state.json'), 'w'), indent=1)
    # what a command inside the loop leaves unredirected: the round's loop.log in the chain, the terminal on its own
    err = log.f if log.f is not None else sys.stderr
    unredirected = log.f if log.f is not None else INHERIT

    def stage(outs, script, args, logp, **extra):
        """a stage through stage_cache.py, `> LOGP 2>&1`; its exit code. In this process (INPROC) a stage runs
        itself: the stage cache is the harness's, and it ends each stage with atexit's exit functions, which in one
        process are every module's"""
        if script == 'whole_solve.py':
            st['solve_env'] = dict(extra)   # (what the re-solve ran under: the coupled solve builds that model again)
        if INPROC:
            return run([script] + list(args), {**env, **extra}, log_path=logp)[0]
        argv = ['stage_cache.py'] + [a for o in outs for a in ('--out', o)] + ['--', script] + list(args)
        return run(argv, {**env, **extra}, log_path=logp)[0]

    def gate(plan, audit, *more):
        """whole_gate.py PLAN AUDIT [...]: (exit code, stdout), its stderr to the loop's log"""
        rc, so = run(['whole_gate.py', plan, audit] + list(more), env, err=err)
        return rc, stripped(so)

    def addhot(plan, found, hot):
        """an audit's findings as history: its hot places (whole_gate --hot), added when it names any"""
        gate(plan, found, '--hot', hot)
        try:
            if len(json.load(open(hot))['hot']) == 0:
                return False
        except Exception:                                    # (the shell's test reads a python traceback as not "0")
            traceback.print_exc(file=err)
        st['hist'] = (st['hist'] + ',' if st['hist'] else '') + hot
        return True

    def failed(logp):
        for ln in tail(logp, 3):
            log(ln)
        return 1

    def progress(score, i, fresh=False):
        # (fresh: the round found side flips it has not tried -- not a stall, whatever its score)
        if st['best'] < 0 or score < st['best']:
            st['best'], st['best_i'], st['stall'] = score, i, 0
            # (the round a loop that does not pass is laid from: the chain's held plan, held_plan)
            json.dump({'i': i, 'score': score}, open(O('best.json'), 'w'))
        elif not fresh:
            st['stall'] += 1
        if st['stall'] >= PATIENCE:
            log(f"=== round {i}: NOT CONVERGING -- score {score}, the best {st['best']} at round {st['best_i']}, "
                f"{PATIENCE} rounds without a better one")
            raise NotConverging

    def scored(score, i):
        """a round the loop STOPS at, scored as progress scores it and held where it is the best -- no stall counted, the
        stop its caller's own. Stopped unscored, the round before stood as the plan the loop held (held_dives, the chain's
        held_plan) though the stopping round had the fewer findings: its pairs were never tried, and the round before's,
        with a dive the stopping round had moved clear, shipped a pair open"""
        if st['best'] < 0 or score < st['best']:
            st['best'], st['best_i'], st['stall'] = score, i, 0
            json.dump({'i': i, 'score': score}, open(O('best.json'), 'w'))

    def resolve(i):
        """the solve again, warm, with every cut and every audit's history so far -- less an island cut a side flip has
        answered since (the flip puts that lane on the island's other side; the cut would keep holding it off the
        island); None to go on, an exit code when the solve failed"""
        if i >= rounds:
            # ...but not after the LAST round: its plan would be for a round that never runs, and nothing reads it --
            # a loop that did not pass is laid from its rounds' own smooth plans, each on its round's bench (held_plan,
            # lay_rounds). The two-end bus's first chain round: 119 s of solve, for a file no stage opened
            log(f"  (round {i} is the last: no solve again)")
            return None
        s2, cuts_out = O(f's{i + 1}.json'), O(f'cuts{i + 1}.json')
        try:
            fl = lambda fs: {tuple(x[:2]) for f in fs if f and os.path.isfile(f)
                             for x in json.load(open(f)).get('flips', [])}
            flipped = fl(st['flips'].split(','))
            cuts, vcuts, lcuts, ccuts = [], [], [], []
            for f in [f for f in st['cuts'].split(',') if f]:
                try:
                    c = json.load(open(f))
                except (OSError, ValueError):
                    # (one cut file unreadable: the rest merged without it -- it used to fail the whole merge, and
                    # every later re-solve ran with no cuts at all)
                    traceback.print_exc(file=err)
                    continue
                # a cut from the geometry of round k whose flip that geometry had ALREADY been given (a polish of an
                # earlier round found it, or the seed): both sides of the island failed -- the solve hears of it.
                # Only a cut the flip answers, one from before it, is dropped (zynq K42: DQ10's cut at C98, made again
                # after its flip, never reached the solve)
                m = re.search(r'[\\/]c(\d+)\.json$', f)
                k = int(m.group(1)) if m else 0
                given = fl(env.get('SEED_FLIPS', '').split(',') + [os.path.join(out, f'{nm}{j}.json')
                                                                   for j in range(1, k) for nm in ('p', 'q', 'qk')])
                cuts += [x for x in c.get('cuts', []) if (x['lane'], x['island']) not in flipped
                         or (x['lane'], x['island']) in given]
                vcuts += c.get('vcuts', [])
                lcuts += [x for x in c.get('lcuts', []) if x not in lcuts]
                ccuts += [x for x in c.get('ccuts', []) if x not in ccuts]
            json.dump({'cuts': cuts, 'vcuts': vcuts, 'lcuts': lcuts, 'ccuts': ccuts}, open(cuts_out, 'w'))
        except Exception:
            traceback.print_exc(file=err)
        soft = {'SOFT_CUTS': st['soft']} if st.get('soft') else {}
        if env.get('JOINT_DIR') and env.get('JOINT_BUS') and env.get('JOINT_ROUND'):
            # (negotiated buses: the re-solve goes to the coupled solve, which builds its model again -- the buses
            # still routing coupled there, not copper, and those ended now read, as a round's first solve: joint_sync.
            # As copper, another bus's lanes as it last published them -- moved on since -- were hard bans on this
            # bus's via ends and via cuts at its lanes, and the re-solve had no plan)
            soft = {**soft, 'JOINT_FOREIGN': 'finished', 'JOINT_FINISHED': ','.join(sorted(
                f_[:-len('.finished')] for f_ in os.listdir(env['JOINT_DIR']) if f_.endswith('.finished')))}
        import route_layers
        if len(route_layers.layers()) > 2 and (awx_settings.get('LOOP_HOLD') or '1') == '1':
            # (more routing layers than two: FIRST the round's REPAIR -- the re-solve with every lane the round's AUDIT
            # does not name held as the plan before laid it (whole_solve SOLVE_HOLD), the free lanes' cuts firm; taken
            # when it has a plan breaking none of them, else the whole re-solve below. Given the whole model, a group
            # bus's re-solves -- 387 to 456 s each, proving nothing -- moved lanes no finding named, each round new
            # findings of its own, and the loop stopped not converging on its first round's plan. A lane the audit
            # passes is one whose cuts the smooth plan answered -- an audit with no finding sends the round to its
            # passes whatever cuts it has -- so it is held, its cuts with it: freed for its cuts, a group bus's round
            # with ONE finding freed 14 lanes, holding the rest broke their cuts, and it widened to 37, 209 s. Only a
            # round whose audits name no lane frees the lanes the cuts name, as before)
            free_ = round_lanes(None, O(f'p{i}.audit'))
            # (...and, the smooth plan passed and the singles did not fit round the pairs laid first, what those audits
            # name -- the pairs held, and laid again keeping the singles' room. Unread, the smooth audit named nothing,
            # the cut lanes went free, and the diving pair stayed held where it dove: the same finding three rounds
            # running, and the loop stopped not converging)
            for qa_ in (f'q{i}.audit', f'qk{i}.audit'):
                free_ |= round_lanes(None, O(qa_))
            if os.path.isfile(O(f'hs{i}.json')):
                # (the round's SNAPPED plan did not pass: what its audit, its lint and the snap's layer cuts name)
                free_ |= round_lanes(O(f'lc{i}.json'), O('plan.found'))
            if not free_:
                free_ = round_lanes(O(f'c{i}.json'), O(f'p{i}.audit'))
            if os.path.isfile(O(f'dc{i}.json')):
                # (...and every pair the snap could not lay, its dive cut with it (smooth_passes): held, its cut was a
                # held lane's and dropped, and the repair solved another lane again to the same plan, three rounds)
                free_ |= {x['lane'] for x in (json.load(open(O(f'dc{i}.json'))).get('vcuts') or [])}
            if env.get('VBAN_WALLS') and os.path.isfile(env['VBAN_WALLS']):
                # (...and every lane an end of which the relayer kept off the solve's layer, for walling a ball in)
                free_ |= {r_[0] for r_ in json.load(open(env['VBAN_WALLS']))}
            # (a lane whose cut the repair breaks -- its change pinned between its crossings with held lanes -- frees
            # the lanes crossing it, and the repair runs again: REPAIR_WIDEN times at most)
            try:
                cross_ = [k_.split('|') for k_ in (json.load(open(st['solve'])).get('cross') or {})]
            except (OSError, ValueError):
                cross_ = []
            for k_w in range(REPAIR_WIDEN + 1):
                if not free_:
                    break
                hold_ = O(f'hold{i + 1}.json')
                json.dump({'free': sorted(free_)}, open(hold_, 'w'))
                # (the cuts of the FREE lanes alone: a held lane stands as the plan laid it, and a cut of its own there
                # could only read broken)
                hcuts_ = ''
                if os.path.isfile(cuts_out):
                    hcuts_ = O(f'cuts{i + 1}.hold.json')
                    json.dump({k_: ([x for x in v_ if not isinstance(x, dict) or x.get('lane') in free_]
                                    if k_ in ('cuts', 'vcuts', 'lcuts') else v_)
                               for k_, v_ in json.load(open(cuts_out)).items()}, open(hcuts_, 'w'))
                rc_h = stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.held.log', BENCH=env.get('BENCH0') or env['BENCH'],
                             HINT=st['solve'], CUTS=hcuts_, HIST=st['hist'],
                             CUTS_FIRM='1', SOLVE_UNPROVED='1', SOLVE_HOLD=hold_, **soft)
                bl_ = grep(s2[:-5] + '.held.log', r'firm cuts broken')
                broke_ = [int(m_.group(1)) for ln in bl_ for m_ in [re.search(r'broken: (\d+)', ln)] if m_]
                # (a firm cut the round's WHOLE re-solve itself broke before, in this round's loop, no plan holds: a
                # repair breaking only such cuts is the plan the whole solve would give -- a crossing bus's clock pair
                # across an LVDS lane's change, broken by the whole solve, broken again by the repair, and the whole
                # solve run again, 210 s each, to break it once more)
                known_ = firm_known(O('firm_whole.json'))
                brk_ = firm_keys(s2) if rc_h == 0 and os.path.isfile(s2) else None
                if rc_h == 0 and os.path.isfile(s2) and (not any(broke_) or (brk_ and brk_ <= known_)):
                    for ln in grep(s2[:-5] + '.held.log', r'whole_solve|held:|firm cuts'):
                        log('  ' + ln)
                    log(f"  the round's repair: {len(free_)} lane(s) solved again, the rest held"
                        + (' -- the firm cuts it breaks the whole solve broke before' if any(broke_) else ''))
                    took(s2, i)
                    return None
                if os.path.isfile(s2):
                    os.remove(s2)
                named_ = {w_ for ln in bl_ for w_ in re.findall(r'(\S+) (?:via|layer|chan|island)', ln.split('--', 1)[-1])}
                more_ = {b_ for a_, b_ in cross_ if a_ in named_} | {a_ for a_, b_ in cross_ if b_ in named_}
                log(f"  the round's repair ({len(free_)} lane(s) free) "
                    + (f"broke a firm cut of {', '.join(sorted(named_))}" if any(broke_) else 'had no plan')
                    + (f' -- the lanes crossing them freed too ({len(more_ - free_)})' if any(broke_) and more_ - free_
                       and k_w < REPAIR_WIDEN else ' -- the whole solve again'))
                if not any(broke_) or not (more_ - free_):
                    break
                free_ |= more_
        if len(route_layers.layers()) > 2:
            # (more routing layers than two: ONE solve, the cuts FIRM -- each held unless broken at a price above
            # anything a plan can buy, so every cut that can hold does and only those no plan holds break -- keeping
            # a plan it cannot prove, as such a solve almost never proves one. The hard pass first spent four minutes
            # of the zynq's LVDS round proving no plan, for two via cuts of 34 that could not hold together, before
            # the soft pass the loop then ran anyway) -- in WINDOWS from the plan before (whole_solve SOLVE_WINDOWS)
            if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                     HINT=st['solve'], CUTS=cuts_out if os.path.isfile(cuts_out) else '', HIST=st['hist'],
                     CUTS_FIRM='1', SOLVE_UNPROVED='1', SOLVE_WINDOWS='1', **soft) != 0:
                return failed(s2[:-5] + '.log')
            for ln in grep(s2[:-5] + '.log', r'whole_solve|windows of|vias|check|history|firm cuts'):
                log('  ' + ln)
            firm_known(O('firm_whole.json'), firm_keys(s2))
            took(s2, i)
            return None
        if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                 HINT=st['solve'], CUTS=cuts_out, HIST=st['hist'], **soft) != 0:
            # the CUTS left no plan: they are absolute, and together they can ask more than any plan gives -- a net
            # over two vias proved necessary, no plan proved (K41 with the stub check per layer: each of round 1's
            # island cuts and via cuts, alone, proved one; all six island cuts at places the polish had cleared).
            # The solve again with them SOFT (whole_solve SOFT_CUTS: a price where they were a wall) and the history,
            # which carries every place the audits found short -- for this solve and the rounds after (as hard cuts
            # they would leave the same solve with no plan again)
            none = O(f'cuts{i + 1}_none.json')
            json.dump({'cuts': [], 'vcuts': []}, open(none, 'w'))
            if not st['cuts'] or not os.path.isfile(cuts_out):
                # no cuts to soften: the same solve, keeping a plan it cannot prove -- the loop goes on with it rather
                # than ending on the plan it held (a plan for every lane before fewer vias)
                log(f"  no proved plan ({', '.join(tail(s2[:-5] + '.log', 1))[:120]}) -- the solve again, keeping one "
                    f"it cannot prove")
                if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                         HINT=st['solve'], CUTS=cuts_out if os.path.isfile(cuts_out) else none, HIST=st['hist'],
                         SOLVE_UNPROVED='1', **soft) != 0:
                    return failed(s2[:-5] + '.log')
                for ln in grep(s2[:-5] + '.log', r'whole_solve|vias|check|history'):
                    log('  ' + ln)
                took(s2, i)
                return None
            st['soft'] = (st['soft'] + ',' if st.get('soft') else '') + cuts_out
            log(f"  the cuts left no plan ({', '.join(tail(s2[:-5] + '.log', 1))[:120]}) -- the solve again with "
                f"them SOFT, and the history")
            soft = {**soft, 'SOFT_CUTS': st['soft']}
            # (that re-solve keeps a plan it cannot prove: refused, K41's -- the cuts soft, best 52 vias against a
            # bound of 45 -- ended the loop on its held plan, 5 nets open)
            if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                     HINT=st['solve'], CUTS=none, HIST=st['hist'], SOLVE_UNPROVED='1', **soft) != 0:
                return failed(s2[:-5] + '.log')
            st['cuts'] = ''
        for ln in grep(s2[:-5] + '.log', r'whole_solve|vias|check|history'):
            log('  ' + ln)
        took(s2, i)
        return None

    def took(s2, i):
        """the re-solve s2 the loop's plan from here: on a relayed bench (BENCH0), the fanout's own bench relayed
        afresh for its via ends' layers, the bench the stages after it read. Negotiated buses (JOINT_DIR): every
        bus's re-solve coupled first, in step (joint_loop_sync), the couplings it settled held from here"""
        if env.get('JOINT_DIR') and env.get('JOINT_BUS') and env.get('JOINT_ROUND'):
            ok_, cf_ = joint_loop_sync({**env, **st.get('solve_env', {})},
                                       (int(env['JOINT_ROUND']), int(env.get('JOINT_LOOP') or '1'), i), s2, log)
            st['xdone'] = i
            if ok_:
                # (the couplings this coupled solve settled REPLACE those of the exchanges before it: each re-solve is
                # coupled again where the lanes now cross, and the cuts held where they crossed rounds ago stood on
                # places no lane crosses any more -- one bus's grew 380 to 1501 in six rounds, 84 of them broken)
                jd_ = os.path.abspath(env['JOINT_DIR']) + os.sep
                st['cuts'] = ','.join([x_ for x_ in st['cuts'].split(',') if x_ and
                                       not os.path.abspath(x_).startswith(jd_)] + ([cf_] if cf_ else []))
        st['solve'] = s2
        if env.get('BENCH0'):
            env['BENCH'] = relay_ends(out, env['BENCH0'], json.load(open(s2)), log, name=f'bench{i + 1}', env=env)

    def held_dives(i):
        """A loop stopping without a pass (crowded ends, not converging) is laid from the plan it holds (held_plan),
        whose pairs are laid first as the snap moves them: a pair the snap cannot lay at a dive ships open. Such a dive
        goes to the solve as a via cut (whole_snap's dive_cuts), as a passing round's does (smooth_passes), and the loop
        goes on a round -- HELD_DIVES times at most -- and the round after it is held where its pairs all lay. The held
        round's pairs laid first here are the held plan's (held_plan reads them). True when the loop goes on"""
        hd_ = st.get('hd_from')
        k = st['best_i'] if not hd_ or st['best_i'] >= hd_ else i
        if not k or not os.path.isfile(O(f'p{k}.json')):
            return False
        pairs_, pl_ = O(f'hd{k}_pairs.json'), O(f'hd{k}_pairs.log')
        if not os.path.isfile(pairs_):
            if stage([pairs_], 'whole_snap.py', [O(f'p{k}.json'), pairs_, '--pairs'], pl_,
                     BENCH=benches.get(str(k), env['BENCH'])) != 0 and not grep(pl_, r'^SNAP FAILED'):
                return False
        bad_ = grep(pl_, r'^SNAP FAILED')
        if not bad_:
            if k != st['best_i']:
                # (its pairs all laid where the held round's did not: this round the plan the chain lays)
                json.dump({'i': k, 'score': st['best'], 'snap': st['best_i']}, open(O('best.json'), 'w'))
                log(f"  the plan held: round {k}'s, its pairs all laid (round {st['best_i']}'s were not)")
            return False
        # (...the cuts of the pairs stuck AT a dive: within the snap's widest reach of a via, RVIA_WIDE, of one of the
        # plan's. A pair stuck at its start, every dive of it far along the lane, is no dive's to move -- a plain bus
        # in turn, its five pairs stuck within 2 mm of their teeth by the buses before, each dive 15 mm on)
        import braid as bd_
        try:
            hp_ = json.load(open(pairs_))
            pv_ = json.load(open(O(f'p{k}.json'))).get('vias') or []
        except (OSError, ValueError):
            hp_, pv_ = {}, []
        at_ = {c_['lanes'][0]: c_['xy'] for c_ in hp_.get('conflicts', []) if c_.get('lanes')}
        d = [c_ for c_ in hp_.get('dive_cuts', []) if c_['lane'] in at_ and min(
             (math.hypot(v_[1] - at_[c_['lane']][0], v_[2] - at_[c_['lane']][1]) for v_ in pv_ if v_[0] == c_['lane']),
             default=math.inf) <= 6 * bd_.LANE_MIN]
        if not d or st.get('hd', 0) >= HELD_DIVES or i >= rounds:
            log(f"  the held round {k}'s pairs: {bad_[-1].split(': ', 1)[-1]}"
                + ('' if d else ' -- none stuck at a dive of its own'))
            return False
        json.dump(own_spans({'vcuts': d}, json.load(open(O(f'g{k}.json')))), open(O(f'dc{i}.json'), 'w'))
        st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'dc{i}.json')
        st['hd'], st['hd_from'] = st.get('hd', 0) + 1, i + 1
        log(f"=== round {i}: the held round {k}'s pairs: {bad_[-1].split(': ', 1)[-1]} -- the solve again, "
            f"{len(d)} dive(s) moved (via cuts)")
        return resolve(i) is None

    def body(start=1):
        for i in range(start, rounds + 1):
            st['cur'] = i
            if i > 1 and env.get('JOINT_DIR') and env.get('JOINT_BUS') and env.get('JOINT_ROUND') and \
                    st.get('xdone') != i - 1:
                # (negotiated buses: round i - 1 went on without a solve again -- its new side flips, the geometry
                # again on the same solve -- and so without its exchange: said there, that no other bus waits for it)
                joint_skip(env, (int(env['JOINT_ROUND']), int(env.get('JOINT_LOOP') or '1'), i - 1))
            if os.path.exists(O(f'hp{i}.json')):
                os.remove(O(f'hp{i}.json'))    # (a round that passes writes none: an earlier run's must not stand in)
            flips = st['flips']
            bench_is(str(i))
            seen[str(i)] = {'solve': st['solve'], 'flips': flips}
            keep_state()
            log(f"=== round {i}: geometry of {os.path.basename(st['solve'])}"
                + (f" (flips from {os.path.basename(flips.split(',')[-1])})" if flips else ''))
            g, p = O(f'g{i}.json'), O(f'p{i}.json')
            if stage([g], 'whole_geo.py', [st['solve'], g], O(f'g{i}.log'), GEO_FLIPS_FROM=flips) != 0:
                return failed(O(f'g{i}.log'))
            if stage([p], 'whole_polish.py', [g, p], O(f'p{i}.log')) != 0:
                return failed(O(f'p{i}.log'))
            joint_publish(env, p, f'polish {i}')
            if env.get('JOINT_DIR') and env.get('JOINT_BUS') and env.get('JOINT_ROUND'):
                # (negotiated buses: the audit IN STEP -- every bus's round i polished and published first, so the
                # audit reads the others' lanes as they stand this round, not as an earlier round of theirs left them)
                joint_step_sync(env, (int(env['JOINT_ROUND']), int(env.get('JOINT_LOOP') or '1'), i), 'polish', log)
            if stage([], 'whole_audit.py', [p], O(f'p{i}.audit')) != 0:
                return failed(O(f'p{i}.audit'))
            gl = gate(p, O(f'p{i}.audit'))[1]
            log(prefixed(gl, '  smooth: '))
            f = findings(gl)
            before = nflips(flips) if flips else 0
            after = nflips(p)
            # the round's cuts -- the geometry's islands and via cuts, the polish's via cuts -- less an island cut that
            # one of the round's NEW flips answers (the flip puts that lane on the island's other side)
            gj, pj = json.load(open(g)), json.load(open(p))
            old = {tuple(x) for fl in flips.split(',') if fl for x in json.load(open(fl)).get('flips', [])}
            new = {tuple(x) for x in pj.get('flips', [])} - old
            cuts = [c for c in gj.get('cuts', []) if (c['lane'], c['island']) not in new]
            vcuts = gj.get('vcuts', []) + pj.get('vcuts', [])
            # ...and the LAYER cuts: the audit's findings (a lane found against an island not on every layer), and on
            # more routing layers than two the geometry's and the polish's own -- there a layer cut, not a flip, is the
            # answer to such an island, and it stands whatever flip names the lane and island
            import route_layers
            nl_ = len(route_layers.layers())
            lcuts = [c for c in audit_layer_cuts(O(f'p{i}.audit'), gj, env.get('BENCH'), env.get('BUS_AREAS'))
                     if nl_ > 2 or (c['lane'], c['island']) not in new]
            lcuts += place_cuts(lcuts, pj, env.get('BENCH'), set(env.get('NETS', '').split(',')))
            lcuts += [c for c in gj.get('lcuts', []) + pj.get('lcuts', []) if c not in lcuts]
            if lcuts:
                log(f"  layer cuts: {', '.join(c_['lane'] + ' under ' + c_['island'] for c_ in lcuts)}")
            # (probe GEO_ROOM_CUTS: the geometry's capacity cuts -- a layer with no room for the lanes the plan put on it
            # there -- go to the solve as they are; each round's own, the solve holding every round's so far)
            ccuts = gj.get('ccuts', [])
            if ccuts:
                log(f"  room cuts: {', '.join(c_['layer'] + ' ' + str(c_['cap']) + '/' + str(c_['on']) for c_ in ccuts)}")
            json.dump(dict(own_spans({'cuts': cuts, 'vcuts': vcuts, 'lcuts': lcuts}, gj, pj), ccuts=ccuts),
                      open(O(f'c{i}.json'), 'w'))
            n = len(cuts) + len(vcuts) + len(lcuts) + len(ccuts)
            passes = gate(p, O(f'p{i}.audit'))[0] == 0
            if not passes and addhot(p, O(f'p{i}.audit'), O(f'hp{i}.json')):
                n += 1
            # a finding at the ENDS standing in two rounds running: the solve had its round and did not move it, the
            # fanout must (whole_feedback --repeat; the fanout's sidecar beside BENCH) -- stopped here rather than
            # solving again and again. Not while the round has new side flips: a flip is the geometry's own answer
            sidecar = env['BENCH'][:-len('.kicad_pcb')] + '.plan.json' if env['BENCH'].endswith('.kicad_pcb') \
                else env['BENCH'] + '.plan.json'
            newflips = after > before
            # (more routing layers than two: the round's LAYER cuts are its own answer, as a side flip is -- a round with
            # them is not one whose findings at the ends no solve moves)
            answered = newflips or (nl_ > 2 and bool(lcuts))
            hp, hp0 = O(f'hp{i}.json'), O(f'hp{i - 1}.json')
            # (probe LOOP_NOW=0: not at once -- the loop's own rounds first, the two-rounds-running stop below still)
            if (not passes and not answered and os.path.isfile(hp) and os.path.isfile(sidecar)
                    and (awx_settings.get('LOOP_NOW') or '1') == '1'
                    and run(['whole_feedback.py', '--now', sidecar, hp], env, to=unredirected)[0] == 0):
                # ...and at once, when a finding there is one no solve moves (a pitch or a static clearance at the ends)
                scored(2000 + f, i)
                log(f"=== round {i}: ENDS CROWDED -- findings at the ends no solve moves: the fanout's to change")
                if held_dives(i):
                    continue
                return 4
            # (two rounds running, the layer cuts excuse nothing: the finding stood through the solve that had the round
            # before's, so they did not move it. On the two-end USB and Ethernet bus every round had a layer cut at a
            # passive mid-route, and the same five findings at the source's teeth stood through all eleven rounds of
            # two chain rounds -- each round 6.5 min)
            if (not passes and not newflips and i > 1 and os.path.isfile(hp0) and os.path.isfile(hp)
                    and os.path.isfile(sidecar)
                    and run(['whole_feedback.py', '--repeat', sidecar, hp0, hp], env, to=unredirected)[0] == 0):
                scored(2000 + f, i)
                log(f"=== round {i}: ENDS CROWDED -- the same findings at the ends two rounds running: "
                    f"the fanout's to change")
                if held_dives(i):
                    continue
                return 4
            if after > before:
                progress(2000 + f, i, fresh=True)
                st['flips'] = p                       # the polish output carries every flip so far
                if n == 0:
                    log(f"=== round {i}: {after - before} new side flip(s) -> the geometry again on the same solve")
                    continue
                # flips AND cuts or findings: both at once -- the solve with them, then the geometry with the flips
                st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'c{i}.json')
                log(f"=== round {i}: {after - before} new side flip(s), and cuts or findings -> the solve again, "
                    f"then the geometry with the flips")
                rc = resolve(i)
                if rc is not None:
                    return rc
                continue
            if passes:
                rc = smooth_passes(i)
                if rc == 'continue':
                    continue
                return rc
            progress(2000 + f, i)
            if n == 0:
                log(f"=== round {i}: no flips, no cuts and no findings with a place left")
                return 1
            st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'c{i}.json')
            log(f"=== round {i}: cuts or findings -> the solve again")
            rc = resolve(i)
            if rc is not None:
                return rc
        log("=== no round passed")
        return 1

    def smooth_passes(i):
        """the PAIRS first, laid as the pair router moves (its turning radius, its straight dives), then the singles
        fitted round them (the polish, the pairs held) and snapped: an exit code, or 'continue' for the next round"""
        p = O(f'p{i}.json')
        log(f"=== round {i}: the smooth plan passes -> the pairs laid first")
        pairs, pl = O(f'pairs{i}.json'), O(f'pairs{i}.log')
        if stage([pairs], 'whole_snap.py', [p, pairs, '--pairs'], pl) != 0 and not grep(pl, r'^SNAP FAILED'):
            return failed(pl)
        for ln in grep(pl, r'^snap:|FAILED'):
            log('  ' + ln)
        if grep(pl, r'^SNAP FAILED'):
            # a pair the snap cannot lay: its dive nearest where it got stuck goes to the solve as a via cut
            # (whole_snap's dive_cuts), and the solve again; a pair with no dive to move there stops
            nd = None
            try:
                d = json.load(open(pairs)).get('dive_cuts', [])
                json.dump(own_spans({'vcuts': d}, json.load(open(O(f'g{i}.json')))), open(O(f'dc{i}.json'), 'w'))
                nd = len(d)
            except Exception:
                traceback.print_exc(file=err)
            if nd == 0:
                log(f"=== round {i}: a pair cannot be laid")
                return 1
            progress(1500, i)
            if os.path.isfile(O(f'dc{i}.json')):
                # (a cut file is added only once written: one missing failed every merge after it, resolve)
                st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'dc{i}.json')
            log(f"=== round {i}: a pair cannot be laid at a dive -> the solve again, {nd if nd is not None else ''} "
                f"dive(s) moved (via cuts)")
            rc = resolve(i)
            return 'continue' if rc is None else rc
        q = O(f'q{i}.json')
        if stage([q], 'whole_polish.py', [pairs, q], O(f'q{i}.log')) != 0:
            return failed(O(f'q{i}.log'))
        joint_publish(env, q, f'singles round pairs {i}')
        if stage([], 'whole_audit.py', [q], O(f'q{i}.audit')) != 0:
            return failed(O(f'q{i}.audit'))
        gq = gate(q, O(f'q{i}.audit'))[1]
        log(prefixed(gq, '  pairs held: '))
        chosen = q
        if gate(q, O(f'q{i}.audit'))[0] != 0:
            # the singles do not fit round the pairs: the pairs laid AGAIN keeping each single's room where it was
            # short (whole_snap SNAP_KEEP), then the singles fitted again round those
            gate(q, O(f'q{i}.audit'), '--hot', O(f'hk{i}.json'))
            # (a pair that cannot be laid so is no plan: the pairs laid first stand, and their singles' places go to
            # the solve below; any other failure stops)
            pk, pkl = O(f'pairs{i}k.json'), O(f'pairs{i}k.log')
            if stage([pk], 'whole_snap.py', [p, pk, '--pairs'], pkl, SNAP_KEEP=O(f'hk{i}.json')) != 0:
                if not grep(pkl, r'^SNAP FAILED'):
                    return failed(pkl)
                log(f"  pairs laid again, the singles kept room: {chr(10).join(grep(pkl, r'^SNAP FAILED'))}")
            if not grep(pkl, r'^SNAP FAILED'):
                qk = O(f'qk{i}.json')
                if stage([qk], 'whole_polish.py', [pk, qk], O(f'qk{i}.log')) != 0:
                    return failed(O(f'qk{i}.log'))
                if stage([], 'whole_audit.py', [qk], O(f'qk{i}.audit')) != 0:
                    return failed(O(f'qk{i}.audit'))
                gk = gate(qk, O(f'qk{i}.audit'))[1]
                log(prefixed(gk, '  pairs laid again, the singles kept room: '))
                if gate(qk, O(f'qk{i}.audit'))[0] == 0:
                    chosen = qk
        if chosen == q and gate(q, O(f'q{i}.audit'))[0] != 0:
            # the singles do not fit round the pairs: where they are short goes to the solve as history -- and the
            # side flips the polish found with the pairs held go to the next geometry, as a smooth polish's do
            qf = O(f'qk{i}.json') if os.path.isfile(O(f'qk{i}.json')) else q
            o = {tuple(x) for fl in st['flips'].split(',') if fl for x in json.load(open(fl)).get('flips', [])}
            nq = len({tuple(x) for x in json.load(open(qf)).get('flips', [])} - o)
            if nq != 0:
                st['flips'] = (st['flips'] + ',' if st['flips'] else '') + qf
                log(f"  pairs held: {nq} new side flip(s) for the next geometry")
            progress(1000 + findings(gq), i, fresh=nq != 0)
            # (...and the pairs laid again's findings, where they were laid again: theirs are places too)
            if os.path.isfile(O(f'qk{i}.json')) and os.path.isfile(O(f'qk{i}.audit')):
                addhot(O(f'qk{i}.json'), O(f'qk{i}.audit'), O(f'hqk{i}.json'))
            if not addhot(q, O(f'q{i}.audit'), O(f'hq{i}.json')):
                log(f"=== round {i}: the singles do not fit round the pairs")
                return 1
            log(f"=== round {i}: the singles do not fit round the pairs -> the solve again, their places priced")
            rc = resolve(i)
            return 'continue' if rc is None else rc
        # (a single the snap cannot lay leaves the plan without it -- the gate fails it -- and where it got stuck goes
        # to the solve with the audit's findings below; any other failure stops)
        plan, sl = O('plan.json'), O('snap.log')
        bench_is('plan')
        if stage([plan], 'whole_snap.py', [chosen, plan], sl) != 0 and not grep(sl, r'^SNAP FAILED'):
            return failed(sl)
        for ln in grep(sl, r'^snap:|FAILED'):
            log('  ' + ln)
        if os.path.isfile(plan):
            # (each round's snapped plan kept as its own: plan.json is the last round's, and held_plan holds the BEST)
            shutil.copyfile(plan, O(f'plan{i}.json'))
        joint_publish(env, plan, 'snap')
        if stage([], 'whole_audit.py', [plan], O('plan.audit')) != 0:
            return failed(O('plan.audit'))
        gs = gate(plan, O('plan.audit'))[1]
        log(prefixed(gs, '  snapped: '))
        run(['whole_lint.py', plan], env, log_path=O('plan.lint'))
        lint = (tail(O('plan.lint'), 1) or [''])[0]
        log(f"  snapped: {lint}")
        if gate(plan, O('plan.audit'))[0] == 0 and lint == 'LINT clean':
            log(f"=== the plan passes: {plan}")
            return 0
        # the snapped plan is short: where goes to the solve as history -- the audit's places and the lint's (a lane
        # folded at its end: the end it folds at), a finding with no place stops
        folded = sum(1 for ln in lines_of(O('plan.lint')) if re.search(r'^LINT \S+ \S+ ', ln)
                     and not re.search(r'^LINT( [a-z-]+ [0-9]+,?)+$', ln))
        progress(findings(gs) + folded, i)
        with open(O('plan.found'), 'w') as ff:
            ff.write(open(O('plan.audit')).read() + open(O('plan.lint')).read())
        if not addhot(plan, O('plan.found'), O(f'hs{i}.json')):
            log(f"=== round {i}: the snapped plan does not pass")
            return 1
        # ...and a lane the snap could not lay beside an island on one layer: held under it (a LAYER cut)
        lc = snap_layer_cuts(sl, json.load(open(O(f'g{i}.json'))))
        if lc:
            json.dump(own_spans({'lcuts': lc}, json.load(open(O(f'g{i}.json'))), json.load(open(chosen))),
                      open(O(f'lc{i}.json'), 'w'))
            st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'lc{i}.json')
            log(f"  layer cuts: {', '.join(c_['lane'] + ' under ' + c_['island'] for c_ in lc)}")
        log(f"=== round {i}: the snapped plan does not pass -> the solve again, its places priced")
        rc = resolve(i)
        return 'continue' if rc is None else rc

    try:
        start_ = 1
        while True:
            try:
                return body(start_)
            except NotConverging:
                if not held_dives(st['cur']):
                    return 3
                st['stall'], start_ = 0, st['cur'] + 1
    finally:
        keep_state()
        if env.get('JOINT_ROUND'):
            joint_pos(env, (int(env['JOINT_ROUND']), int(env.get('JOINT_LOOP') or '1'), LOOP_END))


# =============================================================================== the chain (whole_chain.sh)
def grade_counts(board, nets):
    """(vias, copper mm) of the run's nets on BOARD, counted as the chain counts them"""
    import math
    if PYR not in sys.path:
        sys.path.insert(0, PYR)
    with contextlib.redirect_stdout(io.StringIO()):
        from kicad_parser import parse_kicad_pcb
        p = parse_kicad_pcb(board)
    from coherent_nets import namer
    sn = namer(p)
    nm = {i: sn(n.name) for i, n in p.nets.items()}
    return (str(sum(1 for v in p.vias if nm.get(v.net_id) in nets)),
            str(round(sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in p.segments
                          if nm.get(s.net_id) in nets))))


def count_open(conn_log):
    """the nets check_connected's log finds open: those with no copper ('Unrouted nets (N)') and those in pieces
    ('Connectivity issues (M)')"""
    n = 0
    for ln in lines_of(conn_log):
        m = re.match(r'\s*(Unrouted nets|Connectivity issues) \((\d+)\)', ln)
        if m:
            n += int(m.group(2))
    return n


# the rows of a board's .kicad_dru length rules, which check_drc grades as KiCad does (#1313, #1303): a net's length,
# and how much shorter it is than its group's longest. Meeting them is a length tuner's work, and the bus step lays
# its lanes untuned (#1363), so they are named apart and never counted against a route or a bus
LENGTH_ROWS = ('length-out-of-range', 'skew-out-of-range')


def standing_hots(d, loopd):
    """the hot files (written beside the audits, `.hard.fbhot.json`) of the findings that STAND in each of a round's
    loops -- `d`/loop on the ends as given, `loopd` the one on repaired ends where there was one: a PITCH or a STATIC
    finding (the ends' own doing) of the loop's last audit that its audit before had too, by its kind and its lanes;
    a loop of one audit: that audit's"""
    out = []
    for ld_ in dict.fromkeys([os.path.join(d, 'loop'), loopd]):
        hs = sorted(glob.glob(os.path.join(ld_, 'p*.fbhot.json')),
                    key=lambda f: int(re.sub(r'\D', '', os.path.basename(f).split('.')[0]) or 0))
        if not hs:
            continue
        key = lambda it: (it[2], tuple(sorted(it[3] if len(it) > 3 else ())))     # noqa: E731
        last = [it for it in json.load(open(hs[-1])).get('hot', []) if it[2] in ('PITCH', 'STATIC')]
        if len(hs) > 1:
            before = {key(it) for it in json.load(open(hs[-2])).get('hot', [])}
            last = [it for it in last if key(it) in before]
        if last:
            f = hs[-1][:-len('.fbhot.json')] + '.hard.fbhot.json'
            json.dump({'hot': last}, open(f, 'w'))
            out.append(f)
    return out


def drc_count(json_path):
    """(violations, length rows) of check_drc's --json: the violations less the rows of the board's length rules
    (LENGTH_ROWS), and those rows; (None, 0) when it wrote none"""
    try:
        with open(json_path, encoding='utf-8') as f:
            J = json.load(f)
    except (OSError, ValueError):
        return None, 0
    n = sum(int((J.get('by_type') or {}).get(t, 0)) for t in LENGTH_ROWS)
    return int(J.get('violations', 0)) - n, n


def open_nets(conn_log, keys=None):
    """the nets check_connected's log finds open: unrouted, or in pieces -- by the whole route's names (`keys`,
    coherent_nets.net_keys), else by their last parts"""
    from coherent_nets import key_of
    out, mode = set(), None
    for ln in lines_of(conn_log):
        s = ln.strip()
        if s.startswith('Unrouted nets'):
            mode = 'u'
        elif s.startswith('Connectivity issues'):
            mode = 'c'
        elif mode == 'u' and s.endswith('pads)'):
            out.add(key_of(s.split(' (')[0], keys))
        elif mode == 'c' and '(net ' in s and s.endswith(':'):
            out.add(key_of(s.split(' (net')[0], keys))
    return out


def firm_keys(plan):
    """the firm cuts the solve's `plan` (its JSON) broke: {each cut's key, as JSON} -- empty where it broke none"""
    try:
        return {json.dumps(x_, sort_keys=True) for x_ in (json.load(open(plan)).get('firm_broken') or [])}
    except (OSError, ValueError):
        return set()


def firm_known(path, add=()):
    """the firm cuts a round's whole re-solves broke, kept in `path` (the loop's): `add` added; the set"""
    try:
        got = set(json.load(open(path)))
    except (OSError, ValueError):
        got = set()
    if add:
        got |= set(add)
        json.dump(sorted(got), open(path, 'w'))
    return got


def relay_ends(d, bench, J, say, name='fo_layers', env=None):
    """(more routing layers than two) the bench with every via end on the layer the round's solve chose for it
    (J['layers'], each lane's first and last run; a pair's every leg) -- relayer.py moving each stub's run from its via
    out, where it stands on another -- written as d/fo_layers.kicad_pcb; `bench` itself when none moves. `env`: where
    a layer banned for walling a ball in is named for the re-solves (VBAN_WALLS)"""
    import pairs as _pairs
    sc_ = os.path.splitext(bench)[0] + '.plan.json'
    if not os.path.isfile(sc_):
        say(f"  (no plan sidecar beside {os.path.basename(bench)}: the via ends stay as laid)")
        return bench
    sc = json.load(open(sc_))
    nets = list(sc.get('ends', {}))
    legs = {b_: list(pr_) for b_, pr_ in _pairs.pair_names(nets).items()}
    if PYR not in sys.path:
        sys.path.insert(0, PYR)          # (the relayer reads and writes boards: called inside the loop's own process too)
    # (the solve's layers are the board's as the braid's setup reads it: a board of the other chirality turned over,
    # F and B swapped -- read back into the sidecar's own frame, or a run kept on the board's B moves onto its F)
    from bga_fanout.flip_frame import other_layer
    OL = other_layer if int(sc.get('chi') or 1) < 0 else (lambda L: L)
    moves = []
    for lane, ls in sorted(J['layers'].items()):
        for k_, L1 in ((0, OL(ls[0])), (1, OL(ls[-1]))):
            for leg in legs.get(lane, [lane]):
                L0 = sc['tooth_layer' if k_ == 0 else 'dest_layer'].get(leg)
                if L0 and L1 != L0 and leg in sc['ends']:
                    moves.append((leg, tuple(sc['ends'][leg][k_]), L0, L1, (lane, k_)))     # (a pair end: one group)
    if not moves:
        return bench
    import relayer
    out = os.path.join(d, name + '.kicad_pcb')
    moved, left = relayer.relayer(bench, out, moves, log=lambda s: say('  ' + s))
    if left:
        say(f"  the plan's layers for {', '.join(sorted(set(left)))} could not be laid: their stubs run to no via")
    # NO BALL WALLED IN by a run moved (SRC_WALLS, as the fanout's realize and berths hold it): where the runs moved
    # onto the solve's layers leave a ball of the arrays on no bus net no way out (route_bus.walls_made, the step's cage
    # test), every end whose run passes within WALL_NEAR of that ball stays on the layer it was laid on -- the board
    # round the ball as the fanout left it, which walled nothing -- and that layer is banned for it from here
    # (VBAN_WALLS: the loop's re-solves read it, whole_solve VBAN); asked again, up to WALL_RETRIES times. A group bus's
    # via ends moved onto the solve's layers took five of its source's edge balls' every way out, each net shipped open.
    # (By the net the cage test names alone -- the nearest whose copper, all of it, frees the ball -- the five ends
    # kept left the five balls walled: the run that closed a ball's last way out was another's)
    if moved and (awx_settings.get('SRC_WALLS') or '1') == '1':
        import route_bus
        import fanout_from_plan as _ffp
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_parser import parse_kicad_pcb
            pcb_ = parse_kicad_pcb(bench)
        from coherent_nets import by_key
        byk_ = by_key(pcb_)
        full_ = {n_: byk_[n_][1].name for n_ in nets if n_ in byk_}
        refs_ = sorted({p_.component_ref for n_ in full_ for p_ in byk_[n_][1].pads
                        if len(pcb_.footprints[p_.component_ref].pads) >= 16})
        runs_ = {}
        for m_ in moves:
            if m_[0] in byk_:
                runs_[m_] = relayer.run_to_via(pcb_, byk_[m_[0]][0], m_[1], m_[2]) or []

        def near_(m_, x, y):
            return any(math.hypot(x - (s_.start_x + t_ * (s_.end_x - s_.start_x)),
                                  y - (s_.start_y + t_ * (s_.end_y - s_.start_y))) < route_bus.WALL_NEAR
                       for s_ in runs_.get(m_, ()) for t_ in (0.0, 0.25, 0.5, 0.75, 1.0)) or \
                math.hypot(x - m_[1][0], y - m_[1][1]) < route_bus.WALL_NEAR
        kept_, keep_ = set(), list(moves)
        # (a ball the round already took as walled -- no plan stood without the layers banned for it, WALLS_STAND: its
        # runs laid as planned, not asked again)
        stand_ = {k_ for k_ in ((env or {}).get('WALLS_STAND') or '').split(',') if k_}
        for _w in range(_ffp.WALL_RETRIES):
            walls_ = {k_: v_ for k_, v_ in route_bus.walls_made(
                bench, out, refs_, list(full_.values()), [full_[n_] for n_ in moved if n_ in full_]).items()
                if k_ not in stand_}
            if not walls_:
                break
            if env is not None:
                env['WALLS_LAST'] = ','.join(sorted(walls_))
            pads_ = {f'{r_}.{p_.pad_number}': p_ for r_ in refs_ for p_ in pcb_.footprints[r_].pads}
            grp_ = {m_[4] for k_ in walls_ if k_ in pads_ for m_ in keep_
                    if near_(m_, pads_[k_].global_x, pads_[k_].global_y)}
            if not grp_:
                break
            kept_ |= grp_
            ends_ = [f'{l_} at its {"tooth" if k_ == 0 else "berth"}' for l_, k_ in sorted(grp_)]
            say(f"  WALLS IN {', '.join(sorted(walls_))} -- the runs moved near them kept where they were laid, their "
                f"layers banned from here: {', '.join(ends_)}")
            keep_ = [m_ for m_ in keep_ if m_[4] not in grp_]
            if not keep_:
                out = bench
                break
            moved, left = relayer.relayer(bench, out, keep_, log=lambda s: say('  ' + s))
        if kept_:
            vb_ = (env or {}).get('VBAN_WALLS') or os.path.join(d, 'vban_walls.json')   # (the round's one file)
            rows_ = json.load(open(vb_)) if os.path.isfile(vb_) else []
            for lane_, k_ in sorted(kept_):
                L1_ = J['layers'][lane_][0 if k_ == 0 else -1]
                if [lane_, k_, L1_] not in rows_:
                    rows_.append([lane_, k_, L1_])
            json.dump(rows_, open(vb_, 'w'))
            if env is not None:
                env['VBAN_WALLS'] = vb_
    return out


def relayed(d, bench, J, env, say, name='fo_layers'):
    """(more routing layers than two) the bench a solve's plan `J` is laid on: its VIA ENDS moved onto the layers the
    solve chose for them (relay_ends), `env` pointed at it (BENCH: the geometry, the polish, the snap, the audits and
    the lay read its copper) and at the fanout's own bench (BENCH0: every RE-SOLVE of the loop is the round's solve
    again, its via ends free -- the same model, so the round's proof floors it and its plan warms it, and a re-solve may
    choose an end's layer anew; loop relays its plan in turn). `bench` itself when none moves, BENCH0 set all the same.
    Held on the relayed bench instead (VIA_ENDS 0), a re-solve was another model: no floor, no warm start, every lane a
    tooth via, and the first solve's end layers stood for the whole loop"""
    if J.get('layers'):
        b2 = relay_ends(d, bench, J, say, name=name, env=env)
        # (BENCH0 the fanout's bench even where nothing moved: the loop's re-solves choose the via ends' layers anew and
        # it relays each one's plan from BENCH0 -- unset, a re-solve that moved an end was laid on stubs still on the
        # layers before, its tooth or berth closed on the plan's: an FFC bus's two corner pairs TOOTH-OUT every round)
        env.update(BENCH=b2, BENCH0=bench)
        return b2
    return bench


def loop_seeds(loopd):
    """where a stopped loop got (its state.json and best.json), for the loop run again on its repaired bench: (the
    solve its best round planned from, {SEED_CUTS, SEED_HIST: every cut and history it held, SEED_FLIPS: that round's
    flips}). Run again from the round's first solve, a loop found its cuts again or did not: a DRAM bus's, whose best
    round failed only on the 0402 moved off its lane, held one dive three rounds on the repaired bench and laid 1 open"""
    k = json.load(open(os.path.join(loopd, 'best.json')))['i']
    s_ = json.load(open(os.path.join(loopd, 'state.json')))
    r_ = s_['rounds'][str(k)]
    return r_['solve'], {'SEED_CUTS': s_['cuts'], 'SEED_FLIPS': r_['flips'], 'SEED_HIST': s_['hist']}


def repair_ends(d, loopd, env, nets, say):
    """A loop stopped at CROWDED ENDS on STATIC findings (a lane too close, at its tooth, to copper the solve cannot
    move) has the round's fanout bench repaired where they stand, one lane at a time, before the round gives the ends
    to the next round's fanout: what the finding names, read off the stopping round's audit (STATIC LANE LAYER d/bar
    KIND OBJECT at (x,y): a track of OBJECT on that layer, its via on every layer, a pad, a hole, a keep-out), is
    - another net's via (a plane ball's drop), unlocked: moved off the point (ends_nudge --via-at), a pair's end too;
    - a bus lane's own escape -- its via, or its stub's track on that layer -- unlocked: nudged off it (ends_nudge);
    - anything else right past the tooth (another net's track or pad on the tooth's layer, a locked via on every
      layer): the lane's own ball escaped again (ends_reescape), off that layer when the finding is on the tooth's own
      -- a lane that dives right past its tooth meets the copper on the layer it dives to, and the escape that clears it
      may be the one ON that layer.
    - a movable passive's pad (a two-pad C/R/FB part by an array) at either end, a pair's dive against it too: the
      part moved a short way off the lane's copper (ends_passive).
    A pair's own end otherwise, and at the berths anything but another net's via or such a part, are left to the next
    round. Each move is checked
    on the fanout's own obstacle model and chained on the last one's bench; a nudge refused for copper already clear as
    laid (the finding is another copper's) moves nothing. Measured on DDR and two-end buses on four and eight layers,
    the loop run again on the repaired ends: a round that stopped crowded passed in that round with fewer vias (a GND
    drop at the berths, a lane's own via 0.05 mm), and two lanes left open by a tooth against an earlier bus's locked
    track closed (its ball escaped to the via-in-pad 0.375 mm along its face). (the repaired bench, the loop directory
    for its new loop), or None when nothing moved"""
    if PYR not in sys.path:
        sys.path.insert(0, PYR)
    from kicad_parser import parse_kicad_pcb
    # (the lanes whole_feedback named at the stop: '  STATIC|DIVE LANE[/LANE] at the teeth|berths')
    named = [(ln_, w_) for l_, w_ in re.findall(r'^  (?:STATIC|DIVE) (\S+) at the (teeth|berths)',
                                                 open(os.path.join(d, 'loop.log')).read(), re.M) for ln_ in l_.split('/')]
    if not named:
        return None
    # (the stopping round's audit: the round of the last hot file, whose findings whole_feedback read)
    hps = sorted(glob.glob(os.path.join(loopd, 'hp[0-9]*.json')), key=lambda f_: int(re.sub(r'\D', '', os.path.basename(f_))))
    i_ = re.sub(r'\D', '', os.path.basename(hps[-1])) if hps else None
    # (...but the round the loop GOES ON FROM, where it holds another: the loop run again starts at its best round's
    # solve (loop_seeds), and the bench is repaired for THAT plan's copper. A pair's dive stood half a millimetre from
    # one round's geometry to the next at one route place; the cap moved clear of the stopping round's dive was moved
    # into the held round's, whose pairs had all laid, and the pair shipped open)
    try:
        b_ = str(json.load(open(os.path.join(loopd, 'best.json')))['i'])
        if os.path.isfile(os.path.join(loopd, f'p{b_}.audit')):
            i_ = b_
    except (OSError, ValueError, KeyError):
        pass
    aud = os.path.join(loopd, f'p{i_}.audit') if i_ else None
    if not aud or not os.path.isfile(aud):
        say('  crowded ends: no audit of the stopping round')
        return None
    rows = []
    for ln in lines_of(aud):
        m_ = re.match(r'^STATIC (\S+)\s+([FB]|In\d+\.Cu) ([-+]?[\d.]+)/([\d.]+) (copper|via|pad|hole|keepout) (.+?) '
                      r'at \(([-\d.]+),\s*([-\d.]+)\)', ln)
        if m_:
            L_ = {'F': 'F.Cu', 'B': 'B.Cu'}.get(m_.group(2), m_.group(2))
            rows.append((float(m_.group(3)) - float(m_.group(4)), m_.group(1), L_, m_.group(5), m_.group(6).split(),
                         (float(m_.group(7)), float(m_.group(8))), False))
        # (...and a pair's DIVE short of static copper: 'DIVE LANE (x,y) pair static d/bar (KIND OBJ ...)', on every
        # layer -- its barrels stand through them all)
        m_ = re.match(r'^DIVE (\S+)\s+\(\s*([-\d.]+),\s*([-\d.]+)\)\s+pair\s+static ([-+]?[\d.]+)/([\d.]+) '
                      r'\((copper|via|pad|hole|keepout) ([^)]+)\)', ln)
        if m_:
            rows.append((float(m_.group(4)) - float(m_.group(5)), m_.group(1), None, m_.group(6), m_.group(7).split(),
                         (float(m_.group(2)), float(m_.group(3))), True))
    dive_off = next((float(m_.group(1)) for ln in lines_of(aud)
                     for m_ in [re.search(r'two barrels ([\d.]+) either side', ln)] if m_), None)
    bench0 = env.get('BENCH0') or env['BENCH']
    side = json.load(open(os.path.splitext(bench0)[0] + '.plan.json'))
    # (the audit's points are the plan's: on a board of the other chirality turned over, mirrored back to the board's)
    # (...and its layers the plan's too: turned over, F and B change places -- the board's layer is the other one)
    CY, OL = None, (lambda L_: L_)
    if int(side.get('chi', 1)) < 0:
        from bga_fanout.flip_frame import mirror_axis, other_layer
        CY, OL = mirror_axis(parse_kicad_pcb(bench0)), other_layer
    # (the stopping round's OWN bench -- on more routing layers than two each re-solve's ends relayed afresh, bench{i}:
    # the copper the audit measured stands there, not on the round's first relay)
    bench_r = plan_bench(loopd, f'p{i_}.json', env['BENCH'])
    pcbB = parse_kicad_pcb(bench_r)
    from coherent_nets import namer
    sn = namer(pcbB)
    n2n = {i: sn(n.name) for i, n in pcbB.nets.items()}
    bus = set(nets)

    def seg_d(p, a, b):
        L2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / L2))
        return math.hypot(a[0] + t * (b[0] - a[0]) - p[0], a[1] + t * (b[1] - a[1]) - p[1])
    # (a lane's rows AT THE END whole_feedback named -- in the box of the run's ends there (every lane's teeth, or its
    # berths, on the bench's sidecar) grown by the front of a face, whole_feedback.FRONT: its worst row anywhere nudged
    # a tooth for a finding mid-route or at the other end, an exit 4 that moved nothing that was found)
    import braid as _bd_
    reach_ = 2 * _bd_.VIA_NEED + _bd_.LANE_MIN
    bxy_ = lambda q: (q[0], (2 * CY - q[1]) if CY is not None else q[1])     # noqa: E731

    # (ends_near: whole_feedback's own test. A box of the ends grown by the front of a face, the repair's test before,
    # stopped short of the feedback's beside a side face, where a finding counts within the width the face's lanes
    # stack to: a memory lane's tooth 1.3 mm from a flash part's pad beside the array was named at the teeth, the
    # repair found "no STATIC row of it there", and the loop ran again on the same ends to the same finding)
    import ends_near as _en
    ends_ = {n_: e_ for n_, e_ in (side.get('ends') or {}).items() if e_ and len(e_) > 1}
    bxs_ = _en.boxes(ends_, side.get('dest_of') or {}) if ends_ else None
    cur, k, moved, blocked = bench0, 0, [], []
    for lane, where in dict.fromkeys(named):
        mine = sorted(r_ for r_ in rows if r_[1] == lane)
        if not mine:
            say(f"  crowded ends: {lane} at the {where}: no STATIC row of it in {os.path.basename(aud)}")
            continue
        if bxs_ is not None:
            mine = [r_ for r_ in mine if _en.near(ends_, bxs_, 0 if where == 'teeth' else 1, *bxy_(r_[5]), reach_,
                                                  _bd_.LANE_MIN)]
            if not mine:
                say(f"  crowded ends: {lane} at the {where}: no STATIC row of it there in {os.path.basename(aud)}")
                continue
        short, _, L, kind, obj, p0, dive = mine[0]             # (its worst row)
        pp_ = p0
        p0 = (p0[0], (2 * CY - p0[1]) if CY is not None else p0[1])
        L = OL(L) if L else None
        net = obj[-1] if kind in ('pad', 'hole') else obj[0]
        label = f"{'dive ' if dive else ''}{kind} {' '.join(obj)} on {L or 'every layer'} {short:+.3f}"
        if kind == 'pad' and '.' in obj[0]:
            # A PASSIVE's pad (a two-pad C/R/FB part by an array, unlocked: the cap step's own): the part moved off
            # the copper the lane needs there (ends_passive) -- a decoupling cap in front of the destination's face,
            # where its pairs dive to their berths, moves where no solve moves the dive past it. Only what the cap step
            # itself would move; anything else goes on below as before
            pref_ = obj[0].split('.', 1)[0]
            flags_ = []
            if dive and dive_off:
                xy_ = (json.load(open(os.path.join(loopd, f'p{i_}.json'))).get('lanes', {}).get(lane) or {}).get('xy')
                if xy_ and len(xy_) > 1:
                    k_ = min(range(len(xy_) - 1), key=lambda q_: seg_d(pp_, xy_[q_], xy_[q_ + 1]))
                    ux_, uy_ = xy_[k_ + 1][0] - xy_[k_][0], xy_[k_ + 1][1] - xy_[k_][1]
                    flags_ = [f'--dive={dive_off:.4f},{ux_:.4f},{-uy_ if CY is not None else uy_:.4f}']
            if not dive or flags_:
                out_ = os.path.join(d, f'fo_ends{k}.kicad_pcb')
                rc_ = run(['ends_passive.py', out_, pref_, f'{p0[0]:.4f},{p0[1]:.4f}'] + flags_,
                          {**env, 'BENCH0': cur, 'BENCH': bench_r}, log_path=os.path.join(d, f'ends{k}.log'),
                          own_process=True)[0]
                got = grep(os.path.join(d, f'ends{k}.log'), r'^ends_passive: .*(moved|not moved|no passive|grazes|exited)')
                k += 1
                if rc_ != 2:
                    say(f"  crowded ends: {lane} at the {where}: {label} -- "
                        + (got[-1].split(': ', 1)[1][:150] if got else f'exit {rc_}'))
                    if rc_ == 0 and os.path.isfile(out_):
                        cur = out_
                        moved.append(lane)
                    continue
        if dive:
            say(f"  crowded ends: {lane} at the {where}: {label} -- left to the next round")
            continue
        locked, at = False, None
        if kind == 'via':
            vv = [v for v in pcbB.vias if n2n.get(v.net_id) == net]
            if vv:
                v = min(vv, key=lambda v_: math.hypot(v_.x - p0[0], v_.y - p0[1]))
                locked, at = v.locked, (v.x, v.y)
        elif kind == 'copper':
            ss = [s_ for s_ in pcbB.segments if n2n.get(s_.net_id) == net and s_.layer == L]
            if ss:
                locked = min(ss, key=lambda s_: seg_d(p0, (s_.start_x, s_.start_y), (s_.end_x, s_.end_y))).locked
        label += ' (locked)' if locked else ''
        tool = 'ends_nudge.py'
        if kind == 'via' and at is not None and not locked and net not in bus:
            who, flags = lane, [f'--via-at={at[0]:.4f},{at[1]:.4f}']
        elif lane not in bus or where != 'teeth' or kind == 'keepout':
            say(f"  crowded ends: {lane} at the {where}: {label} -- left to the next round")
            continue
        elif kind == 'via' and at is not None and not locked:
            who, flags = net, ['--kind=via']
        elif kind == 'copper' and not locked and net in bus:
            who, flags = net, ['--kind=track', f'--layer={L}']
        else:
            tl_ = (side.get('tooth_layer') or {}).get(lane)
            tool, who, flags = 'ends_reescape.py', lane, ([f'--avoid={L}'] if kind in ('copper', 'pad') and L == tl_
                                                           else [])
        out_ = os.path.join(d, f'fo_ends{k}.kicad_pcb')
        args_ = [out_, who] + ([f'{p0[0]:.4f},{p0[1]:.4f}'] if tool == 'ends_nudge.py' else []) + flags
        rc_ = run([tool] + args_, {**env, 'BENCH0': cur, 'BENCH': bench_r}, log_path=os.path.join(d, f'ends{k}.log'),
                  own_process=True)[0]
        got = grep(os.path.join(d, f'ends{k}.log'), r'^ends_(nudge|reescape): .*(moved|NO |re-escaped|no escape|goal is met)')
        say(f"  crowded ends: {lane} at the {where}: {label} -- "
            + (got[-1].split(': ', 1)[1][:150] if got else f'exit {rc_}'))
        if rc_ == 0 and os.path.isfile(out_):
            cur = out_
            moved.append(lane)
        elif tool == 'ends_reescape.py' and any('NO escape clears' in g_ for g_ in got):
            blocked.append(lane)
        k += 1
    if blocked:
        put_blocked(os.path.join(os.path.dirname(d), BLOCKED_ENDS), side, blocked, say)
    if not moved:
        return None
    return cur, os.path.join(d, 'loop_ends')


PASSIVE_MOVES = 3              # parts repair_passives moves for a stopped loop at most


def repair_passives(d, loopd, env, say):
    """A loop that stopped WITHOUT PASSING (not converging, or at crowded ends repair_ends moved nothing for) on a
    finding against a MOVABLE PASSIVE's pad -- a lane, or a pair's dive, too close to a pad of a two-pad C/R/FB part by
    an array, in its last two rounds' audits (smooth, pairs held, pairs laid again) -- has that part moved a short way
    off the copper the lane needs there (ends_passive, which moves only what the cap step itself would), PASSIVE_MOVES
    parts at most, nearest the finding's own round first, and the loop run again on the repaired bench: no solve moves
    a part, and the loop stopped with it in the way (an 0402 termination resistor beside the destination, a DRAM data
    lane 0.07 mm short of its pad, the same finding three rounds running). (the repaired bench, the loop directory for
    its new loop), or None when nothing moved"""
    from kicad_parser import parse_kicad_pcb
    rounds_ = sorted({int(m_.group(1)) for f_ in glob.glob(os.path.join(loopd, '*.audit'))
                      for m_ in [re.search(r'[\\/](?:p|q|qk)(\d+)\.audit$', f_)] if m_})
    found, dive_off = {}, None
    for i_ in rounds_[-2:]:
        for nm_ in (f'p{i_}', f'q{i_}', f'qk{i_}'):
            aud = os.path.join(loopd, nm_ + '.audit')
            if not os.path.isfile(aud):
                continue
            for ln in lines_of(aud):
                m_ = re.search(r'two barrels ([\d.]+) either side', ln)
                if m_:
                    dive_off = float(m_.group(1))
                m_ = re.match(r'^STATIC (\S+)\s+(?:[FB]|In\d+\.Cu) ([-+]?[\d.]+)/([\d.]+) pad (\S+)\.\S+ \S+ '
                              r'at \(([-\d.]+),\s*([-\d.]+)\)', ln)
                if m_:
                    found[m_.group(4)] = dict(lane=m_.group(1), short=float(m_.group(2)) - float(m_.group(3)),
                                              at=(float(m_.group(5)), float(m_.group(6))), dive=False, i=i_, plan=nm_)
                    continue
                m_ = re.match(r'^DIVE (\S+)\s+\(\s*([-\d.]+),\s*([-\d.]+)\)\s+pair\s+static ([-+]?[\d.]+)/([\d.]+) '
                              r'\(pad (\S+)\.\S+', ln)
                if m_:
                    found[m_.group(6)] = dict(lane=m_.group(1), short=float(m_.group(4)) - float(m_.group(5)),
                                              at=(float(m_.group(2)), float(m_.group(3))), dive=True, i=i_, plan=nm_)
    # (a two-pad passive's reference: letters then a number -- an array's ball names its part U.., a pin a connector's)
    found = {r_: f_ for r_, f_ in found.items() if re.match(r'^(C|R|FB|L)\d+$', r_)}
    if not found:
        return None
    bench0 = env.get('BENCH0') or env['BENCH']
    side = json.load(open(os.path.splitext(bench0)[0] + '.plan.json'))
    CY = None
    if int(side.get('chi', 1)) < 0:
        from bga_fanout.flip_frame import mirror_axis
        with contextlib.redirect_stdout(io.StringIO()):
            CY = mirror_axis(parse_kicad_pcb(bench0))

    def seg_d(p, a, b):
        L2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / L2))
        return math.hypot(a[0] + t * (b[0] - a[0]) - p[0], a[1] + t * (b[1] - a[1]) - p[1])
    cur, moved, k = bench0, [], 0
    for ref_, f_ in sorted(found.items(), key=lambda kv: (-kv[1]['i'], kv[1]['short']))[:PASSIVE_MOVES]:
        p0 = (f_['at'][0], (2 * CY - f_['at'][1]) if CY is not None else f_['at'][1])
        flags_ = []
        if f_['dive']:
            pf_ = os.path.join(loopd, f_['plan'] + '.json')
            xy_ = (json.load(open(pf_)).get('lanes', {}).get(f_['lane']) or {}).get('xy') \
                if os.path.isfile(pf_) else None
            if not (xy_ and len(xy_) > 1 and dive_off):
                continue
            q_ = min(range(len(xy_) - 1), key=lambda q: seg_d(f_['at'], xy_[q], xy_[q + 1]))
            ux_, uy_ = xy_[q_ + 1][0] - xy_[q_][0], xy_[q_ + 1][1] - xy_[q_][1]
            flags_ = [f'--dive={dive_off:.4f},{ux_:.4f},{-uy_ if CY is not None else uy_:.4f}']
        out_ = os.path.join(d, f'fo_parts{k}.kicad_pcb')
        rc_ = run(['ends_passive.py', out_, ref_, f'{p0[0]:.4f},{p0[1]:.4f}'] + flags_,
                  {**env, 'BENCH0': cur, 'BENCH': plan_bench(loopd, f"p{f_['i']}.json", env['BENCH'])},
                  log_path=os.path.join(d, f'parts{k}.log'), own_process=True)[0]
        got = grep(os.path.join(d, f'parts{k}.log'), r'^ends_passive: .*(moved|not moved|no passive|grazes|exited)')
        k += 1
        if rc_ == 2:
            continue
        say(f"  a stopped loop's part: {f_['lane']} {'dive ' if f_['dive'] else ''}against {ref_} "
            f"{f_['short']:+.3f} (round {f_['i']}) -- " + (got[-1].split(': ', 1)[1][:150] if got else f'exit {rc_}'))
        if rc_ == 0 and os.path.isfile(out_):
            cur = out_
            moved.append(ref_)
    if not moved:
        return None
    return cur, os.path.join(d, 'loop_ends')


def repair_layers(d, crowded_f, env, nets, say):
    """A round whose solve has NO PLAN, the lanes its RULES core names for a TOOTH's fixed layer (whole_solve's no-plan
    diagnosis, OUT.crowded.json's 'rules': 'toothlay') -- a surface tooth whose lane must leave its layer before a
    crossing, with no room for the change between them -- each such lane's ball escaped again OFF that layer
    (ends_reescape --avoid: its own menu's escape nearest the old tooth on its face, clear past its exit), so the change
    is its tooth's via: the fanout bench repaired before the round gives up on its ends. A memory bus's edge-ball data
    line, its surface tooth on F.Cu crossing four lanes within 0.3 mm of route and another with its berth on F.Cu: no
    plan; the 4-round run laid it only in its fourth round, its tooth a via in its pad. A pair, or a lane whose berth's
    layer is the one that cannot hold, is left to the rounds after. The repaired bench, or None when nothing moved"""
    try:
        rules = json.load(open(crowded_f)).get('rules') or []
    except (OSError, ValueError):
        return None
    lanes = sorted({l_ for l_, f_ in rules if f_ == 'toothlay'})
    if not lanes:
        return None
    cur = env.get('BENCH0') or env['BENCH']
    side = json.load(open(os.path.splitext(cur)[0] + '.plan.json'))
    tl = side.get('tooth_layer') or {}
    moved = []
    for k, lane in enumerate(lanes):
        if lane not in tl or lane not in nets:
            say(f"  no plan: {lane}'s tooth layer cannot hold -- not a single lane's tooth here, left to the next round")
            continue
        out_ = os.path.join(d, f'fo_layer{k}.kicad_pcb')
        rc_ = run(['ends_reescape.py', out_, lane, f'--avoid={tl[lane]}'], {**env, 'BENCH0': cur},
                  log_path=os.path.join(d, f'layer{k}.log'), own_process=True)[0]
        got = grep(os.path.join(d, f'layer{k}.log'), r'^ends_reescape: .*(re-escaped|no escape|NO |laid)')
        say(f"  no plan: {lane}'s tooth on {tl[lane]} cannot hold its layer -- "
            + (got[-1].split(': ', 1)[1][:160] if got else f'exit {rc_}'))
        if rc_ == 0 and os.path.isfile(out_):
            cur = out_
            moved.append(lane)
    return cur if moved else None


BLOCKED_ENDS = 'blocked_ends.json'      # (the run's directory: the teeth no escape cleared -- put_blocked)


def put_blocked(path, side, lanes, say=print):
    """the teeth of `lanes` on the round's plan sidecar `side` -- teeth repair_ends found no escape of their ball's
    clears (another net's copper on the tooth's way out it cannot move: an earlier bus's locked lane) -- added to the
    run's BLOCKED_ENDS, every later round's ENDS_BLOCKED (whole_ends.round_feedback): each an end to avoid, priced as a
    front with no room, for the rounds under another seed as for a feedback round"""
    import pairs as _pairs
    fb = json.load(open(path)) if os.path.isfile(path) else {'avoid': []}
    have = {(it['lane'], json.dumps(it['points'])) for it in fb['avoid']}
    prs = _pairs.pair_names(list(side.get('ends') or {}))
    new = []
    for lane in lanes:
        legs = [l_ for l_ in (prs.get(lane) or (lane,)) if l_ in (side.get('ends') or {})]
        if not legs:
            continue
        it = {'lane': lane, 'end': 0, 'points': [list(side['ends'][l_][0]) for l_ in legs],
              'layer': (side.get('tooth_layer') or {}).get(legs[0]), 'blocked': True}
        if (lane, json.dumps(it['points'])) not in have:
            fb['avoid'].append(it)
            new.append(lane)
    if new:
        json.dump(fb, open(path, 'w'), indent=1)
        say(f"  blocked teeth, avoided by every later round: {', '.join(new)}")


def plan_bench(loopd, plan, default):
    """the bench a loop's plan stands on (its round's relayed bench, loop's benches.json), else `default`"""
    try:
        b = json.load(open(os.path.join(loopd, 'benches.json')))
    except (OSError, ValueError):
        return default
    nm = os.path.basename(plan or '')
    if nm == 'plan.json':
        return b.get('plan', default)
    if nm.startswith('held_'):
        return b.get('held', default)            # (held_plan's: the bench of the round it held)
    m = re.match(r'^[a-z]+(\d+)', nm)
    return b.get(m.group(1), default) if m else default


def held_plan(loopd, env, i=None):
    """the plan a round whose loop did not pass is laid from -- the chain never ends with nothing: the loop's snapped
    plan when a round got that far (plan.json, short of the audit or the lint), else its best round's smooth plan
    (best.json, else its last) laid as a passing one is -- the pairs snapped first, the singles polished round them,
    then snapped -- with no gate; a lane the snap cannot lay is left out of it. None when the loop held none.
    Snapped and laid on THAT round's bench (benches.json; recorded under 'held' for plan_bench): on more routing layers
    than two each re-solve relays the via ends afresh, and a later round's plan snapped and laid on round 1's bench
    started its lanes at stubs on other layers (zynq LVDS zDF r2, a later round held: eight lanes refused within a few
    steps of their teeth or berths, 33 nets open; on its own bench 13).
    `i`: loop round i's smooth plan so, into held{i}_*.json (LAY_TRY: lay_rounds), plan_bench finding its bench
    by the round's number"""
    O = lambda name: os.path.join(loopd, name)
    pre = 'held_' if i is None else f'held{i}_'
    if i is None:
        # (the BEST round's snapped plan, plan{i}.json -- a round that got to the snap scores below any that did not,
        # so the best round is a snapped one wherever any was. plan.json alone is the LAST round's: held so, a loop
        # whose round 2 snapped with two findings and round 4 with nine laid round 4, and the lays after it left the
        # best round out as the one laid)
        try:
            bi_ = int(json.load(open(O('best.json')))['i'])
        except (OSError, ValueError, KeyError, TypeError):
            bi_ = None
        if bi_ is not None and os.path.isfile(O(f'plan{bi_}.json')):
            return O(f'plan{bi_}.json')
        if os.path.isfile(O('plan.json')):
            return O('plan.json')
    # (either separator: glob and os.path.join give '\' on Windows, and a '/'-only match there held no round at all)
    ps = sorted((f for f in glob.glob(O('p*.json')) if re.search(r'[\\/]p\d+\.json$', f)),
                key=lambda f: int(re.search(r'[\\/]p(\d+)\.json$', f).group(1)))
    if not ps:
        return None
    src = ps[-1]
    if i is not None:
        src = O(f'p{i}.json')
        if not os.path.isfile(src):
            return None
    else:
        try:
            b = O(f"p{json.load(open(O('best.json')))['i']}.json")
            if os.path.isfile(b):
                src = b
        except Exception:
            pass
    try:
        bs = json.load(open(O('benches.json')))
    except (OSError, ValueError):
        bs = {}
    rb = bs.get(re.search(r'[\\/]p(\d+)\.json$', src).group(1))
    if rb:
        env = dict(env, BENCH=rb)
        if i is None:
            bs['held'] = rb
            json.dump(bs, open(O('benches.json'), 'w'), indent=1)
    m_ = re.search(r'[\\/]p(\d+)\.json$', src)
    hd_ = O(f'hd{m_.group(1)}_pairs.json') if m_ else None
    if hd_ and os.path.isfile(hd_) and not grep(hd_[:-len('.json')] + '.log', r'^SNAP FAILED'):
        # (the loop's own pairs of that round, every one laid at its stop on the same bench: loop's held_dives)
        shutil.copyfile(hd_, O(pre + 'pairs.json'))
        shutil.copyfile(hd_[:-len('.json')] + '.log', O(pre + 'pairs.log'))
    else:
        run(['whole_snap.py', src, O(pre + 'pairs.json'), '--pairs'], env, log_path=O(pre + 'pairs.log'))
    if os.path.isfile(O(pre + 'pairs.json')) and \
            run(['whole_polish.py', O(pre + 'pairs.json'), O(pre + 'q.json')], env, log_path=O(pre + 'q.log'))[0] == 0:
        src = O(pre + 'q.json')
    run(['whole_snap.py', src, O(pre + 'plan.json')], env, log_path=O(pre + 'plan.log'))
    return O(pre + 'plan.json') if os.path.isfile(O(pre + 'plan.json')) else None


def snap_failed(loopd, plan):
    """the NETS a held plan's snap left unlaid (held_plan's logs: each snap's last word, SNAP FAILED -- a lane a sweep
    laid after it failed once is laid; a pair is two nets): zKP K40 r1, rounds 3 and 1 each with six FAILED lines, laid
    10 and 6 open -- four pairs among the one's, one among the other's"""
    m = re.search(r'[\\/](held\d*_)plan\.json$', plan or '')
    if not m:
        return None
    n = 0
    for f, w in ((m.group(1) + 'pairs.log', 2), (m.group(1) + 'plan.log', 1)):
        f = os.path.join(loopd, f)
        if os.path.isfile(f):
            for ln in open(f, errors='replace'):
                mm = re.match(r'SNAP FAILED: \d+ lane\(s\) could not be laid: (.*)$', ln.strip())
                if mm:
                    n += w * len([x for x in mm.group(1).split(',') if x.strip()])
    return n


def pick_by_snap(loopd, env, plan, say):
    """(probe LOOP_PICK_SNAP=N: a loop that did not pass holds the round its SNAP lays best, not its findings.) The held
    plan (held_plan, snapped) and up to N-1 of the loop's other rounds, fewest findings first, each snapped as the held
    one is; the plan whose snap left the fewest lanes unlaid is the one laid, the held one on a tie. Measured on the
    zynq LVDS's rounds where two rounds were laid (LAY_TRY): the snap's failures ranked the two lays right in 6 of 6,
    the findings in 3 -- a static clearance the snap goes round counts as a finding, and a lane the snap cannot lay
    often had none"""
    tries = int(awx_settings.get('LOOP_PICK_SNAP') or '1')
    f0 = snap_failed(loopd, plan)
    if tries <= 1 or not f0:
        return plan
    try:
        held_i = int(json.load(open(os.path.join(loopd, 'best.json')))['i'])
    except Exception:
        held_i = None
    sc = {}
    for p in glob.glob(os.path.join(loopd, 'p*.json')):
        m = re.search(r'[\\/]p(\d+)\.json$', p)
        a = p[:-len('.json')] + '.audit'
        if not m or int(m.group(1)) == held_i or not os.path.isfile(a):
            continue
        sc[int(m.group(1))] = findings(stripped(run(['whole_gate.py', p, a], env)[1]))
    order = sorted(sc, key=lambda i: (sc[i], -i))[:tries - 1]
    best, bf = plan, f0
    for i in order:
        hp = held_plan(loopd, env, i=i)
        fi = snap_failed(loopd, hp) if hp else None
        say(f"  round {i}'s plan snapped: {fi if fi is not None else 'nothing'} net(s) unlaid (the held round {held_i}: "
            f"{f0})")
        if hp and fi is not None and fi < bf:
            best, bf = hp, fi
        if bf == 0:
            break
    if best is not plan:
        say(f"  held by its snap: {os.path.basename(best)} ({bf} net(s) unlaid against {f0})")
    return best


def lay_rounds(loopd, d, env, res, lay_, say, plan=None):
    """a loop that did not pass chooses its round by the LAY, not by its findings. The held round laid with nets open
    (`res`, lay_'s result), the loop's other rounds' smooth plans, fewest findings first, each snapped as the held one
    is (held_plan) and laid (lay_(plan, tag) -> a result, or None), stopping at the first with no net open -- every
    round the loop ran where it must (lay_tries: LAY_TRY=N, N boards in all, a plan that lays none not counted; 1
    off); the best by the lay's own key (open nets, DRC, vias, copper) kept, its board and logs copied over the held
    one's names in `d`. Measured on the zynq LVDS: the held round laid 10, 3, 2 and 2 open where the loop's best round
    laid 6, 0, 2 and 2 -- the findings count a static clearance the snap goes round and a detour not at all. And one
    other round is not enough: on a bus of ten pairs the lay refused a pair of the held round's plan, the round of
    fewest findings held nine lanes (a pair its snap could not lay) and left two nets open as well, and the third
    round's plan laid all ten clean. (`plan`: the probe LOOP_PICK_SNAP's held plan in the held round's place)"""
    tries = lay_tries()
    if tries == 1 or (res is not None and not res['open']):
        return res
    try:
        held_i = int(json.load(open(os.path.join(loopd, 'best.json')))['i'])
    except Exception:
        held_i = None
    # (the round `res` was laid from: the held round's, or the one LOOP_PICK_SNAP held in its place -- left out of the
    # others, and the held round tried among them)
    m_ = re.search(r'held(\d+)_plan\.json$', plan or '')
    if m_:
        held_i = int(m_.group(1))
    sc = {}
    for p in glob.glob(os.path.join(loopd, 'p*.json')):
        m = re.search(r'[\\/]p(\d+)\.json$', p)
        a = p[:-len('.json')] + '.audit'
        if not m or int(m.group(1)) == held_i or not os.path.isfile(a):
            continue
        sc[int(m.group(1))] = findings(stripped(run(['whole_gate.py', p, a], env)[1]))
    order = sorted(sc, key=lambda i: (sc[i], -i))
    say(f"  the held round {held_i} laid " + (f"open {res['open']}, drc {res['drc']}" if res else 'nothing')
        + " -- the loop's other rounds, fewest findings first, until "
        + (f"{tries - 1} laid: " if tries else "one lays with no net open: ")
        + ', '.join(f'{i} ({sc[i]})' for i in order))
    best, laid = res, 0
    for i in order:
        if tries and laid >= tries - 1:
            break
        hp = held_plan(loopd, env, i=i)
        rr = lay_(hp, f'_h{i}') if hp else None
        say(f"  round {i}'s plan laid: " + (f"open {rr['open']}, drc {rr['drc']}, vias {rr['key'][2]}, "
                                            f"copper {rr['key'][3]} mm" if rr else 'nothing'))
        if rr is None:
            continue
        laid += 1
        if best is None or rr['key'] < best['key']:
            best = rr
        if not rr['open']:
            break
    if best is not None and best is not res:
        tag = best['tag']
        for nm in ('seq.kicad_pcb', 'seq.kicad_pro', 'route_seq.log', 'conn.log', 'drc.log'):
            b_, e_ = os.path.splitext(nm)
            src = os.path.join(d, f'{b_}{tag}{e_}')
            if os.path.isfile(src):
                shutil.copyfile(src, os.path.join(d, nm))
        best = dict(best, board=os.path.join(d, 'seq.kicad_pcb'))
        say(f"  kept round {tag[2:]}'s lay")
    return best


def partial_drops(named, xs, lanes):
    """the last resort's sets of lanes to leave out, each larger than the one before: the lanes the rounds named
    (else the three most crossed), then the most crossed besides to a quarter of the lanes, then to half -- never every
    lane. `xs` {lane: its crossings} (the ends model's), `lanes` the run's"""
    L = len(lanes)
    by_x = [ln for ln in sorted(lanes, key=lambda ln: (-xs.get(ln, 0), ln)) if ln not in named]
    drops = []
    for size in (len(named) or 3, -(-L // 4), -(-L // 2)):
        dr = list(named)[:L - 1] + by_x[:max(0, min(size, L - 1) - len(named))]
        if dr and (not drops or len(dr) > len(drops[-1])):
            drops.append(dr)
    return drops


def add_over(fb_path, over_nets):
    """the lanes an UNPROVED plan leaves over two vias (whole_solve 'over_nets'), merged into a feedback round's findings
    (FEEDBACK=, whole_ends 'over': each counted over by at least that much); the lanes it newly names"""
    if not over_nets:
        return []
    fb = json.load(open(fb_path)) if os.path.isfile(fb_path) else {'pairs': [], 'avoid': []}
    ov = fb.setdefault('over', {})
    new = [n for n, k in sorted(over_nets.items()) if int(k) > int(ov.get(n, 0))]
    for n in new:
        ov[n] = int(over_nets[n])
    json.dump(fb, open(fb_path, 'w'), indent=1)
    return new


def stage_env(env):
    """the settings every stage of the chain runs under, set in `env` (and returned): one thread per numeric library,
    the harness's caches, and the plan environment of the fanout on our own ends (resolve_round.py runs one stage
    under the same)"""
    env.update(OMP_NUM_THREADS='1', VECLIB_MAXIMUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               TAUT_MEMO=env.get('TAUT_MEMO') or '1', PROBE_MEMO=env.get('PROBE_MEMO') or '1')
    env.update(PLAN_PAGES='1', PLAN_JUDGE='ends', BRAID_PAIRS='1', PLAN_PAIRS='1',
               BRAID_EXACT_PAGES='0', PLAN_PAGES_SIDERS='2')
    return env


def no_ends(bench, nets, dest):
    """the nets of `nets` the solve cannot read both ends of on BENCH: a net the fanout's plan sidecar runs (its 'nets')
    and laid no end for (its 'ends'), and one it does not name with no free stub end at each part (braid.endpoints,
    which the solve's setup asserts on). A fanout can choose a lane no escape -- sdi_mipi_conv's SDI_D18 once the DDR3
    bus and its terminations took U11's field, its ball a via in pad and no stub -- and the round's solve, and every
    last resort after it, then died on that one lane. A lane its fanout ran and did not lay stands on the field's
    dog-bone, inside the array, not on a tooth: its neighbours SDI_D13 and SDI_D19 there, on the source's far face,
    which the solve's frame refuses (whole_frame.FarFace) -- for the round's every lane"""
    if PYR not in sys.path:
        sys.path.insert(0, PYR)
    from kicad_parser import parse_kicad_pcb
    import braid as te
    pj = os.path.splitext(bench)[0] + '.plan.json'
    sc = json.load(open(pj)) if os.path.isfile(pj) else {}
    planned, ran = set(sc.get('ends') or {}), set(sc.get('nets') or ())
    bad = {n for n in nets if n in ran and n not in planned}
    rest = [n for n in nets if n not in planned and n not in ran]
    if rest:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(bench)
        from coherent_nets import by_key
        bn = by_key(pcb)
        for n in rest:
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    te.endpoints(pcb, [n], bn, dest_ref=dest)
            except (AssertionError, KeyError):
                bad.add(n)
    return [n for n in nets if n in bad]


def with_ends(bench, nets, dest, lanes):
    """(`nets` less the whole lanes -- `lanes` {lane: its nets}, a pair's legs together -- one of whose nets has no end
    on BENCH (no_ends), the nets with none, the legs left out with them): what a round can solve, the rest open"""
    bad = set(no_ends(bench, nets, dest))
    out = bad | {n for lg in lanes.values() if bad.intersection(lg) for n in lg}
    return [n for n in nets if n not in out], sorted(bad), sorted(out - bad)


def footprint_poses(board):
    """{reference: [x, y, rotation]} of each footprint on `board` whose reference no other carries, the rotation in
    [0, 360) -- a board one tool wrote says -90 where another's says 270, and the two are one pose"""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    seen = collections.Counter(fp.reference for fp in pcb.footprints.values())
    return {fp.reference: [round(fp.x, 4), round(fp.y, 4), round((fp.rotation or 0.0) % 360.0, 4) % 360.0]
            for fp in pcb.footprints.values() if seen[fp.reference] == 1}


def place_held(board, out, held):
    """`board` with the footprints `held` ({reference: [x, y, rotation]}) where it says, written to `out` (its project
    beside it); `board` itself when every one of them already stands there"""
    now = footprint_poses(board)
    moves = [{'reference': r, 'new_x': p[0], 'new_y': p[1], 'new_rotation': p[2]}
             for r, p in sorted(held.items()) if r in now and now[r] != p]
    if not moves:
        return board
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_placer'))
    from placement.writer import write_placed_output
    import fanout_from_plan as fp_
    with contextlib.redirect_stdout(io.StringIO()):
        write_placed_output(board, out, moves)
    fp_.copy_pro(board, out)
    return out


JOINT_WAIT = 4 * 3600          # (JOINT_DIR) seconds a bus waits for the round's other buses at most
LOOP_END = 10 ** 6              # (joint_pos) a loop's position once it has ended: past every re-solve of it


def joint_pos(env, pos):
    """(JOINT_DIR) where this bus is in the buses' exchanges, DIR/pos.BUS.json: (round, 0, 0) a round's first solve,
    (round, loop, i) the loop's re-solve after its round i, (round, loop, LOOP_END) the loop ended -- a peer waiting at
    an exchange this bus is past waits no more"""
    jd, bus = env.get('JOINT_DIR'), env.get('JOINT_BUS')
    if jd and bus:
        tmp = os.path.join(jd, f'.pos.{bus}.json')
        json.dump([int(x) for x in pos], open(tmp, 'w'))
        os.replace(tmp, os.path.join(jd, f'pos.{bus}.json'))


def joint_gone(jd, peer, pos):
    """a peer not waited for at the exchange `pos`: its whole route has ended, or it is past `pos` (joint_pos)"""
    if os.path.isfile(os.path.join(jd, f'{peer}.finished')):
        return True
    try:
        return tuple(json.load(open(os.path.join(jd, f'pos.{peer}.json')))) > tuple(pos)
    except (OSError, ValueError):
        return False


def joint_publish(env, plan, stage_name, board=None, nets=None):
    """(JOINT_DIR: negotiated buses) this bus's lanes as `plan` has them -- or the laid `board`'s copper of `nets`
    -- published for the other buses' stages (joint_lanes)"""
    if not (env.get('JOINT_DIR') and env.get('JOINT_BUS')):
        return
    import joint_lanes
    with awx_settings.given(env):
        if board:
            joint_lanes.publish_board(board, nets, stage_name)
        else:
            joint_lanes.publish(plan, stage_name)


def with_end_runs(lanes, env, q, cx, cy):
    """the lanes a bus asks the coupled solve with, its VIA-END RUNS on the fanout's bench (BENCH0, else BENCH) among
    their pieces (joint_lanes.end_runs): the round's relayer moves each run onto its lane's end layer, so they are the
    lane's ways too -- two buses' runs crossing inside the shared array, each moved onto one layer by its own relayer,
    merged as two crossings no bus had seen. Only in the request: the other buses' stages see that copper on their own
    boards"""
    import joint_lanes
    with awx_settings.given(env):
        runs = joint_lanes.end_runs(env.get('BENCH0') or env.get('BENCH'), int(q), cx, cy)
    if not runs or not lanes:
        return lanes                # (no lanes laid out: the coupled solve reads the taut paths, whole_joint)
    lanes = dict(lanes)
    lanes['pieces'] = list(lanes.get('pieces') or []) + runs
    return lanes


def joint_sync(env, r, solve, say):
    """(JOINT_DIR, JOINT_BUS, JOINT_PEERS, JOINT_TURN: route_bus's buses off one shared array, each its own whole route
    in a process of its own) the round's first plan one COUPLED solve: this bus's own, just solved, asked for
    (DIR/rR/BUS.req.json), and the round's coupled solve (whole_joint.py) run by the last of the round's buses to ask --
    a peer not waited for once its whole route has ended (DIR/PEER.finished) or it has asked in a later round -- its
    plan for this bus written over `solve` (kept as it was where the coupled solve has none)"""
    jd, bus = env['JOINT_DIR'], env['JOINT_BUS']
    peers = [p_ for p_ in (env.get('JOINT_PEERS') or '').split(',') if p_ and p_ != bus]
    rd = os.path.join(jd, f'r{r}')
    os.makedirs(rd, exist_ok=True)
    joint_pos(env, (r, 0, 0))
    if not os.path.isfile(solve) or not os.path.isfile(solve[:-len('.json')] + '.vals.json') or \
            json.load(open(solve)).get('relaxed'):
        open(os.path.join(rd, f'{bus}.none'), 'w').close()
        say(f'  coupled solve: no plan of this bus\'s own model to bring to round {r}\'s coupled solve')
        return None
    q_, cx_, cy_ = (float(v_) for v_ in env['JOINT_TURN'].split(','))
    # this bus's own plan LAID OUT -- its geometry and polish, as the loop's first round lays it -- where its lanes go:
    # the couplings are read off the lanes' real ways (a taut path, drawn on one layer, crossed the array the lanes
    # go round), and published, the other buses' stages planning round them from here
    gd = os.path.join(os.path.dirname(os.path.abspath(solve)), 'joint_geo')
    os.makedirs(gd, exist_ok=True)
    genv = {**env, 'BRAID_PAIRS': '1', 'BRAID_EXACT_PAGES': '0', 'PLAN_PAGES_SIDERS': '2',
            'STAGE_CACHE': env.get('STAGE_CACHE') or '1', 'TAUT_MEMO': env.get('TAUT_MEMO') or '1'}
    g0, p0 = os.path.join(gd, 'g0.json'), os.path.join(gd, 'p0.json')
    lanes = None
    # (on the bench its plan is laid on: its via ends relayed onto the plan's layers -- relayed, as the round does)
    quiet = lambda *a_: None                                     # noqa: E731
    genv0 = dict(genv)
    relayed(os.path.dirname(os.path.abspath(solve)), genv0['BENCH'], json.load(open(solve)), genv0, quiet,
            name='joint_solo_layers')
    if run(['whole_geo.py', solve, g0], genv0, log_path=os.path.join(gd, 'g0.log'))[0] == 0 and \
            run(['whole_polish.py', g0, p0], genv0, log_path=os.path.join(gd, 'p0.log'))[0] == 0:
        joint_publish(genv0, p0, f'round {r} own plan')
        try:
            lanes = json.load(open(os.path.join(jd, 'lanes', f'{bus}.json')))
        except (OSError, ValueError):
            lanes = None
    lanes = with_end_runs(lanes, genv, q_, cx_, cy_)
    say('  coupled solve: this bus\'s own plan laid out for the couplings: '
        + (f'{len(lanes["pieces"])} pieces' if lanes else 'NOT (its taut paths instead)'))
    req = {'bus': bus, 'env': {k_: v_ for k_, v_ in env.items() if isinstance(v_, str)}, 'solve': os.path.abspath(solve),
           'turn': [int(q_), cx_, cy_], 'lanes': lanes}
    tmp = os.path.join(rd, f'.{bus}.req.json')
    json.dump(req, open(tmp, 'w'))
    os.replace(tmp, os.path.join(rd, f'{bus}.req.json'))
    t0 = time.time()

    def here(p_):
        return os.path.isfile(os.path.join(rd, f'{p_}.req.json')) or os.path.isfile(os.path.join(rd, f'{p_}.none'))

    def gone(p_):
        return joint_gone(jd, p_, (r, 0, 0)) or any(
            os.path.isfile(os.path.join(jd, f'r{k_}', f'{p_}.req.json')) or
            os.path.isfile(os.path.join(jd, f'r{k_}', f'{p_}.none')) for k_ in range(r + 1, r + 9))
    while not all(here(p_) or gone(p_) for p_ in peers):
        if time.time() - t0 > JOINT_WAIT:
            say(f'  coupled solve: the other buses not there after {JOINT_WAIT} s -- this bus\'s own plan')
            return None
        time.sleep(5)
    with_ = [p_ for p_ in peers if os.path.isfile(os.path.join(rd, f'{p_}.req.json'))]
    if not with_:
        say(f'  coupled solve: no other bus in round {r} -- this bus\'s own plan')
        return None
    try:
        fd_ = os.open(os.path.join(rd, 'coord.lock'), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        os.close(fd_)
        say(f'  coupled solve: round {r}, with {", ".join(with_)} (this bus runs it)')
        run(['whole_joint.py', jd, str(r)], env, log_path=os.path.join(rd, 'joint.log'), own_process=True)
        for p_ in [bus] + with_:
            if not os.path.isfile(os.path.join(rd, f'{p_}.joint.json')):
                json.dump({'bus': p_, 'joint': False}, open(os.path.join(rd, f'{p_}.joint.json'), 'w'))
    except FileExistsError:
        say(f'  coupled solve: round {r}, with {", ".join(with_)} (another bus runs it)')
    while not os.path.isfile(os.path.join(rd, f'{bus}.joint.json')):
        if time.time() - t0 > JOINT_WAIT:
            say('  coupled solve: no coupled plan in time -- this bus\'s own plan')
            return None
        time.sleep(5)
    for ln in grep(os.path.join(rd, 'joint.log'), r'couplings|side by side|coupled search|NOT the same'):
        say('  coupled solve: ' + ln.strip()[:200])
    jj_ = json.load(open(os.path.join(rd, f'{bus}.joint.json')))
    ok_, cuts_, own_ = jj_.get('joint'), jj_.get('cuts'), bool(jj_.get('own'))
    say('  coupled solve: this bus\'s plan '
        + ('its own, the coupled plan (no coupling broken)' if ok_ and own_ else
           "the coupled solve's" if ok_ else 'its own (the coupled solve gave none)')
        + (f', {len(json.load(open(cuts_))["lcuts"])} coupling(s) held through its loop' if cuts_ else ''))
    # ...the COUPLED plan laid out and published in place of the one this bus solved alone, and every other bus's too
    # before the loops start: a loop's first audit read the others' plans as solved by themselves, and found clashes the
    # coupled solve had settled (its own plan, laid out and published above, where that is the coupled one)
    genv1 = dict(genv)
    if ok_ and not own_:
        relayed(os.path.dirname(os.path.abspath(solve)), genv1['BENCH'], json.load(open(solve)), genv1, quiet,
                name='joint_layers')
    if ok_ and not own_ and run(['whole_geo.py', solve, os.path.join(gd, 'g1.json')], genv1,
                   log_path=os.path.join(gd, 'g1.log'))[0] == 0 and \
            run(['whole_polish.py', os.path.join(gd, 'g1.json'), os.path.join(gd, 'p1.json')], genv1,
                log_path=os.path.join(gd, 'p1.log'))[0] == 0:
        joint_publish(genv1, os.path.join(gd, 'p1.json'), f'round {r} coupled plan')
    open(os.path.join(rd, f'{bus}.laid'), 'w').close()
    while not all(os.path.isfile(os.path.join(rd, f'{p_}.laid')) or gone(p_) for p_ in with_):
        if time.time() - t0 > JOINT_WAIT:
            break
        time.sleep(5)
    return cuts_ if ok_ else None


def joint_skip(env, pos):
    """(JOINT_DIR) this bus past the loop's exchange `pos` without a re-solve to bring: its position there, and its
    none (DIR/r{R.L.I}/BUS.none) -- the other buses there take the coupled solve without it"""
    joint_pos(env, pos)
    rd = os.path.join(env['JOINT_DIR'], 'r' + '.'.join(str(int(x)) for x in pos))
    os.makedirs(rd, exist_ok=True)
    open(os.path.join(rd, f'{env["JOINT_BUS"]}.none'), 'w').close()


def joint_step_sync(env, pos, tag, say):
    """(JOINT_DIR) a stage of every bus's loop IN STEP: this bus's stage `tag` of the loop's round `pos` (round, loop,
    i) done, its lanes published, said (DIR/s{tag}.{R.L.I}/BUS.done), and this bus waits until every other bus has said
    so too or is past that round -- at its exchange i or later, or its loop ended (joint_gone) -- so what comes next
    reads every bus's lanes of the same round"""
    jd, bus = env['JOINT_DIR'], env['JOINT_BUS']
    peers = [p_ for p_ in (env.get('JOINT_PEERS') or '').split(',') if p_ and p_ != bus]
    key = '.'.join(str(int(x)) for x in pos)
    sd = os.path.join(jd, f's{tag}.{key}')
    os.makedirs(sd, exist_ok=True)
    open(os.path.join(sd, f'{bus}.done'), 'w').close()
    before = (int(pos[0]), int(pos[1]), int(pos[2]) - 1)       # (the exchange before this round: a peer past it is on)
    t0 = time.time()
    while not all(os.path.isfile(os.path.join(sd, f'{p_}.done')) or joint_gone(jd, p_, before) for p_ in peers):
        if time.time() - t0 > JOINT_WAIT:
            say(f'  negotiation {key}: the other buses\' {tag} not there after {JOINT_WAIT} s -- on without them')
            return
        time.sleep(5)


def joint_loop_sync(env, pos, solve, say):
    """(JOINT_DIR) a loop's re-solve one COUPLED solve, every bus's loop in step: this bus's own re-solve `solve`, just
    made (its model's every variable beside it, SOLVE.vals.json), asked for at the exchange `pos` (round, loop, i) with
    its lanes as it last published them (joint_lanes: its round i's polish), and the coupled solve (whole_joint.py) run
    by the last of the buses there to ask -- every crossing of two buses' lanes, and every place two of them come within
    a lane's pitch, settled in the one model, so a conflict two buses' audits find is the two buses' to answer, not the
    one that found it. A peer not waited for once it is past `pos` (joint_gone). The coupled plan is written over
    `solve`; (whether it gave this bus a coupled plan, the path of the couplings it settled -- firm layer cuts of this
    bus's, held from here -- or None)"""
    jd, bus = env['JOINT_DIR'], env['JOINT_BUS']
    peers = [p_ for p_ in (env.get('JOINT_PEERS') or '').split(',') if p_ and p_ != bus]
    key = '.'.join(str(int(x)) for x in pos)
    rd = os.path.join(jd, f'r{key}')
    os.makedirs(rd, exist_ok=True)
    joint_pos(env, pos)
    if not os.path.isfile(solve) or not os.path.isfile(solve[:-len('.json')] + '.vals.json') or \
            json.load(open(solve)).get('relaxed'):
        open(os.path.join(rd, f'{bus}.none'), 'w').close()
        return False, None
    try:
        lanes = json.load(open(os.path.join(jd, 'lanes', f'{bus}.json')))
    except (OSError, ValueError):
        lanes = None
    q_, cx_, cy_ = (float(v_) for v_ in env['JOINT_TURN'].split(','))
    lanes = with_end_runs(lanes, env, q_, cx_, cy_)
    req = {'bus': bus, 'env': {k_: v_ for k_, v_ in env.items() if isinstance(v_, str)}, 'solve': os.path.abspath(solve),
           'turn': [int(q_), cx_, cy_], 'lanes': lanes}
    tmp = os.path.join(rd, f'.{bus}.req.json')
    json.dump(req, open(tmp, 'w'))
    os.replace(tmp, os.path.join(rd, f'{bus}.req.json'))
    t0 = time.time()
    here = lambda p_: os.path.isfile(os.path.join(rd, f'{p_}.req.json')) or \
        os.path.isfile(os.path.join(rd, f'{p_}.none'))                                 # noqa: E731
    while not all(here(p_) or joint_gone(jd, p_, pos) for p_ in peers):
        if time.time() - t0 > JOINT_WAIT:
            say(f'  coupled re-solve {key}: the other buses not there after {JOINT_WAIT} s -- this bus\'s own re-solve')
            return False, None
        time.sleep(5)
    with_ = [p_ for p_ in peers if os.path.isfile(os.path.join(rd, f'{p_}.req.json'))]
    if not with_:
        return False, None
    try:
        fd_ = os.open(os.path.join(rd, 'coord.lock'), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        os.close(fd_)
        run(['whole_joint.py', jd, key], env, log_path=os.path.join(rd, 'joint.log'), own_process=True)
        for p_ in [bus] + with_:
            if not os.path.isfile(os.path.join(rd, f'{p_}.joint.json')):
                json.dump({'bus': p_, 'joint': False}, open(os.path.join(rd, f'{p_}.joint.json'), 'w'))
    except FileExistsError:
        pass
    while not os.path.isfile(os.path.join(rd, f'{bus}.joint.json')):
        if time.time() - t0 > JOINT_WAIT:
            say(f'  coupled re-solve {key}: no coupled plan in time -- this bus\'s own re-solve')
            return False, None
        time.sleep(5)
    for ln in grep(os.path.join(rd, 'joint.log'), r'couplings|side by side|coupled search|NOT the same'):
        say(f'  coupled re-solve {key}: ' + ln.strip()[:200])
    jj_ = json.load(open(os.path.join(rd, f'{bus}.joint.json')))
    ok_, cuts_ = jj_.get('joint'), jj_.get('cuts')
    say(f'  coupled re-solve {key}: with {", ".join(with_)}, this bus\'s re-solve '
        + ('its own, the coupled plan (no coupling broken)' if ok_ and jj_.get('own') else
           'the coupled solve\'s' if ok_ else 'its own (the coupled solve gave none)')
        + (f', {len(json.load(open(cuts_))["lcuts"])} coupling(s) held from here' if ok_ and cuts_ else ''))
    return bool(ok_), (cuts_ if ok_ else None)


def round_edits(env):
    """{round: its edits}: this bus's section (ROUND_EDITS_BUS, SRC:DEST) of the ROUND EDITS file (ROUND_EDITS, route_bus
    --round-edits) -- {} without one"""
    f_ = env.get('ROUND_EDITS')
    if not f_:
        return {}
    sec_ = json.load(open(f_)).get(env.get('ROUND_EDITS_BUS') or '') or {}
    return {int(k_): v_ for k_, v_ in sec_.items()}


def edit_frame(env, bench=None, frame='board'):
    """(to_bench, to_plan, plan_layer): the ROUND EDITS' frame -- the BOARD's, as the agent driving the step reads it --
    in the step's: a board point turned into the flow frame (ROUND_EDITS_TURN, route_bus: the bench's), and then, on a
    round of pair chirality -1 (the bench's sidecar), turned over about the bench's mirror axis into the plan's; a board
    layer's name in the plan (F.Cu and B.Cu swapped there, braid.setup). `frame` 'bench': the points are the bench's
    already (a round's renders, frame_render, draw the bench), not turned"""
    import flow_frame as ff
    t_ = ((env.get('ROUND_EDITS_TURN') if frame != 'bench' else None) or '0,0,0').split(',')
    q_, cx_, cy_ = int(float(t_[0])), float(t_[1]), float(t_[2])

    def to_bench(p):
        dx_, dy_ = ff._q(float(p[0]) - cx_, float(p[1]) - cy_, q_)
        return cx_ + dx_, cy_ + dy_
    cy2_, sw_ = None, (lambda L_: L_)
    side_ = os.path.splitext(bench)[0] + '.plan.json' if bench else ''
    if side_ and os.path.isfile(side_) and int(json.load(open(side_)).get('chi', 1)) < 0:
        from bga_fanout.flip_frame import mirror_axis, other_layer
        if PYR not in sys.path:
            sys.path.insert(0, PYR)
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(io.StringIO()):
            cy2_ = 2 * mirror_axis(parse_kicad_pcb(bench))
        sw_ = other_layer

    def to_plan(p):
        b_ = to_bench(p)
        return (b_[0], cy2_ - b_[1]) if cy2_ is not None else b_
    return to_bench, to_plan, sw_


def round_edit_files(ed, d, env, bench):
    """(cut file, flips file, areas file) -- each None where the round asks none -- of a round's ROUND EDITS `ed`, written
    in the round's directory `d` in the plan's frame: each `areas` entry {"box" [x0, y0, x1, y1] or "poly" [[x, y]..],
    "layers" [names], "lanes" [..] (every lane without)} and each `cuts` entry {"lane", "box" or "poly", "layers"} -- a
    cut is an area of one lane: the solve holds it where the lane's reference route passes, the audit wherever the lane
    goes -- turned from the board's frame (edit_frame); `raw_cuts` a cut file as the loop writes them (its frame, route u)
    and `flips` [[lane, island]] as they are"""
    if not ed:
        return None, None, None
    _to_bench, to_plan, plan_layer = edit_frame(env, bench, ed.get('frame') or 'board')

    def area(a_, lanes):
        b_ = a_.get('box')
        pts_ = a_.get('poly') or [[b_[0], b_[1]], [b_[2], b_[1]], [b_[2], b_[3]], [b_[0], b_[3]]]
        return {'poly': [[round(v_, 4) for v_ in to_plan(p_)] for p_ in pts_],
                'layers': [plan_layer(L_) for L_ in a_['layers']], 'lanes': lanes}
    ar_ = [area(a_, a_.get('lanes')) for a_ in ed.get('areas') or ()] + \
          [area(c_, [c_['lane']]) for c_ in ed.get('cuts') or ()]
    out_ = []
    for nm_, obj_ in (('edits_cuts.json', ed.get('raw_cuts')), ('edits_flips.json', {'flips': ed['flips']} if
                                                                ed.get('flips') else None),
                      ('edits_areas.json', ar_ or None)):
        f_ = None
        if obj_:
            f_ = os.path.join(d, nm_)
            json.dump(obj_, open(f_, 'w'), indent=1)
        out_.append(f_)
    return tuple(out_)


def chain(K, o, R=4, base=None, dest=None, settings=None, F=0):
    """whole_chain.sh K OUTDIR ROUNDS: the exit code, the grade the last line printed. The bench and its destination
    are `base` and `dest` when given (route_bus.py), else BASE and DEST from the environment; `settings` go over the
    environment for every stage (route_bus.py's caches off). `R` rounds at most, of them up to `F` feedback rounds
    after each draw (below)"""
    o = os.path.abspath(o)
    os.makedirs(o, exist_ok=True)
    t0 = time.time()
    secs = lambda: int(time.time() - t0)
    env = awx_settings.environ()
    env.update(settings or {})
    if base:                                     # (BASE and DEST reach a stage only when given or set)
        env['BASE'] = base
    if dest:
        env['DEST'] = dest
    base = env.get('BASE') or 'fb_t2q_pairs.kicad_pcb'
    dest = env.get('DEST') or 'DU1'
    stage_env(env)
    if env.get('JOINT_DIR') and env.get('JOINT_BUS'):
        # (negotiated buses off one shared array: every stage reads the other buses' lanes as they publish them --
        # joint_lanes -- but a round's first solve, coupled with theirs, joint_sync)
        env['JOINT_FOREIGN'] = 'all'
    nets_out = run(['coherent_nets.py', str(K), f'--board={base}'], env)[1]
    NETS = (nets_out.splitlines() or [''])[-1]
    # (what the rounds named -- their open lanes, the ends their audits found crowded: whole_feedback -- read by the
    # LAST RESORT alone, when no round laid anything)
    FB = os.path.join(o, 'feedback.json')
    if os.path.exists(FB):
        os.remove(FB)
    # (the passives the round's cap step moved, after its fanout -- a record: every round fans out from `base` and runs
    # the cap step again)
    HELD = os.path.join(o, 'held_parts.json')
    if os.path.exists(HELD):
        os.remove(HELD)
    # (the teeth a round found no escape of their ball's clears -- repair_ends, put_blocked: the one thing a draw is
    # handed of the rounds before it, every round's ENDS_BLOCKED)
    BLOCKED = os.path.join(o, BLOCKED_ENDS)
    if os.path.exists(BLOCKED):
        os.remove(BLOCKED)
    # (an earlier run's results in this OUTDIR: its best board, and every round's laid boards -- route_bus takes the
    # best, else the last round's, and an earlier run's stood in for a run that laid nothing, graded as this one's)
    for f_ in glob.glob(os.path.join(o, 'best.*')) + glob.glob(os.path.join(o, 'ends.*')) \
            + glob.glob(os.path.join(o, 'r[0-9]*', 'seq*.kicad_*')):
        os.remove(f_)
    say = Log()
    r = 0
    # THE ROUNDS ARE DRAWS: the d-th draw searches under the step's seed plus d - 1 (cp_seed: every solve -- the
    # fanout's joint ends and escape plans, the round's solve, the loop's re-solves) from the same board, the cap step
    # again and nothing carried from the round before; the first round laid clean ends them, else the best one laid is
    # kept. A round that left nets open was rarely the start of a better one (rounds fed back what it found cleaned 1 of
    # 9 such rounds and made 4 worse), where another draw of the same board laid clean in 5 of 12 tries.
    # Up to F FEEDBACK ROUNDS (route_bus --feedback-rounds, default none) follow each draw that leaves nets open, under
    # its seed: what the round before found -- the ends its audits found crowded, the lanes its solve could not keep to
    # two vias, its open nets (whole_feedback) -- goes back to the fanout (FEEDBACK=, its own file), which chooses again
    # INCREMENTALLY from that round's board, the passives where the draw's cap step left them. A feedback round that has
    # nothing new to say is the next draw instead
    seed0 = int(env.get('BUS_SEED') or cp_seed.DEFAULT)
    base0 = base
    FBD = os.path.join(o, 'findings.json')
    if os.path.exists(FBD):
        os.remove(FBD)
    draw, fed_left, prev = 0, 0, ''
    # the ROUND EDITS: what the agent driving the step asked of its rounds after looking at the round before -- per
    # round, `cuts` and `areas` (lanes kept off layers in a box or polygon: round_edit_files), `raw_cuts` (a cut file)
    # and `flips` (side flips) the round's solve and loop start from, and `ban` / `pin` / `avoid` / `pairs` (ends by
    # their lane and end, 0 the tooth and 1 the berth: whole_feedback --edits; a ban or a pin binds, an avoid is priced)
    # that make it a feedback round, its fanout choosing again from the round before's board. In a file, so a replay
    # runs the same rounds with no one looking
    EDITS = round_edits(env)
    # the best result of the rounds so far, by its open nets, then its vias (and a result with a violation after every
    # clean one): the chain never ends with nothing when a round had a plan to lay
    best = None
    nets_all = [n for n in NETS.split(',') if n]
    # (the board's nets by the whole route's names, coherent_nets.net_keys: each round's grade selects and reads its
    # nets by them)
    if PYR not in sys.path:
        sys.path.insert(0, PYR)
    from kicad_parser import parse_kicad_pcb
    from coherent_nets import net_keys
    with contextlib.redirect_stdout(io.StringIO()):
        keys_ = net_keys(parse_kicad_pcb(base if os.path.isabs(base) else os.path.join(HERE, base)))

    def advance(d, base):
        """a feedback round's base: the round before's realized source board (its fo.log names it), else `base` as it
        was -- the passives where the draw's cap step left them (HELD); the source board is written before it moves
        them, and a base without them had every round move them again, from where they stood, not always the same"""
        was = base
        sb = source_board_named(os.path.join(d, 'fo.log'))
        if sb and os.path.isfile(os.path.join(d, sb)):
            base = os.path.join(d, sb)
        if os.path.isfile(HELD):
            base = place_held(base, os.path.join(d, 'base_held.kicad_pcb'), json.load(open(HELD)))
        if base != was and 'BASE' in env:
            env['BASE'] = base
        return base

    def ends_only(r, bench, planned):
        """ENDS_ONLY: the round stops at its ENDS -- the fanout's escapes on the layers its first plan put them on (the
        relayed bench; the fanout's own where the solve had no plan) -- handed on as OUTDIR/ends.kicad_pcb, and nothing
        is cut, polished, snapped or laid. Where a bus leaves its array is settled there: over 25 recorded runs the
        final routed board had 2 of 979 teeth at another place than the fanout's and 57 on another layer than the first
        plan's, 420 on another than the fanout's alone. (the chain's status, its grade line)"""
        for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
            src_ = bench[:-len('.kicad_pcb')] + ext
            if os.path.isfile(src_):
                shutil.copyfile(src_, os.path.join(o, 'ends' + ext))
        g = (f"ENDS K={K} round={r} secs={secs()} -- the ends only: the fanout's escapes, "
             + ("on the layers of the first plan" if planned else "the solve with no plan for them"))
        say(g)
        return 0, g

    def lay(r, d, plan, bench, nets=None, tag='', quiet=False):
        """route_lanes all at once on PLAN, graded: the result (its key, open nets, violation, grade line, board), or
        None when no board was written. `nets` the nets the plan lays (a partial plan's, the last resort), else the
        run's; open nets are counted over the run's whatever it lays. `tag` in its files' names (seq{tag}.kicad_pcb,
        ...), `quiet` its grade line not said (lay_rounds: one of several lays, the kept one's said)"""
        rr = ['--plan', plan, '--board', bench, '--nets', ','.join(nets) if nets else NETS, '--dest', dest]
        seq = os.path.join(d, f'seq{tag}.kicad_pcb')
        run(['route_lanes.py', 'all'] + rr + ['--mode', 'seq', '--write', seq], {**env, 'BRAID_PAIR_SLACKS': '0'},
            log_path=os.path.join(d, f'route_seq{tag}.log'))
        sm = '\n'.join(grep(os.path.join(d, f'route_seq{tag}.log'), r'^SUMMARY'))
        say(f"  all lanes laid: {sm}")
        if not os.path.isfile(seq):
            return None
        if not quiet:
            joint_publish(env, None, f'round {r} laid{tag}', board=seq, nets=nets_all)
        from coherent_nets import net_patterns
        pats = net_patterns(nets_all, keys_)
        # (the checkers by the path the shell gives them, from awx/: their logs echo it; always in a process of
        # their own: they are the harness's grade, not the route, and a checker run as a script installs its
        # command-line banner -- the CMD / EXIT echo -- for its whole process)
        checker = lambda name: os.path.join('..', 'py_router', name)
        cc = run([checker('check_connected.py'), seq, '--nets'] + pats, env,
                 log_path=os.path.join(d, f'conn{tag}.log'), own_process=True)[0]
        dj = os.path.join(d, f'drc{tag}.json')
        if os.path.exists(dj):
            os.remove(dj)
        dc = run([checker('check_drc.py'), seq, '--nets'] + pats + ['--clearance-margin', '0.1', '--json', dj], env,
                 log_path=os.path.join(d, f'drc{tag}.log'), own_process=True)[0]
        dv = drc_count(dj)[0]
        # (the round's violations, 0 clean -- a round's lanes, laid untuned, are not judged by their lengths:
        # LENGTH_ROWS; a checker that counted none, its exit: one at least)
        ndrc = dv if dv is not None else int(dc != 0)
        nopen = 0 if cc == 0 else max(1, count_open(os.path.join(d, f'conn{tag}.log')))
        lanes = '\n'.join(m.split(' ')[0] for m in re.findall(r'[0-9]+/[0-9]+ in band', sm))
        v, c = grade_counts(seq, set(nets_all))
        g = (f"WHOLE K={K} round={r} lanes={lanes} vias={v} copper={c}mm connected={int(cc == 0)} "
             f"drc={ndrc} secs={secs()} open={nopen}")
        if not quiet:
            say(g)
        # (ranked by the nets left OPEN first: a violation is the step's own grade's to answer after the whole route,
        # an open net no later stage of the step lays)
        return dict(key=(nopen, ndrc, int(v), int(c)), open=nopen, drc=ndrc, grade=g, board=seq,
                    open_nets=sorted(open_nets(os.path.join(d, f'conn{tag}.log'), keys_)) if nopen else [],
                    tag=tag)

    def lane_legs():
        """{lane: its nets} as the ends model makes lanes: a pair one lane named by its base (pairs.pair_names, under
        PLAN_PAIRS), every other net its own"""
        import pairs as _pairs
        prs = (_pairs.pair_names(nets_all) if int(env.get('PLAN_PAIRS', env.get('BRAID_PAIRS', '0')) or 0) else {})
        legs = {l_ for pr in prs.values() for l_ in pr}
        return {**{n: (n,) for n in nets_all if n not in legs}, **{b: tuple(pr) for b, pr in prs.items()}}

    def solvable(bench, nets):
        """`nets` less the lanes with no end on BENCH (with_ends), those named: the round's solve and lay take the
        rest, and the lanes left out count open"""
        keep, bad, mates = with_ends(bench, nets, dest, lane_legs())
        if bad:
            say(f"  no end on {os.path.basename(bench)} for {', '.join(bad)} -- left out of the solve, open"
                + (f" (with {', '.join(mates)})" if mates else ''))
        return keep

    def last_resort(r):
        """THE LAST RESORT, when no round laid anything: a PARTIAL plan on the last ends a fanout laid -- the solve,
        its loop and the route all at once on the lanes LESS those the rounds named (whole_feedback --name: open, else
        the ends model's over two, loaded, most crossed), then less the most crossed besides to a quarter of the lanes, then to
        half; the lanes left out stay open (route_bus hands them on as they came). The first that lays anything is the
        result: a run's open nets are never all of them while a plan for some can be laid. A board the fanout's audit
        refused will do, the lanes it named left out with them (its split pairs, its teeth on the far face). None when
        none does (or no round laid a fanout board at all)"""
        bench, refused_ = None, []
        for k in range(r, 0, -1):
            d_ = os.path.join(o, f'r{k}')
            if not os.path.isfile(os.path.join(d_, 'fo.plan.json')):
                continue
            if os.path.isfile(os.path.join(d_, 'fo.kicad_pcb')):
                bench = os.path.join(d_, 'fo.kicad_pcb')
            elif os.path.isfile(os.path.join(d_, 'fo.refused.kicad_pcb')) and \
                    os.path.isfile(os.path.join(d_, 'fo.refused.json')):
                bench = os.path.join(d_, 'fo.refused.kicad_pcb')
                rj = json.load(open(os.path.join(d_, 'fo.refused.json')))
                refused_ = [x[0] for x in rj.get('splits', [])] + list(rj.get('far', []))
            if bench:
                d0 = d_
                break
        if not bench:
            return None
        legs = lane_legs()
        xs = (json.load(open(os.path.join(d0, 'fo.plan.json'))).get('ends_model') or {}).get('x') or {}
        named = [ln for ln in ((json.load(open(FB)).get('named') if os.path.isfile(FB) else None) or []) if ln in legs]
        named = list(dict.fromkeys([ln for ln in refused_ if ln in legs] + named))
        drops = partial_drops(named, xs, list(legs))
        for j, dr in enumerate(drops, 1):
            dj = os.path.join(o, f'partial{j}')
            os.makedirs(dj, exist_ok=True)
            keep = {n for ln, lg in legs.items() if ln not in dr for n in lg}
            sub = solvable(bench, [n for n in nets_all if n in keep])
            if not sub:
                continue
            say(f"=== the last resort {j}: a partial plan on {os.path.relpath(bench, o)}, {len(dr)} of {len(legs)} "
                f"lanes left out: {', '.join(dr)}")
            env_ = {**env, 'BENCH': bench, 'NETS': ','.join(sub), 'DEST': dest}
            solve = os.path.join(dj, 'solve.json')
            run(['whole_solve.py', solve], {**env_, 'SOLVE_UNPROVED': '1'}, log_path=os.path.join(dj, 'solve.log'))
            for ln in grep(os.path.join(dj, 'solve.log'), r'whole_solve:|workers:'):
                say(ln[:200])
            if not os.path.isfile(solve):
                say("  no plan from the solve")
                continue
            # (its via ends onto the solve's layers, as a round's: laid on the fanout's own bench, the plan met stubs
            # still on the layers it had moved them off)
            bench_l = relayed(dj, bench, json.load(open(solve)), env_, say)
            loopd = os.path.join(dj, 'loop')
            with open(os.path.join(dj, 'loop.log'), 'w') as lf:
                rc_ = loop(solve, loopd, loop_rounds(), env_, Log(lf))
            say(f"  loop exit {rc_} at {secs()} s")
            plan = os.path.join(loopd, 'plan.json') if rc_ == 0 else held_plan(loopd, env_)
            res = lay(f'partial{j}', dj, plan, plan_bench(loopd, plan, bench_l), sub) if plan else None
            if res is not None:
                return res
            say("  nothing laid")
        return None

    for r in range(1, R + 1):
        d = os.path.join(o, f'r{r}')
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'nets.lines'), 'w') as f:
            f.write(NETS.replace(',', '\n') + '\n')
        ed_ = EDITS.get(r) or {}
        # (`fed`: the round this one follows with its findings -- a feedback round -- else '' and a draw)
        fed, prev = prev, ''
        if fed:
            say(f"=== fanout round {r}: round {r - 1}'s findings, under the same seed")
        else:
            draw += 1
            fed_left = F
            base = base0
            if 'BASE' in env:
                env['BASE'] = base0
            if os.path.exists(FBD):
                os.remove(FBD)
            if draw > 1:
                env['BUS_SEED'] = str(seed0 + draw - 1)
            say(f"=== fanout round {r}" + (f": round 1 again under seed {seed0 + draw - 1}" if r > 1 else ''))
        # (an earlier run's fanout of this round: a fanout that fails leaves no board, and one left from before stood in)
        for f_ in glob.glob(os.path.join(d, 'fo.*')) + glob.glob(os.path.join(d, 'fo_*')):
            os.remove(f_)
        fenv = dict(env)
        # (the run's nets, every round: a fanout never re-picks them off a later round's board -- K51's second round,
        # re-picked, dropped its three pairs while their teeth stood on the board, and the solve found them split)
        fenv['FANOUT_NETS'] = NETS
        if os.path.isfile(BLOCKED):
            fenv['ENDS_BLOCKED'] = BLOCKED
        if fed:
            if os.path.isfile(HELD):
                fenv['FANOUT_PASSIVES_FIXED'] = '1'         # (joint_escape.passives_fixed: the cap step has moved them)
            if os.path.isfile(FBD):
                fenv['FEEDBACK'] = FBD
            fenv['INCREMENTAL'] = os.path.join(fed, 'fo.plan.json')
        rc = run(['fanout_from_plan.py', os.path.join(d, 'fo.kicad_pcb'), str(K), f'--board={base}'], fenv,
                 log_path=os.path.join(d, 'fo.log'))[0]
        say(f"  fanout exit {rc} at {secs()} s")
        for ln in grep(os.path.join(d, 'fo.log'), r'plan model'):
            say(ln[:220])
        if not os.path.isfile(os.path.join(d, 'fo.kicad_pcb')):
            # a board the FANOUT AUDIT refused (fanout_from_plan: a pair split by other lanes' ends) is a draw that
            # failed: what it named recorded (the last resort leaves those lanes out), and the next seed's round -- or,
            # a feedback round left, that round: the split pairs and the lanes between their tips, ends not to be
            # chosen together again, freed incrementally, while the refusal names something new
            # (the findings read off the SET-ASIDE sidecar, fo.refused.plan.json: whole_feedback reads the board beside
            # its sidecar -- the plan frame's mirror axis, the nets' names -- and fo.kicad_pcb is no longer there)
            refused = os.path.join(d, 'fo.refused.json')
            side_r = os.path.join(d, 'fo.refused.plan.json')
            if os.path.isfile(refused) and os.path.isfile(side_r):
                for ln in grep(os.path.join(d, 'fo.log'), r'^REFUSED'):
                    say('  ' + ln[:220])
                say(stripped(run(['whole_feedback.py', '--refused', side_r, FB, refused],
                                 {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1]))
                if fed_left and r < R:
                    fbl = stripped(run(['whole_feedback.py', '--refused', side_r, FBD, refused],
                                       {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1])
                    if re.search(r'whole_feedback: [1-9]', fbl):
                        say("=== the fanout refused its board -- a feedback round frees the ends it named")
                        base = advance(d, base)
                        prev, fed_left = d, fed_left - 1
                        continue
                    say("  the refusal names nothing new for a feedback round")
                say("=== the fanout refused its board -- the next seed's round")
                continue
            say("  no fanout board -- the rounds end here")
            break

        # a feedback round's fanout laying the round before's ends again (findings it priced but did not follow): the
        # rest of the round would be the same -- so the findings named in that round are raised (whole_feedback --raise:
        # their ends' prices doubled; the second time, the other ends of the lanes named too) and the fanout ALONE runs
        # again, up to FANOUT_RAISES times; one that still lays the same ends is the next draw
        def same_ends():
            try:
                return json.load(open(os.path.join(d, 'fo.plan.json'))) == \
                    json.load(open(os.path.join(fed, 'fo.plan.json')))
            except Exception:
                return False
        if fed and same_ends():
            say(f"=== fanout round {r} laid round {r - 1}'s ends again")
            for k_ in range(1, FANOUT_RAISES + 1):
                fbr = stripped(run(['whole_feedback.py', '--raise', os.path.join(fed, 'fo.plan.json'), FBD]
                                   + (['--widen'] if k_ == 2 else []), {**env, 'FB_ROUND': str(r - 1)},
                                   err=sys.stderr)[1])
                say('  ' + fbr)
                if not re.search(r'whole_feedback: [1-9]', fbr):
                    break
                os.replace(os.path.join(d, 'fo.log'), os.path.join(d, f'fo.same{k_}.log'))
                rc = run(['fanout_from_plan.py', os.path.join(d, 'fo.kicad_pcb'), str(K), f'--board={base}'], fenv,
                         log_path=os.path.join(d, 'fo.log'))[0]
                say(f"  fanout again, the findings raised: exit {rc} at {secs()} s")
                for ln in grep(os.path.join(d, 'fo.log'), r'plan model'):
                    say(ln[:220])
                if not os.path.isfile(os.path.join(d, 'fo.kicad_pcb')) or not same_ends():
                    break
            if not os.path.isfile(os.path.join(d, 'fo.kicad_pcb')):
                say("  no fanout board -- the next seed's round")
                continue
            if same_ends():
                say(f"=== fanout round {r} laid round {r - 1}'s ends again, the findings raised -- the next seed's "
                    f"round")
                continue
        bench = os.path.join(d, 'fo.kicad_pcb')
        if env.get('FANOUT_JOINT') and fed and os.path.isfile(HELD):
            say('  caps: held where the draw\'s cap step left them -- the fanout went round them')
        elif env.get('FANOUT_JOINT'):
            # the parts the whole-array fanout lays through -- the movable passives under the arrays, as the chain's own
            # fanout does -- move off its copper now, ONCE, as the chain's cap nudge moves them after its fanout
            # (place_fanout_clearance), and the route holds them fixed where they are left: routed round them where
            # they stood, the zynq's bus on four layers met them under its lanes and its loop's layer cuts left no plan.
            # Where it leaves them is HELD: every later round starts from it and fans out round them
            import rules as _rules
            before = footprint_poses(bench)
            # (--beneath-only: a passive is nudged only where it stays beneath its BGA; one beside the array, in the
            # channel the bus leaves across, stays where it is and the fanout and the route go round it) (and only those
            # beneath the run's own arrays -- the whole-array fanout's: another array's passives are another step's,
            # locked -- joint_escape.passives_beyond. Unscoped, the zynq's DDR step moved two parts beneath the LVDS
            # bus's U5)
            import joint_escape as je_
            lock_ = je_.passives_beyond(bench, [a_['ref'] for a_ in json.load(open(env['FANOUT_JOINT']))['arrays']],
                                        [n_ for n_ in NETS.split(',') if n_])
            rcn = run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), bench, bench,
                       '--clearance', str(_rules.active().clearance), '--beneath-only']
                      + (['--lock'] + lock_ if lock_ else []), env,
                      log_path=os.path.join(d, 'caps.log'), own_process=True)[0]
            if lock_:
                say(f'  caps: beneath another array, left where they are: {", ".join(lock_)}')
            for ln in grep(os.path.join(d, 'caps.log'), r'^(Moved|Stuck|  Unresolved)'):
                say('  caps: ' + ln.strip()[:200])
            if rcn != 0:
                say(f"  caps: the nudge exited {rcn} -- the round goes on with them where the fanout left them")
            after = footprint_poses(bench)
            json.dump({r_: p_ for r_, p_ in sorted(after.items()) if before.get(r_) != p_}, open(HELD, 'w'),
                      indent=1)
            # and the fanout's own copper held to them where they now stand, as every later round holds it: a part
            # with nowhere beneath its BGA to go is left on it, and what no longer stands clear is planned again
            # round the parts (fanout_from_plan --hold)
            run(['fanout_from_plan.py', bench, '--hold'], {**env, 'FANOUT_PASSIVES_FIXED': '1'},
                log_path=os.path.join(d, 'hold.log'))
            for ln in grep(os.path.join(d, 'hold.log'), r'whole-array fanout of'):
                say(ln[:220])
        # (the round's own edits, written in the board's frame and read in the round's: raw cuts for its first solve
        # and its loop, flips for the loop's geometry, areas -- its cuts among them -- for the solve and every audit)
        ecuts, eflips, eareas = round_edit_files(ed_, d, env, bench)
        if eareas:
            env['BUS_AREAS'] = eareas
        else:
            env.pop('BUS_AREAS', None)
        if ecuts or eflips or eareas:
            say(f"  round {r}'s edits: " + ', '.join(f'{len(v_)} {k_}' for k_, v_ in sorted(ed_.items())
                                                     if k_ in ('cuts', 'areas', 'flips') and v_)
                + (', raw cuts' if ecuts else ''))
        sub = solvable(bench, nets_all)
        env.update(BENCH=bench, NETS=','.join(sub), DEST=dest)
        solve = os.path.join(d, 'solve.json')
        # (a round never ends with nothing: its solve keeps a plan it cannot prove -- whole_solve SOLVE_UNPROVED -- and
        # the round lays it; the loop's own re-solves still take only a proved one)
        # (and one with NO plan names, in the round's log, the lanes whose crossings the ends leave no room for --
        # whole_solve SOLVE_CROWD, its CROWD diagnosis)
        # (...and none of an earlier run's in the round's directory: a run into an output routed before reuses its run
        # directory, and a solve with no plan writes no plan -- the plan left there was read as this solve's, on another
        # bench)
        crowded_f = os.path.join(d, 'solve.crowded.json')
        for f_ in (solve, crowded_f):
            if os.path.exists(f_):
                os.remove(f_)
        jfin = {}
        if env.get('JOINT_DIR'):
            # (the buses ended now: this solve's and the coupled solve's model both read their lanes, and no other's)
            jfin = {'JOINT_FOREIGN': 'finished', 'JOINT_FINISHED': ','.join(sorted(
                f_[:-len('.finished')] for f_ in os.listdir(env['JOINT_DIR']) if f_.endswith('.finished')))}
        # (the round's edited cuts held by its first solve too, each breakable at a price, as the loop's re-solves hold
        # theirs: CUTS_FIRM)
        ecut_env = {'CUTS': ecuts, 'CUTS_FIRM': '1'} if ecuts else {}
        run(['whole_solve.py', solve], {**env, 'SOLVE_UNPROVED': '1', 'SOLVE_CROWD': '1', **jfin, **ecut_env},
            log_path=os.path.join(d, 'solve.log'))
        for ln in grep(os.path.join(d, 'solve.log'), r'whole_solve:|workers:|crowded:|rules core:'):
            say(ln[:200])
        jcuts = None
        if env.get('JOINT_DIR') and env.get('JOINT_BUS'):
            jcuts = joint_sync({**env, 'SOLVE_UNPROVED': '1', 'SOLVE_CROWD': '1', **jfin}, r, solve, say)
        # (the coupled solve's couplings held through the loop: joint_sync -- the env read at each loop's call, after
        # the relayer has put the relayed bench in it)
        lenv = lambda: {**env, **({'SEED_CUTS': ','.join(f_ for f_ in (jcuts, ecuts) if f_)} if jcuts or ecuts else {}),
                        **({'SEED_FLIPS': eflips} if eflips else {}),
                        **({'JOINT_ROUND': str(r), 'JOINT_LOOP': '1'} if env.get('JOINT_DIR') else {})}
        crowded = json.load(open(crowded_f)).get('crowded') or [] if os.path.isfile(crowded_f) else []
        J, proved, rc, res, loopd = {}, False, None, None, os.path.join(d, 'loop')
        no_plan = not os.path.isfile(solve)
        # ...NO PLAN, its rules core naming teeth whose fixed layer cannot hold: those teeth escaped again off their
        # layer (repair_layers), and the solve asked again on the repaired bench
        rep_ = repair_layers(d, crowded_f, env, sub, say) if no_plan and os.path.isfile(crowded_f) else None
        if rep_ is not None:
            bench = rep_
            env['BENCH'] = bench
            env.pop('BENCH0', None)
            os.replace(crowded_f, crowded_f[:-len('.json')] + '0.json')
            run(['whole_solve.py', solve], {**env, 'SOLVE_UNPROVED': '1', 'SOLVE_CROWD': '1', **jfin, **ecut_env},
                log_path=os.path.join(d, 'solve_layers.log'))
            for ln in grep(os.path.join(d, 'solve_layers.log'), r'whole_solve:|workers:|crowded:|rules core:'):
                say(ln[:200])
            if env.get('JOINT_DIR') and env.get('JOINT_BUS'):
                jcuts = joint_sync({**env, 'SOLVE_UNPROVED': '1', 'SOLVE_CROWD': '1', **jfin}, r, solve, say)
            # (a re-solve with no diagnosis of its own -- its setup refused the repaired bench -- keeps the first's)
            crowded = (json.load(open(crowded_f)).get('crowded') or [] if os.path.isfile(crowded_f) else []) or crowded
            no_plan = not os.path.isfile(solve)
        if no_plan and env.get('ENDS_ONLY'):
            return ends_only(r, bench, False)
        if no_plan:
            say("  no plan from the solve -- nothing for this round to lay"
                + (f" (no room for the crossings of {', '.join(crowded)})" if crowded else ''))
            if env.get('JOINT_DIR'):
                joint_pos(env, (r, LOOP_END, LOOP_END))
        else:
            J = json.load(open(solve))
            proved = J.get('proved', True)
            vb_rows = lambda: (json.load(open(env['VBAN_WALLS'])) if env.get('VBAN_WALLS')
                               and os.path.isfile(env['VBAN_WALLS']) else [])
            vb0_ = vb_rows()
            bench = relayed(d, bench, J, env, say)
            # ...a relay that WALLED A BALL kept the runs near it on the layers they were laid on and banned the plan's
            # layers for them (relay_ends, VBAN_WALLS): the plan then asks those ends on layers the board does not give
            # them, each such lane's band out at its end and laid open -- the bans read only by the loop's re-solves,
            # and a round of one loop round has none. The round's solve again at once, with them, on the fanout's bench
            # (a berth run moved onto In2.Cu walled a ball of the destination; six lanes BERTH-OUT, three laid open)
            for _rs in range(2):
                vb1_ = vb_rows()
                if len(vb1_) <= len(vb0_):
                    break
                say(f"  {len(vb1_) - len(vb0_)} end layer(s) the relay walled a ball on -- the solve again without them")
                vbp_ = vb0_
                vb0_ = vb1_
                shutil.copy2(solve, solve[:-len('.json')] + f'_walls{_rs}.json')
                os.remove(solve)
                # (as the round's first solve: its edited cuts held, and a negotiated bus's coupled again -- solved
                # alone, the walls' plan replaced the coupled one and dropped the round's edits)
                run(['whole_solve.py', solve], {**env, 'BENCH': env['BENCH0'], 'SOLVE_UNPROVED': '1', 'SOLVE_CROWD': '1',
                                                **jfin, **ecut_env}, log_path=os.path.join(d, f'solve_walls{_rs}.log'))
                if os.path.isfile(solve) and env.get('JOINT_DIR') and env.get('JOINT_BUS'):
                    jcuts = joint_sync({**env, 'BENCH': env['BENCH0'], 'SOLVE_UNPROVED': '1', 'SOLVE_CROWD': '1',
                                        **jfin}, r, solve, say)
                for ln in grep(os.path.join(d, f'solve_walls{_rs}.log'), r'whole_solve:|crowded:'):
                    say(ln[:200])
                if not os.path.isfile(solve):
                    # ...NO PLAN without them: the plan before stands, and stands WHOLE -- its bans the ones it was
                    # solved under, every end of it laid on its layer, the ball taken as walled (WALLS_STAND: one ball
                    # of another net named at the step's end, against a lane open for each end kept). Left with the
                    # ends kept where they were laid, the plan asked six teeth on layers the bench did not give them --
                    # six bands broken at the tooth, ten nets open -- and the loop's re-solves, reading the bans no
                    # plan stood under, had none either
                    shutil.copy2(solve[:-len('.json')] + f'_walls{_rs}.json', solve)
                    json.dump(vbp_, open(env['VBAN_WALLS'], 'w'))
                    env['WALLS_STAND'] = ','.join(sorted({k_ for k_ in (env.get('WALLS_STAND', '') + ','
                                                                         + env.get('WALLS_LAST', '')).split(',') if k_}))
                    say(f"  no plan without them -- the plan before stands, laid whole: {env['WALLS_STAND']} walled in")
                    bench = relayed(d, env['BENCH0'], J, env, say)
                    break
                J = json.load(open(solve))
                proved = J.get('proved', True)
                bench = relayed(d, env['BENCH0'], J, env, say)
            if env.get('ENDS_ONLY'):
                return ends_only(r, bench, True)
            if not proved:
                u = J.get('unproved') or {}
                say(f"  the solve's plan is UNPROVED ({u.get('over')} over two vias against a bound of "
                    f"{u.get('bound_over')}): laid all the same; its nets over two to the ends: {J.get('over_nets')}")
            with open(os.path.join(d, 'loop.log'), 'w') as lf:
                rc = loop(solve, loopd, loop_rounds(), lenv(), Log(lf))
            for ln in grep(os.path.join(d, 'loop.log'), r'^=== round|smooth:|NOT CONVERGING|passes'):
                say(ln[:170])
            say(f"  loop exit {rc} at {secs()} s")
            # ...stopped at crowded ends: the fanout bench repaired where they stand (repair_ends) and the loop run again
            # on it, from where the stopped loop got (loop_seeds: its best round's solve, every cut it held) -- the
            # round's lay, and the next round's feedback, from the new loop
            rep_ = repair_ends(d, loopd, env, sub, say) if rc == 4 else None
            if rep_ is None and rc in (3, 4):
                # (...and a loop stopped on a part's pad in its lanes' way, at the ends or not: the part moved a short
                # way off them, and the loop run again on it as on repaired ends)
                rep_ = repair_passives(d, loopd, env, say)
            if rep_ is not None:
                solve_, seeds_ = loop_seeds(loopd)
                b0_, loopd = rep_
                env.pop('BENCH0', None)
                env['BENCH'] = b0_
                bench = relayed(d, b0_, json.load(open(solve_)), env, say, name='fo_layers_ends')
                say(f"  the loop again on the repaired ends from {os.path.basename(solve_)}, its "
                    f"{len([c_ for c_ in seeds_['SEED_CUTS'].split(',') if c_])} cut file(s) held")
                with open(os.path.join(d, 'loop_ends.log'), 'w') as lf:
                    rc = loop(solve_, loopd, loop_rounds(),
                              {**lenv(), **seeds_, **({'JOINT_LOOP': '2'} if env.get('JOINT_DIR') else {})}, Log(lf))
                for ln in grep(os.path.join(d, 'loop_ends.log'), r'^=== round|smooth:|NOT CONVERGING|passes'):
                    say(ln[:170])
                say(f"  the loop again on the repaired ends: exit {rc} at {secs()} s")
            # (the round's loops ended: past every loop exchange of the round, for a bus still in one -- joint_pos)
            if env.get('JOINT_DIR'):
                joint_pos(env, (r, LOOP_END, LOOP_END))
            # a loop that did not pass is laid all the same, from the plan it held (held_plan): a lane it cannot lay
            # stays open, and the round's result stands against the others'
            plan = os.path.join(loopd, 'plan.json') if rc == 0 else held_plan(loopd, env)
            if rc != 0 and plan is not None:
                plan = pick_by_snap(loopd, env, plan, say)
            if rc != 0:
                say(f"  the loop did not pass -- laid from the plan it held: "
                    f"{os.path.relpath(plan, d) if plan else 'none'}")
            try_ = rc != 0 and plan is not None and lay_tries() != 1
            res = lay(r, d, plan, plan_bench(loopd, plan, bench), sub, quiet=try_) if plan else None
            if try_:
                res = lay_rounds(loopd, d, env, res, lambda hp, tag: lay(r, d, hp, plan_bench(loopd, hp, bench), sub,
                                                                          tag=tag, quiet=True), say, plan=plan)
                if res is not None:
                    say(res['grade'])
                    # (the lay KEPT published, as an unquiet lay publishes itself: every lay here was quiet, and the
                    # other buses went on reading this one's last plan, not its copper)
                    joint_publish(env, None, f'round {r} laid', board=res['board'], nets=nets_all)
            if plan and res is None:
                say("  no board laid from the plan (route_seq.log)")
        del env['BENCH']
        env.pop('VIA_ENDS', None)
        env.pop('BENCH0', None)
        env.pop('VBAN_WALLS', None)          # (a draw's own ends: its bans none of the next's)
        env.pop('WALLS_STAND', None)
        env.pop('WALLS_LAST', None)
        env.pop('BUS_AREAS', None)           # (...its edit areas, in its frame, none of the next round's fanout's)
        if res is not None and (best is None or res['key'] < best['key']):
            best = res
        # done: a plan laid with NO NET OPEN -- whether or not its loop passed, proved or not, and whatever its
        # violations, which the step's own grade answers after the whole route: another round is for nets left open
        # (a bus's solo run laid 0 open in each of four rounds, ~600 s each, every one with the same 0.030 mm graze of a
        # cap pad, and the board the step shipped clean). A loop that did not pass names ends its audits found crowded,
        # and the lay went round them; a later round changes those ends and can better only the vias or the copper, and
        # on the synth handoff bench none did (11 of 117 cases went on after such a round, each one to the same grade),
        # while a DDR bus's whole-array fanout, its second round laid clean, spent 360 s on a third. An UNPROVED plan
        # laid is done too: a DDR3 bus went on after one in six runs, each second round 1000-1600 s for 2 to 11 fewer
        # vias, the last for 12 more
        if res is not None and res['open'] == 0:
            break
        # (what the round left open -- or, laying nothing, the lanes its solve's CROWD diagnosis found no room for --
        # recorded for the LAST RESORT, which leaves them out when no round lays anything)
        if res is None or res['open']:
            say(stripped(run(['whole_feedback.py', '--name', os.path.join(d, 'fo.plan.json'), FB]
                             + (res['open_nets'] if res else crowded),
                             {**env, 'FB_ROUND': str(r), 'FB_CONN': os.path.join(d, 'conn.log')}, err=sys.stderr)[1]))
        if (res is None or res['open']) and env.get('GROUP_ENDS') and rc is not None:
            # (THE GROUP'S ENDS are not this step's to change -- its fanout stage lays the group's model's own each
            # round: what this round's audits found crowded at the ends, and the nets it left open, are written for
            # the model, as a round's findings are for its fanout -- `o`/group_findings.json, group_ends --feedback)
            GF = os.path.join(o, 'group_findings.json')
            ghots = []
            # (the loop on the model's own ends, and the one on the ends this step repaired, where it did)
            for ld_ in dict.fromkeys([os.path.join(d, 'loop'), loopd]):
                for a in sorted(glob.glob(os.path.join(ld_, 'p*.audit'))) + \
                        sorted(glob.glob(os.path.join(ld_, 'q*.audit'))):
                    j, h = a[:-len('.audit')] + '.json', a[:-len('.audit')] + '.fbhot.json'
                    run(['whole_gate.py', j, a, '--hot', h], env)
                    ghots.append(h)
                ghots += sorted(glob.glob(os.path.join(ld_, 'hs*.json')))
            say('  for the group\'s model: ' + stripped(run(
                ['whole_feedback.py', os.path.join(d, 'fo.plan.json'), GF] + ghots,
                {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1]))
            # (...and, in a file of their own, the findings that are THE ENDS' OWN and STOOD: a pitch or a static
            # clearance at the ends -- what no solve moves, whole_feedback --now -- in a loop's last audit and the
            # one before it. The model rules those ends out; the rest it prices. Every audit's findings ruled out
            # together, a bus with three rounds of them had 17 ends and 8 pairs named, and no state)
            hhots = standing_hots(d, loopd)
            GH = os.path.join(o, 'group_findings_hard.json')
            if os.path.exists(GH):
                os.remove(GH)
            if hhots:
                say('  for the group\'s model, to rule out: ' + stripped(run(
                    ['whole_feedback.py', os.path.join(d, 'fo.plan.json'), GH] + hhots,
                    {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1]))
            # (...and the ends of the nets the round left OPEN, each at the end where its trouble is: a net the route
            # could not close on these ends has an end that does not serve it. Priced alone, a bus's eleven open
            # nets had their ends wanted again and the model answered with the same state)
            if res is not None and res['open']:
                say('  for the group\'s model, to rule out: ' + stripped(run(
                    ['whole_feedback.py', '--name', os.path.join(d, 'fo.plan.json'), GH] + res['open_nets'],
                    {**env, 'FB_ROUND': str(r), 'FB_HOTS': ','.join(ghots), 'FB_CONN': os.path.join(d, 'conn.log')},
                    err=sys.stderr)[1]))
            say('  for the group\'s model: ' + stripped(run(
                ['whole_feedback.py', '--name', os.path.join(d, 'fo.plan.json'), GF]
                + (res['open_nets'] if res else crowded),
                {**env, 'FB_ROUND': str(r), 'FB_HOTS': ','.join(ghots), 'FB_CONN': os.path.join(d, 'conn.log')},
                err=sys.stderr)[1]))
        # (the agent's edits for the next round naming ENDS: a feedback round from this one, whatever F -- with the
        # findings of this round's own audits only where F or a solve with no plan asks for them)
        ed_n = EDITS.get(r + 1) or {}
        teeth_n = bool(ed_n.get('avoid') or ed_n.get('pairs') or ed_n.get('ban') or ed_n.get('pin'))
        if (fed_left or no_plan or teeth_n) and r < R:
            # ---- a FEEDBACK ROUND next -- and after a round whose solve had NO PLAN whatever F, within the R rounds: a
            # draw under the next seed may choose the same ends and find the same no room (a memory bus's four draws
            # chose the same ends, each solve with no room for one lane's crossings, and the bus was left 2 open; fed
            # the lane the solve named, the next round closed it) -- its findings: the ends the audits found crowded (a
            # loop that did not pass), and the lanes an unproved plan left over two vias, which the ends counted on
            # keeping at two
            nothing = True
            if fed_left or no_plan:
                hots = []
                if rc is not None and rc != 0:
                    for a in sorted(glob.glob(os.path.join(loopd, 'p*.audit'))) + \
                            sorted(glob.glob(os.path.join(loopd, 'q*.audit'))):
                        j, h = a[:-len('.audit')] + '.json', a[:-len('.audit')] + '.fbhot.json'
                        run(['whole_gate.py', j, a, '--hot', h], env)
                        hots.append(h)
                    hots += sorted(glob.glob(os.path.join(loopd, 'hs*.json')))     # the snapped plans' places
                new_over = add_over(FBD, J.get('over_nets') if J and not proved else None)
                # (the round, so an end named again in a later one is raised, not added: whole_feedback)
                fbl = stripped(run(['whole_feedback.py', os.path.join(d, 'fo.plan.json'), FBD] + hots,
                                   {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1])
                say(fbl)
                if new_over:
                    say(f"  the ends to count {', '.join(new_over)} over two vias: the solve could not keep them at two")
                nothing = 'whole_feedback: 0 new' in fbl and not new_over
                if (res is not None and res['open']) or (nothing and res is None):
                    # a round that leaves nets OPEN names them to the fanout, whatever its audits found -- and one that
                    # lays NOTHING with nothing new from its audits names the lanes the ends model names on these ends (its
                    # nets over two, else its lanes loaded on the trunk, else its most crossed): their ends freed and to
                    # be avoided -- an open net's at the end where its trouble is, by the round's findings and its
                    # connectivity. A round with no plan names the lanes its solve's CROWD diagnosis found no room for
                    fbn = stripped(run(['whole_feedback.py', '--name', os.path.join(d, 'fo.plan.json'), FBD]
                                       + (res['open_nets'] if res else crowded),
                                       {**env, 'FB_ROUND': str(r), 'FB_HOTS': ','.join(hots),
                                        'FB_CONN': os.path.join(d, 'conn.log')}, err=sys.stderr)[1])
                    say(fbn)
                    nothing = nothing and not re.search(r'whole_feedback: [1-9]', fbn)
            if teeth_n:
                ef_ = os.path.join(d, 'edits_next.json')
                # (a pin's point in the board's frame, turned into the bench's -- the sidecar's, the ends' menus')
                tb_ = edit_frame(env, frame=ed_n.get('frame') or 'board')[0]
                json.dump(dict(ed_n, pin=[dict(p_, at=list(tb_(p_['at']))) for p_ in ed_n.get('pin') or ()]),
                          open(ef_, 'w'), indent=1)
                fbe = stripped(run(['whole_feedback.py', '--edits', os.path.join(d, 'fo.plan.json'), FBD, ef_],
                                   {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1])
                say(fbe)
                nothing = nothing and not re.search(r'whole_feedback: [1-9]', fbe)
            if not nothing:
                base = advance(d, base)
                prev, fed_left = d, max(0, fed_left - 1)
                say(f"=== round {r} left nets open -- a feedback round, under the same seed")
                continue
            say("  the findings add nothing new: a feedback round would lay the same ends")
        if r < R:
            say(f"=== round {r} left nets open -- round 1 again under the next seed")
    if best is None:
        best = last_resort(r)
    if best is None:
        g = f"WHOLE K={K} round={r} lanes=0/0 vias=0 copper=0mm connected=0 drc=0 secs={secs()} open={len(nets_all)}"
        say(g)
        return 2, g
    # the best round's board, where a caller finds it (route_bus, modal_whole): OUTDIR/best.kicad_pcb, with every
    # sibling the round's board has (copy_board.SIBLING_EXTS) -- the .kicad_dru's rules and the brief too, which the
    # board route_bus grades and hands on would otherwise go without (#1316)
    sys.path.insert(0, PYR)
    from copy_board import SIBLING_EXTS
    for ext in ('.kicad_pcb',) + tuple(SIBLING_EXTS):
        src_ = best['board'][:-len('.kicad_pcb')] + ext
        if os.path.isfile(src_):
            shutil.copyfile(src_, os.path.join(o, 'best' + ext))
    g = re.sub(r'secs=\d+', f'secs={secs()}', best['grade'])
    say(('=== the best round: ' if best['open'] or best['drc'] else '') + g)
    # the step's narrowed copper DISCLOSED as it ships: the run's nets' tracks on the best board narrower than the
    # narrowest width the step asks for (the escape's stubs) -- a lane's necked end stretches (braid.route_lane_necked),
    # the lay's own escalation pieces -- one fab ledger row per net (fab_tiers.replace_track_rows: the rows its
    # attempts noted, every round's, replaced by what ships), and said here. (Not the whole ledger's line: it holds the
    # rest of the process's notes per attempt, every round's fanout of the arrays' other balls among them)
    import fab_tiers as _ft
    import rules as _rules_
    with contextlib.redirect_stdout(io.StringIO()):
        pb_ = parse_kicad_pcb(os.path.join(o, 'best.kicad_pcb'))
    req_ = min(_rules_.active().fan_track, _rules_.active().track)
    set_ = set(nets_all)
    mine_ = {i_: nn_.name for i_, nn_ in pb_.nets.items() if nn_.name and keys_.get(nn_.name, nn_.name) in set_}
    rows_ = []
    for i_, nm_ in sorted(mine_.items()):
        ss_ = [s_ for s_ in pb_.segments if s_.net_id == i_ and s_.width < req_ - 1e-9]
        if ss_:
            rows_.append((i_, nm_, req_, min(s_.width for s_ in ss_),
                          sum(math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y) for s_ in ss_)))
    _ft.replace_track_rows(list(mine_), rows_, 'bus lane end stretch')
    if rows_:
        say(f"  narrowed as shipped: {len(rows_)} of the run's nets carry track narrower than the escape's stubs "
            f"({req_:g} mm), the narrowest {min(r_[3] for r_ in rows_):g} mm -- "
            + ', '.join(f'{r_[1]} {r_[4]:.2f} mm' for r_ in rows_))
    return (0 if best['open'] == 0 and best['drc'] == 0 else 1), g


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('first', metavar='K|SOLVE', help="the rung on the bench's ladder, or with --loop the solve JSON")
    ap.add_argument('outdir', metavar='OUTDIR')
    ap.add_argument('rounds', metavar='ROUNDS', nargs='?', type=int,
                    help="the fanout rounds (default 4), or with --loop the loop's rounds (default 6)")
    ap.add_argument('--feedback-rounds', type=int, default=0, metavar='F',
                    help="after each draw that leaves nets open, up to F rounds under its seed fed its findings "
                         "(default 0: every round a draw)")
    ap.add_argument('--loop', action='store_true',
                    help='the loop alone on a solve, the bench from BENCH / NETS / DEST as every whole_* stage reads it')
    ap.add_argument('--inproc', action='store_true',
                    help='every stage in this process rather than each in its own')
    a = ap.parse_args()
    # (before the run is spent: its solves and fanouts plan with ortools, each stage failing in a log of its own
    # without it, #1355)
    sys.path.insert(0, PYR)
    from startup_checks import StartupCheckError, check_bus_dependencies
    try:
        check_bus_dependencies()
    except StartupCheckError as e:
        print(f'whole_route: {e}', file=sys.stderr)
        sys.exit(2)
    global INPROC
    INPROC = a.inproc
    if a.loop:
        sys.exit(loop(a.first, os.path.abspath(a.outdir), a.rounds or 6, awx_settings.environ(), Log()))
    sys.exit(chain(a.first, a.outdir, a.rounds or 4, F=a.feedback_rounds)[0])


if __name__ == '__main__':
    main()
