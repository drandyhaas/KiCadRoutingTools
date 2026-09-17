#!/usr/bin/env python3
"""#973: a lever that DELIVERS by copy or rename leaves a row naming the output.

Three paths built their board somewhere else and then put it in place with no
row that could vouch for it, so under an armed regime a run made only through
registered levers did not audit CLEAN:

  * `place_seed --repair/--reseat` staged both passes in a temp dir and
    delivered with `write_placed_output(cur, out, [])` -- a row that claimed
    nothing and read a temp board no row produced. Measured on main: every
    repaired or re-seated part came back UNCLAIMED, exit 4.
  * `place_seed`'s polish (and its re-seat fix) wrote `<out>.polish` and
    renamed it over the output. Measured on main: exit 4 with the 34 parts the
    polish moved named as DRIFTED, nothing hand-edited.
  * `place_route_loop` delivered with `shutil.copy(cur_file, out)`. Measured
    on main: the audit picked a ROUND board instead of the output, so a hand
    edit of the output graded CLEAN; with `--work-dir` outside the regime there
    was no ledger at all, exit 4.

Every case here drives the DELIVERY -- the real CLI, or `main()` with only the
router and the quench replaced -- and reads the ledger and the audit, never
just the helper's return value. The helper's own contract (refuse before the
body, commit after it, nothing left pending) is section 1.
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools', 'py_placer', 'tests',
           os.path.join('tests', 'stress')):
    _q = os.path.join(REPO, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

SF = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not os.path.isfile(SF):
    print('SKIP: fixture missing')
    sys.exit(77)

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import provenance as PV                         # noqa: E402
from placement.writer import write_placed_output               # noqa: E402
from placement.floorplan import emit_intent                    # noqa: E402
import provenance_audit as PA                                  # noqa: E402

_sink = io.StringIO()


def quiet(f, *a, **k):
    with contextlib.redirect_stdout(_sink), contextlib.redirect_stderr(_sink):
        return f(*a, **k)


def armed(board_text_or_path, name='board.kicad_pcb'):
    d = tempfile.mkdtemp(prefix='t973_')
    wd = os.path.join(d, 'wk')
    os.makedirs(wd)
    staged = os.path.join(wd, name)
    shutil.copyfile(board_text_or_path, staged)
    PV.start_regime(wd, staged)
    return d, wd, staged


def settled(tag):
    check(f'{tag}: nothing pending, no lever left declared',
          PV._PENDING == {} and PV.active_lever() is None,
          f'pending={list(PV._PENDING)} lever={PV.active_lever()}')


def grade(tag, wd, board, want_code, **want):
    code, doc = PA.audit(wd, board)
    ok = code == want_code and all(doc.get(k) == v for k, v in want.items())
    check(tag, ok, f"exit {code} {doc.get('verdict')} lineage={doc.get('lineage')} "
                   f"delivered={os.path.basename(str(doc.get('delivered')))} "
                   f"drifted={doc.get('drifted_refs')} unclaimed={doc.get('unclaimed_refs')}")
    return code, doc


def rows_naming(wd, path):
    # REALPATH, not abspath: a row's path is resolved against the process's
    # cwd at record time, and on macOS `tempfile.mkdtemp()` hands back
    # `/var/folders/...` while `os.getcwd()` inside it reports the resolved
    # `/private/var/folders/...` (`/var` is a symlink to `private/var`).
    # `abspath` does not follow symlinks, so a row recorded through a relative
    # path under a temp dir compares unequal to the same file named from the
    # dir handle -- the row is correct and the comparison was not.
    want = os.path.normcase(os.path.realpath(path))
    return [r for r in PV.read_ledger(wd)
            if os.path.normcase(os.path.realpath(r.get('path') or '')) == want]


# ==========================================================================
# 1. the helper
# ==========================================================================
print('1. recorded_delivery')
_refs = sorted(parse_kicad_pcb(SF).footprints)
_fp = parse_kicad_pcb(SF).footprints[_refs[0]]
_move = [{'reference': _refs[0], 'new_x': _fp.x + 3.0, 'new_y': _fp.y,
          'new_rotation': _fp.rotation}]

d, wd, staged = armed(SF)
src = os.path.join(d, 'staged_elsewhere.kicad_pcb')
quiet(write_placed_output, staged, src, _move)
OUT = os.path.join(wd, 'out.kicad_pcb')
with open(OUT, 'w', encoding='utf-8') as f:
    f.write('previous deliverable')
with open(os.path.splitext(OUT)[0] + '.kicad_pro', 'w', encoding='utf-8') as f:
    f.write('{"sentinel": true}')
_ran = []
try:
    with PV.recorded_delivery(staged, OUT, _move):
        _ran.append(True)
        shutil.copyfile(src, OUT)
    _err = None
except PV.UnaidedViolation as e:
    _err = e
check('an undeclared delivery into an armed regime is refused', _err is not None
      and 'no registered lever' in str(_err), repr(_err))
check('...BEFORE the body runs: the output and its sibling are untouched',
      not _ran and open(OUT, encoding='utf-8').read() == 'previous deliverable'
      and open(os.path.splitext(OUT)[0] + '.kicad_pro', encoding='utf-8').read()
      == '{"sentinel": true}')
check('...and no row was written', PV.read_ledger(wd) == [])
settled('undeclared')

try:
    with PV.declare_lever('hand_tool.py'):
        with PV.recorded_delivery(staged, OUT, _move):
            _ran.append(True)
    _err = None
except PV.UnaidedViolation as e:
    _err = e
check('a declared but UNREGISTERED lever is refused before the body',
      _err is not None and not _ran and PV.read_ledger(wd) == [], repr(_err))
settled('unregistered')

with PV.declare_lever('place_seed.py'):
    with PV.recorded_delivery(staged, OUT, _move) as _row:
        shutil.copyfile(src, OUT)
_r = rows_naming(wd, OUT)
check('a declared delivery commits ONE row naming the output',
      len(_r) == 1 and _r[0]['refs_moved'] == [_refs[0]] and _row is not None,
      f"{len(_r)} row(s), refs_moved={[x.get('refs_moved') for x in _r]}")
check('...whose board digests are the DELIVERED file\'s, taken after the body',
      _r and _r[0]['board_sha256'] == PV.sha256_file(OUT)
      and _r[0]['board_pose_sha256'] == PV.file_pose_digest(OUT))
check('...and whose parent is the real input, not the staged board',
      _r and _r[0]['parent_pose_sha256'] == PV.file_pose_digest(staged))
check('...and whose caller is the code that delivered, not contextlib',
      _r and os.path.basename(__file__) in _r[0]['caller'], _r and _r[0]['caller'])
grade('...which audits CLEAN', wd, OUT, PA.CLEAN, lineage='verified')
settled('declared')

d, wd, staged = armed(SF)
OUT = os.path.join(wd, 'out.kicad_pcb')
with PV.declare_lever('place_seed.py'):
    try:
        with PV.recorded_delivery(staged, OUT, _move):
            raise OSError('injected copy failure')
    except OSError:
        pass
check('a body that raises leaves no row and nothing pending',
      PV.read_ledger(wd) == [] and PV._PENDING == {})

with PV.declare_lever('place_seed.py'):
    with PV.recorded_delivery(staged, OUT, _move):
        # A writer call to the SAME path inside the body keys its own pending
        # row there; the helper's row must survive it.
        quiet(write_placed_output, staged, OUT, _move)
_r = rows_naming(wd, OUT)
check('a writer call to the same path inside the body does not swallow the row',
      len(_r) == 2, f'{len(_r)} row(s)')
settled('nested')

# A RELATIVE output path and a body that changes directory: the row must be
# committed against the path resolved before the body.
d, wd, staged = armed(SF)
src = os.path.join(d, 'staged_elsewhere.kicad_pcb')
quiet(write_placed_output, staged, src, _move)
_cwd = os.getcwd()
os.chdir(wd)
try:
    with PV.declare_lever('place_seed.py'):
        with PV.recorded_delivery(staged, 'rel_out.kicad_pcb', _move):
            shutil.copyfile(src, 'rel_out.kicad_pcb')
            os.chdir(d)
finally:
    os.chdir(_cwd)
check('a relative output path survives a directory change in the body',
      len(rows_naming(wd, os.path.join(wd, 'rel_out.kicad_pcb'))) == 1,
      f'{len(PV.read_ledger(wd))} row(s)')
settled('relative')

_calls = []
_real = PV.pose_footprints
PV.pose_footprints = lambda p: (_calls.append(p), _real(p))[1]
try:
    _free = tempfile.mkdtemp(prefix='t973_free_')
    with PV.recorded_delivery(SF, os.path.join(_free, 'o.kicad_pcb'), _move) as _row:
        shutil.copyfile(SF, os.path.join(_free, 'o.kicad_pcb'))
finally:
    PV.pose_footprints = _real
check('outside a regime it does nothing and parses nothing',
      _row is None and _calls == [], f'row={_row} parses={len(_calls)}')


# ==========================================================================
# 2. place_seed, the real CLI
# ==========================================================================
print('2. place_seed deliveries')
SEED = os.path.join(REPO, 'py_placer', 'place_seed.py')


def run_seed(inp, out, intent, *flags):
    r = subprocess.run([sys.executable, '-X', 'utf8', '-B', SEED, inp, out,
                        '--intent', intent, *flags],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=900)
    summ = {}
    for line in (r.stdout or '').splitlines():
        if line.startswith('JSON_SUMMARY:'):
            summ = json.loads(line.split(':', 1)[1])
    return r, summ


def intent_file(d, board):
    p = os.path.join(d, 'intent.json')           # OUTSIDE the work dir
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(emit_intent(parse_kicad_pcb(board), board, declare_classes=True), f)
    return p


def damaged(kind):
    """splitflap with parts pushed off the outline (reseat) and/or R1 over
    the bottom edge (repair), written outside any regime."""
    pcb = parse_kicad_pcb(SF)
    b = pcb.board_info.board_bounds
    moves = []
    if 'reseat' in kind:
        victims = [r for r, f in sorted(pcb.footprints.items()) if f.pads][:3]
        for i, ref in enumerate(victims):
            moves.append({'reference': ref, 'new_x': b[2] + 30.0 + 5.0 * i,
                          'new_y': (b[1] + b[3]) / 2,
                          'new_rotation': pcb.footprints[ref].rotation or 0})
    if 'repair' in kind:
        r1 = pcb.footprints['R1']
        moves.append({'reference': 'R1', 'new_x': r1.x, 'new_y': b[3] + 0.3,
                      'new_rotation': r1.rotation or 0})
    out = os.path.join(tempfile.mkdtemp(prefix='t973_dmg_'), 'dmg.kicad_pcb')
    quiet(write_placed_output, SF, out, moves)
    return out


for kind, flags, intent_from in (('reseat', ['--reseat'], 'staged'),
                                 ('repair', ['--repair'], 'source'),
                                 ('reseat+repair', ['--reseat', '--repair'], 'source'),
                                 ('none', ['--repair'], 'source')):
    src = damaged(kind) if kind != 'none' else SF
    d, wd, staged = armed(src)
    intent = intent_file(d, staged if intent_from == 'staged' else SF)
    OUT = os.path.join(wd, 'out.kicad_pcb')
    r, summ = run_seed(staged, OUT, intent, *flags)
    _claimed = set(summ.get('moved_refs') or []) | set(summ.get('reseated_refs') or []) \
        | set(summ.get('evicted') or [])
    _rows = rows_naming(wd, OUT)
    _delivery = [x for x in _rows if x.get('parent_pose_sha256') == PV.file_pose_digest(staged)]
    check(f'--{kind}: exit {r.returncode}, and a row names the output with the real input as parent',
          r.returncode in (0, 4) and len(_delivery) == 1,
          f'rc={r.returncode} rows={len(_rows)} {(r.stderr or "")[-300:]}')
    if kind != 'none':
        check(f'--{kind}: the pass really moved parts (the witness is live)', bool(_claimed),
              str(sorted(_claimed)) if _claimed else str(summ)[:400])
    check(f'--{kind}: that row claims every part the summary says moved',
          _delivery and _claimed <= set(_delivery[0].get('refs_moved') or []),
          f"summary={sorted(_claimed)} row={_delivery and _delivery[0].get('refs_moved')}")
    grade(f'--{kind}: the run audits CLEAN', wd, OUT, PA.CLEAN, lineage='verified')
    grade(f'--{kind}: ...and so does the board the audit picks itself', wd, None, PA.CLEAN)
    check(f'--{kind}: the operator is told where the board was delivered',
          f'Delivered {OUT}' in (r.stdout or ''))
    check(f'--{kind}: no temp board is named by any row',
          all(os.path.dirname(os.path.realpath(x.get('path') or '')) == os.path.realpath(wd)
              for x in PV.read_ledger(wd)),
          str([x.get('path') for x in PV.read_ledger(wd)]))

# The default seed, polish ON: the polish's rename is recorded against the
# output, and the lock stamp between the seed write and the polish does not
# break the lineage.
import stage_unaided as SU                                     # noqa: E402
d = tempfile.mkdtemp(prefix='t973_seed_')
wd = os.path.join(d, 'wk')
os.makedirs(wd)
os.makedirs(os.path.join(d, 'truth'))
staged = os.path.join(wd, 'board.kicad_pcb')
quiet(SU.stage, SF, staged, os.path.join(d, 'truth'))          # a pile, armed
intent = intent_file(d, SF)
OUT = os.path.join(wd, 'seeded.kicad_pcb')
r, summ = run_seed(staged, OUT, intent)
_rows = rows_naming(wd, OUT)
_polish = [x for x in PV.read_ledger(wd) if (x.get('path') or '').endswith('.polish')]
check('seed+polish: the polish ran (the witness is live)', len(_polish) == 1
      and _polish[0].get('refs_moved'), f'rc={r.returncode} polish rows={len(_polish)}')
check('seed+polish: the rename is recorded against the output, same moves',
      len(_rows) == 2 and _polish
      and _rows[-1].get('refs_moved') == _polish[0].get('refs_moved')
      and _rows[-1].get('board_pose_sha256') == PV.file_pose_digest(OUT),
      f'{len(_rows)} row(s) naming the output')
grade('seed+polish: CLEAN (main: 34 polished parts named DRIFTED)', wd, OUT, PA.CLEAN,
      lineage='verified', drifted_refs=[])

# The RE-SEAT FIX: a polish that walks a part into a declared keep-out is
# re-seated against the polished board and renamed onto the output a second
# time. An emitted intent declares no keep-out and the real quench does not
# walk into one, so both are supplied: a keep-out in the intent, and a quench
# that returns its real placements plus one part moved into it.
import placement.quench as _Q                                   # noqa: E402
import place_seed                                               # noqa: E402
d = tempfile.mkdtemp(prefix='t973_fix_')
wd = os.path.join(d, 'wk')
os.makedirs(wd)
os.makedirs(os.path.join(d, 'truth'))
staged = os.path.join(wd, 'board.kicad_pcb')
quiet(SU.stage, SF, staged, os.path.join(d, 'truth'))
_KO = [150.0, 35.0, 162.0, 45.0]
_doc = emit_intent(parse_kicad_pcb(SF), SF)
_doc['keepouts'] = [{'name': 'ko_test', 'rect': _KO}]
intent = os.path.join(d, 'intent.json')
with open(intent, 'w', encoding='utf-8') as fh:
    json.dump(_doc, fh)
OUT = os.path.join(wd, 'seeded.kicad_pcb')
_real_quench, _walked = _Q.quench, {}


def _quench_into_keepout(pcb_data, **kw):
    pl = list(_real_quench(pcb_data, **kw))
    taken = {p['reference'] for p in pl}
    for ref, f in sorted(pcb_data.footprints.items()):
        if (len(f.pads) == 2 and not getattr(f, 'locked', False) and ref not in taken
                and not (_KO[0] <= f.x <= _KO[2] and _KO[1] <= f.y <= _KO[3])):
            _walked['ref'] = ref
            return pl + [{'reference': ref, 'new_x': (_KO[0] + _KO[2]) / 2,
                          'new_y': (_KO[1] + _KO[3]) / 2, 'new_rotation': f.rotation or 0.0}]
    return pl


_Q.quench = _quench_into_keepout
_argv = sys.argv
sys.argv = ['place_seed.py', staged, OUT, '--intent', intent]
try:
    with PV.declare_lever('place_seed.py', sys.argv):
        quiet(place_seed.main)
finally:
    sys.argv, _Q.quench = _argv, _real_quench
_fix = [x for x in PV.read_ledger(wd) if (x.get('path') or '').endswith('.reseat')]
_rows = rows_naming(wd, OUT)
check('re-seat fix: the branch ran (a .reseat write moved the walked part)',
      len(_fix) == 1 and _walked.get('ref') in (_fix[0].get('refs_moved') or []),
      f"walked={_walked.get('ref')} reseat rows={len(_fix)}")
check('re-seat fix: its rename is recorded against the output, same moves',
      len(_rows) == 3 and _fix and _rows[-1].get('refs_moved') == _fix[0].get('refs_moved')
      and _rows[-1].get('board_pose_sha256') == PV.file_pose_digest(OUT),
      f'{len(_rows)} row(s) naming the output')
grade('re-seat fix: CLEAN', wd, OUT, PA.CLEAN, lineage='verified')
settled('re-seat fix')

# --dry-run delivers nothing and records nothing.
src = damaged('repair')
d, wd, staged = armed(src)
intent = intent_file(d, SF)
OUT = os.path.join(wd, 'dry.kicad_pcb')
r, summ = run_seed(staged, OUT, intent, '--repair', '--dry-run')
check('--repair --dry-run: no output and no row',
      r.returncode in (0, 4) and not os.path.exists(OUT) and PV.read_ledger(wd) == []
      and summ.get('moved_refs'),
      f"rc={r.returncode} out={os.path.exists(OUT)} rows={len(PV.read_ledger(wd))} "
      f"moved={summ.get('moved_refs')}")

# Undeclared: `main()` in-process with no lever. The passes stage outside the
# regime, so the delivery is the first write the regime sees -- and it must
# refuse before the output or its siblings exist.
src = damaged('repair')
d, wd, staged = armed(src)
# A project beside the input: the passes carry it to every staged board, so a
# delivery that copied siblings BEFORE refusing would leave one at the output.
with open(os.path.splitext(staged)[0] + '.kicad_pro', 'w', encoding='utf-8') as f:
    f.write('{}')
intent = intent_file(d, SF)
OUT = os.path.join(wd, 'undeclared.kicad_pcb')
import place_seed                                              # noqa: E402
_argv = sys.argv
sys.argv = ['place_seed.py', staged, OUT, '--intent', intent, '--repair']
try:
    quiet(place_seed.main)
    _err = None
except PV.UnaidedViolation as e:
    _err = e
finally:
    sys.argv = _argv
check('an undeclared --repair delivery is refused', _err is not None
      and 'no registered lever' in str(_err), repr(_err))
check('...before the output or its project exist',
      not os.path.exists(OUT) and not os.path.exists(os.path.splitext(OUT)[0] + '.kicad_pro'))
check('...and records nothing', PV.read_ledger(wd) == [])
settled('place_seed')


# ==========================================================================
# 3. place_route_loop, main() with the router and the quench replaced
# ==========================================================================
print('3. place_route_loop delivery')
import place_route_loop as prl                                 # noqa: E402
from test_458_loop_steering import _loop_board                 # noqa: E402


def run_loop(OUT, staged, *, rounds, quench_moves, failures, work_dir=None,
             declare=True, tamper=None, relocation=None):
    """`quench_moves[i]` is round i+1's quench result; `failures[i]` the
    failure count round i routes to (round 0 first). `relocation` is a canned
    `Relocation` every round proposes, with `--relocate --group-by decap`."""
    calls = {'q': 0, 'r': 0}

    def fake_quench(pcb_data, **kw):
        calls['q'] += 1
        return quench_moves[min(calls['q'], len(quench_moves)) - 1]

    def fake_route(pcb_file, routed_file, route_args, log_file, **kw):
        n = calls['r']
        calls['r'] += 1
        if tamper and n > 0:
            tamper(pcb_file)
        shutil.copyfile(pcb_file, routed_file)
        return {'failures': failures[min(n, len(failures) - 1)],
                'failed_nets': ['NA'], 'blockers': [], 'iterations': 1000,
                'vias': 0, 'blocker_report': None}

    argv = ['place_route_loop.py', staged, OUT, '--route-args', '--nets "*"',
            '--rounds', str(rounds), '--max-displacement', '3.0', '--no-movie']
    if work_dir:
        argv += ['--work-dir', work_dir]
    if relocation is not None:
        argv += ['--relocate', '--group-by', 'decap']
    saved = (prl.quench, prl.run_route, prl.relocate_round, sys.argv)
    prl.quench, prl.run_route, sys.argv = fake_quench, fake_route, argv
    if relocation is not None:
        prl.relocate_round = lambda *a, **k: relocation
    try:
        ctx = PV.declare_lever('place_route_loop.py', argv) if declare \
            else contextlib.nullcontext()
        with ctx:
            return quiet(prl.main)
    finally:
        prl.quench, prl.run_route, prl.relocate_round, sys.argv = saved


def loop_dir():
    src, _tmp = _loop_board()
    d, wd, staged = armed(src)
    os.unlink(src)
    return d, wd, staged


C1a = [{'reference': 'C1', 'new_x': 151.0, 'new_y': 100.0, 'new_rotation': 0.0}]
C1b = [{'reference': 'C1', 'new_x': 152.5, 'new_y': 101.0, 'new_rotation': 90.0}]
J1a = [{'reference': 'J1', 'new_x': 171.0, 'new_y': 100.0, 'new_rotation': 0.0}]

d, wd, staged = loop_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=1, quench_moves=[C1a], failures=[2, 1])
_r = rows_naming(wd, OUT)
check('default work dir: one row names the output, claiming the accepted move',
      len(_r) == 1 and _r[0]['refs_moved'] == ['C1']
      and _r[0]['parent_pose_sha256'] == PV.file_pose_digest(staged),
      f"{len(_r)} row(s), refs_moved={[x.get('refs_moved') for x in _r]}")
grade('default work dir: the audit picks the OUTPUT and grades it CLEAN', wd, None,
      PA.CLEAN, lineage='verified')
_code, _doc = PA.audit(wd)
check('...the output, not a round board',
      os.path.basename(str(_doc.get('delivered'))) == 'OUT.kicad_pcb', str(_doc.get('delivered')))

d, wd, staged = loop_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
outside_work = os.path.join(d, 'loopwork')
run_loop(OUT, staged, rounds=2, quench_moves=[C1a, J1a], failures=[2, 1, 0],
         work_dir=outside_work)
_r = PV.read_ledger(wd)
check('outside work dir, two accepted rounds: exactly one row, naming the output',
      len(_r) == 1 and os.path.basename(_r[0]['path']) == 'OUT.kicad_pcb',
      str([os.path.basename(x.get('path') or '') for x in _r]))
check('...claiming BOTH rounds\' moves against the real input',
      _r and _r[0]['refs_moved'] == ['C1', 'J1']
      and _r[0]['parent_pose_sha256'] == PV.file_pose_digest(staged),
      str(_r and _r[0]['refs_moved']))
grade('outside work dir: CLEAN (main: no ledger at all, exit 4)', wd, OUT, PA.CLEAN,
      lineage='verified')

d, wd, staged = loop_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=2, quench_moves=[C1a, C1b], failures=[2, 1, 1],
         work_dir=os.path.join(d, 'loopwork'))
_r = rows_naming(wd, OUT)
check('a REJECTED round\'s moves are not claimed: the accepted pose is',
      len(_r) == 1 and _r[0]['poses_written'].get('C1', [None])[:2] == [151.0, 100.0],
      str(_r and _r[0]['poses_written']))
grade('...and the output audits CLEAN', wd, OUT, PA.CLEAN, lineage='verified')

d, wd, staged = loop_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=1, quench_moves=[C1a], failures=[2, 2],
         work_dir=os.path.join(d, 'loopwork'))
_r = rows_naming(wd, OUT)
check('every round rejected: the row names the output and claims nothing',
      len(_r) == 1 and _r[0]['refs_moved'] == [],
      f"{len(_r)} row(s), refs_moved={[x.get('refs_moved') for x in _r]}")
grade('...CLEAN, nothing moved', wd, OUT, PA.CLEAN, lineage='verified')


def _hand_move_j1(pcb_file):
    """What a person editing a round board mid-run would leave behind."""
    fp = parse_kicad_pcb(pcb_file).footprints['J1']
    tmp = os.path.join(tempfile.mkdtemp(prefix='t973_hand_'), 'h.kicad_pcb')
    quiet(write_placed_output, pcb_file, tmp,
          [{'reference': 'J1', 'new_x': fp.x + 8.0, 'new_y': fp.y, 'new_rotation': 0.0}])
    shutil.copyfile(tmp, pcb_file)


d, wd, staged = loop_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=1, quench_moves=[C1a], failures=[2, 1],
         work_dir=os.path.join(d, 'loopwork'), tamper=_hand_move_j1)
grade('a hand edit of the accepted round board is NOT blessed by the delivery', wd, OUT,
      PA.VIOLATION, unclaimed_refs=['J1'])


def _hand_move_j1_round1(pcb_file):
    if os.path.basename(pcb_file) == 'loop_round1.kicad_pcb':
        _hand_move_j1(pcb_file)


# ...and in the DEFAULT work dir, where the next accepted round's quench then
# carries the hand-moved part on: the delivery row's claims reach the board,
# but round 2's own row read a board nothing recorded. That row is evidence.
J1c = [{'reference': 'J1', 'new_x': 178.2, 'new_y': 100.0, 'new_rotation': 0.0}]
d, wd, staged = loop_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=2, quench_moves=[C1a, J1c], failures=[2, 1, 0],
         tamper=_hand_move_j1_round1)
_c, _d = grade('a hand edit a later round carried on is not CLEAN (default work dir)', wd,
               OUT, PA.UNPROVEN, lineage='broken')

d, wd, staged = loop_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
with open(os.path.splitext(staged)[0] + '.kicad_pro', 'w', encoding='utf-8') as f:
    f.write('{}')
try:
    # The round is REJECTED on purpose: the delivered board is then the
    # round-0 copy, which carries the project, so a delivery that copied
    # siblings before refusing would leave one at the output. An accepted
    # round board is written without siblings and could not show it.
    run_loop(OUT, staged, rounds=1, quench_moves=[C1a], failures=[2, 2],
             work_dir=os.path.join(d, 'loopwork'), declare=False)
    _err = None
except PV.UnaidedViolation as e:
    _err = e
check('an undeclared loop delivering into an armed regime is refused', _err is not None
      and 'no registered lever' in str(_err), repr(_err))
check('...before the output or its project exist',
      not os.path.exists(OUT) and not os.path.exists(os.path.splitext(OUT)[0] + '.kicad_pro'))
check('...and records nothing', PV.read_ledger(wd) == [])
settled('loop')

# --relocate: the round writes the relocation first and quenches over it, and
# the delivery must claim both, in that order, key by key.
from test_554_loop_relocate import _board as _reloc_board, _proposal  # noqa: E402


def reloc_dir():
    src, _tmp = _reloc_board()
    d, wd, staged = armed(src)
    os.unlink(src)
    return d, wd, staged


R1q = [{'reference': 'R1', 'new_x': 161.0, 'new_y': 100.0, 'new_rotation': 0.0}]
d, wd, staged = reloc_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=1, quench_moves=[R1q], failures=[2, 1],
         work_dir=os.path.join(d, 'loopwork'), relocation=_proposal())
_r = rows_naming(wd, OUT)
check('--relocate, outside work dir: the delivery claims the relocation AND the quench',
      len(_r) == 1 and _r[0]['refs_moved'] == ['M1', 'R1'],
      str([x.get('refs_moved') for x in _r]))
grade('--relocate: CLEAN', wd, OUT, PA.CLEAN, lineage='verified')

# The quench re-moves the part the relocation moved, and also turns it: the
# claim is the quench's pose, which only the relocation-then-quench order gives.
M1q = [{'reference': 'M1', 'new_x': 125.5, 'new_y': 91.0, 'new_rotation': 90.0}]
d, wd, staged = reloc_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=1, quench_moves=[M1q], failures=[2, 1],
         work_dir=os.path.join(d, 'loopwork'), relocation=_proposal())
_r = rows_naming(wd, OUT)
check('--relocate then a quench of the same part: the claim is the FINAL pose',
      len(_r) == 1 and _r[0]['poses_written'].get('M1') == [125.5, 91.0, 90.0],
      str(_r and _r[0]['poses_written']))
grade('...and CLEAN', wd, OUT, PA.CLEAN, lineage='verified')

# A later move that says `new_side: None` ("keep the current side") must not
# wipe the earlier move's flip from the claim: the board keeps the flip.
import dataclasses                                             # noqa: E402
_flip = dataclasses.replace(_proposal(), moves=(
    {'reference': 'M1', 'new_x': 124.0, 'new_y': 90.0, 'new_rotation': 0.0,
     'new_side': 'B'},))
d, wd, staged = reloc_dir()
OUT = os.path.join(wd, 'OUT.kicad_pcb')
run_loop(OUT, staged, rounds=1,
         quench_moves=[[dict(M1q[0], new_side=None)]], failures=[2, 1],
         work_dir=os.path.join(d, 'loopwork'), relocation=_flip)
check('fixture: the relocation flipped M1 and the delivered board keeps it',
      PV.pose_table(OUT).get('M1', (0, 0, 0, None))[3] == 'B',
      str(PV.pose_table(OUT).get('M1')))
grade('a later None side does not wipe an earlier flip from the claim', wd, OUT,
      PA.CLEAN, lineage='verified')
settled('relocate')


print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
