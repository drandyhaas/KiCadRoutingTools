#!/usr/bin/env python3
"""#1906: a board THIS repo's code saves through pcbnew keeps its records.

KiCad 10.0.7 and later replace a board's properties with its project's text
variables before every `pcbnew.SaveBoard` (`BOARD::SynchronizeProperties`),
so the board copy of the tool records -- `(property "kicad_routing_tools"
...)`, `tool_records` -- was dropped from every board the tool saved through
pcbnew (a `run_plan.py -o` output, its per-step snapshots, the GUI's staged
copies, a filled deliverable), and from the board in memory. Every such save
now goes through `tool_records.save_board`, which writes the record back to
the file (`tool_records.write_board_namespace`) and to the live board.

The KiCad installed here may keep the property anyway (10.0.6 does), so the
pcbnew half SIMULATES 10.0.7+: `pcbnew.SaveBoard` is wrapped to replace the
board's properties with its project's text variables first, as those
versions do. The user's own save in KiCad does not come through here and is
not covered.

Part 1 needs no KiCad:
  1. write_board_namespace puts the record where pcbnew writes it (after
     `(setup ...)`, a string holding a ')' read as a string), replaces one
     already there, leaves the rest of the file byte for byte, and is a no-op
     when the file already carries it;
  2. save_board over a save that drops the properties (10.0.7+) gives the
     file and the live board their record back; over one that keeps them
     (10.0.6) it rewrites nothing;
  3. no code here calls `SaveBoard` directly any more: every call passes it
     to `tool_records.save_board` (the pcbnew refill script included).
Part 2 runs under KiCad's python (re-exec; exit 77 without one) with the
10.0.7+ save simulated:
  4. `run_plan` with one routing step: the saved board and its per-step
     snapshot carry exactly the record the live board held at each save;
  5. `ui_thread.save_board_on_ui_thread`, the GUI's marshalled save: the same;
  6. the pcbnew refill script (`kicad_exact_fill._REFILL_SCRIPT`, which writes
     the filled deliverable): its saved board carries the source's record.

    python3 tests/test_1906_records_survive_tool_saves.py
"""
import ast
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, REPO)
import tool_records as T  # noqa: E402

PROJECTLESS = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
SEED = {'protected_nets': {'nets': ['/SEED_NET'], 'why': 'test 1906 seed'},
        'records_rev': 4}
_fails = []


def check(cond, label, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}"
          + (f" -- {detail}" if detail and not cond else ''))
    if not cond:
        _fails.append(label)


def seeded_copy(src, dst):
    """SRC copied to DST (no siblings: the board copy is then the only home
    of the record) with SEED on it, put there by text, as a fixture."""
    with open(src, encoding='utf-8', newline='') as f:
        text = f.read()
    assert T._PROP_RE.search(text) is None, 'the fixture carries a record already'
    i = text.index('\n\t(setup')
    j = T._node_end(text, i + 2)
    with open(dst, 'w', encoding='utf-8', newline='') as f:
        f.write(text[:j] + '\n\t' + T.board_property_sexpr(SEED) + text[j:])
    assert T.board_namespace(dst) == SEED
    return dst


# ---- part 1 -----------------------------------------------------------------

class FakeBoard:
    """GetFileName / GetProperties / SetProperties, as tool_records uses them."""
    def __init__(self, path, props):
        self.path, self.props = path, dict(props)

    def GetFileName(self):
        return self.path

    def GetProperties(self):
        return dict(self.props)

    def SetProperties(self, props):
        self.props = dict(props)


BASE = ('(kicad_pcb (version 20241229) (generator "pcbnew")\n'
        '\t(general\n\t\t(thickness 1.6)\n\t)\n'
        '\t(setup\n\t\t(pcbplotparams\n\t\t\t(outputdirectory "gerbers) (x")\n\t\t)\n\t)\n'
        '\t(property "other" "kept")\n'
        '\t(footprint "R"\n\t\t(property "Reference" "R1")\n\t)\n'
        ')\n')


def fake_save(drops):
    """A SaveBoard stand-in writing BASE plus the board's properties as pcbnew
    does; DROPS replaces them with the project's text variables (none) first,
    as KiCad 10.0.7+ does."""
    def save(path, board, aSkipSettings=False):
        if drops:
            board.SetProperties({})
        props = ''.join(f'\t(property "{k}" "{T.kicad_escape(v)}")\n'
                        for k, v in sorted(board.GetProperties().items()))
        head, tail = BASE.split('\t(property "other"', 1)
        with open(path, 'w', encoding='utf-8', newline='') as f:
            f.write(head + props + '\t(property "other"' + tail)
    return save


def part1(td):
    print('1. write_board_namespace')
    p = os.path.join(td, 'w.kicad_pcb')
    with open(p, 'w', encoding='utf-8', newline='') as f:
        f.write(BASE)
    ns = {'fab_floor_origin': {'min_clearance': 0.2}, 'records_rev': 2,
          'note': 'a "quoted" \\ value'}
    check(T.write_board_namespace(p, ns) is True, 'it writes a board with no record')
    with open(p, encoding='utf-8', newline='') as f:
        got = f.read()
    check(T.board_namespace(p) == ns, 'the board reads back the record', str(T.board_namespace(p)))
    want = BASE.replace('\t)\n\t(property "other"',
                        '\t)\n\t' + T.board_property_sexpr(ns) + '\n\t(property "other"')
    check(got == want, 'it stands right after (setup ...), past a string holding a ")", '
          'and nothing else changed', repr(got[:400]))
    check(T.write_board_namespace(p, ns) is False, 'writing the same record again changes nothing')
    ns2 = dict(ns, records_rev=3)
    T.write_board_namespace(p, ns2)
    with open(p, encoding='utf-8', newline='') as f:
        got2 = f.read()
    check(T.board_namespace(p) == ns2 and got2.count('"kicad_routing_tools"') == 1
          and got2 == want.replace(T.board_property_sexpr(ns), T.board_property_sexpr(ns2)),
          'a record already there is replaced in place', repr(got2[:400]))
    q = os.path.join(td, 'nosetup.kicad_pcb')
    with open(q, 'w', encoding='utf-8', newline='') as f:
        f.write('(kicad_pcb (version 20241229)\n\t(general)\n)\n')
    T.write_board_namespace(q, ns)
    check(T.board_namespace(q) == ns, 'a board with no (setup ...) gets it last')

    print('2. save_board')
    for drops, label in ((True, 'a save that drops the properties (KiCad 10.0.7+)'),
                         (False, 'a save that keeps them (KiCad 10.0.6)')):
        out = os.path.join(td, f'saved_{int(drops)}.kicad_pcb')
        b = FakeBoard(os.path.join(td, 'live.kicad_pcb'), {})
        T.set_live_board_namespace(b, ns)
        calls = []
        real_write = T.write_board_namespace
        T.write_board_namespace = lambda *a: calls.append(a) or real_write(*a)
        try:
            T.save_board(fake_save(drops), out, b, aSkipSettings=True)
        finally:
            T.write_board_namespace = real_write
        check(T.board_namespace(out) == ns, f'{label}: the saved file carries the record',
              str(T.board_namespace(out)))
        check(T.live_board_namespace(b) == ns, f'{label}: so does the live board',
              str(T.live_board_namespace(b)))
        check(bool(calls) == drops, f'{label}: the file is rewritten '
              + ('once' if drops else 'never'), str(calls))
    b = FakeBoard(os.path.join(td, 'bare.kicad_pcb'), {})
    out = os.path.join(td, 'bare_out.kicad_pcb')
    T.save_board(fake_save(True), out, b)
    check(T.board_namespace(out) is None, 'a board with no record gets none')

    print('3. every pcbnew save of this repo goes through tool_records.save_board')
    direct = []
    for d in ('py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin', 'awx'):
        for path in glob.glob(os.path.join(REPO, d, '**', '*.py'), recursive=True):
            with open(path, encoding='utf-8') as f:
                src = f.read()
            if 'SaveBoard' not in src:
                continue
            for node in ast.walk(ast.parse(src)):
                if isinstance(node, ast.Call) and getattr(node.func, 'attr', None) == 'SaveBoard':
                    direct.append(f'{os.path.relpath(path, REPO)}:{node.lineno}')
    check(not direct, 'no direct SaveBoard call', ', '.join(direct))
    import kicad_exact_fill
    script = kicad_exact_fill._REFILL_SCRIPT
    check('pcbnew.SaveBoard(' not in script
          and 'tool_records.save_board(pcbnew.SaveBoard, dst, board' in script,
          'the pcbnew refill script saves through it too')


# ---- part 2 (KiCad's python) ------------------------------------------------

SAVED = {}


def simulate_1007(pcbnew):
    """Wrap pcbnew.SaveBoard as KiCad 10.0.7+ behaves: the board's properties
    become its project's text variables (read off the .kicad_pro, none here)
    before the save. Records what the live board held at each save."""
    real = pcbnew.SaveBoard

    def save(path, board, *a, **k):
        SAVED[os.path.abspath(path)] = T.live_board_namespace(board)
        pro = os.path.splitext(board.GetFileName())[0] + '.kicad_pro'
        tv = {}
        if os.path.isfile(pro):
            with open(pro, encoding='utf-8') as f:
                tv = json.load(f).get('text_variables') or {}
        m = type(board.GetProperties())()
        for kk, vv in tv.items():
            m[kk] = vv
        board.SetProperties(m)
        return real(path, board, *a, **k)
    pcbnew.SaveBoard = save
    return real


def part2(td):
    import pcbnew
    from kicad_parser import parse_kicad_pcb
    real = simulate_1007(pcbnew)

    print('0. the simulation drops the record, as KiCad 10.0.7+ does')
    src = seeded_copy(PROJECTLESS, os.path.join(td, 'sim.kicad_pcb'))
    b = pcbnew.LoadBoard(src)
    check(T.live_board_namespace(b) == SEED, 'pcbnew loads the record onto the board')
    plain = os.path.join(td, 'plain.kicad_pcb')
    pcbnew.SaveBoard(plain, b, aSkipSettings=True)
    check(T.board_namespace(plain) is None and T.live_board_namespace(b) is None,
          'a bare SaveBoard loses it from the file and the board',
          str((T.board_namespace(plain), T.live_board_namespace(b))))

    print('4. run_plan: the saved board and its snapshot')
    board = seeded_copy(PROJECTLESS, os.path.join(td, 'plan.kicad_pcb'))
    pd = parse_kicad_pcb(board)
    net = sorted(n.name for nid, n in pd.nets.items()
                 if n.name and len(pd.pads_by_net.get(nid, ())) == 2)[0]
    snaps = os.path.join(td, 'snaps')
    import headless_plan
    steps = [{'action': 'route', 'nets': [net],
              'params': dict(clearance=0.15, track_width=0.15, via_size=0.45,
                             via_drill=0.2)}]
    res = headless_plan.run_plan(board, steps, snapshot_dir=snaps, timeout=900)
    check(res.get('completed') == 1 and not res.get('timed_out'),
          f'the one-step plan ran (route {net})', json.dumps(res.get('steps')))
    saved = sorted(SAVED)
    snap = [s for s in saved if s.startswith(os.path.abspath(snaps))]
    check(os.path.abspath(board) in SAVED and len(snap) == 1,
          'run_plan saved the board and one snapshot through pcbnew.SaveBoard', str(saved))
    for path in [os.path.abspath(board)] + snap:
        held = SAVED.get(path)
        check(bool(held) and revision_at_least(held, SEED),
              f'{os.path.basename(path)}: the live board held a record at the save',
              str(held))
        check(T.board_namespace(path) == held,
              f'{os.path.basename(path)}: the saved file carries exactly that record',
              str(T.board_namespace(path)))

    print('5. ui_thread.save_board_on_ui_thread')
    import ui_thread
    src = seeded_copy(PROJECTLESS, os.path.join(td, 'ui.kicad_pcb'))
    b = pcbnew.LoadBoard(src)
    dst = os.path.join(td, 'ui_saved.kicad_pcb')
    ok = ui_thread.save_board_on_ui_thread(dst, b)
    check(ok and T.board_namespace(dst) == SEED and T.live_board_namespace(b) == SEED,
          'the file and the live board keep the record',
          str((ok, T.board_namespace(dst), T.live_board_namespace(b))))

    print('6. the pcbnew refill script (the filled deliverable)')
    import kicad_exact_fill
    src = seeded_copy(PROJECTLESS, os.path.join(td, 'fill.kicad_pcb'))
    dst = os.path.join(td, 'fill_out.kicad_pcb')
    script = os.path.join(td, 'refill_1007.py')
    with open(script, 'w', encoding='utf-8') as f:
        # the script as shipped, under a 10.0.7+ SaveBoard
        f.write('import sys, pcbnew\n'
                f'sys.path.insert(0, {os.path.join(REPO, "tests")!r})\n'
                f'sys.path.insert(0, {os.path.join(REPO, "py_router")!r})\n'
                'import test_1906_records_survive_tool_saves as t\n'
                't.simulate_1007(pcbnew)\n'
                + kicad_exact_fill._REFILL_SCRIPT)
    r = subprocess.run([sys.executable, script, src, dst, os.path.join(REPO, 'py_router')],
                       capture_output=True, text=True, timeout=600)
    check('REFILL_OK' in r.stdout and T.board_namespace(dst) == SEED,
          'the filled board carries the source board\'s record',
          f'{T.board_namespace(dst)} rc={r.returncode} {r.stdout[-300:]} {r.stderr[-300:]}')
    pcbnew.SaveBoard = real


def revision_at_least(ns, seed):
    return T.revision(ns) >= T.revision(seed)


def _kicad_python():
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
        return sys.executable
    except Exception:
        pass
    from kicad_exact_fill import kicad_python_candidates
    for cand in kicad_python_candidates():
        if cand and cand != sys.executable and os.path.exists(cand) and subprocess.run(
                [cand, '-c', 'import pcbnew, wx'], capture_output=True).returncode == 0:
            return cand
    return None


def main():
    td = tempfile.mkdtemp(prefix='krt1906_')
    try:
        if '--kicad-half' in sys.argv:
            part2(td)
        else:
            part1(td)
            if _fails:
                print(f'FAIL: {len(_fails)}: ' + '; '.join(_fails))
                return 1
            kpy = _kicad_python()
            if kpy is None:
                print('PART 1 PASS')
                print('SKIP: part 2 needs KiCad\'s python (pcbnew + wx) -- the real '
                      'saves under a simulated KiCad 10.0.7+ were not checked')
                return 77
            if kpy == sys.executable:
                part2(td)
            else:
                rc = subprocess.run([kpy, os.path.abspath(__file__), '--kicad-half']).returncode
                if rc:
                    _fails.append(f'part 2 (KiCad python) exit {rc}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if _fails:
        print(f'FAIL: {len(_fails)}: ' + '; '.join(_fails))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
