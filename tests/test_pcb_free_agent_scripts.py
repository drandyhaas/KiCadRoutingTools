#!/usr/bin/env python3
"""pcb-free-agent's scripts, each held to a positive AND a negative control.

The skill's contract rests on three scripts that measure a run from outside
the agent: `grade.py` (the independent grade), `measure.py` (where the wall
clock went, and whether a forbidden entry point was used) and
`make_unplaced.py` (the from-scratch input). A grader that cannot fail, or a
flagger that flags nothing, reports every run as fine, so every check below
has a case that must trip it.

Two traps these pin were measured while the harness was built (runs 33/34):

* `board_score.py` sits in py_tools/, but a path-name match on the retired
  skill folder once flagged the ALLOWED grader as forbidden. Only the drivers
  and non-`record` converge verbs may be flagged.
* A BOM on a transcript's first line made that line fail to parse and it was
  DROPPED silently, taking a forbidden call with it. Unparsed lines are now
  counted, and a BOM parses.
"""
import json
import os
import re
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import run_utils                                              # noqa: E402

SKILL = os.path.join(ROOT, '.claude', 'skills', 'pcb-free-agent')
SCRIPTS = os.path.join(SKILL, 'scripts')
sys.path.insert(0, SCRIPTS)
import measure                                                # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']


def _row(ts, mid, *uses, results=()):
    content = [{'type': 'tool_use', 'id': u[0], 'name': u[1], 'input': u[2]}
               for u in uses]
    content += [{'type': 'tool_result', 'tool_use_id': r} for r in results]
    return {'type': 'assistant' if uses else 'user', 'timestamp': ts,
            'message': {'id': mid, 'usage': {'output_tokens': 5},
                        'content': content}}


def _transcript(td, rows, bom=False):
    p = os.path.join(td, 'session.jsonl')
    with open(p, 'w', encoding='utf-8-sig' if bom else 'utf-8') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')
    return p


# ------------------------------------------------------------------ measure

def test_measure_flags_only_the_retired_drivers_and_counts_converge_verbs():
    with tempfile.TemporaryDirectory() as td:
        p = _transcript(td, [
            _row('2026-09-27T06:00:00Z', 'm1',
                 ('a', 'Bash', {'command': 'python3 -X utf8 py_placer/converge.py record --ledger l --board b'})),
            _row('2026-09-27T06:01:00Z', 'm2',
                 ('b', 'Bash', {'command': 'python3 py_placer/converge.py verdict --ledger l; python3 py_placer/converge.py step-back --best --out o'}),
                 ('c', 'Bash', {'command': 'python3 .claude/skills/plan-pcb-placement-and-routing/scripts/loop_driver.py --stage L1'})),
            _row('2026-09-27T06:02:00Z', 'm3',
                 ('d', 'Bash', {'command': 'python3 -X utf8 py_tools/board_score.py b.kicad_pcb --json s.json'}),
                 # a path after the script name is not a verb
                 ('e', 'Bash', {'command': 'grep -n record py_placer/converge.py py_tools/board_score.py'})),
        ], bom=True)
        out = measure.measure(p)['main']
        hits = sorted(f['hit'] for f in out['forbidden_uses'])
        assert hits == ['loop_driver.py'], hits
        assert out['converge_verbs'] == {'record': 1, 'verdict': 1,
                                         'step-back': 1}, out['converge_verbs']
        assert out['converge_record_calls'] == 1, out
        assert out['unparsed_lines'] == 0, 'the BOM line was dropped'
        assert out['repo_scripts'].get('board_score.py') == 2, out
    print("  PASS: only the drivers are flagged; converge verbs counted, paths are not verbs")


def test_measure_splits_waiting_from_work_and_counts_bad_lines():
    with tempfile.TemporaryDirectory() as td:
        rows = [
            _row('2026-09-27T06:00:00Z', 'm1',
                 ('w', 'Bash', {'command': 'until grep -q exit r.log; do sleep 20; done'})),
            _row('2026-09-27T06:10:00Z', 'u1', results=['w']),
            _row('2026-09-27T06:10:10Z', 'm2',
                 ('k', 'Bash', {'command': 'python3 -X utf8 py_router/route.py a.kicad_pcb b.kicad_pcb'})),
            _row('2026-09-27T06:11:10Z', 'u2', results=['k']),
            # the same message id again: its usage must be counted once
            {'type': 'assistant', 'timestamp': '2026-09-27T06:11:20Z',
             'message': {'id': 'm2', 'usage': {'output_tokens': 5},
                         'content': [{'type': 'text', 'text': 'done'}]}},
        ]
        p = _transcript(td, rows)
        with open(p, 'a', encoding='utf-8') as fh:
            fh.write('{"torn": \n')
        out = measure.measure(p)['main']
        assert out['waiting_on_jobs_seconds'] == 600.0, out
        assert out['other_tool_seconds'] == 60.0, out
        assert out['model_seconds'] == 20.0, out     # 06:10->06:10:10, 06:11:10->:20
        assert out['unparsed_lines'] == 1, out
        assert out['tokens'].get('output_tokens') == 10, out   # m1 + m2, not m2 twice
    print("  PASS: 10 min of polling reads as waiting, 1 min as work; torn line counted")


def test_measure_counts_idling_on_a_background_job_as_waiting():
    """The first cut counted only foreground polls as waiting, so an agent
    idle on its own background job read as model time: run 33 reported 211
    min of model time where the model spent 19 and waited 193."""
    with tempfile.TemporaryDirectory() as td:
        notif = {'type': 'user', 'timestamp': '2026-09-27T06:30:01Z',
                 'message': {'content': '<task-notification> <task-id>x</task-id> '
                                        'completed </task-notification>'}}
        rows = [
            _row('2026-09-27T06:00:00Z', 'm1',
                 ('bg', 'Bash', {'command': 'python3 search.py', 'run_in_background': True})),
            _row('2026-09-27T06:00:01Z', 'u1', results=['bg']),   # launch returns at once
            notif,                                                # 30 min idle, then this
            {'type': 'assistant', 'timestamp': '2026-09-27T06:30:31Z',   # the model
             'message': {'id': 'm2', 'content': [{'type': 'text', 'text': 'ok'}]}},  # answers: 30 s
        ]
        out = measure.measure(_transcript(td, rows))['main']
        assert out['waiting_on_jobs_seconds'] == 1800.0, out
        assert out['model_seconds'] == 30.0, out
        assert out['other_tool_seconds'] == 1.0, out
    print("  PASS: 30 min idle until a background job reports back reads as waiting")


def test_measure_cli_exits_1_on_a_forbidden_use_and_0_without():
    with tempfile.TemporaryDirectory() as td:
        bad = _transcript(td, [_row('2026-09-27T06:00:00Z', 'm1',
                                    ('a', 'Bash', {'command': 'python3 x/placement_driver.py --stage P1'}))])
        r = run_utils.check(PY + [os.path.join(SCRIPTS, 'measure.py'), bad],
                            code=1, refuse='placement_driver.py')
        good = os.path.join(td, 'good.jsonl')
        with open(good, 'w', encoding='utf-8') as fh:
            fh.write(json.dumps(_row('2026-09-27T06:00:00Z', 'm1',
                                     ('a', 'Bash', {'command': 'ls'}))) + '\n')
        run_utils.check(PY + [os.path.join(SCRIPTS, 'measure.py'), good],
                        accept=True)
        assert r.returncode == 1
    print("  PASS: exit 1 names the forbidden verb; a clean session exits 0")


# ------------------------------------------------------------ make_unplaced

def _lock(src, dst, ref):
    """Copy `src` to `dst` with footprint `ref` KiCad-locked."""
    from kicad_parser import iter_footprint_blocks
    with open(src, encoding='utf-8', newline='') as fh:
        text = fh.read()
    for start, end, fp_text, _raw, key in iter_footprint_blocks(text):
        if key == ref:
            nl = fp_text.index('\n')
            new = fp_text[:nl + 1] + '\t\t(locked yes)\n' + fp_text[nl + 1:]
            text = text[:start] + new + text[end:]
            break
    else:
        raise AssertionError(f'{ref} not found in {src}')
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)


def test_make_unplaced_moves_and_unlocks_by_default_and_keeps_on_request():
    from kicad_parser import parse_kicad_pcb
    run_utils.evidence(ESP)
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, 'esp.kicad_pcb')
        _lock(ESP, src, 'USB1')
        assert parse_kicad_pcb(src).footprints['USB1'].locked, 'control: lock not set'
        orig = parse_kicad_pcb(src)
        bx = orig.board_info.board_bounds

        out = os.path.join(td, 'pile.kicad_pcb')
        r = run_utils.check(PY + [os.path.join(SCRIPTS, 'make_unplaced.py'), src, out],
                            accept=True)
        assert 'unplaced=True' in r.stdout, r.stdout[-600:]
        pile = parse_kicad_pcb(out)
        usb = pile.footprints['USB1']
        assert not usb.locked, 'the moved lock was left in place'
        assert usb.y > bx[3], 'USB1 is still on the board'
        assert all(f.y > bx[3] for f in pile.footprints.values() if f.pads)

        kept = os.path.join(td, 'kept.kicad_pcb')
        run_utils.check(PY + [os.path.join(SCRIPTS, 'make_unplaced.py'), src, kept,
                              '--keep-locked'], accept=True)
        k = parse_kicad_pcb(kept).footprints['USB1']
        o = orig.footprints['USB1']
        assert k.locked and (round(k.x, 4), round(k.y, 4)) == (round(o.x, 4), round(o.y, 4))
    print("  PASS: default piles + unlocks USB1; --keep-locked leaves it locked in place")


def test_make_unplaced_refuses_a_routed_input_before_writing():
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
        run_utils.evidence(src)
        out = os.path.join(td, 'pile.kicad_pcb')
        run_utils.check(PY + [os.path.join(SCRIPTS, 'make_unplaced.py'), src, out],
                        refuse='strip_copper_only.py', code=3)
        assert not os.path.exists(out), 'a refused input still wrote a board'
    print("  PASS: a routed input refuses (exit 3), names the strip tool, writes nothing")


# -------------------------------------------------------------------- grade

def test_grade_route_mode_names_every_moved_part_and_is_not_done():
    run_utils.evidence(ESP)
    with tempfile.TemporaryDirectory() as td:
        pile = os.path.join(td, 'pile.kicad_pcb')
        run_utils.check(PY + [os.path.join(SCRIPTS, 'make_unplaced.py'), ESP, pile],
                        accept=True)
        r = run_utils.check(PY + [os.path.join(SCRIPTS, 'grade.py'), pile,
                                  '--baseline', ESP, '--mode', 'route',
                                  '--spec', 'min-via-diameter=0.6',
                                  '--out-dir', td], code=4, refuse='moved_parts')
        doc = json.loads(r.stdout.splitlines()[0])
        assert doc['done'] is False, doc
        assert len(doc['moved_parts']) == 18 and 'U1' in doc['moved_parts'], doc
        with open(os.path.join(td, 'grade_pile.json'), encoding='utf-8') as fh:
            full = json.load(fh)
        # render_placement's own metric keys, read by name (a misspelt key
        # used to leave place mode's hpwl tie-break out of every grade)
        assert isinstance(full.get('hpwl'), (int, float)), sorted(full)
        assert isinstance(full.get('crossings'), int), sorted(full)
        # the declared spec reached the graders, and is on the record
        assert full['spec'] == [['--min-via-diameter', '0.6']], full['spec']
        # #1183: check_assembly graded with the input as --baseline, so its
        # courtyard gate was ARMED, and the record says so
        assert full['assembly_gating_basis'] == 'moved-vs-baseline', \
            full.get('assembly_gating_basis')
        run_utils.check(PY + [os.path.join(SCRIPTS, 'grade.py'), pile,
                              '--baseline', ESP, '--spec', 'no-equals-sign'],
                        refuse='expected NAME=VALUE', code=2,
                        allow=('usage:',))
        # control: a byte copy of the input moves nothing, through the same
        # pose reader the route-mode check uses
        import shutil
        import grade
        same = os.path.join(td, 'same.kicad_pcb')
        shutil.copyfile(ESP, same)
        before, after = grade._poses(ESP), grade._poses(same)
        assert len(before) == 18
        assert sorted(k for k in before if after.get(k) != before[k]) == []
    print("  PASS: route mode lists all 18 moved parts and is not DONE; a copy moves none")


def test_grade_refuses_a_missing_board_or_intent():
    with tempfile.TemporaryDirectory() as td:
        run_utils.check(PY + [os.path.join(SCRIPTS, 'grade.py'),
                              os.path.join(td, 'nope.kicad_pcb'), '--baseline', ESP],
                        refuse='is not a real non-empty file', code=1)
        # a RELATIVE intent resolves against the caller's directory, not the
        # repo root the checkers run in; a missing one refuses up front
        run_utils.check(PY + [os.path.join(SCRIPTS, 'grade.py'), ESP,
                              '--baseline', ESP, '--intent', 'no_such_intent.json'],
                        refuse='does not exist', code=1, cwd=td)
    print("  PASS: a missing board or intent refuses rather than grading nothing")


# ----------------------------------------------------------------- the skill

def test_the_skill_names_every_script_it_ships_and_ships_every_script_it_names():
    with open(os.path.join(SKILL, 'SKILL.md'), encoding='utf-8') as fh:
        text = fh.read()
    named = set(re.findall(r'pcb-free-agent/scripts/(\w+\.py)', text))
    shipped = {n for n in os.listdir(SCRIPTS) if n.endswith('.py')}
    assert named == shipped, (named, shipped)
    assert os.path.isfile(os.path.join(SKILL, 'references', 'verifier.md'))
    assert '/pcb-free-agent' in text and all(m in text for m in ('`full`', '`place`', '`route`'))
    print(f"  PASS: SKILL.md and scripts/ agree on {sorted(shipped)}")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
