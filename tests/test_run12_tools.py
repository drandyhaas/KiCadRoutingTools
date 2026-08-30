#!/usr/bin/env python3
"""Run-12 tool fixes: the fence, the routing banner, and two disclosures.

Five findings, each pinned by the behaviour it changed rather than by the code
that changed:

  Tier 0  The perturbation rig wrote its CONTROL -- the human placement,
          pose for pose -- into the directory the run then works in, and
          `fence_audit` exempted that file BY NAME. The same bytes were a LEAK
          or CLEAN depending only on the filename, which contradicts the
          auditor's own docstring ("this audit ignores names"). Now: a control
          inside the work dir is a leak whatever it is called, and
          `perturb(control_out=...)` puts the control AND the record (it embeds
          `original_poses`) outside the fence in one call.

  1       Every routing tool publishes exactly ONE JSON_SUMMARY, and never a
          second contradicting `incomplete` line after it (run 11). The
          `--deadline` budgets these tools once carried are GONE: no result may
          depend on a wall clock, and the only cancel in the system is the
          GUI's own Cancel button, which reaches the engines' cooperative
          `cancel_check`.

  2       The CMD:/EXIT= banner: exactly one of each per log. `cli_banner`
          promises an `EXIT=` that an EXTERNAL kill never delivers -- a gap the
          harness doing the killing owns, since the tools have no self-budget
          to fall back on.

  3       No grader said anything when a board declared NO floor at all
          (tigard ships no .kicad_pro), so a whole baseline could be measured
          against a fallback with nothing in the transcript recording it.

  6       `check_channels` computed `deficit_finest_grid` per face and surfaced
          it nowhere a caller could read; the gate counts only STARVED faces
          (supply == 0 AND demand >= --min-demand), which structurally cannot
          see a `demand 3 / supply 1` face -- the shape that bounded run 11.
          Reported now, and DELIBERATELY not gated.

Run: python3 -X utf8 tests/test_run12_tools.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
# #522 reorg + skill merge: the engine moved to py_router/, the placer to
# py_placer/, and board_score.py into the placement-and-routing skill. Tests
# that shell out to or import them need those roots on sys.path.
for _p in ('py_router', 'py_placer',
           os.path.join('.claude', 'skills', 'plan-pcb-placement-and-routing',
                        'scripts')):
    _d = os.path.join(ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
DIFF_BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
# tigard is the measured case for finding 3: it ships NO .kicad_pro, so every
# floor accessor answers None.
NO_FLOOR_BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
AUDIT = os.path.join(ROOT, 'tests', 'stress', 'fence_audit.py')


_failures = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {str(detail)[:900]}' if not cond and detail else ''))
    if not cond:
        _failures.append(name)


def run(argv, timeout=900, env=None):
    e = dict(os.environ)
    if env:
        e.update(env)
    p = subprocess.run([sys.executable, '-X', 'utf8'] + argv, cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=timeout, env=e)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def summaries(log):
    """Every JSON_SUMMARY dict in a log, in order."""
    from route_summary import SUMMARY_RE
    return [json.loads(s) for s in SUMMARY_RE.findall(log)]


# --------------------------------------------------------------------------
# Tier 0 -- the fence
# --------------------------------------------------------------------------
def test_fence(tmp):
    print('Tier 0: ground truth leaves the work dir, and a leak is content')
    import placement.perturb as P

    wd = os.path.join(tmp, 'subject')
    truth = os.path.join(tmp, '_truth', 'subject')
    os.makedirs(wd)
    out = os.path.join(wd, 'board.kicad_pcb')
    ctl = os.path.join(truth, 'control.kicad_pcb')
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rec = P.perturb(BOARD, out, kind='translate', dose_mm=8.0, seed=3,
                        control_out=ctl)
    said = buf.getvalue()

    # What matters is that GROUND TRUTH is not in the work dir -- NOT that the
    # work dir holds exactly one file. Pinning the listing conflated the two
    # and broke on machines that are merely used: copy_board deliberately
    # carries every sibling of the source board (a board without its
    # .kicad_pro loses the DRC floor, #441), and `kicad_files/*.kicad_prl` is
    # gitignored, so whether a sibling exists at all depends on whether anyone
    # has opened this board in KiCad here. Green on a clean checkout, red on a
    # developer's, for a reason the fence has nothing to do with.
    truth_in_wd = [f for f in os.listdir(wd)
                   if 'control' in f.lower() or f.endswith('.perturb.json')]
    check('control_out puts the control outside the work dir',
          os.path.isfile(ctl) and not truth_in_wd,
          f'leaked {truth_in_wd} of {sorted(os.listdir(wd))}')
    # The record carries `original_poses`, so fencing the board while leaving
    # the record behind would move the leak rather than close it.
    check('...and the record follows it',
          os.path.isfile(os.path.join(truth, 'board.perturb.json')),
          sorted(os.listdir(truth)))
    check('the record names the fenced control',
          os.path.abspath(rec['control_board']) == os.path.abspath(ctl),
          rec['control_board'])
    check('perturb SAYS where truth went (a caller cannot fence what it '
          'cannot see)',
          'CONTROL (GROUND TRUTH)' in said and 'RECORD (GROUND TRUTH)' in said,
          said[-400:])

    def audit(mode, workdir):
        return run([AUDIT, '--control', ctl, '--workdir', workdir,
                    '--mode', mode])

    rc, _ = audit('create', wd)
    check('a fenced work dir is CLEAN', rc == 0)

    # THE FINDING: the historic default path, under the historic name.
    os.remove(os.path.join(wd, '.fence-manifest.json'))
    leaked = os.path.join(wd, 'board.control.kicad_pcb')
    shutil.copy(ctl, leaked)
    rc, out_txt = audit('create', wd)
    check('a control INSIDE the work dir is a LEAK, under its own name',
          rc == 4, out_txt)
    check('...and the report names the file',
          'board.control.kicad_pcb' in out_txt, out_txt)
    os.remove(leaked)

    # The compatibility default still works -- and warns.
    wd2 = os.path.join(tmp, 'compat')
    os.makedirs(wd2)
    buf2 = io.StringIO()
    with contextlib.redirect_stdout(buf2):
        P.perturb(BOARD, os.path.join(wd2, 'p.kicad_pcb'), kind='translate',
                  dose_mm=6.0, seed=5)
    warned = buf2.getvalue()
    check('the unsafe default still writes beside the output (compatibility)',
          os.path.isfile(os.path.join(wd2, 'p.control.kicad_pcb'))
          and os.path.isfile(os.path.join(wd2, 'p.perturb.json')),
          sorted(os.listdir(wd2)))
    check('...but says so loudly', 'WARNING' in warned and 'LEAK' in warned,
          warned[-400:])


# --------------------------------------------------------------------------
# 1 + 2 -- one summary per run, and the banner
# --------------------------------------------------------------------------
ROUTING_TOOLS = (
    # (label, argv-builder, needs a board with diff pairs)
    ('route.py', lambda o: ['py_router/route.py', BOARD, o, '--nets', '*',
                            '--clearance', '0.1']),
    ('route_diff.py', lambda o: ['py_router/route_diff.py', DIFF_BOARD, o, '--nets', '*',
                                 '--clearance', '0.15']),
    ('route_planes.py', lambda o: ['py_router/route_planes.py', BOARD, o, '--nets', 'GND',
                                   '--plane-layers', 'B.Cu',
                                   '--clearance', '0.1']),
)


def test_banner_and_one_complete_summary(tmp):
    """Every routing tool says how it was invoked, and reports exactly once.

    The second half is a measured defect, not a hypothetical: it shipped in
    route_disconnected_planes (run 11), where every successful run printed
    `complete: true` and then a contradicting
    `{"complete": false, "status": "incomplete"}` line, and any consumer keying
    on the LAST JSON_SUMMARY read a good run as a failed one.

    NOTE these tools have no self-budget: no result they produce may depend on
    a wall clock, so there is no in-tool cancel for a CLI run to trip. An
    EXTERNAL kill still skips `atexit` and loses the EXIT= line -- a gap the
    harness that does the killing owns.
    """
    print('\n1+2: every routing tool says how it was invoked, and reports once')
    for label, argv in ROUTING_TOOLS:
        out = os.path.join(tmp, label.replace('.py', '') + '_run.kicad_pcb')
        rc, log = run(argv(out))
        subs = summaries(log)

        check(f'{label}: exit 0, at least one summary, none of them partial',
              rc == 0 and len(subs) >= 1
              and not any(s.get('status') in ('deadline', 'incomplete')
                          for s in subs)
              and '"incomplete"' not in log and 'DEADLINE:' not in log,
              f'rc={rc} n={len(subs)} '
              + str([s.get('status') for s in subs]))

        check(f'{label}: exactly one CMD: and one EXIT= line',
              log.count('\nCMD: ') + log.startswith('CMD: ') == 1
              and sum(1 for ln in log.splitlines()
                      if ln.startswith('EXIT=')) == 1,
              log[:200])
        check(f'{label}: EXIT= reports the tool\'s own code',
              'EXIT=0' in log, log[-300:])


def test_merge_keeps_incompleteness(tmp):
    """route.py can print TWO summaries (the reconciliation self-invoke). If
    either is partial, the merged tally a consumer actually reads must stay
    partial -- last-wins would otherwise let a complete second pass erase the
    first's disclosure.

    Driven from a SYNTHETIC log: no tool emits `complete: false` today (none of
    them can stop itself on a clock), but the stickiness is the guard that
    makes a partial safe if one ever appears, so it is tested directly rather
    than deleted along with the mechanism that used to produce one.
    """
    print('\n1: the merged tally stays incomplete')
    from route_summary import merge_route_summaries
    log = ('JSON_SUMMARY: ' + json.dumps(
               {'nets_routed': 3, 'complete': False, 'status': 'partial',
                'stopped_in': 'routing'}) + '\n'
           'Note: the line above is the whole-board pass\n'
           'JSON_SUMMARY: ' + json.dumps(
               {'nets_routed': 1, 'complete': True, 'status': 'ok'}) + '\n')
    m = merge_route_summaries(log)
    check('merge_route_summaries reports the run as partial',
          m is not None and m.get('complete') is False
          and m.get('status') == 'partial',
          m and {k: m.get(k) for k in ('complete', 'status', 'stopped_in')})


# --------------------------------------------------------------------------
# 3 -- a board that declares no floor at all
# --------------------------------------------------------------------------
def test_undeclared_floor(tmp):
    print('\n3: the graders name the fallback on a board that declares nothing')
    from list_nets import board_floor_declaration

    d = board_floor_declaration(NO_FLOOR_BOARD)
    check('the accessor sees an undeclared board', d['declares_nothing'], d)
    # A TRACKED declaring board (flat_hierarchy ships its .kicad_pro with a
    # Default class): interf_u_routed is a gitignored generated fixture, so a
    # fresh clone does not have it (tests/fixture_boards.py deliberately does
    # not build the routed interf_u chain -- "use a tracked board instead").
    d2 = board_floor_declaration(os.path.join(ROOT, 'kicad_files',
                                              'flat_hierarchy.kicad_pcb'))
    check('...and does not cry wolf on one that declares', not d2['declares_nothing'], d2)

    # check_drc, on BOTH branches. The pre-existing warning fired only when -c
    # was omitted -- yet CLAUDE.md tells a caller to pass the routed clearance,
    # which is exactly the case that went unrecorded.
    drc_json = os.path.join(tmp, 'drc.json')
    for label, extra in (('with -c', ['--clearance', '0.09']), ('auto', [])):
        rc, log = run(['py_router/check_drc.py', NO_FLOOR_BOARD, '--max-print', '0',
                       '--json', drc_json] + extra)
        check(f'check_drc ({label}) says the floor is a fallback',
              'declares NO net class and NO board constraint' in log,
              log[:900])
    doc = json.load(open(drc_json, encoding='utf-8'))
    check('check_drc records it in graded_at',
          doc['graded_at'].get('board_declares_no_floor') is True,
          doc['graded_at'])

    asm_json = os.path.join(tmp, 'asm.json')
    rc, log = run(['py_tools/check_assembly.py', NO_FLOOR_BOARD, '--json', asm_json])
    check('check_assembly says the floor is a fallback',
          'declares NO net class and NO board constraint' in log, log[:900])
    check('...and records it in its JSON',
          json.load(open(asm_json, encoding='utf-8'))
          .get('board_declares_no_floor') is True)

    score = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing',
                         'scripts', 'board_score.py')
    sc_json = os.path.join(tmp, 'score.json')
    rc, log = run([score, NO_FLOOR_BOARD, '--json', sc_json])
    check('board_score says it once, at the top level',
          'NO DECLARED FLOOR' in log, log[:900])
    doc = json.load(open(sc_json, encoding='utf-8'))
    check('...and records it in floors',
          doc['floors'].get('declares_no_floor') is True, doc['floors'])

    # No exit-code change: this is disclosure, not a gate.
    rc_quiet, log_quiet = run([score, NO_FLOOR_BOARD])
    check('disclosure does not change board_score\'s exit code',
          rc_quiet == rc, f'{rc_quiet} vs {rc}')

    # ...and a board that DOES declare must stay silent, or the line becomes
    # noise everyone learns to skip.
    rc, log = run(['py_router/check_drc.py',
                   os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb'),
                   '--max-print', '0'])
    check('a board that declares a floor gets no such note',
          'declares NO net class' not in log, log[:400])


# --------------------------------------------------------------------------
# 6 -- the deficit report check_channels never surfaced
# --------------------------------------------------------------------------
def test_deficit_faces(tmp):
    print('\n6: check_channels surfaces the absolute deficit -- and does not '
          'gate it')
    js = os.path.join(tmp, 'chan.json')
    rc, log = run(['py_tools/check_channels.py', NO_FLOOR_BOARD, '--clearance', '0.09',
                   '--json', js])
    check('check_channels still exits 0 (report-only)', rc == 0, log[-400:])
    doc = json.load(open(js, encoding='utf-8'))
    check('deficit_faces is in the JSON', 'deficit_faces' in doc,
          sorted(doc))
    dfs = doc['deficit_faces']
    check('...and is non-empty on a board with a deficit face', bool(dfs), dfs)
    check('...worst-first', [e['deficit_finest_grid'] for e in dfs]
          == sorted((e['deficit_finest_grid'] for e in dfs), reverse=True), dfs)
    check('one summary line names the worst face',
          'Faces in DEFICIT at the finest legal grid' in log, log[-900:])

    # THE POINT. The gate's predicate is `supply == 0 AND demand >=
    # --min-demand`; run 11's bounding face was `demand 3 / supply 1`. A
    # deficit face with supply > 0, or with demand below the gate floor, must
    # appear here and NOT in starved_faces.
    starved = {(r[0], r[1]) for r in doc['starved_faces']}
    invisible = [e for e in dfs if (e['ref'], e['face']) not in starved]
    check('a face in deficit that the STARVATION gate cannot see is reported',
          bool(invisible),
          f'deficit={[(e["ref"], e["face"], e["demand_nets"], e["supply_finest_grid"]) for e in dfs]} '
          f'starved={sorted(starved)}')
    check('...and the gate itself is unchanged (still 0 without a baseline)',
          run(['py_tools/check_channels.py', NO_FLOOR_BOARD, '--clearance', '0.09',
               '--gate'])[0] == 0)


def test_stacked_suspect(tmp):
    """A co-located PAIR is a pile or a design; the warning must tell them apart.

    The stacked-footprint warning blamed a netlist re-import for front/back
    fiducial pairs -- parts that share a coordinate BY DESIGN. Three ways to get
    this wrong, and all three were live at some point:

      * suppress on "every one is a marker" -- but tested group-wide, so one
        back-side part exonerated an arbitrarily large front-side pile;
      * suppress on "every one is locked" -- but this toolchain STAMPS its own
        locks (seeder.stamp_locked, on place_seed's output), so a pile the
        tools created would be invisible to the check meant to catch piles;
      * suppress on a marker test that never ran (it read `.kind` off a
        namedtuple whose field is `.name`, into a bare except).

    Silence is the failure mode here, so every case below asserts the DIRECTION,
    not just that something was reported.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.placement_state import assess_placement
    from placement.part_class import classify_part

    MARKERS = ('fiducial', 'mount_hole', 'testpoint')

    def board():
        return parse_kicad_pcb(BOARD)

    def pick(pcb, layer, n, marker=False):
        out = []
        for ref, fp in pcb.footprints.items():
            if (fp.layer or '') != layer or fp.locked or not fp.pads:
                continue
            if (classify_part(fp, ref).name in MARKERS) != marker:
                continue
            out.append(ref)
            if len(out) == n:
                break
        return out

    def stack(pcb, refs, at=(40.0, 40.0)):
        for r in refs:
            pcb.footprints[r].x, pcb.footprints[r].y = at

    st = assess_placement(board(), 'x')
    check('a healthy board reports no stacked suspects',
          not st.stacked_suspect_refs and not st.partially_unplaced,
          f'suspect={sorted(st.stacked_suspect_refs)}')
    check('...and suspects are always a subset of stacked_refs',
          set(st.stacked_suspect_refs) <= set(st.stacked_refs))

    # THE HEADLINE CHANGE, on the board where the two formulas DISAGREE.
    # `partially_unplaced` used to key on any duplicate position at all; it now
    # keys on the suspect subset. Every synthetic case above stacks parts that
    # ARE suspect, so `stacked` and `suspect` coincide and the old and new
    # formulas agree -- which means none of them can detect a revert. glasgow
    # is the case that separates them: three front/back FIDUCIAL PAIRS, so
    # `stacked_refs` is non-empty (the old formula says partially unplaced,
    # which is the false alarm this whole filter exists to remove) while
    # `stacked_suspect_refs` is empty.
    gl = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
    if os.path.isfile(gl):
        st = assess_placement(parse_kicad_pcb(gl), gl)
        check('fixture: glasgow HAS co-located parts', bool(st.stacked_refs),
              f'stacked={sorted(st.stacked_refs)}')
        check('...all explicable, so partially_unplaced is False',
              not st.stacked_suspect_refs and not st.partially_unplaced,
              f'stacked={sorted(st.stacked_refs)} '
              f'suspect={sorted(st.stacked_suspect_refs)} '
              f'partially={st.partially_unplaced}')
        check('...and no reason mentions a netlist re-import',
              not any('re-import' in r for r in st.reasons), st.reasons)

    p = board()
    pair = pick(p, 'F.Cu', 2)
    stack(p, pair)
    st = assess_placement(p, 'x')
    check('two same-side non-marker parts at one spot are suspect',
          set(pair) <= set(st.stacked_suspect_refs) and st.partially_unplaced,
          f'pair={pair} suspect={sorted(st.stacked_suspect_refs)}')

    # THE SIDE-PARTITION CASE. One back-side part must not exonerate the pile.
    # It needs a board with parts on BOTH sides -- splitflap has none on the
    # back, and a fixture that quietly skips is the failure mode this whole
    # function is about, so the precondition is asserted rather than guarded.
    p = parse_kicad_pcb(NO_FLOOR_BOARD)          # tigard: JP1/JP2 on B.Cu
    front = pick(p, 'F.Cu', 3)
    back = [r for r, f in p.footprints.items()
            if (f.layer or '') == 'B.Cu' and f.pads][:1]
    check('fixture: the side-partition case has parts on both sides',
          len(front) == 3 and len(back) == 1, f'front={front} back={back}')
    stack(p, front + back, at=(50.0, 50.0))
    st = assess_placement(p, 'x')
    check('one back-side part does not exonerate a front-side pile',
          set(front) <= set(st.stacked_suspect_refs),
          f'front={front} back={back} '
          f'suspect={sorted(st.stacked_suspect_refs)}')

    # THE DRILLED CASE, and the one corpus row it moves. A through-hole part
    # occupies BOTH sides, so `fp.layer` alone is the wrong partition: tigard's
    # J2 (F.Cu, nine 1.0mm drilled pads) and JP1 (B.Cu) each ended up alone on
    # their side and both were exonerated, while `grade_pad_legality` reports
    # their copper colliding. `legality.footprint_has_through_pads` is
    # deliberately `drill > 0` -- physical obstruction, not layer-tying -- and
    # this check asks the physical question.
    p = parse_kicad_pcb(NO_FLOOR_BOARD)
    stack(p, ['J2', 'JP1'], at=(40.0, 30.0))
    st = assess_placement(p, NO_FLOOR_BOARD)
    check('a DRILLED part co-located with a far-side part is suspect',
          {'J2', 'JP1'} <= set(st.stacked_suspect_refs),
          f'suspect={sorted(st.stacked_suspect_refs)}')

    # The corpus row this moved, recorded so it stays a decision. rp2350's J2
    # (B.Cu connector, three NPTH holes) and U3 (F.Cu) share an origin exactly;
    # they were suppressed as "opposite sides" until drilled parts started
    # counting on both.
    #
    # Be precise about the corroboration, because the first version of this
    # comment overstated it: ONE other instrument agrees --
    # `grade_body_overlap` reports a J2/U3 `courtyard` pair of 6.6mm2. There is
    # NO pad conflict between them (rp2350's single pad conflict is C18/C19,
    # and J2's three NPTH holes at x 145.96/151.04 clear U3's pads at
    # 147.78-149.22), and `blocking` is 0. So this is an advisory-strength
    # agreement, which is the right strength for a co-location warning.
    #
    # Net corpus effect of the whole change is still fewer alarms, not more:
    # HEAD keyed `partially_unplaced` on any duplicate position at all and so
    # flagged all three co-located boards; this flags one. If it ever needs to
    # go quiet, change it here rather than by narrowing the through-pad test --
    # that test is what catches tigard J2/JP1, where the pad copper really does
    # collide.
    rp = os.path.join(ROOT, 'kicad_files',
                      'rp2350_fpga_eensy_prePlane.kicad_pcb')
    if os.path.isfile(rp):
        st = assess_placement(parse_kicad_pcb(rp), rp)
        check('rp2350 J2/U3 (drilled, shared origin) is reported',
              {'J2', 'U3'} <= set(st.stacked_suspect_refs),
              f'suspect={sorted(st.stacked_suspect_refs)}')

    # THE STAMPED-LOCK CASE. `locked` is a decision about WHERE a part goes; it
    # does not make two parts able to occupy one space.
    p = board()
    lockpile = pick(p, 'F.Cu', 3)
    stack(p, lockpile, at=(55.0, 55.0))
    for r in lockpile:
        p.footprints[r].locked = True
    st = assess_placement(p, 'x')
    check('a pile whose parts are all LOCKED is still a pile',
          set(lockpile) <= set(st.stacked_suspect_refs),
          f'pile={lockpile} suspect={sorted(st.stacked_suspect_refs)}')

    # ...and the case the whole filter exists for.
    p = board()
    marks = pick(p, 'F.Cu', 2, marker=True)
    check('fixture: the board has two same-side markers', len(marks) >= 2,
          f'markers={marks}')
    stack(p, marks, at=(60.0, 60.0))
    st = assess_placement(p, 'x')
    check('co-located MARKERS are not reported as a pile',
          not (set(marks) & set(st.stacked_suspect_refs)),
          f'markers={marks} suspect={sorted(st.stacked_suspect_refs)}')

    # A MIXED group is a pile. The suppression is `all(marker)`, and `any`
    # would read almost the same while meaning the opposite: one fiducial in
    # the group would excuse every real part sitting on top of it. Nothing
    # pinned the difference, so the two spellings were interchangeable.
    p = board()
    mixed = pick(p, 'F.Cu', 1, marker=True) + pick(p, 'F.Cu', 1)
    check('fixture: the mixed group is one marker and one ordinary part',
          len(mixed) == 2, f'mixed={mixed}')
    stack(p, mixed, at=(62.0, 62.0))
    st = assess_placement(p, 'x')
    check('a marker does NOT excuse a real part stacked on it',
          set(mixed) <= set(st.stacked_suspect_refs),
          f'mixed={mixed} suspect={sorted(st.stacked_suspect_refs)}')

    # The field must reach the one document that carries this state.
    intent = os.path.join(tmp, 'fp_intent.json')
    out = os.path.join(tmp, 'fp_state.json')
    run(['py_tools/check_floorplan.py', BOARD, '--emit-intent', intent])
    rc, log = run(['py_tools/check_floorplan.py', BOARD, '--intent', intent,
                   '--json', out])
    doc = json.load(open(out, encoding='utf-8')) if os.path.isfile(out) else {}
    check('check_floorplan publishes stacked_suspect_refs',
          'stacked_suspect_refs' in doc.get('state', {}),
          f'rc={rc} state keys={sorted(doc.get("state", {}))} {log[-300:]}')


def main():
    with tempfile.TemporaryDirectory(prefix='run12_') as tmp:
        test_fence(os.path.join(tmp, 'fence'))
        test_banner_and_one_complete_summary(tmp)
        test_merge_keeps_incompleteness(tmp)
        test_undeclared_floor(tmp)
        test_deficit_faces(tmp)
        test_stacked_suspect(tmp)

    print()
    if _failures:
        print(f'FAIL: {len(_failures)} check(s): {", ".join(_failures)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    os.makedirs(os.path.join(tempfile.gettempdir()), exist_ok=True)
    sys.exit(main())
