#!/usr/bin/env python3
"""#837: the per-side assembly census, and the two ways of getting it wrong.

The census answers "which faces will the fab populate", and there are exactly
two traps between that question and the numbers a board hands you:

1. **Zero-pad blocks on the back.** `esp_prog` carries three OLIMEX logo
   footprints on B.Cu with no pads at all. Counting blocks it is two-sided;
   counting copper it is not, and the fab builds the second one. A census that
   reads `fp.layer` alone reports a second reflow pass for three silkscreen
   logos.

2. **`sides_occupied` is not an assembly classifier.** It answers the
   OBSTRUCTION question -- a drilled pad puts its part on both faces -- and
   under it 17 of the 22 tracked boards read double-sided, `splitflap_driver`
   (65 parts, every one on F.Cu, 24 of them through-hole) included. A
   through-hole part on the front does not demand a second reflow pass.

Both are asserted on boards where the wrong answer is a DIFFERENT number, so
neither arm can pass by coincidence. The expected counts are hand-stated
literals: re-deriving them from `assembly_census` would make this file agree
with whatever the module currently says and stop it being a change detector.
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer')]
sys.path.insert(0, HERE)

RUN_ALL_TIMEOUT = 900

FAILURES = []


def check(name, cond, detail=''):
    if cond:
        print(f"  ok   {name}")
    else:
        FAILURES.append(f"{name}: {detail}")
        print(f"  FAIL {name}: {detail}")


def board(stem):
    return os.path.join(ROOT, 'kicad_files', f'{stem}.kicad_pcb')


#: (stem, blocks F/B, pad-bearing F/B, zero-pad F/B, THT, NPTH-only, sides,
#:  reflow passes). Measured at HEAD, hand-stated: re-deriving these from
#: `assembly_census` would make this file agree with whatever the module
#: currently says.
#:
#: Each row is here because it is the board where one rule CHANGES the answer:
#:   esp_prog, watchy  -- pad-bearing vs blocks (the verdict flips F/both)
#:   splitflap_driver  -- `sides_occupied` would call it two-sided (24 drilled)
#:   watchy            -- `drill > 0` counted 5 through-hole parts; 4 of them
#:                        are SMD switches with unplated alignment posts, so
#:                        the plated rule says 1
#:   flat_hierarchy    -- 0 SMD parts, so 0 reflow passes on a populated face,
#:                        and 6 NPTH mounting holes in neither solder bucket
#:   ulx3s             -- back-dominant: an arm assuming "single means front"
#:                        would flag 163 parts on a shipping board
EXPECT = [
    ('esp_prog',           (18, 3),   (18, 0),   (0, 3), 3,  0, 'F',    1),
    ('watchy',             (85, 1),   (84, 0),   (1, 1), 1,  0, 'F',    1),
    ('splitflap_driver',   (65, 0),   (65, 0),   (0, 0), 22, 2, 'F',    1),
    ('flat_hierarchy',     (64, 0),   (64, 0),   (0, 0), 58, 6, 'F',    0),
    ('ulx3s',              (67, 168), (63, 163), (4, 5), 11, 0, 'both', 2),
    ('glasgow_revC',       (178, 94), (172, 92), (6, 2), 17, 0, 'both', 2),
    ('tigard',             (89, 3),   (87, 2),   (2, 1), 8,  4, 'both', 2),
    ('kit-dev-coldfire-xilinx_5213',
                           (146, 14), (146, 14), (0, 0), 41, 0, 'both', 2),
]


def independent(pcb):
    """The same census by a different route, deliberately.

    Reads `fp.layer`, `fp.pads` and `pad.pad_type` directly instead of calling
    `footprint_side` / `pad_is_plated_through`. A test that calls the helpers
    the subject calls proves only that one function was invoked once.

    The through-hole predicate is spelled out here rather than imported, and
    that is the point: the first version of this file copied
    `footprint_has_through_pads`'s bare `drill > 0`, so the arm voted AGAINST
    the fix for the defect it should have caught -- watchy's four SMD switches
    with unplated alignment posts.
    """
    blk = {'F': 0, 'B': 0}
    pad = {'F': 0, 'B': 0}
    zero = {'F': [], 'B': []}
    tht = npth = 0
    for ref, fp in pcb.footprints.items():
        s = 'B' if (getattr(fp, 'layer', '') or '').startswith('B') else 'F'
        blk[s] += 1
        if not (fp.pads or ()):
            zero[s].append(ref)
            continue
        pad[s] += 1
        if any((getattr(p, 'drill', 0) or 0) > 0
               and getattr(p, 'pad_type', '') != 'np_thru_hole'
               for p in fp.pads):
            tht += 1
        elif all(getattr(p, 'pad_type', '') == 'np_thru_hole'
                 for p in fp.pads):
            npth += 1
    return blk, pad, zero, tht, npth


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.legality import (assembly_census, footprint_side,
                                    sides_occupied, footprint_has_through_pads)

    print("#837 assembly census")
    graded = 0
    for (stem, e_blk, e_pad, e_zero, e_tht, e_npth, e_sides,
         e_passes) in EXPECT:
        path = board(stem)
        if not os.path.isfile(path):
            check(f"{stem}: fixture present", False, path)
            continue
        pcb = parse_kicad_pcb(path)
        c = assembly_census(pcb)
        i_blk, i_pad, i_zero, i_tht, i_npth = independent(pcb)
        graded += 1

        check(f"{stem}: blocks {e_blk}",
              (c['blocks']['F'], c['blocks']['B']) == e_blk,
              f"got {(c['blocks']['F'], c['blocks']['B'])}")
        check(f"{stem}: pad-bearing {e_pad}",
              (c['pad_bearing']['F'], c['pad_bearing']['B']) == e_pad,
              f"got {(c['pad_bearing']['F'], c['pad_bearing']['B'])}")
        check(f"{stem}: an independent parse agrees",
              (dict(i_blk), dict(i_pad), i_tht, i_npth)
              == (dict(c['blocks']), dict(c['pad_bearing']),
                  c['through_hole'], sum(c['unsoldered'].values()))
              and {k: sorted(v) for k, v in i_zero.items()}
              == {k: sorted(v) for k, v in c['zero_pad'].items()},
              f"independent {(i_blk, i_pad, i_zero, i_tht, i_npth)} vs "
              f"census {(c['blocks'], c['pad_bearing'], c['zero_pad'], c['through_hole'], c['unsoldered'])}")
        check(f"{stem}: zero-pad F/B {e_zero}",
              (len(c['zero_pad']['F']), len(c['zero_pad']['B'])) == e_zero,
              f"got {(len(c['zero_pad']['F']), len(c['zero_pad']['B']))}: "
              f"{c['zero_pad']}")
        # Every block accounted for, so a reader can reconcile the printed
        # census against the board. Listing only the BACK left 7 of 22 boards
        # short -- interf_u's unexplained 25th block is a FRONT graphic.
        check(f"{stem}: the arithmetic closes",
              sum(c['pad_bearing'].values()) + len(c['zero_pad']['F'])
              + len(c['zero_pad']['B']) == sum(c['blocks'].values()),
              f"{c['pad_bearing']} + {c['zero_pad']} != {c['blocks']}")
        check(f"{stem}: {e_tht} plated through-hole, {e_npth} NPTH-only",
              c['through_hole'] == e_tht
              and sum(c['unsoldered'].values()) == e_npth,
              f"got tht={c['through_hole']} npth={c['unsoldered']}")
        # The three solder buckets partition the pad-bearing population. A
        # part counted in none of them (or in two) is a classification bug.
        check(f"{stem}: the solder buckets partition the parts",
              sum(c['smd'].values()) + c['through_hole']
              + sum(c['unsoldered'].values())
              == sum(c['pad_bearing'].values()),
              f"smd={c['smd']} tht={c['through_hole_by_side']} "
              f"npth={c['unsoldered']} vs {c['pad_bearing']}")
        check(f"{stem}: sides={e_sides}, {e_passes} reflow pass(es)",
              c['sides'] == e_sides and c['reflow_passes'] == e_passes,
              f"got sides={c['sides']} passes={c['reflow_passes']}")

    # --- Trap 1: the zero-pad rule must CHANGE the answer on esp_prog.
    # Without this the first arm above is satisfied by a census that never
    # looks at pads, because on 20 of 22 boards the two rules agree.
    pcb = parse_kicad_pcb(board('esp_prog'))
    naive_b = sum(1 for fp in pcb.footprints.values()
                  if footprint_side(fp) == 'B')
    check("esp_prog: the block rule and the pad rule DISAGREE",
          naive_b == 3 and assembly_census(pcb)['pad_bearing']['B'] == 0,
          f"blocks on B {naive_b}, pad-bearing B "
          f"{assembly_census(pcb)['pad_bearing']['B']}")
    check("esp_prog: so the verdict is single-sided, not double",
          assembly_census(pcb)['sides'] == 'F'
          and assembly_census(pcb)['reflow_passes'] == 1,
          str(assembly_census(pcb)))

    # --- Trap 2: `sides_occupied` would say two-sided; the census must not.
    pcb = parse_kicad_pcb(board('splitflap_driver'))
    occupied = set()
    for fp in pcb.footprints.values():
        occupied |= sides_occupied(footprint_side(fp),
                                   footprint_has_through_pads(fp))
    check("splitflap_driver: sides_occupied really does say BOTH",
          occupied == {'F', 'B'},
          f"{occupied} -- if this is not both, the arm below proves nothing")
    check("splitflap_driver: the census says F anyway",
          assembly_census(pcb)['sides'] == 'F'
          and assembly_census(pcb)['through_hole'] == 22,
          str(assembly_census(pcb)))

    # --- Trap 3: `drill > 0` is not the assembly question either. watchy's
    # SW1-SW4 are SMD switches with unplated 0.75mm alignment posts, so the
    # bare-drill rule reports 5 hand-soldered parts on a board with 1 -- and
    # then the text channel tells the reader they need wave soldering.
    pcb = parse_kicad_pcb(board('watchy'))
    naive = sum(1 for fp in pcb.footprints.values()
                if footprint_has_through_pads(fp))
    check("watchy: the drill rule and the plated rule DISAGREE",
          naive == 5 and assembly_census(pcb)['through_hole'] == 1,
          f"drill>0 says {naive}, plated says "
          f"{assembly_census(pcb)['through_hole']}")

    # --- The CLI: additive only. Run on TWO boards, because on a
    # single-sided one every distinction this feature draws collapses --
    # `blocks` == `pad_bearing`, no zero-pad blocks, sides='F' -- so a wiring
    # bug (printing `blocks` where `pad_bearing` was meant, hard-coding a
    # verdict) is invisible there. ulx3s separates all of them.
    import run_utils
    tool = os.path.join(ROOT, 'py_tools', 'check_assembly.py')
    env = dict(os.environ, KRT_NO_BANNER='1')

    def run_cli(stem, td):
        jp = os.path.join(td, f'{stem}.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool, board(stem),
                            '--json', jp],
                           capture_output=True, text=True, cwd=ROOT, env=env)
        run_utils.evidence(jp, f'the {stem} assembly JSON')
        return r, r.stdout + r.stderr, json.load(open(jp, encoding='utf-8'))

    with tempfile.TemporaryDirectory() as td:
        r, out, doc = run_cli('splitflap_driver', td)
        check("CLI: a clean single-sided board still exits 0",
              r.returncode == 0, f"exit {r.returncode}\n{out[-800:]}")
        check("CLI: the section prints on a board with nothing on the back",
              'ASSEMBLY SIDES: F 65 / B 0' in out, out[-800:])
        check("CLI: the text channel names the counting rule",
              'per footprint BLOCK' in out, out[-800:])
        for k in ('parts_by_side', 'parts_by_side_blocks',
                  'parts_by_side_basis', 'back_side_zero_pad_blocks',
                  'zero_pad_blocks_by_side', 'through_hole_parts',
                  'through_hole_by_side', 'smd_parts_by_side',
                  'unsoldered_parts_by_side', 'assembly_sides',
                  'reflow_passes'):
            check(f"CLI: --json carries {k}", k in doc, sorted(doc))
        check("CLI: the JSON basis names #726",
              '#726' in doc.get('parts_by_side_basis', ''),
              doc.get('parts_by_side_basis'))
        # The verdict channel is untouched: this is a report, not a conjunct.
        check("CLI: the verdict did not learn about sides",
              doc.get('buildable') is True
              and 'blocking' in doc and doc['blocking'] == 0,
              f"buildable={doc.get('buildable')} blocking={doc.get('blocking')}")

        # ulx3s: every published number DIFFERS from every other, so each
        # assertion below fails against a different wiring mistake.
        r2, out2, d2 = run_cli('ulx3s', td)
        check("CLI/ulx3s: exit code unmoved", r2.returncode == 0,
              f"exit {r2.returncode}")
        check("CLI/ulx3s: parts_by_side is the PAD-BEARING census",
              d2.get('parts_by_side') == {'F': 63, 'B': 163},
              d2.get('parts_by_side'))
        check("CLI/ulx3s: parts_by_side_blocks is EVERY block",
              d2.get('parts_by_side_blocks') == {'F': 67, 'B': 168},
              d2.get('parts_by_side_blocks'))
        check("CLI/ulx3s: and the two differ, or the pair proves nothing",
              d2.get('parts_by_side') != d2.get('parts_by_side_blocks'),
              str(d2.get('parts_by_side')))
        check("CLI/ulx3s: assembly_sides=both", d2.get('assembly_sides') == 'both',
              d2.get('assembly_sides'))
        check("CLI/ulx3s: reflow_passes=2", d2.get('reflow_passes') == 2,
              d2.get('reflow_passes'))
        check("CLI/ulx3s: through_hole_parts=11 (plated only)",
              d2.get('through_hole_parts') == 11, d2.get('through_hole_parts'))
        # The NAMES, not the count: the text channel prints them and a reader
        # uses them to find the blocks being excluded.
        check("CLI/ulx3s: the zero-pad blocks are named on BOTH faces",
              sorted(d2.get('zero_pad_blocks_by_side', {}).get('B', []))
              == ['D&M', 'EMARD~2', 'OSHW', 'REF**', 'koncar']
              and len(d2.get('zero_pad_blocks_by_side', {}).get('F', [])) == 4,
              d2.get('zero_pad_blocks_by_side'))
        check("CLI/ulx3s: the text prints the pad-bearing counts",
              'ASSEMBLY SIDES: F 63 / B 163' in out2, out2[-1200:])
        check("CLI/ulx3s: the text prints the reflow-pass count",
              '2 reflow pass(es)' in out2, out2[-1200:])
        check("CLI/ulx3s: the text prints the observed policy",
              'observed policy: sides=both' in out2, out2[-1200:])
        # The one-face contrast must NOT be printed on a board the census
        # already calls two-sided: there is no disagreement to point at.
        check("CLI/ulx3s: no vacuous sides_occupied contrast",
              'would call this board two-sided' not in out2, out2[-1200:])
        check("CLI/splitflap: the contrast IS printed where it differs",
              'would call this board two-sided' in out, out[-1200:])

    # --- The RULE. Armed by hand, because no emitted intent can arm it: the
    # emitter declares the OBSERVED policy, so a board always satisfies its
    # own. That is the design, and it is also why the rule needs a test that
    # does not go through the emitter.
    from placement import floorplan as fp
    from placement.floorplan import IntentError

    def graded_with(stem, sides):
        path = board(stem)
        pcb = parse_kicad_pcb(path)
        doc = fp.emit_intent(pcb, path)
        doc['assembly'] = {'sides': sides}
        return fp.grade(fp.intent_from_dict(doc), pcb, path)

    # The count is the census's off-face PAD-BEARING count, and it is NOT the
    # 94 the issue asks for: 94 is glasgow's back-side BLOCK count, and two of
    # those blocks carry no pads. Same board, two bases, and the census
    # declares which one it used.
    r = graded_with('glasgow_revC', 'F')
    v = [x for x in r.violations if x.rule == 'assembly_side']
    check("rule: glasgow declared F flags its 92 pad-bearing back parts",
          len(v) == 92, f"got {len(v)}")
    check("rule: and 92 is the pad-bearing count, not the 94 block count",
          len(v) != 94, "94 would mean the rule counted zero-pad blocks")
    check("rule: it WARNS by default, so it can never be an unclearable red "
          "mark (nothing in the engine can move a part between faces)",
          v and all(x.severity == 'warn' for x in v)
          and not [e for e in r.errors if e.rule == 'assembly_side'],
          str(sorted({x.severity for x in v})))
    # Both directions on the same board: a rule that hard-coded 'F' as the
    # populated face would pass the first arm and fail this one.
    rb = graded_with('ulx3s', 'B')
    rf = graded_with('ulx3s', 'F')
    nb = len([x for x in rb.violations if x.rule == 'assembly_side'])
    nf = len([x for x in rf.violations if x.rule == 'assembly_side'])
    check("rule: ulx3s declared B flags its 63 front parts",
          nb == 63, f"got {nb}")
    check("rule: ulx3s declared F flags its 163 back parts",
          nf == 163, f"got {nf}")
    check("rule: a single-sided board declared its own face is clean",
          not [x for x in graded_with('splitflap_driver', 'F').violations
               if x.rule == 'assembly_side'], 'splitflap_driver')

    # `both` RUNS and reports nothing. It must not SKIP: the intent declares
    # the key, so a skip lands in the abstention channel as "declared and
    # ungraded" -- measured, that took glasgow_revC and orangecrab_ext_pll
    # from pass:true to pass:false on their own emitted intents.
    rboth = graded_with('ulx3s', 'both')
    check("rule: `both` RUNS rather than skipping",
          'assembly_side' in rboth.rules_run
          and 'assembly_side' not in (rboth.rules_skipped or {}),
          f"run={('assembly_side' in rboth.rules_run)} "
          f"skipped={(rboth.rules_skipped or {}).get('assembly_side')}")
    check("rule: ...and reports nothing, because every face is declared",
          not [x for x in rboth.violations if x.rule == 'assembly_side'])

    # An emitted intent grades clean BY CONSTRUCTION on every tracked board.
    # That is the property that lets this ship default-on.
    import glob
    dirty = []
    for path in sorted(glob.glob(os.path.join(ROOT, 'kicad_files',
                                              '*.kicad_pcb'))):
        pcb = parse_kicad_pcb(path)
        try:
            res = fp.grade(fp.intent_from_dict(fp.emit_intent(pcb, path)),
                           pcb, path)
        except Exception:                                      # noqa: BLE001
            continue                        # outline/board problems: not ours
        bad = [x for x in res.violations if x.rule == 'assembly_side']
        if bad:
            dirty.append((os.path.basename(path), len(bad)))
    check("emit: the observed policy grades clean on every tracked board",
          not dirty, str(dirty))

    # ...and it is the OBSERVED policy, not a constant. A hardcoded `both`
    # also grades clean on every board, so the arm above cannot tell the two
    # apart -- this one can, because 15 of the 22 tracked boards observe a
    # single face.
    emitted = {}
    for path in sorted(glob.glob(os.path.join(ROOT, 'kicad_files',
                                              '*.kicad_pcb'))):
        pcb = parse_kicad_pcb(path)
        doc = fp.emit_intent(pcb, path)
        emitted[os.path.basename(path)] = (
            (doc.get('assembly') or {}).get('sides'),
            assembly_census(pcb)['sides'])
    disagree = {k: v for k, v in emitted.items() if v[0] != v[1]}
    check("emit: every board's declared sides IS its observed sides",
          not disagree, str(disagree))
    check("emit: and the corpus contains both answers, or the arm is vacuous",
          len({v[0] for v in emitted.values()}) >= 2,
          str({v[0] for v in emitted.values()}))

    # The CLI end of the arithmetic. `grow_board` is exercised directly by
    # tests/test_capacity_options.py; this arm exists so the THREADING --
    # flag -> capacity_options -> grow_board -> JSON_SUMMARY -- cannot be cut
    # without a test noticing.
    def capacity(stem, *extra, quiet=True):
        # `-q` suppresses `format_text`, so the arm that checks the TEXT
        # channel must not pass it. Found by that arm failing: a check that
        # looked for the digest in output the flag had turned off would have
        # been a false red, and looking only at the JSON would have been a
        # false green for the very trap the bool type exists to avoid.
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_tools', 'check_capacity.py'),
                            board(stem), '--only', 'grow_board',
                            *(['-q'] if quiet else []), *extra],
                           capture_output=True, text=True, cwd=ROOT,
                           env=dict(os.environ, KRT_NO_BANNER='1'))
        out = r.stdout + r.stderr
        line = [ln for ln in out.splitlines() if 'JSON_SUMMARY' in ln]
        return r, out, (json.loads(line[0].split('JSON_SUMMARY: ', 1)[1])
                        if line else {})

    r_none, _, d_none = capacity('ulx3s')
    r_both, _, d_both = capacity('ulx3s', '--assembly-sides', 'both')
    r_f, out_f, d_f = capacity('ulx3s', '--assembly-sides', 'F',
                                quiet=False)
    check("capacity: the flag reaches grow_board and changes the charge",
          d_f.get('utilisation') != d_none.get('utilisation')
          and d_f.get('charged_area_is_sum') is True
          and d_f.get('fits_by_area') is False,
          f"none={d_none} F={d_f}")
    check("capacity: `both` is identical to undeclared, to the last digit",
          d_both.get('utilisation') == d_none.get('utilisation')
          and d_both.get('fits_by_area') == d_none.get('fits_by_area')
          and d_both.get('charged_area_is_sum') is False,
          f"none={d_none} both={d_both}")
    check("capacity: JSON_SUMMARY names the declared sides",
          d_f.get('assembly_sides') == 'F'
          and d_none.get('assembly_sides') is None,
          f"F={d_f.get('assembly_sides')} none={d_none.get('assembly_sides')}")
    # The basis must reach the TEXT channel. Not via the digest: forcing
    # `charged_area_is_sum` into `_DIGEST_ALWAYS` evicted `part_area_mm2` on
    # 21 of 22 boards, because `_digest` returns `out[:max(limit,
    # len(forced))]` and the forced list was already at the limit. So the
    # human record is `check_capacity`'s own line plus the action string,
    # both unconditional, and the bool stays the machine channel.
    check("capacity: the text channel names the declared policy",
          'assembly sides: F' in out_f, out_f[-600:])
    check("capacity: ...and the action names the face that must carry it",
          'F.Cu, which must carry every part' in out_f, out_f[-900:])
    check("capacity: the digest did not lose part_area_mm2 to the new keys",
          'part_area_mm2=' in capacity('glasgow_revC', quiet=False)[1],
          capacity('glasgow_revC', quiet=False)[1][-400:])
    check("capacity: all three invocations exit 0",
          r_none.returncode == r_both.returncode == r_f.returncode == 0,
          f"{r_none.returncode} {r_both.returncode} {r_f.returncode}")
    # The DISCLOSURE, in the channel a human reads. It lived inside
    # `measured` once, where `format_text` never looks and `_digest` never
    # renders it, so both sentences it carries were invisible outside --json.
    _, out_none, _ = capacity('ulx3s', quiet=False)
    check("capacity: the undeclared run says the back area is credited free",
          'NOT MODELLED' in out_none
          and 'credited free' in out_none, out_none[-700:])
    check("capacity: and the declared run drops that half, keeping the other",
          'NOT MODELLED' in out_f and 'credited free' not in out_f
          and 'through-hole leads' in out_f, out_f[-700:])

    # An intent that declares NOTHING is not an intent that declares `both`.
    # They share an arithmetic and are different statements, and reading them
    # alike made both machine channels assert a policy nobody made.
    with tempfile.TemporaryDirectory() as td:
        empty = os.path.join(td, 'empty_intent.json')
        with open(empty, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent',
                       'units': 'mm'}, fh)
        _, out_e, d_e = capacity('ulx3s', '--intent', empty, quiet=False)
        check("capacity: an intent declaring nothing reports nothing",
              d_e.get('assembly_sides') is None
              and d_e.get('assembly_sides_source') == 'intent declares none',
              f"{d_e.get('assembly_sides')} / "
              f"{d_e.get('assembly_sides_source')}")
        check("capacity: ...and keeps the credited-free disclosure",
              'credited free' in out_e, out_e[-600:])
        decl = os.path.join(td, 'both_intent.json')
        with open(decl, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                       'assembly': {'sides': 'both'}}, fh)
        _, _, d_b = capacity('ulx3s', '--intent', decl)
        check("capacity: a declared `both` is told apart from silence",
              d_b.get('assembly_sides') == 'both'
              and d_b.get('assembly_sides_source') == 'intent'
              and d_b.get('utilisation') == d_e.get('utilisation'),
              f"declared {d_b.get('assembly_sides_source')} vs silent "
              f"{d_e.get('assembly_sides_source')}")

    # The loader refuses by REASON. `single` is the spelling an author reaches
    # for first, and "it does not say WHICH face" is the whole correction --
    # ulx3s is back-dominant, so "single implies front" would flag 163 parts.
    base = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'}
    for bad_doc, want in (
            ({'sides': 'single'}, 'name the face'),
            ({'sides': 'B.Cu'}, "must be one of"),
            ({'why': 'no sides key'}, 'constrains nothing'),
            ({'side': 'F'}, 'unknown key')):
        try:
            fp.intent_from_dict(dict(base, assembly=bad_doc))
            check(f"loader: refuses {bad_doc}", False, 'it LOADED')
        except IntentError as exc:
            check(f"loader: refuses {bad_doc} and says why",
                  want in str(exc), f"{want!r} not in {str(exc)[:160]!r}")
    check("loader: accepts the three declared faces",
          all(fp.intent_from_dict(
              dict(base, assembly={'sides': s})).assembly_sides() == s
              for s in ('F', 'B', 'both')))
    check("loader: an intent with no assembly key defaults to both",
          fp.intent_from_dict(base).assembly_sides() == 'both')
    check("reader version names the field it learned",
          fp.READER_VERSION == 3, fp.READER_VERSION)

    check("graded every fixture", graded == len(EXPECT), f"{graded}")
    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #837 census over {graded} "
          f"boards, {len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f"  - {f}")
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
