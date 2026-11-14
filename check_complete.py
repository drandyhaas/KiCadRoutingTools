#!/usr/bin/env python3
"""Is this board DONE? The one command that answers it, and fails closed.

    python3 -X utf8 check_complete.py BOARD [--authored-from ORIGINAL] \
        [--intent J] [--impedance-nets G...] [--length-groups J] \
        [--net-min-widths J] [--min-track-width MM] [--min-via-diameter MM] \
        [--clearance MM] [--kicad-rules] [--json PATH]

`board_score` is the authority on `blocking`, and it is honest about being
incomplete -- but read alone it says "done" too easily, three separate ways:

  * IT EXITS 0 WITH FOUR OF NINE COMPONENTS UNGRADED. floorplan, impedance,
    length and net_widths return `skipped(...)` when their flag is absent, and
    `ungraded` does not touch the exit code. A board nobody graded exits 0.
  * `undersized` IS SILENTLY PERMISSIVE without spec numbers. It runs at the
    FAB floor for the layer count, so `undersized == 0` is not "the sizes are
    right" -- one board had 141 of 141 vias violating a 0.6mm spec while
    clearing the 0.25mm two-layer floor.
  * IT HAS NO COMPONENT AT ALL for orphan stubs, weird copper, pad overlaps or
    channel starvation. Several of those are run by the routing chain and none
    of them reaches the scalar.

And underneath all of it, the DRC writeback rewrites the board's own
manufacturing floors down to whatever was produced and then everything grades
against the new value. That is disclosed in a printed line and enforced by
nothing.

So this aggregates, and the difference from board_score is the direction it
fails in. Three verdict classes, and only one of them is "done":

    DONE        every component that could be graded is graded and clean
    INCOMPLETE  a component RAN and could not answer, or an instrument this
                adds found something -- exit 4
    UNSOUND     the board grades clean against floors it rewrote -- exit 5
                (a floor the --authored-from board's own copper breaks at
                least as badly is named in the reason instead: the
                reference's, not this board's)

UNGRADED components do NOT block DONE -- a board with no spec files has nothing
to grade them against, and making that fatal would put every corpus board
permanently in the failing state -- but they are NAMED in the verdict line,
because a component nothing examined is UNEXAMINED and never clean.

Exit: 0 done, 2 usage, 3 board state, 4 incomplete, 5 unsound floors.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import argparse
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
# #522/py_placer layout: the engine modules (kicad_parser, cli_banner, ...)
# live in py_router/py_tools/py_placer, not at the repo root, so inserting
# only ROOT left `import cli_banner` failing when run as a script.
sys.path.insert(0, ROOT)
for _pkg in ('py_router', 'py_tools', 'py_placer'):
    _d = os.path.join(ROOT, _pkg)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)
# board_score is a repo instrument, in py_tools/ with the checkers it runs. It
# used to live inside a skill's scripts/ dir, and every time that skill moved
# this lookup broke with "board_score produced no score" -- an INCOMPLETE
# verdict caused by a missing file rather than by the board.
SKILL_SCRIPTS = os.path.join(ROOT, 'py_tools')
PY = [sys.executable, '-X', 'utf8']

DONE, USAGE, BOARD_STATE, INCOMPLETE, UNSOUND = 0, 2, 3, 4, 5


TIMED_OUT = 124                     # the shell's code, and deliberately so:
                                    # a caller reading 124 is reading "killed
                                    # by a clock", which is what happened.


def _score_names(score, key):
    """`score[key]` as sorted names, and why not when it is not a list.

    board_score writes a list. Anything else names no component -- the rule
    converge's verdict follows (#1076): `sorted("abc")` read as three
    components nobody examined, and `sorted(5)` raised.
    """
    v = score.get(key)
    if v is None:
        return [], None
    if isinstance(v, (list, tuple, set)):
        return sorted(str(x) for x in v), None
    return [], f'board_score wrote an `{key}` that is not a list ({v!r})'


def _run(args, timeout=3600):
    """Run a checker. A timeout is a RESULT, not an exception.

    This process is the thing a gate reads, so it must not die on the slowest
    board in the corpus: board_score alone fans out to check_drc, and on a
    6-layer board this runs six orphan-stub sweeps beside it. An uncaught
    TimeoutExpired propagates out of main and takes the whole document with it
    (see the finally in main) -- which loses the verdict on exactly the boards
    a close-out exists for.
    """
    try:
        p = subprocess.run(PY + args, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', cwd=ROOT,
                           timeout=timeout,
                           env=dict(os.environ, KRT_NO_BANNER='1'))
    except subprocess.TimeoutExpired:
        return TIMED_OUT, f'timed out after {timeout}s'
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def copper_layers(board):
    """-> (layers, error). The error is a STRING, never a raised exception.

    Returning a bare [] on a parse failure would make "this board has no copper
    layers" and "nobody could read this board" the same answer, which is the
    conflation this whole file exists to refuse. The caller records the error
    as a reason rather than sweeping a partial sweep in as a clean one.
    """
    try:
        from kicad_parser import parse_kicad_pcb
        return list(parse_kicad_pcb(board).board_info.copper_layers or []), None
    except Exception as exc:                                    # noqa: BLE001
        return [], f'{type(exc).__name__}: {exc}'


#: The objects a measurable fab floor is a statement about (#1198).
_FLOOR_OBJECT = {
    'min_track_width': 'track',
    'min_via_diameter': 'via', 'min_via_annular_width': 'via',
    'min_via_drill': 'via',
    'min_through_hole_diameter': 'drilled hole',
}

#: Slack when one measured minimum is compared with another. KiCad reports an
#: actual distance to 4 decimals (``actual 0.1540 mm``).
_MIN_TOL = 1e-4


def _kicad_drc_report(board, timeout=3600, project_edit=None, rules_edit=None):
    """kicad-cli's DRC report (the parsed JSON) of a STAGED copy of ``board``
    and its siblings, on the fill as shipped (#1370), or ``{'error': why}``
    when kicad-cli is missing, times out or writes no report. Never raises.
    ``project_edit(proj)`` may change the staged project's JSON first, and
    ``rules_edit(text) -> text`` the staged `.kicad_dru`."""
    import tempfile
    from copy_board import copy_board
    try:
        from kicad_oracle import find_kicad_cli
        cli = find_kicad_cli(warn=False)
    except Exception as exc:                                    # noqa: BLE001
        return {'error': f'no kicad-cli lookup: {exc}'}
    if not cli:
        return {'error': 'kicad-cli not found (set KICAD_CLI)'}
    tmp = tempfile.mkdtemp(prefix='check_complete_drc.')
    try:
        dst = os.path.join(tmp, 'b.kicad_pcb')
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(board, dst)
        if project_edit is not None:
            pro = os.path.join(tmp, 'b.kicad_pro')
            proj = {}
            if os.path.isfile(pro):
                with open(pro, encoding='utf-8') as fh:
                    proj = json.load(fh)
            project_edit(proj)
            with open(pro, 'w', encoding='utf-8') as fh:
                json.dump(proj, fh, indent=2)
        if rules_edit is not None:
            dru = os.path.join(tmp, 'b.kicad_dru')
            if not os.path.isfile(dru):
                return {'error': 'the board has no .kicad_dru beside it'}
            with open(dru, encoding='utf-8') as fh:
                text = fh.read()
            with open(dru, 'w', encoding='utf-8') as fh:
                fh.write(rules_edit(text))
        out = os.path.join(tmp, 'drc.json')
        try:
            r = subprocess.run([cli, 'pcb', 'drc', '--format', 'json',
                                '--units', 'mm', '--severity-all',
                                '--output', out, dst],
                               capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return {'error': f'kicad-cli timed out after {timeout:g} s'}
        if not os.path.isfile(out) or os.path.getsize(out) == 0:
            return {'error': f'kicad-cli produced no report (rc={r.returncode}: '
                             f'{r.stderr.strip()[:200]})'}
        with open(out, encoding='utf-8') as fh:
            return json.load(fh)
    except Exception as exc:                                    # noqa: BLE001
        return {'error': f'{type(exc).__name__}: {exc}'}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


#: kicad-cli lists at most this many violations of one type, so a type that
#: reaches it may have dropped a row naming a rule that reports under it.
_KICAD_TYPE_CAP = 199
#: ... except `clearance`, which it lists up to 499 of (#1865; KiCad's
#: `drc_engine.cpp`, `EXTENDED_ERROR_LIMIT`, the same from KiCad 7 to 10): 483
#: `clearance` rows are a whole report, not one cut at 199. (Its other 499
#: type, `unconnected_items`, is listed in an array of its own, which this
#: grade does not read.) A KiCad before 10.0.7 can list a few rows past a
#: limit, so a count is compared with `>=`.
_KICAD_TYPE_CAPS = {'clearance': 499}

#: The violation type(s) KiCad reports a row NAMING a rule under, by the
#: rule's constraint (#1865): a type that was cut can have dropped a row of a
#: rule only when the rule reports under it. Each entry is every type KiCad's
#: DRC test providers create with the rule's name in the description, read
#: off its sources; kicad-cli 10.0.7 reported all of them on a probe board
#: but `microvia_drill_out_of_range` and `silk_edge_clearance`
#: (`tests/test_1865_capped_type_scope.py` asks it again). A kind that is
#: not listed is one nobody measured: `judge_rules` then counts every cut
#: type against the rule, as it did for all of them before.
_KICAD_ROW_TYPES = {
    'clearance': ('clearance',),
    'physical_clearance': ('clearance',),
    'hole_clearance': ('hole_clearance',),
    'physical_hole_clearance': ('hole_clearance',),
    'edge_clearance': ('copper_edge_clearance',),
    'hole_to_hole': ('hole_to_hole',),
    'hole_size': ('drill_out_of_range', 'microvia_drill_out_of_range'),
    'track_width': ('track_width',),
    'track_segment_length': ('track_segment_length',),
    'via_diameter': ('via_diameter',),
    'annular_width': ('annular_width',),
    'courtyard_clearance': ('courtyards_overlap',),
    'silk_clearance': ('silk_overlap', 'silk_over_copper', 'silk_edge_clearance'),
    'text_height': ('text_height',),
    'text_thickness': ('text_thickness',),
    'length': ('length_out_of_range',),
    'skew': ('skew_out_of_range',),
    'via_count': ('too_many_vias',),
    'disallow': ('items_not_allowed',),
    'assertion': ('assertion_failure',),
}


def _type_cap(kind):
    """The number of rows of violation type ``kind`` KiCad's report stops at."""
    return _KICAD_TYPE_CAPS.get(kind, _KICAD_TYPE_CAP)


#: The rule `kicad_rule_grade` appends to the STAGED rules file to learn
#: whether KiCad loaded that file (#1810).
_LOAD_PROBE = 'check_complete load probe'


def _load_probe(board, taken=()):
    """A `.kicad_dru` rule that KiCad reports once on ``board`` whenever it
    loads the rules file the rule stands in, and never when it drops the
    file: an `assertion`, at warning severity, that one footprint -- named by
    a reference no other footprint has -- does not carry that reference.
    ``taken`` is the rule names the file already declares.

    -> ``(name, rule text)``, or ``(None, why)`` when the board has no such
    footprint. It can neither add a finding nor hide one: a warning row is
    never counted, the grader leaves its name out of the per-rule counts, and
    KiCad reports every assertion rule that binds an item, so the probe takes
    nothing from an assertion of the file's own on the same footprint
    (measured on kicad-cli 10.0.6 and 10.0.7: the report with the probe is the
    report without it plus the probe's one row, also on a project whose
    `rule_severities` ignores `assertion_failure`).

    The references are read off the board's text by the parser's own block
    scan (`kicad_parser.iter_footprint_blocks`), not a full parse: the probe
    needs a name, and a large board's full parse costs seconds."""
    import collections
    import re
    try:
        from kicad_parser import iter_footprint_blocks
        with open(board, encoding='utf-8') as fh:
            refs = [raw for _s, _e, _t, raw, _k in iter_footprint_blocks(fh.read())]
    except Exception as exc:                                    # noqa: BLE001
        return None, (f'the board could not be read for a footprint to probe '
                      f'the rules file with: {type(exc).__name__}: {exc}')
    count = collections.Counter(refs)
    ref = next((r for r in refs
                if count[r] == 1 and re.fullmatch(r'[A-Za-z0-9_]+', r or '')), None)
    if ref is None:
        return None, ('the board has no footprint with a plain reference of '
                      'its own to probe the rules file with, so whether KiCad '
                      'loaded the .kicad_dru is not known')
    name, n = _LOAD_PROBE, 1
    while name in taken:
        n += 1
        name = f'{_LOAD_PROBE} {n}'
    return name, (f'(rule "{name}" (severity warning)\n'
                  f'  (constraint assertion "A.Reference != \'{ref}\'")\n'
                  f'  (condition "A.Type == \'Footprint\'"))')


def kicad_rule_grade(board, timeout=3600):
    """KiCad's own grade of the board's `.kicad_dru` rules (#1810): one
    kicad-cli DRC of the board as shipped, its error-severity violations
    counted per rule KiCad names in them (``(rule 'NAME' ...``).

    -> ``{'kicad_version': str, 'errors_by_rule': {name: N}, 'capped_types':
    [type, ...]}``, or ``{'error': why}`` when KiCad cannot be asked: no
    kicad-cli, a timeout, no report, or a rules file KiCad does not load
    (#1690 -- KiCad then enforces none of its rules and says nothing, so its
    silence about a rule is no grade of it). A warning-severity row counts 0,
    as check_drc does not count it (#1626). Never raises.

    Whether KiCad loaded the file is KiCad's answer: the staged copy's rules
    file gets one more rule, `_load_probe`, and a report with no row naming
    it, or any rule of the file, is a file KiCad dropped. The refusal model
    (`read_board_kicad_refusals`, which every tool that does not ask KiCad
    reads) only says WHY, when it knows: beside a file KiCad did not load,
    or in place of KiCad's answer when KiCad cannot be asked."""
    import re
    try:
        from design_rules import read_board_kicad_refusals
        refused = read_board_kicad_refusals(board)
    except Exception as exc:                                    # noqa: BLE001
        refused = [{'rule': '', 'reason': f'the rules file could not be read: '
                                          f'{type(exc).__name__}: {exc}'}]
    known = ('KiCad enforces no rule of the .kicad_dru ('
             + '; '.join(((f"rule '{r['rule']}': " if r['rule'] else '')
                          + r['reason']) for r in refused) + ')') if refused else ''
    try:
        taken = _dru_rule_names(board)
    except Exception:                                           # noqa: BLE001
        taken = []
    probe, rule = _load_probe(board, taken)
    if probe is None:
        return {'error': known or rule}
    report = _kicad_drc_report(
        board, timeout,
        rules_edit=lambda text: text.rstrip('\n') + '\n' + rule + '\n')
    if 'error' in report:
        return {'error': known} if known else report
    by_rule, by_type, loaded = {}, {}, False
    for v in report.get('violations') or ():
        # the probe's row counts toward its type's 199 rows: it took one
        by_type[v.get('type')] = by_type.get(v.get('type'), 0) + 1
        m = re.search(r"\(rule '([^']*)'", v.get('description') or '')
        if m and m.group(1) == probe:
            loaded = True
            continue
        if m and m.group(1) in taken:
            # a row naming a rule of the file is the same proof, and the one
            # left when the probe's own row is past its type's 199
            loaded = True
        if m and v.get('severity') == 'error':
            by_rule[m.group(1)] = by_rule.get(m.group(1), 0) + 1
    if not loaded:
        return {'error': 'KiCad did not load the .kicad_dru: a probe rule '
                         'appended to the staged copy of it did not fire, so '
                         'KiCad enforces none of the file\'s rules and its '
                         'silence about a rule is no grade of it'
                         + (f' ({known})' if known else '')}
    return {'kicad_version': str(report.get('kicad_version') or 'unknown'),
            'errors_by_rule': by_rule,
            'capped_types': sorted(t for t, n in by_type.items()
                                   if n >= _type_cap(t))}


def judge_rules(names, grade, dru_names, kinds=None):
    """Each rule in ``names`` (the ones nothing here grades) judged by KiCad's
    ``grade`` (`kicad_rule_grade`), FAILING CLOSED (#1810). ``dru_names`` is
    every rule name the board's `.kicad_dru` declares, in file order, and
    ``kinds`` each rule's constraint kinds (`_dru_rule_kinds`).

    -> {name: {'verdict': 'met' | 'broken' | 'unknown', 'kicad_errors': N or
    None, 'why': str}}. A rule is `met` only when KiCad was asked, named
    exactly one rule of that name, and its report of 0 violations naming it
    cannot have been cut short; anything else it cannot attribute is
    `unknown`, which blocks as an ungraded rule does.

    A report is cut short FOR A RULE only in a violation type the rule
    reports under (#1865, `_KICAD_ROW_TYPES`): 199 `track_width` rows drop no
    row of a `hole_clearance` rule. A rule with a constraint kind the table
    does not list, or one ``kinds`` does not name, is held by every cut type."""
    out = {}
    for nm in names:
        if 'error' in grade:
            out[nm] = {'verdict': 'unknown', 'kicad_errors': None,
                       'why': f"KiCad could not grade it: {grade['error']}"}
            continue
        n = dru_names.count(nm)
        quoted = sorted(x for x in set(dru_names) if "'" in x)
        if n != 1 or quoted:
            out[nm] = {'verdict': 'unknown', 'kicad_errors': None,
                       'why': ("KiCad's rows cannot be attributed to it: "
                               + (f"the rules file names {n} rule(s) "
                                  f"{nm!r}" if n != 1 else
                                  f"a rule name in the file contains a quote "
                                  f"({', '.join(repr(x) for x in quoted)})"))}
            continue
        k = grade['errors_by_rule'].get(nm, 0)
        if k:
            out[nm] = {'verdict': 'broken', 'kicad_errors': k,
                       'why': f"{k} KiCad violation{'' if k == 1 else 's'}"}
            continue
        mine = sorted(set((kinds or {}).get(nm) or ()))
        unlisted = [c for c in mine if c not in _KICAD_ROW_TYPES]
        under = {t for c in mine for t in _KICAD_ROW_TYPES.get(c, ())}
        cut = [t for t in grade['capped_types']
               if not mine or unlisted or t in under]
        if cut:
            how = ('' if mine and not unlisted else
                   (f" (which type a {', '.join(unlisted)} rule reports under "
                    f"is not known here)" if mine else
                    ' (its constraint kinds were not read)'))
            out[nm] = {'verdict': 'unknown', 'kicad_errors': 0,
                       'why': ("KiCad's report is cut at "
                               + ', '.join(f'{_type_cap(t)} rows of {t}'
                                           for t in cut)
                               + f"{how}, so its 0 rows naming it may be a "
                                 f"dropped row")}
        else:
            out[nm] = {'verdict': 'met', 'kicad_errors': 0,
                       'why': '0 KiCad violations'}
    return out


def _dru_rule_kinds(board):
    """{rule name: its constraint kinds} for every rule the `.kicad_dru`
    beside ``board`` declares once; {} without one. A name declared twice has
    no entry: `judge_rules` cannot attribute a row to it anyway."""
    dru = os.path.splitext(board)[0] + '.kicad_dru'
    if not os.path.isfile(dru):
        return {}
    from design_rules import parse_dru
    with open(dru, encoding='utf-8') as fh:
        rules = parse_dru(fh.read())[0]
    names = [r.name for r in rules]
    return {r.name: sorted(r.constraints) for r in rules
            if names.count(r.name) == 1}


def _dru_rule_names(board):
    """Every rule name the `.kicad_dru` beside ``board`` declares, in file
    order (duplicates kept); [] without one."""
    dru = os.path.splitext(board)[0] + '.kicad_dru'
    if not os.path.isfile(dru):
        return []
    from design_rules import parse_dru
    with open(dru, encoding='utf-8') as fh:
        return [r.name for r in parse_dru(fh.read())[0]]


def kicad_hole_clearance(board, value, timeout=3600):
    """KiCad's own copper-to-hole grade of ``board`` at ``value`` mm (#1198).

    A staged copy (the board and its siblings) whose project declares
    ``min_hole_clearance = value`` at error severity, through kicad-cli on the
    fill AS SHIPPED (#1370): a delivered fill was poured at whatever value the
    chain lowered the rule to, and re-pouring it at the authored value here
    graded copper nobody ships. KiCad's hole_clearance covers every hole: via
    drills, PTH barrels and NPTH, which check_drc's hole arms (NPTH only) do
    not.

    -> ``{'count': N, 'min_actual': mm or None, 'items': [...]}`` where each
    item is ``{'kinds': (...), 'actual': mm}`` naming the KIND of the
    violating pair (its two items' kinds, sorted -- KiCad does not report the
    same pairs on two runs of one board, so positions cannot be matched across
    boards); or ``{'error': why}`` when kicad-cli is missing or
    produced no report. Never raises."""
    import re

    def _at_value(proj):
        ds = proj.setdefault('board', {}).setdefault('design_settings', {})
        ds.setdefault('rules', {})['min_hole_clearance'] = float(value)
        ds.setdefault('rule_severities', {})['hole_clearance'] = 'error'
    report = _kicad_drc_report(board, timeout, project_edit=_at_value)
    if 'error' in report:
        return report
    actual = []
    count = 0
    items = []
    for v in report.get('violations', []):
        if v.get('type') != 'hole_clearance' or v.get('severity') != 'error':
            continue
        count += 1
        m = re.search(r'actual\s+(-?[0-9.]+)\s*mm', v.get('description') or '')
        if m:
            actual.append(float(m.group(1)))
        # What kind of pair it is (track-via, pad-NPTH pad, ...). KiCad does
        # not report the same pairs on two runs of ONE board -- it keeps one of
        # several overlapping violations, and a re-run of an identical file
        # shares barely half its tracks -- so the pair itself cannot be matched
        # across boards; its kind can.
        kinds = tuple(sorted(
            (re.match(r'[A-Za-z]+(?: pad)?', it.get('description') or '')
             or [''])[0] for it in v.get('items') or ()))
        items.append({'kinds': kinds,
                      'actual': float(m.group(1)) if m else None})
    return {'count': count, 'min_actual': min(actual) if actual else None,
            'items': items}


def _counted(row, key='violations', paren=False):
    """A floor row's count as words (``36 vias below it``, ``5 KiCad
    violations``),
    '' when the row carries none. ``paren`` wraps it for a ``was -> is``."""
    n = row.get(key)
    if not isinstance(n, int) or isinstance(n, bool):
        return ''
    if row.get('measured_by') == 'kicad-cli':
        words = f'{n} KiCad violation{"" if n == 1 else "s"}'
    else:
        noun = _FLOOR_OBJECT.get(row.get('key'), 'object')
        words = f'{n} {noun}{"" if n == 1 else "s"} below it'
    return f' ({words})' if paren else words


def _unmatched_by_reference(board_items, ref_items):
    """The board's violations the reference does NOT break in kind, at least
    as badly (#1370).

    ``reference_too`` used to compare the two boards' MINIMUM distances. A
    reference with one pad on a hole at 0.0 mm then excused every violation
    of a board whose copper the reference never had: track-to-via pairs
    against pad-to-hole ones, a different set of items. A violation is the
    reference's too only when the reference breaks the same KIND of pair
    (its two items' kinds) at a distance no larger than the board's worst of
    that kind. Returns the board's violations that fail that."""
    ref = {}
    for it in ref_items or ():
        a = it.get('actual')
        if a is not None:
            ref[it['kinds']] = min(a, ref.get(it['kinds'], a))
    worst = {}
    for it in board_items or ():
        a = it.get('actual')
        if a is not None:
            worst[it['kinds']] = min(a, worst.get(it['kinds'], a))
    out = []
    for it in board_items or ():
        k = it['kinds']
        r = ref.get(k)
        if it.get('actual') is None or r is None or r > worst[k] + _MIN_TOL:
            out.append(it)
    return out


def fab_floor_integrity(board, authored_from, hole_grader=None, timeout=3600):
    """Does this board's copper sit below the floors its project ONCE declared?

    The writeback only ever loosens: every constraint is set to min(current,
    target) and the target is the smallest object actually placed. So a board
    that routed at 0.0889 rewrites its own 0.2 minimum and then grades clean
    against it -- measured at 5629 of 5933 segments below the original floor,
    with check_drc, board_score and KiCad's own DRC all reading clean.

    Needs the ORIGINAL project to compare against, because a chain overwrites
    the floors in place. Without it, say the check did not run rather than
    implying it passed.

    With ``authored_from``, a size floor is judged per object (#1627): each
    track, via and hole against the floor that binds IT in the authored
    project -- the Board Setup minimum, or the minimum an authored
    ``.kicad_dru`` rule sets for it (resolved as check_drc resolves it) -- so
    copper a scoped rule allows is not counted, and is reported under
    ``measured_kept`` with ``held_by_rule`` and the rules. Against the
    recorded ``fab_floor_origin`` there are no authored rules on disk, and the
    count stays against the one origin value.

    The original board is also the REFERENCE (#1198): a floor its own copper
    breaks, at least as badly as this board does, is ``reference_too`` --
    reported, not counted against this board. A board that breaks it worse,
    or a floor the reference holds, is ``relaxed``. Copper-to-hole has no
    measured counterpart in scan_board_minima, so when this board's project
    lowered it, ``hole_grader(board, mm)`` (default: KiCad's own grade,
    :func:`kicad_hole_clearance`) measures both boards at the authored value.
    """
    from fix_kicad_drc_settings import (FAB_FLOOR_KEYS,
                                        FAB_FLOOR_KEYS_MEASURABLE,
                                        count_below_floors, find_project,
                                        scan_board_minima)
    pro = None
    source = 'authored_from'
    if authored_from:
        pro = find_project(authored_from)
        if not os.path.isfile(pro):
            return {'ran': False, 'reason': f'no project beside {authored_from}'}
        try:
            with open(pro, encoding='utf-8') as fh:
                authored = ((json.load(fh).get('board') or {})
                            .get('design_settings') or {}).get('rules') or {}
        except Exception as exc:                            # noqa: BLE001
            return {'ran': False, 'reason': f'unreadable project: {exc}'}
    else:
        # #1368: without a reference board the floors a chain rewrote are
        # still on record -- the writeback seeds
        # ``kicad_routing_tools.fab_floor_origin`` with what the board
        # declared BEFORE the first step lowered anything. Grade against it
        # instead of declaring nothing left to compare with. Only a board no
        # tool ever wrote back has no origin, and there the project IS the
        # declaration.
        try:
            import tool_records
            authored = dict(tool_records.effective_namespace(board)
                            .get('fab_floor_origin') or {})
        except Exception as exc:                            # noqa: BLE001
            return {'ran': False, 'reason': f'unreadable fab_floor_origin: {exc}'}
        if not authored:
            return {'ran': False,
                    'reason': 'no --authored-from and the project records no '
                              'fab_floor_origin: no earlier declaration to '
                              'compare the floors with'}
        source = 'fab_floor_origin'
    now = scan_board_minima(board) or {}
    try:
        with open(find_project(board), encoding='utf-8') as fh:
            declared_now = ((json.load(fh).get('board') or {})
                            .get('design_settings') or {}).get('rules') or {}
    except (OSError, ValueError):
        declared_now = {}
    try:
        # No reference board under the origin source: nothing is excused as
        # `reference_too`, so every breach counts.
        ref_min = (scan_board_minima(authored_from) or {}) if authored_from \
            else {}
    except Exception:                                           # noqa: BLE001
        ref_min = {}
    grade_holes = hole_grader or (
        lambda b, v: kicad_hole_clearance(b, v, timeout=timeout))
    # #1580: a relaxed size floor carries a COUNT -- the objects below the
    # authored value, over the objects its minimum is read off -- on this
    # board and on the reference, not only the smallest one.
    _sizes = {k: authored[k] for k in FAB_FLOOR_KEYS_MEASURABLE
              if isinstance(authored.get(k), (int, float))
              and not isinstance(authored.get(k), bool) and authored[k] > 0}
    below = count_below_floors(board, _sizes) if _sizes else {}
    ref_below = (count_below_floors(authored_from, _sizes)
                 if (_sizes and authored_from) else {})
    # #1627: with the authored board in hand, each object is compared with
    # the floor that binds IT in the authored project -- Board Setup, then the
    # authored .kicad_dru (never the graded board's: a rule a chain added
    # cannot excuse its own copper) -- as check_drc resolves it.
    per_obj = ref_obj = {}
    if _sizes and authored_from:
        from fix_kicad_drc_settings import size_values_below_authored
        per_obj = size_values_below_authored(board, _sizes, authored_from)
        ref_obj = size_values_below_authored(authored_from, _sizes, authored_from)
    relaxed = []
    unmeasured = []
    vacuous = []
    declared_kept = []
    measured_kept = []
    reference_too = []
    for key, label in FAB_FLOOR_KEYS:
        was, is_ = authored.get(key), now.get(key)
        if not isinstance(was, (int, float)) or isinstance(was, bool):
            continue
        if was <= 0:
            # #1198: a declared 0 is honoured by any board (ecc83 and
            # complex_hierarchy declare copper-to-hole 0.0).
            vacuous.append({'key': key, 'label': label, 'authored': was,
                            'why': 'declared 0: nothing can sit below it'})
            continue
        if key in FAB_FLOOR_KEYS_MEASURABLE:
            # #1843: the measured scan leaves out a track a rule area's
            # track_width rule binds (KiCad grades it at the rule), so a board
            # whose every track is in such a room has no measured minimum; the
            # per-object grade below still sees those tracks
            if not isinstance(is_, (int, float)) and key not in per_obj:
                # scan_board_minima emits a key only when the board has that
                # kind of object, so a missing one is a floor held vacuously:
                # a 0-via board breaks no via floor (#1198).
                vacuous.append({'key': key, 'label': label, 'authored': was,
                                'why': f'no {_FLOOR_OBJECT.get(key, "object")} '
                                       f'on the board'})
            elif key in per_obj:
                # #1627: the objects below their OWN authored floor
                po = per_obj[key]
                rv = (ref_obj.get(key) or {}).get('below') or []
                if not po['below']:
                    if po['held_by_rule']:
                        measured_kept.append({
                            'key': key, 'label': label, 'authored': was,
                            'on_board': is_ if is_ is not None else po.get('smallest'),
                            'measured_by': 'authored .kicad_dru rules',
                            'violations': 0,
                            'held_by_rule': po['held_by_rule'],
                            'rules': po['rules']})
                    continue
                worst = min(po['below'])
                row = {'key': key, 'label': label, 'authored': was,
                       'on_board': worst,
                       'measured_by': 'authored floor per object',
                       'violations': len(po['below'])}
                if po['held_by_rule']:
                    row['held_by_rule'] = po['held_by_rule']
                    row['rules'] = po['rules']
                if rv and worst >= min(rv) - _MIN_TOL:
                    row['reference'] = min(rv)
                    row['reference_violations'] = len(rv)
                    reference_too.append(row)
                else:
                    relaxed.append(row)
            elif is_ < was - 1e-9:
                ref = ref_min.get(key)
                row = {'key': key, 'label': label, 'authored': was,
                       'on_board': is_, 'measured_by': 'scan_board_minima',
                       'violations': below.get(key)}
                if isinstance(ref, (int, float)) and ref < was - 1e-9 \
                        and is_ >= ref - _MIN_TOL:
                    row['reference'] = ref
                    row['reference_violations'] = ref_below.get(key)
                    reference_too.append(row)
                else:
                    relaxed.append(row)
            continue
        # Copper-to-hole: scan_board_minima reads object sizes, no pairwise
        # geometry, so it has no measured counterpart (#1198). A project that
        # still declares the authored floor is graded at it by whatever grades
        # copper-to-hole (KiCad's DRC; check_drc's hole arms cover NPTH
        # only): declared vs declared, nothing to report. One that LOWERED it
        # is graded here at the authored value, by KiCad, on this board and
        # then on the reference. Unmeasurable (no kicad-cli) it stays
        # UNKNOWN, not honoured: silently skipping it made `relaxed: []` read
        # as "every declared floor is honoured" when it was never looked at,
        # so it is named and keeps the verdict INCOMPLETE.
        dec = declared_now.get(key)
        if isinstance(dec, (int, float)) and dec >= was - 1e-9:
            declared_kept.append({'key': key, 'label': label, 'authored': was,
                                  'declared': dec})
            continue
        row = {'key': key, 'label': label, 'authored': was, 'declared': dec}
        if not authored_from:
            # Against the recorded origin the floors are DISCLOSED, not
            # graded (see _grade): a KiCad DRC run to measure one is not paid.
            row['why'] = ('lowered since the recorded fab_floor_origin; pass '
                          '--authored-from to have KiCad measure it')
            unmeasured.append(row)
            continue
        g = grade_holes(board, was) or {'error': 'no grade'}
        if g.get('error'):
            row['why'] = f'not measured: {g["error"]}'
            unmeasured.append(row)
            continue
        row.update(measured_by='kicad-cli', violations=g['count'])
        if not g['count']:
            measured_kept.append(row)
            continue
        row['on_board'] = g.get('min_actual')
        if authored_from:
            rg = grade_holes(authored_from, was) or {'error': 'no grade'}
        else:
            rg = {'error': 'no reference board (graded against the recorded '
                           'fab_floor_origin)'}
        if rg.get('error'):
            row['reference_error'] = rg['error']
        else:
            row['reference_violations'] = rg['count']
            row['reference'] = rg.get('min_actual')
        # #1370: the reference excuses a violation only on the SAME pair, at
        # least as badly. Without the pairs of both boards the reference
        # cannot be compared, which counts against the board.
        if not rg.get('error') and rg.get('count') \
                and g.get('items') and rg.get('items') is not None:
            left = _unmatched_by_reference(g['items'], rg['items'])
            row['not_in_reference'] = len(left)
            if not left:
                reference_too.append(row)
                continue
            lefts = [i['actual'] for i in left if i.get('actual') is not None]
            if lefts:
                row['on_board'] = min(lefts)
        relaxed.append(row)
    # #1160: the Default CLASS clearance, declared vs declared (no measured
    # counterpart: clearance is pairwise). Counted only when an automatic
    # descent lowered it -- the board's project carries the writeback's
    # class_clearance_relaxed record -- because a class lowered by the
    # chain's own --clearance / --clearance-ceiling is a decision, not a
    # relaxation (stock classes are aspirational).
    try:
        if not pro:
            raise OSError('no reference project')       # origin source
        from fab_tiers import project_default_class_clearance
        from fix_kicad_drc_settings import CLASS_CLEARANCE_RELAXED_KEY
        with open(pro, encoding='utf-8') as fh:
            _cls_was = project_default_class_clearance(json.load(fh))
        _bpro = find_project(board)
        with open(_bpro, encoding='utf-8') as fh:
            _bproj = json.load(fh)
        _cls_now = project_default_class_clearance(_bproj)
        _rec = (_bproj.get('kicad_routing_tools') or {}).get(
            CLASS_CLEARANCE_RELAXED_KEY)
        if _rec and _cls_was is not None and _cls_now is not None \
                and _cls_now < _cls_was - 1e-9:
            relaxed.append({'key': 'net_class.Default.clearance',
                            'label': 'Default class clearance (lowered by an '
                                     'automatic descent on '
                                     + ', '.join(_rec.get('nets') or ['?']) + ')',
                            'authored': _cls_was, 'on_board': _cls_now})
        # #1492: every NON-Default class a descent on one of its own nets
        # lowered below the ceiling it was routed at, named the same way
        from fix_kicad_drc_settings import CLASS_CLEARANCE_RELAXED_CLASSES_KEY
        _crecs = (_bproj.get('kicad_routing_tools') or {}).get(
            CLASS_CLEARANCE_RELAXED_CLASSES_KEY) or {}
        with open(pro, encoding='utf-8') as fh:
            _was_cls = {c.get('name'): c.get('clearance') for c in
                        ((json.load(fh).get('net_settings') or {}).get('classes') or ())}
        _now_cls = {c.get('name'): c.get('clearance') for c in
                    ((_bproj.get('net_settings') or {}).get('classes') or ())}
        for _cn, _r in sorted(_crecs.items()):
            _w, _n = _was_cls.get(_cn), _now_cls.get(_cn)
            if isinstance(_w, (int, float)) and isinstance(_n, (int, float)) \
                    and _n < _w - 1e-9:
                relaxed.append({'key': f'net_class.{_cn}.clearance',
                                'label': f'{_cn} class clearance (lowered by an '
                                         f'automatic descent on '
                                         + ', '.join(_r.get('nets') or ['?']) + ')',
                                'authored': _w, 'on_board': _n})
    except (OSError, ValueError):
        pass
    return {'ran': True, 'relaxed': relaxed, 'unmeasured': unmeasured,
            'vacuous': vacuous, 'declared_kept': declared_kept,
            'measured_kept': measured_kept, 'reference_too': reference_too,
            'authored_from': authored_from, 'source': source}


def _grade(a, doc):
    """Fill `doc` and return the exit code. Raises only on a genuine bug.

    Split out of main so main can own a try/finally: the document must be
    written even when this raises, because the boards that break an instrument
    are exactly the boards a close-out is being asked about.
    """
    # ---- 1. board_score, with every spec flag the caller has ---------------
    # The intermediate goes to a temp dir, not next to the board: a checker
    # that drops files into the directory it is inspecting turns a read-only
    # question into a write, and on a corpus directory that is somebody's
    # source tree.
    import tempfile
    scratch = tempfile.mkdtemp(prefix='check_complete.')
    try:
        score_path = os.path.join(scratch, 'score.json')
        args = [os.path.join(SKILL_SCRIPTS, 'board_score.py'), a.board,
                '--json', score_path, '--quiet']
        for flag, val in (('--clearance', a.clearance), ('--intent', a.intent),
                          ('--baseline', a.baseline),
                          ('--length-groups', a.length_groups),
                          ('--net-min-widths', a.net_min_widths),
                          ('--min-track-width', a.min_track_width),
                          ('--min-via-diameter', a.min_via_diameter),
                          ('--min-via-drill', a.min_via_drill)):
            if val is not None:
                args += [flag, str(val)]
        if a.impedance_nets:
            args += ['--impedance-nets'] + list(a.impedance_nets)
        code, out = _run(args, timeout=a.timeout)
        score = {}
        if os.path.isfile(score_path):
            with open(score_path, encoding='utf-8') as fh:
                score = json.load(fh)
    finally:
        shutil.rmtree(scratch, ignore_errors=True)

    doc['score'] = score
    # Hoisted so a gate can bind this document to a board by CONTENT rather
    # than by path, reusing the sha board_score already computed.
    doc['board_sha'] = score.get('board_sha')
    blocking = score.get('blocking')
    ungraded, ungraded_bad = _score_names(score, 'ungraded')
    unknown, unknown_bad = _score_names(score, 'unknown')
    score_failed = (code == TIMED_OUT or not score)

    # ---- 1b. is there a board here at all? ---------------------------------
    # ALWAYS, including under --skip-slow. `parse_kicad_pcb` is lenient: a
    # truncated file yields an EMPTY board rather than an error, and an empty
    # board has nothing to violate, so every component grades 0 and the verdict
    # was DONE. "Nothing to grade" is not "graded clean" -- that is the whole
    # claim of this file, and it failed it on the cheapest possible input.
    # A real board always declares at least two copper layers.
    layers, lyr_err = copper_layers(a.board)
    doc['copper_layers'] = layers
    if lyr_err:
        doc['copper_layers_error'] = lyr_err

    # ---- 2. the instruments board_score has no component for ---------------
    extra = {}
    if not a.skip_slow:
        # Ask check_orphan_stubs per layer (it scans every copper layer by
        # default, but one exit per layer is what names which layers carry
        # orphans and which runs failed), over the layers read above.
        stub_hits, stub_ran, stub_exits = 0, [], {}
        for ly in layers:
            c, o = _run([os.path.join('py_tools', 'check_orphan_stubs.py'),
                         a.board, '--layer', ly],
                        timeout=a.timeout)
            stub_ran.append(ly)
            stub_exits[ly] = c
            if c == 1:
                stub_hits += 1
        extra['orphan_stubs'] = {'ran': not lyr_err, 'layers': stub_ran,
                                 'exits': stub_exits,
                                 'layers_with_orphans': stub_hits}
        if lyr_err:
            extra['orphan_stubs']['error'] = lyr_err
        # 1 is "orphans found"; 0 is clean. ANY OTHER CODE IS THE INSTRUMENT
        # FAILING, not the board passing -- a traceback also exits 1 and an
        # argparse error exits 2, and 2 used to be counted as clean because
        # only `c == 1` was tested. The siblings below use `clean = c == 0`,
        # which fails in the safe direction; this now matches them.
        bad = sorted(ly for ly, c in stub_exits.items() if c not in (0, 1))
        if bad:
            extra['orphan_stubs']['failed_layers'] = bad
        for name, cmd in (
                ('weird_copper', [os.path.join('py_router', 'check_weird.py'),
                                  a.board]),
                ('pad_overlaps', [os.path.join('py_router', 'check_pads.py'),
                                  a.board, '--cross-footprint'])):
            c, o = _run(cmd, timeout=a.timeout)
            extra[name] = {'ran': True, 'exit': c, 'clean': c == 0}
    doc['components'] = extra

    # ---- 2b. copper on another net's pad (#1680) ---------------------------
    # `blocking` counts these with every other DRC item, as a number. A short
    # is the one a reader must not have to go and find, so it is named.
    contacts, contacts_err = [], None
    if layers and not lyr_err:
        try:
            from kicad_parser import parse_kicad_pcb
            from pad_contact import shipped_report
            contacts = shipped_report(parse_kicad_pcb(a.board))
        except Exception as exc:                                # noqa: BLE001
            contacts_err = f'{type(exc).__name__}: {exc}'
    doc['pad_contacts'] = contacts

    # ---- 3. the fab floors this board grades itself against ----------------
    try:
        floors = fab_floor_integrity(a.board, a.authored_from,
                                     timeout=a.timeout)
    except Exception as exc:                                    # noqa: BLE001
        floors = {'ran': False,
                  'reason': f'the floor check itself failed: '
                            f'{type(exc).__name__}: {exc}'}
    doc['fab_floors'] = floors

    # ---- the verdict -------------------------------------------------------
    reasons = []
    if lyr_err:
        reasons.append(f'the board did not parse: {lyr_err}')
    elif not layers:
        reasons.append('this file declares NO copper layers -- it did not '
                       'parse into a board. Every component then grades 0 '
                       'against nothing, which is not a clean board')
    if score_failed:
        reasons.append('board_score produced no score'
                       + (f' (exit {code})' if code == TIMED_OUT else ''))
    if unknown:
        reasons.append('a component RAN and could not answer: '
                       + ', '.join(unknown))
    reasons += [r for r in (ungraded_bad, unknown_bad) if r]
    # #1388: a component nothing asked for is UNEXAMINED and does not block.
    # One the BOARD declares (a diff-pair class with no rule to grade its
    # pairs) is named apart, so the promise nobody checked travels with the
    # verdict -- disclosed, not a blocker.
    notes = []
    _comps = (score.get('components') or {}) if isinstance(score, dict) else {}
    # #1390 / #1490 / #1813: a .kicad_dru rule check_drc cannot model -- a
    # size rule (`undersized`), or a clearance, disallow, via_count, length,
    # skew or diff_pair_uncoupled rule, or one of a kind check_drc has no
    # pass for (`kicad_dru.UNGRADED_KINDS`) (`drc`). It blocks: nothing here
    # graded it. With --kicad-rules (#1810) KiCad's own DRC grades it: a rule
    # KiCad finds met is disclosed and stops blocking, one it finds broken
    # blocks with its count, and one it cannot attribute blocks as before.
    _dru = {}
    _label = {'undersized.dru_rules': '.kicad_dru size rule(s)',
              'drc.dru_rules': '.kicad_dru rule(s)'}
    for _name in _label:
        if _name in ungraded:
            _c = _comps.get(_name.split('.')[0]) or {}
            _kinds = _c.get('ungraded_rule_kinds') or {}
            _dru[_name] = [(r, tuple(_kinds.get(r) or ()))
                           for r in _c.get('ungraded_rules') or ()]
    judged = {}
    if a.kicad_rules and any(_dru.values()):
        try:
            grade = kicad_rule_grade(a.board, timeout=a.timeout)
            dru_names = _dru_rule_names(a.board)
            dru_kinds = _dru_rule_kinds(a.board)
        except Exception as exc:                                # noqa: BLE001
            grade, dru_names = {'error': f'{type(exc).__name__}: {exc}'}, []
            dru_kinds = {}
        judged = judge_rules(sorted({r for v in _dru.values() for r, _k in v}),
                             grade, dru_names, dru_kinds)
        doc['kicad_rules'] = {'measured_by': 'kicad-cli',
                              'kicad_version': grade.get('kicad_version'),
                              'error': grade.get('error'),
                              'capped_types': grade.get('capped_types'),
                              'rules': judged}
    _ver = (doc.get('kicad_rules') or {}).get('kicad_version')

    def _named(rows):
        return ', '.join(repr(r) + (f" ({'/'.join(k)})" if k else '')
                         + (f": {judged[r]['why']}" if r in judged else '')
                         for r, k in rows)

    judged_all = set()      # entries KiCad graded every rule of
    for _name, rows in _dru.items():
        met = [(r, k) for r, k in rows
               if (judged.get(r) or {}).get('verdict') == 'met']
        broken = [(r, k) for r, k in rows
                  if (judged.get(r) or {}).get('verdict') == 'broken']
        rest = [(r, k) for r, k in rows if (r, k) not in met + broken]
        if met:
            notes.append(f'{_label[_name]} check_drc cannot model, judged by '
                         f'KiCad {_ver}: ' + _named(met))
        if broken:
            reasons.append(f'{_label[_name]} check_drc cannot model and KiCad '
                           f'{_ver} finds BROKEN: ' + _named(broken))
        if rest or not rows:
            reasons.append(
                f'{_label[_name]} nothing grades: ' + _named(rest)
                + ' -- the copper they bind is not graded at the rule here and '
                  'KiCad still enforces them'
                + ('' if a.kicad_rules else
                   ' (--kicad-rules asks KiCad\'s own DRC to grade them)'))
        else:
            judged_all.add(_name)
    for _name in ungraded:
        if _name in _dru:
            continue
        if _name == 'diff_pairs.unjoined_pairs':
            # #1629: connected pairs whose skew nothing here measured.
            _uj = (_comps.get('diff_pairs') or {}).get('unjoined_connected') or ()
            reasons.append(
                f'{len(_uj)} diff pair(s) connected but not measured (pads not '
                f'joined by track copper): '
                + ', '.join(f"{r['p_net']} / {r['n_net']}" for r in _uj[:4])
                + (' ...' if len(_uj) > 4 else '')
                + ' -- their skew is ungraded here and KiCad still grades it')
            continue
        _c = _comps.get(_name)
        if isinstance(_c, dict) and _c.get('declared'):
            notes.append(f'{_name}: the board DECLARES it '
                           f'({", ".join(sorted(_c["declared"]))}) and nothing '
                           f'grades it -- {_c.get("reason")}')
    # #1626: what the board's own severities make KiCad warnings, or a check
    # KiCad does not run -- check_drc lists them and does not count them.
    _drc_c = _comps.get('drc') or {}
    if _drc_c.get('severity_warnings'):
        notes.append(f"{_drc_c['severity_warnings']} DRC finding(s) KiCad reports "
                     f"as WARNINGS at the board's own severities (a .kicad_dru "
                     f"rule's (severity warning) or the project's "
                     f"rule_severities) -- check_drc lists them under WARNINGS")
    if _drc_c.get('library_pad_gaps'):
        # #1628: library geometry, not graded (#1306) -- but KiCad's DRC of
        # this board will show them, so a DONE says so
        _unv = _drc_c.get('library_pad_gaps_unverified')
        notes.append(f"KiCad will report "
                     + (f"up to {_drc_c['library_pad_gaps']} pair(s) ({_unv} may be "
                        f"cleared by a .kicad_dru rule check_drc cannot fully read)"
                        if _unv else f"{_drc_c['library_pad_gaps']} pair(s)")
                     + f" of "
                     f"library pads as clearance errors, once per copper layer "
                     f"they share (one footprint's pads closer than their "
                     f"clearance; no route can clear "
                     f"them -- a .kicad_dru rule, a pad/footprint clearance "
                     f"override or a net tie can; check_drc's WARNINGS name "
                     f"the pairs)")
    if _drc_c.get('severity_ignored'):
        notes.append(f"{_drc_c['severity_ignored']} DRC finding(s) in a check the "
                     f"project's rule_severities set to ignore, which KiCad "
                     f"does not run")
    # ONE rule for what a `blocking` is (`converge.blocking_defect`, #1075):
    # `false` is falsy, so it used to read as a clean board and exit DONE.
    from converge import blocking_defect
    _bdef = blocking_defect(blocking)
    if _bdef:
        reasons.append(f'blocking is {_bdef}')
    elif blocking is None and not score_failed:
        reasons.append('blocking is null -- something was not graded, and a '
                       'null is not a zero')
    elif blocking:
        reasons.append(f'blocking = {blocking}')
    if contacts_err:
        reasons.append(f'pad contacts: did not run ({contacts_err})')
    elif contacts:
        from pad_contact import format_rows
        reasons.append(f"{len(contacts)} via(s)/track(s) on another net's pad, "
                       f"a short: " + '; '.join(format_rows(contacts, limit=5)))
    for name, r in sorted(extra.items()):
        if r.get('error'):
            reasons.append(f'{name}: did not run ({r["error"]})')
        elif r.get('failed_layers'):
            reasons.append(f'{name}: the instrument failed on '
                           f'{", ".join(r["failed_layers"])} -- '
                           f'not examined, and not clean')
        elif r.get('layers_with_orphans'):
            reasons.append(f'{name}: {r["layers_with_orphans"]} layer(s) carry '
                           f'orphan stubs')
        elif r.get('clean') is False:
            reasons.append(f'{name}: not clean (exit {r.get("exit")})')

    # An UNMEASURED declared floor is not a pass. It does not make the board
    # UNSOUND on its own -- nothing measured a violation -- but it must not
    # vanish, because `fab_floors.relaxed == []` is exactly the sentence a
    # reader takes for "every declared floor is honoured". Append it to the
    # INCOMPLETE reasons so it travels with the verdict.
    # #1368: floors read against the recorded fab_floor_origin (no
    # --authored-from) are DISCLOSED, not graded: the chain's own descent
    # (escalation, an explicit --track-width / --via-size) lowered them, and
    # says so in its design_rules. --authored-from grades them.
    from_origin = floors.get('source') == 'fab_floor_origin'
    if from_origin and (floors.get('relaxed') or floors.get('unmeasured')):
        _lo = ', '.join(f'{r["label"]} {r["authored"]} -> '
                        + (f'{r["on_board"]} on the board'
                           if r.get('on_board') is not None
                           else f'{r.get("declared")} declared')
                        + _counted(r, paren=True)
                        for r in list(floors.get('relaxed') or ())
                        + list(floors.get('unmeasured') or ()))
        notes.append(f'floors below the recorded fab_floor_origin ({_lo}); '
                     f'confirm the fab supports them, or pass --authored-from '
                     f'to grade them')
    if floors.get('unmeasured') and not from_origin:
        _um = ', '.join(f'{r["label"]} (declared {r["authored"]}'
                        + (f', the project now declares {r["declared"]}'
                           if r.get('declared') is not None else '')
                        + (f'; {r["why"]}' if r.get('why') else '')
                        + ')'
                        for r in floors['unmeasured'])
        reasons.append(f'declared fab floor(s) lowered in the project and NOT '
                       f'measured by this check, so unknown rather than '
                       f'honoured: {_um} -- grade the copper with KiCad\'s DRC '
                       f'at the authored value (check_drc\'s hole arms cover '
                       f'NPTH only)')
    if floors.get('relaxed') and not from_origin:
        worst = ', '.join(f'{r["label"]} {r["authored"]} -> {r["on_board"]}'
                          + _counted(r, paren=True)
                          for r in floors['relaxed'])
        verdict, code_out = 'UNSOUND', UNSOUND
        why = (f'this board carries copper below floors its project once '
               f'declared ({worst}). Every checker grades against the NEW '
               f'value, so a clean report here means the rule moved, not the '
               f'copper. Confirm the fab supports it before calling it done')
    elif reasons:
        verdict, code_out = 'INCOMPLETE', INCOMPLETE
        why = '; '.join(reasons)
    else:
        verdict, code_out = 'DONE', DONE
        why = 'every component that could be graded is graded and clean'

    if notes:
        why += '. DISCLOSED, not counted: ' + '; '.join(notes)
    if floors.get('reference_too'):
        def _ref(r):
            n, rn = _counted(r), _counted(r, 'reference_violations')
            return (f'{r["label"]} {r["authored"]} (this board {r["on_board"]}'
                    f'{", " + n if n else ""}, the reference {r["reference"]}'
                    f'{", " + rn if rn else ""})')
        why += ('. DECLARED, AND BROKEN BY THE REFERENCE TOO, at least as '
                'badly (reported, not counted against this board): '
                + ', '.join(_ref(r) for r in floors['reference_too']))
    # #1810: a rule set KiCad graded is examined (met, or broken and a
    # reason above), not unexamined
    ungraded = [u for u in ungraded if u not in judged_all]
    if not floors.get('ran'):
        # #1368: a floor check that did not run examined nothing. It cannot
        # block (a board nothing ever wrote back has no earlier declaration,
        # and its project IS the spec), but it is named like every other
        # component nothing graded, with WHY it did not run.
        ungraded = list(ungraded) + [
            'fab_floors (' + str(floors.get('reason') or 'did not run') + ')']
    if ungraded:
        # "Not asked" is one reason, not the only one: a --length-groups file
        # whose nets are not routed yet is asked and still cannot measure
        # anything (#1246), and saying "nothing was asked" there is false.
        why += ('. UNEXAMINED, and not passed: ' + ', '.join(ungraded)
                + ' -- nothing graded them (no spec was given, or nothing '
                  'they name is measurable yet), so they are unknown '
                  'rather than clean')
    doc['verdict'] = verdict
    doc['reason'] = why
    doc['ungraded'] = ungraded
    return code_out


def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board')
    ap.add_argument('--authored-from', default=None,
                    help='the board the chain STARTED from -- its project '
                         'carries the floors this one should still respect')
    ap.add_argument('--baseline', default=None, metavar='BOARD',
                    help='the board this one was derived from (the run\'s '
                         'input), handed to board_score: it arms '
                         'check_assembly\'s courtyard gate (#1183). Not '
                         '--authored-from, which reads that board\'s FLOORS')
    ap.add_argument('--clearance', type=float, default=None)
    ap.add_argument('--intent', default=None)
    ap.add_argument('--impedance-nets', nargs='+', action='extend', default=None)
    ap.add_argument('--length-groups', default=None)
    ap.add_argument('--net-min-widths', default=None)
    ap.add_argument('--min-track-width', type=float, default=None)
    ap.add_argument('--min-via-diameter', type=float, default=None)
    ap.add_argument('--min-via-drill', type=float, default=None)
    ap.add_argument('--skip-slow', action='store_true',
                    help='the added instruments only; board_score always runs. '
                         'NOTE this empties `components`, so the document is '
                         'then weaker than the board_score it wraps -- a gate '
                         'reading it should say so')
    ap.add_argument('--kicad-rules', action='store_true',
                    help='a .kicad_dru rule check_drc cannot model is graded by '
                         'KiCad\'s own DRC (one kicad-cli run of the board as '
                         'shipped): 0 error rows naming it reads it met, any '
                         'reads it broken, with the count. Without kicad-cli, '
                         'or when its rows cannot be attributed to the rule, '
                         'the rule stays ungraded (#1810). The verdict then '
                         'depends on the installed KiCad')
    ap.add_argument('--timeout', type=float, default=3600, metavar='S',
                    help='per-instrument wall clock (default 3600). A timeout '
                         'is recorded as exit 124, never raised')
    ap.add_argument('--json', default=None, metavar='PATH')
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)

    if not os.path.isfile(a.board):
        print(f'no such board: {a.board}', file=sys.stderr)
        return BOARD_STATE

    # Absolute, because _run uses cwd=ROOT. A relative board path resolves
    # against ROOT inside the subprocess and against the caller's cwd out here,
    # so from any directory but the repo root board_score was handed a path
    # that does not exist -- and returned an empty score, which became a FALSE
    # `INCOMPLETE: blocking is null`. `doc['board']` was abspathed already, so
    # the document even looked correctly bound. The existing test does
    # os.chdir(ROOT), which is why this never surfaced.
    a.board = os.path.abspath(a.board)
    if a.authored_from:
        a.authored_from = os.path.abspath(a.authored_from)
    if a.baseline:
        a.baseline = os.path.abspath(a.baseline)

    doc = {'schema': 1, 'kind': 'board-complete',
           'board': a.board, 'board_sha': None, 'components': {}, 'score': {},
           'fab_floors': {}, 'ungraded': [],
           'verdict': 'INCOMPLETE',
           'reason': 'the close-out itself did not finish'}

    code_out = INCOMPLETE
    try:
        code_out = _grade(a, doc)
    except Exception as exc:                                    # noqa: BLE001
        # An instrument that dies must not take the document with it. This
        # file's own doctrine -- "say the check did not run rather than
        # implying it passed" -- was written for fab_floor_integrity and never
        # applied to its own failure modes: the --json write used to be the
        # last statement, so anything above it that raised produced no
        # document at all, on precisely the broken boards a gate is for.
        doc['verdict'] = 'INCOMPLETE'
        doc['reason'] = (f'the close-out itself did not finish: '
                         f'{type(exc).__name__}: {exc}')
        code_out = INCOMPLETE
    finally:
        print(f"VERDICT: {doc['verdict']} -- {doc['reason']}")
        if a.json:
            try:
                with open(a.json, 'w', encoding='utf-8') as fh:
                    json.dump(doc, fh, indent=1, sort_keys=True, default=str)
                print(f'  JSON -> {a.json}')
            except OSError as exc:
                print(f'  could not write {a.json}: {exc}', file=sys.stderr)
    return code_out


if __name__ == '__main__':
    import cli_banner
    cli_banner.install()
    sys.exit(main())
