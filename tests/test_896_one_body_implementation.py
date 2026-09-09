"""#896 acceptance 3: no consumer computes a body on its own.

`placement.body` answers "what is this part's body, and where did it come
from". Before it, that question was answered in five places with three
different ladders, and the one most consumers used skipped `.Fab` entirely --
which is the whole defect.

WHAT THIS GATE CAN AND CANNOT SEE
---------------------------------
A body ladder has no single spelling, so this does not grep for one. It greps
for the two functions a ladder is BUILT from -- `compute_footprint_bbox_local`
and the `placement.parser.extract_*` readers -- exactly as
`tests/test_878_side_rule_sites.py` anchors on `side_of_layer`. Every call site
outside the model and its parser must be DECLARED here with a reason, and the
declaration is COUNTED, so deleting one of two calls inside a function is a
finding rather than a silent pass.

It cannot see a ladder written from raw regexes, and it says so
(`_UNMATCHABLE`) rather than leaving a reader to assume coverage it does not
have.

WHY SO MANY SITES ARE DECLARED RATHER THAN CONVERTED
----------------------------------------------------
Some of them are not asking the body question at all, and converting them
would be a behaviour change wearing a cleanup's clothes. Each declaration
below carries the reason and, where one exists, the issue that owns it. A
declaration is a decision on the record, not an exemption: the point is that a
NEW site cannot appear without someone writing down which of these it is.
"""
import ast
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'),
                os.path.join(ROOT, 'py_placer')]

RUN_ALL_TIMEOUT = 300
RUN_ALL_FAST_OK = True

FAILURES = []

#: Production trees only. `tests/` is excluded deliberately: a test may spell a
#: ladder in order to check one, and `test_896_body_model.py` does.
_TREES = ('py_placer', 'py_router', 'py_tools', 'kicad_routing_plugin')

#: THE model and its reader. Everything else is a consumer.
_HOME = (os.path.join('py_placer', 'placement', 'body.py'),
         os.path.join('py_placer', 'placement', 'parser.py'),
         os.path.join('py_placer', 'placement', 'utility.py'))

#: What a body ladder is built from. Not the ladder itself -- see the module
#: docstring for why this is the anchor.
_CALLS = re.compile(
    r'\b(compute_footprint_bbox_local|extract_courtyard_sides'
    r'|extract_courtyard_bboxes|extract_fab_sides|extract_silk_sides'
    r'|courtyard_for_side)\b')

#: Spellings this scanner CANNOT see, named in source rather than left to be
#: discovered. A ladder built from `re.finditer` over the board text, or from
#: `pcbnew`'s `GetCourtyard()`, would not match `_CALLS`.
_UNMATCHABLE = (
    'a hand-rolled regex over the board text (what wk/run25/probe_housings.py '
    'was, and what #896 exists to make unnecessary)',
    "pcbnew's GetCourtyard()/GetPolyCourtyard(), which nothing in the tree "
    'calls today',
    'a ladder reached through a variable holding one of these functions',
)

#: Conversely, what the scanner deliberately does NOT count, because counting
#: it would make the gate satisfiable by prose: a mention inside a comment or a
#: string. `routability.LaneContext.graded` names `extract_courtyard_sides` in
#: its docstring and calls it nowhere; `board_brief.SOURCES_NOTE` names two of
#: these functions in a string. Both were false positives of a line-matching
#: first draft, and this repo has already been bitten by the inverse -- a
#: comment quoting code that satisfied a source-grep test while the code was
#: gone.

#: {(relpath, enclosing qualname): (count, why it is not converted)}
#:
#: MEASURED, not remembered. The first draft of this table was written from
#: memory and the gate rejected 7 of its 15 rows as invented qualnames and 2
#: more as wrong counts, which is the table doing its job on its own author.
_DECLARED = {
    # -- goes through the model; the call here is not a ladder.
    (os.path.join('py_placer', 'placement', 'legality.py'),
     'part_local_bounds'): (1, 'the +/-0.5mm fiction for a part the model '
                            'reports as SOURCE_NONE; the ladder itself is '
                            'body.board_bodies'),
    (os.path.join('py_tools', 'board_brief.py'),
     'parts_section'): (1, 'fallback only, when the model returns no box; '
                        'the extent itself is body.occupancy_local'),

    # -- the SEARCH ladder, deferred deliberately. The biggest declaration
    #    here, and the one a follow-up issue owns.
    (os.path.join('py_placer', 'placement', 'quench.py'),
     '_Part.__init__'): (
        2, 'the SEARCH ladder, now the OFF ARM of `body_model` (#916). '
        '`placement.body` supplies `occupancy_local` when the flag is armed; '
        'these two calls remain as the unarmed path, deliberately, so the '
        'A/B control arm is the OLD CODE rather than a re-derivation that '
        'happens to agree. NOTE FOR WHOEVER FLIPS THE DEFAULT: this census '
        'compares COUNTS and PRESENCE, never this sentence, so it cannot tell '
        'you the reason has gone stale -- update it by hand when the default '
        'moves. Adopting the model changes which poses `seeder.pose_ok` '
        'admits (it reads these baked bounds) and so the basin the anneal '
        'lands in: an engine change needing its own A/B, not a ride-along on '
        'a reporting one. Measured at #896: it grows 23 of 1349 corpus parts '
        'and shrinks none, on 4 of 22 boards. Re-measured at #916 on '
        'esp_prog: 5 parts grow, 0 shrink, largest U2 5.707 -> 27.04 mm2.'),
    (os.path.join('py_placer', 'placement', 'quench.py'),
     'QuenchState.__init__'): (1, 'builds the courtyard map the _Part OFF arm '
                               'consumes; the armed path reads '
                               'placement.body.board_bodies beside it'),
    (os.path.join('py_placer', 'placement', 'quench.py'),
     'QuenchState.fab_rect'): (1, 'the fab body for the quench containment '
                               'test, on the same deferred ladder'),

    # -- a DIFFERENT question, where converting would change a measured
    #    currency or reason in a circle.
    (os.path.join('py_placer', 'placement', 'options.py'),
     'grow_board'): (2, '#878 far-face AREA currency, which reads the '
                     'side-BLIND extract_courtyard_bboxes on purpose: a '
                     "through-hole part's leads block the far face too. "
                     'Converting it changes the currency and forces a #878 '
                     're-record.'),
    (os.path.join('py_placer', 'placement', 'labels.py'),
     '_world_courtyard'): (1, 'silkscreen LABEL placement. A silk-derived '
                           'body feeding a silk-placement algorithm is '
                           'circular, and its fallback is a 1mm box rather '
                           "than the model's fiction."),
    (os.path.join('py_placer', 'placement', 'labels.py'),
     'beautify_labels'): (1, 'reads the courtyard map _world_courtyard '
                          'consumes; same argument'),
    (os.path.join('py_placer', 'placement', 'fanout_clearance.py'),
     '_Repair.__init__'): (3, 'the #313 via-nudge cap model, which prices its '
                           'own clearance ceiling (#768/#769) and is the one '
                           'PLACEMENT step that lays copper; a separate '
                           'question from what a part body is'),

    # -- convertible, and simply not in this PR. Each is a candidate for the
    #    follow-up, and saying so is the point: a reader can tell a decision
    #    from a leftover.
    (os.path.join('py_placer', 'placement', 'placement_state.py'),
     '_global_rect'): (2, 'a third copy of courtyard-or-pad-bbox; '
                       'behaviour-preserving to convert, not in this PR'),
    (os.path.join('py_placer', 'placement', 'placement_state.py'),
     '_outside_fraction'): (1, 'builds the courtyard map _global_rect '
                            'consumes'),
    (os.path.join('py_placer', 'placement', 'lock_advisor.py'),
     'advise_locks'): (1, 'pose-plausibility geometry; convertible, not in '
                       "this PR's scope"),
    (os.path.join('py_tools', 'check_pockets.py'),
     'pocket_census'): (3, 'the pocket census; convertible, not in this PR'),
}


def check(name, cond, detail=''):
    if cond:
        print(f'  PASS: {name}')
    else:
        FAILURES.append(f'{name}{(" -- " + detail) if detail else ""}')
        print(f'  FAIL: {name} -- {detail}')


def _code_names(src):
    """[(lineno, NAME token)] with comments, strings and imports removed.

    Returns None when the file cannot be tokenized, which is reported rather
    than skipped: a file the scanner could not read is not a file it checked.
    """
    import io as _io
    import tokenize
    out = []
    try:
        toks = list(tokenize.generate_tokens(
            _io.StringIO(src).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return None
    logical = []
    line_start = True
    skip_line = None
    for tok in toks:
        if tok.type in (tokenize.NEWLINE, tokenize.NL):
            line_start = True
            skip_line = None
            continue
        if tok.type in (tokenize.COMMENT, tokenize.STRING,
                        tokenize.INDENT, tokenize.DEDENT):
            continue
        if line_start and tok.type == tokenize.NAME and                 tok.string in ('import', 'from'):
            # An import names the function without calling it.
            skip_line = tok.start[0]
        line_start = False
        if skip_line is not None:
            continue
        if tok.type == tokenize.NAME:
            out.append((tok.start[0], tok.string))
    del logical
    return out


def _qualnames(src):
    """line number -> enclosing def/class qualname."""
    out = {}
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return None

    def walk(node, prefix):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef,
                                  ast.ClassDef)):
                name = f'{prefix}.{child.name}' if prefix else child.name
                lo = child.lineno
                hi = getattr(child, 'end_lineno', lo)
                for ln in range(lo, hi + 1):
                    out[ln] = name
                walk(child, name)
    walk(tree, '')
    return out


def scan():
    """[(relpath, qualname, count)] over the production trees."""
    found = {}
    files = 0
    broken = []
    for tree in _TREES:
        base = os.path.join(ROOT, tree)
        for root, _dirs, names in os.walk(base):
            if '__pycache__' in root:
                continue
            for fn in names:
                if not fn.endswith('.py'):
                    continue
                path = os.path.join(root, fn)
                rel = os.path.relpath(path, ROOT)
                files += 1
                if rel in _HOME:
                    continue
                with open(path, encoding='utf-8', errors='replace') as fh:
                    src = fh.read()
                if not _CALLS.search(src):
                    continue
                qn = _qualnames(src)
                if qn is None:
                    broken.append(f'UNPARSEABLE: {rel}')
                    continue
                # TOKENIZED, not line-matched. A comment or a docstring
                # that NAMES one of these functions is not a call, and a
                # grep that counts it is satisfiable by prose -- this repo
                # has been bitten by exactly that (a comment quoting code
                # made a source-grep test pass while the code was gone).
                # `board_brief.SOURCES_NOTE` names two of them in a string.
                names = _code_names(src)
                if names is None:
                    broken.append(f'UNTOKENIZABLE: {rel}')
                    continue
                for i, name in names:
                    if not _CALLS.fullmatch(name):
                        continue
                    key = (rel, qn.get(i, '<module>'))
                    found[key] = found.get(key, 0) + 1
    return found, files, broken


def main():
    found, files, broken = scan()
    print(f'--- scanned {files} production file(s), '
          f'{len(found)} declared site(s) expected')
    for b in broken:
        check(b, False, 'a file the scanner could not parse is not a file it '
                        'checked')

    # Not vacuous. A scanner that finds nothing passes every assertion below
    # it, and the two ways that happens -- a broken regex and a wrong tree
    # list -- both look exactly like success.
    check('the scan is not vacuous', files >= 100 and len(found) >= 10,
          f'{files} files, {len(found)} sites')

    undeclared = sorted(k for k in found if k not in _DECLARED)
    check('no UNDECLARED body-ladder site', undeclared == [],
          'a new site must be converted to placement.body or declared here '
          f'with its reason: {undeclared}')

    stale = sorted(k for k in _DECLARED if k not in found)
    check('no STALE declaration', stale == [],
          f'declared but not found -- renamed or removed: {stale}')

    miscount = sorted((k, found[k], _DECLARED[k][0])
                      for k in found if k in _DECLARED
                      and found[k] != _DECLARED[k][0])
    check('every declaration matches its COUNT', miscount == [],
          f'(site, found, declared): {miscount}')

    check('the unmatchable spellings are named in source',
          len(_UNMATCHABLE) >= 3)

    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #896 one body "
          f"implementation, {len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f'  - {f}')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
