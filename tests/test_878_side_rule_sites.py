#!/usr/bin/env python3
"""#878: one side rule, and a gate that refuses a ninth copy of it.

'F' or 'B' from a layer name is a one-line collapse, which is exactly why the
tree grew eight independent spellings of it. `legality.side_of_layer` is the
rule and `legality.footprint_side` is it applied to an object; every site whose
input is a LAYER NAME now calls one of them. Not all of them are footprint
layers -- `_seg_side` takes a track segment's and `label_side` a silkscreen
label's -- which is exactly why the shared primitive is `side_of_layer(layer)`
and `footprint_side(fp)` is the thin wrapper, rather than the other way round.
It also means the corpus arm below, which feeds only FOOTPRINT layers, does not
exercise those two callers' inputs.

This file holds two things that fail for different reasons:

  1  THE IDENTITY PROOF. Over every footprint on every tracked board, the
     canonical rule and the five literal expressions it replaced must agree.
     That is the direct evidence the conversion is a no-op -- stronger than
     running a placement suite, because it covers every footprint rather than
     the handful four boards happen to exercise. It carries its own vacuity
     floor: a run that saw no footprints, or only one distinct layer value,
     proves nothing and says so.

  2  THE REGISTRY. Every surviving `startswith('B'...)` in production source is
     declared here with the reason it is not the canonical call, keyed by
     (file, enclosing function) and counted. Held in BOTH directions: an
     undeclared site fails, and so does a declaration that matches nothing or
     matches a different number of sites. A one-directional registry is how the
     #696 containment guard passed 28/28 while the thing it named had moved.

     Keyed by ENCLOSING FUNCTION, never by line number -- a registry that
     re-flows when someone inserts a blank line is a registry that gets
     deleted. COUNTED as well as keyed, so that removing one of several sites
     inside a function is a finding rather than a silent pass; every entry
     happens to be 1 today, and the count is what keeps that a measurement.

WHY NOT IN `test_718_static_test_hygiene.py`: that file's subject is `tests/`,
and every scanner in it takes `_py_files(only_tests=True)`. This one scans
production source. Widening test_718's stated subject to carry it would blur
the edge that file is careful about.
"""
import ast
import io
import os
import re
import sys

RUN_ALL_TIMEOUT = 600

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
for _p in (_HERE, ROOT, os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                             # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement.legality import footprint_side, side_of_layer  # noqa: E402

#: Production trees only. `tests/` is excluded because a test may spell the
#: collapse in order to CHECK it -- this file does, twice.
_TREES = ('py_placer', 'py_router', 'py_tools', 'kicad_routing_plugin')

#: The tuple form `startswith(('B', '*'))` is NOT optional to match: it is live
#: at `legality.PartPads.__init__`, one line below a plain form, and the first
#: draft of this scanner saw only the plain one -- so that function's entry read
#: `(1, ...)` while it holds two sites, and deleting or inverting the tuple one
#: would have been a silent pass. That is precisely the failure the count is
#: here to prevent, reproduced inside the guard meant to prevent it.
_COLLAPSE = re.compile(r"""startswith\(\s*\(?\s*['"]B""")

#: Spellings this scanner CANNOT see, named rather than left to be discovered.
#: A registry that quietly misses a form is worse than none, because it reads
#: as coverage. `layer[0] == 'B'`, `layer[:1] == 'B'`, `re.match(r'^B', layer)`,
#: a named-constant prefix, and a call wrapped across lines so no single line
#: carries the match. The line-wrapped case FAILS OPEN -- the file-level search
#: matches, the per-line loop does not, and the site is recorded nowhere -- so
#: the check below counts file-level hits against per-line hits and refuses
#: when they disagree.
_UNMATCHABLE_SPELLINGS = (
    "layer[0] == 'B'", "layer[:1] == 'B'", "re.match(r'^B', layer)",
    'startswith(_BACK_PREFIX)', 'a call split across lines')

#: (relpath, enclosing qualname) -> (count, why it is not the canonical call).
#:
#: Everything here is a DECISION with a reason, not a backlog. The two that
#: would be behaviour changes are named as such and left for their own issue.
_DECLARED = {
    ('py_placer/placement/legality.py', 'side_of_layer'):
        (1, 'THE rule. Everything else in this map is measured against it.'),
    ('py_placer/placement/legality.py', 'PartPads.__init__'):
        (2, 'a 3-VALUED PAD rule (None = through/dual-face), hardened for '
            '*.Cu and F+B pads (#834). Different arity, different question. '
            'TWO sites: the plain `startswith(\'B\')` and, one line below, the '
            'tuple form `startswith((\'B\', \'*\'))` -- which the first draft '
            'of this scanner could not see, so this entry read 1 and half of '
            'the rule was unguarded.'),
    ('py_placer/placement/fanout_clearance.py', '_Repair.__init__'):
        (1, 'the UNHARDENED twin of the legality pad rule above -- a real '
            'disagreement on *.Cu and dual-face pads, and the exact mutant '
            'body of the KILLED row `pside-goes-back-to-back-only` in '
            'tests/mutate_834_835.py. Unifying it is a behaviour change no '
            'corpus board can witness, so it needs its own issue, not a '
            'ride-along on #878.'),
    ('py_placer/placement/quench.py', 'QuenchState.fab_rect'):
        (1, 'reads an already-resolved `p.side`, not a layer, and adds a '
            'defensive .upper(). `side_of_layer` has none, so converting '
            'would silently drop the hardening.'),
    ('py_router/movie_camera.py', '_moved_side'):
        (1, 'py_router. Importing placement.legality here would invert the '
            'layer graph -- py_placer imports py_router, not the reverse.'),
    ('py_router/kicad_parser.py', 'flip_layer_token'):
        (1, "a layer-NAME transform on 'B.' WITH the dot, not a side rule; "
            'and py_router again.'),
    ('kicad_routing_plugin/placement_gui.py', 'PlacementTab._pose_moves'):
        (1, 'runs inside KiCad pcbnew, where py_placer is imported only '
            'lazily inside try blocks and is not guaranteed importable. Its '
            'or-F.Cu default is behaviour-identical to side_of_layer.'),
    # main spells this site `PlacementTab._apply_pose`. The IPC port folded
    # that method into `_apply_ipc` -- kipy has no `Flip()`, so applying a
    # pose and applying the copper are one commit here. Same single
    # expression, same reason.
    ('kicad_routing_plugin/placement_gui.py', 'PlacementTab._apply_ipc'):
        (1, 'same as _pose_moves above -- the GUI half. placement_gui reaches '
            'py_placer only lazily, inside the functions that need it '
            '(`from placement.labels import ...`), and nothing imports it at '
            'module scope, so the canonical rule is not guaranteed importable '
            'where this expression runs.'),
}

_fail = []
_ran = []


def check(label, ok, detail=''):
    print(('  ok  ' if ok else '  BAD ') + label
          + (('  -- ' + detail) if detail and not ok else ''))
    _ran.append(label)
    if not ok:
        _fail.append(label)


def _qualname_map(tree):
    """line -> enclosing qualname, for every line inside a def/class."""
    out = {}

    def walk(node, prefix):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef,
                                  ast.ClassDef)):
                name = prefix + child.name
                for ln in range(child.lineno, (child.end_lineno or
                                               child.lineno) + 1):
                    out[ln] = name
                walk(child, name + '.')
            else:
                walk(child, prefix)
    walk(tree, '')
    return out


def _scan():
    found, files = {}, 0
    for tree in _TREES:
        base = os.path.join(ROOT, tree)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames if d != '__pycache__']
            for fn in sorted(filenames):
                if not fn.endswith('.py'):
                    continue
                path = os.path.join(dirpath, fn)
                rel = os.path.relpath(path, ROOT).replace(os.sep, '/')
                src = io.open(path, encoding='utf-8', errors='replace').read()
                files += 1
                if not _COLLAPSE.search(src):
                    continue
                try:
                    qn = _qualname_map(ast.parse(src))
                except SyntaxError as e:
                    # REPORTED, never skipped: a file that will not parse is
                    # a file this gate is blind to.
                    found[(rel, 'UNPARSEABLE: %s' % e.msg)] = 1
                    continue
                per_line = 0
                for i, line in enumerate(src.splitlines(), 1):
                    for _m in _COLLAPSE.finditer(line):
                        per_line += 1
                        key = (rel, qn.get(i, '<module>'))
                        found[key] = found.get(key, 0) + 1
                # FAIL OPEN GUARD. `\s*` spans newlines, so a call wrapped
                # across lines matches the file and no line -- the site would
                # be recorded nowhere and the undeclared check would pass. If
                # the two counts disagree, say so instead of reporting clean.
                whole = len(_COLLAPSE.findall(src))
                if whole != per_line:
                    found[(rel, 'SPLIT ACROSS LINES: %d file-level match(es), '
                                '%d on any single line' % (whole, per_line))] = 1
    return found, files


def test_the_side_rule_has_one_home():
    found, files = _scan()
    check('the scanner is not vacuous', files >= 100 and bool(found),
          '%d file(s), %d site(s)' % (files, len(found)))
    undeclared = {k: v for k, v in found.items() if k not in _DECLARED}
    check('every side-rule site in production is the canonical call or '
          'declared here', not undeclared,
          'undeclared: %s -- call legality.footprint_side / side_of_layer, '
          'or declare it with its reason' % sorted(undeclared))
    stale = [k for k in _DECLARED if k not in found]
    check('and no declaration names a site that has gone', not stale,
          'stale: %s' % sorted(stale))
    miscounted = [(k, found[k], _DECLARED[k][0]) for k in _DECLARED
                  if k in found and found[k] != _DECLARED[k][0]]
    check('and each declaration still covers the number of sites it claims',
          not miscounted, 'moved: %s' % miscounted)


def test_the_canonical_rule_agrees_with_every_form_it_replaced():
    boards = run_utils.corpus_boards()
    if not boards:
        print('SKIP: git could not name the tracked corpus')
        return
    seen, layers, bad = 0, set(), []
    for p in boards:
        try:
            pcb = parse_kicad_pcb(p)
        except Exception:                                    # noqa: BLE001
            continue
        for ref, fp in (pcb.footprints or {}).items():
            seen += 1
            layers.add(fp.layer)
            want = footprint_side(fp)
            # NOT `side_of_layer(fp.layer)` as a fourth row: that is the
            # definition of `footprint_side`, so it could never disagree and
            # would pad the count with a tautology.
            forms = {
                'fanout/render/movie': 'B' if (fp.layer or '').startswith('B') else 'F',
                'labels': 'B' if str(fp.layer).startswith('B') else 'F',
                'gui': 'B' if str(fp.layer or 'F.Cu').startswith('B') else 'F',
            }
            for name, got in forms.items():
                if got != want:
                    bad.append((os.path.basename(p), ref, fp.layer, name,
                                got, want))
    check('every replaced expression agrees with the canonical rule',
          not bad, '%d disagreement(s): %s' % (len(bad), bad[:4]))
    # WHAT THE CORPUS CANNOT SHOW, stated so the arm above is not read as
    # more than it is. On a `str` or `None` layer all these spellings are
    # ALGEBRAICALLY equal, and the corpus carries only `F.Cu` and `B.Cu` --
    # so "0 disagreements over 1349 footprints" could not have come out any
    # other way. It is a change detector for `side_of_layer`, not evidence
    # that the forms differ. The inputs that DO separate them cannot come off
    # a board, so they are supplied here.
    class _Odd:                       # a truthy NON-string layer
        def __init__(self, v):
            self.v = v

        def startswith(self, p):      # duck-types the old `.startswith` path
            return str(self.v).startswith(p)

        def __str__(self):
            return str(self.v)

    edge = []
    for layer, why in ((None, 'absent'), ('', 'empty'), (0, 'falsy non-str')):
        # `str(x or 'F.Cu')` -- the GUI form -- substitutes a default BEFORE
        # the test. On a falsy layer every form still answers 'F', which is
        # what makes leaving the GUI sites unconverted defensible; assert it
        # rather than claiming it in a comment.
        got = {'canonical': side_of_layer(layer),
               'gui': 'B' if str(layer or 'F.Cu').startswith('B') else 'F',
               'labels': 'B' if str(layer).startswith('B') else 'F'}
        if len(set(got.values())) != 1 or got['canonical'] != 'F':
            edge.append((why, got))
    check('a falsy layer reads FRONT under every surviving spelling '
          '(the corpus has none, so this is the only place it is checked)',
          not edge, str(edge))
    # And the one real behaviour difference the widening introduced: the old
    # `(x or '')` form raises on a truthy non-string, where `str(x or '')`
    # answers. Pinned so "a widening, not a change" stays a measured claim.
    odd = _Odd('B.Cu')
    old_raises = False
    try:
        ('B' if (odd or '').startswith('B') else 'F')
    except AttributeError:
        old_raises = True
    check('the widening only changes a truthy NON-string layer, and there it '
          'answers instead of raising',
          side_of_layer(odd) == 'B' and not old_raises,
          'side_of_layer=%r old_raises=%r' % (side_of_layer(odd), old_raises))
    # The vacuity floor. A corpus that shows one layer value cannot
    # distinguish these forms at all, and a green tick on it would be a lie.
    check('the proof is not vacuous (enough footprints, >1 distinct layer)',
          seen >= 1200 and len(layers) >= 2,
          '%d footprint(s), layers %s' % (seen, sorted(map(str, layers))))
    print('    %d footprints, layer values %s'
          % (seen, sorted(map(str, layers))))


def test_no_part_obstructs_more_on_the_far_face_than_the_near_one():
    """The unstated invariant that keeps #878's far charge safe.

    `through_pad_bounds_local` takes `max(drill_radius, projected half pad)`
    per drilled pad; the near-charge fallback `compute_footprint_bbox_local`
    has NO drill term. So a courtyard-less part whose drill exceeds its pad
    copper -- an NPTH mounting hole with a mask-sized `size`, a slot -- can in
    principle present MORE on the far face than on its own.

    That would matter: on a board populated only on F, `sum(far) <= sum(near)`
    is what stops `obstructed['B.Cu']` -- pure lead area on a face nobody
    builds -- from becoming `busiest` and driving `fits_by_area`,
    `shortfall_mm2_at_least` and the proposed board size.

    Not witnessed on the corpus, which is why it is asserted rather than
    described: an invariant nothing checks is one nobody notices breaking.
    """
    from placement import options as O
    from placement.legality import footprint_has_through_pads
    from placement.parser import extract_courtyard_bboxes
    from placement.utility import compute_footprint_bbox_local
    from placement.legality import rotate_local_bounds
    boards = run_utils.corpus_boards()
    if not boards:
        print('SKIP: git could not name the tracked corpus')
        return
    worse, seen = [], 0
    for p in boards:
        try:
            pcb = parse_kicad_pcb(p)
            cy = extract_courtyard_bboxes(p) or {}
        except Exception:                                    # noqa: BLE001
            continue
        for ref, fp in (pcb.footprints or {}).items():
            if not footprint_has_through_pads(fp):
                continue
            box = cy.get(ref) or (compute_footprint_bbox_local(fp)
                                  if fp.pads else None)
            if box is None:
                continue
            gx0, gy0, gx1, gy1 = rotate_local_bounds(*box, fp.rotation or 0.0)
            near = (gx1 - gx0 + 0.2) * (gy1 - gy0 + 0.2)
            far = O._far_face_area(fp, 0.2)
            seen += 1
            if far > near + 1e-9:
                worse.append((os.path.basename(p), ref, round(near, 2),
                              round(far, 2)))
    check('no drilled part presents more on the far face than on its own',
          not worse, '%d part(s): %s' % (len(worse), worse[:4]))
    check('and that was checked against a real population',
          seen >= 200, '%d drilled part(s) seen' % seen)


TESTS = [test_the_side_rule_has_one_home,
         test_the_canonical_rule_agrees_with_every_form_it_replaced,
         test_no_part_obstructs_more_on_the_far_face_than_the_near_one]


def main():
    for t in TESTS:
        print('--- %s' % t.__name__)
        t()
    print('\n%s: %d check(s), %d failed'
          % ('FAIL' if _fail else 'PASS', len(_ran), len(_fail)))
    return 1 if _fail else 0


if __name__ == '__main__':
    sys.exit(main())
