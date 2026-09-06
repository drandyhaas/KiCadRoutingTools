#!/usr/bin/env python3
"""#878: the far-face currency measurement, RE-DERIVED rather than read back.

`tests/878_far_face_currency.json` is the committed argument for which rect a
through-hole part presents on the far face. This is the gate that keeps it from
rotting, and it re-runs the whole sweep (seconds, not minutes) rather than reading its
conclusions -- `tests/test_803_calibration_claims.py:10-15`'s rule: a regen test
must RE-DERIVE the aggregate, not read a constant. Five of seven checks in the
case that motivated that rule read constants, and a 3 mm cap passed green while
the headline halved.

Four things are pinned, and they fail for different reasons on purpose:

  1  THE NOTARY. `measure_878_far_face_area.PREREGISTRATION` must equal the
     copy inside the committed JSON, exactly. The committed file is the
     notarised copy and the module is the live one, so moving a threshold
     after seeing the numbers turns this red and names the key.

     That only means something because `--out` REFUSES to overwrite a
     pre-registration it disagrees with. Otherwise the fix printed in this
     file's own failure message ("re-record with --out") would repair a moved
     threshold as a side effect of re-recording numbers, and the block would
     be a comment with extra steps.
  2  THE CONTROLS. NC1 (the engine reproduces the arm it is meant to
     implement, on every field per board/basis) and NC2 (a board with no
     drilled pad is identical under all three currencies) must both be clean
     on the FRESH run. A sweep whose control did not reproduce decided
     nothing. The FIELD COUNT is compared against the committed one, not
     against `len(NC1_FIELDS)` -- the fresh document is built from that same
     constant, so comparing the two was `len(x) == len(x)`, and it let the
     count go 8 -> 9 across a commit with nothing reacting.
  3  THE VERDICT AND WHICH FALSIFIER PRODUCED IT. Not just `selected` --
     `bound_by` too. An answer that stayed the same because a different
     criterion started binding is a changed finding wearing the old label,
     which is exactly how #694's inverted row kept reading as intact.
  4  THE STRUCTURAL CLAIMS, which hold by construction and not by measurement,
     so they are the gate's own negative control: `flat_hierarchy`'s
     far/near under `courtyard` is 1.0 (64 of 64 pad-bearing parts drilled, so
     the far charge IS the near charge part for part -- the F1 witness), and
     the one-face sum is each part exactly once under `none` alone.

Sets are compared as sets (a board joining or leaving a flip set is a finding);
counts and floats are compared in bands, because a board edit or a courtyard
fix may legitimately move a cell without changing what the sweep concluded.
"""
import json
import os
import sys

RUN_ALL_TIMEOUT = 600

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
for _p in (_HERE, ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import measure_878_far_face_area as M                        # noqa: E402

#: Named in LIVE code, not in prose: `test_718._names_in_live_code` refuses a
#: baseline registration whose gate mentions the file only in a docstring,
#: because three registrations did exactly that.
BASELINE = os.path.join(_HERE, '878_far_face_currency.json')

_fail = []
_ran = []


def check(label, ok, detail=''):
    print(('  ok  ' if ok else '  BAD ') + label + (('  -- ' + detail)
                                                    if detail and not ok
                                                    else ''))
    _ran.append(label)
    if not ok:
        _fail.append(label)


def main():
    if not os.path.isfile(BASELINE):
        print('BAD: the committed baseline is missing: %s' % BASELINE)
        print('A missing baseline FAILS rather than passing -- regenerate it '
              'with `measure_878_far_face_area.py --out`.')
        return 1
    with open(BASELINE, encoding='utf-8') as f:
        base = json.load(f)

    paths = M.boards()
    if paths is None:
        print('SKIP: git could not name the tracked corpus')
        return M.SKIP_EXIT
    doc, rows, failed = M.build(paths)
    if failed or not rows:
        print('BAD: %d board(s) did not measure (%s) -- a gate whose INPUT is '
              'missing tests nothing.' % (len(failed), ', '.join(failed[:6])))
        return 1
    fresh = M.compact(doc)

    # 1 -- the notary.
    check('the pre-registration is unchanged since it was notarised',
          M.PREREGISTRATION == base['preregistration'],
          'keys differing: %s' % sorted(
              k for k in set(M.PREREGISTRATION) | set(base['preregistration'])
              if M.PREREGISTRATION.get(k) != base['preregistration'].get(k)))

    # 2 -- the controls, on the FRESH run.
    c = fresh['control']
    check('NC1: the engine still reproduces the arm it is meant to implement '
          '(currency %r)' % M.ENGINE_CURRENCY,
          not c['nc1_mismatches'], str(c['nc1_mismatches'][:2]))
    # Against the COMMITTED count, not against `len(M.NC1_FIELDS)`. The fresh
    # doc sets that key FROM `len(NC1_FIELDS)`, so comparing the two was
    # `len(x) == len(x)` -- unconditionally true, and it let the field count
    # go 8 -> 9 across a commit with nothing reacting.
    check('NC1 still compares the same number of fields it was recorded with',
          c['nc1_fields_per_row'] == base['control']['nc1_fields_per_row'],
          '%d now vs %d recorded -- if a field was added on purpose, '
          're-record' % (c['nc1_fields_per_row'],
                         base['control']['nc1_fields_per_row']))
    check('NC1 is not vacuous (it compared some rows)',
          c['nc1_rows'] >= 30 and c['nc1_fields_per_row'] >= 8,
          '%d rows x %d fields' % (c['nc1_rows'], c['nc1_fields_per_row']))
    check('NC2: a board with no drilled pad is identical across currencies',
          not c['nc2_mismatches'], str(c['nc2_mismatches'][:2]))
    check('NC2 is not vacuous (some board carries no drilled pad)',
          len(c['nc2_boards']) >= 3, str(c['nc2_boards']))

    # 3 -- the verdict AND the falsifier that produced it.
    check('the selected currency is unchanged',
          fresh['verdict']['selected'] == base['verdict']['selected'],
          '%r vs %r' % (fresh['verdict']['selected'],
                        base['verdict']['selected']))
    check('and the falsifier that produced it is unchanged',
          fresh['verdict']['bound_by'] == base['verdict']['bound_by'],
          '%r vs %r' % (fresh['verdict']['bound_by'],
                        base['verdict']['bound_by']))
    check('F1 still rejects exactly what it rejected',
          fresh['verdict']['f1_rejected'] == base['verdict']['f1_rejected'],
          '%r vs %r' % (fresh['verdict']['f1_rejected'],
                        base['verdict']['f1_rejected']))

    # 4 -- structural claims: true by construction, so they are this gate's
    # own negative control. If either of these ever fails, the sweep is not
    # measuring what its comments say and no other check here means anything.
    fh = fresh['boards'].get('flat_hierarchy', {}).get('arms', {})
    fh_arm = (fh.get('one_face_observed') or {}).get('courtyard', {})
    check('the F1 witness still holds by construction '
          '(flat_hierarchy far/near == 1.0 under `courtyard`)',
          abs(fh_arm.get('far_over_near', 0.0) - 1.0) < 1e-9,
          'got %r' % fh_arm.get('far_over_near'))
    ph = fresh['post_hoc']['one_face_charged_once']
    check('the one-face sum is each part ONCE only under `none`',
          ph['none']['charged_once']
          and not ph['tht']['charged_once']
          and not ph['courtyard']['charged_once'],
          str({k: v['charged_once'] for k, v in ph.items()}))

    # the flip sets, as SETS -- a board joining or leaving is a finding.
    for arm in sorted(base['headline']['flips']):
        a = {r[0] for r in fresh['headline']['flips'].get(arm, [])}
        b = {r[0] for r in base['headline']['flips'][arm]}
        check('flip set unchanged for %s' % arm, a == b,
              'gained %s, lost %s' % (sorted(a - b), sorted(b - a)))

    # the confound census: the `courtyard` arm is only clean while this is 0.
    check('the both-sides courtyard union still confounds nothing',
          fresh['confound']['courtyard_sides_differ'] == 0,
          str(fresh['confound']))

    # move counts in a band, not exactly: a board edit may legitimately move one.
    for arm in sorted(base['headline']['moves']):
        a = fresh['headline']['moves'].get(arm, {}).get('n', -1)
        b = base['headline']['moves'][arm]['n']
        check('move count within 1 for %s (%d vs %d)' % (arm, a, b),
              abs(a - b) <= 1)

    print('\n%s: %d check(s), %d failed'
          % ('FAIL' if _fail else 'PASS', len(_ran), len(_fail)))
    if _fail:
        print('failing: %s' % _fail)
        print('\nIf the ENGINE changed on purpose, re-record with:')
        print('  python3 -X utf8 tests/measure_878_far_face_area.py '
              '--out tests/878_far_face_currency.json')
        print('in the SAME commit as the change -- a stale baseline published '
              'as somebody else\'s effect is a recorded failure here.')
    return 1 if _fail else 0


if __name__ == '__main__':
    sys.exit(main())
