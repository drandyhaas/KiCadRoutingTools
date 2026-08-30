"""#553: the loop may carry the blocker CELL COUNTS, and by default does not.

`metrics_from_summary` reduces `summary['blockers']` to a set of net NAMES
(`{b['net'] for e in jb for b in e.get('blocked_by', [])}`) and throws every
count away. #553 wants those counts to rank movers, and the honest channel is
this function -- not the on-disk `loop_round{N}_route.json`, which `run_route`
writes only when the operator did not put their own `--json-out` in
`--route-args`, so on a legal invocation it does not exist.

What this file stands against, in order of how quietly each would ship:

  1. The default form growing a key. Its dict is serialised verbatim into
     `loop_round{N}.json`, so an unconditional addition changes the bytes of
     every run that never asked for #553. Pinned as an EXACT key set, both arms.
  2. `blockers` itself moving. The new key must be additive; if the name
     reduction changed, every existing consumer of the loop changed with it.
  3. None and [] conflated. `route.py` writes the key only
     `if blockers_report:`, so ABSENT covers both a pre-#409 log and a modern
     run that attributed nothing -- and it is exactly then that `blockers`
     comes from the whole-log regex, which scrapes TRANSIENT lines for nets
     that later routed. `None` is the flag that those names are not evidence.
     `[]` is the narrower reconcile-cleared-everything case and stays
     distinguishable.
  4. A fabricated count. route.py's `stage='preexisting'` entries are
     `{'net': n, 'preexisting': True}` and carry NO counts. Defaulting them to
     1, or to 0 without saying so, invents the evidence #553 ranks on. This
     function derives no count at all -- `diagnosis.blocker_evidence` is the
     single authority, and a second fold here is the drift #431 split this
     function out to prevent.
  5. A report that aliases the caller's summary. Value equality cannot see
     that, so the copy is asserted by IDENTITY and by writing through.
  6. Normalisation dressed as a copy: an entry with no `blocked_by` must not
     grow an empty one, and `blocked_count: 0` must stay a count.

The second caller is `render_placement._load_summary`, which calls
`metrics_from_summary(summary, txt)` POSITIONALLY -- so the new parameter is
keyword-only, and that call shape is asserted here rather than assumed.
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # placement split

import place_route_loop as L  # noqa: E402

# The seven keys the loop, the sidecar and the renderer have always seen.
BASE_KEYS = [
    'blockers', 'failed_nets', 'failures', 'iterations',
    'pad_pairs_connected', 'pad_pairs_total', 'vias',
]
EXTRA_KEYS = ['blocker_report']

# route.py's real shape: a per-failed-net entry whose `blocked_by` list is
# capped at 10 (FRONTIER_BLOCKING_RECORD_CAP) with `more` counting the tail.
STRUCTURED = {
    'failed_single': ['/A'],
    'total_iterations': 12, 'total_vias': 3,
    'blockers': [
        {'net': '/A', 'stage': 'phase3', 'more': 4, 'blocked_by': [
            {'net': 'GND', 'blocked_count': 40, 'unique_cells': 8,
             'track_cells': 30, 'via_cells': 10},
            {'net': '+3V3', 'blocked_count': 7, 'unique_cells': 7,
             'track_cells': 7, 'via_cells': 0},
        ]},
    ],
}

# The variant route.py emits for a boxed-in net: a name and nothing else.
PREEXISTING = {
    'failed_single': ['/A'],
    'blockers': [
        {'net': '/A', 'stage': 'preexisting', 'blocked_by': [
            {'net': '/VBUS', 'preexisting': True},
            {'net': 'GND', 'preexisting': True},
        ]},
    ],
}

# Pre-#409: no key at all, so the whole-log regex is the only source.
LEGACY_LOG = "  1. /MD1: 46 (31.7%) foo\n  2. /MD2: 12 (8.0%) bar\n"

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


def t_default_form_is_exactly_seven_keys():
    for name, summary, log in (('structured', STRUCTURED, ''),
                               ('preexisting', PREEXISTING, ''),
                               ('legacy', {'failed_single': []}, LEGACY_LOG)):
        got = sorted(L.metrics_from_summary(summary, log))
        check(got == BASE_KEYS,
              f'{name}: the default dict is the seven-key form ({got})')


def t_the_flag_adds_exactly_one_key():
    for name, summary, log in (('structured', STRUCTURED, ''),
                               ('preexisting', PREEXISTING, ''),
                               ('legacy', {'failed_single': []}, LEGACY_LOG)):
        got = sorted(L.metrics_from_summary(summary, log,
                                            keep_blocker_cells=True))
        check(got == sorted(BASE_KEYS + EXTRA_KEYS),
              f'{name}: keep_blocker_cells adds exactly {EXTRA_KEYS}')


def t_blockers_itself_never_moves():
    """The additive claim, checked on every input shape rather than argued."""
    for name, summary, log in (('structured', STRUCTURED, ''),
                               ('preexisting', PREEXISTING, ''),
                               ('empty-list', dict(STRUCTURED, blockers=[]),
                                LEGACY_LOG),
                               ('legacy', {'failed_single': []}, LEGACY_LOG)):
        off = L.metrics_from_summary(summary, log)
        on = L.metrics_from_summary(summary, log, keep_blocker_cells=True)
        check(off['blockers'] == on['blockers'],
              f'{name}: blockers is identical with the flag on and off')
        check(all(off[k] == on[k] for k in BASE_KEYS),
              f'{name}: every pre-existing key is identical')


def t_empty_list_is_not_none():
    """[] = reconciliation cleared every failure; None = the key was absent."""
    on = L.metrics_from_summary(dict(STRUCTURED, blockers=[]), LEGACY_LOG,
                                keep_blocker_cells=True)
    check(on['blockers'] == [],
          'an explicit [] still does not regress to the log regex')
    check(on['blocker_report'] == [],
          'an explicit [] is reported as [], not None')
    check(on['blocker_report'] is not None,
          'an explicit [] is not conflated with the pre-#409 path')


def t_the_regex_path_reports_no_structured_evidence():
    on = L.metrics_from_summary({'failed_single': []}, LEGACY_LOG,
                                keep_blocker_cells=True)
    check(on['blockers'] == ['/MD1', '/MD2'], 'the regex fallback still fires')
    check(on['blocker_report'] is None,
          'the pre-#409 path reports None, never [] -- there is no report')
    check('blockers_without_counts' not in on,
          'and no COUNT is derived here: blocker_evidence owns that, and two '
          'implementations of "how many blockers carry no counts" is the '
          'drift #431 split this function out to prevent')


def t_preexisting_entries_are_carried_never_imputed():
    on = L.metrics_from_summary(PREEXISTING, keep_blocker_cells=True)
    check(on['blockers'] == ['GND', '/VBUS'] or
          on['blockers'] == sorted({'GND', '/VBUS'}),
          'the names still reach the loop')
    report = on['blocker_report']
    check(report == PREEXISTING['blockers'],
          'the report is carried verbatim, not normalised')
    check(all('blocked_count' not in b
              for e in report for b in e['blocked_by']),
          'no count was invented for an entry that carried none')


def t_structured_counts_survive_verbatim():
    on = L.metrics_from_summary(STRUCTURED, keep_blocker_cells=True)
    check(on['blocker_report'] == STRUCTURED['blockers'],
          'the raw report is carried through untouched')
    entry = on['blocker_report'][0]
    check(entry.get('more') == 4,
          "the truncation tail ('more') survives, so a share can disclose it")
    check([b['blocked_count'] for b in entry['blocked_by']] == [40, 7],
          'the per-net cell counts are the ones route.py emitted')


def t_a_mixed_report_survives_intact():
    mixed = {'failed_single': ['/A', '/B'], 'blockers': [
        STRUCTURED['blockers'][0], PREEXISTING['blockers'][0]]}
    on = L.metrics_from_summary(mixed, keep_blocker_cells=True)
    check(on['blocker_report'] == mixed['blockers'],
          'a mixed counted/count-less report is carried whole')
    check(on['blockers'] == sorted({'GND', '+3V3', '/VBUS'}),
          'the name reduction is unchanged by the mix')


def t_the_report_is_a_copy_the_caller_cannot_be_written_through():
    """A ranking consumer that sorts or annotates must not reach the summary."""
    s = {'failed_single': ['/A'], 'blockers': [
        {'net': '/A', 'blocked_by': [{'net': 'GND', 'blocked_count': 5}]}]}
    on = L.metrics_from_summary(s, keep_blocker_cells=True)
    rep = on['blocker_report']
    check(rep is not s['blockers'], 'the list is not the caller list')
    check(rep[0] is not s['blockers'][0], 'nor is the entry')
    check(rep[0]['blocked_by'][0] is not s['blockers'][0]['blocked_by'][0],
          'nor the attribution -- the copy goes as deep as the structure')
    rep[0]['rank'] = 1
    rep[0]['blocked_by'][0]['blocked_count'] = 999
    check('rank' not in s['blockers'][0]
          and s['blockers'][0]['blocked_by'][0]['blocked_count'] == 5,
          'annotating the report leaves the summary the loop still holds alone')


def t_an_entry_without_blocked_by_does_not_grow_one():
    """Copying must not normalise: "attributed nothing" and "did not say" are
    different facts, and an invented empty list reads as the first."""
    s = {'failed_single': ['/A'], 'blockers': [{'net': '/A', 'stage': 'x'}]}
    on = L.metrics_from_summary(s, keep_blocker_cells=True)
    check(on['blocker_report'] == [{'net': '/A', 'stage': 'x'}],
          'the entry is unchanged')
    check('blocked_by' not in on['blocker_report'][0],
          'no empty blocked_by is invented')


def t_a_zero_count_is_a_count():
    """`blocked_count: 0` is the router saying zero, not the router saying
    nothing -- a falsy test here would file it with the preexisting entries."""
    s = {'failed_single': ['/A'], 'blockers': [
        {'net': '/A', 'blocked_by': [{'net': 'GND', 'blocked_count': 0}]}]}
    rep = L.metrics_from_summary(s, keep_blocker_cells=True)['blocker_report']
    check(rep[0]['blocked_by'][0]['blocked_count'] == 0,
          'the zero survives the copy as a zero')
    check('blocked_count' in rep[0]['blocked_by'][0],
          'and the key is still present, so a consumer can tell it apart '
          'from an entry that carried no count at all')


def t_the_second_caller_shape_still_works():
    """render_placement._load_summary calls this with TWO POSITIONAL args."""
    m = L.metrics_from_summary(STRUCTURED, '')
    check(m['failures'] == 1, 'the positional two-arg call still returns metrics')
    try:
        L.metrics_from_summary(STRUCTURED, '', None, True)
    except TypeError:
        check(True, 'keep_blocker_cells is keyword-only, so no caller can '
                    'hit it by adding a positional argument')
    else:
        check(False, 'keep_blocker_cells must be keyword-only')


def t_run_route_forwards_it_and_stays_a_one_line_caller():
    import inspect
    src = inspect.getsource(L.run_route)
    check('return metrics_from_summary(summary, log' in src,
          'run_route still delegates the arithmetic (the #431 pin)')
    check('keep_blocker_cells=keep_blocker_cells' in src,
          'run_route forwards the flag rather than re-deriving the counts')
    check('keep_blocker_cells' in
          str(inspect.signature(L.run_route).parameters),
          'run_route accepts the flag')
    kind = inspect.signature(L.run_route).parameters['keep_blocker_cells'].kind
    check(kind is inspect.Parameter.KEYWORD_ONLY,
          'run_route takes it keyword-only too')


TESTS = [
    t_default_form_is_exactly_seven_keys,
    t_the_flag_adds_exactly_one_key,
    t_blockers_itself_never_moves,
    t_empty_list_is_not_none,
    t_the_regex_path_reports_no_structured_evidence,
    t_preexisting_entries_are_carried_never_imputed,
    t_structured_counts_survive_verbatim,
    t_a_mixed_report_survives_intact,
    t_the_report_is_a_copy_the_caller_cannot_be_written_through,
    t_an_entry_without_blocked_by_does_not_grow_one,
    t_a_zero_count_is_a_count,
    t_the_second_caller_shape_still_works,
    t_run_route_forwards_it_and_stays_a_one_line_caller,
]


def main():
    for fn in TESTS:
        print(f'{fn.__name__}:')
        try:
            fn()
        except Exception as e:                       # noqa: BLE001
            check(False, f'{fn.__name__} raised {type(e).__name__}: {e}')
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_553_metrics_blockers: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
