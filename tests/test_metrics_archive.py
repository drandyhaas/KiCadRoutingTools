#!/usr/bin/env python3
"""The reach archive's arithmetic invariants, and the page's disclosures.

OFFLINE BY CONSTRUCTION. Nothing here touches the network: the collector's API
layer is never called, only the pure merge/rollup functions and the renderer,
which read the committed archive. A test that needed GitHub would fail on every
machine without a token and be deleted within a month.

WHY THESE. Each is a claim the page makes that a future edit could quietly
invert while the page still renders and still looks plausible:

1. Merging keeps the MAX per date. A part-elapsed day observed by one run must
   not be frozen at its partial value by a later run seeing the same day, and a
   re-run inside the 14-day window must never REDUCE a banked day. Overwrite
   semantics would pass any "the page renders" check and silently lose counts.
2. PCM installs and router-binary downloads are never summed. They are
   different audiences (a PCM user may never touch git), and a single
   "downloads" headline is the obvious, wrong simplification.
3. The page states that uniques are not additive. The card sums daily uniques
   because that is all GitHub gives, and that sum is NOT a count of people --
   if the caveat goes, the number becomes a lie rather than a proxy.
4. A failed endpoint is DISCLOSED. A silently absent series looks exactly like
   a quiet week, which is the failure mode that makes monitoring worthless.
5. A PARTIAL week is marked and never differenced against. The newest week is
   always incomplete, so an unguarded week-over-week column reports a collapse
   every Monday and trains its reader to ignore the only trend line there is.
6. No clone row claims a human count. GitHub exposes no actor, so `uniques` is
   a proxy and the ratio is an automation index -- a later edit renaming either
   to `people` would turn an honest estimate into a false measurement.
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, ROOT)

import repo_metrics as M                                      # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def t_merge_keeps_the_max_per_date():
    """A later run seeing a lower count for a banked day must not reduce it."""
    store = {}
    M._merge_daily(store, 'views', [{'timestamp': '2026-09-01T00:00:00Z',
                                     'count': 281, 'uniques': 107}])
    # The same day re-observed LOWER (a partial re-read, or GitHub revising).
    M._merge_daily(store, 'views', [{'timestamp': '2026-09-01T00:00:00Z',
                                     'count': 12, 'uniques': 3}])
    kept = store['views']['2026-09-01']
    check('t_merge_keeps_the_max_per_date',
          kept == {'count': 281, 'uniques': 107},
          f"re-observed low, kept {kept}")

    # ...and a genuine increase IS taken (the negative control: a max that
    # never rises is just as broken, and would pass the assertion above).
    M._merge_daily(store, 'views', [{'timestamp': '2026-09-01T00:00:00Z',
                                     'count': 400, 'uniques': 150}])
    risen = store['views']['2026-09-01']
    check('t_merge_still_takes_a_real_increase',
          risen == {'count': 400, 'uniques': 150}, f"rose to {risen}")


def t_pcm_and_binaries_are_counted_apart():
    """The two populations must not collapse into one 'downloads' number."""
    snap = {'2026-09-15': {'v0.20.4': {
        'published_at': '2026-08-14T00:00:00Z',
        'assets': {'KiCadRoutingTools-0.20.4.zip': 4164,
                   'grid_router-linux-x86_64.so': 145,
                   'grid_router-windows-x86_64.pyd': 62}}}}
    rows, plat, pcm = M._release_rollup(snap)
    ok = (len(rows) == 1 and rows[0]['pcm'] == 4164
          and sum(plat.values()) == 207 and pcm == {'v0.20.4': 4164})
    check('t_pcm_and_binaries_are_counted_apart', ok,
          f"pcm={rows[0]['pcm']}, binaries={sum(plat.values())}")

    # The deltas differ them too, rather than differencing one blended total.
    two = {'2026-09-08': {'v1': {'published_at': '', 'assets': {
               'KiCadRoutingTools-1.zip': 10, 'grid_router-linux-x86_64.so': 5}}},
           '2026-09-15': {'v1': {'published_at': '', 'assets': {
               'KiCadRoutingTools-1.zip': 18, 'grid_router-linux-x86_64.so': 9}}}}
    d = M._weekly_deltas(two)
    check('t_deltas_separate_the_two_populations',
          d == [{'date': '2026-09-15', 'pcm': 8, 'bin': 4}], f"{d}")


def _render_into(tmp, meta):
    """Render with the module's paths redirected at a scratch dir."""
    data, site = M.DATA, M.SITE
    M.DATA = os.path.join(tmp, 'data')
    M.SITE = os.path.join(tmp, 'site')
    os.makedirs(M.DATA, exist_ok=True)
    try:
        M._save('traffic_daily.json', {'views': {'2026-09-01': {'count': 5, 'uniques': 2}},
                                       'clones': {'2026-09-01': {'count': 3, 'uniques': 1}}})
        M._save('releases.json', {'2026-09-15': {'v1': {
            'published_at': '2026-09-01T00:00:00Z',
            'assets': {'KiCadRoutingTools-1.zip': 7,
                       'grid_router-linux-x86_64.so': 2}}}})
        M._save('referrers.json', {'2026-09-15': [{'referrer': 'Google', 'count': 9}]})
        M._save('meta.json', meta)
        M.render('owner/repo')
        with open(os.path.join(M.SITE, 'metrics', 'index.html')) as f:
            return ' '.join(f.read().split())
    finally:
        M.DATA, M.SITE = data, site


def t_page_discloses_what_the_numbers_are_not():
    with tempfile.TemporaryDirectory() as tmp:
        flat = _render_into(tmp, {'last_collected': 'x', 'errors': {}})
    # Non-vacuity first: the page must actually have rendered its data.
    check('t_page_rendered_at_all', 'owner/repo' in flat and 'v1' in flat,
          f"{len(flat)} chars")
    check('t_page_says_uniques_are_not_additive',
          'not additive' in flat.lower() and 'unique-days' in flat)
    check('t_page_says_the_populations_are_not_summed',
          'never summed' in flat.lower())
    check('t_page_says_a_download_is_not_a_run',
          'a download is not a run' in flat.lower())
    check('t_page_names_its_own_ci_as_a_confound',
          'own ci' in flat.lower())
    # The clone question is the one most likely to be "simplified" into a
    # headcount by a later edit, because a headcount is what everyone wants.
    check('t_page_refuses_to_claim_a_human_clone_count',
          'no way to count human clones' in flat.lower()
          and 'closest proxy' in flat.lower())


def t_clone_character_is_a_ratio_not_a_headcount():
    """The automation index, and the release day it is keyed to."""
    traffic = {'clones': {'2026-09-03': {'count': 98, 'uniques': 45},
                          '2026-09-04': {'count': 528, 'uniques': 124}},
               'views': {'2026-09-04': {'count': 281, 'uniques': 81}}}
    releases = {'2026-09-15': {'v0.22.0': {
        'published_at': '2026-09-04T00:00:00Z', 'assets': {}}}}
    rows = M.clone_character(traffic, releases)
    by = {r['date']: r for r in rows}
    spike, quiet = by['2026-09-04'], by['2026-09-03']
    check('t_clone_ratio_separates_a_machine_day',
          round(spike['ratio'], 2) == 4.26 and round(quiet['ratio'], 2) == 2.18
          and spike['ratio'] > quiet['ratio'],
          f"spike {spike['ratio']:.2f} vs quiet {quiet['ratio']:.2f}")
    check('t_release_days_are_marked',
          spike['release'] is True and quiet['release'] is False)
    # No row may claim to be a count of people: the keys are what GitHub gave
    # plus a derived ratio, and nothing named `humans`/`manual`.
    check('t_no_row_invents_a_human_count',
          not ({'humans', 'manual', 'people'} & set(spike)),
          f"keys={sorted(spike)}")


def t_weekly_rollup_withholds_a_stub_comparison():
    """A partial week must be marked, and never differenced against."""
    def days(start_day, n, per):
        return {f'2026-09-{start_day + i:02d}': {'count': per, 'uniques': per // 2}
                for i in range(n)}
    # W37 = Mon 2026-09-07 .. Sun 2026-09-13 (complete, 7 days)
    # W38 = Mon 2026-09-14 .. (one day only, partial)
    traffic = {'clones': {**days(7, 7, 100), **days(14, 1, 100)}, 'views': {}}
    rows = {r['week']: r for r in M.weekly_rollup(traffic)}
    full, part = rows['2026-W37'], rows['2026-W38']
    check('t_partial_week_is_marked',
          full['partial'] is False and part['partial'] is True
          and full['days'] == 7 and part['days'] == 1,
          f"full={full['days']}/7, partial={part['days']}/7")
    check('t_no_wow_against_a_partial_week', part['wow'] is None,
          'the newest, partial week would otherwise read as a -600 collapse')

    # The control: two COMPLETE weeks DO get a comparison, or the rule above
    # is indistinguishable from "wow never works".
    traffic2 = {'clones': {**days(7, 7, 100), **days(14, 7, 120)}, 'views': {}}
    r2 = {r['week']: r for r in M.weekly_rollup(traffic2)}
    check('t_two_complete_weeks_do_compare',
          r2['2026-W38']['wow'] == 140,
          f"W38 840 vs W37 700 -> {r2['2026-W38']['wow']}")


def t_a_short_read_cannot_shrink_the_lifetime_total():
    """The rollup takes the max ACROSS snapshots, not the latest snapshot.

    A download counter only grows, so the largest value seen is the true one.
    Reading only the newest snapshot lets one short read cut the lifetime total
    and render it as a decline -- which is exactly what the first CI run would
    have banked, having fetched 30 of 39 releases un-paginated.
    """
    full = {'v1': {'published_at': '2026-01-01T00:00:00Z',
                   'assets': {'KiCadRoutingTools-1.zip': 4000}},
            'v0': {'published_at': '2025-12-01T00:00:00Z',
                   'assets': {'KiCadRoutingTools-0.zip': 2500}}}
    short = {'v1': {'published_at': '2026-01-01T00:00:00Z',
                    'assets': {'KiCadRoutingTools-1.zip': 4100}}}
    rows, _plat, pcm = M._release_rollup({'2026-09-08': full, '2026-09-15': short})
    check('t_a_short_read_cannot_drop_a_release',
          sorted(r['tag'] for r in rows) == ['v0', 'v1'],
          f"tags={sorted(r['tag'] for r in rows)}")
    check('t_a_short_read_cannot_reduce_a_total',
          sum(pcm.values()) == 6600,
          f"4100 (risen) + 2500 (kept) = {sum(pcm.values())}")


def t_the_spread_conserves_every_download():
    """Spreading may reshape the timeline but must not invent or lose totals."""
    from datetime import date
    rows = [{'tag': 'a', 'published': '2026-09-01', 'pcm': 100,
             'binaries': {'grid_router-linux-x86_64.so': 30}},
            {'tag': 'b', 'published': '2026-09-15', 'pcm': 7, 'binaries': {}}]
    sp = M.spread_downloads(rows, today=date(2026, 9, 15))
    tp = sum(v['pcm'] for v in sp.values())
    tb = sum(v['bin'] for v in sp.values())
    check('t_the_spread_conserves_every_download',
          abs(tp - 107) < 1e-6 and abs(tb - 30) < 1e-6,
          f"pcm {tp:.4f} == 107, bin {tb:.4f} == 30")
    # The older release spans 15 days, the same-day one exactly 1 -- so the
    # spread is a RATE, and a fresh release is not smeared into the past.
    check('t_a_release_never_predates_itself',
          min(sp) == '2026-09-01' and sp['2026-09-01']['pcm'] < sp['2026-09-15']['pcm'],
          f"starts {min(sp)}, and the day b lands is higher")
    # A release with no publish date cannot be placed in time and is dropped
    # rather than silently dated today.
    sp2 = M.spread_downloads([{'tag': 'x', 'published': '', 'pcm': 999,
                               'binaries': {}}], today=date(2026, 9, 15))
    check('t_an_undated_release_is_dropped_not_guessed', sp2 == {}, f"{sp2}")


def t_thinning_never_moves_a_lifetime_total():
    """Daily snapshots thin to weekly after 30 days, losing no download."""
    from datetime import date, timedelta
    today = date(2026, 12, 31)
    store = {}
    for i in range(365):
        d = today - timedelta(days=364 - i)
        store[d.isoformat()] = {'v1': {'published_at': '2026-01-01T00:00:00Z',
                                       'assets': {'KiCadRoutingTools-1.zip': 100 + i}}}
    before, lifetime_before = len(store), M._release_rollup(store)[2]
    dropped = M.thin_snapshots(store, today=today)
    after, lifetime_after = len(store), M._release_rollup(store)[2]
    check('t_thinning_never_moves_a_lifetime_total',
          lifetime_before == lifetime_after,
          f"{before} -> {after} snapshots, total {lifetime_after} unchanged")
    check('t_thinning_actually_thins', dropped > 200 and after < before // 3,
          f"dropped {dropped}, kept {after}")
    check('t_thinning_keeps_the_newest', today.isoformat() in store,
          'the snapshot the page renders from survives')
    # Recent days keep FULL resolution -- thinning must not blunt the window
    # anyone actually reads.
    recent = [s for s in store if (today - date(*map(int, s.split('-')))).days <= 30]
    check('t_the_last_30_days_keep_daily_resolution', len(recent) == 31,
          f"{len(recent)} of the last 31 days kept")


def t_a_failed_endpoint_is_disclosed_not_hidden():
    with tempfile.TemporaryDirectory() as tmp:
        clean = _render_into(tmp, {'last_collected': 'x', 'errors': {}})
        broken = _render_into(tmp, {'last_collected': 'x',
                                    'errors': {'traffic/views': 'HTTP 403'}})
    check('t_a_failed_endpoint_is_disclosed_not_hidden',
          'HTTP 403' in broken and 'traffic/views' in broken
          and 'Incomplete collection' in broken,
          'the failure and the endpoint are both named')
    # The control: a clean run must NOT print the warning, or the disclosure
    # is decoration that says nothing.
    check('t_a_clean_run_shows_no_warning',
          'Incomplete collection' not in clean)


def main():
    t_merge_keeps_the_max_per_date()
    t_pcm_and_binaries_are_counted_apart()
    t_page_discloses_what_the_numbers_are_not()
    t_clone_character_is_a_ratio_not_a_headcount()
    t_weekly_rollup_withholds_a_stub_comparison()
    t_a_short_read_cannot_shrink_the_lifetime_total()
    t_the_spread_conserves_every_download()
    t_thinning_never_moves_a_lifetime_total()
    t_a_failed_endpoint_is_disclosed_not_hidden()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
