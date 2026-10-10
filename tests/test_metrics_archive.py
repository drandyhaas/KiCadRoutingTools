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
7. The downloads timeline does not slope with the size of the catalogue. It
   used to spread every release from publish to TODAY, so each new release
   added a layer to every later day and flat interest drew a rising line. A
   flat rate must draw a flat line, and a real rise must still show.
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
          d == [{'date': '2026-09-15', 'pcm': 8, 'zip': 0, 'bin': 4}], f"{d}")


def _render_into(tmp, meta, extra=None):
    """Render with the module's paths redirected at a scratch dir.

    `extra` = {archive file name: content}, saved after the defaults.
    """
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
        for name, obj in (extra or {}).items():
            M._save(name, obj)
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

    # The downloads chart names where estimate ends and measurement begins,
    # and says so when the PCM listing history -- which shapes the whole PCM
    # estimate -- was never read. The control: with it read, no warning.
    # v0 is never listed and bursts; v1 is the listed one.
    two = {'2026-09-15': {'v1': _rel('2026-09-01', 7, 2), 'v0': _rel('2026-08-01', 5)},
           '2026-09-16': {'v1': _rel('2026-09-01', 9, 3), 'v0': _rel('2026-08-01', 50)}}
    listing = {'aaa': {'version': '1', 'listed': '2026-09-01'}}
    with tempfile.TemporaryDirectory() as tmp:
        unread = _render_into(tmp, {'last_collected': 'x', 'errors': {}},
                              {'releases.json': two})
        read = _render_into(tmp, {'last_collected': 'x', 'errors': {}},
                            {'releases.json': two, 'pcm_listings.json': listing})
    check('t_page_marks_where_measurement_begins',
          'measured from 2026-09-15' in read and 'class="mark"' in read
          and 'measured from' not in flat,
          'marked with two snapshots, absent with one')
    check('t_page_says_when_the_pcm_listing_is_unknown',
          'listing history could not be read' in unread
          and 'listing history could not be read' not in read)
    check('t_page_card_reads_the_listing_history',
          'now serving v1 (+2 since 2026-09-15)' in read
          and 'now serving v0' in unread,
          'the listed release with the history, the burst without it')
    card = '<div class="l">PCM installs</div><div class="n">{}</div>'
    check('t_page_card_total_counts_only_listed_zips',
          card.format(9) in read and card.format(59) in unread,
          "9 (v1's zips) with the history, all 59 zips without it")
    check('t_page_deltas_table_splits_the_zips',
          '<th>2026-09-16</th><td class="num">2</td><td class="num">45</td>' in read,
          'v1 +2 as PCM, the unlisted v0 +45 as direct')
    # The chart reads the recorded snapshot times: a steady 1/hour with the
    # middle snapshot taken late (18:00) peaks at 24 a day, where booking
    # whole intervals to a day would read 30.
    hourly = {'2026-09-15': {'v1': _rel('2026-09-15', 0)},
              '2026-09-16': {'v1': _rel('2026-09-15', 30)},
              '2026-09-17': {'v1': _rel('2026-09-15', 48)}}
    at = {'2026-09-15': '2026-09-15T12:00:00Z', '2026-09-16': '2026-09-16T18:00:00Z',
          '2026-09-17': '2026-09-17T12:00:00Z'}
    with tempfile.TemporaryDirectory() as tmp:
        timed = _render_into(tmp, {'last_collected': 'x', 'errors': {}},
                             {'releases.json': hourly, 'pcm_listings.json': listing,
                              'release_times.json': at})
    check('t_page_chart_reads_the_snapshot_times',
          'style="top:0%">24<' in timed and 'style="top:0%">30<' not in timed,
          'the downloads chart tops out at 24/day')
    check('t_page_draws_direct_zips_apart_only_when_it_can',
          'direct zip downloads/day' in read and 'stroke-dasharray="6 4"' in read
          and 'direct zip downloads/day' not in unread,
          'a dashed third line with the history, none without')


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


def _rel(published, pcm=0, binaries=0):
    """One release entry of a snapshot, as the collector writes it."""
    return {'published_at': f'{published}T12:00:00Z',
            'assets': {'KiCadRoutingTools-x.zip': pcm,
                       'grid_router-linux-x86_64.so': binaries}}


def _day(start, n):
    from datetime import date, timedelta
    return (date(*map(int, start.split('-'))) + timedelta(days=n)).isoformat()


def t_flat_interest_draws_a_flat_line():
    """Claim 7: the catalogue growing must not read as interest growing.

    Ten releases, one every five days, each gathering exactly 100 PCM installs
    and 20 binary downloads a day while it is the newest, then nothing. The
    old publish-to-today spread drew this as a ramp, from ~10 a day on the
    first day to ~290 on the last.
    """
    start = '2026-06-01'
    snap = {f'v{i}': _rel(_day(start, 5 * i), pcm=500, binaries=100)
            for i in range(10)}
    listed = {f'v{i}': _day(start, 5 * i) for i in range(10)}
    sp, measured = M.reign_downloads({_day(start, 50): snap}, listed)
    pcm = [sp[d]['pcm'] for d in sorted(sp)]
    binr = [sp[d]['bin'] for d in sorted(sp)]
    check('t_flat_interest_draws_a_flat_line',
          len(pcm) == 50 and max(pcm) - min(pcm) < 1e-9 and abs(pcm[0] - 100) < 1e-9
          and max(binr) - min(binr) < 1e-9 and abs(binr[0] - 20) < 1e-9,
          f"{len(pcm)} days, pcm {min(pcm):.2f}..{max(pcm):.2f}, "
          f"bin {min(binr):.2f}..{max(binr):.2f}")
    check('t_one_snapshot_is_all_estimate', measured is None, f"{measured}")

    # The control: a flat line must come from the data, not be forced by the
    # method. Interest doubling halfway through has to show as a step.
    rising = {f'v{i}': _rel(_day(start, 5 * i), pcm=500 if i < 5 else 1000)
              for i in range(10)}
    sp2, _ = M.reign_downloads({_day(start, 50): rising}, listed)
    early, late = sp2[_day(start, 10)]['pcm'], sp2[_day(start, 40)]['pcm']
    check('t_a_real_rise_still_shows', abs(early - 100) < 1e-9 and abs(late - 200) < 1e-9,
          f"day 10: {early:.1f}/day, day 40: {late:.1f}/day")


def t_pcm_reigns_follow_the_listing_not_the_release():
    """PCM serves only its newest LISTED version, skipping everything between.

    v1 is listed, v2 is published but never listed, v3 is listed ten days
    after v2. v1's zip installs belong to the whole span until v3 took over --
    booking them only until v2 was PUBLISHED would draw a spike, then a hole.
    Binaries follow GitHub's order: v2 does take over from v1 there.
    """
    snap = {'v1': _rel('2026-07-01', pcm=2000, binaries=100),
            'v2': _rel('2026-07-11', pcm=0, binaries=100),
            'v3': _rel('2026-07-21', pcm=1000, binaries=100)}
    listed = {'v1': '2026-07-01', 'v3': '2026-07-21'}
    sp, _ = M.reign_downloads({'2026-07-31': snap}, listed)
    check('t_pcm_reigns_follow_the_listing_not_the_release',
          abs(sp['2026-07-05']['pcm'] - 100) < 1e-9
          and abs(sp['2026-07-15']['pcm'] - 100) < 1e-9,
          f"v1 before and after the unlisted v2: {sp['2026-07-05']['pcm']:.1f}, "
          f"{sp['2026-07-15']['pcm']:.1f}/day")
    check('t_binaries_follow_the_release_order',
          abs(sp['2026-07-05']['bin'] - 10) < 1e-9
          and abs(sp['2026-07-15']['bin'] - 10) < 1e-9,
          f"{sp['2026-07-05']['bin']:.1f} and {sp['2026-07-15']['bin']:.1f}/day")

    # Without a listing history every release takes over in turn -- the
    # fallback the page names -- and v1's installs crowd into ten days.
    bare, _ = M.reign_downloads({'2026-07-31': snap}, {})
    check('t_without_listings_pcm_falls_back_to_release_order',
          abs(bare['2026-07-05']['pcm'] - 200) < 1e-9
          and bare['2026-07-15']['pcm'] < 1e-9,
          f"{bare['2026-07-05']['pcm']:.1f}/day in v1's ten days, then "
          f"{bare['2026-07-15']['pcm']:.1f}")


def t_measured_days_are_the_snapshot_differences():
    """After the first snapshot nothing is estimated: a day IS its delta."""
    def snaps(*counts):
        return {_day('2026-09-15', i): {'v1': _rel('2026-09-01', pcm=c, binaries=c // 10)}
                for i, c in enumerate(counts)}
    # Day 3 is a SHORT read (390 < 400): the running max must not turn it into
    # a negative day followed by a double one.
    sp, measured = M.reign_downloads(snaps(140, 290, 400, 390, 520), {'v1': '2026-09-01'})
    got = [round(sp.get(_day('2026-09-15', i), {}).get('pcm', 0), 9) for i in range(4)]
    check('t_measured_days_are_the_snapshot_differences',
          measured == '2026-09-15' and got == [150, 110, 0, 120],
          f"measured from {measured}: {got}")
    # The lifetime count at the first snapshot stays BEFORE it, in the reign.
    pre = sum(v['pcm'] for d, v in sp.items() if d < '2026-09-15')
    check('t_the_pre_archive_total_stays_before_the_archive',
          abs(pre - 140) < 1e-9 and min(sp) == '2026-09-01',
          f"{pre:.1f} booked from {min(sp)}")
    check('t_no_day_is_negative', min(v['pcm'] for v in sp.values()) >= 0)
    # The chart places points by INDEX, so a missing day would silently
    # squeeze the time axis; a day with no downloads must be present as 0.
    span = sorted(sp)
    check('t_every_day_is_present', len(span) == 18
          and all(_day(span[0], i) == d for i, d in enumerate(span)),
          f"{len(span)} days {span[0]}..{span[-1]}")


def t_the_timeline_conserves_every_download():
    """Reshaping in time must not invent or lose a download.

    It must sum to exactly what the per-release table says, including a short
    read, a release first seen after the archive began, and a same-day pair.
    """
    rel = {'2026-09-15': {'a': _rel('2026-08-01', 900, 70),
                          'b': _rel('2026-08-01', 40, 9),
                          'c': _rel('2026-09-15', 3, 1)},
           '2026-09-16': {'a': _rel('2026-08-01', 950, 71),
                          'c': _rel('2026-09-15', 30, 4)},
           '2026-09-17': {'a': _rel('2026-08-01', 940, 75),
                          'b': _rel('2026-08-01', 41, 9),
                          'c': _rel('2026-09-15', 80, 12),
                          'd': _rel('2026-09-16', 5, 2)}}
    sp, _ = M.reign_downloads(rel, {'a': '2026-08-02', 'c': '2026-09-16'},
                              {'2026-09-16': '2026-09-16T19:04:00Z',
                               '2026-09-17': '2026-09-17T11:20:00Z'}, partial=True)
    rows, plat, pcm = M._release_rollup(rel)
    # Every zip is in exactly one of the two zip series.
    tp = sum(v['pcm'] + v['zip'] for v in sp.values())
    tb = sum(v['bin'] for v in sp.values())
    check('t_the_timeline_conserves_every_download',
          abs(tp - sum(pcm.values())) < 1e-6 and abs(tb - sum(plat.values())) < 1e-6,
          f"pcm {tp:.4f} == {sum(pcm.values())}, bin {tb:.4f} == {sum(plat.values())}")
    # A release with no publish date cannot be placed in time and is dropped
    # rather than silently dated.
    sp2, _ = M.reign_downloads({'2026-09-15': {'x': {'published_at': '',
                                                     'assets': {'KiCadRoutingTools-x.zip': 999}}}})
    check('t_an_undated_release_is_dropped_not_guessed', sp2 == {}, f"{sp2}")


def t_an_interval_is_spread_over_the_hours_it_covers():
    """A late snapshot must not pile a long interval onto one day.

    The real case, 2026-09-27: the scheduled run took its snapshot at 11:59,
    a manual run replaced it at 19:04, and the interval since the previous
    snapshot (09-26 11:21) was 31.7 hours -- all booked to one day, so a steady
    rate read 30% high. Here a release gains exactly one download an hour
    throughout; every complete day must read 24.
    """
    rel = {'2026-09-25': {'v1': _rel('2026-09-25', pcm=12)},
           '2026-09-26': {'v1': _rel('2026-09-25', pcm=36)},
           '2026-09-27': {'v1': _rel('2026-09-25', pcm=67)}}
    times = {'2026-09-25': '2026-09-25T12:00:00Z', '2026-09-26': '2026-09-26T12:00:00Z',
             '2026-09-27': '2026-09-27T19:00:00+00:00'}
    sp, _ = M.reign_downloads(rel, {'v1': '2026-09-25'}, times)
    got = {d: round(v['pcm'], 9) for d, v in sp.items()}
    check('t_an_interval_is_spread_over_the_hours_it_covers',
          got == {'2026-09-25': 24, '2026-09-26': 24}, f"{got}")
    # The day the last snapshot falls in is still accumulating: 19 hours of
    # it drawn as a day would read as a 20% collapse. With `partial` it is
    # there, and it is exactly those 19 hours.
    full, _ = M.reign_downloads(rel, {'v1': '2026-09-25'}, times, partial=True)
    check('t_the_day_still_accumulating_is_left_out',
          '2026-09-27' not in sp
          and abs(full.get('2026-09-27', {}).get('pcm', 0) - 19) < 1e-9,
          f"withheld; with partial it holds "
          f"{full.get('2026-09-27', {}).get('pcm', 0):.1f}")
    # The key IS the collection date, so a time on another day is corrupt and
    # the snapshot counts from the start of its own day instead.
    bad = dict(times, **{'2026-09-26': '2026-09-24T12:00:00Z'})
    sp3, _ = M.reign_downloads(rel, {'v1': '2026-09-25'}, bad, partial=True)
    check('t_a_time_on_another_day_is_not_trusted',
          abs(sum(v['pcm'] for v in sp3.values()) - 67) < 1e-9
          and abs(sp3['2026-09-25']['pcm'] - 36) < 1e-9,
          f"09-25 books 12 + all 24 to 00:00 of 09-26: {sp3['2026-09-25']['pcm']:.1f}")


def t_unlisted_zips_are_direct_downloads_not_pcm():
    """PCM cannot install a version it does not list.

    The real case: v0.19.0 -- never listed, superseded for two months -- took
    415 zip downloads in a day while PCM served v0.22.1, and the PCM line
    counted every one of them as an install.
    """
    rel = {'2026-09-26': {'v0.19.0': _rel('2026-07-23', pcm=65),
                          'v0.22.1': _rel('2026-09-17', pcm=1481)},
           '2026-09-27': {'v0.19.0': _rel('2026-07-23', pcm=480),
                          'v0.22.1': _rel('2026-09-17', pcm=1627)}}
    listed = {'v0.22.1': '2026-09-17'}
    sp, _ = M.reign_downloads(rel, listed)
    day = sp['2026-09-26']
    check('t_unlisted_zips_are_direct_downloads_not_pcm',
          abs(day['pcm'] - 146) < 1e-9 and abs(day['zip'] - 415) < 1e-9,
          f"PCM {day['pcm']:.0f}, direct {day['zip']:.0f}")
    d = M._weekly_deltas(rel, listed)
    check('t_the_deltas_table_splits_the_zips_the_same_way',
          d == [{'date': '2026-09-27', 'pcm': 146, 'zip': 415, 'bin': 0}], f"{d}")
    # With no listing history nothing can be told apart: every zip stays PCM,
    # and the page says it cannot vouch for that line.
    bare, _ = M.reign_downloads(rel, {})
    check('t_without_listings_every_zip_is_pcm',
          abs(bare['2026-09-26']['pcm'] - 561) < 1e-9 and bare['2026-09-26']['zip'] == 0,
          f"{bare['2026-09-26']}")


def t_the_collector_records_when_it_snapshotted():
    """`release_times.json` gains today's collection time and keeps no orphans.

    Offline: both APIs are replaced by canned responders.
    """
    def fake_api(slug, path, token='', paginate=False):
        if path == 'releases':
            return [{'tag_name': 'v1', 'published_at': '2026-09-01T00:00:00Z',
                     'assets': [{'name': 'KiCadRoutingTools-1.zip', 'download_count': 3}]}], ''
        if path in ('traffic/views', 'traffic/clones'):
            return {path.split('/')[1]: []}, ''
        return [], ''
    real_api, real_gl, real_pc, data = M._api, M._gitlab, M._pcm_file_commits, M.DATA
    M._api, M._gitlab = fake_api, lambda path: (None, 'offline')
    M._pcm_file_commits = lambda rel: (None, 'offline')
    with tempfile.TemporaryDirectory() as tmp:
        M.DATA = tmp
        try:
            M._save('release_times.json', {'2020-01-01': '2020-01-01T12:00:00Z'})
            errors, _ = M.collect('owner/repo')
            times = M._load('release_times.json', {})
        finally:
            M._api, M._gitlab, M._pcm_file_commits, M.DATA = real_api, real_gl, real_pc, data
    stamp = M._today()
    check('t_the_collector_records_when_it_snapshotted',
          list(times) == [stamp] and times[stamp].startswith(stamp + 'T'),
          f"{times}; the orphan 2020-01-01 pruned")
    check('t_an_unreachable_gitlab_is_disclosed',
          any('gitlab' in k for k in errors), f"{sorted(errors)}")


def t_pcm_listings_read_the_file_and_the_merge():
    """The listing collector, offline: git and GitLab replaced by canned responders.

    The version is the newest one in the package FILE at each commit (an MR
    title can be stale -- upstream !587 is titled v0.15.5 and shipped v0.15.6),
    the date is the MR's merge in UTC, and a direct push falls back to the
    commit date converted to UTC.
    """
    ident = 'packages%2Fcom.github.drandyhaas.kicadroutingtools%2Fmetadata.json'
    commits = [{'id': 'bbb', 'title': 'Update to v9.9.9',
                'committed_date': '2026-07-01T08:17:14.000-04:00'},
               {'id': 'aaa', 'title': 'Add v0.15.5',
                'committed_date': '2026-05-26T13:58:11.000-04:00'},
               {'id': 'ccc', 'title': 'direct push',
                'committed_date': '2026-08-14T22:30:00.000-04:00'}]
    files = {'aaa': ['0.15.6'], 'bbb': ['0.15.6', '0.17.3'],
             'ccc': ['0.15.6', '0.17.3', '0.20.4']}
    merged = {'aaa': '2026-05-26T21:39:02.213Z', 'bbb': '2026-07-02T12:10:27.549Z'}
    calls = []

    def fake_commits(rel):
        calls.append(rel)
        assert rel == 'packages/com.github.drandyhaas.kicadroutingtools/metadata.json', rel
        return commits, ''

    def fake(path):
        calls.append(path)
        if path.startswith(f'repository/files/{ident}/raw?ref='):
            return {'versions': [{'version': v} for v in files[path.split('=')[-1]]]}, ''
        sha = path.split('/')[2]
        return ([{'merged_at': merged[sha]}] if sha in merged else []), ''
    real, real_pc = M._gitlab, M._pcm_file_commits
    M._gitlab, M._pcm_file_commits = fake, fake_commits
    try:
        store = {}
        err = M.collect_pcm_listings(store)
        got = M.pcm_listing_dates(store)
        check('t_pcm_listings_read_the_file_and_the_merge',
              err == '' and got == {'v0.15.6': '2026-05-26', 'v0.17.3': '2026-07-02',
                                    'v0.20.4': '2026-08-15'},
              f"err={err!r} {got}")
        calls.clear()
        again = M.collect_pcm_listings(store)
        check('t_a_banked_commit_is_never_fetched_again',
              again == '' and len(calls) == 1, f"{len(calls)} call(s) on the rerun")
    finally:
        M._gitlab, M._pcm_file_commits = real, real_pc


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


def t_the_pcm_card_names_the_climbing_release_not_the_biggest_pile():
    """When PCM is repointed, the headline must follow it, not the old total.

    This is the staleness the lifetime maximum cannot avoid: a repointed PCM
    leaves the OLD release holding the larger total for months while the NEW one
    is what people are actually installing.
    """
    def snap(old, new):
        return {'v0.20.4': {'published_at': '2026-08-14T00:00:00Z',
                            'assets': {'KiCadRoutingTools-0.20.4.zip': old}},
                'v0.23.0': {'published_at': '2026-09-20T00:00:00Z',
                            'assets': {'KiCadRoutingTools-0.23.0.zip': new}}}
    rel = {'2026-09-15': snap(4170, 10), '2026-09-16': snap(4172, 310)}
    acc = M.currently_accumulating(rel)
    _rows, _plat, pcm = M._release_rollup(rel)
    biggest = max(pcm.items(), key=lambda kv: kv[1])[0]
    check('t_the_pcm_card_names_the_climbing_release_not_the_biggest_pile',
          acc and acc[0] == 'v0.23.0' and acc[1] == 300 and biggest == 'v0.20.4',
          f"climbing {acc[0]} (+{acc[1]}) while the biggest pile is still {biggest}")
    # With one snapshot there is no delta, so it must answer "I don't know"
    # rather than guessing -- the caller falls back to the labelled maximum.
    check('t_one_snapshot_yields_no_claim',
          M.currently_accumulating({'2026-09-15': snap(4170, 10)}) is None)
    # A repoint that has not happened yet (nothing gained) is also no claim.
    check('t_no_movement_yields_no_claim',
          M.currently_accumulating({'a': snap(10, 1), 'b': snap(10, 1)}) is None)


def t_the_pcm_card_names_the_listed_release_not_a_burst():
    """With the listing history, the card states a fact rather than a guess.

    The real case from the archive, 2026-09-27: PCM was serving v0.22.1
    (+146 that day) while v0.19.0 -- listed nowhere -- took a burst of 264,
    and the climb-based card said "now serving v0.19.0".
    """
    def snap(burst, served):
        return {'v0.19.0': _rel('2026-07-23', pcm=36 + burst),
                'v0.20.4': _rel('2026-08-14', pcm=4528),
                'v0.22.1': _rel('2026-09-17', pcm=1481 + served)}
    rel = {'2026-09-26': snap(0, 0), '2026-09-27': snap(264, 146)}
    store = {'aaa': {'version': '0.20.4', 'listed': '2026-08-14'},
             'bbb': {'version': '0.22.1', 'listed': '2026-09-17'}}
    _rows, _plat, pcm = M._release_rollup(rel)
    listed = M.pcm_card_hint(rel, store, pcm)
    # Non-vacuity: without the history this exact data does fool the card.
    guessed = M.pcm_card_hint(rel, {}, pcm)
    check('t_the_pcm_card_names_the_listed_release_not_a_burst',
          listed == 'now serving v0.22.1 (+146 since 2026-09-26)'
          and guessed.startswith('now serving v0.19.0'),
          f"listed: {listed!r}; without the history: {guessed!r}")
    one = M.pcm_card_hint({'2026-09-27': rel['2026-09-27']}, store, pcm)
    check('t_one_snapshot_still_names_the_listed_release',
          one == 'now serving v0.22.1, listed 2026-09-17', repr(one))
    # A version withdrawn upstream stops being named: the NEWEST commit's file
    # is the catalogue, not the highest version any commit ever carried.
    store['ccc'] = {'version': '0.20.4', 'listed': '2026-09-20'}
    check('t_a_withdrawn_version_is_not_named',
          M.pcm_now_serving(store) == 'v0.20.4', M.pcm_now_serving(store))


def t_chart_labels_do_not_scale_with_the_page():
    """No <text> inside a chart SVG -- axis labels must be HTML.

    The plot is width:100% with preserveAspectRatio="none", which is right for
    the geometry and fatal for type: it scales the SVG's coordinate system, so
    embedded <text> grows on a wide window and shrinks to nothing on a narrow
    one, stretched horizontally either way. Labels therefore live in HTML
    beside the plot, and strokes carry vector-effect so a 2px line stays 2px.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        flat = _render_into(tmp, {'last_collected': 'x', 'errors': {}})
    # Non-vacuity: there must BE charts to have got this wrong.
    check('t_the_page_has_charts', flat.count('<svg') >= 2,
          f"{flat.count('<svg')} chart(s) rendered")
    check('t_chart_labels_do_not_scale_with_the_page',
          '<text' not in flat,
          'no <text> inside any chart SVG')
    check('t_axis_labels_are_html', 'class="yl"' in flat and 'class="xaxis"' in flat)
    check('t_strokes_do_not_stretch', 'non-scaling-stroke' in flat)


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
    t_flat_interest_draws_a_flat_line()
    t_pcm_reigns_follow_the_listing_not_the_release()
    t_measured_days_are_the_snapshot_differences()
    t_the_timeline_conserves_every_download()
    t_pcm_listings_read_the_file_and_the_merge()
    t_an_interval_is_spread_over_the_hours_it_covers()
    t_unlisted_zips_are_direct_downloads_not_pcm()
    t_the_collector_records_when_it_snapshotted()
    t_thinning_never_moves_a_lifetime_total()
    t_the_pcm_card_names_the_climbing_release_not_the_biggest_pile()
    t_the_pcm_card_names_the_listed_release_not_a_burst()
    t_chart_labels_do_not_scale_with_the_page()
    t_a_failed_endpoint_is_disclosed_not_hidden()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
